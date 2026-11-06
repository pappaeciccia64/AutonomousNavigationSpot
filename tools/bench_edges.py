#!/usr/bin/env python3
"""
Bench of the edge refresh of the cruise (phase 'edges' of the planning cycle) on the REAL
scans of a mission, old computation against new (edge_evaluation.FAST_REFRESH), with the
check that they give the same graph.

The scans of the mission (MissionMap/<mission>/scans/*.npz) are replayed in time order as
tests/test_replay.py does (same graph, same global maps, attempt start = local graph
update), into TWO graphs: A with FAST_REFRESH off, B with it on. At every scan inside an
attempt both do what the cruise does in its 'edges' phase (refresh_local_edge_weights, then
the nodes inside obstacles out) and then everything that the next cycles read is compared,
to the bit: edges with their weights and in the same order, validity, the stored live
evaluations, the global slope cache, the veto statistics, the counters. At the end: time per
call of A and B and of every piece (median, 90th percentile), and the number of cycles
whose state differed (must be 0).

    python3 tools/bench_edges.py                                   # 04-11 16:52 (CORE)
    python3 tools/bench_edges.py --missions 04-11-2026_14-08-57 04-11-2026_16-10-25
    python3 tools/bench_edges.py --max-scans 60                   # only the first 60 scans

No robot needed; it also runs on the CORE I/O (tools/core_run.sh, venv).
"""
import argparse
import copy
import math
import os
import re
import sys
import time
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, 'tests')]

import test_replay as R                                        # noqa: E402
from spot_explorer import config as P                         # noqa: E402
from spot_explorer.mission import cells, mission_prm          # noqa: E402
from spot_explorer.perception import geometry, global_maps    # noqa: E402
from spot_explorer.planning import edge_evaluation, roadmap, safe_frontier   # noqa: E402

# test_replay turns the incremental refresh off for its reference: here as in the mission.
edge_evaluation.INCREMENTAL_REFRESH = True

RADIUS0 = edge_evaluation.REFRESH_RADIUS_M

# Pieces of refresh_local_edge_weights timed one by one (methods of the PRM).
PIECES = ['_refresh_nodes', '_refresh_pairs', '_sync_global_slope_cache', '_occupancy_blocked_many',
          '_prefill_global_slope_nodata', '_reusable_profiles', 'precompute_arc_slopes',
          '_refresh_edge', 'live_prune_valid_edges', 'flush_slope_log']
TIMES = {}          # (label, piece) -> [seconds per call]


def _timed(name, fn, label):
    def wrapper(*a, **k):
        t0 = time.perf_counter()
        try:
            return fn(*a, **k)
        finally:
            TIMES.setdefault((label[0], name), []).append(time.perf_counter() - t0)
    return wrapper


def _instrument(prm, label):
    """Per-instance wrappers on the pieces: label is a 1-element list (A or B) set per call."""
    for name in PIECES:
        setattr(prm, name, _timed(name, getattr(prm, name), label))


def state(prm):
    """Everything the next cycles read from the refresh, as comparable values."""
    return dict(
        edges={a: list(lst) for a, lst in prm.edges.items()},
        validity=dict(prm.edge_validity),
        live_eval={k: (e.xy_a, e.xy_b, e.cov, e.mid_d, e.prof, e.valid, e.weight)
                   for k, e in prm._live_eval.items()},
        gslope=dict(prm._gslope_cache),
        slope_info=dict(prm.edge_slope_info),
        counts=(prm.refresh_counts_last, prm.live_veto_last, prm.live_prune_last),
        stats=(prm.veto_count_longitudinal, prm.veto_count_lateral, prm.veto_count_occupancy,
               frozenset(prm.vetoed_arcs_longitudinal), frozenset(prm.vetoed_arcs_lateral),
               frozenset(prm.vetoed_arcs_occupancy), dict(prm.max_slope_seen), prm.slope_flip_count,
               dict(prm.slope_source_counts)),
        traversed=frozenset(prm.traversed_edges))


def first_difference(sa, sb):
    for k in sa:
        if sa[k] == sb[k]:
            continue
        if k == 'edges':
            for a in set(sa[k]) | set(sb[k]):
                la, lb = sa[k].get(a, []), sb[k].get(a, [])
                if la != lb:
                    same_set = sorted(la) == sorted(lb)
                    return f"edges of node {a}{' (same edges, different order)' if same_set else ''}: {la[:4]} vs {lb[:4]}"
        if isinstance(sa[k], dict):
            ks = [x for x in set(sa[k]) | set(sb[k]) if sa[k].get(x) != sb[k].get(x)]
            return f"{k}: {len(ks)} entries differ, e.g. {ks[0]}: {sa[k].get(ks[0])} vs {sb[k].get(ks[0])}"
        return f"{k}: {sa[k]} vs {sb[k]}"
    return None


def replay(name, max_scans=None, b_radius='same'):
    m = R.mission_setup(name)
    if m is None:
        print(f"{name}: no scans or log on this PC")
        return None
    with R._quiet():
        env = cells.EnvironmentMap(rows=m['rows'], cols=m['cols'], cell_size=m['cell'])
        env.set_origin(m['origin'][0], m['origin'][1], math.radians(m['yaw_deg']), *m['start_cell'])
        P.MISSION_LATTICE_SPACING_M = m['spacing']
        g = m['graph']
        P.MISSION_GRAPH_SAMPLING = g['sampling']
        P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = g['min_edge'], g['max_edge']
        if g['sampling'] == 'random':
            P.MISSION_RANDOM_DISTRIBUTION, P.MISSION_RANDOM_DENSITY = g['distribution'], g['density']
            P.MISSION_RANDOM_SEED = g['seed']
        prm, _s, mg = mission_prm.build_mission_prm(env, verbose=False)
        gm, tm = global_maps.GlobalGrid(), global_maps.GlobalTerrainGrid()
        tm.update_rects = []
        prm.set_global_terrain_map(tm)
    side = {'A': [prm, gm, tm], 'B': None}
    label = ['A']
    side['B'] = list(copy.deepcopy((prm, gm, tm)))
    side['B'][0].set_global_terrain_map(side['B'][2])
    for k in 'AB':
        _instrument(side[k][0], label)
    n_cmp = n_diff = 0
    first_diff = None
    prev_it, prev_node = None, None
    scans = m['scans'][:max_scans] if max_scans else m['scans']
    for f in scans:
        d = np.load(f)
        it = int(re.search(r'scan_it(\d+)_', os.path.basename(f)).group(1))
        a = R.scan_arrays(d)
        rx, ry, ryaw = a['robot']
        snap = NS(obstacle_dist=a['od_filt'], seen=a['seen'], origin_x=a['ox'], origin_y=a['oy'],
                  cell_size=a['cs'])
        for k in 'AB':
            p, gm_, tm_ = side[k]
            label[0] = k
            edge_evaluation.FAST_REFRESH = (k == 'B')
            edge_evaluation.REFRESH_RADIUS_M = (b_radius if k == 'B' and b_radius != 'same' else RADIUS0)
            with R._quiet():
                R.feed_scan(a, p, gm_, tm_)
                node = int(d['robot_node_id']) if int(d['robot_node_id']) in p.nodes else \
                    min(p.nodes, key=lambda n: math.hypot(p.nodes[n][0] - rx, p.nodes[n][1] - ry))
                p.set_robot_node(node, 0.0)
                if prev_node is not None and prev_node != node:
                    key = (min(prev_node, node), max(prev_node, node))
                    if key in p.edge_validity:
                        p.traversed_edges.add(key)
                if it != prev_it:
                    mission_prm.rebuild_prm_edges(p, gm_)
                else:
                    t0 = time.perf_counter()
                    p.refresh_local_edge_weights(global_map=gm_, edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
                    TIMES.setdefault((k, 'TOTAL refresh'), []).append(time.perf_counter() - t0)
                    safe_frontier.prune_prm_nodes_in_obstacles(p, snap)
        new_node = node
        if it == prev_it:
            n_cmp += 1
            diff = first_difference(state(side['A'][0]), state(side['B'][0]))
            if diff:
                n_diff += 1
                if first_diff is None:
                    first_diff = f"{os.path.basename(f)}: {diff}"
        prev_it, prev_node = it, new_node
    edge_evaluation.FAST_REFRESH = True
    edge_evaluation.REFRESH_RADIUS_M = RADIUS0
    return n_cmp, n_diff, first_diff, len(prm.nodes)


def report():
    pieces = ['TOTAL refresh'] + PIECES
    print(f"\n{'piece':32s} {'A old: median':>14s} {'p90':>7s} {'B new: median':>14s} {'p90':>7s}  (ms per call)")
    for name in pieces:
        row = []
        for k in 'AB':
            v = np.array(TIMES.get((k, name), [])) * 1000
            if name == '_refresh_edge':      # many calls per refresh: sum per refresh
                n = len(TIMES.get((k, 'TOTAL refresh'), [])) or 1
                row += [v.sum() / n, float('nan')]
            else:
                row += ([np.median(v), np.percentile(v, 90)] if v.size else [float('nan')] * 2)
        print(f"{name:32s} {row[0]:14.1f} {row[1]:7.1f} {row[2]:14.1f} {row[3]:7.1f}")


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--missions', nargs='+', default=['04-11-2026_16-52-30'])
    p.add_argument('--max-scans', type=int, default=None)
    p.add_argument('--b-radius', default='same',
                   help="refresh radius of the NEW side only, in m, or 'window' (all the scan window): "
                        "the states then differ on purpose, the times are what counts")
    a = p.parse_args()
    bad = 0
    for name in a.missions:
        t0 = time.perf_counter()
        br = a.b_radius if a.b_radius == 'same' else (None if a.b_radius == 'window' else float(a.b_radius))
        r = replay(name, a.max_scans, br)
        if r is None:
            continue
        n_cmp, n_diff, first, n_nodes = r
        bad += n_diff
        print(f"{name}: {n_nodes} nodes, {n_cmp} cruise cycles compared, {n_diff} with a different state"
              + (f" -- first: {first}" if first else "") + f" ({time.perf_counter() - t0:.0f} s)")
    report()
    print("\nSAME RESULT" if bad == 0 else f"\n{bad} CYCLES DIFFER")
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
