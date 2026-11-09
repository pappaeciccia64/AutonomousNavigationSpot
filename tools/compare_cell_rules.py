#!/usr/bin/env python3
"""
What the general cell rules (mission/cell_rules.py) would have chosen, at every attempt start
of a real mission, against the cell the mission really attempted with the sides tried.

The scans of the mission are replayed as tests/test_replay.py does (same graph, global maps,
local update at every attempt start, refresh at the other scans with the frontier's rule);
the visited and blocked cells at every attempt start come from the [VISITED] / [BLOCKED]
lines of the log before it. At every attempt start: the choice of the rules from the robot
position of the first scan of the attempt, the attempted cell, and for that cell whether
the rules see a way into it.

    python3 tools/compare_cell_rules.py --missions 04-11-2026_16-52-30 04-11-2026_16-10-25

Read only, no robot.
"""
import argparse
import collections
import contextlib
import io
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, 'tests'), HERE]

import test_replay as R                                          # noqa: E402
import bench_edges as B                                          # noqa: E402
from spot_explorer import config as P                           # noqa: E402
from spot_explorer.mission import cell_rules, cells, mission_prm  # noqa: E402
from spot_explorer.perception import geometry, global_maps       # noqa: E402
from spot_explorer.planning import edge_evaluation               # noqa: E402

edge_evaluation.INCREMENTAL_REFRESH = True       # as in the mission (test_replay turns it off)


def _t(hms):
    h, m, s = hms.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def cell_events(name):
    """[(time of day s, (r, c), +1 visited / -1 blocked)] and the attempt start times."""
    log = open(os.path.join(ROOT, 'MissionLogs', f'Mission_{name}', 'mission_log.txt'), errors='replace').read()
    ev = [(_t(m.group(1)), (int(m.group(3)), int(m.group(4))), 1 if m.group(2) == 'VISITED' else -1)
          for m in re.finditer(r'^(\d\d:\d\d:\d\d\.\d+) \[(VISITED|BLOCKED)\] Cell \((\d+),(\d+)\) marked', log, re.M)]
    fails = [(_t(m.group(1)), (int(m.group(2)), int(m.group(3))),
              None if m.group(4) is None else (int(m.group(4)), int(m.group(5))))
             for m in re.finditer(r'^(\d\d:\d\d:\d\d\.\d+) \[CELL RULES\] Cell \((\d+),(\d+)\) failed .*?'
                                  r'(?:entering from cell \((\d+), (\d+)\)|the robot in the cell)', log, re.M)]
    first = re.search(r'^(\d\d:\d\d:\d\d\.\d+) \[ATTEMPT\] Trying', log, re.M)
    return ev, fails, (_t(first.group(1)) if first else None)


def compare(name):
    m = R.mission_setup(name)
    ev, fails, first_attempt = cell_events(name)
    t0 = float(np.load(m['scans'][0])['time']) % 86400
    # The log is in the local time of the machine that ran the mission (laptop CET, CORE UTC):
    # whole hours between the first attempt in the log and the first scan.
    offset = 3600.0 * round((first_attempt - t0) / 3600.0) if first_attempt is not None else 0.0
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
        prm, sampler, mg = mission_prm.build_mission_prm(env, verbose=False)
        gm, tm = global_maps.GlobalGrid(), global_maps.GlobalTerrainGrid()
        tm.update_rects = []
        prm.set_global_terrain_map(tm)
    rules = cell_rules.CellRules(env, prm, env.generate_visit_path('auto', start_cell=env.start_cell))
    out, prev_it = [], None
    for f in m['scans']:
        d = np.load(f)
        it, tr, tc = (int(v) for v in re.search(r'scan_it(\d+)_cell(\d+)_(\d+)', f).groups())
        a = R.scan_arrays(d)
        rx, ry, _ = a['robot']
        tod = float(d['time']) % 86400 + offset
        with R._quiet():
            R.feed_scan(a, prm, gm, tm)
            node = int(d['robot_node_id']) if int(d['robot_node_id']) in prm.nodes else None
            if node is not None:
                prm.set_robot_node(node, 0.0)
            if it != prev_it:
                mission_prm.rebuild_prm_edges(prm, gm)
            else:
                prm.set_live_snapshot(B.snapshot_of(d))
                prm.refresh_local_edge_weights(global_map=gm, edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
        if it != prev_it:
            for r in range(env.rows):
                for c in range(env.cols):
                    env.map[r][c] = 0
            env.map[env.start_cell[0]][env.start_cell[1]] = 1
            for t, rc, v in ev:
                if t < tod:
                    env.map[rc[0]][rc[1]] = v
            # The failures of the general cell rules logged before this attempt (09-11 on).
            rules.failures = {}
            for t, rc, entry in fails:
                if t < tod:
                    rules.failures.setdefault(rc, []).append(entry)
            if env.map[tr][tc] == 0:
                with contextlib.redirect_stdout(io.StringIO()):
                    ch = rules.choose((rx, ry), gm, sampler)
                start = prm.anchor_node(rx, ry, gm, geometry.PRM_EDGE_SAFETY_MARGIN_M, connected_first=True)
                way = rules.way_into(start, (rx, ry), tr, tc, mission_prm.node_cells(prm, env))
                out.append(dict(it=it, attempted=(tr, tc), chosen=None if ch is None else (ch.row, ch.col),
                                near=None if ch is None else ch.near, way_to_attempted=way is not None))
        prev_it = it
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--missions', nargs='+', default=['04-11-2026_16-52-30', '04-11-2026_16-10-25'])
    a = p.parse_args()
    for name in a.missions:
        rows = compare(name)
        c = collections.Counter()
        print(f"\n{name}: {len(rows)} attempt starts on cells not yet done")
        for r in rows:
            same = r['chosen'] == r['attempted']
            c['same cell'] += same
            c['attempted cell with NO way for the rules'] += not r['way_to_attempted']
            c['rules: no candidate (mission over)'] += r['chosen'] is None
            print(f"  attempt {r['it']:3d}: attempted {r['attempted']}, rules {r['chosen']}"
                  + ("" if r['chosen'] is None else (" in cruise" if r['near'] else " with GraphNav"))
                  + ("" if r['way_to_attempted'] else "  <- no way into the attempted cell"))
        print("  " + "; ".join(f"{k}: {v}" for k, v in c.items()))


if __name__ == '__main__':
    main()
