"""
REPLAY test on the real scans saved during missions.

Replays the scans of one or more missions (MissionMap/<mission>/scans/*.npz, in time
order) through the real code: global maps, mission lattice, graph update, replanning,
gait choice. Used to check a change WITHOUT the robot, on field data, in a few minutes.

Two kinds of check:

  INVARIANTS -- must always hold, whatever the change. If one fails there is a bug, and
  the test exits with code 1.
    1. Local graph update IDENTICAL to the full build (edges and weights), at every cell
       start; and the local update REALLY USED at every cell start after the first
       (without this second check the first would pass even if the code always rebuilt
       everything). As in the mission, the replay records the edges travelled (the robot
       node changes between two scans) and the confirmed blocks.
    2. Slopes of the global terrain map equal to the live ones on the same edges (at least
       99.5%): the two sources must measure the same thing.
    3. Gait: every leg travelled where the robot tilted more than 18 degrees (measured
       pitch) must come out HARD.
    4. At every cell start there is a path from the robot to the goal (in the mission the
       cells were reached, so the graph must not have broken).
    5. Return to wp_0 with GraphNav (if the downloaded graph is in graph/<mission>/): with
       the shortcuts chosen by map_shortcuts.plan_graphnav_shortcuts the path is not longer
       than today's, and is within 1.5 times the straight line plus 2 m.

  REFERENCE -- the behaviour numbers (valid and discarded edges, paths, gaits) compared
  with replay_reference.json. A difference is NOT necessarily a bug: it means the change
  alters what the robot would do. The test lists it and exits with code 1; if the change
  is intended it is accepted with --update-reference, and the updated file goes into the
  commit together with the change.

Usage:
    python3 tests/test_replay.py                       # 2 reference missions, full check
    python3 tests/test_replay.py --all                 # all 10 replayable missions
    python3 tests/test_replay.py --fast                # invariant 1 only on 3 scattered cell starts
    python3 tests/test_replay.py --missions 13-10-2026_12-40-38
    python3 tests/test_replay.py --update-reference
    python3 tests/test_replay.py --all --graph random --distribution poisson --seed 7

GRAPH COMPARISON -- with --graph, --spacing (and --distribution, --density, --seed,
  --min-edge, --max-edge) the missions are replayed with a graph different from the one
  they had in the mission, on the same scans. The comparison with the reference makes no
  sense and is skipped; the invariants still hold. At the end a table per mission (paths
  found, length and number of edges of the paths, graph time) to put next to the one of
  the same command without --graph. The robot attaches to the nearest node, the goal is
  the nearest node with at least one valid edge, and the blocks confirmed in the mission
  (given as lattice edges) are not replayed.

The scans are in MissionMap/ and the logs in MissionLogs/, ignored by git. Since 26-10 the
validation missions (the 10 of ALL_MISSIONS, the three of 22-10 and the one of 26-10 15:11,
the first with the intensity and no_step layers) are in the repository, added by hand:
log, scans/, *_vis.npz, the two global maps and the full GraphNav map (with the snapshots).
A mission missing on this PC is skipped, with a warning.

Known limits of the replay (not of the mission code):
  - in the mission the graph is updated on the scan taken at the start of the attempt,
    which is not saved; here the first saved scan of the attempt is used;
  - the cells allowed for the path are those of the previous attempts (in the reference
    missions all cells were reached);
  - the free-space map and the stop nodes are not replayed.
"""
import argparse
import contextlib
import copy
import csv
import datetime
import glob
import io
import json
import math
import os
import re
import sys
import time
from multiprocessing import Pool
try:
    from zoneinfo import ZoneInfo          # Python 3.9+
except ImportError:                        # Python 3.8 (the mission laptop)
    ZoneInfo = None

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                 # project folder: mission data
sys.path.insert(0, ROOT)                     # the spot_explorer package

REFERENCE_FILE = os.path.join(HERE, 'replay_reference.json')
DEFAULT_MISSIONS = ['13-10-2026_12-14-30', '13-10-2026_12-40-38']
# --all: all missions on this PC with scans in the right format (since 12-10, when the
# lattice and the filtered frontier arrived). Recorded with old versions of the code:
# it does not matter, the test replays the SCANS through today's code.
ALL_MISSIONS = ['12-10-2026_14-10-32', '12-10-2026_14-23-28', '12-10-2026_14-47-06',
                '12-10-2026_17-16-06', '12-10-2026_17-32-39', '13-10-2026_11-22-40',
                '13-10-2026_11-56-15', '13-10-2026_12-14-30', '13-10-2026_12-27-12',
                '13-10-2026_12-40-38']
GLOBAL_LIVE_MIN_MATCH = 0.995      # invariant 2
HARD_PITCH_DEG = 18.0              # invariant 3
ARCS_PER_SCAN = 60                 # random edges per scan, invariant 2
FAST_FULL_CHECKS = 3               # --fast: comparisons with the full build
# Time zone of the segments.csv timestamps (that of the laptop in the mission). Without
# zoneinfo (Python 3.8) the machine's local time is used, right on the mission laptop.
MISSION_TZ = ZoneInfo('Europe/Rome') if ZoneInfo is not None else None


def _quiet():
    return contextlib.redirect_stdout(io.StringIO())


with _quiet():
    import matplotlib
    matplotlib.use('Agg')
    from spot_explorer.legacy import modern
    from spot_explorer.perception import geometry
    from spot_explorer.perception import global_maps
    from spot_explorer.perception import slopes
    from spot_explorer.perception import spot_grid
    from spot_explorer.planning import roadmap as prm_graph
    from spot_explorer.mission import cells as environmentMap
    from spot_explorer.planning import mission_graph
    from spot_explorer.graphnav import map_shortcuts
    from spot_explorer.robot import motion as movements
    from spot_explorer.planning import goal
    from spot_explorer import records
    from spot_explorer import config as P


# --------------------------------------------------------------------------------------
# Reading the mission
# --------------------------------------------------------------------------------------
def mission_setup(name):
    """Origin, heading, grid and graph (spacing, or random nodes and seed) from the log;
    scans in order."""
    log_path = os.path.join(ROOT, 'MissionLogs', f'Mission_{name}', 'mission_log.txt')
    scans = glob.glob(os.path.join(ROOT, 'MissionMap', f'Mission_{name}', 'scans', '*.npz'))
    if not os.path.exists(log_path) or not scans:
        return None
    log = open(log_path, errors='replace').read()
    ox, oy = map(float, re.search(r'(?:Origin set at world coordinates|Origine fissata alle coordinate) \(([-\d.]+), ([-\d.]+)\)', log).groups())
    yaw = float(re.search(r'(?:Initial orientation|Orientamento iniziale): ([-\d.]+)', log).group(1))
    g = re.search(r'Griglia di missione: (\d+) x (\d+) celle da ([\d.]+) m', log)
    rows, cols, cell = (int(g.group(1)), int(g.group(2)), float(g.group(3))) if g else (2, 4, 5.0)
    s = re.search(r'Reticolo: passo richiesto ([\d.]+) m', log)
    spacing = float(s.group(1)) if s else 0.25
    # Since 22-10: node method and edge length; for random nodes distribution,
    # density and actual seed. The earlier missions are all lattice, edges 0.5-1.25.
    gr = re.search(r'\[CONFIG\] Grafo: nodi (\w+).*; archi da ([\d.]+) a ([\d.]+) m', log)
    sampling, min_edge, max_edge = ((modern(gr.group(1)), float(gr.group(2)), float(gr.group(3)))
                                    if gr else ('lattice', 0.5, 1.25))
    rnd = re.search(r"\[GRAFO\] Nodi casuali: distribuzione (\w+), densita' ([\d.]+) nodi/m2, "
                    r"seme (\d+)", log)
    graph = dict(sampling=sampling, min_edge=min_edge, max_edge=max_edge,
                 distribution=modern(rnd.group(1)) if rnd else None,
                 density=float(rnd.group(2)) if rnd else None,
                 seed=int(rnd.group(3)) if rnd else None)
    scans.sort(key=lambda f: float(np.load(f)['time']))
    seg_path = os.path.join(ROOT, 'MissionLogs', f'Mission_{name}', 'segments.csv')
    return dict(origin=(ox, oy), yaw_deg=yaw, rows=rows, cols=cols, cell=cell, spacing=spacing,
                graph=graph,
                scans=scans, segments=seg_path if os.path.exists(seg_path) else None)


def scan_arrays(d):
    """The quantities of a saved scan, as the mission code uses them."""
    z = d['terrain_corrected']
    ny, nx = z.shape
    ox, oy = map(float, d['grid_origin'])
    cs = float(d['cell_size'])
    r, c = np.mgrid[0:ny, 0:nx]
    pts = np.stack([ox + (c + .5) * cs, oy + (r + .5) * cs, z], -1).reshape(-1, 3)
    seen = np.asarray(d['terrain_valid_raw']).reshape(z.shape) >= spot_grid.TERRAIN_VALID_MIN
    if 'unwritten' in d.files and np.size(d['unwritten']) == z.size:
        seen &= ~np.asarray(d['unwritten'], dtype=bool).reshape(z.shape)
    return dict(z=z, nx=nx, ny=ny, ox=ox, oy=oy, cs=cs, pts=pts,
                od_raw=np.asarray(d['obstacle_distance'], dtype=np.float64).reshape(z.shape),
                od_filt=np.asarray(d['obstacle_distance_filtered'], dtype=np.float64).reshape(z.shape),
                valid=np.asarray(d['is_valid'], dtype=bool).reshape(z.shape),
                seen=seen, obstacle_mask=np.asarray(d['obstacle_mask']).ravel(),
                rough=np.asarray(d['roughness']).reshape(z.shape),
                grad=np.asarray(d['gradient']).reshape(z.shape),
                robot=(float(d['robot_xyz'][0]), float(d['robot_xyz'][1]), float(d['robot_yaw'])))


def feed_scan(a, prm, gm, tm):
    """One scan into the global maps (with the mission's own function) and into the PRM."""
    rx, ry, ryaw = a['robot']
    global_maps.fold_scan_into_global_maps(gm, tm, a['pts'], a['z'].ravel(), a['obstacle_mask'], a['valid'].ravel(),
                                  a['od_raw'].ravel(), (a['ny'], a['nx']), (rx, ry, ryaw))
    usable = spot_grid.slope_usable_mask(a['valid'].ravel(), a['od_raw'].ravel(), (a['ny'], a['nx']))
    prm.update_local_grid_data(grad_values_2d=a['grad'], rough_values_2d=a['rough'],
                               terrain_values_2d=a['z'], valid_2d=usable,
                               grid_origin_x=a['ox'], grid_origin_y=a['oy'], cell_size=a['cs'])
    prm.set_live_obstacles(a['od_filt'], a['seen'], a['ox'], a['oy'], a['cs'], rx, ry, ryaw)


def edges_of(prm):
    return {(min(a, n), max(a, n), round(w, 12)) for a, lst in prm.edges.items() for n, w in lst}


# --------------------------------------------------------------------------------------
# Replay of a mission
# --------------------------------------------------------------------------------------
def replay_mission(job):
    name, fast, override = job
    m = mission_setup(name)
    if m is None:
        return name, None
    t_start = time.perf_counter()
    rng = np.random.default_rng(0)
    out = dict(cell_starts=[], invariants={}, notes=[], tempo_grafo_s=0.0)
    with _quiet():
        env = environmentMap.EnvironmentMap(rows=m['rows'], cols=m['cols'], cell_size=m['cell'])
        env.set_origin(m['origin'][0], m['origin'][1], math.radians(m['yaw_deg']))
        P.MISSION_LATTICE_SPACING_M = m['spacing']
        g = m['graph']
        P.MISSION_GRAPH_SAMPLING = g['sampling']
        P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = g['min_edge'], g['max_edge']
        if g['sampling'] == 'random':
            P.MISSION_RANDOM_DISTRIBUTION = g['distribution']
            P.MISSION_RANDOM_DENSITY = g['density']
            P.MISSION_RANDOM_SEED = g['seed']
        # Graph comparison: the graph requested on the command line instead of the mission's
        # one. The node ids saved in the scans are then no longer valid.
        foreign = bool(override)
        for k, v in (override or {}).items():
            setattr(P, k, v)
        prm, _sampler, mg = mission_graph.build_mission_prm(env, verbose=False)
        out['grafo'] = mg.describe()
        gm = global_maps.GlobalGrid()
        tm = global_maps.GlobalTerrainGrid()
        tm.update_rects = []
        prm.set_global_terrain_map(tm)
    # --fast: the comparison with the full graph is done on FAST_FULL_CHECKS cell starts
    # scattered along the mission, the last included. The first ones are not enough: they have
    # little history behind them, and a margin too narrow leaves them still identical (tried:
    # margin cut to a fifth, first 3 identical, 7 different out of 10 in the rest of the mission).
    attempts = sorted({int(re.search(r'scan_it(\d+)_', os.path.basename(f)).group(1)) for f in m['scans']})
    later = attempts[1:]
    fast_pick = {later[round(i * (len(later) - 1) / max(FAST_FULL_CHECKS - 1, 1))]
                 for i in range(FAST_FULL_CHECKS)} if later else set()
    node_cells = None
    prev_it, done_cells = None, []
    full_checks = full_same = 0
    local_used = local_expected = 0
    lattice_pairs = {(min(int(a), int(b)), max(int(a), int(b))) for a, b in mg.pairs}
    prev_node = None
    gl_total = gl_same = 0
    for f in m['scans']:
        d = np.load(f)
        it, tr, tc = (int(v) for v in re.search(r'scan_it(\d+)_cell(\d+)_(\d+)', f).groups())
        a = scan_arrays(d)
        rx, ry, _ = a['robot']
        with _quiet():
            feed_scan(a, prm, gm, tm)
            robot_node = int(d['robot_node_id']) if not foreign and int(d['robot_node_id']) in prm.nodes else \
                min(prm.nodes, key=lambda k: math.hypot(prm.nodes[k][0] - rx, prm.nodes[k][1] - ry))
            prm.set_robot_node(robot_node, 0.0)
            # As in the mission: edge travelled when the robot moves from a node to the neighbour
            # (step completed), edge blocked when the frontier confirms a block.
            if prev_node is not None and prev_node != robot_node:
                key = (min(prev_node, robot_node), max(prev_node, robot_node))
                if key in lattice_pairs:
                    prm.traversed_edges.add(key)
            prev_node = robot_node
            nxt = [int(n) for n in np.ravel(d['path_node_ids'])] if 'path_node_ids' in d.files else []
            if not foreign and str(d['decision_kind']) == 'blocked' and nxt and \
                    (min(robot_node, nxt[0]), max(robot_node, nxt[0])) in lattice_pairs:
                prm.mark_edge_invalid(robot_node, nxt[0])
            if it != prev_it:
                # --- start of an attempt: graph update as in the mission ----
                do_full = (not fast) or it in fast_pick
                twin = copy.deepcopy((prm, gm)) if do_full and prev_it is not None else None
                t_g = time.perf_counter()
                mission_graph.rebuild_prm_edges(prm, gm)
                out['tempo_grafo_s'] += time.perf_counter() - t_g
                if prev_it is not None:
                    local_expected += 1
                    local_used += bool(mg.last_update and mg.last_update[2])
                if twin is not None:
                    mission_graph.rebuild_prm_edges(twin[0], twin[1], incremental=False)
                    full_checks += 1
                    full_same += edges_of(prm) == edges_of(twin[0])
                # --- path towards the goal, with the mission's restrictions ------
                if node_cells is None:
                    node_cells = mission_graph.node_cells(prm, env)
                cells = set(done_cells) | {(tr, tc)} | {tuple(int(v) for v in env.get_cell_from_world(rx, ry))}
                prm.set_allowed_nodes([n for n, c in node_cells.items() if c in cells])
                gx, gy = map(float, d['goal_xy'])
                # With another graph the saved goal may not be a node: if the mission
                # had moved it from the centre (occupied), the nearest node may be against
                # the obstacle, without edges. In the mission the fallback would choose among that
                # graph's nodes, so here the nearest one with at least one valid edge is taken.
                cand = [k for k in prm.nodes if prm.edges.get(k)] if foreign else prm.nodes
                goal = min(cand, key=lambda k: math.hypot(prm.nodes[k][0] - gx, prm.nodes[k][1] - gy))
                path = prm.find_path_dijkstra(robot_node, goal)
                prm.set_allowed_nodes(None)
                ev = prm.edge_validity
                length = (sum(math.dist(prm.nodes[p], prm.nodes[q]) for p, q in zip(path[:-1], path[1:]))
                          if path else None)
                out['cell_starts'].append(dict(
                    attempt=it, cell=[tr, tc],
                    valid_arcs=sum(1 for v in ev.values() if v),
                    discarded_arcs=sum(1 for v in ev.values() if not v),
                    path_found=path is not None,
                    path_arcs=(len(path) - 1) if path else None,
                    path_len_m=round(length, 2) if path else None))
                prev_it = it
                done_cells.append((tr, tc))
            else:
                t_g = time.perf_counter()
                prm.refresh_local_edge_weights(global_map=gm, edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
                out['tempo_grafo_s'] += time.perf_counter() - t_g
        # --- invariant 2: global map against live, on random edges in the window --
        span = a['nx'] * a['cs']
        usable = spot_grid.slope_usable_mask(a['valid'].ravel(), a['od_raw'].ravel(), (a['ny'], a['nx']))
        single = global_maps.GlobalTerrainGrid()          # global map made of this scan only,
        global_maps.fold_scan_into_global_maps(           # with the mission's function
            global_maps.GlobalGrid(), single, a['pts'], a['z'].ravel(), a['obstacle_mask'], a['valid'].ravel(),
            a['od_raw'].ravel(), (a['ny'], a['nx']), a['robot'])
        for _ in range(ARCS_PER_SCAN):
            p = np.array([a['ox'], a['oy']]) + rng.uniform(0.3, span - 0.3, 2)
            ang = rng.uniform(0, 2 * math.pi)
            q = p + rng.uniform(0.5, 1.25) * np.array([math.cos(ang), math.sin(ang)])
            if not (a['ox'] + 0.3 < q[0] < a['ox'] + span - 0.3 and a['oy'] + 0.3 < q[1] < a['oy'] + span - 0.3):
                continue
            live = slopes.compute_arc_slope_profile(a['z'], a['ox'], a['oy'], a['cs'], *p, *q, valid_2d=usable)
            if not live[2]:
                continue
            glob_ = single.compute_arc_slope_profile(*p, *q)
            gl_total += 1
            gl_same += bool(glob_[2] and abs(glob_[0] - live[0]) < 0.02 and abs(glob_[1] - live[1]) < 0.02)
    out['invariants']['grafo_locale_uguale_al_completo'] = [full_same, full_checks]
    out['invariants']['aggiornamento_locale_usato'] = [local_used, local_expected]
    out['invariants']['mappa_globale_uguale_al_vivo'] = [gl_same, gl_total]
    out['invariants']['percorso_a_ogni_inizio_cella'] = [
        sum(c['path_found'] for c in out['cell_starts']), len(out['cell_starts'])]
    out['ritorno'] = return_route_check(name, gm)
    out['gait'] = gait_check(m)
    out['seconds'] = round(time.perf_counter() - t_start, 1)
    return name, out


def return_route_check(name, gm):
    """Return path to wp_0 on the downloaded GraphNav graph, today and with the shortcuts."""
    gdirs = glob.glob(os.path.join(ROOT, 'graph', f'Mission_{name}', 'Grafo_*'))
    log_path = os.path.join(ROOT, 'MissionLogs', f'Mission_{name}', 'mission_log.txt')
    if not gdirs:
        return None
    from bosdyn.api.graph_nav import map_pb2
    from bosdyn.client.frame_helpers import get_a_tform_b
    gdir = sorted(gdirs)[-1]
    g = map_pb2.Graph()
    g.ParseFromString(open(os.path.join(gdir, 'graph'), 'rb').read())
    names = {w.id: w.annotations.name for w in g.waypoints}
    by_name = {v: k for k, v in names.items()}
    xy = {}
    for w in g.waypoints:
        # As in the mission (RecordingInterface.waypoint_poses): position known only for the
        # waypoints created by the code, wp_N; not for the automatic recording ones.
        snap = os.path.join(gdir, 'waypoint_snapshots', w.snapshot_id)
        if not re.fullmatch(r'wp_\d+', w.annotations.name) or not os.path.exists(snap):
            continue
        ws = map_pb2.WaypointSnapshot()
        ws.ParseFromString(open(snap, 'rb').read())
        t = get_a_tform_b(ws.robot_state.kinematic_state.transforms_snapshot, 'vision', 'body')
        xy[w.id] = (t.x, t.y)
    base = [(e.id.from_waypoint, e.id.to_waypoint,
             math.sqrt(e.from_tform_to.position.x ** 2 + e.from_tform_to.position.y ** 2
                       + e.from_tform_to.position.z ** 2)) for e in g.edges]
    log = open(log_path, errors='replace').read() if os.path.exists(log_path) else ''
    found = re.findall(r"prima di 'ritorno a wp_0': localizzato su (\S+)", log)
    start = by_name.get(found[-1]) if found else None
    goal = by_name.get('wp_0')
    if start is None or goal is None or goal not in xy and start not in xy:
        return None
    with _quiet():
        used, before, after = map_shortcuts.plan_graphnav_shortcuts(xy, base, start, goal, gm)
    if before is None:
        return None
    sx = xy.get(start)
    straight = math.dist(sx, xy[goal]) if sx and goal in xy else None
    return dict(start=names[start], oggi_m=round(before, 1), con_scorciatoie_m=round(after, 1),
                scorciatoie=len(used), linea_aria_m=round(straight, 1) if straight else None)


def gait_check(m):
    """Gait today's code would choose on every leg travelled live."""
    if m['segments'] is None:
        return None
    times = [(float(np.load(f)['time']), f) for f in m['scans']]
    rows = []
    for x in csv.DictReader(open(m['segments'])):
        if x['slope_source'] != 'live':
            continue
        t0 = datetime.datetime.strptime(x['timestamp'], '%Y-%m-%d %H:%M:%S').replace(tzinfo=MISSION_TZ).timestamp() - float(x['duration_s'])
        prev = [f for t, f in times if t <= t0 + 1.0]
        if not prev:
            continue
        a = scan_arrays(np.load(prev[-1]))
        usable = spot_grid.slope_usable_mask(a['valid'].ravel(), a['od_raw'].ravel(), (a['ny'], a['nx']))
        seg = [float(x[k]) for k in ('x_from', 'y_from', 'x_to', 'y_to')]
        prof = slopes.compute_arc_slope_profile(a['z'], a['ox'], a['oy'], a['cs'], *seg, valid_2d=usable,
                                            body_ext_m=slopes.ARC_BODY_EXTENSION_M)
        lo, la, has = prm_graph.body_slope(prof)
        if not has:
            continue
        with _quiet():
            movements.select_mobility_params(max(lo, la), float(x['roughness']))
        rows.append(dict(time=x['timestamp'][11:], tier=records.MISSION_STATS['last_tier'],
                         pitch=float(x['pitch_peak_abs_deg']), slope=round(max(lo, la), 3)))
    return rows


# --------------------------------------------------------------------------------------
# Verdict
# --------------------------------------------------------------------------------------
def summary_for_reference(out):
    tiers = {}
    for r in out['gait'] or []:
        tiers[r['tier']] = tiers.get(r['tier'], 0) + 1
    return dict(cell_starts=out['cell_starts'], ritorno=out.get('ritorno'), gait_tiers=dict(sorted(tiers.items())),
                gait_by_segment=[f"{r['time']} {r['tier']}" for r in out['gait'] or []])


def check_invariants(name, out):
    fails = []
    a, b = out['invariants']['grafo_locale_uguale_al_completo']
    print(f"  [{'OK ' if a == b else 'FAIL'}] aggiornamento locale del grafo identico al completo: {a}/{b}")
    if a != b:
        fails.append(f"{name}: grafo locale diverso dal completo ({a}/{b})")
    a, b = out['invariants']['aggiornamento_locale_usato']
    print(f"  [{'OK ' if a == b else 'FAIL'}] aggiornamento locale usato davvero: {a}/{b}")
    if a != b:
        fails.append(f"{name}: costruzione completa al posto dell'aggiornamento locale ({b - a} volte su {b})")
    a, b = out['invariants']['mappa_globale_uguale_al_vivo']
    ok = b > 0 and a / b >= GLOBAL_LIVE_MIN_MATCH
    print(f"  [{'OK ' if ok else 'FAIL'}] pendenza della mappa globale uguale al vivo: {a}/{b} "
          f"({100 * a / max(b, 1):.1f}%, minimo {100 * GLOBAL_LIVE_MIN_MATCH:.1f}%)")
    if not ok:
        fails.append(f"{name}: mappa globale diversa dal vivo ({a}/{b})")
    if out['gait'] is not None:
        big = [r for r in out['gait'] if r['pitch'] > HARD_PITCH_DEG]
        hard = [r for r in big if r['tier'] == 'HARD']
        ok = len(hard) == len(big)
        print(f"  [{'OK ' if ok else 'FAIL'}] tratti con beccheggio oltre {HARD_PITCH_DEG:.0f} gradi in HARD: "
              f"{len(hard)}/{len(big)} (tratti rigiocati {len(out['gait'])})")
        if not ok:
            fails.append(f"{name}: tratti ripidi non HARD: "
                         + ", ".join(f"{r['time']} {r['tier']} {r['pitch']:.0f} gradi" for r in big if r['tier'] != 'HARD'))
    else:
        print("  [-- ] andatura: segments.csv mancante, controllo saltato")
    r = out.get('ritorno')
    if r is None:
        print("  [-- ] ritorno a wp_0: grafo GraphNav scaricato non presente, controllo saltato")
    else:
        limit = 1.5 * r['linea_aria_m'] + 2.0 if r['linea_aria_m'] else float('inf')
        ok = r['con_scorciatoie_m'] <= r['oggi_m'] + 1e-6 and r['con_scorciatoie_m'] <= limit
        print(f"  [{'OK ' if ok else 'FAIL'}] ritorno a wp_0 da {r['start']}: {r['oggi_m']} m nel grafo della missione "
              f"-> {r['con_scorciatoie_m']} m con {r['scorciatoie']} scorciatoie (linea d'aria {r['linea_aria_m']} m)")
        if not ok:
            fails.append(f"{name}: ritorno a wp_0 {r['con_scorciatoie_m']} m (oggi {r['oggi_m']}, limite {limit:.1f})")
    a, b = out['invariants']['percorso_a_ogni_inizio_cella']
    print(f"  [{'OK ' if a == b else 'FAIL'}] percorso trovato a ogni inizio di cella: {a}/{b}")
    if a != b:
        fails.append(f"{name}: nessun percorso in {b - a} inizi di cella")
    return fails


def compare_reference(name, now, ref):
    """List the behaviour differences from the reference."""
    if ref is None:
        print("  [-- ] nessun riferimento salvato per questa missione")
        return []
    diffs = []
    rc, nc = ref['cell_starts'], now['cell_starts']
    if len(rc) != len(nc):
        diffs.append(f"inizi di cella {len(rc)} -> {len(nc)}")
    for r, n in zip(rc, nc):
        for k in ('valid_arcs', 'discarded_arcs', 'path_found', 'path_arcs', 'path_len_m'):
            if r.get(k) != n.get(k):
                diffs.append(f"tentativo {n['attempt']} cella {tuple(n['cell'])}: {k} {r.get(k)} -> {n.get(k)}")
    if ref.get('ritorno') != now.get('ritorno'):
        diffs.append(f"ritorno a wp_0 {ref.get('ritorno')} -> {now.get('ritorno')}")
    if ref['gait_tiers'] != now['gait_tiers']:
        diffs.append(f"andature {ref['gait_tiers']} -> {now['gait_tiers']}")
    rs, ns = dict(s.split(' ', 1) for s in ref['gait_by_segment']), dict(s.split(' ', 1) for s in now['gait_by_segment'])
    for t in sorted(set(rs) | set(ns)):
        if rs.get(t) != ns.get(t):
            diffs.append(f"tratto delle {t}: andatura {rs.get(t)} -> {ns.get(t)}")
    if diffs:
        print(f"  [DIFF] comportamento diverso dal riferimento ({len(diffs)} differenze):")
        for line in diffs[:25]:
            print(f"         {line}")
        if len(diffs) > 25:
            print(f"         ... e altre {len(diffs) - 25}")
    else:
        print("  [OK ] comportamento identico al riferimento")
    return diffs


def graph_table(names, results):
    """One line per mission: the graph used, the paths at cell start and the graph time."""
    print("\n" + "=" * 70)
    print(f"{'missione':22s}{'nodi':>7}{'coppie':>8}{'percorsi':>10}{'lungh.media':>13}"
          f"{'archi medi':>12}{'tempo grafo':>13}")
    tot_t, lens, arcs = 0.0, [], []
    for name in names:
        out = results.get(name)
        if out is None:
            continue
        cs = [c for c in out['cell_starts'] if c['path_found']]
        g = out.get('grafo', {})
        ml = np.mean([c['path_len_m'] for c in cs]) if cs else float('nan')
        ma = np.mean([c['path_arcs'] for c in cs]) if cs else float('nan')
        lens += [c['path_len_m'] for c in cs]
        arcs += [c['path_arcs'] for c in cs]
        tot_t += out['tempo_grafo_s']
        print(f"{name:22s}{g.get('nodi', 0):>7}{g.get('coppie', 0):>8}"
              f"{len(cs):>6}/{len(out['cell_starts']):<3}{ml:>11.2f} m{ma:>12.1f}"
              f"{out['tempo_grafo_s']:>11.1f} s")
    if lens:
        print(f"{'TOTALE':22s}{'':>15}{len(lens):>10}{np.mean(lens):>11.2f} m"
              f"{np.mean(arcs):>12.1f}{tot_t:>11.1f} s")


def main():
    ap = argparse.ArgumentParser(description="Rigioco delle missioni salvate (vedi docstring)")
    ap.add_argument('--missions', nargs='+', default=DEFAULT_MISSIONS,
                    help="nomi come 13-10-2026_12-40-38 (default: le missioni di riferimento)")
    ap.add_argument('--all', action='store_true',
                    help=f"tutte le {len(ALL_MISSIONS)} missioni rigiocabili (circa 5 minuti)")
    ap.add_argument('--fast', action='store_true',
                    help=f"confronto col grafo completo solo su {FAST_FULL_CHECKS} inizi di cella sparsi")
    ap.add_argument('--update-reference', action='store_true',
                    help="salva il comportamento di adesso come nuovo riferimento")
    from spot_explorer.planning import mission_graph
    ap.add_argument('--graph', choices=mission_graph.GRAPH_SAMPLINGS,
                    help="rigioca con questo metodo dei nodi (vedi CONFRONTO FRA GRAFI)")
    ap.add_argument('--distribution', choices=mission_graph.RANDOM_DISTRIBUTIONS)
    ap.add_argument('--density', type=float, help="nodi casuali per m2")
    ap.add_argument('--spacing', type=float, help="passo del reticolo in m")
    ap.add_argument('--seed', type=int, help="seme dei nodi casuali (default: estratto a caso)")
    ap.add_argument('--min-edge', type=float, help="lunghezza minima degli archi in m")
    ap.add_argument('--max-edge', type=float, help="lunghezza massima degli archi in m")
    args = ap.parse_args()

    override = {}
    if args.graph is not None:
        override['MISSION_GRAPH_SAMPLING'] = args.graph
    if args.distribution is not None:
        override['MISSION_RANDOM_DISTRIBUTION'] = args.distribution
    if args.density is not None:
        override['MISSION_RANDOM_DENSITY'] = args.density
    if args.spacing is not None:
        override['MISSION_LATTICE_SPACING_M'] = args.spacing
    if args.min_edge is not None:
        override['PRM_MIN_EDGE_M'] = args.min_edge
    if args.max_edge is not None:
        override['PRM_MAX_EDGE_M'] = args.max_edge
    if override and override.get('MISSION_GRAPH_SAMPLING', P.MISSION_GRAPH_SAMPLING) == 'random':
        # A single seed for all missions, written below so the comparison can be repeated.
        override['MISSION_RANDOM_SEED'] = (args.seed if args.seed is not None
                                           else mission_graph.draw_seed())
    if override and args.update_reference:
        ap.error("--update-reference non si usa con un grafo diverso da quello della missione")

    if args.all:
        args.missions = ALL_MISSIONS
    t0 = time.perf_counter()
    print(f"Rigioco di {len(args.missions)} missioni{' (veloce)' if args.fast else ''}, in parallelo...")
    if override:
        print("Grafo diverso da quello delle missioni: " +
              ", ".join(f"{k}={v}" for k, v in override.items()))
    with Pool(min(len(args.missions), os.cpu_count() or 1)) as pool:
        results = dict(pool.map(replay_mission, [(n, args.fast, override) for n in args.missions]))

    ref_all = json.load(open(REFERENCE_FILE)) if os.path.exists(REFERENCE_FILE) else {}
    fails, diffs, new_ref = [], [], dict(ref_all)
    for name in args.missions:
        out = results[name]
        print(f"\n== {name}")
        if out is None:
            print("  [-- ] scansioni o log non presenti su questo PC: missione saltata")
            continue
        print(f"  rigiocata in {out['seconds']:.0f} s: {len(out['cell_starts'])} inizi di cella")
        fails += check_invariants(name, out)
        now = summary_for_reference(out)
        if override:
            print("  [-- ] grafo diverso da quello della missione: confronto col riferimento saltato")
        else:
            diffs += [f"{name}: {x}" for x in compare_reference(name, now, ref_all.get(name))]
        new_ref[name] = now

    graph_table(args.missions, results)

    print("\n" + "=" * 70)
    if args.update_reference and not fails:
        json.dump(new_ref, open(REFERENCE_FILE, 'w'), indent=1, ensure_ascii=False)
        print(f"Riferimento aggiornato: {os.path.basename(REFERENCE_FILE)} (va nel commit con la modifica).")
        diffs = []
    elif args.update_reference:
        print("Riferimento NON aggiornato: prima vanno risolti gli invarianti falliti.")
    if fails:
        print(f"INVARIANTI FALLITI ({len(fails)}):")
        for x in fails:
            print(f"  - {x}")
    if diffs:
        print(f"COMPORTAMENTO CAMBIATO rispetto al riferimento ({len(diffs)} differenze). Se e' voluto: "
              f"python3 test_replay.py --update-reference")
    if not fails and not diffs:
        print("TUTTO OK")
    print(f"({time.perf_counter() - t0:.0f} s)")
    sys.exit(1 if (fails or diffs) else 0)


if __name__ == '__main__':
    main()
