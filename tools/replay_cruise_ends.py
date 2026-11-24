#!/usr/bin/env python3
"""
The cruise ends of real missions, planned again with today's code (no robot): for every
"[CRUISE] End:" line of the mission logs whose reason matches --reason, the graph and the
global maps are rebuilt from the mission's saved scans up to the last cruise scan before the
end (tests/test_replay.py), and one planning cycle runs on that scan as the planning thread
would: the robot as a temporary node (PRM.connect_point), Dijkstra from it to the goal, the
graph/frontier rule (planning_cycle.closed_edge, edges out until an open path or none), the
confirmed stretch (walk_advance) and the heading rule (CruisePlanner._turns_within).

One line per end: whether a path from the robot exists, its first node (ahead / side /
behind the robot, the same as the old path or not), the stretch confirmed and why it stops,
the edges taken out on the way.

    python3 tools/replay_cruise_ends.py                                   # "out of the graph"
    python3 tools/replay_cruise_ends.py --reason "confirmed stretch ends" --jobs 4
    python3 tools/replay_cruise_ends.py --missions 09-11-2026_14-53-46 --reason .

Limits: the cruise saves a scan every CRUISE_BUNDLE_STEP_M or so, so the scan used is up to a
few seconds before the end and may not show what ended the cruise; the graph is rebuilt in
full (in the mission it is updated incrementally) and the cells allowed for the path are not
restricted.
"""
import argparse
import datetime
import glob
import math
import os
import re
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, 'tests')]

import test_replay as R                                                   # noqa: E402
from spot_explorer import config as P                                     # noqa: E402
from spot_explorer.mission import cells, mission_prm, planning_cycle as PC  # noqa: E402
from spot_explorer.perception import geometry, global_maps, spot_grid     # noqa: E402
from spot_explorer.planning import edge_evaluation as EE, safe_frontier   # noqa: E402

MAX_DROPS = 30


def ends(missions, reason):
    """(mission, 'hh:mm:ss.fff', reason text) of the cruise ends matching reason."""
    out = []
    for log in sorted(glob.glob(os.path.join(ROOT, 'MissionLogs', 'Mission_*', 'mission_log.txt'))):
        name = os.path.basename(os.path.dirname(log))[len('Mission_'):]
        if missions and name not in missions:
            continue
        for line in open(log, errors='replace'):
            m = re.match(r'(\d\d:\d\d:\d\d\.\d+) \[CRUISE\] End: (.*?)\. [\d.]+ m in', line)
            if m and re.search(reason, m.group(2)):
                out.append((name, m.group(1), m.group(2)))
    return out


def snapshot(d):
    """The frontier's GridSnapshot of a saved scan."""
    z = d['terrain_corrected']
    sh = z.shape
    seen = np.asarray(d['terrain_valid_raw']).reshape(sh) >= spot_grid.TERRAIN_VALID_MIN
    if 'unwritten' in d.files and np.size(d['unwritten']) == z.size:
        seen &= ~np.asarray(d['unwritten'], bool).reshape(sh)
    valid = np.asarray(d['is_valid'], bool).reshape(sh)
    rough = np.asarray(d['roughness'], float).reshape(sh)
    latch = np.asarray(d['blocked_latch'], bool).reshape(sh) if 'blocked_latch' in d.files else None
    return safe_frontier.GridSnapshot(
        np.asarray(d['obstacle_distance_filtered'], float).reshape(sh),
        (rough > spot_grid.ROUGH_THRESHOLD) & valid, seen, np.asarray(z, float),
        float(d['grid_origin'][0]), float(d['grid_origin'][1]), float(d['cell_size']),
        float(d['robot_xyz'][0]), float(d['robot_xyz'][1]), valid=valid, blocked_latch=latch,
        robot_yaw=float(d['robot_yaw']))


def _clock(day, hhmmss):
    return datetime.datetime.combine(day, datetime.datetime.strptime(hhmmss, '%H:%M:%S.%f').time()).timestamp()


def replay_end(job):
    name, hhmmss, why = job
    m = R.mission_setup(name)
    if m is None:
        return f"{name} {hhmmss[:8]}: mission data missing"
    day = datetime.datetime.strptime(name.split('_')[0], '%d-%m-%Y').date()
    times = [float(np.load(f)['time']) for f in m['scans']]
    # The log clock and the scan times may differ by whole hours (time zone).
    log = open(os.path.join(ROOT, 'MissionLogs', f'Mission_{name}', 'mission_log.txt'), errors='replace').read()
    first = re.search(r'^(\d\d:\d\d:\d\d\.\d+) ', log, re.M).group(1)
    t_end = _clock(day, hhmmss) + round((times[0] - _clock(day, first)) / 3600.0) * 3600.0
    cruise = [i for i, f in enumerate(m['scans']) if '_cruise' in os.path.basename(f) and times[i] <= t_end + 0.5]
    if not cruise:
        return f"{name} {hhmmss[:8]}: no cruise scan before the end"
    i_end = cruise[-1]
    with R._quiet():
        env = cells.EnvironmentMap(rows=m['rows'], cols=m['cols'], cell_size=m['cell'])
        env.set_origin(m['origin'][0], m['origin'][1], math.radians(m['yaw_deg']), *m['start_cell'])
        P.MISSION_LATTICE_SPACING_M = m['spacing']
        g = m['graph']
        P.MISSION_GRAPH_SAMPLING = g['sampling']
        P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = g['min_edge'], g['max_edge']
        if g['sampling'] == 'random':
            P.MISSION_RANDOM_DISTRIBUTION, P.MISSION_RANDOM_DENSITY, P.MISSION_RANDOM_SEED = \
                g['distribution'], g['density'], g['seed']
        prm, _sampler, _mg = mission_prm.build_mission_prm(env, verbose=False)
        gm, tm = global_maps.GlobalGrid(), global_maps.GlobalTerrainGrid()
        tm.update_rects = []
        prm.set_global_terrain_map(tm)
        for f in m['scans'][:i_end + 1]:
            R.feed_scan(R.scan_arrays(np.load(f)), prm, gm, tm)
        mission_prm.rebuild_prm_edges(prm, gm)
    d = np.load(m['scans'][i_end], allow_pickle=True)
    snap = snapshot(d)
    robot = (float(d['robot_xyz'][0]), float(d['robot_xyz'][1]), float(d['robot_yaw']))
    gx, gy = map(float, d['goal_xy'])
    goal = min(prm.nodes, key=lambda k: math.hypot(prm.nodes[k][0] - gx, prm.nodes[k][1] - gy))
    old_ids = [int(i) for i in np.ravel(d['path_node_ids'])] if 'path_node_ids' in d.files else []
    tmp = EE.TEMP_NODE
    with R._quiet():
        PC.refresh_graph(prm, snap, gm, robot, refresh=True, quiet=True)
        prm.connect_point(robot[0], robot[1], gm, geometry.PRM_EDGE_SAFETY_MARGIN_M)
        links = len(prm.edges.get(tmp, []))
        dropped, found = [], None
        while True:
            found = prm.find_path_dijkstra(tmp, goal)
            if found is None:
                break
            rem = list(found[1:])
            poly = [robot[:2]] + [prm.get_node_position(n) for n in rem]
            fr = safe_frontier.compute_safe_frontier(poly, snap, allow_escape=False)
            e = PC.closed_edge(poly, [tmp] + rem, fr)
            if e is None or e in dropped or len(dropped) >= MAX_DROPS:
                break
            dropped.append(e)
            prm._set_edge(*e, None)
        prm.remove_point()
    head = (f"{name[:16]} {hhmmss[:8]} scan {t_end - times[i_end]:4.1f} s before | links {links:2d} | "
            f"edges out {len(dropped)}")
    if found is None:
        return head + " | NO PATH from the robot"
    adv = PC.walk_advance(poly, fr)
    adv, turn_stop, back = PC.CruisePlanner._turns_within(poly, rem, adv, snap, robot[2])
    nx, ny = prm.get_node_position(rem[0])
    brg = (math.degrees(math.atan2(ny - robot[1], nx - robot[0]) - robot[2]) + 180.0) % 360.0 - 180.0
    where = 'ahead' if abs(brg) < 45 else 'behind' if abs(brg) > 135 else 'side'
    same = 'same' if rem[:1] == old_ids[:1] else 'old path' if rem[0] in old_ids else 'new'
    return (head + f" | first node {rem[0]} {math.hypot(nx - robot[0], ny - robot[1]):.2f} m {where} "
            f"({brg:+.0f} deg, {same}) | confirmed {adv:.2f} m ({fr['reason']})"
            + (f", rotation stop at {turn_stop}" if turn_stop else "")
            + (" | backward" if any(back[:2]) else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reason', default='out of the graph', help='regex on the end reason')
    ap.add_argument('--missions', nargs='*', help='e.g. 09-11-2026_14-53-46 (default: all)')
    ap.add_argument('--jobs', type=int, default=4)
    a = ap.parse_args()
    jobs = ends(a.missions, a.reason)
    print(f"{len(jobs)} cruise ends matching '{a.reason}'.")
    with Pool(max(1, a.jobs)) as pool:
        for line in pool.imap(replay_end, jobs):
            print(line, flush=True)


if __name__ == '__main__':
    main()
