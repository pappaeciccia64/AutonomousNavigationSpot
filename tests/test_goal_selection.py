"""
Test of the CELL GOAL SELECTION on the real scans saved during missions.

test_replay.py replays the scans but uses the goal the mission had chosen. Here, at the
start of every attempt, the choice is made again with today's code
(goal.find_best_point_in_cell, with reachability in the graph and the memory of refused
goals) and put next to the mission's one, with the same measures:

  reachable      -- the graph, rebuilt as in the mission, connects the robot to the goal;
  refusable      -- in one of the attempt's scans the goal has obstacle_distance
                    (filtered, the one the frontier decides on) below FRONTIER_CLEARANCE_M,
                    or the frontier confirmed a block within REFUSED_GOAL_BLOCK_M of it:
                    the robot would never get there. Judged only where the scans cover it,
                    i.e. along the path taken in the mission (see limits). Many refusables
                    are centres still unknown at the start of the attempt: the navigation
                    loop rediscovers them, no initial choice can avoid them;
  on the border  -- less than TARGET_CELL_BORDER_M from the cell border;
  proposed again -- equal (within 0.30 m) to a goal of the same cell already found
                    refusable in a previous attempt.

For today's code a refusable goal is excluded for the rest of the mission
(goal._refuse_goal), as the navigation loop would do.

Usage:
    python3 tests/test_goal_selection.py                     # the outdoor mission of 22-10
    python3 tests/test_goal_selection.py --missions 13-10-2026_12-40-38 22-10-2026_15-50-23
    python3 tests/test_goal_selection.py --detail            # one line per attempt

Before the replay, synthetic checks of the rules on hand-built cases.

Limits: the replayed robot follows the mission's path, not the one towards the new goal;
for the new goal "refusable" is only seen if the scans cover it. The scan taken at the
start of the attempt is not saved: the replayed global map has fewer of them, and a centre
that was occupied in the mission may turn out free here (22-10 16:04, cell (5,2)). Exits
with code 1 if a synthetic check fails, or if today's code does worse than the mission on
reachable, refusable or proposed-again goals.
"""
import argparse
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_replay as tr          # noqa: E402  (also imports the mission code quietly)
from test_replay import goal, P    # noqa: E402
from spot_explorer.perception import geometry, global_maps   # noqa: E402
from spot_explorer.legacy import modern   # noqa: E402

DEFAULT = ['22-10-2026_16-04-22']


def _od_at(a, x, y):
    """Filtered obstacle_distance at the point, None outside the scan."""
    c = int((x - a['ox']) / a['cs'])
    r = int((y - a['oy']) / a['cs'])
    if 0 <= r < a['ny'] and 0 <= c < a['nx']:
        return float(a['od_filt'][r, c])
    return None


def _bad(g_):
    """The frontier would refuse the goal: live free space below threshold, or a block right next to it."""
    return g_['xy'] is not None and (
        g_.get('blocked', False) or (bool(g_['od']) and min(g_['od']) < geometry.FRONTIER_CLEARANCE_M))


def replay_goals(name):
    m = tr.mission_setup(name)
    if m is None:
        return None
    with tr._quiet():
        env = tr.environmentMap.EnvironmentMap(rows=m['rows'], cols=m['cols'], cell_size=m['cell'])
        env.set_origin(m['origin'][0], m['origin'][1], math.radians(m['yaw_deg']))
        P.MISSION_LATTICE_SPACING_M = m['spacing']
        g = m['graph']
        P.MISSION_GRAPH_SAMPLING = g['sampling']
        P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = g['min_edge'], g['max_edge']
        if g['sampling'] == 'random':
            P.MISSION_RANDOM_DISTRIBUTION = g['distribution']
            P.MISSION_RANDOM_DENSITY = g['density']
            P.MISSION_RANDOM_SEED = g['seed']
        prm, sampler, mg = tr.mission_graph.build_mission_prm(env, verbose=False)
        gm = global_maps.GlobalGrid()
        tm = global_maps.GlobalTerrainGrid()
        tm.update_rects = []
        prm.set_global_terrain_map(tm)
    node_cells = None
    done_cells = []
    attempts = []          # one dict per attempt
    cur = None
    for f in m['scans']:
        d = np.load(f)
        it, rr, cc = (int(v) for v in re.search(r'scan_it(\d+)_cell(\d+)_(\d+)', f).groups())
        a = tr.scan_arrays(d)
        rx, ry, _ = a['robot']
        with tr._quiet():
            tr.feed_scan(a, prm, gm, tm)
        if cur is None or cur['it'] != it:
            with tr._quiet():
                tr.mission_graph.rebuild_prm_edges(prm, gm)
                if node_cells is None:
                    node_cells = tr.mission_graph.node_cells(prm, env)
                cells = set(done_cells) | {(rr, cc)} | {tuple(int(v) for v in env.get_cell_from_world(rx, ry))}
                prm.set_allowed_nodes([n for n, c in node_cells.items() if c in cells])
                robot_node = prm.anchor_node(rx, ry, gm, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                             ignore_near_m=prm_graph_ignore())
                prm.set_robot_node(robot_node, 0.0)

                def reach(px, py, _rn=robot_node):
                    nid = prm.anchor_node(px, py, gm, geometry.PRM_EDGE_SAFETY_MARGIN_M)
                    return nid is not None and prm.find_path_dijkstra(_rn, nid) is not None

                nx_, ny_ = goal.find_best_point_in_cell(
                    rx, ry, env, rr, cc, a['pts'], a['od_raw'].ravel(), sampler,
                    global_map=gm, reachable=reach)[:2]
                mx, my = map(float, d['goal_xy'])
                cur = dict(it=it, cell=(rr, cc),
                           old=dict(xy=(mx, my), reach=reach(mx, my), od=[]),
                           new=dict(xy=None if nx_ is None else (float(nx_), float(ny_)),
                                    reach=(nx_ is not None and reach(nx_, ny_)), od=[]))
                prm.set_allowed_nodes(None)
            attempts.append(cur)
            done_cells.append((rr, cc))
        for k in ('old', 'new'):
            if cur[k]['xy'] is not None:
                v = _od_at(a, *cur[k]['xy'])
                if v is not None:
                    cur[k]['od'].append(v)
        # Block confirmed right next to the mission's goal (the replayed path is the one
        # towards it): it applies to the new goal only if it is the same point.
        if (str(d['decision_kind']) == 'blocked'
                and modern(str(d['frontier_reason'])) in ('obstacle', 'roughness', 'slope')):
            sxy = tuple(map(float, d['frontier_stop_xy']))
            if math.dist(sxy, cur['old']['xy']) <= P.REFUSED_GOAL_BLOCK_M:
                cur['old']['blocked'] = True
                if (cur['new']['xy'] is not None
                        and math.dist(cur['new']['xy'], cur['old']['xy']) < P.RECOVERY_SAME_GOAL_M):
                    cur['new']['blocked'] = True
        # The refusable new goal is excluded for the mission, as the loop does.
        if _bad(cur['new']):
            with tr._quiet():
                goal._refuse_goal(env, rr, cc, *cur['new']['xy'], "replay")
    # Measures
    out = {}
    for k in ('old', 'new'):
        refused_by_cell = {}
        n = dict(attempts=0, no_goal=0, reachable=0, refusable=0, on_border=0,
                 proposed_again=0)
        for at in attempts:
            g_ = at[k]
            n['attempts'] += 1
            if g_['xy'] is None:
                n['no_goal'] += 1
                continue
            n['reachable'] += bool(g_['reach'])
            bad = _bad(g_)
            n['refusable'] += bad
            b = tr.environmentMap.cell_border_distance(env, *at['cell'], *g_['xy'])
            n['on_border'] += b is not None and b < P.TARGET_CELL_BORDER_M
            prev = refused_by_cell.setdefault(at['cell'], [])
            n['proposed_again'] += any(math.dist(g_['xy'], p) < P.RECOVERY_SAME_GOAL_M for p in prev)
            if bad:
                prev.append(g_['xy'])
        out[k] = n
    return dict(measures=out, attempts=attempts, env=env)


class _Map:
    """Fake global map: point obstacles, cells occupied within OBSTACLE_THRESHOLD."""

    def __init__(self, obstacles):
        self.obs = [tuple(o) for o in obstacles]

    def _d(self, x, y):
        return min((math.dist((x, y), o) for o in self.obs), default=99.0)

    def is_occupied(self, x, y, safety_margin=0.0):
        return self._d(x, y) <= geometry.OBSTACLE_THRESHOLD + safety_margin

    def is_known(self, x, y):
        return True

    def clearance(self, x, y):
        return self._d(x, y)


def synthetic_checks():
    """The rules of find_best_point_in_cell on hand-built cases. List of failures."""
    fails = []

    def check(cond, msg):
        print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
        if not cond:
            fails.append(msg)

    def pick(obstacles, reachable=None, env=None, robot=(-3.0, 0.0)):
        env = env or _env()
        pts = [(x, y) for x in np.linspace(-0.9, 0.9, 7) for y in np.linspace(-0.9, 0.9, 7)]
        sampler = type('S', (), {'get_point_in_cell': lambda self, r, c: pts})()
        with tr._quiet():
            x, y, _, _ = goal.find_best_point_in_cell(robot[0], robot[1], env, 0, 0, None, None, sampler,
                                                    global_map=_Map(obstacles), reachable=reachable)
        return env, (x, y)

    def _env():
        e = tr.environmentMap.EnvironmentMap(rows=1, cols=1, cell_size=2.0)
        with tr._quiet():
            e.set_origin(0.0, 0.0, 0.0)
        return e

    print("Synthetic checks of the goal choice (2 m cell centred at (0, 0)):")
    e = _env()
    with tr._quiet():
        e30 = tr.environmentMap.EnvironmentMap(rows=1, cols=1, cell_size=2.0)
        e30.set_origin(0.0, 0.0, math.radians(30))
    check(abs(tr.environmentMap.cell_border_distance(e, 0, 0, 0.0, 0.0) - 1.0) < 1e-9
          and abs(tr.environmentMap.cell_border_distance(e, 0, 0, 0.9, 0.2) - 0.1) < 1e-9
          and abs(tr.environmentMap.cell_border_distance(e, 0, 0, 1.2, 0.0) + 0.2) < 1e-9,
          "distance from the border: 1.0 at the centre, 0.1 at 0.9 m, -0.2 outside")
    p = (0.9 * math.cos(math.radians(30)), 0.9 * math.sin(math.radians(30)))
    check(abs(tr.environmentMap.cell_border_distance(e30, 0, 0, *p) - 0.1) < 1e-9,
          "distance from the border with the grid rotated by 30 degrees")

    _, g = pick([])
    check(g == (0.0, 0.0), f"open field: the centre {g}")
    _, g = pick([], reachable=lambda x, y: math.hypot(x, y) > 0.5)
    check(g != (0.0, 0.0) and math.hypot(*g) > 0.5,
          f"centre not reachable: a reachable point {tuple(round(v, 2) for v in g)}")
    _, g = pick([], reachable=lambda x, y: False)
    check(g == (0.0, 0.0), "no reachable point: the best one anyway (the centre)")
    _, g = pick([(0.30, 0.0)])
    check(g != (0.0, 0.0) and math.dist(g, (0.30, 0.0)) >= P.TARGET_CLEARANCE_M,
          f"centre 0.30 m from an obstacle (below {P.TARGET_CLEARANCE_M:.2f}): a point with more "
          f"space {tuple(round(v, 2) for v in g)}")
    _, g = pick([(0.0, 0.0)])
    b = tr.environmentMap.cell_border_distance(e, 0, 0, *g)
    check(b >= P.TARGET_CELL_BORDER_M,
          f"centre occupied: the fallback is at least {P.TARGET_CELL_BORDER_M:.2f} m from the border ({b:.2f})")
    env, g1 = pick([(0.0, 0.0)])
    with tr._quiet():
        goal._refuse_goal(env, 0, 0, *g1, "prova")
    env, g2 = pick([(0.0, 0.0)], env=env)
    check(math.dist(g1, g2) >= P.RECOVERY_SAME_GOAL_M,
          f"refused goal not proposed again: {tuple(round(v, 2) for v in g1)} -> "
          f"{tuple(round(v, 2) for v in g2)}")
    env2 = _env()
    with tr._quiet():
        for x in np.linspace(-0.9, 0.9, 7):
            for y in np.linspace(-0.9, 0.9, 7):
                goal._refuse_goal(env2, 0, 0, x, y, "prova")
        goal._refuse_goal(env2, 0, 0, 0.0, 0.0, "prova")
    _, g = pick([], env=env2)
    check(g[0] is not None, "all points refused: the best one is retried instead of giving up")
    return fails


def prm_graph_ignore():
    from spot_explorer.planning import roadmap as prm_graph
    return prm_graph.ANCHOR_IGNORE_NEAR_M


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--missions', nargs='+', default=DEFAULT)
    ap.add_argument('--detail', action='store_true')
    args = ap.parse_args()
    worse = bool(synthetic_checks())
    for name in args.missions:
        res = replay_goals(name)
        if res is None:
            print(f"{name}: scans or log not found on this PC, skipping.")
            continue
        o, n = res['measures']['old'], res['measures']['new']
        print(f"\n=== {name}: {o['attempts']} attempt starts ===")
        print(f"  {'':22s}{'mission':>10s}{'today':>8s}")
        for k, label in (('reachable', 'reachable'), ('refusable', 'refusable by the frontier'),
                         ('proposed_again', 'proposed again after refusal'), ('on_border', 'on the cell border'),
                         ('no_goal', 'without goal')):
            print(f"  {label:22s}{o[k]:10d}{n[k]:8d}")
        if args.detail:
            for at in res['attempts']:
                def fmt(g_):
                    if g_['xy'] is None:
                        return "none".ljust(38)
                    od = f"{min(g_['od']):.2f}" if g_['od'] else " -- "
                    return (f"({g_['xy'][0]:6.2f},{g_['xy'][1]:7.2f}) "
                            f"{'R' if g_['reach'] else '-'} od_min {od}").ljust(38)
                print(f"   it{at['it']:3d} cell {at['cell']}: mission {fmt(at['old'])} today {fmt(at['new'])}")
        if (n['reachable'] < o['reachable'] or n['refusable'] > o['refusable']
                or n['proposed_again'] > o['proposed_again']):
            worse = True
            print("  CAREFUL: today's code does worse than the mission on at least one measure.")
    sys.exit(1 if worse else 0)


if __name__ == '__main__':
    main()
