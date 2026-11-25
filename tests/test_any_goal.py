#!/usr/bin/env python3
"""
Any node of the graph as the goal: the navigation gets there, or says it cannot, and never
hangs. Independent of how the goals are chosen (cells, serpentine, frontier).

    python3 tests/test_any_goal.py [--goals 6] [--worlds A B rand1 rand2 rand3] [--seed 1]

For each world a fake mission in cruise (tests/test_mission.run), where the exploration is
replaced by a sequence of goals drawn at random among ALL the nodes of the mission graph:
seen or never seen, in the open, next to boxes, inside boxes, behind walls. Each goal is one
attempt of the real code (cell_attempt -> cruise), with the goal forced on the node, and no
next cell chained (env._cell_rules = None): the robot goes to that node and stops.

The truth comes from the fake world: a node is reachable if a disk of radius
TRUTH_RADIUS_M (the 0.30 m clearance rule) can slide from the start to it through free
space. For every goal:
  - reachable and arrived (within GOAL_REACHED_TOLERANCE_M)        -> OK
  - unreachable and not arrived, with a reason                     -> OK (it noticed)
  - unreachable and arrived                                         -> FAIL (it went where it
    could not go: through an obstacle or closer than the rule)
  - reachable and not arrived                                       -> listed (a miss: e.g. a
    goal beside a box where the body cannot turn, accepted by the user, 25-11)
  - an attempt longer than GOAL_TIME_LIMIT_S                        -> FAIL (a hang)
and over the whole mission: the closest approach of the robot centre to an obstacle must
stay at or above CLEARANCE_MIN_M.
"""
import argparse
import contextlib
import io
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), HERE, os.path.join(os.path.dirname(HERE), 'tools')]

import numpy as np                                                   # noqa: E402
from scipy.ndimage import distance_transform_edt, label             # noqa: E402

import test_e2e_loop as T                                            # noqa: E402
import test_mission as M                                             # noqa: E402
from behaviour_snapshot import random_world                          # noqa: E402
from spot_explorer import config as P                                # noqa: E402
from spot_explorer.mission import exploration, mission_prm           # noqa: E402
from spot_explorer.planning import goal                              # noqa: E402
from spot_explorer.robot import connection                           # noqa: E402

TRUTH_RADIUS_M = 0.30
CLEARANCE_MIN_M = 0.25        # half the body width: never closer (the 0.30 rule is a margin above it)
GOAL_TIME_LIMIT_S = 180.0
MIN_GOAL_DIST_M = 0.75        # goals nearer than this to the robot are skipped (nothing to plan)
FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def truth_reachable(world, start_xy):
    """Boolean (W, W) map: cells a disk of TRUTH_RADIUS_M reaches from the start."""
    free = distance_transform_edt(~world.occ) * T.CS > TRUTH_RADIUS_M
    lab, _n = label(free, structure=np.ones((3, 3)))
    r, c = int(start_xy[1] / T.CS), int(start_xy[0] / T.CS)
    k = lab[r, c]
    return (lab == k) if k else np.zeros_like(free)


class AnyGoalMission(exploration.Mission):
    """The exploration replaced by N goals drawn among all the graph nodes."""
    N_GOALS = 6
    SEED = 1
    results = []

    def explore(self):
        env, prm = self.env, self.prm
        env._cell_rules = None                     # no next cell chained: one goal, one attempt
        everywhere = mission_prm.planning_cells
        mission_prm.planning_cells = lambda *a, **k: None     # paths may cross any cell
        try:
            self._first_scan()
            rng = np.random.default_rng(self.SEED)
            reach = truth_reachable(T.WORLD, T.START_XY)
            inside = [i for i, (x, y) in prm.nodes.items()
                      if 0 <= env.get_cell_from_world(x, y)[0] < env.rows and 0 <= env.get_cell_from_world(x, y)[1] < env.cols]
            for k in range(self.N_GOALS):
                x, y, _, _ = connection.getPosition(self.robot_state_client)
                cand = [i for i in inside if np.hypot(prm.nodes[i][0] - x, prm.nodes[i][1] - y) >= MIN_GOAL_DIST_M]
                node = int(rng.choice(cand))
                gx, gy = (float(v) for v in prm.nodes[node])
                row, col = (int(v) for v in env.get_cell_from_world(gx, gy))
                r, c = int(gy / T.CS), int(gx / T.CS)
                truth = bool(0 <= r < T.W and 0 <= c < T.W and reach[r, c])
                seen = bool(self.global_grid.global_occupancy_map is not None
                            and self.global_grid.global_occupancy_map.is_known(gx, gy))
                env._forced_goal = (row, col, gx, gy)
                # As the frontier variant before every goal: the marks a failed attempt leaves on
                # the cell (left for later, chained choice) would end the next one at once.
                env._chain_choice = None
                env._attempt_left_unreachable = False
                env._attempt_fail_why = None
                print(f"[ANY-GOAL] Goal {k + 1}/{self.N_GOALS}: node {node} at ({gx:.2f}, {gy:.2f}) in cell "
                      f"({row},{col}); {'seen' if seen else 'never seen'}; truth: {'reachable' if truth else 'UNREACHABLE'}.")
                t0 = time.time()
                self._attempt(row, col)
                dt = time.time() - t0
                x, y, _, _ = connection.getPosition(self.robot_state_client)
                arrived = float(np.hypot(x - gx, y - gy)) <= P.GOAL_REACHED_TOLERANCE_M
                why = getattr(env, '_attempt_fail_why', None)
                self.results.append(dict(node=node, xy=(gx, gy), seen=seen, truth=truth, arrived=arrived,
                                         why=why, seconds=dt, end_dist=float(np.hypot(x - gx, y - gy))))
                print(f"[ANY-GOAL] -> {'arrived' if arrived else 'not arrived'} in {dt:.0f} s"
                      + ("" if arrived else f" ({why or 'no reason given'})") + ".")
                env._forced_goal = None
        finally:
            mission_prm.planning_cells = everywhere
            env._forced_goal = None

    def _first_scan(self):
        """The first scan and the first graph: an attempt on the robot's own cell, goal on the robot."""
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        row, col = (int(v) for v in self.env.get_cell_from_world(x, y))
        self.env._forced_goal = (row, col, float(x), float(y))
        self._attempt(row, col)
        self.env._forced_goal = None


def forced_goal_choice():
    """goal.find_best_point_in_cell with the goal forced on env._forced_goal (as the frontier
    variant): the forced point once; if already tried in the attempt, no goal (the attempt ends)."""
    best = goal.find_best_point_in_cell

    def choose(robot_x, robot_y, env, cell_row, cell_col, *a, exclude=None, **k):
        forced = getattr(env, '_forced_goal', None)
        if forced is not None and (int(forced[0]), int(forced[1])) == (int(cell_row), int(cell_col)):
            fp = (float(forced[2]), float(forced[3]))
            if any(np.hypot(fp[0] - ex, fp[1] - ey) < P.RECOVERY_SAME_GOAL_M for ex, ey in (exclude or [])):
                return None, None, [], [fp]
            return fp[0], fp[1], [fp], []
        return best(robot_x, robot_y, env, cell_row, cell_col, *a, exclude=exclude, **k)
    return choose, best


def one_world(name, n_goals, seed):
    if name.startswith('rand'):
        M.ROWS, M.COLS, M.CELL, M.START = 4, 5, 2.0, (1.0, 1.0, 0.0)
        walls = random_world(int(name[4:]), 4, 5, 2.0)
    else:
        M.ROWS, M.COLS, M.CELL, M.START = 3, 3, 3.0, (1.5, 1.5, 0.0)
        walls = M.WORLDS[name]
    T.START_XY = M.START[:2]
    AnyGoalMission.N_GOALS, AnyGoalMission.SEED, AnyGoalMission.results = n_goals, seed, []
    choose, best = forced_goal_choice()
    real_mission = exploration.Mission
    exploration.Mission, goal.find_best_point_in_cell = AnyGoalMission, choose
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            r = M.run(walls=walls, name=f'any_goal_{name}', cruise=True)
    finally:
        exploration.Mission, goal.find_best_point_in_cell = real_mission, best
    # Closest approach measured on the track, on the world's own distance map (the robot's
    # min_clear is updated only by some of the fake movers).
    track = np.array(T.ROBOT.track, dtype=float).reshape(-1, 2)
    clear = min((float(T.WORLD.od_at(x, y)) for x, y in track), default=float('inf'))
    return r, list(AnyGoalMission.results), clear


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--goals', type=int, default=6)
    ap.add_argument('--worlds', nargs='+', default=['A', 'B', 'rand1', 'rand2', 'rand3'])
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    totals = dict(ok_arrived=0, ok_noticed=0, misses=0, wrong=0)
    for w in a.worlds:
        print(f"== world {w}: {a.goals} random goals ==")
        r, res, clear = one_world(w, a.goals, a.seed)
        check('Traceback' not in r['log'], f"no traceback in the mission")
        for g in res:
            tag = f"node {g['node']} ({g['xy'][0]:.1f}, {g['xy'][1]:.1f}), {'seen' if g['seen'] else 'never seen'}"
            if g['seconds'] > GOAL_TIME_LIMIT_S:
                check(False, f"{tag}: attempt of {g['seconds']:.0f} s, a hang")
            if g['truth'] and g['arrived']:
                totals['ok_arrived'] += 1
            elif not g['truth'] and not g['arrived']:
                totals['ok_noticed'] += 1
                print(f"  [OK ] {tag}: unreachable, noticed ({g['why'] or 'attempt failed'}, {g['seconds']:.0f} s)")
            elif not g['truth'] and g['arrived']:
                totals['wrong'] += 1
                check(False, f"{tag}: unreachable for the body but reached ({g['end_dist']:.2f} m from it)")
            else:
                totals['misses'] += 1
                print(f"  [MISS] {tag}: reachable, not reached ({g['why'] or 'attempt failed'}, "
                      f"{g['end_dist']:.2f} m from it, {g['seconds']:.0f} s)")
        check(clear >= CLEARANCE_MIN_M, f"closest approach to an obstacle {clear:.3f} m (at least {CLEARANCE_MIN_M})")
    n = sum(totals.values())
    print(f"\n{n} goals: {totals['ok_arrived']} reachable and reached, {totals['ok_noticed']} unreachable and "
          f"noticed, {totals['misses']} reachable but missed, {totals['wrong']} unreachable but reached.")
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
