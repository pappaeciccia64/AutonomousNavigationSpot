#!/usr/bin/env python3
"""
Frontier goals (`run_mission.py --goals frontier`, mission/frontier.py) without the robot.

    python3 tests/test_frontier.py

  1. frontier_nodes on a small lattice with a fake global map: only the nodes seen, free,
     with an edge towards a node never seen;
  2. the route of the visit as a cost (PRM.set_route_line): zero on the line, growing with the
     distance, saturated; none without a line (the cells' path search is unchanged);
  3. the forced goal of goal.find_best_point_in_cell: the point once, then no goal;
  4. the whole fake mission of tests/test_mission.py (world A, in cruise) with --goals
     frontier: it ends by itself, sees most of the area, goes from one frontier goal to the
     next while walking, no traceback; the option is in the log.
"""
import contextlib
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), HERE]

import numpy as np                                                   # noqa: E402
from spot_explorer.mission import frontier                           # noqa: E402
from spot_explorer.planning import goal, roadmap                     # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


class FakeMap:
    """Seen: x < 2 m; occupied: the node at (1.0, 1.0)."""
    def known_many(self, xs, ys):
        return np.asarray(xs) < 2.0

    def is_known(self, x, y):
        return x < 2.0

    def occupied_many(self, xs, ys, margin):
        return (np.abs(np.asarray(xs) - 1.0) < 1e-6) & (np.abs(np.asarray(ys) - 1.0) < 1e-6)


class FakeEnv:
    rows, cols = 10, 10

    def get_cell_from_world(self, x, y):
        return int(round(y / 2.0)), int(round(x / 2.0))


def lattice():
    prm = roadmap.PRM()
    ids = {}
    for i in range(9):
        for j in range(5):
            ids[(i, j)] = len(ids)
            prm.add_node(ids[(i, j)], i * 0.5, j * 0.5)
    for (i, j), a in ids.items():
        for di, dj in ((1, 0), (0, 1)):
            b = ids.get((i + di, j + dj))
            if b is not None:
                prm.edges[a].append((b, 0.5))
                prm.edges[b].append((a, 0.5))
    return prm, ids


def units():
    print("== frontier nodes on a lattice, seen up to x = 2 m ==")
    prm, ids = lattice()
    f = frontier.frontier_nodes(prm, FakeMap(), FakeEnv())
    xs = sorted({prm.nodes[n][0] for n in f})
    check(xs == [1.5], f"frontier nodes on the last seen column x = 1.5 m ({xs})")
    check(ids[(2, 2)] not in f, "a node inside the seen area is not frontier")
    prm.edges[ids[(3, 1)]] = [(n, w if n != ids[(4, 1)] else float('inf')) for n, w in prm.edges[ids[(3, 1)]]]
    f2 = frontier.frontier_nodes(prm, FakeMap(), FakeEnv())
    check(ids[(3, 1)] not in f2, "a node whose only edge towards the unseen is vetoed is not frontier")

    print("== the route of the visit as a cost ==")
    prm, ids = lattice()
    check(prm._route_cost(ids[(4, 4)]) == 0.0, "no line: no route cost (the cells' path search)")
    prm.set_route_line([(0.0, 0.0), (4.0, 0.0)])
    on, half, far = prm._route_cost(ids[(4, 0)]), prm._route_cost(ids[(4, 1)]), prm._route_cost(ids[(4, 4)])
    check(on == 0.0 and abs(half - 0.5 * roadmap.ROUTE_WEIGHT) < 1e-9, f"0 on the line, 0.5 m away {half:.2f}")
    check(abs(far - roadmap.ROUTE_WEIGHT * roadmap.ROUTE_SATURATION_M) < 1e-9, f"2 m away saturated at {far:.2f}")
    prm.set_route_line(None)
    check(prm._route_cost(ids[(4, 4)]) == 0.0, "line removed: no route cost")

    print("== forced goal ==")
    class Env(FakeEnv):
        _forced_goal = (0, 1, 2.3, 0.4)
    gx, gy, ok, _ = goal.find_best_point_in_cell(0.0, 0.0, Env(), 0, 1, None, None, None)
    check((gx, gy) == (2.3, 0.4) and ok == [(2.3, 0.4)], f"the forced point is the goal ({gx}, {gy})")
    gx, _gy, _ok, _ = goal.find_best_point_in_cell(0.0, 0.0, Env(), 0, 1, None, None, None, exclude=[(2.3, 0.4)])
    check(gx is None, "already tried: no goal (the caller picks another one)")


def mission():
    print("== the whole fake mission, world A, in cruise, --goals frontier ==")
    import test_e2e_loop as T
    import test_mission as M
    with contextlib.redirect_stdout(io.StringIO()):
        r = M.run(walls=M.WORLDS['A'], name='frontier', cruise=True, extra_args=['--goals', 'frontier'])
    log = r['log']
    seen = [int(v) for v in re.findall(r'area seen (\d+)%', log)]
    goals = len(re.findall(r'\[FRONTIER\] Goal:', log))
    chained = len(re.findall(r'next cell while walking|done while walking', log))
    end = re.search(r'\[FRONTIER\] End of the exploration: ([^,]+)', log)
    track = np.array(T.ROBOT.track, dtype=float).reshape(-1, 2)
    check('Traceback' not in log, "no traceback in the log")
    check('[CONFIG] Goals: frontier' in log, "the option is in the log")
    check(end is not None, f"the exploration ends by itself ({end.group(1) if end else None})")
    check(seen and seen[-1] >= 90, f"area seen at the end {seen[-1] if seen else None}% (at least 90)")
    check(goals >= 10, f"frontier goals {goals}")
    check(chained >= 5, f"goals reached while walking, the next prepared in the planning thread ({chained})")
    check(len(track) > 1 and float(np.sum(np.hypot(*np.diff(track, axis=0).T))) > 10.0, "the robot walked")


def main():
    units()
    mission()
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
