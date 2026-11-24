#!/usr/bin/env python3
"""
The frontier variant without the robot.

    python3 varianti/variant_tests/test_frontier.py

  1. frontier_nodes on a small lattice with a fake global map: only the nodes seen, free,
     with an edge towards a node never seen;
  2. the whole fake mission of tests/test_mission.py (world A, in cruise): it ends by itself,
     sees most of the area, goes from a frontier goal to the next while walking, no
     traceback, and the switches of the cell mode are back as they were at the end; with the
     variant off the mission is exploration.Mission again.
"""
import contextlib
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(os.path.dirname(HERE)), os.path.dirname(HERE),
               os.path.join(os.path.dirname(os.path.dirname(HERE)), 'tests')]

import numpy as np                                                   # noqa: E402
from spot_explorer.mission import cells, exploration, mission_prm   # noqa: E402
from spot_explorer.planning import roadmap                           # noqa: E402
import frontier                                                      # noqa: E402

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


def frontier_unit():
    print("== frontier nodes on a lattice, seen up to x = 2 m ==")
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
    f = frontier.frontier_nodes(prm, FakeMap(), FakeEnv())
    xs = sorted({prm.nodes[n][0] for n in f})
    check(xs == [1.5], f"frontier nodes on the last seen column x = 1.5 m ({xs})")
    check(ids[(2, 2)] not in f, "a node inside the seen area is not frontier")
    prm.edges[ids[(3, 1)]] = [(n, w if n != ids[(4, 1)] else float('inf')) for n, w in prm.edges[ids[(3, 1)]]]
    f2 = frontier.frontier_nodes(prm, FakeMap(), FakeEnv())
    check(ids[(3, 1)] not in f2, "a node whose only edge towards the unseen is vetoed is not frontier")

    print("== goal beyond the frontier (FRONTIER_BEYOND_M) ==")
    rules = frontier.FrontierRules.__new__(frontier.FrontierRules)
    rules.prm, rules.env, rules.refused = prm, FakeEnv(), []
    saved = frontier.FRONTIER_BEYOND_M
    try:
        frontier.FRONTIER_BEYOND_M = 1.0
        g = rules._beyond(ids[(3, 2)], (0.0, 1.0), FakeMap())
        check(tuple(prm.nodes[g]) == (2.5, 1.0), f"1 m beyond the frontier node, straight on ({tuple(prm.nodes[g])})")
        frontier.FRONTIER_BEYOND_M = 0.4
        g = rules._beyond(ids[(3, 2)], (0.0, 1.0), FakeMap())
        check(g == ids[(3, 2)], "no node within 0.4 m beyond: the goal stays on the frontier node")
        frontier.FRONTIER_BEYOND_M = 1.0
        rules.refused = [(3.0, 1.0)]
        g = rules._beyond(ids[(3, 2)], (0.0, 1.0), FakeMap())
        check(prm.nodes[g][0] == 2.0, f"no goal near a goal refused before ({tuple(prm.nodes[g])})")
        rules.refused = []
        f, n = ids[(3, 2)], ids[(4, 2)]
        prm.edges[f] = [(m, float('inf') if m == n else w) for m, w in prm.edges[f]]
        g = rules._beyond(f, (0.0, 1.0), FakeMap())
        check(g == f, "the only edge into the unseen vetoed: the goal stays on the frontier node")
    finally:
        frontier.FRONTIER_BEYOND_M = saved


def frontier_mission():
    print("== the whole fake mission, world A, in cruise ==")
    import test_mission as M
    import test_e2e_loop as T
    saved = (cells.cell_seen_fraction, mission_prm.planning_cells)
    off = frontier.apply()
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            r = M.run(walls=M.WORLDS['A'], name='frontier', cruise=True)
    finally:
        off()
    log = r['log']
    seen = [int(v) for v in re.findall(r'area seen (\d+)%', log)]
    goals = len(re.findall(r'\[FRONTIER\] Goal:', log))
    chained = len(re.findall(r'next cell while walking|done while walking', log))
    end = re.search(r'\[FRONTIER\] End of the exploration: ([^,]+)', log)
    track = np.array(T.ROBOT.track, dtype=float).reshape(-1, 2)
    check('Traceback' not in log, "no traceback in the log")
    check(end is not None, f"the exploration ends by itself ({end.group(1) if end else None})")
    check(seen and seen[-1] >= 90, f"area seen at the end {seen[-1] if seen else None}% (at least 90)")
    check(goals >= 10, f"frontier goals {goals}")
    check(chained >= 5, f"goals reached while walking, the next prepared in the planning thread ({chained})")
    check(len(track) > 1 and float(np.sum(np.hypot(*np.diff(track, axis=0).T))) > 10.0, "the robot walked")
    check((cells.cell_seen_fraction, mission_prm.planning_cells) == saved,
          "the switches of the cell mode back as they were")
    check(exploration.Mission.__name__ == 'Mission', "variant off: exploration.Mission again")


def main():
    frontier_unit()
    frontier_mission()
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
