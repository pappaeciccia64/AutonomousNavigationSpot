#!/usr/bin/env python3
"""
The cell_on_the_way variant: on a 2 x 3 lattice the cell crossed by the way is chosen first
(off: the visiting order); then the whole fake mission of tests/test_mission.py, world A, in
cruise, with the variant (paths also through the cells not explored).

    python3 varianti/variant_tests/test_cell_on_the_way.py
"""
import contextlib
import io
import os
import shutil
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]
os.chdir(ROOT)

import numpy as np                 # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    import test_mission as M       # noqa: E402
from spot_explorer.mission import mission_prm   # noqa: E402
import cell_on_the_way             # noqa: E402

check = M.check
ORIGINAL = mission_prm.planning_cells


def unit():
    """
    2 x 3 cells of 1 m, the robot in (0,0); (0,0), (1,0),
    (1,1) visited, so (0,1) and (0,2) are both open; the visiting order (0,2) before (0,1), the
    graph only along row 0: the way to (0,2) crosses (0,1). On: (0,1) first; off: (0,2), the visiting order. The goal of a cell: its node
    nearest to the centre (has_goal without map and sampler would not give the way).
    """
    print("== cell choice: the cell on the way first ==")
    from spot_explorer.mission import cells, cell_rules
    from spot_explorer.planning import roadmap
    env = cells.EnvironmentMap(rows=2, cols=3, cell_size=1.0)
    with contextlib.redirect_stdout(io.StringIO()):
        env.set_origin(0.0, 0.0, 0.0)
        for rc in ((0, 0), (1, 0), (1, 1)):
            env.mark_cell_visited(*rc)
    prm = roadmap.PRM()
    x0, y0 = env.get_world_position_from_cell(0, 0)
    x2, y2 = env.get_world_position_from_cell(0, 2)
    pts = [(x0 + t * (x2 - x0), y0 + t * (y2 - y0)) for t in np.linspace(0.0, 1.0, 9)]
    for i, (x, y) in enumerate(pts):
        prm.add_node(i, x, y)
    for i in range(len(pts) - 1):
        w = float(np.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]))
        prm.edges[i].append((i + 1, w))
        prm.edges[i + 1].append((i, w))

    class Rules(cell_rules.CellRules):
        def has_goal(self, start, from_xy, row, col, cells_of, global_map, sampler):
            cx, cy = self.env.get_world_position_from_cell(row, col)
            goal_n = min(self.prm.nodes, key=lambda n: np.hypot(self.prm.nodes[n][0] - cx, self.prm.nodes[n][1] - cy))
            dist, pred, _u = self._costs(start, row, col, from_xy, cells_of)
            if goal_n not in dist:
                return None
            path, u = [], goal_n
            while u is not None:
                path.append(u)
                u = pred[u]
            self.last_goal_path = path[::-1]
            return float(dist[goal_n])

    # Paths through the cells not explored, as the variant sets them (with the paths only
    # through the explored cells (0,2) has no way at all, (0,1) is chosen by the order).
    from hook import replace
    everywhere = replace(mission_prm, 'planning_cells', lambda *a, **k: None)
    try:
        for on, expected in ((True, (0, 1)), (False, (0, 2))):
            off = cell_on_the_way.apply() if on else (lambda: None)
            rules = Rules(env, prm, [(0, 0), (1, 0), (1, 1), (0, 2), (0, 1), (1, 2)])
            with contextlib.redirect_stdout(io.StringIO()):
                ch = rules.choose(pts[0], None, sampler=None)
            got = None if ch is None else (ch.row, ch.col)
            off()
            check(got == expected, f"variant {'on' if on else 'off'}: next cell {got} (expected {expected})")
    finally:
        everywhere()


def mission():
    print("== whole mission, world A, in cruise, with the variant ==")
    off = cell_on_the_way.apply()
    try:
        r = M.run(M.WORLDS['A'], "MISSION_A_cruise_through", cruise=True)
    finally:
        off()
    M._check_common(r)
    M._check_cruise_mission(r)
    print(f"  (cells chosen because on the way: {r['log'].count('on the way to')})")
    shutil.rmtree(r['root'], ignore_errors=True)
    check(mission_prm.planning_cells is ORIGINAL, "variant off: paths only through the explored cells again")


def main():
    unit()
    mission()
    fails = M.FAILS
    print(f"\n{'ALL OK' if not fails else f'{len(fails)} FAILED: ' + '; '.join(fails)}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
