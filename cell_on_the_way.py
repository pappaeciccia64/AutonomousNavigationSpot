"""
The cell on the way first (was cell_rules.EXPLORE_CELL_ON_THE_WAY): a near choice whose way to
the goal crosses a cell not explored yet takes that cell first, if it has a goal the attempt
would accept within cell_rules.NEAR_PATH_M: the robot is going through it anyway. It matters
only with paths also through the cells never visited (not only the explored ones),
which apply() sets too (mission_prm.planning_cells returns None: every cell).

    python3 varianti/run.py --with cell_on_the_way -- [arguments of run_mission.py]
"""
from spot_explorer.mission import cell_rules, mission_prm
from spot_explorer.records import MISSION_STATS
from hook import replace

DESCRIPTION = "the cell on the way first (paths also through cells not explored)"


def _near_choice(self, ch, start, robot_xy, robot_cell, cells_of, global_map, sampler, excluded):
    """The first cell crossed by the way to the goal of ch that is
    not explored yet (not done, not blocked, not excluded or set aside) and has a goal the
    attempt would accept within cell_rules.NEAR_PATH_M: chosen instead of ch. ch otherwise."""
    target = (ch.row, ch.col)
    crossed = []
    for n in self.last_goal_path:
        rc = cells_of.get(n)
        if rc is None or rc in (target, robot_cell) or rc in crossed:
            continue
        crossed.append(rc)
    for (r, c) in crossed:
        if (self.env.map[r][c] != 0 or (r, c) in excluded
                or len(self.failures.get((r, c), [])) >= cell_rules.MAX_FAILURES):
            continue
        way = self.way_into(start, robot_xy, r, c, cells_of)
        cost = self.has_goal(start, robot_xy, r, c, cells_of, global_map, sampler) if way else None
        if cost is None or max(way[0], cost) > cell_rules.NEAR_PATH_M:
            continue
        x = cell_rules.Choice(r, c, self.rank.get((r, c), 10 ** 6), max(way[0], cost), way[1], way[2], way[3],
                   self.prm.nodes[way[3]])
        print(f"[CELL RULES] Cell ({r},{c}) is on the way to ({ch.row},{ch.col}) and not explored yet: "
              f"that one first.")
        MISSION_STATS['cells_on_the_way'] = MISSION_STATS.get('cells_on_the_way', 0) + 1
        return x
    return ch


def apply():
    undo = replace(cell_rules.CellRules, '_near_choice', _near_choice)
    everywhere = replace(mission_prm, 'planning_cells', lambda *a, **k: None)

    def restore():
        everywhere()
        undo()
    return restore
