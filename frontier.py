"""
Exploration by frontier, an alternative to the cells (to compare on fake missions with
varianti/tools/compare_route_line.py, and for a future lidar). Turning it on replaces
exploration.Mission with FrontierMission and turns the route_line variant on.

The goal does not come from the next cell: one Dijkstra search from the robot, with the
distance from the route of the visit as a node cost (the route_line variant), stops at the first frontier node it settles -- the one that minimizes the way there
plus the distance from the route. Frontier node: a node of the graph inside the mission
area, seen free in the global map, with a valid edge towards a node never seen.

The attempt is the one of the cells (cell_attempt), towards the cell that contains the node,
with the goal forced on the node (env._forced_goal, read by the wrapped
goal.find_best_point_in_cell).
FrontierRules has the choose() of cell_rules.CellRules: in cruise the next goal is prepared
by the planning thread as the next cell is (attempt_cruise._prepare_next_cell), and the robot
goes from one frontier goal to the next without stopping. The rules made for the cells are
off for the whole mode (a frontier node lies by definition at the border of what is seen, so
its cell would be "seen" already): the cell done when seen, the cells closed before moving,
the paths limited to the explored cells.

End: no frontier node reachable, or the mission area seen above FRONTIER_END_SEEN.
The progress is the area seen ([PROGRESS] ... area seen X%), the cells are only containers.

With Spot's local grids (~4 m) the frontier is ~2 m ahead: short goals, many attempts. With
a wider local window (a lidar) the goals get longer and the choice less short-sighted.
"""
import time

import numpy as np

from spot_explorer import config as P
from spot_explorer.mission import attempt_common, attempt_cruise, cell_rules, cells, exploration, mission_prm
from spot_explorer.perception import geometry
from spot_explorer.planning import goal, roadmap
from spot_explorer.records import MISSION_STATS
from spot_explorer.robot import connection
from hook import chain, replace
import route_line

DESCRIPTION = "exploration by frontier nodes instead of cells (with route_line)"

FRONTIER_END_SEEN = 0.97          # mission area seen: the exploration ends
FRONTIER_MIN_DIST_M = 0.75        # a frontier node nearer than this to the robot is not a goal
FRONTIER_REFUSED_RADIUS_M = 0.6   # around a goal that failed, no goal for the rest of the mission
FRONTIER_MAX_FAILURES = 6         # failed goals in a row: the exploration ends
# Goal beyond the edge (0 = on the frontier node): from the frontier node, the node up to this
# far into the unseen area, through valid edges, furthest along the direction robot -> frontier
# node. The path is planned through the unseen (assumed free, edges at their length), the safe
# frontier lets the robot walk only what is confirmed free; a goal found in an obstacle is
# refused by the attempt and another one chosen. Longer goals: fewer slow-downs at the end of
# every command (with Spot's 3.84 m grid the frontier is ~1 m ahead).
FRONTIER_BEYOND_M = 0.0


def frontier_nodes(prm, gmap, env):
    """The frontier nodes: inside the mission grid, seen free, with a valid edge (finite
    weight) towards a node never seen."""
    ids = [i for i in prm.nodes if _inside(env, *prm.nodes[i])]
    if not ids or gmap is None:
        return set()
    xy = np.array([prm.nodes[i] for i in ids], dtype=float)
    known = dict(zip(ids, gmap.known_many(xy[:, 0], xy[:, 1])))
    occ = dict(zip(ids, gmap.occupied_many(xy[:, 0], xy[:, 1], 0.0)))
    out = set()
    for i in ids:
        if not known[i] or occ[i]:
            continue
        for j, w in prm.edges.get(i, ()):
            if np.isfinite(w) and not known.get(j, gmap.is_known(*prm.nodes[j])):
                out.add(i)
                break
    return out


def _inside(env, x, y):
    r, c = (int(v) for v in env.get_cell_from_world(x, y))
    return 0 <= r < env.rows and 0 <= c < env.cols


class FrontierRules:
    """The goal choice of the frontier mode, with the choose() of cell_rules.CellRules (used by
    the mission and, in cruise, by the planning thread to prepare the next goal)."""

    def __init__(self, env, prm, robot_state_client=None):
        self.env, self.prm = env, prm
        self.refused = []                  # (x, y) of the goals that failed
        self.last = None                   # (node, x, y, cost, frontier nodes) of the last choice

    def refuse(self, x, y):
        self.refused.append((float(x), float(y)))

    def choose(self, robot_xy, global_map, sampler=None, waypoints=None, exclude=()):
        """The cell of the first frontier node a Dijkstra search from the robot settles (with the
        route penalty), as a cell_rules.Choice, its goal forced on the node; None if none."""
        prm, env = self.prm, self.env
        x, y = float(robot_xy[0]), float(robot_xy[1])
        excl = {tuple(int(v) for v in c) for c in (exclude or ())}

        def usable(i):
            nx, ny = prm.nodes[i]
            if tuple(int(v) for v in env.get_cell_from_world(nx, ny)) in excl:
                return False
            return (float(np.hypot(nx - x, ny - y)) >= FRONTIER_MIN_DIST_M
                    and all(float(np.hypot(nx - rx, ny - ry)) > FRONTIER_REFUSED_RADIUS_M for rx, ry in self.refused))
        front = {i for i in frontier_nodes(prm, global_map, env) if usable(i)}
        if not front:
            return None
        start = prm.anchor_node(x, y, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M, connected_first=True)
        if start is None:
            return None
        prm.set_allowed_nodes(None)            # paths not limited to explored cells in this mode
        dist, _parents, end = prm.costs_from(start, stop=lambda u: u in front, route=True)
        if end is None:
            return None
        goal, beyond = end, 0.0
        if FRONTIER_BEYOND_M > 0:
            goal = self._beyond(end, (x, y), global_map)
            beyond = float(np.hypot(*(np.asarray(prm.nodes[goal]) - np.asarray(prm.nodes[end]))))
        gx, gy = (float(v) for v in prm.nodes[goal])
        row, col = (int(v) for v in env.get_cell_from_world(gx, gy))
        env._forced_goal = (row, col, gx, gy)
        self.last = (goal, gx, gy, float(dist[end]) + beyond, len(front))
        MISSION_STATS['frontier_goals'] = MISSION_STATS.get('frontier_goals', 0) + 1
        print(f"[FRONTIER] Goal: node {goal} at ({gx:.2f}, {gy:.2f}) in cell ({row},{col})"
              + (f", {beyond:.2f} m beyond the frontier node {end}" if goal != end else "")
              + f", cost {dist[end]:.2f} from the robot to the frontier, "
              f"{route_line.route_distance(prm, goal):.2f} m from the route; {len(front)} frontier nodes.")
        return cell_rules.Choice(row, col, 0, dist[end] + beyond, None, None, goal, (gx, gy))

    def _beyond(self, f, robot_xy, global_map):
        """FRONTIER_BEYOND_M: from the frontier node f, the node reachable through valid edges
        and unseen nodes, within FRONTIER_BEYOND_M of f and inside the mission area, furthest
        along the direction robot -> f (ties: nearer to the route); f itself if none."""
        prm, env = self.prm, self.env
        fx, fy = prm.nodes[f]
        d = np.array([fx - robot_xy[0], fy - robot_xy[1]], dtype=float)
        n = float(np.hypot(*d))
        if n < 1e-6:
            return f
        d /= n
        best, best_key = f, (0.0, 0.0)
        seen, todo = {f}, [f]
        while todo:
            u = todo.pop()
            for v, w in prm.edges.get(u, ()):
                if v in seen or not np.isfinite(w):
                    continue
                seen.add(v)
                vx, vy = prm.nodes[v]
                if (float(np.hypot(vx - fx, vy - fy)) > FRONTIER_BEYOND_M or not _inside(env, vx, vy)
                        or global_map.is_known(vx, vy)
                        or any(float(np.hypot(vx - rx, vy - ry)) <= FRONTIER_REFUSED_RADIUS_M for rx, ry in self.refused)):
                    continue
                todo.append(v)
                rd = route_line.route_distance(prm, v)
                key = (float((vx - fx) * d[0] + (vy - fy) * d[1]), -rd if np.isfinite(rd) else 0.0)
                if key > best_key:
                    best, best_key = v, key
        return best

    def explain_end(self, attempts):
        pass


class FrontierMission(exploration.Mission):
    """exploration.Mission with the goal from the frontier (see the module docstring)."""

    def explore(self):
        never_seen = replace(cells, 'cell_seen_fraction', lambda *a, **k: None)   # no cell rules here
        everywhere = replace(mission_prm, 'planning_cells', lambda *a, **k: None)
        print(f"[FRONTIER] Exploration by frontier: goal = the first frontier node of a Dijkstra search "
              f"from the robot with the distance from the route (weight {route_line.WEIGHT}, "
              f"saturated at {route_line.SATURATION_M} m); end at {100 * FRONTIER_END_SEEN:.0f}% "
              f"of the area seen. Cell rules off.")
        try:
            self._explore_frontier()
        finally:
            never_seen()
            everywhere()
            self.env._forced_goal = None

    def _explore_frontier(self):
        rules = self.rules = FrontierRules(self.env, self.prm)
        self.env._cell_rules = rules              # in cruise: the next goal while walking
        self.env._on_chained_visit = self._on_visited
        self._t0 = time.time()
        self._closed_at_start = self._cell_counts()[0]
        failures, why_end = 0, None
        self._bootstrap()
        while True:
            self._log_progress()
            if attempt_common.battery_return_due():
                why_end = "low battery"
                break
            seen = self._seen_area_pct()
            if seen is not None and seen >= FRONTIER_END_SEEN:
                why_end = f"area seen {100 * seen:.0f}% (threshold {100 * FRONTIER_END_SEEN:.0f}%)"
                break
            x, y, _, _ = connection.getPosition(self.robot_state_client)
            choice = rules.choose((x, y), self.global_grid.global_occupancy_map)
            if choice is None:
                why_end = "no frontier node reachable"
                break
            self.env._chain_choice = None
            self.env._attempt_left_unreachable = False
            self.env._attempt_fail_why = None
            ok = self._attempt(choice.row, choice.col)
            # In cruise the robot may have gone on through other frontier goals: the last one.
            last = getattr(self.env, '_forced_goal', None) or (choice.row, choice.col) + choice.node_xy
            self.env._forced_goal = None
            row, col, gx, gy = last
            x, y, _, _ = connection.getPosition(self.robot_state_client)
            if ok or float(np.hypot(x - gx, y - gy)) <= P.GOAL_REACHED_TOLERANCE_M:
                failures = 0
                self._on_visited(row, col, x, y)
            else:
                failures += 1
                rules.refuse(gx, gy)
                MISSION_STATS['frontier_goals_failed'] = MISSION_STATS.get('frontier_goals_failed', 0) + 1
                print(f"[FRONTIER] Goal ({gx:.2f}, {gy:.2f}) not reached "
                      f"({getattr(self.env, '_attempt_fail_why', None) or 'attempt failed'}): no goal within "
                      f"{FRONTIER_REFUSED_RADIUS_M} m of it for the rest of the mission ({failures} failed in a row).")
                if failures >= FRONTIER_MAX_FAILURES:
                    why_end = f"{failures} goals failed in a row"
                    break
        attempt_cruise.stop_if_moving(self.env, self.command_client, "end of the exploration")
        MISSION_STATS['frontier_end'] = why_end
        print(f"[FRONTIER] End of the exploration: {why_end}, after "
              f"{MISSION_STATS.get('frontier_goals', 0)} goals ({MISSION_STATS.get('frontier_goals_failed', 0)} failed).")
        self._log_progress(final=True)

    def _bootstrap(self):
        """No global map or no edge yet (start of the mission): one attempt on the cell the robot
        is in, with the goal on the robot itself -- the first scan, the global maps and the edges
        are made by the attempt as in the cell mode, then it ends at once (arrived)."""
        if self.global_grid.global_occupancy_map is not None and any(self.prm.edges.values()):
            return
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        row, col = (int(v) for v in self.env.get_cell_from_world(x, y))
        print(f"[FRONTIER] First scan: attempt on the cell of the robot ({row},{col}), goal on the robot.")
        self.env._forced_goal = (row, col, float(x), float(y))
        try:
            self._attempt(row, col)
        finally:
            self.env._forced_goal = None


def apply():
    best = goal.find_best_point_in_cell

    def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, *a, exclude=None, **k):
        """The goal of the cell is the frontier node (env._forced_goal), nothing else; once tried in
        the attempt or refused for the mission, no goal (the attempt fails and the frontier mode
        picks another node). Other cells: the normal choice."""
        forced = getattr(env, '_forced_goal', None)
        if forced is not None and (int(forced[0]), int(forced[1])) == (int(cell_row), int(cell_col)):
            fp = (float(forced[2]), float(forced[3]))
            seen_before = list(exclude or []) + list(goal._refused_goals(env, cell_row, cell_col))
            if any(np.hypot(fp[0] - ex, fp[1] - ey) < P.RECOVERY_SAME_GOAL_M for ex, ey in seen_before):
                return None, None, [], [fp]
            return fp[0], fp[1], [fp], []
        return best(robot_x, robot_y, env, cell_row, cell_col, *a, exclude=exclude, **k)

    return chain(route_line.apply(),
                 replace(exploration, 'Mission', FrontierMission),
                 replace(goal, 'find_best_point_in_cell', find_best_point_in_cell))
