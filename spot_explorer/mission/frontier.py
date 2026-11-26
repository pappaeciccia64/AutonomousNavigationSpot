"""
Frontier goals: the mission option `--goals frontier` (the default is `--goals cells`).

The goal does not come from the next cell of the visit: one Dijkstra search from the robot
stops at the first frontier node it settles, i.e. the frontier node that is cheapest to
reach. The search uses the usual costs (edges, the obstacle cost of the nodes) plus the
distance from the route of the visit (PRM.set_route_line: the cell centres in visiting
order), so that the sweep stays orderly. Frontier node: a node of the graph inside the mission
area, seen free in the global map, with a valid edge towards a node never seen. Being seen,
its distance to the obstacles is known: the cheapest frontier node is not one squeezed
against a box.

The attempt is the one of the cells (cell_attempt -> cruise) towards the cell that contains
the node, with the goal forced on the node (env._forced_goal, read by
goal.find_best_point_in_cell). FrontierRules has the choose() of cell_rules.CellRules: in
cruise the next goal is prepared by the planning thread as the next cell is, and the robot
goes from one frontier goal to the next without stopping. The cells are only containers:
the rules made for them (a cell done when seen, paths limited to the explored cells) do not
apply (env.goal_mode == 'frontier', read by cells.cell_seen_fraction and
mission_prm.planning_cells).

End: no frontier node reachable, or the mission area seen above FRONTIER_END_SEEN.
With Spot's local grids (~4 m) the frontier is ~2 m ahead: short goals. With a wider window
(a lidar, a simulator) the goals get longer.
"""
import time

import numpy as np

from spot_explorer import config as P
from spot_explorer.mission import attempt_common, attempt_cruise, cell_rules, exploration
from spot_explorer.perception import geometry
from spot_explorer.planning import roadmap
from spot_explorer.records import MISSION_STATS
from spot_explorer.robot import connection

FRONTIER_END_SEEN = 0.97          # mission area seen: the exploration ends
FRONTIER_MIN_DIST_M = 0.75        # a frontier node nearer than this to the robot is not a goal
FRONTIER_REFUSED_RADIUS_M = 0.6   # around a goal that failed, no goal for the rest of the mission
FRONTIER_MAX_FAILURES = 6         # failed goals in a row: the exploration ends
# ADD 26-11: goal beyond the edge (--frontier-beyond B). 0 = off: the goal is the frontier node
# (seen, free, next to the unseen). B > 0: the goal is the node up to B m beyond it, into the
# unseen, furthest along the direction robot -> frontier node, reached through valid edges and
# not near a refused goal. Longer goals, fewer changes of goal: each change can turn the robot
# round (the path starts from the end of the edge being walked). Tried on 20-11 with fake missions
# (tools/compare_route_line.py, then in varianti/): 1.0 m = 40% fewer goals, none failed, same
# area and time; 1.5 m = goals inside obstacles not seen yet, refused.
FRONTIER_BEYOND_M = 0.0


def _inside(env, x, y):
    r, c = (int(v) for v in env.get_cell_from_world(x, y))
    return 0 <= r < env.rows and 0 <= c < env.cols


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


class FrontierRules:
    """The goal choice of the frontier goals, with the choose() of cell_rules.CellRules (used by
    the mission and, in cruise, by the planning thread to prepare the next goal)."""

    def __init__(self, env, prm):
        self.env, self.prm = env, prm
        self.refused = []                  # (x, y) of the goals that failed
        self.last = None                   # (node, x, y, cost, frontier nodes) of the last choice

    def refuse(self, x, y):
        self.refused.append((float(x), float(y)))

    def choose(self, robot_xy, global_map, sampler=None, waypoints=None, exclude=()):
        """The cell of the first frontier node a Dijkstra search from the robot settles, as a
        cell_rules.Choice, its goal forced on the node; None if none."""
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
        prm.set_allowed_nodes(None)            # paths not limited to explored cells
        dist, _parents, end = prm.costs_from(start, stop=lambda u: u in front, route=True)
        if end is None:
            return None
        frontier_node = end
        beyond = self._beyond(end, x, y, global_map) if FRONTIER_BEYOND_M > 0 else None  # ADD 26-11
        if beyond is not None:
            end = beyond
        gx, gy = (float(v) for v in prm.nodes[end])
        row, col = (int(v) for v in env.get_cell_from_world(gx, gy))
        env._forced_goal = (row, col, gx, gy)
        self.last = (end, gx, gy, float(dist[frontier_node]), len(front))
        MISSION_STATS['frontier_goals'] = MISSION_STATS.get('frontier_goals', 0) + 1
        fx, fy = (float(v) for v in prm.nodes[frontier_node])
        print(f"[FRONTIER] Goal: node {end} at ({gx:.2f}, {gy:.2f}) in cell ({row},{col}), cost "
              f"{dist[frontier_node]:.2f} from the robot, {prm.route_distance(frontier_node):.2f} m from the route; "
              f"{len(front)} frontier nodes"
              + (f"; {float(np.hypot(gx - fx, gy - fy)):.2f} m beyond the frontier node {frontier_node} "
                 f"({fx:.2f}, {fy:.2f}), not seen yet" if beyond is not None else "") + ".")
        return cell_rules.Choice(row, col, 0, dist[frontier_node], None, None, end, (gx, gy))

    def _beyond(self, start, rx, ry, global_map):  # ADD 26-11: goal beyond the edge (FRONTIER_BEYOND_M)
        """The node up to FRONTIER_BEYOND_M beyond the frontier node start, into the unseen: reached
        from start through valid edges (finite weight) over nodes never seen, inside the mission
        grid, not near a refused goal; the one furthest along the direction robot -> start.
        None if there is none (the frontier node stays the goal)."""
        prm, env = self.prm, self.env
        sx, sy = (float(v) for v in prm.nodes[start])
        dx, dy = sx - rx, sy - ry
        n = float(np.hypot(dx, dy))
        if n < 1e-6 or global_map is None:
            return None
        ux, uy = dx / n, dy / n
        best, best_along = None, 0.0
        todo, seen = [start], {start}
        while todo:
            u = todo.pop()
            for v, w in prm.edges.get(u, ()):
                if v in seen or not np.isfinite(w):
                    continue
                vx, vy = (float(c) for c in prm.nodes[v])
                if (float(np.hypot(vx - sx, vy - sy)) > FRONTIER_BEYOND_M or not _inside(env, vx, vy)
                        or global_map.is_known(vx, vy)
                        or any(float(np.hypot(vx - qx, vy - qy)) <= FRONTIER_REFUSED_RADIUS_M for qx, qy in self.refused)):
                    continue
                seen.add(v)
                todo.append(v)
                along = (vx - sx) * ux + (vy - sy) * uy
                if along > best_along:
                    best, best_along = v, along
        return best

    def explain_end(self, attempts):
        pass


class FrontierMission(exploration.Mission):
    """exploration.Mission with the goals from the frontier (see the module docstring)."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.prm.set_route_line([self.env.get_world_position_from_cell(r, c) for r, c in self.path])

    def explore(self):
        self.env.goal_mode = 'frontier'
        print(f"[FRONTIER] Goals from the frontier: the cheapest frontier node of a Dijkstra search "
              f"from the robot, with the distance from the route (weight {roadmap.ROUTE_WEIGHT}, "
              f"saturated at {roadmap.ROUTE_SATURATION_M} m); end at {100 * FRONTIER_END_SEEN:.0f}% "
              f"of the area seen.")
        try:
            self._explore_frontier()
        finally:
            self.env._forced_goal = None

    def _explore_frontier(self):
        rules = self.rules = FrontierRules(self.env, self.prm)
        self.env._cell_rules = rules              # in cruise: the next goal while walking
        self.env._on_chained_visit = self._on_visited
        self._t0 = time.time()
        self._closed_at_start = self._cell_counts()[0]
        failures, why_end = 0, None
        self._first_scan()
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
        attempt_cruise.stop_if_moving(self.env, "end of the exploration")
        MISSION_STATS['frontier_end'] = why_end
        print(f"[FRONTIER] End of the exploration: {why_end}, after "
              f"{MISSION_STATS.get('frontier_goals', 0)} goals ({MISSION_STATS.get('frontier_goals_failed', 0)} failed).")
        self._log_progress(final=True)

    def _first_scan(self):
        """No global map or no edge yet (start of the mission): one attempt on the cell the robot
        is in, with the goal on the robot itself -- the first scan, the global maps and the edges
        are made by the attempt as with the cells, then it ends at once (arrived)."""
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


GOAL_MODES = {'cells': exploration.Mission, 'frontier': FrontierMission}
