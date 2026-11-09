"""
Which cell next: the general rules of 09-11 (switch CELL_RULES), instead of the sides tried,
the deferral, SKIP-ROUND, PRECHECK and the cells dropped from the frontier.

A cell is
  done            seen for CELL_SEEN_FRACTION, or its goal reached (map value 1);
  blocked         every candidate point inside an obstacle already seen (value -1);
  set aside       it failed MAX_FAILURES times and the way into it today is one that failed;
  candidate       none of the above, and the graph of now has a way from the robot into it;
  unreachable     none of the above, and no way.
Set aside and unreachable are not states of the map: they are worked out again at every
choice, so a cell comes back as soon as the graph has a (new) way into it.

The way into a cell: a path on the graph of now (valid edges: in the scan window the
frontier's rule, outside the global map), from the robot node to the first node of the cell,
through the cells the attempt itself may use (mission_prm.planning_cells: visited, the
robot's, the target and the shared neighbours). Its ENTRY is the cell the way comes from when
it crosses into the cell (one of the 8 around it; None if the robot is already in it): it
replaces the four sides. Not the edge itself: a cell is entered by dozens of edges, and on the
forced failures of test_mission one cell failed 3 times by 3 edges from the same side.

The choice: among the candidates whose way is at most NEAR_PATH_M long, the first in the
visiting order, reached in cruise; if none is that near, the first in the visiting order
among all the candidates, reached with GraphNav (the exception, with a stop). A cell that
already failed comes after all the others (in the visiting order among the failed ones):
its second try comes later, from where the robot is then, not at once from the same side
(04-11 14:08: (0,5) failed in the tall grass, the mission tried it again 17 attempts later
from another side, and it worked).
"""
import heapq

import numpy as np

from spot_explorer import config as P
from spot_explorer.mission import mission_prm
from spot_explorer.mission import recovery
from spot_explorer.perception import geometry
from spot_explorer.planning import goal as goal_choice
from spot_explorer.planning import roadmap
from spot_explorer.records import MISSION_STATS

CELL_RULES = True          # False = the rules before 09-11 (exploration.Mission._explore_sides)
NEAR_PATH_M = 4.0          # a way at most this long is walked in cruise; longer, GraphNav
MAX_FAILURES = 2           # real failures (the robot went and stopped) before setting aside


class Choice:
    """The next cell: where, how far, the entry edge, the node it is reached at."""
    __slots__ = ('row', 'col', 'rank', 'length', 'entry', 'edge', 'node', 'node_xy', 'near', 'waypoint')

    def __init__(self, row, col, rank, length, entry, edge, node, node_xy):
        self.row, self.col, self.rank = int(row), int(col), int(rank)
        self.length, self.entry, self.edge = float(length), entry, edge
        self.node, self.node_xy = node, node_xy
        self.near = self.length <= NEAR_PATH_M
        self.waypoint = None        # far: (cell, data) of the GraphNav waypoint to start from

    def describe_entry(self):
        if self.entry is None:
            return "the robot in the cell"
        return f"entering from cell {self.entry} (edge {self.edge[0]}->{self.edge[1]})"

    def describe(self):
        how = "in cruise" if self.near else "with GraphNav"
        return f"cell ({self.row},{self.col}), way of {self.length:.1f} m, {self.describe_entry()}: {how}"


class CellRules:
    """The cell rules of a mission: failures per cell and the choice (see the module)."""

    def __init__(self, env, prm, path):
        self.env, self.prm = env, prm
        self.rank = {tuple(c): i for i, c in enumerate(path)}
        self.failures = {}          # (r, c) -> [entry, ...] of the real failures
        self.status = {}            # (r, c) -> why it is not a candidate now (for the log)
        # Cells left because unreachable now (no goal with a path): not failed, skipped at the
        # next choice only (rule 3: not a candidate NOW; it comes back when the graph changes).
        self.skip_once = set()

    # --- the cells to look at ------------------------------------------------------------
    def open_cells(self, robot_cell=None):
        """The cells not done and not blocked that a way could reach: next to (8 around) a
        visited cell or to the robot's cell. In visiting order."""
        env = self.env
        near = set()
        seeds = [(r, c) for r in range(env.rows) for c in range(env.cols) if env.map[r][c] == 1]
        if robot_cell is not None:
            seeds.append(tuple(robot_cell))
        for r0, c0 in seeds:
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    r, c = r0 + dr, c0 + dc
                    if 0 <= r < env.rows and 0 <= c < env.cols and env.map[r][c] == 0:
                        near.add((r, c))
        return sorted(near, key=lambda rc: self.rank.get(rc, 10 ** 6))

    # --- the way into a cell -------------------------------------------------------------
    def way_into(self, start, robot_xy, row, col, cells_of):
        """(length, entry cell, entry edge, node) of the shortest way from node start into cell
        (row, col) on the graph of now, None if there is none. length = metres to the first node of the
        cell plus the straight line from there to the centre (where the goal will be)."""
        prm, env = self.prm, self.env
        target = (int(row), int(col))
        allowed = (mission_prm.planning_cells(env, row, col, robot_xy)
                   if P.PLAN_ONLY_EXPLORED_CELLS else None)
        centre = env.get_world_position_from_cell(row, col)
        dist, pred, heap = {start: 0.0}, {start: None}, [(0.0, start)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > dist.get(u, np.inf):
                continue
            if cells_of.get(u) == target and np.isfinite(prm.node_penalty(u)):
                p = pred[u]
                edge = None if p is None else (p, u)
                entry = None if p is None else cells_of.get(p)
                x, y = prm.nodes[u]
                return d + float(np.hypot(x - centre[0], y - centre[1])), entry, edge, u
            xu, yu = prm.nodes[u]
            for v, _w in prm.edges.get(u, ()):
                if allowed is not None and cells_of.get(v) not in allowed:
                    continue
                if not np.isfinite(prm.node_penalty(v)):
                    continue
                xv, yv = prm.nodes[v]
                nd = d + float(np.hypot(xv - xu, yv - yu))
                if nd < dist.get(v, np.inf):
                    dist[v], pred[v] = nd, u
                    heapq.heappush(heap, (nd, v))
        return None

    def has_goal(self, start, from_xy, row, col, cells_of, global_map, sampler):
        """Is there a goal of the cell that the attempt starting at from_xy (node start)
        would accept? The SAME choice as the attempt (goal.find_best_point_in_cell, on the
        global map: no live scan here), with "reachable" = in the graph from start through
        the cells the attempt may use. On 09-11 14:38 the rules saw a way into (1,5) (a node
        of the cell) and the robot went there with GraphNav (22 s) to find no goal reachable."""
        if global_map is None or sampler is None:
            return True
        prm, env = self.prm, self.env
        allowed = (mission_prm.planning_cells(env, row, col, from_xy)
                   if P.PLAN_ONLY_EXPLORED_CELLS else None)
        seen, stack = {start}, [start]
        while stack:
            u = stack.pop()
            for v, _w in prm.edges.get(u, ()):
                if v in seen or (allowed is not None and cells_of.get(v) not in allowed):
                    continue
                if not np.isfinite(prm.node_penalty(v)):
                    continue
                seen.add(v)
                stack.append(v)

        def reachable(px, py):
            nid = prm.anchor_node(px, py, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M)
            return nid is not None and nid in seen
        gx, _gy, _v, _b = goal_choice.find_best_point_in_cell(
            from_xy[0], from_xy[1], env, row, col, None, None, sampler, global_map=global_map,
            reachable=reachable, quiet=True, require_reachable=True)
        return gx is not None

    # --- the choice ----------------------------------------------------------------------
    def choose(self, robot_xy, global_map, sampler=None, waypoints=None, exclude=()):
        """The next cell (Choice), None if there is no candidate. Cells found with every
        candidate point inside an obstacle are marked blocked on the way. A candidate needs a
        goal the attempt would accept (has_goal): from the robot if near, from the GraphNav
        waypoint nearest to where the way enters the cell if far (waypoints: {cell: data
        with x, y}, see map_recording.get_all_manual_waypoints_with_cells; None = not checked)."""
        env, prm = self.env, self.prm
        self.status = {}
        robot_cell = tuple(int(v) for v in env.get_cell_from_world(*robot_xy))
        start = prm.anchor_node(float(robot_xy[0]), float(robot_xy[1]), global_map,
                                geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M, connected_first=True)
        if start is None:
            return None
        cells_of = mission_prm.node_cells(prm, env)
        if not any(prm.edges.values()):
            # Mission start: the edges are created at the first scan of an attempt. The first
            # cell in the visiting order next to the robot, from here.
            first = next((rc for rc in self.open_cells(robot_cell)
                          if len(self.failures.get(rc, [])) < MAX_FAILURES), None)
            if first is None:
                return None
            return Choice(first[0], first[1], self.rank.get(first, 10 ** 6), 0.0, None, None, start,
                          prm.nodes[start])
        far = None
        excluded = set(map(tuple, exclude)) | self.skip_once
        self.skip_once = set()
        order = sorted((rc for rc in self.open_cells(robot_cell) if rc not in excluded),
                       key=lambda rc: (len(self.failures.get(rc, [])) > 0, self.rank.get(rc, 10 ** 6)))
        far_failed = None
        for (r, c) in order:
            way = self.way_into(start, robot_xy, r, c, cells_of)
            if way is None:
                if global_map is not None and sampler is not None:
                    ok, _f, _u, _o = recovery.cell_has_reachable_target(env, sampler, global_map, r, c)
                    if not ok:
                        env.mark_cell_blocked(r, c)
                        print(f"[CELL RULES] Cell ({r},{c}) blocked: every candidate point is inside "
                              f"an obstacle seen in the global map.")
                        continue
                self.status[(r, c)] = "unreachable: no way in the graph of now"
                continue
            length, entry, edge, node = way
            failed = self.failures.get((r, c), [])
            if len(failed) >= MAX_FAILURES and entry in failed:
                self.status[(r, c)] = (f"set aside: failed {len(failed)} times, the way of now enters "
                                       f"where it already failed")
                continue
            ch = Choice(r, c, self.rank.get((r, c), 10 ** 6), length, entry, edge, node, prm.nodes[node])
            if ch.near:
                ok = self.has_goal(start, robot_xy, r, c, cells_of, global_map, sampler)
            elif waypoints:
                nx, ny = ch.node_xy
                ch.waypoint = min(waypoints.items(), key=lambda kv: np.hypot(kv[1]['x'] - nx, kv[1]['y'] - ny))
                wxy = (float(ch.waypoint[1]['x']), float(ch.waypoint[1]['y']))
                wstart = prm.anchor_node(wxy[0], wxy[1], global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                         ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M, connected_first=True)
                ok = wstart is not None and self.has_goal(wstart, wxy, r, c, cells_of, global_map, sampler)
            else:
                ok = True
            if not ok:
                self.status[(r, c)] = ("unreachable: a way into the cell, but no goal the attempt "
                                       "would accept is reachable" + ("" if ch.near else " from the waypoint"))
                continue
            if failed:
                # Behind every cell that never failed, near or far.
                if far_failed is None or (ch.near and not far_failed.near):
                    far_failed = ch
                continue
            if ch.near:
                return ch
            if far is None:
                far = ch
        return far if far is not None else far_failed

    def record_failure(self, choice, why):
        """A real failure on the cell of choice (the robot went and stopped, or the attempt
        found no way the choice had seen): counted with its entry (see MAX_FAILURES)."""
        key = (choice.row, choice.col)
        self.failures.setdefault(key, []).append(choice.entry)
        n = len(self.failures[key])
        MISSION_STATS['cell_failures'] = MISSION_STATS.get('cell_failures', 0) + 1
        print(f"[CELL RULES] Cell ({choice.row},{choice.col}) failed ({why}), {choice.describe_entry()}"
              + f": {n} failure{'s' if n > 1 else ''}"
              + (f", set aside until a way enters it elsewhere." if n >= MAX_FAILURES else "."))

    def explain_end(self, attempts):
        """[END] lines: why the cells left are not done."""
        env = self.env
        left = [(r, c) for r in range(env.rows) for c in range(env.cols) if env.map[r][c] == 0]
        visited = sum(1 for row in env.map for v in row if v == 1)
        if not left:
            return
        if visited <= 1:
            print(f"[END] No cell reachable from the start: {attempts} attempts on the neighbouring "
                  f"cells, no way in the graph (obstacles all around: see the [CELL RULES] lines). "
                  f"Move the robot onto free terrain and try again.")
        groups = {}
        for rc in left:
            why = self.status.get(rc, "never next to a visited cell")
            groups.setdefault(why.split(':')[0], []).append(rc)
        print(f"[END] Exploration finished with {len(left)} cells not done: "
              + "; ".join(f"{k} {', '.join(f'({r},{c})' for r, c in v)}" for k, v in groups.items()) + ".")
        MISSION_STATS['cells_left'] = {k: [list(rc) for rc in v] for k, v in groups.items()}
