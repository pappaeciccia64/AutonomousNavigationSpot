"""
Which cell next.

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
robot's, the target and the shared neighbours). Its entry is the cell the way comes from when
it crosses into the cell (one of the 8 around it; None if the robot is already in it). Failures
are counted per entry, not per edge: a cell is entered by dozens of edges from the same side.

The length of a way: its Dijkstra cost on the graph of now (the same weights of the attempt:
length, plus the slopes of the edges already seen, plus the penalties), from the robot node to
the goal the attempt would choose in the cell (has_goal). With alpha = 1 a cost is metres on
flat ground.

The choice: among the candidates whose way costs at most NEAR_PATH_M, the first in the
visiting order, reached in cruise; if none is that near, the first in the visiting order
among all the candidates, reached with GraphNav (the exception, with a stop). A cell that
already failed comes after all the others (in the visiting order among the failed ones):
its second try comes later, from where the robot is then, not at once from the same side.
"""

import numpy as np

from spot_explorer.mission import mission_prm
from spot_explorer.mission import recovery
from spot_explorer.perception import geometry
from spot_explorer.planning import goal as goal_choice
from spot_explorer.planning import roadmap
from spot_explorer.records import MISSION_STATS

NEAR_PATH_M = 4.0          # a way costing at most this (metres on flat ground) is walked in cruise; more, GraphNav
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
        return f"cell ({self.row},{self.col}), way costing {self.length:.1f} (m on flat ground), {self.describe_entry()}: {how}"


class CellRules:
    """The cell rules of a mission: failures per cell and the choice (see the module)."""

    def __init__(self, env, prm, path):
        self.env, self.prm = env, prm
        self.rank = {tuple(c): i for i, c in enumerate(path)}
        self.failures = {}          # (r, c) -> [entry, ...] of the real failures
        self.status = {}            # (r, c) -> why it is not a candidate now (for the log)
        # Cells left because unreachable now (no goal with a path): not failed, skipped at the
        # next choice only (not a candidate now; it comes back when the graph changes).
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
    def _costs(self, start, row, col, from_xy, cells_of, target=None):
        """The search on the graph of now (PRM.costs_from: the weights of the attempt) from node
        start, through the cells the attempt may use. target = a cell: stops at the first node
        in it. (dist, pred, node reached)."""
        prm, env = self.prm, self.env
        allowed = mission_prm.planning_cells(env, row, col, from_xy)
        stop = None
        if target is not None:
            def stop(u):
                return cells_of.get(u) == target and np.isfinite(prm.node_penalty(u))
        return prm.costs_from(start, allowed=None if allowed is None else (lambda v: cells_of.get(v) in allowed),
                              stop=stop)

    def way_into(self, start, robot_xy, row, col, cells_of):
        """(cost, entry cell, entry edge, node) of the cheapest way from node start to the first
        node of cell (row, col) on the graph of now, None if there is none."""
        dist, pred, u = self._costs(start, row, col, robot_xy, cells_of, target=(int(row), int(col)))
        if u is None:
            return None
        p = pred[u]
        return dist[u], (None if p is None else cells_of.get(p)), (None if p is None else (p, u)), u

    def has_goal(self, start, from_xy, row, col, cells_of, global_map, sampler):
        """The Dijkstra cost from from_xy (node start) to the goal of the cell the attempt would
        accept, None if there is none. The same choice as the attempt (goal.find_best_point_in_cell,
        on the global map: no live scan here), with "reachable" = in the graph from start through
        the cells the attempt may use: a way into the cell is not enough if no goal of it can be
        reached. Without map or sampler: 0.0 (not checked)."""
        if global_map is None or sampler is None:
            return 0.0
        prm, env = self.prm, self.env
        dist, pred, _u = self._costs(start, row, col, from_xy, cells_of)
        self.last_goal_path = []

        def anchor(px, py):
            return prm.anchor_node(px, py, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M)

        def reachable(px, py):
            # As the attempt (PRM.reachable_from): a path, within the detour cap.
            nid = anchor(px, py)
            return nid is not None and nid in pred and prm._within_caps(prm.path_to(pred, nid), None)
        gx, gy, _v, _b = goal_choice.find_best_point_in_cell(
            from_xy[0], from_xy[1], env, row, col, None, None, sampler, global_map=global_map,
            reachable=reachable, quiet=True, require_reachable=True)
        if gx is None:
            return None
        nid = anchor(gx, gy)
        if nid not in dist:
            return None
        path, u = [], nid
        while u is not None:
            path.append(u)
            u = pred[u]
        self.last_goal_path = path[::-1]     # start -> goal node of the last has_goal
        return float(dist[nid])

    def _near_choice(self, ch, start, robot_xy, robot_cell, cells_of, global_map, sampler, excluded):
        """The near candidate ch, first in the visiting order, is the choice."""
        return ch

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
        for rc in excluded:
            self.status[rc] = "unreachable: no goal with a path when last tried"
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
            # Near or far: the cost of the way to the goal the attempt would choose (at least
            # the cost to the first node of the cell, so a cell beyond that is far anyway).
            cost = self.has_goal(start, robot_xy, r, c, cells_of, global_map, sampler) if ch.near else None
            if cost is not None:
                ch.length = max(length, cost)
            ch.near = cost is not None and ch.length <= NEAR_PATH_M
            if ch.near:
                ok = True
            elif waypoints:
                nx, ny = ch.node_xy
                ch.waypoint = min(waypoints.items(), key=lambda kv: np.hypot(kv[1]['x'] - nx, kv[1]['y'] - ny))
                wxy = (float(ch.waypoint[1]['x']), float(ch.waypoint[1]['y']))
                wstart = prm.anchor_node(wxy[0], wxy[1], global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                         ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M, connected_first=True)
                ok = wstart is not None and self.has_goal(wstart, wxy, r, c, cells_of, global_map, sampler) is not None
            else:
                # No waypoints to check from: far is fine, near without a goal is not.
                ok = cost is not None or length > NEAR_PATH_M
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
                return self._near_choice(ch, start, robot_xy, robot_cell, cells_of, global_map, sampler, excluded)
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
