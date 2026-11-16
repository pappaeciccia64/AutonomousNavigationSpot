"""
THE RULES OF THE SIDES TRIED (before 09-11-2026), out of the main code (12-11): the frontier of
candidate cells; a cell next to the robot's is attempted from here (_attempt_adjacent_cell),
otherwise the lowest-rank candidate is approached with GraphNav up to the waypoint of the
nearest visited cell (_pick_far_cell, _travel_and_attempt); a failed attempt marks the SIDE it
was tried from (_mark_side_failed) and the cell is blocked only when every reachable side has
been tried (recovery.finalize_or_defer_blocked_cell); deferral, SKIP-ROUND, PRECHECK.

Replaced by the cell rules (mission/cell_rules.py, exploration.Mission.explore). Still runnable:
    python3 tools/run_legacy_mission.py --side-rules [the arguments of run_mission.py]
or in code: exploration.Mission = side_rules.SideRulesMission (test_mission does so).
With these rules the cruise does not prepare the next cell (no env._cell_rules): every cell
ends with the hand-off to the next attempt, as before 09-11.
"""
import time

import numpy as np

from spot_explorer import config as P
from spot_explorer.graphnav import map_shortcuts
from spot_explorer.graphnav import map_recording
from spot_explorer.graphnav import navigation
from spot_explorer.mission import attempt_common, attempt_cruise, cells, mission_prm, recovery
from spot_explorer.mission import exploration
from spot_explorer.perception import geometry
from spot_explorer.perception import spot_grid
from spot_explorer.planning import goal
from spot_explorer.planning import roadmap
from spot_explorer.records import HANDOFF, MISSION_STATS
from spot_explorer.robot import connection
from spot_explorer.robot import motion



# =========================================================================================
# THE SIDE BOOKKEEPING, out of the main modules (16-11): methods of cells.EnvironmentMap and
# map_recording.RecordingInterface, functions of cells, recovery and mission_prm that only
# these rules use. The methods are added to the two classes when this module is imported.
# =========================================================================================
# Before moving with GraphNav to the waypoint from which to attempt a cell, the graph is checked
# to reach the cell from there (see graph_reaches_cell). In the test of 22-10
# 15:24, three moves out of six ended in an attempt failed in less than a second with
# "GRAPH DISCONNECTED", which the graph already knew before leaving: ~25 s out of 3 minutes.
# False = it leaves anyway and the attempt decides. (Was config.GRAPHNAV_PRECHECK.)
GRAPHNAV_PRECHECK = True


class _SideCells:
    def get_lowest_rank_from_frontier_list(self, frontier_list, path):
        """
        Among the cells of frontier_list ((row, column) or (row, column, rank)), the one
        with the lowest rank in the serpentine path. Returns (row, column, rank), None if
        the list is empty or path is missing.
        """
        if path is None:
            print(f"[ERROR] get_lowest_rank_from_frontier_list needs the path parameter")
            return None

        if not frontier_list or len(frontier_list) == 0:
            print(f"[FRONTIER_RANK] Frontier empty")
            return None

        print(f"\n[FRONTIER_RANK] Looking for the lowest-rank cell among {len(frontier_list)} frontier cells")

        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        min_rank = float('inf')
        best_cell = None

        for cell in frontier_list:
            # Any rank already present is ignored: it is re-read from path.
            if isinstance(cell, tuple):
                if len(cell) >= 2:
                    row, col = cell[0], cell[1]
                    cell_tuple = (row, col)

                    rank = cell_to_rank.get(cell_tuple, float('inf'))

                    print(f"  Cell ({row},{col}): rank={rank}")

                    if rank < min_rank:
                        min_rank = rank
                        best_cell = (row, col, rank)

        if best_cell:
            print(f"[FRONTIER_RANK] ✓ Lowest-rank cell: ({best_cell[0]},{best_cell[1]}) rank={best_cell[2]}")
            return best_cell
        else:
            print(f"[FRONTIER_RANK] ⚠️ No valid cell in the list")
            return None

    def get_adjacent_frontier_cells(self, cell_row, cell_col, path):
        """
        The adjacent cells (north, south, east, west) not yet visited (value 0), except
        those already tried from this same side.

        path: the serpentine, list of (row, column); a cell's rank is its position in path.

        Returns [(row, column, rank), ...] in increasing rank order, [] if there are none.
        """
        if path is None:
            print(f"[ERROR] get_adjacent_frontier_cells needs the path parameter")
            return []

        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        unexplored_neighbors = []
        neighbors = [
            (-1, 0),
            (1, 0),
            (0, 1),
            (0, -1)
        ]

        for dr, dc in neighbors:
            neighbor_row = cell_row + dr
            neighbor_col = cell_col + dc

            if not (0 <= neighbor_row < self.rows and 0 <= neighbor_col < self.cols):
                continue

            if self.map[neighbor_row][neighbor_col] == 0:
                # The cell may still be at 0 but already tried from THIS side without
                # success (the other sides remain to be tried). The same attempt is not
                # proposed again right away: it will come back into play from another side.
                side_bit = self.get_side_bit_facing_origin(cell_row, cell_col, neighbor_row, neighbor_col)
                if self.get_cell_sides_status(neighbor_row, neighbor_col) & side_bit:
                    continue

                neighbor_cell = (neighbor_row, neighbor_col)
                rank = cell_to_rank.get(neighbor_cell, float('inf'))
                unexplored_neighbors.append((neighbor_row, neighbor_col, rank))

        unexplored_neighbors.sort(key=lambda x: x[2])
        return unexplored_neighbors

    def mark_cell_side_explored(self, row, col, side_bit):
        """
        Record that entry into the cell was tried from side side_bit
        (North 0b1000, East 0b0100, South 0b0010, West 0b0001).
        """
        cell_key = (row, col)
        current = self.explored_sides.get(cell_key, 0b0000)
        self.explored_sides[cell_key] = current | side_bit
        print(f"[SIDE] Cell ({row},{col}): side {bin(side_bit)} marked as tried "
              f"(now {bin(self.explored_sides[cell_key])})")

    def all_sides_explored(self, row, col):
        """
        True only if entry has already been tried from EVERY side that has a neighbour. Only
        then is the cell marked blocked (-1); before that it stays at 0, so exploration
        proposes it again later from a side not yet tried instead of excluding it at the
        first failure.
        """
        return (self.get_cell_sides_status(row, col) & self.get_valid_neighbor_sides_mask(row, col)) == \
               self.get_valid_neighbor_sides_mask(row, col)

    def untried_side_neighbours(self, row, col):
        """
        The neighbouring cells on the sides of (row, col) not tried yet, split into
        (visited, not_visited). A side can only be attempted from a visited neighbour (its
        GraphNav waypoint): with visited empty the cell waits for one of the others to be
        visited.
        """
        sides = self.get_cell_sides_status(row, col)
        visited, not_visited = [], []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            if not (0 <= r < self.rows and 0 <= c < self.cols):
                continue
            if sides & self.get_side_bit_facing_origin(r, c, row, col):
                continue
            (visited if self.map[r][c] == 1 else not_visited).append((r, c))
        return visited, not_visited

    def get_valid_neighbor_sides_mask(self, row, col):
        """
        Mask of the sides that really have a neighbouring cell inside the grid (border and
        corner cells have fewer than 4): all the sides it could ever be tried from.
        Used by all_sides_explored to know when a cell has run out of options.
        North 0b1000, East 0b0100, South 0b0010, West 0b0001.
        """
        mask = 0b0000
        if row - 1 >= 0:
            mask |= 0b1000  # north
        if row + 1 < self.rows:
            mask |= 0b0010  # south
        if col + 1 < self.cols:
            mask |= 0b0100  # east
        if col - 1 >= 0:
            mask |= 0b0001  # west
        return mask

    def get_side_bit_facing_origin(self, origin_row, origin_col, target_row, target_col):
        """
        The robot went (or tried to go) from (origin_row, origin_col) towards (target_row,
        target_col). Returns the bit of TARGET's side facing origin, i.e. the side actually
        tried. 0 if the cells are not adjacent.
        """
        dr, dc = target_row - origin_row, target_col - origin_col
        if dr == -1 and dc == 0:
            return 0b0010  # target north of origin: entered from the SOUTH side
        if dr == 1 and dc == 0:
            return 0b1000  # target south: entered from the NORTH
        if dr == 0 and dc == 1:
            return 0b0001  # target east: entered from the WEST
        if dr == 0 and dc == -1:
            return 0b0100  # target west: entered from the EAST
        return 0b0000

    def is_cell_blocked(self, row, col):
        """True if the cell is blocked (-1)."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == -1
        return False



class _SideWaypoints:
    def find_nearest_waypoint_cell_to_target(self, target_cell, waypoints_by_cell, env_map):
        """
        The visited cell with a waypoint nearest to target_cell: an adjacent one first,
        otherwise the nearest in Manhattan distance. None if there are none.
        waypoints_by_cell as from get_all_manual_waypoints_with_cells.
        """
        target_row, target_col = target_cell

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        adjacent_with_waypoint = []
        for dr, dc in directions:
            adj_row, adj_col = target_row + dr, target_col + dc
            adj_cell = (adj_row, adj_col)

            if (0 <= adj_row < env_map.rows and
                0 <= adj_col < env_map.cols and
                adj_cell in waypoints_by_cell and
                env_map.map[adj_row][adj_col] == 1):  # visited
                adjacent_with_waypoint.append(adj_cell)

        if adjacent_with_waypoint:
            print(f"[NEAREST_WP] Waypoint in a neighbouring cell: {adjacent_with_waypoint[0]}")
            return adjacent_with_waypoint[0]

        # No adjacent one: the nearest among all
        min_distance = float('inf')
        nearest_cell = None

        for cell in waypoints_by_cell.keys():
            cell_row, cell_col = cell

            if env_map.map[cell_row][cell_col] != 1:
                continue

            distance = abs(cell_row - target_row) + abs(cell_col - target_col)

            if distance < min_distance:
                min_distance = distance
                nearest_cell = cell

        if nearest_cell:
            print(f"[NEAREST_WP] Nearest waypoint in cell {nearest_cell} (distance: {min_distance})")

        return nearest_cell

    def get_manual_waypoint_by_cell(self, cell_row, cell_col):
        """The data of the waypoint created in the cell, None if there is none."""
        waypoints_by_cell = self.get_all_manual_waypoints_with_cells()
        return waypoints_by_cell.get((cell_row, cell_col))



for _cls, _mix in ((cells.EnvironmentMap, _SideCells), (map_recording.RecordingInterface, _SideWaypoints)):
    for _name, _fn in vars(_mix).items():
        if callable(_fn) and not _name.startswith('__'):
            setattr(_cls, _name, _fn)


def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells

def finalize_or_defer_blocked_cell(env, row, col, sampler=None, global_map=None):
    """
    Call right after env.mark_cell_side_explored(row, col, side_bit), when an attempt to
    enter (row, col) has just failed from one side.

    The cell is marked blocked forever (env.mark_cell_blocked -> value -1) only when EVERY
    side with a neighbour inside the grid has been tried and has failed
    (env.all_sides_explored). Otherwise it stays at the not-visited value (0) and is NOT
    retried right away: it stays a normal frontier candidate.
    get_adjacent_frontier_cells and get_lowest_rank_from_frontier_list treat every cell at
    0 the same way, ordered by its position in the visiting path, like a cell never tried.
    So a cell that failed once competes for priority with the SAME logic as all the
    others, and is retried when its turn comes, from the side not yet tried: neither right
    away out of turn, nor discarded after a single side.
    """
    # If there is no longer a point in the cell where the robot can stand, trying from
    # another side is useless -- the goal is inside an obstacle, not behind a gap that
    # opens elsewhere. See cell_has_reachable_target.
    if sampler is not None and global_map is not None:
        ok, free, unknown, occupied = recovery.cell_has_reachable_target(env, sampler, global_map, row, col)
        # Diagnostics: tells at once whether the cell is inside an obstacle or was never
        # seen, i.e. whether the mission grid does not fit the space.
        # CAREFUL how to read it: the candidate points per cell are dozens (lattice at
        # 0.5 m) or hundreds (at 0.25 m), so a high "occupied" does NOT mean the cell is unreachable;
        # it means much of the cell is inside an obstacle. The immediate block below
        # triggers only in the extreme case where NOT a single free or unknown point is left.
        print(f"[CELL] ({row},{col}) after the failed attempt: of the candidate points, "
              f"{free} seen free, {unknown} never seen, {occupied} inside an obstacle "
              f"in the global map.")
        if not ok:
            env.mark_cell_blocked(row, col)
            print(f"[BLOCKED] Cell ({row},{col}) blocked at once: all {occupied} candidate points "
                  f"are inside an obstacle in the global map (none free, "
                  f"none unknown). There is no side to enter from: the goal is in the wall.")
            return
    if env.all_sides_explored(row, col):
        env.mark_cell_blocked(row, col)
        print(f"[BLOCKED] Cell ({row},{col}) blocked forever: every reachable side "
              f"has been tried without success.")
    else:
        visited, not_visited = env.untried_side_neighbours(row, col)
        cells_txt = lambda cells: ', '.join(f'({r},{c})' for r, c in cells)
        if visited:
            when = (f"it will be retried later, in the normal order, from the side of "
                    f"{cells_txt(visited)}")
        else:
            when = (f"the sides not tried yet face cells never visited ({cells_txt(not_visited)}): "
                    f"it can be retried only if one of them gets visited, otherwise it leaves "
                    f"the frontier at the end")
        print(f"[DEFER] Cell ({row},{col}) not yet tried from all sides (sides tried: "
              f"{bin(env.get_cell_sides_status(row, col))}): it stays a candidate of the "
              f"frontier; {when}.")

def _reach_from(prm, env, from_xy, row, col, global_map):
    """(nodes reachable from from_xy, cell of every node) with the rules of
    graph_reaches_cell; None if the graph has no nodes."""
    start = prm.anchor_node(float(from_xy[0]), float(from_xy[1]), global_map,
                            geometry.PRM_EDGE_SAFETY_MARGIN_M,
                            ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M, connected_first=True)
    if start is None:
        return None
    cells = mission_prm.node_cells(prm, env)
    ok_cells = mission_prm.planning_cells(env, row, col, from_xy) if P.PLAN_ONLY_EXPLORED_CELLS else None
    seen, queue = {start}, [start]
    while queue:
        u = queue.pop()
        for v, _w in prm.edges.get(u, ()):
            if v in seen:
                continue
            if ok_cells is not None and cells.get(v) not in ok_cells:
                continue
            seen.add(v)
            queue.append(v)
    return seen, cells

def unreached_cell_reason(prm, env, from_xy, row, col, global_map=None):
    """Why graph_reaches_cell is False, for the [PRECHECK] line: the cell has no usable node
    (all inside obstacles), or the edges entering it from the reachable area are all out
    of the graph (vetoed by obstacles or slope, even if the inside of the cell was never
    seen: 03-11 11:09, cells behind the wall of the room)."""
    reach = _reach_from(prm, env, from_xy, row, col, global_map)
    if reach is None:
        return "graph without nodes"
    seen, cells = reach
    target = (int(row), int(col))
    in_cell = [u for u, c in cells.items() if c == target]
    usable = [u for u in in_cell if np.isfinite(prm.node_penalty(u))]
    if not usable:
        return f"all the {len(in_cell)} nodes of the cell are discarded as inside obstacles"
    reached = sum(1 for u in seen if cells.get(u) != target)
    return (f"no edge of the graph links the {reached} nodes reachable from there to the "
            f"{len(usable)} usable nodes of the cell: the edges in between are vetoed "
            f"(obstacle or slope on the way)")

def graph_reaches_cell(prm, env, from_xy, row, col, global_map=None):
    """
    From the point from_xy (the waypoint the cell would be attempted from) does the graph
    reach at least one node of cell (row, col)? Same rules as the attempt: with
    PLAN_ONLY_EXPLORED_CELLS it passes only through the visited cells, the starting cell and
    the goal cell; edges and nodes as they are now (a node with infinite penalty does not
    count). It is a NECESSARY condition: if False the attempt would surely fail with "GRAPH
    DISCONNECTED"; if True the attempt decides (goal point, detour cap, frontier).
    Breadth-first search, no effect on the graph. None if the graph has no nodes.
    """
    reach = _reach_from(prm, env, from_xy, row, col, global_map)
    if reach is None:
        return None
    seen, cells = reach
    target = (int(row), int(col))
    return any(cells.get(u) == target and np.isfinite(prm.node_penalty(u)) for u in seen)


class SideRulesMission(exploration.Mission):
    """exploration.Mission with the rules of the sides tried: the frontier list of candidate
    cells (row, column, rank), grown by every cell visited."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.frontier = []
        self._last_visited = None      # see _robot_cell
        self._dropped_cells = []       # removed from the frontier: untried sides face only non-visited cells

    def explore(self):
        return self._explore_sides()

    def _on_visited(self, row, col, x, y):
        super()._on_visited(row, col, x, y)
        self._last_visited = (int(row), int(col))
        # The neighbours of the cell JUST VISITED, not of the one the robot is in: with
        # arrival just outside the cell (ARRIVAL_OUTSIDE_CELL_M) the robot may still be
        # in the previous cell.
        self.frontier.extend(find_new_borders(self.env, row, col, self.path, self.frontier))

    def _close_seen_cells(self):
        """SEEN_BEFORE_MOVING: the frontier cells already seen enough become visited without
        moving, in rank order; the neighbours they add to the frontier are checked too.
        Returns how many."""
        if not (exploration.SEEN_BEFORE_MOVING and attempt_common.CELL_SEEN_EXPLORED):
            return 0
        gmap = self.global_grid.global_occupancy_map
        if gmap is None or not self.frontier:
            return 0
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        closed, changed = 0, True
        while changed:
            changed = False
            for entry in sorted(self.frontier, key=lambda f: f[2]):
                r, c = int(entry[0]), int(entry[1])
                if self.env.map[r][c] != 0:
                    continue
                frac = cells.cell_seen_fraction(self.env, gmap, r, c)
                if frac is None or frac < attempt_common.CELL_SEEN_FRACTION:
                    continue
                border = cells.cell_border_distance(self.env, r, c, x, y)
                here = border is not None and -border <= P.ARRIVAL_OUTSIDE_CELL_M
                center = self.env.get_world_position_from_cell(r, c)
                dist = float(np.hypot(center[0] - x, center[1] - y))
                print(f"[SEEN] Cell ({r},{c}): {100 * frac:.0f}% of it already in the global map "
                      f"(threshold {100 * attempt_common.CELL_SEEN_FRACTION:.0f}%) before moving there: "
                      f"explored without going, "
                      + ("the robot is in it: its GraphNav waypoint here." if here else
                         f"centre {dist:.1f} m from the robot: no GraphNav waypoint of its own."))
                self.frontier.remove(entry)
                if here:
                    self._on_visited(r, c, x, y)
                else:
                    self.env.mark_cell_visited(r, c)
                    self.frontier.extend(find_new_borders(self.env, r, c, self.path, self.frontier))
                MISSION_STATS['cells_seen_before_moving'] = MISSION_STATS.get('cells_seen_before_moving', 0) + 1
                closed += 1
                changed = True
                break
        return closed


    def _robot_cell(self):
        """
        The cell from which the robot looks for neighbours and marks the sides tried. Usually
        the one it is in; but if the last visited cell was accepted with the robot just
        OUTSIDE (ARRIVAL_OUTSIDE_CELL_M) and the robot is still there, that one counts: on
        28-10 13:50 (7,2) was visited with the robot 0.29 m outside, in (6,3), and (7,1)
        next to it was never attempted from there.
        """
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        cell = tuple(int(v) for v in self.env.get_cell_from_world(x, y))
        uv = self._last_visited
        if uv is not None and cell != uv:
            border = cells.cell_border_distance(self.env, uv[0], uv[1], x, y)
            if border is not None and -border <= P.ARRIVAL_OUTSIDE_CELL_M:
                print(f"[CELL] The robot is in cell {cell}, but {-border:.2f} m outside the "
                      f"just visited {uv}: I look for the neighbours from {uv}.")
                return uv
        return cell

    def _start_cell_for(self, row, col, waypoints_by_cell):
        """
        The visited cell (with a GraphNav waypoint) from which to approach candidate (row,
        col): among the NEIGHBOURS, one whose side towards the candidate has not been tried
        yet, the nearest to the robot. If no neighbour has a free side, the usual rule
        (find_nearest_waypoint_cell_to_target), which the caller then skips if that side
        has already been tried. Before, the first neighbour in a fixed order (north, south,
        west, east) was always taken: once that side was tried, the cell was no longer
        attempted from the others and dropped out of the frontier (28-10 13:50, cells (7,0)
        and (7,1)).
        """
        env, ri = self.env, self.recordingInterface
        sides = env.get_cell_sides_status(row, col)
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        cands = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            if not (0 <= r < env.rows and 0 <= c < env.cols) or env.map[r][c] != 1:
                continue
            if (r, c) not in waypoints_by_cell or ri.get_manual_waypoint_by_cell(r, c) is None:
                continue
            if sides & env.get_side_bit_facing_origin(r, c, row, col):
                continue
            wx, wy = env.get_world_position_from_cell(r, c)
            cands.append((float(np.hypot(wx - x, wy - y)), (r, c)))
        if cands:
            d, cell = min(cands)
            print(f"[NEAREST_WP] Cell ({row},{col}): approaching from the neighbour {cell}, side not "
                  f"tried yet, {d:.1f} m from the robot"
                  + (f" ({len(cands)} free sides)" if len(cands) > 1 else "") + ".")
            return cell
        return ri.find_nearest_waypoint_cell_to_target((row, col), waypoints_by_cell, env)

    def _mark_side_failed(self, row, col, from_row, from_col):
        """Mark as tried the side of cell (row, col) facing cell (from_row, from_col). True if
        the cell is now blocked (no reachable side left to try)."""
        side_bit = self.env.get_side_bit_facing_origin(from_row, from_col, row, col)
        self.env.mark_cell_side_explored(row, col, side_bit)
        finalize_or_defer_blocked_cell(self.env, row, col,
                                       self.sampler, self.global_grid.global_occupancy_map)
        return self.env.is_cell_blocked(row, col)

    # --- The loop --------------------------------------------------------------------------

    def _explore_sides(self):
        """Visit the cells until the frontier is empty (the rules before 09-11: sides tried)."""
        x, y, z, _ = connection.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x, y)
        self.frontier.extend(find_new_borders(
            self.env, robot_row, robot_col, self.path, self.frontier))
        self._t0 = time.time()
        self._closed_at_start = self._cell_counts()[0]

        while (True):
            self._log_progress()
            if attempt_common.battery_return_due():
                MISSION_STATS['battery_return_cells_left'] = len(self.frontier)
                print(f"[BATTERY] End of exploration for low battery: "
                      f"{len(self.frontier)} cells left in the frontier {[(f[0], f[1]) for f in self.frontier]}.")
                break
            if self._close_seen_cells() and len(self.frontier) == 0:
                break
            HANDOFF.mark("progress and seen cells")
            robot_row, robot_col = self._robot_cell()

            borders = self.env.get_adjacent_frontier_cells(robot_row, robot_col, self.path)
            borders_in_frontier = [b for b in borders
                                   if any((f[0] == b[0] and f[1] == b[1]) for f in self.frontier)]

            if len(borders_in_frontier) != 0:
                self._attempt_adjacent_cell(min(borders_in_frontier, key=lambda b: b[2]),
                                       robot_row, robot_col)
            else:
                # No neighbouring cell in the frontier: go with GraphNav towards the waypoint
                # nearest to the lowest-rank cell and attempt from there.
                attempt_cruise.stop_if_moving(self.env, self.command_client,
                                              "no neighbouring cell to continue to")
                choice = self._pick_far_cell()
                if choice is not None:
                    if self._travel_and_attempt(*choice) == 'caduta':
                        continue
                else:
                    if len(self.frontier) > 0:
                        removed = self.frontier[0]
                        self.frontier.remove(removed)
                        self._dropped_cells.append((int(removed[0]), int(removed[1])))
                        print(f"[CELL FRONTIER] Cell ({removed[0]},{removed[1]}) removed from the frontier: no "
                              f"candidate can be approached any more (the sides not tried yet face "
                              f"only cells never visited, with no waypoint to start from).")

            if len(self.frontier) == 0:
                break
        attempt_cruise.stop_if_moving(self.env, self.command_client, "end of the exploration")
        self._explain_end()
        self._log_progress(final=True)

    def _explain_end(self):
        """If exploration ends with cells not closed, an [END] line saying why (28-10 14:17:
        start in the middle of tall grass, no reachable cell, and in the log only
        "Frontiera vuota")."""
        closed, visited, blocked, total = self._cell_counts()
        if closed >= total or attempt_common.battery_low():
            return
        if visited <= 1:
            print(f"[END] No cell reachable from the start: {self._attempt_count} attempts "
                  f"on the neighbouring cells, all failed without a way in the graph or with the step "
                  f"refused (obstacles all around: see GRAPH DISCONNECTED and [ARRIVAL] above). "
                  f"Move the robot onto free terrain and try again.")
            return
        print(f"[END] Exploration finished with {total - closed} cells not closed"
              + (f"; removed from the frontier because their sides not tried yet face only "
                 f"cells never visited: "
                 f"{', '.join(f'({r},{c})' for r, c in self._dropped_cells)}" if self._dropped_cells else "")
              + ". The others never bordered a visited cell.")

    # --- Progress ------------------------------------------------------------------------

    def _attempt_adjacent_cell(self, selected_border, robot_row, robot_col):
        """Attempt on a frontier cell bordering the robot's cell."""
        row, col = selected_border[0], selected_border[1]
        # Cell entry is attempted from: needed afterwards to mark the SIDE tried. Not the
        # cell the robot is in AFTER the attempt, which a retreat or a partial step
        # may have changed (wrong or null side).
        origin_row, origin_col = robot_row, robot_col
        check = self._attempt(row, col)
        self.frontier.remove(selected_border)

        x_new, y_new, _, _ = connection.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x_new, y_new)

        if check or (row, col) == (robot_row, robot_col):
            # Failed but the robot is inside the cell anyway: it counts as visited.
            self._on_visited(row, col, x_new, y_new)
        elif attempt_common.battery_low():
            # Attempt interrupted for the return: the cell is still to be explored.
            self.frontier.append(selected_border)
        else:
            self._retry_after_failure(selected_border, origin_row, origin_col)

    def _retry_after_failure(self, selected_border, origin_row, origin_col):
        """Right after a failed attempt on a neighbouring cell: retry once if from here there
        is a short way (at most MAX_RETRY_EDGES edges) to a goal different from the one that
        just failed; otherwise the side is marked as tried.

        The edges discarded in the failed attempt (blocks confirmed up close, physical
        failures) are already in the PRM via mark_edge_invalid, and the edge rebuild
        respects them: Dijkstra will not propose the same way again."""
        row, col = selected_border[0], selected_border[1]
        prm, global_map = self.prm, self.global_grid.global_occupancy_map

        # Current position (the failure may be at the start or halfway)
        x_curr, y_curr, _, _ = connection.getPosition(self.robot_state_client)

        # Fresh scan, to choose the goal point from here
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is not None:
            # Without body footprint or obstacle mask: only the terrain is needed for
            # the PRM and the goal choice.
            self.local_grid.process_scan(scan, x_curr, y_curr, 0.0, footprint=False, fuse=False)
            num_x, num_y = scan.num_x, scan.num_y
            grid_origin_x, grid_origin_y = self.local_grid.grid_origin(scan)
            prm.update_local_grid_data(
                grad_values_2d=scan.grad.reshape((num_y, num_x)),
                rough_values_2d=scan.rough.reshape((num_y, num_x)),
                terrain_values_2d=scan.terrain.reshape((num_y, num_x)),
                valid_2d=spot_grid.slope_usable_mask(scan.is_valid, scan.od, (num_y, num_x)),
                grid_origin_x=grid_origin_x,
                grid_origin_y=grid_origin_y,
                cell_size=scan.cell_size
            )
            mission_prm.rebuild_prm_edges(prm, global_map)

            target_x, target_y, _, _ = goal.find_best_point_in_cell(
                x_curr, y_curr, self.env, row, col, scan.pts, scan.od, self.sampler,
                global_map=global_map
            )
        else:
            print(
                "[RECOVERY ERROR] Could not fetch the obstacle_distance layer for the recomputation.")
            target_x, target_y = None, None

        if target_x is None or target_y is None:
            print(
                "[SKIP] No valid goal point in the cell. Carrying on with normal exploration.")
            if not self._mark_side_failed(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)
            return

        mission_prm.restrict_planning_to_explored_cells(prm, self.env, row, col, (x_curr, y_curr))
        start_id = prm.add_stop_node(
            x_curr, y_curr, global_map,
            geometry.PRM_EDGE_SAFETY_MARGIN_M, label="recovery check (robot)")
        goal_id = prm.add_stop_node(
            target_x, target_y, global_map,
            geometry.PRM_EDGE_SAFETY_MARGIN_M, label="recovery check (goal)",
            is_robot=False)
        prm.set_robot_node(start_id)

        path_ids = prm.find_path_dijkstra(start_id, goal_id)

        # Retry right away only if the path has at most MAX_RETRY_EDGES edges (number of
        # nodes - 1)
        MAX_RETRY_EDGES = 4

        # If the goal is the SAME as the attempt that just failed, retrying right away would
        # mean same place, same outcome: the cell is deferred and goes back in the queue like the
        # others (see finalize_or_defer_blocked_cell).
        last_goal = getattr(self.env, '_last_attempt_goal', None)
        same_goal = (last_goal is not None and float(np.hypot(
            target_x - last_goal[0], target_y - last_goal[1])) <= P.RECOVERY_SAME_GOAL_M)
        if same_goal and path_ids is not None:
            print(f"[RECOVERY] The goal ({target_x:.2f}, {target_y:.2f}) is "
                  f"that of the attempt that just failed: I do not retry at once, the "
                  f"cell goes back in the queue.")
            path_ids = None
        if path_ids is None or (len(path_ids) - 1) > MAX_RETRY_EDGES:
            print(
                f"[SKIP] No valid alternative path or cost above {MAX_RETRY_EDGES}. Carrying on with the normal algorithm.")
            if not self._mark_side_failed(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)
            return

        print(
            f"[RECOVERY] Found an alternative path with cost {len(path_ids) - 1} <= {MAX_RETRY_EDGES}. Retrying the entry...")
        if self._attempt(row, col):
            x_final, y_final, _, _ = connection.getPosition(self.robot_state_client)
            self._on_visited(row, col, x_final, y_final)
        elif attempt_common.battery_low():
            self.frontier.append(selected_border)
        else:
            print(
                "[FAIL] The alternative path failed too. Leaving the cell and carrying on with the normal algorithm.")
            if not self._mark_side_failed(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)

    def _pick_far_cell(self):
        """The lowest-rank candidate to approach with GraphNav:
        (row, column, rank, visited cell of the starting waypoint), None if none.

        A blocked cell (-1) never has sides left to try:
        finalize_or_defer_blocked_cell() blocks it only after every reachable side has been
        tried and has failed."""
        env, recordingInterface = self.env, self.recordingInterface
        already_skipped_this_round = set()
        while True:
            remaining = [c for c in self.frontier if (c[0], c[1]) not in already_skipped_this_round]
            if self.frontier and not remaining:
                # Not "Frontier empty" (03-11 11:09): the cells are still there, all skipped.
                print(f"[FRONTIER_RANK] All {len(self.frontier)} frontier cells skipped in this "
                      f"round: none can be approached from a side not tried yet.")
                return None
            lowest_rank_cell = env.get_lowest_rank_from_frontier_list(remaining, self.path)
            if lowest_rank_cell is None:
                return None
            candidate_row, candidate_col, candidate_rank = lowest_rank_cell
            waypoints_by_cell = recordingInterface.get_all_manual_waypoints_with_cells()
            candidate_nearest_cell = self._start_cell_for(candidate_row, candidate_col,
                                                                 waypoints_by_cell)
            if candidate_nearest_cell is None or \
                    recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell) is None:
                # No visited cell with a waypoint to start from: move on to the candidate
                # next.
                print(f"[SKIP-ROUND] Cell ({candidate_row},{candidate_col}): no GraphNav waypoint "
                      f"to approach from. Trying the next one.")
                already_skipped_this_round.add((candidate_row, candidate_col))
                continue
            candidate_side_bit = env.get_side_bit_facing_origin(candidate_nearest_cell[0],
                                                                candidate_nearest_cell[1],
                                                                candidate_row, candidate_col)
            if env.get_cell_sides_status(candidate_row, candidate_col) & candidate_side_bit:
                print(f"[SKIP-ROUND] Cell ({candidate_row},{candidate_col}): I would get there from the "
                      f"same side already tried without success. For this round I try the "
                      f"next candidate.")
                already_skipped_this_round.add((candidate_row, candidate_col))
                continue
            # Before leaving with GraphNav: does the graph reach the cell from that waypoint? If
            # not, the attempt would fail right on arrival (see GRAPHNAV_PRECHECK): the side is
            # marked as tried without moving.
            if GRAPHNAV_PRECHECK:
                cand_wp = recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell)
                wp_xy = (cand_wp.get('x'), cand_wp.get('y')) if isinstance(cand_wp, dict) else (None, None)
                reach = (graph_reaches_cell(self.prm, env, wp_xy, candidate_row, candidate_col,
                                                          self.global_grid.global_occupancy_map)
                         if wp_xy[0] is not None else None)
                if reach is False:
                    MISSION_STATS['graphnav_trips_avoided'] += 1
                    why = unreached_cell_reason(self.prm, env, wp_xy, candidate_row,
                                                            candidate_col,
                                                            self.global_grid.global_occupancy_map)
                    print(f"[PRECHECK] Cell ({candidate_row},{candidate_col}): from the waypoint "
                          f"of cell {candidate_nearest_cell} the graph does not reach any "
                          f"node of the cell ({why}). I mark the side as tried without moving.")
                    env.mark_cell_side_explored(candidate_row, candidate_col, candidate_side_bit)
                    finalize_or_defer_blocked_cell(env, candidate_row, candidate_col,
                                                   self.sampler, self.global_grid.global_occupancy_map)
                    if env.is_cell_blocked(candidate_row, candidate_col):
                        self.frontier[:] = [c for c in self.frontier
                                            if (c[0], c[1]) != (candidate_row, candidate_col)]
                    already_skipped_this_round.add((candidate_row, candidate_col))
                    continue
            return candidate_row, candidate_col, candidate_rank, candidate_nearest_cell

    def _travel_and_attempt(self, target_row, target_col, rank, nearest_cell):
        """With GraphNav up to the waypoint of cell nearest_cell, then the attempt on cell
        (target_row, target_col). 'caduta' if the robot fell during the trip."""
        recordingInterface = self.recordingInterface
        x_current, y_current, _, _ = connection.getPosition(self.robot_state_client)

        nearest_wp = recordingInterface.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
        map_shortcuts.graphnav_add_shortcuts(recordingInterface, "before the move between cells",
                                             map_edges=True, target_wp_id=nearest_wp['id'],
                                             robot_xy=(x_current, y_current))
        HANDOFF.report("GraphNav trip")
        recordingInterface.stop_recording()
        navigation_success = navigation.graphnav_navigate_to(
            recordingInterface, nearest_wp['id'], self.robot_state_client, self.command_client,
            f"to the waypoint of cell {nearest_cell} for cell ({target_row},{target_col})",
            face_xy=self.env.get_world_position_from_cell(target_row, target_col))

        # GraphNav does not record the legs travelled: the old ones no longer describe the way
        # the robot came, the retreat must not use them.
        self.env._traveled_arcs = []
        if motion.check_and_recover_fall(self.robot_state_client, self.command_client,
                                            self.global_grid.global_occupancy_map,
                                            "during the move with GraphNav") is not None:
            recordingInterface.start_recording()
            print(f"[FALL] Got up after the move with GraphNav: I mark the side of "
                  f"cell ({target_row},{target_col}) as tried, so as not to repeat the same way.")
            if self._mark_side_failed(target_row, target_col, nearest_cell[0], nearest_cell[1]):
                self.frontier.remove((target_row, target_col, rank))
            return 'caduta'

        recordingInterface.start_recording()
        # The trip may have been long: if the battery dropped below the threshold the cell
        # is not attempted, the robot returns (the cell stays in the frontier).
        if attempt_common.battery_return_due("after the GraphNav move"):
            return None
        # If GraphNav did not even reach the starting waypoint it counts as a failed
        # attempt. Side marked as tried; the cell leaves the frontier only if it was
        # the LAST untried side (cell really blocked).
        if navigation_success and self._attempt(target_row, target_col):
            x_final, y_final, _, _ = connection.getPosition(self.robot_state_client)
            self.frontier.remove((target_row, target_col, rank))
            self._on_visited(target_row, target_col, x_final, y_final)
        elif attempt_common.battery_low():
            pass
        elif self._mark_side_failed(target_row, target_col, nearest_cell[0], nearest_cell[1]):
            self.frontier.remove((target_row, target_col, rank))
        return None
