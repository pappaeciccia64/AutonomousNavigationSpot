"""The mission cell by cell: Mission, choice of the cell to attempt, trips with GraphNav."""
import os
import time
import numpy as np

from spot_explorer.graphnav import map_shortcuts
from spot_explorer.graphnav import navigation
from spot_explorer.robot import motion
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.mission import cells
from spot_explorer.robot import connection

from spot_explorer.mission import mission_prm

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, visualize_grid_with_candidates
from spot_explorer.mission import attempt_common, cell_attempt
from spot_explorer.planning import goal
from spot_explorer.mission import recovery


# =========================================================================
# THE MISSION: the cell-by-cell loop, start-up and closure
# =========================================================================


class Mission:
    """The mission cell by cell, after start-up (see run.py).

    The frontier is the list of candidate cells (row, column, rank on the serpentine path).
    At every turn: if a frontier cell borders the robot's cell, entry is attempted from here
    (_attempt_adjacent_cell); otherwise the lowest-rank candidate is chosen, the robot
    approaches it with GraphNav up to the waypoint of the nearest visited cell and attempts
    from there (_pick_far_cell, _travel_and_attempt). A failed attempt marks the SIDE it was
    tried from (_mark_side_failed): the cell is blocked only when every reachable side has
    been tried (finalize_or_defer_blocked_cell).

    On low battery (see battery.py) exploration stops even with a non-empty frontier: an
    attempt interrupted for the return does not mark the side as tried and the cell stays
    in the frontier, so the saved state tells what was left to explore.
    """

    def __init__(self, robot_state_client, command_client, recordingInterface, local_grid,
                 global_grid, env, prm, sampler, verification_tracker, mission_folder,
                 mobility_kwargs):
        self.robot_state_client = robot_state_client
        self.command_client = command_client
        self.recordingInterface = recordingInterface
        self.local_grid = local_grid
        self.global_grid = global_grid
        self.env = env
        self.prm = prm
        self.sampler = sampler
        self.verification_tracker = verification_tracker
        self.mission_folder = mission_folder
        self.mobility_kwargs = mobility_kwargs
        self.path = env.generate_serpentine_path(start_cell=env.start_cell)
        self.frontier = []
        self.visualization_counter = 0   # running number of the saved images
        # Progress (see _log_progress): start instant, closed cells at the start and
        # at the last printed line, points of the mission rectangle for the seen area.
        self._t0 = None
        self._closed_at_start = 0
        self._last_visited = None      # see _robot_cell
        self._attempt_count = 0               # attempts to enter a cell (see _explain_end)
        self._dropped_cells = []            # removed from the frontier with no sides left to try
        self._closed_logged = None
        self._area_points = None

    # --- Shared pieces -----------------------------------------------------------------------

    def _attempt(self, row, col):
        """One attempt to enter cell (row, col) from the current position."""
        check = cell_attempt.attempt_enter_cell_from_position(
            self.local_grid, self.global_grid, self.robot_state_client, self.command_client,
            self.env, row, col, self.sampler, self.prm, self.mission_folder,
            self.visualization_counter, self.recordingInterface, self.verification_tracker,
            mobility_kwargs=self.mobility_kwargs)
        self.visualization_counter += 1
        self._attempt_count += 1
        return check

    def _on_visited(self, row, col, x, y):
        """Cell (row, col) visited with the robot at (x, y): GraphNav waypoint of the cell and
        new candidates around it."""
        self.env.update_position(x, y)
        self.recordingInterface.create_default_waypoint(cell_row=row, cell_col=col)
        self.env.mark_cell_visited(row, col)
        self._last_visited = (int(row), int(col))
        # The neighbours of the cell JUST VISITED, not of the one the robot is in: with
        # arrival just outside the cell (ARRIVAL_OUTSIDE_CELL_M) the robot may still be
        # in the previous cell.
        self.frontier.extend(cells.find_new_borders(
            self.env, row, col, self.path, self.frontier))

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
        recovery.finalize_or_defer_blocked_cell(self.env, row, col,
                                       self.sampler, self.global_grid.global_occupancy_map)
        return self.env.is_cell_blocked(row, col)

    # --- The loop --------------------------------------------------------------------------

    def explore(self):
        """Visit the cells until the frontier is empty."""
        x, y, z, _ = connection.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x, y)
        self.frontier.extend(cells.find_new_borders(
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
                choice = self._pick_far_cell()
                if choice is not None:
                    if self._travel_and_attempt(*choice) == 'caduta':
                        continue
                else:
                    if len(self.frontier) > 0:
                        removed = self.frontier[0]
                        self.frontier.remove(removed)
                        self._dropped_cells.append((int(removed[0]), int(removed[1])))
                        print(f"[CELL FRONTIER] Cell ({removed[0]},{removed[1]}) removed from the frontier: in "
                              f"this round no candidate has a side left to try.")

            if len(self.frontier) == 0:
                break
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
              + (f"; removed from the frontier with no sides left to try: "
                 f"{', '.join(f'({r},{c})' for r, c in self._dropped_cells)}" if self._dropped_cells else "")
              + ". The others were never reachable from a visited cell.")

    # --- Progress ------------------------------------------------------------------------

    def _cell_counts(self):
        """(closed, visited, blocked, total): closed = visited or blocked."""
        vals = [v for row in self.env.map for v in row]
        visited = sum(1 for v in vals if v == 1)
        blocked = sum(1 for v in vals if v == -1)
        return visited + blocked, visited, blocked, len(vals)

    def _seen_area_pct(self):
        """Fraction of the mission rectangle already observed (free or obstacle) in the global
        map, on points every 0.1 m. None if there is no map yet."""
        gm = self.global_grid.global_occupancy_map
        if gm is None:
            return None
        if self._area_points is None:
            env = self.env
            k = np.arange(-env.cell_size / 2.0 + 0.05, env.cell_size / 2.0, 0.1)
            ox, oy = np.meshgrid(k, k)
            c, s_ = np.cos(env.origin_yaw), np.sin(env.origin_yaw)
            pts = []
            for r in range(env.rows):
                for col in range(env.cols):
                    cx, cy = env.get_world_position_from_cell(r, col)
                    pts.append(np.c_[cx + ox.ravel() * c - oy.ravel() * s_,
                                     cy + ox.ravel() * s_ + oy.ravel() * c])
            self._area_points = np.vstack(pts)
        res = gm.resolution
        keys = np.floor(self._area_points / res).astype(int)
        return sum(1 for gx, gy in keys if (gx, gy) in gm.grid) / len(keys)

    def _log_progress(self, final=False):
        """
        Where the mission stands: a [PROGRESS] line when the closed cells change (visited
        or blocked), and one at the end. Closed cells over the total (the main number),
        mission area already seen in the global map, time, estimate of the remaining time
        from the average time per cell, battery. Log and MISSION_STATS['progress'] only
        (report, mission_summary.json): no decision depends on it.
        """
        closed, visited, blocked, total = self._cell_counts()
        if not final and closed == self._closed_logged:
            return
        self._closed_logged = closed
        try:
            area = self._seen_area_pct()
        except Exception:
            area = None
        minutes = (time.time() - self._t0) / 60.0 if self._t0 is not None else 0.0
        done_here = closed - self._closed_at_start
        per_cell = minutes / done_here if done_here > 0 else None
        # The last charge read by the battery monitor (battery.py), without an extra reading.
        battery = MISSION_STATS.get('battery_last_pct')
        pct = 100.0 * closed / total
        area_txt = f"area seen {100.0 * area:.0f}%" if area is not None else "area seen ?"
        bat_txt = f"battery {battery:.0f}%" if battery is not None else "battery ?"
        if final:
            time_txt = (f"{minutes:.1f} min of exploration"
                     + (f" ({per_cell:.2f} min per cell)" if per_cell else "")
                     + (f", {total - closed} cells never reached" if total > closed else ""))
        else:
            time_txt = (f"{minutes:.1f} min elapsed"
                     + (f", ~{(total - closed) * per_cell:.0f} min to the end ({per_cell:.2f} min per cell)"
                        if per_cell else ""))
        print(f"[PROGRESS] {'End of exploration: ' if final else ''}{closed}/{total} cells closed "
              f"({pct:.0f}%): {visited} visited, {blocked} blocked, {total - closed} to do | "
              f"{area_txt}\n            {time_txt} | {bat_txt}")
        MISSION_STATS['progress'] = {
            'closed_cells': closed, 'total_cells': total, 'percent': round(pct, 1),
            'visited': visited, 'blocked': blocked,
            'seen_area': None if area is None else round(100.0 * area, 1),
            'minutes': round(minutes, 1), 'minutes_per_cell': None if per_cell is None else round(per_cell, 2),
            'battery': battery, 'final': bool(final)}

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
            if P.GRAPHNAV_PRECHECK:
                cand_wp = recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell)
                wp_xy = (cand_wp.get('x'), cand_wp.get('y')) if isinstance(cand_wp, dict) else (None, None)
                reach = (mission_prm.graph_reaches_cell(self.prm, env, wp_xy, candidate_row, candidate_col,
                                                          self.global_grid.global_occupancy_map)
                         if wp_xy[0] is not None else None)
                if reach is False:
                    MISSION_STATS['graphnav_trips_avoided'] += 1
                    print(f"[PRECHECK] Cell ({candidate_row},{candidate_col}): from the waypoint "
                          f"of cell {candidate_nearest_cell} the graph does not reach any "
                          f"node of the cell (obstacles already in the map). I mark the side as "
                          f"tried without moving.")
                    env.mark_cell_side_explored(candidate_row, candidate_col, candidate_side_bit)
                    recovery.finalize_or_defer_blocked_cell(env, candidate_row, candidate_col,
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

    def final_scan(self):
        """The area around the last position, before going back: it goes into the global maps."""
        print("[FINAL SCAN] Last scan around the robot before going back to the start...")
        x_final, y_final, z_final, quat_final = connection.getPosition(self.robot_state_client)
        final_yaw = np.arctan2(2.0 * (quat_final.w * quat_final.z + quat_final.x * quat_final.y),
                               1.0 - 2.0 * (quat_final.y ** 2 + quat_final.z ** 2))

        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[FINAL SCAN] CAREFUL: local grid not received, carrying on without the final scan.")
            return
        self.local_grid.process_scan(scan, x_final, y_final, final_yaw, ground_label="scan-final")
        # The last scan goes into the global maps too, like all the others
        global_maps.fold_scan_into_global_maps(
            self.global_grid.global_occupancy_map, self.global_grid.global_terrain_map, scan.pts,
            scan.terrain, scan.obstacle_mask, scan.is_valid, scan.od,
            (scan.num_y, scan.num_x), (x_final, y_final, final_yaw))

        final_save_path = os.path.join(
            self.mission_folder, f"iteration_{self.visualization_counter}_FINAL_SCAN.png"
        ) if self.mission_folder else None

        visualize_grid_with_candidates(
            pts=scan.pts, terrain_real=scan.terrain, obstacle_mask=scan.obstacle_mask,
            robot_x=x_final, robot_y=y_final,
            candidates={'rejected': [], 'valid': []},
            chosen_point=None, iteration=self.visualization_counter, env=self.env,
            save_path=final_save_path, prm_graph=self.prm, chosen_path=None,
            cells_obstacle_dist=scan.od, valid_values=scan.valid,
            grad_values=scan.grad, rough_values=scan.rough
        )
        self.visualization_counter += 1
        print("[FINAL SCAN] Done: the last scan is in the global maps.")
