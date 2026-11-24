"""The mission cell by cell: Mission, choice of the cell to attempt, trips with GraphNav."""
import os
import time
import numpy as np

from spot_explorer.graphnav import map_shortcuts
from spot_explorer.graphnav import navigation
from spot_explorer.robot import motion
from spot_explorer.perception import global_maps
from spot_explorer.mission import attempt_cruise
from spot_explorer.mission import cells
from spot_explorer.robot import connection


from spot_explorer import config as P
from spot_explorer.records import HANDOFF, MISSION_STATS, visualize_grid_with_candidates
from spot_explorer.mission import attempt_common, cell_attempt
from spot_explorer.mission import cell_rules


# =========================================================================
# The mission: the cell-by-cell loop, start-up and closure
# =========================================================================

# Before moving towards a cell (GraphNav trip or attempt), the frontier cells already seen
# for at least attempt_common.CELL_SEEN_FRACTION in the global map are marked visited where
# they are (only with attempt_common.CELL_SEEN_EXPLORED). The GraphNav waypoint of the cell is
# made only if the robot is in it (or just outside, ARRIVAL_OUTSIDE_CELL_M), as after an
# attempt; a cell seen from afar gets none. This saves GraphNav trips towards cells that are
# already seen from where the robot stands.
SEEN_BEFORE_MOVING = True


class Mission:
    """The mission cell by cell, after start-up (see run.py): explore() with the cell rules
    (cell_rules): at every turn the cells already seen are closed, then the next cell is the
    first in the visiting order with a near way (attempt from here, in cruise; the cruise goes
    on into the next cells), otherwise the first with any way (GraphNav, then the attempt).

    On low battery (see battery.py) exploration stops even with cells left: the saved state
    tells what was left to explore.
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
        self.path = env.generate_visit_path(P.MISSION_VISIT_ORDER, start_cell=env.start_cell)
        print(f"[CONFIG] Visiting order: {env.visit_order_name(P.MISSION_VISIT_ORDER)} "
              f"(from {P.MISSION_VISIT_ORDER}), first cells "
              + " ".join(f"({r},{c})" for r, c in self.path[:8])
              + (" ..." if len(self.path) > 8 else ""))
        self.visualization_counter = 0   # running number of the saved images
        # Progress (see _log_progress): start instant, closed cells at the start and
        # at the last printed line, points of the mission rectangle for the seen area.
        self._t0 = None
        self._closed_at_start = 0
        self._attempt_count = 0               # attempts to enter a cell
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
        HANDOFF.mark("end of the attempt")
        self.visualization_counter += 1
        self._attempt_count += 1
        return check

    def _on_visited(self, row, col, x, y):
        """Cell (row, col) visited with the robot at (x, y): GraphNav waypoint of the cell and
        the cell marked visited. The waypoint (and the robot cell of the grid) only if the robot
        is in the cell or within ARRIVAL_OUTSIDE_CELL_M of it: a cell seen from afar (at the
        attempt start, or in cruise) gets no waypoint of its own. Also for the cells done in
        cruise (env._on_chained_visit), called once the robot walks towards the next cell."""
        border = cells.cell_border_distance(self.env, row, col, x, y)
        if border is not None and -border <= P.ARRIVAL_OUTSIDE_CELL_M:
            self.env.update_position(x, y)
            self.recordingInterface.create_default_waypoint(cell_row=row, cell_col=col)
        else:
            print(f"[VISITED] Cell ({row},{col}) seen from {-border:.1f} m outside: no GraphNav "
                  f"waypoint of its own.")
        HANDOFF.mark("waypoint of the visited cell")
        self.env.mark_cell_visited(row, col)

    def explore(self):
        """
        The cells with the general rules (see cell_rules): at every turn the cells already
        seen are closed, then the next cell is the first in the visiting order with a way of
        at most NEAR_PATH_M in the graph of now (attempt from here, in cruise), otherwise
        the first with any way (GraphNav to the waypoint nearest to it, then the attempt).
        A real failure counts against the cell with its entry; no candidate = the end.
        """
        rules = self.rules = cell_rules.CellRules(self.env, self.prm, self.path)
        # For the next cell prepared in the cruise (see attempt_cruise).
        self.env._cell_rules = rules
        self.env._on_chained_visit = self._on_visited
        self._t0 = time.time()
        self._closed_at_start = self._cell_counts()[0]
        while True:
            self._log_progress()
            if attempt_common.battery_return_due():
                left = [(r, c) for r in range(self.env.rows) for c in range(self.env.cols)
                        if self.env.map[r][c] == 0]
                MISSION_STATS['battery_return_cells_left'] = len(left)
                print(f"[BATTERY] End of exploration for low battery: {len(left)} cells not done.")
                break
            x, y, _, _ = connection.getPosition(self.robot_state_client)
            robot_cell = tuple(int(v) for v in self.env.get_cell_from_world(x, y))
            self._close_seen_cells(lambda: rules.open_cells(robot_cell))
            HANDOFF.mark("progress and seen cells")
            x, y, _, _ = connection.getPosition(self.robot_state_client)
            choice = rules.choose((x, y), self.global_grid.global_occupancy_map, self.sampler,
                                  waypoints=self.recordingInterface.get_all_manual_waypoints_with_cells())
            if choice is None:
                break
            print(f"[CELL RULES] Next: {choice.describe()}.")
            MISSION_STATS['cell_choices_near' if choice.near else 'cell_choices_far'] = \
                MISSION_STATS.get('cell_choices_near' if choice.near else 'cell_choices_far', 0) + 1
            self.env._chain_choice = None
            self.env._attempt_left_unreachable = False
            self.env._attempt_fail_why = None
            self.env._cells_left_unreachable = []
            if choice.near:
                ok = self._attempt(choice.row, choice.col)
            else:
                attempt_cruise.stop_if_moving(self.env, self.command_client, "next cell far: GraphNav")
                ok = self._travel_to_choice(choice)
            # The cruise may have gone on into other cells (see attempt_cruise): the outcome
            # is that of the last one.
            choice = getattr(self.env, '_chain_choice', None) or choice
            if ok == 'fall':
                rules.record_failure(choice, "fall during the GraphNav move")
                continue
            x, y, _, _ = connection.getPosition(self.robot_state_client)
            inside = tuple(int(v) for v in self.env.get_cell_from_world(x, y)) == (choice.row, choice.col)
            if ok or inside:
                # Failed but the robot is inside the cell anyway: it counts as visited.
                self._on_visited(choice.row, choice.col, x, y)
            elif attempt_common.battery_low():
                continue
            elif getattr(self.env, '_attempt_left_unreachable', False):
                # No goal of the cell has a path any more (rule 3): not failed, skipped once.
                rules.skip_once.add((choice.row, choice.col))
                print(f"[CELL RULES] Cell ({choice.row},{choice.col}) unreachable now (no goal with a "
                      f"path): not a failure, skipped at the next choice.")
            else:
                default = "attempt failed" if (choice.near or ok is not None) else "GraphNav did not get there"
                rules.record_failure(choice, getattr(self.env, '_attempt_fail_why', None) or default)
            rules.skip_once |= set(map(tuple, getattr(self.env, '_cells_left_unreachable', [])))
        attempt_cruise.stop_if_moving(self.env, self.command_client, "end of the exploration")
        rules.explain_end(self._attempt_count)
        self._log_progress(final=True)

    def _travel_to_choice(self, choice):
        """GraphNav to the waypoint nearest to the node the way enters the cell at, then the
        attempt. True/False as the attempt, None if GraphNav did not get there, 'fall'."""
        if choice.waypoint is not None:
            cell, wp = choice.waypoint          # the one the choice checked the goal from
        else:
            wps = self.recordingInterface.get_all_manual_waypoints_with_cells()
            if not wps:
                return None
            nx, ny = choice.node_xy
            cell, wp = min(wps.items(), key=lambda kv: np.hypot(kv[1]['x'] - nx, kv[1]['y'] - ny))
        res = self._graphnav_to(wp, f"to the waypoint of cell {cell} for cell ({choice.row},{choice.col})",
                                self.env.get_world_position_from_cell(choice.row, choice.col))
        if res != 'ok':
            return 'fall' if res == 'fall' else None
        if attempt_common.battery_return_due("after the GraphNav move"):
            return None
        return self._attempt(choice.row, choice.col)

    def _graphnav_to(self, wp, label, face_xy):
        """GraphNav trip to the waypoint wp (data of map_recording), with our shortcuts first:
        'ok', 'fail' (it did not get there) or 'fall' (it fell and got up)."""
        recordingInterface = self.recordingInterface
        x_current, y_current, _, _ = connection.getPosition(self.robot_state_client)
        map_shortcuts.graphnav_add_shortcuts(recordingInterface, "before the move between cells",
                                             map_edges=True, target_wp_id=wp['id'],
                                             robot_xy=(x_current, y_current))
        HANDOFF.report("GraphNav trip")
        recordingInterface.stop_recording()
        ok = navigation.graphnav_navigate_to(recordingInterface, wp['id'], self.robot_state_client,
                                             self.command_client, label, face_xy=face_xy)
        # GraphNav does not record the legs travelled: the old ones no longer describe the way
        # the robot came, the retreat must not use them.
        self.env._traveled_arcs = []
        fell = motion.check_and_recover_fall(self.robot_state_client, self.command_client,
                                             self.global_grid.global_occupancy_map,
                                             "during the move with GraphNav") is not None
        recordingInterface.start_recording()
        if fell:
            print("[FALL] Got up after the move with GraphNav.")
            return 'fall'
        return 'ok' if ok else 'fail'

    def _close_seen_cells(self, candidates):
        """SEEN_BEFORE_MOVING: the candidate cells (candidates(): the open cells, in visiting
        order) already seen enough become visited without moving; the cells they open are
        checked too (candidates() asked again after every one). Returns how many."""
        if not (SEEN_BEFORE_MOVING and attempt_common.CELL_SEEN_EXPLORED):
            return 0
        gmap = self.global_grid.global_occupancy_map
        if gmap is None:
            return 0
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        closed, changed = 0, True
        while changed:
            changed = False
            for r, c in candidates():
                r, c = int(r), int(c)
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
                if here:
                    self._on_visited(r, c, x, y)
                else:
                    self.env.mark_cell_visited(r, c)
                MISSION_STATS['cells_seen_before_moving'] = MISSION_STATS.get('cells_seen_before_moving', 0) + 1
                closed += 1
                changed = True
                break
        return closed

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

    def final_scan(self):
        """The area around the last position, before going back: it goes into the global maps."""
        print("[FINAL SCAN] Last scan around the robot before going back to the start...")
        x_final, y_final, z_final, quat_final = connection.getPosition(self.robot_state_client)
        final_yaw = connection.yaw_of(quat_final)

        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[FINAL SCAN] CAREFUL: local grid not received, carrying on without the final scan.")
            return
        # A mission that ended before any attempt has no global maps yet.
        global_maps.ensure_global_maps(self.global_grid, scan.cell_size)
        self.local_grid.process_scan(scan, x_final, y_final, final_yaw, ground_label="scan-final")
        # The last scan goes into the global maps too, like all the others
        global_maps.fold_scan_into_global_maps(
            self.global_grid.global_occupancy_map, self.global_grid.global_terrain_map, scan.pts,
            scan.terrain, scan.obstacle_mask, scan.is_valid, scan.od,
            (scan.num_y, scan.num_x), (x_final, y_final, final_yaw))
        global_maps.save_global_maps(self.global_grid, self.mission_folder, force=True)

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
