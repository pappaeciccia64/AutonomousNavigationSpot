"""One attempt to enter a cell: CellAttempt, the navigation loop with the safe frontier."""
import os
import time
from datetime import datetime
import numpy as np

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from types import SimpleNamespace
from spot_explorer.graphnav import map_recording
from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import slopes
from spot_explorer.perception import spot_grid
from spot_explorer.mission import cells as environmentMap
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier

from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.planning import roadmap as prm_graph_module   # inside functions `prm_graph` is often the parameter (instance)
from spot_explorer.planning import mission_graph                   # mission node lattice

from spot_explorer import config as P
from spot_explorer.records import (MISSION_STATS, MISSION_CTX, BodyTiltRecorder, TrajectoryLog, append_csv, append_trajectory, log_segment, mission_z0, save_npz_async, timing_add, visualize_grid_with_candidates)
from spot_explorer.planning import goal
from spot_explorer.mission import recovery


def _battery_return_due(where=None):
    """One battery reading (see battery.BatteryMonitor.check): True if the mission must stop
    exploring and return. False if there is no monitor."""
    monitor = MISSION_CTX.get('batteria')
    return monitor.check(where) if monitor is not None else False


def _battery_low():
    """Like _battery_low but without reading: only if the return has already been decided."""
    monitor = MISSION_CTX.get('batteria')
    return monitor is not None and monitor.return_due


# =========================================================================
# ENTERING A CELL: one attempt (CellAttempt)
# =========================================================================


# Outcomes of a phase of the navigation loop (see CellAttempt._loop_once): None = go on
# with the next phase.
CONTINUE = "continua"        # turn finished: start again from the scan
STOP = "interrompi"    # end of the loop: the final position decides (CellAttempt._finish)
FAILED = "fallito"          # the attempt failed


class CellAttempt:
    """Try to enter cell (target_row, target_col) from the robot's current position.

    Scan, update of maps and graph, choice of the goal point, path, then the navigation
    loop with the safe frontier. True if the robot ends up inside the cell.

    mobility_kwargs: optional dict forcing the gait (locomotion_hint, ground_mu_hint,
        swing_height) for all legs, passed down to motion.relative_move through
        navigate_to. Empty = gait chosen leg by leg (select_mobility_params).

    HOW IT IS MADE. _prepare once (_first_scan, _local_data_into_graph, _goal_and_path,
    _reset_counters), then _loop_once until a phase decides the outcome:

      _guards                 too many turns, low battery, fall, robot still for too long
      _scan_and_graph         _fresh_scan, _update_graph, _replan
      _frontier_and_decision  shortcut, safe frontier, decisions.decide_next_move:
                              move / reached_node / arrived / wait / blocked
      _rotation               (move) cannot turn: _advance_in_cone, _back_up_to_turn,
                              trap, leg discarded
      _node_or_arrival        (reached_node, arrived)
      _wait                   (wait) _repick_unreachable_goal, _replan_now,
                              otherwise look again standing still
      _block                  (blocked) goal to change, trap, leg out for the mission,
                              new way or return with GraphNav
      _execute_legs           (move) _check_arrival_pose, then for every leg
                              _prepare_leg and _walk_leg (_leg_terrain,
                              _move_and_record, outcome)

    Actions shared by the phases: _use_path, _drop_edge (for now / for the attempt / for
    the mission), _walked_path and _stop_node_here, _escape_fan and _escape_trap,
    _repick_goal, _go_back (GraphNav backwards, see _trap_backtrack).
    """

    def __init__(self, local_grid, global_grid, robot_state_client, command_client,
                 env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                 iteration=0,
                 recordingInterface=None, verification_tracker=None, mobility_kwargs=None):
        self.local_grid = local_grid
        self.global_grid = global_grid
        self.robot_state_client = robot_state_client
        self.command_client = command_client
        self.env = env
        self.target_row = target_row
        self.target_col = target_col
        self.sampler = sampler
        self.prm_graph = prm_graph
        self.mission_folder = mission_folder
        self.iteration = iteration
        self.recordingInterface = recordingInterface
        self.verification_tracker = verification_tracker
        self.mobility_kwargs = mobility_kwargs

    def run(self):
        """True if the robot ends up inside the cell (or accepted just outside, see _finish)."""
        if self._prepare() is FAILED:
            return False

        while self.path_waypoints and len(self.path_waypoints) > 0:
            outcome = self._loop_once()
            if outcome is FAILED:
                return False
            if outcome is STOP:
                break
        return self._finish()

    def _loop_once(self):
        """One turn of the navigation loop: the phases in order, until one decides the outcome."""
        for phase in (self._guards, self._scan_and_graph, self._frontier_and_decision,
                     self._rotation, self._timings, self._node_or_arrival, self._wait,
                     self._block, self._execute_legs):
            outcome = phase()
            if outcome is not None:
                return outcome
        return CONTINUE

    def _prepare(self):
        """Before the loop: get up if fallen, first scan, maps and graph, goal point and path,
        attempt counters. FAILED if it cannot even start."""
        self.mobility_kwargs = self.mobility_kwargs or {}

        print(f"\n[ATTEMPT] Provo a entrare nella cella ({self.target_row},{self.target_col}) dalla posizione attuale...")

        scan = self._first_scan()
        if scan is None:
            return FAILED
        self._local_data_into_graph(scan)
        outcome = self._goal_and_path(scan)
        if outcome is not None:
            return outcome
        self._reset_counters()

    def _first_scan(self):
        """Get up if fallen, first scan of the attempt, robot pose, global maps updated and
        saved. The processed scan, or None if it does not arrive."""
        # The robot gets here from another move (navigation between cells, retreat): if it
        # fell in the meantime it gets up before anything else. Raises RobotFallenError if it cannot.
        # Then it leaves the hazard disk by retracing its steps: once up, it is still INSIDE
        # the spot of the fall, and from there every PRM edge would be discarded.
        if movements.check_and_recover_fall(self.robot_state_client, self.command_client,
                                   self.global_grid.global_occupancy_map, "all'inizio del tentativo") is not None:
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)

        # 1. The scan: the layers in one request, then (below) their processing
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Rilevamento griglie locali fallito.")
            return None
        main_proto = scan.proto
        pts, cells_obstacle_dist = scan.pts, scan.od
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size

        # =========================================================================
        # GLOBAL MAPS: created at the first scan, then accumulated for the whole mission
        # =========================================================================
        if self.global_grid.global_occupancy_map is None:
            self.global_grid.global_occupancy_map = global_maps.GlobalGrid(resolution=cell_size)
            print("[MAP] Inizializzata nuova mappa di occupazione globale persistente.")

        # Global height map: same native resolution as the local grid (no
        # reduction), created like the occupancy one.
        if self.global_grid.global_terrain_map is None:
            self.global_grid.global_terrain_map = global_maps.GlobalTerrainGrid(resolution=cell_size)
            print("[MAP] Inizializzata nuova mappa globale persistente del terreno (altezza).")

        self.global_map = self.global_grid.global_occupancy_map
        self.global_terrain_map = self.global_grid.global_terrain_map

        transforms_snapshot = main_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        self.robot_x, self.robot_y, self.robot_z = vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

        quat = vision_tform_body.rotation
        self.robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))

        # 2. Body footprint, corrected terrain (from here on only terrain_real is used),
        # slope and roughness, obstacle mask
        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-iniziale")
        self.footprint_mask_2d, terrain_real, is_valid = scan.footprint_mask, scan.terrain, scan.is_valid
        obstacle_mask = scan.obstacle_mask

        # Update the global maps (occupancy and terrain) with the data just processed
        global_maps.fold_scan_into_global_maps(self.global_map, self.global_terrain_map, pts, terrain_real, obstacle_mask,
                                   is_valid, cells_obstacle_dist, (num_y, num_x),
                                   (self.robot_x, self.robot_y, self.robot_yaw))

        # The terrain map serves the PRM as a fallback for the slope
        self.prm_graph.set_global_terrain_map(self.global_terrain_map)

        # Global maps on disk (.pkl), at every attempt
        if self.mission_folder is not None:
            pkl_map_path = os.path.join(self.mission_folder, "global_occupancy_map.pkl")
            self.global_map.save_map(pkl_map_path)
            pkl_terrain_path = os.path.join(self.mission_folder, "global_terrain_map.pkl")
            self.global_terrain_map.mission_z0 = MISSION_CTX.get('mission_z0')
            self.global_terrain_map.save_map(pkl_terrain_path)
        return scan

    def _local_data_into_graph(self, scan):
        """Terrain, slope and obstacles of the scan into the PRM, then edge update."""
        pts, cells_obstacle_dist, valid_values = scan.pts, scan.od, scan.valid
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size
        terrain_real, is_valid = scan.terrain, scan.is_valid
        grad_values, rough_values = scan.grad, scan.rough
        grad_for_prm = grad_values.copy()
        rough_for_prm = rough_values.copy()
        main_proto = scan.proto
        transforms_snapshot = main_proto.local_grid.transforms_snapshot

        # ================================================================== #
        # LOCAL DATA INTO THE PRM: terrain, slope and roughness of the scan
        # ================================================================== #
        local_x_min, local_x_max = pts[:, 0].min(), pts[:, 0].max()
        local_y_min, local_y_max = pts[:, 1].min(), pts[:, 1].max()

        grid_frame_name = main_proto.local_grid.frame_name_local_grid_data
        vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
        grid_origin_x = vision_tform_grid.position.x
        grid_origin_y = vision_tform_grid.position.y

        grad_values_2d = grad_for_prm.reshape((num_y, num_x))
        rough_values_2d = rough_for_prm.reshape((num_y, num_x))
        terrain_values_2d = terrain_real.reshape((num_y, num_x))

        self.prm_graph.update_local_grid_data(
            grad_values_2d=grad_values_2d,
            rough_values_2d=rough_values_2d,
            terrain_values_2d=terrain_values_2d,
            valid_2d=spot_grid.slope_usable_mask(is_valid, cells_obstacle_dist, (num_y, num_x)),
            grid_origin_x=grid_origin_x,
            grid_origin_y=grid_origin_y,
            cell_size=cell_size
        )

        # Gradient and roughness of the PRM nodes inside the local grid; those outside keep the
        # value they have (0.0 if they do not have one yet)
        for node_id, (nx, ny) in self.prm_graph.nodes.items():
            if local_x_min <= nx <= local_x_max and local_y_min <= ny <= local_y_max:
                dx = nx - grid_origin_x
                dy = ny - grid_origin_y

                col_idx = int(dx / cell_size)
                row_idx = int(dy / cell_size)

                if 0 <= row_idx < num_y and 0 <= col_idx < num_x:
                    flat_idx = row_idx * num_x + col_idx
                    if flat_idx < len(grad_values):
                        self.prm_graph.node_gradients[node_id] = grad_values[flat_idx]
                    if flat_idx < len(rough_values):
                        self.prm_graph.node_roughness[node_id] = rough_values[flat_idx]
            else:
                if node_id not in self.prm_graph.node_gradients:
                    self.prm_graph.node_gradients[node_id] = 0.0
                if node_id not in self.prm_graph.node_roughness:
                    self.prm_graph.node_roughness[node_id] = 0.0

        # The scan just made also goes into the preliminary edge discard
        # (roadmap.LIVE_PREFILTER_OD_M): at build time the global map has not yet
        # confirmed anything about this place, and the edges crossing the obstacles just
        # seen would stay valid. Here obstacle_distance is the raw one: the filtered
        # snapshot is born in the navigation loop.
        try:
            self.prm_graph.set_live_obstacles(
                np.asarray(cells_obstacle_dist, dtype=np.float64).reshape((num_y, num_x)),
                np.asarray(valid_values, dtype=np.float64).reshape((num_y, num_x)) >= spot_grid.TERRAIN_VALID_MIN,
                grid_origin_x, grid_origin_y, cell_size, self.robot_x, self.robot_y, self.robot_yaw)
        except Exception as e:
            self.prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
            print(f"[ARCHI] Scansione non usabile per lo scarto preliminare ({e}).")

        t_build = time.perf_counter()
        # CAREFUL WITH THE NAMES: here `self.prm_graph` is the PRM instance, not the module of the
        # same name (imported as prm_graph_module). The candidate pairs are those of the mission
        # graph (see mission_graph.build_mission_prm); vetoes and weights are computed by
        # PRM._evaluate_and_add_edge.
        mission_graph.rebuild_prm_edges(self.prm_graph, self.global_map)
        timing_add('costruzione grafo', time.perf_counter() - t_build)
        _upd = getattr(getattr(self.prm_graph, 'mission_graph', None), 'last_update', None)
        print(f"[TEMPO] {'aggiornamento locale' if _upd and _upd[2] else 'costruzione completa'} "
              f"del grafo: {time.perf_counter() - t_build:.2f} s")
        if getattr(self.prm_graph, 'live_veto_last', 0):
            print(f"[ARCHI] {self.prm_graph.live_veto_last} archi scartati subito dalla scansione dal "
                  f"vivo (passano a meno di {prm_graph_module.LIVE_PREFILTER_OD_M:.2f} m da un "
                  f"ostacolo appena visto), prima della pendenza.")
        # A new attempt: the traps of the previous attempt no longer apply (see
        # _trap_backtrack). The penalties are a layer separate from the edges, so
        # they survive the rebuilds WITHIN the attempt.
        self.prm_graph.clear_penalties_for_attempt()

    def _goal_and_path(self, scan):
        """Robot node, goal point in the cell, starting breadcrumb and first path.
        FAILED if there is no valid goal point in the cell."""
        pts, cells_obstacle_dist, valid_values = scan.pts, scan.od, scan.valid
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size
        terrain_real, obstacle_mask = scan.terrain, scan.obstacle_mask
        grad_values, rough_values = scan.grad, scan.rough
        save_path = os.path.join(self.mission_folder,
                                 f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}.png") if self.mission_folder else None

        # Only visited cells + robot cell + goal cell (see PLAN_ONLY_EXPLORED_CELLS).
        mission_graph.restrict_planning_to_explored_cells(self.prm_graph, self.env, self.target_row, self.target_col,
                                             (self.robot_x, self.robot_y))

        # --- Start and arrival nodes: EXISTING graph nodes ------------------------------------
        # add_stop_node attaches the position to the suitable node instead of creating one. The
        # robot node is needed BEFORE the goal choice: the choice is among the points the graph
        # connects to the robot (see find_best_point_in_cell, criterion 2).
        self.robot_node_id = self.prm_graph.add_stop_node(
            self.robot_x, self.robot_y, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
            label="partenza tentativo")
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)


        # Search for the target point and planning
        self.target_x, self.target_y, valid_samples, rejected_samples = goal.find_best_point_in_cell(
            self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, pts, cells_obstacle_dist, self.sampler,
            global_map=self.global_map, reachable=self._reachable_from_robot)

        # The goal of this attempt, for the recovery right after a failure
        # (see RECOVERY_SAME_GOAL_M in the main loop).
        self.env._last_attempt_goal = (None if self.target_x is None or self.target_y is None
                                  else (float(self.target_x), float(self.target_y)))
        if self.target_x is None or self.target_y is None:
            print(f"[FAIL] Nessun obiettivo valido nella cella ({self.target_row},{self.target_col}) dalla posizione attuale")
            visualize_grid_with_candidates(
                pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': rejected_samples, 'valid': valid_samples},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=save_path, prm_graph=self.prm_graph, chosen_path=None,
                cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
                grad_values=grad_values, rough_values=rough_values
            )
            return FAILED

        # GraphNav breadcrumb at the attempt's starting point: it is the one the robot needs to
        # be able to go back to if the cell turns out to be a dead end.
        map_recording.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client, self.robot_node_id,
                                 self.target_row, self.target_col, force=True)
        # The goal is the centre or a lattice node (find_best_point_in_cell chooses it among
        # the sampler points).
        self.goal_id = self.prm_graph.add_stop_node(
            self.target_x, self.target_y, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"obiettivo cella ({self.target_row},{self.target_col})", is_robot=False)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
        path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)

        if path_ids is None:
            print(
                f"[GRAFO DISCONNESSO] Nessun percorso nel PRM dal nodo {self.robot_node_id} al nodo "
                f"{self.goal_id}: i due nodi stanno in componenti separate del grafo. Non e' un "
                f"errore di Dijkstra -- vuol dire che tutti gli archi di collegamento sono "
                f"stati vetati (occupazione o pendenza). Vedi i conteggi in [PRM MISSION SUMMARY].")
            print(f" -> Coordinate partenza robot: ({self.robot_x:.2f}, {self.robot_y:.2f})")
            print(f" -> Coordinate target cella: ({self.target_x:.2f}, {self.target_y:.2f})")


        self.full_path_coords = None
        self.path_waypoints = []

        if path_ids is not None:
            self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in path_ids]
            self.path_waypoints = self._waypoints_from_ids(path_ids)
        else:
            self.path_waypoints = None

        visualize_grid_with_candidates(
            pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
            robot_x=self.robot_x, robot_y=self.robot_y,
            candidates={'rejected': rejected_samples, 'valid': valid_samples},
            chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
            save_path=save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
            cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
            grad_values=grad_values, rough_values=rough_values
        )
        return None

    def _reset_counters(self):
        """Counters and memories of the attempt, before the first turn."""
        # ===================================================================================
        # NAVIGATION LOOP WITH THE SAFE FRONTIER
        #
        # At every turn: fresh scan, update of maps and PRM, replanning if needed,
        # then the FRONTIER: how far the path is confirmed free now. The robot advances
        # up to there (minus half a body and margin), along the path.
        #   - the nose ALWAYS points at the next point: no sideways walking; if the
        #     rotation is not possible the cone is tried, then the straight back-off (see
        #     plan_cone_move and plan_straight_retreat);
        #   - the rotation is judged on the cells really swept (geometry.rotation_is_clear),
        #     not on the circumscribed circle;
        #   - if the goal turns out unreachable on the fresh data another one in the cell is
        #     chosen, instead of waiting and declaring a block (goal_still_reachable).
        # ===================================================================================
        self.replan_counter = 0
        self.need_refresh = True        # first turn of the attempt: see EVENT REFRESH
        self.no_progress = 0
        self.block_replans = 0
        self.loop_iter = 0
        self.rotation_fails = {}     # leg -> how many times the rotation towards it was refused
        self.rotation_discards = 0   # discards for rotation refusal in THIS attempt
        self.best_goal_dist = float('inf')   # to reset the refusals only on PROGRESS
        self.idle_loops = 0          # consecutive turns without moving (see MAX_IDLE_LOOPS)
        self.last_xy = None          # position at the previous turn, to count the idle turns
        self.rotation_fail_xy = {}   # leg -> robot position at the last refusal
        self.retreats_in_a_row = 0   # consecutive back-offs: reset by a real advance
        self.early_replans = 0       # replans at the first blocking cell ('wait' branch)
        self.arrival_refusals = 0    # steps refused because the arrival pose has no exits
        self.early_replanned_edges = set()   # legs already discarded by [SUBITO] in this attempt
        self.goal_repicks = 0        # how many times the goal point was chosen again
        self.tried_goals = []        # goal points already tried and found unreachable
        self.arrived_near_goal = False   # arrival accepted with the robot just outside the cell
        self.trap_state = {'n': 0}   # goings back in THIS attempt, never reset (see _trap_backtrack)

    def _reachable_from_robot(self, px, py):
        nid = self.prm_graph.anchor_node(px, py, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M)
        return nid is not None and self.prm_graph.find_path_dijkstra(self.robot_node_id, nid) is not None

    def _waypoints_from_ids(self, ids):
        """[(node_id, x, y), ...] without the first node (the robot position)."""
        out = []
        for nid in ids[1:]:
            nx, ny = self.prm_graph.get_node_position(nid)
            out.append((nid, nx, ny))
        return out

    # --- Actions shared by the decisions ---------------------------------------------------
    def _use_path(self, ids):
        """The path becomes ids (from the robot node on); the confirmation of a block starts
        again from zero."""
        self.path_waypoints = self._waypoints_from_ids(ids)
        self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in ids]
        self.no_progress = 0

    def _drop_edge(self, a, b, duration):
        """Remove leg a-b from the graph. duration:
          'ora'       -- until the next refresh, which puts it back if the data says it is free
                         (see roadmap.mark_edge_blocked_soft);
          'tentativo' -- for the rest of the attempt (attempt penalty);
          'missione'  -- forever: block confirmed while still or physical failure."""
        if duration == 'missione':
            self.prm_graph.mark_edge_invalid(a, b)
            return
        prm_graph_module.mark_edge_blocked_soft(self.prm_graph, a, b)
        if duration == 'tentativo':
            self.prm_graph.edge_penalty_persistent[(min(a, b), max(a, b))] = self.prm_graph._ATTEMPT_MARK

    def _walked_path(self, x0, y0):
        """The leg walked from (x0, y0) to the current position, for the retreat. Returns the
        current position."""
        ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
        if not hasattr(self.env, '_traveled_arcs'):
            self.env._traveled_arcs = []
        self.env._traveled_arcs.append((x0, y0, ax, ay))
        return ax, ay

    def _stop_node_here(self, x, y, label):
        """The position (x, y) becomes the robot node (see add_stop_node)."""
        self.robot_node_id = self.prm_graph.add_stop_node(
            x, y, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M, label=label)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)

    def _escape_fan(self):
        """The 24 exit directions from here (see decisions.is_trapped): (trapped?,
        usable directions, total, text for the log)."""
        trapped, fan = decisions.is_trapped((self.robot_x, self.robot_y), self.robot_yaw, self.snap)
        n_us, fan_txt = decisions.describe_escape_fan(fan)
        return trapped, n_us, len(fan), fan_txt

    def _escape_trap(self, edge_ab, message):
        """No exit direction: discarding edges is useless, the problem is the place. Go back
        with GraphNav (CONTINUE) or the attempt ends (FAILED)."""
        MISSION_STATS['traps_detected'] += 1
        print(message)
        if self._go_back(edge_ab, f"dal nodo {self.robot_node_id}"):
            return CONTINUE
        return FAILED

    def _go_back(self, block_arc, label, skip_if_graph_cannot_help=False):
        """Return with GraphNav (see _trap_backtrack) instead of walking backwards: go back one
        waypoint, try to plan again from there, and so on up to
        GRAPHNAV_MAX_BACKTRACK_HOPS; the walked retreat stays inside as the last net.
        True if it starts again with a new path (turn counters reset), False if the
        attempt must end."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()
        esc = recovery._trap_backtrack(
            self.trap_state, block_arc,
            self.env, self.prm_graph, self.goal_id, self.recordingInterface, self.robot_state_client,
            self.command_client, self.global_map, self.mobility_kwargs, self.pts_up, self.obstacle_mask_updated,
            label=label, skip_if_graph_cannot_help=skip_if_graph_cannot_help)
        if not esc['ok']:
            return False
        self.robot_node_id = esc['robot_node']
        self._use_path(esc['ids'])
        self.early_replans = 0
        self.retreats_in_a_row = 0
        self.rotation_discards = 0
        self.best_goal_dist = float('inf')
        return True

    def _frontier_stops_at_goal(self):
        """The frontier stops for an obstacle (or roughness, slope) right next to the goal
        (within REFUSED_GOAL_BLOCK_M) or beyond the end of the path, in the extension for the
        nose: no path to this goal can pass, the problem is the point."""
        if self.frontier['reason'] not in ('ostacolo', 'rugosita', 'pendenza'):
            return False
        sx, sy = self.frontier['stop_xy']
        return (self.frontier['dist'] >= self.frontier['path_len']
                or float(np.hypot(sx - self.target_x, sy - self.target_y)) <= P.REFUSED_GOAL_BLOCK_M)

    def _repick_goal(self, why, exclusion_reason):
        """
        Exclude the current goal (for the mission, see _refuse_goal) and choose another one
        in the cell, reachable from the robot. CONTINUE if there is a new goal with a path;
        None if there is none (the goal stays excluded, the caller carries on).
        At most MAX_BLOCK_REPLANS times per attempt.
        """
        self.goal_repicks += 1
        self.need_refresh = True          # plan changed: see EVENT REFRESH
        MISSION_STATS['goal_repicks'] += 1
        print(f"[TARGET] {why} Ne cerco un altro nella cella "
              f"({self.goal_repicks}/{P.MAX_BLOCK_REPLANS}).")
        self.tried_goals.append((float(self.target_x), float(self.target_y)))
        goal._refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                     exclusion_reason)
        # Up to GOAL_REPICK_CANDIDATES candidates: the first one may not have a path in the graph.
        # If none has one, goal and path go back to the previous ones (excluded for future
        # choices, but consistent with the path): before, the NEW goal remained with the OLD
        # path, towards the point just excluded.
        old = (self.target_x, self.target_y, self.goal_id)
        new_ids = None
        for _k in range(P.GOAL_REPICK_CANDIDATES):
            new_tx, new_ty, _vs, _rs = goal.find_best_point_in_cell(
                self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, self.pts_up, self.cells_obs_up,
                self.sampler, global_map=self.global_map, exclude=self.tried_goals,
                reachable=self._reachable_from_robot)
            if new_tx is None or new_ty is None:
                print("[TARGET] Nessun altro punto valido in questa cella.")
                break
            goal_id = self.prm_graph.add_stop_node(
                new_tx, new_ty, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                label=f"obiettivo riscelto cella ({self.target_row},{self.target_col})",
                is_robot=False)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, goal_id)
            if new_ids is not None:
                self.target_x, self.target_y, self.goal_id = new_tx, new_ty, goal_id
                break
            print(f"[TARGET] Nessun percorso nel grafo verso ({new_tx:.2f}, {new_ty:.2f}): provo il "
                  f"punto successivo.")
            self.tried_goals.append((float(new_tx), float(new_ty)))
        if new_ids is None:
            self.target_x, self.target_y, self.goal_id = old
            print("[TARGET] Nessun nuovo obiettivo con un percorso: resto sul piano di prima.")
            return None
        self.best_goal_dist = float('inf')    # new goal: progress from scratch
        self._use_path(new_ids)
        print(f"[REPLAN] Verso il nuovo obiettivo ({self.target_x:.2f}, "
              f"{self.target_y:.2f}): {len(self.path_waypoints)} waypoint.")
        return CONTINUE

    def _guards(self):
        """Loop limits: too many turns (MAX_LOOP_ITERATIONS), fall, too many still turns."""
        self.loop_iter += 1
        self.t_iter = time.perf_counter()
        if self.loop_iter > P.MAX_LOOP_ITERATIONS:
            print(f"[STALLO] {P.MAX_LOOP_ITERATIONS} giri del ciclo di navigazione senza arrivare: "
                  f"interrompo il tentativo sulla cella ({self.target_row},{self.target_col}).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED

        # Battery below threshold: the attempt stops here, the mission closes and returns
        # (see Mission.explore).
        if _battery_return_due():
            print(f"[BATTERIA] Interrompo il tentativo sulla cella ({self.target_row},"
                  f"{self.target_col}) per il rientro a batteria bassa.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED

        if movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                   "durante il ciclo di navigazione") is not None:
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)
            return FAILED

        self.robot_x, self.robot_y, self.robot_z, robot_quat = spotUtils.getPosition(self.robot_state_client)

        self.robot_yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                               1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))

        # --- Generic anti-stall guard: how many turns in a row without moving.
        if self.last_xy is not None and float(np.hypot(self.robot_x - self.last_xy[0], self.robot_y - self.last_xy[1])) < P.IDLE_MOVE_M:
            self.idle_loops += 1
        else:
            self.idle_loops = 0
        # The rotation refusals are reset only when the robot really gets CLOSER
        # to the goal (20 cm better than the best reached so far), not when it just
        # moves: unable to turn, the planner can take the robot one node further
        # on the wrong side and try again from there, and so on, moving away with the
        # counter always at zero (test_e2e_loop.py, scenario S11). Moving is not making
        # progress.
        d_goal_now = float(np.hypot(self.robot_x - self.target_x, self.robot_y - self.target_y))
        if d_goal_now < self.best_goal_dist - 0.20:
            self.best_goal_dist = d_goal_now
            self.rotation_discards = 0
        self.last_xy = (self.robot_x, self.robot_y)
        if self.idle_loops >= P.MAX_IDLE_LOOPS:
            MISSION_STATS['idle_loops'] += self.idle_loops
            print(f"[STALLO] {self.idle_loops} giri di ciclo consecutivi senza spostarsi di "
                  f"{P.IDLE_MOVE_M:.2f} m, con il robot in ({self.robot_x:.2f}, {self.robot_y:.2f}). "
                  f"Qualunque sia il motivo, continuare a girare qui non porta da nessuna "
                  f"parte: rinuncio a entrare in ({self.target_row},{self.target_col}) da questo lato.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED

    def _scan_and_graph(self):
        """Fresh scan, global maps, local data into the PRM and replanning if needed (EVENT
        REFRESH); the loop is left if there is no path any more."""
        outcome = self._fresh_scan()
        if outcome is not None:
            return outcome
        touched_nodes = self._update_graph()
        if touched_nodes:
            self._replan(touched_nodes)
        if not self.path_waypoints:
            return STOP

    def _fresh_scan(self):
        """Scan, processing, global maps, local data into the PRM, snapshot for the frontier
        and free-space map. FAILED if the scan does not arrive."""
        # 1. FRESH SCAN, before anything else
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Rilevamento locale fallito durante il tracking. Arresto precauzionale.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED
        self.t_scan = time.perf_counter()

        self.grids_data_up, main_proto_up = scan.grids_data, scan.proto
        self.lg_proto_up = main_proto_up
        self.pts_up, self.cells_obs_up = scan.pts, scan.od
        self.terrain_up, self.valid_up = scan.terrain_raw, scan.valid
        self.num_x_up, self.num_y_up, self.cell_size_up = scan.num_x, scan.num_y, scan.cell_size
        self.grid_origin_x_up, self.grid_origin_y_up = self.local_grid.grid_origin(scan)

        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-passo")
        self.footprint_mask_2d = scan.footprint_mask
        self.terrain_real_up, self.is_valid_up, self.unwritten_up = scan.terrain, scan.is_valid, scan.unwritten
        self.grad_up, self.rough_up, self.obstacle_mask_updated = scan.grad, scan.rough, scan.obstacle_mask

        self.t_proc = time.perf_counter()
        _gm_core_up = global_maps.fold_scan_into_global_maps(
            self.global_map, self.global_terrain_map, self.pts_up, self.terrain_real_up, self.obstacle_mask_updated,
            self.is_valid_up, self.cells_obs_up, (self.num_y_up, self.num_x_up), (self.robot_x, self.robot_y, self.robot_yaw))
        self.t_maps = time.perf_counter()

        # 2. Fresh local data into the PRM, then replanning from the current node if needed
        grad_up_2d = self.grad_up.reshape((self.num_y_up, self.num_x_up))
        self.rough_up_2d = self.rough_up.reshape((self.num_y_up, self.num_x_up))
        self.terrain_up_2d = self.terrain_real_up.reshape((self.num_y_up, self.num_x_up))

        # The local data is always injected: it costs nothing (it stores the array references) and
        # it is needed anyway by sample_terrain_between_points for the gait of the leg about to
        # be travelled.
        self.prm_graph.update_local_grid_data(
            grad_values_2d=grad_up_2d,
            rough_values_2d=self.rough_up_2d,
            terrain_values_2d=self.terrain_up_2d,
            valid_2d=spot_grid.slope_usable_mask(self.is_valid_up, self.cells_obs_up, (self.num_y_up, self.num_x_up)),
            grid_origin_x=self.grid_origin_x_up,
            grid_origin_y=self.grid_origin_y_up,
            cell_size=self.cell_size_up
        )

        # Snapshot of the scan just made: it is needed already here -- by the preliminary edge
        # discard in the refresh (see roadmap.LIVE_PREFILTER_OD_M), by that of the nodes
        # in obstacles -- and then by the safe frontier. It must be created ONLY ONCE per scan:
        # make_grid_snapshot also updates the temporal obstacle filter.
        self.snap = safe_frontier.make_grid_snapshot(
            self.cells_obs_up, self.rough_up, self.is_valid_up, self.valid_up, self.terrain_real_up,
            self.num_x_up, self.num_y_up, self.grid_origin_x_up, self.grid_origin_y_up, self.cell_size_up,
            self.robot_x, self.robot_y, unwritten=self.unwritten_up, robot_yaw=self.robot_yaw)

        # Free-space map: HERE, not together with global_map.update, because it needs the
        # filtered obstacle_distance that lives in the snapshot (see global_maps.update_clearance_map).
        # It is fed only by the navigation loop, so all observations go through
        # the same filter.
        global_maps.update_clearance_map(self.global_map, self.pts_up, self.snap, self.is_valid_up, self.unwritten_up,
                              _gm_core_up, (self.robot_x, self.robot_y))

    def _update_graph(self):
        """Edges re-evaluated on the new data (only after a move or an event), nodes in
        obstacles out, rotation refusals still valid. The nodes touched by the refresh."""
        # The preliminary edge discard reads the FILTERED obstacle_distance
        # (median over 3 scans + hysteresis), the same as the frontier.
        self.prm_graph.set_live_obstacles(self.snap.obstacle_dist, self.snap.seen, self.snap.origin_x, self.snap.origin_y,
                                     self.snap.cell_size, self.robot_x, self.robot_y, self.robot_yaw)

        # --- EVENT REFRESH -------------------------------------------------------------
        # Edge weights and slopes change only when the data under them changes,
        # i.e. when the window moves: with the robot still a refresh would recompute the
        # same numbers, and about half the scans are rescans while still (block
        # confirmations, turns without movement).
        # It refreshes only when needed: at the end of a MOVE (completed,
        # interrupted or failed, even if it crossed several PRM nodes and edges) and after
        # the EVENTS that change the plan (confirmed block, edge discarded,
        # replan, goal chosen again, return with GraphNav, fall).
        # THIS DOES NOT AFFECT SAFETY: the protection net is not the graph vetoes but
        # the SAFE FRONTIER, which is recomputed from scratch at every scan (0.5 ms) and at ~5 Hz
        # during the move in the thread. Postponing the refresh only delays the moment
        # the PLANNER learns of a new obstacle; if the robot is still there is
        # nothing new to learn, and if something really enters the field the frontier gets
        # shorter, the decision becomes 'wait' or 'blocked' -- which are events, so the
        # refresh triggers anyway. The uncovered case closes by itself.
        if self.need_refresh or getattr(self.prm_graph, 'needs_refresh', False):
            self.prm_graph.needs_refresh = False
            touched_nodes = self.prm_graph.refresh_local_edge_weights(
                global_map=self.global_map, edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
            self.need_refresh = False
            MISSION_STATS['prm_refresh'] += 1
            n_live = (getattr(self.prm_graph, 'live_veto_last', 0)
                      + getattr(self.prm_graph, 'live_prune_last', 0))
            MISSION_STATS['live_arc_vetoes'] = MISSION_STATS.get('live_arc_vetoes', 0) + n_live
            if n_live:
                print(f"[ARCHI] {n_live} archi fuori dal grafo per la scansione dal vivo "
                      f"({self.prm_graph.live_veto_last} rivalutati, {self.prm_graph.live_prune_last} "
                      f"oltre il raggio di refresh), prima della pendenza.")
        else:
            # No new data under the edges: the weights of the previous turn are kept and
            # there is no replan (the block below runs only if touched_nodes is not empty).
            touched_nodes = set()
            MISSION_STATS['prm_refresh_skipped'] += 1


        # Nodes INSIDE obstacles: out of the graph (see prune_prm_nodes_in_obstacles).
        pruned_nodes = safe_frontier.prune_prm_nodes_in_obstacles(self.prm_graph, self.snap)
        if pruned_nodes:
            MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
            print(f"[NODI] {len(pruned_nodes)} nodi scartati: meno di "
                  f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero dove li vediamo "
                  f"(primi: {pruned_nodes[:8]}).")

        # Edges already refused for rotation FROM THIS NODE: they stay out until the robot
        # moves. The discard is soft, so refresh_local_edge_weights would put them back
        # into play at every turn, and the robot could stay still alternating two refused
        # edges without ever getting to the retreat. The robot can stay attached to the
        # same node even after moving, so the discard holds only while the robot
        # is where the rotation was refused.
        for (from_id, to_id), n_fails in self.rotation_fails.items():
            fx, fy = self.rotation_fail_xy.get((from_id, to_id), (self.robot_x, self.robot_y))
            if (from_id == self.robot_node_id and n_fails > 0
                    and float(np.hypot(self.robot_x - fx, self.robot_y - fy)) <= prm_graph_module.ANCHOR_ON_NODE_M):
                prm_graph_module.mark_edge_blocked_soft(self.prm_graph, from_id, to_id)
        return touched_nodes

    def _replan(self, touched_nodes):
        """Dijkstra from the robot node to the goal on the updated graph; the path changes
        only if the new one costs at least 10% less. Without a path, new goal if the goal is
        the one isolated (REPICK_ON_REPLAN_FAIL)."""
        current_plan_ids = [self.robot_node_id] + [w[0] for w in self.path_waypoints]
        chosen_path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id,
                                                       current_path_ids=current_plan_ids, margin=0.10)

        if chosen_path_ids is None:
            n_free = self.prm_graph.reachable_count(self.robot_node_id, P.REPLAN_FAIL_ROBOT_FREE_NODES)
            if (P.REPICK_ON_REPLAN_FAIL and self.goal_repicks < P.MAX_BLOCK_REPLANS
                    and n_free >= P.REPLAN_FAIL_ROBOT_FREE_NODES):
                # From the robot the graph still goes somewhere: it is the goal that has been
                # left isolated. Another one with a path is searched at once; if there is
                # none, the previous plan stays as below.
                MISSION_STATS['replan_fail_repicks'] = MISSION_STATS.get('replan_fail_repicks', 0) + 1
                self._repick_goal(
                    f"Ripianificazione fallita: nessun percorso da {self.robot_node_id} all'obiettivo "
                    f"({self.target_x:.2f}, {self.target_y:.2f}) nel grafo aggiornato, mentre "
                    f"dal robot il grafo prosegue: e' isolato l'obiettivo.",
                    "nessun percorso nel grafo aggiornato")
            else:
                print(f"[REPLAN FALLITO] Nessun percorso esistente da {self.robot_node_id} a "
                      f"{self.goal_id} nel grafo aggiornato. Mantengo il percorso precedente: il "
                      f"fronte sicuro decide comunque fin dove si puo' avanzare.")
        elif chosen_path_ids != current_plan_ids:
            self._use_path(chosen_path_ids)
            print(f"[REPLAN] Percorso aggiornato ({len(touched_nodes)} nodi locali rivalutati): "
                  f"{len(self.path_waypoints)} waypoint rimanenti.")

            self.replan_counter += 1
            MISSION_STATS['replan_count'] += 1
            replan_save_path = os.path.join(
                self.mission_folder, f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_replan_{self.replan_counter}.png"
            ) if self.mission_folder else None

            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=replan_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up,
                include_diagnostics=False  # light: main view and global map
            )
        else:
            print(f"[REPLAN STABILE] Percorso invariato da {self.robot_node_id} a {self.goal_id} "
                  f"({len(touched_nodes)} nodi locali rivalutati): l'alternativa non "
                  f"supera la soglia di stabilita'. Proseguo sul piano corrente.")

    def _frontier_and_decision(self):
        """Shortcut, safe frontier on the path and decision (decisions.decide_next_move):
        move, node reached, arrived, wait or confirmed block."""
        self.t_prm = time.perf_counter()

        # 3. SAFE FRONTIER on the scan just made ---------------------------------------
        # The shortcut respects the edges already discarded: see shortcut_index.
        k_short = decisions.shortcut_index((self.robot_x, self.robot_y), self.path_waypoints, self.snap, self.global_map,
                                 prm=self.prm_graph, robot_node_id=self.robot_node_id)
        if k_short > 0:
            skipped = [w[0] for w in self.path_waypoints[:k_short]]
            self.path_waypoints = self.path_waypoints[k_short:]
            print(f"[SCORCIATOIA] Tratto diretto confermato libero verso il nodo {self.path_waypoints[0][0]}: "
                  f"salto {len(skipped)} nodi intermedi {skipped}.")
            MISSION_STATS['shortcuts'] += 1
        self.polyline = [(self.robot_x, self.robot_y)] + [(x, y) for _, x, y in self.path_waypoints]
        self.frontier = safe_frontier.compute_safe_frontier(self.polyline, self.snap)
        print(safe_frontier.describe_frontier(self.frontier))

        self.path_version = None
        if self.verification_tracker is not None:
            self.path_version = self.verification_tracker.update_path(self.polyline[1:])

        in_target_cell = (tuple(int(v) for v in self.env.get_cell_from_world(self.robot_x, self.robot_y))
                          == (int(self.target_row), int(self.target_col)))
        # Just outside the cell counts as inside, for the arrival by tolerance (see
        # ARRIVAL_OUTSIDE_CELL_M). On 22-10 16:04: "Mancano 0.06 m ... ancora fuori dalla
        # cella (2,7)", with the robot outside by 0.01 m.
        robot_border = environmentMap.cell_border_distance(self.env, self.target_row, self.target_col, self.robot_x, self.robot_y)
        near_goal = (not in_target_cell and robot_border is not None
                     and -robot_border <= P.ARRIVAL_OUTSIDE_CELL_M)
        self.decision = decisions.decide_next_move((self.robot_x, self.robot_y), self.path_waypoints, self.frontier, self.no_progress,
                                    in_target_cell=in_target_cell or near_goal)
        if self.decision.get('final'):
            MISSION_STATS['final_steps'] = MISSION_STATS.get('final_steps', 0) + 1
            print(f"[PASSO FINALE] Il fronte permette solo {self.decision['dist']:.2f} m, sotto il "
                  f"passo minimo di {P.MIN_PARTIAL_STEP_M:.2f} m, ma bastano: dopo il passo "
                  f"l'obiettivo e' a {self.frontier['path_len'] - self.decision['dist']:.2f} m, entro "
                  f"la tolleranza d'arrivo di {P.GOAL_REACHED_TOLERANCE_M:.2f} m.")
        if self.decision['kind'] == 'arrived' and near_goal:
            self.arrived_near_goal = True
            MISSION_STATS['arrivals_near_goal'] += 1
            print(f"[OBIETTIVO] Il robot e' fuori dalla cella ({self.target_row},{self.target_col}) di "
                  f"{-robot_border:.2f} m, a {self.frontier['path_len']:.2f} m dall'obiettivo: vale come "
                  f"arrivo (fuori al massimo di {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m, il muso e' dentro).")
        if (self.decision['kind'] in ('wait', 'blocked') and not in_target_cell and not near_goal
                and self.frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M):
            print(f"[OBIETTIVO] Mancano {self.frontier['path_len']:.2f} m, entro la tolleranza di "
                  f"{P.GOAL_REACHED_TOLERANCE_M:.2f} m, ma il robot e' ancora fuori dalla cella "
                  f"({self.target_row},{self.target_col}): non vale come arrivo, proseguo come per un "
                  f"fronte bloccato.")
        self.next_node_id, next_x, next_y = self.path_waypoints[0]
        self.keep_heading = False     # always False (no sideways): it stays in the saved data and in navigate_to

        # The bundle is saved here, before the phases that can close the turn (rotation,
        # wait, block): otherwise those scans would not end up on disk.
        self._save_scan_bundle()

    def _save_scan_bundle(self):
        """Everything needed to review this scan afterwards (scans/*.npz): grid layers,
        filtered snapshot, path, frontier and decision. See SAVE_SCAN_BUNDLES."""
        # A rotation refusal can be recomputed offline from obstacle_distance, robot_xyz,
        # robot_yaw and decision_target.
        if P.SAVE_SCAN_BUNDLES and self.mission_folder is not None:
            gf = self.local_grid.last_ground_filter
            shape_up = (self.num_y_up, self.num_x_up)
            extra = {}
            if P.SAVE_EXTRA_LAYERS:
                t_x = time.perf_counter()
                for name, lay in self.local_grid.extra_layers(P.SAVE_EXTRA_LAYERS).items():
                    extra[name] = lay['values']
                    extra[f'{name}_origin'] = lay['origin']
                    extra[f'{name}_cell'] = lay['cell']
                timing_add('scansione', time.perf_counter() - t_x)
            save_npz_async(
                os.path.join(self.mission_folder, "scans",
                             f"scan_it{self.iteration:03d}_cell{self.target_row}_{self.target_col}_{self.loop_iter:03d}.npz"),
                time=time.time(), iteration=self.iteration, cell=np.array([self.target_row, self.target_col]),
                loop_iter=self.loop_iter,
                terrain_raw=np.asarray(self.terrain_up, dtype=np.float32).reshape(shape_up),
                terrain_valid_raw=np.asarray(self.valid_up, dtype=np.float32).reshape(shape_up),
                obstacle_distance=np.asarray(self.cells_obs_up, dtype=np.float32).reshape(shape_up),
                # The layer the frontier DECIDES on (median over the last readings + hysteresis,
                # see safe_frontier._ObstacleHistory) and the latch mask. Without these
                # two an offline replay cannot reproduce the blocks that come from the
                # thread's readings during the move. Same orientation as the others:
                # (num_y, num_x), row = y.
                obstacle_distance_filtered=np.asarray(self.snap.obstacle_dist, dtype=np.float32),
                blocked_latch=(np.zeros(shape_up, bool) if self.snap.blocked_latch is None
                               else np.asarray(self.snap.blocked_latch, dtype=bool)),
                terrain_corrected=np.asarray(self.terrain_real_up, dtype=np.float32).reshape(shape_up),
                is_valid=np.asarray(self.is_valid_up, dtype=bool).reshape(shape_up),
                roughness=np.asarray(self.rough_up, dtype=np.float32).reshape(shape_up),
                gradient=np.asarray(self.grad_up, dtype=np.float32).reshape(shape_up),
                obstacle_mask=np.asarray(self.obstacle_mask_updated, dtype=np.int8).reshape(shape_up),
                footprint_mask=(np.zeros(shape_up, bool) if self.footprint_mask_2d is None
                                else np.asarray(self.footprint_mask_2d, dtype=bool).reshape(shape_up)),
                grid_origin=np.array([self.grid_origin_x_up, self.grid_origin_y_up]), cell_size=self.cell_size_up,
                robot_xyz=np.array([self.robot_x, self.robot_y, self.robot_z]), robot_yaw=self.robot_yaw,
                ground_z=np.nan if gf.get('ground_z') is None else gf['ground_z'],
                ground_n_above=gf.get('n_above', 0),
                mission_z0=mission_z0(),
                terrain_raw_zero=np.nan if spot_grid.raw_zero(self.grids_data_up) is None else spot_grid.raw_zero(self.grids_data_up),
                terrain_scale=np.nan if not spot_grid.raw_scale(self.grids_data_up) else spot_grid.raw_scale(self.grids_data_up),
                unwritten=np.asarray(self.unwritten_up, dtype=bool).reshape(shape_up),
                polyline=np.array(self.polyline), path_node_ids=np.array([w[0] for w in self.path_waypoints]),
                robot_node_id=self.robot_node_id, goal_xy=np.array([self.target_x, self.target_y]),
                frontier_dist=self.frontier['dist'], frontier_reason=self.frontier['reason'],
                frontier_stop_xy=np.array(self.frontier['stop_xy']),
                frontier_value=np.nan if self.frontier['value'] is None else self.frontier['value'],
                frontier_od_robot=self.frontier['od_robot'], frontier_half_along=self.frontier['half_along'],
                frontier_clearance=self.frontier['clearance_nominal'], frontier_path_len=self.frontier['path_len'],
                allowed_advance=safe_frontier.allowed_advance(self.frontier),
                keep_heading=self.keep_heading, no_progress=self.no_progress,
                **extra,
                decision_kind=self.decision['kind'],
                decision_target=np.array(self.decision.get('target', (np.nan, np.nan)), dtype=np.float64))
            MISSION_STATS['scan_bundles'] += 1

    def _rotation(self):
        """If the decided step requires turning where it cannot: advance in the cone, back up
        straight, recognize the trap or discard the leg."""
        # 4. ROTATION -------------------------------------------------------------------------
        # The nose ALWAYS points at the point to reach. If the rotation is not possible it does
        # NOT walk sideways (see ROTATION AND BACK-OFF in config.py). In order:
        # advance inside the possible rotation cone, back up straight if from there the
        # rotation becomes possible, recognize a trap, discard the leg.
        if self.decision['kind'] == 'move':
            rot = decisions.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), self.decision['target'],
                                self.frontier['od_robot'], snap=self.snap)
            if not rot['turn']:
                MISSION_STATS['rotation_refusals'] += 1
                edge_key = (self.robot_node_id, self.next_node_id)
                self.rotation_fails[edge_key] = self.rotation_fails.get(edge_key, 0) + 1
                self.rotation_fail_xy[edge_key] = (self.robot_x, self.robot_y)
                margin_txt = (f"margine {rot['margin']:+.2f} m sulle celle spazzate"
                              if rot['margin'] is not None else f"motivo: {rot['why']}")
                print(f"[ROTAZIONE] Servono {rot['dyaw_deg']:+.0f} gradi verso il nodo "
                      f"{self.next_node_id}, ma il corpo toccherebbe ({margin_txt}; "
                      f"obstacle_distance sotto il robot {self.frontier['od_robot']:.2f} m). "
                      f"Non cammino di traverso: tentativo "
                      f"{self.rotation_fails[edge_key]}/{P.MAX_ROTATION_FAILS_SAME_EDGE}.")

                outcome = self._advance_in_cone(rot)
                if outcome is None:
                    outcome = self._back_up_to_turn(rot, edge_key)
                if outcome is not None:
                    return outcome

                # Neither the cone nor the back-off: is it a trap or just a wrong edge?
                # If it is a trap it makes no sense to discard edges one after the other: go back
                # at once.
                trapped_rot, n_us_r, n_fan_r, fan_txt_r = self._escape_fan()
                if trapped_rot:
                    return self._escape_trap(
                        (self.robot_node_id, self.next_node_id),
                        f"[TRAPPOLA] Non posso girarmi verso {self.next_node_id} e nessuna delle "
                        f"{n_fan_r} direzioni del giro e' insieme percorribile e "
                        f"raggiungibile girandosi. Le piu' larghe: {fan_txt_r}. Ripercorro il "
                        f"tratto da cui sono venuto.")
                print(f"[ROTAZIONE] Direzioni utilizzabili da qui: {n_us_r}/{n_fan_r} "
                      f"({fan_txt_r}).")
                # Limit per ATTEMPT, not per pair of nodes (see MAX_ROTATION_DISCARDS).
                self.rotation_discards += 1
                if self.rotation_discards > P.MAX_ROTATION_DISCARDS:
                    print(f"[ROTAZIONE] Gia' {self.rotation_discards - 1} tratti scartati in questo "
                          f"tentativo perche' la rotazione era impossibile, e sono sempre qui: "
                          f"non e' l'arco, e' il posto. Ripercorro il tratto da cui sono venuto.")
                    if self._go_back((self.robot_node_id, self.next_node_id),
                                     f"dal nodo {self.robot_node_id} (troppi rifiuti di rotazione)"):
                        return CONTINUE
                    return FAILED
                print(f"[ROTAZIONE] Scarto il tratto {self.robot_node_id}->{self.next_node_id}: "
                      f"richiede di girarsi dove non c'e' spazio DA QUI. Ripianifico.")
                # NON-permanent discard: a rotation refusal depends on the robot's POSE,
                # not on the edge -- the same edge, approached from another point or with another
                # heading, can be perfectly travelable. Only what is confirmed by three still
                # scans or by a physical failure stays permanent.
                self._drop_edge(self.robot_node_id, self.next_node_id, 'ora')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None:
                    self._use_path(new_ids)
                    return CONTINUE
                print("[ROTAZIONE] Nessuna alternativa nel grafo dal punto in cui sono. "
                      "Mi stacco arretrando e rimando la cella.")
                if self._go_back((self.robot_node_id, self.next_node_id),
                                 f"dal nodo {self.robot_node_id}", skip_if_graph_cannot_help=True):
                    return CONTINUE
                return FAILED

    def _repick_unreachable_goal(self):
        """The frontier does not allow advancing: if the goal is to blame (inside an obstacle, or
        frontier stopped right next to it) another one is chosen. The outcome, or None."""
        # Before insisting: is the goal still reachable on the FRESH data? The
        # point is chosen at the start of the attempt, from metres away, when the
        # cell is still entirely unknown; getting there it may turn out to be right next to a wall,
        # and then no number of scans will make it reachable.
        if self.goal_repicks < P.MAX_BLOCK_REPLANS:
            if not safe_frontier.goal_still_reachable(self.snap, self.target_x, self.target_y):
                r_g, c_g, in_g = safe_frontier._cells(
                    self.snap, np.array([float(self.target_x)]), np.array([float(self.target_y)]))
                od_goal = float(self.snap.obstacle_dist[r_g[0], c_g[0]]) if in_g[0] else float('nan')
                outcome = self._repick_goal(
                    f"L'obiettivo ({self.target_x:.2f}, {self.target_y:.2f}) ha obstacle_distance "
                    f"{od_goal:.2f} m, sotto i {geometry.FRONTIER_CLEARANCE_M:.2f} m che il fronte "
                    f"pretende: non e' raggiungibile da nessuna direzione.",
                    f"spazio libero dal vivo {od_goal:.2f} m")
                if outcome is not None:
                    return outcome
            elif self._frontier_stops_at_goal():
                sx_g, sy_g = self.frontier['stop_xy']
                d_g = float(np.hypot(sx_g - self.target_x, sy_g - self.target_y))
                beyond = self.frontier['dist'] >= self.frontier['path_len']
                outcome = self._repick_goal(
                    f"Il fronte si ferma per {self.frontier['reason']} in ({sx_g:.2f}, {sy_g:.2f}), a "
                    f"{d_g:.2f} m dall'obiettivo ({self.target_x:.2f}, {self.target_y:.2f})"
                    + (", oltre la fine del percorso, dove andrebbe il muso" if beyond else "")
                    + ": e' l'obiettivo a non andare, non la strada.",
                    f"il fronte si ferma a {d_g:.2f} m da lui ({self.frontier['reason']})")
                if outcome is not None:
                    return outcome
        return None

    def _replan_now(self):
        """[SUBITO]: at the first blocking cell the leg is discarded and another way is searched,
        without waiting for the confirmation while still. The outcome, or None if it stays waiting."""
        # REPLAN AT ONCE, at the FIRST blocking cell, instead of staying still for three
        # scans on the same edge. The obstacle_distance noise is already filtered
        # (median + hysteresis in safe_frontier), so a blocking cell is
        # information to use: the edge is discarded and a path passing elsewhere is searched
        # at once. The still scans remain as the last net, to
        # declare the cell unreachable.
        if (self.early_replans < P.MAX_EARLY_REPLANS
                and self.frontier['reason'] in ('ostacolo', 'rugosita', 'pendenza')
                and self.path_waypoints):
            poly_now = [(self.robot_x, self.robot_y)] + [(x, y) for _, x, y in self.path_waypoints]
            _, seg_blk = safe_frontier.point_along(poly_now, self.frontier['dist'])
            seg_blk = int(seg_blk)
            ea = self.robot_node_id if seg_blk == 0 else self.path_waypoints[seg_blk - 1][0]
            eb = self.path_waypoints[min(seg_blk, len(self.path_waypoints) - 1)][0]
            sx_b, sy_b = self.frontier['stop_xy']

            # Before discarding yet another edge: am I trapped? If all the edges of the
            # node pass next to the same obstacle, the block is a PLACE, not an
            # edge, and discarding them one by one is useless. The fan sees it in one scan.
            trapped, n_us, n_fan, fan_txt = self._escape_fan()
            if trapped:
                return self._escape_trap(
                    (ea, eb),
                    f"[TRAPPOLA] Nessuna delle {n_fan} direzioni del giro e' insieme "
                    f"percorribile e raggiungibile girandosi. Le piu' larghe: {fan_txt}. "
                    f"Non ho dove andare ne' come girarmi: ripercorro il tratto da cui "
                    f"sono venuto, senza scartare altri archi.")

            self.early_replans += 1
            MISSION_STATS['early_replans'] += 1
            print(f"[SUBITO] {self.frontier['reason']} in ({sx_b:.2f}, {sy_b:.2f}) sul tratto "
                  f"{ea}->{eb}: lo scarto adesso e cerco un altro percorso, senza "
                  f"aspettare la conferma da fermo ({self.early_replans}/{P.MAX_EARLY_REPLANS}). "
                  f"Direzioni utilizzabili da qui: {n_us}/{n_fan}.")
            # NON-permanent discard: after a single scan the edge must be removed from the plan,
            # not condemned for the mission (see mark_edge_blocked_soft).
            self._drop_edge(ea, eb, 'ora')
            # But if it is the SECOND time on the same leg, the "for now" discard was not
            # enough: the refresh put it back into the graph (the preliminary discard looks at one
            # scan at 0.25 m, the frontier at the median of three with hysteresis up to 0.29),
            # and the plan went back to the previous one. Out of the plan for the rest of the
            # attempt, like the returns and [ARRIVO]. On 28-10 14:23, 8 [SUBITO] out of 14 were
            # repetitions of the same leg (197->569 four times in a row).
            key_s = (min(ea, eb), max(ea, eb))
            if key_s in self.early_replanned_edges:
                self._drop_edge(ea, eb, 'tentativo')
                MISSION_STATS['early_replans_repeated'] = MISSION_STATS.get('early_replans_repeated', 0) + 1
                print(f"[SUBITO] Il tratto {ea}->{eb} era gia' stato scartato in questo tentativo e "
                      f"il piano ci e' tornato: fuori dal piano per il resto del tentativo.")
            self.early_replanned_edges.add(key_s)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None and len(new_ids) > 1:
                self._use_path(new_ids)
                print(f"[SUBITO] Percorso alternativo con {len(self.path_waypoints)} waypoint, "
                      f"dal punto in cui sono.")
                return CONTINUE
            print("[SUBITO] Il grafo non offre alternative da qui: resto sul percorso e "
                  "lascio decidere il fronte.")
        return None

    def _advance_in_cone(self, rot):
        """Rotation impossible: advance inside the allowed rotation cone, if it gets closer to
        the path (CONTINUE), otherwise None."""
        # --- 4a. FIRST OF ALL: can I GO FORWARD inside the rotation cone I am
        # allowed? Often going straight, or almost, gets closer to the goal than
        # any back-off (see plan_cone_move).
        cone_goal, _ = safe_frontier.point_along(self.polyline, P.CONE_PATH_LOOKAHEAD_M)
        cone = decisions.plan_cone_move((self.robot_x, self.robot_y), self.robot_yaw, self.snap, cone_goal)
        if cone is not None:
            print(f"[CONO] Non posso girarmi di {rot['dyaw_deg']:+.0f} gradi, ma ruotando di "
                  f"{cone['dyaw_deg']:+.0f} posso avanzare {cone['dist']:.2f} m "
                  f"({cone['reason']}) e avvicinarmi di {cone['gain']:.2f} m al percorso "
                  f"({cone_goal[0]:.2f}, {cone_goal[1]:.2f}, {P.CONE_PATH_LOOKAHEAD_M:.1f} m "
                  f"avanti lungo il percorso). Vado, invece di arretrare.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['cone_moves'] += 1
            cx, cy = cone['target']
            self.need_refresh = True      # end of a move: see EVENT REFRESH
            with TrajectoryLog(self.robot_state_client, 'cono', (cx, cy)):
                ok_c, dist_c, _ = movements.navigate_to(
                    cx, cy, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                    get_a_tform_b(self.lg_proto_up.local_grid.transforms_snapshot,
                                  VISION_FRAME_NAME, BODY_FRAME_NAME),
                    should_abort=None, keep_heading=False, **self.mobility_kwargs)
            if dist_c > 0.05:
                self._stop_node_here(*self._walked_path(self.robot_x, self.robot_y), "sosta (avanzamento nel cono)")
                self.retreats_in_a_row = 0      # it made real progress: the chain breaks
            else:
                print(f"[CONO] Non sono riuscito ad avanzare (percorsi {dist_c:.2f} m).")
            self.no_progress = 0
            return CONTINUE
        return None

    def _back_up_to_turn(self, rot, edge_key):
        """Rotation impossible: back up straight, if from there the rotation becomes possible
        (CONTINUE), otherwise None."""
        # --- 4b. Back up, but ONLY if it serves a purpose (see retreat_would_help).
        back = decisions.plan_straight_retreat((self.robot_x, self.robot_y), self.robot_yaw, self.snap)
        helps, margin_back = (False, None)
        if back is not None:
            helps, margin_back = decisions.retreat_would_help(
                self.snap, (back[0], back[1]), self.robot_yaw, rot['dyaw_deg'])
            if not helps:
                MISSION_STATS['retreats_suppressed'] += 1
                mb = "" if margin_back is None else f" (margine {margin_back:+.2f} m da li')"
                print(f"[ARRETRAMENTO] Potrei arretrare di {back[2]:.2f} m, ma da li' la "
                      f"rotazione di {rot['dyaw_deg']:+.0f} gradi resterebbe impossibile{mb}: "
                      f"non lo faccio, sposterei il problema di 60 cm.")
        if (back is not None and helps
                and self.retreats_in_a_row < P.MAX_STRAIGHT_RETREATS_IN_A_ROW
                and self.rotation_fails[edge_key] < P.MAX_ROTATION_FAILS_SAME_EDGE):
            bx, by, bdist = back
            self.retreats_in_a_row += 1
            print(f"[ARRETRAMENTO] Indietro dritto di {bdist:.2f} m verso "
                  f"({bx:.2f}, {by:.2f}), senza girarmi, per fare spazio alla rotazione "
                  f"({self.retreats_in_a_row}/{P.MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['straight_retreats'] += 1
            self.need_refresh = True      # end of a move: see EVENT REFRESH
            with TrajectoryLog(self.robot_state_client, 'arretramento', (bx, by)):
                ok_back, dist_back = movements.move_to_world_point_without_turning(
                    bx, by, "vision", self.command_client, self.robot_state_client, **self.mobility_kwargs)
            if dist_back > 0.05:
                self._stop_node_here(*self._walked_path(self.robot_x, self.robot_y), "sosta (arretramento per ruotare)")
            else:
                print(f"[ARRETRAMENTO] Non sono riuscito ad arretrare "
                      f"(percorsi {dist_back:.2f} m).")
            self.no_progress = 0
            return CONTINUE

        if back is None:
            print("[ARRETRAMENTO] Non c'e' spazio per arretrare dritto.")
        elif helps and self.retreats_in_a_row >= P.MAX_STRAIGHT_RETREATS_IN_A_ROW:
            print(f"[ARRETRAMENTO] Gia' {self.retreats_in_a_row} arretramenti di fila senza "
                  f"combinare niente: smetto di arretrare.")
        return None

    def _save_block_data(self):
        """Scan and path of the confirmed block, to look at it afterwards (BLOCKED_n.npz)."""
        if self.mission_folder is not None:
            try:
                npz_blocked = os.path.join(
                    self.mission_folder,
                    f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED_{self.block_replans + 1}.npz")
                np.savez_compressed(
                    npz_blocked,
                    obstacle_distance=self.snap.obstacle_dist, rough_veto=self.snap.rough_veto,
                    seen=self.snap.seen, terrain=self.snap.terrain,
                    terrain_raw=np.asarray(self.terrain_up).reshape((self.num_y_up, self.num_x_up)),
                    terrain_valid_raw=np.asarray(self.valid_up).reshape((self.num_y_up, self.num_x_up)),
                    obstacle_mask=np.asarray(self.obstacle_mask_updated).reshape((self.num_y_up, self.num_x_up)),
                    roughness=self.rough_up_2d,
                    grid_origin_x=self.snap.origin_x, grid_origin_y=self.snap.origin_y, cell_size=self.snap.cell_size,
                    robot_xy=np.array([self.robot_x, self.robot_y]), robot_yaw=self.robot_yaw,
                    polyline=np.array(self.polyline), frontier_dist=self.frontier['dist'],
                    frontier_reason=self.frontier['reason'], frontier_stop_xy=np.array(self.frontier['stop_xy']),
                    ground_z=np.nan if self.local_grid.last_ground_filter.get('ground_z') is None
                    else self.local_grid.last_ground_filter['ground_z'])
                print(f"[DATA SAVE] Dati del blocco salvati in {os.path.basename(npz_blocked)}")
            except Exception as e:
                print(f"[DATA SAVE] Impossibile salvare i dati del blocco: {e}")

    def _timings(self):
        """Timings of the turn, for [TEMPO] and the summary."""
        t_front = time.perf_counter()

        timing_add('scansione', self.t_scan - self.t_iter)
        timing_add('elaborazione griglia', self.t_proc - self.t_scan)
        timing_add('mappe globali', self.t_maps - self.t_proc)
        timing_add('PRM e ripianificazione', self.t_prm - self.t_maps)
        timing_add('fronte e decisione', t_front - self.t_prm)
        print(f"[TEMPO] scansione {self.t_scan - self.t_iter:.2f} s | elaborazione {self.t_proc - self.t_scan:.2f} | "
              f"mappe {self.t_maps - self.t_proc:.2f} | PRM {self.t_prm - self.t_maps:.2f} | fronte {t_front - self.t_prm:.2f} "
              f"| decisione: {self.decision['kind']}")

    def _node_or_arrival(self):
        """Path node reached, or arrival accepted before the end."""
        if self.decision['kind'] == 'reached_node':
            self.robot_node_id = self.next_node_id
            self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
            self.path_waypoints.pop(0)
            self.no_progress = 0
            return CONTINUE

        if self.decision['kind'] == 'arrived':
            print(f"[OBIETTIVO] Il fronte non permette gli ultimi {self.frontier['path_len']:.2f} m "
                  f"({self.frontier['reason']}), entro la tolleranza di {P.GOAL_REACHED_TOLERANCE_M:.2f} m: "
                  f"considero raggiunto l'obiettivo.")
            return STOP

    def _wait(self):
        """The frontier does not allow advancing: goal still reachable? replan at once
        (SUBITO) or wait for the confirmation while still."""
        if self.decision['kind'] == 'wait':
            outcome = self._repick_unreachable_goal()
            if outcome is None:
                outcome = self._replan_now()
            if outcome is not None:
                return outcome

            self.no_progress += 1
            print(f"[FRONTE] Non posso avanzare nemmeno di {P.MIN_PARTIAL_STEP_M:.2f} m: riguardo "
                  f"da fermo ({self.no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni).")
            time.sleep(P.FRONTIER_RESCAN_WAIT_S)
            return CONTINUE

    def _block(self):
        """Block confirmed by several still scans: edge discarded, replan or return."""
        if self.decision['kind'] == 'blocked':
            k = self.decision['segment']
            edge_a = self.robot_node_id if k == 0 else self.path_waypoints[k - 1][0]
            edge_b = self.path_waypoints[min(k, len(self.path_waypoints) - 1)][0]
            sx, sy = self.frontier['stop_xy']
            print(f"[BLOCCO] Confermato da vicino: {P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni consecutive "
                  f"senza poter avanzare. Motivo: {self.frontier['reason']} in ({sx:.2f}, {sy:.2f}), "
                  f"sul tratto {edge_a}->{edge_b}.")
            MISSION_STATS['frontier_blocks'] += 1
            # Blocked right next to the goal (or beyond): it is the goal that does not work, not the
            # way. It is excluded and, if possible, another one is chosen (below, after the trap)
            # instead of discarding the leg and replanning towards the same goal.
            goal_blocked = self._frontier_stops_at_goal()
            d_goal = float(np.hypot(sx - self.target_x, sy - self.target_y))
            why_goal = f"blocco confermato a {d_goal:.2f} m da lui ({self.frontier['reason']})"
            if goal_blocked:
                goal._refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                             why_goal)

            self._save_block_data()

            # Trap? If there is no usable direction, replanning
            # is useless by construction: the problem is not the edge, it is the place.
            trapped_blk, n_us_b, n_fan_b, fan_txt_b = self._escape_fan()
            print(f"[BLOCCO] Direzioni utilizzabili da qui: {n_us_b}/{n_fan_b} ({fan_txt_b}).")
            if trapped_blk:
                return self._escape_trap(
                    (edge_a, edge_b), "[TRAPPOLA] Nessuna via d'uscita girandomi: non scarto altri "
                                      "archi, ripercorro il tratto da cui sono venuto.")

            if goal_blocked and self.goal_repicks < P.MAX_BLOCK_REPLANS:
                outcome = self._repick_goal(
                    f"Blocco a {d_goal:.2f} m dall'obiettivo ({self.target_x:.2f}, "
                    f"{self.target_y:.2f}): e' l'obiettivo a non andare, non il tratto {edge_a}->{edge_b}.",
                    why_goal)
                if outcome is not None:
                    return outcome

            # Here the block is confirmed by FRONTIER_BLOCK_CONFIRM_SCANS still scans:
            # the invalidation stays permanent (unlike [SUBITO], see
            # mark_edge_blocked_soft). Exception: on the first leg, if the robot
            # is not on the node it is attached to, the blocked leg is robot->node, not
            # the graph edge: the edge is discarded only for now.
            on_node = k > 0 or mission_graph.robot_on_node(self.prm_graph, edge_a, self.robot_x, self.robot_y)
            self._drop_edge(edge_a, edge_b, 'missione' if on_node else 'ora')
            self.block_replans += 1
            self.no_progress = 0

            new_ids = None
            if self.block_replans <= P.MAX_BLOCK_REPLANS:
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None:
                self._use_path(new_ids)
                print(f"[REPLAN] Dopo il blocco ({self.block_replans}/{P.MAX_BLOCK_REPLANS}): percorso "
                      f"alternativo con {len(self.path_waypoints)} waypoint, dal punto in cui sono.")
                return CONTINUE

            print(f"[BLOCCO] Nessuna alternativa dal punto in cui sono"
                  + (f" (gia' {P.MAX_BLOCK_REPLANS} ripianificazioni dopo blocchi)."
                     if self.block_replans > P.MAX_BLOCK_REPLANS else ".")
                  + " Mi stacco arretrando e rimando la cella.")
            if self._go_back((edge_a, edge_b),
                             f"dal nodo {self.robot_node_id}", skip_if_graph_cannot_help=True):
                return CONTINUE

            blocked_save_path = os.path.join(self.mission_folder,
                                             f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=self.frontier['stop_xy'], iteration=self.iteration, env=self.env,
                save_path=blocked_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up
            )
            return FAILED

    def _execute_legs(self):
        """The decided move: check of the arrival pose, then one or more legs on the polyline
        confirmed by the frontier, with a stop halfway if the frontier gets shorter."""
        outcome = self._check_arrival_pose()
        if outcome is not None:
            return outcome

        # --- Execution: one or more LEGS on the polyline already confirmed by the frontier,
        # without scan, refresh or replan in between (see MULTI-NODE
        # ADVANCE in decide_next_move). n_legs = 0 is a single partial step.
        n_legs = int(self.decision.get('nodes', 1)) if self.decision['full'] else 0
        if n_legs > 1:
            MISSION_STATS['multinode_moves'] += 1
            MISSION_STATS['multinode_nodes'] += n_legs
            print(f"[MULTINODO] Il fronte conferma "
                  f"{safe_frontier.allowed_advance(self.frontier):.2f} m: percorro {n_legs} "
                  f"tratti di fila {[w[0] for w in self.path_waypoints[:n_legs]]} senza fermarmi "
                  f"a riscansionare.")
        for leg_i in range(max(1, n_legs)):
            if not self.path_waypoints:
                break
            self.next_node_id, next_x, next_y = self.path_waypoints[0]
            if n_legs == 0:
                move_x, move_y = self.decision['target']
                is_partial_step = True
            else:
                move_x, move_y = next_x, next_y
                is_partial_step = False
            if leg_i > 0:
                vision_tform_body_current = self._prepare_leg(leg_i, n_legs, move_x, move_y)
                if vision_tform_body_current is None:
                    break
            else:
                vision_tform_body_current = get_a_tform_b(self.lg_proto_up.local_grid.transforms_snapshot,
                                                          VISION_FRAME_NAME, BODY_FRAME_NAME)
            self.no_progress = 0
            self.retreats_in_a_row = 0   # a real advance breaks the chain of back-offs
            if is_partial_step:
                print(f"[PASSO PARZIALE] Verso il nodo {self.next_node_id}: avanzo di {self.decision['dist']:.2f} m "
                      f"su {np.hypot(next_x - self.robot_x, next_y - self.robot_y):.2f} fino a "
                      f"({move_x:.2f}, {move_y:.2f}), poi riguardo con una scansione fresca.")
            else:
                print(f"[OK] Tratto {self.robot_node_id}->{self.next_node_id} confermato libero dal fronte. Eseguo movimento...")

            outcome = self._walk_leg(move_x, move_y, next_x, next_y, is_partial_step,
                                         vision_tform_body_current)
            if outcome == 'fermo':
                break
            if outcome is FAILED:
                return FAILED
        # Legs finished (or interrupted): back to the top with a fresh scan.
        return CONTINUE

    def _check_arrival_pose(self):
        """Would the step lead to a point without exits? Then it does not go: leg out of the
        plan for the attempt and new path (CONTINUE), or side abandoned (FAILED).
        None = it can go."""
        move_x, move_y = self.decision['target']
        # --- Preventive check on the arrival pose ----------------------------------------------
        # Only for short steps or towards tight spaces: those are the ones that enter
        # dead-end corners. See arrival_has_exit.
        step_len = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
        r_a, c_a, in_a = safe_frontier._cells(self.snap, np.array([move_x]), np.array([move_y]))
        od_arrival = float(self.snap.obstacle_dist[r_a[0], c_a[0]]) if in_a[0] else float('inf')
        if (self.arrival_refusals < P.MAX_ARRIVAL_REFUSALS
                and (step_len <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_arrival <= P.ARRIVAL_EXIT_CHECK_OD_M)):
            heading_arr = float(np.arctan2(move_y - self.robot_y, move_x - self.robot_x))
            has_exit, fan_arr = decisions.arrival_has_exit(self.snap, (move_x, move_y), heading_arr)
            if not has_exit:
                self.arrival_refusals += 1
                MISSION_STATS['arrival_refusals'] += 1
                n_us_a, fan_txt_a = decisions.describe_escape_fan(fan_arr)
                print(f"[ARRIVO] Il passo di {step_len:.2f} m verso ({move_x:.2f}, {move_y:.2f}) "
                      f"(obstacle_distance li' {od_arrival:.2f} m) mi porterebbe dove non ci sono "
                      f"uscite: {n_us_a}/{len(fan_arr)} direzioni utilizzabili ({fan_txt_a}). "
                      f"Non ci vado ({self.arrival_refusals}/{P.MAX_ARRIVAL_REFUSALS}), scarto il tratto "
                      f"{self.robot_node_id}->{self.next_node_id} per il resto del tentativo e ripianifico.")
                # Out of the plan for the REST OF THE ATTEMPT (attempt penalty, like the
                # returns: see _trap_backtrack), not only for now: the leg itself is free,
                # it is the arrival point that has no exits, so the refresh would put it back
                # into the graph at the next turn and the same plan would be made again. On 28-10 12:16 the
                # same step was refused three times in a row, and executed at the third.
                self._drop_edge(self.robot_node_id, self.next_node_id, 'tentativo')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None and len(new_ids) > 1:
                    self._use_path(new_ids)
                    return CONTINUE
                # Knowing in advance that the arrival point is a dead end and going there
                # anyway makes no sense: this side of the cell is given up, and the cell will be
                # retried later from another one (finalize_or_defer_blocked_cell).
                print(f"[ARRIVO] Nessuna alternativa nel grafo e il punto d'arrivo non ha "
                      f"uscite: non ci vado. Rinuncio a entrare in ({self.target_row},{self.target_col}) "
                      f"da questo lato.")
                if self.verification_tracker is not None:
                    self.verification_tracker.clear_path()
                return FAILED
        return None

    def _prepare_leg(self, leg_i, n_legs, move_x, move_y):
        """Before an intermediate multi-node leg: true pose, possible rotation and exits on
        arrival. The VISION pose to start from, or None if it stops to look again."""
        # TRUE pose after the previous leg. It serves three purposes, all necessary:
        #  - navigate_to computes the rotation angle from the heading it receives,
        #    and relative_move rotates RELATIVE TO THE CURRENT POSE: with the heading
        #    of the initial scan (before leg 0) the robot would rotate
        #    by the wrong angle and walk off the verified path;
        #  - the distance to travel is computed from (robot_x, robot_y);
        #  - the stop halfway through the move measures the remaining path from here.
        self.robot_x, self.robot_y, self.robot_z, robot_quat = spotUtils.getPosition(self.robot_state_client)
        self.robot_yaw = np.arctan2(
            2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
            1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))
        fresh_vision_tform_body = SimpleNamespace(
            position=SimpleNamespace(x=self.robot_x, y=self.robot_y, z=self.robot_z), rotation=robot_quat)

        # At the intermediate node the robot STOPS and ROTATES towards the next node: the
        # initial decision only checked the rotation of the first leg.
        # Here this one is checked, on the scan snapshot (in world
        # coordinates, so valid even after the move) -- but only if the WHOLE
        # swept circle is inside the data: rotation_is_clear ignores the cells
        # outside the window (optimistic), and with a curved path an intermediate node
        # can be near the edge. If it cannot rotate, or it cannot be told,
        # it stops and looks again: the loop handles the rotation, with cone,
        # back-off and fan, on the fresh data.
        r_h, c_h, in_h = safe_frontier._cells(self.snap, np.array([self.robot_x]), np.array([self.robot_y]))
        od_here = float(self.snap.obstacle_dist[r_h[0], c_h[0]]) if in_h[0] else 0.0
        rot_leg = decisions.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), (move_x, move_y),
                                od_here, snap=self.snap)
        reach_rot = float(geometry.ROTATION_CLEARANCE_M)
        ny_s, nx_s = self.snap.obstacle_dist.shape
        inside_rot = (self.snap.origin_x <= self.robot_x - reach_rot and
                      self.robot_x + reach_rot <= self.snap.origin_x + nx_s * self.snap.cell_size and
                      self.snap.origin_y <= self.robot_y - reach_rot and
                      self.robot_y + reach_rot <= self.snap.origin_y + ny_s * self.snap.cell_size)
        if not rot_leg['turn'] or (rot_leg['why'] == 'settore spazzato' and not inside_rot):
            reason_txt = ("il corpo toccherebbe" if not rot_leg['turn']
                      else "il cerchio di rotazione esce dai dati della scansione")
            print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: per puntare il nodo "
                  f"{self.next_node_id} servono {rot_leg['dyaw_deg']:+.0f} gradi e {reason_txt}. "
                  f"Mi fermo qui e riguardo con una scansione fresca.")
            MISSION_STATS['multinode_cut'] += 1
            return None

        # Same caution as the first leg on the arrival pose (arrival_has_exit):
        # do not stop at a point without exits.
        step_leg = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
        r_l, c_l, in_l = safe_frontier._cells(self.snap, np.array([move_x]), np.array([move_y]))
        od_leg = float(self.snap.obstacle_dist[r_l[0], c_l[0]]) if in_l[0] else float('inf')
        if step_leg <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_leg <= P.ARRIVAL_EXIT_CHECK_OD_M:
            has_exit_leg, _fan_leg = decisions.arrival_has_exit(
                self.snap, (move_x, move_y), float(np.arctan2(move_y - self.robot_y, move_x - self.robot_x)))
            if not has_exit_leg:
                print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: il nodo {self.next_node_id} "
                      f"(obstacle_distance {od_leg:.2f} m) non avrebbe uscite. Mi fermo "
                      f"qui e riguardo con una scansione fresca.")
                MISSION_STATS['multinode_cut'] += 1
                return None

        if self.verification_tracker is not None:
            self.path_version = self.verification_tracker.update_path(
                [(x, y) for _, x, y in self.path_waypoints])
        return fresh_vision_tform_body

    def _leg_terrain(self, move_x, move_y):
        """Slope and roughness of the leg, gait chosen accordingly."""
        # --- Measure of the difficulty of THIS segment (for the gait and the CSV).
        segment_long_slope, segment_lat_slope, segment_roughness = self.prm_graph.sample_terrain_between_points(
            self.robot_x, self.robot_y, move_x, move_y,
            node_idx1=self.robot_node_id, node_idx2=self.next_node_id
        )
        slope_source = self.prm_graph.last_slope_source

        near_thresh = slopes.SLOPE_THRESHOLD * (1.0 - slopes.NEAR_THRESHOLD_MARGIN_FRACTION)
        if (segment_long_slope >= near_thresh or segment_lat_slope >= near_thresh) and self.mission_folder is not None:
            try:
                npz_near = os.path.join(
                    self.mission_folder,
                    f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_"
                    f"arc{self.robot_node_id}-{self.next_node_id}_NEARLIMIT.npz")
                np.savez_compressed(
                    npz_near,
                    obstacle_grid=np.asarray(self.obstacle_mask_updated).reshape((self.num_y_up, self.num_x_up)),
                    terrain_grid=np.asarray(self.terrain_real_up).reshape((self.num_y_up, self.num_x_up)),
                    grid_origin_x=self.grid_origin_x_up, grid_origin_y=self.grid_origin_y_up, cell_size=self.cell_size_up,
                    arc_xy=np.array([self.robot_x, self.robot_y, move_x, move_y]),
                    long_slope=segment_long_slope, lat_slope=segment_lat_slope,
                    slope_threshold=slopes.SLOPE_THRESHOLD)
                MISSION_STATS['near_threshold_saves'] += 1
                margin_left = slopes.SLOPE_THRESHOLD - max(segment_long_slope, segment_lat_slope)
                print(f"[NEAR-LIMIT] Arco {self.robot_node_id}->{self.next_node_id} vicino soglia "
                      f"(long={segment_long_slope:.3f}, lat={segment_lat_slope:.3f}, "
                      f"soglia={slopes.SLOPE_THRESHOLD:.3f}, margine residuo={margin_left:+.3f}) "
                      f"-- dati salvati in {os.path.basename(npz_near)}")
            except Exception as e:
                print(f"[DATA SAVE] Impossibile salvare il file vicino soglia: {e}")

        if self.mobility_kwargs:
            segment_mobility_kwargs = self.mobility_kwargs
            segment_tier = 'override'
        else:
            segment_gradient = max(segment_long_slope, segment_lat_slope)
            segment_mobility_kwargs = movements.select_mobility_params(segment_gradient, segment_roughness)
            segment_tier = MISSION_STATS['last_tier']
        return (segment_long_slope, segment_lat_slope, segment_roughness, slope_source,
                segment_mobility_kwargs, segment_tier)

    def _move_and_record(self, move_x, move_y, vision_tform_body_current, terrain_info):
        """The motion command, with the stop halfway if the frontier gets shorter, and the rows
        of trajectory.csv, frontier_live.csv and segments.csv. Returns (success, metres
        travelled, metres commanded, stop reason or None)."""
        (segment_long_slope, segment_lat_slope, segment_roughness, slope_source,
         segment_mobility_kwargs, segment_tier) = terrain_info
        # --- Stop condition halfway through the move ---------------------------------
        abort_state = {'reason': None, 'bad': 0}
        move_t0 = time.time()
        live_frontiers = []

        def _should_abort_move():
            if self.verification_tracker is None:
                return False
            f = self.verification_tracker.get_frontier()
            if f is None or f.get('path_version') != self.path_version or f['time'] <= move_t0:
                return False
            frx, fry = f['robot_xy']
            remaining = float(np.hypot(move_x - frx, move_y - fry))
            reach = safe_frontier.allowed_advance(f)
            if not live_frontiers or live_frontiers[-1][0] != f['time']:
                live_frontiers.append((f['time'], frx, fry, f['reason'], float(f['dist']), reach, remaining,
                                       abort_state['bad']))
            if (f['reason'] == 'fine_dati'
                    and float(np.hypot(frx - self.robot_x, fry - self.robot_y)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                abort_state['bad'] = 0      # still rotating in place: see the constant
                return False
            if reach + P.FRONTIER_ABORT_TOLERANCE_M < remaining:
                abort_state['bad'] += 1
                if abort_state['bad'] >= P.FRONTIER_ABORT_CONFIRM:
                    sxa, sya = f['stop_xy']
                    abort_state['reason'] = (f"il fronte si e' accorciato ({f['reason']} in "
                                             f"({sxa:.2f}, {sya:.2f})): raggiungibili {reach:.2f} m "
                                             f"su {remaining:.2f} m mancanti")
                    return True
            else:
                abort_state['bad'] = 0
            return False

        # --- Execution with recording of the real body tilt ---
        tilt_recorder = BodyTiltRecorder(self.robot_state_client)
        tilt_recorder.start()
        try:
            self.need_refresh = True              # end of a move: see EVENT REFRESH
            success_move, distance_traveled, distance_commanded = movements.navigate_to(
                move_x, move_y, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                vision_tform_body_current, should_abort=_should_abort_move,
                keep_heading=self.keep_heading,
                **segment_mobility_kwargs
            )
        finally:
            tilt_stats = tilt_recorder.stop()
        move_duration = time.time() - move_t0
        timing_add('movimento', move_duration)
        try:   # log only: it must never interrupt the mission
            _seg_id = [self.iteration, self.target_row, self.target_col, self.loop_iter, self.robot_node_id, self.next_node_id]
            append_trajectory(getattr(tilt_recorder, 'samples', []), 'prm', (move_x, move_y), _seg_id)
            append_csv("frontier_live.csv",
                        ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                         'robot_x', 'robot_y', 'reason', 'frontier_dist', 'reachable_m', 'remaining_m',
                         'consecutive_bad', 'aborted'],
                        [[lf[0]] + _seg_id + list(lf[1:]) + [abort_state['reason'] is not None]
                         for lf in live_frontiers])
        except Exception as e:
            print(f"[LOG] Traiettoria/fronti del movimento non registrati: {e}")

        # --- CSV row for this segment (even if the move failed) ---
        try:
            seg_dist = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
            cost_dist = self.prm_graph.alpha * seg_dist
            cost_long = self.prm_graph.beta * segment_long_slope
            cost_lat = self.prm_graph.beta_lateral * segment_lat_slope
            cost_rough = self.prm_graph.gamma * segment_roughness
            log_segment({
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'iteration': self.iteration, 'cell_row': self.target_row, 'cell_col': self.target_col,
                'node_from': self.robot_node_id, 'node_to': self.next_node_id,
                'x_from': self.robot_x, 'y_from': self.robot_y, 'x_to': move_x, 'y_to': move_y,
                'dist_m': seg_dist,
                'slope_source': slope_source,
                'long_slope': segment_long_slope, 'lat_slope': segment_lat_slope,
                'roughness': segment_roughness,
                'cost_dist': cost_dist, 'cost_long': cost_long, 'cost_lat': cost_lat,
                'cost_rough': cost_rough, 'cost_total': cost_dist + cost_long + cost_lat + cost_rough,
                'tier': segment_tier, 'gait_override': bool(self.mobility_kwargs),
                'success': bool(success_move),
                'dist_traveled_m': distance_traveled, 'dist_commanded_m': distance_commanded,
                'duration_s': move_duration,
                'roll_start_deg': tilt_stats['roll_start'], 'pitch_start_deg': tilt_stats['pitch_start'],
                'roll_end_deg': tilt_stats['roll_end'], 'pitch_end_deg': tilt_stats['pitch_end'],
                'roll_peak_abs_deg': tilt_stats['roll_peak_abs'], 'pitch_peak_abs_deg': tilt_stats['pitch_peak_abs'],
                'tilt_samples': tilt_stats['samples'], 'tilt_errors': tilt_stats['errors'],
            })
        except Exception as e:
            print(f"[LOG] Impossibile registrare il segmento: {e}")
        return success_move, distance_traveled, distance_commanded, abort_state['reason']

    def _walk_leg(self, move_x, move_y, next_x, next_y, is_partial_step, vision_tform_body_current):
        """One leg: gait, move, outcome. 'avanti' = succeeded, the next leg can be done;
        'fermo' = interrupted or replanned, look again; FAILED = physical failure."""
        terrain_info = self._leg_terrain(move_x, move_y)
        success_move, distance_traveled, distance_commanded, abort_reason = self._move_and_record(
            move_x, move_y, vision_tform_body_current, terrain_info)
        # Command ended badly (timeout, Spot not settled...) without a stop requested
        # by the frontier, but with the robot ARRIVED within a few cm of the point: the leg counts as
        # successful instead of as a physical failure (edge discarded forever and return).
        # 28-10 13:50, cell (3,2): 20 degree climb in HARD, 0.42 m out of 0.44 past the 15 s.
        # segments.csv keeps the real outcome of the command.
        if not success_move and abort_reason is None:
            try:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                d_left = float(np.hypot(move_x - ax, move_y - ay))
            except Exception:
                d_left = float('inf')
            if d_left <= P.MOVE_NEAR_GOAL_M and distance_traveled >= 0.5 * distance_commanded:
                MISSION_STATS['moves_near_goal'] = MISSION_STATS.get('moves_near_goal', 0) + 1
                print(f"[QUASI] Comando verso ({move_x:.2f}, {move_y:.2f}) non concluso, ma il robot "
                      f"e' a {d_left:.2f} m dal punto (percorsi {distance_traveled:.2f} m su "
                      f"{distance_commanded:.2f}): vale come arrivato, l'arco resta valido.")
                success_move = True

        if success_move:
            if not hasattr(self.env, '_traveled_arcs'):
                self.env._traveled_arcs = []
            self.env._traveled_arcs.append((self.robot_x, self.robot_y, move_x, move_y))

            if is_partial_step:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                self._stop_node_here(ax, ay, "sosta (passo parziale)")
            else:
                # Edge "travelled" only if the robot started from the node: otherwise the leg
                # walked is not the graph edge (see add_stop_node, no new node).
                if mission_graph.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y):
                    self.prm_graph.traversed_edges.add((min(self.robot_node_id, self.next_node_id),
                                                   max(self.robot_node_id, self.next_node_id)))
                self.robot_node_id = self.next_node_id
                self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
                self.path_waypoints.pop(0)

            # Breadcrumb for the returns: a GraphNav waypoint every
            # GRAPHNAV_WAYPOINT_MIN_SPACING_M of path travelled.
            map_recording.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client,
                                     self.robot_node_id, self.target_row, self.target_col)

        elif abort_reason is not None:
            print(f"[STOP] Movimento verso ({move_x:.2f}, {move_y:.2f}) interrotto dopo "
                  f"{distance_traveled:.2f}m: {abort_reason}.")
            MISSION_STATS['frontier_aborts'] += 1
            if distance_traveled > 0.05:
                self._stop_node_here(*self._walked_path(self.robot_x, self.robot_y), "sosta (arresto)")
            return 'fermo'

        else:
            print(f"[FAIL] Comando di movimento fallito meccanicamente per ({move_x:.2f}, {move_y:.2f}) "
                  f"(percorsi {distance_traveled:.2f}m su {distance_commanded:.2f}m richiesti)")

            movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                    "dopo il fallimento del movimento")

            if distance_traveled > 0.05:
                self._walked_path(self.robot_x, self.robot_y)

            # If the robot did not start from the node, the failed leg is not the graph edge.
            on_node = mission_graph.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y)
            self._drop_edge(self.robot_node_id, self.next_node_id, 'missione' if on_node else 'ora')
            print(f"[INFO] Tratto {self.robot_node_id}-{self.next_node_id} scartato dopo fallimento fisico.")
            if self._go_back((self.robot_node_id, self.next_node_id),
                             f"dal nodo {self.robot_node_id}"):
                return 'fermo'

            fail_save_path = os.path.join(self.mission_folder,
                                          f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_MOVE_FAIL.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(next_x, next_y), iteration=self.iteration, env=self.env,
                save_path=fail_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up
            )
            return FAILED
        return 'avanti'

    def _finish(self):
        """After the loop: True if the robot is in the cell (or the arrival just outside was accepted)."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()

        # Final check of the position at the end of the path
        self.robot_x, self.robot_y, _, _ = spotUtils.getPosition(self.robot_state_client)
        if self.arrived_near_goal:
            return True
        return self.env.is_point_in_cell(self.robot_x, self.robot_y, self.target_row, self.target_col)



def attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client,
                                     env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                                     iteration=0,
                                     recordingInterface=None, verification_tracker=None, mobility_kwargs=None):
    """One attempt to enter cell (target_row, target_col): see CellAttempt."""
    return CellAttempt(local_grid, global_grid, robot_state_client, command_client,
                          env, target_row, target_col, sampler, prm_graph, mission_folder,
                          iteration, recordingInterface, verification_tracker,
                          mobility_kwargs).run()
