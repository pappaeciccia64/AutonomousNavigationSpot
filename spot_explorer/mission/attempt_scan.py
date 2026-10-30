"""Scan -> maps -> graph -> path, the phases of CellAttempt (see cell_attempt.py) that look
at the world: the first scan of the attempt, the fresh scan of every turn, the local data
and the edge update in the PRM, the replan, the saved data."""
import os
import time
import numpy as np
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from spot_explorer.graphnav import map_recording
from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.planning import safe_frontier
from spot_explorer.planning import roadmap as prm_graph_module   # inside functions `prm_graph` is often the parameter (instance)
from spot_explorer.planning import edge_evaluation
from spot_explorer.planning import mission_graph                   # mission node lattice
from spot_explorer import config as P
from spot_explorer.records import (MISSION_STATS, MISSION_CTX, mission_z0, save_npz_async, timing_add, visualize_grid_with_candidates)
from spot_explorer.planning import goal
from spot_explorer.mission import recovery
from spot_explorer.mission.attempt_common import STOP, FAILED


class ScanSteps:
    """Mixin of CellAttempt: the state is CellAttempt's, see its docstring for the map of the phases."""

    def _first_scan(self):
        """Get up if fallen, first scan of the attempt, robot pose, global maps updated and
        saved. The processed scan, or None if it does not arrive."""
        # The robot gets here from another move (navigation between cells, retreat): if it
        # fell in the meantime it gets up before anything else. Raises RobotFallenError if it cannot.
        # Then it leaves the hazard disk by retracing its steps: once up, it is still INSIDE
        # the spot of the fall, and from there every PRM edge would be discarded.
        if movements.check_and_recover_fall(self.robot_state_client, self.command_client,
                                   self.global_grid.global_occupancy_map, "at the start of the attempt") is not None:
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)

        # 1. The scan: the layers in one request, then (below) their processing
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Local grid acquisition failed.")
            return None
        main_proto = scan.proto
        pts, cells_obstacle_dist = scan.pts, scan.od
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size

        # =========================================================================
        # GLOBAL MAPS: created at the first scan, then accumulated for the whole mission
        # =========================================================================
        if self.global_grid.global_occupancy_map is None:
            self.global_grid.global_occupancy_map = global_maps.GlobalGrid(resolution=cell_size)
            print("[MAP] Initialized new persistent global occupancy map.")

        # Global height map: same native resolution as the local grid (no
        # reduction), created like the occupancy one.
        if self.global_grid.global_terrain_map is None:
            self.global_grid.global_terrain_map = global_maps.GlobalTerrainGrid(resolution=cell_size)
            print("[MAP] Initialized new persistent global terrain map (height).")

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
                                ground_label="scan-initial")
        terrain_real, is_valid = scan.terrain, scan.is_valid
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
        # (edge_evaluation.LIVE_PREFILTER_OD_M): at build time the global map has not yet
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
            print(f"[EDGES] Scan not usable for the preliminary discard ({e}).")

        t_build = time.perf_counter()
        # CAREFUL WITH THE NAMES: here `self.prm_graph` is the PRM instance, not the module of the
        # same name (imported as prm_graph_module). The candidate pairs are those of the mission
        # graph (see mission_graph.build_mission_prm); vetoes and weights are computed by
        # PRM._evaluate_and_add_edge.
        mission_graph.rebuild_prm_edges(self.prm_graph, self.global_map)
        timing_add('graph construction', time.perf_counter() - t_build)
        _upd = getattr(getattr(self.prm_graph, 'mission_graph', None), 'last_update', None)
        print(f"[TIMING] {'local update' if _upd and _upd[2] else 'full build'} "
              f"of the graph: {time.perf_counter() - t_build:.2f} s")
        if getattr(self.prm_graph, 'live_veto_last', 0):
            print(f"[EDGES] {self.prm_graph.live_veto_last} edges discarded at once by the live "
                  f"scan (they pass less than {edge_evaluation.LIVE_PREFILTER_OD_M:.2f} m from an "
                  f"obstacle just seen), before the slope.")
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
            label="attempt start")
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
            print(f"[FAIL] No valid goal in cell ({self.target_row},{self.target_col}) from the current position")
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
            label=f"goal of cell ({self.target_row},{self.target_col})", is_robot=False)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
        path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)

        if path_ids is None:
            print(
                f"[GRAPH DISCONNECTED] No path in the PRM from node {self.robot_node_id} to node "
                f"{self.goal_id}: the two nodes are in separate components of the graph. It is not a "
                f"Dijkstra error -- it means that all the connecting edges have "
                f"been vetoed (occupancy or slope). See the counts in [PRM MISSION SUMMARY].")
            print(f" -> Robot start coordinates: ({self.robot_x:.2f}, {self.robot_y:.2f})")
            print(f" -> Cell target coordinates: ({self.target_x:.2f}, {self.target_y:.2f})")


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
            print("[ERROR] Local acquisition failed during tracking. Precautionary stop.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED
        self.t_scan = time.perf_counter()

        self.scan = scan
        scan.grid_origin_x, scan.grid_origin_y = self.local_grid.grid_origin(scan)
        scan.shape = (scan.num_y, scan.num_x)

        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-step")

        self.t_proc = time.perf_counter()
        body_core = global_maps.fold_scan_into_global_maps(
            self.global_map, self.global_terrain_map, scan.pts, scan.terrain, scan.obstacle_mask,
            scan.is_valid, scan.od, scan.shape, (self.robot_x, self.robot_y, self.robot_yaw))
        self.t_maps = time.perf_counter()

        # 2. Fresh local data into the PRM, then replanning from the current node if needed
        scan.grad_2d = scan.grad.reshape(scan.shape)
        scan.rough_2d = scan.rough.reshape(scan.shape)
        scan.terrain_2d = scan.terrain.reshape(scan.shape)

        # The local data is always injected: it costs nothing (it stores the array references) and
        # it is needed anyway by sample_terrain_between_points for the gait of the leg about to
        # be travelled.
        self.prm_graph.update_local_grid_data(
            grad_values_2d=scan.grad_2d,
            rough_values_2d=scan.rough_2d,
            terrain_values_2d=scan.terrain_2d,
            valid_2d=spot_grid.slope_usable_mask(scan.is_valid, scan.od, scan.shape),
            grid_origin_x=scan.grid_origin_x,
            grid_origin_y=scan.grid_origin_y,
            cell_size=scan.cell_size
        )

        # Snapshot of the scan just made: it is needed already here -- by the preliminary edge
        # discard in the refresh (see edge_evaluation.LIVE_PREFILTER_OD_M), by that of the nodes
        # in obstacles -- and then by the safe frontier. It must be created ONLY ONCE per scan:
        # make_grid_snapshot also updates the temporal obstacle filter.
        self.snap = safe_frontier.make_grid_snapshot(
            scan.od, scan.rough, scan.is_valid, scan.valid, scan.terrain,
            scan.num_x, scan.num_y, scan.grid_origin_x, scan.grid_origin_y, scan.cell_size,
            self.robot_x, self.robot_y, unwritten=scan.unwritten, robot_yaw=self.robot_yaw)

        # Free-space map: HERE, not together with global_map.update, because it needs the
        # filtered obstacle_distance that lives in the snapshot (see global_maps.update_clearance_map).
        # It is fed only by the navigation loop, so all observations go through
        # the same filter.
        global_maps.update_clearance_map(self.global_map, scan.pts, self.snap, scan.is_valid, scan.unwritten,
                              body_core, (self.robot_x, self.robot_y))

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
                print(f"[EDGES] {n_live} edges out of the graph because of the live scan "
                      f"({self.prm_graph.live_veto_last} re-evaluated, {self.prm_graph.live_prune_last} "
                      f"beyond the refresh radius), before the slope.")
        else:
            # No new data under the edges: the weights of the previous turn are kept and
            # there is no replan (the block below runs only if touched_nodes is not empty).
            touched_nodes = set()
            MISSION_STATS['prm_refresh_skipped'] += 1


        # Nodes INSIDE obstacles: out of the graph (see prune_prm_nodes_in_obstacles).
        pruned_nodes = safe_frontier.prune_prm_nodes_in_obstacles(self.prm_graph, self.snap)
        if pruned_nodes:
            MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
            print(f"[NODES] {len(pruned_nodes)} nodes discarded: less than "
                  f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m of free space where we see them "
                  f"(first: {pruned_nodes[:8]}).")

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
                    f"Replan failed: no path from {self.robot_node_id} to the goal "
                    f"({self.target_x:.2f}, {self.target_y:.2f}) in the updated graph, while "
                    f"from the robot the graph goes on: it is the goal that is isolated.",
                    "no path in the updated graph")
            else:
                print(f"[REPLAN FAILED] No existing path from {self.robot_node_id} to "
                      f"{self.goal_id} in the updated graph. I keep the previous path: the "
                      f"safe frontier decides anyway how far it can advance.")
        elif chosen_path_ids != current_plan_ids:
            self._use_path(chosen_path_ids)
            print(f"[REPLAN] Path updated ({len(touched_nodes)} local nodes re-evaluated): "
                  f"{len(self.path_waypoints)} waypoints left.")

            self.replan_counter += 1
            MISSION_STATS['replan_count'] += 1
            replan_save_path = os.path.join(
                self.mission_folder, f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_replan_{self.replan_counter}.png"
            ) if self.mission_folder else None

            visualize_grid_with_candidates(
                pts=self.scan.pts, terrain_real=self.scan.terrain, obstacle_mask=self.scan.obstacle_mask,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=replan_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.scan.od, valid_values=self.scan.valid,
                grad_values=self.scan.grad, rough_values=self.scan.rough,
                include_diagnostics=False  # light: main view and global map
            )
        else:
            print(f"[REPLAN STABLE] Path unchanged from {self.robot_node_id} to {self.goal_id} "
                  f"({len(touched_nodes)} local nodes re-evaluated): the alternative does not "
                  f"pass the stability threshold. Carrying on with the current plan.")

    def _save_scan_bundle(self):
        """Everything needed to review this scan afterwards (scans/*.npz): grid layers,
        filtered snapshot, path, frontier and decision. See SAVE_SCAN_BUNDLES."""
        # A rotation refusal can be recomputed offline from obstacle_distance, robot_xyz,
        # robot_yaw and decision_target.
        if P.SAVE_SCAN_BUNDLES and self.mission_folder is not None:
            gf = self.local_grid.last_ground_filter
            shape = self.scan.shape
            extra = {}
            if P.SAVE_EXTRA_LAYERS:
                t_x = time.perf_counter()
                for name, lay in self.local_grid.extra_layers(P.SAVE_EXTRA_LAYERS).items():
                    extra[name] = lay['values']
                    extra[f'{name}_origin'] = lay['origin']
                    extra[f'{name}_cell'] = lay['cell']
                timing_add('scan', time.perf_counter() - t_x)
            save_npz_async(
                os.path.join(self.mission_folder, "scans",
                             f"scan_it{self.iteration:03d}_cell{self.target_row}_{self.target_col}_{self.loop_iter:03d}.npz"),
                time=time.time(), iteration=self.iteration, cell=np.array([self.target_row, self.target_col]),
                loop_iter=self.loop_iter,
                terrain_raw=np.asarray(self.scan.terrain_raw, dtype=np.float32).reshape(shape),
                terrain_valid_raw=np.asarray(self.scan.valid, dtype=np.float32).reshape(shape),
                obstacle_distance=np.asarray(self.scan.od, dtype=np.float32).reshape(shape),
                # The layer the frontier DECIDES on (median over the last readings + hysteresis,
                # see safe_frontier._ObstacleHistory) and the latch mask. Without these
                # two an offline replay cannot reproduce the blocks that come from the
                # thread's readings during the move. Same orientation as the others:
                # (num_y, num_x), row = y.
                obstacle_distance_filtered=np.asarray(self.snap.obstacle_dist, dtype=np.float32),
                blocked_latch=(np.zeros(shape, bool) if self.snap.blocked_latch is None
                               else np.asarray(self.snap.blocked_latch, dtype=bool)),
                terrain_corrected=np.asarray(self.scan.terrain, dtype=np.float32).reshape(shape),
                is_valid=np.asarray(self.scan.is_valid, dtype=bool).reshape(shape),
                roughness=np.asarray(self.scan.rough, dtype=np.float32).reshape(shape),
                gradient=np.asarray(self.scan.grad, dtype=np.float32).reshape(shape),
                obstacle_mask=np.asarray(self.scan.obstacle_mask, dtype=np.int8).reshape(shape),
                footprint_mask=(np.zeros(shape, bool) if self.scan.footprint_mask is None
                                else np.asarray(self.scan.footprint_mask, dtype=bool).reshape(shape)),
                grid_origin=np.array([self.scan.grid_origin_x, self.scan.grid_origin_y]), cell_size=self.scan.cell_size,
                robot_xyz=np.array([self.robot_x, self.robot_y, self.robot_z]), robot_yaw=self.robot_yaw,
                ground_z=np.nan if gf.get('ground_z') is None else gf['ground_z'],
                ground_n_above=gf.get('n_above', 0),
                mission_z0=mission_z0(),
                terrain_raw_zero=np.nan if spot_grid.raw_zero(self.scan.grids_data) is None else spot_grid.raw_zero(self.scan.grids_data),
                terrain_scale=np.nan if not spot_grid.raw_scale(self.scan.grids_data) else spot_grid.raw_scale(self.scan.grids_data),
                unwritten=np.asarray(self.scan.unwritten, dtype=bool).reshape(shape),
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
                    terrain_raw=np.asarray(self.scan.terrain_raw).reshape(self.scan.shape),
                    terrain_valid_raw=np.asarray(self.scan.valid).reshape(self.scan.shape),
                    obstacle_mask=np.asarray(self.scan.obstacle_mask).reshape(self.scan.shape),
                    roughness=self.scan.rough_2d,
                    grid_origin_x=self.snap.origin_x, grid_origin_y=self.snap.origin_y, cell_size=self.snap.cell_size,
                    robot_xy=np.array([self.robot_x, self.robot_y]), robot_yaw=self.robot_yaw,
                    polyline=np.array(self.polyline), frontier_dist=self.frontier['dist'],
                    frontier_reason=self.frontier['reason'], frontier_stop_xy=np.array(self.frontier['stop_xy']),
                    ground_z=np.nan if self.local_grid.last_ground_filter.get('ground_z') is None
                    else self.local_grid.last_ground_filter['ground_z'])
                print(f"[DATA SAVE] Block data saved in {os.path.basename(npz_blocked)}")
            except Exception as e:
                print(f"[DATA SAVE] Could not save the block data: {e}")
