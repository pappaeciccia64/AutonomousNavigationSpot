"""The phases of CellAttempt (see cell_attempt.py) that execute the decided move: check of
the arrival pose, then one or more legs on the polyline confirmed by the frontier, each with
gait, motion command, stop halfway and records."""
import os
import time
from datetime import datetime
from types import SimpleNamespace
import numpy as np
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from spot_explorer.graphnav import map_recording
from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.perception import slopes
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier
from spot_explorer.mission import mission_prm
from spot_explorer import config as P
from spot_explorer.records import (MISSION_STATS, BodyTiltRecorder, append_csv, append_trajectory, log_segment, timing_add, visualize_grid_with_candidates)
from spot_explorer.mission.attempt_common import CONTINUE, FAILED


class LegSteps:
    """Mixin of CellAttempt: the state is CellAttempt's, see its docstring for the map of the phases."""

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
            print(f"[MULTINODE] The frontier confirms "
                  f"{safe_frontier.allowed_advance(self.frontier):.2f} m: I travel {n_legs} "
                  f"legs in a row {[w[0] for w in self.path_waypoints[:n_legs]]} without stopping "
                  f"to rescan.")
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
                vision_tform_body_current = get_a_tform_b(self.scan.proto.local_grid.transforms_snapshot,
                                                          VISION_FRAME_NAME, BODY_FRAME_NAME)
            self.no_progress = 0
            self.retreats_in_a_row = 0   # a real advance breaks the chain of back-offs
            if is_partial_step:
                print(f"[PARTIAL STEP] Towards node {self.next_node_id}: advancing {self.decision['dist']:.2f} m "
                      f"out of {np.hypot(next_x - self.robot_x, next_y - self.robot_y):.2f} up to "
                      f"({move_x:.2f}, {move_y:.2f}), then looking again with a fresh scan.")
            else:
                print(f"[OK] Leg {self.robot_node_id}->{self.next_node_id} confirmed free by the frontier. Executing the move...")

            outcome = self._walk_leg(move_x, move_y, next_x, next_y, is_partial_step,
                                         vision_tform_body_current)
            if outcome == 'stopped':
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
                print(f"[ARRIVAL] The step of {step_len:.2f} m towards ({move_x:.2f}, {move_y:.2f}) "
                      f"(obstacle_distance there {od_arrival:.2f} m) would take me where there are no "
                      f"exits: {n_us_a}/{len(fan_arr)} usable directions ({fan_txt_a}). "
                      f"I do not go ({self.arrival_refusals}/{P.MAX_ARRIVAL_REFUSALS}), I discard the leg "
                      f"{self.robot_node_id}->{self.next_node_id} for the rest of the attempt and replan.")
                # Out of the plan for the REST OF THE ATTEMPT (attempt penalty, like the
                # returns: see _trap_backtrack), not only for now: the leg itself is free,
                # it is the arrival point that has no exits, so the refresh would put it back
                # into the graph at the next turn and the same plan would be made again. On 28-10 12:16 the
                # same step was refused three times in a row, and executed at the third.
                self._drop_edge(self.robot_node_id, self.next_node_id, 'attempt')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None and len(new_ids) > 1:
                    self._use_path(new_ids)
                    return CONTINUE
                # Knowing in advance that the arrival point is a dead end and going there
                # anyway makes no sense: this side of the cell is given up, and the cell will be
                # retried later from another one (finalize_or_defer_blocked_cell).
                print(f"[ARRIVAL] No alternative in the graph and the arrival point has no "
                      f"exits: I do not go. I give up entering ({self.target_row},{self.target_col}) "
                      f"from this side.")
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
        if not rot_leg['turn'] or (rot_leg['why'] == 'swept sector' and not inside_rot):
            reason_txt = ("the body would touch" if not rot_leg['turn']
                      else "the rotation circle goes outside the scan data")
            print(f"[MULTINODE] Leg {leg_i + 1}/{n_legs}: to point at node "
                  f"{self.next_node_id} {rot_leg['dyaw_deg']:+.0f} deg are needed and {reason_txt}. "
                  f"I stop here and look again with a fresh scan.")
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
                print(f"[MULTINODE] Leg {leg_i + 1}/{n_legs}: node {self.next_node_id} "
                      f"(obstacle_distance {od_leg:.2f} m) would have no exits. I stop "
                      f"here and look again with a fresh scan.")
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
                    obstacle_grid=np.asarray(self.scan.obstacle_mask).reshape(self.scan.shape),
                    terrain_grid=np.asarray(self.scan.terrain).reshape(self.scan.shape),
                    grid_origin_x=self.scan.grid_origin_x, grid_origin_y=self.scan.grid_origin_y, cell_size=self.scan.cell_size,
                    arc_xy=np.array([self.robot_x, self.robot_y, move_x, move_y]),
                    long_slope=segment_long_slope, lat_slope=segment_lat_slope,
                    slope_threshold=slopes.SLOPE_THRESHOLD)
                MISSION_STATS['near_threshold_saves'] += 1
                margin_left = slopes.SLOPE_THRESHOLD - max(segment_long_slope, segment_lat_slope)
                print(f"[NEAR-LIMIT] Edge {self.robot_node_id}->{self.next_node_id} near the threshold "
                      f"(long={segment_long_slope:.3f}, lat={segment_lat_slope:.3f}, "
                      f"threshold={slopes.SLOPE_THRESHOLD:.3f}, remaining margin={margin_left:+.3f}) "
                      f"-- data saved in {os.path.basename(npz_near)}")
            except Exception as e:
                print(f"[DATA SAVE] Could not save the near-threshold file: {e}")

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
            if (f['reason'] == 'end_of_data'
                    and float(np.hypot(frx - self.robot_x, fry - self.robot_y)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                abort_state['bad'] = 0      # still rotating in place: see the constant
                return False
            if reach + P.FRONTIER_ABORT_TOLERANCE_M < remaining:
                abort_state['bad'] += 1
                if abort_state['bad'] >= P.FRONTIER_ABORT_CONFIRM:
                    sxa, sya = f['stop_xy']
                    abort_state['reason'] = (f"the frontier got shorter ({f['reason']} at "
                                             f"({sxa:.2f}, {sya:.2f})): reachable {reach:.2f} m "
                                             f"out of {remaining:.2f} m to go")
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
        timing_add('motion', move_duration)
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
            print(f"[LOG] Trajectory/frontiers of the move not recorded: {e}")

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
            print(f"[LOG] Could not record the segment: {e}")
        return success_move, distance_traveled, distance_commanded, abort_state['reason']

    def _walk_leg(self, move_x, move_y, next_x, next_y, is_partial_step, vision_tform_body_current):
        """One leg: gait, move, outcome. 'avanti' = succeeded, the next leg can be done;
        'stopped' = interrupted or replanned, look again; FAILED = physical failure."""
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
                print(f"[ALMOST] Command towards ({move_x:.2f}, {move_y:.2f}) not completed, but the robot "
                      f"is {d_left:.2f} m from the point (travelled {distance_traveled:.2f} m out of "
                      f"{distance_commanded:.2f}): it counts as arrived, the edge stays valid.")
                success_move = True

        if success_move:
            if not hasattr(self.env, '_traveled_arcs'):
                self.env._traveled_arcs = []
            self.env._traveled_arcs.append((self.robot_x, self.robot_y, move_x, move_y))

            if is_partial_step:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                self._stop_node_here(ax, ay, "stop (partial step)")
            else:
                # Edge "travelled" only if the robot started from the node: otherwise the leg
                # walked is not the graph edge (see add_stop_node, no new node).
                if mission_prm.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y):
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
            print(f"[STOP] Move towards ({move_x:.2f}, {move_y:.2f}) interrupted after "
                  f"{distance_traveled:.2f}m: {abort_reason}.")
            MISSION_STATS['frontier_aborts'] += 1
            if distance_traveled > 0.05:
                self._stop_node_here(*self._walked_path(self.robot_x, self.robot_y), "stop (abort)")
            return 'stopped'

        else:
            print(f"[FAIL] Motion command failed mechanically for ({move_x:.2f}, {move_y:.2f}) "
                  f"(travelled {distance_traveled:.2f}m out of {distance_commanded:.2f}m requested)")

            movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                    "after the motion failure")

            if distance_traveled > 0.05:
                self._walked_path(self.robot_x, self.robot_y)

            # If the robot did not start from the node, the failed leg is not the graph edge.
            on_node = mission_prm.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y)
            self._drop_edge(self.robot_node_id, self.next_node_id, 'mission' if on_node else 'now')
            print(f"[INFO] Leg {self.robot_node_id}-{self.next_node_id} discarded after a physical failure.")
            if self._go_back((self.robot_node_id, self.next_node_id),
                             f"from node {self.robot_node_id}"):
                return 'stopped'

            fail_save_path = os.path.join(self.mission_folder,
                                          f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_MOVE_FAIL.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.scan.pts, terrain_real=self.scan.terrain, obstacle_mask=self.scan.obstacle_mask,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(next_x, next_y), iteration=self.iteration, env=self.env,
                save_path=fail_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.scan.od, valid_values=self.scan.valid,
                grad_values=self.scan.grad, rough_values=self.scan.rough
            )
            return FAILED
        return 'ahead'
