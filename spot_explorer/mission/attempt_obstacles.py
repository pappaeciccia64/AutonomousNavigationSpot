"""The phases of CellAttempt (see cell_attempt.py) for when the robot cannot simply advance:
rotation refused (cone, straight back-off, trap, leg discarded), frontier that does not let
it advance (other goal, early replan, wait) and confirmed block."""
import os
import time
import numpy as np
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier
from spot_explorer.mission import mission_prm
from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, TrajectoryLog, visualize_grid_with_candidates
from spot_explorer.planning import goal
from spot_explorer.mission.attempt_common import CONTINUE, FAILED


class ObstacleSteps:
    """Mixin of CellAttempt: the state is CellAttempt's, see its docstring for the map of the phases."""

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
                margin_txt = (f"margin {rot['margin']:+.2f} m on the swept cells"
                              if rot['margin'] is not None else f"reason: {rot['why']}")
                print(f"[ROTATION] {rot['dyaw_deg']:+.0f} deg needed towards node "
                      f"{self.next_node_id}, but the body would touch ({margin_txt}; "
                      f"obstacle_distance under the robot {self.frontier['od_robot']:.2f} m). "
                      f"I do not walk sideways: attempt "
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
                        f"[TRAP] I cannot turn towards {self.next_node_id} and none of the "
                        f"{n_fan_r} directions of the circle is both travelable and "
                        f"reachable by turning. The widest: {fan_txt_r}. I retrace the "
                        f"leg I came from.")
                print(f"[ROTATION] Usable directions from here: {n_us_r}/{n_fan_r} "
                      f"({fan_txt_r}).")
                # Limit per ATTEMPT, not per pair of nodes (see MAX_ROTATION_DISCARDS).
                self.rotation_discards += 1
                if self.rotation_discards > P.MAX_ROTATION_DISCARDS:
                    print(f"[ROTATION] Already {self.rotation_discards - 1} legs discarded in this "
                          f"attempt because the rotation was impossible, and I am still here: "
                          f"it is not the edge, it is the place. I retrace the leg I came from.")
                    if self._go_back((self.robot_node_id, self.next_node_id),
                                     f"from node {self.robot_node_id} (too many rotation refusals)"):
                        return CONTINUE
                    return FAILED
                print(f"[ROTATION] Discarding the leg {self.robot_node_id}->{self.next_node_id}: "
                      f"it requires turning where there is no room FROM HERE. Replanning.")
                # NON-permanent discard: a rotation refusal depends on the robot's POSE,
                # not on the edge -- the same edge, approached from another point or with another
                # heading, can be perfectly travelable. Only what is confirmed by three still
                # scans or by a physical failure stays permanent.
                self._drop_edge(self.robot_node_id, self.next_node_id, 'now')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None:
                    self._use_path(new_ids)
                    return CONTINUE
                print("[ROTATION] No alternative in the graph from where I am. "
                      "I back away and defer the cell.")
                if self._go_back((self.robot_node_id, self.next_node_id),
                                 f"from node {self.robot_node_id}", skip_if_graph_cannot_help=True):
                    return CONTINUE
                return FAILED

    def _advance_in_cone(self, rot):
        """Rotation impossible: advance inside the allowed rotation cone, if it gets closer to
        the path (CONTINUE), otherwise None."""
        # --- 4a. FIRST OF ALL: can I GO FORWARD inside the rotation cone I am
        # allowed? Often going straight, or almost, gets closer to the goal than
        # any back-off (see plan_cone_move).
        cone_goal, _ = safe_frontier.point_along(self.polyline, P.CONE_PATH_LOOKAHEAD_M)
        cone = decisions.plan_cone_move((self.robot_x, self.robot_y), self.robot_yaw, self.snap, cone_goal)
        if cone is not None:
            print(f"[CONE] I cannot turn by {rot['dyaw_deg']:+.0f} deg, but rotating by "
                  f"{cone['dyaw_deg']:+.0f} I can advance {cone['dist']:.2f} m "
                  f"({cone['reason']}) and get {cone['gain']:.2f} m closer to the path "
                  f"({cone_goal[0]:.2f}, {cone_goal[1]:.2f}, {P.CONE_PATH_LOOKAHEAD_M:.1f} m "
                  f"ahead along the path). Going, instead of backing up.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['cone_moves'] += 1
            cx, cy = cone['target']
            self.need_refresh = True      # end of a move: see EVENT REFRESH
            with TrajectoryLog(self.robot_state_client, 'cone', (cx, cy)):
                ok_c, dist_c, _ = movements.navigate_to(
                    cx, cy, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                    get_a_tform_b(self.scan.proto.local_grid.transforms_snapshot,
                                  VISION_FRAME_NAME, BODY_FRAME_NAME),
                    should_abort=None, keep_heading=False, **self.mobility_kwargs)
            if dist_c > 0.05:
                self._stop_node_here(*self._walked_path(self.robot_x, self.robot_y), "stop (advance in the cone)")
                self.retreats_in_a_row = 0      # it made real progress: the chain breaks
            else:
                print(f"[CONE] I could not advance (travelled {dist_c:.2f} m).")
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
                mb = "" if margin_back is None else f" (margin {margin_back:+.2f} m from there)"
                print(f"[BACKOFF] I could back up by {back[2]:.2f} m, but from there the "
                      f"rotation of {rot['dyaw_deg']:+.0f} deg would stay impossible{mb}: "
                      f"I do not do it, I would move the problem by 60 cm.")
        if (back is not None and helps
                and self.retreats_in_a_row < P.MAX_STRAIGHT_RETREATS_IN_A_ROW
                and self.rotation_fails[edge_key] < P.MAX_ROTATION_FAILS_SAME_EDGE):
            bx, by, bdist = back
            self.retreats_in_a_row += 1
            print(f"[BACKOFF] Straight back by {bdist:.2f} m towards "
                  f"({bx:.2f}, {by:.2f}), without turning, to make room for the rotation "
                  f"({self.retreats_in_a_row}/{P.MAX_STRAIGHT_RETREATS_IN_A_ROW} in a row).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['straight_retreats'] += 1
            self.need_refresh = True      # end of a move: see EVENT REFRESH
            with TrajectoryLog(self.robot_state_client, 'backoff', (bx, by)):
                ok_back, dist_back = movements.move_to_world_point_without_turning(
                    bx, by, "vision", self.command_client, self.robot_state_client, **self.mobility_kwargs)
            if dist_back > 0.05:
                self._stop_node_here(*self._walked_path(self.robot_x, self.robot_y), "stop (back-off to rotate)")
            else:
                print(f"[BACKOFF] I could not back up "
                      f"(travelled {dist_back:.2f} m).")
            self.no_progress = 0
            return CONTINUE

        if back is None:
            print("[BACKOFF] There is no room to back up straight.")
        elif helps and self.retreats_in_a_row >= P.MAX_STRAIGHT_RETREATS_IN_A_ROW:
            print(f"[BACKOFF] Already {self.retreats_in_a_row} back-offs in a row without "
                  f"achieving anything: I stop backing up.")
        return None

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
            print(f"[FRONTIER] I cannot advance even {P.MIN_PARTIAL_STEP_M:.2f} m: looking again "
                  f"while still ({self.no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS} scans).")
            time.sleep(P.FRONTIER_RESCAN_WAIT_S)
            return CONTINUE

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
                    f"The goal ({self.target_x:.2f}, {self.target_y:.2f}) has obstacle_distance "
                    f"{od_goal:.2f} m, below the {geometry.FRONTIER_CLEARANCE_M:.2f} m the frontier "
                    f"requires: it cannot be reached from any direction.",
                    f"live free space {od_goal:.2f} m")
                if outcome is not None:
                    return outcome
            elif self._frontier_stops_at_goal():
                sx_g, sy_g = self.frontier['stop_xy']
                d_g = float(np.hypot(sx_g - self.target_x, sy_g - self.target_y))
                beyond = self.frontier['dist'] >= self.frontier['path_len']
                outcome = self._repick_goal(
                    f"The frontier stops for {self.frontier['reason']} at ({sx_g:.2f}, {sy_g:.2f}), "
                    f"{d_g:.2f} m from the goal ({self.target_x:.2f}, {self.target_y:.2f})"
                    + (", beyond the end of the path, where the nose would go" if beyond else "")
                    + ": it is the goal that does not work, not the way.",
                    f"the frontier stops {d_g:.2f} m from it ({self.frontier['reason']})")
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
                and self.frontier['reason'] in ('obstacle', 'roughness', 'slope')
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
                    f"[TRAP] None of the {n_fan} directions of the circle is both "
                    f"travelable and reachable by turning. The widest: {fan_txt}. "
                    f"I have nowhere to go and no way to turn: I retrace the leg I "
                    f"came from, without discarding other edges.")

            self.early_replans += 1
            MISSION_STATS['early_replans'] += 1
            print(f"[EARLY] {self.frontier['reason']} at ({sx_b:.2f}, {sy_b:.2f}) on the leg "
                  f"{ea}->{eb}: I discard it now and look for another path, without "
                  f"waiting for the confirmation while still ({self.early_replans}/{P.MAX_EARLY_REPLANS}). "
                  f"Usable directions from here: {n_us}/{n_fan}.")
            # NON-permanent discard: after a single scan the edge must be removed from the plan,
            # not condemned for the mission (see mark_edge_blocked_soft).
            self._drop_edge(ea, eb, 'now')
            # But if it is the SECOND time on the same leg, the "for now" discard was not
            # enough: the refresh put it back into the graph (the preliminary discard looks at one
            # scan at 0.25 m, the frontier at the median of three with hysteresis up to 0.29),
            # and the plan went back to the previous one. Out of the plan for the rest of the
            # attempt, like the returns and [ARRIVO]. On 28-10 14:23, 8 [SUBITO] out of 14 were
            # repetitions of the same leg (197->569 four times in a row).
            key_s = (min(ea, eb), max(ea, eb))
            if key_s in self.early_replanned_edges:
                self._drop_edge(ea, eb, 'attempt')
                MISSION_STATS['early_replans_repeated'] = MISSION_STATS.get('early_replans_repeated', 0) + 1
                print(f"[EARLY] The leg {ea}->{eb} had already been discarded in this attempt and "
                      f"the plan came back to it: out of the plan for the rest of the attempt.")
            self.early_replanned_edges.add(key_s)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None and len(new_ids) > 1:
                self._use_path(new_ids)
                print(f"[EARLY] Alternative path with {len(self.path_waypoints)} waypoints, "
                      f"from where I am.")
                return CONTINUE
            print("[EARLY] The graph offers no alternatives from here: I stay on the path and "
                  "let the frontier decide.")
        return None

    def _block(self):
        """Block confirmed by several still scans: edge discarded, replan or return."""
        if self.decision['kind'] == 'blocked':
            k = self.decision['segment']
            edge_a = self.robot_node_id if k == 0 else self.path_waypoints[k - 1][0]
            edge_b = self.path_waypoints[min(k, len(self.path_waypoints) - 1)][0]
            sx, sy = self.frontier['stop_xy']
            print(f"[BLOCK] Confirmed up close: {P.FRONTIER_BLOCK_CONFIRM_SCANS} consecutive scans "
                  f"without being able to advance. Reason: {self.frontier['reason']} at ({sx:.2f}, {sy:.2f}), "
                  f"on the leg {edge_a}->{edge_b}.")
            MISSION_STATS['frontier_blocks'] += 1
            # Blocked right next to the goal (or beyond): it is the goal that does not work, not the
            # way. It is excluded and, if possible, another one is chosen (below, after the trap)
            # instead of discarding the leg and replanning towards the same goal.
            goal_blocked = self._frontier_stops_at_goal()
            d_goal = float(np.hypot(sx - self.target_x, sy - self.target_y))
            why_goal = f"block confirmed {d_goal:.2f} m from it ({self.frontier['reason']})"
            if goal_blocked:
                goal._refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                             why_goal)

            self._save_block_data()

            # Trap? If there is no usable direction, replanning
            # is useless by construction: the problem is not the edge, it is the place.
            trapped_blk, n_us_b, n_fan_b, fan_txt_b = self._escape_fan()
            print(f"[BLOCK] Usable directions from here: {n_us_b}/{n_fan_b} ({fan_txt_b}).")
            if trapped_blk:
                return self._escape_trap(
                    (edge_a, edge_b), "[TRAP] No way out by turning: I do not discard other "
                                      "edges, I retrace the leg I came from.")

            if goal_blocked and self.goal_repicks < P.MAX_BLOCK_REPLANS:
                outcome = self._repick_goal(
                    f"Block {d_goal:.2f} m from the goal ({self.target_x:.2f}, "
                    f"{self.target_y:.2f}): it is the goal that does not work, not the leg {edge_a}->{edge_b}.",
                    why_goal)
                if outcome is not None:
                    return outcome

            # Here the block is confirmed by FRONTIER_BLOCK_CONFIRM_SCANS still scans:
            # the invalidation stays permanent (unlike [SUBITO], see
            # mark_edge_blocked_soft). Exception: on the first leg, if the robot
            # is not on the node it is attached to, the blocked leg is robot->node, not
            # the graph edge: the edge is discarded only for now.
            on_node = k > 0 or mission_prm.robot_on_node(self.prm_graph, edge_a, self.robot_x, self.robot_y)
            self._drop_edge(edge_a, edge_b, 'mission' if on_node else 'now')
            self.block_replans += 1
            self.no_progress = 0

            new_ids = None
            if self.block_replans <= P.MAX_BLOCK_REPLANS:
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None:
                self._use_path(new_ids)
                print(f"[REPLAN] After the block ({self.block_replans}/{P.MAX_BLOCK_REPLANS}): alternative "
                      f"path with {len(self.path_waypoints)} waypoints, from where I am.")
                return CONTINUE

            print(f"[BLOCK] No alternative from where I am"
                  + (f" (already {P.MAX_BLOCK_REPLANS} replans after blocks)."
                     if self.block_replans > P.MAX_BLOCK_REPLANS else ".")
                  + " I back away and defer the cell.")
            if self._go_back((edge_a, edge_b),
                             f"from node {self.robot_node_id}", skip_if_graph_cannot_help=True):
                return CONTINUE

            blocked_save_path = os.path.join(self.mission_folder,
                                             f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.scan.pts, terrain_real=self.scan.terrain, obstacle_mask=self.scan.obstacle_mask,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=self.frontier['stop_xy'], iteration=self.iteration, env=self.env,
                save_path=blocked_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.scan.od, valid_values=self.scan.valid,
                grad_values=self.scan.grad, rough_values=self.scan.rough
            )
            return FAILED
