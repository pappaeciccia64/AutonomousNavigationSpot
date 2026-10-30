"""One attempt to enter a cell: CellAttempt, the navigation loop with the safe frontier."""
import time
import numpy as np

from spot_explorer.robot import motion
from spot_explorer.perception import geometry
from spot_explorer.mission import cells
from spot_explorer.robot import connection
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier

from spot_explorer.planning import roadmap

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, timing_add
from spot_explorer.planning import goal
from spot_explorer.mission import recovery
from spot_explorer.mission.attempt_common import CONTINUE, STOP, FAILED, battery_return_due
from spot_explorer.mission.attempt_scan import ScanSteps
from spot_explorer.mission.attempt_obstacles import ObstacleSteps
from spot_explorer.mission.attempt_legs import LegSteps


# =========================================================================
# ENTERING A CELL: one attempt (CellAttempt)
# =========================================================================


class CellAttempt(ScanSteps, ObstacleSteps, LegSteps):
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

    Files: the loop, the guards, the decision and the shared actions are here; the scan
    phases in attempt_scan.py (ScanSteps), rotation/wait/block in attempt_obstacles.py
    (ObstacleSteps), the legs in attempt_legs.py (LegSteps). Outcomes and battery checks
    in attempt_common.py.

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

        print(f"\n[ATTEMPT] Trying to enter cell ({self.target_row},{self.target_col}) from the current position...")

        scan = self._first_scan()
        if scan is None:
            return FAILED
        self._local_data_into_graph(scan)
        outcome = self._goal_and_path(scan)
        if outcome is not None:
            return outcome
        self._reset_counters()

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
          'now'       -- until the next refresh, which puts it back if the data says it is free
                         (see roadmap.mark_edge_blocked_soft);
          'attempt' -- for the rest of the attempt (attempt penalty);
          'mission'  -- forever: block confirmed while still or physical failure."""
        if duration == 'mission':
            self.prm_graph.mark_edge_invalid(a, b)
            return
        roadmap.mark_edge_blocked_soft(self.prm_graph, a, b)
        if duration == 'attempt':
            self.prm_graph.edge_penalty_persistent[(min(a, b), max(a, b))] = self.prm_graph._ATTEMPT_MARK

    def _walked_path(self, x0, y0):
        """The leg walked from (x0, y0) to the current position, for the retreat. Returns the
        current position."""
        ax, ay, _, _ = connection.getPosition(self.robot_state_client)
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
        if self._go_back(edge_ab, f"from node {self.robot_node_id}"):
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
            self.command_client, self.global_map, self.mobility_kwargs, self.scan.pts, self.scan.obstacle_mask,
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
        if self.frontier['reason'] not in ('obstacle', 'roughness', 'slope'):
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
        print(f"[TARGET] {why} I look for another one in the cell "
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
                self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, self.scan.pts, self.scan.od,
                self.sampler, global_map=self.global_map, exclude=self.tried_goals,
                reachable=self._reachable_from_robot)
            if new_tx is None or new_ty is None:
                print("[TARGET] No other valid point in this cell.")
                break
            goal_id = self.prm_graph.add_stop_node(
                new_tx, new_ty, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                label=f"goal chosen again in cell ({self.target_row},{self.target_col})",
                is_robot=False)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, goal_id)
            if new_ids is not None:
                self.target_x, self.target_y, self.goal_id = new_tx, new_ty, goal_id
                break
            print(f"[TARGET] No path in the graph to ({new_tx:.2f}, {new_ty:.2f}): trying the "
                  f"next point.")
            self.tried_goals.append((float(new_tx), float(new_ty)))
        if new_ids is None:
            self.target_x, self.target_y, self.goal_id = old
            print("[TARGET] No new goal with a path: I stay on the previous plan.")
            return None
        self.best_goal_dist = float('inf')    # new goal: progress from scratch
        self._use_path(new_ids)
        print(f"[REPLAN] To the new goal ({self.target_x:.2f}, "
              f"{self.target_y:.2f}): {len(self.path_waypoints)} waypoint.")
        return CONTINUE

    def _guards(self):
        """Loop limits: too many turns (MAX_LOOP_ITERATIONS), fall, too many still turns."""
        self.loop_iter += 1
        self.t_iter = time.perf_counter()
        if self.loop_iter > P.MAX_LOOP_ITERATIONS:
            print(f"[STALL] {P.MAX_LOOP_ITERATIONS} turns of the navigation loop without arriving: "
                  f"interrupting the attempt on cell ({self.target_row},{self.target_col}).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED

        # Battery below threshold: the attempt stops here, the mission closes and returns
        # (see Mission.explore).
        if battery_return_due():
            print(f"[BATTERY] Interrupting the attempt on cell ({self.target_row},"
                  f"{self.target_col}) for the low-battery return.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED

        if motion.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                   "during the navigation loop") is not None:
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)
            return FAILED

        self.robot_x, self.robot_y, self.robot_z, robot_quat = connection.getPosition(self.robot_state_client)

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
            print(f"[STALL] {self.idle_loops} consecutive loop turns without moving by "
                  f"{P.IDLE_MOVE_M:.2f} m, with the robot at ({self.robot_x:.2f}, {self.robot_y:.2f}). "
                  f"Whatever the reason, going on spinning here leads nowhere: "
                  f"I give up entering ({self.target_row},{self.target_col}) from this side.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FAILED

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
            print(f"[SHORTCUT] Direct stretch confirmed free to node {self.path_waypoints[0][0]}: "
                  f"skipping {len(skipped)} intermediate nodes {skipped}.")
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
        # ARRIVAL_OUTSIDE_CELL_M). On 22-10 16:04: "0.06 m to go ... still outside
        # cell (2,7)", with the robot outside by 0.01 m.
        robot_border = cells.cell_border_distance(self.env, self.target_row, self.target_col, self.robot_x, self.robot_y)
        near_goal = (not in_target_cell and robot_border is not None
                     and -robot_border <= P.ARRIVAL_OUTSIDE_CELL_M)
        self.decision = decisions.decide_next_move((self.robot_x, self.robot_y), self.path_waypoints, self.frontier, self.no_progress,
                                    in_target_cell=in_target_cell or near_goal)
        if self.decision.get('final'):
            MISSION_STATS['final_steps'] = MISSION_STATS.get('final_steps', 0) + 1
            print(f"[FINAL STEP] The frontier allows only {self.decision['dist']:.2f} m, below the "
                  f"minimum step of {P.MIN_PARTIAL_STEP_M:.2f} m, but it is enough: after the step "
                  f"the goal is {self.frontier['path_len'] - self.decision['dist']:.2f} m away, within "
                  f"the arrival tolerance of {P.GOAL_REACHED_TOLERANCE_M:.2f} m.")
        if self.decision['kind'] == 'arrived' and near_goal:
            self.arrived_near_goal = True
            MISSION_STATS['arrivals_near_goal'] += 1
            print(f"[GOAL] The robot is outside cell ({self.target_row},{self.target_col}) by "
                  f"{-robot_border:.2f} m, {self.frontier['path_len']:.2f} m from the goal: it counts as "
                  f"arrival (outside by at most {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m, the nose is inside).")
        if (self.decision['kind'] in ('wait', 'blocked') and not in_target_cell and not near_goal
                and self.frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M):
            print(f"[GOAL] {self.frontier['path_len']:.2f} m to go, within the tolerance of "
                  f"{P.GOAL_REACHED_TOLERANCE_M:.2f} m, but the robot is still outside cell "
                  f"({self.target_row},{self.target_col}): it does not count as arrival, carrying on as for a "
                  f"blocked frontier.")
        self.next_node_id, next_x, next_y = self.path_waypoints[0]
        self.keep_heading = False     # always False (no sideways): it stays in the saved data and in navigate_to

        # The bundle is saved here, before the phases that can close the turn (rotation,
        # wait, block): otherwise those scans would not end up on disk.
        self._save_scan_bundle()

    def _timings(self):
        """Timings of the turn, for [TIMING] and the summary."""
        t_front = time.perf_counter()

        timing_add('scan', self.t_scan - self.t_iter)
        timing_add('grid processing', self.t_proc - self.t_scan)
        timing_add('global maps', self.t_maps - self.t_proc)
        timing_add('PRM and replanning', self.t_prm - self.t_maps)
        timing_add('frontier and decision', t_front - self.t_prm)
        print(f"[TIMING] scan {self.t_scan - self.t_iter:.2f} s | processing {self.t_proc - self.t_scan:.2f} | "
              f"maps {self.t_maps - self.t_proc:.2f} | PRM {self.t_prm - self.t_maps:.2f} | frontier {t_front - self.t_prm:.2f} "
              f"| decision: {self.decision['kind']}")

    def _node_or_arrival(self):
        """Path node reached, or arrival accepted before the end."""
        if self.decision['kind'] == 'reached_node':
            self.robot_node_id = self.next_node_id
            self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
            self.path_waypoints.pop(0)
            self.no_progress = 0
            return CONTINUE

        if self.decision['kind'] == 'arrived':
            print(f"[GOAL] The frontier does not allow the last {self.frontier['path_len']:.2f} m "
                  f"({self.frontier['reason']}), within the tolerance of {P.GOAL_REACHED_TOLERANCE_M:.2f} m: "
                  f"I consider the goal reached.")
            return STOP

    def _finish(self):
        """After the loop: True if the robot is in the cell (or the arrival just outside was accepted)."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()

        # Final check of the position at the end of the path
        self.robot_x, self.robot_y, _, _ = connection.getPosition(self.robot_state_client)
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
