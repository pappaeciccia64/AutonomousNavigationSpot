"""One attempt to enter a cell: CellAttempt, the cruise towards a goal in the cell."""
from spot_explorer.robot import motion
from spot_explorer.perception import geometry
from spot_explorer.mission import cells
from spot_explorer.robot import connection

from spot_explorer.records import HANDOFF, MISSION_STATS
from spot_explorer.mission import recovery
from spot_explorer.mission import attempt_common
from spot_explorer.mission.attempt_common import CONTINUE, STOP, FAILED, battery_return_due
from spot_explorer.mission.attempt_scan import ScanSteps
from spot_explorer.mission import attempt_cruise
from spot_explorer.mission.attempt_cruise import CruiseSteps

# A cruise that ends for an exception (no new plan, a command that ends badly) is followed by a
# new one from where the robot is; this many in a row end the attempt.
MAX_CRUISE_RESTARTS = 3


# =========================================================================
# ENTERING A CELL: one attempt (CellAttempt)
# =========================================================================



class CellAttempt(ScanSteps, CruiseSteps):
    """Try to enter cell (target_row, target_col) from the robot's current position, in cruise.
    True if the robot ends up inside the cell (or accepted just outside), or walking into it
    after a hand-off.

    mobility_kwargs: optional dict forcing the gait (locomotion_hint, ground_mu_hint,
        swing_height) for the whole attempt (attempt_cruise._cruise_gait). Empty = gait chosen
        by the plans from the terrain ahead.

    How it is made. The start is the first cycle of the planning thread (attempt_cruise:
    _start_in_thread -- seen check, _local_data_into_graph, _goal_and_path, _reset_counters),
    then the cruise (attempt_cruise._cruise): the planning thread plans, the main thread sends
    the confirmed stretch; goal and cell change while walking. A cruise that ends for an
    exception is followed by a new one from where the robot is (MAX_CRUISE_RESTARTS); a fall,
    blocked on the spot and low battery end the attempt.

    Files: the start and the outcome here; the scan into the graph, the goal and the first path
    in attempt_scan.py (ScanSteps); the cruise in attempt_cruise.py (CruiseSteps), its
    planning cycle in planning_cycle.py. Outcomes and battery checks in attempt_common.py.
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
        self._handoff = False
        self._seen_only = False
        HANDOFF.mark("choice of the next cell")
        HANDOFF.start("attempt start (robot standing) -> first command", restart=False)
        try:
            if not self._can_start_in_thread():
                self.env._attempt_fail_why = ("no planning thread (the frontier tracker has no planning mode): "
                                              "the cruise cannot run")
                print(f"[ATTEMPT] Cell ({self.target_row},{self.target_col}): {self.env._attempt_fail_why}.")
                return False
            # The attempt start is the first cycle of the planning thread: the main thread
            # only gets up if fallen and starts the planner (attempt_cruise._start_in_thread).
            self._prepare_for_thread()
            started = None
            for k in range(attempt_cruise.START_TRIES):
                self.loop_iter = 1
                started = self._cruise_from_start()
                if started is not None or self.path_waypoints:
                    break
                print(f"[ATTEMPT] No path from the planning thread within {attempt_cruise.START_PLAN_WAIT_S:.0f} s "
                      f"(try {k + 1} of {attempt_cruise.START_TRIES}).")
            # A cruise that ended for an exception (not arrived, no hand-off, no block that
            # leaves the cell): the attempt goes on with a new cruise from where the robot is,
            # the start in the planning thread again. A fall (the robot gets up and backs
            # away), blocked on the spot, low battery, or MAX_CRUISE_RESTARTS in a row: the
            # attempt ends.
            restarts = 0
            while started is CONTINUE:
                why = getattr(self, 'cruise_end_reason', None) or "the cruise ended"
                if why.startswith('fall'):
                    # The robot gets up, backs away along the way it came, and the attempt
                    # ends (a failure of the cell).
                    motion.check_and_recover_fall(self.robot_state_client, self.command_client,
                                                  self.global_grid.global_occupancy_map, "during the cruise")
                    recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                                         mobility_kwargs=self.mobility_kwargs)
                    self.env._attempt_fail_why = f"fall during the cruise ({why})"
                    return False
                if why.startswith(attempt_cruise.BLOCKED_ON_SPOT):
                    # Every goal needs a way the robot cannot take from where it stands: a new
                    # cruise from here would find the same.
                    self.env._attempt_fail_why = f"cruise ended {why}"
                    print(f"[ATTEMPT] Cell ({self.target_row},{self.target_col}): "
                          f"{self.env._attempt_fail_why}: a failure of the cell, retried later.")
                    return False
                if battery_return_due():
                    print(f"[BATTERY] Interrupting the attempt on cell ({self.target_row},"
                          f"{self.target_col}) for the low-battery return.")
                    return False
                restarts += 1
                if restarts > MAX_CRUISE_RESTARTS:
                    self.env._attempt_fail_why = (f"{MAX_CRUISE_RESTARTS} cruises in a row ended early "
                                                  f"(the last: {why})")
                    print(f"[ATTEMPT] Cell ({self.target_row},{self.target_col}): "
                          f"{self.env._attempt_fail_why}: a failure of the cell, retried later.")
                    return False
                MISSION_STATS['cruise_restarts'] = MISSION_STATS.get('cruise_restarts', 0) + 1
                print(f"[ATTEMPT] The cruise ended ({why}): a new cruise from here "
                      f"({restarts} of {MAX_CRUISE_RESTARTS}).")
                self._prepare_for_thread()
                self.loop_iter = 1
                started = self._cruise_from_start()
            if started is FAILED:
                return False
            if started is STOP:
                return True if self._seen_only else self._finish()
            if started is None and not self.path_waypoints:
                # Not "unreachable now": that is a fact of the graph, this may be a slow or
                # stuck computer. A failure of the cell (retried later, set aside after
                # cell_rules.MAX_FAILURES), with its cause.
                self.loop_iter = 0
                why = getattr(self, 'start_failure_reason', None) or "the planning thread gave no path"
                self.env._attempt_fail_why = f"start in the planning thread failed: {why}"
                MISSION_STATS.setdefault('start_failures', []).append(
                    dict(cell=[self.target_row, self.target_col], reason=why))
                print(f"[ATTEMPT] Cell ({self.target_row},{self.target_col}): no start from the planning thread "
                      f"after {attempt_cruise.START_TRIES} tries ({why}): a failure of the cell, retried later.")
                return False
            if started is None:
                # The first plan did not allow to start, and no other goal did in time
                # (attempt_cruise._first_plan: a block at once): the cell left for later.
                self.env._attempt_fail_why = (f"no start in cruise: "
                                              f"{getattr(self, 'start_failure_reason', None) or 'no plan'}")
                print(f"[ATTEMPT] Cell ({self.target_row},{self.target_col}): {self.env._attempt_fail_why}.")
                return False
            return self._finish()
        finally:
            # A successful hand-off leaves the robot walking (see the hand-off in attempt_cruise), and
            # so does an attempt that failed before its loop: only computations, no command, so
            # there is no reason to stop the robot.
            if not self._handoff and getattr(self, 'loop_iter', 0) > 0:
                attempt_cruise.stop_if_moving(self.env, self.command_client, "end of the attempt")

    def _prepare_for_thread(self):
        """Attempt start in the planning thread: what stays in the main thread before the planner starts -- get up if
        fallen (and leave the spot of the fall), global maps created if this is the first attempt."""
        self.mobility_kwargs = self.mobility_kwargs or {}
        print(f"\n[ATTEMPT] Trying to enter cell ({self.target_row},{self.target_col}) from the current position "
              f"(start in the planning thread)...")
        if motion.check_and_recover_fall(self.robot_state_client, self.command_client,
                                         self.global_grid.global_occupancy_map, "at the start of the attempt") is not None:
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                                 mobility_kwargs=self.mobility_kwargs)
        self._ensure_global_maps()
        self.path_waypoints = []

    def _cell_already_seen(self):
        """
        The cell counts as explored without entering it when at least
        attempt_common.CELL_SEEN_FRACTION of it is already in the global map (seen free or obstacle, after the
        first scan of this attempt). The robot stays where it is (still walking after a
        hand-off: like an attempt that ends before its loop) and the mission marks the cell
        visited, its GraphNav waypoint where the robot is. A cell seen from outside does not need
        to be entered, and entering it (often towards a goal against furniture) costs back-offs,
        refused rotations and returns with GraphNav.
        """
        if self.global_map is None:
            return False
        frac = cells.cell_seen_fraction(self.env, self.global_map, self.target_row, self.target_col)
        if frac is None or frac < attempt_common.CELL_SEEN_FRACTION:
            return False
        print(f"[SEEN] Cell ({self.target_row},{self.target_col}): {100 * frac:.0f}% of it already in the "
              f"global map (threshold {100 * attempt_common.CELL_SEEN_FRACTION:.0f}%): explored without entering it.")
        MISSION_STATS['cells_seen_only'] = MISSION_STATS.get('cells_seen_only', 0) + 1
        self._seen_only = True
        return True

    def _reset_counters(self):
        """Counters and memories of the attempt, before the first command."""
        self.loop_iter = 0
        self.cruise_stalls = 0       # consecutive cruises that did not walk (MAX_CRUISE_STALLS)
        self.goal_repicks = 0        # how many times the goal point was chosen again
        self.tried_goals = []        # goal points already tried and found unreachable

    def _reachable_from_robot(self):
        """reachable(x, y) for the goal choice: a path from the robot node (one search, see
        PRM.reachable_from). To be asked again after the graph changes."""
        return self.prm_graph.reachable_from(self.robot_node_id, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M)

    def _waypoints_from_ids(self, ids):
        """[(node_id, x, y), ...] without the first node (the robot position)."""
        out = []
        for nid in ids[1:]:
            nx, ny = self.prm_graph.get_node_position(nid)
            out.append((nid, nx, ny))
        return out

    def _use_path(self, ids):
        """The path becomes ids (from the robot node on)."""
        self.path_waypoints = self._waypoints_from_ids(ids)
        self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in ids]

    def _finish(self):
        """After the loop: True if the robot is in the cell (or the arrival just outside was accepted)."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()

        # Final check of the position at the end of the path
        self.robot_x, self.robot_y, _, _ = connection.getPosition(self.robot_state_client)
        if self._handoff:
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
