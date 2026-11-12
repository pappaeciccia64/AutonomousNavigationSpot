"""
THE STOP-AND-GO START OF AN ATTEMPT, out of the main code (11-11-2026): the main thread scans
standing (or reuses the last scan of the cruise), folds it into the maps, puts the local data
into the graph, chooses goal and path, then the loop of the attempt.

Since 11-11 the attempt start is the first cycle of the planning thread
(attempt_cruise.CruiseSteps._start_in_thread): with the continuous motion on (the mission as in
the field) nothing here is used. This mixin of CellAttempt serves only the stop-and-go mode
(attempt_cruise.CONTINUOUS_MOTION = False: test_e2e_loop, most of test_mission, a mission run
with the cruise off), and stays runnable for comparisons.

CHAIN_REUSE_SCAN (04-11 16:52, CORE): the attempt that starts with the robot still walking used
the last scan of the cruise (env._chain_scan) instead of downloading a new one. The cruise no
longer leaves that scan (the thread scans by itself), so here it never applies.
"""
import time

import numpy as np
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b

from spot_explorer.mission import recovery
from spot_explorer.mission.attempt_common import STOP, FAILED
from spot_explorer.perception import global_maps
from spot_explorer.records import HANDOFF, MISSION_STATS
from spot_explorer.robot import connection
from spot_explorer.robot import motion

# The attempt that starts with the robot still walking (hand-off) uses the last scan of the
# cruise, already processed and in the global maps, instead of downloading and processing a
# new one (attempt_scan._first_scan), if it is at most CHAIN_SCAN_MAX_AGE_S old. It serves
# the graph update and the goal choice; the first move still waits for the fresh scan and
# frontier of the first turn. Why: on 04-11 16:52 (CORE I/O) the scan at the attempt start
# with its processing and maps cost ~0.5-1 s of the 4.1-4.7 s hand-offs, on data 0.4 s newer
# than those the cruise had just put in the maps.
CHAIN_REUSE_SCAN = True
CHAIN_SCAN_MAX_AGE_S = 2.0


class StopAndGoStart:
    """_prepare and its scan, for CellAttempt in the stop-and-go mode."""

    def _prepare(self):
        """Before the loop: get up if fallen, first scan, maps and graph, goal point and path,
        attempt counters. FAILED if it cannot even start."""
        self.mobility_kwargs = self.mobility_kwargs or {}

        print(f"\n[ATTEMPT] Trying to enter cell ({self.target_row},{self.target_col}) from the current position...")

        scan = self._first_scan()
        if scan is None:
            return FAILED
        seen = self._cell_already_seen()
        HANDOFF.mark("seen check")
        if seen:
            return STOP
        self._local_data_into_graph(scan)
        outcome = self._goal_and_path(scan)
        if outcome is not None:
            return outcome
        self._reset_counters()
        HANDOFF.mark("attempt counters")

    def _first_scan(self):
        """Get up if fallen, first scan of the attempt, robot pose, global maps updated and
        saved. The processed scan, or None if it does not arrive."""
        # The robot gets here from another move (navigation between cells, retreat): if it
        # fell in the meantime it gets up before anything else. Raises RobotFallenError if it cannot.
        # Then it leaves the hazard disk by retracing its steps: once up, it is still INSIDE
        # the spot of the fall, and from there every PRM edge would be discarded.
        if motion.check_and_recover_fall(self.robot_state_client, self.command_client,
                                   self.global_grid.global_occupancy_map, "at the start of the attempt") is not None:
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)

        reused = self._chained_scan()
        if reused is not None:
            return reused

        # 1. The scan: the layers in one request, then (below) their processing
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        HANDOFF.mark("scan download")
        if scan is None:
            print("[ERROR] Local grid acquisition failed.")
            return None
        main_proto = scan.proto
        pts, cells_obstacle_dist = scan.pts, scan.od
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size

        self._ensure_global_maps(cell_size)

        transforms_snapshot = main_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        self.robot_x, self.robot_y, self.robot_z = vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

        quat = vision_tform_body.rotation
        self.robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))

        # 2. Body footprint, corrected terrain (from here on only terrain_real is used),
        # slope and roughness, obstacle mask
        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-initial")
        HANDOFF.mark("scan processing")
        terrain_real, is_valid = scan.terrain, scan.is_valid
        obstacle_mask = scan.obstacle_mask

        # Update the global maps (occupancy and terrain) with the data just processed
        global_maps.fold_scan_into_global_maps(self.global_map, self.global_terrain_map, pts, terrain_real, obstacle_mask,
                                   is_valid, cells_obstacle_dist, (num_y, num_x),
                                   (self.robot_x, self.robot_y, self.robot_yaw))

        # The terrain map serves the PRM as a fallback for the slope
        self.prm_graph.set_global_terrain_map(self.global_terrain_map)
        HANDOFF.mark("global maps")

        # Global maps on disk (.pkl): at most every MAPS_SAVE_EVERY_S
        _save_global_maps(self.global_grid, self.mission_folder)
        HANDOFF.mark("maps on disk")
        return scan

    def _chained_scan(self):
        """
        CHAIN_REUSE_SCAN: robot still walking after a hand-off and the last scan
        of the cruise at most CHAIN_SCAN_MAX_AGE_S old -> that scan (already processed and in
        the global maps), with the robot pose of NOW (the robot kept walking). None otherwise.
        """
        reuse = getattr(self.env, '_chain_scan', None)
        if (not CHAIN_REUSE_SCAN or reuse is None
                or getattr(self.env, '_moving_pc', None) is None or self.global_grid.global_occupancy_map is None):
            return None
        scan, t_scan = reuse
        age = time.time() - t_scan
        if age > CHAIN_SCAN_MAX_AGE_S:
            return None
        self.global_map = self.global_grid.global_occupancy_map
        self.global_terrain_map = self.global_grid.global_terrain_map
        x, y, z, quat = connection.getPosition(self.robot_state_client)
        self.robot_x, self.robot_y, self.robot_z = x, y, z
        self.robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))
        self.prm_graph.set_global_terrain_map(self.global_terrain_map)
        print(f"[ATTEMPT] Scan of the cruise reused ({age:.1f} s old, already in the maps): no new scan "
              f"at the start of the attempt.")
        MISSION_STATS['chained_scans_reused'] = MISSION_STATS.get('chained_scans_reused', 0) + 1
        HANDOFF.mark("scan of the cruise reused")
        _save_global_maps(self.global_grid, self.mission_folder)
        HANDOFF.mark("maps on disk")
        return scan


def _save_global_maps(global_grid, folder):
    from spot_explorer.mission.attempt_scan import save_global_maps
    save_global_maps(global_grid, folder)
