"""The attempt start, the part of CellAttempt (see cell_attempt.py) that looks at the world:
the scan into the PRM and the edge update, the robot node, the goal in the cell and the first
path (run in the planning thread by attempt_cruise._start_in_thread)."""
import os
import time
import numpy as np
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_a_tform_b
from spot_explorer.graphnav import map_recording
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.planning import edge_evaluation
from spot_explorer.mission import mission_prm
from spot_explorer.records import HANDOFF, timing_add, visualize_grid_with_candidates
from spot_explorer.planning import goal
from spot_explorer.mission.attempt_common import FAILED

class ScanSteps:
    """Mixin of CellAttempt: the state is CellAttempt's, see its docstring for the map of the phases."""

    def _ensure_global_maps(self, cell_size=0.03):
        """GLOBAL MAPS: created at the first attempt (resolution of the local grid, 3 cm), then
        accumulated for the whole mission."""
        self.global_map, self.global_terrain_map = global_maps.ensure_global_maps(self.global_grid, cell_size)
        self.prm_graph.set_global_terrain_map(self.global_terrain_map)

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
        # Local data into the PRM: terrain, slope and roughness of the scan
        # ================================================================== #

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

        HANDOFF.mark("scan into the PRM")
        # The scan just made also goes into the preliminary edge discard
        # (edge_evaluation.LIVE_PREFILTER_OD_M): at build time the global map has not yet
        # confirmed anything about this place, and the edges crossing the obstacles just
        # seen would stay valid. Here obstacle_distance is the raw one: the filtered
        # snapshot is born in the navigation loop.
        self.prm_graph.set_live_obstacles(
            np.asarray(cells_obstacle_dist, dtype=np.float64).reshape((num_y, num_x)),
            np.asarray(valid_values, dtype=np.float64).reshape((num_y, num_x)) >= spot_grid.TERRAIN_VALID_MIN,
            grid_origin_x, grid_origin_y, cell_size, self.robot_x, self.robot_y, self.robot_yaw)

        t_build = time.perf_counter()
        # The candidate pairs are those of the mission graph (see mission_prm.build_mission_prm);
        # vetoes and weights are computed by PRM._evaluate_and_add_edge.
        mission_prm.rebuild_prm_edges(self.prm_graph, self.global_map)
        HANDOFF.mark("graph update")
        timing_add('graph construction', time.perf_counter() - t_build)
        _upd = getattr(getattr(self.prm_graph, 'mission_graph', None), 'last_update', None)
        print(f"[TIMING] {'local update' if _upd and _upd[2] else 'full build'} "
              f"of the graph: {time.perf_counter() - t_build:.2f} s")
        if getattr(self.prm_graph, 'live_veto_last', 0):
            print(f"[EDGES] {self.prm_graph.live_veto_last} edges discarded at once by the live "
                  f"scan (they pass less than {edge_evaluation.LIVE_PREFILTER_OD_M:.2f} m from an "
                  f"obstacle just seen), before the slope.")
        # A new attempt: the edges taken out for the previous attempt (_ATTEMPT_MARK) no longer
        # apply. The penalties are a layer separate from the edges, so they survive the
        # rebuilds WITHIN the attempt.
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

        # Only visited cells + robot cell + goal cell (see mission_prm.planning_cells).
        mission_prm.restrict_planning_to_explored_cells(self.prm_graph, self.env, self.target_row, self.target_col,
                                             (self.robot_x, self.robot_y))
        HANDOFF.mark("allowed cells")

        # --- Start and arrival nodes: Existing graph nodes ------------------------------------
        # add_stop_node attaches the position to the suitable node instead of creating one. The
        # robot node is needed before the goal choice: the choice is among the points the graph
        # connects to the robot (see find_best_point_in_cell, criterion 2).
        self.robot_node_id = self.prm_graph.add_stop_node(
            self.robot_x, self.robot_y, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
            label="attempt start")
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
        HANDOFF.mark("robot node")


        # Search for the target point and planning
        self.target_x, self.target_y, valid_samples, rejected_samples = goal.find_best_point_in_cell(
            self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, pts, cells_obstacle_dist, self.sampler,
            global_map=self.global_map, reachable=self._reachable_from_robot())
        HANDOFF.mark("goal choice")

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
        # be able to go back to if the cell turns out to be a dead end. Robot still walking
        # after a hand-off: the waypoint of the cell just visited is 10 cm
        # behind and the cruise left its breadcrumbs, so only with the usual spacing (each
        # waypoint costs ~0.4 s).
        moving = getattr(self.env, '_moving_pc', None) is not None
        map_recording.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client, self.robot_node_id,
                                 self.target_row, self.target_col, force=not moving)
        HANDOFF.mark("breadcrumb")
        # The goal is the centre or a lattice node (find_best_point_in_cell chooses it among
        # the sampler points).
        self.goal_id = self.prm_graph.add_stop_node(
            self.target_x, self.target_y, self.global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"goal of cell ({self.target_row},{self.target_col})", is_robot=False)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
        path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
        HANDOFF.mark("goal node and path")

        if path_ids is None:
            print(
                f"[NO PATH] No path in the graph from node {self.robot_node_id} to the goal node "
                f"{self.goal_id}: {self.prm_graph.why_no_path(self.robot_node_id, self.goal_id)}.")
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
        HANDOFF.mark("figure data")
        return None

