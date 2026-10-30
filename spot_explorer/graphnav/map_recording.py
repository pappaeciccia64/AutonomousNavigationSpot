"""GraphNav map recording: cell waypoints and breadcrumbs, edges, Spot's loop closure, anchoring, download."""
import os
import time
import numpy as np
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api.graph_nav import graph_nav_pb2, recording_pb2, nav_pb2, map_processing_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient, map_pb2
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from google.protobuf import wrappers_pb2 as wrappers
from spot_explorer import config as P
from spot_explorer.robot import connection
from spot_explorer.graphnav.localization import graphnav_log_localization

class LoopClosureUploadError(Exception):
    """Spot computed the loop-closure edges but uploading them failed."""

    def __init__(self, n_edges, cause):
        super().__init__(f"{n_edges} edges computed but not uploaded into the map "
                         f"({type(cause).__name__}: {cause})")
        self.n_edges = n_edges


class RecordingInterface(object):
    def __init__(self, robot, download_filepath, client_metadata):
        # Base folder: every map download creates a subfolder
        self._base_download_filepath = download_filepath
        self._download_filepath = os.path.join(download_filepath, 'downloaded_graph')

        self._recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
        self._recording_environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(client_metadata=client_metadata)
        )

        self._graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
        self._map_processing_client = robot.ensure_client(MapProcessingServiceClient.default_service_name)
        self.robot = robot

        # Poses of the waypoints we created: {name: {'x', 'y', 'z', 'yaw', 'waypoint_id', 'cell_row', 'cell_col'}}
        self.waypoint_poses = {}

        # Local copy of the graph, valid until someone modifies it
        self._cached_graph = None
        self._graph_cache_valid = False

        # Name and id of the last waypoint created (see create_default_waypoint): avoids
        # downloading the graph again only to re-read them.
        self.last_created_waypoint = None

    def optimize_anchoring(self):
        """
        Optimize the map anchoring directly on the robot's server.
        Corrects the odometric drift and makes the navgraph metrically consistent,
        without using external floor plans.
        """
        print(f"\n[{'=' * 40}]")
        print(f"[ANCHORING] Starting global anchoring optimization...")

        try:
            from google.protobuf import wrappers_pb2 as wrappers
            from bosdyn.api.graph_nav import map_processing_pb2

            # Call the service without any "initial_hint"
            response = self._map_processing_client.process_anchoring(
                params=map_processing_pb2.ProcessAnchoringRequest.Params(
                    # Set to False, the robot discards the current raw anchoring
                    # and generates a new global, optimized one from scratch
                    optimize_existing_anchoring=wrappers.BoolValue(value=False)
                ),
                modify_anchoring_on_server=True,  # Apply the changes to the map in memory
                stream_intermediate_results=False
            )

            print(f"[ANCHORING] ✓ Optimization completed!")
            print(
                f"[ANCHORING] Status: {response.status}, Iterations: {response.iteration}, Final cost: {response.cost:.4f}")
            print(f"[{'=' * 40}]\n")

            # Invalidate the cache to force the download of the updated map
            self.invalidate_graph_cache()
            return True

        except Exception as e:
            print(f"[ANCHORING] ✗ Optimization failed: {e}")
            print(f"[{'=' * 40}]\n")
            return False

    def auto_close_loops(self, close_fiducial_loops, close_odometry_loops, *args):
        """Spot's loop closures on the graph (fiducial and/or odometry), without collision checking."""
        response = self._map_processing_client.process_topology(
            params=map_processing_pb2.ProcessTopologyRequest.Params(
                do_fiducial_loop_closure=wrappers.BoolValue(value=close_fiducial_loops),
                do_odometry_loop_closure=wrappers.BoolValue(value=close_odometry_loops)),
            modify_map_on_server=True)
        print(f'[LOOPS] Created {len(response.new_subgraph.edges)} new edges.')


    def close_loops_checked(self, max_edge_length_m=2.0, robot_radius_m=0.35,
                            height_variation_m=0.25, timeout_s=None, max_path_m=None,
                            min_path_m=None, modify_map_on_server=True, check_collision=True,
                            icp_iters=None, feature_matching=None, upload_result=False,
                            prune_edges=None):
        """
        Spot's loop closures (fiducial + odometry) DURING the mission, with Spot's
        collision check on the new edges, so the graph does not stay a chain and a return
        does not have to retrace the whole way.

        timeout_s: maximum processing time ON SPOT'S SIDE (Params.timeout_seconds). When it
        expires Spot returns the edges found so far (timed_out = True) and a following call
        starts over (bench test of 26-10: it does not resume the work). Without it, Spot
        stops by itself after ~30 s, together with the RPC: on 22-10 the RPC expired first
        and no result arrived. Spot's limit is approximate and after every map change the
        call gets longer (4 s requested -> up to 12 s; 60 s -> over 120 s once): the RPC
        limit here is three times timeout_s, at least + 15 s.
        max_path_m / min_path_m: limits on the distance between the two waypoints along the
        graph (max/min_loop_closure_path_length); None = Spot's values.
        modify_map_on_server, check_collision: False only for bench measurements
        (bench_loop_closure.py): the map stays as it was and different settings are compared.
        icp_iters (0 = no ICP) and feature_matching: without them, Spot finishes in a few
        seconds but the edges are not corrected on the scans; None = Spot's values (with
        ICP). Only for bench measurements, like max_path_m, min_path_m and prune_edges.
        upload_result: Spot computes WITHOUT writing to the map and this function uploads
        the edges (upload_graph, which adds them to the map in one go). Writing by itself,
        Spot can answer MapModifiedError AT THE END of the processing, if the map changed
        in the meantime, and the processing is lost: on 26-10 15:36, right after stopping
        the recording, ~73 s of failed attempts before the good one.
        prune_edges: False = also keeps the candidate edges in areas already connected (by
        default Spot keeps only one per area, the "best"); None = Spot's value.

        Returns (edges created, timed_out). Raises on error: the caller decides what to do.
        """
        odo = map_processing_pb2.ProcessTopologyRequest.OdometryLoopClosureParams(
            max_loop_closure_edge_length=wrappers.DoubleValue(value=float(max_edge_length_m)))
        if prune_edges is not None:
            odo.prune_edges.value = bool(prune_edges)
        if max_path_m is not None:
            odo.max_loop_closure_path_length.value = float(max_path_m)
        if min_path_m is not None:
            odo.min_loop_closure_path_length.value = float(min_path_m)
        col = map_processing_pb2.ProcessTopologyRequest.CollisionCheckingParams(
            check_edges_for_collision=wrappers.BoolValue(value=bool(check_collision)),
            collision_check_robot_radius=wrappers.DoubleValue(value=float(robot_radius_m)),
            collision_check_height_variation=wrappers.DoubleValue(value=float(height_variation_m)))
        params = map_processing_pb2.ProcessTopologyRequest.Params(
            do_fiducial_loop_closure=wrappers.BoolValue(value=True),
            do_odometry_loop_closure=wrappers.BoolValue(value=True),
            odometry_loop_closure_params=odo,
            collision_check_params=col)
        if icp_iters is not None:
            params.icp_params.icp_iters.value = int(icp_iters)
        if feature_matching is not None:
            params.feature_matching_params.do_feature_matching.value = bool(feature_matching)
        kwargs = {}
        if timeout_s is not None:
            params.timeout_seconds = float(timeout_s)
            kwargs['timeout'] = max(3.0 * float(timeout_s), float(timeout_s) + 15.0)
        write_on_server = bool(modify_map_on_server) and not upload_result
        response = self._map_processing_client.process_topology(
            params=params, modify_map_on_server=write_on_server, **kwargs)
        n = len(response.new_subgraph.edges)
        if upload_result and n:
            try:
                self._graph_nav_client.upload_graph(graph=response.new_subgraph,
                                                    generate_new_anchoring=False)
            except Exception as e:
                raise LoopClosureUploadError(n, e) from e
        self.invalidate_graph_cache()
        return n, bool(getattr(response, 'timed_out', False))

    def _get_graph(self, force_refresh=False):
        """The GraphNav graph, from the local copy if valid. force_refresh always downloads it again."""
        if force_refresh or not self._graph_cache_valid or self._cached_graph is None:
            self._cached_graph = self._graph_nav_client.download_graph()
            self._graph_cache_valid = True
        return self._cached_graph

    def invalidate_graph_cache(self):
        """
        Drop the local copy of the graph. Call after every operation that modifies it
        (waypoints, edges, loop closures, recording start).
        """
        self._graph_cache_valid = False
        self._cached_graph = None

    def set_download_filepath(self, filepath):
        """Mission folder where downloaded maps are saved (one subfolder each time)."""
        self._base_download_filepath = filepath
        self._download_filepath = filepath
        os.makedirs(filepath, exist_ok=True)
        print(f"[INFO] Folder to download the graph to: {filepath}")

    def _generate_unique_map_folder(self, base_name='Graph'):
        """
        Path of a new folder for the download, like
        "Graph_10-02-2026_14-30-00" (Grafo_... in missions before 30-10-2026) (day-month-year_hours-minutes-seconds).
        """
        from datetime import datetime

        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        unique_folder = f"{base_name}_{timestamp}"
        full_path = os.path.join(self._base_download_filepath, unique_folder)

        # If it already exists (two downloads in the same second), a counter is added
        counter = 1
        original_path = full_path
        while os.path.exists(full_path):
            full_path = f"{original_path}_{counter}"
            counter += 1

        print(f"[MAP DOWNLOAD] The new map will go to: {unique_folder}")
        return full_path

    def initialize_with_fiducial(self, robot_state_client, fiducial_id=None):
        """
        Localize the robot on the GraphNav map using a visible fiducial. fiducial_id:
        a specific tag; None = the nearest. True if it succeeds.

        The fiducial must ALREADY be in the map: in the mission it is called after creating
        wp_0 with the tag in view, so it is in its snapshot (see find_fiducials).
        """
        import time

        print(f"\n[{'=' * 40}]")
        print(f"[FIDUCIAL INIT] Initialization with the FIDUCIAL...")

        try:
            robot_state = robot_state_client.get_robot_state()
            current_odom_tform_body = get_odom_tform_body(
                robot_state.kinematic_state.transforms_snapshot).to_proto()
        except Exception as e:
            print(f"[FIDUCIAL INIT] ✗ Robot state not readable: {e}")
            return False

        if fiducial_id is not None:
            print(f"[FIDUCIAL INIT] Fiducial searched: ID {fiducial_id}")
            init_type = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_SPECIFIC
        else:
            print(f"[FIDUCIAL INIT] Fiducial searched: the nearest (any ID)")
            init_type = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST

        localization = nav_pb2.Localization()

        try:
            self._graph_nav_client.set_localization(
                initial_guess_localization=localization,
                ko_tform_body=current_odom_tform_body,

                fiducial_init=init_type,
                use_fiducial_id=fiducial_id if fiducial_id else 0,

                refine_fiducial_result_with_icp=True,
                do_ambiguity_check=True
            )

            # A moment for GraphNav to apply the localization
            time.sleep(0.5)
            print(f"[FIDUCIAL INIT] ✓ SUCCEEDED: robot localized on the fiducial, which stays in the map.")
            print(f"[{'=' * 40}]\n")
            return True

        except Exception as e:
            print(f"[FIDUCIAL INIT] ✗ FAILED: {e}")
            print(f"[FIDUCIAL INIT] Possible causes:")
            print(f"  1. Fiducial {fiducial_id if fiducial_id else '(any)'} is not visible")
            print(f"  2. The robot is too far from the fiducial (better within 1-2 m)")
            print(f"  3. The fiducial is not in the snapshot of wp_0 (it was not in view when it was created)")
            print(f"  4. Not enough light")
            print(f"[FIDUCIAL INIT] Carrying on without localization on the fiducial.")
            print(f"[{'=' * 40}]\n")
            return False

    def _get_transform(self, from_wp, to_wp):
        """Transform from from_wp to to_wp, from their poses in the odometry frame."""

        from_se3 = from_wp.waypoint_tform_ko
        from_tf = SE3Pose(
            from_se3.position.x, from_se3.position.y, from_se3.position.z,
            Quat(w=from_se3.rotation.w, x=from_se3.rotation.x, y=from_se3.rotation.y,
                 z=from_se3.rotation.z))

        to_se3 = to_wp.waypoint_tform_ko
        to_tf = SE3Pose(
            to_se3.position.x, to_se3.position.y, to_se3.position.z,
            Quat(w=to_se3.rotation.w, x=to_se3.rotation.x, y=to_se3.rotation.y,
                 z=to_se3.rotation.z))

        from_T_to = from_tf.mult(to_tf.inverse())
        return from_T_to.to_proto()


    def should_we_start_recording(self):
        graph = self._get_graph()
        if graph is not None:
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    return False
        return True

    def get_localization_state(self):
        """
        Where the robot is localized in the graph. Returns a dict:
            is_localized         True if the robot is localized on a waypoint
            waypoint_id          waypoint id (None if not localized)
            waypoint_name        waypoint name (needs the graph: may download it)
            waypoint_tform_body  body pose relative to the waypoint
            localization_state   the full LocalizationState object
            seed_tform_body      pose in the seed frame, if available
        or None on error.
        """
        try:
            localization_state = self._graph_nav_client.get_localization_state()

            # Localized = has a waypoint_id
            is_localized = bool(localization_state.localization.waypoint_id)
            waypoint_id = localization_state.localization.waypoint_id if is_localized else None

            waypoint_name = None
            if is_localized:
                graph = self._get_graph()
                for waypoint in graph.waypoints:
                    if waypoint.id == waypoint_id:
                        waypoint_name = waypoint.annotations.name if waypoint.annotations.name else 'unnamed'
                        break

            result = {
                'is_localized': is_localized,
                'waypoint_id': waypoint_id,
                'waypoint_name': waypoint_name,
                'waypoint_tform_body': localization_state.localization.waypoint_tform_body,
                'localization_state': localization_state,
                'seed_tform_body': localization_state.localization.seed_tform_body if hasattr(localization_state.localization, 'seed_tform_body') else None
            }

            if is_localized:
                print(f"\n[LOCALIZATION STATE] ✓ Robot LOCALIZED")
                print(f"  Waypoint ID: {waypoint_id}")
                print(f"  Waypoint name: {waypoint_name}")

                if localization_state.localization.waypoint_tform_body:
                    pos = localization_state.localization.waypoint_tform_body.position
                    print(f"  Position relative to the waypoint: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            else:
                print(f"\n[LOCALIZATION STATE] ✗ Robot NOT LOCALIZED")
                print(f"  No waypoint_id in the localization state")

            return result

        except Exception as e:
            print(f"\n[LOCALIZATION STATE] ✗ Localization state not readable: {e}")
            return None

    def create_default_waypoint(self, cell_row=None, cell_col=None):
        """
        Create a GraphNav waypoint at the current position, with a progressive name (wp_0,
        wp_1, ...), and save its pose and, if given, the cell in waypoint_poses.
        """
        robot_state_client = self.robot.ensure_client('robot-state')
        x, y, z, quat = connection.getPosition(robot_state_client)

        yaw = np.arctan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2)
        )

        # The next number is derived from the waypoints created by US (waypoint_poses), without
        # downloading the graph again: create_waypoint invalidates the cache at every waypoint, and
        # the graph grows with the mission. We are the only ones creating wp_N waypoints, so the
        # maximum we know is the maximum that exists. If we have not created any yet the graph
        # is checked, since it might come from a previous session.
        def _wp_number(name):
            try:
                return int(name.split('_')[-1])
            except (ValueError, IndexError):
                return -1

        mine = [_wp_number(n) for n in self.waypoint_poses if n.startswith('wp_')]
        if mine:
            next_number = max(mine) + 1
        else:
            graph = self._get_graph()
            if not graph or not graph.waypoints:
                next_number = 0
            else:
                max_number = -1
                for wp in graph.waypoints:
                    name = wp.annotations.name
                    if name and name.startswith('wp_'):
                        n = _wp_number(name)
                        if n >= 0:
                            max_number = max(max_number, n)
                next_number = max_number + 1

        new_name = f'wp_{next_number}'
        resp = self._recording_client.create_waypoint(waypoint_name=new_name)

        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            created_waypoint_id = resp.created_waypoint.id if hasattr(resp, 'created_waypoint') else 'ID_not_available'

            self.waypoint_poses[new_name] = {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                't': time.time(),        # for the drift measurement (graphnav_log_drift)
                'waypoint_id': created_waypoint_id,
                'cell_row': cell_row,
                'cell_col': cell_col
            }

            print(f"[WAYPOINT] {new_name} created at ({x:.2f}, {y:.2f}), yaw {np.degrees(yaw):.0f} deg"
                  + (f", cell ({cell_row},{cell_col})" if cell_row is not None and cell_col is not None else "")
                  + ".")

            # Name and id of the last waypoint, from the robot's answer: whoever wants them does not
            # have to go through get_localization_state, which would download again the graph just
            # invalidated only for the label.
            self.last_created_waypoint = {'name': new_name, 'id': created_waypoint_id}

            self.invalidate_graph_cache()

            return resp
        else:
            print(f"[WAYPOINT] {new_name} NOT created (status {resp.status}).")
            return False

    def clear_map(self, *args):
        return self._graph_nav_client.clear_graph()

    def start_recording(self, *args):
        should_start_recording = self.should_we_start_recording()
        if not should_start_recording:
            print(
                'GraphNav is not in the right state to record: '
                'clear the map (clear_map) or localize the robot on the map '
                '(for example with graph_nav_command_line).'
            )
            return
        try:
            status = self._recording_client.start_recording(recording_environment=self._recording_environment)
            self.invalidate_graph_cache()  # the recording will add waypoints
            print('[RECORDING] Map recording started.')
        except Exception as err:
            print(f'[RECORDING] Starting the recording failed: {err}')

    def stop_recording(self, *args):
        first_iter = True
        while True:
            try:
                status = self._recording_client.stop_recording()
                print('[RECORDING] Map recording stopped.')
                break
            except bosdyn.client.recording.NotReadyYetError as err:
                if first_iter:
                    print('[RECORDING] Waiting for the map to finish processing...')
                first_iter = False
                time.sleep(0.5)
                continue
            except Exception as err:
                print(f'[RECORDING] Stopping the recording failed: {err}')
                break


    def download_full_graph(self, *args):
        """
        Download the full graph, with the waypoint and edge snapshots, into a new folder
        (see _generate_unique_map_folder). Returns the path, None if it fails.
        """
        graph = self._get_graph(force_refresh=True)
        if graph is None:
            print('[MAP DOWNLOAD] ✗ Graph not downloaded.')
            return None

        unique_path = self._generate_unique_map_folder()

        # The _download_and_write_* methods write into self._download_filepath: it is changed for now
        old_path = self._download_filepath
        self._download_filepath = unique_path

        try:
            print(f'\n[MAP DOWNLOAD] Downloading...')
            print(f'[MAP DOWNLOAD] Destination: {os.path.basename(unique_path)}')

            self._write_full_graph(graph)
            print(f'[MAP DOWNLOAD] Graph downloaded: {len(graph.waypoints)} waypoints and {len(graph.edges)} edges')

            self._download_and_write_waypoint_snapshots(graph.waypoints)
            self._download_and_write_edge_snapshots(graph.edges)

            print(f'[MAP DOWNLOAD] ✓ Map saved in: {unique_path}')
            return unique_path

        finally:
            self._download_filepath = old_path

    def _write_full_graph(self, graph):
        graph_bytes = graph.SerializeToString()
        self._write_bytes(self._download_filepath, 'graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints):
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(waypoint.snapshot_id)
            except Exception:
                print(f'[MAP DOWNLOAD] Waypoint snapshot not downloaded: {waypoint.snapshot_id}')
                continue
            self._write_bytes(
                os.path.join(self._download_filepath, 'waypoint_snapshots'),
                str(waypoint.snapshot_id),
                waypoint_snapshot.SerializeToString(),
            )
            num_waypoint_snapshots_downloaded += 1
        print(f'[MAP DOWNLOAD] Waypoint snapshots: {num_waypoint_snapshots_downloaded} of {len(waypoints)}.')

    def _download_and_write_edge_snapshots(self, edges):
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                print(f'[MAP DOWNLOAD] Edge snapshot not downloaded: {edge.snapshot_id}')
                continue
            self._write_bytes(
                os.path.join(self._download_filepath, 'edge_snapshots'),
                str(edge.snapshot_id),
                edge_snapshot.SerializeToString(),
            )
            num_edge_snapshots_downloaded += 1
        print(f'[MAP DOWNLOAD] Edge snapshots: {num_edge_snapshots_downloaded} of {num_to_download}.')

    def _write_bytes(self, filepath, filename, data):
        os.makedirs(filepath, exist_ok=True)
        with open(os.path.join(filepath, filename), 'wb+') as f:
            f.write(data)
            f.close()

    def get_all_manual_waypoints_with_cells(self):
        """
        The waypoints created by us (wp_N) that have a cell: {(row, column): data}, with
        data = name, id, x, y, z, yaw, cell_row, cell_col.
        """
        waypoints_by_cell = {}

        for wp_name, wp_data in self.waypoint_poses.items():
            if not (wp_name.startswith('wp_') and len(wp_name.split('_')) == 2):
                continue

            if wp_data.get('cell_row') is not None and wp_data.get('cell_col') is not None:
                cell_key = (wp_data['cell_row'], wp_data['cell_col'])
                waypoints_by_cell[cell_key] = {
                    'name': wp_name,
                    'id': wp_data['waypoint_id'],
                    'x': wp_data['x'],
                    'y': wp_data['y'],
                    'z': wp_data['z'],
                    'yaw': wp_data['yaw'],
                    'cell_row': wp_data['cell_row'],
                    'cell_col': wp_data['cell_col']
                }

        print(f"[WAYPOINTS] {len(waypoints_by_cell)} manual waypoints with their cell")
        return waypoints_by_cell

    def get_manual_waypoint_by_cell(self, cell_row, cell_col):
        """The data of the waypoint created in the cell, None if there is none."""
        waypoints_by_cell = self.get_all_manual_waypoints_with_cells()
        return waypoints_by_cell.get((cell_row, cell_col))

    def create_edge_between_waypoint_objs(self, from_wp, to_wp):
        """
        Create an edge between two waypoints already read from the graph (map_pb2.Waypoint
        objects), without downloading the graph again or printing the transform. The caller
        invalidates the cache at the end (invalidate_graph_cache). Raises on error.
        """
        new_edge = map_pb2.Edge()
        new_edge.id.from_waypoint = from_wp.id
        new_edge.id.to_waypoint = to_wp.id
        new_edge.from_tform_to.CopyFrom(self._get_transform(from_wp, to_wp))
        self._recording_client.create_edge(edge=new_edge)

    def find_nearest_waypoint_cell_to_target(self, target_cell, waypoints_by_cell, env_map):
        """
        The visited cell with a waypoint nearest to target_cell: an adjacent one first,
        otherwise the nearest in Manhattan distance. None if there are none.
        waypoints_by_cell as from get_all_manual_waypoints_with_cells.
        """
        target_row, target_col = target_cell

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        adjacent_with_waypoint = []
        for dr, dc in directions:
            adj_row, adj_col = target_row + dr, target_col + dc
            adj_cell = (adj_row, adj_col)

            if (0 <= adj_row < env_map.rows and
                0 <= adj_col < env_map.cols and
                adj_cell in waypoints_by_cell and
                env_map.map[adj_row][adj_col] == 1):  # visited
                adjacent_with_waypoint.append(adj_cell)

        if adjacent_with_waypoint:
            print(f"[NEAREST_WP] Waypoint in a neighbouring cell: {adjacent_with_waypoint[0]}")
            return adjacent_with_waypoint[0]

        # No adjacent one: the nearest among all
        min_distance = float('inf')
        nearest_cell = None

        for cell in waypoints_by_cell.keys():
            cell_row, cell_col = cell

            if env_map.map[cell_row][cell_col] != 1:
                continue

            distance = abs(cell_row - target_row) + abs(cell_col - target_col)

            if distance < min_distance:
                min_distance = distance
                nearest_cell = cell

        if nearest_cell:
            print(f"[NEAREST_WP] Nearest waypoint in cell {nearest_cell} (distance: {min_distance})")

        return nearest_cell


def graphnav_is_recording(recordingInterface):
    """True/False from the GraphNav recording, None if it cannot be known."""
    try:
        return bool(recordingInterface._recording_client.get_record_status().is_recording)
    except Exception:
        return None


def graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client, prm_node_id,
                             cell_row, cell_col, force=False):
    """
    Drop a breadcrumb: a GraphNav waypoint at the point where the robot is, linked to the
    PRM node it is passing through.

    Used for returns. Walking BACKWARDS over the recorded legs
    (retreat_along_traveled_path) the robot cannot see where it puts its feet, does not
    relocalize, and an error accumulates. With a trail of waypoints the return is done by
    GraphNav, which walks forwards, with Boston Dynamics' localization and obstacle
    avoidance (user's decision).

    Density: a waypoint only if the robot is at least GRAPHNAV_WAYPOINT_MIN_SPACING_M from
    the last one dropped, plus always one on entering a cell (force=True). A waypoint for
    EVERY stop position would give a map that is too dense: partial steps leave stops
    20-60 cm apart.

    The trail lives on env (_graphnav_trail) because it must survive the cell attempts.
    Never raises: a missed breadcrumb is a breadcrumb, not a reason to stop the robot.
    """
    if recordingInterface is None:
        return None
    trail = getattr(env, '_graphnav_trail', None)
    if trail is None:
        trail = env._graphnav_trail = []
    try:
        x, y, _, _ = connection.getPosition(robot_state_client)
    except Exception as e:
        print(f"[BREADCRUMB] Position not readable: {e}")
        return None
    if not force and trail:
        last = trail[-1]
        if float(np.hypot(x - last['x'], y - last['y'])) < P.GRAPHNAV_WAYPOINT_MIN_SPACING_M:
            return None
    try:
        recordingInterface.create_default_waypoint(cell_row=cell_row, cell_col=cell_col)
        # Name and id from the waypoint just created (the robot's answer carries them), not from
        # get_localization_state(), which downloads the whole graph again only to read them.
        # Falls back on get_localization_state() if the interface does not expose them.
        last = getattr(recordingInterface, 'last_created_waypoint', None)
        if isinstance(last, dict) and last.get('id'):
            wp_id, wp_name = last['id'], last.get('name')
        else:
            loc = recordingInterface.get_localization_state()
            wp_id = loc.get('waypoint_id') if isinstance(loc, dict) else None
            wp_name = loc.get('waypoint_name') if isinstance(loc, dict) else None
    except Exception as e:
        print(f"[BREADCRUMB] Waypoint not created: {e}")
        return None
    if not wp_id:
        print("[BREADCRUMB] Waypoint created but not localized: I do not put it in the trail "
              "(a waypoint without an id cannot be reached).")
        return None
    crumb = {'wp_id': wp_id, 'wp_name': wp_name, 'x': float(x), 'y': float(y),
             'prm_node': prm_node_id, 'cell': (cell_row, cell_col)}
    trail.append(crumb)
    print(f"[BREADCRUMB] Waypoint {wp_name} at ({x:.2f}, {y:.2f}), PRM node {prm_node_id} "
          f"(trail: {len(trail)} waypoints).")
    # GraphNav localizer state, printed only when it changes.
    graphnav_log_localization(env, recordingInterface, robot_state_client, f"breadcrumb {wp_name}")
    return crumb
