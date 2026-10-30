"""Mission start-up and shutdown, emergency shutdown, command line (see run_mission.py)."""
import os
import sys
import time
from time import sleep
from datetime import datetime
import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, blocking_stand)
from types import SimpleNamespace
from spot_explorer.graphnav import apriltags
from spot_explorer.graphnav import map_recording
from spot_explorer.graphnav import map_shortcuts
from spot_explorer.graphnav import navigation
from spot_explorer.robot import motion as movements
from spot_explorer.perception import gait
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import slopes
from spot_explorer.perception import spot_grid
from spot_explorer.robot import connection
from spot_explorer.mission import cells as environmentMap
from spot_explorer.planning import safe_frontier
from spot_explorer.robot import battery

from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.mission import mission_prm
from spot_explorer.planning import graph_nodes

from spot_explorer import config as P
from spot_explorer import REPO_ROOT
from spot_explorer.records import (MISSION_STATS, MISSION_CTX, SegmentLog, install_stdout_tee, remove_stdout_tee, set_mission_z0)
from spot_explorer.mission import attempt_common
from spot_explorer.mission import exploration
from spot_explorer.mission import summary

# Where MissionMap/, MissionLogs/ and graph/ are created (the tests move it to a temporary folder).
DATA_ROOT = REPO_ROOT


def _emergency_shutdown(robot, command_client, recordingInterface, exc):
    """
    Mission interrupted (exception or Ctrl+C) with the robot powered and the lease still
    held: stop it, sit it down and power off the motors, then download the GraphNav map,
    which lives only on the robot and would be erased at the start of the next mission.
    Before, the program just exited: robot standing until the software E-Stop, no longer
    renewed, cut the power (~9 s), and map lost. If the robot has fallen it is not sat down
    (it is already on the ground). Each step is tried on its own: never raises from here.
    """
    fallen = isinstance(exc, movements.RobotFallenError)
    print(f"\n[EMERGENCY] Mission interrupted ({type(exc).__name__}): "
          + ("the robot is on the ground, " if fallen else "I stop the robot, sit it down, ")
          + "I save the GraphNav map.")
    tracker = MISSION_CTX.get('tracker')
    if tracker is not None:
        try:
            tracker.clear_path()
        except Exception:
            pass
    if not fallen:
        try:
            movements.stop_robot(command_client, reason="mission interrupted")
            command_client.robot_command(RobotCommandBuilder.synchro_sit_command(),
                                         end_time_secs=time.time() + 20)
            sleep(3)
            robot.power_off(cut_immediately=False)
            print("[EMERGENCY] Robot sitting and motors off.")
        except BaseException as e:
            print(f"[EMERGENCY] I could not sit the robot down ({e!r}): use the tablet.")
    try:
        recordingInterface.stop_recording()
    except BaseException:
        pass
    try:
        recordingInterface.download_full_graph()
        print("[EMERGENCY] GraphNav map downloaded.")
    except BaseException as e:
        print(f"[EMERGENCY] GraphNav map not downloaded ({e!r}).")


def _mission_folders():
    """Mission folders (graph/, MissionMap/, MissionLogs/ with the date): (graph, data and
    images, logs)."""
    mission_timestamp = datetime.now().strftime("Mission_%d-%m-%Y_%H-%M-%S")
    base = DATA_ROOT
    graph_folder = os.path.join(base, "graph", mission_timestamp)
    mission_map_folder = os.path.join(base, "MissionMap", mission_timestamp)
    mission_log_folder = os.path.join(base, "MissionLogs", mission_timestamp)
    os.makedirs(graph_folder, exist_ok=True)
    os.makedirs(mission_map_folder, exist_ok=True)
    os.makedirs(mission_log_folder, exist_ok=True)
    return graph_folder, mission_map_folder, mission_log_folder


def _print_configuration():
    """Switches and thresholds at the top of the log, so it can be read knowing which values
    produced it."""
    print("[CONFIG] SWITCHES: " + "  ".join(
        f"{k.split('.')[-1]}={v}" for k, v in summary.mission_switches().items()))
    print(f"[CONFIG] SLOPE_THRESHOLD={slopes.SLOPE_THRESHOLD}  "
          f"LATERAL_SLOPE_COST_MULTIPLIER={slopes.LATERAL_SLOPE_COST_MULTIPLIER}  "
          f"OBSTACLE_THRESHOLD={geometry.OBSTACLE_THRESHOLD}  ROUGH_THRESHOLD={spot_grid.ROUGH_THRESHOLD}  "
          f"NEAR_THRESHOLD_MARGIN_FRACTION={slopes.NEAR_THRESHOLD_MARGIN_FRACTION}")
    print(f"[CONFIG] EDGE PROFILE: base={slopes.SLOPE_BASELINE_M}m  "
          f"lateral_slice=+/-{slopes.LATERAL_SLICE_HALF_WIDTH_M}m  "
          f"min_coverage={slopes.MIN_ARC_SAMPLE_FRACTION:.0%}")
    print(f"[CONFIG] GAIT: plain<{gait.GAIT_PLAIN_SLOPE_MAX}/{gait.GAIT_PLAIN_ROUGH_MAX}  "
          f"moderate<{gait.GAIT_MODERATE_SLOPE_MAX}/{gait.GAIT_MODERATE_ROUGH_MAX}  "
          f"mu={gait.GAIT_MU_PLAIN}/{gait.GAIT_MU_MODERATE}/{gait.GAIT_MU_HARD}")
    print(f"[CONFIG] TERRAIN: terrain_valid >= {spot_grid.TERRAIN_VALID_MIN} "
          f"(was > 0.0, depended on the sign of a numerical residue)  "
          f"cells never written: plateau >= {spot_grid.UNWRITTEN_MIN_CELLS} cells, "
          f"at least {spot_grid.UNWRITTEN_MIN_SPOT_INVALID:.0%} marked invalid by Spot")
    print(f"[CONFIG] ROTATION: prefilter {geometry.ROTATION_CLEARANCE_M:.3f} m "
          f"(air {geometry.ROTATION_CLEARANCE_AIR_M:.2f} m, separate from the "
          f"{geometry.ROBOT_CLEARANCE_AIR_M:.2f} m of the advance), then check on the "
          f"swept sector in steps of {geometry.ROTATION_SWEEP_STEP_DEG:.0f} deg "
          f"(tolerance {geometry.ROTATION_NOISE_TOLERANCE_M:.2f} m on the noise); "
          f"SIDEWAYS WALKING DISABLED")
    print(f"[CONFIG] IF IT CANNOT ROTATE: 1) fan in the cone +/-{P.CONE_SWEEP_MAX_DEG:.0f} deg "
          f"in steps of {P.CONE_SWEEP_STEP_DEG:.0f}, it goes in the freest direction among those "
          f"getting closer by at least {P.CONE_MIN_GAIN_M:.2f} m; 2) straight back-off up to "
          f"{P.ROTATION_RETREAT_M:.2f} m ONLY if from there the rotation becomes possible, "
          f"max {P.MAX_STRAIGHT_RETREATS_IN_A_ROW} in a row; 3) the leg is discarded")
    print(f"[CONFIG] TRAP: 360 degree fan in steps of {P.ESCAPE_FAN_STEP_DEG:.0f} "
          f"({int(360/P.ESCAPE_FAN_STEP_DEG)} directions); a direction counts if it allows "
          f"{P.ESCAPE_FAN_MIN_ADVANCE_M:.2f} m AND the robot can turn to point at it. Zero "
          f"directions -> it retraces the leg it came from AT ONCE, without "
          f"discarding edges. Check on the arrival pose for steps below "
          f"{P.ARRIVAL_EXIT_CHECK_MAX_STEP_M:.2f} m or with obstacle_distance below "
          f"{P.ARRIVAL_EXIT_CHECK_OD_M:.2f} m, max {P.MAX_ARRIVAL_REFUSALS} refusals. "
          f"Discard by [EARLY] and [ARRIVAL]: NOT permanent")
    print(f"[CONFIG] RETURNS: GraphNav breadcrumb every "
          f"{P.GRAPHNAV_WAYPOINT_MIN_SPACING_M:.2f} m, return with GraphNav up to "
          f"{P.GRAPHNAV_MAX_BACKTRACK_HOPS} waypoints back retrying the "
          f"planning at every stop, then the cell is deferred; walked retreat "
          f"only as the last net. PRM nodes discarded below "
          f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m of free space, only where seen. "
          f"Exemption of {prm_graph.STOP_NODE_MAP_IGNORE_M:.2f} m from the global map "
          f"around the robot: now only after a fall")
    print(f"[CONFIG] OBSTACLES: median over {safe_frontier.OBSTACLE_MEDIAN_FRAMES} scans "
          f"+ hysteresis {safe_frontier.OBSTACLE_HYSTERESIS_M:.2f} m around "
          f"{geometry.FRONTIER_CLEARANCE_M:.2f} m (measured noise 1.5 cm typical, 7.3 peak); "
          f"obstacle cells excluded from the edge slope sampling")
    print(f"[CONFIG] BATTERY: return to wp_0 below {P.BATTERY_RETURN_PCT:.0f}% "
          f"({P.BATTERY_RETURN_CONFIRM} readings in a row; 0 = off), reading at most every "
          f"{P.BATTERY_SAMPLE_S:.0f} s, data in battery.csv")
    print(f"[CONFIG] GOAL: arrival tolerance {P.GOAL_REACHED_TOLERANCE_M:.2f} m "
          f"(= half body + frontier margin), point chosen again up to "
          f"{P.MAX_BLOCK_REPLANS} times if it turns out unreachable; replan at the "
          f"FIRST blocking cell, up to {P.MAX_EARLY_REPLANS} times per attempt")
    print(f"[CONFIG] GOAL CHOICE: only points reachable from the robot in the graph (if there "
          f"are any), preferred free space {P.TARGET_CLEARANCE_M:.2f} m (minimum "
          f"{geometry.PLAN_CLEARANCE_M:.2f}), at least {P.TARGET_CELL_BORDER_M:.2f} m from the border "
          f"of the cell, goals refused by the frontier excluded for the mission; arrival "
          f"valid even with the robot outside the cell up to {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m")


def _mission_grid(options):
    """The mission grid: EnvironmentMap with columns ahead and rows to the left.

    Default P.MISSION_GRID_ROWS x P.MISSION_GRID_COLS cells of P.MISSION_CELL_SIZE_M. They
    can be changed WITHOUT touching the code, in order of precedence:
      1. arguments:  python3 run_mission.py --rows 2 --cols 4 --cell 5
      2. variable:   SPOT_MISSION_GRID="rows,cols,cell_side"   e.g. "2,3,3"
    """
    grid_rows, grid_cols, grid_cell = P.MISSION_GRID_ROWS, P.MISSION_GRID_COLS, P.MISSION_CELL_SIZE_M
    _grid_source = "mission_graph.MISSION_*"
    _env_grid = os.environ.get("SPOT_MISSION_GRID", "").strip()
    if _env_grid:
        try:
            _r, _c, _cs = _env_grid.split(",")
            grid_rows, grid_cols, grid_cell = int(_r), int(_c), float(_cs)
            _grid_source = f"SPOT_MISSION_GRID='{_env_grid}'"
        except Exception:
            print(f"[CONFIG] SPOT_MISSION_GRID='{_env_grid}' not valid (expected 'rows,cols,side'): "
                  f"using {grid_rows},{grid_cols},{grid_cell:g}.")
    _cli = (getattr(options, 'grid_rows', None), getattr(options, 'grid_cols', None),
            getattr(options, 'grid_cell', None))
    if any(v is not None for v in _cli):
        grid_rows = int(_cli[0]) if _cli[0] is not None else grid_rows
        grid_cols = int(_cli[1]) if _cli[1] is not None else grid_cols
        grid_cell = float(_cli[2]) if _cli[2] is not None else grid_cell
        _grid_source = "arguments --rows/--cols/--cell"
    print(f"[CONFIG] Mission grid: {grid_rows} x {grid_cols} cells of {grid_cell:.1f} m "
          f"({grid_rows * grid_cell:.0f} m to the left x {grid_cols * grid_cell:.0f} m ahead; "
          f"source: {_grid_source})")
    print(f"[CONFIG] Lattice: requested spacing "
          f"{'suggested' if P.MISSION_LATTICE_SPACING_M is None else '%.3f m' % P.MISSION_LATTICE_SPACING_M}, "
          f"rings {P.MISSION_LATTICE_RINGS if P.MISSION_LATTICE_RINGS is not None else 'all'}")
    # test_replay reads method and edges from this line; distribution, density and seed from
    # the [GRAPH] Random nodes line, written when the graph is built.
    print(f"[CONFIG] Graph: nodes {P.MISSION_GRAPH_SAMPLING}"
          + (f" ({P.MISSION_RANDOM_DISTRIBUTION}, {P.MISSION_RANDOM_DENSITY:g} nodes/m2, seed "
             f"{'random' if P.MISSION_RANDOM_SEED is None else P.MISSION_RANDOM_SEED})"
             if P.MISSION_GRAPH_SAMPLING == 'random' else "")
          + f"; edges from {P.PRM_MIN_EDGE_M:g} to {P.PRM_MAX_EDGE_M:g} m")
    return environmentMap.EnvironmentMap(rows=grid_rows, cols=grid_cols, cell_size=grid_cell)


def _close_mission(robot, robot_state_client, command_client, recordingInterface):
    """After the summary: GraphNav map closure, return to wp_0, sit, power off, map
    download. Not counted in any [TIMING] entry: it is measured separately and printed
    at closure."""
    _t_close = {}
    _t = time.perf_counter()
    try:
        _wp0_id = next((w.id for w in recordingInterface._get_graph(force_refresh=True).waypoints
                        if w.annotations.name == "wp_0"), None)
        _xy_end = connection.getPosition(robot_state_client)[:2]
    except Exception:
        _wp0_id, _xy_end = None, None
    map_shortcuts.graphnav_add_shortcuts(recordingInterface, "before the return to wp_0",
                                         map_edges=True, target_wp_id=_wp0_id, robot_xy=_xy_end)
    recordingInterface.auto_close_loops(True, False)
    recordingInterface.stop_recording()
    # Spot's loop closure only once, BEFORE anchoring: its edges, verified on the scans,
    # correct the drift of the downloaded map and shorten the return
    # (see GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
    map_shortcuts.graphnav_add_shortcuts(recordingInterface, "map closure", map_edges=False,
                                         loop_closure_s=P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S)
    recordingInterface.optimize_anchoring()
    _t_close['map closure and anchoring'] = time.perf_counter() - _t

    _t = time.perf_counter()
    monitor = MISSION_CTX.get('battery')
    if monitor is not None:
        monitor.check("start of the return to wp_0")
    _pct0 = MISSION_STATS.get('battery_last_pct')
    wp0_outcome = navigation.graphnav_return_to_start(recordingInterface, robot_state_client, command_client)
    if wp0_outcome in ('arrived', 'near'):
        # Even when stopped near wp_0 the tag is searched: it measures the drift of the whole mission.
        measure = apriltags.graphnav_check_start_fiducial(robot, robot_state_client)
        # Arrived according to GraphNav but far according to the tag: correction (see
        # WP0_FIDUCIAL_CORRECTION). Not when stopped nearby (STUCK): the way is blocked.
        if wp0_outcome == 'arrived' and measure and measure.get('offset') is not None:
            apriltags.graphnav_correct_wp0_with_fiducial(robot, recordingInterface, robot_state_client,
                                                             command_client, measure['offset'])
    if wp0_outcome != 'arrived':
        print("[GRAPHNAV] Return to wp_0 failed: the robot sits down where it is.")
    _t_close['return to wp_0'] = time.perf_counter() - _t
    if monitor is not None:
        monitor.check("end of the return to wp_0")
        _pct1 = MISSION_STATS.get('battery_last_pct')
        if _pct0 is not None and _pct1 is not None:
            # What the return costs: used to tune BATTERY_RETURN_PCT.
            MISSION_STATS['battery_wp0_return_pct'] = round(_pct0 - _pct1, 1)
            print(f"[BATTERY] Return to wp_0: from {_pct0:.0f}% to {_pct1:.0f}% "
                  f"({_pct0 - _pct1:.1f} points in {_t_close['return to wp_0']:.0f} s)")

    _t = time.perf_counter()
    command_client.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
    sleep(3)
    robot.power_off(cut_immediately=False)
    _t_close['sit and power off'] = time.perf_counter() - _t

    _t = time.perf_counter()
    recordingInterface.download_full_graph()
    _t_close['map download'] = time.perf_counter() - _t

    print(f"[TIMING] Mission closure, {sum(_t_close.values()):.1f} s in all:")
    for k, v in sorted(_t_close.items(), key=lambda kv: -kv[1]):
        print(f"  {k:32s} {v:6.1f} s")


def easy_walk(options):
    """The whole mission: robot and GraphNav start-up, graph, cell-by-cell exploration
    (Mission), final scan, summary, return to the start and power-off."""
    robot, lease_client, robot_state_client, client_metadata = connection.setLogInfo(options)
    estop = connection.SimpleEstop(robot, options.name + "_estop")

    local_grid = spot_grid.LocalGrid(robot)
    MISSION_CTX['local_grid'] = local_grid
    global_grid = global_maps.GlobalGrid()
    MISSION_CTX['global_grid'] = global_grid   # for graphnav_add_shortcuts

    recordingInterface = map_recording.RecordingInterface(robot, options.download_filepath, client_metadata)
    recordingInterface.stop_recording()
    recordingInterface.clear_map()

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Folders and log created at once, so the copy of the output to file (mission_log.txt)
        # also captures the whole start-up and graph construction.
        graph_folder, mission_folder, mission_log_folder = _mission_folders()
        install_stdout_tee(os.path.join(mission_log_folder, "mission_log.txt"))
        try:
            MISSION_CTX['log_folder'] = mission_log_folder
            MISSION_CTX['segment_log'] = SegmentLog(os.path.join(mission_log_folder, "segments.csv"))
            MISSION_CTX['summary_done'] = False
            print(f"[LOG] Log folder: {mission_log_folder}  |  data and images: {mission_folder}")
            _print_configuration()
        except Exception as e:
            print(f"[LOG] Could not initialize the segment CSV: {e}")

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.logger.info('Powering on the robot...')
        robot.power_on()
        assert robot.is_powered_on(), 'Robot power-on failed.'
        robot.logger.info('Robot powered on.')
        try:
            blocking_stand(command_client)

            # Battery: first reading, then at every loop turn (see battery.py).
            MISSION_CTX['battery'] = battery.BatteryMonitor(robot_state_client)
            MISSION_CTX['battery'].check("start")

            # Grids and local maps all come from Spot's cameras (no Velodyne).
            recordingInterface.clear_map()
            recordingInterface.start_recording()

            # Starting fiducial (see START_FIDUCIAL_ID): searched BEFORE creating wp_0, so that
            # it is in its snapshot and localization on the fiducial has something to find.
            fiducials = apriltags.find_fiducials(robot, robot_state_client,
                                                     wait_s=P.START_FIDUCIAL_WAIT_S,
                                                     fiducial_id=P.START_FIDUCIAL_ID)
            MISSION_CTX['start_fiducial'] = fiducials[0] if fiducials else None
            if fiducials:
                print(f"[FIDUCIAL] Starting fiducial: {apriltags.describe_fiducial(fiducials[0])}"
                      + (f"; I see {len(fiducials) - 1} more: "
                         + ", ".join(str(f['id']) for f in fiducials[1:]) if len(fiducials) > 1 else ""))
            else:
                print(f"[FIDUCIAL] No fiducial "
                      f"{'' if P.START_FIDUCIAL_ID is None else f'with ID {P.START_FIDUCIAL_ID} '}"
                      f"seen in {P.START_FIDUCIAL_WAIT_S:.0f} s by any camera: starting without. "
                      f"Spot only recognizes AprilTag 36h11 (the Boston Dynamics fiducials).")

            start_row, start_col = 0, 0
            recordingInterface.create_default_waypoint(cell_row=start_row, cell_col=start_col)
            if fiducials:
                recordingInterface.initialize_with_fiducial(robot_state_client, fiducials[0]['id'])

            env = _mission_grid(options)

            # --- Gait for the whole mission ---
            # Empty: gait, step height and ground_mu_hint are chosen LEG BY LEG based on the
            # terrain (see motion.select_mobility_params). To force a fixed profile
            # (for example in a test) a non-empty dict here is enough: CellAttempt uses it for all
            # legs and skips the automatic choice. Example:
            #   mobility_kwargs = dict(
            #       locomotion_hint=spot_command_pb2.HINT_CRAWL,
            #       ground_mu_hint=0.4,
            #       swing_height=spot_command_pb2.SWING_HEIGHT_HIGH
            #   )
            mobility_kwargs = {}

            # --- Starting pose, BEFORE building the graph: the lattice is relative to it ---
            x_boot, y_boot, z_boot, quat_boot = connection.getPosition(robot_state_client)
            yaw_boot = np.arctan2(2.0 * (quat_boot.w * quat_boot.z + quat_boot.x * quat_boot.y),
                                  1.0 - 2.0 * (quat_boot.y ** 2 + quat_boot.z ** 2))
            env.set_origin(x_boot, y_boot, yaw_boot, start_row=start_row, start_col=start_col)
            set_mission_z0(robot_state_client, z_boot)

            # PRM nodes: the mission graph (see mission_prm.build_mission_prm). The power-on
            # point and the cell centres are graph nodes, with both methods. The EDGES are not
            # created here: there is no terrain or occupancy data yet. CellAttempt creates them
            # at the first scan (see mission_prm.rebuild_prm_edges).
            prm, gb_sampler, mission_lattice = mission_prm.build_mission_prm(env)
            MISSION_CTX['prm'] = prm  # for the end-of-mission summary (even if interrupted)

            verification_tracker = safe_frontier.ArcVerificationTracker(robot)
            MISSION_CTX['tracker'] = verification_tracker
            verification_tracker.start()

            recordingInterface.set_download_filepath(graph_folder)

            mission = exploration.Mission(robot_state_client, command_client, recordingInterface, local_grid,
                                global_grid, env, prm, gb_sampler, verification_tracker,
                                mission_folder, mobility_kwargs)
            mission.explore()

            env.print_map()
            if attempt_common.battery_low():
                print("[BATTERY] Mission closed before finishing the exploration: the closure is "
                      "the normal one (final scan, summary, return to wp_0, map downloaded).")
            x_final, y_final, _, _ = connection.getPosition(robot_state_client)
            final_row, final_col = env.get_cell_from_world(x_final, y_final)
            recordingInterface.create_default_waypoint(cell_row=final_row, cell_col=final_col)

            mission.final_scan()

            # MISSION SUMMARY -- diagnostics only, to be read after the run.
            summary._emit_mission_summary_once()

            _close_mission(robot, robot_state_client, command_client, recordingInterface)
            estop.stop()
        except BaseException as exc:
            _emergency_shutdown(robot, command_client, recordingInterface, exc)
            raise


# =========================================================================
# COMMAND LINE
# =========================================================================


def _parse_args(argv=None):
    """
    Command-line arguments. All optional: without arguments the mission uses the values
    of config.py (MISSION_GRID_ROWS, MISSION_GRID_COLS, MISSION_CELL_SIZE_M,
    MISSION_LATTICE_SPACING_M, MISSION_LATTICE_RINGS, MISSION_GRAPH_SAMPLING...).
    """
    import argparse
    p = argparse.ArgumentParser(description="Autonomous exploration mission with Spot")
    p.add_argument('--rows', type=int, default=None,
                   help=f"rows of cells, to the robot's left (default {P.MISSION_GRID_ROWS})")
    p.add_argument('--cols', type=int, default=None,
                   help=f"columns of cells, ahead of the robot (default {P.MISSION_GRID_COLS})")
    p.add_argument('--cell', type=float, default=None,
                   help=f"cell side in m (default {P.MISSION_CELL_SIZE_M:g})")
    p.add_argument('--spacing', type=float, default=None,
                   help=f"spacing between lattice nodes in m, rounded to cell/n; the edges "
                        f"stay >= {P.PRM_MIN_EDGE_M:g} m (default {P.MISSION_LATTICE_SPACING_M}; "
                        f"0 = suggested)")
    p.add_argument('--rings', default=None,
                   help=f"rings of linked neighbours, or 'all' (default {P.MISSION_LATTICE_RINGS})")
    p.add_argument('--graph', choices=graph_nodes.GRAPH_SAMPLINGS, default=None,
                   help=f"how the nodes are generated (default {P.MISSION_GRAPH_SAMPLING})")
    p.add_argument('--distribution', choices=graph_nodes.RANDOM_DISTRIBUTIONS, default=None,
                   help=f"random nodes: distribution (default {P.MISSION_RANDOM_DISTRIBUTION})")
    p.add_argument('--density', type=float, default=None,
                   help=f"random nodes: nodes per m2 (default {P.MISSION_RANDOM_DENSITY:g})")
    p.add_argument('--seed', type=int, default=None,
                   help="random nodes: seed, to repeat a graph (default: drawn at random, "
                        "written in the log)")
    p.add_argument('--min-edge', type=float, default=None,
                   help=f"minimum edge length in m (default {P.PRM_MIN_EDGE_M:g})")
    p.add_argument('--max-edge', type=float, default=None,
                   help=f"maximum edge length in m (default {P.PRM_MAX_EDGE_M:g})")
    a = p.parse_args(argv)
    if a.rows is not None and a.rows < 1 or a.cols is not None and a.cols < 1:
        p.error("--rows and --cols must be at least 1")
    if a.cell is not None and a.cell <= 0:
        p.error("--cell must be positive")
    if a.density is not None and a.density <= 0:
        p.error("--density must be positive")
    lo = P.PRM_MIN_EDGE_M if a.min_edge is None else a.min_edge
    hi = P.PRM_MAX_EDGE_M if a.max_edge is None else a.max_edge
    if lo <= 0 or hi < lo:
        p.error("0 < --min-edge <= --max-edge is required")
    return a


def main():
    args = _parse_args()
    if args.graph is not None:
        P.MISSION_GRAPH_SAMPLING = args.graph
    if args.distribution is not None:
        P.MISSION_RANDOM_DISTRIBUTION = args.distribution
    if args.density is not None:
        P.MISSION_RANDOM_DENSITY = float(args.density)
    if args.seed is not None:
        P.MISSION_RANDOM_SEED = int(args.seed)
    if args.min_edge is not None:
        P.PRM_MIN_EDGE_M = float(args.min_edge)
    if args.max_edge is not None:
        P.PRM_MAX_EDGE_M = float(args.max_edge)
    if args.spacing is not None:
        P.MISSION_LATTICE_SPACING_M = None if args.spacing <= 0 else float(args.spacing)
    if args.rings is not None:
        P.MISSION_LATTICE_RINGS = (None if str(args.rings).lower() in ('none', 'tutti', 'all', '-1')
                                 else int(args.rings))

    options = SimpleNamespace()
    options.grid_rows = args.rows
    options.grid_cols = args.cols
    options.grid_cell = args.cell
    options.name = "easyWalk"
    options.hostname = "192.168.80.3"
    options.verbose = False
    options.recording_user_name = ""
    options.recording_session_name = ""
    options.download_filepath = os.getcwd()

    try:
        easy_walk(options)
        return True
    except movements.RobotFallenError as exc:
        print("\n" + "=" * 70)
        print(f"[FALL] MISSION INTERRUPTED: {exc}")
        print("[FALL] The robot is on the ground and has NOT tried to go back to base.")
        print("[FALL] Take control from the tablet, check the robot, then stand it up by hand.")
        print("=" * 70)
        return False
    except Exception as exc:
        logger = bosdyn.client.util.get_logger()
        logger.error('The mission was interrupted by an exception: %r', exc)
        try:
            import traceback
            print(f"[LOG] The mission ended with an exception: {exc!r}")
            print(traceback.format_exc())
        except Exception:
            pass
        return False
    finally:
        summary._emit_mission_summary_once()
        remove_stdout_tee()


if __name__ == '__main__':
    if not main():
        sys.exit(1)
