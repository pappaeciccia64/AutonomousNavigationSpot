"""GraphNav moves: trip to a waypoint, going back a few breadcrumbs, return to wp_0."""
import time
import numpy as np
from bosdyn.api.graph_nav import graph_nav_pb2 as _graph_nav_pb2
from bosdyn.api import geometry_pb2
from spot_explorer.robot import motion
from spot_explorer import config as P
from spot_explorer.robot import connection
from spot_explorer.records import MISSION_STATS, TrajectoryLog, timing_add
from spot_explorer.graphnav.localization import (_distance_from_waypoint, graphnav_is_lost, graphnav_log_drift, graphnav_log_localization, graphnav_relocalize)
from spot_explorer.graphnav.map_shortcuts import graphnav_add_shortcuts


# =========================================================================================
# GRAPHNAV DURING THE MISSION: moves between cells, going back, return to wp_0; shortcuts
# and loop closure; localization, re-anchoring and drift; breadcrumbs.
# Uses RecordingInterface (map_recording) to talk to GraphNav.
# =========================================================================================
_GN = _graph_nav_pb2.NavigationFeedbackResponse

_GRAPHNAV_FAIL_STATUSES = {
    _GN.STATUS_STUCK: "blocked (STUCK)", _GN.STATUS_NO_ROUTE: "no path in the graph",
    _GN.STATUS_NO_LOCALIZATION: "not localized", _GN.STATUS_ROBOT_IMPAIRED: "robot impaired (fallen?)",
    _GN.STATUS_CONSTRAINT_FAULT: "constraint violated", _GN.STATUS_COMMAND_OVERRIDDEN: "command replaced",
    _GN.STATUS_NOT_LOCALIZED_TO_ROUTE: "not localized on the route", _GN.STATUS_LEASE_ERROR: "lease error",
    _GN.STATUS_AREA_CALLBACK_ERROR: "area callback error",
}


def graphnav_arrival_heading(recordingInterface, waypoint_id, face_xy, label):
    """
    HEADING ON ARRIVAL (see P.GRAPHNAV_ARRIVAL_HEADING): the body pose relative to the
    waypoint to ask GraphNav for (destination_waypoint_tform_body_goal), to arrive facing
    face_xy (VISION point). The waypoint frame has the yaw the body had when we created it
    (waypoint_poses), so the angle is the difference between the wanted direction and that
    one. None = waypoint heading, as without it: waypoint not created by us, face_xy missing
    or too close. Never raises.
    """
    if face_xy is None or P.GRAPHNAV_ARRIVAL_HEADING != 'toward_goal':
        return None
    try:
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        wp = next((p for p in poses.values() if p.get('waypoint_id') == waypoint_id), None)
        if wp is None:
            return None
        dx, dy = face_xy[0] - wp['x'], face_xy[1] - wp['y']
        if float(np.hypot(dx, dy)) < P.GRAPHNAV_ARRIVAL_HEADING_MIN_M:
            return None
        want = float(np.arctan2(dy, dx))
        rel = float((want - wp['yaw'] + np.pi) % (2.0 * np.pi) - np.pi)
        print(f"[GRAPHNAV] {label}: arrival facing ({face_xy[0]:.2f}, {face_xy[1]:.2f}), "
              f"yaw {np.degrees(want):.0f} deg ({np.degrees(rel):+.0f} relative to the waypoint).")
        return geometry_pb2.SE2Pose(position=geometry_pb2.Vec2(x=0.0, y=0.0), angle=rel)
    except Exception as e:
        print(f"[GRAPHNAV] {label}: arrival heading not computed ({e}): using the waypoint's.")
        return None


def graphnav_navigate_to(recordingInterface, waypoint_id, robot_state_client, command_client, label,
                          timeout_s=P.GRAPHNAV_TIMEOUT_S, face_xy=None, move='graphnav'):
    """Take the robot to a waypoint with GraphNav. True only if it really gets there.

    face_xy: VISION point to face on arrival (see graphnav_arrival_heading); None =
    waypoint heading. With GRAPHNAV_ARRIVAL_HEADING = 'free', if face_xy is given the
    robot arrives without turning. move: label of the move in trajectory.csv."""
    # Before moving: Spot's loop closures (with recording stopped). The shortcuts from
    # the map are added by the caller, BEFORE stopping the recording.
    # Everything in here ends up in the 'GraphNav' entry of [TIMING]: it is one of the
    # longest phases of the mission (the return to wp_0 alone can take minutes).
    _t_gn = time.perf_counter()
    try:
        # 2026-10-26: GraphNav legs end up in trajectory.csv too (see _TRAJECTORY_COLUMNS).
        with TrajectoryLog(robot_state_client, move):
            return _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client,
                                               command_client, label, timeout_s, face_xy)
    finally:
        timing_add('GraphNav', time.perf_counter() - _t_gn)


# Why the last GraphNav move stopped without arriving (for the return to wp_0).
_LAST_GRAPHNAV_FAIL = {'reason': None}


def _stop_graphnav(command_client, reason):
    _LAST_GRAPHNAV_FAIL['reason'] = reason.replace("GraphNav: ", "")
    motion.stop_robot(command_client, reason=reason)


def _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client, command_client,
                                label, timeout_s, face_xy=None):
    _LAST_GRAPHNAV_FAIL['reason'] = None
    client = recordingInterface._graph_nav_client
    tp = _graph_nav_pb2.TravelParams()
    tp.velocity_limit.CopyFrom(motion._velocity_limit(P.GRAPHNAV_MAX_LINEAR_VEL_MPS, P.GRAPHNAV_MAX_ANGULAR_VEL_RPS))
    # Permissive loss detector (see GRAPHNAV LOCALIZATION).
    if P.GRAPHNAV_LOST_DETECTOR_STRICTNESS and hasattr(tp, 'lost_detector_strictness'):
        try:
            # from the descriptor: the enum is in lost_detection_pb2, which old SDK versions
            # do not have; this way it works wherever the field exists.
            enum = tp.DESCRIPTOR.fields_by_name['lost_detector_strictness'].enum_type
            tp.lost_detector_strictness = enum.values_by_name[P.GRAPHNAV_LOST_DETECTOR_STRICTNESS].number
        except Exception:
            pass
    goal_pose = graphnav_arrival_heading(recordingInterface, waypoint_id, face_xy, label)
    if face_xy is not None and P.GRAPHNAV_ARRIVAL_HEADING == 'free' and hasattr(tp, 'ignore_final_yaw'):
        tp.ignore_final_yaw = True
    print(f"[GRAPHNAV] {label}: start (at most {timeout_s:.0f} s, {P.GRAPHNAV_MAX_LINEAR_VEL_MPS:.1f} m/s).")
    # Before leaving, check the localizer state; if it is lost, re-anchor
    # right away, instead of having the command rejected.
    relocs = 0
    loc0 = graphnav_log_localization(None, recordingInterface, robot_state_client,
                                     f"before '{label}'", force=True)
    if graphnav_is_lost(loc0):
        relocs += 1
        graphnav_relocalize(recordingInterface, robot_state_client, f"before '{label}'")
    t0, cmd_id, fb_errors, nav_errors = time.time(), None, 0, 0
    while time.time() - t0 < timeout_s:
        try:
            cmd_id = client.navigate_to(waypoint_id, 1.0, command_id=cmd_id, travel_params=tp,
                                        destination_waypoint_tform_body_goal=goal_pose)
            nav_errors = 0
        except Exception as e:
            if 'RobotLost' in type(e).__name__ and relocs < P.GRAPHNAV_MAX_RELOCALIZATIONS:
                relocs += 1
                print(f"[GRAPHNAV] {label}: GraphNav says the robot is lost: re-anchoring "
                      f"({relocs}/{P.GRAPHNAV_MAX_RELOCALIZATIONS}).")
                graphnav_relocalize(recordingInterface, robot_state_client, f"during '{label}'")
                cmd_id = None
                time.sleep(0.5)
                continue
            nav_errors += 1
            print(f"[GRAPHNAV] {label}: command rejected ({nav_errors}/3): {e}")
            # A transient RPC error does not abandon the move: give up at the third.
            if nav_errors >= 3:
                _stop_graphnav(command_client, reason="GraphNav: command rejected")
                MISSION_STATS['graphnav_fail'] += 1
                return False
            cmd_id = None
            time.sleep(0.5)
            continue
        time.sleep(0.5)
        try:
            status = client.navigation_feedback(cmd_id).status
            fb_errors = 0
        except Exception as e:
            fb_errors += 1
            print(f"[GRAPHNAV] {label}: feedback error ({fb_errors}/3): {e}")
            if fb_errors >= 3:
                _stop_graphnav(command_client, reason="GraphNav: feedback not available")
                MISSION_STATS['graphnav_fail'] += 1
                return False
            cmd_id = None      # the command may no longer be valid: send a new one
            time.sleep(0.5)
            continue
        if status == _GN.STATUS_REACHED_GOAL:
            print(f"[GRAPHNAV] {label}: arrived in {time.time() - t0:.0f} s.")
            MISSION_STATS['graphnav_ok'] += 1
            graphnav_log_drift(recordingInterface, robot_state_client, label)
            return True
        if status == _GN.STATUS_LOST and relocs < P.GRAPHNAV_MAX_RELOCALIZATIONS:
            # Re-anchor on the nearest waypoint (NOT on the destination, and without declaring
            # arrival: the command is resent and navigation continues).
            relocs += 1
            print(f"[GRAPHNAV] {label}: GraphNav lost the localization on the way: "
                  f"re-anchoring ({relocs}/{P.GRAPHNAV_MAX_RELOCALIZATIONS}).")
            graphnav_log_localization(None, recordingInterface, robot_state_client,
                                      f"lost during '{label}'", force=True)
            graphnav_relocalize(recordingInterface, robot_state_client, f"during '{label}'")
            cmd_id = None
            continue
        if status == _GN.STATUS_LOST:
            # Localization is not forced on the destination (see above): failure.
            print(f"[GRAPHNAV] {label}: GraphNav lost the localization. Stopping the robot.")
            _stop_graphnav(command_client, reason="GraphNav: robot lost")
            MISSION_STATS['graphnav_fail'] += 1
            return False
        if status == _GN.STATUS_COMMAND_TIMED_OUT:
            cmd_id = None          # command expired: send a new one
            continue
        if status in _GRAPHNAV_FAIL_STATUSES:
            print(f"[GRAPHNAV] {label}: interrupted -- {_GRAPHNAV_FAIL_STATUSES[status]}.")
            _stop_graphnav(command_client, reason=f"GraphNav: {_GRAPHNAV_FAIL_STATUSES[status]}")
            MISSION_STATS['graphnav_fail'] += 1
            return False
    print(f"[GRAPHNAV] {label}: time limit of {timeout_s:.0f} s exceeded. Stopping the robot.")
    _stop_graphnav(command_client, reason="GraphNav: time limit exceeded")
    MISSION_STATS['graphnav_fail'] += 1
    return False


def graphnav_return_to_start(recordingInterface, robot_state_client, command_client):
    """
    Return to wp_0 at the end of the mission, with a time limit. Three outcomes:
      'arrived'      GraphNav says it has arrived;
      'near'        GraphNav stopped without arriving (STUCK, timeout...) but the robot
                      is within P.GRAPHNAV_RETURN_NEAR_M of wp_0: the return did NOT
                      succeed, but the fiducial and the drift are measured anyway (13:50 on
                      28-10: stuck in the grass at 0.63 m, measurements lost);
      'not_arrived'  everything else.
    The distance from wp_0 is the one GraphNav reports if it is localized on wp_0,
    otherwise the one from the VISION poses (with the odometry drift in it).
    """
    try:
        graph = recordingInterface._get_graph(force_refresh=True)
        wp0 = next((w for w in graph.waypoints if w.annotations.name == "wp_0"), None)
    except Exception as e:
        print(f"[GRAPHNAV] Return to wp_0: graph not available ({e}).")
        return 'not_arrived'
    if wp0 is None:
        print("[GRAPHNAV] Return to wp_0: waypoint wp_0 not found in the graph.")
        return 'not_arrived'
    if graphnav_navigate_to(recordingInterface, wp0.id, robot_state_client, command_client,
                            "return to wp_0", timeout_s=2 * P.GRAPHNAV_TIMEOUT_S):
        MISSION_STATS['wp0_return'] = 'arrived'
        return 'arrived'
    why = _LAST_GRAPHNAV_FAIL.get('reason') or 'reason not recorded'
    d, how = _distance_from_waypoint(recordingInterface, robot_state_client, wp0.id, 'wp_0')
    MISSION_STATS['wp0_return'] = 'not_arrived'
    MISSION_STATS['wp0_return_dist_m'] = None if d is None else round(d, 2)
    if d is not None and d <= P.GRAPHNAV_RETURN_NEAR_M:
        MISSION_STATS['wp0_return'] = 'near'
        print(f"[GRAPHNAV] Return to wp_0 NOT successful but near: stopped {d:.2f} m from wp_0 "
              f"(reason: {why}; distance {how}). Measuring fiducial and drift anyway.")
        graphnav_log_drift(recordingInterface, robot_state_client, "return to wp_0 (stopped nearby)")
        return 'near'
    print(f"[GRAPHNAV] Return to wp_0 NOT successful: stopped "
          + (f"{d:.2f} m from wp_0" if d is not None else "at an unknown distance from wp_0")
          + f" (reason: {why}).")
    return 'not_arrived'


def graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                       min_dist_m=0.40, face_xy=None):
    """
    Go back with GraphNav to the most recent waypoint at least min_dist_m BEHIND.

    The search goes backwards along the track because the last breadcrumb may have been
    dropped here, where the robot is now: going back to it would move nothing. If no
    breadcrumb is far enough, the oldest available one is used anyway.

    Returns (ok, crumb). ok=True only if GraphNav declares it has ARRIVED: in
    _graphnav_navigate_to_inner STATUS_LOST is a failure, localization is not forced on the
    destination (see GRAPHNAV MOVES).

    Recording must be STOPPED before navigating and resumed after: GraphNav does not
    navigate while recording. It is the same sequence used for moves between cells.
    """
    trail = getattr(env, '_graphnav_trail', None) or []
    if not trail:
        return False, None
    try:
        rx, ry, _, _ = connection.getPosition(robot_state_client)
    except Exception:
        rx, ry = trail[-1]['x'], trail[-1]['y']
    idx = None
    for k in range(len(trail) - 1, -1, -1):
        if float(np.hypot(trail[k]['x'] - rx, trail[k]['y'] - ry)) >= min_dist_m:
            idx = k
            break
    if idx is None:
        idx = 0
    hop = len(trail) - 1 - idx
    crumb = trail[idx]
    d_crumb = float(np.hypot(crumb['x'] - rx, crumb['y'] - ry))
    if d_crumb < min_dist_m:
        # All breadcrumbs are around here (22-10 16:22: the robot was ON wp_138): GraphNav
        # would not move the robot. Replan from where it is, without moving.
        MISSION_STATS['backtracks_in_place'] += 1
        print(f"[RETURN] The farthest breadcrumb behind, {crumb['wp_name']}, is {d_crumb:.2f} m away "
              f"(less than {min_dist_m:.2f}): I do not move, I replan from here.")
        return True, dict(crumb, in_place=True)
    print(f"[RETURN] Going back to {crumb['wp_name']}, {d_crumb:.2f} m from the robot.")
    recording_stopped = False
    graphnav_add_shortcuts(recordingInterface, "before the return", map_edges=True,
                           target_wp_id=crumb['wp_id'], robot_xy=(rx, ry))
    try:
        recordingInterface.stop_recording()
        recording_stopped = True
    except Exception as e:
        print(f"[RETURN] Could not stop the recording ({e}): trying anyway.")
    ok = graphnav_navigate_to(
        recordingInterface, crumb['wp_id'], robot_state_client, command_client,
        f"return to waypoint {crumb['wp_name']} at ({crumb['x']:.2f}, {crumb['y']:.2f})",
        face_xy=face_xy)
    if recording_stopped:
        try:
            recordingInterface.start_recording()
        except Exception as e:
            print(f"[RETURN] Recording not resumed ({e}): the following breadcrumbs "
                  "may not be created.")
    if ok:
        # The walked legs no longer describe the way the robot came:
        # GraphNav does not record them, and the fallback retreat must not use them.
        env._traveled_arcs = []
        del trail[len(trail) - hop:]        # breadcrumbs beyond this one are no longer needed
    return ok, crumb
