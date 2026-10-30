"""Where GraphNav thinks the robot is: localization state, re-anchoring when lost, odometry drift measurement."""
import time
import numpy as np
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.api.graph_nav import graph_nav_pb2 as _graph_nav_pb2
from spot_explorer import config as P
from spot_explorer.robot import connection
from spot_explorer.records import MISSION_STATS, append_csv

_DRIFT_COLUMNS = ['time', 'where', 'waypoint', 'age_s', 'dx', 'dy', 'd', 'gn_x', 'gn_y', 'vision_x',
                  'vision_y']


def graphnav_localization(recordingInterface):
    """GraphNav localizer state as a dict, or None if unreadable. Does not download the graph."""
    try:
        st = recordingInterface._graph_nav_client.get_localization_state()
    except Exception as e:
        return {'error': str(e)}
    loc = st.localization
    out = {'wp_id': loc.waypoint_id or None, 'is_lost': None}
    try:
        has_ld = st.HasField('lost_detector_state')
    except Exception:
        has_ld = False      # SDK without a loss detector
    if has_ld:
        ld = st.lost_detector_state
        out.update(is_lost=bool(ld.is_lost),
                   accepted=int(ld.total_num_accepted_localizations),
                   rejected=int(ld.total_num_rejected_localizations),
                   rej_since=int(ld.num_rejected_localizations_since_accepted),
                   dist_rej=float(ld.distance_traveled_with_rejected_localization),
                   bad_edges=int(ld.num_consecutive_bad_edges))
    return out


def graphnav_log_drift(recordingInterface, robot_state_client, where):
    """
    ODOMETRY DRIFT, measured at every GraphNav arrival. GraphNav localizes on the
    waypoint's scans and says where the body is relative to it (waypoint_tform_body);
    composing it with the VISION pose the waypoint had when we created it gives where the
    robot SHOULD be in VISION without drift. The difference from the current VISION pose is
    the drift accumulated since the waypoint was created: the same drift that shifts, in our
    global maps (built in VISION), what was seen many minutes earlier. On 26-10 15:11, on
    arrival at wp_0, the raw offset was 0.66 m (arrival tolerance included).

    Measurement only: nothing is corrected. Only measurable on waypoints we created (cell
    waypoints and breadcrumbs), whose VISION pose is known. Never raises to the caller.
    """
    try:
        loc = recordingInterface._graph_nav_client.get_localization_state().localization
        wid = loc.waypoint_id
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        hit = next(((n, p) for n, p in poses.items() if p.get('waypoint_id') == wid), None)
        if not wid or hit is None:
            print(f"[DRIFT] {where}: localized on an automatic Spot waypoint (or not localized): "
                  f"drift not measurable here.")
            return None
        name, wp = hit
        b = loc.waypoint_tform_body.position
        c, sn = np.cos(wp['yaw']), np.sin(wp['yaw'])
        gx = wp['x'] + c * b.x - sn * b.y
        gy = wp['y'] + sn * b.x + c * b.y
        rx, ry, _, _ = connection.getPosition(robot_state_client)
        dx, dy = rx - gx, ry - gy
        d = float(np.hypot(dx, dy))
        age = time.time() - wp['t'] if wp.get('t') else float('nan')
        MISSION_STATS['drift_samples'] += 1
        MISSION_STATS['drift_max_m'] = max(MISSION_STATS['drift_max_m'], d)
        MISSION_STATS['drift_last_m'] = d
        print(f"[DRIFT] {where}: localized on {name}, created {age / 60:.1f} min ago -> the odometry "
              f"(VISION) is off by {d:.2f} m (dx {dx:+.2f}, dy {dy:+.2f}) from where GraphNav "
              f"puts the robot.")
        append_csv("drift.csv", _DRIFT_COLUMNS,
                    [[time.time(), where, name, round(age, 1), round(dx, 4), round(dy, 4), round(d, 4),
                      round(gx, 4), round(gy, 4), round(rx, 4), round(ry, 4)]])
        return d
    except Exception as e:
        print(f"[DRIFT] {where}: measurement failed ({type(e).__name__}: {e}).")
        return None


def graphnav_is_lost(loc):
    return bool(loc) and (loc.get('is_lost') is True or ('error' not in loc and not loc.get('wp_id')))


def graphnav_log_localization(env, recordingInterface, robot_state_client, where, force=False):
    """
    [LOCALIZZAZIONE] line: printed only if the state changes (lost yes/no, localizations
    rejected since the last accepted one, bad edges in a row) or with force=True. Also
    reports the tier of the leg just done and the current body pitch, to see whether the
    loss happens in crawl or uphill. Never raises.
    """
    if recordingInterface is None:
        return None
    loc = graphnav_localization(recordingInterface)
    if loc is None:
        return None
    if 'error' in loc:
        print(f"[LOCALIZATION] {where}: state not readable ({loc['error']}).")
        return loc
    key = (loc.get('is_lost'), bool(loc.get('wp_id')), (loc.get('rej_since') or 0) > 0,
           (loc.get('bad_edges') or 0) > 0)
    prev = getattr(env, '_loc_key', None) if env is not None else None
    if env is not None:
        env._loc_key = key
    if not force and key == prev:
        return loc
    pitch = float('nan')
    try:
        q = connection.getPosition(robot_state_client)[3]
        pitch = float(np.degrees(np.arcsin(np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))))
    except Exception:
        pass
    if loc.get('is_lost') is None:
        det = "loss detector not available"
    else:
        det = (f"localizations accepted {loc['accepted']}, rejected {loc['rejected']} "
               f"({loc['rej_since']} since the last accepted, {loc['dist_rej']:.1f} m done like that), "
               f"bad edges in a row {loc['bad_edges']}")
    loc_state = ("PERSO" if loc.get('is_lost') else
             (f"localized on {wp_name_by_id(recordingInterface, loc['wp_id'])}" if loc.get('wp_id')
              else "NOT localized"))
    print(f"[LOCALIZATION] {where}: {loc_state} | {det} | last step {MISSION_STATS.get('last_tier')}, "
          f"pitch {pitch:.1f} deg.")
    return loc


def graphnav_relocalize(recordingInterface, robot_state_client, why=""):
    """
    Re-anchor GraphNav on the nearest waypoint (according to our vision poses). The
    initial guess is waypoint_tform_ko (from the graph) * ko_tform_body (now): the same
    odometry Spot recorded the waypoint with. True if Spot accepts.
    """
    try:
        rx, ry, _, _ = connection.getPosition(robot_state_client)
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        cands = sorted(((float(np.hypot(p['x'] - rx, p['y'] - ry)), name, p['waypoint_id'])
                        for name, p in poses.items() if p.get('waypoint_id')), key=lambda c: c[0])
        if not cands:
            print(f"[LOCALIZATION] Re-anchoring {why}: no waypoint of ours known.")
            return False
        d_wp, name, wp_id = cands[0]
        if d_wp > P.GRAPHNAV_RELOC_MAX_WP_DIST_M:
            print(f"[LOCALIZATION] Re-anchoring {why}: the nearest waypoint ({name}) is "
                  f"{d_wp:.1f} m away, too far.")
            return False
        graph = recordingInterface._get_graph()
        wp = next((w for w in graph.waypoints if w.id == wp_id), None) if graph else None
        if wp is None:
            graph = recordingInterface._get_graph(force_refresh=True)
            wp = next((w for w in graph.waypoints if w.id == wp_id), None) if graph else None
        if wp is None:
            print(f"[LOCALIZATION] Re-anchoring {why}: {name} is not in the graph.")
            return False
        from bosdyn.client.math_helpers import SE3Pose
        from bosdyn.api.graph_nav import nav_pb2 as _nav_pb2
        state = robot_state_client.get_robot_state()
        odom_tform_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
        guess = SE3Pose.from_proto(wp.waypoint_tform_ko) * odom_tform_body
        loc = _nav_pb2.Localization()
        loc.waypoint_id = wp.id
        loc.waypoint_tform_body.CopyFrom(guess.to_proto())
        recordingInterface._graph_nav_client.set_localization(
            initial_guess_localization=loc,
            ko_tform_body=odom_tform_body.to_proto(),
            max_distance=P.GRAPHNAV_RELOC_MAX_DIST_M,
            max_yaw=P.GRAPHNAV_RELOC_MAX_YAW_RAD,
            fiducial_init=_graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL)
        MISSION_STATS['graphnav_relocalizations'] = MISSION_STATS.get('graphnav_relocalizations', 0) + 1
        print(f"[LOCALIZATION] Re-anchoring {why}: succeeded on {name} ({d_wp:.2f} m away according to "
              f"our poses; guess from odometry, refined by ICP).")
        return True
    except Exception as e:
        MISSION_STATS['graphnav_relocalization_fail'] = MISSION_STATS.get('graphnav_relocalization_fail', 0) + 1
        print(f"[LOCALIZATION] Re-anchoring {why}: failed ({e}).")
        return False


def _distance_from_waypoint(recordingInterface, robot_state_client, waypoint_id, name):
    """(robot distance from the waypoint, 'da GraphNav' or 'dalle pose VISION'), (None, '') if
    unknown. Never raises."""
    try:
        loc = recordingInterface._graph_nav_client.get_localization_state().localization
        if loc.waypoint_id == waypoint_id:
            p = loc.waypoint_tform_body.position
            return float(np.hypot(p.x, p.y)), 'from GraphNav'
        # Localized on ANOTHER waypoint (for example a breadcrumb at the start, next to
        # wp_0): body pose relative to it, composed with the poses of the two waypoints saved
        # at creation. The drift between the two creations is small if they are close in time,
        # much less than that of the current VISION poses (28-10 14:23: ~1 m on wp_0).
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        by_id = {p.get('waypoint_id'): p for p in poses.values() if p.get('waypoint_id')}
        a, b = by_id.get(loc.waypoint_id), by_id.get(waypoint_id)
        if loc.waypoint_id and a is not None and b is not None:
            bx, by = float(loc.waypoint_tform_body.position.x), float(loc.waypoint_tform_body.position.y)
            c, s = np.cos(a['yaw']), np.sin(a['yaw'])
            rx, ry = a['x'] + bx * c - by * s, a['y'] + bx * s + by * c
            return float(np.hypot(rx - b['x'], ry - b['y'])), 'from GraphNav (on a nearby waypoint)'
    except Exception:
        pass
    try:
        wp = (getattr(recordingInterface, 'waypoint_poses', None) or {}).get(name)
        if wp is not None:
            rx, ry, _, _ = connection.getPosition(robot_state_client)
            return float(np.hypot(rx - wp['x'], ry - wp['y'])), 'from the VISION poses'
    except Exception:
        pass
    return None, ''


def wp_name_by_id(recordingInterface, wp_id):
    for name, p in (getattr(recordingInterface, 'waypoint_poses', None) or {}).items():
        if p.get('waypoint_id') == wp_id:
            return name
    return (wp_id or '?')[:12]
