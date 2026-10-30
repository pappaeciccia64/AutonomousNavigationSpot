"""The starting fiducial (AprilTag): search, end-of-mission drift, correction of the return to wp_0."""
import time
import numpy as np
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, BODY_FRAME_NAME, get_a_tform_b
from bosdyn.client.math_helpers import Quat, SE3Pose
from spot_explorer import config as P
from spot_explorer.robot import connection as spotUtils
from spot_explorer.records import MISSION_CTX, MISSION_STATS
from spot_explorer.graphnav.navigation import graphnav_navigate_to


def find_fiducials(robot, robot_state_client, wait_s=0.0, fiducial_id=None):
    """
    Fiducials (AprilTags) Spot sees right now from any camera, nearest first:
    [{'id', 'xy' (VISION frame), 'dist', 'avanti', 'sinistra' (m, in the body frame),
    'camera'}]. fiducial_id: only that one. Retries for up to wait_s seconds if it sees none.
    Empty list if none, or if the service does not answer (never raises).
    """
    try:
        from bosdyn.api import world_object_pb2
        from bosdyn.client.world_object import WorldObjectClient
        client = robot.ensure_client(WorldObjectClient.default_service_name)
    except Exception as e:
        print(f"[FIDUCIAL] World-object service not available: {e}")
        return []
    t_end = time.time() + max(0.0, wait_s)
    while True:
        found = []
        try:
            rx, ry, _, q = spotUtils.getPosition(robot_state_client)
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
            resp = client.list_world_objects(object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG])
            for o in resp.world_objects:
                tag = o.apriltag_properties
                if fiducial_id is not None and tag.tag_id != int(fiducial_id):
                    continue
                frame = tag.frame_name_fiducial_filtered or tag.frame_name_fiducial
                pose = get_a_tform_b(o.transforms_snapshot, VISION_FRAME_NAME, frame)
                if pose is None:
                    continue
                dx, dy = pose.x - rx, pose.y - ry
                found.append({'id': int(tag.tag_id), 'xy': (float(pose.x), float(pose.y)),
                              'robot_in_tag': _robot_in_tag_frame(o.transforms_snapshot, frame, pose,
                                                                    robot_state_client),
                              'dist': float(np.hypot(dx, dy)),
                              'forward': float(dx * np.cos(yaw) + dy * np.sin(yaw)),
                              'left': float(-dx * np.sin(yaw) + dy * np.cos(yaw)),
                              'camera': tag.frame_name_camera})
        except Exception as e:
            print(f"[FIDUCIAL] Reading the fiducials failed: {e}")
            return []
        if found or time.time() >= t_end:
            return sorted(found, key=lambda f: f['dist'])
        time.sleep(0.3)


def _robot_in_tag_frame(snapshot, frame, vision_tform_tag, robot_state_client):
    """Body position in the tag frame (x, y, z), or None. The tag does not move, so this
    position is not affected by odometry drift: compared with the one at the start it
    tells how far the robot really is from the starting point."""
    try:
        tag_tform_body = get_a_tform_b(snapshot, frame, BODY_FRAME_NAME)
        if tag_tform_body is None:
            x, y, z, q = spotUtils.getPosition(robot_state_client)
            vision_tform_body = SE3Pose(x, y, z, Quat(w=q.w, x=q.x, y=q.y, z=q.z))
            tag_tform_body = vision_tform_tag.inverse() * vision_tform_body
        return (float(tag_tform_body.x), float(tag_tform_body.y), float(tag_tform_body.z))
    except Exception:
        return None


def _offset_from_start(start, now):
    """Distance (m) between the robot position at the start and now, in the frame of the
    starting tag. Without the tag pose (None), from the VISION positions of the tag relative
    to the robot, which are affected by the odometry rotation."""
    a, b = start.get('robot_in_tag'), now.get('robot_in_tag')
    if a is not None and b is not None:
        return float(np.linalg.norm(np.subtract(a, b)))
    return None


def describe_fiducial(f):
    """One log line for a fiducial from find_fiducials."""
    side = 'in front' if f['forward'] >= 0 else 'behind'
    return (f"tag {f['id']} at {f['dist']:.2f} m, {side} ({f['forward']:+.2f} m forward, "
            f"{f['left']:+.2f} m to the left), camera {f['camera'] or '?'}")


def graphnav_check_start_fiducial(robot, robot_state_client):
    """
    Back on wp_0: does it see the starting fiducial again (MISSION_CTX['start_fiducial'])?
    The tag has not moved, so the shift of its position in the VISION frame is the odometry
    drift accumulated over the whole mission. Measurement and log only.
    """
    start = MISSION_CTX.get('start_fiducial')
    if not start:
        return None
    now = find_fiducials(robot, robot_state_client, wait_s=P.START_FIDUCIAL_WAIT_S,
                         fiducial_id=start['id'])
    if not now:
        print(f"[FIDUCIAL] Back on wp_0 the starting tag {start['id']} is not visible: "
              f"no end-of-mission drift measurement.")
        return None
    dx, dy = now[0]['xy'][0] - start['xy'][0], now[0]['xy'][1] - start['xy'][1]
    drift = float(np.hypot(dx, dy))
    MISSION_STATS['start_fiducial_drift_m'] = drift
    print(f"[FIDUCIAL] Back on wp_0 I see again {describe_fiducial(now[0])}. In the "
          f"VISION frame it moved by {drift:.2f} m (dx {dx:+.2f}, dy {dy:+.2f}) since "
          f"I saw it at the start: it is the odometry drift of the whole mission.")
    offset = _offset_from_start(start, now[0])
    if offset is not None:
        print(f"[WP0] According to the tag the robot is {offset:.2f} m from the starting point.")
    return {'drift': drift, 'offset': offset}


def graphnav_correct_wp0_with_fiducial(robot, recordingInterface, robot_state_client, command_client,
                                       offset):
    """
    CORRECTION OF THE RETURN TO wp_0 (see WP0_FIDUCIAL_CORRECTION). GraphNav says "arrived",
    but the starting tag says the robot is `offset` m from the start: GraphNav localized
    badly (29-10 10:57: 1.2 m on a lawn with a single hedge as reference). GraphNav is
    relocalized on the tag (as at start-up) and the robot goes back to wp_0 again, up to
    WP0_FIDUCIAL_MAX_CORRECTIONS times, until the offset drops below
    WP0_FIDUCIAL_TOLERANCE_M. If the tag is not visible or the relocalization fails, it
    stops where it is. Returns the final offset, or the starting one if nothing was
    corrected. Never raises.
    """
    start = MISSION_CTX.get('start_fiducial')
    MISSION_STATS['wp0_offset_m'] = round(offset, 2)
    MISSION_STATS['wp0_corrections'] = 0
    if not P.WP0_FIDUCIAL_CORRECTION or not start or offset <= P.WP0_FIDUCIAL_TOLERANCE_M:
        MISSION_STATS['wp0_offset_final_m'] = round(offset, 2)
        return offset
    try:
        wp0 = next(w for w in recordingInterface._get_graph(force_refresh=True).waypoints
                   if w.annotations.name == "wp_0")
    except Exception as e:
        print(f"[WP0] Correction not possible: wp_0 not found ({e}).")
        MISSION_STATS['wp0_offset_final_m'] = round(offset, 2)
        return offset
    for k in range(1, P.WP0_FIDUCIAL_MAX_CORRECTIONS + 1):
        print(f"[WP0] CORRECTION {k}/{P.WP0_FIDUCIAL_MAX_CORRECTIONS}: GraphNav says arrived, but "
              f"according to tag {start['id']} the robot is {offset:.2f} m from the start (beyond "
              f"{P.WP0_FIDUCIAL_TOLERANCE_M:.2f} m). I relocalize GraphNav on the tag and go back to wp_0.")
        try:
            ok = recordingInterface.initialize_with_fiducial(robot_state_client, fiducial_id=start['id'])
        except Exception as e:
            print(f"[WP0] Relocalization on the tag failed ({e}).")
            ok = False
        if not ok:
            print("[WP0] Without relocalization on the tag I do not correct: I stay where I am.")
            break
        arrived = graphnav_navigate_to(recordingInterface, wp0.id, robot_state_client, command_client,
                                       f"correction {k} of the return to wp_0", move='graphnav_correction')
        MISSION_STATS['wp0_corrections'] = k
        now = find_fiducials(robot, robot_state_client, wait_s=P.START_FIDUCIAL_WAIT_S, fiducial_id=start['id'])
        new = _offset_from_start(start, now[0]) if now else None
        if new is None:
            print(f"[WP0] After correction {k} the tag is no longer visible (GraphNav: "
                  f"{'arrived' if arrived else 'not arrived'}): I do not know how far I am.")
            offset = float('nan')
            break
        print(f"[WP0] After correction {k}: {new:.2f} m from the start (before {offset:.2f} m), "
              f"GraphNav {'arrived' if arrived else 'NOT arrived'}.")
        offset = new
        if offset <= P.WP0_FIDUCIAL_TOLERANCE_M:
            break
    MISSION_STATS['wp0_offset_final_m'] = None if not np.isfinite(offset) else round(offset, 2)
    return offset
