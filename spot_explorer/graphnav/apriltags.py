"""Il fiducial (AprilTag) di partenza: ricerca, deriva di fine missione, correzione del ritorno a wp_0."""
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
    Fiducial (AprilTag) che Spot vede adesso da una qualunque telecamera, dal piu' vicino:
    [{'id', 'xy' (frame VISION), 'dist', 'avanti', 'sinistra' (m, nel frame del corpo),
    'camera'}]. fiducial_id: solo quello. Riprova fino a wait_s secondi se non ne vede.
    Lista vuota se nessuno, o se il servizio non risponde (mai eccezioni).
    """
    try:
        from bosdyn.api import world_object_pb2
        from bosdyn.client.world_object import WorldObjectClient
        client = robot.ensure_client(WorldObjectClient.default_service_name)
    except Exception as e:
        print(f"[FIDUCIAL] Servizio degli oggetti visti non disponibile: {e}")
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
                              'robot_nel_tag': _robot_in_tag_frame(o.transforms_snapshot, frame, pose,
                                                                    robot_state_client),
                              'dist': float(np.hypot(dx, dy)),
                              'avanti': float(dx * np.cos(yaw) + dy * np.sin(yaw)),
                              'sinistra': float(-dx * np.sin(yaw) + dy * np.cos(yaw)),
                              'camera': tag.frame_name_camera})
        except Exception as e:
            print(f"[FIDUCIAL] Lettura dei fiducial non riuscita: {e}")
            return []
        if found or time.time() >= t_end:
            return sorted(found, key=lambda f: f['dist'])
        time.sleep(0.3)


def _robot_in_tag_frame(snapshot, frame, vision_tform_tag, robot_state_client):
    """Posizione del corpo nel frame del tag (x, y, z), o None. Il tag e' fermo, quindi questa
    posizione non risente della deriva dell'odometria: confrontata con quella della partenza
    dice di quanto il robot e' davvero lontano dal punto di partenza."""
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
    """Distanza (m) fra la posizione del robot alla partenza e adesso, nel frame del tag di
    partenza. Senza la posa del tag (None), dalle posizioni VISION del tag rispetto al robot,
    che risentono della rotazione dell'odometria."""
    a, b = start.get('robot_nel_tag'), now.get('robot_nel_tag')
    if a is not None and b is not None:
        return float(np.linalg.norm(np.subtract(a, b)))
    return None


def describe_fiducial(f):
    """Una riga di log per un fiducial di find_fiducials."""
    side = 'davanti' if f['avanti'] >= 0 else 'dietro'
    return (f"tag {f['id']} a {f['dist']:.2f} m, {side} ({f['avanti']:+.2f} m avanti, "
            f"{f['sinistra']:+.2f} m a sinistra), telecamera {f['camera'] or '?'}")


def graphnav_check_start_fiducial(robot, robot_state_client):
    """
    Al ritorno su wp_0: rivede il fiducial di partenza (MISSION_CTX['start_fiducial'])? Il
    tag non si e' mosso, quindi lo spostamento della sua posizione nel frame VISION e' la
    deriva dell'odometria accumulata in tutta la missione. Solo misura e log.
    """
    start = MISSION_CTX.get('start_fiducial')
    if not start:
        return None
    now = find_fiducials(robot, robot_state_client, wait_s=P.START_FIDUCIAL_WAIT_S,
                         fiducial_id=start['id'])
    if not now:
        print(f"[FIDUCIAL] Al ritorno su wp_0 il tag {start['id']} di partenza non si vede: "
              f"niente misura della deriva di fine missione.")
        return None
    dx, dy = now[0]['xy'][0] - start['xy'][0], now[0]['xy'][1] - start['xy'][1]
    drift = float(np.hypot(dx, dy))
    MISSION_STATS['start_fiducial_drift_m'] = drift
    print(f"[FIDUCIAL] Al ritorno su wp_0 rivedo il {describe_fiducial(now[0])}. Nel frame "
          f"VISION si e' spostato di {drift:.2f} m (dx {dx:+.2f}, dy {dy:+.2f}) da quando "
          f"l'ho visto alla partenza: e' la deriva dell'odometria di tutta la missione.")
    offset = _offset_from_start(start, now[0])
    if offset is not None:
        print(f"[WP0] Secondo il tag il robot e' a {offset:.2f} m dal punto di partenza.")
    return {'drift': drift, 'offset': offset}


def graphnav_correct_wp0_with_fiducial(robot, recordingInterface, robot_state_client, command_client,
                                       offset):
    """
    CORREZIONE DEL RITORNO A wp_0 (vedi WP0_FIDUCIAL_CORRECTION). GraphNav dice "arrivato",
    ma il tag di partenza dice che il robot e' a `offset` m dalla partenza: GraphNav si e'
    localizzato male (29-10 10:57: 1.2 m in un prato con una sola siepe come riferimento).
    Si rilocalizza GraphNav sul tag (come all'avvio) e si torna di nuovo a wp_0, fino a
    WP0_FIDUCIAL_MAX_CORRECTIONS volte, finche' lo scarto non scende sotto
    WP0_FIDUCIAL_TOLERANCE_M. Se il tag non si vede o la rilocalizzazione non riesce, ci si
    ferma dove si e'. Lo scarto finale, o quello di partenza se non si e' corretto niente.
    Mai eccezioni.
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
        print(f"[WP0] Correzione non possibile: wp_0 non trovato ({e}).")
        MISSION_STATS['wp0_offset_final_m'] = round(offset, 2)
        return offset
    for k in range(1, P.WP0_FIDUCIAL_MAX_CORRECTIONS + 1):
        print(f"[WP0] CORREZIONE {k}/{P.WP0_FIDUCIAL_MAX_CORRECTIONS}: GraphNav dice arrivato, ma "
              f"secondo il tag {start['id']} il robot e' a {offset:.2f} m dalla partenza (oltre "
              f"{P.WP0_FIDUCIAL_TOLERANCE_M:.2f} m). Rilocalizzo GraphNav sul tag e torno a wp_0.")
        try:
            ok = recordingInterface.initialize_with_fiducial(robot_state_client, fiducial_id=start['id'])
        except Exception as e:
            print(f"[WP0] Rilocalizzazione sul tag non riuscita ({e}).")
            ok = False
        if not ok:
            print("[WP0] Senza rilocalizzazione sul tag non correggo: resto dove sono.")
            break
        arrived = graphnav_navigate_to(recordingInterface, wp0.id, robot_state_client, command_client,
                                       f"correzione {k} del ritorno a wp_0", move='graphnav_correzione')
        MISSION_STATS['wp0_corrections'] = k
        now = find_fiducials(robot, robot_state_client, wait_s=P.START_FIDUCIAL_WAIT_S, fiducial_id=start['id'])
        new = _offset_from_start(start, now[0]) if now else None
        if new is None:
            print(f"[WP0] Dopo la correzione {k} il tag non si vede piu' (GraphNav: "
                  f"{'arrivato' if arrived else 'non arrivato'}): non so di quanto sono lontano.")
            offset = float('nan')
            break
        print(f"[WP0] Dopo la correzione {k}: a {new:.2f} m dalla partenza (prima {offset:.2f} m), "
              f"GraphNav {'arrivato' if arrived else 'NON arrivato'}.")
        offset = new
        if offset <= P.WP0_FIDUCIAL_TOLERANCE_M:
            break
    MISSION_STATS['wp0_offset_final_m'] = None if not np.isfinite(offset) else round(offset, 2)
    return offset
