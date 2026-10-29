"""Dove GraphNav crede che sia il robot: stato della localizzazione, riaggancio quando e' perso, misura della deriva dell'odometria."""
import time
import numpy as np
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.api.graph_nav import graph_nav_pb2 as _graph_nav_pb2
from spot_explorer import config as P
from spot_explorer.robot import connection as spotUtils
from spot_explorer.records import MISSION_STATS, append_csv

_DRIFT_COLUMNS = ['time', 'where', 'waypoint', 'age_s', 'dx', 'dy', 'd', 'gn_x', 'gn_y', 'vision_x',
                  'vision_y']


def graphnav_localization(recordingInterface):
    """Stato del localizzatore di GraphNav come dict, o None se non leggibile. Non scarica il grafo."""
    try:
        st = recordingInterface._graph_nav_client.get_localization_state()
    except Exception as e:
        return {'error': str(e)}
    loc = st.localization
    out = {'wp_id': loc.waypoint_id or None, 'is_lost': None}
    try:
        has_ld = st.HasField('lost_detector_state')
    except Exception:
        has_ld = False      # SDK senza rilevatore di perdita
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
    DERIVA DELL'ODOMETRIA, misurata a ogni arrivo GraphNav. GraphNav si localizza sulle
    scansioni del waypoint e dice dove sta il corpo rispetto a lui (waypoint_tform_body);
    componendolo con la posa VISION che il waypoint aveva quando l'abbiamo creato si ottiene
    dove il robot DOVREBBE essere in VISION senza deriva. La differenza con la posa VISION di
    adesso e' la deriva accumulata dalla creazione del waypoint: la stessa che sposta, nelle
    nostre mappe globali (costruite in VISION), cio' che si e' visto molti minuti prima. Il
    26-10 15:11, all'arrivo su wp_0, lo scarto grezzo era 0.66 m (tolleranza d'arrivo inclusa).

    Solo misura: niente si corregge. Misurabile solo sui waypoint creati da noi (di cella e
    briciole), di cui si conosce la posa VISION. Mai eccezioni verso chi chiama.
    """
    try:
        loc = recordingInterface._graph_nav_client.get_localization_state().localization
        wid = loc.waypoint_id
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        hit = next(((n, p) for n, p in poses.items() if p.get('waypoint_id') == wid), None)
        if not wid or hit is None:
            print(f"[DERIVA] {where}: localizzato su un waypoint automatico di Spot (o non localizzato): "
                  f"deriva non misurabile qui.")
            return None
        name, wp = hit
        b = loc.waypoint_tform_body.position
        c, sn = np.cos(wp['yaw']), np.sin(wp['yaw'])
        gx = wp['x'] + c * b.x - sn * b.y
        gy = wp['y'] + sn * b.x + c * b.y
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
        dx, dy = rx - gx, ry - gy
        d = float(np.hypot(dx, dy))
        age = time.time() - wp['t'] if wp.get('t') else float('nan')
        MISSION_STATS['drift_samples'] += 1
        MISSION_STATS['drift_max_m'] = max(MISSION_STATS['drift_max_m'], d)
        MISSION_STATS['drift_last_m'] = d
        print(f"[DERIVA] {where}: localizzato su {name}, creato {age / 60:.1f} min fa -> l'odometria "
              f"(VISION) e' scostata di {d:.2f} m (dx {dx:+.2f}, dy {dy:+.2f}) da dove GraphNav "
              f"mette il robot.")
        append_csv("drift.csv", _DRIFT_COLUMNS,
                    [[time.time(), where, name, round(age, 1), round(dx, 4), round(dy, 4), round(d, 4),
                      round(gx, 4), round(gy, 4), round(rx, 4), round(ry, 4)]])
        return d
    except Exception as e:
        print(f"[DERIVA] {where}: misura non riuscita ({type(e).__name__}: {e}).")
        return None


def graphnav_is_lost(loc):
    return bool(loc) and (loc.get('is_lost') is True or ('error' not in loc and not loc.get('wp_id')))


def graphnav_log_localization(env, recordingInterface, robot_state_client, where, force=False):
    """
    Riga [LOCALIZZAZIONE]: stampata solo se lo stato cambia (perso si'/no, localizzazioni
    rifiutate dall'ultima accettata, archi cattivi di fila) o con force=True. Riporta anche
    il tier del passo appena fatto e il beccheggio del corpo adesso, per vedere se la perdita
    scatta in crawl o in salita. Non solleva mai.
    """
    if recordingInterface is None:
        return None
    loc = graphnav_localization(recordingInterface)
    if loc is None:
        return None
    if 'error' in loc:
        print(f"[LOCALIZZAZIONE] {where}: stato non leggibile ({loc['error']}).")
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
        q = spotUtils.getPosition(robot_state_client)[3]
        pitch = float(np.degrees(np.arcsin(np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))))
    except Exception:
        pass
    if loc.get('is_lost') is None:
        det = "rilevatore di perdita non disponibile"
    else:
        det = (f"localizzazioni accettate {loc['accepted']}, rifiutate {loc['rejected']} "
               f"({loc['rej_since']} dall'ultima accettata, {loc['dist_rej']:.1f} m fatti cosi'), "
               f"archi cattivi di fila {loc['bad_edges']}")
    loc_state = ("PERSO" if loc.get('is_lost') else
             (f"localizzato su {wp_name_by_id(recordingInterface, loc['wp_id'])}" if loc.get('wp_id')
              else "NON localizzato"))
    print(f"[LOCALIZZAZIONE] {where}: {loc_state} | {det} | ultimo passo {MISSION_STATS.get('last_tier')}, "
          f"beccheggio {pitch:.1f} gradi.")
    return loc


def graphnav_relocalize(recordingInterface, robot_state_client, why=""):
    """
    Riaggancia GraphNav sul waypoint piu' vicino (secondo le nostre pose in visione). La
    stima iniziale e' waypoint_tform_ko (dal grafo) * ko_tform_body (adesso): la stessa
    odometria con cui Spot ha registrato il waypoint. True se Spot accetta.
    """
    try:
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        cands = sorted(((float(np.hypot(p['x'] - rx, p['y'] - ry)), name, p['waypoint_id'])
                        for name, p in poses.items() if p.get('waypoint_id')), key=lambda c: c[0])
        if not cands:
            print(f"[LOCALIZZAZIONE] Riaggancio {why}: nessun waypoint nostro noto.")
            return False
        d_wp, name, wp_id = cands[0]
        if d_wp > P.GRAPHNAV_RELOC_MAX_WP_DIST_M:
            print(f"[LOCALIZZAZIONE] Riaggancio {why}: il waypoint piu' vicino ({name}) e' a "
                  f"{d_wp:.1f} m, troppo lontano.")
            return False
        graph = recordingInterface._get_graph()
        wp = next((w for w in graph.waypoints if w.id == wp_id), None) if graph else None
        if wp is None:
            graph = recordingInterface._get_graph(force_refresh=True)
            wp = next((w for w in graph.waypoints if w.id == wp_id), None) if graph else None
        if wp is None:
            print(f"[LOCALIZZAZIONE] Riaggancio {why}: {name} non e' nel grafo.")
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
        print(f"[LOCALIZZAZIONE] Riaggancio {why}: riuscito su {name} (a {d_wp:.2f} m secondo le "
              f"nostre pose; stima dall'odometria, rifinita dall'ICP).")
        return True
    except Exception as e:
        MISSION_STATS['graphnav_relocalization_fail'] = MISSION_STATS.get('graphnav_relocalization_fail', 0) + 1
        print(f"[LOCALIZZAZIONE] Riaggancio {why}: non riuscito ({e}).")
        return False


def _distance_from_waypoint(recordingInterface, robot_state_client, waypoint_id, name):
    """(distanza del robot dal waypoint, 'da GraphNav' o 'dalle pose VISION'), (None, '') se
    non si sa. Mai eccezioni."""
    try:
        loc = recordingInterface._graph_nav_client.get_localization_state().localization
        if loc.waypoint_id == waypoint_id:
            p = loc.waypoint_tform_body.position
            return float(np.hypot(p.x, p.y)), 'da GraphNav'
        # Localizzato su un ALTRO waypoint (per esempio una briciola della partenza, accanto a
        # wp_0): posa del corpo rispetto a lui, composta con le pose dei due waypoint salvate
        # alla creazione. La deriva fra le due creazioni e' piccola se sono vicine nel tempo,
        # molto meno di quella delle pose VISION di adesso (28-10 14:23: ~1 m su wp_0).
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        by_id = {p.get('waypoint_id'): p for p in poses.values() if p.get('waypoint_id')}
        a, b = by_id.get(loc.waypoint_id), by_id.get(waypoint_id)
        if loc.waypoint_id and a is not None and b is not None:
            bx, by = float(loc.waypoint_tform_body.position.x), float(loc.waypoint_tform_body.position.y)
            c, s = np.cos(a['yaw']), np.sin(a['yaw'])
            rx, ry = a['x'] + bx * c - by * s, a['y'] + bx * s + by * c
            return float(np.hypot(rx - b['x'], ry - b['y'])), 'da GraphNav (su un waypoint vicino)'
    except Exception:
        pass
    try:
        wp = (getattr(recordingInterface, 'waypoint_poses', None) or {}).get(name)
        if wp is not None:
            rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
            return float(np.hypot(rx - wp['x'], ry - wp['y'])), 'dalle pose VISION'
    except Exception:
        pass
    return None, ''


def wp_name_by_id(recordingInterface, wp_id):
    for name, p in (getattr(recordingInterface, 'waypoint_poses', None) or {}).items():
        if p.get('waypoint_id') == wp_id:
            return name
    return (wp_id or '?')[:12]
