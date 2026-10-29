"""Spostamenti con GraphNav: viaggio a un waypoint, ritorno di qualche briciola, ritorno a wp_0."""
import time
import numpy as np
from bosdyn.api.graph_nav import graph_nav_pb2 as _graph_nav_pb2
from bosdyn.api import geometry_pb2
from spot_explorer.robot import motion as movements
from spot_explorer import config as P
from spot_explorer.robot import connection as spotUtils
from spot_explorer.records import MISSION_STATS, TrajectoryLog, timing_add
from spot_explorer.graphnav.localization import (_distance_from_waypoint, graphnav_is_lost, graphnav_log_drift, graphnav_log_localization, graphnav_relocalize)
from spot_explorer.graphnav.map_shortcuts import graphnav_add_shortcuts


# =========================================================================================
# GRAPHNAV DURANTE LA MISSIONE: spostamenti fra celle, ritorni, ritorno a wp_0; scorciatoie
# e chiusura d'anello; localizzazione, riaggancio e deriva; briciole.
# Usa RecordingInterface (sopra) per parlare con GraphNav.
# =========================================================================================
_GN = _graph_nav_pb2.NavigationFeedbackResponse

_GRAPHNAV_FAIL_STATUSES = {
    _GN.STATUS_STUCK: "bloccato (STUCK)", _GN.STATUS_NO_ROUTE: "nessun percorso nel grafo",
    _GN.STATUS_NO_LOCALIZATION: "non localizzato", _GN.STATUS_ROBOT_IMPAIRED: "robot in difficolta' (caduto?)",
    _GN.STATUS_CONSTRAINT_FAULT: "vincolo violato", _GN.STATUS_COMMAND_OVERRIDDEN: "comando sostituito",
    _GN.STATUS_NOT_LOCALIZED_TO_ROUTE: "non localizzato sul percorso", _GN.STATUS_LEASE_ERROR: "errore di lease",
    _GN.STATUS_AREA_CALLBACK_ERROR: "errore di area callback",
}


def graphnav_arrival_heading(recordingInterface, waypoint_id, face_xy, label):
    """
    ORIENTAMENTO ALL'ARRIVO (vedi P.GRAPHNAV_ARRIVAL_HEADING): la posa del corpo rispetto al
    waypoint da chiedere a GraphNav (destination_waypoint_tform_body_goal), per arrivare
    girati verso face_xy (punto VISION). Il frame del waypoint ha l'imbardata che il corpo
    aveva quando l'abbiamo creato (waypoint_poses), quindi l'angolo e' la differenza fra la
    direzione voluta e quella. None = orientamento del waypoint, come senza: waypoint non
    creato da noi, face_xy mancante o troppo vicino. Mai eccezioni.
    """
    if face_xy is None or P.GRAPHNAV_ARRIVAL_HEADING != 'verso_obiettivo':
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
        print(f"[GRAPHNAV] {label}: arrivo girato verso ({face_xy[0]:.2f}, {face_xy[1]:.2f}), "
              f"imbardata {np.degrees(want):.0f} gradi ({np.degrees(rel):+.0f} rispetto al waypoint).")
        return geometry_pb2.SE2Pose(position=geometry_pb2.Vec2(x=0.0, y=0.0), angle=rel)
    except Exception as e:
        print(f"[GRAPHNAV] {label}: orientamento d'arrivo non calcolato ({e}): uso quello del waypoint.")
        return None


def graphnav_navigate_to(recordingInterface, waypoint_id, robot_state_client, command_client, label,
                          timeout_s=P.GRAPHNAV_TIMEOUT_S, face_xy=None, move='graphnav'):
    """Porta il robot a un waypoint con GraphNav. True solo se ci arriva davvero.

    face_xy: punto VISION verso cui arrivare girati (vedi graphnav_arrival_heading); None =
    orientamento del waypoint. Con GRAPHNAV_ARRIVAL_HEADING = 'libero', se face_xy c'e' il
    robot arriva senza girarsi. move: etichetta del movimento in trajectory.csv."""
    # Prima di muoversi: chiusure d'anello di Spot (a registrazione ferma). Le scorciatoie
    # dalla mappa le aggiunge chi chiama, PRIMA di fermare la registrazione.
    # Tutto cio' che sta qui dentro finisce nella voce 'GraphNav' di [TEMPO]: e' una delle
    # fasi piu' lunghe della missione (il solo ritorno a wp_0 puo' durare minuti).
    _t_gn = time.perf_counter()
    try:
        # 2026-10-26: anche i tratti GraphNav finiscono in trajectory.csv (vedi _TRAJECTORY_COLUMNS).
        with TrajectoryLog(robot_state_client, move):
            return _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client,
                                               command_client, label, timeout_s, face_xy)
    finally:
        timing_add('GraphNav', time.perf_counter() - _t_gn)


# Perche' l'ultimo spostamento GraphNav si e' fermato senza arrivare (per il ritorno a wp_0).
_LAST_GRAPHNAV_FAIL = {'reason': None}


def _stop_graphnav(command_client, reason):
    _LAST_GRAPHNAV_FAIL['reason'] = reason.replace("GraphNav: ", "")
    movements.stop_robot(command_client, reason=reason)


def _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client, command_client,
                                label, timeout_s, face_xy=None):
    _LAST_GRAPHNAV_FAIL['reason'] = None
    client = recordingInterface._graph_nav_client
    tp = _graph_nav_pb2.TravelParams()
    tp.velocity_limit.CopyFrom(movements._velocity_limit(P.GRAPHNAV_MAX_LINEAR_VEL_MPS, P.GRAPHNAV_MAX_ANGULAR_VEL_RPS))
    # Rilevatore di perdita permissivo (vedi LOCALIZZAZIONE DI GRAPHNAV).
    if P.GRAPHNAV_LOST_DETECTOR_STRICTNESS and hasattr(tp, 'lost_detector_strictness'):
        try:
            # dal descrittore: l'enum sta in lost_detection_pb2, che nelle versioni vecchie
            # dell'SDK non c'e'; cosi' funziona ovunque il campo esista.
            enum = tp.DESCRIPTOR.fields_by_name['lost_detector_strictness'].enum_type
            tp.lost_detector_strictness = enum.values_by_name[P.GRAPHNAV_LOST_DETECTOR_STRICTNESS].number
        except Exception:
            pass
    goal_pose = graphnav_arrival_heading(recordingInterface, waypoint_id, face_xy, label)
    if face_xy is not None and P.GRAPHNAV_ARRIVAL_HEADING == 'libero' and hasattr(tp, 'ignore_final_yaw'):
        tp.ignore_final_yaw = True
    print(f"[GRAPHNAV] {label}: avvio (massimo {timeout_s:.0f} s, {P.GRAPHNAV_MAX_LINEAR_VEL_MPS:.1f} m/s).")
    # Prima di partire si guarda lo stato del localizzatore; se e' perso si riaggancia
    # subito, invece di farsi rifiutare il comando.
    relocs = 0
    loc0 = graphnav_log_localization(None, recordingInterface, robot_state_client,
                                     f"prima di '{label}'", force=True)
    if graphnav_is_lost(loc0):
        relocs += 1
        graphnav_relocalize(recordingInterface, robot_state_client, f"prima di '{label}'")
    t0, cmd_id, fb_errors, nav_errors = time.time(), None, 0, 0
    while time.time() - t0 < timeout_s:
        try:
            cmd_id = client.navigate_to(waypoint_id, 1.0, command_id=cmd_id, travel_params=tp,
                                        destination_waypoint_tform_body_goal=goal_pose)
            nav_errors = 0
        except Exception as e:
            if 'RobotLost' in type(e).__name__ and relocs < P.GRAPHNAV_MAX_RELOCALIZATIONS:
                relocs += 1
                print(f"[GRAPHNAV] {label}: GraphNav dice che il robot e' perso: riaggancio "
                      f"({relocs}/{P.GRAPHNAV_MAX_RELOCALIZATIONS}).")
                graphnav_relocalize(recordingInterface, robot_state_client, f"durante '{label}'")
                cmd_id = None
                time.sleep(0.5)
                continue
            nav_errors += 1
            print(f"[GRAPHNAV] {label}: comando rifiutato ({nav_errors}/3): {e}")
            # Un errore RPC transitorio non abbandona lo spostamento: si rinuncia al terzo.
            if nav_errors >= 3:
                _stop_graphnav(command_client, reason="GraphNav: comando rifiutato")
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
            print(f"[GRAPHNAV] {label}: errore nel feedback ({fb_errors}/3): {e}")
            if fb_errors >= 3:
                _stop_graphnav(command_client, reason="GraphNav: feedback non disponibile")
                MISSION_STATS['graphnav_fail'] += 1
                return False
            cmd_id = None      # il comando potrebbe non essere piu' valido: se ne invia uno nuovo
            time.sleep(0.5)
            continue
        if status == _GN.STATUS_REACHED_GOAL:
            print(f"[GRAPHNAV] {label}: arrivato in {time.time() - t0:.0f} s.")
            MISSION_STATS['graphnav_ok'] += 1
            graphnav_log_drift(recordingInterface, robot_state_client, label)
            return True
        if status == _GN.STATUS_LOST and relocs < P.GRAPHNAV_MAX_RELOCALIZATIONS:
            # Riaggancio sul waypoint piu' vicino (NON sulla destinazione, e senza dichiarare
            # l'arrivo: si rimanda il comando e si continua a navigare).
            relocs += 1
            print(f"[GRAPHNAV] {label}: GraphNav ha perso la localizzazione per strada: "
                  f"riaggancio ({relocs}/{P.GRAPHNAV_MAX_RELOCALIZATIONS}).")
            graphnav_log_localization(None, recordingInterface, robot_state_client,
                                      f"perso durante '{label}'", force=True)
            graphnav_relocalize(recordingInterface, robot_state_client, f"durante '{label}'")
            cmd_id = None
            continue
        if status == _GN.STATUS_LOST:
            # Non si forza la localizzazione sulla destinazione (vedi sopra): fallimento.
            print(f"[GRAPHNAV] {label}: GraphNav ha perso la localizzazione. Fermo il robot.")
            _stop_graphnav(command_client, reason="GraphNav: robot perso")
            MISSION_STATS['graphnav_fail'] += 1
            return False
        if status == _GN.STATUS_COMMAND_TIMED_OUT:
            cmd_id = None          # comando scaduto: se ne invia uno nuovo
            continue
        if status in _GRAPHNAV_FAIL_STATUSES:
            print(f"[GRAPHNAV] {label}: interrotto -- {_GRAPHNAV_FAIL_STATUSES[status]}.")
            _stop_graphnav(command_client, reason=f"GraphNav: {_GRAPHNAV_FAIL_STATUSES[status]}")
            MISSION_STATS['graphnav_fail'] += 1
            return False
    print(f"[GRAPHNAV] {label}: tempo massimo di {timeout_s:.0f} s superato. Fermo il robot.")
    _stop_graphnav(command_client, reason="GraphNav: tempo massimo superato")
    MISSION_STATS['graphnav_fail'] += 1
    return False


def graphnav_return_to_start(recordingInterface, robot_state_client, command_client):
    """
    Ritorno a wp_0 a fine missione, con tempo massimo. Tre esiti:
      'arrivato'      GraphNav dice di essere arrivato;
      'vicino'        GraphNav si e' fermato senza arrivare (STUCK, tempo scaduto...) ma il
                      robot e' entro P.GRAPHNAV_RETURN_NEAR_M da wp_0: il ritorno NON e'
                      riuscito, ma il fiducial e la deriva si misurano lo stesso (13:50 del
                      28-10: fermo nell'erba a 0.63 m, misure perse);
      'non arrivato'  tutto il resto.
    La distanza da wp_0 e' quella che dice GraphNav se e' localizzato su wp_0, altrimenti
    quella delle pose VISION (con la deriva dell'odometria dentro).
    """
    try:
        graph = recordingInterface._get_graph(force_refresh=True)
        wp0 = next((w for w in graph.waypoints if w.annotations.name == "wp_0"), None)
    except Exception as e:
        print(f"[GRAPHNAV] Ritorno a wp_0: grafo non disponibile ({e}).")
        return 'non arrivato'
    if wp0 is None:
        print("[GRAPHNAV] Ritorno a wp_0: waypoint wp_0 non trovato nel grafo.")
        return 'non arrivato'
    if graphnav_navigate_to(recordingInterface, wp0.id, robot_state_client, command_client,
                            "ritorno a wp_0", timeout_s=2 * P.GRAPHNAV_TIMEOUT_S):
        MISSION_STATS['wp0_return'] = 'arrivato'
        return 'arrivato'
    why = _LAST_GRAPHNAV_FAIL.get('reason') or 'motivo non registrato'
    d, how = _distance_from_waypoint(recordingInterface, robot_state_client, wp0.id, 'wp_0')
    MISSION_STATS['wp0_return'] = 'non arrivato'
    MISSION_STATS['wp0_return_dist_m'] = None if d is None else round(d, 2)
    if d is not None and d <= P.GRAPHNAV_RETURN_NEAR_M:
        MISSION_STATS['wp0_return'] = 'vicino'
        print(f"[GRAPHNAV] Ritorno a wp_0 NON riuscito ma vicino: fermo a {d:.2f} m da wp_0 "
              f"(motivo: {why}; distanza {how}). Misuro comunque fiducial e deriva.")
        graphnav_log_drift(recordingInterface, robot_state_client, "ritorno a wp_0 (fermo vicino)")
        return 'vicino'
    print(f"[GRAPHNAV] Ritorno a wp_0 NON riuscito: fermo "
          + (f"a {d:.2f} m da wp_0" if d is not None else "a distanza sconosciuta da wp_0")
          + f" (motivo: {why}).")
    return 'non arrivato'


def graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                       min_dist_m=0.40, face_xy=None):
    """
    Torna al waypoint piu' recente che sia almeno min_dist_m DIETRO, con GraphNav.

    Si cerca indietro nella traccia perche' l'ultima briciola puo' essere stata lasciata qui,
    dove il robot e' adesso: tornarci non sposterebbe niente. Se nessuna briciola e' abbastanza
    lontana si usa comunque la piu' vecchia disponibile.

    Restituisce (ok, crumb). ok=True solo se GraphNav dichiara di essere ARRIVATO: in
    _graphnav_navigate_to_inner lo STATUS_LOST e' un fallimento, non si forza la localizzazione
    sulla destinazione (vedi SPOSTAMENTI CON GRAPHNAV).

    La registrazione va FERMATA prima di navigare e ripresa dopo: GraphNav non naviga mentre
    registra. E' la stessa sequenza usata per gli spostamenti fra celle.
    """
    trail = getattr(env, '_graphnav_trail', None) or []
    if not trail:
        return False, None
    try:
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
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
        # Tutte le briciole sono qui attorno (22-10 16:22: il robot stava SU wp_138): GraphNav
        # non sposterebbe il robot. Si ripianifica da dove si e', senza muoversi.
        MISSION_STATS['backtracks_in_place'] += 1
        print(f"[RITORNO] La briciola piu' lontana dietro, {crumb['wp_name']}, e' a {d_crumb:.2f} m "
              f"(meno di {min_dist_m:.2f}): non mi muovo, ripianifico da qui.")
        return True, dict(crumb, in_place=True)
    print(f"[RITORNO] Torno a {crumb['wp_name']}, a {d_crumb:.2f} m dal robot.")
    recording_stopped = False
    graphnav_add_shortcuts(recordingInterface, "prima del ritorno", map_edges=True,
                           target_wp_id=crumb['wp_id'], robot_xy=(rx, ry))
    try:
        recordingInterface.stop_recording()
        recording_stopped = True
    except Exception as e:
        print(f"[RITORNO] Non ho potuto fermare la registrazione ({e}): provo comunque.")
    ok = graphnav_navigate_to(
        recordingInterface, crumb['wp_id'], robot_state_client, command_client,
        f"ritorno al waypoint {crumb['wp_name']} in ({crumb['x']:.2f}, {crumb['y']:.2f})",
        face_xy=face_xy)
    if recording_stopped:
        try:
            recordingInterface.start_recording()
        except Exception as e:
            print(f"[RITORNO] Registrazione non ripresa ({e}): le briciole successive "
                  "potrebbero non essere create.")
    if ok:
        # I tratti percorsi a zampe non descrivono piu' la strada da cui si e' arrivati:
        # GraphNav non li registra, e la ritirata di ripiego non deve usarli.
        env._traveled_arcs = []
        del trail[len(trail) - hop:]        # le briciole oltre questa non servono piu'
    return ok, crumb
