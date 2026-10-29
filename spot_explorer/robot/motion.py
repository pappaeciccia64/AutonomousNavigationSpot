"""
movements.py -- i comandi di movimento di Spot e la sicurezza del movimento.

  relative_move, move_to_world_point_without_turning   un tratto, con limite di velocita',
                                                       andatura e arresto a meta' (should_abort)
  navigate_to                                          un tratto del ciclo di navigazione:
                                                       ruota, poi avanza (con il fronte che
                                                       puo' fermarlo)
  select_mobility_params                               andatura, passo e limiti di velocita'
                                                       dal terreno dell'arco
  check_fall, recover_from_fall, check_and_recover_fall   cadute: riconoscerle, rialzarsi,
                                                       segnare il punto pericoloso
  stop_robot
"""
import time
import numpy as np
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_se2_a_tform_b)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client import robot_command as _robot_command
from bosdyn.api import robot_state_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import geometry_pb2
from google.protobuf import wrappers_pb2

from spot_explorer import config as P
from spot_explorer.perception import grids as spotGrid
from spot_explorer.records import MISSION_STATS


# =========================================================================
# LIMITI DI VELOCITA'.
#
# Ogni comando di movimento imposta un limite: senza, Spot cammina fino al proprio massimo
# (~1.6 m/s), troppo per un terreno esterno anche se in piano. I limiti per livello di
# terreno stanno in spotGrid.GAIT_VEL_* e arrivano tramite select_mobility_params()
# (qui sotto). Questi sono i valori usati quando il chiamante non ne passa (comandi
# isolati, ritirata).
#
# ATTENZIONE: in Spot il limite e' una coppia (min_vel, max_vel). Impostando solo
# max_vel, min_vel resta ZERO e il robot non puo' piu' muoversi all'indietro ne' di
# lato a sinistra: la ritirata smetterebbe di funzionare SENZA dare errori. Per questo
# _build_mobility_params imposta sempre entrambi, simmetrici.
# =========================================================================
DEFAULT_MAX_LINEAR_VEL_MPS = 0.5
DEFAULT_MAX_ANGULAR_VEL_RPS = 0.6
RETREAT_MAX_LINEAR_VEL_MPS = 0.3     # all'indietro il robot vede peggio: si va piu' piano

# =========================================================================
# TEMPO MASSIMO DI UN MOVIMENTO e ERRORI DI RETE.
#
# Se Spot non riesce ad arrivare (corpo fermato dalla sua anticollisione, zampe che
# scivolano su un pendio bagnato) non deve restare li' a provare all'infinito. Tempo
# massimo = MOVE_TIMEOUT_FACTOR x (tempo teorico alla velocita' limite) +
# MOVE_TIMEOUT_MARGIN_S; scaduto, il robot viene fermato e il movimento conta come
# fallito. Lo stesso tempo e' la scadenza del comando dentro Spot.
#
# Un errore di rete durante l'attesa (WiFi in esterno) non deve lasciare il robot a finire
# la traiettoria da solo: si riprova fino a MOVE_MAX_RPC_ERRORS volte di fila, poi si
# ferma il robot e si rilancia l'eccezione.
# =========================================================================
MOVE_TIMEOUT_FACTOR = 3.0
MOVE_TIMEOUT_MARGIN_S = 10.0
MOVE_MAX_RPC_ERRORS = 3


# =========================================================================
# ANTICOLLISIONE DEI PIEDI DI SPOT -- default della missione.
#
# `disable_vision_foot_obstacle_avoidance` spegne il controllo con cui Spot,
# autonomamente e a basso livello, evita di APPOGGIARE UNA ZAMPA su un ostacolo
# che vede. NON riguarda il corpo: `disable_vision_body_obstacle_avoidance`
# resta al suo default (False), quindi Spot continua comunque a rifiutarsi di
# portare il corpo dentro un ostacolo.
#
# PERCHE' RESTA DISATTIVATA (True): la classificazione `no_step` di Spot considera
# l'ERBA un ostacolo -- lo stesso identico motivo per cui questo progetto usa
# `obstacle_distance` invece di `no_step` (vedi il commento in
# spotGrid.create_vtk_obstacle_grid). Con la protezione attiva, su prato Spot
# rifiuta di appoggiare le zampe e la missione si ferma. Il campo d'uso previsto
# di questo sistema e' l'esplorazione in ESTERNO: le prove al chiuso sono una
# contingenza, non il caso d'uso, quindi il default segue l'esterno.
#
# CONSEGUENZA DA TENERE PRESENTE: con questa protezione spenta, l'unica rete di
# sicurezza sotto le zampe e' il nostro fronte sicuro (arcVerification), con l'arresto
# a meta' movimento e la ritirata. Non ci sono controlli indipendenti sotto: per questo
# non si percorrono mai tratti non confermati.
#
# Per una prova al chiuso in cui si voglia la rete in piu', basta passare
# disable_foot_obstacle_avoidance=False alle funzioni qui sotto -- senza toccare
# questo default, che deve restare quello buono per il campo.
# =========================================================================
DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT = True


def _velocity_limit(max_linear_vel, max_angular_vel):
    """SE2VelocityLimit SIMMETRICO: vedi l'avvertenza sopra DEFAULT_MAX_LINEAR_VEL_MPS."""
    v, w = abs(float(max_linear_vel)), abs(float(max_angular_vel))
    return geometry_pb2.SE2VelocityLimit(
        max_vel=geometry_pb2.SE2Velocity(linear=geometry_pb2.Vec2(x=v, y=v), angular=w),
        min_vel=geometry_pb2.SE2Velocity(linear=geometry_pb2.Vec2(x=-v, y=-v), angular=-w),
    )


def _build_mobility_params(locomotion_hint=None, ground_mu_hint=None, swing_height=None,
                           disable_foot_obstacle_avoidance=None,
                           max_linear_vel=None, max_angular_vel=None):
    """
    Costruisce MobilityParams in un solo posto, cosi' i comandi di movimento qui sotto non
    possono divergere fra loro.

    max_linear_vel / max_angular_vel: limiti in m/s e rad/s. None = default di modulo.
    Il limite viene SEMPRE impostato: senza, Spot usa il proprio massimo (~1.6 m/s).
    """
    if disable_foot_obstacle_avoidance is None:
        disable_foot_obstacle_avoidance = DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT
    if max_linear_vel is None:
        max_linear_vel = DEFAULT_MAX_LINEAR_VEL_MPS
    if max_angular_vel is None:
        max_angular_vel = DEFAULT_MAX_ANGULAR_VEL_RPS

    obstacle_params = spot_command_pb2.ObstacleParams(
        disable_vision_foot_obstacle_avoidance=bool(disable_foot_obstacle_avoidance)
    )

    mobility_kwargs = {'obstacle_params': obstacle_params,
                       'vel_limit': _velocity_limit(max_linear_vel, max_angular_vel)}
    if locomotion_hint is not None:
        mobility_kwargs['locomotion_hint'] = locomotion_hint
    if swing_height is not None:
        mobility_kwargs['swing_height'] = swing_height
    if ground_mu_hint is not None:
        mobility_kwargs['terrain_params'] = spot_command_pb2.TerrainParams(
            ground_mu_hint=wrappers_pb2.DoubleValue(value=ground_mu_hint)
        )

    return spot_command_pb2.MobilityParams(**mobility_kwargs)


# =========================================================================
# CADUTE -- rilevazione e raddrizzamento.
#
# Una caduta e' riconosciuta se lo stato del robot riporta un fault di comportamento con
# causa CAUSE_FALL, OPPURE se il corpo e' inclinato oltre FALL_TILT_DEG (vale anche se il
# fault non c'e', per esempio un robot rovesciato a meta' di un raddrizzamento).
#
# Il recupero: cancella i fault cancellabili, raddrizza (selfright), controlla che il corpo
# sia tornato entro UPRIGHT_TILT_DEG, si rimette in piedi, controlla che sia davvero in
# piedi. Al massimo SELF_RIGHT_MAX_ATTEMPTS tentativi.
#
# NON si tenta nulla -- e si solleva RobotFallenError -- se un fault e' NON cancellabile o
# se i motori sono spenti (E-Stop, guasto): in quei casi serve l'operatore, e un robot a
# terra non deve provare a fare altro da solo.
# =========================================================================
FALL_TILT_DEG = 50.0
UPRIGHT_TILT_DEG = 20.0
SELF_RIGHT_TIMEOUT_S = 30.0
STAND_TIMEOUT_S = 10.0
SELF_RIGHT_MAX_ATTEMPTS = 2


class RobotFallenError(RuntimeError):
    """Il robot e' caduto e non si e' potuto rialzarlo da solo: serve l'operatore."""


def _roll_pitch_deg(q):
    roll = np.degrees(np.arctan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)))
    sinp = np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
    return float(roll), float(np.degrees(np.arcsin(sinp)))


def check_fall(robot_state_client):
    """
    Legge lo stato del robot e dice se e' caduto. Restituisce un dict:
      fallen, reason, roll, pitch, x, y, fall_faults (id), clearable (id), unclearable (id),
      motors_on, standing
    """
    state = robot_state_client.get_robot_state()
    BF = robot_state_pb2.BehaviorFault
    faults = list(state.behavior_fault_state.faults)
    fall_faults = [f.behavior_fault_id for f in faults if f.cause == BF.CAUSE_FALL]
    clearable = [f.behavior_fault_id for f in faults if f.status == BF.STATUS_CLEARABLE]
    unclearable = [f.behavior_fault_id for f in faults if f.status == BF.STATUS_UNCLEARABLE]

    vision_tform_body = get_a_tform_b(state.kinematic_state.transforms_snapshot,
                                      VISION_FRAME_NAME, BODY_FRAME_NAME)
    roll, pitch = _roll_pitch_deg(vision_tform_body.rotation)
    tilted = max(abs(roll), abs(pitch)) > FALL_TILT_DEG

    PS = robot_state_pb2.PowerState
    motors_on = state.power_state.motor_power_state == PS.MOTOR_POWER_STATE_ON
    standing = state.behavior_state.state in (robot_state_pb2.BehaviorState.STATE_STANDING,
                                              robot_state_pb2.BehaviorState.STATE_STEPPING)
    reasons = []
    if fall_faults:
        reasons.append(f"fault di caduta ({len(fall_faults)})")
    if tilted:
        reasons.append(f"corpo inclinato (roll {roll:.0f}, pitch {pitch:.0f} gradi)")
    return dict(fallen=bool(fall_faults) or tilted, reason=", ".join(reasons), roll=roll, pitch=pitch,
                x=vision_tform_body.position.x, y=vision_tform_body.position.y,
                fall_faults=fall_faults, clearable=clearable, unclearable=unclearable,
                motors_on=motors_on, standing=standing)


def recover_from_fall(robot_command_client, robot_state_client, status=None):
    """
    Se il robot e' caduto, prova a rialzarlo. Restituisce True se alla fine e' in piedi
    (anche se non era caduto). Solleva RobotFallenError se serve l'operatore.
    """
    st = status or check_fall(robot_state_client)
    if not st['fallen']:
        return True
    print(f"[CADUTA] Robot a terra in ({st['x']:.2f}, {st['y']:.2f}): {st['reason']}.")

    for attempt in range(1, SELF_RIGHT_MAX_ATTEMPTS + 1):
        if st['unclearable']:
            raise RobotFallenError(
                f"fault non cancellabile {st['unclearable']}: il robot non puo' rialzarsi da solo. "
                f"Intervento dell'operatore.")
        if not st['motors_on']:
            raise RobotFallenError(
                "motori spenti dopo la caduta (E-Stop o guasto): il robot non puo' rialzarsi da solo. "
                "Intervento dell'operatore.")
        try:
            for fid in st['clearable']:
                robot_command_client.clear_behavior_fault(fid)
            print(f"[CADUTA] Tentativo {attempt}/{SELF_RIGHT_MAX_ATTEMPTS}: cancellati {len(st['clearable'])} "
                  f"fault, raddrizzamento in corso...")
            _robot_command.blocking_selfright(robot_command_client, timeout_sec=SELF_RIGHT_TIMEOUT_S)
            st = check_fall(robot_state_client)
            if max(abs(st['roll']), abs(st['pitch'])) > UPRIGHT_TILT_DEG:
                print(f"[CADUTA] Dopo il raddrizzamento il corpo e' ancora inclinato "
                      f"(roll {st['roll']:.0f}, pitch {st['pitch']:.0f} gradi).")
                continue
            for fid in st['clearable']:
                robot_command_client.clear_behavior_fault(fid)
            _robot_command.blocking_stand(robot_command_client, timeout_sec=STAND_TIMEOUT_S)
            st = check_fall(robot_state_client)
            if not st['fallen'] and st['standing']:
                print(f"[CADUTA] Rialzato al tentativo {attempt}: di nuovo in piedi in "
                      f"({st['x']:.2f}, {st['y']:.2f}).")
                return True
            print(f"[CADUTA] Dopo il comando di stand il robot non risulta in piedi ({st['reason'] or 'stato'}).")
        except RobotFallenError:
            raise
        except Exception as e:
            print(f"[CADUTA] Tentativo {attempt} fallito: {e}")
            st = check_fall(robot_state_client)
    raise RobotFallenError(f"{SELF_RIGHT_MAX_ATTEMPTS} tentativi di raddrizzamento non riusciti. "
                           f"Intervento dell'operatore.")


def stop_robot(robot_command_client, reason=""):
    """
    Ferma immediatamente il robot, annullando la traiettoria in corso.

    Spot resta in piedi e sotto controllo: non e' un arresto di emergenza (per quello
    c'e' l'E-Stop), e' l'equivalente di lasciare lo stick. Dopo questa chiamata si puo'
    comandare subito un altro movimento -- per esempio una ritirata all'indietro.
    """
    try:
        robot_command_client.robot_command(RobotCommandBuilder.stop_command(),
                                           end_time_secs=time.time() + 5.0)
        if reason:
            print(f"[STOP] Movimento interrotto: {reason}")
        else:
            print("[STOP] Movimento interrotto.")
        return True
    except Exception as e:
        print(f"[STOP] Impossibile fermare il robot: {e}")
        return False


def relative_move(dx, dy, dyaw, frame_name, robot_command_client, robot_state_client,
                   locomotion_hint=None, ground_mu_hint=None, swing_height=None,
                   disable_foot_obstacle_avoidance=None,
                   should_abort=None, poll_period_s=0.2,
                   max_linear_vel=None, max_angular_vel=None):
    """Muove il robot rispetto alla posa attuale.

    dx, dy, dyaw sono nel frame del CORPO: dx>0 avanti, dx<0 indietro, dy laterale,
    dyaw rotazione. Spot cammina in tutte le direzioni, quindi dx negativo con dyaw=0
    e' una marcia indietro vera e propria, senza bisogno di girarsi.

    Args:
        locomotion_hint: spot_command_pb2.LocomotionHint opzionale (es. HINT_CRAWL), l'andatura.
        ground_mu_hint: coefficiente d'attrito atteso opzionale (es. 0.4-0.8), in terrain_params.
        swing_height: spot_command_pb2.SwingHeight opzionale (es. SWING_HEIGHT_HIGH).
        disable_foot_obstacle_avoidance: se True spegne l'anticollisione dei PIEDI di Spot
            (serve su prato, vedi DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT in cima al file).
            None = usa il default della missione.
        should_abort: callable senza argomenti, interrogato a ogni ciclo di feedback.
            Se restituisce un valore vero il movimento viene FERMATO subito e la funzione
            torna (False, distanza_percorsa). E' cosi' che il fronte sicuro, che gira in un
            thread, ferma il robot a meta' movimento. Il MOTIVO dell'arresto lo conosce il
            chiamante, che possiede la callback.
        poll_period_s: intervallo fra due letture del feedback. A ~0.5 m/s, 0.2 s vuol dire
            ~10 cm percorsi fra un controllo e il successivo.
        max_linear_vel, max_angular_vel: limiti di velocita' (m/s, rad/s). None = default
            di modulo (DEFAULT_MAX_*). Vedi l'avvertenza in cima al file sul minimo.

    Returns:
        (success, distance_traveled): success True se l'obiettivo e' raggiunto, False se il
        movimento e' fallito o e' stato interrotto; distance_traveled in metri.
    """
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Posizione di partenza
    initial_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    initial_x = initial_tform_body.x
    initial_y = initial_tform_body.y

    # Obiettivo nel frame richiesto: posa attuale del corpo composta con lo spostamento.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    mobility_params = _build_mobility_params(
        locomotion_hint=locomotion_hint, ground_mu_hint=ground_mu_hint,
        swing_height=swing_height,
        disable_foot_obstacle_avoidance=disable_foot_obstacle_avoidance,
        max_linear_vel=max_linear_vel, max_angular_vel=max_angular_vel)

    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=mobility_params)
    v_lim = abs(float(max_linear_vel if max_linear_vel is not None else DEFAULT_MAX_LINEAR_VEL_MPS)) or 0.1
    w_lim = abs(float(max_angular_vel if max_angular_vel is not None else DEFAULT_MAX_ANGULAR_VEL_RPS)) or 0.1
    expected_s = float(np.hypot(dx, dy)) / v_lim + abs(float(dyaw)) / w_lim
    timeout_s = MOVE_TIMEOUT_FACTOR * expected_s + MOVE_TIMEOUT_MARGIN_S
    t_start = time.time()
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=t_start + timeout_s)

    # Attesa: arrivo, fallimento, tempo scaduto o arresto chiesto dal chiamante.
    rpc_errors = 0
    distance_traveled = 0.0
    while True:
        if time.time() - t_start > timeout_s:
            stop_robot(robot_command_client,
                       reason=f"tempo massimo {timeout_s:.0f} s superato (percorsi {distance_traveled:.2f}m)")
            return False, distance_traveled
        try:
            feedback = robot_command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

            # Posizione attuale
            current_state = robot_state_client.get_robot_state()
            current_transforms = current_state.kinematic_state.transforms_snapshot
            current_tform_body = get_se2_a_tform_b(current_transforms, frame_name, BODY_FRAME_NAME)
            rpc_errors = 0
        except Exception as e:
            rpc_errors += 1
            print(f"[MOVIMENTO] Errore di comunicazione durante il movimento ({rpc_errors}/"
                  f"{MOVE_MAX_RPC_ERRORS}): {e}")
            if rpc_errors >= MOVE_MAX_RPC_ERRORS:
                stop_robot(robot_command_client, reason="errori di comunicazione ripetuti")
                raise
            time.sleep(poll_period_s)
            continue

        distance_traveled = np.sqrt((current_tform_body.x - initial_x) ** 2 +
                                    (current_tform_body.y - initial_y) ** 2)

        # Arresto richiesto dal chiamante (il fronte sicuro ha visto qualcosa sul tratto).
        # Una callback che solleva un'eccezione viene ignorata: il movimento prosegue.
        if should_abort is not None:
            try:
                abort_now = bool(should_abort())
            except Exception as e:
                print(f"[STOP] Errore nella condizione di arresto, la ignoro: {e}")
                abort_now = False
            if abort_now:
                stop_robot(robot_command_client,
                           reason=f"richiesta del chiamante dopo {distance_traveled:.2f}m")
                return False, distance_traveled

        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print(f"[MOVIMENTO] Obiettivo non raggiunto: percorsi {distance_traveled:.2f} m su "
                  f"{float(np.hypot(dx, dy)):.2f} richiesti (stato del comando "
                  f"{mobility_feedback.status}).")
            return False, distance_traveled

        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            # Dopo una rotazione sul posto i pochi cm percorsi sono il centro del corpo che si
            # sposta girando, non un avanzamento: il messaggio lo dice.
            if abs(float(dx)) < 1e-6 and abs(float(dy)) < 1e-6:
                turned = float(np.degrees(np.arctan2(
                    np.sin(current_tform_body.angle - initial_tform_body.angle),
                    np.cos(current_tform_body.angle - initial_tform_body.angle))))
                print(f"[MOVIMENTO] Rotazione completata: {turned:+.0f} gradi su "
                      f"{np.degrees(float(dyaw)):+.0f} richiesti (il centro del corpo si e' "
                      f"spostato di {distance_traveled:.2f} m girando).")
            else:
                print(f"[MOVIMENTO] Arrivato: percorsi {distance_traveled:.2f} m su "
                      f"{float(np.hypot(dx, dy)):.2f} richiesti.")
            return True, distance_traveled

        time.sleep(poll_period_s)


def move_to_world_point_without_turning(target_x, target_y, frame_name,
                                        robot_command_client, robot_state_client,
                                        locomotion_hint=None, ground_mu_hint=None,
                                        swing_height=None, disable_foot_obstacle_avoidance=None,
                                        should_abort=None,
                                        max_linear_vel=None, max_angular_vel=None):
    """
    Raggiunge un punto del mondo MANTENENDO l'orientamento attuale: Spot ci arriva
    camminando all'indietro o di lato, come serve.
    Velocita' di default: RETREAT_MAX_LINEAR_VEL_MPS.

    Serve alla ritirata sui propri passi: i punti da cui si e' passati stanno dietro al
    robot, e in uno spazio stretto non c'e' modo di girarsi per affrontarli di muso.
    Il delta viene convertito dal frame del mondo a quello del corpo, cosi' la rotazione
    comandata resta esattamente zero.
    """
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)

    dx_world = target_x - out_tform_body.x
    dy_world = target_y - out_tform_body.y

    cos_a, sin_a = np.cos(out_tform_body.angle), np.sin(out_tform_body.angle)
    dx_body = dx_world * cos_a + dy_world * sin_a
    dy_body = -dx_world * sin_a + dy_world * cos_a

    if max_linear_vel is None:
        max_linear_vel = RETREAT_MAX_LINEAR_VEL_MPS
    return relative_move(dx_body, dy_body, 0.0, frame_name, robot_command_client, robot_state_client,
                         locomotion_hint=locomotion_hint, ground_mu_hint=ground_mu_hint,
                         swing_height=swing_height,
                         disable_foot_obstacle_avoidance=disable_foot_obstacle_avoidance,
                         should_abort=should_abort,
                         max_linear_vel=max_linear_vel, max_angular_vel=max_angular_vel)


# =========================================================================================
# UN TRATTO DELLA MISSIONE: andatura, movimento con controllo d'arresto, cadute
# =========================================================================================
def check_and_recover_fall(robot_state_client, command_client, global_map, where):
    """
    Controlla se il robot e' caduto e, se si', lo rialza (movements.recover_from_fall) e segna
    il punto come pericoloso nella mappa globale, cosi' il pianificatore lo evita.

    Restituisce None se non c'e' stata nessuna caduta, altrimenti (x, y) del punto in cui il
    robot e' caduto, a robot di nuovo in piedi. Solleva movements.RobotFallenError se non si
    puo' rialzare da solo: la missione si ferma e serve l'operatore.
    """
    try:
        st = check_fall(robot_state_client)
    except Exception as e:
        print(f"[CADUTA] Impossibile leggere lo stato del robot ({where}): {e}")
        return None
    if not st['fallen']:
        return None
    MISSION_STATS['falls'] += 1
    print(f"[CADUTA] Rilevata {where} (caduta n. {MISSION_STATS['falls']} della missione).")
    recover_from_fall(command_client, robot_state_client, status=st)
    MISSION_STATS['self_rights'] += 1
    if MISSION_STATS['falls'] >= P.MAX_FALLS_PER_MISSION:
        raise RobotFallenError(
            f"{MISSION_STATS['falls']} cadute in questa missione: il robot si e' rialzato ma la missione "
            f"si ferma qui. Controllare il terreno e il robot prima di ripartire.")
    if global_map is not None:
        n = global_map.mark_hazard(st['x'], st['y'], P.FALL_HAZARD_RADIUS_M)
        print(f"[CADUTA] Punto della caduta ({st['x']:.2f}, {st['y']:.2f}) segnato come pericoloso "
              f"nella mappa: disco di {P.FALL_HAZARD_RADIUS_M:.1f} m ({n} celle), il pianificatore lo evitera'.")
    return (st['x'], st['y'])


def select_mobility_params(gradient, roughness):
    """
    Andatura su tre livelli, scelta in base alla difficolta' del terreno del SINGOLO tratto
    che si sta per percorrere (vedi prm_graph.PRM.sample_terrain_between_points), invece di
    un profilo fisso per tutta la missione.

    Decide la peggiore fra pendenza e rugosita' (basta superare la soglia su UNA delle due
    per passare al livello piu' duro) -- la stessa logica OR del veto d'ostacolo in
    spotGrid.fuse_obstacle_mask. Soglie, attriti e velocita' stanno in spotGrid.GAIT_*.

    Livelli:
      PLAIN    -- terreno piano e regolare: andatura normale (HINT_AUTO), passo medio,
                  ground_mu_hint alto (buona presa)
      MODERATE -- un po' di pendenza o rugosita': CRAWL, passo alto, ground_mu_hint medio
      HARD     -- terreno difficile ma non vietato (il veto e' una soglia separata e molto
                  piu' alta, in fuse_obstacle_mask): CRAWL, passo alto, ground_mu_hint
                  basso (meno presa, prudenza)
    """
    # Tarato per l'ESTERNO: anche un prato in piano non e' un pavimento, quindi passo
    # almeno medio per tutti i livelli, e velocita' limitate a ogni livello.
    if gradient < spotGrid.GAIT_PLAIN_SLOPE_MAX and roughness < spotGrid.GAIT_PLAIN_ROUGH_MAX:
        tier = "PLAIN"
        params = dict(locomotion_hint=spot_command_pb2.HINT_AUTO,
                      swing_height=spot_command_pb2.SWING_HEIGHT_MEDIUM,
                      ground_mu_hint=spotGrid.GAIT_MU_PLAIN,
                      max_linear_vel=spotGrid.GAIT_VEL_PLAIN,
                      max_angular_vel=spotGrid.GAIT_ANG_PLAIN)
    elif gradient < spotGrid.GAIT_MODERATE_SLOPE_MAX and roughness < spotGrid.GAIT_MODERATE_ROUGH_MAX:
        tier = "MODERATE"
        # CRAWL anche qui, su richiesta dell'utente. Sul prato MODERATE prende ~60% dei
        # passi, quindi il crawl e' l'andatura prevalente e la missione e' piu' lenta;
        # con HINT_AUTO qui MODERATE torna all'andatura normale.
        params = dict(locomotion_hint=spot_command_pb2.HINT_CRAWL,
                      swing_height=spot_command_pb2.SWING_HEIGHT_HIGH,
                      ground_mu_hint=spotGrid.GAIT_MU_MODERATE,
                      max_linear_vel=spotGrid.GAIT_VEL_MODERATE,
                      max_angular_vel=spotGrid.GAIT_ANG_MODERATE)
    else:
        tier = "HARD"
        params = dict(locomotion_hint=spot_command_pb2.HINT_CRAWL,
                      swing_height=spot_command_pb2.SWING_HEIGHT_HIGH,
                      ground_mu_hint=spotGrid.GAIT_MU_HARD,
                      max_linear_vel=spotGrid.GAIT_VEL_HARD,
                      max_angular_vel=spotGrid.GAIT_ANG_HARD)

    print(f"[GAIT] Livello del terreno: {tier} (pendenza={gradient:.3f}, rugosita'={roughness:.3f}) -> "
          f"locomotion_hint={params['locomotion_hint']}, swing_height={params['swing_height']}, "
          f"ground_mu_hint={params['ground_mu_hint']}, "
          f"vel_max={params['max_linear_vel']:.2f} m/s, rot_max={params['max_angular_vel']:.2f} rad/s")
    MISSION_STATS['gait_tier_counts'][tier] += 1
    MISSION_STATS['last_tier'] = tier
    return params


def navigate_to(target_x, target_y, robot_x, robot_y, robot_state_client, command_client, vision_tform_body,
                 should_abort=None, keep_heading=False, **mobility_kwargs):
    """
    Porta il robot al punto (target_x, target_y): prima ruota verso il punto, poi cammina
    dritto (due chiamate a movements.relative_move).

    keep_heading: se True il robot NON ruota prima di muoversi, raggiunge il punto con
        l'orientamento che ha (anche di lato o all'indietro). Il ciclo di navigazione passa
        sempre False: di traverso Spot occupa 1.10 m invece di 0.50 e il fronte pretende
        0.59 m di obstacle_distance invece di 0.30 (vedi ROTAZIONE E ARRETRAMENTO in parametri.py). Il
        parametro resta per i test.

    mobility_kwargs: andatura facoltativa passata a movements.relative_move, es.
        locomotion_hint=spot_command_pb2.HINT_CRAWL,
        ground_mu_hint=0.4,
        swing_height=spot_command_pb2.SWING_HEIGHT_HIGH

    Returns:
        (success: bool, distance_traveled: float, distance_commanded: float)

    movements.relative_move() restituisce una TUPLA (successo, distanza percorsa): qui se ne
    estrae il booleano, perche' una tupla non vuota in Python e' sempre vera e un
    movimento fallito sembrerebbe riuscito.
    """
    dx, dy = target_x - robot_x, target_y - robot_y
    distance = np.sqrt(dx ** 2 + dy ** 2)
    target_yaw = np.arctan2(dy, dx)

    quat = vision_tform_body.rotation
    current_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))
    dyaw = np.arctan2(np.sin(target_yaw - current_yaw), np.cos(target_yaw - current_yaw))

    if keep_heading:
        print(f"[INFO] Raggiungo il punto senza ruotare ({distance:.2f} m, "
              f"{np.degrees(dyaw):.0f} gradi rispetto al muso)...")
        move_success, distance_traveled = move_to_world_point_without_turning(
            target_x, target_y, "vision", command_client, robot_state_client,
            should_abort=should_abort, **mobility_kwargs)
    else:
        rotate_success, _ = relative_move(0, 0, dyaw, "vision", command_client, robot_state_client,
                                                    should_abort=should_abort, **mobility_kwargs)
        if not rotate_success:
            print(f"[FAIL] Rotazione non riuscita: il movimento in avanti non parte.")
            return False, 0.0, distance

        move_success, distance_traveled = relative_move(distance, 0, 0, "vision", command_client,
                                                                   robot_state_client,
                                                                   should_abort=should_abort, **mobility_kwargs)

    # Controllo di plausibilita': relative_move() puo' dichiarare successo (traiettoria
    # "assestata") avendo percorso quasi niente della distanza richiesta -- per esempio con
    # Spot fermo contro un ostacolo fisico, tanto piu' che di default l'anticollisione dei
    # piedi e' spenta (movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT). Un successo con
    # un avanzamento quasi nullo su un movimento vero conta come fallimento.
    MIN_PROGRESS_FRACTION = 0.5
    MIN_PROGRESS_ABS_M = 0.05
    required_progress = max(MIN_PROGRESS_ABS_M, MIN_PROGRESS_FRACTION * distance)

    if move_success and distance > MIN_PROGRESS_ABS_M and distance_traveled < required_progress:
        print(f"[FAIL] Movimento riportato come riuscito ma percorsi solo {distance_traveled:.2f}m "
              f"su {distance:.2f}m richiesti -- trattato come fallimento meccanico reale (probabile "
              f"ostacolo fisico non rilevato).")
        move_success = False

    return move_success, distance_traveled, distance
