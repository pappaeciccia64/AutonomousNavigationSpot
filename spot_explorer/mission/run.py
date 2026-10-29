"""Avvio e chiusura della missione, chiusura di emergenza, riga di comando (vedi run_mission.py)."""
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
from spot_explorer.robot import connection as spotLogInUtils
from spot_explorer.mission import cells as environmentMap
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import safe_frontier
from spot_explorer.robot import battery as batteria

from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.planning import mission_graph                   # reticolo dei nodi della missione

from spot_explorer import config as P
from spot_explorer import REPO_ROOT
from spot_explorer.records import (MISSION_STATS, MISSION_CTX, SegmentLog, install_stdout_tee, remove_stdout_tee, set_mission_z0)
from spot_explorer.mission import cell_attempt
from spot_explorer.mission import exploration
from spot_explorer.mission import summary

# Dove nascono MissionMap/, MissionLogs/ e graph/ (i test la spostano in una cartella temporanea).
DATA_ROOT = REPO_ROOT


def _chiusura_di_emergenza(robot, command_client, recordingInterface, exc):
    """
    Missione interrotta (eccezione o Ctrl+C) con il robot acceso e il lease ancora in mano:
    fermarlo, farlo sedere e spegnere i motori, poi scaricare la mappa GraphNav, che resta
    solo sul robot e all'avvio della missione successiva verrebbe cancellata. Prima il
    programma usciva e basta: robot in piedi finche' l'E-Stop software, non piu' rinnovato,
    toglieva la potenza (~9 s), e mappa persa. Se il robot e' caduto non lo si fa sedere (e'
    gia' a terra). Ogni passo e' tentato a se': mai eccezioni da qui.
    """
    caduto = isinstance(exc, movements.RobotFallenError)
    print(f"\n[EMERGENZA] Missione interrotta ({type(exc).__name__}): "
          + ("il robot e' a terra, " if caduto else "fermo il robot, lo faccio sedere, ")
          + "salvo la mappa GraphNav.")
    tracker = MISSION_CTX.get('tracker')
    if tracker is not None:
        try:
            tracker.clear_path()
        except Exception:
            pass
    if not caduto:
        try:
            movements.stop_robot(command_client, reason="missione interrotta")
            command_client.robot_command(RobotCommandBuilder.synchro_sit_command(),
                                         end_time_secs=time.time() + 20)
            sleep(3)
            robot.power_off(cut_immediately=False)
            print("[EMERGENZA] Robot seduto e motori spenti.")
        except BaseException as e:
            print(f"[EMERGENZA] Non sono riuscito a far sedere il robot ({e!r}): usare il tablet.")
    try:
        recordingInterface.stop_recording()
    except BaseException:
        pass
    try:
        recordingInterface.download_full_graph()
        print("[EMERGENZA] Mappa GraphNav scaricata.")
    except BaseException as e:
        print(f"[EMERGENZA] Mappa GraphNav non scaricata ({e!r}).")


def _cartelle_di_missione():
    """Cartelle di missione (graph/, MissionMap/, MissionLogs/ con la data): (grafo, dati e
    immagini, log)."""
    mission_timestamp = datetime.now().strftime("Mission_%d-%m-%Y_%H-%M-%S")
    base = DATA_ROOT
    graph_folder = os.path.join(base, "graph", mission_timestamp)
    mission_map_folder = os.path.join(base, "MissionMap", mission_timestamp)
    mission_log_folder = os.path.join(base, "MissionLogs", mission_timestamp)
    os.makedirs(graph_folder, exist_ok=True)
    os.makedirs(mission_map_folder, exist_ok=True)
    os.makedirs(mission_log_folder, exist_ok=True)
    return graph_folder, mission_map_folder, mission_log_folder


def _stampa_configurazione():
    """Interruttori e soglie in testa al log, per poterlo leggere sapendo con quali valori e'
    stato prodotto."""
    print("[CONFIG] INTERRUTTORI: " + "  ".join(
        f"{k.split('.')[-1]}={v}" for k, v in summary.mission_switches().items()))
    print(f"[CONFIG] SLOPE_THRESHOLD={slopes.SLOPE_THRESHOLD}  "
          f"LATERAL_SLOPE_COST_MULTIPLIER={slopes.LATERAL_SLOPE_COST_MULTIPLIER}  "
          f"OBSTACLE_THRESHOLD={geometry.OBSTACLE_THRESHOLD}  ROUGH_THRESHOLD={spot_grid.ROUGH_THRESHOLD}  "
          f"NEAR_THRESHOLD_MARGIN_FRACTION={slopes.NEAR_THRESHOLD_MARGIN_FRACTION}")
    print(f"[CONFIG] PROFILO ARCO: base={slopes.SLOPE_BASELINE_M}m  "
          f"fetta_laterale=+/-{slopes.LATERAL_SLICE_HALF_WIDTH_M}m  "
          f"copertura_minima={slopes.MIN_ARC_SAMPLE_FRACTION:.0%}")
    print(f"[CONFIG] GAIT: plain<{gait.GAIT_PLAIN_SLOPE_MAX}/{gait.GAIT_PLAIN_ROUGH_MAX}  "
          f"moderate<{gait.GAIT_MODERATE_SLOPE_MAX}/{gait.GAIT_MODERATE_ROUGH_MAX}  "
          f"mu={gait.GAIT_MU_PLAIN}/{gait.GAIT_MU_MODERATE}/{gait.GAIT_MU_HARD}")
    print(f"[CONFIG] TERRENO: terrain_valid >= {spot_grid.TERRAIN_VALID_MIN} "
          f"(era > 0.0, dipendeva dal segno di un residuo numerico)  "
          f"celle mai scritte: plateau >= {spot_grid.UNWRITTEN_MIN_CELLS} celle, "
          f"almeno {spot_grid.UNWRITTEN_MIN_SPOT_INVALID:.0%} marcate non valide da Spot")
    print(f"[CONFIG] ROTAZIONE: prefiltro {geometry.ROTATION_CLEARANCE_M:.3f} m "
          f"(aria {geometry.ROTATION_CLEARANCE_AIR_M:.2f} m, separata dai "
          f"{geometry.ROBOT_CLEARANCE_AIR_M:.2f} m dell'avanzamento), poi controllo sul "
          f"settore spazzato a passi di {geometry.ROTATION_SWEEP_STEP_DEG:.0f} gradi "
          f"(tolleranza {geometry.ROTATION_NOISE_TOLERANCE_M:.2f} m sul rumore); "
          f"MARCIA DI TRAVERSO DISATTIVATA")
    print(f"[CONFIG] SE NON PUO' RUOTARE: 1) ventaglio nel cono +/-{P.CONE_SWEEP_MAX_DEG:.0f} gradi "
          f"a passi di {P.CONE_SWEEP_STEP_DEG:.0f}, si va nella direzione piu' libera fra quelle "
          f"che avvicinano di almeno {P.CONE_MIN_GAIN_M:.2f} m; 2) arretramento dritto fino a "
          f"{P.ROTATION_RETREAT_M:.2f} m SOLO se da li' la rotazione diventa possibile, "
          f"max {P.MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi; 3) si scarta il tratto")
    print(f"[CONFIG] TRAPPOLA: ventaglio a 360 gradi a passi di {P.ESCAPE_FAN_STEP_DEG:.0f} "
          f"({int(360/P.ESCAPE_FAN_STEP_DEG)} direzioni); una direzione vale se concede "
          f"{P.ESCAPE_FAN_MIN_ADVANCE_M:.2f} m E il robot puo' girarsi per puntarla. Zero "
          f"direzioni -> si ripercorre il tratto da cui si e' venuti SUBITO, senza "
          f"scartare archi. Controllo sulla posa d'arrivo per i passi sotto "
          f"{P.ARRIVAL_EXIT_CHECK_MAX_STEP_M:.2f} m o con obstacle_distance sotto "
          f"{P.ARRIVAL_EXIT_CHECK_OD_M:.2f} m, max {P.MAX_ARRIVAL_REFUSALS} rifiuti. "
          f"Scarto da [SUBITO] e da [ARRIVO]: NON permanente")
    print(f"[CONFIG] RITORNI: briciola GraphNav ogni "
          f"{P.GRAPHNAV_WAYPOINT_MIN_SPACING_M:.2f} m, ritorno con GraphNav fino a "
          f"{P.GRAPHNAV_MAX_BACKTRACK_HOPS} waypoint indietro riprovando la "
          f"pianificazione a ogni tappa, poi si rimanda la cella; ritirata a zampe "
          f"solo come ultima rete. Nodi PRM scartati sotto "
          f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero, solo dove visti. "
          f"Deroga di {prm_graph.STOP_NODE_MAP_IGNORE_M:.2f} m alla mappa globale "
          f"attorno al robot: ora solo dopo una caduta")
    print(f"[CONFIG] OSTACOLI: mediana su {safe_frontier.OBSTACLE_MEDIAN_FRAMES} scansioni "
          f"+ isteresi {safe_frontier.OBSTACLE_HYSTERESIS_M:.2f} m attorno a "
          f"{geometry.FRONTIER_CLEARANCE_M:.2f} m (rumore misurato 1.5 cm tipici, 7.3 di punta); "
          f"celle-ostacolo escluse dal campionamento della pendenza d'arco")
    print(f"[CONFIG] BATTERIA: rientro a wp_0 sotto {P.BATTERY_RETURN_PCT:.0f}% "
          f"({P.BATTERY_RETURN_CONFIRM} letture di fila; 0 = spento), lettura al massimo ogni "
          f"{P.BATTERY_SAMPLE_S:.0f} s, dati in battery.csv")
    print(f"[CONFIG] OBIETTIVO: tolleranza d'arrivo {P.GOAL_REACHED_TOLERANCE_M:.2f} m "
          f"(= meta' corpo + margine del fronte), riscelta del punto fino a "
          f"{P.MAX_BLOCK_REPLANS} volte se risulta irraggiungibile; ripianificazione alla "
          f"PRIMA cella bloccante, fino a {P.MAX_EARLY_REPLANS} volte per tentativo")
    print(f"[CONFIG] SCELTA OBIETTIVO: solo punti raggiungibili dal robot nel grafo (se ce "
          f"ne sono), spazio libero preferito {P.TARGET_CLEARANCE_M:.2f} m (minimo "
          f"{geometry.PLAN_CLEARANCE_M:.2f}), almeno {P.TARGET_CELL_BORDER_M:.2f} m dal bordo "
          f"della cella, obiettivi rifiutati dal fronte esclusi per la missione; arrivo "
          f"valido anche col robot fuori dalla cella fino a {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m")


def _griglia_di_missione(options):
    """La griglia di missione: EnvironmentMap con colonne davanti e righe a sinistra.

    Default P.MISSION_GRID_ROWS x P.MISSION_GRID_COLS celle da P.MISSION_CELL_SIZE_M. Si
    cambiano SENZA toccare il codice, in ordine di precedenza:
      1. argomenti:  python3 easy_walk.py --rows 2 --cols 4 --cell 5
      2. variabile:  SPOT_MISSION_GRID="righe,colonne,lato_cella"   es. "2,3,3"
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
            print(f"[CONFIG] SPOT_MISSION_GRID='{_env_grid}' non valido (atteso 'righe,colonne,lato'): "
                  f"uso {grid_rows},{grid_cols},{grid_cell:g}.")
    _cli = (getattr(options, 'grid_rows', None), getattr(options, 'grid_cols', None),
            getattr(options, 'grid_cell', None))
    if any(v is not None for v in _cli):
        grid_rows = int(_cli[0]) if _cli[0] is not None else grid_rows
        grid_cols = int(_cli[1]) if _cli[1] is not None else grid_cols
        grid_cell = float(_cli[2]) if _cli[2] is not None else grid_cell
        _grid_source = "argomenti --rows/--cols/--cell"
    print(f"[CONFIG] Griglia di missione: {grid_rows} x {grid_cols} celle da {grid_cell:.1f} m "
          f"({grid_rows * grid_cell:.0f} m a sinistra x {grid_cols * grid_cell:.0f} m davanti; "
          f"fonte: {_grid_source})")
    print(f"[CONFIG] Reticolo: passo richiesto "
          f"{'suggerito' if P.MISSION_LATTICE_SPACING_M is None else '%.3f m' % P.MISSION_LATTICE_SPACING_M}, "
          f"anelli {P.MISSION_LATTICE_RINGS if P.MISSION_LATTICE_RINGS is not None else 'tutti'}")
    # test_replay legge metodo e archi da questa riga; distribuzione, densita' e seme dalla
    # riga [GRAFO] Nodi casuali, scritta quando il grafo e' costruito.
    print(f"[CONFIG] Grafo: nodi {P.MISSION_GRAPH_SAMPLING}"
          + (f" ({P.MISSION_RANDOM_DISTRIBUTION}, {P.MISSION_RANDOM_DENSITY:g} nodi/m2, seme "
             f"{'a caso' if P.MISSION_RANDOM_SEED is None else P.MISSION_RANDOM_SEED})"
             if P.MISSION_GRAPH_SAMPLING == 'casuale' else "")
          + f"; archi da {P.PRM_MIN_EDGE_M:g} a {P.PRM_MAX_EDGE_M:g} m")
    return environmentMap.EnvironmentMap(rows=grid_rows, cols=grid_cols, cell_size=grid_cell)


def _chiudi_missione(robot, robot_state_client, command_client, recordingInterface):
    """Dopo il riepilogo: chiusura della mappa GraphNav, ritorno a wp_0, seduta, spegnimento,
    scaricamento della mappa. Non entra in nessuna voce di [TEMPO]: si misura a parte e si
    stampa in chiusura."""
    _t_close = {}
    _t = time.perf_counter()
    try:
        _wp0_id = next((w.id for w in recordingInterface._get_graph(force_refresh=True).waypoints
                        if w.annotations.name == "wp_0"), None)
        _xy_end = spotUtils.getPosition(robot_state_client)[:2]
    except Exception:
        _wp0_id, _xy_end = None, None
    map_shortcuts.graphnav_add_shortcuts(recordingInterface, "prima del ritorno a wp_0",
                                         map_edges=True, target_wp_id=_wp0_id, robot_xy=_xy_end)
    recordingInterface.auto_close_loops(True, False)
    recordingInterface.stop_recording()
    # Chiusura d'anello di Spot una volta sola, PRIMA dell'ancoraggio: i suoi archi,
    # verificati sulle scansioni, correggono la deriva della mappa scaricata e accorciano il
    # ritorno (vedi GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
    map_shortcuts.graphnav_add_shortcuts(recordingInterface, "chiusura della mappa", map_edges=False,
                                         loop_closure_s=P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S)
    recordingInterface.optimize_anchoring()
    _t_close['chiusura mappa e ancoraggio'] = time.perf_counter() - _t

    _t = time.perf_counter()
    monitor = MISSION_CTX.get('batteria')
    if monitor is not None:
        monitor.controlla("inizio del ritorno a wp_0")
    _pct0 = MISSION_STATS.get('battery_last_pct')
    esito_wp0 = navigation.graphnav_return_to_start(recordingInterface, robot_state_client, command_client)
    if esito_wp0 in ('arrivato', 'vicino'):
        # Anche fermo vicino a wp_0 il tag si cerca: e' la misura della deriva di tutta la missione.
        misura = apriltags.graphnav_check_start_fiducial(robot, robot_state_client)
        # Arrivato secondo GraphNav ma lontano secondo il tag: correzione (vedi
        # WP0_FIDUCIAL_CORRECTION). Fermo vicino (STUCK) no: la strada e' bloccata.
        if esito_wp0 == 'arrivato' and misura and misura.get('offset') is not None:
            apriltags.graphnav_correct_wp0_with_fiducial(robot, recordingInterface, robot_state_client,
                                                             command_client, misura['offset'])
    if esito_wp0 != 'arrivato':
        print("[GRAPHNAV] Ritorno a wp_0 non riuscito: il robot si siede dove si trova.")
    _t_close['ritorno a wp_0'] = time.perf_counter() - _t
    if monitor is not None:
        monitor.controlla("fine del ritorno a wp_0")
        _pct1 = MISSION_STATS.get('battery_last_pct')
        if _pct0 is not None and _pct1 is not None:
            # Quanto costa il ritorno: serve a tarare BATTERY_RETURN_PCT.
            MISSION_STATS['battery_wp0_return_pct'] = round(_pct0 - _pct1, 1)
            print(f"[BATTERIA] Ritorno a wp_0: da {_pct0:.0f}% a {_pct1:.0f}% "
                  f"({_pct0 - _pct1:.1f} punti in {_t_close['ritorno a wp_0']:.0f} s)")

    _t = time.perf_counter()
    command_client.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
    sleep(3)
    robot.power_off(cut_immediately=False)
    _t_close['seduta e spegnimento'] = time.perf_counter() - _t

    _t = time.perf_counter()
    recordingInterface.download_full_graph()
    _t_close['scaricamento della mappa'] = time.perf_counter() - _t

    print(f"[TEMPO] Chiusura della missione, {sum(_t_close.values()):.1f} s in tutto:")
    for k, v in sorted(_t_close.items(), key=lambda kv: -kv[1]):
        print(f"  {k:32s} {v:6.1f} s")


def easy_walk(options):
    """La missione intera: avvio del robot e di GraphNav, grafo, esplorazione cella per cella
    (Missione), scansione finale, riepilogo, ritorno alla partenza e spegnimento."""
    robot, lease_client, robot_state_client, client_metadata = spotLogInUtils.setLogInfo(options)
    estop = spotLogInUtils.SimpleEstop(robot, options.name + "_estop")

    local_grid = spot_grid.LocalGrid(robot)
    MISSION_CTX['local_grid'] = local_grid
    global_grid = global_maps.GlobalGrid()
    MISSION_CTX['global_grid'] = global_grid   # per graphnav_add_shortcuts

    recordingInterface = map_recording.RecordingInterface(robot, options.download_filepath, client_metadata)
    recordingInterface.stop_recording()
    recordingInterface.clear_map()

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Cartelle e log creati subito, cosi' la copia dell'output su file (mission_log.txt)
        # cattura anche tutto l'avvio e la costruzione del grafo.
        graph_folder, mission_folder, mission_log_folder = _cartelle_di_missione()
        install_stdout_tee(os.path.join(mission_log_folder, "mission_log.txt"))
        try:
            MISSION_CTX['log_folder'] = mission_log_folder
            MISSION_CTX['segment_log'] = SegmentLog(os.path.join(mission_log_folder, "segments.csv"))
            MISSION_CTX['summary_done'] = False
            print(f"[LOG] Cartella log: {mission_log_folder}  |  dati e immagini: {mission_folder}")
            _stampa_configurazione()
        except Exception as e:
            print(f"[LOG] Impossibile inizializzare il CSV dei segmenti: {e}")

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.logger.info('Accensione del robot...')
        robot.power_on()
        assert robot.is_powered_on(), 'Accensione del robot non riuscita.'
        robot.logger.info('Robot acceso.')
        try:
            blocking_stand(command_client)

            # Batteria: prima lettura, poi a ogni giro del ciclo (vedi batteria.py).
            MISSION_CTX['batteria'] = batteria.MonitorBatteria(robot_state_client)
            MISSION_CTX['batteria'].controlla("avvio")

            # Griglie e mappe locali vengono tutte dalle telecamere di Spot (nessun Velodyne).
            recordingInterface.clear_map()
            recordingInterface.start_recording()

            # Fiducial di partenza (vedi START_FIDUCIAL_ID): cercato PRIMA di creare wp_0, cosi'
            # entra nella sua istantanea e la localizzazione sul fiducial ha qualcosa da trovare.
            fiducials = apriltags.find_fiducials(robot, robot_state_client,
                                                     wait_s=P.START_FIDUCIAL_WAIT_S,
                                                     fiducial_id=P.START_FIDUCIAL_ID)
            MISSION_CTX['start_fiducial'] = fiducials[0] if fiducials else None
            if fiducials:
                print(f"[FIDUCIAL] Fiducial di partenza: {apriltags.describe_fiducial(fiducials[0])}"
                      + (f"; ne vedo altri {len(fiducials) - 1}: "
                         + ", ".join(str(f['id']) for f in fiducials[1:]) if len(fiducials) > 1 else ""))
            else:
                print(f"[FIDUCIAL] Nessun fiducial "
                      f"{'' if P.START_FIDUCIAL_ID is None else f'con ID {P.START_FIDUCIAL_ID} '}"
                      f"visto in {P.START_FIDUCIAL_WAIT_S:.0f} s da nessuna telecamera: si parte senza. "
                      f"Spot riconosce solo gli AprilTag 36h11 (i fiducial di Boston Dynamics).")

            start_row, start_col = 0, 0
            recordingInterface.create_default_waypoint(cell_row=start_row, cell_col=start_col)
            if fiducials:
                recordingInterface.initialize_with_fiducial(robot_state_client, fiducials[0]['id'])

            env = _griglia_di_missione(options)

            # --- Andatura per tutta la missione ---
            # Vuoto: andatura, altezza del passo e ground_mu_hint si scelgono TRATTO PER TRATTO in
            # base al terreno (vedi movements.select_mobility_params). Per imporre un profilo fisso
            # (per esempio in prova) basta un dict non vuoto qui: TentativoCella lo usa per tutti i
            # tratti e salta la scelta automatica. Esempio:
            #   mobility_kwargs = dict(
            #       locomotion_hint=spot_command_pb2.HINT_CRAWL,
            #       ground_mu_hint=0.4,
            #       swing_height=spot_command_pb2.SWING_HEIGHT_HIGH
            #   )
            mobility_kwargs = {}

            # --- Posa di partenza, PRIMA di costruire il grafo: il reticolo e' riferito a questa ---
            x_boot, y_boot, z_boot, quat_boot = spotUtils.getPosition(robot_state_client)
            yaw_boot = np.arctan2(2.0 * (quat_boot.w * quat_boot.z + quat_boot.x * quat_boot.y),
                                  1.0 - 2.0 * (quat_boot.y ** 2 + quat_boot.z ** 2))
            env.set_origin(x_boot, y_boot, yaw_boot, start_row=start_row, start_col=start_col)
            set_mission_z0(robot_state_client, z_boot)

            # Nodi del PRM: il grafo della missione (vedi mission_graph.build_mission_prm). Il punto
            # di accensione e i centri cella sono nodi del grafo, con entrambi i metodi. Gli ARCHI
            # non si creano qui: non c'e' ancora nessun dato di terreno ne' di occupazione. Li crea
            # TentativoCella alla prima scansione (vedi mission_graph.rebuild_prm_edges).
            prm, gb_sampler, mission_lattice = mission_graph.build_mission_prm(env)
            MISSION_CTX['prm'] = prm  # per il riepilogo di fine missione (anche se interrotta)

            verification_tracker = safe_frontier.ArcVerificationTracker(robot)
            MISSION_CTX['tracker'] = verification_tracker
            verification_tracker.start()

            recordingInterface.set_download_filepath(graph_folder)

            missione = exploration.Missione(robot_state_client, command_client, recordingInterface, local_grid,
                                global_grid, env, prm, gb_sampler, verification_tracker,
                                mission_folder, mobility_kwargs)
            missione.esplora()

            env.print_map()
            if cell_attempt._batteria_scarica():
                print("[BATTERIA] Missione chiusa prima di finire l'esplorazione: la chiusura e' "
                      "quella normale (scansione finale, riepilogo, ritorno a wp_0, mappa scaricata).")
            x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
            final_row, final_col = env.get_cell_from_world(x_final, y_final)
            recordingInterface.create_default_waypoint(cell_row=final_row, cell_col=final_col)

            missione.scansione_finale()

            # RIEPILOGO DI MISSIONE -- solo diagnostica, va letto dopo il volo.
            summary._emit_mission_summary_once()

            _chiudi_missione(robot, robot_state_client, command_client, recordingInterface)
            estop.stop()
        except BaseException as exc:
            _chiusura_di_emergenza(robot, command_client, recordingInterface, exc)
            raise


# =========================================================================
# RIGA DI COMANDO
# =========================================================================


def _parse_args(argv=None):
    """
    Argomenti da riga di comando. Tutti facoltativi: senza argomenti la missione usa i
    valori di parametri.py (MISSION_GRID_ROWS, MISSION_GRID_COLS, MISSION_CELL_SIZE_M,
    MISSION_LATTICE_SPACING_M, MISSION_LATTICE_RINGS, MISSION_GRAPH_SAMPLING...).
    """
    import argparse
    p = argparse.ArgumentParser(description="Missione di esplorazione autonoma con Spot")
    p.add_argument('--rows', type=int, default=None,
                   help=f"righe di celle, a sinistra del robot (default {P.MISSION_GRID_ROWS})")
    p.add_argument('--cols', type=int, default=None,
                   help=f"colonne di celle, davanti al robot (default {P.MISSION_GRID_COLS})")
    p.add_argument('--cell', type=float, default=None,
                   help=f"lato della cella in m (default {P.MISSION_CELL_SIZE_M:g})")
    p.add_argument('--spacing', type=float, default=None,
                   help=f"passo fra i nodi del reticolo in m, arrotondato a cell/n; gli archi "
                        f"restano >= {P.PRM_MIN_EDGE_M:g} m (default {P.MISSION_LATTICE_SPACING_M}; "
                        f"0 = suggerito)")
    p.add_argument('--rings', default=None,
                   help=f"anelli di vicini collegati, o 'tutti' (default {P.MISSION_LATTICE_RINGS})")
    p.add_argument('--grafo', choices=mission_graph.GRAPH_SAMPLINGS, default=None,
                   help=f"come si generano i nodi (default {P.MISSION_GRAPH_SAMPLING})")
    p.add_argument('--distribuzione', choices=mission_graph.RANDOM_DISTRIBUTIONS, default=None,
                   help=f"nodi casuali: distribuzione (default {P.MISSION_RANDOM_DISTRIBUTION})")
    p.add_argument('--densita', type=float, default=None,
                   help=f"nodi casuali: nodi per m2 (default {P.MISSION_RANDOM_DENSITY:g})")
    p.add_argument('--seme', type=int, default=None,
                   help="nodi casuali: seme, per ripetere un grafo (default: estratto a caso, "
                        "scritto nel log)")
    p.add_argument('--min-edge', type=float, default=None,
                   help=f"lunghezza minima degli archi in m (default {P.PRM_MIN_EDGE_M:g})")
    p.add_argument('--max-edge', type=float, default=None,
                   help=f"lunghezza massima degli archi in m (default {P.PRM_MAX_EDGE_M:g})")
    a = p.parse_args(argv)
    if a.rows is not None and a.rows < 1 or a.cols is not None and a.cols < 1:
        p.error("--rows e --cols devono essere almeno 1")
    if a.cell is not None and a.cell <= 0:
        p.error("--cell deve essere positivo")
    if a.densita is not None and a.densita <= 0:
        p.error("--densita deve essere positiva")
    lo = P.PRM_MIN_EDGE_M if a.min_edge is None else a.min_edge
    hi = P.PRM_MAX_EDGE_M if a.max_edge is None else a.max_edge
    if lo <= 0 or hi < lo:
        p.error("serve 0 < --min-edge <= --max-edge")
    return a


def main():
    args = _parse_args()
    if args.grafo is not None:
        P.MISSION_GRAPH_SAMPLING = args.grafo
    if args.distribuzione is not None:
        P.MISSION_RANDOM_DISTRIBUTION = args.distribuzione
    if args.densita is not None:
        P.MISSION_RANDOM_DENSITY = float(args.densita)
    if args.seme is not None:
        P.MISSION_RANDOM_SEED = int(args.seme)
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
        print(f"[CADUTA] MISSIONE INTERROTTA: {exc}")
        print("[CADUTA] Il robot e' a terra e NON ha tentato di tornare alla base.")
        print("[CADUTA] Prendere il controllo dal tablet, verificare il robot, poi rialzarlo a mano.")
        print("=" * 70)
        return False
    except Exception as exc:
        logger = bosdyn.client.util.get_logger()
        logger.error('La missione si e\' interrotta per un\'eccezione: %r', exc)
        try:
            import traceback
            print(f"[LOG] La missione e' terminata con un'eccezione: {exc!r}")
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
