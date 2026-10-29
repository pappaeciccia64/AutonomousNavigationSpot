"""
REGISTRO DELLA MISSIONE: statistiche, log e dati salvati su disco.

Niente di quello che c'e' qui decide il comportamento del robot: serve a capire dopo cosa e'
successo (mission_report.py, mission_viewer.py, test_replay.py).

  MISSION_STATS      contatori e tempi per fase, riepilogati a fine missione
  MISSION_CTX        oggetti della missione in corso per i log che non li ricevono come
                     argomento (PRM, cartella dei log, quota di riferimento)
  install_stdout_tee copia di tutto l'output in mission_log.txt
  BodyTiltRecorder   campiona posizione e inclinazione del corpo durante un movimento
  SegmentLog, append_csv, append_trajectory, TrajectoryLog, log_segment
                     i CSV della missione (segments.csv, trajectory.csv, drift.csv, ...)
  save_npz_async     salvataggio dei .npz su un thread di appoggio, senza fermare il ciclo
"""
import csv
import os
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import numpy as np
from bosdyn.client.frame_helpers import GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b

from spot_explorer.robot import connection as spotUtils

# Un thread di appoggio per salvare i dati su disco senza fermare il ciclo di navigazione.
VIS_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vis_worker")


# ==================================================================
# STATISTICHE DI MISSIONE -- solo diagnostica/log, mai lette per decidere il
# comportamento del robot. Mai azzerato: una missione per processo (i test che ne fanno
# girare piu' di una nello stesso processo vedono i conteggi sommati).
# I contatori sul veto/fonte-pendenza vivono invece sull'istanza PRM stessa
# (vedi prm_graph.PRM.print_mission_summary), perché il PRM persiste per tutta
# la missione mentre qui teniamo solo cio' che non ha un posto naturale li'.
# ==================================================================
MISSION_STATS = {
    'gait_tier_counts': {'PLAIN': 0, 'MODERATE': 0, 'HARD': 0},
    'replan_count': 0,
    'near_threshold_saves': 0,
    'segments_logged': 0,
    'last_tier': None,   # tier dell'ultimo segmento, letto per la riga del CSV
    'time_s': {},        # secondi spesi per fase, vedi timing_add / [TEMPO]
    'scan_bundles': 0,
    'frontier_blocks': 0,
    'frontier_aborts': 0,
    'rotation_refusals': 0,   # rotazioni rifiutate dal controllo sul settore
    'straight_retreats': 0,   # arretramenti dritti dopo un rifiuto di rotazione
    'goal_repicks': 0,        # punti obiettivo riscelti dentro il ciclo
    'goals_refused': 0,       # obiettivi esclusi per il resto della missione (_refuse_goal)
    'goals_unreachable': 0,   # scelte d'obiettivo senza nessun punto raggiungibile nel grafo
    'arrivals_near_goal': 0,  # arrivi accettati con il robot appena fuori dalla cella
    'backtracks_in_place': 0, # ritorni senza movimento: nessuna briciola abbastanza lontana
    'sdk_loop_calls': 0,      # chiusure d'anello di Spot riuscite (con risposta)
    'sdk_loop_s': 0.0,        # secondi spesi in quelle chiamate
    'sdk_loop_edges': 0,      # archi creati da Spot
    'sdk_loop_partial': 0,    # chiamate finite allo scadere del limite (risultato parziale)
    'sdk_loop_failures': 0,   # tentativi di chiusura d'anello falliti (errore, nel log)
    'drift_samples': 0,       # misure della deriva dell'odometria (graphnav_log_drift)
    'drift_max_m': 0.0,       # deriva massima misurata
    'drift_last_m': 0.0,      # ultima misura (di solito all'arrivo su wp_0)
    'graphnav_trips_avoided': 0,  # spostamenti GraphNav evitati: dal waypoint il grafo non arriva alla cella
    'cone_moves': 0,          # avanzamenti dentro il cono di rotazione possibile
    'retreats_suppressed': 0,  # arretramenti evitati perche' inutili
    'early_replans': 0,       # ripianificazioni alla prima cella bloccante
    'traps_detected': 0,      # trappole riconosciute dal ventaglio a 360 gradi
    'arrival_refusals': 0,    # passi rifiutati perche' la posa d'arrivo non ha uscite
    'graphnav_backtracks': 0,     # ritorni di un waypoint riusciti con GraphNav
    'graphnav_backtrack_fail': 0, # ritorni con GraphNav falliti (ripiego a zampe)
    'nodes_pruned': 0,            # nodi PRM scartati perche' dentro un ostacolo
    'idle_loops': 0,              # giri di ciclo a vuoto (guardia anti-stallo)
    'multinode_cut': 0,           # sequenze multinodo fermate a un nodo intermedio
    'multinode_moves': 0,         # movimenti che hanno attraversato piu' nodi
    'multinode_nodes': 0,         # nodi attraversati da quei movimenti
    'prm_refresh': 0,             # refresh degli archi effettuati
    'prm_refresh_skipped': 0,     # refresh saltati (nessun evento, robot fermo)
    'rotation_maneuvers': 0,
    'shortcuts': 0,
    'falls': 0,
    'self_rights': 0,
    'graphnav_ok': 0,
    'graphnav_fail': 0,
}


# Contesto della missione in corso, per i log diagnostici che non hanno modo di ricevere
# questi oggetti come argomento (attempt_enter_cell_from_position e' chiamata da piu' punti
# e non vogliamo cambiarne la firma). Riempito in easy_walk(), letto da log_segment() e
# easy_walk._emit_mission_summary_once(). Solo diagnostica: nessuna decisione del robot lo legge.
MISSION_CTX = {
    'prm': None,
    'log_folder': None,
    'segment_log': None,
    'summary_done': False,
    # Quota del suolo all'AVVIO DELLA MISSIONE nel frame VISION. Spot misura le
    # quote rispetto a dove e' stato ACCESO: se lo si porta alla zona di missione facendo
    # delle scale, tutte le quote risultano spostate (di metri). x, y e direzione
    # sono gia' riferiti all'avvio missione (env.set_origin con la posa di partenza); per la
    # quota si salva questo valore in tutti i dati e nelle mappe, e il viewer mostra le quote
    # rispetto ad esso. I CALCOLI non ne dipendono: usano solo differenze di quota e il
    # suolo stimato sotto il robot a ogni scansione (vedi spotGrid.GROUND_HEIGHT_TOLERANCE_M).
    'mission_z0': None,
    # Fiducial visto alla partenza (navGraphUtils.find_fiducials): ID e posizione VISION, per
    # rivederlo al ritorno su wp_0 (graphnav_check_start_fiducial). None = nessuno.
    'start_fiducial': None,
}


_SEGMENT_COLUMNS = [
    'timestamp', 'iteration', 'cell_row', 'cell_col', 'node_from', 'node_to',
    'x_from', 'y_from', 'x_to', 'y_to', 'dist_m',
    'slope_source', 'long_slope', 'lat_slope',
    'roughness',
    'cost_dist', 'cost_long', 'cost_lat', 'cost_rough', 'cost_total',
    'tier', 'gait_override', 'success', 'dist_traveled_m', 'dist_commanded_m', 'duration_s',
    'roll_start_deg', 'pitch_start_deg', 'roll_end_deg', 'pitch_end_deg',
    'roll_peak_abs_deg', 'pitch_peak_abs_deg', 'tilt_samples', 'tilt_errors',
]


_CSV_LOCK = threading.Lock()


# Colonne di trajectory.csv. 'move' (2026-10-26) dice che tipo di movimento ha prodotto il
# campione: 'prm' (tratto del ciclo di navigazione), 'graphnav', 'cono', 'arretramento',
# 'ritirata'. Fino al 2026-10-26 i movimenti diversi da 'prm' NON finivano nel file: il
# visualizzatore univa con una retta l'ultimo punto prima di un ritorno GraphNav e il primo
# dopo, una linea che tagliava gli ostacoli (missione del 22-10, 16:00 circa).
_TRAJECTORY_COLUMNS = ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                       'x', 'y', 'yaw_deg', 'roll_deg', 'pitch_deg', 'target_x', 'target_y', 'move']


def timing_add(phase, seconds):
    """Accumula il tempo speso in una fase (riepilogo [TEMPO] a fine missione)."""
    MISSION_STATS['time_s'][phase] = MISSION_STATS['time_s'].get(phase, 0.0) + float(seconds)


def save_npz_async(path, **arrays):
    """Salvataggio compresso nel thread di servizio: il ciclo principale non aspetta il disco."""
    def _job():
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            np.savez_compressed(path, **arrays)
        except Exception as e:
            print(f"[DATA SAVE] Impossibile salvare {os.path.basename(path)}: {e}")
    VIS_EXECUTOR.submit(_job)


def mission_z0():
    """Quota di riferimento della missione (NaN finche' non e' fissata), per i file salvati."""
    z0 = MISSION_CTX.get('mission_z0')
    return np.nan if z0 is None else float(z0)


def set_mission_z0(robot_state_client, z_body):
    """
    Fissa la quota di riferimento all'avvio missione: il piano del suolo stimato da Spot
    sotto i piedi (frame 'gpe'); se non disponibile, la quota del corpo meno l'altezza
    nominale in piedi. Stampa di quanto la quota di missione differisce da quella di accensione.
    """
    z0, source = None, "piano del suolo di Spot"
    try:
        snap = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        z0 = float(get_a_tform_b(snap, VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME).position.z)
    except Exception as e:
        print(f"[QUOTE] Piano del suolo non disponibile ({e}): uso la quota del corpo.")
    if z0 is None or not np.isfinite(z0):
        z0, source = float(z_body) - 0.5, "quota del corpo - 0.5 m"
    MISSION_CTX['mission_z0'] = z0
    note = ""
    if abs(z0) > 1.0:
        note = (" -- il robot e' stato acceso a un'altra quota (scale?). La navigazione non ne "
                "risente; il valore e' salvato come mission_z0 in ogni file e mission_viewer "
                "mostra le quote rispetto all'avvio missione")
    print(f"[QUOTE] Riferimento quote = suolo all'avvio missione: z={z0:.3f} m nel frame VISION "
          f"({source}), cioe' {z0:+.2f} m rispetto all'accensione{note}.")
    return z0


class _TeeStream:
    """
    Copia tutto quello che viene stampato su un file, mantenendo anche la stampa normale a
    schermo. Nel file ogni riga e' preceduta da un'ora (HH:MM:SS.mmm), utile per allineare i
    log con un eventuale video o con le note prese a mano durante la prova.
    """
    def __init__(self, original, file_path):
        self._orig = original
        self._file = open(file_path, 'a', buffering=1, encoding='utf-8')
        self._lock = threading.Lock()
        self._at_line_start = True

    def write(self, s):
        try:
            self._orig.write(s)
        except Exception:
            pass
        try:
            with self._lock:
                out = []
                for chunk in s.splitlines(keepends=True):
                    if self._at_line_start:
                        out.append(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' ')
                    out.append(chunk)
                    self._at_line_start = chunk.endswith('\n')
                if out:
                    self._file.write(''.join(out))
        except Exception:
            pass
        return len(s)

    def flush(self):
        try:
            self._orig.flush()
        except Exception:
            pass
        try:
            with self._lock:
                self._file.flush()
        except Exception:
            pass

    def close_file(self):
        try:
            with self._lock:
                self._file.close()
        except Exception:
            pass

    def __getattr__(self, name):
        # isatty, encoding, fileno... passano allo stream originale
        return getattr(self._orig, name)


def install_stdout_tee(file_path):
    """Da qui in poi ogni print finisce anche in file_path. Idempotente."""
    try:
        if not isinstance(sys.stdout, _TeeStream):
            sys.stdout = _TeeStream(sys.stdout, file_path)
            print(f"[LOG] Copia completa dell'output su file: {file_path}")
    except Exception as e:
        print(f"[LOG] Impossibile attivare la copia dell'output su file: {e}")


def remove_stdout_tee():
    try:
        if isinstance(sys.stdout, _TeeStream):
            tee = sys.stdout
            sys.stdout = tee._orig
            tee.close_file()
    except Exception:
        pass


def _quat_to_roll_pitch_deg(q):
    """Roll e pitch (gradi) di un quaternione con attributi w, x, y, z. Nel frame VISION,
    che e' allineato alla gravita', sono l'inclinazione reale del corpo rispetto
    all'orizzontale (stessa convenzione con cui il file calcola gia' lo yaw)."""
    roll = np.arctan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x ** 2 + q.y ** 2))
    pitch = np.arcsin(np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
    return float(np.degrees(roll)), float(np.degrees(pitch))


class BodyTiltRecorder:
    """
    Registra l'inclinazione REALE del corpo di Spot (roll e pitch) mentre esegue un
    movimento, interrogando lo stato del robot circa 5 volte al secondo in un thread a parte.
    Serve a confrontare cio' che il pianificatore stimava (pendenza del terreno) con cio' che
    Spot ha effettivamente sentito: e' il dato che permette di ritarare soglie e pesi.

    Uso:
        rec = BodyTiltRecorder(robot_state_client); rec.start()
        ... movimento ...
        stats = rec.stop()   # dict con inizio, fine, picchi, numero di campioni

    Nota di lettura: Spot puo' livellare il corpo rispetto al terreno, quindi l'inclinazione
    del corpo e' un indicatore indiretto della pendenza, non la pendenza stessa. Ogni errore
    di lettura viene contato e ignorato.
    """
    def __init__(self, robot_state_client, period_s=0.2):
        self._client = robot_state_client
        self._period = period_s
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._start = None
        self._last = None
        self._peak_roll = 0.0
        self._peak_pitch = 0.0
        self._n = 0
        self._errors = 0
        # Traiettoria reale campionata: (t, x, y, yaw, roll, pitch). Finisce in
        # trajectory.csv, per confrontare dove il robot e' andato davvero con cio' che era
        # stato comandato.
        self.samples = []

    def _sample(self):
        try:
            x, y, _, q = spotUtils.getPosition(self._client)
            roll, pitch = _quat_to_roll_pitch_deg(q)
            yaw = float(np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))))
        except Exception:
            with self._lock:
                self._errors += 1
            return None
        with self._lock:
            if self._start is None:
                self._start = (roll, pitch)
            self._last = (roll, pitch)
            self._peak_roll = max(self._peak_roll, abs(roll))
            self._peak_pitch = max(self._peak_pitch, abs(pitch))
            self._n += 1
            self.samples.append((time.time(), float(x), float(y), yaw, roll, pitch))
        return roll, pitch

    def _run(self):
        while not self._stop.wait(self._period):
            self._sample()

    def start(self):
        try:
            self._sample()
            self._thread = threading.Thread(target=self._run, name="body_tilt_recorder", daemon=True)
            self._thread.start()
        except Exception:
            pass

    def stop(self):
        try:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=1.0)
            self._sample()
        except Exception:
            pass
        with self._lock:
            return {
                'roll_start': self._start[0] if self._start else None,
                'pitch_start': self._start[1] if self._start else None,
                'roll_end': self._last[0] if self._last else None,
                'pitch_end': self._last[1] if self._last else None,
                'roll_peak_abs': self._peak_roll if self._n else None,
                'pitch_peak_abs': self._peak_pitch if self._n else None,
                'samples': self._n,
                'errors': self._errors,
            }


class SegmentLog:
    """CSV con una riga per ogni segmento (da un nodo al successivo) effettivamente
    percorso o tentato. Si apre in un foglio di calcolo. Una riga viene scritta e scaricata su
    disco subito, cosi' anche una missione interrotta lascia i dati fino all'ultimo segmento."""
    def __init__(self, path):
        self._path = path
        self._lock = threading.Lock()
        with open(path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(_SEGMENT_COLUMNS)

    def write_row(self, row_dict):
        try:
            def fmt(v):
                if v is None:
                    return ''
                if isinstance(v, (float, np.floating)):
                    return f"{float(v):.4f}"
                return v
            with self._lock:
                with open(self._path, 'a', newline='', encoding='utf-8') as f:
                    csv.writer(f).writerow([fmt(row_dict.get(c)) for c in _SEGMENT_COLUMNS])
        except Exception as e:
            print(f"[LOG] Impossibile scrivere la riga del segmento: {e}")


def append_csv(name, header, rows):
    """Aggiunge righe a un CSV nella cartella dei log (intestazione alla prima scrittura). Mai eccezioni."""
    folder = MISSION_CTX.get('log_folder')
    if not folder or not rows:
        return
    try:
        path = os.path.join(folder, name)
        with _CSV_LOCK:
            new = not os.path.exists(path)
            with open(path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(header)
                for r in rows:
                    w.writerow([f"{v:.4f}" if isinstance(v, (float, np.floating)) else v for v in r])
    except Exception as e:
        print(f"[LOG] Impossibile scrivere {name}: {e}")


def append_trajectory(samples, move, target_xy=(np.nan, np.nan), seg_id=None):
    """Campioni di un BodyTiltRecorder in trajectory.csv. seg_id: le 6 colonne da iteration a node_to."""
    seg_id = list(seg_id) if seg_id is not None else [''] * 6
    append_csv("trajectory.csv", _TRAJECTORY_COLUMNS,
                [[t] + seg_id + [x, y, yw, r, pt, target_xy[0], target_xy[1], move]
                 for (t, x, y, yw, r, pt) in samples])


class TrajectoryLog:
    """
    Registra in trajectory.csv dove passa il robot durante un movimento che non e' un tratto
    del ciclo di navigazione (GraphNav, cono, arretramenti). Mai eccezioni verso chi chiama.

        with TrajectoryLog(robot_state_client, 'graphnav', (x, y)):
            ... movimento ...
    """
    def __init__(self, robot_state_client, move, target_xy=(np.nan, np.nan)):
        self._client, self._move, self._target = robot_state_client, move, target_xy
        self._rec = None

    def __enter__(self):
        try:
            self._rec = BodyTiltRecorder(self._client)
            self._rec.start()
        except Exception:
            self._rec = None
        return self

    def __exit__(self, *exc):
        if self._rec is None:
            return False
        try:
            self._rec.stop()
            append_trajectory(list(self._rec.samples), self._move, self._target)
        except Exception as e:
            print(f"[LOG] Traiettoria del movimento '{self._move}' non registrata: {e}")
        return False


def log_segment(row_dict):
    seg_log = MISSION_CTX.get('segment_log')
    if seg_log is not None:
        seg_log.write_row(row_dict)
        MISSION_STATS['segments_logged'] += 1


def visualize_grid_with_candidates(pts, terrain_real, obstacle_mask, robot_x, robot_y,
                                   candidates, chosen_point, iteration, env, save_path,
                                   prm_graph=None, chosen_path=None,
                                   cells_obstacle_dist=None, intensity_values=None,
                                   valid_values=None, grad_values=None, rough_values=None,
                                   include_diagnostics=True):
    """
    Dati di una figura (scansione, candidati, punto scelto, grafo PRM, percorso) salvati in
    <nome>_vis.npz. Durante la missione non si disegna: le figure le fa mission_viewer.py
    dopo, da questi file.
    """
    if save_path is None:
        return
    save_visualization_bundle(save_path, pts, terrain_real, obstacle_mask, robot_x, robot_y,
                               candidates, chosen_point, iteration, env, prm_graph, chosen_path,
                               cells_obstacle_dist, valid_values, grad_values, rough_values)


def save_visualization_bundle(save_path, pts, terrain_real, obstacle_mask, robot_x, robot_y,
                               candidates, chosen_point, iteration, env, prm_graph, chosen_path,
                               cells_obstacle_dist, valid_values, grad_values, rough_values):
    """
    Al posto di una figura: tutti i dati che la figura avrebbe mostrato, compreso il grafo
    PRM in quel momento. Viene chiamata quando il grafo cambia (inizio tentativo,
    ripianificazione, blocco), quindi il grafo si salva qui e non a ogni scansione.
    """
    t0 = time.perf_counter()
    try:
        def _arr(a, dtype=np.float64):
            return np.empty(0, dtype=dtype) if a is None else np.asarray(a, dtype=dtype)

        node_ids = np.array(list(prm_graph.nodes.keys()), dtype=np.int64) if prm_graph else np.empty(0, np.int64)
        node_xy = (np.array([prm_graph.nodes[i] for i in node_ids], dtype=np.float64)
                   if prm_graph and len(node_ids) else np.empty((0, 2)))
        pairs, weights = [], []
        if prm_graph:
            for a, nbrs in prm_graph.edges.items():
                for b, w in nbrs:
                    if a < b:
                        pairs.append((a, b)); weights.append(w)
        slope_keys = list(prm_graph.edge_slope_info.keys()) if prm_graph else []
        slope_vals = [prm_graph.edge_slope_info[k] for k in slope_keys] if prm_graph else []
        traversed = np.array(sorted(getattr(prm_graph, 'traversed_edges', set())), dtype=np.int64)

        cells = []
        if env is not None:
            for row in range(env.rows):
                for col in range(env.cols):
                    wp = env.get_world_position_from_cell(row, col)
                    if wp is None:
                        continue
                    st = env.get_cell_status(row, col)
                    st = st[0] if isinstance(st, (tuple, list)) else st
                    cells.append((row, col, wp[0], wp[1], str(st)))

        def _pts(lst):
            try:
                a = np.asarray([(float(p[0]), float(p[1])) for p in (lst or [])], dtype=np.float64)
                return a.reshape(-1, 2)
            except Exception:
                return np.empty((0, 2))

        out = os.path.splitext(save_path)[0] + "_vis.npz"
        save_npz_async(
            out,
            pts=_arr(pts), terrain=_arr(terrain_real), obstacle_mask=_arr(obstacle_mask),
            obstacle_distance=_arr(cells_obstacle_dist), terrain_valid=_arr(valid_values),
            gradient=_arr(grad_values), roughness=_arr(rough_values),
            robot_xy=np.array([robot_x, robot_y], dtype=np.float64), mission_z0=mission_z0(),
            # istante della vista: mission_viewer abbina la vista alle scansioni con questo, non
            # con la data del file (dopo un git clone o pull e' quella del download)
            time=time.time(),
            chosen_point=np.array([np.nan if v is None else v for v in (chosen_point or (None, None))],
                                  dtype=np.float64),
            chosen_path=_pts(chosen_path), iteration=iteration,
            candidates_valid=_pts((candidates or {}).get('valid')),
            candidates_rejected=_pts((candidates or {}).get('rejected')),
            prm_node_ids=node_ids, prm_node_xy=node_xy,
            prm_edges=np.array(pairs, dtype=np.int64).reshape(-1, 2),
            prm_edge_weights=np.array(weights, dtype=np.float64),
            prm_slope_keys=np.array(slope_keys, dtype=np.int64).reshape(-1, 2),
            prm_slope_values=np.array(slope_vals, dtype=np.float64).reshape(-1, 2),
            prm_traversed_edges=traversed.reshape(-1, 2),
            env_cells=np.array([c[:4] for c in cells], dtype=np.float64).reshape(-1, 4),
            env_cell_status=np.array([c[4] for c in cells]),
            env_cell_size=np.nan if env is None else float(env.cell_size),
            env_origin_yaw=np.nan if env is None else float(env.origin_yaw),
            traveled_arcs=np.array(getattr(env, '_traveled_arcs', []), dtype=np.float64).reshape(-1, 4),
        )
    except Exception as e:
        print(f"[DATA SAVE] Impossibile preparare i dati della figura: {e}")
    timing_add('dati figure', time.perf_counter() - t0)
