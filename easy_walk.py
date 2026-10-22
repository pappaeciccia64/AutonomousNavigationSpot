import os
import sys
import csv
import json
import time
from time import sleep
from datetime import datetime
import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.frame_helpers import *
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, blocking_stand)
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.frame_helpers import get_a_tform_b
from types import SimpleNamespace
import navGraphUtils
from bosdyn.api.graph_nav import graph_nav_pb2 as _graph_nav_pb2
import movements
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import spotGrid
import spotLogInUtils
import environmentMap
import spotUtils
import arcVerification

import prm_graph
import prm_graph as prm_graph_module   # dentro le funzioni `prm_graph` e' spesso il parametro (istanza)
import mission_graph                   # reticolo dei nodi della missione

import threading
from concurrent.futures import ThreadPoolExecutor

# Un thread di appoggio per salvare i dati su disco senza fermare il ciclo di navigazione.
_VIS_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vis_worker")

# ==================================================================
# STATISTICHE DI MISSIONE -- solo diagnostica/log, mai lette per decidere il
# comportamento del robot. Resettato all'inizio di ogni chiamata a easy_walk().
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
    'time_s': {},        # secondi spesi per fase, vedi _timing_add / [TEMPO]
    'scan_bundles': 0,
    'frontier_blocks': 0,
    'frontier_aborts': 0,
    'rotation_refusals': 0,   # rotazioni rifiutate dal controllo sul settore
    'straight_retreats': 0,   # arretramenti dritti dopo un rifiuto di rotazione
    'goal_repicks': 0,        # punti obiettivo riscelti dentro il ciclo
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


def _check_and_recover_fall(robot_state_client, command_client, global_map, where):
    """
    Controlla se il robot e' caduto e, se si', lo rialza (movements.recover_from_fall) e segna
    il punto come pericoloso nella mappa globale, cosi' il pianificatore lo evita.

    Restituisce None se non c'e' stata nessuna caduta, altrimenti (x, y) del punto in cui il
    robot e' caduto, a robot di nuovo in piedi. Solleva movements.RobotFallenError se non si
    puo' rialzare da solo: la missione si ferma e serve l'operatore.
    """
    try:
        st = movements.check_fall(robot_state_client)
    except Exception as e:
        print(f"[CADUTA] Impossibile leggere lo stato del robot ({where}): {e}")
        return None
    if not st['fallen']:
        return None
    MISSION_STATS['falls'] += 1
    print(f"[CADUTA] Rilevata {where} (caduta n. {MISSION_STATS['falls']} della missione).")
    movements.recover_from_fall(command_client, robot_state_client, status=st)
    MISSION_STATS['self_rights'] += 1
    if MISSION_STATS['falls'] >= MAX_FALLS_PER_MISSION:
        raise movements.RobotFallenError(
            f"{MISSION_STATS['falls']} cadute in questa missione: il robot si e' rialzato ma la missione "
            f"si ferma qui. Controllare il terreno e il robot prima di ripartire.")
    if global_map is not None:
        n = global_map.mark_hazard(st['x'], st['y'], FALL_HAZARD_RADIUS_M)
        print(f"[CADUTA] Punto della caduta ({st['x']:.2f}, {st['y']:.2f}) segnato come pericoloso "
              f"nella mappa: disco di {FALL_HAZARD_RADIUS_M:.1f} m ({n} celle), il pianificatore lo evitera'.")
    return (st['x'], st['y'])


def _timing_add(phase, seconds):
    """Accumula il tempo speso in una fase (riepilogo [TEMPO] a fine missione)."""
    MISSION_STATS['time_s'][phase] = MISSION_STATS['time_s'].get(phase, 0.0) + float(seconds)


# ==================================================================
# GRAFO DI MISSIONE REGOLARE -- vedi mission_graph.py.
#
# I nodi stanno su un reticolo rettangolare con passo cell_size/n. Conseguenze:
#   - grafo DETERMINISTICO: stessi parametri, stesso grafo, quindi test_replay puo'
#     riprodurre le scelte del pianificatore e due missioni sono confrontabili;
#   - il centro di ogni cella e' un nodo del reticolo per costruzione, quindi l'obiettivo
#     di cella e' il centro geometrico ESATTO (non un punto vicino);
#   - il punto di accensione e' il centro della cella di partenza ed e' anch'esso un nodo.
#
#   MISSION_LATTICE_SPACING_M  passo fra i nodi, da mission_graph.MISSION_SPACING_M (None =
#                              0.25 m). Puo' stare sotto PRM_MIN_EDGE_M: i vicini piu'
#                              corti di PRM_MIN_EDGE_M non si collegano (vedi mission_graph,
#                              SPAZIATURA e ANELLI). None = mission_graph.suggested_spacing().
#                              Arrotondato a cell_size/n. Da riga di comando: --spacing.
#   MISSION_LATTICE_RINGS      anelli di vicini collegati: k = tutti i nodi entro la diagonale
#                              del k-esimo anello, purche' a >= PRM_MIN_EDGE_M. Con passo 0.25
#                              e k=2: archi da 0.50, 0.56 e 0.71 m in 16 direzioni. None =
#                              tutti quelli entro max_edge_length. Da riga di comando: --rings.
#   MISSION_GRID_ROWS/COLS, MISSION_CELL_SIZE_M  dimensione della missione, da
#                              mission_graph.MISSION_*. Si cambiano con
#                              --rows --cols --cell oppure SPOT_MISSION_GRID (vedi easy_walk).
#   MISSION_GRAPH_SAMPLING     'reticolo' (quello descritto qui) o 'casuale': nodi sparsi con
#                              una distribuzione, centri cella sempre compresi (vedi
#                              mission_graph, NODI CASUALI). Da riga di comando: --grafo.
#   MISSION_RANDOM_*           distribuzione, densita' e seme dei nodi casuali, da
#                              mission_graph.RANDOM_*. Seme None = estratto a caso; quello
#                              usato va nel log e in mission_summary.json. Da riga di
#                              comando: --distribuzione --densita --seme.
#   PRM_MIN_EDGE_M, PRM_MAX_EDGE_M  lunghezza minima e massima degli archi, per entrambi
#                              i metodi. Da riga di comando: --min-edge --max-edge.
#
# Regola degli archi (scelta dell'utente): nodi a 0.25 m, ma un arco esiste SOLO fra nodi
# distanti fra 0.5 e 1.25 m -- cioe' `rings=None` (nessun limite per gusci) con
# PRM_MAX_EDGE_M = 1.25. Un arco salta fino a cinque passi di reticolo: con archi piu' corti
# l'ingresso in una cella diventa una catena di molti waypoint, e ogni waypoint e' una
# rotazione, proprio nei corridoi stretti in cui ruotare non si puo'. Il prezzo e' una
# costruzione del grafo ~5 volte piu' lenta (vedi CHANGELOG), in parte restituito dal
# refresh a eventi.
#
# PRM_MAX_EDGE_M e' anche il max_edge_length del PRM (mg.reach_m):
# refresh_local_edge_weights ricostruisce da se' le coppie candidate dentro la finestra
# usando quel valore, quindi un tetto diverso varrebbe solo fuori dalla finestra e il grafo
# si riempirebbe di archi lunghi solo attorno al robot. I tratti lunghi li produce comunque
# shortcut_index, che li conferma sui dati DAL VIVO invece di giudicarli da lontano.
#
# Righe, colonne, lato cella e passo vengono da mission_graph.mission_settings(): si
# scelgono la' (MISSION_ROWS, MISSION_COLS, MISSION_CELL_SIZE_M, MISSION_SPACING_M; None =
# default). Da qui si costruiscono sia la mappa delle celle sia il grafo.
# ==================================================================
_MISSION = mission_graph.mission_settings()
MISSION_GRID_ROWS = _MISSION[0]
MISSION_GRID_COLS = _MISSION[1]
MISSION_CELL_SIZE_M = _MISSION[2]
MISSION_LATTICE_SPACING_M = _MISSION[3]
MISSION_LATTICE_RINGS = mission_graph.LATTICE_RINGS   # None: nessun limite per gusci, decide PRM_MAX_EDGE_M
MISSION_GRAPH_SAMPLING = mission_graph.GRAPH_SAMPLING
MISSION_RANDOM_DISTRIBUTION = mission_graph.RANDOM_DISTRIBUTION
MISSION_RANDOM_DENSITY = mission_graph.RANDOM_DENSITY_PER_M2
MISSION_RANDOM_SEED = mission_graph.RANDOM_SEED
PRM_MIN_EDGE_M = mission_graph.DEFAULT_MIN_EDGE_M     # 0.5 m
PRM_MAX_EDGE_M = mission_graph.DEFAULT_MAX_EDGE_M     # 1.25 m, tetto degli archi (vedi sopra)
PRM_CONNECTION_RADIUS_M = 3.0

# --- Dove puo' passare il pianificatore -------------------------------------------------
# Per entrare in una cella il percorso puo' usare SOLO le celle gia' visitate, la cella in
# cui si trova il robot e la cella obiettivo: le celle mai esplorate non si attraversano.
# Se l'unica strada passa da una cella non visitata non si prova: il tentativo fallisce, il
# lato viene segnato come provato (finalize_or_defer_blocked_cell) e si passa a un'altra
# cella; quella bloccata verra' ritentata piu' avanti, da un altro lato.
PLAN_ONLY_EXPLORED_CELLS = True
# Tetto alla deviazione: un percorso piu' lungo di RATIO * linea d'aria + SLACK vale
# "nessun percorso" (vedi PRM.set_detour_limit). None = nessun tetto.
PLAN_MAX_DETOUR_RATIO = 2.0
PLAN_DETOUR_SLACK_M = 1.0
# Copia del modulo: dentro attempt_enter_cell_from_position il nome prm_graph e' l'istanza.
ANCHOR_ON_NODE_M = prm_graph.ANCHOR_ON_NODE_M


def _build_mission_prm(env, verbose=True):
    """
    Crea il grafo della missione (reticolo o nodi casuali, vedi MISSION_GRAPH_SAMPLING) e il
    PRM con i suoi nodi, dalla dimensione della missione e dalla posa di partenza che `env`
    ha gia' dentro (set_origin va fatto PRIMA).

    Restituisce (prm, sampler, mission_lattice): `sampler` da' i nodi cella per cella (vedi
    mission_graph.LatticeSampler), `mission_lattice` e' il MissionGraph.
    Gli ARCHI non vengono creati qui: servono i dati di terreno, vedi _rebuild_prm_edges.
    """
    mg = mission_graph.build_mission_graph(
        env=env, spacing_m=MISSION_LATTICE_SPACING_M, rings=MISSION_LATTICE_RINGS,
        min_edge_length=PRM_MIN_EDGE_M, max_edge_length=PRM_MAX_EDGE_M,
        connection_radius=PRM_CONNECTION_RADIUS_M, verbose=verbose,
        sampling=MISSION_GRAPH_SAMPLING, distribution=MISSION_RANDOM_DISTRIBUTION,
        density_per_m2=MISSION_RANDOM_DENSITY, seed=MISSION_RANDOM_SEED)
    prm = prm_graph.PRM(min_edge_length=PRM_MIN_EDGE_M, max_edge_length=mg.reach_m,
                        connection_radius=PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg          # lo rilegge _rebuild_prm_edges
    prm.set_detour_limit(PLAN_MAX_DETOUR_RATIO, PLAN_DETOUR_SLACK_M)
    mg.into_prm(prm)
    return prm, mg.as_sampler(), mg


def _node_cells(prm, env):
    """
    {id nodo: (riga, colonna)}. Dal reticolo quando c'e' (mission_graph.node_cell, stessa
    regola di env.get_cell_from_world), altrimenti da env. In cache sul PRM: i nodi non
    cambiano durante la missione.
    """
    cache = getattr(prm, '_node_cell_cache', None)
    if cache is None:
        cache = {}
        prm._node_cell_cache = cache
    mg = getattr(prm, 'mission_graph', None)
    for nid, (x, y) in prm.nodes.items():
        if nid in cache:
            continue
        if mg is not None and 0 <= nid < mg.n_nodes:
            r, c = mg.node_cell[nid]
            cache[nid] = (int(r), int(c))
        else:
            cache[nid] = tuple(int(v) for v in env.get_cell_from_world(x, y))
    return cache


def _restrict_planning_to_explored_cells(prm, env, target_row, target_col, robot_xy):
    """
    Limita la ricerca del percorso ai nodi delle celle visitate, della cella del robot e
    della cella obiettivo (vedi PLAN_ONLY_EXPLORED_CELLS). Da chiamare prima di ogni
    tentativo di cella: le celle visitate cambiano fra un tentativo e l'altro.
    """
    if not PLAN_ONLY_EXPLORED_CELLS:
        prm.set_allowed_nodes(None)
        return
    cells = {(r, c) for r in range(env.rows) for c in range(env.cols) if env.is_cell_visited(r, c)}
    cells.add((int(target_row), int(target_col)))
    cells.add(tuple(int(v) for v in env.get_cell_from_world(robot_xy[0], robot_xy[1])))
    allowed = [nid for nid, cell in _node_cells(prm, env).items() if cell in cells]
    prm.set_allowed_nodes(allowed)
    print(f"[PIANO] Percorsi ammessi solo nelle celle {sorted(cells)}: {len(allowed)} nodi su "
          f"{len(prm.nodes)}. Se serve passare da una cella non visitata, rinuncio a questo "
          f"lato e passo a un'altra cella.")


def _robot_on_node(prm, node_id, x, y):
    """True se il robot e' praticamente sul nodo (vedi prm_graph.ANCHOR_ON_NODE_M)."""
    pos = prm.get_node_position(node_id)
    return pos is not None and float(np.hypot(pos[0] - x, pos[1] - y)) <= ANCHOR_ON_NODE_M


# True: all'inizio di ogni cella si rivalutano solo gli archi vicini alle zone con dati
# nuovi (vedi mission_graph.MissionGraph.add_edges_to_prm, incremental). False = si
# rivalutano tutti; il risultato e' lo stesso.
INCREMENTAL_GRAPH_UPDATE = True


def _rebuild_prm_edges(prm, global_map):
    """
    Rivaluta gli archi del PRM sui dati correnti, sulle coppie del reticolo gia' calcolate
    (vedi mission_graph.MissionGraph.add_edges_to_prm). Veti e pesi li calcola
    PRM._evaluate_and_add_edge.
    """
    return prm.mission_graph.add_edges_to_prm(
        prm, global_map=global_map,
        edge_safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
        incremental=INCREMENTAL_GRAPH_UPDATE)


def _save_npz_async(path, **arrays):
    """Salvataggio compresso nel thread di servizio: il ciclo principale non aspetta il disco."""
    def _job():
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            np.savez_compressed(path, **arrays)
        except Exception as e:
            print(f"[DATA SAVE] Impossibile salvare {os.path.basename(path)}: {e}")
    _VIS_EXECUTOR.submit(_job)

# Contesto della missione in corso, per i log diagnostici che non hanno modo di ricevere
# questi oggetti come argomento (attempt_enter_cell_from_position e' chiamata da piu' punti
# e non vogliamo cambiarne la firma). Riempito in easy_walk(), letto da _log_segment() e
# _emit_mission_summary_once(). Solo diagnostica: nessuna decisione del robot lo legge.
_MISSION_CTX = {
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
}


def _mission_z0():
    """Quota di riferimento della missione (NaN finche' non e' fissata), per i file salvati."""
    z0 = _MISSION_CTX.get('mission_z0')
    return np.nan if z0 is None else float(z0)


def _set_mission_z0(robot_state_client, z_body):
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
    _MISSION_CTX['mission_z0'] = z0
    note = ""
    if abs(z0) > 1.0:
        note = (" -- il robot e' stato acceso a un'altra quota (scale?). La navigazione non ne "
                "risente; il valore e' salvato come mission_z0 in ogni file e mission_viewer "
                "mostra le quote rispetto all'avvio missione")
    print(f"[QUOTE] Riferimento quote = suolo all'avvio missione: z={z0:.3f} m nel frame VISION "
          f"({source}), cioe' {z0:+.2f} m rispetto all'accensione{note}.")
    return z0

# ==================================================================
# NAVIGAZIONE CON IL FRONTE SICURO -- vedi decide_next_move() e il ciclo
# in attempt_enter_cell_from_position(). Le distanze del fronte stanno in spotGrid
# (FRONTIER_*), qui solo la logica del ciclo.
#   MIN_PARTIAL_STEP_M           -- passo minimo: sotto questa lunghezza non vale la pena
#                                   muoversi (si pagherebbe una scansione per pochi cm).
#   FRONTIER_BLOCK_CONFIRM_SCANS -- quante scansioni consecutive, a robot FERMO, senza poter
#                                   avanzare nemmeno del passo minimo, prima di dichiarare
#                                   un blocco: la conferma "da vicino", invece di
#                                   condannare un arco al primo scorcio da lontano.
#   FRONTIER_RESCAN_WAIT_S       -- pausa fra due scansioni di conferma.
#   MAX_BLOCK_REPLANS            -- dopo un blocco si ripianifica dal punto in cui si e'; se
#                                   succede piu' di queste volte nello stesso tentativo, ci
#                                   si arrende (ritirata, cella rimandata). Lo stesso
#                                   valore limita anche le riscelte del punto obiettivo.
#   FRONTIER_ABORT_CONFIRM       -- a movimento in corso, quanti fronti consecutivi (~0.2 s
#                                   l'uno) devono dire "non arrivi dove stai andando" prima
#                                   di fermare il robot. 2 evita di inchiodare per un singolo
#                                   fotogramma sporco; a 0.5 m/s sono ~20 cm.
#   GOAL_REACHED_TOLERANCE_M     -- se il fronte impedisce gli ultimi centimetri verso
#                                   l'obiettivo (es. un muro appena oltre), ci si accontenta.
#   MAX_LOOP_ITERATIONS          -- salvaguardia anti-stallo del ciclo.
# ==================================================================
MIN_PARTIAL_STEP_M = 0.40
FRONTIER_BLOCK_CONFIRM_SCANS = 3
FRONTIER_RESCAN_WAIT_S = 0.3
MAX_BLOCK_REPLANS = 3
FRONTIER_ABORT_CONFIRM = 2
FRONTIER_ABORT_TOLERANCE_M = 0.05

# Legata allo scostamento che il fronte stesso impone: allowed_advance() ferma il CENTRO del
# robot a FRONTIER_BODY_HALF_LENGTH_M + FRONTIER_MARGIN_M = 0.65 m prima del primo punto non
# libero. Una tolleranza piu' piccola lascerebbe una banda morta in cui il robot non puo' ne'
# avanzare ne' dichiarare di essere arrivato, e finirebbe per dichiarare un blocco. Le due
# costanti devono essere la stessa cosa.
GOAL_REACHED_TOLERANCE_M = spotGrid.FRONTIER_BODY_HALF_LENGTH_M + spotGrid.FRONTIER_MARGIN_M
MAX_LOOP_ITERATIONS = 150

# --- GUARDIE ANTI-STALLO ------------------------------------------------------------
# Il ciclo puo' girare a vuoto con il robot immobile: la rotazione verso il prossimo nodo
# viene rifiutata, il cono non trova direzioni che avvicinino, ma le direzioni utilizzabili
# non sono zero (quindi non e' "trappola"); si scarta il tratto, si ripianifica, e al giro
# dopo il primo waypoint punta da un'altra parte e la rotazione viene rifiutata di nuovo.
#   MAX_ROTATION_DISCARDS -- scarti di tratto per rifiuto di rotazione ammessi in un
#       tentativo di cella. Contati per TENTATIVO e non per coppia di nodi: la coppia cambia
#       a ogni ripianificazione, e un contatore per coppia non scatterebbe mai (e' il
#       limite di MAX_ROTATION_FAILS_SAME_EDGE).
#   MAX_IDLE_LOOPS -- giri di ciclo consecutivi senza che il robot si sia spostato di
#       almeno IDLE_MOVE_M. E' la rete generica: qualunque sia il ramo che gira a vuoto,
#       dopo tanti giri fermi si rinuncia alla cella in modo ordinato invece di consumare
#       batteria. Deve stare SOPRA la sequenza legittima piu' lunga a robot fermo,
#       altrimenti taglia via proprio le reti di ripiego (ritorno con GraphNav) che quella
#       sequenza deve raggiungere. Calcolata sui limiti del ciclo:
#         [SUBITO]                MAX_EARLY_REPLANS              4
#         obiettivo riscelto      MAX_BLOCK_REPLANS              3
#         conferme di blocco      FRONTIER_BLOCK_CONFIRM_SCANS x (MAX_BLOCK_REPLANS+1) = 3 x 4 = 12
#         rifiuti di rotazione    MAX_ROTATION_DISCARDS          4
#         rifiuti d'arrivo        MAX_ARRIVAL_REFUSALS           3
#                                                        totale 26  -> 30
#       A ~0.7 s per giro fermo sono ~20 s. Il giro a vuoto sulla rotazione lo prende
#       prima MAX_ROTATION_DISCARDS; questa e' la rete per tutto il resto.
MAX_ROTATION_DISCARDS = 4
MAX_IDLE_LOOPS = 30
IDLE_MOVE_M = 0.05

# ==================================================================
# ROTAZIONE E ARRETRAMENTO.
#
# Decisione dell'utente: il robot punta SEMPRE il muso verso il punto da raggiungere.
# Niente marcia obliqua. Di traverso Spot e' largo 1.10 m invece di 0.50, quindi il
# fronte pretende 0.59 m di obstacle_distance invece di 0.30: andare di lato quando non
# si riesce a ruotare renderebbe il robot PIU' LARGO proprio dove lo spazio e' poco.
#
# Quando la rotazione non e' possibile il robot ARRETRA IN LINEA RETTA lungo l'asse
# del corpo -- che non e' marcia obliqua: l'ingombro resta 0.50 m e la soglia del
# fronte resta 0.30 -- e al giro dopo riprova da li'. Alla seconda volta sullo stesso
# tratto il tratto si scarta e si ripianifica.
#   ROTATION_RETREAT_M        -- quanto si tenta di arretrare (il fronte puo' concederne meno)
#   ROTATION_RETREAT_MIN_M    -- sotto questo arretramento non vale la pena muoversi
#   MAX_ROTATION_FAILS_SAME_EDGE -- tentativi di arretramento sullo stesso tratto prima di scartarlo
# ==================================================================
ROTATION_RETREAT_M = 0.60
ROTATION_RETREAT_MIN_M = 0.20
MAX_ROTATION_FAILS_SAME_EDGE = 2

# --- Ventaglio dentro il cono di rotazione possibile -- vedi plan_cone_move.
CONE_SWEEP_MAX_DEG = 90.0      # quanto in la' si guarda, a destra e a sinistra del muso
CONE_SWEEP_STEP_DEG = 10.0     # passo della scansione angolare (9 direzioni per lato)
CONE_MIN_GAIN_M = 0.20         # una direzione vale solo se avvicina all'obiettivo di tanto
# Il cono punta al punto del PERCORSO a questa distanza dal robot, non all'obiettivo della
# cella in linea d'aria: se il percorso gira attorno a un blocco, "avvicinarsi" all'obiettivo
# in linea d'aria puo' portare il robot dentro un vicolo cieco, dalla parte opposta al
# percorso. Se quel punto e' dietro, nessuna direzione del cono avvicina, e si passa
# all'arretramento -- che e' la mossa giusta.
CONE_PATH_LOOKAHEAD_M = 1.5
# Durante la rotazione sul posto un fronte "fine dati" piu' corto non interrompe il movimento:
# ruotando la zona vista si sposta e il bordo dei dati si accorcia di qualche centimetro senza
# alcun ostacolo. Vale finche'
# il robot non si e' spostato di almeno tanto; un fronte accorciato da un OSTACOLO interrompe
# sempre.
ABORT_IGNORE_DATA_EDGE_UNTIL_M = 0.15

# Arretramenti consecutivi ammessi, CONTATI PER SEQUENZA e non per arco:
# MAX_ROTATION_FAILS_SAME_EDGE e' indicizzato su (nodo_robot, nodo_successivo), e dopo un
# arretramento il robot puo' trovarsi agganciato a un altro nodo, quindi da solo non ferma
# una catena di arretramenti.
MAX_STRAIGHT_RETREATS_IN_A_ROW = 2

# --- Ventaglio a 360 gradi: riconoscere la trappola -- vedi scan_escape_directions.
ESCAPE_FAN_STEP_DEG = 15.0        # 24 direzioni; 24 valutazioni del fronte, ~frazione di scansione
ESCAPE_FAN_MIN_ADVANCE_M = 0.30   # sotto questo spazio una direzione non e' una via d'uscita
# Controllo preventivo sulla posa d'arrivo (vedi arrival_has_exit): si paga solo dove serve,
# cioe' sui passi corti e verso spazi stretti, che sono i casi in cui si entra negli angoli.
ARRIVAL_EXIT_CHECK_MAX_STEP_M = 0.40
ARRIVAL_EXIT_CHECK_OD_M = 0.50
MAX_ARRIVAL_REFUSALS = 3          # per tentativo di cella, per non avvitarsi sul rifiuto

# --- Ritorni con GraphNav invece che camminando all'indietro ------------------------------
# Tornare indietro a zampe, all'indietro, sui tratti registrati e' la manovra piu' rischiosa
# che facciamo: il robot non vede dove va e non si rilocalizza. GraphNav cammina in avanti,
# con la localizzazione e l'anti-ostacolo di Boston Dynamics. Si lascia una traccia di
# waypoint lungo la strada (vedi graphnav_drop_breadcrumb) e si torna su quella.
GRAPHNAV_WAYPOINT_MIN_SPACING_M = 0.75   # distanza minima fra due briciole
GRAPHNAV_MAX_BACKTRACK_HOPS = 3          # poi si rimanda la cella e si passa alla priorita' dopo

# --- Scorciatoie nel grafo GraphNav -----------------------------------------------------
# Ogni waypoint si collega solo al precedente: il grafo GraphNav e' una CATENA che ricalca la
# strada fatta, e GraphNav cammina solo sugli archi. Senza scorciatoie, tornare a un waypoint
# a un metro di distanza ma registrato molti passi prima vuol dire ripercorrere tutta la
# catena. Prima di ogni spostamento con GraphNav (graphnav_add_shortcuts):
#   1. scorciatoie NOSTRE, scelte SULLA STRADA (vedi plan_graphnav_shortcuts): si calcola la
#      strada piu' corta verso la destinazione considerando TUTTE le coppie di waypoint a
#      meno di GRAPHNAV_SHORTCUT_MAX_M, senza arco diretto, la cui linea retta e' stata
#      VISTA libera nella mappa globale con GRAPHNAV_SHORTCUT_MARGIN_M di margine, e si
#      creano solo quelle che la strada usa. Mai attraverso zone mai viste.
#   2. chiusure d'anello di Spot (fiducial + odometria) con il loro controllo di collisione.
# GraphNav conserva comunque il suo anti-ostacolo durante il movimento.
# GRAPHNAV_SHORTCUT_MARGIN_M: margine dalle celle occupate (che sono gia' a obstacle_distance
# <= 0.15): 0.30 + 0.15 = 0.45 m dall'ostacolo, largo perche' gli archi sono lunghi.
GRAPHNAV_SHORTCUTS = True
GRAPHNAV_SDK_LOOP_CLOSURE = True
GRAPHNAV_SHORTCUT_MAX_M = 3.0
GRAPHNAV_SHORTCUT_MARGIN_M = 0.30
# Lunghezza massima degli archi che crea la chiusura d'anello di Spot.
GRAPHNAV_SDK_LOOP_MAX_M = 2.0
# Un nodo PRM con meno di questo spazio libero non e' raggiungibile dal fronte: si scarta.
PRM_NODE_MIN_CLEARANCE_M = spotGrid.FRONTIER_CLEARANCE_M   # 0.30
# Ripianificazioni "precoci" (alla prima cella bloccante, senza aspettare la conferma da
# ferma) ammesse per tentativo di cella. Vedi il ramo 'wait'.
MAX_EARLY_REPLANS = 4
# Ogni ritorno indietro ha memoria per il tentativo (vedi _trap_backtrack): l'arco su cui il
# robot si e' fermato diventa impercorribile fino alla fine del tentativo, e al
# MAX_BACKTRACKS_PER_ATTEMPT-esimo ritorno il robot torna indietro e rinuncia alla cella da
# questo lato. Senza memoria, dalla briciola precedente il pianificatore ritroverebbe lo
# stesso percorso e il robot rientrerebbe nella stessa trappola.
MAX_BACKTRACKS_PER_ATTEMPT = 2
# Entro questa distanza l'obiettivo del recupero e' "lo stesso" di quello appena fallito.
RECOVERY_SAME_GOAL_M = 0.30

# ==================================================================
# DATI DI MISSIONE.
#   Durante la missione non si disegna nulla: ogni figura viene salvata come dati in un
#   <nome>_vis.npz (vedi visualize_grid_with_candidates) e le figure si fanno dopo con
#   mission_viewer.py.
#   SAVE_SCAN_BUNDLES -- un .npz per ogni scansione del ciclo di navigazione, nella
#       sottocartella scans/: layer GREZZI come arrivano da Spot (terrain, terrain_valid,
#       obstacle_distance), i derivati usati per decidere, la georeferenziazione (origine,
#       cella, posa del robot, suolo stimato), percorso, fronte e decisione. E' cio' che usano
#       mission_viewer.py, mission_report.py e test_replay.py. ~0.3 MB a scansione.
# ==================================================================
SAVE_SCAN_BUNDLES = True

# Raggio del disco segnato come pericoloso nella mappa globale attorno al punto di una
# caduta (vedi _check_and_recover_fall e GlobalGrid.mark_hazard).
FALL_HAZARD_RADIUS_M = 0.5

# ==================================================================
# RITIRATA (vedi retreat_along_traveled_path).
#   RETREAT_MAX_SEGMENTS  -- quanti tratti gia' percorsi ripercorrere a ritroso al
#                            massimo. Pochi: serve a uscire da una tasca, non a
#                            tornare al punto di partenza.
#   RETREAT_MIN_SEGMENT_M -- sotto questa lunghezza il tratto si salta (non vale un
#                            comando di movimento a se').
#   RETREAT_AFTER_ABORT_M -- quanto arretrare subito dopo un arresto a meta'
#                            movimento, per staccarsi da cio' che ha fatto fermare.
# ==================================================================
RETREAT_MAX_SEGMENTS = 3
RETREAT_MIN_SEGMENT_M = 0.20
RETREAT_AFTER_ABORT_M = 0.40
#   RETREAT_CONTINUITY_M  -- si arretra lungo un tratto registrato solo se il robot sta
#                            davvero dove quel tratto finiva. Dopo uno spostamento con GraphNav
#                            fra celle (che non registra tratti) l'ultimo tratto registrato puo'
#                            stare a molti metri, e il robot ci andrebbe camminando
#                            all'indietro, alla cieca.
#   RETREAT_MAX_SEGMENT_M -- tratto piu' lungo di cosi': non lo si ripercorre all'indietro.
RETREAT_CONTINUITY_M = 0.50
RETREAT_MAX_SEGMENT_M = 2.50


# =========================================================================
# STRUMENTI DIAGNOSTICI -- tutto quello che segue serve SOLO a registrare cosa succede
# durante la missione. Nessuna decisione del robot dipende da questi oggetti, e ogni
# funzione e' scritta per non sollevare mai eccezioni verso il chiamante: un log che
# fallisce non deve mai fermare o alterare un movimento.
# =========================================================================

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


def _install_stdout_tee(file_path):
    """Da qui in poi ogni print finisce anche in file_path. Idempotente."""
    try:
        if not isinstance(sys.stdout, _TeeStream):
            sys.stdout = _TeeStream(sys.stdout, file_path)
            print(f"[LOG] Copia completa dell'output su file: {file_path}")
    except Exception as e:
        print(f"[LOG] Impossibile attivare la copia dell'output su file: {e}")


def _remove_stdout_tee():
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


_SEGMENT_COLUMNS = [
    'timestamp', 'iteration', 'cell_row', 'cell_col', 'node_from', 'node_to',
    'x_from', 'y_from', 'x_to', 'y_to', 'dist_m',
    'slope_source', 'long_slope', 'lat_slope', 'long_interp', 'lat_interp',
    'roughness', 'coverage_along', 'coverage_lateral',
    'cost_dist', 'cost_long', 'cost_lat', 'cost_rough', 'cost_total',
    'tier', 'gait_override', 'success', 'dist_traveled_m', 'dist_commanded_m', 'duration_s',
    'roll_start_deg', 'pitch_start_deg', 'roll_end_deg', 'pitch_end_deg',
    'roll_peak_abs_deg', 'pitch_peak_abs_deg', 'tilt_samples', 'tilt_errors',
]


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


_CSV_LOCK = threading.Lock()


def _append_csv(name, header, rows):
    """Aggiunge righe a un CSV nella cartella dei log (intestazione alla prima scrittura). Mai eccezioni."""
    folder = _MISSION_CTX.get('log_folder')
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


def _log_segment(row_dict):
    seg_log = _MISSION_CTX.get('segment_log')
    if seg_log is not None:
        seg_log.write_row(row_dict)
        MISSION_STATS['segments_logged'] += 1


def _segment_diagnostics(terrain_2d, origin_x, origin_y, cell_size, x1, y1, x2, y2):
    """Stima alternativa (interpolata) e copertura dei campioni per un segmento. Solo log:
    la stima in uso per decidere resta quella di PRM.sample_terrain_between_points."""
    out = {'long_interp': None, 'lat_interp': None, 'coverage_along': None, 'coverage_lateral': None}
    try:
        li, lai, ok = spotGrid.compute_arc_slope_profile_interp(
            terrain_2d, origin_x, origin_y, cell_size, x1, y1, x2, y2)
        if ok:
            out['long_interp'], out['lat_interp'] = li, lai
        out['coverage_along'], out['coverage_lateral'] = spotGrid.compute_arc_sample_coverage(
            terrain_2d, origin_x, origin_y, cell_size, x1, y1, x2, y2)
    except Exception as e:
        print(f"[LOG] Diagnostica del segmento non disponibile: {e}")
    return out


def _raw_zero(grids_data):
    """Quota del valore grezzo 0 del layer terrain di questa scansione (None se assente)."""
    try:
        return grids_data['terrain'].get('raw_zero')
    except (KeyError, TypeError, AttributeError):
        return None


def _raw_scale(grids_data):
    try:
        return grids_data['terrain'].get('scale')
    except (KeyError, TypeError, AttributeError):
        return None


# ==================================================================
# SPOSTAMENTI CON GRAPHNAV -- fra celle lontane, per i ritorni e per il ritorno a wp_0.
# Questo file non usa RecordingInterface.navigate_to_waypoint / navigate_to_first_waypoint
# di navGraphUtils.py, ma _graphnav_navigate_to, che:
#   - ha un tempo massimo (GRAPHNAV_TIMEOUT_S), cosi' un robot caduto o senza percorso non
#     blocca la missione;
#   - limita la velocita' (GRAPHNAV_MAX_*_VEL): gli spostamenti con GraphNav sono i piu'
#     lunghi della missione;
#   - tratta STATUS_LOST come fallimento, a robot fermo, dopo i tentativi di riaggancio: non
#     forza la localizzazione sul waypoint di destinazione dichiarando "arrivato" quando il
#     robot potrebbe trovarsi altrove.
# ==================================================================
MAX_FALLS_PER_MISSION = 3      # alla terza caduta il robot si rialza e la missione si ferma
GRAPHNAV_TIMEOUT_S = 240.0
GRAPHNAV_MAX_LINEAR_VEL_MPS = 0.5
GRAPHNAV_MAX_ANGULAR_VEL_RPS = 0.6
_GN = _graph_nav_pb2.NavigationFeedbackResponse
_GRAPHNAV_FAIL_STATUSES = {
    _GN.STATUS_STUCK: "bloccato (STUCK)", _GN.STATUS_NO_ROUTE: "nessun percorso nel grafo",
    _GN.STATUS_NO_LOCALIZATION: "non localizzato", _GN.STATUS_ROBOT_IMPAIRED: "robot in difficolta' (caduto?)",
    _GN.STATUS_CONSTRAINT_FAULT: "vincolo violato", _GN.STATUS_COMMAND_OVERRIDDEN: "comando sostituito",
    _GN.STATUS_NOT_LOCALIZED_TO_ROUTE: "non localizzato sul percorso", _GN.STATUS_LEASE_ERROR: "errore di lease",
    _GN.STATUS_AREA_CALLBACK_ERROR: "errore di area callback",
}


# --- LOCALIZZAZIONE DI GRAPHNAV --------------------------------------------------------
# All'aperto il localizzatore dal vivo di GraphNav si puo' perdere anche quando la mappa e'
# sana (odometria delle zampe e visione d'accordo entro ~15 cm su decine di metri), e un
# robot perso si vede rifiutare ogni spostamento. Quindi:
#   1. riga [LOCALIZZAZIONE] a ogni briciola, solo quando qualcosa cambia, per vedere QUANDO
#      e PERCHE' si perde (rilevatore di perdita di Spot);
#   2. riaggancio: se GraphNav risulta perso, si rilocalizza sul waypoint piu' vicino secondo
#      le nostre pose, con la stima iniziale dall'odometria (waypoint_tform_ko del grafo per
#      ko_tform_body di adesso: esatta a meno della deriva), poi la rifinisce l'ICP di Spot.
#      NON si dichiara "arrivato": si riprova a navigare.
#   3. rilevatore di perdita PERMISSIVO negli spostamenti GraphNav.
GRAPHNAV_LOST_DETECTOR_STRICTNESS = 'LOST_DETECTOR_STRICTNESS_PERMISSIVE'   # None = quella di Spot
GRAPHNAV_MAX_RELOCALIZATIONS = 3       # riagganci per singolo spostamento GraphNav
GRAPHNAV_RELOC_MAX_DIST_M = 0.5        # raggio di ricerca dell'ICP attorno alla stima
GRAPHNAV_RELOC_MAX_YAW_RAD = float(np.radians(20.0))
GRAPHNAV_RELOC_MAX_WP_DIST_M = 3.0     # oltre questa distanza dal waypoint piu' vicino non si prova


def _graphnav_localization(recordingInterface):
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


def _graphnav_is_lost(loc):
    return bool(loc) and (loc.get('is_lost') is True or ('error' not in loc and not loc.get('wp_id')))


def _wp_name_by_id(recordingInterface, wp_id):
    for name, p in (getattr(recordingInterface, 'waypoint_poses', None) or {}).items():
        if p.get('waypoint_id') == wp_id:
            return name
    return (wp_id or '?')[:12]


def graphnav_log_localization(env, recordingInterface, robot_state_client, where, force=False):
    """
    Riga [LOCALIZZAZIONE]: stampata solo se lo stato cambia (perso si'/no, localizzazioni
    rifiutate dall'ultima accettata, archi cattivi di fila) o con force=True. Riporta anche
    il tier del passo appena fatto e il beccheggio del corpo adesso, per vedere se la perdita
    scatta in crawl o in salita. Non solleva mai.
    """
    if recordingInterface is None:
        return None
    loc = _graphnav_localization(recordingInterface)
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
    stato = ("PERSO" if loc.get('is_lost') else
             (f"localizzato su {_wp_name_by_id(recordingInterface, loc['wp_id'])}" if loc.get('wp_id')
              else "NON localizzato"))
    print(f"[LOCALIZZAZIONE] {where}: {stato} | {det} | ultimo passo {MISSION_STATS.get('last_tier')}, "
          f"beccheggio {pitch:.1f} gradi.")
    return loc


def _graphnav_relocalize(recordingInterface, robot_state_client, why=""):
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
        if d_wp > GRAPHNAV_RELOC_MAX_WP_DIST_M:
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
            max_distance=GRAPHNAV_RELOC_MAX_DIST_M,
            max_yaw=GRAPHNAV_RELOC_MAX_YAW_RAD,
            fiducial_init=_graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL)
        MISSION_STATS['graphnav_relocalizations'] = MISSION_STATS.get('graphnav_relocalizations', 0) + 1
        print(f"[LOCALIZZAZIONE] Riaggancio {why}: riuscito su {name} (a {d_wp:.2f} m secondo le "
              f"nostre pose; stima dall'odometria, rifinita dall'ICP).")
        return True
    except Exception as e:
        MISSION_STATS['graphnav_relocalization_fail'] = MISSION_STATS.get('graphnav_relocalization_fail', 0) + 1
        print(f"[LOCALIZZAZIONE] Riaggancio {why}: non riuscito ({e}).")
        return False


def _segment_seen_free(global_map, x1, y1, x2, y2, margin_m, step_m=0.05):
    """True se il segmento e' stato osservato in ogni punto e non passa vicino a celle occupate."""
    n = max(2, int(np.ceil(np.hypot(x2 - x1, y2 - y1) / step_m)) + 1)
    for t in np.linspace(0.0, 1.0, n):
        x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
        if not global_map.is_known(x, y) or global_map.is_occupied(x, y, safety_margin=margin_m):
            return False
    return True


def _graphnav_is_recording(recordingInterface):
    """True/False dalla registrazione GraphNav, None se non si riesce a saperlo."""
    try:
        return bool(recordingInterface._recording_client.get_record_status().is_recording)
    except Exception:
        return None


def _graph_route(adj, start, goal):
    """Dijkstra su {wp: {vicino: lunghezza}}: (lunghezza, [wp...]) o (None, None)."""
    import heapq
    dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, float('inf')):
            continue
        for v, w in adj.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, float('inf')):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        return None, None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return dist[goal], path[::-1]


def plan_graphnav_shortcuts(poses, base_edges, start_id, goal_id, global_map,
                            max_m=None, margin_m=None):
    """
    Scorciatoie da aggiungere al grafo GraphNav per andare da start_id a goal_id.
    Funzione pura (niente robot), provata offline sui grafi scaricati: vedi test_replay.py.

    poses: {wp_id: (x, y)} dei waypoint di cui si conosce la posizione (frame della mappa
        globale); base_edges: [(wp_a, wp_b, lunghezza_m)] gli archi che il grafo ha gia'.
    Candidate: coppie di waypoint con posizione a meno di max_m, senza arco diretto, la cui
    linea retta e' stata VISTA libera nella mappa globale con margin_m (_segment_seen_free).
    Si calcola la strada piu' corta col grafo piu' TUTTE le candidate, e si restituiscono solo
    le candidate che quella strada usa.

    Restituisce (scorciatoie [(a, b, d)], lunghezza oggi, lunghezza con le scorciatoie); le
    lunghezze sono None se goal non e' raggiungibile.
    """
    max_m = GRAPHNAV_SHORTCUT_MAX_M if max_m is None else max_m
    margin_m = GRAPHNAV_SHORTCUT_MARGIN_M if margin_m is None else margin_m
    adj = {}
    for a, b, w in base_edges:
        w = float(w)
        if w < adj.get(a, {}).get(b, float('inf')):
            adj.setdefault(a, {})[b] = w
            adj.setdefault(b, {})[a] = w
    before, _ = _graph_route(adj, start_id, goal_id)
    ids = list(poses)
    shortcut = {}
    for i in range(len(ids)):
        xa, ya = poses[ids[i]]
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if b in adj.get(a, {}):
                continue
            xb, yb = poses[b]
            d = float(np.hypot(xa - xb, ya - yb))
            if d > max_m or d < 0.05:
                continue
            if _segment_seen_free(global_map, xa, ya, xb, yb, margin_m):
                shortcut[frozenset((a, b))] = d
    full = {k: dict(v) for k, v in adj.items()}
    for k, d in shortcut.items():
        a, b = tuple(k)
        full.setdefault(a, {})[b] = d
        full.setdefault(b, {})[a] = d
    after, path = _graph_route(full, start_id, goal_id)
    used = []
    if path:
        for a, b in zip(path[:-1], path[1:]):
            k = frozenset((a, b))
            if k in shortcut and b not in adj.get(a, {}):
                used.append((a, b, shortcut[k]))
    return used, before, after


def graphnav_add_shortcuts(recordingInterface, label="", map_edges=True, sdk=True,
                           target_wp_id=None, robot_xy=None):
    """
    Aggiunge scorciatoie al grafo GraphNav prima di uno spostamento (vedi GRAPHNAV_SHORTCUTS).
    Non solleva mai: senza scorciatoie GraphNav funziona lo stesso, sulla catena.

    Spot accetta archi nuovi (create_edge) SOLO mentre registra ("NotRecordingError"),
    mentre la chiusura d'anello (process_topology) va fatta a registrazione ferma, e appena
    fermata puo' rispondere "MapModifiedError" finche' la mappa non e' finita di elaborare.
    Quindi:
      map_edges -- le scorciatoie dalla mappa globale: da chiamare PRIMA di stop_recording
                   (si saltano da sole se la registrazione e' ferma). Servono
                   target_wp_id (la destinazione) e, se il localizzatore non dice dove si
                   trova il robot, robot_xy; si creano solo quelle sulla strada (vedi
                   plan_graphnav_shortcuts).
      sdk       -- la chiusura d'anello di Spot: da chiamare DOPO stop_recording (si salta da
                   sola se si sta registrando), con qualche tentativo su MapModifiedError.
    """
    if not GRAPHNAV_SHORTCUTS or recordingInterface is None:
        return 0
    added = 0
    recording = _graphnav_is_recording(recordingInterface)
    gg = _MISSION_CTX.get('global_grid')
    global_map = getattr(gg, 'global_occupancy_map', None) if gg is not None else None
    poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
    if (map_edges and target_wp_id and recording is not False and global_map is not None
            and len(poses) >= 2):
        try:
            graph = recordingInterface._get_graph(force_refresh=True)
            by_id = {w.id: w for w in graph.waypoints}
            xy = {p['waypoint_id']: (float(p['x']), float(p['y'])) for p in poses.values()
                  if p.get('waypoint_id') in by_id}
            base = []
            for e in graph.edges:
                t = e.from_tform_to.position
                base.append((e.id.from_waypoint, e.id.to_waypoint,
                             float(np.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2))))
            start = (_graphnav_localization(recordingInterface) or {}).get('wp_id')
            if start not in by_id:
                if robot_xy is not None and xy:
                    start = min(xy, key=lambda k: np.hypot(xy[k][0] - robot_xy[0], xy[k][1] - robot_xy[1]))
                else:
                    start = None
            if start is not None and target_wp_id in by_id:
                used, before, after = plan_graphnav_shortcuts(xy, base, start, target_wp_id, global_map)
                for a, b, d in used:
                    try:
                        recordingInterface.create_edge_between_waypoint_objs(by_id[a], by_id[b])
                        added += 1
                    except Exception as e:
                        print(f"[GRAPHNAV] Scorciatoia {_wp_name_by_id(recordingInterface, a)}-"
                              f"{_wp_name_by_id(recordingInterface, b)} rifiutata ({e}).")
                if before is not None and after is not None:
                    print(f"[GRAPHNAV] {label}: strada verso {_wp_name_by_id(recordingInterface, target_wp_id)} "
                          f"{before:.1f} m -> {after:.1f} m con {len(used)} scorciatoie "
                          f"(fino a {GRAPHNAV_SHORTCUT_MAX_M:.1f} m, viste libere con "
                          f"{GRAPHNAV_SHORTCUT_MARGIN_M:.2f} m di margine).")
        except Exception as e:
            print(f"[GRAPHNAV] Scorciatoie dalla mappa non aggiunte ({e}).")
        finally:
            if added:
                recordingInterface.invalidate_graph_cache()
    n_sdk = 0
    if sdk and GRAPHNAV_SDK_LOOP_CLOSURE and recording is not True:
        for attempt in range(1, 4):
            try:
                n_sdk = recordingInterface.close_loops_checked(max_edge_length_m=GRAPHNAV_SDK_LOOP_MAX_M)
                break
            except Exception as e:
                if 'MapModified' in type(e).__name__ or 'modified' in str(e):
                    time.sleep(1.0)      # mappa ancora in elaborazione dopo stop_recording
                    continue
                print(f"[GRAPHNAV] Chiusura d'anello di Spot non riuscita ({e}).")
                break
        else:
            print("[GRAPHNAV] Chiusura d'anello di Spot: la mappa risulta ancora in "
                  "elaborazione dopo 3 tentativi, proseguo senza.")
    if added or n_sdk:
        MISSION_STATS['graphnav_shortcuts'] = MISSION_STATS.get('graphnav_shortcuts', 0) + added + n_sdk
        print(f"[GRAPHNAV] {label}: {added} scorciatoie dalla mappa globale + {n_sdk} da Spot "
              f"(chiusure d'anello con controllo di collisione).")
    return added + n_sdk


def _graphnav_navigate_to(recordingInterface, waypoint_id, robot_state_client, command_client, label,
                          timeout_s=GRAPHNAV_TIMEOUT_S):
    """Porta il robot a un waypoint con GraphNav. True solo se ci arriva davvero."""
    # Prima di muoversi: chiusure d'anello di Spot (a registrazione ferma). Le scorciatoie
    # dalla mappa le aggiunge chi chiama, PRIMA di fermare la registrazione.
    # Tutto cio' che sta qui dentro finisce nella voce 'GraphNav' di [TEMPO]: e' una delle
    # fasi piu' lunghe della missione (il solo ritorno a wp_0 puo' durare minuti).
    _t_gn = time.perf_counter()
    try:
        return _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client,
                                           command_client, label, timeout_s)
    finally:
        _timing_add('GraphNav', time.perf_counter() - _t_gn)


def _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client, command_client,
                                label, timeout_s):
    graphnav_add_shortcuts(recordingInterface, label, map_edges=False, sdk=True)
    client = recordingInterface._graph_nav_client
    tp = _graph_nav_pb2.TravelParams()
    tp.velocity_limit.CopyFrom(movements._velocity_limit(GRAPHNAV_MAX_LINEAR_VEL_MPS, GRAPHNAV_MAX_ANGULAR_VEL_RPS))
    # Rilevatore di perdita permissivo (vedi LOCALIZZAZIONE DI GRAPHNAV).
    if GRAPHNAV_LOST_DETECTOR_STRICTNESS and hasattr(tp, 'lost_detector_strictness'):
        try:
            # dal descrittore: l'enum sta in lost_detection_pb2, che nelle versioni vecchie
            # dell'SDK non c'e'; cosi' funziona ovunque il campo esista.
            enum = tp.DESCRIPTOR.fields_by_name['lost_detector_strictness'].enum_type
            tp.lost_detector_strictness = enum.values_by_name[GRAPHNAV_LOST_DETECTOR_STRICTNESS].number
        except Exception:
            pass
    print(f"[GRAPHNAV] {label}: avvio (massimo {timeout_s:.0f} s, {GRAPHNAV_MAX_LINEAR_VEL_MPS:.1f} m/s).")
    # Prima di partire si guarda lo stato del localizzatore; se e' perso si riaggancia
    # subito, invece di farsi rifiutare il comando.
    relocs = 0
    loc0 = graphnav_log_localization(None, recordingInterface, robot_state_client,
                                     f"prima di '{label}'", force=True)
    if _graphnav_is_lost(loc0):
        relocs += 1
        _graphnav_relocalize(recordingInterface, robot_state_client, f"prima di '{label}'")
    t0, cmd_id, fb_errors, nav_errors = time.time(), None, 0, 0
    while time.time() - t0 < timeout_s:
        try:
            cmd_id = client.navigate_to(waypoint_id, 1.0, command_id=cmd_id, travel_params=tp)
            nav_errors = 0
        except Exception as e:
            if 'RobotLost' in type(e).__name__ and relocs < GRAPHNAV_MAX_RELOCALIZATIONS:
                relocs += 1
                print(f"[GRAPHNAV] {label}: GraphNav dice che il robot e' perso: riaggancio "
                      f"({relocs}/{GRAPHNAV_MAX_RELOCALIZATIONS}).")
                _graphnav_relocalize(recordingInterface, robot_state_client, f"durante '{label}'")
                cmd_id = None
                time.sleep(0.5)
                continue
            nav_errors += 1
            print(f"[GRAPHNAV] {label}: comando rifiutato ({nav_errors}/3): {e}")
            # Un errore RPC transitorio non abbandona lo spostamento: si rinuncia al terzo.
            if nav_errors >= 3:
                movements.stop_robot(command_client, reason="GraphNav: comando rifiutato")
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
                movements.stop_robot(command_client, reason="GraphNav: feedback non disponibile")
                MISSION_STATS['graphnav_fail'] += 1
                return False
            cmd_id = None      # il comando potrebbe non essere piu' valido: se ne invia uno nuovo
            time.sleep(0.5)
            continue
        if status == _GN.STATUS_REACHED_GOAL:
            print(f"[GRAPHNAV] {label}: arrivato in {time.time() - t0:.0f} s.")
            MISSION_STATS['graphnav_ok'] += 1
            return True
        if status == _GN.STATUS_LOST and relocs < GRAPHNAV_MAX_RELOCALIZATIONS:
            # Riaggancio sul waypoint piu' vicino (NON sulla destinazione, e senza dichiarare
            # l'arrivo: si rimanda il comando e si continua a navigare).
            relocs += 1
            print(f"[GRAPHNAV] {label}: GraphNav ha perso la localizzazione per strada: "
                  f"riaggancio ({relocs}/{GRAPHNAV_MAX_RELOCALIZATIONS}).")
            graphnav_log_localization(None, recordingInterface, robot_state_client,
                                      f"perso durante '{label}'", force=True)
            _graphnav_relocalize(recordingInterface, robot_state_client, f"durante '{label}'")
            cmd_id = None
            continue
        if status == _GN.STATUS_LOST:
            # Non si forza la localizzazione sulla destinazione (vedi sopra): fallimento.
            print(f"[GRAPHNAV] {label}: GraphNav ha perso la localizzazione. Fermo il robot.")
            movements.stop_robot(command_client, reason="GraphNav: robot perso")
            MISSION_STATS['graphnav_fail'] += 1
            return False
        if status == _GN.STATUS_COMMAND_TIMED_OUT:
            cmd_id = None          # comando scaduto: se ne invia uno nuovo
            continue
        if status in _GRAPHNAV_FAIL_STATUSES:
            print(f"[GRAPHNAV] {label}: interrotto -- {_GRAPHNAV_FAIL_STATUSES[status]}.")
            movements.stop_robot(command_client, reason=f"GraphNav: {_GRAPHNAV_FAIL_STATUSES[status]}")
            MISSION_STATS['graphnav_fail'] += 1
            return False
    print(f"[GRAPHNAV] {label}: tempo massimo di {timeout_s:.0f} s superato. Fermo il robot.")
    movements.stop_robot(command_client, reason="GraphNav: tempo massimo superato")
    MISSION_STATS['graphnav_fail'] += 1
    return False


def _graphnav_return_to_start(recordingInterface, robot_state_client, command_client):
    """Ritorno a wp_0 a fine missione, con tempo massimo. False se non ci riesce."""
    try:
        graph = recordingInterface._get_graph(force_refresh=True)
        wp0 = next((w for w in graph.waypoints if w.annotations.name == "wp_0"), None)
    except Exception as e:
        print(f"[GRAPHNAV] Ritorno a wp_0: grafo non disponibile ({e}).")
        return False
    if wp0 is None:
        print("[GRAPHNAV] Ritorno a wp_0: waypoint wp_0 non trovato nel grafo.")
        return False
    return _graphnav_navigate_to(recordingInterface, wp0.id, robot_state_client, command_client,
                                 "ritorno a wp_0", timeout_s=2 * GRAPHNAV_TIMEOUT_S)


# --- Zona attorno al corpo nella mappa GLOBALE ------------------------------------------
# Le celle a ridosso del fianco del robot risultano "ostacolo" SOLO quando il robot ci sta
# accanto: lette da 1-2 m sono libere, lette con il robot fermo li' accanto scendono sotto
# soglia, a 0.2-0.45 m dall'asse, all'altezza delle zampe posteriori. Due scansioni da fermo
# basterebbero a confermarle nella mappa globale (GLOBAL_OCC_CONFIRM_OBS = 2), e il PRM
# scarterebbe l'arco su cui il robot e' APPENA passato.
#   - NUCLEO (il rettangolo del corpo): il robot ci sta sopra, quindi e' libero per
#     costruzione -> si scrive "visto libero".
#   - ANELLO (fino a GLOBAL_MAP_SELF_RING_M oltre il corpo): da qui la lettura non e'
#     attendibile -> un ostacolo NON si scrive (nessuna informazione); il libero si'.
# Vale solo per la mappa globale: il fronte sicuro usa la scansione dal vivo, intera, quindi
# un ostacolo vero accanto al robot continua a fermarlo. E un ostacolo vero viene comunque
# scritto nella mappa globale dalle scansioni fatte da un altro punto.
GLOBAL_MAP_SELF_BODY_LENGTH_M = 1.10
GLOBAL_MAP_SELF_BODY_WIDTH_M = 0.50
GLOBAL_MAP_SELF_RING_M = 0.25


def _mask_for_global_map(obstacle_mask, is_valid, pts=None, robot_pose=None,
                         return_core=False):
    """
    Maschera da passare a GlobalGrid.update: -1 ostacolo, +1 vista libera, 0 nessuna
    informazione (cella non attendibile in questa scansione, che quindi non conta come
    "vista libera": vedi GlobalGrid.update).

    pts, robot_pose=(x, y, yaw): se passati, applica la zona attorno al corpo (vedi
    GLOBAL_MAP_SELF_RING_M): nucleo libero, anello senza ostacoli.

    return_core: restituisce anche la maschera del NUCLEO, cioe' le celle
    coperte dal corpo adesso. Va passata a GlobalGrid.update come `walked`: sono celle su
    cui il robot e' fisicamente passato, e per dichiararle occupate serviranno piu'
    conferme (vedi spotGrid.GLOBAL_OCC_CONFIRM_WALKED).
    """
    m = np.asarray(obstacle_mask, dtype=np.float64).ravel()
    v = np.asarray(is_valid, dtype=bool).ravel()
    out = np.where(m < 0, -1.0, np.where(v, 1.0, 0.0))
    if pts is None or robot_pose is None:
        return (out, np.zeros(out.shape, dtype=bool)) if return_core else out
    rx, ry, ryaw = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    if len(p) != len(out):
        return (out, np.zeros(out.shape, dtype=bool)) if return_core else out
    dx, dy = p[:, 0] - rx, p[:, 1] - ry
    c, s_ = np.cos(ryaw), np.sin(ryaw)
    xb, yb = np.abs(dx * c + dy * s_), np.abs(-dx * s_ + dy * c)
    hl, hw = GLOBAL_MAP_SELF_BODY_LENGTH_M / 2.0, GLOBAL_MAP_SELF_BODY_WIDTH_M / 2.0
    core = (xb <= hl) & (yb <= hw)
    ring = ~core & (xb <= hl + GLOBAL_MAP_SELF_RING_M) & (yb <= hw + GLOBAL_MAP_SELF_RING_M)
    out[ring & (out < 0)] = 0.0
    out[core] = 1.0
    return (out, core) if return_core else out


def fold_scan_into_global_maps(global_map, global_terrain_map, pts, terrain, obstacle_mask, is_valid,
                               cells_obstacle_dist, shape, robot_pose):
    """
    Una scansione nelle due mappe globali: occupazione (con la zona attorno al
    corpo, vedi _mask_for_global_map) e terreno. Unico punto usato dalla missione (inizio
    tentativo, ogni giro, scansione finale) e da test_replay.py, cosi' la prova di rigioco
    verifica il codice vero e non una sua copia.

    La mappa del terreno riceve la STESSA maschera del calcolo dal vivo (slope_usable_mask:
    attendibili E non ostacolo): le quote delle celle ostacolo trasformerebbero tronchi e
    muri in "pendenze" nella mappa globale.

    Restituisce la maschera del nucleo (celle sotto il corpo), che serve alla mappa dello
    spazio libero.
    """
    gm_mask, gm_core = _mask_for_global_map(obstacle_mask, is_valid, pts, robot_pose, return_core=True)
    global_map.update(pts, gm_mask, walked=gm_core)
    global_terrain_map.update(pts, terrain, slope_usable_mask(is_valid, cells_obstacle_dist, shape))
    return gm_core


def _update_clearance_map(global_map, pts, snap, is_valid, unwritten, core, robot_xy):
    """
    Alimenta la mappa dello spazio libero con la scansione corrente.

    Usa la obstacle_distance FILTRATA (quella su cui il fronte decide davvero, mediana +
    isteresi: vedi arcVerification), non la lettura grezza. Imparano solo le celle
    attendibili; sotto il corpo il valore non puo' essere minore di mezza larghezza, perche'
    il robot ci sta sopra (stesso principio di _under_body nel fronte).
    """
    if not hasattr(global_map, 'update_clearance') or snap is None:
        return
    try:
        od = np.asarray(snap.obstacle_dist, dtype=np.float64).ravel()
        v = np.asarray(is_valid, dtype=bool).ravel()
        if unwritten is not None and np.size(unwritten) == v.size:
            v = v & ~np.asarray(unwritten, dtype=bool).ravel()
        c = np.asarray(core, dtype=bool).ravel()
        if od.size != v.size or c.size != v.size:
            return
        usable = v | c
        od = np.where(c, np.maximum(od, spotGrid.ROBOT_HALF_WIDTH_M), od)
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        near = np.hypot(p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1]) <= spotGrid.CLEARANCE_NEAR_M
        global_map.update_clearance(p, od, usable, near=near)
    except Exception as e:      # mai fatale: e' una mappa di appoggio
        print(f"[SPAZIO] Mappa dello spazio libero non aggiornata: {e}")


def _log_clearance_gap(prm, global_map):
    """
    Una riga per giro sul divario fra i due criteri di percorribilita':
    la mappa binaria (bit occupato + margine) e la mappa dello spazio libero (stessa soglia
    del fronte). Tace quando sono d'accordo. Serve a decidere sul campo se commutare
    spotGrid.CLEARANCE_MAP_DECIDES.
    """
    d = getattr(prm, 'clearance_disagreement', None)
    if not d:
        return
    only_bin, only_clear, tot = d
    if not (only_bin or only_clear) or not tot:
        return
    chi = "lo spazio libero" if spotGrid.CLEARANCE_MAP_DECIDES else "la mappa binaria"
    n_cells = len(getattr(global_map, 'clear_all_n', ()) or ())
    print(f"[SPAZIO] Archi su cui i due criteri non concordano: {only_bin + only_clear} su "
          f"{tot} ({100.0 * (only_bin + only_clear) / tot:.1f}%) -- "
          f"{only_bin} bloccati solo dalla mappa binaria, {only_clear} solo dallo spazio "
          f"libero. Adesso decide {chi}. Celle con spazio libero noto: {n_cells}.")


def _log_ground_filter(local_grid, label):
    """
    Una riga solo quando il filtro sull'altezza del suolo ha scartato qualcosa o non ha
    potuto stimare il suolo -- nel caso normale tace. Vedi spotGrid.GROUND_HEIGHT_TOLERANCE_M.
    """
    info = getattr(local_grid, 'last_ground_filter', None) or {}
    if info.get('ground_z') is None:
        print(f"[GROUND-FILTER] {label}: suolo non stimabile ({info.get('source')}, "
              f"{info.get('n_ref', 0)} celle di riferimento) -- filtro non applicato")
    elif info.get('n_above', 0) > 0:
        print(f"[GROUND-FILTER] {label}: {info['n_above']} celle scartate, fino a "
              f"{info['max_above']:.2f} m sopra il suolo (suolo z={info['ground_z']:.3f}, "
              f"stimato {info['source']} su {info['n_ref']} celle)")
    if info.get('n_unwritten', 0) > 0:
        print(f"[GROUND-FILTER] {label}: {info['n_unwritten']} celle mai scritte dai sensori "
              f"({info.get('n_unwritten_below', 0)} sotto il suolo) -> trattate come non viste"
              + (f", quota {info['quota_unwritten']:+.4f} m" if info.get('quota_unwritten') is not None else ""))


def _log_footprint_diagnostic(local_grid, pts, robot_x, robot_y, robot_yaw,
                              num_x, num_y, obstacle_mask, label,
                              cells_obstacle_dist=None, rough_values=None, is_valid=None):
    """
    Misura quante celle sotto l'impronta del robot risultano ostacolo, e PERCHE'.
    PURAMENTE OSSERVATIVO: usa diagnostic_only=True, quindi non consuma il flag
    one-shot di compute_robot_footprint_mask e non cambia nessun comportamento.

    La maschera dell'impronta si applica una volta sola: applicarla a ogni scansione
    nasconderebbe muri veri in un corridoio stretto. Celle ostacolo NEL NUCLEO, quando
    compaiono, sono di solito celle mai scritte dai sensori (sotto il corpo non possono
    scrivere) lasciate alla quota sbagliata, che fanno esplodere la rugosita': le toglie
    spotGrid._cells_never_written, non questa maschera.

    PERCHE' -- i due veti di fuse_obstacle_mask, ricalcolati identici:
      - "prossimita'": cells_obstacle_dist <= OBSTACLE_THRESHOLD. NON vuol dire
        "qui c'e' un ostacolo", vuol dire "qui sei a meno di 15 cm da un ostacolo".
        Se domina questo, il footprint NON c'entra: il robot sta davvero passando
        vicino a qualcosa, e mascherare cancellerebbe proprio quell'avviso.
      - "rugosita'": rough_values > ROUGH_THRESHOLD su celle che il sensore dichiara
        valide.

    DOVE -- rettangolo intero (1.10 x 0.50 + margine) contro il solo NUCLEO
    (0.90 x 0.35), cioe' la fascia entro la carreggiata delle zampe. Un ostacolo
    dentro il nucleo significherebbe che il robot ci sta gia' sopra, quindi mascherare li'
    non puo' nascondere nulla di raggiungibile, mentre mascherare l'intero rettangolo in un
    corridoio stretto puo' cancellare un muro vero.

    Nota: anche arcVerification.py chiama compute_robot_footprint_mask, quindi in
    condizioni di corsa puo' essere il thread di background a consumare l'unica
    maschera prevista, lasciando senza il ciclo principale.
    """
    try:
        fp = local_grid.compute_robot_footprint_mask(
            pts, robot_x, robot_y, robot_yaw, num_x, num_y, diagnostic_only=True)
        if fp is None:
            return
        fp_flat = fp.ravel()
        obs_flat = np.asarray(obstacle_mask).ravel()
        n_fp = int(fp_flat.sum())
        if n_fp == 0:
            return
        blocked = (obs_flat < 0) & fp_flat
        n_obs = int(blocked.sum())

        line = (f"[FOOTPRINT-DIAG] {label}: celle sotto il corpo marcate ostacolo: "
                f"{n_obs}/{n_fp} ({100.0 * n_obs / n_fp:.1f}%)")

        # --- PERCHE': i due veti separati, stessa identica logica di fuse_obstacle_mask ---
        if n_obs > 0 and cells_obstacle_dist is not None and rough_values is not None \
                and is_valid is not None:
            d = np.asarray(cells_obstacle_dist).ravel()
            r = np.asarray(rough_values).ravel()
            v = np.asarray(is_valid).ravel()
            if d.shape == fp_flat.shape and r.shape == fp_flat.shape and v.shape == fp_flat.shape:
                v1 = (d <= spotGrid.OBSTACLE_THRESHOLD) & v   # prossimita' a un ostacolo
                v3 = (r > spotGrid.ROUGH_THRESHOLD) & v       # terreno irregolare
                n_prox = int(np.sum(v1 & fp_flat & ~v3))
                n_rough = int(np.sum(v3 & fp_flat & ~v1))
                n_both = int(np.sum(v1 & v3 & fp_flat))
                n_invalid = int(np.sum(~v & fp_flat))
                line += (f"\n                 perche': prossimita'={n_prox}  rugosita'={n_rough}  "
                         f"entrambi={n_both}   (celle non valide sotto il corpo: {n_invalid})")

        # --- DOVE: nucleo (dentro la carreggiata delle zampe) contro corona esterna ---
        if n_obs > 0:
            core = local_grid.compute_robot_footprint_mask(
                pts, robot_x, robot_y, robot_yaw, num_x, num_y,
                diagnostic_only=True, body_length=0.90, body_width=0.35)
            if core is not None:
                core_flat = core.ravel()
                n_core_fp = int(core_flat.sum())
                n_core_obs = int(np.sum(blocked & core_flat))
                n_ring_obs = n_obs - n_core_obs
                line += (f"\n                 dove: nucleo={n_core_obs}/{n_core_fp}  "
                         f"corona esterna={n_ring_obs}/{n_fp - n_core_fp}")

        print(line)
    except Exception as e:
        print(f"[FOOTPRINT-DIAG] diagnostica non disponibile: {e}")


def mission_switches():
    """
    Gli interruttori che cambiano come lavora la missione, con il loro valore adesso. Vanno nel
    log ([CONFIG] INTERRUTTORI) e in mission_summary.json, cosi' dopo una prova si sa con quali
    impostazioni e' stata fatta ogni missione (per i confronti acceso/spento).
    """
    return {
        'easy_walk.MISSION_GRAPH_SAMPLING': MISSION_GRAPH_SAMPLING,
        'easy_walk.INCREMENTAL_GRAPH_UPDATE': INCREMENTAL_GRAPH_UPDATE,
        'mission_graph.BATCH_SLOPES': mission_graph.BATCH_SLOPES,
        'prm_graph.REFRESH_BATCH_SLOPES': prm_graph_module.REFRESH_BATCH_SLOPES,
        'spotGrid.CLEARANCE_MAP_DECIDES': spotGrid.CLEARANCE_MAP_DECIDES,
        'easy_walk.PLAN_ONLY_EXPLORED_CELLS': PLAN_ONLY_EXPLORED_CELLS,
        'easy_walk.GRAPHNAV_SHORTCUTS': GRAPHNAV_SHORTCUTS,
        'easy_walk.GRAPHNAV_SDK_LOOP_CLOSURE': GRAPHNAV_SDK_LOOP_CLOSURE,
        'easy_walk.SAVE_SCAN_BUNDLES': SAVE_SCAN_BUNDLES,
        'movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT': movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT,
        'spotGrid.ARC_BODY_EXTENSION_M': spotGrid.ARC_BODY_EXTENSION_M,
    }


def _emit_mission_summary_once():
    """Stampa e salva il riepilogo di missione una sola volta. Viene chiamata sia a fine
    missione normale sia da main() in un blocco finally, cosi' esce anche se la missione
    termina per un'eccezione o per una ripresa del controllo."""
    if _MISSION_CTX.get('summary_done') or _MISSION_CTX.get('prm') is None:
        return
    _MISSION_CTX['summary_done'] = True
    try:
        prm = _MISSION_CTX['prm']
        prm.print_mission_summary()
        total_gait = sum(MISSION_STATS['gait_tier_counts'].values())
        print("================ [MISSION STATS] ================")
        print(f"Distribuzione tier di gait (totale {total_gait} passi):")
        for tier, count in MISSION_STATS['gait_tier_counts'].items():
            pct = (100.0 * count / total_gait) if total_gait > 0 else 0.0
            print(f"  {tier:10s}: {count:6d}  ({pct:5.1f}%)")
        print(f"Ripianificazioni totali: {MISSION_STATS['replan_count']}")
        print(f"Segmenti registrati nel CSV: {MISSION_STATS['segments_logged']}")
        print(f"Salvataggi .npz per archi vicino soglia: {MISSION_STATS['near_threshold_saves']}")
        print(f"Blocchi confermati dal fronte: {MISSION_STATS['frontier_blocks']}, "
              f"arresti a meta' movimento: {MISSION_STATS['frontier_aborts']}, "
              f"rotazioni rifiutate: {MISSION_STATS['rotation_refusals']}, "
              f"arretramenti dritti: {MISSION_STATS['straight_retreats']}, "
              f"obiettivi riscelti: {MISSION_STATS['goal_repicks']}, "
              f"scorciatoie: {MISSION_STATS['shortcuts']}")
        print(f"Avanzamenti nel cono di rotazione: {MISSION_STATS['cone_moves']}, "
              f"arretramenti evitati perche' inutili: {MISSION_STATS['retreats_suppressed']}, "
              f"ripianificazioni immediate: {MISSION_STATS['early_replans']}")
        print(f"Ritorni con GraphNav: {MISSION_STATS['graphnav_backtracks']} riusciti, "
              f"{MISSION_STATS['graphnav_backtrack_fail']} falliti (ripiego a zampe); "
              f"nodi PRM scartati perche' dentro ostacoli: {MISSION_STATS['nodes_pruned']}")
        print(f"Trappole riconosciute dal ventaglio: {MISSION_STATS['traps_detected']}, "
              f"passi rifiutati per mancanza di uscite all'arrivo: {MISSION_STATS['arrival_refusals']}")
        tot_r = MISSION_STATS['prm_refresh'] + MISSION_STATS['prm_refresh_skipped']
        mn = MISSION_STATS['multinode_moves']
        print(f"Movimenti multinodo: {mn}"
              + (f", {MISSION_STATS['multinode_nodes']} nodi attraversati "
                 f"({MISSION_STATS['multinode_nodes'] / mn:.1f} per movimento)" if mn else "")
              + f"; fermati a un nodo intermedio (rotazione o arrivo): {MISSION_STATS['multinode_cut']}")
        print(f"Refresh degli archi del PRM: {MISSION_STATS['prm_refresh']} fatti, "
              f"{MISSION_STATS['prm_refresh_skipped']} saltati perche' senza eventi"
              + (f" ({100.0 * MISSION_STATS['prm_refresh_skipped'] / tot_r:.0f}% di giri risparmiati)"
                 if tot_r else ""))
        print(f"Cadute: {MISSION_STATS['falls']}, rialzate da solo: {MISSION_STATS['self_rights']}")
        print(f"Spostamenti GraphNav riusciti: {MISSION_STATS['graphnav_ok']}, falliti: {MISSION_STATS['graphnav_fail']}")
        print(f"Riagganci GraphNav (robot perso): {MISSION_STATS.get('graphnav_relocalizations', 0)} riusciti, "
              f"{MISSION_STATS.get('graphnav_relocalization_fail', 0)} falliti | celle rimandate dopo "
              f"{MAX_BACKTRACKS_PER_ATTEMPT} ritorni nello stesso tentativo: {MISSION_STATS.get('backtrack_giveups', 0)} | "
              f"archi tolti dalla scansione dal vivo nei refresh: {MISSION_STATS.get('live_arc_vetoes', 0)}")
        lg_main = _MISSION_CTX.get('local_grid')
        lg_trk = getattr(_MISSION_CTX.get('tracker'), '_local_grid', None)
        print(f"Scansioni scartate (layer mancanti o non allineati): ciclo principale "
              f"{getattr(lg_main, 'rejected_scans', 0)}, controllo in movimento {getattr(lg_trk, 'rejected_scans', 0)}")
        print(f"Pacchetti di scansione salvati: {MISSION_STATS['scan_bundles']}")
        times = MISSION_STATS['time_s']
        if times:
            tot = sum(times.values())
            print(f"[TEMPO] Ripartizione del tempo misurato ({tot:.0f} s):")
            for phase, sec in sorted(times.items(), key=lambda kv: -kv[1]):
                print(f"  {phase:22s} {sec:8.1f} s  ({100.0 * sec / tot:5.1f}%)")
        print("===================================================\n")

        folder = _MISSION_CTX.get('log_folder')
        if folder:
            summary = {
                'prm': prm.get_mission_summary_dict(),
                'mission_stats': {k: v for k, v in MISSION_STATS.items()},
                'lateral_slope_cost_multiplier': spotGrid.LATERAL_SLOPE_COST_MULTIPLIER,
                'slope_threshold': spotGrid.SLOPE_THRESHOLD,
                'near_threshold_margin_fraction': spotGrid.NEAR_THRESHOLD_MARGIN_FRACTION,
                'slope_baseline_m': spotGrid.SLOPE_BASELINE_M,
                'lateral_slice_half_width_m': spotGrid.LATERAL_SLICE_HALF_WIDTH_M,
                'min_arc_sample_fraction': spotGrid.MIN_ARC_SAMPLE_FRACTION,
                'terrain_valid_min': spotGrid.TERRAIN_VALID_MIN,
                'goal_reached_tolerance_m': GOAL_REACHED_TOLERANCE_M,
                'interruttori': mission_switches(),
            }
            mg = getattr(prm, 'mission_graph', None)
            if mg is not None:
                summary['grafo'] = mg.describe()
            with open(os.path.join(folder, 'mission_summary.json'), 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, default=str)
    except Exception as e:
        print(f"[LOG] Impossibile produrre il riepilogo di missione: {e}")

# TODO: verificare se si puo' evitare la pausa dopo ogni comando di movimento
# TODO: cambiare la cartella in cui si scarica il grafo

def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist,
                            sampler, global_map=None, safety_margin=None, exclude=None):
    """
    Sceglie il punto obiettivo dentro la cella: il piu' CENTRALE fra quelli non occupati.

    Regola: il CENTRO GEOMETRICO e' l'obiettivo, libero o ignoto che sia.
    Si cambia solo se si VEDE che e' occupato (mappa globale con safety_margin) o se il
    ciclo di navigazione lo ha gia' scartato sui dati dal vivo (`exclude`). In quel caso:
      1. punti OSSERVATI liberi ad almeno FRONTIER_CLEARANCE_M dagli ostacoli (scansione
         dal vivo, spazio libero imparato, mappa globale), il piu' vicino al centro
      2. se non ce ne sono, punti MAI OSSERVATI, il piu' vicino al centro
      3. (mai scelti) punti occupati o troppo vicini a un ostacolo
    Se non resta nessun punto la cella non ha un obiettivo valido e la funzione
    restituisce (None, None, ...): il chiamante lo tratta gia' come fallimento di
    ingresso e passa alla logica dei lati non ancora provati.

    exclude: lista di punti gia' provati e risultati irraggiungibili, da non
    riproporre. Serve alla riscelta dentro il ciclo di navigazione (vedi
    attempt_enter_cell_from_position): il punto scelto all'inizio del tentativo viene
    deciso da lontano, quando la cella e' ancora tutta ignota, e puo' rivelarsi a ridosso
    di un muro quando ci si arriva. Senza escluderlo, la riscelta ripescherebbe lo stesso.

    global_map: spotGrid.GlobalGrid accumulata. Viene aggiornata con la scansione
    corrente appena prima di questa chiamata, quindi e' la fonte piu' completa
    disponibile -- piu' della sola griglia locale, che copre ~3.8 m e in genere non
    arriva al centro della cella bersaglio. Se None si usa il centro senza controlli.

    Returns:
        (target_x, target_y, valid_samples, rejected_samples)
        Le due liste alimentano la visualizzazione (gialli = ammessi, rossi = scartati).
    """
    # Stesso margine del PRM e del fronte sicuro: un punto obiettivo piu' vicino agli
    # ostacoli di quanto il fronte accetti non verrebbe mai raggiunto.
    if safety_margin is None:
        safety_margin = spotGrid.PRM_EDGE_SAFETY_MARGIN_M
    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center
    sampled = list(sampler.get_point_in_cell(cell_row, cell_col))

    if global_map is None:
        # Nessuna informazione di occupazione: il centro della cella.
        return cell_center_x, cell_center_y, sampled, []

    # REGOLA DEL CENTRO. Il centro geometrico e' l'obiettivo e si cambia SOLO se si VEDE
    # che e' occupato: nella mappa globale, o perche' il ciclo di navigazione lo ha gia'
    # scartato sui dati dal vivo (e' in `exclude`). Se e' ignoto ci si va vicino e si
    # guarda; se e' libero resta lui. Un punto gia' visto libero non batte il centro ignoto:
    # di solito e' l'ultimo lembo scansionato dalla cella precedente, cioe' sul bordo.
    def _near_tried(p):
        return bool(exclude) and any(np.hypot(p[0] - ex, p[1] - ey) < 0.30 for ex, ey in exclude)

    center = (cell_center_x, cell_center_y)
    center_tried = _near_tried(center)
    center_occupied = global_map.is_occupied(cell_center_x, cell_center_y,
                                             safety_margin=safety_margin)
    if not center_tried and not center_occupied:
        state = "libero" if global_map.is_known(cell_center_x, cell_center_y) else "ignoto"
        print(f"[TARGET] Cella ({cell_row},{cell_col}): uso il centro geometrico "
              f"({cell_center_x:.2f}, {cell_center_y:.2f}), {state}.")
        return cell_center_x, cell_center_y, sampled, []

    # Il centro e' occupato: si cerca il punto piu' vicino al centro dove il robot CI STA,
    # cioe' ad almeno FRONTIER_CLEARANCE_M (0.30 m) dagli ostacoli, la stessa distanza che
    # pretende il fronte. La mappa globale puo' non aver ancora visto tutto l'ostacolo, e un
    # punto a mezzo metro dal centro scartato puo' cadere ancora dentro: per questo si guarda
    # anche la scansione dal vivo (pts, cells_obstacle_dist) e lo spazio libero imparato
    # (global_map.clearance).
    min_clear = spotGrid.FRONTIER_CLEARANCE_M
    live_od = None
    if pts is not None and cells_obstacle_dist is not None:
        try:
            from scipy.spatial import cKDTree
            p2 = np.asarray(pts, dtype=np.float64).reshape(-1, np.shape(pts)[-1])[:, :2]
            od = np.asarray(cells_obstacle_dist, dtype=np.float64).ravel()
            if len(p2) == len(od) and len(p2):
                live_od = (cKDTree(p2), od)
        except Exception:
            live_od = None

    def _live_clearance(px, py):
        """obstacle_distance dal vivo nel punto, o None se fuori dalla scansione."""
        if live_od is None:
            return None
        d, i = live_od[0].query((px, py))
        return float(live_od[1][i]) if d <= 0.05 else None

    free, unknown, blocked = [], [], []
    for (px, py) in sampled:
        if _near_tried((px, py)) or global_map.is_occupied(px, py, safety_margin=safety_margin):
            blocked.append((px, py))
            continue
        c_live = _live_clearance(px, py)
        c_mem = global_map.clearance(px, py) if hasattr(global_map, 'clearance') else None
        if (c_live is not None and c_live < min_clear) or (c_mem is not None and c_mem < min_clear):
            blocked.append((px, py))
        elif c_live is not None or global_map.is_known(px, py):
            free.append((px, py))
        else:
            unknown.append((px, py))

    # Prima i punti VISTI liberi con spazio sufficiente: il centro e' gia' risultato
    # occupato, quindi la zona e' nota e tornare a "guardare" un ignoto accanto all'ostacolo
    # sarebbe un giro a vuoto. Solo se non ce n'e' nessuno, l'ignoto.
    pool = free if free else unknown
    reason = "gia' scartato dal vivo" if center_tried else "occupato nella mappa"
    if not pool:
        print(f"[TARGET] Cella ({cell_row},{cell_col}): centro {reason} e nessun punto della "
              f"cella ad almeno {min_clear:.2f} m dagli ostacoli. Nessun obiettivo valido.")
        return None, None, [], blocked

    def _d2_from_center(p):
        return (p[0] - cell_center_x) ** 2 + (p[1] - cell_center_y) ** 2

    # A parita' di distanza dal centro (a meno di un passo di reticolo) vince il punto piu'
    # vicino al ROBOT: e' quello dalla sua parte dell'ostacolo, non quello oltre.
    step = 0.25
    best_x, best_y = min(pool, key=lambda p: (round(np.sqrt(_d2_from_center(p)) / step),
                                              np.hypot(p[0] - robot_x, p[1] - robot_y)))
    dist_from_center = float(np.sqrt(_d2_from_center((best_x, best_y))))
    print(f"[TARGET] Cella ({cell_row},{cell_col}): centro {reason}, uso il punto "
          f"{'libero' if free else 'ignoto'} piu' vicino al centro ad almeno {min_clear:.2f} m "
          f"dagli ostacoli ({best_x:.2f}, {best_y:.2f}), a {dist_from_center:.2f} m dal centro  "
          f"[liberi={len(free)} ignoti={len(unknown)} scartati={len(blocked)}]")

    return best_x, best_y, free + unknown, blocked


def goal_still_reachable(snap, goal_x, goal_y):
    """
    Il punto obiettivo e' ancora raggiungibile sui dati FRESCHI?

    Il fronte sicuro pretende che la linea centrale del percorso stia ad almeno
    FRONTIER_CLEARANCE_M da un ostacolo. Un punto obiettivo con obstacle_distance sotto
    quella soglia non potra' MAI essere raggiunto, per quante scansioni si facciano: il
    robot aspetterebbe e dichiarerebbe un blocco. Succede perche' il punto si sceglie da
    lontano, quando la zona e' ancora ignota.

    Returns:
        True se il punto e' raggiungibile o se non lo si puo' giudicare (fuori dalla
        finestra scansionata: non sappiamo, e non si tocca niente sulla base di nulla).
    """
    if snap is None:
        return True
    rows, cols, inside = arcVerification._cells(snap, np.array([float(goal_x)]), np.array([float(goal_y)]))
    if not bool(inside[0]):
        return True
    return float(snap.obstacle_dist[rows[0], cols[0]]) >= spotGrid.FRONTIER_CLEARANCE_M


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
    _save_visualization_bundle(save_path, pts, terrain_real, obstacle_mask, robot_x, robot_y,
                               candidates, chosen_point, iteration, env, prm_graph, chosen_path,
                               cells_obstacle_dist, valid_values, grad_values, rough_values)


def _save_visualization_bundle(save_path, pts, terrain_real, obstacle_mask, robot_x, robot_y,
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
        _save_npz_async(
            out,
            pts=_arr(pts), terrain=_arr(terrain_real), obstacle_mask=_arr(obstacle_mask),
            obstacle_distance=_arr(cells_obstacle_dist), terrain_valid=_arr(valid_values),
            gradient=_arr(grad_values), roughness=_arr(rough_values),
            robot_xy=np.array([robot_x, robot_y], dtype=np.float64), mission_z0=_mission_z0(),
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
    _timing_add('dati figure', time.perf_counter() - t0)


def decide_next_move(robot_xy, path_waypoints, frontier, no_progress_count):
    """
    Cosa fare adesso, dato il fronte sicuro. Funzione pura: niente robot, niente I/O, quindi
    provabile fuori dal campo.

    path_waypoints: [(node_id, x, y), ...] -- i nodi ancora da raggiungere, il primo e' il
        prossimo. La posizione del robot NON e' inclusa.
    frontier: risultato di arcVerification.compute_safe_frontier sul percorso
        [robot] + waypoint.
    no_progress_count: scansioni consecutive in cui non si e' potuto avanzare.

    Restituisce un dict con 'kind':
      'reached_node'  il prossimo nodo e' gia' sotto il robot (< 5 cm): consumarlo e basta
      'move'          muoversi verso 'target'; 'full' = True se il target e' il nodo stesso.
                      'nodes' = quanti nodi del percorso stanno dentro l'avanzamento
                      consentito dal fronte: il ciclo li percorre uno dopo l'altro senza
                      riscansionare (vedi AVANZAMENTO MULTINODO). Vale 1 o piu' quando
                      'full', e non c'e' quando il passo e' parziale.
      'arrived'       non si puo' avanzare, ma l'obiettivo finale e' entro tolleranza
      'wait'          non si puo' avanzare: riscansionare da fermi prima di concludere
      'blocked'       non si puo' avanzare da FRONTIER_BLOCK_CONFIRM_SCANS scansioni
                      consecutive: blocco confermato da vicino. 'segment' = indice del
                      tratto del percorso in cui si e' fermato il fronte (0 = dal robot al
                      primo nodo).
    """
    rx, ry = robot_xy
    _, nx, ny = path_waypoints[0]
    seg_len = float(np.hypot(nx - rx, ny - ry))
    if seg_len < 0.05:
        return {'kind': 'reached_node'}

    adv = arcVerification.allowed_advance(frontier)
    if adv >= seg_len - 0.02:
        # --- AVANZAMENTO MULTINODO ----------------------------------------------------
        # Il fronte sicuro percorre TUTTO il percorso pianificato, non il primo tratto:
        # restituisce fin dove e' confermato libero adesso, spesso piu' nodi avanti.
        # Si contano TUTTI i nodi che stanno dentro l'avanzamento consentito, e il
        # ciclo li percorre uno dopo l'altro SENZA scansione, refresh ne' ripianificazione
        # in mezzo, invece di fermarsi e riscansionare a ogni nodo. Si cammina sempre sulla
        # polilinea, un tratto per volta: NON si taglia la corda fra due nodi, che
        # passerebbe su terreno non verificato (e' quello che fa shortcut_index, ma solo
        # dopo avere verificato la corda col fronte).
        # Il controllo NON diventa un verdetto ricordato: durante il movimento il thread
        # ricalcola il fronte a ~5 Hz sul percorso rimanente e ferma il robot se si
        # accorcia (vedi _should_abort_move e FRONTIER_ABORT_CONFIRM). Si eliminano le
        # FERMATE, non le verifiche.
        cum, nodes = 0.0, 0
        px, py = rx, ry
        for (_, wx, wy) in path_waypoints:
            cum += float(np.hypot(wx - px, wy - py))
            if cum > adv + 0.02:
                break
            px, py, nodes = wx, wy, nodes + 1
        return {'kind': 'move', 'target': (nx, ny), 'full': True, 'dist': seg_len,
                'nodes': max(1, nodes)}
    if adv >= MIN_PARTIAL_STEP_M:
        f = adv / seg_len
        return {'kind': 'move', 'target': (rx + f * (nx - rx), ry + f * (ny - ry)),
                'full': False, 'dist': adv}

    if frontier['path_len'] <= GOAL_REACHED_TOLERANCE_M:
        return {'kind': 'arrived'}
    if no_progress_count + 1 < FRONTIER_BLOCK_CONFIRM_SCANS:
        return {'kind': 'wait'}

    polyline = [(rx, ry)] + [(x, y) for _, x, y in path_waypoints]
    _, seg_idx = arcVerification.point_along(polyline, frontier['dist'])
    return {'kind': 'blocked', 'segment': int(seg_idx)}


def rotation_plan(robot_yaw, robot_xy, target_xy, od_robot, snap=None):
    """
    Si puo' ruotare sul posto per guardare il punto da raggiungere?

    Ruotando di un certo angolo il corpo spazza un settore, non tutto il cerchio di raggio
    0.60 attorno al robot: si controllano le celle davvero spazzate. Chiedere libero tutto
    il cerchio (obstacle_distance sotto il robot >= ROTATION_CLEARANCE_M) e' il caso
    peggiore su tutte le direzioni e tutti gli angoli, e rifiuterebbe molte rotazioni sicure.

    Ordine dei controlli, dal piu' economico:
      1. rotazione sotto ROTATION_CHECK_MIN_DYAW_DEG -- trascurabile, si ruota
      2. obstacle_distance sotto il robot >= ROTATION_CLEARANCE_M -- cerchio circoscritto
         libero, si ruota senza altri calcoli (prefiltro)
      3. spotGrid.rotation_is_clear() sulle celle davvero spazzate (0.26 ms)

    Senza `snap` (nessuna fotografia della griglia) si ricade sul solo confronto scalare.

    Funzione pura, provabile fuori dal campo. Restituisce dict con:
      turn (bool), dyaw_deg, motion_dir (rad), margin (margine del settore spazzato, in
      metri, o None se non calcolato).
    """
    rx, ry = robot_xy
    tx, ty = target_xy
    motion_dir = float(np.arctan2(ty - ry, tx - rx))
    dyaw = float(np.degrees(np.arctan2(np.sin(motion_dir - robot_yaw), np.cos(motion_dir - robot_yaw))))

    if abs(dyaw) < spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'rotazione trascurabile'}
    if od_robot >= spotGrid.ROTATION_CLEARANCE_M:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'cerchio circoscritto libero'}
    if snap is not None:
        ok, margin = spotGrid.rotation_is_clear(
            snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
            rx, ry, robot_yaw, float(np.radians(dyaw)))
        return {'turn': bool(ok), 'dyaw_deg': dyaw, 'motion_dir': motion_dir,
                'margin': float(margin) if np.isfinite(margin) else None,
                'why': 'settore spazzato'}
    return {'turn': False, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
            'why': 'nessun dato di griglia'}


SHORTCUT_MAX_DIST_M = 2.5          # oltre, il tratto non sta comunque nella finestra


def shortcut_index(robot_xy, path_waypoints, snap, global_map=None, prm=None, robot_node_id=None):
    """
    Scorciatoia: l'indice del waypoint PIU' LONTANO raggiungibile in linea retta
    dal robot con il tratto interamente confermato libero dal fronte sicuro (muso compreso).
    0 = nessuna scorciatoia. Toglie le svolte inutili fra nodi del PRM.
    Il controllo e' il fronte stesso sui dati dal vivo: ostacoli, rugosita', pendenza.

    E anche la MAPPA GLOBALE, con lo stesso margine del PRM: alcune cose stanno solo li' --
    il punto di una caduta, per esempio, che ai sensori puo' sembrare terreno normale (e' per
    questo che il robot ci e' caduto). Gli archi del PRM le evitano gia'; una scorciatoia
    controllata solo dal vivo potrebbe riportarci sopra il robot.

    E GLI ARCHI GIA' SCARTATI. Senza questo controllo la scorciatoia riaggancerebbe un
    waypoint il cui collegamento diretto e' appena stato invalidato, e il ciclo non
    uscirebbe piu': il tratto viene scartato, Dijkstra trova una strada che passa comunque
    per quel nodo attraverso i nodi intermedi, e la scorciatoia ci ripunta dritto. Servono
    `prm` e `robot_node_id`; senza, il controllo si salta.
    """
    rx, ry = robot_xy
    for k in range(len(path_waypoints) - 1, 0, -1):
        wid, wx, wy = path_waypoints[k]
        if prm is not None and robot_node_id is not None:
            key = (min(robot_node_id, wid), max(robot_node_id, wid))
            if key in getattr(prm, 'tracker_blocked_edges', ()):
                continue
            # Anche gli scarti soft (mark_edge_blocked_soft: [SUBITO], rotazione rifiutata),
            # altrimenti la scorciatoia ripunta il nodo appena scartato per rotazione.
            if getattr(prm, 'edge_validity', {}).get(key) is False:
                continue
        d = float(np.hypot(wx - rx, wy - ry))
        if d > SHORTCUT_MAX_DIST_M:
            continue
        if global_map is not None:
            n = max(prm_graph.OCCUPANCY_SAMPLES_MIN, int(np.ceil(d * prm_graph.OCCUPANCY_SAMPLES_PER_M)) + 1)
            if any(global_map.is_occupied(rx + t * (wx - rx), ry + t * (wy - ry),
                                          safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
                   for t in np.linspace(0.0, 1.0, n)[1:]):
                continue
        f = arcVerification.compute_safe_frontier([(rx, ry), (wx, wy)], snap)
        if f['reason'] == 'fine_percorso':
            return k
    return 0


ROTATION_ROOM_SEARCH_M = (0.15, 0.30, 0.45, 0.60, 0.80, 1.00, 1.20)
ROTATION_ROOM_DIRECTIONS_DEG = (180, 135, -135, 90, -90, 45, -45, 0)   # rispetto al muso: indietro prima


def plan_straight_retreat(robot_xy, robot_yaw, snap, max_dist=ROTATION_RETREAT_M):
    """
    Quanto puo' arretrare il robot IN LINEA RETTA, all'indietro, senza girarsi.

    Non e' marcia obliqua: muovendosi lungo il proprio asse l'ingombro resta 0.50 m di
    larghezza e il fronte applica la soglia normale di 0.30 m, non i 0.59 della marcia di
    traverso. E' la via d'uscita quando la rotazione verso il prossimo punto non e'
    possibile: ci si stacca da cio' che impedisce di girarsi e al giro dopo si riprova da
    una posizione diversa.

    Funzione pura. Restituisce (x, y, distanza) del punto raggiungibile, oppure None se non
    si puo' arretrare di almeno ROTATION_RETREAT_MIN_M.
    """
    rx, ry = robot_xy
    back_x = rx - max_dist * np.cos(robot_yaw)
    back_y = ry - max_dist * np.sin(robot_yaw)
    body = spotGrid.body_extents(robot_yaw, robot_yaw + np.pi)   # all'indietro: (0.55, 0.25)
    f = arcVerification.compute_safe_frontier([(rx, ry), (back_x, back_y)], snap, body=body)
    adv = arcVerification.allowed_advance(f)
    if adv < ROTATION_RETREAT_MIN_M:
        return None
    adv = min(adv, max_dist)
    return (float(rx - adv * np.cos(robot_yaw)), float(ry - adv * np.sin(robot_yaw)), float(adv))


def slope_usable_mask(is_valid_flat, cells_obstacle_dist, shape):
    """
    Maschera delle celle le cui QUOTE possono
    entrare nel calcolo della pendenza di un arco: attendibili (is_valid di correct_terrain)
    E non-ostacolo.

    La faccia verticale di un mobile non e' terreno in pendenza, e' un ostacolo -- ed e' gia'
    gestita come tale due volte: dal veto di occupazione del PRM e dal fronte sicuro, che si
    ferma a FRONTIER_CLEARANCE_M da qualunque cella con obstacle_distance bassa. Contarla
    ANCHE come pendenza la conta la terza volta, e con il peggiore degli effetti: non ferma
    il robot davanti al mobile (ci pensa il fronte), scollega il grafo PRM in tutta la stanza.

    Gli archi che perdono il dato NON diventano "liberi per decreto": diventano "senza dati",
    e un arco senza dati non viene vetato (entra nel grafo con un costo di ripiego, vedi
    PRM._evaluate_and_add_edge); chi lo percorre lo verifica metro per metro col fronte
    sicuro. E' una scelta esplicita: su quegli archi la sicurezza la da' il fronte, non una
    pendenza misurata su un armadio.

    La maschera si combina QUI e non dentro prm_graph perche' e' qui che si hanno in mano
    entrambi gli array, e perche' in prm_graph `current_valid_2d` serve a una cosa sola -- il
    profilo di pendenza per-arco (compute_arc_slope_profile) -- quindi restringerla non ha
    altri effetti. Chi legge prm_graph.update_local_grid_data trova `valid_2d` documentato
    come "celle attendibili": qui significa "attendibili come TERRENO".
    """
    valid = np.asarray(is_valid_flat, dtype=bool).reshape(shape)
    if cells_obstacle_dist is None:
        return valid
    od = np.asarray(cells_obstacle_dist, dtype=np.float64).reshape(shape)
    return valid & ~(od < spotGrid.OBSTACLE_THRESHOLD)


def graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client, prm_node_id,
                             cell_row, cell_col, force=False):
    """
    Lascia una briciola: un waypoint GraphNav nel punto in cui si trova il robot,
    legato al nodo PRM da cui sta passando.

    Serve per i ritorni. Camminando ALL'INDIETRO sui tratti registrati
    (retreat_along_traveled_path) il robot non vede dove mette le zampe, non si
    rilocalizza, e un errore si accumula. Con una traccia di waypoint il ritorno lo fa
    GraphNav, che cammina in avanti, con la localizzazione e l'anti-ostacolo di Boston
    Dynamics (decisione dell'utente).

    Densita': un waypoint solo se il robot e' almeno GRAPHNAV_WAYPOINT_MIN_SPACING_M
    dall'ultimo lasciato, piu' sempre uno all'ingresso di una cella (force=True). Un
    waypoint per OGNI posizione di sosta darebbe una mappa troppo densa: i passi parziali
    lasciano soste a 20-60 cm una dall'altra.

    La traccia vive su env (_graphnav_trail) perche' deve sopravvivere ai tentativi di cella.
    Non solleva mai: una briciola mancata e' una briciola, non un motivo per fermare il robot.
    """
    if recordingInterface is None:
        return None
    trail = getattr(env, '_graphnav_trail', None)
    if trail is None:
        trail = env._graphnav_trail = []
    try:
        x, y, _, _ = spotUtils.getPosition(robot_state_client)
    except Exception as e:
        print(f"[BRICIOLA] Posizione non leggibile: {e}")
        return None
    if not force and trail:
        last = trail[-1]
        if float(np.hypot(x - last['x'], y - last['y'])) < GRAPHNAV_WAYPOINT_MIN_SPACING_M:
            return None
    try:
        recordingInterface.create_default_waypoint(cell_row=cell_row, cell_col=cell_col)
        # Nome e id dal waypoint appena creato (li porta la risposta del robot), non da
        # get_localization_state(), che riscarica tutto il grafo solo per leggerli. Ripiego
        # su get_localization_state() se l'interfaccia non li espone.
        last = getattr(recordingInterface, 'last_created_waypoint', None)
        if isinstance(last, dict) and last.get('id'):
            wp_id, wp_name = last['id'], last.get('name')
        else:
            loc = recordingInterface.get_localization_state()
            wp_id = loc.get('waypoint_id') if isinstance(loc, dict) else None
            wp_name = loc.get('waypoint_name') if isinstance(loc, dict) else None
    except Exception as e:
        print(f"[BRICIOLA] Waypoint non creato: {e}")
        return None
    if not wp_id:
        print("[BRICIOLA] Waypoint creato ma non localizzato: non lo metto nella traccia "
              "(un waypoint senza id non si puo' raggiungere).")
        return None
    crumb = {'wp_id': wp_id, 'wp_name': wp_name, 'x': float(x), 'y': float(y),
             'prm_node': prm_node_id, 'cell': (cell_row, cell_col)}
    trail.append(crumb)
    print(f"[BRICIOLA] Waypoint {wp_name} in ({x:.2f}, {y:.2f}), nodo PRM {prm_node_id} "
          f"(traccia: {len(trail)} waypoint).")
    # Stato del localizzatore di GraphNav, stampato solo quando cambia.
    graphnav_log_localization(env, recordingInterface, robot_state_client, f"briciola {wp_name}")
    return crumb


def graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                       min_dist_m=0.40):
    """
    Torna al waypoint piu' recente che sia almeno min_dist_m DIETRO, con GraphNav.

    Si cerca indietro nella traccia perche' l'ultima briciola puo' essere stata lasciata qui,
    dove il robot e' adesso: tornarci non sposterebbe niente. Se nessuna briciola e' abbastanza
    lontana si usa comunque la piu' vecchia disponibile.

    Restituisce (ok, crumb). ok=True solo se GraphNav dichiara di essere ARRIVATO: in
    _graphnav_navigate_to lo STATUS_LOST e' un fallimento, non si forza la localizzazione
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
    recording_stopped = False
    graphnav_add_shortcuts(recordingInterface, "prima del ritorno", map_edges=True, sdk=False,
                           target_wp_id=crumb['wp_id'], robot_xy=(rx, ry))
    try:
        recordingInterface.stop_recording()
        recording_stopped = True
    except Exception as e:
        print(f"[RITORNO] Non ho potuto fermare la registrazione ({e}): provo comunque.")
    ok = _graphnav_navigate_to(
        recordingInterface, crumb['wp_id'], robot_state_client, command_client,
        f"ritorno al waypoint {crumb['wp_name']} in ({crumb['x']:.2f}, {crumb['y']:.2f})")
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


def backtracking_cannot_help(env, prm_graph, goal_id, robot_state_client, max_hops,
                             min_dist_m=0.40):
    """
    Tornare indietro con GraphNav puo' servire a qualcosa?

    escape_by_backtracking torna a una briciola e SOLO DOPO chiede al grafo un percorso da
    li'. Ma ogni briciola ha gia' il suo nodo PRM: la stessa domanda si puo' fare prima di
    muoversi. Se l'obiettivo sta in un'altra componente del grafo, nessun waypoint dietro
    puo' cambiare la risposta, e i ritorni sarebbero solo strada sprecata.

    Risponde True solo se OGNI briciola che verrebbe visitata ha un nodo PRM collegato nel
    grafo e da nessuno di essi c'e' un percorso. Se anche una sola non si puo' giudicare
    (nodo mancante o isolato, es. scartato dentro un ostacolo) risponde False e si torna
    indietro: nel dubbio non si toglie la via d'uscita.
    """
    trail = getattr(env, '_graphnav_trail', None) or []
    if not trail or goal_id not in prm_graph.nodes:
        return False
    try:
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
    except Exception:
        return False
    candidates = [c for c in reversed(trail)
                  if float(np.hypot(c['x'] - rx, c['y'] - ry)) >= min_dist_m][:int(max_hops)]
    if not candidates:
        return False
    for crumb in candidates:
        nid = crumb.get('prm_node')
        if nid is None or nid not in prm_graph.nodes or not prm_graph.edges.get(nid):
            return False
        if prm_graph.find_path_dijkstra(nid, goal_id) is not None:
            return False
    return True


def _trap_backtrack(trap_state, block_arc, env, prm_graph, goal_id, recordingInterface,
                    robot_state_client, command_client, global_map, mobility_kwargs, pts,
                    obstacle_mask, label="", **kwargs):
    """
    Ritorno indietro (escape_by_backtracking) con MEMORIA per il tentativo.

    trap_state: dict del tentativo ({'n': ritorni gia' fatti}), che non si azzera mai dentro
    il tentativo. block_arc: (a, b), l'arco su cui il robot si e' fermato.

    Prima di ripianificare, l'arco diventa impercorribile per il resto del tentativo
    (penalita' del tentativo: sopravvive a refresh e ricostruzioni e si toglie solo
    all'inizio del tentativo successivo, vedi prm_graph.clear_penalties_for_attempt). Da li'
    il pianificatore deve proporre un'altra strada, o ammettere che non c'e'. Al
    MAX_BACKTRACKS_PER_ATTEMPT-esimo ritorno il robot torna comunque indietro -- la' dov'e'
    non puo' restare -- ma il tentativo finisce: la cella si rimanda e si prova da un altro
    lato. Senza questa memoria il robot rifarebbe lo stesso giro all'infinito.
    """
    trap_state['n'] = trap_state.get('n', 0) + 1
    if block_arc is not None and block_arc[0] is not None and block_arc[1] is not None:
        a, b = int(block_arc[0]), int(block_arc[1])
        if a != b:
            key = (min(a, b), max(a, b))
            prm_graph.edge_penalty_persistent[key] = prm_graph._ATTEMPT_MARK
            print(f"[RITORNO] Arco {a}->{b} fuori dal piano per il resto del tentativo "
                  f"(ritorno {trap_state['n']}/{MAX_BACKTRACKS_PER_ATTEMPT}).")
    esc = escape_by_backtracking(
        env, prm_graph, goal_id, recordingInterface, robot_state_client, command_client,
        global_map, mobility_kwargs, pts, obstacle_mask, label=label, **kwargs)
    if trap_state['n'] >= MAX_BACKTRACKS_PER_ATTEMPT and esc.get('ok'):
        MISSION_STATS['backtrack_giveups'] = MISSION_STATS.get('backtrack_giveups', 0) + 1
        print(f"[RITORNO] {trap_state['n']} ritorni in questo tentativo: sono tornato "
              f"indietro, ma non riprovo. Rimando la cella e la provo da un altro lato.")
        esc = dict(esc, ok=False, how='rinuncia dopo ritorni ripetuti')
    return esc


def escape_by_backtracking(env, prm_graph, goal_id, recordingInterface, robot_state_client,
                           command_client, global_map, mobility_kwargs, pts, obstacle_mask,
                           max_hops=GRAPHNAV_MAX_BACKTRACK_HOPS, label="",
                           skip_if_graph_cannot_help=False):
    """
    La via d'uscita quando da qui non si va da nessuna parte.

    Si torna indietro di un waypoint con GraphNav e si prova a ripianificare verso
    l'obiettivo da li'; se non si puo', si torna indietro di un altro, e cosi' via fino a
    max_hops. Dopo max_hops la cella si rimanda e si passa alla priorita' successiva.

    Se GraphNav non ce la fa (traccia vuota, non localizzato, STUCK, tempo scaduto) si ricade
    sulla ritirata a zampe: e' peggiore, ma e' l'ultima rete e non la togliamo. In una
    trappola dove ruotare e' impossibile GraphNav, che per percorrere un arco vuole girare il
    muso, puo' tornare STUCK proprio dove serve.

    Restituisce dict: ok (si puo' continuare la missione), ids (nuovo percorso o None),
    robot_node (nuovo nodo del robot o None), how (cosa e' successo, per il log).
    """
    # Solo quando il robot NON e' intrappolato (chiamanti "nessuna alternativa nel grafo"):
    # in una trappola o dopo un fallimento fisico tornare indietro serve anche a liberarlo.
    if skip_if_graph_cannot_help and backtracking_cannot_help(
            env, prm_graph, goal_id, robot_state_client, max_hops):
        print(f"[RITORNO] {label}: dai nodi delle prossime {max_hops} briciole il grafo non "
              f"arriva comunque all'obiettivo (non dipende da dove sono io). Non torno "
              f"indietro: rimando subito la cella.")
        MISSION_STATS['backtracks_skipped'] = MISSION_STATS.get('backtracks_skipped', 0) + 1
        return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}

    for hop in range(1, int(max_hops) + 1):
        ok, crumb = graphnav_step_back(env, recordingInterface, robot_state_client, command_client)
        if not ok:
            if crumb is None:
                print(f"[RITORNO] {label}: nessuna briciola GraphNav disponibile.")
            else:
                print(f"[RITORNO] {label}: GraphNav non e' riuscito a tornare a "
                      f"{crumb['wp_name']}.")
            MISSION_STATS['graphnav_backtrack_fail'] += 1
            print("[RITORNO] Ripiego sulla ritirata a zampe sui tratti percorsi.")
            retreat_along_traveled_path(env, robot_state_client, command_client,
                                        mobility_kwargs=mobility_kwargs,
                                        pts=pts, obstacle_mask=obstacle_mask)
            return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'ritirata a zampe'}

        MISSION_STATS['graphnav_backtracks'] += 1
        prm_graph.needs_refresh = True       # il robot si e' spostato: vedi REFRESH A EVENTI
        # La scansione dello scarto preliminare e' stata fatta da un altro punto (e
        # l'esclusione del corpo starebbe dove il robot era): via, fino alla prossima scansione.
        prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
        try:
            ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
        except Exception:
            ax, ay = crumb['x'], crumb['y']
        new_node = prm_graph.add_stop_node(
            ax, ay, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"sosta (ritorno GraphNav a {crumb['wp_name']})")
        # Niente deroga alla mappa globale attorno a questo nodo: vedi il commento al
        # chiamante di set_robot_node nel ciclo principale.
        prm_graph.set_robot_node(new_node, 0.0)

        ids = prm_graph.find_path_dijkstra(new_node, goal_id)
        if ids is not None and len(ids) > 1:
            print(f"[RITORNO] {label}: tornato a {crumb['wp_name']} ({hop}/{max_hops}) e da qui "
                  f"c'e' un percorso con {len(ids) - 1} tratti. Riprendo.")
            return {'ok': True, 'ids': ids, 'robot_node': new_node, 'how': 'ripianificato'}
        print(f"[RITORNO] {label}: tornato a {crumb['wp_name']} ({hop}/{max_hops}) ma da qui il "
              f"grafo non offre percorsi verso l'obiettivo.")

    print(f"[RITORNO] {label}: {max_hops} ritorni senza trovare un percorso. Rimando la cella.")
    return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}


def prune_prm_nodes_in_obstacles(prm, snap, min_clearance_m=PRM_NODE_MIN_CLEARANCE_M,
                                 global_map=None):
    """
    Scarta dal grafo i nodi che cadono DENTRO un ostacolo.

    Gli archi che attraversano celle occupate sono vetati (_occupancy_blocked), ma i nodi
    nascono dal reticolo prima che il robot abbia visto qualcosa: un nodo dentro un
    ostacolo resterebbe destinazione o punto di passaggio, da scoprire solo sul posto.

    Soglia min_clearance_m = FRONTIER_CLEARANCE_M (0.30): sotto quella il centro del corpo
    non ci sta e il fronte non ci arrivera' mai, quindi il nodo e' inutile sia come
    destinazione sia come punto di passaggio.

    Si scarta SOLO dove la cella e' stata davvero osservata: ignoto non e' occupato (e' la
    stessa regola per cui il fronte distingue 'fine_dati' da 'ostacolo'). Un nodo fuori
    dalla finestra o mai visto resta dov'e' e verra' giudicato quando lo si vedra'.

    Si rivaluta a ogni ciclo, quindi un nodo che si scopre dentro un muro cade subito; e
    siccome gli archi vengono tolti in modo NON permanente, un nodo giudicato male da una
    lettura rumorosa torna in gioco quando i dati freschi lo smentiscono.

    La finestra della scansione contiene solo una piccola parte del grafo (~7%), e lo
    scarto e' soft: al refresh successivo gli archi tornano. Con `global_map` (e
    spotGrid.CLEARANCE_MAP_DECIDES) si guarda ANCHE la mappa dello spazio libero, che copre
    tutto il grafo e ricorda, cosi' il giudizio su un nodo lontano non va perso a ogni giro.
    """
    ids = [nid for nid in prm.nodes if prm.edges.get(nid)]   # gia' isolati: niente da togliere
    if not ids:
        return []
    xy = np.array([prm.nodes[nid] for nid in ids], dtype=np.float64)
    rows, cols, inside = arcVerification._cells(snap, xy[:, 0], xy[:, 1])
    bad = np.zeros(len(ids), dtype=bool)
    if np.any(inside):
        r, c = rows[inside], cols[inside]
        bad[inside] = snap.seen[r, c] & (snap.obstacle_dist[r, c] < min_clearance_m)
    if (spotGrid.CLEARANCE_MAP_DECIDES and global_map is not None
            and hasattr(global_map, 'clearance_many')):
        cl = global_map.clearance_many(xy[:, 0], xy[:, 1])
        bad |= np.isfinite(cl) & (cl < min_clearance_m)
    pruned = []
    for k in np.nonzero(bad)[0]:
        nid = ids[int(k)]
        for nb in [n for n, _ in prm.edges.get(nid, [])]:
            mark_edge_blocked_soft(prm, nid, nb)
        pruned.append(nid)
    return pruned


def scan_escape_directions(robot_xy, robot_yaw, snap, step_deg=ESCAPE_FAN_STEP_DEG,
                           min_advance_m=ESCAPE_FAN_MIN_ADVANCE_M, probe_m=3.0):
    """
    Il giro completo: in quali direzioni il robot puo' DAVVERO andare da qui?

    Per ciascuna direzione del giro (passo step_deg) misura due cose indipendenti:
      - quanto spazio concede il fronte sicuro andando in linea retta in quella direzione;
      - se il robot puo' girarsi tanto da puntarci il muso (spotGrid.rotation_is_clear).
    Una direzione e' UTILIZZABILE solo se entrambe rispondono si'. Il robot non cammina di
    traverso (vedi ROTAZIONE E ARRETRAMENTO), quindi una corsia larga che richiede una
    rotazione impossibile NON e' una via d'uscita, per quanto larga sia.

    Serve a riconoscere una trappola (zero direzioni utilizzabili: robot in un angolo, con
    le corsie aperte dietro rotazioni che li' non si fanno) con 24 valutazioni del fronte,
    invece di scoprirlo scartando gli archi uno per uno, una scansione dopo l'altra.

    Funzione pura. Restituisce la lista delle direzioni esaminate, ciascuna con:
    dir_deg (assoluta), rel_deg (rotazione necessaria), advance, reason, rotatable, usable.
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    out = []
    for deg in np.arange(-180.0, 180.0, abs(step_deg)):
        a = float(np.radians(deg))
        f = arcVerification.compute_safe_frontier(
            [(rx, ry), (rx + probe_m * np.cos(a), ry + probe_m * np.sin(a))], snap)
        adv = arcVerification.allowed_advance(f)
        rel = float(np.degrees(np.arctan2(np.sin(a - robot_yaw), np.cos(a - robot_yaw))))
        if abs(rel) < spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
            rotatable = True
        else:
            rotatable = bool(spotGrid.rotation_is_clear(
                snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
                rx, ry, float(robot_yaw), float(np.radians(rel)))[0])
        out.append({'dir_deg': float(deg), 'rel_deg': rel, 'advance': float(adv),
                    'reason': f['reason'], 'rotatable': rotatable,
                    'usable': bool(rotatable and adv >= min_advance_m)})
    return out


def describe_escape_fan(dirs, max_rows=4):
    """Riga di log compatta: quante direzioni utilizzabili e le migliori, per i log."""
    usable = [d for d in dirs if d['usable']]
    best = sorted(dirs, key=lambda d: -d['advance'])[:max_rows]
    txt = "; ".join(f"{d['dir_deg']:+.0f} gradi: {d['advance']:.2f} m "
                    f"({'ruotabile' if d['rotatable'] else 'NON ruotabile'})" for d in best)
    return len(usable), txt


def is_trapped(robot_xy, robot_yaw, snap):
    """
    True se NESSUNA direzione del giro e' insieme percorribile e raggiungibile
    girandosi: la condizione che l'utente ha posto per tornare indietro ("solo se non puo'
    piu' andare avanti ne' ruotare"), verificata in una scansione invece che in venti.
    Restituisce (trapped, dirs) -- dirs serve per il log, cosi' nei dati si vede perche'.
    """
    dirs = scan_escape_directions(robot_xy, robot_yaw, snap)
    return (not any(d['usable'] for d in dirs)), dirs


def arrival_has_exit(snap, arrival_xy, heading, min_advance_m=ESCAPE_FAN_MIN_ADVANCE_M):
    """
    Dal punto in cui sto per fermarmi, ci sara' un'uscita?

    Il fronte sicuro guarda la strada DAVANTI, non se dalla posa d'arrivo si potra'
    ripartire: con un passo corto il robot puo' entrare in un angolo senza uscite.

    Si valuta il ventaglio nel punto d'arrivo usando la fotografia CORRENTE: e' una stima
    (da li' il robot vedra' di piu'), ma e' la stessa informazione su cui si sta decidendo
    di andarci, quindi la decisione e' almeno coerente con se stessa. Per non pagarla su
    ogni movimento la chiama solo il ciclo principale, e solo per i passi corti o verso
    spazi stretti -- vedi ARRIVAL_EXIT_CHECK_MAX_STEP_M / _OD_M.

    Restituisce (ha_uscita, dirs).
    """
    ax, ay = float(arrival_xy[0]), float(arrival_xy[1])
    # Stessa griglia, ma con il robot dichiarato nel punto d'arrivo: e' cio' che cambia il
    # riferimento della regola di uscita del fronte e il centro delle rotazioni provate.
    probe = arcVerification.GridSnapshot(
        snap.obstacle_dist, snap.rough_veto, snap.seen, snap.terrain,
        snap.origin_x, snap.origin_y, snap.cell_size, ax, ay,
        timestamp=snap.time, valid=snap.valid, blocked_latch=snap.blocked_latch)
    dirs = scan_escape_directions((ax, ay), float(heading), probe, min_advance_m=min_advance_m)
    return any(d['usable'] for d in dirs), dirs


def mark_edge_blocked_soft(prm, idx1, idx2):
    """
    Toglie un arco dal grafo ADESSO, senza condannarlo per sempre.

    prm_graph.mark_edge_invalid() lo mette anche in `tracker_blocked_edges`, che non viene
    MAI ricreato -- ne' dal refresh ne' dalla ricostruzione degli archi. Giusto per un
    blocco confermato da tre scansioni ferme; sbagliato per [SUBITO], che scarta dopo UNA
    scansione: un ostacolo marginale visto una volta isolerebbe un nodo per tutta la missione.

    Qui si tolgono solo l'arco e la sua validita': alla prossima rivalutazione, se i dati
    freschi dicono che e' libero, l'arco torna -- che e' esattamente il comportamento di
    rientro che refresh_local_edge_weights e' stato scritto per avere.
    """
    key = (min(idx1, idx2), max(idx1, idx2))
    # Scartare un arco cambia il piano, quindi al prossimo giro il refresh degli archi va
    # fatto (vedi REFRESH A EVENTI nel ciclo di navigazione).
    prm.needs_refresh = True
    prm.edge_validity[key] = False
    prm.traversed_edges.discard(key)
    prm.edges[idx1] = [(n, w) for n, w in prm.edges.get(idx1, []) if n != idx2]
    prm.edges[idx2] = [(n, w) for n, w in prm.edges.get(idx2, []) if n != idx1]


def plan_cone_move(robot_xy, robot_yaw, snap, goal_xy, max_dyaw_deg=CONE_SWEEP_MAX_DEG,
                   step_deg=CONE_SWEEP_STEP_DEG, min_gain_m=CONE_MIN_GAIN_M):
    """
    "Ruota quanto PUOI, non quanto vorresti, e cammina."

    Quando la rotazione verso il prossimo nodo e' rifiutata, la domanda giusta non e' piu'
    "posso girarmi verso il nodo?" ma "qual e' la rotazione piu' grande che posso fare qui,
    e in quale direzione dentro quel cono conviene andare?". In un corridoio stretto girarsi
    richiede 1.21 m di diametro e non si puo', ma camminare si': spesso andare dritti, o
    quasi, avvicina all'obiettivo piu' di qualunque arretramento.

    Criterio di scelta: fra le direzioni che AVVICINANO all'obiettivo
    di almeno min_gain_m, si prende quella con piu' spazio confermato libero. Non la piu'
    libera in assoluto: quella porterebbe il robot via dalla cella.

    Funzione pura. Restituisce None se nessuna direzione del cono avvicina, altrimenti
    dict con: dyaw_deg (rotazione da fare, gradi, puo' essere 0), target (x, y), dist
    (avanzamento), gain (metri guadagnati verso l'obiettivo), reason (motivo del fronte).
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    d0 = float(np.hypot(gx - rx, gy - ry))

    best = None
    rels = np.arange(-abs(max_dyaw_deg), abs(max_dyaw_deg) + 1e-9, abs(step_deg))
    for rel in rels:
        # La rotazione deve essere possibile: sotto la soglia di trascurabilita' non si
        # controlla nulla (e' quello che fa anche rotation_plan), sopra si misura il settore.
        if abs(rel) >= spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
            ok, _ = spotGrid.rotation_is_clear(
                snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
                rx, ry, robot_yaw, float(np.radians(rel)))
            if not ok:
                continue

        heading = robot_yaw + float(np.radians(rel))
        far_x, far_y = rx + 3.0 * np.cos(heading), ry + 3.0 * np.sin(heading)
        f = arcVerification.compute_safe_frontier([(rx, ry), (far_x, far_y)], snap)
        adv = arcVerification.allowed_advance(f)
        if adv < MIN_PARTIAL_STEP_M:
            continue

        nx, ny = rx + adv * np.cos(heading), ry + adv * np.sin(heading)
        gain = d0 - float(np.hypot(gx - nx, gy - ny))
        if gain < min_gain_m:
            continue
        cand = {'dyaw_deg': float(rel), 'target': (float(nx), float(ny)), 'dist': float(adv),
                'gain': float(gain), 'reason': f['reason']}
        # Piu' spazio libero vince; a pari spazio, la rotazione piu' piccola.
        if best is None or (cand['dist'], -abs(cand['dyaw_deg'])) > (best['dist'], -abs(best['dyaw_deg'])):
            best = cand
    return best


def retreat_would_help(snap, back_xy, robot_yaw, dyaw_deg):
    """
    L'arretramento di 0.60 m renderebbe possibile la rotazione che serve?

    In un corridoio la geometria 0.60 m piu' indietro e' la stessa: arretrare non apre
    spazio, lo sposta. Si arretra solo se da li' la rotazione diventa possibile.

    Si valuta la rotazione nel punto di ARRIVO usando la fotografia corrente -- e' una stima,
    perche' da li' il robot vedra' qualcosa di piu', ma e' la stessa informazione su cui si
    e' deciso di arretrare, quindi almeno la decisione e' coerente con se stessa.
    """
    if abs(dyaw_deg) < spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
        return True, None
    ok, margin = spotGrid.rotation_is_clear(
        snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
        float(back_xy[0]), float(back_xy[1]), float(robot_yaw), float(np.radians(dyaw_deg)))
    return bool(ok), margin


def attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client,
                                     env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                                     iteration=0,
                                     recordingInterface=None, verification_tracker=None, mobility_kwargs=None):

    """Prova a entrare nella cella (target_row, target_col) dalla posizione attuale del robot.

    Scansione, aggiornamento delle mappe e del grafo, scelta del punto obiettivo, percorso,
    poi il ciclo di navigazione con il fronte sicuro. True se il robot finisce dentro la cella.

    mobility_kwargs: dict facoltativo che impone l'andatura (locomotion_hint, ground_mu_hint,
        swing_height) per tutti i tratti, passato fino a movements.relative_move attraverso
        navigate_to. Vuoto = andatura scelta tratto per tratto (select_mobility_params).
    """
    mobility_kwargs = mobility_kwargs or {}

    print(f"\n[ATTEMPT] Provo a entrare nella cella ({target_row},{target_col}) dalla posizione attuale...")

    # Il robot arriva qui da un altro movimento (navigazione fra celle, ritirata): se e'
    # caduto nel frattempo si rialza prima di tutto. Solleva RobotFallenError se non puo'.
    # Poi esce dal disco pericoloso ripercorrendo i propri passi: rialzato, e' ancora DENTRO
    # il punto della caduta, e da li' ogni arco del PRM verrebbe scartato.
    if _check_and_recover_fall(robot_state_client, command_client,
                               global_grid.global_occupancy_map, "all'inizio del tentativo") is not None:
        retreat_along_traveled_path(env, robot_state_client, command_client,
                                    mobility_kwargs=mobility_kwargs)

    local_grid_client = local_grid.local_grid_client

    # 1. Tutti i layer necessari, in una richiesta
    grids_data, main_proto, all_grids_proto = local_grid.return_local_grid(
        ['obstacle_distance', 'terrain', 'terrain_valid'],
        robot_state_client
    )

    if grids_data is None or 'obstacle_distance' not in grids_data:
        print("[ERROR] Rilevamento griglie locali fallito.")
        return False

    # 2. Valori dei layer
    pts = grids_data['obstacle_distance']['pts']
    cells_obstacle_dist = grids_data['obstacle_distance']['values']
    terrain_values = grids_data['terrain']['values']
    valid_values = grids_data['terrain_valid']['values']

    # Geometria della griglia
    num_x = main_proto.local_grid.extent.num_cells_x
    num_y = main_proto.local_grid.extent.num_cells_y
    cell_size = main_proto.local_grid.extent.cell_size

    # =========================================================================
    # MAPPE GLOBALI: create alla prima scansione, poi accumulate per tutta la missione
    # =========================================================================
    if global_grid.global_occupancy_map is None:
        global_grid.global_occupancy_map = spotGrid.GlobalGrid(resolution=cell_size)
        print("[MAP] Inizializzata nuova mappa di occupazione globale persistente.")

    # Mappa globale delle altezze: stessa risoluzione nativa della griglia locale (nessuna
    # riduzione), creata come quella di occupazione.
    if global_grid.global_terrain_map is None:
        global_grid.global_terrain_map = spotGrid.GlobalTerrainGrid(resolution=cell_size)
        print("[MAP] Inizializzata nuova mappa globale persistente del terreno (altezza).")

    global_map = global_grid.global_occupancy_map
    global_terrain_map = global_grid.global_terrain_map

    transforms_snapshot = main_proto.local_grid.transforms_snapshot
    vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
    robot_x, robot_y, robot_z = vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

    quat = vision_tform_body.rotation
    robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))

    footprint_mask_2d = local_grid.compute_robot_footprint_mask(
        pts, robot_x, robot_y, robot_yaw, num_x, num_y
    )

    # Correzione del terreno (celle non valide e impronta del robot): da qui in poi si usa
    # solo terrain_real
    terrain_real, is_valid = local_grid.correct_terrain(
        terrain_values, valid_values, num_x, num_y,
        robot_footprint_mask=footprint_mask_2d, cell_size=cell_size,
        unwritten_value=_raw_zero(grids_data), unwritten_scale=_raw_scale(grids_data)
    )
    _log_ground_filter(local_grid, "scan-iniziale")

    # Pendenza e rugosita' sul terreno GIA' corretto
    grad_values, rough_values = local_grid.compute_gradient_and_roughness(
        terrain_real, valid_values, num_x, num_y, cell_size, is_valid=is_valid
    )

    grad_for_prm = grad_values.copy()
    rough_for_prm = rough_values.copy()

    # Fusione (VETO): pura logica di soglia, il terreno e' gia' stato corretto sopra.
    # Il gradiente NON entra in questo veto per cella -- vedi fuse_obstacle_mask().
    obstacle_mask = local_grid.fuse_obstacle_mask(
        cells_obstacle_dist=cells_obstacle_dist,
        rough_values=rough_values,
        is_valid=is_valid,
        obstacle_threshold=spotGrid.OBSTACLE_THRESHOLD,
        rough_threshold=spotGrid.ROUGH_THRESHOLD
    )

    _log_footprint_diagnostic(local_grid, pts, robot_x, robot_y, robot_yaw,
                              num_x, num_y, obstacle_mask, "scan-cella",
                              cells_obstacle_dist=cells_obstacle_dist,
                              rough_values=rough_values, is_valid=is_valid)

    # Aggiorna le mappe globali (occupazione e terreno) con i dati appena processati
    fold_scan_into_global_maps(global_map, global_terrain_map, pts, terrain_real, obstacle_mask,
                               is_valid, cells_obstacle_dist, (num_y, num_x),
                               (robot_x, robot_y, robot_yaw))

    # La mappa del terreno serve al PRM come ripiego per la pendenza
    prm_graph.set_global_terrain_map(global_terrain_map)

    # Mappe globali su disco (.pkl), a ogni tentativo
    if mission_folder is not None:
        pkl_map_path = os.path.join(mission_folder, "global_occupancy_map.pkl")
        global_map.save_map(pkl_map_path)
        pkl_terrain_path = os.path.join(mission_folder, "global_terrain_map.pkl")
        global_terrain_map.mission_z0 = _MISSION_CTX.get('mission_z0')
        global_terrain_map.save_map(pkl_terrain_path)

        # Le mappe globali crescono per tutta la missione e non vengono mai potate: a
        # risoluzione nativa (3 cm) un'area di 25x25 m sono ~0.7 milioni di celle per
        # mappa, cioe' qualche centinaio di MB fra occupazione e terreno.
        print(f"[MAP] Dimensione mappe globali -- occupazione: {len(global_map.grid)} celle, "
              f"terreno: {len(global_terrain_map.height_count)} celle")

    save_path = os.path.join(mission_folder,
                             f"iteration_{iteration}_cell_{target_row}_{target_col}.png") if mission_folder else None

    # =========================================================================
    # SALVATAGGIO DEI VALORI NUMERICI IN FORMATO NumPy (.npy)
    # =========================================================================
    if mission_folder is not None:
        try:
            obstacle_grid_2d = obstacle_mask.reshape((num_y, num_x))
            terrain_grid_2d = terrain_real.reshape((num_y, num_x))

            npy_obstacle_path = os.path.join(mission_folder,
                                             f"iteration_{iteration}_cell_{target_row}_{target_col}_obstacles.npy")
            npy_terrain_path = os.path.join(mission_folder,
                                            f"iteration_{iteration}_cell_{target_row}_{target_col}_terrain.npy")

            np.save(npy_obstacle_path, obstacle_grid_2d)
            np.save(npy_terrain_path, terrain_grid_2d)

            print(f"[DATA SAVE] Matrici .npy salvate per cella ({target_row},{target_col})")

        except Exception as e:
            print(f"[DATA SAVE] Impossibile salvare i file .npy: {e}")

    # ================================================================== #
    # DATI LOCALI NEL PRM: terreno, pendenza e rugosita' della scansione
    # ================================================================== #
    local_x_min, local_x_max = pts[:, 0].min(), pts[:, 0].max()
    local_y_min, local_y_max = pts[:, 1].min(), pts[:, 1].max()

    grid_frame_name = main_proto.local_grid.frame_name_local_grid_data
    vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
    grid_origin_x = vision_tform_grid.position.x
    grid_origin_y = vision_tform_grid.position.y

    grad_values_2d = grad_for_prm.reshape((num_y, num_x))
    rough_values_2d = rough_for_prm.reshape((num_y, num_x))
    terrain_values_2d = terrain_real.reshape((num_y, num_x))

    prm_graph.update_local_grid_data(
        grad_values_2d=grad_values_2d,
        rough_values_2d=rough_values_2d,
        terrain_values_2d=terrain_values_2d,
        valid_2d=slope_usable_mask(is_valid, cells_obstacle_dist, (num_y, num_x)),
        grid_origin_x=grid_origin_x,
        grid_origin_y=grid_origin_y,
        cell_size=cell_size
    )

    # Gradiente e rugosita' dei nodi PRM dentro la griglia locale; quelli fuori tengono il
    # valore che hanno (0.0 se non ne hanno ancora uno)
    for node_id, (nx, ny) in prm_graph.nodes.items():
        if local_x_min <= nx <= local_x_max and local_y_min <= ny <= local_y_max:
            dx = nx - grid_origin_x
            dy = ny - grid_origin_y

            col_idx = int(dx / cell_size)
            row_idx = int(dy / cell_size)

            if 0 <= row_idx < num_y and 0 <= col_idx < num_x:
                flat_idx = row_idx * num_x + col_idx
                if flat_idx < len(grad_values):
                    prm_graph.node_gradients[node_id] = grad_values[flat_idx]
                if flat_idx < len(rough_values):
                    prm_graph.node_roughness[node_id] = rough_values[flat_idx]
        else:
            if node_id not in prm_graph.node_gradients:
                prm_graph.node_gradients[node_id] = 0.0
            if node_id not in prm_graph.node_roughness:
                prm_graph.node_roughness[node_id] = 0.0

    # La scansione appena fatta entra anche nello scarto preliminare degli archi
    # (prm_graph.LIVE_PREFILTER_OD_M): alla costruzione la mappa globale non ha ancora
    # confermato niente di questo posto, e gli archi che attraversano gli ostacoli appena
    # visti resterebbero validi. Qui obstacle_distance e' quella grezza: la fotografia
    # filtrata nasce nel ciclo di navigazione.
    try:
        prm_graph.set_live_obstacles(
            np.asarray(cells_obstacle_dist, dtype=np.float64).reshape((num_y, num_x)),
            np.asarray(valid_values, dtype=np.float64).reshape((num_y, num_x)) >= spotGrid.TERRAIN_VALID_MIN,
            grid_origin_x, grid_origin_y, cell_size, robot_x, robot_y, robot_yaw)
    except Exception as e:
        prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
        print(f"[ARCHI] Scansione non usabile per lo scarto preliminare ({e}).")

    t_build = time.perf_counter()
    # ATTENZIONE AI NOMI: qui `prm_graph` e' il PARAMETRO della funzione (l'istanza di PRM),
    # non il modulo omonimo. Le coppie candidate sono quelle del reticolo (vedi
    # _build_mission_prm); veti e pesi li calcola PRM._evaluate_and_add_edge.
    _rebuild_prm_edges(prm_graph, global_map)
    _timing_add('costruzione grafo', time.perf_counter() - t_build)
    _upd = getattr(getattr(prm_graph, 'mission_graph', None), 'last_update', None)
    print(f"[TEMPO] {'aggiornamento locale' if _upd and _upd[2] else 'costruzione completa'} "
          f"del grafo: {time.perf_counter() - t_build:.2f} s")
    if getattr(prm_graph, 'live_veto_last', 0):
        print(f"[ARCHI] {prm_graph.live_veto_last} archi scartati subito dalla scansione dal "
              f"vivo (passano a meno di {prm_graph_module.LIVE_PREFILTER_OD_M:.2f} m da un "
              f"ostacolo appena visto), prima della pendenza.")
    # Un tentativo nuovo: le trappole del tentativo precedente non valgono piu' (vedi
    # _trap_backtrack). Le penalita' sono uno strato separato dagli archi, quindi
    # sopravvivono alle ricostruzioni DENTRO il tentativo.
    prm_graph.clear_penalties_for_attempt()

    # ================================================================== #

    # Ricerca del punto target e pianificazione
    target_x, target_y, valid_samples, rejected_samples = find_best_point_in_cell(
        robot_x, robot_y, env, target_row, target_col, pts, cells_obstacle_dist, sampler,
        global_map=global_map)

    # L'obiettivo di questo tentativo, per il recupero subito dopo un fallimento
    # (vedi RECOVERY_SAME_GOAL_M nel ciclo principale).
    env._last_attempt_goal = (None if target_x is None or target_y is None
                              else (float(target_x), float(target_y)))
    if target_x is None or target_y is None:
        print(f"[FAIL] Nessun obiettivo valido nella cella ({target_row},{target_col}) dalla posizione attuale")
        visualize_grid_with_candidates(
            pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
            robot_x=robot_x, robot_y=robot_y,
            candidates={'rejected': rejected_samples, 'valid': valid_samples},
            chosen_point=(target_x, target_y), iteration=iteration, env=env,
            save_path=save_path, prm_graph=prm_graph, chosen_path=None,
            cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
            grad_values=grad_values, rough_values=rough_values
        )
        return False

    # Solo celle visitate + cella del robot + cella obiettivo (vedi PLAN_ONLY_EXPLORED_CELLS).
    _restrict_planning_to_explored_cells(prm_graph, env, target_row, target_col,
                                         (robot_x, robot_y))

    # --- Nodi di partenza e di arrivo: nodi ESISTENTI del grafo ---------------------------
    # add_stop_node aggancia la posizione al nodo adatto invece di crearne uno.
    # L'obiettivo e' gia' un nodo del reticolo (lo sceglie find_best_point_in_cell fra i
    # punti del campionatore).
    robot_node_id = prm_graph.add_stop_node(
        robot_x, robot_y, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
        label="partenza tentativo")
    prm_graph.set_robot_node(robot_node_id, 0.0)
    # Briciola GraphNav del punto di partenza del tentativo: e' quello a cui serve poter
    # tornare se la cella si rivela un vicolo cieco.
    graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client, robot_node_id,
                             target_row, target_col, force=True)
    goal_id = prm_graph.add_stop_node(
        target_x, target_y, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
        label=f"obiettivo cella ({target_row},{target_col})", is_robot=False)
    prm_graph.set_robot_node(robot_node_id, 0.0)
    path_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)

    if path_ids is None:
        print(
            f"[GRAFO DISCONNESSO] Nessun percorso nel PRM dal nodo {robot_node_id} al nodo "
            f"{goal_id}: i due nodi stanno in componenti separate del grafo. Non e' un "
            f"errore di Dijkstra -- vuol dire che tutti gli archi di collegamento sono "
            f"stati vetati (occupazione o pendenza). Vedi i conteggi in [PRM MISSION SUMMARY].")
        print(f" -> Coordinate partenza robot: ({robot_x:.2f}, {robot_y:.2f})")
        print(f" -> Coordinate target cella: ({target_x:.2f}, {target_y:.2f})")

    def _waypoints_from_ids(ids):
        """[(node_id, x, y), ...] senza il primo nodo (la posizione del robot)."""
        out = []
        for nid in ids[1:]:
            nx, ny = prm_graph.get_node_position(nid)
            out.append((nid, nx, ny))
        return out

    full_path_coords = None
    path_waypoints = []

    if path_ids is not None:
        full_path_coords = [prm_graph.get_node_position(nid) for nid in path_ids]
        path_waypoints = _waypoints_from_ids(path_ids)
    else:
        path_waypoints = None

    visualize_grid_with_candidates(
        pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
        robot_x=robot_x, robot_y=robot_y,
        candidates={'rejected': rejected_samples, 'valid': valid_samples},
        chosen_point=(target_x, target_y), iteration=iteration, env=env,
        save_path=save_path, prm_graph=prm_graph, chosen_path=full_path_coords,
        cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
        grad_values=grad_values, rough_values=rough_values
    )

    # ===================================================================================
    # CICLO DI NAVIGAZIONE CON IL FRONTE SICURO
    #
    # A ogni giro: scansione fresca, aggiornamento mappe e PRM, eventuale ripianificazione,
    # poi il FRONTE: fin dove il percorso e' confermato libero adesso. Il robot avanza
    # fino a li' (meno meta' corpo e margine), lungo il percorso.
    #   - il muso punta SEMPRE il prossimo punto: niente marcia di traverso; se la
    #     rotazione non e' possibile si prova il cono, poi l'arretramento dritto (vedi
    #     plan_cone_move e plan_straight_retreat);
    #   - la rotazione si giudica sulle celle davvero spazzate (spotGrid.rotation_is_clear),
    #     non sul cerchio circoscritto;
    #   - se l'obiettivo risulta irraggiungibile sui dati freschi se ne sceglie un altro
    #     nella cella, invece di aspettare e dichiarare un blocco (goal_still_reachable).
    # ===================================================================================
    replan_counter = 0
    need_refresh = True        # primo giro del tentativo: vedi REFRESH A EVENTI
    no_progress = 0
    block_replans = 0
    loop_iter = 0
    rotation_fails = {}     # tratto -> quante volte la rotazione verso di esso e' stata rifiutata
    rotation_discards = 0   # scarti per rifiuto di rotazione in QUESTO tentativo
    best_goal_dist = float('inf')   # per azzerare i rifiuti solo sui PROGRESSI
    idle_loops = 0          # giri consecutivi senza spostarsi (vedi MAX_IDLE_LOOPS)
    last_xy = None          # posizione al giro precedente, per contare i giri a vuoto
    rotation_fail_xy = {}   # tratto -> posizione del robot all'ultimo rifiuto
    retreats_in_a_row = 0   # arretramenti consecutivi: azzerato da un avanzamento vero
    early_replans = 0       # ripianificazioni alla prima cella bloccante (ramo 'wait')
    arrival_refusals = 0    # passi rifiutati perche' la posa d'arrivo non ha uscite
    goal_repicks = 0        # quante volte si e' riscelto il punto obiettivo
    tried_goals = []        # punti obiettivo gia' provati e risultati irraggiungibili
    trap_state = {'n': 0}   # ritorni indietro in QUESTO tentativo, mai azzerato (vedi _trap_backtrack)

    while path_waypoints and len(path_waypoints) > 0:
        loop_iter += 1
        t_iter = time.perf_counter()
        if loop_iter > MAX_LOOP_ITERATIONS:
            print(f"[STALLO] {MAX_LOOP_ITERATIONS} giri del ciclo di navigazione senza arrivare: "
                  f"interrompo il tentativo sulla cella ({target_row},{target_col}).")
            if verification_tracker is not None:
                verification_tracker.clear_path()
            return False

        if _check_and_recover_fall(robot_state_client, command_client, global_map,
                                   "durante il ciclo di navigazione") is not None:
            if verification_tracker is not None:
                verification_tracker.clear_path()
            retreat_along_traveled_path(env, robot_state_client, command_client,
                                        mobility_kwargs=mobility_kwargs)
            return False

        robot_x, robot_y, robot_z, robot_quat = spotUtils.getPosition(robot_state_client)

        robot_yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                               1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))

        # --- Guardia generica anti-stallo: quanti giri di fila senza spostarsi.
        if last_xy is not None and float(np.hypot(robot_x - last_xy[0], robot_y - last_xy[1])) < IDLE_MOVE_M:
            idle_loops += 1
        else:
            idle_loops = 0
        # I rifiuti di rotazione si azzerano solo quando il robot si AVVICINA davvero
        # all'obiettivo (20 cm meglio del meglio raggiunto finora), non quando si sposta e
        # basta: non potendo girarsi, il pianificatore puo' portare il robot un nodo piu' in
        # la' dalla parte sbagliata e riprovare da li', e cosi' via, allontanandosi con il
        # contatore sempre a zero (test_e2e_loop.py, scenario S11). Muoversi non e' fare
        # progressi.
        d_goal_now = float(np.hypot(robot_x - target_x, robot_y - target_y))
        if d_goal_now < best_goal_dist - 0.20:
            best_goal_dist = d_goal_now
            rotation_discards = 0
        last_xy = (robot_x, robot_y)
        if idle_loops >= MAX_IDLE_LOOPS:
            MISSION_STATS['idle_loops'] += idle_loops
            print(f"[STALLO] {idle_loops} giri di ciclo consecutivi senza spostarsi di "
                  f"{IDLE_MOVE_M:.2f} m, con il robot in ({robot_x:.2f}, {robot_y:.2f}). "
                  f"Qualunque sia il motivo, continuare a girare qui non porta da nessuna "
                  f"parte: rinuncio a entrare in ({target_row},{target_col}) da questo lato.")
            if verification_tracker is not None:
                verification_tracker.clear_path()
            return False

        # 1. SCANSIONE FRESCA, prima di tutto il resto
        grids_data_up, main_proto_up, all_grids_proto_up = local_grid.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'],
            robot_state_client
        )

        if grids_data_up is None or 'obstacle_distance' not in grids_data_up:
            print("[ERROR] Rilevamento locale fallito durante il tracking. Arresto precauzionale.")
            if verification_tracker is not None:
                verification_tracker.clear_path()
            return False
        t_scan = time.perf_counter()

        pts_up = grids_data_up['obstacle_distance']['pts']
        cells_obs_up = grids_data_up['obstacle_distance']['values']
        terrain_up = grids_data_up['terrain']['values']
        valid_up = grids_data_up['terrain_valid']['values']

        lg_proto_up = main_proto_up
        num_x_up = lg_proto_up.local_grid.extent.num_cells_x
        num_y_up = lg_proto_up.local_grid.extent.num_cells_y
        cell_size_up = lg_proto_up.local_grid.extent.cell_size

        transforms_snapshot_up = main_proto_up.local_grid.transforms_snapshot
        grid_frame_name_up = main_proto_up.local_grid.frame_name_local_grid_data
        vision_tform_grid_up = get_a_tform_b(transforms_snapshot_up, VISION_FRAME_NAME, grid_frame_name_up)
        grid_origin_x_up = vision_tform_grid_up.position.x
        grid_origin_y_up = vision_tform_grid_up.position.y

        footprint_mask_2d = local_grid.compute_robot_footprint_mask(
            pts_up, robot_x, robot_y, robot_yaw, num_x_up, num_y_up
        )

        terrain_real_up, is_valid_up, unwritten_up = local_grid.correct_terrain(
            terrain_up, valid_up, num_x_up, num_y_up,
            robot_footprint_mask=footprint_mask_2d, cell_size=cell_size_up,
            unwritten_value=_raw_zero(grids_data_up), unwritten_scale=_raw_scale(grids_data_up),
            return_unwritten=True
        )
        _log_ground_filter(local_grid, "scan-passo")

        grad_up, rough_up = local_grid.compute_gradient_and_roughness(
            terrain_real_up, valid_up, num_x_up, num_y_up, cell_size_up, is_valid=is_valid_up
        )

        obstacle_mask_updated = local_grid.fuse_obstacle_mask(
            cells_obstacle_dist=cells_obs_up,
            rough_values=rough_up,
            is_valid=is_valid_up,
            obstacle_threshold=spotGrid.OBSTACLE_THRESHOLD,
            rough_threshold=spotGrid.ROUGH_THRESHOLD
        )

        _log_footprint_diagnostic(local_grid, pts_up, robot_x, robot_y, robot_yaw,
                                  num_x_up, num_y_up, obstacle_mask_updated, "scan-passo",
                                  cells_obstacle_dist=cells_obs_up,
                                  rough_values=rough_up, is_valid=is_valid_up)

        t_proc = time.perf_counter()
        _gm_core_up = fold_scan_into_global_maps(
            global_map, global_terrain_map, pts_up, terrain_real_up, obstacle_mask_updated,
            is_valid_up, cells_obs_up, (num_y_up, num_x_up), (robot_x, robot_y, robot_yaw))
        t_maps = time.perf_counter()

        # 2. Dati locali freschi nel PRM, poi eventuale ripianificazione dal nodo attuale
        grad_up_2d = grad_up.reshape((num_y_up, num_x_up))
        rough_up_2d = rough_up.reshape((num_y_up, num_x_up))
        terrain_up_2d = terrain_real_up.reshape((num_y_up, num_x_up))

        # I dati locali si iniettano sempre: costa niente (salva i riferimenti agli array) e
        # servono comunque a sample_terrain_between_points per il gait del tratto che si sta
        # per percorrere.
        prm_graph.update_local_grid_data(
            grad_values_2d=grad_up_2d,
            rough_values_2d=rough_up_2d,
            terrain_values_2d=terrain_up_2d,
            valid_2d=slope_usable_mask(is_valid_up, cells_obs_up, (num_y_up, num_x_up)),
            grid_origin_x=grid_origin_x_up,
            grid_origin_y=grid_origin_y_up,
            cell_size=cell_size_up
        )

        # Fotografia della scansione appena fatta: serve gia' qui -- allo scarto preliminare
        # degli archi nel refresh (vedi prm_graph.LIVE_PREFILTER_OD_M), a quello dei nodi
        # negli ostacoli -- e poi al fronte sicuro. Va creata UNA volta sola per scansione:
        # make_grid_snapshot aggiorna anche il filtro temporale degli ostacoli.
        snap = arcVerification.make_grid_snapshot(
            cells_obs_up, rough_up, is_valid_up, valid_up, terrain_real_up,
            num_x_up, num_y_up, grid_origin_x_up, grid_origin_y_up, cell_size_up,
            robot_x, robot_y, unwritten=unwritten_up, robot_yaw=robot_yaw)

        # Mappa dello spazio libero: QUI, non insieme a global_map.update, perche' serve la
        # obstacle_distance filtrata che vive nella fotografia (vedi _update_clearance_map).
        # Si alimenta solo dal ciclo di navigazione, cosi' tutte le osservazioni passano per
        # lo stesso filtro.
        _update_clearance_map(global_map, pts_up, snap, is_valid_up, unwritten_up,
                              _gm_core_up, (robot_x, robot_y))

        # Lo scarto preliminare degli archi legge la obstacle_distance FILTRATA
        # (mediana su 3 scansioni + isteresi), la stessa del fronte.
        prm_graph.set_live_obstacles(snap.obstacle_dist, snap.seen, snap.origin_x, snap.origin_y,
                                     snap.cell_size, robot_x, robot_y, robot_yaw)

        # --- REFRESH A EVENTI ---------------------------------------------------------
        # Pesi e pendenze degli archi cambiano solo quando cambiano i dati sotto di essi,
        # cioe' quando la finestra si muove: a robot fermo un refresh ricalcolerebbe gli
        # stessi numeri, e circa meta' delle scansioni sono riscansioni da fermo (conferme
        # di blocco, giri senza movimento).
        # Si rinfresca solo quando serve: alla fine di un MOVIMENTO (completato,
        # interrotto o fallito, anche se ha attraversato piu' nodi e archi del PRM) e dopo
        # gli EVENTI che cambiano il piano (blocco confermato, arco scartato,
        # ripianificazione, obiettivo riscelto, ritorno con GraphNav, caduta).
        # QUESTO NON TOCCA LA SICUREZZA: la rete di protezione non sono i veti del grafo ma
        # il FRONTE SICURO, che si ricalcola da zero a ogni scansione (0.5 ms) e a ~5 Hz
        # durante il movimento nel thread. Rimandare il refresh ritarda solo il momento in
        # cui il PIANIFICATORE viene a sapere di un ostacolo nuovo; se il robot e' fermo non
        # c'e' nulla di nuovo da sapere, e se qualcosa entra davvero nel campo il fronte si
        # accorcia, la decisione diventa 'wait' o 'blocked' -- che sono eventi, quindi il
        # refresh scatta comunque. Il caso scoperto si richiude da solo.
        if need_refresh or getattr(prm_graph, 'needs_refresh', False):
            prm_graph.needs_refresh = False
            touched_nodes = prm_graph.refresh_local_edge_weights(
                global_map=global_map, edge_safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
            need_refresh = False
            MISSION_STATS['prm_refresh'] += 1
            n_live = (getattr(prm_graph, 'live_veto_last', 0)
                      + getattr(prm_graph, 'live_prune_last', 0))
            MISSION_STATS['live_arc_vetoes'] = MISSION_STATS.get('live_arc_vetoes', 0) + n_live
            if n_live:
                print(f"[ARCHI] {n_live} archi fuori dal grafo per la scansione dal vivo "
                      f"({prm_graph.live_veto_last} rivalutati, {prm_graph.live_prune_last} "
                      f"oltre il raggio di refresh), prima della pendenza.")
        else:
            # Nessun dato nuovo sotto gli archi: si tengono i pesi del giro precedente e
            # non si ripianifica (il blocco sotto gira solo se touched_nodes non e' vuoto).
            touched_nodes = set()
            MISSION_STATS['prm_refresh_skipped'] += 1


        # Nodi DENTRO gli ostacoli: fuori dal grafo (vedi prune_prm_nodes_in_obstacles).
        pruned_nodes = prune_prm_nodes_in_obstacles(prm_graph, snap, global_map=global_map)
        if pruned_nodes:
            MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
            print(f"[NODI] {len(pruned_nodes)} nodi scartati: meno di "
                  f"{PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero dove li vediamo "
                  f"(primi: {pruned_nodes[:8]}).")

        # Quanto la mappa dello spazio libero si discosta dal criterio in uso (vedi
        # spotGrid.CLEARANCE_MAP_DECIDES). Finche' decide la mappa binaria questa riga e'
        # solo una misura: dice quanti archi cambierebbero verdetto commutando.
        _log_clearance_gap(prm_graph, global_map)

        # Archi gia' rifiutati per rotazione DA QUESTO NODO: restano fuori finche' il robot
        # non si sposta. Lo scarto e' soft, quindi refresh_local_edge_weights li rimetterebbe
        # in gioco a ogni giro, e il robot potrebbe restare fermo alternando due archi
        # rifiutati senza mai arrivare alla ritirata. Il robot puo' restare agganciato allo
        # stesso nodo anche dopo essersi spostato, quindi lo scarto vale solo finche' il robot
        # e' dove la rotazione e' stata rifiutata.
        for (from_id, to_id), n_fails in rotation_fails.items():
            fx, fy = rotation_fail_xy.get((from_id, to_id), (robot_x, robot_y))
            if (from_id == robot_node_id and n_fails > 0
                    and float(np.hypot(robot_x - fx, robot_y - fy)) <= ANCHOR_ON_NODE_M):
                mark_edge_blocked_soft(prm_graph, from_id, to_id)

        if touched_nodes:
            current_plan_ids = [robot_node_id] + [w[0] for w in path_waypoints]
            chosen_path_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id,
                                                           current_path_ids=current_plan_ids, margin=0.10)

            if chosen_path_ids is None:
                print(f"[REPLAN FALLITO] Nessun percorso esistente da {robot_node_id} a "
                      f"{goal_id} nel grafo aggiornato. Mantengo il percorso precedente: il "
                      f"fronte sicuro decide comunque fin dove si puo' avanzare.")
            elif chosen_path_ids != current_plan_ids:
                path_waypoints = _waypoints_from_ids(chosen_path_ids)
                full_path_coords = [prm_graph.get_node_position(nid) for nid in chosen_path_ids]
                no_progress = 0   # percorso nuovo: la conferma di un blocco riparte da zero
                print(f"[REPLAN] Percorso aggiornato ({len(touched_nodes)} nodi locali rivalutati): "
                      f"{len(path_waypoints)} waypoint rimanenti.")

                replan_counter += 1
                MISSION_STATS['replan_count'] += 1
                replan_save_path = os.path.join(
                    mission_folder, f"iteration_{iteration}_cell_{target_row}_{target_col}_replan_{replan_counter}.png"
                ) if mission_folder else None

                visualize_grid_with_candidates(
                    pts=pts_up, terrain_real=terrain_real_up, obstacle_mask=obstacle_mask_updated,
                    robot_x=robot_x, robot_y=robot_y,
                    candidates={'rejected': [], 'valid': []},
                    chosen_point=(target_x, target_y), iteration=iteration, env=env,
                    save_path=replan_save_path, prm_graph=prm_graph, chosen_path=full_path_coords,
                    cells_obstacle_dist=cells_obs_up, valid_values=valid_up,
                    grad_values=grad_up, rough_values=rough_up,
                    include_diagnostics=False  # leggero: vista principale e mappa globale
                )
            else:
                print(f"[REPLAN STABILE] Percorso invariato da {robot_node_id} a {goal_id} "
                      f"({len(touched_nodes)} nodi locali rivalutati): l'alternativa non "
                      f"supera la soglia di stabilita'. Proseguo sul piano corrente.")

        if not path_waypoints:
            break
        t_prm = time.perf_counter()

        # 3. FRONTE SICURO sulla scansione appena fatta ---------------------------------
        # La scorciatoia rispetta gli archi gia' scartati: vedi shortcut_index.
        k_short = shortcut_index((robot_x, robot_y), path_waypoints, snap, global_map,
                                 prm=prm_graph, robot_node_id=robot_node_id)
        if k_short > 0:
            skipped = [w[0] for w in path_waypoints[:k_short]]
            path_waypoints = path_waypoints[k_short:]
            print(f"[SCORCIATOIA] Tratto diretto confermato libero verso il nodo {path_waypoints[0][0]}: "
                  f"salto {len(skipped)} nodi intermedi {skipped}.")
            MISSION_STATS['shortcuts'] += 1
        polyline = [(robot_x, robot_y)] + [(x, y) for _, x, y in path_waypoints]
        frontier = arcVerification.compute_safe_frontier(polyline, snap)
        print(arcVerification.describe_frontier(frontier))

        path_version = None
        if verification_tracker is not None:
            path_version = verification_tracker.update_path(polyline[1:])

        decision = decide_next_move((robot_x, robot_y), path_waypoints, frontier, no_progress)
        next_node_id, next_x, next_y = path_waypoints[0]
        keep_heading = False     # sempre False (niente traverso): resta nei dati salvati e in navigate_to

        # Pacchetto della scansione salvato QUI, prima del blocco rotazione: i rami di
        # rotazione rifiutata/cono/arretramento/trappola fanno `continue`, e salvando dopo
        # quelle scansioni non finirebbero su disco. Il rifiuto si ricalcola offline da
        # obstacle_distance, robot_xyz, robot_yaw e decision_target.
        if SAVE_SCAN_BUNDLES and mission_folder is not None:
            gf = local_grid.last_ground_filter
            shape_up = (num_y_up, num_x_up)
            _save_npz_async(
                os.path.join(mission_folder, "scans",
                             f"scan_it{iteration:03d}_cell{target_row}_{target_col}_{loop_iter:03d}.npz"),
                time=time.time(), iteration=iteration, cell=np.array([target_row, target_col]),
                loop_iter=loop_iter,
                terrain_raw=np.asarray(terrain_up, dtype=np.float32).reshape(shape_up),
                terrain_valid_raw=np.asarray(valid_up, dtype=np.float32).reshape(shape_up),
                obstacle_distance=np.asarray(cells_obs_up, dtype=np.float32).reshape(shape_up),
                # Il layer su cui il fronte DECIDE (mediana sulle ultime letture + isteresi,
                # vedi arcVerification._ObstacleHistory) e la maschera del latch. Senza questi
                # due un rigioco offline non puo' riprodurre i blocchi che nascono dalle
                # letture del thread durante il movimento. Stesso orientamento degli altri:
                # (num_y, num_x), riga = y.
                obstacle_distance_filtered=np.asarray(snap.obstacle_dist, dtype=np.float32),
                blocked_latch=(np.zeros(shape_up, bool) if snap.blocked_latch is None
                               else np.asarray(snap.blocked_latch, dtype=bool)),
                terrain_corrected=np.asarray(terrain_real_up, dtype=np.float32).reshape(shape_up),
                is_valid=np.asarray(is_valid_up, dtype=bool).reshape(shape_up),
                roughness=np.asarray(rough_up, dtype=np.float32).reshape(shape_up),
                gradient=np.asarray(grad_up, dtype=np.float32).reshape(shape_up),
                obstacle_mask=np.asarray(obstacle_mask_updated, dtype=np.int8).reshape(shape_up),
                footprint_mask=(np.zeros(shape_up, bool) if footprint_mask_2d is None
                                else np.asarray(footprint_mask_2d, dtype=bool).reshape(shape_up)),
                grid_origin=np.array([grid_origin_x_up, grid_origin_y_up]), cell_size=cell_size_up,
                robot_xyz=np.array([robot_x, robot_y, robot_z]), robot_yaw=robot_yaw,
                ground_z=np.nan if gf.get('ground_z') is None else gf['ground_z'],
                ground_n_above=gf.get('n_above', 0),
                mission_z0=_mission_z0(),
                terrain_raw_zero=np.nan if _raw_zero(grids_data_up) is None else _raw_zero(grids_data_up),
                terrain_scale=np.nan if not _raw_scale(grids_data_up) else _raw_scale(grids_data_up),
                unwritten=np.asarray(unwritten_up, dtype=bool).reshape(shape_up),
                polyline=np.array(polyline), path_node_ids=np.array([w[0] for w in path_waypoints]),
                robot_node_id=robot_node_id, goal_xy=np.array([target_x, target_y]),
                frontier_dist=frontier['dist'], frontier_reason=frontier['reason'],
                frontier_stop_xy=np.array(frontier['stop_xy']),
                frontier_value=np.nan if frontier['value'] is None else frontier['value'],
                frontier_od_robot=frontier['od_robot'], frontier_half_along=frontier['half_along'],
                frontier_clearance=frontier['clearance_nominal'], frontier_path_len=frontier['path_len'],
                allowed_advance=arcVerification.allowed_advance(frontier),
                keep_heading=keep_heading, no_progress=no_progress,
                decision_kind=decision['kind'],
                decision_target=np.array(decision.get('target', (np.nan, np.nan)), dtype=np.float64))
            MISSION_STATS['scan_bundles'] += 1

        # 4. ROTAZIONE -------------------------------------------------------------------
        # Il muso punta SEMPRE il punto da raggiungere. Se la rotazione non e' possibile NON
        # si cammina di traverso (vedi ROTAZIONE E ARRETRAMENTO in cima al file). Nell'ordine:
        # avanzare dentro il cono di rotazione possibile, arretrare dritti se da li' la
        # rotazione diventa possibile, riconoscere una trappola, scartare il tratto.
        if decision['kind'] == 'move':
            rot = rotation_plan(robot_yaw, (robot_x, robot_y), decision['target'],
                                frontier['od_robot'], snap=snap)
            if not rot['turn']:
                MISSION_STATS['rotation_refusals'] += 1
                edge_key = (robot_node_id, next_node_id)
                rotation_fails[edge_key] = rotation_fails.get(edge_key, 0) + 1
                rotation_fail_xy[edge_key] = (robot_x, robot_y)
                margin_txt = (f"margine {rot['margin']:+.2f} m sulle celle spazzate"
                              if rot['margin'] is not None else f"motivo: {rot['why']}")
                print(f"[ROTAZIONE] Servono {rot['dyaw_deg']:+.0f} gradi verso il nodo "
                      f"{next_node_id}, ma il corpo toccherebbe ({margin_txt}; "
                      f"obstacle_distance sotto il robot {frontier['od_robot']:.2f} m). "
                      f"Non cammino di traverso: tentativo "
                      f"{rotation_fails[edge_key]}/{MAX_ROTATION_FAILS_SAME_EDGE}.")

                # --- 4a. PRIMA DI TUTTO: posso ANDARE AVANTI dentro il cono di rotazione che
                # mi e' concesso? Spesso andare dritti, o quasi, avvicina all'obiettivo piu'
                # di qualunque arretramento (vedi plan_cone_move).
                cone_goal, _ = arcVerification.point_along(polyline, CONE_PATH_LOOKAHEAD_M)
                cone = plan_cone_move((robot_x, robot_y), robot_yaw, snap, cone_goal)
                if cone is not None:
                    print(f"[CONO] Non posso girarmi di {rot['dyaw_deg']:+.0f} gradi, ma ruotando di "
                          f"{cone['dyaw_deg']:+.0f} posso avanzare {cone['dist']:.2f} m "
                          f"({cone['reason']}) e avvicinarmi di {cone['gain']:.2f} m al percorso "
                          f"({cone_goal[0]:.2f}, {cone_goal[1]:.2f}, {CONE_PATH_LOOKAHEAD_M:.1f} m "
                          f"avanti lungo il percorso). Vado, invece di arretrare.")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    MISSION_STATS['cone_moves'] += 1
                    cx, cy = cone['target']
                    need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
                    ok_c, dist_c, _ = navigate_to(
                        cx, cy, robot_x, robot_y, robot_state_client, command_client,
                        get_a_tform_b(lg_proto_up.local_grid.transforms_snapshot,
                                      VISION_FRAME_NAME, BODY_FRAME_NAME),
                        should_abort=None, keep_heading=False, **mobility_kwargs)
                    if dist_c > 0.05:
                        if not hasattr(env, '_traveled_arcs'):
                            env._traveled_arcs = []
                        ax_c, ay_c, _, _ = spotUtils.getPosition(robot_state_client)
                        env._traveled_arcs.append((robot_x, robot_y, ax_c, ay_c))
                        robot_node_id = prm_graph.add_stop_node(
                            ax_c, ay_c, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="sosta (avanzamento nel cono)")
                        prm_graph.set_robot_node(robot_node_id, 0.0)
                        retreats_in_a_row = 0      # ha fatto strada vera: la catena si rompe
                    else:
                        print(f"[CONO] Non sono riuscito ad avanzare (percorsi {dist_c:.2f} m).")
                    no_progress = 0
                    continue

                # --- 4b. Arretrare, ma SOLO se serve a qualcosa (vedi retreat_would_help).
                back = plan_straight_retreat((robot_x, robot_y), robot_yaw, snap)
                helps, margin_back = (False, None)
                if back is not None:
                    helps, margin_back = retreat_would_help(
                        snap, (back[0], back[1]), robot_yaw, rot['dyaw_deg'])
                    if not helps:
                        MISSION_STATS['retreats_suppressed'] += 1
                        mb = "" if margin_back is None else f" (margine {margin_back:+.2f} m da li')"
                        print(f"[ARRETRAMENTO] Potrei arretrare di {back[2]:.2f} m, ma da li' la "
                              f"rotazione di {rot['dyaw_deg']:+.0f} gradi resterebbe impossibile{mb}: "
                              f"non lo faccio, sposterei il problema di 60 cm.")
                if (back is not None and helps
                        and retreats_in_a_row < MAX_STRAIGHT_RETREATS_IN_A_ROW
                        and rotation_fails[edge_key] < MAX_ROTATION_FAILS_SAME_EDGE):
                    bx, by, bdist = back
                    retreats_in_a_row += 1
                    print(f"[ARRETRAMENTO] Indietro dritto di {bdist:.2f} m verso "
                          f"({bx:.2f}, {by:.2f}), senza girarmi, per fare spazio alla rotazione "
                          f"({retreats_in_a_row}/{MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi).")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    MISSION_STATS['straight_retreats'] += 1
                    need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
                    ok_back, dist_back = movements.move_to_world_point_without_turning(
                        bx, by, "vision", command_client, robot_state_client, **mobility_kwargs)
                    if dist_back > 0.05:
                        if not hasattr(env, '_traveled_arcs'):
                            env._traveled_arcs = []
                        ax_b, ay_b, _, _ = spotUtils.getPosition(robot_state_client)
                        env._traveled_arcs.append((robot_x, robot_y, ax_b, ay_b))
                        robot_node_id = prm_graph.add_stop_node(
                            ax_b, ay_b, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="sosta (arretramento per ruotare)")
                        prm_graph.set_robot_node(robot_node_id, 0.0)
                    else:
                        print(f"[ARRETRAMENTO] Non sono riuscito ad arretrare "
                              f"(percorsi {dist_back:.2f} m).")
                    no_progress = 0
                    continue

                if back is None:
                    print("[ARRETRAMENTO] Non c'e' spazio per arretrare dritto.")
                elif helps and retreats_in_a_row >= MAX_STRAIGHT_RETREATS_IN_A_ROW:
                    print(f"[ARRETRAMENTO] Gia' {retreats_in_a_row} arretramenti di fila senza "
                          f"combinare niente: smetto di arretrare.")

                # Ne' il cono ne' l'arretramento: e' una trappola o solo un arco sbagliato?
                # Se e' una trappola non ha senso scartare archi uno dopo l'altro: si torna
                # indietro subito.
                trapped_rot, fan_rot = is_trapped((robot_x, robot_y), robot_yaw, snap)
                n_us_r, fan_txt_r = describe_escape_fan(fan_rot)
                if trapped_rot:
                    MISSION_STATS['traps_detected'] += 1
                    print(f"[TRAPPOLA] Non posso girarmi verso {next_node_id} e nessuna delle "
                          f"{len(fan_rot)} direzioni del giro e' insieme percorribile e "
                          f"raggiungibile girandosi. Le piu' larghe: {fan_txt_r}. Ripercorro il "
                          f"tratto da cui sono venuto.")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    # Ritorno con GraphNav invece che a zampe all'indietro: si torna di un
                    # waypoint, si riprova a pianificare da li', e cosi' via fino a
                    # GRAPHNAV_MAX_BACKTRACK_HOPS. La ritirata a zampe resta dentro come ultima rete.
                    esc = _trap_backtrack(
                        trap_state, (robot_node_id, next_node_id),
                        env, prm_graph, goal_id, recordingInterface, robot_state_client,
                        command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                        label=f"dal nodo {robot_node_id}")
                    if esc['ok']:
                        path_waypoints = _waypoints_from_ids(esc['ids'])
                        full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                        robot_node_id = esc['robot_node']
                        no_progress = 0
                        early_replans = 0
                        retreats_in_a_row = 0
                        rotation_discards = 0
                        best_goal_dist = float('inf')
                        continue
                    return False
                print(f"[ROTAZIONE] Direzioni utilizzabili da qui: {n_us_r}/{len(fan_rot)} "
                      f"({fan_txt_r}).")
                # Limite per TENTATIVO, non per coppia di nodi (vedi MAX_ROTATION_DISCARDS).
                rotation_discards += 1
                if rotation_discards > MAX_ROTATION_DISCARDS:
                    print(f"[ROTAZIONE] Gia' {rotation_discards - 1} tratti scartati in questo "
                          f"tentativo perche' la rotazione era impossibile, e sono sempre qui: "
                          f"non e' l'arco, e' il posto. Ripercorro il tratto da cui sono venuto.")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    esc = _trap_backtrack(
                        trap_state, (robot_node_id, next_node_id),
                        env, prm_graph, goal_id, recordingInterface, robot_state_client,
                        command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                        label=f"dal nodo {robot_node_id} (troppi rifiuti di rotazione)")
                    if esc['ok']:
                        path_waypoints = _waypoints_from_ids(esc['ids'])
                        full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                        robot_node_id = esc['robot_node']
                        no_progress = 0
                        early_replans = 0
                        retreats_in_a_row = 0
                        rotation_discards = 0
                        best_goal_dist = float('inf')
                        continue
                    return False
                print(f"[ROTAZIONE] Scarto il tratto {robot_node_id}->{next_node_id}: "
                      f"richiede di girarsi dove non c'e' spazio DA QUI. Ripianifico.")
                # Scarto NON permanente: un rifiuto di rotazione dipende dalla POSA del robot,
                # non dall'arco -- lo stesso arco, affrontato da un altro punto o con un altro
                # muso, puo' essere percorribilissimo. Permanente resta solo cio' che e'
                # confermato da tre scansioni ferme o da un fallimento fisico.
                mark_edge_blocked_soft(prm_graph, robot_node_id, next_node_id)
                new_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)
                if new_ids is not None:
                    path_waypoints = _waypoints_from_ids(new_ids)
                    full_path_coords = [prm_graph.get_node_position(nid) for nid in new_ids]
                    no_progress = 0
                    continue
                print("[ROTAZIONE] Nessuna alternativa nel grafo dal punto in cui sono. "
                      "Mi stacco arretrando e rimando la cella.")
                if verification_tracker is not None:
                    verification_tracker.clear_path()
                # Ritorno con GraphNav invece che a zampe all'indietro: si torna di un
                # waypoint, si riprova a pianificare da li', e cosi' via fino a
                # GRAPHNAV_MAX_BACKTRACK_HOPS. La ritirata a zampe resta dentro come ultima rete.
                esc = _trap_backtrack(
                    trap_state, (robot_node_id, next_node_id),
                    env, prm_graph, goal_id, recordingInterface, robot_state_client,
                    command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                    label=f"dal nodo {robot_node_id}", skip_if_graph_cannot_help=True)
                if esc['ok']:
                    path_waypoints = _waypoints_from_ids(esc['ids'])
                    full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                    robot_node_id = esc['robot_node']
                    no_progress = 0
                    early_replans = 0
                    retreats_in_a_row = 0
                    rotation_discards = 0
                    best_goal_dist = float('inf')
                    continue
                return False
        t_front = time.perf_counter()

        _timing_add('scansione', t_scan - t_iter)
        _timing_add('elaborazione griglia', t_proc - t_scan)
        _timing_add('mappe globali', t_maps - t_proc)
        _timing_add('PRM e ripianificazione', t_prm - t_maps)
        _timing_add('fronte e decisione', t_front - t_prm)
        print(f"[TEMPO] scansione {t_scan - t_iter:.2f} s | elaborazione {t_proc - t_scan:.2f} | "
              f"mappe {t_maps - t_proc:.2f} | PRM {t_prm - t_maps:.2f} | fronte {t_front - t_prm:.2f} "
              f"| decisione: {decision['kind']}")


        if decision['kind'] == 'reached_node':
            robot_node_id = next_node_id
            prm_graph.set_robot_node(robot_node_id, 0.0)
            path_waypoints.pop(0)
            no_progress = 0
            continue

        if decision['kind'] == 'arrived':
            print(f"[OBIETTIVO] Il fronte non permette gli ultimi {frontier['path_len']:.2f} m "
                  f"({frontier['reason']}), entro la tolleranza di {GOAL_REACHED_TOLERANCE_M:.2f} m: "
                  f"considero raggiunto l'obiettivo.")
            break

        if decision['kind'] == 'wait':
            # Prima di insistere: l'obiettivo e' ancora raggiungibile sui dati FRESCHI? Il
            # punto viene scelto all'inizio del tentativo, da metri di distanza, quando la
            # cella e' ancora tutta ignota; arrivandoci puo' risultare a ridosso di un muro,
            # e allora nessun numero di scansioni lo rendera' raggiungibile.
            if goal_repicks < MAX_BLOCK_REPLANS and not goal_still_reachable(snap, target_x, target_y):
                goal_repicks += 1
                need_refresh = True          # piano cambiato: vedi REFRESH A EVENTI
                MISSION_STATS['goal_repicks'] += 1
                r_g, c_g, in_g = arcVerification._cells(
                    snap, np.array([float(target_x)]), np.array([float(target_y)]))
                od_goal = float(snap.obstacle_dist[r_g[0], c_g[0]]) if in_g[0] else float('nan')
                print(f"[TARGET] L'obiettivo ({target_x:.2f}, {target_y:.2f}) ha "
                      f"obstacle_distance {od_goal:.2f} m, sotto i "
                      f"{spotGrid.FRONTIER_CLEARANCE_M:.2f} m che il fronte pretende: non e' "
                      f"raggiungibile da nessuna direzione. Ne cerco un altro nella cella "
                      f"({goal_repicks}/{MAX_BLOCK_REPLANS}).")
                tried_goals.append((float(target_x), float(target_y)))
                new_tx, new_ty, _vs, _rs = find_best_point_in_cell(
                    robot_x, robot_y, env, target_row, target_col, pts_up, cells_obs_up,
                    sampler, global_map=global_map, exclude=tried_goals)
                if new_tx is not None and new_ty is not None:
                    target_x, target_y = new_tx, new_ty
                    best_goal_dist = float('inf')    # nuovo obiettivo: progressi da capo
                    goal_id = prm_graph.add_stop_node(
                        target_x, target_y, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
                        label=f"obiettivo riscelto cella ({target_row},{target_col})",
                        is_robot=False)
                    new_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)
                    if new_ids is not None:
                        path_waypoints = _waypoints_from_ids(new_ids)
                        full_path_coords = [prm_graph.get_node_position(nid) for nid in new_ids]
                        no_progress = 0
                        print(f"[REPLAN] Verso il nuovo obiettivo ({target_x:.2f}, "
                              f"{target_y:.2f}): {len(path_waypoints)} waypoint.")
                        continue
                    print("[TARGET] Nessun percorso nel grafo verso il nuovo obiettivo.")
                else:
                    print("[TARGET] Nessun altro punto valido in questa cella.")

            # RIPIANIFICA SUBITO, alla PRIMA cella bloccante, invece di restare fermo tre
            # scansioni sullo stesso arco. Il rumore di obstacle_distance e' gia' filtrato
            # (mediana + isteresi in arcVerification), quindi una cella bloccante e'
            # un'informazione da usare: l'arco si scarta e si cerca subito un percorso che
            # passi da un'altra parte. Le scansioni da fermo restano come ultima rete, per
            # dichiarare la cella irraggiungibile.
            if (early_replans < MAX_EARLY_REPLANS
                    and frontier['reason'] in ('ostacolo', 'rugosita', 'pendenza')
                    and path_waypoints):
                poly_now = [(robot_x, robot_y)] + [(x, y) for _, x, y in path_waypoints]
                _, seg_blk = arcVerification.point_along(poly_now, frontier['dist'])
                seg_blk = int(seg_blk)
                ea = robot_node_id if seg_blk == 0 else path_waypoints[seg_blk - 1][0]
                eb = path_waypoints[min(seg_blk, len(path_waypoints) - 1)][0]
                sx_b, sy_b = frontier['stop_xy']

                # Prima di scartare l'ennesimo arco: sono in trappola? Se tutti gli archi del
                # nodo passano accanto allo stesso ostacolo, il blocco e' un LUOGO, non un
                # arco, e scartarli uno per uno non serve. Il ventaglio lo vede in una scansione.
                trapped, fan = is_trapped((robot_x, robot_y), robot_yaw, snap)
                n_us, fan_txt = describe_escape_fan(fan)
                if trapped:
                    MISSION_STATS['traps_detected'] += 1
                    print(f"[TRAPPOLA] Nessuna delle {len(fan)} direzioni del giro e' insieme "
                          f"percorribile e raggiungibile girandosi. Le piu' larghe: {fan_txt}. "
                          f"Non ho dove andare ne' come girarmi: ripercorro il tratto da cui "
                          f"sono venuto, senza scartare altri archi.")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    # Ritorno con GraphNav invece che a zampe all'indietro: si torna di un
                    # waypoint, si riprova a pianificare da li', e cosi' via fino a
                    # GRAPHNAV_MAX_BACKTRACK_HOPS. La ritirata a zampe resta dentro come ultima rete.
                    esc = _trap_backtrack(
                        trap_state, (ea, eb),
                        env, prm_graph, goal_id, recordingInterface, robot_state_client,
                        command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                        label=f"dal nodo {robot_node_id}")
                    if esc['ok']:
                        path_waypoints = _waypoints_from_ids(esc['ids'])
                        full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                        robot_node_id = esc['robot_node']
                        no_progress = 0
                        early_replans = 0
                        retreats_in_a_row = 0
                        rotation_discards = 0
                        best_goal_dist = float('inf')
                        continue
                    return False

                early_replans += 1
                MISSION_STATS['early_replans'] += 1
                print(f"[SUBITO] {frontier['reason']} in ({sx_b:.2f}, {sy_b:.2f}) sul tratto "
                      f"{ea}->{eb}: lo scarto adesso e cerco un altro percorso, senza "
                      f"aspettare la conferma da fermo ({early_replans}/{MAX_EARLY_REPLANS}). "
                      f"Direzioni utilizzabili da qui: {n_us}/{len(fan)}.")
                # Scarto NON permanente: dopo una sola scansione l'arco va tolto dal piano,
                # non condannato per la missione (vedi mark_edge_blocked_soft).
                mark_edge_blocked_soft(prm_graph, ea, eb)
                new_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)
                if new_ids is not None and len(new_ids) > 1:
                    path_waypoints = _waypoints_from_ids(new_ids)
                    full_path_coords = [prm_graph.get_node_position(nid) for nid in new_ids]
                    no_progress = 0
                    print(f"[SUBITO] Percorso alternativo con {len(path_waypoints)} waypoint, "
                          f"dal punto in cui sono.")
                    continue
                print("[SUBITO] Il grafo non offre alternative da qui: resto sul percorso e "
                      "lascio decidere il fronte.")

            no_progress += 1
            print(f"[FRONTE] Non posso avanzare nemmeno di {MIN_PARTIAL_STEP_M:.2f} m: riguardo "
                  f"da fermo ({no_progress}/{FRONTIER_BLOCK_CONFIRM_SCANS} scansioni).")
            time.sleep(FRONTIER_RESCAN_WAIT_S)
            continue

        if decision['kind'] == 'blocked':
            k = decision['segment']
            edge_a = robot_node_id if k == 0 else path_waypoints[k - 1][0]
            edge_b = path_waypoints[min(k, len(path_waypoints) - 1)][0]
            sx, sy = frontier['stop_xy']
            print(f"[BLOCCO] Confermato da vicino: {FRONTIER_BLOCK_CONFIRM_SCANS} scansioni consecutive "
                  f"senza poter avanzare. Motivo: {frontier['reason']} in ({sx:.2f}, {sy:.2f}), "
                  f"sul tratto {edge_a}->{edge_b}.")
            MISSION_STATS['frontier_blocks'] += 1

            if mission_folder is not None:
                try:
                    npz_blocked = os.path.join(
                        mission_folder,
                        f"iteration_{iteration}_cell_{target_row}_{target_col}_BLOCKED_{block_replans + 1}.npz")
                    np.savez_compressed(
                        npz_blocked,
                        obstacle_distance=snap.obstacle_dist, rough_veto=snap.rough_veto,
                        seen=snap.seen, terrain=snap.terrain,
                        terrain_raw=np.asarray(terrain_up).reshape((num_y_up, num_x_up)),
                        terrain_valid_raw=np.asarray(valid_up).reshape((num_y_up, num_x_up)),
                        obstacle_mask=np.asarray(obstacle_mask_updated).reshape((num_y_up, num_x_up)),
                        roughness=rough_up_2d,
                        grid_origin_x=snap.origin_x, grid_origin_y=snap.origin_y, cell_size=snap.cell_size,
                        robot_xy=np.array([robot_x, robot_y]), robot_yaw=robot_yaw,
                        polyline=np.array(polyline), frontier_dist=frontier['dist'],
                        frontier_reason=frontier['reason'], frontier_stop_xy=np.array(frontier['stop_xy']),
                        ground_z=np.nan if local_grid.last_ground_filter.get('ground_z') is None
                        else local_grid.last_ground_filter['ground_z'])
                    print(f"[DATA SAVE] Dati del blocco salvati in {os.path.basename(npz_blocked)}")
                except Exception as e:
                    print(f"[DATA SAVE] Impossibile salvare i dati del blocco: {e}")

            # Trappola? Se non c'e' nessuna direzione utilizzabile, ripianificare
            # e' inutile per costruzione: il problema non e' l'arco, e' il posto.
            trapped_blk, fan_blk = is_trapped((robot_x, robot_y), robot_yaw, snap)
            n_us_b, fan_txt_b = describe_escape_fan(fan_blk)
            print(f"[BLOCCO] Direzioni utilizzabili da qui: {n_us_b}/{len(fan_blk)} ({fan_txt_b}).")
            if trapped_blk:
                MISSION_STATS['traps_detected'] += 1
                print("[TRAPPOLA] Nessuna via d'uscita girandomi: non scarto altri archi, "
                      "ripercorro il tratto da cui sono venuto.")
                if verification_tracker is not None:
                    verification_tracker.clear_path()
                # Ritorno con GraphNav invece che a zampe all'indietro: si torna di un
                # waypoint, si riprova a pianificare da li', e cosi' via fino a
                # GRAPHNAV_MAX_BACKTRACK_HOPS. La ritirata a zampe resta dentro come ultima rete.
                esc = _trap_backtrack(
                    trap_state, (edge_a, edge_b),
                    env, prm_graph, goal_id, recordingInterface, robot_state_client,
                    command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                    label=f"dal nodo {robot_node_id}")
                if esc['ok']:
                    path_waypoints = _waypoints_from_ids(esc['ids'])
                    full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                    robot_node_id = esc['robot_node']
                    no_progress = 0
                    early_replans = 0
                    retreats_in_a_row = 0
                    rotation_discards = 0
                    best_goal_dist = float('inf')
                    continue
                return False

            # Qui il blocco e' confermato da FRONTIER_BLOCK_CONFIRM_SCANS scansioni ferme:
            # l'invalidazione resta permanente (a differenza di [SUBITO], vedi
            # mark_edge_blocked_soft). Eccezione: sul primo tratto, se il robot
            # non e' sul nodo a cui e' agganciato, il tratto bloccato e' robot->nodo, non
            # l'arco del grafo: si scarta l'arco solo per ora.
            if k == 0 and not _robot_on_node(prm_graph, edge_a, robot_x, robot_y):
                mark_edge_blocked_soft(prm_graph, edge_a, edge_b)
            else:
                prm_graph.mark_edge_invalid(edge_a, edge_b)
            block_replans += 1
            no_progress = 0

            new_ids = None
            if block_replans <= MAX_BLOCK_REPLANS:
                new_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)
            if new_ids is not None:
                path_waypoints = _waypoints_from_ids(new_ids)
                full_path_coords = [prm_graph.get_node_position(nid) for nid in new_ids]
                print(f"[REPLAN] Dopo il blocco ({block_replans}/{MAX_BLOCK_REPLANS}): percorso "
                      f"alternativo con {len(path_waypoints)} waypoint, dal punto in cui sono.")
                continue

            print(f"[BLOCCO] Nessuna alternativa dal punto in cui sono"
                  + (f" (gia' {MAX_BLOCK_REPLANS} ripianificazioni dopo blocchi)."
                     if block_replans > MAX_BLOCK_REPLANS else ".")
                  + " Mi stacco arretrando e rimando la cella.")
            if verification_tracker is not None:
                verification_tracker.clear_path()

            # Ritorno con GraphNav invece che a zampe all'indietro: si torna di un
            # waypoint, si riprova a pianificare da li', e cosi' via fino a
            # GRAPHNAV_MAX_BACKTRACK_HOPS. La ritirata a zampe resta dentro come ultima rete.
            esc = _trap_backtrack(
                trap_state, (edge_a, edge_b),
                env, prm_graph, goal_id, recordingInterface, robot_state_client,
                command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                label=f"dal nodo {robot_node_id}", skip_if_graph_cannot_help=True)
            if esc['ok']:
                path_waypoints = _waypoints_from_ids(esc['ids'])
                full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                robot_node_id = esc['robot_node']
                no_progress = 0
                early_replans = 0
                retreats_in_a_row = 0
                rotation_discards = 0
                best_goal_dist = float('inf')
                continue

            blocked_save_path = os.path.join(mission_folder,
                                             f"iteration_{iteration}_cell_{target_row}_{target_col}_BLOCKED.png") if mission_folder else None
            visualize_grid_with_candidates(
                pts=pts_up, terrain_real=terrain_real_up, obstacle_mask=obstacle_mask_updated,
                robot_x=robot_x, robot_y=robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=frontier['stop_xy'], iteration=iteration, env=env,
                save_path=blocked_save_path, prm_graph=prm_graph, chosen_path=full_path_coords,
                cells_obstacle_dist=cells_obs_up, valid_values=valid_up,
                grad_values=grad_up, rough_values=rough_up
            )
            return False

        # --- decision['kind'] == 'move' ----------------------------------------------------
        move_x, move_y = decision['target']
        is_partial_step = not decision['full']

        # --- Controllo preventivo sulla posa d'arrivo --------------------------------------
        # Solo per i passi corti o verso spazi stretti: sono quelli con cui si entra negli
        # angoli senza uscita. Vedi arrival_has_exit.
        step_len = float(np.hypot(move_x - robot_x, move_y - robot_y))
        r_a, c_a, in_a = arcVerification._cells(snap, np.array([move_x]), np.array([move_y]))
        od_arrival = float(snap.obstacle_dist[r_a[0], c_a[0]]) if in_a[0] else float('inf')
        if (arrival_refusals < MAX_ARRIVAL_REFUSALS
                and (step_len <= ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_arrival <= ARRIVAL_EXIT_CHECK_OD_M)):
            heading_arr = float(np.arctan2(move_y - robot_y, move_x - robot_x))
            has_exit, fan_arr = arrival_has_exit(snap, (move_x, move_y), heading_arr)
            if not has_exit:
                arrival_refusals += 1
                MISSION_STATS['arrival_refusals'] += 1
                n_us_a, fan_txt_a = describe_escape_fan(fan_arr)
                print(f"[ARRIVO] Il passo di {step_len:.2f} m verso ({move_x:.2f}, {move_y:.2f}) "
                      f"(obstacle_distance li' {od_arrival:.2f} m) mi porterebbe dove non ci sono "
                      f"uscite: {n_us_a}/{len(fan_arr)} direzioni utilizzabili ({fan_txt_a}). "
                      f"Non ci vado ({arrival_refusals}/{MAX_ARRIVAL_REFUSALS}), scarto il tratto "
                      f"{robot_node_id}->{next_node_id} e ripianifico.")
                mark_edge_blocked_soft(prm_graph, robot_node_id, next_node_id)
                new_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)
                if new_ids is not None and len(new_ids) > 1:
                    path_waypoints = _waypoints_from_ids(new_ids)
                    full_path_coords = [prm_graph.get_node_position(nid) for nid in new_ids]
                    no_progress = 0
                    continue
                # Sapere in anticipo che il punto d'arrivo e' un vicolo cieco e andarci lo
                # stesso non ha senso: si rinuncia a questo lato della cella, che verra'
                # ritentata piu' avanti da un altro (finalize_or_defer_blocked_cell).
                print(f"[ARRIVO] Nessuna alternativa nel grafo e il punto d'arrivo non ha "
                      f"uscite: non ci vado. Rinuncio a entrare in ({target_row},{target_col}) "
                      f"da questo lato.")
                if verification_tracker is not None:
                    verification_tracker.clear_path()
                return False

        # --- Esecuzione: una o piu' GAMBE sulla polilinea gia' confermata dal fronte,
        # senza scansione, refresh ne' ripianificazione in mezzo (vedi AVANZAMENTO
        # MULTINODO in decide_next_move). n_legs = 0 e' un solo passo parziale.
        n_legs = int(decision.get('nodes', 1)) if decision['full'] else 0
        if n_legs > 1:
            MISSION_STATS['multinode_moves'] += 1
            MISSION_STATS['multinode_nodes'] += n_legs
            print(f"[MULTINODO] Il fronte conferma "
                  f"{arcVerification.allowed_advance(frontier):.2f} m: percorro {n_legs} "
                  f"tratti di fila {[w[0] for w in path_waypoints[:n_legs]]} senza fermarmi "
                  f"a riscansionare.")
        leg_exit = None
        for leg_i in range(max(1, n_legs)):
            if not path_waypoints:
                break
            next_node_id, next_x, next_y = path_waypoints[0]
            if n_legs == 0:
                move_x, move_y = decision['target']
                is_partial_step = True
            else:
                move_x, move_y = next_x, next_y
                is_partial_step = False
            if leg_i > 0:
                # Posa VERA dopo la gamba precedente. Serve a tre cose, tutte necessarie:
                #  - navigate_to calcola l'angolo di rotazione dall'orientamento che riceve,
                #    e relative_move ruota RISPETTO ALLA POSA ATTUALE: con l'orientamento
                #    della scansione iniziale (prima della gamba 0) il robot ruoterebbe
                #    dell'angolo sbagliato e camminerebbe fuori dal percorso verificato;
                #  - la distanza da percorrere si calcola da (robot_x, robot_y);
                #  - l'arresto a meta' movimento misura il percorso rimanente da qui.
                robot_x, robot_y, robot_z, robot_quat = spotUtils.getPosition(robot_state_client)
                robot_yaw = np.arctan2(
                    2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                    1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))
                fresh_vision_tform_body = SimpleNamespace(
                    position=SimpleNamespace(x=robot_x, y=robot_y, z=robot_z), rotation=robot_quat)

                # Al nodo intermedio il robot si FERMA e RUOTA verso il nodo successivo: la
                # decisione iniziale ha controllato solo la rotazione della prima gamba.
                # Qui si controlla questa, sulla fotografia della scansione (in coordinate
                # mondo, quindi valida anche dopo lo spostamento) -- ma solo se TUTTO il
                # cerchio spazzato sta dentro i dati: rotation_is_clear ignora le celle
                # fuori finestra (ottimista), e con un percorso curvo un nodo intermedio
                # puo' stare vicino al bordo. Se non si puo' ruotare, o non si puo' dire,
                # ci si ferma e si riguarda: la rotazione la gestisce il ciclo, con cono,
                # arretramento e ventaglio, sui dati freschi.
                r_h, c_h, in_h = arcVerification._cells(snap, np.array([robot_x]), np.array([robot_y]))
                od_here = float(snap.obstacle_dist[r_h[0], c_h[0]]) if in_h[0] else 0.0
                rot_leg = rotation_plan(robot_yaw, (robot_x, robot_y), (move_x, move_y),
                                        od_here, snap=snap)
                reach_rot = float(spotGrid.ROTATION_CLEARANCE_M)
                ny_s, nx_s = snap.obstacle_dist.shape
                inside_rot = (snap.origin_x <= robot_x - reach_rot and
                              robot_x + reach_rot <= snap.origin_x + nx_s * snap.cell_size and
                              snap.origin_y <= robot_y - reach_rot and
                              robot_y + reach_rot <= snap.origin_y + ny_s * snap.cell_size)
                if not rot_leg['turn'] or (rot_leg['why'] == 'settore spazzato' and not inside_rot):
                    motivo = ("il corpo toccherebbe" if not rot_leg['turn']
                              else "il cerchio di rotazione esce dai dati della scansione")
                    print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: per puntare il nodo "
                          f"{next_node_id} servono {rot_leg['dyaw_deg']:+.0f} gradi e {motivo}. "
                          f"Mi fermo qui e riguardo con una scansione fresca.")
                    MISSION_STATS['multinode_cut'] += 1
                    leg_exit = 'rotazione'
                    break

                # Stessa prudenza della prima gamba sulla posa d'arrivo (arrival_has_exit):
                # non fermarsi in un punto senza uscite.
                step_leg = float(np.hypot(move_x - robot_x, move_y - robot_y))
                r_l, c_l, in_l = arcVerification._cells(snap, np.array([move_x]), np.array([move_y]))
                od_leg = float(snap.obstacle_dist[r_l[0], c_l[0]]) if in_l[0] else float('inf')
                if step_leg <= ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_leg <= ARRIVAL_EXIT_CHECK_OD_M:
                    has_exit_leg, _fan_leg = arrival_has_exit(
                        snap, (move_x, move_y), float(np.arctan2(move_y - robot_y, move_x - robot_x)))
                    if not has_exit_leg:
                        print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: il nodo {next_node_id} "
                              f"(obstacle_distance {od_leg:.2f} m) non avrebbe uscite. Mi fermo "
                              f"qui e riguardo con una scansione fresca.")
                        MISSION_STATS['multinode_cut'] += 1
                        leg_exit = 'arrivo'
                        break

                if verification_tracker is not None:
                    path_version = verification_tracker.update_path(
                        [(x, y) for _, x, y in path_waypoints])
            no_progress = 0
            retreats_in_a_row = 0   # un avanzamento vero rompe la catena di arretramenti
            if is_partial_step:
                print(f"[PASSO PARZIALE] Verso il nodo {next_node_id}: avanzo di {decision['dist']:.2f} m "
                      f"su {np.hypot(next_x - robot_x, next_y - robot_y):.2f} fino a "
                      f"({move_x:.2f}, {move_y:.2f}), poi riguardo con una scansione fresca.")
            else:
                print(f"[OK] Tratto {robot_node_id}->{next_node_id} confermato libero dal fronte. Eseguo movimento...")

            if leg_i == 0:
                vision_tform_body_current = get_a_tform_b(lg_proto_up.local_grid.transforms_snapshot,
                                                          VISION_FRAME_NAME, BODY_FRAME_NAME)
            else:
                vision_tform_body_current = fresh_vision_tform_body   # vedi sopra: posa VERA

            # --- Misura della difficolta' di QUESTO segmento (per il gait e per il CSV).
            segment_long_slope, segment_lat_slope, segment_roughness = prm_graph.sample_terrain_between_points(
                robot_x, robot_y, move_x, move_y,
                node_idx1=robot_node_id, node_idx2=next_node_id
            )
            slope_source = prm_graph.last_slope_source
            print(f"[SLOPE-SOURCE] Arco {robot_node_id}->{next_node_id}: "
                  f"{str(slope_source).upper()} "
                  f"(long={segment_long_slope:.3f}, lat={segment_lat_slope:.3f})")

            seg_diag = _segment_diagnostics(terrain_up_2d, grid_origin_x_up, grid_origin_y_up, cell_size_up,
                                            robot_x, robot_y, move_x, move_y)
            if seg_diag['long_interp'] is not None:
                print(f"[SLOPE-CHECK] Arco {robot_node_id}->{next_node_id}: in uso long={segment_long_slope:.3f} "
                      f"lat={segment_lat_slope:.3f} | interpolata long={seg_diag['long_interp']:.3f} "
                      f"lat={seg_diag['lat_interp']:.3f} | campioni dentro griglia: "
                      f"lungo arco {seg_diag['coverage_along']:.0%}, laterali {seg_diag['coverage_lateral']:.0%}")
            else:
                print(f"[SLOPE-CHECK] Arco {robot_node_id}->{next_node_id}: nessun dato di terreno live "
                      f"per la stima interpolata (fonte pendenza: {str(slope_source).upper()})")

            near_thresh = spotGrid.SLOPE_THRESHOLD * (1.0 - spotGrid.NEAR_THRESHOLD_MARGIN_FRACTION)
            if (segment_long_slope >= near_thresh or segment_lat_slope >= near_thresh) and mission_folder is not None:
                try:
                    npz_near = os.path.join(
                        mission_folder,
                        f"iteration_{iteration}_cell_{target_row}_{target_col}_"
                        f"arc{robot_node_id}-{next_node_id}_NEARLIMIT.npz")
                    np.savez_compressed(
                        npz_near,
                        obstacle_grid=np.asarray(obstacle_mask_updated).reshape((num_y_up, num_x_up)),
                        terrain_grid=np.asarray(terrain_real_up).reshape((num_y_up, num_x_up)),
                        grid_origin_x=grid_origin_x_up, grid_origin_y=grid_origin_y_up, cell_size=cell_size_up,
                        arc_xy=np.array([robot_x, robot_y, move_x, move_y]),
                        long_slope=segment_long_slope, lat_slope=segment_lat_slope,
                        long_interp=np.nan if seg_diag['long_interp'] is None else seg_diag['long_interp'],
                        lat_interp=np.nan if seg_diag['lat_interp'] is None else seg_diag['lat_interp'],
                        slope_threshold=spotGrid.SLOPE_THRESHOLD)
                    MISSION_STATS['near_threshold_saves'] += 1
                    margine = spotGrid.SLOPE_THRESHOLD - max(segment_long_slope, segment_lat_slope)
                    print(f"[NEAR-LIMIT] Arco {robot_node_id}->{next_node_id} vicino soglia "
                          f"(long={segment_long_slope:.3f}, lat={segment_lat_slope:.3f}, "
                          f"soglia={spotGrid.SLOPE_THRESHOLD:.3f}, margine residuo={margine:+.3f}) "
                          f"-- dati salvati in {os.path.basename(npz_near)}")
                except Exception as e:
                    print(f"[DATA SAVE] Impossibile salvare il file vicino soglia: {e}")

            if mobility_kwargs:
                segment_mobility_kwargs = mobility_kwargs
                segment_tier = 'override'
            else:
                segment_gradient = max(segment_long_slope, segment_lat_slope)
                segment_mobility_kwargs = select_mobility_params(segment_gradient, segment_roughness)
                segment_tier = MISSION_STATS['last_tier']

            # --- Condizione di arresto a meta' movimento ---------------------------------
            abort_state = {'reason': None, 'bad': 0}
            move_t0 = time.time()
            live_frontiers = []

            def _should_abort_move():
                if verification_tracker is None:
                    return False
                f = verification_tracker.get_frontier()
                if f is None or f.get('path_version') != path_version or f['time'] <= move_t0:
                    return False
                frx, fry = f['robot_xy']
                remaining = float(np.hypot(move_x - frx, move_y - fry))
                reach = arcVerification.allowed_advance(f)
                if not live_frontiers or live_frontiers[-1][0] != f['time']:
                    live_frontiers.append((f['time'], frx, fry, f['reason'], float(f['dist']), reach, remaining,
                                           abort_state['bad']))
                if (f['reason'] == 'fine_dati'
                        and float(np.hypot(frx - robot_x, fry - robot_y)) < ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                    abort_state['bad'] = 0      # sta ancora ruotando sul posto: vedi la costante
                    return False
                if reach + FRONTIER_ABORT_TOLERANCE_M < remaining:
                    abort_state['bad'] += 1
                    if abort_state['bad'] >= FRONTIER_ABORT_CONFIRM:
                        sxa, sya = f['stop_xy']
                        abort_state['reason'] = (f"il fronte si e' accorciato ({f['reason']} in "
                                                 f"({sxa:.2f}, {sya:.2f})): raggiungibili {reach:.2f} m "
                                                 f"su {remaining:.2f} m mancanti")
                        return True
                else:
                    abort_state['bad'] = 0
                return False

            # --- Esecuzione con registrazione dell'inclinazione reale del corpo ---
            tilt_recorder = BodyTiltRecorder(robot_state_client)
            tilt_recorder.start()
            try:
                need_refresh = True              # fine di un movimento: vedi REFRESH A EVENTI
                success_move, distance_traveled, distance_commanded = navigate_to(
                    move_x, move_y, robot_x, robot_y, robot_state_client, command_client,
                    vision_tform_body_current, should_abort=_should_abort_move,
                    keep_heading=keep_heading,
                    **segment_mobility_kwargs
                )
            finally:
                tilt_stats = tilt_recorder.stop()
            move_duration = time.time() - move_t0
            _timing_add('movimento', move_duration)
            try:   # solo log: non deve mai interrompere la missione
                _seg_id = [iteration, target_row, target_col, loop_iter, robot_node_id, next_node_id]
                _append_csv("trajectory.csv",
                            ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                             'x', 'y', 'yaw_deg', 'roll_deg', 'pitch_deg', 'target_x', 'target_y'],
                            [[t] + _seg_id + [x, y, yw, r, pt, move_x, move_y]
                             for (t, x, y, yw, r, pt) in getattr(tilt_recorder, 'samples', [])])
                _append_csv("frontier_live.csv",
                            ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                             'robot_x', 'robot_y', 'reason', 'frontier_dist', 'reachable_m', 'remaining_m',
                             'consecutive_bad', 'aborted'],
                            [[lf[0]] + _seg_id + list(lf[1:]) + [abort_state['reason'] is not None]
                             for lf in live_frontiers])
            except Exception as e:
                print(f"[LOG] Traiettoria/fronti del movimento non registrati: {e}")

            # --- Riga del CSV per questo segmento (anche se il movimento e' fallito) ---
            try:
                seg_dist = float(np.hypot(move_x - robot_x, move_y - robot_y))
                cost_dist = prm_graph.alpha * seg_dist
                cost_long = prm_graph.beta * segment_long_slope
                cost_lat = prm_graph.beta_lateral * segment_lat_slope
                cost_rough = prm_graph.gamma * segment_roughness
                _log_segment({
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'iteration': iteration, 'cell_row': target_row, 'cell_col': target_col,
                    'node_from': robot_node_id, 'node_to': next_node_id,
                    'x_from': robot_x, 'y_from': robot_y, 'x_to': move_x, 'y_to': move_y,
                    'dist_m': seg_dist,
                    'slope_source': slope_source,
                    'long_slope': segment_long_slope, 'lat_slope': segment_lat_slope,
                    'long_interp': seg_diag['long_interp'], 'lat_interp': seg_diag['lat_interp'],
                    'roughness': segment_roughness,
                    'coverage_along': seg_diag['coverage_along'], 'coverage_lateral': seg_diag['coverage_lateral'],
                    'cost_dist': cost_dist, 'cost_long': cost_long, 'cost_lat': cost_lat,
                    'cost_rough': cost_rough, 'cost_total': cost_dist + cost_long + cost_lat + cost_rough,
                    'tier': segment_tier, 'gait_override': bool(mobility_kwargs),
                    'success': bool(success_move),
                    'dist_traveled_m': distance_traveled, 'dist_commanded_m': distance_commanded,
                    'duration_s': move_duration,
                    'roll_start_deg': tilt_stats['roll_start'], 'pitch_start_deg': tilt_stats['pitch_start'],
                    'roll_end_deg': tilt_stats['roll_end'], 'pitch_end_deg': tilt_stats['pitch_end'],
                    'roll_peak_abs_deg': tilt_stats['roll_peak_abs'], 'pitch_peak_abs_deg': tilt_stats['pitch_peak_abs'],
                    'tilt_samples': tilt_stats['samples'], 'tilt_errors': tilt_stats['errors'],
                })
                if tilt_stats['samples']:
                    print(f"[TILT] Arco {robot_node_id}->{next_node_id}: picco roll={tilt_stats['roll_peak_abs']:.1f} deg, "
                          f"pitch={tilt_stats['pitch_peak_abs']:.1f} deg ({tilt_stats['samples']} campioni) | "
                          f"stimato long={segment_long_slope:.3f} (~{np.degrees(np.arctan(segment_long_slope)):.1f} deg), "
                          f"lat={segment_lat_slope:.3f} (~{np.degrees(np.arctan(segment_lat_slope)):.1f} deg) | "
                          f"tier={segment_tier}, {move_duration:.1f}s")
            except Exception as e:
                print(f"[LOG] Impossibile registrare il segmento: {e}")

            if success_move:
                print(f"[INFO] Spostamento completato con successo su ({move_x:.2f}, {move_y:.2f})")

                if not hasattr(env, '_traveled_arcs'):
                    env._traveled_arcs = []
                env._traveled_arcs.append((robot_x, robot_y, move_x, move_y))

                if is_partial_step:
                    ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
                    robot_node_id = prm_graph.add_stop_node(
                        ax, ay, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="sosta (passo parziale)")
                    prm_graph.set_robot_node(robot_node_id, 0.0)
                else:
                    # Arco "percorso" solo se il robot e' partito dal nodo: altrimenti il tratto
                    # camminato non e' l'arco del grafo (vedi add_stop_node, nessun nodo nuovo).
                    if _robot_on_node(prm_graph, robot_node_id, robot_x, robot_y):
                        prm_graph.traversed_edges.add((min(robot_node_id, next_node_id),
                                                       max(robot_node_id, next_node_id)))
                    robot_node_id = next_node_id
                    prm_graph.set_robot_node(robot_node_id, 0.0)
                    path_waypoints.pop(0)

                # Briciola per i ritorni: un waypoint GraphNav ogni
                # GRAPHNAV_WAYPOINT_MIN_SPACING_M di strada percorsa.
                graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client,
                                         robot_node_id, target_row, target_col)

            elif abort_state['reason'] is not None:
                print(f"[STOP] Movimento verso ({move_x:.2f}, {move_y:.2f}) interrotto dopo "
                      f"{distance_traveled:.2f}m: {abort_state['reason']}.")
                MISSION_STATS['frontier_aborts'] += 1
                if distance_traveled > 0.05:
                    if not hasattr(env, '_traveled_arcs'):
                        env._traveled_arcs = []
                    ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
                    env._traveled_arcs.append((robot_x, robot_y, ax, ay))
                    robot_node_id = prm_graph.add_stop_node(
                        ax, ay, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="sosta (arresto)")
                    prm_graph.set_robot_node(robot_node_id, 0.0)
                leg_exit = 'arresto'
                break

            else:
                print(f"[FAIL] Comando di movimento fallito meccanicamente per ({move_x:.2f}, {move_y:.2f}) "
                      f"(percorsi {distance_traveled:.2f}m su {distance_commanded:.2f}m richiesti)")

                _check_and_recover_fall(robot_state_client, command_client, global_map,
                                        "dopo il fallimento del movimento")

                if distance_traveled > 0.05:
                    if not hasattr(env, '_traveled_arcs'):
                        env._traveled_arcs = []
                    ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
                    env._traveled_arcs.append((robot_x, robot_y, ax, ay))

                if _robot_on_node(prm_graph, robot_node_id, robot_x, robot_y):
                    prm_graph.mark_edge_invalid(robot_node_id, next_node_id)
                else:   # il robot non partiva dal nodo: il tratto fallito non e' l'arco del grafo
                    mark_edge_blocked_soft(prm_graph, robot_node_id, next_node_id)
                print(f"[INFO] Tratto {robot_node_id}-{next_node_id} scartato dopo fallimento fisico.")
                if verification_tracker is not None:
                    verification_tracker.clear_path()

                # Ritorno con GraphNav invece che a zampe all'indietro: si torna di un
                # waypoint, si riprova a pianificare da li', e cosi' via fino a
                # GRAPHNAV_MAX_BACKTRACK_HOPS. La ritirata a zampe resta dentro come ultima rete.
                esc = _trap_backtrack(
                    trap_state, (robot_node_id, next_node_id),
                    env, prm_graph, goal_id, recordingInterface, robot_state_client,
                    command_client, global_map, mobility_kwargs, pts_up, obstacle_mask_updated,
                    label=f"dal nodo {robot_node_id}")
                if esc['ok']:
                    path_waypoints = _waypoints_from_ids(esc['ids'])
                    full_path_coords = [prm_graph.get_node_position(nid) for nid in esc['ids']]
                    robot_node_id = esc['robot_node']
                    no_progress = 0
                    early_replans = 0
                    retreats_in_a_row = 0
                    rotation_discards = 0
                    best_goal_dist = float('inf')
                    leg_exit = 'ripianificato'
                    break

                fail_save_path = os.path.join(mission_folder,
                                              f"iteration_{iteration}_cell_{target_row}_{target_col}_MOVE_FAIL.png") if mission_folder else None
                visualize_grid_with_candidates(
                    pts=pts_up, terrain_real=terrain_real_up, obstacle_mask=obstacle_mask_updated,
                    robot_x=robot_x, robot_y=robot_y,
                    candidates={'rejected': [], 'valid': []},
                    chosen_point=(next_x, next_y), iteration=iteration, env=env,
                    save_path=fail_save_path, prm_graph=prm_graph, chosen_path=full_path_coords,
                    cells_obstacle_dist=cells_obs_up, valid_values=valid_up,
                    grad_values=grad_up, rough_values=rough_up
                )
                return False
        # Gambe finite (o interrotte): si torna in cima con una scansione fresca.
        continue

    if verification_tracker is not None:
        verification_tracker.clear_path()

    # Controllo finale della posizione a fine percorso
    robot_x, robot_y, _, _ = spotUtils.getPosition(robot_state_client)
    return env.is_point_in_cell(robot_x, robot_y, target_row, target_col)

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
        0.59 m di obstacle_distance invece di 0.30 (vedi ROTAZIONE E ARRETRAMENTO). Il
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
        move_success, distance_traveled = movements.move_to_world_point_without_turning(
            target_x, target_y, "vision", command_client, robot_state_client,
            should_abort=should_abort, **mobility_kwargs)
    else:
        print("[INFO] Passo 1: rotazione verso il punto...")
        rotate_success, _ = movements.relative_move(0, 0, dyaw, "vision", command_client, robot_state_client,
                                                    should_abort=should_abort, **mobility_kwargs)
        if not rotate_success:
            print(f"[FAIL] Rotazione non riuscita: il movimento in avanti non parte.")
            return False, 0.0, distance

        print(f"[INFO] Passo 2: avanti di {distance:.2f} m...")
        move_success, distance_traveled = movements.relative_move(distance, 0, 0, "vision", command_client,
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



def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells


def retreat_along_traveled_path(env, robot_state_client, command_client,
                                max_segments=RETREAT_MAX_SEGMENTS,
                                mobility_kwargs=None,
                                pts=None, obstacle_mask=None):
    """
    Ritirata: torna indietro sui propri passi camminando ALL'INDIETRO, senza mai girarsi.
    E' l'ultima rete, quando il ritorno con GraphNav non riesce (vedi escape_by_backtracking).

    Quando il robot si infila in uno spazio stretto e li' non esiste piu' un percorso
    verso l'obiettivo, girarsi non e' una via d'uscita --
    serve spazio che in quel punto non c'e'. Arretrare invece ripercorre il varco da cui
    si e' entrati, che per costruzione era abbastanza largo da passarci. E arretrare lungo
    l'asse del corpo non e' marcia di traverso: l'ingombro resta 0.50 m di larghezza.

    I punti da cui si e' passati sono gia' registrati in env._traveled_arcs, riempito a
    ogni spostamento completato. Si ripercorrono a ritroso, uno per uno, mantenendo
    l'orientamento attuale (vedi movements.move_to_world_point_without_turning).

    Se vengono passati `pts` e `obstacle_mask` della scansione corrente, ogni tratto
    viene prima controllato sui dati freschi: il fatto di esserci passati poco fa e' un
    ottimo indizio, non una garanzia, e non ha senso arretrare dentro qualcosa che nel
    frattempo si vede occupato. In quel caso la ritirata si ferma li' e lo dichiara.

    Returns:
        (segmenti_percorsi: int, distanza_totale: float)
    """
    mobility_kwargs = mobility_kwargs or {}
    traveled = getattr(env, '_traveled_arcs', None)
    if not traveled:
        print("[RITIRATA] Nessun tratto percorso registrato: non so da dove sono arrivato, "
              "non arretro.")
        return 0, 0.0

    done, total = 0, 0.0
    for _ in range(max_segments):
        if not traveled:
            print("[RITIRATA] Esauriti i tratti percorsi registrati.")
            break

        from_x, from_y, to_x, to_y = traveled[-1]
        robot_x, robot_y, _, _ = spotUtils.getPosition(robot_state_client)
        seg_len = float(np.hypot(from_x - robot_x, from_y - robot_y))

        gap = float(np.hypot(to_x - robot_x, to_y - robot_y))
        if gap > RETREAT_CONTINUITY_M:
            print(f"[RITIRATA] Non sono dove finiva l'ultimo tratto registrato (distanza {gap:.2f} m, "
                  f"es. dopo uno spostamento con GraphNav): non arretro alla cieca.")
            del traveled[:]
            break
        if seg_len > RETREAT_MAX_SEGMENT_M:
            print(f"[RITIRATA] Il tratto di rientro e' lungo {seg_len:.2f} m (oltre "
                  f"{RETREAT_MAX_SEGMENT_M:.1f} m): non lo ripercorro all'indietro.")
            break

        if seg_len < RETREAT_MIN_SEGMENT_M:
            # Tratto troppo corto per valere un comando a se': lo consumiamo e proseguiamo.
            traveled.pop()
            continue

        if pts is not None and obstacle_mask is not None:
            try:
                if arcVerification.is_arc_in_fov(robot_x, robot_y, from_x, from_y, pts):
                    status = arcVerification.verify_arc_safety(robot_x, robot_y, from_x, from_y,
                                                               pts, obstacle_mask)
                    if status == 'blocked':
                        print(f"[RITIRATA] Il tratto di rientro verso ({from_x:.2f}, {from_y:.2f}) "
                              f"risulta ostruito nella scansione corrente. Mi fermo qui.")
                        break
            except Exception as e:
                print(f"[RITIRATA] Controllo del tratto di rientro non riuscito ({e}); "
                      f"procedo comunque, e' la via da cui sono venuto.")

        print(f"[RITIRATA] Tratto {done + 1}/{max_segments}: rientro verso "
              f"({from_x:.2f}, {from_y:.2f}), {seg_len:.2f} m all'indietro.")
        success, dist = movements.move_to_world_point_without_turning(
            from_x, from_y, "vision", command_client, robot_state_client, **mobility_kwargs)
        total += dist
        traveled.pop()

        if not success:
            print(f"[RITIRATA] Il rientro si e' fermato dopo {dist:.2f} m. "
                  f"Interrompo la ritirata.")
            break
        done += 1

    if done:
        print(f"[RITIRATA] Completata: {done} tratti, {total:.2f} m percorsi all'indietro.")
    else:
        print("[RITIRATA] Nessun tratto percorso.")
    return done, total


def cell_has_reachable_target(env, sampler, global_map, row, col, safety_margin=None):
    """
    Nella cella c'e' ANCORA un punto dove il robot potrebbe stare, secondo la
    mappa globale accumulata finora? Restituisce (si_no, liberi, ignoti, occupati).

    Serve a non ritentare una cella il cui obiettivo e' fisicamente dentro un ostacolo
    (per esempio quando la griglia di missione non sta nello spazio):
    finalize_or_defer_blocked_cell marca bloccata una cella solo dopo averne provati tutti
    i lati, ma se l'obiettivo e' in un muro i lati non c'entrano niente.

    Si guarda solo cio' che e' stato VISTO: ignoto non e' occupato (e' la stessa regola del
    fronte, che distingue 'fine_dati' da 'ostacolo'). Quindi una cella mai vista risponde
    sempre si', e questo controllo non puo' impedire una prima esplorazione.
    """
    if global_map is None:
        return True, 0, 0, 0
    if safety_margin is None:
        safety_margin = spotGrid.PRM_EDGE_SAFETY_MARGIN_M
    centro = env.get_world_position_from_cell(row, col)
    if centro is None:
        return False, 0, 0, 0
    punti = [centro]
    try:
        punti += list(sampler.get_point_in_cell(row, col))
    except Exception:
        pass
    liberi = ignoti = occupati = 0
    for (px, py) in punti:
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            occupati += 1
        elif global_map.is_known(px, py):
            liberi += 1
        else:
            ignoti += 1
    return (liberi + ignoti) > 0, liberi, ignoti, occupati


def finalize_or_defer_blocked_cell(env, row, col, sampler=None, global_map=None):
    """
    Da chiamare subito dopo env.mark_cell_side_explored(row, col, side_bit), quando un
    tentativo di entrare in (row, col) e' appena fallito da un lato.

    La cella si marca bloccata per sempre (env.mark_cell_blocked -> valore -1) solo quando
    OGNI lato che ha un vicino dentro la griglia e' stato provato ed e' fallito
    (env.all_sides_explored). Altrimenti resta al valore di non visitata (0) e NON si
    ritenta subito: resta una candidata normale della frontiera.
    get_adjacent_frontier_cells e get_lowest_rank_from_frontier_list trattano ogni cella a
    0 allo stesso modo, ordinata per la sua posizione nella serpentina, come una cella mai
    tentata. Cosi' una cella fallita una volta compete per la priorita' con la STESSA
    logica di tutte le altre, e si ritenta quando tocca a lei, dal lato non ancora provato:
    ne' subito fuori turno, ne' scartata dopo un solo lato.
    """
    # Se nella cella non esiste piu' un punto dove il robot possa stare, provare da un
    # altro lato e' inutile -- l'obiettivo e' dentro un ostacolo, non dietro un varco
    # che si apre da un'altra parte. Vedi cell_has_reachable_target.
    if sampler is not None and global_map is not None:
        ok, liberi, ignoti, occupati = cell_has_reachable_target(env, sampler, global_map, row, col)
        # Diagnostica: dice subito se la cella sta dentro un ostacolo o non e' mai stata
        # vista, cioe' se la griglia di missione non sta nello spazio.
        # ATTENZIONE a come si legge: con il reticolo a 0.25 m i punti candidati per cella
        # sono centinaia, quindi "occupati" alto NON vuol dire cella irraggiungibile; vuol
        # dire che gran parte della cella e' dentro un ostacolo. Il blocco immediato sotto
        # scatta solo nel caso estremo in cui NON resta un solo punto libero o ignoto.
        print(f"[CELLA] ({row},{col}) dopo il tentativo fallito: dei punti candidati, "
              f"{liberi} visti liberi, {ignoti} mai visti, {occupati} dentro un ostacolo "
              f"nella mappa globale.")
        if not ok:
            env.mark_cell_blocked(row, col)
            print(f"[BLOCKED] Cella ({row},{col}) bloccata subito: tutti i {occupati} punti "
                  f"candidati risultano dentro un ostacolo nella mappa globale (nessuno libero, "
                  f"nessuno ignoto). Non c'e' un lato da cui entrare: l'obiettivo e' nel muro.")
            return
    if env.all_sides_explored(row, col):
        env.mark_cell_blocked(row, col)
        print(f"[BLOCKED] Cella ({row},{col}) bloccata per sempre: ogni lato raggiungibile "
              f"e' stato provato senza successo.")
    else:
        print(f"[DEFER] Cella ({row},{col}) non ancora provata da tutti i lati (lati provati: "
              f"{bin(env.get_cell_sides_status(row, col))}): resta candidata della "
              f"frontiera e si ritentera' piu' avanti, nell'ordine normale, "
              f"da un lato non ancora provato.")


def easy_walk(options):
    robot, lease_client, robot_state_client, client_metadata = spotLogInUtils.setLogInfo(options)
    estop = spotLogInUtils.SimpleEstop(robot, options.name + "_estop")

    local_grid = spotGrid.LocalGrid(robot)
    _MISSION_CTX['local_grid'] = local_grid
    global_grid = spotGrid.GlobalGrid()
    _MISSION_CTX['global_grid'] = global_grid   # per graphnav_add_shortcuts

    recordingInterface = navGraphUtils.RecordingInterface(robot, options.download_filepath, client_metadata)
    recordingInterface.stop_recording()
    recordingInterface.clear_map()

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # --- Cartelle di missione e log, create subito, cosi' la copia dell'output su file
        # (mission_log.txt) cattura anche tutto l'avvio e la costruzione del grafo.
        mission_timestamp = datetime.now().strftime("Mission_%d-%m-%Y_%H-%M-%S")
        base_graph_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "graph")
        graph_folder = os.path.join(base_graph_folder, mission_timestamp)
        mission_map_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionMap", mission_timestamp)
        mission_log_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionLogs", mission_timestamp)
        os.makedirs(graph_folder, exist_ok=True)
        os.makedirs(mission_map_folder, exist_ok=True)
        os.makedirs(mission_log_folder, exist_ok=True)
        mission_folder = mission_map_folder
        mission_log_path = os.path.join(mission_log_folder, "mission_log.txt")
        _install_stdout_tee(mission_log_path)
        try:
            _MISSION_CTX['log_folder'] = mission_log_folder
            _MISSION_CTX['segment_log'] = SegmentLog(os.path.join(mission_log_folder, "segments.csv"))
            _MISSION_CTX['summary_done'] = False
            print(f"[LOG] Cartella log: {mission_log_folder}  |  dati e immagini: {mission_map_folder}")
            print("[CONFIG] INTERRUTTORI: " + "  ".join(
                f"{k.split('.')[-1]}={v}" for k, v in mission_switches().items()))
            print(f"[CONFIG] SLOPE_THRESHOLD={spotGrid.SLOPE_THRESHOLD}  "
                  f"LATERAL_SLOPE_COST_MULTIPLIER={spotGrid.LATERAL_SLOPE_COST_MULTIPLIER}  "
                  f"OBSTACLE_THRESHOLD={spotGrid.OBSTACLE_THRESHOLD}  ROUGH_THRESHOLD={spotGrid.ROUGH_THRESHOLD}  "
                  f"NEAR_THRESHOLD_MARGIN_FRACTION={spotGrid.NEAR_THRESHOLD_MARGIN_FRACTION}")
            print(f"[CONFIG] PROFILO ARCO: base={spotGrid.SLOPE_BASELINE_M}m  "
                  f"fetta_laterale=+/-{spotGrid.LATERAL_SLICE_HALF_WIDTH_M}m  "
                  f"copertura_minima={spotGrid.MIN_ARC_SAMPLE_FRACTION:.0%}")
            print(f"[CONFIG] GAIT: plain<{spotGrid.GAIT_PLAIN_SLOPE_MAX}/{spotGrid.GAIT_PLAIN_ROUGH_MAX}  "
                  f"moderate<{spotGrid.GAIT_MODERATE_SLOPE_MAX}/{spotGrid.GAIT_MODERATE_ROUGH_MAX}  "
                  f"mu={spotGrid.GAIT_MU_PLAIN}/{spotGrid.GAIT_MU_MODERATE}/{spotGrid.GAIT_MU_HARD}")
            # Soglie di terreno, rotazione, ritorni e obiettivo, stampate per poter leggere i
            # log sapendo con quali valori sono stati prodotti.
            print(f"[CONFIG] TERRENO: terrain_valid >= {spotGrid.TERRAIN_VALID_MIN} "
                  f"(era > 0.0, dipendeva dal segno di un residuo numerico)  "
                  f"celle mai scritte: plateau >= {spotGrid.UNWRITTEN_MIN_CELLS} celle, "
                  f"almeno {spotGrid.UNWRITTEN_MIN_SPOT_INVALID:.0%} marcate non valide da Spot")
            print(f"[CONFIG] ROTAZIONE: prefiltro {spotGrid.ROTATION_CLEARANCE_M:.3f} m "
                  f"(aria {spotGrid.ROTATION_CLEARANCE_AIR_M:.2f} m, separata dai "
                  f"{spotGrid.ROBOT_CLEARANCE_AIR_M:.2f} m dell'avanzamento), poi controllo sul "
                  f"settore spazzato a passi di {spotGrid.ROTATION_SWEEP_STEP_DEG:.0f} gradi "
                  f"(tolleranza {spotGrid.ROTATION_NOISE_TOLERANCE_M:.2f} m sul rumore); "
                  f"MARCIA DI TRAVERSO DISATTIVATA")
            print(f"[CONFIG] SE NON PUO' RUOTARE: 1) ventaglio nel cono +/-{CONE_SWEEP_MAX_DEG:.0f} gradi "
                  f"a passi di {CONE_SWEEP_STEP_DEG:.0f}, si va nella direzione piu' libera fra quelle "
                  f"che avvicinano di almeno {CONE_MIN_GAIN_M:.2f} m; 2) arretramento dritto fino a "
                  f"{ROTATION_RETREAT_M:.2f} m SOLO se da li' la rotazione diventa possibile, "
                  f"max {MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi; 3) si scarta il tratto")
            print(f"[CONFIG] TRAPPOLA: ventaglio a 360 gradi a passi di {ESCAPE_FAN_STEP_DEG:.0f} "
                  f"({int(360/ESCAPE_FAN_STEP_DEG)} direzioni); una direzione vale se concede "
                  f"{ESCAPE_FAN_MIN_ADVANCE_M:.2f} m E il robot puo' girarsi per puntarla. Zero "
                  f"direzioni -> si ripercorre il tratto da cui si e' venuti SUBITO, senza "
                  f"scartare archi. Controllo sulla posa d'arrivo per i passi sotto "
                  f"{ARRIVAL_EXIT_CHECK_MAX_STEP_M:.2f} m o con obstacle_distance sotto "
                  f"{ARRIVAL_EXIT_CHECK_OD_M:.2f} m, max {MAX_ARRIVAL_REFUSALS} rifiuti. "
                  f"Scarto da [SUBITO] e da [ARRIVO]: NON permanente")
            print(f"[CONFIG] RITORNI: briciola GraphNav ogni "
                  f"{GRAPHNAV_WAYPOINT_MIN_SPACING_M:.2f} m, ritorno con GraphNav fino a "
                  f"{GRAPHNAV_MAX_BACKTRACK_HOPS} waypoint indietro riprovando la "
                  f"pianificazione a ogni tappa, poi si rimanda la cella; ritirata a zampe "
                  f"solo come ultima rete. Nodi PRM scartati sotto "
                  f"{PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero, solo dove visti. "
                  f"Deroga di {prm_graph.STOP_NODE_MAP_IGNORE_M:.2f} m alla mappa globale "
                  f"attorno al robot: ora solo dopo una caduta")
            print(f"[CONFIG] OSTACOLI: mediana su {arcVerification.OBSTACLE_MEDIAN_FRAMES} scansioni "
                  f"+ isteresi {arcVerification.OBSTACLE_HYSTERESIS_M:.2f} m attorno a "
                  f"{spotGrid.FRONTIER_CLEARANCE_M:.2f} m (rumore misurato 1.5 cm tipici, 7.3 di punta); "
                  f"celle-ostacolo escluse dal campionamento della pendenza d'arco")
            print(f"[CONFIG] OBIETTIVO: tolleranza d'arrivo {GOAL_REACHED_TOLERANCE_M:.2f} m "
                  f"(= meta' corpo + margine del fronte), riscelta del punto fino a "
                  f"{MAX_BLOCK_REPLANS} volte se risulta irraggiungibile; ripianificazione alla "
                  f"PRIMA cella bloccante, fino a {MAX_EARLY_REPLANS} volte per tentativo")
        except Exception as e:
            print(f"[LOG] Impossibile inizializzare il CSV dei segmenti: {e}")

        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.logger.info('Accensione del robot...')
        robot.power_on()
        assert robot.is_powered_on(), 'Accensione del robot non riuscita.'
        robot.logger.info('Robot acceso.')
        blocking_stand(command_client)

        # Griglie e mappe locali vengono tutte dalle telecamere di Spot (nessun Velodyne).
        recordingInterface.clear_map()
        recordingInterface.start_recording()

        recordingInterface.initialize_with_fiducial(robot_state_client, 549)

        start_row, start_col = 0, 0
        recordingInterface.create_default_waypoint(cell_row=start_row, cell_col=start_col)

        # --- Griglia di missione
        # Dimensioni della griglia di missione. Default MISSION_GRID_ROWS x MISSION_GRID_COLS
        # celle da MISSION_CELL_SIZE_M (da mission_graph.MISSION_*). Si cambiano
        # SENZA toccare il codice, in ordine di precedenza:
        #   1. argomenti:  python3 easy_walk.py --rows 2 --cols 4 --cell 5
        #   2. variabile:  SPOT_MISSION_GRID="righe,colonne,lato_cella"   es. "2,3,3"
        grid_rows, grid_cols, grid_cell = MISSION_GRID_ROWS, MISSION_GRID_COLS, MISSION_CELL_SIZE_M
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
              f"{'suggerito' if MISSION_LATTICE_SPACING_M is None else '%.3f m' % MISSION_LATTICE_SPACING_M}, "
              f"anelli {MISSION_LATTICE_RINGS if MISSION_LATTICE_RINGS is not None else 'tutti'}")
        # test_replay legge metodo e archi da questa riga; distribuzione, densita' e seme
        # dalla riga [GRAFO] Nodi casuali, scritta quando il grafo e' costruito.
        print(f"[CONFIG] Grafo: nodi {MISSION_GRAPH_SAMPLING}"
              + (f" ({MISSION_RANDOM_DISTRIBUTION}, {MISSION_RANDOM_DENSITY:g} nodi/m2, seme "
                 f"{'a caso' if MISSION_RANDOM_SEED is None else MISSION_RANDOM_SEED})"
                 if MISSION_GRAPH_SAMPLING == 'casuale' else "")
              + f"; archi da {PRM_MIN_EDGE_M:g} a {PRM_MAX_EDGE_M:g} m")
        env = environmentMap.EnvironmentMap(rows=grid_rows, cols=grid_cols, cell_size=grid_cell)  # colonne davanti, righe a sinistra

        # --- Andatura per tutta la missione ---
        # Vuoto: andatura, altezza del passo e ground_mu_hint si scelgono TRATTO PER TRATTO
        # in base al terreno (vedi select_mobility_params()). Per imporre un profilo fisso
        # (per esempio in prova) basta un dict non vuoto qui:
        # attempt_enter_cell_from_position() lo usa per tutti i tratti e salta la scelta
        # automatica. Esempio:
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
        _set_mission_z0(robot_state_client, z_boot)

        # Nodi del PRM: il grafo della missione (vedi _build_mission_prm e mission_graph.py).
        # Il punto di accensione e i centri cella sono nodi del grafo, con entrambi i metodi.
        prm, gb_sampler, mission_lattice = _build_mission_prm(env)
        _MISSION_CTX['prm'] = prm  # per il riepilogo di fine missione (anche se interrotta)

        # Gli ARCHI non si creano qui: non c'e' ancora nessun dato di terreno ne' di
        # occupazione. Li crea attempt_enter_cell_from_position() alla prima scansione
        # (vedi _rebuild_prm_edges).
        verification_tracker = arcVerification.ArcVerificationTracker(robot)
        _MISSION_CTX['tracker'] = verification_tracker
        verification_tracker.start()

        recordingInterface.set_download_filepath(graph_folder)
        path = env.generate_serpentine_path(start_cell=env.start_cell)

        frontier = []
        visualization_counter = 0

        x, y, z, _ = spotUtils.getPosition(robot_state_client)
        robot_row, robot_col = env.get_cell_from_world(x, y)
        frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))

        while (True):
            print(frontier)
            x, y, z, _ = spotUtils.getPosition(robot_state_client)
            robot_row, robot_col = env.get_cell_from_world(x, y)

            borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
            borders_in_frontier = [b for b in borders if any((f[0] == b[0] and f[1] == b[1]) for f in frontier)]

            if len(borders_in_frontier) != 0:
                selected_border = min(borders_in_frontier, key=lambda b: b[2])
                # Cella da cui si tenta l'ingresso: serve dopo per segnare il LATO provato.
                # Non la cella in cui il robot si trova DOPO il tentativo, che una ritirata o
                # un passo parziale possono aver cambiato (lato sbagliato o nullo).
                origin_row, origin_col = robot_row, robot_col
                check = attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client, env,
                                                         selected_border[0], selected_border[1], gb_sampler, prm,
                                                         mission_folder,
                                                         visualization_counter, recordingInterface, verification_tracker,
                                                         mobility_kwargs=mobility_kwargs)
                visualization_counter += 1
                frontier.remove(selected_border)

                x_new, y_new, _, _ = spotUtils.getPosition(robot_state_client)
                robot_row, robot_col = env.get_cell_from_world(x_new, y_new)

                if check:
                    env.update_position(x_new, y_new)
                    recordingInterface.create_default_waypoint(cell_row=selected_border[0], cell_col=selected_border[1])
                    env.add_waypoint(x_new, y_new)
                    env.mark_cell_visited(selected_border[0], selected_border[1])
                    frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
                else:
                    if (selected_border[0], selected_border[1]) == (robot_row, robot_col):
                        env.update_position(x_new, y_new)
                        recordingInterface.create_default_waypoint(cell_row=selected_border[0],
                                                                   cell_col=selected_border[1])
                        env.add_waypoint(x_new, y_new)
                        env.mark_cell_visited(selected_border[0], selected_border[1])
                        frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
                    else:
                        # RECUPERO subito dopo un tentativo fallito.
                        # 1. Gli archi scartati nel tentativo fallito (blocchi confermati da vicino,
                        # fallimenti fisici) sono gia' nel PRM tramite mark_edge_invalid, e la
                        # ricostruzione degli archi li rispetta: Dijkstra non riproporra' la
                        # stessa strada.

                        # 2. Posizione attuale (il fallimento puo' essere in partenza o a meta' strada)
                        x_curr, y_curr, _, _ = spotUtils.getPosition(robot_state_client)

                        # 3. Scansione fresca, per scegliere il punto obiettivo da qui
                        grids_data_rec, main_proto_rec, _ = local_grid.return_local_grid(
                            ['obstacle_distance', 'terrain', 'terrain_valid'], robot_state_client
                        )

                        if grids_data_rec and 'obstacle_distance' in grids_data_rec:
                            pts = grids_data_rec['obstacle_distance']['pts']
                            cells_obstacle_dist = grids_data_rec['obstacle_distance']['values']
                            terrain_values = grids_data_rec['terrain']['values']
                            valid_values = grids_data_rec['terrain_valid']['values']

                            num_x = main_proto_rec.local_grid.extent.num_cells_x
                            num_y = main_proto_rec.local_grid.extent.num_cells_y
                            cell_size = main_proto_rec.local_grid.extent.cell_size

                            terrain_corrected_rec, is_valid_rec = local_grid.correct_terrain(
                                terrain_values, valid_values, num_x, num_y, cell_size=cell_size,
                                unwritten_value=_raw_zero(grids_data_rec), unwritten_scale=_raw_scale(grids_data_rec)
                            )
                            grad_values, rough_values = local_grid.compute_gradient_and_roughness(
                                terrain_corrected_rec, valid_values, num_x, num_y, cell_size,
                                is_valid=is_valid_rec
                            )

                            grad_2d = grad_values.reshape((num_y, num_x))
                            rough_2d = rough_values.reshape((num_y, num_x))
                            terrain_2d_rec = terrain_corrected_rec.reshape((num_y, num_x))

                            transforms_snapshot = main_proto_rec.local_grid.transforms_snapshot
                            grid_frame_name = main_proto_rec.local_grid.frame_name_local_grid_data
                            vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
                            grid_origin_x = vision_tform_grid.position.x
                            grid_origin_y = vision_tform_grid.position.y

                            prm.update_local_grid_data(
                                grad_values_2d=grad_2d,
                                rough_values_2d=rough_2d,
                                terrain_values_2d=terrain_2d_rec,
                                valid_2d=slope_usable_mask(is_valid_rec, cells_obstacle_dist, (num_y, num_x)),
                                grid_origin_x=grid_origin_x,
                                grid_origin_y=grid_origin_y,
                                cell_size=cell_size
                            )
                            _rebuild_prm_edges(prm, global_grid.global_occupancy_map)

                            target_x, target_y, _, _ = find_best_point_in_cell(
                                x_curr, y_curr, env, selected_border[0], selected_border[1],
                                pts, cells_obstacle_dist, gb_sampler,
                                global_map=global_grid.global_occupancy_map
                            )
                        else:
                            print(
                                "[RECOVERY ERROR] Impossibile recuperare il layer obstacle_distance per il ricalcolo.")
                            target_x, target_y = None, None

                        if target_x is not None and target_y is not None:
                            _restrict_planning_to_explored_cells(
                                prm, env, selected_border[0], selected_border[1], (x_curr, y_curr))
                            start_id = prm.add_stop_node(
                                x_curr, y_curr, global_grid.global_occupancy_map,
                                spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="verifica recupero (robot)")
                            goal_id = prm.add_stop_node(
                                target_x, target_y, global_grid.global_occupancy_map,
                                spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="verifica recupero (obiettivo)",
                                is_robot=False)
                            prm.set_robot_node(start_id)

                            path_ids = prm.find_path_dijkstra(start_id, goal_id)

                            # Si riprova subito solo se il percorso ha al massimo COSTO_SOGLIA
                            # archi (numero di nodi - 1)
                            COSTO_SOGLIA = 4

                            # Se l'obiettivo e' lo STESSO del tentativo appena fallito, riprovare
                            # subito vorrebbe dire stesso posto, stesso esito: la cella si rimanda
                            # e torna in coda come le altre (vedi finalize_or_defer_blocked_cell).
                            last_goal = getattr(env, '_last_attempt_goal', None)
                            same_goal = (last_goal is not None and float(np.hypot(
                                target_x - last_goal[0], target_y - last_goal[1])) <= RECOVERY_SAME_GOAL_M)
                            if same_goal and path_ids is not None:
                                print(f"[RECOVERY] L'obiettivo ({target_x:.2f}, {target_y:.2f}) e' "
                                      f"quello del tentativo appena fallito: non riprovo subito, la "
                                      f"cella torna in coda.")
                                path_ids = None
                            if path_ids is not None and (len(path_ids) - 1) <= COSTO_SOGLIA:
                                print(
                                    f"[RECOVERY] Trovato percorso alternativo con costo {len(path_ids) - 1} <= {COSTO_SOGLIA}. Riprovo l'ingresso...")

                                retry_check = attempt_enter_cell_from_position(
                                    local_grid, global_grid, robot_state_client, command_client, env,
                                    selected_border[0], selected_border[1], gb_sampler, prm,
                                    mission_folder, visualization_counter, recordingInterface, verification_tracker,
                                    mobility_kwargs=mobility_kwargs
                                )
                                visualization_counter += 1

                                if retry_check:
                                    x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
                                    env.update_position(x_final, y_final)
                                    recordingInterface.create_default_waypoint(cell_row=selected_border[0],
                                                                               cell_col=selected_border[1])
                                    env.add_waypoint(x_final, y_final)
                                    env.mark_cell_visited(selected_border[0], selected_border[1])
                                    robot_row_final, robot_col_final = env.get_cell_from_world(x_final, y_final)
                                    frontier.extend(
                                        find_new_borders(env, robot_row_final, robot_col_final, path, frontier))
                                else:
                                    print(
                                        "[FAIL] Anche il percorso alternativo ha fallito. Abbandono la cella e continuo l'algoritmo normale.")
                                    side_bit = env.get_side_bit_facing_origin(origin_row, origin_col, selected_border[0],
                                                                              selected_border[1])
                                    env.mark_cell_side_explored(selected_border[0], selected_border[1], side_bit)
                                    finalize_or_defer_blocked_cell(env, selected_border[0], selected_border[1],
                                                               gb_sampler, global_grid.global_occupancy_map)
                                    if not env.is_cell_blocked(selected_border[0], selected_border[1]):
                                        frontier.append(selected_border)
                            else:
                                print(
                                    f"[SKIP] Nessun percorso alternativo valido o costo superiore a {COSTO_SOGLIA}. Continuo l'algoritmo normale.")
                                side_bit = env.get_side_bit_facing_origin(origin_row, origin_col, selected_border[0],
                                                                          selected_border[1])
                                env.mark_cell_side_explored(selected_border[0], selected_border[1], side_bit)
                                finalize_or_defer_blocked_cell(env, selected_border[0], selected_border[1],
                                                               gb_sampler, global_grid.global_occupancy_map)
                                if not env.is_cell_blocked(selected_border[0], selected_border[1]):
                                    frontier.append(selected_border)
                        else:
                            print(
                                "[SKIP] Nessun punto obiettivo valido nella cella. Continuo con l'esplorazione normale.")
                            side_bit = env.get_side_bit_facing_origin(origin_row, origin_col, selected_border[0],
                                                                      selected_border[1])
                            env.mark_cell_side_explored(selected_border[0], selected_border[1], side_bit)
                            finalize_or_defer_blocked_cell(env, selected_border[0], selected_border[1],
                                                               gb_sampler, global_grid.global_occupancy_map)
                            if not env.is_cell_blocked(selected_border[0], selected_border[1]):
                                frontier.append(selected_border)

            else:
                # Nessuna cella confinante in frontiera: si va con GraphNav verso il waypoint
                # piu' vicino alla cella di rango piu' basso e si tenta da li'.
                # Una cella bloccata (-1) non ha mai lati ancora da provare:
                # finalize_or_defer_blocked_cell() la blocca solo dopo che ogni lato
                # raggiungibile e' stato provato ed e' fallito.
                already_skipped_this_round = set()
                target_row = target_col = rank = nearest_cell = None
                while True:
                    remaining = [c for c in frontier if (c[0], c[1]) not in already_skipped_this_round]
                    lowest_rank_cell = env.get_lowest_rank_from_frontier_list(remaining, path)
                    if lowest_rank_cell is None:
                        break
                    candidate_row, candidate_col, candidate_rank = lowest_rank_cell
                    waypoints_by_cell = recordingInterface.get_all_manual_waypoints_with_cells()
                    candidate_nearest_cell = recordingInterface.find_nearest_waypoint_cell_to_target(
                        (candidate_row, candidate_col), waypoints_by_cell, env)
                    if candidate_nearest_cell is None or \
                            recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell) is None:
                        # Nessuna cella visitata con waypoint da cui partire: si passa alla
                        # candidata successiva.
                        print(f"[SKIP-ROUND] Cella ({candidate_row},{candidate_col}): nessun waypoint "
                              f"GraphNav da cui avvicinarsi. Provo la successiva.")
                        already_skipped_this_round.add((candidate_row, candidate_col))
                        continue
                    candidate_side_bit = env.get_side_bit_facing_origin(candidate_nearest_cell[0],
                                                                        candidate_nearest_cell[1],
                                                                        candidate_row, candidate_col)
                    if env.get_cell_sides_status(candidate_row, candidate_col) & candidate_side_bit:
                        print(f"[SKIP-ROUND] Cella ({candidate_row},{candidate_col}): ci arriverei dallo "
                              f"stesso lato gia' provato senza successo. Per questo giro provo la "
                              f"candidata successiva.")
                        already_skipped_this_round.add((candidate_row, candidate_col))
                        continue
                    target_row, target_col, rank = candidate_row, candidate_col, candidate_rank
                    nearest_cell = candidate_nearest_cell
                    break

                if target_row is not None:
                    x_current, y_current, _, _ = spotUtils.getPosition(robot_state_client)

                    nearest_wp = recordingInterface.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
                    graphnav_add_shortcuts(recordingInterface, "prima dello spostamento fra celle",
                                           map_edges=True, sdk=False, target_wp_id=nearest_wp['id'],
                                           robot_xy=(x_current, y_current))
                    recordingInterface.stop_recording()
                    navigation_success = _graphnav_navigate_to(
                        recordingInterface, nearest_wp['id'], robot_state_client, command_client,
                        f"verso il waypoint della cella {nearest_cell} per la cella ({target_row},{target_col})")

                    # GraphNav non registra i tratti percorsi: quelli vecchi non descrivono piu'
                    # la strada da cui il robot e' arrivato, la ritirata non deve usarli.
                    env._traveled_arcs = []
                    if _check_and_recover_fall(robot_state_client, command_client,
                                               global_grid.global_occupancy_map,
                                               "durante lo spostamento con GraphNav") is not None:
                        recordingInterface.start_recording()
                        print(f"[CADUTA] Rialzato dopo lo spostamento con GraphNav: segno il lato della "
                              f"cella ({target_row},{target_col}) come provato, per non ripetere la stessa strada.")
                        side_bit = env.get_side_bit_facing_origin(nearest_cell[0], nearest_cell[1],
                                                                  target_row, target_col)
                        env.mark_cell_side_explored(target_row, target_col, side_bit)
                        finalize_or_defer_blocked_cell(env, target_row, target_col,
                                                     gb_sampler, global_grid.global_occupancy_map)
                        if env.is_cell_blocked(target_row, target_col):
                            frontier.remove((target_row, target_col, rank))
                        continue

                    if navigation_success:
                        recordingInterface.start_recording()
                        check = attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client,
                                                                 env, target_row, target_col, gb_sampler, prm,
                                                                 mission_folder,
                                                                 visualization_counter, recordingInterface, verification_tracker,
                                                                 mobility_kwargs=mobility_kwargs)
                        visualization_counter += 1

                        if check:
                            x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
                            env.update_position(x_final, y_final)
                            recordingInterface.create_default_waypoint(cell_row=target_row, cell_col=target_col)
                            env.add_waypoint(x_final, y_final)
                            env.mark_cell_visited(target_row, target_col)
                            frontier.remove((target_row, target_col, rank))
                            robot_row, robot_col = env.get_cell_from_world(x_final, y_final)
                            frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
                        else:
                            # Lato segnato come provato; la cella esce dalla frontiera solo se era
                            # l'ULTIMO lato non provato (cella davvero bloccata).
                            side_bit = env.get_side_bit_facing_origin(nearest_cell[0], nearest_cell[1],
                                                                      target_row, target_col)
                            env.mark_cell_side_explored(target_row, target_col, side_bit)
                            finalize_or_defer_blocked_cell(env, target_row, target_col,
                                                     gb_sampler, global_grid.global_occupancy_map)
                            if env.is_cell_blocked(target_row, target_col):
                                frontier.remove((target_row, target_col, rank))
                    else:
                        # GraphNav non e' nemmeno arrivato al waypoint di partenza -- stessa regola:
                        # la cella esce dalla frontiera solo se era l'ultimo lato non provato.
                        recordingInterface.start_recording()
                        side_bit = env.get_side_bit_facing_origin(nearest_cell[0], nearest_cell[1],
                                                                  target_row, target_col)
                        env.mark_cell_side_explored(target_row, target_col, side_bit)
                        finalize_or_defer_blocked_cell(env, target_row, target_col,
                                                     gb_sampler, global_grid.global_occupancy_map)
                        if env.is_cell_blocked(target_row, target_col):
                            frontier.remove((target_row, target_col, rank))
                else:
                    if len(frontier) > 0:
                        frontier.remove(frontier[0])

            if len(frontier) == 0:
                break

        env.print_map()
        x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
        final_row, final_col = env.get_cell_from_world(x_final, y_final)
        recordingInterface.create_default_waypoint(cell_row=final_row, cell_col=final_col)

        # --- SCANSIONE FINALE: l'area attorno all'ultima posizione, prima di tornare ---
        print("[FINAL SCAN] Ultima scansione attorno al robot prima di tornare alla partenza...")
        x_final, y_final, z_final, quat_final = spotUtils.getPosition(robot_state_client)
        final_yaw = np.arctan2(2.0 * (quat_final.w * quat_final.z + quat_final.x * quat_final.y),
                               1.0 - 2.0 * (quat_final.y ** 2 + quat_final.z ** 2))

        grids_data_final, main_proto_final, _ = local_grid.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'], robot_state_client
        )

        if grids_data_final is not None and 'obstacle_distance' in grids_data_final:
            pts_final = grids_data_final['obstacle_distance']['pts']
            cells_obs_final = grids_data_final['obstacle_distance']['values']
            terrain_final = grids_data_final['terrain']['values']
            valid_final = grids_data_final['terrain_valid']['values']

            num_x_final = main_proto_final.local_grid.extent.num_cells_x
            num_y_final = main_proto_final.local_grid.extent.num_cells_y
            cell_size_final = main_proto_final.local_grid.extent.cell_size

            footprint_mask_final = local_grid.compute_robot_footprint_mask(
                pts_final, x_final, y_final, final_yaw, num_x_final, num_y_final
            )

            terrain_real_final, is_valid_final = local_grid.correct_terrain(
                terrain_final, valid_final, num_x_final, num_y_final,
                robot_footprint_mask=footprint_mask_final, cell_size=cell_size_final,
                unwritten_value=_raw_zero(grids_data_final), unwritten_scale=_raw_scale(grids_data_final)
            )
            _log_ground_filter(local_grid, "scan-finale")

            grad_final, rough_final = local_grid.compute_gradient_and_roughness(
                terrain_real_final, valid_final, num_x_final, num_y_final, cell_size_final,
                is_valid=is_valid_final
            )

            obstacle_mask_final = local_grid.fuse_obstacle_mask(
                cells_obstacle_dist=cells_obs_final,
                rough_values=rough_final,
                is_valid=is_valid_final,
                obstacle_threshold=spotGrid.OBSTACLE_THRESHOLD,
                rough_threshold=spotGrid.ROUGH_THRESHOLD
            )

            # Anche l'ultima scansione entra nelle mappe globali, come tutte le altre
            fold_scan_into_global_maps(
                global_grid.global_occupancy_map, global_grid.global_terrain_map, pts_final,
                terrain_real_final, obstacle_mask_final, is_valid_final, cells_obs_final,
                (num_y_final, num_x_final), (x_final, y_final, final_yaw))

            final_save_path = os.path.join(
                mission_folder, f"iteration_{visualization_counter}_FINAL_SCAN.png"
            ) if mission_folder else None

            visualize_grid_with_candidates(
                pts=pts_final, terrain_real=terrain_real_final, obstacle_mask=obstacle_mask_final,
                robot_x=x_final, robot_y=y_final,
                candidates={'rejected': [], 'valid': []},
                chosen_point=None, iteration=visualization_counter, env=env,
                save_path=final_save_path, prm_graph=prm, chosen_path=None,
                cells_obstacle_dist=cells_obs_final, valid_values=valid_final,
                grad_values=grad_final, rough_values=rough_final
            )
            visualization_counter += 1
            print("[FINAL SCAN] Fatto: l'ultima scansione e' nelle mappe globali.")
        else:
            print("[FINAL SCAN] ATTENZIONE: griglia locale non ricevuta, proseguo senza scansione finale.")

        # ---------------------------------------------------------------------
        # RIEPILOGO DI MISSIONE -- solo diagnostica, va letto dopo il volo.
        # ---------------------------------------------------------------------
        _emit_mission_summary_once()

        # Tutto cio' che segue avviene DOPO il riepilogo, quindi non entra in nessuna voce di
        # [TEMPO]: si misura a parte e si stampa in chiusura.
        _t_close = {}
        _t = time.perf_counter()
        try:
            _wp0_id = next((w.id for w in recordingInterface._get_graph(force_refresh=True).waypoints
                            if w.annotations.name == "wp_0"), None)
            _xy_end = spotUtils.getPosition(robot_state_client)[:2]
        except Exception:
            _wp0_id, _xy_end = None, None
        graphnav_add_shortcuts(recordingInterface, "prima del ritorno a wp_0",
                               map_edges=True, sdk=False, target_wp_id=_wp0_id, robot_xy=_xy_end)
        recordingInterface.auto_close_loops(True, False)
        recordingInterface.stop_recording()
        recordingInterface.optimize_anchoring()
        _t_close['chiusura mappa e ancoraggio'] = time.perf_counter() - _t

        _t = time.perf_counter()
        if not _graphnav_return_to_start(recordingInterface, robot_state_client, command_client):
            print("[GRAPHNAV] Ritorno a wp_0 non riuscito: il robot si siede dove si trova.")
        _t_close['ritorno a wp_0'] = time.perf_counter() - _t

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
        estop.stop()


def _parse_args(argv=None):
    """
    Argomenti da riga di comando. Tutti facoltativi: senza argomenti la missione usa i
    valori di mission_graph (MISSION_ROWS, MISSION_COLS, MISSION_CELL_SIZE_M,
    MISSION_SPACING_M) e gli anelli di MISSION_LATTICE_RINGS.
    """
    import argparse
    p = argparse.ArgumentParser(description="Missione di esplorazione autonoma con Spot")
    p.add_argument('--rows', type=int, default=None,
                   help=f"righe di celle, a sinistra del robot (default {MISSION_GRID_ROWS})")
    p.add_argument('--cols', type=int, default=None,
                   help=f"colonne di celle, davanti al robot (default {MISSION_GRID_COLS})")
    p.add_argument('--cell', type=float, default=None,
                   help=f"lato della cella in m (default {MISSION_CELL_SIZE_M:g})")
    p.add_argument('--spacing', type=float, default=None,
                   help=f"passo fra i nodi del reticolo in m, arrotondato a cell/n; gli archi "
                        f"restano >= {PRM_MIN_EDGE_M:g} m (default {MISSION_LATTICE_SPACING_M}; "
                        f"0 = suggerito)")
    p.add_argument('--rings', default=None,
                   help=f"anelli di vicini collegati, o 'tutti' (default {MISSION_LATTICE_RINGS})")
    p.add_argument('--grafo', choices=mission_graph.GRAPH_SAMPLINGS, default=None,
                   help=f"come si generano i nodi (default {MISSION_GRAPH_SAMPLING})")
    p.add_argument('--distribuzione', choices=mission_graph.RANDOM_DISTRIBUTIONS, default=None,
                   help=f"nodi casuali: distribuzione (default {MISSION_RANDOM_DISTRIBUTION})")
    p.add_argument('--densita', type=float, default=None,
                   help=f"nodi casuali: nodi per m2 (default {MISSION_RANDOM_DENSITY:g})")
    p.add_argument('--seme', type=int, default=None,
                   help="nodi casuali: seme, per ripetere un grafo (default: estratto a caso, "
                        "scritto nel log)")
    p.add_argument('--min-edge', type=float, default=None,
                   help=f"lunghezza minima degli archi in m (default {PRM_MIN_EDGE_M:g})")
    p.add_argument('--max-edge', type=float, default=None,
                   help=f"lunghezza massima degli archi in m (default {PRM_MAX_EDGE_M:g})")
    a = p.parse_args(argv)
    if a.rows is not None and a.rows < 1 or a.cols is not None and a.cols < 1:
        p.error("--rows e --cols devono essere almeno 1")
    if a.cell is not None and a.cell <= 0:
        p.error("--cell deve essere positivo")
    if a.densita is not None and a.densita <= 0:
        p.error("--densita deve essere positiva")
    lo = PRM_MIN_EDGE_M if a.min_edge is None else a.min_edge
    hi = PRM_MAX_EDGE_M if a.max_edge is None else a.max_edge
    if lo <= 0 or hi < lo:
        p.error("serve 0 < --min-edge <= --max-edge")
    return a


def main():
    global MISSION_LATTICE_SPACING_M, MISSION_LATTICE_RINGS, PRM_MIN_EDGE_M, PRM_MAX_EDGE_M
    global MISSION_GRAPH_SAMPLING, MISSION_RANDOM_DISTRIBUTION, MISSION_RANDOM_DENSITY
    global MISSION_RANDOM_SEED
    args = _parse_args()
    if args.grafo is not None:
        MISSION_GRAPH_SAMPLING = args.grafo
    if args.distribuzione is not None:
        MISSION_RANDOM_DISTRIBUTION = args.distribuzione
    if args.densita is not None:
        MISSION_RANDOM_DENSITY = float(args.densita)
    if args.seme is not None:
        MISSION_RANDOM_SEED = int(args.seme)
    if args.min_edge is not None:
        PRM_MIN_EDGE_M = float(args.min_edge)
    if args.max_edge is not None:
        PRM_MAX_EDGE_M = float(args.max_edge)
    if args.spacing is not None:
        MISSION_LATTICE_SPACING_M = None if args.spacing <= 0 else float(args.spacing)
    if args.rings is not None:
        MISSION_LATTICE_RINGS = (None if str(args.rings).lower() in ('none', 'tutti', 'all', '-1')
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
        _emit_mission_summary_once()
        _remove_stdout_tee()


if __name__ == '__main__':
    if not main():
        sys.exit(1)