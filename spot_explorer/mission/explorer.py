"""
easy_walk.py -- la missione di esplorazione autonoma di Spot, cella per cella.

    python3 easy_walk.py [--rows R --cols C --cell L --grafo reticolo|casuale ...]
    (vedi _parse_args; senza argomenti valgono i valori di parametri.py)

L'area e' una griglia di celle (environmentMap). Il robot le visita in ordine a
serpentina: entra in una cella confinante quando c'e', altrimenti ci si avvicina con
GraphNav dal waypoint della cella visitata piu' vicina. Ogni ingresso e' un ciclo
scansione -> mappe -> grafo PRM -> fronte sicuro -> decisione -> movimento.

Contenuto, nell'ordine:
  mission_switches, _emit_mission_summary_once   interruttori e riepilogo di fine missione
  find_best_point_in_cell (+ obiettivi rifiutati) il punto da raggiungere in una cella
  escape_by_backtracking, _trap_backtrack        ritorno indietro con GraphNav
  TentativoCella                                 UN tentativo di entrare in una cella:
                                                 _prepara, poi a ogni giro le fasi
                                                 _guardie ... _esegui_tratti, infine _fine
  retreat_along_traveled_path                    ritirata a zampe, l'ultima rete
  cell_has_reachable_target, finalize_or_defer_blocked_cell   celle bloccate o rimandate
  Missione                                       il ciclo cella per cella
  easy_walk, _chiudi_missione, main              avvio, chiusura, riga di comando

Gli altri moduli: parametri (numeri e interruttori), registro (log e statistiche), spotGrid
(scansione e mappe), mission_graph e prm_graph (grafo), arcVerification (fronte e decisioni
di movimento), movements (comandi di movimento, cadute), navGraphUtils (GraphNav),
environmentMap (celle). Vedi README.md.
"""
import os
import sys
import json
import time
from time import sleep
from datetime import datetime
import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, blocking_stand)
from types import SimpleNamespace
from spot_explorer.graphnav import graphnav as navGraphUtils
from spot_explorer.robot import motion as movements
from spot_explorer.perception import grids as spotGrid
from spot_explorer.robot import connection as spotLogInUtils
from spot_explorer.mission import cells as environmentMap
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import safe_frontier as arcVerification
from spot_explorer.robot import battery as batteria

from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.planning import roadmap as prm_graph_module   # dentro le funzioni `prm_graph` e' spesso il parametro (istanza)
from spot_explorer.planning import mission_graph                   # reticolo dei nodi della missione

from spot_explorer import config as P
from spot_explorer import REPO_ROOT
from spot_explorer.records import (MISSION_STATS, MISSION_CTX, BodyTiltRecorder, SegmentLog,
                      TrajectoryLog, append_csv, append_trajectory, install_stdout_tee,
                      log_segment, mission_z0, remove_stdout_tee, save_npz_async,
                      set_mission_z0, timing_add,
                      visualize_grid_with_candidates)

# Dove nascono MissionMap/, MissionLogs/ e graph/ (i test la spostano in una cartella temporanea).
DATA_ROOT = REPO_ROOT


# =========================================================================
# INTERRUTTORI E RIEPILOGO -- solo diagnostica: nessuna decisione del robot dipende da
# queste funzioni, e non sollevano mai eccezioni verso il chiamante (un log che fallisce
# non deve fermare o alterare un movimento). Log e statistiche stanno in registro.py.
# =========================================================================


def mission_switches():
    """
    Gli interruttori che cambiano come lavora la missione, con il loro valore adesso. Vanno nel
    log ([CONFIG] INTERRUTTORI) e in mission_summary.json, cosi' dopo una prova si sa con quali
    impostazioni e' stata fatta ogni missione (per i confronti acceso/spento).
    """
    return {
        'parametri.MISSION_GRAPH_SAMPLING': P.MISSION_GRAPH_SAMPLING,
        'mission_graph.BATCH_SLOPES': mission_graph.BATCH_SLOPES,
        'prm_graph.REFRESH_BATCH_SLOPES': prm_graph_module.REFRESH_BATCH_SLOPES,
        'parametri.PLAN_ONLY_EXPLORED_CELLS': P.PLAN_ONLY_EXPLORED_CELLS,
        'parametri.GRAPHNAV_SHORTCUTS': P.GRAPHNAV_SHORTCUTS,
        'parametri.GRAPHNAV_PRECHECK': P.GRAPHNAV_PRECHECK,
        'parametri.REPICK_ON_REPLAN_FAIL': P.REPICK_ON_REPLAN_FAIL,
        'parametri.GRAPHNAV_ARRIVAL_HEADING': P.GRAPHNAV_ARRIVAL_HEADING,
        'parametri.WP0_FIDUCIAL_CORRECTION': P.WP0_FIDUCIAL_CORRECTION,
        'parametri.MOVE_NEAR_GOAL_M': P.MOVE_NEAR_GOAL_M,
        'parametri.GRAPHNAV_SDK_LOOP_CLOSURE': P.GRAPHNAV_SDK_LOOP_CLOSURE,
        'parametri.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S': P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S,
        'parametri.SAVE_SCAN_BUNDLES': P.SAVE_SCAN_BUNDLES,
        'parametri.SAVE_EXTRA_LAYERS': list(P.SAVE_EXTRA_LAYERS),
        'movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT': movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT,
        'spotGrid.ARC_BODY_EXTENSION_M': spotGrid.ARC_BODY_EXTENSION_M,
        'spotGrid.ROBOT_CLEARANCE_AIR_M': spotGrid.ROBOT_CLEARANCE_AIR_M,
        'spotGrid.PLAN_CLEARANCE_EXTRA_M': spotGrid.PLAN_CLEARANCE_EXTRA_M,
        'parametri.BATTERY_RETURN_PCT': P.BATTERY_RETURN_PCT,
    }


def _rientro_batteria(where=None):
    """Una lettura della batteria (vedi batteria.MonitorBatteria.controlla): True se la
    missione deve smettere di esplorare e rientrare. False se il monitor non c'e'."""
    monitor = MISSION_CTX.get('batteria')
    return monitor.controlla(where) if monitor is not None else False


def _batteria_scarica():
    """Come _rientro_batteria ma senza leggere: solo se il rientro e' gia' deciso."""
    monitor = MISSION_CTX.get('batteria')
    return monitor is not None and monitor.rientro


def _emit_mission_summary_once():
    """Stampa e salva il riepilogo di missione una sola volta. Viene chiamata sia a fine
    missione normale sia da main() in un blocco finally, cosi' esce anche se la missione
    termina per un'eccezione o per una ripresa del controllo."""
    if MISSION_CTX.get('summary_done') or MISSION_CTX.get('prm') is None:
        return
    MISSION_CTX['summary_done'] = True
    try:
        prm = MISSION_CTX['prm']
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
        print(f"Obiettivi esclusi per la missione: {MISSION_STATS['goals_refused']}, scelte senza "
              f"punti raggiungibili: {MISSION_STATS['goals_unreachable']}, arrivi con il robot appena "
              f"fuori dalla cella: {MISSION_STATS['arrivals_near_goal']}, ritorni senza "
              f"movimento: {MISSION_STATS['backtracks_in_place']}")
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
        print(f"Tratti non conclusi ma arrivati entro {P.MOVE_NEAR_GOAL_M:.2f} m dal punto (validi): "
              f"{MISSION_STATS.get('moves_near_goal', 0)}")
        print(f"Spostamenti GraphNav riusciti: {MISSION_STATS['graphnav_ok']}, falliti: {MISSION_STATS['graphnav_fail']}")
        print(f"Deriva dell'odometria: {MISSION_STATS['drift_samples']} misure agli arrivi GraphNav, "
              f"massima {MISSION_STATS['drift_max_m']:.2f} m, ultima {MISSION_STATS['drift_last_m']:.2f} m "
              f"(dettaglio in drift.csv)")
        print(f"Chiusura d'anello di Spot: {MISSION_STATS['sdk_loop_calls']} chiamate, "
              f"{MISSION_STATS['sdk_loop_s']:.0f} s, {MISSION_STATS['sdk_loop_edges']} archi, "
              f"{MISSION_STATS['sdk_loop_partial']} finite allo scadere del limite, "
              f"{MISSION_STATS['sdk_loop_failures']} tentativi falliti")
        print(f"Riagganci GraphNav (robot perso): {MISSION_STATS.get('graphnav_relocalizations', 0)} riusciti, "
              f"{MISSION_STATS.get('graphnav_relocalization_fail', 0)} falliti | celle rimandate dopo "
              f"{P.MAX_BACKTRACKS_PER_ATTEMPT} ritorni nello stesso tentativo: {MISSION_STATS.get('backtrack_giveups', 0)} | "
              f"archi tolti dalla scansione dal vivo nei refresh: {MISSION_STATS.get('live_arc_vetoes', 0)}")
        lg_main = MISSION_CTX.get('local_grid')
        lg_trk = getattr(MISSION_CTX.get('tracker'), '_local_grid', None)
        print(f"Scansioni scartate (layer mancanti o non allineati): ciclo principale "
              f"{getattr(lg_main, 'rejected_scans', 0)}, controllo in movimento {getattr(lg_trk, 'rejected_scans', 0)}")
        print(f"Pacchetti di scansione salvati: {MISSION_STATS['scan_bundles']}")
        if MISSION_CTX.get('batteria') is not None:
            print(MISSION_CTX['batteria'].riepilogo())
        times = MISSION_STATS['time_s']
        if times:
            tot = sum(times.values())
            print(f"[TEMPO] Ripartizione del tempo misurato ({tot:.0f} s):")
            for phase, sec in sorted(times.items(), key=lambda kv: -kv[1]):
                print(f"  {phase:22s} {sec:8.1f} s  ({100.0 * sec / tot:5.1f}%)")
        print("===================================================\n")

        folder = MISSION_CTX.get('log_folder')
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
                'goal_reached_tolerance_m': P.GOAL_REACHED_TOLERANCE_M,
                'interruttori': mission_switches(),
                'fiducial_partenza': MISSION_CTX.get('start_fiducial'),
            }
            mg = getattr(prm, 'mission_graph', None)
            if mg is not None:
                summary['grafo'] = mg.describe()
            with open(os.path.join(folder, 'mission_summary.json'), 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, default=str)
    except Exception as e:
        print(f"[LOG] Impossibile produrre il riepilogo di missione: {e}")


# =========================================================================
# SCELTA DEL PUNTO OBIETTIVO in una cella
# =========================================================================


def _refused_goals(env, row, col):
    """Gli obiettivi della cella rifiutati dal fronte in questa missione (lista di (x, y))."""
    store = getattr(env, '_refused_goals', None)
    if store is None:
        store = env._refused_goals = {}
    return store.setdefault((int(row), int(col)), [])


def _refuse_goal(env, row, col, x, y, why):
    """
    Esclude l'obiettivo (x, y) della cella fino alla fine della missione. Il 22-10 16:04 il
    nodo 548 della cella (7,4) e' stato riscelto a ogni tentativo e bloccato tre volte nello
    stesso punto: la mappa globale lo dava libero a 0.28 m, il fronte dal vivo a 0.24.
    """
    lst = _refused_goals(env, row, col)
    if any(np.hypot(x - ex, y - ey) < P.RECOVERY_SAME_GOAL_M for ex, ey in lst):
        return
    lst.append((float(x), float(y)))
    MISSION_STATS['goals_refused'] = MISSION_STATS.get('goals_refused', 0) + 1
    print(f"[TARGET] Obiettivo ({x:.2f}, {y:.2f}) della cella ({row},{col}) escluso per il "
          f"resto della missione: {why}.")


def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist,
                            sampler, global_map=None, safety_margin=None, exclude=None,
                            reachable=None):
    """
    Sceglie il punto obiettivo dentro la cella fra il centro geometrico e i nodi del grafo
    nella cella.

    Scartati sempre (mai scelti): i punti in `exclude` (gia' provati in questo tentativo),
    quelli occupati nella mappa globale (con safety_margin) e quelli a meno di
    spotGrid.PLAN_CLEARANCE_M (0.28) da un ostacolo nella scansione dal vivo o nello spazio
    libero imparato. Fra i rimasti, nell'ordine:
      1. non rifiutati dal fronte in questa missione (_refuse_goal); i rifiutati tornano
         solo se non c'e' altro;
      2. RAGGIUNGIBILI dal robot nel grafo (reachable(x, y) -> bool, provato nell'ordine
         dei criteri seguenti finche' uno risponde si'). Non basta ripiegare sui successivi
         del piu' vicino al centro quando il grafo non ci arriva: stanno nella stessa zona
         isolata (il 22-10 16:04 quel ripiego e' fallito 9 volte su 10). Se nessun punto e'
         raggiungibile si sceglie comunque il migliore, e il tentativo fallisce;
      3. con almeno TARGET_CLEARANCE_M (0.33) di spazio libero;
      4. il centro geometrico, libero o ignoto che sia (REGOLA DEL CENTRO: un punto gia'
         visto libero di solito e' l'ultimo lembo scansionato dalla cella precedente, cioe'
         sul bordo);
      5. visti liberi prima che mai visti: se il centro e' scartato la zona e' nota, e
         tornare a guardare un ignoto accanto all'ostacolo sarebbe un giro a vuoto;
      6. ad almeno TARGET_CELL_BORDER_M dal bordo della cella;
      7. piu' vicino al centro (a passi di 0.25 m), poi piu' vicino al robot: a parita',
         quello dalla sua parte dell'ostacolo.

    Se non resta nessun punto la cella non ha un obiettivo valido e la funzione
    restituisce (None, None, ...): il chiamante lo tratta gia' come fallimento di
    ingresso e passa alla logica dei lati non ancora provati.

    global_map: spotGrid.GlobalGrid accumulata, aggiornata con la scansione corrente appena
    prima di questa chiamata. Se None si usa il centro senza controlli.

    Returns:
        (target_x, target_y, valid_samples, rejected_samples)
        Le due liste alimentano la visualizzazione (gialli = ammessi, rossi = scartati).
    """
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

    # min_clear e' la distanza con cui si pianificano gli archi (spotGrid.PLAN_CLEARANCE_M):
    # un obiettivo piu' vicino agli ostacoli non avrebbe archi. La mappa globale dichiara
    # occupata una cella solo dopo due osservazioni, quindi un ostacolo appena visto c'e'
    # solo nella scansione dal vivo (22-10 15:50, cella (0,2)).
    min_clear = spotGrid.PLAN_CLEARANCE_M
    pref_margin = max(safety_margin, P.TARGET_CLEARANCE_M - spotGrid.OBSTACLE_THRESHOLD)
    refused = _refused_goals(env, cell_row, cell_col)
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

    def _near(p, pts_list):
        return bool(pts_list) and any(np.hypot(p[0] - ex, p[1] - ey) < P.RECOVERY_SAME_GOAL_M
                                      for ex, ey in pts_list)

    center = (cell_center_x, cell_center_y)
    candidates = [center] + [p for p in sampled
                             if np.hypot(p[0] - cell_center_x, p[1] - cell_center_y) > 0.01]
    rows, blocked = [], []
    center_reason = None
    for p in candidates:
        px, py = p
        is_center = p is center
        if _near(p, exclude):
            blocked.append(p)
            center_reason = center_reason or ("gia' scartato dal vivo" if is_center else None)
            continue
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            blocked.append(p)
            center_reason = center_reason or ("occupato nella mappa" if is_center else None)
            continue
        c_live = _live_clearance(px, py)
        # Per il centro conta la sola scansione dal vivo: lo spazio libero imparato, che
        # raccoglie anche letture da lontano, cambierebbe il centro piu' del dovuto.
        c_mem = None if is_center else (global_map.clearance(px, py)
                                        if hasattr(global_map, 'clearance') else None)
        known_c = [c for c in (c_live, c_mem) if c is not None]
        if known_c and min(known_c) < min_clear:
            blocked.append(p)
            if is_center:
                center_reason = f"a {min(known_c):.2f} m da un ostacolo nella scansione dal vivo"
            continue
        tight = (global_map.is_occupied(px, py, safety_margin=pref_margin)
                 or (bool(known_c) and min(known_c) < P.TARGET_CLEARANCE_M))
        border = environmentMap.cell_border_distance(env, cell_row, cell_col, px, py)
        rows.append(dict(
            p=(float(px), float(py)), center=is_center, refused=_near(p, refused),
            seen=(c_live is not None or global_map.is_known(px, py)), tight=tight,
            clear=(min(known_c) if known_c else None),
            edge=(border is not None and border < P.TARGET_CELL_BORDER_M),
            dc=float(np.hypot(px - cell_center_x, py - cell_center_y)),
            dr=float(np.hypot(px - robot_x, py - robot_y))))

    valid = [r['p'] for r in rows]
    if not rows:
        print(f"[TARGET] Cella ({cell_row},{cell_col}): centro {center_reason or 'scartato'} e "
              f"nessun punto della cella ad almeno {min_clear:.2f} m dagli ostacoli. "
              f"Nessun obiettivo valido.")
        return None, None, [], blocked

    step = 0.25
    rows.sort(key=lambda r: (r['tight'], not r['center'], not r['seen'], r['edge'],
                             round(r['dc'] / step), r['dr']))
    fresh = [r for r in rows if not r['refused']]
    pool = fresh if fresh else rows
    chosen, reach_note = None, ""
    if reachable is None:
        chosen = pool[0]
    else:
        for r in pool:
            if reachable(*r['p']):
                chosen = r
                break
        if chosen is None:
            chosen = pool[0]
            reach_note = "; NESSUN punto della cella e' raggiungibile dal robot nel grafo"
            MISSION_STATS['goals_unreachable'] = MISSION_STATS.get('goals_unreachable', 0) + 1
    if not fresh:
        reach_note += "; tutti i punti erano gia' stati rifiutati dal fronte, riprovo il migliore"

    bx, by = chosen['p']
    flags = []
    if chosen['tight']:
        flags.append(f"meno di {P.TARGET_CLEARANCE_M:.2f} m di spazio libero")
    if chosen['edge']:
        flags.append(f"a meno di {P.TARGET_CELL_BORDER_M:.2f} m dal bordo della cella")
    flag_txt = f" -- {', '.join(flags)}" if flags else ""
    if chosen['center']:
        state = "libero" if chosen['seen'] else "ignoto"
        print(f"[TARGET] Cella ({cell_row},{cell_col}): uso il centro geometrico "
              f"({bx:.2f}, {by:.2f}), {state}{flag_txt}{reach_note}.")
        return bx, by, valid, blocked

    if center_reason is None:
        crow = next((r for r in rows if r['center']), None)
        if crow is None:
            center_reason = "scartato"
        elif crow['refused']:
            center_reason = "rifiutato dal fronte in questa missione"
        elif crow['tight'] and not chosen['tight']:
            center_reason = (f"con solo {crow['clear']:.2f} m di spazio libero"
                             if crow['clear'] is not None else "troppo vicino a un ostacolo nella mappa")
        else:
            center_reason = "non raggiungibile dal robot nel grafo"
    n_seen = sum(1 for r in rows if r['seen'])
    print(f"[TARGET] Cella ({cell_row},{cell_col}): centro {center_reason}, uso il punto "
          f"{'libero' if chosen['seen'] else 'ignoto'} ({bx:.2f}, {by:.2f}), a {chosen['dc']:.2f} m "
          f"dal centro{flag_txt}{reach_note}  [liberi={n_seen} ignoti={len(rows) - n_seen} "
          f"scartati={len(blocked)} stretti={sum(r['tight'] for r in rows)} "
          f"sul bordo={sum(r['edge'] for r in rows)} rifiutati={sum(r['refused'] for r in rows)}]")
    return bx, by, valid, blocked


# =========================================================================
# RITORNO INDIETRO CON GRAPHNAV, quando dal punto in cui si e' il grafo non porta
# alla cella
# =========================================================================


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
                  f"(ritorno {trap_state['n']}/{P.MAX_BACKTRACKS_PER_ATTEMPT}).")
    esc = escape_by_backtracking(
        env, prm_graph, goal_id, recordingInterface, robot_state_client, command_client,
        global_map, mobility_kwargs, pts, obstacle_mask, label=label, **kwargs)
    if trap_state['n'] >= P.MAX_BACKTRACKS_PER_ATTEMPT and esc.get('ok'):
        MISSION_STATS['backtrack_giveups'] = MISSION_STATS.get('backtrack_giveups', 0) + 1
        print(f"[RITORNO] {trap_state['n']} ritorni in questo tentativo: sono tornato "
              f"indietro, ma non riprovo. Rimando la cella e la provo da un altro lato.")
        esc = dict(esc, ok=False, how='rinuncia dopo ritorni ripetuti')
    return esc


def escape_by_backtracking(env, prm_graph, goal_id, recordingInterface, robot_state_client,
                           command_client, global_map, mobility_kwargs, pts, obstacle_mask,
                           max_hops=P.GRAPHNAV_MAX_BACKTRACK_HOPS, label="",
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
        # Si arriva girati verso l'obiettivo del tentativo: da li' si ripianifica verso di lui.
        ok, crumb = navGraphUtils.graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                                                     face_xy=prm_graph.get_node_position(goal_id))
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

        if not crumb.get('in_place'):
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
        if crumb.get('in_place'):
            break           # non ci sono briciole piu' lontane: altri giri sarebbero uguali

    print(f"[RITORNO] {label}: {hop} ritorni senza trovare un percorso. Rimando la cella.")
    return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}


# =========================================================================
# INGRESSO IN UNA CELLA: un tentativo (TentativoCella)
# =========================================================================


# Esiti di una fase del ciclo di navigazione (vedi TentativoCella._giro): None = si va avanti
# con la fase successiva.
CONTINUA = "continua"        # giro finito: si ricomincia dalla scansione
INTERROMPI = "interrompi"    # fine del ciclo: decide la posizione finale (TentativoCella._fine)
FALLITO = "fallito"          # il tentativo e' fallito


class TentativoCella:
    """Prova a entrare nella cella (target_row, target_col) dalla posizione attuale del robot.

    Scansione, aggiornamento delle mappe e del grafo, scelta del punto obiettivo, percorso,
    poi il ciclo di navigazione con il fronte sicuro. True se il robot finisce dentro la cella.

    mobility_kwargs: dict facoltativo che impone l'andatura (locomotion_hint, ground_mu_hint,
        swing_height) per tutti i tratti, passato fino a movements.relative_move attraverso
        navigate_to. Vuoto = andatura scelta tratto per tratto (select_mobility_params).

    COME E' FATTO. _prepara una volta (_prima_scansione, _dati_locali_nel_grafo,
    _obiettivo_e_percorso, _azzera_contatori), poi _giro finche' una fase decide l'esito:

      _guardie              troppi giri, batteria bassa, caduta, robot fermo da troppo
      _scansione_e_grafo    _scansione_fresca, _aggiorna_grafo, _ripianifica
      _fronte_e_decisione   scorciatoia, fronte sicuro, arcVerification.decide_next_move:
                            move / reached_node / arrived / wait / blocked
      _rotazione            (move) non posso girarmi: _avanza_nel_cono, _arretra_per_ruotare,
                            trappola, tratto scartato
      _nodo_o_arrivo        (reached_node, arrived)
      _attesa               (wait) _cambia_obiettivo_irraggiungibile, _ripianifica_subito,
                            altrimenti riguardo da fermo
      _blocco               (blocked) obiettivo da cambiare, trappola, tratto fuori per la
                            missione, nuova strada o ritorno con GraphNav
      _esegui_tratti        (move) _controlla_posa_d_arrivo, poi per ogni gamba
                            _prepara_gamba e _cammina_tratto (_terreno_del_tratto,
                            _muovi_e_registra, esito)

    Azioni comuni alle fasi: _usa_percorso, _scarta_tratto (per ora / per il tentativo / per
    la missione), _strada_fatta e _sosta, _ventaglio ed _esci_dalla_trappola,
    _riscegli_obiettivo, _ritorno (GraphNav all'indietro, vedi _trap_backtrack).
    """

    def __init__(self, local_grid, global_grid, robot_state_client, command_client,
                 env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                 iteration=0,
                 recordingInterface=None, verification_tracker=None, mobility_kwargs=None):
        self.local_grid = local_grid
        self.global_grid = global_grid
        self.robot_state_client = robot_state_client
        self.command_client = command_client
        self.env = env
        self.target_row = target_row
        self.target_col = target_col
        self.sampler = sampler
        self.prm_graph = prm_graph
        self.mission_folder = mission_folder
        self.iteration = iteration
        self.recordingInterface = recordingInterface
        self.verification_tracker = verification_tracker
        self.mobility_kwargs = mobility_kwargs

    def esegui(self):
        """True se il robot finisce dentro la cella (o accettato appena fuori, vedi _fine)."""
        if self._prepara() is FALLITO:
            return False

        while self.path_waypoints and len(self.path_waypoints) > 0:
            esito = self._giro()
            if esito is FALLITO:
                return False
            if esito is INTERROMPI:
                break
        return self._fine()

    def _giro(self):
        """Un giro del ciclo di navigazione: le fasi in ordine, finche' una non decide l'esito."""
        for fase in (self._guardie, self._scansione_e_grafo, self._fronte_e_decisione,
                     self._rotazione, self._tempi, self._nodo_o_arrivo, self._attesa,
                     self._blocco, self._esegui_tratti):
            esito = fase()
            if esito is not None:
                return esito
        return CONTINUA

    def _prepara(self):
        """Prima del ciclo: rialzo se caduto, prima scansione, mappe e grafo, punto obiettivo
        e percorso, contatori del tentativo. FALLITO se non si puo' nemmeno partire."""
        self.mobility_kwargs = self.mobility_kwargs or {}

        print(f"\n[ATTEMPT] Provo a entrare nella cella ({self.target_row},{self.target_col}) dalla posizione attuale...")

        scan = self._prima_scansione()
        if scan is None:
            return FALLITO
        self._dati_locali_nel_grafo(scan)
        esito = self._obiettivo_e_percorso(scan)
        if esito is not None:
            return esito
        self._azzera_contatori()

    def _prima_scansione(self):
        """Rialzo se caduto, prima scansione del tentativo, posa del robot, mappe globali
        aggiornate e salvate. La scansione elaborata, o None se non arriva."""
        # Il robot arriva qui da un altro movimento (navigazione fra celle, ritirata): se e'
        # caduto nel frattempo si rialza prima di tutto. Solleva RobotFallenError se non puo'.
        # Poi esce dal disco pericoloso ripercorrendo i propri passi: rialzato, e' ancora DENTRO
        # il punto della caduta, e da li' ogni arco del PRM verrebbe scartato.
        if movements.check_and_recover_fall(self.robot_state_client, self.command_client,
                                   self.global_grid.global_occupancy_map, "all'inizio del tentativo") is not None:
            retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)

        # 1. La scansione: i layer in una richiesta, poi (sotto) la loro elaborazione
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Rilevamento griglie locali fallito.")
            return None
        main_proto = scan.proto
        pts, cells_obstacle_dist = scan.pts, scan.od
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size

        # =========================================================================
        # MAPPE GLOBALI: create alla prima scansione, poi accumulate per tutta la missione
        # =========================================================================
        if self.global_grid.global_occupancy_map is None:
            self.global_grid.global_occupancy_map = spotGrid.GlobalGrid(resolution=cell_size)
            print("[MAP] Inizializzata nuova mappa di occupazione globale persistente.")

        # Mappa globale delle altezze: stessa risoluzione nativa della griglia locale (nessuna
        # riduzione), creata come quella di occupazione.
        if self.global_grid.global_terrain_map is None:
            self.global_grid.global_terrain_map = spotGrid.GlobalTerrainGrid(resolution=cell_size)
            print("[MAP] Inizializzata nuova mappa globale persistente del terreno (altezza).")

        self.global_map = self.global_grid.global_occupancy_map
        self.global_terrain_map = self.global_grid.global_terrain_map

        transforms_snapshot = main_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        self.robot_x, self.robot_y, self.robot_z = vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

        quat = vision_tform_body.rotation
        self.robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))

        # 2. Impronta del corpo, terreno corretto (da qui in poi si usa solo terrain_real),
        # pendenza e rugosita', maschera degli ostacoli
        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-iniziale")
        self.footprint_mask_2d, terrain_real, is_valid = scan.footprint_mask, scan.terrain, scan.is_valid
        obstacle_mask = scan.obstacle_mask

        # Aggiorna le mappe globali (occupazione e terreno) con i dati appena processati
        spotGrid.fold_scan_into_global_maps(self.global_map, self.global_terrain_map, pts, terrain_real, obstacle_mask,
                                   is_valid, cells_obstacle_dist, (num_y, num_x),
                                   (self.robot_x, self.robot_y, self.robot_yaw))

        # La mappa del terreno serve al PRM come ripiego per la pendenza
        self.prm_graph.set_global_terrain_map(self.global_terrain_map)

        # Mappe globali su disco (.pkl), a ogni tentativo
        if self.mission_folder is not None:
            pkl_map_path = os.path.join(self.mission_folder, "global_occupancy_map.pkl")
            self.global_map.save_map(pkl_map_path)
            pkl_terrain_path = os.path.join(self.mission_folder, "global_terrain_map.pkl")
            self.global_terrain_map.mission_z0 = MISSION_CTX.get('mission_z0')
            self.global_terrain_map.save_map(pkl_terrain_path)
        return scan

    def _dati_locali_nel_grafo(self, scan):
        """Terreno, pendenza e ostacoli della scansione nel PRM, poi aggiornamento degli archi."""
        pts, cells_obstacle_dist, valid_values = scan.pts, scan.od, scan.valid
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size
        terrain_real, is_valid = scan.terrain, scan.is_valid
        grad_values, rough_values = scan.grad, scan.rough
        grad_for_prm = grad_values.copy()
        rough_for_prm = rough_values.copy()
        main_proto = scan.proto
        transforms_snapshot = main_proto.local_grid.transforms_snapshot

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

        self.prm_graph.update_local_grid_data(
            grad_values_2d=grad_values_2d,
            rough_values_2d=rough_values_2d,
            terrain_values_2d=terrain_values_2d,
            valid_2d=spotGrid.slope_usable_mask(is_valid, cells_obstacle_dist, (num_y, num_x)),
            grid_origin_x=grid_origin_x,
            grid_origin_y=grid_origin_y,
            cell_size=cell_size
        )

        # Gradiente e rugosita' dei nodi PRM dentro la griglia locale; quelli fuori tengono il
        # valore che hanno (0.0 se non ne hanno ancora uno)
        for node_id, (nx, ny) in self.prm_graph.nodes.items():
            if local_x_min <= nx <= local_x_max and local_y_min <= ny <= local_y_max:
                dx = nx - grid_origin_x
                dy = ny - grid_origin_y

                col_idx = int(dx / cell_size)
                row_idx = int(dy / cell_size)

                if 0 <= row_idx < num_y and 0 <= col_idx < num_x:
                    flat_idx = row_idx * num_x + col_idx
                    if flat_idx < len(grad_values):
                        self.prm_graph.node_gradients[node_id] = grad_values[flat_idx]
                    if flat_idx < len(rough_values):
                        self.prm_graph.node_roughness[node_id] = rough_values[flat_idx]
            else:
                if node_id not in self.prm_graph.node_gradients:
                    self.prm_graph.node_gradients[node_id] = 0.0
                if node_id not in self.prm_graph.node_roughness:
                    self.prm_graph.node_roughness[node_id] = 0.0

        # La scansione appena fatta entra anche nello scarto preliminare degli archi
        # (prm_graph.LIVE_PREFILTER_OD_M): alla costruzione la mappa globale non ha ancora
        # confermato niente di questo posto, e gli archi che attraversano gli ostacoli appena
        # visti resterebbero validi. Qui obstacle_distance e' quella grezza: la fotografia
        # filtrata nasce nel ciclo di navigazione.
        try:
            self.prm_graph.set_live_obstacles(
                np.asarray(cells_obstacle_dist, dtype=np.float64).reshape((num_y, num_x)),
                np.asarray(valid_values, dtype=np.float64).reshape((num_y, num_x)) >= spotGrid.TERRAIN_VALID_MIN,
                grid_origin_x, grid_origin_y, cell_size, self.robot_x, self.robot_y, self.robot_yaw)
        except Exception as e:
            self.prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
            print(f"[ARCHI] Scansione non usabile per lo scarto preliminare ({e}).")

        t_build = time.perf_counter()
        # ATTENZIONE AI NOMI: qui `self.prm_graph` e' l'istanza di PRM, non il modulo omonimo
        # (importato come prm_graph_module). Le coppie candidate sono quelle del grafo di
        # missione (vedi mission_graph.build_mission_prm); veti e pesi li calcola
        # PRM._evaluate_and_add_edge.
        mission_graph.rebuild_prm_edges(self.prm_graph, self.global_map)
        timing_add('costruzione grafo', time.perf_counter() - t_build)
        _upd = getattr(getattr(self.prm_graph, 'mission_graph', None), 'last_update', None)
        print(f"[TEMPO] {'aggiornamento locale' if _upd and _upd[2] else 'costruzione completa'} "
              f"del grafo: {time.perf_counter() - t_build:.2f} s")
        if getattr(self.prm_graph, 'live_veto_last', 0):
            print(f"[ARCHI] {self.prm_graph.live_veto_last} archi scartati subito dalla scansione dal "
                  f"vivo (passano a meno di {prm_graph_module.LIVE_PREFILTER_OD_M:.2f} m da un "
                  f"ostacolo appena visto), prima della pendenza.")
        # Un tentativo nuovo: le trappole del tentativo precedente non valgono piu' (vedi
        # _trap_backtrack). Le penalita' sono uno strato separato dagli archi, quindi
        # sopravvivono alle ricostruzioni DENTRO il tentativo.
        self.prm_graph.clear_penalties_for_attempt()

    def _obiettivo_e_percorso(self, scan):
        """Nodo del robot, punto obiettivo nella cella, briciola di partenza e primo percorso.
        FALLITO se nella cella non c'e' nessun punto obiettivo valido."""
        pts, cells_obstacle_dist, valid_values = scan.pts, scan.od, scan.valid
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size
        terrain_real, obstacle_mask = scan.terrain, scan.obstacle_mask
        grad_values, rough_values = scan.grad, scan.rough
        save_path = os.path.join(self.mission_folder,
                                 f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}.png") if self.mission_folder else None

        # Solo celle visitate + cella del robot + cella obiettivo (vedi PLAN_ONLY_EXPLORED_CELLS).
        mission_graph.restrict_planning_to_explored_cells(self.prm_graph, self.env, self.target_row, self.target_col,
                                             (self.robot_x, self.robot_y))

        # --- Nodi di partenza e di arrivo: nodi ESISTENTI del grafo ---------------------------
        # add_stop_node aggancia la posizione al nodo adatto invece di crearne uno. Il nodo del
        # robot serve PRIMA della scelta dell'obiettivo: si sceglie fra i punti che il grafo
        # collega al robot (vedi find_best_point_in_cell, criterio 2).
        self.robot_node_id = self.prm_graph.add_stop_node(
            self.robot_x, self.robot_y, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
            label="partenza tentativo")
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)


        # Ricerca del punto target e pianificazione
        self.target_x, self.target_y, valid_samples, rejected_samples = find_best_point_in_cell(
            self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, pts, cells_obstacle_dist, self.sampler,
            global_map=self.global_map, reachable=self._reachable_from_robot)

        # L'obiettivo di questo tentativo, per il recupero subito dopo un fallimento
        # (vedi RECOVERY_SAME_GOAL_M nel ciclo principale).
        self.env._last_attempt_goal = (None if self.target_x is None or self.target_y is None
                                  else (float(self.target_x), float(self.target_y)))
        if self.target_x is None or self.target_y is None:
            print(f"[FAIL] Nessun obiettivo valido nella cella ({self.target_row},{self.target_col}) dalla posizione attuale")
            visualize_grid_with_candidates(
                pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': rejected_samples, 'valid': valid_samples},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=save_path, prm_graph=self.prm_graph, chosen_path=None,
                cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
                grad_values=grad_values, rough_values=rough_values
            )
            return FALLITO

        # Briciola GraphNav del punto di partenza del tentativo: e' quello a cui serve poter
        # tornare se la cella si rivela un vicolo cieco.
        navGraphUtils.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client, self.robot_node_id,
                                 self.target_row, self.target_col, force=True)
        # L'obiettivo e' il centro o un nodo del reticolo (lo sceglie find_best_point_in_cell fra
        # i punti del campionatore).
        self.goal_id = self.prm_graph.add_stop_node(
            self.target_x, self.target_y, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"obiettivo cella ({self.target_row},{self.target_col})", is_robot=False)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
        path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)

        if path_ids is None:
            print(
                f"[GRAFO DISCONNESSO] Nessun percorso nel PRM dal nodo {self.robot_node_id} al nodo "
                f"{self.goal_id}: i due nodi stanno in componenti separate del grafo. Non e' un "
                f"errore di Dijkstra -- vuol dire che tutti gli archi di collegamento sono "
                f"stati vetati (occupazione o pendenza). Vedi i conteggi in [PRM MISSION SUMMARY].")
            print(f" -> Coordinate partenza robot: ({self.robot_x:.2f}, {self.robot_y:.2f})")
            print(f" -> Coordinate target cella: ({self.target_x:.2f}, {self.target_y:.2f})")


        self.full_path_coords = None
        self.path_waypoints = []

        if path_ids is not None:
            self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in path_ids]
            self.path_waypoints = self._waypoints_from_ids(path_ids)
        else:
            self.path_waypoints = None

        visualize_grid_with_candidates(
            pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
            robot_x=self.robot_x, robot_y=self.robot_y,
            candidates={'rejected': rejected_samples, 'valid': valid_samples},
            chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
            save_path=save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
            cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
            grad_values=grad_values, rough_values=rough_values
        )
        return None

    def _azzera_contatori(self):
        """Contatori e memorie del tentativo, prima del primo giro."""
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
        self.replan_counter = 0
        self.need_refresh = True        # primo giro del tentativo: vedi REFRESH A EVENTI
        self.no_progress = 0
        self.block_replans = 0
        self.loop_iter = 0
        self.rotation_fails = {}     # tratto -> quante volte la rotazione verso di esso e' stata rifiutata
        self.rotation_discards = 0   # scarti per rifiuto di rotazione in QUESTO tentativo
        self.best_goal_dist = float('inf')   # per azzerare i rifiuti solo sui PROGRESSI
        self.idle_loops = 0          # giri consecutivi senza spostarsi (vedi MAX_IDLE_LOOPS)
        self.last_xy = None          # posizione al giro precedente, per contare i giri a vuoto
        self.rotation_fail_xy = {}   # tratto -> posizione del robot all'ultimo rifiuto
        self.retreats_in_a_row = 0   # arretramenti consecutivi: azzerato da un avanzamento vero
        self.early_replans = 0       # ripianificazioni alla prima cella bloccante (ramo 'wait')
        self.arrival_refusals = 0    # passi rifiutati perche' la posa d'arrivo non ha uscite
        self.subito_tratti = set()   # tratti gia' scartati da [SUBITO] in questo tentativo
        self.goal_repicks = 0        # quante volte si e' riscelto il punto obiettivo
        self.tried_goals = []        # punti obiettivo gia' provati e risultati irraggiungibili
        self.arrived_near_goal = False   # arrivo accettato con il robot appena fuori dalla cella
        self.trap_state = {'n': 0}   # ritorni indietro in QUESTO tentativo, mai azzerato (vedi _trap_backtrack)

    def _reachable_from_robot(self, px, py):
        nid = self.prm_graph.anchor_node(px, py, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
        return nid is not None and self.prm_graph.find_path_dijkstra(self.robot_node_id, nid) is not None

    def _waypoints_from_ids(self, ids):
        """[(node_id, x, y), ...] senza il primo nodo (la posizione del robot)."""
        out = []
        for nid in ids[1:]:
            nx, ny = self.prm_graph.get_node_position(nid)
            out.append((nid, nx, ny))
        return out

    # --- Azioni comuni alle decisioni ----------------------------------------------------
    def _usa_percorso(self, ids):
        """Il percorso diventa ids (dal nodo del robot in poi); la conferma di un blocco
        riparte da zero."""
        self.path_waypoints = self._waypoints_from_ids(ids)
        self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in ids]
        self.no_progress = 0

    def _scarta_tratto(self, a, b, durata):
        """Toglie il tratto a-b dal grafo. durata:
          'ora'       -- fino al prossimo refresh, che lo rimette se i dati dicono che e' libero
                         (vedi prm_graph.mark_edge_blocked_soft);
          'tentativo' -- per il resto del tentativo (penalita' del tentativo);
          'missione'  -- per sempre: blocco confermato da fermo o guasto fisico."""
        if durata == 'missione':
            self.prm_graph.mark_edge_invalid(a, b)
            return
        prm_graph_module.mark_edge_blocked_soft(self.prm_graph, a, b)
        if durata == 'tentativo':
            self.prm_graph.edge_penalty_persistent[(min(a, b), max(a, b))] = self.prm_graph._ATTEMPT_MARK

    def _strada_fatta(self, x0, y0):
        """Il tratto camminato da (x0, y0) alla posizione di adesso, per la ritirata. Restituisce
        la posizione di adesso."""
        ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
        if not hasattr(self.env, '_traveled_arcs'):
            self.env._traveled_arcs = []
        self.env._traveled_arcs.append((x0, y0, ax, ay))
        return ax, ay

    def _sosta(self, x, y, label):
        """La posizione (x, y) diventa il nodo del robot (vedi add_stop_node)."""
        self.robot_node_id = self.prm_graph.add_stop_node(
            x, y, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label=label)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)

    def _ventaglio(self):
        """Le 24 direzioni d'uscita da qui (vedi arcVerification.is_trapped): (trappola?,
        direzioni utilizzabili, totale, testo per il log)."""
        trapped, fan = arcVerification.is_trapped((self.robot_x, self.robot_y), self.robot_yaw, self.snap)
        n_us, fan_txt = arcVerification.describe_escape_fan(fan)
        return trapped, n_us, len(fan), fan_txt

    def _esci_dalla_trappola(self, tratto, messaggio):
        """Nessuna direzione d'uscita: scartare archi non serve, il problema e' il posto. Si
        torna indietro con GraphNav (CONTINUA) o il tentativo finisce (FALLITO)."""
        MISSION_STATS['traps_detected'] += 1
        print(messaggio)
        if self._ritorno(tratto, f"dal nodo {self.robot_node_id}"):
            return CONTINUA
        return FALLITO

    def _ritorno(self, block_arc, label, skip_if_graph_cannot_help=False):
        """Ritorno con GraphNav (vedi _trap_backtrack) invece che a zampe all'indietro: si torna
        di un waypoint, si riprova a pianificare da li', e cosi' via fino a
        GRAPHNAV_MAX_BACKTRACK_HOPS; la ritirata a zampe resta dentro come ultima rete.
        True se si riparte con un percorso nuovo (contatori del giro azzerati), False se il
        tentativo deve finire."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()
        esc = _trap_backtrack(
            self.trap_state, block_arc,
            self.env, self.prm_graph, self.goal_id, self.recordingInterface, self.robot_state_client,
            self.command_client, self.global_map, self.mobility_kwargs, self.pts_up, self.obstacle_mask_updated,
            label=label, skip_if_graph_cannot_help=skip_if_graph_cannot_help)
        if not esc['ok']:
            return False
        self.robot_node_id = esc['robot_node']
        self._usa_percorso(esc['ids'])
        self.early_replans = 0
        self.retreats_in_a_row = 0
        self.rotation_discards = 0
        self.best_goal_dist = float('inf')
        return True

    def _fronte_fermo_sull_obiettivo(self):
        """Il fronte si ferma per un ostacolo (o rugosita', pendenza) a ridosso dell'obiettivo
        (entro REFUSED_GOAL_BLOCK_M) o oltre la fine del percorso, nel prolungamento per il
        muso: nessun percorso verso questo obiettivo puo' passare, il problema e' il punto."""
        if self.frontier['reason'] not in ('ostacolo', 'rugosita', 'pendenza'):
            return False
        sx, sy = self.frontier['stop_xy']
        return (self.frontier['dist'] >= self.frontier['path_len']
                or float(np.hypot(sx - self.target_x, sy - self.target_y)) <= P.REFUSED_GOAL_BLOCK_M)

    def _riscegli_obiettivo(self, perche, motivo_esclusione):
        """
        Esclude l'obiettivo corrente (per la missione, vedi _refuse_goal) e ne sceglie un altro
        nella cella, raggiungibile dal robot. CONTINUA se c'e' un nuovo obiettivo con un
        percorso; None se non ce n'e' (l'obiettivo resta escluso, il chiamante prosegue).
        Al massimo MAX_BLOCK_REPLANS volte per tentativo.
        """
        self.goal_repicks += 1
        self.need_refresh = True          # piano cambiato: vedi REFRESH A EVENTI
        MISSION_STATS['goal_repicks'] += 1
        print(f"[TARGET] {perche} Ne cerco un altro nella cella "
              f"({self.goal_repicks}/{P.MAX_BLOCK_REPLANS}).")
        self.tried_goals.append((float(self.target_x), float(self.target_y)))
        _refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                     motivo_esclusione)
        # Fino a GOAL_REPICK_CANDIDATES candidati: il primo puo' non avere un percorso nel grafo.
        # Se nessuno ce l'ha, obiettivo e percorso tornano quelli di prima (escluso per le scelte
        # future, ma coerente con il percorso): prima restava il NUOVO obiettivo con il percorso
        # VECCHIO, verso il punto appena escluso.
        old = (self.target_x, self.target_y, self.goal_id)
        new_ids = None
        for _k in range(P.GOAL_REPICK_CANDIDATES):
            new_tx, new_ty, _vs, _rs = find_best_point_in_cell(
                self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, self.pts_up, self.cells_obs_up,
                self.sampler, global_map=self.global_map, exclude=self.tried_goals,
                reachable=self._reachable_from_robot)
            if new_tx is None or new_ty is None:
                print("[TARGET] Nessun altro punto valido in questa cella.")
                break
            goal_id = self.prm_graph.add_stop_node(
                new_tx, new_ty, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
                label=f"obiettivo riscelto cella ({self.target_row},{self.target_col})",
                is_robot=False)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, goal_id)
            if new_ids is not None:
                self.target_x, self.target_y, self.goal_id = new_tx, new_ty, goal_id
                break
            print(f"[TARGET] Nessun percorso nel grafo verso ({new_tx:.2f}, {new_ty:.2f}): provo il "
                  f"punto successivo.")
            self.tried_goals.append((float(new_tx), float(new_ty)))
        if new_ids is None:
            self.target_x, self.target_y, self.goal_id = old
            print("[TARGET] Nessun nuovo obiettivo con un percorso: resto sul piano di prima.")
            return None
        self.best_goal_dist = float('inf')    # nuovo obiettivo: progressi da capo
        self._usa_percorso(new_ids)
        print(f"[REPLAN] Verso il nuovo obiettivo ({self.target_x:.2f}, "
              f"{self.target_y:.2f}): {len(self.path_waypoints)} waypoint.")
        return CONTINUA

    def _guardie(self):
        """Limiti del ciclo: troppi giri (MAX_LOOP_ITERATIONS), caduta, troppi giri fermi."""
        self.loop_iter += 1
        self.t_iter = time.perf_counter()
        if self.loop_iter > P.MAX_LOOP_ITERATIONS:
            print(f"[STALLO] {P.MAX_LOOP_ITERATIONS} giri del ciclo di navigazione senza arrivare: "
                  f"interrompo il tentativo sulla cella ({self.target_row},{self.target_col}).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO

        # Batteria sotto soglia: il tentativo si ferma qui, la missione chiude e rientra
        # (vedi Missione.esplora).
        if _rientro_batteria():
            print(f"[BATTERIA] Interrompo il tentativo sulla cella ({self.target_row},"
                  f"{self.target_col}) per il rientro a batteria bassa.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO

        if movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                   "durante il ciclo di navigazione") is not None:
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)
            return FALLITO

        self.robot_x, self.robot_y, self.robot_z, robot_quat = spotUtils.getPosition(self.robot_state_client)

        self.robot_yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                               1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))

        # --- Guardia generica anti-stallo: quanti giri di fila senza spostarsi.
        if self.last_xy is not None and float(np.hypot(self.robot_x - self.last_xy[0], self.robot_y - self.last_xy[1])) < P.IDLE_MOVE_M:
            self.idle_loops += 1
        else:
            self.idle_loops = 0
        # I rifiuti di rotazione si azzerano solo quando il robot si AVVICINA davvero
        # all'obiettivo (20 cm meglio del meglio raggiunto finora), non quando si sposta e
        # basta: non potendo girarsi, il pianificatore puo' portare il robot un nodo piu' in
        # la' dalla parte sbagliata e riprovare da li', e cosi' via, allontanandosi con il
        # contatore sempre a zero (test_e2e_loop.py, scenario S11). Muoversi non e' fare
        # progressi.
        d_goal_now = float(np.hypot(self.robot_x - self.target_x, self.robot_y - self.target_y))
        if d_goal_now < self.best_goal_dist - 0.20:
            self.best_goal_dist = d_goal_now
            self.rotation_discards = 0
        self.last_xy = (self.robot_x, self.robot_y)
        if self.idle_loops >= P.MAX_IDLE_LOOPS:
            MISSION_STATS['idle_loops'] += self.idle_loops
            print(f"[STALLO] {self.idle_loops} giri di ciclo consecutivi senza spostarsi di "
                  f"{P.IDLE_MOVE_M:.2f} m, con il robot in ({self.robot_x:.2f}, {self.robot_y:.2f}). "
                  f"Qualunque sia il motivo, continuare a girare qui non porta da nessuna "
                  f"parte: rinuncio a entrare in ({self.target_row},{self.target_col}) da questo lato.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO

    def _scansione_e_grafo(self):
        """Scansione fresca, mappe globali, dati locali nel PRM ed eventuale
        ripianificazione (REFRESH A EVENTI); si esce dal ciclo se non c'e' piu' percorso."""
        esito = self._scansione_fresca()
        if esito is not None:
            return esito
        touched_nodes = self._aggiorna_grafo()
        if touched_nodes:
            self._ripianifica(touched_nodes)
        if not self.path_waypoints:
            return INTERROMPI

    def _scansione_fresca(self):
        """Scansione, elaborazione, mappe globali, dati locali nel PRM, fotografia per il
        fronte e mappa dello spazio libero. FALLITO se la scansione non arriva."""
        # 1. SCANSIONE FRESCA, prima di tutto il resto
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Rilevamento locale fallito durante il tracking. Arresto precauzionale.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO
        self.t_scan = time.perf_counter()

        self.grids_data_up, main_proto_up = scan.grids_data, scan.proto
        self.lg_proto_up = main_proto_up
        self.pts_up, self.cells_obs_up = scan.pts, scan.od
        self.terrain_up, self.valid_up = scan.terrain_raw, scan.valid
        self.num_x_up, self.num_y_up, self.cell_size_up = scan.num_x, scan.num_y, scan.cell_size
        self.grid_origin_x_up, self.grid_origin_y_up = self.local_grid.grid_origin(scan)

        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-passo")
        self.footprint_mask_2d = scan.footprint_mask
        self.terrain_real_up, self.is_valid_up, self.unwritten_up = scan.terrain, scan.is_valid, scan.unwritten
        self.grad_up, self.rough_up, self.obstacle_mask_updated = scan.grad, scan.rough, scan.obstacle_mask

        self.t_proc = time.perf_counter()
        _gm_core_up = spotGrid.fold_scan_into_global_maps(
            self.global_map, self.global_terrain_map, self.pts_up, self.terrain_real_up, self.obstacle_mask_updated,
            self.is_valid_up, self.cells_obs_up, (self.num_y_up, self.num_x_up), (self.robot_x, self.robot_y, self.robot_yaw))
        self.t_maps = time.perf_counter()

        # 2. Dati locali freschi nel PRM, poi eventuale ripianificazione dal nodo attuale
        grad_up_2d = self.grad_up.reshape((self.num_y_up, self.num_x_up))
        self.rough_up_2d = self.rough_up.reshape((self.num_y_up, self.num_x_up))
        self.terrain_up_2d = self.terrain_real_up.reshape((self.num_y_up, self.num_x_up))

        # I dati locali si iniettano sempre: costa niente (salva i riferimenti agli array) e
        # servono comunque a sample_terrain_between_points per il gait del tratto che si sta
        # per percorrere.
        self.prm_graph.update_local_grid_data(
            grad_values_2d=grad_up_2d,
            rough_values_2d=self.rough_up_2d,
            terrain_values_2d=self.terrain_up_2d,
            valid_2d=spotGrid.slope_usable_mask(self.is_valid_up, self.cells_obs_up, (self.num_y_up, self.num_x_up)),
            grid_origin_x=self.grid_origin_x_up,
            grid_origin_y=self.grid_origin_y_up,
            cell_size=self.cell_size_up
        )

        # Fotografia della scansione appena fatta: serve gia' qui -- allo scarto preliminare
        # degli archi nel refresh (vedi prm_graph.LIVE_PREFILTER_OD_M), a quello dei nodi
        # negli ostacoli -- e poi al fronte sicuro. Va creata UNA volta sola per scansione:
        # make_grid_snapshot aggiorna anche il filtro temporale degli ostacoli.
        self.snap = arcVerification.make_grid_snapshot(
            self.cells_obs_up, self.rough_up, self.is_valid_up, self.valid_up, self.terrain_real_up,
            self.num_x_up, self.num_y_up, self.grid_origin_x_up, self.grid_origin_y_up, self.cell_size_up,
            self.robot_x, self.robot_y, unwritten=self.unwritten_up, robot_yaw=self.robot_yaw)

        # Mappa dello spazio libero: QUI, non insieme a global_map.update, perche' serve la
        # obstacle_distance filtrata che vive nella fotografia (vedi spotGrid.update_clearance_map).
        # Si alimenta solo dal ciclo di navigazione, cosi' tutte le osservazioni passano per
        # lo stesso filtro.
        spotGrid.update_clearance_map(self.global_map, self.pts_up, self.snap, self.is_valid_up, self.unwritten_up,
                              _gm_core_up, (self.robot_x, self.robot_y))

    def _aggiorna_grafo(self):
        """Archi rivalutati sui dati nuovi (solo dopo un movimento o un evento), nodi negli
        ostacoli fuori, rifiuti di rotazione ancora validi. I nodi toccati dal refresh."""
        # Lo scarto preliminare degli archi legge la obstacle_distance FILTRATA
        # (mediana su 3 scansioni + isteresi), la stessa del fronte.
        self.prm_graph.set_live_obstacles(self.snap.obstacle_dist, self.snap.seen, self.snap.origin_x, self.snap.origin_y,
                                     self.snap.cell_size, self.robot_x, self.robot_y, self.robot_yaw)

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
        if self.need_refresh or getattr(self.prm_graph, 'needs_refresh', False):
            self.prm_graph.needs_refresh = False
            touched_nodes = self.prm_graph.refresh_local_edge_weights(
                global_map=self.global_map, edge_safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
            self.need_refresh = False
            MISSION_STATS['prm_refresh'] += 1
            n_live = (getattr(self.prm_graph, 'live_veto_last', 0)
                      + getattr(self.prm_graph, 'live_prune_last', 0))
            MISSION_STATS['live_arc_vetoes'] = MISSION_STATS.get('live_arc_vetoes', 0) + n_live
            if n_live:
                print(f"[ARCHI] {n_live} archi fuori dal grafo per la scansione dal vivo "
                      f"({self.prm_graph.live_veto_last} rivalutati, {self.prm_graph.live_prune_last} "
                      f"oltre il raggio di refresh), prima della pendenza.")
        else:
            # Nessun dato nuovo sotto gli archi: si tengono i pesi del giro precedente e
            # non si ripianifica (il blocco sotto gira solo se touched_nodes non e' vuoto).
            touched_nodes = set()
            MISSION_STATS['prm_refresh_skipped'] += 1


        # Nodi DENTRO gli ostacoli: fuori dal grafo (vedi prune_prm_nodes_in_obstacles).
        pruned_nodes = arcVerification.prune_prm_nodes_in_obstacles(self.prm_graph, self.snap)
        if pruned_nodes:
            MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
            print(f"[NODI] {len(pruned_nodes)} nodi scartati: meno di "
                  f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero dove li vediamo "
                  f"(primi: {pruned_nodes[:8]}).")

        # Archi gia' rifiutati per rotazione DA QUESTO NODO: restano fuori finche' il robot
        # non si sposta. Lo scarto e' soft, quindi refresh_local_edge_weights li rimetterebbe
        # in gioco a ogni giro, e il robot potrebbe restare fermo alternando due archi
        # rifiutati senza mai arrivare alla ritirata. Il robot puo' restare agganciato allo
        # stesso nodo anche dopo essersi spostato, quindi lo scarto vale solo finche' il robot
        # e' dove la rotazione e' stata rifiutata.
        for (from_id, to_id), n_fails in self.rotation_fails.items():
            fx, fy = self.rotation_fail_xy.get((from_id, to_id), (self.robot_x, self.robot_y))
            if (from_id == self.robot_node_id and n_fails > 0
                    and float(np.hypot(self.robot_x - fx, self.robot_y - fy)) <= prm_graph_module.ANCHOR_ON_NODE_M):
                prm_graph_module.mark_edge_blocked_soft(self.prm_graph, from_id, to_id)
        return touched_nodes

    def _ripianifica(self, touched_nodes):
        """Dijkstra dal nodo del robot all'obiettivo sul grafo aggiornato; il percorso cambia
        solo se il nuovo costa almeno il 10% in meno. Senza percorso, nuovo obiettivo se e'
        isolato l'obiettivo (REPICK_ON_REPLAN_FAIL)."""
        current_plan_ids = [self.robot_node_id] + [w[0] for w in self.path_waypoints]
        chosen_path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id,
                                                       current_path_ids=current_plan_ids, margin=0.10)

        if chosen_path_ids is None:
            n_free = self.prm_graph.reachable_count(self.robot_node_id, P.REPLAN_FAIL_ROBOT_FREE_NODES)
            if (P.REPICK_ON_REPLAN_FAIL and self.goal_repicks < P.MAX_BLOCK_REPLANS
                    and n_free >= P.REPLAN_FAIL_ROBOT_FREE_NODES):
                # Dal robot il grafo va ancora da qualche parte: e' l'obiettivo a essere
                # rimasto isolato. Se ne cerca subito un altro con un percorso; se non
                # c'e', resta il piano di prima come sotto.
                MISSION_STATS['replan_fail_repicks'] = MISSION_STATS.get('replan_fail_repicks', 0) + 1
                self._riscegli_obiettivo(
                    f"Ripianificazione fallita: nessun percorso da {self.robot_node_id} all'obiettivo "
                    f"({self.target_x:.2f}, {self.target_y:.2f}) nel grafo aggiornato, mentre "
                    f"dal robot il grafo prosegue: e' isolato l'obiettivo.",
                    "nessun percorso nel grafo aggiornato")
            else:
                print(f"[REPLAN FALLITO] Nessun percorso esistente da {self.robot_node_id} a "
                      f"{self.goal_id} nel grafo aggiornato. Mantengo il percorso precedente: il "
                      f"fronte sicuro decide comunque fin dove si puo' avanzare.")
        elif chosen_path_ids != current_plan_ids:
            self._usa_percorso(chosen_path_ids)
            print(f"[REPLAN] Percorso aggiornato ({len(touched_nodes)} nodi locali rivalutati): "
                  f"{len(self.path_waypoints)} waypoint rimanenti.")

            self.replan_counter += 1
            MISSION_STATS['replan_count'] += 1
            replan_save_path = os.path.join(
                self.mission_folder, f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_replan_{self.replan_counter}.png"
            ) if self.mission_folder else None

            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=replan_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up,
                include_diagnostics=False  # leggero: vista principale e mappa globale
            )
        else:
            print(f"[REPLAN STABILE] Percorso invariato da {self.robot_node_id} a {self.goal_id} "
                  f"({len(touched_nodes)} nodi locali rivalutati): l'alternativa non "
                  f"supera la soglia di stabilita'. Proseguo sul piano corrente.")

    def _fronte_e_decisione(self):
        """Scorciatoia, fronte sicuro sul percorso e decisione (arcVerification.decide_next_move):
        muoversi, nodo raggiunto, arrivati, aspettare o blocco confermato."""
        self.t_prm = time.perf_counter()

        # 3. FRONTE SICURO sulla scansione appena fatta ---------------------------------
        # La scorciatoia rispetta gli archi gia' scartati: vedi shortcut_index.
        k_short = arcVerification.shortcut_index((self.robot_x, self.robot_y), self.path_waypoints, self.snap, self.global_map,
                                 prm=self.prm_graph, robot_node_id=self.robot_node_id)
        if k_short > 0:
            skipped = [w[0] for w in self.path_waypoints[:k_short]]
            self.path_waypoints = self.path_waypoints[k_short:]
            print(f"[SCORCIATOIA] Tratto diretto confermato libero verso il nodo {self.path_waypoints[0][0]}: "
                  f"salto {len(skipped)} nodi intermedi {skipped}.")
            MISSION_STATS['shortcuts'] += 1
        self.polyline = [(self.robot_x, self.robot_y)] + [(x, y) for _, x, y in self.path_waypoints]
        self.frontier = arcVerification.compute_safe_frontier(self.polyline, self.snap)
        print(arcVerification.describe_frontier(self.frontier))

        self.path_version = None
        if self.verification_tracker is not None:
            self.path_version = self.verification_tracker.update_path(self.polyline[1:])

        in_target_cell = (tuple(int(v) for v in self.env.get_cell_from_world(self.robot_x, self.robot_y))
                          == (int(self.target_row), int(self.target_col)))
        # Appena fuori dalla cella vale come dentro, per l'arrivo per tolleranza (vedi
        # ARRIVAL_OUTSIDE_CELL_M). Il 22-10 16:04: "Mancano 0.06 m ... ancora fuori dalla
        # cella (2,7)", con il robot fuori di 0.01 m.
        robot_border = environmentMap.cell_border_distance(self.env, self.target_row, self.target_col, self.robot_x, self.robot_y)
        near_goal = (not in_target_cell and robot_border is not None
                     and -robot_border <= P.ARRIVAL_OUTSIDE_CELL_M)
        self.decision = arcVerification.decide_next_move((self.robot_x, self.robot_y), self.path_waypoints, self.frontier, self.no_progress,
                                    in_target_cell=in_target_cell or near_goal)
        if self.decision.get('final'):
            MISSION_STATS['final_steps'] = MISSION_STATS.get('final_steps', 0) + 1
            print(f"[PASSO FINALE] Il fronte permette solo {self.decision['dist']:.2f} m, sotto il "
                  f"passo minimo di {P.MIN_PARTIAL_STEP_M:.2f} m, ma bastano: dopo il passo "
                  f"l'obiettivo e' a {self.frontier['path_len'] - self.decision['dist']:.2f} m, entro "
                  f"la tolleranza d'arrivo di {P.GOAL_REACHED_TOLERANCE_M:.2f} m.")
        if self.decision['kind'] == 'arrived' and near_goal:
            self.arrived_near_goal = True
            MISSION_STATS['arrivals_near_goal'] += 1
            print(f"[OBIETTIVO] Il robot e' fuori dalla cella ({self.target_row},{self.target_col}) di "
                  f"{-robot_border:.2f} m, a {self.frontier['path_len']:.2f} m dall'obiettivo: vale come "
                  f"arrivo (fuori al massimo di {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m, il muso e' dentro).")
        if (self.decision['kind'] in ('wait', 'blocked') and not in_target_cell and not near_goal
                and self.frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M):
            print(f"[OBIETTIVO] Mancano {self.frontier['path_len']:.2f} m, entro la tolleranza di "
                  f"{P.GOAL_REACHED_TOLERANCE_M:.2f} m, ma il robot e' ancora fuori dalla cella "
                  f"({self.target_row},{self.target_col}): non vale come arrivo, proseguo come per un "
                  f"fronte bloccato.")
        self.next_node_id, next_x, next_y = self.path_waypoints[0]
        self.keep_heading = False     # sempre False (niente traverso): resta nei dati salvati e in navigate_to

        # Il pacchetto si salva qui, prima delle fasi che possono chiudere il giro (rotazione,
        # attesa, blocco): altrimenti quelle scansioni non finirebbero su disco.
        self._salva_pacchetto_scansione()

    def _salva_pacchetto_scansione(self):
        """Tutto quello che serve a rivedere questa scansione dopo (scans/*.npz): layer della
        griglia, fotografia filtrata, percorso, fronte e decisione. Vedi SAVE_SCAN_BUNDLES."""
        # Un rifiuto di rotazione si ricalcola offline da obstacle_distance, robot_xyz,
        # robot_yaw e decision_target.
        if P.SAVE_SCAN_BUNDLES and self.mission_folder is not None:
            gf = self.local_grid.last_ground_filter
            shape_up = (self.num_y_up, self.num_x_up)
            extra = {}
            if P.SAVE_EXTRA_LAYERS:
                t_x = time.perf_counter()
                for name, lay in self.local_grid.extra_layers(P.SAVE_EXTRA_LAYERS).items():
                    extra[name] = lay['values']
                    extra[f'{name}_origin'] = lay['origin']
                    extra[f'{name}_cell'] = lay['cell']
                timing_add('scansione', time.perf_counter() - t_x)
            save_npz_async(
                os.path.join(self.mission_folder, "scans",
                             f"scan_it{self.iteration:03d}_cell{self.target_row}_{self.target_col}_{self.loop_iter:03d}.npz"),
                time=time.time(), iteration=self.iteration, cell=np.array([self.target_row, self.target_col]),
                loop_iter=self.loop_iter,
                terrain_raw=np.asarray(self.terrain_up, dtype=np.float32).reshape(shape_up),
                terrain_valid_raw=np.asarray(self.valid_up, dtype=np.float32).reshape(shape_up),
                obstacle_distance=np.asarray(self.cells_obs_up, dtype=np.float32).reshape(shape_up),
                # Il layer su cui il fronte DECIDE (mediana sulle ultime letture + isteresi,
                # vedi arcVerification._ObstacleHistory) e la maschera del latch. Senza questi
                # due un rigioco offline non puo' riprodurre i blocchi che nascono dalle
                # letture del thread durante il movimento. Stesso orientamento degli altri:
                # (num_y, num_x), riga = y.
                obstacle_distance_filtered=np.asarray(self.snap.obstacle_dist, dtype=np.float32),
                blocked_latch=(np.zeros(shape_up, bool) if self.snap.blocked_latch is None
                               else np.asarray(self.snap.blocked_latch, dtype=bool)),
                terrain_corrected=np.asarray(self.terrain_real_up, dtype=np.float32).reshape(shape_up),
                is_valid=np.asarray(self.is_valid_up, dtype=bool).reshape(shape_up),
                roughness=np.asarray(self.rough_up, dtype=np.float32).reshape(shape_up),
                gradient=np.asarray(self.grad_up, dtype=np.float32).reshape(shape_up),
                obstacle_mask=np.asarray(self.obstacle_mask_updated, dtype=np.int8).reshape(shape_up),
                footprint_mask=(np.zeros(shape_up, bool) if self.footprint_mask_2d is None
                                else np.asarray(self.footprint_mask_2d, dtype=bool).reshape(shape_up)),
                grid_origin=np.array([self.grid_origin_x_up, self.grid_origin_y_up]), cell_size=self.cell_size_up,
                robot_xyz=np.array([self.robot_x, self.robot_y, self.robot_z]), robot_yaw=self.robot_yaw,
                ground_z=np.nan if gf.get('ground_z') is None else gf['ground_z'],
                ground_n_above=gf.get('n_above', 0),
                mission_z0=mission_z0(),
                terrain_raw_zero=np.nan if spotGrid.raw_zero(self.grids_data_up) is None else spotGrid.raw_zero(self.grids_data_up),
                terrain_scale=np.nan if not spotGrid.raw_scale(self.grids_data_up) else spotGrid.raw_scale(self.grids_data_up),
                unwritten=np.asarray(self.unwritten_up, dtype=bool).reshape(shape_up),
                polyline=np.array(self.polyline), path_node_ids=np.array([w[0] for w in self.path_waypoints]),
                robot_node_id=self.robot_node_id, goal_xy=np.array([self.target_x, self.target_y]),
                frontier_dist=self.frontier['dist'], frontier_reason=self.frontier['reason'],
                frontier_stop_xy=np.array(self.frontier['stop_xy']),
                frontier_value=np.nan if self.frontier['value'] is None else self.frontier['value'],
                frontier_od_robot=self.frontier['od_robot'], frontier_half_along=self.frontier['half_along'],
                frontier_clearance=self.frontier['clearance_nominal'], frontier_path_len=self.frontier['path_len'],
                allowed_advance=arcVerification.allowed_advance(self.frontier),
                keep_heading=self.keep_heading, no_progress=self.no_progress,
                **extra,
                decision_kind=self.decision['kind'],
                decision_target=np.array(self.decision.get('target', (np.nan, np.nan)), dtype=np.float64))
            MISSION_STATS['scan_bundles'] += 1

    def _rotazione(self):
        """Se il passo deciso richiede di girarsi dove non si puo': avanzare nel cono,
        arretrare dritti, riconoscere la trappola o scartare il tratto."""
        # 4. ROTAZIONE -------------------------------------------------------------------
        # Il muso punta SEMPRE il punto da raggiungere. Se la rotazione non e' possibile NON
        # si cammina di traverso (vedi ROTAZIONE E ARRETRAMENTO in parametri.py). Nell'ordine:
        # avanzare dentro il cono di rotazione possibile, arretrare dritti se da li' la
        # rotazione diventa possibile, riconoscere una trappola, scartare il tratto.
        if self.decision['kind'] == 'move':
            rot = arcVerification.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), self.decision['target'],
                                self.frontier['od_robot'], snap=self.snap)
            if not rot['turn']:
                MISSION_STATS['rotation_refusals'] += 1
                edge_key = (self.robot_node_id, self.next_node_id)
                self.rotation_fails[edge_key] = self.rotation_fails.get(edge_key, 0) + 1
                self.rotation_fail_xy[edge_key] = (self.robot_x, self.robot_y)
                margin_txt = (f"margine {rot['margin']:+.2f} m sulle celle spazzate"
                              if rot['margin'] is not None else f"motivo: {rot['why']}")
                print(f"[ROTAZIONE] Servono {rot['dyaw_deg']:+.0f} gradi verso il nodo "
                      f"{self.next_node_id}, ma il corpo toccherebbe ({margin_txt}; "
                      f"obstacle_distance sotto il robot {self.frontier['od_robot']:.2f} m). "
                      f"Non cammino di traverso: tentativo "
                      f"{self.rotation_fails[edge_key]}/{P.MAX_ROTATION_FAILS_SAME_EDGE}.")

                esito = self._avanza_nel_cono(rot)
                if esito is None:
                    esito = self._arretra_per_ruotare(rot, edge_key)
                if esito is not None:
                    return esito

                # Ne' il cono ne' l'arretramento: e' una trappola o solo un arco sbagliato?
                # Se e' una trappola non ha senso scartare archi uno dopo l'altro: si torna
                # indietro subito.
                trapped_rot, n_us_r, n_fan_r, fan_txt_r = self._ventaglio()
                if trapped_rot:
                    return self._esci_dalla_trappola(
                        (self.robot_node_id, self.next_node_id),
                        f"[TRAPPOLA] Non posso girarmi verso {self.next_node_id} e nessuna delle "
                        f"{n_fan_r} direzioni del giro e' insieme percorribile e "
                        f"raggiungibile girandosi. Le piu' larghe: {fan_txt_r}. Ripercorro il "
                        f"tratto da cui sono venuto.")
                print(f"[ROTAZIONE] Direzioni utilizzabili da qui: {n_us_r}/{n_fan_r} "
                      f"({fan_txt_r}).")
                # Limite per TENTATIVO, non per coppia di nodi (vedi MAX_ROTATION_DISCARDS).
                self.rotation_discards += 1
                if self.rotation_discards > P.MAX_ROTATION_DISCARDS:
                    print(f"[ROTAZIONE] Gia' {self.rotation_discards - 1} tratti scartati in questo "
                          f"tentativo perche' la rotazione era impossibile, e sono sempre qui: "
                          f"non e' l'arco, e' il posto. Ripercorro il tratto da cui sono venuto.")
                    if self._ritorno((self.robot_node_id, self.next_node_id),
                                     f"dal nodo {self.robot_node_id} (troppi rifiuti di rotazione)"):
                        return CONTINUA
                    return FALLITO
                print(f"[ROTAZIONE] Scarto il tratto {self.robot_node_id}->{self.next_node_id}: "
                      f"richiede di girarsi dove non c'e' spazio DA QUI. Ripianifico.")
                # Scarto NON permanente: un rifiuto di rotazione dipende dalla POSA del robot,
                # non dall'arco -- lo stesso arco, affrontato da un altro punto o con un altro
                # muso, puo' essere percorribilissimo. Permanente resta solo cio' che e'
                # confermato da tre scansioni ferme o da un fallimento fisico.
                self._scarta_tratto(self.robot_node_id, self.next_node_id, 'ora')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None:
                    self._usa_percorso(new_ids)
                    return CONTINUA
                print("[ROTAZIONE] Nessuna alternativa nel grafo dal punto in cui sono. "
                      "Mi stacco arretrando e rimando la cella.")
                if self._ritorno((self.robot_node_id, self.next_node_id),
                                 f"dal nodo {self.robot_node_id}", skip_if_graph_cannot_help=True):
                    return CONTINUA
                return FALLITO

    def _cambia_obiettivo_irraggiungibile(self):
        """Il fronte non lascia avanzare: se e' colpa dell'obiettivo (dentro un ostacolo, o fronte
        fermo a ridosso di lui) se ne sceglie un altro. L'esito, o None."""
        # Prima di insistere: l'obiettivo e' ancora raggiungibile sui dati FRESCHI? Il
        # punto viene scelto all'inizio del tentativo, da metri di distanza, quando la
        # cella e' ancora tutta ignota; arrivandoci puo' risultare a ridosso di un muro,
        # e allora nessun numero di scansioni lo rendera' raggiungibile.
        if self.goal_repicks < P.MAX_BLOCK_REPLANS:
            if not arcVerification.goal_still_reachable(self.snap, self.target_x, self.target_y):
                r_g, c_g, in_g = arcVerification._cells(
                    self.snap, np.array([float(self.target_x)]), np.array([float(self.target_y)]))
                od_goal = float(self.snap.obstacle_dist[r_g[0], c_g[0]]) if in_g[0] else float('nan')
                esito = self._riscegli_obiettivo(
                    f"L'obiettivo ({self.target_x:.2f}, {self.target_y:.2f}) ha obstacle_distance "
                    f"{od_goal:.2f} m, sotto i {spotGrid.FRONTIER_CLEARANCE_M:.2f} m che il fronte "
                    f"pretende: non e' raggiungibile da nessuna direzione.",
                    f"spazio libero dal vivo {od_goal:.2f} m")
                if esito is not None:
                    return esito
            elif self._fronte_fermo_sull_obiettivo():
                sx_g, sy_g = self.frontier['stop_xy']
                d_g = float(np.hypot(sx_g - self.target_x, sy_g - self.target_y))
                oltre = self.frontier['dist'] >= self.frontier['path_len']
                esito = self._riscegli_obiettivo(
                    f"Il fronte si ferma per {self.frontier['reason']} in ({sx_g:.2f}, {sy_g:.2f}), a "
                    f"{d_g:.2f} m dall'obiettivo ({self.target_x:.2f}, {self.target_y:.2f})"
                    + (", oltre la fine del percorso, dove andrebbe il muso" if oltre else "")
                    + ": e' l'obiettivo a non andare, non la strada.",
                    f"il fronte si ferma a {d_g:.2f} m da lui ({self.frontier['reason']})")
                if esito is not None:
                    return esito
        return None

    def _ripianifica_subito(self):
        """[SUBITO]: alla prima cella bloccante il tratto si scarta e si cerca un'altra strada,
        senza aspettare la conferma da fermo. L'esito, o None se si resta ad aspettare."""
        # RIPIANIFICA SUBITO, alla PRIMA cella bloccante, invece di restare fermo tre
        # scansioni sullo stesso arco. Il rumore di obstacle_distance e' gia' filtrato
        # (mediana + isteresi in arcVerification), quindi una cella bloccante e'
        # un'informazione da usare: l'arco si scarta e si cerca subito un percorso che
        # passi da un'altra parte. Le scansioni da fermo restano come ultima rete, per
        # dichiarare la cella irraggiungibile.
        if (self.early_replans < P.MAX_EARLY_REPLANS
                and self.frontier['reason'] in ('ostacolo', 'rugosita', 'pendenza')
                and self.path_waypoints):
            poly_now = [(self.robot_x, self.robot_y)] + [(x, y) for _, x, y in self.path_waypoints]
            _, seg_blk = arcVerification.point_along(poly_now, self.frontier['dist'])
            seg_blk = int(seg_blk)
            ea = self.robot_node_id if seg_blk == 0 else self.path_waypoints[seg_blk - 1][0]
            eb = self.path_waypoints[min(seg_blk, len(self.path_waypoints) - 1)][0]
            sx_b, sy_b = self.frontier['stop_xy']

            # Prima di scartare l'ennesimo arco: sono in trappola? Se tutti gli archi del
            # nodo passano accanto allo stesso ostacolo, il blocco e' un LUOGO, non un
            # arco, e scartarli uno per uno non serve. Il ventaglio lo vede in una scansione.
            trapped, n_us, n_fan, fan_txt = self._ventaglio()
            if trapped:
                return self._esci_dalla_trappola(
                    (ea, eb),
                    f"[TRAPPOLA] Nessuna delle {n_fan} direzioni del giro e' insieme "
                    f"percorribile e raggiungibile girandosi. Le piu' larghe: {fan_txt}. "
                    f"Non ho dove andare ne' come girarmi: ripercorro il tratto da cui "
                    f"sono venuto, senza scartare altri archi.")

            self.early_replans += 1
            MISSION_STATS['early_replans'] += 1
            print(f"[SUBITO] {self.frontier['reason']} in ({sx_b:.2f}, {sy_b:.2f}) sul tratto "
                  f"{ea}->{eb}: lo scarto adesso e cerco un altro percorso, senza "
                  f"aspettare la conferma da fermo ({self.early_replans}/{P.MAX_EARLY_REPLANS}). "
                  f"Direzioni utilizzabili da qui: {n_us}/{n_fan}.")
            # Scarto NON permanente: dopo una sola scansione l'arco va tolto dal piano,
            # non condannato per la missione (vedi mark_edge_blocked_soft).
            self._scarta_tratto(ea, eb, 'ora')
            # Ma se e' la SECONDA volta sullo stesso tratto, lo scarto "per ora" non e'
            # bastato: il refresh lo ha rimesso nel grafo (lo scarto preliminare guarda una
            # scansione a 0.25 m, il fronte la mediana di tre con l'isteresi fino a 0.29),
            # e il piano e' tornato quello di prima. Fuori dal piano per il resto del
            # tentativo, come i ritorni e [ARRIVO]. Il 28-10 14:23, 8 [SUBITO] su 14 erano
            # ripetizioni dello stesso tratto (197->569 quattro volte di fila).
            key_s = (min(ea, eb), max(ea, eb))
            if key_s in self.subito_tratti:
                self._scarta_tratto(ea, eb, 'tentativo')
                MISSION_STATS['early_replans_repeated'] = MISSION_STATS.get('early_replans_repeated', 0) + 1
                print(f"[SUBITO] Il tratto {ea}->{eb} era gia' stato scartato in questo tentativo e "
                      f"il piano ci e' tornato: fuori dal piano per il resto del tentativo.")
            self.subito_tratti.add(key_s)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None and len(new_ids) > 1:
                self._usa_percorso(new_ids)
                print(f"[SUBITO] Percorso alternativo con {len(self.path_waypoints)} waypoint, "
                      f"dal punto in cui sono.")
                return CONTINUA
            print("[SUBITO] Il grafo non offre alternative da qui: resto sul percorso e "
                  "lascio decidere il fronte.")
        return None

    def _avanza_nel_cono(self, rot):
        """Rotazione impossibile: si avanza dentro il cono di rotazione concesso, se avvicina
        al percorso (CONTINUA), altrimenti None."""
        # --- 4a. PRIMA DI TUTTO: posso ANDARE AVANTI dentro il cono di rotazione che
        # mi e' concesso? Spesso andare dritti, o quasi, avvicina all'obiettivo piu'
        # di qualunque arretramento (vedi plan_cone_move).
        cone_goal, _ = arcVerification.point_along(self.polyline, P.CONE_PATH_LOOKAHEAD_M)
        cone = arcVerification.plan_cone_move((self.robot_x, self.robot_y), self.robot_yaw, self.snap, cone_goal)
        if cone is not None:
            print(f"[CONO] Non posso girarmi di {rot['dyaw_deg']:+.0f} gradi, ma ruotando di "
                  f"{cone['dyaw_deg']:+.0f} posso avanzare {cone['dist']:.2f} m "
                  f"({cone['reason']}) e avvicinarmi di {cone['gain']:.2f} m al percorso "
                  f"({cone_goal[0]:.2f}, {cone_goal[1]:.2f}, {P.CONE_PATH_LOOKAHEAD_M:.1f} m "
                  f"avanti lungo il percorso). Vado, invece di arretrare.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['cone_moves'] += 1
            cx, cy = cone['target']
            self.need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
            with TrajectoryLog(self.robot_state_client, 'cono', (cx, cy)):
                ok_c, dist_c, _ = movements.navigate_to(
                    cx, cy, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                    get_a_tform_b(self.lg_proto_up.local_grid.transforms_snapshot,
                                  VISION_FRAME_NAME, BODY_FRAME_NAME),
                    should_abort=None, keep_heading=False, **self.mobility_kwargs)
            if dist_c > 0.05:
                self._sosta(*self._strada_fatta(self.robot_x, self.robot_y), "sosta (avanzamento nel cono)")
                self.retreats_in_a_row = 0      # ha fatto strada vera: la catena si rompe
            else:
                print(f"[CONO] Non sono riuscito ad avanzare (percorsi {dist_c:.2f} m).")
            self.no_progress = 0
            return CONTINUA
        return None

    def _arretra_per_ruotare(self, rot, edge_key):
        """Rotazione impossibile: si arretra dritti, se da li' la rotazione diventa possibile
        (CONTINUA), altrimenti None."""
        # --- 4b. Arretrare, ma SOLO se serve a qualcosa (vedi retreat_would_help).
        back = arcVerification.plan_straight_retreat((self.robot_x, self.robot_y), self.robot_yaw, self.snap)
        helps, margin_back = (False, None)
        if back is not None:
            helps, margin_back = arcVerification.retreat_would_help(
                self.snap, (back[0], back[1]), self.robot_yaw, rot['dyaw_deg'])
            if not helps:
                MISSION_STATS['retreats_suppressed'] += 1
                mb = "" if margin_back is None else f" (margine {margin_back:+.2f} m da li')"
                print(f"[ARRETRAMENTO] Potrei arretrare di {back[2]:.2f} m, ma da li' la "
                      f"rotazione di {rot['dyaw_deg']:+.0f} gradi resterebbe impossibile{mb}: "
                      f"non lo faccio, sposterei il problema di 60 cm.")
        if (back is not None and helps
                and self.retreats_in_a_row < P.MAX_STRAIGHT_RETREATS_IN_A_ROW
                and self.rotation_fails[edge_key] < P.MAX_ROTATION_FAILS_SAME_EDGE):
            bx, by, bdist = back
            self.retreats_in_a_row += 1
            print(f"[ARRETRAMENTO] Indietro dritto di {bdist:.2f} m verso "
                  f"({bx:.2f}, {by:.2f}), senza girarmi, per fare spazio alla rotazione "
                  f"({self.retreats_in_a_row}/{P.MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['straight_retreats'] += 1
            self.need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
            with TrajectoryLog(self.robot_state_client, 'arretramento', (bx, by)):
                ok_back, dist_back = movements.move_to_world_point_without_turning(
                    bx, by, "vision", self.command_client, self.robot_state_client, **self.mobility_kwargs)
            if dist_back > 0.05:
                self._sosta(*self._strada_fatta(self.robot_x, self.robot_y), "sosta (arretramento per ruotare)")
            else:
                print(f"[ARRETRAMENTO] Non sono riuscito ad arretrare "
                      f"(percorsi {dist_back:.2f} m).")
            self.no_progress = 0
            return CONTINUA

        if back is None:
            print("[ARRETRAMENTO] Non c'e' spazio per arretrare dritto.")
        elif helps and self.retreats_in_a_row >= P.MAX_STRAIGHT_RETREATS_IN_A_ROW:
            print(f"[ARRETRAMENTO] Gia' {self.retreats_in_a_row} arretramenti di fila senza "
                  f"combinare niente: smetto di arretrare.")
        return None

    def _salva_dati_del_blocco(self):
        """Scansione e percorso del blocco confermato, per guardarlo dopo (BLOCKED_n.npz)."""
        if self.mission_folder is not None:
            try:
                npz_blocked = os.path.join(
                    self.mission_folder,
                    f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED_{self.block_replans + 1}.npz")
                np.savez_compressed(
                    npz_blocked,
                    obstacle_distance=self.snap.obstacle_dist, rough_veto=self.snap.rough_veto,
                    seen=self.snap.seen, terrain=self.snap.terrain,
                    terrain_raw=np.asarray(self.terrain_up).reshape((self.num_y_up, self.num_x_up)),
                    terrain_valid_raw=np.asarray(self.valid_up).reshape((self.num_y_up, self.num_x_up)),
                    obstacle_mask=np.asarray(self.obstacle_mask_updated).reshape((self.num_y_up, self.num_x_up)),
                    roughness=self.rough_up_2d,
                    grid_origin_x=self.snap.origin_x, grid_origin_y=self.snap.origin_y, cell_size=self.snap.cell_size,
                    robot_xy=np.array([self.robot_x, self.robot_y]), robot_yaw=self.robot_yaw,
                    polyline=np.array(self.polyline), frontier_dist=self.frontier['dist'],
                    frontier_reason=self.frontier['reason'], frontier_stop_xy=np.array(self.frontier['stop_xy']),
                    ground_z=np.nan if self.local_grid.last_ground_filter.get('ground_z') is None
                    else self.local_grid.last_ground_filter['ground_z'])
                print(f"[DATA SAVE] Dati del blocco salvati in {os.path.basename(npz_blocked)}")
            except Exception as e:
                print(f"[DATA SAVE] Impossibile salvare i dati del blocco: {e}")

    def _tempi(self):
        """Tempi del giro, per [TEMPO] e il riepilogo."""
        t_front = time.perf_counter()

        timing_add('scansione', self.t_scan - self.t_iter)
        timing_add('elaborazione griglia', self.t_proc - self.t_scan)
        timing_add('mappe globali', self.t_maps - self.t_proc)
        timing_add('PRM e ripianificazione', self.t_prm - self.t_maps)
        timing_add('fronte e decisione', t_front - self.t_prm)
        print(f"[TEMPO] scansione {self.t_scan - self.t_iter:.2f} s | elaborazione {self.t_proc - self.t_scan:.2f} | "
              f"mappe {self.t_maps - self.t_proc:.2f} | PRM {self.t_prm - self.t_maps:.2f} | fronte {t_front - self.t_prm:.2f} "
              f"| decisione: {self.decision['kind']}")

    def _nodo_o_arrivo(self):
        """Nodo del percorso raggiunto, oppure arrivo accettato prima della fine."""
        if self.decision['kind'] == 'reached_node':
            self.robot_node_id = self.next_node_id
            self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
            self.path_waypoints.pop(0)
            self.no_progress = 0
            return CONTINUA

        if self.decision['kind'] == 'arrived':
            print(f"[OBIETTIVO] Il fronte non permette gli ultimi {self.frontier['path_len']:.2f} m "
                  f"({self.frontier['reason']}), entro la tolleranza di {P.GOAL_REACHED_TOLERANCE_M:.2f} m: "
                  f"considero raggiunto l'obiettivo.")
            return INTERROMPI

    def _attesa(self):
        """Il fronte non lascia avanzare: obiettivo ancora raggiungibile? ripianificare subito
        (SUBITO) o aspettare la conferma da fermo."""
        if self.decision['kind'] == 'wait':
            esito = self._cambia_obiettivo_irraggiungibile()
            if esito is None:
                esito = self._ripianifica_subito()
            if esito is not None:
                return esito

            self.no_progress += 1
            print(f"[FRONTE] Non posso avanzare nemmeno di {P.MIN_PARTIAL_STEP_M:.2f} m: riguardo "
                  f"da fermo ({self.no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni).")
            time.sleep(P.FRONTIER_RESCAN_WAIT_S)
            return CONTINUA

    def _blocco(self):
        """Blocco confermato da piu' scansioni ferme: arco scartato, ripianificazione o ritorno."""
        if self.decision['kind'] == 'blocked':
            k = self.decision['segment']
            edge_a = self.robot_node_id if k == 0 else self.path_waypoints[k - 1][0]
            edge_b = self.path_waypoints[min(k, len(self.path_waypoints) - 1)][0]
            sx, sy = self.frontier['stop_xy']
            print(f"[BLOCCO] Confermato da vicino: {P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni consecutive "
                  f"senza poter avanzare. Motivo: {self.frontier['reason']} in ({sx:.2f}, {sy:.2f}), "
                  f"sul tratto {edge_a}->{edge_b}.")
            MISSION_STATS['frontier_blocks'] += 1
            # Bloccato a ridosso dell'obiettivo (o oltre): e' l'obiettivo a non andare, non la
            # strada. Si esclude e, se si puo', se ne sceglie un altro (sotto, dopo la trappola)
            # invece di scartare il tratto e ripianificare verso lo stesso obiettivo.
            goal_blocked = self._fronte_fermo_sull_obiettivo()
            d_goal = float(np.hypot(sx - self.target_x, sy - self.target_y))
            why_goal = f"blocco confermato a {d_goal:.2f} m da lui ({self.frontier['reason']})"
            if goal_blocked:
                _refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                             why_goal)

            self._salva_dati_del_blocco()

            # Trappola? Se non c'e' nessuna direzione utilizzabile, ripianificare
            # e' inutile per costruzione: il problema non e' l'arco, e' il posto.
            trapped_blk, n_us_b, n_fan_b, fan_txt_b = self._ventaglio()
            print(f"[BLOCCO] Direzioni utilizzabili da qui: {n_us_b}/{n_fan_b} ({fan_txt_b}).")
            if trapped_blk:
                return self._esci_dalla_trappola(
                    (edge_a, edge_b), "[TRAPPOLA] Nessuna via d'uscita girandomi: non scarto altri "
                                      "archi, ripercorro il tratto da cui sono venuto.")

            if goal_blocked and self.goal_repicks < P.MAX_BLOCK_REPLANS:
                esito = self._riscegli_obiettivo(
                    f"Blocco a {d_goal:.2f} m dall'obiettivo ({self.target_x:.2f}, "
                    f"{self.target_y:.2f}): e' l'obiettivo a non andare, non il tratto {edge_a}->{edge_b}.",
                    why_goal)
                if esito is not None:
                    return esito

            # Qui il blocco e' confermato da FRONTIER_BLOCK_CONFIRM_SCANS scansioni ferme:
            # l'invalidazione resta permanente (a differenza di [SUBITO], vedi
            # mark_edge_blocked_soft). Eccezione: sul primo tratto, se il robot
            # non e' sul nodo a cui e' agganciato, il tratto bloccato e' robot->nodo, non
            # l'arco del grafo: si scarta l'arco solo per ora.
            sul_nodo = k > 0 or mission_graph.robot_on_node(self.prm_graph, edge_a, self.robot_x, self.robot_y)
            self._scarta_tratto(edge_a, edge_b, 'missione' if sul_nodo else 'ora')
            self.block_replans += 1
            self.no_progress = 0

            new_ids = None
            if self.block_replans <= P.MAX_BLOCK_REPLANS:
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None:
                self._usa_percorso(new_ids)
                print(f"[REPLAN] Dopo il blocco ({self.block_replans}/{P.MAX_BLOCK_REPLANS}): percorso "
                      f"alternativo con {len(self.path_waypoints)} waypoint, dal punto in cui sono.")
                return CONTINUA

            print(f"[BLOCCO] Nessuna alternativa dal punto in cui sono"
                  + (f" (gia' {P.MAX_BLOCK_REPLANS} ripianificazioni dopo blocchi)."
                     if self.block_replans > P.MAX_BLOCK_REPLANS else ".")
                  + " Mi stacco arretrando e rimando la cella.")
            if self._ritorno((edge_a, edge_b),
                             f"dal nodo {self.robot_node_id}", skip_if_graph_cannot_help=True):
                return CONTINUA

            blocked_save_path = os.path.join(self.mission_folder,
                                             f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=self.frontier['stop_xy'], iteration=self.iteration, env=self.env,
                save_path=blocked_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up
            )
            return FALLITO

    def _esegui_tratti(self):
        """Il movimento deciso: controllo della posa d'arrivo, poi una o piu' gambe sulla
        polilinea confermata dal fronte, con arresto a meta' se il fronte si accorcia."""
        esito = self._controlla_posa_d_arrivo()
        if esito is not None:
            return esito

        # --- Esecuzione: una o piu' GAMBE sulla polilinea gia' confermata dal fronte,
        # senza scansione, refresh ne' ripianificazione in mezzo (vedi AVANZAMENTO
        # MULTINODO in decide_next_move). n_legs = 0 e' un solo passo parziale.
        n_legs = int(self.decision.get('nodes', 1)) if self.decision['full'] else 0
        if n_legs > 1:
            MISSION_STATS['multinode_moves'] += 1
            MISSION_STATS['multinode_nodes'] += n_legs
            print(f"[MULTINODO] Il fronte conferma "
                  f"{arcVerification.allowed_advance(self.frontier):.2f} m: percorro {n_legs} "
                  f"tratti di fila {[w[0] for w in self.path_waypoints[:n_legs]]} senza fermarmi "
                  f"a riscansionare.")
        for leg_i in range(max(1, n_legs)):
            if not self.path_waypoints:
                break
            self.next_node_id, next_x, next_y = self.path_waypoints[0]
            if n_legs == 0:
                move_x, move_y = self.decision['target']
                is_partial_step = True
            else:
                move_x, move_y = next_x, next_y
                is_partial_step = False
            if leg_i > 0:
                vision_tform_body_current = self._prepara_gamba(leg_i, n_legs, move_x, move_y)
                if vision_tform_body_current is None:
                    break
            else:
                vision_tform_body_current = get_a_tform_b(self.lg_proto_up.local_grid.transforms_snapshot,
                                                          VISION_FRAME_NAME, BODY_FRAME_NAME)
            self.no_progress = 0
            self.retreats_in_a_row = 0   # un avanzamento vero rompe la catena di arretramenti
            if is_partial_step:
                print(f"[PASSO PARZIALE] Verso il nodo {self.next_node_id}: avanzo di {self.decision['dist']:.2f} m "
                      f"su {np.hypot(next_x - self.robot_x, next_y - self.robot_y):.2f} fino a "
                      f"({move_x:.2f}, {move_y:.2f}), poi riguardo con una scansione fresca.")
            else:
                print(f"[OK] Tratto {self.robot_node_id}->{self.next_node_id} confermato libero dal fronte. Eseguo movimento...")

            esito = self._cammina_tratto(move_x, move_y, next_x, next_y, is_partial_step,
                                         vision_tform_body_current)
            if esito == 'fermo':
                break
            if esito is FALLITO:
                return FALLITO
        # Gambe finite (o interrotte): si torna in cima con una scansione fresca.
        return CONTINUA

    def _controlla_posa_d_arrivo(self):
        """Il passo porterebbe in un punto senza uscite? Allora non ci si va: tratto fuori dal
        piano per il tentativo e nuovo percorso (CONTINUA), o lato abbandonato (FALLITO).
        None = si puo' andare."""
        move_x, move_y = self.decision['target']
        # --- Controllo preventivo sulla posa d'arrivo --------------------------------------
        # Solo per i passi corti o verso spazi stretti: sono quelli con cui si entra negli
        # angoli senza uscita. Vedi arrival_has_exit.
        step_len = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
        r_a, c_a, in_a = arcVerification._cells(self.snap, np.array([move_x]), np.array([move_y]))
        od_arrival = float(self.snap.obstacle_dist[r_a[0], c_a[0]]) if in_a[0] else float('inf')
        if (self.arrival_refusals < P.MAX_ARRIVAL_REFUSALS
                and (step_len <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_arrival <= P.ARRIVAL_EXIT_CHECK_OD_M)):
            heading_arr = float(np.arctan2(move_y - self.robot_y, move_x - self.robot_x))
            has_exit, fan_arr = arcVerification.arrival_has_exit(self.snap, (move_x, move_y), heading_arr)
            if not has_exit:
                self.arrival_refusals += 1
                MISSION_STATS['arrival_refusals'] += 1
                n_us_a, fan_txt_a = arcVerification.describe_escape_fan(fan_arr)
                print(f"[ARRIVO] Il passo di {step_len:.2f} m verso ({move_x:.2f}, {move_y:.2f}) "
                      f"(obstacle_distance li' {od_arrival:.2f} m) mi porterebbe dove non ci sono "
                      f"uscite: {n_us_a}/{len(fan_arr)} direzioni utilizzabili ({fan_txt_a}). "
                      f"Non ci vado ({self.arrival_refusals}/{P.MAX_ARRIVAL_REFUSALS}), scarto il tratto "
                      f"{self.robot_node_id}->{self.next_node_id} per il resto del tentativo e ripianifico.")
                # Fuori dal piano per il RESTO DEL TENTATIVO (penalita' del tentativo, come i
                # ritorni: vedi _trap_backtrack), non solo per ora: il tratto in se' e' libero,
                # e' il punto d'arrivo a non avere uscite, quindi il refresh lo rimetterebbe
                # nel grafo al giro dopo e si rifarebbe lo stesso piano. Il 28-10 12:16 lo
                # stesso passo e' stato rifiutato tre volte di fila, e alla terza eseguito.
                self._scarta_tratto(self.robot_node_id, self.next_node_id, 'tentativo')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None and len(new_ids) > 1:
                    self._usa_percorso(new_ids)
                    return CONTINUA
                # Sapere in anticipo che il punto d'arrivo e' un vicolo cieco e andarci lo
                # stesso non ha senso: si rinuncia a questo lato della cella, che verra'
                # ritentata piu' avanti da un altro (finalize_or_defer_blocked_cell).
                print(f"[ARRIVO] Nessuna alternativa nel grafo e il punto d'arrivo non ha "
                      f"uscite: non ci vado. Rinuncio a entrare in ({self.target_row},{self.target_col}) "
                      f"da questo lato.")
                if self.verification_tracker is not None:
                    self.verification_tracker.clear_path()
                return FALLITO
        return None

    def _prepara_gamba(self, leg_i, n_legs, move_x, move_y):
        """Prima di una gamba intermedia del multinodo: posa vera, rotazione possibile e
        uscite all'arrivo. La posa VISION da cui partire, o None se ci si ferma a riguardare."""
        # Posa VERA dopo la gamba precedente. Serve a tre cose, tutte necessarie:
        #  - navigate_to calcola l'angolo di rotazione dall'orientamento che riceve,
        #    e relative_move ruota RISPETTO ALLA POSA ATTUALE: con l'orientamento
        #    della scansione iniziale (prima della gamba 0) il robot ruoterebbe
        #    dell'angolo sbagliato e camminerebbe fuori dal percorso verificato;
        #  - la distanza da percorrere si calcola da (robot_x, robot_y);
        #  - l'arresto a meta' movimento misura il percorso rimanente da qui.
        self.robot_x, self.robot_y, self.robot_z, robot_quat = spotUtils.getPosition(self.robot_state_client)
        self.robot_yaw = np.arctan2(
            2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
            1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))
        fresh_vision_tform_body = SimpleNamespace(
            position=SimpleNamespace(x=self.robot_x, y=self.robot_y, z=self.robot_z), rotation=robot_quat)

        # Al nodo intermedio il robot si FERMA e RUOTA verso il nodo successivo: la
        # decisione iniziale ha controllato solo la rotazione della prima gamba.
        # Qui si controlla questa, sulla fotografia della scansione (in coordinate
        # mondo, quindi valida anche dopo lo spostamento) -- ma solo se TUTTO il
        # cerchio spazzato sta dentro i dati: rotation_is_clear ignora le celle
        # fuori finestra (ottimista), e con un percorso curvo un nodo intermedio
        # puo' stare vicino al bordo. Se non si puo' ruotare, o non si puo' dire,
        # ci si ferma e si riguarda: la rotazione la gestisce il ciclo, con cono,
        # arretramento e ventaglio, sui dati freschi.
        r_h, c_h, in_h = arcVerification._cells(self.snap, np.array([self.robot_x]), np.array([self.robot_y]))
        od_here = float(self.snap.obstacle_dist[r_h[0], c_h[0]]) if in_h[0] else 0.0
        rot_leg = arcVerification.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), (move_x, move_y),
                                od_here, snap=self.snap)
        reach_rot = float(spotGrid.ROTATION_CLEARANCE_M)
        ny_s, nx_s = self.snap.obstacle_dist.shape
        inside_rot = (self.snap.origin_x <= self.robot_x - reach_rot and
                      self.robot_x + reach_rot <= self.snap.origin_x + nx_s * self.snap.cell_size and
                      self.snap.origin_y <= self.robot_y - reach_rot and
                      self.robot_y + reach_rot <= self.snap.origin_y + ny_s * self.snap.cell_size)
        if not rot_leg['turn'] or (rot_leg['why'] == 'settore spazzato' and not inside_rot):
            motivo = ("il corpo toccherebbe" if not rot_leg['turn']
                      else "il cerchio di rotazione esce dai dati della scansione")
            print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: per puntare il nodo "
                  f"{self.next_node_id} servono {rot_leg['dyaw_deg']:+.0f} gradi e {motivo}. "
                  f"Mi fermo qui e riguardo con una scansione fresca.")
            MISSION_STATS['multinode_cut'] += 1
            return None

        # Stessa prudenza della prima gamba sulla posa d'arrivo (arrival_has_exit):
        # non fermarsi in un punto senza uscite.
        step_leg = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
        r_l, c_l, in_l = arcVerification._cells(self.snap, np.array([move_x]), np.array([move_y]))
        od_leg = float(self.snap.obstacle_dist[r_l[0], c_l[0]]) if in_l[0] else float('inf')
        if step_leg <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_leg <= P.ARRIVAL_EXIT_CHECK_OD_M:
            has_exit_leg, _fan_leg = arcVerification.arrival_has_exit(
                self.snap, (move_x, move_y), float(np.arctan2(move_y - self.robot_y, move_x - self.robot_x)))
            if not has_exit_leg:
                print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: il nodo {self.next_node_id} "
                      f"(obstacle_distance {od_leg:.2f} m) non avrebbe uscite. Mi fermo "
                      f"qui e riguardo con una scansione fresca.")
                MISSION_STATS['multinode_cut'] += 1
                return None

        if self.verification_tracker is not None:
            self.path_version = self.verification_tracker.update_path(
                [(x, y) for _, x, y in self.path_waypoints])
        return fresh_vision_tform_body

    def _terreno_del_tratto(self, move_x, move_y):
        """Pendenza e rugosita' del tratto, andatura scelta di conseguenza."""
        # --- Misura della difficolta' di QUESTO segmento (per il gait e per il CSV).
        segment_long_slope, segment_lat_slope, segment_roughness = self.prm_graph.sample_terrain_between_points(
            self.robot_x, self.robot_y, move_x, move_y,
            node_idx1=self.robot_node_id, node_idx2=self.next_node_id
        )
        slope_source = self.prm_graph.last_slope_source

        near_thresh = spotGrid.SLOPE_THRESHOLD * (1.0 - spotGrid.NEAR_THRESHOLD_MARGIN_FRACTION)
        if (segment_long_slope >= near_thresh or segment_lat_slope >= near_thresh) and self.mission_folder is not None:
            try:
                npz_near = os.path.join(
                    self.mission_folder,
                    f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_"
                    f"arc{self.robot_node_id}-{self.next_node_id}_NEARLIMIT.npz")
                np.savez_compressed(
                    npz_near,
                    obstacle_grid=np.asarray(self.obstacle_mask_updated).reshape((self.num_y_up, self.num_x_up)),
                    terrain_grid=np.asarray(self.terrain_real_up).reshape((self.num_y_up, self.num_x_up)),
                    grid_origin_x=self.grid_origin_x_up, grid_origin_y=self.grid_origin_y_up, cell_size=self.cell_size_up,
                    arc_xy=np.array([self.robot_x, self.robot_y, move_x, move_y]),
                    long_slope=segment_long_slope, lat_slope=segment_lat_slope,
                    slope_threshold=spotGrid.SLOPE_THRESHOLD)
                MISSION_STATS['near_threshold_saves'] += 1
                margine = spotGrid.SLOPE_THRESHOLD - max(segment_long_slope, segment_lat_slope)
                print(f"[NEAR-LIMIT] Arco {self.robot_node_id}->{self.next_node_id} vicino soglia "
                      f"(long={segment_long_slope:.3f}, lat={segment_lat_slope:.3f}, "
                      f"soglia={spotGrid.SLOPE_THRESHOLD:.3f}, margine residuo={margine:+.3f}) "
                      f"-- dati salvati in {os.path.basename(npz_near)}")
            except Exception as e:
                print(f"[DATA SAVE] Impossibile salvare il file vicino soglia: {e}")

        if self.mobility_kwargs:
            segment_mobility_kwargs = self.mobility_kwargs
            segment_tier = 'override'
        else:
            segment_gradient = max(segment_long_slope, segment_lat_slope)
            segment_mobility_kwargs = movements.select_mobility_params(segment_gradient, segment_roughness)
            segment_tier = MISSION_STATS['last_tier']
        return (segment_long_slope, segment_lat_slope, segment_roughness, slope_source,
                segment_mobility_kwargs, segment_tier)

    def _muovi_e_registra(self, move_x, move_y, vision_tform_body_current, terreno):
        """Il comando di movimento, con l'arresto a meta' se il fronte si accorcia, e le righe
        di trajectory.csv, frontier_live.csv e segments.csv. Restituisce (riuscito, metri
        percorsi, metri comandati, motivo dell'arresto o None)."""
        (segment_long_slope, segment_lat_slope, segment_roughness, slope_source,
         segment_mobility_kwargs, segment_tier) = terreno
        # --- Condizione di arresto a meta' movimento ---------------------------------
        abort_state = {'reason': None, 'bad': 0}
        move_t0 = time.time()
        live_frontiers = []

        def _should_abort_move():
            if self.verification_tracker is None:
                return False
            f = self.verification_tracker.get_frontier()
            if f is None or f.get('path_version') != self.path_version or f['time'] <= move_t0:
                return False
            frx, fry = f['robot_xy']
            remaining = float(np.hypot(move_x - frx, move_y - fry))
            reach = arcVerification.allowed_advance(f)
            if not live_frontiers or live_frontiers[-1][0] != f['time']:
                live_frontiers.append((f['time'], frx, fry, f['reason'], float(f['dist']), reach, remaining,
                                       abort_state['bad']))
            if (f['reason'] == 'fine_dati'
                    and float(np.hypot(frx - self.robot_x, fry - self.robot_y)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                abort_state['bad'] = 0      # sta ancora ruotando sul posto: vedi la costante
                return False
            if reach + P.FRONTIER_ABORT_TOLERANCE_M < remaining:
                abort_state['bad'] += 1
                if abort_state['bad'] >= P.FRONTIER_ABORT_CONFIRM:
                    sxa, sya = f['stop_xy']
                    abort_state['reason'] = (f"il fronte si e' accorciato ({f['reason']} in "
                                             f"({sxa:.2f}, {sya:.2f})): raggiungibili {reach:.2f} m "
                                             f"su {remaining:.2f} m mancanti")
                    return True
            else:
                abort_state['bad'] = 0
            return False

        # --- Esecuzione con registrazione dell'inclinazione reale del corpo ---
        tilt_recorder = BodyTiltRecorder(self.robot_state_client)
        tilt_recorder.start()
        try:
            self.need_refresh = True              # fine di un movimento: vedi REFRESH A EVENTI
            success_move, distance_traveled, distance_commanded = movements.navigate_to(
                move_x, move_y, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                vision_tform_body_current, should_abort=_should_abort_move,
                keep_heading=self.keep_heading,
                **segment_mobility_kwargs
            )
        finally:
            tilt_stats = tilt_recorder.stop()
        move_duration = time.time() - move_t0
        timing_add('movimento', move_duration)
        try:   # solo log: non deve mai interrompere la missione
            _seg_id = [self.iteration, self.target_row, self.target_col, self.loop_iter, self.robot_node_id, self.next_node_id]
            append_trajectory(getattr(tilt_recorder, 'samples', []), 'prm', (move_x, move_y), _seg_id)
            append_csv("frontier_live.csv",
                        ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                         'robot_x', 'robot_y', 'reason', 'frontier_dist', 'reachable_m', 'remaining_m',
                         'consecutive_bad', 'aborted'],
                        [[lf[0]] + _seg_id + list(lf[1:]) + [abort_state['reason'] is not None]
                         for lf in live_frontiers])
        except Exception as e:
            print(f"[LOG] Traiettoria/fronti del movimento non registrati: {e}")

        # --- Riga del CSV per questo segmento (anche se il movimento e' fallito) ---
        try:
            seg_dist = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
            cost_dist = self.prm_graph.alpha * seg_dist
            cost_long = self.prm_graph.beta * segment_long_slope
            cost_lat = self.prm_graph.beta_lateral * segment_lat_slope
            cost_rough = self.prm_graph.gamma * segment_roughness
            log_segment({
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'iteration': self.iteration, 'cell_row': self.target_row, 'cell_col': self.target_col,
                'node_from': self.robot_node_id, 'node_to': self.next_node_id,
                'x_from': self.robot_x, 'y_from': self.robot_y, 'x_to': move_x, 'y_to': move_y,
                'dist_m': seg_dist,
                'slope_source': slope_source,
                'long_slope': segment_long_slope, 'lat_slope': segment_lat_slope,
                'roughness': segment_roughness,
                'cost_dist': cost_dist, 'cost_long': cost_long, 'cost_lat': cost_lat,
                'cost_rough': cost_rough, 'cost_total': cost_dist + cost_long + cost_lat + cost_rough,
                'tier': segment_tier, 'gait_override': bool(self.mobility_kwargs),
                'success': bool(success_move),
                'dist_traveled_m': distance_traveled, 'dist_commanded_m': distance_commanded,
                'duration_s': move_duration,
                'roll_start_deg': tilt_stats['roll_start'], 'pitch_start_deg': tilt_stats['pitch_start'],
                'roll_end_deg': tilt_stats['roll_end'], 'pitch_end_deg': tilt_stats['pitch_end'],
                'roll_peak_abs_deg': tilt_stats['roll_peak_abs'], 'pitch_peak_abs_deg': tilt_stats['pitch_peak_abs'],
                'tilt_samples': tilt_stats['samples'], 'tilt_errors': tilt_stats['errors'],
            })
        except Exception as e:
            print(f"[LOG] Impossibile registrare il segmento: {e}")
        return success_move, distance_traveled, distance_commanded, abort_state['reason']

    def _cammina_tratto(self, move_x, move_y, next_x, next_y, is_partial_step, vision_tform_body_current):
        """Un tratto: andatura, movimento, esito. 'avanti' = riuscito, si puo' fare la gamba
        dopo; 'fermo' = interrotto o ripianificato, si riguarda; FALLITO = guasto fisico."""
        terreno = self._terreno_del_tratto(move_x, move_y)
        success_move, distance_traveled, distance_commanded, abort_reason = self._muovi_e_registra(
            move_x, move_y, vision_tform_body_current, terreno)
        # Comando finito male (tempo scaduto, Spot non assestato...) senza un arresto chiesto
        # dal fronte, ma con il robot ARRIVATO a pochi cm dal punto: il tratto vale come
        # riuscito invece che come fallimento fisico (arco scartato per sempre e ritorno).
        # 28-10 13:50, cella (3,2): salita a 20 gradi in HARD, 0.42 m su 0.44 oltre i 15 s.
        # In segments.csv resta l'esito vero del comando.
        if not success_move and abort_reason is None:
            try:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                d_left = float(np.hypot(move_x - ax, move_y - ay))
            except Exception:
                d_left = float('inf')
            if d_left <= P.MOVE_NEAR_GOAL_M and distance_traveled >= 0.5 * distance_commanded:
                MISSION_STATS['moves_near_goal'] = MISSION_STATS.get('moves_near_goal', 0) + 1
                print(f"[QUASI] Comando verso ({move_x:.2f}, {move_y:.2f}) non concluso, ma il robot "
                      f"e' a {d_left:.2f} m dal punto (percorsi {distance_traveled:.2f} m su "
                      f"{distance_commanded:.2f}): vale come arrivato, l'arco resta valido.")
                success_move = True

        if success_move:
            if not hasattr(self.env, '_traveled_arcs'):
                self.env._traveled_arcs = []
            self.env._traveled_arcs.append((self.robot_x, self.robot_y, move_x, move_y))

            if is_partial_step:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                self._sosta(ax, ay, "sosta (passo parziale)")
            else:
                # Arco "percorso" solo se il robot e' partito dal nodo: altrimenti il tratto
                # camminato non e' l'arco del grafo (vedi add_stop_node, nessun nodo nuovo).
                if mission_graph.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y):
                    self.prm_graph.traversed_edges.add((min(self.robot_node_id, self.next_node_id),
                                                   max(self.robot_node_id, self.next_node_id)))
                self.robot_node_id = self.next_node_id
                self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
                self.path_waypoints.pop(0)

            # Briciola per i ritorni: un waypoint GraphNav ogni
            # GRAPHNAV_WAYPOINT_MIN_SPACING_M di strada percorsa.
            navGraphUtils.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client,
                                     self.robot_node_id, self.target_row, self.target_col)

        elif abort_reason is not None:
            print(f"[STOP] Movimento verso ({move_x:.2f}, {move_y:.2f}) interrotto dopo "
                  f"{distance_traveled:.2f}m: {abort_reason}.")
            MISSION_STATS['frontier_aborts'] += 1
            if distance_traveled > 0.05:
                self._sosta(*self._strada_fatta(self.robot_x, self.robot_y), "sosta (arresto)")
            return 'fermo'

        else:
            print(f"[FAIL] Comando di movimento fallito meccanicamente per ({move_x:.2f}, {move_y:.2f}) "
                  f"(percorsi {distance_traveled:.2f}m su {distance_commanded:.2f}m richiesti)")

            movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                    "dopo il fallimento del movimento")

            if distance_traveled > 0.05:
                self._strada_fatta(self.robot_x, self.robot_y)

            # Se il robot non partiva dal nodo, il tratto fallito non e' l'arco del grafo.
            sul_nodo = mission_graph.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y)
            self._scarta_tratto(self.robot_node_id, self.next_node_id, 'missione' if sul_nodo else 'ora')
            print(f"[INFO] Tratto {self.robot_node_id}-{self.next_node_id} scartato dopo fallimento fisico.")
            if self._ritorno((self.robot_node_id, self.next_node_id),
                             f"dal nodo {self.robot_node_id}"):
                return 'fermo'

            fail_save_path = os.path.join(self.mission_folder,
                                          f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_MOVE_FAIL.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(next_x, next_y), iteration=self.iteration, env=self.env,
                save_path=fail_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up
            )
            return FALLITO
        return 'avanti'

    def _fine(self):
        """Dopo il ciclo: True se il robot e' nella cella (o l'arrivo appena fuori e' stato accettato)."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()

        # Controllo finale della posizione a fine percorso
        self.robot_x, self.robot_y, _, _ = spotUtils.getPosition(self.robot_state_client)
        if self.arrived_near_goal:
            return True
        return self.env.is_point_in_cell(self.robot_x, self.robot_y, self.target_row, self.target_col)



def attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client,
                                     env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                                     iteration=0,
                                     recordingInterface=None, verification_tracker=None, mobility_kwargs=None):
    """Un tentativo di entrare nella cella (target_row, target_col): vedi TentativoCella."""
    return TentativoCella(local_grid, global_grid, robot_state_client, command_client,
                          env, target_row, target_col, sampler, prm_graph, mission_folder,
                          iteration, recordingInterface, verification_tracker,
                          mobility_kwargs).esegui()



# =========================================================================
# RITIRATA A ZAMPE all'indietro: l'ultima rete
# =========================================================================


def retreat_along_traveled_path(env, robot_state_client, command_client,
                                max_segments=P.RETREAT_MAX_SEGMENTS,
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
        if gap > P.RETREAT_CONTINUITY_M:
            print(f"[RITIRATA] Non sono dove finiva l'ultimo tratto registrato (distanza {gap:.2f} m, "
                  f"es. dopo uno spostamento con GraphNav): non arretro alla cieca.")
            del traveled[:]
            break
        if seg_len > P.RETREAT_MAX_SEGMENT_M:
            print(f"[RITIRATA] Il tratto di rientro e' lungo {seg_len:.2f} m (oltre "
                  f"{P.RETREAT_MAX_SEGMENT_M:.1f} m): non lo ripercorro all'indietro.")
            break

        if seg_len < P.RETREAT_MIN_SEGMENT_M:
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
        with TrajectoryLog(robot_state_client, 'ritirata', (from_x, from_y)):
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


# =========================================================================
# CELLE BLOCCATE O RIMANDATE
# =========================================================================


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
        # ATTENZIONE a come si legge: i punti candidati per cella sono decine (reticolo a
        # 0.5 m) o centinaia (a 0.25 m), quindi "occupati" alto NON vuol dire cella irraggiungibile; vuol
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


# =========================================================================
# LA MISSIONE: il ciclo cella per cella, avvio e chiusura
# =========================================================================


class Missione:
    """La missione cella per cella, dopo l'avvio (vedi easy_walk).

    La frontiera e' la lista delle celle candidate (riga, colonna, rango sul percorso a
    serpentina). A ogni giro: se una cella della frontiera confina con quella del robot si
    tenta di entrarci da qui (_cella_confinante); altrimenti si sceglie la candidata di rango
    piu' basso, ci si avvicina con GraphNav fino al waypoint della cella visitata piu' vicina
    e si tenta da li' (_scegli_cella_lontana, _viaggio_e_tentativo). Un tentativo fallito segna
    il LATO da cui si e' provato (_lato_fallito): la cella si blocca solo quando ogni lato
    raggiungibile e' stato provato (finalize_or_defer_blocked_cell).

    A batteria bassa (vedi batteria.py) l'esplorazione si ferma anche con la frontiera non
    vuota: un tentativo interrotto per il rientro non segna il lato come provato e la cella
    resta in frontiera, cosi' lo stato salvato dice cosa restava da esplorare.
    """

    def __init__(self, robot_state_client, command_client, recordingInterface, local_grid,
                 global_grid, env, prm, sampler, verification_tracker, mission_folder,
                 mobility_kwargs):
        self.robot_state_client = robot_state_client
        self.command_client = command_client
        self.recordingInterface = recordingInterface
        self.local_grid = local_grid
        self.global_grid = global_grid
        self.env = env
        self.prm = prm
        self.sampler = sampler
        self.verification_tracker = verification_tracker
        self.mission_folder = mission_folder
        self.mobility_kwargs = mobility_kwargs
        self.path = env.generate_serpentine_path(start_cell=env.start_cell)
        self.frontier = []
        self.visualization_counter = 0   # numero progressivo delle immagini salvate
        # Avanzamento (vedi _progresso): istante d'inizio, celle chiuse all'inizio e
        # all'ultima riga stampata, punti del rettangolo della missione per l'area vista.
        self._t0 = None
        self._closed_at_start = 0
        self._ultima_visitata = None      # vedi _cella_del_robot
        self._tentativi = 0               # tentativi di entrare in una cella (vedi _fine_esplorazione)
        self._celle_perse = []            # tolte dalla frontiera senza lati ancora provabili
        self._closed_logged = None
        self._area_points = None

    # --- Pezzi comuni --------------------------------------------------------------------

    def _tenta(self, row, col):
        """Un tentativo di entrare nella cella (row, col) dalla posizione attuale."""
        check = attempt_enter_cell_from_position(
            self.local_grid, self.global_grid, self.robot_state_client, self.command_client,
            self.env, row, col, self.sampler, self.prm, self.mission_folder,
            self.visualization_counter, self.recordingInterface, self.verification_tracker,
            mobility_kwargs=self.mobility_kwargs)
        self.visualization_counter += 1
        self._tentativi += 1
        return check

    def _visitata(self, row, col, x, y):
        """Cella (row, col) visitata con il robot in (x, y): waypoint GraphNav della cella e
        nuove candidate attorno a lei."""
        self.env.update_position(x, y)
        self.recordingInterface.create_default_waypoint(cell_row=row, cell_col=col)
        self.env.mark_cell_visited(row, col)
        self._ultima_visitata = (int(row), int(col))
        # Le confinanti della cella APPENA VISITATA, non di quella in cui sta il robot: con
        # l'arrivo appena fuori dalla cella (ARRIVAL_OUTSIDE_CELL_M) il robot puo' essere
        # ancora nella cella di prima.
        self.frontier.extend(environmentMap.find_new_borders(
            self.env, row, col, self.path, self.frontier))

    def _cella_del_robot(self):
        """
        La cella da cui il robot cerca le confinanti e segna i lati provati. Di solito quella
        in cui sta; ma se l'ultima cella visitata e' stata accettata con il robot appena FUORI
        (ARRIVAL_OUTSIDE_CELL_M) e il robot e' ancora li', vale quella: il 28-10 13:50 la (7,2)
        e' stata visitata con il robot 0.29 m fuori, nella (6,3), e la (7,1) che le stava
        accanto non e' mai stata tentata da li'.
        """
        x, y, _, _ = spotUtils.getPosition(self.robot_state_client)
        cell = tuple(int(v) for v in self.env.get_cell_from_world(x, y))
        uv = self._ultima_visitata
        if uv is not None and cell != uv:
            border = environmentMap.cell_border_distance(self.env, uv[0], uv[1], x, y)
            if border is not None and -border <= P.ARRIVAL_OUTSIDE_CELL_M:
                print(f"[CELLA] Il robot e' nella cella {cell}, ma {-border:.2f} m fuori dalla "
                      f"{uv} appena visitata: cerco le confinanti dalla {uv}.")
                return uv
        return cell

    def _cella_di_partenza_per(self, row, col, waypoints_by_cell):
        """
        La cella visitata (con un waypoint GraphNav) da cui avvicinarsi alla candidata (row,
        col): fra le CONFINANTI, una il cui lato verso la candidata non e' ancora stato
        provato, la piu' vicina al robot. Se nessuna confinante ha un lato libero, la regola
        di sempre (find_nearest_waypoint_cell_to_target), che il chiamante poi salta se quel
        lato e' gia' provato. Prima si prendeva sempre la prima confinante in ordine fisso
        (nord, sud, ovest, est): provato quel lato, la cella non veniva piu' tentata dagli
        altri e usciva dalla frontiera (28-10 13:50, celle (7,0) e (7,1)).
        """
        env, ri = self.env, self.recordingInterface
        sides = env.get_cell_sides_status(row, col)
        x, y, _, _ = spotUtils.getPosition(self.robot_state_client)
        cands = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            if not (0 <= r < env.rows and 0 <= c < env.cols) or env.map[r][c] != 1:
                continue
            if (r, c) not in waypoints_by_cell or ri.get_manual_waypoint_by_cell(r, c) is None:
                continue
            if sides & env.get_side_bit_facing_origin(r, c, row, col):
                continue
            wx, wy = env.get_world_position_from_cell(r, c)
            cands.append((float(np.hypot(wx - x, wy - y)), (r, c)))
        if cands:
            d, cell = min(cands)
            print(f"[NEAREST_WP] Cella ({row},{col}): mi avvicino dalla confinante {cell}, lato non "
                  f"ancora provato, a {d:.1f} m dal robot"
                  + (f" ({len(cands)} lati liberi)" if len(cands) > 1 else "") + ".")
            return cell
        return ri.find_nearest_waypoint_cell_to_target((row, col), waypoints_by_cell, env)

    def _lato_fallito(self, row, col, from_row, from_col):
        """Segna come provato il lato della cella (row, col) che guarda la cella (from_row,
        from_col). True se ora la cella e' bloccata (nessun lato raggiungibile da provare)."""
        side_bit = self.env.get_side_bit_facing_origin(from_row, from_col, row, col)
        self.env.mark_cell_side_explored(row, col, side_bit)
        finalize_or_defer_blocked_cell(self.env, row, col,
                                       self.sampler, self.global_grid.global_occupancy_map)
        return self.env.is_cell_blocked(row, col)

    # --- Il ciclo ------------------------------------------------------------------------

    def esplora(self):
        """Visita le celle finche' la frontiera non e' vuota."""
        x, y, z, _ = spotUtils.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x, y)
        self.frontier.extend(environmentMap.find_new_borders(
            self.env, robot_row, robot_col, self.path, self.frontier))
        self._t0 = time.time()
        self._closed_at_start = self._celle()[0]

        while (True):
            self._progresso()
            if _rientro_batteria():
                MISSION_STATS['battery_return_cells_left'] = len(self.frontier)
                print(f"[BATTERIA] Fine dell'esplorazione per batteria bassa: restano "
                      f"{len(self.frontier)} celle in frontiera {[(f[0], f[1]) for f in self.frontier]}.")
                break
            robot_row, robot_col = self._cella_del_robot()

            borders = self.env.get_adjacent_frontier_cells(robot_row, robot_col, self.path)
            borders_in_frontier = [b for b in borders
                                   if any((f[0] == b[0] and f[1] == b[1]) for f in self.frontier)]

            if len(borders_in_frontier) != 0:
                self._cella_confinante(min(borders_in_frontier, key=lambda b: b[2]),
                                       robot_row, robot_col)
            else:
                # Nessuna cella confinante in frontiera: si va con GraphNav verso il waypoint
                # piu' vicino alla cella di rango piu' basso e si tenta da li'.
                scelta = self._scegli_cella_lontana()
                if scelta is not None:
                    if self._viaggio_e_tentativo(*scelta) == 'caduta':
                        continue
                else:
                    if len(self.frontier) > 0:
                        tolta = self.frontier[0]
                        self.frontier.remove(tolta)
                        self._celle_perse.append((int(tolta[0]), int(tolta[1])))
                        print(f"[FRONTIERA] Cella ({tolta[0]},{tolta[1]}) tolta dalla frontiera: in "
                              f"questo giro nessuna candidata ha un lato ancora da provare.")

            if len(self.frontier) == 0:
                break
        self._fine_esplorazione()
        self._progresso(finale=True)

    def _fine_esplorazione(self):
        """Se l'esplorazione finisce con celle non chiuse, una riga [FINE] che dice perche'
        (28-10 14:17: partenza in mezzo all'erba alta, nessuna cella raggiungibile, e nel log
        solo "Frontiera vuota")."""
        closed, visited, blocked, total = self._celle()
        if closed >= total or _batteria_scarica():
            return
        if visited <= 1:
            print(f"[FINE] Nessuna cella raggiungibile dalla partenza: {self._tentativi} tentativi "
                  f"sulle celle confinanti, tutti falliti senza una strada nel grafo o con il passo "
                  f"rifiutato (ostacoli tutto intorno: vedi GRAFO DISCONNESSO e [ARRIVO] sopra). "
                  f"Spostare il robot su terreno libero e riprovare.")
            return
        print(f"[FINE] Esplorazione finita con {total - closed} celle non chiuse"
              + (f"; tolte dalla frontiera senza lati ancora da provare: "
                 f"{', '.join(f'({r},{c})' for r, c in self._celle_perse)}" if self._celle_perse else "")
              + ". Le altre non sono mai state raggiungibili da una cella visitata.")

    # --- Avanzamento ---------------------------------------------------------------------

    def _celle(self):
        """(chiuse, visitate, bloccate, totale): chiusa = visitata o bloccata."""
        vals = [v for row in self.env.map for v in row]
        visited = sum(1 for v in vals if v == 1)
        blocked = sum(1 for v in vals if v == -1)
        return visited + blocked, visited, blocked, len(vals)

    def _area_vista(self):
        """Frazione del rettangolo della missione gia' osservata (libera o ostacolo) nella
        mappa globale, su punti ogni 0.1 m. None se la mappa non c'e' ancora."""
        gm = self.global_grid.global_occupancy_map
        if gm is None:
            return None
        if self._area_points is None:
            env = self.env
            k = np.arange(-env.cell_size / 2.0 + 0.05, env.cell_size / 2.0, 0.1)
            ox, oy = np.meshgrid(k, k)
            c, s_ = np.cos(env.origin_yaw), np.sin(env.origin_yaw)
            pts = []
            for r in range(env.rows):
                for col in range(env.cols):
                    cx, cy = env.get_world_position_from_cell(r, col)
                    pts.append(np.c_[cx + ox.ravel() * c - oy.ravel() * s_,
                                     cy + ox.ravel() * s_ + oy.ravel() * c])
            self._area_points = np.vstack(pts)
        res = gm.resolution
        keys = np.floor(self._area_points / res).astype(int)
        return sum(1 for gx, gy in keys if (gx, gy) in gm.grid) / len(keys)

    def _progresso(self, finale=False):
        """
        A che punto e' la missione: una riga [PROGRESSO] quando cambiano le celle chiuse
        (visitate o bloccate), e una alla fine. Celle chiuse sul totale (il numero
        principale), area della missione gia' vista nella mappa globale, tempo, stima del
        tempo che manca dal tempo medio per cella, batteria. Solo log e MISSION_STATS
        ['progresso'] (rapporto, mission_summary.json): nessuna decisione ne dipende.
        """
        closed, visited, blocked, total = self._celle()
        if not finale and closed == self._closed_logged:
            return
        self._closed_logged = closed
        try:
            area = self._area_vista()
        except Exception:
            area = None
        minutes = (time.time() - self._t0) / 60.0 if self._t0 is not None else 0.0
        done_here = closed - self._closed_at_start
        per_cell = minutes / done_here if done_here > 0 else None
        # L'ultima carica letta dal monitor della batteria (batteria.py), senza una lettura in piu'.
        battery = MISSION_STATS.get('battery_last_pct')
        pct = 100.0 * closed / total
        area_txt = f"area vista {100.0 * area:.0f}%" if area is not None else "area vista ?"
        bat_txt = f"batteria {battery:.0f}%" if battery is not None else "batteria ?"
        if finale:
            tempo = (f"{minutes:.1f} min di esplorazione"
                     + (f" ({per_cell:.2f} min a cella)" if per_cell else "")
                     + (f", {total - closed} celle mai raggiunte" if total > closed else ""))
        else:
            tempo = (f"{minutes:.1f} min trascorsi"
                     + (f", ~{(total - closed) * per_cell:.0f} min alla fine ({per_cell:.2f} min a cella)"
                        if per_cell else ""))
        print(f"[PROGRESSO] {'Fine esplorazione: ' if finale else ''}{closed}/{total} celle chiuse "
              f"({pct:.0f}%): {visited} visitate, {blocked} bloccate, {total - closed} da fare | "
              f"{area_txt}\n            {tempo} | {bat_txt}")
        MISSION_STATS['progresso'] = {
            'celle_chiuse': closed, 'celle_totali': total, 'percentuale': round(pct, 1),
            'visitate': visited, 'bloccate': blocked,
            'area_vista': None if area is None else round(100.0 * area, 1),
            'minuti': round(minutes, 1), 'minuti_a_cella': None if per_cell is None else round(per_cell, 2),
            'batteria': battery, 'finale': bool(finale)}

    def _cella_confinante(self, selected_border, robot_row, robot_col):
        """Tentativo su una cella della frontiera che confina con quella del robot."""
        row, col = selected_border[0], selected_border[1]
        # Cella da cui si tenta l'ingresso: serve dopo per segnare il LATO provato. Non la
        # cella in cui il robot si trova DOPO il tentativo, che una ritirata o un passo
        # parziale possono aver cambiato (lato sbagliato o nullo).
        origin_row, origin_col = robot_row, robot_col
        check = self._tenta(row, col)
        self.frontier.remove(selected_border)

        x_new, y_new, _, _ = spotUtils.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x_new, y_new)

        if check or (row, col) == (robot_row, robot_col):
            # Fallito ma il robot e' comunque dentro la cella: vale come visitata.
            self._visitata(row, col, x_new, y_new)
        elif _batteria_scarica():
            # Tentativo interrotto per il rientro: la cella resta da esplorare.
            self.frontier.append(selected_border)
        else:
            self._recupero(selected_border, origin_row, origin_col)

    def _recupero(self, selected_border, origin_row, origin_col):
        """Subito dopo un tentativo fallito su una cella confinante: si riprova una volta se
        da qui c'e' una strada corta (al massimo COSTO_SOGLIA archi) verso un obiettivo
        diverso da quello appena fallito; altrimenti il lato si segna come provato.

        Gli archi scartati nel tentativo fallito (blocchi confermati da vicino, fallimenti
        fisici) sono gia' nel PRM tramite mark_edge_invalid, e la ricostruzione degli archi
        li rispetta: Dijkstra non riproporra' la stessa strada."""
        row, col = selected_border[0], selected_border[1]
        prm, global_map = self.prm, self.global_grid.global_occupancy_map

        # Posizione attuale (il fallimento puo' essere in partenza o a meta' strada)
        x_curr, y_curr, _, _ = spotUtils.getPosition(self.robot_state_client)

        # Scansione fresca, per scegliere il punto obiettivo da qui
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is not None:
            # Senza impronta del corpo ne' maschera degli ostacoli: serve solo il terreno per
            # il PRM e la scelta dell'obiettivo.
            self.local_grid.process_scan(scan, x_curr, y_curr, 0.0, footprint=False, fuse=False)
            num_x, num_y = scan.num_x, scan.num_y
            grid_origin_x, grid_origin_y = self.local_grid.grid_origin(scan)
            prm.update_local_grid_data(
                grad_values_2d=scan.grad.reshape((num_y, num_x)),
                rough_values_2d=scan.rough.reshape((num_y, num_x)),
                terrain_values_2d=scan.terrain.reshape((num_y, num_x)),
                valid_2d=spotGrid.slope_usable_mask(scan.is_valid, scan.od, (num_y, num_x)),
                grid_origin_x=grid_origin_x,
                grid_origin_y=grid_origin_y,
                cell_size=scan.cell_size
            )
            mission_graph.rebuild_prm_edges(prm, global_map)

            target_x, target_y, _, _ = find_best_point_in_cell(
                x_curr, y_curr, self.env, row, col, scan.pts, scan.od, self.sampler,
                global_map=global_map
            )
        else:
            print(
                "[RECOVERY ERROR] Impossibile recuperare il layer obstacle_distance per il ricalcolo.")
            target_x, target_y = None, None

        if target_x is None or target_y is None:
            print(
                "[SKIP] Nessun punto obiettivo valido nella cella. Continuo con l'esplorazione normale.")
            if not self._lato_fallito(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)
            return

        mission_graph.restrict_planning_to_explored_cells(prm, self.env, row, col, (x_curr, y_curr))
        start_id = prm.add_stop_node(
            x_curr, y_curr, global_map,
            spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="verifica recupero (robot)")
        goal_id = prm.add_stop_node(
            target_x, target_y, global_map,
            spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label="verifica recupero (obiettivo)",
            is_robot=False)
        prm.set_robot_node(start_id)

        path_ids = prm.find_path_dijkstra(start_id, goal_id)

        # Si riprova subito solo se il percorso ha al massimo COSTO_SOGLIA archi (numero di
        # nodi - 1)
        COSTO_SOGLIA = 4

        # Se l'obiettivo e' lo STESSO del tentativo appena fallito, riprovare subito vorrebbe
        # dire stesso posto, stesso esito: la cella si rimanda e torna in coda come le altre
        # (vedi finalize_or_defer_blocked_cell).
        last_goal = getattr(self.env, '_last_attempt_goal', None)
        same_goal = (last_goal is not None and float(np.hypot(
            target_x - last_goal[0], target_y - last_goal[1])) <= P.RECOVERY_SAME_GOAL_M)
        if same_goal and path_ids is not None:
            print(f"[RECOVERY] L'obiettivo ({target_x:.2f}, {target_y:.2f}) e' "
                  f"quello del tentativo appena fallito: non riprovo subito, la "
                  f"cella torna in coda.")
            path_ids = None
        if path_ids is None or (len(path_ids) - 1) > COSTO_SOGLIA:
            print(
                f"[SKIP] Nessun percorso alternativo valido o costo superiore a {COSTO_SOGLIA}. Continuo l'algoritmo normale.")
            if not self._lato_fallito(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)
            return

        print(
            f"[RECOVERY] Trovato percorso alternativo con costo {len(path_ids) - 1} <= {COSTO_SOGLIA}. Riprovo l'ingresso...")
        if self._tenta(row, col):
            x_final, y_final, _, _ = spotUtils.getPosition(self.robot_state_client)
            self._visitata(row, col, x_final, y_final)
        elif _batteria_scarica():
            self.frontier.append(selected_border)
        else:
            print(
                "[FAIL] Anche il percorso alternativo ha fallito. Abbandono la cella e continuo l'algoritmo normale.")
            if not self._lato_fallito(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)

    def _scegli_cella_lontana(self):
        """La candidata di rango piu' basso da cui avvicinarsi con GraphNav:
        (riga, colonna, rango, cella visitata del waypoint di partenza), None se nessuna.

        Una cella bloccata (-1) non ha mai lati ancora da provare:
        finalize_or_defer_blocked_cell() la blocca solo dopo che ogni lato raggiungibile e'
        stato provato ed e' fallito."""
        env, recordingInterface = self.env, self.recordingInterface
        already_skipped_this_round = set()
        while True:
            remaining = [c for c in self.frontier if (c[0], c[1]) not in already_skipped_this_round]
            lowest_rank_cell = env.get_lowest_rank_from_frontier_list(remaining, self.path)
            if lowest_rank_cell is None:
                return None
            candidate_row, candidate_col, candidate_rank = lowest_rank_cell
            waypoints_by_cell = recordingInterface.get_all_manual_waypoints_with_cells()
            candidate_nearest_cell = self._cella_di_partenza_per(candidate_row, candidate_col,
                                                                 waypoints_by_cell)
            if candidate_nearest_cell is None or \
                    recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell) is None:
                # Nessuna cella visitata con waypoint da cui partire: si passa alla candidata
                # successiva.
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
            # Prima di partire con GraphNav: da quel waypoint il grafo arriva alla cella? Se
            # no, il tentativo fallirebbe appena arrivati (vedi GRAPHNAV_PRECHECK): il lato si
            # segna come provato senza muoversi.
            if P.GRAPHNAV_PRECHECK:
                cand_wp = recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell)
                wp_xy = (cand_wp.get('x'), cand_wp.get('y')) if isinstance(cand_wp, dict) else (None, None)
                reach = (mission_graph.graph_reaches_cell(self.prm, env, wp_xy, candidate_row, candidate_col,
                                                          self.global_grid.global_occupancy_map)
                         if wp_xy[0] is not None else None)
                if reach is False:
                    MISSION_STATS['graphnav_trips_avoided'] += 1
                    print(f"[PRECHECK] Cella ({candidate_row},{candidate_col}): dal waypoint "
                          f"della cella {candidate_nearest_cell} il grafo non arriva a nessun "
                          f"nodo della cella (ostacoli gia' in mappa). Segno il lato come "
                          f"provato senza spostarmi.")
                    env.mark_cell_side_explored(candidate_row, candidate_col, candidate_side_bit)
                    finalize_or_defer_blocked_cell(env, candidate_row, candidate_col,
                                                   self.sampler, self.global_grid.global_occupancy_map)
                    if env.is_cell_blocked(candidate_row, candidate_col):
                        self.frontier[:] = [c for c in self.frontier
                                            if (c[0], c[1]) != (candidate_row, candidate_col)]
                    already_skipped_this_round.add((candidate_row, candidate_col))
                    continue
            return candidate_row, candidate_col, candidate_rank, candidate_nearest_cell

    def _viaggio_e_tentativo(self, target_row, target_col, rank, nearest_cell):
        """Con GraphNav fino al waypoint della cella nearest_cell, poi il tentativo sulla cella
        (target_row, target_col). 'caduta' se il robot e' caduto durante il viaggio."""
        recordingInterface = self.recordingInterface
        x_current, y_current, _, _ = spotUtils.getPosition(self.robot_state_client)

        nearest_wp = recordingInterface.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
        navGraphUtils.graphnav_add_shortcuts(recordingInterface, "prima dello spostamento fra celle",
                                             map_edges=True, target_wp_id=nearest_wp['id'],
                                             robot_xy=(x_current, y_current))
        recordingInterface.stop_recording()
        navigation_success = navGraphUtils.graphnav_navigate_to(
            recordingInterface, nearest_wp['id'], self.robot_state_client, self.command_client,
            f"verso il waypoint della cella {nearest_cell} per la cella ({target_row},{target_col})",
            face_xy=self.env.get_world_position_from_cell(target_row, target_col))

        # GraphNav non registra i tratti percorsi: quelli vecchi non descrivono piu' la strada
        # da cui il robot e' arrivato, la ritirata non deve usarli.
        self.env._traveled_arcs = []
        if movements.check_and_recover_fall(self.robot_state_client, self.command_client,
                                            self.global_grid.global_occupancy_map,
                                            "durante lo spostamento con GraphNav") is not None:
            recordingInterface.start_recording()
            print(f"[CADUTA] Rialzato dopo lo spostamento con GraphNav: segno il lato della "
                  f"cella ({target_row},{target_col}) come provato, per non ripetere la stessa strada.")
            if self._lato_fallito(target_row, target_col, nearest_cell[0], nearest_cell[1]):
                self.frontier.remove((target_row, target_col, rank))
            return 'caduta'

        recordingInterface.start_recording()
        # Il viaggio puo' essere stato lungo: se la batteria e' scesa sotto soglia non si
        # tenta la cella, si rientra (la cella resta in frontiera).
        if _rientro_batteria("dopo lo spostamento GraphNav"):
            return None
        # Se GraphNav non e' nemmeno arrivato al waypoint di partenza vale come un tentativo
        # fallito. Lato segnato come provato; la cella esce dalla frontiera solo se era
        # l'ULTIMO lato non provato (cella davvero bloccata).
        if navigation_success and self._tenta(target_row, target_col):
            x_final, y_final, _, _ = spotUtils.getPosition(self.robot_state_client)
            self.frontier.remove((target_row, target_col, rank))
            self._visitata(target_row, target_col, x_final, y_final)
        elif _batteria_scarica():
            pass
        elif self._lato_fallito(target_row, target_col, nearest_cell[0], nearest_cell[1]):
            self.frontier.remove((target_row, target_col, rank))
        return None

    def scansione_finale(self):
        """L'area attorno all'ultima posizione, prima di tornare: entra nelle mappe globali."""
        print("[FINAL SCAN] Ultima scansione attorno al robot prima di tornare alla partenza...")
        x_final, y_final, z_final, quat_final = spotUtils.getPosition(self.robot_state_client)
        final_yaw = np.arctan2(2.0 * (quat_final.w * quat_final.z + quat_final.x * quat_final.y),
                               1.0 - 2.0 * (quat_final.y ** 2 + quat_final.z ** 2))

        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[FINAL SCAN] ATTENZIONE: griglia locale non ricevuta, proseguo senza scansione finale.")
            return
        self.local_grid.process_scan(scan, x_final, y_final, final_yaw, ground_label="scan-finale")
        # Anche l'ultima scansione entra nelle mappe globali, come tutte le altre
        spotGrid.fold_scan_into_global_maps(
            self.global_grid.global_occupancy_map, self.global_grid.global_terrain_map, scan.pts,
            scan.terrain, scan.obstacle_mask, scan.is_valid, scan.od,
            (scan.num_y, scan.num_x), (x_final, y_final, final_yaw))

        final_save_path = os.path.join(
            self.mission_folder, f"iteration_{self.visualization_counter}_FINAL_SCAN.png"
        ) if self.mission_folder else None

        visualize_grid_with_candidates(
            pts=scan.pts, terrain_real=scan.terrain, obstacle_mask=scan.obstacle_mask,
            robot_x=x_final, robot_y=y_final,
            candidates={'rejected': [], 'valid': []},
            chosen_point=None, iteration=self.visualization_counter, env=self.env,
            save_path=final_save_path, prm_graph=self.prm, chosen_path=None,
            cells_obstacle_dist=scan.od, valid_values=scan.valid,
            grad_values=scan.grad, rough_values=scan.rough
        )
        self.visualization_counter += 1
        print("[FINAL SCAN] Fatto: l'ultima scansione e' nelle mappe globali.")


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
    print(f"[CONFIG] OSTACOLI: mediana su {arcVerification.OBSTACLE_MEDIAN_FRAMES} scansioni "
          f"+ isteresi {arcVerification.OBSTACLE_HYSTERESIS_M:.2f} m attorno a "
          f"{spotGrid.FRONTIER_CLEARANCE_M:.2f} m (rumore misurato 1.5 cm tipici, 7.3 di punta); "
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
          f"{spotGrid.PLAN_CLEARANCE_M:.2f}), almeno {P.TARGET_CELL_BORDER_M:.2f} m dal bordo "
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
    navGraphUtils.graphnav_add_shortcuts(recordingInterface, "prima del ritorno a wp_0",
                                         map_edges=True, target_wp_id=_wp0_id, robot_xy=_xy_end)
    recordingInterface.auto_close_loops(True, False)
    recordingInterface.stop_recording()
    # Chiusura d'anello di Spot una volta sola, PRIMA dell'ancoraggio: i suoi archi,
    # verificati sulle scansioni, correggono la deriva della mappa scaricata e accorciano il
    # ritorno (vedi GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
    navGraphUtils.graphnav_add_shortcuts(recordingInterface, "chiusura della mappa", map_edges=False,
                                         loop_closure_s=P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S)
    recordingInterface.optimize_anchoring()
    _t_close['chiusura mappa e ancoraggio'] = time.perf_counter() - _t

    _t = time.perf_counter()
    monitor = MISSION_CTX.get('batteria')
    if monitor is not None:
        monitor.controlla("inizio del ritorno a wp_0")
    _pct0 = MISSION_STATS.get('battery_last_pct')
    esito_wp0 = navGraphUtils.graphnav_return_to_start(recordingInterface, robot_state_client, command_client)
    if esito_wp0 in ('arrivato', 'vicino'):
        # Anche fermo vicino a wp_0 il tag si cerca: e' la misura della deriva di tutta la missione.
        misura = navGraphUtils.graphnav_check_start_fiducial(robot, robot_state_client)
        # Arrivato secondo GraphNav ma lontano secondo il tag: correzione (vedi
        # WP0_FIDUCIAL_CORRECTION). Fermo vicino (STUCK) no: la strada e' bloccata.
        if esito_wp0 == 'arrivato' and misura and misura.get('offset') is not None:
            navGraphUtils.graphnav_correct_wp0_with_fiducial(robot, recordingInterface, robot_state_client,
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

    local_grid = spotGrid.LocalGrid(robot)
    MISSION_CTX['local_grid'] = local_grid
    global_grid = spotGrid.GlobalGrid()
    MISSION_CTX['global_grid'] = global_grid   # per graphnav_add_shortcuts

    recordingInterface = navGraphUtils.RecordingInterface(robot, options.download_filepath, client_metadata)
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
            fiducials = navGraphUtils.find_fiducials(robot, robot_state_client,
                                                     wait_s=P.START_FIDUCIAL_WAIT_S,
                                                     fiducial_id=P.START_FIDUCIAL_ID)
            MISSION_CTX['start_fiducial'] = fiducials[0] if fiducials else None
            if fiducials:
                print(f"[FIDUCIAL] Fiducial di partenza: {navGraphUtils.describe_fiducial(fiducials[0])}"
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

            verification_tracker = arcVerification.ArcVerificationTracker(robot)
            MISSION_CTX['tracker'] = verification_tracker
            verification_tracker.start()

            recordingInterface.set_download_filepath(graph_folder)

            missione = Missione(robot_state_client, command_client, recordingInterface, local_grid,
                                global_grid, env, prm, gb_sampler, verification_tracker,
                                mission_folder, mobility_kwargs)
            missione.esplora()

            env.print_map()
            if _batteria_scarica():
                print("[BATTERIA] Missione chiusa prima di finire l'esplorazione: la chiusura e' "
                      "quella normale (scansione finale, riepilogo, ritorno a wp_0, mappa scaricata).")
            x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
            final_row, final_col = env.get_cell_from_world(x_final, y_final)
            recordingInterface.create_default_waypoint(cell_row=final_row, cell_col=final_col)

            missione.scansione_finale()

            # RIEPILOGO DI MISSIONE -- solo diagnostica, va letto dopo il volo.
            _emit_mission_summary_once()

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
        _emit_mission_summary_once()
        remove_stdout_tee()


if __name__ == '__main__':
    if not main():
        sys.exit(1)