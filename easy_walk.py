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
from bosdyn.client.frame_helpers import *
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, blocking_stand)
from bosdyn.client.frame_helpers import get_a_tform_b
from types import SimpleNamespace
import navGraphUtils
import movements
import spotGrid
import spotLogInUtils
import environmentMap
import spotUtils
import arcVerification

import prm_graph
import prm_graph as prm_graph_module   # dentro le funzioni `prm_graph` e' spesso il parametro (istanza)
import mission_graph                   # reticolo dei nodi della missione

import parametri as P
from registro import (MISSION_STATS, MISSION_CTX, BodyTiltRecorder, SegmentLog,
                      TrajectoryLog, append_csv, append_trajectory, install_stdout_tee,
                      log_segment, mission_z0, remove_stdout_tee, save_npz_async,
                      set_mission_z0, timing_add,
                      visualize_grid_with_candidates)


# =========================================================================
# STRUMENTI DIAGNOSTICI -- tutto quello che segue serve SOLO a registrare cosa succede
# durante la missione. Nessuna decisione del robot dipende da questi oggetti, e ogni
# funzione e' scritta per non sollevare mai eccezioni verso il chiamante: un log che
# fallisce non deve mai fermare o alterare un movimento.
# =========================================================================


def mission_switches():
    """
    Gli interruttori che cambiano come lavora la missione, con il loro valore adesso. Vanno nel
    log ([CONFIG] INTERRUTTORI) e in mission_summary.json, cosi' dopo una prova si sa con quali
    impostazioni e' stata fatta ogni missione (per i confronti acceso/spento).
    """
    return {
        'easy_walk.MISSION_GRAPH_SAMPLING': P.MISSION_GRAPH_SAMPLING,
        'easy_walk.INCREMENTAL_GRAPH_UPDATE': P.INCREMENTAL_GRAPH_UPDATE,
        'mission_graph.BATCH_SLOPES': mission_graph.BATCH_SLOPES,
        'prm_graph.REFRESH_BATCH_SLOPES': prm_graph_module.REFRESH_BATCH_SLOPES,
        'spotGrid.CLEARANCE_MAP_DECIDES': spotGrid.CLEARANCE_MAP_DECIDES,
        'easy_walk.PLAN_ONLY_EXPLORED_CELLS': P.PLAN_ONLY_EXPLORED_CELLS,
        'easy_walk.GRAPHNAV_SHORTCUTS': P.GRAPHNAV_SHORTCUTS,
        'easy_walk.GRAPHNAV_PRECHECK': P.GRAPHNAV_PRECHECK,
        'easy_walk.GRAPHNAV_SDK_LOOP_CLOSURE': P.GRAPHNAV_SDK_LOOP_CLOSURE,
        'easy_walk.GRAPHNAV_SDK_LOOP_BUDGET_S': P.GRAPHNAV_SDK_LOOP_BUDGET_S,
        'easy_walk.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S': P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S,
        'easy_walk.GRAPHNAV_SDK_LOOP_FAST': P.GRAPHNAV_SDK_LOOP_FAST,
        'easy_walk.GRAPHNAV_SDK_LOOP_FINAL_PRUNE': P.GRAPHNAV_SDK_LOOP_FINAL_PRUNE,
        'easy_walk.GRAPHNAV_SDK_LOOP_MAX_PATH_M': P.GRAPHNAV_SDK_LOOP_MAX_PATH_M,
        'easy_walk.SAVE_SCAN_BUNDLES': P.SAVE_SCAN_BUNDLES,
        'easy_walk.SAVE_EXTRA_LAYERS': list(P.SAVE_EXTRA_LAYERS),
        'movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT': movements.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT,
        'spotGrid.ARC_BODY_EXTENSION_M': spotGrid.ARC_BODY_EXTENSION_M,
        'spotGrid.ROBOT_CLEARANCE_AIR_M': spotGrid.ROBOT_CLEARANCE_AIR_M,
        'spotGrid.PLAN_CLEARANCE_EXTRA_M': spotGrid.PLAN_CLEARANCE_EXTRA_M,
    }


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
         dei criteri seguenti finche' uno risponde si'). Prima dal 22-10 si sceglieva il piu'
         vicino al centro e, se il grafo non ci arrivava, i tre successivi: stavano nella
         stessa zona isolata e la riserva e' fallita 9 volte su 10. Se nessun punto e'
         raggiungibile si sceglie comunque il migliore: il tentativo fallisce come prima;
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
        # Per il centro conta la sola scansione dal vivo: lo spazio libero imparato e' ancora
        # in prova (spotGrid.CLEARANCE_MAP_DECIDES) e cambierebbe il centro piu' del dovuto.
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
        ok, crumb = navGraphUtils.graphnav_step_back(env, recordingInterface, robot_state_client, command_client)
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
    if movements.check_and_recover_fall(robot_state_client, command_client,
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
        unwritten_value=spotGrid.raw_zero(grids_data), unwritten_scale=spotGrid.raw_scale(grids_data)
    )
    spotGrid.log_ground_filter(local_grid, "scan-iniziale")

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

    spotGrid.log_footprint_diagnostic(local_grid, pts, robot_x, robot_y, robot_yaw,
                              num_x, num_y, obstacle_mask, "scan-cella",
                              cells_obstacle_dist=cells_obstacle_dist,
                              rough_values=rough_values, is_valid=is_valid)

    # Aggiorna le mappe globali (occupazione e terreno) con i dati appena processati
    spotGrid.fold_scan_into_global_maps(global_map, global_terrain_map, pts, terrain_real, obstacle_mask,
                               is_valid, cells_obstacle_dist, (num_y, num_x),
                               (robot_x, robot_y, robot_yaw))

    # La mappa del terreno serve al PRM come ripiego per la pendenza
    prm_graph.set_global_terrain_map(global_terrain_map)

    # Mappe globali su disco (.pkl), a ogni tentativo
    if mission_folder is not None:
        pkl_map_path = os.path.join(mission_folder, "global_occupancy_map.pkl")
        global_map.save_map(pkl_map_path)
        pkl_terrain_path = os.path.join(mission_folder, "global_terrain_map.pkl")
        global_terrain_map.mission_z0 = MISSION_CTX.get('mission_z0')
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
        valid_2d=spotGrid.slope_usable_mask(is_valid, cells_obstacle_dist, (num_y, num_x)),
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
    mission_graph.rebuild_prm_edges(prm_graph, global_map)
    timing_add('costruzione grafo', time.perf_counter() - t_build)
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

    # Solo celle visitate + cella del robot + cella obiettivo (vedi PLAN_ONLY_EXPLORED_CELLS).
    mission_graph.restrict_planning_to_explored_cells(prm_graph, env, target_row, target_col,
                                         (robot_x, robot_y))

    # --- Nodi di partenza e di arrivo: nodi ESISTENTI del grafo ---------------------------
    # add_stop_node aggancia la posizione al nodo adatto invece di crearne uno. Il nodo del
    # robot serve PRIMA della scelta dell'obiettivo: si sceglie fra i punti che il grafo
    # collega al robot (vedi find_best_point_in_cell, criterio 2).
    robot_node_id = prm_graph.add_stop_node(
        robot_x, robot_y, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
        label="partenza tentativo")
    prm_graph.set_robot_node(robot_node_id, 0.0)

    def _reachable_from_robot(px, py):
        nid = prm_graph.anchor_node(px, py, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
        return nid is not None and prm_graph.find_path_dijkstra(robot_node_id, nid) is not None

    # Ricerca del punto target e pianificazione
    target_x, target_y, valid_samples, rejected_samples = find_best_point_in_cell(
        robot_x, robot_y, env, target_row, target_col, pts, cells_obstacle_dist, sampler,
        global_map=global_map, reachable=_reachable_from_robot)

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

    # Briciola GraphNav del punto di partenza del tentativo: e' quello a cui serve poter
    # tornare se la cella si rivela un vicolo cieco.
    navGraphUtils.graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client, robot_node_id,
                             target_row, target_col, force=True)
    # L'obiettivo e' il centro o un nodo del reticolo (lo sceglie find_best_point_in_cell fra
    # i punti del campionatore).
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
    arrived_near_goal = False   # arrivo accettato con il robot appena fuori dalla cella
    trap_state = {'n': 0}   # ritorni indietro in QUESTO tentativo, mai azzerato (vedi _trap_backtrack)

    while path_waypoints and len(path_waypoints) > 0:
        loop_iter += 1
        t_iter = time.perf_counter()
        if loop_iter > P.MAX_LOOP_ITERATIONS:
            print(f"[STALLO] {P.MAX_LOOP_ITERATIONS} giri del ciclo di navigazione senza arrivare: "
                  f"interrompo il tentativo sulla cella ({target_row},{target_col}).")
            if verification_tracker is not None:
                verification_tracker.clear_path()
            return False

        if movements.check_and_recover_fall(robot_state_client, command_client, global_map,
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
        if last_xy is not None and float(np.hypot(robot_x - last_xy[0], robot_y - last_xy[1])) < P.IDLE_MOVE_M:
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
        if idle_loops >= P.MAX_IDLE_LOOPS:
            MISSION_STATS['idle_loops'] += idle_loops
            print(f"[STALLO] {idle_loops} giri di ciclo consecutivi senza spostarsi di "
                  f"{P.IDLE_MOVE_M:.2f} m, con il robot in ({robot_x:.2f}, {robot_y:.2f}). "
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
            unwritten_value=spotGrid.raw_zero(grids_data_up), unwritten_scale=spotGrid.raw_scale(grids_data_up),
            return_unwritten=True
        )
        spotGrid.log_ground_filter(local_grid, "scan-passo")

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

        spotGrid.log_footprint_diagnostic(local_grid, pts_up, robot_x, robot_y, robot_yaw,
                                  num_x_up, num_y_up, obstacle_mask_updated, "scan-passo",
                                  cells_obstacle_dist=cells_obs_up,
                                  rough_values=rough_up, is_valid=is_valid_up)

        t_proc = time.perf_counter()
        _gm_core_up = spotGrid.fold_scan_into_global_maps(
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
            valid_2d=spotGrid.slope_usable_mask(is_valid_up, cells_obs_up, (num_y_up, num_x_up)),
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
        spotGrid.update_clearance_map(global_map, pts_up, snap, is_valid_up, unwritten_up,
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
        pruned_nodes = arcVerification.prune_prm_nodes_in_obstacles(prm_graph, snap, global_map=global_map)
        if pruned_nodes:
            MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
            print(f"[NODI] {len(pruned_nodes)} nodi scartati: meno di "
                  f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero dove li vediamo "
                  f"(primi: {pruned_nodes[:8]}).")

        # Quanto la mappa dello spazio libero si discosta dal criterio in uso (vedi
        # spotGrid.CLEARANCE_MAP_DECIDES). Finche' decide la mappa binaria questa riga e'
        # solo una misura: dice quanti archi cambierebbero verdetto commutando.
        spotGrid.log_clearance_gap(prm_graph, global_map)

        # Archi gia' rifiutati per rotazione DA QUESTO NODO: restano fuori finche' il robot
        # non si sposta. Lo scarto e' soft, quindi refresh_local_edge_weights li rimetterebbe
        # in gioco a ogni giro, e il robot potrebbe restare fermo alternando due archi
        # rifiutati senza mai arrivare alla ritirata. Il robot puo' restare agganciato allo
        # stesso nodo anche dopo essersi spostato, quindi lo scarto vale solo finche' il robot
        # e' dove la rotazione e' stata rifiutata.
        for (from_id, to_id), n_fails in rotation_fails.items():
            fx, fy = rotation_fail_xy.get((from_id, to_id), (robot_x, robot_y))
            if (from_id == robot_node_id and n_fails > 0
                    and float(np.hypot(robot_x - fx, robot_y - fy)) <= prm_graph_module.ANCHOR_ON_NODE_M):
                prm_graph_module.mark_edge_blocked_soft(prm_graph, from_id, to_id)

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
        k_short = arcVerification.shortcut_index((robot_x, robot_y), path_waypoints, snap, global_map,
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

        in_target_cell = (tuple(int(v) for v in env.get_cell_from_world(robot_x, robot_y))
                          == (int(target_row), int(target_col)))
        # Appena fuori dalla cella vale come dentro, per l'arrivo per tolleranza (vedi
        # ARRIVAL_OUTSIDE_CELL_M). Il 22-10 16:04: "Mancano 0.06 m ... ancora fuori dalla
        # cella (2,7)", con il robot fuori di 0.01 m.
        robot_border = environmentMap.cell_border_distance(env, target_row, target_col, robot_x, robot_y)
        near_goal = (not in_target_cell and robot_border is not None
                     and -robot_border <= P.ARRIVAL_OUTSIDE_CELL_M)
        decision = arcVerification.decide_next_move((robot_x, robot_y), path_waypoints, frontier, no_progress,
                                    in_target_cell=in_target_cell or near_goal)
        if decision['kind'] == 'arrived' and near_goal:
            arrived_near_goal = True
            MISSION_STATS['arrivals_near_goal'] += 1
            print(f"[OBIETTIVO] Il robot e' fuori dalla cella ({target_row},{target_col}) di "
                  f"{-robot_border:.2f} m, a {frontier['path_len']:.2f} m dall'obiettivo: vale come "
                  f"arrivo (fuori al massimo di {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m, il muso e' dentro).")
        if (decision['kind'] in ('wait', 'blocked') and not in_target_cell and not near_goal
                and frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M):
            print(f"[OBIETTIVO] Mancano {frontier['path_len']:.2f} m, entro la tolleranza di "
                  f"{P.GOAL_REACHED_TOLERANCE_M:.2f} m, ma il robot e' ancora fuori dalla cella "
                  f"({target_row},{target_col}): non vale come arrivo, proseguo come per un "
                  f"fronte bloccato.")
        next_node_id, next_x, next_y = path_waypoints[0]
        keep_heading = False     # sempre False (niente traverso): resta nei dati salvati e in navigate_to

        # Pacchetto della scansione salvato QUI, prima del blocco rotazione: i rami di
        # rotazione rifiutata/cono/arretramento/trappola fanno `continue`, e salvando dopo
        # quelle scansioni non finirebbero su disco. Il rifiuto si ricalcola offline da
        # obstacle_distance, robot_xyz, robot_yaw e decision_target.
        if P.SAVE_SCAN_BUNDLES and mission_folder is not None:
            gf = local_grid.last_ground_filter
            shape_up = (num_y_up, num_x_up)
            extra = {}
            if P.SAVE_EXTRA_LAYERS:
                t_x = time.perf_counter()
                for name, lay in local_grid.extra_layers(P.SAVE_EXTRA_LAYERS).items():
                    extra[name] = lay['values']
                    extra[f'{name}_origin'] = lay['origin']
                    extra[f'{name}_cell'] = lay['cell']
                timing_add('scansione', time.perf_counter() - t_x)
            save_npz_async(
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
                mission_z0=mission_z0(),
                terrain_raw_zero=np.nan if spotGrid.raw_zero(grids_data_up) is None else spotGrid.raw_zero(grids_data_up),
                terrain_scale=np.nan if not spotGrid.raw_scale(grids_data_up) else spotGrid.raw_scale(grids_data_up),
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
                **extra,
                decision_kind=decision['kind'],
                decision_target=np.array(decision.get('target', (np.nan, np.nan)), dtype=np.float64))
            MISSION_STATS['scan_bundles'] += 1

        # 4. ROTAZIONE -------------------------------------------------------------------
        # Il muso punta SEMPRE il punto da raggiungere. Se la rotazione non e' possibile NON
        # si cammina di traverso (vedi ROTAZIONE E ARRETRAMENTO in cima al file). Nell'ordine:
        # avanzare dentro il cono di rotazione possibile, arretrare dritti se da li' la
        # rotazione diventa possibile, riconoscere una trappola, scartare il tratto.
        if decision['kind'] == 'move':
            rot = arcVerification.rotation_plan(robot_yaw, (robot_x, robot_y), decision['target'],
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
                      f"{rotation_fails[edge_key]}/{P.MAX_ROTATION_FAILS_SAME_EDGE}.")

                # --- 4a. PRIMA DI TUTTO: posso ANDARE AVANTI dentro il cono di rotazione che
                # mi e' concesso? Spesso andare dritti, o quasi, avvicina all'obiettivo piu'
                # di qualunque arretramento (vedi plan_cone_move).
                cone_goal, _ = arcVerification.point_along(polyline, P.CONE_PATH_LOOKAHEAD_M)
                cone = arcVerification.plan_cone_move((robot_x, robot_y), robot_yaw, snap, cone_goal)
                if cone is not None:
                    print(f"[CONO] Non posso girarmi di {rot['dyaw_deg']:+.0f} gradi, ma ruotando di "
                          f"{cone['dyaw_deg']:+.0f} posso avanzare {cone['dist']:.2f} m "
                          f"({cone['reason']}) e avvicinarmi di {cone['gain']:.2f} m al percorso "
                          f"({cone_goal[0]:.2f}, {cone_goal[1]:.2f}, {P.CONE_PATH_LOOKAHEAD_M:.1f} m "
                          f"avanti lungo il percorso). Vado, invece di arretrare.")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    MISSION_STATS['cone_moves'] += 1
                    cx, cy = cone['target']
                    need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
                    with TrajectoryLog(robot_state_client, 'cono', (cx, cy)):
                        ok_c, dist_c, _ = movements.navigate_to(
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
                back = arcVerification.plan_straight_retreat((robot_x, robot_y), robot_yaw, snap)
                helps, margin_back = (False, None)
                if back is not None:
                    helps, margin_back = arcVerification.retreat_would_help(
                        snap, (back[0], back[1]), robot_yaw, rot['dyaw_deg'])
                    if not helps:
                        MISSION_STATS['retreats_suppressed'] += 1
                        mb = "" if margin_back is None else f" (margine {margin_back:+.2f} m da li')"
                        print(f"[ARRETRAMENTO] Potrei arretrare di {back[2]:.2f} m, ma da li' la "
                              f"rotazione di {rot['dyaw_deg']:+.0f} gradi resterebbe impossibile{mb}: "
                              f"non lo faccio, sposterei il problema di 60 cm.")
                if (back is not None and helps
                        and retreats_in_a_row < P.MAX_STRAIGHT_RETREATS_IN_A_ROW
                        and rotation_fails[edge_key] < P.MAX_ROTATION_FAILS_SAME_EDGE):
                    bx, by, bdist = back
                    retreats_in_a_row += 1
                    print(f"[ARRETRAMENTO] Indietro dritto di {bdist:.2f} m verso "
                          f"({bx:.2f}, {by:.2f}), senza girarmi, per fare spazio alla rotazione "
                          f"({retreats_in_a_row}/{P.MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi).")
                    if verification_tracker is not None:
                        verification_tracker.clear_path()
                    MISSION_STATS['straight_retreats'] += 1
                    need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
                    with TrajectoryLog(robot_state_client, 'arretramento', (bx, by)):
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
                elif helps and retreats_in_a_row >= P.MAX_STRAIGHT_RETREATS_IN_A_ROW:
                    print(f"[ARRETRAMENTO] Gia' {retreats_in_a_row} arretramenti di fila senza "
                          f"combinare niente: smetto di arretrare.")

                # Ne' il cono ne' l'arretramento: e' una trappola o solo un arco sbagliato?
                # Se e' una trappola non ha senso scartare archi uno dopo l'altro: si torna
                # indietro subito.
                trapped_rot, fan_rot = arcVerification.is_trapped((robot_x, robot_y), robot_yaw, snap)
                n_us_r, fan_txt_r = arcVerification.describe_escape_fan(fan_rot)
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
                if rotation_discards > P.MAX_ROTATION_DISCARDS:
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
                prm_graph_module.mark_edge_blocked_soft(prm_graph, robot_node_id, next_node_id)
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

        timing_add('scansione', t_scan - t_iter)
        timing_add('elaborazione griglia', t_proc - t_scan)
        timing_add('mappe globali', t_maps - t_proc)
        timing_add('PRM e ripianificazione', t_prm - t_maps)
        timing_add('fronte e decisione', t_front - t_prm)
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
                  f"({frontier['reason']}), entro la tolleranza di {P.GOAL_REACHED_TOLERANCE_M:.2f} m: "
                  f"considero raggiunto l'obiettivo.")
            break

        if decision['kind'] == 'wait':
            # Prima di insistere: l'obiettivo e' ancora raggiungibile sui dati FRESCHI? Il
            # punto viene scelto all'inizio del tentativo, da metri di distanza, quando la
            # cella e' ancora tutta ignota; arrivandoci puo' risultare a ridosso di un muro,
            # e allora nessun numero di scansioni lo rendera' raggiungibile.
            if goal_repicks < P.MAX_BLOCK_REPLANS and not arcVerification.goal_still_reachable(snap, target_x, target_y):
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
                      f"({goal_repicks}/{P.MAX_BLOCK_REPLANS}).")
                tried_goals.append((float(target_x), float(target_y)))
                _refuse_goal(env, target_row, target_col, target_x, target_y,
                             f"spazio libero dal vivo {od_goal:.2f} m")
                new_tx, new_ty, _vs, _rs = find_best_point_in_cell(
                    robot_x, robot_y, env, target_row, target_col, pts_up, cells_obs_up,
                    sampler, global_map=global_map, exclude=tried_goals,
                    reachable=_reachable_from_robot)
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
            if (early_replans < P.MAX_EARLY_REPLANS
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
                trapped, fan = arcVerification.is_trapped((robot_x, robot_y), robot_yaw, snap)
                n_us, fan_txt = arcVerification.describe_escape_fan(fan)
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
                      f"aspettare la conferma da fermo ({early_replans}/{P.MAX_EARLY_REPLANS}). "
                      f"Direzioni utilizzabili da qui: {n_us}/{len(fan)}.")
                # Scarto NON permanente: dopo una sola scansione l'arco va tolto dal piano,
                # non condannato per la missione (vedi mark_edge_blocked_soft).
                prm_graph_module.mark_edge_blocked_soft(prm_graph, ea, eb)
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
            print(f"[FRONTE] Non posso avanzare nemmeno di {P.MIN_PARTIAL_STEP_M:.2f} m: riguardo "
                  f"da fermo ({no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni).")
            time.sleep(P.FRONTIER_RESCAN_WAIT_S)
            continue

        if decision['kind'] == 'blocked':
            k = decision['segment']
            edge_a = robot_node_id if k == 0 else path_waypoints[k - 1][0]
            edge_b = path_waypoints[min(k, len(path_waypoints) - 1)][0]
            sx, sy = frontier['stop_xy']
            print(f"[BLOCCO] Confermato da vicino: {P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni consecutive "
                  f"senza poter avanzare. Motivo: {frontier['reason']} in ({sx:.2f}, {sy:.2f}), "
                  f"sul tratto {edge_a}->{edge_b}.")
            MISSION_STATS['frontier_blocks'] += 1
            # Bloccato a ridosso dell'obiettivo: e' l'obiettivo a non andare, non la strada.
            if (frontier['reason'] in ('ostacolo', 'rugosita', 'pendenza')
                    and float(np.hypot(sx - target_x, sy - target_y)) <= P.REFUSED_GOAL_BLOCK_M):
                _refuse_goal(env, target_row, target_col, target_x, target_y,
                             f"blocco confermato a {float(np.hypot(sx - target_x, sy - target_y)):.2f} m "
                             f"da lui ({frontier['reason']})")

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
            trapped_blk, fan_blk = arcVerification.is_trapped((robot_x, robot_y), robot_yaw, snap)
            n_us_b, fan_txt_b = arcVerification.describe_escape_fan(fan_blk)
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
            if k == 0 and not mission_graph.robot_on_node(prm_graph, edge_a, robot_x, robot_y):
                prm_graph_module.mark_edge_blocked_soft(prm_graph, edge_a, edge_b)
            else:
                prm_graph.mark_edge_invalid(edge_a, edge_b)
            block_replans += 1
            no_progress = 0

            new_ids = None
            if block_replans <= P.MAX_BLOCK_REPLANS:
                new_ids = prm_graph.find_path_dijkstra(robot_node_id, goal_id)
            if new_ids is not None:
                path_waypoints = _waypoints_from_ids(new_ids)
                full_path_coords = [prm_graph.get_node_position(nid) for nid in new_ids]
                print(f"[REPLAN] Dopo il blocco ({block_replans}/{P.MAX_BLOCK_REPLANS}): percorso "
                      f"alternativo con {len(path_waypoints)} waypoint, dal punto in cui sono.")
                continue

            print(f"[BLOCCO] Nessuna alternativa dal punto in cui sono"
                  + (f" (gia' {P.MAX_BLOCK_REPLANS} ripianificazioni dopo blocchi)."
                     if block_replans > P.MAX_BLOCK_REPLANS else ".")
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
        if (arrival_refusals < P.MAX_ARRIVAL_REFUSALS
                and (step_len <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_arrival <= P.ARRIVAL_EXIT_CHECK_OD_M)):
            heading_arr = float(np.arctan2(move_y - robot_y, move_x - robot_x))
            has_exit, fan_arr = arcVerification.arrival_has_exit(snap, (move_x, move_y), heading_arr)
            if not has_exit:
                arrival_refusals += 1
                MISSION_STATS['arrival_refusals'] += 1
                n_us_a, fan_txt_a = arcVerification.describe_escape_fan(fan_arr)
                print(f"[ARRIVO] Il passo di {step_len:.2f} m verso ({move_x:.2f}, {move_y:.2f}) "
                      f"(obstacle_distance li' {od_arrival:.2f} m) mi porterebbe dove non ci sono "
                      f"uscite: {n_us_a}/{len(fan_arr)} direzioni utilizzabili ({fan_txt_a}). "
                      f"Non ci vado ({arrival_refusals}/{P.MAX_ARRIVAL_REFUSALS}), scarto il tratto "
                      f"{robot_node_id}->{next_node_id} e ripianifico.")
                prm_graph_module.mark_edge_blocked_soft(prm_graph, robot_node_id, next_node_id)
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
                rot_leg = arcVerification.rotation_plan(robot_yaw, (robot_x, robot_y), (move_x, move_y),
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
                if step_leg <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_leg <= P.ARRIVAL_EXIT_CHECK_OD_M:
                    has_exit_leg, _fan_leg = arcVerification.arrival_has_exit(
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

            seg_diag = spotGrid.segment_diagnostics(terrain_up_2d, grid_origin_x_up, grid_origin_y_up, cell_size_up,
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
                segment_mobility_kwargs = movements.select_mobility_params(segment_gradient, segment_roughness)
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
                        and float(np.hypot(frx - robot_x, fry - robot_y)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
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
            tilt_recorder = BodyTiltRecorder(robot_state_client)
            tilt_recorder.start()
            try:
                need_refresh = True              # fine di un movimento: vedi REFRESH A EVENTI
                success_move, distance_traveled, distance_commanded = movements.navigate_to(
                    move_x, move_y, robot_x, robot_y, robot_state_client, command_client,
                    vision_tform_body_current, should_abort=_should_abort_move,
                    keep_heading=keep_heading,
                    **segment_mobility_kwargs
                )
            finally:
                tilt_stats = tilt_recorder.stop()
            move_duration = time.time() - move_t0
            timing_add('movimento', move_duration)
            try:   # solo log: non deve mai interrompere la missione
                _seg_id = [iteration, target_row, target_col, loop_iter, robot_node_id, next_node_id]
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
                seg_dist = float(np.hypot(move_x - robot_x, move_y - robot_y))
                cost_dist = prm_graph.alpha * seg_dist
                cost_long = prm_graph.beta * segment_long_slope
                cost_lat = prm_graph.beta_lateral * segment_lat_slope
                cost_rough = prm_graph.gamma * segment_roughness
                log_segment({
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
                    if mission_graph.robot_on_node(prm_graph, robot_node_id, robot_x, robot_y):
                        prm_graph.traversed_edges.add((min(robot_node_id, next_node_id),
                                                       max(robot_node_id, next_node_id)))
                    robot_node_id = next_node_id
                    prm_graph.set_robot_node(robot_node_id, 0.0)
                    path_waypoints.pop(0)

                # Briciola per i ritorni: un waypoint GraphNav ogni
                # GRAPHNAV_WAYPOINT_MIN_SPACING_M di strada percorsa.
                navGraphUtils.graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client,
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

                movements.check_and_recover_fall(robot_state_client, command_client, global_map,
                                        "dopo il fallimento del movimento")

                if distance_traveled > 0.05:
                    if not hasattr(env, '_traveled_arcs'):
                        env._traveled_arcs = []
                    ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
                    env._traveled_arcs.append((robot_x, robot_y, ax, ay))

                if mission_graph.robot_on_node(prm_graph, robot_node_id, robot_x, robot_y):
                    prm_graph.mark_edge_invalid(robot_node_id, next_node_id)
                else:   # il robot non partiva dal nodo: il tratto fallito non e' l'arco del grafo
                    prm_graph_module.mark_edge_blocked_soft(prm_graph, robot_node_id, next_node_id)
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
    if arrived_near_goal:
        return True
    return env.is_point_in_cell(robot_x, robot_y, target_row, target_col)


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


def easy_walk(options):
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
        install_stdout_tee(mission_log_path)
        try:
            MISSION_CTX['log_folder'] = mission_log_folder
            MISSION_CTX['segment_log'] = SegmentLog(os.path.join(mission_log_folder, "segments.csv"))
            MISSION_CTX['summary_done'] = False
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
            print(f"[CONFIG] OBIETTIVO: tolleranza d'arrivo {P.GOAL_REACHED_TOLERANCE_M:.2f} m "
                  f"(= meta' corpo + margine del fronte), riscelta del punto fino a "
                  f"{P.MAX_BLOCK_REPLANS} volte se risulta irraggiungibile; ripianificazione alla "
                  f"PRIMA cella bloccante, fino a {P.MAX_EARLY_REPLANS} volte per tentativo")
            print(f"[CONFIG] SCELTA OBIETTIVO: solo punti raggiungibili dal robot nel grafo (se ce "
                  f"ne sono), spazio libero preferito {P.TARGET_CLEARANCE_M:.2f} m (minimo "
                  f"{spotGrid.PLAN_CLEARANCE_M:.2f}), almeno {P.TARGET_CELL_BORDER_M:.2f} m dal bordo "
                  f"della cella, obiettivi rifiutati dal fronte esclusi per la missione; arrivo "
                  f"valido anche col robot fuori dalla cella fino a {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m")
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
        # test_replay legge metodo e archi da questa riga; distribuzione, densita' e seme
        # dalla riga [GRAFO] Nodi casuali, scritta quando il grafo e' costruito.
        print(f"[CONFIG] Grafo: nodi {P.MISSION_GRAPH_SAMPLING}"
              + (f" ({P.MISSION_RANDOM_DISTRIBUTION}, {P.MISSION_RANDOM_DENSITY:g} nodi/m2, seme "
                 f"{'a caso' if P.MISSION_RANDOM_SEED is None else P.MISSION_RANDOM_SEED})"
                 if P.MISSION_GRAPH_SAMPLING == 'casuale' else "")
              + f"; archi da {P.PRM_MIN_EDGE_M:g} a {P.PRM_MAX_EDGE_M:g} m")
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
        set_mission_z0(robot_state_client, z_boot)

        # Nodi del PRM: il grafo della missione (vedi _build_mission_prm e mission_graph.py).
        # Il punto di accensione e i centri cella sono nodi del grafo, con entrambi i metodi.
        prm, gb_sampler, mission_lattice = mission_graph.build_mission_prm(env)
        MISSION_CTX['prm'] = prm  # per il riepilogo di fine missione (anche se interrotta)

        # Gli ARCHI non si creano qui: non c'e' ancora nessun dato di terreno ne' di
        # occupazione. Li crea attempt_enter_cell_from_position() alla prima scansione
        # (vedi _rebuild_prm_edges).
        verification_tracker = arcVerification.ArcVerificationTracker(robot)
        MISSION_CTX['tracker'] = verification_tracker
        verification_tracker.start()

        recordingInterface.set_download_filepath(graph_folder)
        path = env.generate_serpentine_path(start_cell=env.start_cell)

        frontier = []
        visualization_counter = 0

        x, y, z, _ = spotUtils.getPosition(robot_state_client)
        robot_row, robot_col = env.get_cell_from_world(x, y)
        frontier.extend(environmentMap.find_new_borders(env, robot_row, robot_col, path, frontier))

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
                    frontier.extend(environmentMap.find_new_borders(env, robot_row, robot_col, path, frontier))
                else:
                    if (selected_border[0], selected_border[1]) == (robot_row, robot_col):
                        env.update_position(x_new, y_new)
                        recordingInterface.create_default_waypoint(cell_row=selected_border[0],
                                                                   cell_col=selected_border[1])
                        env.add_waypoint(x_new, y_new)
                        env.mark_cell_visited(selected_border[0], selected_border[1])
                        frontier.extend(environmentMap.find_new_borders(env, robot_row, robot_col, path, frontier))
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
                                unwritten_value=spotGrid.raw_zero(grids_data_rec), unwritten_scale=spotGrid.raw_scale(grids_data_rec)
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
                                valid_2d=spotGrid.slope_usable_mask(is_valid_rec, cells_obstacle_dist, (num_y, num_x)),
                                grid_origin_x=grid_origin_x,
                                grid_origin_y=grid_origin_y,
                                cell_size=cell_size
                            )
                            mission_graph.rebuild_prm_edges(prm, global_grid.global_occupancy_map)

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
                            mission_graph.restrict_planning_to_explored_cells(
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
                                target_x - last_goal[0], target_y - last_goal[1])) <= P.RECOVERY_SAME_GOAL_M)
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
                                        environmentMap.find_new_borders(env, robot_row_final, robot_col_final, path, frontier))
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
                    # Prima di partire con GraphNav: da quel waypoint il grafo arriva alla
                    # cella? Se no, il tentativo fallirebbe appena arrivati (vedi
                    # GRAPHNAV_PRECHECK): il lato si segna come provato senza muoversi.
                    if P.GRAPHNAV_PRECHECK:
                        cand_wp = recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell)
                        wp_xy = (cand_wp.get('x'), cand_wp.get('y')) if isinstance(cand_wp, dict) else (None, None)
                        reach = (mission_graph.graph_reaches_cell(prm, env, wp_xy, candidate_row, candidate_col,
                                                    global_grid.global_occupancy_map)
                                 if wp_xy[0] is not None else None)
                        if reach is False:
                            MISSION_STATS['graphnav_trips_avoided'] += 1
                            print(f"[PRECHECK] Cella ({candidate_row},{candidate_col}): dal waypoint "
                                  f"della cella {candidate_nearest_cell} il grafo non arriva a nessun "
                                  f"nodo della cella (ostacoli gia' in mappa). Segno il lato come "
                                  f"provato senza spostarmi.")
                            env.mark_cell_side_explored(candidate_row, candidate_col, candidate_side_bit)
                            finalize_or_defer_blocked_cell(env, candidate_row, candidate_col,
                                                           gb_sampler, global_grid.global_occupancy_map)
                            if env.is_cell_blocked(candidate_row, candidate_col):
                                frontier[:] = [c for c in frontier
                                               if (c[0], c[1]) != (candidate_row, candidate_col)]
                            already_skipped_this_round.add((candidate_row, candidate_col))
                            continue
                    target_row, target_col, rank = candidate_row, candidate_col, candidate_rank
                    nearest_cell = candidate_nearest_cell
                    break

                if target_row is not None:
                    x_current, y_current, _, _ = spotUtils.getPosition(robot_state_client)

                    nearest_wp = recordingInterface.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
                    navGraphUtils.graphnav_add_shortcuts(recordingInterface, "prima dello spostamento fra celle",
                                           map_edges=True, sdk=False, target_wp_id=nearest_wp['id'],
                                           robot_xy=(x_current, y_current))
                    recordingInterface.stop_recording()
                    navigation_success = navGraphUtils.graphnav_navigate_to(
                        recordingInterface, nearest_wp['id'], robot_state_client, command_client,
                        f"verso il waypoint della cella {nearest_cell} per la cella ({target_row},{target_col})")

                    # GraphNav non registra i tratti percorsi: quelli vecchi non descrivono piu'
                    # la strada da cui il robot e' arrivato, la ritirata non deve usarli.
                    env._traveled_arcs = []
                    if movements.check_and_recover_fall(robot_state_client, command_client,
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
                            frontier.extend(environmentMap.find_new_borders(env, robot_row, robot_col, path, frontier))
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
                unwritten_value=spotGrid.raw_zero(grids_data_final), unwritten_scale=spotGrid.raw_scale(grids_data_final)
            )
            spotGrid.log_ground_filter(local_grid, "scan-finale")

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
            spotGrid.fold_scan_into_global_maps(
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
        navGraphUtils.graphnav_add_shortcuts(recordingInterface, "prima del ritorno a wp_0",
                               map_edges=True, sdk=False, target_wp_id=_wp0_id, robot_xy=_xy_end)
        recordingInterface.auto_close_loops(True, False)
        recordingInterface.stop_recording()
        # Chiusura d'anello di Spot una volta sola, PRIMA dell'ancoraggio: i suoi archi,
        # verificati sulle scansioni, correggono la deriva della mappa scaricata e accorciano
        # il ritorno (vedi GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
        navGraphUtils.graphnav_add_shortcuts(recordingInterface, "chiusura della mappa", map_edges=False,
                               sdk=True, sdk_budget_s=P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S, sdk_fast=False)
        recordingInterface.optimize_anchoring()
        _t_close['chiusura mappa e ancoraggio'] = time.perf_counter() - _t

        _t = time.perf_counter()
        if not navGraphUtils.graphnav_return_to_start(recordingInterface, robot_state_client, command_client):
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