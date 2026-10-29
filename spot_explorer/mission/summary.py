"""Interruttori e riepilogo di fine missione: solo diagnostica, mai eccezioni verso chi chiama."""
import os
import json

from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.perception import slopes
from spot_explorer.perception import spot_grid

from spot_explorer.planning import roadmap as prm_graph_module   # dentro le funzioni `prm_graph` e' spesso il parametro (istanza)
from spot_explorer.planning import mission_graph                   # reticolo dei nodi della missione

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, MISSION_CTX



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
        'slopes.ARC_BODY_EXTENSION_M': slopes.ARC_BODY_EXTENSION_M,
        'geometry.ROBOT_CLEARANCE_AIR_M': geometry.ROBOT_CLEARANCE_AIR_M,
        'geometry.PLAN_CLEARANCE_EXTRA_M': geometry.PLAN_CLEARANCE_EXTRA_M,
        'parametri.BATTERY_RETURN_PCT': P.BATTERY_RETURN_PCT,
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
            print(MISSION_CTX['batteria'].summary())
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
                'lateral_slope_cost_multiplier': slopes.LATERAL_SLOPE_COST_MULTIPLIER,
                'slope_threshold': slopes.SLOPE_THRESHOLD,
                'near_threshold_margin_fraction': slopes.NEAR_THRESHOLD_MARGIN_FRACTION,
                'slope_baseline_m': slopes.SLOPE_BASELINE_M,
                'lateral_slice_half_width_m': slopes.LATERAL_SLICE_HALF_WIDTH_M,
                'min_arc_sample_fraction': slopes.MIN_ARC_SAMPLE_FRACTION,
                'terrain_valid_min': spot_grid.TERRAIN_VALID_MIN,
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
