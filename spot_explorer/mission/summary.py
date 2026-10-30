"""Switches and end-of-mission summary: diagnostics only, never raises to the caller."""
import os
import json

from spot_explorer.robot import motion
from spot_explorer.perception import geometry
from spot_explorer.perception import slopes
from spot_explorer.perception import spot_grid

from spot_explorer.planning import edge_evaluation
from spot_explorer.planning import mission_graph                   # mission node lattice

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, MISSION_CTX



# =========================================================================
# SWITCHES AND SUMMARY -- diagnostics only: no robot decision depends on these
# functions, and they never raise to the caller (a failing log must not stop or alter
# a movement). Logs and statistics live in records.py.
# =========================================================================


def mission_switches():
    """
    The switches that change how the mission works, with their current value. They go to
    the log ([CONFIG] SWITCHES) and to mission_summary.json, so after a test it is known
    which settings each mission ran with (for on/off comparisons).
    """
    return {
        'config.MISSION_GRAPH_SAMPLING': P.MISSION_GRAPH_SAMPLING,
        'mission_graph.BATCH_SLOPES': mission_graph.BATCH_SLOPES,
        'edge_evaluation.REFRESH_BATCH_SLOPES': edge_evaluation.REFRESH_BATCH_SLOPES,
        'config.PLAN_ONLY_EXPLORED_CELLS': P.PLAN_ONLY_EXPLORED_CELLS,
        'config.GRAPHNAV_SHORTCUTS': P.GRAPHNAV_SHORTCUTS,
        'config.GRAPHNAV_PRECHECK': P.GRAPHNAV_PRECHECK,
        'config.REPICK_ON_REPLAN_FAIL': P.REPICK_ON_REPLAN_FAIL,
        'config.GRAPHNAV_ARRIVAL_HEADING': P.GRAPHNAV_ARRIVAL_HEADING,
        'config.WP0_FIDUCIAL_CORRECTION': P.WP0_FIDUCIAL_CORRECTION,
        'config.MOVE_NEAR_GOAL_M': P.MOVE_NEAR_GOAL_M,
        'config.GRAPHNAV_SDK_LOOP_CLOSURE': P.GRAPHNAV_SDK_LOOP_CLOSURE,
        'config.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S': P.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S,
        'config.SAVE_SCAN_BUNDLES': P.SAVE_SCAN_BUNDLES,
        'config.SAVE_EXTRA_LAYERS': list(P.SAVE_EXTRA_LAYERS),
        'motion.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT': motion.DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT,
        'slopes.ARC_BODY_EXTENSION_M': slopes.ARC_BODY_EXTENSION_M,
        'geometry.ROBOT_CLEARANCE_AIR_M': geometry.ROBOT_CLEARANCE_AIR_M,
        'geometry.PLAN_CLEARANCE_EXTRA_M': geometry.PLAN_CLEARANCE_EXTRA_M,
        'config.BATTERY_RETURN_PCT': P.BATTERY_RETURN_PCT,
    }


def _emit_mission_summary_once():
    """Print and save the mission summary only once. Called both at the end of a normal
    mission and by main() in a finally block, so it comes out even if the mission ends
    with an exception or because control was taken back."""
    if MISSION_CTX.get('summary_done') or MISSION_CTX.get('prm') is None:
        return
    MISSION_CTX['summary_done'] = True
    try:
        prm = MISSION_CTX['prm']
        prm.print_mission_summary()
        total_gait = sum(MISSION_STATS['gait_tier_counts'].values())
        print("================ [MISSION STATS] ================")
        print(f"Gait tier distribution (total {total_gait} steps):")
        for tier, count in MISSION_STATS['gait_tier_counts'].items():
            pct = (100.0 * count / total_gait) if total_gait > 0 else 0.0
            print(f"  {tier:10s}: {count:6d}  ({pct:5.1f}%)")
        print(f"Total replans: {MISSION_STATS['replan_count']}")
        print(f"Segments recorded in the CSV: {MISSION_STATS['segments_logged']}")
        print(f".npz saves for edges near the threshold: {MISSION_STATS['near_threshold_saves']}")
        print(f"Blocks confirmed by the frontier: {MISSION_STATS['frontier_blocks']}, "
              f"stops halfway through a move: {MISSION_STATS['frontier_aborts']}, "
              f"rotations refused: {MISSION_STATS['rotation_refusals']}, "
              f"straight back-offs: {MISSION_STATS['straight_retreats']}, "
              f"goals chosen again: {MISSION_STATS['goal_repicks']}, "
              f"shortcuts: {MISSION_STATS['shortcuts']}")
        print(f"Advances in the rotation cone: {MISSION_STATS['cone_moves']}, "
              f"back-offs avoided as useless: {MISSION_STATS['retreats_suppressed']}, "
              f"immediate replans: {MISSION_STATS['early_replans']}")
        print(f"Returns with GraphNav: {MISSION_STATS['graphnav_backtracks']} successful, "
              f"{MISSION_STATS['graphnav_backtrack_fail']} failed (walked fallback); "
              f"PRM nodes discarded as inside obstacles: {MISSION_STATS['nodes_pruned']}")
        print(f"Goals excluded for the mission: {MISSION_STATS['goals_refused']}, choices without "
              f"reachable points: {MISSION_STATS['goals_unreachable']}, arrivals with the robot just "
              f"outside the cell: {MISSION_STATS['arrivals_near_goal']}, returns without "
              f"movement: {MISSION_STATS['backtracks_in_place']}")
        print(f"Traps recognized by the fan: {MISSION_STATS['traps_detected']}, "
              f"steps refused for lack of exits on arrival: {MISSION_STATS['arrival_refusals']}")
        tot_r = MISSION_STATS['prm_refresh'] + MISSION_STATS['prm_refresh_skipped']
        mn = MISSION_STATS['multinode_moves']
        print(f"Multi-node moves: {mn}"
              + (f", {MISSION_STATS['multinode_nodes']} nodes crossed "
                 f"({MISSION_STATS['multinode_nodes'] / mn:.1f} per move)" if mn else "")
              + f"; stopped at an intermediate node (rotation or arrival): {MISSION_STATS['multinode_cut']}")
        print(f"PRM edge refreshes: {MISSION_STATS['prm_refresh']} done, "
              f"{MISSION_STATS['prm_refresh_skipped']} skipped because without events"
              + (f" ({100.0 * MISSION_STATS['prm_refresh_skipped'] / tot_r:.0f}% of turns saved)"
                 if tot_r else ""))
        print(f"Falls: {MISSION_STATS['falls']}, got up by itself: {MISSION_STATS['self_rights']}")
        print(f"Legs not completed but arrived within {P.MOVE_NEAR_GOAL_M:.2f} m of the point (valid): "
              f"{MISSION_STATS.get('moves_near_goal', 0)}")
        print(f"GraphNav moves successful: {MISSION_STATS['graphnav_ok']}, failed: {MISSION_STATS['graphnav_fail']}")
        print(f"Odometry drift: {MISSION_STATS['drift_samples']} measurements at GraphNav arrivals, "
              f"maximum {MISSION_STATS['drift_max_m']:.2f} m, last {MISSION_STATS['drift_last_m']:.2f} m "
              f"(details in drift.csv)")
        print(f"Spot loop closure: {MISSION_STATS['sdk_loop_calls']} calls, "
              f"{MISSION_STATS['sdk_loop_s']:.0f} s, {MISSION_STATS['sdk_loop_edges']} edges, "
              f"{MISSION_STATS['sdk_loop_partial']} ended at the time limit, "
              f"{MISSION_STATS['sdk_loop_failures']} failed attempts")
        print(f"GraphNav re-anchorings (robot lost): {MISSION_STATS.get('graphnav_relocalizations', 0)} successful, "
              f"{MISSION_STATS.get('graphnav_relocalization_fail', 0)} failed | cells deferred after "
              f"{P.MAX_BACKTRACKS_PER_ATTEMPT} returns in the same attempt: {MISSION_STATS.get('backtrack_giveups', 0)} | "
              f"edges removed by the live scan in the refreshes: {MISSION_STATS.get('live_arc_vetoes', 0)}")
        lg_main = MISSION_CTX.get('local_grid')
        lg_trk = getattr(MISSION_CTX.get('tracker'), '_local_grid', None)
        print(f"Scans discarded (layers missing or misaligned): main loop "
              f"{getattr(lg_main, 'rejected_scans', 0)}, check while moving {getattr(lg_trk, 'rejected_scans', 0)}")
        print(f"Scan bundles saved: {MISSION_STATS['scan_bundles']}")
        if MISSION_CTX.get('battery') is not None:
            print(MISSION_CTX['battery'].summary())
        times = MISSION_STATS['time_s']
        if times:
            tot = sum(times.values())
            print(f"[TIMING] Breakdown of the measured time ({tot:.0f} s):")
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
                'switches': mission_switches(),
                'start_fiducial': MISSION_CTX.get('start_fiducial'),
            }
            mg = getattr(prm, 'mission_graph', None)
            if mg is not None:
                summary['graph'] = mg.describe()
            with open(os.path.join(folder, 'mission_summary.json'), 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, default=str)
    except Exception as e:
        print(f"[LOG] Could not produce the mission summary: {e}")
