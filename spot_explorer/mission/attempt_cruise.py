"""
The cruise of CellAttempt (see cell_attempt.py), switch CONTINUOUS_MOTION: the robot walks
along the path without stopping at the nodes, while the frontier thread runs the whole
planning cycle (planning_cycle.CruisePlanner) and the main thread sends it the stretch
confirmed free of every new plan (motion.PathCommand).

    stop-and-go decides 'move' --> _cruise():
        send the stretch confirmed by the frontier of the stop-and-go
        thread: scan, maps, edges, Dijkstra, frontier every PLAN_PERIOD_S
        main:   at every new plan, send its confirmed stretch (it replaces the previous
                one without stopping the robot); the robot never goes beyond the stretch
                confirmed by the last plan
    back to the stop-and-go (a turn with a fresh scan, as after a move) when:
        arrived at the goal; the confirmed stretch ahead of the robot ends (obstacle, edge
        of the data, corner where the rotation is not clear); the edge being walked is out
        of the graph; no path; the command ends badly (stopped, off the path, Spot
        reports it blocked); no new plan for CRUISE_PLAN_STALE_S; fall (checked every
        CRUISE_FALL_CHECK_S); low battery; CRUISE_MAX_S.

While the thread plans the main thread does not touch graph and maps: it sends commands,
reads plans and the robot state. It takes them back after stop_planning().

WITHOUT STOPPING, while the robot walks on the last stretch confirmed (the command lasts
PATH_HOLD_S, so the main thread has that long):
  - goal isolated (plan 'no_path'): the planning pauses for the cycle running to end, a new
    goal is chosen in the cell (_repick_goal, as the stop-and-go does) and the cruise goes
    on towards it (_cruise_repick); with no other goal the robot stops and the attempt ends
    (FAILED: the side is marked as tried);
  - CHAIN_CELLS: inside the target cell and within CRUISE_HANDOFF_M of the goal (or within
    GOAL_REACHED_TOLERANCE_M with the last stretch closed by an obstacle), the attempt
    ends successfully with the robot still walking towards the goal (env._moving_pc, last
    command over the whole confirmed stretch, not PATH_HOLD_S): the
    mission marks the cell and the next attempt starts while the robot walks. Its first turn
    goes on in cruise if the rotation is clear: within PATH_START_IN_PLACE_DEG of the nose it
    turns while walking, beyond it on the spot inside the new command if the swept area is
    clear with CHAIN_TURN_EXTRA_AIR_M more air (_moving_check); otherwise, and for any other
    decision, the robot is stopped first and the turn is made standing as today. An attempt
    that fails before its loop (no valid goal, cell not reachable in the graph: only
    computations) leaves the robot walking, and the mission goes on to the next cell. stop_if_moving stops it before
    GraphNav and at the end of the exploration.
"""
import functools
import re
import threading
import time

import numpy as np

from spot_explorer import config as P
from spot_explorer.graphnav import map_recording
from spot_explorer.mission import planning_cycle
from spot_explorer.mission import mission_prm
from spot_explorer.planning import goal as goal_choice
from spot_explorer.mission.attempt_common import CONTINUE, STOP, FAILED, battery_return_due
from spot_explorer.perception import gait
from spot_explorer.perception import geometry
from spot_explorer.planning import decisions
from spot_explorer.planning import roadmap
from spot_explorer.planning import safe_frontier
from spot_explorer.records import HANDOFF, MISSION_STATS, BodyTiltRecorder, append_trajectory, timing_add
from spot_explorer.robot import connection
from spot_explorer.robot import motion
from spot_explorer.robot import photos

CONTINUOUS_MOTION = True
CRUISE_POLL_S = 0.2             # main thread: one feedback reading every 0.2 s
CRUISE_PLAN_STALE_S = 1.2       # no new plan for this long: stop
CRUISE_MIN_AHEAD_M = 0.10       # confirmed stretch ahead of the robot below this: stop
CRUISE_BAD_PLANS = 3            # consecutive plans without scan or with an error: stop
CRUISE_MAX_S = 180.0
# Cruises in a row that walk less than CRUISE_STALL_M (not arrived, no hand-off) before the
# attempt is given up. On 09-11 14:53 the goal of (0,5) was 9 cm from a wall: the attempt
# planned a way back through a node behind the robot (the frontier confirmed 1.26 m of it),
# the cruise put the robot on the last edge, already blocked (0.00 m ahead), and so on for 61
# cruises of 2-5 cm in 90 s. The other nets did not trigger: every cruise moves a little.
CRUISE_STALL_M = 0.10
MAX_CRUISE_STALLS = 3
CRUISE_TRACK_STEP_M = 0.30      # spacing of the walked track kept for the retreat
CRUISE_BREADCRUMB_S = 1.0       # how often the GraphNav breadcrumb is tried
CRUISE_BATTERY_S = 5.0          # how often the battery is read
CRUISE_FALL_CHECK_S = 1.0       # how often the fall is checked (the stop-and-go does it at every turn)
CRUISE_ARRIVAL_WAIT_S = 2.0     # at the goal: wait at most this for the command to end
CRUISE_SETTLE_S = 1.0           # after a stop: wait at most this for the robot to stand still
CHAIN_CELLS = True              # from a cell goal to the next cell without stopping (see above)
# Distance from the goal at which the next cell is chosen: the robot walks it while the next
# attempt is computed. With 0.6 m (~2 s) it stood still at 46 hand-offs out of 58 on 04-11 14:08.
CRUISE_HANDOFF_M = 1.0
# The attempt that starts with the robot still walking (CHAIN_CELLS) uses the last scan of the
# cruise, already processed and in the global maps, instead of downloading and processing a
# new one (attempt_scan._first_scan), if it is at most CHAIN_SCAN_MAX_AGE_S old. It serves
# the graph update and the goal choice; the first move still waits for the fresh scan and
# frontier of the first turn. Why: on 04-11 16:52 (CORE I/O) the scan at the attempt start
# with its processing and maps cost ~0.5-1 s of the 4.1-4.7 s hand-offs, on data 0.4 s newer
# than those the cruise had just put in the maps.
CHAIN_REUSE_SCAN = True
# The cruise also hands off as soon as the target cell is seen for at least
# cell_attempt.CELL_SEEN_FRACTION in the global map (with CELL_SEEN_EXPLORED), wherever the
# robot is: the cell exists to be seen, the goal is only where to see it from (06-11).
CHAIN_ON_SEEN = True
# STEP C (09-11): when the target cell is done the cruise does not end. The planning thread,
# the only one touching graph, maps and cells, marks the cell, chooses the next one with the
# cell rules (cell_rules, only 'near' choices: within NEAR_PATH_M), its goal and the path,
# and goes on planning towards it (_prepare_next_cell, run by CruisePlanner.switch_job);
# meanwhile the robot walks the last confirmed stretch. The main thread only makes the
# GraphNav waypoint of the cell done and logs. When the next cell cannot be prepared (far,
# none, no goal with a path) the cruise ends with the hand-off as before. Needs the cell rules
# of the mission (env._cell_rules, set by exploration.Mission._explore_rules).
CRUISE_SWITCH = True
# The main thread waits at most this for the next cell prepared by the thread; then the cruise
# ends with the hand-off as before (the robot walks the stretch sent, ~3 s).
CRUISE_SWITCH_WAIT_S = 3.0
CHAIN_SCAN_MAX_AGE_S = 2.0
# Gait in cruise: back to a lighter level only after the plans have asked for it this long
# (04-11 14:08: 239 changes in 20 min, one every 5 s, without it).
GAIT_DOWN_HOLD_S = 2.0
# The locomotion_hint (AUTO <-> CRAWL) changes only after this long of plans asking for it,
# both ways: at every change while walking Spot made a side step of 10-15 cm (04-11 14:08:
# 10 of the 12 sideways jumps of the cruise came within 2 s of one of the 118 changes; cell
# (0,1): MODERATE, PLAIN, MODERATE in one second, then 15 cm sideways). On that mission it
# would leave 29 changes instead of 117.
GAIT_HINT_HOLD_S = 3.0
# A bigger turn than PATH_START_IN_PLACE_DEG with the robot still walking: rotation on the spot
# inside the new command, without stopping first, if the swept area is clear with this much
# more air. The check is made at the pose of the scan, and the robot still walks some cm
# before the command arrives.
CHAIN_TURN_EXTRA_AIR_M = 0.15
# After a path change while walking the new line starts where the robot was at the scan, and
# the robot has walked on meanwhile: it rejoins the line with a short step if it is closer
# than this, otherwise the cruise stops and the stop-and-go takes over. With 0.10 m two path
# changes out of ten ended the cruise in the end-to-end world with the gap (12 and 15 cm).
CRUISE_REJOIN_MAX_M = 0.20


def stop_if_moving(env, command_client, reason):
    """If an attempt ended with the robot still walking (CHAIN_CELLS), stop it and wait until it
    stands. Before GraphNav, at the end of the exploration, after a failed attempt."""
    pc = getattr(env, '_moving_pc', None)
    if pc is None:
        return
    env._moving_pc = None
    pc.stop(reason)
    CruiseSteps._cruise_settle(pc)
    _close_track(env)


def cell_attempt_seen_fraction():
    """cell_attempt.CELL_SEEN_FRACTION (imported here: cell_attempt imports this module)."""
    from spot_explorer.mission import cell_attempt
    return cell_attempt.CELL_SEEN_FRACTION


def _close_track(env):
    """The trajectory of a cruise is recorded until the robot stops or the next command
    takes over: after a hand-off it goes on walking (on 03-11 15:39 the stretch after the
    hand-off was missing from trajectory.csv: 'unrecorded leg' in the viewer)."""
    rec = getattr(env, '_moving_track', None)
    env._moving_track = None
    if rec is None:
        return
    recorder, target, seg_id = rec
    recorder.stop()
    try:
        append_trajectory(getattr(recorder, 'samples', []), 'cruise', target, seg_id)
    except Exception as e:
        print(f"[LOG] Trajectory of the cruise not recorded: {e}")


def _along(polyline, x, y):
    """Distance along the polyline of the projection of (x, y) on its nearest segment."""
    best, best_s, s0 = np.inf, 0.0, 0.0
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        abx, aby = bx - ax, by - ay
        L2 = abx * abx + aby * aby
        L = float(np.sqrt(L2))
        t = float(np.clip(((x - ax) * abx + (y - ay) * aby) / max(L2, 1e-12), 0.0, 1.0))
        d = float(np.hypot(x - ax - t * abx, y - ay - t * aby))
        if d < best:
            best, best_s = d, s0 + t * L
        s0 += L
    return best_s


_TIERS = ('PLAIN', 'MODERATE', 'HARD')


def _tier(grad, rough):
    """The level of motion.select_mobility_params, without choosing (and printing)."""
    plain_max, moderate_max = gait.slope_thresholds()
    if grad < plain_max and rough < gait.GAIT_PLAIN_ROUGH_MAX:
        return 'PLAIN'
    if grad < moderate_max and rough < gait.GAIT_MODERATE_ROUGH_MAX:
        return 'MODERATE'
    return 'HARD'


class CruiseSteps:
    """Mixin of CellAttempt: the state is CellAttempt's, see its docstring for the map of the phases."""

    def _can_cruise(self):
        """The decided move goes in cruise: switch on, frontier thread with the planning
        mode, at least one whole leg (a partial step stays a partial step), and not in a
        tight space (frontier with the exit rule active: the stop-and-go gets out of it)."""
        tight = self.frontier.get('clearance', 0.0) < self.frontier.get('clearance_nominal', 0.0) - 1e-9
        return (CONTINUOUS_MOTION and self.verification_tracker is not None and not tight
                and hasattr(self.verification_tracker, 'start_planning')
                and self.decision['kind'] == 'move' and self.decision['full']
                and not self.keep_heading and len(self.path_waypoints) > 0)

    def _cruise_bundle_info(self):
        """Where and how the planner names its scan bundles (CruisePlanner.bundle)."""
        if self.mission_folder is None:
            return None
        return dict(folder=self.mission_folder, iteration=self.iteration,
                    cell=(self.target_row, self.target_col), loop_iter=self.loop_iter)

    def _cruise_gait(self, terrain):
        """
        Gait for the worst terrain of the first metre of the confirmed stretch. The level: a
        harder one at once, a lighter one only after GAIT_DOWN_HOLD_S of plans asking for it.
        The locomotion_hint (AUTO <-> CRAWL) changes only after GAIT_HINT_HOLD_S of plans asking
        for the new one, both ways; until then the new level's speed, friction and step go
        with the old hint. The state lives on env: it carries over the hand-offs.
        """
        if self.mobility_kwargs:
            return self.mobility_kwargs
        g, now = self.env, time.time()
        tier = _tier(*terrain)
        cur = getattr(g, '_gait_tier', None)
        if cur is not None and _TIERS.index(tier) < _TIERS.index(cur):
            if getattr(g, '_gait_down_since', None) is None:
                g._gait_down_since = now
            if now - g._gait_down_since < GAIT_DOWN_HOLD_S:
                tier = cur
            else:
                g._gait_down_since = None
        else:
            g._gait_down_since = None
        if tier != cur:
            g._gait_params = motion.select_mobility_params(*terrain)
            g._gait_tier = tier
        kwargs = dict(g._gait_params)
        hint = getattr(g, '_gait_hint', None)
        if hint is None:
            g._gait_hint = hint = kwargs['locomotion_hint']
        if kwargs['locomotion_hint'] != hint:
            if getattr(g, '_gait_hint_since', None) is None:
                g._gait_hint_since = now
            if now - g._gait_hint_since >= GAIT_HINT_HOLD_S:
                print(f"[GAIT] locomotion_hint {hint} -> {kwargs['locomotion_hint']} after "
                      f"{now - g._gait_hint_since:.1f} s of plans asking for it.")
                g._gait_hint, g._gait_hint_since = kwargs['locomotion_hint'], None
            else:
                kwargs['locomotion_hint'] = hint
        else:
            g._gait_hint_since = None
        return kwargs

    def _cruise(self):
        """The cruise (see the module docstring). Returns CONTINUE: the loop looks again
        with a fresh scan, as after a move."""
        tracker = self.verification_tracker
        ids = [self.robot_node_id] + [w[0] for w in self.path_waypoints]
        planner = planning_cycle.CruisePlanner(self.local_grid, self.global_grid, self.prm_graph,
                                               self.robot_state_client, self.goal_id, ids)
        planner.bundle = self._cruise_bundle_info()
        planner.seen_cell = self._seen_cell()
        self._thread_goals(planner)
        # First stretch: the one confirmed by the frontier of this turn, with the same corner
        # check the planner will make.
        adv, turn_stop = planner._turns_within(self.polyline, ids[1:],
                                               planning_cycle.walk_advance(self.polyline, self.frontier), self.snap)
        confirmed = planning_cycle._cut_polyline(self.polyline, adv)
        kwargs = self._cruise_gait(planner._terrain(confirmed, ids[1:]))
        path_len = float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.polyline, self.polyline[1:])))
        to_goal = turn_stop is None and adv >= path_len - 0.01
        HANDOFF.mark("cruise setup")
        print(f"[CRUISE] Start from node {self.robot_node_id} towards the goal "
              f"({self.target_x:.2f}, {self.target_y:.2f}): {len(ids) - 1} nodes, {path_len:.2f} m, "
              f"{adv:.2f} m confirmed" + (f" (rotation not clear at node {turn_stop})" if turn_stop else "") + ".")

        self.need_refresh = True                  # after the cruise: see EVENT REFRESH
        tracker.clear_path()
        # Robot still walking from the previous cell (CHAIN_CELLS): this command replaces that
        # one, turning while walking (_moving_check has let only small turns through).
        start_in_place = (motion.PATH_START_IN_PLACE_DEG if getattr(self.env, '_moving_pc', None) is not None
                          else motion.LEG_START_IN_PLACE_DEG)
        t_hand = getattr(self.env, '_handoff_t', None)
        self.env._handoff_t = None
        if t_hand is not None and getattr(self.env, '_moving_pc', None) is not None:
            print(f"[CRUISE] Hand-off: this cruise starts {time.time() - t_hand:.1f} s after the previous "
                  f"one ended, with the robot still walking.")
        self.env._moving_pc = None
        _close_track(self.env)
        handoff = False
        n_repicks = 0
        cycles_done = replans_done = 0
        pc = motion.PathCommand(self.command_client, self.robot_state_client)
        recorder = BodyTiltRecorder(self.robot_state_client)
        recorder.start()
        t0 = time.time()
        track = [(self.robot_x, self.robot_y)]
        reason, arrived, no_path_end = None, False, False
        switch_pending = report_switch = False
        switch_t0, switch_why, last_switch_id = 0.0, "", None
        n_plans = n_sends = bad = 0
        last_version, last_plan_t = 0, t0
        last_crumb_t, last_battery_t, last_fall_t = t0, t0, t0
        crumb = None
        try:
            pc.send(confirmed[1:], start_in_place_deg=start_in_place, **kwargs)
            HANDOFF.report("first send")
            n_sends += 1
            tracker.start_planning(planner, planning_cycle.PLAN_PERIOD_S)
            while reason is None:
                time.sleep(CRUISE_POLL_S)
                now = time.time()
                r = pc.poll()
                if r['x'] is not None and np.hypot(r['x'] - track[-1][0], r['y'] - track[-1][1]) >= CRUISE_TRACK_STEP_M:
                    track.append((r['x'], r['y']))
                st = r['status']
                if st == 'arrived' and to_goal:
                    reason, arrived = "arrived at the goal", True
                    break
                if st not in ('moving', 'near_goal', 'arrived', 'comm_error'):
                    reason = {'off_path': "too far from the path",
                              'blocked': "Spot reports the point blocked",
                              'expired': "command expired (no new send in time)",
                              'stopped': "command replaced or robot frozen"}.get(st, st)
                    break
                if now - t0 > CRUISE_MAX_S:
                    reason = f"cruise longer than {CRUISE_MAX_S:.0f} s"
                    break
                # Fall: the cruise ends and the next turn of the stop-and-go gets the robot up
                # and backs away (_guards, motion.check_and_recover_fall).
                if now - last_fall_t >= CRUISE_FALL_CHECK_S:
                    last_fall_t = now
                    try:
                        fall = motion.check_fall(self.robot_state_client)
                    except Exception:
                        fall = None
                    if fall is not None and fall.get('fallen'):
                        reason = f"fall ({fall.get('reason', '')})"
                        break
                if now - last_battery_t >= CRUISE_BATTERY_S:
                    last_battery_t = now
                    if battery_return_due():
                        reason = "low battery"
                        break
                # GraphNav waypoint in another thread: creating it took up to 1.6 s on 03-11
                # 14:48, with the main thread not reading plans nor sending in the meantime.
                if now - last_crumb_t >= CRUISE_BREADCRUMB_S and (crumb is None or not crumb.is_alive()):
                    last_crumb_t = now
                    k = min(planner._seg, len(planner.path_ids) - 1)
                    crumb = threading.Thread(
                        target=map_recording.graphnav_drop_breadcrumb, daemon=True,
                        args=(self.env, self.recordingInterface, self.robot_state_client,
                              planner.path_ids[k], self.target_row, self.target_col))
                    crumb.start()
                now = time.time()

                plan = tracker.get_plan()
                if plan is None or plan['version'] == last_version:
                    if now - last_plan_t > CRUISE_PLAN_STALE_S:
                        reason = f"no new plan for {now - last_plan_t:.1f} s"
                    continue
                last_version = plan['version']
                n_plans += 1
                if plan['status'] in ('no_scan', 'error'):
                    bad += 1
                    print(f"[CRUISE] Plan {last_version}: {plan['status']} {plan.get('error', '')}")
                    if bad >= CRUISE_BAD_PLANS:
                        reason = f"{bad} plans in a row without data ({plan['status']})"
                    continue
                bad, last_plan_t = 0, now
                sw = plan.get('switch')
                if sw is not None and sw.get('id') == last_switch_id:
                    sw = None         # already applied (it stays in the plans until cleared)
                if switch_pending and sw is None:
                    if now - switch_t0 > CRUISE_SWITCH_WAIT_S:
                        reason, handoff = (f"cell done ({switch_why}), the next one not prepared within "
                                           f"{CRUISE_SWITCH_WAIT_S:.0f} s: next cell while walking"), True
                        planner.switch_job = None
                        MISSION_STATS['cruise_switch_timeouts'] = MISSION_STATS.get('cruise_switch_timeouts', 0) + 1
                        HANDOFF.start("hand-off (end of the cruise -> first command of the next move)")
                        break
                    continue          # plans still towards the cell done: the robot walks the stretch sent
                if sw is not None and sw.get('kind') == 'end':
                    last_switch_id = sw.get('id')
                    planner.switch_info = None
                    reason, no_path_end = sw['reason'], True
                    if sw.get('left') is not None:
                        self.env._attempt_left_unreachable = True
                    break
                if sw is not None and sw.get('kind') in ('goal', 'cell') and sw.get('done') is None \
                        and sw.get('next', sw.get('cell')) is not None:
                    # Prepared by the thread on a path lost (_on_no_path): applied at once.
                    last_switch_id = sw.get('id')
                    planner.switch_info = None
                    rx0, ry0 = (r['x'], r['y']) if r['x'] is not None else plan['robot'][:2]
                    self._apply_switch(sw, rx0, ry0)
                    to_goal, report_switch = False, True
                    sw = None
                if sw is not None:
                    last_switch_id = sw.get('id')
                    planner.switch_info = None
                    switch_pending = False
                    if sw.get('next') is None:
                        reason, handoff = (f"{sw['where']}{sw['why']}: next cell while walking "
                                           f"(not prepared in cruise: {sw.get('reason')})"), True
                        HANDOFF.start("hand-off (end of the cruise -> first command of the next move)")
                        break
                    rx0, ry0 = (r['x'], r['y']) if r['x'] is not None else plan['robot'][:2]
                    self._apply_switch(sw, rx0, ry0)
                    to_goal, report_switch = False, True
                if plan['status'] == 'edge_blocked':
                    reason = "the edge being walked is out of the graph"
                    break
                if plan['status'] == 'no_path' and planner.no_path_job is not None:
                    continue          # the thread is choosing another goal or cell: the robot walks the stretch
                if plan['status'] == 'no_path':
                    # The stretch confirmed by this plan (still towards the old goal) is sent
                    # again, before and after the choice: while the main thread computes, the
                    # command must not expire (in the end-to-end world it did, after 2 s).
                    if len(plan.get('confirmed', [])) > 1:
                        pc.send(plan['confirmed'][1:], **self._cruise_gait(plan['terrain']))
                        n_sends += 1
                    new_planner = self._cruise_repick(planner, plan)
                    if new_planner is not None and len(plan.get('confirmed', [])) > 1:
                        pc.send(plan['confirmed'][1:], **self._cruise_gait(plan['terrain']))
                        n_sends += 1
                    if new_planner is None:
                        # No other goal with a path: the attempt ends here, once, and the side
                        # is marked as tried. Going on with a new cruise at every turn gave eight
                        # cruises of 4-9 cm in a row on 04-11 11:38, cell (1,3).
                        reason, no_path_end = "no path in the graph to the goal and no other goal in the cell", True
                        break
                    cycles_done += planner.cycles
                    replans_done += planner.replans
                    for k, v in planner.phase_s.items():
                        new_planner.phase_s[k] = v + new_planner.phase_s.get(k, [])
                    new_planner.grid_dt = planner.grid_dt + new_planner.grid_dt
                    planner, to_goal, last_plan_t = new_planner, False, time.time()
                    n_repicks += 1
                    continue
                rx, ry = (r['x'], r['y']) if r['x'] is not None else plan['robot'][:2]
                ahead = plan['advance'] - _along(plan['polyline'], rx, ry)
                f = plan['frontier']
                print(f"[CRUISE] Plan {last_version}: {ahead:.2f} m confirmed ahead"
                      + (" up to the goal" if plan['to_goal'] else f" ({f['reason']})")
                      + (f", rotation not clear at node {plan['turn_stop']}" if plan['turn_stop'] else "")
                      + f", {len(plan['ids'])} nodes to the goal"
                      + (", PATH CHANGED" if plan['replanned'] else "")
                      + f" | cycle {plan['cycle_s']:.2f} s"
                      + (" (" + " ".join(f"{k} {v:.2f}" for k, v in plan['phases'].items()) + ")"
                         if plan.get('phases') else "")
                      + (f", Spot grid +{plan['grid_dt']:.2f} s" if plan.get('grid_dt') is not None else "")
                      + f", robot at {100 * r['deviation']:.0f} cm from the line"
                      + (f", cell seen {100 * plan['cell_seen']:.0f}%" if plan.get('cell_seen') is not None else "")
                      + (" | edges: slopes {0} recomputed, {1} kept, {2} untouched, {3} out (live scan)".format(*plan['edges'])
                         + (f", {plan['edges'][4]} nodes pruned" if plan['edges'][4] else "")
                         if plan.get('edges') else ""))
                # The new stretch must start where the robot is: after a path change while
                # walking the line can pass beside it (see PathCommand.send).
                off = motion.distance_from_polyline(rx, ry, plan['confirmed'])
                if off > CRUISE_REJOIN_MAX_M and report_switch:
                    # The plan with the next cell was computed on a scan a moment old and the
                    # robot, walking the stretch sent, has moved on: the next plan starts from
                    # where it is (test_cruise step C, 1 run in 6: 31 cm, cruise ended).
                    continue
                if off > CRUISE_REJOIN_MAX_M:
                    reason = (f"the new path passes {100 * off:.0f} cm from the robot "
                              f"(limit {100 * CRUISE_REJOIN_MAX_M:.0f} cm)")
                    break
                # Hand-off to the next cell (CHAIN_CELLS), robot inside the target cell: goal
                # confirmed and within CRUISE_HANDOFF_M; or the frontier stops for an obstacle
                # (roughness, slope) with the goal within GOAL_REACHED_TOLERANCE_M along the
                # path, the arrival the stop-and-go would accept standing (decisions: 'arrived';
                # 03-11 15:39, 4 cells out of 6 ended like that, each with a stop).
                to_go = (sum(float(np.hypot(b[0] - a[0], b[1] - a[1]))
                             for a, b in zip(plan['polyline'], plan['polyline'][1:]))
                         - _along(plan['polyline'], rx, ry))
                # Or the cell already seen enough (CELL_SEEN_FRACTION of it in the global map),
                # wherever the robot is: the cell exists to be seen (06-11).
                seen = plan.get('cell_seen')
                why = None
                if (CHAIN_CELLS and CHAIN_ON_SEEN and seen is not None
                        and seen >= cell_attempt_seen_fraction()):
                    why = (f"cell seen {100 * seen:.0f}% (threshold {100 * cell_attempt_seen_fraction():.0f}%), "
                           f"{to_go:.2f} m from the goal")
                    where = ""
                    MISSION_STATS['cruise_seen_handoffs'] = MISSION_STATS.get('cruise_seen_handoffs', 0) + 1
                elif CHAIN_CELLS and self.env.is_point_in_cell(rx, ry, self.target_row, self.target_col):
                    where = "in the cell, "
                    if plan['to_goal'] and to_go <= CRUISE_HANDOFF_M:
                        why = f"{to_go:.2f} m from the goal"
                    elif (not plan['to_goal'] and f['reason'] in ('obstacle', 'roughness', 'slope')
                          and plan['turn_stop'] is None and to_go <= P.GOAL_REACHED_TOLERANCE_M):
                        why = (f"{to_go:.2f} m from the goal, the last stretch closed by "
                               f"{f['reason']}: within the arrival tolerance of {P.GOAL_REACHED_TOLERANCE_M:.2f} m")
                if why is not None:
                    # The last send covers the whole confirmed stretch (dead man of the
                    # trajectory time, as a leg): the robot walks it while the next cell is
                    # prepared (in the planning thread, or by the next attempt).
                    pc.send(plan['confirmed'][1:], hold_s=None, **self._cruise_gait(plan['terrain']))
                    n_sends += 1
                    self.env._handoff_t = time.time()
                    if self._can_switch():
                        planner.switch_job = functools.partial(
                            self._prepare_next_cell, done=(self.target_row, self.target_col), why=why,
                            where=where)
                        switch_pending, switch_t0, switch_why = True, time.time(), why
                        HANDOFF.start("next cell prepared in the planning thread -> first command towards it")
                        continue
                    reason, handoff = f"{where}{why}: next cell while walking", True
                    HANDOFF.start("hand-off (end of the cruise -> first command of the next move)")
                    break
                if ahead < CRUISE_MIN_AHEAD_M:
                    if plan['to_goal']:
                        reason, arrived = "arrived at the goal", True
                        self._cruise_finish_command(pc)
                    else:
                        reason = (f"the confirmed stretch ends {max(ahead, 0.0):.2f} m ahead ("
                                  + (f"rotation not clear at node {plan['turn_stop']}" if plan['turn_stop']
                                     else f['reason']) + ")")
                    break
                to_goal = plan['to_goal']
                pc.send(plan['confirmed'][1:], **self._cruise_gait(plan['terrain']))
                n_sends += 1
                if report_switch:
                    HANDOFF.report("first send towards the next cell")
                    report_switch = False
        finally:
            if handoff:
                self.env._moving_pc = pc          # still walking to the goal: see CHAIN_CELLS
            elif not arrived:
                pc.stop(f"end of the cruise: {reason}")
                self._cruise_settle(pc)
                # What the robot had in front when the cruise stopped on the terrain (only the
                # confirmed blocks had photos: one on 04-11 14:08, tall grass, against 23 stops).
                if reason and re.search(r'obstacle|slope|roughness|out of the graph', reason):
                    photos.capture(f"cruise_it{self.iteration:03d}_cell{self.target_row}_{self.target_col}_"
                                   f"{self.loop_iter:03d}_{time.strftime('%H-%M-%S')}",
                                   extra={'reason': reason})
            tracker.stop_planning()
            # The last scan of the cruise, for the next attempt (CHAIN_REUSE_SCAN).
            self.env._chain_scan = ((planner.last_scan, time.time())
                                    if handoff and planner.last_scan is not None else None)
            if crumb is not None:
                crumb.join(timeout=5.0)
            self.env._moving_track = (recorder, (self.target_x, self.target_y),
                                      [self.iteration, self.target_row, self.target_col, self.loop_iter,
                                       ids[0], self.goal_id])
            if not handoff:
                _close_track(self.env)
        planner.cycles += cycles_done
        planner.replans += replans_done
        MISSION_STATS['cruise_goal_repicks'] = MISSION_STATS.get('cruise_goal_repicks', 0) + n_repicks
        outcome = self._after_cruise(planner, track, t0, reason, n_plans, n_sends, pc, handoff)
        if arrived or handoff or pc.travelled >= CRUISE_STALL_M:
            self.cruise_stalls = 0
        else:
            self.cruise_stalls += 1
            if self.cruise_stalls >= MAX_CRUISE_STALLS:
                MISSION_STATS['cruise_stalls'] = MISSION_STATS.get('cruise_stalls', 0) + 1
                print(f"[STALL] {self.cruise_stalls} cruises in a row of less than "
                      f"{CRUISE_STALL_M:.2f} m towards ({self.target_x:.2f}, {self.target_y:.2f}): "
                      f"the way confirmed at the start is not one the cruise can walk. I give up "
                      f"entering ({self.target_row},{self.target_col}) from here.")
                return FAILED
        if no_path_end:
            print(f"[CRUISE] Attempt on cell ({self.target_row},{self.target_col}) ended: no path in "
                  f"the graph to any goal of the cell from here.")
            return FAILED
        if handoff:
            self._handoff = True
            MISSION_STATS['cruise_handoffs'] = MISSION_STATS.get('cruise_handoffs', 0) + 1
            return STOP
        return outcome

    def _thread_goals(self, planner):
        """The planner chooses itself the new goal (or cell) when the path is lost
        (CRUISE_SWITCH: _on_no_path), with the state of the attempt."""
        planner.target_cell = (self.target_row, self.target_col)
        planner.tried_goals = list(self.tried_goals)
        planner.repicks = self.goal_repicks
        planner.no_path_job = self._on_no_path if CRUISE_SWITCH else None

    def _can_switch(self):
        """CRUISE_SWITCH with the cell rules of the mission at hand."""
        from spot_explorer.mission import cell_rules
        return (CRUISE_SWITCH and cell_rules.CELL_RULES
                and getattr(self.env, '_cell_rules', None) is not None)

    def _robot_edge(self, planner, robot):
        """(prev, next): the edge of the planner's path the robot is walking."""
        coords = [self.prm_graph.get_node_position(n) for n in planner.path_ids]
        _, seg = motion.path_ahead(coords, robot[0], robot[1], from_seg=planner._seg)
        k = min(seg, len(planner.path_ids) - 2) if len(planner.path_ids) > 1 else 0
        prev_id = planner.path_ids[k]
        next_id = planner.path_ids[k + 1] if len(planner.path_ids) > 1 else planner.path_ids[0]
        return prev_id, next_id

    def _goal_with_path(self, row, col, scan, robot, next_id, exclude):
        """IN THE PLANNING THREAD: the goal of cell (row, col) (goal.find_best_point_in_cell,
        reachable from next_id) and the path to it, up to GOAL_REPICK_CANDIDATES points.
        (ids, goal_id, x, y), ids None if no goal has a path; the points tried go into exclude."""
        env, prm, gm = self.env, self.prm_graph, self.global_map
        mission_prm.restrict_planning_to_explored_cells(prm, env, row, col, (robot[0], robot[1]))

        def reachable(px, py):
            nid = prm.anchor_node(px, py, gm, geometry.PRM_EDGE_SAFETY_MARGIN_M)
            return nid is not None and prm.find_path_dijkstra(next_id, nid) is not None
        for _k in range(P.GOAL_REPICK_CANDIDATES):
            tx, ty, _vs, _rs = goal_choice.find_best_point_in_cell(
                robot[0], robot[1], env, row, col, scan.pts, scan.od, self.sampler,
                global_map=gm, exclude=exclude, reachable=reachable)
            if tx is None or ty is None:
                break
            gid = prm.add_stop_node(tx, ty, gm, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                    label=f"goal of cell ({row},{col}), chosen while walking", is_robot=False)
            ids = prm.find_path_dijkstra(next_id, gid)
            if ids is not None:
                return ids, gid, float(tx), float(ty)
            exclude.append((float(tx), float(ty)))
        return None, None, None, None

    @staticmethod
    def _planner_to(planner, prm, prev_id, next_id, ids, gid, cell):
        """The planner goes on along ids towards goal gid of cell, keeping the edge being walked."""
        planner.path_ids = ([prev_id] if prev_id != next_id else []) + list(ids)
        planner._seg = 0
        planner._not_edges = {(a, b) for a, b in zip(planner.path_ids, planner.path_ids[1:])
                              if not planning_cycle._edge_in_graph(prm, a, b)}
        planner.goal_id = gid
        if planner.seen_cell is not None:
            planner.seen_cell = (planner.seen_cell[0], cell[0], cell[1])
        if planner.bundle is not None:
            planner.bundle = dict(planner.bundle, cell=tuple(cell))
        if tuple(cell) != tuple(getattr(planner, 'target_cell', cell)):
            planner.target_cell, planner.tried_goals, planner.repicks = tuple(cell), [], 0

    def _prepare_next_cell(self, planner, scan, robot, done, why, where="", exclude=()):
        """
        IN THE PLANNING THREAD (CruisePlanner.switch_job, or _on_no_path), on the fresh scan of
        the cycle: the cell done is marked (done None: the cell is left because unreachable
        now, not marked), the next one chosen (cell_rules, only within NEAR_PATH_M, not in
        exclude), its goal and the path; the planner goes on towards it, keeping the edge being
        walked. Returns the 'switch' of the plan: next = (row, col), goal and the path ids, or
        next None and the reason.
        """
        t0 = time.perf_counter()
        env, prm, gm = self.env, self.prm_graph, self.global_map
        if done is not None:
            env.mark_cell_visited(*done)
        info = dict(kind='cell', done=done, why=why, where=where, next=None)
        rules = getattr(env, '_cell_rules', None)
        choice = (rules.choose((robot[0], robot[1]), gm, self.sampler, exclude=exclude)
                  if rules is not None else None)
        if choice is None:
            info['reason'] = "no other candidate cell"
            return info
        info['choice'] = choice
        if not choice.near:
            info['reason'] = f"next cell ({choice.row},{choice.col}) {choice.length:.1f} m away: GraphNav"
            return info
        prev_id, next_id = self._robot_edge(planner, robot)
        prm.clear_penalties_for_attempt()
        ids, gid, tx, ty = self._goal_with_path(choice.row, choice.col, scan, robot, next_id, [])
        if ids is None:
            info['reason'] = f"no goal with a path in the next cell ({choice.row},{choice.col})"
            return info
        self._planner_to(planner, prm, prev_id, next_id, ids, gid, (choice.row, choice.col))
        info.update(next=(choice.row, choice.col), goal=(tx, ty), goal_id=gid, ids=list(ids),
                    n_nodes=len(ids), prep_s=time.perf_counter() - t0)
        return info

    def _on_no_path(self, planner, scan, robot, prev_id, next_id):
        """
        IN THE PLANNING THREAD (CruisePlanner.no_path_job): no path from the node the robot
        walks to towards the goal. If from there the graph goes on (the goal is the problem,
        not the robot): the goal is excluded for the mission and another one chosen in the cell
        (up to MAX_BLOCK_REPLANS per cell); with none, the cell is unreachable now and the
        next one is chosen (_prepare_next_cell, the cell left NOT marked done nor failed).
        Returns the 'switch' of the plan: kind 'goal' or 'cell' with the new path ids, or kind
        'end' with the reason (the cruise ends).
        """
        t0 = time.perf_counter()
        env, prm = self.env, self.prm_graph
        row, col = planner.target_cell
        prm.set_robot_node(next_id, 0.0)
        n_free = prm.reachable_count(next_id, P.REPLAN_FAIL_ROBOT_FREE_NODES)
        if not P.REPICK_ON_REPLAN_FAIL or n_free < P.REPLAN_FAIL_ROBOT_FREE_NODES:
            planner.no_path_job = None
            return dict(kind='end', reason=(f"no path from node {next_id}, and from there the graph reaches only "
                                            f"{n_free} nodes: the robot is shut in, not the goal"))
        gx, gy = prm.get_node_position(planner.goal_id)
        goal_choice._refuse_goal(env, row, col, gx, gy, "no path in the updated graph")
        planner.tried_goals.append((float(gx), float(gy)))
        MISSION_STATS['replan_fail_repicks'] = MISSION_STATS.get('replan_fail_repicks', 0) + 1
        if planner.repicks < P.MAX_BLOCK_REPLANS:
            planner.repicks += 1
            ids, gid, tx, ty = self._goal_with_path(row, col, scan, robot, next_id, planner.tried_goals)
            if ids is not None:
                self._planner_to(planner, prm, prev_id, next_id, ids, gid, (row, col))
                return dict(kind='goal', cell=(row, col), goal=(tx, ty), goal_id=gid, ids=list(ids),
                            n_nodes=len(ids), prep_s=time.perf_counter() - t0)
        if self._can_switch():
            info = self._prepare_next_cell(planner, scan, robot, done=None,
                                           why=f"no goal with a path in cell ({row},{col})",
                                           exclude={(row, col)})
            info['left'] = (row, col)
            if info.get('next') is not None:
                return info
            planner.no_path_job = None
            return dict(kind='end', left=(row, col),
                        reason=f"no goal with a path in cell ({row},{col}) and {info.get('reason')}")
        planner.no_path_job = None
        return dict(kind='end', left=(row, col), reason=f"no goal with a path in cell ({row},{col})")

    def _apply_switch(self, sw, x, y):
        """MAIN THREAD, a plan with what the planning thread prepared (see _prepare_next_cell,
        _on_no_path): kind 'goal' -- a new goal in the same cell; kind 'cell' -- the next cell
        (with done: the GraphNav waypoint of the cell done, exploration env._on_chained_visit;
        with left: the cell left because unreachable now, recorded for the mission in
        env._cells_left_unreachable). The attempt now on the new goal / cell, the log."""
        self.target_x, self.target_y = sw['goal']
        self.goal_id = sw['goal_id']
        self.best_goal_dist = float('inf')
        self.env._last_attempt_goal = sw['goal']
        if sw['kind'] == 'goal':
            self.goal_repicks += 1
            MISSION_STATS['cruise_goal_repicks'] = MISSION_STATS.get('cruise_goal_repicks', 0) + 1
            print(f"[CRUISE] New goal ({sw['goal'][0]:.2f}, {sw['goal'][1]:.2f}) in cell ({sw['cell'][0]},"
                  f"{sw['cell'][1]}) chosen while walking (the previous one has no path any more): "
                  f"{sw['n_nodes']} nodes, prepared in the planning thread in {sw['prep_s']:.2f} s.")
            return
        done, nxt = sw.get('done'), sw['next']
        if done is not None:
            cb = getattr(self.env, '_on_chained_visit', None)
            if cb is not None:
                cb(done[0], done[1], x, y)
            what = f"Cell ({done[0]},{done[1]}) done while walking ({sw['why']})"
        else:
            left = sw['left']
            self.env._cells_left_unreachable = getattr(self.env, '_cells_left_unreachable', []) + [left]
            what = f"Cell ({left[0]},{left[1]}) left while walking ({sw['why']}: not failed, unreachable now)"
        self.target_row, self.target_col = nxt
        self.tried_goals, self.goal_repicks = [], 0
        self.env._chain_choice = sw['choice']
        MISSION_STATS['cruise_switches'] = MISSION_STATS.get('cruise_switches', 0) + 1
        HANDOFF.mark("next cell prepared, waypoint of the cell done")
        print(f"[CRUISE] {what}: next cell ({nxt[0]},{nxt[1]}), goal ({sw['goal'][0]:.2f}, {sw['goal'][1]:.2f}), "
              f"{sw['n_nodes']} nodes, prepared in the planning thread in {sw['prep_s']:.2f} s.")

    def _seen_cell(self):
        """(env, row, col) of the target cell for the planner's 'cell_seen', None if the seen
        rule is off."""
        from spot_explorer.mission import cell_attempt
        if not (CHAIN_ON_SEEN and cell_attempt.CELL_SEEN_EXPLORED):
            return None
        return (self.env, self.target_row, self.target_col)

    def _cruise_repick(self, planner, plan):
        """
        The goal is isolated (no path from the next node): new goal in the cell while the robot
        walks on the last stretch confirmed. The planning pauses (stop_planning waits for the
        cycle running), _repick_goal chooses as the stop-and-go does (same limits:
        REPICK_ON_REPLAN_FAIL, MAX_BLOCK_REPLANS, the graph must go on from the robot), and a
        new planner starts towards it, keeping the edge being walked. The new planner, or None
        (the caller ends the cruise as before).
        """
        tracker = self.verification_tracker
        tracker.stop_planning()
        k = min(planner._seg, max(len(planner.path_ids) - 2, 0))
        prev_id = planner.path_ids[k]
        next_id = planner.path_ids[min(k + 1, len(planner.path_ids) - 1)]
        self.robot_x, self.robot_y, self.robot_yaw = plan['robot']
        if planner.last_scan is not None:
            self.scan, self.snap = planner.last_scan, planner.last_snap
        self.robot_node_id = next_id
        self.prm_graph.set_robot_node(next_id, 0.0)
        n_free = self.prm_graph.reachable_count(next_id, P.REPLAN_FAIL_ROBOT_FREE_NODES)
        if not (P.REPICK_ON_REPLAN_FAIL and self.goal_repicks < P.MAX_BLOCK_REPLANS
                and n_free >= P.REPLAN_FAIL_ROBOT_FREE_NODES):
            return None
        MISSION_STATS['replan_fail_repicks'] = MISSION_STATS.get('replan_fail_repicks', 0) + 1
        if self._repick_goal(f"While walking: no path from node {next_id} to the goal "
                             f"({self.target_x:.2f}, {self.target_y:.2f}), while from the robot the graph "
                             f"goes on: it is the goal that is isolated.",
                             "no path in the updated graph") is not CONTINUE:
            return None
        ids = [self.robot_node_id] + [w[0] for w in self.path_waypoints]
        if prev_id != next_id:
            ids = [prev_id] + ids
        new_planner = planning_cycle.CruisePlanner(self.local_grid, self.global_grid, self.prm_graph,
                                                   self.robot_state_client, self.goal_id, ids)
        new_planner.bundle = self._cruise_bundle_info()
        new_planner.seen_cell = self._seen_cell()
        self._thread_goals(new_planner)
        # Its bundles go on numbering from those of the previous planner (same attempt turn):
        # from 1 again they overwrote them (04-11 14:08, cell (0,2)).
        new_planner.bundle_seq = planner.bundle_seq + planner.cycles
        tracker.start_planning(new_planner, planning_cycle.PLAN_PERIOD_S)
        print(f"[CRUISE] New goal ({self.target_x:.2f}, {self.target_y:.2f}) chosen while walking: "
              f"{len(ids) - 1} nodes from node {ids[0]}.")
        return new_planner

    def _moving_check(self):
        """
        Turn of the loop with the robot still walking from the previous cell (CHAIN_CELLS): it
        goes on only into a cruise with a small turn (within PATH_START_IN_PLACE_DEG, rotation
        clear) or past a path node; for anything else (rotation on the spot, wait, block,
        partial step) the robot is stopped first and the turn is made again standing.
        """
        if getattr(self.env, '_moving_pc', None) is None:
            return None
        kind = self.decision['kind']
        if kind == 'reached_node':
            return None
        if kind == 'move' and self._can_cruise():
            rot = decisions.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), self.decision['target'],
                                          self.frontier['od_robot'], snap=self.snap)
            if rot['turn'] and abs(rot['dyaw_deg']) <= motion.PATH_START_IN_PLACE_DEG:
                return None
            if rot['turn'] and self._turn_clear_while_moving(rot['dyaw_deg']):
                print(f"[CRUISE] {rot['dyaw_deg']:+.0f} deg to turn towards the next cell: on the spot "
                      f"inside the new command, without stopping first (swept area clear with "
                      f"{100 * CHAIN_TURN_EXTRA_AIR_M:.0f} cm more air).")
                return None
            why = (f"{rot['dyaw_deg']:+.0f} deg to turn towards the next cell, swept area not clear "
                   f"with {100 * CHAIN_TURN_EXTRA_AIR_M:.0f} cm more air")
        else:
            why = f"decision '{kind}' with the robot still walking"
        stop_if_moving(self.env, self.command_client, f"{why}: I stop and look again standing")
        self.need_refresh = True
        return CONTINUE

    @staticmethod
    def _cruise_finish_command(pc):
        """At the goal the last centimetres are still being walked: wait for the command to end
        (at most CRUISE_ARRIVAL_WAIT_S), so the next scan is made with the robot standing."""
        t_end = time.time() + CRUISE_ARRIVAL_WAIT_S
        while time.time() < t_end:
            if pc.poll()['status'] not in ('moving', 'near_goal', 'comm_error'):
                return
            time.sleep(CRUISE_POLL_S)
        pc.stop("arrival at the goal: the command did not end in time")

    @staticmethod
    def _cruise_settle(pc):
        """After a stop at 0.3 m/s the robot still slides a few centimetres: wait until it
        stands (less than 1 cm and 1 deg in 0.1 s), at most CRUISE_SETTLE_S. On 03-11 15:04
        the next move started 0.28 s after the stop, from a pose still moving, and ended at
        once 11 cm off its line; and the scan of the stop-and-go is made with the robot standing."""
        t_end = time.time() + CRUISE_SETTLE_S
        try:
            prev = pc.pose()
            while time.time() < t_end:
                time.sleep(0.1)
                now = pc.pose()
                if (np.hypot(now[0] - prev[0], now[1] - prev[1]) < 0.01
                        and abs(np.degrees(np.arctan2(np.sin(now[2] - prev[2]), np.cos(now[2] - prev[2])))) < 1.0):
                    return
                prev = now
        except Exception as e:
            print(f"[CRUISE] Pose not readable while waiting for the robot to stand: {e}")

    def _turn_clear_while_moving(self, dyaw_deg):
        """Rotation on the spot from the pose of the scan, with CHAIN_TURN_EXTRA_AIR_M more air."""
        extra = CHAIN_TURN_EXTRA_AIR_M
        if self.frontier['od_robot'] >= geometry.ROTATION_CLEARANCE_M + extra:
            return True
        clear, _margin = geometry.rotation_is_clear(
            self.snap.obstacle_dist, self.snap.origin_x, self.snap.origin_y, self.snap.cell_size,
            self.robot_x, self.robot_y, self.robot_yaw, float(np.radians(dyaw_deg)),
            air=geometry.ROTATION_CLEARANCE_AIR_M + extra)
        return bool(clear)

    def _stop_on_edge(self, planner, x, y):
        """The robot node after a cruise: an END of the edge it is on, not the nearest node
        (10-11): the node ahead if the edge is still in the graph (or the robot is on it),
        otherwise the node it came from -- the part just walked is free. Before: the nearest
        node with edges, 0.20 m away (median) on 09-11 14:36, also behind or beside the robot,
        with a first stretch robot -> node that is not an edge."""
        k = min(planner._seg, max(len(planner.path_ids) - 2, 0))
        prev_id = planner.path_ids[k]
        next_id = planner.path_ids[min(k + 1, len(planner.path_ids) - 1)]
        prm = self.prm_graph
        nx, ny = prm.get_node_position(next_id)
        on_next = float(np.hypot(nx - x, ny - y)) <= roadmap.ANCHOR_ON_NODE_M
        if prev_id == next_id or on_next or planning_cycle._edge_in_graph(prm, prev_id, next_id):
            nid, why = next_id, "the node ahead"
        else:
            nid, why = prev_id, "the node it came from (the edge ahead is out of the graph)"
        self.robot_node_id = nid
        prm.set_robot_node(nid, 0.0)
        px, py = prm.get_node_position(nid)
        print(f"[NODE] stop (cruise): on edge {prev_id}->{next_id}, robot node {nid} ({why}) at "
              f"({px:.2f}, {py:.2f}), {float(np.hypot(px - x, py - y)):.2f} m from ({x:.2f}, {y:.2f})")

    def _after_cruise(self, planner, track, t0, reason, n_plans, n_sends, pc, handoff=False):
        """The graph and maps are the main thread's again: track for the retreat, robot node
        where the robot is, path from there, breadcrumb."""
        duration = time.time() - t0
        x, y, _, _ = connection.getPosition(self.robot_state_client)
        track = track + [(x, y)]
        if not hasattr(self.env, '_traveled_arcs'):
            self.env._traveled_arcs = []
        for (ax, ay), (bx, by) in zip(track, track[1:]):
            if np.hypot(bx - ax, by - ay) > 0.05:
                self.env._traveled_arcs.append((ax, ay, bx, by))
        if planner.last_scan is not None:
            self.scan, self.snap = planner.last_scan, planner.last_snap
        remainder = planner.path_ids[min(planner._seg + 1, len(planner.path_ids) - 1):]
        self._stop_on_edge(planner, x, y)
        ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
        if ids is None:
            ids = [self.robot_node_id] + [n for n in remainder if n != self.robot_node_id]
        self._use_path(ids)
        self.no_progress = 0
        self.retreats_in_a_row = 0
        if not handoff:   # with the hand-off the next attempt drops one at its start
            map_recording.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client,
                                                   self.robot_node_id, self.target_row, self.target_col)

        walked = pc.travelled
        timing_add('motion', duration)
        for k, v in (('cruises', 1), ('cruise_s', duration), ('cruise_m', walked), ('cruise_cycles', planner.cycles),
                     ('cruise_plans', n_plans), ('cruise_sends', n_sends),
                     ('cruise_replans', planner.replans)):
            MISSION_STATS[k] = MISSION_STATS.get(k, 0) + v
        MISSION_STATS['cruise_max_dev_cm'] = max(MISSION_STATS.get('cruise_max_dev_cm', 0.0),
                                                 round(100 * pc.max_deviation, 1))
        exits = MISSION_STATS.setdefault('cruise_exits', {})
        # Grouped by reason: the numbers out ("ends 0.06 m ahead", "0.59 m from the goal").
        key = re.sub(r"\d+(\.\d+)?", "#", (reason or '?').split(' (')[0].split(':')[0])
        exits[key] = exits.get(key, 0) + 1
        print(f"[CRUISE] End: {reason}. {walked:.2f} m in {duration:.1f} s "
              f"({walked / max(duration, 1e-6):.2f} m/s), {n_plans} plans ({planner.cycles} cycles, "
              f"{planner.replans} path changes), {n_sends} sends, at most {100 * pc.max_deviation:.1f} cm "
              f"from the line. Robot at ({x:.2f}, {y:.2f}), node {self.robot_node_id}, "
              f"{len(self.path_waypoints)} nodes to the goal.")
        times = planner.timing_summary()
        if times:
            print(f"[CRUISE] Cycle times (median, s): {times}")
        for k, v in planner.phase_s.items():
            st = MISSION_STATS.setdefault('cruise_phase_s', {}).setdefault(k, [0.0, 0])
            st[0] += float(sum(v)); st[1] += len(v)
        g = MISSION_STATS.setdefault('cruise_grid_dt', [0.0, 0, 0])     # sum of the new, new, repeated
        new = [d for d in planner.grid_dt if d > 1e-6]
        g[0] += float(sum(new)); g[1] += len(new); g[2] += len(planner.grid_dt) - len(new)
        return CONTINUE
