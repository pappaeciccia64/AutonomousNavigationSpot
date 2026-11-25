"""
The cruise of CellAttempt (see cell_attempt.py), switch CONTINUOUS_MOTION: the robot walks
along the path without stopping at the nodes, while the frontier thread runs the whole
planning cycle (planning_cycle.CruisePlanner) and the main thread sends it the stretch
confirmed free of every new plan (motion.PathCommand).

    stop-and-go decides 'move' --> _cruise():
        send the stretch confirmed by the frontier of the stop-and-go
        thread: scan, maps, edges, Dijkstra, frontier every PLAN_PERIOD_S (planning_cycle.plan_period)
        main:   at every new plan, send its confirmed stretch (it replaces the previous
                one without stopping the robot); the robot never goes beyond the stretch
                confirmed by the last plan
    back to the stop-and-go (a turn with a fresh scan, as after a move) when:
        arrived at the goal; the confirmed stretch ahead of the robot ends (obstacle, edge
        of the data, corner where the rotation is not clear); no path, not even from where
        the robot is; the command ends badly (stopped, off the path, Spot
        reports it blocked); no new plan for CRUISE_PLAN_STALE_S; fall (checked every
        CRUISE_FALL_CHECK_S); low battery; CRUISE_MAX_S.

While the thread plans the main thread does not touch graph and maps: it sends commands,
reads plans and the robot state. It takes them back after stop_planning().

Without stopping, while the robot walks on the last stretch confirmed (the command lasts
PATH_HOLD_S, so the main thread has that long):
  - goal isolated (plan 'no_path'): the planning thread itself chooses another goal in the
    cell, or the next cell (_on_no_path); with none the cruise ends;
  - the cell done (seen CELL_SEEN_FRACTION, or the goal within CRUISE_HANDOFF_M): the planning
    thread prepares the next cell and the same cruise goes on (_prepare_next_cell);
  - the next cell not prepared in cruise (far: GraphNav; none): the hand-off. Inside the
    target cell and within CRUISE_HANDOFF_M of the goal (or within
    GOAL_REACHED_TOLERANCE_M with the last stretch closed by an obstacle), the attempt
    ends successfully with the robot still walking towards the goal (env._moving_pc, last
    command over the whole confirmed stretch, not PATH_HOLD_S): the
    mission marks the cell and the next attempt starts while the robot walks. A cruise start
    goes on from its first plan (the rotation at the start, forward or backward, is in the
    plan); a stop-and-go turn goes on if the rotation is clear: within
    PATH_START_IN_PLACE_DEG of the nose it turns while walking, beyond it on the spot inside
    the new command if the swept area is clear with CHAIN_TURN_EXTRA_AIR_M more air
    (_moving_check); otherwise the robot is stopped first and the turn is made standing. An attempt
    that fails before its loop (no valid goal, cell not reachable in the graph: only
    computations) leaves the robot walking, and the mission goes on to the next cell.
    stop_if_moving stops it before GraphNav and at the end of the exploration.
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
from spot_explorer.mission import attempt_common
from spot_explorer.mission.attempt_common import CONTINUE, STOP, FAILED, battery_return_due
from spot_explorer.perception import gait
from spot_explorer.perception import global_maps
from spot_explorer.perception import geometry
from spot_explorer.planning import decisions
from spot_explorer.planning import roadmap
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
# MAX_CRUISE_STALLS cruises in a row shorter than CRUISE_STALL_M (not arrived, no hand-off) end
# the attempt: the robot is stuck in front of a stretch it cannot walk, and every short cruise
# would otherwise reset the other stall checks.
CRUISE_STALL_M = 0.10
MAX_CRUISE_STALLS = 3
CRUISE_TRACK_STEP_M = 0.30      # spacing of the walked track kept for the retreat
CRUISE_BREADCRUMB_S = 1.0       # how often the GraphNav breadcrumb is tried
CRUISE_BATTERY_S = 5.0          # how often the battery is read
CRUISE_FALL_CHECK_S = 1.0       # how often the fall is checked (the stop-and-go does it at every turn)
CRUISE_ARRIVAL_WAIT_S = 2.0     # at the goal: wait at most this for the command to end
CRUISE_SETTLE_S = 1.0           # after a stop: wait at most this for the robot to stand still
# Distance from the goal at which the cell counts as done and the next one is chosen. The robot
# walks it while the planning thread prepares the next cell (one cycle, ~0.3 s; 0.5 m is ~1.4 s at
# 0.35 m/s). If the next cell cannot be prepared in cruise (far, or none) the hand-off happens
# here and the next attempt starts with the robot on the last 0.5 m.
CRUISE_HANDOFF_M = 0.5
# A cell is also done as soon as attempt_common.CELL_SEEN_FRACTION of it is seen in the global
# map, wherever the robot is: the goal is only a place to see the cell from.
# When the target cell is done the cruise goes on. The planning thread (the only one touching
# graph, maps and cells) marks the cell, chooses the next one with the cell rules (only cells
# within NEAR_PATH_M), its goal and the path, and keeps planning towards it (_prepare_next_cell,
# run by CruisePlanner.switch_job), while the robot walks the last confirmed stretch. The main
# thread only records the GraphNav waypoint of the cell done. If the next cell cannot be prepared
# the cruise ends with the hand-off. The main thread waits at most CRUISE_SWITCH_WAIT_S for it.
CRUISE_SWITCH_WAIT_S = 3.0
# The first command of a cruise comes from the first plan of the planning thread (the same
# cycle as every other): it starts if the plan confirms at least CRUISE_MIN_AHEAD_M (the
# rotation at the start, forward or backward, is in the plan: CruisePlanner._turns_within).
# If the plan does not allow to start, the thread stops and the stop-and-go takes the turn.
FIRST_PLAN_WAIT_S = 3.0
# The attempt start (scan, seen check, local data into the graph, goal, path) is the first cycle
# of the planning thread (_start_in_thread), so the first plan can take this long. Without a path
# by then the start is tried once more, then the cell is left as unreachable for now (not a
# failure).
START_PLAN_WAIT_S = 10.0
START_TRIES = 2
# Gait in cruise: a lighter level only after the plans have asked for it this long, so the gait
# does not flip back and forth on mixed terrain.
GAIT_DOWN_HOLD_S = 2.0
# The locomotion_hint (AUTO <-> CRAWL) changes only after this long of plans asking for it, both
# ways: Spot makes a side step of 10-15 cm at every hint change while walking.
GAIT_HINT_HOLD_S = 3.0
# A turn larger than PATH_START_IN_PLACE_DEG with the robot still walking (stop-and-go decision,
# _moving_check) is made on the spot inside the new command, without stopping first, if the swept
# area is clear with this much more air. The check uses the pose of the scan, and the robot walks
# a few cm more before the command arrives.
CHAIN_TURN_EXTRA_AIR_M = 0.15
# A plan starts where the robot was when the plan was made, and the robot walks on until it is
# read: it rejoins the new line with a short step if it is closer than this, otherwise the plan
# is skipped (the robot keeps walking the stretch sent) and the next plan starts from where it is.
CRUISE_REJOIN_MAX_M = 0.20


def stop_if_moving(env, command_client, reason):
    """If an attempt ended with the robot still walking (hand-off), stop it and wait until it
    stands. Before GraphNav, at the end of the exploration, after a failed attempt."""
    pc = getattr(env, '_moving_pc', None)
    if pc is None:
        return
    env._moving_pc = None
    pc.stop(reason)
    CruiseSteps._cruise_settle(pc)
    _close_track(env)


def _main_timing(turn_t, poll_s, send_s):
    """The [CRUISE] Main thread line: interval between two turns of the cruise loop and what
    the network requests of a turn cost (feedback + pose, path sent). A turn longer than the
    planning period means some plans are never read."""
    ms = lambda a, q: f"{1000 * float(np.percentile(a, q)):.0f}" if len(a) else "-"
    dt = np.diff(turn_t) if len(turn_t) > 1 else np.zeros(0)
    return (f"[CRUISE] Main thread: turn every p50/p90 {ms(dt, 50)}/{ms(dt, 90)} ms, feedback + pose "
            f"{ms(poll_s, 50)}/{ms(poll_s, 90)} ms, path sent {ms(send_s, 50)}/{ms(send_s, 90)} ms "
            f"({len(turn_t)} turns, {len(send_s)} sends timed)")


def _close_track(env):
    """The trajectory of a cruise is recorded until the robot stops or the next command
    takes over: after a hand-off the robot is still walking, and that stretch belongs to the
    cruise too."""
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


class CellSwitch:
    """
    The change of goal or cell while walking, seen from the main thread of a cruise (_cruise).
    The planning thread prepares it (_prepare_next_cell when a cell is done, _on_no_path when
    the path is lost) and puts it in every plan ('switch', with an id) until it is applied here.

        ask(why, where)     the cell is done: the thread is asked for the next one (pending)
        take(plan)          the switch of the plan not applied yet, or None
        late(now)           beyond CRUISE_SWITCH_WAIT_S: the hand-off
        at_goal(now)        the robot reached the goal of the cell done while pending: said once
        apply(sw)           the switch: new goal or cell for the attempt, or the end of the cruise
    """

    def __init__(self, cruise, planner):
        self.cruise, self.planner = cruise, planner
        self.pending = False          # a cell done, the next one asked to the thread
        self.t0, self.why = 0.0, ""   # when and why it was asked
        self.last_id = None           # id of the last switch applied (it stays in the plans)
        self.waiting_logged = False
        self.report = False           # the first send towards the new goal or cell is still to come

    def ask(self, why, where):
        cruise = self.cruise
        self.planner.switch_job = functools.partial(
            cruise._prepare_next_cell, done=(cruise.target_row, cruise.target_col), why=why, where=where)
        self.pending, self.t0, self.why, self.waiting_logged = True, time.time(), why, False
        HANDOFF.start("next cell prepared in the planning thread -> first command towards it")

    def take(self, plan):
        sw = plan.get('switch')
        if sw is None or sw.get('id') == self.last_id or sw.get('kind') in ('start', 'seen', 'nogoal'):
            return None           # none, already applied, or the attempt start (see _first_plan)
        return sw

    def at_goal(self, now):
        # At the goal of the cell done: the next cell is on its way. Said once (a stop standing
        # on the goal, otherwise invisible in the log).
        if not self.waiting_logged:
            self.waiting_logged = True
            MISSION_STATS['cruise_switch_waits_at_goal'] = MISSION_STATS.get('cruise_switch_waits_at_goal', 0) + 1
            print(f"[CRUISE] At the goal of the cell done, {now - self.t0:.1f} s after asking the next "
                  f"cell: standing until it is prepared (at most {CRUISE_SWITCH_WAIT_S:.0f} s).")

    def late(self, now):
        """The reason of the hand-off if the next cell is not prepared within CRUISE_SWITCH_WAIT_S,
        else None."""
        if now - self.t0 <= CRUISE_SWITCH_WAIT_S:
            return None
        self.planner.switch_job = None
        MISSION_STATS['cruise_switch_timeouts'] = MISSION_STATS.get('cruise_switch_timeouts', 0) + 1
        HANDOFF.start("hand-off (end of the cruise -> first command of the next move)")
        return (f"cell done ({self.why}), the next one not prepared within {CRUISE_SWITCH_WAIT_S:.0f} s: "
                f"next cell while walking")

    def apply(self, sw):
        """The switch of a plan. Returns None when the cruise goes on (new goal or cell applied,
        the plan is handled as usual), otherwise (reason, handoff, no_path_end)."""
        cruise = self.cruise
        self.last_id = sw.get('id')
        self.planner.switch_info = None
        if sw.get('kind') == 'end':
            # No goal with a path and no cell to go on to (_on_no_path).
            if sw.get('left') is not None:
                cruise.env._attempt_left_unreachable = True
            return sw['reason'], False, True
        if sw.get('done') is None:
            # Prepared by the thread on a path lost (_on_no_path): another goal, or the next cell
            # with this one left as unreachable now.
            cruise._apply_switch(sw)
            self.report = True
            return None
        # The cell done (ask): the next cell, or the hand-off when it could not be prepared in
        # cruise (far: GraphNav; none).
        self.pending = False
        if sw.get('next') is None:
            HANDOFF.start("hand-off (end of the cruise -> first command of the next move)")
            return (f"{sw['where']}{sw['why']}: next cell while walking "
                    f"(not prepared in cruise: {sw.get('reason')})"), True, False
        cruise._apply_switch(sw)
        self.report = True
        return None


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

    def _start_in_thread(self, planner, scan, robot):
        """
        In the planning thread (CruisePlanner.switch_job of the first cycle):
        the attempt start on the fresh scan of the cycle -- cell already seen? local data into
        the graph and the edges of the areas with new data, robot node, goal and path (the same
        steps of the stop-and-go start, attempt_scan) -- then the cycle plans on that path. The
        main thread waits for the first plan (_first_plan) and does not touch graph and maps.
        Returns the 'switch' of the plan: kind 'start' with the path, 'seen', or 'nogoal'.
        """
        t0 = time.perf_counter()
        self.robot_x, self.robot_y, self.robot_yaw = robot
        self.scan = scan
        HANDOFF.mark("first scan of the planning thread")
        if self._cell_already_seen():
            return dict(kind='seen')
        HANDOFF.mark("seen check")
        self._local_data_into_graph(scan)
        if self._goal_and_path(scan) is FAILED or not self.path_waypoints:
            return dict(kind='nogoal')
        self._reset_counters()
        self.loop_iter = 1          # the cruise is the first turn (_reset_counters puts 0)
        global_maps.save_global_maps(self.global_grid, self.mission_folder)
        HANDOFF.mark("attempt counters")
        planner.set_path([self.robot_node_id] + [w[0] for w in self.path_waypoints], self.goal_id)
        self._thread_goals(planner)
        return dict(kind='start', prep_s=time.perf_counter() - t0)

    def _can_start_in_thread(self):
        """The whole attempt start can be the first cycle of the planner (cruise on, tracker with
        the planning mode, gait not forced)."""
        return (CONTINUOUS_MOTION and self.verification_tracker is not None
                and hasattr(self.verification_tracker, 'start_planning') and not self.mobility_kwargs)

    def _first_plan(self, planner):
        """The planner starts and its first plan (with data) is the first turn
        of the attempt. The plan if the robot can start on it (confirmed stretch, rotation towards
        the first node clear). A plan that does not allow to start is a block at once
        (_block_goal): the thread chooses another goal or cell and its plan is taken. None if no
        usable plan comes in time, or no goal is left (the planning is stopped). Leaves the
        robot pose, scan, snapshot, frontier and polyline of the plan."""
        tracker = self.verification_tracker
        tracker.start_planning(planner, planning_cycle.plan_period())
        t0, plan, last = time.time(), None, None
        wait_s = START_PLAN_WAIT_S if planner.switch_job == self._start_in_thread else FIRST_PLAN_WAIT_S
        bad, why, asked = [], None, None
        while time.time() - t0 < wait_s:
            time.sleep(CRUISE_POLL_S)
            p = tracker.get_plan()
            if p is None or p.get('version') == last:
                continue
            last = p.get('version')
            if p.get('status') in ('no_scan', 'error'):
                # A plan without data or with an error is logged, so a failed start says why.
                bad.append((p.get('status'), p.get('error', '')))
                print(f"[CRUISE] First plan {last}: {p.get('status')} {p.get('error', '')}")
                continue
            plan, why = p, None
            self.robot_x, self.robot_y, self.robot_yaw = plan['robot']
            if planner.last_scan is not None:
                self.scan, self.snap = planner.last_scan, planner.last_snap
            self.frontier, self.polyline = plan['frontier'], plan['polyline']
            sw = plan.get('switch')
            if sw is not None and sw.get('kind') in ('start', 'seen', 'nogoal'):
                planner.switch_info = None
                if sw['kind'] in ('seen', 'nogoal'):
                    tracker.stop_planning()
                    return sw['kind']
                sw = None
            if sw is not None and sw.get('kind') == 'end':
                planner.switch_info = None
                if sw.get('left') is not None:
                    self.env._attempt_left_unreachable = True
                why = sw.get('reason')
                break
            if sw is not None and sw.get('kind') in ('goal', 'cell'):
                planner.switch_info = None
                self._apply_switch(sw)
                asked = None
            if plan['status'] == 'no_path' and planner.no_path_job is not None:
                continue                  # the thread is choosing another goal or cell
            if plan['status'] != 'ok':
                why = f"first plan '{plan['status']}'"
                break
            if plan['advance'] < CRUISE_MIN_AHEAD_M:
                f = plan['frontier']
                why = (f"first plan: {plan['advance']:.2f} m confirmed ("
                       + (f"rotation not clear at node {plan['turn_stop']}" if plan['turn_stop'] else f['reason']) + ")")
                if asked is None and self._block_goal(planner, why):
                    asked, t0 = why, time.time()     # the new goal gets its own wait
                if asked is not None:
                    continue
                break
            break
        moving = getattr(self.env, '_moving_pc', None) is not None
        if plan is None:
            # Why: kept for the attempt (start_failure_reason), it ends up in the log and in the
            # mission summary: a start that fails is a failure of the cell, the cause tells
            # whether the graph or the computer was to blame.
            errors = [e for st, e in bad if st == 'error']
            noscan = [e for st, e in bad if st == 'no_scan']
            why = (f"no plan from the planning thread within {wait_s:.0f} s: "
                   + (f"{len(errors)} cycles in error, the last: {errors[-1]}" if errors else
                      f"{len(noscan)} cycles without the scan of Spot ({noscan[-1] or 'no cause'})" if noscan else
                      "no cycle ended (thread busy or stuck)"))
            self.start_failure_reason = why
        elif why is None and (plan['status'] != 'ok' or plan['advance'] < CRUISE_MIN_AHEAD_M):
            why = (f"first plan '{plan['status']}', {plan['advance']:.2f} m confirmed: no other goal "
                   f"in {wait_s:.0f} s")
        if why is not None and plan is not None:
            self.start_failure_reason = why
            if asked is not None:
                self.env._attempt_left_unreachable = True     # blocked: the cell left for later
        if why is None and moving and len(plan['confirmed']) > 1:
            (ax, ay), (bx, by) = plan['confirmed'][:2]
            back = bool(plan['backward'] and plan['backward'][0])
            dyaw = np.degrees(motion._angle_diff(np.arctan2(by - ay, bx - ax) + (np.pi if back else 0.0),
                                                 self.robot_yaw))
            if abs(dyaw) > motion.PATH_START_IN_PLACE_DEG:
                print(f"[CRUISE] {dyaw:+.0f} deg to turn towards the next cell: on the spot inside the new "
                      f"command, without stopping first" + (" (then tail first)" if back else "") + ".")
        if why is None:
            MISSION_STATS['thread_first_turns'] = MISSION_STATS.get('thread_first_turns', 0) + 1
            return plan
        tracker.stop_planning()
        MISSION_STATS['thread_first_turn_fallbacks'] = MISSION_STATS.get('thread_first_turn_fallbacks', 0) + 1
        print(f"[CRUISE] The attempt does not start in cruise ({why}).")
        if moving:
            stop_if_moving(self.env, self.command_client, "first plan not usable while walking")
        self.need_refresh = True
        return None

    def _new_planner(self, goal_id, path_ids, thread_goals=True):
        """The planning cycle of a cruise of this attempt: towards goal_id along path_ids, scan
        bundles, share seen of the target cell; with thread_goals the new goal or cell chosen by
        the thread itself (_thread_goals: needs the counters of the attempt, see _reset_counters;
        at the attempt start in the thread _start_in_thread sets it)."""
        planner = planning_cycle.CruisePlanner(self.local_grid, self.global_grid, self.prm_graph,
                                               self._cruise_state_client(), goal_id, path_ids)
        planner.bundle = self._cruise_bundle_info()
        planner.seen_cell = self._seen_cell()
        if thread_goals:
            self._thread_goals(planner)
        return planner

    def _cruise_from_start(self):
        """The attempt start in the thread: the planner starts with a
        placeholder path at the robot node, its first cycle runs _start_in_thread. Returns as
        _cruise, or stop (cell already seen), FAILED (no goal), None (stop-and-go)."""
        start = self.prm_graph.robot_node if self.prm_graph.robot_node in self.prm_graph.nodes \
            else next(iter(self.prm_graph.nodes))
        planner = self._new_planner(start, [start], thread_goals=False)
        planner.switch_job = self._start_in_thread
        return self._cruise(from_thread=True, planner=planner)

    def _cruise(self, from_thread=False, planner=None):
        """The cruise (see the module docstring). Returns CONTINUE: the loop looks again
        with a fresh scan, as after a move. from_thread: the first command
        comes from the first plan of the planning thread; None if it does not allow to start
        (the stop-and-go takes the turn)."""
        tracker = self.verification_tracker
        if planner is None:
            planner = self._new_planner(self.goal_id, [self.robot_node_id] + [w[0] for w in self.path_waypoints])
        ids = list(planner.path_ids)
        if from_thread:
            plan0 = self._first_plan(planner)
            # Seen or no goal: only computations, no command of this attempt -- the robot keeps
            # walking if it was (hand-off), as an attempt that ends before its loop.
            if plan0 == 'seen':
                self._seen_only, self.loop_iter = True, 0
                return STOP
            if plan0 == 'nogoal':
                self.loop_iter = 0
                return FAILED
            if plan0 is None:
                return None
            ids = [planner.path_ids[0]] + list(plan0['ids'])
            adv, turn_stop = plan0['advance'], plan0['turn_stop']
            confirmed, backward = plan0['confirmed'], plan0['backward']
            kwargs = self._cruise_gait(plan0['terrain'])
        else:
            # First stretch: the one confirmed by the frontier of this turn, with the same corner
            # check the planner will make.
            adv, turn_stop, backward = planner._turns_within(
                self.polyline, ids[1:], planning_cycle.walk_advance(self.polyline, self.frontier), self.snap,
                self.robot_yaw)
            confirmed = planning_cycle._cut_polyline(self.polyline, adv)
            backward = backward[:max(len(confirmed) - 1, 0)]
            kwargs = self._cruise_gait(planner._terrain(confirmed, ids[1:]))
        path_len = float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.polyline, self.polyline[1:])))
        to_goal = turn_stop is None and adv >= path_len - 0.01
        HANDOFF.mark("cruise setup")
        print(f"[CRUISE] Start from node {self.robot_node_id} towards the goal "
              f"({self.target_x:.2f}, {self.target_y:.2f}): {len(ids) - 1} nodes, {path_len:.2f} m, "
              f"{adv:.2f} m confirmed" + (f" (rotation not clear at node {turn_stop})" if turn_stop else "") + ".")

        self.need_refresh = True                  # after the cruise: see event refresh
        tracker.clear_path()
        # Robot still walking from the previous cell (hand-off): this command replaces that
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
        state_client = self._cruise_state_client()
        planner.robot_state_client = state_client
        pc = motion.PathCommand(self.command_client, state_client)
        recorder = BodyTiltRecorder(state_client)
        recorder.start()
        t0 = time.time()
        track = [(self.robot_x, self.robot_y)]
        reason, arrived, no_path_end = None, False, False
        asked_block = None              # block_request asked, waiting for the new goal or cell
        spot_blocked, blocked_send = 0, -1   # sends Spot reported blocked, the last one counted
        switch = CellSwitch(self, planner)
        n_plans = n_sends = bad = 0
        turn_t, poll_s, send_s = [], [], []      # main thread timing, see _main_timing
        last_version, last_plan_t = 0, t0
        last_crumb_t, last_battery_t, last_fall_t = t0, t0, t0
        crumb = None
        try:
            pc.send(confirmed[1:], start_in_place_deg=start_in_place, backward=backward, **kwargs)
            HANDOFF.report("first send")
            n_sends += 1
            self._visit_after_send()
            if from_thread:
                last_version = plan0['version']       # the planner is running already
            else:
                tracker.start_planning(planner, planning_cycle.plan_period())
            while reason is None:
                self._wait_for_plan(tracker, last_version)
                now = time.time()
                turn_t.append(now)
                r = pc.poll()
                poll_s.append(time.time() - now)
                if r['x'] is not None and np.hypot(r['x'] - track[-1][0], r['y'] - track[-1][1]) >= CRUISE_TRACK_STEP_M:
                    track.append((r['x'], r['y']))
                st = r['status']
                if st == 'arrived' and switch.pending:
                    switch.at_goal(now)
                elif st == 'arrived' and to_goal:
                    reason, arrived = "arrived at the goal", True
                    break
                if st == 'moving' and n_sends > blocked_send:
                    spot_blocked = 0
                if st == 'blocked' and blocked_send != n_sends:
                    # Spot reports the point blocked, the robot still: once, the next plan sends
                    # again; again on a later send, a block (_block_goal).
                    blocked_send, spot_blocked = n_sends, spot_blocked + 1
                    if spot_blocked >= 2 and asked_block is None:
                        why = "Spot reports the point blocked again"
                        if self._block_goal(planner, why):
                            asked_block = why
                # Waiting for the next goal after a block, the robot stands at the end of its
                # stretch: its command may end meanwhile.
                waiting = ((st == 'blocked' and (spot_blocked < 2 or asked_block is not None))
                           or (st == 'expired' and asked_block is not None))
                if st not in ('moving', 'near_goal', 'arrived', 'comm_error') and not waiting:
                    reason = {'off_path': "too far from the path",
                              'blocked': "Spot reports the point blocked",
                              'expired': "command expired (no new send in time)",
                              'stopped': "command replaced or robot frozen"}.get(st, st)
                    break
                if now - t0 > CRUISE_MAX_S:
                    reason = f"cruise longer than {CRUISE_MAX_S:.0f} s"
                    break
                # Fall: the cruise ends, the attempt gets the robot up and backs away before the
                # next cruise (CellAttempt.run, _prepare_for_thread).
                if now - last_fall_t >= CRUISE_FALL_CHECK_S:
                    last_fall_t = now
                    try:
                        fall = motion.check_fall(state_client)
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
                # The GraphNav waypoint is created in another thread: it can take more than a
                # second, and meanwhile the main thread must keep reading plans and sending.
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
                sw = switch.take(plan)
                if switch.pending and sw is None:
                    if plan['status'] == 'ok' and len(plan.get('confirmed', [])) > 1:
                        # The stretch sent follows the confirmed one of every plan (it may get
                        # shorter), whole (no dead man: the robot walks it while waiting).
                        pc.send(plan['confirmed'][1:], hold_s=None, backward=plan['backward'],
                                **self._cruise_gait(plan['terrain']))
                        n_sends += 1
                        self._log_plan(plan, r, "waiting for the next cell")
                    reason = switch.late(now)
                    if reason is not None:
                        handoff = True
                        break
                    continue
                if sw is not None:
                    end = switch.apply(sw)
                    if end is not None:
                        reason, handoff, no_path_end = end
                        break
                    to_goal, asked_block = False, None
                if plan['status'] == 'no_path':
                    if planner.no_path_job is not None:
                        continue      # the thread is choosing another goal or cell: the robot walks the stretch
                    reason, no_path_end = "no path in the graph to the goal and no other goal in the cell", True
                    break
                rx, ry = (r['x'], r['y']) if r['x'] is not None else plan['robot'][:2]
                ahead = plan['advance'] - _along(plan['polyline'], rx, ry)
                f = plan['frontier']
                self._log_plan(plan, r)
                # The new stretch must start where the robot is. Every plan starts where the robot
                # was when the plan was made (planning_cycle.CruisePlanner.pose_now): if it is
                # still far from the new path, the plan is a moment old -- the robot keeps
                # walking the stretch sent (safe up to its frontier) and the next plan starts
                # from where it is.
                off = motion.distance_from_polyline(rx, ry, plan['confirmed'])
                if off > CRUISE_REJOIN_MAX_M:
                    MISSION_STATS['cruise_stale_plans'] = MISSION_STATS.get('cruise_stale_plans', 0) + 1
                    continue
                # Hand-off to the next cell, robot inside the target cell: goal
                # confirmed and within CRUISE_HANDOFF_M; or the frontier stops for an obstacle
                # (roughness, slope) with the goal within GOAL_REACHED_TOLERANCE_M along the
                # path, the arrival the stop-and-go would accept standing (decisions: 'arrived').
                to_go = (sum(float(np.hypot(b[0] - a[0], b[1] - a[1]))
                             for a, b in zip(plan['polyline'], plan['polyline'][1:]))
                         - _along(plan['polyline'], rx, ry))
                # Or the cell already seen enough (CELL_SEEN_FRACTION of it in the global map),
                # wherever the robot is.
                seen = plan.get('cell_seen')
                why = None
                if (seen is not None
                        and seen >= attempt_common.CELL_SEEN_FRACTION):
                    why = (f"cell seen {100 * seen:.0f}% (threshold {100 * attempt_common.CELL_SEEN_FRACTION:.0f}%), "
                           f"{to_go:.2f} m from the goal")
                    where = ""
                    MISSION_STATS['cruise_seen_handoffs'] = MISSION_STATS.get('cruise_seen_handoffs', 0) + 1
                else:
                    # The arrival is the distance from the goal, inside the target cell or not: a
                    # goal on the cell border with an obstacle just beyond it leaves the robot
                    # outside the cell for good.
                    where = "in the cell, " if self.env.is_point_in_cell(rx, ry, self.target_row, self.target_col) else ""
                    if plan['to_goal'] and to_go <= CRUISE_HANDOFF_M:
                        why = f"{to_go:.2f} m from the goal"
                    elif (not plan['to_goal'] and to_go <= P.GOAL_REACHED_TOLERANCE_M
                          and (f['reason'] in ('obstacle', 'roughness', 'slope') or ahead < CRUISE_MIN_AHEAD_M)):
                        why = (f"{to_go:.2f} m from the goal, the last stretch closed by "
                               + (f"rotation not clear at node {plan['turn_stop']}" if plan['turn_stop'] else f['reason'])
                               + f": within the arrival tolerance of {P.GOAL_REACHED_TOLERANCE_M:.2f} m")
                if why is not None:
                    # The last send covers the whole confirmed stretch (dead man of the
                    # trajectory time, as a leg): the robot walks it while the next cell is
                    # prepared (in the planning thread, or by the next attempt).
                    pc.send(plan['confirmed'][1:], hold_s=None, backward=plan['backward'],
                            **self._cruise_gait(plan['terrain']))
                    n_sends += 1
                    self.env._handoff_t = time.time()
                    if self._can_switch():
                        switch.ask(why, where)
                        continue
                    reason, handoff = f"{where}{why}: next cell while walking", True
                    HANDOFF.start("hand-off (end of the cruise -> first command of the next move)")
                    break
                if ahead < CRUISE_MIN_AHEAD_M:
                    if plan['to_goal']:
                        reason, arrived = "arrived at the goal", True
                        self._cruise_finish_command(pc)
                        break
                    why = (f"the confirmed stretch ends {max(ahead, 0.0):.2f} m ahead ("
                           + (f"rotation not clear at node {plan['turn_stop']}" if plan['turn_stop']
                              else f['reason']) + ")")
                    # No way on from here: a block at once, the planning thread chooses the next
                    # goal or cell and the same cruise goes on towards it.
                    if asked_block is not None or self._block_goal(planner, why):
                        asked_block = asked_block or why
                        continue
                    reason = why
                    break
                to_goal = plan['to_goal']
                t_send = time.time()
                pc.send(plan['confirmed'][1:], backward=plan['backward'], **self._cruise_gait(plan['terrain']))
                send_s.append(time.time() - t_send)
                n_sends += 1
                if switch.report:
                    HANDOFF.report("first send towards the next cell")
                    switch.report = False
                self._visit_after_send()
        finally:
            self._visit_after_send()     # cruise ended before a send towards the next cell
            if handoff:
                self.env._moving_pc = pc          # still walking to the goal: the hand-off
            elif not arrived:
                pc.stop(f"end of the cruise: {reason}")
                self._cruise_settle(pc)
                # Photos of what the robot had in front when the cruise stopped on the terrain.
                if reason and re.search(r'obstacle|slope|roughness', reason):
                    photos.capture(f"cruise_it{self.iteration:03d}_cell{self.target_row}_{self.target_col}_"
                                   f"{self.loop_iter:03d}_{time.strftime('%H-%M-%S')}",
                                   extra={'reason': reason})
            tracker.stop_planning()
            print(_main_timing(turn_t, poll_s, send_s))
            if hasattr(state_client, 'summary'):
                print(f"[CRUISE] {state_client.summary()}")
            if crumb is not None:
                crumb.join(timeout=5.0)
            self.env._moving_track = (recorder, (self.target_x, self.target_y),
                                      [self.iteration, self.target_row, self.target_col, self.loop_iter,
                                       ids[0], self.goal_id])
            if not handoff:
                _close_track(self.env)
        self.cruise_end_reason = reason
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

    def _cruise_state_client(self):
        """The robot state client the cruise reads the pose and the feedback with."""
        return self.robot_state_client

    def _wait_for_plan(self, tracker, last_version):
        """Between two turns of the cruise loop: until a new plan is published, at most
        CRUISE_POLL_S (the feedback is read at every turn, and the latest plan taken)."""
        tracker.wait_plan(last_version, CRUISE_POLL_S)

    def _block_goal(self, planner, why):
        """No way on towards the goal for the main thread: the planning thread treats the goal
        as one without a path at its next cycle (CruisePlanner.block_request -> _on_no_path:
        another goal in the cell, or the next cell). False if the thread cannot choose goals."""
        if planner.no_path_job is None:
            return False
        planner.block_request = why
        MISSION_STATS['cruise_blocks'] = MISSION_STATS.get('cruise_blocks', 0) + 1
        print(f"[CRUISE] Block: {why}. The goal ({self.target_x:.2f}, {self.target_y:.2f}) is left, "
              f"the planning thread chooses the next one.")
        return True

    def _log_plan(self, plan, r, note=None):
        """The [CRUISE] Plan line of a plan: confirmed stretch ahead and why it ends, path,
        timings of the cycle phases, deviation, share of the cell seen, terrain of the first
        metre (the gait comes from it), edges updated."""
        rx, ry = (r['x'], r['y']) if r['x'] is not None else plan['robot'][:2]
        ahead = plan['advance'] - _along(plan['polyline'], rx, ry)
        f = plan['frontier']
        print(f"[CRUISE] Plan {plan['version']}" + (f" ({note})" if note else "") + f": {ahead:.2f} m confirmed ahead"
              + (" up to the goal" if plan['to_goal'] else f" ({f['reason']})")
              + (f", rotation not clear at node {plan['turn_stop']}" if plan['turn_stop'] else "")
              + f", {len(plan['ids'])} nodes to the goal"
              + (", PATH CHANGED" if plan['replanned'] else "")
              + f" | cycle {plan['cycle_s']:.2f} s"
              + (" (" + " ".join(f"{k} {v:.2f}" for k, v in plan['phases'].items()) + ")"
                 if plan.get('phases') else "")
              + (f", Spot grid +{plan['grid_dt']:.2f} s" if plan.get('grid_dt') is not None else "")
              # how long the plan waited before the main thread read it
              + (f", read after {1000 * (time.time() - plan['time']):.0f} ms" if plan.get('time') else "")
              + f", robot at {100 * r['deviation']:.0f} cm from the line"
              + (f", cell seen {100 * plan['cell_seen']:.0f}%" if plan.get('cell_seen') is not None else "")
              # The terrain the gait is chosen from (first GAIT_LOOKAHEAD_M of the confirmed
              # stretch): worst slope (90th percentile of the plane, along or across) and
              # roughness -- to compare with the ground under the feet (gpe, trajectory.csv).
              + (f", terrain {np.degrees(np.arctan(plan['terrain'][0])):.1f} deg "
                 f"rough {100 * plan['terrain'][1]:.1f} cm" if plan.get('terrain') else "")
              + (" | edges: slopes {0} recomputed, {1} kept, {2} untouched, {3} out (live scan)".format(*plan['edges'])
                 + (f", {plan['edges'][4]} nodes pruned" if plan['edges'][4] else "")
                 if plan.get('edges') else ""))

    def _thread_goals(self, planner):
        """The planner chooses itself the new goal (or cell) when the path is lost
        (_on_no_path), with the state of the attempt."""
        planner.target_cell = (self.target_row, self.target_col)
        planner.tried_goals = list(self.tried_goals)
        planner.repicks = self.goal_repicks
        planner.no_path_job = self._on_no_path

    def _can_switch(self):
        """The next cell can be prepared in cruise: the cell rules of the mission at hand."""
        return getattr(self.env, '_cell_rules', None) is not None

    def _goal_with_path(self, row, col, scan, robot, next_id, exclude):
        """In the planning thread: the goal of cell (row, col) (goal.find_best_point_in_cell,
        reachable from next_id) and the path to it, up to GOAL_REPICK_CANDIDATES points.
        (ids, goal_id, x, y), ids None if no goal has a path; the points tried go into exclude."""
        env, prm, gm = self.env, self.prm_graph, self.global_map
        mission_prm.restrict_planning_to_explored_cells(prm, env, row, col, (robot[0], robot[1]))

        reachable = prm.reachable_from(next_id, gm, geometry.PRM_EDGE_SAFETY_MARGIN_M)
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
        planner.set_path(([prev_id] if prev_id != next_id else []) + list(ids), gid)
        if planner.seen_cell is not None:
            planner.seen_cell = (planner.seen_cell[0], cell[0], cell[1])
        if planner.bundle is not None:
            planner.bundle = dict(planner.bundle, cell=tuple(cell))
        if tuple(cell) != tuple(getattr(planner, 'target_cell', cell)):
            planner.target_cell, planner.tried_goals, planner.repicks = tuple(cell), [], 0

    def _prepare_next_cell(self, planner, scan, robot, done, why, where="", exclude=()):
        """
        In the planning thread (CruisePlanner.switch_job, or _on_no_path), on the fresh scan of
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
            info['reason'] = f"next cell ({choice.row},{choice.col}) way costing {choice.length:.1f} (m on flat ground): GraphNav"
            return info
        _seg, prev_id, next_id = planner.robot_edge(robot)
        prm.clear_penalties_for_attempt()
        ids, gid, tx, ty = self._goal_with_path(choice.row, choice.col, scan, robot, next_id, [])
        if ids is None:
            info['reason'] = f"no goal with a path in the next cell ({choice.row},{choice.col})"
            return info
        self._planner_to(planner, prm, prev_id, next_id, ids, gid, (choice.row, choice.col))
        info.update(next=(choice.row, choice.col), goal=(tx, ty), goal_id=gid, ids=list(ids),
                    n_nodes=len(ids), prep_s=time.perf_counter() - t0)
        return info

    def _on_no_path(self, planner, scan, robot, prev_id, next_id, why=None):
        """
        In the planning thread (CruisePlanner.no_path_job): no path from the node the robot
        walks to towards the goal, or (why) no way on for the main thread (block_request). If
        from there the graph goes on (the goal is the problem,
        not the robot): the goal is excluded for the mission and another one chosen in the cell
        (up to MAX_BLOCK_REPLANS per cell); with none, the cell is unreachable now and the
        next one is chosen (_prepare_next_cell; the cell is not marked done nor failed).
        Returns the 'switch' of the plan: kind 'goal' or 'cell' with the new path ids, or kind
        'end' with the reason (the cruise ends).
        """
        t0 = time.perf_counter()
        env, prm = self.env, self.prm_graph
        row, col = planner.target_cell
        prm.set_robot_node(next_id, 0.0)
        n_free = prm.reachable_count(next_id, P.REPLAN_FAIL_ROBOT_FREE_NODES)
        if n_free < P.REPLAN_FAIL_ROBOT_FREE_NODES:
            planner.no_path_job = None
            return dict(kind='end', reason=(f"no path from node {next_id}, and from there the graph reaches only "
                                            f"{n_free} nodes: the robot is shut in, not the goal"))
        gx, gy = prm.get_node_position(planner.goal_id)
        goal_choice._refuse_goal(env, row, col, gx, gy, why or "no path in the updated graph")
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

    def _apply_switch(self, sw):
        """Main thread, a plan with what the planning thread prepared (see _prepare_next_cell,
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
            # The GraphNav waypoint of the cell done (a call to the robot, ~0.8 s) is made after
            # the first command towards the next cell (_visit_after_send), so the robot does not
            # finish the old command and stop meanwhile.
            cb = getattr(self.env, '_on_chained_visit', None)
            if cb is not None:
                self._pending_visit = functools.partial(cb, done[0], done[1])
            what = f"Cell ({done[0]},{done[1]}) done while walking ({sw['why']})"
        else:
            left = sw['left']
            self.env._cells_left_unreachable = getattr(self.env, '_cells_left_unreachable', []) + [left]
            what = f"Cell ({left[0]},{left[1]}) left while walking ({sw['why']}: not failed, unreachable now)"
        self.target_row, self.target_col = nxt
        self.tried_goals, self.goal_repicks = [], 0
        self.env._chain_choice = sw['choice']
        MISSION_STATS['cruise_switches'] = MISSION_STATS.get('cruise_switches', 0) + 1
        HANDOFF.mark("next cell prepared")
        print(f"[CRUISE] {what}: next cell ({nxt[0]},{nxt[1]}), goal ({sw['goal'][0]:.2f}, {sw['goal'][1]:.2f}), "
              f"{sw['n_nodes']} nodes, prepared in the planning thread in {sw['prep_s']:.2f} s.")

    def _visit_after_send(self):
        """The waypoint of the cell done (see _apply_switch), once the robot walks towards the
        next cell. The cell is marked visited there too."""
        job, self._pending_visit = getattr(self, '_pending_visit', None), None
        if job is not None:
            # The waypoint is recorded where the robot is now (create_default_waypoint).
            x, y, _, _ = connection.getPosition(self.robot_state_client)
            job(x, y)
            HANDOFF.mark("waypoint of the cell done, while walking")

    def _seen_cell(self):
        """(env, row, col) of the target cell for the planner's 'cell_seen'."""
        return (self.env, self.target_row, self.target_col)

    def _moving_check(self):
        """
        Turn of the loop with the robot still walking from the previous cell (hand-off): it
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
        stands (less than 1 cm and 1 deg in 0.1 s), at most CRUISE_SETTLE_S: a move started from
        a pose still moving begins off its line, and the stop-and-go scans with the robot
        standing."""
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
        """The robot node after a cruise: an end of the edge it is on, not the nearest node. The
        node ahead if the edge is still in the graph (or the robot is on it), otherwise the node
        it came from, since the part just walked is free. The nearest node could be behind or
        beside the robot, with a first stretch robot -> node that is not an edge."""
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
