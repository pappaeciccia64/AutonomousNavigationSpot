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
        reports it blocked); no new plan for CRUISE_PLAN_STALE_S; low battery; CRUISE_MAX_S.

While the thread plans the main thread does not touch graph and maps: it sends commands,
reads plans and the robot state. It takes them back after stop_planning().

WITHOUT STOPPING, while the robot walks on the last stretch confirmed (the command lasts
PATH_HOLD_S, so the main thread has that long):
  - goal isolated (plan 'no_path'): the planning pauses for the cycle running to end, a new
    goal is chosen in the cell (_repick_goal, as the stop-and-go does) and the cruise goes
    on towards it (_cruise_repick);
  - CHAIN_CELLS: inside the target cell and within CRUISE_HANDOFF_M of the goal (or within
    GOAL_REACHED_TOLERANCE_M with the last stretch closed by an obstacle), the attempt
    ends successfully with the robot still walking towards the goal (env._moving_pc): the
    mission marks the cell and the next attempt starts while the robot walks. Its first turn
    goes on in cruise if the rotation is clear: within PATH_START_IN_PLACE_DEG of the nose it
    turns while walking, beyond it on the spot inside the new command if the swept area is
    clear with CHAIN_TURN_EXTRA_AIR_M more air (_moving_check); otherwise, and for any other
    decision, the robot is stopped first and the turn is made standing as today. An attempt
    that fails before its loop (no valid goal, cell not reachable in the graph: only
    computations) leaves the robot walking, and the mission goes on to the next cell. stop_if_moving stops it before
    GraphNav and at the end of the exploration.
"""
import re
import threading
import time

import numpy as np

from spot_explorer import config as P
from spot_explorer.graphnav import map_recording
from spot_explorer.mission import planning_cycle
from spot_explorer.mission.attempt_common import CONTINUE, STOP, battery_return_due
from spot_explorer.perception import gait
from spot_explorer.perception import geometry
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier
from spot_explorer.records import MISSION_STATS, BodyTiltRecorder, append_trajectory, timing_add
from spot_explorer.robot import connection
from spot_explorer.robot import motion

CONTINUOUS_MOTION = True
CRUISE_POLL_S = 0.2             # main thread: one feedback reading every 0.2 s
CRUISE_PLAN_STALE_S = 1.2       # no new plan for this long: stop
CRUISE_MIN_AHEAD_M = 0.10       # confirmed stretch ahead of the robot below this: stop
CRUISE_BAD_PLANS = 3            # consecutive plans without scan or with an error: stop
CRUISE_MAX_S = 180.0
CRUISE_TRACK_STEP_M = 0.30      # spacing of the walked track kept for the retreat
CRUISE_BREADCRUMB_S = 1.0       # how often the GraphNav breadcrumb is tried
CRUISE_BATTERY_S = 5.0          # how often the battery is read
CRUISE_ARRIVAL_WAIT_S = 2.0     # at the goal: wait at most this for the command to end
CRUISE_SETTLE_S = 1.0           # after a stop: wait at most this for the robot to stand still
CHAIN_CELLS = True              # from a cell goal to the next cell without stopping (see above)
CRUISE_HANDOFF_M = 0.6          # distance from the goal at which the next cell is chosen
# A bigger turn than PATH_START_IN_PLACE_DEG with the robot still walking: rotation on the spot
# inside the new command, without stopping first, if the swept area is clear with this much
# more air. The check is made at the pose of the scan, and the robot still walks some cm
# before the command arrives.
CHAIN_TURN_EXTRA_AIR_M = 0.15


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


def _tier(grad, rough):
    """The level of motion.select_mobility_params, without choosing (and printing)."""
    if grad < gait.GAIT_PLAIN_SLOPE_MAX and rough < gait.GAIT_PLAIN_ROUGH_MAX:
        return 'PLAIN'
    if grad < gait.GAIT_MODERATE_SLOPE_MAX and rough < gait.GAIT_MODERATE_ROUGH_MAX:
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
        """Gait for the worst terrain of the confirmed stretch; chosen again (and printed) only
        when the level changes."""
        if self.mobility_kwargs:
            return self.mobility_kwargs
        tier = _tier(*terrain)
        if tier != getattr(self, '_cruise_tier', None):
            self._cruise_kwargs = motion.select_mobility_params(*terrain)
            self._cruise_tier = tier
        return self._cruise_kwargs

    def _cruise(self):
        """The cruise (see the module docstring). Returns CONTINUE: the loop looks again
        with a fresh scan, as after a move."""
        tracker = self.verification_tracker
        ids = [self.robot_node_id] + [w[0] for w in self.path_waypoints]
        planner = planning_cycle.CruisePlanner(self.local_grid, self.global_grid, self.prm_graph,
                                               self.robot_state_client, self.goal_id, ids)
        planner.bundle = self._cruise_bundle_info()
        # First stretch: the one confirmed by the frontier of this turn, with the same corner
        # check the planner will make.
        adv, turn_stop = planner._turns_within(self.polyline, ids[1:],
                                               safe_frontier.allowed_advance(self.frontier), self.snap)
        confirmed = planning_cycle._cut_polyline(self.polyline, adv)
        kwargs = self._cruise_gait(planner._terrain(confirmed, ids[1:]))
        path_len = float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.polyline, self.polyline[1:])))
        to_goal = turn_stop is None and adv >= path_len - 0.01
        print(f"[CRUISE] Start from node {self.robot_node_id} towards the goal "
              f"({self.target_x:.2f}, {self.target_y:.2f}): {len(ids) - 1} nodes, {path_len:.2f} m, "
              f"{adv:.2f} m confirmed" + (f" (rotation not clear at node {turn_stop})" if turn_stop else "") + ".")

        self.need_refresh = True                  # after the cruise: see EVENT REFRESH
        tracker.clear_path()
        # Robot still walking from the previous cell (CHAIN_CELLS): this command replaces that
        # one, turning while walking (_moving_check has let only small turns through).
        start_in_place = (motion.PATH_START_IN_PLACE_DEG if getattr(self.env, '_moving_pc', None) is not None
                          else motion.LEG_START_IN_PLACE_DEG)
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
        reason, arrived = None, False
        n_plans = n_sends = bad = 0
        last_version, last_plan_t = 0, t0
        last_crumb_t, last_battery_t = t0, t0
        crumb = None
        try:
            pc.send(confirmed[1:], start_in_place_deg=start_in_place, **kwargs)
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
                if plan['status'] == 'edge_blocked':
                    reason = "the edge being walked is out of the graph"
                    break
                if plan['status'] == 'no_path':
                    new_planner = self._cruise_repick(planner, plan)
                    if new_planner is None:
                        reason = "no path to the goal from the next node"
                        break
                    cycles_done += planner.cycles
                    replans_done += planner.replans
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
                      + f" | cycle {plan['cycle_s']:.2f} s, robot at {100 * r['deviation']:.0f} cm from the line"
                      + (" | edges: slopes {0} recomputed, {1} kept, {2} untouched, {3} out (live scan)".format(*plan['edges'])
                         + (f", {plan['edges'][4]} nodes pruned" if plan['edges'][4] else "")
                         if plan.get('edges') else ""))
                # The new stretch must start where the robot is: after a path change while
                # walking the line can pass beside it (see PathCommand.send).
                off = motion.distance_from_polyline(rx, ry, plan['confirmed'])
                if off > motion.PATH_MAX_DEVIATION_M:
                    reason = (f"the new path passes {100 * off:.0f} cm from the robot "
                              f"(limit {100 * motion.PATH_MAX_DEVIATION_M:.0f} cm)")
                    break
                # Hand-off to the next cell (CHAIN_CELLS), robot inside the target cell: goal
                # confirmed and within CRUISE_HANDOFF_M; or the frontier stops for an obstacle
                # (roughness, slope) with the goal within GOAL_REACHED_TOLERANCE_M along the
                # path, the arrival the stop-and-go would accept standing (decisions: 'arrived';
                # 03-11 15:39, 4 cells out of 6 ended like that, each with a stop).
                to_go = (sum(float(np.hypot(b[0] - a[0], b[1] - a[1]))
                             for a, b in zip(plan['polyline'], plan['polyline'][1:]))
                         - _along(plan['polyline'], rx, ry))
                if CHAIN_CELLS and self.env.is_point_in_cell(rx, ry, self.target_row, self.target_col):
                    if plan['to_goal'] and to_go <= CRUISE_HANDOFF_M:
                        why = f"{to_go:.2f} m from the goal"
                    elif (not plan['to_goal'] and f['reason'] in ('obstacle', 'roughness', 'slope')
                          and plan['turn_stop'] is None and to_go <= P.GOAL_REACHED_TOLERANCE_M):
                        why = (f"{to_go:.2f} m from the goal, the last stretch closed by "
                               f"{f['reason']}: within the arrival tolerance of {P.GOAL_REACHED_TOLERANCE_M:.2f} m")
                    else:
                        why = None
                    if why is not None:
                        reason, handoff = f"in the cell, {why}: next cell while walking", True
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
        finally:
            if handoff:
                self.env._moving_pc = pc          # still walking to the goal: see CHAIN_CELLS
            elif not arrived:
                pc.stop(f"end of the cruise: {reason}")
                self._cruise_settle(pc)
            tracker.stop_planning()
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
        if handoff:
            self._handoff = True
            MISSION_STATS['cruise_handoffs'] = MISSION_STATS.get('cruise_handoffs', 0) + 1
            return STOP
        return outcome

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
        self._stop_node_here(x, y, "stop (cruise)")
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
        return CONTINUE
