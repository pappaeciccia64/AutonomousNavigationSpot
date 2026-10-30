"""Decisions on the frontier: the step to take (decide_next_move), the rotation, the fan of exit directions, cone, back-off, exits on arrival, shortcuts."""
import numpy as np
from spot_explorer import config as P
from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.perception import geometry
from spot_explorer.planning.safe_frontier import (GridSnapshot, allowed_advance, compute_safe_frontier, point_along)


def decide_next_move(robot_xy, path_waypoints, frontier, no_progress_count, in_target_cell=True):
    """
    What to do now, given the safe frontier. Pure function: no robot, no I/O, so it can be
    tested off the field.

    path_waypoints: [(node_id, x, y), ...] -- the nodes still to reach, the first is the
        next. The robot position is NOT included.
    frontier: result of safe_frontier.compute_safe_frontier on the path
        [robot] + waypoints.
    no_progress_count: consecutive scans in which no advance was possible.
    in_target_cell: the robot is already inside the goal cell, or outside by at most
        ARRIVAL_OUTSIDE_CELL_M (the caller decides). Arrival by tolerance only counts
        there: stopping 0.65 m before a goal near the border may mean staying in the
        previous cell. Otherwise it continues as for a blocked frontier (wait, then block).

    Returns a dict with 'kind':
      'reached_node'  the next node is already under the robot (< 5 cm): just consume it
      'move'          move towards 'target'; 'full' = True if the target is the node itself.
                      'final' = True: step below MIN_PARTIAL_STEP_M that brings it within
                      the arrival tolerance (see FINAL_STEP_MIN_M).
                      'nodes' = how many path nodes lie inside the advance allowed by the
                      frontier: the loop travels them one after the other without
                      rescanning (see MULTI-NODE ADVANCE). It is 1 or more when 'full',
                      and absent when the step is partial.
      'arrived'       no advance is possible, but the final goal is within tolerance and
                      the robot is inside the goal cell
      'wait'          no advance is possible: rescan standing still before concluding
      'blocked'       no advance has been possible for FRONTIER_BLOCK_CONFIRM_SCANS
                      consecutive scans: block confirmed up close. 'segment' = index of
                      the path leg where the frontier stopped (0 = from the robot to the
                      first node).
    """
    rx, ry = robot_xy
    _, nx, ny = path_waypoints[0]
    seg_len = float(np.hypot(nx - rx, ny - ry))
    if seg_len < 0.05:
        return {'kind': 'reached_node'}

    adv = allowed_advance(frontier)
    if adv >= seg_len - 0.02:
        # --- MULTI-NODE ADVANCE -------------------------------------------------------
        # The safe frontier covers the WHOLE planned path, not the first leg:
        # it returns how far it is confirmed free now, often several nodes ahead.
        # ALL the nodes inside the allowed advance are counted, and the
        # loop travels them one after the other WITHOUT a scan, refresh or replan
        # in between, instead of stopping and rescanning at every node. It always walks on the
        # polyline, one leg at a time: it does NOT cut the chord between two nodes, which
        # would pass over unverified terrain (that is what shortcut_index does, but only
        # after verifying the chord with the frontier).
        # The check does NOT become a remembered verdict: during the move the thread
        # recomputes the frontier at ~5 Hz on the remaining path and stops the robot if it
        # gets shorter (see _should_abort_move and FRONTIER_ABORT_CONFIRM). What is removed is
        # the STOPS, not the checks.
        cum, nodes = 0.0, 0
        px, py = rx, ry
        for (_, wx, wy) in path_waypoints:
            cum += float(np.hypot(wx - px, wy - py))
            if cum > adv + 0.02:
                break
            px, py, nodes = wx, wy, nodes + 1
        return {'kind': 'move', 'target': (nx, ny), 'full': True, 'dist': seg_len,
                'nodes': max(1, nodes)}
    # FINAL STEP (see FINAL_STEP_MIN_M): even below the minimum step, if after the step
    # the goal is within the arrival tolerance and the robot is in the cell.
    final = (in_target_cell and adv >= P.FINAL_STEP_MIN_M
             and frontier['path_len'] > P.GOAL_REACHED_TOLERANCE_M
             and frontier['path_len'] - adv <= P.GOAL_REACHED_TOLERANCE_M)
    if adv >= P.MIN_PARTIAL_STEP_M or final:
        f = adv / seg_len
        out = {'kind': 'move', 'target': (rx + f * (nx - rx), ry + f * (ny - ry)),
               'full': False, 'dist': adv}
        if final and adv < P.MIN_PARTIAL_STEP_M:
            out['final'] = True
        return out

    if frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M and in_target_cell:
        return {'kind': 'arrived'}
    if no_progress_count + 1 < P.FRONTIER_BLOCK_CONFIRM_SCANS:
        return {'kind': 'wait'}

    polyline = [(rx, ry)] + [(x, y) for _, x, y in path_waypoints]
    _, seg_idx = point_along(polyline, frontier['dist'])
    return {'kind': 'blocked', 'segment': int(seg_idx)}


def rotation_plan(robot_yaw, robot_xy, target_xy, od_robot, snap=None):
    """
    Can the robot rotate in place to face the point to reach?

    Rotating by a given angle the body sweeps a sector, not the whole circle of radius
    0.60 around the robot: the cells really swept are checked. Requiring the whole circle
    free (obstacle_distance under the robot >= ROTATION_CLEARANCE_M) is the worst case over
    all directions and all angles, and would refuse many safe rotations.

    Order of the checks, cheapest first:
      1. rotation below ROTATION_CHECK_MIN_DYAW_DEG -- negligible, rotate
      2. obstacle_distance under the robot >= ROTATION_CLEARANCE_M -- circumscribed circle
         free, rotate without further computation (prefilter)
      3. geometry.rotation_is_clear() on the cells really swept (0.26 ms)

    Without `snap` (no snapshot of the grid) it falls back on the scalar comparison alone.

    Pure function, testable off the field. Returns a dict with:
      turn (bool), dyaw_deg, motion_dir (rad), margin (margin of the swept sector, in
      metres, or None if not computed).
    """
    rx, ry = robot_xy
    tx, ty = target_xy
    motion_dir = float(np.arctan2(ty - ry, tx - rx))
    dyaw = float(np.degrees(np.arctan2(np.sin(motion_dir - robot_yaw), np.cos(motion_dir - robot_yaw))))

    if abs(dyaw) < geometry.ROTATION_CHECK_MIN_DYAW_DEG:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'rotazione trascurabile'}
    if od_robot >= geometry.ROTATION_CLEARANCE_M:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'cerchio circoscritto libero'}
    if snap is not None:
        ok, margin = geometry.rotation_is_clear(
            snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
            rx, ry, robot_yaw, float(np.radians(dyaw)))
        return {'turn': bool(ok), 'dyaw_deg': dyaw, 'motion_dir': motion_dir,
                'margin': float(margin) if np.isfinite(margin) else None,
                'why': 'settore spazzato'}
    return {'turn': False, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
            'why': 'nessun dato di griglia'}


def shortcut_index(robot_xy, path_waypoints, snap, global_map=None, prm=None, robot_node_id=None):
    """
    Shortcut: the index of the FARTHEST waypoint reachable in a straight line from the
    robot with the stretch entirely confirmed free by the safe frontier (nose included).
    0 = no shortcut. Removes useless turns between PRM nodes.
    The check is the frontier itself on the live data: obstacles, roughness, slope.

    And also the GLOBAL MAP, with the same margin as the PRM: some things are only there --
    the spot of a fall, for example, which to the sensors may look like normal terrain
    (that is why the robot fell there). The PRM edges already avoid them; a shortcut
    checked only live could take the robot back over them.

    And the EDGES ALREADY DISCARDED. Without this check the shortcut would re-attach a
    waypoint whose direct link has just been invalidated, and the loop would never get
    out: the leg is discarded, Dijkstra finds a way that still goes through that node via
    the intermediate nodes, and the shortcut points straight at it again. Needs `prm` and
    `robot_node_id`; without them, the check is skipped.
    """
    rx, ry = robot_xy
    for k in range(len(path_waypoints) - 1, 0, -1):
        wid, wx, wy = path_waypoints[k]
        if prm is not None and robot_node_id is not None:
            key = (min(robot_node_id, wid), max(robot_node_id, wid))
            if key in getattr(prm, 'tracker_blocked_edges', ()):
                continue
            # Also the soft discards (mark_edge_blocked_soft: [SUBITO], rotation refused),
            # otherwise the shortcut points again at the node just discarded for rotation.
            if getattr(prm, 'edge_validity', {}).get(key) is False:
                continue
        d = float(np.hypot(wx - rx, wy - ry))
        if d > P.SHORTCUT_MAX_DIST_M:
            continue
        if global_map is not None:
            n = max(prm_graph.OCCUPANCY_SAMPLES_MIN, int(np.ceil(d * prm_graph.OCCUPANCY_SAMPLES_PER_M)) + 1)
            if any(global_map.is_occupied(rx + t * (wx - rx), ry + t * (wy - ry),
                                          safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
                   for t in np.linspace(0.0, 1.0, n)[1:]):
                continue
        f = compute_safe_frontier([(rx, ry), (wx, wy)], snap)
        if f['reason'] == 'fine_percorso':
            return k
    return 0


def plan_straight_retreat(robot_xy, robot_yaw, snap, max_dist=P.ROTATION_RETREAT_M):
    """
    How far the robot can back up IN A STRAIGHT LINE, backwards, without turning.

    It is not oblique walking: moving along its own axis the footprint stays 0.50 m wide
    and the frontier applies the normal FRONTIER_CLEARANCE_M threshold, not the 0.59 of
    sideways walking. It is the way out when rotating towards the next point is not
    possible: it moves away from what prevents turning and at the next turn it tries again
    from a different position.

    Pure function. Returns (x, y, distance) of the reachable point, or None if it cannot
    back up by at least ROTATION_RETREAT_MIN_M.
    """
    rx, ry = robot_xy
    back_x = rx - max_dist * np.cos(robot_yaw)
    back_y = ry - max_dist * np.sin(robot_yaw)
    body = geometry.body_extents(robot_yaw, robot_yaw + np.pi)   # backwards: (0.55, 0.25)
    f = compute_safe_frontier([(rx, ry), (back_x, back_y)], snap, body=body)
    adv = allowed_advance(f)
    if adv < P.ROTATION_RETREAT_MIN_M:
        return None
    adv = min(adv, max_dist)
    return (float(rx - adv * np.cos(robot_yaw)), float(ry - adv * np.sin(robot_yaw)), float(adv))


def scan_escape_directions(robot_xy, robot_yaw, snap, step_deg=P.ESCAPE_FAN_STEP_DEG,
                           min_advance_m=P.ESCAPE_FAN_MIN_ADVANCE_M, probe_m=3.0):
    """
    The full circle: in which directions can the robot REALLY go from here?

    For each direction of the circle (step step_deg) it measures two independent things:
      - how much space the safe frontier allows going in a straight line in that direction;
      - whether the robot can turn enough to point its nose there (geometry.rotation_is_clear).
    A direction is USABLE only if both answer yes. The robot does not walk sideways (see
    ROTATION AND BACK-OFF in config.py), so a wide lane that requires an impossible rotation
    is NOT a way out, however wide it is.

    Used to recognize a trap (zero usable directions: robot in a corner, with the lanes
    open behind rotations that cannot be made there) with 24 frontier evaluations, instead
    of finding out by discarding edges one by one, one scan after another.

    Pure function. Returns the list of the directions examined, each with:
    dir_deg (absolute), rel_deg (rotation needed), advance, reason, rotatable, usable.
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    out = []
    for deg in np.arange(-180.0, 180.0, abs(step_deg)):
        a = float(np.radians(deg))
        f = compute_safe_frontier(
            [(rx, ry), (rx + probe_m * np.cos(a), ry + probe_m * np.sin(a))], snap)
        adv = allowed_advance(f)
        rel = float(np.degrees(np.arctan2(np.sin(a - robot_yaw), np.cos(a - robot_yaw))))
        if abs(rel) < geometry.ROTATION_CHECK_MIN_DYAW_DEG:
            rotatable = True
        else:
            rotatable = bool(geometry.rotation_is_clear(
                snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
                rx, ry, float(robot_yaw), float(np.radians(rel)))[0])
        out.append({'dir_deg': float(deg), 'rel_deg': rel, 'advance': float(adv),
                    'reason': f['reason'], 'rotatable': rotatable,
                    'usable': bool(rotatable and adv >= min_advance_m)})
    return out


def describe_escape_fan(dirs, max_rows=4):
    """Compact log line: how many usable directions and the best ones, for the logs."""
    usable = [d for d in dirs if d['usable']]
    best = sorted(dirs, key=lambda d: -d['advance'])[:max_rows]
    txt = "; ".join(f"{d['dir_deg']:+.0f} gradi: {d['advance']:.2f} m "
                    f"({'ruotabile' if d['rotatable'] else 'NON ruotabile'})" for d in best)
    return len(usable), txt


def is_trapped(robot_xy, robot_yaw, snap):
    """
    True if NO direction of the circle is both travelable and reachable by turning: the
    condition the user set for going back ("only if it can no longer go forward or
    rotate"), checked in one scan instead of twenty.
    Returns (trapped, dirs) -- dirs is for the log, so the data shows why.
    """
    dirs = scan_escape_directions(robot_xy, robot_yaw, snap)
    return (not any(d['usable'] for d in dirs)), dirs


def arrival_has_exit(snap, arrival_xy, heading, min_advance_m=P.ESCAPE_FAN_MIN_ADVANCE_M):
    """
    From the point where I am about to stop, will there be an exit?

    The safe frontier looks at the road AHEAD, not at whether it can set off again from
    the arrival pose: with a short step the robot can enter a corner with no exits.

    The fan is evaluated at the arrival point using the CURRENT snapshot: it is an
    estimate (from there the robot will see more), but it is the same information the
    decision to go there is based on, so the decision is at least consistent with itself.
    To avoid paying for it on every move only the main loop calls it, and only for short
    steps or towards tight spaces -- see ARRIVAL_EXIT_CHECK_MAX_STEP_M / _OD_M.

    Returns (has_exit, dirs).
    """
    ax, ay = float(arrival_xy[0]), float(arrival_xy[1])
    # Same grid, but with the robot declared at the arrival point: that is what changes the
    # reference of the frontier's exit rule and the centre of the rotations tried.
    probe = GridSnapshot(
        snap.obstacle_dist, snap.rough_veto, snap.seen, snap.terrain,
        snap.origin_x, snap.origin_y, snap.cell_size, ax, ay,
        timestamp=snap.time, valid=snap.valid, blocked_latch=snap.blocked_latch)
    dirs = scan_escape_directions((ax, ay), float(heading), probe, min_advance_m=min_advance_m)
    # A turnable direction that stops because THE DATA RUNS OUT counts as an exit: from the
    # arrival point the robot will see more, and "not seen" does not mean "closed". Without
    # this, in a corridor the forward direction ended at the edge of the scan (0.09 m) and the
    # step was refused; at the third refusal it went anyway, and it was the right step
    # (28-10 12:16, and scenario S16 of test_e2e_loop.py).
    return any(d['usable'] or (d['rotatable'] and d['reason'] == 'fine_dati') for d in dirs), dirs


def plan_cone_move(robot_xy, robot_yaw, snap, goal_xy, max_dyaw_deg=P.CONE_SWEEP_MAX_DEG,
                   step_deg=P.CONE_SWEEP_STEP_DEG, min_gain_m=P.CONE_MIN_GAIN_M):
    """
    "Rotate as much as you CAN, not as much as you would like, and walk."

    When the rotation towards the next node is refused, the right question is no longer
    "can I turn towards the node?" but "what is the largest rotation I can make here, and
    in which direction inside that cone is it best to go?". In a narrow corridor turning
    around takes 1.21 m of diameter and cannot be done, but walking can: often going
    straight, or almost, gets closer to the goal than any back-off.

    Selection criterion: among the directions that bring it CLOSER to the goal by at
    least min_gain_m, the one with the most space confirmed free is taken. Not the most
    free in absolute terms: that would take the robot away from the cell.

    Pure function. Returns None if no direction of the cone gets closer, otherwise a dict
    with: dyaw_deg (rotation to make, degrees, may be 0), target (x, y), dist (advance),
    gain (metres gained towards the goal), reason (frontier reason).
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    d0 = float(np.hypot(gx - rx, gy - ry))

    best = None
    rels = np.arange(-abs(max_dyaw_deg), abs(max_dyaw_deg) + 1e-9, abs(step_deg))
    for rel in rels:
        # The rotation must be possible: below the negligible threshold nothing is checked
        # (that is what rotation_plan does too), above it the sector is measured.
        if abs(rel) >= geometry.ROTATION_CHECK_MIN_DYAW_DEG:
            ok, _ = geometry.rotation_is_clear(
                snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
                rx, ry, robot_yaw, float(np.radians(rel)))
            if not ok:
                continue

        heading = robot_yaw + float(np.radians(rel))
        far_x, far_y = rx + 3.0 * np.cos(heading), ry + 3.0 * np.sin(heading)
        f = compute_safe_frontier([(rx, ry), (far_x, far_y)], snap)
        adv = allowed_advance(f)
        if adv < P.MIN_PARTIAL_STEP_M:
            continue

        nx, ny = rx + adv * np.cos(heading), ry + adv * np.sin(heading)
        gain = d0 - float(np.hypot(gx - nx, gy - ny))
        if gain < min_gain_m:
            continue
        cand = {'dyaw_deg': float(rel), 'target': (float(nx), float(ny)), 'dist': float(adv),
                'gain': float(gain), 'reason': f['reason']}
        # More free space wins; with equal space, the smaller rotation.
        if best is None or (cand['dist'], -abs(cand['dyaw_deg'])) > (best['dist'], -abs(best['dyaw_deg'])):
            best = cand
    return best


def retreat_would_help(snap, back_xy, robot_yaw, dyaw_deg):
    """
    Would the 0.60 m back-off make the needed rotation possible?

    In a corridor the geometry 0.60 m further back is the same: backing up does not open
    space, it moves it. It backs up only if from there the rotation becomes possible.

    The rotation is evaluated at the ARRIVAL point using the current snapshot -- it is an
    estimate, because from there the robot will see a bit more, but it is the same
    information the decision to back up was based on, so at least the decision is
    consistent with itself.
    """
    if abs(dyaw_deg) < geometry.ROTATION_CHECK_MIN_DYAW_DEG:
        return True, None
    ok, margin = geometry.rotation_is_clear(
        snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
        float(back_xy[0]), float(back_xy[1]), float(robot_yaw), float(np.radians(dyaw_deg)))
    return bool(ok), margin
