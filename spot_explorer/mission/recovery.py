"""Getting out of a block: going back with GraphNav, walked retreat, blocked or deferred cells."""
import numpy as np

from spot_explorer.graphnav import navigation
from spot_explorer.robot import motion
from spot_explorer.perception import geometry
from spot_explorer.robot import connection
from spot_explorer.planning import safe_frontier

from spot_explorer.planning import roadmap

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, TrajectoryLog



# =========================================================================
# GOING BACK WITH GRAPHNAV, when from where the robot is the graph does not lead
# to the cell
# =========================================================================


def backtracking_cannot_help(env, prm_graph, goal_id, robot_state_client, max_hops,
                             min_dist_m=0.40):
    """
    Can going back with GraphNav achieve anything?

    escape_by_backtracking goes back to a breadcrumb and ONLY THEN asks the graph for a path
    from there. But every breadcrumb already has its PRM node: the same question can be asked
    before moving. If the goal is in another component of the graph, no waypoint behind can
    change the answer, and the returns would only be wasted travel.

    Answers True only if EVERY breadcrumb that would be visited has a PRM node connected in
    the graph and there is no path from any of them. If even one cannot be judged (missing
    or isolated node, e.g. discarded inside an obstacle) it answers False and the robot goes
    back: when in doubt, the way out is not taken away.
    """
    trail = getattr(env, '_graphnav_trail', None) or []
    if not trail or goal_id not in prm_graph.nodes:
        return False
    try:
        rx, ry, _, _ = connection.getPosition(robot_state_client)
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
    Going back (escape_by_backtracking) with MEMORY for the attempt.

    trap_state: the attempt's dict ({'n': returns already made}), never reset within the
    attempt. block_arc: (a, b), the edge the robot stopped on.

    Before replanning, the edge becomes impassable for the rest of the attempt (attempt
    penalty: it survives refreshes and rebuilds and is removed only at the start of the
    next attempt, see roadmap.clear_penalties_for_attempt). From there the planner must
    propose another way, or admit there is none. At the MAX_BACKTRACKS_PER_ATTEMPT-th
    return the robot goes back anyway -- it cannot stay where it is -- but the attempt
    ends: the cell is deferred and tried from another side. Without this memory the robot
    would repeat the same loop forever.
    """
    trap_state['n'] = trap_state.get('n', 0) + 1
    if block_arc is not None and block_arc[0] is not None and block_arc[1] is not None:
        a, b = int(block_arc[0]), int(block_arc[1])
        if a != b:
            key = (min(a, b), max(a, b))
            prm_graph.edge_penalty_persistent[key] = prm_graph._ATTEMPT_MARK
            print(f"[RETURN] Edge {a}->{b} out of the plan for the rest of the attempt "
                  f"(return {trap_state['n']}/{P.MAX_BACKTRACKS_PER_ATTEMPT}).")
    esc = escape_by_backtracking(
        env, prm_graph, goal_id, recordingInterface, robot_state_client, command_client,
        global_map, mobility_kwargs, pts, obstacle_mask, label=label, **kwargs)
    if trap_state['n'] >= P.MAX_BACKTRACKS_PER_ATTEMPT and esc.get('ok'):
        MISSION_STATS['backtrack_giveups'] = MISSION_STATS.get('backtrack_giveups', 0) + 1
        print(f"[RETURN] {trap_state['n']} returns in this attempt: I went "
              f"back, but I do not retry. I defer the cell and try it from another side.")
        esc = dict(esc, ok=False, how='gave up after repeated returns')
    return esc


def escape_by_backtracking(env, prm_graph, goal_id, recordingInterface, robot_state_client,
                           command_client, global_map, mobility_kwargs, pts, obstacle_mask,
                           max_hops=P.GRAPHNAV_MAX_BACKTRACK_HOPS, label="",
                           skip_if_graph_cannot_help=False):
    """
    The way out when from here there is nowhere to go.

    Go back one waypoint with GraphNav and try to replan towards the goal from there; if
    that is not possible, go back another one, and so on up to max_hops. After max_hops the
    cell is deferred and the next priority is taken.

    If GraphNav cannot do it (empty track, not localized, STUCK, timeout) it falls back on
    the walked retreat: it is worse, but it is the last safety net and we keep it. In a
    trap where rotating is impossible GraphNav, which wants to turn the nose to follow an
    edge, can return STUCK exactly where it is needed.

    Returns a dict: ok (the mission can continue), ids (new path or None), robot_node (new
    robot node or None), how (what happened, for the log).
    """
    # Only when the robot is NOT trapped (callers "no alternative in the graph"):
    # in a trap or after a physical failure going back also serves to free it.
    if skip_if_graph_cannot_help and backtracking_cannot_help(
            env, prm_graph, goal_id, robot_state_client, max_hops):
        print(f"[RETURN] {label}: from the nodes of the next {max_hops} breadcrumbs the graph does not "
              f"reach the goal anyway (it does not depend on where I am). I do not go "
              f"back: I defer the cell at once.")
        MISSION_STATS['backtracks_skipped'] = MISSION_STATS.get('backtracks_skipped', 0) + 1
        return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cell deferred'}

    for hop in range(1, int(max_hops) + 1):
        # Arrive facing the attempt's goal: from there replan towards it.
        ok, crumb = navigation.graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                                                     face_xy=prm_graph.get_node_position(goal_id))
        if not ok:
            if crumb is None:
                print(f"[RETURN] {label}: no GraphNav breadcrumb available.")
            else:
                print(f"[RETURN] {label}: GraphNav could not go back to "
                      f"{crumb['wp_name']}.")
            MISSION_STATS['graphnav_backtrack_fail'] += 1
            print("[RETURN] Falling back on the walked retreat along the travelled legs.")
            retreat_along_traveled_path(env, robot_state_client, command_client,
                                        mobility_kwargs=mobility_kwargs,
                                        pts=pts, obstacle_mask=obstacle_mask)
            return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'walked retreat'}

        if not crumb.get('in_place'):
            MISSION_STATS['graphnav_backtracks'] += 1
        prm_graph.needs_refresh = True       # the robot has moved: see EVENT REFRESH
        # The scan of the preliminary discard was taken from another point (and the
        # body exclusion would be where the robot was): drop it, until the next scan.
        prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
        try:
            ax, ay, _, _ = connection.getPosition(robot_state_client)
        except Exception:
            ax, ay = crumb['x'], crumb['y']
        new_node = prm_graph.add_stop_node(
            ax, ay, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"stop (GraphNav return to {crumb['wp_name']})")
        # No global-map exemption around this node: see the comment at the
        # caller of set_robot_node in the main loop.
        prm_graph.set_robot_node(new_node, 0.0)

        ids = prm_graph.find_path_dijkstra(new_node, goal_id)
        if ids is not None and len(ids) > 1:
            print(f"[RETURN] {label}: back at {crumb['wp_name']} ({hop}/{max_hops}) and from here "
                  f"there is a path with {len(ids) - 1} legs. Resuming.")
            return {'ok': True, 'ids': ids, 'robot_node': new_node, 'how': 'replanned'}
        print(f"[RETURN] {label}: back at {crumb['wp_name']} ({hop}/{max_hops}) but from here the "
              f"graph offers no path to the goal.")
        if crumb.get('in_place'):
            break           # there are no farther breadcrumbs: more rounds would be the same

    print(f"[RETURN] {label}: {hop} returns without finding a path. I defer the cell.")
    return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cell deferred'}



# =========================================================================
# WALKED RETREAT backwards: the last safety net
# =========================================================================


def retreat_along_traveled_path(env, robot_state_client, command_client,
                                max_segments=P.RETREAT_MAX_SEGMENTS,
                                mobility_kwargs=None,
                                pts=None, obstacle_mask=None):
    """
    Retreat: go back over its own steps walking BACKWARDS, without ever turning.
    It is the last safety net, when the return with GraphNav fails (see
    escape_by_backtracking).

    When the robot gets into a narrow space and there is no longer a path towards the goal
    from there, turning around is not a way out -- it needs space that is not there at that
    point. Backing up instead retraces the gap it came in through, which by construction
    was wide enough to pass. And backing up along the body axis is not sideways walking:
    the footprint stays 0.50 m wide.

    The points it passed through are already recorded in env._traveled_arcs, filled at
    every completed move. They are retraced backwards, one by one, keeping the current
    heading (see motion.move_to_world_point_without_turning).

    If `pts` and `obstacle_mask` of the current scan are passed, every leg is first checked
    on the fresh data: having passed there shortly before is a very good hint, not a
    guarantee, and it makes no sense to back into something that in the meantime is seen
    occupied. In that case the retreat stops there and says so.

    Returns:
        (segments_travelled: int, total_distance: float)
    """
    mobility_kwargs = mobility_kwargs or {}
    traveled = getattr(env, '_traveled_arcs', None)
    if not traveled:
        print("[RETREAT] No travelled leg recorded: I do not know where I came from, "
              "I do not back up.")
        return 0, 0.0

    done, total = 0, 0.0
    for _ in range(max_segments):
        if not traveled:
            print("[RETREAT] Recorded travelled legs exhausted.")
            break

        from_x, from_y, to_x, to_y = traveled[-1]
        robot_x, robot_y, _, _ = connection.getPosition(robot_state_client)
        seg_len = float(np.hypot(from_x - robot_x, from_y - robot_y))

        gap = float(np.hypot(to_x - robot_x, to_y - robot_y))
        if gap > P.RETREAT_CONTINUITY_M:
            print(f"[RETREAT] I am not where the last recorded leg ended (distance {gap:.2f} m, "
                  f"e.g. after a move with GraphNav): I do not back up blind.")
            del traveled[:]
            break
        if seg_len > P.RETREAT_MAX_SEGMENT_M:
            print(f"[RETREAT] The way-back leg is {seg_len:.2f} m long (beyond "
                  f"{P.RETREAT_MAX_SEGMENT_M:.1f} m): I do not retrace it backwards.")
            break

        if seg_len < P.RETREAT_MIN_SEGMENT_M:
            # Leg too short to be worth a command of its own: consume it and continue.
            traveled.pop()
            continue

        if pts is not None and obstacle_mask is not None:
            try:
                if safe_frontier.is_arc_in_fov(robot_x, robot_y, from_x, from_y, pts):
                    status = safe_frontier.verify_arc_safety(robot_x, robot_y, from_x, from_y,
                                                               pts, obstacle_mask)
                    if status == 'blocked':
                        print(f"[RETREAT] The way-back leg towards ({from_x:.2f}, {from_y:.2f}) "
                              f"is obstructed in the current scan. I stop here.")
                        break
            except Exception as e:
                print(f"[RETREAT] Check of the way-back leg failed ({e}); "
                      f"carrying on anyway, it is the way I came from.")

        print(f"[RETREAT] Leg {done + 1}/{max_segments}: going back towards "
              f"({from_x:.2f}, {from_y:.2f}), {seg_len:.2f} m backwards.")
        with TrajectoryLog(robot_state_client, 'retreat', (from_x, from_y)):
            success, dist = motion.move_to_world_point_without_turning(
                from_x, from_y, "vision", command_client, robot_state_client, **mobility_kwargs)
        total += dist
        traveled.pop()

        if not success:
            print(f"[RETREAT] The way back stopped after {dist:.2f} m. "
                  f"Interrupting the retreat.")
            break
        done += 1

    if done:
        print(f"[RETREAT] Completed: {done} legs, {total:.2f} m travelled backwards.")
    else:
        print("[RETREAT] No leg travelled.")
    return done, total


# =========================================================================
# BLOCKED OR DEFERRED CELLS
# =========================================================================


def cell_has_reachable_target(env, sampler, global_map, row, col, safety_margin=None):
    """
    Is there STILL a point in the cell where the robot could stand, according to the
    global map accumulated so far? Returns (yes_no, free, unknown, occupied).

    Used not to retry a cell whose goal is physically inside an obstacle (for example
    when the mission grid does not fit the space): finalize_or_defer_blocked_cell marks a
    cell blocked only after trying all its sides, but if the goal is in a wall the sides
    have nothing to do with it.

    Only what has been SEEN is considered: unknown is not occupied (the same rule as the
    frontier, which tells 'end_of_data' from 'obstacle'). So a cell never seen always answers
    yes, and this check cannot prevent a first exploration.
    """
    if global_map is None:
        return True, 0, 0, 0
    if safety_margin is None:
        safety_margin = geometry.PRM_EDGE_SAFETY_MARGIN_M
    center = env.get_world_position_from_cell(row, col)
    if center is None:
        return False, 0, 0, 0
    points = [center]
    try:
        points += list(sampler.get_point_in_cell(row, col))
    except Exception:
        pass
    free = unknown = occupied = 0
    for (px, py) in points:
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            occupied += 1
        elif global_map.is_known(px, py):
            free += 1
        else:
            unknown += 1
    return (free + unknown) > 0, free, unknown, occupied


def finalize_or_defer_blocked_cell(env, row, col, sampler=None, global_map=None):
    """
    Call right after env.mark_cell_side_explored(row, col, side_bit), when an attempt to
    enter (row, col) has just failed from one side.

    The cell is marked blocked forever (env.mark_cell_blocked -> value -1) only when EVERY
    side with a neighbour inside the grid has been tried and has failed
    (env.all_sides_explored). Otherwise it stays at the not-visited value (0) and is NOT
    retried right away: it stays a normal frontier candidate.
    get_adjacent_frontier_cells and get_lowest_rank_from_frontier_list treat every cell at
    0 the same way, ordered by its position in the serpentine, like a cell never tried.
    So a cell that failed once competes for priority with the SAME logic as all the
    others, and is retried when its turn comes, from the side not yet tried: neither right
    away out of turn, nor discarded after a single side.
    """
    # If there is no longer a point in the cell where the robot can stand, trying from
    # another side is useless -- the goal is inside an obstacle, not behind a gap that
    # opens elsewhere. See cell_has_reachable_target.
    if sampler is not None and global_map is not None:
        ok, free, unknown, occupied = cell_has_reachable_target(env, sampler, global_map, row, col)
        # Diagnostics: tells at once whether the cell is inside an obstacle or was never
        # seen, i.e. whether the mission grid does not fit the space.
        # CAREFUL how to read it: the candidate points per cell are dozens (lattice at
        # 0.5 m) or hundreds (at 0.25 m), so a high "occupied" does NOT mean the cell is unreachable;
        # it means much of the cell is inside an obstacle. The immediate block below
        # triggers only in the extreme case where NOT a single free or unknown point is left.
        print(f"[CELL] ({row},{col}) after the failed attempt: of the candidate points, "
              f"{free} seen free, {unknown} never seen, {occupied} inside an obstacle "
              f"in the global map.")
        if not ok:
            env.mark_cell_blocked(row, col)
            print(f"[BLOCKED] Cell ({row},{col}) blocked at once: all {occupied} candidate points "
                  f"are inside an obstacle in the global map (none free, "
                  f"none unknown). There is no side to enter from: the goal is in the wall.")
            return
    if env.all_sides_explored(row, col):
        env.mark_cell_blocked(row, col)
        print(f"[BLOCKED] Cell ({row},{col}) blocked forever: every reachable side "
              f"has been tried without success.")
    else:
        print(f"[DEFER] Cell ({row},{col}) not yet tried from all sides (sides tried: "
              f"{bin(env.get_cell_sides_status(row, col))}): it stays a candidate of the "
              f"frontier and will be retried later, in the normal order, "
              f"from a side not yet tried.")
