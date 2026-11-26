"""Getting away after a fall: the walked retreat along the way the robot came; reachable targets of a cell."""
import numpy as np

from spot_explorer.robot import motion
from spot_explorer.perception import geometry
from spot_explorer.robot import connection
from spot_explorer.planning import safe_frontier


from spot_explorer import config as P
from spot_explorer.records import TrajectoryLog



# =========================================================================
# Going back with graphnav, when from where the robot is the graph does not lead
# to the cell
# =========================================================================


def retreat_along_traveled_path(env, robot_state_client, command_client,
                                max_segments=P.RETREAT_MAX_SEGMENTS,
                                mobility_kwargs=None,
                                pts=None, obstacle_mask=None):
    """
    Retreat: go back over its own steps walking BACKWARDS, without ever turning. After a
    fall (cell_attempt).

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
            if safe_frontier.is_arc_in_fov(robot_x, robot_y, from_x, from_y, pts):
                status = safe_frontier.verify_arc_safety(robot_x, robot_y, from_x, from_y,
                                                           pts, obstacle_mask)
                if status == 'blocked':
                    print(f"[RETREAT] The way-back leg towards ({from_x:.2f}, {from_y:.2f}) "
                          f"is obstructed in the current scan. I stop here.")
                    break

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
# Blocked or deferred cells
# =========================================================================


def cell_has_reachable_target(env, sampler, global_map, row, col, safety_margin=None):
    """
    Is there still a point in the cell where the robot could stand, according to the
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
    points = [center] + list(sampler.get_point_in_cell(row, col))
    free = unknown = occupied = 0
    for (px, py) in points:
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            occupied += 1
        elif global_map.is_known(px, py):
            free += 1
        else:
            unknown += 1
    return (free + unknown) > 0, free, unknown, occupied

