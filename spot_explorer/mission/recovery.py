"""Getting out of a block: going back with GraphNav, walked retreat, blocked or deferred cells."""
import numpy as np

from spot_explorer.graphnav import navigation
from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import safe_frontier

from spot_explorer.planning import roadmap as prm_graph

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
        print(f"[RITORNO] {label}: dai nodi delle prossime {max_hops} briciole il grafo non "
              f"arriva comunque all'obiettivo (non dipende da dove sono io). Non torno "
              f"indietro: rimando subito la cella.")
        MISSION_STATS['backtracks_skipped'] = MISSION_STATS.get('backtracks_skipped', 0) + 1
        return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}

    for hop in range(1, int(max_hops) + 1):
        # Arrive facing the attempt's goal: from there replan towards it.
        ok, crumb = navigation.graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
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
        prm_graph.needs_refresh = True       # the robot has moved: see EVENT REFRESH
        # The scan of the preliminary discard was taken from another point (and the
        # body exclusion would be where the robot was): drop it, until the next scan.
        prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
        try:
            ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
        except Exception:
            ax, ay = crumb['x'], crumb['y']
        new_node = prm_graph.add_stop_node(
            ax, ay, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"sosta (ritorno GraphNav a {crumb['wp_name']})")
        # No global-map exemption around this node: see the comment at the
        # caller of set_robot_node in the main loop.
        prm_graph.set_robot_node(new_node, 0.0)

        ids = prm_graph.find_path_dijkstra(new_node, goal_id)
        if ids is not None and len(ids) > 1:
            print(f"[RITORNO] {label}: tornato a {crumb['wp_name']} ({hop}/{max_hops}) e da qui "
                  f"c'e' un percorso con {len(ids) - 1} tratti. Riprendo.")
            return {'ok': True, 'ids': ids, 'robot_node': new_node, 'how': 'ripianificato'}
        print(f"[RITORNO] {label}: tornato a {crumb['wp_name']} ({hop}/{max_hops}) ma da qui il "
              f"grafo non offre percorsi verso l'obiettivo.")
        if crumb.get('in_place'):
            break           # there are no farther breadcrumbs: more rounds would be the same

    print(f"[RITORNO] {label}: {hop} ritorni senza trovare un percorso. Rimando la cella.")
    return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}



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
            # Leg too short to be worth a command of its own: consume it and continue.
            traveled.pop()
            continue

        if pts is not None and obstacle_mask is not None:
            try:
                if safe_frontier.is_arc_in_fov(robot_x, robot_y, from_x, from_y, pts):
                    status = safe_frontier.verify_arc_safety(robot_x, robot_y, from_x, from_y,
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
        with TrajectoryLog(robot_state_client, 'retreat', (from_x, from_y)):
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
        print(f"[CELLA] ({row},{col}) dopo il tentativo fallito: dei punti candidati, "
              f"{free} visti liberi, {unknown} mai visti, {occupied} dentro un ostacolo "
              f"nella mappa globale.")
        if not ok:
            env.mark_cell_blocked(row, col)
            print(f"[BLOCKED] Cella ({row},{col}) bloccata subito: tutti i {occupied} punti "
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
