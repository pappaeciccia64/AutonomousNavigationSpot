"""Choice of the goal point in a cell (find_best_point_in_cell) and excluded goals."""
import numpy as np

from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.mission import cells


from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS



# =========================================================================
# CHOICE OF THE GOAL POINT in a cell
# =========================================================================


def _refused_goals(env, row, col):
    """The cell goals refused by the frontier in this mission (list of (x, y))."""
    store = getattr(env, '_refused_goals', None)
    if store is None:
        store = env._refused_goals = {}
    return store.setdefault((int(row), int(col)), [])


def _refuse_goal(env, row, col, x, y, why):
    """
    Exclude the goal (x, y) of the cell until the end of the mission. On 22-10 16:04
    node 548 of cell (7,4) was chosen again at every attempt and blocked three times at the
    same point: the global map gave it free at 0.28 m, the live frontier at 0.24.
    """
    lst = _refused_goals(env, row, col)
    if any(np.hypot(x - ex, y - ey) < P.RECOVERY_SAME_GOAL_M for ex, ey in lst):
        return
    lst.append((float(x), float(y)))
    MISSION_STATS['goals_refused'] = MISSION_STATS.get('goals_refused', 0) + 1
    print(f"[TARGET] Goal ({x:.2f}, {y:.2f}) of cell ({row},{col}) excluded for the "
          f"rest of the mission: {why}.")


def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist,
                            sampler, global_map=None, safety_margin=None, exclude=None,
                            reachable=None):
    """
    Choose the goal point inside the cell among the geometric centre and the graph nodes
    in the cell.

    Always discarded (never chosen): the points in `exclude` (already tried in this attempt),
    those occupied in the global map (with safety_margin) and those closer than
    geometry.PLAN_CLEARANCE_M (0.28) to an obstacle in the live scan or in the learned
    free space. Among the rest, in order:
      1. not refused by the frontier in this mission (_refuse_goal); refused ones come
         back only if there is nothing else;
      2. REACHABLE from the robot in the graph (reachable(x, y) -> bool, tried in the order
         of the following criteria until one answers yes). Falling back on the next ones
         nearest to the centre when the graph does not get there is not enough: they lie in
         the same isolated area (on 22-10 16:04 that fallback failed 9 times out of 10). If
         no point is reachable the best one is chosen anyway, and the attempt fails;
      3. with at least TARGET_CLEARANCE_M (0.33) of free space;
      4. the geometric centre, whether free or unknown (CENTRE RULE: a point already seen
         free is usually the last strip scanned from the previous cell, i.e. on the
         border);
      5. seen free before never seen: if the centre is discarded the area is known, and
         going back to look at an unknown spot next to the obstacle would be a wasted trip;
      6. at least TARGET_CELL_BORDER_M from the cell border;
      7. nearest to the centre (in steps of 0.25 m), then nearest to the robot: on a tie,
         the one on its side of the obstacle.

    If no point is left the cell has no valid goal and the function returns
    (None, None, ...): the caller already treats it as an entry failure and moves on to
    the logic of the sides not yet tried.

    global_map: accumulated global_maps.GlobalGrid, updated with the current scan right
    before this call. If None the centre is used without checks.

    Returns:
        (target_x, target_y, valid_samples, rejected_samples)
        The two lists feed the visualization (yellow = accepted, red = discarded).
    """
    if safety_margin is None:
        safety_margin = geometry.PRM_EDGE_SAFETY_MARGIN_M
    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center
    sampled = list(sampler.get_point_in_cell(cell_row, cell_col))

    if global_map is None:
        # No occupancy information: the centre of the cell.
        return cell_center_x, cell_center_y, sampled, []

    # min_clear is the distance edges are planned with (geometry.PLAN_CLEARANCE_M):
    # a goal closer to obstacles would have no edges. The global map declares a cell
    # occupied only after two observations, so an obstacle just seen is there only
    # in the live scan (22-10 15:50, cell (0,2)).
    min_clear = geometry.PLAN_CLEARANCE_M
    pref_margin = max(safety_margin, P.TARGET_CLEARANCE_M - geometry.OBSTACLE_THRESHOLD)
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
        """Live obstacle_distance at the point, or None if outside the scan."""
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
            center_reason = center_reason or ("already discarded live" if is_center else None)
            continue
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            blocked.append(p)
            center_reason = center_reason or ("occupied in the map" if is_center else None)
            continue
        c_live = _live_clearance(px, py)
        # For the centre only the live scan counts: the learned free space, which also
        # collects readings from far away, would move the centre more than it should.
        c_mem = None if is_center else (global_map.clearance(px, py)
                                        if hasattr(global_map, 'clearance') else None)
        known_c = [c for c in (c_live, c_mem) if c is not None]
        if known_c and min(known_c) < min_clear:
            blocked.append(p)
            if is_center:
                center_reason = f"{min(known_c):.2f} m from an obstacle in the live scan"
            continue
        tight = (global_map.is_occupied(px, py, safety_margin=pref_margin)
                 or (bool(known_c) and min(known_c) < P.TARGET_CLEARANCE_M))
        border = cells.cell_border_distance(env, cell_row, cell_col, px, py)
        rows.append(dict(
            p=(float(px), float(py)), center=is_center, refused=_near(p, refused),
            seen=(c_live is not None or global_map.is_known(px, py)), tight=tight,
            clear=(min(known_c) if known_c else None),
            edge=(border is not None and border < P.TARGET_CELL_BORDER_M),
            dc=float(np.hypot(px - cell_center_x, py - cell_center_y)),
            dr=float(np.hypot(px - robot_x, py - robot_y))))

    valid = [r['p'] for r in rows]
    if not rows:
        print(f"[TARGET] Cell ({cell_row},{cell_col}): centre {center_reason or 'discarded'} and "
              f"no point of the cell at least {min_clear:.2f} m from obstacles. "
              f"No valid goal.")
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
            reach_note = "; NO point of the cell is reachable from the robot in the graph"
            MISSION_STATS['goals_unreachable'] = MISSION_STATS.get('goals_unreachable', 0) + 1
    if not fresh:
        reach_note += "; all points had already been refused by the frontier, retrying the best one"

    bx, by = chosen['p']
    flags = []
    if chosen['tight']:
        flags.append(f"less than {P.TARGET_CLEARANCE_M:.2f} m of free space")
    if chosen['edge']:
        flags.append(f"less than {P.TARGET_CELL_BORDER_M:.2f} m from the cell border")
    flag_txt = f" -- {', '.join(flags)}" if flags else ""
    if chosen['center']:
        state = "libero" if chosen['seen'] else "unknown"
        print(f"[TARGET] Cell ({cell_row},{cell_col}): using the geometric centre "
              f"({bx:.2f}, {by:.2f}), {state}{flag_txt}{reach_note}.")
        return bx, by, valid, blocked

    if center_reason is None:
        crow = next((r for r in rows if r['center']), None)
        if crow is None:
            center_reason = "discarded"
        elif crow['refused']:
            center_reason = "refused by the frontier in this mission"
        elif crow['tight'] and not chosen['tight']:
            center_reason = (f"with only {crow['clear']:.2f} m of free space"
                             if crow['clear'] is not None else "too close to an obstacle in the map")
        else:
            center_reason = "not reachable from the robot in the graph"
    n_seen = sum(1 for r in rows if r['seen'])
    print(f"[TARGET] Cell ({cell_row},{cell_col}): centre {center_reason}, using the "
          f"{'free' if chosen['seen'] else 'unknown'} point ({bx:.2f}, {by:.2f}), {chosen['dc']:.2f} m "
          f"from the centre{flag_txt}{reach_note}  [free={n_seen} unknown={len(rows) - n_seen} "
          f"discarded={len(blocked)} tight={sum(r['tight'] for r in rows)} "
          f"on_border={sum(r['edge'] for r in rows)} refused={sum(r['refused'] for r in rows)}]")
    return bx, by, valid, blocked
