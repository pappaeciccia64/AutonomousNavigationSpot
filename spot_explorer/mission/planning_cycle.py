"""
The planning cycle: scan -> global maps -> edges of the graph -> path -> frontier.

One piece of code for the two ways the robot moves:
  - the navigation loop of today (CellAttempt, stop-and-go): it runs the steps itself, one
    turn at a time, with the robot standing (attempt_scan.py: _fresh_scan, _update_graph,
    _replan);
  - the cruise (continuous motion): CruisePlanner runs the whole cycle in the frontier
    thread (safe_frontier.ArcVerificationTracker.start_planning) at PLAN_PERIOD_S while the
    robot walks, and publishes a plan that the main thread only reads.

While the thread plans, it is the ONLY one touching graph and maps: the main thread sends
the motion commands and reads the plans, and takes graph and maps back only after
stop_planning() (which waits for the cycle running to end).
"""
import os
import time

import numpy as np
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b

from spot_explorer import config as P
from spot_explorer.mission import cells
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.planning import edge_evaluation
from spot_explorer.planning import safe_frontier
from spot_explorer.records import MISSION_STATS, mission_z0, save_npz_async
from spot_explorer.robot import motion

PLAN_PERIOD_S = 0.4            # one cycle every 0.4 s at most (2.5 Hz)
# Phases of a cycle, timed for the log: scan download, processing + global maps, edge
# refresh, Dijkstra, frontier + rotation checks, the rest (bundle, gait terrain).
PHASES = ('scan', 'maps', 'edges', 'path', 'frontier', 'other')
# The gait follows the terrain of the first metre of the confirmed stretch, not of all of it
# (up to 3 m): on 04-11 14:08 it was HARD 48% of the time and changed 239 times in 20 min.
GAIT_LOOKAHEAD_M = 1.0
PLAN_REPLAN_MARGIN = 0.10      # as in the stop-and-go: the new path must cost 10% less
CRUISE_BUNDLE_STEP_M = 0.5     # a scan bundle every this many metres walked in cruise
CRUISE_BUNDLE_REASON_S = 2.0   # and one when the frontier reason changes, at most this often


# =========================================================================================
# STEPS SHARED WITH THE NAVIGATION LOOP
# =========================================================================================
def scan_into_maps(local_grid, prm, global_map, global_terrain_map, scan, robot_xyyaw,
                   ground_label="scan-step"):
    """
    Process the scan, fold it into the global maps, local data into the PRM, snapshot for
    the frontier and free-space map. Fills scan (grid_origin_x/y, shape, grad_2d, ...).
    Returns (snapshot, time after the processing, time after the global maps).

    The snapshot must be created ONLY ONCE per scan: make_grid_snapshot also updates the
    temporal obstacle filter.
    """
    robot_x, robot_y, robot_yaw = robot_xyyaw
    scan.grid_origin_x, scan.grid_origin_y = local_grid.grid_origin(scan)
    scan.shape = (scan.num_y, scan.num_x)

    local_grid.process_scan(scan, robot_x, robot_y, robot_yaw, ground_label=ground_label)

    t_proc = time.perf_counter()
    body_core = global_maps.fold_scan_into_global_maps(
        global_map, global_terrain_map, scan.pts, scan.terrain, scan.obstacle_mask,
        scan.is_valid, scan.od, scan.shape, (robot_x, robot_y, robot_yaw))
    t_maps = time.perf_counter()

    scan.grad_2d = scan.grad.reshape(scan.shape)
    scan.rough_2d = scan.rough.reshape(scan.shape)
    scan.terrain_2d = scan.terrain.reshape(scan.shape)

    # The local data is always injected: it costs nothing (it stores the array references) and
    # it is needed anyway by sample_terrain_between_points for the gait of the leg about to
    # be travelled.
    prm.update_local_grid_data(
        grad_values_2d=scan.grad_2d,
        rough_values_2d=scan.rough_2d,
        terrain_values_2d=scan.terrain_2d,
        valid_2d=spot_grid.slope_usable_mask(scan.is_valid, scan.od, scan.shape),
        grid_origin_x=scan.grid_origin_x,
        grid_origin_y=scan.grid_origin_y,
        cell_size=scan.cell_size
    )

    # Snapshot of the scan just made: it is needed already here -- by the preliminary edge
    # discard in the refresh (see edge_evaluation.LIVE_PREFILTER_OD_M), by that of the nodes
    # in obstacles -- and then by the safe frontier.
    snap = safe_frontier.make_grid_snapshot(
        scan.od, scan.rough, scan.is_valid, scan.valid, scan.terrain,
        scan.num_x, scan.num_y, scan.grid_origin_x, scan.grid_origin_y, scan.cell_size,
        robot_x, robot_y, unwritten=scan.unwritten, robot_yaw=robot_yaw)

    # Free-space map: HERE, not together with global_map.update, because it needs the
    # filtered obstacle_distance that lives in the snapshot (see global_maps.update_clearance_map).
    # It is fed only by the planning cycle, so all observations go through the same filter.
    global_maps.update_clearance_map(global_map, scan.pts, snap, scan.is_valid, scan.unwritten,
                                     body_core, (robot_x, robot_y))
    return snap, t_proc, t_maps


def refresh_graph(prm, snap, global_map, robot_xyyaw, refresh, quiet=False):
    """
    Edges on the new data: live discard from the filtered snapshot, then (if refresh) the
    edge weights and slopes around the robot node, then the nodes inside obstacles out.
    Returns the nodes touched by the refresh (empty without refresh).

    quiet (cruise, a cycle every 0.4 s): no log lines and no mission statistics; the counts
    are left in prm.cycle_counts = (slopes recomputed, profile kept, untouched, edges out
    because of the live scan, nodes pruned) for the [CRUISE] line.
    """
    stats = {} if quiet else MISSION_STATS
    robot_x, robot_y, robot_yaw = robot_xyyaw
    # The preliminary edge discard reads the FILTERED obstacle_distance
    # (median over 3 scans + hysteresis), the same as the frontier.
    prm.set_live_snapshot(snap)
    if refresh:
        prm.needs_refresh = False
        touched_nodes = prm.refresh_local_edge_weights(
            global_map=global_map, edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
        stats['prm_refresh'] = stats.get('prm_refresh', 0) + 1
        n_new, n_reused, n_untouched = getattr(prm, 'refresh_counts_last', (0, 0, 0))
        if edge_evaluation.INCREMENTAL_REFRESH and not quiet:
            for k, v in (('refresh_slopes_new', n_new), ('refresh_slopes_reused', n_reused),
                         ('refresh_edges_untouched', n_untouched)):
                MISSION_STATS[k] = MISSION_STATS.get(k, 0) + v
            print(f"[REFRESH] Incremental: slopes recomputed on {n_new} edges (new or better "
                  f"seen), profile kept on {n_reused}, {n_untouched} edges untouched.")
        n_live = getattr(prm, 'live_veto_last', 0) + getattr(prm, 'live_prune_last', 0)
        stats['live_arc_vetoes'] = stats.get('live_arc_vetoes', 0) + n_live
        prm.cycle_counts = (n_new, n_reused, n_untouched, n_live, 0)
        if n_live and not quiet:
            print(f"[EDGES] {n_live} edges out of the graph because of the live scan "
                  f"({prm.live_veto_last} re-evaluated, {prm.live_prune_last} "
                  f"beyond the refresh radius), before the slope.")
    else:
        # No new data under the edges: the weights of the previous turn are kept and
        # there is no replan.
        touched_nodes = set()
        stats['prm_refresh_skipped'] = stats.get('prm_refresh_skipped', 0) + 1

    # Nodes INSIDE obstacles: out of the graph (see prune_prm_nodes_in_obstacles).
    pruned_nodes = safe_frontier.prune_prm_nodes_in_obstacles(prm, snap)
    if quiet:
        prm.cycle_counts = getattr(prm, 'cycle_counts', (0, 0, 0, 0, 0))[:4] + (len(pruned_nodes),)
    elif pruned_nodes:
        MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
        print(f"[NODES] {len(pruned_nodes)} nodes discarded: less than "
              f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m of free space where we see them "
              f"(first: {pruned_nodes[:8]}).")
    return touched_nodes


def scan_bundle_arrays(scan, snap, local_grid, robot_xyz, robot_yaw):
    """The layers of a processed scan for its bundle (scans/*.npz, see SAVE_SCAN_BUNDLES):
    grid, filtered snapshot, masks, geometry and robot pose. The caller adds the path, the
    frontier and the decision."""
    shape = scan.shape
    gf = local_grid.last_ground_filter
    return dict(
        terrain_raw=np.asarray(scan.terrain_raw, dtype=np.float32).reshape(shape),
        terrain_valid_raw=np.asarray(scan.valid, dtype=np.float32).reshape(shape),
        obstacle_distance=np.asarray(scan.od, dtype=np.float32).reshape(shape),
        # Spot's own, before low_obstacles (equal to obstacle_distance with the filter off).
        obstacle_distance_spot=np.asarray(getattr(scan, 'od_spot', scan.od), dtype=np.float32).reshape(shape),
        # The layer the frontier DECIDES on (median over the last readings + hysteresis,
        # see safe_frontier._ObstacleHistory) and the latch mask. Without these
        # two an offline replay cannot reproduce the blocks that come from the
        # thread's readings during the move. Same orientation as the others:
        # (num_y, num_x), row = y.
        obstacle_distance_filtered=np.asarray(snap.obstacle_dist, dtype=np.float32),
        blocked_latch=(np.zeros(shape, bool) if snap.blocked_latch is None
                       else np.asarray(snap.blocked_latch, dtype=bool)),
        terrain_corrected=np.asarray(scan.terrain, dtype=np.float32).reshape(shape),
        is_valid=np.asarray(scan.is_valid, dtype=bool).reshape(shape),
        roughness=np.asarray(scan.rough, dtype=np.float32).reshape(shape),
        gradient=np.asarray(scan.grad, dtype=np.float32).reshape(shape),
        obstacle_mask=np.asarray(scan.obstacle_mask, dtype=np.int8).reshape(shape),
        footprint_mask=(np.zeros(shape, bool) if scan.footprint_mask is None
                        else np.asarray(scan.footprint_mask, dtype=bool).reshape(shape)),
        grid_origin=np.array([scan.grid_origin_x, scan.grid_origin_y]), cell_size=scan.cell_size,
        robot_xyz=np.array(robot_xyz), robot_yaw=robot_yaw,
        ground_z=np.nan if gf.get('ground_z') is None else gf['ground_z'],
        ground_n_above=gf.get('n_above', 0),
        mission_z0=mission_z0(),
        terrain_raw_zero=np.nan if spot_grid.raw_zero(scan.grids_data) is None else spot_grid.raw_zero(scan.grids_data),
        terrain_scale=np.nan if not spot_grid.raw_scale(scan.grids_data) else spot_grid.raw_scale(scan.grids_data),
        unwritten=np.asarray(scan.unwritten, dtype=bool).reshape(shape))


def _edge_in_graph(prm, a, b):
    return any(n == b for n, _w in prm.edges.get(a, []))


# =========================================================================================
# CRUISE: the whole cycle, in the frontier thread
# =========================================================================================
class CruisePlanner:
    """
    One call = one planning cycle while the robot walks along path_ids towards goal_id:

      1. fresh scan, robot pose from the SAME snapshot of transforms as the grid;
      2. global maps, local data into the PRM, snapshot, free-space map (scan_into_maps);
      3. where the robot is along the path: the edge it is walking (prev -> next) is
         committed, the next node becomes the robot node (centre of the refresh);
      4. edges refreshed at every cycle (the window moves with the robot), nodes in
         obstacles out (refresh_graph);
      5. Dijkstra from the next node to the goal, with the stability margin of the
         stop-and-go: the path changes only if the new one costs 10% less;
      6. safe frontier on [robot] + path and the stretch confirmed free (allowed_advance).

    Returns the plan (dict):
      status      'ok'; 'edge_blocked' (the edge being walked is out of the graph now);
                  'no_path' (no path from the next node: the previous one is kept);
                  'no_scan'; 'error'
      robot       (x, y, yaw) of the scan
      ids         path node ids from the next node to the goal
      polyline    [(x, y) robot] + positions of ids
      frontier    compute_safe_frontier on polyline (decision threshold, as stop-and-go, but
                  without the exit rule from a tight space: see allow_escape)
      advance     metres of polyline confirmed free for the robot centre (allowed_advance),
                  shortened to the first corner where the rotation on the spot is not clear
      confirmed   polyline cut at advance: the points to give to motion.PathCommand
      turn_stop   node where the confirmed stretch stops because the rotation there is not
                  clear (None otherwise): the stop-and-go takes over there
      terrain     (slope, roughness): the worst over the edges of the confirmed stretch, for
                  the gait (motion.select_mobility_params)
      to_goal     True if the confirmed stretch reaches the goal
      edges       prm.cycle_counts of the refresh (see refresh_graph)
      replanned   True if this cycle changed the path
      cycle_s     duration of the cycle
    """

    def __init__(self, local_grid, global_grid, prm, robot_state_client, goal_id, path_ids):
        self.local_grid = local_grid
        self.global_map = global_grid.global_occupancy_map
        self.global_terrain_map = global_grid.global_terrain_map
        self.prm = prm
        self.robot_state_client = robot_state_client
        self.goal_id = goal_id
        self.path_ids = list(path_ids)     # from the node the robot starts from
        # Stretches of the starting path that are not graph edges (shortcut of the
        # stop-and-go, see decisions.shortcut_index): not 'edge_blocked' for that.
        self._not_edges = {(a, b) for a, b in zip(self.path_ids, self.path_ids[1:])
                           if not _edge_in_graph(prm, a, b)}
        self._seg = 0                      # edge of path_ids being walked
        self.cycles = 0
        self.replans = 0
        self.last_scan = None
        self.last_snap = None
        # Scan bundles for the review (mission_viewer): dict(folder, iteration, cell,
        # loop_iter) set by the cruise, None = none. One every CRUISE_BUNDLE_STEP_M.
        self.bundle = None
        self._bundle_xy = None
        self._bundle_why = None          # (frontier reason, time) of the last bundle saved
        self.bundle_seq = 0              # added to the cycle number in the bundle names
        # (env, row, col) of the target cell: every plan says how much of it the global map
        # has seen ('cell_seen', see cells.cell_seen_fraction). None = not computed.
        self.seen_cell = None
        # Times of the cycle, for the log: seconds per phase (PHASES) of every cycle, and the
        # interval between the acquisition times of two consecutive Spot grids (0 = the same
        # grid read again: the cycle was faster than Spot's update).
        self.phase_s = {k: [] for k in PHASES}
        self.grid_dt = []
        self._grid_t = None

    def _grid_time(self, scan):
        try:
            t = scan.proto.local_grid.acquisition_time
            t = t.seconds + t.nanos * 1e-9
        except Exception:
            return None
        dt = None if self._grid_t is None else t - self._grid_t
        self._grid_t = t
        if dt is not None:
            self.grid_dt.append(dt)
        return dt

    def __call__(self):
        t0 = time.perf_counter()
        ph = {}
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            return dict(status='no_scan', cycle_s=time.perf_counter() - t0)
        t = time.perf_counter(); ph['scan'] = t - t0
        grid_dt = self._grid_time(scan)
        robot = _pose_of_scan(scan)
        snap, _, _ = scan_into_maps(self.local_grid, self.prm, self.global_map, self.global_terrain_map,
                                    scan, robot, ground_label=None)
        self.last_scan, self.last_snap = scan, snap
        t1 = time.perf_counter(); ph['maps'] = t1 - t; t = t1
        self.cycles += 1

        # Where the robot is along the path: the edge it is walking stays (committed), the
        # plan changes from the next node on.
        coords = [self.prm.get_node_position(n) for n in self.path_ids]
        _, self._seg = motion.path_ahead(coords, robot[0], robot[1], from_seg=self._seg)
        k = min(self._seg, len(self.path_ids) - 2) if len(self.path_ids) > 1 else 0
        prev_id = self.path_ids[k]
        next_id = self.path_ids[k + 1] if len(self.path_ids) > 1 else self.path_ids[0]
        self.prm.set_robot_node(next_id, 0.0)

        refresh_graph(self.prm, snap, self.global_map, robot, refresh=True, quiet=True)
        t1 = time.perf_counter(); ph['edges'] = t1 - t; t = t1

        status, replanned = 'ok', False
        if (len(self.path_ids) > 1 and (prev_id, next_id) not in self._not_edges
                and not _edge_in_graph(self.prm, prev_id, next_id)):
            status = 'edge_blocked'
        remainder = self.path_ids[k + 1:]
        if next_id != self.goal_id:
            new_ids = self.prm.find_path_dijkstra(next_id, self.goal_id, current_path_ids=remainder,
                                                  margin=PLAN_REPLAN_MARGIN)
            if new_ids is None:
                status = 'no_path' if status == 'ok' else status
            elif new_ids != remainder:
                self.path_ids = self.path_ids[:k + 1] + list(new_ids)
                remainder = list(new_ids)
                replanned = True
                self.replans += 1
                MISSION_STATS['replan_count'] += 1
                print(f"[CRUISE] Replan from node {next_id}: {len(remainder)} nodes to the goal "
                      f"(cycle {self.cycles}).")

        t1 = time.perf_counter(); ph['path'] = t1 - t; t = t1
        polyline = [(robot[0], robot[1])] + [self.prm.get_node_position(n) for n in remainder]
        frontier = safe_frontier.compute_safe_frontier(polyline, snap, allow_escape=False)
        advance = safe_frontier.allowed_advance(frontier)
        advance, turn_stop = self._turns_within(polyline, remainder, advance, snap)
        confirmed = _cut_polyline(polyline, advance)
        t1 = time.perf_counter(); ph['frontier'] = t1 - t; t = t1
        path_len = float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(polyline, polyline[1:])))
        self._save_bundle(scan, snap, robot, polyline, remainder, next_id, frontier, advance, confirmed)
        terrain = self._terrain(confirmed, remainder)
        cell_seen = (cells.cell_seen_fraction(self.seen_cell[0], self.global_map, *self.seen_cell[1:])
                     if self.seen_cell is not None and self.global_map is not None else None)
        ph['other'] = time.perf_counter() - t
        for k, v in ph.items():
            self.phase_s[k].append(v)
        return dict(status=status, robot=robot, ids=list(remainder), polyline=polyline,
                    frontier=frontier, advance=advance, confirmed=confirmed, replanned=replanned,
                    turn_stop=turn_stop, terrain=terrain,
                    to_goal=(turn_stop is None and advance >= path_len - 0.01),
                    edges=getattr(self.prm, 'cycle_counts', None), cell_seen=cell_seen,
                    phases=ph, grid_dt=grid_dt, cycle_s=time.perf_counter() - t0)

    def timing_summary(self):
        """'scan 0.09 maps 0.04 ... | Spot grid every 0.20 s, N read twice' on the medians, or ''."""
        if not self.phase_s['scan']:
            return ''
        txt = ' '.join(f"{k} {np.median(v):.2f}" for k, v in self.phase_s.items() if v)
        if self.grid_dt:
            dt = np.array(self.grid_dt)
            new = dt[dt > 1e-6]
            txt += (f" | Spot grid every {np.median(new):.2f} s" if new.size else " | Spot grid never new") \
                + f", {int(np.sum(dt <= 1e-6))} read twice out of {dt.size}"
        return txt

    def _save_bundle(self, scan, snap, robot, polyline, ids, next_id, frontier, advance, confirmed):
        """A bundle like those of the stop-and-go (decision 'cruise', target = end of the
        confirmed stretch), every CRUISE_BUNDLE_STEP_M walked, and when the reason of the
        frontier changes (at most one every CRUISE_BUNDLE_REASON_S): turning on the spot or held
        by the frontier the robot does not walk, and those were the scans missing in the
        viewer (04-11 14:08, cell (0,2): 15 s of 'slope' stops, no bundle). Never raises."""
        if self.bundle is None or not P.SAVE_SCAN_BUNDLES:
            return
        now = time.time()
        walked = (self._bundle_xy is None or np.hypot(robot[0] - self._bundle_xy[0],
                                                      robot[1] - self._bundle_xy[1]) >= CRUISE_BUNDLE_STEP_M)
        new_reason = (self._bundle_why is not None and frontier['reason'] != self._bundle_why[0]
                      and now - self._bundle_why[1] >= CRUISE_BUNDLE_REASON_S)
        if not (walked or new_reason):
            return
        try:
            b = self.bundle
            tf = get_a_tform_b(scan.proto.local_grid.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
            goal = self.prm.get_node_position(self.goal_id)
            save_npz_async(
                os.path.join(b['folder'], "scans",
                             f"scan_it{b['iteration']:03d}_cell{b['cell'][0]}_{b['cell'][1]}_"
                             f"{b['loop_iter']:03d}_cruise{self.bundle_seq + self.cycles:03d}.npz"),
                time=time.time(), iteration=b['iteration'], cell=np.array(b['cell']), loop_iter=b['loop_iter'],
                **scan_bundle_arrays(scan, snap, self.local_grid, (robot[0], robot[1], tf.position.z), robot[2]),
                polyline=np.array(polyline), path_node_ids=np.array(ids), robot_node_id=next_id,
                goal_xy=np.array(goal if goal is not None else (np.nan, np.nan)),
                frontier_dist=frontier['dist'], frontier_reason=frontier['reason'],
                frontier_stop_xy=np.array(frontier['stop_xy']),
                frontier_value=np.nan if frontier['value'] is None else frontier['value'],
                frontier_od_robot=frontier['od_robot'], frontier_half_along=frontier['half_along'],
                frontier_clearance=frontier['clearance_nominal'], frontier_path_len=frontier['path_len'],
                allowed_advance=advance, keep_heading=False, no_progress=0,
                decision_kind='cruise', decision_target=np.array(confirmed[-1], dtype=np.float64))
            self._bundle_xy = (robot[0], robot[1])
            self._bundle_why = (frontier['reason'], now)
            MISSION_STATS['scan_bundles'] += 1
        except Exception as e:
            print(f"[CRUISE] Scan bundle not saved: {e}")

    @staticmethod
    def _turns_within(polyline, ids, advance, snap):
        """
        The corners inside the confirmed stretch sharper than motion.PATH_TURN_IN_PLACE_DEG are
        a rotation on the spot at the node: the swept area is checked as the stop-and-go does
        (geometry.rotation_is_clear). The confirmed stretch stops at the first corner that
        is not clear. Returns (advance, node of that corner or None).
        """
        s = 0.0
        for i in range(1, len(polyline) - 1):
            s += float(np.hypot(polyline[i][0] - polyline[i - 1][0], polyline[i][1] - polyline[i - 1][1]))
            if s > advance:
                break
            h_in = np.arctan2(polyline[i][1] - polyline[i - 1][1], polyline[i][0] - polyline[i - 1][0])
            h_out = np.arctan2(polyline[i + 1][1] - polyline[i][1], polyline[i + 1][0] - polyline[i][0])
            dyaw = float(np.arctan2(np.sin(h_out - h_in), np.cos(h_out - h_in)))
            if abs(np.degrees(dyaw)) <= motion.PATH_TURN_IN_PLACE_DEG:
                continue
            clear, _margin = geometry.rotation_is_clear(snap.obstacle_dist, snap.origin_x, snap.origin_y,
                                                        snap.cell_size, polyline[i][0], polyline[i][1],
                                                        float(h_in), dyaw)
            if not clear:
                return s, ids[i - 1]
        return advance, None

    def _terrain(self, confirmed, ids):
        """Worst slope and roughness over the first GAIT_LOOKAHEAD_M of the confirmed polyline
        (the robot walks less than that before the next plan)."""
        grad = rough = 0.0
        full = [tuple(map(float, p)) for p in confirmed]
        near = [tuple(map(float, p)) for p in _cut_polyline(full, GAIT_LOOKAHEAD_M)]
        for k in range(min(len(near), len(full)) - 1):
            (x1, y1), (x2, y2) = near[k], near[k + 1]
            if np.hypot(x2 - x1, y2 - y1) < 0.05:
                continue
            n1 = ids[k - 1] if 1 <= k <= len(ids) else None
            # A stretch cut short does not end on its node (the node pair would read the
            # global map cache of the whole edge).
            n2 = ids[k] if k < len(ids) and near[k + 1] == full[k + 1] else None
            try:
                lo, la, ro = self.prm.sample_terrain_between_points(x1, y1, x2, y2, node_idx1=n1, node_idx2=n2)
            except Exception:
                continue
            grad, rough = max(grad, lo, la), max(rough, ro)
        return grad, rough


def _pose_of_scan(scan):
    """(x, y, yaw) of the body in VISION at the instant of the grid."""
    tf = get_a_tform_b(scan.proto.local_grid.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
    q = tf.rotation
    yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
    return float(tf.position.x), float(tf.position.y), yaw


def _cut_polyline(polyline, s):
    """The polyline from its start up to distance s along it."""
    if len(polyline) < 2:
        return list(polyline)
    end, k = safe_frontier.point_along(polyline, s)
    return list(polyline[:k + 1]) + [end]
