"""
Edge evaluation of the mission PRM (see roadmap.py): occupancy (live scan and global map),
slope with the body (live scan, global height map, node fallback), weight; the refresh
around the robot during the move. The vetoes are counted in edge_stats.py.
"""

import numpy as np
from typing import Tuple, Optional
from spot_explorer.perception import geometry
from spot_explorer.perception import slopes


# Samples per metre with which occupancy is checked along an edge (minimum 5), the same
# everywhere. Every sample checks a disk of radius geometry.PRM_EDGE_SAFETY_MARGIN_M
# (0.13): with a sample every 10 cm, halfway between two samples the disk still covers 0.12 m,
# i.e. the edge line stays ~0.27 m from obstacles (geometry.PLAN_CLEARANCE_M = 0.28;
# the frontier requires 0.25).
OCCUPANCY_SAMPLES_PER_M = 10
OCCUPANCY_SAMPLES_MIN = 5

# --- PRELIMINARY DISCARD ON THE LIVE SCAN -----------------------------------------------
# The global map declares a cell occupied only after 2 confirmations, so at the first
# scan of a place it knows nothing yet. Before anything else, an edge is discarded if
# one of its samples (every 10 cm) falls on a cell SEEN in the current scan with
# obstacle_distance below LIVE_PREFILTER_OD_M and not under the robot's body.
#   - The threshold is the frontier distance (geometry.FRONTIER_CLEARANCE_M, 0.25 m): it removes
#     the edges the frontier would refuse. With a threshold below the frontier (0.20 m) the
#     planner chose edges at 0.20-0.25 m that the frontier then refused.
#   - NOT the planning distance (0.28, that of the global map): from a tight
#     spot the exit edges necessarily pass near obstacles, and the frontier
#     allows them (in the first metre it is enough not to get closer, see FRONTIER_ESCAPE_DIST_M). At 0.28
#     the replay of mission 13-10 11:22 lost the path from the last cell start.
#   - It is a vectorized read, costs very little, and comes before the slope.
#   - It is not permanent: it is redone at every refresh with the new scan.
# None = disabled.
LIVE_PREFILTER_OD_M = geometry.FRONTIER_CLEARANCE_M     # = 0.25

# Radius within which refresh_local_edge_weights re-evaluates the edges during the move. The
# robot travels a few nearby edges in the next step; outside here the edges keep the
# weights of the last evaluation (safety stays with the frontier, on the live data). 1.25 m is
# the maximum edge length: the edge the robot is about to travel always falls
# entirely in the re-evaluated area. None = no limit.
REFRESH_RADIUS_M = 1.25

# Slopes of the refresh pairs computed in batch with matrices (see
# precompute_arc_slopes), as at cell start (mission_graph.BATCH_SLOPES).
# False = edge by edge; the result is identical.
REFRESH_BATCH_SLOPES = True


def body_slope(prof):
    """
    (long, lat, has) to use for veto, cost and gait, given the result of
    compute_arc_slope_profile: with 6 values (body=True) those with the body (the worse of
    edge and edge extended by half a body, see slopes._body_slope); with 3 values those of
    the edge. If the edge has no data of its own the edge values remain: the extension does
    not turn an edge never seen into a measured one.
    """
    if prof[2] and len(prof) == 6:
        return prof[3], prof[4], True
    return prof[0], prof[1], prof[2]


# LOCAL GRAPH UPDATE (see mission_graph.MissionGraph.add_edges_to_prm). At cell
# start only the edges near the areas with new data since the last evaluation are re-evaluated
# (dirty_rects: the scan windows, the only place where the maps change).
# DIRTY_MARGIN_M is how far the evaluation of an edge reads data beyond its rectangle:
# half a body beyond the ends, lateral slope slice, occupancy margin, plus
# a decimetre of safety. Reducing it breaks the equality with the full build
# (test_replay.py checks it).
DIRTY_MARGIN_M = (slopes.ARC_BODY_EXTENSION_M + slopes.LATERAL_SLICE_HALF_WIDTH_M
                  + geometry.PRM_EDGE_SAFETY_MARGIN_M + 0.10)


class EdgeEvaluation:
    """Mixin of roadmap.PRM: occupancy, slope and weight of the edges."""

    def _init_edge_evaluation(self):
        # Live local grid of the current scan (see update_local_grid_data)
        self.current_grad_2d = None
        self.current_rough_2d = None
        self.current_terrain_2d = None  # corrected heights of the scan, for the edge slope
        self.current_valid_2d = None    # reliable cells: filled heights do not enter the slope
        self.grid_origin_x = None
        self.grid_origin_y = None
        self.cell_size = None
        # Live scan for the preliminary discard (see LIVE_PREFILTER_OD_M and
        # set_live_obstacles). None = no scan: no discard.
        self.live_obstacles = None
        self.live_veto_last = 0       # edges discarded by the scan at the last call
        self.live_prune_last = 0      # valid edges outside the radius removed at the last refresh

        # Global height map (global_maps.GlobalTerrainGrid): slope of the edges outside
        # the current window but already seen before (see set_global_terrain_map).
        self.global_terrain_map = None
        # Cache of the slope from the global map, per edge: an entry stays valid as long as the
        # map does not change near the edge (see _sync_global_slope_cache).
        self._gslope_cache = {}
        self._gslope_map_id = None
        self._gslope_seen_updates = 0

        # Areas (xmin, xmax, ymin, ymax) with new data since the last edge evaluation:
        # see DIRTY_MARGIN_M and mission_graph.MissionGraph.add_edges_to_prm(incremental=True).
        self.dirty_rects = []
        self.hazards_seen = 0         # hazards of the global map already applied (mission_graph)

    def set_global_terrain_map(self, global_terrain_map):
        """
        Inject the persistent global height map (global_maps.GlobalTerrainGrid).
        To be called once the object exists (typically right after creating or
        retrieving it in the mission code) -- it can be called more than once, it is
        idempotent, it only updates the reference.
        """
        self.global_terrain_map = global_terrain_map

    def update_local_grid_data(self, grad_values_2d, rough_values_2d, grid_origin_x, grid_origin_y, cell_size,
                               terrain_values_2d=None, valid_2d=None):
        """
        Temporarily store the current local map of gradients, roughness and (optionally)
        raw terrain heights (already corrected, see correct_terrain) to allow accurate
        sampling of the edges -- including the dense per-edge slope profile computed
        directly from the real heights.
        """
        self.current_grad_2d = grad_values_2d
        self.current_rough_2d = rough_values_2d
        self.current_terrain_2d = terrain_values_2d
        # reliable cells (is_valid of correct_terrain): filled heights do not enter
        # the slope profile -- see slopes.compute_arc_slope_profile
        self.current_valid_2d = valid_2d
        self.grid_origin_x = grid_origin_x
        self.grid_origin_y = grid_origin_y
        self.cell_size = cell_size
        self.dirty_rects.extend(self.live_rects())

    def live_rects(self):
        """Rectangles covered by the current live data (local grid and obstacle scan)."""
        out = []
        if self.current_grad_2d is not None and self.grid_origin_x is not None and self.cell_size:
            nr, nc = np.shape(self.current_grad_2d)[:2]
            out.append((float(self.grid_origin_x), float(self.grid_origin_x + nc * self.cell_size),
                        float(self.grid_origin_y), float(self.grid_origin_y + nr * self.cell_size)))
        lo = self.live_obstacles
        if lo is not None:
            nr, nc = lo['od'].shape[:2]
            out.append((lo['ox'], lo['ox'] + nc * lo['cs'], lo['oy'], lo['oy'] + nr * lo['cs']))
        return out

    def set_live_obstacles(self, obstacle_dist_2d, seen_2d, origin_x, origin_y, cell_size,
                           robot_x, robot_y, robot_yaw=None):
        """
        The scan just made, for the preliminary edge discard (see
        LIVE_PREFILTER_OD_M). 2D arrays (num_y, num_x), row = y. obstacle_dist_2d: in the
        navigation loop the FILTERED one from the frontier snapshot (median + hysteresis); at
        the start of the attempt the raw one. seen_2d: cells seen by the sensor.
        """
        if obstacle_dist_2d is None:
            self.live_obstacles = None
            return
        od = np.asarray(obstacle_dist_2d, dtype=np.float64)
        seen = (np.ones(od.shape, dtype=bool) if seen_2d is None
                else np.asarray(seen_2d, dtype=bool).reshape(od.shape))
        self.live_obstacles = {
            'od': od, 'seen': seen, 'ox': float(origin_x), 'oy': float(origin_y),
            'cs': float(cell_size), 'rx': float(robot_x), 'ry': float(robot_y),
            'yaw': None if robot_yaw is None else float(robot_yaw)}
        nr, nc = od.shape[:2]
        self.dirty_rects.append((float(origin_x), float(origin_x + nc * cell_size),
                                 float(origin_y), float(origin_y + nr * cell_size)))

    def _live_blocked_samples(self, sx, sy):
        """
        Samples (arrays of any shape) -> True where the live scan says "here the robot's
        flank enters an obstacle": cell inside the window, seen, with obstacle_distance
        below LIVE_PREFILTER_OD_M, outside the body rectangle.
        """
        lo = self.live_obstacles
        if lo is None or LIVE_PREFILTER_OD_M is None:
            return np.zeros(np.shape(sx), dtype=bool)
        od = lo['od']
        ny, nx = od.shape
        cols = np.floor((sx - lo['ox']) / lo['cs']).astype(np.int64)
        rows = np.floor((sy - lo['oy']) / lo['cs']).astype(np.int64)
        inside = (rows >= 0) & (rows < ny) & (cols >= 0) & (cols < nx)
        rows = np.clip(rows, 0, ny - 1)
        cols = np.clip(cols, 0, nx - 1)
        v = od[rows, cols]
        hit = inside & lo['seen'][rows, cols] & np.isfinite(v) & (v < LIVE_PREFILTER_OD_M)
        dx, dy = sx - lo['rx'], sy - lo['ry']
        if lo['yaw'] is None:
            body = np.hypot(dx, dy) <= geometry.ROBOT_HALF_WIDTH_M
        else:
            c, s = np.cos(lo['yaw']), np.sin(lo['yaw'])
            body = ((np.abs(c * dx + s * dy) <= geometry.ROBOT_HALF_LENGTH_M)
                    & (np.abs(-s * dx + c * dy) <= geometry.ROBOT_HALF_WIDTH_M))
        return hit & ~body

    def _live_blocked_many(self, pairs):
        """[(idx1, idx2, dist), ...] -> bool array: edge discarded by the live scan."""
        n_pairs = len(pairs)
        out = np.zeros(n_pairs, dtype=bool)
        if n_pairs == 0 or self.live_obstacles is None or LIVE_PREFILTER_OD_M is None:
            return out
        xy1 = np.array([self.nodes[p[0]] for p in pairs], dtype=np.float64)
        xy2 = np.array([self.nodes[p[1]] for p in pairs], dtype=np.float64)
        d = np.array([p[2] for p in pairs], dtype=np.float64)
        ns = np.maximum(OCCUPANCY_SAMPLES_MIN,
                        np.ceil(d * OCCUPANCY_SAMPLES_PER_M).astype(np.int64) + 1)
        for m in np.unique(ns):
            sel = np.nonzero(ns == m)[0]
            t = np.arange(int(m)) / (int(m) - 1)
            sx = xy1[sel, 0, None] + t[None, :] * (xy2[sel, 0] - xy1[sel, 0])[:, None]
            sy = xy1[sel, 1, None] + t[None, :] * (xy2[sel, 1] - xy1[sel, 1])[:, None]
            out[sel] = self._live_blocked_samples(sx, sy).any(axis=1)
        return out

    def live_prune_valid_edges(self, skip_keys=()):
        """
        Preliminary discard on the edges ALREADY valid that the refresh does not re-evaluate
        (outside REFRESH_RADIUS_M but inside the scan window): an edge 1.3-1.9 m from the robot
        crossing an obstacle just seen does not stay valid until the robot gets closer.
        They are only removed from the graph (validity False): at the next full evaluation they
        come back if the data says so. Returns how many edges it removed.
        """
        lo = self.live_obstacles
        if lo is None or LIVE_PREFILTER_OD_M is None:
            return 0
        ny, nx = lo['od'].shape
        x0, y0 = lo['ox'], lo['oy']
        x1, y1 = x0 + nx * lo['cs'], y0 + ny * lo['cs']
        reach = self.max_edge_length
        cand = []
        for a, nbrs in self.edges.items():
            xa, ya = self.nodes.get(a, (None, None))
            if xa is None or not (x0 - reach <= xa <= x1 + reach and y0 - reach <= ya <= y1 + reach):
                continue
            for b, _w in nbrs:
                if b <= a:
                    continue
                key = (a, b)
                if key in skip_keys or key in self.traversed_edges:
                    continue
                xb, yb = self.nodes[b]
                cand.append((a, b, float(np.hypot(xb - xa, yb - ya))))
        if not cand:
            return 0
        flags = self._live_blocked_many(cand)
        n = 0
        for (a, b, _d), f in zip(cand, flags):
            if not f:
                continue
            self.edge_validity[(a, b)] = False
            self.edges[a] = [(k, w) for k, w in self.edges.get(a, []) if k != b]
            self.edges[b] = [(k, w) for k, w in self.edges.get(b, []) if k != a]
            self._record_occupancy_veto((a, b))
            n += 1
        return n

    def refresh_local_edge_weights(self, global_map=None, edge_safety_margin: float = 0.05):
        """
        Re-evaluate from scratch (occupancy, slope, weight) the edges within REFRESH_RADIUS_M of
        the robot that have at least ONE end in the scan window, including those discarded
        before: an edge removed for a noisy reading becomes valid again as soon as the data
        says so, and an edge crossing a wall just seen towards a distant node is re-examined
        at once. The edges travelled by the robot (traversed_edges) are not touched. Returns
        the re-evaluated nodes.
        """
        if self.current_grad_2d is None or self.grid_origin_x is None or self.cell_size is None:
            return set()

        local_nodes = self._refresh_nodes()
        pairs = self._refresh_pairs(local_nodes)
        self._sync_global_slope_cache()

        # Occupancy of all pairs in one go (outcome identical to _occupancy_blocked)
        # and global slope cache filled where there is certainly no data.
        use_occ = global_map is not None and edge_safety_margin > 0.0
        occ_flags = (self._occupancy_blocked_many(pairs, global_map, edge_safety_margin)
                     if use_occ else np.zeros(len(pairs), dtype=bool))
        self._prefill_global_slope_nodata(pairs)

        # Live slopes of all the pairs to evaluate in one go (same numbers as the
        # edge-by-edge computation). Those _refresh_edge does not evaluate are skipped: discarded for
        # occupancy, blocked up close, travelled by the robot (if one of the latter turns out
        # to need re-evaluation, its profile is computed edge by edge).
        live_profs = [None] * len(pairs)
        if REFRESH_BATCH_SLOPES and pairs:
            skip = np.array([bool(o) or (min(a, b), max(a, b)) in self.tracker_blocked_edges
                             or (min(a, b), max(a, b)) in self.traversed_edges
                             for (a, b, _d), o in zip(pairs, occ_flags)], dtype=bool)
            live_profs = self.precompute_arc_slopes(pairs, skip=skip)

        for (idx1, idx2, dist), occ_flag, live_prof in zip(pairs, occ_flags, live_profs):
            self._refresh_edge(idx1, idx2, dist, global_map, occ_flag, live_prof)

        # The valid edges of the window outside the refresh radius at least go through the
        # preliminary discard on the live scan (live_prune_valid_edges).
        done = {(min(a, b), max(a, b)) for a, b, _d in pairs}
        self.live_prune_last = self.live_prune_valid_edges(skip_keys=done)
        self.flush_slope_log("refresh")

        return set(local_nodes)

    def _refresh_nodes(self):
        """The nodes in the scan window and within REFRESH_RADIUS_M of the robot node (if unknown,
        of the centre of the window, which is the robot anyway)."""
        num_rows, num_cols = self.current_grad_2d.shape
        center = None
        if REFRESH_RADIUS_M is not None:
            if self.robot_node is not None and self.robot_node in self.nodes:
                center = self.nodes[self.robot_node]
            else:
                center = (self.grid_origin_x + num_cols * self.cell_size / 2.0,
                          self.grid_origin_y + num_rows * self.cell_size / 2.0)
        local_nodes = []
        for idx, (x, y) in self.nodes.items():
            col_idx = int(np.floor((x - self.grid_origin_x) / self.cell_size))
            row_idx = int(np.floor((y - self.grid_origin_y) / self.cell_size))
            if not (0 <= row_idx < num_rows and 0 <= col_idx < num_cols):
                continue
            if center is not None and np.hypot(x - center[0], y - center[1]) > REFRESH_RADIUS_M:
                continue        # outside the radius: keeps the weights of the last evaluation
            local_nodes.append(idx)
        return local_nodes

    def _refresh_pairs(self, local_nodes):
        """[(idx1, idx2, dist)]: every pair of nodes at edge distance with idx1 local, each
        pair once."""
        local_set = set(local_nodes)
        all_ids = list(self.nodes.keys())
        all_xy = np.array([self.nodes[i] for i in all_ids], dtype=np.float64) if all_ids else np.empty((0, 2))
        reach = min(self.connection_radius, self.max_edge_length)
        pairs = []
        for idx1 in local_nodes:
            x1, y1 = self.nodes[idx1]
            d_all = np.hypot(all_xy[:, 0] - x1, all_xy[:, 1] - y1)
            for j in np.nonzero((d_all >= self.min_edge_length) & (d_all <= reach))[0]:
                idx2 = all_ids[j]
                if idx2 == idx1 or (idx2 in local_set and idx2 < idx1):
                    continue           # local-local pair already considered from the other end
                pairs.append((idx1, idx2, float(d_all[j])))
        return pairs

    def _refresh_edge(self, idx1, idx2, dist, global_map, occ_blocked, live_prof):
        """
        Re-evaluate one edge of the refresh: occ_blocked is the occupancy outcome already
        computed, live_prof the slope profile computed in batch (None = computed here).
        """
        key = (min(idx1, idx2), max(idx1, idx2))
        if key in self.tracker_blocked_edges:
            return  # block confirmed up close (safe frontier) -- never recreated
        if key in self.traversed_edges:
            if self._traversed_edge_ok(idx1, idx2, global_map):
                return  # travelled by the robot: free by construction
            self.traversed_edges.discard(key)   # now crosses the spot of a fall

        old_valid = self.edge_validity.get(key)  # None if the edge had never been evaluated before
        prof = (0.0, 0.0, False)
        blocked = bool(occ_blocked)
        if blocked:
            self._record_occupancy_veto(key)
        else:
            # A single terrain reading; veto and cost use the same value, the one
            # with the body (see body_slope and slopes.ARC_BODY_EXTENSION_M).
            x1, y1 = self.nodes[idx1]
            x2, y2 = self.nodes[idx2]
            prof = (live_prof if live_prof is not None
                    else self.compute_arc_slope_profile(x1, y1, x2, y2, body=True))
            long_slope, lat_slope, has_slope_data = body_slope(prof)
            if has_slope_data:
                self.edge_slope_info[key] = (long_slope, lat_slope)  # for visualization, no recomputation
                if self._record_slope_veto(key, long_slope, lat_slope):
                    blocked = True

                # --- Diagnostics only: an edge near the threshold that changes state between two
                # consecutive refreshes may indicate oscillation due to sensor noise
                # rather than a real change in the terrain -- it does not affect
                # any decision, it only serves to bring it out in the logs.
                near_thresh = slopes.SLOPE_THRESHOLD * (1.0 - slopes.NEAR_THRESHOLD_MARGIN_FRACTION)
                is_near_threshold = (long_slope >= near_thresh) or (lat_slope >= near_thresh)
                if is_near_threshold and old_valid is not None and old_valid != (not blocked):
                    self.slope_flip_count += 1
                    self._slope_pending()['flip'] += 1

        self.edges[idx1] = [(n, w) for n, w in self.edges.get(idx1, []) if n != idx2]
        self.edges[idx2] = [(n, w) for n, w in self.edges.get(idx2, []) if n != idx1]
        if blocked:
            self.edge_validity[key] = False
        else:
            # The live profile is already computed (even when it says "no data"):
            # it is not redone inside the weight computation.
            weight = self._compute_edge_weight(idx1, idx2, dist, precomputed_slope=prof)
            self.edges[idx1].append((idx2, weight))
            self.edges[idx2].append((idx1, weight))
            self.edge_validity[key] = True

    def compute_arc_slope_profile(self, x1: float, y1: float, x2: float, y2: float,
                                   perp_sample_spacing_m: float = 0.10, body: bool = False):
        """
        slopes.compute_arc_slope_profile on the live grid of this loop (see
        update_local_grid_data). Returns (longitudinal slope, lateral, has_data);
        with body=True the edge is extended by half a body on each side.
        """
        return slopes.compute_arc_slope_profile(
            self.current_terrain_2d, self.grid_origin_x, self.grid_origin_y, self.cell_size,
            x1, y1, x2, y2, perp_sample_spacing_m=perp_sample_spacing_m,
            valid_2d=self.current_valid_2d,
            body_ext_m=(slopes.ARC_BODY_EXTENSION_M if body else 0.0)
        )

    def sample_terrain_between_points(self, x1: float, y1: float, x2: float, y2: float,
                                       node_idx1: Optional[int] = None, node_idx2: Optional[int] = None,
                                       precomputed_slope: Optional[Tuple[float, float, bool]] = None,
                                       need_roughness: bool = True):
        """
        Difficulty of the stretch (x1, y1) -> (x2, y2): (longitudinal slope, lateral slope,
        roughness). Used for the edge cost and the gait choice.

        Slope: directional, with the body (body_slope on compute_arc_slope_profile). Sources,
        from the best to the coarsest:
          1. the current scan (self.current_terrain_2d);
          2. the global height map, for a stretch already seen but now outside the window;
          3. the mean slope of the two nodes, all on the longitudinal component.
        precomputed_slope: profile already computed by the caller (3 or 6 values), so it is not redone.

        Roughness: 90th percentile of the per-cell field along the stretch (OCCUPANCY_SAMPLES_PER_M
        samples per metre); outside the scan, the mean of the nodes. need_roughness=False
        skips it (with gamma = 0 it does not enter the cost).

        node_idx1/node_idx2: node ids, for the global map cache and for the fallback.
        """
        dist = float(np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
        num_samples = max(OCCUPANCY_SAMPLES_MIN, int(dist * OCCUPANCY_SAMPLES_PER_M))

        # --- Roughness: isotropic, from the per-cell field of the scan ---
        sampled_roughness = []
        has_rough_grid = (need_roughness and self.current_rough_2d is not None
                          and self.grid_origin_x is not None and self.cell_size is not None)
        if has_rough_grid:
            num_rows, num_cols = self.current_rough_2d.shape
            for i in range(num_samples):
                t = i / (num_samples - 1)
                cx = x1 + t * (x2 - x1)
                cy = y1 + t * (y2 - y1)

                dx = cx - self.grid_origin_x
                dy = cy - self.grid_origin_y
                col_idx = int(np.floor(dx / self.cell_size))   # floor: int() is wrong below zero
                row_idx = int(np.floor(dy / self.cell_size))

                if 0 <= row_idx < num_rows and 0 <= col_idx < num_cols:
                    sampled_roughness.append(self.current_rough_2d[row_idx, col_idx])

        if sampled_roughness:
            actual_roughness = slopes.p90(sampled_roughness)
        else:
            r1 = self.node_roughness.get(node_idx1, 0.0)
            r2 = self.node_roughness.get(node_idx2, 0.0)
            actual_roughness = float((r1 + r2) / 2.0)

        # --- Slope: directional and with the body (body_slope), from the live scan ---
        if precomputed_slope is None:
            precomputed_slope = self.compute_arc_slope_profile(x1, y1, x2, y2, body=True)
        longitudinal_slope, lateral_slope, has_slope_data = body_slope(precomputed_slope)
        source = 'live'

        if not has_slope_data and self.global_terrain_map is not None:
            # Outside the current scan: the global map, if the stretch had already been seen.
            g_long, g_lat, g_has_data = self._global_slope(x1, y1, x2, y2, node_idx1, node_idx2)
            if g_has_data:
                longitudinal_slope, lateral_slope, has_slope_data = g_long, g_lat, True
                source = 'global_map'

        if not has_slope_data:
            # No data from any source: mean of the nodes, all on the longitudinal.
            grad1 = self.node_gradients.get(node_idx1, 0.0)
            grad2 = self.node_gradients.get(node_idx2, 0.0)
            longitudinal_slope = float((grad1 + grad2) / 2.0)
            lateral_slope = 0.0
            source = 'fallback_nodes'

        # Statistics: only for the summary, they decide nothing.
        self.slope_source_counts[source] += 1
        self.last_slope_source = source

        return float(longitudinal_slope), float(lateral_slope), actual_roughness

    def _compute_edge_weight(self, idx1: int, idx2: int, dist: float,
                             precomputed_slope: Optional[Tuple[float, float, bool]] = None) -> float:
        """
        Edge cost: alpha * length + beta * longitudinal slope + beta_lateral * lateral slope
        + gamma * roughness, with the slopes from sample_terrain_between_points.
        The slope term is fixed per edge (not per metre): it favours long edges, which is
        intended (see CHANGELOG 2026-10-14). precomputed_slope avoids recomputing the profile.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]

        # With gamma = 0 roughness does not enter the cost, so it is not sampled (the gait
        # asks sample_terrain_between_points for it directly).
        longitudinal_slope, lateral_slope, actual_roughness = self.sample_terrain_between_points(
            x1, y1, x2, y2, node_idx1=idx1, node_idx2=idx2,
            precomputed_slope=precomputed_slope, need_roughness=(self.gamma != 0.0)
        )

        # Edge cost
        weight = ((self.alpha * dist) + (self.beta * longitudinal_slope) +
                 (self.beta_lateral * lateral_slope) + (self.gamma * actual_roughness))
        return float(weight)

    def _global_slope(self, x1, y1, x2, y2, idx1=None, idx2=None):
        """
        Slope from the global map, with a per-edge cache. The cache is used only when the points
        really are the two nodes (not for a partial stretch towards an intermediate point).
        """
        tm = self.global_terrain_map

        def _with_body(x1, y1, x2, y2):
            # As for the live one (sample_terrain_between_points): the worse of edge and
            # edge extended by half a body, only if the edge has data of its own.
            ext = slopes.ARC_BODY_EXTENSION_M
            if ext <= 0:
                return tm.compute_arc_slope_profile(x1, y1, x2, y2)
            r = tm.compute_arc_slope_profile(x1, y1, x2, y2, body_ext_m=ext)
            return (r[3], r[4], True) if r[2] else r[:3]

        cacheable = (idx1 is not None and idx2 is not None and idx1 in self.nodes and idx2 in self.nodes
                     and self.nodes[idx1] == (x1, y1) and self.nodes[idx2] == (x2, y2))
        if not cacheable:
            return _with_body(x1, y1, x2, y2)
        key = (min(idx1, idx2), max(idx1, idx2))
        hit = self._gslope_cache.get(key)
        if hit is None:
            hit = _with_body(x1, y1, x2, y2)
            self._gslope_cache[key] = hit
        return hit

    def _sync_global_slope_cache(self):
        """
        Drop the cache entries on which the global map has changed: the edges whose
        rectangle (widened by the lateral slice) intersects an area updated after the
        computation. If the map does not record the updated areas, clear everything (always correct).
        """
        tm = self.global_terrain_map
        if tm is None:
            self._gslope_cache.clear()
            return
        if id(tm) != self._gslope_map_id:
            self._gslope_cache.clear()
            self._gslope_map_id = id(tm)
            self._gslope_seen_updates = len(getattr(tm, 'update_rects', None) or [])
            return
        rects = getattr(tm, 'update_rects', None)
        if rects is None:
            self._gslope_cache.clear()
            return
        new = rects[self._gslope_seen_updates:]
        self._gslope_seen_updates = len(rects)
        if not new or not self._gslope_cache:
            return
        keys = [k for k in self._gslope_cache if k[0] in self.nodes and k[1] in self.nodes]
        if len(keys) != len(self._gslope_cache):
            self._gslope_cache = {k: self._gslope_cache[k] for k in keys}
        if not keys:
            return
        a = np.array([self.nodes[k[0]] for k in keys]); b = np.array([self.nodes[k[1]] for k in keys])
        # + ARC_BODY_EXTENSION_M: the cached value also looks beyond the ends.
        m = slopes.LATERAL_SLICE_HALF_WIDTH_M + 2 * tm.resolution + slopes.ARC_BODY_EXTENSION_M
        exmin, exmax = np.minimum(a[:, 0], b[:, 0]) - m, np.maximum(a[:, 0], b[:, 0]) + m
        eymin, eymax = np.minimum(a[:, 1], b[:, 1]) - m, np.maximum(a[:, 1], b[:, 1]) + m
        stale = np.zeros(len(keys), dtype=bool)
        for (rxmin, rxmax, rymin, rymax) in new:
            stale |= (exmax >= rxmin) & (exmin <= rxmax) & (eymax >= rymin) & (eymin <= rymax)
        for j in np.nonzero(stale)[0]:
            del self._gslope_cache[keys[j]]

    def _traversed_edge_ok(self, a, b, global_map) -> bool:
        """
        An edge travelled by the robot stays in the graph without checks (it is free by
        construction), EXCEPT if it was blocked afterwards (confirmed block or physical failure)
        or if it now passes over the spot of a fall: an edge travelled on the way out and then
        blocked must not come back into the graph at the next rebuild.
        """
        key = (min(a, b), max(a, b))
        if key in self.tracker_blocked_edges:
            return False
        if global_map is not None and hasattr(global_map, 'segment_hits_hazard'):
            (xa, ya), (xb, yb) = self.nodes[a], self.nodes[b]
            if global_map.segment_hits_hazard(xa, ya, xb, yb, geometry.PRM_EDGE_SAFETY_MARGIN_M):
                return False
        return True

    def _occupancy_blocked(self, idx1, idx2, dist, global_map, edge_safety_margin,
                           use_live=True) -> bool:
        """
        True if the edge is blocked by occupancy: by the preliminary discard on the live scan
        (LIVE_PREFILTER_OD_M) or by the global map, with a sample every 10 cm. The samples
        within robot_node_ignore_m of the robot node do not count for the global map.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]
        if use_live and self.live_obstacles is not None:
            if self._live_blocked_many([(idx1, idx2, float(dist))])[0]:
                return True
        ignore_xy = None
        if self.robot_node is not None and self.robot_node in (idx1, idx2) and self.robot_node_ignore_m > 0:
            ignore_xy = self.nodes[self.robot_node]
        # ceil + 1: step between samples <= 1/OCCUPANCY_SAMPLES_PER_M.
        num_samples = max(OCCUPANCY_SAMPLES_MIN, int(np.ceil(dist * OCCUPANCY_SAMPLES_PER_M)) + 1)
        for k in range(num_samples):
            t = k / (num_samples - 1)
            sx = x1 + t * (x2 - x1)
            sy = y1 + t * (y2 - y1)
            if ignore_xy is not None and np.hypot(sx - ignore_xy[0], sy - ignore_xy[1]) <= self.robot_node_ignore_m:
                continue
            if global_map.is_occupied(sx, sy, safety_margin=edge_safety_margin):
                return True
        return False

    def _occupancy_blocked_many(self, pairs, global_map, edge_safety_margin):
        """
        _occupancy_blocked for many edges together: [(idx1, idx2, dist), ...] -> bool array,
        IDENTICAL edge by edge (same samples, same exemption around the robot node, same
        GlobalGrid test through occupied_many).
        """
        # Preliminary discard on the live scan (LIVE_PREFILTER_OD_M) and global map:
        # the outcome is the OR of the two.
        live = self._live_blocked_many(pairs)
        self.live_veto_last = int(live.sum())
        out = self._global_occupancy_blocked_many(pairs, global_map, edge_safety_margin)
        return out | live

    def _global_occupancy_blocked_many(self, pairs, global_map, edge_safety_margin):
        """The global map check of _occupancy_blocked_many, without the live scan."""
        n_pairs = len(pairs)
        out = np.zeros(n_pairs, dtype=bool)
        if n_pairs == 0 or global_map is None or edge_safety_margin <= 0.0:
            return out
        if not hasattr(global_map, 'occupied_many'):
            for i, (a, b, d) in enumerate(pairs):
                out[i] = self._occupancy_blocked(a, b, d, global_map, edge_safety_margin,
                                                 use_live=False)
            return out
        a_ids = [p[0] for p in pairs]
        b_ids = [p[1] for p in pairs]
        xy1 = np.array([self.nodes[a] for a in a_ids], dtype=np.float64)
        xy2 = np.array([self.nodes[b] for b in b_ids], dtype=np.float64)
        d = np.array([p[2] for p in pairs], dtype=np.float64)
        ns = np.maximum(OCCUPANCY_SAMPLES_MIN,
                        np.ceil(d * OCCUPANCY_SAMPLES_PER_M).astype(np.int64) + 1)
        ignore_xy = None
        touches = np.zeros(n_pairs, dtype=bool)
        if self.robot_node is not None and self.robot_node_ignore_m > 0 and self.robot_node in self.nodes:
            ignore_xy = self.nodes[self.robot_node]
            touches = np.array([(a == self.robot_node) or (b == self.robot_node)
                                for a, b in zip(a_ids, b_ids)], dtype=bool)
        for m in np.unique(ns):
            sel = np.nonzero(ns == m)[0]
            t = np.arange(int(m)) / (int(m) - 1)
            sx = xy1[sel, 0, None] + t[None, :] * (xy2[sel, 0] - xy1[sel, 0])[:, None]
            sy = xy1[sel, 1, None] + t[None, :] * (xy2[sel, 1] - xy1[sel, 1])[:, None]
            keep = None
            if ignore_xy is not None and touches[sel].any():
                near = np.hypot(sx - ignore_xy[0], sy - ignore_xy[1]) <= self.robot_node_ignore_m
                keep = ~(near & touches[sel, None])
            occ = global_map.occupied_many(sx, sy, safety_margin=edge_safety_margin)
            if keep is not None:
                occ &= keep
            out[sel] = occ.any(axis=1)
        return out

    def _prefill_global_slope_nodata(self, pairs):
        """
        Cache (0, 0, False) -- "no data" -- for the edges on which the global terrain map
        would certainly answer so (GlobalTerrainGrid.certainly_no_slope_data): it is the
        same value, in the same direction, that _global_slope would compute at the first call.
        """
        tm = self.global_terrain_map
        if tm is None or not hasattr(tm, 'certainly_no_slope_data') or not pairs:
            return 0
        todo = [(a, b) for a, b, _ in pairs
                if (min(a, b), max(a, b)) not in self._gslope_cache]
        if not todo:
            return 0
        xy1 = np.array([self.nodes[a] for a, _ in todo], dtype=np.float64)
        xy2 = np.array([self.nodes[b] for _, b in todo], dtype=np.float64)
        nodata = tm.certainly_no_slope_data(xy1[:, 0], xy1[:, 1], xy2[:, 0], xy2[:, 1])
        n = 0
        for (a, b), nd in zip(todo, nodata):
            if nd:
                self._gslope_cache[(min(a, b), max(a, b))] = (0.0, 0.0, False)
                n += 1
        return n

    def precompute_arc_slopes(self, pairs, skip=None):
        """
        Slope of many pairs in one go, with the matrices of
        slopes.arc_slope_profiles_batch: numbers IDENTICAL to the edge-by-edge computation.

        pairs: [(idx1, idx2, dist)], in the direction they will be evaluated; skip: mask of
        the pairs not to compute (e.g. already discarded for occupancy).
        Returns, for every pair, the live profile (6 values, like
        compute_arc_slope_profile(..., body=True)) or None if skipped. Also fills the global
        slope cache (_gslope_cache) for the pairs without live data, with the same value and
        the same direction _global_slope would compute at the first call.
        """
        n = len(pairs)
        out = [None] * n
        if n == 0:
            return out
        use = np.ones(n, dtype=bool) if skip is None else ~np.asarray(skip, dtype=bool)
        idx = np.nonzero(use)[0]
        if idx.size == 0:
            return out
        xy1 = np.array([self.nodes[pairs[i][0]] for i in idx], dtype=np.float64)
        xy2 = np.array([self.nodes[pairs[i][1]] for i in idx], dtype=np.float64)
        ext = slopes.ARC_BODY_EXTENSION_M
        live = slopes.compute_arc_slope_profiles_batch(
            self.current_terrain_2d, self.grid_origin_x, self.grid_origin_y, self.cell_size,
            xy1[:, 0], xy1[:, 1], xy2[:, 0], xy2[:, 1],
            valid_2d=self.current_valid_2d, body_ext_m=ext)
        for j, i in enumerate(idx):
            r = live[j]
            out[i] = (float(r[0]), float(r[1]), bool(r[2]), float(r[3]), float(r[4]), bool(r[5]))
        tm = self.global_terrain_map
        if tm is not None and hasattr(tm, 'compute_arc_slope_profiles_batch') and ext > 0:
            need = [j for j, i in enumerate(idx) if not live[j, 2]
                    and (min(pairs[i][0], pairs[i][1]), max(pairs[i][0], pairs[i][1])) not in self._gslope_cache]
            if need:
                need = np.array(need)
                g = tm.compute_arc_slope_profiles_batch(xy1[need, 0], xy1[need, 1], xy2[need, 0], xy2[need, 1],
                                                        body_ext_m=ext)
                for j, r in zip(need, g):
                    a, b = pairs[idx[j]][0], pairs[idx[j]][1]
                    # like _global_slope._with_body: with the body if the edge has data of its own
                    self._gslope_cache[(min(a, b), max(a, b))] = (
                        (float(r[3]), float(r[4]), True) if r[2] else (float(r[0]), float(r[1]), False))
        return out

    def _live_slope_certainly_nodata(self, pairs):
        """
        For every pair: True if compute_arc_slope_profile on the live scan would exit at once
        with "no data" (its own quick exit, same numbers).
        """
        n = len(pairs)
        out = np.ones(n, dtype=bool)
        if (n == 0 or self.current_terrain_2d is None or self.grid_origin_x is None
                or self.cell_size is None):
            return out
        xy1 = np.array([self.nodes[a] for a, _, _ in pairs], dtype=np.float64)
        xy2 = np.array([self.nodes[b] for _, b, _ in pairs], dtype=np.float64)
        num_rows, num_cols = self.current_terrain_2d.shape
        pad = slopes.LATERAL_SLICE_HALF_WIDTH_M + self.cell_size
        x_hi = self.grid_origin_x + num_cols * self.cell_size
        y_hi = self.grid_origin_y + num_rows * self.cell_size
        out = ((np.maximum(xy1[:, 0], xy2[:, 0]) + pad < self.grid_origin_x)
               | (np.minimum(xy1[:, 0], xy2[:, 0]) - pad > x_hi)
               | (np.maximum(xy1[:, 1], xy2[:, 1]) + pad < self.grid_origin_y)
               | (np.minimum(xy1[:, 1], xy2[:, 1]) - pad > y_hi))
        return out

    def _add_edge_without_data(self, idx1: int, idx2: int, dist: float) -> bool:
        """
        AREA WITHOUT DATA: edge on which no source has information (outside the scan, global
        terrain map certainly without data, occupancy free, not blocked, gamma = 0).
        _evaluate_and_add_edge would end up at the fallback cost (length + mean slope of the
        nodes): here it is computed directly, with the same formula and the same counters,
        result identical to the bit. Only for adjacencies just cleared (fresh).
        """
        grad1 = self.node_gradients.get(idx1, 0.0)
        grad2 = self.node_gradients.get(idx2, 0.0)
        longitudinal_slope = float((grad1 + grad2) / 2.0)
        lateral_slope = 0.0
        r1 = self.node_roughness.get(idx1, 0.0)
        r2 = self.node_roughness.get(idx2, 0.0)
        actual_roughness = float((r1 + r2) / 2.0)
        self.slope_source_counts['fallback_nodes'] += 1
        self.last_slope_source = 'fallback_nodes'
        weight = float((self.alpha * dist) + (self.beta * float(longitudinal_slope)) +
                       (self.beta_lateral * float(lateral_slope)) + (self.gamma * actual_roughness))
        self.edges.setdefault(idx1, []).append((idx2, weight))
        self.edges.setdefault(idx2, []).append((idx1, weight))
        self.edge_validity[(min(idx1, idx2), max(idx1, idx2))] = True
        return True

    def _evaluate_and_add_edge(self, idx1: int, idx2: int, dist: float,
                               global_map=None, edge_safety_margin: float = 0.0,
                               force: bool = False, occ_blocked=None,
                               fresh: bool = False, live_prof=None) -> bool:
        """
        Decide whether edge idx1-idx2 is allowed and, if so, add it with its weight.
        Checks in order: edge blocked, occupancy, slope with the body; then the weight.

        force=True skips the vetoes and adds the edge anyway: ONLY for the stretch the robot
        has just travelled on its own legs, free by construction.
        occ_blocked: outcome already computed by _occupancy_blocked_many (None = computed here).
        fresh=True: the adjacencies have just been cleared, there is no old edge to remove.
        live_prof: slope profile already computed in batch (precompute_arc_slopes).
        Returns True if the edge was added.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]
        key = (min(idx1, idx2), max(idx1, idx2))

        long_slope, lat_slope, has_slope_data = 0.0, 0.0, False
        prof = (long_slope, lat_slope, has_slope_data)
        if not force:
            if key in self.tracker_blocked_edges:
                self.edge_validity[key] = False
                return False

            # Occupancy (live scan and global map).
            if global_map is not None and edge_safety_margin > 0.0:
                blocked_now = (bool(occ_blocked) if occ_blocked is not None else
                               self._occupancy_blocked(idx1, idx2, dist, global_map, edge_safety_margin))
                if blocked_now:
                    self._record_occupancy_veto(key)
                    self.edge_validity[key] = False
                    return False

            # Slope with the body, forward or sideways. live_prof = profile already computed in
            # batch (precompute_arc_slopes).
            prof = (live_prof if live_prof is not None
                    else self.compute_arc_slope_profile(x1, y1, x2, y2, body=True))
            long_slope, lat_slope, has_slope_data = body_slope(prof)
            if has_slope_data:
                self.edge_slope_info[key] = (long_slope, lat_slope)  # for visualization
                if self._record_slope_veto(key, long_slope, lat_slope):
                    self.edge_validity[key] = False
                    return False

        weight = self._compute_edge_weight(
            idx1, idx2, dist,
            precomputed_slope=(None if force else prof))

        if fresh:
            self.edges.setdefault(idx1, []).append((idx2, weight))
            self.edges.setdefault(idx2, []).append((idx1, weight))
        else:
            self.edges[idx1] = [(n, w) for n, w in self.edges.get(idx1, []) if n != idx2]
            self.edges[idx2] = [(n, w) for n, w in self.edges.get(idx2, []) if n != idx1]
            self.edges[idx1].append((idx2, weight))
            self.edges[idx2].append((idx1, weight))
        self.edge_validity[key] = True
        return True
