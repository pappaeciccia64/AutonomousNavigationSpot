"""Global maps accumulated over the whole mission: occupancy, free space, terrain."""
import numpy as np
from scipy.ndimage import binary_dilation
import pickle
from spot_explorer.perception.geometry import ROBOT_HALF_WIDTH_M
from spot_explorer.perception.slopes import (LATERAL_SLICE_HALF_WIDTH_M, MIN_ARC_SAMPLE_FRACTION, _no_slope_data, _slope_profile_from_sampler, arc_slope_profiles_batch)
from spot_explorer.perception.spot_grid import slope_usable_mask

# =========================================================================
# GLOBAL OBSTACLE MAP (see GlobalGrid.update)
#   GLOBAL_OCC_CONFIRM_OBS: a cell becomes occupied after this many CONSECUTIVE obstacle
#     observations: the map remembers obstacles from one attempt to the next and cannot
#     trust an isolated scan. The first obstacle observation resets the "free" evidence
#     (fast reaction to a new obstacle).
#   GLOBAL_OCC_CONFIRM_WALKED: confirmations required where the robot passed with its body:
#     having walked there is stronger evidence than a reading from afar. The cells do not
#     become immune (the world changes), but more confirmations are needed.
#   GLOBAL_OCC_MAX_COUNT: cap of the evidence counter.
# =========================================================================
GLOBAL_OCC_CONFIRM_OBS = 2
GLOBAL_OCC_MAX_COUNT = 4
GLOBAL_OCC_COARSE_CELLS = 8          # side of the coarse index block (8 x 3 cm = 24 cm)
GLOBAL_OCC_CONFIRM_WALKED = 4

# =========================================================================
# FREE-SPACE MAP (see GlobalGrid.update_clearance)
# For each cell it remembers HOW MUCH space there was (obstacle_distance), not only "obstacle yes/no".
# Used by the cell goal choice (goal.find_best_point_in_cell); graph edges and
# nodes are judged on the binary map. Statistic: mean of the observations taken
# within CLEARANCE_NEAR_M of the robot; where there are none, mean of all.
# =========================================================================
CLEARANCE_NEAR_M = 1.0


class GlobalGrid:
    def __init__(self, resolution=0.03):
        self.resolution = resolution
        # Global dictionary: the key is (grid_x, grid_y), the value is the state:
        #  1.0 -> Free/Safe
        # -1.0 -> Occupied/Obstacle
        self.grid = {}
        self.obs_count = {}
        self._cached_margin = None
        self._cached_offsets = []
        self.global_occupancy_map = None
        self.global_terrain_map = None  # GlobalTerrainGrid, lazily initialized elsewhere
        # Coarse index: block (GLOBAL_OCC_COARSE_CELLS cells per side) -> number of
        # occupied cells inside it. Lets is_occupied() answer "free" without
        # looking cell by cell when there is no block with obstacles nearby,
        # which is by far the most common case (see PRM_EDGE_SAFETY_MARGIN_M).
        self._coarse_occ = {}
        # Cells of the hazard disks (mark_hazard): they stay occupied for the whole mission.
        self.hazard_cells = set()
        self.hazards = []
        # Cells the robot's BODY passed over: declaring them occupied requires
        # GLOBAL_OCC_CONFIRM_WALKED confirmations instead of GLOBAL_OCC_CONFIRM_OBS.
        self.walked_cells = set()
        # FREE-SPACE MAP (see CLEARANCE_NEAR_M): mean of the observed obstacle_distance,
        # kept separately for close-up views and all views.
        self.clear_near_sum = {}
        self.clear_near_n = {}
        self.clear_all_sum = {}
        self.clear_all_n = {}
        # Version of the occupied cells: grows at every change to/from -1. Used
        # to reuse the dilated raster of occupied_many until the map changes.
        self._occ_version = 0
        self._margin_raster = None      # (version, steps, raster, gx0, gy0)

    def _coarse_key(self, gx, gy):
        c = GLOBAL_OCC_COARSE_CELLS
        return (gx // c, gy // c)

    def _set_state(self, key, new_state):
        """The only place where a cell's state changes: keeps the coarse index aligned."""
        old_state = self.grid.get(key)
        if old_state == new_state:
            return
        self.grid[key] = new_state
        if new_state == -1.0 or old_state == -1.0:
            self._occ_version = getattr(self, '_occ_version', 0) + 1
        ck = self._coarse_key(*key)
        if new_state == -1.0:
            self._coarse_occ[ck] = self._coarse_occ.get(ck, 0) + 1
        elif old_state == -1.0:
            n = self._coarse_occ.get(ck, 0) - 1
            if n > 0:
                self._coarse_occ[ck] = n
            else:
                self._coarse_occ.pop(ck, None)

    def clearance(self, x, y):
        """
        Free space remembered at that point (m), or None if never observed. Mean of the
        close-up views when there are some, otherwise mean of all (see CLEARANCE_NEAR_M).
        """
        key = (int(np.floor(x / self.resolution)), int(np.floor(y / self.resolution)))
        n = self.clear_near_n.get(key, 0)
        if n:
            return self.clear_near_sum[key] / n
        n = self.clear_all_n.get(key, 0)
        return (self.clear_all_sum[key] / n) if n else None

    def update_clearance(self, pts, obstacle_distance, usable, near=None):
        """
        Learn the free space per cell. Called next to update().

        obstacle_distance: the (already filtered) distances of the scan, aligned with pts.
        usable: cells it can learn from (reliable, or under the body).
        near: observations taken within CLEARANCE_NEAR_M of the robot -- they go into a
            separate accumulator, which takes precedence (see clearance()).
        """
        p = np.asarray(pts).reshape(-1, pts.shape[-1] if np.ndim(pts) > 1 else 2)
        od = np.asarray(obstacle_distance, dtype=np.float64).ravel()
        us = np.asarray(usable, dtype=bool).ravel()
        if od.size != len(p) or us.size != len(p):
            return
        gx = np.floor(p[:, 0] / self.resolution).astype(np.int64)
        gy = np.floor(p[:, 1] / self.resolution).astype(np.int64)
        nr = (np.zeros(len(p), dtype=bool) if near is None
              else np.asarray(near, dtype=bool).ravel())
        for k, v, ok, isnear in zip(zip(gx.tolist(), gy.tolist()), od, us, nr):
            if not ok:
                continue
            self.clear_all_sum[k] = self.clear_all_sum.get(k, 0.0) + float(v)
            self.clear_all_n[k] = self.clear_all_n.get(k, 0) + 1
            if isnear:
                self.clear_near_sum[k] = self.clear_near_sum.get(k, 0.0) + float(v)
                self.clear_near_n[k] = self.clear_near_n.get(k, 0) + 1

    def update(self, pts, obstacle_mask, walked=None):
        """
        Update the map with a scan. obstacle_mask: -1 obstacle, +1 seen free, 0 no
        information (cell not reliable in this scan: skipped).

        Per-cell hysteresis (counter in obs_count):
          - occupied after GLOBAL_OCC_CONFIRM_OBS consecutive obstacle observations (the
            first resets the "free" evidence);
          - free when the counter goes back to 0 or below;
          - in between it stays as it was. A cell never seen and observed free becomes free
            (needed by is_known).
        The cells of a hazard disk (mark_hazard) never become free again.

        walked: mask, aligned with pts, of the cells covered by the robot's BODY now.
        They are remembered (walked_cells) and declaring them occupied requires
        GLOBAL_OCC_CONFIRM_WALKED observations instead of GLOBAL_OCC_CONFIRM_OBS.
        """
        pts_flat = pts.reshape(-1, 3)
        mask_flat = obstacle_mask.ravel()

        # Cell key = floor(x / resolution), as in is_occupied (np.round on the cell centres
        # would round to even and leave keys never written).
        gx_array = np.floor(pts_flat[:, 0] / self.resolution).astype(int)
        gy_array = np.floor(pts_flat[:, 1] / self.resolution).astype(int)

        hazard_cells = self.hazard_cells
        walked_cells = self.walked_cells
        if walked is not None:
            w = np.asarray(walked, dtype=bool).ravel()
            if w.size == gx_array.size:
                walked_cells.update(zip(gx_array[w].tolist(), gy_array[w].tolist()))
        for gx, gy, state in zip(gx_array, gy_array, mask_flat):
            if state == 0:
                continue
            key = (int(gx), int(gy))
            if key in hazard_cells:
                continue
            curr_count = self.obs_count.get(key, 0)

            if state == -1.0:
                new_count = min(curr_count + 1 if curr_count >= 0 else 1, GLOBAL_OCC_MAX_COUNT)
                self.obs_count[key] = new_count
                confirm = (GLOBAL_OCC_CONFIRM_WALKED if key in walked_cells
                           else GLOBAL_OCC_CONFIRM_OBS)
                if new_count >= confirm:
                    self._set_state(key, -1.0)
                elif key not in self.grid:
                    self._set_state(key, 1.0)    # seen, but not yet confirmed occupied
            else:
                new_count = max(curr_count - 1, -GLOBAL_OCC_MAX_COUNT)
                self.obs_count[key] = new_count
                if new_count <= 0:
                    self._set_state(key, 1.0)
                elif key not in self.grid:
                    self._set_state(key, 1.0)

    def is_occupied(self, x, y, safety_margin=0.0):
        """
        True if (x, y) falls in an occupied cell or, with safety_margin > 0, if there is an
        occupied cell within safety_margin metres (disk). A cell never seen is free.
        """
        gx_center = int(np.floor(x / self.resolution))
        gy_center = int(np.floor(y / self.resolution))

        if safety_margin <= 0.0:
            return self.grid.get((gx_center, gy_center), 1.0) == -1.0

        # Coarse prefilter: if no block touching the neighbourhood contains occupied
        # cells, the answer is "free" without looking at the single cells (same result).
        steps = int(np.ceil(safety_margin / self.resolution))
        c = GLOBAL_OCC_COARSE_CELLS
        any_coarse = False
        for cx in range((gx_center - steps) // c, (gx_center + steps) // c + 1):
            for cy in range((gy_center - steps) // c, (gy_center + steps) // c + 1):
                if (cx, cy) in self._coarse_occ:
                    any_coarse = True
                    break
            if any_coarse:
                break
        if not any_coarse:
            return False

        # Regenerate the neighbourhood mask only if the margin changes
        if self._cached_margin != safety_margin:
            self._cached_margin = safety_margin
            steps = int(np.ceil(safety_margin / self.resolution))
            offsets = []
            for dx in range(-steps, steps + 1):
                for dy in range(-steps, steps + 1):
                    if dx ** 2 + dy ** 2 <= steps ** 2:
                        offsets.append((dx, dy))
            self._cached_offsets = offsets

        for dx, dy in self._cached_offsets:
            if self.grid.get((gx_center + dx, gy_center + dy), 1.0) == -1.0:
                return True

        return False

    def _get_margin_raster(self, safety_margin):
        """
        Raster of the cells for which is_occupied(x, y, safety_margin) is True: the
        occupied cells dilated with THE SAME disk as is_occupied (dx^2 + dy^2 <= steps^2,
        steps = ceil(margin / resolution)). Returns (raster, gx0, gy0) or None if there is
        no occupied cell. Rebuilt only when the map changes (_occ_version).
        """
        steps = int(np.ceil(safety_margin / self.resolution)) if safety_margin > 0.0 else 0
        ver = getattr(self, '_occ_version', 0)
        cached = getattr(self, '_margin_raster', None)
        if cached is not None and cached[0] == ver and cached[1] == steps:
            return cached[2]
        occ = [k for k, v in self.grid.items() if v == -1.0]
        if not occ:
            result = None
        else:
            k = np.array(occ, dtype=np.int64)
            gx0, gy0 = int(k[:, 0].min()) - steps, int(k[:, 1].min()) - steps
            nx = int(k[:, 0].max()) - gx0 + steps + 1
            ny = int(k[:, 1].max()) - gy0 + steps + 1
            raster = np.zeros((nx, ny), dtype=bool)
            raster[k[:, 0] - gx0, k[:, 1] - gy0] = True
            if steps > 0:
                d = np.arange(-steps, steps + 1)
                disk = (d[:, None] ** 2 + d[None, :] ** 2) <= steps ** 2
                raster = binary_dilation(raster, structure=disk)
            result = (raster, gx0, gy0)
        self._margin_raster = (ver, steps, result)
        return result

    def occupied_many(self, xs, ys, safety_margin=0.0):
        """
        is_occupied on many points together, with a result IDENTICAL point by point (same
        keys floor(x / resolution), same disk): used for building the graph.
        """
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        out = np.zeros(xs.shape, dtype=bool)
        r = self._get_margin_raster(safety_margin)
        if r is None or xs.size == 0:
            return out
        raster, gx0, gy0 = r
        gx = np.floor(xs / self.resolution).astype(np.int64) - gx0
        gy = np.floor(ys / self.resolution).astype(np.int64) - gy0
        inside = (gx >= 0) & (gx < raster.shape[0]) & (gy >= 0) & (gy < raster.shape[1])
        out[inside] = raster[gx[inside], gy[inside]]
        return out

    def mark_hazard(self, x, y, radius=0.5):
        """
        Mark as occupied a disk around a dangerous point (today: where the robot FELL).
        The terrain there may look normal to the sensors (slippery, yielding, a hole
        covered by grass), so no confirmation from the scans is awaited, and later scans
        do not free it: it stays occupied for the whole mission. Returns the number of
        cells marked.
        """
        if not hasattr(self, 'hazards'):
            self.hazards = []
        if not hasattr(self, 'hazard_cells'):
            self.hazard_cells = set()
        self.hazards.append((float(x), float(y), float(radius)))
        r = self.resolution
        steps = int(np.ceil(radius / r))
        gx0, gy0 = int(np.floor(x / r)), int(np.floor(y / r))
        n = 0
        for dx in range(-steps, steps + 1):
            for dy in range(-steps, steps + 1):
                if dx * dx + dy * dy <= steps * steps:
                    key = (gx0 + dx, gy0 + dy)
                    self.obs_count[key] = GLOBAL_OCC_MAX_COUNT
                    self._set_state(key, -1.0)
                    self.hazard_cells.add(key)
                    n += 1
        return n

    def segment_hits_hazard(self, x1, y1, x2, y2, margin=0.0):
        """True if the segment passes within (radius + margin) of the centre of a hazard disk."""
        for hx, hy, hr in getattr(self, 'hazards', []):
            dx, dy = x2 - x1, y2 - y1
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((hx - x1) * dx + (hy - y1) * dy) / L2))
            if np.hypot(x1 + t * dx - hx, y1 + t * dy - hy) <= hr + margin:
                return True
        return False

    def is_known(self, x, y):
        """
        True if this position has been OBSERVED at least once (free or occupied).
        is_occupied alone does not tell "seen free" from "never seen" (both False): this
        does, and is used for example to choose a goal seen free rather than unknown
        (find_best_point_in_cell) and not to create GraphNav shortcuts on terrain never seen.
        """
        gx = int(np.floor(x / self.resolution))
        gy = int(np.floor(y / self.resolution))
        return (gx, gy) in self.grid

    def save_map(self, file_path):
        """Save the global occupancy map to a pickle file."""
        with open(file_path, 'wb') as f:
            pickle.dump({
                'resolution': self.resolution,
                'grid': self.grid,
                'hazards': list(getattr(self, 'hazards', [])),   # fall disks (x, y, radius)
                'walked_cells': list(getattr(self, 'walked_cells', ())),  # cells the body passed over
                # free-space map (see clearance()): sums and counts
                'clear_near_sum': getattr(self, 'clear_near_sum', {}),
                'clear_near_n': getattr(self, 'clear_near_n', {}),
                'clear_all_sum': getattr(self, 'clear_all_sum', {}),
                'clear_all_n': getattr(self, 'clear_all_n', {}),
            }, f)


class GlobalTerrainGrid:
    """
    Global, persistent map of the terrain HEIGHT (mean of the observations per cell), not
    of the slope: the slope depends on how an edge is travelled and is recomputed on the
    fly (compute_arc_slope_profile). Same resolution as the local grid; if memory became a
    problem in long missions, the lever is `resolution`.

    It is fed only with corrected heights (LocalGrid.correct_terrain) and with the same
    mask as the live computation (slope_usable_mask: reliable, non-obstacle cells),
    through fold_scan_into_global_maps.

    Limit: if the odometry (VISION frame) drifted, readings of the "same" cell would come
    from slightly different points. On the missions of 13-10 the offset between
    consecutive scans is 0.1 cm in median.
    """
    def __init__(self, resolution=0.03):
        self.resolution = resolution
        self.height_sum = {}    # (gx, gy) -> sum of the valid heights observed
        self.height_count = {}  # (gx, gy) -> number of valid observations
        # Rectangles (xmin, xmax, ymin, ymax) updated, one per call to update: they tell
        # the PRM which cached slopes to redo (roadmap._sync_global_slope_cache).
        self.update_rects = []

    def update(self, pts, heights, is_valid):
        """
        Args:
            pts: (N,3) or (N,2) -- only the x,y columns are used (world position)
            heights: 1D array of N heights ALREADY corrected (see correct_terrain)
            is_valid: 1D boolean array of N elements, True where the height is to be used
        """
        pts_flat = np.asarray(pts).reshape(-1, pts.shape[-1] if pts.ndim > 1 else 2)
        x_array = pts_flat[:, 0]
        y_array = pts_flat[:, 1]
        heights_flat = np.asarray(heights).ravel()
        valid_flat = np.asarray(is_valid).ravel()

        # Key = floor(x / resolution) as in get_height (np.round on the cell centres
        # would round to even and leave keys never written).
        gx_array = np.floor(x_array / self.resolution).astype(int)
        gy_array = np.floor(y_array / self.resolution).astype(int)

        if valid_flat.any():
            xv, yv = x_array[valid_flat.astype(bool)], y_array[valid_flat.astype(bool)]
            if not hasattr(self, 'update_rects'):
                self.update_rects = []
            self.update_rects.append((float(xv.min()), float(xv.max()), float(yv.min()), float(yv.max())))

        for gx, gy, h, v in zip(gx_array, gy_array, heights_flat, valid_flat):
            if not v:
                continue
            key = (gx, gy)
            self.height_sum[key] = self.height_sum.get(key, 0.0) + float(h)
            self.height_count[key] = self.height_count.get(key, 0) + 1

    def get_height(self, x, y):
        """Mean height observed at that position, or None if never seen."""
        gx = int(np.floor(x / self.resolution))
        gy = int(np.floor(y / self.resolution))
        key = (gx, gy)
        count = self.height_count.get(key, 0)
        if count == 0:
            return None
        return self.height_sum[key] / count

    def compute_arc_slope_profiles_batch(self, x1, y1, x2, y2, perp_sample_spacing_m=0.10, body_ext_m=0.0):
        """compute_arc_slope_profile of the global map for many edges (arrays), same early
        exits. See arc_slope_profiles_batch for the result format."""
        x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
        x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
        out = np.zeros((x1.size, 6))
        rects = getattr(self, 'update_rects', None)
        near = np.ones(x1.size, dtype=bool)
        if rects is not None:
            if not rects:
                return out
            if not hasattr(self, '_seen_bbox') or self._seen_bbox[4] != len(rects):
                r = np.array(rects)
                self._seen_bbox = (r[:, 0].min(), r[:, 1].max(), r[:, 2].min(), r[:, 3].max(), len(rects))
            bxmin, bxmax, bymin, bymax, _ = self._seen_bbox
            pad = LATERAL_SLICE_HALF_WIDTH_M + 2 * self.resolution
            near = ~((np.maximum(x1, x2) + pad < bxmin) | (np.minimum(x1, x2) - pad > bxmax)
                     | (np.maximum(y1, y2) + pad < bymin) | (np.minimum(y1, y2) - pad > bymax))
        if near.any():
            def sample(xs, ys):
                h = self._sample_heights(xs, ys)
                return h, np.isfinite(h)
            out[near] = arc_slope_profiles_batch(sample, x1[near], y1[near], x2[near], y2[near],
                                                 self.resolution, perp_sample_spacing_m,
                                                 body_ext_m=body_ext_m, sampler_mode=True)
        return out

    def compute_arc_slope_profile(self, x1, y1, x2, y2, perp_sample_spacing_m=0.10, body_ext_m=0.0):
        """
        Like slopes.compute_arc_slope_profile, but on the global map: used for the edges
        outside the current window in areas already seen before (see
        _slope_profile_from_sampler).

        Returns:
            (longitudinal_slope, lateral_slope, has_data)
        """
        rects = getattr(self, 'update_rects', None)
        if rects is not None:
            if not rects:
                return _no_slope_data(body_ext_m)
            # Quick exit: edge entirely outside the rectangle containing everything that has
            # been seen -> no sample can have data (same result, without reading).
            if not hasattr(self, '_seen_bbox') or self._seen_bbox[4] != len(rects):
                r = np.array(rects)
                self._seen_bbox = (r[:, 0].min(), r[:, 1].max(), r[:, 2].min(), r[:, 3].max(), len(rects))
            bxmin, bxmax, bymin, bymax, _ = self._seen_bbox
            pad = LATERAL_SLICE_HALF_WIDTH_M + 2 * self.resolution
            if (max(x1, x2) + pad < bxmin or min(x1, x2) - pad > bxmax or
                    max(y1, y2) + pad < bymin or min(y1, y2) - pad > bymax):
                return _no_slope_data(body_ext_m)
        return _slope_profile_from_sampler(
            self.get_height, x1, y1, x2, y2, self.resolution, perp_sample_spacing_m,
            sample_array=self._sample_heights, body_ext_m=body_ext_m
        )

    def _mean_height_raster(self):
        """
        Raster of the mean heights (height_sum / height_count, NaN where never seen),
        rebuilt only when the map changes. Same division as get_height, so the same
        values to the bit.
        """
        ver = (len(self.height_count), len(getattr(self, 'update_rects', None) or []))
        cached = getattr(self, '_mh_raster', None)
        if cached is not None and cached[0] == ver:
            return cached[1]
        keys = [key for key, c in self.height_count.items() if c > 0]
        if not keys:
            result = None
        else:
            k = np.array(keys, dtype=np.int64)
            s = np.array([self.height_sum[key] for key in keys], dtype=np.float64)
            c = np.array([self.height_count[key] for key in keys], dtype=np.float64)
            gx0, gy0 = int(k[:, 0].min()), int(k[:, 1].min())
            raster = np.full((int(k[:, 0].max()) - gx0 + 1, int(k[:, 1].max()) - gy0 + 1), np.nan)
            raster[k[:, 0] - gx0, k[:, 1] - gy0] = s / c
            result = (raster, gx0, gy0)
        self._mh_raster = (ver, result)
        return result

    def _sample_heights(self, xs, ys):
        """get_height on an array of points (NaN where never seen), same key floor(x / res)."""
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        out = np.full(xs.shape, np.nan)
        rr = self._mean_height_raster()
        if rr is None or xs.size == 0:
            return out
        raster, gx0, gy0 = rr
        gx = np.floor(xs / self.resolution).astype(np.int64) - gx0
        gy = np.floor(ys / self.resolution).astype(np.int64) - gy0
        ok = (gx >= 0) & (gx < raster.shape[0]) & (gy >= 0) & (gy < raster.shape[1])
        out[ok] = raster[gx[ok], gy[ok]]
        return out

    def _has_height_raster(self):
        """Boolean raster of the cells with at least one height (height_count > 0), cached."""
        ver = (len(self.height_count), len(getattr(self, 'update_rects', None) or []))
        cached = getattr(self, '_hh_raster', None)
        if cached is not None and cached[0] == ver:
            return cached[1]
        if not self.height_count:
            result = None
        else:
            k = np.array([key for key, c in self.height_count.items() if c > 0], dtype=np.int64)
            if k.size == 0:
                result = None
            else:
                gx0, gy0 = int(k[:, 0].min()), int(k[:, 1].min())
                raster = np.zeros((int(k[:, 0].max()) - gx0 + 1, int(k[:, 1].max()) - gy0 + 1), dtype=bool)
                raster[k[:, 0] - gx0, k[:, 1] - gy0] = True
                result = (raster, gx0, gy0)
        self._hh_raster = (ver, result)
        return result

    def certainly_no_slope_data(self, x1, y1, x2, y2, min_sample_fraction=MIN_ARC_SAMPLE_FRACTION):
        """
        For many edges together (arrays): True where compute_arc_slope_profile would
        CERTAINLY answer "no data". Reproduces exactly its early exits (empty map, edge
        outside the rectangle of what was seen, start/middle/end without data, line
        coverage below min_sample_fraction) with the same samples. Where it answers False
        the real function is called: the result is identical, only useless calls are avoided.
        """
        x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
        x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
        n = x1.size
        out = np.ones(n, dtype=bool)
        if n == 0:
            return out
        rects = getattr(self, 'update_rects', None)
        if rects is not None and not rects:
            return out
        rr = self._has_height_raster()
        if rr is None:
            return out
        raster, gx0, gy0 = rr
        res = self.resolution

        def has(px, py):
            gx = np.floor(px / res).astype(np.int64) - gx0
            gy = np.floor(py / res).astype(np.int64) - gy0
            ok = (gx >= 0) & (gx < raster.shape[0]) & (gy >= 0) & (gy < raster.shape[1])
            h = np.zeros(px.shape, dtype=bool)
            h[ok] = raster[gx[ok], gy[ok]]
            return h

        dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        three = has(x1, y1) | has(0.5 * (x1 + x2), 0.5 * (y1 + y2)) | has(x2, y2)
        cand = (dist >= 1e-6) & three
        if not cand.any():
            return out
        idx = np.nonzero(cand)[0]
        num_along = np.maximum(2, (dist[idx] / res).astype(np.int64) + 1)
        for m in np.unique(num_along):
            sel = idx[num_along == m]
            t = np.linspace(0.0, 1.0, int(m))
            px = x1[sel, None] + t[None, :] * (x2[sel] - x1[sel])[:, None]
            py = y1[sel, None] + t[None, :] * (y2[sel] - y1[sel])[:, None]
            cov = has(px, py).mean(axis=1)
            out[sel] = cov < min_sample_fraction
        return out

    def save_map(self, file_path):
        """Save the global height map to a pickle file."""
        with open(file_path, 'wb') as f:
            pickle.dump({
                'resolution': self.resolution,
                'height_sum': self.height_sum,
                'height_count': self.height_count,
                # heights in the VISION frame (relative to power-on); subtract this value
                # to have them relative to the ground at mission start. None if unknown.
                'mission_z0': getattr(self, 'mission_z0', None),
            }, f)

# =========================================================================================
# FROM THE SCAN TO THE GLOBAL MAPS, and log lines on the state of grids and maps
# =========================================================================================
# --- Area around the body in the GLOBAL map ---------------------------------------------
# The cells right next to the robot's flank come out as "obstacle" ONLY when the robot is
# beside them: read from 1-2 m they are free, read with the robot standing next to them
# they drop below threshold, 0.2-0.45 m from the axis, at the height of the rear legs. Two
# scans standing still would be enough to confirm them in the global map
# (GLOBAL_OCC_CONFIRM_OBS = 2), and the PRM would discard the edge the robot JUST travelled.
#   - CORE (the body rectangle): the robot stands on it, so it is free by
#     construction -> "seen free" is written.
#   - RING (up to GLOBAL_MAP_SELF_RING_M beyond the body): from here the reading is not
#     reliable -> an obstacle is NOT written (no information); free is.
# Only for the global map: the safe frontier uses the live scan, whole, so
# a real obstacle next to the robot still stops it. And a real obstacle is still
# written in the global map by the scans taken from another point.
GLOBAL_MAP_SELF_BODY_LENGTH_M = 1.10

GLOBAL_MAP_SELF_BODY_WIDTH_M = 0.50

GLOBAL_MAP_SELF_RING_M = 0.25


def mask_for_global_map(obstacle_mask, is_valid, pts=None, robot_pose=None,
                         return_core=False):
    """
    Mask to pass to GlobalGrid.update: -1 obstacle, +1 seen free, 0 no information (cell
    not reliable in this scan, which therefore does not count as "seen free": see
    GlobalGrid.update).

    pts, robot_pose=(x, y, yaw): if given, applies the area around the body (see
    GLOBAL_MAP_SELF_RING_M): free core, ring without obstacles.

    return_core: also return the CORE mask, i.e. the cells covered by the body now. It
    must be passed to GlobalGrid.update as `walked`: they are cells the robot physically
    passed over, and declaring them occupied will require more confirmations (see
    GLOBAL_OCC_CONFIRM_WALKED).
    """
    m = np.asarray(obstacle_mask, dtype=np.float64).ravel()
    v = np.asarray(is_valid, dtype=bool).ravel()
    out = np.where(m < 0, -1.0, np.where(v, 1.0, 0.0))
    if pts is None or robot_pose is None:
        return (out, np.zeros(out.shape, dtype=bool)) if return_core else out
    rx, ry, ryaw = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    if len(p) != len(out):
        return (out, np.zeros(out.shape, dtype=bool)) if return_core else out
    dx, dy = p[:, 0] - rx, p[:, 1] - ry
    c, s_ = np.cos(ryaw), np.sin(ryaw)
    xb, yb = np.abs(dx * c + dy * s_), np.abs(-dx * s_ + dy * c)
    hl, hw = GLOBAL_MAP_SELF_BODY_LENGTH_M / 2.0, GLOBAL_MAP_SELF_BODY_WIDTH_M / 2.0
    core = (xb <= hl) & (yb <= hw)
    ring = ~core & (xb <= hl + GLOBAL_MAP_SELF_RING_M) & (yb <= hw + GLOBAL_MAP_SELF_RING_M)
    out[ring & (out < 0)] = 0.0
    out[core] = 1.0
    return (out, core) if return_core else out


def fold_scan_into_global_maps(global_map, global_terrain_map, pts, terrain, obstacle_mask, is_valid,
                               cells_obstacle_dist, shape, robot_pose):
    """
    One scan into the two global maps: occupancy (with the area around the body, see
    _mask_for_global_map) and terrain. The only entry point used by the mission (attempt
    start, every turn, final scan) and by test_replay.py, so the replay test checks the
    real code and not a copy of it.

    The terrain map receives THE SAME mask as the live computation (slope_usable_mask:
    reliable AND non-obstacle): the heights of obstacle cells would turn logs and walls
    into "slopes" in the global map.

    Returns the core mask (cells under the body), needed by the free-space map.
    """
    gm_mask, gm_core = mask_for_global_map(obstacle_mask, is_valid, pts, robot_pose, return_core=True)
    global_map.update(pts, gm_mask, walked=gm_core)
    global_terrain_map.update(pts, terrain, slope_usable_mask(is_valid, cells_obstacle_dist, shape))
    return gm_core


def update_clearance_map(global_map, pts, snap, is_valid, unwritten, core, robot_xy):
    """
    Feed the free-space map with the current scan.

    Uses the FILTERED obstacle_distance (the one the frontier really decides on, median +
    hysteresis: see safe_frontier), not the raw reading. Only reliable cells learn; under
    the body the value cannot be less than half the width, because the robot stands
    there (same principle as _under_body in the frontier).
    """
    if not hasattr(global_map, 'update_clearance') or snap is None:
        return
    try:
        od = np.asarray(snap.obstacle_dist, dtype=np.float64).ravel()
        v = np.asarray(is_valid, dtype=bool).ravel()
        if unwritten is not None and np.size(unwritten) == v.size:
            v = v & ~np.asarray(unwritten, dtype=bool).ravel()
        c = np.asarray(core, dtype=bool).ravel()
        if od.size != v.size or c.size != v.size:
            return
        usable = v | c
        od = np.where(c, np.maximum(od, ROBOT_HALF_WIDTH_M), od)
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        near = np.hypot(p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1]) <= CLEARANCE_NEAR_M
        global_map.update_clearance(p, od, usable, near=near)
    except Exception as e:      # never fatal: it is a helper map
        print(f"[SPACE] Free-space map not updated: {e}")
