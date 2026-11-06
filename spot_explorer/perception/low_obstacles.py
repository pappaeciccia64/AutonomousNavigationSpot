"""
LOW INTERMITTENT OBSTACLES (grass): Spot's obstacle_distance filtered on the history of each
point of the world.

On 04-11 14:08 (lawn) the cells Spot marked as obstacle, observed at least 5 times from
0.6-1.9 m, were obstacle in this share of their observations (median): 17% below 10 cm above
the ground, 45% at 10-30 cm, 87% at 30-60 cm, 100% above 60 cm (92% of those in at least 80%
of the observations). The low ones come and go from one scan to the next, so edges and
frontier said "free", then "blocked" as the robot got closer, then "free" again.

The rule, cell by cell of the scan:
  - an obstacle cell (obstacle_distance <= 0) HIGHER than LOW_OBSTACLE_HEIGHT_M above the local
    ground, or without a reliable height (walls, whose terrain correct_terrain fills with the
    ground), always counts, at the first sight;
  - a LOW obstacle cell counts if that point of the world was obstacle in at least
    LOW_OBSTACLE_KEEP_FRAC of its last LOW_OBSTACLE_HISTORY observations, or if it has been
    observed fewer than LOW_OBSTACLE_MIN_OBS times (new: as before), or if it was obstacle in
    its last two observations in a row (just appeared: counts with the frontier's latency);
  - a low cell within LOW_OBSTACLE_NEAR_KEPT_M of an obstacle cell that counts (high or
    steady) stays: the edge of a real obstacle, also one that reads the floor height (under
    a table). Without it the gap of (1,3) on 04-11 11:38 was freed, and on 16:10 the edge of a
    piece of furniture in (1,0). So only isolated specks are dropped (grass in the open);
  - the others are dropped, and obstacle_distance is recomputed without them: never below
    Spot's value, never beyond what the window lets us check (distance to its border).
Local ground: the minimum of the reliable raw heights within LOW_OBSTACLE_GROUND_HALF_M.

filter_obstacle_distance is called on every scan (process_scan and the frontier thread):
the history is shared, the scans come from the same robot fractions of a second apart.
"""
import threading

import numpy as np
from scipy.ndimage import distance_transform_edt, minimum_filter

LOW_OBSTACLE_FILTER = True
LOW_OBSTACLE_HEIGHT_M = 0.30
LOW_OBSTACLE_HISTORY = 10          # observations kept per point (bits)
LOW_OBSTACLE_MIN_OBS = 4
LOW_OBSTACLE_KEEP_FRAC = 0.6
LOW_OBSTACLE_GROUND_HALF_M = 0.30
LOW_OBSTACLE_NEAR_KEPT_M = 0.30     # a low cell this close to an obstacle cell that counts stays

_POP = np.array([bin(i).count('1') for i in range(1 << LOW_OBSTACLE_HISTORY)], dtype=np.uint8)


class _History:
    """Per point of the world (key floor(centre / cell)): the last observations as bits
    (1 = obstacle) and how many there are, on rasters grown as the robot moves."""
    PAD = 128

    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        self.cs = None
        self.ix0 = self.iy0 = 0
        self.bits = np.zeros((0, 0), dtype=np.uint16)
        self.n = np.zeros((0, 0), dtype=np.uint8)
        self.dropped_last = 0

    def _ensure(self, ix_min, ix_max, iy_min, iy_max):
        ny, nx = self.bits.shape
        if nx and self.ix0 <= ix_min and ix_max < self.ix0 + nx and self.iy0 <= iy_min and iy_max < self.iy0 + ny:
            return
        x0 = (min(ix_min, self.ix0) if nx else ix_min) - self.PAD
        y0 = (min(iy_min, self.iy0) if nx else iy_min) - self.PAD
        x1 = (max(ix_max, self.ix0 + nx - 1) if nx else ix_max) + self.PAD
        y1 = (max(iy_max, self.iy0 + ny - 1) if nx else iy_max) + self.PAD
        bits = np.zeros((y1 - y0 + 1, x1 - x0 + 1), dtype=np.uint16)
        n = np.zeros_like(bits, dtype=np.uint8)
        if nx:
            bits[self.iy0 - y0:self.iy0 - y0 + ny, self.ix0 - x0:self.ix0 - x0 + nx] = self.bits
            n[self.iy0 - y0:self.iy0 - y0 + ny, self.ix0 - x0:self.ix0 - x0 + nx] = self.n
        self.bits, self.n, self.ix0, self.iy0 = bits, n, x0, y0


HISTORY = _History()


def filter_obstacle_distance(od, terrain_raw, terrain_seen, num_x, num_y, origin_x, origin_y, cell_size):
    """
    obstacle_distance (flat, as Spot gives it) without the low intermittent obstacle cells,
    same shape and dtype. terrain_raw: the raw heights of the scan (flat); terrain_seen: the
    cells with a reliable raw height (flat bool). Updates the history. Returns the input
    itself when the switch is off or nothing is dropped.
    """
    if not LOW_OBSTACLE_FILTER:
        return od
    shape = (int(num_y), int(num_x))
    od_in = od
    od = np.asarray(od, dtype=np.float64).reshape(shape)
    cs = float(cell_size)
    core = od <= 0.0
    seen = np.asarray(terrain_seen, dtype=bool).reshape(shape)
    observed = seen | core
    h = np.asarray(terrain_raw, dtype=np.float64).reshape(shape)
    hs = np.where(seen & np.isfinite(h), h, np.inf)
    k = max(1, int(round(LOW_OBSTACLE_GROUND_HALF_M / cs)))
    ground = minimum_filter(hs, size=2 * k + 1, mode='constant', cval=np.inf)
    low = core & seen & np.isfinite(h) & np.isfinite(ground) & (h - ground < LOW_OBSTACLE_HEIGHT_M)

    ix = int(np.floor(float(origin_x) / cs + 0.5))
    iy = int(np.floor(float(origin_y) / cs + 0.5))
    H = HISTORY
    with H.lock:
        if H.cs is None or abs(H.cs - cs) > 1e-6:
            H.reset()
            H.cs = cs
        H._ensure(ix, ix + shape[1] - 1, iy, iy + shape[0] - 1)
        r0, c0 = iy - H.iy0, ix - H.ix0
        bits = H.bits[r0:r0 + shape[0], c0:c0 + shape[1]]
        n = H.n[r0:r0 + shape[0], c0:c0 + shape[1]]
        mask = (1 << LOW_OBSTACLE_HISTORY) - 1
        new_bits = ((bits.astype(np.uint32) << 1) | core.astype(np.uint32)) & mask
        bits[observed] = new_bits[observed].astype(np.uint16)
        n[observed] = np.minimum(n[observed].astype(np.int32) + 1, LOW_OBSTACLE_HISTORY).astype(np.uint8)
        frac = _POP[bits] / np.maximum(n, 1)
        # Obstacle in the last two observations in a row: something that has just appeared
        # (a box put down where it was free) counts at once, with the latency the frontier
        # already has (median over 3 scans); grass comes and goes.
        steady = (bits & 0b11) == 0b11
        drop = low & (n >= LOW_OBSTACLE_MIN_OBS) & (frac < LOW_OBSTACLE_KEEP_FRAC) & ~steady
        if drop.any():
            # Not next to an obstacle cell that counts (high, or steady): the edge of a real
            # obstacle stays as it is, also when all of it reads the floor height (under a
            # table or a bench: 04-11 16:10, cell (1,0), its edge was dropped up to 0.20 m
            # from the steady cells and the frontier let the robot on 0.65 m instead of 0.29).
            stay = core & ~drop
            if stay.any():
                drop &= distance_transform_edt(~stay) * cs > LOW_OBSTACLE_NEAR_KEPT_M
        H.dropped_last = int(drop.sum())
    if not drop.any():
        return od_in
    keep = core & ~drop
    big = 1e3
    d_keep = distance_transform_edt(~keep) * cs if keep.any() else np.full(shape, big)
    d_all = distance_transform_edt(~core) * cs
    rows, cols = np.indices(shape)
    border = np.minimum(np.minimum(cols + 0.5, shape[1] - cols - 0.5),
                        np.minimum(rows + 0.5, shape[0] - rows - 0.5)) * cs
    # Only where the nearest obstacle was a dropped cell: elsewhere Spot's value stays.
    changed = d_keep > d_all + 1e-9
    out = np.where(changed, np.maximum(od, np.minimum(d_keep, border)), od)
    return out.astype(np.asarray(od_in).dtype, copy=False).reshape(np.shape(od_in))
