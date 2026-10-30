"""Edge slope: longitudinal and lateral profiles over a 0.48 m base, live and from the terrain map."""
import numpy as np
from spot_explorer.perception.geometry import ROBOT_HALF_LENGTH_M
SLOPE_THRESHOLD = 0.6       # gradient (tan of slope): tan(28deg)=~0.53, tan(25deg)=~0.46
FRONTIER_SLOPE_MIN_LENGTH_M = 0.50   # below this length the slope cannot be estimated

# Weight of the LATERAL slope (roll) relative to the longitudinal one in the edge cost
# (beta_lateral = beta * this). 1.0 = same weight. Does not discard edges: it only serves to
# prefer, between two accepted edges, the one less exposed to roll.
LATERAL_SLOPE_COST_MULTIPLIER = 1.0

# How close to SLOPE_THRESHOLD counts as "near the limit" in the logs and in the diagnostic
# saves. Does not affect any decision.
NEAR_THRESHOLD_MARGIN_FRACTION = 0.15

# =========================================================================
# SLOPE PROFILE OF AN EDGE (see compute_arc_slope_profile)
#   SLOPE_BASELINE_M: HALF base of the height difference. The difference is centred, k cells
#     ahead and k behind with k = round(0.25 / 0.03) = 8: the drop is measured over ~0.48 m,
#     about one step. On a shorter base the face of a piece of furniture would give huge
#     slopes; on this one a sharp step of H counts as H / 0.48.
#   LATERAL_SLICE_HALF_WIDTH_M: half width of the perpendicular slice for the roll:
#     as wide as the robot (0.50 m) plus margin. Beyond that band it is not roll.
#   MIN_ARC_SAMPLE_FRACTION: minimum fraction of samples with data for the edge to have a
#     slope (has_data); below it, the edge is "without data", which does not mean flat.
#   ARC_BODY_EXTENSION_M: how much the edge is extended at EACH end for the slope "with the
#     body", used for veto, cost and gait: the robot standing on a node has its legs 0.55 m
#     ahead and behind, so it feels the terrain beyond the two nodes. The frontier stays on
#     the edge between the nodes (more permissive than the PRM). 0 = the edge only.
# =========================================================================
SLOPE_BASELINE_M = 0.25
LATERAL_SLICE_HALF_WIDTH_M = 0.30
MIN_ARC_SAMPLE_FRACTION = 0.5
ARC_BODY_EXTENSION_M = ROBOT_HALF_LENGTH_M


def _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size, xs, ys, valid_2d=None):
    """
    Grid heights at positions (xs, ys), arrays of any shape.

    Returns (heights, inside): where there is no data the height is NaN and the sample
    stays in its place (it is not compacted away, which would invent slopes between points
    that are not close). With valid_2d the samples on unreliable cells are NaN like those
    outside the grid, and `inside` means "inside AND reliable". Uses np.floor and not int(),
    which gets the cell wrong for negative coordinates.
    """
    num_rows, num_cols = terrain_2d.shape
    cols = np.floor((xs - grid_origin_x) / cell_size).astype(np.int64)
    rows = np.floor((ys - grid_origin_y) / cell_size).astype(np.int64)
    inside = (rows >= 0) & (rows < num_rows) & (cols >= 0) & (cols < num_cols)
    out = np.full(np.shape(xs), np.nan, dtype=np.float64)
    if valid_2d is not None:
        ok = inside.copy()
        ok[inside] = np.asarray(valid_2d, dtype=bool)[rows[inside], cols[inside]]
        inside = ok
    if np.any(inside):
        out[inside] = terrain_2d[rows[inside], cols[inside]]
    return out, inside


def p90(values):
    """
    90th percentile, IDENTICAL to float(np.percentile(values, 90)) (linear interpolation)
    for finite values, but without the fixed cost of np.percentile, which is called very
    often here.
    """
    a = np.asarray(values, dtype=np.float64).ravel()
    n = a.size
    if n == 1:
        return float(a[0])
    v = (n - 1) * 0.9
    lo = int(np.floor(v))
    hi = min(lo + 1, n - 1)
    part = np.partition(a, (lo, hi)) if hi != lo else np.partition(a, lo)
    x0, x1 = float(part[lo]), float(part[hi])
    t = v - lo
    d = x1 - x0
    return x1 - d * (1.0 - t) if t >= 0.5 else x0 + d * t


def _baseline_gradient(heights, step, baseline_m):
    """
    Slope as a centred difference over a PHYSICAL base (see SLOPE_BASELINE_M), along the
    last axis: fine for a 1D profile (longitudinal) and for a stack of 2D slices (lateral),
    which are pooled. Where a sample is missing the result is NaN and is discarded. Returns
    the 1D array of the valid slope magnitudes (possibly empty).
    """
    n = heights.shape[-1]
    k = max(1, int(round(baseline_m / step)))
    if n <= 2 * k:
        k = max(1, (n - 1) // 2)
    if n <= 2 * k:
        return np.empty(0)
    g = np.abs(heights[..., 2 * k:] - heights[..., :-2 * k]) / (2 * k * step)
    return g[np.isfinite(g)]


def compute_arc_slope_profile(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                              x1, y1, x2, y2, perp_sample_spacing_m=0.10,
                              baseline_m=SLOPE_BASELINE_M,
                              lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M,
                              min_sample_fraction=MIN_ARC_SAMPLE_FRACTION, valid_2d=None,
                              obstacle_2d=None, body_ext_m=0.0):
    """
    Slope Spot would feel travelling the edge (x1, y1) -> (x2, y2), measured on the heights
    of the given grid. Pure function of the data: used by the PRM (on the current scan) and
    by the frontier (on its snapshot).

      - LONGITUDINAL (pitch): heights every cell along the edge line, differences over
        SLOPE_BASELINE_M (see _baseline_gradient).
      - LATERAL (roll): every perp_sample_spacing_m along the edge a perpendicular slice as
        wide as the robot (+/- lateral_half_width_m), with the same differences; the samples
        of all the slices go together.

    Each of the two is summarized with the 90th percentile: an isolated wrong cell does not
    count, but a steep stretch is not watered down by the average of the edge.

    has_data is True only if at least min_sample_fraction of the samples have data, in both
    directions: an edge seen for less than half is "without data", which does not mean
    flat (the PRM then moves on to the global map).

    valid_2d: mask of the reliable cells; the others (filled, filtered, never written) do
    not count. obstacle_2d: cells already obstacle by obstacle_distance, excluded too: the
    face of a piece of furniture is not a slope, and is already handled as an obstacle by
    the PRM and the frontier. An edge that loses its data this way becomes "without data",
    not "free": it enters the graph with a fallback cost and whoever travels it checks it
    with the frontier.

    body_ext_m > 0: additionally the three values "with the body" (see _body_slope).

    Returns:
        (long, lat, has_data), or with body_ext_m > 0
        (long, lat, has_data, long with body, lat with body, has with body)
    """
    if terrain_2d is None or grid_origin_x is None or cell_size is None:
        return _no_slope_data(body_ext_m)

    # x*x and not **2: it is the operation numpy does on arrays, so the length (and from
    # there step and slope) is bit-for-bit identical to that of the batch computation
    # (arc_slope_profiles_batch); **2 on a single number sometimes differs in the last digit.
    dist = float(np.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))
    if dist < 1e-6:
        return _no_slope_data(body_ext_m)

    # Quick exit: if the edge, widened by the lateral slice, does not touch the grid, no
    # sample falls inside it -> "without data", identical to the full computation but without
    # paying for it (most edges lie outside the 3.84 m window).
    num_rows, num_cols = terrain_2d.shape
    pad = lateral_half_width_m + cell_size
    if (max(x1, x2) + pad < grid_origin_x or min(x1, x2) - pad > grid_origin_x + num_cols * cell_size or
            max(y1, y2) + pad < grid_origin_y or min(y1, y2) - pad > grid_origin_y + num_rows * cell_size):
        return _no_slope_data(body_ext_m)

    dir_x, dir_y = (x2 - x1) / dist, (y2 - y1) / dist   # along-arc unit vector
    perp_x, perp_y = -dir_y, dir_x                      # across-arc unit vector

    # Reliable AND non-obstacle cells: a single mask for both directions.
    usable_2d = valid_2d
    if obstacle_2d is not None:
        obs = np.asarray(obstacle_2d, dtype=bool)
        usable_2d = (~obs) if valid_2d is None else (np.asarray(valid_2d, dtype=bool) & ~obs)

    # --- LONGITUDINAL: along the edge line ---
    num_along = max(2, int(dist / cell_size) + 1)
    t = np.linspace(0.0, 1.0, num_along)
    step_along = dist / (num_along - 1)
    h_along, inside_along = _gather_heights(
        terrain_2d, grid_origin_x, grid_origin_y, cell_size,
        x1 + t * (x2 - x1), y1 + t * (y2 - y1), valid_2d=usable_2d
    )
    grad_long = _baseline_gradient(h_along, step_along, baseline_m)
    longitudinal_slope = p90(grad_long) if grad_long.size else 0.0

    # --- LATERAL: perpendicular slices as wide as the ROBOT, not as the edge ---
    num_slices = max(1, int(dist / perp_sample_spacing_m) + 1)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    step_perp = 2 * lateral_half_width_m / (num_perp - 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    ts = np.linspace(0.0, 1.0, num_slices) if num_slices > 1 else np.array([0.5])
    xs_perp = (x1 + ts * (x2 - x1))[:, None] + u[None, :] * perp_x
    ys_perp = (y1 + ts * (y2 - y1))[:, None] + u[None, :] * perp_y
    h_perp, inside_perp = _gather_heights(
        terrain_2d, grid_origin_x, grid_origin_y, cell_size, xs_perp, ys_perp, valid_2d=usable_2d
    )
    grad_lat = _baseline_gradient(h_perp, step_perp, baseline_m)
    lateral_slope = p90(grad_lat) if grad_lat.size else 0.0

    has_data = (float(inside_along.mean()) >= min_sample_fraction
                and float(inside_perp.mean()) >= min_sample_fraction
                and grad_long.size > 0 and grad_lat.size > 0)

    base = (longitudinal_slope, lateral_slope, has_data)
    if body_ext_m <= 0:
        return base
    gather = lambda xs, ys: _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                            xs, ys, valid_2d=usable_2d)
    return base + _body_slope(gather, base, x1, y1, x2, y2, dir_x, dir_y, perp_x, perp_y,
                              h_along, inside_along, step_along, grad_lat, inside_perp, u,
                              step_perp, num_slices, dist, perp_sample_spacing_m, baseline_m,
                              min_sample_fraction, body_ext_m)


# =========================================================================
# SLOPE OF MANY EDGES TOGETHER
#
# Same computation as compute_arc_slope_profile / _slope_profile_from_sampler / _body_slope,
# done with matrices: one row per edge. Edges are grouped by number of samples (which
# depends only on the length: on the lattice there are only a few dozen lengths), and for
# each group positions, height readings, differences and 90th percentile are done in one
# go. The operations are the same and in the same order, so the numbers are bit-for-bit
# IDENTICAL to the edge-by-edge ones: it must stay that way (test_replay.py checks it).
# =========================================================================
def p90_rows(m):
    """Row-by-row p90 of a matrix, ignoring NaNs; IDENTICAL to p90 on the finite values of
    each row. Returns (values, how many finite); 0.0 where the row has no finite values."""
    m = np.asarray(m, dtype=np.float64)
    if m.ndim == 1:
        m = m[None, :]
    cnt = np.isfinite(m).sum(axis=1)
    out = np.zeros(m.shape[0], dtype=np.float64)
    ok = cnt > 0
    if not ok.any() or m.shape[1] == 0:
        return out, cnt
    srt = np.sort(m[ok], axis=1)                      # NaNs go to the end
    n = cnt[ok]
    v = (n - 1) * 0.9
    lo = np.floor(v).astype(np.int64)
    hi = np.minimum(lo + 1, n - 1)
    r = np.arange(srt.shape[0])
    x0, x1 = srt[r, lo], srt[r, hi]
    t = v - lo
    d = x1 - x0
    out[ok] = np.where(t >= 0.5, x1 - d * (1.0 - t), x0 + d * t)
    return out, cnt


def _baseline_gradient_rows(h, step, baseline_m):
    """_baseline_gradient on a group of rows with the SAME k (step: one value per row).
    Returns the slope matrix with NaN where a sample is missing (no compaction: p90_rows
    takes care of it), or None if the row is too short."""
    n = h.shape[-1]
    k = max(1, int(round(baseline_m / float(step[0]))))
    if n <= 2 * k:
        k = max(1, (n - 1) // 2)
    if n <= 2 * k:
        return None
    den = (2 * k * step).reshape((-1,) + (1,) * (h.ndim - 1))
    return np.abs(h[..., 2 * k:] - h[..., :-2 * k]) / den


def _k_of(baseline_m, step):
    """The k of _baseline_gradient for each row (used for grouping)."""
    return np.array([max(1, int(round(baseline_m / float(s)))) for s in step], dtype=np.int64)


def arc_slope_profiles_batch(sample, x1, y1, x2, y2, cell_size, perp_sample_spacing_m=0.10,
                             baseline_m=SLOPE_BASELINE_M,
                             lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M,
                             min_sample_fraction=MIN_ARC_SAMPLE_FRACTION, body_ext_m=0.0,
                             sampler_mode=False):
    """
    Slope profile of many edges together. sample(xs, ys) -> (heights, inside) on arrays of
    any shape (NaN / False where there is no data).

    sampler_mode=False: like compute_arc_slope_profile (live grid).
    sampler_mode=True:  like _slope_profile_from_sampler (global map): same early exits --
        the three points start/middle/end without data, centre-line coverage below
        min_sample_fraction -- which give "no data" with slopes 0.

    Returns an (N, 6) array: long, lat, has, long with body, lat with body, has with body
    (has as 0.0 / 1.0). Without body_ext_m the last three columns repeat the first three.
    """
    x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
    x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
    N = x1.size
    out = np.zeros((N, 6), dtype=np.float64)
    if N == 0:
        return out
    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    live = dist >= 1e-6
    if sampler_mode:
        h3, _ = sample(np.stack([x1, 0.5 * (x1 + x2), x2], -1), np.stack([y1, 0.5 * (y1 + y2), y2], -1))
        live &= np.isfinite(h3).any(axis=1)
    idx_all = np.nonzero(live)[0]
    if idx_all.size == 0:
        return out
    d = dist[idx_all]
    num_along = np.maximum(2, np.floor(d / cell_size).astype(np.int64) + 1)
    num_slices = np.maximum(1, np.floor(d / perp_sample_spacing_m).astype(np.int64) + 1)
    step_along = d / (num_along - 1)
    k_along = _k_of(baseline_m, step_along)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    step_perp = 2 * lateral_half_width_m / (num_perp - 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    if body_ext_m > 0:
        kb = np.array([max(1, int(round(body_ext_m / float(s)))) for s in step_along], dtype=np.int64)
        ds = np.where(num_slices > 1, d / np.maximum(num_slices - 1, 1), perp_sample_spacing_m)
        kl = np.array([max(1, int(round(body_ext_m / float(s)))) for s in ds], dtype=np.int64)
    else:
        kb = kl = np.zeros(idx_all.size, dtype=np.int64)
        ds = np.zeros(idx_all.size)
    keys = np.stack([num_along, num_slices, k_along, kb, kl], axis=1)
    _, group = np.unique(keys, axis=0, return_inverse=True)
    group = np.asarray(group).ravel()
    for g in range(int(group.max()) + 1):
        sel = np.nonzero(group == g)[0]
        rows = idx_all[sel]
        n_al, n_sl = int(num_along[sel[0]]), int(num_slices[sel[0]])
        ax1, ay1, ax2, ay2 = x1[rows], y1[rows], x2[rows], y2[rows]
        dd = d[sel]
        st = step_along[sel]
        dir_x, dir_y = (ax2 - ax1) / dd, (ay2 - ay1) / dd
        perp_x, perp_y = -dir_y, dir_x
        # --- longitudinal
        t = np.linspace(0.0, 1.0, n_al)
        h_al, in_al = sample(ax1[:, None] + t[None, :] * (ax2 - ax1)[:, None],
                             ay1[:, None] + t[None, :] * (ay2 - ay1)[:, None])
        cov_al = in_al.mean(axis=1)
        go = np.ones(rows.size, dtype=bool)
        if sampler_mode:
            go = cov_al >= min_sample_fraction
        g_long = _baseline_gradient_rows(h_al, st, baseline_m)
        if g_long is None:
            lo, n_lo = np.zeros(rows.size), np.zeros(rows.size, dtype=np.int64)
        else:
            lo, n_lo = p90_rows(g_long)
        # --- lateral
        ts = np.linspace(0.0, 1.0, n_sl) if n_sl > 1 else np.array([0.5])
        cx = ax1[:, None] + ts[None, :] * (ax2 - ax1)[:, None]
        cy = ay1[:, None] + ts[None, :] * (ay2 - ay1)[:, None]
        h_pp, in_pp = sample(cx[:, :, None] + u[None, None, :] * perp_x[:, None, None],
                             cy[:, :, None] + u[None, None, :] * perp_y[:, None, None])
        g_lat = _baseline_gradient_rows(h_pp, np.full(rows.size, step_perp), baseline_m)
        if g_lat is None:
            g_lat_flat = np.zeros((rows.size, 0))
        else:
            g_lat_flat = g_lat.reshape(rows.size, -1)
        la, n_la = p90_rows(g_lat_flat)
        cov_pp = in_pp.reshape(rows.size, -1).mean(axis=1)
        has = (cov_al >= min_sample_fraction) & (cov_pp >= min_sample_fraction) & (n_lo > 0) & (n_la > 0)
        base = np.stack([lo, la, has.astype(np.float64)], axis=1)
        base[~go] = 0.0
        has &= go
        res = np.concatenate([base, base], axis=1)
        if body_ext_m > 0 and has.any():
            k_b, k_l = int(kb[sel[0]]), int(kl[sel[0]])
            off = np.arange(1, k_b + 1)[None, :] * st[:, None]
            xs = np.concatenate((ax1[:, None] - dir_x[:, None] * off[:, ::-1], ax2[:, None] + dir_x[:, None] * off), axis=1)
            ys = np.concatenate((ay1[:, None] - dir_y[:, None] * off[:, ::-1], ay2[:, None] + dir_y[:, None] * off), axis=1)
            h_ex, in_ex = sample(xs, ys)
            h_full = np.concatenate((h_ex[:, :k_b], h_al, h_ex[:, k_b:]), axis=1)
            cov_b = (in_ex.sum(axis=1).astype(np.float64) + in_al.sum(axis=1).astype(np.float64)) / (2 * k_b + n_al)
            gb_long = _baseline_gradient_rows(h_full, st, baseline_m)
            offl = np.arange(1, k_l + 1)[None, :] * ds[sel][:, None]
            bx = np.concatenate((ax1[:, None] - dir_x[:, None] * offl[:, ::-1], ax2[:, None] + dir_x[:, None] * offl), axis=1)
            by = np.concatenate((ay1[:, None] - dir_y[:, None] * offl[:, ::-1], ay2[:, None] + dir_y[:, None] * offl), axis=1)
            h_l, in_l = sample(bx[:, :, None] + u[None, None, :] * perp_x[:, None, None],
                               by[:, :, None] + u[None, None, :] * perp_y[:, None, None])
            gl_ext = _baseline_gradient_rows(h_l, np.full(rows.size, step_perp), baseline_m)
            parts = [g_lat_flat] + ([gl_ext.reshape(rows.size, -1)] if gl_ext is not None else [])
            gb_lat = np.concatenate(parts, axis=1)
            cov_lb = ((in_l.reshape(rows.size, -1).sum(axis=1).astype(np.float64)
                       + in_pp.reshape(rows.size, -1).sum(axis=1).astype(np.float64))
                      / (in_l[0].size + in_pp[0].size))
            if gb_long is None:
                blo, n_blo = np.zeros(rows.size), np.zeros(rows.size, dtype=np.int64)
            else:
                blo, n_blo = p90_rows(gb_long)
            bla, n_bla = p90_rows(gb_lat)
            ok = (has & (cov_b >= min_sample_fraction) & (cov_lb >= min_sample_fraction)
                  & (n_blo > 0) & (n_bla > 0))
            res[ok, 3] = np.maximum(lo[ok], blo[ok])
            res[ok, 4] = np.maximum(la[ok], bla[ok])
            res[ok, 5] = 1.0
        out[rows] = res
    return out


def compute_arc_slope_profiles_batch(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                     x1, y1, x2, y2, valid_2d=None, body_ext_m=0.0, **kw):
    """compute_arc_slope_profile for many edges (arrays), from the live grid. See
    arc_slope_profiles_batch for the result format."""
    n = np.size(x1)
    if terrain_2d is None or grid_origin_x is None or cell_size is None:
        return np.zeros((n, 6))
    x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
    x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
    out = np.zeros((n, 6))
    num_rows, num_cols = terrain_2d.shape
    pad = kw.get('lateral_half_width_m', LATERAL_SLICE_HALF_WIDTH_M) + cell_size
    near = ~((np.maximum(x1, x2) + pad < grid_origin_x) | (np.minimum(x1, x2) - pad > grid_origin_x + num_cols * cell_size)
             | (np.maximum(y1, y2) + pad < grid_origin_y) | (np.minimum(y1, y2) - pad > grid_origin_y + num_rows * cell_size))
    if near.any():
        sample = lambda xs, ys: _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                               xs, ys, valid_2d=valid_2d)
        out[near] = arc_slope_profiles_batch(sample, x1[near], y1[near], x2[near], y2[near], cell_size,
                                             body_ext_m=body_ext_m, **kw)
    return out


def _no_slope_data(body_ext_m):
    """"No data" result: 3 values, or 6 if the measurement with the body was also requested."""
    return (0.0, 0.0, False) if body_ext_m <= 0 else (0.0, 0.0, False, 0.0, 0.0, False)


def _body_slope(gather, base, x1, y1, x2, y2, dir_x, dir_y, perp_x, perp_y,
                h_along, inside_along, step_along, grad_lat, inside_perp, u, step_perp,
                num_slices, dist, perp_spacing, baseline_m, min_sample_fraction, ext_m):
    """
    Slope "with the body": like the edge's, but on the stretch extended by ext_m at both
    ends (see ARC_BODY_EXTENSION_M). Reuses the samples already read for the edge and reads
    only the added pieces, at the same step (ext rounded to a multiple of the step:
    +/- 1.5 cm along the edge, +/- 5 cm for the lateral slices).

    Returns (long, lat, has) already taken as the WORSE between edge and extended edge, so
    the added stretch cannot dilute a climb that lies inside the edge. If the edge has no
    data, or the extended stretch does not have enough, returns the edge values.
    """
    lo, la, has = base
    if not has:
        return base
    # Longitudinal: continuous profile piece before + edge + piece after.
    k = max(1, int(round(ext_m / step_along)))
    off = np.arange(1, k + 1) * step_along
    xs = np.concatenate((x1 - dir_x * off[::-1], x2 + dir_x * off))
    ys = np.concatenate((y1 - dir_y * off[::-1], y2 + dir_y * off))
    h_ext, in_ext = gather(xs, ys)
    h_full = np.concatenate((h_ext[:k], h_along, h_ext[k:]))
    cov_along = (float(np.count_nonzero(in_ext)) + float(np.count_nonzero(inside_along))) / (2 * k + h_along.size)
    g_long = _baseline_gradient(h_full, step_along, baseline_m)
    # Lateral: slices added at the same step as the edge slices.
    ds = dist / (num_slices - 1) if num_slices > 1 else perp_spacing
    kl = max(1, int(round(ext_m / ds)))
    offl = np.arange(1, kl + 1) * ds
    cx = np.concatenate((x1 - dir_x * offl[::-1], x2 + dir_x * offl))
    cy = np.concatenate((y1 - dir_y * offl[::-1], y2 + dir_y * offl))
    h_l, in_l = gather(cx[:, None] + u[None, :] * perp_x, cy[:, None] + u[None, :] * perp_y)
    g_lat = np.concatenate((grad_lat, _baseline_gradient(h_l, step_perp, baseline_m)))
    cov_lat = ((float(np.count_nonzero(in_l)) + float(np.count_nonzero(inside_perp)))
               / (in_l.size + np.size(inside_perp)))
    if (cov_along < min_sample_fraction or cov_lat < min_sample_fraction
            or g_long.size == 0 or g_lat.size == 0):
        return base
    return max(lo, p90(g_long)), max(la, p90(g_lat)), True

def _slope_profile_from_sampler(sample_height, x1, y1, x2, y2, cell_size,
                                perp_sample_spacing_m=0.10,
                                baseline_m=SLOPE_BASELINE_M,
                                lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M,
                                min_sample_fraction=MIN_ARC_SAMPLE_FRACTION,
                                sample_array=None, body_ext_m=0.0):
    """
    Slope of an edge on a SPARSE map (the global terrain map, see
    GlobalTerrainGrid.compute_arc_slope_profile). sample_height(px, py) gives the height at
    a point or None; sample_array does the same on an array (NaN where missing).

    Same geometry and same base as compute_arc_slope_profile: the two sources must measure
    the same quantity, otherwise an edge would change slope entering or leaving the scanned
    window (test_replay.py checks it). has_data tells whether there are enough REAL samples.

    Returns:
        (longitudinal_slope, lateral_slope, has_data)
    """
    # x*x and not **2: it is the operation numpy does on arrays, so the length (and from
    # there step and slope) is bit-for-bit identical to that of the batch computation
    # (arc_slope_profiles_batch); **2 on a single number sometimes differs in the last digit.
    dist = float(np.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))
    if dist < 1e-6:
        return _no_slope_data(body_ext_m)

    dir_x, dir_y = (x2 - x1) / dist, (y2 - y1) / dist
    perp_x, perp_y = -dir_y, dir_x

    # Early exit: if start, middle and end of the edge have no data, "without data"
    # (most edges lie outside everything that has been seen). If at least
    # one answers the full computation is done, and min_sample_fraction decides.
    if sample_array is not None:
        h3 = sample_array(np.array([x1, 0.5 * (x1 + x2), x2]), np.array([y1, 0.5 * (y1 + y2), y2]))
        if not np.isfinite(h3).any():
            return _no_slope_data(body_ext_m)
    elif (sample_height(x1, y1) is None
            and sample_height(0.5 * (x1 + x2), 0.5 * (y1 + y2)) is None
            and sample_height(x2, y2) is None):
        return _no_slope_data(body_ext_m)

    def _sample_array(xs, ys):
        """Sample an array of positions, NaN where the map has no data."""
        if sample_array is not None:
            return sample_array(xs, ys)
        flat_x = np.ravel(xs)
        flat_y = np.ravel(ys)
        out = np.empty(flat_x.size, dtype=np.float64)
        for i in range(flat_x.size):
            h = sample_height(flat_x[i], flat_y[i])
            out[i] = np.nan if h is None else float(h)
        return out.reshape(np.shape(xs))

    # --- LONGITUDINAL ---
    num_along = max(2, int(dist / cell_size) + 1)
    t = np.linspace(0.0, 1.0, num_along)
    step_along = dist / (num_along - 1)
    h_along = _sample_array(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    cov_along = float(np.isfinite(h_along).mean())

    # The lateral costs ~6 times the longitudinal (one slice every 10 cm, each
    # as wide as the robot). If the longitudinal coverage is already insufficient,
    # has_data would be False anyway: no point paying for it.
    if cov_along < min_sample_fraction:
        return _no_slope_data(body_ext_m)

    grad_long = _baseline_gradient(h_along, step_along, baseline_m)
    longitudinal_slope = p90(grad_long) if grad_long.size else 0.0

    # --- LATERAL ---
    num_slices = max(1, int(dist / perp_sample_spacing_m) + 1)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    step_perp = 2 * lateral_half_width_m / (num_perp - 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    ts = np.linspace(0.0, 1.0, num_slices) if num_slices > 1 else np.array([0.5])
    xs_perp = (x1 + ts * (x2 - x1))[:, None] + u[None, :] * perp_x
    ys_perp = (y1 + ts * (y2 - y1))[:, None] + u[None, :] * perp_y
    h_perp = _sample_array(xs_perp, ys_perp)
    cov_lat = float(np.isfinite(h_perp).mean())
    grad_lat = _baseline_gradient(h_perp, step_perp, baseline_m)
    lateral_slope = p90(grad_lat) if grad_lat.size else 0.0

    has_data = (cov_along >= min_sample_fraction and cov_lat >= min_sample_fraction
                and grad_long.size > 0 and grad_lat.size > 0)

    base = (longitudinal_slope, lateral_slope, has_data)
    if body_ext_m <= 0:
        return base

    def gather(xs, ys):
        h = _sample_array(xs, ys)
        return h, np.isfinite(h)
    return base + _body_slope(gather, base, x1, y1, x2, y2, dir_x, dir_y, perp_x, perp_y,
                              h_along, np.isfinite(h_along), step_along, grad_lat,
                              np.isfinite(h_perp), u, step_perp, num_slices, dist,
                              perp_sample_spacing_m, baseline_m, min_sample_fraction, body_ext_m)
