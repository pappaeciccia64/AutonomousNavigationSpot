"""Safe frontier: how far the planned path is confirmed free NOW, on the filtered scan; the thread that recomputes it during the move."""
import threading
import time
import logging
import numpy as np
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME
from spot_explorer import config as P
from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.robot import connection as spotUtils
from spot_explorer.perception import geometry
from spot_explorer.perception import slopes
from spot_explorer.perception import spot_grid


LOGGER = logging.getLogger(__name__)


# =========================================================================
# TEMPORAL FILTER ON obstacle_distance
#
# obstacle_distance fluctuates by a few centimetres between two scans even with the robot
# still (1.5 cm on average, 7.3 cm peak), i.e. as much as the air margin it is compared
# with: without a filter a path grazing an obstacle flips from "free" to "blocked"
# and back at every scan. Two remedies:
#   - MEDIAN over the last OBSTACLE_MEDIAN_FRAMES readings of the same world cell: an
#     isolated spike does not decide alone. Costs at most one scan of delay.
#   - HYSTERESIS: a cell becomes blocking below FRONTIER_CLEARANCE_M and becomes free again
#     only above FRONTIER_CLEARANCE_M + OBSTACLE_HYSTERESIS_M.
#
# The origins of Spot's grids are integer multiples of the cell, so realigning two
# scans is an index shift, without interpolation. If the shift is not
# integer or the cell size changes, the history is reset.
# =========================================================================
OBSTACLE_MEDIAN_FRAMES = 3
OBSTACLE_HYSTERESIS_M = 0.04


class _ObstacleHistory:
    """
    History of obstacle_distance aligned to the world, shared between the main loop and
    the frontier thread (the scans come from the same robot fractions of a second apart:
    more samples make the median better, not worse).

    update() returns (median, latch): the median to use instead of the raw reading, and
    the boolean mask of the cells the hysteresis considers blocking.
    """

    def __init__(self, frames=OBSTACLE_MEDIAN_FRAMES, hysteresis_m=OBSTACLE_HYSTERESIS_M):
        self._lock = threading.Lock()
        self._frames = max(1, int(frames))
        self._hyst = float(hysteresis_m)
        self._hist = []          # [(od 2D, ix, iy)] -- ix/iy = origin in world CELLS
        self._latch = None       # 2D bool, aligned to the last element of _hist
        self._latch_ij = None
        self._cell_size = None

    def reset(self):
        with self._lock:
            self._hist = []
            self._latch = None
            self._latch_ij = None
            self._cell_size = None

    @staticmethod
    def _align(src, dix, diy, shape):
        """
        src re-read in the indices of a grid shifted by (dix, diy) cells: the value the
        new grid sees at (r, c) is at src[r + diy, c + dix]. Outside src: NaN for floats,
        False for booleans.
        """
        rows, cols = shape
        fill = False if src.dtype == bool else np.nan
        out = np.full(shape, fill, dtype=src.dtype if src.dtype == bool else np.float64)
        r0, c0 = max(0, diy), max(0, dix)                       # first valid index in src
        rr0, cc0 = max(0, -diy), max(0, -dix)                   # where it ends up in out
        nr = min(src.shape[0] - r0, rows - rr0)
        nc = min(src.shape[1] - c0, cols - cc0)
        if nr > 0 and nc > 0:
            out[rr0:rr0 + nr, cc0:cc0 + nc] = src[r0:r0 + nr, c0:c0 + nc]
        return out

    # Tolerance on the remainder of the shift, as a fraction of a cell. The shift is measured
    # between two CONSECUTIVE grids, never from an absolute origin: cell_size arrives as float32
    # (0.029999999329447746), and on a distant origin the error would exceed the tolerance.
    _SHIFT_TOL_CELLS = 0.05

    def update(self, od, origin_x, origin_y, cell_size):
        od = np.asarray(od, dtype=np.float64)
        cs = float(cell_size)
        ox, oy = float(origin_x), float(origin_y)

        def shift_from(old_ox, old_oy):
            """(dix, diy) in cells, or None if the shift is not integer."""
            fx, fy = (ox - old_ox) / cs, (oy - old_oy) / cs
            dix, diy = int(round(fx)), int(round(fy))
            if abs(fx - dix) > self._SHIFT_TOL_CELLS or abs(fy - diy) > self._SHIFT_TOL_CELLS:
                return None
            return dix, diy

        with self._lock:
            if self._cell_size is not None and abs(self._cell_size - cs) > 1e-9:
                self._hist = []
                self._latch = None
                self._latch_ij = None
            self._cell_size = cs

            stack = [od]
            for (old, old_ox, old_oy) in self._hist:
                if old.shape != od.shape:
                    continue
                sh = shift_from(old_ox, old_oy)
                if sh is None:
                    continue
                stack.append(self._align(old, sh[0], sh[1], od.shape))
            if len(stack) > 1:
                with np.errstate(invalid='ignore'):
                    med = np.nanmedian(np.stack(stack, axis=0), axis=0)
                med = np.where(np.isfinite(med), med, od)
            else:
                med = od.copy()

            thr = geometry.FRONTIER_CLEARANCE_M
            latch = med < thr
            if self._latch is not None and self._latch_ij is not None:
                sh = shift_from(self._latch_ij[0], self._latch_ij[1])
                if sh is not None:
                    prev = self._align(self._latch, sh[0], sh[1], od.shape)
                    latch |= prev & (med < thr + self._hyst)

            self._hist.append((od, ox, oy))
            if len(self._hist) > self._frames:
                self._hist.pop(0)
            self._latch = latch
            self._latch_ij = (ox, oy)

        return med, latch


OBSTACLE_HISTORY = _ObstacleHistory()


class GridSnapshot:
    """
    A scan ready for the frontier. All arrays are 2D (num_y, num_x), with the same ordering
    used everywhere in the project: row = y, column = x, origin at the bottom-left corner
    (see slopes._gather_heights).

    blocked_latch: hysteresis mask (see _ObstacleHistory); None = no filter.
    """
    __slots__ = ('obstacle_dist', 'rough_veto', 'seen', 'terrain', 'origin_x', 'origin_y',
                 'cell_size', 'robot_x', 'robot_y', 'time', 'valid', 'blocked_latch', 'robot_yaw')

    def __init__(self, obstacle_dist, rough_veto, seen, terrain, origin_x, origin_y, cell_size,
                 robot_x, robot_y, timestamp=None, valid=None, blocked_latch=None, robot_yaw=None):
        self.blocked_latch = blocked_latch
        # Robot yaw in the same snapshot, to recognize the cells under the
        # body (see _under_body). None = unknown: the inscribed circle is used.
        self.robot_yaw = None if robot_yaw is None else float(robot_yaw)
        self.obstacle_dist = obstacle_dist
        self.rough_veto = rough_veto
        self.seen = seen
        self.terrain = terrain
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size = float(cell_size)
        self.robot_x = float(robot_x)
        self.robot_y = float(robot_y)
        self.time = time.time() if timestamp is None else timestamp
        self.valid = valid      # is_valid 2D (None = all reliable): for the slope profile


def make_grid_snapshot(cells_obstacle_dist, rough_values, is_valid, terrain_valid_raw,
                       terrain_corrected, num_x, num_y, origin_x, origin_y, cell_size,
                       robot_x, robot_y, unwritten=None, robot_yaw=None):
    """
    Build the snapshot from the flat vectors the code already computes at every scan.

    - rough_veto: same criterion as fuse_obstacle_mask (roughness > threshold, on a valid cell).
    - seen: the sensor gave a height for that cell (raw terrain_valid, compared with
      spot_grid.TERRAIN_VALID_MIN and not with 0: after scale and offset 0 becomes +/-5e-17).
      It is not is_valid: is_valid also excludes the cells discarded by the ground height
      filter (walls), which have been seen; obstacle_distance flags the obstacle.
    - unwritten: cells never written by the sensors (correct_terrain with return_unwritten=True):
      they count as NOT seen, so the frontier stops there with 'end_of_data'.
    """
    shape = (num_y, num_x)
    rough = np.asarray(rough_values, dtype=np.float64).reshape(shape)
    valid = np.asarray(is_valid, dtype=bool).reshape(shape)
    if terrain_valid_raw is not None and np.size(terrain_valid_raw) == num_x * num_y:
        seen = np.asarray(terrain_valid_raw).reshape(shape) >= spot_grid.TERRAIN_VALID_MIN
    else:
        seen = np.ones(shape, dtype=bool)
    if unwritten is not None and np.size(unwritten) == num_x * num_y:
        seen = seen & ~np.asarray(unwritten, dtype=bool).reshape(shape)
    # Temporal filter + hysteresis on obstacle_distance (see _ObstacleHistory): frontier and
    # rotation check read this snapshot, not the raw reading.
    od_raw = np.asarray(cells_obstacle_dist, dtype=np.float64).reshape(shape)
    od_filtered, latch = OBSTACLE_HISTORY.update(od_raw, origin_x, origin_y, cell_size)

    return GridSnapshot(
        obstacle_dist=od_filtered,
        rough_veto=(rough > spot_grid.ROUGH_THRESHOLD) & valid,
        seen=seen,
        terrain=np.asarray(terrain_corrected, dtype=np.float64).reshape(shape),
        origin_x=origin_x, origin_y=origin_y, cell_size=cell_size,
        robot_x=robot_x, robot_y=robot_y, valid=valid, blocked_latch=latch, robot_yaw=robot_yaw)


def _under_body(snap, xs, ys):
    """
    True for the points that lie UNDER THE ROBOT'S BODY now: rectangle
    +/-ROBOT_HALF_LENGTH_M x +/-ROBOT_HALF_WIDTH_M around the pose, oriented with the
    snapshot's yaw. Without yaw, the inscribed circle (radius half width), which lies under
    the body in any orientation.

    The robot is there, so there is no obstacle to enter: those points never stop the
    frontier (same principle as geometry.rotation_is_clear).
    """
    dx = np.asarray(xs, dtype=np.float64) - snap.robot_x
    dy = np.asarray(ys, dtype=np.float64) - snap.robot_y
    yaw = getattr(snap, 'robot_yaw', None)
    if yaw is None:
        return np.hypot(dx, dy) <= geometry.ROBOT_HALF_WIDTH_M
    c, s = np.cos(yaw), np.sin(yaw)
    along = c * dx + s * dy
    across = -s * dx + c * dy
    return (np.abs(along) <= geometry.ROBOT_HALF_LENGTH_M) & (np.abs(across) <= geometry.ROBOT_HALF_WIDTH_M)


def _cells(snap, xs, ys):
    """Cell indices for direct indexing (no neighbour search) + 'inside' mask."""
    ny, nx = snap.obstacle_dist.shape
    cols = np.floor((xs - snap.origin_x) / snap.cell_size).astype(np.int64)
    rows = np.floor((ys - snap.origin_y) / snap.cell_size).astype(np.int64)
    inside = (rows >= 0) & (rows < ny) & (cols >= 0) & (cols < nx)
    return np.clip(rows, 0, ny - 1), np.clip(cols, 0, nx - 1), inside


def _polyline_samples(polyline, step):
    """Equally spaced samples along the polyline: (xs, ys, s, segment index)."""
    xs, ys, ss, seg = [], [], [], []
    s0 = 0.0
    for k in range(len(polyline) - 1):
        (x1, y1), (x2, y2) = polyline[k], polyline[k + 1]
        L = float(np.hypot(x2 - x1, y2 - y1))
        if L < 1e-9:
            continue
        n = max(1, int(np.ceil(L / step)))
        t = np.arange(n) / n                      # the last point is the first of the next segment
        xs.append(x1 + t * (x2 - x1)); ys.append(y1 + t * (y2 - y1))
        ss.append(s0 + t * L); seg.append(np.full(n, k))
        s0 += L
    if not xs:
        x, y = polyline[0]
        return np.array([x]), np.array([y]), np.array([0.0]), np.array([0]), 0.0
    x_last, y_last = polyline[-1]
    xs.append([x_last]); ys.append([y_last]); ss.append([s0]); seg.append([len(polyline) - 2])
    return (np.concatenate(xs), np.concatenate(ys), np.concatenate(ss),
            np.concatenate(seg).astype(int), s0)


def compute_safe_frontier(polyline, snap, extend_end_m=None, check_slope=True, body=None,
                          air_m=None):
    """
    How far the path is confirmed free now.

    Args:
        polyline: [(x, y), ...] -- the FIRST point is the robot position, then the waypoints.
        snap: GridSnapshot of the scan.
        extend_end_m: extend the last segment by this length beyond the goal, to also check
            the terrain under the nose when the robot gets there. Default
            FRONTIER_BODY_HALF_LENGTH_M.
        check_slope: also check the slope of the free stretches.
        body: (half footprint along the travel, half footprint across), see
            geometry.body_extents. Default: robot facing where it goes (0.55, 0.25).
        air_m: air between flank and obstacle. None = ROBOT_CLEARANCE_AIR_M (0.05), i.e. the
            minimum distance 0.30 with which it DECIDES to start. The stop thread passes
            geometry.FRONTIER_ABORT_AIR_M (0.0): while moving it only stops for a real
            intrusion, closer than half the width (0.25). The 5 cm between the two thresholds
            avoid stopping on a stretch just declared free. With an air different from the
            nominal one the hysteresis does not apply (it is tuned on the 0.30 threshold).

        The samples under the robot's body now are never an obstacle (see _under_body).

    Returns: dict with
        dist        distance along the path of the first NON-free point (m)
        reason      see the module docstring
        stop_xy     where the frontier stops
        value       the value that stopped the frontier (obstacle_distance, slope, ...)
        path_len    length of the real path, without extension
        od_robot    obstacle_distance under the robot
        clearance   minimum distance applied on the first stretch
        robot_xy, time
    """
    half_along, half_across = body if body is not None else (geometry.FRONTIER_BODY_HALF_LENGTH_M,
                                                             geometry.ROBOT_HALF_WIDTH_M)
    use_latch = air_m is None
    clearance_nominal = half_across + (geometry.ROBOT_CLEARANCE_AIR_M if air_m is None else float(air_m))
    if extend_end_m is None:
        extend_end_m = half_along
    pl = [(float(x), float(y)) for x, y in polyline]

    # Real length and extension beyond the goal
    path_len = float(sum(np.hypot(pl[k + 1][0] - pl[k][0], pl[k + 1][1] - pl[k][1])
                         for k in range(len(pl) - 1)))
    if len(pl) >= 2 and extend_end_m > 0:
        (xa, ya), (xb, yb) = pl[-2], pl[-1]
        L = np.hypot(xb - xa, yb - ya)
        if L > 1e-6:
            pl.append((xb + extend_end_m * (xb - xa) / L, yb + extend_end_m * (yb - ya) / L))

    xs, ys, ss, seg, _ = _polyline_samples(pl, geometry.FRONTIER_SAMPLE_STEP_M)
    rows, cols, inside = _cells(snap, xs, ys)

    # obstacle_distance under the robot: decides whether we are already in a tight space
    r0, c0, in0 = _cells(snap, np.array([snap.robot_x]), np.array([snap.robot_y]))
    od_robot = float(snap.obstacle_dist[r0[0], c0[0]]) if in0[0] else np.inf
    clearance_escape = max(geometry.FRONTIER_MIN_CLEARANCE_M,
                           min(clearance_nominal, od_robot - 0.02))
    thr = np.where(ss < geometry.FRONTIER_ESCAPE_DIST_M, clearance_escape, clearance_nominal)

    od = snap.obstacle_dist[rows, cols]
    near_robot = np.hypot(xs - snap.robot_x, ys - snap.robot_y) <= geometry.FRONTIER_ROBOT_RADIUS_M
    under_body = _under_body(snap, xs, ys)

    no_data = ~inside | (~snap.seen[rows, cols] & ~near_robot)
    obstacle = inside & (od < thr) & ~under_body
    # Hysteresis: a cell already blocking stays so until it rises above
    # FRONTIER_CLEARANCE_M + OBSTACLE_HYSTERESIS_M (see _ObstacleHistory). Only where the
    # nominal threshold applies: where the exit rule lowered it the robot is getting out of a
    # tight space, and the hysteresis would pin it there.
    if snap.blocked_latch is not None and use_latch:
        nominal = thr >= clearance_nominal - 1e-9
        obstacle |= inside & nominal & snap.blocked_latch[rows, cols] & ~under_body
    # Exit rule from a tight space: in the first metre (FRONTIER_ESCAPE_DIST_M) the
    # threshold drops to od_robot - 2 cm, but the promise "do not get closer than you
    # already are" holds: a sample below the nominal threshold is allowed only if it does not
    # drop below the best reached so far along the path (minus the tolerance). Moving away yes,
    # getting closer again no: a pole on the flank a bit further ahead stays blocking.
    if clearance_escape < clearance_nominal:
        od_capped = np.minimum(np.where(inside, od, clearance_nominal), clearance_nominal)
        # Under the body the reference is the robot's distance now, not the readings under
        # the belly (which would raise "the best reached" for no reason).
        od_capped = np.where(under_body, min(od_robot, clearance_nominal), od_capped)
        best_so_far = np.maximum.accumulate(od_capped)
        in_escape = inside & (ss < geometry.FRONTIER_ESCAPE_DIST_M) & (od < clearance_nominal) & ~under_body
        # Tolerance: at least 1.5 cells. obstacle_distance goes in steps of one cell (3 cm), and
        # along a wall almost parallel to the path a step must not count as
        # riavvicinamento.
        tol = max(0.02, 1.5 * snap.cell_size)
        obstacle = obstacle | (in_escape & (od < best_so_far - tol))
    rough = inside & snap.rough_veto[rows, cols] & ~near_robot

    bad = no_data | obstacle | rough
    result = dict(path_len=path_len, od_robot=od_robot, clearance=clearance_escape,
                  clearance_nominal=clearance_nominal, half_along=half_along,
                  robot_xy=(snap.robot_x, snap.robot_y), time=snap.time)

    if bad.any():
        k = int(np.argmax(bad))
        if no_data[k]:
            reason, value = 'end_of_data', None
        elif obstacle[k]:
            reason, value = 'obstacle', float(od[k])
            # The EFFECTIVE threshold at that point and whether the hysteresis decided, for the log:
            # without it, "ostacolo a 0.27 m (minimo 0.25 m)" would look like a contradiction.
            result['stop_threshold'] = float(thr[k])
            result['stop_by_latch'] = bool(
                snap.blocked_latch is not None and use_latch
                and snap.blocked_latch[rows[k], cols[k]] and od[k] >= thr[k])
        else:
            reason, value = 'roughness', None
        dist, stop_xy = float(ss[k]), (float(xs[k]), float(ys[k]))
    else:
        reason, value = 'end_of_path', None
        dist, stop_xy = float(ss[-1]), (float(xs[-1]), float(ys[-1]))

    # Slope on the stretches already confirmed free, segment by segment: if a stretch is
    # too steep the frontier moves back to its start. Same function and same threshold as the
    # PRM on the edge between the nodes (the PRM also uses the value with the body, so it is
    # stricter: the frontier cannot refuse for slope an edge the PRM has accepted).
    if check_slope and slopes.FRONTIER_SLOPE_MIN_LENGTH_M > 0:
        s_start = 0.0
        for k in range(len(pl) - 1):
            (x1, y1), (x2, y2) = pl[k], pl[k + 1]
            L = float(np.hypot(x2 - x1, y2 - y1))
            if s_start >= dist:
                break
            free_len = min(L, dist - s_start)
            if free_len >= slopes.FRONTIER_SLOPE_MIN_LENGTH_M:
                f = free_len / L
                long_s, lat_s, has = slopes.compute_arc_slope_profile(
                    snap.terrain, snap.origin_x, snap.origin_y, snap.cell_size,
                    x1, y1, x1 + f * (x2 - x1), y1 + f * (y2 - y1), valid_2d=snap.valid)
                if has and max(long_s, lat_s) > slopes.SLOPE_THRESHOLD:
                    reason, value = 'slope', float(max(long_s, lat_s))
                    dist, stop_xy = s_start, (x1, y1)
                    break
            s_start += L

    result.update(dist=dist, reason=reason, stop_xy=stop_xy, value=value)
    return result


def allowed_advance(frontier):
    """
    How far the robot CENTRE can advance along the path.

    If the whole path (extension included) is free, up to the goal. Otherwise the centre
    stops half a body length + margin before the first non-free point, so the nose also
    stays on confirmed terrain. This also holds for 'end_of_data': the nose must not end up
    on terrain never seen.
    """
    if frontier['reason'] == 'end_of_path':
        return frontier['path_len']
    half_along = frontier.get('half_along', geometry.FRONTIER_BODY_HALF_LENGTH_M)
    return max(0.0, frontier['dist'] - half_along - geometry.FRONTIER_MARGIN_M)


def point_along(polyline, s):
    """The point at distance s along the polyline (clamped at the ends) and the segment index."""
    s_acc = 0.0
    for k in range(len(polyline) - 1):
        (x1, y1), (x2, y2) = polyline[k], polyline[k + 1]
        L = float(np.hypot(x2 - x1, y2 - y1))
        if s <= s_acc + L or k == len(polyline) - 2:
            t = 0.0 if L < 1e-9 else min(1.0, max(0.0, (s - s_acc) / L))
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1)), k
        s_acc += L
    return polyline[-1], max(0, len(polyline) - 2)


def describe_frontier(frontier):
    """One readable line for the log."""
    r = frontier['reason']
    adv = allowed_advance(frontier)
    sx, sy = frontier['stop_xy']
    base = (f"{frontier['dist']:.2f} m confermati su {frontier['path_len']:.2f} m di percorso, "
            f"avanzamento consentito {adv:.2f} m")
    if r == 'end_of_path':
        return f"[FRONTE] {base} -- percorso libero fino all'obiettivo"
    if r == 'end_of_data':
        return f"[FRONTE] {base} -- si ferma sul bordo dei dati in ({sx:.2f}, {sy:.2f}): avanzo e riguardo"
    if r == 'obstacle':
        thr = frontier.get('stop_threshold', frontier.get('clearance_nominal',
                                                          geometry.FRONTIER_CLEARANCE_M))
        if frontier.get('stop_by_latch'):
            why = (f"(minimo {thr:.2f} m; qui decide l'isteresi, che tiene bloccante la cella "
                   f"fino a {thr + OBSTACLE_HYSTERESIS_M:.2f} m)")
        else:
            why = f"(minimo {thr:.2f} m)"
        return (f"[FRONTE] {base} -- ostacolo a {frontier['value']:.2f} m dalla linea centrale in "
                f"({sx:.2f}, {sy:.2f}) {why}")
    if r == 'roughness':
        return f"[FRONTE] {base} -- terreno troppo irregolare in ({sx:.2f}, {sy:.2f})"
    return (f"[FRONTE] {base} -- tratto troppo ripido da ({sx:.2f}, {sy:.2f}) "
            f"(pendenza {frontier['value']:.3f} > {slopes.SLOPE_THRESHOLD:.3f})")


# =============================================================================
# Background checker: keeps the frontier up to date while the robot walks.
# =============================================================================
class ArcVerificationTracker:
    """
    Background thread that recomputes the safe frontier on the current path at every scan
    (~5 Hz), with its own fresh data.

    Used DURING the move: the main loop queries it to stop the robot if the frontier gets
    shorter than the point it is heading to. Before starting the main loop computes the
    frontier itself, on its own scan.

    The result carries the version of the path it was computed on: after a replan an old
    frontier cannot be mistaken for a new one.
    """

    def __init__(self, robot):
        self.robot = robot
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._local_grid = spot_grid.LocalGrid(self.robot)
        self._lock = threading.Lock()
        self._waypoints = []          # [(x, y), ...] WITHOUT the robot position
        self._extend_end_m = None
        self._body = None
        self._path_version = 0
        self._latest = None           # last frontier computed
        self._running = False
        self._thread = None

    # --- interface for the main loop -----------------------------------------------------
    def update_path(self, waypoints_xy, extend_end_m=None, body=None):
        """
        Set the path to follow (waypoints only). Returns the new version.
        body: body footprint relative to the travel (see compute_safe_frontier), to pass
        when the robot walks without turning.
        """
        with self._lock:
            self._waypoints = [(float(x), float(y)) for x, y in waypoints_xy]
            self._extend_end_m = extend_end_m
            self._body = body
            self._path_version += 1
            self._latest = None
            return self._path_version

    def clear_path(self):
        self.update_path([])

    def get_frontier(self):
        """Copy of the last frontier (dict) with 'path_version', or None."""
        with self._lock:
            return None if self._latest is None else dict(self._latest)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()
        print("ArcVerificationTracker (fronte sicuro) avviato in background.")

    def stop(self):
        self._running = False
        print('ArcVerificationTracker stopped.')

    # --- background loop -------------------------------------------------------------------
    def _process_loop(self):
        while self._running:
            with self._lock:
                waypoints = list(self._waypoints)
                extend = self._extend_end_m
                body = self._body
                version = self._path_version
            if not waypoints:
                time.sleep(0.2)
                continue
            try:
                snap = self._fresh_snapshot()
                if snap is None:
                    time.sleep(0.2)
                    continue
                # This frontier is ONLY for the stop halfway through a move: intrusion threshold
                # (half width, without air), see geometry.FRONTIER_ABORT_AIR_M.
                frontier = compute_safe_frontier([(snap.robot_x, snap.robot_y)] + waypoints, snap,
                                                 extend_end_m=extend, body=body,
                                                 air_m=geometry.FRONTIER_ABORT_AIR_M)
                frontier['path_version'] = version
                with self._lock:
                    if version == self._path_version:   # the path has not changed in the meantime
                        self._latest = frontier
            except Exception as e:
                LOGGER.error(f"Errore nel calcolo del fronte: {e}")
            time.sleep(0.2)

    def _fresh_snapshot(self):
        grids_data, main_proto, _ = self._local_grid.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'],
            robot_state_client=self._robot_state_client)
        if grids_data is None or 'obstacle_distance' not in grids_data:
            return None

        pts = grids_data['obstacle_distance']['pts']
        cells_obs = grids_data['obstacle_distance']['values']
        terrain_vals = grids_data['terrain']['values']
        valid_vals = grids_data['terrain_valid']['values']
        num_x = main_proto.local_grid.extent.num_cells_x
        num_y = main_proto.local_grid.extent.num_cells_y
        cell_size = main_proto.local_grid.extent.cell_size

        # Robot pose AT THE SAME instant as the grid (same snapshot of transforms)
        transforms_snapshot = main_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        robot_x, robot_y = vision_tform_body.position.x, vision_tform_body.position.y
        q = vision_tform_body.rotation
        robot_yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        footprint_mask_2d = self._local_grid.compute_robot_footprint_mask(
            pts, robot_x, robot_y, robot_yaw, num_x, num_y)
        terrain_corrected, is_valid, unwritten = self._local_grid.correct_terrain(
            terrain_vals, valid_vals, num_x, num_y,
            robot_footprint_mask=footprint_mask_2d, cell_size=cell_size,
            unwritten_value=grids_data['terrain'].get('raw_zero'),
            unwritten_scale=grids_data['terrain'].get('scale'), return_unwritten=True)
        _, rough_vals = self._local_grid.compute_gradient_and_roughness(
            terrain_corrected, valid_vals, num_x, num_y, cell_size, is_valid=is_valid)

        grid_frame_name = main_proto.local_grid.frame_name_local_grid_data
        vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
        return make_grid_snapshot(cells_obs, rough_vals, is_valid, valid_vals, terrain_corrected,
                                  num_x, num_y, vision_tform_grid.position.x,
                                  vision_tform_grid.position.y, cell_size, robot_x, robot_y,
                                  unwritten=unwritten, robot_yaw=robot_yaw)


def is_arc_in_fov(x1, y1, x2, y2, pts, fov_margin=0.0):
    """
    True if the edge lies ENTIRELY inside the local grid window (~3.8 m). It is not the
    cameras' field of view. Used by the retreat.
    """
    if len(pts) == 0:
        return False

    x_min, x_max = pts[:, 0].min() - fov_margin, pts[:, 0].max() + fov_margin
    y_min, y_max = pts[:, 1].min() - fov_margin, pts[:, 1].max() + fov_margin

    p1_in_fov = (x_min <= x1 <= x_max) and (y_min <= y1 <= y_max)
    p2_in_fov = (x_min <= x2 <= x_max) and (y_min <= y2 <= y_max)
    mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
    mid_in_fov = (x_min <= mid_x <= x_max) and (y_min <= mid_y <= y_max)

    return p1_in_fov and p2_in_fov and mid_in_fov


def verify_arc_safety(x1, y1, x2, y2, pts, obstacle_mask):
    """Point check of a segment on the fused mask. Used by the retreat."""
    return spotUtils.check_line_of_sight(x1, y1, x2, y2, pts, obstacle_mask)


# =========================================================================================
# DECISIONS ON THE FRONTIER: what to do now given the safe frontier and the fresh scan.
# Pure functions (no robot, no I/O), tested by the end-to-end test and the replay.
# =========================================================================================
def goal_still_reachable(snap, goal_x, goal_y):
    """
    Is the goal point still reachable on the FRESH data?

    The safe frontier requires the centre line of the path to be at least
    FRONTIER_CLEARANCE_M from an obstacle. A goal point with obstacle_distance below that
    threshold can NEVER be reached, however many scans are made: the robot would wait and
    declare a block. It happens because the point is chosen from afar, when the area is
    still unknown.

    Returns:
        True if the point is reachable or if it cannot be judged (outside the scanned
        window: we do not know, and nothing is touched on the basis of nothing).
    """
    if snap is None:
        return True
    rows, cols, inside = _cells(snap, np.array([float(goal_x)]), np.array([float(goal_y)]))
    if not bool(inside[0]):
        return True
    return float(snap.obstacle_dist[rows[0], cols[0]]) >= geometry.FRONTIER_CLEARANCE_M


def prune_prm_nodes_in_obstacles(prm, snap, min_clearance_m=P.PRM_NODE_MIN_CLEARANCE_M):
    """
    Discard from the graph the nodes that fall INSIDE an obstacle.

    The edges crossing occupied cells are vetoed (_occupancy_blocked), but the nodes are
    born from the lattice before the robot has seen anything: a node inside an obstacle
    would remain a destination or waypoint, to be discovered only on the spot.

    Threshold min_clearance_m = FRONTIER_CLEARANCE_M (0.30): below that the body centre
    does not fit and the frontier will never get there, so the node is useless both as a
    destination and as a waypoint.

    Discarded ONLY where the cell has really been observed: unknown is not occupied (the
    same rule by which the frontier tells 'end_of_data' from 'obstacle'). A node outside the
    window or never seen stays where it is and will be judged when it is seen.

    Re-evaluated at every loop, so a node found to be inside a wall drops at once; and
    since the edges are removed NON-permanently, a node misjudged by a noisy reading comes
    back into play when the fresh data contradicts it.

    The scan window contains only a small part of the graph (~7%), and the discard is
    soft: at the next refresh the edges come back.
    """
    ids = [nid for nid in prm.nodes if prm.edges.get(nid)]   # already isolated: nothing to remove
    if not ids:
        return []
    xy = np.array([prm.nodes[nid] for nid in ids], dtype=np.float64)
    rows, cols, inside = _cells(snap, xy[:, 0], xy[:, 1])
    bad = np.zeros(len(ids), dtype=bool)
    if np.any(inside):
        r, c = rows[inside], cols[inside]
        bad[inside] = snap.seen[r, c] & (snap.obstacle_dist[r, c] < min_clearance_m)
    pruned = []
    for k in np.nonzero(bad)[0]:
        nid = ids[int(k)]
        for nb in [n for n, _ in prm.edges.get(nid, [])]:
            prm_graph.mark_edge_blocked_soft(prm, nid, nb)
        pruned.append(nid)
    return pruned
