"""
End-to-end test of the REAL navigation loop (attempt_enter_cell_from_position in
mission/cell_attempt.py) against a fake robot in a synthetic world.

What is fake: the robot position, the local grid (128x128 window of 3 cm following the
robot, with obstacle_distance computed from the world and ghost cells in the corners as
in the real data), the motion (the robot moves along the straight line, querying
should_abort at every leg), the frontier thread (computes the real frontier at the
current position). Everything else is the real code: terrain correction, global maps,
PRM, stop nodes, frontier, decisions, blocks, replans, retreat, data saving.

    python3 tests/test_e2e_loop.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile
import time
import types
from types import SimpleNamespace as NS

import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # the spot_explorer package

import matplotlib  # noqa: E402
matplotlib.use("Agg")
from spot_explorer.mission import attempt_legs, attempt_obstacles, attempt_scan, cell_attempt     # noqa: E402
from spot_explorer import config as parametri     # noqa: E402
from spot_explorer import records as registro      # noqa: E402
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier
from spot_explorer.planning import roadmap as prm_graph     # noqa: E402
from spot_explorer.robot import motion as movements     # noqa: E402
from spot_explorer.robot import connection as spotUtils     # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


CS, N, W = 0.03, 128, 400          # 12 x 12 m world


class World:
    def __init__(self, rects, ghosts=True):
        self.occ = np.zeros((W, W), bool)
        wx = (np.arange(W) + 0.5) * CS
        WX, WY = np.meshgrid(wx, wx)
        for x0, x1, y0, y1 in rects:
            self.occ |= (WX >= x0) & (WX <= x1) & (WY >= y0) & (WY <= y1)
        self.dynamic = None          # rectangle that appears during a move
        self.ghosts = ghosts
        self._recompute()

    def _recompute(self):
        occ = self.occ.copy()
        if self.dynamic is not None:
            x0, x1, y0, y1 = self.dynamic
            wx = (np.arange(W) + 0.5) * CS
            WX, WY = np.meshgrid(wx, wx)
            occ |= (WX >= x0) & (WX <= x1) & (WY >= y0) & (WY <= y1)
        self.occ_now = occ
        self.od = distance_transform_edt(~occ) * CS if occ.any() else np.full((W, W), 9.0)
        # SIGNED distance inside obstacles, like Spot's real layer (mission 08-10
        # 16:35: 22% of the cells negative, down to below -0.5 m). With a fixed value the
        # rotation check cannot see how far the body goes into the wall.
        if occ.any():
            self.od[occ] = -(distance_transform_edt(occ) * CS)[occ]

    def od_at(self, x, y):
        r, c = int(y / CS), int(x / CS)
        return self.od[r, c] if 0 <= r < W and 0 <= c < W else 9.0


class Robot:
    def __init__(self, x, y, yaw=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.min_clear = np.inf
        self.track = [(x, y)]
        self.commands = 0          # how many motion commands were given
        self.walked = 0.0          # metres travelled
        self.turns = 0             # rotations above the negligible threshold (10 degrees)
        self.turn_deg = 0.0        # total degrees rotated
        self.max_endpoint_error = 0.0   # maximum gap between commanded point and reached point


WORLD, ROBOT = None, None
# Extra parameters for build_mission_graph in the scenarios: empty = the lattice.
GRAPH_KW = {}


def quat(yaw):
    return NS(w=np.cos(yaw / 2), x=0.0, y=0.0, z=np.sin(yaw / 2))


def window():
    c0 = int(np.floor(ROBOT.x / CS)) - N // 2
    r0 = int(np.floor(ROBOT.y / CS)) - N // 2
    rr, cc = np.mgrid[0:N, 0:N]
    wr, wc = rr + r0, cc + c0
    ok = (wr >= 0) & (wr < W) & (wc >= 0) & (wc < W)
    od = np.full((N, N), 9.0)
    od[ok] = WORLD.od[wr[ok], wc[ok]]
    terr = -3.23 + np.random.default_rng(int(ROBOT.x * 1000) % 99991).normal(0, 0.004, (N, N))
    if WORLD.ghosts:
        terr[108:, 120:] = -0.00052261
        terr[:12, :10] = -0.00052261
    ox, oy = c0 * CS, r0 * CS
    xs = ox + (np.arange(N) + 0.5) * CS
    ys = oy + (np.arange(N) + 0.5) * CS
    X, Y = np.meshgrid(xs, ys)
    pts = np.stack([X.ravel(), Y.ravel(), np.full(N * N, -3.23)], axis=1)
    snapshot = NS(robot=(ROBOT.x, ROBOT.y, ROBOT.yaw), origin=(ox, oy))
    return pts, od.ravel(), terr.astype(np.float32).ravel(), np.ones(N * N, np.float32), snapshot


def fake_get_a_tform_b(snapshot, a, b):
    if b == attempt_legs.BODY_FRAME_NAME:
        x, y, yaw = snapshot.robot
        return NS(position=NS(x=x, y=y, z=-2.7), rotation=quat(yaw))
    ox, oy = snapshot.origin
    return NS(position=NS(x=ox, y=oy, z=0.0), rotation=quat(0.0))


class FakeLocalGrid(spot_grid.LocalGrid):
    def __init__(self):
        self.footprint_correction_pending = True
        self.last_ground_filter = {'ground_z': None, 'n_above': 0, 'max_above': 0.0,
                                   'source': '', 'n_ref': 0}
        self.local_grid_client = None
        self.rejected_scans = 0

    def return_local_grid(self, layers, robot_state_client=None):
        pts, od, terr, valid, snap = window()
        data = {'obstacle_distance': {'pts': pts, 'values': od},
                'terrain': {'values': terr, 'raw_zero': float(np.float32(-0.00052261)), 'scale': 0.001},
                'terrain_valid': {'values': valid}}
        proto = NS(local_grid=NS(extent=NS(num_cells_x=N, num_cells_y=N, cell_size=CS),
                                 transforms_snapshot=snap, frame_name_local_grid_data="grid"))
        return data, proto, None

    def extra_layers(self, names):
        # Like spot_grid.LocalGrid.extra_layers: a fake layer to check the saving;
        # 'no_step' is missing on purpose (an absent layer must not stop anything).
        if 'intensity' not in names:
            return {}
        return {'intensity': dict(values=np.full((N, N), 7.0, np.float32),
                                  origin=np.array([ROBOT.x - N * CS / 2, ROBOT.y - N * CS / 2]),
                                  cell=CS)}


class FakeTracker:
    """The frontier thread: here it computes the REAL frontier at the current position."""
    def __init__(self):
        self.version, self.waypoints, self.body = 0, [], None
        self.lg = FakeLocalGrid()

    def update_path(self, waypoints_xy, extend_end_m=None, body=None):
        self.version += 1
        self.waypoints, self.body = list(waypoints_xy), body
        return self.version

    def clear_path(self):
        self.update_path([])

    def get_frontier(self):
        if not self.waypoints:
            return None
        pts, od, terr, valid, _ = window()
        tc, iv = self.lg.correct_terrain(terr, valid, N, N, cell_size=CS)
        _, rough = self.lg.compute_gradient_and_roughness(tc, valid, N, N, CS, is_valid=iv)
        c0 = int(np.floor(ROBOT.x / CS)) - N // 2
        r0 = int(np.floor(ROBOT.y / CS)) - N // 2
        snap = safe_frontier.make_grid_snapshot(od, rough, iv, valid, tc, N, N, c0 * CS, r0 * CS, CS, ROBOT.x, ROBOT.y)
        f = safe_frontier.compute_safe_frontier([(ROBOT.x, ROBOT.y)] + self.waypoints, snap, body=self.body)
        f['path_version'] = self.version
        f['time'] = time.time()
        return f


DYNAMIC_TRIGGER = {'armed': False, 'rect': None}
FALL = {'armed': False, 'fallen': False, 'recoverable': True, 'where': None}
# Command that does not complete (timeout): the next 'n' legs longer than 0.3 m stop
# 'short_m' before the point and return failure (scenario S17).
STALL = {'n': 0, 'short_m': 0.0}


def fake_check_fall(rsc):
    return dict(fallen=FALL['fallen'], reason="fall fault (test)" if FALL['fallen'] else "",
                roll=80.0 if FALL['fallen'] else 0.0, pitch=0.0, x=ROBOT.x, y=ROBOT.y,
                fall_faults=[1] if FALL['fallen'] else [], clearable=[1] if FALL['fallen'] else [],
                unclearable=[], motors_on=True, standing=not FALL['fallen'])


def fake_recover(cc, rsc, status=None):
    if not FALL['fallen']:
        return True
    if not FALL['recoverable']:
        raise movements.RobotFallenError("test: the robot does not get up by itself")
    FALL['fallen'] = False
    return True


def _yaw_of(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))


def fake_navigate_to(tx, ty, rx, ry, rsc, cc, vtb, should_abort=None, keep_heading=False, **kw):
    """
    FAITHFUL to the real navigate_to, to catch a heading passed in late. The real
    navigate_to:
      1. computes dyaw = target direction - heading read from `vtb`;
      2. relative_move(0, 0, dyaw): rotates by dyaw RELATIVE TO THE TRUE POSE of the robot;
      3. relative_move(dist, 0, 0): walks straight for `dist`, along the TRUE nose, with
         dist computed from the (rx, ry) passed in, not from the true position.
    If `vtb` or (rx, ry) are stale, the robot goes somewhere else: and that is exactly
    what this fake now reproduces.
    """
    ROBOT.commands += 1
    dist = float(np.hypot(tx - rx, ty - ry))
    if keep_heading:
        sx, sy = ROBOT.x, ROBOT.y
        ex, ey = tx, ty
    else:
        target_yaw = float(np.arctan2(ty - ry, tx - rx))
        dyaw = float(np.arctan2(np.sin(target_yaw - _yaw_of(vtb.rotation)),
                                np.cos(target_yaw - _yaw_of(vtb.rotation))))
        ROBOT.turn_deg += abs(np.degrees(dyaw))
        if abs(np.degrees(dyaw)) >= geometry.ROTATION_CHECK_MIN_DYAW_DEG:
            ROBOT.turns += 1
        ROBOT.yaw = float(np.arctan2(np.sin(ROBOT.yaw + dyaw), np.cos(ROBOT.yaw + dyaw)))
        sx, sy = ROBOT.x, ROBOT.y
        ex, ey = sx + dist * np.cos(ROBOT.yaw), sy + dist * np.sin(ROBOT.yaw)
    stalled = STALL['n'] > 0 and dist > 0.3
    if stalled:
        STALL['n'] -= 1
        f = (dist - STALL['short_m']) / dist
        ex, ey = sx + f * (ex - sx), sy + f * (ey - sy)
    # How far the true arrival point is from the commanded one: that is the symptom.
    ROBOT.max_endpoint_error = max(ROBOT.max_endpoint_error, float(np.hypot(ex - tx, ey - ty)))
    steps = max(2, int(dist / 0.05))
    for k in range(1, steps + 1):
        t = k / steps
        ROBOT.x, ROBOT.y = sx + t * (ex - sx), sy + t * (ey - sy)
        ROBOT.min_clear = min(ROBOT.min_clear, WORLD.od_at(ROBOT.x, ROBOT.y))
        ROBOT.track.append((ROBOT.x, ROBOT.y))
        remaining = (1 - t) * dist
        if DYNAMIC_TRIGGER['armed'] and t > 0.25 and remaining > 0.45:
            ux, uy = (ex - sx) / max(dist, 1e-9), (ey - sy) / max(dist, 1e-9)
            # 0.85 m ahead of the robot CENTRE: beyond half the body (0.55) plus the
            # frontier margin (0.10), so stopping in time is possible.
            cx, cy = ROBOT.x + 0.85 * ux, ROBOT.y + 0.85 * uy
            WORLD.dynamic = (cx - 0.2, cx + 0.2, cy - 0.2, cy + 0.2); WORLD._recompute()
            DYNAMIC_TRIGGER['armed'] = False
        if FALL['armed'] and t > 0.5:
            FALL.update(armed=False, fallen=True, where=(ROBOT.x, ROBOT.y))
            d = float(np.hypot(ROBOT.x - sx, ROBOT.y - sy)); ROBOT.walked += d
            return False, d, dist
        if should_abort is not None and k % 2 == 0 and should_abort():
            d = float(np.hypot(ROBOT.x - sx, ROBOT.y - sy)); ROBOT.walked += d
            return False, d, dist
    if stalled:
        d = float(np.hypot(ROBOT.x - sx, ROBOT.y - sy)); ROBOT.walked += d
        return False, d, dist
    ROBOT.walked += dist
    return True, dist, dist


def fake_move_no_turn(tx, ty, frame, cc, rsc, **kw):
    sx, sy = ROBOT.x, ROBOT.y
    ROBOT.x, ROBOT.y = tx, ty
    ROBOT.track.append((tx, ty))
    ROBOT.commands += 1
    ROBOT.walked += float(np.hypot(tx - sx, ty - sy))
    return True, float(np.hypot(tx - sx, ty - sy))


class FakeEnv:
    rows, cols, cell_size, origin_yaw = 2, 2, 5.0, 0.0
    start_cell = (0, 0)
    origin_x, origin_y = 3.5, 3.5

    def __init__(self):
        # (0,0) visited: it is the cell the robot starts from in all scenarios.
        self.visited = {(0, 0)}

    def is_cell_visited(self, r, c):
        return 1 if (r, c) in self.visited else 0

    def mark_cell_visited(self, r, c):
        self.visited.add((r, c))

    def get_cell_from_world(self, x, y):
        return (int(round((y - 3.5) / 5.0)), int(round((x - 3.5) / 5.0)))

    def get_world_position_from_cell(self, r, c):
        return (1.0 + c * 5.0 + 2.5, 1.0 + r * 5.0 + 2.5) if 0 <= r < 2 and 0 <= c < 2 else None

    def is_point_in_cell(self, x, y, r, c):
        cx, cy = self.get_world_position_from_cell(r, c)
        return abs(x - cx) <= 2.5 and abs(y - cy) <= 2.5

    def get_cell_status(self, r, c):
        return (0, 0)


class FakeSampler:
    def get_point_in_cell(self, r, c):
        cx, cy = FakeEnv().get_world_position_from_cell(r, c)
        g = np.linspace(-2.2, 2.2, 9)
        return [(cx + a, cy + b) for a in g for b in g]


# ---- patches: only the boundaries with the robot ----------------------------------------
attempt_scan.get_a_tform_b = attempt_obstacles.get_a_tform_b = attempt_legs.get_a_tform_b = spot_grid.get_a_tform_b = fake_get_a_tform_b
spotUtils.getPosition = lambda rsc: (ROBOT.x, ROBOT.y, -2.7, quat(ROBOT.yaw))
movements.navigate_to = fake_navigate_to
movements.move_to_world_point_without_turning = fake_move_no_turn
parametri.FRONTIER_RESCAN_WAIT_S = 0.0
movements.check_fall = fake_check_fall
movements.recover_from_fall = fake_recover


class _NoTilt:
    def __init__(self, *a): pass
    def start(self): pass
    def stop(self):
        return dict(roll_start=0, pitch_start=0, roll_end=0, pitch_end=0, roll_peak_abs=0,
                    pitch_peak_abs=0, samples=0, errors=0)


attempt_legs.BodyTiltRecorder = registro.BodyTiltRecorder = _NoTilt


def run(name, rects, start, target_cell, ghosts=True, dynamic=None, fall=None, second_attempt=False,
        memory=None):
    global WORLD, ROBOT
    WORLD, ROBOT = World(rects, ghosts), Robot(*start)
    # The temporal obstacle filter (OBSTACLE_HISTORY) is aligned to the
    # WORLD and survives from one scan to the next: on the robot that is right, the world is
    # always the same. Here every scenario is a different world with the same coordinates, and
    # without resetting it the median mixed the previous scenario's scans with the
    # new ones: S11 passed or failed depending on what ran before.
    safe_frontier.OBSTACLE_HISTORY.reset()
    DYNAMIC_TRIGGER.update(armed=dynamic is not None, rect=dynamic)
    FALL.update(armed=fall is not None, fallen=False, recoverable=(fall == 'recuperabile'), where=None)
    registro.MISSION_STATS['falls'] = registro.MISSION_STATS['self_rights'] = 0
    # The real mission lattice: 0.25 m spacing, 0.5-1.25 m edges, on FakeEnv's 2 x 2 cells of
    # 5 m, for all scenarios. GRAPH_KW selects the random nodes (comparison at the end).
    from spot_explorer.planning import mission_graph
    mg = mission_graph.build_mission_graph(
        rows=2, cols=2, cell_size_m=5.0, spacing_m=0.25, rings=None,
        origin_xy=(3.5, 3.5), origin_yaw=0.0, min_edge_length=parametri.PRM_MIN_EDGE_M,
        max_edge_length=parametri.PRM_MAX_EDGE_M,
        connection_radius=parametri.PRM_CONNECTION_RADIUS_M, verbose=False,
        **dict({'sampling': 'lattice'}, **GRAPH_KW))
    prm = prm_graph.PRM(min_edge_length=parametri.PRM_MIN_EDGE_M,
                        max_edge_length=parametri.PRM_MAX_EDGE_M,
                        connection_radius=parametri.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg
    mg.into_prm(prm)
    folder = tempfile.mkdtemp(prefix="mission_e2e_")
    for k in list(registro.MISSION_STATS['time_s']):
        registro.MISSION_STATS['time_s'][k] = 0.0
    for k in ('frontier_blocks', 'frontier_aborts', 'scan_bundles', 'prm_refresh',
              'prm_refresh_skipped', 'multinode_moves', 'multinode_nodes', 'idle_loops'):
        if k in registro.MISSION_STATS:
            registro.MISSION_STATS[k] = 0
    gg = NS(global_occupancy_map=None, global_terrain_map=None)
    if memory:
        # Obstacles already in the global map from earlier observations (in a mission: the cells
        # already visited). Cells with obstacle_distance <= OBSTACLE_THRESHOLD are occupied, as
        # fold_scan_into_global_maps writes them.
        gm0 = global_maps.GlobalGrid(resolution=CS)
        t = geometry.OBSTACLE_THRESHOLD
        for x0, x1, y0, y1 in memory:
            for gx in range(int(np.floor((x0 - t) / CS)), int(np.ceil((x1 + t) / CS))):
                for gy in range(int(np.floor((y0 - t) / CS)), int(np.ceil((y1 + t) / CS))):
                    gm0.obs_count[(gx, gy)] = global_maps.GLOBAL_OCC_MAX_COUNT
                    gm0._set_state((gx, gy), -1.0)
        gg.global_occupancy_map = gm0
    out = io.StringIO()
    err = None
    t0 = time.time()
    with contextlib.redirect_stdout(out):
        try:
            ok = cell_attempt.attempt_enter_cell_from_position(
                FakeLocalGrid(), gg, None, None, FakeEnv(), target_cell[0], target_cell[1],
                FakeSampler(), prm, mission_folder=folder, iteration=0,
                recordingInterface=None, verification_tracker=FakeTracker())
        except movements.RobotFallenError as e:
            err, ok = None, f"RobotFallenError: {e}"
        except Exception as e:
            import traceback
            err = traceback.format_exc()
            ok = None
        ok2, track2_from = None, None
        if second_attempt and err is None:
            ROBOT.track.append((ROBOT.x, ROBOT.y))
            track2_from = len(ROBOT.track) - 1
            ok2 = cell_attempt.attempt_enter_cell_from_position(
                FakeLocalGrid(), gg, None, None, FakeEnv(), target_cell[0], target_cell[1],
                FakeSampler(), prm, mission_folder=folder, iteration=1,
                recordingInterface=None, verification_tracker=FakeTracker())
    registro.VIS_EXECUTOR.submit(lambda: None).result()      # wait for the saves
    # The real code returns env.is_point_in_cell(...), which with numpy coordinates is an
    # np.bool_: the caller does `if check:` and it works, but the checks here use `is True`.
    if isinstance(ok, np.bool_):
        ok = bool(ok)
    if isinstance(ok2, np.bool_):
        ok2 = bool(ok2)
    log = out.getvalue()
    # Reference recording (E2E_LOG_DIR=folder): the full log of every scenario,
    # to compare behaviour line by line before and after a code
    # restructure (see tools/compare_logs.py).
    if os.environ.get('E2E_LOG_DIR'):
        os.makedirs(os.environ['E2E_LOG_DIR'], exist_ok=True)
        slug = "".join(c if c.isalnum() else "_" for c in name)[:60]
        with open(os.path.join(os.environ['E2E_LOG_DIR'], slug + ".log"), "w") as fh:
            fh.write(log)
    n_scans = len(os.listdir(os.path.join(folder, "scans"))) if os.path.isdir(os.path.join(folder, "scans")) else 0
    print(f"\n{name}  ({time.time() - t0:.1f} s)")
    if err:
        print(err)
    tags = {t: log.count(t) for t in ("[FALL] Detected", "[FRONTIER]", "[PARTIAL STEP]", "[OK] Leg",
                                      "[BLOCK] Confirmed", "[SHORTCUT]", "[MULTINODE]",
                                      "[REPLAN] After the block", "[STOP]", "[NODE]", "[RETREAT]",
                                      "[ROTATION]", "[GROUND-FILTER]", "[GOAL]", "[STALL]",
                                      "[EARLY]", "[TRAP]", "[ARRIVAL]", "[CONE]")}
    print("       " + "  ".join(f"{k} x{v}" for k, v in tags.items() if v))
    print(f"       maximum gap between commanded and reached point: {ROBOT.max_endpoint_error:.3f} m")
    print(f"       motion commands: {ROBOT.commands}, metres travelled: {ROBOT.walked:.1f}, "
          f"scans: {n_scans}, refresh {registro.MISSION_STATS.get('prm_refresh', 0)} done / "
          f"{registro.MISSION_STATS.get('prm_refresh_skipped', 0)} skipped")
    return dict(ok=ok, err=err, log=log, scans=n_scans, folder=folder, tags=tags,
                min_clear=ROBOT.min_clear, final=(ROBOT.x, ROBOT.y), ok2=ok2, gg=gg,
                commands=ROBOT.commands, walked=ROBOT.walked,
                turns=ROBOT.turns, turn_deg=ROBOT.turn_deg,
                endpoint_error=ROBOT.max_endpoint_error,
                stats=dict(registro.MISSION_STATS),
                track2=ROBOT.track[track2_from:] if track2_from is not None else [])


if __name__ == '__main__':
    print("Fake robot, real code. Goal cell (0,1): centre (8.5, 3.5).")
    print("NB: the goal cell must be ADJACENT to a visited one -- since 2026-10-08\n    planning is limited to the explored cells (PLAN_ONLY_EXPLORED_CELLS).")

    r = run("S1. open field, ghost cells in the corners of every window (2026-10-05)",
            [], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is True, f"entered the cell, final position ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['tags']["[BLOCK] Confirmed"] == 0 and r['tags']["[RETREAT]"] == 0,
          "no block and no retreat on free terrain")
    check(r['scans'] > 0, f"{r['scans']} scan bundles saved")
    _b = np.load(os.path.join(r['folder'], 'scans', sorted(os.listdir(os.path.join(r['folder'], 'scans')))[0]))
    check('intensity' in _b.files and _b['intensity'].shape == (N, N) and 'intensity_origin' in _b.files
          and 'no_step' not in _b.files,
          "extra layers in the bundle: intensity saved with its origin, no_step absent without harm")
    check("[TIMING]" in r['log'], "[TIMING] line present")

    r = run("S2. wall with a 1.2 m gap far from the straight line: it must be found by going around",
            [(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is True, f"entered the cell through the gap ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.05,
          f"minimum distance of the centre from obstacles along the whole way: {r['min_clear']:.2f} m")

    r = run("S3. a person steps in front of the robot while it walks",
            [], (2.0, 2.0, 0.785), (0, 1), dynamic=True)
    check(r['err'] is None, "no exception")
    check(r['min_clear'] > 0.0,
          f"never touches the obstacle that appeared during the move "
          f"(minimum distance of the centre {r['min_clear']:.2f} m)")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.10,
          f"and stays beyond the minimum distance required by the frontier "
          f"({geometry.FRONTIER_CLEARANCE_M:.2f} m): measured {r['min_clear']:.2f} m")
    print(f"       [STOP] x{r['tags']['[STOP]']}, [MULTINODE] x{r['tags']['[MULTINODE]']}")

    r = run("S4. goal cell closed by walls on all sides",
            [(6.1, 6.3, 1.0, 6.0), (6.1, 10.9, 1.0, 1.2), (10.7, 10.9, 1.0, 6.0), (6.1, 10.9, 5.8, 6.0)],
            (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is False, "attempt failed in an orderly way (cell unreachable)")
    check(r['min_clear'] > 0.0, f"never enters the wall (minimum distance {r['min_clear']:.2f} m)")
    # The second [SUBITO] on the same leg removes it from the plan for the rest of the attempt
    # (28-10 14:23: the same leg discarded four times in a row).
    legs = re.findall(r"\[EARLY\] \w+ at \([^)]*\) on the leg (\d+->\d+)", r['log'])
    check("out of the plan for the rest of the attempt" in r['log'] and max(
          [legs.count(t) for t in legs] or [0]) <= 2,
          f"[EARLY] repeated: at the second time the leg leaves the plan ({legs})")

    r = run("S5. start in a tight space: 0.45 m from the wall, target at 90 degrees",
            [(2.45, 2.65, 0.0, 6.0)], (2.0, 3.0, 1.5708), (0, 0))
    check(r['err'] is None, "no exception")

    r = run("S6. fall halfway through a move, the robot gets up; then a second attempt",
            [], (2.0, 2.0, 0.785), (0, 1), fall='recuperabile', second_attempt=True)
    check(r['err'] is None, "no exception")
    check(r['tags']["[FALL] Detected"] == 1 and registro.MISSION_STATS['self_rights'] == 1,
          "fall detected and robot up again")
    fx, fy = FALL['where']
    gm = r['gg'].global_occupancy_map
    check(gm is not None and gm.is_occupied(fx, fy),
          f"fall point ({fx:.2f}, {fy:.2f}) occupied in the global map")

    r = run("S7. fall from which the robot does NOT get up by itself",
            [], (2.0, 2.0, 0.785), (0, 1), fall='irrecuperabile')
    check(r['err'] is None and isinstance(r['ok'], str) and r['ok'].startswith("RobotFallenError"),
          "the mission stops with RobotFallenError (main() catches it and calls the operator)")
    check("[RETREAT]" not in r['log'], "no attempt to move from the ground")

    r = run("S8. REAL lattice (spacing 0.25, edges 0.5-1.25) on open field",
            [], (3.5, 3.5, 0.0), (0, 1))
    check(r['err'] is None, "no exception with the real mission graph")
    check(r['ok'] is True, f"entered the cell ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['walked'] > 0 and r['commands'] > 0,
          f"{r['commands']} commands for {r['walked']:.1f} m = {r['walked'] / max(1, r['commands']):.2f} m per command")

    r = run("S10. REAL lattice + wall with gap: the path CURVES (multi-node is needed here)",
            [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)], (3.5, 3.5, 0.0), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.10,
          f"does not graze the wall passing through the gap (minimum distance {r['min_clear']:.2f} m)")
    print(f"       [MULTINODE] x{r['tags']['[MULTINODE]']}, [SHORTCUT] x{r['tags']['[SHORTCUT]']}, "
          f"nodes crossed by the multi-node moves: {r['stats'].get('multinode_nodes', 0)}")

    # Reconstruction of the situation of mission 08-10 16:35: narrow corridor and
    # goal BEHIND the robot, i.e. a ~180 degree rotation where it cannot turn. There
    # the loop made 8 turns in 3.2 s without moving. The robot must start OUTSIDE the goal
    # cell, otherwise the scenario passes without testing anything.
    r = run("S11. 0.80 m corridor, goal behind: rotation impossible (case 16:35)",
            [(0.0, 12.0, 0.0, 3.10), (0.0, 12.0, 3.90, 12.0)], (7.0, 3.5, 0.0), (0, 0))
    check(r['err'] is None, "no exception")
    loops = r['log'].count('[FRONTIER]')
    refusals = r['log'].count('deg needed towards node')
    check(refusals >= 1, f"the ~180 degree rotation in the corridor is refused ({refusals} times)")
    check(r['turns'] == 0, f"the robot never rotates in the corridor (rotations executed: {r['turns']})")
    check(r['ok'] is False and loops <= 8,
          f"the attempt closes in an orderly way and in few turns: {loops} turns, outcome {r['ok']} "
          f"(at 16:35 it made 8 in 3.2 s, and without a limit it could go on)")
    print(f"       [STALL] x{r['tags']['[STALL]']}, "
          f"rotation-refusal limit x{r['log'].count('legs discarded in this attempt')}, "
          f"'no alternative in the graph' x{r['log'].count('No alternative in the graph from where')}, "
          f"[TRAP] x{r['tags']['[TRAP]']}, [CONE] x{r['tags']['[CONE]']}")

    # S12: multi-node advance must pass the robot's TRUE heading
    # at every leg. With short edges multi-node triggers, and with a path
    # that curves a heading one leg old sends the robot off the path.
    parametri.PRM_MAX_EDGE_M = 0.70711
    r = run("S12. multi-node on a curving path: every leg goes where it was commanded",
            [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)], (3.5, 3.5, 0.0), (0, 1))
    parametri.PRM_MAX_EDGE_M = 1.25
    check(r['err'] is None, "no exception")
    check(r['stats'].get('multinode_moves', 0) >= 1,
          f"multi-node triggered ({r['stats'].get('multinode_moves', 0)} moves, "
          f"{r['stats'].get('multinode_nodes', 0)} nodes)")
    check(r['endpoint_error'] < 0.02,
          f"every leg arrives where it was commanded: maximum gap {r['endpoint_error']:.3f} m")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.10,
          f"does not graze the wall (minimum distance {r['min_clear']:.2f} m)")

    # S14: of the goal cell only a strip along the border near the robot is free; the
    # rest is already known occupied (memory: with 5 m cells the centre is outside the scan
    # window, and without memory it would stay unknown and be chosen). In front
    # of the obstacle the frontier stops the centre half a body short, just outside the cell:
    # it must count as arrival (ARRIVAL_OUTSIDE_CELL_M). On 22-10 16:04 the same case gave
    # "0.06 m to go ... still outside cell" until the block.
    r = run("S14. cell occupied except a strip along the border: arrival just outside",
            [(6.75, 12.0, 0.0, 12.0)], (5.0, 3.5, 0.0), (0, 1), memory=[(6.75, 12.0, 0.0, 12.0)])
    check(r['err'] is None, "no exception")
    tgt = [l for l in r['log'].splitlines() if l.startswith('[TARGET] Cell (0,1)')]
    check(bool(tgt) and 'using the geometric centre' not in tgt[0],
          f"goal in the free strip: {tgt[0][:120] if tgt else 'no [TARGET] line'}")
    check('GRAPH DISCONNECTED' not in r['log'], "the graph reaches the chosen goal")
    check(r['ok'] is True, f"the cell counts as reached (final position "
                           f"({r['final'][0]:.2f}, {r['final'][1]:.2f}), border at x = 6.00)")
    check(r['final'][0] >= 6.0 - parametri.ARRIVAL_OUTSIDE_CELL_M - 0.02,
          f"the robot is outside the cell by at most {parametri.ARRIVAL_OUTSIDE_CELL_M:.2f} m "
          f"({max(0.0, 6.0 - r['final'][0]):.2f} m)")
    check(r['min_clear'] > 0.0, f"does not enter the wall (minimum distance {r['min_clear']:.2f} m)")

    print("\n--- Comparison: how far the robot walks for every motion command ---")
    comparison = {}
    for label, maxe in (("edges 0.5-0.707 (mission 16:35)", 0.70711),
                            ("edges 0.5-1.25 (new)", 1.25)):
        parametri.PRM_MAX_EDGE_M = maxe
        r = run(f"S9. {label}", [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)],
                (3.5, 3.5, 0.0), (0, 1))
        comparison[label] = (r['commands'], r['walked'], r['stats'].get('multinode_moves', 0),
                                r['stats'].get('multinode_nodes', 0), r['scans'], r['ok'],
                                r['turns'], r['turn_deg'])
    parametri.PRM_MAX_EDGE_M = 1.25
    print()
    for k, (c, w, mm, mn, sc, ok, tn, td) in comparison.items():
        print(f"  {k:<34} {c:2d} commands, {w:4.1f} m -> {w / max(1, c):.2f} m/command; "
              f"rotations above 10 degrees: {tn} ({td:.0f} degrees in all); "
              f"multi-node {mm} ({mn} nodes); {sc} scans; arrived: {ok}")

    print("\n--- Comparison: lattice and random nodes (16 nodes/m2, seed 7), corridor of S9 ---")
    for label, kw in [("lattice", {})] + [
            (f"random {d}", dict(sampling='random', distribution=d, density_per_m2=16.0, seed=7))
            for d in ("uniform", "poisson", "jittered", "halton")]:
        GRAPH_KW.clear()
        GRAPH_KW.update(kw)
        r = run(f"S13. {label}", [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)],
                (3.5, 3.5, 0.0), (0, 1))
        check(r['err'] is None and r['ok'], f"{label}: arrived in the cell without exceptions")
        check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.10,
              f"{label}: does not graze the wall (minimum distance {r['min_clear']:.2f} m)")
        print(f"       {r['commands']} commands, {r['walked']:.1f} m, rotations above 10 degrees "
              f"{r['turns']} ({r['turn_deg']:.0f} degrees), {r['scans']} scans")
    GRAPH_KW.clear()

    # S15: an obstacle right BEYOND the goal (28-10 11:36, cell (0,2)). The frontier checks
    # the path extended by half a body for the nose, and stops on the obstacle: no
    # path towards that goal can pass. Before, legs were discarded one by one
    # ([SUBITO], up to 8) until the block, and on these same cases the code of the morning of 28-10
    # failed the cell 7 times out of 24 (seed 9: 143 commands). Random nodes as in the mission.
    obstacle_beyond = [(8.75, 9.15, 2.0, 5.0)]
    for label, seed, x0, wait in (("final step below the minimum step", 4, 8.85, 'step'),
                                        ("new goal at the first stop", 9, 8.75, 'wait')):
        GRAPH_KW.clear()
        GRAPH_KW.update(sampling='random', distribution='poisson', density_per_m2=4.0, seed=seed)
        r = run(f"S15. obstacle right beyond the goal: {label}",
                [(x0, x0 + 0.4, 2.0, 5.0)], (2.0, 3.0, 0.0), (0, 1))
        check(r['err'] is None and r['ok'] is True, f"{label}: cell reached without exceptions")
        check(r['tags']['[EARLY]'] == 0 and r['tags']['[BLOCK] Confirmed'] == 0,
              f"{label}: no [EARLY] nor block ({r['tags']['[EARLY]']}, "
              f"{r['tags']['[BLOCK] Confirmed']})")
        if wait == 'step':
            check('[FINAL STEP]' in r['log'], "the step below 0.40 m that is enough to arrive is taken")
        else:
            check("[TARGET] The frontier stops" in r['log'] and "beyond the end of the path" in r['log'],
                  "the frontier stopped beyond the goal makes it choose another goal at once")
        check(r['scans'] <= 9, f"{label}: {r['scans']} scans")

    # The same case with the wait rule removed: the confirmed block right next to
    # the goal must make it choose another goal, not replan towards the one
    # just excluded (28-10 11:36: "escluso", and right after a path to the same node).
    real_check = cell_attempt.CellAttempt._frontier_stops_at_goal

    def _only_in_block(self):
        return sys._getframe(1).f_code.co_name != '_repick_unreachable_goal' and real_check(self)
    cell_attempt.CellAttempt._frontier_stops_at_goal = _only_in_block
    early_saved = parametri.MAX_EARLY_REPLANS
    parametri.MAX_EARLY_REPLANS = 0
    try:
        GRAPH_KW.clear()
        GRAPH_KW.update(sampling='random', distribution='poisson', density_per_m2=4.0, seed=9)
        r = run("S15. obstacle right beyond the goal: block, then new goal",
                obstacle_beyond, (2.0, 3.0, 0.0), (0, 1))
    finally:
        cell_attempt.CellAttempt._frontier_stops_at_goal = real_check
        parametri.MAX_EARLY_REPLANS = early_saved
    check(r['err'] is None and r['ok'] is True, "block near the goal: cell reached")
    check("[TARGET] Block " in r['log'] and "[REPLAN] After the block" not in r['log'],
          "after the block right next to the goal another one is chosen, without replanning "
          "towards the excluded one")
    GRAPH_KW.clear()

    # S16: the check on the arrival pose (arrival_has_exit). A turnable direction that stops
    # because the data runs out is an exit: "not seen" does not mean "closed". On 28-10
    # 12:16 the same step in the corridor was refused three times in a row ("0/24
    # direzioni"), and executed at the third: it was the right one. First a direct test on two
    # hand-made grids, then a short mission in the corridor.
    print("\nS16. check on the arrival pose: corridor continuing beyond the data, blind niche")
    from scipy.ndimage import distance_transform_edt

    def _snap(occupied, seen, ax, ay, cs=0.03, n=128, ox=0.0, oy=-1.92):
        free_d = distance_transform_edt(~occupied) * cs
        occ_d = distance_transform_edt(occupied) * cs
        od = np.where(occupied, -occ_d, free_d)
        z = np.zeros((n, n))
        return safe_frontier.GridSnapshot(od, np.zeros((n, n), bool), seen, z, ox, oy, cs, ax, ay)
    n, cs = 128, 0.03
    xs = (np.arange(n) + 0.5) * cs
    ys = -1.92 + (np.arange(n) + 0.5) * cs
    X, Y = np.meshgrid(xs, ys)
    walls = (np.abs(Y) >= 0.45)                         # corridor 0.9 m wide along x
    seen_corr = X <= 2.2                                 # the scan ends at x = 2.2
    ok_corr, fan_corr = decisions.arrival_has_exit(_snap(walls, seen_corr, 2.0, 0.0), (2.0, 0.0), 0.0)
    fwd = [d for d in fan_corr if abs(d['rel_deg']) < 1.0][0]
    check(ok_corr, f"corridor continuing beyond the data: there is an exit (forward {fwd['advance']:.2f} m, "
                   f"{fwd['reason']}, turnable {fwd['rotatable']})")
    pocket = walls | (X >= 2.45)                         # same width, closed in front
    ok_pocket, fan_pocket = decisions.arrival_has_exit(_snap(pocket, np.ones((n, n), bool), 2.0, 0.0),
                                                (2.0, 0.0), 0.0)
    check(not ok_pocket, f"blind niche all seen: no exit "
                         f"({sum(d['usable'] for d in fan_pocket)}/{len(fan_pocket)} directions)")

    GRAPH_KW.clear()
    GRAPH_KW.update(sampling='random', distribution='poisson', density_per_m2=4.0, seed=1)
    r = run("S16. 0.9 m wide corridor: no empty [ARRIVAL] refusal",
            [(5.0, 8.0, 0.0, 3.0), (5.0, 8.0, 3.9, 12.0)], (2.0, 3.5, 0.0), (0, 1))
    GRAPH_KW.clear()
    check(r['err'] is None and r['ok'] is True, "cell reached through the corridor")
    check(r['tags']['[ARRIVAL]'] == 0, f"no [ARRIVAL] refusal ({r['tags']['[ARRIVAL]']}; "
                                      f"before correction 3, the last one then executed anyway)")
    check(r['scans'] <= 8, f"{r['scans']} scans (before 10)")

    # S17: command that expires a few cm from the point (28-10 13:50, cell (3,2): 0.42 m out of 0.44
    # uphill). Within MOVE_NEAR_GOAL_M it counts as arrived; farther it is a physical failure.
    STALL.update(n=1, short_m=0.03)
    r = run("S17. timeout 3 cm from the point: counts as arrived", [], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None and r['ok'] is True, "cell reached")
    check("[ALMOST]" in r['log'] and "failed mechanically" not in r['log'],
          "the leg counts as successful, no edge discarded for physical failure")
    STALL.update(n=1, short_m=0.30)
    r = run("S17. timeout 30 cm from the point: physical failure as before", [], (2.0, 2.0, 0.785), (0, 1))
    STALL.update(n=0)
    check(r['err'] is None, "no exception")
    check("[ALMOST]" not in r['log'] and "failed mechanically" in r['log'],
          "beyond the threshold it stays a physical failure")

    for d in [x for x in os.listdir(tempfile.gettempdir()) if x.startswith("mission_e2e_")]:
        shutil.rmtree(os.path.join(tempfile.gettempdir(), d), ignore_errors=True)
    print("\n" + "=" * 70)
    print("ALL OK" if not FAILS else f"{len(FAILS)} CHECKS FAILED:\n  - " + "\n  - ".join(FAILS))
    sys.exit(1 if FAILS else 0)
