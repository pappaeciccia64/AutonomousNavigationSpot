"""
End-to-end test of the REAL attempt (attempt_enter_cell_from_position in
mission/cell_attempt.py, the cruise) against a fake robot in a synthetic world.

What is fake: the robot (RTSpot: fake_spot.FakeSpot on the real-time clock, walking the
trajectories the cruise sends), the local grid (128x128 window of 3 cm following the robot,
with obstacle_distance computed from the world and ghost cells in the corners as in the real
data), the frontier thread (PlanTracker: the planning mode of the real tracker on the fake
grid), falls. Everything else is the real code: terrain correction, global maps, PRM, planning
thread, frontier, goals, blocks, data saving. The fakes here are shared by test_cruise and
test_mission.

    python3 tests/test_e2e_loop.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import threading
import time
import traceback
from types import SimpleNamespace as NS

import numpy as np
from scipy.ndimage import distance_transform_edt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # the spot_explorer package

import matplotlib  # noqa: E402
matplotlib.use("Agg")
from bosdyn.client.frame_helpers import BODY_FRAME_NAME     # noqa: E402
from fake_spot import FakeSpot     # noqa: E402
from spot_explorer.mission import attempt_cruise, attempt_scan, cell_attempt, planning_cycle     # noqa: E402
from spot_explorer import config
from spot_explorer import records
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.planning import safe_frontier
from spot_explorer.perception import low_obstacles   # noqa: E402
# The fake sensor sees through the obstacles: from the next cell a cell is "all seen" at once,
# and these scenarios are about entering it. In the fake world no cell counts as seen
# (attempt_common.CELL_SEEN_FRACTION); a scenario about that rule sets REAL_CELL_SEEN_FRACTION
# back for itself and restores the fake in a finally.
from spot_explorer.mission import cells as _cells     # noqa: E402
REAL_CELL_SEEN_FRACTION = _cells.cell_seen_fraction
_cells.cell_seen_fraction = lambda *a, **k: None
from spot_explorer.planning import roadmap
from spot_explorer.robot import motion
from spot_explorer.robot import connection

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
    if b == BODY_FRAME_NAME:
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

    def return_local_grid(self, layers, robot_state_client=None, max_attempts=3, timeout=None):
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


FALL = {'armed': False, 'fallen': False, 'recoverable': True, 'where': None}


def fake_check_fall(rsc):
    return dict(fallen=FALL['fallen'], reason="fall fault (test)" if FALL['fallen'] else "",
                roll=80.0 if FALL['fallen'] else 0.0, pitch=0.0, x=ROBOT.x, y=ROBOT.y,
                fall_faults=[1] if FALL['fallen'] else [], clearable=[1] if FALL['fallen'] else [],
                unclearable=[], motors_on=True, standing=not FALL['fallen'])


def fake_recover(cc, rsc, status=None):
    if not FALL['fallen']:
        return True
    if not FALL['recoverable']:
        raise motion.RobotFallenError("test: the robot does not get up by itself")
    FALL['fallen'] = False
    return True


def _yaw_of(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))


def fake_navigate_to(tx, ty, rx, ry, rsc, cc, vtb, should_abort=None, keep_heading=False, **kw):
    """The moves outside the cruise (the retreat after a fall), as the real navigate_to: turn
    by (target direction - heading read from vtb) relative to the true pose, then walk straight
    for the distance from (rx, ry), with should_abort queried along the way."""
    dist = float(np.hypot(tx - rx, ty - ry))
    if keep_heading:
        sx, sy, ex, ey = ROBOT.x, ROBOT.y, tx, ty
    else:
        target_yaw = float(np.arctan2(ty - ry, tx - rx))
        dyaw = float(np.arctan2(np.sin(target_yaw - _yaw_of(vtb.rotation)),
                                np.cos(target_yaw - _yaw_of(vtb.rotation))))
        ROBOT.yaw = float(np.arctan2(np.sin(ROBOT.yaw + dyaw), np.cos(ROBOT.yaw + dyaw)))
        sx, sy = ROBOT.x, ROBOT.y
        ex, ey = sx + dist * np.cos(ROBOT.yaw), sy + dist * np.sin(ROBOT.yaw)
    steps = max(2, int(dist / 0.05))
    for k in range(1, steps + 1):
        t = k / steps
        ROBOT.x, ROBOT.y = sx + t * (ex - sx), sy + t * (ey - sy)
        ROBOT.min_clear = min(ROBOT.min_clear, WORLD.od_at(ROBOT.x, ROBOT.y))
        ROBOT.track.append((ROBOT.x, ROBOT.y))
        if should_abort is not None and k % 2 == 0 and should_abort():
            return False, float(np.hypot(ROBOT.x - sx, ROBOT.y - sy)), dist
    return True, dist, dist


def fake_move_no_turn(tx, ty, frame, cc, rsc, **kw):
    sx, sy = ROBOT.x, ROBOT.y
    ROBOT.x, ROBOT.y = tx, ty
    ROBOT.track.append((tx, ty))
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
for _m in (attempt_scan, planning_cycle, spot_grid):
    if hasattr(_m, 'get_a_tform_b'):
        _m.get_a_tform_b = fake_get_a_tform_b
connection.getPosition = lambda rsc: (ROBOT.x, ROBOT.y, -2.7, quat(ROBOT.yaw))
# The moves outside the cruise (the retreat after a fall): straight legs on ROBOT.
motion.navigate_to = fake_navigate_to
motion.navigate_to_path = fake_navigate_to
motion.move_to_world_point_without_turning = fake_move_no_turn
config.FRONTIER_RESCAN_WAIT_S = 0.0
motion.check_fall = fake_check_fall
motion.recover_from_fall = fake_recover


class _NoTilt:
    def __init__(self, *a): pass
    def start(self): pass
    def stop(self):
        return dict(roll_start=0, pitch_start=0, roll_end=0, pitch_end=0, roll_peak_abs=0,
                    pitch_peak_abs=0, samples=0, errors=0)


attempt_cruise.BodyTiltRecorder = records.BodyTiltRecorder = _NoTilt


class PlanTracker(FakeTracker):
    """The frontier thread of the test world, with the planning mode of the real tracker."""
    AVT = safe_frontier.ArcVerificationTracker

    def __init__(self):
        super().__init__()
        self._lock, self._cycle_lock = threading.Lock(), threading.Lock()
        self._plan_cond = threading.Condition(self._lock)
        self._job, self._plan, self._plan_version, self._job_period_s = None, None, 0, 0.4
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    start_planning, stop_planning, get_plan, wait_plan, _planning_cycle = (
        AVT.start_planning, AVT.stop_planning, AVT.get_plan, AVT.wait_plan, AVT._planning_cycle)

    def _loop(self):
        while self._running:
            with self._lock:
                job, period = self._job, self._job_period_s
            if job is not None:
                self._planning_cycle(job, period)
            else:
                time.sleep(0.05)


class RTSpot(FakeSpot):
    """Real-time clock; pose shared with ROBOT (the moves outside the cruise, the retreat,
    move ROBOT through the fake navigate_to). on_step(x, y) is called at every step while a
    trajectory runs (an obstacle appearing, a fall)."""

    def __init__(self, on_step=None, track=None):
        super().__init__(ROBOT.x, ROBOT.y, ROBOT.yaw, blocked=lambda x, y: WORLD.od_at(x, y) < 0.10)
        self._t, self._l, self.on_step, self.rt_track = time.time(), threading.RLock(), on_step, track

    def _walking(self):
        c = self._cmds.get(self._active)
        return c is not None and c['points'] is not None and c['status'] == 1

    def _sync(self):
        """While a trajectory runs the fake Spot moves the robot; otherwise the robot is where
        the last move left it."""
        with self._l:
            now = time.time()
            if self._walking():
                self.advance(now - self._t)
                ROBOT.x, ROBOT.y, ROBOT.yaw = self.x, self.y, self.yaw
                ROBOT.track.append((self.x, self.y))
                ROBOT.min_clear = min(ROBOT.min_clear, WORLD.od_at(self.x, self.y))
                if self.rt_track is not None:
                    self.rt_track.append((self.x, self.y, now))
                if self.on_step is not None:
                    self.on_step(self.x, self.y)
            else:
                self.x, self.y, self.yaw = ROBOT.x, ROBOT.y, ROBOT.yaw
            self._t = now

    def robot_command(self, command=None, end_time_secs=None, lease=None, **kw):
        with self._l:
            self._sync()
            return super().robot_command(command, end_time_secs, lease)

    def robot_command_feedback(self, cid):
        self._sync()
        return super().robot_command_feedback(cid)

    def get_robot_state(self):
        self._sync()
        return super().get_robot_state()


def _person_steps_in(state):
    """on_step of S3: once the robot has walked 1 m, a 0.4 m box appears 0.85 m ahead of its
    centre along the direction of travel -- beyond half the body (0.55) plus the frontier
    margin (0.10), as in the stop-and-go version of this scenario."""
    def on_step(x, y):
        prev, state['prev'] = state.get('prev'), (x, y)
        if state.get('done') or prev is None or np.hypot(x - prev[0], y - prev[1]) < 1e-3:
            return
        sx, sy = state['start']
        if np.hypot(x - sx, y - sy) < 1.0:
            return
        state['done'] = True
        d = np.hypot(x - prev[0], y - prev[1])
        cx, cy = x + 0.85 * (x - prev[0]) / d, y + 0.85 * (y - prev[1]) / d
        WORLD.dynamic = (cx - 0.2, cx + 0.2, cy - 0.2, cy + 0.2)
        WORLD._recompute()
    return on_step


def _falls_halfway(start, target_cell):
    """on_step of S6/S7: the robot falls once it is halfway to the centre of the target cell."""
    cx, cy = FakeEnv().get_world_position_from_cell(*target_cell)
    half = 0.5 * float(np.hypot(cx - start[0], cy - start[1]))

    def on_step(x, y):
        if FALL['armed'] and np.hypot(x - start[0], y - start[1]) >= half:
            FALL.update(armed=False, fallen=True, where=(x, y))
    return on_step


def run(name, rects, start, target_cell, ghosts=True, dynamic=None, fall=None, second_attempt=False,
        memory=None, on_step=None, env=None):
    """One attempt on target_cell (a second one with second_attempt) in the world rects, the
    robot at start = (x, y, yaw), in cruise. dynamic: a person steps in front of the robot
    (_person_steps_in); fall: 'recuperabile' / 'irrecuperabile' halfway (_falls_halfway);
    memory: obstacles already in the global map."""
    global WORLD, ROBOT
    WORLD, ROBOT = World(rects, ghosts), Robot(*start)
    # The temporal obstacle filter (OBSTACLE_HISTORY) is aligned to the
    # WORLD and survives from one scan to the next: on the robot that is right, the world is
    # always the same. Here every scenario is a different world with the same coordinates, and
    # without resetting it the median mixed the previous scenario's scans with the
    # new ones: S11 passed or failed depending on what ran before.
    safe_frontier.OBSTACLE_HISTORY.reset()
    low_obstacles.HISTORY.reset()
    FALL.update(armed=fall is not None, fallen=False, recoverable=(fall == 'recuperabile'), where=None)
    records.MISSION_STATS['falls'] = records.MISSION_STATS['self_rights'] = 0
    for k in list(records.MISSION_STATS):
        if k.startswith('cruise'):
            del records.MISSION_STATS[k]
    steps = [f for f in (on_step,
                         _person_steps_in({'start': start[:2]}) if dynamic else None,
                         _falls_halfway(start, target_cell) if fall else None) if f is not None]
    # The real mission lattice: 0.25 m spacing, 0.5-1.25 m edges, on FakeEnv's 2 x 2 cells of
    # 5 m, for all scenarios. GRAPH_KW selects the random nodes.
    from spot_explorer.planning import mission_graph
    mg = mission_graph.build_mission_graph(
        rows=2, cols=2, cell_size_m=5.0, spacing_m=0.25, rings=None,
        origin_xy=(3.5, 3.5), origin_yaw=0.0, min_edge_length=config.PRM_MIN_EDGE_M,
        max_edge_length=config.PRM_MAX_EDGE_M,
        connection_radius=config.PRM_CONNECTION_RADIUS_M, verbose=False,
        **dict({'sampling': 'lattice'}, **GRAPH_KW))
    prm = roadmap.PRM(min_edge_length=config.PRM_MIN_EDGE_M,
                        max_edge_length=config.PRM_MAX_EDGE_M,
                        connection_radius=config.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg
    mg.into_prm(prm)
    env = FakeEnv() if env is None else env
    if getattr(env, '_cell_rules', None) is not None:
        env._cell_rules.prm = prm               # the cell rules of the run work on its graph
    folder = tempfile.mkdtemp(prefix="mission_e2e_")
    for k in list(records.MISSION_STATS['time_s']):
        records.MISSION_STATS['time_s'][k] = 0.0
    for k in ('scan_bundles', 'prm_refresh', 'prm_refresh_skipped'):
        if k in records.MISSION_STATS:
            records.MISSION_STATS[k] = 0
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
    rt_track = []
    spot = RTSpot((lambda x, y: [f(x, y) for f in steps]) if steps else None, track=rt_track)
    tracker = PlanTracker()
    out = io.StringIO()
    err, ok, ok2, track2_from = None, None, None, None
    t0 = time.time()

    def attempt(k):
        return cell_attempt.attempt_enter_cell_from_position(
            FakeLocalGrid(), gg, spot, spot, env, target_cell[0], target_cell[1],
            FakeSampler(), prm, mission_folder=folder, iteration=k,
            recordingInterface=None, verification_tracker=tracker)
    with contextlib.redirect_stdout(out):
        try:
            ok = attempt(0)
            if second_attempt:
                ROBOT.track.append((ROBOT.x, ROBOT.y))
                track2_from = len(ROBOT.track) - 1
                ok2 = attempt(1)
            attempt_cruise.stop_if_moving(env, spot, "end of the scenario")
        except motion.RobotFallenError as e:
            ok = f"RobotFallenError: {e}"
        except Exception:
            err = traceback.format_exc()
    tracker._running = False
    records.VIS_EXECUTOR.submit(lambda: None).result()      # wait for the saves
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
    tr = np.array(ROBOT.track, dtype=float).reshape(-1, 2)
    walked = float(np.sum(np.hypot(*np.diff(tr, axis=0).T))) if len(tr) > 1 else 0.0
    ends = [l.split('End: ')[1].split('. ')[0] for l in log.splitlines() if '[CRUISE] End:' in l]
    print(f"\n{name}  ({time.time() - t0:.1f} s)")
    if err:
        print(err)
    print(f"       outcome {ok}" + (f" / {ok2}" if second_attempt else "") + f", {walked:.1f} m, "
          f"minimum distance of the centre from obstacles {ROBOT.min_clear:.2f} m, scans {n_scans}, "
          f"cruise ends {ends}")
    return dict(ok=ok, err=err, log=log, scans=n_scans, folder=folder, ends=ends,
                min_clear=ROBOT.min_clear, final=(ROBOT.x, ROBOT.y), ok2=ok2, gg=gg, walked=walked,
                rt_track=rt_track, stats=dict(records.MISSION_STATS),
                track2=ROBOT.track[track2_from:] if track2_from is not None else [])


if __name__ == '__main__':
    print("Fake robot, real code, in cruise. Goal cell (0,1): centre (8.5, 3.5).")
    print("NB: the goal cell must be ADJACENT to a visited one -- since 2026-10-08\n    planning is limited to the explored cells (mission_prm.planning_cells).")
    clear_min = geometry.FRONTIER_CLEARANCE_M - 0.01

    r = run("S1. open field, ghost cells in the corners of every window (2026-10-05)",
            [], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is True, f"entered the cell, final position ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check('[CRUISE] Block:' not in r['log'] and '[RETREAT]' not in r['log'],
          "no block and no retreat on free terrain")
    check(r['scans'] > 0, f"{r['scans']} scan bundles saved")
    _b = np.load(os.path.join(r['folder'], 'scans', sorted(os.listdir(os.path.join(r['folder'], 'scans')))[0]))
    check('intensity' in _b.files and _b['intensity'].shape == (N, N) and 'intensity_origin' in _b.files
          and 'no_step' not in _b.files,
          "extra layers in the bundle: intensity saved with its origin, no_step absent without harm")
    check("[CRUISE] Plan" in r['log'], "[CRUISE] Plan lines present")

    r = run("S2. wall with a 1.2 m gap far from the straight line: it must be found by going around",
            [(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is True, f"entered the cell through the gap ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.05,
          f"minimum distance of the centre from obstacles along the whole way: {r['min_clear']:.2f} m")

    # A person steps in 0.85 m ahead of the centre (0.3 m from the nose): the obstacle filter
    # counts it from its second scan, ~0.6 s, so the cruise stops late. On the robot the brake is
    # Spot's own body obstacle avoidance, on in every command (only the foot one is off,
    # motion.py); this fake robot does not model it. So, as the 0.8 m box of test_cruise (user,
    # 26-11): the centre never inside the obstacle and the attempt ends cleanly, no distance
    # threshold for this scenario.
    r = run("S3. a person steps in front of the robot while it walks",
            [], (2.0, 2.0, 0.785), (0, 1), dynamic=True)
    check(r['err'] is None, "no exception")
    check(r['min_clear'] > 0.0,
          f"the centre never inside the obstacle that appeared during the move "
          f"(minimum distance of the centre {r['min_clear']:.2f} m; the body is Spot's obstacle avoidance)")
    check(r['ok'] is not None, f"the attempt ends cleanly ({r['ok']}, ends {r['ends']})")

    r = run("S4. goal cell closed by walls on all sides",
            [(6.1, 6.3, 1.0, 6.0), (6.1, 10.9, 1.0, 1.2), (10.7, 10.9, 1.0, 6.0), (6.1, 10.9, 5.8, 6.0)],
            (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is False, "attempt failed in an orderly way (cell unreachable)")
    check(r['min_clear'] > 0.0, f"never enters the wall (minimum distance {r['min_clear']:.2f} m)")

    r = run("S5. start in a tight space: 0.45 m from the wall, target at 90 degrees",
            [(2.45, 2.65, 0.0, 6.0)], (2.0, 3.0, 1.5708), (0, 0))
    check(r['err'] is None, "no exception")
    check(r['min_clear'] > 0.0, f"never enters the wall (minimum distance {r['min_clear']:.2f} m)")

    r = run("S6. fall halfway through a move, the robot gets up; then a second attempt",
            [], (2.0, 2.0, 0.785), (0, 1), fall='recuperabile', second_attempt=True)
    check(r['err'] is None, "no exception")
    check(FALL['where'] is not None and records.MISSION_STATS['self_rights'] == 1,
          "fall detected and robot up again")
    if FALL['where'] is not None:
        fx, fy = FALL['where']
        gm = r['gg'].global_occupancy_map
        check(gm is not None and gm.is_occupied(fx, fy),
              f"fall point ({fx:.2f}, {fy:.2f}) occupied in the global map")
    check(r['ok2'] is not None, f"second attempt made ({r['ok2']})")

    r = run("S7. fall from which the robot does NOT get up by itself",
            [], (2.0, 2.0, 0.785), (0, 1), fall='irrecuperabile')
    check(r['err'] is None and isinstance(r['ok'], str) and r['ok'].startswith("RobotFallenError"),
          "the mission stops with RobotFallenError (main() catches it and calls the operator)")
    check("[RETREAT]" not in r['log'], "no attempt to move from the ground")

    r = run("S8. REAL lattice (spacing 0.25, edges 0.5-1.25) on open field",
            [], (3.5, 3.5, 0.0), (0, 1))
    check(r['err'] is None, "no exception with the real mission graph")
    check(r['ok'] is True, f"entered the cell ({r['final'][0]:.2f}, {r['final'][1]:.2f})")

    r = run("S10. REAL lattice + wall with gap: the path CURVES",
            [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)], (3.5, 3.5, 0.0), (0, 1))
    check(r['err'] is None, "no exception")
    check(r['ok'] is True, "entered the cell through the gap")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.10,
          f"does not graze the wall passing through the gap (minimum distance {r['min_clear']:.2f} m)")

    # Reconstruction of the situation of mission 08-10 16:35: narrow corridor and goal BEHIND
    # the robot, a ~180 degree rotation where it cannot turn. The cruise walks backwards (the
    # heading rule, CruisePlanner._turns_within) or leaves the cell: either way the attempt ends,
    # in few seconds, and the body never enters the walls. The robot starts OUTSIDE the goal
    # cell, otherwise the scenario passes without testing anything.
    r = run("S11. 0.80 m corridor, goal behind: rotation impossible (case 16:35)",
            [(0.0, 12.0, 0.0, 3.10), (0.0, 12.0, 3.90, 12.0)], (7.0, 3.5, 0.0), (0, 0))
    check(r['err'] is None, "no exception")
    check(r['ok'] is not None, f"the attempt ends ({r['ok']}, ends {r['ends']})")
    check(r['min_clear'] > 0.0, f"never enters the walls (minimum distance {r['min_clear']:.2f} m)")
    print(f"       walked {r['walked']:.1f} m" + (" backwards" if '| backward' in r['log'] or 'tail first' in r['log'] else ""))

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
    check('[NO PATH]' not in r['log'], "the graph reaches the chosen goal")
    check(r['ok'] is True, f"the cell counts as reached (final position "
                           f"({r['final'][0]:.2f}, {r['final'][1]:.2f}), border at x = 6.00)")
    check(r['final'][0] >= 6.0 - config.ARRIVAL_OUTSIDE_CELL_M - 0.02,
          f"the robot is outside the cell by at most {config.ARRIVAL_OUTSIDE_CELL_M:.2f} m "
          f"({max(0.0, 6.0 - r['final'][0]):.2f} m)")
    check(r['min_clear'] > 0.0, f"does not enter the wall (minimum distance {r['min_clear']:.2f} m)")

    print("\n--- Lattice and random nodes (16 nodes/m2, seed 7), wall with a gap ---")
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
    GRAPH_KW.clear()

    # S15: an obstacle right BEYOND the goal (28-10 11:36, cell (0,2)). The frontier checks
    # the path extended by half a body for the nose, and stops on the obstacle: no path
    # towards that goal can pass. Random nodes as in the mission; three graphs.
    for seed, x0 in ((0, 8.85), (1, 8.75), (9, 8.75)):
        GRAPH_KW.clear()
        GRAPH_KW.update(sampling='random', distribution='poisson', density_per_m2=4.0, seed=seed)
        r = run(f"S15. obstacle right beyond the goal (seed {seed})",
                [(x0, x0 + 0.4, 2.0, 5.0)], (2.0, 3.0, 0.0), (0, 1))
        check(r['err'] is None and r['ok'] is True, f"seed {seed}: cell reached without exceptions")
        check(r['min_clear'] > 0.0, f"seed {seed}: never enters the obstacle ({r['min_clear']:.2f} m)")
    GRAPH_KW.clear()

    # S16: 0.9 m wide corridor towards the goal (28-10 12:16). 5 nodes/m2 as in the mission:
    # with the threshold of 0.30 m (09-11) the centre of the robot has a band of 0.30 m in this
    # corridor.
    GRAPH_KW.update(sampling='random', distribution='poisson', density_per_m2=5.0, seed=1)
    r = run("S16. 0.9 m wide corridor",
            [(5.0, 8.0, 0.0, 3.0), (5.0, 8.0, 3.9, 12.0)], (2.0, 3.5, 0.0), (0, 1))
    GRAPH_KW.clear()
    check(r['err'] is None and r['ok'] is True, "cell reached through the corridor")
    check(r['min_clear'] >= geometry.FRONTIER_CLEARANCE_M - 0.10,
          f"does not graze the corridor walls (minimum distance {r['min_clear']:.2f} m)")

    # S18: Spot does not answer the scan request (16-11, spot_grid.SCAN_TIMEOUT_S): the download
    # ends at once with the cause, no fall back to one request per layer, the single request is
    # still used afterwards; the planning cycle ends 'no_scan' with that cause.
    print("\n== S18. scan request without an answer: short timeout, the cause in the plan ==")
    from bosdyn.client.exceptions import TimedOutError
    from spot_explorer.mission import planning_cycle
    calls = []

    def silent(names, **kw):
        calls.append((tuple(names), kw.get('timeout')))
        raise TimedOutError(None, "fake: no answer")
    lg = spot_grid.LocalGrid.__new__(spot_grid.LocalGrid)
    lg.local_grid_client = NS(get_local_grids=silent)
    with contextlib.redirect_stdout(io.StringIO()):
        scan = lg.fetch_scan(None, timeout=spot_grid.SCAN_TIMEOUT_S, max_attempts=1)
    check(scan is None and "no answer from Spot" in (lg.last_fetch_problem or ''),
          f"no scan, the cause kept ({lg.last_fetch_problem})")
    check(len(calls) == 1 and calls[0][1] == spot_grid.SCAN_TIMEOUT_S,
          f"one request, with the short timeout ({calls})")
    check(getattr(lg, '_multi_request_ok', True) and getattr(lg, '_multi_request_fails', 0) == 0,
          "the single request for all the layers is not given up for a timeout")
    planner = planning_cycle.CruisePlanner.__new__(planning_cycle.CruisePlanner)
    planner.local_grid, planner.robot_state_client = lg, None
    with contextlib.redirect_stdout(io.StringIO()):
        plan = planner()
    check(plan['status'] == 'no_scan' and "no answer from Spot" in plan.get('error', ''),
          f"planning cycle: 'no_scan' with the cause ({plan.get('status')}: {plan.get('error')})")

    for d in [x for x in os.listdir(tempfile.gettempdir()) if x.startswith("mission_e2e_")]:
        shutil.rmtree(os.path.join(tempfile.gettempdir(), d), ignore_errors=True)
    print("\n" + "=" * 70)
    print("ALL OK" if not FAILS else f"{len(FAILS)} CHECKS FAILED:\n  - " + "\n  - ".join(FAILS))
    sys.exit(1 if FAILS else 0)
