"""
End-to-end test of the CRUISE (continuous motion, attempt_cruise.py) with the real planning
thread: the worlds and the fake local grid of test_e2e_loop.py, the fake Spot of fake_spot.py
moving the robot in REAL time (the planning cycles take their real time), the frontier thread
in planning mode (safe_frontier.ArcVerificationTracker.start_planning).

    python3 tests/test_cruise.py            # ~4 minutes

Scenarios: open field; wall with a gap (path changes while walking, distance from the wall
corner); a box appearing in front of the robot during the cruise; goal isolated inside a box
(new goal while walking); obstacle right beyond the goal (hand-off within the arrival
tolerance); three cells in a row (hand-offs, big turns without stopping); a fall halfway.
"""
import contextlib
import io
import os
import sys
import tempfile
import threading
import time
import traceback
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), HERE]

import test_e2e_loop as e2e                                                    # noqa: E402
from fake_spot import FakeSpot                                                 # noqa: E402
from spot_explorer import config, records                                      # noqa: E402
from spot_explorer.mission import attempt_cruise, cell_attempt, planning_cycle  # noqa: E402
from spot_explorer.perception import geometry                                  # noqa: E402
from spot_explorer.planning import edge_evaluation, mission_graph, roadmap, safe_frontier   # noqa: E402

# test_e2e_loop pins the switches off for the stop-and-go: here the cruise is on, as in the field.
attempt_cruise.CONTINUOUS_MOTION = True
attempt_cruise.CHAIN_CELLS = True
edge_evaluation.INCREMENTAL_REFRESH = True
planning_cycle.get_a_tform_b = e2e.fake_get_a_tform_b
attempt_cruise.BodyTiltRecorder = e2e._NoTilt

FAILS = []
TRACK = []          # (x, y, t) of the fake Spot while it commands the robot


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


class PlanTracker(e2e.FakeTracker):
    """The frontier thread of the test world, with the planning mode of the real tracker."""
    AVT = safe_frontier.ArcVerificationTracker

    def __init__(self):
        super().__init__()
        self._lock, self._cycle_lock = threading.Lock(), threading.Lock()
        self._job, self._plan, self._plan_version, self._job_period_s = None, None, 0, 0.4
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    start_planning, stop_planning, get_plan, _planning_cycle = (AVT.start_planning, AVT.stop_planning,
                                                                AVT.get_plan, AVT._planning_cycle)

    def _loop(self):
        while self._running:
            with self._lock:
                job, period = self._job, self._job_period_s
            if job is not None:
                self._planning_cycle(job, period)
            else:
                time.sleep(0.05)


class RTSpot(FakeSpot):
    """Real-time clock; pose shared with e2e.ROBOT (the stop-and-go legs move ROBOT through the
    fake navigate_to). on_step(x, y) is called at every step (a box appearing)."""

    def __init__(self, on_step=None):
        super().__init__(e2e.ROBOT.x, e2e.ROBOT.y, e2e.ROBOT.yaw,
                         blocked=lambda x, y: e2e.WORLD.od_at(x, y) < 0.10)
        self._t, self._l, self.on_step = time.time(), threading.RLock(), on_step

    def _walking(self):
        c = self._cmds.get(self._active)
        return c is not None and c['points'] is not None and c['status'] == 1

    def _sync(self):
        """While a trajectory runs the fake Spot moves the robot; otherwise the robot is where
        the stop-and-go (fake navigate_to of test_e2e_loop) left it."""
        with self._l:
            now = time.time()
            if self._walking():
                self.advance(now - self._t)
                e2e.ROBOT.x, e2e.ROBOT.y, e2e.ROBOT.yaw = self.x, self.y, self.yaw
                e2e.ROBOT.track.append((self.x, self.y))
                e2e.ROBOT.min_clear = min(e2e.ROBOT.min_clear, e2e.WORLD.od_at(self.x, self.y))
                TRACK.append((self.x, self.y, now))
                if self.on_step is not None:
                    self.on_step(self.x, self.y)
            else:
                self.x, self.y, self.yaw = e2e.ROBOT.x, e2e.ROBOT.y, e2e.ROBOT.yaw
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


def run(name, rects, start, cells_seq, on_step=None):
    print(f"\n== {name} ==")
    e2e.WORLD, e2e.ROBOT = e2e.World(rects, True), e2e.Robot(*start)
    TRACK.clear()
    safe_frontier.OBSTACLE_HISTORY.reset()
    for k in list(records.MISSION_STATS):
        if k.startswith('cruise'):
            del records.MISSION_STATS[k]
    mg = mission_graph.build_mission_graph(
        rows=2, cols=2, cell_size_m=5.0, spacing_m=0.25, rings=None, origin_xy=(3.5, 3.5), origin_yaw=0.0,
        min_edge_length=config.PRM_MIN_EDGE_M, max_edge_length=config.PRM_MAX_EDGE_M,
        connection_radius=config.PRM_CONNECTION_RADIUS_M, verbose=False, sampling='lattice')
    prm = roadmap.PRM(min_edge_length=config.PRM_MIN_EDGE_M, max_edge_length=config.PRM_MAX_EDGE_M,
                      connection_radius=config.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg
    mg.into_prm(prm)
    spot, tracker = RTSpot(on_step), PlanTracker()
    gg = NS(global_occupancy_map=None, global_terrain_map=None)
    env = e2e.FakeEnv()
    folder = tempfile.mkdtemp(prefix="cruise_")
    os.makedirs(os.path.join(folder, 'scans'))
    out, oks, err, t0 = io.StringIO(), [], None, time.time()
    with contextlib.redirect_stdout(out):
        try:
            for k, cell in enumerate(cells_seq):
                ok = cell_attempt.attempt_enter_cell_from_position(
                    e2e.FakeLocalGrid(), gg, spot, spot, env, cell[0], cell[1], e2e.FakeSampler(), prm,
                    mission_folder=folder, iteration=k, recordingInterface=None, verification_tracker=tracker)
                oks.append(bool(ok))
                if ok:
                    env.mark_cell_visited(*cell)
            attempt_cruise.stop_if_moving(env, spot, "end of the scenario")
        except Exception:
            err = traceback.format_exc()
    tracker._running = False
    records.VIS_EXECUTOR.submit(lambda: None).result()
    log = out.getvalue()
    with open(os.path.join(folder, 'log.txt'), 'w') as f:
        f.write(log)
    tr = np.array(TRACK) if TRACK else np.zeros((0, 3))
    # Cruise and stop-and-go legs (the fake navigate_to keeps ROBOT.min_clear too).
    clear = float(e2e.ROBOT.min_clear)
    dt = np.diff(tr[:, 2]) if len(tr) > 1 else np.zeros(0)
    v = np.hypot(np.diff(tr[:, 0]), np.diff(tr[:, 1])) / np.maximum(dt, 1e-3) if len(tr) > 1 else np.zeros(0)
    res = dict(log=log, err=err, oks=oks, clear=clear, secs=time.time() - t0,
               stats={k: v_ for k, v_ in records.MISSION_STATS.items() if k.startswith('cruise')},
               bundles=len([f for f in os.listdir(os.path.join(folder, 'scans')) if 'cruise' in f]),
               standing=float(np.sum(dt[v < 0.05])) if len(v) else 0.0,
               ends=[l.split('End: ')[1].split('. ')[0] for l in log.splitlines() if '[CRUISE] End:' in l])
    print(f"  log: {os.path.join(folder, 'log.txt')}")
    print(f"  {res['secs']:.0f} s, cells {oks}, minimum distance of the centre from obstacles {clear:.2f} m, "
          f"standing {res['standing']:.1f} s, {res['stats'].get('cruises', 0)} cruises, ends {res['ends']}")
    check(err is None, "no exception" + ("" if err is None else ":\n" + err))
    check("Traceback" not in log, "no traceback in the log")
    return res


def main():
    t_all = time.time()
    clear_min = geometry.FRONTIER_CLEARANCE_M - 0.01

    r = run("open field", [], (2.0, 2.0, 0.785), [(0, 1)])
    check(r['oks'] == [True], "entered the cell")
    check(r['stats'].get('cruises') == 1, f"one cruise from the start to the goal ({r['stats'].get('cruises')})")
    walked, secs = r['stats'].get('cruise_m', 0), r['stats'].get('cruise_s', 1)
    check(walked / secs > 0.3, f"cruise speed {walked / secs:.2f} m/s (above 0.3)")
    check(r['bundles'] >= 8, f"scan bundles of the cruise saved ({r['bundles']})")
    check(r['stats'].get('cruise_max_dev_cm', 99) < 5, f"on the line ({r['stats'].get('cruise_max_dev_cm')} cm)")

    r = run("wall with a 1.2 m gap: path changes while walking",
            [(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), [(0, 1)])
    check(r['oks'] == [True], "entered the cell through the gap")
    check(r['stats'].get('cruise_replans', 0) > 0, f"path changed while walking ({r['stats'].get('cruise_replans')})")
    check(r['clear'] >= clear_min, f"centre at least {clear_min:.2f} m from the wall corner ({r['clear']:.2f})")

    # A 0.4 m box appears 1.2 m ahead of the robot when it passes x = 4 m (heading to 8.5, 3.5).
    state = {'done': False}

    def box(x, y):
        if not state['done'] and x > 4.0:      # during a cruise: on_step is called only while walking
            state['done'] = True
            cx, cy = x + 1.2, y + 0.3 * 1.2 / 6.5
            e2e.WORLD.dynamic = (cx - 0.2, cx + 0.2, cy - 0.2, cy + 0.2)
            e2e.WORLD._recompute()
    r = run("a box appears 1.2 m ahead during the cruise", [], (2.0, 2.0, 0.785), [(0, 1)], on_step=box)
    check(state['done'], "the box appeared")
    check(r['clear'] >= geometry.ROBOT_HALF_WIDTH_M, f"never closer to the box than half the body "
                                                     f"({r['clear']:.2f} m, limit {geometry.ROBOT_HALF_WIDTH_M})")
    check(r['oks'] == [True], "entered the cell anyway (around the box)")

    r = run("goal inside a box: new goal while walking", [(8.25, 8.75, 3.25, 3.75)], (2.0, 2.0, 0.785), [(0, 1)])
    check(r['oks'] == [True], "entered the cell")
    check(r['stats'].get('cruise_goal_repicks', 0) >= 1, f"new goal chosen while walking "
                                                         f"({r['stats'].get('cruise_goal_repicks', 0)})")

    r = run("obstacle right beyond the goal, then the next cell", [(8.7, 9.3, 3.0, 4.0)], (2.0, 2.0, 0.785),
            [(0, 1), (1, 1)])
    check(r['oks'] == [True, True], "both cells entered")
    check(r['stats'].get('cruise_handoffs', 0) >= 1, f"next cell while walking ({r['stats'].get('cruise_handoffs', 0)})")

    r = run("three cells in a row: hand-offs and big turns", [(1.0, 6.0, 5.9, 6.1)], (2.0, 2.0, 0.785),
            [(0, 1), (1, 0), (1, 1)])
    # Known (1 time out of 6 on 03-11): the cruise stops before the end of the wall because the
    # rotation there is not clear, and the stop-and-go does not get into (1,0) from that pose;
    # here there are no retries nor GraphNav as in the mission. The log of the scenario says why.
    check(r['oks'] == [True, True, True], "three cells entered")
    check(r['stats'].get('cruise_handoffs', 0) >= 2, f"hand-offs while walking ({r['stats'].get('cruise_handoffs', 0)})")
    big = [l for l in r['log'].splitlines() if 'inside the new command, without stopping first' in l]
    check(len(big) >= 1, f"big turn towards the next cell without stopping first ({len(big)})")
    check(r['clear'] >= clear_min, f"centre at least {clear_min:.2f} m from the wall ({r['clear']:.2f})")

    # The robot falls halfway (recoverable): the cruise ends at once, the stop-and-go gets it up.
    fall = {'done': False, 'xy': None}

    def falls(x, y):
        if not fall['done'] and x > 4.5:
            fall['done'], fall['xy'] = True, (x, y)
            e2e.FALL.update(fallen=True, recoverable=True, where=(x, y))
    r = run("the robot falls during the cruise", [], (2.0, 2.0, 0.785), [(0, 1)], on_step=falls)
    e2e.FALL.update(fallen=False)
    ends = [e for e in r['ends'] if e.startswith('fall')]
    check(fall['done'] and len(ends) == 1, f"the cruise ended for the fall ({r['ends']})")
    walked_after = [np.hypot(x - fall['xy'][0], y - fall['xy'][1]) for x, y, _ in TRACK
                    if fall['xy'] is not None] if fall['done'] else [0.0]
    check(max(walked_after) < 3.0, "no wandering after the fall")
    check("[FALL]" in r['log'] or "fall" in r['log'].lower(), "fall handled by the stop-and-go (log)")

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)} ({time.time() - t_all:.0f} s)")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
