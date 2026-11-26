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
import atexit
import contextlib
import io
import os
import shutil
import sys
import tempfile
import time
import traceback
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), HERE]

import test_e2e_loop as e2e                                                    # noqa: E402
from spot_explorer import config, records                                      # noqa: E402
from spot_explorer.mission import attempt_common, attempt_cruise, cell_attempt, cells, planning_cycle  # noqa: E402
from spot_explorer.perception import geometry                                  # noqa: E402
from spot_explorer.planning import edge_evaluation, mission_graph, roadmap, safe_frontier   # noqa: E402
from spot_explorer.perception import low_obstacles   # noqa: E402


FAILS = []
TRACK = []          # (x, y, t) of the fake Spot while it commands the robot



# Every run writes its log and scan bundles under one temporary folder, removed when the test
# ends unless KEEP_TEST_DATA is set (each run is ~5 MB, and the test makes dozens).
RUN_DIR = tempfile.mkdtemp(prefix="test_cruise_")
if not os.environ.get('KEEP_TEST_DATA'):
    atexit.register(shutil.rmtree, RUN_DIR, ignore_errors=True)

def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


PlanTracker = e2e.PlanTracker


class RTSpot(e2e.RTSpot):
    """The real-time fake Spot of test_e2e_loop, its walked track in TRACK."""

    def __init__(self, on_step=None):
        super().__init__(on_step, track=TRACK)


def run(name, rects, start, cells_seq, on_step=None, env=None):
    print(f"\n== {name} ==")
    e2e.WORLD, e2e.ROBOT = e2e.World(rects, True), e2e.Robot(*start)
    TRACK.clear()
    safe_frontier.OBSTACLE_HISTORY.reset()
    low_obstacles.HISTORY.reset()
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
    if getattr(env, '_cell_rules', None) is not None:
        env._cell_rules.prm = prm               # the cell rules of the run work on its graph
    spot, tracker = RTSpot(on_step), PlanTracker()
    gg = NS(global_occupancy_map=None, global_terrain_map=None)
    env = e2e.FakeEnv() if env is None else env
    folder = tempfile.mkdtemp(prefix="cruise_", dir=RUN_DIR)
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
            attempt_cruise.stop_if_moving(env, "end of the scenario")
        except Exception:
            err = traceback.format_exc()
    tracker._running = False
    planning_cycle.BUNDLE_SAVER.drain()                  # the bundles of the cruise
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


def check_cell_switch():
    """attempt_cruise.CellSwitch, the change of goal or cell while walking, case by case."""
    print("\n== CellSwitch: the change of goal or cell while walking ==")
    applied = []
    cruise = NS(env=NS(_attempt_left_unreachable=False), target_row=0, target_col=1,
                _prepare_next_cell=lambda *a, **k: None, _apply_switch=lambda sw: applied.append(sw['kind']))
    planner = NS(switch_job=None, switch_info='x')
    sw = attempt_cruise.CellSwitch(cruise, planner)
    check(sw.take({'switch': None}) is None and sw.take({'switch': {'kind': 'start', 'id': 1}}) is None,
          "no switch, or the attempt start: nothing to apply")
    sw.ask("seen 90%", "")
    check(sw.pending and planner.switch_job is not None, "cell done: the next cell asked to the thread")
    check(sw.late(sw.t0 + 1.0) is None, "waiting: in time")
    check(sw.late(sw.t0 + attempt_cruise.CRUISE_SWITCH_WAIT_S + 0.1) is not None and planner.switch_job is None,
          "waiting: beyond the wait, the hand-off")
    end = sw.apply({'kind': 'end', 'id': 2, 'reason': 'no goal', 'left': (0, 1)})
    check(end == ('no goal', False, True) and cruise.env._attempt_left_unreachable and planner.switch_info is None,
          f"'end': the cruise ends, the cell left as unreachable now ({end})")
    check(sw.take({'switch': {'kind': 'end', 'id': 2}}) is None, "a switch already applied is not taken again")
    check(sw.apply({'kind': 'goal', 'id': 3, 'cell': (0, 1), 'done': None}) is None and applied[-1] == 'goal'
          and sw.report, "path lost: the new goal applied, the cruise goes on")
    sw.ask("0.40 m from the goal", "in the cell, ")
    end = sw.apply({'kind': 'cell', 'id': 4, 'done': (0, 1), 'next': None, 'why': '0.40 m from the goal',
                    'where': 'in the cell, ', 'reason': 'far: GraphNav'})
    check(end is not None and end[1] and not sw.pending, f"cell done, the next not in cruise: hand-off ({end})")
    sw.ask("0.40 m from the goal", "in the cell, ")
    check(sw.apply({'kind': 'cell', 'id': 5, 'done': (0, 1), 'next': (1, 1)}) is None and applied[-1] == 'cell'
          and not sw.pending, "cell done, the next one prepared: applied, the cruise goes on")


def check_bundle_saver():
    """planning_cycle.BundleSaver: the planner hands bundles over without waiting; the saver
    asks the extra layers and writes; behind, the oldest is dropped; drain writes the rest."""
    print("\n== Bundle saver: the planner never waits for Spot or the disk ==")
    folder = tempfile.mkdtemp(prefix="saver_", dir=RUN_DIR)

    class SlowGrid:
        def extra_layers(self, names):
            time.sleep(0.2)                       # a slow request to Spot
            return {'intensity': dict(values=np.ones((2, 2), np.float32), origin=np.zeros(2), cell=0.03)}
    saver = planning_cycle.BundleSaver()
    drops0 = records.MISSION_STATS.get('bundle_drops', 0)
    n = planning_cycle.BUNDLE_QUEUE_MAX + 4
    t0 = time.time()
    for k in range(n):
        saver.put(os.path.join(folder, f"b{k:02d}.npz"), SlowGrid(), dict(time=float(k), x=np.arange(3)))
    put_s = time.time() - t0
    check(put_s < 0.1, f"{n} bundles handed over in {put_s * 1000:.0f} ms: the planner does not wait")
    check(saver.drain(10.0), "drain: everything handed over is written")
    files = sorted(os.listdir(folder))
    drops = records.MISSION_STATS.get('bundle_drops', 0) - drops0
    check(drops >= 1 and len(files) + drops == n and files[-1] == f"b{n - 1:02d}.npz",
          f"behind: the oldest dropped ({drops}), the newest written ({len(files)} files)")
    d = np.load(os.path.join(folder, files[-1]))
    check('intensity' in d.files and 'intensity_origin' in d.files and float(d['extra_time']) >= float(d['time'])
          and list(d['x']) == [0, 1, 2], "same arrays, the extra layers and their time added")


def check_heading_rule():
    """CruisePlanner._turns_within: forward if the rotation is clear, otherwise backward."""
    print("\n== Heading rule: forward if the rotation is clear, otherwise backward ==")
    cs, ox, oy = 0.05, -4.0, -2.0
    xs = ox + cs * (np.arange(120) + 0.5)
    ys = oy + cs * (np.arange(80) + 0.5)
    X, Y = np.meshgrid(xs, ys)
    free = NS(obstacle_dist=np.full(X.shape, 5.0), origin_x=ox, origin_y=oy, cell_size=cs)
    # Dead-end corridor along x, 0.7 m wide, closed at x = 1.0: Spot cannot turn round in it.
    od = np.where(np.abs(Y) < 0.35, np.minimum(0.35 - np.abs(Y), np.maximum(1.0 - X, 0.0)), 0.0)
    corridor = NS(obstacle_dist=od, origin_x=ox, origin_y=oy, cell_size=cs)
    f = planning_cycle.CruisePlanner._turns_within
    back_out = [(0.3, 0.0), (-1.0, 0.0), (-2.5, 0.0)]          # robot nose to the dead end, path out
    adv, stop, back = f(back_out, [1, 2], 2.8, free, 0.0)
    check(stop is None and back == [False, False], f"open field: turns round and walks forward ({back})")
    adv, stop, back = f(back_out, [1, 2], 2.8, corridor, 0.0)
    check(stop is None and adv == 2.8 and back == [True, True], f"dead end: out backwards, no rotation ({back})")
    adv, stop, back = f([(0.3, 0.0), (0.3, 0.25)], [1], 0.25, corridor, 0.0)
    check(stop == 'start' and adv == 0.0, f"node beside the robot in the corridor: 90 deg neither way, the stretch stops ({stop})")
    # Corridor opening into the open at x < -1.5: out backwards, round at the first node outside.
    od2 = np.where(X < -1.5, 5.0, od)
    opening = NS(obstacle_dist=od2, origin_x=ox, origin_y=oy, cell_size=cs)
    path = [(0.3, 0.0), (-2.5, 0.0), (-2.5, 1.5)]
    adv, stop, back = f(path, [1, 2], 4.3, opening, 0.0)
    check(stop is None and back == [True, False], f"backwards out of the corridor, forward once it can turn ({back})")


def check_closed_edge():
    """planning_cycle.closed_edge: the edge closed in the graph when the frontier closes the path."""
    print("\n== The path closed by the frontier: which edge goes out of the graph ==")
    poly = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]   # robot, then nodes 10..13
    robot = edge_evaluation.TEMP_NODE
    ids = [robot, 10, 11, 12, 13]

    def f(dist, reason='obstacle'):
        return planning_cycle.closed_edge(poly, ids, {'dist': dist, 'reason': reason, 'path_len': 4.0})
    check(f(0.5) == (robot, 10), f"closed on the stretch the robot is walking: the robot's link ({f(0.5)})")
    check(f(1.5) == (10, 11), f"closed on the second edge: 10->11 ({f(1.5)})")
    check(f(2.5, 'slope') == (11, 12), "closed by slope on the third edge: 11->12")
    check(f(3.5) is None, "closed within the arrival tolerance of the goal: none, it is the arrival")
    check(f(1.5, 'end_of_data') is None and f(4.0, 'end_of_path') is None, "missing data, end of path: none")


def main():
    t_all = time.time()
    clear_min = geometry.FRONTIER_CLEARANCE_M - 0.01
    check_cell_switch()
    check_closed_edge()
    check_heading_rule()
    check_bundle_saver()

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

    # A 0.4 m box appears on the way when the robot passes x = 4 m (heading to 8.5, 3.5), its
    # near face `face` metres ahead along the direction of travel.
    #  - 1.8 m: where Spot sees it in time (grid of 1.92 m around the robot). The planning must
    #    go around it: the centre never closer than half the body.
    #  - 0.8 m: appearing at the last moment (a person stepping in). The obstacle filter (median
    #    of 3 scans) counts it from its second scan, ~0.6 s: the robot stops late, with the nose
    #    into the box -- there the emergency brake is Spot's own body obstacle avoidance (on in
    #    every command, not in this fake robot; user, 12-11). Checked here: no way on is a block
    #    at once (the goal left, the next one chosen in the same cruise), the centre never inside
    #    the box, the attempt ends cleanly.
    for face, realistic in ((1.8, True), (0.8, False)):
        state = {'done': False, 'prev': None}

        def box(x, y, face=face, state=state):
            prev, state['prev'] = state['prev'], (x, y)
            if state['done'] or x <= 4.0 or prev is None or np.hypot(x - prev[0], y - prev[1]) < 1e-3:
                return                      # during a cruise: on_step is called only while walking
            state['done'] = True
            d = np.hypot(x - prev[0], y - prev[1])
            ux, uy = (x - prev[0]) / d, (y - prev[1]) / d
            cx, cy = x + (face + 0.2) * ux, y + (face + 0.2) * uy
            state['box'] = (cx - 0.2, cx + 0.2, cy - 0.2, cy + 0.2)
            e2e.WORLD.dynamic = state['box']
            e2e.WORLD._recompute()
        r = run(f"a box appears {face:.1f} m ahead during the cruise", [], (2.0, 2.0, 0.785), [(0, 1)], on_step=box)
        check(state['done'], "the box appeared")
        if realistic:
            check(r['clear'] >= geometry.ROBOT_HALF_WIDTH_M, f"never closer to the box than half the body "
                                                             f"({r['clear']:.2f} m, limit {geometry.ROBOT_HALF_WIDTH_M})")
        else:
            check('[CRUISE] Block:' in r['log'] or r['oks'] == [True],
                  f"past the box: a block at once and the next goal chosen, or the cell entered ({r['ends']})")
            check(r['clear'] > 0.0, f"the centre never inside the box ({r['clear']:.2f} m; the body is "
                                    f"Spot's obstacle avoidance)")
            # This fake robot has no body obstacle avoidance: it can stop with the body against
            # the box, unable to turn either way; then every goal of the cell is a block and the
            # cell is left for later, or after attempt_cruise.BLOCK_STILL_MAX blocks it is
            # blocked on the spot, a failure of the cell (on the robot Spot would have stopped
            # before).
            ends = ' '.join(r['ends'])
            check(r['oks'] == [True] or (('no goal with a path in cell' in ends or 'blocked on the spot' in ends)
                                         and 'Traceback' not in r['log']),
                  f"the attempt ends cleanly: cell entered, or left for later ({r['oks']}, {r['ends']})")
            continue
        check(r['oks'] == [True], "entered the cell anyway (around the box)")

    r = run("goal inside a box: new goal while walking", [(8.25, 8.75, 3.25, 3.75)], (2.0, 2.0, 0.785), [(0, 1)])
    check(r['oks'] == [True], "entered the cell")
    check(r['stats'].get('cruise_goal_repicks', 0) >= 1, f"new goal chosen while walking "
                                                         f"({r['stats'].get('cruise_goal_repicks', 0)})")

    r = run("obstacle right beyond the goal, then the next cell", [(8.7, 9.3, 3.0, 4.0)], (2.0, 2.0, 0.785),
            [(0, 1), (1, 1)])
    check(r['oks'] == [True, True], "both cells entered")
    check(r['stats'].get('cruise_handoffs', 0) >= 1, f"next cell while walking ({r['stats'].get('cruise_handoffs', 0)})")

    # The wall ends 0.5 m before the border between the cells. With its end ON the border
    # (x = 6.0, the first version of this scenario) going around it with 0.30 m of clearance
    # enters cell (1,1), forbidden by mission_prm.planning_cells: whether a way inside the allowed
    # cells exists depended on a few cm of the robot pose (1 run out of 6 failed; from a
    # standing robot under the wall end 0/6 with the restriction, 6/6 without, stop-and-go only).
    r = run("three cells in a row: hand-offs and big turns", [(1.0, 5.5, 5.9, 6.1)], (2.0, 2.0, 0.785),
            [(0, 1), (1, 0), (1, 1)])
    check(r['oks'] == [True, True, True], "three cells entered")
    check(r['stats'].get('cruise_handoffs', 0) >= 2, f"hand-offs while walking ({r['stats'].get('cruise_handoffs', 0)})")
    big = [l for l in r['log'].splitlines() if 'inside the new command, without stopping first' in l]
    check(len(big) >= 1, f"big turn towards the next cell without stopping first ({len(big)})")
    check(r['clear'] >= clear_min, f"centre at least {clear_min:.2f} m from the wall ({r['clear']:.2f})")

    # The cell counts as explored as soon as the global map has seen 90% of it, wherever the
    # robot is (06-11): open field, two cells in a row. The cells
    # here are 5 m and the scan window 3.84 m: before arriving the robot sees ~55% of the cell,
    # so the threshold is 50% here (90% in the mission, cells of 2 m).
    fake_seen, frac0 = cells.cell_seen_fraction, attempt_common.CELL_SEEN_FRACTION
    cells.cell_seen_fraction, attempt_common.CELL_SEEN_FRACTION = e2e.REAL_CELL_SEEN_FRACTION, 0.50
    try:
        r = run("open field, two cells: the cell is done when seen", [], (2.0, 2.0, 0.785), [(0, 1), (1, 1)])
    finally:
        cells.cell_seen_fraction, attempt_common.CELL_SEEN_FRACTION = fake_seen, frac0
    seen_ends = [e for e in r['ends'] if e.startswith('cell seen')]
    check(r['oks'] == [True, True], f"two cells done ({r['oks']})")
    check(len(seen_ends) >= 1, f"hand-off because the cell is seen ({r['ends']})")
    check("cell seen" in r['log'] and "[CRUISE] Plan" in r['log'], "the [CRUISE] Plan lines give the share seen")

    # Step C (09-11): the next cell prepared in the planning thread, the cruise goes on through
    # the cells. A real cell grid and the cell rules, as the mission sets them; ONE attempt on
    # (0,1). Cells of 5 m here: "near" up to 8 m (4 m in the mission, cells of 2 m).
    from spot_explorer.mission import cell_rules, cells as cells_mod
    real_env = cells_mod.EnvironmentMap(rows=2, cols=2, cell_size=5.0)
    with contextlib.redirect_stdout(io.StringIO()):
        real_env.set_origin(3.5, 3.5, 0.0)
    rules = cell_rules.CellRules(real_env, None, real_env.generate_visit_path('auto', start_cell=(0, 0)))
    done_cb = []
    real_env._cell_rules = rules
    real_env._on_chained_visit = lambda r, c, x, y: done_cb.append((r, c))
    near0, cell_rules.NEAR_PATH_M = cell_rules.NEAR_PATH_M, 8.0
    try:
        r = run("step C: one cruise through three cells", [], (2.0, 2.0, 0.785), [(0, 1)], env=real_env)
    finally:
        cell_rules.NEAR_PATH_M = near0
    switches = [l for l in r['log'].splitlines() if 'done while walking' in l]
    check(r['oks'] == [True], f"the attempt ends well ({r['oks']})")
    check(len(switches) >= 2, f"two cells done while walking, the next prepared in the planning thread "
                              f"({len(switches)}: {[l.split('[CRUISE] ')[1][:40] for l in switches]})")
    check(done_cb[:2] == [(0, 1), (1, 1)], f"waypoint of the cells done, in order ({done_cb})")
    check(r['stats'].get('cruises') == 1, f"one cruise for the three cells ({r['stats'].get('cruises')})")
    # Still (no linear speed) only for the turns on the spot towards the next cell: 90 degrees
    # at every switch here, ~2 s each (2.6 with the computer busy); no wait for the next cell.
    tr = np.array(TRACK)
    v = np.hypot(np.diff(tr[:, 0]), np.diff(tr[:, 1])) / np.maximum(np.diff(tr[:, 2]), 1e-3)
    pauses, k = [], 0
    while k < len(v):
        if v[k] < 0.05:
            j = k
            while j < len(v) and v[j] < 0.05:
                j += 1
            pauses.append(float(tr[j, 2] - tr[k, 2]))
            k = j
        else:
            k += 1
    mid = [p_ for p_ in pauses[1:-1] if p_ >= 0.3] if len(pauses) > 2 else [p_ for p_ in pauses if p_ >= 0.3]
    check(all(p_ < 3.0 for p_ in mid), f"between the cells only the turns on the spot "
                                        f"({[round(p_, 1) for p_ in mid]} s)")

    # A cell with no goal reachable (a big box fills (0,1) but for 0.3 m strips): the planning
    # thread excludes the goals, finds none with a path, leaves the cell -- not done, not failed
    # -- and goes on to the next one, in the same cruise (09-11: before, the robot stopped and
    # the cell counted as failed).
    env2 = cells_mod.EnvironmentMap(rows=2, cols=2, cell_size=5.0)
    with contextlib.redirect_stdout(io.StringIO()):
        env2.set_origin(3.5, 3.5, 0.0)
    env2._cell_rules = cell_rules.CellRules(env2, None, env2.generate_visit_path('auto', start_cell=(0, 0)))
    env2._on_chained_visit = lambda r, c, x, y: None
    # The way to the next cell goes around the box (~10 m): "near" up to 15 m here.
    near0, cell_rules.NEAR_PATH_M = cell_rules.NEAR_PATH_M, 15.0
    try:
        r = run("cell without a reachable goal: left while walking", [(6.3, 10.7, 1.3, 5.7)],
                (2.0, 2.0, 0.785), [(0, 1)], env=env2)
    finally:
        cell_rules.NEAR_PATH_M = near0
    left = [l for l in r['log'].splitlines() if 'left while walking' in l]
    check(len(left) == 1 and 'Cell (0,1) left' in left[0], f"(0,1) left while walking ({left[:1]})")
    lines = r['log'].splitlines()
    k_left = next((k for k, l in enumerate(lines) if 'left while walking' in l), None)
    check(k_left is not None and not any('[CRUISE] Start' in l for l in lines[k_left:])
          or any('Cell (1,1) done while walking' in l for l in lines[k_left or 0:]),
          "after leaving (0,1) the same cruise goes on into (1,1)")
    check(r['clear'] >= clear_min, f"centre at least {clear_min:.2f} m from the box ({r['clear']:.2f})")

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
    check("[FALL]" in r['log'] or "fall" in r['log'].lower(), "fall handled by the attempt (log)")

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)} ({time.time() - t_all:.0f} s)")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
