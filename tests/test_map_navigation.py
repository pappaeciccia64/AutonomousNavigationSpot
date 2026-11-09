"""
Test WITHOUT THE ROBOT of map navigation (spot_explorer/navigation, run_navigation.py): the real
loop (MapNavigator.go) against the fake robot and local grid of test_e2e_loop.py.

The fake world is in TODAY's frame. The mission map (occupancy, terrain, PRM, GraphNav
waypoints) is built in the OLD frame, moved and rotated with respect to today by T_TRUE: the
situation after a reboot of the robot. A fake GraphNav localizes the robot on the nearest
waypoint, like the real one; everything else (frame estimate, PRM on the map, safe frontier,
decisions, dropped edges and new plans) is the real code.

Scenarios:
  1  wall with a gap: the path goes around it and the robot arrives;
  2  an obstacle that was not there at the end of the mission: it enters the map, the path
     changes along the way and the robot arrives anyway (2b: with fixed maps, dropping edges);
  3  goal inside an obstacle: the robot stops on the nearest reachable node;
  4  goal chosen with a click (ClickGoalPicker), with fake events instead of the mouse: right
     point, path at once, Enter confirms, clicks outside the map ignored, Esc gives no goal, h
     goes home; "home" and "c R,C" from the terminal;
  5  an obstacle of the map that is gone today: with the updated maps the cell turns free and
     the robot goes straight, without (--no-map-update) it goes around;
  6  the updated maps: today's height brought to the mission's, the new obstacle of scenario
     2 occupied in the map, save and reload from --maps;
  7  battery low half-way: no more goals, back home (fake GraphNav that does not arrive, then
     the PRM);
  8  GraphNav lost: relocalization succeeds and the robot resumes; relocalization fails and
     the robot stays where it is;
  9  logs: navigation.csv, segments.csv, state and map image for the viewer, the viewer's
     figure; "home" goal from run_goals;
 10  reading the mission log: Italian and English logs give the same configuration, a log
     without the grid line is an error (not silent defaults).

    python3 tests/test_map_navigation.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # the spot_explorer package
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))

import test_e2e_loop as T          # fake world, robot and grid; applies its patches  # noqa: E402
from spot_explorer import records                                                 # noqa: E402
from spot_explorer.navigation import mission_map as mm                            # noqa: E402
from spot_explorer.navigation import navigator as nv                              # noqa: E402
from spot_explorer.navigation import run as nrun                                  # noqa: E402
from spot_explorer.navigation.frames import MissionFrame, Transform2D             # noqa: E402
from spot_explorer.navigation.goal_picker import ClickGoalPicker                  # noqa: E402
from spot_explorer.perception import geometry, global_maps                        # noqa: E402
from spot_explorer.planning import safe_frontier as sf                            # noqa: E402
import navigation_viewer                                                          # noqa: E402

FAILS = []
T_TRUE = Transform2D(2.0, -1.0, 0.5)     # mission frame -> today's frame
RES = 0.03


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def old_from_new(x, y):
    c, s = np.cos(T_TRUE.th), np.sin(T_TRUE.th)
    dx, dy = x - T_TRUE.tx, y - T_TRUE.ty
    return c * dx + s * dy, -s * dx + c * dy


def new_from_old(x, y):
    c, s = np.cos(T_TRUE.th), np.sin(T_TRUE.th)
    return T_TRUE.tx + c * x - s * y, T_TRUE.ty + s * x + c * y


def map_from_world(world, start_xy, start_yaw):
    """The end-of-mission map, in the old frame, as if the exploration had seen the whole
    world `world`. Cells of the old map -> today's point -> state in the world."""
    ext = T.W * T.CS
    corners = np.array([old_from_new(x, y) for x in (0, ext) for y in (0, ext)])
    gx = np.arange(int(np.floor(corners[:, 0].min() / RES)), int(np.ceil(corners[:, 0].max() / RES)))
    gy = np.arange(int(np.floor(corners[:, 1].min() / RES)), int(np.ceil(corners[:, 1].max() / RES)))
    GX, GY = np.meshgrid(gx, gy, indexing='ij')
    nx, ny = new_from_old((GX + 0.5) * RES, (GY + 0.5) * RES)
    c, r = np.floor(nx / T.CS).astype(int), np.floor(ny / T.CS).astype(int)
    inside = (c >= 0) & (c < T.W) & (r >= 0) & (r < T.W)
    occ = np.zeros(GX.shape, bool)
    # Like the mission: a cell is occupied when obstacle_distance is under the threshold
    # (global_maps.mask_for_global_map), not only the physical obstacle.
    occ[inside] = world.od[r[inside], c[inside]] <= geometry.OBSTACLE_THRESHOLD
    gm = global_maps.GlobalGrid(resolution=RES)
    tm = global_maps.GlobalTerrainGrid(resolution=RES)
    for k1, k2, o in zip(GX[inside].tolist(), GY[inside].tolist(), occ[inside].tolist()):
        gm._set_state((k1, k2), -1.0 if o else 1.0)
        tm.height_sum[(k1, k2)] = 0.0
        tm.height_count[(k1, k2)] = 1
    tm.update_rects = [(gx[0] * RES, (gx[-1] + 1) * RES, gy[0] * RES, (gy[-1] + 1) * RES)]
    ox, oy = old_from_new(*start_xy)
    cfg = dict(name='fake', origin=(ox, oy), yaw=start_yaw - T_TRUE.th, rows=5, cols=5, cell=2.0,
               spacing=0.5, sampling='lattice', min_edge=0.5, max_edge=1.25,
               distribution=None, density=None, seed=None)
    # wp_0 on the start pose, as in the mission; then GraphNav waypoints every 1.5 m on free
    # ground, randomly oriented (in the old frame).
    rng = np.random.default_rng(3)
    wps = {'id0': dict(name='wp_0', x=ox, y=oy, yaw=start_yaw - T_TRUE.th)}
    for x in np.arange(1.0, ext - 0.5, 1.5):
        for y in np.arange(1.0, ext - 0.5, 1.5):
            if world.od_at(x, y) > 0.5:
                wx, wy = old_from_new(x, y)
                wid = f"id{len(wps)}"
                wps[wid] = dict(name=f"wp_{len(wps)}", x=wx, y=wy, yaw=float(rng.uniform(-np.pi, np.pi)))
    ng = dict(dir=None, graph=None, wp=wps, by_name={v['name']: k for k, v in wps.items()})
    with contextlib.redirect_stdout(io.StringIO()):
        return mm.MissionMap('fake', cfg, gm, tm, ng)


class FakeGraphNav:
    """Localization on the nearest waypoint, with the body pose relative to it."""

    def __init__(self, wps):
        self.wps = wps
        self.n = 0                 # readings done
        self.lost_from = None      # from reading n on: robot lost
        self.lost_for = None       # for how many readings (None = for good)

    def get_localization_state(self):
        self.n += 1
        if (self.lost_from is not None and self.n >= self.lost_from
                and (self.lost_for is None or self.n < self.lost_from + self.lost_for)):
            return NS(localization=NS(waypoint_id='', waypoint_tform_body=None),
                      lost_detector_state=NS(is_lost=True))
        ox, oy = old_from_new(T.ROBOT.x, T.ROBOT.y)
        oyaw = T.ROBOT.yaw - T_TRUE.th
        wid = min(self.wps, key=lambda k: np.hypot(self.wps[k]['x'] - ox, self.wps[k]['y'] - oy))
        w = self.wps[wid]
        c, s = np.cos(w['yaw']), np.sin(w['yaw'])
        dx, dy = ox - w['x'], oy - w['y']
        pos = NS(x=c * dx + s * dy, y=-s * dx + c * dy, z=0.0)
        return NS(localization=NS(waypoint_id=wid, waypoint_tform_body=NS(position=pos,
                                                                         rotation=T.quat(oyaw - w['yaw']))))


def navigator(mission_map, update=True):
    nav = object.__new__(nv.MapNavigator)     # without the robot: no RecordingInterface
    nav.map, nav.robot, nav.rsc, nav.cc = mission_map, None, None, None
    nav.ri = NS(waypoint_poses={})
    nav.gn = FakeGraphNav(mission_map.ng['wp'])
    nav.T = None
    nav.tracker = T.FakeTracker()
    nav.local_grid = T.FakeLocalGrid()
    nav.init_maps(update=update)
    nav.init_session(None)
    return nav


def run(name, rects_map, rects_today, start, goal_new, update=True):
    print(f"\n=== {name} ===")
    sf.OBSTACLE_HISTORY.reset()
    mission_map = map_from_world(T.World(rects_map, ghosts=False), start[:2], start[2])
    T.WORLD = T.World(rects_today, ghosts=False)
    T.ROBOT = T.Robot(*start)
    nav = navigator(mission_map, update)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ok_loc = nav.update_transform("test") is not None
        ok = nav.go(old_from_new(*goal_new), f"({goal_new[0]}, {goal_new[1]}) of today",
                    graphnav_fallback=False)
    log = out.getvalue()
    if '--log' in sys.argv:
        print(log)
    dm, dg = nav.T.difference(T_TRUE)
    check(ok_loc and dm < 0.01 and dg < 0.1,
          f"estimated frame {nav.T} against the true {T_TRUE} (off by {dm:.3f} m, {dg:.2f} degrees)")
    d_goal = float(np.hypot(T.ROBOT.x - goal_new[0], T.ROBOT.y - goal_new[1]))
    return dict(ok=ok, d_goal=d_goal, min_clear=T.ROBOT.min_clear, walked=T.ROBOT.walked, nav=nav,
                log=log, plans=log.count('] Plan '))


def test_mission_config():
    print("\n=== 10 reading the mission log ===")
    it = mm.mission_config('26-10-2026_15-11-21')
    check(it['rows'] == 8 and it['cols'] == 7 and it['sampling'] == 'random' and it['seed'] == 381472552
          and it['distribution'] == 'poisson', f"Italian log (26-10): {it['rows']} x {it['cols']}, "
                                               f"{it['sampling']} {it['distribution']}, seed {it['seed']}")
    en = mm.mission_config('29-10-2026_10-57-47')
    check(en['rows'] == 8 and en['cols'] == 5 and en['sampling'] == 'random' and en['seed'] == 1673619233,
          f"log of 29-10: {en['rows']} x {en['cols']}, {en['sampling']}, seed {en['seed']}")
    # The same lines as the main mission code writes them today (English).
    english = ("[OK] Origin set at world coordinates (1.50, -2.00)\n[OK] Initial orientation: 30.0°\n"
               "[OK] Starting cell (0, 1) marked as visited\n"
               "[CONFIG] Mission grid: 4 x 6 cells of 2.0 m (8 m to the left x 12 m ahead; source: default)\n"
               "[CONFIG] Lattice: requested spacing 0.500 m, rings all\n"
               "[CONFIG] Graph: nodes random (halton, 5 nodes/m2, seed random); edges from 0.5 to 1.25 m\n"
               "[GRAPH] Random nodes: distribution halton, density 5.00 nodes/m2, seed 42 (to repeat: ...)\n")
    root = tempfile.mkdtemp(prefix="mapnav_logs_")
    old_root, mm.REPO_ROOT = mm.REPO_ROOT, root
    try:
        for name, text in (('en', english), ('nogrid', english.replace('[CONFIG] Mission grid', '[CONFIG] Grid'))):
            os.makedirs(os.path.join(root, 'MissionLogs', f'Mission_{name}'))
            open(os.path.join(root, 'MissionLogs', f'Mission_{name}', 'mission_log.txt'), 'w').write(text)
        c = mm.mission_config('en')
        check((c['rows'], c['cols'], c['cell'], c['start_cell'], c['sampling'], c['distribution'], c['seed'])
              == (4, 6, 2.0, (0, 1), 'random', 'halton', 42) and abs(np.degrees(c['yaw']) - 30) < 1e-9,
              "English log of today's mission code: grid, starting cell, random nodes and seed")
        try:
            mm.mission_config('nogrid')
            check(False, "log without the grid line: no error")
        except ValueError as e:
            check('Mission grid' in str(e), f"log without the grid line: error ({e})")
    finally:
        mm.REPO_ROOT = old_root


def main():
    np.seterr(all='ignore')
    # 1. Wall from x = 5 to 5.4 with a gap at the top (y > 8): from (1.5, 1.5) to (9, 2).
    wall = [(5.0, 5.4, 0.0, 8.0)]
    r = run("1 wall with a gap", wall, wall, (1.5, 1.5, 0.0), (9.0, 2.0))
    check(r['ok'], f"arrived ({r['plans']} plans, {r['walked']:.1f} m walked)")
    check(r['d_goal'] < 0.8, f"{r['d_goal']:.2f} m from the goal")
    check(r['min_clear'] > 0.2, f"minimum distance from obstacles {r['min_clear']:.2f} m")

    # 2. A new block on the corridor below the gap, which the map does not have.
    new = wall + [(5.4, 7.0, 8.3, 8.7)]
    r = run("2 new obstacle", wall, new, (1.5, 1.5, 0.0), (9.0, 2.0))
    check(r['ok'], f"arrived anyway ({r['plans']} plans)")
    check(r['plans'] >= 2 or 'new path' in r['log'],
          f"path changed after the new obstacle ({r['plans']} plans, "
          f"{r['log'].count('new path')} path changes along the way)")
    check(r['min_clear'] > 0.2, f"minimum distance from obstacles {r['min_clear']:.2f} m")

    r0 = run("2b new obstacle, fixed maps", wall, new, (1.5, 1.5, 0.0), (9.0, 2.0), update=False)
    check(r0['ok'], f"arrived with fixed maps too ({r0['plans']} plans)")
    gm = r['nav'].map.gm
    ox, oy = old_from_new(6.2, 8.5)                       # inside the new obstacle
    check(gm.is_occupied(ox, oy), "6: the new obstacle is occupied in the updated map")
    check(r['nav'].dz is not None and abs(r['nav'].dz - 3.23) < 0.02,
          f"6: estimated height offset {r['nav'].dz:+.3f} m (true +3.23: today's fake heights at -3.23)")
    check(r['nav'].n_scans_in_map > 10,
          f"6: {r['nav'].n_scans_in_map} scans in the maps, {r['nav'].t_maps_s:.1f} s in all")
    tmp = tempfile.mkdtemp(prefix="mapnav_maps_")
    with contextlib.redirect_stdout(io.StringIO()):
        r['nav'].save_maps(tmp)
    gm2 = mm.load_occupancy_map(os.path.join(tmp, 'global_occupancy_map.pkl'))
    tm2 = mm.load_terrain_map(os.path.join(tmp, 'global_terrain_map.pkl'))
    check(gm2.is_occupied(ox, oy) and len(gm2.grid) == len(gm.grid)
          and len(tm2.height_count) == len(r['nav'].map.tm.height_count),
          "6: maps saved and reloaded identical")
    h = tm2.get_height(*old_from_new(8.0, 1.0))
    check(h is not None and abs(h) < 0.02, f"6: map height unchanged where the robot walked ({h:+.3f} m)")

    # 5. Obstacle in the map, gone today: in front of the start, on the line towards (9, 1.5).
    # All inside the scan window from the start (3.8 m around the robot): a far part would
    # stay occupied until the robot gets close.
    gone = [(2.6, 3.2, 0.5, 2.5)]
    r = run("5 obstacle gone, updated maps", gone, [], (1.5, 1.5, 0.0), (9.0, 1.5))
    r0 = run("5b obstacle gone, fixed maps", gone, [], (1.5, 1.5, 0.0), (9.0, 1.5), update=False)
    check(r['ok'] and r0['ok'], "arrived in both cases")
    check(r['walked'] < 8.0, f"updated maps: {r['walked']:.1f} m walked (straight line 7.5 m)")
    check(r0['walked'] > r['walked'] + 0.3, f"fixed maps: {r0['walked']:.1f} m, around the obstacle "
                                            f"that is no longer there")
    check('new path' in r['log'], "with the updated maps the path changes along the way")

    # 3. Goal inside a solid obstacle.
    block = [(7.0, 9.0, 3.0, 5.0)]
    r = run("3 goal inside an obstacle", block, block, (1.5, 1.5, 0.0), (8.0, 4.0))
    check(r['ok'], "arrived at the nearest reachable node")
    check(1.0 < r['d_goal'] < 2.0, f"stopped outside the obstacle, {r['d_goal']:.2f} m from its centre")

    # 4. Choice with a click: fake events, figure without a window (Agg backend).
    print("\n=== 4 goal with a click ===")
    mission_map = map_from_world(T.World(wall, ghosts=False), (1.5, 1.5), 0.0)
    start = old_from_new(1.5, 1.5)
    with contextlib.redirect_stdout(io.StringIO()):
        pk = ClickGoalPicker(mission_map, start)
        # Goal (9, 2) of today = (7.5, 0.5) in the mission frame (start at (1.5, 1.5), nose
        # along +x); in the figure the point (xm, ym) is at (ym, xm).
        pk._click(NS(inaxes=pk.ax, button=1, xdata=0.5, ydata=7.5))
    gx, gy = new_from_old(*pk.goal)
    check(np.hypot(gx - 9.0, gy - 2.0) < 1e-6, f"click at (7.5, 0.5) of the mission frame -> "
                                               f"({gx:.2f}, {gy:.2f}) of today")
    check(pk.plan is not None and pk.plan['length'] > 9.0,
          f"path planned at once, around the wall ({pk.plan['length']:.1f} m)")
    with contextlib.redirect_stdout(io.StringIO()):
        pk._click(NS(inaxes=None, button=1, xdata=None, ydata=None))           # outside the axes
        pk._click(NS(inaxes=pk.ax, button=3, xdata=3.0, ydata=3.0))            # right button
    check(np.hypot(*np.subtract(new_from_old(*pk.goal), (9.0, 2.0))) < 1e-6,
          "clicks outside the map and with the right button ignored")
    check(len(pk._artists) == 3, "a single path drawn (the old one removed)")
    pk._key(NS(key='enter'))
    r = pk.choose()
    check(r is not None and r[0] == pk.goal, "Enter: goal confirmed")
    with contextlib.redirect_stdout(io.StringIO()):
        pk2 = ClickGoalPicker(mission_map, start)
        pk2._click(NS(inaxes=pk2.ax, button=1, xdata=2.0, ydata=2.0))
    pk2._key(NS(key='escape'))
    check(pk2.choose() is None, "Esc: no goal")
    with contextlib.redirect_stdout(io.StringIO()):
        pk3 = ClickGoalPicker(mission_map, start)
    pk3._key(NS(key='h'))
    check(pk3.choose() == nv.HOME, "h: go home")
    check(nrun.goal_from_text(mission_map, 'home') == nv.HOME
          and nrun.goal_from_text(mission_map, 'c 1,2')[1] == "centre of cell (1,2)",
          "from the terminal: 'home' and 'c R,C'")

    # 7. Battery low half-way.
    print("\n=== 7 battery low ===")
    sf.OBSTACLE_HISTORY.reset()
    mission_map = map_from_world(T.World(wall, ghosts=False), (1.5, 1.5), 0.0)
    T.WORLD, T.ROBOT = T.World(wall, ghosts=False), T.Robot(1.5, 1.5, 0.0)
    nav = navigator(mission_map)

    class FakeBattery:
        threshold_pct = 30.0
        n = 0

        def check(self, where=None):
            self.n += 1
            return self.n > 12
    nav.init_session(None, battery=FakeBattery())
    graphnav_called = []
    nav._graphnav_to_wp0 = lambda: graphnav_called.append(1) and False
    args = NS(click=False, interactive=False, no_fallback=True, return_at_end=False)
    with contextlib.redirect_stdout(io.StringIO()) as out:
        nav.update_transform("test")
        ok = nrun.run_goals(nav, mission_map, args, [(old_from_new(9.0, 2.0), "far")])
    log = out.getvalue()
    d_home = float(np.hypot(T.ROBOT.x - 1.5, T.ROBOT.y - 1.5))
    check(not ok and 'Battery below the return threshold' in log, "goal interrupted by the battery")
    check(graphnav_called == [1], "return tried first with GraphNav")
    check(d_home < 0.8 and nav.phase == 'arrived', f"then with the PRM: home, {d_home:.2f} m from wp_0")
    check(not nav.returning and nav.battery.n <= 14,
          f"during the return the battery does not interrupt it ({nav.battery.n} readings)")

    # 8. GraphNav lost.
    print("\n=== 8 GraphNav lost ===")
    for fix, name in ((True, "relocalization succeeds"), (False, "relocalization fails")):
        sf.OBSTACLE_HISTORY.reset()
        mission_map = map_from_world(T.World([], ghosts=False), (1.5, 1.5), 0.0)
        T.WORLD, T.ROBOT = T.World([], ghosts=False), T.Robot(1.5, 1.5, 0.0)
        nav = navigator(mission_map)
        with contextlib.redirect_stdout(io.StringIO()):
            nav.update_transform("test")
        nav.gn.lost_from = nav.gn.n + 6
        attempts, commands_at_reloc = [], []

        def relocalize(nav=nav, fix=fix):
            attempts.append(1)
            commands_at_reloc.append(T.ROBOT.commands)
            if fix:
                nav.gn.lost_from = None
            return fix
        nav._relocalize = relocalize
        nav._relocalize_fiducial = lambda: False
        with contextlib.redirect_stdout(io.StringIO()) as out:
            ok = nav.go(old_from_new(9.0, 1.5), "straight", graphnav_fallback=False)
        if fix:
            check(ok and attempts, f"{name}: arrived ({len(attempts)} relocalizations)")
        else:
            check(not ok and nav.end_reason == 'lost' and nav.phase == 'lost',
                  f"{name}: goal abandoned, reason 'lost'")
            check(T.ROBOT.commands == commands_at_reloc[0],
                  f"{name}: no move after the loss ({T.ROBOT.commands} commands in all)")
            check('LOST' in out.getvalue() and 'stays still' in out.getvalue(), f"{name}: said in the log")

    # 9. Logs, viewer, "home" goal.
    print("\n=== 9 logs and home ===")
    tmp = tempfile.mkdtemp(prefix="mapnav_log_")
    records.MISSION_CTX['log_folder'] = tmp
    records.MISSION_CTX['segment_log'] = records.SegmentLog(os.path.join(tmp, 'segments.csv'))
    try:
        sf.OBSTACLE_HISTORY.reset()
        mission_map = map_from_world(T.World(wall, ghosts=False), (1.5, 1.5), 0.0)
        T.WORLD, T.ROBOT = T.World(wall, ghosts=False), T.Robot(9.0, 2.0, np.pi)
        nav = navigator(mission_map)
        nav.init_session(tmp)
        args = NS(click=False, interactive=False, no_fallback=True, return_at_end=False)
        with contextlib.redirect_stdout(io.StringIO()):
            nav.update_transform("test")
            ok = nrun.run_goals(nav, mission_map, args, [nv.HOME])
        d_home = float(np.hypot(T.ROBOT.x - 1.5, T.ROBOT.y - 1.5))
        check(ok and d_home < 0.8, f"'home': from (9, 2) to wp_0, arrived {d_home:.2f} m away")
        rows = open(os.path.join(tmp, 'navigation.csv')).read().splitlines()
        segs = open(os.path.join(tmp, 'segments.csv')).read().splitlines()
        check(len(rows) > 10 and len(segs) > 5, f"navigation.csv {len(rows) - 1} rows, segments.csv {len(segs) - 1} legs")
        st = json.load(open(os.path.join(tmp, 'navigation_state.json')))
        check(st['phase'] == 'arrived' and np.hypot(*st['robot'][:2]) < 0.8,
              f"state: phase {st['phase']}, robot at ({st['robot'][0]:.2f}, {st['robot'][1]:.2f}) of the "
              f"mission frame (home = 0, 0)")
        info = json.load(open(os.path.join(tmp, 'navigation_info.json')))
        check(np.hypot(*info['wp0']) < 1e-6 and len(info['waypoints']) == len(mission_map.ng['wp']),
              "info: wp_0 at the origin of the mission frame, all the waypoints")
        png = os.path.join(tmp, 'viewer.png')
        with contextlib.redirect_stdout(io.StringIO()):
            navigation_viewer.NavigationViewer(tmp).png(png)
        check(os.path.exists(os.path.join(tmp, 'navigation_map.npz')) and os.path.getsize(png) > 10000,
              "viewer: map image written and figure drawn")
    finally:
        records.MISSION_CTX['log_folder'] = None
        records.MISSION_CTX['segment_log'] = None
    print(f"  (viewer figure: {png})")

    test_mission_config()

    # Transforms: there and back.
    t = Transform2D.from_poses((3.0, 4.0, 1.0), (1.0, -2.0, 0.3))
    x, y = t.inverse(*t.apply(5.0, 6.0))
    check(np.hypot(x - 5.0, y - 6.0) < 1e-9 and np.hypot(*np.subtract(t.apply(1.0, -2.0), (3.0, 4.0))) < 1e-9,
          "Transform2D: there and back")
    fr = MissionFrame(10.0, 20.0, np.radians(90))
    check(np.allclose(fr.to_vision(1.0, 0.0), (10.0, 21.0)) and np.allclose(fr.from_vision(*fr.to_vision(2, 3)), (2, 3)),
          "MissionFrame: x ahead, y to the left")

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} CHECKS FAILED'}")
    return not FAILS


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
