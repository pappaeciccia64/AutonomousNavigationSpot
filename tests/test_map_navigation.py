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
     without the grid line is an error (not silent defaults);
 11  frame jump: a single wrong GraphNav reading 1 m away is not trusted, a lasting correction
     is accepted after the confirmation readings;
 12  check before starting: where the robot is, and no move without a yes;
 13  whole session (run_navigation.main) against a fake robot, lease, E-Stop and GraphNav:
     graph uploaded with the motors off, then standing, localization, goal, sitting down,
     motors off, maps saved after; a second session with the graph already on Spot sends no
     snapshot; no terminal and no --yes: no move; exception and Ctrl+C half-way: stop, sit,
     motors off, then the maps, and the error goes up; fall that cannot be recovered: no
     sitting down.

    python3 tests/test_map_navigation.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile
import time
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
        self.jump_from = None      # from reading n on: position off by jump_xy (old frame)
        self.jump_for = None       # for how many readings (None = for good)
        self.jump_xy = (0.0, 0.0)

    def get_localization_state(self):
        self.n += 1
        if (self.lost_from is not None and self.n >= self.lost_from
                and (self.lost_for is None or self.n < self.lost_from + self.lost_for)):
            return NS(localization=NS(waypoint_id='', waypoint_tform_body=None),
                      lost_detector_state=NS(is_lost=True))
        ox, oy = old_from_new(T.ROBOT.x, T.ROBOT.y)
        if (self.jump_from is not None and self.n >= self.jump_from
                and (self.jump_for is None or self.n < self.jump_from + self.jump_for)):
            ox, oy = ox + self.jump_xy[0], oy + self.jump_xy[1]
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


def test_frame_jump(wall):
    print("\n=== 11 frame jump ===")
    nv.FRAME_JUMP_CONFIRM_WAIT_S = 0.0
    for lasting, name in ((False, "single wrong reading 1 m away"), (True, "lasting correction of 0.6 m")):
        sf.OBSTACLE_HISTORY.reset()
        mission_map = map_from_world(T.World([], ghosts=False), (1.5, 1.5), 0.0)
        T.WORLD, T.ROBOT = T.World([], ghosts=False), T.Robot(1.5, 1.5, 0.0)
        nav = navigator(mission_map)
        with contextlib.redirect_stdout(io.StringIO()):
            nav.update_transform("test")
        nav.gn.jump_from = nav.gn.n + 8
        nav.gn.jump_for = None if lasting else 1
        nav.gn.jump_xy = (0.6, 0.0) if lasting else (1.0, 0.0)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            ok = nav.go(old_from_new(9.0, 1.5), "straight", graphnav_fallback=False)
        log = out.getvalue()
        d = float(np.hypot(T.ROBOT.x - 9.0, T.ROBOT.y - 1.5))
        if lasting:
            check(ok and 'Jump confirmed' in log and 'NOT confirmed' not in log,
                  f"{name}: accepted after the confirmation readings, arrived where GraphNav says "
                  f"({d:.2f} m from the true goal: the fake correction moves the map)")
        else:
            check(ok and 'Jump NOT confirmed' in log and d < 0.8,
                  f"{name}: not trusted, arrived {d:.2f} m from the goal")


def test_confirm_start(wall):
    print("\n=== 12 check before starting ===")
    mission_map = map_from_world(T.World(wall, ghosts=False), (1.5, 1.5), 0.0)
    T.WORLD, T.ROBOT = T.World(wall, ghosts=False), T.Robot(3.5, 2.5, 0.3)
    nav = navigator(mission_map)
    with contextlib.redirect_stdout(io.StringIO()):
        nav.update_transform("test")
    w = nav.where_am_i()
    check(abs(w['x'] - 2.0) < 1e-6 and abs(w['y'] - 1.0) < 1e-6 and abs(w['heading_deg'] - np.degrees(0.3)) < 1e-6
          and w['seen'] and not w['occupied'],
          f"where am I: ({w['x']:.2f}, {w['y']:.2f}) heading {w['heading_deg']:.1f} degrees in the mission "
          f"frame, {w['waypoint_m']:.2f} m from {w['waypoint']}")
    asked = []
    with contextlib.redirect_stdout(io.StringIO()) as out:
        yes = nrun.confirm_start(nav, NS(yes=False), ask=lambda q: asked.append(q) or 'y')
        no = nrun.confirm_start(nav, NS(yes=False), ask=lambda q: 'n')
        empty = nrun.confirm_start(nav, NS(yes=False), ask=lambda q: '')
        skip = nrun.confirm_start(nav, NS(yes=True), ask=lambda q: asked.append(q) or 'n')
    check(yes and not no and not empty and skip and len(asked) == 1 and '(2.00, 1.00)' in out.getvalue(),
          "position said, start only with a yes (no and empty: no move), --yes does not ask")


# --- 13: whole session against a fake robot -----------------------------------------------
class FakeSpot:
    """GraphNav + recording + map processing of the session: a graph uploaded to Spot,
    snapshots, localization on the nearest waypoint, GraphNav moves."""

    def __init__(self):
        self.graph, self.snapshots = None, set()
        self.cleared = self.sent = 0
        self.localized = False

    def stop_recording(self, **kw):
        from bosdyn.api.graph_nav import recording_pb2
        return recording_pb2.StopRecordingResponse(status=recording_pb2.StopRecordingResponse.STATUS_OK)

    def get_record_status(self, **kw):
        return NS(is_recording=False)

    def download_graph(self, **kw):
        from bosdyn.api.graph_nav import map_pb2
        g = map_pb2.Graph()
        if self.graph is not None:
            g.CopyFrom(self.graph)
        return g

    def clear_graph(self, **kw):
        self.graph, self.snapshots, self.localized = None, set(), False
        self.cleared += 1

    def upload_graph(self, graph=None, **kw):
        from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2
        self.graph = map_pb2.Graph()
        self.graph.CopyFrom(graph)
        r = graph_nav_pb2.UploadGraphResponse()
        r.unknown_waypoint_snapshot_ids.extend(w.snapshot_id for w in graph.waypoints
                                               if w.snapshot_id not in self.snapshots)
        return r

    def upload_waypoint_snapshot(self, snap, **kw):
        self.snapshots.add(snap.id)
        self.sent += 1

    def upload_edge_snapshot(self, snap, **kw):
        self.sent += 1

    def set_localization(self, **kw):
        from bosdyn.api.graph_nav import graph_nav_pb2
        if self.graph is None:
            raise RuntimeError("no graph (fake)")
        self.localized = True
        return graph_nav_pb2.SetLocalizationResponse(status=graph_nav_pb2.SetLocalizationResponse.STATUS_OK)

    def _pose(self, wid):
        return SESSION['wps'][wid]

    def get_localization_state(self, **kw):
        from bosdyn.api.graph_nav import graph_nav_pb2
        from bosdyn.client.math_helpers import Quat, SE3Pose
        st = graph_nav_pb2.GetLocalizationStateResponse()
        if not self.localized:
            return st
        ox, oy = old_from_new(T.ROBOT.x, T.ROBOT.y)
        wps = SESSION['wps']
        wid = min(wps, key=lambda k: np.hypot(wps[k]['x'] - ox, wps[k]['y'] - oy))
        w = wps[wid]
        rel = SE3Pose(w['x'], w['y'], 0, Quat.from_yaw(w['yaw'])).inverse() * \
            SE3Pose(ox, oy, 0, Quat.from_yaw(T.ROBOT.yaw - T_TRUE.th))
        st.localization.waypoint_id = wid
        st.localization.waypoint_tform_body.CopyFrom(rel.to_proto())
        return st

    def navigate_to(self, waypoint_id, cmd_duration, **kw):
        w = SESSION['wps'][waypoint_id]
        T.ROBOT.x, T.ROBOT.y = new_from_old(w['x'], w['y'])
        return 1

    def navigation_feedback(self, command_id, **kw):
        from bosdyn.api.graph_nav import graph_nav_pb2
        return NS(status=graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL)


SESSION = {'wps': {}, 'events': []}


class FakeSessionRobot:
    def __init__(self, spot):
        self.spot = spot
        self.powered = False
        self.logger = NS(info=lambda *a, **k: None)

    def ensure_client(self, name):
        if name in ('recording-service', 'graph-nav-service', 'map-processing-service'):
            return self.spot
        if name == 'robot-command':
            return NS(robot_command=self._command)
        raise KeyError(name)

    def _command(self, cmd, **kw):
        text = str(cmd)
        SESSION['events'].append('sit' if 'sit_request' in text else 'stop' if 'stop_request' in text else 'cmd')
        return 1

    def power_on(self, **kw):
        SESSION['events'].append('power_on')
        self.powered = True

    def power_off(self, **kw):
        SESSION['events'].append('power_off')
        self.powered = False

    def is_powered_on(self):
        return self.powered


def _robot_state():
    """Robot state with odom -> body at the fake robot's pose (for the localization calls)."""
    from bosdyn.api import robot_state_pb2
    from bosdyn.client.math_helpers import Quat, SE3Pose
    st = robot_state_pb2.RobotState()
    edges = st.kinematic_state.transforms_snapshot.child_to_parent_edge_map
    edges['odom'].parent_frame_name = ''
    edges['odom'].parent_tform_child.rotation.w = 1.0
    edges['body'].parent_frame_name = 'odom'
    edges['body'].parent_tform_child.CopyFrom(SE3Pose(T.ROBOT.x, T.ROBOT.y, 0, Quat.from_yaw(T.ROBOT.yaw)).to_proto())
    return st


def session_map(wall, folder):
    """A mission map with a GraphNav graph proto and its snapshot files, as downloaded."""
    from bosdyn.api.graph_nav import map_pb2
    mission_map = map_from_world(T.World(wall, ghosts=False), (1.5, 1.5), 0.0)
    g = map_pb2.Graph()
    os.makedirs(os.path.join(folder, 'waypoint_snapshots'), exist_ok=True)
    for wid, w in mission_map.ng['wp'].items():
        wp = g.waypoints.add()
        wp.id, wp.annotations.name, wp.snapshot_id = wid, w['name'], f"snap-{wid}"
        with open(os.path.join(folder, 'waypoint_snapshots', wp.snapshot_id), 'wb') as f:
            f.write(map_pb2.WaypointSnapshot(id=wp.snapshot_id).SerializeToString())
    mission_map.ng['graph'], mission_map.ng['dir'] = g, folder
    SESSION['wps'] = mission_map.ng['wp']
    return mission_map


def test_session(wall):
    print("\n=== 13 whole session against a fake robot ===")
    import bosdyn.client.lease
    from bosdyn.client import robot_command
    from spot_explorer.graphnav import map_recording
    from spot_explorer.perception import spot_grid
    from spot_explorer.robot import battery, connection, motion

    class Tracker(T.FakeTracker):
        def start(self):
            pass

        def stop(self):
            pass

    class NoTilt(T._NoTilt):
        samples = []

    root = tempfile.mkdtemp(prefix="mapnav_session_")
    spot = FakeSpot()
    real = dict(setLogInfo=connection.setLogInfo, SimpleEstop=connection.SimpleEstop,
                lease=bosdyn.client.lease.LeaseKeepAlive, stand=robot_command.blocking_stand,
                LocalGrid=spot_grid.LocalGrid, tracker=sf.ArcVerificationTracker,
                read=battery.read_battery, tilt=records.BodyTiltRecorder, load=nrun.MissionMap.load,
                root=nrun.REPO_ROOT, nav_to=motion.navigate_to, fall=motion.check_and_recover_fall,
                save=global_maps.GlobalGrid.save_map, fid=map_recording.RecordingInterface.initialize_with_fiducial)
    robot = FakeSessionRobot(spot)
    connection.setLogInfo = lambda o: (robot, NS(), NS(get_robot_state=_robot_state), None)
    connection.SimpleEstop = lambda r, n: NS(stop=lambda: None, allow=lambda: None)
    bosdyn.client.lease.LeaseKeepAlive = lambda *a, **k: contextlib.nullcontext()
    robot_command.blocking_stand = lambda *a, **k: SESSION['events'].append('stand')
    spot_grid.LocalGrid = lambda r: T.FakeLocalGrid()
    sf.ArcVerificationTracker = lambda r: Tracker()
    battery.read_battery = lambda rsc: {'charge_pct': 80.0}
    records.BodyTiltRecorder = NoTilt
    nrun.REPO_ROOT = root
    map_recording.RecordingInterface.initialize_with_fiducial = (
        lambda self, rsc, fid=None: self._graph_nav_client.set_localization() is not None)

    def save(self, path, _real=real['save']):
        SESSION['events'].append('save_maps')
        return _real(self, path)
    global_maps.GlobalGrid.save_map = save

    def session(argv, wall_today=wall, fresh_spot=False):
        nonlocal spot
        if fresh_spot:
            spot = FakeSpot()
            robot.spot = spot
        sf.OBSTACLE_HISTORY.reset()
        snaps = tempfile.mkdtemp(prefix="graph_", dir=root)
        mission_map = session_map(wall, snaps)
        nrun.MissionMap.load = lambda *a, **k: mission_map
        T.WORLD, T.ROBOT = T.World(wall_today, ghosts=False), T.Robot(1.5, 1.5, 0.0)
        SESSION['events'] = []
        sent0, cleared0 = spot.sent, spot.cleared
        out = io.StringIO()
        exc = None
        try:
            with contextlib.redirect_stdout(out):
                ok = nrun.main(argv)
        except BaseException as e:      # Ctrl+C included
            ok, exc = None, e
        records.MISSION_CTX['log_folder'] = None
        records.MISSION_CTX['segment_log'] = None
        return dict(ok=ok, exc=exc, out=out.getvalue(), events=list(SESSION['events']), sent=spot.sent - sent0,
                    cleared=spot.cleared - cleared0, d=float(np.hypot(T.ROBOT.x - 9.0, T.ROBOT.y - 2.0)))

    def ev(r):
        return [e for e in r['events'] if e != 'cmd']

    to = "{:.4f},{:.4f}"
    try:
        goal_m = (7.5, 0.5)                     # (9, 2) of today in the mission frame
        r = session(['fake', '--to', to.format(*goal_m), '--yes'], fresh_spot=True)
        check(r['ok'] and r['d'] < 0.8, f"session: arrived {r['d']:.2f} m from the goal")
        e = ev(r)
        check(r['sent'] == len(SESSION['wps']) and r['cleared'] == 1 and e[:2] == ['power_on', 'stand'],
              f"graph uploaded before powering the motors ({r['sent']} snapshots), then standing")
        check(e[-3:] == ['sit', 'power_off', 'save_maps'], f"end: sit, motors off, then the maps ({e[-3:]})")
        check('Localized: the robot is at (0.00, 0.00)' in r['out'], "position said before starting")
        logs = [d for d in os.listdir(os.path.join(root, 'MissionLogs')) if d.startswith('Navigation_')]
        check(len(logs) == 1 and os.path.exists(os.path.join(root, 'MissionLogs', logs[0], 'navigation_log.txt')),
              "log folder MissionLogs/Navigation_<date> with navigation_log.txt")

        time.sleep(1.0)                          # another Navigation_<date> folder
        r = session(['fake', '--to', to.format(*goal_m), '--yes'])
        check(r['ok'] and r['sent'] == 0 and r['cleared'] == 0 and 'already has this graph' in r['out'],
              "second session, graph already on Spot: not cleared, no snapshot sent")

        time.sleep(1.0)
        r = session(['fake', '--to', to.format(*goal_m)])
        check(r['ok'] is False and 'No terminal to confirm on' in r['out'] and 'cmd' not in r['events']
              and T.ROBOT.commands == 0 and ev(r)[-2:] == ['sit', 'power_off'],
              f"no terminal and no --yes: no move, sitting down ({ev(r)})")

        real_nav = motion.navigate_to
        for how, err in (("exception", RuntimeError("fake failure")), ("Ctrl+C", KeyboardInterrupt())):
            n = {'k': 0}

            def nav_to(*a, _err=err, **k):
                n['k'] += 1
                if n['k'] == 3:
                    raise _err
                return real_nav(*a, **k)
            motion.navigate_to = nav_to
            time.sleep(1.0)
            try:
                r = session(['fake', '--to', to.format(*goal_m), '--yes'])
            finally:
                motion.navigate_to = real_nav
            e = ev(r)
            check(type(r['exc']) is type(err), f"{how}: the error goes up ({type(r['exc']).__name__})")
            check(e[-4:] == ['stop', 'sit', 'power_off', 'save_maps'] and '[EMERGENCY]' in r['out'],
                  f"{how}: stop, sit, motors off, then the maps ({e[-4:]})")

        def fall(*a, **k):
            raise motion.RobotFallenError("fake: cannot get up")
        motion.check_and_recover_fall = fall
        time.sleep(1.0)
        try:
            r = session(['fake', '--to', to.format(*goal_m), '--yes'])
        finally:
            motion.check_and_recover_fall = real['fall']
        e = ev(r)
        check(r['ok'] is False and '[FALL]' in r['out'] and 'sit' not in e and 'power_off' not in e
              and e[-1] == 'save_maps',
              f"fall that cannot be recovered: no sitting down, maps saved ({e[-3:]})")
    finally:
        connection.setLogInfo, connection.SimpleEstop = real['setLogInfo'], real['SimpleEstop']
        bosdyn.client.lease.LeaseKeepAlive, robot_command.blocking_stand = real['lease'], real['stand']
        spot_grid.LocalGrid, sf.ArcVerificationTracker = real['LocalGrid'], real['tracker']
        battery.read_battery, records.BodyTiltRecorder = real['read'], real['tilt']
        nrun.MissionMap.load, nrun.REPO_ROOT = real['load'], real['root']
        motion.navigate_to, motion.check_and_recover_fall = real['nav_to'], real['fall']
        global_maps.GlobalGrid.save_map = real['save']
        map_recording.RecordingInterface.initialize_with_fiducial = real['fid']


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
    test_frame_jump(wall)
    test_confirm_start(wall)
    test_session(wall)

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
