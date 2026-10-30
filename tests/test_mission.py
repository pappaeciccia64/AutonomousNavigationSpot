"""
WHOLE MISSION test: the real run.main(), from start-up to the final sit, against a fake
robot in a synthetic world and a fake GraphNav.

test_e2e_loop.py tests entering ONE cell; here everything else runs: choice of the next
cell, GraphNav moves between cells (with the check before the move and the shortcuts),
recovery after a failed attempt, blocked and deferred cells, final scan, loop closure and
anchoring, return to wp_0, map download.

What is fake, besides what is already fake in test_e2e_loop.py (position, local grid,
motion, frontier):
  - the robot and its clients (login, E-Stop, lease, commands);
  - GraphNav: an in-memory graph. Waypoints are created where the robot is, the recording
    edges link each one to the previous, navigate_to puts the robot on the waypoint,
    localization is the nearest waypoint.
Everything else is the real code, including RecordingInterface from spot_explorer.graphnav.map_recording.

    python3 tests/test_mission.py
    E2E_LOG_DIR=/tmp/before python3 tests/test_mission.py   # saves the mission log (see
                                                             # tools/compare_logs.py)

The worlds have a 3 x 3 grid of 3 m cells, the robot starts in the centre of (0,0) looking
along +x (columns ahead, rows to the left). In all of them cell (2,2) is closed by walls on
all sides and (1,1) has an obstacle in the centre, so its goal ends up near the border and
the robot arrives just outside the cell.
  A  a wall between column 1 and 2 on row 0: (0,2) is entered from (1,2); the check before
     the GraphNav move triggers.
  B  the longer wall and an obstacle inside (1,2): confirmed blocks and returns with GraphNav.
  C  world B with forced failures (see _with_failures): attempts failed on purpose, no goal
     point, a fall during a GraphNav move, the return to wp_0 stuck (STUCK) 0.60 m from the
     start. Exercises the new attempt right after recovery ([RECOVERY]), successful and
     failed, and the mission branches that real worlds rarely reach.
  D  world A with the fake battery dropping one point per reading, from 36%: below 30% the
     mission stops exploring and closes as at the end of a mission (return to wp_0).
In world A there is also a fiducial (ID 7) behind the starting point.
What nobody exercises: traps and falls inside an attempt (test_e2e_loop.py tests them).
"""
import contextlib
import glob
import io
import logging
import os
import shutil
import sys
import tempfile
import time
from types import SimpleNamespace as NS

import numpy as np

import test_e2e_loop as T          # fake world, robot and local grid; applies its patches
import bosdyn.client.lease
from bosdyn.api import world_object_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.client.math_helpers import Quat, SE3Pose

from spot_explorer.planning import safe_frontier
from spot_explorer.robot import battery as batteria
from spot_explorer.mission import cell_attempt, exploration
from spot_explorer.mission import run as mission_run
from spot_explorer.planning import goal
from spot_explorer.robot import motion as movements
from spot_explorer.graphnav import map_recording
from spot_explorer import config as parametri
from spot_explorer import records as registro
from spot_explorer.perception import spot_grid
from spot_explorer.robot import connection as spotLogInUtils

CELL = 3.0
ROWS, COLS = 3, 3
START = (1.5, 1.5, 0.0)
_CLOSED_22 = [(6.1, 9.0, 6.0, 6.2), (6.1, 6.3, 6.0, 9.0), (8.8, 9.0, 6.0, 9.0), (6.1, 9.0, 8.8, 9.0)]
WORLDS = {
    'A': [(5.9, 6.1, 0.0, 3.0)] + _CLOSED_22 + [(4.2, 4.8, 4.2, 4.8)],
    'B': [(5.9, 6.1, 0.0, 4.6)] + _CLOSED_22 + [(4.2, 4.8, 4.2, 4.8), (7.0, 7.4, 3.6, 5.4)],
}
WALLS = WORLDS['A']
# Fiducial of the current world: (ID, x, y), None = none. In world A a tag 1 m BEHIND the
# starting point (the rear camera sees it).
FIDUCIAL_TAG = None
FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


# ---------------------------------------------------------------------------------------
# Fake GraphNav
# ---------------------------------------------------------------------------------------
class FakeGraphNav:
    """Recording + GraphNav + map processing, on an in-memory graph."""

    def __init__(self):
        self.graph = map_pb2.Graph()
        self.pos = {}                  # id -> (x, y, yaw)
        self.recording = False
        self.last_wp = None
        self.n = 0
        self.trips = []                # (from, to) of the moves
        self.headings = []             # arrival angles requested (relative to the waypoint)
        self.stuck_wp0_m = None        # return to wp_0 stuck (STUCK) at this distance
        self.wp0_offset_m = None       # return to wp_0 "arrived" but off by this much (see run)
        self.wp0_returns = 0           # returns to wp_0 done
        self.relocated = False         # relocalized on the tag AFTER a return to wp_0
        self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL

    # --- recording -----------------------------------------------------------------------
    def _new_waypoint(self, name):
        self.n += 1
        wid = f"wp-finto-{self.n}"
        w = self.graph.waypoints.add()
        w.id = wid
        w.annotations.name = name
        w.annotations.creation_time.seconds = int(time.time())
        x, y, yaw = T.ROBOT.x, T.ROBOT.y, T.ROBOT.yaw
        self.pos[wid] = (x, y, yaw)
        ko = SE3Pose(x, y, 0.0, Quat.from_yaw(yaw)).inverse()
        w.waypoint_tform_ko.CopyFrom(ko.to_proto())
        if self.last_wp is not None:
            self._edge(self.last_wp, wid, map_pb2.Edge.EDGE_SOURCE_ODOMETRY)
        self.last_wp = wid
        return w

    def _edge(self, a, b, source):
        e = self.graph.edges.add()
        e.id.from_waypoint, e.id.to_waypoint = a, b
        (xa, ya, ta), (xb, yb, tb) = self.pos[a], self.pos[b]
        rel = SE3Pose(xa, ya, 0, Quat.from_yaw(ta)).inverse() * SE3Pose(xb, yb, 0, Quat.from_yaw(tb))
        e.from_tform_to.CopyFrom(rel.to_proto())
        e.annotations.edge_source = source

    def start_recording(self, **kw):
        self.recording = True
        self.last_wp = self._nearest()
        r = recording_pb2.StartRecordingResponse()
        r.status = recording_pb2.StartRecordingResponse.STATUS_OK
        return r

    def stop_recording(self, **kw):
        self.recording = False
        r = recording_pb2.StopRecordingResponse()
        r.status = recording_pb2.StopRecordingResponse.STATUS_OK
        return r

    def get_record_status(self, **kw):
        return NS(is_recording=self.recording)

    def create_waypoint(self, waypoint_name=None, **kw):
        w = self._new_waypoint(waypoint_name or f"auto_{self.n}")
        r = recording_pb2.CreateWaypointResponse()
        r.status = recording_pb2.CreateWaypointResponse.STATUS_OK
        r.created_waypoint.CopyFrom(w)
        return r

    def create_edge(self, edge=None, **kw):
        if not self.recording:
            raise RuntimeError("NotRecordingError (finto)")
        self.graph.edges.add().CopyFrom(edge)

    # --- GraphNav ------------------------------------------------------------------
    def _nearest(self):
        if not self.pos:
            return None
        return min(self.pos, key=lambda k: np.hypot(self.pos[k][0] - T.ROBOT.x, self.pos[k][1] - T.ROBOT.y))

    def download_graph(self, **kw):
        g = map_pb2.Graph()
        g.CopyFrom(self.graph)
        return g

    def get_localization_state(self, **kw):
        st = graph_nav_pb2.GetLocalizationStateResponse()
        wid = self._nearest()
        if wid is not None:
            st.localization.waypoint_id = wid
            x, y, yaw = self.pos[wid]
            rel = SE3Pose(x, y, 0, Quat.from_yaw(yaw)).inverse() * \
                SE3Pose(T.ROBOT.x, T.ROBOT.y, 0, Quat.from_yaw(T.ROBOT.yaw))
            st.localization.waypoint_tform_body.CopyFrom(rel.to_proto())
        return st

    def set_localization(self, **kw):
        if (FIDUCIAL_TAG is not None and kw.get('use_fiducial_id') == FIDUCIAL_TAG[0]
                and kw.get('fiducial_init') == graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_SPECIFIC):
            if self.wp0_returns > 0:
                self.relocated = True             # from here GraphNav really knows where wp_0 is
            return graph_nav_pb2.SetLocalizationResponse(status=graph_nav_pb2.SetLocalizationResponse.STATUS_OK)
        raise RuntimeError("STATUS_NO_MATCHING_FIDUCIAL (finto: nessun fiducial nel mondo)")

    def navigate_to(self, waypoint_id, cmd_duration, command_id=None, travel_params=None,
                    destination_waypoint_tform_body_goal=None, **kw):
        x, y, yaw = self.pos[waypoint_id]
        if destination_waypoint_tform_body_goal is not None:     # turned arrival (angle only)
            yaw += destination_waypoint_tform_body_goal.angle
            self.headings.append(destination_waypoint_tform_body_goal.angle)
        self.trips.append((self._nearest(), waypoint_id))
        self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        name = next(w.annotations.name for w in self.graph.waypoints if w.id == waypoint_id)
        if name == 'wp_0' and self.stuck_wp0_m is not None:
            x += self.stuck_wp0_m                 # stopped short, on the way from +x
            self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK
        if name == 'wp_0':
            self.wp0_returns += 1
            if self.wp0_offset_m is not None and not self.relocated:
                y += self.wp0_offset_m            # wrong localization: "arrived" to one side
        T.ROBOT.x, T.ROBOT.y, T.ROBOT.yaw = x, y, yaw
        T.ROBOT.track.append((x, y))
        return 1

    def navigation_feedback(self, command_id, **kw):
        return NS(status=self.status)

    def clear_graph(self, **kw):
        stuck, offset = self.stuck_wp0_m, self.wp0_offset_m
        self.__init__()
        self.stuck_wp0_m, self.wp0_offset_m = stuck, offset

    def upload_graph(self, graph=None, **kw):
        for e in graph.edges:
            self.graph.edges.add().CopyFrom(e)

    def download_waypoint_snapshot(self, snapshot_id, **kw):
        return map_pb2.WaypointSnapshot(id=snapshot_id)

    def download_edge_snapshot(self, snapshot_id, **kw):
        return map_pb2.EdgeSnapshot(id=snapshot_id)

    # --- map processing ------------------------------------------------------------------
    def process_topology(self, params=None, modify_map_on_server=False, **kw):
        return map_processing_pb2.ProcessTopologyResponse(
            status=map_processing_pb2.ProcessTopologyResponse.STATUS_OK)

    def process_anchoring(self, *a, **kw):
        r = map_processing_pb2.ProcessAnchoringResponse(
            status=map_processing_pb2.ProcessAnchoringResponse.STATUS_OK)
        r.iteration = 1
        return r


class FakeWorldObjects:
    """The world-object service: the tag FIDUCIAL_TAG, always in view, or none."""

    def list_world_objects(self, object_type=None, **kw):
        resp = world_object_pb2.ListWorldObjectResponse()
        if FIDUCIAL_TAG is None:
            return resp
        tid, x, y = FIDUCIAL_TAG
        o = resp.world_objects.add()
        o.apriltag_properties.tag_id = tid
        o.apriltag_properties.frame_name_fiducial = f"fiducial_{tid}"
        o.apriltag_properties.frame_name_camera = "back"
        edges = o.transforms_snapshot.child_to_parent_edge_map
        edges['vision'].parent_frame_name = ''
        edges['vision'].parent_tform_child.rotation.w = 1.0
        e = edges[f"fiducial_{tid}"]
        e.parent_frame_name = 'vision'
        e.parent_tform_child.position.x, e.parent_tform_child.position.y = x, y
        e.parent_tform_child.rotation.w = 1.0
        return resp


def _robot_state_with_odom():
    """Robot state with only odom -> body at the fake robot's pose (for
    initialize_with_fiducial: the rest of the test wants a state that fails)."""
    from bosdyn.api import robot_state_pb2
    st = robot_state_pb2.RobotState()
    edges = st.kinematic_state.transforms_snapshot.child_to_parent_edge_map
    edges['odom'].parent_frame_name = ''
    edges['odom'].parent_tform_child.rotation.w = 1.0
    body = SE3Pose(T.ROBOT.x, T.ROBOT.y, 0, Quat.from_yaw(T.ROBOT.yaw))
    edges['body'].parent_frame_name = 'odom'
    edges['body'].parent_tform_child.CopyFrom(body.to_proto())
    return st


_real_initialize_with_fiducial = map_recording.RecordingInterface.initialize_with_fiducial


def _initialize_with_fiducial(self, robot_state_client, fiducial_id=None):
    return _real_initialize_with_fiducial(self, NS(get_robot_state=_robot_state_with_odom), fiducial_id)


map_recording.RecordingInterface.initialize_with_fiducial = _initialize_with_fiducial


class FakeRobot:
    def __init__(self, gn):
        self.gn = gn
        self.logger = logging.getLogger("robot-finto")
        self.time_sync = NS(wait_for_sync=lambda *a, **k: None)
        self._current_user = "prova"
        self.powered = False
        self.state = NS(get_robot_state=lambda: (_ for _ in ()).throw(RuntimeError("stato finto")))
        self.commands = NS(robot_command=lambda *a, **k: 1)

    def ensure_client(self, name):
        if name in ('recording-service', 'graph-nav-service', 'map-processing-service'):
            return self.gn
        if name == 'robot-state':
            return self.state
        if name == 'world-objects':
            return FakeWorldObjects()
        return self.commands

    def power_on(self, **kw):
        self.powered = True

    def power_off(self, **kw):
        self.powered = False

    def is_powered_on(self):
        return self.powered


class _NoTilt(T._NoTilt):
    samples = []


class _Tracker(T.FakeTracker):
    def start(self):
        pass

    def stop(self):
        pass


@contextlib.contextmanager
def _with_failures():
    """Forced failures, always the same: the 1st, 4th, 7th... attempt and the 2nd fail
    without moving the robot, one goal choice in 5 finds no points, the first GraphNav
    move ends with a fall (fake: the robot is already standing)."""
    real_attempt = cell_attempt.attempt_enter_cell_from_position
    real_best = goal.find_best_point_in_cell
    real_fall = movements.check_and_recover_fall
    n = {'tentativi': 0, 'obiettivi': 0, 'viaggi': 0}

    def attempt(*a, **k):
        n['tentativi'] += 1
        if n['tentativi'] % 3 == 1 or n['tentativi'] % 7 == 2:
            print(f"[FINTO] tentativo {n['tentativi']} fallito d'ufficio")
            return False
        return real_attempt(*a, **k)

    def best(*a, **k):
        n['obiettivi'] += 1
        if n['obiettivi'] % 5 == 3:
            print(f"[FINTO] nessun punto obiettivo ({n['obiettivi']})")
            return None, None, None, None
        return real_best(*a, **k)

    def fall(rsc, cc, gm, why, *a, **k):
        if why == "durante lo spostamento con GraphNav":
            n['viaggi'] += 1
            if n['viaggi'] == 1:
                print("[FINTO] caduta durante lo spostamento GraphNav")
                return 'caduto'
        return real_fall(rsc, cc, gm, why, *a, **k)

    cell_attempt.attempt_enter_cell_from_position = attempt
    goal.find_best_point_in_cell = best
    movements.check_and_recover_fall = fall
    try:
        yield n
    finally:
        cell_attempt.attempt_enter_cell_from_position = real_attempt
        goal.find_best_point_in_cell = real_best
        movements.check_and_recover_fall = real_fall


@contextlib.contextmanager
def _with_battery(from_pct=36.0, points_per_read=1.0):
    """Fake battery: every reading (one per loop turn, interval 0 s) one point less,
    from from_pct."""
    real_read, real_s = batteria.read_battery, parametri.BATTERY_SAMPLE_S
    n = {'letture': 0}

    def read(rsc):
        n['letture'] += 1
        return {'charge_pct': max(0.0, from_pct - points_per_read * (n['letture'] - 1)),
                'spot_runtime_min': 40.0}

    batteria.read_battery = read
    parametri.BATTERY_SAMPLE_S = 0.0
    try:
        yield n
    finally:
        batteria.read_battery = real_read
        parametri.BATTERY_SAMPLE_S = real_s


def run(walls=None, name='MISSIONE', fiducial=None, stuck_wp0_m=None, wp0_offset_m=None):
    global FIDUCIAL_TAG
    FIDUCIAL_TAG = fiducial
    T.WORLD, T.ROBOT = T.World(WALLS if walls is None else walls, ghosts=True), T.Robot(*START)
    safe_frontier.OBSTACLE_HISTORY.reset()
    gn = FakeGraphNav()
    gn.stuck_wp0_m = stuck_wp0_m
    gn.wp0_offset_m = wp0_offset_m
    robot = FakeRobot(gn)
    spotLogInUtils.setLogInfo = lambda o: (robot, NS(), robot.state, None)
    spotLogInUtils.SimpleEstop = lambda r, n: NS(stop=lambda: None, allow=lambda: None)
    bosdyn.client.lease.LeaseKeepAlive = lambda *a, **k: contextlib.nullcontext()
    mission_run.blocking_stand = lambda *a, **k: None
    spot_grid.LocalGrid = lambda r: T.FakeLocalGrid()
    safe_frontier.ArcVerificationTracker = lambda r: _Tracker()
    cell_attempt.BodyTiltRecorder = registro.BodyTiltRecorder = _NoTilt
    parametri.MISSION_GRAPH_SAMPLING = 'lattice'

    root = tempfile.mkdtemp(prefix="missione_finta_")
    mission_run.DATA_ROOT = root                                    # data folders here
    argv = sys.argv
    sys.argv = ["run_mission.py", "--rows", str(ROWS), "--cols", str(COLS), "--cell", str(CELL),
                "--graph", "lattice", "--spacing", "0.5"]
    t0 = time.time()
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            ok = mission_run.main()
    finally:
        sys.argv = argv
    logs = [os.path.join(dp, f) for dp, _, fs in os.walk(os.path.join(root, "MissionLogs")) for f in fs
            if f == "mission_log.txt"]
    log = open(logs[0]).read() if logs else out.getvalue()
    if os.environ.get('E2E_LOG_DIR'):
        os.makedirs(os.environ['E2E_LOG_DIR'], exist_ok=True)
        with open(os.path.join(os.environ['E2E_LOG_DIR'], name + ".log"), "w") as fh:
            fh.write(log)
    return dict(ok=ok, log=log, gn=gn, root=root, seconds=time.time() - t0, out=out.getvalue())


def _check_common(r, return_ok=True):
    log = r['log']
    print(f"  ({r['seconds']:.0f} s)")
    check(r['ok'] is True, "main() arriva in fondo senza eccezioni")
    check('Traceback' not in log and 'Traceback' not in r['out'], "nessuna eccezione nel log")
    if return_ok:
        check("ritorno a wp_0: arrivato" in log, "ritorno a wp_0 riuscito")
    check("[FINAL SCAN] Fatto" in log, "scansione finale fatta")
    check("[ANCHORING]" in log, "ancoraggio finale chiamato")


def _check_world(name, r):
    log = r['log']
    grid = log.split("--- Mappa delle celle ---")[-1].splitlines()[1:ROWS + 1]
    cells = [line.split()[1:] for line in grid]
    visited = sum(v == '1' for row in cells for v in row)
    check(visited == 8, f"visitate tutte le 8 celle raggiungibili (visitate {visited})")
    check(cells and cells[2][2] == '-1', "la cella chiusa (2,2) e' bloccata")
    check("[OBIETTIVO] Il robot e' fuori dalla cella (1,1)" in log,
          "in (1,1) l'arrivo vale con il robot appena fuori dalla cella")
    check(len(r['gn'].trips) >= 2, f"spostamenti GraphNav: {len(r['gn'].trips)}")
    check("[PROGRESSO] Fine esplorazione: 9/9 celle chiuse (100%): 8 visitate, 1 bloccate" in log,
          "avanzamento finale: 9/9 celle chiuse, 8 visitate e 1 bloccata")
    lines = [l for l in log.splitlines() if '[PROGRESSO]' in l and 'Fine' not in l]
    closed = [int(l.split('[PROGRESSO] ')[1].split('/')[0]) for l in lines]
    check(closed == sorted(set(closed)), f"una riga [PROGRESSO] per ogni cella chiusa in piu': {closed}")
    check("[BATTERIA] Carica non disponibile" in log and "RIENTRO" not in log,
          "senza la carica nello stato del robot la missione va avanti senza rientro")
    check("arrivo girato verso" in log and len(r['gn'].headings) > 0,
          f"viaggi GraphNav con l'arrivo girato verso la cella ({len(r['gn'].headings)})")
    check(len(r['gn'].headings) < len(r['gn'].trips),
          "il ritorno a wp_0 tiene l'orientamento del waypoint")
    if name == 'A':
        check("[PRECHECK]" in log, "il controllo prima dello spostamento GraphNav scatta")
        check("[FIDUCIAL] Fiducial di partenza: tag 7 a 1.00 m, dietro" in log,
              "fiducial di partenza visto dietro il robot, qualunque ID")
        check("[FIDUCIAL INIT] Fiducial cercato: ID 7" in log and "[FIDUCIAL INIT] ✓" in log,
              "localizzazione sul fiducial trovato, dopo wp_0")
        check("[WP0] Secondo il tag il robot e' a 0.00 m dal punto di partenza" in log
              and "[WP0] CORREZIONE" not in log, "ritorno a wp_0 giusto: nessuna correzione")
        check("rivedo il tag 7" in log and "si e' spostato di 0.00 m" in log,
              "al ritorno su wp_0 il tag si rivede e la deriva si misura")
    else:
        check("[FIDUCIAL] Nessun fiducial" in log and "[FIDUCIAL INIT]" not in log,
              "senza fiducial si parte senza, e senza tentare la localizzazione")
    if name == 'B':
        check("[BLOCCO] Confermato" in log and "[RITORNO]" in log, "blocco confermato e ritorno con GraphNav")


def _check_failures(r):
    log = r['log']
    check("ritorno a wp_0: arrivato" not in log
          and "Ritorno a wp_0 NON riuscito ma vicino: fermo a 0.60 m da wp_0 (motivo: bloccato (STUCK)" in log,
          "ritorno a wp_0 fermo a 0.60 m: NON riuscito, ma vicino")
    check("[DERIVA] ritorno a wp_0 (fermo vicino)" in log, "deriva misurata anche fermo vicino a wp_0")
    check("Ritorno a wp_0 non riuscito: il robot si siede" in log, "il robot si siede dove si trova")
    check("[RECOVERY] Trovato percorso alternativo" in log, "nuovo tentativo subito dopo il recupero")
    check("[FAIL] Anche il percorso alternativo" in log, "nuovo tentativo fallito: lato segnato")
    check("[SKIP] Nessun punto obiettivo valido" in log, "nessun punto obiettivo nel recupero")
    check("[CADUTA] Rialzato dopo lo spostamento con GraphNav" in log, "caduta durante il viaggio")
    check("[PROGRESSO] Fine esplorazione: 9/9 celle chiuse" in log and "SKIP-ROUND" not in log,
          "nessuna cella persa dalla frontiera: ogni cella rimandata ritentata dagli altri lati")


def _check_emergency():
    """Mission interrupted halfway (exception or Ctrl+C): the robot sits, the motors power off
    and the GraphNav map is downloaded, then the error propagates as before."""
    for nome, err in (("eccezione", RuntimeError("guasto finto")), ("Ctrl+C", KeyboardInterrupt())):
        print(f"\nMissione intera nel mondo F: interrotta da {nome} al terzo tentativo.")
        real = cell_attempt.attempt_enter_cell_from_position
        n = {'k': 0}

        def attempt(*a, _err=err, **k):
            n['k'] += 1
            if n['k'] == 3:
                raise _err
            return real(*a, **k)
        cell_attempt.attempt_enter_cell_from_position = attempt
        try:
            try:
                r = run(WORLDS['A'], f"MISSIONE_F_{'eccezione' if nome == 'eccezione' else 'ctrlc'}")
                rise = r['ok'] is False
            except KeyboardInterrupt:
                rise, r = True, None
        finally:
            cell_attempt.attempt_enter_cell_from_position = real
        if r is None:      # Ctrl+C: the log is in the mission folder
            logs = sorted(glob.glob(os.path.join(tempfile.gettempdir(), "missione_finta_*", "MissionLogs",
                                                 "*", "mission_log.txt")), key=os.path.getmtime)
            log = open(logs[-1]).read() if logs else ""
        else:
            log = r['log']
        check(rise, f"{nome}: la missione finisce con l'errore, come prima")
        check("[EMERGENZA] Robot seduto e motori spenti." in log, f"{nome}: robot seduto e motori spenti")
        check("[EMERGENZA] Mappa GraphNav scaricata." in log, f"{nome}: mappa GraphNav scaricata")


def _check_cell_choice():
    """
    The case of 28-10 13:50, 8 x 7 grid of 2 m: (7,2) was visited with the robot just
    outside, in (6,3), and (7,1) had already been tried from the side of (6,1). The robot
    must look for the neighbours from (7,2), and approach (7,1) from (7,2), not again from
    (6,1) (before: a jump, then the cell dropped out of the frontier).
    """
    print("\nScelta delle celle: il caso delle 13:50 (celle (7,1) e (7,2))")
    from spot_explorer.mission import cells as environmentMap
    env = environmentMap.EnvironmentMap(rows=8, cols=7, cell_size=2.0)
    with contextlib.redirect_stdout(io.StringIO()):
        env.set_origin(0.0, 0.0, 0.0)
        for r_ in range(8):
            for c_ in range(7):
                if (r_, c_) not in ((7, 0), (7, 1)):
                    env.mark_cell_visited(r_, c_)
        env.mark_cell_side_explored(7, 1, env.get_side_bit_facing_origin(6, 1, 7, 1))
    wps = {(r_, c_): {'id': f"wp_{r_}_{c_}", 'x': 2.0 * c_, 'y': 2.0 * r_}
           for r_ in range(8) for c_ in range(7) if env.map[r_][c_] == 1}
    ri = NS(get_manual_waypoint_by_cell=lambda r_, c_: wps.get((r_, c_)),
            find_nearest_waypoint_cell_to_target=lambda *a: (6, 1))
    m = exploration.Mission.__new__(exploration.Mission)
    m.env, m.recordingInterface, m.robot_state_client = env, ri, None
    m._last_visited = (7, 2)
    T.ROBOT.x, T.ROBOT.y = 4.0, 12.75                 # 0.25 m outside (7,2), in (6,2)
    with contextlib.redirect_stdout(io.StringIO()):
        cell = m._robot_cell()
        start_xy = m._start_cell_for(7, 1, wps)
    check(cell == (7, 2), f"robot appena fuori dalla (7,2) appena visitata: cella del robot {cell}")
    check(start_xy == (7, 2), f"avvicinamento alla (7,1) dal lato non ancora provato: dalla {start_xy}")
    T.ROBOT.x, T.ROBOT.y = 4.0, 11.5                  # well inside (6,2): the real cell counts
    with contextlib.redirect_stdout(io.StringIO()):
        cell = m._robot_cell()
    check(cell == (6, 2), f"robot lontano dalla cella visitata: vale quella in cui sta ({cell})")


def _check_battery(r):
    log = r['log']
    grid = log.split("--- Mappa delle celle ---")[-1].splitlines()[1:ROWS + 1]
    visited = sum(v == '1' for line in grid for v in line.split()[1:])
    check("[BATTERIA] RIENTRO: carica 29% <= soglia 30% (2 letture di fila)" in log,
          "il rientro scatta alla seconda lettura sotto il 30%")
    check("[BATTERIA] Fine dell'esplorazione per batteria bassa" in log and 0 < visited < 8,
          f"l'esplorazione si ferma prima di finire (visitate {visited} celle su 8)")
    check("[BATTERIA] Ritorno a wp_0: da" in log, "carica misurata prima e dopo il ritorno a wp_0")
    check("RIENTRO ANTICIPATO" in log, "il riepilogo dice che la missione e' rientrata per la batteria")
    csv_path = [os.path.join(dp, f) for dp, _, fs in os.walk(os.path.join(r['root'], "MissionLogs"))
                for f in fs if f == "battery.csv"]
    check(bool(csv_path) and len(open(csv_path[0]).read().splitlines()) > 5, "battery.csv scritto")
    js = [os.path.join(dp, f) for dp, _, fs in os.walk(os.path.join(r['root'], "MissionLogs"))
          for f in fs if f == "mission_summary.json"]
    import json
    st = json.load(open(js[0]))['mission_stats'] if js else {}
    check(st.get('battery_return') is True and st.get('battery_return_cells_left', 0) > 0,
          f"mission_summary.json: rientro e celle rimaste ({st.get('battery_return_cells_left')})")


if __name__ == '__main__':
    for name, walls in WORLDS.items():
        print(f"\nMissione intera nel mondo {name}, robot e GraphNav finti: 3 x 3 celle da 3 m.")
        r = run(walls, f"MISSIONE_{name}",
                fiducial=(7, START[0] - 1.0, START[1]) if name == 'A' else None)
        _check_common(r)
        _check_world(name, r)
        shutil.rmtree(r['root'], ignore_errors=True)
    print("\nMissione intera nel mondo C: il B con fallimenti forzati.")
    with _with_failures():
        r = run(WORLDS['B'], "MISSIONE_C", stuck_wp0_m=0.6)
    _check_common(r, return_ok=False)
    _check_failures(r)
    shutil.rmtree(r['root'], ignore_errors=True)
    _check_cell_choice()
    _check_emergency()
    print("\nMissione intera nel mondo G: il mondo A con il ritorno a wp_0 'arrivato' ma 0.8 m di lato "
          "(29-10 10:57: 1.2 m).")
    r = run(WORLDS['A'], "MISSIONE_G", fiducial=(7, START[0] - 1.0, START[1]), wp0_offset_m=0.8)
    _check_common(r)
    log = r['log']
    check("[WP0] Secondo il tag il robot e' a 0.80 m dal punto di partenza" in log,
          "il tag misura lo scarto vero dalla partenza (0.80 m)")
    check("[WP0] CORREZIONE 1/2" in log and "correzione 1 del ritorno a wp_0: arrivato" in log,
          "rilocalizzazione sul tag e nuovo ritorno a wp_0")
    check("[WP0] Dopo la correzione 1: a 0.00 m dalla partenza (prima 0.80 m)" in log
          and "CORREZIONE 2/2" not in log, "una correzione basta: 0.00 m dalla partenza")
    shutil.rmtree(r['root'], ignore_errors=True)
    print("\nMissione intera nel mondo E: partenza chiusa tutto intorno (28-10 14:17, erba alta).")
    enclosed = [(0.4, 2.6, 0.4, 0.6), (0.4, 2.6, 2.4, 2.6), (0.4, 0.6, 0.4, 2.6), (2.4, 2.6, 0.4, 2.6)]
    r = run(enclosed, "MISSIONE_E")
    log = r['log']
    check(r['ok'] is True and 'Traceback' not in log, "main() arriva in fondo senza eccezioni")
    check("[FINE] Nessuna cella raggiungibile dalla partenza" in log,
          "la riga [FINE] dice che dalla partenza non si raggiunge nessuna cella")
    check("ritorno a wp_0" in log, "chiusura normale della missione")
    shutil.rmtree(r['root'], ignore_errors=True)
    print("\nMissione intera nel mondo D: il mondo A con la batteria che si scarica.")
    with _with_battery():
        r = run(WORLDS['A'], "MISSIONE_D")
    _check_common(r)
    _check_battery(r)
    shutil.rmtree(r['root'], ignore_errors=True)
    print("\n" + "=" * 70)
    print("TUTTO OK" if not FAILS else f"{len(FAILS)} CONTROLLI FALLITI:\n  - " + "\n  - ".join(FAILS))
    sys.exit(1 if FAILS else 0)
