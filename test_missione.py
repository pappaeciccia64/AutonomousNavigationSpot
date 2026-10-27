"""
Prova di MISSIONE INTERA: easy_walk.main() vero, dall'avvio alla seduta finale, contro un
robot finto in un mondo sintetico e un GraphNav finto.

test_e2e_loop.py prova l'ingresso in UNA cella; qui gira tutto il resto: scelta della cella
successiva, spostamenti GraphNav fra celle (con il controllo prima dello spostamento e le
scorciatoie), recupero dopo un tentativo fallito, celle bloccate e rimandate, scansione finale,
chiusura d'anello e ancoraggio, ritorno a wp_0, scaricamento della mappa.

Cosa e' finto, oltre a quanto gia' finto in test_e2e_loop.py (posizione, griglia locale,
movimento, fronte):
  - il robot e i suoi client (login, E-Stop, lease, comandi);
  - GraphNav: un grafo in memoria. I waypoint nascono dove sta il robot, gli archi della
    registrazione collegano ognuno al precedente, navigate_to porta il robot sul waypoint,
    la localizzazione e' il waypoint piu' vicino.
Tutto il resto e' il codice vero, compreso RecordingInterface di navGraphUtils.py.

    python3 test_missione.py
    E2E_LOG_DIR=/tmp/prima python3 test_missione.py    # salva il log della missione (vedi
                                                        # confronta_log.py)

Due mondi, griglia 3 x 3 celle da 3 m, il robot parte nel centro di (0,0) guardando lungo +x
(colonne in avanti, righe a sinistra). In entrambi la cella (2,2) e' chiusa da muri su tutti i
lati e la (1,1) ha un ostacolo nel centro, quindi il suo obiettivo finisce vicino al bordo e
il robot arriva appena fuori dalla cella.
  A  un muro fra la colonna 1 e la 2 sulla riga 0: si entra in (0,2) da (1,2); scatta il
     controllo prima dello spostamento GraphNav.
  B  il muro piu' lungo e un ostacolo dentro (1,2): blocchi confermati e ritorni con GraphNav.
Cio' che nessuno dei due esercita: il nuovo tentativo subito dopo il recupero ([RECOVERY]
Trovato percorso alternativo), le trappole, le cadute (le prova test_e2e_loop.py).
"""
import contextlib
import io
import logging
import os
import shutil
import sys
import tempfile
import time
from types import SimpleNamespace as NS

import numpy as np

import test_e2e_loop as T          # mondo, robot e griglia locale finti; applica le sue patch
import bosdyn.client.lease
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.client.math_helpers import Quat, SE3Pose

import arcVerification
import easy_walk
import movements
import navGraphUtils
import parametri
import registro
import spotGrid
import spotLogInUtils

CELL = 3.0
ROWS, COLS = 3, 3
START = (1.5, 1.5, 0.0)
_CHIUSA_22 = [(6.1, 9.0, 6.0, 6.2), (6.1, 6.3, 6.0, 9.0), (8.8, 9.0, 6.0, 9.0), (6.1, 9.0, 8.8, 9.0)]
WORLDS = {
    'A': [(5.9, 6.1, 0.0, 3.0)] + _CHIUSA_22 + [(4.2, 4.8, 4.2, 4.8)],
    'B': [(5.9, 6.1, 0.0, 4.6)] + _CHIUSA_22 + [(4.2, 4.8, 4.2, 4.8), (7.0, 7.4, 3.6, 5.4)],
}
WALLS = WORLDS['A']
FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


# ---------------------------------------------------------------------------------------
# GraphNav finto
# ---------------------------------------------------------------------------------------
class FakeGraphNav:
    """Registrazione + GraphNav + elaborazione della mappa, su un grafo in memoria."""

    def __init__(self):
        self.graph = map_pb2.Graph()
        self.pos = {}                  # id -> (x, y, yaw)
        self.recording = False
        self.last_wp = None
        self.n = 0
        self.trips = []                # (da, a) degli spostamenti

    # --- registrazione -------------------------------------------------------------
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
        raise RuntimeError("STATUS_NO_MATCHING_FIDUCIAL (finto: nessun fiducial nel mondo)")

    def navigate_to(self, waypoint_id, cmd_duration, command_id=None, travel_params=None, **kw):
        x, y, yaw = self.pos[waypoint_id]
        self.trips.append((self._nearest(), waypoint_id))
        T.ROBOT.x, T.ROBOT.y, T.ROBOT.yaw = x, y, yaw
        T.ROBOT.track.append((x, y))
        return 1

    def navigation_feedback(self, command_id, **kw):
        return NS(status=graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL)

    def clear_graph(self, **kw):
        self.__init__()

    def upload_graph(self, graph=None, **kw):
        for e in graph.edges:
            self.graph.edges.add().CopyFrom(e)

    def download_waypoint_snapshot(self, snapshot_id, **kw):
        return map_pb2.WaypointSnapshot(id=snapshot_id)

    def download_edge_snapshot(self, snapshot_id, **kw):
        return map_pb2.EdgeSnapshot(id=snapshot_id)

    # --- elaborazione della mappa -------------------------------------------------------
    def process_topology(self, params=None, modify_map_on_server=False, **kw):
        return map_processing_pb2.ProcessTopologyResponse(
            status=map_processing_pb2.ProcessTopologyResponse.STATUS_OK)

    def process_anchoring(self, *a, **kw):
        r = map_processing_pb2.ProcessAnchoringResponse(
            status=map_processing_pb2.ProcessAnchoringResponse.STATUS_OK)
        r.iteration = 1
        return r


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


def run(walls=None, name='MISSIONE'):
    T.WORLD, T.ROBOT = T.World(WALLS if walls is None else walls, ghosts=True), T.Robot(*START)
    arcVerification.OBSTACLE_HISTORY.reset()
    gn = FakeGraphNav()
    robot = FakeRobot(gn)
    spotLogInUtils.setLogInfo = lambda o: (robot, NS(), robot.state, None)
    spotLogInUtils.SimpleEstop = lambda r, n: NS(stop=lambda: None, allow=lambda: None)
    bosdyn.client.lease.LeaseKeepAlive = lambda *a, **k: contextlib.nullcontext()
    easy_walk.blocking_stand = lambda *a, **k: None
    spotGrid.LocalGrid = lambda r: T.FakeLocalGrid()
    arcVerification.ArcVerificationTracker = lambda r: _Tracker()
    easy_walk.BodyTiltRecorder = registro.BodyTiltRecorder = _NoTilt
    parametri.MISSION_GRAPH_SAMPLING = 'reticolo'

    root = tempfile.mkdtemp(prefix="missione_finta_")
    easy_walk.__file__ = os.path.join(root, "easy_walk.py")      # cartelle dei dati qui
    argv = sys.argv
    sys.argv = ["easy_walk.py", "--rows", str(ROWS), "--cols", str(COLS), "--cell", str(CELL),
                "--grafo", "reticolo", "--spacing", "0.5"]
    t0 = time.time()
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            ok = easy_walk.main()
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


if __name__ == '__main__':
    for name, walls in WORLDS.items():
        print(f"\nMissione intera nel mondo {name}, robot e GraphNav finti: 3 x 3 celle da 3 m.")
        r = run(walls, f"MISSIONE_{name}")
        log = r['log']
        print(f"  ({r['seconds']:.0f} s)")
        check(r['ok'] is True, "main() arriva in fondo senza eccezioni")
        check('Traceback' not in log and 'Traceback' not in r['out'], "nessuna eccezione nel log")
        grid = log.split("--- Mappa delle celle ---")[-1].splitlines()[1:ROWS + 1]
        cells = [line.split()[1:] for line in grid]
        visited = sum(v == '1' for row in cells for v in row)
        check(visited == 8, f"visitate tutte le 8 celle raggiungibili (visitate {visited})")
        check(cells and cells[2][2] == '-1', "la cella chiusa (2,2) e' bloccata")
        check("[OBIETTIVO] Il robot e' fuori dalla cella (1,1)" in log,
              "in (1,1) l'arrivo vale con il robot appena fuori dalla cella")
        check(len(r['gn'].trips) >= 2, f"spostamenti GraphNav: {len(r['gn'].trips)}")
        check("ritorno a wp_0: arrivato" in log, "ritorno a wp_0 riuscito")
        check("[FINAL SCAN] Fatto" in log, "scansione finale fatta")
        check("[ANCHORING]" in log, "ancoraggio finale chiamato")
        if name == 'A':
            check("[PRECHECK]" in log, "il controllo prima dello spostamento GraphNav scatta")
        if name == 'B':
            check("[BLOCCO] Confermato" in log and "[RITORNO]" in log, "blocco confermato e ritorno con GraphNav")
        shutil.rmtree(r['root'], ignore_errors=True)
    print("\n" + "=" * 70)
    print("TUTTO OK" if not FAILS else f"{len(FAILS)} CONTROLLI FALLITI:\n  - " + "\n  - ".join(FAILS))
    sys.exit(1 if FAILS else 0)
