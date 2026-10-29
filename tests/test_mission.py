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
  C  il mondo B con fallimenti forzati (vedi _con_fallimenti): tentativi falliti d'ufficio,
     nessun punto obiettivo, una caduta durante uno spostamento GraphNav, il ritorno a wp_0
     fermo (STUCK) a 0.60 m dalla partenza. Esercita il nuovo
     tentativo subito dopo il recupero ([RECOVERY]), riuscito e fallito, e i rami della
     missione che i mondi veri raggiungono di rado.
  D  il mondo A con la batteria finta che scende di un punto a lettura, da 36%: sotto il 30%
     la missione smette di esplorare e chiude come a fine missione (rientro a wp_0).
Nel mondo A c'e' anche un fiducial (ID 7) dietro il punto di partenza.
Cio' che nessuno esercita: le trappole e le cadute dentro un tentativo (le prova
test_e2e_loop.py).
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

import test_e2e_loop as T          # mondo, robot e griglia locale finti; applica le sue patch
import bosdyn.client.lease
from bosdyn.api import world_object_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.client.math_helpers import Quat, SE3Pose

from spot_explorer.planning import safe_frontier as arcVerification
from spot_explorer.robot import battery as batteria
from spot_explorer.mission import cell_attempt, exploration
from spot_explorer.mission import run as mission_run
from spot_explorer.planning import goal
from spot_explorer.robot import motion as movements
from spot_explorer.graphnav import graphnav as navGraphUtils
from spot_explorer import config as parametri
from spot_explorer import records as registro
from spot_explorer.perception import grids as spotGrid
from spot_explorer.robot import connection as spotLogInUtils

CELL = 3.0
ROWS, COLS = 3, 3
START = (1.5, 1.5, 0.0)
_CHIUSA_22 = [(6.1, 9.0, 6.0, 6.2), (6.1, 6.3, 6.0, 9.0), (8.8, 9.0, 6.0, 9.0), (6.1, 9.0, 8.8, 9.0)]
WORLDS = {
    'A': [(5.9, 6.1, 0.0, 3.0)] + _CHIUSA_22 + [(4.2, 4.8, 4.2, 4.8)],
    'B': [(5.9, 6.1, 0.0, 4.6)] + _CHIUSA_22 + [(4.2, 4.8, 4.2, 4.8), (7.0, 7.4, 3.6, 5.4)],
}
WALLS = WORLDS['A']
# Fiducial del mondo in corso: (ID, x, y), None = nessuno. Nel mondo A un tag 1 m DIETRO il
# punto di partenza (lo vede la telecamera posteriore).
FIDUCIAL_TAG = None
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
        self.headings = []             # angoli d'arrivo chiesti (rispetto al waypoint)
        self.stuck_wp0_m = None        # ritorno a wp_0 fermo (STUCK) a questa distanza
        self.wp0_offset_m = None       # ritorno a wp_0 "arrivato" ma spostato di tanto (vedi run)
        self.wp0_returns = 0           # ritorni a wp_0 fatti
        self.relocated = False         # rilocalizzato sul tag DOPO un ritorno a wp_0
        self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL

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
        if (FIDUCIAL_TAG is not None and kw.get('use_fiducial_id') == FIDUCIAL_TAG[0]
                and kw.get('fiducial_init') == graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_SPECIFIC):
            if self.wp0_returns > 0:
                self.relocated = True             # da qui GraphNav sa davvero dov'e' wp_0
            return graph_nav_pb2.SetLocalizationResponse(status=graph_nav_pb2.SetLocalizationResponse.STATUS_OK)
        raise RuntimeError("STATUS_NO_MATCHING_FIDUCIAL (finto: nessun fiducial nel mondo)")

    def navigate_to(self, waypoint_id, cmd_duration, command_id=None, travel_params=None,
                    destination_waypoint_tform_body_goal=None, **kw):
        x, y, yaw = self.pos[waypoint_id]
        if destination_waypoint_tform_body_goal is not None:     # arrivo girato (solo l'angolo)
            yaw += destination_waypoint_tform_body_goal.angle
            self.headings.append(destination_waypoint_tform_body_goal.angle)
        self.trips.append((self._nearest(), waypoint_id))
        self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        name = next(w.annotations.name for w in self.graph.waypoints if w.id == waypoint_id)
        if name == 'wp_0' and self.stuck_wp0_m is not None:
            x += self.stuck_wp0_m                 # fermo prima, sulla strada da +x
            self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK
        if name == 'wp_0':
            self.wp0_returns += 1
            if self.wp0_offset_m is not None and not self.relocated:
                y += self.wp0_offset_m            # localizzazione sbagliata: "arrivato" di lato
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

    # --- elaborazione della mappa -------------------------------------------------------
    def process_topology(self, params=None, modify_map_on_server=False, **kw):
        return map_processing_pb2.ProcessTopologyResponse(
            status=map_processing_pb2.ProcessTopologyResponse.STATUS_OK)

    def process_anchoring(self, *a, **kw):
        r = map_processing_pb2.ProcessAnchoringResponse(
            status=map_processing_pb2.ProcessAnchoringResponse.STATUS_OK)
        r.iteration = 1
        return r


class FakeWorldObjects:
    """Il servizio degli oggetti visti: il tag FIDUCIAL_TAG, sempre in vista, o nessuno."""

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
    """Stato del robot con il solo odom -> corpo alla posa del robot finto (per
    initialize_with_fiducial: il resto del test vuole uno stato che fallisce)."""
    from bosdyn.api import robot_state_pb2
    st = robot_state_pb2.RobotState()
    edges = st.kinematic_state.transforms_snapshot.child_to_parent_edge_map
    edges['odom'].parent_frame_name = ''
    edges['odom'].parent_tform_child.rotation.w = 1.0
    body = SE3Pose(T.ROBOT.x, T.ROBOT.y, 0, Quat.from_yaw(T.ROBOT.yaw))
    edges['body'].parent_frame_name = 'odom'
    edges['body'].parent_tform_child.CopyFrom(body.to_proto())
    return st


_real_initialize_with_fiducial = navGraphUtils.RecordingInterface.initialize_with_fiducial


def _initialize_with_fiducial(self, robot_state_client, fiducial_id=None):
    return _real_initialize_with_fiducial(self, NS(get_robot_state=_robot_state_with_odom), fiducial_id)


navGraphUtils.RecordingInterface.initialize_with_fiducial = _initialize_with_fiducial


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
def _con_fallimenti():
    """Fallimenti forzati, sempre gli stessi: il 1o, 4o, 7o... tentativo e il 2o falliscono
    senza muovere il robot, una scelta dell'obiettivo su 5 non trova punti, il primo
    spostamento GraphNav finisce con una caduta (finta: il robot e' gia' in piedi)."""
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
def _con_batteria(da_pct=36.0, punti_a_lettura=1.0):
    """Batteria finta: ogni lettura (una per giro del ciclo, intervallo 0 s) un punto in
    meno, da da_pct."""
    real_read, real_s = batteria.leggi_batteria, parametri.BATTERY_SAMPLE_S
    n = {'letture': 0}

    def read(rsc):
        n['letture'] += 1
        return {'charge_pct': max(0.0, da_pct - punti_a_lettura * (n['letture'] - 1)),
                'spot_runtime_min': 40.0}

    batteria.leggi_batteria = read
    parametri.BATTERY_SAMPLE_S = 0.0
    try:
        yield n
    finally:
        batteria.leggi_batteria = real_read
        parametri.BATTERY_SAMPLE_S = real_s


def run(walls=None, name='MISSIONE', fiducial=None, stuck_wp0_m=None, wp0_offset_m=None):
    global FIDUCIAL_TAG
    FIDUCIAL_TAG = fiducial
    T.WORLD, T.ROBOT = T.World(WALLS if walls is None else walls, ghosts=True), T.Robot(*START)
    arcVerification.OBSTACLE_HISTORY.reset()
    gn = FakeGraphNav()
    gn.stuck_wp0_m = stuck_wp0_m
    gn.wp0_offset_m = wp0_offset_m
    robot = FakeRobot(gn)
    spotLogInUtils.setLogInfo = lambda o: (robot, NS(), robot.state, None)
    spotLogInUtils.SimpleEstop = lambda r, n: NS(stop=lambda: None, allow=lambda: None)
    bosdyn.client.lease.LeaseKeepAlive = lambda *a, **k: contextlib.nullcontext()
    mission_run.blocking_stand = lambda *a, **k: None
    spotGrid.LocalGrid = lambda r: T.FakeLocalGrid()
    arcVerification.ArcVerificationTracker = lambda r: _Tracker()
    cell_attempt.BodyTiltRecorder = registro.BodyTiltRecorder = _NoTilt
    parametri.MISSION_GRAPH_SAMPLING = 'reticolo'

    root = tempfile.mkdtemp(prefix="missione_finta_")
    mission_run.DATA_ROOT = root                                    # cartelle dei dati qui
    argv = sys.argv
    sys.argv = ["easy_walk.py", "--rows", str(ROWS), "--cols", str(COLS), "--cell", str(CELL),
                "--grafo", "reticolo", "--spacing", "0.5"]
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


def _controlli_comuni(r, ritorno=True):
    log = r['log']
    print(f"  ({r['seconds']:.0f} s)")
    check(r['ok'] is True, "main() arriva in fondo senza eccezioni")
    check('Traceback' not in log and 'Traceback' not in r['out'], "nessuna eccezione nel log")
    if ritorno:
        check("ritorno a wp_0: arrivato" in log, "ritorno a wp_0 riuscito")
    check("[FINAL SCAN] Fatto" in log, "scansione finale fatta")
    check("[ANCHORING]" in log, "ancoraggio finale chiamato")


def _controlli_mondo(name, r):
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
    righe = [l for l in log.splitlines() if '[PROGRESSO]' in l and 'Fine' not in l]
    chiuse = [int(l.split('[PROGRESSO] ')[1].split('/')[0]) for l in righe]
    check(chiuse == sorted(set(chiuse)), f"una riga [PROGRESSO] per ogni cella chiusa in piu': {chiuse}")
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


def _controlli_fallimenti(r):
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


def _controlli_emergenza():
    """Missione interrotta a meta' (eccezione o Ctrl+C): il robot si siede, i motori si spengono e
    la mappa GraphNav si scarica, poi l'errore risale come prima."""
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
                risalita = r['ok'] is False
            except KeyboardInterrupt:
                risalita, r = True, None
        finally:
            cell_attempt.attempt_enter_cell_from_position = real
        if r is None:      # Ctrl+C: il log e' nella cartella della missione
            logs = sorted(glob.glob(os.path.join(tempfile.gettempdir(), "missione_finta_*", "MissionLogs",
                                                 "*", "mission_log.txt")), key=os.path.getmtime)
            log = open(logs[-1]).read() if logs else ""
        else:
            log = r['log']
        check(risalita, f"{nome}: la missione finisce con l'errore, come prima")
        check("[EMERGENZA] Robot seduto e motori spenti." in log, f"{nome}: robot seduto e motori spenti")
        check("[EMERGENZA] Mappa GraphNav scaricata." in log, f"{nome}: mappa GraphNav scaricata")


def _controlli_scelta_celle():
    """
    Il caso del 28-10 13:50, griglia 8 x 7 da 2 m: la (7,2) e' stata visitata con il robot
    appena fuori, nella (6,3), e la (7,1) era gia' stata provata dal lato della (6,1). Il robot
    deve cercare le confinanti dalla (7,2), e avvicinarsi alla (7,1) dalla (7,2), non di nuovo
    dalla (6,1) (prima: salto, poi la cella usciva dalla frontiera).
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
    m = exploration.Missione.__new__(exploration.Missione)
    m.env, m.recordingInterface, m.robot_state_client = env, ri, None
    m._ultima_visitata = (7, 2)
    T.ROBOT.x, T.ROBOT.y = 4.0, 12.75                 # 0.25 m fuori dalla (7,2), nella (6,2)
    with contextlib.redirect_stdout(io.StringIO()):
        cella = m._cella_del_robot()
        partenza = m._cella_di_partenza_per(7, 1, wps)
    check(cella == (7, 2), f"robot appena fuori dalla (7,2) appena visitata: cella del robot {cella}")
    check(partenza == (7, 2), f"avvicinamento alla (7,1) dal lato non ancora provato: dalla {partenza}")
    T.ROBOT.x, T.ROBOT.y = 4.0, 11.5                  # ben dentro la (6,2): vale la cella vera
    with contextlib.redirect_stdout(io.StringIO()):
        cella = m._cella_del_robot()
    check(cella == (6, 2), f"robot lontano dalla cella visitata: vale quella in cui sta ({cella})")


def _controlli_batteria(r):
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
        _controlli_comuni(r)
        _controlli_mondo(name, r)
        shutil.rmtree(r['root'], ignore_errors=True)
    print("\nMissione intera nel mondo C: il B con fallimenti forzati.")
    with _con_fallimenti():
        r = run(WORLDS['B'], "MISSIONE_C", stuck_wp0_m=0.6)
    _controlli_comuni(r, ritorno=False)
    _controlli_fallimenti(r)
    shutil.rmtree(r['root'], ignore_errors=True)
    _controlli_scelta_celle()
    _controlli_emergenza()
    print("\nMissione intera nel mondo G: il mondo A con il ritorno a wp_0 'arrivato' ma 0.8 m di lato "
          "(29-10 10:57: 1.2 m).")
    r = run(WORLDS['A'], "MISSIONE_G", fiducial=(7, START[0] - 1.0, START[1]), wp0_offset_m=0.8)
    _controlli_comuni(r)
    log = r['log']
    check("[WP0] Secondo il tag il robot e' a 0.80 m dal punto di partenza" in log,
          "il tag misura lo scarto vero dalla partenza (0.80 m)")
    check("[WP0] CORREZIONE 1/2" in log and "correzione 1 del ritorno a wp_0: arrivato" in log,
          "rilocalizzazione sul tag e nuovo ritorno a wp_0")
    check("[WP0] Dopo la correzione 1: a 0.00 m dalla partenza (prima 0.80 m)" in log
          and "CORREZIONE 2/2" not in log, "una correzione basta: 0.00 m dalla partenza")
    shutil.rmtree(r['root'], ignore_errors=True)
    print("\nMissione intera nel mondo E: partenza chiusa tutto intorno (28-10 14:17, erba alta).")
    chiuso = [(0.4, 2.6, 0.4, 0.6), (0.4, 2.6, 2.4, 2.6), (0.4, 0.6, 0.4, 2.6), (2.4, 2.6, 0.4, 2.6)]
    r = run(chiuso, "MISSIONE_E")
    log = r['log']
    check(r['ok'] is True and 'Traceback' not in log, "main() arriva in fondo senza eccezioni")
    check("[FINE] Nessuna cella raggiungibile dalla partenza" in log,
          "la riga [FINE] dice che dalla partenza non si raggiunge nessuna cella")
    check("ritorno a wp_0" in log, "chiusura normale della missione")
    shutil.rmtree(r['root'], ignore_errors=True)
    print("\nMissione intera nel mondo D: il mondo A con la batteria che si scarica.")
    with _con_batteria():
        r = run(WORLDS['A'], "MISSIONE_D")
    _controlli_comuni(r)
    _controlli_batteria(r)
    shutil.rmtree(r['root'], ignore_errors=True)
    print("\n" + "=" * 70)
    print("TUTTO OK" if not FAILS else f"{len(FAILS)} CONTROLLI FALLITI:\n  - " + "\n  - ".join(FAILS))
    sys.exit(1 if FAILS else 0)
