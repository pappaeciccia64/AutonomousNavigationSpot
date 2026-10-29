"""
Prova end-to-end del ciclo di navigazione VERO (attempt_enter_cell_from_position in
easy_walk.py) contro un robot finto in un mondo sintetico.

Cosa e' finto: la posizione del robot, la griglia locale (finestra 128x128 da 3 cm che
segue il robot, con obstacle_distance calcolata dal mondo e celle fantasma negli angoli
come nei dati veri), il movimento (il robot si sposta lungo la retta, interrogando
should_abort a ogni tratto), il thread del fronte (calcola il fronte vero sulla posizione
del momento). Tutto il resto e' il codice vero: correzione terreno, mappe globali, PRM,
nodi di sosta, fronte, decisioni, blocchi, ripianificazioni, ritirata, salvataggio dati.

    python3 test_e2e_loop.py
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
STUBS = tempfile.mkdtemp(prefix="stub_moduli_")
for name in ("navGraphUtils", "spotLogInUtils", "environmentMap"):
    open(os.path.join(STUBS, name + ".py"), "w").write("# vuoto: usato solo a runtime sul robot\n")
sys.path.insert(0, HERE)
sys.path.insert(1, STUBS)

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import easy_walk     # noqa: E402
import parametri     # noqa: E402
import registro      # noqa: E402
import spotGrid      # noqa: E402
import arcVerification as av   # noqa: E402
import prm_graph     # noqa: E402
import movements     # noqa: E402
import spotUtils     # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


CS, N, W = 0.03, 128, 400          # mondo 12 x 12 m


class World:
    def __init__(self, rects, ghosts=True):
        self.occ = np.zeros((W, W), bool)
        wx = (np.arange(W) + 0.5) * CS
        WX, WY = np.meshgrid(wx, wx)
        for x0, x1, y0, y1 in rects:
            self.occ |= (WX >= x0) & (WX <= x1) & (WY >= y0) & (WY <= y1)
        self.dynamic = None          # rettangolo che compare durante un movimento
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
        # Distanza CON SEGNO dentro gli ostacoli, come il layer vero di Spot (missione 08-10
        # 16:35: il 22% delle celle negativo, fino a oltre -0.5 m). Con un valore fisso il
        # controllo di rotazione non vede quanto il corpo entra nel muro.
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
        self.commands = 0          # quanti comandi di movimento sono stati dati
        self.walked = 0.0          # metri percorsi
        self.turns = 0             # rotazioni oltre la soglia di trascurabilita' (10 gradi)
        self.turn_deg = 0.0        # gradi totali ruotati
        self.max_endpoint_error = 0.0   # massimo scarto fra punto comandato e punto raggiunto


WORLD, ROBOT = None, None
# Parametri in piu' per build_mission_graph negli scenari: vuoto = il reticolo.
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
    if b == easy_walk.BODY_FRAME_NAME:
        x, y, yaw = snapshot.robot
        return NS(position=NS(x=x, y=y, z=-2.7), rotation=quat(yaw))
    ox, oy = snapshot.origin
    return NS(position=NS(x=ox, y=oy, z=0.0), rotation=quat(0.0))


class FakeLocalGrid(spotGrid.LocalGrid):
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
        # Come spotGrid.LocalGrid.extra_layers: un layer finto per controllare il salvataggio;
        # 'no_step' manca apposta (un layer assente non deve fermare niente).
        if 'intensity' not in names:
            return {}
        return {'intensity': dict(values=np.full((N, N), 7.0, np.float32),
                                  origin=np.array([ROBOT.x - N * CS / 2, ROBOT.y - N * CS / 2]),
                                  cell=CS)}


class FakeTracker:
    """Il thread del fronte: qui calcola il fronte VERO nella posizione del momento."""
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
        snap = av.make_grid_snapshot(od, rough, iv, valid, tc, N, N, c0 * CS, r0 * CS, CS, ROBOT.x, ROBOT.y)
        f = av.compute_safe_frontier([(ROBOT.x, ROBOT.y)] + self.waypoints, snap, body=self.body)
        f['path_version'] = self.version
        f['time'] = time.time()
        return f


DYNAMIC_TRIGGER = {'armed': False, 'rect': None}
FALL = {'armed': False, 'fallen': False, 'recoverable': True, 'where': None}
# Comando che non si conclude (tempo scaduto): i prossimi 'n' tratti piu' lunghi di 0.3 m si
# fermano 'short_m' prima del punto e restituiscono fallimento (scenario S17).
STALL = {'n': 0, 'short_m': 0.0}


def fake_check_fall(rsc):
    return dict(fallen=FALL['fallen'], reason="fault di caduta (prova)" if FALL['fallen'] else "",
                roll=80.0 if FALL['fallen'] else 0.0, pitch=0.0, x=ROBOT.x, y=ROBOT.y,
                fall_faults=[1] if FALL['fallen'] else [], clearable=[1] if FALL['fallen'] else [],
                unclearable=[], motors_on=True, standing=not FALL['fallen'])


def fake_recover(cc, rsc, status=None):
    if not FALL['fallen']:
        return True
    if not FALL['recoverable']:
        raise movements.RobotFallenError("prova: il robot non si rialza da solo")
    FALL['fallen'] = False
    return True


def _yaw_of(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))


def fake_navigate_to(tx, ty, rx, ry, rsc, cc, vtb, should_abort=None, keep_heading=False, **kw):
    """
    FEDELE al navigate_to vero, per accorgersi di un orientamento passato in ritardo. Il vero
    navigate_to:
      1. calcola dyaw = direzione del bersaglio - orientamento letto da `vtb`;
      2. relative_move(0, 0, dyaw): ruota di dyaw RISPETTO ALLA POSA VERA del robot;
      3. relative_move(dist, 0, 0): cammina dritto per `dist`, lungo il muso VERO, con
         dist calcolata da (rx, ry) passati, non dalla posizione vera.
    Se `vtb` o (rx, ry) sono vecchi, il robot va da un'altra parte: ed e' esattamente
    quello che questo finto ora riproduce.
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
        if abs(np.degrees(dyaw)) >= spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
            ROBOT.turns += 1
        ROBOT.yaw = float(np.arctan2(np.sin(ROBOT.yaw + dyaw), np.cos(ROBOT.yaw + dyaw)))
        sx, sy = ROBOT.x, ROBOT.y
        ex, ey = sx + dist * np.cos(ROBOT.yaw), sy + dist * np.sin(ROBOT.yaw)
    stalled = STALL['n'] > 0 and dist > 0.3
    if stalled:
        STALL['n'] -= 1
        f = (dist - STALL['short_m']) / dist
        ex, ey = sx + f * (ex - sx), sy + f * (ey - sy)
    # Quanto il punto d'arrivo vero si discosta da quello comandato: e' il sintomo.
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
            # 0.85 m davanti al CENTRO del robot: fuori dal mezzo corpo (0.55) piu' il
            # margine del fronte (0.10), quindi fermarsi in tempo e' possibile.
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
        # (0,0) visitata: e' la cella da cui il robot parte in tutti gli scenari.
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


# ---- patch: solo i confini col robot ------------------------------------------------------
easy_walk.get_a_tform_b = spotGrid.get_a_tform_b = fake_get_a_tform_b
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


easy_walk.BodyTiltRecorder = registro.BodyTiltRecorder = _NoTilt


def run(name, rects, start, target_cell, ghosts=True, dynamic=None, fall=None, second_attempt=False,
        memory=None):
    global WORLD, ROBOT
    WORLD, ROBOT = World(rects, ghosts), Robot(*start)
    # Il filtro temporale degli ostacoli (arcVerification.OBSTACLE_HISTORY) e' allineato al
    # MONDO e sopravvive fra una scansione e l'altra: sul robot e' giusto, il mondo e'
    # sempre quello. Qui ogni scenario e' un mondo diverso con le stesse coordinate, e
    # senza azzerarlo la mediana mescolava le scansioni dello scenario precedente con
    # quelle nuove: S11 passava o falliva a seconda di cosa girava prima.
    av.OBSTACLE_HISTORY.reset()
    DYNAMIC_TRIGGER.update(armed=dynamic is not None, rect=dynamic)
    FALL.update(armed=fall is not None, fallen=False, recoverable=(fall == 'recuperabile'), where=None)
    easy_walk.MISSION_STATS['falls'] = easy_walk.MISSION_STATS['self_rights'] = 0
    # Il reticolo vero della missione: passo 0.25 m, archi 0.5-1.25 m, sulle 2 x 2 celle da
    # 5 m di FakeEnv, per tutti gli scenari. GRAPH_KW sceglie i nodi casuali (confronto in fondo).
    import mission_graph
    mg = mission_graph.build_mission_graph(
        rows=2, cols=2, cell_size_m=5.0, spacing_m=0.25, rings=None,
        origin_xy=(3.5, 3.5), origin_yaw=0.0, min_edge_length=parametri.PRM_MIN_EDGE_M,
        max_edge_length=parametri.PRM_MAX_EDGE_M,
        connection_radius=parametri.PRM_CONNECTION_RADIUS_M, verbose=False,
        **dict({'sampling': 'reticolo'}, **GRAPH_KW))
    prm = prm_graph.PRM(min_edge_length=parametri.PRM_MIN_EDGE_M,
                        max_edge_length=parametri.PRM_MAX_EDGE_M,
                        connection_radius=parametri.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg
    mg.into_prm(prm)
    folder = tempfile.mkdtemp(prefix="missione_e2e_")
    for k in list(easy_walk.MISSION_STATS['time_s']):
        easy_walk.MISSION_STATS['time_s'][k] = 0.0
    for k in ('frontier_blocks', 'frontier_aborts', 'scan_bundles', 'prm_refresh',
              'prm_refresh_skipped', 'multinode_moves', 'multinode_nodes', 'idle_loops'):
        if k in easy_walk.MISSION_STATS:
            easy_walk.MISSION_STATS[k] = 0
    gg = NS(global_occupancy_map=None, global_terrain_map=None)
    if memory:
        # Ostacoli gia' nella mappa globale da osservazioni precedenti (in missione: le celle
        # gia' visitate). Occupate le celle con obstacle_distance <= OBSTACLE_THRESHOLD, come
        # le scrive fold_scan_into_global_maps.
        gm0 = spotGrid.GlobalGrid(resolution=CS)
        t = spotGrid.OBSTACLE_THRESHOLD
        for x0, x1, y0, y1 in memory:
            for gx in range(int(np.floor((x0 - t) / CS)), int(np.ceil((x1 + t) / CS))):
                for gy in range(int(np.floor((y0 - t) / CS)), int(np.ceil((y1 + t) / CS))):
                    gm0.obs_count[(gx, gy)] = spotGrid.GLOBAL_OCC_MAX_COUNT
                    gm0._set_state((gx, gy), -1.0)
        gg.global_occupancy_map = gm0
    out = io.StringIO()
    err = None
    t0 = time.time()
    with contextlib.redirect_stdout(out):
        try:
            ok = easy_walk.attempt_enter_cell_from_position(
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
            ok2 = easy_walk.attempt_enter_cell_from_position(
                FakeLocalGrid(), gg, None, None, FakeEnv(), target_cell[0], target_cell[1],
                FakeSampler(), prm, mission_folder=folder, iteration=1,
                recordingInterface=None, verification_tracker=FakeTracker())
    registro.VIS_EXECUTOR.submit(lambda: None).result()      # aspetta i salvataggi
    # Il codice vero restituisce env.is_point_in_cell(...), che con coordinate numpy e' un
    # np.bool_: il chiamante fa `if check:` e funziona, ma qui i controlli usano `is True`.
    if isinstance(ok, np.bool_):
        ok = bool(ok)
    if isinstance(ok2, np.bool_):
        ok2 = bool(ok2)
    log = out.getvalue()
    # Registrazione di riferimento (E2E_LOG_DIR=cartella): il log completo di ogni scenario,
    # per confrontare il comportamento riga per riga prima e dopo una ristrutturazione del
    # codice (vedi confronta_log.py).
    if os.environ.get('E2E_LOG_DIR'):
        os.makedirs(os.environ['E2E_LOG_DIR'], exist_ok=True)
        slug = "".join(c if c.isalnum() else "_" for c in name)[:60]
        with open(os.path.join(os.environ['E2E_LOG_DIR'], slug + ".log"), "w") as fh:
            fh.write(log)
    n_scans = len(os.listdir(os.path.join(folder, "scans"))) if os.path.isdir(os.path.join(folder, "scans")) else 0
    print(f"\n{name}  ({time.time() - t0:.1f} s)")
    if err:
        print(err)
    tags = {t: log.count(t) for t in ("[CADUTA] Rilevata", "[FRONTE]", "[PASSO PARZIALE]", "[OK] Tratto",
                                      "[BLOCCO] Confermato", "[SCORCIATOIA]", "[MULTINODO]",
                                      "[REPLAN] Dopo il blocco", "[STOP]", "[NODO]", "[RITIRATA]",
                                      "[ROTAZIONE]", "[GROUND-FILTER]", "[OBIETTIVO]", "[STALLO]",
                                      "[SUBITO]", "[TRAPPOLA]", "[ARRIVO]", "[CONO]")}
    print("       " + "  ".join(f"{k} x{v}" for k, v in tags.items() if v))
    print(f"       scarto massimo fra punto comandato e raggiunto: {ROBOT.max_endpoint_error:.3f} m")
    print(f"       comandi di movimento: {ROBOT.commands}, metri percorsi: {ROBOT.walked:.1f}, "
          f"scansioni: {n_scans}, refresh {easy_walk.MISSION_STATS.get('prm_refresh', 0)} fatti / "
          f"{easy_walk.MISSION_STATS.get('prm_refresh_skipped', 0)} saltati")
    return dict(ok=ok, err=err, log=log, scans=n_scans, folder=folder, tags=tags,
                min_clear=ROBOT.min_clear, final=(ROBOT.x, ROBOT.y), ok2=ok2, gg=gg,
                commands=ROBOT.commands, walked=ROBOT.walked,
                turns=ROBOT.turns, turn_deg=ROBOT.turn_deg,
                endpoint_error=ROBOT.max_endpoint_error,
                stats=dict(easy_walk.MISSION_STATS),
                track2=ROBOT.track[track2_from:] if track2_from is not None else [])


if __name__ == '__main__':
    print("Robot finto, codice vero. Cella obiettivo (0,1): centro (8.5, 3.5).")
    print("NB: la cella obiettivo deve essere ADIACENTE a una visitata -- dal 2026-10-08\n    la pianificazione e' limitata alle celle esplorate (PLAN_ONLY_EXPLORED_CELLS).")

    r = run("S1. campo libero, celle fantasma negli angoli di ogni finestra (2026-10-05)",
            [], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "nessuna eccezione")
    check(r['ok'] is True, f"entrato nella cella, posizione finale ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['tags']["[BLOCCO] Confermato"] == 0 and r['tags']["[RITIRATA]"] == 0,
          "nessun blocco e nessuna ritirata su terreno libero")
    check(r['scans'] > 0, f"{r['scans']} pacchetti di scansione salvati")
    _b = np.load(os.path.join(r['folder'], 'scans', sorted(os.listdir(os.path.join(r['folder'], 'scans')))[0]))
    check('intensity' in _b.files and _b['intensity'].shape == (N, N) and 'intensity_origin' in _b.files
          and 'no_step' not in _b.files,
          "layer in piu' nel pacchetto: intensity salvata con la sua origine, no_step assente senza danni")
    check("[TEMPO]" in r['log'], "riga [TEMPO] presente")

    r = run("S2. muro con un varco largo 1.2 m lontano dalla retta: va trovato aggirando",
            [(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "nessuna eccezione")
    check(r['ok'] is True, f"entrato nella cella passando dal varco ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['min_clear'] >= spotGrid.FRONTIER_CLEARANCE_M - 0.05,
          f"distanza minima del centro dagli ostacoli lungo tutto il tragitto: {r['min_clear']:.2f} m")

    r = run("S3. una persona si mette davanti al robot mentre cammina",
            [], (2.0, 2.0, 0.785), (0, 1), dynamic=True)
    check(r['err'] is None, "nessuna eccezione")
    check(r['min_clear'] > 0.0,
          f"non tocca mai l'ostacolo apparso durante il movimento "
          f"(distanza minima del centro {r['min_clear']:.2f} m)")
    check(r['min_clear'] >= spotGrid.FRONTIER_CLEARANCE_M - 0.10,
          f"e resta oltre la distanza minima richiesta dal fronte "
          f"({spotGrid.FRONTIER_CLEARANCE_M:.2f} m): misurata {r['min_clear']:.2f} m")
    print(f"       [STOP] x{r['tags']['[STOP]']}, [MULTINODO] x{r['tags']['[MULTINODO]']}")

    r = run("S4. cella obiettivo chiusa da muri su tutti i lati",
            [(6.1, 6.3, 1.0, 6.0), (6.1, 10.9, 1.0, 1.2), (10.7, 10.9, 1.0, 6.0), (6.1, 10.9, 5.8, 6.0)],
            (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None, "nessuna eccezione")
    check(r['ok'] is False, "tentativo fallito in modo ordinato (cella irraggiungibile)")
    check(r['min_clear'] > 0.0, f"non entra mai nel muro (distanza minima {r['min_clear']:.2f} m)")
    # Il secondo [SUBITO] sullo stesso tratto lo toglie dal piano per il resto del tentativo
    # (28-10 14:23: lo stesso tratto scartato quattro volte di fila).
    tratti = re.findall(r"\[SUBITO\] \w+ in \([^)]*\) sul tratto (\d+->\d+)", r['log'])
    check("fuori dal piano per il resto del tentativo" in r['log'] and max(
          [tratti.count(t) for t in tratti] or [0]) <= 2,
          f"[SUBITO] ripetuto: alla seconda volta il tratto esce dal piano ({tratti})")

    r = run("S5. partenza in uno spazio stretto: 0.45 m dal muro, bersaglio a 90 gradi",
            [(2.45, 2.65, 0.0, 6.0)], (2.0, 3.0, 1.5708), (0, 0))
    check(r['err'] is None, "nessuna eccezione")

    r = run("S6. caduta a meta' movimento, il robot si rialza; poi un secondo tentativo",
            [], (2.0, 2.0, 0.785), (0, 1), fall='recuperabile', second_attempt=True)
    check(r['err'] is None, "nessuna eccezione")
    check(r['tags']["[CADUTA] Rilevata"] == 1 and easy_walk.MISSION_STATS['self_rights'] == 1,
          "caduta rilevata e robot rialzato")
    fx, fy = FALL['where']
    gm = r['gg'].global_occupancy_map
    check(gm is not None and gm.is_occupied(fx, fy),
          f"punto della caduta ({fx:.2f}, {fy:.2f}) occupato nella mappa globale")

    r = run("S7. caduta da cui il robot NON si rialza da solo",
            [], (2.0, 2.0, 0.785), (0, 1), fall='irrecuperabile')
    check(r['err'] is None and isinstance(r['ok'], str) and r['ok'].startswith("RobotFallenError"),
          "la missione si ferma con RobotFallenError (main() la intercetta e chiama l'operatore)")
    check("[RITIRATA]" not in r['log'], "nessun tentativo di muoversi da terra")

    r = run("S8. reticolo VERO (passo 0.25, archi 0.5-1.25) su campo libero",
            [], (3.5, 3.5, 0.0), (0, 1))
    check(r['err'] is None, "nessuna eccezione con il grafo di missione vero")
    check(r['ok'] is True, f"entrato nella cella ({r['final'][0]:.2f}, {r['final'][1]:.2f})")
    check(r['walked'] > 0 and r['commands'] > 0,
          f"{r['commands']} comandi per {r['walked']:.1f} m = {r['walked'] / max(1, r['commands']):.2f} m per comando")

    r = run("S10. reticolo VERO + muro con varco: il percorso CURVA (qui serve il multinodo)",
            [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)], (3.5, 3.5, 0.0), (0, 1))
    check(r['err'] is None, "nessuna eccezione")
    check(r['min_clear'] >= spotGrid.FRONTIER_CLEARANCE_M - 0.10,
          f"non sfiora il muro passando dal varco (distanza minima {r['min_clear']:.2f} m)")
    print(f"       [MULTINODO] x{r['tags']['[MULTINODO]']}, [SCORCIATOIA] x{r['tags']['[SCORCIATOIA]']}, "
          f"nodi attraversati dai movimenti multinodo: {r['stats'].get('multinode_nodes', 0)}")

    # Ricostruzione della situazione della missione 08-10 16:35: corridoio stretto e
    # obiettivo DIETRO il robot, cioe' rotazione di ~180 gradi dove non si puo' girare. La'
    # il ciclo fece 8 giri in 3.2 s senza muoversi. Il robot deve partire FUORI dalla cella
    # obiettivo, altrimenti lo scenario passa senza provare niente.
    r = run("S11. corridoio da 0.80 m, obiettivo dietro: rotazione impossibile (caso 16:35)",
            [(0.0, 12.0, 0.0, 3.10), (0.0, 12.0, 3.90, 12.0)], (7.0, 3.5, 0.0), (0, 0))
    check(r['err'] is None, "nessuna eccezione")
    giri = r['log'].count('[FRONTE]')
    rifiuti = r['log'].count('[ROTAZIONE] Servono')
    check(rifiuti >= 1, f"la rotazione di ~180 gradi nel corridoio viene rifiutata ({rifiuti} volte)")
    check(r['turns'] == 0, f"il robot non ruota mai nel corridoio (rotazioni eseguite: {r['turns']})")
    check(r['ok'] is False and giri <= 8,
          f"il tentativo si chiude in modo ordinato e in pochi giri: {giri} giri, esito {r['ok']} "
          f"(il 16:35 ne fece 8 in 3.2 s, e senza limite poteva continuare)")
    print(f"       [STALLO] x{r['tags']['[STALLO]']}, "
          f"limite rifiuti-rotazione x{r['log'].count('tratti scartati in questo tentativo')}, "
          f"'nessuna alternativa nel grafo' x{r['log'].count('Nessuna alternativa nel grafo dal punto')}, "
          f"[TRAPPOLA] x{r['tags']['[TRAPPOLA]']}, [CONO] x{r['tags']['[CONO]']}")

    # S12: l'avanzamento multinodo deve passare a ogni gamba
    # l'orientamento VERO del robot. Con archi corti il multinodo scatta, e con un percorso
    # che curva un orientamento vecchio di una gamba manda il robot fuori dal percorso.
    parametri.PRM_MAX_EDGE_M = 0.70711
    r = run("S12. multinodo su percorso che curva: ogni gamba va dove e' stata comandata",
            [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)], (3.5, 3.5, 0.0), (0, 1))
    parametri.PRM_MAX_EDGE_M = 1.25
    check(r['err'] is None, "nessuna eccezione")
    check(r['stats'].get('multinode_moves', 0) >= 1,
          f"il multinodo e' scattato ({r['stats'].get('multinode_moves', 0)} movimenti, "
          f"{r['stats'].get('multinode_nodes', 0)} nodi)")
    check(r['endpoint_error'] < 0.02,
          f"ogni gamba arriva dove e' stata comandata: scarto massimo {r['endpoint_error']:.3f} m")
    check(r['min_clear'] >= spotGrid.FRONTIER_CLEARANCE_M - 0.10,
          f"non sfiora il muro (distanza minima {r['min_clear']:.2f} m)")

    # S14: della cella obiettivo e' libera solo una striscia lungo il bordo vicino al robot; il
    # resto e' gia' noto occupato (memory: con celle da 5 m il centro e' fuori dalla finestra
    # di scansione, e senza memoria resterebbe ignoto e verrebbe scelto). Davanti
    # all'ostacolo il fronte ferma il centro mezzo corpo prima, appena fuori dalla cella:
    # deve valere come arrivo (ARRIVAL_OUTSIDE_CELL_M). Il 22-10 16:04 lo stesso caso dava
    # "Mancano 0.06 m ... ancora fuori dalla cella" fino al blocco.
    r = run("S14. cella occupata salvo una striscia lungo il bordo: arrivo appena fuori",
            [(6.75, 12.0, 0.0, 12.0)], (5.0, 3.5, 0.0), (0, 1), memory=[(6.75, 12.0, 0.0, 12.0)])
    check(r['err'] is None, "nessuna eccezione")
    tgt = [l for l in r['log'].splitlines() if l.startswith('[TARGET] Cella (0,1)')]
    check(bool(tgt) and 'uso il centro' not in tgt[0],
          f"obiettivo nella striscia libera: {tgt[0][:120] if tgt else 'nessuna riga [TARGET]'}")
    check('GRAFO DISCONNESSO' not in r['log'], "il grafo arriva all'obiettivo scelto")
    check(r['ok'] is True, f"la cella vale come raggiunta (posizione finale "
                           f"({r['final'][0]:.2f}, {r['final'][1]:.2f}), bordo a x = 6.00)")
    check(r['final'][0] >= 6.0 - parametri.ARRIVAL_OUTSIDE_CELL_M - 0.02,
          f"il robot e' fuori dalla cella al massimo di {parametri.ARRIVAL_OUTSIDE_CELL_M:.2f} m "
          f"({max(0.0, 6.0 - r['final'][0]):.2f} m)")
    check(r['min_clear'] > 0.0, f"non entra nel muro (distanza minima {r['min_clear']:.2f} m)")

    print("\n--- Confronto: quanto cammina il robot per ogni comando di movimento ---")
    confronto = {}
    for etichetta, maxe in (("archi 0.5-0.707 (missione 16:35)", 0.70711),
                            ("archi 0.5-1.25 (nuovo)", 1.25)):
        parametri.PRM_MAX_EDGE_M = maxe
        r = run(f"S9. {etichetta}", [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)],
                (3.5, 3.5, 0.0), (0, 1))
        confronto[etichetta] = (r['commands'], r['walked'], r['stats'].get('multinode_moves', 0),
                                r['stats'].get('multinode_nodes', 0), r['scans'], r['ok'],
                                r['turns'], r['turn_deg'])
    parametri.PRM_MAX_EDGE_M = 1.25
    print()
    for k, (c, w, mm, mn, sc, ok, tn, td) in confronto.items():
        print(f"  {k:<34} {c:2d} comandi, {w:4.1f} m -> {w / max(1, c):.2f} m/comando; "
              f"rotazioni oltre 10 gradi: {tn} ({td:.0f} gradi in tutto); "
              f"multinodo {mm} ({mn} nodi); {sc} scansioni; arrivato: {ok}")

    print("\n--- Confronto: reticolo e nodi casuali (16 nodi/m2, seme 7), corridoio di S9 ---")
    for etichetta, kw in [("reticolo", {})] + [
            (f"casuale {d}", dict(sampling='casuale', distribution=d, density_per_m2=16.0, seed=7))
            for d in ("uniforme", "poisson", "perturbata", "halton")]:
        GRAPH_KW.clear()
        GRAPH_KW.update(kw)
        r = run(f"S13. {etichetta}", [(5.0, 5.2, 0.0, 5.5), (5.0, 5.2, 6.9, 12.0)],
                (3.5, 3.5, 0.0), (0, 1))
        check(r['err'] is None and r['ok'], f"{etichetta}: arrivato nella cella senza eccezioni")
        check(r['min_clear'] >= spotGrid.FRONTIER_CLEARANCE_M - 0.10,
              f"{etichetta}: non sfiora il muro (distanza minima {r['min_clear']:.2f} m)")
        print(f"       {r['commands']} comandi, {r['walked']:.1f} m, rotazioni oltre 10 gradi "
              f"{r['turns']} ({r['turn_deg']:.0f} gradi), {r['scans']} scansioni")
    GRAPH_KW.clear()

    # S15: un ostacolo subito OLTRE l'obiettivo (28-10 11:36, cella (0,2)). Il fronte controlla
    # il percorso prolungato di mezzo corpo per il muso, e si ferma sull'ostacolo: nessun
    # percorso verso quell'obiettivo puo' passare. Prima si scartavano i tratti uno per uno
    # ([SUBITO], fino a 8) fino al blocco, e su questi stessi casi il codice del 28-10 mattina
    # falliva la cella 7 volte su 24 (seme 9: 143 comandi). Nodi casuali come in missione.
    obstacle_beyond = [(8.75, 9.15, 2.0, 5.0)]
    for etichetta, seed, x0, attesa in (("passo finale sotto il passo minimo", 4, 8.85, 'passo'),
                                        ("nuovo obiettivo alla prima fermata", 9, 8.75, 'attesa')):
        GRAPH_KW.clear()
        GRAPH_KW.update(sampling='casuale', distribution='poisson', density_per_m2=4.0, seed=seed)
        r = run(f"S15. ostacolo subito oltre l'obiettivo: {etichetta}",
                [(x0, x0 + 0.4, 2.0, 5.0)], (2.0, 3.0, 0.0), (0, 1))
        check(r['err'] is None and r['ok'] is True, f"{etichetta}: cella raggiunta senza eccezioni")
        check(r['tags']['[SUBITO]'] == 0 and r['tags']['[BLOCCO] Confermato'] == 0,
              f"{etichetta}: nessun [SUBITO] ne' blocco ({r['tags']['[SUBITO]']}, "
              f"{r['tags']['[BLOCCO] Confermato']})")
        if attesa == 'passo':
            check('[PASSO FINALE]' in r['log'], "il passo sotto 0.40 m che basta per arrivare si fa")
        else:
            check("[TARGET] Il fronte si ferma" in r['log'] and "oltre la fine del percorso" in r['log'],
                  "il fronte fermo oltre l'obiettivo fa scegliere subito un altro obiettivo")
        check(r['scans'] <= 9, f"{etichetta}: {r['scans']} scansioni")

    # Lo stesso caso con la regola dell'attesa tolta: il blocco confermato a ridosso
    # dell'obiettivo deve far scegliere un altro obiettivo, non ripianificare verso quello
    # appena escluso (28-10 11:36: "escluso", e subito dopo un percorso verso lo stesso nodo).
    real_check = easy_walk.TentativoCella._fronte_fermo_sull_obiettivo

    def _solo_nel_blocco(self):
        return sys._getframe(1).f_code.co_name != '_cambia_obiettivo_irraggiungibile' and real_check(self)
    easy_walk.TentativoCella._fronte_fermo_sull_obiettivo = _solo_nel_blocco
    parametri_early = parametri.MAX_EARLY_REPLANS
    parametri.MAX_EARLY_REPLANS = 0
    try:
        GRAPH_KW.clear()
        GRAPH_KW.update(sampling='casuale', distribution='poisson', density_per_m2=4.0, seed=9)
        r = run("S15. ostacolo subito oltre l'obiettivo: blocco, poi nuovo obiettivo",
                obstacle_beyond, (2.0, 3.0, 0.0), (0, 1))
    finally:
        easy_walk.TentativoCella._fronte_fermo_sull_obiettivo = real_check
        parametri.MAX_EARLY_REPLANS = parametri_early
    check(r['err'] is None and r['ok'] is True, "blocco vicino all'obiettivo: cella raggiunta")
    check("[TARGET] Blocco a" in r['log'] and "[REPLAN] Dopo il blocco" not in r['log'],
          "dopo il blocco a ridosso dell'obiettivo se ne sceglie un altro, senza ripianificare "
          "verso quello escluso")
    GRAPH_KW.clear()

    # S16: il controllo sulla posa d'arrivo (arrival_has_exit). Una direzione girabile che si
    # ferma perche' finiscono i dati e' un'uscita: "non visto" non vuol dire "chiuso". Il 28-10
    # 12:16 lo stesso passo nel corridoio e' stato rifiutato tre volte di fila ("0/24
    # direzioni"), e alla terza eseguito: era quello giusto. Prima prova diretta su due griglie
    # fatte a mano, poi una missione corta nel corridoio.
    print("\nS16. controllo sulla posa d'arrivo: corridoio che continua oltre i dati, nicchia cieca")
    from scipy.ndimage import distance_transform_edt

    def _snap(occupied, seen, ax, ay, cs=0.03, n=128, ox=0.0, oy=-1.92):
        free_d = distance_transform_edt(~occupied) * cs
        occ_d = distance_transform_edt(occupied) * cs
        od = np.where(occupied, -occ_d, free_d)
        z = np.zeros((n, n))
        return av.GridSnapshot(od, np.zeros((n, n), bool), seen, z, ox, oy, cs, ax, ay)
    n, cs = 128, 0.03
    xs = (np.arange(n) + 0.5) * cs
    ys = -1.92 + (np.arange(n) + 0.5) * cs
    X, Y = np.meshgrid(xs, ys)
    walls = (np.abs(Y) >= 0.45)                         # corridoio largo 0.9 m lungo x
    seen_corr = X <= 2.2                                 # la scansione finisce a x = 2.2
    ok_corr, fan_corr = av.arrival_has_exit(_snap(walls, seen_corr, 2.0, 0.0), (2.0, 0.0), 0.0)
    fwd = [d for d in fan_corr if abs(d['rel_deg']) < 1.0][0]
    check(ok_corr, f"corridoio che continua oltre i dati: un'uscita c'e' (avanti {fwd['advance']:.2f} m, "
                   f"{fwd['reason']}, girabile {fwd['rotatable']})")
    pocket = walls | (X >= 2.45)                         # stessa larghezza, chiuso davanti
    ok_pocket, fan_pocket = av.arrival_has_exit(_snap(pocket, np.ones((n, n), bool), 2.0, 0.0),
                                                (2.0, 0.0), 0.0)
    check(not ok_pocket, f"nicchia cieca tutta vista: nessuna uscita "
                         f"({sum(d['usable'] for d in fan_pocket)}/{len(fan_pocket)} direzioni)")

    GRAPH_KW.clear()
    GRAPH_KW.update(sampling='casuale', distribution='poisson', density_per_m2=4.0, seed=1)
    r = run("S16. corridoio largo 0.9 m: nessun rifiuto [ARRIVO] a vuoto",
            [(5.0, 8.0, 0.0, 3.0), (5.0, 8.0, 3.9, 12.0)], (2.0, 3.5, 0.0), (0, 1))
    GRAPH_KW.clear()
    check(r['err'] is None and r['ok'] is True, "cella raggiunta attraverso il corridoio")
    check(r['tags']['[ARRIVO]'] == 0, f"nessun rifiuto [ARRIVO] ({r['tags']['[ARRIVO]']}; "
                                      f"prima della correzione 3, l'ultimo poi eseguito lo stesso)")
    check(r['scans'] <= 8, f"{r['scans']} scansioni (prima 10)")

    # S17: comando che scade a pochi cm dal punto (28-10 13:50, cella (3,2): 0.42 m su 0.44
    # in salita). Entro MOVE_NEAR_GOAL_M vale come arrivato; piu' lontano e' un fallimento fisico.
    STALL.update(n=1, short_m=0.03)
    r = run("S17. tempo scaduto a 3 cm dal punto: vale come arrivato", [], (2.0, 2.0, 0.785), (0, 1))
    check(r['err'] is None and r['ok'] is True, "cella raggiunta")
    check("[QUASI]" in r['log'] and "fallito meccanicamente" not in r['log'],
          "il tratto vale come riuscito, nessun arco scartato per fallimento fisico")
    STALL.update(n=1, short_m=0.30)
    r = run("S17. tempo scaduto a 30 cm dal punto: fallimento fisico come prima", [], (2.0, 2.0, 0.785), (0, 1))
    STALL.update(n=0)
    check(r['err'] is None, "nessuna eccezione")
    check("[QUASI]" not in r['log'] and "fallito meccanicamente" in r['log'],
          "oltre la soglia resta un fallimento fisico")

    for d in [x for x in os.listdir(tempfile.gettempdir()) if x.startswith("missione_e2e_")]:
        shutil.rmtree(os.path.join(tempfile.gettempdir(), d), ignore_errors=True)
    shutil.rmtree(STUBS, ignore_errors=True)
    print("\n" + "=" * 70)
    print("TUTTO OK" if not FAILS else f"{len(FAILS)} CONTROLLI FALLITI:\n  - " + "\n  - ".join(FAILS))
    sys.exit(1 if FAILS else 0)
