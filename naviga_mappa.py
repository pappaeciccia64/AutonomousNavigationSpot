"""
naviga_mappa.py -- «vai lì» su una zona GIA' ESPLORATA, con la mappa di fine missione.

Idea 3 del TODO: a esplorazione finita si riusano le mappe globali (occupazione e terreno),
il grafo PRM della missione e il grafo GraphNav scaricato, per mandare il robot in un punto
scelto da noi senza rifare l'esplorazione. Non tocca il ciclo di easy_walk.py: e' un modo
d'uso a parte.

COME SI COLLEGANO I TRE PEZZI
  - Mappe globali e PRM sono nel frame VISION DELLA MISSIONE (quello del robot acceso quel
    giorno): dopo un riavvio quel frame non esiste piu'.
  - Il grafo GraphNav scaricato (graph/<missione>/Grafo_*) ha, nell'istantanea di ogni
    waypoint, lo stato del robot al momento della creazione: da li' la posa del waypoint
    nel VECCHIO frame VISION (vedi posa_vecchia_del_waypoint).
  - Oggi: si carica il grafo su Spot e ci si localizza (fiducial di partenza, oppure robot
    messo sulla posa di wp_0). GraphNav dice su quale waypoint e' il robot e dove sta
    rispetto a lui; con la posa vecchia di quel waypoint si ricava la trasformazione
    vecchio VISION -> nuovo VISION (Trasformazione2D). Si ricalcola a ogni giro del ciclo,
    quindi segue anche la deriva dell'odometria di oggi, finche' GraphNav e' localizzato.

COSA FA
  1. Pianifica sul PRM della missione (stessi nodi: stesso seme o reticolo, letti dal log),
     con gli archi rivalutati sulla mappa globale finale e limitati alla zona VISTA.
  2. Segue il percorso con il fronte sicuro sulla scansione dal vivo (arcVerification), come
     il ciclo di missione: arresto a meta' movimento, attesa, blocco confermato -> arco
     scartato e nuova pianificazione.
  3. Se il PRM non ce la fa (blocchi ripetuti, nessun percorso), ripiego: GraphNav fino al
     waypoint piu' vicino all'obiettivo, poi ultimo tentativo col PRM da li'.

COSA NON FA (ancora)
  - niente cono, arretramento, trappole, ritorni sulle briciole: un tratto che richiede una
    rotazione impossibile si scarta e si ripianifica;
  - le mappe NON si aggiornano con le scansioni di oggi: il fronte vede gli ostacoli nuovi
    e prima di ogni piano si scartano gli archi che la scansione di adesso vede bloccati
    (scarta_archi_dal_vivo), ma fuori dalla finestra della scansione decide la mappa vecchia;
  - la deriva DENTRO la mappa globale della missione (TODO, lavori principali punto 3) resta:
    un ostacolo visto a fine missione puo' essere spostato di decine di cm rispetto a GraphNav.

USO
  Senza robot (anteprima del piano, coordinate nel frame della missione: x avanti, y a
  sinistra rispetto alla partenza, in metri):
    python3 naviga_mappa.py 26-10-2026_15-11-21 --verso 6,2 --anteprima piano.png
    python3 naviga_mappa.py 26-10-2026_15-11-21 --cella 1,3 --da 4,0 --anteprima piano.png
  Sul robot (robot alla partenza della missione, con il tag di partenza in vista):
    python3 naviga_mappa.py 26-10-2026_15-11-21 --verso 6,2
    python3 naviga_mappa.py 26-10-2026_15-11-21 --interattivo        # chiede gli obiettivi
    python3 naviga_mappa.py 26-10-2026_15-11-21 --clic               # obiettivi con un clic
    python3 naviga_mappa.py 26-10-2026_15-11-21 --clic --anteprima piano.png   # clic, senza robot
  Obiettivi: --verso X,Y (frame della missione), --cella R,C (centro della cella),
  --waypoint wp_N (un waypoint del grafo GraphNav), oppure --clic: clic sinistro sulla mappa,
  il percorso compare subito, Invio conferma, Esc esce. Senza nome della missione: l'ultima
  che ha mappe globali, log e grafo GraphNav.
"""
import argparse
import contextlib
import glob
import io
import os
import pickle
import re
import sys
import time
from datetime import datetime
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

import parametri as P
import spotGrid as sg
import prm_graph
import environmentMap
import mission_graph
import arcVerification

LOG_TAG = "[NAVIGA]"
# Piani al massimo per un obiettivo, prima del ripiego su GraphNav. Un piano nuovo si fa dopo
# un blocco confermato da fermo e quando la scansione vede bloccato un arco del percorso.
MAX_PIANI = 10


def _quiet():
    return contextlib.redirect_stdout(io.StringIO())


# =========================================================================================
# GEOMETRIA: il passaggio fra il vecchio frame VISION (missione) e quello di oggi
# =========================================================================================
def _yaw_of_quat(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))


def _wrap(a):
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


class Trasformazione2D:
    """Rototraslazione piana: p_nuovo = R(th) p_vecchio + t. Identita' di default."""

    def __init__(self, tx=0.0, ty=0.0, th=0.0):
        self.tx, self.ty, self.th = float(tx), float(ty), float(th)

    @staticmethod
    def da_pose(pose_nuova, pose_vecchia):
        """La trasformazione che porta pose_vecchia (x, y, yaw) in pose_nuova: stessa posa
        fisica del corpo vista nei due frame."""
        xn, yn, an = pose_nuova
        xo, yo, ao = pose_vecchia
        th = _wrap(an - ao)
        c, s = np.cos(th), np.sin(th)
        return Trasformazione2D(xn - (c * xo - s * yo), yn - (s * xo + c * yo), th)

    def applica(self, x, y):
        c, s = np.cos(self.th), np.sin(self.th)
        return float(self.tx + c * x - s * y), float(self.ty + s * x + c * y)

    def inversa(self, x, y):
        c, s = np.cos(self.th), np.sin(self.th)
        dx, dy = x - self.tx, y - self.ty
        return float(c * dx + s * dy), float(-s * dx + c * dy)

    def applica_yaw(self, yaw):
        return _wrap(yaw + self.th)

    def scarto(self, altra):
        """Quanto si sposta un punto vicino all'origine fra le due stime (m) e di quanto
        cambia l'angolo (gradi): serve al log, per vedere se la stima balla."""
        return (float(np.hypot(self.tx - altra.tx, self.ty - altra.ty)),
                float(np.degrees(abs(_wrap(self.th - altra.th)))))

    def __repr__(self):
        return f"(tx {self.tx:+.2f} m, ty {self.ty:+.2f} m, rotazione {np.degrees(self.th):+.1f} gradi)"


class FrameMissione:
    """Frame della missione: origine nella partenza, x avanti, y a sinistra (come le celle)."""

    def __init__(self, ox, oy, yaw):
        self.ox, self.oy, self.yaw = float(ox), float(oy), float(yaw)

    def a_vision(self, xm, ym):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return self.ox + c * xm - s * ym, self.oy + s * xm + c * ym

    def da_vision(self, x, y):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        dx, dy = x - self.ox, y - self.oy
        return c * dx + s * dy, -s * dx + c * dy


# =========================================================================================
# LETTURA DELLA MISSIONE (senza robot)
# =========================================================================================
def _ultima_missione_completa():
    cands = []
    for d in glob.glob(os.path.join(HERE, 'MissionMap', 'Mission_*')):
        name = os.path.basename(d)[len('Mission_'):]
        if (os.path.exists(os.path.join(d, 'global_occupancy_map.pkl'))
                and os.path.exists(os.path.join(HERE, 'MissionLogs', f'Mission_{name}', 'mission_log.txt'))
                and glob.glob(os.path.join(HERE, 'graph', f'Mission_{name}', 'Grafo_*', 'graph'))):
            cands.append((os.path.getmtime(os.path.join(d, 'global_occupancy_map.pkl')), name))
    return max(cands)[1] if cands else None


def configurazione_missione(name):
    """Origine, orientamento, griglia e grafo della missione, dal suo log (stesse regole del
    rigioco: test_replay.mission_setup, qui senza richiedere le scansioni)."""
    log_path = os.path.join(HERE, 'MissionLogs', f'Mission_{name}', 'mission_log.txt')
    if not os.path.exists(log_path):
        raise FileNotFoundError(f"manca il log della missione: {log_path}")
    log = open(log_path, errors='replace').read()
    m = re.search(r'(?:Origin set at world coordinates|Origine fissata alle coordinate) '
                  r'\(([-\d.]+), ([-\d.]+)\)', log)
    a = re.search(r'(?:Initial orientation|Orientamento iniziale): ([-\d.]+)', log)
    if m is None or a is None:
        raise ValueError("origine o orientamento della missione non trovati nel log")
    g = re.search(r'Griglia di missione: (\d+) x (\d+) celle da ([\d.]+) m', log)
    rows, cols, cell = (int(g.group(1)), int(g.group(2)), float(g.group(3))) if g else (2, 4, 5.0)
    s = re.search(r'Reticolo: passo richiesto ([\d.]+) m', log)
    gr = re.search(r'\[CONFIG\] Grafo: nodi (\w+).*; archi da ([\d.]+) a ([\d.]+) m', log)
    rnd = re.search(r"\[GRAFO\] Nodi casuali: distribuzione (\w+), densita' ([\d.]+) nodi/m2, "
                    r"seme (\d+)", log)
    return dict(
        name=name, origin=(float(m.group(1)), float(m.group(2))), yaw=np.radians(float(a.group(1))),
        rows=rows, cols=cols, cell=cell, spacing=float(s.group(1)) if s else 0.25,
        sampling=gr.group(1) if gr else 'reticolo',
        min_edge=float(gr.group(2)) if gr else 0.5, max_edge=float(gr.group(3)) if gr else 1.25,
        distribution=rnd.group(1) if rnd else None,
        density=float(rnd.group(2)) if rnd else None,
        seed=int(rnd.group(3)) if rnd else None)


def carica_mappa_occupazione(path):
    """GlobalGrid dal .pkl di fine missione (GlobalGrid.save_map)."""
    with open(path, 'rb') as f:
        d = pickle.load(f)
    gm = sg.GlobalGrid(resolution=d['resolution'])
    for k, v in d['grid'].items():
        gm._set_state(tuple(k), float(v))     # tiene allineato l'indice a grana grossa
    for x, y, r in d.get('hazards', []):
        gm.mark_hazard(x, y, r)
    gm.walked_cells = set(tuple(k) for k in d.get('walked_cells', []))
    for key in ('clear_near_sum', 'clear_near_n', 'clear_all_sum', 'clear_all_n'):
        setattr(gm, key, dict(d.get(key, {})))
    return gm


def carica_mappa_terreno(path):
    """GlobalTerrainGrid dal .pkl di fine missione (GlobalTerrainGrid.save_map)."""
    with open(path, 'rb') as f:
        d = pickle.load(f)
    tm = sg.GlobalTerrainGrid(resolution=d['resolution'])
    tm.height_sum = dict(d['height_sum'])
    tm.height_count = dict(d['height_count'])
    tm.mission_z0 = d.get('mission_z0')
    keys = np.array([k for k, c in tm.height_count.items() if c > 0], dtype=np.float64)
    r = tm.resolution
    tm.update_rects = ([(keys[:, 0].min() * r, (keys[:, 0].max() + 1) * r,
                         keys[:, 1].min() * r, (keys[:, 1].max() + 1) * r)] if keys.size else [])
    return tm


def aggiungi_scansioni(name, gm, tm):
    """Passa nelle mappe anche i pacchetti di scansione salvati (scans/), con la funzione stessa
    della missione. I .pkl si salvano a ogni INIZIO di tentativo: le scansioni del ciclo
    dell'ultimo tentativo ci sono solo qui. Restituisce quante ne ha usate."""
    import test_replay                           # lettura dei pacchetti, come nel rigioco
    files = glob.glob(os.path.join(HERE, 'MissionMap', f'Mission_{name}', 'scans', '*.npz'))
    files.sort(key=lambda f: float(np.load(f)['time']))
    for f in files:
        a = test_replay.scan_arrays(np.load(f))
        rx, ry, ryaw = a['robot']
        sg.fold_scan_into_global_maps(gm, tm, a['pts'], a['z'].ravel(), a['obstacle_mask'],
                                      a['valid'].ravel(), a['od_raw'].ravel(), (a['ny'], a['nx']),
                                      (rx, ry, ryaw))
    return len(files)


def posa_vecchia_del_waypoint(waypoint, snapshot):
    """
    (x, y, yaw) del frame del waypoint nel VISION della missione. Esatta: vision_T_odom
    dall'istantanea (stesso istante, stessa fotografia di trasformate) per odom_T_waypoint
    (l'inversa di waypoint_tform_ko, che il grafo porta con se'). Se manca qualcosa, la posa
    del corpo nell'istantanea (come mission_viewer e graphnav_log_drift).
    """
    from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, ODOM_FRAME_NAME, \
        BODY_FRAME_NAME
    from bosdyn.client.math_helpers import SE3Pose
    ts = snapshot.robot_state.kinematic_state.transforms_snapshot
    v_T_body = get_a_tform_b(ts, VISION_FRAME_NAME, BODY_FRAME_NAME)
    o_T_body = get_a_tform_b(ts, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    if v_T_body is not None and o_T_body is not None and waypoint.HasField('waypoint_tform_ko'):
        v_T_wp = v_T_body * o_T_body.inverse() * SE3Pose.from_proto(waypoint.waypoint_tform_ko).inverse()
        return float(v_T_wp.x), float(v_T_wp.y), _yaw_of_quat(v_T_wp.rotation)
    if v_T_body is not None:
        return float(v_T_body.x), float(v_T_body.y), _yaw_of_quat(v_T_body.rotation)
    return None


def carica_grafo_graphnav(name):
    """
    Il grafo GraphNav scaricato a fine missione (l'ultimo Grafo_*), con la posa vecchia di
    ogni waypoint. dict: dir, graph (proto), wp {id: {name, x, y, yaw}}, by_name {nome: id}.
    """
    from bosdyn.api.graph_nav import map_pb2
    gdirs = sorted(glob.glob(os.path.join(HERE, 'graph', f'Mission_{name}', 'Grafo_*')))
    gdirs = [d for d in gdirs if os.path.exists(os.path.join(d, 'graph'))]
    if not gdirs:
        raise FileNotFoundError(f"nessun grafo GraphNav in graph/Mission_{name}/Grafo_*")
    gdir = gdirs[-1]
    graph = map_pb2.Graph()
    with open(os.path.join(gdir, 'graph'), 'rb') as f:
        graph.ParseFromString(f.read())
    wps, missing = {}, 0
    for w in graph.waypoints:
        path = os.path.join(gdir, 'waypoint_snapshots', w.snapshot_id)
        if not os.path.exists(path):
            missing += 1
            continue
        snap = map_pb2.WaypointSnapshot()
        with open(path, 'rb') as f:
            snap.ParseFromString(f.read())
        pose = posa_vecchia_del_waypoint(w, snap)
        if pose is None:
            missing += 1
            continue
        wps[w.id] = dict(name=w.annotations.name or w.id[:12], x=pose[0], y=pose[1], yaw=pose[2])
    by_name = {v['name']: k for k, v in wps.items()}
    print(f"{LOG_TAG} Grafo GraphNav {os.path.basename(gdir)}: {len(graph.waypoints)} waypoint "
          f"({len(wps)} con posa nota, {missing} senza istantanea), {len(graph.edges)} archi.")
    return dict(dir=gdir, graph=graph, wp=wps, by_name=by_name)


# =========================================================================================
# IL PRM DELLA MISSIONE SULLA MAPPA FINALE
# =========================================================================================
def _imposta_grafo_della_missione(cfg):
    """Gli stessi parametri dei nodi della missione (stesso seme: stessi nodi, stessi id)."""
    P.MISSION_LATTICE_SPACING_M = cfg['spacing']
    P.MISSION_GRAPH_SAMPLING = cfg['sampling']
    P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = cfg['min_edge'], cfg['max_edge']
    if cfg['sampling'] == 'casuale':
        P.MISSION_RANDOM_DISTRIBUTION = cfg['distribution']
        P.MISSION_RANDOM_DENSITY = cfg['density']
        P.MISSION_RANDOM_SEED = cfg['seed']


def _frazione_vista(gm, x1, y1, x2, y2, step=0.05):
    n = max(2, int(np.ceil(np.hypot(x2 - x1, y2 - y1) / step)) + 1)
    t = np.linspace(0.0, 1.0, n)
    return float(np.mean([gm.is_known(x1 + s * (x2 - x1), y1 + s * (y2 - y1)) for s in t]))


# Un arco si usa solo se e' stato visto per almeno questa frazione della sua lunghezza.
MIN_FRAZIONE_VISTA = 0.9


def limita_alla_zona_vista(prm, gm, min_frazione=MIN_FRAZIONE_VISTA):
    """
    Il PRM della missione copre TUTTA l'area, e un arco su terreno mai visto non ha veti
    (nessun dato = nessun motivo per scartarlo). Qui si naviga solo sul visto: nodi su celle
    osservate, archi osservati per almeno min_frazione della loro lunghezza. Restituisce
    (nodi ammessi, archi tolti).
    """
    allowed = [nid for nid, (x, y) in prm.nodes.items() if gm.is_known(x, y)]
    ok = set(allowed)
    tolti = 0
    for a in list(prm.edges):
        for b, _w in list(prm.edges.get(a, ())):
            if a >= b:
                continue
            if a not in ok or b not in ok or _frazione_vista(gm, *prm.nodes[a], *prm.nodes[b]) < min_frazione:
                prm.mark_edge_invalid(a, b)
                tolti += 1
    prm.set_allowed_nodes(allowed)
    return allowed, tolti


class MappaMissione:
    """Tutto cio' che serve per pianificare, senza robot: frame, mappe, PRM, grafo GraphNav."""

    def __init__(self, name, cfg, gm, tm, ng):
        """Dai pezzi gia' pronti (le prove li costruiscono in memoria); dai file: carica()."""
        self.name, self.cfg, self.gm, self.tm, self.ng = name, cfg, gm, tm, ng
        self.frame = FrameMissione(cfg['origin'][0], cfg['origin'][1], cfg['yaw'])
        self._costruisci_prm()

    @classmethod
    def carica(cls, name=None, con_scansioni=False, senza_graphnav=False):
        """La missione dai suoi file: log, mappe .pkl (e scansioni), grafo GraphNav scaricato."""
        name = name or _ultima_missione_completa()
        if name is None:
            raise FileNotFoundError("nessuna missione con mappe globali, log e grafo GraphNav")
        name = name[len('Mission_'):] if name.startswith('Mission_') else name
        cfg = configurazione_missione(name)
        folder = os.path.join(HERE, 'MissionMap', f'Mission_{name}')
        gm = carica_mappa_occupazione(os.path.join(folder, 'global_occupancy_map.pkl'))
        tm = carica_mappa_terreno(os.path.join(folder, 'global_terrain_map.pkl'))
        n_scan = 0
        if con_scansioni:
            with _quiet():
                n_scan = aggiungi_scansioni(name, gm, tm)
        n_occ = sum(1 for v in gm.grid.values() if v == -1.0)
        print(f"{LOG_TAG} Missione {name}: {cfg['rows']} x {cfg['cols']} celle da "
              f"{cfg['cell']:g} m, nodi {cfg['sampling']}"
              + (f" (seme {cfg['seed']})" if cfg['seed'] is not None else "")
              + f"; mappa: {len(gm.grid)} celle viste, {n_occ} occupate"
              + (f", piu' {n_scan} scansioni salvate" if con_scansioni else "") + ".")
        ng = None if senza_graphnav else carica_grafo_graphnav(name)
        return cls(name, cfg, gm, tm, ng)

    def _costruisci_prm(self):
        t0 = time.perf_counter()
        cfg = self.cfg
        with _quiet():
            self.env = environmentMap.EnvironmentMap(rows=cfg['rows'], cols=cfg['cols'], cell_size=cfg['cell'])
            self.env.set_origin(cfg['origin'][0], cfg['origin'][1], cfg['yaw'])
            _imposta_grafo_della_missione(cfg)
            self.prm, _sampler, self.mg = mission_graph.build_mission_prm(self.env, verbose=False)
            self.prm.set_global_terrain_map(self.tm)
            mission_graph.rebuild_prm_edges(self.prm, self.gm, incremental=False)
        # Obiettivi lontani: nessun tetto alla deviazione (in missione serve a non attraversare
        # mezza area per entrare in una cella vicina; qui la strada lunga e' quella giusta).
        self.prm.set_detour_limit(None)
        allowed, tolti = limita_alla_zona_vista(self.prm, self.gm)
        n_edges = sum(len(v) for v in self.prm.edges.values()) // 2
        print(f"{LOG_TAG} PRM: {len(self.prm.nodes)} nodi, {len(allowed)} sul visto; {n_edges} archi "
              f"validi ({tolti} tolti perche' su terreno non visto) in {time.perf_counter() - t0:.1f} s.")

    # --- obiettivi -----------------------------------------------------------------------
    def obiettivo_vision(self, verso=None, cella=None, waypoint=None):
        """L'obiettivo nel VISION della missione, dai tre modi di indicarlo, e una descrizione."""
        if verso is not None:
            x, y = self.frame.a_vision(*verso)
            return (x, y), f"({verso[0]:.2f}, {verso[1]:.2f}) nel frame della missione"
        if cella is not None:
            r, c = cella
            if not (0 <= r < self.env.rows and 0 <= c < self.env.cols):
                raise ValueError(f"la cella ({r},{c}) e' fuori dalla griglia "
                                 f"{self.env.rows} x {self.env.cols}")
            return tuple(self.env.get_world_position_from_cell(r, c)), f"centro della cella ({r},{c})"
        if waypoint is not None:
            if self.ng is None or waypoint not in self.ng['by_name']:
                raise ValueError(f"waypoint {waypoint} non trovato nel grafo GraphNav")
            w = self.ng['wp'][self.ng['by_name'][waypoint]]
            return (w['x'], w['y']), f"waypoint {waypoint}"
        raise ValueError("serve un obiettivo: --verso, --cella o --waypoint")

    def _componente(self, start):
        seen, stack = {start}, [start]
        allowed = self.prm.allowed_nodes
        while stack:
            u = stack.pop()
            for v, _w in self.prm.edges.get(u, ()):
                if v not in seen and (allowed is None or v in allowed):
                    seen.add(v)
                    stack.append(v)
        return seen

    def pianifica(self, da_xy, a_xy):
        """
        Percorso sul PRM da da_xy ad a_xy (VISION della missione). Il nodo d'arrivo e' il nodo
        ammesso piu' vicino all'obiettivo FRA QUELLI RAGGIUNGIBILI dalla partenza: un obiettivo
        dentro un ostacolo o in una zona non collegata porta il robot il piu' vicino possibile,
        e il log dice quanto manca. Restituisce dict (path, start, goal, lunghezza,
        mancano_m) o None.
        """
        start = self.prm.anchor_node(da_xy[0], da_xy[1], self.gm, sg.PRM_EDGE_SAFETY_MARGIN_M,
                                     ignore_near_m=prm_graph.ANCHOR_IGNORE_NEAR_M)
        if start is None:
            return None
        comp = self._componente(start)
        if len(comp) <= 1:
            print(f"{LOG_TAG} Il nodo di partenza {start} non ha archi validi verso il visto.")
            return None
        ids = np.array(sorted(comp))
        xy = np.array([self.prm.nodes[int(i)] for i in ids])
        d = np.hypot(xy[:, 0] - a_xy[0], xy[:, 1] - a_xy[1])
        goal = int(ids[int(np.argmin(d))])
        path = self.prm.find_path_dijkstra(start, goal)
        if path is None:
            return None
        return dict(path=path, start=start, goal=goal, lunghezza=self.prm.path_length(path),
                    mancano_m=float(d.min()))

    def waypoint_piu_vicino(self, xy, nomi_nostri=False):
        if not self.ng or not self.ng['wp']:
            return None
        cands = [(np.hypot(w['x'] - xy[0], w['y'] - xy[1]), wid) for wid, w in self.ng['wp'].items()
                 if not nomi_nostri or re.fullmatch(r'wp_\d+', w['name'])]
        return min(cands)[1] if cands else None

    # --- figura ---------------------------------------------------------------------------
    # Tutto nel frame della missione: in figura l'asse orizzontale e' y (a sinistra della
    # partenza, invertito: la sinistra del robot sta a sinistra) e quello verticale e' x
    # (avanti). Un punto (xm, ym) si disegna in (ym, xm).
    def disegna_mappa(self, ax):
        """Sfondo: mappa (rosso occupato, verde visto libero), archi del PRM sul visto,
        waypoint GraphNav."""
        from matplotlib.collections import LineCollection
        fr = self.frame
        # Mappa come immagine, ricampionata nel frame della missione (ruotato rispetto al
        # VISION): 0 mai vista, 1 vista libera, 2 occupata.
        from matplotlib.colors import ListedColormap
        r = self.gm.resolution
        keys = np.array(list(self.gm.grid.keys()), dtype=np.int64)
        vals = np.array(list(self.gm.grid.values()))
        gx0, gy0 = keys.min(axis=0)
        ras = np.zeros(tuple(keys.max(axis=0) - (gx0, gy0) + 1), dtype=np.uint8)
        ras[keys[:, 0] - gx0, keys[:, 1] - gy0] = np.where(vals > 0, 1, 2)
        mx, my = fr.da_vision((keys[:, 0] + 0.5) * r, (keys[:, 1] + 0.5) * r)
        xs = np.arange(mx.min(), mx.max() + r, r)
        ys = np.arange(my.min(), my.max() + r, r)
        XM, YM = np.meshgrid(xs, ys, indexing='ij')
        vx, vy = fr.a_vision(XM, YM)
        ix = np.floor(vx / r).astype(np.int64) - gx0
        iy = np.floor(vy / r).astype(np.int64) - gy0
        ok = (ix >= 0) & (ix < ras.shape[0]) & (iy >= 0) & (iy < ras.shape[1])
        img = np.zeros(XM.shape, dtype=np.uint8)
        img[ok] = ras[ix[ok], iy[ok]]
        ax.imshow(img, origin='lower', interpolation='nearest', zorder=1,
                  cmap=ListedColormap(['#ffffff', '#b8dfb0', '#d0342c']), vmin=0, vmax=2,
                  extent=(ys[0] - r / 2, ys[-1] + r / 2, xs[0] - r / 2, xs[-1] + r / 2))
        ax.plot([], [], 's', c='#d0342c', label='occupato')
        ax.plot([], [], 's', c='#b8dfb0', label='visto libero')
        segs = []
        for a, lst in self.prm.edges.items():
            for b, _w in lst:
                if a < b:
                    (x1, y1), (x2, y2) = fr.da_vision(*self.prm.nodes[a]), fr.da_vision(*self.prm.nodes[b])
                    segs.append([(y1, x1), (y2, x2)])
        ax.add_collection(LineCollection(segs, colors='#888888', linewidths=0.3, alpha=0.5, zorder=3))
        if self.ng and self.ng['wp']:
            w = np.array([fr.da_vision(v['x'], v['y']) for v in self.ng['wp'].values()])
            ax.scatter(w[:, 1], w[:, 0], s=8, c='#2b6cb0', zorder=4, label='waypoint GraphNav')
        ax.set_aspect('equal')
        ax.set_xlim(ys[-1] + 0.5, ys[0] - 0.5)       # invertito: la sinistra del robot a sinistra
        ax.set_ylim(xs[0] - 0.5, xs[-1] + 0.5)
        ax.set_xlabel("y: a sinistra della partenza [m]")
        ax.set_ylabel("x: avanti dalla partenza [m]")
        ax.grid(alpha=0.3)

    def disegna_piano(self, ax, piano=None, da_xy=None, a_xy=None, etichetta_da='partenza'):
        """Percorso, partenza e obiettivo (punti VISION della missione). Restituisce gli
        oggetti disegnati, da togliere quando il piano cambia."""
        fr = self.frame
        out = []
        if piano is not None:
            p = np.array([fr.da_vision(*self.prm.nodes[n]) for n in piano['path']])
            out += ax.plot(p[:, 1], p[:, 0], '-', c='#ff8c00', lw=2.5, zorder=5,
                           label=f"percorso PRM {piano['lunghezza']:.1f} m")
        if da_xy is not None:
            m = fr.da_vision(*da_xy)
            out += ax.plot(m[1], m[0], 'o', c='k', ms=9, zorder=6, label=etichetta_da)
        if a_xy is not None:
            m = fr.da_vision(*a_xy)
            out += ax.plot(m[1], m[0], '*', c='#ffd400', mec='k', ms=18, zorder=6, label='obiettivo')
        return out

    def anteprima(self, png, piano=None, da_xy=None, a_xy=None, titolo=""):
        """La figura del piano in un file (senza finestra)."""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 10))
        self.disegna_mappa(ax)
        self.disegna_piano(ax, piano, da_xy, a_xy)
        ax.set_title(titolo or f"Missione {self.name}")
        ax.legend(loc='upper right', fontsize=8)
        fig.tight_layout()
        fig.savefig(png, dpi=130)
        plt.close(fig)
        print(f"{LOG_TAG} Anteprima salvata in {png}")


def descrivi_piano(piano, da_xy, a_xy):
    if piano is None:
        return "nessun percorso sul visto"
    return (f"{len(piano['path']) - 1} archi, {piano['lunghezza']:.1f} m (linea d'aria "
            f"{np.hypot(a_xy[0] - da_xy[0], a_xy[1] - da_xy[1]):.1f} m)"
            + (f", arrivo a {piano['mancano_m']:.2f} m dall'obiettivo" if piano['mancano_m'] > 0.05 else ""))


class SceltaConClic:
    """
    L'obiettivo scelto con un clic sulla mappa. Clic sinistro: obiettivo in quel punto, con il
    percorso pianificato subito (un altro clic lo sposta); Invio: conferma; Esc, q o finestra
    chiusa: nessun obiettivo. Con lo zoom o lo spostamento della barra degli strumenti attivi i
    clic servono a quelli e non scelgono niente.

    da_xy: partenza del piano (VISION della missione): il robot adesso, o la partenza.
    """

    def __init__(self, mappa, da_xy, titolo="", etichetta_da='partenza'):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.mappa, self.da_xy = mappa, da_xy
        self.titolo = titolo or f"Missione {mappa.name}"
        self.goal = self.piano = None
        self.confermato = False
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        mappa.disegna_mappa(self.ax)
        self._artisti = mappa.disegna_piano(self.ax, None, da_xy, None, etichetta_da=etichetta_da)
        self.etichetta_da = etichetta_da
        self.ax.legend(loc='upper right', fontsize=8)
        self._titolo("clic sinistro: obiettivo | Invio: conferma | Esc: esci")
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect('button_press_event', self._clic)
        self.fig.canvas.mpl_connect('key_press_event', self._tasto)

    def _titolo(self, riga):
        self.ax.set_title(f"{self.titolo}\n{riga}", fontsize=10)

    def _strumento_attivo(self):
        tb = getattr(getattr(self.fig.canvas, 'manager', None), 'toolbar', None)
        return bool(getattr(tb, 'mode', ''))

    def _clic(self, ev):
        if ev.inaxes is not self.ax or ev.button != 1 or ev.xdata is None or self._strumento_attivo():
            return
        xm, ym = float(ev.ydata), float(ev.xdata)          # (y, x) in figura
        self.goal = self.mappa.frame.a_vision(xm, ym)
        self.piano = self.mappa.pianifica(self.da_xy, self.goal)
        for art in self._artisti:
            art.remove()
        self._artisti = self.mappa.disegna_piano(self.ax, self.piano, self.da_xy, self.goal,
                                                 etichetta_da=self.etichetta_da)
        testo = descrivi_piano(self.piano, self.da_xy, self.goal)
        print(f"{LOG_TAG} Clic in ({xm:.2f}, {ym:.2f}) nel frame della missione: {testo}.")
        self._titolo(f"obiettivo ({xm:.2f}, {ym:.2f}): {testo}\n"
                     + ("Invio: conferma | clic: un altro punto | Esc: esci" if self.piano is not None
                        else "clic: un altro punto | Esc: esci"))
        self.ax.legend(loc='upper right', fontsize=8)
        self.fig.canvas.draw_idle()

    def _tasto(self, ev):
        if ev.key == 'enter' and self.goal is not None and self.piano is not None:
            self.confermato = True
            self.plt.close(self.fig)
        elif ev.key in ('escape', 'q'):
            self.confermato = False
            self.plt.close(self.fig)

    def scegli(self):
        """Apre la finestra e aspetta. (obiettivo VISION, descrizione) o None."""
        self.plt.show()
        if not self.confermato:
            return None
        xm, ym = self.mappa.frame.da_vision(*self.goal)
        return self.goal, f"({xm:.2f}, {ym:.2f}) nel frame della missione, scelto con un clic"

    def salva(self, png):
        self.fig.savefig(png, dpi=130)


def _finestra_disponibile():
    """Serve un backend con finestre: su Linux senza DISPLAY matplotlib non ne apre."""
    import matplotlib
    if sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print(f"{LOG_TAG} --clic apre una finestra, ma qui non c'e' uno schermo (DISPLAY vuoto).")
        return False
    if matplotlib.get_backend().lower() in ('agg', 'pdf', 'svg', 'ps', 'cairo', 'template'):
        print(f"{LOG_TAG} --clic apre una finestra, ma matplotlib usa il backend "
              f"'{matplotlib.get_backend()}', senza finestre.")
        return False
    return True


# =========================================================================================
# SUL ROBOT
# =========================================================================================
class NavigatoreRobot:
    """
    Il robot su una mappa gia' fatta: carica il grafo GraphNav, si localizza, mantiene la
    trasformazione vecchio VISION -> nuovo VISION e segue i percorsi del PRM col fronte sicuro.
    """

    def __init__(self, mappa, robot, robot_state_client, command_client, client_metadata, log_folder):
        import navGraphUtils
        self.mappa = mappa
        self.robot = robot
        self.rsc = robot_state_client
        self.cc = command_client
        self.ri = navGraphUtils.RecordingInterface(robot, log_folder, client_metadata)
        self.gn = self.ri._graph_nav_client
        self.T = None                       # Trasformazione2D vecchio -> nuovo VISION
        self.tracker = arcVerification.ArcVerificationTracker(robot)

    # --- grafo e localizzazione ----------------------------------------------------------
    def carica_grafo(self):
        """Il grafo scaricato a fine missione su Spot, con le istantanee che Spot non ha."""
        from bosdyn.api.graph_nav import map_pb2
        ng = self.mappa.ng
        self.ri.stop_recording()
        self.gn.clear_graph()
        resp = self.gn.upload_graph(graph=ng['graph'], generate_new_anchoring=not ng['graph'].anchoring.anchors)
        for sid in resp.unknown_waypoint_snapshot_ids:
            s = map_pb2.WaypointSnapshot()
            with open(os.path.join(ng['dir'], 'waypoint_snapshots', sid), 'rb') as f:
                s.ParseFromString(f.read())
            self.gn.upload_waypoint_snapshot(s)
        for sid in resp.unknown_edge_snapshot_ids:
            path = os.path.join(ng['dir'], 'edge_snapshots', sid)
            if not os.path.exists(path):
                continue
            s = map_pb2.EdgeSnapshot()
            with open(path, 'rb') as f:
                s.ParseFromString(f.read())
            self.gn.upload_edge_snapshot(s)
        self.ri.invalidate_graph_cache()
        print(f"{LOG_TAG} Grafo caricato su Spot ({len(resp.unknown_waypoint_snapshot_ids)} istantanee "
              f"di waypoint e {len(resp.unknown_edge_snapshot_ids)} di archi inviate).")

    def localizza(self, modo='auto'):
        """
        'fiducial': sul tag piu' visibile che sta nella mappa (quello di partenza);
        'wp_0': il robot e' fermo sulla posa di partenza della missione, stima = wp_0 + ICP;
        'auto': prima il fiducial, poi wp_0. True se GraphNav e' localizzato.
        """
        from bosdyn.api.graph_nav import graph_nav_pb2, nav_pb2
        from bosdyn.client.frame_helpers import get_odom_tform_body
        ok = False
        if modo in ('auto', 'fiducial'):
            ok = self.ri.initialize_with_fiducial(self.rsc, P.START_FIDUCIAL_ID)
        if not ok and modo in ('auto', 'wp_0'):
            wp0 = self.mappa.ng['by_name'].get('wp_0')
            if wp0 is None:
                print(f"{LOG_TAG} wp_0 non e' nel grafo: localizzazione impossibile senza fiducial.")
                return False
            try:
                state = self.rsc.get_robot_state()
                odom_T_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
                loc = nav_pb2.Localization()
                loc.waypoint_id = wp0
                loc.waypoint_tform_body.rotation.w = 1.0
                self.gn.set_localization(
                    initial_guess_localization=loc, ko_tform_body=odom_T_body.to_proto(),
                    max_distance=P.GRAPHNAV_RELOC_MAX_DIST_M, max_yaw=P.GRAPHNAV_RELOC_MAX_YAW_RAD,
                    fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL)
                ok = True
                print(f"{LOG_TAG} Localizzato su wp_0 (robot sulla posa di partenza, rifinito dall'ICP).")
            except Exception as e:
                print(f"{LOG_TAG} Localizzazione su wp_0 non riuscita: {e}")
        if ok:
            ok = self.aggiorna_trasformazione("dopo la localizzazione") is not None
        return ok

    def aggiorna_trasformazione(self, where=""):
        """
        Vecchio VISION -> nuovo VISION, da: posa vecchia del waypoint su cui GraphNav e'
        localizzato * dove sta il corpo rispetto a lui (adesso) = posa vecchia del corpo;
        posa nuova del corpo dall'odometria di oggi. None (e T invariata) se GraphNav non e'
        localizzato su un waypoint di cui si conosce la posa.
        """
        import spotUtils
        try:
            loc = self.gn.get_localization_state().localization
        except Exception as e:
            print(f"{LOG_TAG} Stato di GraphNav non leggibile ({where}): {e}")
            return None
        w = self.mappa.ng['wp'].get(loc.waypoint_id)
        if not loc.waypoint_id or w is None:
            print(f"{LOG_TAG} GraphNav non localizzato su un waypoint noto ({where}): tengo la stima di prima.")
            return None
        b = loc.waypoint_tform_body
        c, s = np.cos(w['yaw']), np.sin(w['yaw'])
        old = (w['x'] + c * b.position.x - s * b.position.y,
               w['y'] + s * b.position.x + c * b.position.y,
               _wrap(w['yaw'] + _yaw_of_quat(b.rotation)))
        x, y, _z, q = spotUtils.getPosition(self.rsc)
        nuova = Trasformazione2D.da_pose((x, y, _yaw_of_quat(q)), old)
        if self.T is None:
            print(f"{LOG_TAG} Frame della missione -> frame di oggi: {nuova} (su {w['name']}).")
        else:
            dm, dg = nuova.scarto(self.T)
            if dm > 0.10 or dg > 3.0:
                print(f"{LOG_TAG} Stima del frame cambiata di {dm:.2f} m e {dg:.1f} gradi {where} "
                      f"(su {w['name']}): GraphNav ha corretto la deriva di oggi.")
        self.T = nuova
        # Pose dei waypoint nel frame di OGGI: le usano il riaggancio e l'orientamento d'arrivo
        # di navGraphUtils, che confrontano con la posizione VISION attuale.
        self.ri.waypoint_poses = {
            v['name']: dict(zip(('x', 'y'), nuova.applica(v['x'], v['y'])),
                            yaw=nuova.applica_yaw(v['yaw']), waypoint_id=wid, t=None)
            for wid, v in self.mappa.ng['wp'].items()}
        return nuova

    def posa_robot(self):
        """(x, y, yaw) nel frame di oggi e (x, y) nel frame della missione."""
        import spotUtils
        x, y, _z, q = spotUtils.getPosition(self.rsc)
        return (x, y, _yaw_of_quat(q)), self.T.inversa(x, y)

    # --- navigazione ---------------------------------------------------------------------
    def scarta_archi_dal_vivo(self, snap):
        """
        La mappa della missione non sa degli ostacoli comparsi dopo. Prima di ogni piano: gli
        archi del PRM che cadono nella finestra della scansione e passano, in una cella VISTA
        adesso e non sotto il corpo, a meno di FRONTIER_CLEARANCE_M da un ostacolo, si scartano
        (come il veto dal vivo della missione, prm_graph.set_live_obstacles). La soglia e'
        quella del fronte, non quella del pianificatore: un arco che il fronte accetterebbe
        non si toglie, cosi' i passaggi stretti restano. Senza, un
        ostacolo nuovo costa un piano per ogni arco che lo attraversa. Scarto permanente per
        questa sessione: un ostacolo che se ne va (una persona) non restituisce gli archi.
        Restituisce quanti archi ha scartato.
        """
        prm = self.mappa.prm
        ny, nx = snap.obstacle_dist.shape
        x0, y0 = snap.origin_x, snap.origin_y
        x1, y1 = x0 + nx * snap.cell_size, y0 + ny * snap.cell_size
        node_new = {}
        for n, (x, y) in prm.nodes.items():
            px, py = self.T.applica(x, y)
            if x0 <= px <= x1 and y0 <= py <= y1:
                node_new[n] = (px, py)
        tolti = 0
        for a, (ax, ay) in node_new.items():
            for b, _w in list(prm.edges.get(a, ())):
                if b <= a or b not in node_new:
                    continue
                bx, by = node_new[b]
                k = max(2, int(np.ceil(np.hypot(bx - ax, by - ay) / 0.05)) + 1)
                t = np.linspace(0.0, 1.0, k)
                xs, ys = ax + t * (bx - ax), ay + t * (by - ay)
                rows, cols, inside = arcVerification._cells(snap, xs, ys)
                keep = inside & snap.seen[rows, cols] & ~arcVerification._under_body(snap, xs, ys)
                if np.any(snap.obstacle_dist[rows[keep], cols[keep]] < sg.FRONTIER_CLEARANCE_M):
                    prm.mark_edge_invalid(a, b)
                    tolti += 1
        if tolti:
            print(f"{LOG_TAG} Scansione di adesso: {tolti} archi del PRM passano vicino a ostacoli "
                  f"che la mappa della missione non ha (o non aveva li'): scartati.")
        return tolti

    def vai(self, goal_xy, descr, ripiego_graphnav=True):
        """
        Porta il robot verso goal_xy (VISION della missione). Prima il PRM, ripianificando
        dopo ogni blocco (al massimo MAX_PIANI piani); poi, se serve, GraphNav al
        waypoint piu' vicino all'obiettivo e un ultimo tentativo col PRM da li'. True se arriva.
        """
        print(f"\n{LOG_TAG} ===== Obiettivo: {descr} =====")
        for n_piano in range(1, MAX_PIANI + 1):
            self.aggiorna_trasformazione(f"prima del piano {n_piano}")
            snap = self.tracker._fresh_snapshot()
            if snap is not None:
                self.scarta_archi_dal_vivo(snap)
            _new, old = self.posa_robot()
            piano = self.mappa.pianifica(old, goal_xy)
            if piano is None:
                print(f"{LOG_TAG} Nessun percorso sul PRM da qui verso {descr}.")
                break
            print(f"{LOG_TAG} Piano {n_piano}: {len(piano['path']) - 1} archi, {piano['lunghezza']:.1f} m, "
                  f"dal nodo {piano['start']} al nodo {piano['goal']}"
                  + (f" (a {piano['mancano_m']:.2f} m dall'obiettivo: piu' vicino non si arriva sul visto)"
                     if piano['mancano_m'] > 0.3 else "") + ".")
            esito = self._segui(piano)
            if esito == 'arrivato':
                _new, old = self.posa_robot()
                print(f"{LOG_TAG} ARRIVATO: a {np.hypot(old[0] - goal_xy[0], old[1] - goal_xy[1]):.2f} m "
                      f"dall'obiettivo (frame della missione).")
                return True
            if esito == 'fermo':
                return False
        if not ripiego_graphnav:
            return False
        return self._ripiego_graphnav(goal_xy, descr)

    def _ripiego_graphnav(self, goal_xy, descr):
        import navGraphUtils
        wid = self.mappa.waypoint_piu_vicino(goal_xy)
        if wid is None:
            return False
        w = self.mappa.ng['wp'][wid]
        print(f"{LOG_TAG} Ripiego: GraphNav fino a {w['name']}, a "
              f"{np.hypot(w['x'] - goal_xy[0], w['y'] - goal_xy[1]):.2f} m dall'obiettivo.")
        face = self.T.applica(*goal_xy)
        if not navGraphUtils.graphnav_navigate_to(self.ri, wid, self.rsc, self.cc,
                                                  f"ripiego verso {descr}", face_xy=face):
            return False
        return self.vai(goal_xy, descr + " (dopo il ripiego)", ripiego_graphnav=False)

    def _segui(self, piano):
        """
        Il percorso, un giro per scansione: fronte sicuro sul resto del percorso (nel frame
        di oggi), decisione come in missione (arcVerification.decide_next_move), movimento con
        arresto a meta' se il fronte si accorcia. 'arrivato', 'bloccato' (arco scartato:
        ripianificare) o 'fermo' (caduta o errore: smettere).
        """
        import movements
        import spotUtils
        prm = self.mappa.prm
        robot_node = piano['start']
        wps_old = [(n, *prm.nodes[n]) for n in piano['path'][1:]]
        no_progress = 0
        while wps_old:
            self.aggiorna_trasformazione("nel giro")
            snap = self.tracker._fresh_snapshot()
            if snap is None:
                print(f"{LOG_TAG} Scansione non riuscita: riprovo.")
                time.sleep(0.3)
                continue
            rx, ry, ryaw = snap.robot_x, snap.robot_y, snap.robot_yaw
            # Come [SUBITO] in missione: un arco del resto del percorso che la scansione vede
            # bloccato adesso fa ripianificare subito, senza aspettare il fronte da fermo.
            if self.scarta_archi_dal_vivo(snap):
                seq = [robot_node] + [n for n, _x, _y in wps_old]
                if any((min(a, b), max(a, b)) in prm.tracker_blocked_edges for a, b in zip(seq[:-1], seq[1:])):
                    print(f"{LOG_TAG} Il percorso passa da un arco appena scartato: ripianifico subito.")
                    return 'bloccato'
            wps = [(n, *self.T.applica(x, y)) for n, x, y in wps_old]
            polyline = [(rx, ry)] + [(x, y) for _, x, y in wps]
            frontier = arcVerification.compute_safe_frontier(polyline, snap)
            print(arcVerification.describe_frontier(frontier))
            version = self.tracker.update_path(polyline[1:])
            dec = arcVerification.decide_next_move((rx, ry), wps, frontier, no_progress, in_target_cell=True)

            if dec['kind'] == 'reached_node':
                robot_node = wps_old.pop(0)[0]
                prm.set_robot_node(robot_node, 0.0)
                continue
            if dec['kind'] == 'arrived':
                print(f"{LOG_TAG} Il fronte non permette gli ultimi {frontier['path_len']:.2f} m: "
                      f"entro la tolleranza di {P.GOAL_REACHED_TOLERANCE_M:.2f} m, vale come arrivo.")
                return 'arrivato'
            if dec['kind'] == 'wait':
                no_progress += 1
                print(f"{LOG_TAG} Non posso avanzare: riguardo da fermo "
                      f"({no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS}).")
                time.sleep(P.FRONTIER_RESCAN_WAIT_S)
                continue
            if dec['kind'] == 'blocked':
                k = dec['segment']
                a = robot_node if k == 0 else wps_old[k - 1][0]
                b = wps_old[min(k, len(wps_old) - 1)][0]
                prm.mark_edge_invalid(a, b)
                print(f"{LOG_TAG} Blocco confermato da fermo sull'arco {a}->{b}: scartato, ripianifico.")
                return 'bloccato'

            # --- movimento ----------------------------------------------------------------
            rot = arcVerification.rotation_plan(ryaw, (rx, ry), dec['target'], frontier['od_robot'], snap=snap)
            if not rot['turn']:
                a, b = robot_node, wps_old[0][0]
                prm.mark_edge_invalid(a, b)
                print(f"{LOG_TAG} Servono {rot['dyaw_deg']:+.0f} gradi verso il nodo {b} ma il corpo "
                      f"toccherebbe ({rot['why']}): scarto l'arco {a}->{b} e ripianifico.")
                return 'bloccato'
            n_legs = int(dec.get('nodes', 1)) if dec['full'] else 0
            for leg in range(max(1, n_legs)):
                if not wps_old:
                    break
                nid, ox, oy = wps_old[0]
                tx, ty = self.T.applica(ox, oy) if n_legs else dec['target']
                cx, cy, cz, q = spotUtils.getPosition(self.rsc)
                # navigate_to legge solo l'orientamento (rotation) della posa del corpo
                v_T_body = SimpleNamespace(position=SimpleNamespace(x=cx, y=cy, z=cz), rotation=q)
                # Andatura dal terreno del tratto, nel frame della missione (mappa e PRM sono li').
                o_from = self.T.inversa(cx, cy)
                o_to = self.T.inversa(tx, ty)
                long_s, lat_s, rough = prm.sample_terrain_between_points(
                    o_from[0], o_from[1], o_to[0], o_to[1], node_idx1=robot_node, node_idx2=nid)
                mob = movements.select_mobility_params(max(long_s, lat_s), rough)
                print(f"{LOG_TAG} Tratto {robot_node}->{nid}"
                      + ("" if n_legs else f" (passo parziale di {dec['dist']:.2f} m)")
                      + f": verso ({tx:.2f}, {ty:.2f}).")
                ok, trav, cmd, why = self._muovi(tx, ty, cx, cy, v_T_body, version, mob)
                if movements.check_and_recover_fall(self.rsc, self.cc, None, "durante la navigazione"):
                    return 'fermo'
                if why is not None:
                    print(f"{LOG_TAG} Fermato a meta': {why}. Riguardo.")
                    break
                if not ok:
                    print(f"{LOG_TAG} Movimento non riuscito ({trav:.2f} m su {cmd:.2f}): scarto "
                          f"l'arco {robot_node}->{nid} e ripianifico.")
                    prm.mark_edge_invalid(robot_node, nid)
                    return 'bloccato'
                if n_legs:
                    robot_node = wps_old.pop(0)[0]
                    prm.set_robot_node(robot_node, 0.0)
                if leg + 1 < n_legs:
                    version = self.tracker.update_path([self.T.applica(x, y) for _, x, y in wps_old])
            no_progress = 0
        return 'arrivato'

    def _muovi(self, tx, ty, rx, ry, v_T_body, version, mob):
        """movements.navigate_to con l'arresto a meta' del ciclo di missione (vedi
        TentativoCella._muovi_e_registra): (riuscito, percorsi, comandati, motivo o None)."""
        import movements
        stato = {'bad': 0, 'why': None}
        t0 = time.time()

        def _abort():
            f = self.tracker.get_frontier()
            if f is None or f.get('path_version') != version or f['time'] <= t0:
                return False
            frx, fry = f['robot_xy']
            remaining = float(np.hypot(tx - frx, ty - fry))
            reach = arcVerification.allowed_advance(f)
            if (f['reason'] == 'fine_dati'
                    and float(np.hypot(frx - rx, fry - ry)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                stato['bad'] = 0
                return False
            if reach + P.FRONTIER_ABORT_TOLERANCE_M < remaining:
                stato['bad'] += 1
                if stato['bad'] >= P.FRONTIER_ABORT_CONFIRM:
                    stato['why'] = (f"il fronte si e' accorciato ({f['reason']}): raggiungibili "
                                    f"{reach:.2f} m su {remaining:.2f} m")
                    return True
            else:
                stato['bad'] = 0
            return False

        ok, trav, cmd = movements.navigate_to(tx, ty, rx, ry, self.rsc, self.cc, v_T_body,
                                              should_abort=_abort, **mob)
        return ok, trav, cmd, stato['why']


# =========================================================================================
# RIGA DI COMANDO
# =========================================================================================
def _coppia(s, tipo=float):
    a, b = (t.strip() for t in s.split(','))
    return tipo(a), tipo(b)


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description="Navigazione su una zona gia' esplorata, con la mappa "
                                            "di fine missione e il grafo PRM")
    p.add_argument('missione', nargs='?', default=None,
                   help="nome della missione (es. 26-10-2026_15-11-21); default: l'ultima completa")
    g = p.add_mutually_exclusive_group()
    g.add_argument('--verso', type=_coppia, metavar='X,Y',
                   help="obiettivo nel frame della missione: X avanti, Y a sinistra della partenza (m)")
    g.add_argument('--cella', type=lambda s: _coppia(s, int), metavar='R,C', help="centro della cella")
    g.add_argument('--waypoint', metavar='wp_N', help="un waypoint del grafo GraphNav")
    g.add_argument('--interattivo', action='store_true',
                   help="sul robot: chiede un obiettivo dopo l'altro (X,Y | c R,C | wp_N | q)")
    g.add_argument('--clic', action='store_true',
                   help="obiettivo scelto con un clic sulla mappa (Invio conferma, Esc esce); sul "
                        "robot uno dopo l'altro, con --anteprima solo il piano")
    p.add_argument('--anteprima', metavar='PNG',
                   help="SENZA ROBOT: pianifica e salva la figura del piano, poi esce")
    p.add_argument('--da', type=_coppia, metavar='X,Y',
                   help="con --anteprima: partenza nel frame della missione (default: la partenza, 0,0)")
    p.add_argument('--localizzazione', choices=('auto', 'fiducial', 'wp_0'), default='auto',
                   help="come localizzarsi sul grafo caricato (default: fiducial, poi wp_0)")
    p.add_argument('--con-scansioni', action='store_true',
                   help="aggiunge alle mappe .pkl le scansioni salvate in scans/ (piu' completo, piu' lento)")
    p.add_argument('--senza-ripiego', action='store_true',
                   help="niente ripiego su GraphNav quando il PRM non ce la fa")
    p.add_argument('--hostname', default='192.168.80.3')
    return p.parse_args(argv)


def _obiettivo_da_testo(mappa, s):
    s = s.strip()
    if re.fullmatch(r'wp_\d+', s):
        return mappa.obiettivo_vision(waypoint=s)
    if s.lower().startswith('c'):
        return mappa.obiettivo_vision(cella=_coppia(s[1:], int))
    return mappa.obiettivo_vision(verso=_coppia(s))


def anteprima(args):
    """Senza robot: piano dalla partenza (o da --da) all'obiettivo, e figura."""
    mappa = MappaMissione.carica(args.missione, con_scansioni=args.con_scansioni)
    da = mappa.frame.a_vision(*(args.da or (0.0, 0.0)))
    if args.clic:
        scelta = SceltaConClic(mappa, da, titolo=f"Missione {mappa.name} (senza robot)")
        if scelta.scegli() is None:
            print(f"{LOG_TAG} Nessun obiettivo confermato.")
            return False
        goal = scelta.goal
        descr = "il punto scelto con un clic"
    else:
        goal, descr = mappa.obiettivo_vision(args.verso, args.cella, args.waypoint)
    piano = mappa.pianifica(da, goal)
    if piano is None:
        print(f"{LOG_TAG} Nessun percorso verso {descr}.")
    else:
        print(f"{LOG_TAG} Piano verso {descr}: {len(piano['path']) - 1} archi, {piano['lunghezza']:.1f} m "
              f"(linea d'aria {np.hypot(goal[0] - da[0], goal[1] - da[1]):.1f} m), arrivo a "
              f"{piano['mancano_m']:.2f} m dall'obiettivo.")
    mappa.anteprima(args.anteprima, piano, da, goal, titolo=f"Missione {mappa.name}: verso {descr}")
    return piano is not None


def sul_robot(args):
    """Collegamento, grafo su Spot, localizzazione, uno o piu' obiettivi, seduta."""
    import bosdyn.client.lease
    from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
    import spotLogInUtils
    from registro import MISSION_CTX, install_stdout_tee, remove_stdout_tee

    mappa = MappaMissione.carica(args.missione, con_scansioni=args.con_scansioni)
    goals = []
    if not (args.interattivo or args.clic):
        goals.append(mappa.obiettivo_vision(args.verso, args.cella, args.waypoint))

    log_folder = os.path.join(HERE, 'MissionLogs', f"Naviga_{datetime.now().strftime('%d-%m-%Y_%H-%M-%S')}")
    os.makedirs(log_folder, exist_ok=True)
    MISSION_CTX['log_folder'] = log_folder
    install_stdout_tee(os.path.join(log_folder, 'naviga_log.txt'))
    print(f"{LOG_TAG} Mappa della missione {mappa.name}; log in {log_folder}")

    options = SimpleNamespace(name="navigaMappa", hostname=args.hostname, verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, lease_client, rsc, client_metadata = spotLogInUtils.setLogInfo(options)
    estop = spotLogInUtils.SimpleEstop(robot, options.name + "_estop")
    try:
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            cc = robot.ensure_client(RobotCommandClient.default_service_name)
            robot.power_on()
            assert robot.is_powered_on(), "Accensione del robot non riuscita."
            blocking_stand(cc)
            nav = NavigatoreRobot(mappa, robot, rsc, cc, client_metadata, log_folder)
            nav.carica_grafo()
            if not nav.localizza(args.localizzazione):
                print(f"{LOG_TAG} Non localizzato sulla mappa: non mi muovo.")
                return False
            nav.tracker.start()
            try:
                esito = True
                while True:
                    if args.clic:
                        _new, qui = nav.posa_robot()
                        r = SceltaConClic(mappa, qui, titolo=f"Missione {mappa.name}: dove vado?",
                                          etichetta_da='robot adesso').scegli()
                        if r is None:
                            break
                        goals = [r]
                    elif args.interattivo:
                        try:
                            s = input("\nObiettivo (X,Y avanti/sinistra | c R,C | wp_N | q per finire): ")
                        except EOFError:
                            break
                        if s.strip().lower() in ('q', 'quit', 'esci', ''):
                            break
                        try:
                            goals = [_obiettivo_da_testo(mappa, s)]
                        except Exception as e:
                            print(f"{LOG_TAG} Obiettivo non valido: {e}")
                            continue
                    if not goals:
                        break
                    goal, descr = goals.pop(0)
                    esito = nav.vai(goal, descr, ripiego_graphnav=not args.senza_ripiego)
                    if not esito:
                        print(f"{LOG_TAG} Obiettivo {descr} NON raggiunto.")
                    if not (args.interattivo or args.clic):
                        break
                return esito
            finally:
                nav.tracker.stop()
                try:
                    cc.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
                    time.sleep(3)
                    robot.power_off(cut_immediately=False)
                except Exception as e:
                    print(f"{LOG_TAG} Seduta/spegnimento non riusciti: {e}")
    finally:
        try:
            estop.stop()
        except Exception:
            pass
        remove_stdout_tee()


def main(argv=None):
    args = _parse_args(argv)
    if args.clic and not _finestra_disponibile():
        return False
    if args.anteprima:
        if not (args.verso or args.cella or args.waypoint or args.clic):
            print("Con --anteprima serve un obiettivo: --verso, --cella, --waypoint o --clic.")
            return False
        return anteprima(args)
    if not (args.verso or args.cella or args.waypoint or args.interattivo or args.clic):
        print("Serve un obiettivo (--verso, --cella, --waypoint) oppure --interattivo o --clic.")
        return False
    return sul_robot(args)


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
