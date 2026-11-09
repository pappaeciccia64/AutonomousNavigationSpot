"""
A finished mission, read back for map navigation (no robot needed).

  mission_config         origin, heading, cell grid and graph of the mission, from its log
                         (old Italian logs and new English ones); a clear error when the log
                         does not say them, never silent defaults
  load_occupancy_map, load_terrain_map   the global maps saved at the end of the mission
  load_graphnav_graph    the downloaded GraphNav graph, with the pose of every waypoint in the
                         mission's VISION frame (from its snapshot)
  MissionMap             frame, maps, mission PRM limited to the seen area, planning, drawing
  mission_raster         the occupancy map as an image in the mission frame

HOW THE THREE PIECES ARE LINKED
  - Global maps and PRM are in the VISION frame OF THE MISSION (the robot's, booted that
    day): after a reboot that frame no longer exists.
  - The downloaded GraphNav graph has, in the snapshot of each waypoint, the robot state at
    creation: from there the waypoint pose in the OLD VISION frame (old_waypoint_pose).
  - Today the graph is uploaded to Spot and the robot localizes on it; the old pose of the
    waypoint it is localized on gives the transform old VISION -> today's VISION (see
    navigator.MapNavigator.update_transform).
"""
import contextlib
import glob
import io
import os
import pickle
import re
import time

import numpy as np

from spot_explorer import REPO_ROOT
from spot_explorer import config as P
from spot_explorer.legacy import modern
from spot_explorer.mission import cells
from spot_explorer.mission import mission_prm
from spot_explorer.navigation.frames import MissionFrame, yaw_of_quat
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.planning import roadmap

LOG_TAG = "[MAPNAV]"
# An edge is used only if it was seen along at least this fraction of its length.
MIN_SEEN_FRACTION = 0.9
# GraphNav graph folders: Graph_* since 30-10-2026, Grafo_* before.
GRAPH_FOLDER_PATTERNS = ('Graph_*', 'Grafo_*')


def quiet():
    return contextlib.redirect_stdout(io.StringIO())


# =========================================================================================
# READING THE MISSION
# =========================================================================================
def _graph_folders(name):
    gdirs = []
    for pattern in GRAPH_FOLDER_PATTERNS:
        gdirs += glob.glob(os.path.join(REPO_ROOT, 'graph', f'Mission_{name}', pattern))
    return sorted((d for d in gdirs if os.path.exists(os.path.join(d, 'graph'))), key=os.path.getmtime)


def latest_complete_mission():
    """The latest mission with global maps, log and GraphNav graph."""
    cands = []
    for d in glob.glob(os.path.join(REPO_ROOT, 'MissionMap', 'Mission_*')):
        name = os.path.basename(d)[len('Mission_'):]
        pkl = os.path.join(d, 'global_occupancy_map.pkl')
        if (os.path.exists(pkl) and _graph_folders(name)
                and os.path.exists(os.path.join(REPO_ROOT, 'MissionLogs', f'Mission_{name}', 'mission_log.txt'))):
            cands.append((os.path.getmtime(pkl), name))
    return max(cands)[1] if cands else None


def mission_config(name):
    """
    Origin, heading, cell grid and graph of the mission, from its log: the same lines the
    replay reads (tests/test_replay.py mission_setup), in both languages of the logs. The
    PRM rebuilt here must be the mission's one, node by node: a line that is missing raises
    ValueError, because a default would give another graph on another area without telling.
    Only the lattice spacing and the starting cell have a default, for the missions that
    did not write them (all lattice at 0.25 m before 22-10, always cell (0, 0)).
    """
    log_path = os.path.join(REPO_ROOT, 'MissionLogs', f'Mission_{name}', 'mission_log.txt')
    if not os.path.exists(log_path):
        raise FileNotFoundError(f"the mission log is missing: {log_path}")
    log = open(log_path, errors='replace').read()

    def need(match, what):
        if match is None:
            raise ValueError(f"{what} not found in {log_path}: cannot rebuild the mission graph")
        return match

    m = need(re.search(r'(?:Origin set at world coordinates|Origine fissata alle coordinate) '
                       r'\(([-\d.]+), ([-\d.]+)\)', log), "mission origin")
    a = need(re.search(r'(?:Initial orientation|Orientamento iniziale): ([-\d.]+)', log), "initial heading")
    g = need(re.search(r'(?:Griglia di missione|Mission grid): (\d+) x (\d+) (?:celle da|cells of) ([\d.]+) m',
                       log), "mission grid ([CONFIG] Mission grid)")
    sc = re.search(r'(?:Starting cell|Cella di partenza) \((\d+), (\d+)\)', log)
    s = re.search(r'(?:Reticolo: passo richiesto|Lattice: requested spacing) ([\d.]+) m', log)
    gr = re.search(r'\[CONFIG\] (?:Grafo: nodi|Graph: nodes) (\w+).*; (?:archi da|edges from) ([\d.]+) '
                   r'(?:a|to) ([\d.]+) m', log)
    if gr is None and re.search(r'\[CONFIG\] (?:Grafo|Graph):', log):
        need(None, "graph line ([CONFIG] Graph: nodes ...) in a known format")
    # Before 22-10 there was no graph line: lattice, edges 0.5-1.25 m.
    sampling, min_edge, max_edge = ((modern(gr.group(1)), float(gr.group(2)), float(gr.group(3)))
                                    if gr else ('lattice', 0.5, 1.25))
    rnd = re.search(r"\[(?:GRAFO|GRAPH)\] (?:Nodi casuali: distribuzione|Random nodes: distribution) (\w+), "
                    r"(?:densita'|density) ([\d.]+) (?:nodi|nodes)/m2, (?:seme|seed) (\d+)", log)
    if sampling == 'random':
        need(rnd, "random nodes line ([GRAPH] Random nodes: distribution ..., seed ...)")
    return dict(
        name=name, origin=(float(m.group(1)), float(m.group(2))), yaw=np.radians(float(a.group(1))),
        rows=int(g.group(1)), cols=int(g.group(2)), cell=float(g.group(3)),
        start_cell=(int(sc.group(1)), int(sc.group(2))) if sc else (0, 0),
        spacing=float(s.group(1)) if s else 0.25,
        sampling=sampling, min_edge=min_edge, max_edge=max_edge,
        distribution=modern(rnd.group(1)) if rnd else None,
        density=float(rnd.group(2)) if rnd else None,
        seed=int(rnd.group(3)) if rnd else None)


def load_occupancy_map(path):
    """GlobalGrid from the end-of-mission .pkl (GlobalGrid.save_map)."""
    with open(path, 'rb') as f:
        d = pickle.load(f)
    gm = global_maps.GlobalGrid(resolution=d['resolution'])
    for k, v in d['grid'].items():
        gm._set_state(tuple(k), float(v))     # keeps the coarse index aligned
    for x, y, r in d.get('hazards', []):
        gm.mark_hazard(x, y, r)
    gm.walked_cells = set(tuple(k) for k in d.get('walked_cells', []))
    for key in ('clear_near_sum', 'clear_near_n', 'clear_all_sum', 'clear_all_n'):
        setattr(gm, key, dict(d.get(key, {})))
    return gm


def prepare_hysteresis(gm):
    """
    The .pkl does not save the observation counters (obs_count): without them, an occupied
    cell of the mission would turn free at the first scan of today that sees it free. The
    counter of a just-confirmed obstacle is put back (GLOBAL_OCC_CONFIRM_OBS): to disappear
    it takes as many free observations as an obstacle needs to appear. Cells that already
    have a counter stay as they are.
    """
    n = 0
    for k, v in gm.grid.items():
        if v == -1.0 and k not in gm.obs_count:
            gm.obs_count[k] = global_maps.GLOBAL_OCC_CONFIRM_OBS
            n += 1
    return n


def load_terrain_map(path):
    """GlobalTerrainGrid from the end-of-mission .pkl (GlobalTerrainGrid.save_map)."""
    with open(path, 'rb') as f:
        d = pickle.load(f)
    tm = global_maps.GlobalTerrainGrid(resolution=d['resolution'])
    tm.height_sum = dict(d['height_sum'])
    tm.height_count = dict(d['height_count'])
    tm.mission_z0 = d.get('mission_z0')
    keys = np.array([k for k, c in tm.height_count.items() if c > 0], dtype=np.float64)
    r = tm.resolution
    tm.update_rects = ([(keys[:, 0].min() * r, (keys[:, 0].max() + 1) * r,
                         keys[:, 1].min() * r, (keys[:, 1].max() + 1) * r)] if keys.size else [])
    return tm


def _scan_arrays(d):
    """The quantities of a saved scan bundle that the global maps need (as in the replay)."""
    z = d['terrain_corrected']
    ny, nx = z.shape
    ox, oy = map(float, d['grid_origin'])
    cs = float(d['cell_size'])
    r, c = np.mgrid[0:ny, 0:nx]
    pts = np.stack([ox + (c + .5) * cs, oy + (r + .5) * cs, z], -1).reshape(-1, 3)
    return dict(z=z, nx=nx, ny=ny, pts=pts,
                od_raw=np.asarray(d['obstacle_distance'], dtype=np.float64).reshape(z.shape),
                valid=np.asarray(d['is_valid'], dtype=bool).reshape(z.shape),
                obstacle_mask=np.asarray(d['obstacle_mask']).ravel(),
                robot=(float(d['robot_xyz'][0]), float(d['robot_xyz'][1]), float(d['robot_yaw'])))


def add_saved_scans(name, gm, tm):
    """Folds into the maps also the saved scan bundles (scans/), with the mission's own
    function. The .pkl files are saved at every START of an attempt: the loop scans of the
    last attempt are only here. Returns how many were used."""
    files = glob.glob(os.path.join(REPO_ROOT, 'MissionMap', f'Mission_{name}', 'scans', '*.npz'))
    files.sort(key=lambda f: float(np.load(f)['time']))
    for f in files:
        a = _scan_arrays(np.load(f))
        global_maps.fold_scan_into_global_maps(gm, tm, a['pts'], a['z'].ravel(), a['obstacle_mask'],
                                               a['valid'].ravel(), a['od_raw'].ravel(),
                                               (a['ny'], a['nx']), a['robot'])
    return len(files)


def old_waypoint_pose(waypoint, snapshot):
    """
    (x, y, yaw) of the waypoint frame in the mission's VISION frame. Exact: vision_T_odom
    from the snapshot (same instant, same transform snapshot) times odom_T_waypoint (the
    inverse of waypoint_tform_ko, which the graph carries). If something is missing, the
    body pose in the snapshot (like mission_viewer and graphnav_log_drift).
    """
    from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                             get_a_tform_b)
    from bosdyn.client.math_helpers import SE3Pose
    ts = snapshot.robot_state.kinematic_state.transforms_snapshot
    v_T_body = get_a_tform_b(ts, VISION_FRAME_NAME, BODY_FRAME_NAME)
    o_T_body = get_a_tform_b(ts, ODOM_FRAME_NAME, BODY_FRAME_NAME)
    if v_T_body is not None and o_T_body is not None and waypoint.HasField('waypoint_tform_ko'):
        v_T_wp = v_T_body * o_T_body.inverse() * SE3Pose.from_proto(waypoint.waypoint_tform_ko).inverse()
        return float(v_T_wp.x), float(v_T_wp.y), yaw_of_quat(v_T_wp.rotation)
    if v_T_body is not None:
        return float(v_T_body.x), float(v_T_body.y), yaw_of_quat(v_T_body.rotation)
    return None


def load_graphnav_graph(name):
    """
    The GraphNav graph downloaded at the end of the mission (the latest Graph_*/Grafo_*),
    with the old pose of every waypoint. dict: dir, graph (proto), wp {id: {name, x, y, yaw}},
    by_name {name: id}.
    """
    from bosdyn.api.graph_nav import map_pb2
    gdirs = _graph_folders(name)
    if not gdirs:
        raise FileNotFoundError(f"no GraphNav graph in graph/Mission_{name}/Graph_* (or Grafo_*)")
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
        pose = old_waypoint_pose(w, snap)
        if pose is None:
            missing += 1
            continue
        wps[w.id] = dict(name=w.annotations.name or w.id[:12], x=pose[0], y=pose[1], yaw=pose[2])
    by_name = {v['name']: k for k, v in wps.items()}
    print(f"{LOG_TAG} GraphNav graph {os.path.basename(gdir)}: {len(graph.waypoints)} waypoints "
          f"({len(wps)} with a known pose, {missing} without snapshot), {len(graph.edges)} edges.")
    return dict(dir=gdir, graph=graph, wp=wps, by_name=by_name)


# =========================================================================================
# THE MISSION PRM ON THE FINAL MAP
# =========================================================================================
def _set_mission_graph_parameters(cfg):
    """The same node parameters as the mission (same seed: same nodes, same ids)."""
    P.MISSION_LATTICE_SPACING_M = cfg['spacing']
    P.MISSION_GRAPH_SAMPLING = cfg['sampling']
    P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = cfg['min_edge'], cfg['max_edge']
    if cfg['sampling'] == 'random':
        P.MISSION_RANDOM_DISTRIBUTION = cfg['distribution']
        P.MISSION_RANDOM_DENSITY = cfg['density']
        P.MISSION_RANDOM_SEED = cfg['seed']


def _seen_fraction(gm, x1, y1, x2, y2, step=0.05):
    n = max(2, int(np.ceil(np.hypot(x2 - x1, y2 - y1) / step)) + 1)
    t = np.linspace(0.0, 1.0, n)
    return float(np.mean([gm.is_known(x1 + s * (x2 - x1), y1 + s * (y2 - y1)) for s in t]))


def remove_edge(prm, a, b):
    """Removes an edge WITHOUT making it permanent (unlike mark_edge_invalid): the next
    evaluation with new data can put it back."""
    prm.edge_validity[(min(a, b), max(a, b))] = False
    prm.edges[a] = [(n, w) for n, w in prm.edges.get(a, []) if n != b]
    prm.edges[b] = [(n, w) for n, w in prm.edges.get(b, []) if n != a]


def limit_to_seen_area(prm, gm, min_fraction=MIN_SEEN_FRACTION, rect=None):
    """
    The mission PRM covers the WHOLE area, and an edge on ground never seen has no veto (no
    data = no reason to drop it). Here navigation stays on what was seen: nodes on observed
    cells, edges observed along at least min_fraction of their length. rect (xmin, xmax,
    ymin, ymax): only the nodes and edges touching it (after a map update); None = the whole
    graph. Not permanent: when the robot sees a new area, its edges come back.
    Returns (allowed nodes, edges removed).
    """
    def touches(x1, y1, x2, y2):
        return rect is None or (max(x1, x2) >= rect[0] and min(x1, x2) <= rect[1]
                                and max(y1, y2) >= rect[2] and min(y1, y2) <= rect[3])
    allowed = set(prm.allowed_nodes) if (rect is not None and prm.allowed_nodes is not None) else set()
    for nid, (x, y) in prm.nodes.items():
        if touches(x, y, x, y):
            if gm.is_known(x, y):
                allowed.add(nid)
            else:
                allowed.discard(nid)
    removed = 0
    for a in list(prm.edges):
        for b, _w in list(prm.edges.get(a, ())):
            if a >= b or not touches(*prm.nodes[a], *prm.nodes[b]):
                continue
            if (a not in allowed or b not in allowed
                    or _seen_fraction(gm, *prm.nodes[a], *prm.nodes[b]) < min_fraction):
                remove_edge(prm, a, b)
                removed += 1
    prm.set_allowed_nodes(allowed)
    return sorted(allowed), removed


class MissionMap:
    """Everything needed to plan, without the robot: frame, maps, PRM, GraphNav graph."""

    def __init__(self, name, cfg, gm, tm, ng):
        """From ready pieces (the tests build them in memory); from files: load()."""
        self.name, self.cfg, self.gm, self.tm, self.ng = name, cfg, gm, tm, ng
        self.frame = MissionFrame(cfg['origin'][0], cfg['origin'][1], cfg['yaw'])
        prepare_hysteresis(gm)
        self._build_prm()

    @classmethod
    def load(cls, name=None, with_scans=False, without_graphnav=False, maps_folder=None):
        """The mission from its files: log, .pkl maps (and scans), downloaded GraphNav graph.
        maps_folder: the .pkl files from another folder, for example those updated by a
        previous navigation (MissionLogs/Navigation_<date>/); same frame as the mission."""
        name = name or latest_complete_mission()
        if name is None:
            raise FileNotFoundError("no mission with global maps, log and GraphNav graph")
        name = name[len('Mission_'):] if name.startswith('Mission_') else name
        cfg = mission_config(name)
        folder = maps_folder or os.path.join(REPO_ROOT, 'MissionMap', f'Mission_{name}')
        if maps_folder:
            print(f"{LOG_TAG} Maps from {maps_folder} (not the end-of-mission ones).")
        gm = load_occupancy_map(os.path.join(folder, 'global_occupancy_map.pkl'))
        tm = load_terrain_map(os.path.join(folder, 'global_terrain_map.pkl'))
        n_scan = 0
        if with_scans:
            with quiet():
                n_scan = add_saved_scans(name, gm, tm)
        n_occ = sum(1 for v in gm.grid.values() if v == -1.0)
        print(f"{LOG_TAG} Mission {name}: {cfg['rows']} x {cfg['cols']} cells of "
              f"{cfg['cell']:g} m, {cfg['sampling']} nodes"
              + (f" (seed {cfg['seed']})" if cfg['seed'] is not None else "")
              + f"; map: {len(gm.grid)} cells seen, {n_occ} occupied"
              + (f", plus {n_scan} saved scans" if with_scans else "") + ".")
        ng = None if without_graphnav else load_graphnav_graph(name)
        return cls(name, cfg, gm, tm, ng)

    def _build_prm(self):
        t0 = time.perf_counter()
        cfg = self.cfg
        with quiet():
            self.env = cells.EnvironmentMap(rows=cfg['rows'], cols=cfg['cols'], cell_size=cfg['cell'])
            self.env.set_origin(cfg['origin'][0], cfg['origin'][1], cfg['yaw'],
                                *cfg.get('start_cell', (0, 0)))
            _set_mission_graph_parameters(cfg)
            self.prm, _sampler, self.mg = mission_prm.build_mission_prm(self.env, verbose=False)
            self.prm.set_global_terrain_map(self.tm)
            mission_prm.rebuild_prm_edges(self.prm, self.gm, incremental=False)
        # Far goals: no detour cap (in the mission it stops crossing half the area to enter a
        # nearby cell; here the long way is the right one).
        self.prm.set_detour_limit(None)
        allowed, removed = limit_to_seen_area(self.prm, self.gm)
        n_edges = sum(len(v) for v in self.prm.edges.values()) // 2
        print(f"{LOG_TAG} PRM: {len(self.prm.nodes)} nodes, {len(allowed)} on seen ground; {n_edges} "
              f"valid edges ({removed} removed as on unseen ground) in {time.perf_counter() - t0:.1f} s.")

    # --- goals -----------------------------------------------------------------------------
    def goal_vision(self, to=None, cell=None, waypoint=None):
        """The goal in the mission's VISION frame, from the three ways of giving it, and a
        description."""
        if to is not None:
            x, y = self.frame.to_vision(*to)
            return (x, y), f"({to[0]:.2f}, {to[1]:.2f}) in the mission frame"
        if cell is not None:
            r, c = cell
            if not (0 <= r < self.env.rows and 0 <= c < self.env.cols):
                raise ValueError(f"cell ({r},{c}) is outside the {self.env.rows} x {self.env.cols} grid")
            return tuple(self.env.get_world_position_from_cell(r, c)), f"centre of cell ({r},{c})"
        if waypoint is not None:
            if self.ng is None or waypoint not in self.ng['by_name']:
                raise ValueError(f"waypoint {waypoint} not found in the GraphNav graph")
            w = self.ng['wp'][self.ng['by_name'][waypoint]]
            return (w['x'], w['y']), f"waypoint {waypoint}"
        raise ValueError("a goal is needed: --to, --cell or --waypoint")

    def _component(self, start):
        seen, stack = {start}, [start]
        allowed = self.prm.allowed_nodes
        while stack:
            u = stack.pop()
            for v, _w in self.prm.edges.get(u, ()):
                if v not in seen and (allowed is None or v in allowed):
                    seen.add(v)
                    stack.append(v)
        return seen

    def plan(self, from_xy, to_xy):
        """
        Path on the PRM from from_xy to to_xy (mission VISION). The goal node is the allowed
        node nearest to the goal AMONG THOSE REACHABLE from the start: a goal inside an
        obstacle or in an unconnected area brings the robot as close as possible, and the log
        says how far it stays. Returns dict (path, start, goal, length, missing_m) or None.
        """
        start = self.prm.anchor_node(from_xy[0], from_xy[1], self.gm, geometry.PRM_EDGE_SAFETY_MARGIN_M,
                                     ignore_near_m=roadmap.ANCHOR_IGNORE_NEAR_M)
        if start is None:
            return None
        comp = self._component(start)
        if len(comp) <= 1:
            print(f"{LOG_TAG} The start node {start} has no valid edge towards seen ground.")
            return None
        ids = np.array(sorted(comp))
        xy = np.array([self.prm.nodes[int(i)] for i in ids])
        d = np.hypot(xy[:, 0] - to_xy[0], xy[:, 1] - to_xy[1])
        goal = int(ids[int(np.argmin(d))])
        path = self.prm.find_path_dijkstra(start, goal)
        if path is None:
            return None
        return dict(path=path, start=start, goal=goal, length=self.prm.path_length(path),
                    missing_m=float(d.min()))

    def nearest_waypoint(self, xy):
        if not self.ng or not self.ng['wp']:
            return None
        return min((np.hypot(w['x'] - xy[0], w['y'] - xy[1]), wid) for wid, w in self.ng['wp'].items())[1]

    # --- drawing ---------------------------------------------------------------------------
    # Everything in the mission frame: in the figure the horizontal axis is y (to the left of
    # the start, inverted: the robot's left on the left) and the vertical one is x (ahead).
    # A point (xm, ym) is drawn at (ym, xm).
    def draw_map(self, ax):
        """Background: map (red occupied, green seen free), PRM edges on seen ground,
        GraphNav waypoints."""
        from matplotlib.collections import LineCollection
        fr = self.frame
        img, ext = mission_raster(self.gm, fr)
        draw_raster(ax, img, ext)
        ax.plot([], [], 's', c='#d0342c', label='occupied')
        ax.plot([], [], 's', c='#b8dfb0', label='seen free')
        segs = []
        for a, lst in self.prm.edges.items():
            for b, _w in lst:
                if a < b:
                    (x1, y1), (x2, y2) = fr.from_vision(*self.prm.nodes[a]), fr.from_vision(*self.prm.nodes[b])
                    segs.append([(y1, x1), (y2, x2)])
        ax.add_collection(LineCollection(segs, colors='#888888', linewidths=0.3, alpha=0.5, zorder=3))
        if self.ng and self.ng['wp']:
            w = np.array([fr.from_vision(v['x'], v['y']) for v in self.ng['wp'].values()])
            ax.scatter(w[:, 1], w[:, 0], s=8, c='#2b6cb0', zorder=4, label='GraphNav waypoints')
        ax.set_aspect('equal')
        ax.set_xlim(ext[1] + 0.5, ext[0] - 0.5)       # inverted: the robot's left on the left
        ax.set_ylim(ext[2] - 0.5, ext[3] + 0.5)
        ax.set_xlabel("y: to the left of the start [m]")
        ax.set_ylabel("x: ahead of the start [m]")
        ax.grid(alpha=0.3)

    def draw_plan(self, ax, plan=None, from_xy=None, to_xy=None, from_label='start'):
        """Path, start and goal (mission VISION points). Returns the drawn artists, to remove
        when the plan changes."""
        fr = self.frame
        out = []
        if plan is not None:
            p = np.array([fr.from_vision(*self.prm.nodes[n]) for n in plan['path']])
            out += ax.plot(p[:, 1], p[:, 0], '-', c='#ff8c00', lw=2.5, zorder=5,
                           label=f"PRM path {plan['length']:.1f} m")
        if from_xy is not None:
            m = fr.from_vision(*from_xy)
            out += ax.plot(m[1], m[0], 'o', c='k', ms=9, zorder=6, label=from_label)
        if to_xy is not None:
            m = fr.from_vision(*to_xy)
            out += ax.plot(m[1], m[0], '*', c='#ffd400', mec='k', ms=18, zorder=6, label='goal')
        return out

    def preview(self, png, plan=None, from_xy=None, to_xy=None, title=""):
        """The figure of the plan in a file (no window)."""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 10))
        self.draw_map(ax)
        self.draw_plan(ax, plan, from_xy, to_xy)
        ax.set_title(title or f"Mission {self.name}")
        ax.legend(loc='upper right', fontsize=8)
        fig.tight_layout()
        fig.savefig(png, dpi=130)
        plt.close(fig)
        print(f"{LOG_TAG} Preview saved to {png}")


def mission_raster(gm, fr):
    """
    The occupancy map as an image in the mission frame (rotated with respect to VISION, same
    resolution): 0 never seen, 1 seen free, 2 occupied. Rows = x (ahead), columns = y (to the
    left). Returns (image, extent for imshow in (y, x)).
    """
    r = gm.resolution
    keys = np.array(list(gm.grid.keys()), dtype=np.int64)
    if not keys.size:
        return np.zeros((1, 1), dtype=np.uint8), (0.0, r, 0.0, r)
    vals = np.array(list(gm.grid.values()))
    gx0, gy0 = keys.min(axis=0)
    ras = np.zeros(tuple(keys.max(axis=0) - (gx0, gy0) + 1), dtype=np.uint8)
    ras[keys[:, 0] - gx0, keys[:, 1] - gy0] = np.where(vals > 0, 1, 2)
    mx, my = fr.from_vision((keys[:, 0] + 0.5) * r, (keys[:, 1] + 0.5) * r)
    xs = np.arange(mx.min(), mx.max() + r, r)
    ys = np.arange(my.min(), my.max() + r, r)
    XM, YM = np.meshgrid(xs, ys, indexing='ij')
    vx, vy = fr.to_vision(XM, YM)
    ix = np.floor(vx / r).astype(np.int64) - gx0
    iy = np.floor(vy / r).astype(np.int64) - gy0
    ok = (ix >= 0) & (ix < ras.shape[0]) & (iy >= 0) & (iy < ras.shape[1])
    img = np.zeros(XM.shape, dtype=np.uint8)
    img[ok] = ras[ix[ok], iy[ok]]
    return img, (float(ys[0] - r / 2), float(ys[-1] + r / 2), float(xs[0] - r / 2), float(xs[-1] + r / 2))


def draw_raster(ax, img, ext):
    from matplotlib.colors import ListedColormap
    return ax.imshow(img, origin='lower', interpolation='nearest', zorder=1,
                     cmap=ListedColormap(['#ffffff', '#b8dfb0', '#d0342c']), vmin=0, vmax=2, extent=ext)


def describe_plan(plan, from_xy, to_xy):
    if plan is None:
        return "no path on seen ground"
    return (f"{len(plan['path']) - 1} edges, {plan['length']:.1f} m (straight line "
            f"{np.hypot(to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]):.1f} m)"
            + (f", arriving {plan['missing_m']:.2f} m from the goal" if plan['missing_m'] > 0.05 else ""))
