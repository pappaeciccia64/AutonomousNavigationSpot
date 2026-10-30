#!/usr/bin/env python3
"""
Mission data viewer. Runs on any computer: it only needs numpy and matplotlib, NOT the
Spot SDK.

During the mission the robot draws no figures (see visualize_grid_with_candidates in
records.py): it saves the raw data, and the figures are made here, afterwards.

    python3 tools/mission_viewer.py                                  # (or Run key) the latest mission in MissionMap/
    python3 tools/mission_viewer.py <mission_folder>                 # interactive, all scans
    python3 tools/mission_viewer.py 13-10-2026_12-40-38              # or just the mission name
    python3 tools/mission_viewer.py <file.npz>                       # a scan or a PRM view
    python3 tools/mission_viewer.py <mission_folder> --png <dir>     # export all figures as PNG
    python3 tools/mission_viewer.py <mission_folder> --overview      # only the mission overview
    ... --background terreno | fusione                               # initial background of the global panel
    ... --navgraph                                                   # nav graph visible from the start
    ... --mission-frame                                              # global panel in the mission frame

Keys (interactive mode):
    On the left the scan (layer of choice), on the right the GLOBAL MAP at that moment:
    mosaic of the scans made so far (red obstacle, light green free, white never seen),
    mission cells (green visited, red blocked), PRM graph coloured by slope
    (green forward, red sideways), real path, planned path, current window (light
    blue), blocks (red squares).
    right / left arrow           next / previous scan
    up / down arrow              10 scans forward / back
    1..9, 0                      layer of the left panel (see LAYERS)
    i / x                        Spot's intensity / no_step layer (missions since 26-10)
    l                            next layer
    o                            right panel: end of mission <-> moment of the scan
    g                            PRM graph over the scan: show / hide
    m                            right panel: the robot's map (confirmed obstacles, default)
                                 <-> raw mosaic (the last scan wins)
    t                            right panel background: obstacles -> terrain -> fusion
                                 (terrain coloured by height, obstacles in red on top;
                                 white line = height 0, the ground at mission start, also
                                 marked on the bar)
    r                            both panels in the MISSION frame: cell grid straight, start
                                 at the origin, robot nose at the start pointing up (under the
                                 mouse the VISION coordinates are still shown)
    b / B                        next / previous block
    c / C                        first scan of the next / previous attempt (cell)
    v / V                        first scan after the next / previous GraphNav move
                                 (or unrecorded leg, in missions before 26-10)
    e                            legends: show / hide
    n                            GraphNav nav graph on the right panel: show / hide
                                 (needs the Spot SDK to read it, see NAV GRAPH)
    the zoom of the right panel is kept when the scan or background changes
    the value under the mouse appears at the bottom right, with the VISION coordinates

Files read:
    scans/scan_*.npz    one bundle per scan of the navigation loop
    *_vis.npz           instead of the old figures: data + PRM graph at that moment
    *_BLOCKED_*.npz     the data that decided a block
"""
import glob
import os
import re
import pickle
import sys

import threading
import numpy as np

PNG_MODE = "--png" in sys.argv
if PNG_MODE:
    import matplotlib
    matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.collections import LineCollection    # noqa: E402
from matplotlib.colors import ListedColormap, Normalize   # noqa: E402
import matplotlib.patches as patches                  # noqa: E402
import matplotlib.patheffects as patheffects          # noqa: E402
import csv                                            # noqa: E402
import matplotlib.patheffects as pe                   # noqa: E402
from matplotlib.transforms import Affine2D            # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # the spot_explorer package

from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import slopes
from spot_explorer.perception import spot_grid
from spot_explorer.legacy import modern

SLOPE_THRESHOLD = slopes.SLOPE_THRESHOLD    # only used to colour the edges

# GLOBAL MAP AS THE ROBOT BUILT IT. The rules of global_maps.mask_for_global_map and
# global_maps.GlobalGrid.update redone on the saved data (the scans in scans/), with the same
# constants, read from the perception package (which can be imported without the Spot SDK too).
# Before, the right panel showed the "last scan wins" mosaic: an obstacle
# read ONCE, even next to the legs, covered the cells the robot had just
# walked over, and the real path seemed to cross the obstacles, especially at the end
# (mission of 22-10, around 16:00). The robot discarded those readings or required them
# confirmed. The 'm' key goes back to the raw mosaic.
SELF_BODY_LENGTH_M = global_maps.GLOBAL_MAP_SELF_BODY_LENGTH_M
SELF_BODY_WIDTH_M = global_maps.GLOBAL_MAP_SELF_BODY_WIDTH_M
SELF_RING_M = global_maps.GLOBAL_MAP_SELF_RING_M
OCC_CONFIRM_OBS = global_maps.GLOBAL_OCC_CONFIRM_OBS
OCC_CONFIRM_WALKED = global_maps.GLOBAL_OCC_CONFIRM_WALKED
OCC_MAX_COUNT = global_maps.GLOBAL_OCC_MAX_COUNT

# Two consecutive trajectory.csv samples farther apart than this: in between there is an
# unrecorded movement (until 2026-10-26 GraphNav, cone and back-offs did not write
# the file). It is drawn dashed, not as a real path.
TRAJ_GAP_M = 0.5

# BACKGROUND OF THE GLOBAL PANEL (2026-10-26), 't' key.
#   ostacoli -- the occupancy map (red obstacle, light green free, white never seen)
#   terreno  -- the global HEIGHT map, as the robot kept it (mean of the corrected
#               heights, only reliable non-obstacle cells: see spot_grid.slope_usable_mask),
#               relative to the ground at mission start, with relief shading
#   fusione  -- the terrain, with the obstacles of the occupancy map in red on top
# The height colour scale is fixed for the whole mission, so scrolling through the
# scans the same colour always means the same height.
BACKGROUNDS = ('ostacoli', 'terreno', 'fusione')
TERRAIN_CMAP = 'viridis'                # no reds: in the fusion red belongs to obstacles only
OBSTACLE_RGB = (0.84, 0.15, 0.16)       # like MOSAIC_CMAP
NO_TERRAIN_RGB = (0.80, 0.80, 0.80)     # fusion: cell seen free but without a reliable height
OBSTACLE_THRESHOLD = geometry.OBSTACLE_THRESHOLD       # terrain mask
TERRAIN_PERCENTILES = (2, 98)           # ends of the height colour scale (with 0 always inside)
ZERO_LEVEL_COLOR = 'white'              # level line and line on the bar at height 0 (start)
ZERO_LEVEL_SMOOTH_CELLS = 5             # mean over 5 cells (15 cm) only to trace that line
HILLSHADE_STRENGTH = 0.45               # 0 = no relief shading

# The maps are built scan by scan; going back restarts from the last
# saved copy instead of from the first scan (343 scans in the mission of 22-10 16:04).
CHECKPOINT_EVERY = 25

# NAV GRAPH (2026-10-26), 'n' key. The GraphNav graph downloaded at the end of the mission in
# graph/<mission>/Graph_*/ (Grafo_* before 30-10): for every waypoint the position is that of the body in the
# VISION frame when the waypoint was created (from the snapshot), i.e. the same frame as the
# map. From the snapshots (558 MB in the mission of 22-10 16:04, almost all point clouds)
# only the robot state is read, without decoding the rest: 0.3 s instead of 51.
# Reading the Boston Dynamics format requires the SDK (bosdyn); without it, the key does nothing.
NAVGRAPH_EDGE_STYLE = {    # edge source -> (colour, width, style, label)
    'EDGE_SOURCE_ODOMETRY': ('#303030', 0.9, '-', 'nav graph: recorded edge'),
    'EDGE_SOURCE_USER_REQUEST': ('deepskyblue', 1.2, '-', 'nav graph: our shortcut'),
    'EDGE_SOURCE_LOCALIZATION': ('darkviolet', 1.2, '--', "nav graph: loop closure"),
    'EDGE_SOURCE_SMALL_LOOP_CLOSURE': ('darkviolet', 1.2, '--', "nav graph: loop closure"),
    'EDGE_SOURCE_ALTERNATE_ROUTE_FINDING': ('darkviolet', 1.2, '--', "nav graph: loop closure"),
}
NAVGRAPH_OTHER_STYLE = ('gray', 0.9, ':', 'nav graph: other edge')

REASON_COLORS = {'end_of_path': 'tab:green', 'end_of_data': 'tab:blue', 'obstacle': 'tab:red',
                 'roughness': 'tab:orange', 'slope': 'tab:purple'}

# name, function extracting the matrix, colour map, vmin, vmax, label
LAYERS = [
    ("height relative to the ground", lambda d: d['terrain_raw'] - _ground(d), 'terrain', -0.5, 0.5,
     "raw height - estimated ground (m)"),
    ("raw terrain_valid", lambda d: d['terrain_valid_raw'], 'gray_r', 0, 1, "1 = valid"),
    ("obstacle_distance", lambda d: d['obstacle_distance'], 'plasma', 0, 1.0, "distance from the obstacle (m)"),
    ("roughness", lambda d: d['roughness'], 'coolwarm', 0, 0.3, "standard deviation (m)"),
    ("gradient", lambda d: d['gradient'], 'viridis', 0, 1.0, "tan(pendenza)"),
    ("fused mask", lambda d: d['obstacle_mask'], 'RdYlGn', -1, 1, "-1 = obstacle"),
    ("cells used (is_valid)", lambda d: d['is_valid'].astype(float), 'gray_r', 0, 1,
     "0 = filled/discarded"),
    ("corrected height", lambda d: d['terrain_corrected'] - _ground(d), 'terrain', -0.5, 0.5,
     "corrected height - ground (m)"),
    ("height relative to the mission start", lambda d: d['terrain_corrected'] - _mission_z0(d), 'terrain',
     -2.0, 2.0, "corrected height - ground at mission start (m)"),
    ("cells never written", lambda d: d['unwritten'].astype(float) if 'unwritten' in d
     else np.zeros_like(d['terrain_raw']), 'gray_r', 0, 1, "1 = never written: treated as not seen"),
    # Spot layers saved since 26-10 (config.SAVE_EXTRA_LAYERS), each on its own grid
    # (<name>_origin, <name>_cell): seventh element = field name.
    ("intensity (Spot)", lambda d: _extra(d, 'intensity'), 'gray', 0, 255,
     "intensity of the sensor return", 'intensity'),
    ("no_step (Spot)", lambda d: _extra(d, 'no_step'), 'RdBu', -0.3, 0.3,
     "distance from the no-step boundary (m): < 0 = Spot does not put its feet there", 'no_step'),
]
# Layer keys: 1..9, 0 for the first ten, then these.
LAYER_KEYS = {'i': 10, 'x': 11}


def _extra(d, key):
    """An extra Spot layer, NaN if the mission did not save it."""
    if key in d:
        return np.asarray(d[key], dtype=float)
    return np.full(np.shape(d['terrain_raw']), np.nan)


def _ground(d):
    g = float(d['ground_z']) if 'ground_z' in d else np.nan
    if not np.isfinite(g):
        t = d['terrain_raw']
        g = float(np.median(t[np.isfinite(t)]))
    return g


def _mission_z0(d):
    """Ground at mission start; for old files (without the field) the scan's ground."""
    z0 = float(d['mission_z0']) if 'mission_z0' in d else np.nan
    return z0 if np.isfinite(z0) else _ground(d)


def _body_masks(d):
    """Cells under the body (core) and in the ring around it, for scan d. See SELF_*."""
    h, w = d['terrain_raw'].shape
    if 'robot_yaw' not in d:
        z = np.zeros((h, w), bool)
        return z, z
    cs = float(d['cell_size'])
    gox, goy = d['grid_origin']
    rx, ry = d['robot_xyz'][:2]
    yaw = float(d['robot_yaw'])
    X, Y = np.meshgrid(gox + (np.arange(w) + 0.5) * cs - rx, goy + (np.arange(h) + 0.5) * cs - ry)
    xb = np.abs(X * np.cos(yaw) + Y * np.sin(yaw))
    yb = np.abs(-X * np.sin(yaw) + Y * np.cos(yaw))
    hl, hw = SELF_BODY_LENGTH_M / 2.0, SELF_BODY_WIDTH_M / 2.0
    core = (xb <= hl) & (yb <= hw)
    ring = ~core & (xb <= hl + SELF_RING_M) & (yb <= hw + SELF_RING_M)
    return core, ring


# HEIGHT UNDER THE BODY (2026-10-26). At the first scan of a mission the robot has
# just stood up and the terrain under the body is "never written": in the height layers a
# white patch remained. The robot sees that spot again later, from afar: in the height layers
# of the left panel and in the terrain of the global panel those cells take the height of the
# end-of-mission terrain map, hatched. Mission of 26-10 15:11: all 90
# cells covered, 1.5 cm from the surrounding terrain. Drawing only: the robot treats them as not seen.
# Height layer -> subtracted reference (same as the layer).
QUOTA_LAYER_REF = {
    "height relative to the ground": lambda d: _ground(d),
    "corrected height": lambda d: _ground(d),
    "height relative to the mission start": lambda d: _mission_z0(d),
}


def _load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def _extent(d):
    ny, nx = d['terrain_raw'].shape
    ox, oy = d['grid_origin']
    cs = float(d['cell_size'])
    return [ox, ox + nx * cs, oy, oy + ny * cs]


def _hover(ax, mat, extent):
    ny, nx = mat.shape
    x0, x1, y0, y1 = extent
    cs = (x1 - x0) / nx

    def fmt(x, y):
        c, r = int((x - x0) / cs), int((y - y0) / cs)
        if 0 <= r < ny and 0 <= c < nx:
            return f"x={x:.2f} y={y:.2f} | row {r} col {c} | value {mat[r, c]:.4f}"
        return f"x={x:.2f} y={y:.2f}"
    ax.format_coord = fmt


def draw_scan(ax, d, layer_idx, title_prefix="", ctx=None):
    ax.clear()
    name, fn, cmap, vmin, vmax, label = LAYERS[layer_idx][:6]
    src = LAYERS[layer_idx][6] if len(LAYERS[layer_idx]) > 6 else None
    mat = np.asarray(fn(d), dtype=float)
    ext = _extent(d)
    if src is not None and f"{src}_origin" in d and src in d:
        # the layer's own grid
        (lox, loy), lcs = d[f"{src}_origin"], float(d.get(f"{src}_cell", d['cell_size']))
        ext = [lox, lox + mat.shape[1] * lcs, loy, loy + mat.shape[0] * lcs]
    if src is not None and src not in d:
        name += " -- not saved in this mission"
    # height under the body seen later (see HEIGHT UNDER THE BODY)
    hole = None
    if ctx is not None and name in QUOTA_LAYER_REF:
        q, hole = ctx.body_quota(d)
        if hole is not None:
            mat = mat.copy()
            mat[hole] = q[hole] - QUOTA_LAYER_REF[name](d)
    if name == "height relative to the mission start" and ctx is not None:
        # Same scale and colours as the global panel ('t'), fixed for the mission: a
        # fixed scale from -2 to +2 m sent the scans at the top of a climb off scale (white)
        # (29-10 10:57: +2.1 m at the end of the mission).
        norm = ctx.terrain_norm()
        cmap, vmin, vmax = TERRAIN_CMAP, norm.vmin, norm.vmax
    im = ax.imshow(mat, origin='lower', extent=ext, cmap=cmap, vmin=vmin, vmax=vmax,
                   interpolation='nearest')
    _hover(ax, mat, ext)
    if hole is not None:
        cs = float(d['cell_size'])
        xs_c = ext[0] + (np.arange(mat.shape[1]) + 0.5) * cs
        ys_c = ext[2] + (np.arange(mat.shape[0]) + 0.5) * cs
        ax.contourf(xs_c, ys_c, hole.astype(float), levels=[0.5, 1.5], colors='none', hatches=['////'])
        ax.add_patch(patches.Rectangle((0, 0), 0, 0, fill=False, hatch='////', edgecolor='k', lw=0,
                                       label='under the body: height seen later'))

    if src == 'no_step' and np.isfinite(mat).any():
        cs_l = (ext[1] - ext[0]) / mat.shape[1]
        ax.contour(ext[0] + (np.arange(mat.shape[1]) + 0.5) * cs_l, ext[2] + (np.arange(mat.shape[0]) + 0.5) * cs_l,
                   mat, levels=[0.0], colors='k', linewidths=0.9)
    ext = _extent(d)        # from here on the scan grid

    # contour of the minimum distance from the path centre
    od = d['obstacle_distance']
    clear = float(d.get('frontier_clearance', 0.30))
    xs = np.linspace(ext[0], ext[1], od.shape[1]); ys = np.linspace(ext[2], ext[3], od.shape[0])
    ax.contour(xs, ys, od, levels=[clear], colors='white', linewidths=0.8, linestyles='--')

    pl = d['polyline']
    ax.plot(pl[:, 0], pl[:, 1], '-', color='magenta', lw=2, label='percorso')
    rx, ry = d['robot_xyz'][:2]
    yaw = float(d['robot_yaw']) if 'robot_yaw' in d else np.nan
    ax.plot(rx, ry, 'o', color='blue', ms=9)
    if np.isfinite(yaw):
        ax.arrow(rx, ry, 0.35 * np.cos(yaw), 0.35 * np.sin(yaw), color='blue', width=0.02)
    # Body footprint (like the body area of the global map). At the first scan
    # of a mission the terrain under the body is often "never written" (white): the robot has
    # just stood up there and the sensors have never seen it. It is treated as not seen,
    # not as an obstacle.
    if np.isfinite(yaw):
        ax.add_patch(patches.Rectangle((rx - SELF_BODY_LENGTH_M / 2, ry - SELF_BODY_WIDTH_M / 2),
                                       SELF_BODY_LENGTH_M, SELF_BODY_WIDTH_M, angle=float(np.degrees(yaw)),
                                       rotation_point=(rx, ry), fill=False, edgecolor='blue', lw=1.2,
                                       ls='--', zorder=6))
    # The frontier checks the path EXTENDED by half a body beyond the goal (the nose,
    # see safe_frontier.compute_safe_frontier): the X is where the check ends, which with
    # 'end_of_path' is half a body beyond the commanded point. The extension is dashed.
    reason = modern(str(d['frontier_reason']))
    sx, sy = d['frontier_stop_xy']
    poly = np.asarray(d['polyline']) if 'polyline' in d else np.zeros((0, 2))
    path_len = float(d['frontier_path_len']) if 'frontier_path_len' in d else np.nan
    if np.isfinite(sx) and len(poly) >= 2 and float(d['frontier_dist']) > path_len + 1e-6:
        ax.plot([poly[-1][0], sx], [poly[-1][1], sy], ':', color='magenta', lw=2,
                label=f"extension for the nose ({float(d['frontier_dist']) - path_len:.2f} m)")
    if np.isfinite(sx):
        ax.plot(sx, sy, 'X', color=REASON_COLORS.get(reason, 'k'), ms=13, mec='white',
                label=f"end of the frontier check: {reason}")
    tx, ty = d['decision_target']
    if np.isfinite(tx):
        ax.plot(tx, ty, '*', color='lime', ms=14, mec='k', label='commanded point')
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect('equal')
    ax.set_xlabel("X [m] (VISION)"); ax.set_ylabel("Y [m] (VISION)")
    if 'extra_kind' in d:
        sub = (f"{str(d['extra_kind'])}: scan outside the loop, rebuilt from the PRM view "
               f"(no frontier or decision)")
    else:
        sub = (f"frontier {float(d['frontier_dist']):.2f} m ({reason}), advance "
               f"{float(d['allowed_advance']):.2f} m, decision: {str(d['decision_kind'])}"
               + (" [without rotating]" if bool(d.get('keep_heading', False)) else ""))
    ax.set_title(f"{title_prefix}{name}\n{sub}", fontsize=9)
    ax.legend(loc='upper left', fontsize=7, framealpha=0.8)
    return im, label


def draw_vis_bundle(ax, d):
    """The old 'Path Visualization' figures: scan, PRM graph, path, cells."""
    ax.clear()
    pts = d['pts']
    if pts.size:
        mask = d['obstacle_mask']
        ax.scatter(pts[:, 0], pts[:, 1], c=np.where(mask < 0, 1.0, 0.0), cmap='RdYlGn_r', s=1,
                   vmin=0, vmax=1, alpha=0.5)
    xy = {int(i): tuple(p) for i, p in zip(d['prm_node_ids'], d['prm_node_xy'])}
    segs = [(xy[a], xy[b]) for a, b in d['prm_edges'] if a in xy and b in xy]
    if segs:
        ax.add_collection(LineCollection(segs, colors='gray', linewidths=0.3, alpha=0.4))
    trav = [(xy[a], xy[b]) for a, b in d['prm_traversed_edges'] if a in xy and b in xy]
    if trav:
        ax.add_collection(LineCollection(trav, colors='tab:blue', linewidths=2))
    # stop nodes: only in the missions up to 08-10, before the graph stayed fixed
    stops = np.array([xy[int(i)] for i in _arr(d, 'prm_stop_nodes', ()) if int(i) in xy]).reshape(-1, 2)
    if len(stops):
        ax.plot(stops[:, 0], stops[:, 1], 'o', color='tab:orange', ms=5, label='stop nodes')
    cp = d['chosen_path']
    if len(cp):
        ax.plot(cp[:, 0], cp[:, 1], '-', color='magenta', lw=2.5, label='chosen path')
    ax.plot(*d['robot_xy'], 'o', color='blue', ms=10, label='robot')
    if np.all(np.isfinite(d['chosen_point'])):
        ax.plot(*d['chosen_point'], '*', color='green', ms=16, label='obiettivo')
    for (r, c, x, y), st in zip(d['env_cells'], d['env_cell_status']):
        ax.text(x, y, f"{int(r)},{int(c)}", fontsize=7, ha='center')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.autoscale_view()
    ax.set_title(f"Iteration {int(d['iteration'])}: {len(segs)} PRM edges, {len(stops)} stop nodes",
                 fontsize=10)
    ax.legend(fontsize=7)



# =============================================================================
# GLOBAL VIEW.
# Everything is rebuilt "as it was at that moment": the map is the mosaic of the scans
# made UP TO the scan shown, the graph is the last PRM view saved before.
# =============================================================================
MOSAIC_CMAP = ListedColormap(['#ffffff', '#dfe8d8', '#d62728'])   # unknown, free, obstacle


def _navgraph_folder(folder):
    """Most recent graph/<mission>/Graph_* (or Grafo_*), searched next to MissionMap/ and to this file."""
    name = os.path.basename(os.path.normpath(folder))
    bases = {os.path.dirname(os.path.dirname(os.path.abspath(folder))),
             os.path.dirname(os.path.dirname(os.path.abspath(__file__)))}
    cands = []
    for b in bases:
        cands += [d for d in glob.glob(os.path.join(b, "graph", name, "Gra[pf][ho]_*")) if os.path.isdir(d)]
    return max(cands, key=lambda d: os.path.basename(d)) if cands else None


def _proto_field(buf, number):
    """
    The `number` field (length-delimited) at the first level of a serialized protobuf
    message, without decoding the others: in the waypoint snapshots it allows skipping the
    point clouds. None if absent.
    """
    def varint(i):
        r = sh = 0
        while True:
            c = buf[i]; i += 1
            r |= (c & 0x7f) << sh; sh += 7
            if c < 0x80:
                return r, i
    i = 0
    while i < len(buf):
        tag, i = varint(i)
        f, wt = tag >> 3, tag & 7
        if wt == 2:
            n, i = varint(i)
            if f == number:
                return buf[i:i + n]
            i += n
        elif wt == 0:
            _, i = varint(i)
        elif wt == 1:
            i += 8
        elif wt == 5:
            i += 4
        else:
            return None
    return None


def load_navgraph(folder):
    """
    The mission nav graph: dict with 'names', 'xy' (N, 2) in VISION, 'time' (N,) creation
    instant (robot clock), 'edges' (M, 2) indices, 'source' (M,) edge source, 'folder'.
    None if the graph is missing or there is no SDK to read it.
    """
    gdir = _navgraph_folder(folder)
    if gdir is None:
        print("[viewer] nav graph: no graph/<mission>/Graph_* folder for this mission.")
        return None
    try:
        from bosdyn.api.graph_nav import map_pb2
        from bosdyn.api import robot_state_pb2
        from bosdyn.client.frame_helpers import get_a_tform_b
    except Exception:
        print("[viewer] nav graph: reading it needs the Spot SDK (bosdyn-client), not available here.")
        return None
    try:
        g = map_pb2.Graph()
        with open(os.path.join(gdir, "graph"), 'rb') as f:
            g.ParseFromString(f.read())
        fn = map_pb2.WaypointSnapshot.DESCRIPTOR.fields_by_name['robot_state'].number
        ids, names, xy, times = [], [], [], []
        for w in g.waypoints:
            path = os.path.join(gdir, "waypoint_snapshots", w.snapshot_id)
            if not os.path.exists(path):
                continue
            with open(path, 'rb') as f:
                raw = _proto_field(f.read(), fn)
            if raw is None:
                continue
            rs = robot_state_pb2.RobotState()
            rs.ParseFromString(raw)
            ks = rs.kinematic_state
            p = get_a_tform_b(ks.transforms_snapshot, 'vision', 'body')
            if p is None:
                continue
            ids.append(w.id); names.append(w.annotations.name or w.id[:12])
            xy.append((p.x, p.y))
            ts = ks.acquisition_timestamp
            times.append(ts.seconds + ts.nanos * 1e-9 if ts.seconds else np.nan)
        index = {k: n for n, k in enumerate(ids)}
        edges, source = [], []
        for e in g.edges:
            a, b = index.get(e.id.from_waypoint), index.get(e.id.to_waypoint)
            if a is None or b is None:
                continue
            edges.append((a, b))
            try:
                source.append(map_pb2.Edge.EdgeSource.Name(e.annotations.edge_source))
            except Exception:
                source.append('')
        print(f"[viewer] nav graph: {len(ids)} waypoints, {len(edges)} edges ({os.path.basename(gdir)}).")
        return {'names': np.array(names), 'xy': np.array(xy, float).reshape(-1, 2),
                'time': np.array(times, float), 'edges': np.array(edges, int).reshape(-1, 2),
                'source': np.array(source), 'folder': gdir}
    except Exception as e:
        print(f"[viewer] nav graph not readable ({gdir}): {e}")
        return None


_VIS_NAME = re.compile(r'iteration_(\d+)(?:_cell_\d+_\d+)?(?:_replan_(\d+)|_(BLOCKED)|_(FINAL_SCAN))?_vis\.npz$')


def _vis_times(vis, scans):
    """
    Instant of every PRM view, to match it with the scans (see vis_before), and sorted list.
    Since 2026-10-26 the view has the 'time' field. Before it did not, and the file date is
    no use: after a git clone or a pull it is the download date, the same for all, and the
    first view was always shown. So the instant is rebuilt from the name and position:
      iteration_I_cell_R_C          start of attempt I: shortly before its first scan
      ..._replan_K                  replan during the turn: the scan of iteration I in which
                                    the robot was where the view says (robot_xy)
      ..._BLOCKED                   end of the blocked attempt: after its last scan
      iteration_I_FINAL_SCAN        final scan of the mission: after all of them
    An attempt without scans (failed before moving) goes after the last previous scan.
    The instants are made increasing in the order of the names.
    """
    if not scans:
        return sorted(vis, key=lambda v: v[1])
    t_sc = np.array([float(d['time']) if 'time' in d else np.nan for d in scans])
    it_sc = np.array([int(d['iteration']) if 'iteration' in d else -1 for d in scans])
    xy_sc = np.array([np.asarray(d['robot_xyz'], float)[:2] for d in scans])
    if not np.isfinite(t_sc).all():
        return sorted(vis, key=lambda v: v[1])
    keyed, last_match = [], {}
    for _t, name, d in vis:
        m = _VIS_NAME.search(name)
        it = int(m.group(1)) if m else -1
        phase, sub = ((3, 0) if m and m.group(4) else (2, 0) if m and m.group(3)
                      else (1, int(m.group(2))) if m and m.group(2) else (0, 0))
        keyed.append(((it, phase, sub, name), name, d))
    keyed.sort(key=lambda k: k[0])
    out, t_prev = [], -np.inf
    for (it, phase, sub, _n), name, d in keyed:
        if 'time' in d and np.isfinite(_scalar(d, 'time', np.nan)):
            t = _scalar(d, 'time', np.nan)
        else:
            idx = np.flatnonzero(it_sc == it)
            before = t_sc[it_sc < it]
            t_after_prev = (before.max() if before.size else t_sc[0] - 1.0) + 0.01
            if phase == 3:
                t = t_sc.max() + 1.0
            elif not idx.size:
                t = t_after_prev
            elif phase == 0:
                t = max(t_sc[idx[0]] - 0.5, min(t_after_prev, t_sc[idx[0]] - 1e-3))
            elif phase == 2:
                t = t_sc[idx[-1]] + 0.01
            else:
                # the scan of the iteration nearest to robot_xy, not before that
                # of the previous replan
                idx = idx[idx >= last_match.get(it, -1)]
                rxy = _arr(d, 'robot_xy').astype(float).ravel()
                k = idx[int(np.argmin(np.hypot(*(xy_sc[idx] - rxy).T)))] if rxy.size == 2 else idx[0]
                last_match[it] = k
                t = t_sc[k]
        t = max(t, t_prev + 1e-3)
        t_prev = t
        out.append((t, name, d))
    return out


# SCANS OUTSIDE THE LOOP. scans/ only has the navigation loop scans; the
# start scan of every attempt and the final one go into the robot's maps but not
# into scans/. If an attempt fails before moving, its start scan is the only one
# made at that point (28-10 10:27: the wall in front of cell (0,5) was missing in the global
# panel; 10:51: between scan 11 and 12 it seemed the robot had never looked
# from the arrival point). Those scans are however in the attempt's PRM view
# (iteration_I_cell_R_C_vis.npz) and in the final one (iteration_I_FINAL_SCAN_vis.npz): they
# are rebuilt from there and inserted among the others, in time order, both in browsing
# and in the maps. Approximations: the view does not have the "never written" cells, so a cell
# counts as seen if terrain_valid >= spot_grid.TERRAIN_VALID_MIN (on 28-10 equal to the robot's
# in 98.8% of the cells); the raw height is the corrected one; the heading comes
# from trajectory.csv (last sample before, at the same point) and without it the area
# under the body is missing. Frontier and decision do not exist: empty fields, and the 'extra_kind' field.
_VIS_SCAN_NAME = re.compile(r'iteration_(\d+)_(?:cell_(\d+)_(\d+)|FINAL_SCAN)_vis\.npz$')


def _scan_from_vis(t, name, v, yaw_at=None):
    """A start-of-attempt or final PRM view as a scan (see SCANS OUTSIDE THE
    LOOP), None if it is not enough. yaw_at(t, x, y): robot heading, NaN if unknown."""
    m = _VIS_SCAN_NAME.search(name)
    if not m:
        return None
    try:
        pts = np.asarray(v['pts'], float)
        n = int(round(np.sqrt(len(pts))))
        if n * n != len(pts):
            return None
        xs, ys = pts[:, 0].reshape(n, n), pts[:, 1].reshape(n, n)
        cs = float(np.median(np.diff(xs[0])))
        shape = (n, n)
        rx, ry = np.asarray(v['robot_xy'], float).ravel()[:2]
        terrain = np.asarray(v['terrain'], float).reshape(shape)
        d = {
            'terrain_raw': terrain.astype(np.float32),
            'terrain_corrected': terrain,
            'terrain_valid_raw': np.asarray(v['terrain_valid'], float).reshape(shape),
            'is_valid': np.asarray(v['terrain_valid'], float).reshape(shape) >= spot_grid.TERRAIN_VALID_MIN,
            'unwritten': np.zeros(shape, bool),
            'obstacle_mask': np.sign(np.asarray(v['obstacle_mask'], float)).reshape(shape).astype(np.int8),
            'obstacle_distance': np.asarray(v['obstacle_distance'], float).reshape(shape),
            'gradient': np.asarray(v['gradient'], float).reshape(shape),
            'roughness': np.asarray(v['roughness'], float).reshape(shape),
            'grid_origin': np.array([xs.min() - cs / 2.0, ys.min() - cs / 2.0]),
            'cell_size': np.array(cs),
            'robot_xyz': np.array([rx, ry, np.nan]),
            'time': np.array(t),
            'iteration': np.array(int(m.group(1))),
            'decision_kind': np.array('attempt start' if m.group(2) else 'final scan'),
            'extra_kind': np.array(f"start of the attempt on cell ({m.group(2)},{m.group(3)})"
                                   if m.group(2) else "final scan of the mission"),
            'polyline': np.zeros((0, 2)),
            'frontier_reason': np.array(''),
            'frontier_stop_xy': np.array([np.nan, np.nan]),
            'decision_target': np.array([np.nan, np.nan]),
            'frontier_dist': np.array(np.nan),
            'allowed_advance': np.array(np.nan),
        }
        if 'mission_z0' in v:
            d['mission_z0'] = np.asarray(v['mission_z0'])
        yaw = yaw_at(t, rx, ry) if yaw_at is not None else np.nan
        if np.isfinite(yaw):
            d['robot_yaw'] = np.array(yaw)
        return d
    except (KeyError, ValueError):
        return None


def _yaw_lookup(folder):
    """yaw_at(t, x, y) from trajectory.csv: the last sample before t, if within 0.3 m of
    (x, y); NaN otherwise or if the file is missing."""
    name = os.path.basename(os.path.normpath(folder))
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.normpath(folder))), "MissionLogs",
                        name, "trajectory.csv")
    try:
        import csv
        with open(path, newline='') as fh:
            rows = np.array([(float(r['time']), float(r['x']), float(r['y']), float(r['yaw_deg']))
                             for r in csv.DictReader(fh)])
    except Exception:
        rows = np.zeros((0, 4))

    def yaw_at(t, x, y):
        k = int(np.searchsorted(rows[:, 0], t)) - 1 if len(rows) else -1
        if k < 0 or np.hypot(rows[k, 1] - x, rows[k, 2] - y) > 0.3:
            return np.nan
        return float(np.radians(rows[k, 3]))
    return yaw_at


class MissionContext:
    """Data of the whole mission, loaded once: scans, PRM views, trajectory."""

    def __init__(self, folder, scans, scan_files, vis_files):
        self.scans, self.scan_files = scans, scan_files
        self.vis = []
        for f in vis_files:
            try:
                self.vis.append((np.nan, os.path.basename(f), _load(f)))
            except Exception as e:
                print(f"[viewer] PRM view not readable {os.path.basename(f)}: {e}")
        self.vis = _vis_times(self.vis, scans)
        # Scans outside the loop (see SCANS OUTSIDE THE LOOP), among the others in time
        # order: from here on self.scans and self.scan_files include them.
        t_sc = [_scalar(d, 'time', np.nan) for d in scans]
        if scans and np.isfinite(t_sc).all():
            paths = {os.path.basename(p): p for p in vis_files}
            yaw_at = _yaw_lookup(folder)
            items = [(t, 0, d, p) for t, d, p in zip(t_sc, scans, scan_files)]
            for t, name, v in self.vis:
                d = _scan_from_vis(t, name, v, yaw_at)
                if d is not None:
                    items.append((t, 1, d, paths.get(name, name)))
            items.sort(key=lambda it: (it[0], it[1]))
            self.scans = scans = [it[2] for it in items]
            self.scan_files = scan_files = [it[3] for it in items]
        self.traj = self._load_trajectory(folder)
        occ = self._load_pkl_cells(folder, "global_occupancy_map.pkl", 'grid')
        ter = self._load_pkl_cells(folder, "global_terrain_map.pkl", 'height_count')
        self.final_map = self.final_terrain = None
        self._cache = {}
        self._terrain_norm = None
        self._folder = folder
        self._navgraph = False          # False = not read yet, None = not available
        if not scans:
            return
        # Common grid: all the scans plus the maps saved at the end of the mission (which
        # also contain the start-of-attempt scans). The origin is a multiple of the
        # resolution, so the cells coincide with those of the robot's maps (floor(x/res)).
        self.res = float(scans[0]['cell_size'])
        ext = np.array([_extent(d) for d in scans])
        xmin, xmax, ymin, ymax = ext[:, 0].min(), ext[:, 1].max(), ext[:, 2].min(), ext[:, 3].max()
        for cells in (occ, ter):
            if cells is not None:
                k, res = cells[0], cells[2]
                xmin, xmax = min(xmin, k[:, 0].min() * res), max(xmax, (k[:, 0].max() + 1) * res)
                ymin, ymax = min(ymin, k[:, 1].min() * res), max(ymax, (k[:, 1].max() + 1) * res)
        pad = 1.0
        self.kx0 = int(np.floor((xmin - pad) / self.res))
        self.ky0 = int(np.floor((ymin - pad) / self.res))
        self.x0, self.y0 = self.kx0 * self.res, self.ky0 * self.res
        self.nx = int(np.ceil((xmax + pad - self.x0) / self.res))
        self.ny = int(np.ceil((ymax + pad - self.y0) / self.res))
        # Ground at mission start: reference for the heights.
        self.z0 = np.nan
        if ter is not None and ter[3] is not None:
            self.z0 = float(ter[3])
        if not np.isfinite(self.z0):
            self.z0 = _mission_z0(scans[0])
        if occ is not None:
            self.final_map = self._raster(occ[0], np.where(occ[1] < 0, 2, 1), np.uint8, 0)
        if ter is not None:
            self.final_terrain = self._raster(ter[0], ter[1], np.float32, np.nan) - self.z0

    @staticmethod
    def _load_pkl_cells(folder, name, key):
        """
        Cells of a global map saved at the end of the mission (.pkl from the mission):
        (keys (N, 2) as (gx, gy), values, resolution, mission_z0). For occupancy the values
        are -1 obstacle / +1 free; for terrain the absolute mean height. None if missing.
        """
        path = os.path.join(folder, name)
        if not os.path.exists(path):
            return None
        try:
            with open(path, 'rb') as f:
                g = pickle.load(f)
            cells = g[key]
            if not cells:
                return None
            keys = np.array(list(cells.keys()), dtype=np.int64).reshape(-1, 2)
            if key == 'grid':
                vals = np.array(list(cells.values()), dtype=float)
            else:
                hs = g['height_sum']
                vals = np.array([hs[k] / n for k, n in cells.items()], dtype=float)
            return keys, vals, float(g['resolution']), g.get('mission_z0')
        except Exception as e:
            print(f"[viewer] global map not readable ({path}): {e}")
            return None

    def _raster(self, keys, vals, dtype, fill):
        """Cells (gx, gy) -> matrix on the common grid."""
        img = np.full((self.ny, self.nx), fill, dtype=dtype)
        c, r = keys[:, 0] - self.kx0, keys[:, 1] - self.ky0
        ok = (c >= 0) & (c < self.nx) & (r >= 0) & (r < self.ny)
        img[r[ok], c[ok]] = np.asarray(vals)[ok]
        return img

    @property
    def extent(self):
        return [self.x0, self.x0 + self.nx * self.res, self.y0, self.y0 + self.ny * self.res]

    @staticmethod
    def _load_trajectory(folder):
        """trajectory.csv in MissionLogs/<same mission>, if there is one (older missions do not have it)."""
        name = os.path.basename(os.path.normpath(folder))
        for cand in (os.path.join(os.path.dirname(os.path.dirname(os.path.normpath(folder))), "MissionLogs", name,
                                  "trajectory.csv"),
                     os.path.join(folder, "trajectory.csv")):
            if os.path.exists(cand):
                try:
                    with open(cand, newline='', encoding='utf-8') as f:
                        rows = list(csv.DictReader(f))
                    # fourth column: 1 GraphNav move ('move' column, since 2026-10-26),
                    # 2 correction of the return to wp_0 with the fiducial (since 29-10), 0 the rest
                    kind = {'graphnav': 1.0, 'graphnav_correction': 2.0}
                    return np.array([(float(r['time']), float(r['x']), float(r['y']),
                                      kind.get(modern(r.get('move')), 0.0)) for r in rows]).reshape(-1, 4)
                except Exception as e:
                    print(f"[viewer] trajectory.csv not readable: {e}")
        return None

    def _window(self, d):
        """Slices of the common grid and of scan d for the overlapping part."""
        gox, goy = d['grid_origin']
        r0 = int(np.floor((goy - self.y0) / self.res))
        c0 = int(np.floor((gox - self.x0) / self.res))
        h, w = d['terrain_raw'].shape
        rs, cs_ = slice(max(r0, 0), min(r0 + h, self.ny)), slice(max(c0, 0), min(c0 + w, self.nx))
        return rs, cs_, slice(rs.start - r0, rs.stop - r0), slice(cs_.start - c0, cs_.stop - c0)

    @staticmethod
    def _robot_mask(d):
        """
        Like global_maps.mask_for_global_map: -1 obstacle, +1 free, 0 no information;
        free under the body, no obstacle in the ring around it. Also the body mask.
        """
        om = np.asarray(d['obstacle_mask']).reshape(d['terrain_raw'].shape)
        out = np.where(om < 0, -1, np.where(np.asarray(d['is_valid'], bool), 1, 0)).astype(np.int8)
        core, ring = _body_masks(d)
        out[ring & (out < 0)] = 0
        out[core] = 1
        return out, core

    def _build(self, name, upto, init, step):
        """
        State of a map after scans 0..upto, built one scan at a time with
        step(state, scan). Forward it continues from where it was; backward it restarts from
        the nearest saved copy (one every CHECKPOINT_EVERY scans).
        """
        c = self._cache.setdefault(name, {'upto': -1, 'state': None, 'ckpt': {}})
        if c['state'] is None or upto < c['upto']:
            ks = [k for k in c['ckpt'] if k <= upto]
            if ks:
                c['upto'] = max(ks)
                c['state'] = tuple(a.copy() for a in c['ckpt'][c['upto']])
            else:
                c['upto'], c['state'] = -1, init()
        for i in range(c['upto'] + 1, upto + 1):
            step(c['state'], self.scans[i])
            if (i + 1) % CHECKPOINT_EVERY == 0 and i not in c['ckpt']:
                c['ckpt'][i] = tuple(a.copy() for a in c['state'])
        c['upto'] = upto
        return c['state']

    def _zeros(self, *dtypes):
        return lambda: tuple(np.zeros((self.ny, self.nx), dt) for dt in dtypes)

    def robot_map(self, upto):
        """
        0/1/2 map (unknown/free/obstacle) of scans 0..upto as the robot kept it: an
        obstacle counts only after OCC_CONFIRM_OBS consecutive readings (OCC_CONFIRM_WALKED on
        the cells the body passed over), and becomes free again when the free readings even it out.
        With the start-of-attempt and final scans (see SCANS OUTSIDE THE LOOP); the fall
        disks and the recovery scans are missing.
        """
        def step(st_all, d):
            state, count, walked = st_all
            m, core = self._robot_mask(d)
            rs, cs_, sr, sc = self._window(d)
            m, core = m[sr, sc], core[sr, sc]
            st, cnt, wk = state[rs, cs_], count[rs, cs_], walked[rs, cs_]   # views: they write into the maps
            wk |= core
            obs, free = m < 0, m > 0
            c_obs = np.minimum(np.where(cnt >= 0, cnt + 1, 1), OCC_MAX_COUNT)
            c_free = np.maximum(cnt - 1, -OCC_MAX_COUNT)
            new = np.where(obs, c_obs, np.where(free, c_free, cnt)).astype(np.int8)
            confirm = np.where(wk, OCC_CONFIRM_WALKED, OCC_CONFIRM_OBS)
            to_obs = obs & (new >= confirm)
            to_free = free & (new <= 0)
            seen_new = (obs | free) & (st == 0) & ~to_obs
            st[to_obs] = 2
            st[to_free | seen_new] = 1
            cnt[...] = new
        return self._build('robot', upto, self._zeros(np.uint8, np.int8, bool), step)[0]

    def mosaic(self, upto):
        """0/1/2 mosaic (unknown/free/obstacle) of scans 0..upto: the last one wins."""
        def step(st_all, d):
            om = np.asarray(d['obstacle_mask']).reshape(d['terrain_raw'].shape)
            known = np.asarray(d['is_valid'], bool) | (om < 0)
            if 'unwritten' in d:
                known &= ~np.asarray(d['unwritten'], bool) | (om < 0)
            val = np.where(om < 0, 2, 1).astype(np.uint8)
            rs, cs_, sub_r, sub_c = self._window(d)
            tgt = st_all[0][rs, cs_]
            k = known[sub_r, sub_c]
            tgt[k] = val[sub_r, sub_c][k]
        return self._build('raw', upto, self._zeros(np.uint8), step)[0]

    def terrain(self, upto):
        """
        Mean height (m, relative to the ground at mission start) of scans 0..upto, like
        global_maps.GlobalTerrainGrid: corrected heights, only reliable non-obstacle cells
        (spot_grid.slope_usable_mask). NaN where there is no height.
        """
        def step(st_all, d):
            hsum, hn = st_all
            shape = d['terrain_raw'].shape
            t = np.asarray(d['terrain_corrected'], dtype=np.float64).reshape(shape)
            ok = np.asarray(d['is_valid'], bool).reshape(shape) & np.isfinite(t)
            if 'obstacle_distance' in d:
                ok &= ~(np.asarray(d['obstacle_distance']).reshape(shape) < OBSTACLE_THRESHOLD)
            rs, cs_, sr, sc = self._window(d)
            t, ok = t[sr, sc], ok[sr, sc]
            hsum[rs, cs_][ok] += t[ok]
            hn[rs, cs_][ok] += 1
        hsum, hn = self._build('terrain', upto, self._zeros(np.float64, np.uint32), step)
        with np.errstate(invalid='ignore', divide='ignore'):
            return np.where(hn > 0, hsum / np.maximum(hn, 1), np.nan) - self.z0

    def navgraph(self):
        """The nav graph (see load_navgraph), read at the first request."""
        if self._navgraph is False:
            self._navgraph = load_navgraph(self._folder)
            ng = self._navgraph
            if ng is not None and self.scans:
                # If the robot clock cannot be compared with that of the scans,
                # no filter by instant: the whole graph is always shown.
                t = ng['time']
                t_scan = [float(d['time']) for d in (self.scans[0], self.scans[-1]) if 'time' in d]
                if (not np.isfinite(t).any() or len(t_scan) < 2
                        or np.nanmin(t) > t_scan[1] + 60 or np.nanmax(t) < t_scan[0] - 60):
                    ng['time'] = np.full(len(t), np.nan)
        return self._navgraph

    def walked(self, upto):
        """Cells the body passed over in scans 0..upto (from the robot's map)."""
        self.robot_map(upto)
        return self._cache['robot']['state'][2]

    def body_quota(self, d):
        """
        Absolute height of the end-of-mission terrain map on the cells of scan d that lie
        under the body and have no height (see HEIGHT UNDER THE BODY), and their mask.
        (None, None) if not needed or not available.
        """
        if self.final_terrain is None:
            return None, None
        core, _ = _body_masks(d)
        shape = d['terrain_raw'].shape
        hole = core & ~np.asarray(d['is_valid'], bool).reshape(shape)
        if not hole.any():
            return None, None
        h, w = shape
        cs = float(d['cell_size'])
        gox, goy = d['grid_origin']
        X, Y = np.meshgrid(gox + (np.arange(w) + 0.5) * cs, goy + (np.arange(h) + 0.5) * cs)
        c = np.floor((X - self.x0) / self.res).astype(int)
        r = np.floor((Y - self.y0) / self.res).astype(int)
        inside = (c >= 0) & (c < self.nx) & (r >= 0) & (r < self.ny)
        q = np.full(shape, np.nan)
        q[inside] = self.final_terrain[r[inside], c[inside]] + self.z0
        hole &= np.isfinite(q)
        return (q, hole) if hole.any() else (None, None)

    def events(self):
        """
        Indices of the scans to jump to (keys b, c, v), by kind:
          'block'    -- scans with decision 'blocked'
          'cell'     -- first scan of every attempt (the iteration changes)
          'graphnav' -- first scan after a GraphNav move ('graphnav' samples of
                        trajectory.csv) or after an unrecorded leg (older missions)
        """
        if getattr(self, '_events', None) is None:
            kinds = [str(d['decision_kind']) for d in self.scans]
            its = [int(d['iteration']) if 'iteration' in d else -1 for d in self.scans]
            ev = {'block': [i for i, k in enumerate(kinds) if k == 'blocked'],
                  'cell': [i for i in range(len(its)) if i == 0 or its[i] != its[i - 1]],
                  'graphnav': []}
            t_sc = np.array([float(d['time']) if 'time' in d else np.nan for d in self.scans])
            if self.traj is not None and len(self.traj) and np.isfinite(t_sc).all():
                tr = self.traj
                ends = []
                gn = tr[:, 3] > 0.5
                # end of every recorded GraphNav leg
                ends += list(tr[np.flatnonzero(gn & ~np.r_[gn[1:], False]), 0])
                # end of every jump (unrecorded leg)
                jump = np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])) > TRAJ_GAP_M
                ends += list(tr[np.flatnonzero(jump) + 1, 0])
                idx = {int(np.searchsorted(t_sc, t)) for t in ends}
                ev['graphnav'] = sorted(i for i in idx if i < len(t_sc))
            self._events = ev
        return self._events

    def mission_frame(self):
        """
        (x, y, yaw) of the mission frame: origin at the centre of cell (0, 0), which is the
        starting point, and orientation of the cell grid (env_origin_yaw, the robot's nose at
        the start). From the PRM views; without them, from the first scan.
        """
        for _t, _n, vd in self.vis:
            cells = _arr(vd, 'env_cells', (4,))
            yaw = _scalar(vd, 'env_origin_yaw', np.nan)
            k = np.flatnonzero((cells[:, 0] == 0) & (cells[:, 1] == 0)) if len(cells) else []
            if len(k) and np.isfinite(yaw):
                return float(cells[k[0], 2]), float(cells[k[0], 3]), float(yaw)
        d = self.scans[0]
        return float(d['robot_xyz'][0]), float(d['robot_xyz'][1]), float(d.get('robot_yaw', 0.0))

    def terrain_norm(self):
        """Height colour scale, fixed for the mission (see TERRAIN_PERCENTILES)."""
        if self._terrain_norm is None:
            h = self.final_terrain if self.final_terrain is not None else self.terrain(len(self.scans) - 1)
            h = h[np.isfinite(h)]
            lo, hi = (np.percentile(h, TERRAIN_PERCENTILES) if h.size else (-0.5, 0.5))
            # The starting height (0) is always in the scale, so it shows on the bar.
            lo, hi = min(lo, 0.0), max(hi, 0.0)
            if hi - lo < 0.05:
                lo, hi = lo - 0.025, hi + 0.025
            self._terrain_norm = Normalize(vmin=float(lo), vmax=float(hi))
        return self._terrain_norm

    def vis_before(self, t):
        """Last PRM view saved by instant t (None if none). See _vis_times."""
        best = None
        for mt, name, d in self.vis:
            if t is None or not (mt > t):
                best = (name, d)
        return best if best is not None else ((self.vis[0][1], self.vis[0][2]) if self.vis else None)


def _scalar(d, key, default):
    """Scalar value from an npz, robust to missing, empty or NaN fields (old or test files)."""
    try:
        v = np.asarray(d[key], dtype=float).ravel()
        return float(v[0]) if v.size and np.isfinite(v[0]) else default
    except Exception:
        return default


def _arr(d, key, shape_tail=None):
    """Array from an npz, empty if missing (files from earlier versions)."""
    if key in d:
        return np.asarray(d[key])
    return np.empty((0,) + tuple(shape_tail or ()))


# The colour map is taken ONCE: plt.get_cmap(...)(v) rebuilds the table at every
# call, and with thousands of edges it was almost all the time of a redraw.
_SLOPE_CMAP = plt.get_cmap('RdYlGn_r')
_SLOPE_LUT_CACHE = {}   # fallback when the bundle does not accept attributes


def _slope_colors_many(keys, long_s, lat_s):
    """
    _slope_color on many edges at once: same colours, computed with array operations
    instead of one by one.
    """
    long_s = np.asarray(long_s, dtype=np.float64)
    lat_s = np.asarray(lat_s, dtype=np.float64)
    total = long_s + lat_s
    frac = np.where(total < 1e-6, 0.5, lat_s / np.where(total < 1e-6, 1.0, total))
    rgba = np.asarray(_SLOPE_CMAP(frac), dtype=np.float64).reshape(-1, 4)
    alpha = 0.3 + 0.5 * np.minimum(1.0, np.maximum(long_s, lat_s) / SLOPE_THRESHOLD)
    rgba[:, 3] = np.where(total < 1e-6, 0.25, alpha)
    rgba[total < 1e-6, :3] = 0.6
    return rgba


# --- EDGES AS AN IMAGE ----------------------------------------------------------------------
# The edges are drawn ONCE into an RGBA image (where they overlap the transparency
# adds up) and shown with imshow, which costs as much as the map: with tens of thousands of
# edges, drawing them as lines made every redraw slow, including dragging and zooming.
# The image is cached per PRM view and extent.
_EDGE_RASTER_CACHE = {}
_EDGE_RASTER_CACHE_MAX = 256
# Maximum side of the edge image, in pixels: beyond it, the resolution widens.
EDGE_RASTER_MAX_PX = 1600
# Opacity of an edge in a pixel, tuned by eye so the image resembles lines of
# 0.5 points.
EDGE_RASTER_COVERAGE = 0.45
EDGE_RASTER_COVERAGE_GLOBAL = 0.22   # in the global panel there are many more edges per pixel


def _slope_table(vd):
    """Keys (a, b) sorted as integer codes + values, to look up the slopes with searchsorted."""
    tab = getattr(vd, '_slope_tab', None) if not isinstance(vd, dict) else vd.get('_slope_tab')
    if tab is None:
        tab = _SLOPE_LUT_CACHE.get(('tab', id(vd)))
    if tab is None:
        k = np.asarray(_arr(vd, 'prm_slope_keys', (2,)), dtype=np.int64).reshape(-1, 2)
        v = np.asarray(_arr(vd, 'prm_slope_values', (2,)), dtype=np.float64).reshape(-1, 2)
        k = np.sort(k, axis=1)
        code = k[:, 0] * (1 << 32) + k[:, 1]
        order = np.argsort(code)
        tab = (code[order], v[order])
        try:
            vd._slope_tab = tab
        except Exception:
            _SLOPE_LUT_CACHE[('tab', id(vd))] = tab
    return tab


def _edge_colors(vd, e, grey_rgba, alpha=None):
    """Colour of every edge like _slope_color (grey if there is no slope), all vectorized."""
    cols = np.tile(np.asarray(grey_rgba, dtype=np.float64), (len(e), 1))
    codes, vals = _slope_table(vd)
    if len(codes) and len(e):
        ee = np.sort(e, axis=1)
        q = ee[:, 0] * (1 << 32) + ee[:, 1]
        j = np.clip(np.searchsorted(codes, q), 0, len(codes) - 1)
        have = codes[j] == q
        if have.any():
            cols[have] = _slope_colors_many(None, vals[j[have], 0], vals[j[have], 1])
    if alpha is not None:
        cols[:, 3] = alpha
    return cols


def _edges_image(segs, cols, x0, x1, y0, y1, max_px=EDGE_RASTER_MAX_PX, min_res=None,
                 coverage=EDGE_RASTER_COVERAGE):
    """
    Segments (N, 2, 2) with RGBA colours (N, 4) -> RGBA image (ny, nx, 4) over the given
    extent. Every edge is sampled at one-pixel steps; in a pixel touched by k edges the colour
    is the mean and the opacity 1 - prod(1 - alpha), as when overlaying transparent lines.
    """
    w, h = max(x1 - x0, 1e-6), max(y1 - y0, 1e-6)
    res = max(w, h) / float(max_px)
    if min_res is not None:
        res = max(res, float(min_res))
    nx, ny = max(1, int(np.ceil(w / res))), max(1, int(np.ceil(h / res)))
    img = np.zeros((ny, nx, 4), dtype=np.float32)
    if len(segs) == 0:
        return img
    p0, p1 = segs[:, 0, :], segs[:, 1, :]
    L = np.hypot(p1[:, 0] - p0[:, 0], p1[:, 1] - p0[:, 1])
    n = np.maximum(2, np.ceil(L / res).astype(np.int64) + 1)
    idx = np.repeat(np.arange(len(segs)), n)
    start = np.cumsum(n) - n
    t = (np.arange(int(n.sum())) - np.repeat(start, n)) / np.repeat(n - 1, n)
    px = p0[idx, 0] + t * (p1[idx, 0] - p0[idx, 0])
    py = p0[idx, 1] + t * (p1[idx, 1] - p0[idx, 1])
    c = np.floor((px - x0) / res).astype(np.int64)
    r = np.floor((py - y0) / res).astype(np.int64)
    ok = (c >= 0) & (c < nx) & (r >= 0) & (r < ny)
    flat, idx = r[ok] * nx + c[ok], idx[ok]
    # an edge counts once per pixel: its samples are consecutive, so it is enough to
    # discard those falling in the same pixel as the previous one (no sorting).
    if len(flat) > 1:
        keep = np.empty(len(flat), dtype=bool)
        keep[0] = True
        keep[1:] = (flat[1:] != flat[:-1]) | (idx[1:] != idx[:-1])
        flat, idx = flat[keep], idx[keep]
    m = nx * ny
    cnt = np.bincount(flat, minlength=m).astype(np.float64)
    hit = cnt > 0
    out = img.reshape(-1, 4)
    for ch in range(3):
        sch = np.bincount(flat, weights=cols[idx, ch], minlength=m)
        out[hit, ch] = (sch[hit] / cnt[hit]).astype(np.float32)
    # coverage: fraction of pixel a thin line (0.5 points) really covers; without it, a
    # "touched" image pixel would become full and the drawing much darker than before.
    logt = np.bincount(flat, weights=np.log1p(-np.clip(coverage * cols[idx, 3], 0.0, 0.999)),
                       minlength=m)
    out[hit, 3] = (1.0 - np.exp(logt[hit])).astype(np.float32)
    return img


def _cached_edges_image(key, build):
    img = _EDGE_RASTER_CACHE.get(key)
    if img is None:
        img = build()
        if len(_EDGE_RASTER_CACHE) >= _EDGE_RASTER_CACHE_MAX:
            _EDGE_RASTER_CACHE.pop(next(iter(_EDGE_RASTER_CACHE)))
        _EDGE_RASTER_CACHE[key] = img
    return img


def _draw_cells(ax, vd):
    """Mission grid: green = visited, red = blocked, hatched = to explore."""
    if vd is None or 'env_cells' not in vd or not len(vd['env_cells']):
        return
    size = _scalar(vd, 'env_cell_size', 5.0)
    yaw = _scalar(vd, 'env_origin_yaw', 0.0)
    h = size / 2.0
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    base = np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) @ R.T
    for (r, c, x, y), st in zip(vd['env_cells'], vd['env_cell_status']):
        st = str(st)
        if st == '1':
            kw = dict(edgecolor='darkgreen', facecolor='lightgreen', alpha=0.25, lw=1.2)
        elif st == '-1':
            kw = dict(edgecolor='darkred', facecolor='lightcoral', alpha=0.35, lw=1.2)
        else:
            kw = dict(edgecolor='gray', facecolor='none', alpha=0.6, lw=0.8, ls='--')
        ax.add_patch(patches.Polygon(base + [x, y], zorder=1, **kw))
        ax.text(x, y, f"{int(r)},{int(c)}", ha='center', va='center', fontsize=7, color='0.3', zorder=1)


def overlay_prm_local(ax, ctx, i):
    """Graph edges and nodes (last PRM view before the scan) over the local window."""
    try:
        _overlay_prm_local(ax, ctx, i)
    except Exception as e:
        print(f"[viewer] local graph not drawn: {e}")


def _overlay_prm_local(ax, ctx, i):
    d = ctx.scans[i]
    vis = ctx.vis_before(float(d['time']) if 'time' in d else None)
    if not vis:
        return
    vd = vis[1]
    x0, x1, y0, y1 = _extent(d)
    # Only the edges touching the window are kept, with array operations: the PRM
    # edges are tens of thousands and are rechecked at every redraw.
    ids = np.asarray(_arr(vd, 'prm_node_ids', ()), dtype=np.int64).ravel()
    nxy = np.asarray(_arr(vd, 'prm_node_xy', (2,)), dtype=np.float64).reshape(-1, 2)
    edges = np.asarray(_arr(vd, 'prm_edges', (2,)), dtype=np.int64).reshape(-1, 2)
    if len(ids) == 0 or len(nxy) != len(ids):
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
        return
    pos = np.full((int(ids.max()) + 2, 2), np.nan)
    pos[ids] = nxy
    inside_node = np.zeros(len(pos), dtype=bool)
    inside_node[ids] = ((nxy[:, 0] >= x0 - 1) & (nxy[:, 0] <= x1 + 1)
                        & (nxy[:, 1] >= y0 - 1) & (nxy[:, 1] <= y1 + 1))
    if len(edges):
        ok = (edges[:, 0] < len(pos)) & (edges[:, 1] < len(pos))
        e = edges[ok]
        e = e[inside_node[e[:, 0]] & inside_node[e[:, 1]]]
    else:
        e = edges
    if len(e):
        # As an image, see EDGES AS AN IMAGE.
        def build():
            segs = np.stack([pos[e[:, 0]], pos[e[:, 1]]], axis=1)
            cols = _edge_colors(vd, e, (0.2, 0.2, 0.2, 0.35), alpha=0.35)
            return _edges_image(segs, cols, x0, x1, y0, y1, max_px=480)
        img = _cached_edges_image(('loc', vis[0], round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)), build)
        ax.imshow(img, origin='lower', extent=[x0, x0 + img.shape[1] * ((x1 - x0) / img.shape[1]),
                                              y0, y0 + img.shape[0] * ((y1 - y0) / img.shape[0])],
                  interpolation='nearest', zorder=3)
    pts = nxy[inside_node[ids]]
    if len(pts):
        ax.plot(pts[:, 0], pts[:, 1], '.', color='k', ms=3, alpha=0.6, zorder=3)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)


def _global_edges_image(ctx, name, vd):
    """
    Image of the global panel edges for a PRM view, cached (see EDGES AS AN
    IMAGE). All edges, without thinning. (img, extent) or (None, None).
    """
    gx0, gx1 = ctx.x0, ctx.x0 + ctx.nx * ctx.res
    gy0, gy1 = ctx.y0, ctx.y0 + ctx.ny * ctx.res

    def build():
        ids = np.asarray(_arr(vd, 'prm_node_ids', ()), dtype=np.int64).ravel()
        nxy = np.asarray(_arr(vd, 'prm_node_xy', (2,)), dtype=np.float64).reshape(-1, 2)
        edges = np.asarray(_arr(vd, 'prm_edges', (2,)), dtype=np.int64).reshape(-1, 2)
        if not (len(ids) and len(nxy) == len(ids) and len(edges)):
            return None
        pos = np.full((int(ids.max()) + 2, 2), np.nan)
        pos[ids] = nxy
        known = np.zeros(len(pos), dtype=bool)
        known[ids] = True
        ok = (edges[:, 0] < len(pos)) & (edges[:, 1] < len(pos)) & (edges[:, 0] >= 0) & (edges[:, 1] >= 0)
        e = edges[ok]
        e = e[known[e[:, 0]] & known[e[:, 1]]]
        if not len(e):
            return None
        segs = np.stack([pos[e[:, 0]], pos[e[:, 1]]], axis=1)
        cols = _edge_colors(vd, e, (0.5, 0.5, 0.5, 0.25))
        return _edges_image(segs, cols, gx0, gx1, gy0, gy1, min_res=ctx.res,
                            coverage=EDGE_RASTER_COVERAGE_GLOBAL)

    img = _cached_edges_image(('glob', name, gx0, gy0, gx1, gy1), build)
    if img is None:
        return None, None
    return img, [gx0, gx0 + img.shape[1] * ((gx1 - gx0) / img.shape[1]),
                 gy0, gy0 + img.shape[0] * ((gy1 - gy0) / img.shape[0])]


def _prebuild_global_images(ctx):
    """In the background, at opening: the images of all PRM views, so scrolling does not wait."""
    for _mt, name, vd in list(ctx.vis):
        try:
            _global_edges_image(ctx, name, vd)
        except Exception as e:
            print(f"[viewer] edge image of {name} not prepared: {e}")


def _traj_runs(tr):
    """
    Split the trajectory (t, x, y, kind) where between two consecutive samples there is a
    jump beyond TRAJ_GAP_M, and where the kind of movement changes (0 walked, 1 GraphNav, 2
    correction of the return to wp_0). Returns (recorded stretches as (points, kind), jumps
    as pairs of points).
    """
    if len(tr) == 0:
        return [], []
    kind = np.rint(tr[:, 3]).astype(int) if tr.shape[1] > 3 else np.zeros(len(tr), int)
    jump = np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])) > TRAJ_GAP_M
    cuts = np.flatnonzero(jump | (kind[1:] != kind[:-1])) + 1
    runs = []
    for idx in np.split(np.arange(len(tr)), cuts):
        if len(idx):
            # the stretch starts from the last point of the previous one, if there is no jump in between
            k = idx[0]
            if k > 0 and not jump[k - 1]:
                idx = np.concatenate(([k - 1], idx))
            runs.append((tr[idx, 1:3], int(kind[idx[-1]])))
    gaps = [(tr[k - 1, 1:3], tr[k, 1:3]) for k in np.flatnonzero(jump) + 1]
    return runs, gaps

def _hillshade(h, res):
    """Relief shading (light from north-west, 45 degrees): 1 = flat. NaN -> 1."""
    gy, gx = np.gradient(h, res)
    alt, az = np.radians(45.0), np.radians(315.0)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    sh = (np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)) / np.sin(alt)
    return np.where(np.isfinite(sh), sh, 1.0)


def _background_rgb(occ, height, mode, norm, res):
    """RGB image of the 'terreno' or 'fusione' background (see BACKGROUNDS)."""
    rgb = plt.get_cmap(TERRAIN_CMAP)(norm(np.nan_to_num(height, nan=norm.vmin)))[..., :3]
    if HILLSHADE_STRENGTH > 0:
        shade = 1.0 + HILLSHADE_STRENGTH * (np.clip(_hillshade(height, res), 0.0, 2.0) - 1.0)
        rgb = np.clip(rgb * shade[..., None], 0.0, 1.0)
    has_h = np.isfinite(height)
    rgb[~has_h] = 1.0                                         # never seen: white
    if mode == 'fusione' and occ is not None:
        rgb[~has_h & (occ == 1)] = NO_TERRAIN_RGB
        rgb[occ == 2] = OBSTACLE_RGB
    return rgb


def _box_sum(a, k):
    """Sum over a centred k x k window (zero borders), with cumulative sums: numpy only."""
    r = k // 2
    p = np.pad(a, ((r + 1, r), (r + 1, r)))
    c = p.cumsum(0).cumsum(1)
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


def _draw_zero_level(ax, height, extent):
    """
    Level line at height 0 (the ground at mission start) over the terrain background:
    separates what lies below the start from what lies above. No line if the height does
    not cross zero.
    """
    if height is None:
        return
    # Mean over ZERO_LEVEL_SMOOTH_CELLS cells, only for the line: on the flat terrain around
    # the start the height noise (a few cm) would fill it with small rings.
    k = ZERO_LEVEL_SMOOTH_CELLS
    ok = np.isfinite(height)
    num = _box_sum(np.where(ok, height, 0.0), k)
    den = _box_sum(ok.astype(np.float64), k)
    with np.errstate(invalid='ignore', divide='ignore'):
        smooth = np.where(ok & (den > 0), num / np.maximum(den, 1), np.nan)
    h = np.ma.masked_invalid(smooth)
    if h.count() == 0 or not (h.min() < 0.0 < h.max()):
        return
    ny, nx = h.shape
    xs = np.linspace(extent[0], extent[1], nx, endpoint=False) + (extent[1] - extent[0]) / nx / 2
    ys = np.linspace(extent[2], extent[3], ny, endpoint=False) + (extent[3] - extent[2]) / ny / 2
    cs = ax.contour(xs, ys, h, levels=[0.0], colors=ZERO_LEVEL_COLOR, linewidths=1.4, zorder=1)
    stroke = [patheffects.withStroke(linewidth=3.0, foreground='black')]
    # The black border is added afterwards: contour does not accept path_effects in old versions, and
    # in new ones it no longer has .collections.
    for art in (getattr(cs, 'collections', None) or [cs]):
        art.set_path_effects(stroke)
    ax.plot([], [], '-', color=ZERO_LEVEL_COLOR, lw=1.4,
            path_effects=[patheffects.withStroke(linewidth=3.0, foreground='black')],
            label="height 0 (ground at mission start)")


def _mark_zero(cbar):
    """The zero on the height bar: tick and line, if it is in the scale."""
    lo, hi = cbar.mappable.norm.vmin, cbar.mappable.norm.vmax
    if not (lo <= 0.0 <= hi):
        return
    ticks = [t for t in cbar.get_ticks() if lo <= t <= hi and abs(t) > 1e-9 * max(1.0, hi - lo)]
    extra = [0.0]
    if lo < 0.0 and all(abs(t - lo) > 0.25 * (hi - lo) / max(len(ticks), 1) for t in ticks + extra):
        extra.append(round(lo, 2))      # the negative end, if there is no tick nearby already
    cbar.set_ticks(sorted(ticks + extra))
    cbar.ax.axhline(0.0, color=ZERO_LEVEL_COLOR, lw=2.0,
                    path_effects=[patheffects.withStroke(linewidth=3.5, foreground='black')])


def _hover_global(ax, ctx, occ, height, ng=None):
    """Coordinates, cell state, height and nearby waypoint under the mouse in the global panel."""
    names = {0: 'never seen', 1: 'free', 2: 'OSTACOLO'}

    def fmt(x, y):
        out = _fmt_cell(x, y)
        if ng is not None and len(ng['xy']):
            dist = np.hypot(ng['xy'][:, 0] - x, ng['xy'][:, 1] - y)
            k = int(np.argmin(dist))
            if dist[k] < 0.3:
                out += f" | {ng['names'][k]}"
        return out

    def _fmt_cell(x, y):
        c, r = int(np.floor((x - ctx.x0) / ctx.res)), int(np.floor((y - ctx.y0) / ctx.res))
        out = f"x={x:.2f} y={y:.2f}"
        if 0 <= r < ctx.ny and 0 <= c < ctx.nx:
            if occ is not None:
                out += f" | {names.get(int(occ[r, c]), '?')}"
            if height is not None:
                h = height[r, c]
                out += f" | height {h:+.3f} m" if np.isfinite(h) else " | height: none"
        return out
    ax.format_coord = fmt


def draw_navgraph(ax, ng, t_now=None):
    """Nav graph on the global panel; with t_now only the waypoints already created at that instant."""
    shown = np.ones(len(ng['xy']), bool)
    if t_now is not None and np.isfinite(ng['time']).any():
        shown = ~(ng['time'] > t_now + 1.0)
    xy, e = ng['xy'], ng['edges']
    keep = shown[e[:, 0]] & shown[e[:, 1]] if len(e) else np.zeros(0, bool)
    styles = {}
    for (a, b), src in zip(e[keep], ng['source'][keep]):
        styles.setdefault(NAVGRAPH_EDGE_STYLE.get(src, NAVGRAPH_OTHER_STYLE), []).append((xy[a], xy[b]))
    for (col, lw, ls, lab), segs in styles.items():
        ax.add_collection(LineCollection(segs, colors=col, linewidths=lw, linestyles=ls, zorder=3,
                                         label=lab))
    names = ng['names']
    ours = shown & np.char.startswith(names.astype(str), 'wp_')
    auto = shown & ~ours
    if auto.any():
        ax.plot(xy[auto, 0], xy[auto, 1], '.', color='dimgray', ms=3, zorder=3,
                label='automatic GraphNav waypoints')
    if ours.any():
        ax.plot(xy[ours, 0], xy[ours, 1], 'o', color='black', mfc='white', ms=3.5, mew=0.8, zorder=3,
                label='mission waypoints (breadcrumbs, cells)')
    k0 = np.flatnonzero(shown & (names == 'wp_0'))
    if len(k0):
        ax.plot(*xy[k0[0]], 'P', color='black', mfc='gold', ms=11, zorder=8, label='wp_0 (start)')


def _to_mission_frame(ax, ctx, fmt_vision, extent=None):
    """
    Rotate and translate everything drawn on ax into the mission frame (see
    MissionContext.mission_frame): x lateral, y forward, origin at the start. The data stay
    in VISION; only the drawing transform changes. extent: VISION area to frame (default
    the whole common grid).
    """
    ox, oy, yaw = ctx.mission_frame()
    T = Affine2D().translate(-ox, -oy).rotate(np.pi / 2 - yaw)
    for art in list(ax.images) + list(ax.lines) + list(ax.collections) + list(ax.patches) + list(ax.texts):
        art.set_transform(T + ax.transData)
    x0, x1, y0, y1 = ctx.extent if extent is None else extent
    corners = T.transform(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]))
    ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
    ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
    inv = T.inverted()

    def fmt(x, y):
        xv, yv = inv.transform((x, y))
        return f"mission ({x:.2f}, {y:.2f}) | VISION " + fmt_vision(xv, yv)
    ax.format_coord = fmt
    ax.set_xlabel("lateral [m] (mission frame: origin at the start)")
    ax.set_ylabel("forward [m]")


def draw_global(ax, ctx, i, final=False, raw=False, bg='ostacoli', navgraph=False, mframe=False):
    """
    Global view at scan i (or at the end of the mission with final=True). raw: raw mosaic.
    bg: background, see BACKGROUNDS. navgraph: draw the nav graph (see load_navgraph).
    mframe: mission frame instead of VISION (see _to_mission_frame). Returns the object for
    the height colour bar (None for the 'ostacoli' background).
    """
    ax.clear()
    upto = len(ctx.scans) - 1 if final else i
    d = ctx.scans[upto]
    t_now = float(d['time']) if 'time' in d else None
    use_final = final and not raw and ctx.final_map is not None
    if use_final:
        # End of mission: the global map saved by the planner, which also contains the
        # start-of-attempt scans (not saved in scans/).
        occ = ctx.final_map
    else:
        occ = ctx.mosaic(upto) if raw else ctx.robot_map(upto)
    height = None
    if bg != 'ostacoli':
        if final and ctx.final_terrain is not None:
            height = ctx.final_terrain
        else:
            height = ctx.terrain(upto)
            if ctx.final_terrain is not None:
                # where the body passed and the height is missing: the one seen later (HEIGHT UNDER THE BODY)
                fill = np.isnan(height) & ctx.walked(upto) & np.isfinite(ctx.final_terrain)
                height[fill] = ctx.final_terrain[fill]
    mappable = None
    if bg == 'ostacoli':
        ax.imshow(occ, origin='lower', cmap=MOSAIC_CMAP, vmin=0, vmax=2, interpolation='nearest',
                  extent=ctx.extent, zorder=0)
    else:
        norm = ctx.terrain_norm()
        ax.imshow(_background_rgb(occ, height, bg, norm, ctx.res), origin='lower',
                  interpolation='nearest', extent=ctx.extent, zorder=0)
        mappable = plt.cm.ScalarMappable(norm=norm, cmap=TERRAIN_CMAP)
        _draw_zero_level(ax, height, ctx.extent)
    ng = ctx.navgraph() if navgraph else None
    if ng is not None:
        draw_navgraph(ax, ng, None if final else t_now)
    _hover_global(ax, ctx, occ, height, ng)

    vis = ctx.vis_before(None if final else t_now)
    vd = vis[1] if vis else None
    try:
        _draw_cells(ax, vd)
    except Exception as e:
        print(f"[viewer] cells not drawn: {e}")
    if vd is not None:
        xy = {int(k): tuple(p) for k, p in zip(_arr(vd, 'prm_node_ids', ()), _arr(vd, 'prm_node_xy', (2,)))}
        # This panel draws ALL the graph edges (up to ~90 000), as a cached image
        # prepared in the background at opening (see EDGES AS AN IMAGE).
        img, gext = _global_edges_image(ctx, vis[0], vd)
        if img is not None:
            ax.imshow(img, origin='lower', interpolation='nearest', zorder=2, extent=gext)
        stops = np.array([xy[int(k)] for k in _arr(vd, 'prm_stop_nodes', ()) if int(k) in xy]).reshape(-1, 2)
        if len(stops):
            ax.plot(stops[:, 0], stops[:, 1], '.', color='tab:orange', ms=5, zorder=4, label='stop nodes')
        cp = _arr(vd, 'chosen_point').astype(float).ravel()
        if cp.size == 2 and np.all(np.isfinite(cp)):
            ax.plot(*cp, '*', color='green', ms=15, mec='k', zorder=6, label='cell goal')

    # real trajectory (trajectory.csv) or, if missing, the positions at the scans
    if ctx.traj is not None and len(ctx.traj):
        tr = ctx.traj if (final or t_now is None) else ctx.traj[ctx.traj[:, 0] <= t_now + 0.5]
        runs, gaps = _traj_runs(tr)
        halo = [pe.Stroke(linewidth=3.2, foreground='white', alpha=0.8), pe.Normal()]
        labelled = set()
        for r, kind in runs:
            lab, col = {1: ('real path (GraphNav)', 'darkorange'),
                        2: ('correction of the return to wp_0 (fiducial)', 'crimson')}.get(
                kind, ('real path', 'tab:blue'))
            ax.plot(r[:, 0], r[:, 1], '-', color=col, lw=2.5 if kind == 2 else 1.5,
                    zorder=6 if kind == 2 else 5, path_effects=halo, label=None if lab in labelled else lab)
            labelled.add(lab)
        for k, (a, b) in enumerate(gaps):
            # no white border: under a dashed line the continuous border makes it look like a solid line
            ax.plot([a[0], b[0]], [a[1], b[1]], linestyle=(0, (1.5, 2.5)), color='navy', lw=1.3, zorder=5,
                    label='unrecorded leg (GraphNav or back-off)' if k == 0 else None)
    else:
        track = np.array([s['robot_xyz'][:2] for s in ctx.scans[:upto + 1]])
        ax.plot(track[:, 0], track[:, 1], '-', color='tab:blue', lw=1.5, alpha=0.8, zorder=5,
                label='robot (at the scans)')

    for s in ctx.scans[:upto + 1]:
        if str(s['decision_kind']) == 'blocked':
            ax.plot(*s['frontier_stop_xy'], 's', color='red', ms=11, mfc='none', mew=2, zorder=7)
    if not final:
        pl = d['polyline']
        ax.plot(pl[:, 0], pl[:, 1], '-', color='magenta', lw=2.5, zorder=6, label='planned path')
        x0, x1, y0, y1 = _extent(d)
        ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor='cyan', lw=2, zorder=6))
        reason = modern(str(d['frontier_reason']))
        ax.plot(*d['frontier_stop_xy'], 'X', color=REASON_COLORS.get(reason, 'k'), ms=11, mec='white', zorder=8)
    rx, ry = d['robot_xyz'][:2]
    ax.plot(rx, ry, 'o', color='blue', ms=9, mec='white', zorder=9)

    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
    n_blk = sum(str(s['decision_kind']) == 'blocked' for s in ctx.scans[:upto + 1])
    if use_final:
        title = "End of mission -- global map saved by the planner ('m': raw mosaic)"
    else:
        title = ("End of mission" if final else f"Global map at scan {upto + 1}/{len(ctx.scans)}")
        title += (" -- raw mosaic, the last scan wins ('m')" if raw
                  else " -- the robot's map, confirmed obstacles ('m')")
    legend = {
        'ostacoli': "red = obstacle seen, light green = free, white = never seen",
        'terreno': "background: terrain height relative to the mission start; white = no height",
        'fusione': "background: terrain height, red = obstacle, grey = free without height, white = never seen",
    }[bg]
    ax.set_title(f"{title}\n{legend} ('t'); red squares = blocks ({n_blk})"
                 + (f"\ngraph: {vis[0]}" if vis else ""), fontsize=8)
    ax.set_xlabel("X [m] (VISION)")
    if mframe:
        _to_mission_frame(ax, ctx, ax.format_coord)
    ax.legend(loc='upper left', fontsize=7, framealpha=0.8)
    return mappable


# Positions [left, bottom, width, height] as fractions of the window (see main).
LAYOUT = {
    'scan': [0.035, 0.055, 0.405, 0.835],
    'scan_cbar': [0.447, 0.12, 0.008, 0.70],
    'global': [0.535, 0.055, 0.405, 0.835],
    'global_cbar': [0.947, 0.12, 0.008, 0.70],
}


def _maximize_window(fig):
    """Open the window maximized, if the backend allows it (Qt, Tk, wx)."""
    try:
        mng = fig.canvas.manager
        win = getattr(mng, 'window', None)
        if win is None:
            return
        if hasattr(win, 'showMaximized'):          # Qt
            win.showMaximized()
        elif hasattr(win, 'state'):                # Tk
            try:
                win.state('zoomed')                # Windows, macOS
            except Exception:
                win.attributes('-zoomed', True)    # Linux
        elif hasattr(win, 'Maximize'):             # wx
            win.Maximize(True)
    except Exception:
        pass


def _latest_mission_folder():
    """
    Without arguments (e.g. the editor's Run key): the latest mission in MissionMap/,
    searched next to this file and in the current folder. If there is none, a window to choose it.
    """
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cands = []
    for base in (here, os.getcwd(), os.path.dirname(here)):
        cands += [d for d in glob.glob(os.path.join(base, "MissionMap", "Mission_*")) if os.path.isdir(d)]
    if cands:
        return max(set(cands), key=os.path.getmtime)
    try:
        import tkinter
        from tkinter import filedialog
        root = tkinter.Tk()
        root.withdraw()
        return filedialog.askdirectory(title="Mission folder (inside MissionMap)") or None
    except Exception:
        return None


def main():
    skip = {sys.argv.index(o) + 1 for o in ("--png", "--background") if o in sys.argv}
    args = [a for k, a in enumerate(sys.argv[1:], 1) if not a.startswith("--") and k not in skip]
    if not args:
        found = _latest_mission_folder()
        if not found:
            print(__doc__)
            print("No MissionMap/Mission_* folder found next to this file.")
            return 1
        print(f"No folder given: opening the latest mission, {found}")
        args = [found]
    target = args[0]
    # The mission name alone is enough too (e.g. 13-10-2026_12-40-38), as for mission_report.py.
    by_name = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "MissionMap",
                           "Mission_" + target.replace("Mission_", ""))
    if not os.path.exists(target) and os.path.isdir(by_name):
        target = by_name
    out_dir = sys.argv[sys.argv.index("--png") + 1] if PNG_MODE else None
    bg0 = 'ostacoli'
    ng0 = "--navgraph" in sys.argv
    mf0 = "--mission-frame" in sys.argv
    if "--background" in sys.argv:
        bg0 = sys.argv[sys.argv.index("--background") + 1]
        if bg0 not in BACKGROUNDS:
            print(f"--background: one of {', '.join(BACKGROUNDS)}")
            return 1

    if os.path.isdir(target):
        scan_files = sorted(glob.glob(os.path.join(target, "scans", "scan_*.npz")))
        vis_files = sorted(glob.glob(os.path.join(target, "*_vis.npz")))
    else:
        scan_files = [target] if os.path.basename(target).startswith("scan_") else []
        vis_files = [target] if target.endswith("_vis.npz") else []
    scans = [_load(f) for f in scan_files]
    n_loop = len(scans)
    print(f"{len(scans)} scans, {len(vis_files)} PRM views")
    folder = target if os.path.isdir(target) else os.path.dirname(os.path.dirname(os.path.abspath(target)))
    ctx = MissionContext(folder, scans, scan_files, vis_files) if scans else None
    if ctx is not None:
        scans, scan_files = ctx.scans, ctx.scan_files     # with the scans outside the loop
        print(f"  plus {len(scans) - n_loop} start-of-attempt and final scans, from the PRM views")

    if PNG_MODE:
        os.makedirs(out_dir, exist_ok=True)
        for k, (f, d) in enumerate(zip(scan_files, scans)):
            fig, (axl, axr) = plt.subplots(1, 2, figsize=(20, 9.5))
            im, label = draw_scan(axl, d, 5, ctx=ctx)          # obstacle mask
            overlay_prm_local(axl, ctx, k)
            if mf0:
                _to_mission_frame(axl, ctx, axl.format_coord, extent=_extent(d))
            fig.colorbar(im, ax=axl, label=label, shrink=0.8)
            draw_global(axr, ctx, k, bg=bg0, navgraph=ng0, mframe=mf0)
            fig.tight_layout()
            png = os.path.basename(f).replace(".npz", ".png")
            m = _VIS_SCAN_NAME.search(os.path.basename(f))
            if m:                                         # scan outside the loop
                png = (f"scan_it{int(m.group(1)):03d}_cell{m.group(2)}_{m.group(3)}_000_start.png"
                       if m.group(2) else f"scan_it{int(m.group(1)):03d}_finale.png")
            fig.savefig(os.path.join(out_dir, png), dpi=100)
            plt.close(fig)
        for f in vis_files:
            fig, ax = plt.subplots(figsize=(11, 11))
            draw_vis_bundle(ax, _load(f))
            fig.savefig(os.path.join(out_dir, os.path.basename(f).replace(".npz", ".png")), dpi=110)
            plt.close(fig)
        for bg in (BACKGROUNDS if scans else ()):
            fig, ax = plt.subplots(figsize=(12, 12))
            m = draw_global(ax, ctx, len(scans) - 1, final=True, bg=bg, navgraph=ng0, mframe=mf0)
            if m is not None:
                _mark_zero(fig.colorbar(m, ax=ax, label="height relative to the mission start (m)", shrink=0.7))
            fig.savefig(os.path.join(out_dir, "overview.png" if bg == 'ostacoli' else f"overview_{bg}.png"),
                        dpi=110)
            plt.close(fig)
        print(f"Figures saved in {out_dir}")
        return 0

    if "--overview" in sys.argv or not scans:
        fig, ax = plt.subplots(figsize=(11, 11))
        if scans:
            m = draw_global(ax, ctx, len(scans) - 1, final=True, bg=bg0, navgraph=ng0, mframe=mf0)
            if m is not None:
                _mark_zero(fig.colorbar(m, ax=ax, label="height relative to the mission start (m)", shrink=0.7))
        elif vis_files:
            draw_vis_bundle(ax, _load(vis_files[0]))
        plt.show()
        return 0

    # The viewer keys must not do anything else: for matplotlib 'g' is the grid,
    # 'o' the zoom, 'r' the initial view, 'l' the log scale, and the arrows and 'c' / 'v' the
    # previous / next zoom view (scrolling after a zoom the view changed).
    ours = {'left', 'right', 'up', 'down', 'g', 'o', 'm', 't', 'n', 'r', 'b', 'B', 'c', 'C', 'v', 'V',
            'e', 'l', *LAYER_KEYS, *[str(k) for k in range(10)]}
    for km in [k for k in plt.rcParams if k.startswith('keymap.')]:
        plt.rcParams[km] = [k for k in plt.rcParams[km] if k not in ours]
    state = {'i': 0, 'layer': 0, 'overview': False, 'prm': True, 'raw': False,
             'bg': bg0, 'nav': ng0, 'rot': mf0, 'auto_lim': None, 'legend': True}
    # Fixed layout (2026-10-26): the two panels and their colour bars in given
    # positions, which use almost the whole window. With plt.subplots and the bars added "next to"
    # them, full screen (1920 x 1080) the panels were 541 x 541 px; this way about 800 x 800.
    # The bars have their own axes: redrawing them does not shrink the panels.
    fig = plt.figure(figsize=(18, 9.5))
    ax = fig.add_axes(LAYOUT['scan'])
    cax = fig.add_axes(LAYOUT['scan_cbar'])
    axg = fig.add_axes(LAYOUT['global'])
    cax_g = fig.add_axes(LAYOUT['global_cbar'])
    for a in (ax, axg):
        a.set_anchor('C')
    _maximize_window(fig)
    # Height bar of the right panel: created only once and hidden with the
    # 'ostacoli' background.
    cbar_g = fig.colorbar(plt.cm.ScalarMappable(norm=ctx.terrain_norm(), cmap=TERRAIN_CMAP), cax=cax_g,
                          label="height relative to the mission start (m)")
    _mark_zero(cbar_g)

    def redraw():
        d = scans[state['i']]
        im, label = draw_scan(ax, d, state['layer'], ctx=ctx,
                              title_prefix=f"[{state['i'] + 1}/{len(scans)}] "
                                           f"{os.path.basename(scan_files[state['i']])}\n")
        cax.clear()
        fig.colorbar(im, cax=cax, label=label)
        if state['prm']:
            overlay_prm_local(ax, ctx, state['i'])
        if state['rot']:
            _to_mission_frame(ax, ctx, ax.format_coord, extent=_extent(d))
        # The zoom of the right panel is kept: if the limits differ from the automatic ones of the
        # previous drawing the user changed them, and they are restored after the redraw.
        lim = (axg.get_xlim(), axg.get_ylim())
        zoomed = state['auto_lim'] is not None and lim != state['auto_lim']
        m = draw_global(axg, ctx, state['i'], final=state['overview'], raw=state['raw'], bg=state['bg'],
                        navgraph=state['nav'], mframe=state['rot'])
        state['auto_lim'] = (axg.get_xlim(), axg.get_ylim())
        if zoomed:
            axg.set_xlim(lim[0]); axg.set_ylim(lim[1])
        for a in (ax, axg):
            if a.get_legend() is not None:
                a.get_legend().set_visible(state['legend'])
        cbar_g.ax.set_visible(m is not None)
        fig.canvas.draw_idle()

    def on_key(ev):
        if ev.key == 'right':
            state['i'] = min(len(scans) - 1, state['i'] + 1); state['overview'] = False
        elif ev.key == 'left':
            state['i'] = max(0, state['i'] - 1); state['overview'] = False
        elif ev.key == 'up':
            state['i'] = min(len(scans) - 1, state['i'] + 10); state['overview'] = False
        elif ev.key == 'down':
            state['i'] = max(0, state['i'] - 10); state['overview'] = False
        elif ev.key in [str((k + 1) % 10) for k in range(min(len(LAYERS), 10))]:
            state['layer'] = (int(ev.key) - 1) % 10   # '0' = tenth layer
        elif ev.key in LAYER_KEYS and LAYER_KEYS[ev.key] < len(LAYERS):
            state['layer'] = LAYER_KEYS[ev.key]
        elif ev.key == 'l':
            state['layer'] = (state['layer'] + 1) % len(LAYERS)
        elif ev.key in ('b', 'B', 'c', 'C', 'v', 'V'):
            kind = {'b': 'block', 'c': 'cell', 'v': 'graphnav'}[ev.key.lower()]
            lst = ctx.events()[kind]
            fwd = ev.key.islower()
            cand = [k for k in lst if (k > state['i'] if fwd else k < state['i'])]
            what = {'block': 'block', 'cell': 'tentativo', 'graphnav': 'GraphNav move'}[kind]
            if not cand:
                print(f"[viewer] no {what} {'after' if fwd else 'before'} this scan "
                      f"({len(lst)} in all).")
                return
            state['i'] = cand[0] if fwd else cand[-1]
            state['overview'] = False
            print(f"[viewer] {what} {lst.index(state['i']) + 1}/{len(lst)}: scan "
                  f"{state['i'] + 1}/{len(scans)}")
        elif ev.key == 'e':
            state['legend'] = not state['legend']
        elif ev.key == 'o':
            state['overview'] = not state['overview']
        elif ev.key == 'g':
            state['prm'] = not state['prm']
        elif ev.key == 'm':
            state['raw'] = not state['raw']
        elif ev.key == 'n':
            state['nav'] = not state['nav']
        elif ev.key == 'r':
            state['rot'] = not state['rot']
            state['auto_lim'] = None        # different coordinates: the zoom is not kept
        elif ev.key == 't':
            state['bg'] = BACKGROUNDS[(BACKGROUNDS.index(state['bg']) + 1) % len(BACKGROUNDS)]
        else:
            return
        redraw()

    fig.canvas.mpl_connect('key_press_event', on_key)
    print("Left/right arrows: scan | up/down: 10 scans | 1-9, 0: left layer | "
          "o: right end of mission / moment of the scan | g: left graph yes/no | "
          "m: right robot map / raw mosaic | t: background obstacles / terrain / fusion | "
          "n: nav graph yes/no | r: mission frame yes/no | b/c/v (B/C/V): block / attempt / "
          "next (previous) GraphNav | e: legends yes/no | i, x, l: intensity, no_step, next layer | "
          "value under the mouse at the bottom right")
    keys = {v: k for k, v in LAYER_KEYS.items()}
    for k, layer in enumerate(LAYERS):
        print(f"  {keys.get(k, (k + 1) % 10)}: {layer[0]}")
    ev = ctx.events()
    print(f"  jumps: {len(ev['block'])} blocks, {len(ev['cell'])} attempts, "
          f"{len(ev['graphnav'])} GraphNav moves")
    redraw()
    # The edge images of the global panel are prepared in the background.
    threading.Thread(target=_prebuild_global_images, args=(ctx,), daemon=True).start()
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())