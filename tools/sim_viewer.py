#!/usr/bin/env python3
"""
Simulation viewer: what is really there, what the sensors return, and what the planner chose.

It runs in its own terminal next to the simulator and the mission. It only READS: the true
robot pose and the world from the simulator (as a second client, it never sends commands),
and the files the mission already writes in MissionMap/. Needs numpy and matplotlib, not the SDK.

    python3 tools/sim_viewer.py                        # simulator on 127.0.0.1:8765, latest mission
    python3 tools/sim_viewer.py --sim HOST:PORT
    python3 tools/sim_viewer.py --mission 12-11-2026_15-02-10
    python3 tools/sim_viewer.py --png out.png          # one figure, no window

LEFT, the simulated world as it really is (the mission never sees this directly):
  background  ground height (colour and contour lines every 25 cm); grey = flat world
  tan         obstacles of the world file (trunks, boxes)
  dots        the latest LiDAR scan: with Isaac Sim the simulator's own lidar (RTX Ouster OS1,
              it sees the drawn trees, leaves included), otherwise an ideal VLP-16 cast against
              the world file. Ground hits coloured by their distance from the robot (yellow
              near, blue far: the rings move with the robot; grass below 30 cm counts as ground),
              red = something between 30 cm and 1 m, in the robot's way, small green = above the
              robot (crowns). Black cross = where the scan was taken. The title says its number
              and how old it is: a new number at every refresh means the lidar is live.
  blue box    the window of Spot's local grid (3.84 m), the only area the mission perceives
  blue line   where the robot really went; arrow = the robot now

RIGHT, the planner, from the mission's latest PRM view (iteration_*_vis.npz):
  edges       the PRM, coloured by cost per metre (green cheap, red expensive): slope and
              roughness make an edge cost more than its length
  thick blue  the path the mission CHOSE
  dashed      the SHORTEST path by distance between the same two points, if different: the
              gap between the two is what the cost function decided
  cells       green visited, red blocked, dashed to explore
  red dots    obstacle cells the mission saw in that scan
In the title: length and cost of the two paths.

Keys: f follow the robot / whole area,  l LiDAR on / off,  space pause,  q quit.
"""
import argparse
import glob
import heapq
import math
import os
import sys
import time

import numpy as np
import matplotlib

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from spot_explorer.sim.controller import yaw_of                 # noqa: E402
from spot_explorer.sim.lidar import Lidar, scan_to_world       # noqa: E402
from spot_explorer.sim.link import SimLink, SimLinkError, parse_address   # noqa: E402
from spot_explorer.sim.world import GRID_CELL_M, GRID_CELLS, World       # noqa: E402

REFRESH_S = 0.5
FOLLOW_HALF_M = 9.0
GROUND_HIT_TOL_M = 0.08
ROBOT_HEIGHT_M = 1.0            # hits up to this height above the ground are in the robot's way
STEP_OVER_M = 0.30              # below this, ground and low grass: Spot steps over it


# --- mission files ---------------------------------------------------------------------------
def latest_vis(mission):
    """(path, dict) of the newest PRM view that has a graph, or (None, None)."""
    root = os.path.join(HERE, "MissionMap")
    if mission and os.path.isdir(mission):
        folder = mission
    elif mission:
        folder = os.path.join(root, mission if mission.startswith("Mission_") else "Mission_" + mission)
    else:
        cands = [d for d in glob.glob(os.path.join(root, "Mission_*")) if os.path.isdir(d)]
        if not cands:
            return None, None
        folder = max(cands, key=os.path.getmtime)
    files = sorted(glob.glob(os.path.join(folder, "*_vis.npz")), key=os.path.getmtime, reverse=True)
    for path in files[:6]:
        try:
            with np.load(path, allow_pickle=False) as z:
                d = {k: z[k] for k in z.files}
        except Exception:
            continue                      # still being written
        if 'prm_edges' in d and len(d['prm_edges']):
            return path, d
    return None, None


def graph_of(vis):
    pos = {int(n): (float(x), float(y)) for n, (x, y) in zip(vis['prm_node_ids'], vis['prm_node_xy'])}
    adj = {}
    for (a, b), w in zip(vis['prm_edges'], vis['prm_edge_weights']):
        a, b = int(a), int(b)
        if a in pos and b in pos:
            length = math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
            adj.setdefault(a, {})[b] = (float(w), length)
            adj.setdefault(b, {})[a] = (float(w), length)
    return pos, adj


def path_nodes(vis, pos):
    """Node ids of the chosen path (its points are node positions); [] if they do not match."""
    pts = np.asarray(vis.get('chosen_path', np.empty((0, 2)))).reshape(-1, 2)
    if len(pts) < 2 or not pos:
        return []
    ids = np.array(list(pos.keys()))
    xy = np.array([pos[i] for i in ids])
    out = []
    for p in pts:
        d = np.hypot(xy[:, 0] - p[0], xy[:, 1] - p[1])
        k = int(np.argmin(d))
        if d[k] > 0.08:
            return []
        if not out or out[-1] != int(ids[k]):
            out.append(int(ids[k]))
    return out


def path_totals(nodes, adj):
    """(length, cost) along the edges; cost is None if a step is not an edge of the graph."""
    length, cost = 0.0, 0.0
    for a, b in zip(nodes[:-1], nodes[1:]):
        if b not in adj.get(a, {}):
            return None, None
        w, d = adj[a][b]
        length, cost = length + d, cost + w
    return length, cost


def shortest_by_length(adj, start, goal):
    dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, float('inf')):
            continue
        for v, (_, length) in adj.get(u, {}).items():
            nd = d + length
            if nd < dist.get(v, float('inf')):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]


# --- drawing ---------------------------------------------------------------------------------
def draw_shapes(ax, world, **kw):
    import matplotlib.patches as patches
    for s in world.shapes:
        if s['type'] == 'cylinder':
            ax.add_patch(patches.Circle((s['x'], s['y']), s['r'], **kw))
        else:
            a = math.radians(s['yaw_deg'])
            c, sn = math.cos(a), math.sin(a)
            base = np.array([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]]) * [s['sx'], s['sy']]
            ax.add_patch(patches.Polygon(base @ np.array([[c, sn], [-sn, c]]) + [s['x'], s['y']], **kw))


def draw_robot(ax, st, color='blue', z=9):
    yaw = st['yaw']
    ax.annotate('', xy=(st['x'] + 0.9 * math.cos(yaw), st['y'] + 0.9 * math.sin(yaw)),
                xytext=(st['x'], st['y']), zorder=z,
                arrowprops=dict(arrowstyle='-|>', color=color, lw=2.2, mutation_scale=18))
    ax.plot(st['x'], st['y'], 'o', color=color, ms=6, zorder=z)


def view_limits(follow, st, world, trail, vis):
    if follow:
        return (st['x'] - FOLLOW_HALF_M, st['x'] + FOLLOW_HALF_M,
                st['y'] - FOLLOW_HALF_M, st['y'] + FOLLOW_HALF_M)
    xs = [st['x']] + [p[0] for p in trail] + [s['x'] for s in world.shapes]
    ys = [st['y']] + [p[1] for p in trail] + [s['y'] for s in world.shapes]
    if vis is not None and len(vis.get('env_cells', ())):
        half = float(vis.get('env_cell_size', 2.0))
        xs += [float(vis['env_cells'][:, 2].min()) - half, float(vis['env_cells'][:, 2].max()) + half]
        ys += [float(vis['env_cells'][:, 3].min()) - half, float(vis['env_cells'][:, 3].max()) + half]
    cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
    half = 0.5 * max(max(xs) - min(xs), max(ys) - min(ys), 6.0) + 1.5
    return cx - half, cx + half, cy - half, cy + half


def draw_truth(ax, world, st, trail, scan, lim, show_lidar):
    import matplotlib.patches as patches
    x0, x1, y0, y1 = lim
    if world.is_flat:
        ax.set_facecolor('0.93')
        relief = "flat ground"
    else:
        gx, gy = np.meshgrid(np.linspace(x0, x1, 180), np.linspace(y0, y1, 180))
        gz = world.height(gx, gy)
        ax.imshow(gz, extent=(x0, x1, y0, y1), origin='lower', cmap='terrain', alpha=0.75,
                  vmin=min(gz.min(), -0.5), vmax=max(gz.max(), 0.5), zorder=0)
        levels = np.arange(math.floor(gz.min() * 4) / 4, gz.max() + 0.25, 0.25)
        if len(levels) > 1:
            cs = ax.contour(gx, gy, gz, levels=levels, colors='k', linewidths=0.35, alpha=0.5, zorder=1)
            ax.clabel(cs, levels[::2], fmt='%.1f', fontsize=6)
        relief = "ground %+.2f m here" % (world.height_at(st['x'], st['y']) - world.ground_z)
    draw_shapes(ax, world, facecolor='tan', edgecolor='0.35', lw=0.6, alpha=0.6, zorder=4)
    n_obst = 0
    if scan is not None and show_lidar and len(scan['points']):
        p = scan['points']
        h = p[:, 2] - world.height(p[:, 0], p[:, 1])
        ground, low, high = h <= STEP_OVER_M, (h > STEP_OVER_M) & (h <= ROBOT_HEIGHT_M), h > ROBOT_HEIGHT_M
        n_obst = int(low.sum())
        dist = np.hypot(p[:, 0] - scan['origin'][0], p[:, 1] - scan['origin'][1])
        ax.scatter(p[ground, 0], p[ground, 1], s=2.0, c=dist[ground], cmap='viridis_r', vmin=0.0, vmax=12.0,
                   edgecolors='none', alpha=0.9, zorder=3)
        ax.scatter(p[high, 0], p[high, 1], s=1.5, c='forestgreen', edgecolors='none', alpha=0.35, zorder=3)
        ax.scatter(p[low, 0], p[low, 1], s=5, c='red', edgecolors='none', zorder=6)
        ax.plot(scan['origin'][0], scan['origin'][1], '+', color='k', ms=10, mew=1.5, zorder=9)
    half = 0.5 * GRID_CELLS * GRID_CELL_M
    ax.add_patch(patches.Rectangle((st['x'] - half, st['y'] - half), 2 * half, 2 * half, fill=False,
                                   edgecolor='deepskyblue', lw=1.6, zorder=7))
    if len(trail) > 1:
        t = np.array(trail)
        ax.plot(t[:, 0], t[:, 1], '-', color='blue', lw=1.4, zorder=8)
    draw_robot(ax, st)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.set_xlabel("x, ahead at the start (m)")
    ax.set_ylabel("y, left at the start (m)")
    if scan is not None and show_lidar:
        lidar = "%s: %d points, %d at the robot's height; scan #%d, %.1f s old" % (
            scan['source'], len(scan['points']), n_obst, scan['count'], scan['age'])
    else:
        lidar = "LiDAR off" if not show_lidar else "LiDAR: no scan yet"
    ax.set_title("SIMULATED WORLD '%s' (truth)\nrobot (%.2f, %.2f), heading %.0f deg, roll %+.1f pitch %+.1f deg, %s\n%s"
                 % (world.name or 'empty', st['x'], st['y'], math.degrees(st['yaw']),
                    st['roll'], st['pitch'], relief, lidar), fontsize=9)


def draw_planner(ax, cax, fig, vis, vis_path, world, st, lim, state):
    """cax: an axes of its own for the colour bar, made once. (A colour bar made with ax=ax
    takes its room from the panel each time: refreshed twice a second the panel shrank until
    it disappeared.)"""
    import matplotlib.patches as patches
    from matplotlib.collections import LineCollection
    cax.clear()
    cax.set_visible(False)
    ax.set_xlim(lim[0], lim[1])
    ax.set_ylim(lim[2], lim[3])
    ax.set_aspect('equal')
    ax.set_xlabel("x (m)")
    draw_shapes(ax, world, facecolor='none', edgecolor='0.55', lw=0.6, ls=':', zorder=1)
    draw_robot(ax, st)
    if vis is None:
        ax.set_title("PLANNER\nno PRM view yet: waiting for a mission in MissionMap/", fontsize=9)
        return
    # cells
    if len(vis.get('env_cells', ())):
        size, yaw = float(vis['env_cell_size']), float(vis['env_origin_yaw'])
        c, sn = math.cos(yaw), math.sin(yaw)
        base = np.array([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]]) * size @ np.array([[c, sn], [-sn, c]])
        for (r, col, x, y), status in zip(vis['env_cells'], vis['env_cell_status']):
            status = str(status)
            kw = (dict(edgecolor='darkgreen', facecolor='lightgreen', alpha=0.25) if status == '1' else
                  dict(edgecolor='darkred', facecolor='lightcoral', alpha=0.35) if status == '-1' else
                  dict(edgecolor='gray', facecolor='none', alpha=0.6, ls='--'))
            ax.add_patch(patches.Polygon(base + [x, y], lw=0.8, zorder=2, **kw))
    # obstacle cells the mission saw
    pts, mask = np.asarray(vis.get('pts', ())), np.asarray(vis.get('obstacle_mask', ()))
    if pts.ndim == 2 and len(pts) and len(mask) == len(pts):
        o = pts[mask < 0]
        ax.scatter(o[:, 0], o[:, 1], s=2, c='red', edgecolors='none', alpha=0.5, zorder=3)
    pos, adj = graph_of(vis)
    segs, per_m = [], []
    for a in adj:
        for b, (w, length) in adj[a].items():
            if a < b and length > 1e-6 and np.isfinite(w):
                segs.append((pos[a], pos[b]))
                per_m.append(w / length)
    title = "PLANNER, %s\n%d nodes, %d edges" % (os.path.basename(vis_path).replace('_vis.npz', ''),
                                                 len(pos), len(segs))
    if segs:
        per_m = np.array(per_m)
        hi = max(float(np.percentile(per_m, 95)), float(per_m.min()) + 1e-6)
        lc = LineCollection(segs, cmap='RdYlGn_r', linewidths=1.0, alpha=0.8, zorder=4)
        lc.set_array(per_m)
        lc.set_clim(float(per_m.min()), hi)
        ax.add_collection(lc)
        cax.set_visible(True)
        fig.colorbar(lc, cax=cax).set_label("edge cost per metre", fontsize=8)
    chosen = path_nodes(vis, pos)
    path_pts = np.asarray(vis.get('chosen_path', np.empty((0, 2)))).reshape(-1, 2)
    if len(path_pts) >= 2:
        ax.plot(path_pts[:, 0], path_pts[:, 1], '-', color='blue', lw=3.0, zorder=6, label='chosen')
        ax.plot(path_pts[-1, 0], path_pts[-1, 1], '*', color='gold', mec='k', ms=15, zorder=8)
    if len(chosen) >= 2:
        length, cost = path_totals(chosen, adj)
        short = shortest_by_length(adj, chosen[0], chosen[-1])
        s_len, s_cost = path_totals(short, adj) if short else (None, None)
        if length is not None:
            title += "\nchosen: %.2f m, cost %.2f" % (length, cost)
            if short and short != chosen and s_len is not None:
                xy = np.array([pos[i] for i in short])
                ax.plot(xy[:, 0], xy[:, 1], '--', color='k', lw=1.8, zorder=7, label='shortest')
                title += "   |   shortest: %.2f m, cost %.2f" % (s_len, s_cost)
                title += "\nthe mission walks %.2f m more to save %.2f of cost" % (length - s_len, s_cost - cost) \
                    if s_cost > cost else "\n(the shortest is also cheaper by these weights: penalties, cell limits or the stable-plan margin decided)"
            else:
                title += "   (it is also the shortest by distance)"
        else:
            title += "\nchosen path: %d points (not all on edges of this graph)" % len(path_pts)
    elif len(path_pts) < 2:
        title += "\nno path in this view (scan only, or attempt closed)"
    age = time.time() - float(vis.get('time', time.time()))
    title += "\nview saved %.0f s ago" % age if age < 36000 else ""
    ax.set_title(title, fontsize=9)


def read_state(link):
    raw = link.step(None)
    q = tuple(raw['q'])
    w, x, y, z = q
    return {'x': raw['p'][0], 'y': raw['p'][1], 'z': raw['p'][2], 'q': q, 'yaw': yaw_of(q),
            'roll': math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            'pitch': math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))),
            't': float(raw.get('t', 0.0))}


def main():
    ap = argparse.ArgumentParser(description="Simulation viewer: truth, LiDAR and planner choices")
    ap.add_argument('--sim', default=os.environ.get('SPOT_SIM', ''), metavar='HOST:PORT')
    ap.add_argument('--mission', default=None, help="mission name or folder (default: the latest)")
    ap.add_argument('--png', default=None, help="write one figure to this file and exit")
    ap.add_argument('--lidar-range', type=float, default=20.0)
    ap.add_argument('--no-lidar', action='store_true')
    ap.add_argument('--ideal-lidar', action='store_true',
                    help="cast the ideal VLP-16 against the world file even when the simulator has a lidar")
    args = ap.parse_args()
    if args.png:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    host, port = parse_address(args.sim)
    try:
        link = SimLink(host, port)
    except SimLinkError as e:
        sys.exit("[viewer] %s" % e)
    world = World.from_dict(link.world())
    lidar = Lidar(max_range=args.lidar_range)
    # The simulator's own lidar when it has one (Isaac Sim), on a link of its own.
    sensor_link = None
    if 'lidar' in link.ops and not args.ideal_lidar:
        try:
            if link.sensors().get('lidar') is not None:
                sensor_link = SimLink(host, port)
        except SimLinkError:
            sensor_link = None
    ui = {'follow': False, 'lidar': not args.no_lidar, 'pause': False, 'quit': False,
          'scan_t': None, 'scan_wall': None, 'scan_count': 0}
    trail = []

    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(17, 8.5))
    fig.subplots_adjust(left=0.05, right=0.93, top=0.86, bottom=0.07, wspace=0.14)
    pos_r = ax_r.get_position()
    cax = fig.add_axes([pos_r.x1 + 0.01, pos_r.y0 + 0.1 * pos_r.height, 0.012, 0.8 * pos_r.height])

    def get_scan(st):
        """The latest scan in the world frame, with its source, number and age."""
        if sensor_link is not None:
            try:
                raw = sensor_link.lidar()
            except SimLinkError:
                return None
            if raw['t'] != ui['scan_t']:
                ui['scan_t'], ui['scan_wall'] = raw['t'], time.time()
                ui['scan_count'] += 1
            pts = scan_to_world(raw)
            origin = np.asarray(raw['p']) + np.asarray(raw['mount_p'])
            return {'points': pts, 'origin': origin, 'source': "Isaac Sim lidar (%s)" % raw.get('name', ''),
                    'count': ui['scan_count'], 'age': max(0.0, st['t'] - raw['t'])}
        s = lidar.scan(world, (st['x'], st['y'], st['z']), st['q'])
        ui['scan_count'] += 1
        return {'points': s['points_world'], 'origin': s['origin'], 'source': "ideal VLP-16 (world file)",
                'count': ui['scan_count'], 'age': 0.0}

    def on_key(ev):
        if ev.key == 'f':
            ui['follow'] = not ui['follow']
        elif ev.key == 'l':
            ui['lidar'] = not ui['lidar']
        elif ev.key == ' ':
            ui['pause'] = not ui['pause']
        elif ev.key == 'q':
            ui['quit'] = True
    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.canvas.mpl_connect('close_event', lambda ev: ui.update(quit=True))

    def frame():
        st = read_state(link)
        if not trail or math.hypot(st['x'] - trail[-1][0], st['y'] - trail[-1][1]) > 0.03:
            trail.append((st['x'], st['y']))
        scan = get_scan(st) if ui['lidar'] else None
        vis_path, vis = latest_vis(args.mission)
        lim = view_limits(ui['follow'], st, world, trail, vis)
        ax_l.clear()
        ax_r.clear()
        draw_truth(ax_l, world, st, trail, scan, lim, ui['lidar'])
        draw_planner(ax_r, cax, fig, vis, vis_path, world, st, lim, ui)

    if args.png:
        frame()
        fig.savefig(args.png, dpi=110)
        print("[viewer] figure written to %s" % args.png)
        return
    print("[viewer] simulator '%s' on %s:%d. Keys: f follow, l LiDAR, space pause, q quit."
          % (link.backend, host, port))
    plt.ion()
    plt.show()
    while not ui['quit']:
        if not ui['pause']:
            try:
                frame()
            except SimLinkError as e:
                print("[viewer] %s" % e)
                break
        plt.pause(REFRESH_S)


if __name__ == '__main__':
    main()
