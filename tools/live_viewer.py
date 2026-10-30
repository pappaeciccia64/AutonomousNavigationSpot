#!/usr/bin/env python3
"""
LIVE mission viewer: a single figure that updates itself, to understand roughly what the
robot is doing. It runs in a separate terminal, next to the mission, and only reads the
files the mission already writes:
  MissionMap/Mission_<name>/scans/*.npz          one per loop turn (scan, path, goal,
                                                 frontier, decision)
  MissionMap/Mission_<name>/iteration_*_vis.npz  one per attempt (cells, PRM graph)
  MissionLogs/Mission_<name>/mission_log.txt     the log, line by line
  MissionLogs/Mission_<name>/trajectory.csv      the real trajectory, at the end of every leg
It does not touch the mission and does not use the SDK: if it is closed or hangs, the
mission does not notice.

    python3 tools/live_viewer.py                       # the latest mission, live
    python3 tools/live_viewer.py 28-10-2026_14-23-50   # a specific mission, live
    python3 tools/live_viewer.py 28-10-2026_14-23-50 --replay [--speed 20]
                                                       # finished mission, replayed in time
    python3 tools/live_viewer.py <name> --png <file.png> [--until HH:MM:SS]
                                                       # a single figure, no window

In the figure (a single layer with everything on top):
  background  map seen so far: red obstacle, light green free, white never seen
              (mosaic of the loop scans, the last one wins); inside the light-blue box
              the latest scan, darker
  cells       green visited, red blocked, hatched to explore, blue border = goal
  PRM graph   light grey
  blue        path taken (dashed = GraphNav)
  orange      path planned now; star = goal in the cell
  X           where the safe frontier stops (red obstacle, purple end of data, green end
              of path); thick green line = allowed advance
  arrow       the robot
On the right: status (cell, decision, frontier, advance, battery) and the last useful log lines.

Keys: f follow the robot / whole mission,  space pause,  q quit.
"""
import argparse
import csv
import glob
import os
import re
import sys
import textwrap
import time

import numpy as np
import matplotlib
import matplotlib.patches as patches

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # project folder
MOSAIC_RES = 0.10           # side of the mosaic cells, m
REFRESH_S = 1.0             # how often to re-read, live
LOG_TAGS = ('[PROGRESSO]', '[TARGET]', '[ATTEMPT]', '[VISITED]', '[BLOCKED]', '[BLOCCO]',
            '[SUBITO]', '[REPLAN]', '[REPLAN FALLITO]', '[ARRIVO]', '[OBIETTIVO]', '[STOP]',
            '[GRAPHNAV]', '[BATTERIA]', '[FINE]', '[FRONTIERA]', '[STALLO]', '[EMERGENZA]',
            '[QUASI]', '[PASSO FINALE]', '[RITORNO]', '[MULTINODO]', '[DEFER]', '[PRECHECK]', '[WP0]')
N_LOG_LINES = 12
REASON_COLORS = {'ostacolo': 'red', 'rugosita': 'darkorange', 'pendenza': 'darkorange',
                 'fine_dati': 'purple', 'fine_percorso': 'green'}


def _folders(nome):
    """(MissionMap folder, MissionLogs folder) of the mission; the latest if name is None."""
    if nome is None:
        cands = [d for d in glob.glob(os.path.join(HERE, "MissionMap", "Mission_*")) if os.path.isdir(d)]
        if not cands:
            sys.exit("Nessuna missione in MissionMap/.")
        mm = max(cands, key=os.path.getmtime)
    elif os.path.isdir(nome):
        mm = os.path.abspath(nome)
    else:
        mm = os.path.join(HERE, "MissionMap", nome if nome.startswith("Mission_") else f"Mission_{nome}")
    ml = os.path.join(HERE, "MissionLogs", os.path.basename(mm))
    return mm, ml


def _load_npz(path):
    """npz as a dict; None if the file cannot be read (for example it is still being written)."""
    try:
        with np.load(path, allow_pickle=False) as z:
            return {k: z[k] for k in z.files}
    except Exception:
        return None


def _obstacles(d):
    """Obstacle cells of the scan: obstacle_mask is -1 on obstacles, 1 on free space."""
    return np.asarray(d['obstacle_mask']) < 0


def _hms(s):
    h, m, x = s.split(':')
    return int(h) * 3600 + int(m) * 60 + float(x)


def _time_of_day(t):
    lt = time.localtime(t)
    return lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec + (t % 1.0)


class LiveState:
    """Everything known about the mission up to the instant `until` (None = now)."""

    def __init__(self, mm, ml):
        self.mm, self.ml = mm, ml
        self.mosaic = {}            # (ix, iy) -> 1 obstacle, 0 free
        self.scans_read = set()
        self.last = None           # latest loop scan (dict)
        self.vis = None              # latest PRM view (cells and graph)
        self.vis_path = None
        self.log_lines = []
        self.log_pos = 0
        self.log_state = {}

    # --- loop scans ----------------------------------------------------------------------
    def _scans(self):
        return sorted(glob.glob(os.path.join(self.mm, "scans", "scan_*.npz")), key=os.path.getmtime)

    def _add_to_mosaic(self, d):
        obst = _obstacles(d)
        seen = ~np.asarray(d['unwritten'], dtype=bool) if 'unwritten' in d else np.ones_like(obst)
        ny, nx = obst.shape
        ox, oy = (float(v) for v in d['grid_origin'])
        cs = float(d['cell_size'])
        jj, ii = np.nonzero(seen)
        if not len(jj):
            return
        ix = np.floor((ox + (ii + 0.5) * cs) / MOSAIC_RES).astype(np.int64)
        iy = np.floor((oy + (jj + 0.5) * cs) / MOSAIC_RES).astype(np.int64)
        val = obst[jj, ii]
        # For each mosaic cell: obstacle if at least 30% of the scan points are.
        key = ix * 1_000_003 + iy
        order = np.argsort(key)
        key, ix, iy, val = key[order], ix[order], iy[order], val[order]
        starts = np.r_[0, np.flatnonzero(np.diff(key)) + 1]
        counts = np.diff(np.r_[starts, len(key)])
        anyobst = np.add.reduceat(val.astype(np.int32), starts) >= 0.3 * counts
        for a, b, v in zip(ix[starts], iy[starts], anyobst):
            self.mosaic[(int(a), int(b))] = int(v)

    def update_scans(self, until=None):
        new_count = 0
        for p in self._scans():
            if p in self.scans_read:
                continue
            d = _load_npz(p)
            if d is None:
                continue                      # still being written: next turn
            if until is not None and float(d['time']) > until:
                continue
            self.scans_read.add(p)
            self._add_to_mosaic(d)
            if self.last is None or float(d['time']) >= float(self.last['time']):
                self.last = d
            new_count += 1
        return new_count

    # --- PRM views (cells and graph) -------------------------------------------------------
    def update_vis(self, until=None):
        cands = glob.glob(os.path.join(self.mm, "iteration_*_vis.npz"))
        if until is not None:
            cands = [p for p in cands if os.path.getmtime(p) <= until]
        if not cands:
            return False
        p = max(cands, key=os.path.getmtime)
        if p == self.vis_path:
            return False
        d = _load_npz(p)
        if d is not None:
            self.vis, self.vis_path = d, p
        return d is not None

    # --- log -------------------------------------------------------------------------------
    def update_log(self, until=None):
        path = os.path.join(self.ml, "mission_log.txt")
        if not os.path.exists(path):
            return False
        before = self.log_pos
        limit = None if until is None else _time_of_day(until)
        with open(path, encoding='utf-8', errors='replace') as f:
            f.seek(self.log_pos)
            while True:
                pos = f.tell()
                riga = f.readline()
                if not riga or not riga.endswith('\n'):
                    f.seek(pos)               # half-written line: re-read next turn
                    break
                m = re.match(r'(\d\d:\d\d:\d\d\.\d+) (.*)', riga.rstrip('\n'))
                if m and limit is not None and _hms(m.group(1)) > limit:
                    f.seek(pos)
                    break
                if m:
                    self._read_line(m.group(1)[:8], m.group(2))
            self.log_pos = f.tell()
        return self.log_pos != before

    def _read_line(self, ora, text):
        s = self.log_state
        if text.startswith('[PROGRESSO]'):
            s['progresso'] = text[len('[PROGRESSO] '):]
        elif text.lstrip().startswith(('min trascorsi', '0.', '1', '2', '3', '4', '5', '6', '7', '8', '9')) \
                and 'min trascorsi' in text:
            s['tempo'] = text.strip()
        elif text.startswith('[ATTEMPT]'):
            s['tentativo'] = text[len('[ATTEMPT] '):]
        elif text.startswith('[BATTERIA]'):
            m = re.search(r'(\d+(?:\.\d+)?)\s*%', text)
            if m:
                s['batteria'] = m.group(1) + '%'
        elif text.startswith('[GRAPHNAV]'):
            s['graphnav'] = f"{ora} {text[len('[GRAPHNAV] '):]}"
        # Mission phase, from the last line that states it.
        if text.startswith('[ATTEMPT]'):
            m = re.search(r'cella \((\d+),(\d+)\)', text)
            s['fase'] = f"esplorazione, tentativo sulla cella ({m.group(1)},{m.group(2)})" if m else "esplorazione"
        elif text.startswith('[GRAPHNAV]') and ': avvio' in text:
            s['fase'] = "viaggio con GraphNav: " + text[len('[GRAPHNAV] '):].split(':')[0]
        elif text.startswith('[GRAPHNAV] prima del ritorno a wp_0'):
            s['fase'] = "fine esplorazione: chiusura della mappa e ritorno a wp_0"
        elif text.startswith('[WP0] CORREZIONE'):
            s['fase'] = "CORREZIONE del ritorno a wp_0 con il fiducial"
        elif text.startswith('[EMERGENZA]'):
            s['fase'] = "EMERGENZA: " + text[len('[EMERGENZA] '):]
        elif text.startswith('[TEMPO] Chiusura della missione'):
            s['fase'] = "missione finita"
        # Return to wp_0: arrival, offset from the tag, corrections.
        if 'ritorno a wp_0: arrivato' in text:
            s['wp0'] = "arrivato secondo GraphNav"
        elif 'Ritorno a wp_0 NON riuscito' in text:
            s['wp0'] = text.split('] ', 1)[-1][:80]
        m = re.search(r"\[WP0\] Secondo il tag il robot e' a ([\d.]+) m", text)
        if m:
            s['wp0'] = f"{s.get('wp0', 'arrivato')}; secondo il tag a {m.group(1)} m dalla partenza"
        m = re.search(r"\[WP0\] Dopo la correzione (\d+): a ([\d.]+) m dalla partenza \(prima ([\d.]+) m\)", text)
        if m:
            s['wp0'] = f"CORRETTO con il fiducial ({m.group(1)}): da {m.group(3)} a {m.group(2)} m dalla partenza"
        if text.startswith(LOG_TAGS):
            self.log_lines.append(f"{ora} {text}")
            del self.log_lines[:-N_LOG_LINES]

    def trajectory(self, until=None):
        """Array (t, x, y, kind) from the recorded trajectory: kind 0 walked, 1 GraphNav,
        2 correction of the return to wp_0."""
        path = os.path.join(self.ml, "trajectory.csv")
        if not os.path.exists(path):
            return np.zeros((0, 4))
        rows = []
        try:
            with open(path, newline='') as f:
                for r in csv.DictReader(f):
                    try:
                        t = float(r['time'])
                        if until is not None and t > until:
                            break
                        kind = {'graphnav': 1.0, 'graphnav_correzione': 2.0}.get(r.get('move'), 0.0)
                        rows.append((t, float(r['x']), float(r['y']), kind))
                    except (TypeError, ValueError, KeyError):
                        continue
        except OSError:
            pass
        return np.array(rows) if rows else np.zeros((0, 4))


# =========================================================================================
# DRAWING
# =========================================================================================
def _mosaic_image(mosaic):
    keys = np.array(list(mosaic.keys()))
    vals = np.array(list(mosaic.values()))
    x0, y0 = keys.min(axis=0)
    x1, y1 = keys.max(axis=0)
    img = np.ones((y1 - y0 + 1, x1 - x0 + 1, 3))
    img[keys[:, 1] - y0, keys[:, 0] - x0] = np.where(vals[:, None] > 0, [0.92, 0.35, 0.35], [0.85, 0.95, 0.85])
    ext = [x0 * MOSAIC_RES, (x1 + 1) * MOSAIC_RES, y0 * MOSAIC_RES, (y1 + 1) * MOSAIC_RES]
    return img, ext


def _draw_cells(ax, vis, target_cell):
    if vis is None or 'env_cells' not in vis or not len(vis['env_cells']):
        return
    size = float(vis.get('env_cell_size', 5.0))
    yaw = float(vis.get('env_origin_yaw', 0.0))
    h = size / 2.0
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    base = np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) @ R.T
    for (r, c, x, y), st in zip(vis['env_cells'], vis['env_cell_status']):
        st = str(st)
        if st == '1':
            kw = dict(edgecolor='darkgreen', facecolor='lightgreen', alpha=0.25, lw=1.0)
        elif st == '-1':
            kw = dict(edgecolor='darkred', facecolor='lightcoral', alpha=0.35, lw=1.0)
        else:
            kw = dict(edgecolor='gray', facecolor='none', alpha=0.6, lw=0.7, ls='--')
        if target_cell is not None and (int(r), int(c)) == target_cell:
            kw.update(edgecolor='blue', lw=2.2, alpha=0.9)
        ax.add_patch(patches.Polygon(base + [x, y], zorder=4, **kw))
        ax.text(x, y, f"{int(r)},{int(c)}", ha='center', va='center', fontsize=7, color='0.25', zorder=4,
                clip_on=True)


def _draw_graph(ax, vis):
    if vis is None or 'prm_edges' not in vis or not len(vis['prm_edges']):
        return
    pos = {int(n): xy for n, xy in zip(vis['prm_node_ids'], vis['prm_node_xy'])}
    segs = [(pos[int(a)], pos[int(b)]) for a, b in vis['prm_edges'] if int(a) in pos and int(b) in pos]
    from matplotlib.collections import LineCollection
    ax.add_collection(LineCollection(segs, colors=(0.5, 0.5, 0.5, 0.18), linewidths=0.4, zorder=2))


def draw(fig, ax_map, ax_txt, st, follow, until=None):
    ax_map.clear()
    ax_txt.clear()
    ax_txt.axis('off')
    d = st.last
    if st.mosaic:
        img, ext = _mosaic_image(st.mosaic)
        ax_map.imshow(img, origin='lower', extent=ext, interpolation='nearest', zorder=0)
    cell = None
    if d is not None:
        cell = (int(d['cell'][0]), int(d['cell'][1]))
        # Latest scan, darker, inside its box.
        ny, nx = d['obstacle_mask'].shape
        ox, oy = (float(v) for v in d['grid_origin'])
        cs = float(d['cell_size'])
        ext_l = [ox, ox + nx * cs, oy, oy + ny * cs]
        loc = np.zeros((ny, nx, 4))
        seen = ~np.asarray(d['unwritten'], dtype=bool) if 'unwritten' in d else np.ones((ny, nx), bool)
        obst = _obstacles(d)
        loc[seen & ~obst] = [0.55, 0.80, 0.55, 0.55]
        loc[seen & obst] = [0.75, 0.05, 0.05, 0.85]
        ax_map.imshow(loc, origin='lower', extent=ext_l, interpolation='nearest', zorder=3)
        ax_map.add_patch(patches.Rectangle((ext_l[0], ext_l[2]), nx * cs, ny * cs, fill=False,
                                           edgecolor='deepskyblue', lw=1.2, zorder=4))
    _draw_cells(ax_map, st.vis, cell)
    _draw_graph(ax_map, st.vis)

    tr = st.trajectory(until)
    if len(tr):
        kind = np.rint(tr[:, 3]).astype(int)
        jump = np.r_[False, np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])) > 1.5]
        for k, ls, col, lw in ((0, '-', 'royalblue', 1.4), (1, '--', 'royalblue', 1.4), (2, '-', 'crimson', 2.6)):
            xy = tr[:, 1:3].copy()
            xy[(kind != k) | jump] = np.nan
            ax_map.plot(xy[:, 0], xy[:, 1], ls, color=col, lw=lw, zorder=5 if k < 2 else 8)
        # Start: the first recorded point (in the VISION frame, so with the drift in it).
        ax_map.plot(tr[0, 1], tr[0, 2], '^', color='black', mfc='white', ms=10, mew=1.5, zorder=9)

    lines = []
    if 'fase' in st.log_state:
        lines.append(f"FASE             {st.log_state['fase']}")
    if d is not None:
        rx, ry, _ = (float(v) for v in d['robot_xyz'])
        yaw = float(d['robot_yaw'])
        poly = np.asarray(d['polyline'])
        if len(poly) > 1:
            ax_map.plot(poly[:, 0], poly[:, 1], '-o', color='darkorange', lw=2, ms=3, zorder=6)
        gx, gy = (float(v) for v in d['goal_xy'])
        ax_map.plot(gx, gy, '*', color='gold', mec='k', ms=16, zorder=8)
        reason = str(d['frontier_reason'])
        sx, sy = (float(v) for v in d['frontier_stop_xy'])
        ax_map.plot(sx, sy, 'x', color=REASON_COLORS.get(reason, 'k'), ms=12, mew=3, zorder=8)
        adv = float(d['allowed_advance'])
        if len(poly) > 1 and adv > 0:
            pts, rest = [poly[0]], adv
            for a, b in zip(poly[:-1], poly[1:]):
                L = float(np.hypot(*(b - a)))
                if L >= rest:
                    pts.append(a + (b - a) * (rest / max(L, 1e-9)))
                    break
                pts.append(b)
                rest -= L
            pts = np.array(pts)
            ax_map.plot(pts[:, 0], pts[:, 1], '-', color='limegreen', lw=5, alpha=0.7, zorder=7)
        ax_map.add_patch(patches.FancyArrow(rx, ry, 0.6 * np.cos(yaw), 0.6 * np.sin(yaw), width=0.12,
                                            head_width=0.35, head_length=0.25, color='k', zorder=9))
        if follow:
            ax_map.set_xlim(rx - 6, rx + 6)
            ax_map.set_ylim(ry - 6, ry + 6)
        eta = time.time() - float(d['time']) if until is None else until - float(d['time'])
        lines += [
            f"Cella obiettivo  ({cell[0]},{cell[1]})   giro {int(d['loop_iter'])}, tentativo {int(d['iteration'])}",
            f"Decisione        {str(d['decision_kind'])}",
            f"Fronte           {reason}: {float(d['frontier_dist']):.2f} m confermati su "
            f"{float(d['frontier_path_len']):.2f} m, avanzo {adv:.2f} m",
            f"Robot            ({rx:.2f}, {ry:.2f})  muso {np.degrees(yaw):.0f} gradi"
            + (f"   fermo da {int(float(d['no_progress']))} scansioni" if float(d['no_progress']) > 0 else ""),
            f"Obiettivo        ({gx:.2f}, {gy:.2f}), a {np.hypot(gx - rx, gy - ry):.2f} m in linea d'aria",
            f"Ultima scansione {eta:.0f} s fa",
        ]
    s = st.log_state
    if 'progresso' in s:
        lines.append(f"Avanzamento      {s['progresso']}")
    if 'tempo' in s:
        lines.append(f"                 {s['tempo']}")
    if 'batteria' in s:
        lines.append(f"Batteria         {s['batteria']}")
    if 'graphnav' in s:
        lines.append(f"Ultimo GraphNav  {s['graphnav'][:90]}")
    if 'wp0' in s:
        lines.append(f"Ritorno a wp_0   {s['wp0']}")
    lines = [w for r in lines for w in (textwrap.wrap(r, 66, subsequent_indent=' ' * 17) or [''])]
    ax_txt.text(0.0, 1.0, "\n".join(lines), va='top', ha='left', family='monospace', fontsize=9,
                transform=ax_txt.transAxes)
    log = "\n".join(w for r in st.log_lines for w in textwrap.wrap(r, 92, subsequent_indent='    '))
    ax_txt.text(0.0, 0.52, "Ultime righe del log\n" + log, va='top', ha='left', family='monospace',
                fontsize=7.5, transform=ax_txt.transAxes, color='0.15')
    ax_map.set_aspect('equal')
    if not follow and st.mosaic:
        ax_map.autoscale()
    from matplotlib.lines import Line2D
    leg = [Line2D([], [], color='royalblue', lw=1.4, label='strada fatta'),
           Line2D([], [], color='royalblue', lw=1.4, ls='--', label='GraphNav'),
           Line2D([], [], color='crimson', lw=2.6, label='correzione wp_0'),
           Line2D([], [], color='darkorange', lw=2, marker='o', ms=3, label='percorso pianificato'),
           Line2D([], [], color='gold', marker='*', mec='k', ms=11, ls='', label='obiettivo'),
           Line2D([], [], color='k', marker='x', ms=8, mew=2, ls='', label='fine del fronte'),
           Line2D([], [], color='black', marker='^', mfc='white', ms=8, ls='', label='partenza')]
    ax_map.legend(handles=leg, loc='upper center', bbox_to_anchor=(0.5, -0.035), ncol=4, fontsize=7.5,
                  frameon=False)
    nome = os.path.basename(st.mm).replace('Mission_', '')
    when = time.strftime('%H:%M:%S', time.localtime(until if until is not None else time.time()))
    ax_map.set_title(f"Missione {nome} -- {when}" + ("   (segue il robot: f)" if follow else "   (tutta: f)"),
                     fontsize=10)
    fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(description="Visualizzatore dal vivo della missione.")
    ap.add_argument('mission', nargs='?', help="nome (es. 28-10-2026_14-23-50) o cartella; default l'ultima")
    ap.add_argument('--replay', action='store_true', help="rigioca una missione finita nel suo tempo")
    ap.add_argument('--speed', type=float, default=20.0, help="velocita' del rigioco (default 20x)")
    ap.add_argument('--png', help="salva una figura e esce (con --until: a quell'ora)")
    ap.add_argument('--until', help="HH:MM:SS, con --png")
    ap.add_argument('--whole', action='store_true', help="parte mostrando tutta la missione invece di seguire il robot")
    args = ap.parse_args()

    if args.png:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    mm, ml = _folders(args.mission)
    if not os.path.isdir(mm):
        sys.exit(f"Cartella non trovata: {mm}")
    st = LiveState(mm, ml)
    fig = plt.figure(figsize=(15, 8.5))
    ax_map = fig.add_axes([0.04, 0.11, 0.55, 0.85])
    ax_txt = fig.add_axes([0.61, 0.03, 0.38, 0.94])
    state = {'segui': not args.whole, 'pausa': False, 'esci': False, 'ridisegna': True}

    def on_key(ev):
        state['ridisegna'] = True
        if ev.key == 'f':
            state['segui'] = not state['segui']
        elif ev.key == ' ':
            state['pausa'] = not state['pausa']
        elif ev.key in ('q', 'escape'):
            state['esci'] = True
    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.canvas.mpl_connect('close_event', lambda ev: state.update(esci=True))

    # Start instant for replay and --until: the first scan of the mission.
    scans = st._scans()
    t_start = None
    if scans:
        d0 = _load_npz(scans[0])
        t_start = float(d0['time']) if d0 is not None else None

    if args.png:
        until = None
        if args.until and t_start is not None:
            base = time.localtime(t_start)
            h, m, s = (int(v) for v in args.until.split(':'))
            until = time.mktime((base.tm_year, base.tm_mon, base.tm_mday, h, m, s, 0, 0, -1))
        elif scans:
            dl = _load_npz(scans[-1])
            until = float(dl['time']) + 60.0 if dl is not None else None
        st.update_scans(until)
        st.update_vis(until)
        st.update_log(until)
        draw(fig, ax_map, ax_txt, st, state['segui'], until)
        fig.savefig(args.png, dpi=110)
        print(f"Salvata {args.png}")
        return

    if args.replay and t_start is None:
        sys.exit("Nessuna scansione da rigiocare.")
    t_rig, t_wall = t_start, time.time()
    print(f"Missione {os.path.basename(mm)}: " + ("rigioco" if args.replay else "dal vivo") +
          ". Tasti: f segui/tutta, spazio pausa, q esci.")
    while not state['esci']:
        now = time.time()
        if not state['pausa']:
            until = None
            if args.replay:
                t_rig += (now - t_wall) * args.speed
                until = t_rig
            # Redraw only if there is something new (or a key): drawing costs ~0.4 s
            # of CPU, on the same computer as the mission.
            changed = st.update_scans(until) > 0
            changed = st.update_vis(until) or changed
            changed = st.update_log(until) or changed
            if changed or state['ridisegna'] or args.replay:
                draw(fig, ax_map, ax_txt, st, state['segui'], until)
                state['ridisegna'] = False
        t_wall = now
        plt.pause(REFRESH_S)
        if not plt.fignum_exists(fig.number):
            break


if __name__ == '__main__':
    main()
