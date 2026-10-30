#!/usr/bin/env python3
"""
Report of a mission, to read right after a test.

Gathers on one text page the numbers needed to understand how it went: cells, timings,
motion and gaits, frontier stops and their reason, blocks and replans, return to wp_0.
It only reads the files the mission saves; like mission_viewer.py it only needs numpy,
NOT the Spot SDK (which, if present, is used for the return path in the GraphNav graph).

    python3 tools/mission_report.py                         # the latest mission in MissionLogs/
    python3 tools/mission_report.py 13-10-2026_12-40-38     # a mission by name
    python3 tools/mission_report.py --all                   # one summary line per mission

Files read (missing ones are skipped: older missions have fewer):
    MissionLogs/Mission_<name>/mission_log.txt, mission_summary.json, segments.csv, trajectory.csv,
        drift.csv (since the evening of 26-10), battery.csv (since 28-10)
    MissionMap/Mission_<name>/scans/*.npz       the navigation loop scans
    graph/Mission_<name>/Graph_*/               the GraphNav graph downloaded at the end of the mission
"""
import csv
import glob
import json
import math
import os
import re
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # project folder
sys.path.insert(0, HERE)                                     # the spot_explorer package
from spot_explorer.legacy import modern, modern_keys, modern_log_line   # noqa: E402
LOGS = os.path.join(HERE, 'MissionLogs')
MAPS = os.path.join(HERE, 'MissionMap')
GRAPHS = os.path.join(HERE, 'graph')

# As in the perception package: copied here because the report does not import the mission code.
INTRUSION_M = 0.25                 # half body width (geometry.ROBOT_HALF_WIDTH_M)


def mission_thresholds(summary, log):
    """
    (frontier, live clearance): the obstacle distances THIS mission worked with, from the
    recorded switches (since 21-10). Frontier = 0.25 + geometry.ROBOT_CLEARANCE_AIR_M;
    live edge clearance = frontier since 22-10 (when PLAN_CLEARANCE_EXTRA_M is present).
    Without recorded switches: 0.30 and 0.20, the values of the earlier missions.
    """
    sw = dict((summary or {}).get('switches') or {})
    if not sw:
        m = re.search(r'\[CONFIG\] (?:INTERRUTTORI|SWITCHES): (.*)', log)
        if m:
            sw = {f"x.{k}": v for k, v in re.findall(r'(\w+)=(\S+)', m.group(1))}
    by_name = {k.split('.')[-1]: v for k, v in sw.items()}

    def num(name):
        try:
            return float(by_name[name])
        except (KeyError, TypeError, ValueError):
            return None
    air, extra = num('ROBOT_CLEARANCE_AIR_M'), num('PLAN_CLEARANCE_EXTRA_M')
    frontier = INTRUSION_M + (0.05 if air is None else air)
    live = 0.20 if extra is None else frontier
    return frontier, live
TERRAIN_VALID_MIN = 0.5            # spot_grid.TERRAIN_VALID_MIN
PITCH_LIMITS_DEG = (14.0, 18.0)    # "steep" legs for the gait check


# --------------------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------------------
def mission_files(name):
    log_dir = os.path.join(LOGS, f'Mission_{name}')
    f = lambda *p: os.path.join(log_dir, *p)
    return dict(
        log=f('mission_log.txt') if os.path.exists(f('mission_log.txt')) else None,
        summary=f('mission_summary.json') if os.path.exists(f('mission_summary.json')) else None,
        segments=f('segments.csv') if os.path.exists(f('segments.csv')) else None,
        trajectory=f('trajectory.csv') if os.path.exists(f('trajectory.csv')) else None,
        scans=sorted(glob.glob(os.path.join(MAPS, f'Mission_{name}', 'scans', '*.npz'))),
        graph=sorted(glob.glob(os.path.join(GRAPHS, f'Mission_{name}', 'Graph_*'))
                     + glob.glob(os.path.join(GRAPHS, f'Mission_{name}', 'Grafo_*'))))


def read_csv(path):
    if path is None:
        return []
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


def log_time(line):
    """Seconds of the day from the hh:mm:ss.mmm time at the start of the line, or None."""
    m = re.match(r'(\d\d):(\d\d):(\d\d(?:\.\d+)?)', line)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None


# --------------------------------------------------------------------------------------
# Sezioni
# --------------------------------------------------------------------------------------
def section_cells(log):
    # The cell messages are in Italian from 21-10; earlier missions have them in English.
    grid = re.search(r'(?:Griglia di missione|Mission grid): (\d+) x (\d+) (?:celle da|cells of) ([\d.]+) m', log)
    attempts = re.findall(r'\[ATTEMPT\] (?:Trying to enter cell|Provo a entrare nella cella) \((\d+),(\d+)\)', log)
    visited = re.findall(r'\[VISITED\] Cell[a]? \((\d+),(\d+)\) (?:marked as visited|segnata come visitata)', log)
    blocked = set(re.findall(r'\[BLOCKED\] Cell[a]? \((\d+),(\d+)\) (?:marked as blocked|segnata come bloccata)', log))
    times = [t for t in (log_time(l) for l in log.splitlines()) if t is not None]
    out = dict(grid=(int(grid.group(1)), int(grid.group(2)), float(grid.group(3))) if grid else None,
               attempts=len(attempts), visited=len(set(visited)), blocked=len(blocked),
               duration_s=(times[-1] - times[0]) if len(times) > 1 else None)
    return out


def section_times(log, summary):
    out = {}
    if summary:
        out['breakdown'] = summary.get('mission_stats', {}).get('time_s', {})
    upd = [(modern(kind), float(s)) for kind, s in re.findall(
        r'\[(?:TEMPO|TIMING)\] (aggiornamento locale|costruzione completa|local update|full build) (?:del grafo|of the graph): ([\d.]+) s', log)]
    out['graph'] = upd
    closing = {}
    block = re.search(r'\[(?:TEMPO|TIMING)\] (?:Chiusura della missione|Mission closure).*?\n((?:.*\n){1,8})', log)
    if block:
        for name, s in re.findall(r'\]?\s{2,}([a-z][a-z0-9_ \']+?)\s{2,}([\d.]+) s', block.group(1)):
            closing[name.strip()] = float(s)
    out['closure'] = closing
    return out


def section_motion(segments, trajectory):
    out = dict(segments=len(segments))
    if trajectory:
        xy = np.array([[float(r['x']), float(r['y'])] for r in trajectory])
        out['distance_m'] = float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0
    if not segments:
        return out
    out['ok'] = sum(r.get('success') == 'True' for r in segments)
    out['tiers'] = Counter(r.get('tier', '?') for r in segments)
    pitch = [(float(r['pitch_peak_abs_deg']), r.get('tier', '?')) for r in segments
             if r.get('pitch_peak_abs_deg') not in (None, '', 'nan')]
    out['steep'] = {lim: (sum(p > lim for p, _ in pitch), sum(p > lim and t == 'HARD' for p, t in pitch))
                    for lim in PITCH_LIMITS_DEG}
    out['max_pitch'] = max((p for p, _ in pitch), default=None)
    out['calm_hard'] = sum(p < 8 and t == 'HARD' for p, t in pitch)
    # Smoothness: for each leg, the time before the robot moves 5 cm (rotation in place
    # and start) and the walking time; average speed over all the legs.
    dur = [float(r['duration_s']) for r in segments if r.get('duration_s') not in (None, '')]
    dist = [float(r['dist_traveled_m']) for r in segments if r.get('dist_traveled_m') not in (None, '')]
    if dur and sum(dur) > 0:
        out['speed'] = sum(dist) / sum(dur)
        out['move_s'] = sum(dur)
    if trajectory:
        by_seg = {}
        for r in trajectory:
            key = (r['iteration'], r['loop_iter'], r['node_from'], r['node_to'])
            by_seg.setdefault(key, []).append((float(r['time']), float(r['x']), float(r['y'])))
        turn = walk = 0.0
        for s in by_seg.values():
            s.sort()
            t0, x0, y0 = s[0]
            t_go = next((t for t, x, y in s if math.hypot(x - x0, y - y0) > 0.05), s[-1][0])
            turn += t_go - t0
            walk += s[-1][0] - t_go
        out['turn_s'], out['walk_s'] = turn, walk
    return out


def _seen_value(d, x, y):
    """Filtered obstacle_distance at (x, y) if scan d saw that point, otherwise None."""
    if 'obstacle_distance_filtered' not in d.files:
        return None
    z = d['terrain_corrected']
    ox, oy = map(float, d['grid_origin'])
    cs = float(d['cell_size'])
    c, r = int(math.floor((x - ox) / cs)), int(math.floor((y - oy) / cs))
    if not (0 <= r < z.shape[0] and 0 <= c < z.shape[1]):
        return None
    if np.asarray(d['terrain_valid_raw']).reshape(z.shape)[r, c] < TERRAIN_VALID_MIN:
        return None
    if 'unwritten' in d.files and np.size(d['unwritten']) == z.size and \
            np.asarray(d['unwritten']).reshape(z.shape)[r, c]:
        return None
    return float(np.asarray(d['obstacle_distance_filtered']).reshape(z.shape)[r, c])


def section_frontier(scans, thresholds=(0.30, 0.20)):
    """
    Frontier stops, from the saved scans: why and, for obstacles, whether they were already
    visible. thresholds = (frontier, live clearance) of the mission (see mission_thresholds).
    """
    FRONTIER_CLEARANCE_M, LIVE_PREFILTER_OD_M = thresholds
    if not scans:
        return None
    data = [np.load(f) for f in scans]
    if 'frontier_reason' not in data[0].files:
        return None
    reasons = Counter(modern(str(d['frontier_reason'])) for d in data if 'frontier_reason' in d.files)
    events, last = [], None
    for i, d in enumerate(data):
        if modern(str(d['frontier_reason'])) != 'obstacle':
            last = None
            continue
        p = np.asarray(d['frontier_stop_xy'], dtype=float)
        it = int(d['iteration'])
        if last is not None and last[0] == it and np.hypot(*(p - last[1])) < 0.3:
            continue                                  # same stop, next scan
        last = (it, p)
        before = next((v for v in (_seen_value(data[j], *p) for j in range(i - 1, -1, -1)) if v is not None), None)
        events.append((float(d['frontier_value']), before))
    bands = Counter()
    origin = Counter()
    for value, before in events:
        f, lv = FRONTIER_CLEARANCE_M, LIVE_PREFILTER_OD_M
        bands['real intrusion (below 0.25 m)' if value < INTRUSION_M else
              f'grazing (0.25-{f:.2f} m)' if value < f else
              f"held by the hysteresis ({f:.2f} m or more)"] += 1
        lo, hi = min(f, lv), max(f, lv)
        origin['never seen before' if before is None else
               f'seen before below {lo:.2f} m' if before < lo else
               f'seen before between {lo:.2f} and {hi:.2f} m' if before < hi else
               f'seen before above {hi:.2f} m (read from afar)'] += 1
    return dict(reasons=reasons, stops=len(events), bands=bands, origin=origin)


def section_battery(stats, log):
    """Charge and consumption (battery.py, since 28-10): one line, None if the mission does not have them."""
    if 'battery_start_pct' not in stats:
        return ("Battery: charge never read from the robot state"
                if stats.get('battery_reads_failed') else None)
    line = (f"Battery: from {stats['battery_start_pct']:.0f}% to {stats['battery_last_pct']:.0f}% in "
            f"{stats.get('battery_elapsed_min', 0):.0f} min and {stats.get('battery_dist_m', 0):.0f} m")
    if stats.get('battery_pct_per_min') is not None:
        line += f", {stats['battery_pct_per_min']:.2f} %/min"
    if stats.get('battery_pct_per_m') is not None:
        line += f", {stats['battery_pct_per_m']:.3f} %/m"
    # The return to wp_0 comes after the summary: it is read from the log.
    m = re.search(r"\[(?:BATTERIA|BATTERY)\] (?:Ritorno a wp_0: da|Return to wp_0: from) (\d+)% (?:a|to) (\d+)% \(([\d.]+) (?:punti in|points in) (\d+) s\)", log)
    if m:
        line += f"; return to wp_0 {m.group(3)} points in {m.group(4)} s"
    if stats.get('battery_return'):
        line += (f" | EARLY RETURN at {stats['battery_return_pct']:.0f}% after "
                 f"{stats['battery_return_min']:.0f} min, {stats.get('battery_return_cells_left', '?')} "
                 f"cells left in the frontier")
    return line


def section_arrival_rotation(trajectory):
    """After every GraphNav arrival, how much the robot must turn for the first move: angle
    between the arrival yaw and the direction of the first point to reach, and seconds of
    rotation in place (position within 0.12 m). Used to judge GRAPHNAV_ARRIVAL_HEADING (28-10).
    List of (degrees, seconds); empty if trajectory.csv has no GraphNav trips (before 26-10)."""
    rows = []
    for r in trajectory:
        try:
            rows.append((float(r['time']), float(r['x']), float(r['y']), float(r['yaw_deg']),
                         float(r['target_x']), float(r['target_y']), modern(r['move'].strip())))
        except (KeyError, ValueError):
            continue
    rows.sort()
    out, k = [], 0
    while k < len(rows):
        if rows[k][6] != 'graphnav':
            k += 1
            continue
        while k + 1 < len(rows) and rows[k + 1][6] == 'graphnav' and rows[k + 1][0] - rows[k][0] <= 2.0:
            k += 1
        end = rows[k]
        nxt = k + 1
        while nxt < len(rows) and rows[nxt][6] == 'graphnav':
            nxt += 1
        k = nxt
        if nxt >= len(rows):
            break
        first = rows[nxt]
        # The first move must start from where GraphNav left the robot, with a point to reach.
        if math.isnan(first[4]) or math.hypot(first[1] - end[1], first[2] - end[2]) > 0.30:
            continue
        want = math.degrees(math.atan2(first[5] - first[2], first[4] - first[1]))
        turn = abs((want - first[3] + 180.0) % 360.0 - 180.0)
        j = nxt
        while (j + 1 < len(rows) and rows[j + 1][6] == first[6] and rows[j + 1][0] - rows[j][0] <= 2.0
               and math.hypot(rows[j + 1][1] - first[1], rows[j + 1][2] - first[2]) < 0.12):
            j += 1
        out.append((turn, rows[j][0] - first[0]))
    return out


def section_return(log, graph_dirs):
    out = {}
    m = re.search(r'(?:ritorno a wp_0: arrivato|return to wp_0: arrived) in (\d+) s', log)
    out['time_s'] = int(m.group(1)) if m else None
    # Since the evening of 28-10: failed return, with the robot stopped near wp_0 or far away.
    m = re.search(r"(?:Ritorno a wp_0 NON riuscito|Return to wp_0 NOT successful)( ma vicino| but near)?: (?:fermo|stopped) (?:(?:a )?([\d.]+) m (?:da|from) wp_0)?.*?\((?:motivo|reason): (.*?)(?:; (?:distanza|distance) .*)?\)\.", log)
    if m:
        out['failed'] = dict(near=bool(m.group(1)), metres=float(m.group(2)) if m.group(2) else None,
                              reason=m.group(3))
    m = re.search(r'(?:strada verso|path to) wp_0 ([\d.]+) m -> ([\d.]+) m (?:con|with) (\d+) (?:scorciatoie|shortcuts)', log)
    if m:
        out['path_m'] = (float(m.group(1)), float(m.group(2)), int(m.group(3)))
    # Since 29-10: true offset from the start according to the tag, and corrections (WP0_FIDUCIAL_CORRECTION).
    m = re.search(r"\[WP0\] (?:Secondo il tag il robot e' a|According to the tag the robot is) ([\d.]+) m (?:dal punto di partenza|from the starting point)", log)
    if m:
        out['offset_m'] = float(m.group(1))
    after = re.findall(r"\[WP0\] (?:Dopo la correzione|After correction) (\d+): (?:a )?([\d.]+) m (?:dalla partenza|from the start)", log)
    if after:
        out['corrections'] = (int(after[-1][0]), float(after[-1][1]))
    elif re.search(r"\[WP0\] (?:CORREZIONE|CORRECTION)", log):
        out['corrections'] = (len(re.findall(r"\[WP0\] (?:CORREZIONE|CORRECTION)", log)), None)
    start = re.findall(r"(?:prima di 'ritorno a wp_0'|before 'return to wp_0'): (?:localizzato su|localized on) (\S+)", log)
    if graph_dirs and start:
        out['graph'] = _graph_route(graph_dirs[-1], start[-1])
    return out


def _graph_route(gdir, start_name):
    """Length of the GraphNav path from start_name to wp_0 in the downloaded graph (needs the SDK)."""
    try:
        from bosdyn.api.graph_nav import map_pb2
    except ImportError:
        return None
    import heapq
    g = map_pb2.Graph()
    with open(os.path.join(gdir, 'graph'), 'rb') as fh:
        g.ParseFromString(fh.read())
    ids = {w.annotations.name: w.id for w in g.waypoints}
    if start_name not in ids or 'wp_0' not in ids:
        return None
    adj = {}
    for e in g.edges:
        p = e.from_tform_to.position
        w = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
        adj.setdefault(e.id.from_waypoint, []).append((e.id.to_waypoint, w))
        adj.setdefault(e.id.to_waypoint, []).append((e.id.from_waypoint, w))
    s, goal = ids[start_name], ids['wp_0']
    dist, pq = {s: 0.0}, [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            return dict(from_wp=start_name, metres=round(d, 1), waypoint=len(g.waypoints), edges=len(g.edges))
        if d > dist[u]:
            continue
        for v, w in adj.get(u, []):
            if d + w < dist.get(v, float('inf')):
                dist[v] = d + w
                heapq.heappush(pq, (d + w, v))
    return None


def section_checks(log, segments):
    """
    Checks of the features never tried on the robot (see PROVA_CAMPO.md): for each one
    (name, outcome, detail). Outcome: 'OK', 'DA GUARDARE' (to look at), 'NON SCATTATO'
    (the situation did not occur: neither good nor bad) or 'DECISION' (a number to decide on).
    """
    out = []

    # 1. Local graph update and slopes in batch: timings.
    upd = [(modern(k), float(s)) for k, s in re.findall(
        r'\[(?:TEMPO|TIMING)\] (aggiornamento locale|costruzione completa|local update|full build) (?:del grafo|of the graph): ([\d.]+) s', log)]
    loc = [s for k, s in upd if k == 'local update']
    full = [s for k, s in upd if k == 'full build']
    if len(upd) < 2:
        out.append(("Local graph update", 'NOT TRIGGERED',
                    f"{len(upd)} graph updates: at least two are needed"))
    else:
        det = (f"{len(full)} full (mean {np.mean(full):.1f} s), " if full else "") + \
              f"{len(loc)} local" + (f" (mean {np.mean(loc):.1f} s, maximum {max(loc):.1f} s)" if loc else "")
        ok = loc and len(full) <= 2 and np.mean(loc) <= 4.0
        out.append(("Local graph update", 'OK' if ok else 'CHECK',
                    det + "; expected: a single full one at the start, then local ones of 1-3 s"))

    # 2. Goal at the cell centre, fallbacks near the centre.
    center = len(re.findall(r"\[TARGET\] (?:Cella|Cell) \(\d+,\d+\): (?:uso il centro geometrico|using the geometric centre)", log))
    moved = [float(d) for d in re.findall(
        r"\[TARGET\] (?:Cella|Cell) \(\d+,\d+\): (?:centro|centre) [^,]+, (?:uso il punto .*?a|using the .*?) ([\d.]+) m (?:dal centro|from the centre)", log)]
    none = len(re.findall(r"\[TARGET\] (?:Cella|Cell) \(\d+,\d+\): (?:centro|centre) .*(?:Nessun obiettivo valido|No valid goal)", log))
    if center + len(moved) + none == 0:
        out.append(("Goal at the cell centre", 'NOT TRIGGERED', "no [TARGET] line in the current format"))
    else:
        det = f"centre {center} times, fallback {len(moved)}" + \
              (f" (up to {max(moved):.2f} m from the centre)" if moved else "") + \
              (f", no valid goal {none}" if none else "")
        ok = not moved or max(moved) <= 1.5
        out.append(("Goal at the cell centre", 'OK' if ok else 'CHECK',
                    det + "; expected: almost always the centre, fallbacks within ~1 m"))

    # 3. Slope with the body and gaits: steep legs must be HARD.
    pitch = [(float(r['pitch_peak_abs_deg']), r.get('tier', '?')) for r in segments
             if r.get('pitch_peak_abs_deg') not in (None, '', 'nan')]
    if not pitch:
        out.append(("Slope with the body and gaits", 'NOT TRIGGERED', "no leg with recorded tilt"))
    else:
        steep = [(p, t) for p, t in pitch if p > 18.0]
        miss = [p for p, t in steep if t != 'HARD']
        veto = [int(a) + int(b) for a, b in re.findall(
            r'\[(?:PENDENZE|SLOPES)\] (?:costruzione|build): (?:archi vetati per pendenza|edges vetoed for slope) -- (?:longitudinale|longitudinal) (\d+) .*?(?:laterale|lateral) (\d+)', log)]
        det = (f"{len(steep)} legs with the nose above 18 degrees, {len(miss)} not in HARD"
               + (f"; edges vetoed for slope at build time: {veto[0]}" if veto else ""))
        out.append(("Slope with the body and gaits", 'OK' if not miss else 'CHECK',
                    det + "; expected: all legs above 18 degrees in HARD"))

    # 4. GraphNav shortcuts on the way.
    roads = [(float(a), float(b), int(n)) for a, b, n in re.findall(
        r'(?:strada verso|path to) \S+ ([\d.]+) m -> ([\d.]+) m (?:con|with) (\d+) (?:scorciatoie|shortcuts)', log)]
    refused = len(re.findall(r'\[GRAPHNAV\] (?:Scorciatoia \S+ rifiutata|Shortcut \S+ rejected)', log))
    ret = re.search(r'(?:ritorno a wp_0: arrivato|return to wp_0: arrived) in (\d+) s', log)
    near = re.search(r"(?:Ritorno a wp_0 NON riuscito ma vicino: fermo a|Return to wp_0 NOT successful but near: stopped) ([\d.]+) m (?:da|from) wp_0 \((?:motivo|reason): (.*?); (?:distanza|distance)", log)
    wp0 = re.findall(r'(?:strada verso|path to) wp_0 ([\d.]+) m -> ([\d.]+) m', log)
    if not roads:
        out.append(("GraphNav shortcuts on the way", 'NOT TRIGGERED', "no 'path to' line"))
    else:
        det = (f"{len(roads)} moves, {sum(a for a, _, _ in roads):.0f} m -> "
               f"{sum(b for _, b, _ in roads):.0f} m with {sum(n for _, _, n in roads)} shortcuts")
        if wp0:
            det += f"; wp_0: {float(wp0[-1][0]):.1f} -> {float(wp0[-1][1]):.1f} m"
        det += f", return {'in ' + ret.group(1) + ' s' if ret else 'NOT arrived'}"
        if near:
            det += f" (stopped {near.group(1)} m from wp_0, {near.group(2)}: fiducial and drift measured anyway)"
        if refused:
            det += f"; {refused} shortcuts rejected by Spot"
        out.append(("GraphNav shortcuts on the way", 'OK' if ret and not refused else 'CHECK',
                    det + "; expected: return to wp_0 arrived, no shortcut rejected"))

    # 5. GraphNav localization: losses and re-anchoring.
    lost = len(re.findall(r'\[(?:LOCALIZZAZIONE|LOCALIZATION)\] [^|]*: (?:PERSO|LOST)', log))
    re_ok = len(re.findall(r'\[(?:LOCALIZZAZIONE|LOCALIZATION)\] (?:Riaggancio|Re-anchoring) .*: (?:riuscito su|succeeded on)', log))
    re_all = len(re.findall(r'\[(?:LOCALIZZAZIONE|LOCALIZATION)\] (?:Riaggancio|Re-anchoring) ', log))
    fatal = len(re.findall(r'(?:ha perso la localizzazione\. Fermo il robot|lost the localization\. Stopping the robot)', log))
    if lost == 0 and re_all == 0:
        out.append(("GraphNav re-anchoring", 'NOT TRIGGERED', "GraphNav never got lost"))
    else:
        out.append(("GraphNav re-anchoring", 'OK' if re_ok == re_all and not fatal else 'CHECK',
                    f"lost {lost} times, re-anchorings {re_ok} successful out of {re_all}, moves failed "
                    f"for loss {fatal}; expected: every re-anchoring successful"))

    # 6. Returns with memory and recovery on the same goal.
    excl = len(re.findall(r'\[(?:RITORNO|RETURN)\] (?:Arco|Edge) \d+->\d+ (?:fuori dal piano|out of the plan)', log))
    giveup = len(re.findall(r'(?:ritorni in questo tentativo: sono tornato indietro, ma non riprovo|returns in this attempt: I went back, but I do not retry)', log))
    same = len(re.findall(r"\[RECOVERY\] (?:L'obiettivo .* quello del tentativo appena fallito|The goal .* that of the attempt that just failed)", log))
    if excl + giveup + same == 0:
        out.append(("Returns with memory and recovery", 'NOT TRIGGERED', "no going back from a trap"))
    else:
        out.append(("Returns with memory and recovery", 'OK',
                    f"edges excluded for the attempt {excl}, cells deferred after 2 returns {giveup}, "
                    f"recoveries deferred because same goal {same}; check there are no repeated loops"))

    # 8. Wait before GraphNav moves: from the preparation (shortcuts, Spot's loop
    #    closure) to departure. On 22-10 16:04 the loop closure timed out 20 times,
    #    30 s each: 603 s of waiting against 257 s of travel.
    #    The final loop closure (before the return to wp_0) has its own limit and does not
    #    count as waiting: it is subtracted from that trip's wait.
    final_closure = [float(t) for t in re.findall(
        r"(?:Chiusura d'anello di Spot|Spot loop closure) \((?:completa|full)\): \d+ (?:archi|edges) in ([\d.]+) s", log)]
    waits, trips, prep, to_wp0 = [], [], None, False
    for line in log.splitlines():
        t = log_time(line)
        if t is None:
            continue
        if re.search(r'\[GRAPHNAV\] (?:prima (?:dello|del) |before )', line):
            prep, to_wp0 = t, 'wp_0' in line
        elif re.search(r'\[GRAPHNAV\] .*: (?:avvio \(massimo|start \(at most)', line) and prep is not None:
            w = t - prep
            if to_wp0 and final_closure:
                w = max(0.0, w - final_closure[-1])
            waits.append(w)
            prep = None
        m = re.search(r'\[GRAPHNAV\] .*: (?:arrivato|arrived) in (\d+) s', line)
        if m:
            trips.append(int(m.group(1)))
    #    Between 26 and 28-10 there was also a quick loop closure before every trip,
    #    with its own limit on Spot's side: a wait up to the limit is normal, beyond it not.
    timeouts = len(re.findall(r"(?:Chiusura d'anello di Spot non riuscita|Spot loop closure failed) \(TimedOut", log))
    loop_fail = len(re.findall(r"(?:Chiusura d'anello di Spot|Spot loop closure) \([^)]*\): (?:tentativo|attempt) \d+ (?:non riuscito|failed)", log))
    #    Quick (without ICP) only in the missions of 26-10; full at the end of the mission (limit
    #    GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
    loops = [(int(n), float(t), float(b), 'SCADUTO' in rest or 'TIMED OUT' in rest, mode) for mode, n, t, b, rest in re.findall(
        r"(?:Chiusura d'anello di Spot|Spot loop closure)(?: \(([^)]*)\))?: (\d+) (?:archi|edges) in ([\d.]+) s \((?:limite|limit) (\d+) s, ([^)]*)\)", log)]
    fast = [l for l in loops if 'veloce' in l[4] or 'quick' in l[4]]
    final = [l for l in loops if not ('veloce' in l[4] or 'quick' in l[4])]
    loops = [l[:4] for l in loops]
    if not waits:
        out.append(("Wait before GraphNav", 'NOT TRIGGERED', "no GraphNav move"))
    else:
        limit = max([b for _, _, b, _, _ in fast] + [0.0]) + 3.0
        long_w = sum(1 for w in waits if w > max(5.0, limit))
        det = (f"{len(waits)} moves: wait {sum(waits):.0f} s in all (maximum {max(waits):.1f} s, "
               f"{long_w} beyond the limit), travel {sum(trips)} s; RPC timeouts {timeouts}")
        if fast:
            det += (f"; quick loop closure {len(fast)} times, {sum(l[1] for l in fast):.0f} s "
                    f"(maximum {max(l[1] for l in fast):.1f} s), {sum(l[0] for l in fast)} edges, "
                    f"{sum(l[3] for l in fast)} partial")
        if final:
            det += (f"; at the end of the mission {final[-1][0]} edges in {final[-1][1]:.0f} s")
        if loop_fail:
            det += f"; failed loop-closure attempts {loop_fail}"

        out.append(("Wait before GraphNav", 'OK' if not long_w and not timeouts and not loop_fail
                    else 'CHECK',
                    det + "; expected: no RPC timeout or failed attempt, waits within the limit"))

    # 9. Goal choice and arrival just outside the cell (since 26-10).
    refused = len(re.findall(r'\[TARGET\] (?:Obiettivo .* escluso per il resto della missione|Goal .* excluded for the rest of the mission)', log))
    unreach = len(re.findall(r"(?:NESSUN punto della cella e' raggiungibile dal robot nel grafo|NO point of the cell is reachable from the robot in the graph)", log))
    outside_ok = len(re.findall(r"\[(?:OBIETTIVO|GOAL)\] (?:Il robot e' fuori dalla cella .* vale come arrivo|The robot is outside cell .* counts as arrival)", log))
    outside_no = len(re.findall(r"(?:ancora fuori dalla cella .*: non vale come arrivo|still outside cell .*: it does not count as arrival)", log))
    disc = len(re.findall(r'\[(?:GRAFO DISCONNESSO|GRAPH DISCONNECTED)\]', log))
    chosen = re.findall(r"\[TARGET\] (?:Cella|Cell) \((\d+),(\d+)\): .*?\((-?[\d.]+), (-?[\d.]+)\)", log)
    repeat = 0
    excl = {}
    for line in log.splitlines():
        m = re.search(r'\[TARGET\] (?:Obiettivo|Goal) \((-?[\d.]+), (-?[\d.]+)\) (?:della cella|of cell) \((\d+),(\d+)\) (?:escluso|excluded)', line)
        if m:
            excl.setdefault((m.group(3), m.group(4)), []).append((float(m.group(1)), float(m.group(2))))
            continue
        m = re.search(r"\[TARGET\] (?:Cella|Cell) \((\d+),(\d+)\): .*?\((-?[\d.]+), (-?[\d.]+)\)", line)
        if m and "rifiutati dal fronte, riprovo" not in line and "already been refused by the frontier" not in line:
            p = (float(m.group(3)), float(m.group(4)))
            repeat += any(math.dist(p, q) < 0.30 for q in excl.get((m.group(1), m.group(2)), []))
    if not chosen:
        out.append(("Goal choice and arrival", 'NOT TRIGGERED', "no [TARGET] line"))
    else:
        ok = repeat == 0 and outside_no <= 3
        out.append(("Goal choice and arrival", 'OK' if ok else 'CHECK',
                    f"{len(chosen)} choices; goals excluded for the mission {refused}, proposed again after "
                    f"the exclusion {repeat}; choices without reachable points {unreach}, graph disconnected "
                    f"at the start {disc}; arrivals with the robot just outside {outside_ok}, refusals "
                    f"'outside the cell' {outside_no}; expected: none proposed again, few refusals outside "
                    f"the cell (on 22-10 there were 44)"))

    # 10. Returns with GraphNav: how far the robot really moves.
    d_ret = [float(d) for d in re.findall(r"\[(?:RITORNO|RETURN)\] (?:Torno a|Going back to) \S+, (?:a )?([\d.]+) m (?:dal robot|from the robot)", log)]
    in_place = len(re.findall(r"\[(?:RITORNO|RETURN)\] (?:La briciola piu' lontana dietro.*non mi muovo|The farthest breadcrumb behind.*I do not move)", log))
    if not d_ret and not in_place:
        out.append(("Returns with GraphNav", 'NOT TRIGGERED', "no return"))
    else:
        out.append(("Returns with GraphNav", 'OK',
                    f"{len(d_ret)} returns" + (f" of {min(d_ret):.2f}-{max(d_ret):.2f} m" if d_ret else "")
                    + f", {in_place} without movement (breadcrumbs all within 0.40 m: it replans from "
                    f"where it is)"))

    # 11. Odometry drift at GraphNav arrivals (since the evening of 26-10, see graphnav_log_drift).
    drift = [(float(a), float(d)) for a, d in re.findall(
        r"\[(?:DERIVA|DRIFT)\] .*?(?:creato|created) ([\d.]+) min (?:fa|ago) -> (?:l'odometria|the odometry) \(VISION\) (?:e' scostata di|is off by) ([\d.]+) m", log)]
    if not drift:
        out.append(("Odometry drift", 'NOT TRIGGERED', "no measurement (no GraphNav arrivals on "
                    "our waypoints, or mission before the evening of 26-10)"))
    else:
        old_ = [d for a, d in drift if a >= 10.0]
        det = (f"{len(drift)} measurements at GraphNav arrivals: maximum {max(d for _, d in drift):.2f} m, last "
               f"{drift[-1][1]:.2f} m (waypoint created {drift[-1][0]:.0f} min before)")
        if old_:
            det += f"; on waypoints created 10 min or more before: median {np.median(old_):.2f} m"
        out.append(("Odometry drift", 'DECISION',
                    det + "; it is the shift of our global maps over the areas seen earlier "
                    "(details in drift.csv)"))
    return out


# --------------------------------------------------------------------------------------
# Printing
# --------------------------------------------------------------------------------------
def report(name):
    f = mission_files(name)
    if f['log'] is None:
        print(f"Mission {name}: mission_log.txt not found.")
        return
    log = open(f['log'], errors='replace').read()
    summary = modern_keys(json.load(open(f['summary']))) if f['summary'] else None
    stats = (summary or {}).get('mission_stats', {})
    print(f"\n=== Mission {name} ===")

    c = section_cells(log)
    g = c['grid']
    print(f"Grid: {f'{g[0]} x {g[1]} cells of {g[2]:.1f} m' if g else 'not stated in the log'}"
          f" | duration {c['duration_s'] / 60:.1f} min" if c['duration_s'] else "")
    print(f"Cells: {c['visited']} visited, {c['blocked']} blocked, {c['attempts']} attempts")
    # Progress (since 28-10): from mission_summary.json, or from the last [PROGRESSO] line of the log
    # (mission interrupted before the summary).
    pr = ((summary or {}).get('mission_stats') or {}).get('progress')
    if pr:
        print(f"Progress: {pr['closed_cells']}/{pr['total_cells']} cells closed "
              f"({pr['percent']:.0f}%, including the starting one), area seen "
              + (f"{pr['seen_area']:.0f}%" if pr.get('seen_area') is not None else "?")
              + f", {pr['minutes']:.1f} min of exploration"
              + (f" ({pr['minutes_per_cell']:.2f} min per cell)" if pr.get('minutes_per_cell') else "")
              + (f", battery at the end {pr['battery']:.0f}%" if pr.get('battery') is not None else "")
              + ("" if pr.get('final') else "  [MISSION INTERRUPTED: last recorded value]"))
    else:
        lines = [modern_log_line(l) for l in log.splitlines() if '[PROGRESSO]' in l or '[PROGRESS]' in l]
        if lines:
            print("Progress (last line of the log): " + lines[-1].split('[PROGRESS]', 1)[1].strip())
    # Mission switches (since 21-10): from the summary, or from the [CONFIG] line of the log.
    sw = (summary or {}).get('switches')
    if sw:
        print("Switches: " + "  ".join(f"{k.split('.')[-1]}={v}" for k, v in sw.items()))
    else:
        m = re.search(r'\[CONFIG\] (?:INTERRUTTORI|SWITCHES): (.*)', log)
        print("Switches: " + (m.group(1).strip() if m else "not recorded (mission before 21-10)"))
    # Mission graph (since 22-10): node method and, for random nodes, the seed.
    g = (summary or {}).get('graph')
    if g:
        extra = (f"{g.get('distribution')}, {g.get('node_density_m2'):g} nodes/m2, seed {g.get('seed')}"
                 if modern(g.get('method')) == 'random' else f"spacing {g.get('spacing_m')} m")
        print(f"Graph: {g.get('method')} ({extra}), {g.get('nodes')} nodes, edges "
              f"{g.get('edge_min_m'):g}-{g.get('edge_max_m'):g} m, coverage {g.get('coverage_m')} m")
    else:
        m = re.search(r'\[CONFIG\] (?:Grafo|Graph): (.*)', log)
        r = re.search(r"\[(?:GRAFO|GRAPH)\] (?:Nodi casuali|Random nodes): (.*?) \((?:per ripetere|to repeat)", log)
        print("Graph: " + (r.group(1) if r else m.group(1).strip() if m else "lattice (mission before 22-10)"))

    t = section_times(log, summary)
    if t.get('breakdown'):
        tot = sum(t['breakdown'].values())
        parts = ", ".join(f"{k} {v:.0f} s" for k, v in sorted(t['breakdown'].items(), key=lambda kv: -kv[1])[:5])
        print(f"\nMeasured time {tot:.0f} s: {parts}")
    if t['graph']:
        loc = [s for k, s in t['graph'] if k == 'local update']
        full = [s for k, s in t['graph'] if k == 'full build']
        line = f"Graph: {len(full)} full builds" + (f" (mean {np.mean(full):.1f} s)" if full else "")
        if loc:
            line += f", {len(loc)} local updates (mean {np.mean(loc):.1f} s, maximum {max(loc):.1f} s)"
        print(line)
    if t['closure']:
        print("Closure: " + ", ".join(f"{k} {v:.0f} s" for k, v in t['closure'].items()))

    mo = section_motion(read_csv(f['segments']), read_csv(f['trajectory']))
    print(f"\nMotion: {mo['segments']} legs" + (f", {mo['ok']} successful" if 'ok' in mo else "")
          + (f", {mo['distance_m']:.0f} m travelled" if 'distance_m' in mo else ""))
    if 'speed' in mo:
        line = f"Smoothness: {mo['speed']:.2f} m/s on average over {mo['move_s']:.0f} s of movement"
        if 'turn_s' in mo and mo['turn_s'] + mo['walk_s'] > 0:
            tot = mo['turn_s'] + mo['walk_s']
            line += (f"; standing to rotate or start {mo['turn_s']:.0f} s ({100 * mo['turn_s'] / tot:.0f}%), "
                     f"walking {mo['walk_s']:.0f} s")
        print(line)
    if 'tiers' in mo:
        print("Gaits: " + ", ".join(f"{k} {v}" for k, v in sorted(mo['tiers'].items())))
        for lim, (n, hard) in mo['steep'].items():
            print(f"  legs with the nose above {lim:.0f} degrees: {n}, in HARD {hard}")
        if mo['max_pitch'] is not None:
            print(f"  maximum pitch {mo['max_pitch']:.1f} degrees; HARD on calm legs (below 8 degrees): {mo['calm_hard']}")

    fr = section_frontier(f['scans'], mission_thresholds(summary, log))
    if fr:
        print(f"\nFrontier, reason per scan: " + ", ".join(f"{k} {v}" for k, v in fr['reasons'].most_common()))
        if fr['stops']:
            print(f"Stops for obstacle: {fr['stops']}")
            for k, v in fr['bands'].most_common():
                print(f"  {k}: {v}")
            for k, v in fr['origin'].most_common():
                print(f"  {k}: {v}")
    else:
        print("\nFrontier: scans missing or in the old format, section skipped")

    if stats:
        print(f"\nReplans {stats.get('replan_count', '?')}, immediate replans "
              f"{stats.get('early_replans', '?')}, confirmed blocks {stats.get('frontier_blocks', '?')}, "
              f"goals chosen again {stats.get('goal_repicks', '?')}, returns with GraphNav "
              f"{stats.get('graphnav_backtracks', '?')}, falls {stats.get('falls', 0)}"
              + (f", GraphNav moves avoided (the graph did not reach the cell) "
                 f"{stats['graphnav_trips_avoided']}" if 'graphnav_trips_avoided' in stats else ""))

    b = section_battery(stats, log)
    if b:
        print("\n" + b)

    r = section_return(log, f['graph'])
    if r.get('time_s') is not None or r.get('graph') or r.get('failed') or r.get('offset_m') is not None:
        line = f"\nReturn to wp_0: {r['time_s']} s" if r.get('time_s') is not None else "\nReturn to wp_0:"
        if r.get('failed'):
            fl = r['failed']
            line += (" NOT successful, stopped " + (f"{fl['metres']:.2f} m from wp_0" if fl['metres'] is not None
                                                else "at an unknown distance")
                     + f" ({fl['reason']})" + ("; fiducial and drift measured anyway" if fl['near'] else ""))
        if r.get('path_m'):
            a, b, n = r['path_m']
            line += f", path {a:.1f} m -> {b:.1f} m with {n} shortcuts"
        if r.get('graph'):
            line += f" | in the downloaded graph from {r['graph']['from_wp']}: {r['graph']['metres']} m"
        if r.get('offset_m') is not None:
            line += f"\n  according to the starting tag: arrived {r['offset_m']:.2f} m from the start"
            if r.get('corrections'):
                n, fin = r['corrections']
                line += (f", CORRECTED with the fiducial ({n} " + ("correzione" if n == 1 else "corrections")
                         + (f"), at the end {fin:.2f} m" if fin is not None else "), final offset not measured"))
            else:
                line += ", no correction needed"
        print(line)

    ar = section_arrival_rotation(read_csv(f['trajectory']))
    if ar:
        m = re.search(r"GRAPHNAV_ARRIVAL_HEADING=(\S+)", log)
        print(f"Rotation after the GraphNav arrivals ({m.group(1) if m else 'waypoint'}): {len(ar)} arrivals, "
              f"mean {np.mean([a for a, _ in ar]):.0f} degrees, {sum(s for _, s in ar):.0f} s in place in all "
              f"(26-10 15:11, waypoint heading: 79 degrees, 38 s over 8 arrivals)")

    print("\nTest checks (features never tried on the robot, see PROVA_CAMPO.md):")
    for check_name, outcome, det in section_checks(log, read_csv(f['segments'])):
        print(f"  [{outcome:^12s}] {check_name}: {det}")


def one_line(name):
    f = mission_files(name)
    if f['log'] is None:
        return None
    log = open(f['log'], errors='replace').read()
    c = section_cells(log)
    if not c['attempts']:
        return None
    seg = read_csv(f['segments'])
    r = section_return(log, [])
    dur = f"{c['duration_s'] / 60:5.1f} min" if c['duration_s'] else "    ? min"
    ret = f"{r['time_s']:4d} s" if r.get('time_s') is not None else "   - "
    return (f"{name}  {dur}  cells {c['visited']:2d} visited {c['blocked']:2d} blocked  "
            f"legs {len(seg):3d}  return {ret}  scans {len(f['scans']):3d}")


def main():
    args = sys.argv[1:]
    names = sorted((os.path.basename(p)[len('Mission_'):] for p in glob.glob(os.path.join(LOGS, 'Mission_*'))),
                   key=lambda n: (n[6:10], n[3:5], n[0:2], n[11:]))
    if '--all' in args:
        for n in names:
            line = one_line(n)
            if line:
                print(line)
        return
    if args:
        report(args[0].replace('Mission_', ''))
    elif names:
        report(names[-1])
    else:
        print("No mission in MissionLogs/.")


if __name__ == '__main__':
    main()
