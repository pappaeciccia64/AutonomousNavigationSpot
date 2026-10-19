#!/usr/bin/env python3
"""
Rapporto di una missione, da leggere subito dopo una prova.

Raccoglie in una pagina di testo i numeri che servono a capire com'e' andata: celle, tempi,
movimento e andature, fermate del fronte e il loro perche', blocchi e ripianificazioni,
ritorno a wp_0. Legge solo i file che la missione salva; come mission_viewer.py serve solo
numpy, NON l'SDK di Spot (che se c'e' viene usato per la strada di ritorno nel grafo GraphNav).

    python3 mission_report.py                         # l'ultima missione in MissionLogs/
    python3 mission_report.py 13-10-2026_12-40-38     # una missione per nome
    python3 mission_report.py --tutte                 # una riga di sintesi per ogni missione

File letti (quelli che mancano vengono saltati: le missioni vecchie ne hanno meno):
    MissionLogs/Mission_<nome>/mission_log.txt, mission_summary.json, segments.csv, trajectory.csv
    MissionMap/Mission_<nome>/scans/*.npz       le scansioni del ciclo di navigazione
    graph/Mission_<nome>/Grafo_*/               il grafo GraphNav scaricato a fine missione
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

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, 'MissionLogs')
MAPS = os.path.join(HERE, 'MissionMap')
GRAPHS = os.path.join(HERE, 'graph')

# Come in spotGrid / easy_walk: copiate qui perche' il rapporto non importa il codice di missione.
FRONTIER_CLEARANCE_M = 0.30        # spotGrid.FRONTIER_CLEARANCE_M
INTRUSION_M = 0.25                 # mezza larghezza del corpo (spotGrid.ROBOT_HALF_WIDTH_M)
LIVE_PREFILTER_OD_M = 0.20         # prm_graph.LIVE_PREFILTER_OD_M
TERRAIN_VALID_MIN = 0.5            # spotGrid.TERRAIN_VALID_MIN
PITCH_LIMITS_DEG = (14.0, 18.0)    # tratti "ripidi" per il controllo delle andature


# --------------------------------------------------------------------------------------
# Lettura
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
        graph=sorted(glob.glob(os.path.join(GRAPHS, f'Mission_{name}', 'Grafo_*'))))


def read_csv(path):
    if path is None:
        return []
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


def log_time(line):
    """Secondi del giorno dall'orario hh:mm:ss.mmm a inizio riga, o None."""
    m = re.match(r'(\d\d):(\d\d):(\d\d(?:\.\d+)?)', line)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None


# --------------------------------------------------------------------------------------
# Sezioni
# --------------------------------------------------------------------------------------
def section_cells(log):
    grid = re.search(r'Griglia di missione: (\d+) x (\d+) celle da ([\d.]+) m', log)
    attempts = re.findall(r'\[ATTEMPT\] Trying to enter cell \((\d+),(\d+)\)', log)
    visited = re.findall(r'\[VISITED\] Cell \((\d+),(\d+)\) marked as visited', log)
    blocked = set(re.findall(r'\[BLOCKED\] Cell \((\d+),(\d+)\) marked as blocked', log))
    times = [t for t in (log_time(l) for l in log.splitlines()) if t is not None]
    out = dict(grid=(int(grid.group(1)), int(grid.group(2)), float(grid.group(3))) if grid else None,
               attempts=len(attempts), visited=len(set(visited)), blocked=len(blocked),
               duration_s=(times[-1] - times[0]) if len(times) > 1 else None)
    return out


def section_times(log, summary):
    out = {}
    if summary:
        out['ripartizione'] = summary.get('mission_stats', {}).get('time_s', {})
    upd = [(kind, float(s)) for kind, s in re.findall(
        r'\[TEMPO\] (aggiornamento locale|costruzione completa) del grafo: ([\d.]+) s', log)]
    out['grafo'] = upd
    closing = {}
    block = re.search(r'\[TEMPO\] Chiusura della missione.*?\n((?:.*\n){1,8})', log)
    if block:
        for name, s in re.findall(r'\]?\s{2,}([a-z][a-z0-9_ \']+?)\s{2,}([\d.]+) s', block.group(1)):
            closing[name.strip()] = float(s)
    out['chiusura'] = closing
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
    return out


def _seen_value(d, x, y):
    """obstacle_distance filtrata in (x, y) se la scansione d ha visto quel punto, se no None."""
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


def section_frontier(scans):
    """Fermate del fronte, dalle scansioni salvate: perche' e, per gli ostacoli, se si vedevano gia'."""
    if not scans:
        return None
    data = [np.load(f) for f in scans]
    if 'frontier_reason' not in data[0].files:
        return None
    reasons = Counter(str(d['frontier_reason']) for d in data if 'frontier_reason' in d.files)
    events, last = [], None
    for i, d in enumerate(data):
        if str(d['frontier_reason']) != 'ostacolo':
            last = None
            continue
        p = np.asarray(d['frontier_stop_xy'], dtype=float)
        it = int(d['iteration'])
        if last is not None and last[0] == it and np.hypot(*(p - last[1])) < 0.3:
            continue                                  # stessa fermata, scansione dopo
        last = (it, p)
        before = next((v for v in (_seen_value(data[j], *p) for j in range(i - 1, -1, -1)) if v is not None), None)
        events.append((float(d['frontier_value']), before))
    bands = Counter()
    origin = Counter()
    for value, before in events:
        bands['intrusione vera (sotto 0.25 m)' if value < INTRUSION_M else
              'di striscio (0.25-0.30 m)' if value < FRONTIER_CLEARANCE_M else
              "tenuta dall'isteresi (0.30 m o piu')"] += 1
        origin['mai visto prima' if before is None else
               'visto prima sotto 0.20 m' if before < LIVE_PREFILTER_OD_M else
               'visto prima fra 0.20 e 0.30 m' if before < FRONTIER_CLEARANCE_M else
               'visto prima sopra 0.30 m (letto da lontano)'] += 1
    return dict(reasons=reasons, stops=len(events), bands=bands, origin=origin)


def section_return(log, graph_dirs):
    out = {}
    m = re.search(r'ritorno a wp_0: arrivato in (\d+) s', log)
    out['tempo_s'] = int(m.group(1)) if m else None
    m = re.search(r'strada verso wp_0 ([\d.]+) m -> ([\d.]+) m con (\d+) scorciatoie', log)
    if m:
        out['strada_m'] = (float(m.group(1)), float(m.group(2)), int(m.group(3)))
    start = re.findall(r"prima di 'ritorno a wp_0': localizzato su (\S+)", log)
    if graph_dirs and start:
        out['grafo'] = _graph_route(graph_dirs[-1], start[-1])
    return out


def _graph_route(gdir, start_name):
    """Lunghezza della strada GraphNav da start_name a wp_0 nel grafo scaricato (serve l'SDK)."""
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
            return dict(da=start_name, metri=round(d, 1), waypoint=len(g.waypoints), archi=len(g.edges))
        if d > dist[u]:
            continue
        for v, w in adj.get(u, []):
            if d + w < dist.get(v, float('inf')):
                dist[v] = d + w
                heapq.heappush(pq, (d + w, v))
    return None


# --------------------------------------------------------------------------------------
# Stampa
# --------------------------------------------------------------------------------------
def report(name):
    f = mission_files(name)
    if f['log'] is None:
        print(f"Missione {name}: mission_log.txt non trovato.")
        return
    log = open(f['log'], errors='replace').read()
    summary = json.load(open(f['summary'])) if f['summary'] else None
    stats = (summary or {}).get('mission_stats', {})
    print(f"\n=== Missione {name} ===")

    c = section_cells(log)
    g = c['grid']
    print(f"Griglia: {f'{g[0]} x {g[1]} celle da {g[2]:.1f} m' if g else 'non indicata nel log'}"
          f" | durata {c['duration_s'] / 60:.1f} min" if c['duration_s'] else "")
    print(f"Celle: {c['visited']} visitate, {c['blocked']} bloccate, {c['attempts']} tentativi")

    t = section_times(log, summary)
    if t.get('ripartizione'):
        tot = sum(t['ripartizione'].values())
        parts = ", ".join(f"{k} {v:.0f} s" for k, v in sorted(t['ripartizione'].items(), key=lambda kv: -kv[1])[:5])
        print(f"\nTempo misurato {tot:.0f} s: {parts}")
    if t['grafo']:
        loc = [s for k, s in t['grafo'] if k == 'aggiornamento locale']
        full = [s for k, s in t['grafo'] if k == 'costruzione completa']
        line = f"Grafo: {len(full)} costruzioni complete" + (f" (media {np.mean(full):.1f} s)" if full else "")
        if loc:
            line += f", {len(loc)} aggiornamenti locali (media {np.mean(loc):.1f} s, massimo {max(loc):.1f} s)"
        print(line)
    if t['chiusura']:
        print("Chiusura: " + ", ".join(f"{k} {v:.0f} s" for k, v in t['chiusura'].items()))

    mo = section_motion(read_csv(f['segments']), read_csv(f['trajectory']))
    print(f"\nMovimento: {mo['segments']} tratti" + (f", {mo['ok']} riusciti" if 'ok' in mo else "")
          + (f", {mo['distance_m']:.0f} m percorsi" if 'distance_m' in mo else ""))
    if 'tiers' in mo:
        print("Andature: " + ", ".join(f"{k} {v}" for k, v in sorted(mo['tiers'].items())))
        for lim, (n, hard) in mo['steep'].items():
            print(f"  tratti col muso oltre {lim:.0f} gradi: {n}, in HARD {hard}")
        if mo['max_pitch'] is not None:
            print(f"  beccheggio massimo {mo['max_pitch']:.1f} gradi; HARD su tratti tranquilli (sotto 8 gradi): {mo['calm_hard']}")

    fr = section_frontier(f['scans'])
    if fr:
        print(f"\nFronte, motivo per scansione: " + ", ".join(f"{k} {v}" for k, v in fr['reasons'].most_common()))
        if fr['stops']:
            print(f"Fermate per ostacolo: {fr['stops']}")
            for k, v in fr['bands'].most_common():
                print(f"  {k}: {v}")
            for k, v in fr['origin'].most_common():
                print(f"  {k}: {v}")
    else:
        print("\nFronte: scansioni assenti o di formato vecchio, sezione saltata")

    if stats:
        print(f"\nRipianificazioni {stats.get('replan_count', '?')}, ripianificazioni immediate "
              f"{stats.get('early_replans', '?')}, blocchi confermati {stats.get('frontier_blocks', '?')}, "
              f"obiettivi riscelti {stats.get('goal_repicks', '?')}, ritorni con GraphNav "
              f"{stats.get('graphnav_backtracks', '?')}, cadute {stats.get('falls', 0)}")

    r = section_return(log, f['graph'])
    if r.get('tempo_s') is not None or r.get('grafo'):
        line = f"\nRitorno a wp_0: {r['tempo_s']} s" if r.get('tempo_s') is not None else "\nRitorno a wp_0:"
        if r.get('strada_m'):
            a, b, n = r['strada_m']
            line += f", strada {a:.1f} m -> {b:.1f} m con {n} scorciatoie"
        if r.get('grafo'):
            line += f" | nel grafo scaricato da {r['grafo']['da']}: {r['grafo']['metri']} m"
        print(line)


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
    ret = f"{r['tempo_s']:4d} s" if r.get('tempo_s') is not None else "   - "
    return (f"{name}  {dur}  celle {c['visited']:2d} visitate {c['blocked']:2d} bloccate  "
            f"tratti {len(seg):3d}  ritorno {ret}  scansioni {len(f['scans']):3d}")


def main():
    args = sys.argv[1:]
    names = sorted((os.path.basename(p)[len('Mission_'):] for p in glob.glob(os.path.join(LOGS, 'Mission_*'))),
                   key=lambda n: (n[6:10], n[3:5], n[0:2], n[11:]))
    if '--tutte' in args:
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
        print("Nessuna missione in MissionLogs/.")


if __name__ == '__main__':
    main()
