#!/usr/bin/env python3
"""
Snapshot of what the code DECIDES, to check that a cleanup does not change the behaviour.

    python3 tools/behaviour_snapshot.py record before.json        # before touching the code
    ... a cleanup step ...
    python3 tools/behaviour_snapshot.py check before.json         # after: same behaviour?
    python3 tools/behaviour_snapshot.py compare before.json after.json

Three parts, all deterministic (same code -> same snapshot, checked by running it twice):

  perception  real scans saved by missions (MissionMap/<mission>/scans), packed back into
              Spot's messages (tools/saved_scan.py) and run through the real fetch_scan +
              process_scan, in mission order (the low-obstacle filter has memory). Per scan,
              a fingerprint of every output layer: obstacle distance after the filters,
              corrected terrain, valid cells, never-written cells, slope, roughness,
              obstacle mask.
  replay      the same missions through tests/test_replay.py (global maps, graph updates,
              Dijkstra at every cell start): the fingerprint of all edges and weights after
              every graph update, every path found, and the replay summary.
  missions    whole fake missions, stop-and-go (tests/test_mission.py, worlds A and B and
              random box worlds), each in its own process: the mission log with times,
              dates and temporary folders taken out, line by line.

The cruise (continuous motion) is NOT in the snapshot: it runs in real time with two threads,
two runs of the same code already differ. Check it with tests/test_cruise.py and
varianti/tools/compare_route_line.py.

check / compare print what differs (first scans, first edges, first log lines) and exit 1
if anything does. A difference is not necessarily a bug: if the step was meant to change
behaviour, say so in its commit and record a new snapshot.

The mission data (MissionMap/, MissionLogs/, graph/) are not in git: --data is the folder
that holds them (default: the repository of this script).
"""
import argparse
import contextlib
import datetime
import difflib
import hashlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path[:0] = [REPO, HERE, os.path.join(REPO, 'tests')]

MISSIONS = ['13-10-2026_12-14-30', '13-10-2026_12-40-38', '26-10-2026_15-11-21', '28-10-2026_13-50-46']
WORLDS = ['A', 'B', 'rand1', 'rand2', 'rand3']
PARTS = ('perception', 'replay', 'missions', 'cruise')
DEFAULT_PARTS = ('perception', 'replay', 'missions')
# cruise: not deterministic (real time, two threads): each world run REPEAT times, the
# reference is the range of each number; a new run passes if inside it, within these margins.
REPEAT = 3
CRUISE_MARGIN = dict(seen=3.0, walked_frac=0.15, clearance=0.03)
MARK = '@@SNAPSHOT@@'
WORKER_TIMEOUT_S = 900     # one scan set, replay or fake mission: a hung one must not block the queue


def _h(*arrays, decimals=6):
    """Short fingerprint of arrays (rounded: the last float bits are not behaviour)."""
    import numpy as np
    m = hashlib.sha1()
    for a in arrays:
        a = np.asarray(a)
        if a.dtype.kind == 'f':
            a = np.round(a.astype(np.float64), decimals)
            a[a == 0] = 0.0                          # -0.0 and 0.0 alike
        m.update(str(a.shape).encode())
        m.update(np.ascontiguousarray(a).tobytes())
    return m.hexdigest()[:12]


# --------------------------------------------------------------------------------------
# Workers (each in its own process: the mission code keeps state in module globals)
# --------------------------------------------------------------------------------------
def work_perception(mission, data):
    import glob
    import numpy as np
    from saved_scan import decode_and_process, load
    files = glob.glob(os.path.join(data, 'MissionMap', f'Mission_{mission}', 'scans', '*.npz'))
    files.sort(key=lambda f: float(np.load(f)['time']))
    out = []
    for f in files:
        _lg, s = decode_and_process(load(f))
        layers = dict(od=s.od, terrain=s.terrain, is_valid=s.is_valid, unwritten=s.unwritten,
                      grad=s.grad, rough=s.rough, mask=s.obstacle_mask)
        out.append(dict(scan=os.path.basename(f),
                        layers={k: _h(v) for k, v in layers.items()},
                        obstacle_cells=int(np.sum(np.asarray(s.obstacle_mask) < 0)),
                        valid_cells=int(np.sum(np.asarray(s.is_valid, bool)))))
    return out


def work_replay(mission, data):
    import numpy as np
    import test_replay as R
    from spot_explorer.mission import mission_prm
    from spot_explorer.planning import roadmap
    R.ROOT = data
    graphs, paths = [], []
    rebuild = mission_prm.rebuild_prm_edges

    def recorded_rebuild(prm, gm, *a, **kw):
        r = rebuild(prm, gm, *a, **kw)
        if kw.get('incremental', True):              # not the full-build twin of the replay check
            keys = sorted((min(u, v), max(u, v), w) for u, lst in prm.edges.items() for v, w in lst)
            graphs.append(dict(edges=len(keys), h=_h(np.array([k[:2] for k in keys], dtype=np.int64),
                                                     np.array([k[2] for k in keys], dtype=np.float64))))
        return r
    mission_prm.rebuild_prm_edges = recorded_rebuild
    dijkstra = roadmap.PRM.find_path_dijkstra

    def recorded_dijkstra(self, start, goal, *a, **kw):
        p = dijkstra(self, start, goal, *a, **kw)
        paths.append([int(start), int(goal), None if p is None else [int(n) for n in p]])
        return p
    roadmap.PRM.find_path_dijkstra = recorded_dijkstra
    _name, out = R.replay_mission((mission, True, None))
    if out is None:
        return None
    for k in ('seconds', 'graph_time_s'):
        out.pop(k, None)
    return dict(summary=json.loads(json.dumps(out, default=str)), graphs=graphs, paths=paths)


_NORMALISE = [
    (re.compile(r'\d{2}:\d{2}:\d{2}(\.\d+)?'), 'HH:MM:SS'),
    (re.compile(r'/tmp/(test_mission_\w+/)?fake_mission_\w+'), '<tmp>'),
    (re.compile(r'(Mission|Graph)_\d{2}-\d{2}-\d{4}_\d{2}-\d{2}-\d{2}'), r'\1_<date>'),
    (re.compile(r'\(\d+ failed readings\)'), '(<n> failed readings)'),   # the battery is read on a timer
    (re.compile(r'\b\d+(\.\d+)? ?(s|ms|min)\b'), '<t>'),
    (re.compile(r'\b\d+(\.\d+)? ?(m/s)\b'), '<v>'),
]
# Timing lines, and the time profile of the summary ("  motion   <t>  ( 39.1%)").
# The [CONFIG] SWITCHES line lists the switches: it changes whenever one is removed.
_DROP = re.compile(r'\[CONFIG\] SWITCHES|\[TIMING\]|seconds|duration|elapsed|took|\bin <t>|<t> +\( *[\d.]+%\)', re.I)


# Rows of a timing table, sorted by time ("  sit and power off     <t>"): their order moves.
_TIME_ROW = re.compile(r'^HH:MM:SS +[\w ,/()+-]+? {2,}<t>$')


def normalise(log):
    lines = []
    for line in log.splitlines():
        for rx, rep in _NORMALISE:
            line = rx.sub(rep, line)
        if not _DROP.search(line) and not _TIME_ROW.match(line):
            lines.append(line.rstrip())
    return lines


def random_world(seed, rows, cols, cell):
    """Random boxes, the start corner kept free (the worlds of compare_route_line, now in
    varianti/tools: the same seeds give the same worlds)."""
    import numpy as np
    rng = np.random.default_rng(seed)
    w, h = cols * cell, rows * cell
    boxes = []
    while len(boxes) < rng.integers(6, 10):
        sx, sy = rng.uniform(0.4, 1.2, 2)
        x0, y0 = rng.uniform(0, w - sx), rng.uniform(0, h - sy)
        if x0 < 2.2 and y0 < 2.2:
            continue
        boxes.append((float(x0), float(x0 + sx), float(y0), float(y0 + sy)))
    return boxes


def work_missions(world, data):
    import test_mission as M
    if world.startswith('rand'):
        M.ROWS, M.COLS, M.CELL, M.START = 4, 5, 2.0, (1.0, 1.0, 0.0)
        walls = random_world(int(world[4:]), 4, 5, 2.0)
    else:
        walls = M.WORLDS[world]
    r = M.run(walls=walls, name='snapshot')
    return dict(ok=bool(r['ok'] in (None, True)), lines=normalise(r['log']))


def _clearance(track, rects):
    """Smallest distance of the robot centre from the boxes along the track, m."""
    import numpy as np
    if len(track) == 0 or not rects:
        return None
    t = np.asarray(track, dtype=float)
    best = np.inf
    for x0, x1, y0, y1 in rects:
        dx = np.maximum(np.maximum(x0 - t[:, 0], 0), t[:, 0] - x1)
        dy = np.maximum(np.maximum(y0 - t[:, 1], 0), t[:, 1] - y1)
        best = min(best, float(np.min(np.hypot(dx, dy))))
    return round(best, 3)


def work_cruise(item, data):
    """One fake mission in cruise (item 'world#k'): the numbers that must hold when the code
    changes -- cells, area seen, metres, clearance from the boxes, cruise ends."""
    import numpy as np
    import test_e2e_loop as T
    import test_mission as M
    world = item.split('#')[0]
    if world.startswith('rand'):
        M.ROWS, M.COLS, M.CELL, M.START = 4, 5, 2.0, (1.0, 1.0, 0.0)
        walls = random_world(int(world[4:]), 4, 5, 2.0)
    else:
        walls = M.WORLDS[world]
    variants = [v for v in os.environ.get('SNAPSHOT_VARIANTS', '').split(',') if v]
    if variants:                      # alternatives of the local varianti/ folder (its run.apply)
        import importlib.util
        folder = os.environ.get('SNAPSHOT_VARIANTS_DIR') or os.path.join(REPO, 'varianti')
        sys.path.insert(0, folder)
        spec = importlib.util.spec_from_file_location('varianti_run', os.path.join(folder, 'run.py'))
        vr = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vr)
        vr.apply(variants)
    r = M.run(walls=walls, name='snapshot_cruise', cruise=True)
    log = r['log']
    track = np.array(T.ROBOT.track, dtype=float).reshape(-1, 2)
    seen = re.findall(r'\[COVERAGE\] End: area seen (\d+)%', log) or re.findall(r'area seen (\d+)%', log)
    sg = re.findall(r'stop-and-go instead (\d+)', log)
    return dict(visited=sorted(set(re.findall(r'\[VISITED\] Cell \((\d+,\d+)\)', log))),
                blocked=sorted(set(re.findall(r'\[BLOCKED\] Cell \((\d+,\d+)\)', log))),
                seen=int(seen[-1]) if seen else None,
                walked=round(float(np.sum(np.hypot(*np.diff(track, axis=0).T))) if len(track) > 1 else 0.0, 2),
                clearance=_clearance(track, walls),
                cruise_ends=len(re.findall(r'\[CRUISE\] End', log)),
                stop_and_go=int(sg[-1]) if sg else None,
                seconds=round(float(r.get('seconds', 0.0)), 1),
                blocked_on_spot=log.count('blocked on the spot'),
                start_failures=len(re.findall(r'Attempt start failed on cell', log)),
                frontier_goals=len(re.findall(r'\[FRONTIER\] Goal:', log)),
                traceback='Traceback' in log, ok=bool(r['ok'] in (None, True)))


def compare_cruise(key, runs_a, runs_b):
    """Ranges of the reference runs against the new runs of one world; lines if outside."""
    out = []
    if any(r is None for r in runs_b):
        out.append(f"cruise {key}: {sum(r is None for r in runs_b)} of {len(runs_b)} runs did not end or failed "
                   f"(see the [ERROR] lines of the recording)")
    a = [r for r in runs_a if r]
    b = [r for r in runs_b if r]
    if not a or not b:
        return [f"cruise {key}: no runs to compare"]
    rng = lambda runs, k: (min(r[k] for r in runs if r[k] is not None), max(r[k] for r in runs if r[k] is not None))
    m = CRUISE_MARGIN
    if any(r['traceback'] or not r['ok'] for r in b):
        out.append(f"cruise {key}: a run ended with an error")
    lo, _ = rng(a, 'seen')
    if min(r['seen'] for r in b) < lo - m['seen']:
        out.append(f"cruise {key}: area seen {[r['seen'] for r in b]}% against {[r['seen'] for r in a]}%")
    if min(len(r['visited']) for r in b) < min(len(r['visited']) for r in a):
        out.append(f"cruise {key}: cells visited {[len(r['visited']) for r in b]} against {[len(r['visited']) for r in a]}")
    lo, hi = rng(a, 'walked')
    wb = [r['walked'] for r in b]
    if not (lo * (1 - m['walked_frac']) <= sum(wb) / len(wb) <= hi * (1 + m['walked_frac'])):
        out.append(f"cruise {key}: metres walked {wb} against {[r['walked'] for r in a]}")
    if all(r['clearance'] is not None for r in a + b):
        lo, _ = rng(a, 'clearance')
        if min(r['clearance'] for r in b) < lo - m['clearance']:
            out.append(f"cruise {key}: closest to a box {[r['clearance'] for r in b]} m against {[r['clearance'] for r in a]} m")
    return out


def cruise_table(snap):
    lines = []
    for key, runs in sorted(snap.get('cruise', {}).items()):
        runs = [r for r in runs if r]
        if runs:
            extra = (f"  time {[r.get('seconds') for r in runs]} s  on-the-spot {[r.get('blocked_on_spot') for r in runs]}  "
                     f"start fails {[r.get('start_failures') for r in runs]}  frontier goals {[r.get('frontier_goals') for r in runs]}")
            lines.append(f"  {key:6s} cells {[len(r['visited']) for r in runs]}  seen {[r['seen'] for r in runs]}%  "
                         f"walked {[r['walked'] for r in runs]} m  closest {[r['clearance'] for r in runs]} m  "
                         f"cruise ends {[r['cruise_ends'] for r in runs]}  failed first turns {[r['stop_and_go'] for r in runs]}" + extra)
    return lines


WORKERS = dict(perception=work_perception, replay=work_replay, missions=work_missions, cruise=work_cruise)


def worker_main(part, item, data):
    real = sys.stdout
    with contextlib.redirect_stdout(io.StringIO()):
        res = WORKERS[part](item, data)
    real.write(MARK + json.dumps(res) + '\n')


# --------------------------------------------------------------------------------------
# Record / compare
# --------------------------------------------------------------------------------------
def _run_worker(part, item, data):
    try:
        p = subprocess.run([sys.executable, os.path.abspath(__file__), '_worker', part, item, '--data', data],
                           capture_output=True, text=True, cwd=REPO, timeout=WORKER_TIMEOUT_S)
    except subprocess.TimeoutExpired as e:
        tail = (e.stdout or b'')[-1500:]
        tail = tail.decode(errors='replace') if isinstance(tail, bytes) else tail
        return None, f"did not end within {WORKER_TIMEOUT_S} s (a mission that never ends?); last output:\n{tail}"

    for line in p.stdout.splitlines():
        if line.startswith(MARK):
            return json.loads(line[len(MARK):]), None
    return None, (p.stderr or p.stdout)[-2000:]


def _git_state():
    try:
        head = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=REPO, capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(['git', 'status', '--porcelain', '--', 'spot_explorer'], cwd=REPO,
                               capture_output=True, text=True).stdout.strip()
        return head + (' + uncommitted changes in spot_explorer/' if dirty else '')
    except OSError:
        return '?'


def record(path, parts, missions, worlds, data, jobs, repeat=REPEAT):
    jobs_list = [(p, m) for p in parts if p not in ('missions', 'cruise') for m in missions] + \
                ([('missions', w) for w in worlds] if 'missions' in parts else [])
    snap = dict(meta=dict(date=datetime.datetime.now().isoformat(timespec='seconds'), code=_git_state(), variants=os.environ.get('SNAPSHOT_VARIANTS', ''),
                          repo=REPO, data=data, parts=list(parts)),
                **{p: {} for p in parts})
    print(f"Recording {len(jobs_list)} jobs ({', '.join(parts)}) with {jobs} processes...")
    with ThreadPoolExecutor(jobs) as ex:
        futs = {ex.submit(_run_worker, p, m, data): (p, m) for p, m in jobs_list}
        for fut, (p, m) in futs.items():
            res, err = fut.result()
            if res is None:
                if err:
                    print(f"  [ERROR] {p} {m}:\n{err}")
                else:
                    print(f"  [SKIP] {p} {m}: no data on this PC")
                snap[p][m] = None
                continue
            snap[p][m] = res
            n = len(res) if p == 'perception' else len(res['graphs']) if p == 'replay' else len(res['lines'])
            what = 'scans' if p == 'perception' else 'graph updates' if p == 'replay' else 'log lines'
            print(f"  {p:10s} {m:22s} {n:5d} {what}")
    if 'cruise' in parts:
        # Real time: one at a time, nothing else running (the other parts are done).
        print(f"Cruise: {len(worlds)} worlds x {repeat} runs, one at a time...")
        for w in worlds:
            snap['cruise'][w] = []
            for k in range(repeat):
                res, err = _run_worker('cruise', f"{w}#{k}", data)
                if err and res is None:
                    print(f"  [ERROR] cruise {w}#{k}:\n{err}")
                snap['cruise'][w].append(res)
        print('\n'.join(cruise_table(snap)))
    with open(path, 'w') as fh:
        json.dump(snap, fh)
    print(f"Snapshot: {path} (code {snap['meta']['code']})")
    return snap


def _first(items, n=5):
    return items[:n] + ([f"... and {len(items) - n} more"] if len(items) > n else [])


def compare(a, b):
    """Differences between two snapshots, as lines; [] = same behaviour."""
    out = []
    for part in PARTS:
        if part not in a or part not in b:
            continue
        for key in sorted(set(a[part]) | set(b[part])):
            x, y = a[part].get(key), b[part].get(key)
            if x is None or y is None:
                if (x is None) != (y is None):
                    out.append(f"{part} {key}: present in only one snapshot")
                continue
            if part == 'cruise':
                out += compare_cruise(key, x, y)
                continue
            if part == 'perception':
                if len(x) != len(y):
                    out.append(f"perception {key}: {len(x)} scans against {len(y)}")
                    continue
                diffs = [f"{s['scan']}: {', '.join(k for k in s['layers'] if s['layers'][k] != t['layers'][k])} "
                         f"(obstacle cells {s['obstacle_cells']} -> {t['obstacle_cells']}, "
                         f"valid {s['valid_cells']} -> {t['valid_cells']})"
                         for s, t in zip(x, y) if s['layers'] != t['layers']]
                if diffs:
                    out.append(f"perception {key}: {len(diffs)} of {len(x)} scans differ")
                    out += ['    ' + d for d in _first(diffs)]
            elif part == 'replay':
                g = [i for i, (s, t) in enumerate(zip(x['graphs'], y['graphs'])) if s != t]
                if g or len(x['graphs']) != len(y['graphs']):
                    i = g[0] if g else min(len(x['graphs']), len(y['graphs']))
                    out.append(f"replay {key}: graph differs from update {i + 1} of {len(x['graphs'])} "
                               f"({len(g)} updates differ; edges {x['graphs'][i]['edges'] if i < len(x['graphs']) else '-'} -> "
                               f"{y['graphs'][i]['edges'] if i < len(y['graphs']) else '-'})")
                p = [i for i, (s, t) in enumerate(zip(x['paths'], y['paths'])) if s != t]
                if p or len(x['paths']) != len(y['paths']):
                    out.append(f"replay {key}: {len(p)} of {len(x['paths'])} paths differ"
                               + (f", first: {x['paths'][p[0]]} -> {y['paths'][p[0]]}" if p else ''))
                if x['summary'] != y['summary']:
                    ks = [k for k in set(x['summary']) | set(y['summary']) if x['summary'].get(k) != y['summary'].get(k)]
                    out.append(f"replay {key}: summary differs in {', '.join(sorted(ks))}")
            else:
                xl, yl = normalise('\n'.join(x['lines'])), normalise('\n'.join(y['lines']))   # rules of today
                if xl != yl or x['ok'] != y['ok']:
                    d = [l for l in difflib.unified_diff(xl, yl, 'before', 'after', n=1, lineterm='')]
                    changed = sum(1 for l in d if l[:1] in '+-' and not l.startswith(('+++', '---')))
                    out.append(f"missions world {key}: {changed} log lines differ")
                    out += ['    ' + l[:160] for l in d[2:30]]
    return out


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '_worker':
        ap = argparse.ArgumentParser()
        ap.add_argument('cmd'); ap.add_argument('part'); ap.add_argument('item'); ap.add_argument('--data')
        a = ap.parse_args()
        worker_main(a.part, a.item, a.data)
        return 0
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__.split('\n\n', 2)[2])
    ap.add_argument('cmd', choices=['record', 'check', 'compare'])
    ap.add_argument('files', nargs='+', help='record: output; check: the snapshot to check against; compare: two snapshots')
    ap.add_argument('--parts', nargs='+', choices=PARTS, default=None,
                    help='default: perception replay missions; cruise (slow, ~%d min per world) only if named' % (REPEAT * 2))
    ap.add_argument('--repeat', type=int, default=REPEAT, help='cruise runs per world')
    ap.add_argument('--with', dest='variants', default='',
                    help='cruise part: variants of the local varianti/ folder to turn on (e.g. frontier)')
    ap.add_argument('--variants-dir', default=None, help='the varianti/ folder (default: next to tools/)')
    ap.add_argument('--missions', nargs='+', default=MISSIONS, help='saved missions for perception and replay')
    ap.add_argument('--worlds', nargs='+', default=WORLDS, help='fake mission worlds: A, B, rand<seed>')
    ap.add_argument('--data', default=REPO, help='folder with MissionMap/, MissionLogs/, graph/')
    ap.add_argument('-j', '--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    data = os.path.abspath(a.data)
    if a.variants:
        os.environ['SNAPSHOT_VARIANTS'] = a.variants
        if a.variants_dir:
            os.environ['SNAPSHOT_VARIANTS_DIR'] = os.path.abspath(a.variants_dir)
    if a.cmd in ('record', 'check') and not os.path.isdir(os.path.join(data, 'MissionMap')) \
            and not set(a.parts or DEFAULT_PARTS) <= {'missions', 'cruise'}:
        sys.exit(f"No MissionMap/ in {data}: give the folder of the mission data with --data")
    if a.cmd == 'record':
        record(a.files[0], a.parts or list(DEFAULT_PARTS), a.missions, a.worlds, data, a.jobs, a.repeat)
        return 0
    if a.cmd == 'compare':
        old, new = (json.load(open(f)) for f in a.files[:2])
    else:
        old = json.load(open(a.files[0]))
        parts = [p for p in old['meta']['parts'] if a.parts is None or p in a.parts]
        missions = sorted({m for p in ('perception', 'replay') if p in old for m in old[p]}) or a.missions
        tmp = os.path.join(tempfile.gettempdir(), 'behaviour_snapshot_now.json')
        worlds = sorted(old.get('missions', {}) or old.get('cruise', {})) or a.worlds
        new = record(tmp, parts, missions, worlds, data, a.jobs, a.repeat)
    print(f"\nBefore: {old['meta']['date']}, code {old['meta']['code']}")
    print(f"After:  {new['meta']['date']}, code {new['meta']['code']}\n")
    if 'cruise' in old and 'cruise' in new:
        print("Cruise, before:\n" + '\n'.join(cruise_table(old)) + "\nCruise, after:\n" + '\n'.join(cruise_table(new)) + "\n")
    diffs = compare(old, new)
    if not diffs:
        print("SAME BEHAVIOUR: no difference in any part.")
        return 0
    print("DIFFERENCES:")
    print('\n'.join(diffs))
    return 1


if __name__ == '__main__':
    sys.exit(main())
