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
PARTS = ('perception', 'replay', 'missions')
MARK = '@@SNAPSHOT@@'


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


WORKERS = dict(perception=work_perception, replay=work_replay, missions=work_missions)


def worker_main(part, item, data):
    real = sys.stdout
    with contextlib.redirect_stdout(io.StringIO()):
        res = WORKERS[part](item, data)
    real.write(MARK + json.dumps(res) + '\n')


# --------------------------------------------------------------------------------------
# Record / compare
# --------------------------------------------------------------------------------------
def _run_worker(part, item, data):
    p = subprocess.run([sys.executable, os.path.abspath(__file__), '_worker', part, item, '--data', data],
                       capture_output=True, text=True, cwd=REPO)
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


def record(path, parts, missions, worlds, data, jobs):
    jobs_list = [(p, m) for p in parts if p != 'missions' for m in missions] + \
                ([('missions', w) for w in worlds] if 'missions' in parts else [])
    snap = dict(meta=dict(date=datetime.datetime.now().isoformat(timespec='seconds'), code=_git_state(),
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
    ap.add_argument('--parts', nargs='+', choices=PARTS, default=list(PARTS))
    ap.add_argument('--missions', nargs='+', default=MISSIONS, help='saved missions for perception and replay')
    ap.add_argument('--worlds', nargs='+', default=WORLDS, help='fake mission worlds: A, B, rand<seed>')
    ap.add_argument('--data', default=REPO, help='folder with MissionMap/, MissionLogs/, graph/')
    ap.add_argument('-j', '--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    data = os.path.abspath(a.data)
    if a.cmd in ('record', 'check') and not os.path.isdir(os.path.join(data, 'MissionMap')) and a.parts != ['missions']:
        sys.exit(f"No MissionMap/ in {data}: give the folder of the mission data with --data")
    if a.cmd == 'record':
        record(a.files[0], a.parts, a.missions, a.worlds, data, a.jobs)
        return 0
    if a.cmd == 'compare':
        old, new = (json.load(open(f)) for f in a.files[:2])
    else:
        old = json.load(open(a.files[0]))
        parts = [p for p in old['meta']['parts'] if p in a.parts]
        missions = sorted({m for p in ('perception', 'replay') if p in old for m in old[p]}) or a.missions
        worlds = sorted(old.get('missions', {})) or a.worlds
        tmp = os.path.join(tempfile.gettempdir(), 'behaviour_snapshot_now.json')
        new = record(tmp, parts, missions, worlds, data, a.jobs)
    print(f"\nBefore: {old['meta']['date']}, code {old['meta']['code']}")
    print(f"After:  {new['meta']['date']}, code {new['meta']['code']}\n")
    diffs = compare(old, new)
    if not diffs:
        print("SAME BEHAVIOUR: no difference in any part.")
        return 0
    print("DIFFERENCES:")
    print('\n'.join(diffs))
    return 1


if __name__ == '__main__':
    sys.exit(main())
