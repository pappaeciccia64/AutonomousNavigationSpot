#!/usr/bin/env python3
"""
Ways of choosing the goal and the path on fake missions (no robot): the whole mission of
tests/test_mission.py, in cruise (the real-time fake Spot of test_cruise), run in each world
once per variant (--variants: cells, cells + roadmap.ROUTE_LINE_PENALTY, goal from the
frontier, mission/frontier.py) and compared:

  cells     visited / blocked (from the log)
  walked    metres of the robot track
  off line  how far the track is from the route of the visit (polyline through the cell
            centres in visiting order): mean and 90th percentile
  seen      area of the mission seen at the end ([PROGRESS] ... area seen X%)
  time      seconds of the fake mission (the cruise runs in real time)

    python3 tools/compare_route_line.py                       # worlds A, B and 3 random
    python3 tools/compare_route_line.py --random 6 --weight 1.0 --saturation 1.0
    python3 tools/compare_route_line.py --worlds A --random 0 --variants cells route frontier

Worlds A and B: 3 x 3 cells of 3 m (tests/test_mission.py). Random: 4 x 5 cells of 2 m (as
the mission), 6-9 boxes of 0.4-1.2 m away from the start.
"""
import argparse
import contextlib
import io
import os
import re
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [ROOT, os.path.join(ROOT, 'tests')]

import test_mission as M                                      # noqa: E402
import test_e2e_loop as T                                     # noqa: E402
from spot_explorer import config as P                         # noqa: E402
from spot_explorer.mission import cells as cells_mod          # noqa: E402
from spot_explorer.mission import exploration                 # noqa: E402
from spot_explorer.planning import roadmap                    # noqa: E402

VARIANTS = {'cells': "the cells of today", 'route': "cells + route penalty (roadmap.ROUTE_LINE_PENALTY)",
            'frontier': "goal from the frontier (exploration.GOAL_SOURCE = 'frontier', with the route penalty)"}


def random_world(seed, rows, cols, cell):
    rng = np.random.default_rng(seed)
    w, h = cols * cell, rows * cell
    boxes = []
    while len(boxes) < rng.integers(6, 10):
        sx, sy = rng.uniform(0.4, 1.2, 2)
        x0, y0 = rng.uniform(0, w - sx), rng.uniform(0, h - sy)
        if x0 < 2.2 and y0 < 2.2:                      # keep the start free
            continue
        boxes.append((float(x0), float(x0 + sx), float(y0), float(y0 + sy)))
    return boxes


def route_line(rows, cols, cell, start):
    env = cells_mod.EnvironmentMap(rows=rows, cols=cols, cell_size=cell)
    with contextlib.redirect_stdout(io.StringIO()):
        env.set_origin(start[0], start[1], start[2])
    path = env.generate_visit_path(P.MISSION_VISIT_ORDER, start_cell=(0, 0))
    return np.array([env.get_world_position_from_cell(r, c) for r, c in path], dtype=float)


def dist_to_line(pts, line):
    a, b = line[:-1], line[1:]
    ab = b - a
    out = []
    for p in pts:
        t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0, 1)
        out.append(np.min(np.hypot(*(a + t[:, None] * ab - p).T)))
    return np.array(out)


def one(walls, rows, cols, cell, start, variant):
    M.ROWS, M.COLS, M.CELL, M.START = rows, cols, cell, start
    roadmap.ROUTE_LINE_PENALTY = variant == 'route'
    exploration.GOAL_SOURCE = 'frontier' if variant == 'frontier' else 'cells'
    t0 = time.time()
    with contextlib.redirect_stdout(io.StringIO()):
        r = M.run(walls=walls, name='route', cruise=True)
    roadmap.ROUTE_LINE_PENALTY = False
    exploration.GOAL_SOURCE = 'cells'
    log = r['log']
    track = np.array(T.ROBOT.track, dtype=float).reshape(-1, 2)
    walked = float(np.sum(np.hypot(*np.diff(track, axis=0).T))) if len(track) > 1 else 0.0
    d = dist_to_line(track[::3], route_line(rows, cols, cell, start)) if len(track) else np.zeros(1)
    seen = re.findall(r'area seen (\d+)%', log)
    return dict(log=log, goals=len(re.findall(r'\[FRONTIER\] Goal:', log)),
                visited=len(set(re.findall(r'\[VISITED\] Cell \((\d+,\d+)\)', log))),
                blocked=len(set(re.findall(r'\[BLOCKED\] Cell \((\d+,\d+)\)', log))),
                walked=walked, off_mean=float(d.mean()), off_p90=float(np.percentile(d, 90)),
                seen=int(seen[-1]) if seen else -1, secs=time.time() - t0,
                traceback='Traceback' in log)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--worlds', nargs='*', default=['A', 'B'], help='worlds of test_mission (A, B)')
    ap.add_argument('--random', type=int, default=3, help='random worlds of 4 x 5 cells of 2 m')
    ap.add_argument('--weight', type=float, default=roadmap.ROUTE_LINE_WEIGHT)
    ap.add_argument('--saturation', type=float, default=roadmap.ROUTE_LINE_SATURATION_M)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--variants', nargs='+', choices=list(VARIANTS), default=['cells', 'route'])
    ap.add_argument('--keep-logs', default=None, help='folder where to save the mission logs')
    ap.add_argument('--frontier-min-dist', type=float, default=None,
                    help='frontier.FRONTIER_MIN_DIST_M (no goal nearer to the robot than this)')
    a = ap.parse_args()
    if a.frontier_min_dist is not None:
        from spot_explorer.mission import frontier
        frontier.FRONTIER_MIN_DIST_M = a.frontier_min_dist
    roadmap.ROUTE_LINE_WEIGHT, roadmap.ROUTE_LINE_SATURATION_M = a.weight, a.saturation
    cases = [(f"world {w}", M.WORLDS[w], 3, 3, 3.0, (1.5, 1.5, 0.0)) for w in a.worlds]
    cases += [(f"random {a.seed + k}", random_world(a.seed + k, 4, 5, 2.0), 4, 5, 2.0, (1.0, 1.0, 0.0))
              for k in range(a.random)]
    print(f"route penalty: weight {a.weight}, saturation {a.saturation} m\n")
    for v in a.variants:
        print(f"  {v:9s} {VARIANTS[v]}")
    print(f"\n{'world':12s} {'variant':9s} {'cells':>8s} {'walked':>8s} {'off line mean/p90':>18s} {'seen':>5s} {'time':>6s}")
    tot = {v: [] for v in a.variants}
    for name, walls, rows, cols, cell, start in cases:
        for v in a.variants:
            r = one(walls, rows, cols, cell, start, v)
            tot[v].append(r)
            if a.keep_logs:
                os.makedirs(a.keep_logs, exist_ok=True)
                with open(os.path.join(a.keep_logs, f"{name.replace(' ', '_')}_{v}.log"), 'w') as fh:
                    fh.write(r['log'])
            print(f"{name:12s} {v:9s} {r['visited']:>4d}/{r['blocked']:<3d} {r['walked']:7.1f}m "
                  f"{r['off_mean']:8.2f} / {r['off_p90']:.2f} m {r['seen']:4d}% {r['secs']:5.0f}s"
                  + (f"  ({r['goals']} frontier goals)" if v == 'frontier' else "")
                  + ("  TRACEBACK" if r['traceback'] else ""), flush=True)
    print()
    for v in a.variants:
        t = tot[v]
        print(f"total {v:9s}: walked {sum(r['walked'] for r in t):.0f} m, off line mean "
              f"{np.mean([r['off_mean'] for r in t]):.2f} m, seen {np.mean([r['seen'] for r in t]):.0f}%, "
              f"time {sum(r['secs'] for r in t):.0f} s, tracebacks {sum(r['traceback'] for r in t)}")


if __name__ == '__main__':
    main()
