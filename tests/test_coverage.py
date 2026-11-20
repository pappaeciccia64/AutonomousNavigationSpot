#!/usr/bin/env python3
"""
The coverage of the mission area (exploration.Mission._seen_area_pct, _log_coverage) without
the robot.

    python3 tests/test_coverage.py

  1. the area seen counted on the raster (known_many) equals the old count point by point on
     the keys of the global map, on maps filled at random;
  2. a [COVERAGE] line every COVERAGE_STEP of growth and one at the end, coverage.csv written;
  3. the whole fake mission of tests/test_mission.py (world A): [COVERAGE] lines with the area
     growing, coverage.csv and the coverage in the progress of the summary, and the
     [PROGRESS] lines as before (one per closed cell).
"""
import contextlib
import csv
import io
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), HERE]

import numpy as np                                                 # noqa: E402
from spot_explorer.mission import cells, exploration               # noqa: E402
from spot_explorer.perception import global_maps                   # noqa: E402
from spot_explorer.records import MISSION_CTX                      # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def bare_mission(gm, rows=4, cols=5, cell=2.0):
    env = cells.EnvironmentMap(rows=rows, cols=cols, cell_size=cell)
    with contextlib.redirect_stdout(io.StringIO()):
        env.set_origin(1.0, 1.0, 0.3)
    m = exploration.Mission.__new__(exploration.Mission)
    m.env, m.global_grid = env, type('G', (), {'global_occupancy_map': gm})()
    m._area_points, m._coverage = None, []
    return m


def old_count(m, gm):
    keys = np.floor(m._area_points / gm.resolution).astype(int)
    return sum(1 for gx, gy in keys if (gx, gy) in gm.grid) / len(keys)


def unit():
    print("== the area seen: raster against the old count ==")
    rng = np.random.default_rng(3)
    same = True
    for k in range(5):
        gm = global_maps.GlobalGrid(0.03)
        for _ in range(int(rng.integers(2000, 40000))):
            gm._set_state((int(rng.integers(-50, 400)), int(rng.integers(-50, 300))),
                          -1.0 if rng.random() < 0.2 else 1.0)
        m = bare_mission(gm)
        new = m._seen_area_pct()
        same &= abs(new - old_count(m, gm)) < 1e-12
    check(same, "same fraction on 5 random maps")
    print("== [COVERAGE] lines and coverage.csv ==")
    folder = tempfile.mkdtemp(prefix="coverage_")
    MISSION_CTX['log_folder'] = folder
    m = bare_mission(global_maps.GlobalGrid(0.03))
    m.env.map = [[0] * m.env.cols for _ in range(m.env.rows)]
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        for i, a in enumerate((0.10, 0.12, 0.16, 0.30, 0.31, 0.50)):
            m._log_coverage(a, float(i))
        m._log_coverage(0.52, 7.0, final=True)
    lines = [l for l in out.getvalue().splitlines() if l.startswith('[COVERAGE]')]
    seen = [int(v) for v in re.findall(r'area seen (\d+)%', out.getvalue())]
    check(seen == [10, 16, 30, 50, 52], f"a line every 5 points and the end ({seen})")
    check(lines[-1].startswith('[COVERAGE] End: area seen 52%'), "the final line")
    check('min to 95%' in lines[1], f"time to 95% estimated ({lines[1]})")
    rows = list(csv.DictReader(open(os.path.join(folder, 'coverage.csv'))))
    check(len(rows) == 5 and float(rows[-1]['seen_pct']) == 52.0, f"coverage.csv: {len(rows)} rows")
    MISSION_CTX.pop('log_folder', None)


def mission():
    print("== the whole fake mission, world A ==")
    import test_mission as M
    with contextlib.redirect_stdout(io.StringIO()):
        r = M.run(walls=M.WORLDS['A'], name='coverage')
    log = r['log']
    seen = [int(v) for v in re.findall(r'\[COVERAGE\] (?:End: )?area seen (\d+)%', log)]
    check(len(seen) >= 4 and seen == sorted(seen), f"[COVERAGE] lines, area growing ({seen})")
    check(re.search(r'\[COVERAGE\] End: area seen \d+%', log) is not None, "the final [COVERAGE] line")
    csvs = [os.path.join(dp, f) for dp, _, fs in os.walk(r['root']) for f in fs if f == 'coverage.csv']
    check(bool(csvs) and len(list(csv.DictReader(open(csvs[0])))) == len(seen), "coverage.csv, one row per line")
    lines = [l for l in log.splitlines() if '[PROGRESS]' in l and 'End' not in l]
    closed = [int(l.split('[PROGRESS] ')[1].split('/')[0]) for l in lines]
    check(closed == sorted(set(closed)), f"[PROGRESS] as before: one line per additional closed cell ({closed})")
    summ = [os.path.join(dp, f) for dp, _, fs in os.walk(r['root']) for f in fs if f == 'mission_summary.json']
    if summ:
        import json
        pr = json.load(open(summ[0])).get('mission_stats', {}).get('progress', {})
        check(len(pr.get('coverage', [])) == len(seen), "the coverage in the progress of the summary")


def main():
    unit()
    mission()
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
