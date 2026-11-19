#!/usr/bin/env python3
"""
Where the planning cycle of the cruise spends its time, function by function (no robot): the
scenarios of tests/test_cruise.py (fake Spot, 128 x 128 grid of 3 cm, the real planning
thread) with cProfile around every CruisePlanner cycle. Prints the functions of the project
with most time of their own and most cumulative time, over all the cycles.

    python3 tools/profile_cycle.py                        # scenarios open and three
    python3 tools/profile_cycle.py --scenarios wall --top 40
    python3 tools/profile_cycle.py --dump cycle.prof      # for snakeviz / pstats

The download of the grid is fake here (see bench_cruise.py): what this shows is the
computation, the part that makes the cycle long on the CORE I/O (maps + edges ~0.27 s on
09-11 against ~0.12 s of scan).
"""
import argparse
import contextlib
import cProfile
import io
import os
import pstats
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), os.path.join(os.path.dirname(HERE), 'tests')]

import test_cruise as tc                                     # noqa: E402
from spot_explorer.mission import planning_cycle             # noqa: E402

SCENARIOS = {
    'open': ("open field", [], (2.0, 2.0, 0.785), [(0, 1)]),
    'wall': ("wall with a gap", [(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), [(0, 1)]),
    'three': ("three cells", [(1.0, 5.5, 5.9, 6.1)], (2.0, 2.0, 0.785), [(0, 1), (1, 0), (1, 1)]),
}
PROF = cProfile.Profile()
N = [0, 0.0]
_call = planning_cycle.CruisePlanner.__call__


def _profiled(self):
    t = time.perf_counter()
    PROF.enable()
    try:
        return _call(self)
    finally:
        PROF.disable()
        N[0] += 1
        N[1] += time.perf_counter() - t


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--scenarios', nargs='+', choices=list(SCENARIOS), default=['open', 'three'])
    ap.add_argument('--top', type=int, default=25)
    ap.add_argument('--dump', default=None, help='write the raw profile here')
    a = ap.parse_args()
    planning_cycle.CruisePlanner.__call__ = _profiled
    for name in a.scenarios:
        with contextlib.redirect_stdout(io.StringIO()):
            tc.run(*SCENARIOS[name])
        print(f"scenario {name}: done ({N[0]} cycles so far)")
    print(f"\n{N[0]} cycles, {1000 * N[1] / max(N[0], 1):.0f} ms per cycle on average (with the profiler: "
          f"slower than without, the shares are what counts)\n")
    st = pstats.Stats(PROF)
    if a.dump:
        st.dump_stats(a.dump)
    for key, title in (('tottime', 'OWN TIME (the function itself)'), ('cumulative', 'CUMULATIVE (with what it calls)')):
        rows = []
        for (f, line, fn), (cc, nc, tt, ct, _) in st.stats.items():
            if 'spot_explorer' not in f and key == 'cumulative':
                continue
            rows.append((tt if key == 'tottime' else ct, nc, f, line, fn))
        rows.sort(reverse=True)
        print(f"== {title}, ms per cycle ==")
        for t, nc, f, line, fn in rows[:a.top]:
            where = f.split('AutonomousNavigationSpot/')[-1] if 'spot_explorer' in f else os.path.basename(f)
            print(f"  {1000 * t / max(N[0], 1):7.1f}  {nc / max(N[0], 1):8.1f} calls  {fn}  ({where}:{line})")
        print()


if __name__ == '__main__':
    main()
