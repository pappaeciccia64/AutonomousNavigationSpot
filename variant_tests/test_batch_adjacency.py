#!/usr/bin/env python3
"""
The batch_adjacency variant without the robot: in the scenarios of tests/test_cruise.py,
at every edge refresh of the planning cycle the graph is copied and refreshed edge by edge
(variant off) while the real one is refreshed in batch (variant on): neighbour lists (order
included), validity, slopes and the counts of the refresh must be identical.

    python3 varianti/variant_tests/test_batch_adjacency.py
"""
import contextlib
import copy
import io
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]

import test_cruise as tc                                       # noqa: E402
from spot_explorer.planning import edge_evaluation as ee       # noqa: E402
import batch_adjacency                                         # noqa: E402

N = dict(refreshes=0, diffs=0, t_old=0.0, t_new=0.0)
_orig = ee.EdgeEvaluation.refresh_local_edge_weights


def _both(self, *a, **k):
    other = copy.deepcopy(self)
    t = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):
        r_old = _orig(other, *a, **k)
    N['t_old'] += time.perf_counter() - t
    off = batch_adjacency.apply()
    t = time.perf_counter()
    r_new = _orig(self, *a, **k)
    N['t_new'] += time.perf_counter() - t
    off()
    N['refreshes'] += 1
    same = (r_old == r_new and dict(other.edges) == dict(self.edges)
            and other.edge_validity == self.edge_validity and other.edge_slope_info == self.edge_slope_info
            and getattr(other, 'refresh_counts_last', None) == getattr(self, 'refresh_counts_last', None))
    if not same:
        N['diffs'] += 1
    return r_new


def main():
    ee.EdgeEvaluation.refresh_local_edge_weights = _both
    for name, args in (("open field", ([], (2.0, 2.0, 0.785), [(0, 1)])),
                       ("wall with a gap", ([(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), [(0, 1)])),
                       ("three cells", ([(1.0, 5.5, 5.9, 6.1)], (2.0, 2.0, 0.785), [(0, 1), (1, 0), (1, 1)]))):
        with contextlib.redirect_stdout(io.StringIO()):
            tc.run(name, *args)
        print(f"  {name}: {N['refreshes']} refreshes so far, {N['diffs']} different")
    ok = N['refreshes'] > 20 and N['diffs'] == 0
    print(f"  [{'OK ' if ok else 'FAIL'}] {N['refreshes']} refreshes compared, {N['diffs']} with a difference")
    print(f"  refresh time: edge by edge {1000 * N['t_old'] / max(N['refreshes'], 1):.0f} ms, "
          f"in batch {1000 * N['t_new'] / max(N['refreshes'], 1):.0f} ms per refresh")
    print(f"\n{'ALL OK' if ok else 'FAILED'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
