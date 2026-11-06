#!/usr/bin/env python3
"""
Bench test of the planning cycle of the CRUISE against the node density of the graph (no
robot): the scenarios of tests/test_cruise.py (fake Spot in real time, 128 x 128 grid of 3 cm
as Spot's, the real planning thread), with random Poisson nodes at several densities instead
of the test lattice. For every density it prints, over the cycles of all the scenarios, the
median and the 90th percentile of the cycle and of each phase (planning_cycle.PHASES), the
edges recomputed per cycle and the cycles per second.

    python3 tools/bench_cruise.py                          # densities 4 5 9 16, ~2 min each
    python3 tools/bench_cruise.py --densities 5 9 25 --scenarios open wall

What it cannot measure: the download of the scan from Spot ('scan' here is the fake grid,
on the robot it was 0.09 s median, 0.17 s p90 on 03-11) and how often Spot updates its grid
(both in the [CRUISE] lines of a real mission). The rest (maps, edges, path, frontier) is
the same computation on this PC.
"""
import argparse
import contextlib
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), os.path.join(os.path.dirname(HERE), 'tests')]

import test_cruise as tc                                     # noqa: E402
from spot_explorer.mission import planning_cycle             # noqa: E402
from spot_explorer.planning import mission_graph             # noqa: E402

SCENARIOS = {
    'open': ("open field", [], (2.0, 2.0, 0.785), [(0, 1)]),
    'wall': ("wall with a gap", [(5.0, 5.2, 0.0, 5.0), (5.0, 5.2, 6.2, 12.0)], (2.0, 2.0, 0.785), [(0, 1)]),
    'three': ("three cells", [(1.0, 5.5, 5.9, 6.1)], (2.0, 2.0, 0.785), [(0, 1), (1, 0), (1, 1)]),
}
_build = mission_graph.build_mission_graph
_call = planning_cycle.CruisePlanner.__call__
CYCLES = []         # (cycle_s, phases, edges recomputed) of every planning cycle
NODES = []          # nodes of the graph of every scenario


def _timed_call(self):
    plan = _call(self)
    if plan.get('phases'):
        CYCLES.append((plan['cycle_s'], plan['phases'], (plan.get('edges') or [0])[0]))
    return plan


def _graph_at(density, seed):
    def build(**kw):
        kw.update(sampling='random', distribution='poisson', density_per_m2=density, seed=seed)
        g = _build(**kw)
        NODES.append(len(g.xy_local))
        return g
    return build


def bench(density, names, seed):
    mission_graph.build_mission_graph = _graph_at(density, seed)
    planning_cycle.CruisePlanner.__call__ = _timed_call
    CYCLES.clear()
    NODES.clear()
    secs, n_cycles = 0.0, 0
    for key in names:
        name, rects, start, cells = SCENARIOS[key]
        with contextlib.redirect_stdout(io.StringIO()):
            r = tc.run(f"{name}, {density:g} nodes/m2", rects, start, cells)
        secs += r['stats'].get('cruise_s', 0.0)
        n_cycles += r['stats'].get('cruise_cycles', 0)
        if r['err'] or r['oks'].count(True) != len(cells):
            print(f"  {density:g} nodes/m2, {name}: cells {r['oks']}" + (" EXCEPTION" if r['err'] else ""))
    return dict(cyc=np.array([c[0] for c in CYCLES]),
                ph={k: np.array([c[1][k] for c in CYCLES if k in c[1]]) for k in planning_cycle.PHASES},
                rec=np.array([c[2] for c in CYCLES]), rate=n_cycles / max(secs, 1e-6),
                nodes=int(np.mean(NODES)) if NODES else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--densities', type=float, nargs='+', default=[4.0, 5.0, 9.0, 16.0])
    ap.add_argument('--scenarios', nargs='+', choices=list(SCENARIOS), default=list(SCENARIOS))
    ap.add_argument('--seed', type=int, default=3)
    a = ap.parse_args()
    print(f"Planning period PLAN_PERIOD_S = {planning_cycle.PLAN_PERIOD_S} s. Values in seconds, median / p90.")
    head = f"{'nodes/m2':>8s} {'nodes':>6s} {'cycles':>6s} {'cycle':>11s} " + " ".join(
        f"{k:>11s}" for k in planning_cycle.PHASES) + f" {'edges rec.':>10s} {'cycles/s':>8s}"
    print(head)
    rows = []
    for d in a.densities:
        r = bench(d, a.scenarios, a.seed)
        if r['cyc'].size == 0:
            rows.append(f"{d:8g}  no cruise cycle")
            continue
        f = lambda x: f"{np.median(x):.3f}/{np.percentile(x, 90):.3f}" if x.size else "-"
        rows.append(f"{d:8g} {r['nodes']:6d} {r['cyc'].size:6d} {f(r['cyc']):>11s} "
                    + " ".join(f"{f(r['ph'][k]):>11s}" for k in planning_cycle.PHASES)
                    + f" {int(np.median(r['rec'])):>4d}/{int(np.percentile(r['rec'], 90)):<5d} {r['rate']:8.2f}")
        print(rows[-1], flush=True)


if __name__ == '__main__':
    main()
