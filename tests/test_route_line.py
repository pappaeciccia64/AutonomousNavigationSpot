#!/usr/bin/env python3
"""
roadmap.ROUTE_LINE_PENALTY without the robot, on small lattices:
  - switch off: the paths of today, whatever the route line;
  - on: between two ways of equal length the one on the route line;
  - an obstacle on the line: the path goes round it (the penalty is saturated);
  - the searches of the cell rules and of reachability (costs_from without route) do not
    change; the stability check (compute_path_cost) uses the same costs as the path.

    python3 tests/test_route_line.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np                                     # noqa: E402
from spot_explorer.planning import roadmap             # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def lattice(nx, ny, step=0.5, holes=()):
    """4-connected lattice, weight = length; holes: nodes removed."""
    prm = roadmap.PRM()
    ids = {}
    for i in range(nx):
        for j in range(ny):
            if (i, j) in holes:
                continue
            ids[(i, j)] = len(ids)
            prm.add_node(ids[(i, j)], i * step, j * step)
    for (i, j), a in ids.items():
        for di, dj in ((1, 0), (0, 1)):
            b = ids.get((i + di, j + dj))
            if b is not None:
                prm.edges[a].append((b, step))
                prm.edges[b].append((a, step))
    return prm, ids


def ys(prm, path):
    return [prm.nodes[n][1] for n in path]


def main():
    print("== a 9 x 5 lattice, from (0,0) to (4,2) m: many ways of equal length ==")
    prm, ids = lattice(9, 5)
    s, g = ids[(0, 0)], ids[(8, 4)]
    L = lambda p: prm.path_length(p)
    pen = lambda p: sum(min(prm.route_distance(n), roadmap.ROUTE_LINE_SATURATION_M) for n in p[1:])
    better = 0
    for y_line in (0.0, 1.0, 2.0):
        prm.set_route_line([(0.0, y_line), (4.0, y_line)])
        roadmap.ROUTE_LINE_PENALTY = False
        p_off = prm._dijkstra(s, g)
        roadmap.ROUTE_LINE_PENALTY = True
        p_on = prm._dijkstra(s, g)
        roadmap.ROUTE_LINE_PENALTY = False
        check(abs(L(p_off) - L(p_on)) < 1e-9 and pen(p_on) <= pen(p_off) + 1e-9,
              f"line at y = {y_line} m: same length, distance from the line {pen(p_on):.1f} on / {pen(p_off):.1f} off")
        better += pen(p_on) < pen(p_off) - 1e-9
    check(better >= 1, f"closer to the line than today in {better} of 3")

    print("== an obstacle on the line: the path goes round it ==")
    holes = {(i, 4) for i in range(3, 6)} | {(i, 3) for i in range(3, 6)}
    prm, ids = lattice(9, 5, holes=holes)
    prm.set_route_line([(0.0, 2.0), (4.0, 2.0)])
    roadmap.ROUTE_LINE_PENALTY = True
    p = prm._dijkstra(ids[(0, 4)], ids[(8, 4)])
    roadmap.ROUTE_LINE_PENALTY = False
    check(p is not None and all((n not in [ids.get(h) for h in holes]) for n in p), "a path round the obstacle")
    check(p is not None and ys(prm, p)[-1] == 2.0, "back on the line after it")

    print("== the other searches and the stability check ==")
    prm, ids = lattice(9, 5)
    prm.set_route_line([(0.0, 2.0), (4.0, 2.0)])
    roadmap.ROUTE_LINE_PENALTY = False
    d_off, _, _ = prm.costs_from(ids[(0, 0)])
    roadmap.ROUTE_LINE_PENALTY = True
    d_on, _, _ = prm.costs_from(ids[(0, 0)])
    path = prm._dijkstra(ids[(0, 0)], ids[(8, 0)])
    cost = prm.compute_path_cost(path)
    exp = sum(prm.effective_weight(a, b, 0.5, route=True) for a, b in zip(path, path[1:]))
    roadmap.ROUTE_LINE_PENALTY = False
    check(d_off == d_on, "costs_from without route: the same costs (cell rules, reachability)")
    check(abs(cost - exp) < 1e-9 and cost > prm.path_length(path), f"compute_path_cost with the route penalty ({cost:.2f})")
    check(np.isinf(roadmap.PRM().route_distance(0)), "no line: infinite distance, no penalty applied")

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
