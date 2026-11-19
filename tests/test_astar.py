#!/usr/bin/env python3
"""
roadmap.A_STAR_PATH without the robot: on random graphs like the mission's (Poisson nodes,
edges 0.5-1.25 m, weight = length + slope terms, some penalties, some impassable edges and
nodes, planning limited to some nodes) the path of A* costs exactly as Dijkstra's, for many
start / goal pairs, and the search is faster.

    python3 tests/test_astar.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np                                     # noqa: E402
from spot_explorer.planning import roadmap             # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def graph(seed, side=16.0, density=9.0):
    rng = np.random.default_rng(seed)
    prm = roadmap.PRM()
    n = rng.poisson(density * side * side)
    xy = rng.uniform(0, side, (n, 2))
    for i, (x, y) in enumerate(xy):
        prm.add_node(i, float(x), float(y))
    cell = 1.25
    buckets = {}
    for i, (x, y) in enumerate(xy):
        buckets.setdefault((int(x / cell), int(y / cell)), []).append(i)
    for i, (x, y) in enumerate(xy):
        cx, cy = int(x / cell), int(y / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in buckets.get((cx + dx, cy + dy), ()):
                    if j <= i:
                        continue
                    L = float(np.hypot(*(xy[j] - xy[i])))
                    if not 0.5 <= L <= 1.25:
                        continue
                    w = L + 2 * rng.exponential(0.1) + 2 * rng.exponential(0.1)   # length + slopes
                    if rng.random() < 0.03:
                        w = float('inf')                                      # vetoed
                    prm.edges[i].append((j, w))
                    prm.edges[j].append((i, w))
    for _ in range(n // 50):                                                  # cost layer
        a = int(rng.integers(n))
        if prm.edges[a]:
            b = prm.edges[a][0][0]
            prm.edge_penalty_persistent[(min(a, b), max(a, b))] = float(rng.exponential(1.0))
    for _ in range(n // 200):
        prm.node_penalty_transient[int(rng.integers(n))] = float('inf')
    return prm, rng


def cost(prm, path):
    return sum(prm.effective_weight(a, b, dict(prm.edges[a])[b]) for a, b in zip(path, path[1:]))


def main():
    t_d = t_a = 0.0
    n_q = same_cost = same_path = both_none = 0
    for seed in range(6):
        prm, rng = graph(seed)
        ids = list(prm.nodes)
        if seed % 2:                                     # planning limited to some nodes
            keep = set(i for i in ids if prm.nodes[i][0] < 11.0)
            prm.set_allowed_nodes(keep)
        for _ in range(40):
            s, g = (int(v) for v in rng.choice(ids, 2, replace=False))
            roadmap.A_STAR_PATH = False
            t = time.perf_counter(); pd = prm._dijkstra(s, g); t_d += time.perf_counter() - t
            roadmap.A_STAR_PATH = True
            t = time.perf_counter(); pa = prm._dijkstra(s, g); t_a += time.perf_counter() - t
            n_q += 1
            if pd is None or pa is None:
                both_none += pd is None and pa is None
                continue
            same_cost += abs(cost(prm, pd) - cost(prm, pa)) < 1e-9
            same_path += pd == pa
    roadmap.A_STAR_PATH = False
    found = n_q - both_none
    print(f"{n_q} searches on 6 graphs of ~2300 nodes, {found} with a path")
    check(both_none + found == n_q and same_cost == found, f"A* finds a path exactly when Dijkstra does, "
                                                            f"same cost in {same_cost}/{found}")
    print(f"  same nodes in {same_path}/{found} (the others: equal-cost ties)")
    check(t_a < t_d, f"search time {1000 * t_a / n_q:.1f} ms with A* against {1000 * t_d / n_q:.1f} ms "
                     f"({100 * (1 - t_a / t_d):.0f}% less)")
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
