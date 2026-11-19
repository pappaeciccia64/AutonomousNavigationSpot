#!/usr/bin/env python3
"""
Does the slope cost change the routes? Chosen route against the shortest one, per decision.

Every iteration_*_vis.npz saves the PRM graph at the moment of a decision (nodes, edges, the
edge costs the planner used, the slopes known so far) and the chosen path. On that graph this
tool runs two Dijkstra searches between the nodes nearest to the start and the end of the
chosen path: one with the saved edge costs (the planner's route) and one with the plain edge
length (the shortest route). A decision counts only if the cost route reproduces the logged
chosen path (every node within REPRODUCE_M of it).

Per route: length, and on the edges whose slope is known the worst edge slope and the metres
above STEEP_DEG (edge slope = the worse of |along| and |across|, in degrees). The summary
compares the routes that differ and whose slope is known on at least MIN_SEEN of the length.

Usage: python3 tools/route_vs_shortest.py 28-10-2026_13-50-46 28-10-2026_14-23-50 [--png out.png]
"""
import argparse
import glob
import heapq
import os

import numpy as np

REPRODUCE_M = 0.3
MIN_SEEN = 0.8
STEEP_DEG = 15.0
SAME_DEG = 0.5


def _dijkstra(adj, s, t):
    dist, prev, pq = {s: 0.0}, {}, [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == t:
            break
        if d > dist[u]:
            continue
        for v, w in adj.get(u, ()):
            if d + w < dist.get(v, np.inf):
                dist[v], prev[v] = d + w, u
                heapq.heappush(pq, (d + w, v))
    if t not in dist:
        return None
    path = [t]
    while path[-1] != s:
        path.append(prev[path[-1]])
    return path[::-1]


def _dist_to_polyline(q, poly):
    best = np.inf
    for a, b in zip(poly[:-1], poly[1:]):
        ab = b - a
        t = np.clip(np.dot(q - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0)
        best = min(best, float(np.hypot(*(a + t * ab - q))))
    return best


def decisions(mission):
    """One dict per decision of the mission: file, same (bool), chosen / shortest route stats."""
    out = []
    for f in sorted(glob.glob(f'MissionMap/Mission_{mission}/iteration_*_vis.npz')):
        d = np.load(f, allow_pickle=True)
        cp = d['chosen_path']
        if len(cp) < 2 or 'prm_edge_weights' not in d.files:
            continue
        ids, xy = d['prm_node_ids'], d['prm_node_xy']
        pos = {int(i): p for i, p in zip(ids, xy)}
        slope = {(min(a, b), max(a, b)): float(np.degrees(np.arctan(max(abs(lo), abs(la)))))
                 for (a, b), (lo, la) in zip(d['prm_slope_keys'], d['prm_slope_values'])}
        cost_adj, len_adj = {}, {}
        for (a, b), w in zip(d['prm_edges'], d['prm_edge_weights']):
            a, b = int(a), int(b)
            if a not in pos or b not in pos or not np.isfinite(w):
                continue
            ln = float(np.hypot(*(pos[a] - pos[b])))
            for u, v in ((a, b), (b, a)):
                cost_adj.setdefault(u, []).append((v, float(w)))
                len_adj.setdefault(u, []).append((v, ln))
        nearest = lambda p: int(ids[np.argmin(np.hypot(*(xy - p).T))])
        s, t = nearest(cp[0]), nearest(cp[-1])
        if s == t:
            continue
        chosen, shortest = _dijkstra(cost_adj, s, t), _dijkstra(len_adj, s, t)
        if chosen is None or shortest is None:
            continue
        if max(_dist_to_polyline(pos[n], cp) for n in chosen) > REPRODUCE_M:
            continue

        def stats(path):
            length = seen = steep = 0.0
            worst = np.nan
            for a, b in zip(path[:-1], path[1:]):
                ln = float(np.hypot(*(pos[a] - pos[b])))
                length += ln
                k = (min(a, b), max(a, b))
                if k in slope:
                    seen += ln
                    worst = slope[k] if np.isnan(worst) else max(worst, slope[k])
                    steep += ln if slope[k] > STEEP_DEG else 0.0
            return dict(len=length, seen=seen / length, worst=worst, steep=steep)

        out.append(dict(file=os.path.basename(f), same=chosen == shortest,
                        chosen=stats(chosen), shortest=stats(shortest)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('missions', nargs='+', help='e.g. 28-10-2026_13-50-46')
    ap.add_argument('--png', help='scatter of the worst edge slope, shortest against chosen')
    args = ap.parse_args()
    rows = []
    for m in args.missions:
        r = decisions(m)
        print(f'{m}: {len(r)} decisions, {sum(not x["same"] for x in r)} differ from the shortest')
        rows += r
    diff = [r for r in rows if not r['same']]
    cmp_ = [r for r in diff if r['chosen']['seen'] >= MIN_SEEN and r['shortest']['seen'] >= MIN_SEEN]
    print(f'\n{len(rows)} decisions: {len(rows) - len(diff)} same as the shortest, {len(diff)} differ, '
          f'{len(cmp_)} of these with slopes known on >= {MIN_SEEN:.0%} of both routes')
    if not cmp_:
        return
    wc = np.array([r['chosen']['worst'] for r in cmp_])
    ws = np.array([r['shortest']['worst'] for r in cmp_])
    lc, ls = sum(r['chosen']['len'] for r in cmp_), sum(r['shortest']['len'] for r in cmp_)
    sc, ss = sum(r['chosen']['steep'] for r in cmp_), sum(r['shortest']['steep'] for r in cmp_)
    print(f'worst edge: lower {(wc < ws - SAME_DEG).sum()}, same {(abs(wc - ws) <= SAME_DEG).sum()}, '
          f'higher {(wc > ws + SAME_DEG).sum()} (median {np.median(wc):.1f} vs {np.median(ws):.1f} deg)')
    print(f'length: {lc:.1f} m vs {ls:.1f} m ({100 * (lc / ls - 1):+.0f}%)')
    print(f'above {STEEP_DEG:.0f} deg: {sc:.1f} m vs {ss:.1f} m'
          + (f' ({100 * (sc / ss - 1):+.0f}%)' if ss else ''))
    if args.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        extra = np.array([100 * (r['chosen']['len'] / r['shortest']['len'] - 1) for r in cmp_])
        fig, ax = plt.subplots(figsize=(8, 7))
        pts = ax.scatter(ws, wc, c=extra, cmap='viridis', vmin=0, vmax=20, s=60, edgecolors='k')
        fig.colorbar(pts, ax=ax).set_label('extra length of the chosen route (%)')
        lim = [0, max(ws.max(), wc.max()) + 2]
        ax.plot(lim, lim, 'k--', lw=1)
        ax.set_xlim(lim); ax.set_ylim(lim); ax.set_aspect('equal')
        ax.set_xlabel('worst edge of the SHORTEST route (deg)')
        ax.set_ylabel('worst edge of the CHOSEN route (deg)')
        ax.set_title(f'{len(cmp_)} decisions where the routes differ (below the diagonal: chosen is less steep)')
        fig.tight_layout()
        fig.savefig(args.png, dpi=100)
        print('saved', args.png)


if __name__ == '__main__':
    main()
