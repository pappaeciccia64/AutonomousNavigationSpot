#!/usr/bin/env python3
"""
Which summary of the plane slope along an edge reads best what the robot then walks on?

The edge slope (slopes.plane_slope_along) samples the plane gradient at every cell along the
edge and keeps the 90th percentile of |slope along| and |slope across|. This tool checks that
choice on the stretches really walked: the trajectory (trajectory.csv, ~5 Hz, every move,
cruise included) is cut into straight stretches of STRETCH_M; for every stretch the last scan
saved before it started (scans/*.npz, the data the planner had, at most MAX_SCAN_AGE_S old)
gives the plane field (the same as the mission: PLANE_EDGE_HALF_WINDOW_M on the usable cells)
and, along the straight line from start to end of the stretch, the samples of the slope along
it and across it. Summaries of those samples: mean, p50, p75, p90, max.

Measured, in the stretch time: the ground plane Spot estimates from the legs (gpe_long_deg,
gpe_lat_deg, from 04-11): the slope under the feet, along and across the heading. Two truths:
  worst   the 90th percentile of the measured samples: what the veto and the gait must catch
          (the measured max is one sample of a noisy 5 Hz signal)
  mean    the mean of the measured samples: the typical effort, closer to what a cost wants

Errors in degrees on the magnitudes: bias, MAE, 90th percentile of |error|, correlation; and,
as a classifier of the steep stretches (measured worst above STEEP_DEG), how many each summary
catches with its threshold set to the same STEEP_DEG.

    python3 tools/slope_percentile_check.py                  # every mission with gpe and scans
    python3 tools/slope_percentile_check.py 04-11-2026_14-08-57 --stretch 0.8
Read only.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from spot_explorer.perception import slopes             # noqa: E402
from slope_check import _load_scans                     # noqa: E402

MAX_SCAN_AGE_S = 20.0
STRETCH_M = 1.0                 # about one edge (0.5-1.25 m)
MIN_SAMPLES = 4                 # measured samples in a stretch
STRAIGHT_MIN = 0.9              # chord / walked length: only (almost) straight stretches
STEEP_DEG = 10.0
SUMMARIES = ['mean', 'p50', 'p75', 'p90', 'max']


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def _summ(v):
    return dict(mean=float(np.mean(v)), p50=float(np.percentile(v, 50)), p75=float(np.percentile(v, 75)),
                p90=float(np.percentile(v, 90)), max=float(np.max(v)))


def _field(scan):
    if 'plane' not in scan['fields']:
        gx, gy, _ = slopes.plane_gradient_field(np.where(scan['usable'], scan['terrain'], np.nan), scan['usable'],
                                                scan['cs'], half_window_m=slopes.PLANE_EDGE_HALF_WINDOW_M)
        scan['fields']['plane'] = (gx, gy)
    return scan['fields']['plane']


def _along(scan, x1, y1, x2, y2):
    """|slope along| and |slope across| (degrees) at every cell sample of the segment, None if
    fewer than half have a value (as plane_slope_along)."""
    gx, gy = _field(scan)
    ox, oy = scan['origin']
    cs = scan['cs']
    d = float(np.hypot(x2 - x1, y2 - y1))
    ux, uy = (x2 - x1) / d, (y2 - y1) / d
    n = max(2, int(np.ceil(d / cs)) + 1)
    s = np.linspace(0.0, d, n)
    xs, ys = x1 + ux * s, y1 + uy * s
    c = np.floor((xs - ox) / cs).astype(int)
    r = np.floor((ys - oy) / cs).astype(int)
    ok = (r >= 0) & (r < gx.shape[0]) & (c >= 0) & (c < gx.shape[1])
    g_x, g_y = np.full(n, np.nan), np.full(n, np.nan)
    g_x[ok], g_y[ok] = gx[r[ok], c[ok]], gy[r[ok], c[ok]]
    good = np.isfinite(g_x) & np.isfinite(g_y)
    if good.sum() < slopes.MIN_ARC_SAMPLE_FRACTION * n:
        return None
    lon = np.degrees(np.arctan(np.abs(g_x[good] * ux + g_y[good] * uy)))
    lat = np.degrees(np.arctan(np.abs(-g_x[good] * uy + g_y[good] * ux)))
    return lon, lat


def stretches_of(name, stretch_m):
    logs = os.path.join(ROOT, 'MissionLogs', 'Mission_' + name)
    tpath = os.path.join(logs, 'trajectory.csv')
    if not os.path.exists(tpath):
        return []
    rows = []
    with open(tpath, newline='') as f:
        for r in csv.DictReader(f):
            t, x, y = _num(r.get('time')), _num(r.get('x')), _num(r.get('y'))
            gl, gt = _num(r.get('gpe_long_deg')), _num(r.get('gpe_lat_deg'))
            if np.isfinite(t) and np.isfinite(x) and np.isfinite(y) and np.isfinite(gl) and np.isfinite(gt):
                rows.append((t, x, y, abs(gl), abs(gt)))
    if not rows:
        return []
    scans = _load_scans(os.path.join(ROOT, 'MissionMap', 'Mission_' + name))
    if not scans:
        return []
    times = np.array([s['time'] for s in scans])
    T = np.array(rows)
    out, i = [], 0
    while i < len(T) - 1:
        j, walked = i, 0.0
        while j + 1 < len(T) and walked < stretch_m and T[j + 1, 0] - T[j, 0] < 1.5:
            walked += float(np.hypot(T[j + 1, 1] - T[j, 1], T[j + 1, 2] - T[j, 2]))
            j += 1
        if walked < stretch_m:
            i = j + 1
            continue
        seg = T[i:j + 1]
        chord = float(np.hypot(seg[-1, 1] - seg[0, 1], seg[-1, 2] - seg[0, 2]))
        k = int(np.searchsorted(times, seg[0, 0], side='right')) - 1
        if chord >= STRAIGHT_MIN * walked and len(seg) >= MIN_SAMPLES and k >= 0 \
                and seg[0, 0] - times[k] <= MAX_SCAN_AGE_S:
            est = _along(scans[k], seg[0, 1], seg[0, 2], seg[-1, 1], seg[-1, 2])
            if est is not None:
                out.append(dict(mission=name, est_long=_summ(est[0]), est_lat=_summ(est[1]),
                                worst_long=float(np.percentile(seg[:, 3], 90)), mean_long=float(np.mean(seg[:, 3])),
                                worst_lat=float(np.percentile(seg[:, 4], 90)), mean_lat=float(np.mean(seg[:, 4]))))
        i = j
    return out


def report(S):
    print(f"\n{len(S)} stretches of the trajectory with a scan before them "
          f"({len({s['mission'] for s in S})} missions).")
    for comp in ('long', 'lat'):
        worst = np.array([s[f'worst_{comp}'] for s in S])
        mean = np.array([s[f'mean_{comp}'] for s in S])
        print(f"\n=== {'ALONG the heading (pitch)' if comp == 'long' else 'ACROSS the heading (roll)'}: measured "
              f"worst p50 {np.median(worst):.1f} p90 {np.percentile(worst, 90):.1f} deg, max {worst.max():.1f}")
        for tname, truth in (('worst (p90 of the measured)', worst), ('mean of the measured', mean)):
            print(f"  against the {tname}:")
            print(f"    {'summary':8s} {'bias':>6s} {'MAE':>6s} {'p90|e|':>7s} {'corr':>6s}")
            for m in SUMMARIES:
                e = np.array([s[f'est_{comp}'][m] for s in S])
                err = e - truth
                corr = float(np.corrcoef(e, truth)[0, 1]) if np.std(e) > 0 and np.std(truth) > 0 else float('nan')
                print(f"    {m:8s} {err.mean():+6.2f} {np.abs(err).mean():6.2f} {np.percentile(np.abs(err), 90):7.2f} {corr:6.2f}")
        steep = worst > STEEP_DEG
        if steep.any():
            print(f"  steep stretches (measured worst > {STEEP_DEG:.0f} deg): {int(steep.sum())}; "
                  f"caught with the threshold at {STEEP_DEG:.0f} deg / false alarms on the {int((~steep).sum())} others:")
            for m in SUMMARIES:
                e = np.array([s[f'est_{comp}'][m] for s in S])
                print(f"    {m:8s} caught {int((e[steep] > STEEP_DEG).sum()):4d}/{int(steep.sum())}   "
                      f"false {int((e[~steep] > STEEP_DEG).sum()):4d}")


def main():
    global STEEP_DEG
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('missions', nargs='*', help="e.g. 04-11-2026_14-08-57 (default: every mission with gpe)")
    ap.add_argument('--stretch', type=float, default=STRETCH_M)
    ap.add_argument('--steep', type=float, default=STEEP_DEG)
    a = ap.parse_args()
    STEEP_DEG = a.steep
    names = a.missions or sorted(os.path.basename(p)[len('Mission_'):]
                                 for p in glob.glob(os.path.join(ROOT, 'MissionLogs', 'Mission_*')))
    S = []
    for n in names:
        s = stretches_of(n, a.stretch)
        if s:
            print(f"  {n}: {len(s)} stretches")
        S += s
    if not S:
        sys.exit("no stretch with ground plane data (gpe, from 04-11) and a scan before it")
    report(S)


if __name__ == '__main__':
    main()
