#!/usr/bin/env python3
"""
GAIT THRESHOLDS FOR THE PLANE SLOPES (slopes.PLANE_SLOPES), from the legs already walked.

The gait thresholds (gait.GAIT_PLAIN_SLOPE_MAX, GAIT_MODERATE_SLOPE_MAX) act on
max(longitudinal, lateral) of the leg and were tuned on today's estimate (differences on a
base along the edge, with the body), which on the slopes reads ~2 deg more than the robot.
With PLANE_SLOPES on the same thresholds would put fewer legs in CRAWL. This tool says which
plane thresholds give what.

For every stop-and-go leg of segments.csv (the cruise ones are not there) the last scan saved
before the leg started (scans/*.npz, the data the planner had) is taken, and on it, along the
leg from x_from to x_to:
  edges   today's estimate with the body (slopes.compute_arc_slope_profile, body_ext_m)
  plane   the estimate of PLANE_SLOPES (plane_gradient_field with PLANE_EDGE_HALF_WINDOW_M,
          plane_slope_profile)
Measured, from trajectory.csv in the leg time: the body pitch (|pitch|, 90th percentile of
the samples). The roll is not a measure of the ground (Spot keeps the body level), so the
lateral is only compared between the two methods; from 04-11 trajectory.csv also has the
ground plane of the legs (gpe_long_deg, gpe_lat_deg), used when present.

What it prints:
  1. How well each method reads the slope of the leg (longitudinal, against the body pitch).
  2. Levels with today's thresholds: on edges (what the robot did) and on plane (what it
     would do with the switch on and the same thresholds), and the level matrix.
  3. Plane thresholds equivalent to today's:
       same share  the plane threshold that puts the same share of legs above it
       same catch  the plane threshold that catches the same inclined legs (measured pitch
                   above the threshold in degrees) as today's, with the false alarms of both
  4. Table of candidate plane thresholds as a classifier against the measured slope.

    python3 tools/plane_thresholds.py                     # every mission with legs and scans
    python3 tools/plane_thresholds.py --since 2026-10-22
    python3 tools/plane_thresholds.py --csv legs.csv      # one row per leg
It changes no parameter: the decision goes in the CHANGELOG with the numbers.
"""
import argparse
import csv
import datetime
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools'))

from spot_explorer.perception import gait, slopes        # noqa: E402
from slope_check import _load_scans                      # noqa: E402

MAX_SCAN_AGE_S = 30.0
MIN_TRAVELLED_M = 0.20
MIN_SAMPLES = 3


def _deg(t):
    return float(np.degrees(np.arctan(t)))


def _tan(d):
    return float(np.tan(np.radians(d)))


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def _mission_date_ok(name, since):
    """True without --since, or if 'Mission_DD-MM-YYYY_...' is from that day (YYYY-MM-DD) on."""
    if not since:
        return True
    try:
        d, m, y = name.split('_')[1].split('-')
    except (IndexError, ValueError):
        return False
    return f"{y}-{m}-{d}" >= since


def _epoch(stamp):
    """'2026-10-28 13:51:01' (local time of the robot PC) -> seconds."""
    return datetime.datetime.strptime(stamp.strip(), '%Y-%m-%d %H:%M:%S').timestamp()


def _plane_field(scan):
    half = slopes.PLANE_EDGE_HALF_WINDOW_M
    if half not in scan['fields']:
        gx, gy, _ = slopes.plane_gradient_field(np.where(scan['usable'], scan['terrain'], np.nan),
                                                scan['usable'], scan['cs'], half_window_m=half)
        scan['fields'][half] = (gx, gy)
    return scan['fields'][half]


def legs_of(name):
    logs = os.path.join(ROOT, 'MissionLogs', name)
    scans = _load_scans(os.path.join(ROOT, 'MissionMap', name))
    if not scans:
        return []
    times = np.array([s['time'] for s in scans])
    with open(os.path.join(logs, 'segments.csv'), newline='', encoding='utf-8') as f:
        segs = list(csv.DictReader(f))
    traj = []
    tpath = os.path.join(logs, 'trajectory.csv')
    if os.path.exists(tpath):
        with open(tpath, newline='') as f:
            for r in csv.DictReader(f):
                t, p = _num(r.get('time')), _num(r.get('pitch_deg'))
                if np.isfinite(t) and np.isfinite(p):
                    traj.append((t, p, _num(r.get('gpe_long_deg')), _num(r.get('gpe_lat_deg'))))
    traj = np.array(traj) if traj else np.zeros((0, 4))
    out = []
    for r in segs:
        try:
            t_end = _epoch(r['timestamp'])
            x1, y1, x2, y2 = (float(r[k]) for k in ('x_from', 'y_from', 'x_to', 'y_to'))
        except (KeyError, ValueError):
            continue
        dur, trav = _num(r.get('duration_s')), _num(r.get('dist_traveled_m'))
        if not (np.isfinite(dur) and np.isfinite(trav)) or trav < MIN_TRAVELLED_M:
            continue
        t0 = t_end - dur
        # The scan of the leg is taken standing before it starts (the stamp has 1 s resolution).
        k = int(np.searchsorted(times, t0 + 1.0, side='right')) - 1
        if k < 0 or t0 - times[k] > MAX_SCAN_AGE_S:
            continue
        scan = scans[k]
        ox, oy = scan['origin']
        e = slopes.compute_arc_slope_profile(scan['terrain'], ox, oy, scan['cs'], x1, y1, x2, y2,
                                             valid_2d=scan['usable'], body_ext_m=slopes.ARC_BODY_EXTENSION_M)
        gx, gy = _plane_field(scan)
        p_lo, p_la, p_has = slopes.plane_slope_along(gx, gy, ox, oy, scan['cs'], x1, y1, x2, y2)
        if not (e[5] and p_has):
            continue
        sel = (traj[:, 0] >= t0 - 0.5) & (traj[:, 0] <= t_end + 0.5) if len(traj) else np.zeros(0, bool)
        if np.count_nonzero(sel) < MIN_SAMPLES:
            continue
        pitch = np.abs(traj[sel, 1])
        gl, gt = np.abs(traj[sel, 2]), np.abs(traj[sel, 3])
        out.append(dict(
            mission=name, timestamp=r['timestamp'], tier=r.get('tier', ''), success=r.get('success', ''),
            dist_m=_num(r.get('dist_m')), roughness=_num(r.get('roughness')),
            logged_long=_num(r.get('long_slope')), logged_lat=_num(r.get('lat_slope')),
            edges_long=float(e[3]), edges_lat=float(e[4]), plane_long=float(p_lo), plane_lat=float(p_la),
            pitch_p90_deg=float(np.percentile(pitch, 90)), pitch_med_deg=float(np.median(pitch)),
            gpe_long_p90_deg=float(np.nanpercentile(gl, 90)) if np.isfinite(gl).any() else np.nan,
            gpe_lat_p90_deg=float(np.nanpercentile(gt, 90)) if np.isfinite(gt).any() else np.nan,
            scan_age_s=float(t0 - times[k])))
    return out


def tier(g, rough, plain, moderate):
    """Level as motion.select_mobility_params, with the given slope thresholds."""
    return np.where((g < plain) & (rough < gait.GAIT_PLAIN_ROUGH_MAX), 0,
                    np.where((g < moderate) & (rough < gait.GAIT_MODERATE_ROUGH_MAX), 1, 2))


def title(t):
    print("\n" + "=" * len(t) + "\n" + t + "\n" + "=" * len(t))


def report(L):
    col = lambda k: np.array([l[k] for l in L], dtype=np.float64)
    meas = col('pitch_p90_deg')
    rough = np.nan_to_num(col('roughness'), nan=0.0)
    g = {m: np.maximum(col(f'{m}_long'), col(f'{m}_lat')) for m in ('edges', 'plane')}
    n_m = len(set(l['mission'] for l in L))
    print(f"{len(L)} legs of {n_m} missions (travelled >= {MIN_TRAVELLED_M} m, scan within {MAX_SCAN_AGE_S:.0f} s, "
          f"body samples during the leg).")
    print(f"Measured |pitch| (p90 in the leg): median {np.median(meas):.1f} deg, 90th percentile "
          f"{np.percentile(meas, 90):.1f}, maximum {meas.max():.1f}.")

    title("1. LONGITUDINAL SLOPE OF THE LEG against the body pitch (degrees)")
    print(f"{'method':8s} {'bias':>6s} {'MAE':>6s} {'p90|e|':>7s} {'corr':>6s}   {'on slopes (meas > 5 deg)':>26s}")
    steep = meas > 5.0
    for m in ('edges', 'plane'):
        est = np.degrees(np.arctan(col(f'{m}_long')))
        e = est - meas
        es = e[steep]
        c = np.corrcoef(est, meas)[0, 1]
        s_txt = f"bias {es.mean():+.2f} MAE {np.abs(es).mean():.2f} ({steep.sum()})" if steep.sum() > 4 else "-"
        print(f"{m:8s} {e.mean():+6.2f} {np.abs(e).mean():6.2f} {np.percentile(np.abs(e), 90):7.2f} {c:6.2f}   {s_txt:>26s}")
    lat_e, lat_p = np.degrees(np.arctan(col('edges_lat'))), np.degrees(np.arctan(col('plane_lat')))
    print(f"Lateral (not measurable on the body): edges median {np.median(lat_e):.1f} p90 {np.percentile(lat_e, 90):.1f} "
          f"deg, plane median {np.median(lat_p):.1f} p90 {np.percentile(lat_p, 90):.1f} deg.")
    gpe = col('gpe_long_p90_deg')
    if np.isfinite(gpe).sum() > 4:
        ok = np.isfinite(gpe)
        print(f"Against the ground plane of the legs (gpe, {ok.sum()} legs):")
        for m in ('edges', 'plane'):
            for comp in ('long', 'lat'):
                e = np.degrees(np.arctan(col(f'{m}_{comp}')))[ok] - col(f'gpe_{comp}_p90_deg')[ok]
                print(f"  {m:6s} {comp:4s} bias {e.mean():+.2f} MAE {np.abs(e).mean():.2f}")
    a = np.abs(col('edges_long') - col('logged_long'))
    print(f"Check: edges recomputed against the value logged by the mission, |difference| median {np.median(a):.3f}, "
          f"within 0.02 in {np.mean(a < 0.02):.0%} of the legs (the mission may have used the global map).")

    P, M = gait.GAIT_PLAIN_SLOPE_MAX, gait.GAIT_MODERATE_SLOPE_MAX
    title(f"2. LEVELS WITH TODAY'S THRESHOLDS ({P} / {M}, on max(long, lat))")
    names = ('PLAIN', 'MODERATE', 'HARD')
    te, tp = tier(g['edges'], rough, P, M), tier(g['plane'], rough, P, M)
    for m, t in (('edges', te), ('plane', tp)):
        print(f"{m:6s} " + "  ".join(f"{names[i]} {np.mean(t == i):5.1%}" for i in range(3)))
    print("Matrix (rows edges, columns plane):")
    print(f"{'':10s}" + "".join(f"{n:>10s}" for n in names))
    for i in range(3):
        print(f"{names[i]:10s}" + "".join(f"{np.sum((te == i) & (tp == j)):10d}" for j in range(3)))
    print(f"Only the slope decides in {np.mean(rough < gait.GAIT_PLAIN_ROUGH_MAX):.0%} of the legs "
          f"(roughness below {gait.GAIT_PLAIN_ROUGH_MAX}); above, the roughness is enough for MODERATE.")

    title("3. PLANE THRESHOLDS EQUIVALENT TO TODAY'S")
    for label, t_old in (('PLAIN -> MODERATE', P), ('MODERATE -> HARD', M)):
        share = np.mean(g['edges'] >= t_old)
        t_share = float(np.quantile(g['plane'], 1.0 - share))
        # The slope in degrees today's threshold corresponds to, on the measured pitch: the
        # inclined legs are those above it.
        incl = meas > _deg(t_old)
        line = (f"{label}: today {t_old:.3f} ({_deg(t_old):.1f} deg), {share:.1%} of the legs above. "
                f"Same share on plane: {t_share:.3f} ({_deg(t_share):.1f} deg).")
        print(line)
        if incl.sum() >= 5:
            catch = np.mean(g['edges'][incl] >= t_old)
            t_catch = float(np.quantile(g['plane'][incl], 1.0 - catch))
            fa_e = np.mean(g['edges'][~incl] >= t_old)
            fa_p = np.mean(g['plane'][~incl] >= t_catch)
            print(f"   {incl.sum()} legs with measured pitch > {_deg(t_old):.1f} deg: today caught {catch:.0%}, "
                  f"false alarms {fa_e:.0%}. Same catch on plane: {t_catch:.3f} ({_deg(t_catch):.1f} deg), "
                  f"false alarms {fa_p:.0%}.")
        else:
            print(f"   only {incl.sum()} legs with measured pitch > {_deg(t_old):.1f} deg: not enough for the catch.")

    title("4. CANDIDATE PLANE THRESHOLDS as a classifier (inclined = measured pitch above the threshold's degrees)")
    for target_deg in (_deg(P), _deg(M)):
        incl = meas > target_deg
        print(f"\nInclined = measured pitch > {target_deg:.1f} deg: {incl.sum()} legs out of {len(meas)}")
        print(f"{'method':6s} {'threshold':>9s} {'deg':>5s} {'caught':>7s} {'false al.':>9s} {'legs above':>10s}")
        for t in (0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25):
            for m in ('edges', 'plane'):
                if incl.sum() == 0 or (~incl).sum() == 0:
                    continue
                s = g[m]
                print(f"{m:6s} {t:9.3f} {_deg(t):5.1f} {np.mean(s[incl] >= t):7.0%} {np.mean(s[~incl] >= t):9.0%} "
                      f"{np.mean(s >= t):10.0%}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('missions', nargs='*')
    ap.add_argument('--since', default=None, help='only the missions from this day on (YYYY-MM-DD)')
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()
    names = [n if n.startswith('Mission_') else 'Mission_' + n for n in a.missions]
    if not names:
        names = sorted(os.path.basename(os.path.dirname(p))
                       for p in glob.glob(os.path.join(ROOT, 'MissionLogs', 'Mission_*', 'segments.csv')))
    names = [n for n in names if _mission_date_ok(n, a.since)]
    L = []
    for n in names:
        legs = legs_of(n)
        if legs:
            print(f"{n}: {len(legs)} legs")
        L += legs
    if not L:
        print("No leg with scan and trajectory.")
        return
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(L[0].keys()))
            w.writeheader()
            w.writerows(L)
        print(f"Legs in {a.csv}")
    report(L)


if __name__ == '__main__':
    main()
