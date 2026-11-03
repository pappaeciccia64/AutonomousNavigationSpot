#!/usr/bin/env python3
"""
How good are the slope estimates? For every sample of the real trajectory (trajectory.csv,
~5 Hz during the moves) the slope under the robot is estimated from the last scan saved
BEFORE that instant (scans/*.npz, the data the planner had), with several methods, and
compared with what the robot really stood on:

  measured   body tilt (pitch, roll of the body in the VISION frame: all missions; Spot
             partly levels the body, so it reads less than the ground) and, from 04-11,
             the ground plane estimated by Spot from the legs (frame 'gpe': gpe_long_deg,
             gpe_lat_deg columns), the direct measure of the ground under the feet.
  methods    edges      today's estimate (slopes.compute_arc_slope_profile) on the body
                        segment, nose to tail along the heading: differences over
                        SLOPE_BASELINE_M, 90th percentile
             plane050   local plane 0.5 x 0.5 m (slopes.plane_gradient_field), 90th
                        percentile along the same segment (slopes.plane_slope_along)
             plane080   the same with a 0.8 m window
             point080   the 0.8 m plane at the robot centre only (no percentile)
             point110   the same with a 1.1 m window (the body length)
             body_fit   one plane fitted on the body footprint (1.1 x 0.5 m oriented with
                        the robot): the slope the body would take on that ground

Longitudinal = along the heading, lateral = across it, in degrees. The planner uses
magnitudes, so the errors are on |estimate| - |measured|: bias (mean), MAE, RMSE, 90th
percentile of |error|, correlation. For the methods that give a sign (body_fit) also the
signed correlation. Only the samples where every method has data are compared, split by
the distance between the robot at the scan and the sample (how far ahead the data were seen).

    python3 tools/slope_check.py 28-10-2026_13-50-46
    python3 tools/slope_check.py 28-10-2026_13-50-46 03-11-2026_16-02-47 --max-age 20
    python3 tools/slope_check.py --all                 # every mission with trajectory and scans
    ... --csv out.csv                                    # one row per sample
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from spot_explorer.perception import slopes, spot_grid           # noqa: E402
from spot_explorer.perception.geometry import ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
METHODS = ['edges', 'plane050', 'plane080', 'point080', 'point110', 'body_fit']
DIST_BANDS = [(0.0, 1.0), (1.0, 2.0), (2.0, 99.0)]


def _folders(name):
    name = name if name.startswith('Mission_') else 'Mission_' + name
    return os.path.join(ROOT, 'MissionLogs', name), os.path.join(ROOT, 'MissionMap', name)


def _load_scans(map_folder):
    out = []
    for f in sorted(glob.glob(os.path.join(map_folder, 'scans', 'scan_*.npz'))):
        try:
            d = np.load(f, allow_pickle=True)
            if 'terrain_corrected' not in d.files or 'time' not in d.files:
                continue
            terr = np.asarray(d['terrain_corrected'], dtype=np.float64)
            shape = terr.shape
            iv = np.asarray(d['is_valid'], dtype=bool).ravel()
            if 'unwritten' in d.files:
                iv = iv & ~np.asarray(d['unwritten'], dtype=bool).ravel()
            usable = spot_grid.slope_usable_mask(iv, np.asarray(d['obstacle_distance']).ravel(), shape)
            out.append(dict(time=float(d['time']), terrain=terr, usable=np.asarray(usable, dtype=bool).reshape(shape),
                            origin=(float(d['grid_origin'][0]), float(d['grid_origin'][1])),
                            cs=float(d['cell_size']), robot=np.asarray(d['robot_xyz'], dtype=np.float64)[:2],
                            fields={}))
        except Exception as e:
            print(f"  {os.path.basename(f)} not readable: {e}")
    out.sort(key=lambda s: s['time'])
    return out


def _field(scan, half_m):
    if half_m not in scan['fields']:
        gx, gy, _ = slopes.plane_gradient_field(np.where(scan['usable'], scan['terrain'], np.nan),
                                                scan['usable'], scan['cs'], half_window_m=half_m)
        scan['fields'][half_m] = (gx, gy)
    return scan['fields'][half_m]


def _body_fit(scan, x, y, yaw):
    """Least-squares plane on the cells under the body footprint: signed (long, lat) slopes
    as tangents, or None with less than half of the footprint usable."""
    cs = scan['cs']
    ox, oy = scan['origin']
    ny, nx = scan['terrain'].shape
    r = np.hypot(ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M)
    c0, c1 = int(np.floor((x - r - ox) / cs)), int(np.ceil((x + r - ox) / cs))
    r0, r1 = int(np.floor((y - r - oy) / cs)), int(np.ceil((y + r - oy) / cs))
    c0, r0, c1, r1 = max(c0, 0), max(r0, 0), min(c1, nx), min(r1, ny)
    if c1 <= c0 or r1 <= r0:
        return None
    rows, cols = np.mgrid[r0:r1, c0:c1]
    px, py = ox + (cols + 0.5) * cs - x, oy + (rows + 0.5) * cs - y
    c, s = np.cos(yaw), np.sin(yaw)
    along, across = c * px + s * py, -s * px + c * py
    inside = (np.abs(along) <= ROBOT_HALF_LENGTH_M) & (np.abs(across) <= ROBOT_HALF_WIDTH_M)
    n_foot = np.count_nonzero(inside)
    ok = inside & scan['usable'][r0:r1, c0:c1]
    if n_foot == 0 or np.count_nonzero(ok) < 0.5 * n_foot:
        return None
    A = np.c_[np.ones(np.count_nonzero(ok)), along[ok], across[ok]]
    coef, *_ = np.linalg.lstsq(A, scan['terrain'][r0:r1, c0:c1][ok], rcond=None)
    return float(coef[1]), float(coef[2])


def _estimates(scan, x, y, yaw):
    """{method: (long, lat)} in degrees (magnitudes; body_fit signed), None where no data."""
    h = (np.cos(yaw), np.sin(yaw))
    x1, y1 = x - ROBOT_HALF_LENGTH_M * h[0], y - ROBOT_HALF_LENGTH_M * h[1]
    x2, y2 = x + ROBOT_HALF_LENGTH_M * h[0], y + ROBOT_HALF_LENGTH_M * h[1]
    ox, oy = scan['origin']
    deg = lambda v: float(np.degrees(np.arctan(v)))
    out = {}
    lo, la, has = slopes.compute_arc_slope_profile(scan['terrain'], ox, oy, scan['cs'], x1, y1, x2, y2,
                                                   valid_2d=scan['usable'])
    out['edges'] = (deg(lo), deg(la)) if has else None
    for name, half in (('plane050', 0.25), ('plane080', 0.40)):
        gx, gy = _field(scan, half)
        lo, la, has = slopes.plane_slope_along(gx, gy, ox, oy, scan['cs'], x1, y1, x2, y2)
        out[name] = (deg(lo), deg(la)) if has else None
    for name, half in (('point080', 0.40), ('point110', 0.55)):
        gx, gy = _field(scan, half)
        c = int(np.floor((x - ox) / scan['cs']))
        rr = int(np.floor((y - oy) / scan['cs']))
        ny, nx = gx.shape
        if 0 <= rr < ny and 0 <= c < nx and np.isfinite(gx[rr, c]) and np.isfinite(gy[rr, c]):
            lo = gx[rr, c] * h[0] + gy[rr, c] * h[1]
            la = -gx[rr, c] * h[1] + gy[rr, c] * h[0]
            out[name] = (abs(deg(lo)), abs(deg(la)))
        else:
            out[name] = None
    bf = _body_fit(scan, x, y, yaw)
    out['body_fit'] = (deg(bf[0]), deg(bf[1])) if bf is not None else None
    return out


def _stats(pred, meas):
    e = np.abs(pred) - np.abs(meas)
    corr = float(np.corrcoef(np.abs(pred), np.abs(meas))[0, 1]) if len(e) > 2 and np.std(pred) > 0 and np.std(meas) > 0 else np.nan
    return dict(n=len(e), bias=float(np.mean(e)), mae=float(np.mean(np.abs(e))), rmse=float(np.sqrt(np.mean(e * e))),
                p90=float(np.percentile(np.abs(e), 90)), corr=corr)


def check(names, max_age_s, csv_path=None):
    rows = []
    for name in names:
        logs, maps = _folders(name)
        tpath = os.path.join(logs, 'trajectory.csv')
        if not os.path.exists(tpath):
            print(f"{name}: no trajectory.csv")
            continue
        scans = _load_scans(maps)
        if not scans:
            print(f"{name}: no scans with terrain")
            continue
        times = np.array([s['time'] for s in scans])
        with open(tpath, newline='') as f:
            traj = list(csv.DictReader(f))
        n_used = 0
        for r in traj:
            try:
                t, x, y = float(r['time']), float(r['x']), float(r['y'])
                yaw = np.radians(float(r['yaw_deg']))
                pitch, roll = float(r['pitch_deg']), float(r['roll_deg'])
            except (KeyError, ValueError):
                continue
            k = int(np.searchsorted(times, t, side='right')) - 1
            if k < 0 or t - times[k] > max_age_s:
                continue
            scan = scans[k]
            est = _estimates(scan, x, y, yaw)
            if any(v is None for v in est.values()):
                continue
            gl, gt = r.get('gpe_long_deg'), r.get('gpe_lat_deg')
            row = dict(mission=name, time=t, x=x, y=y, yaw_deg=float(r['yaw_deg']), move=r.get('move', ''),
                       scan_age_s=t - times[k], scan_dist_m=float(np.hypot(x - scan['robot'][0], y - scan['robot'][1])),
                       # Body: pitch > 0 = nose down, so the ground rising ahead gives a negative pitch.
                       body_long=-pitch, body_lat=roll,
                       gpe_long=float(gl) if gl not in (None, '', 'nan') else np.nan,
                       gpe_lat=float(gt) if gt not in (None, '', 'nan') else np.nan)
            for m, (lo, la) in est.items():
                row[f'{m}_long'], row[f'{m}_lat'] = lo, la
            rows.append(row)
            n_used += 1
        print(f"{name}: {n_used} samples compared out of {len(traj)} ({len(scans)} scans)")
    if not rows:
        return
    if csv_path:
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"Samples in {csv_path}")

    def col(key, sel=None):
        return np.array([r[key] for r in rows if sel is None or sel(r)], dtype=np.float64)

    for truth in ('body', 'gpe'):
        if np.all(np.isnan(col(f'{truth}_long'))):
            continue
        ok = lambda r, tr=truth: np.isfinite(r[f'{tr}_long'])
        n = len(col(f'{truth}_long', ok))
        meas_l, meas_t = col(f'{truth}_long', ok), col(f'{truth}_lat', ok)
        print(f"\n=== Against the {'BODY TILT (Spot partly levels it)' if truth == 'body' else 'GROUND PLANE from the legs (gpe)'}"
              f": {n} samples | measured |long| p50 {np.median(np.abs(meas_l)):.1f} p90 "
              f"{np.percentile(np.abs(meas_l), 90):.1f} deg, |lat| p50 {np.median(np.abs(meas_t)):.1f} p90 "
              f"{np.percentile(np.abs(meas_t), 90):.1f} deg")
        for comp, label in (('long', 'LONGITUDINAL'), ('lat', 'LATERAL')):
            print(f"  {label:12s} {'method':9s} {'bias':>6s} {'MAE':>6s} {'RMSE':>6s} {'p90|e|':>7s} {'corr':>6s}   (degrees)")
            for m in METHODS:
                st = _stats(col(f'{m}_{comp}', ok), col(f'{truth}_{comp}', ok))
                print(f"  {'':12s} {m:9s} {st['bias']:+6.2f} {st['mae']:6.2f} {st['rmse']:6.2f} {st['p90']:7.2f} {st['corr']:6.2f}")
            s_corr = np.corrcoef(col(f'body_fit_{comp}', ok), col(f'{truth}_{comp}', ok))[0, 1]
            print(f"  {'':12s} body_fit signed correlation {s_corr:+.2f}")
        print("  MAE by distance from the robot at the scan (longitudinal / lateral):")
        for lo_d, hi_d in DIST_BANDS:
            sel = lambda r, tr=truth, a=lo_d, b=hi_d: np.isfinite(r[f'{tr}_long']) and a <= r['scan_dist_m'] < b
            nb = len(col('scan_dist_m', sel))
            if nb < 5:
                continue
            parts = []
            for m in METHODS:
                sl = _stats(col(f'{m}_long', sel), col(f'{truth}_long', sel))
                stt = _stats(col(f'{m}_lat', sel), col(f'{truth}_lat', sel))
                parts.append(f"{m} {sl['mae']:.2f}/{stt['mae']:.2f}")
            print(f"    {lo_d:.0f}-{hi_d if hi_d < 99 else 'inf'} m ({nb}): " + ", ".join(parts))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('missions', nargs='*')
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--max-age', type=float, default=30.0, help='seconds between scan and sample at most (default 30)')
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()
    names = a.missions
    if a.all:
        names = sorted(os.path.basename(p)[len('Mission_'):] for p in glob.glob(os.path.join(ROOT, 'MissionLogs', 'Mission_*'))
                       if os.path.exists(os.path.join(p, 'trajectory.csv'))
                       and glob.glob(os.path.join(ROOT, 'MissionMap', os.path.basename(p), 'scans', '*.npz')))
    if not names:
        ap.error("one or more missions, or --all")
    check(names, a.max_age, a.csv)


if __name__ == '__main__':
    main()
