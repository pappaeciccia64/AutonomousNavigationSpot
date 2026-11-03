#!/usr/bin/env python3
"""
CALIBRATION from the data: PRM cost weights and gait thresholds, measured on the logs.

    python3 tools/calibration.py                          # every mission in MissionLogs/
    python3 tools/calibration.py --missions Mission_26-10-2026_15-11-21 Mission_22-10-2026_16-04-22
    python3 tools/calibration.py --since 2026-10-22        # only the missions from that day on
    python3 tools/calibration.py --pitch 12                # pitch of an "inclined" leg, for the thresholds
    python3 tools/calibration.py --trials GaitTrials/trials.csv   # also the track trials

It does not touch the robot nor change any parameter: it prints what the data say. Decisions
are taken by reading, and go in the CHANGELOG with the numbers that justify them.

What it prints:
  1. Legs per gait level: how many, successful, pitch and roll, real speed against the limit
     set (if the real one is well below, the limit does not matter).
  2. PRM COST. Regression of the time of a successful leg on length, longitudinal slope,
     lateral slope, roughness:
        duration = a*length + b*long_slope + c*lat_slope + d*roughness + k
     Dijkstra looks only at the ratios between the weights, so the suggested weights are b/a,
     c/a, d/a (with alpha = 1): "how many metres of path a unit of slope is worth", in time.
     Two versions: without the gait level (total effect: the slope also costs because it
     switches to CRAWL) and with the level (direct effect, plus the cost in equivalent metres
     of each level change). 90% intervals with the bootstrap.
  3. FAILURES by slope and roughness band, separating the legs with 0 m travelled (refusals,
     frontier blocks: they do not depend on the terrain) from the others.
  4. GAIT THRESHOLDS as a classifier: for each slope threshold, how many "inclined" legs
     (peak pitch > --pitch degrees) fall above it, how many calm legs end up above it for
     nothing, and what share of the legs goes above. Then how often the roughness thresholds
     really trigger.
  5. TRACK TRIALS (tools/gait_trial.py), if any: per terrain and configuration, success,
     time, speed, oscillations (RMS of pitch and roll around the mean, which on a ramp is the
     slope itself), foot slip, friction estimated by Spot, energy per metre; and the slope
     estimated against the one measured with the inclinometer and by the legs (gpe).

Caveats: the mission data are not a controlled experiment (the gait depends on the slope, so
the two effects mix: that is why there are two regressions) and a high pitch on a ramp is
normal, not instability. For the choice of the gait the track trials count.

The legs of the cruise (continuous motion) are not in segments.csv: only the stop-and-go legs.
Ported on 03-11-2026 from taratura.py of the AdL-version branch.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from spot_explorer.legacy import modern                 # noqa: E402
from spot_explorer.perception import gait, slopes        # noqa: E402
from spot_explorer.planning import roadmap               # noqa: E402

TIERS = ('PLAIN', 'MODERATE', 'HARD')
CAP_VEL = {'PLAIN': gait.GAIT_VEL_PLAIN, 'MODERATE': gait.GAIT_VEL_MODERATE, 'HARD': gait.GAIT_VEL_HARD}
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# -----------------------------------------------------------------------------------------
# Reading
# -----------------------------------------------------------------------------------------
def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def _bool(v):
    return str(v).strip().lower() in ('true', '1', 'si', 'yes')


def _mission_date(name):
    """'Mission_26-10-2026_15-11-21' -> '2026-10-26' (None if the name is different)."""
    try:
        d, m, y = name.split('_')[1].split('-')
        return f"{y}-{m}-{d}"
    except Exception:
        return None


def load_segments(folder, missions=None, since=None):
    rows = []
    for path in sorted(glob.glob(os.path.join(folder, '*', 'segments.csv'))):
        name = os.path.basename(os.path.dirname(path))
        if missions and name not in missions:
            continue
        if since and (_mission_date(name) or '') < since:
            continue
        with open(path, newline='', encoding='utf-8') as f:
            for r in csv.DictReader(f):
                r['_mission'] = name
                rows.append(r)
    if not rows:
        return None
    col = lambda k: np.array([_num(r.get(k)) for r in rows])
    return {
        'mission': np.array([r['_mission'] for r in rows]),
        'tier': np.array([modern(r.get('tier', '')) for r in rows]),
        'success': np.array([_bool(r.get('success')) for r in rows]),
        'override': np.array([_bool(r.get('gait_override')) for r in rows]),
        'dist': col('dist_m'),
        'dist_trav': col('dist_traveled_m'),
        'dist_cmd': col('dist_commanded_m'),
        'duration': col('duration_s'),
        'long': np.abs(col('long_slope')),
        'lat': np.abs(col('lat_slope')),
        'rough': col('roughness'),
        'pitch': col('pitch_peak_abs_deg'),
        'roll': col('roll_peak_abs_deg'),
    }


# -----------------------------------------------------------------------------------------
# Tools
# -----------------------------------------------------------------------------------------
def regression(X, y):
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    ss_tot = float(((y - y.mean()) ** 2).sum())
    r2 = 1.0 - float(((y - pred) ** 2).sum()) / ss_tot if ss_tot > 0 else float('nan')
    return coef, r2


def bootstrap_ratios(X, y, idx_num, idx_den=0, n=300, seed=0):
    """90% interval of coef[idx_num]/coef[idx_den], resampling the rows."""
    rng = np.random.default_rng(seed)
    out = {i: [] for i in idx_num}
    for _ in range(n):
        s = rng.integers(0, len(y), len(y))
        try:
            c, *_ = np.linalg.lstsq(X[s], y[s], rcond=None)
        except np.linalg.LinAlgError:
            continue
        if c[idx_den] <= 0:
            continue
        for i in idx_num:
            out[i].append(c[i] / c[idx_den])
    return {i: (np.percentile(v, 5), np.percentile(v, 95)) if v else (np.nan, np.nan)
            for i, v in out.items()}


def title(t):
    print("\n" + "=" * 92 + f"\n{t}\n" + "=" * 92)


# -----------------------------------------------------------------------------------------
# Sections
# -----------------------------------------------------------------------------------------
def section_levels(S):
    title("1. LEGS PER GAIT LEVEL")
    v = S['dist_trav'] / S['duration']
    print(f"{'level':9s} {'legs':>6s} {'success':>9s} {'pitch.med':>9s} {'pitch.p95':>9s} "
          f"{'roll.med':>10s} {'speed.med':>9s} {'limit':>7s}")
    for t in TIERS:
        m = S['tier'] == t
        if not m.any():
            continue
        ok = m & S['success'] & (S['duration'] > 0)
        print(f"{t:9s} {m.sum():6d} {S['success'][m].mean():9.0%} {np.nanmedian(S['pitch'][m]):8.1f}° "
              f"{np.nanpercentile(S['pitch'][m], 95):8.1f}° {np.nanmedian(S['roll'][m]):9.1f}° "
              f"{np.nanmedian(v[ok]):8.2f}  {CAP_VEL[t]:6.2f}")
    if S['override'].any():
        print(f"(careful: {S['override'].sum()} legs with the gait forced, gait_override)")
    print("Speeds in m/s. If the real speed is well below the limit, the limit is not what slows "
          "the robot down.")


def section_cost(S):
    title("2. PRM COST: how much the slope really costs (regression of the time)")
    ok = S['success'] & (S['duration'] > 0) & (S['dist_trav'] > 0.05)
    for k in ('long', 'lat', 'rough'):
        ok &= np.isfinite(S[k])
    n = int(ok.sum())
    if n < 30:
        print(f"Only {n} usable successful legs: too few for a regression.")
        return
    d, lo, la, ro = S['dist_trav'][ok], S['long'][ok], S['lat'][ok], S['rough'][ok]
    y = S['duration'][ok]
    tier = S['tier'][ok]
    one = np.ones(n)

    X1 = np.c_[d, lo, la, ro, one]
    c1, r21 = regression(X1, y)
    ic1 = bootstrap_ratios(X1, y, [1, 2, 3])
    mod, hard = (tier == 'MODERATE').astype(float), (tier == 'HARD').astype(float)
    X2 = np.c_[d, lo, la, ro, mod, hard, one]
    c2, r22 = regression(X2, y)
    ic2 = bootstrap_ratios(X2, y, [1, 2, 3, 4, 5])

    print(f"{n} successful legs. Time per metre: {c1[0]:.2f} s/m (without level), "
          f"{c2[0]:.2f} s/m (with level).\n")
    print(f"{'weight (alpha = 1)':28s} {'without level':>22s} {'with level':>22s}")
    for name, i in (('beta  (longitudinal slope)', 1), ('beta_lat (lateral slope)', 2),
                    ('gamma (roughness)', 3)):
        print(f"{name:28s} {c1[i] / c1[0]:6.2f} [{ic1[i][0]:5.2f};{ic1[i][1]:5.2f}]  "
              f"{c2[i] / c2[0]:6.2f} [{ic2[i][0]:5.2f};{ic2[i][1]:5.2f}]")
    print(f"{'switch to MODERATE (m)':28s} {'':22s} {c2[4] / c2[0]:6.2f} [{ic2[4][0]:5.2f};{ic2[4][1]:5.2f}]")
    print(f"{'switch to HARD (m)':28s} {'':22s} {c2[5] / c2[0]:6.2f} [{ic2[5][0]:5.2f};{ic2[5][1]:5.2f}]")
    print(f"{'R² (share of time explained)':28s} {r21:22.2f} {r22:22.2f}")
    prm = roadmap.PRM()
    print(f"\nToday in the PRM: beta = {prm.beta:.1f}, beta_lat = {prm.beta_lateral:.1f}, gamma = {prm.gamma:g} "
          f"(roadmap.PRM, slopes.LATERAL_SLOPE_COST_MULTIPLIER = {slopes.LATERAL_SLOPE_COST_MULTIPLIER:g}).")
    rq = np.nanpercentile(S['rough'], [50, 95])
    print(f"Typical roughness: median {rq[0]:.3f}, 95th percentile {rq[1]:.3f}: gamma * {rq[1]:.3f} "
          f"is how much the roughness weighs on a rough edge; if it is much less than a metre, "
          f"gamma = 0 does not change the paths.")
    print("Reading: 'without level' is the total cost in time (slope + the CRAWL that follows); "
          "'with level' separates the two. The values in brackets are the 90% interval: if it "
          "contains today's value, the data do not ask to change it. Time is not the only "
          "criterion: a high beta_lat can stay as a safety margin on the roll.")


def _bands(name, x, failed, thresholds):
    print(f"  {name}:")
    for q in thresholds:
        below, above = x < q, x >= q
        if above.sum() == 0:
            print(f"     <{q:g}: {failed[below].mean():.0%} (n{below.sum()}) | >=: none")
            continue
        print(f"     <{q:g}: {failed[below].mean():.0%} (n{below.sum()}) | "
              f">=: {failed[above].mean():.0%} (n{above.sum()})")


def section_failures(S):
    title("3. FAILURES: do they depend on the terrain?")
    fail = ~S['success']
    still = fail & (np.nan_to_num(S['dist_trav']) < 0.05)
    print(f"Failed legs: {fail.sum()} out of {len(fail)} ({fail.mean():.1%}); of these {still.sum()} "
          f"with 0 m travelled (refusals, blocks: it is not the terrain).")
    physical = fail & ~still
    print(f"Failures with the robot moving: {physical.sum()} ({physical.mean():.1%} of the legs). "
          f"Rate per band:")
    _bands("longitudinal slope", S['long'], physical, [0.1, 0.2, 0.3])
    _bands("lateral slope", S['lat'], physical, [0.05, 0.1, 0.2])
    _bands("roughness", S['rough'], physical, [0.02, 0.04])
    print("If the rate above a threshold is not clearly higher than below, the risk does not "
          "belong in the slope weights (the penalties on the edges already handle it).")


def section_thresholds(S, pitch_deg):
    title(f"4. GAIT THRESHOLDS ('inclined' leg = peak pitch > {pitch_deg:g}°)")
    ok = np.isfinite(S['pitch']) & np.isfinite(S['long'])
    lab, s = S['pitch'][ok] > pitch_deg, S['long'][ok]
    if lab.sum() == 0 or (~lab).sum() == 0:
        print("Both inclined and calm legs are needed.")
        return
    print(f"{lab.sum()} inclined legs out of {ok.sum()}.")
    print(f"{'threshold':>9s} {'inclined caught':>16s} {'false alarms':>13s} {'legs above':>11s}")
    for t in (0.10, 0.125, 0.15, 0.175, 0.20, 0.225, 0.25, 0.30):
        note = ""
        if abs(t - gait.GAIT_PLAIN_SLOPE_MAX) < 1e-9:
            note = "  <- today PLAIN -> MODERATE"
        elif abs(t - gait.GAIT_MODERATE_SLOPE_MAX) < 1e-9:
            note = "  <- today MODERATE -> HARD"
        print(f"{t:9.3f} {(s[lab] >= t).mean():16.0%} {(s[~lab] >= t).mean():13.0%} "
              f"{(s >= t).mean():11.0%}{note}")
    print("'inclined caught': of the inclined legs, how many would be above the threshold (higher "
          "is better for the HARD threshold). 'false alarms': of the calm legs, how many are above "
          "it for nothing (each one is a useless leg in CRAWL).")
    r = S['rough'][np.isfinite(S['rough'])]
    for name, val in (('GAIT_PLAIN_ROUGH_MAX', gait.GAIT_PLAIN_ROUGH_MAX),
                      ('GAIT_MODERATE_ROUGH_MAX', gait.GAIT_MODERATE_ROUGH_MAX)):
        print(f"{name} = {val:g}: exceeded in {(r >= val).sum()} legs out of {len(r)} "
              f"({(r >= val).mean():.1%}).")
    print(f"Measured roughness: median {np.median(r):.3f}, 95th percentile {np.percentile(r, 95):.3f}, "
          f"maximum {r.max():.3f}. A threshold that never triggers decides nothing.")


def section_trials(path):
    title(f"5. TRACK TRIALS ({path})")
    if not os.path.exists(path):
        print("No file: it is created by tools/gait_trial.py.")
        return
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("Empty file.")
        return
    groups = {}
    for r in rows:
        groups.setdefault((r['terrain'], r['direction'], r['config']), []).append(r)
    print(f"{'terrain':14s} {'dir':7s} {'config':14s} {'n':>2s} {'succ':>5s} {'time':>6s} "
          f"{'speed':>5s} {'pitchRMS':>8s} {'rollRMS':>7s} {'slip':>6s} {'mu_est':>6s} {'J/m':>5s} "
          f"{'gap':>6s}")
    for (ter, direction, cfg), rr in sorted(groups.items()):
        if cfg == 'estimate_only':
            continue
        g = lambda k: np.array([_num(r.get(k)) for r in rr])
        ok = np.array([_bool(r.get('success')) for r in rr])
        dist = g('dist_traveled_m')
        nm = lambda a: np.nanmean(a) if np.isfinite(a).any() else float('nan')
        gap = np.abs(g('dist_commanded_m') - dist)
        print(f"{ter[:14]:14s} {direction[:7]:7s} {cfg[:14]:14s} {len(rr):2d} {ok.mean():5.0%} "
              f"{nm(g('duration_s')):6.1f} {nm(g('speed_mps')):5.2f} {nm(g('pitch_rms_deg')):8.2f} "
              f"{nm(g('roll_rms_deg')):7.2f} {nm(g('slip_peak_m')):6.3f} {nm(g('mu_est_mean')):6.2f} "
              f"{nm(g('energy_per_m_j')):5.0f} {nm(gap):6.2f}")
    print("time in s, speed in m/s, RMS in degrees around the mean of the leg (oscillation, not "
          "slope), slip = peak slip of a foot in contact (m), mu_est = friction estimated by Spot "
          "under the feet, J/m = energy per metre, gap = |commanded - travelled| m.")
    print("For every terrain and direction the best configuration is the one that always succeeds "
          "with low oscillation and slip; with equal results, the fastest or the least energy-hungry.")

    # Estimated slope against the inclinometer and the legs
    pairs = []
    for r in rows:
        true_deg = _num(r.get('true_slope_deg'))
        est = _num(r.get('est_long'))
        if np.isfinite(true_deg) and np.isfinite(est):
            # The estimate is always positive (90th percentile of the magnitude); on the way back
            # the true slope is negative (descent): magnitudes are compared, the pitch keeps its sign.
            pairs.append((f"{r['terrain']} {r.get('direction', '')}", abs(true_deg),
                          float(np.degrees(np.arctan(est))),
                          abs(_num(r.get('est_plane_long_deg'))), abs(_num(r.get('gpe_long_mean_deg'))),
                          _num(r.get('pitch_mean_deg'))))
    if pairs:
        print("\nSlope estimated from the scan against the measured one (inclinometer; legs = gpe):")
        print(f"{'terrain and direction':22s} {'true':>6s} {'edges':>7s} {'plane':>7s} {'legs':>6s} {'mean pitch':>11s}")
        per = {}
        for ter, *vals in pairs:
            per.setdefault(ter, []).append(vals)
        for ter, v in sorted(per.items()):
            a = np.array(v, dtype=float)
            m = lambda k: np.nanmean(a[:, k]) if np.isfinite(a[:, k]).any() else float('nan')
            print(f"{ter[:22]:22s} {m(0):5.1f}° {m(1):6.1f}° {m(2):6.1f}° {m(3):5.1f}° {m(4):10.1f}°")
        print("edges = today's estimate of the PRM (with the body, 90th percentile, usually a bit above "
              "the mean slope); plane = local plane on a body-long window (slopes.PLANE_SLOPES). A "
              "few degrees move the chosen level a lot (0.15 = 8.5°, 0.25 = 14°).")


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--folder', default=os.path.join(ROOT, 'MissionLogs'))
    p.add_argument('--missions', nargs='*', default=None)
    p.add_argument('--since', default=None, help="YYYY-MM-DD: only the missions from that day on")
    p.add_argument('--pitch', type=float, default=12.0,
                   help="peak pitch in degrees above which a leg is 'inclined'")
    p.add_argument('--trials', default=os.path.join('GaitTrials', 'trials.csv'))
    a = p.parse_args()

    S = load_segments(a.folder, a.missions, a.since)
    if S is None:
        print(f"No segments.csv in {a.folder}/ with these filters.")
    else:
        print(f"{len(S['tier'])} legs from {len(set(S['mission']))} missions.")
        section_levels(S)
        section_cost(S)
        section_failures(S)
        section_thresholds(S, a.pitch)
    section_trials(a.trials)


if __name__ == '__main__':
    sys.exit(main())
