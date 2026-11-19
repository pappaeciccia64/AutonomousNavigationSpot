#!/usr/bin/env python3
"""
The terrain of a mission in 3D, with the path the robot walked on it.

Height: the global terrain map the mission saves (MissionMap/Mission_<m>/global_terrain_map.pkl,
mean height per cell over all scans) minus the ground at the mission start. The cells are
grouped BLOCK x BLOCK with the median (a 5 cm surface is too heavy and noisy to read), then
isolated spikes are removed: cells more than SPIKE_M from the median of their neighbours
(edges of bushes). Holes are obstacles: Spot sees the obstacle there, not the ground.
Path: MissionLogs/Mission_<m>/trajectory.csv, laid on the surface. Height exaggerated by --zscale.

Usage: python3 tools/terrain_3d.py 28-10-2026_13-50-46 [--out terrain.png] [--zscale 4]
"""
import argparse
import csv
import glob
import pickle

import numpy as np

BLOCK = 4
SPIKE_M = 0.15


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('mission', help='e.g. 28-10-2026_13-50-46')
    ap.add_argument('--out', help='PNG to save (default: show the window)')
    ap.add_argument('--zscale', type=float, default=4.0, help='height exaggeration (default 4)')
    ap.add_argument('--elev', type=float, default=38.0, help='view elevation in degrees')
    ap.add_argument('--azim', type=float, default=-125.0, help='view azimuth in degrees')
    ap.add_argument('--title', help='figure title (default: mission and what is shown)')
    ap.add_argument('--path-label', default='robot path', help='legend label of the path')
    args = ap.parse_args()

    import matplotlib
    if args.out:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from scipy.ndimage import generic_filter

    folder = f'MissionMap/Mission_{args.mission}'
    tm = pickle.load(open(f'{folder}/global_terrain_map.pkl', 'rb'))
    res = tm['resolution']
    z0 = tm.get('mission_z0')
    if z0 is None:
        vis = sorted(glob.glob(f'{folder}/*FINAL_SCAN_vis.npz'))
        z0 = float(np.load(vis[0], allow_pickle=True)['mission_z0']) if vis else 0.0
    keys = np.array(list(tm['height_sum'].keys()))
    vals = np.array([tm['height_sum'][tuple(k)] / tm['height_count'][tuple(k)] for k in keys]) - z0
    gx0, gy0 = keys.min(0)
    H = np.full(tuple(keys.max(0) - keys.min(0) + 1), np.nan)
    H[keys[:, 0] - gx0, keys[:, 1] - gy0] = vals

    b = BLOCK
    nx, ny = H.shape[0] // b, H.shape[1] // b
    blocks = H[:nx * b, :ny * b].reshape(nx, b, ny, b).transpose(0, 2, 1, 3).reshape(nx, ny, -1)
    with np.errstate(all='ignore'):
        import warnings
        warnings.simplefilter('ignore', RuntimeWarning)
        Z = np.nanmedian(blocks, axis=2)
        med = generic_filter(Z, np.nanmedian, size=5, mode='nearest')
    Z[np.abs(Z - med) > SPIKE_M] = np.nan
    X, Y = np.meshgrid((gx0 + np.arange(nx) * b + b / 2) * res, (gy0 + np.arange(ny) * b + b / 2) * res,
                       indexing='ij')

    T = np.array([(float(r['x']), float(r['y']))
                  for r in csv.DictReader(open(f'MissionLogs/Mission_{args.mission}/trajectory.csv'))])
    ix = np.clip(((T[:, 0] / res - gx0) / b).astype(int), 0, nx - 1)
    iy = np.clip(((T[:, 1] / res - gy0) / b).astype(int), 0, ny - 1)
    tz = med[ix, iy]
    ok = np.isfinite(tz)

    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(111, projection='3d', computed_zorder=False)   # path drawn over the surface
    lo, hi = np.nanpercentile(Z, 1), np.nanpercentile(Z, 99)
    surf = ax.plot_surface(X, Y, Z, cmap='terrain', vmin=lo - 0.15, vmax=hi + 0.15,
                           rstride=1, cstride=1, linewidth=0, antialiased=True, alpha=0.95, zorder=1)
    ax.plot(T[ok, 0], T[ok, 1], tz[ok] + 0.08, color='red', lw=1.6, label=args.path_label, zorder=10)
    ax.set_box_aspect((np.ptp(X), np.ptp(Y), args.zscale * max(np.nanmax(Z) - np.nanmin(Z), 0.1)))
    ax.view_init(elev=args.elev, azim=args.azim)
    ax.set_xlabel('X [m] (VISION)'); ax.set_ylabel('Y [m] (VISION)'); ax.set_zlabel('height [m]')
    fig.colorbar(surf, ax=ax, shrink=0.55, label='height relative to the mission start (m)')
    ax.set_title(args.title or f'{args.mission}: terrain rebuilt by the robot (height x{args.zscale:g}) and the path walked\n'
                 '(holes are obstacles: no ground height there)')
    ax.legend(loc='upper left')
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=100)
        print('saved', args.out)
    else:
        plt.show()


if __name__ == '__main__':
    main()
