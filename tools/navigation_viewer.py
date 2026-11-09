#!/usr/bin/env python3
"""
Live viewer of a map navigation (run_navigation.py), in a second terminal.

    python3 tools/navigation_viewer.py                                  # the latest navigation, live
    python3 tools/navigation_viewer.py MissionLogs/Navigation_<date>    # a specific one
    python3 tools/navigation_viewer.py <folder> --png figure.png        # one figure, no window

Updated map, path walked, planned path, goal, robot, home (wp_0); in the title phase, battery,
last decision and how long since the last update. It does not touch the robot and needs no
SDK: it reads navigation_info.json, navigation_state.json, navigation_map.npz and
navigation.csv from the log folder, and redraws when they change. It also works on a finished
navigation (the last state). Keys: q quits.
"""
import argparse
import csv
import glob
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)                                     # the spot_explorer package
from spot_explorer.navigation.frames import MissionFrame                       # noqa: E402
from spot_explorer.navigation.mission_map import (LOG_TAG, draw_raster,        # noqa: E402
                                                  load_occupancy_map, mission_raster)


def latest_navigation():
    dirs = [d for d in glob.glob(os.path.join(HERE, 'MissionLogs', 'Navigation_*'))
            if os.path.exists(os.path.join(d, 'navigation_info.json'))]
    return max(dirs, key=os.path.getmtime) if dirs else None


class NavigationViewer:
    def __init__(self, folder):
        self.dir = folder
        self.info = json.load(open(os.path.join(folder, 'navigation_info.json')))
        self.frame = MissionFrame(*self.info['frame'])
        self._mt = {}
        self._img = None

    def _changed(self, name):
        path = os.path.join(self.dir, name)
        try:
            mt = os.path.getmtime(path)
        except OSError:
            return False
        if self._mt.get(name) == mt:
            return False
        self._mt[name] = mt
        return True

    def _image(self):
        """Map image: the one updated by the navigation, if there is one; before that, the map
        of the mission (slower to read: 20 MB)."""
        if self._changed('navigation_map.npz') or self._img is None:
            path = os.path.join(self.dir, 'navigation_map.npz')
            if os.path.exists(path):
                d = np.load(path)
                self._img = (d['img'], tuple(float(v) for v in d['ext']))
            else:
                pkl = os.path.join(HERE, 'MissionMap', f"Mission_{self.info['mission']}",
                                   'global_occupancy_map.pkl')
                if os.path.exists(pkl):
                    self._img = mission_raster(load_occupancy_map(pkl), self.frame)
        return self._img

    def _state(self):
        try:
            return json.load(open(os.path.join(self.dir, 'navigation_state.json')))
        except Exception:
            return {}

    def _trail(self):
        path = os.path.join(self.dir, 'navigation.csv')
        if not os.path.exists(path):
            return np.zeros((0, 2))
        with open(path) as f:
            rows = [(float(r['x_m']), float(r['y_m'])) for r in csv.DictReader(f) if r.get('x_m')]
        return np.array(rows) if rows else np.zeros((0, 2))

    def draw(self, ax):
        ax.clear()
        img = self._image()
        if img is not None:
            draw_raster(ax, *img)
        st = self._state()
        w = np.array(self.info.get('waypoints') or [])
        if w.size:
            ax.scatter(w[:, 1], w[:, 0], s=6, c='#2b6cb0', zorder=4, label='GraphNav waypoints')
        if self.info.get('wp0'):
            ax.plot(self.info['wp0'][1], self.info['wp0'][0], 'H', c='#6b46c1', ms=12, zorder=6,
                    label='home (wp_0)')
        tr = self._trail()
        if len(tr):
            ax.plot(tr[:, 1], tr[:, 0], '-', c='#1f4e9c', lw=1.5, zorder=5, label='path walked')
        if st.get('path'):
            p = np.array(st['path'])
            ax.plot(p[:, 1], p[:, 0], '-', c='#ff8c00', lw=2.5, zorder=5, label='planned path')
        if st.get('goal'):
            g = st['goal']
            ax.plot(g[1], g[0], '*', c='#ffd400', mec='k', ms=18, zorder=7, label='goal')
        if st.get('robot'):
            x, y, yaw = st['robot']
            ax.annotate('', xy=(y + 0.6 * np.sin(yaw), x + 0.6 * np.cos(yaw)), xytext=(y, x),
                        arrowprops=dict(arrowstyle='-|>', color='k', lw=2), zorder=8)
            ax.plot(y, x, 'o', c='k', ms=7, zorder=8, label='robot')
        if img is not None:
            ext = img[1]
            ax.set_xlim(ext[1] + 0.5, ext[0] - 0.5)      # inverted: the robot's left on the left
            ax.set_ylim(ext[2] - 0.5, ext[3] + 0.5)
        ax.set_aspect('equal')
        ax.set_xlabel("y: to the left of the start [m]")
        ax.set_ylabel("x: ahead of the start [m]")
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        age = time.time() - st['t'] if st.get('t') else None
        g = st.get('goal')
        line1 = (f"{st.get('phase', '?')}" + (f" towards {g[2]}" if g else "")
                 + (f" | plan {st.get('plan')} round {st.get('round')}" if st.get('round') else ""))
        line2 = " | ".join(t for t in (
            f"battery {st['battery_pct']:.0f}%" if st.get('battery_pct') is not None else "",
            (f"decision {st.get('decision')}" + (f" ({st['frontier']})" if st.get('frontier') else ""))
            if st.get('decision') else "",
            "GraphNav NOT localized" if st.get('lost') else "",
            f"updated {age:.0f} s ago" if age is not None else "") if t)
        ax.set_title(f"Map navigation: mission {self.info['mission']}\n{line1}\n{line2}", fontsize=10)

    def png(self, path):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 10))
        self.draw(ax)
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        print(f"{LOG_TAG} Figure saved to {path}")

    def live(self, period_s=1.0):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 10))
        state = {'quit': False}
        fig.canvas.mpl_connect('key_press_event', lambda ev: state.update(quit=ev.key in ('q', 'escape')))
        fig.canvas.mpl_connect('close_event', lambda ev: state.update(quit=True))
        self.draw(ax)
        fig.tight_layout()
        while not state['quit']:
            if any([self._changed(n) for n in ('navigation_state.json', 'navigation.csv')]):
                self.draw(ax)
                fig.canvas.draw_idle()
            plt.pause(period_s)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Live viewer of a map navigation (run_navigation.py).")
    ap.add_argument('folder', nargs='?', help="MissionLogs/Navigation_<date>; default the latest")
    ap.add_argument('--png', help="save one figure and exit")
    args = ap.parse_args(argv)
    folder = args.folder or latest_navigation()
    if not folder or not os.path.exists(os.path.join(folder, 'navigation_info.json')):
        print(f"{LOG_TAG} No navigation to view (MissionLogs/Navigation_<date>/navigation_info.json is needed).")
        return False
    viewer = NavigationViewer(folder)
    if args.png:
        viewer.png(args.png)
        return True
    from spot_explorer.navigation.goal_picker import window_available
    if not window_available("the viewer"):
        return False
    viewer.live()
    return True


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
