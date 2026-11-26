#!/usr/bin/env python3
"""
LIVE viewer of the simulated sensors: what Spot's cameras, lidar and IMU see in Isaac Sim,
in one figure that updates itself. It runs next to the simulator, with or without a mission:

    python3 tools/sim_sensor_viewer.py                    # simulator on 127.0.0.1:8765
    python3 tools/sim_sensor_viewer.py --sim HOST:PORT
    python3 tools/sim_sensor_viewer.py --png sensors.png  # one figure to a file, no window

It only reads: it never sends a command, so it does not disturb the mission. No SDK needed
(numpy, matplotlib and Pillow). For the mission itself there is tools/live_viewer.py.

In the figure:
  top row     the five cameras, colour, in the order they look around the robot
              (left, front-left, front-right, right, back)
  second row  the depth of the same cameras: near = yellow, far = dark blue, grey = nothing
              within range
  bottom left the lidar seen from above, in the world frame: every point coloured by its
              height above the ground (the ground itself is left out), the obstacles of the
              world file as outlines, the robot as an arrow with the path walked so far.
              Points and outlines agree where a trunk stands; leaves above have points only.
  bottom right the IMU over the last 20 s (specific force and angular rate, body frame),
              and the numbers: pose, speed, simulated time, real-time factor.

Keys: q quit, space pause.
"""
import argparse
import io
import math
import os
import sys
import time

import numpy as np
import matplotlib

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # project folder
sys.path.insert(0, HERE)
from spot_explorer.sim.lidar import scan_to_world                          # noqa: E402
from spot_explorer.sim.link import SimLink, SimLinkError, parse_address   # noqa: E402

CAMERA_ORDER = ('left', 'frontleft', 'frontright', 'right', 'back')
LIDAR_RANGE_M = 12.0            # half side of the lidar panel
GROUND_BAND_M = 0.12            # lidar points lower than this above the ground are not drawn
IMU_WINDOW_S = 20.0
DEPTH_SHOWN_M = 8.0             # the colour scale of the depth pictures ends here


def q_rotate(q, v):
    """Rotate the points v (n, 3) by the quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return v @ r.T


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def lidar_in_world(scan):
    """The scan (lidar frame) in the world frame, from the body pose at the capture."""
    return scan_to_world(scan)


class Viewer:
    def __init__(self, link, to_file=False):
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
        self.plt = plt
        self.link = link
        self.world = link.world()
        self.ground_z = float(self.world.get('ground_z', 0.0))
        self.sensors = link.sensors()
        names = [c['name'] for c in self.sensors.get('cameras', ())]
        self.cameras = [n for n in CAMERA_ORDER if n in names] + [n for n in names if n not in CAMERA_ORDER]
        self.paused = False
        self.trail = []
        self.imu_t, self.imu_a, self.imu_g = [], [], []
        self.rate_ref = None
        self.rtf = None

        self.fig = plt.figure(figsize=(15.5, 9.5))
        self.fig.canvas.manager.set_window_title('Simulated sensors') if not to_file else None
        n = max(len(self.cameras), 1)
        gs = GridSpec(4, n * 2, figure=self.fig, height_ratios=[1.0, 1.0, 1.15, 1.15],
                      left=0.035, right=0.99, top=0.955, bottom=0.05, hspace=0.32, wspace=0.25)
        self.ax_rgb, self.ax_depth, self.im_rgb, self.im_depth = [], [], [], []
        for i, name in enumerate(self.cameras or ['(no cameras)']):
            a = self.fig.add_subplot(gs[0, 2 * i:2 * i + 2])
            a.set_title(name, fontsize=10)
            a.set_xticks([]), a.set_yticks([])
            self.ax_rgb.append(a)
            self.im_rgb.append(None)
            d = self.fig.add_subplot(gs[1, 2 * i:2 * i + 2])
            d.set_title(name + ' depth (m)', fontsize=9)
            d.set_xticks([]), d.set_yticks([])
            self.ax_depth.append(d)
            self.im_depth.append(None)
        half = n
        self.ax_lidar = self.fig.add_subplot(gs[2:, 0:half])
        self.ax_acc = self.fig.add_subplot(gs[2, half + 1:2 * n - 2])
        self.ax_gyro = self.fig.add_subplot(gs[3, half + 1:2 * n - 2])
        self.ax_text = self.fig.add_subplot(gs[2:, 2 * n - 2:])
        self.ax_text.axis('off')
        self.text = self.ax_text.text(0.0, 1.0, '', va='top', ha='left', family='monospace', fontsize=9)
        self.lidar_scatter = None
        self.fig.canvas.mpl_connect('key_press_event', self._key)
        self._lidar_static()

    def _key(self, event):
        if event.key == 'q':
            self.plt.close(self.fig)
        elif event.key == ' ':
            self.paused = not self.paused

    def _lidar_static(self):
        import matplotlib.patches as patches
        ax = self.ax_lidar
        ax.set_aspect('equal')
        ax.set_facecolor('#f4f4f1')
        ax.set_xlabel('x ahead (m)'), ax.set_ylabel('y left (m)')
        for s in self.world.get('shapes', ()):
            if s['type'] == 'cylinder':
                ax.add_patch(patches.Circle((s['x'], s['y']), s['r'], fill=False, ec='black', lw=1.0, zorder=3))
            else:
                a = math.radians(s.get('yaw_deg', 0.0))
                c, sn = math.cos(a), math.sin(a)
                corners = [(sx * 0.5 * s['sx'], sy * 0.5 * s['sy']) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                ax.add_patch(patches.Polygon([(s['x'] + c * px - sn * py, s['y'] + sn * px + c * py)
                                              for px, py in corners], fill=False, ec='black', lw=1.0, zorder=3))
        self.trail_line, = ax.plot([], [], '-', color='tab:blue', lw=1.2, zorder=4)
        self.robot_arrow = None

    # --- one refresh -----------------------------------------------------------------------
    def refresh(self):
        from PIL import Image
        st = self.link.step()
        p, q = st['p'], st['q']
        yaw = yaw_of(q)
        wall = time.time()
        if self.rate_ref is not None and wall - self.rate_ref[0] > 2.0:
            self.rtf = (st['t'] - self.rate_ref[1]) / (wall - self.rate_ref[0])
            self.rate_ref = (wall, st['t'])
        elif self.rate_ref is None:
            self.rate_ref = (wall, st['t'])
        if not self.trail or math.hypot(p[0] - self.trail[-1][0], p[1] - self.trail[-1][1]) > 0.05:
            self.trail.append((p[0], p[1]))

        notes = []
        if self.cameras:
            try:
                items = self.link.images(self.cameras + [c + '_depth' for c in self.cameras])
                by_name = {it['name']: it for it in items}
                for i, name in enumerate(self.cameras):
                    rgb = np.asarray(Image.open(io.BytesIO(by_name[name]['jpeg'])))
                    it = by_name[name + '_depth']
                    depth = np.frombuffer(it['depth_mm'], dtype='<u2').reshape(it['h'], it['w']) / 1000.0
                    depth = np.ma.masked_where(depth <= 0, depth)
                    if self.im_rgb[i] is None:
                        self.im_rgb[i] = self.ax_rgb[i].imshow(rgb)
                        cmap = matplotlib.colormaps['viridis_r'].copy()
                        cmap.set_bad('#b0b0b0')
                        self.im_depth[i] = self.ax_depth[i].imshow(depth, cmap=cmap, vmin=0.0, vmax=DEPTH_SHOWN_M)
                    else:
                        self.im_rgb[i].set_data(rgb)
                        self.im_depth[i].set_data(depth)
            except SimLinkError as e:
                notes.append('cameras: %s' % e)
        else:
            notes.append('no cameras in this simulator')

        n_points, scan_note = 0, ''
        if self.sensors.get('lidar') is not None:
            try:
                scan = self.link.lidar()
                w = lidar_in_world(scan)
                height = w[:, 2] - self.ground_z
                keep = height > GROUND_BAND_M
                n_points = int(scan['n'])
                if scan['t'] != getattr(self, '_scan_t', None):
                    self._scan_t = scan['t']
                    self.scan_count = getattr(self, 'scan_count', 0) + 1
                scan_note = ', scan #%d, %.1f s old' % (self.scan_count, max(0.0, st['t'] - scan['t']))
                if self.lidar_scatter is None:
                    self.lidar_scatter = self.ax_lidar.scatter(w[keep, 0], w[keep, 1], c=height[keep], s=2.0,
                                                               cmap='plasma', vmin=0.0, vmax=3.0, zorder=2)
                    cb = self.fig.colorbar(self.lidar_scatter, ax=self.ax_lidar, fraction=0.035, pad=0.02)
                    cb.set_label('height above the ground (m)', fontsize=8)
                else:
                    self.lidar_scatter.set_offsets(w[keep, :2])
                    self.lidar_scatter.set_array(height[keep])
            except SimLinkError as e:
                notes.append('lidar: %s' % e)
        else:
            notes.append('no lidar in this simulator')
        ax = self.ax_lidar
        ax.set_xlim(p[0] - LIDAR_RANGE_M, p[0] + LIDAR_RANGE_M)
        ax.set_ylim(p[1] - LIDAR_RANGE_M, p[1] + LIDAR_RANGE_M)
        ax.set_title('lidar from above, %d points%s\n(outlines: obstacles of the world file)' % (n_points, scan_note),
                     fontsize=10)
        self.trail_line.set_data([t[0] for t in self.trail], [t[1] for t in self.trail])
        if self.robot_arrow is not None:
            self.robot_arrow.remove()
        self.robot_arrow = ax.annotate('', xy=(p[0] + 0.9 * math.cos(yaw), p[1] + 0.9 * math.sin(yaw)),
                                       xytext=(p[0] - 0.3 * math.cos(yaw), p[1] - 0.3 * math.sin(yaw)),
                                       arrowprops=dict(arrowstyle='-|>', color='tab:blue', lw=2.5), zorder=5)

        imu = st.get('imu')
        if imu is not None:
            try:                        # every reading since the last refresh (100 per second)
                rows = self.link.imu(self.imu_t[-1] if self.imu_t else -1.0)
                self.imu_t += rows['t']
                self.imu_a += rows['a']
                self.imu_g += rows['g']
            except SimLinkError:        # a simulator that only gives the latest reading
                self.imu_t.append(st['t'])
                self.imu_a.append(imu['a'])
                self.imu_g.append(imu['g'])
            while self.imu_t and self.imu_t[0] < self.imu_t[-1] - IMU_WINDOW_S:
                self.imu_t.pop(0), self.imu_a.pop(0), self.imu_g.pop(0)
            for ax_, data, title, unit in ((self.ax_acc, self.imu_a, 'IMU specific force', 'm/s2'),
                                           (self.ax_gyro, self.imu_g, 'IMU angular rate', 'rad/s')):
                ax_.clear()
                arr = np.asarray(data)
                for k, (label, color) in enumerate((('x', 'tab:red'), ('y', 'tab:green'), ('z', 'tab:blue'))):
                    ax_.plot(self.imu_t, arr[:, k], color=color, lw=0.7, label=label)
                ax_.set_title('%s (%s), body frame' % (title, unit), fontsize=9)
                ax_.legend(loc='upper left', fontsize=7, ncol=3)
                ax_.grid(alpha=0.3)
                ax_.tick_params(labelsize=8)
            self.ax_gyro.set_xlabel('simulated time (s)', fontsize=8)
        else:
            notes.append('no IMU in this simulator')

        v = st.get('v')
        lines = ['simulator   %s' % self.link.backend,
                 'world       %s' % (self.world.get('name') or 'empty'),
                 'time        %.1f s' % st['t'],
                 'real-time   %s' % ('%.2f' % self.rtf if self.rtf is not None else '...'),
                 '',
                 'x, y        %+.2f, %+.2f m' % (p[0], p[1]),
                 'heading     %+.0f deg' % math.degrees(yaw),
                 'body height %.2f m' % (p[2] - self.ground_z)]
        if v is not None:
            lines.append('speed       %.2f m/s' % math.hypot(v[0], v[1]))
        if imu is not None:
            lines += ['', 'force  %+.2f %+.2f %+.2f' % tuple(imu['a']), 'rate   %+.2f %+.2f %+.2f' % tuple(imu['g'])]
        if notes:
            lines += [''] + notes
        self.text.set_text('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(description="Live viewer of the simulated cameras, lidar and IMU")
    ap.add_argument('--sim', default=os.environ.get('SPOT_SIM', ''), metavar='HOST:PORT',
                    help="the simulator (default 127.0.0.1:8765, or the SPOT_SIM variable)")
    ap.add_argument('--hz', type=float, default=4.0, help="refreshes per second (default 4)")
    ap.add_argument('--png', default=None, metavar='FILE', help="save one figure to a file and exit")
    ap.add_argument('--settle', type=float, default=3.0,
                    help="with --png: seconds of readings before the figure (default 3)")
    args = ap.parse_args()
    if args.png:
        matplotlib.use('Agg')
    host, port = parse_address(args.sim)
    try:
        link = SimLink(host, port)
    except SimLinkError as e:
        sys.exit(str(e))
    viewer = Viewer(link, to_file=bool(args.png))
    period = 1.0 / max(args.hz, 0.2)
    if args.png:
        t_end = time.time() + max(args.settle, 0.0)
        while True:
            try:
                viewer.refresh()
            except SimLinkError as e:
                print('waiting: %s' % e)
            if time.time() >= t_end:
                break
            time.sleep(period)
        viewer.fig.savefig(args.png, dpi=110)
        print('Figure saved in %s' % os.path.abspath(args.png))
        return
    plt = viewer.plt
    plt.ion()
    plt.show()
    while plt.fignum_exists(viewer.fig.number):
        t0 = time.time()
        if not viewer.paused:
            try:
                viewer.refresh()
            except SimLinkError as e:
                viewer.text.set_text('link to the simulator lost:\n%s\n\n(close the window, or wait)' % e)
        viewer.fig.canvas.draw_idle()
        plt.pause(max(0.02, period - (time.time() - t0)))


if __name__ == '__main__':
    main()
