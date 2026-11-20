"""
KinematicWorld: a SimBackend without physics, for the bridge and the scenes of sim/worlds.py.

The ground is a sum of shapes (a plane with a slope, Gaussian hills, steps); obstacles are
boxes with a height. The robot follows the path exactly in time, with the speed limits of
the command, and stops (path 'blocked') before entering an obstacle or a slope steeper than
MAX_SLOPE_DEG along its way. Body roll and pitch follow the ground under the robot.
"""
import numpy as np

from sim.backend import SimBackend

BODY_Z = 0.50              # body above the ground, m
STEP_S = 0.02              # integration step, s
MAX_SLOPE_DEG = 30.0       # steeper than this along the way: blocked
BLOCK_MARGIN_M = 0.10      # the body centre this close to an obstacle: blocked (as tests/fake_spot)
NEAR_GOAL_M, AT_GOAL_M, AT_GOAL_RAD = 0.30, 0.03, np.radians(3.0)


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


class KinematicWorld(SimBackend):
    """
    boxes:   [(x0, x1, y0, y1, height), ...] obstacles
    ground:  [('plane', slope_x, slope_y), ('hill', cx, cy, height, radius),
              ('step', x_from, height) -- the ground rises by height for x > x_from, ...]
    start:   (x, y, yaw)
    """

    def __init__(self, boxes=(), ground=(), start=(0.0, 0.0, 0.0), size=(30.0, 30.0)):
        self.boxes = [tuple(map(float, b)) for b in boxes]
        self.ground = list(ground)
        self.size = size
        self.x, self.y, self.yaw = map(float, start)
        self.t = 0.0
        self._path = None                 # (points, max_linear, max_angular)
        self._status = 'stopped'
        self.track = [(self.x, self.y)]

    # --- world ----------------------------------------------------------------------------
    def _ground(self, xs, ys):
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        z = np.zeros(np.broadcast(xs, ys).shape)
        for g in self.ground:
            if g[0] == 'plane':
                z = z + g[1] * xs + g[2] * ys
            elif g[0] == 'hill':
                _, cx, cy, h, r = g
                z = z + h * np.exp(-((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * r * r))
            elif g[0] == 'step':
                z = z + np.where(xs > g[1], g[2], 0.0)
        return z

    def _box_mask(self, xs, ys):
        xs, ys = np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
        m = np.zeros(np.broadcast(xs, ys).shape, dtype=bool)
        top = np.zeros(m.shape)
        for x0, x1, y0, y1, h in self.boxes:
            inside = (xs >= x0) & (xs <= x1) & (ys >= y0) & (ys <= y1)
            m |= inside
            top = np.where(inside, np.maximum(top, h), top)
        return m, top

    def terrain(self, xs, ys):
        m, top = self._box_mask(xs, ys)
        return self._ground(xs, ys) + np.where(m, top, 0.0)

    def obstacles(self, xs, ys):
        return self._box_mask(xs, ys)[0]

    def _blocked(self, x, y, nx, ny):
        if any(x0 - BLOCK_MARGIN_M <= nx <= x1 + BLOCK_MARGIN_M and y0 - BLOCK_MARGIN_M <= ny <= y1 + BLOCK_MARGIN_M
               for x0, x1, y0, y1, _h in self.boxes):
            return True
        d = float(np.hypot(nx - x, ny - y))
        if d < 1e-6:
            return False
        dz = float(self._ground(nx, ny) - self._ground(x, y))
        return np.degrees(np.arctan2(abs(dz), d)) > MAX_SLOPE_DEG

    # --- clock and robot --------------------------------------------------------------------
    def now(self):
        return self.t

    def step_to(self, t):
        while self.t < t - 1e-9:
            h = min(STEP_S, t - self.t)
            self.t += h
            self._step(h)

    def pose(self):
        z = float(self._ground(self.x, self.y)) + BODY_Z
        e = 0.25
        gx = float(self._ground(self.x + e, self.y) - self._ground(self.x - e, self.y)) / (2 * e)
        gy = float(self._ground(self.x, self.y + e) - self._ground(self.x, self.y - e)) / (2 * e)
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        pitch = -np.arctan(gx * c + gy * s)            # nose up on an uphill (body frame convention)
        roll = np.arctan(-gx * s + gy * c)
        return self.x, self.y, z, self.yaw, float(roll), float(pitch)

    def follow_path(self, points, max_linear, max_angular):
        self._path = ([tuple(map(float, p)) for p in points], float(max_linear), float(max_angular))
        self._status = 'moving'

    def stop(self):
        self._path = None
        self._status = 'stopped'

    def path_status(self):
        return self._status

    def teleport(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.track.append((self.x, self.y))
        self.stop()

    def _reference(self, points):
        if self.t <= points[0][3]:
            return points[0][:3]
        for a, b in zip(points, points[1:]):
            if self.t <= b[3]:
                f = (self.t - a[3]) / max(b[3] - a[3], 1e-9)
                return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * _wrap(b[2] - a[2])
        return points[-1][:3]

    def _step(self, h):
        if self._path is None or self._status in ('stopped', 'blocked', 'at_goal'):
            return
        points, v, w = self._path
        rx, ry, rh = self._reference(points)
        dx, dy = rx - self.x, ry - self.y
        d = float(np.hypot(dx, dy))
        k = min(1.0, v * h / d) if d > 1e-9 else 0.0
        nx, ny = self.x + k * dx, self.y + k * dy
        if self._blocked(self.x, self.y, nx, ny):
            self._status = 'blocked'
            return
        self.x, self.y = nx, ny
        self.yaw = _wrap(self.yaw + float(np.clip(_wrap(rh - self.yaw), -w * h, w * h)))
        self.track.append((self.x, self.y))
        gx, gy, gh, _ = points[-1]
        dist = float(np.hypot(gx - self.x, gy - self.y))
        if dist < AT_GOAL_M and abs(_wrap(gh - self.yaw)) < AT_GOAL_RAD:
            self._status = 'at_goal'
        elif dist < NEAR_GOAL_M:
            self._status = 'near_goal'
        else:
            self._status = 'moving'
