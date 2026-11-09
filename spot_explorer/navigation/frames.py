"""
Frames of map navigation.

  MissionFrame   origin at the mission start, x ahead, y to the left (like the cells)
  Transform2D    rigid planar transform from the mission's VISION frame to today's: the
                 VISION frame of the day of the mission no longer exists after a reboot, and
                 GraphNav gives the link between the two (see navigator.MapNavigator)
"""
import numpy as np


def yaw_of_quat(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))


def wrap(a):
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


class Transform2D:
    """Planar rigid transform: p_new = R(th) p_old + t. Identity by default."""

    def __init__(self, tx=0.0, ty=0.0, th=0.0):
        self.tx, self.ty, self.th = float(tx), float(ty), float(th)

    @staticmethod
    def from_poses(new_pose, old_pose):
        """The transform that takes old_pose (x, y, yaw) to new_pose: the same physical body
        pose seen in the two frames."""
        xn, yn, an = new_pose
        xo, yo, ao = old_pose
        th = wrap(an - ao)
        c, s = np.cos(th), np.sin(th)
        return Transform2D(xn - (c * xo - s * yo), yn - (s * xo + c * yo), th)

    def apply(self, x, y):
        c, s = np.cos(self.th), np.sin(self.th)
        return float(self.tx + c * x - s * y), float(self.ty + s * x + c * y)

    def inverse(self, x, y):
        c, s = np.cos(self.th), np.sin(self.th)
        dx, dy = x - self.tx, y - self.ty
        return float(c * dx + s * dy), float(-s * dx + c * dy)

    def apply_many(self, x, y):
        """apply() on arrays."""
        c, s = np.cos(self.th), np.sin(self.th)
        x, y = np.asarray(x, dtype=np.float64), np.asarray(y, dtype=np.float64)
        return self.tx + c * x - s * y, self.ty + s * x + c * y

    def inverse_many(self, x, y):
        """inverse() on arrays."""
        c, s = np.cos(self.th), np.sin(self.th)
        dx, dy = np.asarray(x, dtype=np.float64) - self.tx, np.asarray(y, dtype=np.float64) - self.ty
        return c * dx + s * dy, -s * dx + c * dy

    def apply_yaw(self, yaw):
        return wrap(yaw + self.th)

    def difference(self, other):
        """How far a point near the origin moves between the two estimates (m) and how much the
        angle changes (degrees): for the log, to see whether the estimate jumps."""
        return (float(np.hypot(self.tx - other.tx, self.ty - other.ty)),
                float(np.degrees(abs(wrap(self.th - other.th)))))

    def __repr__(self):
        return f"(tx {self.tx:+.2f} m, ty {self.ty:+.2f} m, rotation {np.degrees(self.th):+.1f} degrees)"


class MissionFrame:
    """Mission frame: origin at the start, x ahead, y to the left (like the cells)."""

    def __init__(self, ox, oy, yaw):
        self.ox, self.oy, self.yaw = float(ox), float(oy), float(yaw)

    def to_vision(self, xm, ym):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        return self.ox + c * xm - s * ym, self.oy + s * xm + c * ym

    def from_vision(self, x, y):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        dx, dy = x - self.ox, y - self.oy
        return c * dx + s * dy, -s * dx + c * dy
