"""
A simulated spinning LiDAR (Velodyne VLP-16 by default) on the robot's back.

The scan is computed by casting rays against the world file (world.raycast): ground, boxes
and cylinders, from the true pose of the sensor, tilted with the body. It is what an ideal
sensor would return at one instant.

What it is NOT: no range noise unless asked, no intensity, no motion distortion (all the rays
are cast at the same instant, while a real Velodyne takes 0.1 s per turn), no robot body in
the way, no leaves or grass. Good for looking at what the sensor would see and for geometry
checks; a SLAM that relies on IMU and per-point timestamps needs more than this (see
sim/README_SIM.md, "LiDAR and SLAM").
Numpy only.
"""
import numpy as np

VLP16_ELEVATIONS_DEG = tuple(range(-15, 16, 2))     # 16 channels
MOUNT_XYZ = (0.0, 0.0, 0.22)                        # sensor in the body frame (m)


def quat_to_matrix(q):
    """Rotation matrix of a quaternion (w, x, y, z)."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def scan_to_world(scan):
    """A scan of the simulator's own lidar (link.lidar(): float32 x, y, z in the lidar frame,
    with the body pose and the mount at the capture) -> (n, 3) points in the world frame."""
    xyz = np.frombuffer(scan['xyz'], dtype='<f4').reshape(-1, 3).astype(np.float64)
    in_body = xyz @ quat_to_matrix(scan['mount_q']).T + np.asarray(scan['mount_p'], dtype=np.float64)
    return in_body @ quat_to_matrix(scan['q']).T + np.asarray(scan['p'], dtype=np.float64)


class Lidar:
    def __init__(self, elevations_deg=VLP16_ELEVATIONS_DEG, azimuth_step_deg=1.0,
                 max_range=30.0, min_range=0.4, mount_xyz=MOUNT_XYZ, noise_m=0.0, seed=0):
        el = np.radians(np.asarray(elevations_deg, dtype=np.float64))
        az = np.radians(np.arange(0.0, 360.0, float(azimuth_step_deg)))
        el_g, az_g = np.meshgrid(el, az, indexing='ij')
        self.channels, self.columns = el_g.shape
        # Unit vectors in the sensor frame (x ahead, y left, z up), ring by ring.
        self.dirs = np.stack([np.cos(el_g) * np.cos(az_g), np.cos(el_g) * np.sin(az_g),
                              np.sin(el_g)], axis=-1).reshape(-1, 3)
        self.ring = np.repeat(np.arange(self.channels), self.columns)
        self.max_range, self.min_range = float(max_range), float(min_range)
        self.mount = np.asarray(mount_xyz, dtype=np.float64)
        self.noise_m = float(noise_m)
        self._rng = np.random.RandomState(seed)

    def scan(self, world, position, quat_wxyz):
        """
        One turn from the body pose (position (3,), quaternion (w, x, y, z)).

        Returns a dict: points_sensor (M, 3) in the sensor frame, points_world (M, 3), ranges (M,),
        ring (M,) channel of each point, origin (3,) of the sensor in the world. Only the rays
        that hit something within range are returned.
        """
        rot = quat_to_matrix(quat_wxyz)
        origin = np.asarray(position, dtype=np.float64) + rot @ self.mount
        dirs_world = self.dirs @ rot.T
        r = world.raycast(origin, dirs_world, max_range=self.max_range)
        if self.noise_m > 0:
            r = r + self._rng.normal(0.0, self.noise_m, r.shape)
        ok = np.isfinite(r) & (r >= self.min_range) & (r <= self.max_range)
        r = r[ok]
        return {'points_sensor': self.dirs[ok] * r[:, None],
                'points_world': origin + dirs_world[ok] * r[:, None],
                'ranges': r, 'ring': self.ring[ok], 'origin': origin}
