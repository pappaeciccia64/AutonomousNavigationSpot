"""
The sensors of the simulated Spot in Isaac Sim: five body cameras (colour and depth), a
rotating lidar on the back, an IMU in the body. Imported by isaac_spot_server.py after the
simulation app has started.

    cameras   frontleft, frontright, left, right, back: the five of the real robot, here as
              ordinary wide-angle cameras (no fisheye distortion), colour and depth from the
              same viewpoint
    lidar     an Ouster OS1 with 32 beams, 10 turns per second, where Spot carries its lidar
              payload (the real robot has no lidar without that payload)
    imu       specific force and angular rate in the body frame, 100 readings per simulated
              second, the last few seconds kept for whoever asks

Frames. Body: x ahead, y left, z up, origin at the centre of the body. A camera "optical"
frame, as in the SDK images: z along the view, x to the right of the image, y down. The lidar
points are in the lidar frame, which is the body frame moved to the mount.

Nothing here talks to the mission: the server serves what this module holds (see the "sensors",
"image" and "lidar" requests in spot_explorer/sim/server.py).
"""
import base64
import collections
import io
import math
import threading
import time
import zlib

import numpy as np

# name: (position in the body frame, yaw and pitch-down of the view, degrees)
CAMERAS = {
    "frontleft": ((0.44, 0.045, 0.02), -14.0, 25.0),      # like the real one, it looks across
    "frontright": ((0.44, -0.045, 0.02), 14.0, 25.0),
    "left": ((-0.10, 0.13, 0.05), 90.0, 22.0),
    "right": ((-0.10, -0.13, 0.05), -90.0, 22.0),
    "back": ((-0.44, 0.0, 0.03), 180.0, 25.0),
}
CAMERA_HFOV_DEG = 110.0
CAMERA_NEAR_M = 0.03
CAMERA_FAR_M = 60.0
DEPTH_MAX_M = 30.0              # beyond this the depth image says "nothing" (0), like Spot's
LIDAR_MOUNT = (-0.20, 0.0, 0.20)
LIDAR_CONFIG, LIDAR_VARIANT = "OS1", "OS1_REV6_32ch10hz512res"
LIDAR_SOURCE = "velodyne-point-cloud"   # the name of the service on the real robot
IMU_MOUNT = (0.0, 0.0, 0.0)
IMU_HZ = 100.0
IMU_KEPT_S = 5.0                # readings kept for the "imu" request
DEMAND_S = 3.0                  # images are copied from the GPU only while somebody asks


def _quat_from_matrix(m):
    """Rotation matrix (columns = axes) -> quaternion (w, x, y, z)."""
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    return tuple(float(v) for v in q)


def _view_axes(yaw_deg, pitch_down_deg):
    """Right, down and forward axes of a camera in the body frame."""
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_down_deg)
    fwd = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    return right, down, fwd


def _to_numpy(a):
    if a is None:
        return None
    if hasattr(a, "numpy"):
        return np.asarray(a.numpy())
    return np.asarray(a)


class SensorRig:
    def __init__(self, body_path, cameras=True, lidar=True, imu=True, camera_hz=5.0,
                 resolution=(240, 320), show_lidar=False):
        self.body_path = body_path
        self.resolution = (int(resolution[0]), int(resolution[1]))       # (height, width)
        self.camera_period = 1.0 / max(float(camera_hz), 0.1)
        self._lock = threading.Lock()
        self._pose = None                    # latest body pose (p, q), set by the server
        self._frames = None                  # {'t', 'p', 'q', 'rgb': (N,H,W,3), 'depth': (N,H,W)}
        self._scan = None                    # {'t', 'p', 'q', 'xyz': (N,3) float32, 'intensity'}
        self._imu = None
        self._imu_log = collections.deque(maxlen=int(IMU_HZ * IMU_KEPT_S))
        self._imu_next_t = 0.0
        self._camera_due = 0.0
        self._demand_wall = time.time()      # the first frames are always taken
        self.camera_names = []
        self._tiled = None
        self._imu_sensor = None
        self._lidar_sensor = None
        self.notes = []
        if cameras:
            self._guard("cameras", self._make_cameras)
        if lidar:
            self._guard("lidar", lambda: self._make_lidar(show_lidar))
        if imu:
            self._guard("imu", self._make_imu)

    def _guard(self, what, fn):
        """A sensor that cannot be created must not stop the simulator."""
        try:
            fn()
        except Exception as error:  # noqa: BLE001
            self.notes.append(f"{what} not available: {type(error).__name__}: {error}")
            print(f"[SIM] Sensors: {self.notes[-1]}")

    # --- creation ------------------------------------------------------------------------------
    def _make_cameras(self):
        from isaacsim.sensors.experimental.rtx import RtxCamera, TiledCameraSensor
        from pxr import UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, self.body_path + "/sim_cameras")
        h, w = self.resolution
        aperture = 20.955
        focal = 0.5 * aperture / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
        self.intrinsics = {"fx": 0.5 * w / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)}
        self.intrinsics.update({"fy": self.intrinsics["fx"], "cx": 0.5 * w, "cy": 0.5 * h})
        self.camera_mounts = {}
        paths = []
        for name, (pos, yaw, pitch) in CAMERAS.items():
            right, down, fwd = _view_axes(yaw, pitch)
            # A USD camera looks along -z with +y up: columns right, up, back.
            q_usd = _quat_from_matrix(np.column_stack([right, -down, -fwd]))
            path = f"{self.body_path}/sim_cameras/{name}"
            # tick_rate: the renderer draws these pictures only as often as they are wanted
            RtxCamera(path, tick_rate=1.0 / self.camera_period, translations=np.array([pos], dtype=float),
                      orientations=np.array([q_usd], dtype=float))
            cam = UsdGeom.Camera(stage.GetPrimAtPath(path))
            cam.GetHorizontalApertureAttr().Set(aperture)
            cam.GetVerticalApertureAttr().Set(aperture * h / w)
            cam.GetFocalLengthAttr().Set(focal)
            cam.GetClippingRangeAttr().Set((CAMERA_NEAR_M, CAMERA_FAR_M))
            # The optical frame of the SDK images: columns right, down, forward.
            self.camera_mounts[name] = (tuple(pos), _quat_from_matrix(np.column_stack([right, down, fwd])))
            paths.append(path)
        self._tiled = TiledCameraSensor(paths, resolution=self.resolution,
                                        annotators=["rgb", "distance_to_image_plane"])
        self.camera_names = list(CAMERAS)

    def _make_lidar(self, show):
        import omni.replicator.core as rep
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
        from omni.replicator.core import Writer

        lidar = Lidar.create(self.body_path + "/sim_lidar", config=LIDAR_CONFIG, variant=LIDAR_VARIANT,
                             translations=np.array([LIDAR_MOUNT], dtype=float),
                             aux_output_level="BASIC", accumulate_outputs=True,
                             attributes={"omni:sensor:Core:elementsCoordsType": "CARTESIAN"})
        rig = self

        class SpotLidarWriter(Writer):
            def __init__(self):
                self.data_structure = "renderProduct"
                self.annotators = [rep.annotators.get("GenericModelOutput")]

            def write(self, data):
                for rp_data in (data.get("renderProducts") or {}).values():
                    raw = rp_data.get("GenericModelOutput")
                    if isinstance(raw, dict):
                        raw = raw.get("data")
                    gmo = parse_generic_model_output_data(raw)
                    if gmo.numElements > 0:
                        rig._on_scan(gmo)

        rep.WriterRegistry.register(SpotLidarWriter)
        self._lidar_sensor = LidarSensor(lidar, annotators=[],
                                         writers=["draw-point-cloud"] if show else [])
        self._lidar_sensor.attach_writer("SpotLidarWriter")

    def _make_imu(self):
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor

        self._imu_sensor = IMUSensor(IMU.create(self.body_path + "/sim_imu",
                                                translations=np.array([IMU_MOUNT], dtype=float)))

    # --- every frame ---------------------------------------------------------------------------
    def _on_scan(self, gmo):
        n = int(gmo.numElements)
        x, y, z = (np.asarray(a, dtype=np.float32)[:n] for a in (gmo.x, gmo.y, gmo.z))
        if "SPHER" in str(gmo.elementsCoordsType).upper():        # azimuth, elevation (deg), range
            az, el, rng = np.radians(x), np.radians(y), z
            x, y, z = rng * np.cos(el) * np.cos(az), rng * np.cos(el) * np.sin(az), rng * np.sin(el)
        xyz = np.column_stack([x, y, z])
        keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-4)
        try:
            intensity = np.asarray(gmo.scalar, dtype=np.float32)[:n][keep]
        except Exception:  # noqa: BLE001
            intensity = None
        with self._lock:
            pose = self._pose
            self._scan = {"t": self._t, "p": pose[0] if pose else None, "q": pose[1] if pose else None,
                          "xyz": xyz[keep].copy(), "intensity": intensity}

    def update(self, sim_time, position, quat):
        """Called by the server once per frame, with the body pose of that frame."""
        self._t = float(sim_time)
        with self._lock:
            self._pose = ([float(v) for v in position], [float(v) for v in quat])
        if self._imu_sensor is not None and not self._imu_log:
            self.sample_imu(sim_time)        # until the physics callback takes over
        if (self._tiled is not None and sim_time >= self._camera_due
                and time.time() - self._demand_wall < DEMAND_S):
            self._camera_due = sim_time + self.camera_period
            try:
                rgb, _ = self._tiled.get_data("rgb")
                depth, _ = self._tiled.get_data("distance_to_image_plane")
                rgb, depth = _to_numpy(rgb), _to_numpy(depth)
                if rgb is not None and depth is not None:
                    frames = {"t": self._t, "p": self._pose[0], "q": self._pose[1],
                              "rgb": np.ascontiguousarray(rgb[..., :3]),
                              "depth": depth.reshape(depth.shape[0], depth.shape[1], depth.shape[2])}
                    with self._lock:
                        self._frames = frames
            except Exception as error:  # noqa: BLE001
                if not getattr(self, "_camera_warned", False):
                    self._camera_warned = True
                    print(f"[SIM] Sensors: camera frames not readable: {type(error).__name__}: {error}")

    def sample_imu(self, t):
        """Called after every physics step (500 per second): keeps one reading every 10 ms."""
        if self._imu_sensor is None or t < self._imu_next_t:
            return
        self._imu_next_t = max(self._imu_next_t + 1.0 / IMU_HZ, t - 1.0 / IMU_HZ)
        try:
            d = self._imu_sensor.get_data()
            reading = {"t": float(t), "a": [float(v) for v in d["linear_acceleration"]],
                       "g": [float(v) for v in d["angular_velocity"]],
                       "q": [float(v) for v in d["orientation"]]}
        except Exception:  # noqa: BLE001 - the first steps have no reading yet
            return
        self._imu = reading
        self._imu_log.append(reading)

    def imu(self):
        return self._imu

    def imu_history(self, req):
        """{"since": t} -> the readings later than t (at most the last IMU_KEPT_S seconds)."""
        if self._imu_sensor is None:
            raise RuntimeError("this simulator has no IMU")
        since = float(req.get("since", -1.0))
        rows = [r for r in list(self._imu_log) if r["t"] > since]
        return {"hz": IMU_HZ, "t": [r["t"] for r in rows], "a": [r["a"] for r in rows],
                "g": [r["g"] for r in rows], "q": [r["q"] for r in rows]}

    # --- what the server serves (called from the socket threads) -----------------------------
    def describe(self):
        out = {"cameras": [], "lidar": None, "imu": None, "notes": list(self.notes)}
        if self._tiled is not None:
            h, w = self.resolution
            for name in self.camera_names:
                p, q = self.camera_mounts[name]
                out["cameras"].append({"name": name, "w": w, "h": h, "mount_p": list(p), "mount_q": list(q),
                                       "hfov_deg": CAMERA_HFOV_DEG, **self.intrinsics})
        if self._lidar_sensor is not None:
            out["lidar"] = {"name": LIDAR_SOURCE, "model": LIDAR_VARIANT, "mount_p": list(LIDAR_MOUNT),
                            "mount_q": [1.0, 0.0, 0.0, 0.0]}
        if self._imu_sensor is not None:
            out["imu"] = {"mount_p": list(IMU_MOUNT), "hz": IMU_HZ}
        return out

    def images(self, req):
        """{"sources": ["frontleft", "frontleft_depth", ...]} -> the latest frames. Colour as
        JPEG; depth as 16-bit millimetres along the view axis, zlib-compressed, 0 = nothing."""
        from PIL import Image

        self._demand_wall = time.time()
        with self._lock:
            frames = self._frames
        if self._tiled is None:
            raise RuntimeError("this simulator has no cameras")
        if frames is None:
            raise RuntimeError("no camera frame yet, ask again")
        out = []
        for source in req.get("sources") or self.camera_names:
            name, depth = (source[:-6], True) if source.endswith("_depth") else (source, False)
            if name not in self.camera_names:
                raise KeyError("unknown camera %r" % (source,))
            i = self.camera_names.index(name)
            h, w = self.resolution
            item = {"name": source, "camera": name, "t": frames["t"], "p": frames["p"], "q": frames["q"],
                    "w": w, "h": h}
            if depth:
                d = frames["depth"][i]
                mm = np.where(np.isfinite(d) & (d < DEPTH_MAX_M), d * 1000.0, 0.0)
                item.update({"format": "depth_u16_zlib",
                             "data": base64.b64encode(zlib.compress(mm.astype("<u2").tobytes(), 1)).decode()})
            else:
                buf = io.BytesIO()
                Image.fromarray(frames["rgb"][i]).save(buf, format="JPEG", quality=int(req.get("quality", 80)))
                item.update({"format": "jpeg", "data": base64.b64encode(buf.getvalue()).decode()})
            out.append(item)
        return {"images": out}

    def lidar(self, req):
        """The latest full turn: float32 x, y, z in the lidar frame, zlib-compressed."""
        with self._lock:
            scan = self._scan
        if self._lidar_sensor is None:
            raise RuntimeError("this simulator has no lidar")
        if scan is None:
            raise RuntimeError("no lidar scan yet, ask again")
        xyz = scan["xyz"]
        return {"name": LIDAR_SOURCE, "t": scan["t"], "p": scan["p"], "q": scan["q"], "n": int(len(xyz)),
                "mount_p": list(LIDAR_MOUNT), "mount_q": [1.0, 0.0, 0.0, 0.0],
                "data": base64.b64encode(zlib.compress(xyz.astype("<f4").tobytes(), 1)).decode()}
