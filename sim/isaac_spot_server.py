"""
Spot in Isaac Sim, driven by the mission: the simulator side of `run_mission.py --sim`.

It is NVIDIA's spot_standalone.py example (Isaac Sim 6.2,
standalone_examples/api/isaacsim.robot.policy.examples) with the scripted command sequence
replaced by a socket: the mission sends the base velocity, this script feeds it to the
flat-terrain locomotion policy and sends back the body pose. The ground (flat, or the hills
of the world file), the obstacles and, in a forest world, the trees are built from the world
file (isaac_scene.py), so the robot really walks on the slopes and bumps into things. The
body carries five cameras, a lidar and an IMU (isaac_sensors.py).

Run it with sim/run_isaac.sh, or with Isaac Sim's Python from the Isaac Sim folder:

    cd /data/Programs/isaacsim
    ./python.sh /path/to/AutonomousNavigationSpot/sim/isaac_spot_server.py \
        --world /path/to/AutonomousNavigationSpot/sim/worlds/woods.json

then, in another terminal, with the mission's own Python:

    python3 run_mission.py --sim --rows 2 --cols 3

Options: --headless (no window), --device cpu|cuda, --engine physx|newton, --port,
--render-hz (frames drawn per simulated second: lower = faster simulation),
--sensors cameras,lidar,imu | none, --camera-hz, --camera-size WxH, --show-lidar (the lidar
points drawn in the window), --view chase|free (the window follows the robot or not).

Time. The simulation never runs faster than the clock, because the mission measures its time
limits on the clock. Simulated time is counted on physics steps: Isaac Sim does at most
1 / minFrameRate seconds of physics per frame, and silently less if a frame takes longer, so
counting frames would overstate it. While no velocity is commanded the robot holds its pose
(see hold_command). The world frame is the mission's VISION frame: the robot spawns at the
origin facing +x.
"""
import argparse
import math
import os
import socket
import sys

# Lines appear at once also when the output goes to a file or a pipe.
sys.stdout.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="Spot in Isaac Sim for run_mission.py --sim")
parser.add_argument("--world", default=None, help="world file (see spot_explorer/sim/world.py)")
parser.add_argument("--host", default="127.0.0.1")
parser.add_argument("--port", type=int, default=8765)
parser.add_argument("--headless", default=False, action="store_true")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu",
                    help="Simulation device (one robot: the CPU is faster than the GPU)")
parser.add_argument("--engine", type=str, choices=["physx", "newton"], default="physx", help="Physics engine")
parser.add_argument("--render-hz", type=float, default=25.0,
                    help="frames drawn per simulated second (default 25; the policy's own is 50)")
parser.add_argument("--sensors", default="cameras,lidar,imu",
                    help="which sensors to simulate: any of cameras,lidar,imu, or none (default: all)")
parser.add_argument("--camera-hz", type=float, default=5.0, help="camera frames per simulated second")
parser.add_argument("--camera-size", default="320x240", help="camera image size, WIDTHxHEIGHT")
parser.add_argument("--show-lidar", default=False, action="store_true",
                    help="draw the lidar points in the window")
parser.add_argument("--view", choices=["chase", "free"], default="chase",
                    help="chase: the window follows the robot; free: the camera is yours")
parser.add_argument("--assets", default=None,
                    help="folder of the downloaded forest assets (default: sim/assets/forest)")
args, unknown = parser.parse_known_args()

# Before the long start of Isaac Sim: is the port free? (Usually: a simulator still running.)
_probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    _probe.bind((args.host, args.port))
except OSError as error:
    sys.exit(f"[SIM] Port {args.host}:{args.port} is busy ({error}): another simulator is still "
             f"running. Close it (pkill -f isaac_spot_server.py) or choose another --port.")
finally:
    _probe.close()
try:
    cam_w, cam_h = (int(v) for v in args.camera_size.lower().split("x"))
except ValueError:
    sys.exit(f"[SIM] --camera-size {args.camera_size!r}: expected WIDTHxHEIGHT, for example 320x240")

extra_args = [f"--/exts/isaacsim.core.simulation_manager/default_engine={args.engine}"]
if args.engine == "newton":
    extra_args.extend(["--enable", "isaacsim.physics.newton", "--enable", "isaacsim.physics.newton.tensors"])
simulation_app = SimulationApp({"headless": args.headless, "extra_args": extra_args})

import time  # noqa: E402

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.utils.stage import define_prim, set_stage_units, set_stage_up_axis  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents  # noqa: E402
from isaacsim.robot.policy.examples import PolicyEnvConfig, RobotPolicyRunner, get_spot_spec  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

# The mission project: this file is in <project>/sim/.
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from isaac_scene import build_scene  # noqa: E402
from isaac_sensors import SensorRig  # noqa: E402
from spot_explorer.sim.server import SimServer  # noqa: E402
from spot_explorer.sim.world import World  # noqa: E402

ROBOT_PRIM = "/World/Spot"

first_step = True
reset_needed = False
policy_failed = False
base_command = np.zeros(3, dtype=np.float32)
sim_time = 0.0                  # simulated seconds, counted on physics steps
rig = None


def on_physics_step(step_size: float, context: object) -> None:
    """Initialize, reset, or advance the Spot policy runner after a physics step (as in
    spot_standalone.py); count the simulated time; sample the IMU."""
    global first_step, reset_needed, policy_failed, sim_time
    sim_time += step_size
    if rig is not None:
        rig.sample_imu(sim_time)
    if policy_failed:
        return
    try:
        if first_step:
            spot.restart_from_default_state(base_command)
            first_step = False
        elif reset_needed:
            reset_needed = False
            first_step = True
        else:
            spot.step(step_size, base_command)
    except Exception as error:  # noqa: BLE001 - a physics callback must not raise
        policy_failed = True
        carb.log_error(f"isaac_spot_server: policy deployment failed, stopping: {error}")


def _to_numpy(a):
    """Warp array, torch tensor or numpy array -> numpy."""
    if hasattr(a, "numpy"):
        try:
            return np.asarray(a.numpy())
        except Exception:  # noqa: BLE001 - a torch tensor on the GPU
            return np.asarray(a.detach().cpu().numpy())
    return np.asarray(a)


def _pose_from_articulation():
    positions, orientations = spot.articulation.get_world_poses()
    p = _to_numpy(positions).reshape(-1, 3)[0]
    q = _to_numpy(orientations).reshape(-1, 4)[0]      # (w, x, y, z)
    return p, q


def _pose_from_usd():
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(ROBOT_PRIM + "/body")
    if not prim.IsValid():
        prim = stage.GetPrimAtPath(ROBOT_PRIM)
    m = omni.usd.get_world_transform_matrix(prim)
    t, r = m.ExtractTranslation(), m.ExtractRotationQuat()
    im = r.GetImaginary()
    return np.array([t[0], t[1], t[2]]), np.array([r.GetReal(), im[0], im[1], im[2]])


POSE_SOURCES = [("the articulation (get_world_poses)", _pose_from_articulation),
                ("the USD prim transform", _pose_from_usd)]
pose_source = None


def read_pose():
    """Body position and orientation (w, x, y, z) in the world frame. The first source that
    gives a sane pose is kept for the whole run and named once in the output."""
    global pose_source
    sources = [pose_source] if pose_source is not None else POSE_SOURCES
    for name, fn in sources:
        try:
            p, q = fn()
            if np.all(np.isfinite(p)) and abs(float(np.linalg.norm(q)) - 1.0) < 1e-2:
                if pose_source is None:
                    pose_source = (name, fn)
                    print(f"[SIM] Robot pose read from {name}: position "
                          f"({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}).")
                return p, q
        except Exception as error:  # noqa: BLE001 - try the next source
            if pose_source is None:
                print(f"[SIM] Pose not readable from {name}: {type(error).__name__}: {error}")
    return None, None


# --- holding the pose ------------------------------------------------------------------------
# With a zero command the locomotion policy does not stand still: the body turns about 2 deg/s
# and creeps for many seconds. Spot stands where it is told to, so while no velocity is
# commanded this script closes the loop itself on the pose the robot stopped at (at start-up:
# the spawn pose, so the mission finds the robot at the origin facing +x).
HOLD_CAPTURE_S = 0.8        # after the last non-zero command, time to come to rest
HOLD_KP_LINEAR = 1.5        # 1/s
HOLD_KP_ANGULAR = 2.0       # 1/s
HOLD_MAX_LINEAR_MPS = 0.3
HOLD_MAX_ANGULAR_RPS = 0.5
hold_pose = None            # (x, y, yaw) to keep; None while moving or coming to rest
last_motion_t = -1e9


def yaw_of(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def hold_command(position, quat):
    """Body velocity that takes the robot back to hold_pose."""
    yaw = yaw_of(quat)
    ex, ey = hold_pose[0] - float(position[0]), hold_pose[1] - float(position[1])
    eyaw = math.atan2(math.sin(hold_pose[2] - yaw), math.cos(hold_pose[2] - yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = HOLD_KP_LINEAR * (c * ex + s * ey), HOLD_KP_LINEAR * (-s * ex + c * ey)
    speed = math.hypot(vx, vy)
    if speed > HOLD_MAX_LINEAR_MPS:
        vx, vy = vx * HOLD_MAX_LINEAR_MPS / speed, vy * HOLD_MAX_LINEAR_MPS / speed
    wz = max(-HOLD_MAX_ANGULAR_RPS, min(HOLD_MAX_ANGULAR_RPS, HOLD_KP_ANGULAR * eyaw))
    return np.array([vx, vy, wz], dtype=np.float32)


world = World.from_file(args.world) if args.world else World()
print(f"Using engine: {args.engine}")
print(f"Using device: {args.device}")

# spawn world
set_stage_up_axis("Z")
set_stage_units(meters_per_unit=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

define_prim("/World/PhysicsScene", "PhysicsScene")
build_scene(world, args.assets or os.path.join(HERE, "assets", "forest"), assets_root_path)

# select the simulation device before constructing the policy articulation
SimulationManager.set_physics_sim_device(args.device)

# spawn robot through the generic policy runner (bundled Spot spec)
spec = get_spot_spec()
spot = RobotPolicyRunner(spec, prim_path=ROBOT_PRIM)
spot.spawn()

# timing from the deployed artifact's env config
timing = PolicyEnvConfig.from_file(spec.engines[args.engine].env_config_path).timing
policy_dt = timing.render_interval * timing.physics_dt
# One window frame every `steps` policy periods: drawing is what slows the simulation down.
steps = max(1, int(round(1.0 / (max(args.render_hz, 1.0) * policy_dt))))
frame_dt = steps * policy_dt
RenderingManager.set_dt(frame_dt)
SimulationManager.set_physics_dt(spot.physics_dt)
# Isaac Sim does at most 1 / minFrameRate seconds of physics per frame: below one frame of
# ours it would silently drop physics steps. Lowered for this run, restored at the end.
_settings = carb.settings.get_settings()
MIN_FRAME_RATE_KEY = "/persistent/simulation/minFrameRate"
_old_min_frame_rate = _settings.get(MIN_FRAME_RATE_KEY)
_settings.set(MIN_FRAME_RATE_KEY, max(1, int(math.floor(0.8 / frame_dt))))
print(f"[SIM] Physics {1.0 / spot.physics_dt:.0f} Hz, policy period {policy_dt * 1000:.0f} ms, "
      f"one frame drawn every {frame_dt * 1000:.0f} ms of simulation.")

# sensors on the body: cameras, lidar, IMU (see isaac_sensors.py)
wanted = {w.strip().lower() for w in args.sensors.split(",")} - {"", "none"}
rig = SensorRig(ROBOT_PRIM + "/body", cameras="cameras" in wanted, lidar="lidar" in wanted,
                imu="imu" in wanted, camera_hz=args.camera_hz, resolution=(cam_h, cam_w),
                show_lidar=args.show_lidar)
summary = rig.describe()
print("[SIM] Sensors: "
      + (f"{len(summary['cameras'])} cameras {cam_w}x{cam_h} at {args.camera_hz:g} Hz (colour and depth)"
         if summary["cameras"] else "no cameras")
      + (f", lidar {summary['lidar']['model']}" if summary["lidar"] else ", no lidar")
      + (", IMU" if summary["imu"] is not None else ", no IMU") + ".")


# --- the window follows the robot ------------------------------------------------------------
CHASE_BACK_M, CHASE_UP_M, CHASE_LOOK_UP_M = 3.4, 1.9, 0.25
chase_yaw = None
chase_op = None


def setup_chase_camera():
    global chase_op
    from omni.kit.viewport.utility import get_active_viewport

    stage = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera.Define(stage, "/World/ChaseCamera")
    cam.GetFocalLengthAttr().Set(16.0)
    cam.GetClippingRangeAttr().Set((0.05, 500.0))
    chase_op = UsdGeom.Xformable(cam.GetPrim()).AddTransformOp()
    viewport = get_active_viewport()
    if viewport is not None:
        viewport.camera_path = "/World/ChaseCamera"


def update_chase_camera(position, quat, dt):
    """Behind and above the robot, turning slowly so that the picture does not swing at every
    correction of the heading."""
    global chase_yaw
    yaw = yaw_of(quat)
    if chase_yaw is None:
        chase_yaw = yaw
    chase_yaw += math.atan2(math.sin(yaw - chase_yaw), math.cos(yaw - chase_yaw)) * min(1.0, 0.8 * dt)
    x, y, z = (float(v) for v in position)
    ex, ey = x - CHASE_BACK_M * math.cos(chase_yaw), y - CHASE_BACK_M * math.sin(chase_yaw)
    eye = Gf.Vec3d(ex, ey, max(world.height_at(ex, ey), z - 0.5) + CHASE_UP_M)
    look = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(x, y, z + CHASE_LOOK_UP_M), Gf.Vec3d(0, 0, 1))
    chase_op.Set(look.GetInverse())


if args.view == "chase":
    try:
        setup_chase_camera()
    except Exception as error:  # noqa: BLE001 - the simulation does not depend on the view
        print(f"[SIM] Chase camera not available: {type(error).__name__}: {error}")
        chase_op = None

_physics_callback_id = SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

server = SimServer("isaac", world.to_dict(looks=False), host=args.host, port=args.port)
server.providers["sensors"] = lambda req: rig.describe()
server.providers["image"] = rig.images
server.providers["lidar"] = rig.lidar
server.providers["imu"] = rig.imu_history
screenshots = []                 # paths asked with the "screenshot" request, taken by the loop


def ask_screenshot(req):
    path = os.path.abspath(str(req["path"]))
    screenshots.append(path)
    return {"path": path}


server.providers["screenshot"] = ask_screenshot


def body_motion():
    """Velocity of the body in the world frame, joint positions and velocities: what the robot
    state of the SDK carries besides the pose."""
    out = {}
    try:
        lin, ang = spot.articulation.get_velocities()
        out["v"] = [float(v) for v in _to_numpy(lin).reshape(-1, 3)[0]]
        out["w"] = [float(v) for v in _to_numpy(ang).reshape(-1, 3)[0]]
        out["jp"] = [float(v) for v in _to_numpy(spot.articulation.get_dof_positions()).reshape(-1)]
        out["jv"] = [float(v) for v in _to_numpy(spot.articulation.get_dof_velocities()).reshape(-1)]
    except Exception:  # noqa: BLE001 - the pose alone is enough for the mission
        pass
    return out


try:
    server.info["joints"] = list(spot.articulation.dof_names)
except Exception:  # noqa: BLE001
    server.info["joints"] = []

timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

announced = False
report_wall, report_sim = time.time(), 0.0
pace_wall, pace_sim = time.time(), 0.0
try:
    while simulation_app.is_running():
        simulation_app.update()
        # Never faster than the clock; if the computer is slower, do not try to catch up.
        ahead = (sim_time - pace_sim) - (time.time() - pace_wall)
        if ahead > 0:
            time.sleep(ahead)
        elif ahead < -0.5:
            pace_wall, pace_sim = time.time(), sim_time
        if not SimulationManager.is_simulating():
            reset_needed = True
            continue
        position, quat = read_pose()
        commanded = np.asarray(server.command(), dtype=np.float32)
        if policy_failed or position is None:
            base_command = np.zeros(3, dtype=np.float32)
        elif np.any(np.abs(commanded) > 1e-4):
            base_command = commanded
            hold_pose, last_motion_t = None, sim_time
        else:
            if hold_pose is None and sim_time - last_motion_t >= HOLD_CAPTURE_S:
                # First time: the spawn pose, heading +x. Later: where the robot stopped.
                hold_pose = ((float(position[0]), float(position[1]), 0.0) if last_motion_t < 0
                             else (float(position[0]), float(position[1]), yaw_of(quat)))
            base_command = (hold_command(position, quat) if hold_pose is not None
                            else np.zeros(3, dtype=np.float32))
        if position is not None:
            rig.update(sim_time, position, quat)
            extra = body_motion()
            if rig.imu() is not None:
                extra["imu"] = rig.imu()
            server.set_state(sim_time, position, quat, extra)
            if chase_op is not None:
                update_chase_camera(position, quat, frame_dt)
            while screenshots:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

                capture_viewport_to_file(get_active_viewport(), screenshots.pop(0))
            if not announced:
                announced = True
                print(f"[SIM] Ready: waiting for the mission on {server.address[0]}:{server.address[1]} "
                      f"(python3 run_mission.py --sim).")
        # Every ~10 s of clock: how fast the simulation runs compared with the clock, on the
        # physics steps. The mission measures its time limits on the clock, so well below 1.0
        # the moves may time out.
        now = time.time()
        if now - report_wall >= 10.0:
            print(f"[SIM] t = {sim_time:.0f} s, real-time factor {(sim_time - report_sim) / (now - report_wall):.2f}, "
                  f"command ({base_command[0]:+.2f}, {base_command[1]:+.2f}, {base_command[2]:+.2f}), "
                  f"mission {'connected' if server.clients else 'not connected'}.")
            report_wall, report_sim = now, sim_time
finally:
    if _old_min_frame_rate is not None:
        _settings.set(MIN_FRAME_RATE_KEY, _old_min_frame_rate)
timeline.stop()
server.close()
SimulationManager.deregister_callback(_physics_callback_id)
spot.close()
simulation_app.close()
