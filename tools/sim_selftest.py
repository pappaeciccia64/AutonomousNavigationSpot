#!/usr/bin/env python3
"""
Check of the simulated robot (spot_explorer/sim/robot.py) through the mission's own
functions: state and frames, local grid, motion commands, GraphNav recording and navigation,
battery, fiducials; and of the simulated sensors (cameras, lidar, IMU) where the simulator has them. Needs the Spot SDK (bosdyn), like the mission; needs NO robot.

    python3 tools/sim_selftest.py                      # starts the kinematic simulator itself
    python3 tools/sim_selftest.py --sim                # uses the simulator already running
    python3 tools/sim_selftest.py --sim 127.0.0.1:8765 # (for example Isaac Sim)

Run it once after copying the project to a new computer, and again against Isaac Sim before
the first simulated mission: it takes a couple of minutes and moves the robot about 3 m.
Exit 0 = everything answered as the mission expects.
"""
import argparse
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import traceback
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def step(title, fn):
    """Run one group of checks; an exception is a failure of the group, not of the script."""
    print(title)
    try:
        fn()
    except Exception as e:
        print(traceback.format_exc())
        check(False, f"{title}: {type(e).__name__}: {e}")


def main():
    ap = argparse.ArgumentParser(description="Self-test of the simulated robot")
    ap.add_argument('--sim', nargs='?', const='', default=None, metavar='HOST:PORT',
                    help="use a simulator already running (default: start the kinematic one)")
    args = ap.parse_args()

    from bosdyn.client.frame_helpers import (GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME,
                                             get_a_tform_b)
    from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
    from spot_explorer import config as P
    from spot_explorer.graphnav import apriltags, map_recording, navigation
    from spot_explorer.graphnav.localization import graphnav_localization
    from spot_explorer.perception import spot_grid
    from spot_explorer.robot import battery, connection, motion

    sim_proc, address = None, args.sim
    if address is None:
        s = socket.socket()
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
        s.close()
        sim_proc = subprocess.Popen(
            [sys.executable, os.path.join(ROOT, 'sim', 'kinematic_server.py'), '--world',
             os.path.join(ROOT, 'sim', 'worlds', 'boxes.json'), '--port', str(port)],
            stdout=subprocess.DEVNULL)
        time.sleep(1.0)
        address = f'127.0.0.1:{port}'
    tmp = tempfile.mkdtemp(prefix='sim_selftest_')
    ctx = {}
    try:
        def connect():
            options = SimpleNamespace(verbose=False, name='selftest', hostname='', sim=address,
                                      recording_session_name='', recording_user_name='')
            robot, lease_client, state_client, metadata = connection.setLogInfo(options)
            estop = connection.SimpleEstop(robot, 'selftest_estop')
            estop.allow()
            ctx.update(robot=robot, state=state_client, metadata=metadata,
                       command=robot.ensure_client(RobotCommandClient.default_service_name))
            check(getattr(robot, 'is_sim', False), "connection.setLogInfo returns the simulated robot")
            robot.time_sync.wait_for_sync()
            robot.power_on()
            check(robot.is_powered_on(), "power on")
            blocking_stand(ctx['command'])
            st = motion.check_fall(state_client)
            check(st['motors_on'] and st['standing'] and not st['fallen'],
                  f"stand: motors on, standing, body level (roll {st['roll']:.1f}, pitch {st['pitch']:.1f} deg)")
        step("Connection, power, stand", connect)
        if 'robot' not in ctx:
            return False
        robot, rsc, cmd = ctx['robot'], ctx['state'], ctx['command']

        def state():
            x, y, z, q = connection.getPosition(rsc)
            snap = rsc.get_robot_state().kinematic_state.transforms_snapshot
            ground = get_a_tform_b(snap, VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME)
            check(ground is not None and 0.3 < z - ground.position.z < 0.8,
                  f"frames: body {z - ground.position.z:.2f} m above the ground plane")
            ctx['start'] = (x, y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
            b = battery.read_battery(rsc)          # dict with 'charge_pct' (robot/battery.py)
            b = None if b is None else b.get('charge_pct')
            check(b is not None and 0 <= b <= 100, f"battery: {b if b is not None else float('nan'):.0f}%")
            check(apriltags.find_fiducials(robot, rsc) == [], "fiducials: none in the simulation, no error")
        step("Robot state", state)

        def sensors():
            """Cameras, lidar and IMU through the SDK-like clients. Only Isaac Sim has them:
            on the kinematic simulator the check is that their absence is said clearly."""
            import io
            from bosdyn.api import image_pb2, point_cloud_pb2
            from bosdyn.client.image import ImageClient
            from bosdyn.client.point_cloud import PointCloudClient
            from spot_explorer.sim.link import SimLinkError
            from spot_explorer.sim.world import World
            images = robot.ensure_client(ImageClient.default_service_name)
            clouds = robot.ensure_client(PointCloudClient.default_service_name)
            core = robot._core
            st = rsc.get_robot_state()
            has = core.sensors()
            has_cameras, has_lidar, has_imu = bool(has.get('cameras')), has.get('lidar') is not None, has.get('imu') is not None
            if not (has_cameras or has_lidar or has_imu):
                check(images.list_image_sources() == [] and clouds.list_point_cloud_sources() == []
                      and rsc.get_imu_state() is None,
                      "no sensors in this simulator: empty lists, no error")
                try:
                    images.get_image_from_sources(['frontleft_fisheye_image'])
                    check(False, "asking for an image without cameras raises a clear error")
                except SimLinkError as e:
                    check('no cameras' in str(e), "asking for an image without cameras raises a clear error")
                return
            # Robot state and IMU, averaged over two seconds: the walking policy of Isaac Sim
            # may be stepping on the spot to hold the pose, so one reading alone can be off.
            speeds, forces = [], []
            for _ in range(20):
                v = rsc.get_robot_state().kinematic_state.velocity_of_body_in_vision
                speeds.append((v.linear.x, v.linear.y))
                imu = rsc.get_imu_state()
                for packet in (imu.packets if imu is not None else ()):
                    f = packet.acceleration_rt_odom_in_link_frame
                    forces.append((f.x, f.y, f.z))
                time.sleep(0.1)
            drift = math.hypot(*np.mean(speeds, axis=0))
            check(st.kinematic_state.HasField('velocity_of_body_in_vision') and drift < 0.1,
                  f"robot state: body velocity while standing ({drift:.3f} m/s on average)")
            names = [j.name for j in st.kinematic_state.joint_states]
            check(len(names) == 12 and 'fl.hx' in names and 'hr.kn' in names,
                  f"robot state: 12 joint states with the SDK names ({', '.join(names[:3])}, ...)")
            a = np.mean(forces, axis=0) if forces else np.zeros(3)
            if not has_imu:
                print("        IMU: not simulated in this run")
            else:
                check(len(forces) > 100 and abs(float(np.linalg.norm(a)) - 9.81) < 0.6 and a[2] > 9.0,
                      f"IMU: standing it reads gravity ({float(np.linalg.norm(a)):.2f} m/s2 on average, "
                      f"z {a[2]:+.2f}, {len(forces)} packets in 2 s)")
            def cameras():
                # cameras
                sources = [src.name for src in images.list_image_sources()]
                check(len(sources) == 15 and 'frontleft_fisheye_image' in sources and 'back_depth' in sources,
                      f"cameras: {len(sources)} image sources (5 cameras: colour, depth, depth in the visual frame)")
                for attempt in range(40):               # the first frames come a moment after the first request
                    try:
                        shots = images.get_image_from_sources(['frontleft_fisheye_image', 'frontleft_depth_in_visual_frame'])
                        break
                    except SimLinkError:
                        time.sleep(0.25)
                else:
                    shots = images.get_image_from_sources(['frontleft_fisheye_image'])
                rgb, depth = shots
                from PIL import Image
                picture = np.asarray(Image.open(io.BytesIO(rgb.shot.image.data)))
                check(rgb.shot.image.format == image_pb2.Image.FORMAT_JPEG
                      and picture.shape == (rgb.shot.image.rows, rgb.shot.image.cols, 3) and picture.std() > 5,
                      f"camera frontleft: a {picture.shape[1]}x{picture.shape[0]} colour picture that is not blank")
                d = np.frombuffer(depth.shot.image.data, dtype='<u2').reshape(depth.shot.image.rows, depth.shot.image.cols)
                d_m = d[d > 0] / depth.source.depth_scale
                check(depth.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16 and d_m.size > 0.5 * d.size
                      and 0.05 < float(np.median(d_m)) < 20.0,
                      f"camera frontleft: depth in millimetres, median {float(np.median(d_m)):.2f} m")
                cam = get_a_tform_b(rgb.shot.transforms_snapshot, 'body', rgb.shot.frame_name_image_sensor)
                check(cam is not None and cam.position.x > 0.3 and rgb.source.pinhole.intrinsics.focal_length.x > 10,
                      "camera frontleft: mounted on the nose in the frame snapshot, pinhole intrinsics given")
                # The lowest quarter of the front picture looks at the ground just ahead: with the
                # camera 0.5 m high and pitched down, that is less than 1.5 m away.
                near = d[int(0.8 * d.shape[0]):, :]
                check(near[near > 0].size > 0 and float(np.median(near[near > 0])) / 1000.0 < 1.5,
                      f"camera frontleft: the bottom of the picture sees the ground ahead "
                      f"({float(np.median(near[near > 0])) / 1000.0:.2f} m)")

            def lidar():
                # lidar
                for attempt in range(40):
                    try:
                        cloud = clouds.get_point_cloud_from_sources(['velodyne-point-cloud'])[0].point_cloud
                        break
                    except SimLinkError:
                        time.sleep(0.25)
                xyz = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 3).astype(np.float64)
                check(cloud.encoding == point_cloud_pb2.PointCloud.ENCODING_XYZ_32F and cloud.num_points == len(xyz)
                      and len(xyz) > 2000 and np.isfinite(xyz).all(), f"lidar: {len(xyz)} points, float32 x y z")
                tf = get_a_tform_b(cloud.source.transforms_snapshot, VISION_FRAME_NAME, cloud.source.frame_name_sensor)
                q = tf.rotation
                rot = np.array([[1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                                [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2), 2 * (q.y * q.z - q.x * q.w)],
                                [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x ** 2 + q.y ** 2)]])
                w = xyz @ rot.T + np.array([tf.position.x, tf.position.y, tf.position.z])
                world = World.from_dict(core.link.world())
                ground = w[w[:, 2] < world.ground_z + 0.10]
                check(len(ground) > 200 and abs(float(np.median(ground[:, 2])) - world.ground_z) < 0.04,
                      f"lidar: in the VISION frame the ground points are on the ground "
                      f"(median height {float(np.median(ground[:, 2])) - world.ground_z:+.3f} m)")
                # What the lidar hits at the robot's height must be an obstacle of the world file:
                # that is what the mission's grid is made of. (The robot's own legs are left out.)
                x, y, _, _ = connection.getPosition(rsc)
                band = w[(w[:, 2] > world.ground_z + 0.25) & (w[:, 2] < world.ground_z + 0.70)
                         & (np.hypot(w[:, 0] - x, w[:, 1] - y) > 0.9) & (np.hypot(w[:, 0] - x, w[:, 1] - y) < 8.0)]
                if len(band) >= 30 and world.shapes:
                    on_obstacle = float((world.signed_distance(band[:, 0], band[:, 1]) < 0.20).mean())
                    check(on_obstacle > 0.85,
                          f"lidar: {100 * on_obstacle:.0f}% of the hits at the robot's height are on an obstacle "
                          f"of the world file ({len(band)} points within 8 m)")
                else:
                    print(f"        lidar against the world file: not checked ({len(band)} hits at the robot's height)")

            for present, name, fn in ((has_cameras, "cameras", cameras), (has_lidar, "lidar", lidar)):
                if present:
                    fn()
                else:
                    print(f"        {name}: not simulated in this run")
        step("Sensors", sensors)

        def grid():
            lg = spot_grid.LocalGrid(robot)
            scan = lg.fetch_scan(rsc)
            check(scan is not None and scan.num_x == 128 and scan.num_y == 128
                  and abs(scan.cell_size - 0.03) < 1e-9, "local grid: three layers, 128 x 128 cells of 3 cm")
            x, y, _, _ = connection.getPosition(rsc)
            cx, cy = scan.pts[:, 0].mean(), scan.pts[:, 1].mean()
            check(math.hypot(cx - x, cy - y) < 0.06, "local grid centred on the robot in the VISION frame")
            from bosdyn.client.frame_helpers import get_a_tform_b
            body = get_a_tform_b(scan.proto.local_grid.transforms_snapshot, 'vision', 'body')
            check(body is not None and math.hypot(body.position.x - x, body.position.y - y) < 0.2,
                  "the grid's own snapshot carries the body pose (the mission reads it there)")
            od = np.asarray(scan.od)
            near = int(np.argmin(np.hypot(scan.pts[:, 0] - x, scan.pts[:, 1] - y)))
            check(od[near] > 0.25, f"obstacle_distance under the robot is free ({od[near]:.2f} m)")
            valid = np.asarray(scan.valid)
            check(float((valid >= spot_grid.TERRAIN_VALID_MIN).mean()) > 0.99, "terrain_valid: all cells valid")
            ground_z = lg.compute_ground_height_in_vision_frame(rsc)
            check(abs(float(np.median(scan.terrain_raw)) - ground_z) < 0.02,
                  "terrain: heights in VISION at the ground plane")
            yaw = ctx['start'][2]
            s = lg.process_scan(scan, x, y, yaw)
            check(s is not None, "process_scan (terrain correction, slope, roughness, obstacle mask) runs")
            ctx['n_obstacle_cells'] = int((od < 0).sum())
            print(f"        cells inside obstacles in this window: {ctx['n_obstacle_cells']}")
        step("Local grid", grid)

        def move():
            x0, y0, yaw0 = ctx['start']
            ok, _ = motion.relative_move(0, 0, math.radians(45), 'vision', cmd, rsc,
                                         max_linear_vel=0.5, max_angular_vel=0.6)
            _, _, _, q = connection.getPosition(rsc)
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            turned = math.degrees(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
            check(ok and abs(turned - 45) < 8, f"relative_move: rotation of 45 deg ({turned:+.0f} done)")
            ok, d = motion.relative_move(0.8, 0, 0, 'vision', cmd, rsc, max_linear_vel=0.5, max_angular_vel=0.6)
            check(ok and abs(d - 0.8) < 0.12, f"relative_move: 0.8 m forward ({d:.2f} m travelled)")
            t0 = time.time()
            ok, d = motion.relative_move(3.0, 0, 0, 'vision', cmd, rsc, max_linear_vel=0.3,
                                         should_abort=lambda: time.time() - t0 > 1.0)
            check(not ok and d < 1.0, f"relative_move: stopped halfway by should_abort after {d:.2f} m")
            time.sleep(1.5)
            a = connection.getPosition(rsc)[:2]
            time.sleep(0.5)
            b = connection.getPosition(rsc)[:2]
            check(math.hypot(a[0] - b[0], a[1] - b[1]) < 0.03, "after the stop the robot stands still")
            ok, _ = motion.move_to_world_point_without_turning(x0, y0, 'vision', cmd, rsc)
            x, y, _, _ = connection.getPosition(rsc)
            check(ok and math.hypot(x - x0, y - y0) < 0.12,
                  "move_to_world_point_without_turning: back at the start without turning")
        step("Motion commands", move)

        def graphnav():
            x0, y0, _ = ctx['start']
            rec = map_recording.RecordingInterface(robot, tmp, ctx['metadata'])
            rec.stop_recording()
            rec.clear_map()
            rec.start_recording()
            check(map_recording.graphnav_is_recording(rec) is True, "recording started")
            r0 = rec.create_default_waypoint(cell_row=0, cell_col=0)
            check(bool(r0) and 'wp_0' in rec.waypoint_poses, "waypoint wp_0 created")
            for dx, dy in ((1.0, 0.0), (1.0, 1.0)):
                x, y, _, q = connection.getPosition(rsc)
                motion.navigate_to(x0 + dx, y0 + dy, x, y, rsc, cmd, SimpleNamespace(rotation=q),
                                   max_linear_vel=0.5, max_angular_vel=0.6)
            rec.create_default_waypoint(cell_row=0, cell_col=1)
            graph = rec._get_graph(force_refresh=True)
            names = [w.annotations.name for w in graph.waypoints]
            check('wp_0' in names and 'wp_1' in names and len(graph.edges) >= len(graph.waypoints) - 1,
                  f"graph: {len(graph.waypoints)} waypoints, {len(graph.edges)} edges, wp_0 and wp_1 in it")
            loc = graphnav_localization(rec)
            check(loc.get('wp_id') == rec.waypoint_poses['wp_1']['waypoint_id'], "localized on wp_1")
            wp0 = next(w for w in graph.waypoints if w.annotations.name == 'wp_0')
            wp1 = next(w for w in graph.waypoints if w.annotations.name == 'wp_1')
            rec.create_edge_between_waypoint_objs(wp1, wp0)
            rec.invalidate_graph_cache()
            check(len(rec._get_graph(force_refresh=True).edges) == len(graph.edges) + 1,
                  "shortcut edge wp_1 - wp_0 accepted while recording")
            rec.stop_recording()
            n, timed_out = rec.close_loops_checked(timeout_s=2.0, upload_result=True)
            check(n == 0 and not timed_out and rec.optimize_anchoring(),
                  "loop closure and anchoring answer (nothing to do in the simulation)")
            ok = navigation.graphnav_navigate_to(rec, wp0.id, rsc, cmd, "self-test: back to wp_0",
                                                 timeout_s=90.0, face_xy=(x0 + 3.0, y0))
            x, y, _, _ = connection.getPosition(rsc)
            check(ok and math.hypot(x - x0, y - y0) < 0.2,
                  f"graphnav_navigate_to: arrived on wp_0 ({math.hypot(x - x0, y - y0):.2f} m from it)")
            path = rec.download_full_graph()
            check(path is not None and os.path.isfile(os.path.join(path, 'graph')), "graph downloaded to a file")
        step("GraphNav", graphnav)

        def shutdown():
            motion.stop_robot(cmd, reason="self-test finished")
            robot.power_off(cut_immediately=False)
            check(not robot.is_powered_on(), "power off")
        step("Shutdown", shutdown)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        if sim_proc is not None:
            sim_proc.terminate()
    print(f"\n{'ALL OK: the simulated robot answers as the mission expects.' if not FAILS else str(len(FAILS)) + ' FAILED'}")
    return not FAILS


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
