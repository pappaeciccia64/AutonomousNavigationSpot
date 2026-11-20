#!/usr/bin/env python3
"""
Field test of the path command of the continuous motion (motion.PathCommand) on the real
robot, outside the mission: the robot walks a fixed shape drawn from where it stands,
the path is sent again every --resend seconds as the cruise will do, and every poll is
written to a CSV. At the end: time, metres, distance from the line (max and mean), time
spent standing.

    python3 tools/try_path.py --shape L --size 1.5            # forward 1.5 m, then left 1.5 m
    python3 tools/try_path.py --shape zigzag --size 1.0 --right
    python3 tools/try_path.py --shape square --stop-and-go    # same shape with today's legs

Shapes (from the robot, x forward, y left; --right mirrors them):
  line     forward size
  L        forward size, then left size (90 deg corner: rotation on the spot at the node)
  zigzag   four legs with corners of 53 deg (turns while walking), 2 x size forward
  square   square of side size, back to the start (four 90 deg corners)
  soft     gentle curve: five points with corners of 30 deg (turns while walking)
The points of the shape are printed before starting, with the corners in degrees.

Needs a free area of about size x size around the shape (2 x size for zigzag). Ctrl+C
stops the robot. At the end the robot sits down and powers off.
"""
import argparse
import csv
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import bosdyn.client.lease                                                        # noqa: E402
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand        # noqa: E402

from spot_explorer.robot import connection, motion                                # noqa: E402


def shape_points(shape, size):
    a = size
    if shape == 'line':
        return [(a, 0.0)]
    if shape == 'L':
        return [(a, 0.0), (a, a)]
    if shape == 'square':
        return [(a, 0.0), (a, a), (0.0, a), (0.0, 0.0)]
    if shape == 'zigzag':
        return [(0.5 * a, 0.25 * a), (a, 0.0), (1.5 * a, 0.25 * a), (2.0 * a, 0.0)]
    if shape == 'soft':
        pts, h, x, y = [], 0.0, 0.0, 0.0
        for _ in range(5):
            x, y = x + 0.5 * a * np.cos(h), y + 0.5 * a * np.sin(h)
            pts.append((x, y))
            h += np.radians(30)
        return pts
    raise ValueError(shape)


def to_world(points, x0, y0, yaw0, right):
    c, s = np.cos(yaw0), np.sin(yaw0)
    out = []
    for px, py in points:
        py = -py if right else py
        out.append((x0 + c * px - s * py, y0 + s * px + c * py))
    return out


def corners_deg(xy):
    seg = [np.arctan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(xy, xy[1:])]
    return [float(np.degrees(np.arctan2(np.sin(b - a), np.cos(b - a)))) for a, b in zip(seg, seg[1:])]


def run_path(pc, spot_path, args, writer):
    t0 = time.time()
    pc.send(spot_path, hold_s=args.hold, max_linear_vel=args.speed, max_angular_vel=args.turn_speed)
    last_send, r = t0, None
    while True:
        time.sleep(args.poll)
        r = pc.poll()
        now = time.time()
        writer.writerow([f"{now - t0:.2f}", f"{r['x']:.3f}", f"{r['y']:.3f}", f"{np.degrees(r['yaw']):.1f}",
                         f"{100 * r['deviation']:.1f}", f"{r['travelled']:.3f}", r['status'], pc.sends])
        if r['status'] in ('arrived', 'blocked', 'expired', 'stopped', 'off_path', 'idle'):
            return r, now - t0
        if now - last_send >= args.resend:
            pc.send(spot_path, hold_s=args.hold, max_linear_vel=args.speed, max_angular_vel=args.turn_speed)
            last_send = now


def run_stop_and_go(spot_path, rsc, cc, args, writer):
    t0, travelled = time.time(), 0.0
    for tx, ty in spot_path:
        x, y, _, rot = connection.getPosition(rsc)
        ok, d, _ = motion.navigate_to(tx, ty, x, y, rsc, cc, SimpleNamespace(rotation=rot),
                                      max_linear_vel=args.speed, max_angular_vel=args.turn_speed)
        travelled += d
        x, y, _, rot = connection.getPosition(rsc)
        writer.writerow([f"{time.time() - t0:.2f}", f"{x:.3f}", f"{y:.3f}", "", "", f"{travelled:.3f}",
                         'arrived' if ok else 'failed', ""])
        if not ok:
            return dict(status='failed', travelled=travelled), time.time() - t0
    return dict(status='arrived', travelled=travelled), time.time() - t0


def summary(csv_path, path_len, elapsed, r):
    rows = list(csv.reader(open(csv_path)))[1:]
    print(f"\n[TRY] End: {r['status']} in {elapsed:.1f} s, {r['travelled']:.2f} m walked "
          f"(path {path_len:.2f} m), {r['travelled'] / max(elapsed, 1e-6):.2f} m/s on average")
    devs = [float(x[4]) for x in rows if x[4]]
    if devs:
        print(f"[TRY] Distance from the line: max {max(devs):.1f} cm, mean {np.mean(devs):.1f} cm, "
              f"p90 {np.percentile(devs, 90):.1f} cm")
    t = np.array([float(x[0]) for x in rows])
    xy = np.array([[float(x[1]), float(x[2])] for x in rows])
    if len(t) > 2:
        v = np.hypot(*np.diff(xy, axis=0).T) / np.maximum(np.diff(t), 1e-3)
        still = float(np.sum(np.diff(t)[v < 0.05]))
        print(f"[TRY] Standing (under 5 cm/s): {still:.1f} s out of {elapsed:.1f} "
              f"(includes the rotations on the spot at the sharp corners)")
    print(f"[TRY] Every poll in {csv_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--shape', choices=['line', 'L', 'square', 'zigzag', 'soft'], default='L')
    ap.add_argument('--size', type=float, default=1.5, help='metres (default 1.5)')
    ap.add_argument('--right', action='store_true', help='mirror the shape to the right')
    ap.add_argument('--speed', type=float, default=0.5, help='linear limit m/s (default 0.5)')
    ap.add_argument('--turn-speed', type=float, default=motion.DEFAULT_MAX_ANGULAR_VEL_RPS,
                    help=f'angular limit rad/s (default {motion.DEFAULT_MAX_ANGULAR_VEL_RPS})')
    ap.add_argument('--resend', type=float, default=0.5, help='seconds between two sends (default 0.5)')
    ap.add_argument('--hold', type=float, default=motion.PATH_HOLD_S,
                    help=f'dead man, seconds (default {motion.PATH_HOLD_S})')
    ap.add_argument('--poll', type=float, default=0.1, help='seconds between two polls (default 0.1)')
    ap.add_argument('--stop-and-go', action='store_true', help="same shape with today's legs (navigate_to)")
    ap.add_argument('--hostname', default='192.168.80.3')
    args = ap.parse_args()
    if args.resend >= args.hold:
        ap.error("--resend must be shorter than --hold, or the command expires between two sends")

    options = SimpleNamespace(name='tryPath', hostname=args.hostname, verbose=False,
                              recording_user_name='', recording_session_name='tryPath')
    robot, lease_client, rsc, _ = connection.setLogInfo(options)
    estop = connection.SimpleEstop(robot, 'tryPath_estop')
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        cc = robot.ensure_client(RobotCommandClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.power_on()
        assert robot.is_powered_on(), 'Robot power-on failed.'
        blocking_stand(cc)
        time.sleep(1.0)
        pc = motion.PathCommand(cc, rsc)
        x0, y0, yaw0 = pc.pose()
        local = shape_points(args.shape, args.size)
        spot_path = to_world(local, x0, y0, yaw0, args.right)
        path_len = float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip([(x0, y0)] + spot_path, spot_path)))
        print(f"[TRY] Shape {args.shape}{' (right)' if args.right else ''}, size {args.size} m, "
              f"{path_len:.2f} m of path, corners {[round(c) for c in corners_deg([(x0, y0)] + spot_path)]} deg "
              f"(on the spot above {motion.PATH_TURN_IN_PLACE_DEG:.0f})")
        print("[TRY] Points from the robot: " + ", ".join(f"({px:.2f}, {(-py if args.right else py):.2f})"
                                                       for px, py in local))
        mode = 'stop_and_go' if args.stop_and_go else 'path'
        os.makedirs('MissionLogs', exist_ok=True)
        csv_path = os.path.join('MissionLogs', f"try_path_{mode}_{args.shape}_{time.strftime('%d-%m-%Y_%H-%M-%S')}.csv")
        input("[TRY] Area free? Enter to start, Ctrl+C to give up... ")
        r, elapsed = None, 0.0
        try:
            with open(csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t_s', 'x', 'y', 'yaw_deg', 'deviation_cm', 'travelled_m', 'status', 'sends'])
                if args.stop_and_go:
                    r, elapsed = run_stop_and_go(spot_path, rsc, cc, args, w)
                else:
                    r, elapsed = run_path(pc, spot_path, args, w)
                    pts = pc.points
                    print(f"[TRY] Last trajectory sent: {len(pts)} points, {pts[-1][3]:.1f} s; "
                          f"{pc.sends} sends; max distance from the line {100 * pc.max_deviation:.1f} cm")
        except KeyboardInterrupt:
            print("\n[TRY] Interrupted by hand.")
        finally:
            motion.stop_robot(cc, "end of the test")
        if r is not None:
            summary(csv_path, path_len, elapsed, r)
        robot.power_off(cut_immediately=False, timeout_sec=20)
        estop.stop()


if __name__ == '__main__':
    main()
