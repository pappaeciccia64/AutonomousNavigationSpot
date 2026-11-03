"""
Test of the multi-point path command without waiting (motion.PathCommand) against the
fake Spot with a simulated clock (tests/fake_spot.py): the proto that reaches Spot, a path
with a corner followed without stopping, new sends while walking, the dead man, stop,
point blocked, network errors.

    python3 tests/test_path_command.py
"""
import contextlib
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from bosdyn.api import trajectory_pb2                      # noqa: E402
from bosdyn.api.spot import robot_command_pb2 as spot_cmd  # noqa: E402
from bosdyn.util import duration_to_seconds                # noqa: E402

from fake_spot import FakeSpot                             # noqa: E402
from spot_explorer.robot import motion                     # noqa: E402

FAILS = []
DT = 0.2          # poll period of the caller


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def dist_to_polyline(p, poly):
    best = np.inf
    for a, b in zip(poly, poly[1:]):
        a, b, q = np.asarray(a, float), np.asarray(b, float), np.asarray(p, float)
        ab = b - a
        t = np.clip(np.dot(q - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(q - (a + t * ab))))
    return best


def follow(spot, pc, max_s, resend=None, path=None):
    """Poll every DT until a final status; resend=(period, hold) sends the path again."""
    statuses, last_send = [], spot.now
    while spot.now < max_s:
        spot.advance(DT)
        r = pc.poll()
        statuses.append(r['status'])
        if r['status'] not in ('moving', 'near_goal', 'comm_error'):
            return r, statuses
        if resend and spot.now - last_send >= resend[0] - 1e-9:
            pc.send(path, hold_s=resend[1])
            last_send = spot.now
    return r, statuses


def test_trajectory_points():
    print("\n== Trajectory points ==")
    pts = motion.path_trajectory([(0.005, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 1.0)], (0.0, 0.0), 0.0, 0.5, 0.6)
    check(len(pts) == 3, f"start + 2 points, close duplicates dropped ({len(pts)})")
    check(abs(np.degrees(pts[1][2]) - 45.0) < 1e-6, f"heading at the corner halfway (45 deg: {np.degrees(pts[1][2]):.1f})")
    check(abs(np.degrees(pts[2][2]) - 90.0) < 1e-6, "heading at the end along the last segment")
    expected = motion.PATH_TIME_FACTOR * (1.0 / 0.5 + np.radians(45) / 0.6)
    check(abs(pts[1][3] - expected) < 1e-6, f"time of the first point {pts[1][3]:.2f} s (expected {expected:.2f})")
    check(all(b[3] > a[3] for a, b in zip(pts, pts[1:])), "times increasing")
    ahead = motion.path_ahead([(0, 0), (1, 0), (1, 1)], 1.02, 0.4)
    check(np.allclose(ahead, [(1.0, 0.4), (1.0, 1.0)]), f"part ahead: from the projection on (1,0)-(1,1) ({ahead})")
    ahead = motion.path_ahead([(1, 0), (2, 0)], 0.0, 0.0)
    check(np.allclose(ahead, [(1.0, 0.0), (2.0, 0.0)]), "robot before the path: path unchanged")
    only = motion.path_trajectory([(0.0, 0.0)], (0.0, 0.0), 0.3, 0.5, 0.6)
    check(len(only) == 1 and only[0][3] == 0.0, "empty path: only the start pose")


def test_proto():
    print("\n== Command sent to Spot ==")
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    pc.send([(1.0, 0.0), (1.0, 1.0)], max_linear_vel=0.4, max_angular_vel=0.5,
            locomotion_hint=spot_cmd.HINT_CRAWL)
    _, cmd, end = spot.commands[-1]
    req = cmd.synchronized_command.mobility_command.se2_trajectory_request
    params = spot_cmd.MobilityParams()
    cmd.synchronized_command.mobility_command.params.Unpack(params)
    check(req.se2_frame_name == 'vision', "frame vision")
    check(len(req.trajectory.points) == 3, f"3 points in the trajectory ({len(req.trajectory.points)})")
    check(req.trajectory.interpolation == trajectory_pb2.POS_INTERP_LINEAR, "linear interpolation")
    check(abs(duration_to_seconds(req.trajectory.points[-1].time_since_reference) - pc.points[-1][3]) < 1e-6,
          "times of the points in the proto")
    check(req.trajectory.HasField('reference_time'), "reference time set")
    check(abs(end - (pc.sent_at + motion.PATH_HOLD_S)) < 1e-6, f"end time = send + {motion.PATH_HOLD_S} s (dead man)")
    check(params.vel_limit.max_vel.linear.x == 0.4 and params.vel_limit.min_vel.linear.x == -0.4,
          "speed limit symmetric")
    check(params.locomotion_hint == spot_cmd.HINT_CRAWL, "gait passed on")
    pc.send([(1.0, 0.0)], hold_s=None)
    _, _, end = spot.commands[-1]
    full = motion.MOVE_TIMEOUT_FACTOR * pc.points[-1][3] + motion.MOVE_TIMEOUT_MARGIN_S
    check(abs(end - pc.sent_at - full) < 1e-6, "hold_s=None: the whole path with the time limit of relative_move")


def test_corner_without_stopping():
    print("\n== Path with a corner, sent again while walking ==")
    path = [(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (2.0, 2.0)]
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    duration = pc.send(path, hold_s=1.0)
    r, statuses = follow(spot, pc, 30.0, resend=(0.5, 1.0), path=path)
    check(r['status'] == 'arrived', f"arrived ({r['status']})")
    check(np.hypot(spot.x - 2.0, spot.y - 2.0) < 0.05, f"at the last point ({spot.x:.2f}, {spot.y:.2f})")
    check('expired' not in statuses, "the dead man never expires while it sends every 0.5 s")
    dev = max(dist_to_polyline(p, [(0, 0)] + path) for p in spot.track)
    check(dev < 0.05, f"track on the path, corner included ({100 * dev:.1f} cm at most)")
    # Never stopped along the way: speed between two polls never below half the reference.
    tr = np.array(spot.track)
    steps = np.hypot(np.diff(tr[:, 0]), np.diff(tr[:, 1])) / 0.02
    moving = steps[5:-25]        # start and approach to the goal apart
    v_ref = 0.5 / motion.PATH_TIME_FACTOR
    check(moving.min() > 0.5 * v_ref, f"never stopped at the points (min speed {moving.min():.2f} m/s, "
                                     f"reference {v_ref:.2f})")
    check(abs(r['travelled'] - 4.0) < 0.1, f"travelled {r['travelled']:.2f} m (path 4.0)")
    check(spot.now < duration + 2.0, f"time {spot.now:.1f} s, trajectory {duration:.1f} s")
    check(abs(np.degrees(spot.yaw) - 90.0) < 5.0, f"heading at the end along the path ({np.degrees(spot.yaw):.0f} deg)")


def test_dead_man():
    print("\n== Dead man: no new send ==")
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    pc.send([(3.0, 0.0)], hold_s=2.0)
    r, _ = follow(spot, pc, 10.0)
    check(r['status'] == 'expired', f"command expired ({r['status']})")
    x_stop = spot.x
    check(1.5 < spot.now <= 2.6, f"after ~2 s ({spot.now:.1f})")
    spot.advance(2.0)
    check(spot.x == x_stop and x_stop < 1.0, f"robot stopped at {x_stop:.2f} m, then still")


def test_stop():
    print("\n== Stop ==")
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    pc.send([(3.0, 0.0)])
    spot.advance(1.0)
    pc.poll()
    with contextlib.redirect_stdout(io.StringIO()):
        pc.stop("test")
    x_stop = spot.x
    spot.advance(1.0)
    check(spot.x == x_stop, "after stop the robot does not move")
    check(pc.poll()['status'] == 'idle', "status idle after stop")


def test_blocked():
    print("\n== Point blocked ==")
    spot = FakeSpot(0.0, 0.0, blocked=lambda x, y: x > 1.2)
    pc = motion.PathCommand(spot, spot)
    pc.send([(3.0, 0.0)], hold_s=None)
    r, _ = follow(spot, pc, 20.0)
    check(r['status'] == 'blocked', f"Spot reports the goal blocked ({r['status']})")
    check(1.1 < spot.x <= 1.2, f"robot before the obstacle ({spot.x:.2f})")


def test_network_errors():
    print("\n== Network errors ==")
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    pc.send([(3.0, 0.0)], hold_s=None)
    spot.advance(0.4)
    pc.poll()
    spot.fail_next = 2
    with contextlib.redirect_stdout(io.StringIO()):
        s = [pc.poll()['status'] for _ in range(3)]
    check(s == ['comm_error', 'comm_error', 'moving'], f"two errors and then it goes on ({s})")
    spot.fail_next = 10
    raised = False
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            for _ in range(motion.MOVE_MAX_RPC_ERRORS):
                pc.poll()
        except RuntimeError:
            raised = True
    stop_sent = not spot.commands[-1][1].synchronized_command.mobility_command.HasField('se2_trajectory_request')
    check(raised and stop_sent, f"after {motion.MOVE_MAX_RPC_ERRORS} in a row: stop sent and error raised")


if __name__ == '__main__':
    test_trajectory_points()
    test_proto()
    test_corner_without_stopping()
    test_dead_man()
    test_stop()
    test_blocked()
    test_network_errors()
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)
