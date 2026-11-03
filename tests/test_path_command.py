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


def _seg_heading_ok(pts, a, b, h, zone):
    """Points strictly inside the segment a-b farther than zone from both ends have heading h."""
    ok = True
    L = np.hypot(b[0] - a[0], b[1] - a[1])
    for x, y, hh, _t in pts:
        d = dist_to_polyline((x, y), [a, b])
        s_ = np.hypot(x - a[0], y - a[1])
        if d < 1e-6 and zone + 1e-6 < s_ < L - zone - 1e-6:
            ok &= abs(np.arctan2(np.sin(hh - h), np.cos(hh - h))) < 1e-6
    return ok


def test_trajectory_points():
    print("\n== Trajectory points ==")
    z = motion.PATH_TURN_ZONE_M
    # 40-degree corner (turn while walking) then a 90-degree one (on the spot).
    c1 = (1.0 + np.cos(np.radians(40)), np.sin(np.radians(40)))
    path = [(1.0, 0.0), c1, (c1[0] + np.cos(np.radians(130)), c1[1] + np.sin(np.radians(130)))]
    pts = motion.path_trajectory(path, (0.0, 0.0), 0.0, 0.5, 0.6)
    gaps = np.hypot(np.diff([p[0] for p in pts]), np.diff([p[1] for p in pts]))
    lim = max(motion.PATH_POINT_SPACING_M, motion.PATH_TURN_ZONE_M)
    check(gaps.max() <= lim + 1e-6, f"points at most {lim} m apart ({gaps.max():.3f}: spacing, or the turn zone)")
    on_line = max(dist_to_polyline((p[0], p[1]), [(0.0, 0.0)] + path) for p in pts)
    check(on_line < 1e-9, "every point on the edges of the path")
    check(_seg_heading_ok(pts, (0.0, 0.0), (1.0, 0.0), 0.0, z)
          and _seg_heading_ok(pts, (1.0, 0.0), c1, np.radians(40), z)
          and _seg_heading_ok(pts, c1, path[2], np.radians(130), z),
          f"heading along each edge farther than {z} m from the nodes")
    at1 = [p for p in pts if np.hypot(p[0] - 1.0, p[1]) < 1e-9]
    check(len(at1) == 1 and abs(np.degrees(at1[0][2]) - 20.0) < 1e-6,
          f"40-deg corner: one pose at the node, halfway (20 deg)")
    at2 = [p for p in pts if np.hypot(p[0] - c1[0], p[1] - c1[1]) < 1e-9]
    check(len(at2) == 2 and abs(np.degrees(at2[0][2]) - 40.0) < 1e-6 and abs(np.degrees(at2[1][2]) - 130.0) < 1e-6,
          "90-deg corner (above PATH_TURN_IN_PLACE_DEG): rotation on the spot at the node")
    check(all(b_[3] > a_[3] for a_, b_ in zip(pts, pts[1:])), "times increasing")
    ok_t = all(abs((b_[3] - a_[3]) - motion.PATH_TIME_FACTOR * max(
        np.hypot(b_[0] - a_[0], b_[1] - a_[1]) / 0.5,
        abs(np.arctan2(np.sin(b_[2] - a_[2]), np.cos(b_[2] - a_[2]))) / 0.6)) < 1e-9 for a_, b_ in zip(pts, pts[1:]))
    check(ok_t, "time of every point = factor x the longer of walking and turning time")
    st = motion.path_trajectory([(1.0, 0.0)], (0.0, 0.0), np.radians(90), 0.5, 0.6)
    check(np.hypot(st[1][0], st[1][1]) < 1e-9 and abs(st[1][2]) < 1e-6,
          f"start {90} deg off the edge (above {motion.PATH_START_IN_PLACE_DEG:.0f}): rotation on the spot first")
    st = motion.path_trajectory([(1.0, 0.0)], (0.0, 0.0), np.radians(20), 0.5, 0.6)
    check(np.hypot(st[1][0], st[1][1]) > 0.01, "start 20 deg off the edge: turns while walking")
    st = motion.path_trajectory([(1.0, 0.0)], (0.0, 0.0), np.radians(20), 0.5, 0.6, start_in_place_deg=10.0)
    check(np.hypot(st[1][0], st[1][1]) < 1e-9, "start_in_place_deg=10 (legs of the mission): on the spot")
    st = motion.path_trajectory([(0.0, 0.08), (1.0, 0.08)], (0.0, 0.0), 0.0, 0.5, 0.6, first_is_step=True)
    check(abs(st[1][2]) < 1e-6, "short step back onto the line: sideways, nose on the next segment")
    st = motion.path_trajectory([(0.1, 0.0), (0.1 + np.cos(0.7), np.sin(0.7))], (0.0, 0.0), 0.0, 0.5, 0.6)
    node = [p for p in st if np.hypot(p[0] - 0.1, p[1]) < 1e-9]
    check(abs(st[-1][2] - 0.7) < 1e-6 and len(node) == 1 and abs(node[0][2] - 0.35) < 1e-6,
          "10 cm from the next node along the line (no step): the corner turns as usual")
    only = motion.path_trajectory([(0.0, 0.0)], (0.0, 0.0), 0.3, 0.5, 0.6)
    check(len(only) == 1 and only[0][3] == 0.0, "empty path: only the start pose")
    ahead, k = motion.path_ahead([(0, 0), (1, 0), (1, 1)], 1.02, 0.4)
    check(np.allclose(ahead, [(1.0, 0.4), (1.0, 1.0)]) and k == 1, f"part ahead: projection on (1,0)-(1,1)")
    ahead, k = motion.path_ahead([(1, 0), (2, 0)], 0.0, 0.0)
    check(np.allclose(ahead, [(1.0, 0.0), (2.0, 0.0)]), "robot before the path: path unchanged")
    square = [(1, 0), (1, 1), (0, 1), (0, 0)]
    ahead, k = motion.path_ahead(square, 0.0, 0.0)
    check(len(ahead) == 4 and k == 0, "closed path starting where the robot is: not cut (searched from the start)")


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
    check(len(req.trajectory.points) == len(pc.points) > 3, f"all the points in the trajectory ({len(req.trajectory.points)})")
    last = req.trajectory.points[-1].pose
    check(np.hypot(last.position.x - 1.0, last.position.y - 1.0) < 1e-9, "last point = end of the path")
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


def _run_path(path, resend=(0.5, 1.0), max_s=40.0):
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    duration = pc.send(path, hold_s=resend[1])
    r, statuses = follow(spot, pc, max_s, resend=resend, path=path)
    return spot, pc, r, statuses, duration


def test_corner_without_stopping():
    print("\n== Path with 40-degree corners, sent again while walking ==")
    h = np.radians(40)
    p1 = (1.0, 0.0)
    p2 = (p1[0] + np.cos(h), np.sin(h))
    p3 = (p2[0] + 1.0, p2[1])
    path = [p1, p2, p3]
    spot, pc, r, statuses, duration = _run_path(path)
    check(r['status'] == 'arrived', f"arrived ({r['status']})")
    check(np.hypot(spot.x - p3[0], spot.y - p3[1]) < 0.05, f"at the last point ({spot.x:.2f}, {spot.y:.2f})")
    check('expired' not in statuses, "the dead man never expires while it sends every 0.5 s")
    dev = max(dist_to_polyline(p, [(0, 0)] + path) for p in spot.track)
    check(dev < 0.05, f"track on the path, corners included ({100 * dev:.1f} cm at most)")
    tr = np.array(spot.track)
    speed = np.hypot(np.diff(tr[:, 0]), np.diff(tr[:, 1])) / 0.02
    moving = speed[5:-25]
    check(moving.min() > 0.15, f"never stopped at the corners (min speed {moving.min():.2f} m/s)")
    check(abs(r['travelled'] - 3.0) < 0.1, f"travelled {r['travelled']:.2f} m (path 3.0)")
    check(abs(np.degrees(spot.yaw)) < 5.0, f"heading at the end along the last edge ({np.degrees(spot.yaw):.0f} deg)")


def test_sharp_corner():
    print("\n== 90-degree corner: rotation on the spot at the node, inside the same command ==")
    path = [(1.0, 0.0), (1.0, 1.0)]
    spot, pc, r, statuses, _ = _run_path(path)
    check(r['status'] == 'arrived', f"arrived ({r['status']})")
    dev = max(dist_to_polyline(p, [(0, 0)] + path) for p in spot.track)
    check(dev < 0.05, f"track on the edges, no corner cut ({100 * dev:.1f} cm at most)")
    check('expired' not in statuses and 'off_path' not in statuses, "no expiry, never off the path")


def test_resend_back_on_line():
    print("\n== Sent again with the robot beside the line: back onto it ==")
    spot = FakeSpot(0.0, 0.07)
    pc = motion.PathCommand(spot, spot)
    path = [(0.0, 0.0), (2.0, 0.0)]
    pc.send(path, hold_s=None)
    r, _ = follow(spot, pc, 20.0)
    check(r['status'] == 'arrived', f"arrived ({r['status']})")
    late = [abs(y) for x, y in spot.track if x > 0.5]
    check(max(late) < 0.01, f"on the checked line after the first 0.5 m (at most {100 * max(late):.1f} cm off)")


@contextlib.contextmanager
def _sim_clock(spot):
    """PathCommand reads time.time() (dead man, rotation at the start, standing time): on the
    simulated clock of the fake Spot."""
    old = motion.time.time
    motion.time.time = lambda s=spot: 1e9 + s.now
    try:
        yield
    finally:
        motion.time.time = old


def test_off_path():
    print("\n== Off the path ==")
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    with _sim_clock(spot):
        pc.send([(3.0, 0.0)], hold_s=None)
        spot.advance(1.0)
        pc.poll()
        spot.y += 0.12                        # pushed sideways
        with contextlib.redirect_stdout(io.StringIO()):
            st = pc.poll()['status']
    check(st == 'off_path', f"12 cm off the line while walking: stopped ({st})")
    spot = FakeSpot(0.0, 0.0, yaw=np.radians(90))
    pc = motion.PathCommand(spot, spot)
    with _sim_clock(spot):
        pc.send([(3.0, 0.0)], hold_s=None, start_in_place_deg=10.0)
        spot.advance(0.5)
        pc.poll()
        spot.y += 0.12                        # the body centre moves while turning on the spot
        with contextlib.redirect_stdout(io.StringIO()):
            st = pc.poll()['status']
    check(st != 'off_path', f"12 cm during the rotation on the spot at the start: goes on ({st}, limit "
                            f"{100 * motion.PATH_MAX_DEVIATION_TURN_M:.0f} cm)")


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
    spot = FakeSpot(0.0, 0.0)
    spot.advise_blocked = True             # Spot says blocked, but the robot walks (03-11 14:45)
    pc = motion.PathCommand(spot, spot)
    pc.send([(2.0, 0.0)], hold_s=None)
    with contextlib.redirect_stdout(io.StringIO()):
        r, statuses = follow(spot, pc, 20.0)
    check(r['status'] == 'arrived' and 'blocked' not in statuses,
          f"'blocked' with the robot walking is advice: arrived ({r['status']})")
    spot = FakeSpot(0.0, 0.0, blocked=lambda x, y: x > 1.2)
    pc = motion.PathCommand(spot, spot)
    with _sim_clock(spot):
        pc.send([(3.0, 0.0)], hold_s=None)
        with contextlib.redirect_stdout(io.StringIO()):
            r, _ = follow(spot, pc, 20.0)
    check(r['status'] == 'blocked', f"blocked and standing: blocked ({r['status']})")
    check(1.1 < spot.x <= 1.2, f"robot before the obstacle ({spot.x:.2f})")


def test_leg_command():
    print("\n== Leg of the stop-and-go with the path command (navigate_to_path) ==")
    import types
    for yaw_deg, abort_after in ((0.0, None), (90.0, None), (0.0, 1.0)):
        spot = FakeSpot(0.0, 0.0, np.radians(yaw_deg))
        old_sleep, old_time = motion.time.sleep, motion.time.time
        motion.time.sleep = lambda dt, s=spot: s.advance(dt)
        motion.time.time = lambda s=spot: 1e9 + s.now
        try:
            vtb = types.SimpleNamespace(rotation=types.SimpleNamespace(w=np.cos(np.radians(yaw_deg) / 2), x=0, y=0,
                                                                       z=np.sin(np.radians(yaw_deg) / 2)))
            ab = (lambda s=spot, t=abort_after: s.now > t) if abort_after else None
            with contextlib.redirect_stdout(io.StringIO()):
                ok, d, cmd = motion.navigate_to_path(1.5, 0.0, 0.0, 0.0, spot, spot, vtb, should_abort=ab,
                                                     max_linear_vel=0.5)
        finally:
            motion.time.sleep, motion.time.time = old_sleep, old_time
        if abort_after:
            check(not ok and 0.2 < d < 1.0, f"stop requested by the frontier: interrupted after {d:.2f} m")
        else:
            check(ok and abs(d - 1.5) < 0.05, f"start {yaw_deg:.0f} deg off: arrived, {d:.2f} m out of {cmd:.2f}")


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
    test_sharp_corner()
    test_resend_back_on_line()
    test_off_path()
    test_dead_man()
    test_stop()
    test_blocked()
    test_leg_command()
    test_network_errors()
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    sys.exit(1 if FAILS else 0)
