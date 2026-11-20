#!/usr/bin/env python3
"""
The simulator bridge (sim/) without a simulator: the reference backend KinematicWorld.

    python3 tests/test_sim_bridge.py

  1. the local grids built from the backend, decoded by the mission's own code
     (spot_grid.LocalGrid.fetch_scan): obstacle where it is, heights of a slope, distance from
     the obstacle, the ground plane frame;
  2. the path command of the mission (motion.PathCommand) on the simulated robot: arrives on
     time over a hill (body pitch follows the ground), reports blocked before a box;
  3. a short whole mission (tools/run_sim_mission.py, scene 'walls'): it ends, sees most of
     the area, no traceback.
"""
import contextlib
import io
import os
import re
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), os.path.join(os.path.dirname(HERE), 'tools')]

import numpy as np                                       # noqa: E402
from sim.kinematic_world import KinematicWorld           # noqa: E402
from sim.spot_adapter import SimRobot                    # noqa: E402
from spot_explorer.perception import spot_grid           # noqa: E402
from spot_explorer.robot import motion                   # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def grids():
    print("== local grids decoded by spot_grid.LocalGrid ==")
    w = KinematicWorld(boxes=[(1.0, 1.4, -0.2, 0.2, 0.8)], ground=[('plane', 0.1, 0.0)], start=(0.0, 0.0, 0.0))
    r = SimRobot(w)
    scan = spot_grid.LocalGrid(r).fetch_scan(r.spot)
    check(scan is not None and (scan.num_x, scan.num_y) == (128, 128), "a scan of 128 x 128 cells")
    pts, od = scan.pts, scan.od
    box = od < 0
    check(abs(pts[box, 0].min() - 1.0) < 0.05 and abs(pts[box, 0].max() - 1.4) < 0.05
          and abs(pts[box, 1].min() + 0.2) < 0.05, "the obstacle where the box is")
    i = int(np.argmin(np.hypot(pts[:, 0] - 0.5, pts[:, 1] - 0.3)))
    check(abs(scan.terrain_raw[i] - 0.05) < 0.005, f"height of the slope at x = 0.5 m ({scan.terrain_raw[i]:.3f})")
    j = int(np.argmin(np.hypot(pts[:, 0], pts[:, 1])))
    check(abs(od[j] - 1.0) < 0.05, f"distance from the box at the robot ({od[j]:.2f} m)")
    from bosdyn.client.frame_helpers import get_a_tform_b
    snap = scan.proto.local_grid.transforms_snapshot
    check(get_a_tform_b(snap, 'vision', 'gpe') is not None, "the ground plane frame in the grid snapshot")


def path():
    print("== motion.PathCommand on the simulated robot ==")
    w = KinematicWorld(ground=[('hill', 1.0, 0.0, 0.4, 0.8)], start=(0.0, 0.0, 0.0))
    r = SimRobot(w)
    pc = motion.PathCommand(r.spot, r.spot)
    dur = pc.send([(1.0, 0.0), (1.5, 0.8)], hold_s=None)
    t0, pitch, res = time.time(), [], None
    while time.time() - t0 < dur + 5:
        time.sleep(0.2)
        res = pc.poll()
        pitch.append(abs(np.degrees(w.pose()[5])))
        if res['status'] not in ('moving', 'near_goal', 'comm_error'):
            break
    check(res['status'] == 'arrived' and time.time() - t0 < dur + 1.5,
          f"arrived ({res['status']}) in {time.time() - t0:.1f} s, commanded {dur:.1f} s")
    check(max(pitch) > 8.0, f"body pitch over the hill up to {max(pitch):.0f} degrees")
    w2 = KinematicWorld(boxes=[(0.8, 1.2, -0.5, 0.5, 1.0)], start=(0.0, 0.0, 0.0))
    r2 = SimRobot(w2)
    pc2 = motion.PathCommand(r2.spot, r2.spot)
    with contextlib.redirect_stdout(io.StringIO()):
        pc2.send([(2.0, 0.0)], hold_s=None)
        t0 = time.time()
        while time.time() - t0 < 8:
            time.sleep(0.2)
            res = pc2.poll()
            if w2.path_status() == 'blocked':
                break
    check(w2.path_status() == 'blocked' and w2.pose()[0] < 0.75, f"stops before the box (x {w2.pose()[0]:.2f} m)")


def mission():
    print("== a whole mission, scene 'walls' ==")
    import run_sim_mission as RS
    out = tempfile.mkdtemp(prefix="sim_test_")
    argv = sys.argv
    sys.argv = ['run_sim_mission.py', '--world', 'walls', '--out', out]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = RS.main()
    finally:
        sys.argv = argv
    log = buf.getvalue()
    seen = [int(v) for v in re.findall(r'\[COVERAGE\] End: area seen (\d+)%', log)]
    check(rc == 0 and 'Traceback' not in log, f"the mission ends without errors (rc {rc})")
    check(seen and seen[-1] >= 80, f"area seen at the end {seen[-1] if seen else None}% (at least 80)")
    check('[CRUISE] End' in log, "the robot moved in cruise")


def main():
    grids()
    path()
    mission()
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
