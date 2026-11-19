#!/usr/bin/env python3
"""
How often does Spot renew its local grids, and what does a scan cost? READ ONLY: no lease,
no E-Stop, no command; the robot can stand still or sit (it only needs to be on).

    python3 tools/grid_rate_check.py [--hostname 192.168.80.3] [--seconds 10]
                                     [--periods 0,0.1,0.2,0.4] [--csv grid_rate.csv]

For every request period (0 = back to back) it asks for the same layers as the mission
(obstacle_distance, terrain, terrain_valid, one request) for --seconds and, as the mission
does for every scan, the robot state (the ground height of the obstacle layer). Per period:

  new grids     how many requests brought a grid with a new acquisition time; "same" = the
                grid already read (we asked faster than Spot renews it)
  interval      between two NEW grids (robot clock): how often Spot renews them at that
                request rate; in multiples of 1/15 s (the cameras)
  grid request  time of get_local_grids (network + Spot), p50 / p90
  state request time of get_robot_state (the second round trip of every scan), p50 / p90
  decode        time to unpack the three layers here (CPU, holds the GIL)
  age           how old the grid is when it arrives: local time at the answer, in the robot
                clock (time sync), minus its acquisition time
Before the measures it prints the frames carried by the grid and by the robot state (is the
ground plane 'gpe' in the grid? then a scan could skip the robot state request).

If at period 0 almost every request brings a new grid, Spot renews them at least as often
as we can read them, and the planning cycle can run "as soon as it is free". If many are
"same", the interval says how often it is worth asking.
Login: BOSDYN_CLIENT_USERNAME / BOSDYN_CLIENT_PASSWORD, or asked.
"""
import argparse
import csv
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import numpy as np                                           # noqa: E402
import bosdyn.client                                         # noqa: E402
import bosdyn.client.util                                    # noqa: E402
from bosdyn.client.robot_state import RobotStateClient       # noqa: E402

from spot_explorer.perception import spot_grid                # noqa: E402

LAYERS = ['obstacle_distance', 'terrain', 'terrain_valid']   # as spot_grid.LocalGrid.fetch_scan
FRAME_S = 1.0 / 15.0


def _secs(ts):
    return ts.seconds + ts.nanos * 1e-9


def measure(local_grid, client, state_client, skew_s, period, seconds):
    rows, t_end = [], time.time() + seconds
    while time.time() < t_end:
        t0 = time.time()
        try:
            protos = list(client.get_local_grids(LAYERS, timeout=spot_grid.SCAN_TIMEOUT_S))
        except Exception as e:
            rows.append(dict(period=period, t=t0, error=repr(e)))
            time.sleep(0.2)
            continue
        t1 = time.time()
        for lg in protos:
            local_grid.unpack_grid(lg)
        t2 = time.time()
        state_client.get_robot_state(timeout=spot_grid.SCAN_TIMEOUT_S)
        t3 = time.time()
        acq = _secs(protos[0].local_grid.acquisition_time)
        rows.append(dict(period=period, t=t0, acquisition=acq, grid_s=t1 - t0, decode_s=t2 - t1,
                         state_s=t3 - t2, age_s=(t1 + skew_s) - acq, error=''))
        time.sleep(max(0.0, period - (time.time() - t0)))
    return rows


def summary(rows, period):
    ok = [r for r in rows if not r['error']]
    errors = len(rows) - len(ok)
    if not ok:
        return f"period {period:.2f} s: no grid ({errors} errors)"
    acq = np.array([r['acquisition'] for r in ok])
    new = np.r_[True, np.diff(acq) > 1e-6]
    dt = np.diff(acq[new])
    p = lambda k, q: np.percentile([r[k] for r in ok], q)
    txt = (f"period {period:.2f} s: {len(ok)} requests, {new.sum()} new grids, {len(ok) - new.sum()} same"
           f"{f', {errors} errors' if errors else ''}\n")
    if dt.size:
        frames = np.round(dt / FRAME_S).astype(int)
        vals, counts = np.unique(frames, return_counts=True)
        hist = ', '.join(f"{v}x{1000 * FRAME_S:.0f}ms: {c}" for v, c in zip(vals[:8], counts[:8]))
        txt += (f"   interval between new grids: median {np.median(dt):.3f} s, min {dt.min():.3f} s"
                f"  [{hist}]\n")
    txt += (f"   grid request p50/p90 {p('grid_s', 50):.3f}/{p('grid_s', 90):.3f} s | state request "
            f"{p('state_s', 50):.3f}/{p('state_s', 90):.3f} s | decode {p('decode_s', 50):.3f} s | "
            f"age at arrival {p('age_s', 50):.3f}/{p('age_s', 90):.3f} s")
    return txt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--hostname', default=os.environ.get('SPOT_HOSTNAME', '192.168.80.3'),
                    help='192.168.80.3 from the laptop on the robot WiFi, 192.168.50.3 from the CORE I/O')
    ap.add_argument('--seconds', type=float, default=10.0, help='per request period')
    ap.add_argument('--periods', default='0,0.1,0.2,0.4', help='request periods to try, s (0 = back to back)')
    ap.add_argument('--csv', default=None, help='write every request here')
    a = ap.parse_args()

    robot = bosdyn.client.create_standard_sdk('grid_rate_check').create_robot(a.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
    skew_s = robot.time_sync.get_robot_clock_skew().ToTimedelta().total_seconds()
    print(f"Connected to {a.hostname}, clock skew {1000 * skew_s:.1f} ms. Layers: {', '.join(LAYERS)}.\n")
    local_grid = spot_grid.LocalGrid(robot)
    state_client = robot.ensure_client(RobotStateClient.default_service_name)
    # Which frames the grid carries: if the ground plane ('gpe') is there, the scan would not
    # need the robot state request for the ground height (the second round trip of a scan).
    try:
        lg0 = list(local_grid.local_grid_client.get_local_grids(LAYERS[:1], timeout=spot_grid.SCAN_TIMEOUT_S))[0]
        snap = lg0.local_grid.transforms_snapshot
        print(f"Frames in the grid snapshot: {', '.join(sorted(snap.child_to_parent_edge_map.keys()))}")
        st = state_client.get_robot_state().kinematic_state.transforms_snapshot
        print(f"Frames in the robot state: {', '.join(sorted(st.child_to_parent_edge_map.keys()))}\n")
    except Exception as e:
        print(f"Frames not read: {e!r}\n")

    all_rows = []
    for period in [float(x) for x in a.periods.split(',')]:
        rows = measure(local_grid, local_grid.local_grid_client, state_client, skew_s, period, a.seconds)
        print(summary(rows, period) + '\n')
        all_rows += rows
    if a.csv:
        keys = ['period', 't', 'acquisition', 'grid_s', 'decode_s', 'state_s', 'age_s', 'error']
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in all_rows:
                w.writerow({k: r.get(k, '') for k in keys})
        print(f"Every request in {a.csv}")


if __name__ == '__main__':
    main()
