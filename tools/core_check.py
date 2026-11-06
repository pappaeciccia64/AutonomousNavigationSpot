#!/usr/bin/env python3
"""
Connection check with the robot, READ ONLY: no lease, no E-Stop registered, no command. For
the CORE I/O (robot at 192.168.50.3) before the first mission from there, and anywhere else.

    python3 tools/core_check.py [--hostname 192.168.50.3] [--scans 5]

It prints: login and time sync, battery and motor power, the E-Stop endpoints registered and
their state (with --no-estop the mission needs one already there: the tablet's), who holds
the body lease, and the download time of the local grids the mission uses.
Login: BOSDYN_CLIENT_USERNAME / BOSDYN_CLIENT_PASSWORD, or asked.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import bosdyn.client                                         # noqa: E402
import bosdyn.client.util                                    # noqa: E402
from bosdyn.client.estop import EstopClient                  # noqa: E402
from bosdyn.client.lease import LeaseClient                  # noqa: E402
from bosdyn.client.robot_state import RobotStateClient       # noqa: E402

from spot_explorer.perception import spot_grid                # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--hostname', default=os.environ.get('SPOT_HOSTNAME', '192.168.50.3'))
    ap.add_argument('--scans', type=int, default=5)
    a = ap.parse_args()

    t = time.time()
    robot = bosdyn.client.create_standard_sdk('core_check').create_robot(a.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
    print(f"[OK] login and time sync with {a.hostname} in {time.time() - t:.1f} s "
          f"(clock skew {robot.time_sync.get_robot_clock_skew().ToTimedelta().total_seconds() * 1000:.1f} ms)")

    state = robot.ensure_client(RobotStateClient.default_service_name).get_robot_state()
    bat = state.battery_states[0].charge_percentage.value if state.battery_states else float('nan')
    print(f"[OK] battery {bat:.0f}%, motor power {state.power_state.motor_power_state}")

    estop = robot.ensure_client(EstopClient.default_service_name)
    cfg, st = estop.get_config(), estop.get_status()
    eps = list(cfg.endpoints)
    print(f"[E-STOP] {len(eps)} endpoints registered: "
          + ", ".join(f"{e.role} '{e.name}' (timeout {e.timeout.seconds} s)" for e in eps))
    for s in st.endpoints:
        print(f"         {s.endpoint.name}: {s.stop_level} ({s.time_since_valid_response.seconds} s since its last answer)")
    print(f"         robot stop level: {st.stop_level} (4 = NONE: free to move; 1-2 = stopped)")

    for lease in robot.ensure_client(LeaseClient.default_service_name).list_leases():
        print(f"[LEASE] {lease.resource}: held by '{lease.lease_owner.client_name}' ({lease.lease_owner.user_name})")

    grid = spot_grid.LocalGrid(robot)
    rsc = robot.ensure_client(RobotStateClient.default_service_name)
    times = []
    for _ in range(a.scans):
        t = time.time()
        scan = grid.fetch_scan(rsc)
        times.append(time.time() - t)
        if scan is None:
            print("[SCAN] layers not received")
    if times:
        times.sort()
        print(f"[SCAN] {len(times)} downloads of the local grid: median {times[len(times) // 2]:.3f} s, "
              f"max {times[-1]:.3f} s (from the laptop on WiFi: ~0.08 s median)")


if __name__ == '__main__':
    main()
