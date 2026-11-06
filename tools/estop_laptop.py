#!/usr/bin/env python3
"""
Software E-Stop held by the LAPTOP, for the missions run on the CORE I/O (run_mission.py
--no-estop there). The same SimpleEstop the mission registers when it runs on the laptop
(force_simple_setup, timeout 9 s): if the WiFi drops or this program stops, the robot stops.

    python3 tools/estop_laptop.py [--hostname 192.168.80.3]

Keys: Enter = STOP (cut), 'r' + Enter = release again, 'q' + Enter = quit (the E-Stop
stops answering: the robot stops within 9 s). Start it BEFORE the mission, keep it open.
Login: BOSDYN_CLIENT_USERNAME / BOSDYN_CLIENT_PASSWORD, or asked.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import bosdyn.client                                  # noqa: E402
import bosdyn.client.util                             # noqa: E402

from spot_explorer.robot import connection            # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--hostname', default=os.environ.get('SPOT_HOSTNAME', '192.168.80.3'))
    a = ap.parse_args()
    robot = bosdyn.client.create_standard_sdk('estop_laptop').create_robot(a.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
    estop = connection.SimpleEstop(robot, 'laptop_estop')
    print("E-Stop of the laptop ACTIVE and released. Enter = STOP, r = release, q = quit.")
    while True:
        try:
            cmd = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            cmd = ''
        if cmd == 'q':
            estop.stop()
            print("Quit: E-Stop engaged.")
            return
        if cmd == 'r':
            estop.allow()
        else:
            estop.stop()


if __name__ == '__main__':
    main()
