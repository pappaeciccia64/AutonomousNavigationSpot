"""
bench_fiducial.py -- which fiducials Spot sees right now, and which one the mission would pick.

    python3 tools/bench_fiducial.py            # one reading per second for 10 s
    python3 tools/bench_fiducial.py 30         # for 30 s

Read only: no lease, no motor power, the robot does not move. Works with the robot sitting
too (it only needs to be on). Uses the same function as the mission
(apriltags.find_fiducials) and the same selection rule (the nearest one, or
config.START_FIDUCIAL_ID if it is a number).

Spot only recognizes AprilTag 36h11 fiducials (the Boston Dynamics ones), not ArUco.
"""
import sys
import time
from types import SimpleNamespace

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # the spot_explorer package

from spot_explorer.graphnav import apriltags
from spot_explorer import config as P
from spot_explorer.robot import connection


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    options = SimpleNamespace(name="provaFiducial", hostname="192.168.80.3", verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, _lease, robot_state_client, _meta = connection.setLogInfo(options)
    print(f"Connected. Looking for fiducials for {duration:.0f} s "
          f"({'any ID' if P.START_FIDUCIAL_ID is None else f'only ID {P.START_FIDUCIAL_ID}'}).\n")
    t_end = time.time() + duration
    seen = {}
    while time.time() < t_end:
        found = apriltags.find_fiducials(robot, robot_state_client,
                                             fiducial_id=P.START_FIDUCIAL_ID)
        stamp = time.strftime('%H:%M:%S')
        if found:
            for f in found:
                seen[f['id']] = seen.get(f['id'], 0) + 1
            print(f"{stamp}  the mission would choose: {apriltags.describe_fiducial(found[0])}")
            for f in found[1:]:
                print(f"           also seen: {apriltags.describe_fiducial(f)}")
        else:
            print(f"{stamp}  no fiducial seen")
        time.sleep(1.0)
    print("\nRiepilogo: " + (", ".join(f"tag {k} in {n} readings" for k, n in sorted(seen.items()))
                             if seen else "no fiducial seen. Check that it is an AprilTag "
                                           "36h11, in the light, 1-2 m away and not too oblique."))


if __name__ == '__main__':
    main()
