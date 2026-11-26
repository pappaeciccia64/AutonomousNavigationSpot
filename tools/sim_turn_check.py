#!/usr/bin/env python3
"""
How well the simulated Spot follows a velocity command: turning on the spot and walking.

With Isaac Sim running (sim/run_isaac.sh <world>) and NO mission connected:

    python3 tools/sim_turn_check.py              # 127.0.0.1:8765
    python3 tools/sim_turn_check.py --sim HOST:PORT

For each command it holds it for a few seconds and prints, measured over the second half
(after the robot has picked up speed):
  - the rate on the SIMULATED clock: what the walking policy does with the command;
  - the rate on the WALL clock: what the mission sees (= simulated x real-time factor);
  - the real-time factor of that stretch.
If the simulated rate is close to the command, the slowness is the real-time factor (the
computer); if it is well below, it is the walking policy itself.
The robot ends where it started roughly (it turns both ways and walks back).
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from spot_explorer.sim.link import SimLink, parse_address  # noqa: E402

RATE_HZ = 25.0
# (label, vx m/s, vy m/s, wz rad/s, seconds)
PHASES = [
    ("still", 0.0, 0.0, 0.0, 2.0),
    ("turn +0.3 rad/s", 0.0, 0.0, 0.3, 6.0),
    ("turn -0.6 rad/s", 0.0, 0.0, -0.6, 6.0),
    ("turn +1.0 rad/s", 0.0, 0.0, 1.0, 6.0),
    ("walk +0.5 m/s", 0.5, 0.0, 0.0, 6.0),
    ("walk -0.5 m/s", -0.5, 0.0, 0.0, 6.0),
    ("walk 0.3 m/s + turn 0.6 rad/s", 0.3, 0.0, 0.6, 6.0),
    ("still", 0.0, 0.0, 0.0, 2.0),
]


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def run_phase(link, vx, vy, wz, seconds):
    """Samples (wall time, sim time, x, y, unwrapped yaw) while the command is held."""
    out, last_yaw, turns = [], None, 0.0
    t_end = time.time() + seconds
    while time.time() < t_end:
        t0 = time.time()
        s = link.step((vx, vy, wz))
        yaw = yaw_of(s['q'])
        if last_yaw is not None:
            turns += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
        else:
            turns = yaw
        last_yaw = yaw
        out.append((time.time(), float(s.get('t', 0.0)), s['p'][0], s['p'][1], turns))
        time.sleep(max(0.0, 1.0 / RATE_HZ - (time.time() - t0)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument('--sim', default='', metavar='HOST:PORT')
    args = ap.parse_args()
    link = SimLink(*parse_address(args.sim))
    print(f"{'command':32s} {'sim rate':>10s} {'wall rate':>10s} {'RTF':>5s}   (asked)")
    for label, vx, vy, wz, seconds in PHASES:
        smp = run_phase(link, vx, vy, wz, seconds)
        half = smp[len(smp) // 2:]
        (w0, s0, x0, y0, a0), (w1, s1, x1, y1, a1) = half[0], half[-1]
        dw, ds = max(w1 - w0, 1e-6), max(s1 - s0, 1e-6)
        if wz != 0.0 and vx == 0.0:
            got, unit, asked = abs(a1 - a0), 'rad/s', abs(wz)
        else:
            got, unit, asked = math.hypot(x1 - x0, y1 - y0), 'm/s', math.hypot(vx, vy)
        extra = ""
        if wz != 0.0 and vx != 0.0:
            extra = f"   turning {abs(a1 - a0) / ds:.2f} rad/s sim (asked {abs(wz):.2f})"
        print(f"{label:32s} {got / ds:8.2f}   {got / dw:8.2f}   {ds / dw:5.2f}   ({asked:.2f} {unit}){extra}")
    link.step((0.0, 0.0, 0.0))


if __name__ == '__main__':
    main()
