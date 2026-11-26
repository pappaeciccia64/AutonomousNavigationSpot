#!/usr/bin/env python3
"""
Kinematic simulator: a Spot that simply integrates the velocity it is told, with no physics,
no legs and no GPU. It speaks the same protocol as sim/isaac_spot_server.py, so the mission
cannot tell them apart.

What it is for: trying the mission logic on any computer (the laptop too) and checking the
simulation back end without starting Isaac Sim. What it is NOT: a test of locomotion: on a
slope the body simply follows the ground and tilts with it, at the commanded speed.

    python3 sim/kinematic_server.py --world sim/worlds/boxes.json
    python3 run_mission.py --sim --rows 2 --cols 3            # in another terminal

Needs only numpy. Runs with any Python 3.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from spot_explorer.sim.server import DEFAULT_PORT, SimServer   # noqa: E402
from spot_explorer.sim.world import World                      # noqa: E402

BODY_HEIGHT_M = 0.52       # standing body height above the ground
BODY_RADIUS_M = 0.25       # the body cannot get closer than this to a shape
RESPONSE_S = 0.25          # first-order lag of the velocity: Spot does not start instantly
RATE_HZ = 100.0


def body_quaternion(world, x, y, yaw):
    """Body orientation (w, x, y, z): heading yaw, and lying on the slope of the ground under
    it (nose up when the ground rises ahead, left side up when it rises to the left)."""
    if world.is_flat:
        return (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
    gx, gy = world.gradient_at(x, y)
    c, s = math.cos(yaw), math.sin(yaw)
    pitch = -math.atan(c * gx + s * gy)          # slope ahead
    roll = math.atan(-s * gx + c * gy)           # slope to the left
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Kinematic Spot simulator")
    ap.add_argument('--world', default=None, help="world file (see spot_explorer/sim/world.py)")
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--start', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                    metavar=('X', 'Y', 'YAW_DEG'), help="starting pose (default: origin, facing +x)")
    ap.add_argument('--speed', type=float, default=1.0, help="simulated seconds per real second")
    ap.add_argument('--duration', type=float, default=0.0, help="stop after this many seconds (0 = never)")
    args = ap.parse_args(argv)

    world = World.from_file(args.world) if args.world else World()
    server = SimServer('kinematic', world.to_dict(), host=args.host, port=args.port)
    x, y, yaw = args.start[0], args.start[1], math.radians(args.start[2])
    vx = vy = wz = 0.0
    t, dt = 0.0, 1.0 / RATE_HZ
    print("[SIM] Kinematic simulator on %s:%d, world '%s' (%d shapes), start (%.2f, %.2f, %.0f deg)."
          % (server.address[0], server.address[1], world.name or 'empty', len(world.shapes),
             x, y, math.degrees(yaw)))
    t_wall0 = time.time()
    try:
        while not args.duration or time.time() - t_wall0 < args.duration:
            tick = time.time()
            cx, cy, cw = server.command()
            a = min(1.0, dt * args.speed / RESPONSE_S)
            vx, vy, wz = vx + a * (cx - vx), vy + a * (cy - vy), wz + a * (cw - wz)
            step = dt * args.speed
            c, s = math.cos(yaw), math.sin(yaw)
            nx, ny = x + (c * vx - s * vy) * step, y + (s * vx + c * vy) * step
            # A solid stops the body: no step that takes it closer than its radius, unless it
            # is already inside and the step takes it out.
            d_new = world.clearance(nx, ny)
            if d_new >= BODY_RADIUS_M or d_new > world.clearance(x, y):
                x, y = nx, ny
            yaw = math.atan2(math.sin(yaw + wz * step), math.cos(yaw + wz * step))
            t += step
            server.set_state(t, (x, y, world.height_at(x, y) + BODY_HEIGHT_M),
                             body_quaternion(world, x, y, yaw))
            rest = dt - (time.time() - tick)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
    print("[SIM] Kinematic simulator stopped.")


if __name__ == '__main__':
    main()
