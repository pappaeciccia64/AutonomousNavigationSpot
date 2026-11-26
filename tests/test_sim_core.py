"""
Test of the simulation back end WITHOUT the SDK and without Isaac Sim: world (signed
distance, local grid window), link and server, controller, the small GraphNav. It starts the
kinematic simulator by itself on a free port.

    python3 tests/test_sim_core.py

The SDK side (spot_explorer/sim/robot.py) is checked by tools/sim_selftest.py, on a computer
where bosdyn is installed.
"""
import math
import os
import socket
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from spot_explorer.sim import controller as ctl      # noqa: E402
from spot_explorer.sim import graph as G             # noqa: E402
from spot_explorer.sim.link import SimLink, SimLinkError, parse_address    # noqa: E402
from spot_explorer.sim.world import World            # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def walk_to(c, x, y, v=0.5, w=0.8):
    st = c.state()
    heading = math.atan2(y - st['y'], x - st['x'])
    c.wait(c.go_to(st['x'], st['y'], heading, v, w, 20))
    return c.wait(c.go_to(x, y, heading, v, w, 40))


def test_world():
    print("World: signed distance and local grid")
    for shape in ({"type": "box", "x": 2.6, "y": 0.9, "sx": 0.8, "sy": 0.6},
                  {"type": "box", "x": 2.3, "y": 1.2, "sx": 1.0, "sy": 0.4, "yaw_deg": 30},
                  {"type": "cylinder", "x": 2.2, "y": 0.8, "r": 0.3}):
        w = World([shape])
        # A point on the surface, one inside, one outside along +x of the shape.
        a = math.radians(shape.get('yaw_deg', 0.0))
        half = shape.get('r', 0.5 * shape.get('sx', 0.0))
        def at(d):
            return w.clearance(shape['x'] + (half + d) * math.cos(a), shape['y'] + (half + d) * math.sin(a))
        check(abs(at(0.0)) < 1e-9 and abs(at(0.37) - 0.37) < 1e-9 and abs(at(-0.1) + 0.1) < 1e-9,
              f"{shape['type']} (yaw {shape.get('yaw_deg', 0)}): distance 0 on the surface, +0.37 "
              f"outside, -0.10 inside")
    w = World.from_file(os.path.join(ROOT, 'sim', 'worlds', 'boxes.json'))
    g = w.local_grid(2.0, 1.0)
    n, cs, (ox, oy) = g['n'], g['cell'], g['origin']
    check(n == 128 and abs(cs - 0.03) < 1e-12, "window of 128 x 128 cells of 3 cm")
    check(abs(ox + n * cs / 2 - 2.0) <= cs and abs(oy + n * cs / 2 - 1.0) <= cs
          and abs(ox / cs - round(ox / cs)) < 1e-6, "window centred on the robot, origin on whole cells")
    r, c = int((0.9 - oy) / cs), int((2.6 - ox) / cs)
    check(g['obstacle_distance'][r, c] < -0.25, "cell [row = y, column = x] at the box centre is inside")
    check(g['obstacle_distance'][int((0.2 - oy) / cs), int((1.0 - ox) / cs)] > 0.5, "free cell is positive")
    check(np.array_equal(g['terrain'], w.local_grid(2.0, 1.0)['terrain']),
          "two requests for the same window give the same terrain")
    check(abs(float(g['terrain'].mean())) < 0.002 and 0.002 < float(g['terrain'].std()) < 0.006,
          "terrain flat at the ground height with ~4 mm of noise")
    check(parse_address('') == ('127.0.0.1', 8765) and parse_address('10.0.0.5:9000') == ('10.0.0.5', 9000)
          and parse_address(':9001') == ('127.0.0.1', 9001), "simulator address parsing")


def test_terrain_and_lidar():
    print("Terrain and LiDAR")
    from spot_explorer.sim.lidar import Lidar, quat_to_matrix
    sys.path.insert(0, os.path.join(ROOT, 'sim'))
    import kinematic_server
    flat = World.from_file(os.path.join(ROOT, 'sim', 'worlds', 'boxes.json'))
    check(flat.is_flat and flat.height_at(3.0, 2.0) == 0.0, "a world without terrain is flat at ground_z")
    xs = np.arange(-15, 45, 0.1)
    gx, gy = np.meshgrid(xs, np.arange(-30, 30, 0.1))
    for name, max_deg in (('forest_hills', 13.0), ('slope', 7.0)):
        w = World.from_file(os.path.join(ROOT, 'sim', 'worlds', name + '.json'))
        h = w.height(gx, gy)
        dy, dx = np.gradient(h, 0.1)
        slope = np.degrees(np.arctan(np.hypot(dx, dy)))
        start = np.hypot(gx, gy) < 1.2
        check(abs(w.height_at(0, 0)) < 1e-9 and np.abs(h[start]).max() < 0.01 and slope[start].max() < 1.5,
              f"{name}: flat and at height 0 where the robot starts")
        check(3.0 < slope.max() < max_deg, f"{name}: slopes up to {slope.max():.1f} deg, heights "
                                           f"{h.min():+.2f}..{h.max():+.2f} m")
    w = World.from_file(os.path.join(ROOT, 'sim', 'worlds', 'slope.json'))
    check(abs(w.gradient_at(6, 2)[0] - 0.12) < 0.002 and abs(w.gradient_at(6, 2)[1]) < 1e-6,
          "slope: the ramp is 12% uphill ahead")
    g = w.local_grid(6.0, 2.0)
    ox = g['origin'][0]
    rise = float(np.polyfit(ox + (np.arange(g['n']) + 0.5) * g['cell'], g['terrain'].mean(axis=0), 1)[0])
    check(abs(rise - 0.12) < 0.005, f"local grid terrain layer shows the same ramp ({100 * rise:.1f}%)")
    q = kinematic_server.body_quaternion(w, 6.0, 2.0, 0.0)
    rot = quat_to_matrix(q)
    check(abs(rot[2, 0] / rot[0, 0] - 0.12) < 0.002 and abs(rot[2, 1]) < 1e-6,
          "kinematic robot: nose up by the slope when it faces uphill, no roll")
    q = kinematic_server.body_quaternion(w, 6.0, 2.0, math.pi / 2)
    rot = quat_to_matrix(q)
    check(abs(rot[2, 0]) < 1e-6 and rot[2, 1] < -0.1, "facing across the slope: no pitch, rolled downhill side")

    lidar = Lidar()
    check(lidar.dirs.shape == (16 * 360, 3) and np.allclose(np.linalg.norm(lidar.dirs, axis=1), 1.0),
          "LiDAR: 16 rings x 360 rays, unit directions")
    s = lidar.scan(World(), (0, 0, 0.52), (1, 0, 0, 0))
    r0 = s['ranges'][s['ring'] == 0]
    check(len(r0) == 360 and np.allclose(r0, 0.74 / math.sin(math.radians(15)), atol=1e-6),
          "flat ground: the -15 deg ring lands at the distance trigonometry gives (2.86 m)")
    check(not (s['ring'] >= 8).any(), "flat ground: rays at or above the horizon return nothing")
    s = lidar.scan(World([{"type": "cylinder", "x": 5, "y": 0, "r": 0.2, "h": 4}]), (0, 0, 0.52), (1, 0, 0, 0))
    front = s['points_sensor'][(np.abs(s['points_sensor'][:, 1]) < 0.01) & (s['ring'] == 8)]
    check(len(front) == 1 and abs(front[0, 0] - 4.8) < 1e-3, "trunk 5 m ahead: hit at 4.80 m (its near side)")
    s = lidar.scan(World([{"type": "box", "x": 4, "y": 0, "sx": 1, "sy": 2, "yaw_deg": 45, "h": 2}]),
                   (0, 0, 0.52), (1, 0, 0, 0))
    front = s['points_sensor'][(np.abs(s['points_sensor'][:, 1]) < 0.01) & (s['ring'] == 8)]
    check(len(front) == 1 and abs(front[0, 0] - (4 - 0.5 * math.sqrt(2))) < 1e-3, "box turned 45 deg: hit on the right face")
    w = World.from_file(os.path.join(ROOT, 'sim', 'worlds', 'forest_hills.json'))
    pose_q = kinematic_server.body_quaternion(w, 4.0, 3.0, 0.6)
    s = lidar.scan(w, (4.0, 3.0, w.height_at(4.0, 3.0) + 0.52), pose_q)
    p = s['points_world']
    on_ground = np.abs(p[:, 2] - w.height(p[:, 0], p[:, 1])) < 0.01
    on_shape = w.signed_distance(p[:, 0], p[:, 1]) < 0.01      # a side, or the top of a low log
    check(len(p) > 1500 and (on_ground | on_shape).all() and on_shape.sum() > 100,
          f"hilly forest, tilted robot: every one of the {len(p)} hits lies on the ground or on a shape")
    back = lidar.scan(w, (4.0, 3.0, w.height_at(4.0, 3.0) + 0.52), pose_q)
    rot = quat_to_matrix(pose_q)
    check(np.allclose(back['origin'] + back['points_sensor'] @ rot.T, back['points_world'], atol=1e-9),
          "points in the sensor frame and in the world agree through the body pose")


def test_motion_and_graph(port):
    print("Link and controller")
    link = SimLink(port=port)
    check(link.backend == 'kinematic' and len(link.world()['shapes']) == 3,
          "link: hello and world served by the simulator")
    c = ctl.BaseController(link)
    st = c.state()
    check(abs(st['x']) < 1e-6 and abs(st['z'] - 0.52) < 1e-6, "state: robot at the origin, body 0.52 m up")
    status = c.wait(c.go_to(0, 0, math.radians(90), 0.5, 0.6, 20))
    st = c.state()
    check(status == ctl.AT_GOAL and abs(math.degrees(st['yaw']) - 90) < 5 and math.hypot(st['x'], st['y']) < 0.05,
          f"rotation in place of 90 deg ({math.degrees(st['yaw']):.1f} reached)")
    t0 = time.time()
    status = c.wait(c.go_to(0.0, 1.5, math.radians(90), 0.5, 0.6, 30))
    st, dt = c.state(), time.time() - t0
    check(status == ctl.AT_GOAL and math.hypot(st['x'], st['y'] - 1.5) < 0.06,
          f"straight leg of 1.5 m: arrived within 6 cm in {dt:.1f} s")
    check(dt > 1.5 / 0.5, "speed limit respected (1.5 m at 0.5 m/s takes more than 3 s)")
    status = c.wait(c.go_to(0.8, 1.5, math.radians(90), 0.3, 0.6, 30))
    st = c.state()
    check(status == ctl.AT_GOAL and abs(st['x'] - 0.8) < 0.06 and abs(math.degrees(st['yaw']) - 90) < 5,
          "walk sideways keeping the heading")
    check(c.wait(c.go_to(5, 5, 0, 0.5, 0.6, 0.5)) == ctl.TIMED_OUT, "a goal past its deadline times out")
    gid = c.go_to(5, 5, 0, 0.5, 0.6, 30)
    time.sleep(0.3)
    c.stop()
    time.sleep(1.5)
    a = c.state()
    time.sleep(0.4)
    b = c.state()
    check(c.status(gid) == ctl.STOPPED and math.hypot(a['x'] - b['x'], a['y'] - b['y']) < 0.005,
          "stop: the goal is dropped and the robot stands still")
    g1 = c.go_to(0.8, 1.5, None, 0.5, 0.6, 30)
    g2 = c.go_to(0.8, 0.9, 0.0, 0.5, 0.6, 30)
    check(c.status(g1) == ctl.OVERRIDDEN, "a new goal overrides the one in progress")
    c.wait(g2)
    status = c.wait(c.go_to(2.6, 0.9, 0.0, 0.5, 0.6, 6))
    st = c.state()
    check(status == ctl.TIMED_OUT and 1.90 < st['x'] < 2.0,
          f"a box stops the kinematic robot (x = {st['x']:.2f}, face at 2.20 minus 0.25 of body)")

    print("GraphNav")
    gr = G.SimGraph(c)
    check(gr.localization() == (None, None), "empty map: not localized")
    gr.start_recording()
    wp0 = gr.create_waypoint('wp_0')
    for x, y in ((0.8, 2.2), (2.0, 2.2), (2.0, 3.6)):
        walk_to(c, x, y)
    wp1 = gr.create_waypoint('wp_1')
    n_auto = sum(w['auto'] for w in gr.waypoints.values())
    check(n_auto >= 5 and len(gr.edges()) == len(gr.waypoints) - 1,
          f"recording: {n_auto} automatic waypoints along the way, linked in a chain")
    wid, rel = gr.localization()
    check(wid == wp1['id'] and math.hypot(rel[0], rel[1]) < 0.02, "localized on the waypoint just created")
    hops = len(gr.route(wp1['id'], wp0['id']))
    gr.add_edge(wp1['id'], wp0['id'])
    check(hops >= 6 and gr.route(wp1['id'], wp0['id']) == [wp1['id'], wp0['id']],
          f"route along the chain ({hops} waypoints), direct once the shortcut edge exists")
    del gr.adj[wp1['id']][wp0['id']], gr.adj[wp0['id']][wp1['id']]
    gr.stop_recording()
    t0, nid, status = time.time(), None, G.FOLLOWING
    while time.time() - t0 < 90 and status == G.FOLLOWING:
        nid = gr.navigate(wp0['id'], 1.0, 0.5, 0.6, final_yaw_offset=math.radians(-90), command_id=nid)
        time.sleep(0.5)
        status = gr.navigation_status(nid)
    st, p0 = c.state(), wp0['pose']
    yaw_err = math.degrees(ctl.wrap(st['yaw'] - (ctl.yaw_of(p0[3]) + math.radians(-90))))
    check(status == G.REACHED and math.hypot(st['x'] - p0[0], st['y'] - p0[1]) < 0.12 and abs(yaw_err) < 6,
          f"navigation back to wp_0 along the edges, arrival heading as requested (off by {yaw_err:+.1f} deg)")
    check(gr.localization()[0] == wp0['id'], "after the trip: localized on wp_0")
    nid = gr.navigate(wp1['id'], 1.0, 0.5, 0.6)
    time.sleep(2.2)
    a = c.state()
    time.sleep(0.4)
    b = c.state()
    check(gr.navigation_status(nid) == G.TIMED_OUT and math.hypot(a['x'] - b['x'], a['y'] - b['y']) < 0.01,
          "a navigation command that is not renewed expires and the robot stops")
    nid = gr.navigate('not-a-waypoint', 1.0, 0.5, 0.6)
    time.sleep(0.3)
    check(gr.navigation_status(nid) == G.NO_ROUTE, "unknown destination: no route")
    gr.clear()
    check(gr.localization() == (None, None), "clear: map empty and robot not localized")
    a = (1.0, 2.0, 0.5, G.q_from_yaw(0.7))
    b = (3.0, -1.0, 0.5, G.q_from_yaw(-1.1))
    r = G.pose_mul(a, G.pose_mul(G.pose_inverse(a), b))
    check(all(abs(u - v) < 1e-9 for u, v in zip(r[:3], b[:3])), "pose algebra: a * (a^-1 * b) = b")
    c.close()
    link.close()
    try:
        SimLink(port=free_port(), timeout_s=0.5)
        check(False, "no simulator: SimLinkError")
    except SimLinkError:
        check(True, "no simulator on the port: clear error instead of hanging")


def main():
    test_world()
    test_terrain_and_lidar()
    port = free_port()
    sim = subprocess.Popen([sys.executable, os.path.join(ROOT, 'sim', 'kinematic_server.py'),
                            '--world', os.path.join(ROOT, 'sim', 'worlds', 'boxes.json'),
                            '--port', str(port), '--duration', '300'], stdout=subprocess.DEVNULL)
    time.sleep(1.0)
    try:
        test_motion_and_graph(port)
    finally:
        sim.terminate()
    print(f"\n{'ALL OK' if not FAILS else str(len(FAILS)) + ' FAILED'}")
    return not FAILS


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
