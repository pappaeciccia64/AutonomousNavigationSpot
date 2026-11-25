#!/usr/bin/env python3
"""
The whole mission (mission/run.py, unchanged) against a simulated robot (sim/, SIMULATOR.md).

    python3 tools/run_sim_mission.py --world hill
    python3 tools/run_sim_mission.py --world walls -- --goals frontier   # frontier goals
    python3 tools/run_sim_mission.py --world slope --out /tmp/sim_slope -- --rows 3 --cols 4

The robot is sim/kinematic_world.KinematicWorld (no physics: it follows the commanded path
exactly, stops before obstacles and slopes above 30 degrees); the local grids reach the
mission as real SDK protos and are decoded by its own code; GraphNav is in memory (its trips
teleport the robot). No lease, E-Stop or login. Another simulator: a SimBackend subclass in
place of KinematicWorld (see --help of sim.backend and SIMULATOR.md).

Logs and data under --out (MissionLogs/, MissionMap/), readable with the usual tools.
"""
import argparse
import ast
import contextlib
import importlib
import os
import sys
import time
from types import SimpleNamespace as NS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import bosdyn.client.lease                                       # noqa: E402
from sim import worlds                                           # noqa: E402
from sim.spot_adapter import SimRobot                            # noqa: E402
from spot_explorer.mission import run as mission_run             # noqa: E402
from spot_explorer.robot import connection                       # noqa: E402


def _set(assignment):
    name, value = assignment.split('=', 1)
    mod, attr = name.rsplit('.', 1)
    for prefix in ('', 'spot_explorer.', 'spot_explorer.mission.', 'spot_explorer.planning.',
                   'spot_explorer.perception.', 'spot_explorer.robot.'):
        try:
            module = importlib.import_module(prefix + mod)
            break
        except ImportError:
            continue
    else:
        sys.exit(f"no module {mod}")
    if not hasattr(module, attr):
        sys.exit(f"{module.__name__} has no {attr}")
    setattr(module, attr, ast.literal_eval(value))
    print(f"[SIM] {module.__name__}.{attr} = {value}")


def _variants(folder, names):
    """Turn on variants of the local varianti/ folder (its run.apply), as varianti/run.py does."""
    import importlib.util
    path = os.path.join(folder, 'run.py')
    if not os.path.exists(path):
        sys.exit(f"no varianti/run.py in {folder}: give the folder with --variants-dir")
    sys.path.insert(0, folder)
    spec = importlib.util.spec_from_file_location('varianti_run', path)
    v = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(v)
    unknown = [n for n in names if n not in v.available()]
    if unknown:
        sys.exit(f"unknown variants {unknown}; known: {', '.join(v.available())}")
    v.apply(names)
    print(f"[SIM] Variants on: {', '.join(names)}")


def plug(robot, out):
    """The mission start-up on the simulated robot: login, E-Stop, lease and the stand are
    skipped; the data go under out."""
    connection.setLogInfo = lambda options: (robot, NS(), robot.spot, None)
    connection.SimpleEstop = lambda r, n: NS(stop=lambda: None, allow=lambda: None, release=lambda: None)
    bosdyn.client.lease.LeaseKeepAlive = lambda *a, **k: contextlib.nullcontext()
    mission_run.blocking_stand = lambda *a, **k: None
    mission_run.DATA_ROOT = out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 epilog="Arguments after -- go to run_mission.py.")
    ap.add_argument('--world', choices=worlds.SCENES, default='open')
    ap.add_argument('--seed', type=int, default=1, help='seed of the boxes of the scene')
    ap.add_argument('--out', default=None, help='data folder (default /tmp/sim_missions/<world>_<time>)')
    ap.add_argument('--set', action='append', default=[], metavar='MODULE.NAME=VALUE')
    ap.add_argument('--with', dest='variants', default='',
                    help='variants of the local varianti/ folder to turn on, comma-separated (varianti/run.py --list)')
    ap.add_argument('--variants-dir', default=os.path.join(os.path.dirname(HERE), 'varianti'),
                    help='the varianti/ folder (default: next to tools/)')
    a, rest = ap.parse_known_args()
    rest = [r for r in rest if r != '--']
    out = a.out or os.path.join('/tmp/sim_missions', f"{a.world}_{time.strftime('%d-%m_%H-%M-%S')}")
    os.makedirs(out, exist_ok=True)
    if a.variants:
        _variants(a.variants_dir, [n.strip() for n in a.variants.split(',') if n.strip()])
    for s in a.set:
        _set(s)
    world, rows, cols, cell = worlds.scene(a.world, a.seed)
    robot = SimRobot(world)
    plug(robot, out)
    print(f"[SIM] World '{a.world}' (seed {a.seed}): {len(world.boxes)} boxes, ground {world.ground or 'flat'}; "
          f"grid {rows} x {cols} cells of {cell} m; data in {out}")
    sys.argv = ['run_mission.py', '--rows', str(rows), '--cols', str(cols), '--cell', str(cell)] + rest
    t0 = time.time()
    ok = mission_run.main()
    print(f"[SIM] Mission ended in {time.time() - t0:.0f} s, {len(world.track)} track points, "
          f"{len(robot.graph_nav.trips)} GraphNav trips; data in {out}")
    return 0 if ok in (None, True) else 1


if __name__ == '__main__':
    sys.exit(main())
