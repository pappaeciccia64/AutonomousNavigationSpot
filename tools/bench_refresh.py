"""
Bench test of the edge refresh during a continuous walk (no robot, no SDK needed beyond
the imports of the test world): the robot of the end-to-end test world walks a straight
line in small steps, as it would with the scan running while it walks, and at every step
the refresh runs on the new window. It prints the time per refresh and how many edges
were recomputed, with INCREMENTAL_REFRESH off and on, for the refresh radius of the
mission and for the whole window.

    python3 tools/bench_refresh.py                    # step 0.15 m, random nodes 5/m2
    python3 tools/bench_refresh.py --step 0.5 --density 9

The world's heights get new noise at every step (4 mm), so with 'off' every weight follows
the last scan and with 'on' the one of the evaluation kept: the weights differ by that noise.
The validity of the edges must be the same.
"""
import argparse
import contextlib
import io
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tests'))

import test_e2e_loop as e2e                                                  # noqa: E402
from spot_explorer import config                                             # noqa: E402
from spot_explorer.planning import edge_evaluation, mission_graph, roadmap   # noqa: E402


def walk(step, density, seed, radius, incremental):
    edge_evaluation.REFRESH_RADIUS_M = radius
    edge_evaluation.INCREMENTAL_REFRESH = incremental
    # Boxes beside the line, so that occupancy and live discard have something to do.
    e2e.WORLD = e2e.World([(4.0, 5.0, 1.0, 2.6), (6.5, 7.5, 4.2, 6.0), (8.5, 9.5, 2.0, 3.6)], ghosts=True)
    e2e.ROBOT = e2e.Robot(2.0, 3.5, 0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        mg = mission_graph.build_mission_graph(
            rows=2, cols=2, cell_size_m=5.0, spacing_m=0.25, rings=None, origin_xy=(3.5, 3.5),
            origin_yaw=0.0, min_edge_length=config.PRM_MIN_EDGE_M,
            max_edge_length=config.PRM_MAX_EDGE_M, connection_radius=config.PRM_CONNECTION_RADIUS_M,
            verbose=False, sampling='random', distribution='poisson', density_per_m2=density, seed=seed)
        prm = roadmap.PRM(min_edge_length=config.PRM_MIN_EDGE_M, max_edge_length=config.PRM_MAX_EDGE_M,
                          connection_radius=config.PRM_CONNECTION_RADIUS_M)
        prm.mission_graph = mg
        mg.into_prm(prm)
    lg = e2e.FakeLocalGrid()
    N, CS = e2e.N, e2e.CS
    times, counts, edges_valid = [], [], []
    while e2e.ROBOT.x < 10.0:
        pts, od, terr, valid, _ = e2e.window()
        tc, iv = lg.correct_terrain(terr, valid, N, N, cell_size=CS)
        grad, rough = lg.compute_gradient_and_roughness(tc, valid, N, N, CS, is_valid=iv)
        ox = (int(np.floor(e2e.ROBOT.x / CS)) - N // 2) * CS
        oy = (int(np.floor(e2e.ROBOT.y / CS)) - N // 2) * CS
        prm.update_local_grid_data(np.reshape(grad, (N, N)), np.reshape(rough, (N, N)), ox, oy, CS,
                                   terrain_values_2d=np.reshape(tc, (N, N)),
                                   valid_2d=np.reshape(iv, (N, N)))
        prm.set_live_obstacles(np.reshape(od, (N, N)), None, ox, oy, CS, e2e.ROBOT.x, e2e.ROBOT.y, 0.0)
        near = min(prm.nodes, key=lambda i: np.hypot(prm.nodes[i][0] - e2e.ROBOT.x,
                                                     prm.nodes[i][1] - e2e.ROBOT.y))
        prm.set_robot_node(near, 0.0)
        t0 = time.perf_counter()
        with contextlib.redirect_stdout(io.StringIO()):
            prm.refresh_local_edge_weights(global_map=None)
        times.append(time.perf_counter() - t0)
        counts.append(prm.refresh_counts_last)
        edges_valid.append(sum(1 for v in prm.edge_validity.values() if v))
        e2e.ROBOT.x += step
    return np.array(times), np.array(counts), prm


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--step', type=float, default=0.15, help='metres between two refreshes (default 0.15)')
    ap.add_argument('--density', type=float, default=5.0, help='random nodes per m2 (default 5)')
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    ref = {}
    for radius in (edge_evaluation.REFRESH_RADIUS_M, None):
        for inc in (False, True):
            t, c, prm = walk(a.step, a.density, a.seed, radius, inc)
            state = {k: (v, prm._edge_weight(*k)) for k, v in prm.edge_validity.items()}
            if not inc:
                ref[radius] = state
            line = (f"radius {'window' if radius is None else f'{radius:.2f} m'}  incremental "
                    f"{'on ' if inc else 'off'}: {len(t)} refreshes, mean {1000 * t.mean():.0f} ms, "
                    f"p90 {1000 * np.percentile(t, 90):.0f} ms")
            if inc:
                tot = c.sum(axis=0)
                keys = set(state) | set(ref[radius])
                n_valid = sum(1 for k in keys if state.get(k, (None,))[0] != ref[radius].get(k, (None,))[0])
                rel = [abs(state[k][1] - ref[radius][k][1]) / ref[radius][k][1] for k in keys
                       if k in state and k in ref[radius] and state[k][1] and ref[radius][k][1]]
                line += (f"\n    slopes recomputed {tot[0]}, profile kept {tot[1]}, untouched {tot[2]}; "
                         f"at the end, compared with 'off': validity different on {n_valid} edges, "
                         f"weights {100 * np.median(rel):.1f}% apart (median), {100 * max(rel):.0f}% at most")
            print(line)


if __name__ == '__main__':
    main()
