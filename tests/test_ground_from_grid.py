#!/usr/bin/env python3
"""
spot_grid.GROUND_FROM_GRID without the robot: a real LocalGrid proto of the Boston Dynamics
SDK (obstacle_distance, 16 x 16 cells) whose snapshot carries, or not, the ground plane frame.

    python3 tests/test_ground_from_grid.py

Checks: switch off, the ground height comes from the robot state request as before; on, from
the grid when it carries 'gpe' (no request), from the request when it does not; the points
of the layer are the same either way when the two heights agree.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np                                               # noqa: E402
from bosdyn.api import geometry_pb2, local_grid_pb2, robot_state_pb2  # noqa: E402
from spot_explorer.perception import spot_grid                   # noqa: E402

FAILS = []
GROUND_Z = -0.48


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def snapshot(with_gpe, gpe_z=GROUND_Z):
    snap = geometry_pb2.FrameTreeSnapshot()
    def edge(child, parent, x=0.0, y=0.0, z=0.0):
        e = snap.child_to_parent_edge_map[child]
        e.parent_frame_name = parent
        e.parent_tform_child.position.x, e.parent_tform_child.position.y, e.parent_tform_child.position.z = x, y, z
        e.parent_tform_child.rotation.w = 1.0
    edge('vision', '')
    edge('body', 'vision', 3.0, 2.0, 0.0)
    edge('grid', 'vision', 1.0, 0.0, 0.0)
    if with_gpe:
        edge('gpe', 'vision', 3.0, 2.0, gpe_z)
    return snap


def grid_proto(with_gpe):
    n = 16
    lg = local_grid_pb2.LocalGridResponse(local_grid_type_name='obstacle_distance')
    g = lg.local_grid
    g.extent.cell_size, g.extent.num_cells_x, g.extent.num_cells_y = 0.03, n, n
    g.cell_format = local_grid_pb2.LocalGrid.CELL_FORMAT_INT16
    g.cell_value_scale, g.cell_value_offset = 0.001, 0.0
    g.encoding = local_grid_pb2.LocalGrid.ENCODING_RAW
    g.data = np.full(n * n, 500, np.int16).tobytes()
    g.frame_name_local_grid_data = 'grid'
    g.transforms_snapshot.CopyFrom(snapshot(with_gpe))
    return lg


class StateClient:
    def __init__(self):
        self.calls = 0

    def get_robot_state(self, timeout=None):
        self.calls += 1
        st = robot_state_pb2.RobotState()
        st.kinematic_state.transforms_snapshot.CopyFrom(snapshot(True))
        return st


def run(switch, with_gpe):
    spot_grid.GROUND_FROM_GRID = switch
    lg = spot_grid.LocalGrid.__new__(spot_grid.LocalGrid)
    client = StateClient()
    pts, vals, _ = lg.create_vtk_obstacle_grid([grid_proto(with_gpe)], client, timeout=1.0)
    return pts, vals, client.calls, getattr(lg, 'ground_from_grid', None)


def main():
    print("== switch off: robot state request, as before ==")
    pts0, vals0, calls, cnt = run(False, True)
    check(calls == 1, f"one robot state request ({calls})")
    check(np.allclose(pts0[:, 2], GROUND_Z), f"points at the ground height ({pts0[0, 2]:.3f})")
    check(np.allclose(vals0, 0.5), "distances decoded")

    print("== switch on, the grid carries 'gpe': no request ==")
    pts1, vals1, calls, cnt = run(True, True)
    check(calls == 0, f"no robot state request ({calls})")
    check(np.allclose(pts1, pts0), "same points as with the request")
    check(cnt == {'grid': 1, 'robot_state': 0}, f"counted ({cnt})")

    print("== switch on, the grid without 'gpe': the request as before ==")
    pts2, _, calls, cnt = run(True, False)
    check(calls == 1, f"one robot state request ({calls})")
    check(np.allclose(pts2, pts0), "same points")
    check(cnt == {'grid': 0, 'robot_state': 1}, f"counted ({cnt})")

    spot_grid.GROUND_FROM_GRID = False
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
