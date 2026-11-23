"""
Ground height of the obstacle layer from the ground plane frame ('gpe') carried by the grid
itself, when there, instead of a robot state request (spot_grid.LocalGrid._ground_z_of_grid).

Saves the second network round trip of every scan and takes the ground at the instant of the
grid. Without the frame: the robot state request, as in the main code. LocalGrid.ground_from_grid
counts which source was used (the planning thread logs it). tools/grid_rate_check.py says whether
the grids carry the frame.
"""
from bosdyn.client.frame_helpers import GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b

from spot_explorer.perception import spot_grid
from hook import replace

DESCRIPTION = "ground height from the grid's own frame (one network request less per scan)"


def _ground_z_of_grid(self, transforms_snapshot):
    counts = self.__dict__.setdefault('ground_from_grid', {'grid': 0, 'robot_state': 0})
    try:
        z = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME).position.z
    except Exception:
        counts['robot_state'] += 1
        return None
    counts['grid'] += 1
    return float(z)


def apply():
    return replace(spot_grid.LocalGrid, '_ground_z_of_grid', _ground_z_of_grid)
