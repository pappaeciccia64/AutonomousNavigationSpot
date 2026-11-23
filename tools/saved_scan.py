"""
A scan saved by a mission (MissionMap/<mission>/scans/*.npz, SAVE_SCAN_BUNDLES) packed back
into the messages Spot sent, so that the real decoding and processing code
(perception/spot_grid.py: fetch_scan, process_scan) can run on it without the robot.

    from saved_scan import load, SavedSpot, decode_and_process
    lg, scan = decode_and_process(load(path))

The bundle keeps Spot's raw layers: obstacle_distance and terrain_valid as decoded values,
terrain as heights in VISION with Spot's quantization (terrain_scale, terrain_raw_zero).
Rebuilt as: terrain int16 with that scale and offset (so 'raw 0' means what it meant that
day: the cells never written), the other two float32 with their exact values. Grid frame:
its corner at grid_origin; for terrain its z puts the decoded heights back where they were
(the never-written cells, raw 0, sat at about z 0 in VISION); z = 0 for the other two
layers, because the decoder adds the frame z to the values of every terrain-type layer
(see percorso step 1, note 5).
"""
import numpy as np
from bosdyn.api import local_grid_pb2, robot_state_pb2

GRID_FRAME = 'grid'


def load(path):
    return {k: v for k, v in np.load(path, allow_pickle=True).items()}


def _edge(snapshot, child, parent, x=0.0, y=0.0, z=0.0):
    e = snapshot.child_to_parent_edge_map[child]
    e.parent_frame_name = parent
    p = e.parent_tform_child
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.rotation.w = 1.0


def rebuild_messages(b):
    """The three LocalGridResponse of a scan bundle."""
    n = b['terrain_raw'].shape[0]
    cell = float(b['cell_size'])
    scale, offset = float(b['terrain_scale']), float(b['terrain_raw_zero'])
    terrain_v = b['terrain_raw'].astype(np.float64)
    unwritten = np.asarray(b.get('unwritten', np.zeros(terrain_v.shape, bool)), dtype=bool)
    unwritten_height = float(np.median(terrain_v[unwritten])) if unwritten.any() else None
    frame_z = (unwritten_height - offset) if unwritten_height is not None else -offset
    raw = np.round((terrain_v - frame_z - offset) / scale).astype(np.int16)
    ox, oy = (float(v) for v in b['grid_origin'])
    F = local_grid_pb2.LocalGrid
    out = []
    for name, values, fmt, sc, off, z in (
            ('obstacle_distance', b['obstacle_distance'].astype(np.float32), F.CELL_FORMAT_FLOAT32, 0.0, 0.0, 0.0),
            ('terrain', raw, F.CELL_FORMAT_INT16, scale, offset, frame_z),
            ('terrain_valid', b['terrain_valid_raw'].astype(np.float32), F.CELL_FORMAT_FLOAT32, 0.0, 0.0, 0.0)):
        r = local_grid_pb2.LocalGridResponse(local_grid_type_name=name,
                                             status=local_grid_pb2.LocalGridResponse.STATUS_OK)
        g = r.local_grid
        g.local_grid_type_name = name
        g.extent.cell_size, g.extent.num_cells_x, g.extent.num_cells_y = cell, n, n
        g.frame_name_local_grid_data = GRID_FRAME
        g.cell_format, g.encoding = fmt, F.ENCODING_RAW
        g.cell_value_scale, g.cell_value_offset = sc, off
        g.data = np.ascontiguousarray(values).tobytes()
        _edge(g.transforms_snapshot, 'vision', '')
        _edge(g.transforms_snapshot, GRID_FRAME, 'vision', ox, oy, z)
        out.append(r)
    return out


class SavedSpot:
    """Stands in for Spot's local-grid and robot-state services, answering with one saved scan."""

    def __init__(self, bundle):
        self.bundle = bundle
        self.messages = rebuild_messages(bundle)

    def ensure_client(self, name):
        return self

    def get_local_grids(self, names, **kw):
        return [m for m in self.messages if m.local_grid_type_name in names]

    def get_robot_state(self, **kw):          # the decoder asks only for the ground plane height
        s = robot_state_pb2.RobotState()
        snap = s.kinematic_state.transforms_snapshot
        _edge(snap, 'vision', '')
        _edge(snap, 'gpe', 'vision', *self.bundle['robot_xyz'][:2], float(self.bundle['ground_z']))
        return s


def decode_and_process(bundle, ground_label=None):
    """The real LocalGrid on the saved scan: fetch_scan then process_scan at the saved robot
    pose. Returns (LocalGrid, Scan)."""
    from spot_explorer.perception import spot_grid
    spot = SavedSpot(bundle)
    lg = spot_grid.LocalGrid(spot)
    scan = lg.fetch_scan(spot)
    x, y, _ = bundle['robot_xyz']
    lg.process_scan(scan, x, y, float(bundle['robot_yaw']), ground_label=ground_label)
    return lg, scan
