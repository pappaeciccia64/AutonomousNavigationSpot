"""Spot's local grid: reading the layers, ground filter, corrected terrain, slope, roughness, obstacle mask."""
import numpy as np
from scipy.ndimage import uniform_filter, distance_transform_edt
import time
try:
    import bosdyn.client
    from bosdyn.client.frame_helpers import *
    from bosdyn.client.frame_helpers import get_a_tform_b
    from bosdyn.api import local_grid_pb2
    from bosdyn.client.local_grid import LocalGridClient
except ImportError:
    get_a_tform_b = local_grid_pb2 = LocalGridClient = None
from spot_explorer.perception.geometry import OBSTACLE_THRESHOLD
from spot_explorer.perception.slopes import SLOPE_BASELINE_M
ROUGH_THRESHOLD = 0.15      # local roughness (std-dev of terrain height, m)

# Spot's `terrain_valid` layer contains 0 and 1, but after scale and offset 0 becomes
# +/-5.5e-17 with variable sign: it is compared with 0.5, insensitive to the residue.
TERRAIN_VALID_MIN = 0.5

# =========================================================================
# GROUND HEIGHT FILTER (see LocalGrid.correct_terrain)
#
# The `terrain` heights are relative to the reference fixed at power-on. The ground the
# robot stands on is estimated as the median of the valid cells within GROUND_REF_RADIUS_M;
# cells more than GROUND_HEIGHT_TOLERANCE_M from the ground are treated ASYMMETRICALLY:
#   - ABOVE: not a walkable surface (wall, tree, cell never written). Excluded from
#     slope and roughness and filled with the nearby ground; the obstacle stays in obstacle_distance.
#   - BELOW: may be a real void (low wall, stairs going down, ditch): left untouched,
#     filling it would hide a drop.
# 1.2 m: at ~1.9 m from the robot (window edge) that is already ~32 degrees, beyond Spot's
# limit, so no climbable slope is discarded.
# =========================================================================
GROUND_HEIGHT_TOLERANCE_M = 1.2
GROUND_REF_RADIUS_M = 0.7            # cells under/around the robot used to estimate the ground
GROUND_REF_FALLBACK_RADIUS_M = 1.2   # if there are too few valid cells under the robot
GROUND_REF_MIN_CELLS = 50

# -------------------------------------------------------------------------
# CELLS NEVER WRITTEN (see LocalGrid._cells_never_written)
# Cells the sensors have never written all stay at the same exact height (a
# "plateau"). Spot almost always declares them invalid; this rule catches the rest.
# -------------------------------------------------------------------------
UNWRITTEN_MATCH_FRACTION = 0.25    # tolerance: 0.25 * quantization step
UNWRITTEN_MATCH_FALLBACK_M = 1e-4  # if the step is unknown
UNWRITTEN_MIN_CELLS = 100          # cells that must share the height for it to be a plateau
UNWRITTEN_MIN_SPOT_INVALID = 0.80  # fraction of the plateau that Spot must mark invalid


def fill_invalid_nearest(values_2d, invalid_mask):
    """
    Fill the unreliable cells (invalid_mask True) with the value of the nearest reliable
    cell: cells Spot declares invalid and the robot footprint.

    Returns a copy; if everything is valid, or everything invalid, the values unchanged.
    """
    if not np.any(invalid_mask) or not np.any(~invalid_mask):
        return values_2d.copy()
    nearest_indices = distance_transform_edt(invalid_mask, return_distances=False, return_indices=True)
    return values_2d[tuple(nearest_indices)]


class Scan:
    """
    A scan of the local grid (see LocalGrid.fetch_scan and process_scan). Flat arrays, in
    the order of the grid cells; (num_y, num_x) to put them back in 2D.
    """

    def __init__(self, **fields):
        self.__dict__.update(fields)


class LocalGrid:
    def __init__(self, robot):
        self.local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
        self.footprint_correction_pending = True
        # Outcome of the last ground height filter (see _cells_above_ground).
        self.last_ground_filter = {'ground_z': None, 'n_above': 0, 'max_above': 0.0,
                                   'source': 'non ancora eseguito', 'n_ref': 0}

    def create_vtk_no_step_grid(self, proto, robot_state_client):
        """
        "no_step" layer of the response: (points (x, y, z) in the VISION frame, values, RGB
        colours: red = not walkable, blue = walkable). Empty arrays if the layer is missing.
        """
        local_grid_proto = None
        cell_size = 0.0
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == 'no_step':
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        # If no relevant local grid found, return empty arrays (caller can handle)
        if local_grid_proto is None:
            return np.empty((0, 3), dtype=np.float32), np.array([], dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

        # Unpack the data field for the local grid.
        cells_no_step = self.unpack_grid(local_grid_proto).astype(np.float32)
        # Populate the x,y values with a complete combination of all possible pairs for the dimensions in the grid extent.
        ys, xs = np.mgrid[0:local_grid_proto.local_grid.extent.num_cells_x,
                          0:local_grid_proto.local_grid.extent.num_cells_y]
        # Get the estimated height (z value) of the ground in the vision frame as if the robot was standing.
        transforms_snapshot = local_grid_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        z_ground_in_vision_frame = self.compute_ground_height_in_vision_frame(robot_state_client)
        # Numpy vstack makes it so that each column is (x,y,z) for a single no step grid point. The height values come
        # from the estimated height of the ground plane.
        cell_count = local_grid_proto.local_grid.extent.num_cells_x * local_grid_proto.local_grid.extent.num_cells_y
        cells_est_height = np.ones(cell_count) * z_ground_in_vision_frame
        pts = np.vstack(
            [np.ravel(xs).astype(np.float32),
             np.ravel(ys).astype(np.float32), cells_est_height]).T
        pts[:, [0, 1]] *= (local_grid_proto.local_grid.extent.cell_size,
                           local_grid_proto.local_grid.extent.cell_size)
        # Determine the coloration based on whether or not the region is steppable. The regions that Spot considers it
        # cannot safely step are colored red, and the regions that are considered safe to step are colored blue.
        color = np.zeros([cell_count, 3], dtype=np.uint8)
        color[:, 0] = (cells_no_step <= 0.0)
        color[:, 2] = (cells_no_step > 0.0)
        color *= 255
        # Offset the grid points to be in the vision frame instead of the local grid frame.
        vision_tform_local_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME,
                                                local_grid_proto.local_grid.frame_name_local_grid_data)
        pts = self.offset_grid_pixels(pts, vision_tform_local_grid, cell_size)

        return pts, cells_no_step, color


    def create_vtk_obstacle_grid(self, proto, robot_state_client):
        """
        "obstacle_distance" layer: signed distance (m) from the nearest obstacle, negative
        inside an obstacle. Unlike no_step it does not treat grass as an obstacle, which is
        why it is the layer used for obstacles.

        Returns:
            (points (N, 3) in the VISION frame, distances (N,), RGB colours (N, 3): red
            inside an obstacle, blue outside). Empty arrays if the layer is missing.
        """
        local_grid_proto = None
        cell_size = 0.0
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == 'obstacle_distance':
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        # If no relevant local grid found, return empty arrays (caller can handle)
        if local_grid_proto is None:
            return np.empty((0, 3), dtype=np.float32), np.array([], dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

        # Unpack the raw distance values.
        cells_obstacle_dist = self.unpack_grid(local_grid_proto).astype(np.float32)

        # Build (x, y) grid coordinates.
        ys, xs = np.mgrid[0:local_grid_proto.local_grid.extent.num_cells_x,
                          0:local_grid_proto.local_grid.extent.num_cells_y]

        # Use ground-plane height for the z coordinate.
        transforms_snapshot = local_grid_proto.local_grid.transforms_snapshot
        z_ground_in_vision_frame = self.compute_ground_height_in_vision_frame(robot_state_client)
        cell_count = local_grid_proto.local_grid.extent.num_cells_x * local_grid_proto.local_grid.extent.num_cells_y
        z = np.ones(cell_count, dtype=np.float32) * z_ground_in_vision_frame

        pts = np.vstack([np.ravel(xs).astype(np.float32),
                         np.ravel(ys).astype(np.float32), z]).T
        pts[:, [0, 1]] *= (local_grid_proto.local_grid.extent.cell_size,
                           local_grid_proto.local_grid.extent.cell_size)

        # Colour coding (zero-padding - no border zone):
        #   red  -> strictly inside obstacle  (dist < 0)
        #   blue -> passable: border or free  (dist >= 0)
        color = np.zeros([cell_count, 3], dtype=np.uint8)
        color[:, 0] = (cells_obstacle_dist < 0.0)   # red  = blocked
        color[:, 2] = (cells_obstacle_dist >= 0.0)  # blue = passable
        color *= 255

        # Offset to VISION frame.
        vision_tform_local_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME,
                                                local_grid_proto.local_grid.frame_name_local_grid_data)
        pts = self.offset_grid_pixels(pts, vision_tform_local_grid, cell_size)

        return pts, cells_obstacle_dist, color


    def compute_ground_height_in_vision_frame(self, robot_state_client):
        """Get the z-height of the ground plane in vision frame from the current robot state."""
        robot_state = robot_state_client.get_robot_state()
        vision_tform_ground_plane = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                                  VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME)
        return vision_tform_ground_plane.position.z

    def offset_grid_pixels(self, pts, vision_tform_local_grid, cell_size):
        """
        Bring the points from the local grid frame to the VISION frame (full rotation and
        translation), referring them to the CENTRE of each cell.
        """
        pts_centered = pts.copy()
        pts_centered[:, 0] += cell_size * 0.5
        pts_centered[:, 1] += cell_size * 0.5

        # If the transform object supports transform_cloud, use it directly
        if hasattr(vision_tform_local_grid, 'transform_cloud'):
            return vision_tform_local_grid.transform_cloud(pts_centered)
        else:
            # Manual fallback through rotation matrix R and translation T
            rot_mat = vision_tform_local_grid.rotation.to_matrix()
            trans = np.array([
                vision_tform_local_grid.position.x,
                vision_tform_local_grid.position.y,
                vision_tform_local_grid.position.z
            ], dtype=np.float32)
            return (pts_centered @ rot_mat.T) + trans


    def unpack_grid(self, local_grid_proto):
        """Unpack the local grid proto."""
        # Determine the data type for the bytes data.
        data_type = self.get_numpy_data_type(local_grid_proto.local_grid)
        if data_type is None:
            print('[GRID] Formato dei dati della griglia locale non riconosciuto.')
            return None
        # Decode the local grid.
        if local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
            full_grid = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        elif local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
            full_grid = self.expand_data_by_rle_count(local_grid_proto, data_type=data_type)
        else:
            # Return nothing if there is no encoding type set.
            return None
        # Apply the offset and scaling to the local grid.
        if local_grid_proto.local_grid.cell_value_scale == 0:
            return full_grid
        full_grid_float = full_grid.astype(np.float64)
        full_grid_float *= local_grid_proto.local_grid.cell_value_scale
        full_grid_float += local_grid_proto.local_grid.cell_value_offset
        return full_grid_float


    def get_numpy_data_type(self, local_grid_proto):
        """Convert the cell format of the local grid proto to a numpy data type."""
        if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
            return np.uint16
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
            return np.int16
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
            return np.uint8
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
            return np.int8
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
            return np.float64
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
            return np.float32
        else:
            return None


    def expand_data_by_rle_count(self, local_grid_proto, data_type=np.int16):
        """Expand local grid data to full bytes data using the RLE count."""
        cells_pz = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        cells_pz_full = []
        # For each value of rle_counts, we expand the cell data at the matching index
        # to have that many repeated, consecutive values.
        for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
            for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
                cells_pz_full.append(cells_pz[i])
        return np.array(cells_pz_full)

    def _fetch_layers(self, types_of_grid):
        """
        Download the requested layers with ONE single request (same snapshot for all); if
        that does not work fall back on one request per layer.
        """
        if getattr(self, '_multi_request_ok', True):
            try:
                out = list(self.local_grid_client.get_local_grids(list(types_of_grid)))
                self._multi_request_fails = 0
                return out
            except Exception as e:
                # After 3 failures in a row it is no longer tried (a request that always fails
                # would cost a network round trip at every scan); a single network error
                # does not turn it off.
                self._multi_request_fails = getattr(self, '_multi_request_fails', 0) + 1
                if self._multi_request_fails >= 3:
                    print(f"[WARNING] Richiesta unica dei layer non riuscita 3 volte di fila ({e}): da ora "
                          f"li chiedo uno alla volta (con controllo di allineamento).")
                    self._multi_request_ok = False
        p = []
        for t in types_of_grid:
            try:
                p.extend(self.local_grid_client.get_local_grids([t]))
            except Exception as e:
                print(f"[WARNING] Fallito recupero layer '{t}': {e}")
        return p

    @staticmethod
    def _layer_geometry(lg):
        """(origin x, origin y, yaw in degrees, nx, ny, cell) of the layer in the VISION frame."""
        g = lg.local_grid
        tf = get_a_tform_b(g.transforms_snapshot, VISION_FRAME_NAME, g.frame_name_local_grid_data)
        q = tf.rotation
        yaw = np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        return (tf.position.x, tf.position.y, float(yaw), g.extent.num_cells_x, g.extent.num_cells_y,
                g.extent.cell_size)

    def fetch_scan(self, robot_state_client):
        """
        Download the layers of a scan (obstacle_distance, terrain, terrain_valid) and read
        their values and geometry. Returns a Scan, or None if the layers do not arrive. The
        processing (terrain correction, slope, obstacle mask) is done by process_scan.
        """
        grids_data, proto, _ = self.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'], robot_state_client)
        if grids_data is None or 'obstacle_distance' not in grids_data:
            return None
        ext = proto.local_grid.extent
        return Scan(grids_data=grids_data, proto=proto,
                    pts=grids_data['obstacle_distance']['pts'],
                    od=grids_data['obstacle_distance']['values'],
                    terrain_raw=grids_data['terrain']['values'],
                    valid=grids_data['terrain_valid']['values'],
                    num_x=ext.num_cells_x, num_y=ext.num_cells_y, cell_size=ext.cell_size)

    @staticmethod
    def grid_origin(scan):
        """Origin (x, y) of the scan grid in the VISION frame."""
        g = scan.proto.local_grid
        tf = get_a_tform_b(g.transforms_snapshot, VISION_FRAME_NAME, g.frame_name_local_grid_data)
        return tf.position.x, tf.position.y

    def process_scan(self, scan, robot_x, robot_y, robot_yaw, footprint=True, fuse=True,
                     ground_label=None):
        """
        Process a scan read with fetch_scan, filling the Scan:
          footprint_mask  cells under the robot's body (footprint=False: none)
          terrain, is_valid, unwritten  corrected terrain (see correct_terrain): from here
                          on only this is used
          grad, rough     slope and roughness on the corrected terrain
          obstacle_mask   per-cell veto, pure threshold logic (fuse=False: not computed).
                          The slope does not enter this veto: see fuse_obstacle_mask
        ground_label: label of the [GROUND-FILTER] line (None = no line). Returns
        the Scan.
        """
        s = scan
        s.footprint_mask = (self.compute_robot_footprint_mask(s.pts, robot_x, robot_y, robot_yaw,
                                                              s.num_x, s.num_y) if footprint else None)
        s.terrain, s.is_valid, s.unwritten = self.correct_terrain(
            s.terrain_raw, s.valid, s.num_x, s.num_y, robot_footprint_mask=s.footprint_mask,
            cell_size=s.cell_size, unwritten_value=raw_zero(s.grids_data),
            unwritten_scale=raw_scale(s.grids_data), return_unwritten=True)
        if ground_label:
            log_ground_filter(self, ground_label)
        s.grad, s.rough = self.compute_gradient_and_roughness(
            s.terrain, s.valid, s.num_x, s.num_y, s.cell_size, is_valid=s.is_valid)
        s.obstacle_mask = None
        if fuse:
            s.obstacle_mask = self.fuse_obstacle_mask(
                cells_obstacle_dist=s.od, rough_values=s.rough, is_valid=s.is_valid,
                obstacle_threshold=OBSTACLE_THRESHOLD, rough_threshold=ROUGH_THRESHOLD)
        return s

    def extra_layers(self, names):
        """
        Extra layers to SAVE only (e.g. 'intensity', 'no_step'), with a separate request:
        they do not enter the alignment checks of return_local_grid, so if they are
        missing or have another geometry the scan used for navigation is not affected.
        Never raises. Returns {name: dict(values (ny, nx) float32, origin (x, y) in the
        VISION frame, cell)}; {} if the request fails.
        """
        try:
            protos = list(self.local_grid_client.get_local_grids(list(names)))
        except Exception:
            return {}
        out = {}
        for lg in protos:
            try:
                ox, oy, _, nx, ny, cell = self._layer_geometry(lg)
                vals = self.unpack_grid(lg)
                if vals is None or np.size(vals) != nx * ny:
                    continue
                out[lg.local_grid_type_name] = dict(
                    values=np.asarray(vals, dtype=np.float32).reshape((ny, nx)),
                    origin=np.array([ox, oy]), cell=float(cell))
            except Exception:
                continue
        return out

    def return_local_grid(self, types_of_grid, robot_state_client, max_attempts=3):
        """
        Download and decode the requested layers.

        Returns (grids_data, main_proto, protos), or (None, None, None) if after
        max_attempts attempts even a single layer is missing or the layers are not aligned
        with each other (different origin, size or cell: if the robot moves between one
        request and the next the grid may recentre). main_proto is the first requested
        layer: its origin indexes all the layers.

        Warns once if the grid is rotated relative to VISION: all the code assumes it is
        not (cell indices computed with the translation only).
        """
        if isinstance(types_of_grid, str):
            types_of_grid = [types_of_grid]

        p, problem = [], None
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                time.sleep(0.05)
            p = self._fetch_layers(types_of_grid)
            names = [lg.local_grid_type_name for lg in p]
            missing = [t for t in types_of_grid if t not in names]
            if missing:
                problem = f"layer mancanti {missing}"
                continue
            try:
                geoms = {lg.local_grid_type_name: self._layer_geometry(lg) for lg in p}
            except Exception as e:
                problem = f"geometria dei layer non leggibile ({e})"
                continue
            ref = geoms[types_of_grid[0]]
            bad = [n for n, gm in geoms.items()
                   if gm[3:5] != ref[3:5] or abs(gm[5] - ref[5]) > 1e-6
                   or np.hypot(gm[0] - ref[0], gm[1] - ref[1]) > 0.5 * ref[5]]
            if bad:
                problem = f"layer non allineati a {types_of_grid[0]}: {bad}"
                continue
            if abs(ref[2]) > 0.5 and not getattr(self, '_warned_grid_yaw', False):
                print(f"[WARNING] La griglia locale e' ruotata di {ref[2]:.2f} gradi rispetto a VISION: "
                      f"gli indici di cella assumono rotazione nulla. Da segnalare.")
                self._warned_grid_yaw = True
            problem = None
            break

        if problem is not None or not p:
            self.rejected_scans = getattr(self, 'rejected_scans', 0) + 1
            print(f"[ERROR] Griglie locali non utilizzabili dopo {max_attempts} tentativi: "
                  f"{problem or 'nessun layer scaricato'} (scansioni scartate finora: {self.rejected_scans}).")
            return None, None, None

        grids_data = {}
        # main_proto = the first requested layer (obstacle_distance in all callers):
        # its origin is the one ALL layers are indexed with.
        main_proto = next(lg for lg in p if lg.local_grid_type_name == types_of_grid[0])

        for lg in p:
            grid_type = lg.local_grid_type_name
            if grid_type == 'obstacle_distance':
                pts, vals, color = self.create_vtk_obstacle_grid(p, robot_state_client)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
            elif grid_type == 'no_step':
                pts, vals, color = self.create_vtk_no_step_grid(p, robot_state_client)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
            elif grid_type in ['terrain', 'intensity', 'terrain_valid']:
                pts, vals, color = self.create_vtk_terrain_grid(p, robot_state_client, layer_name=grid_type)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
                if grid_type == 'terrain':
                    # Height quantization step of THIS scan: tolerance for
                    # _cells_never_written. 'raw_zero' only acts as the switch of that
                    # rule (None = off).
                    grids_data[grid_type]['raw_zero'] = float(np.float32(lg.local_grid.cell_value_offset))
                    grids_data[grid_type]['scale'] = float(lg.local_grid.cell_value_scale)

        return grids_data, main_proto, p

    def create_vtk_terrain_grid(self, proto, robot_state_client, layer_name='terrain'):
        """
        Terrain-type layer (terrain, terrain_valid, intensity): (points in the VISION frame,
        values, colours). For `terrain` the returned value is the height in VISION.
        """
        local_grid_proto = None
        cell_size = 0.0

        for local_grid_found in proto:
            # Check against the dynamic layer_name instead of a hardcoded string
            if local_grid_found.local_grid_type_name == layer_name:
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        if local_grid_proto is None:
            return np.empty((0, 3), dtype=np.float32), np.array([], dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

        # Decode values
        terrain_values = self.unpack_grid(local_grid_proto).astype(np.float32)

        # Build XY grid
        ys, xs = np.mgrid[
                 0:local_grid_proto.local_grid.extent.num_cells_x,
                 0:local_grid_proto.local_grid.extent.num_cells_y
                 ]

        z_vals = np.ravel(terrain_values)
        pts = np.vstack([
            np.ravel(xs).astype(np.float32),
            np.ravel(ys).astype(np.float32),
            z_vals
        ]).T

        pts[:, [0, 1]] *= (cell_size, cell_size)

        transforms_snapshot = local_grid_proto.local_grid.transforms_snapshot
        vision_tform_local_grid = get_a_tform_b(
            transforms_snapshot,
            VISION_FRAME_NAME,
            local_grid_proto.local_grid.frame_name_local_grid_data
        )

        pts = self.offset_grid_pixels(pts, vision_tform_local_grid, cell_size)

        # The decoded heights are in the local grid frame; after offset_grid_pixels
        # pts[:, 2] is the height in the VISION frame, which is what is returned.
        terrain_values_vision = pts[:, 2].astype(np.float32)

        color = np.zeros([len(z_vals), 3], dtype=np.uint8)
        color[:, 1] = 255  # green

        return pts, terrain_values_vision, color

    def correct_terrain(self, terrain_values, terrain_valid_values, num_cells_x, num_cells_y,
                        robot_footprint_mask=None, cell_size=0.03, robot_rc=None,
                        unwritten_value=None, unwritten_scale=None, return_unwritten=False):
        """
        The only place where the terrain is corrected. The unreliable cells -- invalid for
        Spot, under the robot's body, too far above the ground (_cells_above_ground), never
        written by the sensors (_cells_never_written) -- are filled with the height of the
        nearest reliable cell and marked invalid. All the rest of the code uses this
        corrected terrain, and is_valid prevents a filled value from deciding anything.

        Args:
            terrain_values: raw heights (flat or (num_cells_y, num_cells_x))
            terrain_valid_values: Spot's terrain_valid layer (reliable if >= TERRAIN_VALID_MIN)
            robot_footprint_mask: optional 2D mask, True under the robot's body
            unwritten_value: switch of the never-written-cells rule
                (grids_data['terrain']['raw_zero']); None = rule off
            unwritten_scale: quantization step (grids_data['terrain']['scale'])
            return_unwritten: also return the mask of the cells never written

        Returns:
            terrain_corrected: corrected heights, 1D
            is_valid: 1D, True where the ORIGINAL reading was reliable
            unwritten: (only with return_unwritten) 1D, cells never written, to be treated
                as NOT SEEN (see safe_frontier.make_grid_snapshot)
        """
        terrain_2d = terrain_values.reshape((num_cells_y, num_cells_x)).astype(np.float32)

        if terrain_valid_values is not None and terrain_valid_values.size > 0:
            valid_2d = (terrain_valid_values.reshape((num_cells_y, num_cells_x)) >= TERRAIN_VALID_MIN)
        else:
            valid_2d = np.ones((num_cells_y, num_cells_x), dtype=bool)

        sensor_invalid_2d = ~valid_2d
        fill_mask = sensor_invalid_2d.copy()
        if robot_footprint_mask is not None:
            fill_mask = fill_mask | robot_footprint_mask

        # Ground height filter -- see GROUND_HEIGHT_TOLERANCE_M.
        above_ground = self._cells_above_ground(terrain_2d, ~fill_mask, cell_size, robot_rc)
        # Never-written cells: valid_2d is passed (Spot's flag), not ~fill_mask: see
        # _cells_never_written.
        unwritten = self._cells_never_written(terrain_2d, valid_2d, unwritten_value, unwritten_scale)
        fill_mask = fill_mask | above_ground | unwritten

        terrain_corrected = fill_invalid_nearest(terrain_2d, fill_mask)
        is_valid = ~fill_mask

        if return_unwritten:
            return terrain_corrected.ravel(), is_valid.ravel(), unwritten.ravel()
        return terrain_corrected.ravel(), is_valid.ravel()

    def _cells_never_written(self, terrain_2d, sensor_valid_2d, unwritten_value, unwritten_scale):
        """
        Cells the sensors have never written: they all stay at the SAME exact height.

        The height cannot be predicted (it depends on the frame and Spot moves the offset at
        every scan): it is DERIVED as the most frequent exact height among the cells Spot
        itself declares unreliable. If at least UNWRITTEN_MIN_CELLS share it (and at least
        UNWRITTEN_MIN_SPOT_INVALID of those at that height are invalid for Spot), every cell
        at that height is never written, including the few Spot marks good by mistake.
        It is a safety net: Spot already marks almost all these cells invalid.

        unwritten_value only acts as a switch (None = rule off).
        Updates self.last_ground_filter['n_unwritten'] and ['n_unwritten_below'].
        """
        none = np.zeros_like(sensor_valid_2d, dtype=bool)
        lgf = self.last_ground_filter
        lgf['n_unwritten'] = 0
        lgf['n_unwritten_below'] = 0
        if unwritten_value is None:
            return none

        tol = (UNWRITTEN_MATCH_FRACTION * abs(float(unwritten_scale)) if unwritten_scale
               else UNWRITTEN_MATCH_FALLBACK_M)
        tol = max(tol, 1e-9)

        finite = np.isfinite(terrain_2d)
        cand = terrain_2d[(~sensor_valid_2d) & finite]
        if cand.size < UNWRITTEN_MIN_CELLS:
            return none
        keys, counts = np.unique(np.round(cand / tol).astype(np.int64), return_counts=True)
        i = int(np.argmax(counts))
        if counts[i] < UNWRITTEN_MIN_CELLS:
            return none
        quota = float(keys[i]) * tol

        unwritten = finite & (np.abs(terrain_2d - quota) <= tol)
        # A real plateau is made of cells Spot itself declares unreliable. If
        # that height coincided with a REAL surface (a table top
        # exactly at that height) the cells would be mostly valid: left untouched.
        if not unwritten.any() or \
                float(np.mean(~sensor_valid_2d[unwritten])) < UNWRITTEN_MIN_SPOT_INVALID:
            return none

        lgf['n_unwritten'] = int(unwritten.sum())
        lgf['quota_unwritten'] = quota
        ground_z = lgf.get('ground_z')
        if ground_z is not None:
            lgf['n_unwritten_below'] = int((unwritten & (terrain_2d < ground_z)).sum())
        return unwritten

    def _cells_above_ground(self, terrain_2d, trusted_2d, cell_size, robot_rc=None):
        """
        Cells lying more than GROUND_HEIGHT_TOLERANCE_M ABOVE the ground the robot stands
        on. Only above: see the comment on the constant for why.

        The ground is the median of the reliable cells within GROUND_REF_RADIUS_M of the
        robot. Spot's local grid is centred on the robot (within ~5 cm), so by default the
        robot is at the centre; robot_rc=(row, column) overrides it.

        Records the outcome in self.last_ground_filter for the logs and the data saving.
        Returns a 2D boolean mask (all False if the ground cannot be estimated).
        """
        num_rows, num_cols = terrain_2d.shape
        none = np.zeros_like(trusted_2d, dtype=bool)
        if robot_rc is None:
            cr, cc = (num_rows - 1) / 2.0, (num_cols - 1) / 2.0
        else:
            cr, cc = robot_rc

        rr, cc_grid = np.ogrid[:num_rows, :num_cols]
        dist = np.hypot((rr - cr) * cell_size, (cc_grid - cc) * cell_size)
        finite = np.isfinite(terrain_2d)

        source = 'sotto il robot'
        ref = trusted_2d & finite & (dist <= GROUND_REF_RADIUS_M)
        if ref.sum() < GROUND_REF_MIN_CELLS:
            source = 'anello esteso'
            ref = trusted_2d & finite & (dist <= GROUND_REF_FALLBACK_RADIUS_M)
        if ref.sum() < GROUND_REF_MIN_CELLS:
            self.last_ground_filter = {'ground_z': None, 'n_above': 0, 'max_above': 0.0,
                                       'source': 'non stimabile', 'n_ref': int(ref.sum())}
            return none

        ground_z = float(np.median(terrain_2d[ref]))
        height_above = terrain_2d - ground_z
        above = trusted_2d & finite & (height_above > GROUND_HEIGHT_TOLERANCE_M)

        self.last_ground_filter = {
            'ground_z': ground_z,
            'n_above': int(above.sum()),
            'max_above': float(height_above[above].max()) if above.any() else 0.0,
            'source': source,
            'n_ref': int(ref.sum()),
        }
        return above

    def compute_gradient_and_roughness(self, terrain_corrected, terrain_valid_values, num_cells_x, num_cells_y, cell_size,
                                       slope_baseline_m=SLOPE_BASELINE_M, is_valid=None):
        """
        Slope and roughness layers per cell, from a terrain ALREADY corrected (see
        correct_terrain). The cells Spot declares invalid are 0 in the output.

        SLOPE: centred difference over a physical base (see SLOPE_BASELINE_M: ~0.48 m,
        about one step) instead of over the 3 cm cell, on which a small edge would count as
        much as a real climb. This per-cell field is used for nodes and figures; the slope
        that decides is the per-edge one (compute_arc_slope_profile).

        ROUGHNESS: standard deviation of the height over a short window (5 cells, 15 cm),
        for the irregular ground under the foot: widening it would hide exactly that.
        The statistics use only reliable cells (is_valid).
        """
        terrain_2d = terrain_corrected.reshape((num_cells_y, num_cells_x)).astype(np.float64)

        if terrain_valid_values is not None and terrain_valid_values.size > 0:
            sensor_invalid_mask = ~(terrain_valid_values.reshape((num_cells_y, num_cells_x)) >= TERRAIN_VALID_MIN)
        else:
            sensor_invalid_mask = np.zeros((num_cells_y, num_cells_x), dtype=bool)

        window_size = 5

        # Cells that can enter the neighbourhood statistics. With is_valid (output of
        # correct_terrain) the filled cells and those discarded by the ground height
        # filter stay out too: otherwise an unreliable cell "contaminates" the
        # roughness of the valid neighbours in the 5x5 window.
        if is_valid is not None:
            weight_2d = np.asarray(is_valid).reshape((num_cells_y, num_cells_x)).astype(np.float64)
        else:
            weight_2d = (~sensor_invalid_mask).astype(np.float64)

        # 1. "Normalized" smoothing: mean of the valid cells of the window only.
        #    Where the window has no valid cell the corrected terrain is kept.
        n_valid = uniform_filter(weight_2d, size=window_size)
        sum_t = uniform_filter(terrain_2d * weight_2d, size=window_size)
        sum_t2 = uniform_filter((terrain_2d ** 2) * weight_2d, size=window_size)
        has_support = n_valid > (0.5 / (window_size * window_size))
        with np.errstate(invalid='ignore', divide='ignore'):
            mean_t = np.where(has_support, sum_t / n_valid, terrain_2d)
            mean_t2 = np.where(has_support, sum_t2 / n_valid, terrain_2d ** 2)

        # 2. Gradient (slope) computation over a physical base, not over the native cell spacing
        k = max(1, round(slope_baseline_m / cell_size))  # e.g. 0.25m / 0.03m =~ 8 cells

        def _centered_diff(arr, k, axis):
            # Centred difference at distance k cells, with "clamped" border (edge padding)
            # so we do not introduce fake slopes at the edges of the local grid.
            pad = [(0, 0), (0, 0)]
            pad[axis] = (k, k)
            padded = np.pad(arr, pad, mode='edge')
            sl_ahead = [slice(None), slice(None)]
            sl_ahead[axis] = slice(2 * k, 2 * k + arr.shape[axis])
            sl_behind = [slice(None), slice(None)]
            sl_behind[axis] = slice(0, arr.shape[axis])
            return padded[tuple(sl_ahead)] - padded[tuple(sl_behind)]

        grad_x = _centered_diff(mean_t, k, axis=1) / (2 * k * cell_size)
        grad_y = _centered_diff(mean_t, k, axis=0) / (2 * k * cell_size)
        gradient_2d = np.sqrt(grad_x ** 2 + grad_y ** 2)

        # 3. Roughness computation (local standard deviation, valid cells only) -- short window unchanged
        variance = mean_t2 - (mean_t ** 2)
        roughness_2d = np.sqrt(np.maximum(0.0, variance))
        # With fewer than 3 valid cells in the window the standard deviation means nothing.
        roughness_2d[n_valid * window_size * window_size < 3] = 0.0

        # Zero ONLY the truly blind areas in the final output.
        # The robot footprint keeps the realistic interpolated estimate instead.
        gradient_2d[sensor_invalid_mask] = 0.0
        roughness_2d[sensor_invalid_mask] = 0.0

        return gradient_2d.ravel(), roughness_2d.ravel()


    def fuse_obstacle_mask(self, cells_obstacle_dist,
                        rough_values,
                        is_valid,
                        obstacle_threshold=OBSTACLE_THRESHOLD,
                        rough_threshold=ROUGH_THRESHOLD):
        """
        Per-cell obstacle mask: -1 obstacle, +1 free. A reliable cell is an obstacle if
        obstacle_distance <= obstacle_threshold or roughness > rough_threshold; an
        unreliable (filled) cell is never an obstacle.

        The slope does not enter here: it is evaluated per EDGE (compute_arc_slope_profile),
        over its whole length and in both directions. Roughness instead is a local property.
        """
        cells_obstacle_dist = cells_obstacle_dist.ravel()
        rough_values = rough_values.ravel()
        is_valid = is_valid.ravel()

        assert cells_obstacle_dist.shape == rough_values.shape == is_valid.shape, \
            "Errore: I layer ambientali hanno dimensioni disallineate!"

        # VETO computation applied ONLY to the areas with valid data
        # If an area is NOT valid (blind spot or robot footprint), it must not trigger the obstacle veto
        v1 = (cells_obstacle_dist <= obstacle_threshold) & is_valid
        v3 = (rough_values > rough_threshold) & is_valid

        # Final binary obstacle mask
        obstacle_mask = v1 | v3
        obstacle_mask = np.where(obstacle_mask, -1.0, 1.0)

        return obstacle_mask

    def compute_robot_footprint_mask(self, pts, robot_x, robot_y, robot_yaw, num_cells_x, num_cells_y, margin=0.05,
                                     body_length=1.10, body_width=0.50):
        """
        2D mask of the cells under the robot's body (oriented rectangle, plus margin).
        It is returned ONLY ONCE per mission, at the first call (at the first scan the
        terrain under the robot has never been seen); afterwards it returns None and Spot's
        terrain_valid applies.

        body_length / body_width: body dimensions.

        Careful: the frontier thread (safe_frontier) calls it too, so the "only once" mask
        may be consumed by it first.
        """
        if not self.footprint_correction_pending:
            return None
        self.footprint_correction_pending = False

        dx = pts[:, 0] - robot_x
        dy = pts[:, 1] - robot_y
        cos_yaw, sin_yaw = np.cos(robot_yaw), np.sin(robot_yaw)
        x_body = dx * cos_yaw + dy * sin_yaw
        y_body = -dx * sin_yaw + dy * cos_yaw

        half_length = (body_length / 2.0) + margin
        half_width = (body_width / 2.0) + margin
        mask_flat = (np.abs(x_body) <= half_length) & (np.abs(y_body) <= half_width)
        return mask_flat.reshape((num_cells_y, num_cells_x))


def slope_usable_mask(is_valid_flat, cells_obstacle_dist, shape):
    """
    Mask of the cells whose HEIGHTS can enter the slope computation of an edge: reliable
    (is_valid of correct_terrain) AND non-obstacle.

    The vertical face of a piece of furniture is not sloping terrain, it is an obstacle --
    and it is already handled as such twice: by the PRM occupancy veto and by the safe
    frontier, which stops FRONTIER_CLEARANCE_M from any cell with low obstacle_distance.
    Counting it ALSO as slope counts it a third time, with the worst of effects: it does
    not stop the robot in front of the furniture (the frontier does that), it disconnects
    the PRM graph in the whole room.

    The edges that lose the data do NOT become "free by decree": they become "without
    data", and an edge without data is not vetoed (it enters the graph with a fallback
    cost, see PRM._evaluate_and_add_edge); whoever travels it checks it metre by metre with
    the safe frontier. It is an explicit choice: on those edges safety comes from the
    frontier, not from a slope measured on a wardrobe.

    The mask is combined HERE and not inside roadmap because this is where both arrays
    are at hand, and because in roadmap `current_valid_2d` serves one thing only -- the
    per-edge slope profile (compute_arc_slope_profile) -- so narrowing it has no other
    effects. Whoever reads roadmap.update_local_grid_data finds `valid_2d` documented as
    "reliable cells": here it means "reliable as TERRAIN".
    """
    valid = np.asarray(is_valid_flat, dtype=bool).reshape(shape)
    if cells_obstacle_dist is None:
        return valid
    od = np.asarray(cells_obstacle_dist, dtype=np.float64).reshape(shape)
    return valid & ~(od < OBSTACLE_THRESHOLD)


def raw_zero(grids_data):
    """Height of the raw value 0 of this scan's terrain layer (None if absent)."""
    try:
        return grids_data['terrain'].get('raw_zero')
    except (KeyError, TypeError, AttributeError):
        return None


def raw_scale(grids_data):
    try:
        return grids_data['terrain'].get('scale')
    except (KeyError, TypeError, AttributeError):
        return None


def log_ground_filter(local_grid, label):
    """
    One line only when the ground height filter has discarded something or could not
    estimate the ground -- in the normal case it is silent. See GROUND_HEIGHT_TOLERANCE_M.
    """
    info = getattr(local_grid, 'last_ground_filter', None) or {}
    if info.get('ground_z') is None:
        print(f"[GROUND-FILTER] {label}: suolo non stimabile ({info.get('source')}, "
              f"{info.get('n_ref', 0)} celle di riferimento) -- filtro non applicato")
    elif info.get('n_above', 0) > 0:
        print(f"[GROUND-FILTER] {label}: {info['n_above']} celle scartate, fino a "
              f"{info['max_above']:.2f} m sopra il suolo (suolo z={info['ground_z']:.3f}, "
              f"stimato {info['source']} su {info['n_ref']} celle)")
    if info.get('n_unwritten', 0) > 0:
        print(f"[GROUND-FILTER] {label}: {info['n_unwritten']} celle mai scritte dai sensori "
              f"({info.get('n_unwritten_below', 0)} sotto il suolo) -> trattate come non viste"
              + (f", quota {info['quota_unwritten']:+.4f} m" if info.get('quota_unwritten') is not None else ""))
