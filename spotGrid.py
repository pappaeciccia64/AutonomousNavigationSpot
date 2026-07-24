#spotSDK/spotGrid.py


import numpy as np
from scipy.ndimage import uniform_filter, distance_transform_edt
import bosdyn.client
from bosdyn.client.frame_helpers import *
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.api import local_grid_pb2
from bosdyn.client.local_grid import LocalGridClient

import pickle
import os

class LocalGrid:
    def __init__(self, robot):
        self.local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
        self.footprint_correction_pending = True

    def create_vtk_no_step_grid(self, proto, robot_state_client):
        """Generate VTK polydata for the no step grid from the local grid response.
        Questa funzione:

        Cerca la local grid “no_step” nel messaggio ricevuto.

        Decodifica i valori (raw o RLE).

        Costruisce una griglia di punti (x,y,z) nel frame VISION.

        Colora le celle (rosso = non steppable, blu = steppable).

        Restituisce:

            pts → punti 3D nel frame VISION

            cells_no_step → valori della grid

            color → colori RGB"""
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
        """Generate points, cell values and colors for the obstacle distance grid.

        The obstacle_distance grid encodes the signed distance (in metres) from each
        cell to the nearest detected obstacle:
            dist < 0   -> strictly inside an obstacle  → blocked  (red)
            dist >= 0  -> border or free space          → passable (blue)

        Zero-padding policy: no safety margin is added around obstacles.
        A cell is passable as soon as its distance is >= 0.
        Unlike the no_step grid, grass is NOT classified as an obstacle here,
        making this grid more suitable for outdoor environments.

        Returns:
            pts (np.ndarray, shape (N,3)): cell positions in the VISION frame
            cells_obstacle_dist (np.ndarray, shape (N,)): raw signed-distance values
            color (np.ndarray, shape (N,3) uint8): RGB colours per cell
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

        # Colour coding (zero-padding – no border zone):
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
        [FIX ROTOTRASLAZIONE SE3]
        Ruota e trasla la griglia locale nel frame VISION considerando
        sia la posizione sia il quaternione di rotazione.
        """
        pts_centered = pts.copy()
        pts_centered[:, 0] += cell_size * 0.5
        pts_centered[:, 1] += cell_size * 0.5

        # Se l'oggetto della trasformazione supporta transform_cloud, usalo direttamente
        if hasattr(vision_tform_local_grid, 'transform_cloud'):
            return vision_tform_local_grid.transform_cloud(pts_centered)
        else:
            # Fallback manuale tramite matrice di rotazione R e traslazione T
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
            print('Cannot determine the dataformat for the local grid.')
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

    def return_local_grid(self, types_of_grid, robot_state_client):
        """
        Scarica e decodifica i layer richiesti.
        Esegue richieste multiple in caso di errore Protobuf sulla lista.
        """
        if isinstance(types_of_grid, str):
            types_of_grid = [types_of_grid]

        p = []
        # Soluzione a prova di bomba per vecchi SDK / bug Protobuf:
        # Invece di chiedere tutto insieme, iteriamo e facciamo 1 richiesta gRPC per layer.
        for t in types_of_grid:
            try:
                # Chiediamo un layer alla volta (è rapido via gRPC locale)
                response = self.local_grid_client.get_local_grids([t])
                p.extend(response)
            except Exception as e:
                print(f"[WARNING] Fallito recupero layer '{t}': {e}")
                continue

        grids_data = {}
        main_proto = None

        if not p:
            print(f"[ERROR] Nessuna delle griglie richieste {types_of_grid} è stata scaricata.")
            return None, None, None

        for lg in p:
            grid_type = lg.local_grid_type_name
            if main_proto is None:
                main_proto = lg

            if grid_type == 'obstacle_distance':
                pts, vals, color = self.create_vtk_obstacle_grid(p, robot_state_client)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
            elif grid_type == 'no_step':
                pts, vals, color = self.create_vtk_no_step_grid(p, robot_state_client)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
            elif grid_type in ['terrain', 'intensity', 'terrain_valid']:
                pts, vals, color = self.create_vtk_terrain_grid(p, robot_state_client, layer_name=grid_type)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}

        return grids_data, main_proto, p

    def create_vtk_terrain_grid(self, proto, robot_state_client, layer_name='terrain'):
        """Generate VTK polydata for any terrain-based grid from the local grid response."""
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

        color = np.zeros([len(z_vals), 3], dtype=np.uint8)
        color[:, 1] = 255  # green

        return pts, terrain_values, color

    def compute_gradient_and_roughness(self, terrain_values, terrain_valid_values, num_cells_x, num_cells_y, cell_size,robot_footprint_mask_2d=None):
        """
        Genera layer di pendenza e rugosità estendendo i valori del bordo valido
        nelle zone cieche per evitare falsi ostacoli lungo i confini di scansione.
        """
        terrain_2d = terrain_values.reshape((num_cells_y, num_cells_x)).astype(np.float64)
        valid_2d = (terrain_valid_values.reshape((num_cells_y, num_cells_x)) > 0.0)

        # Zone realmente cieche (nessun dato sensore) - usate per azzerare l'OUTPUT finale
        sensor_invalid_mask = ~valid_2d

        # Per l'INTERPOLAZIONE pre-smoothing, includiamo anche l'impronta del robot:
        # questo evita che il blur a 5 celle faccia sanguinare la contaminazione
        # nell'anello di celle attorno all'impronta.
        fill_mask = sensor_invalid_mask.copy()
        if robot_footprint_mask_2d is not None:
            fill_mask = fill_mask | robot_footprint_mask_2d

        terrain_filled = terrain_2d.copy()

        # --- PADDING ADIACENTE (Nearest Neighbor) ---
        # Estendiamo il valore del pixel valido più vicino dentro le zone cieche.
        # In questo modo il gradiente sul bordo tra valido/invalido sarà esattamente 0.

        if np.any(fill_mask) and np.any(~fill_mask):
            nearest_indices = distance_transform_edt(fill_mask, return_distances=False, return_indices=True)
            terrain_filled = terrain_2d[tuple(nearest_indices)]

        window_size = 5

        # 1. Smoothing sul terreno senza discontinuità di bordo
        mean_t = uniform_filter(terrain_filled, size=window_size)

        # 2. Calcolo Gradiente (Pendenza)
        grad_x, grad_y = np.gradient(mean_t, cell_size)
        gradient_2d = np.sqrt(grad_x ** 2 + grad_y ** 2)

        # 3. Calcolo Rugosità (Deviazione Standard locale)
        mean_t2 = uniform_filter(terrain_filled ** 2, size=window_size)
        variance = mean_t2 - (mean_t ** 2)
        roughness_2d = np.sqrt(np.maximum(0.0, variance))

        # Azzeriamo SOLO le zone realmente cieche nell'output finale.
        # L'impronta del robot mantiene invece la stima interpolata realistica.
        gradient_2d[sensor_invalid_mask] = 0.0
        roughness_2d[sensor_invalid_mask] = 0.0

        return gradient_2d.ravel(), roughness_2d.ravel()


    def fuse_all_layers(self, cells_obstacle_dist,
                        terrain_values,
                        grad_values,
                        rough_values,
                        terrain_valid_values=None,
                        intensity_values=None,
                        pts = None,              # Passiamo l'array dei punti 3D (VISION)
                        robot_footprint_mask = None,
                        obstacle_threshold=0.15,
                        slope_threshold=0.6, #= tan(gradi) -> $\tan(28^\circ) \approx$ 0.53, $\tan(25^\circ) \approx$ 0.46
                        rough_threshold=0.15,
                        intensity_threshold=None,
                        robot_z=0.0,
                        step_threshold=0.40):
        """
        Fonde i layer ambientali restituendo la mappa ad altezze reali
        e una maschera binaria separata per gli ostacoli, ripulendo la zona
        occupata dal corpo fisico del robot (110cm x 50cm).
        """

        assert cells_obstacle_dist.shape == terrain_values.shape == grad_values.shape, "Errore: I layer ambientali hanno dimensioni disallineate!"

        cells_obstacle_dist = cells_obstacle_dist.ravel()
        terrain_values = terrain_values.ravel()
        grad_values = grad_values.ravel()
        rough_values = rough_values.ravel()

        # Mappa ad altezze REALI intatta
        terrain_real = terrain_values.copy().astype(np.float32)

        # Identifica le zone VALIDATE (dove il sensore vede davvero)
        # Se non viene passato, assumiamo di default che tutto sia valido
        is_valid = np.ones_like(cells_obstacle_dist, dtype=bool)
        if terrain_valid_values is not None and terrain_valid_values.size > 0:
            is_valid = (terrain_valid_values.ravel() > 0.0)

        # Calcolo dei VETI applicato SOLO alle zone con dati validi
        # Se una zona NON è valida (punto cieco), non deve attivare il veto di ostacolo
        v1 = (cells_obstacle_dist <= obstacle_threshold) & is_valid
        v2 = (grad_values > slope_threshold) & is_valid
        v3 = (rough_values > rough_threshold) & is_valid

        #print(f"[DEBUG FUSIONE] Totale celle griglia: {cells_obstacle_dist.size}")
        #print(f"[DEBUG FUSIONE] Celle bloccate da DISTANZA OSTACOLI: {np.sum(v1)}")
        #print(f"[DEBUG FUSIONE] Celle bloccate da PENDENZA: {np.sum(v2)}")
        #print(f"[DEBUG FUSIONE] Celle bloccate da RUGOSITÀ: {np.sum(v3)}")

        # Maschera binaria finale degli ostacoli
        obstacle_mask = v1 | v2 | v3

        obstacle_mask = np.where(obstacle_mask, -1.0, 1.0)

        # =========================================================
        # FILTRO DI AUTOCONSERVAZIONE RETTANGOLARE (ORIENTED FOOTPRINT)
        # =========================================================
        if robot_footprint_mask is not None:

            # Ripuliamo forzatamente le celle sotto la carrozzeria di Spot
            obstacle_mask[robot_footprint_mask] = 1.0

            # Azzeriamo pendenze e rugosità fittizie lette sotto i piedi
            grad_values[robot_footprint_mask] = 0.0
            rough_values[robot_footprint_mask] = 0.0

            terrain_real[robot_footprint_mask] = np.nan
        # =========================================================

        # RESTITUIAMO ENTRAMBI I LAYER SEPARATI
        return terrain_real, obstacle_mask

    def compute_robot_footprint_mask(self, pts, robot_x, robot_y, robot_yaw, num_cells_x, num_cells_y, margin=0.05):
        """
        Returns the footprint mask only once per mission (the very first call).
        Every call after that returns None, so downstream functions skip footprint
        masking entirely and rely on Spot's own terrain_valid_values from then on --
        the images showed the footprint gap is only ever real on the very first scan.
        """
        if not self.footprint_correction_pending:
            return None
        self.footprint_correction_pending = False

        dx = pts[:, 0] - robot_x
        dy = pts[:, 1] - robot_y
        cos_yaw, sin_yaw = np.cos(robot_yaw), np.sin(robot_yaw)
        x_body = dx * cos_yaw + dy * sin_yaw
        y_body = -dx * sin_yaw + dy * cos_yaw

        half_length = (1.10 / 2.0) + margin
        half_width = (0.50 / 2.0) + margin
        mask_flat = (np.abs(x_body) <= half_length) & (np.abs(y_body) <= half_width)
        return mask_flat.reshape((num_cells_y, num_cells_x))


class GlobalGrid:
    def __init__(self, resolution=0.03):
        self.resolution = resolution
        # Dizionario globale: la chiave è (grid_x, grid_y), il valore è lo stato:
        #  1.0 -> Libero/Sicuro
        # -1.0 -> Occupato/Ostacolo
        self.grid = {}
        self.obs_count = {}
        self._cached_margin = None
        self._cached_offsets = []
        self.global_occupancy_map = None


    def update(self, pts, obstacle_mask):
        """
        [FIX AGGIORNAMENTO PROBABILISTICO]
        Se una cella inizialmente vista occupata viene vista libera per
        più scansioni consecutive, torna ad essere considerata libera (1.0).
        """
        pts_flat = pts.reshape(-1, 3)
        mask_flat = obstacle_mask.ravel()

        gx_array = np.round(pts_flat[:, 0] / self.resolution).astype(int)
        gy_array = np.round(pts_flat[:, 1] / self.resolution).astype(int)

        for gx, gy, state in zip(gx_array, gy_array, mask_flat):
            key = (gx, gy)
            curr_count = self.obs_count.get(key, 0)

            if state == -1.0:
                # Incrementa contatore ostacolo (fino a un max di +4)
                new_count = min(curr_count + 1 if curr_count >= 0 else 1, 4)
                self.obs_count[key] = new_count
                self.grid[key] = -1.0
            else:
                # Decrementa contatore se l'area viene vista libera
                new_count = curr_count - 1
                self.obs_count[key] = max(new_count, -4)
                # Se la cella è confermata libera (contatore <= 0), sbloccala
                if new_count <= 0:
                    self.grid[key] = 1.0

    def is_occupied(self, x, y, safety_margin=0.0):
        """
        Verifica se una coordinata reale (x, y) cade in una cella occupata.
        Opzionalmente controlla anche un intorno (safety_margin in metri).
        Utilizza offset pre-calcolati per evitare cicli nidificati ripetitivi.
        """
        gx_center = int(np.round(x / self.resolution))
        gy_center = int(np.round(y / self.resolution))

        if safety_margin <= 0.0:
            return self.grid.get((gx_center, gy_center), 1.0) == -1.0

        # Rigenera la maschera di vicinato solo se il margine cambia
        if self._cached_margin != safety_margin:
            self._cached_margin = safety_margin
            steps = int(np.ceil(safety_margin / self.resolution))
            offsets = []
            for dx in range(-steps, steps + 1):
                for dy in range(-steps, steps + 1):
                    if dx ** 2 + dy ** 2 <= steps ** 2:
                        offsets.append((dx, dy))
            self._cached_offsets = offsets

        for dx, dy in self._cached_offsets:
            if self.grid.get((gx_center + dx, gy_center + dy), 1.0) == -1.0:
                return True

        return False

    def save_map(self, file_path):
        """Salva la mappa globale di occupazione in un file pickle."""
        with open(file_path, 'wb') as f:
            pickle.dump({
                'resolution': self.resolution,
                'grid': self.grid
            }, f)
        print(f"[MAP] Mappa globale salvata correttamente in {file_path}")

    def load_map(self, file_path):
        """Ricarica una mappa precedentemente salvata."""
        if os.path.exists(file_path):
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
                self.resolution = data['resolution']
                self.grid = data['grid']
            print(f"[MAP] Mappa globale caricata con {len(self.grid)} celle occupate/libere.")
        else:
            print(f"[WARNING] File {file_path} non trovato. Inizializzo mappa vuota.")

