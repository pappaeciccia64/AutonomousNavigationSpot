"""La griglia locale di Spot: lettura dei layer, filtro del suolo, terreno corretto, pendenza, rugosita', maschera degli ostacoli."""
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

# Il layer `terrain_valid` di Spot contiene 0 e 1, ma dopo scala e offset lo 0 diventa
# +/-5.5e-17 con segno variabile: si confronta con 0.5, insensibile al residuo.
TERRAIN_VALID_MIN = 0.5

# =========================================================================
# FILTRO SULL'ALTEZZA DEL SUOLO (vedi LocalGrid.correct_terrain)
#
# Le quote di `terrain` sono rispetto al riferimento fissato all'accensione. Il suolo su cui
# il robot sta in piedi si stima come mediana delle celle valide entro GROUND_REF_RADIUS_M;
# le celle oltre GROUND_HEIGHT_TOLERANCE_M dal suolo si trattano in modo ASIMMETRICO:
#   - SOPRA: non e' superficie calpestabile (muro, albero, cella mai scritta). Esclusa da
#     pendenza e rugosita' e riempita col suolo vicino; l'ostacolo resta in obstacle_distance.
#   - SOTTO: puo' essere un vuoto vero (muretto, scala che scende, fosso): non si tocca,
#     riempirla nasconderebbe un dirupo.
# 1.2 m: a ~1.9 m dal robot (bordo della finestra) sono gia' ~32 gradi, oltre il limite di
# Spot, quindi nessuna salita affrontabile viene scartata.
# =========================================================================
GROUND_HEIGHT_TOLERANCE_M = 1.2
GROUND_REF_RADIUS_M = 0.7            # celle sotto/attorno al robot usate per stimare il suolo
GROUND_REF_FALLBACK_RADIUS_M = 1.2   # se sotto il robot ci sono troppe poche celle valide
GROUND_REF_MIN_CELLS = 50

# -------------------------------------------------------------------------
# CELLE MAI SCRITTE (vedi LocalGrid._cells_never_written)
# Le celle che i sensori non hanno mai scritto restano tutte alla stessa quota esatta (un
# "plateau"). Spot le dichiara quasi sempre non valide; questa regola cattura il resto.
# -------------------------------------------------------------------------
UNWRITTEN_MATCH_FRACTION = 0.25    # tolleranza: 0.25 * passo di quantizzazione
UNWRITTEN_MATCH_FALLBACK_M = 1e-4  # se il passo non e' noto
UNWRITTEN_MIN_CELLS = 100          # celle che devono condividere la quota perche' sia un plateau
UNWRITTEN_MIN_SPOT_INVALID = 0.80  # frazione del plateau che Spot deve marcare non valida


def fill_invalid_nearest(values_2d, invalid_mask):
    """
    Riempie le celle non attendibili (invalid_mask True) con il valore della cella
    attendibile piu' vicina: celle che Spot dichiara non valide e impronta del robot.

    Restituisce una copia; se tutto e' valido, o tutto non valido, i valori invariati.
    """
    if not np.any(invalid_mask) or not np.any(~invalid_mask):
        return values_2d.copy()
    nearest_indices = distance_transform_edt(invalid_mask, return_distances=False, return_indices=True)
    return values_2d[tuple(nearest_indices)]


class Scan:
    """
    Una scansione della griglia locale (vedi LocalGrid.fetch_scan e process_scan). Array piatti,
    nell'ordine delle celle della griglia; (num_y, num_x) per rimetterli in 2D.
    """

    def __init__(self, **fields):
        self.__dict__.update(fields)


class LocalGrid:
    def __init__(self, robot):
        self.local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
        self.footprint_correction_pending = True
        # Esito dell'ultimo filtro sull'altezza del suolo (vedi _cells_above_ground).
        self.last_ground_filter = {'ground_z': None, 'n_above': 0, 'max_above': 0.0,
                                   'source': 'non ancora eseguito', 'n_ref': 0}

    def create_vtk_no_step_grid(self, proto, robot_state_client):
        """
        Layer "no_step" della risposta: (punti (x, y, z) nel frame VISION, valori, colori RGB:
        rosso = non calpestabile, blu = calpestabile). Array vuoti se il layer manca.
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
        Layer "obstacle_distance": distanza col segno (m) dall'ostacolo piu' vicino, negativa
        dentro un ostacolo. A differenza di no_step non tratta l'erba come ostacolo, per
        questo e' il layer usato per gli ostacoli.

        Returns:
            (punti (N, 3) nel frame VISION, distanze (N,), colori RGB (N, 3): rosso dentro un
            ostacolo, blu fuori). Array vuoti se il layer manca.
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
        Porta i punti dal frame della griglia locale al frame VISION (rotazione e traslazione
        complete), riferendoli al CENTRO di ogni cella.
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
        Scarica i layer richiesti con UNA sola richiesta (stessa fotografia per tutti); se non
        funziona ripiega su una richiesta per layer.
        """
        if getattr(self, '_multi_request_ok', True):
            try:
                out = list(self.local_grid_client.get_local_grids(list(types_of_grid)))
                self._multi_request_fails = 0
                return out
            except Exception as e:
                # Dopo 3 fallimenti di fila non la si riprova piu' (una richiesta che fallisce
                # sempre costerebbe un giro di rete a ogni scansione); un singolo errore di
                # rete invece non la spegne.
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
        """(origine x, origine y, yaw in gradi, nx, ny, cella) del layer nel frame VISION."""
        g = lg.local_grid
        tf = get_a_tform_b(g.transforms_snapshot, VISION_FRAME_NAME, g.frame_name_local_grid_data)
        q = tf.rotation
        yaw = np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        return (tf.position.x, tf.position.y, float(yaw), g.extent.num_cells_x, g.extent.num_cells_y,
                g.extent.cell_size)

    def fetch_scan(self, robot_state_client):
        """
        Scarica i layer di una scansione (obstacle_distance, terrain, terrain_valid) e ne legge
        valori e geometria. Restituisce una Scan, o None se i layer non arrivano. L'elaborazione
        (correzione del terreno, pendenza, maschera degli ostacoli) la fa process_scan.
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
        """Origine (x, y) della griglia della scansione nel frame VISION."""
        g = scan.proto.local_grid
        tf = get_a_tform_b(g.transforms_snapshot, VISION_FRAME_NAME, g.frame_name_local_grid_data)
        return tf.position.x, tf.position.y

    def process_scan(self, scan, robot_x, robot_y, robot_yaw, footprint=True, fuse=True,
                     ground_label=None):
        """
        Elabora una scansione letta con fetch_scan, riempiendo la Scan:
          footprint_mask  celle sotto il corpo del robot (footprint=False: nessuna)
          terrain, is_valid, unwritten  terreno corretto (vedi correct_terrain): da qui in
                          poi si usa solo questo
          grad, rough     pendenza e rugosita' sul terreno corretto
          obstacle_mask   veto per cella, pura logica di soglia (fuse=False: non calcolata).
                          La pendenza non entra in questo veto: vedi fuse_obstacle_mask
        ground_label: etichetta della riga [GROUND-FILTER] (None = niente riga). Restituisce
        la Scan.
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
        Layer in piu' da SALVARE soltanto (es. 'intensity', 'no_step'), con una richiesta a
        parte: non entrano nei controlli di allineamento di return_local_grid, quindi se
        mancano o hanno un'altra geometria la scansione usata per navigare non ne risente.
        Non solleva mai. Restituisce {nome: dict(values (ny, nx) float32, origin (x, y) nel
        frame VISION, cell)}; {} se la richiesta fallisce.
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
        Scarica e decodifica i layer richiesti.

        Restituisce (grids_data, main_proto, protos), oppure (None, None, None) se dopo
        max_attempts tentativi manca anche un solo layer o i layer non sono allineati fra
        loro (origine, dimensioni o cella diverse: se il robot si muove fra una richiesta e
        l'altra la griglia puo' ricentrarsi). main_proto e' il primo layer richiesto: la sua
        origine indicizza tutti i layer.

        Avvisa una volta se la griglia e' ruotata rispetto a VISION: tutto il codice assume
        che non lo sia (indici di cella calcolati con la sola traslazione).
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
        # main_proto = il primo layer richiesto (obstacle_distance in tutti i chiamanti):
        # la sua origine e' quella con cui si indicizzano TUTTI i layer.
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
                    # Passo di quantizzazione delle quote di QUESTA scansione: tolleranza per
                    # _cells_never_written. 'raw_zero' fa solo da interruttore di quella
                    # regola (None = spenta).
                    grids_data[grid_type]['raw_zero'] = float(np.float32(lg.local_grid.cell_value_offset))
                    grids_data[grid_type]['scale'] = float(lg.local_grid.cell_value_scale)

        return grids_data, main_proto, p

    def create_vtk_terrain_grid(self, proto, robot_state_client, layer_name='terrain'):
        """
        Layer di tipo terreno (terrain, terrain_valid, intensity): (punti nel frame VISION,
        valori, colori). Per `terrain` il valore restituito e' la quota in VISION.
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

        # Le quote decodificate sono nel frame della griglia locale; dopo offset_grid_pixels
        # pts[:, 2] e' la quota nel frame VISION, che e' quella che si restituisce.
        terrain_values_vision = pts[:, 2].astype(np.float32)

        color = np.zeros([len(z_vals), 3], dtype=np.uint8)
        color[:, 1] = 255  # green

        return pts, terrain_values_vision, color

    def correct_terrain(self, terrain_values, terrain_valid_values, num_cells_x, num_cells_y,
                        robot_footprint_mask=None, cell_size=0.03, robot_rc=None,
                        unwritten_value=None, unwritten_scale=None, return_unwritten=False):
        """
        Unico punto di correzione del terreno. Le celle non attendibili -- non valide per
        Spot, sotto il corpo del robot, troppo sopra il suolo (_cells_above_ground), mai
        scritte dai sensori (_cells_never_written) -- vengono riempite con la quota della
        cella attendibile piu' vicina e marcate non valide. Tutto il resto del codice usa
        questo terreno corretto, e is_valid impedisce che un valore riempito decida qualcosa.

        Args:
            terrain_values: quote grezze (piatte o (num_cells_y, num_cells_x))
            terrain_valid_values: layer terrain_valid di Spot (attendibile se >= TERRAIN_VALID_MIN)
            robot_footprint_mask: maschera 2D facoltativa, True sotto il corpo del robot
            unwritten_value: interruttore della regola delle celle mai scritte
                (grids_data['terrain']['raw_zero']); None = regola spenta
            unwritten_scale: passo di quantizzazione (grids_data['terrain']['scale'])
            return_unwritten: restituisce anche la maschera delle celle mai scritte

        Returns:
            terrain_corrected: quote corrette, 1D
            is_valid: 1D, True dove la lettura ORIGINALE era attendibile
            unwritten: (solo con return_unwritten) 1D, celle mai scritte, da trattare come
                NON VISTE (vedi arcVerification.make_grid_snapshot)
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

        # Filtro sull'altezza del suolo -- vedi GROUND_HEIGHT_TOLERANCE_M.
        above_ground = self._cells_above_ground(terrain_2d, ~fill_mask, cell_size, robot_rc)
        # Celle mai scritte: si passa valid_2d (la bandiera di Spot), non ~fill_mask: vedi
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
        Celle che i sensori non hanno mai scritto: restano tutte alla STESSA quota esatta.

        La quota non si prevede (dipende dal frame e Spot sposta l'offset a ogni scansione): si
        RICAVA come la quota esatta piu' frequente fra le celle che Spot stesso dichiara non
        attendibili. Se almeno UNWRITTEN_MIN_CELLS la condividono (e almeno
        UNWRITTEN_MIN_SPOT_INVALID di quelle a quella quota sono non valide per Spot), ogni
        cella a quella quota e' mai scritta, comprese le poche che Spot marca buone per errore.
        E' una rete: quasi tutte queste celle Spot le marca gia' non valide.

        unwritten_value fa solo da interruttore (None = regola spenta).
        Aggiorna self.last_ground_filter['n_unwritten'] e ['n_unwritten_below'].
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
        # Un plateau vero e' fatto di celle che Spot stesso dichiara non attendibili. Se
        # quella quota coincidesse con una superficie REALE (il piano di un tavolo
        # esattamente a quell'altezza) le celle sarebbero in gran parte valide: non si tocca.
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
        Celle che stanno piu' di GROUND_HEIGHT_TOLERANCE_M SOPRA il suolo su cui il robot
        e' appoggiato. Solo sopra: vedi il commento alla costante per il perche'.

        Il suolo e' la mediana delle celle attendibili entro GROUND_REF_RADIUS_M dal robot.
        La griglia locale di Spot e' centrata sul robot (entro ~5 cm), quindi di default il
        robot e' al centro; robot_rc=(riga, colonna) lo sovrascrive.

        Registra l'esito in self.last_ground_filter per i log e il salvataggio dati.
        Restituisce una maschera 2D booleana (tutta False se il suolo non e' stimabile).
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
        Layer di pendenza e rugosita' per cella, da un terreno GIA' corretto (vedi
        correct_terrain). Le celle che Spot dichiara non valide valgono 0 nell'uscita.

        PENDENZA: differenza centrata su una base fisica (vedi SLOPE_BASELINE_M: ~0.48 m,
        circa un passo) invece che sulla cella da 3 cm, su cui un piccolo bordo varrebbe
        quanto una salita vera. Questo campo per cella serve ai nodi e alle figure; la
        pendenza che decide e' quella per arco (compute_arc_slope_profile).

        RUGOSITA': deviazione standard della quota su una finestra corta (5 celle, 15 cm),
        per il terreno irregolare sotto il piede: allargarla nasconderebbe proprio quello.
        Le statistiche usano solo celle attendibili (is_valid).
        """
        terrain_2d = terrain_corrected.reshape((num_cells_y, num_cells_x)).astype(np.float64)

        if terrain_valid_values is not None and terrain_valid_values.size > 0:
            sensor_invalid_mask = ~(terrain_valid_values.reshape((num_cells_y, num_cells_x)) >= TERRAIN_VALID_MIN)
        else:
            sensor_invalid_mask = np.zeros((num_cells_y, num_cells_x), dtype=bool)

        window_size = 5

        # Celle che possono entrare nelle statistiche di vicinato. Con is_valid (uscita di
        # correct_terrain) restano fuori anche le celle riempite e quelle scartate dal
        # filtro sull'altezza del suolo: altrimenti una cella non attendibile "contamina" la
        # rugosita' delle vicine valide nella finestra 5x5.
        if is_valid is not None:
            weight_2d = np.asarray(is_valid).reshape((num_cells_y, num_cells_x)).astype(np.float64)
        else:
            weight_2d = (~sensor_invalid_mask).astype(np.float64)

        # 1. Smoothing "normalizzato": media delle sole celle valide della finestra.
        #    Dove nella finestra non c'e' nessuna cella valida si tiene il terreno corretto.
        n_valid = uniform_filter(weight_2d, size=window_size)
        sum_t = uniform_filter(terrain_2d * weight_2d, size=window_size)
        sum_t2 = uniform_filter((terrain_2d ** 2) * weight_2d, size=window_size)
        has_support = n_valid > (0.5 / (window_size * window_size))
        with np.errstate(invalid='ignore', divide='ignore'):
            mean_t = np.where(has_support, sum_t / n_valid, terrain_2d)
            mean_t2 = np.where(has_support, sum_t2 / n_valid, terrain_2d ** 2)

        # 2. Calcolo Gradiente (Pendenza) su una base fisica, non sulla spaziatura nativa della cella
        k = max(1, round(slope_baseline_m / cell_size))  # es. 0.25m / 0.03m =~ 8 celle

        def _centered_diff(arr, k, axis):
            # Differenza centrata a distanza k celle, con bordo "clampato" (edge padding)
            # cosi' non introduciamo pendenze finte ai margini della griglia locale.
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

        # 3. Calcolo Rugosita' (Deviazione Standard locale, solo celle valide) -- finestra corta invariata
        variance = mean_t2 - (mean_t ** 2)
        roughness_2d = np.sqrt(np.maximum(0.0, variance))
        # Con meno di 3 celle valide nella finestra la deviazione standard non significa nulla.
        roughness_2d[n_valid * window_size * window_size < 3] = 0.0

        # Azzeriamo SOLO le zone realmente cieche nell'output finale.
        # L'impronta del robot mantiene invece la stima interpolata realistica.
        gradient_2d[sensor_invalid_mask] = 0.0
        roughness_2d[sensor_invalid_mask] = 0.0

        return gradient_2d.ravel(), roughness_2d.ravel()


    def fuse_obstacle_mask(self, cells_obstacle_dist,
                        rough_values,
                        is_valid,
                        obstacle_threshold=OBSTACLE_THRESHOLD,
                        rough_threshold=ROUGH_THRESHOLD):
        """
        Maschera degli ostacoli per cella: -1 ostacolo, +1 libero. Una cella attendibile e'
        ostacolo se obstacle_distance <= obstacle_threshold oppure rugosita' >
        rough_threshold; una cella non attendibile (riempita) non e' mai ostacolo.

        La pendenza non entra qui: si valuta per ARCO (compute_arc_slope_profile), su tutta
        la sua lunghezza e nelle due direzioni. La rugosita' invece e' una proprieta' locale.
        """
        cells_obstacle_dist = cells_obstacle_dist.ravel()
        rough_values = rough_values.ravel()
        is_valid = is_valid.ravel()

        assert cells_obstacle_dist.shape == rough_values.shape == is_valid.shape, \
            "Errore: I layer ambientali hanno dimensioni disallineate!"

        # Calcolo dei VETI applicato SOLO alle zone con dati validi
        # Se una zona NON e' valida (punto cieco o impronta robot), non deve attivare il veto di ostacolo
        v1 = (cells_obstacle_dist <= obstacle_threshold) & is_valid
        v3 = (rough_values > rough_threshold) & is_valid

        # Maschera binaria finale degli ostacoli
        obstacle_mask = v1 | v3
        obstacle_mask = np.where(obstacle_mask, -1.0, 1.0)

        return obstacle_mask

    def compute_robot_footprint_mask(self, pts, robot_x, robot_y, robot_yaw, num_cells_x, num_cells_y, margin=0.05,
                                     body_length=1.10, body_width=0.50):
        """
        Maschera 2D delle celle sotto il corpo del robot (rettangolo orientato, piu' margin).
        Viene restituita UNA SOLA VOLTA per missione, alla prima chiamata (alla prima
        scansione il terreno sotto il robot non e' mai stato visto); dopo restituisce None e
        vale il terrain_valid di Spot.

        body_length / body_width: dimensioni del corpo.

        Attenzione: la chiama anche il thread del fronte (arcVerification), quindi la maschera
        "una volta sola" puo' consumarla lui per primo.
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
    Maschera delle celle le cui QUOTE possono
    entrare nel calcolo della pendenza di un arco: attendibili (is_valid di correct_terrain)
    E non-ostacolo.

    La faccia verticale di un mobile non e' terreno in pendenza, e' un ostacolo -- ed e' gia'
    gestita come tale due volte: dal veto di occupazione del PRM e dal fronte sicuro, che si
    ferma a FRONTIER_CLEARANCE_M da qualunque cella con obstacle_distance bassa. Contarla
    ANCHE come pendenza la conta la terza volta, e con il peggiore degli effetti: non ferma
    il robot davanti al mobile (ci pensa il fronte), scollega il grafo PRM in tutta la stanza.

    Gli archi che perdono il dato NON diventano "liberi per decreto": diventano "senza dati",
    e un arco senza dati non viene vetato (entra nel grafo con un costo di ripiego, vedi
    PRM._evaluate_and_add_edge); chi lo percorre lo verifica metro per metro col fronte
    sicuro. E' una scelta esplicita: su quegli archi la sicurezza la da' il fronte, non una
    pendenza misurata su un armadio.

    La maschera si combina QUI e non dentro prm_graph perche' e' qui che si hanno in mano
    entrambi gli array, e perche' in prm_graph `current_valid_2d` serve a una cosa sola -- il
    profilo di pendenza per-arco (compute_arc_slope_profile) -- quindi restringerla non ha
    altri effetti. Chi legge prm_graph.update_local_grid_data trova `valid_2d` documentato
    come "celle attendibili": qui significa "attendibili come TERRENO".
    """
    valid = np.asarray(is_valid_flat, dtype=bool).reshape(shape)
    if cells_obstacle_dist is None:
        return valid
    od = np.asarray(cells_obstacle_dist, dtype=np.float64).reshape(shape)
    return valid & ~(od < OBSTACLE_THRESHOLD)


def raw_zero(grids_data):
    """Quota del valore grezzo 0 del layer terrain di questa scansione (None se assente)."""
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
    Una riga solo quando il filtro sull'altezza del suolo ha scartato qualcosa o non ha
    potuto stimare il suolo -- nel caso normale tace. Vedi GROUND_HEIGHT_TOLERANCE_M.
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
