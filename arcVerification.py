import threading
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot_state import RobotStateClient

import spotUtils
import spotGrid
import time
import logging
import numpy as np


LOGGER = logging.getLogger(__name__)

def _update_thread(async_task):
    while True:
        async_task.update()
        time.sleep(0.1)

class AsyncArcVerificationTracker(AsyncPeriodicQuery):

    def __init__(self, robot_state_client):
        super(AsyncArcVerificationTracker, self).__init__('arc_verification', robot_state_client, LOGGER,
                                                          period_sec=0.2)

    def _start_query(self):
        return self._client.get_arc_verification(['arc_verification'])

class ArcVerificationTracker:
    """Tracks which PRM arcs have been verified as safe to traverse."""

    def __init__(self, robot):
        self.robot = robot
        self.verified_arcs = set()
        self.blocked_arcs = set()
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._local_grid = spotGrid.LocalGrid(self.robot)
        self.path = []  # List of tuples: (node_id, x, y, status)
        self.path_lock = threading.Lock()
        self._running = False
        self._thread = None

    def update_path(self, new_path):
        """Aggiorna il path in modo thread-safe ricevendo tuple (node_id, x, y)."""
        with self.path_lock:
            # Inizializziamo lo status dell'arco a None
            self.path = [(node_id, x, y, None) for node_id, x, y in new_path]
            print(f"[TRACKER] Path aggiornato: {len(self.path)} nodi.")

    def set_arc_status_by_nodes(self, node_id1, node_id2, status):
        """
        Aggiorna in modo thread-safe lo status dell'arco identificato dai node IDs.
        Risolve il bug di disallineamento degli indici se il path cambia dinamicamente.
        """
        with self.path_lock:
            for i in range(len(self.path) - 1):
                n1 = self.path[i][0]
                n2 = self.path[i + 1][0]

                # Controllo se l'arco corrisponde (indipendente dall'ordine)
                if {n1, n2} == {node_id1, node_id2}:
                    # Mantieni node_id, x, y originari ma aggiorna lo status dell'arco
                    self.path[i] = (n1, self.path[i][1], self.path[i][2], status)
                    print(f"[TRACKER] Stato arco {node_id1}-{node_id2} impostato a: {status}")
                    break

    def get_path_copy(self):
        with self.path_lock:
            return list(self.path)

    def mark_arc_verified(self, node_id1, node_id2):
        """Mark arc as verified. Order-independent."""
        arc = tuple(sorted([node_id1, node_id2]))
        self.verified_arcs.add(arc)

    def mark_arc_blocked(self, node_id1, node_id2):
        """Mark arc as blocked."""
        arc = tuple(sorted([node_id1, node_id2]))
        self.blocked_arcs.add(arc)

    def is_arc_verified(self, node_id1, node_id2):
        """Check if arc has been verified."""
        arc = tuple(sorted([node_id1, node_id2]))
        return arc in self.verified_arcs

    def is_arc_blocked(self, node_id1, node_id2):
        """Check if arc is known to be blocked."""
        arc = tuple(sorted([node_id1, node_id2]))
        return arc in self.blocked_arcs

    def start(self):
        """Starts the background verification thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()
        print("ArcVerificationTracker started successfully in background.")

    def stop(self):
        self._running = False
        print('ArcVerificationTracker stopped.')

    def get_blocked_arcs(self):
        """Restituisce una copia thread-safe degli archi bloccati per evitare collisioni tra thread."""
        with self.path_lock:
            return list(self.blocked_arcs)

    def _process_loop(self):
        while self._running:
            current_path = self.get_path_copy()

            if len(current_path) < 2:
                time.sleep(0.2)
                continue

            try:
                # Recuperiamo posizione robot REALE
                robot_x, robot_y, robot_z, robot_quat = spotUtils.getPosition(self._robot_state_client)
                robot_yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                                       1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))

                # 1. Scarichiamo tutti i layer necessari (ATTENZIONE A SPACCHETTARE 3 VALORI)
                grids_data, main_proto, all_grids = self._local_grid.return_local_grid(
                    ['obstacle_distance', 'terrain', 'terrain_valid'],
                    robot_state_client=self._robot_state_client
                )

                # Se grids_data è None (in caso di errore), return_local_grid restituisce None, None, None
                if grids_data is None or 'obstacle_distance' not in grids_data:
                    time.sleep(0.2)
                    continue

                pts = grids_data['obstacle_distance']['pts']
                cells_obs = grids_data['obstacle_distance']['values']
                terrain_vals = grids_data['terrain']['values']
                valid_vals = grids_data['terrain_valid']['values']

                num_x = main_proto.local_grid.extent.num_cells_x
                num_y = main_proto.local_grid.extent.num_cells_y
                cell_size = main_proto.local_grid.extent.cell_size

                # 2. Calcolo derivate e footprint mask
                footprint_mask_2d = self._local_grid.compute_robot_footprint_mask(
                    pts, robot_x, robot_y, robot_yaw, num_x, num_y
                )

                grad_vals, rough_vals = self._local_grid.compute_gradient_and_roughness(
                    terrain_vals, valid_vals, num_x, num_y, cell_size,
                    robot_footprint_mask_2d=footprint_mask_2d
                )

                # 3. FUSIONE DEI LAYER IN BACKGROUND
                _, obstacle_mask_fused = self._local_grid.fuse_all_layers(
                    pts=pts,
                    cells_obstacle_dist=cells_obs,
                    terrain_values=terrain_vals,
                    grad_values=grad_vals,
                    rough_values=rough_vals,
                    robot_footprint_mask=(footprint_mask_2d.ravel() if footprint_mask_2d is not None else None),
                    robot_z=robot_z,
                    terrain_valid_values=valid_vals,
                    obstacle_threshold=0.15,
                    slope_threshold=0.6,  # Soglia pendenza per il tracker
                    rough_threshold=0.15
                )

            except Exception as e:
                LOGGER.error(f"Errore nel recupero dati sensori/stato: {e}")
                time.sleep(0.2)
                continue

            # Scorriamo la copia locale del percorso
            for i in range(len(current_path) - 1):
                node1, x1, y1, _ = current_path[i]
                node2, x2, y2, _ = current_path[i + 1]

                if self.is_arc_verified(node1, node2) or self.is_arc_blocked(node1, node2):
                    continue

                if is_arc_in_fov(x1, y1, x2, y2, pts):
                    # PASSIAMO LA MASCHERA FUSA A VERIFY_ARC_SAFETY!
                    status = verify_arc_safety(x1, y1, x2, y2, pts, obstacle_mask_fused)

                    if status == 'clear':
                        self.mark_arc_verified(node1, node2)
                        self.set_arc_status_by_nodes(node1, node2, True)

                    elif status == 'blocked':
                        self.mark_arc_blocked(node1, node2)
                        self.set_arc_status_by_nodes(node1, node2, False)

            time.sleep(0.2)


def is_arc_in_fov(x1, y1, x2, y2, pts, fov_margin=0.0):
    """
    Determine if an arc (edge) is within the camera's local map grid bounds.
    Both endpoints and midpoint must be within local grid bounds.
    """
    if len(pts) == 0:
        return False

    x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
    y_min, y_max = pts[:, 1].min(), pts[:, 1].max()

    # Expand bounds with margin
    x_min -= fov_margin
    x_max += fov_margin
    y_min -= fov_margin
    y_max += fov_margin

    # Check if both endpoints are in grid range
    p1_in_fov = (x_min <= x1 <= x_max) and (y_min <= y1 <= y_max)
    p2_in_fov = (x_min <= x2 <= x_max) and (y_min <= y2 <= y_max)

    # Check midpoint as well
    mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
    mid_in_fov = (x_min <= mid_x <= x_max) and (y_min <= mid_y <= y_max)

    return p1_in_fov and p2_in_fov and mid_in_fov


def verify_arc_safety(x1, y1, x2, y2, pts, obstacle_mask):
    """
    Verify if an arc is safe to traverse (no obstacles blocking it).
    """
    return spotUtils.check_line_of_sight(x1, y1, x2, y2, pts, obstacle_mask)


def get_visible_arcs_from_path(path_coords, robot_x, robot_y, pts, prm_graph):
    """
    Get list of arcs (edges) from the path that are currently visible in FOV.
    """
    if len(path_coords) < 2:
        return []

    visible_arcs = []

    # Check arcs between robot position and first waypoint
    if len(path_coords) > 0:
        next_x, next_y = path_coords[0]
        if is_arc_in_fov(robot_x, robot_y, next_x, next_y, pts):
            start_node = prm_graph.get_nearest_node(robot_x, robot_y)
            next_node = prm_graph.get_nearest_node(next_x, next_y)
            visible_arcs.append((start_node, next_node, 'visible'))

    # Check arcs between consecutive waypoints
    for i in range(len(path_coords) - 1):
        x1, y1 = path_coords[i]
        x2, y2 = path_coords[i + 1]

        if is_arc_in_fov(x1, y1, x2, y2, pts):
            node1 = prm_graph.get_nearest_node(x1, y1)
            node2 = prm_graph.get_nearest_node(x2, y2)
            visible_arcs.append((node1, node2, 'visible'))

    return visible_arcs

