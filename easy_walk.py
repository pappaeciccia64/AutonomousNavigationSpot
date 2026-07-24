import os
import sys
import time
from time import sleep
from datetime import datetime
import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.frame_helpers import *
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, blocking_stand)
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.frame_helpers import get_a_tform_b
from types import SimpleNamespace
import navGraphUtils
import movements
import spotGrid
import spotLogInUtils
import environmentMap
import spotUtils
#import velodyneClient
import arcVerification

import global_sampler
import prm_graph

import threading
from concurrent.futures import ThreadPoolExecutor

import matplotlib
# CRUCIALE: Impostare il backend 'Agg' PRIMA di importare pyplot!
# Questo disabilita Tkinter ed evita qualsiasi crash multithread/SIGABRT.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection

# Thread safety and background execution setup
_VIS_LOCK = threading.Lock()
_VIS_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vis_worker")

# TODO: check if we can avoid to set a sleep after each movement command
# TODO: change the folder destination of the name download of graph

def sample_cell_points(env, cell_row, cell_col, num_samples=200):
    """Sample random points within a cell."""
    world_pos = env.get_world_position_from_cell(cell_row, cell_col)
    if world_pos is None:
        return []

    cell_center_x, cell_center_y = world_pos
    half_size = env.cell_size / 2.0

    samples = []
    for _ in range(num_samples):
        offset_x = np.random.uniform(-half_size * 0.8, half_size * 0.8)
        offset_y = np.random.uniform(-half_size * 0.8, half_size * 0.8)

        cos_yaw = np.cos(env.origin_yaw)
        sin_yaw = np.sin(env.origin_yaw)

        world_offset_x = offset_x * cos_yaw - offset_y * sin_yaw
        world_offset_y = offset_x * sin_yaw + offset_y * cos_yaw

        sample_x = cell_center_x + world_offset_x
        sample_y = cell_center_y + world_offset_y

        samples.append((sample_x, sample_y))

    return samples

#TODO: test this part. Now this method take a point nearest to the center of the cell.
def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist, global_sampler):
    """Sample random points in a cell and explicitly return the cell center as the main target point."""
    target_cell_points = global_sampler.get_point_in_cell(cell_row, cell_col)

    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center
    rejected_samples = []

    # Forziamo il ritorno esatto del centro geometrico della cella
    return cell_center_x, cell_center_y, target_cell_points, rejected_samples


def visualize_grid_with_candidates(pts, terrain_real, obstacle_mask, robot_x, robot_y,
                                   candidates, chosen_point, iteration, env, save_path,
                                   prm_graph=None, chosen_path=None,
                                   cells_obstacle_dist=None, intensity_values=None,
                                   valid_values=None, grad_values=None, rough_values=None,
                                   include_diagnostics=True):
    """
    Non-blocking async wrapper. Snapshots current data and offloads heavy Matplotlib
    rendering to a background thread executor so robot navigation is not delayed.
    """
    if save_path is None:
        return

    # 1. Fast snapshot of NumPy arrays to prevent thread race conditions
    pts_snap = pts.copy() if pts is not None else None
    terrain_snap = terrain_real.copy() if terrain_real is not None else None
    obs_snap = obstacle_mask.copy() if obstacle_mask is not None else None
    dist_snap = cells_obstacle_dist.copy() if cells_obstacle_dist is not None else None
    valid_snap = valid_values.copy() if valid_values is not None else None
    grad_snap = grad_values.copy() if grad_values is not None else None
    rough_snap = rough_values.copy() if rough_values is not None else None

    # 2. Extract graph snapshots safely
    prm_nodes = dict(prm_graph.nodes) if (prm_graph and hasattr(prm_graph, 'nodes')) else {}
    prm_edges = dict(prm_graph.edges) if (prm_graph and hasattr(prm_graph, 'edges')) else {}

    # 3. Snapshot environment structures safely
    env_info = None
    if env is not None:
        # Pre-compute vectorized global accumulation on main thread (takes < 1ms)
        ACCUM_RES = 0.05
        if not hasattr(env, '_accumulated_pts'):
            env._accumulated_pts = {}

        # Compute colors for accumulation
        fused_colors_temp = np.zeros((len(obs_snap), 3), dtype=np.float32) if obs_snap is not None else np.zeros((0, 3))
        if terrain_snap is not None:
            z_terrain = terrain_snap.ravel()
            z_min, z_max = z_terrain.min(), z_terrain.max()
            z_norm = (z_terrain - z_min) / (z_max - z_min) if (z_max - z_min) > 0.001 else np.zeros_like(z_terrain)
            cmap_walkable = matplotlib.colormaps.get_cmap('YlGn')
            fused_colors_temp[:] = cmap_walkable(z_norm)[:, :3]
        else:
            fused_colors_temp[:] = [0.9, 0.9, 0.9]

        if obs_snap is not None:
            fused_colors_temp[obs_snap.ravel() == -1] = [1.0, 0.0, 0.0]

        # Vectorized key computation
        keys = (np.round(pts_snap[:, :2] / ACCUM_RES)).astype(np.int32)
        colors_uint8 = (fused_colors_temp * 255).astype(np.uint8)
        for k, col in zip(keys, colors_uint8):
            env._accumulated_pts[tuple(k)] = col

        # Extract accumulated points array snapshot
        if env._accumulated_pts:
            accum_keys = np.array(list(env._accumulated_pts.keys()), dtype=np.float32)
            accum_wx = accum_keys[:, 0] * ACCUM_RES
            accum_wy = accum_keys[:, 1] * ACCUM_RES
            accum_colors = np.array(list(env._accumulated_pts.values()), dtype=np.float32) / 255.0
        else:
            accum_wx = np.array([robot_x], dtype=np.float32)
            accum_wy = np.array([robot_y], dtype=np.float32)
            accum_colors = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)

        traveled_arcs = list(getattr(env, '_traveled_arcs', []))

        # Snapshot grid cell parameters
        grid_cells = []
        cos_yaw, sin_yaw = np.cos(env.origin_yaw), np.sin(env.origin_yaw)
        half_size = env.cell_size / 2.0
        base_corners = np.array([[-half_size, -half_size], [half_size, -half_size],
                                 [half_size, half_size], [-half_size, half_size]])
        rot_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

        for row in range(env.rows):
            for col in range(env.cols):
                world_pos = env.get_world_position_from_cell(row, col)
                if world_pos is None:
                    continue
                cell_x, cell_y = world_pos
                world_corners = np.dot(base_corners, rot_matrix.T) + [cell_x, cell_y]
                status_res = env.get_cell_status(row, col)
                cell_status = status_res[0] if isinstance(status_res, (tuple, list)) else status_res
                grid_cells.append((cell_x, cell_y, row, col, world_corners, cell_status))

        env_info = {
            'cell_size': env.cell_size,
            'accum_wx': accum_wx,
            'accum_wy': accum_wy,
            'accum_colors': accum_colors,
            'traveled_arcs': traveled_arcs,
            'grid_cells': grid_cells
        }

    # 4. Dispatch rendering asynchronously to worker queue
    _VIS_EXECUTOR.submit(
        _async_render_worker,
        pts_snap, terrain_snap, obs_snap, robot_x, robot_y, candidates,
        chosen_point, iteration, env_info, save_path, prm_nodes, prm_edges,
        chosen_path, dist_snap, valid_snap, grad_snap, rough_snap, include_diagnostics
    )


def _async_render_worker(pts, terrain_real, obstacle_mask, robot_x, robot_y,
                         candidates, chosen_point, iteration, env_info, save_path,
                         prm_nodes, prm_edges, chosen_path, cells_obstacle_dist,
                         valid_values, grad_values, rough_values, include_diagnostics):
    """Background thread worker handling figure creation and file I/O."""
    with _VIS_LOCK:
        try:
            x = pts[:, 0]
            y = pts[:, 1]

            ZOOM_RADIUS = 3.0
            local_xmin_zoom, local_xmax_zoom = robot_x - ZOOM_RADIUS, robot_x + ZOOM_RADIUS
            local_ymin_zoom, local_ymax_zoom = robot_y - ZOOM_RADIUS, robot_y + ZOOM_RADIUS
            local_x_min, local_x_max = x.min(), x.max()
            local_y_min, local_y_max = y.min(), y.max()

            def _apply_common_axis_settings(target_ax, title_text):
                target_ax.set_xlim(local_xmin_zoom, local_xmax_zoom)
                target_ax.set_ylim(local_ymin_zoom, local_ymax_zoom)
                target_ax.set_aspect('equal', adjustable='box')
                target_ax.set_xlabel('X [m] (VISION)', fontsize=11, fontweight='bold')
                target_ax.set_ylabel('Y [m] (VISION)', fontsize=11, fontweight='bold')
                target_ax.set_title(title_text, fontsize=12, fontweight='bold')
                target_ax.grid(True, alpha=0.3)

            def _draw_prm_and_robot(target_ax):
                # High-speed LineCollection rendering for PRM graph
                if prm_nodes and prm_edges:
                    edge_segments = []
                    for node_id, edges in prm_edges.items():
                        if node_id in prm_nodes:
                            nx1, ny1 = prm_nodes[node_id]
                            if local_xmin_zoom <= nx1 <= local_xmax_zoom and local_ymin_zoom <= ny1 <= local_ymax_zoom:
                                for neighbor_id, _ in edges:
                                    if neighbor_id in prm_nodes:
                                        nx2, ny2 = prm_nodes[neighbor_id]
                                        edge_segments.append([(nx1, ny1), (nx2, ny2)])

                    if edge_segments:
                        lc = LineCollection(edge_segments, colors='gray', linewidths=0.5, alpha=0.3, zorder=2)
                        target_ax.add_collection(lc)

                    # Plot visible PRM nodes
                    visible_nodes = np.array([pos for pos in prm_nodes.values()
                                              if local_xmin_zoom <= pos[0] <= local_xmax_zoom and
                                              local_ymin_zoom <= pos[1] <= local_ymax_zoom])
                    if visible_nodes.size > 0:
                        target_ax.plot(visible_nodes[:, 0], visible_nodes[:, 1], 'k.', markersize=3, alpha=0.5, zorder=3)

                # Path
                if chosen_path and len(chosen_path) > 1:
                    path_x = [p[0] for p in chosen_path if p is not None]
                    path_y = [p[1] for p in chosen_path if p is not None]
                    target_ax.plot(path_x, path_y, color='magenta', linewidth=3.0, linestyle='-', zorder=4)
                    target_ax.plot(path_x, path_y, 'mo', markersize=6, markeredgecolor='white', zorder=5)

                # Target
                if chosen_point is not None:
                    target_ax.plot(chosen_point[0], chosen_point[1], 'g*', markersize=18, markeredgewidth=1.5, zorder=6)
                    target_ax.plot([robot_x, chosen_point[0]], [robot_y, chosen_point[1]], 'g--', linewidth=1.8, alpha=0.6, zorder=3)

                # Robot
                target_ax.plot(robot_x, robot_y, 'bo', markersize=12, zorder=7)
                for r in [1.0, 2.0]:
                    circle = patches.Circle((robot_x, robot_y), r, fill=False, linestyle=':', linewidth=1, edgecolor='blue', alpha=0.3, zorder=2)
                    target_ax.add_patch(circle)

            base_save_path, ext = os.path.splitext(save_path)

            # Colors matrix
            fused_colors = np.zeros((len(obstacle_mask), 3), dtype=np.float32)
            if terrain_real is not None:
                z_terrain = terrain_real.ravel()
                z_min, z_max = z_terrain.min(), z_terrain.max()
                z_norm = (z_terrain - z_min) / (z_max - z_min) if (z_max - z_min) > 0.001 else np.zeros_like(z_terrain)
                cmap_walkable = matplotlib.colormaps.get_cmap('YlGn')
                fused_colors[:] = cmap_walkable(z_norm)[:, :3]
            else:
                fused_colors[:] = [0.9, 0.9, 0.9]

            fused_colors[obstacle_mask.ravel() == -1] = [1.0, 0.0, 0.0]

            # ------------------------------------------------------------------ #
            # FIGURE 1: MAIN LOCAL VIEW
            # ------------------------------------------------------------------ #
            fig, ax = plt.subplots(figsize=(8, 8))
            try:
                ax.scatter(x, y, c=fused_colors, s=8, alpha=0.7, zorder=1, label='Terreno / Ostacoli')

                if env_info:
                    margin = env_info['cell_size']
                    for cell_x, cell_y, row, col, world_corners, cell_status in env_info['grid_cells']:
                        if not (local_x_min - margin <= cell_x <= local_x_max + margin and
                                local_y_min - margin <= cell_y <= local_y_max + margin):
                            continue

                        if cell_status == 1:
                            rect = patches.Polygon(world_corners, linewidth=1.5, edgecolor='darkgreen', facecolor='lightgreen', alpha=0.3, zorder=2)
                        elif cell_status == -1:
                            rect = patches.Polygon(world_corners, linewidth=1.5, edgecolor='darkred', facecolor='lightcoral', alpha=0.4, zorder=2)
                        else:
                            rect = patches.Polygon(world_corners, linewidth=1.0, edgecolor='gray', facecolor='none', alpha=0.5, linestyle='--', zorder=2)

                        ax.add_patch(rect)
                        ax.text(cell_x, cell_y, f'{row},{col}', ha='center', va='center', fontsize=7, color='black', weight='bold', zorder=3)

                if candidates:
                    if 'rejected' in candidates:
                        for point in candidates['rejected']:
                            ax.plot(point[0], point[1], 'rx', markersize=8, markeredgewidth=2, zorder=5)
                    if 'valid' in candidates:
                        for point in candidates['valid']:
                            ax.plot(point[0], point[1], 'yo', markersize=8, markerfacecolor='yellow', markeredgewidth=1.5, markeredgecolor='orange', zorder=5)

                _draw_prm_and_robot(ax)
                _apply_common_axis_settings(ax, f'Iterazione {iteration}: Path Visualization & Local Scan')
                ax.legend(loc='upper right', fontsize=8)
                plt.tight_layout()
                fig.savefig(save_path, dpi=120, bbox_inches='tight')
            finally:
                plt.close(fig)

            # ------------------------------------------------------------------ #
            # DIAGNOSTICS (Off by default during navigation)
            # ------------------------------------------------------------------ #
            if include_diagnostics:
                diagnostic_layers = {
                    'terrain': (terrain_real, 'YlGn', 'Quota (m)', 'Layer: TERRAIN'),
                    'obstacle_dist': (cells_obstacle_dist, 'plasma', 'Distanza (m)', 'Layer: OBSTACLE DISTANCE'),
                    'valid': (valid_values, 'binary', '1=Valido, 0=Cieco', 'Layer: VALID MAP'),
                    'gradient': (grad_values, 'YlOrRd', 'Gradiente / Pendenza', 'Diagnostica: PENDENZA'),
                    'roughness': (rough_values, 'coolwarm', 'Indice di Rugosità', 'Diagnostica: RUGOSITÀ')
                }

                for layer_key, (data_matrix, cmap, label_cb, title_suffix) in diagnostic_layers.items():
                    if data_matrix is not None and data_matrix.size > 0:
                        fig_diag, ax_diag = plt.subplots(figsize=(8, 8))
                        try:
                            z_data = data_matrix.flatten() if data_matrix.shape != x.shape else data_matrix
                            sc = ax_diag.scatter(x, y, c=z_data, cmap=cmap, s=6, alpha=0.6, zorder=1)
                            fig_diag.colorbar(sc, ax=ax_diag, label=label_cb, shrink=0.8)
                            _draw_prm_and_robot(ax_diag)
                            _apply_common_axis_settings(ax_diag, f'Iterazione {iteration} - {title_suffix}')
                            plt.tight_layout()
                            diag_save_path = f"{base_save_path}_{layer_key}{ext}"
                            fig_diag.savefig(diag_save_path, dpi=100, bbox_inches='tight')
                        finally:
                            plt.close(fig_diag)

            # ------------------------------------------------------------------ #
            # FIGURE 2: GLOBAL MAP
            # ------------------------------------------------------------------ #
            if env_info:
                fig2, ax2 = plt.subplots(figsize=(12, 10))
                try:
                    ax2.scatter(env_info['accum_wx'], env_info['accum_wy'], c=env_info['accum_colors'], s=2, alpha=0.6, label='Accumulated Local Scan')

                    # Vectorized global PRM edges
                    if prm_nodes and prm_edges:
                        global_segments = []
                        for node_id, edges in prm_edges.items():
                            if node_id in prm_nodes:
                                nx1, ny1 = prm_nodes[node_id]
                                for neighbor_id, _ in edges:
                                    if neighbor_id in prm_nodes:
                                        nx2, ny2 = prm_nodes[neighbor_id]
                                        global_segments.append([(nx1, ny1), (nx2, ny2)])
                        if global_segments:
                            lc_glob = LineCollection(global_segments, colors='gray', linewidths=0.5, alpha=0.3, zorder=2)
                            ax2.add_collection(lc_glob)

                        node_coords = np.array(list(prm_nodes.values()))
                        if node_coords.size > 0:
                            ax2.plot(node_coords[:, 0], node_coords[:, 1], 'k.', markersize=4, alpha=0.5, zorder=3)

                    if chosen_path and len(chosen_path) > 1:
                        path_x = [p[0] for p in chosen_path if p is not None]
                        path_y = [p[1] for p in chosen_path if p is not None]
                        ax2.plot(path_x, path_y, color='magenta', linewidth=3.5, linestyle='-', zorder=6, label='Chosen Path')

                    if env_info['traveled_arcs']:
                        segments = [[(x1, y1), (x2, y2)] for (x1, y1, x2, y2) in env_info['traveled_arcs']]
                        traveled_lc = LineCollection(segments, colors='blue', linewidths=2.0, alpha=0.6, zorder=5)
                        ax2.add_collection(traveled_lc)
                        ax2.plot([], [], color='blue', linewidth=2.0, alpha=0.6, label='Traveled Path')

                    rect_local = patches.Rectangle((local_x_min, local_y_min), local_x_max - local_x_min, local_y_max - local_y_min,
                                                   linewidth=2, edgecolor='cyan', facecolor='none', alpha=0.8, zorder=6, label='Current scan')
                    ax2.add_patch(rect_local)

                    for cell_x, cell_y, row, col, world_corners, cell_status in env_info['grid_cells']:
                        if cell_status == 1:
                            rect = patches.Polygon(world_corners, linewidth=1.5, edgecolor='darkgreen', facecolor='lightgreen', alpha=0.3, zorder=2)
                        elif cell_status == -1:
                            rect = patches.Polygon(world_corners, linewidth=1.5, edgecolor='darkred', facecolor='lightcoral', alpha=0.4, zorder=2)
                        else:
                            rect = patches.Polygon(world_corners, linewidth=1.0, edgecolor='gray', facecolor='none', alpha=0.5, linestyle='--', zorder=2)

                        ax2.add_patch(rect)
                        ax2.text(cell_x, cell_y, f'{row},{col}', ha='center', va='center', fontsize=7, color='black', weight='bold', zorder=3)

                    if chosen_point is not None:
                        ax2.plot(chosen_point[0], chosen_point[1], 'g*', markersize=20, markeredgewidth=2, label='Target', zorder=6)

                    ax2.set_xlabel('X [m] (VISION)', fontsize=12, fontweight='bold')
                    ax2.set_ylabel('Y [m] (VISION)', fontsize=12, fontweight='bold')
                    ax2.set_title(f'Iteration {iteration}: Global Map View', fontsize=13, fontweight='bold')
                    ax2.set_aspect('equal', adjustable='box')
                    ax2.grid(True, alpha=0.3)
                    ax2.legend(loc='upper right', fontsize=9)
                    plt.tight_layout()

                    global_save_path = f"{base_save_path}_global{ext}"
                    fig2.savefig(global_save_path, dpi=120, bbox_inches='tight')
                finally:
                    plt.close(fig2)

        except Exception as err:
            print(f"[VISUALIZATION ERROR] Async rendering failed: {err}")


def attempt_enter_cell_from_position(local_grid,global_grid, robot_state_client, command_client,
                                     env, target_row, target_col, global_sampler, prm_graph, mission_folder=None,
                                     iteration=0,
                                     recordingInterface=None, verification_tracker=None):

    """Attempt to enter a target cell from the current robot position."""

    print(f"\n[ATTEMPT] Trying to enter cell ({target_row},{target_col}) from current position...")

    # Recuperiamo il client nativo dall'istanza della nostra classe LocalGrid
    local_grid_client = local_grid.local_grid_client

    # 1. Richiedi TUTTI i layer necessari usando la nuova funzione
    grids_data, main_proto, all_grids_proto = local_grid.return_local_grid(
        ['obstacle_distance', 'terrain', 'terrain_valid'], #,'intensity'
        robot_state_client
    )

    if grids_data is None or 'obstacle_distance' not in grids_data:
        print("[ERROR] Rilevamento griglie locali fallito.")
        return False

    # 2. Estrai i valori dal dizionario
    pts = grids_data['obstacle_distance']['pts']
    cells_obstacle_dist = grids_data['obstacle_distance']['values']
    terrain_values = grids_data['terrain']['values']
    #intensity_values = grids_data['intensity']['values']
    valid_values = grids_data['terrain_valid']['values']

    # Estrai metadati geometrici
    num_x = main_proto.local_grid.extent.num_cells_x
    num_y = main_proto.local_grid.extent.num_cells_y
    cell_size = main_proto.local_grid.extent.cell_size

    # =========================================================================
    # --- MODIFICA GLOBAL MAP: Gestione persistente della mappa di occupazione ---
    # =========================================================================
    # Verifichiamo se l'istanza di local_grid possiede già la mappa globale
    if global_grid.global_occupancy_map is None:
        global_grid.global_occupancy_map = spotGrid.GlobalGrid(resolution=cell_size)
        print("[MAP] Inizializzata nuova mappa di occupazione globale persistente.")

    # Estraiamo il riferimento per usarlo nel resto della funzione
    global_map = global_grid.global_occupancy_map

    transforms_snapshot = main_proto.local_grid.transforms_snapshot
    vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
    robot_x, robot_y, robot_z = vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

    quat = vision_tform_body.rotation
    robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))

    footprint_mask_2d = local_grid.compute_robot_footprint_mask(
        pts, robot_x, robot_y, robot_yaw, num_x, num_y
    )

    # Calcolo derivate geometriche (Pendenza e Rugosità)
    grad_values, rough_values = local_grid.compute_gradient_and_roughness(
        terrain_values, valid_values, num_x, num_y, cell_size,
        robot_footprint_mask_2d=footprint_mask_2d
    )

    grad_for_prm = grad_values.copy()
    rough_for_prm = rough_values.copy()

    # Esegui la fusione dei layer (Genera la mappa binaria di sicurezza)
    terrain_real, obstacle_mask = local_grid.fuse_all_layers(
        pts=pts,
        cells_obstacle_dist=cells_obstacle_dist,
        terrain_values=terrain_values,
        grad_values=grad_values,
        rough_values=rough_values,
        robot_footprint_mask=(footprint_mask_2d.ravel() if footprint_mask_2d is not None else None),
        terrain_valid_values=valid_values,
        obstacle_threshold=0.15,
        slope_threshold=0.6,
        rough_threshold=0.15,
        robot_z=robot_z,
        step_threshold=0.40
    )

    #Aggiorna la mappa di occupazione globale con i dati appena processati
    global_map.update(pts, obstacle_mask)

    # Salvataggio periodico su disco della mappa globale di occupazione (.pkl)
    if mission_folder is not None:
        pkl_map_path = os.path.join(mission_folder, "global_occupancy_map.pkl")
        global_map.save_map(pkl_map_path)

    # Definizione anticipata del path di salvataggio per evitare NameError in caso di fallimento iniziale
    save_path = os.path.join(mission_folder,
                             f"iteration_{iteration}_cell_{target_row}_{target_col}.png") if mission_folder else None

    # =========================================================================
    # SALVATAGGIO DEI VALORI NUMERICI IN FORMATO NumPy (.npy)
    # =========================================================================
    if mission_folder is not None:
        try:
            # Usiamo le variabili reali della griglia di Spot
            obstacle_grid_2d = obstacle_mask.reshape((num_y, num_x))
            terrain_grid_2d = terrain_real.reshape((num_y, num_x))

            # Definiamo i percorsi dei file .npy nella cartella di questa specifica missione
            npy_obstacle_path = os.path.join(mission_folder,
                                             f"iteration_{iteration}_cell_{target_row}_{target_col}_obstacles.npy")
            npy_terrain_path = os.path.join(mission_folder,
                                            f"iteration_{iteration}_cell_{target_row}_{target_col}_terrain.npy")

            # Salvataggio dei file binari
            np.save(npy_obstacle_path, obstacle_grid_2d)
            np.save(npy_terrain_path, terrain_grid_2d)

            print(f"[DATA SAVE] ✓ Matrici .npy salvate per cella ({target_row},{target_col})")

        except Exception as e:
            print(f"[DATA SAVE] ⚠️ Impossibile salvare i file .npy: {e}")

    # ================================================================== #
    # --- [CORREZIONE PRM] AGGIORNAMENTO LOCALE DI GRADIENTE E RUGOSITÀ ---
    # ================================================================== #
    # Determiniamo i confini geometrici della scansione locale attuale nel frame VISION
    local_x_min, local_x_max = pts[:, 0].min(), pts[:, 0].max()
    local_y_min, local_y_max = pts[:, 1].min(), pts[:, 1].max()

    # Estrarre l'esatta origine della local grid rispetto al frame VISION
    grid_frame_name = main_proto.local_grid.frame_name_local_grid_data
    vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
    grid_origin_x = vision_tform_grid.position.x
    grid_origin_y = vision_tform_grid.position.y

    # Convertiamo i layer monodimensionali in matrici 2D per poterle passare al PRM
    grad_values_2d = grad_for_prm.reshape((num_y, num_x))
    rough_values_2d = rough_for_prm.reshape((num_y, num_x))

    # Passiamo le matrici complete e l'origine geometrica al PRM
    prm_graph.update_local_grid_data(
        grad_values_2d=grad_values_2d,
        rough_values_2d=rough_values_2d,
        grid_origin_x=grid_origin_x,
        grid_origin_y=grid_origin_y,
        cell_size=cell_size
    )

    # Aggiorna gradienti e rugosità SOLO per i nodi PRM che ricadono all'interno della local grid
    for node_id, (nx, ny) in prm_graph.nodes.items():
        if local_x_min <= nx <= local_x_max and local_y_min <= ny <= local_y_max:
            # Calcoliamo la distanza dall'esatto angolo inferiore sinistro della griglia locale
            dx = nx - grid_origin_x
            dy = ny - grid_origin_y

            # Trasformiamo la coordinata metrica negli indici di cella corretti
            col_idx = int(dx / cell_size)
            row_idx = int(dy / cell_size)

            if 0 <= row_idx < num_y and 0 <= col_idx < num_x:
                flat_idx = row_idx * num_x + col_idx
                if flat_idx < len(grad_values):
                    prm_graph.node_gradients[node_id] = grad_values[flat_idx]
                if flat_idx < len(rough_values):
                    prm_graph.node_roughness[node_id] = rough_values[flat_idx]  # <-- Mappa rugosità live
        else:
            # Per i nodi fuori dalla scansione attuale, mantieni i costi base a zero
            if node_id not in prm_graph.node_gradients:
                prm_graph.node_gradients[node_id] = 0.0
            if node_id not in prm_graph.node_roughness:
                prm_graph.node_roughness[node_id] = 0.0

    # --- MODIFICA PRM: Passiamo la mappa globale persistente per filtrare gli archi ostruiti storicamente ---
    prm_graph.build_graph(global_map=global_map, edge_safety_margin=0.05)

    # ================================================================== #

    # applica immediatamente gli archi bloccati al PRM appena ricostruito!
    if verification_tracker is not None:
        blocked_arcs = []
        if hasattr(verification_tracker, 'get_blocked_arcs'):
            blocked_arcs = verification_tracker.get_blocked_arcs()
        elif hasattr(verification_tracker, 'blocked_arcs'):
            with verification_tracker.path_lock:
                blocked_arcs = list(verification_tracker.blocked_arcs)

        for u, v in blocked_arcs:
            prm_graph.mark_edge_invalid(u, v)

    # Ricerca del punto target e pianificazione
    target_x, target_y, valid_samples, rejected_samples = find_best_point_in_cell(
        robot_x, robot_y, env, target_row, target_col, pts, cells_obstacle_dist, global_sampler)

    if target_x is None or target_y is None:
        print(f"[FAIL] No clear path found to cell ({target_row},{target_col}) from current position")
        visualize_grid_with_candidates(
            pts=pts,
            terrain_real=terrain_real,
            obstacle_mask=obstacle_mask,
            robot_x=robot_x,
            robot_y=robot_y,
            candidates={'rejected': rejected_samples, 'valid': valid_samples},
            chosen_point=(target_x, target_y),
            iteration=iteration,
            env=env,
            save_path=save_path,
            prm_graph=prm_graph,
            chosen_path=None,
            cells_obstacle_dist=cells_obstacle_dist,
            #intensity_values=intensity_values,
            valid_values=valid_values,
            grad_values=grad_values,
            rough_values=rough_values
        )
        return False

    # --- Cerchiamo i nodi più vicini direttamente sul PRM ---
    start_id = prm_graph.get_nearest_node(robot_x, robot_y)
    goal_id = prm_graph.get_nearest_node(target_x, target_y)
    path_ids = prm_graph.find_path_dijkstra(start_id, goal_id)

    # ABILITAZIONE DIAGNOSTICA ESPLICITA
    if path_ids is None:
        print(
            f"[DIJKSTRA CRITICAL FAIL] Impossibile trovare un percorso nel grafo PRM dal nodo di partenza {start_id} al nodo target {goal_id}!")
        print(f" -> Coordinate partenza robot: ({robot_x:.2f}, {robot_y:.2f})")
        print(f" -> Coordinate target cella: ({target_x:.2f}, {target_y:.2f})")

    full_path_coords = None
    path_waypoints = []

    if path_ids is not None:
        full_path_coords = [prm_graph.get_node_position(nid) for nid in path_ids]
        for nid in path_ids:
            nx, ny = prm_graph.get_node_position(nid)
            path_waypoints.append((nid, nx, ny))

        if len(path_waypoints) > 0:
            path_waypoints.pop(0)  # Rimuove la posizione attuale di partenza
    else:
        path_waypoints = None

    # Generazione della visualizzazione iniziale pianificata con successo
    visualize_grid_with_candidates(
        pts=pts,
        terrain_real=terrain_real,
        obstacle_mask=obstacle_mask,
        robot_x=robot_x,
        robot_y=robot_y,
        candidates={'rejected': rejected_samples, 'valid': valid_samples},
        chosen_point=(target_x, target_y),
        iteration=iteration,
        env=env,
        save_path=save_path,
        prm_graph=prm_graph,
        chosen_path=full_path_coords,
        cells_obstacle_dist=cells_obstacle_dist,
        #intensity_values=intensity_values,
        valid_values=valid_values,
        grad_values=grad_values,
        rough_values=rough_values
    )

    # --- CICLO DI NAVIGAZIONE CON VERIFICA DEGLI ARCHI MULTI-LAYER ---
    replan_counter = 0

    while path_waypoints and len(path_waypoints) > 0:
        robot_x, robot_y, robot_z, robot_quat = spotUtils.getPosition(robot_state_client)
        current_node_id = prm_graph.get_nearest_node(robot_x, robot_y)

        robot_yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                               1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))

        # 1. LETTURA AGGIORNATA DELLA SCENA LOCALE (fetch fresco, PRIMA di tutto il resto)
        grids_data_up, main_proto_up, all_grids_proto_up = local_grid.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'], #, 'intensity'
            robot_state_client
        )

        if grids_data_up is None or 'obstacle_distance' not in grids_data_up:
            print("[ERROR] Rilevamento locale fallito durante il tracking. Arresto precauzionale.")
            return False

        pts_up = grids_data_up['obstacle_distance']['pts']
        cells_obs_up = grids_data_up['obstacle_distance']['values']
        terrain_up = grids_data_up['terrain']['values']
        #intensity_up = grids_data_up['intensity']['values']
        valid_up = grids_data_up['terrain_valid']['values']

        lg_proto_up = main_proto_up
        num_x_up = lg_proto_up.local_grid.extent.num_cells_x
        num_y_up = lg_proto_up.local_grid.extent.num_cells_y
        cell_size_up = lg_proto_up.local_grid.extent.cell_size

        transforms_snapshot_up = main_proto_up.local_grid.transforms_snapshot
        grid_frame_name_up = main_proto_up.local_grid.frame_name_local_grid_data
        vision_tform_grid_up = get_a_tform_b(transforms_snapshot_up, VISION_FRAME_NAME, grid_frame_name_up)
        grid_origin_x_up = vision_tform_grid_up.position.x
        grid_origin_y_up = vision_tform_grid_up.position.y

        footprint_mask_2d = local_grid.compute_robot_footprint_mask(
            pts_up, robot_x, robot_y, robot_yaw, num_x_up, num_y_up
        )

        grad_up, rough_up = local_grid.compute_gradient_and_roughness(
            terrain_up, valid_up, num_x_up, num_y_up, cell_size_up,
            robot_footprint_mask_2d=footprint_mask_2d
        )

        terrain_real_up, obstacle_mask_updated = local_grid.fuse_all_layers(
            pts=pts_up,
            cells_obstacle_dist=cells_obs_up,
            terrain_values=terrain_up,
            grad_values=grad_up,
            rough_values=rough_up,
            robot_footprint_mask=(footprint_mask_2d.ravel() if footprint_mask_2d is not None else None),
            terrain_valid_values=valid_up,
            obstacle_threshold=0.15,
            slope_threshold=0.6,
            rough_threshold=0.15,
            robot_z=robot_z,
            step_threshold=0.40
        )

        global_map.update(pts_up, obstacle_mask_updated)

        # 2. NUOVO: iniettiamo i dati locali freschi nel PRM e ripianifichiamo dal nodo attuale
        grad_up_2d = grad_up.reshape((num_y_up, num_x_up))
        rough_up_2d = rough_up.reshape((num_y_up, num_x_up))

        prm_graph.update_local_grid_data(
            grad_values_2d=grad_up_2d,
            rough_values_2d=rough_up_2d,
            grid_origin_x=grid_origin_x_up,
            grid_origin_y=grid_origin_y_up,
            cell_size=cell_size_up
        )
        touched_nodes = prm_graph.refresh_local_edge_weights(global_map=global_map, edge_safety_margin=0.05)

        if touched_nodes:
            current_plan_ids = [current_node_id] + [w[0] for w in path_waypoints]
            chosen_path_ids = prm_graph.find_path_dijkstra(current_node_id, goal_id,
                                                           current_path_ids=current_plan_ids, margin=0.10)

            if chosen_path_ids is None:
                print(f"[REPLAN WARNING] Nessun percorso valido da {current_node_id} a {goal_id}. "
                      f"Mantengo il percorso precedente.")
            elif chosen_path_ids != current_plan_ids:
                new_full_path_coords = [prm_graph.get_node_position(nid) for nid in chosen_path_ids]
                new_path_waypoints = []
                for nid in chosen_path_ids:
                    nx, ny = prm_graph.get_node_position(nid)
                    new_path_waypoints.append((nid, nx, ny))
                if len(new_path_waypoints) > 0:
                    new_path_waypoints.pop(0)

                print(f"[REPLAN] Percorso aggiornato ({len(touched_nodes)} nodi locali rivalutati): "
                      f"{len(new_path_waypoints)} waypoint rimanenti.")
                path_waypoints = new_path_waypoints
                full_path_coords = new_full_path_coords

                replan_counter += 1
                replan_save_path = os.path.join(
                    mission_folder, f"iteration_{iteration}_cell_{target_row}_{target_col}_replan_{replan_counter}.png"
                ) if mission_folder else None

                visualize_grid_with_candidates(
                    pts=pts_up,
                    terrain_real=terrain_real_up,
                    obstacle_mask=obstacle_mask_updated,
                    robot_x=robot_x,
                    robot_y=robot_y,
                    candidates={'rejected': [], 'valid': []},
                    chosen_point=(target_x, target_y),
                    iteration=iteration,
                    env=env,
                    save_path=replan_save_path,
                    prm_graph=prm_graph,
                    chosen_path=full_path_coords,
                    cells_obstacle_dist=cells_obs_up,
                    valid_values=valid_up,
                    grad_values=grad_up,
                    rough_values=rough_up,
                    include_diagnostics=False  # lightweight: main view + global map only
                )
            else:
                print(f"[REPLAN WARNING] Nessun percorso trovato da {current_node_id} a {goal_id} "
                      f"dopo l'aggiornamento locale. Mantengo il percorso precedente.")

        if not path_waypoints:
            break  # la ripianificazione indica che siamo di fatto già al nodo obiettivo

        next_node_id, next_x, next_y = path_waypoints[0]

        # 3. AGGIORNA IL TRACKER ASINCRONO (ora riflette il percorso eventualmente aggiornato)
        tracker_payload = [(current_node_id, robot_x, robot_y)] + path_waypoints
        verification_tracker.update_path(tracker_payload)

        # 4. STAMPA LIVE DELLO STATO DEL THREAD SECONDARIO
        print("\n================ [LIVE TRACKER MONITOR] ================")
        current_tracker_path = verification_tracker.get_path_copy()
        for idx, (nid, nx, ny, status) in enumerate(current_tracker_path):
            if idx == 0:
                print(f" -> [POS ATTUALE ROBOT] Nodo ID: {nid} ({nx:.2f}, {ny:.2f})")
            else:
                status_str = "ATTESA VERIFICA ⏳" if status is None else ("LIBERO ✅" if status is True else "BLOCCATO ❌")
                print(f"    Segmento {idx}: Verso Nodo ID: {nid} ({nx:.2f}, {ny:.2f}) -> {status_str}")
        print("========================================================\n")

        # 5. CONTROLLO SICUREZZA ARCO CORRENTE
        print(f"[INFO] Controllo sicurezza arco corrente: {current_node_id} -> {next_node_id}")
        is_blocked = False

        # Se l'arco è già noto come bloccato dal tracker, ci fermiamo subito
        if verification_tracker.is_arc_blocked(current_node_id, next_node_id):
            print(f"[FAIL] L'arco corrente {current_node_id}-{next_node_id} è BLOCCATO. Interruzione percorso!")
            is_blocked = True

        # Se non è ancora verificato come libero, entriamo in un loop di attesa sicuro con timeout
        elif not verification_tracker.is_arc_verified(current_node_id, next_node_id):
            print(
                f"[INFO] L'arco {current_node_id}-{next_node_id} non è ancora verificato. Attesa elaborazione background...")

            timeout = 4.0
            start_wait = time.time()
            verified_clear = False

            while time.time() - start_wait < timeout:
                if verification_tracker.is_arc_blocked(current_node_id, next_node_id):
                    print(f"[FAIL] Il thread secondario ha rilevato l'arco come BLOCCATO durante l'attesa!")
                    is_blocked = True
                    break
                if verification_tracker.is_arc_verified(current_node_id, next_node_id):
                    verified_clear = True
                    break
                time.sleep(0.05)

            # Se scatta il timeout, eseguiamo un controllo di fallback istantaneo usando la mappa fusa aggiornata
            if not is_blocked and not verified_clear:
                print(f"[WARNING] Timeout di attesa superato. Eseguo un controllo istantaneo di fallback...")

                if arcVerification.is_arc_in_fov(robot_x, robot_y, next_x, next_y, pts_up):
                    # Usiamo la maschera fusa ad alta precisione aggiornata per il controllo di sicurezza
                    safety_status = arcVerification.verify_arc_safety(robot_x, robot_y, next_x, next_y, pts_up,
                                                                      obstacle_mask_updated)
                    if safety_status == 'blocked':
                        print(f"[FAIL] Fallback manuale: Rilevato ostacolo sull'arco. Interruzione!")
                        verification_tracker.mark_arc_blocked(current_node_id, next_node_id)
                        is_blocked = True
                    elif safety_status == 'clear':
                        print(f"[OK] Fallback manuale: L'arco è libero. Procedo.")
                        verification_tracker.mark_arc_verified(current_node_id, next_node_id)
                else:
                    # Se non è nemmeno nel FOV (es. alle spalle), procediamo con estrema cautela affidandoci al PRM globale
                    print(f"[WARNING] L'arco non è nel FOV locale delle telecamere. Procedo basandomi sul PRM globale.")

        # ---------------------------------------------------------------------
        # GESTIONE BLOCCO RILEVATO (Salvataggio dati di diagnostica dedicati)
        # ---------------------------------------------------------------------
        if is_blocked:
            if mission_folder is not None:
                try:
                    obs_array = np.asarray(obstacle_mask_updated)
                    terr_array = np.asarray(terrain_real_up)

                    obstacle_grid_2d = obs_array.reshape((num_y_up, num_x_up))
                    terrain_grid_2d = terr_array.reshape((num_y_up, num_x_up))

                    npy_obs_blocked = os.path.join(mission_folder,
                                                   f"iteration_{iteration}_cell_{target_row}_{target_col}_obstacles_BLOCKED.npy")
                    npy_terr_blocked = os.path.join(mission_folder,
                                                    f"iteration_{iteration}_cell_{target_row}_{target_col}_terrain_BLOCKED.npy")

                    np.save(npy_obs_blocked, obstacle_grid_2d)
                    np.save(npy_terr_blocked, terrain_grid_2d)
                    print(f"[DATA SAVE] ✓ Matrici di BLOCCO .npy salvate per cella ({target_row},{target_col})")
                except Exception as e:
                    print(f"[DATA SAVE] ⚠️ Impossibile salvare i file .npy di blocco: {e}")

            blocked_save_path = os.path.join(mission_folder,
                                             f"iteration_{iteration}_cell_{target_row}_{target_col}_BLOCKED.png") if mission_folder else None

            print(f"[VISUALIZATION] Generazione della mappa con gli ostacoli che hanno causato il blocco...")
            visualize_grid_with_candidates(
                pts=pts_up,
                terrain_real=terrain_real_up,
                obstacle_mask=obstacle_mask_updated,
                robot_x=robot_x,
                robot_y=robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(next_x, next_y),  # Evidenzia il nodo interrotto
                iteration=iteration,
                env=env,
                save_path=blocked_save_path,
                prm_graph=prm_graph,
                chosen_path=full_path_coords,
                cells_obstacle_dist=cells_obs_up,
                #intensity_values=intensity_up,
                valid_values=valid_up,
                grad_values=grad_up,
                rough_values=rough_up
            )
            return False

        # --- ESECUZIONE MOVIMENTO FISICO ---
        print(f"[OK] Arco {current_node_id}->{next_node_id} verificato sicuro. Eseguo movimento...")

        vision_tform_body_current = get_a_tform_b(lg_proto_up.local_grid.transforms_snapshot, VISION_FRAME_NAME,
                                                  BODY_FRAME_NAME)

        success_move = navigate_to(next_x, next_y, robot_x, robot_y, robot_state_client, command_client,
                                   vision_tform_body_current)

        if success_move:
            print(f"[INFO] Spostamento completato con successo su ({next_x:.2f}, {next_y:.2f})")

            # --- Registriamo l'arco effettivamente percorso, persistente per tutta la missione ---
            if not hasattr(env, '_traveled_arcs'):
                env._traveled_arcs = []
            env._traveled_arcs.append((robot_x, robot_y, next_x, next_y))

            path_waypoints.pop(0)  # Rimuove il waypoint appena raggiunto
        else:
            print(f"[FAIL] Comando di movimento fallito meccanicamente per ({next_x:.2f}, {next_y:.2f})")

            fail_save_path = os.path.join(mission_folder,
                                          f"iteration_{iteration}_cell_{target_row}_{target_col}_MOVE_FAIL.png") if mission_folder else None
            visualize_grid_with_candidates(
                pts=pts_up,
                terrain_real=terrain_real_up,
                obstacle_mask=obstacle_mask_updated,
                robot_x=robot_x,
                robot_y=robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(next_x, next_y),
                iteration=iteration,
                env=env,
                save_path=fail_save_path,
                prm_graph=prm_graph,
                chosen_path=full_path_coords,
                cells_obstacle_dist=cells_obs_up,
                #intensity_values=intensity_up,
                valid_values=valid_up,
                grad_values=grad_up,
                rough_values=rough_up
            )
            return False

    # Controllo finale della posizione a fine percorso
    robot_x, robot_y, _, _ = spotUtils.getPosition(robot_state_client)
    return env.is_point_in_cell(robot_x, robot_y, target_row, target_col)

def navigate_to(target_x, target_y, robot_x, robot_y, robot_state_client, command_client, vision_tform_body):
    dx, dy = target_x - robot_x, target_y - robot_y
    distance = np.sqrt(dx ** 2 + dy ** 2)
    target_yaw = np.arctan2(dy, dx)

    quat = vision_tform_body.rotation
    current_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))
    dyaw = np.arctan2(np.sin(target_yaw - current_yaw), np.cos(target_yaw - current_yaw))

    print("[INFO] Step 1: Rotating to face target...")
    movements.relative_move(0, 0, dyaw, "vision", command_client, robot_state_client)

    print(f"[INFO] Step 2: Moving forward {distance:.2f}m...")
    success_move = movements.relative_move(distance, 0, 0, "vision", command_client, robot_state_client)

    return success_move


def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells


def easy_walk(options):
    robot, lease_client, robot_state_client, client_metadata = spotLogInUtils.setLogInfo(options)
    estop = spotLogInUtils.SimpleEstop(robot, options.name + "_estop")

    local_grid = spotGrid.LocalGrid(robot)
    global_grid = spotGrid.GlobalGrid()

    recordingInterface = navGraphUtils.RecordingInterface(robot, options.download_filepath, client_metadata)
    recordingInterface.stop_recording()
    recordingInterface.clear_map()

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        #local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.logger.info('Powering on robot...')
        robot.power_on()
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')
        blocking_stand(command_client)

        print("[INIT] Initializing Velodyne client...")
        recordingInterface.clear_map()
        recordingInterface.start_recording()

        recordingInterface.initialize_with_fiducial(robot_state_client, 549)

        start_row, start_col = 0, 0
        recordingInterface.create_default_waypoint(cell_row=start_row, cell_col=start_col)

        env = environmentMap.EnvironmentMap(rows=2, cols=3, cell_size=5)

        # --- [FIX CRUCIALE] Ricaviamo la posizione di boot PRIMA di configurare ed elaborare il grafo PRM ---
        x_boot, y_boot, z_boot, quat_boot = spotUtils.getPosition(robot_state_client)
        yaw_boot = np.arctan2(2.0 * (quat_boot.w * quat_boot.z + quat_boot.x * quat_boot.y),
                              1.0 - 2.0 * (quat_boot.y ** 2 + quat_boot.z ** 2))
        env.set_origin(x_boot, y_boot, yaw_boot, start_row=start_row, start_col=start_col)

        gb_sampler = global_sampler.GlobalSampler(env, 5)
        gb_sampler.sample_global_grid()

        prm = prm_graph.PRM(min_edge_length=0.5, max_edge_length=1.5, connection_radius=3)
        prm.add_nodes_from_sampler(gb_sampler)

        # Inseriamo il punto iniziale (Boot Node) dentro la lista dei nodi permanenti prima di generare gli archi
        current_max_id = max(prm.nodes.keys(), default=-1)
        start_node_id = current_max_id + 1
        prm.add_node(start_node_id, x_boot, y_boot)

        # Inseriamo forzatamente tutti i centri geometrici delle celle come nodi del PRM
        current_max_id = start_node_id
        for r in range(env.rows):
            for c in range(env.cols):
                world_pos = env.get_world_position_from_cell(r, c)
                if world_pos is not None:
                    current_max_id += 1
                    prm.add_node(current_max_id, world_pos[0], world_pos[1])

        # Costruiamo il grafo finale ADESSO: in questo modo collegherà in automatico
        # sia i nodi del campionatore che il punto iniziale ed i centri cella!
        prm.build_graph(global_map=global_grid.global_occupancy_map, edge_safety_margin=0.05)
        verification_tracker = arcVerification.ArcVerificationTracker(robot)
        verification_tracker.start()


        mission_timestamp = datetime.now().strftime("Mission_%d-%m-%Y_%H-%M-%S")
        base_graph_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "graph")
        graph_folder = os.path.join(base_graph_folder, mission_timestamp)
        mission_map_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionMap", mission_timestamp)
        mission_log_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionLogs", mission_timestamp)
        os.makedirs(graph_folder, exist_ok=True)
        os.makedirs(mission_map_folder, exist_ok=True)
        os.makedirs(mission_log_folder, exist_ok=True)
        mission_folder = mission_map_folder
        mission_log_path = os.path.join(mission_log_folder, "mission_log.txt")
        open(mission_log_path, "a", buffering=1)

        recordingInterface.set_download_filepath(graph_folder)
        path = env.generate_serpentine_path(start_cell=env.start_cell)

        frontier = []
        visualization_counter = 0

        x, y, z, _ = spotUtils.getPosition(robot_state_client)
        robot_row, robot_col = env.get_cell_from_world(x, y)
        frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))

        while (True):
            print(frontier)
            x, y, z, _ = spotUtils.getPosition(robot_state_client)
            robot_row, robot_col = env.get_cell_from_world(x, y)

            borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
            borders_in_frontier = [b for b in borders if any((f[0] == b[0] and f[1] == b[1]) for f in frontier)]

            if len(borders_in_frontier) != 0:
                selected_border = min(borders_in_frontier, key=lambda b: b[2])
                check = attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client, env,
                                                         selected_border[0], selected_border[1], gb_sampler, prm,
                                                         mission_folder,
                                                         visualization_counter, recordingInterface, verification_tracker)
                visualization_counter += 1
                frontier.remove(selected_border)

                x_new, y_new, _, _ = spotUtils.getPosition(robot_state_client)
                robot_row, robot_col = env.get_cell_from_world(x_new, y_new)

                if check:
                    env.update_position(x_new, y_new)
                    recordingInterface.create_default_waypoint(cell_row=selected_border[0], cell_col=selected_border[1])
                    env.add_waypoint(x_new, y_new)
                    env.mark_cell_visited(selected_border[0], selected_border[1])
                    frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
                else:
                    # TODO qui se non ho trovato il percorso e devo verificare se sono dentro la cella esatta o meno
                    if (selected_border[0], selected_border[1]) == (robot_row, robot_col):
                        env.update_position(x_new, y_new)
                        recordingInterface.create_default_waypoint(cell_row=selected_border[0],
                                                                   cell_col=selected_border[1])
                        env.add_waypoint(x_new, y_new)
                        env.mark_cell_visited(selected_border[0], selected_border[1])
                        frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
                        # TODO qui sono dentro la cella ma n on sono arrivato nel punto desiderato e quindi va bene lo stesso
                    else:
                        # TODO: qui niente fallisco vuol dire nemmeno sono entrato nella cella e quindi procedo con il normale algoritmo
                        # devo però controllare che ci se ho già trovato un path allora provo a trovarne un altro che sta sotto un certo costo...

                        # 1. Sincronizziamo il PRM con gli archi bloccati rilevati dal tracker.
                        # Questo è fondamentale per far sì che Dijkstra non riproponga la stessa strada.
                        blocked_arcs = []
                        if hasattr(verification_tracker, 'get_blocked_arcs'):
                            blocked_arcs = verification_tracker.get_blocked_arcs()
                        elif hasattr(verification_tracker, 'blocked_arcs'):
                            with verification_tracker.path_lock:
                                blocked_arcs = list(verification_tracker.blocked_arcs)

                        for arc in blocked_arcs:
                            prm.mark_edge_invalid(arc[0], arc[1])

                        # 2. Ricalcoliamo la posizione attuale (gestisce sia il fallimento in partenza che a metà strada)
                        x_curr, y_curr, _, _ = spotUtils.getPosition(robot_state_client)

                        # 3. Estrapoliamo nuovamente i dati locali per trovare il target point corretto
                        grids_data_rec, main_proto_rec, _ = local_grid.return_local_grid(
                            ['obstacle_distance', 'terrain', 'terrain_valid'], robot_state_client
                        )

                        if grids_data_rec and 'obstacle_distance' in grids_data_rec:
                            # Estrazione dati nativi
                            pts = grids_data_rec['obstacle_distance']['pts']
                            cells_obstacle_dist = grids_data_rec['obstacle_distance']['values']
                            terrain_values = grids_data_rec['terrain']['values']
                            valid_values = grids_data_rec['terrain_valid']['values']

                            # Estrazione metadati geometrici dal protobuffer principale
                            num_x = main_proto_rec.local_grid.extent.num_cells_x
                            num_y = main_proto_rec.local_grid.extent.num_cells_y
                            cell_size = main_proto_rec.local_grid.extent.cell_size

                            # Calcolo dei gradienti e della rugosità
                            grad_values, rough_values = local_grid.compute_gradient_and_roughness(
                                terrain_values, valid_values, num_x, num_y, cell_size
                            )

                            # Reshape in matrici 2D
                            grad_2d = grad_values.reshape((num_y, num_x))
                            rough_2d = rough_values.reshape((num_y, num_x))

                            # Recupero dell'esatta origine geometrica della griglia nel frame VISION
                            transforms_snapshot = main_proto_rec.local_grid.transforms_snapshot
                            grid_frame_name = main_proto_rec.local_grid.frame_name_local_grid_data
                            vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
                            grid_origin_x = vision_tform_grid.position.x
                            grid_origin_y = vision_tform_grid.position.y

                            # Aggiorniamo il PRM locale
                            prm.update_local_grid_data(
                                grad_values_2d=grad_2d,
                                rough_values_2d=rough_2d,
                                grid_origin_x=grid_origin_x,
                                grid_origin_y=grid_origin_y,
                                cell_size=cell_size
                            )
                            # Ricostruiamo il grafo con i pesi aggiornati
                            prm.build_graph(global_map=global_grid.global_occupancy_map, edge_safety_margin=0.05)

                            # 3. Trova il target point ottimale all'interno della cella obiettivo
                            target_x, target_y, _, _ = find_best_point_in_cell(
                                x_curr, y_curr, env, selected_border[0], selected_border[1],
                                pts, cells_obstacle_dist, gb_sampler
                            )
                        else:
                            print(
                                "[RECOVERY ERROR] Impossibile recuperare il layer obstacle_distance per il ricalcolo.")
                            target_x, target_y = None, None

                        if target_x is not None and target_y is not None:
                            # Troviamo i nodi più vicini sul PRM aggiornato
                            start_id = prm.get_nearest_node(x_curr, y_curr)
                            goal_id = prm.get_nearest_node(target_x, target_y)

                            # 4. Ricalcoliamo il percorso
                            path_ids = prm.find_path_dijkstra(start_id, goal_id)

                            # Il costo in questo PRM corrisponde al numero di archi (ovvero numero di nodi - 1)
                            COSTO_SOGLIA = 4

                            if path_ids is not None and (len(path_ids) - 1) <= COSTO_SOGLIA:
                                print(
                                    f"[RECOVERY] Trovato percorso alternativo con costo {len(path_ids) - 1} <= {COSTO_SOGLIA}. Riprovo l'ingresso...")

                                # 5. Ritentiamo il movimento con il nuovo path calcolato
                                retry_check = attempt_enter_cell_from_position(
                                    local_grid, global_grid, robot_state_client, command_client, env,
                                    selected_border[0], selected_border[1], gb_sampler, prm,
                                    mission_folder, visualization_counter, recordingInterface, verification_tracker
                                )
                                visualization_counter += 1

                                if retry_check:
                                    # Se il retry ha successo, eseguiamo le routine di aggiornamento frontiera
                                    x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
                                    env.update_position(x_final, y_final)
                                    recordingInterface.create_default_waypoint(cell_row=selected_border[0],
                                                                               cell_col=selected_border[1])
                                    env.add_waypoint(x_final, y_final)
                                    env.mark_cell_visited(selected_border[0], selected_border[1])
                                    robot_row_final, robot_col_final = env.get_cell_from_world(x_final, y_final)
                                    frontier.extend(
                                        find_new_borders(env, robot_row_final, robot_col_final, path, frontier))
                                else:
                                    # Fallito anche il percorso alternativo
                                    print(
                                        "[FAIL] Anche il percorso alternativo ha fallito. Abbandono la cella e continuo l'algoritmo normale.")
                                    side_bit = env.get_side_bit_facing_origin(robot_row, robot_col, selected_border[0],
                                                                              selected_border[1])
                                    env.mark_cell_side_explored(selected_border[0], selected_border[1], side_bit)
                                    env.mark_cell_blocked(selected_border[0], selected_border[1])
                            else:
                                # Costo troppo alto o percorso inesistente
                                print(
                                    f"[SKIP] Nessun percorso alternativo valido o costo superiore a {COSTO_SOGLIA}. Continuo l'algoritmo normale.")
                                side_bit = env.get_side_bit_facing_origin(robot_row, robot_col, selected_border[0],
                                                                          selected_border[1])
                                env.mark_cell_side_explored(selected_border[0], selected_border[1], side_bit)
                                env.mark_cell_blocked(selected_border[0], selected_border[1])
                        else:
                            print(
                                "[SKIP] Impossibile trovare un target point valido nella cella. Continuo l'algoritmo normale.")
                            side_bit = env.get_side_bit_facing_origin(robot_row, robot_col, selected_border[0],
                                                                      selected_border[1])
                            env.mark_cell_side_explored(selected_border[0], selected_border[1], side_bit)
                            env.mark_cell_blocked(selected_border[0], selected_border[1])

            else:

                retry_candidates = env.get_blocked_neighbors_with_unexplored_side(robot_row, robot_col, path)

                if retry_candidates:

                    b_row, b_col = retry_candidates[0]

                    print(f"[RETRY-BLOCKED] Re-attempting cell ({b_row},{b_col}) from a new unexplored side")

                    check = attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client,
                                                             command_client, env,

                                                             b_row, b_col, gb_sampler, prm,

                                                             mission_folder, visualization_counter,

                                                             recordingInterface, verification_tracker)

                    visualization_counter += 1

                    x_new, y_new, _, _ = spotUtils.getPosition(robot_state_client)

                    robot_row_new, robot_col_new = env.get_cell_from_world(x_new, y_new)

                    if check:

                        env.update_position(x_new, y_new)

                        recordingInterface.create_default_waypoint(cell_row=b_row, cell_col=b_col)

                        env.add_waypoint(x_new, y_new)

                        env.mark_cell_visited(b_row, b_col)  # overwrites -1 -> 1, cell is now solved

                        frontier.extend(find_new_borders(env, robot_row_new, robot_col_new, path, frontier))

                    else:

                        side_bit = env.get_side_bit_facing_origin(robot_row, robot_col, b_row, b_col)

                        env.mark_cell_side_explored(b_row, b_col, side_bit)

                        print(f"[RETRY-BLOCKED] Cell ({b_row},{b_col}) still blocked from this side too "

                              f"(sides now {bin(env.get_cell_sides_status(b_row, b_col))})")
                                # stays -1; will only resurface again if a still-unexplored side remains
                else:
                    lowest_rank_cell = env.get_lowest_rank_from_frontier_list(frontier, path)
                    if lowest_rank_cell is not None:
                        target_row, target_col, rank = lowest_rank_cell
                        x_current, y_current, _, _ = spotUtils.getPosition(robot_state_client)

                        recordingInterface.stop_recording()
                        target_cell = (target_row, target_col)
                        waypoints_by_cell = recordingInterface.get_all_manual_waypoints_with_cells()
                        nearest_cell = recordingInterface.find_nearest_waypoint_cell_to_target(target_cell,
                                                                                               waypoints_by_cell, env)
                        nearest_wp = recordingInterface.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
                        navigation_success = recordingInterface.navigate_to_waypoint(nearest_wp['id'], robot_state_client)

                        if navigation_success:
                            recordingInterface.start_recording()
                            check = attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client,
                                                                     env, target_row, target_col, gb_sampler, prm,
                                                                     mission_folder,
                                                                     visualization_counter, recordingInterface, verification_tracker)
                            visualization_counter += 1

                            if check:
                                x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
                                env.update_position(x_final, y_final)
                                recordingInterface.create_default_waypoint(cell_row=target_row, cell_col=target_col)
                                env.add_waypoint(x_final, y_final)
                                env.mark_cell_visited(target_row, target_col)
                                frontier.remove((target_row, target_col, rank))
                                robot_row, robot_col = env.get_cell_from_world(x_final, y_final)
                                frontier.extend(find_new_borders(env, robot_row, robot_col, path, frontier))
                            else:
                                # --- FIX: mark side + block, instead of silently dropping the cell ---
                                side_bit = env.get_side_bit_facing_origin(nearest_cell[0], nearest_cell[1],
                                                                          target_row, target_col)
                                env.mark_cell_side_explored(target_row, target_col, side_bit)
                                env.mark_cell_blocked(target_row, target_col)
                                print(f"[BLOCKED] Cell ({target_row},{target_col}) marked as blocked "
                                      f"(entry attempt failed via nearest-waypoint navigation)")
                                frontier.remove((target_row, target_col, rank))
                        else:
                            # --- FIX: navigation itself failed to even reach the staging waypoint ---
                            recordingInterface.start_recording()
                            side_bit = env.get_side_bit_facing_origin(nearest_cell[0], nearest_cell[1],
                                                                      target_row, target_col)
                            env.mark_cell_side_explored(target_row, target_col, side_bit)
                            env.mark_cell_blocked(target_row, target_col)
                            print(f"[BLOCKED] Cell ({target_row},{target_col}) marked as blocked "
                                  f"(could not navigate to staging waypoint)")
                            frontier.remove((target_row, target_col, rank))
                    else:
                        if len(frontier) > 0:
                            frontier.remove(frontier[0])

            if len(frontier) == 0:
                break

        env.print_map()
        x_final, y_final, _, _ = spotUtils.getPosition(robot_state_client)
        final_row, final_col = env.get_cell_from_world(x_final, y_final)
        recordingInterface.create_default_waypoint(cell_row=final_row, cell_col=final_col)

        recordingInterface.auto_close_loops(True, False)
        recordingInterface.stop_recording()
        recordingInterface.optimize_anchoring()
        recordingInterface.navigate_to_first_waypoint(robot_state_client)

        command_client.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
        sleep(3)
        robot.power_off(cut_immediately=False)
        recordingInterface.download_full_graph()
        estop.stop()


def main():
    options = SimpleNamespace()
    options.name = "easyWalk"
    options.hostname = "192.168.80.3"
    options.verbose = False
    options.recording_user_name = ""
    options.recording_session_name = ""
    options.download_filepath = os.getcwd()

    try:
        easy_walk(options)
        return True
    except Exception as exc:
        logger = bosdyn.client.util.get_logger()
        logger.error('Hello, Spot! threw an exception: %r', exc)
        return False


if __name__ == '__main__':
    if not main():
        sys.exit(1)

