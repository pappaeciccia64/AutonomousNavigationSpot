import math
import os
import time
import numpy as np
from bosdyn.api.graph_nav.graph_nav_pb2 import TravelParams
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.api.graph_nav import graph_nav_pb2, recording_pb2, nav_pb2, map_processing_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient, map_pb2
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from google.protobuf import wrappers_pb2 as wrappers

class RecordingInterface(object):
    def __init__(self, robot, download_filepath, client_metadata):
        # Cartella base: ogni scaricamento della mappa crea una sottocartella
        self._base_download_filepath = download_filepath
        self._download_filepath = os.path.join(download_filepath, 'downloaded_graph')

        self._recording_client = robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
        self._recording_environment = GraphNavRecordingServiceClient.make_recording_environment(
            waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(client_metadata=client_metadata)
        )

        self._graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)
        self._map_processing_client = robot.ensure_client(MapProcessingServiceClient.default_service_name)
        self.robot = robot

        # Pose dei waypoint creati da noi: {nome: {'x', 'y', 'z', 'yaw', 'waypoint_id', 'cell_row', 'cell_col'}}
        self.waypoint_poses = {}

        # Copia locale del grafo, valida finche' nessuno lo modifica
        self._cached_graph = None
        self._graph_cache_valid = False

        # Nome e id dell'ultimo waypoint creato (vedi create_default_waypoint): evita di
        # riscaricare il grafo solo per rileggerli.
        self.last_created_waypoint = None

    def optimize_anchoring(self):
        """
        Ottimizza l'ancoraggio della mappa direttamente sul server del robot.
        Corregge il drift odometrico e rende il navgraph metricamente coerente,
        senza usare planimetrie esterne.
        """
        print(f"\n[{'=' * 40}]")
        print(f"[ANCHORING] Avvio ottimizzazione globale dell'ancoraggio...")

        try:
            from google.protobuf import wrappers_pb2 as wrappers
            from bosdyn.api.graph_nav import map_processing_pb2

            # Chiamiamo il servizio senza alcun "initial_hint"
            response = self._map_processing_client.process_anchoring(
                params=map_processing_pb2.ProcessAnchoringRequest.Params(
                    # Impostato a False, il robot scarta l'ancoraggio grezzo corrente
                    # e ne genera uno nuovo globale e ottimizzato da zero
                    optimize_existing_anchoring=wrappers.BoolValue(value=False)
                ),
                modify_anchoring_on_server=True,  # Applica le modifiche alla mappa in memoria
                stream_intermediate_results=False
            )

            print(f"[ANCHORING] ✓ Ottimizzazione completata!")
            print(
                f"[ANCHORING] Status: {response.status}, Iterazioni: {response.iteration}, Costo finale: {response.cost:.4f}")
            print(f"[{'=' * 40}]\n")

            # Invalida la cache per forzare il download della mappa aggiornata
            self.invalidate_graph_cache()
            return True

        except Exception as e:
            print(f"[ANCHORING] ✗ Ottimizzazione fallita: {e}")
            print(f"[{'=' * 40}]\n")
            return False

    def auto_close_loops(self, close_fiducial_loops, close_odometry_loops, *args):
        """Chiusure d'anello di Spot sul grafo (fiducial e/o odometria), senza controllo di collisione."""
        response = self._map_processing_client.process_topology(
            params=map_processing_pb2.ProcessTopologyRequest.Params(
                do_fiducial_loop_closure=wrappers.BoolValue(value=close_fiducial_loops),
                do_odometry_loop_closure=wrappers.BoolValue(value=close_odometry_loops)),
            modify_map_on_server=True)
        print(f'Created {len(response.new_subgraph.edges)} new edge(s).')


    def close_loops_checked(self, max_edge_length_m=2.0, robot_radius_m=0.35,
                            height_variation_m=0.25):
        """
        Chiusure d'anello di Spot (fiducial + odometria) DURANTE la missione, con il
        controllo di collisione di Spot sugli archi nuovi, cosi' il grafo non resta una catena
        e un ritorno non deve ripercorrere tutta la strada fatta. Restituisce quanti archi ha
        creato. Solleva in caso di errore: chi chiama decide cosa fare.
        """
        odo = map_processing_pb2.ProcessTopologyRequest.OdometryLoopClosureParams(
            max_loop_closure_edge_length=wrappers.DoubleValue(value=float(max_edge_length_m)))
        col = map_processing_pb2.ProcessTopologyRequest.CollisionCheckingParams(
            check_edges_for_collision=wrappers.BoolValue(value=True),
            collision_check_robot_radius=wrappers.DoubleValue(value=float(robot_radius_m)),
            collision_check_height_variation=wrappers.DoubleValue(value=float(height_variation_m)))
        response = self._map_processing_client.process_topology(
            params=map_processing_pb2.ProcessTopologyRequest.Params(
                do_fiducial_loop_closure=wrappers.BoolValue(value=True),
                do_odometry_loop_closure=wrappers.BoolValue(value=True),
                odometry_loop_closure_params=odo,
                collision_check_params=col),
            modify_map_on_server=True)
        self.invalidate_graph_cache()
        return len(response.new_subgraph.edges)

    def _get_graph(self, force_refresh=False):
        """Il grafo di GraphNav, dalla copia locale se valida. force_refresh lo riscarica sempre."""
        if force_refresh or not self._graph_cache_valid or self._cached_graph is None:
            print("[GRAPH_CACHE] Downloading fresh graph from robot...")
            self._cached_graph = self._graph_nav_client.download_graph()
            self._graph_cache_valid = True
        else:
            print("[GRAPH_CACHE] Using cached graph")

        return self._cached_graph

    def invalidate_graph_cache(self):
        """
        Butta la copia locale del grafo. Da chiamare dopo ogni operazione che lo modifica
        (waypoint, archi, chiusure d'anello, inizio registrazione).
        """
        self._graph_cache_valid = False
        self._cached_graph = None
        print("[GRAPH_CACHE] Cache invalidated")

    def set_download_filepath(self, filepath):
        """Cartella della missione in cui salvare le mappe scaricate (una sottocartella per volta)."""
        self._base_download_filepath = filepath
        self._download_filepath = filepath
        os.makedirs(filepath, exist_ok=True)
        print(f"[INFO] Graph download path set to: {filepath}")

    def _generate_unique_map_folder(self, base_name='Grafo'):
        """
        Percorso di una cartella nuova per lo scaricamento, del tipo
        "Grafo_10-02-2026_14-30-00" (giorno-mese-anno_ore-minuti-secondi).
        """
        from datetime import datetime

        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        unique_folder = f"{base_name}_{timestamp}"
        full_path = os.path.join(self._base_download_filepath, unique_folder)

        # Se esiste gia' (due scaricamenti nello stesso secondo), si aggiunge un contatore
        counter = 1
        original_path = full_path
        while os.path.exists(full_path):
            full_path = f"{original_path}_{counter}"
            counter += 1

        print(f"[MAP DOWNLOAD] New map will be saved to: {unique_folder}")
        return full_path

    def initialize_with_fiducial(self, robot_state_client, fiducial_id=None):
        """
        Localizza il robot sulla mappa di GraphNav usando un fiducial visibile, che diventa
        l'origine della mappa. fiducial_id: un tag preciso (es. 305); None = il piu' vicino.
        True se riesce.

        Il robot deve stare a circa 1 m dal fiducial e guardarlo; per partire da una mappa
        vuota si chiama prima clear_map().
        """
        import time

        print(f"\n[{'=' * 40}]")
        print(f"[FIDUCIAL INIT] Starting initialization via FIDUCIAL...")

        try:
            robot_state = robot_state_client.get_robot_state()
            current_odom_tform_body = get_odom_tform_body(
                robot_state.kinematic_state.transforms_snapshot).to_proto()
        except Exception as e:
            print(f"[FIDUCIAL INIT] ✗ Error reading robot state: {e}")
            return False

        if fiducial_id is not None:
            print(f"[FIDUCIAL INIT] Target: Fiducial ID {fiducial_id}")
            init_type = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_SPECIFIC
        else:
            print(f"[FIDUCIAL INIT] Target: Nearest fiducial (any ID)")
            init_type = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST

        localization = nav_pb2.Localization()

        try:
            self._graph_nav_client.set_localization(
                initial_guess_localization=localization,
                ko_tform_body=current_odom_tform_body,

                fiducial_init=init_type,
                use_fiducial_id=fiducial_id if fiducial_id else 0,

                refine_fiducial_result_with_icp=True,
                do_ambiguity_check=True
            )

            # Un attimo perche' GraphNav applichi la localizzazione
            time.sleep(0.5)
            print(f"[FIDUCIAL INIT] ✓ SUCCESS: Robot localized. Map origin is now the Fiducial.")
            print(f"[{'=' * 40}]\n")
            return True

        except Exception as e:
            print(f"[FIDUCIAL INIT] ✗ FAILED: {e}")
            print(f"[FIDUCIAL INIT] Possible reasons:")
            print(f"  1. Fiducial {fiducial_id if fiducial_id else 'any'} is not visible")
            print(f"  2. Robot is too far from fiducial (should be ~1m)")
            print(f"  3. Robot is not facing the fiducial")
            print(f"  4. Lighting conditions are poor")
            print(f"  5. Map not cleared before initialization (call clear_map() first)")
            print(f"[FIDUCIAL INIT] The system can continue without fiducial - first waypoint will be origin.")
            print(f"[{'=' * 40}]\n")
            return False

    def _get_transform(self, from_wp, to_wp):
        """Trasformazione da from_wp a to_wp, dalle loro pose nel frame dell'odometria."""

        from_se3 = from_wp.waypoint_tform_ko
        from_tf = SE3Pose(
            from_se3.position.x, from_se3.position.y, from_se3.position.z,
            Quat(w=from_se3.rotation.w, x=from_se3.rotation.x, y=from_se3.rotation.y,
                 z=from_se3.rotation.z))

        to_se3 = to_wp.waypoint_tform_ko
        to_tf = SE3Pose(
            to_se3.position.x, to_se3.position.y, to_se3.position.z,
            Quat(w=to_se3.rotation.w, x=to_se3.rotation.x, y=to_se3.rotation.y,
                 z=to_se3.rotation.z))

        from_T_to = from_tf.mult(to_tf.inverse())
        return from_T_to.to_proto()


    def should_we_start_recording(self):
        graph = self._get_graph()
        if graph is not None:
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    return False
        return True

    def get_localization_state(self):
        """
        Dove e' localizzato il robot nel grafo. Restituisce un dict:
            is_localized         True se il robot e' localizzato su un waypoint
            waypoint_id          id del waypoint (None se non localizzato)
            waypoint_name        nome del waypoint (serve il grafo: puo' scaricarlo)
            waypoint_tform_body  posa del corpo rispetto al waypoint
            localization_state   l'oggetto LocalizationState completo
            seed_tform_body      posa nel frame seed, se disponibile
        oppure None in caso di errore.
        """
        try:
            localization_state = self._graph_nav_client.get_localization_state()

            # Localizzato = ha un waypoint_id
            is_localized = bool(localization_state.localization.waypoint_id)
            waypoint_id = localization_state.localization.waypoint_id if is_localized else None

            waypoint_name = None
            if is_localized:
                graph = self._get_graph()
                for waypoint in graph.waypoints:
                    if waypoint.id == waypoint_id:
                        waypoint_name = waypoint.annotations.name if waypoint.annotations.name else 'unnamed'
                        break

            result = {
                'is_localized': is_localized,
                'waypoint_id': waypoint_id,
                'waypoint_name': waypoint_name,
                'waypoint_tform_body': localization_state.localization.waypoint_tform_body,
                'localization_state': localization_state,
                'seed_tform_body': localization_state.localization.seed_tform_body if hasattr(localization_state.localization, 'seed_tform_body') else None
            }

            if is_localized:
                print(f"\n[LOCALIZATION STATE] ✓ Robot is LOCALIZED")
                print(f"  Waypoint ID: {waypoint_id}")
                print(f"  Waypoint Name: {waypoint_name}")

                if localization_state.localization.waypoint_tform_body:
                    pos = localization_state.localization.waypoint_tform_body.position
                    print(f"  Position relative to waypoint: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            else:
                print(f"\n[LOCALIZATION STATE] ✗ Robot is NOT LOCALIZED")
                print(f"  No waypoint_id found in localization state")

            return result

        except Exception as e:
            print(f"\n[LOCALIZATION STATE] ✗ Error getting localization state: {e}")
            return None

    def create_default_waypoint(self, cell_row=None, cell_col=None):
        """
        Crea un waypoint di GraphNav nella posizione attuale, con nome progressivo (wp_0,
        wp_1, ...), e ne salva la posa e, se data, la cella in waypoint_poses.
        """
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.abspath(__file__)))
        import spotUtils
        import numpy as np

        robot_state_client = self.robot.ensure_client('robot-state')
        x, y, z, quat = spotUtils.getPosition(robot_state_client)

        yaw = np.arctan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2)
        )

        # Il numero successivo si ricava dai waypoint creati da NOI (waypoint_poses), senza
        # riscaricare il grafo: create_waypoint invalida la cache a ogni waypoint, e il grafo
        # cresce con la missione. Siamo gli unici a creare waypoint wp_N, quindi il massimo
        # che conosciamo e' il massimo che esiste. Se non ne abbiamo ancora creati si guarda
        # il grafo, che potrebbe arrivare da una sessione precedente.
        def _wp_number(name):
            try:
                return int(name.split('_')[-1])
            except (ValueError, IndexError):
                return -1

        mine = [_wp_number(n) for n in self.waypoint_poses if n.startswith('wp_')]
        if mine:
            next_number = max(mine) + 1
        else:
            graph = self._get_graph()
            if not graph or not graph.waypoints:
                next_number = 0
            else:
                max_number = -1
                for wp in graph.waypoints:
                    name = wp.annotations.name
                    if name and name.startswith('wp_'):
                        n = _wp_number(name)
                        if n >= 0:
                            max_number = max(max_number, n)
                next_number = max_number + 1

        new_name = f'wp_{next_number}'
        print(f"\n[WAYPOINT CREATION] Creating new MANUAL waypoint:")
        print(f"  Name: {new_name} (manual waypoint format)")
        print(f"  Number: {next_number}")
        print(f"  Position: ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"  Orientation (yaw): {np.degrees(yaw):.1f}°")
        if cell_row is not None and cell_col is not None:
            print(f"  Cell: ({cell_row}, {cell_col})")

        resp = self._recording_client.create_waypoint(waypoint_name=new_name)

        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            created_waypoint_id = resp.created_waypoint.id if hasattr(resp, 'created_waypoint') else 'ID_not_available'

            self.waypoint_poses[new_name] = {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'waypoint_id': created_waypoint_id,
                'cell_row': cell_row,
                'cell_col': cell_col
            }

            print(f"[WAYPOINT CREATION] ✓ Successfully created waypoint:")
            print(f"  Name: {new_name}")
            print(f"  ID: {created_waypoint_id}")
            print(f"  Pose saved for future realignment")
            if cell_row is not None and cell_col is not None:
                print(f"  Cell saved: ({cell_row}, {cell_col})")
            print(f"  Status: {resp.status}\n")

            # Nome e id dell'ultimo waypoint, dalla risposta del robot: chi li vuole non deve
            # passare da get_localization_state, che riscaricherebbe il grafo appena
            # invalidato solo per l'etichetta.
            self.last_created_waypoint = {'name': new_name, 'id': created_waypoint_id}

            self.invalidate_graph_cache()

            return resp
        else:
            print(f"[WAYPOINT CREATION] ✗ Could not create waypoint {new_name}")
            print(f"  Status: {resp.status}\n")
            return False

    def clear_map(self, *args):
        return self._graph_nav_client.clear_graph()

    def start_recording(self, *args):
        should_start_recording = self.should_we_start_recording()
        if not should_start_recording:
            print(
                'The system is not in the proper state to start recording.'
                'Try using the graph_nav_command_line to either clear the map or'
                'attempt to localize to the map.'
            )
            return
        try:
            status = self._recording_client.start_recording(recording_environment=self._recording_environment)
            self.invalidate_graph_cache()  # la registrazione aggiungera' waypoint
            print('Successfully started recording a map.')
        except Exception as err:
            print(f'Start recording failed: {err}')

    def stop_recording(self, *args):
        first_iter = True
        while True:
            try:
                status = self._recording_client.stop_recording()
                print('Successfully stopped recording a map.')
                break
            except bosdyn.client.recording.NotReadyYetError as err:
                if first_iter:
                    print('Cleaning up recording...')
                first_iter = False
                time.sleep(0.5)
                continue
            except Exception as err:
                print(f'Stop recording failed: {err}')
                break


    def download_full_graph(self, *args):
        """
        Scarica il grafo completo, con le istantanee di waypoint e archi, in una cartella
        nuova (vedi _generate_unique_map_folder). Restituisce il percorso, None se fallisce.
        """
        graph = self._get_graph(force_refresh=True)
        if graph is None:
            print('[MAP DOWNLOAD] ✗ Failed to download the graph.')
            return None

        unique_path = self._generate_unique_map_folder()

        # I _download_and_write_* scrivono in self._download_filepath: lo si cambia per ora
        old_path = self._download_filepath
        self._download_filepath = unique_path

        try:
            print(f'\n[MAP DOWNLOAD] Starting download...')
            print(f'[MAP DOWNLOAD] Destination: {os.path.basename(unique_path)}')

            self._write_full_graph(graph)
            print(f'[MAP DOWNLOAD] Graph downloaded with {len(graph.waypoints)} waypoints and {len(graph.edges)} edges')

            self._download_and_write_waypoint_snapshots(graph.waypoints)
            self._download_and_write_edge_snapshots(graph.edges)

            print(f'[MAP DOWNLOAD] ✓ Map successfully saved to: {unique_path}')
            return unique_path

        finally:
            self._download_filepath = old_path

    def _write_full_graph(self, graph):
        graph_bytes = graph.SerializeToString()
        self._write_bytes(self._download_filepath, 'graph', graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints):
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(waypoint.snapshot_id)
            except Exception:
                print(f'Failed to download waypoint snapshot: {waypoint.snapshot_id}')
                continue
            self._write_bytes(
                os.path.join(self._download_filepath, 'waypoint_snapshots'),
                str(waypoint.snapshot_id),
                waypoint_snapshot.SerializeToString(),
            )
            num_waypoint_snapshots_downloaded += 1
            print(
                f'Downloaded {num_waypoint_snapshots_downloaded} of the total {len(waypoints)} waypoint snapshots.'
            )

    def _download_and_write_edge_snapshots(self, edges):
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                print(f'Failed to download edge snapshot: {edge.snapshot_id}')
                continue
            self._write_bytes(
                os.path.join(self._download_filepath, 'edge_snapshots'),
                str(edge.snapshot_id),
                edge_snapshot.SerializeToString(),
            )
            num_edge_snapshots_downloaded += 1
            print(
                f'Downloaded {num_edge_snapshots_downloaded} of the total {num_to_download} edge snapshots.'
            )

    def _write_bytes(self, filepath, filename, data):
        os.makedirs(filepath, exist_ok=True)
        with open(os.path.join(filepath, filename), 'wb+') as f:
            f.write(data)
            f.close()

    def get_all_manual_waypoints_with_cells(self):
        """
        I waypoint creati da noi (wp_N) che hanno una cella: {(riga, colonna): dati}, con
        dati = name, id, x, y, z, yaw, cell_row, cell_col.
        """
        waypoints_by_cell = {}

        for wp_name, wp_data in self.waypoint_poses.items():
            if not (wp_name.startswith('wp_') and len(wp_name.split('_')) == 2):
                continue

            if wp_data.get('cell_row') is not None and wp_data.get('cell_col') is not None:
                cell_key = (wp_data['cell_row'], wp_data['cell_col'])
                waypoints_by_cell[cell_key] = {
                    'name': wp_name,
                    'id': wp_data['waypoint_id'],
                    'x': wp_data['x'],
                    'y': wp_data['y'],
                    'z': wp_data['z'],
                    'yaw': wp_data['yaw'],
                    'cell_row': wp_data['cell_row'],
                    'cell_col': wp_data['cell_col']
                }

        print(f"[WAYPOINTS] Found {len(waypoints_by_cell)} manual waypoints with cell data")
        return waypoints_by_cell

    def get_manual_waypoint_by_cell(self, cell_row, cell_col):
        """I dati del waypoint creato nella cella, None se non c'e'."""
        waypoints_by_cell = self.get_all_manual_waypoints_with_cells()
        return waypoints_by_cell.get((cell_row, cell_col))

    def create_edge_between_waypoint_objs(self, from_wp, to_wp):
        """
        Crea un arco fra due waypoint gia' letti dal grafo (oggetti map_pb2.Waypoint), senza
        riscaricare il grafo ne' stampare la trasformazione.  Chi chiama invalida la cache alla fine
        (invalidate_graph_cache). Solleva in caso di errore.
        """
        new_edge = map_pb2.Edge()
        new_edge.id.from_waypoint = from_wp.id
        new_edge.id.to_waypoint = to_wp.id
        new_edge.from_tform_to.CopyFrom(self._get_transform(from_wp, to_wp))
        self._recording_client.create_edge(edge=new_edge)

    def find_nearest_waypoint_cell_to_target(self, target_cell, waypoints_by_cell, env_map):
        """
        La cella visitata con un waypoint piu' vicina a target_cell: prima una adiacente,
        altrimenti la piu' vicina in distanza di Manhattan. None se non ce ne sono.
        waypoints_by_cell come da get_all_manual_waypoints_with_cells.
        """
        target_row, target_col = target_cell

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        adjacent_with_waypoint = []
        for dr, dc in directions:
            adj_row, adj_col = target_row + dr, target_col + dc
            adj_cell = (adj_row, adj_col)

            if (0 <= adj_row < env_map.rows and
                0 <= adj_col < env_map.cols and
                adj_cell in waypoints_by_cell and
                env_map.map[adj_row][adj_col] == 1):  # visitata
                adjacent_with_waypoint.append(adj_cell)

        if adjacent_with_waypoint:
            print(f"[NEAREST_WP] Found adjacent waypoint at {adjacent_with_waypoint[0]}")
            return adjacent_with_waypoint[0]

        # Nessuna adiacente: la piu' vicina fra tutte
        min_distance = float('inf')
        nearest_cell = None

        for cell in waypoints_by_cell.keys():
            cell_row, cell_col = cell

            if env_map.map[cell_row][cell_col] != 1:
                continue

            distance = abs(cell_row - target_row) + abs(cell_col - target_col)

            if distance < min_distance:
                min_distance = distance
                nearest_cell = cell

        if nearest_cell:
            print(f"[NEAREST_WP] Nearest waypoint at {nearest_cell} (distance: {min_distance})")

        return nearest_cell
