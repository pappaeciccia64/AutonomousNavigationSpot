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
from bosdyn.api.graph_nav import graph_nav_pb2 as _graph_nav_pb2

import movements
import parametri as P
import spotUtils
from registro import MISSION_CTX, MISSION_STATS, TrajectoryLog, append_csv, timing_add

class LoopClosureUploadError(Exception):
    """Spot ha calcolato gli archi della chiusura d'anello ma il loro caricamento e' fallito."""

    def __init__(self, n_edges, cause):
        super().__init__(f"{n_edges} archi calcolati ma non caricati nella mappa "
                         f"({type(cause).__name__}: {cause})")
        self.n_edges = n_edges


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
        print(f'[ANELLI] Creati {len(response.new_subgraph.edges)} archi nuovi.')


    def close_loops_checked(self, max_edge_length_m=2.0, robot_radius_m=0.35,
                            height_variation_m=0.25, timeout_s=None, max_path_m=None,
                            min_path_m=None, modify_map_on_server=True, check_collision=True,
                            icp_iters=None, feature_matching=None, upload_result=False,
                            prune_edges=None):
        """
        Chiusure d'anello di Spot (fiducial + odometria) DURANTE la missione, con il
        controllo di collisione di Spot sugli archi nuovi, cosi' il grafo non resta una catena
        e un ritorno non deve ripercorrere tutta la strada fatta.

        timeout_s: tempo massimo di elaborazione LATO SPOT (Params.timeout_seconds). Allo
        scadere Spot restituisce gli archi trovati fino a li' (timed_out = True) e una
        chiamata successiva ricomincia (prova al banco del 26-10: non riprende il lavoro).
        Senza, Spot si ferma da solo dopo ~30 s, insieme alla RPC: il 22-10 la RPC scadeva
        prima e non arrivava nessun risultato. Il limite di Spot e' approssimativo e dopo ogni
        modifica della mappa la chiamata si allunga (4 s richiesti -> fino a 12 s; 60 s -> oltre
        120 s una volta): il limite della RPC qui e' il triplo di timeout_s, almeno + 15 s.
        max_path_m / min_path_m: limiti sulla distanza fra i due waypoint lungo il grafo
        (max/min_loop_closure_path_length); None = valori di Spot.
        modify_map_on_server, check_collision: False solo per le misure al banco
        (prova_anelli.py): la mappa resta com'era e si confrontano impostazioni diverse.
        icp_iters (0 = niente ICP) e feature_matching: senza, Spot finisce in pochi secondi ma
        gli archi non sono corretti sulle scansioni (vedi easy_walk.GRAPHNAV_SDK_LOOP_FAST);
        None = valori di Spot.
        upload_result: Spot calcola SENZA scrivere nella mappa e gli archi li carica questa
        funzione (upload_graph, che li aggiunge alla mappa in un colpo solo). Scrivendo da se',
        Spot puo' rispondere MapModifiedError ALLA FINE dell'elaborazione, se la mappa e'
        cambiata nel frattempo, e l'elaborazione va persa: il 26-10 15:36, appena fermata la
        registrazione, ~73 s di tentativi falliti prima di quello buono.
        prune_edges: False = tiene anche gli archi candidati in zone gia' collegate (Spot di
        default ne tiene uno solo per zona, il "migliore"); None = valore di Spot.

        Restituisce (archi creati, timed_out). Solleva in caso di errore: chi chiama decide
        cosa fare.
        """
        odo = map_processing_pb2.ProcessTopologyRequest.OdometryLoopClosureParams(
            max_loop_closure_edge_length=wrappers.DoubleValue(value=float(max_edge_length_m)))
        if prune_edges is not None:
            odo.prune_edges.value = bool(prune_edges)
        if max_path_m is not None:
            odo.max_loop_closure_path_length.value = float(max_path_m)
        if min_path_m is not None:
            odo.min_loop_closure_path_length.value = float(min_path_m)
        col = map_processing_pb2.ProcessTopologyRequest.CollisionCheckingParams(
            check_edges_for_collision=wrappers.BoolValue(value=bool(check_collision)),
            collision_check_robot_radius=wrappers.DoubleValue(value=float(robot_radius_m)),
            collision_check_height_variation=wrappers.DoubleValue(value=float(height_variation_m)))
        params = map_processing_pb2.ProcessTopologyRequest.Params(
            do_fiducial_loop_closure=wrappers.BoolValue(value=True),
            do_odometry_loop_closure=wrappers.BoolValue(value=True),
            odometry_loop_closure_params=odo,
            collision_check_params=col)
        if icp_iters is not None:
            params.icp_params.icp_iters.value = int(icp_iters)
        if feature_matching is not None:
            params.feature_matching_params.do_feature_matching.value = bool(feature_matching)
        kwargs = {}
        if timeout_s is not None:
            params.timeout_seconds = float(timeout_s)
            kwargs['timeout'] = max(3.0 * float(timeout_s), float(timeout_s) + 15.0)
        write_on_server = bool(modify_map_on_server) and not upload_result
        response = self._map_processing_client.process_topology(
            params=params, modify_map_on_server=write_on_server, **kwargs)
        n = len(response.new_subgraph.edges)
        if upload_result and n:
            try:
                self._graph_nav_client.upload_graph(graph=response.new_subgraph,
                                                    generate_new_anchoring=False)
            except Exception as e:
                raise LoopClosureUploadError(n, e) from e
        self.invalidate_graph_cache()
        return n, bool(getattr(response, 'timed_out', False))

    def _get_graph(self, force_refresh=False):
        """Il grafo di GraphNav, dalla copia locale se valida. force_refresh lo riscarica sempre."""
        if force_refresh or not self._graph_cache_valid or self._cached_graph is None:
            print("[GRAPH_CACHE] Scarico il grafo aggiornato dal robot...")
            self._cached_graph = self._graph_nav_client.download_graph()
            self._graph_cache_valid = True
        else:
            print("[GRAPH_CACHE] Uso il grafo in memoria")

        return self._cached_graph

    def invalidate_graph_cache(self):
        """
        Butta la copia locale del grafo. Da chiamare dopo ogni operazione che lo modifica
        (waypoint, archi, chiusure d'anello, inizio registrazione).
        """
        self._graph_cache_valid = False
        self._cached_graph = None
        print("[GRAPH_CACHE] Grafo in memoria da riscaricare")

    def set_download_filepath(self, filepath):
        """Cartella della missione in cui salvare le mappe scaricate (una sottocartella per volta)."""
        self._base_download_filepath = filepath
        self._download_filepath = filepath
        os.makedirs(filepath, exist_ok=True)
        print(f"[INFO] Cartella per scaricare il grafo: {filepath}")

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

        print(f"[MAP DOWNLOAD] La mappa nuova andra' in: {unique_folder}")
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
        print(f"[FIDUCIAL INIT] Inizializzazione con il FIDUCIAL...")

        try:
            robot_state = robot_state_client.get_robot_state()
            current_odom_tform_body = get_odom_tform_body(
                robot_state.kinematic_state.transforms_snapshot).to_proto()
        except Exception as e:
            print(f"[FIDUCIAL INIT] ✗ Stato del robot non leggibile: {e}")
            return False

        if fiducial_id is not None:
            print(f"[FIDUCIAL INIT] Fiducial cercato: ID {fiducial_id}")
            init_type = graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_SPECIFIC
        else:
            print(f"[FIDUCIAL INIT] Fiducial cercato: il piu' vicino (qualunque ID)")
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
            print(f"[FIDUCIAL INIT] ✓ RIUSCITO: robot localizzato. L'origine della mappa e' il fiducial.")
            print(f"[{'=' * 40}]\n")
            return True

        except Exception as e:
            print(f"[FIDUCIAL INIT] ✗ NON RIUSCITO: {e}")
            print(f"[FIDUCIAL INIT] Possibili cause:")
            print(f"  1. Il fiducial {fiducial_id if fiducial_id else '(qualunque)'} non e' visibile")
            print(f"  2. Il robot e' troppo lontano dal fiducial (deve stare a ~1 m)")
            print(f"  3. Il robot non guarda il fiducial")
            print(f"  4. Luce insufficiente")
            print(f"  5. Mappa non svuotata prima dell'inizializzazione (chiamare prima clear_map())")
            print(f"[FIDUCIAL INIT] Si prosegue senza fiducial: l'origine sara' il primo waypoint.")
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
                print(f"\n[LOCALIZATION STATE] ✓ Robot LOCALIZZATO")
                print(f"  Waypoint ID: {waypoint_id}")
                print(f"  Nome del waypoint: {waypoint_name}")

                if localization_state.localization.waypoint_tform_body:
                    pos = localization_state.localization.waypoint_tform_body.position
                    print(f"  Posizione rispetto al waypoint: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            else:
                print(f"\n[LOCALIZATION STATE] ✗ Robot NON LOCALIZZATO")
                print(f"  Nessun waypoint_id nello stato di localizzazione")

            return result

        except Exception as e:
            print(f"\n[LOCALIZATION STATE] ✗ Stato di localizzazione non leggibile: {e}")
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
        print(f"\n[WAYPOINT CREATION] Creo un nuovo waypoint MANUALE:")
        print(f"  Nome: {new_name} (formato dei waypoint manuali)")
        print(f"  Numero: {next_number}")
        print(f"  Posizione: ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"  Orientamento (imbardata): {np.degrees(yaw):.1f}°")
        if cell_row is not None and cell_col is not None:
            print(f"  Cella: ({cell_row}, {cell_col})")

        resp = self._recording_client.create_waypoint(waypoint_name=new_name)

        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            created_waypoint_id = resp.created_waypoint.id if hasattr(resp, 'created_waypoint') else 'ID_not_available'

            self.waypoint_poses[new_name] = {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                't': time.time(),        # per la misura della deriva (easy_walk.graphnav_log_drift)
                'waypoint_id': created_waypoint_id,
                'cell_row': cell_row,
                'cell_col': cell_col
            }

            print(f"[WAYPOINT CREATION] ✓ Waypoint creato:")
            print(f"  Nome: {new_name}")
            print(f"  ID: {created_waypoint_id}")
            print(f"  Posa salvata per i riallineamenti")
            if cell_row is not None and cell_col is not None:
                print(f"  Cella salvata: ({cell_row}, {cell_col})")
            print(f"  Stato: {resp.status}\n")

            # Nome e id dell'ultimo waypoint, dalla risposta del robot: chi li vuole non deve
            # passare da get_localization_state, che riscaricherebbe il grafo appena
            # invalidato solo per l'etichetta.
            self.last_created_waypoint = {'name': new_name, 'id': created_waypoint_id}

            self.invalidate_graph_cache()

            return resp
        else:
            print(f"[WAYPOINT CREATION] ✗ Waypoint {new_name} non creato")
            print(f"  Stato: {resp.status}\n")
            return False

    def clear_map(self, *args):
        return self._graph_nav_client.clear_graph()

    def start_recording(self, *args):
        should_start_recording = self.should_we_start_recording()
        if not should_start_recording:
            print(
                'GraphNav non e\' nello stato giusto per registrare: '
                'svuotare la mappa (clear_map) o localizzare il robot sulla mappa '
                '(per esempio con graph_nav_command_line).'
            )
            return
        try:
            status = self._recording_client.start_recording(recording_environment=self._recording_environment)
            self.invalidate_graph_cache()  # la registrazione aggiungera' waypoint
            print('[REGISTRAZIONE] Registrazione della mappa avviata.')
        except Exception as err:
            print(f'[REGISTRAZIONE] Avvio della registrazione non riuscito: {err}')

    def stop_recording(self, *args):
        first_iter = True
        while True:
            try:
                status = self._recording_client.stop_recording()
                print('[REGISTRAZIONE] Registrazione della mappa fermata.')
                break
            except bosdyn.client.recording.NotReadyYetError as err:
                if first_iter:
                    print('[REGISTRAZIONE] Attendo che la mappa finisca di elaborare...')
                first_iter = False
                time.sleep(0.5)
                continue
            except Exception as err:
                print(f'[REGISTRAZIONE] Arresto della registrazione non riuscito: {err}')
                break


    def download_full_graph(self, *args):
        """
        Scarica il grafo completo, con le istantanee di waypoint e archi, in una cartella
        nuova (vedi _generate_unique_map_folder). Restituisce il percorso, None se fallisce.
        """
        graph = self._get_graph(force_refresh=True)
        if graph is None:
            print('[MAP DOWNLOAD] ✗ Grafo non scaricato.')
            return None

        unique_path = self._generate_unique_map_folder()

        # I _download_and_write_* scrivono in self._download_filepath: lo si cambia per ora
        old_path = self._download_filepath
        self._download_filepath = unique_path

        try:
            print(f'\n[MAP DOWNLOAD] Scaricamento in corso...')
            print(f'[MAP DOWNLOAD] Destinazione: {os.path.basename(unique_path)}')

            self._write_full_graph(graph)
            print(f'[MAP DOWNLOAD] Grafo scaricato: {len(graph.waypoints)} waypoint e {len(graph.edges)} archi')

            self._download_and_write_waypoint_snapshots(graph.waypoints)
            self._download_and_write_edge_snapshots(graph.edges)

            print(f'[MAP DOWNLOAD] ✓ Mappa salvata in: {unique_path}')
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
                print(f'[MAP DOWNLOAD] Istantanea del waypoint non scaricata: {waypoint.snapshot_id}')
                continue
            self._write_bytes(
                os.path.join(self._download_filepath, 'waypoint_snapshots'),
                str(waypoint.snapshot_id),
                waypoint_snapshot.SerializeToString(),
            )
            num_waypoint_snapshots_downloaded += 1
            print(
                f'[MAP DOWNLOAD] Istantanee dei waypoint: {num_waypoint_snapshots_downloaded} su {len(waypoints)}.'
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
                print(f'[MAP DOWNLOAD] Istantanea dell\'arco non scaricata: {edge.snapshot_id}')
                continue
            self._write_bytes(
                os.path.join(self._download_filepath, 'edge_snapshots'),
                str(edge.snapshot_id),
                edge_snapshot.SerializeToString(),
            )
            num_edge_snapshots_downloaded += 1
            print(
                f'[MAP DOWNLOAD] Istantanee degli archi: {num_edge_snapshots_downloaded} su {num_to_download}.'
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

        print(f"[WAYPOINTS] {len(waypoints_by_cell)} waypoint manuali con la cella")
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
            print(f"[NEAREST_WP] Waypoint in una cella confinante: {adjacent_with_waypoint[0]}")
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
            print(f"[NEAREST_WP] Waypoint piu' vicino nella cella {nearest_cell} (distanza: {min_distance})")

        return nearest_cell


# =========================================================================================
# GRAPHNAV DURANTE LA MISSIONE: spostamenti fra celle, ritorni, ritorno a wp_0; scorciatoie
# e chiusura d'anello; localizzazione, riaggancio e deriva; briciole.
# Usa RecordingInterface (sopra) per parlare con GraphNav.
# =========================================================================================
_GN = _graph_nav_pb2.NavigationFeedbackResponse

_GRAPHNAV_FAIL_STATUSES = {
    _GN.STATUS_STUCK: "bloccato (STUCK)", _GN.STATUS_NO_ROUTE: "nessun percorso nel grafo",
    _GN.STATUS_NO_LOCALIZATION: "non localizzato", _GN.STATUS_ROBOT_IMPAIRED: "robot in difficolta' (caduto?)",
    _GN.STATUS_CONSTRAINT_FAULT: "vincolo violato", _GN.STATUS_COMMAND_OVERRIDDEN: "comando sostituito",
    _GN.STATUS_NOT_LOCALIZED_TO_ROUTE: "non localizzato sul percorso", _GN.STATUS_LEASE_ERROR: "errore di lease",
    _GN.STATUS_AREA_CALLBACK_ERROR: "errore di area callback",
}

_DRIFT_COLUMNS = ['time', 'where', 'waypoint', 'age_s', 'dx', 'dy', 'd', 'gn_x', 'gn_y', 'vision_x',
                  'vision_y']


def graphnav_localization(recordingInterface):
    """Stato del localizzatore di GraphNav come dict, o None se non leggibile. Non scarica il grafo."""
    try:
        st = recordingInterface._graph_nav_client.get_localization_state()
    except Exception as e:
        return {'error': str(e)}
    loc = st.localization
    out = {'wp_id': loc.waypoint_id or None, 'is_lost': None}
    try:
        has_ld = st.HasField('lost_detector_state')
    except Exception:
        has_ld = False      # SDK senza rilevatore di perdita
    if has_ld:
        ld = st.lost_detector_state
        out.update(is_lost=bool(ld.is_lost),
                   accepted=int(ld.total_num_accepted_localizations),
                   rejected=int(ld.total_num_rejected_localizations),
                   rej_since=int(ld.num_rejected_localizations_since_accepted),
                   dist_rej=float(ld.distance_traveled_with_rejected_localization),
                   bad_edges=int(ld.num_consecutive_bad_edges))
    return out


def graphnav_log_drift(recordingInterface, robot_state_client, where):
    """
    DERIVA DELL'ODOMETRIA, misurata a ogni arrivo GraphNav. GraphNav si localizza sulle
    scansioni del waypoint e dice dove sta il corpo rispetto a lui (waypoint_tform_body);
    componendolo con la posa VISION che il waypoint aveva quando l'abbiamo creato si ottiene
    dove il robot DOVREBBE essere in VISION senza deriva. La differenza con la posa VISION di
    adesso e' la deriva accumulata dalla creazione del waypoint: la stessa che sposta, nelle
    nostre mappe globali (costruite in VISION), cio' che si e' visto molti minuti prima. Il
    26-10 15:11, all'arrivo su wp_0, lo scarto grezzo era 0.66 m (tolleranza d'arrivo inclusa).

    Solo misura: niente si corregge. Misurabile solo sui waypoint creati da noi (di cella e
    briciole), di cui si conosce la posa VISION. Mai eccezioni verso chi chiama.
    """
    try:
        loc = recordingInterface._graph_nav_client.get_localization_state().localization
        wid = loc.waypoint_id
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        hit = next(((n, p) for n, p in poses.items() if p.get('waypoint_id') == wid), None)
        if not wid or hit is None:
            print(f"[DERIVA] {where}: localizzato su un waypoint automatico di Spot (o non localizzato): "
                  f"deriva non misurabile qui.")
            return None
        name, wp = hit
        b = loc.waypoint_tform_body.position
        c, sn = np.cos(wp['yaw']), np.sin(wp['yaw'])
        gx = wp['x'] + c * b.x - sn * b.y
        gy = wp['y'] + sn * b.x + c * b.y
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
        dx, dy = rx - gx, ry - gy
        d = float(np.hypot(dx, dy))
        age = time.time() - wp['t'] if wp.get('t') else float('nan')
        MISSION_STATS['drift_samples'] += 1
        MISSION_STATS['drift_max_m'] = max(MISSION_STATS['drift_max_m'], d)
        MISSION_STATS['drift_last_m'] = d
        print(f"[DERIVA] {where}: localizzato su {name}, creato {age / 60:.1f} min fa -> l'odometria "
              f"(VISION) e' scostata di {d:.2f} m (dx {dx:+.2f}, dy {dy:+.2f}) da dove GraphNav "
              f"mette il robot.")
        append_csv("drift.csv", _DRIFT_COLUMNS,
                    [[time.time(), where, name, round(age, 1), round(dx, 4), round(dy, 4), round(d, 4),
                      round(gx, 4), round(gy, 4), round(rx, 4), round(ry, 4)]])
        return d
    except Exception as e:
        print(f"[DERIVA] {where}: misura non riuscita ({type(e).__name__}: {e}).")
        return None


def graphnav_is_lost(loc):
    return bool(loc) and (loc.get('is_lost') is True or ('error' not in loc and not loc.get('wp_id')))


def wp_name_by_id(recordingInterface, wp_id):
    for name, p in (getattr(recordingInterface, 'waypoint_poses', None) or {}).items():
        if p.get('waypoint_id') == wp_id:
            return name
    return (wp_id or '?')[:12]


def graphnav_log_localization(env, recordingInterface, robot_state_client, where, force=False):
    """
    Riga [LOCALIZZAZIONE]: stampata solo se lo stato cambia (perso si'/no, localizzazioni
    rifiutate dall'ultima accettata, archi cattivi di fila) o con force=True. Riporta anche
    il tier del passo appena fatto e il beccheggio del corpo adesso, per vedere se la perdita
    scatta in crawl o in salita. Non solleva mai.
    """
    if recordingInterface is None:
        return None
    loc = graphnav_localization(recordingInterface)
    if loc is None:
        return None
    if 'error' in loc:
        print(f"[LOCALIZZAZIONE] {where}: stato non leggibile ({loc['error']}).")
        return loc
    key = (loc.get('is_lost'), bool(loc.get('wp_id')), (loc.get('rej_since') or 0) > 0,
           (loc.get('bad_edges') or 0) > 0)
    prev = getattr(env, '_loc_key', None) if env is not None else None
    if env is not None:
        env._loc_key = key
    if not force and key == prev:
        return loc
    pitch = float('nan')
    try:
        q = spotUtils.getPosition(robot_state_client)[3]
        pitch = float(np.degrees(np.arcsin(np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))))
    except Exception:
        pass
    if loc.get('is_lost') is None:
        det = "rilevatore di perdita non disponibile"
    else:
        det = (f"localizzazioni accettate {loc['accepted']}, rifiutate {loc['rejected']} "
               f"({loc['rej_since']} dall'ultima accettata, {loc['dist_rej']:.1f} m fatti cosi'), "
               f"archi cattivi di fila {loc['bad_edges']}")
    stato = ("PERSO" if loc.get('is_lost') else
             (f"localizzato su {wp_name_by_id(recordingInterface, loc['wp_id'])}" if loc.get('wp_id')
              else "NON localizzato"))
    print(f"[LOCALIZZAZIONE] {where}: {stato} | {det} | ultimo passo {MISSION_STATS.get('last_tier')}, "
          f"beccheggio {pitch:.1f} gradi.")
    return loc


def graphnav_relocalize(recordingInterface, robot_state_client, why=""):
    """
    Riaggancia GraphNav sul waypoint piu' vicino (secondo le nostre pose in visione). La
    stima iniziale e' waypoint_tform_ko (dal grafo) * ko_tform_body (adesso): la stessa
    odometria con cui Spot ha registrato il waypoint. True se Spot accetta.
    """
    try:
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
        poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
        cands = sorted(((float(np.hypot(p['x'] - rx, p['y'] - ry)), name, p['waypoint_id'])
                        for name, p in poses.items() if p.get('waypoint_id')), key=lambda c: c[0])
        if not cands:
            print(f"[LOCALIZZAZIONE] Riaggancio {why}: nessun waypoint nostro noto.")
            return False
        d_wp, name, wp_id = cands[0]
        if d_wp > P.GRAPHNAV_RELOC_MAX_WP_DIST_M:
            print(f"[LOCALIZZAZIONE] Riaggancio {why}: il waypoint piu' vicino ({name}) e' a "
                  f"{d_wp:.1f} m, troppo lontano.")
            return False
        graph = recordingInterface._get_graph()
        wp = next((w for w in graph.waypoints if w.id == wp_id), None) if graph else None
        if wp is None:
            graph = recordingInterface._get_graph(force_refresh=True)
            wp = next((w for w in graph.waypoints if w.id == wp_id), None) if graph else None
        if wp is None:
            print(f"[LOCALIZZAZIONE] Riaggancio {why}: {name} non e' nel grafo.")
            return False
        from bosdyn.client.math_helpers import SE3Pose
        from bosdyn.api.graph_nav import nav_pb2 as _nav_pb2
        state = robot_state_client.get_robot_state()
        odom_tform_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
        guess = SE3Pose.from_proto(wp.waypoint_tform_ko) * odom_tform_body
        loc = _nav_pb2.Localization()
        loc.waypoint_id = wp.id
        loc.waypoint_tform_body.CopyFrom(guess.to_proto())
        recordingInterface._graph_nav_client.set_localization(
            initial_guess_localization=loc,
            ko_tform_body=odom_tform_body.to_proto(),
            max_distance=P.GRAPHNAV_RELOC_MAX_DIST_M,
            max_yaw=P.GRAPHNAV_RELOC_MAX_YAW_RAD,
            fiducial_init=_graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL)
        MISSION_STATS['graphnav_relocalizations'] = MISSION_STATS.get('graphnav_relocalizations', 0) + 1
        print(f"[LOCALIZZAZIONE] Riaggancio {why}: riuscito su {name} (a {d_wp:.2f} m secondo le "
              f"nostre pose; stima dall'odometria, rifinita dall'ICP).")
        return True
    except Exception as e:
        MISSION_STATS['graphnav_relocalization_fail'] = MISSION_STATS.get('graphnav_relocalization_fail', 0) + 1
        print(f"[LOCALIZZAZIONE] Riaggancio {why}: non riuscito ({e}).")
        return False


def segment_seen_free(global_map, x1, y1, x2, y2, margin_m, step_m=0.05):
    """True se il segmento e' stato osservato in ogni punto e non passa vicino a celle occupate."""
    n = max(2, int(np.ceil(np.hypot(x2 - x1, y2 - y1) / step_m)) + 1)
    for t in np.linspace(0.0, 1.0, n):
        x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
        if not global_map.is_known(x, y) or global_map.is_occupied(x, y, safety_margin=margin_m):
            return False
    return True


def graphnav_is_recording(recordingInterface):
    """True/False dalla registrazione GraphNav, None se non si riesce a saperlo."""
    try:
        return bool(recordingInterface._recording_client.get_record_status().is_recording)
    except Exception:
        return None


def graph_route(adj, start, goal):
    """Dijkstra su {wp: {vicino: lunghezza}}: (lunghezza, [wp...]) o (None, None)."""
    import heapq
    dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, float('inf')):
            continue
        for v, w in adj.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, float('inf')):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        return None, None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return dist[goal], path[::-1]


def plan_graphnav_shortcuts(poses, base_edges, start_id, goal_id, global_map,
                            max_m=None, margin_m=None):
    """
    Scorciatoie da aggiungere al grafo GraphNav per andare da start_id a goal_id.
    Funzione pura (niente robot), provata offline sui grafi scaricati: vedi test_replay.py.

    poses: {wp_id: (x, y)} dei waypoint di cui si conosce la posizione (frame della mappa
        globale); base_edges: [(wp_a, wp_b, lunghezza_m)] gli archi che il grafo ha gia'.
    Candidate: coppie di waypoint con posizione a meno di max_m, senza arco diretto, la cui
    linea retta e' stata VISTA libera nella mappa globale con margin_m (_segment_seen_free).
    Si calcola la strada piu' corta col grafo piu' TUTTE le candidate, e si restituiscono solo
    le candidate che quella strada usa.

    Restituisce (scorciatoie [(a, b, d)], lunghezza oggi, lunghezza con le scorciatoie); le
    lunghezze sono None se goal non e' raggiungibile.
    """
    max_m = P.GRAPHNAV_SHORTCUT_MAX_M if max_m is None else max_m
    margin_m = P.GRAPHNAV_SHORTCUT_MARGIN_M if margin_m is None else margin_m
    adj = {}
    for a, b, w in base_edges:
        w = float(w)
        if w < adj.get(a, {}).get(b, float('inf')):
            adj.setdefault(a, {})[b] = w
            adj.setdefault(b, {})[a] = w
    before, _ = graph_route(adj, start_id, goal_id)
    ids = list(poses)
    shortcut = {}
    for i in range(len(ids)):
        xa, ya = poses[ids[i]]
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if b in adj.get(a, {}):
                continue
            xb, yb = poses[b]
            d = float(np.hypot(xa - xb, ya - yb))
            if d > max_m or d < 0.05:
                continue
            if segment_seen_free(global_map, xa, ya, xb, yb, margin_m):
                shortcut[frozenset((a, b))] = d
    full = {k: dict(v) for k, v in adj.items()}
    for k, d in shortcut.items():
        a, b = tuple(k)
        full.setdefault(a, {})[b] = d
        full.setdefault(b, {})[a] = d
    after, path = graph_route(full, start_id, goal_id)
    used = []
    if path:
        for a, b in zip(path[:-1], path[1:]):
            k = frozenset((a, b))
            if k in shortcut and b not in adj.get(a, {}):
                used.append((a, b, shortcut[k]))
    return used, before, after


def graphnav_add_shortcuts(recordingInterface, label="", map_edges=True, sdk=True,
                           target_wp_id=None, robot_xy=None, sdk_budget_s=None, sdk_fast=None):
    """
    Aggiunge scorciatoie al grafo GraphNav prima di uno spostamento (vedi GRAPHNAV_SHORTCUTS).
    Non solleva mai: senza scorciatoie GraphNav funziona lo stesso, sulla catena.

    Spot accetta archi nuovi (create_edge) SOLO mentre registra ("NotRecordingError"),
    mentre la chiusura d'anello (process_topology) va fatta a registrazione ferma, e appena
    fermata puo' rispondere "MapModifiedError" finche' la mappa non e' finita di elaborare.
    Quindi:
      map_edges -- le scorciatoie dalla mappa globale: da chiamare PRIMA di stop_recording
                   (si saltano da sole se la registrazione e' ferma). Servono
                   target_wp_id (la destinazione) e, se il localizzatore non dice dove si
                   trova il robot, robot_xy; si creano solo quelle sulla strada (vedi
                   plan_graphnav_shortcuts).
      sdk       -- la chiusura d'anello di Spot: da chiamare DOPO stop_recording (si salta da
                   sola se si sta registrando), con qualche tentativo su MapModifiedError.
                   sdk_budget_s: limite lato Spot; None = GRAPHNAV_SDK_LOOP_BUDGET_S.
                   sdk_fast: senza ICP ne' feature matching; None = GRAPHNAV_SDK_LOOP_FAST
                   (vedi GRAPHNAV_SDK_LOOP_BUDGET_S).
    """
    if not P.GRAPHNAV_SHORTCUTS or recordingInterface is None:
        return 0
    added = 0
    recording = graphnav_is_recording(recordingInterface)
    gg = MISSION_CTX.get('global_grid')
    global_map = getattr(gg, 'global_occupancy_map', None) if gg is not None else None
    poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
    if (map_edges and target_wp_id and recording is not False and global_map is not None
            and len(poses) >= 2):
        try:
            graph = recordingInterface._get_graph(force_refresh=True)
            by_id = {w.id: w for w in graph.waypoints}
            xy = {p['waypoint_id']: (float(p['x']), float(p['y'])) for p in poses.values()
                  if p.get('waypoint_id') in by_id}
            base = []
            for e in graph.edges:
                t = e.from_tform_to.position
                base.append((e.id.from_waypoint, e.id.to_waypoint,
                             float(np.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2))))
            start = (graphnav_localization(recordingInterface) or {}).get('wp_id')
            if start not in by_id:
                if robot_xy is not None and xy:
                    start = min(xy, key=lambda k: np.hypot(xy[k][0] - robot_xy[0], xy[k][1] - robot_xy[1]))
                else:
                    start = None
            if start is not None and target_wp_id in by_id:
                used, before, after = plan_graphnav_shortcuts(xy, base, start, target_wp_id, global_map)
                for a, b, d in used:
                    try:
                        recordingInterface.create_edge_between_waypoint_objs(by_id[a], by_id[b])
                        added += 1
                    except Exception as e:
                        print(f"[GRAPHNAV] Scorciatoia {wp_name_by_id(recordingInterface, a)}-"
                              f"{wp_name_by_id(recordingInterface, b)} rifiutata ({e}).")
                if before is not None and after is not None:
                    print(f"[GRAPHNAV] {label}: strada verso {wp_name_by_id(recordingInterface, target_wp_id)} "
                          f"{before:.1f} m -> {after:.1f} m con {len(used)} scorciatoie "
                          f"(fino a {P.GRAPHNAV_SHORTCUT_MAX_M:.1f} m, viste libere con "
                          f"{P.GRAPHNAV_SHORTCUT_MARGIN_M:.2f} m di margine).")
        except Exception as e:
            print(f"[GRAPHNAV] Scorciatoie dalla mappa non aggiunte ({e}).")
        finally:
            if added:
                recordingInterface.invalidate_graph_cache()
    n_sdk = 0
    budget = P.GRAPHNAV_SDK_LOOP_BUDGET_S if sdk_budget_s is None else sdk_budget_s
    fast = P.GRAPHNAV_SDK_LOOP_FAST if sdk_fast is None else sdk_fast
    if sdk and P.GRAPHNAV_SDK_LOOP_CLOSURE and budget > 0 and recording is not True:
        # Spot calcola senza scrivere e gli archi li carichiamo noi (upload_result): niente
        # MapModifiedError a fine elaborazione. Ogni tentativo fallito finisce nel log con la
        # sua durata, e il tempo riportato e' il TOTALE (il 26-10 15:36 i tentativi falliti non
        # comparivano e il tempo era solo quello dell'ultimo: 65 s dichiarati su 138).
        what = 'veloce, senza ICP' if fast else 'completa'
        t_all = time.perf_counter()
        write_on_spot = False
        for attempt in range(1, 3):
            t0 = time.perf_counter()
            try:
                n_sdk, timed_out = recordingInterface.close_loops_checked(
                    max_edge_length_m=P.GRAPHNAV_SDK_LOOP_MAX_M, timeout_s=budget,
                    max_path_m=P.GRAPHNAV_SDK_LOOP_MAX_PATH_M, min_path_m=P.GRAPHNAV_SDK_LOOP_MIN_PATH_M,
                    icp_iters=0 if fast else None, feature_matching=False if fast else None,
                    upload_result=not write_on_spot,
                    prune_edges=None if fast else P.GRAPHNAV_SDK_LOOP_FINAL_PRUNE)
            except Exception as e:
                dt = time.perf_counter() - t0
                MISSION_STATS['sdk_loop_failures'] += 1
                retry = attempt == 1 and ('MapModified' in type(e).__name__ or 'modified' in str(e)
                                          or 'LoopClosureUpload' in type(e).__name__)
                # Caricamento degli archi rifiutato: al secondo tentativo li scrive Spot stesso
                # (mai provato sul robot con archi da caricare: la serie 3 ne ha trovati 0).
                write_on_spot = 'LoopClosureUpload' in type(e).__name__
                print(f"[GRAPHNAV] Chiusura d'anello di Spot ({what}): tentativo {attempt} non riuscito "
                      f"dopo {dt:.1f} s ({type(e).__name__}: {e})"
                      + (("; riprovo facendoli scrivere a Spot." if write_on_spot else "; riprovo.")
                         if retry else "; proseguo senza."))
                if retry:
                    time.sleep(1.0)
                    continue
                n_sdk = 0
                break
            dt_all = time.perf_counter() - t_all
            MISSION_STATS['sdk_loop_calls'] += 1
            MISSION_STATS['sdk_loop_s'] += dt_all
            MISSION_STATS['sdk_loop_edges'] += n_sdk
            MISSION_STATS['sdk_loop_partial'] += int(timed_out)
            print(f"[GRAPHNAV] Chiusura d'anello di Spot ({what}): {n_sdk} archi in {dt_all:.1f} s "
                  f"(limite {budget:.0f} s, "
                  f"{'SCADUTO: risultato parziale' if timed_out else 'finita'}"
                  + (f", {attempt} tentativi" if attempt > 1 else "") + ").")
            break
    if added or n_sdk:
        MISSION_STATS['graphnav_shortcuts'] = MISSION_STATS.get('graphnav_shortcuts', 0) + added + n_sdk
        print(f"[GRAPHNAV] {label}: {added} scorciatoie dalla mappa globale + {n_sdk} da Spot "
              f"(chiusure d'anello con controllo di collisione).")
    return added + n_sdk


def graphnav_navigate_to(recordingInterface, waypoint_id, robot_state_client, command_client, label,
                          timeout_s=P.GRAPHNAV_TIMEOUT_S):
    """Porta il robot a un waypoint con GraphNav. True solo se ci arriva davvero."""
    # Prima di muoversi: chiusure d'anello di Spot (a registrazione ferma). Le scorciatoie
    # dalla mappa le aggiunge chi chiama, PRIMA di fermare la registrazione.
    # Tutto cio' che sta qui dentro finisce nella voce 'GraphNav' di [TEMPO]: e' una delle
    # fasi piu' lunghe della missione (il solo ritorno a wp_0 puo' durare minuti).
    _t_gn = time.perf_counter()
    try:
        # 2026-10-26: anche i tratti GraphNav finiscono in trajectory.csv (vedi _TRAJECTORY_COLUMNS).
        with TrajectoryLog(robot_state_client, 'graphnav'):
            return _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client,
                                               command_client, label, timeout_s)
    finally:
        timing_add('GraphNav', time.perf_counter() - _t_gn)


def _graphnav_navigate_to_inner(recordingInterface, waypoint_id, robot_state_client, command_client,
                                label, timeout_s):
    graphnav_add_shortcuts(recordingInterface, label, map_edges=False, sdk=True)
    client = recordingInterface._graph_nav_client
    tp = _graph_nav_pb2.TravelParams()
    tp.velocity_limit.CopyFrom(movements._velocity_limit(P.GRAPHNAV_MAX_LINEAR_VEL_MPS, P.GRAPHNAV_MAX_ANGULAR_VEL_RPS))
    # Rilevatore di perdita permissivo (vedi LOCALIZZAZIONE DI GRAPHNAV).
    if P.GRAPHNAV_LOST_DETECTOR_STRICTNESS and hasattr(tp, 'lost_detector_strictness'):
        try:
            # dal descrittore: l'enum sta in lost_detection_pb2, che nelle versioni vecchie
            # dell'SDK non c'e'; cosi' funziona ovunque il campo esista.
            enum = tp.DESCRIPTOR.fields_by_name['lost_detector_strictness'].enum_type
            tp.lost_detector_strictness = enum.values_by_name[P.GRAPHNAV_LOST_DETECTOR_STRICTNESS].number
        except Exception:
            pass
    print(f"[GRAPHNAV] {label}: avvio (massimo {timeout_s:.0f} s, {P.GRAPHNAV_MAX_LINEAR_VEL_MPS:.1f} m/s).")
    # Prima di partire si guarda lo stato del localizzatore; se e' perso si riaggancia
    # subito, invece di farsi rifiutare il comando.
    relocs = 0
    loc0 = graphnav_log_localization(None, recordingInterface, robot_state_client,
                                     f"prima di '{label}'", force=True)
    if graphnav_is_lost(loc0):
        relocs += 1
        graphnav_relocalize(recordingInterface, robot_state_client, f"prima di '{label}'")
    t0, cmd_id, fb_errors, nav_errors = time.time(), None, 0, 0
    while time.time() - t0 < timeout_s:
        try:
            cmd_id = client.navigate_to(waypoint_id, 1.0, command_id=cmd_id, travel_params=tp)
            nav_errors = 0
        except Exception as e:
            if 'RobotLost' in type(e).__name__ and relocs < P.GRAPHNAV_MAX_RELOCALIZATIONS:
                relocs += 1
                print(f"[GRAPHNAV] {label}: GraphNav dice che il robot e' perso: riaggancio "
                      f"({relocs}/{P.GRAPHNAV_MAX_RELOCALIZATIONS}).")
                graphnav_relocalize(recordingInterface, robot_state_client, f"durante '{label}'")
                cmd_id = None
                time.sleep(0.5)
                continue
            nav_errors += 1
            print(f"[GRAPHNAV] {label}: comando rifiutato ({nav_errors}/3): {e}")
            # Un errore RPC transitorio non abbandona lo spostamento: si rinuncia al terzo.
            if nav_errors >= 3:
                movements.stop_robot(command_client, reason="GraphNav: comando rifiutato")
                MISSION_STATS['graphnav_fail'] += 1
                return False
            cmd_id = None
            time.sleep(0.5)
            continue
        time.sleep(0.5)
        try:
            status = client.navigation_feedback(cmd_id).status
            fb_errors = 0
        except Exception as e:
            fb_errors += 1
            print(f"[GRAPHNAV] {label}: errore nel feedback ({fb_errors}/3): {e}")
            if fb_errors >= 3:
                movements.stop_robot(command_client, reason="GraphNav: feedback non disponibile")
                MISSION_STATS['graphnav_fail'] += 1
                return False
            cmd_id = None      # il comando potrebbe non essere piu' valido: se ne invia uno nuovo
            time.sleep(0.5)
            continue
        if status == _GN.STATUS_REACHED_GOAL:
            print(f"[GRAPHNAV] {label}: arrivato in {time.time() - t0:.0f} s.")
            MISSION_STATS['graphnav_ok'] += 1
            graphnav_log_drift(recordingInterface, robot_state_client, label)
            return True
        if status == _GN.STATUS_LOST and relocs < P.GRAPHNAV_MAX_RELOCALIZATIONS:
            # Riaggancio sul waypoint piu' vicino (NON sulla destinazione, e senza dichiarare
            # l'arrivo: si rimanda il comando e si continua a navigare).
            relocs += 1
            print(f"[GRAPHNAV] {label}: GraphNav ha perso la localizzazione per strada: "
                  f"riaggancio ({relocs}/{P.GRAPHNAV_MAX_RELOCALIZATIONS}).")
            graphnav_log_localization(None, recordingInterface, robot_state_client,
                                      f"perso durante '{label}'", force=True)
            graphnav_relocalize(recordingInterface, robot_state_client, f"durante '{label}'")
            cmd_id = None
            continue
        if status == _GN.STATUS_LOST:
            # Non si forza la localizzazione sulla destinazione (vedi sopra): fallimento.
            print(f"[GRAPHNAV] {label}: GraphNav ha perso la localizzazione. Fermo il robot.")
            movements.stop_robot(command_client, reason="GraphNav: robot perso")
            MISSION_STATS['graphnav_fail'] += 1
            return False
        if status == _GN.STATUS_COMMAND_TIMED_OUT:
            cmd_id = None          # comando scaduto: se ne invia uno nuovo
            continue
        if status in _GRAPHNAV_FAIL_STATUSES:
            print(f"[GRAPHNAV] {label}: interrotto -- {_GRAPHNAV_FAIL_STATUSES[status]}.")
            movements.stop_robot(command_client, reason=f"GraphNav: {_GRAPHNAV_FAIL_STATUSES[status]}")
            MISSION_STATS['graphnav_fail'] += 1
            return False
    print(f"[GRAPHNAV] {label}: tempo massimo di {timeout_s:.0f} s superato. Fermo il robot.")
    movements.stop_robot(command_client, reason="GraphNav: tempo massimo superato")
    MISSION_STATS['graphnav_fail'] += 1
    return False


def graphnav_return_to_start(recordingInterface, robot_state_client, command_client):
    """Ritorno a wp_0 a fine missione, con tempo massimo. False se non ci riesce."""
    try:
        graph = recordingInterface._get_graph(force_refresh=True)
        wp0 = next((w for w in graph.waypoints if w.annotations.name == "wp_0"), None)
    except Exception as e:
        print(f"[GRAPHNAV] Ritorno a wp_0: grafo non disponibile ({e}).")
        return False
    if wp0 is None:
        print("[GRAPHNAV] Ritorno a wp_0: waypoint wp_0 non trovato nel grafo.")
        return False
    return graphnav_navigate_to(recordingInterface, wp0.id, robot_state_client, command_client,
                                 "ritorno a wp_0", timeout_s=2 * P.GRAPHNAV_TIMEOUT_S)


def graphnav_drop_breadcrumb(env, recordingInterface, robot_state_client, prm_node_id,
                             cell_row, cell_col, force=False):
    """
    Lascia una briciola: un waypoint GraphNav nel punto in cui si trova il robot,
    legato al nodo PRM da cui sta passando.

    Serve per i ritorni. Camminando ALL'INDIETRO sui tratti registrati
    (retreat_along_traveled_path) il robot non vede dove mette le zampe, non si
    rilocalizza, e un errore si accumula. Con una traccia di waypoint il ritorno lo fa
    GraphNav, che cammina in avanti, con la localizzazione e l'anti-ostacolo di Boston
    Dynamics (decisione dell'utente).

    Densita': un waypoint solo se il robot e' almeno GRAPHNAV_WAYPOINT_MIN_SPACING_M
    dall'ultimo lasciato, piu' sempre uno all'ingresso di una cella (force=True). Un
    waypoint per OGNI posizione di sosta darebbe una mappa troppo densa: i passi parziali
    lasciano soste a 20-60 cm una dall'altra.

    La traccia vive su env (_graphnav_trail) perche' deve sopravvivere ai tentativi di cella.
    Non solleva mai: una briciola mancata e' una briciola, non un motivo per fermare il robot.
    """
    if recordingInterface is None:
        return None
    trail = getattr(env, '_graphnav_trail', None)
    if trail is None:
        trail = env._graphnav_trail = []
    try:
        x, y, _, _ = spotUtils.getPosition(robot_state_client)
    except Exception as e:
        print(f"[BRICIOLA] Posizione non leggibile: {e}")
        return None
    if not force and trail:
        last = trail[-1]
        if float(np.hypot(x - last['x'], y - last['y'])) < P.GRAPHNAV_WAYPOINT_MIN_SPACING_M:
            return None
    try:
        recordingInterface.create_default_waypoint(cell_row=cell_row, cell_col=cell_col)
        # Nome e id dal waypoint appena creato (li porta la risposta del robot), non da
        # get_localization_state(), che riscarica tutto il grafo solo per leggerli. Ripiego
        # su get_localization_state() se l'interfaccia non li espone.
        last = getattr(recordingInterface, 'last_created_waypoint', None)
        if isinstance(last, dict) and last.get('id'):
            wp_id, wp_name = last['id'], last.get('name')
        else:
            loc = recordingInterface.get_localization_state()
            wp_id = loc.get('waypoint_id') if isinstance(loc, dict) else None
            wp_name = loc.get('waypoint_name') if isinstance(loc, dict) else None
    except Exception as e:
        print(f"[BRICIOLA] Waypoint non creato: {e}")
        return None
    if not wp_id:
        print("[BRICIOLA] Waypoint creato ma non localizzato: non lo metto nella traccia "
              "(un waypoint senza id non si puo' raggiungere).")
        return None
    crumb = {'wp_id': wp_id, 'wp_name': wp_name, 'x': float(x), 'y': float(y),
             'prm_node': prm_node_id, 'cell': (cell_row, cell_col)}
    trail.append(crumb)
    print(f"[BRICIOLA] Waypoint {wp_name} in ({x:.2f}, {y:.2f}), nodo PRM {prm_node_id} "
          f"(traccia: {len(trail)} waypoint).")
    # Stato del localizzatore di GraphNav, stampato solo quando cambia.
    graphnav_log_localization(env, recordingInterface, robot_state_client, f"briciola {wp_name}")
    return crumb


def graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                       min_dist_m=0.40):
    """
    Torna al waypoint piu' recente che sia almeno min_dist_m DIETRO, con GraphNav.

    Si cerca indietro nella traccia perche' l'ultima briciola puo' essere stata lasciata qui,
    dove il robot e' adesso: tornarci non sposterebbe niente. Se nessuna briciola e' abbastanza
    lontana si usa comunque la piu' vecchia disponibile.

    Restituisce (ok, crumb). ok=True solo se GraphNav dichiara di essere ARRIVATO: in
    _graphnav_navigate_to lo STATUS_LOST e' un fallimento, non si forza la localizzazione
    sulla destinazione (vedi SPOSTAMENTI CON GRAPHNAV).

    La registrazione va FERMATA prima di navigare e ripresa dopo: GraphNav non naviga mentre
    registra. E' la stessa sequenza usata per gli spostamenti fra celle.
    """
    trail = getattr(env, '_graphnav_trail', None) or []
    if not trail:
        return False, None
    try:
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
    except Exception:
        rx, ry = trail[-1]['x'], trail[-1]['y']
    idx = None
    for k in range(len(trail) - 1, -1, -1):
        if float(np.hypot(trail[k]['x'] - rx, trail[k]['y'] - ry)) >= min_dist_m:
            idx = k
            break
    if idx is None:
        idx = 0
    hop = len(trail) - 1 - idx
    crumb = trail[idx]
    d_crumb = float(np.hypot(crumb['x'] - rx, crumb['y'] - ry))
    if d_crumb < min_dist_m:
        # Tutte le briciole sono qui attorno (22-10 16:22: il robot stava SU wp_138): GraphNav
        # non sposterebbe il robot. Si ripianifica da dove si e', senza muoversi.
        MISSION_STATS['backtracks_in_place'] += 1
        print(f"[RITORNO] La briciola piu' lontana dietro, {crumb['wp_name']}, e' a {d_crumb:.2f} m "
              f"(meno di {min_dist_m:.2f}): non mi muovo, ripianifico da qui.")
        return True, dict(crumb, in_place=True)
    print(f"[RITORNO] Torno a {crumb['wp_name']}, a {d_crumb:.2f} m dal robot.")
    recording_stopped = False
    graphnav_add_shortcuts(recordingInterface, "prima del ritorno", map_edges=True, sdk=False,
                           target_wp_id=crumb['wp_id'], robot_xy=(rx, ry))
    try:
        recordingInterface.stop_recording()
        recording_stopped = True
    except Exception as e:
        print(f"[RITORNO] Non ho potuto fermare la registrazione ({e}): provo comunque.")
    ok = graphnav_navigate_to(
        recordingInterface, crumb['wp_id'], robot_state_client, command_client,
        f"ritorno al waypoint {crumb['wp_name']} in ({crumb['x']:.2f}, {crumb['y']:.2f})")
    if recording_stopped:
        try:
            recordingInterface.start_recording()
        except Exception as e:
            print(f"[RITORNO] Registrazione non ripresa ({e}): le briciole successive "
                  "potrebbero non essere create.")
    if ok:
        # I tratti percorsi a zampe non descrivono piu' la strada da cui si e' arrivati:
        # GraphNav non li registra, e la ritirata di ripiego non deve usarli.
        env._traveled_arcs = []
        del trail[len(trail) - hop:]        # le briciole oltre questa non servono piu'
    return ok, crumb
