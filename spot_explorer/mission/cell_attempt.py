"""Un tentativo di entrare in una cella: TentativoCella, il ciclo di navigazione con il fronte sicuro."""
import os
import time
from datetime import datetime
import numpy as np

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from types import SimpleNamespace
from spot_explorer.graphnav import graphnav as navGraphUtils
from spot_explorer.robot import motion as movements
from spot_explorer.perception import grids as spotGrid
from spot_explorer.mission import cells as environmentMap
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import safe_frontier as arcVerification

from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.planning import roadmap as prm_graph_module   # dentro le funzioni `prm_graph` e' spesso il parametro (istanza)
from spot_explorer.planning import mission_graph                   # reticolo dei nodi della missione

from spot_explorer import config as P
from spot_explorer.records import (MISSION_STATS, MISSION_CTX, BodyTiltRecorder, TrajectoryLog, append_csv, append_trajectory, log_segment, mission_z0, save_npz_async, timing_add, visualize_grid_with_candidates)
from spot_explorer.planning import goal
from spot_explorer.mission import recovery


def _rientro_batteria(where=None):
    """Una lettura della batteria (vedi batteria.MonitorBatteria.controlla): True se la
    missione deve smettere di esplorare e rientrare. False se il monitor non c'e'."""
    monitor = MISSION_CTX.get('batteria')
    return monitor.controlla(where) if monitor is not None else False


def _batteria_scarica():
    """Come _rientro_batteria ma senza leggere: solo se il rientro e' gia' deciso."""
    monitor = MISSION_CTX.get('batteria')
    return monitor is not None and monitor.rientro


# =========================================================================
# INGRESSO IN UNA CELLA: un tentativo (TentativoCella)
# =========================================================================


# Esiti di una fase del ciclo di navigazione (vedi TentativoCella._giro): None = si va avanti
# con la fase successiva.
CONTINUA = "continua"        # giro finito: si ricomincia dalla scansione
INTERROMPI = "interrompi"    # fine del ciclo: decide la posizione finale (TentativoCella._fine)
FALLITO = "fallito"          # il tentativo e' fallito


class TentativoCella:
    """Prova a entrare nella cella (target_row, target_col) dalla posizione attuale del robot.

    Scansione, aggiornamento delle mappe e del grafo, scelta del punto obiettivo, percorso,
    poi il ciclo di navigazione con il fronte sicuro. True se il robot finisce dentro la cella.

    mobility_kwargs: dict facoltativo che impone l'andatura (locomotion_hint, ground_mu_hint,
        swing_height) per tutti i tratti, passato fino a movements.relative_move attraverso
        navigate_to. Vuoto = andatura scelta tratto per tratto (select_mobility_params).

    COME E' FATTO. _prepara una volta (_prima_scansione, _dati_locali_nel_grafo,
    _obiettivo_e_percorso, _azzera_contatori), poi _giro finche' una fase decide l'esito:

      _guardie              troppi giri, batteria bassa, caduta, robot fermo da troppo
      _scansione_e_grafo    _scansione_fresca, _aggiorna_grafo, _ripianifica
      _fronte_e_decisione   scorciatoia, fronte sicuro, arcVerification.decide_next_move:
                            move / reached_node / arrived / wait / blocked
      _rotazione            (move) non posso girarmi: _avanza_nel_cono, _arretra_per_ruotare,
                            trappola, tratto scartato
      _nodo_o_arrivo        (reached_node, arrived)
      _attesa               (wait) _cambia_obiettivo_irraggiungibile, _ripianifica_subito,
                            altrimenti riguardo da fermo
      _blocco               (blocked) obiettivo da cambiare, trappola, tratto fuori per la
                            missione, nuova strada o ritorno con GraphNav
      _esegui_tratti        (move) _controlla_posa_d_arrivo, poi per ogni gamba
                            _prepara_gamba e _cammina_tratto (_terreno_del_tratto,
                            _muovi_e_registra, esito)

    Azioni comuni alle fasi: _usa_percorso, _scarta_tratto (per ora / per il tentativo / per
    la missione), _strada_fatta e _sosta, _ventaglio ed _esci_dalla_trappola,
    _riscegli_obiettivo, _ritorno (GraphNav all'indietro, vedi _trap_backtrack).
    """

    def __init__(self, local_grid, global_grid, robot_state_client, command_client,
                 env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                 iteration=0,
                 recordingInterface=None, verification_tracker=None, mobility_kwargs=None):
        self.local_grid = local_grid
        self.global_grid = global_grid
        self.robot_state_client = robot_state_client
        self.command_client = command_client
        self.env = env
        self.target_row = target_row
        self.target_col = target_col
        self.sampler = sampler
        self.prm_graph = prm_graph
        self.mission_folder = mission_folder
        self.iteration = iteration
        self.recordingInterface = recordingInterface
        self.verification_tracker = verification_tracker
        self.mobility_kwargs = mobility_kwargs

    def esegui(self):
        """True se il robot finisce dentro la cella (o accettato appena fuori, vedi _fine)."""
        if self._prepara() is FALLITO:
            return False

        while self.path_waypoints and len(self.path_waypoints) > 0:
            esito = self._giro()
            if esito is FALLITO:
                return False
            if esito is INTERROMPI:
                break
        return self._fine()

    def _giro(self):
        """Un giro del ciclo di navigazione: le fasi in ordine, finche' una non decide l'esito."""
        for fase in (self._guardie, self._scansione_e_grafo, self._fronte_e_decisione,
                     self._rotazione, self._tempi, self._nodo_o_arrivo, self._attesa,
                     self._blocco, self._esegui_tratti):
            esito = fase()
            if esito is not None:
                return esito
        return CONTINUA

    def _prepara(self):
        """Prima del ciclo: rialzo se caduto, prima scansione, mappe e grafo, punto obiettivo
        e percorso, contatori del tentativo. FALLITO se non si puo' nemmeno partire."""
        self.mobility_kwargs = self.mobility_kwargs or {}

        print(f"\n[ATTEMPT] Provo a entrare nella cella ({self.target_row},{self.target_col}) dalla posizione attuale...")

        scan = self._prima_scansione()
        if scan is None:
            return FALLITO
        self._dati_locali_nel_grafo(scan)
        esito = self._obiettivo_e_percorso(scan)
        if esito is not None:
            return esito
        self._azzera_contatori()

    def _prima_scansione(self):
        """Rialzo se caduto, prima scansione del tentativo, posa del robot, mappe globali
        aggiornate e salvate. La scansione elaborata, o None se non arriva."""
        # Il robot arriva qui da un altro movimento (navigazione fra celle, ritirata): se e'
        # caduto nel frattempo si rialza prima di tutto. Solleva RobotFallenError se non puo'.
        # Poi esce dal disco pericoloso ripercorrendo i propri passi: rialzato, e' ancora DENTRO
        # il punto della caduta, e da li' ogni arco del PRM verrebbe scartato.
        if movements.check_and_recover_fall(self.robot_state_client, self.command_client,
                                   self.global_grid.global_occupancy_map, "all'inizio del tentativo") is not None:
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)

        # 1. La scansione: i layer in una richiesta, poi (sotto) la loro elaborazione
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Rilevamento griglie locali fallito.")
            return None
        main_proto = scan.proto
        pts, cells_obstacle_dist = scan.pts, scan.od
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size

        # =========================================================================
        # MAPPE GLOBALI: create alla prima scansione, poi accumulate per tutta la missione
        # =========================================================================
        if self.global_grid.global_occupancy_map is None:
            self.global_grid.global_occupancy_map = spotGrid.GlobalGrid(resolution=cell_size)
            print("[MAP] Inizializzata nuova mappa di occupazione globale persistente.")

        # Mappa globale delle altezze: stessa risoluzione nativa della griglia locale (nessuna
        # riduzione), creata come quella di occupazione.
        if self.global_grid.global_terrain_map is None:
            self.global_grid.global_terrain_map = spotGrid.GlobalTerrainGrid(resolution=cell_size)
            print("[MAP] Inizializzata nuova mappa globale persistente del terreno (altezza).")

        self.global_map = self.global_grid.global_occupancy_map
        self.global_terrain_map = self.global_grid.global_terrain_map

        transforms_snapshot = main_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        self.robot_x, self.robot_y, self.robot_z = vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z

        quat = vision_tform_body.rotation
        self.robot_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))

        # 2. Impronta del corpo, terreno corretto (da qui in poi si usa solo terrain_real),
        # pendenza e rugosita', maschera degli ostacoli
        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-iniziale")
        self.footprint_mask_2d, terrain_real, is_valid = scan.footprint_mask, scan.terrain, scan.is_valid
        obstacle_mask = scan.obstacle_mask

        # Aggiorna le mappe globali (occupazione e terreno) con i dati appena processati
        spotGrid.fold_scan_into_global_maps(self.global_map, self.global_terrain_map, pts, terrain_real, obstacle_mask,
                                   is_valid, cells_obstacle_dist, (num_y, num_x),
                                   (self.robot_x, self.robot_y, self.robot_yaw))

        # La mappa del terreno serve al PRM come ripiego per la pendenza
        self.prm_graph.set_global_terrain_map(self.global_terrain_map)

        # Mappe globali su disco (.pkl), a ogni tentativo
        if self.mission_folder is not None:
            pkl_map_path = os.path.join(self.mission_folder, "global_occupancy_map.pkl")
            self.global_map.save_map(pkl_map_path)
            pkl_terrain_path = os.path.join(self.mission_folder, "global_terrain_map.pkl")
            self.global_terrain_map.mission_z0 = MISSION_CTX.get('mission_z0')
            self.global_terrain_map.save_map(pkl_terrain_path)
        return scan

    def _dati_locali_nel_grafo(self, scan):
        """Terreno, pendenza e ostacoli della scansione nel PRM, poi aggiornamento degli archi."""
        pts, cells_obstacle_dist, valid_values = scan.pts, scan.od, scan.valid
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size
        terrain_real, is_valid = scan.terrain, scan.is_valid
        grad_values, rough_values = scan.grad, scan.rough
        grad_for_prm = grad_values.copy()
        rough_for_prm = rough_values.copy()
        main_proto = scan.proto
        transforms_snapshot = main_proto.local_grid.transforms_snapshot

        # ================================================================== #
        # DATI LOCALI NEL PRM: terreno, pendenza e rugosita' della scansione
        # ================================================================== #
        local_x_min, local_x_max = pts[:, 0].min(), pts[:, 0].max()
        local_y_min, local_y_max = pts[:, 1].min(), pts[:, 1].max()

        grid_frame_name = main_proto.local_grid.frame_name_local_grid_data
        vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
        grid_origin_x = vision_tform_grid.position.x
        grid_origin_y = vision_tform_grid.position.y

        grad_values_2d = grad_for_prm.reshape((num_y, num_x))
        rough_values_2d = rough_for_prm.reshape((num_y, num_x))
        terrain_values_2d = terrain_real.reshape((num_y, num_x))

        self.prm_graph.update_local_grid_data(
            grad_values_2d=grad_values_2d,
            rough_values_2d=rough_values_2d,
            terrain_values_2d=terrain_values_2d,
            valid_2d=spotGrid.slope_usable_mask(is_valid, cells_obstacle_dist, (num_y, num_x)),
            grid_origin_x=grid_origin_x,
            grid_origin_y=grid_origin_y,
            cell_size=cell_size
        )

        # Gradiente e rugosita' dei nodi PRM dentro la griglia locale; quelli fuori tengono il
        # valore che hanno (0.0 se non ne hanno ancora uno)
        for node_id, (nx, ny) in self.prm_graph.nodes.items():
            if local_x_min <= nx <= local_x_max and local_y_min <= ny <= local_y_max:
                dx = nx - grid_origin_x
                dy = ny - grid_origin_y

                col_idx = int(dx / cell_size)
                row_idx = int(dy / cell_size)

                if 0 <= row_idx < num_y and 0 <= col_idx < num_x:
                    flat_idx = row_idx * num_x + col_idx
                    if flat_idx < len(grad_values):
                        self.prm_graph.node_gradients[node_id] = grad_values[flat_idx]
                    if flat_idx < len(rough_values):
                        self.prm_graph.node_roughness[node_id] = rough_values[flat_idx]
            else:
                if node_id not in self.prm_graph.node_gradients:
                    self.prm_graph.node_gradients[node_id] = 0.0
                if node_id not in self.prm_graph.node_roughness:
                    self.prm_graph.node_roughness[node_id] = 0.0

        # La scansione appena fatta entra anche nello scarto preliminare degli archi
        # (prm_graph.LIVE_PREFILTER_OD_M): alla costruzione la mappa globale non ha ancora
        # confermato niente di questo posto, e gli archi che attraversano gli ostacoli appena
        # visti resterebbero validi. Qui obstacle_distance e' quella grezza: la fotografia
        # filtrata nasce nel ciclo di navigazione.
        try:
            self.prm_graph.set_live_obstacles(
                np.asarray(cells_obstacle_dist, dtype=np.float64).reshape((num_y, num_x)),
                np.asarray(valid_values, dtype=np.float64).reshape((num_y, num_x)) >= spotGrid.TERRAIN_VALID_MIN,
                grid_origin_x, grid_origin_y, cell_size, self.robot_x, self.robot_y, self.robot_yaw)
        except Exception as e:
            self.prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
            print(f"[ARCHI] Scansione non usabile per lo scarto preliminare ({e}).")

        t_build = time.perf_counter()
        # ATTENZIONE AI NOMI: qui `self.prm_graph` e' l'istanza di PRM, non il modulo omonimo
        # (importato come prm_graph_module). Le coppie candidate sono quelle del grafo di
        # missione (vedi mission_graph.build_mission_prm); veti e pesi li calcola
        # PRM._evaluate_and_add_edge.
        mission_graph.rebuild_prm_edges(self.prm_graph, self.global_map)
        timing_add('costruzione grafo', time.perf_counter() - t_build)
        _upd = getattr(getattr(self.prm_graph, 'mission_graph', None), 'last_update', None)
        print(f"[TEMPO] {'aggiornamento locale' if _upd and _upd[2] else 'costruzione completa'} "
              f"del grafo: {time.perf_counter() - t_build:.2f} s")
        if getattr(self.prm_graph, 'live_veto_last', 0):
            print(f"[ARCHI] {self.prm_graph.live_veto_last} archi scartati subito dalla scansione dal "
                  f"vivo (passano a meno di {prm_graph_module.LIVE_PREFILTER_OD_M:.2f} m da un "
                  f"ostacolo appena visto), prima della pendenza.")
        # Un tentativo nuovo: le trappole del tentativo precedente non valgono piu' (vedi
        # _trap_backtrack). Le penalita' sono uno strato separato dagli archi, quindi
        # sopravvivono alle ricostruzioni DENTRO il tentativo.
        self.prm_graph.clear_penalties_for_attempt()

    def _obiettivo_e_percorso(self, scan):
        """Nodo del robot, punto obiettivo nella cella, briciola di partenza e primo percorso.
        FALLITO se nella cella non c'e' nessun punto obiettivo valido."""
        pts, cells_obstacle_dist, valid_values = scan.pts, scan.od, scan.valid
        num_x, num_y, cell_size = scan.num_x, scan.num_y, scan.cell_size
        terrain_real, obstacle_mask = scan.terrain, scan.obstacle_mask
        grad_values, rough_values = scan.grad, scan.rough
        save_path = os.path.join(self.mission_folder,
                                 f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}.png") if self.mission_folder else None

        # Solo celle visitate + cella del robot + cella obiettivo (vedi PLAN_ONLY_EXPLORED_CELLS).
        mission_graph.restrict_planning_to_explored_cells(self.prm_graph, self.env, self.target_row, self.target_col,
                                             (self.robot_x, self.robot_y))

        # --- Nodi di partenza e di arrivo: nodi ESISTENTI del grafo ---------------------------
        # add_stop_node aggancia la posizione al nodo adatto invece di crearne uno. Il nodo del
        # robot serve PRIMA della scelta dell'obiettivo: si sceglie fra i punti che il grafo
        # collega al robot (vedi find_best_point_in_cell, criterio 2).
        self.robot_node_id = self.prm_graph.add_stop_node(
            self.robot_x, self.robot_y, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
            label="partenza tentativo")
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)


        # Ricerca del punto target e pianificazione
        self.target_x, self.target_y, valid_samples, rejected_samples = goal.find_best_point_in_cell(
            self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, pts, cells_obstacle_dist, self.sampler,
            global_map=self.global_map, reachable=self._reachable_from_robot)

        # L'obiettivo di questo tentativo, per il recupero subito dopo un fallimento
        # (vedi RECOVERY_SAME_GOAL_M nel ciclo principale).
        self.env._last_attempt_goal = (None if self.target_x is None or self.target_y is None
                                  else (float(self.target_x), float(self.target_y)))
        if self.target_x is None or self.target_y is None:
            print(f"[FAIL] Nessun obiettivo valido nella cella ({self.target_row},{self.target_col}) dalla posizione attuale")
            visualize_grid_with_candidates(
                pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': rejected_samples, 'valid': valid_samples},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=save_path, prm_graph=self.prm_graph, chosen_path=None,
                cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
                grad_values=grad_values, rough_values=rough_values
            )
            return FALLITO

        # Briciola GraphNav del punto di partenza del tentativo: e' quello a cui serve poter
        # tornare se la cella si rivela un vicolo cieco.
        navGraphUtils.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client, self.robot_node_id,
                                 self.target_row, self.target_col, force=True)
        # L'obiettivo e' il centro o un nodo del reticolo (lo sceglie find_best_point_in_cell fra
        # i punti del campionatore).
        self.goal_id = self.prm_graph.add_stop_node(
            self.target_x, self.target_y, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"obiettivo cella ({self.target_row},{self.target_col})", is_robot=False)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
        path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)

        if path_ids is None:
            print(
                f"[GRAFO DISCONNESSO] Nessun percorso nel PRM dal nodo {self.robot_node_id} al nodo "
                f"{self.goal_id}: i due nodi stanno in componenti separate del grafo. Non e' un "
                f"errore di Dijkstra -- vuol dire che tutti gli archi di collegamento sono "
                f"stati vetati (occupazione o pendenza). Vedi i conteggi in [PRM MISSION SUMMARY].")
            print(f" -> Coordinate partenza robot: ({self.robot_x:.2f}, {self.robot_y:.2f})")
            print(f" -> Coordinate target cella: ({self.target_x:.2f}, {self.target_y:.2f})")


        self.full_path_coords = None
        self.path_waypoints = []

        if path_ids is not None:
            self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in path_ids]
            self.path_waypoints = self._waypoints_from_ids(path_ids)
        else:
            self.path_waypoints = None

        visualize_grid_with_candidates(
            pts=pts, terrain_real=terrain_real, obstacle_mask=obstacle_mask,
            robot_x=self.robot_x, robot_y=self.robot_y,
            candidates={'rejected': rejected_samples, 'valid': valid_samples},
            chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
            save_path=save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
            cells_obstacle_dist=cells_obstacle_dist, valid_values=valid_values,
            grad_values=grad_values, rough_values=rough_values
        )
        return None

    def _azzera_contatori(self):
        """Contatori e memorie del tentativo, prima del primo giro."""
        # ===================================================================================
        # CICLO DI NAVIGAZIONE CON IL FRONTE SICURO
        #
        # A ogni giro: scansione fresca, aggiornamento mappe e PRM, eventuale ripianificazione,
        # poi il FRONTE: fin dove il percorso e' confermato libero adesso. Il robot avanza
        # fino a li' (meno meta' corpo e margine), lungo il percorso.
        #   - il muso punta SEMPRE il prossimo punto: niente marcia di traverso; se la
        #     rotazione non e' possibile si prova il cono, poi l'arretramento dritto (vedi
        #     plan_cone_move e plan_straight_retreat);
        #   - la rotazione si giudica sulle celle davvero spazzate (spotGrid.rotation_is_clear),
        #     non sul cerchio circoscritto;
        #   - se l'obiettivo risulta irraggiungibile sui dati freschi se ne sceglie un altro
        #     nella cella, invece di aspettare e dichiarare un blocco (goal_still_reachable).
        # ===================================================================================
        self.replan_counter = 0
        self.need_refresh = True        # primo giro del tentativo: vedi REFRESH A EVENTI
        self.no_progress = 0
        self.block_replans = 0
        self.loop_iter = 0
        self.rotation_fails = {}     # tratto -> quante volte la rotazione verso di esso e' stata rifiutata
        self.rotation_discards = 0   # scarti per rifiuto di rotazione in QUESTO tentativo
        self.best_goal_dist = float('inf')   # per azzerare i rifiuti solo sui PROGRESSI
        self.idle_loops = 0          # giri consecutivi senza spostarsi (vedi MAX_IDLE_LOOPS)
        self.last_xy = None          # posizione al giro precedente, per contare i giri a vuoto
        self.rotation_fail_xy = {}   # tratto -> posizione del robot all'ultimo rifiuto
        self.retreats_in_a_row = 0   # arretramenti consecutivi: azzerato da un avanzamento vero
        self.early_replans = 0       # ripianificazioni alla prima cella bloccante (ramo 'wait')
        self.arrival_refusals = 0    # passi rifiutati perche' la posa d'arrivo non ha uscite
        self.subito_tratti = set()   # tratti gia' scartati da [SUBITO] in questo tentativo
        self.goal_repicks = 0        # quante volte si e' riscelto il punto obiettivo
        self.tried_goals = []        # punti obiettivo gia' provati e risultati irraggiungibili
        self.arrived_near_goal = False   # arrivo accettato con il robot appena fuori dalla cella
        self.trap_state = {'n': 0}   # ritorni indietro in QUESTO tentativo, mai azzerato (vedi _trap_backtrack)

    def _reachable_from_robot(self, px, py):
        nid = self.prm_graph.anchor_node(px, py, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
        return nid is not None and self.prm_graph.find_path_dijkstra(self.robot_node_id, nid) is not None

    def _waypoints_from_ids(self, ids):
        """[(node_id, x, y), ...] senza il primo nodo (la posizione del robot)."""
        out = []
        for nid in ids[1:]:
            nx, ny = self.prm_graph.get_node_position(nid)
            out.append((nid, nx, ny))
        return out

    # --- Azioni comuni alle decisioni ----------------------------------------------------
    def _usa_percorso(self, ids):
        """Il percorso diventa ids (dal nodo del robot in poi); la conferma di un blocco
        riparte da zero."""
        self.path_waypoints = self._waypoints_from_ids(ids)
        self.full_path_coords = [self.prm_graph.get_node_position(nid) for nid in ids]
        self.no_progress = 0

    def _scarta_tratto(self, a, b, durata):
        """Toglie il tratto a-b dal grafo. durata:
          'ora'       -- fino al prossimo refresh, che lo rimette se i dati dicono che e' libero
                         (vedi prm_graph.mark_edge_blocked_soft);
          'tentativo' -- per il resto del tentativo (penalita' del tentativo);
          'missione'  -- per sempre: blocco confermato da fermo o guasto fisico."""
        if durata == 'missione':
            self.prm_graph.mark_edge_invalid(a, b)
            return
        prm_graph_module.mark_edge_blocked_soft(self.prm_graph, a, b)
        if durata == 'tentativo':
            self.prm_graph.edge_penalty_persistent[(min(a, b), max(a, b))] = self.prm_graph._ATTEMPT_MARK

    def _strada_fatta(self, x0, y0):
        """Il tratto camminato da (x0, y0) alla posizione di adesso, per la ritirata. Restituisce
        la posizione di adesso."""
        ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
        if not hasattr(self.env, '_traveled_arcs'):
            self.env._traveled_arcs = []
        self.env._traveled_arcs.append((x0, y0, ax, ay))
        return ax, ay

    def _sosta(self, x, y, label):
        """La posizione (x, y) diventa il nodo del robot (vedi add_stop_node)."""
        self.robot_node_id = self.prm_graph.add_stop_node(
            x, y, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M, label=label)
        self.prm_graph.set_robot_node(self.robot_node_id, 0.0)

    def _ventaglio(self):
        """Le 24 direzioni d'uscita da qui (vedi arcVerification.is_trapped): (trappola?,
        direzioni utilizzabili, totale, testo per il log)."""
        trapped, fan = arcVerification.is_trapped((self.robot_x, self.robot_y), self.robot_yaw, self.snap)
        n_us, fan_txt = arcVerification.describe_escape_fan(fan)
        return trapped, n_us, len(fan), fan_txt

    def _esci_dalla_trappola(self, tratto, messaggio):
        """Nessuna direzione d'uscita: scartare archi non serve, il problema e' il posto. Si
        torna indietro con GraphNav (CONTINUA) o il tentativo finisce (FALLITO)."""
        MISSION_STATS['traps_detected'] += 1
        print(messaggio)
        if self._ritorno(tratto, f"dal nodo {self.robot_node_id}"):
            return CONTINUA
        return FALLITO

    def _ritorno(self, block_arc, label, skip_if_graph_cannot_help=False):
        """Ritorno con GraphNav (vedi _trap_backtrack) invece che a zampe all'indietro: si torna
        di un waypoint, si riprova a pianificare da li', e cosi' via fino a
        GRAPHNAV_MAX_BACKTRACK_HOPS; la ritirata a zampe resta dentro come ultima rete.
        True se si riparte con un percorso nuovo (contatori del giro azzerati), False se il
        tentativo deve finire."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()
        esc = recovery._trap_backtrack(
            self.trap_state, block_arc,
            self.env, self.prm_graph, self.goal_id, self.recordingInterface, self.robot_state_client,
            self.command_client, self.global_map, self.mobility_kwargs, self.pts_up, self.obstacle_mask_updated,
            label=label, skip_if_graph_cannot_help=skip_if_graph_cannot_help)
        if not esc['ok']:
            return False
        self.robot_node_id = esc['robot_node']
        self._usa_percorso(esc['ids'])
        self.early_replans = 0
        self.retreats_in_a_row = 0
        self.rotation_discards = 0
        self.best_goal_dist = float('inf')
        return True

    def _fronte_fermo_sull_obiettivo(self):
        """Il fronte si ferma per un ostacolo (o rugosita', pendenza) a ridosso dell'obiettivo
        (entro REFUSED_GOAL_BLOCK_M) o oltre la fine del percorso, nel prolungamento per il
        muso: nessun percorso verso questo obiettivo puo' passare, il problema e' il punto."""
        if self.frontier['reason'] not in ('ostacolo', 'rugosita', 'pendenza'):
            return False
        sx, sy = self.frontier['stop_xy']
        return (self.frontier['dist'] >= self.frontier['path_len']
                or float(np.hypot(sx - self.target_x, sy - self.target_y)) <= P.REFUSED_GOAL_BLOCK_M)

    def _riscegli_obiettivo(self, perche, motivo_esclusione):
        """
        Esclude l'obiettivo corrente (per la missione, vedi _refuse_goal) e ne sceglie un altro
        nella cella, raggiungibile dal robot. CONTINUA se c'e' un nuovo obiettivo con un
        percorso; None se non ce n'e' (l'obiettivo resta escluso, il chiamante prosegue).
        Al massimo MAX_BLOCK_REPLANS volte per tentativo.
        """
        self.goal_repicks += 1
        self.need_refresh = True          # piano cambiato: vedi REFRESH A EVENTI
        MISSION_STATS['goal_repicks'] += 1
        print(f"[TARGET] {perche} Ne cerco un altro nella cella "
              f"({self.goal_repicks}/{P.MAX_BLOCK_REPLANS}).")
        self.tried_goals.append((float(self.target_x), float(self.target_y)))
        goal._refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                     motivo_esclusione)
        # Fino a GOAL_REPICK_CANDIDATES candidati: il primo puo' non avere un percorso nel grafo.
        # Se nessuno ce l'ha, obiettivo e percorso tornano quelli di prima (escluso per le scelte
        # future, ma coerente con il percorso): prima restava il NUOVO obiettivo con il percorso
        # VECCHIO, verso il punto appena escluso.
        old = (self.target_x, self.target_y, self.goal_id)
        new_ids = None
        for _k in range(P.GOAL_REPICK_CANDIDATES):
            new_tx, new_ty, _vs, _rs = goal.find_best_point_in_cell(
                self.robot_x, self.robot_y, self.env, self.target_row, self.target_col, self.pts_up, self.cells_obs_up,
                self.sampler, global_map=self.global_map, exclude=self.tried_goals,
                reachable=self._reachable_from_robot)
            if new_tx is None or new_ty is None:
                print("[TARGET] Nessun altro punto valido in questa cella.")
                break
            goal_id = self.prm_graph.add_stop_node(
                new_tx, new_ty, self.global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
                label=f"obiettivo riscelto cella ({self.target_row},{self.target_col})",
                is_robot=False)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, goal_id)
            if new_ids is not None:
                self.target_x, self.target_y, self.goal_id = new_tx, new_ty, goal_id
                break
            print(f"[TARGET] Nessun percorso nel grafo verso ({new_tx:.2f}, {new_ty:.2f}): provo il "
                  f"punto successivo.")
            self.tried_goals.append((float(new_tx), float(new_ty)))
        if new_ids is None:
            self.target_x, self.target_y, self.goal_id = old
            print("[TARGET] Nessun nuovo obiettivo con un percorso: resto sul piano di prima.")
            return None
        self.best_goal_dist = float('inf')    # nuovo obiettivo: progressi da capo
        self._usa_percorso(new_ids)
        print(f"[REPLAN] Verso il nuovo obiettivo ({self.target_x:.2f}, "
              f"{self.target_y:.2f}): {len(self.path_waypoints)} waypoint.")
        return CONTINUA

    def _guardie(self):
        """Limiti del ciclo: troppi giri (MAX_LOOP_ITERATIONS), caduta, troppi giri fermi."""
        self.loop_iter += 1
        self.t_iter = time.perf_counter()
        if self.loop_iter > P.MAX_LOOP_ITERATIONS:
            print(f"[STALLO] {P.MAX_LOOP_ITERATIONS} giri del ciclo di navigazione senza arrivare: "
                  f"interrompo il tentativo sulla cella ({self.target_row},{self.target_col}).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO

        # Batteria sotto soglia: il tentativo si ferma qui, la missione chiude e rientra
        # (vedi Missione.esplora).
        if _rientro_batteria():
            print(f"[BATTERIA] Interrompo il tentativo sulla cella ({self.target_row},"
                  f"{self.target_col}) per il rientro a batteria bassa.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO

        if movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                   "durante il ciclo di navigazione") is not None:
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            recovery.retreat_along_traveled_path(self.env, self.robot_state_client, self.command_client,
                                        mobility_kwargs=self.mobility_kwargs)
            return FALLITO

        self.robot_x, self.robot_y, self.robot_z, robot_quat = spotUtils.getPosition(self.robot_state_client)

        self.robot_yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                               1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))

        # --- Guardia generica anti-stallo: quanti giri di fila senza spostarsi.
        if self.last_xy is not None and float(np.hypot(self.robot_x - self.last_xy[0], self.robot_y - self.last_xy[1])) < P.IDLE_MOVE_M:
            self.idle_loops += 1
        else:
            self.idle_loops = 0
        # I rifiuti di rotazione si azzerano solo quando il robot si AVVICINA davvero
        # all'obiettivo (20 cm meglio del meglio raggiunto finora), non quando si sposta e
        # basta: non potendo girarsi, il pianificatore puo' portare il robot un nodo piu' in
        # la' dalla parte sbagliata e riprovare da li', e cosi' via, allontanandosi con il
        # contatore sempre a zero (test_e2e_loop.py, scenario S11). Muoversi non e' fare
        # progressi.
        d_goal_now = float(np.hypot(self.robot_x - self.target_x, self.robot_y - self.target_y))
        if d_goal_now < self.best_goal_dist - 0.20:
            self.best_goal_dist = d_goal_now
            self.rotation_discards = 0
        self.last_xy = (self.robot_x, self.robot_y)
        if self.idle_loops >= P.MAX_IDLE_LOOPS:
            MISSION_STATS['idle_loops'] += self.idle_loops
            print(f"[STALLO] {self.idle_loops} giri di ciclo consecutivi senza spostarsi di "
                  f"{P.IDLE_MOVE_M:.2f} m, con il robot in ({self.robot_x:.2f}, {self.robot_y:.2f}). "
                  f"Qualunque sia il motivo, continuare a girare qui non porta da nessuna "
                  f"parte: rinuncio a entrare in ({self.target_row},{self.target_col}) da questo lato.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO

    def _scansione_e_grafo(self):
        """Scansione fresca, mappe globali, dati locali nel PRM ed eventuale
        ripianificazione (REFRESH A EVENTI); si esce dal ciclo se non c'e' piu' percorso."""
        esito = self._scansione_fresca()
        if esito is not None:
            return esito
        touched_nodes = self._aggiorna_grafo()
        if touched_nodes:
            self._ripianifica(touched_nodes)
        if not self.path_waypoints:
            return INTERROMPI

    def _scansione_fresca(self):
        """Scansione, elaborazione, mappe globali, dati locali nel PRM, fotografia per il
        fronte e mappa dello spazio libero. FALLITO se la scansione non arriva."""
        # 1. SCANSIONE FRESCA, prima di tutto il resto
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[ERROR] Rilevamento locale fallito durante il tracking. Arresto precauzionale.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            return FALLITO
        self.t_scan = time.perf_counter()

        self.grids_data_up, main_proto_up = scan.grids_data, scan.proto
        self.lg_proto_up = main_proto_up
        self.pts_up, self.cells_obs_up = scan.pts, scan.od
        self.terrain_up, self.valid_up = scan.terrain_raw, scan.valid
        self.num_x_up, self.num_y_up, self.cell_size_up = scan.num_x, scan.num_y, scan.cell_size
        self.grid_origin_x_up, self.grid_origin_y_up = self.local_grid.grid_origin(scan)

        self.local_grid.process_scan(scan, self.robot_x, self.robot_y, self.robot_yaw,
                                ground_label="scan-passo")
        self.footprint_mask_2d = scan.footprint_mask
        self.terrain_real_up, self.is_valid_up, self.unwritten_up = scan.terrain, scan.is_valid, scan.unwritten
        self.grad_up, self.rough_up, self.obstacle_mask_updated = scan.grad, scan.rough, scan.obstacle_mask

        self.t_proc = time.perf_counter()
        _gm_core_up = spotGrid.fold_scan_into_global_maps(
            self.global_map, self.global_terrain_map, self.pts_up, self.terrain_real_up, self.obstacle_mask_updated,
            self.is_valid_up, self.cells_obs_up, (self.num_y_up, self.num_x_up), (self.robot_x, self.robot_y, self.robot_yaw))
        self.t_maps = time.perf_counter()

        # 2. Dati locali freschi nel PRM, poi eventuale ripianificazione dal nodo attuale
        grad_up_2d = self.grad_up.reshape((self.num_y_up, self.num_x_up))
        self.rough_up_2d = self.rough_up.reshape((self.num_y_up, self.num_x_up))
        self.terrain_up_2d = self.terrain_real_up.reshape((self.num_y_up, self.num_x_up))

        # I dati locali si iniettano sempre: costa niente (salva i riferimenti agli array) e
        # servono comunque a sample_terrain_between_points per il gait del tratto che si sta
        # per percorrere.
        self.prm_graph.update_local_grid_data(
            grad_values_2d=grad_up_2d,
            rough_values_2d=self.rough_up_2d,
            terrain_values_2d=self.terrain_up_2d,
            valid_2d=spotGrid.slope_usable_mask(self.is_valid_up, self.cells_obs_up, (self.num_y_up, self.num_x_up)),
            grid_origin_x=self.grid_origin_x_up,
            grid_origin_y=self.grid_origin_y_up,
            cell_size=self.cell_size_up
        )

        # Fotografia della scansione appena fatta: serve gia' qui -- allo scarto preliminare
        # degli archi nel refresh (vedi prm_graph.LIVE_PREFILTER_OD_M), a quello dei nodi
        # negli ostacoli -- e poi al fronte sicuro. Va creata UNA volta sola per scansione:
        # make_grid_snapshot aggiorna anche il filtro temporale degli ostacoli.
        self.snap = arcVerification.make_grid_snapshot(
            self.cells_obs_up, self.rough_up, self.is_valid_up, self.valid_up, self.terrain_real_up,
            self.num_x_up, self.num_y_up, self.grid_origin_x_up, self.grid_origin_y_up, self.cell_size_up,
            self.robot_x, self.robot_y, unwritten=self.unwritten_up, robot_yaw=self.robot_yaw)

        # Mappa dello spazio libero: QUI, non insieme a global_map.update, perche' serve la
        # obstacle_distance filtrata che vive nella fotografia (vedi spotGrid.update_clearance_map).
        # Si alimenta solo dal ciclo di navigazione, cosi' tutte le osservazioni passano per
        # lo stesso filtro.
        spotGrid.update_clearance_map(self.global_map, self.pts_up, self.snap, self.is_valid_up, self.unwritten_up,
                              _gm_core_up, (self.robot_x, self.robot_y))

    def _aggiorna_grafo(self):
        """Archi rivalutati sui dati nuovi (solo dopo un movimento o un evento), nodi negli
        ostacoli fuori, rifiuti di rotazione ancora validi. I nodi toccati dal refresh."""
        # Lo scarto preliminare degli archi legge la obstacle_distance FILTRATA
        # (mediana su 3 scansioni + isteresi), la stessa del fronte.
        self.prm_graph.set_live_obstacles(self.snap.obstacle_dist, self.snap.seen, self.snap.origin_x, self.snap.origin_y,
                                     self.snap.cell_size, self.robot_x, self.robot_y, self.robot_yaw)

        # --- REFRESH A EVENTI ---------------------------------------------------------
        # Pesi e pendenze degli archi cambiano solo quando cambiano i dati sotto di essi,
        # cioe' quando la finestra si muove: a robot fermo un refresh ricalcolerebbe gli
        # stessi numeri, e circa meta' delle scansioni sono riscansioni da fermo (conferme
        # di blocco, giri senza movimento).
        # Si rinfresca solo quando serve: alla fine di un MOVIMENTO (completato,
        # interrotto o fallito, anche se ha attraversato piu' nodi e archi del PRM) e dopo
        # gli EVENTI che cambiano il piano (blocco confermato, arco scartato,
        # ripianificazione, obiettivo riscelto, ritorno con GraphNav, caduta).
        # QUESTO NON TOCCA LA SICUREZZA: la rete di protezione non sono i veti del grafo ma
        # il FRONTE SICURO, che si ricalcola da zero a ogni scansione (0.5 ms) e a ~5 Hz
        # durante il movimento nel thread. Rimandare il refresh ritarda solo il momento in
        # cui il PIANIFICATORE viene a sapere di un ostacolo nuovo; se il robot e' fermo non
        # c'e' nulla di nuovo da sapere, e se qualcosa entra davvero nel campo il fronte si
        # accorcia, la decisione diventa 'wait' o 'blocked' -- che sono eventi, quindi il
        # refresh scatta comunque. Il caso scoperto si richiude da solo.
        if self.need_refresh or getattr(self.prm_graph, 'needs_refresh', False):
            self.prm_graph.needs_refresh = False
            touched_nodes = self.prm_graph.refresh_local_edge_weights(
                global_map=self.global_map, edge_safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
            self.need_refresh = False
            MISSION_STATS['prm_refresh'] += 1
            n_live = (getattr(self.prm_graph, 'live_veto_last', 0)
                      + getattr(self.prm_graph, 'live_prune_last', 0))
            MISSION_STATS['live_arc_vetoes'] = MISSION_STATS.get('live_arc_vetoes', 0) + n_live
            if n_live:
                print(f"[ARCHI] {n_live} archi fuori dal grafo per la scansione dal vivo "
                      f"({self.prm_graph.live_veto_last} rivalutati, {self.prm_graph.live_prune_last} "
                      f"oltre il raggio di refresh), prima della pendenza.")
        else:
            # Nessun dato nuovo sotto gli archi: si tengono i pesi del giro precedente e
            # non si ripianifica (il blocco sotto gira solo se touched_nodes non e' vuoto).
            touched_nodes = set()
            MISSION_STATS['prm_refresh_skipped'] += 1


        # Nodi DENTRO gli ostacoli: fuori dal grafo (vedi prune_prm_nodes_in_obstacles).
        pruned_nodes = arcVerification.prune_prm_nodes_in_obstacles(self.prm_graph, self.snap)
        if pruned_nodes:
            MISSION_STATS['nodes_pruned'] += len(pruned_nodes)
            print(f"[NODI] {len(pruned_nodes)} nodi scartati: meno di "
                  f"{P.PRM_NODE_MIN_CLEARANCE_M:.2f} m di spazio libero dove li vediamo "
                  f"(primi: {pruned_nodes[:8]}).")

        # Archi gia' rifiutati per rotazione DA QUESTO NODO: restano fuori finche' il robot
        # non si sposta. Lo scarto e' soft, quindi refresh_local_edge_weights li rimetterebbe
        # in gioco a ogni giro, e il robot potrebbe restare fermo alternando due archi
        # rifiutati senza mai arrivare alla ritirata. Il robot puo' restare agganciato allo
        # stesso nodo anche dopo essersi spostato, quindi lo scarto vale solo finche' il robot
        # e' dove la rotazione e' stata rifiutata.
        for (from_id, to_id), n_fails in self.rotation_fails.items():
            fx, fy = self.rotation_fail_xy.get((from_id, to_id), (self.robot_x, self.robot_y))
            if (from_id == self.robot_node_id and n_fails > 0
                    and float(np.hypot(self.robot_x - fx, self.robot_y - fy)) <= prm_graph_module.ANCHOR_ON_NODE_M):
                prm_graph_module.mark_edge_blocked_soft(self.prm_graph, from_id, to_id)
        return touched_nodes

    def _ripianifica(self, touched_nodes):
        """Dijkstra dal nodo del robot all'obiettivo sul grafo aggiornato; il percorso cambia
        solo se il nuovo costa almeno il 10% in meno. Senza percorso, nuovo obiettivo se e'
        isolato l'obiettivo (REPICK_ON_REPLAN_FAIL)."""
        current_plan_ids = [self.robot_node_id] + [w[0] for w in self.path_waypoints]
        chosen_path_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id,
                                                       current_path_ids=current_plan_ids, margin=0.10)

        if chosen_path_ids is None:
            n_free = self.prm_graph.reachable_count(self.robot_node_id, P.REPLAN_FAIL_ROBOT_FREE_NODES)
            if (P.REPICK_ON_REPLAN_FAIL and self.goal_repicks < P.MAX_BLOCK_REPLANS
                    and n_free >= P.REPLAN_FAIL_ROBOT_FREE_NODES):
                # Dal robot il grafo va ancora da qualche parte: e' l'obiettivo a essere
                # rimasto isolato. Se ne cerca subito un altro con un percorso; se non
                # c'e', resta il piano di prima come sotto.
                MISSION_STATS['replan_fail_repicks'] = MISSION_STATS.get('replan_fail_repicks', 0) + 1
                self._riscegli_obiettivo(
                    f"Ripianificazione fallita: nessun percorso da {self.robot_node_id} all'obiettivo "
                    f"({self.target_x:.2f}, {self.target_y:.2f}) nel grafo aggiornato, mentre "
                    f"dal robot il grafo prosegue: e' isolato l'obiettivo.",
                    "nessun percorso nel grafo aggiornato")
            else:
                print(f"[REPLAN FALLITO] Nessun percorso esistente da {self.robot_node_id} a "
                      f"{self.goal_id} nel grafo aggiornato. Mantengo il percorso precedente: il "
                      f"fronte sicuro decide comunque fin dove si puo' avanzare.")
        elif chosen_path_ids != current_plan_ids:
            self._usa_percorso(chosen_path_ids)
            print(f"[REPLAN] Percorso aggiornato ({len(touched_nodes)} nodi locali rivalutati): "
                  f"{len(self.path_waypoints)} waypoint rimanenti.")

            self.replan_counter += 1
            MISSION_STATS['replan_count'] += 1
            replan_save_path = os.path.join(
                self.mission_folder, f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_replan_{self.replan_counter}.png"
            ) if self.mission_folder else None

            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(self.target_x, self.target_y), iteration=self.iteration, env=self.env,
                save_path=replan_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up,
                include_diagnostics=False  # leggero: vista principale e mappa globale
            )
        else:
            print(f"[REPLAN STABILE] Percorso invariato da {self.robot_node_id} a {self.goal_id} "
                  f"({len(touched_nodes)} nodi locali rivalutati): l'alternativa non "
                  f"supera la soglia di stabilita'. Proseguo sul piano corrente.")

    def _fronte_e_decisione(self):
        """Scorciatoia, fronte sicuro sul percorso e decisione (arcVerification.decide_next_move):
        muoversi, nodo raggiunto, arrivati, aspettare o blocco confermato."""
        self.t_prm = time.perf_counter()

        # 3. FRONTE SICURO sulla scansione appena fatta ---------------------------------
        # La scorciatoia rispetta gli archi gia' scartati: vedi shortcut_index.
        k_short = arcVerification.shortcut_index((self.robot_x, self.robot_y), self.path_waypoints, self.snap, self.global_map,
                                 prm=self.prm_graph, robot_node_id=self.robot_node_id)
        if k_short > 0:
            skipped = [w[0] for w in self.path_waypoints[:k_short]]
            self.path_waypoints = self.path_waypoints[k_short:]
            print(f"[SCORCIATOIA] Tratto diretto confermato libero verso il nodo {self.path_waypoints[0][0]}: "
                  f"salto {len(skipped)} nodi intermedi {skipped}.")
            MISSION_STATS['shortcuts'] += 1
        self.polyline = [(self.robot_x, self.robot_y)] + [(x, y) for _, x, y in self.path_waypoints]
        self.frontier = arcVerification.compute_safe_frontier(self.polyline, self.snap)
        print(arcVerification.describe_frontier(self.frontier))

        self.path_version = None
        if self.verification_tracker is not None:
            self.path_version = self.verification_tracker.update_path(self.polyline[1:])

        in_target_cell = (tuple(int(v) for v in self.env.get_cell_from_world(self.robot_x, self.robot_y))
                          == (int(self.target_row), int(self.target_col)))
        # Appena fuori dalla cella vale come dentro, per l'arrivo per tolleranza (vedi
        # ARRIVAL_OUTSIDE_CELL_M). Il 22-10 16:04: "Mancano 0.06 m ... ancora fuori dalla
        # cella (2,7)", con il robot fuori di 0.01 m.
        robot_border = environmentMap.cell_border_distance(self.env, self.target_row, self.target_col, self.robot_x, self.robot_y)
        near_goal = (not in_target_cell and robot_border is not None
                     and -robot_border <= P.ARRIVAL_OUTSIDE_CELL_M)
        self.decision = arcVerification.decide_next_move((self.robot_x, self.robot_y), self.path_waypoints, self.frontier, self.no_progress,
                                    in_target_cell=in_target_cell or near_goal)
        if self.decision.get('final'):
            MISSION_STATS['final_steps'] = MISSION_STATS.get('final_steps', 0) + 1
            print(f"[PASSO FINALE] Il fronte permette solo {self.decision['dist']:.2f} m, sotto il "
                  f"passo minimo di {P.MIN_PARTIAL_STEP_M:.2f} m, ma bastano: dopo il passo "
                  f"l'obiettivo e' a {self.frontier['path_len'] - self.decision['dist']:.2f} m, entro "
                  f"la tolleranza d'arrivo di {P.GOAL_REACHED_TOLERANCE_M:.2f} m.")
        if self.decision['kind'] == 'arrived' and near_goal:
            self.arrived_near_goal = True
            MISSION_STATS['arrivals_near_goal'] += 1
            print(f"[OBIETTIVO] Il robot e' fuori dalla cella ({self.target_row},{self.target_col}) di "
                  f"{-robot_border:.2f} m, a {self.frontier['path_len']:.2f} m dall'obiettivo: vale come "
                  f"arrivo (fuori al massimo di {P.ARRIVAL_OUTSIDE_CELL_M:.2f} m, il muso e' dentro).")
        if (self.decision['kind'] in ('wait', 'blocked') and not in_target_cell and not near_goal
                and self.frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M):
            print(f"[OBIETTIVO] Mancano {self.frontier['path_len']:.2f} m, entro la tolleranza di "
                  f"{P.GOAL_REACHED_TOLERANCE_M:.2f} m, ma il robot e' ancora fuori dalla cella "
                  f"({self.target_row},{self.target_col}): non vale come arrivo, proseguo come per un "
                  f"fronte bloccato.")
        self.next_node_id, next_x, next_y = self.path_waypoints[0]
        self.keep_heading = False     # sempre False (niente traverso): resta nei dati salvati e in navigate_to

        # Il pacchetto si salva qui, prima delle fasi che possono chiudere il giro (rotazione,
        # attesa, blocco): altrimenti quelle scansioni non finirebbero su disco.
        self._salva_pacchetto_scansione()

    def _salva_pacchetto_scansione(self):
        """Tutto quello che serve a rivedere questa scansione dopo (scans/*.npz): layer della
        griglia, fotografia filtrata, percorso, fronte e decisione. Vedi SAVE_SCAN_BUNDLES."""
        # Un rifiuto di rotazione si ricalcola offline da obstacle_distance, robot_xyz,
        # robot_yaw e decision_target.
        if P.SAVE_SCAN_BUNDLES and self.mission_folder is not None:
            gf = self.local_grid.last_ground_filter
            shape_up = (self.num_y_up, self.num_x_up)
            extra = {}
            if P.SAVE_EXTRA_LAYERS:
                t_x = time.perf_counter()
                for name, lay in self.local_grid.extra_layers(P.SAVE_EXTRA_LAYERS).items():
                    extra[name] = lay['values']
                    extra[f'{name}_origin'] = lay['origin']
                    extra[f'{name}_cell'] = lay['cell']
                timing_add('scansione', time.perf_counter() - t_x)
            save_npz_async(
                os.path.join(self.mission_folder, "scans",
                             f"scan_it{self.iteration:03d}_cell{self.target_row}_{self.target_col}_{self.loop_iter:03d}.npz"),
                time=time.time(), iteration=self.iteration, cell=np.array([self.target_row, self.target_col]),
                loop_iter=self.loop_iter,
                terrain_raw=np.asarray(self.terrain_up, dtype=np.float32).reshape(shape_up),
                terrain_valid_raw=np.asarray(self.valid_up, dtype=np.float32).reshape(shape_up),
                obstacle_distance=np.asarray(self.cells_obs_up, dtype=np.float32).reshape(shape_up),
                # Il layer su cui il fronte DECIDE (mediana sulle ultime letture + isteresi,
                # vedi arcVerification._ObstacleHistory) e la maschera del latch. Senza questi
                # due un rigioco offline non puo' riprodurre i blocchi che nascono dalle
                # letture del thread durante il movimento. Stesso orientamento degli altri:
                # (num_y, num_x), riga = y.
                obstacle_distance_filtered=np.asarray(self.snap.obstacle_dist, dtype=np.float32),
                blocked_latch=(np.zeros(shape_up, bool) if self.snap.blocked_latch is None
                               else np.asarray(self.snap.blocked_latch, dtype=bool)),
                terrain_corrected=np.asarray(self.terrain_real_up, dtype=np.float32).reshape(shape_up),
                is_valid=np.asarray(self.is_valid_up, dtype=bool).reshape(shape_up),
                roughness=np.asarray(self.rough_up, dtype=np.float32).reshape(shape_up),
                gradient=np.asarray(self.grad_up, dtype=np.float32).reshape(shape_up),
                obstacle_mask=np.asarray(self.obstacle_mask_updated, dtype=np.int8).reshape(shape_up),
                footprint_mask=(np.zeros(shape_up, bool) if self.footprint_mask_2d is None
                                else np.asarray(self.footprint_mask_2d, dtype=bool).reshape(shape_up)),
                grid_origin=np.array([self.grid_origin_x_up, self.grid_origin_y_up]), cell_size=self.cell_size_up,
                robot_xyz=np.array([self.robot_x, self.robot_y, self.robot_z]), robot_yaw=self.robot_yaw,
                ground_z=np.nan if gf.get('ground_z') is None else gf['ground_z'],
                ground_n_above=gf.get('n_above', 0),
                mission_z0=mission_z0(),
                terrain_raw_zero=np.nan if spotGrid.raw_zero(self.grids_data_up) is None else spotGrid.raw_zero(self.grids_data_up),
                terrain_scale=np.nan if not spotGrid.raw_scale(self.grids_data_up) else spotGrid.raw_scale(self.grids_data_up),
                unwritten=np.asarray(self.unwritten_up, dtype=bool).reshape(shape_up),
                polyline=np.array(self.polyline), path_node_ids=np.array([w[0] for w in self.path_waypoints]),
                robot_node_id=self.robot_node_id, goal_xy=np.array([self.target_x, self.target_y]),
                frontier_dist=self.frontier['dist'], frontier_reason=self.frontier['reason'],
                frontier_stop_xy=np.array(self.frontier['stop_xy']),
                frontier_value=np.nan if self.frontier['value'] is None else self.frontier['value'],
                frontier_od_robot=self.frontier['od_robot'], frontier_half_along=self.frontier['half_along'],
                frontier_clearance=self.frontier['clearance_nominal'], frontier_path_len=self.frontier['path_len'],
                allowed_advance=arcVerification.allowed_advance(self.frontier),
                keep_heading=self.keep_heading, no_progress=self.no_progress,
                **extra,
                decision_kind=self.decision['kind'],
                decision_target=np.array(self.decision.get('target', (np.nan, np.nan)), dtype=np.float64))
            MISSION_STATS['scan_bundles'] += 1

    def _rotazione(self):
        """Se il passo deciso richiede di girarsi dove non si puo': avanzare nel cono,
        arretrare dritti, riconoscere la trappola o scartare il tratto."""
        # 4. ROTAZIONE -------------------------------------------------------------------
        # Il muso punta SEMPRE il punto da raggiungere. Se la rotazione non e' possibile NON
        # si cammina di traverso (vedi ROTAZIONE E ARRETRAMENTO in parametri.py). Nell'ordine:
        # avanzare dentro il cono di rotazione possibile, arretrare dritti se da li' la
        # rotazione diventa possibile, riconoscere una trappola, scartare il tratto.
        if self.decision['kind'] == 'move':
            rot = arcVerification.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), self.decision['target'],
                                self.frontier['od_robot'], snap=self.snap)
            if not rot['turn']:
                MISSION_STATS['rotation_refusals'] += 1
                edge_key = (self.robot_node_id, self.next_node_id)
                self.rotation_fails[edge_key] = self.rotation_fails.get(edge_key, 0) + 1
                self.rotation_fail_xy[edge_key] = (self.robot_x, self.robot_y)
                margin_txt = (f"margine {rot['margin']:+.2f} m sulle celle spazzate"
                              if rot['margin'] is not None else f"motivo: {rot['why']}")
                print(f"[ROTAZIONE] Servono {rot['dyaw_deg']:+.0f} gradi verso il nodo "
                      f"{self.next_node_id}, ma il corpo toccherebbe ({margin_txt}; "
                      f"obstacle_distance sotto il robot {self.frontier['od_robot']:.2f} m). "
                      f"Non cammino di traverso: tentativo "
                      f"{self.rotation_fails[edge_key]}/{P.MAX_ROTATION_FAILS_SAME_EDGE}.")

                esito = self._avanza_nel_cono(rot)
                if esito is None:
                    esito = self._arretra_per_ruotare(rot, edge_key)
                if esito is not None:
                    return esito

                # Ne' il cono ne' l'arretramento: e' una trappola o solo un arco sbagliato?
                # Se e' una trappola non ha senso scartare archi uno dopo l'altro: si torna
                # indietro subito.
                trapped_rot, n_us_r, n_fan_r, fan_txt_r = self._ventaglio()
                if trapped_rot:
                    return self._esci_dalla_trappola(
                        (self.robot_node_id, self.next_node_id),
                        f"[TRAPPOLA] Non posso girarmi verso {self.next_node_id} e nessuna delle "
                        f"{n_fan_r} direzioni del giro e' insieme percorribile e "
                        f"raggiungibile girandosi. Le piu' larghe: {fan_txt_r}. Ripercorro il "
                        f"tratto da cui sono venuto.")
                print(f"[ROTAZIONE] Direzioni utilizzabili da qui: {n_us_r}/{n_fan_r} "
                      f"({fan_txt_r}).")
                # Limite per TENTATIVO, non per coppia di nodi (vedi MAX_ROTATION_DISCARDS).
                self.rotation_discards += 1
                if self.rotation_discards > P.MAX_ROTATION_DISCARDS:
                    print(f"[ROTAZIONE] Gia' {self.rotation_discards - 1} tratti scartati in questo "
                          f"tentativo perche' la rotazione era impossibile, e sono sempre qui: "
                          f"non e' l'arco, e' il posto. Ripercorro il tratto da cui sono venuto.")
                    if self._ritorno((self.robot_node_id, self.next_node_id),
                                     f"dal nodo {self.robot_node_id} (troppi rifiuti di rotazione)"):
                        return CONTINUA
                    return FALLITO
                print(f"[ROTAZIONE] Scarto il tratto {self.robot_node_id}->{self.next_node_id}: "
                      f"richiede di girarsi dove non c'e' spazio DA QUI. Ripianifico.")
                # Scarto NON permanente: un rifiuto di rotazione dipende dalla POSA del robot,
                # non dall'arco -- lo stesso arco, affrontato da un altro punto o con un altro
                # muso, puo' essere percorribilissimo. Permanente resta solo cio' che e'
                # confermato da tre scansioni ferme o da un fallimento fisico.
                self._scarta_tratto(self.robot_node_id, self.next_node_id, 'ora')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None:
                    self._usa_percorso(new_ids)
                    return CONTINUA
                print("[ROTAZIONE] Nessuna alternativa nel grafo dal punto in cui sono. "
                      "Mi stacco arretrando e rimando la cella.")
                if self._ritorno((self.robot_node_id, self.next_node_id),
                                 f"dal nodo {self.robot_node_id}", skip_if_graph_cannot_help=True):
                    return CONTINUA
                return FALLITO

    def _cambia_obiettivo_irraggiungibile(self):
        """Il fronte non lascia avanzare: se e' colpa dell'obiettivo (dentro un ostacolo, o fronte
        fermo a ridosso di lui) se ne sceglie un altro. L'esito, o None."""
        # Prima di insistere: l'obiettivo e' ancora raggiungibile sui dati FRESCHI? Il
        # punto viene scelto all'inizio del tentativo, da metri di distanza, quando la
        # cella e' ancora tutta ignota; arrivandoci puo' risultare a ridosso di un muro,
        # e allora nessun numero di scansioni lo rendera' raggiungibile.
        if self.goal_repicks < P.MAX_BLOCK_REPLANS:
            if not arcVerification.goal_still_reachable(self.snap, self.target_x, self.target_y):
                r_g, c_g, in_g = arcVerification._cells(
                    self.snap, np.array([float(self.target_x)]), np.array([float(self.target_y)]))
                od_goal = float(self.snap.obstacle_dist[r_g[0], c_g[0]]) if in_g[0] else float('nan')
                esito = self._riscegli_obiettivo(
                    f"L'obiettivo ({self.target_x:.2f}, {self.target_y:.2f}) ha obstacle_distance "
                    f"{od_goal:.2f} m, sotto i {spotGrid.FRONTIER_CLEARANCE_M:.2f} m che il fronte "
                    f"pretende: non e' raggiungibile da nessuna direzione.",
                    f"spazio libero dal vivo {od_goal:.2f} m")
                if esito is not None:
                    return esito
            elif self._fronte_fermo_sull_obiettivo():
                sx_g, sy_g = self.frontier['stop_xy']
                d_g = float(np.hypot(sx_g - self.target_x, sy_g - self.target_y))
                oltre = self.frontier['dist'] >= self.frontier['path_len']
                esito = self._riscegli_obiettivo(
                    f"Il fronte si ferma per {self.frontier['reason']} in ({sx_g:.2f}, {sy_g:.2f}), a "
                    f"{d_g:.2f} m dall'obiettivo ({self.target_x:.2f}, {self.target_y:.2f})"
                    + (", oltre la fine del percorso, dove andrebbe il muso" if oltre else "")
                    + ": e' l'obiettivo a non andare, non la strada.",
                    f"il fronte si ferma a {d_g:.2f} m da lui ({self.frontier['reason']})")
                if esito is not None:
                    return esito
        return None

    def _ripianifica_subito(self):
        """[SUBITO]: alla prima cella bloccante il tratto si scarta e si cerca un'altra strada,
        senza aspettare la conferma da fermo. L'esito, o None se si resta ad aspettare."""
        # RIPIANIFICA SUBITO, alla PRIMA cella bloccante, invece di restare fermo tre
        # scansioni sullo stesso arco. Il rumore di obstacle_distance e' gia' filtrato
        # (mediana + isteresi in arcVerification), quindi una cella bloccante e'
        # un'informazione da usare: l'arco si scarta e si cerca subito un percorso che
        # passi da un'altra parte. Le scansioni da fermo restano come ultima rete, per
        # dichiarare la cella irraggiungibile.
        if (self.early_replans < P.MAX_EARLY_REPLANS
                and self.frontier['reason'] in ('ostacolo', 'rugosita', 'pendenza')
                and self.path_waypoints):
            poly_now = [(self.robot_x, self.robot_y)] + [(x, y) for _, x, y in self.path_waypoints]
            _, seg_blk = arcVerification.point_along(poly_now, self.frontier['dist'])
            seg_blk = int(seg_blk)
            ea = self.robot_node_id if seg_blk == 0 else self.path_waypoints[seg_blk - 1][0]
            eb = self.path_waypoints[min(seg_blk, len(self.path_waypoints) - 1)][0]
            sx_b, sy_b = self.frontier['stop_xy']

            # Prima di scartare l'ennesimo arco: sono in trappola? Se tutti gli archi del
            # nodo passano accanto allo stesso ostacolo, il blocco e' un LUOGO, non un
            # arco, e scartarli uno per uno non serve. Il ventaglio lo vede in una scansione.
            trapped, n_us, n_fan, fan_txt = self._ventaglio()
            if trapped:
                return self._esci_dalla_trappola(
                    (ea, eb),
                    f"[TRAPPOLA] Nessuna delle {n_fan} direzioni del giro e' insieme "
                    f"percorribile e raggiungibile girandosi. Le piu' larghe: {fan_txt}. "
                    f"Non ho dove andare ne' come girarmi: ripercorro il tratto da cui "
                    f"sono venuto, senza scartare altri archi.")

            self.early_replans += 1
            MISSION_STATS['early_replans'] += 1
            print(f"[SUBITO] {self.frontier['reason']} in ({sx_b:.2f}, {sy_b:.2f}) sul tratto "
                  f"{ea}->{eb}: lo scarto adesso e cerco un altro percorso, senza "
                  f"aspettare la conferma da fermo ({self.early_replans}/{P.MAX_EARLY_REPLANS}). "
                  f"Direzioni utilizzabili da qui: {n_us}/{n_fan}.")
            # Scarto NON permanente: dopo una sola scansione l'arco va tolto dal piano,
            # non condannato per la missione (vedi mark_edge_blocked_soft).
            self._scarta_tratto(ea, eb, 'ora')
            # Ma se e' la SECONDA volta sullo stesso tratto, lo scarto "per ora" non e'
            # bastato: il refresh lo ha rimesso nel grafo (lo scarto preliminare guarda una
            # scansione a 0.25 m, il fronte la mediana di tre con l'isteresi fino a 0.29),
            # e il piano e' tornato quello di prima. Fuori dal piano per il resto del
            # tentativo, come i ritorni e [ARRIVO]. Il 28-10 14:23, 8 [SUBITO] su 14 erano
            # ripetizioni dello stesso tratto (197->569 quattro volte di fila).
            key_s = (min(ea, eb), max(ea, eb))
            if key_s in self.subito_tratti:
                self._scarta_tratto(ea, eb, 'tentativo')
                MISSION_STATS['early_replans_repeated'] = MISSION_STATS.get('early_replans_repeated', 0) + 1
                print(f"[SUBITO] Il tratto {ea}->{eb} era gia' stato scartato in questo tentativo e "
                      f"il piano ci e' tornato: fuori dal piano per il resto del tentativo.")
            self.subito_tratti.add(key_s)
            new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None and len(new_ids) > 1:
                self._usa_percorso(new_ids)
                print(f"[SUBITO] Percorso alternativo con {len(self.path_waypoints)} waypoint, "
                      f"dal punto in cui sono.")
                return CONTINUA
            print("[SUBITO] Il grafo non offre alternative da qui: resto sul percorso e "
                  "lascio decidere il fronte.")
        return None

    def _avanza_nel_cono(self, rot):
        """Rotazione impossibile: si avanza dentro il cono di rotazione concesso, se avvicina
        al percorso (CONTINUA), altrimenti None."""
        # --- 4a. PRIMA DI TUTTO: posso ANDARE AVANTI dentro il cono di rotazione che
        # mi e' concesso? Spesso andare dritti, o quasi, avvicina all'obiettivo piu'
        # di qualunque arretramento (vedi plan_cone_move).
        cone_goal, _ = arcVerification.point_along(self.polyline, P.CONE_PATH_LOOKAHEAD_M)
        cone = arcVerification.plan_cone_move((self.robot_x, self.robot_y), self.robot_yaw, self.snap, cone_goal)
        if cone is not None:
            print(f"[CONO] Non posso girarmi di {rot['dyaw_deg']:+.0f} gradi, ma ruotando di "
                  f"{cone['dyaw_deg']:+.0f} posso avanzare {cone['dist']:.2f} m "
                  f"({cone['reason']}) e avvicinarmi di {cone['gain']:.2f} m al percorso "
                  f"({cone_goal[0]:.2f}, {cone_goal[1]:.2f}, {P.CONE_PATH_LOOKAHEAD_M:.1f} m "
                  f"avanti lungo il percorso). Vado, invece di arretrare.")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['cone_moves'] += 1
            cx, cy = cone['target']
            self.need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
            with TrajectoryLog(self.robot_state_client, 'cono', (cx, cy)):
                ok_c, dist_c, _ = movements.navigate_to(
                    cx, cy, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                    get_a_tform_b(self.lg_proto_up.local_grid.transforms_snapshot,
                                  VISION_FRAME_NAME, BODY_FRAME_NAME),
                    should_abort=None, keep_heading=False, **self.mobility_kwargs)
            if dist_c > 0.05:
                self._sosta(*self._strada_fatta(self.robot_x, self.robot_y), "sosta (avanzamento nel cono)")
                self.retreats_in_a_row = 0      # ha fatto strada vera: la catena si rompe
            else:
                print(f"[CONO] Non sono riuscito ad avanzare (percorsi {dist_c:.2f} m).")
            self.no_progress = 0
            return CONTINUA
        return None

    def _arretra_per_ruotare(self, rot, edge_key):
        """Rotazione impossibile: si arretra dritti, se da li' la rotazione diventa possibile
        (CONTINUA), altrimenti None."""
        # --- 4b. Arretrare, ma SOLO se serve a qualcosa (vedi retreat_would_help).
        back = arcVerification.plan_straight_retreat((self.robot_x, self.robot_y), self.robot_yaw, self.snap)
        helps, margin_back = (False, None)
        if back is not None:
            helps, margin_back = arcVerification.retreat_would_help(
                self.snap, (back[0], back[1]), self.robot_yaw, rot['dyaw_deg'])
            if not helps:
                MISSION_STATS['retreats_suppressed'] += 1
                mb = "" if margin_back is None else f" (margine {margin_back:+.2f} m da li')"
                print(f"[ARRETRAMENTO] Potrei arretrare di {back[2]:.2f} m, ma da li' la "
                      f"rotazione di {rot['dyaw_deg']:+.0f} gradi resterebbe impossibile{mb}: "
                      f"non lo faccio, sposterei il problema di 60 cm.")
        if (back is not None and helps
                and self.retreats_in_a_row < P.MAX_STRAIGHT_RETREATS_IN_A_ROW
                and self.rotation_fails[edge_key] < P.MAX_ROTATION_FAILS_SAME_EDGE):
            bx, by, bdist = back
            self.retreats_in_a_row += 1
            print(f"[ARRETRAMENTO] Indietro dritto di {bdist:.2f} m verso "
                  f"({bx:.2f}, {by:.2f}), senza girarmi, per fare spazio alla rotazione "
                  f"({self.retreats_in_a_row}/{P.MAX_STRAIGHT_RETREATS_IN_A_ROW} consecutivi).")
            if self.verification_tracker is not None:
                self.verification_tracker.clear_path()
            MISSION_STATS['straight_retreats'] += 1
            self.need_refresh = True      # fine di un movimento: vedi REFRESH A EVENTI
            with TrajectoryLog(self.robot_state_client, 'arretramento', (bx, by)):
                ok_back, dist_back = movements.move_to_world_point_without_turning(
                    bx, by, "vision", self.command_client, self.robot_state_client, **self.mobility_kwargs)
            if dist_back > 0.05:
                self._sosta(*self._strada_fatta(self.robot_x, self.robot_y), "sosta (arretramento per ruotare)")
            else:
                print(f"[ARRETRAMENTO] Non sono riuscito ad arretrare "
                      f"(percorsi {dist_back:.2f} m).")
            self.no_progress = 0
            return CONTINUA

        if back is None:
            print("[ARRETRAMENTO] Non c'e' spazio per arretrare dritto.")
        elif helps and self.retreats_in_a_row >= P.MAX_STRAIGHT_RETREATS_IN_A_ROW:
            print(f"[ARRETRAMENTO] Gia' {self.retreats_in_a_row} arretramenti di fila senza "
                  f"combinare niente: smetto di arretrare.")
        return None

    def _salva_dati_del_blocco(self):
        """Scansione e percorso del blocco confermato, per guardarlo dopo (BLOCKED_n.npz)."""
        if self.mission_folder is not None:
            try:
                npz_blocked = os.path.join(
                    self.mission_folder,
                    f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED_{self.block_replans + 1}.npz")
                np.savez_compressed(
                    npz_blocked,
                    obstacle_distance=self.snap.obstacle_dist, rough_veto=self.snap.rough_veto,
                    seen=self.snap.seen, terrain=self.snap.terrain,
                    terrain_raw=np.asarray(self.terrain_up).reshape((self.num_y_up, self.num_x_up)),
                    terrain_valid_raw=np.asarray(self.valid_up).reshape((self.num_y_up, self.num_x_up)),
                    obstacle_mask=np.asarray(self.obstacle_mask_updated).reshape((self.num_y_up, self.num_x_up)),
                    roughness=self.rough_up_2d,
                    grid_origin_x=self.snap.origin_x, grid_origin_y=self.snap.origin_y, cell_size=self.snap.cell_size,
                    robot_xy=np.array([self.robot_x, self.robot_y]), robot_yaw=self.robot_yaw,
                    polyline=np.array(self.polyline), frontier_dist=self.frontier['dist'],
                    frontier_reason=self.frontier['reason'], frontier_stop_xy=np.array(self.frontier['stop_xy']),
                    ground_z=np.nan if self.local_grid.last_ground_filter.get('ground_z') is None
                    else self.local_grid.last_ground_filter['ground_z'])
                print(f"[DATA SAVE] Dati del blocco salvati in {os.path.basename(npz_blocked)}")
            except Exception as e:
                print(f"[DATA SAVE] Impossibile salvare i dati del blocco: {e}")

    def _tempi(self):
        """Tempi del giro, per [TEMPO] e il riepilogo."""
        t_front = time.perf_counter()

        timing_add('scansione', self.t_scan - self.t_iter)
        timing_add('elaborazione griglia', self.t_proc - self.t_scan)
        timing_add('mappe globali', self.t_maps - self.t_proc)
        timing_add('PRM e ripianificazione', self.t_prm - self.t_maps)
        timing_add('fronte e decisione', t_front - self.t_prm)
        print(f"[TEMPO] scansione {self.t_scan - self.t_iter:.2f} s | elaborazione {self.t_proc - self.t_scan:.2f} | "
              f"mappe {self.t_maps - self.t_proc:.2f} | PRM {self.t_prm - self.t_maps:.2f} | fronte {t_front - self.t_prm:.2f} "
              f"| decisione: {self.decision['kind']}")

    def _nodo_o_arrivo(self):
        """Nodo del percorso raggiunto, oppure arrivo accettato prima della fine."""
        if self.decision['kind'] == 'reached_node':
            self.robot_node_id = self.next_node_id
            self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
            self.path_waypoints.pop(0)
            self.no_progress = 0
            return CONTINUA

        if self.decision['kind'] == 'arrived':
            print(f"[OBIETTIVO] Il fronte non permette gli ultimi {self.frontier['path_len']:.2f} m "
                  f"({self.frontier['reason']}), entro la tolleranza di {P.GOAL_REACHED_TOLERANCE_M:.2f} m: "
                  f"considero raggiunto l'obiettivo.")
            return INTERROMPI

    def _attesa(self):
        """Il fronte non lascia avanzare: obiettivo ancora raggiungibile? ripianificare subito
        (SUBITO) o aspettare la conferma da fermo."""
        if self.decision['kind'] == 'wait':
            esito = self._cambia_obiettivo_irraggiungibile()
            if esito is None:
                esito = self._ripianifica_subito()
            if esito is not None:
                return esito

            self.no_progress += 1
            print(f"[FRONTE] Non posso avanzare nemmeno di {P.MIN_PARTIAL_STEP_M:.2f} m: riguardo "
                  f"da fermo ({self.no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni).")
            time.sleep(P.FRONTIER_RESCAN_WAIT_S)
            return CONTINUA

    def _blocco(self):
        """Blocco confermato da piu' scansioni ferme: arco scartato, ripianificazione o ritorno."""
        if self.decision['kind'] == 'blocked':
            k = self.decision['segment']
            edge_a = self.robot_node_id if k == 0 else self.path_waypoints[k - 1][0]
            edge_b = self.path_waypoints[min(k, len(self.path_waypoints) - 1)][0]
            sx, sy = self.frontier['stop_xy']
            print(f"[BLOCCO] Confermato da vicino: {P.FRONTIER_BLOCK_CONFIRM_SCANS} scansioni consecutive "
                  f"senza poter avanzare. Motivo: {self.frontier['reason']} in ({sx:.2f}, {sy:.2f}), "
                  f"sul tratto {edge_a}->{edge_b}.")
            MISSION_STATS['frontier_blocks'] += 1
            # Bloccato a ridosso dell'obiettivo (o oltre): e' l'obiettivo a non andare, non la
            # strada. Si esclude e, se si puo', se ne sceglie un altro (sotto, dopo la trappola)
            # invece di scartare il tratto e ripianificare verso lo stesso obiettivo.
            goal_blocked = self._fronte_fermo_sull_obiettivo()
            d_goal = float(np.hypot(sx - self.target_x, sy - self.target_y))
            why_goal = f"blocco confermato a {d_goal:.2f} m da lui ({self.frontier['reason']})"
            if goal_blocked:
                goal._refuse_goal(self.env, self.target_row, self.target_col, self.target_x, self.target_y,
                             why_goal)

            self._salva_dati_del_blocco()

            # Trappola? Se non c'e' nessuna direzione utilizzabile, ripianificare
            # e' inutile per costruzione: il problema non e' l'arco, e' il posto.
            trapped_blk, n_us_b, n_fan_b, fan_txt_b = self._ventaglio()
            print(f"[BLOCCO] Direzioni utilizzabili da qui: {n_us_b}/{n_fan_b} ({fan_txt_b}).")
            if trapped_blk:
                return self._esci_dalla_trappola(
                    (edge_a, edge_b), "[TRAPPOLA] Nessuna via d'uscita girandomi: non scarto altri "
                                      "archi, ripercorro il tratto da cui sono venuto.")

            if goal_blocked and self.goal_repicks < P.MAX_BLOCK_REPLANS:
                esito = self._riscegli_obiettivo(
                    f"Blocco a {d_goal:.2f} m dall'obiettivo ({self.target_x:.2f}, "
                    f"{self.target_y:.2f}): e' l'obiettivo a non andare, non il tratto {edge_a}->{edge_b}.",
                    why_goal)
                if esito is not None:
                    return esito

            # Qui il blocco e' confermato da FRONTIER_BLOCK_CONFIRM_SCANS scansioni ferme:
            # l'invalidazione resta permanente (a differenza di [SUBITO], vedi
            # mark_edge_blocked_soft). Eccezione: sul primo tratto, se il robot
            # non e' sul nodo a cui e' agganciato, il tratto bloccato e' robot->nodo, non
            # l'arco del grafo: si scarta l'arco solo per ora.
            sul_nodo = k > 0 or mission_graph.robot_on_node(self.prm_graph, edge_a, self.robot_x, self.robot_y)
            self._scarta_tratto(edge_a, edge_b, 'missione' if sul_nodo else 'ora')
            self.block_replans += 1
            self.no_progress = 0

            new_ids = None
            if self.block_replans <= P.MAX_BLOCK_REPLANS:
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
            if new_ids is not None:
                self._usa_percorso(new_ids)
                print(f"[REPLAN] Dopo il blocco ({self.block_replans}/{P.MAX_BLOCK_REPLANS}): percorso "
                      f"alternativo con {len(self.path_waypoints)} waypoint, dal punto in cui sono.")
                return CONTINUA

            print(f"[BLOCCO] Nessuna alternativa dal punto in cui sono"
                  + (f" (gia' {P.MAX_BLOCK_REPLANS} ripianificazioni dopo blocchi)."
                     if self.block_replans > P.MAX_BLOCK_REPLANS else ".")
                  + " Mi stacco arretrando e rimando la cella.")
            if self._ritorno((edge_a, edge_b),
                             f"dal nodo {self.robot_node_id}", skip_if_graph_cannot_help=True):
                return CONTINUA

            blocked_save_path = os.path.join(self.mission_folder,
                                             f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_BLOCKED.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=self.frontier['stop_xy'], iteration=self.iteration, env=self.env,
                save_path=blocked_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up
            )
            return FALLITO

    def _esegui_tratti(self):
        """Il movimento deciso: controllo della posa d'arrivo, poi una o piu' gambe sulla
        polilinea confermata dal fronte, con arresto a meta' se il fronte si accorcia."""
        esito = self._controlla_posa_d_arrivo()
        if esito is not None:
            return esito

        # --- Esecuzione: una o piu' GAMBE sulla polilinea gia' confermata dal fronte,
        # senza scansione, refresh ne' ripianificazione in mezzo (vedi AVANZAMENTO
        # MULTINODO in decide_next_move). n_legs = 0 e' un solo passo parziale.
        n_legs = int(self.decision.get('nodes', 1)) if self.decision['full'] else 0
        if n_legs > 1:
            MISSION_STATS['multinode_moves'] += 1
            MISSION_STATS['multinode_nodes'] += n_legs
            print(f"[MULTINODO] Il fronte conferma "
                  f"{arcVerification.allowed_advance(self.frontier):.2f} m: percorro {n_legs} "
                  f"tratti di fila {[w[0] for w in self.path_waypoints[:n_legs]]} senza fermarmi "
                  f"a riscansionare.")
        for leg_i in range(max(1, n_legs)):
            if not self.path_waypoints:
                break
            self.next_node_id, next_x, next_y = self.path_waypoints[0]
            if n_legs == 0:
                move_x, move_y = self.decision['target']
                is_partial_step = True
            else:
                move_x, move_y = next_x, next_y
                is_partial_step = False
            if leg_i > 0:
                vision_tform_body_current = self._prepara_gamba(leg_i, n_legs, move_x, move_y)
                if vision_tform_body_current is None:
                    break
            else:
                vision_tform_body_current = get_a_tform_b(self.lg_proto_up.local_grid.transforms_snapshot,
                                                          VISION_FRAME_NAME, BODY_FRAME_NAME)
            self.no_progress = 0
            self.retreats_in_a_row = 0   # un avanzamento vero rompe la catena di arretramenti
            if is_partial_step:
                print(f"[PASSO PARZIALE] Verso il nodo {self.next_node_id}: avanzo di {self.decision['dist']:.2f} m "
                      f"su {np.hypot(next_x - self.robot_x, next_y - self.robot_y):.2f} fino a "
                      f"({move_x:.2f}, {move_y:.2f}), poi riguardo con una scansione fresca.")
            else:
                print(f"[OK] Tratto {self.robot_node_id}->{self.next_node_id} confermato libero dal fronte. Eseguo movimento...")

            esito = self._cammina_tratto(move_x, move_y, next_x, next_y, is_partial_step,
                                         vision_tform_body_current)
            if esito == 'fermo':
                break
            if esito is FALLITO:
                return FALLITO
        # Gambe finite (o interrotte): si torna in cima con una scansione fresca.
        return CONTINUA

    def _controlla_posa_d_arrivo(self):
        """Il passo porterebbe in un punto senza uscite? Allora non ci si va: tratto fuori dal
        piano per il tentativo e nuovo percorso (CONTINUA), o lato abbandonato (FALLITO).
        None = si puo' andare."""
        move_x, move_y = self.decision['target']
        # --- Controllo preventivo sulla posa d'arrivo --------------------------------------
        # Solo per i passi corti o verso spazi stretti: sono quelli con cui si entra negli
        # angoli senza uscita. Vedi arrival_has_exit.
        step_len = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
        r_a, c_a, in_a = arcVerification._cells(self.snap, np.array([move_x]), np.array([move_y]))
        od_arrival = float(self.snap.obstacle_dist[r_a[0], c_a[0]]) if in_a[0] else float('inf')
        if (self.arrival_refusals < P.MAX_ARRIVAL_REFUSALS
                and (step_len <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_arrival <= P.ARRIVAL_EXIT_CHECK_OD_M)):
            heading_arr = float(np.arctan2(move_y - self.robot_y, move_x - self.robot_x))
            has_exit, fan_arr = arcVerification.arrival_has_exit(self.snap, (move_x, move_y), heading_arr)
            if not has_exit:
                self.arrival_refusals += 1
                MISSION_STATS['arrival_refusals'] += 1
                n_us_a, fan_txt_a = arcVerification.describe_escape_fan(fan_arr)
                print(f"[ARRIVO] Il passo di {step_len:.2f} m verso ({move_x:.2f}, {move_y:.2f}) "
                      f"(obstacle_distance li' {od_arrival:.2f} m) mi porterebbe dove non ci sono "
                      f"uscite: {n_us_a}/{len(fan_arr)} direzioni utilizzabili ({fan_txt_a}). "
                      f"Non ci vado ({self.arrival_refusals}/{P.MAX_ARRIVAL_REFUSALS}), scarto il tratto "
                      f"{self.robot_node_id}->{self.next_node_id} per il resto del tentativo e ripianifico.")
                # Fuori dal piano per il RESTO DEL TENTATIVO (penalita' del tentativo, come i
                # ritorni: vedi _trap_backtrack), non solo per ora: il tratto in se' e' libero,
                # e' il punto d'arrivo a non avere uscite, quindi il refresh lo rimetterebbe
                # nel grafo al giro dopo e si rifarebbe lo stesso piano. Il 28-10 12:16 lo
                # stesso passo e' stato rifiutato tre volte di fila, e alla terza eseguito.
                self._scarta_tratto(self.robot_node_id, self.next_node_id, 'tentativo')
                new_ids = self.prm_graph.find_path_dijkstra(self.robot_node_id, self.goal_id)
                if new_ids is not None and len(new_ids) > 1:
                    self._usa_percorso(new_ids)
                    return CONTINUA
                # Sapere in anticipo che il punto d'arrivo e' un vicolo cieco e andarci lo
                # stesso non ha senso: si rinuncia a questo lato della cella, che verra'
                # ritentata piu' avanti da un altro (finalize_or_defer_blocked_cell).
                print(f"[ARRIVO] Nessuna alternativa nel grafo e il punto d'arrivo non ha "
                      f"uscite: non ci vado. Rinuncio a entrare in ({self.target_row},{self.target_col}) "
                      f"da questo lato.")
                if self.verification_tracker is not None:
                    self.verification_tracker.clear_path()
                return FALLITO
        return None

    def _prepara_gamba(self, leg_i, n_legs, move_x, move_y):
        """Prima di una gamba intermedia del multinodo: posa vera, rotazione possibile e
        uscite all'arrivo. La posa VISION da cui partire, o None se ci si ferma a riguardare."""
        # Posa VERA dopo la gamba precedente. Serve a tre cose, tutte necessarie:
        #  - navigate_to calcola l'angolo di rotazione dall'orientamento che riceve,
        #    e relative_move ruota RISPETTO ALLA POSA ATTUALE: con l'orientamento
        #    della scansione iniziale (prima della gamba 0) il robot ruoterebbe
        #    dell'angolo sbagliato e camminerebbe fuori dal percorso verificato;
        #  - la distanza da percorrere si calcola da (robot_x, robot_y);
        #  - l'arresto a meta' movimento misura il percorso rimanente da qui.
        self.robot_x, self.robot_y, self.robot_z, robot_quat = spotUtils.getPosition(self.robot_state_client)
        self.robot_yaw = np.arctan2(
            2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
            1.0 - 2.0 * (robot_quat.y ** 2 + robot_quat.z ** 2))
        fresh_vision_tform_body = SimpleNamespace(
            position=SimpleNamespace(x=self.robot_x, y=self.robot_y, z=self.robot_z), rotation=robot_quat)

        # Al nodo intermedio il robot si FERMA e RUOTA verso il nodo successivo: la
        # decisione iniziale ha controllato solo la rotazione della prima gamba.
        # Qui si controlla questa, sulla fotografia della scansione (in coordinate
        # mondo, quindi valida anche dopo lo spostamento) -- ma solo se TUTTO il
        # cerchio spazzato sta dentro i dati: rotation_is_clear ignora le celle
        # fuori finestra (ottimista), e con un percorso curvo un nodo intermedio
        # puo' stare vicino al bordo. Se non si puo' ruotare, o non si puo' dire,
        # ci si ferma e si riguarda: la rotazione la gestisce il ciclo, con cono,
        # arretramento e ventaglio, sui dati freschi.
        r_h, c_h, in_h = arcVerification._cells(self.snap, np.array([self.robot_x]), np.array([self.robot_y]))
        od_here = float(self.snap.obstacle_dist[r_h[0], c_h[0]]) if in_h[0] else 0.0
        rot_leg = arcVerification.rotation_plan(self.robot_yaw, (self.robot_x, self.robot_y), (move_x, move_y),
                                od_here, snap=self.snap)
        reach_rot = float(spotGrid.ROTATION_CLEARANCE_M)
        ny_s, nx_s = self.snap.obstacle_dist.shape
        inside_rot = (self.snap.origin_x <= self.robot_x - reach_rot and
                      self.robot_x + reach_rot <= self.snap.origin_x + nx_s * self.snap.cell_size and
                      self.snap.origin_y <= self.robot_y - reach_rot and
                      self.robot_y + reach_rot <= self.snap.origin_y + ny_s * self.snap.cell_size)
        if not rot_leg['turn'] or (rot_leg['why'] == 'settore spazzato' and not inside_rot):
            motivo = ("il corpo toccherebbe" if not rot_leg['turn']
                      else "il cerchio di rotazione esce dai dati della scansione")
            print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: per puntare il nodo "
                  f"{self.next_node_id} servono {rot_leg['dyaw_deg']:+.0f} gradi e {motivo}. "
                  f"Mi fermo qui e riguardo con una scansione fresca.")
            MISSION_STATS['multinode_cut'] += 1
            return None

        # Stessa prudenza della prima gamba sulla posa d'arrivo (arrival_has_exit):
        # non fermarsi in un punto senza uscite.
        step_leg = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
        r_l, c_l, in_l = arcVerification._cells(self.snap, np.array([move_x]), np.array([move_y]))
        od_leg = float(self.snap.obstacle_dist[r_l[0], c_l[0]]) if in_l[0] else float('inf')
        if step_leg <= P.ARRIVAL_EXIT_CHECK_MAX_STEP_M or od_leg <= P.ARRIVAL_EXIT_CHECK_OD_M:
            has_exit_leg, _fan_leg = arcVerification.arrival_has_exit(
                self.snap, (move_x, move_y), float(np.arctan2(move_y - self.robot_y, move_x - self.robot_x)))
            if not has_exit_leg:
                print(f"[MULTINODO] Gamba {leg_i + 1}/{n_legs}: il nodo {self.next_node_id} "
                      f"(obstacle_distance {od_leg:.2f} m) non avrebbe uscite. Mi fermo "
                      f"qui e riguardo con una scansione fresca.")
                MISSION_STATS['multinode_cut'] += 1
                return None

        if self.verification_tracker is not None:
            self.path_version = self.verification_tracker.update_path(
                [(x, y) for _, x, y in self.path_waypoints])
        return fresh_vision_tform_body

    def _terreno_del_tratto(self, move_x, move_y):
        """Pendenza e rugosita' del tratto, andatura scelta di conseguenza."""
        # --- Misura della difficolta' di QUESTO segmento (per il gait e per il CSV).
        segment_long_slope, segment_lat_slope, segment_roughness = self.prm_graph.sample_terrain_between_points(
            self.robot_x, self.robot_y, move_x, move_y,
            node_idx1=self.robot_node_id, node_idx2=self.next_node_id
        )
        slope_source = self.prm_graph.last_slope_source

        near_thresh = spotGrid.SLOPE_THRESHOLD * (1.0 - spotGrid.NEAR_THRESHOLD_MARGIN_FRACTION)
        if (segment_long_slope >= near_thresh or segment_lat_slope >= near_thresh) and self.mission_folder is not None:
            try:
                npz_near = os.path.join(
                    self.mission_folder,
                    f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_"
                    f"arc{self.robot_node_id}-{self.next_node_id}_NEARLIMIT.npz")
                np.savez_compressed(
                    npz_near,
                    obstacle_grid=np.asarray(self.obstacle_mask_updated).reshape((self.num_y_up, self.num_x_up)),
                    terrain_grid=np.asarray(self.terrain_real_up).reshape((self.num_y_up, self.num_x_up)),
                    grid_origin_x=self.grid_origin_x_up, grid_origin_y=self.grid_origin_y_up, cell_size=self.cell_size_up,
                    arc_xy=np.array([self.robot_x, self.robot_y, move_x, move_y]),
                    long_slope=segment_long_slope, lat_slope=segment_lat_slope,
                    slope_threshold=spotGrid.SLOPE_THRESHOLD)
                MISSION_STATS['near_threshold_saves'] += 1
                margine = spotGrid.SLOPE_THRESHOLD - max(segment_long_slope, segment_lat_slope)
                print(f"[NEAR-LIMIT] Arco {self.robot_node_id}->{self.next_node_id} vicino soglia "
                      f"(long={segment_long_slope:.3f}, lat={segment_lat_slope:.3f}, "
                      f"soglia={spotGrid.SLOPE_THRESHOLD:.3f}, margine residuo={margine:+.3f}) "
                      f"-- dati salvati in {os.path.basename(npz_near)}")
            except Exception as e:
                print(f"[DATA SAVE] Impossibile salvare il file vicino soglia: {e}")

        if self.mobility_kwargs:
            segment_mobility_kwargs = self.mobility_kwargs
            segment_tier = 'override'
        else:
            segment_gradient = max(segment_long_slope, segment_lat_slope)
            segment_mobility_kwargs = movements.select_mobility_params(segment_gradient, segment_roughness)
            segment_tier = MISSION_STATS['last_tier']
        return (segment_long_slope, segment_lat_slope, segment_roughness, slope_source,
                segment_mobility_kwargs, segment_tier)

    def _muovi_e_registra(self, move_x, move_y, vision_tform_body_current, terreno):
        """Il comando di movimento, con l'arresto a meta' se il fronte si accorcia, e le righe
        di trajectory.csv, frontier_live.csv e segments.csv. Restituisce (riuscito, metri
        percorsi, metri comandati, motivo dell'arresto o None)."""
        (segment_long_slope, segment_lat_slope, segment_roughness, slope_source,
         segment_mobility_kwargs, segment_tier) = terreno
        # --- Condizione di arresto a meta' movimento ---------------------------------
        abort_state = {'reason': None, 'bad': 0}
        move_t0 = time.time()
        live_frontiers = []

        def _should_abort_move():
            if self.verification_tracker is None:
                return False
            f = self.verification_tracker.get_frontier()
            if f is None or f.get('path_version') != self.path_version or f['time'] <= move_t0:
                return False
            frx, fry = f['robot_xy']
            remaining = float(np.hypot(move_x - frx, move_y - fry))
            reach = arcVerification.allowed_advance(f)
            if not live_frontiers or live_frontiers[-1][0] != f['time']:
                live_frontiers.append((f['time'], frx, fry, f['reason'], float(f['dist']), reach, remaining,
                                       abort_state['bad']))
            if (f['reason'] == 'fine_dati'
                    and float(np.hypot(frx - self.robot_x, fry - self.robot_y)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                abort_state['bad'] = 0      # sta ancora ruotando sul posto: vedi la costante
                return False
            if reach + P.FRONTIER_ABORT_TOLERANCE_M < remaining:
                abort_state['bad'] += 1
                if abort_state['bad'] >= P.FRONTIER_ABORT_CONFIRM:
                    sxa, sya = f['stop_xy']
                    abort_state['reason'] = (f"il fronte si e' accorciato ({f['reason']} in "
                                             f"({sxa:.2f}, {sya:.2f})): raggiungibili {reach:.2f} m "
                                             f"su {remaining:.2f} m mancanti")
                    return True
            else:
                abort_state['bad'] = 0
            return False

        # --- Esecuzione con registrazione dell'inclinazione reale del corpo ---
        tilt_recorder = BodyTiltRecorder(self.robot_state_client)
        tilt_recorder.start()
        try:
            self.need_refresh = True              # fine di un movimento: vedi REFRESH A EVENTI
            success_move, distance_traveled, distance_commanded = movements.navigate_to(
                move_x, move_y, self.robot_x, self.robot_y, self.robot_state_client, self.command_client,
                vision_tform_body_current, should_abort=_should_abort_move,
                keep_heading=self.keep_heading,
                **segment_mobility_kwargs
            )
        finally:
            tilt_stats = tilt_recorder.stop()
        move_duration = time.time() - move_t0
        timing_add('movimento', move_duration)
        try:   # solo log: non deve mai interrompere la missione
            _seg_id = [self.iteration, self.target_row, self.target_col, self.loop_iter, self.robot_node_id, self.next_node_id]
            append_trajectory(getattr(tilt_recorder, 'samples', []), 'prm', (move_x, move_y), _seg_id)
            append_csv("frontier_live.csv",
                        ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                         'robot_x', 'robot_y', 'reason', 'frontier_dist', 'reachable_m', 'remaining_m',
                         'consecutive_bad', 'aborted'],
                        [[lf[0]] + _seg_id + list(lf[1:]) + [abort_state['reason'] is not None]
                         for lf in live_frontiers])
        except Exception as e:
            print(f"[LOG] Traiettoria/fronti del movimento non registrati: {e}")

        # --- Riga del CSV per questo segmento (anche se il movimento e' fallito) ---
        try:
            seg_dist = float(np.hypot(move_x - self.robot_x, move_y - self.robot_y))
            cost_dist = self.prm_graph.alpha * seg_dist
            cost_long = self.prm_graph.beta * segment_long_slope
            cost_lat = self.prm_graph.beta_lateral * segment_lat_slope
            cost_rough = self.prm_graph.gamma * segment_roughness
            log_segment({
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'iteration': self.iteration, 'cell_row': self.target_row, 'cell_col': self.target_col,
                'node_from': self.robot_node_id, 'node_to': self.next_node_id,
                'x_from': self.robot_x, 'y_from': self.robot_y, 'x_to': move_x, 'y_to': move_y,
                'dist_m': seg_dist,
                'slope_source': slope_source,
                'long_slope': segment_long_slope, 'lat_slope': segment_lat_slope,
                'roughness': segment_roughness,
                'cost_dist': cost_dist, 'cost_long': cost_long, 'cost_lat': cost_lat,
                'cost_rough': cost_rough, 'cost_total': cost_dist + cost_long + cost_lat + cost_rough,
                'tier': segment_tier, 'gait_override': bool(self.mobility_kwargs),
                'success': bool(success_move),
                'dist_traveled_m': distance_traveled, 'dist_commanded_m': distance_commanded,
                'duration_s': move_duration,
                'roll_start_deg': tilt_stats['roll_start'], 'pitch_start_deg': tilt_stats['pitch_start'],
                'roll_end_deg': tilt_stats['roll_end'], 'pitch_end_deg': tilt_stats['pitch_end'],
                'roll_peak_abs_deg': tilt_stats['roll_peak_abs'], 'pitch_peak_abs_deg': tilt_stats['pitch_peak_abs'],
                'tilt_samples': tilt_stats['samples'], 'tilt_errors': tilt_stats['errors'],
            })
        except Exception as e:
            print(f"[LOG] Impossibile registrare il segmento: {e}")
        return success_move, distance_traveled, distance_commanded, abort_state['reason']

    def _cammina_tratto(self, move_x, move_y, next_x, next_y, is_partial_step, vision_tform_body_current):
        """Un tratto: andatura, movimento, esito. 'avanti' = riuscito, si puo' fare la gamba
        dopo; 'fermo' = interrotto o ripianificato, si riguarda; FALLITO = guasto fisico."""
        terreno = self._terreno_del_tratto(move_x, move_y)
        success_move, distance_traveled, distance_commanded, abort_reason = self._muovi_e_registra(
            move_x, move_y, vision_tform_body_current, terreno)
        # Comando finito male (tempo scaduto, Spot non assestato...) senza un arresto chiesto
        # dal fronte, ma con il robot ARRIVATO a pochi cm dal punto: il tratto vale come
        # riuscito invece che come fallimento fisico (arco scartato per sempre e ritorno).
        # 28-10 13:50, cella (3,2): salita a 20 gradi in HARD, 0.42 m su 0.44 oltre i 15 s.
        # In segments.csv resta l'esito vero del comando.
        if not success_move and abort_reason is None:
            try:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                d_left = float(np.hypot(move_x - ax, move_y - ay))
            except Exception:
                d_left = float('inf')
            if d_left <= P.MOVE_NEAR_GOAL_M and distance_traveled >= 0.5 * distance_commanded:
                MISSION_STATS['moves_near_goal'] = MISSION_STATS.get('moves_near_goal', 0) + 1
                print(f"[QUASI] Comando verso ({move_x:.2f}, {move_y:.2f}) non concluso, ma il robot "
                      f"e' a {d_left:.2f} m dal punto (percorsi {distance_traveled:.2f} m su "
                      f"{distance_commanded:.2f}): vale come arrivato, l'arco resta valido.")
                success_move = True

        if success_move:
            if not hasattr(self.env, '_traveled_arcs'):
                self.env._traveled_arcs = []
            self.env._traveled_arcs.append((self.robot_x, self.robot_y, move_x, move_y))

            if is_partial_step:
                ax, ay, _, _ = spotUtils.getPosition(self.robot_state_client)
                self._sosta(ax, ay, "sosta (passo parziale)")
            else:
                # Arco "percorso" solo se il robot e' partito dal nodo: altrimenti il tratto
                # camminato non e' l'arco del grafo (vedi add_stop_node, nessun nodo nuovo).
                if mission_graph.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y):
                    self.prm_graph.traversed_edges.add((min(self.robot_node_id, self.next_node_id),
                                                   max(self.robot_node_id, self.next_node_id)))
                self.robot_node_id = self.next_node_id
                self.prm_graph.set_robot_node(self.robot_node_id, 0.0)
                self.path_waypoints.pop(0)

            # Briciola per i ritorni: un waypoint GraphNav ogni
            # GRAPHNAV_WAYPOINT_MIN_SPACING_M di strada percorsa.
            navGraphUtils.graphnav_drop_breadcrumb(self.env, self.recordingInterface, self.robot_state_client,
                                     self.robot_node_id, self.target_row, self.target_col)

        elif abort_reason is not None:
            print(f"[STOP] Movimento verso ({move_x:.2f}, {move_y:.2f}) interrotto dopo "
                  f"{distance_traveled:.2f}m: {abort_reason}.")
            MISSION_STATS['frontier_aborts'] += 1
            if distance_traveled > 0.05:
                self._sosta(*self._strada_fatta(self.robot_x, self.robot_y), "sosta (arresto)")
            return 'fermo'

        else:
            print(f"[FAIL] Comando di movimento fallito meccanicamente per ({move_x:.2f}, {move_y:.2f}) "
                  f"(percorsi {distance_traveled:.2f}m su {distance_commanded:.2f}m richiesti)")

            movements.check_and_recover_fall(self.robot_state_client, self.command_client, self.global_map,
                                    "dopo il fallimento del movimento")

            if distance_traveled > 0.05:
                self._strada_fatta(self.robot_x, self.robot_y)

            # Se il robot non partiva dal nodo, il tratto fallito non e' l'arco del grafo.
            sul_nodo = mission_graph.robot_on_node(self.prm_graph, self.robot_node_id, self.robot_x, self.robot_y)
            self._scarta_tratto(self.robot_node_id, self.next_node_id, 'missione' if sul_nodo else 'ora')
            print(f"[INFO] Tratto {self.robot_node_id}-{self.next_node_id} scartato dopo fallimento fisico.")
            if self._ritorno((self.robot_node_id, self.next_node_id),
                             f"dal nodo {self.robot_node_id}"):
                return 'fermo'

            fail_save_path = os.path.join(self.mission_folder,
                                          f"iteration_{self.iteration}_cell_{self.target_row}_{self.target_col}_MOVE_FAIL.png") if self.mission_folder else None
            visualize_grid_with_candidates(
                pts=self.pts_up, terrain_real=self.terrain_real_up, obstacle_mask=self.obstacle_mask_updated,
                robot_x=self.robot_x, robot_y=self.robot_y,
                candidates={'rejected': [], 'valid': []},
                chosen_point=(next_x, next_y), iteration=self.iteration, env=self.env,
                save_path=fail_save_path, prm_graph=self.prm_graph, chosen_path=self.full_path_coords,
                cells_obstacle_dist=self.cells_obs_up, valid_values=self.valid_up,
                grad_values=self.grad_up, rough_values=self.rough_up
            )
            return FALLITO
        return 'avanti'

    def _fine(self):
        """Dopo il ciclo: True se il robot e' nella cella (o l'arrivo appena fuori e' stato accettato)."""
        if self.verification_tracker is not None:
            self.verification_tracker.clear_path()

        # Controllo finale della posizione a fine percorso
        self.robot_x, self.robot_y, _, _ = spotUtils.getPosition(self.robot_state_client)
        if self.arrived_near_goal:
            return True
        return self.env.is_point_in_cell(self.robot_x, self.robot_y, self.target_row, self.target_col)



def attempt_enter_cell_from_position(local_grid, global_grid, robot_state_client, command_client,
                                     env, target_row, target_col, sampler, prm_graph, mission_folder=None,
                                     iteration=0,
                                     recordingInterface=None, verification_tracker=None, mobility_kwargs=None):
    """Un tentativo di entrare nella cella (target_row, target_col): vedi TentativoCella."""
    return TentativoCella(local_grid, global_grid, robot_state_client, command_client,
                          env, target_row, target_col, sampler, prm_graph, mission_folder,
                          iteration, recordingInterface, verification_tracker,
                          mobility_kwargs).esegui()
