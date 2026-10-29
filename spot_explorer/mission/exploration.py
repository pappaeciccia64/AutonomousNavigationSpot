"""La missione cella per cella: Missione, scelta della cella da tentare, viaggi con GraphNav."""
import os
import time
import numpy as np

from spot_explorer.graphnav import graphnav as navGraphUtils
from spot_explorer.robot import motion as movements
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.mission import cells as environmentMap
from spot_explorer.robot import connection as spotUtils

from spot_explorer.planning import mission_graph                   # reticolo dei nodi della missione

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, visualize_grid_with_candidates
from spot_explorer.mission import cell_attempt
from spot_explorer.planning import goal
from spot_explorer.mission import recovery


# =========================================================================
# LA MISSIONE: il ciclo cella per cella, avvio e chiusura
# =========================================================================


class Missione:
    """La missione cella per cella, dopo l'avvio (vedi easy_walk).

    La frontiera e' la lista delle celle candidate (riga, colonna, rango sul percorso a
    serpentina). A ogni giro: se una cella della frontiera confina con quella del robot si
    tenta di entrarci da qui (_cella_confinante); altrimenti si sceglie la candidata di rango
    piu' basso, ci si avvicina con GraphNav fino al waypoint della cella visitata piu' vicina
    e si tenta da li' (_scegli_cella_lontana, _viaggio_e_tentativo). Un tentativo fallito segna
    il LATO da cui si e' provato (_lato_fallito): la cella si blocca solo quando ogni lato
    raggiungibile e' stato provato (finalize_or_defer_blocked_cell).

    A batteria bassa (vedi batteria.py) l'esplorazione si ferma anche con la frontiera non
    vuota: un tentativo interrotto per il rientro non segna il lato come provato e la cella
    resta in frontiera, cosi' lo stato salvato dice cosa restava da esplorare.
    """

    def __init__(self, robot_state_client, command_client, recordingInterface, local_grid,
                 global_grid, env, prm, sampler, verification_tracker, mission_folder,
                 mobility_kwargs):
        self.robot_state_client = robot_state_client
        self.command_client = command_client
        self.recordingInterface = recordingInterface
        self.local_grid = local_grid
        self.global_grid = global_grid
        self.env = env
        self.prm = prm
        self.sampler = sampler
        self.verification_tracker = verification_tracker
        self.mission_folder = mission_folder
        self.mobility_kwargs = mobility_kwargs
        self.path = env.generate_serpentine_path(start_cell=env.start_cell)
        self.frontier = []
        self.visualization_counter = 0   # numero progressivo delle immagini salvate
        # Avanzamento (vedi _progresso): istante d'inizio, celle chiuse all'inizio e
        # all'ultima riga stampata, punti del rettangolo della missione per l'area vista.
        self._t0 = None
        self._closed_at_start = 0
        self._ultima_visitata = None      # vedi _cella_del_robot
        self._tentativi = 0               # tentativi di entrare in una cella (vedi _fine_esplorazione)
        self._celle_perse = []            # tolte dalla frontiera senza lati ancora provabili
        self._closed_logged = None
        self._area_points = None

    # --- Pezzi comuni --------------------------------------------------------------------

    def _tenta(self, row, col):
        """Un tentativo di entrare nella cella (row, col) dalla posizione attuale."""
        check = cell_attempt.attempt_enter_cell_from_position(
            self.local_grid, self.global_grid, self.robot_state_client, self.command_client,
            self.env, row, col, self.sampler, self.prm, self.mission_folder,
            self.visualization_counter, self.recordingInterface, self.verification_tracker,
            mobility_kwargs=self.mobility_kwargs)
        self.visualization_counter += 1
        self._tentativi += 1
        return check

    def _visitata(self, row, col, x, y):
        """Cella (row, col) visitata con il robot in (x, y): waypoint GraphNav della cella e
        nuove candidate attorno a lei."""
        self.env.update_position(x, y)
        self.recordingInterface.create_default_waypoint(cell_row=row, cell_col=col)
        self.env.mark_cell_visited(row, col)
        self._ultima_visitata = (int(row), int(col))
        # Le confinanti della cella APPENA VISITATA, non di quella in cui sta il robot: con
        # l'arrivo appena fuori dalla cella (ARRIVAL_OUTSIDE_CELL_M) il robot puo' essere
        # ancora nella cella di prima.
        self.frontier.extend(environmentMap.find_new_borders(
            self.env, row, col, self.path, self.frontier))

    def _cella_del_robot(self):
        """
        La cella da cui il robot cerca le confinanti e segna i lati provati. Di solito quella
        in cui sta; ma se l'ultima cella visitata e' stata accettata con il robot appena FUORI
        (ARRIVAL_OUTSIDE_CELL_M) e il robot e' ancora li', vale quella: il 28-10 13:50 la (7,2)
        e' stata visitata con il robot 0.29 m fuori, nella (6,3), e la (7,1) che le stava
        accanto non e' mai stata tentata da li'.
        """
        x, y, _, _ = spotUtils.getPosition(self.robot_state_client)
        cell = tuple(int(v) for v in self.env.get_cell_from_world(x, y))
        uv = self._ultima_visitata
        if uv is not None and cell != uv:
            border = environmentMap.cell_border_distance(self.env, uv[0], uv[1], x, y)
            if border is not None and -border <= P.ARRIVAL_OUTSIDE_CELL_M:
                print(f"[CELLA] Il robot e' nella cella {cell}, ma {-border:.2f} m fuori dalla "
                      f"{uv} appena visitata: cerco le confinanti dalla {uv}.")
                return uv
        return cell

    def _cella_di_partenza_per(self, row, col, waypoints_by_cell):
        """
        La cella visitata (con un waypoint GraphNav) da cui avvicinarsi alla candidata (row,
        col): fra le CONFINANTI, una il cui lato verso la candidata non e' ancora stato
        provato, la piu' vicina al robot. Se nessuna confinante ha un lato libero, la regola
        di sempre (find_nearest_waypoint_cell_to_target), che il chiamante poi salta se quel
        lato e' gia' provato. Prima si prendeva sempre la prima confinante in ordine fisso
        (nord, sud, ovest, est): provato quel lato, la cella non veniva piu' tentata dagli
        altri e usciva dalla frontiera (28-10 13:50, celle (7,0) e (7,1)).
        """
        env, ri = self.env, self.recordingInterface
        sides = env.get_cell_sides_status(row, col)
        x, y, _, _ = spotUtils.getPosition(self.robot_state_client)
        cands = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            if not (0 <= r < env.rows and 0 <= c < env.cols) or env.map[r][c] != 1:
                continue
            if (r, c) not in waypoints_by_cell or ri.get_manual_waypoint_by_cell(r, c) is None:
                continue
            if sides & env.get_side_bit_facing_origin(r, c, row, col):
                continue
            wx, wy = env.get_world_position_from_cell(r, c)
            cands.append((float(np.hypot(wx - x, wy - y)), (r, c)))
        if cands:
            d, cell = min(cands)
            print(f"[NEAREST_WP] Cella ({row},{col}): mi avvicino dalla confinante {cell}, lato non "
                  f"ancora provato, a {d:.1f} m dal robot"
                  + (f" ({len(cands)} lati liberi)" if len(cands) > 1 else "") + ".")
            return cell
        return ri.find_nearest_waypoint_cell_to_target((row, col), waypoints_by_cell, env)

    def _lato_fallito(self, row, col, from_row, from_col):
        """Segna come provato il lato della cella (row, col) che guarda la cella (from_row,
        from_col). True se ora la cella e' bloccata (nessun lato raggiungibile da provare)."""
        side_bit = self.env.get_side_bit_facing_origin(from_row, from_col, row, col)
        self.env.mark_cell_side_explored(row, col, side_bit)
        recovery.finalize_or_defer_blocked_cell(self.env, row, col,
                                       self.sampler, self.global_grid.global_occupancy_map)
        return self.env.is_cell_blocked(row, col)

    # --- Il ciclo ------------------------------------------------------------------------

    def esplora(self):
        """Visita le celle finche' la frontiera non e' vuota."""
        x, y, z, _ = spotUtils.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x, y)
        self.frontier.extend(environmentMap.find_new_borders(
            self.env, robot_row, robot_col, self.path, self.frontier))
        self._t0 = time.time()
        self._closed_at_start = self._celle()[0]

        while (True):
            self._progresso()
            if cell_attempt._rientro_batteria():
                MISSION_STATS['battery_return_cells_left'] = len(self.frontier)
                print(f"[BATTERIA] Fine dell'esplorazione per batteria bassa: restano "
                      f"{len(self.frontier)} celle in frontiera {[(f[0], f[1]) for f in self.frontier]}.")
                break
            robot_row, robot_col = self._cella_del_robot()

            borders = self.env.get_adjacent_frontier_cells(robot_row, robot_col, self.path)
            borders_in_frontier = [b for b in borders
                                   if any((f[0] == b[0] and f[1] == b[1]) for f in self.frontier)]

            if len(borders_in_frontier) != 0:
                self._cella_confinante(min(borders_in_frontier, key=lambda b: b[2]),
                                       robot_row, robot_col)
            else:
                # Nessuna cella confinante in frontiera: si va con GraphNav verso il waypoint
                # piu' vicino alla cella di rango piu' basso e si tenta da li'.
                scelta = self._scegli_cella_lontana()
                if scelta is not None:
                    if self._viaggio_e_tentativo(*scelta) == 'caduta':
                        continue
                else:
                    if len(self.frontier) > 0:
                        tolta = self.frontier[0]
                        self.frontier.remove(tolta)
                        self._celle_perse.append((int(tolta[0]), int(tolta[1])))
                        print(f"[FRONTIERA] Cella ({tolta[0]},{tolta[1]}) tolta dalla frontiera: in "
                              f"questo giro nessuna candidata ha un lato ancora da provare.")

            if len(self.frontier) == 0:
                break
        self._fine_esplorazione()
        self._progresso(finale=True)

    def _fine_esplorazione(self):
        """Se l'esplorazione finisce con celle non chiuse, una riga [FINE] che dice perche'
        (28-10 14:17: partenza in mezzo all'erba alta, nessuna cella raggiungibile, e nel log
        solo "Frontiera vuota")."""
        closed, visited, blocked, total = self._celle()
        if closed >= total or cell_attempt._batteria_scarica():
            return
        if visited <= 1:
            print(f"[FINE] Nessuna cella raggiungibile dalla partenza: {self._tentativi} tentativi "
                  f"sulle celle confinanti, tutti falliti senza una strada nel grafo o con il passo "
                  f"rifiutato (ostacoli tutto intorno: vedi GRAFO DISCONNESSO e [ARRIVO] sopra). "
                  f"Spostare il robot su terreno libero e riprovare.")
            return
        print(f"[FINE] Esplorazione finita con {total - closed} celle non chiuse"
              + (f"; tolte dalla frontiera senza lati ancora da provare: "
                 f"{', '.join(f'({r},{c})' for r, c in self._celle_perse)}" if self._celle_perse else "")
              + ". Le altre non sono mai state raggiungibili da una cella visitata.")

    # --- Avanzamento ---------------------------------------------------------------------

    def _celle(self):
        """(chiuse, visitate, bloccate, totale): chiusa = visitata o bloccata."""
        vals = [v for row in self.env.map for v in row]
        visited = sum(1 for v in vals if v == 1)
        blocked = sum(1 for v in vals if v == -1)
        return visited + blocked, visited, blocked, len(vals)

    def _area_vista(self):
        """Frazione del rettangolo della missione gia' osservata (libera o ostacolo) nella
        mappa globale, su punti ogni 0.1 m. None se la mappa non c'e' ancora."""
        gm = self.global_grid.global_occupancy_map
        if gm is None:
            return None
        if self._area_points is None:
            env = self.env
            k = np.arange(-env.cell_size / 2.0 + 0.05, env.cell_size / 2.0, 0.1)
            ox, oy = np.meshgrid(k, k)
            c, s_ = np.cos(env.origin_yaw), np.sin(env.origin_yaw)
            pts = []
            for r in range(env.rows):
                for col in range(env.cols):
                    cx, cy = env.get_world_position_from_cell(r, col)
                    pts.append(np.c_[cx + ox.ravel() * c - oy.ravel() * s_,
                                     cy + ox.ravel() * s_ + oy.ravel() * c])
            self._area_points = np.vstack(pts)
        res = gm.resolution
        keys = np.floor(self._area_points / res).astype(int)
        return sum(1 for gx, gy in keys if (gx, gy) in gm.grid) / len(keys)

    def _progresso(self, finale=False):
        """
        A che punto e' la missione: una riga [PROGRESSO] quando cambiano le celle chiuse
        (visitate o bloccate), e una alla fine. Celle chiuse sul totale (il numero
        principale), area della missione gia' vista nella mappa globale, tempo, stima del
        tempo che manca dal tempo medio per cella, batteria. Solo log e MISSION_STATS
        ['progresso'] (rapporto, mission_summary.json): nessuna decisione ne dipende.
        """
        closed, visited, blocked, total = self._celle()
        if not finale and closed == self._closed_logged:
            return
        self._closed_logged = closed
        try:
            area = self._area_vista()
        except Exception:
            area = None
        minutes = (time.time() - self._t0) / 60.0 if self._t0 is not None else 0.0
        done_here = closed - self._closed_at_start
        per_cell = minutes / done_here if done_here > 0 else None
        # L'ultima carica letta dal monitor della batteria (batteria.py), senza una lettura in piu'.
        battery = MISSION_STATS.get('battery_last_pct')
        pct = 100.0 * closed / total
        area_txt = f"area vista {100.0 * area:.0f}%" if area is not None else "area vista ?"
        bat_txt = f"batteria {battery:.0f}%" if battery is not None else "batteria ?"
        if finale:
            tempo = (f"{minutes:.1f} min di esplorazione"
                     + (f" ({per_cell:.2f} min a cella)" if per_cell else "")
                     + (f", {total - closed} celle mai raggiunte" if total > closed else ""))
        else:
            tempo = (f"{minutes:.1f} min trascorsi"
                     + (f", ~{(total - closed) * per_cell:.0f} min alla fine ({per_cell:.2f} min a cella)"
                        if per_cell else ""))
        print(f"[PROGRESSO] {'Fine esplorazione: ' if finale else ''}{closed}/{total} celle chiuse "
              f"({pct:.0f}%): {visited} visitate, {blocked} bloccate, {total - closed} da fare | "
              f"{area_txt}\n            {tempo} | {bat_txt}")
        MISSION_STATS['progresso'] = {
            'celle_chiuse': closed, 'celle_totali': total, 'percentuale': round(pct, 1),
            'visitate': visited, 'bloccate': blocked,
            'area_vista': None if area is None else round(100.0 * area, 1),
            'minuti': round(minutes, 1), 'minuti_a_cella': None if per_cell is None else round(per_cell, 2),
            'batteria': battery, 'finale': bool(finale)}

    def _cella_confinante(self, selected_border, robot_row, robot_col):
        """Tentativo su una cella della frontiera che confina con quella del robot."""
        row, col = selected_border[0], selected_border[1]
        # Cella da cui si tenta l'ingresso: serve dopo per segnare il LATO provato. Non la
        # cella in cui il robot si trova DOPO il tentativo, che una ritirata o un passo
        # parziale possono aver cambiato (lato sbagliato o nullo).
        origin_row, origin_col = robot_row, robot_col
        check = self._tenta(row, col)
        self.frontier.remove(selected_border)

        x_new, y_new, _, _ = spotUtils.getPosition(self.robot_state_client)
        robot_row, robot_col = self.env.get_cell_from_world(x_new, y_new)

        if check or (row, col) == (robot_row, robot_col):
            # Fallito ma il robot e' comunque dentro la cella: vale come visitata.
            self._visitata(row, col, x_new, y_new)
        elif cell_attempt._batteria_scarica():
            # Tentativo interrotto per il rientro: la cella resta da esplorare.
            self.frontier.append(selected_border)
        else:
            self._recupero(selected_border, origin_row, origin_col)

    def _recupero(self, selected_border, origin_row, origin_col):
        """Subito dopo un tentativo fallito su una cella confinante: si riprova una volta se
        da qui c'e' una strada corta (al massimo COSTO_SOGLIA archi) verso un obiettivo
        diverso da quello appena fallito; altrimenti il lato si segna come provato.

        Gli archi scartati nel tentativo fallito (blocchi confermati da vicino, fallimenti
        fisici) sono gia' nel PRM tramite mark_edge_invalid, e la ricostruzione degli archi
        li rispetta: Dijkstra non riproporra' la stessa strada."""
        row, col = selected_border[0], selected_border[1]
        prm, global_map = self.prm, self.global_grid.global_occupancy_map

        # Posizione attuale (il fallimento puo' essere in partenza o a meta' strada)
        x_curr, y_curr, _, _ = spotUtils.getPosition(self.robot_state_client)

        # Scansione fresca, per scegliere il punto obiettivo da qui
        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is not None:
            # Senza impronta del corpo ne' maschera degli ostacoli: serve solo il terreno per
            # il PRM e la scelta dell'obiettivo.
            self.local_grid.process_scan(scan, x_curr, y_curr, 0.0, footprint=False, fuse=False)
            num_x, num_y = scan.num_x, scan.num_y
            grid_origin_x, grid_origin_y = self.local_grid.grid_origin(scan)
            prm.update_local_grid_data(
                grad_values_2d=scan.grad.reshape((num_y, num_x)),
                rough_values_2d=scan.rough.reshape((num_y, num_x)),
                terrain_values_2d=scan.terrain.reshape((num_y, num_x)),
                valid_2d=spot_grid.slope_usable_mask(scan.is_valid, scan.od, (num_y, num_x)),
                grid_origin_x=grid_origin_x,
                grid_origin_y=grid_origin_y,
                cell_size=scan.cell_size
            )
            mission_graph.rebuild_prm_edges(prm, global_map)

            target_x, target_y, _, _ = goal.find_best_point_in_cell(
                x_curr, y_curr, self.env, row, col, scan.pts, scan.od, self.sampler,
                global_map=global_map
            )
        else:
            print(
                "[RECOVERY ERROR] Impossibile recuperare il layer obstacle_distance per il ricalcolo.")
            target_x, target_y = None, None

        if target_x is None or target_y is None:
            print(
                "[SKIP] Nessun punto obiettivo valido nella cella. Continuo con l'esplorazione normale.")
            if not self._lato_fallito(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)
            return

        mission_graph.restrict_planning_to_explored_cells(prm, self.env, row, col, (x_curr, y_curr))
        start_id = prm.add_stop_node(
            x_curr, y_curr, global_map,
            geometry.PRM_EDGE_SAFETY_MARGIN_M, label="verifica recupero (robot)")
        goal_id = prm.add_stop_node(
            target_x, target_y, global_map,
            geometry.PRM_EDGE_SAFETY_MARGIN_M, label="verifica recupero (obiettivo)",
            is_robot=False)
        prm.set_robot_node(start_id)

        path_ids = prm.find_path_dijkstra(start_id, goal_id)

        # Si riprova subito solo se il percorso ha al massimo COSTO_SOGLIA archi (numero di
        # nodi - 1)
        COSTO_SOGLIA = 4

        # Se l'obiettivo e' lo STESSO del tentativo appena fallito, riprovare subito vorrebbe
        # dire stesso posto, stesso esito: la cella si rimanda e torna in coda come le altre
        # (vedi finalize_or_defer_blocked_cell).
        last_goal = getattr(self.env, '_last_attempt_goal', None)
        same_goal = (last_goal is not None and float(np.hypot(
            target_x - last_goal[0], target_y - last_goal[1])) <= P.RECOVERY_SAME_GOAL_M)
        if same_goal and path_ids is not None:
            print(f"[RECOVERY] L'obiettivo ({target_x:.2f}, {target_y:.2f}) e' "
                  f"quello del tentativo appena fallito: non riprovo subito, la "
                  f"cella torna in coda.")
            path_ids = None
        if path_ids is None or (len(path_ids) - 1) > COSTO_SOGLIA:
            print(
                f"[SKIP] Nessun percorso alternativo valido o costo superiore a {COSTO_SOGLIA}. Continuo l'algoritmo normale.")
            if not self._lato_fallito(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)
            return

        print(
            f"[RECOVERY] Trovato percorso alternativo con costo {len(path_ids) - 1} <= {COSTO_SOGLIA}. Riprovo l'ingresso...")
        if self._tenta(row, col):
            x_final, y_final, _, _ = spotUtils.getPosition(self.robot_state_client)
            self._visitata(row, col, x_final, y_final)
        elif cell_attempt._batteria_scarica():
            self.frontier.append(selected_border)
        else:
            print(
                "[FAIL] Anche il percorso alternativo ha fallito. Abbandono la cella e continuo l'algoritmo normale.")
            if not self._lato_fallito(row, col, origin_row, origin_col):
                self.frontier.append(selected_border)

    def _scegli_cella_lontana(self):
        """La candidata di rango piu' basso da cui avvicinarsi con GraphNav:
        (riga, colonna, rango, cella visitata del waypoint di partenza), None se nessuna.

        Una cella bloccata (-1) non ha mai lati ancora da provare:
        finalize_or_defer_blocked_cell() la blocca solo dopo che ogni lato raggiungibile e'
        stato provato ed e' fallito."""
        env, recordingInterface = self.env, self.recordingInterface
        already_skipped_this_round = set()
        while True:
            remaining = [c for c in self.frontier if (c[0], c[1]) not in already_skipped_this_round]
            lowest_rank_cell = env.get_lowest_rank_from_frontier_list(remaining, self.path)
            if lowest_rank_cell is None:
                return None
            candidate_row, candidate_col, candidate_rank = lowest_rank_cell
            waypoints_by_cell = recordingInterface.get_all_manual_waypoints_with_cells()
            candidate_nearest_cell = self._cella_di_partenza_per(candidate_row, candidate_col,
                                                                 waypoints_by_cell)
            if candidate_nearest_cell is None or \
                    recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell) is None:
                # Nessuna cella visitata con waypoint da cui partire: si passa alla candidata
                # successiva.
                print(f"[SKIP-ROUND] Cella ({candidate_row},{candidate_col}): nessun waypoint "
                      f"GraphNav da cui avvicinarsi. Provo la successiva.")
                already_skipped_this_round.add((candidate_row, candidate_col))
                continue
            candidate_side_bit = env.get_side_bit_facing_origin(candidate_nearest_cell[0],
                                                                candidate_nearest_cell[1],
                                                                candidate_row, candidate_col)
            if env.get_cell_sides_status(candidate_row, candidate_col) & candidate_side_bit:
                print(f"[SKIP-ROUND] Cella ({candidate_row},{candidate_col}): ci arriverei dallo "
                      f"stesso lato gia' provato senza successo. Per questo giro provo la "
                      f"candidata successiva.")
                already_skipped_this_round.add((candidate_row, candidate_col))
                continue
            # Prima di partire con GraphNav: da quel waypoint il grafo arriva alla cella? Se
            # no, il tentativo fallirebbe appena arrivati (vedi GRAPHNAV_PRECHECK): il lato si
            # segna come provato senza muoversi.
            if P.GRAPHNAV_PRECHECK:
                cand_wp = recordingInterface.get_manual_waypoint_by_cell(*candidate_nearest_cell)
                wp_xy = (cand_wp.get('x'), cand_wp.get('y')) if isinstance(cand_wp, dict) else (None, None)
                reach = (mission_graph.graph_reaches_cell(self.prm, env, wp_xy, candidate_row, candidate_col,
                                                          self.global_grid.global_occupancy_map)
                         if wp_xy[0] is not None else None)
                if reach is False:
                    MISSION_STATS['graphnav_trips_avoided'] += 1
                    print(f"[PRECHECK] Cella ({candidate_row},{candidate_col}): dal waypoint "
                          f"della cella {candidate_nearest_cell} il grafo non arriva a nessun "
                          f"nodo della cella (ostacoli gia' in mappa). Segno il lato come "
                          f"provato senza spostarmi.")
                    env.mark_cell_side_explored(candidate_row, candidate_col, candidate_side_bit)
                    recovery.finalize_or_defer_blocked_cell(env, candidate_row, candidate_col,
                                                   self.sampler, self.global_grid.global_occupancy_map)
                    if env.is_cell_blocked(candidate_row, candidate_col):
                        self.frontier[:] = [c for c in self.frontier
                                            if (c[0], c[1]) != (candidate_row, candidate_col)]
                    already_skipped_this_round.add((candidate_row, candidate_col))
                    continue
            return candidate_row, candidate_col, candidate_rank, candidate_nearest_cell

    def _viaggio_e_tentativo(self, target_row, target_col, rank, nearest_cell):
        """Con GraphNav fino al waypoint della cella nearest_cell, poi il tentativo sulla cella
        (target_row, target_col). 'caduta' se il robot e' caduto durante il viaggio."""
        recordingInterface = self.recordingInterface
        x_current, y_current, _, _ = spotUtils.getPosition(self.robot_state_client)

        nearest_wp = recordingInterface.get_manual_waypoint_by_cell(nearest_cell[0], nearest_cell[1])
        navGraphUtils.graphnav_add_shortcuts(recordingInterface, "prima dello spostamento fra celle",
                                             map_edges=True, target_wp_id=nearest_wp['id'],
                                             robot_xy=(x_current, y_current))
        recordingInterface.stop_recording()
        navigation_success = navGraphUtils.graphnav_navigate_to(
            recordingInterface, nearest_wp['id'], self.robot_state_client, self.command_client,
            f"verso il waypoint della cella {nearest_cell} per la cella ({target_row},{target_col})",
            face_xy=self.env.get_world_position_from_cell(target_row, target_col))

        # GraphNav non registra i tratti percorsi: quelli vecchi non descrivono piu' la strada
        # da cui il robot e' arrivato, la ritirata non deve usarli.
        self.env._traveled_arcs = []
        if movements.check_and_recover_fall(self.robot_state_client, self.command_client,
                                            self.global_grid.global_occupancy_map,
                                            "durante lo spostamento con GraphNav") is not None:
            recordingInterface.start_recording()
            print(f"[CADUTA] Rialzato dopo lo spostamento con GraphNav: segno il lato della "
                  f"cella ({target_row},{target_col}) come provato, per non ripetere la stessa strada.")
            if self._lato_fallito(target_row, target_col, nearest_cell[0], nearest_cell[1]):
                self.frontier.remove((target_row, target_col, rank))
            return 'caduta'

        recordingInterface.start_recording()
        # Il viaggio puo' essere stato lungo: se la batteria e' scesa sotto soglia non si
        # tenta la cella, si rientra (la cella resta in frontiera).
        if cell_attempt._rientro_batteria("dopo lo spostamento GraphNav"):
            return None
        # Se GraphNav non e' nemmeno arrivato al waypoint di partenza vale come un tentativo
        # fallito. Lato segnato come provato; la cella esce dalla frontiera solo se era
        # l'ULTIMO lato non provato (cella davvero bloccata).
        if navigation_success and self._tenta(target_row, target_col):
            x_final, y_final, _, _ = spotUtils.getPosition(self.robot_state_client)
            self.frontier.remove((target_row, target_col, rank))
            self._visitata(target_row, target_col, x_final, y_final)
        elif cell_attempt._batteria_scarica():
            pass
        elif self._lato_fallito(target_row, target_col, nearest_cell[0], nearest_cell[1]):
            self.frontier.remove((target_row, target_col, rank))
        return None

    def scansione_finale(self):
        """L'area attorno all'ultima posizione, prima di tornare: entra nelle mappe globali."""
        print("[FINAL SCAN] Ultima scansione attorno al robot prima di tornare alla partenza...")
        x_final, y_final, z_final, quat_final = spotUtils.getPosition(self.robot_state_client)
        final_yaw = np.arctan2(2.0 * (quat_final.w * quat_final.z + quat_final.x * quat_final.y),
                               1.0 - 2.0 * (quat_final.y ** 2 + quat_final.z ** 2))

        scan = self.local_grid.fetch_scan(self.robot_state_client)
        if scan is None:
            print("[FINAL SCAN] ATTENZIONE: griglia locale non ricevuta, proseguo senza scansione finale.")
            return
        self.local_grid.process_scan(scan, x_final, y_final, final_yaw, ground_label="scan-finale")
        # Anche l'ultima scansione entra nelle mappe globali, come tutte le altre
        global_maps.fold_scan_into_global_maps(
            self.global_grid.global_occupancy_map, self.global_grid.global_terrain_map, scan.pts,
            scan.terrain, scan.obstacle_mask, scan.is_valid, scan.od,
            (scan.num_y, scan.num_x), (x_final, y_final, final_yaw))

        final_save_path = os.path.join(
            self.mission_folder, f"iteration_{self.visualization_counter}_FINAL_SCAN.png"
        ) if self.mission_folder else None

        visualize_grid_with_candidates(
            pts=scan.pts, terrain_real=scan.terrain, obstacle_mask=scan.obstacle_mask,
            robot_x=x_final, robot_y=y_final,
            candidates={'rejected': [], 'valid': []},
            chosen_point=None, iteration=self.visualization_counter, env=self.env,
            save_path=final_save_path, prm_graph=self.prm, chosen_path=None,
            cells_obstacle_dist=scan.od, valid_values=scan.valid,
            grad_values=scan.grad, rough_values=scan.rough
        )
        self.visualization_counter += 1
        print("[FINAL SCAN] Fatto: l'ultima scansione e' nelle mappe globali.")
