"""
environmentMap.py -- la mappa a celle della missione.

Una griglia rows x cols di celle quadrate di lato cell_size. Per ogni cella: 0 = non
visitata, 1 = visitata, -1 = bloccata (provata da tutti i lati, senza successo), piu' una
maschera a 4 bit dei lati da cui si e' gia' provato a entrare (Nord 0b1000, Est 0b0100,
Sud 0b0010, Ovest 0b0001).

Il frame della griglia e' fissato all'accensione (set_origin): il centro della cella di
partenza e' nell'origine, l'asse x della griglia e' l'avanti del robot in quel momento
(colonne), l'asse y la sua sinistra (righe). L'ordine di visita e' a serpentina.
"""
import numpy as np

class EnvironmentMap(object):
    def __init__(self, rows=12, cols=12, cell_size=2.0):
        self.map = [[0 for _ in range(cols)] for _ in range(rows)]
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size  # lato della cella, m
        self.origin_x = 0.0  # origine della griglia nel frame del mondo
        self.origin_y = 0.0
        self.origin_yaw = 0.0  # orientamento del robot all'accensione, rad
        self.start_cell = (0, 0)  # cella di partenza (riga, colonna)
        self.current_cell = (0, 0)
        self.explored_sides = {}
        self.waypoints = []

    def mark_cell_visited(self, row, col):
        """Segna la cella come visitata (1). False se e' fuori dalla griglia."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.map[row][col] = 1
            print(f"[VISITED] Cella ({row},{col}) segnata come visitata (valore 1)")
            return True
        else:
            print(f"[ERROR] Impossibile segnare visitata la cella ({row},{col}): fuori dalla griglia")
            return False

    def is_cell_visited(self, row, col):
        """True se la cella e' visitata (1)."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == 1
        return False

    def is_point_in_cell(self, x, y, target_row, target_col):
        """
        True se il punto (x, y) del mondo sta dentro la cella, bordi compresi. Controlla i
        confini veri della cella, ruotati con la griglia.
        """
        cell_center = self.get_world_position_from_cell(target_row, target_col)
        if cell_center is None:
            return False

        center_x, center_y = cell_center

        half_size = self.cell_size / 2.0

        # Punto rispetto al centro della cella, ruotato nel frame della griglia
        delta_x = x - center_x
        delta_y = y - center_y

        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        local_x = delta_x * cos_yaw - delta_y * sin_yaw
        local_y = delta_x * sin_yaw + delta_y * cos_yaw

        is_inside = (abs(local_x) <= half_size and abs(local_y) <= half_size)

        if is_inside:
            print(f"[CHECK] Il punto ({x:.2f},{y:.2f}) e' DENTRO la cella ({target_row},{target_col})")
        else:
            print(f"[CHECK] Il punto ({x:.2f},{y:.2f}) e' FUORI dalla cella ({target_row},{target_col}) di ({local_x:.2f},{local_y:.2f})")

        return is_inside

    def mark_cell_blocked(self, row, col):
        """Segna la cella come bloccata (-1): provata e non raggiungibile."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.map[row][col] = -1
            print(f"[BLOCKED] Cella ({row},{col}) segnata come bloccata (valore -1)")
        else:
            print(f"[ERROR] Impossibile segnare bloccata la cella ({row},{col}): fuori dalla griglia")

    def is_cell_blocked(self, row, col):
        """True se la cella e' bloccata (-1)."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == -1
        return False

    def get_side_bit_facing_origin(self, origin_row, origin_col, target_row, target_col):
        """
        Il robot e' andato (o ha provato) da (origin_row, origin_col) verso (target_row,
        target_col). Restituisce il bit del lato di TARGET rivolto verso origin, cioe' il lato
        provato davvero. 0 se le celle non sono adiacenti.
        """
        dr, dc = target_row - origin_row, target_col - origin_col
        if dr == -1 and dc == 0:
            return 0b0010  # target a nord di origin: si entra dal lato SUD
        if dr == 1 and dc == 0:
            return 0b1000  # target a sud: si entra da NORD
        if dr == 0 and dc == 1:
            return 0b0001  # target a est: si entra da OVEST
        if dr == 0 and dc == -1:
            return 0b0100  # target a ovest: si entra da EST
        return 0b0000

    def get_valid_neighbor_sides_mask(self, row, col):
        """
        Maschera dei lati che hanno davvero una cella vicina dentro la griglia (le celle di
        bordo e d'angolo ne hanno meno di 4): tutti i lati da cui si potrebbe mai provare.
        La usa all_sides_explored per sapere quando una cella ha finito le possibilita'.
        Nord 0b1000, Est 0b0100, Sud 0b0010, Ovest 0b0001.
        """
        mask = 0b0000
        if row - 1 >= 0:
            mask |= 0b1000  # nord
        if row + 1 < self.rows:
            mask |= 0b0010  # sud
        if col + 1 < self.cols:
            mask |= 0b0100  # est
        if col - 1 >= 0:
            mask |= 0b0001  # ovest
        return mask

    def all_sides_explored(self, row, col):
        """
        True solo se si e' gia' provato a entrare da OGNI lato che ha un vicino. Solo allora
        la cella si segna bloccata (-1); prima resta a 0, cosi' l'esplorazione la ripropone
        piu' avanti da un lato non ancora provato invece di escluderla al primo fallimento.
        """
        return (self.get_cell_sides_status(row, col) & self.get_valid_neighbor_sides_mask(row, col)) == \
               self.get_valid_neighbor_sides_mask(row, col)

    def mark_cell_side_explored(self, row, col, side_bit):
        """
        Registra che si e' provato a entrare nella cella dal lato side_bit
        (Nord 0b1000, Est 0b0100, Sud 0b0010, Ovest 0b0001).
        """
        cell_key = (row, col)
        current = self.explored_sides.get(cell_key, 0b0000)
        self.explored_sides[cell_key] = current | side_bit
        print(f"[SIDE] Cella ({row},{col}): lato {bin(side_bit)} segnato come provato "
              f"(ora {bin(self.explored_sides[cell_key])})")

    def get_adjacent_frontier_cells(self, cell_row, cell_col, path):
        """
        Le celle adiacenti (nord, sud, est, ovest) non ancora visitate (valore 0), escluse
        quelle in cui si e' gia' provato a entrare da questo stesso lato.

        path: la serpentina, lista di (riga, colonna); il rango di una cella e' la sua
        posizione in path.

        Restituisce [(riga, colonna, rango), ...] in ordine di rango crescente, [] se non
        ce ne sono.
        """
        if path is None:
            print(f"[ERROR] get_adjacent_frontier_cells richiede il parametro path")
            return []

        print(f"\n[ADJACENT] Cerco le celle confinanti non esplorate attorno a ({cell_row},{cell_col})")

        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        unexplored_neighbors = []
        neighbors = [
            (-1, 0, 'north'),
            (1, 0, 'south'),
            (0, 1, 'east'),
            (0, -1, 'west')
        ]

        for dr, dc, direction in neighbors:
            neighbor_row = cell_row + dr
            neighbor_col = cell_col + dc

            if not (0 <= neighbor_row < self.rows and 0 <= neighbor_col < self.cols):
                continue

            if self.map[neighbor_row][neighbor_col] == 0:
                # La cella puo' essere ancora a 0 ma gia' provata da QUESTO lato senza
                # successo (gli altri lati restano da provare). Non si ripropone subito lo
                # stesso tentativo: tornera' in gioco da un altro lato.
                side_bit = self.get_side_bit_facing_origin(cell_row, cell_col, neighbor_row, neighbor_col)
                if self.get_cell_sides_status(neighbor_row, neighbor_col) & side_bit:
                    print(f"  ✗ Vicina {direction}: ({neighbor_row},{neighbor_col}) gia' provata da "
                          f"questo lato senza successo: per ora la salto")
                    continue

                neighbor_cell = (neighbor_row, neighbor_col)
                rank = cell_to_rank.get(neighbor_cell, float('inf'))
                unexplored_neighbors.append((neighbor_row, neighbor_col, rank))
                print(f"  ✓ Vicina {direction}: ({neighbor_row},{neighbor_col}) rango={rank}")

        if unexplored_neighbors:
            unexplored_neighbors.sort(key=lambda x: x[2])
            print(f"[ADJACENT] ✓ {len(unexplored_neighbors)} celle confinanti non esplorate")
            print(f"[ADJACENT] Quella di rango piu' basso: ({unexplored_neighbors[0][0]},{unexplored_neighbors[0][1]}) rango={unexplored_neighbors[0][2]}")
            return unexplored_neighbors
        else:
            print(f"[ADJACENT] Nessuna cella confinante non esplorata")
            return []

    def get_lowest_rank_from_frontier_list(self, frontier_list, path):
        """
        Fra le celle di frontier_list ((riga, colonna) o (riga, colonna, rango)), quella con
        il rango piu' basso nella serpentina path. Restituisce (riga, colonna, rango), None
        se la lista e' vuota o manca path.
        """
        if path is None:
            print(f"[ERROR] get_lowest_rank_from_frontier_list richiede il parametro path")
            return None

        if not frontier_list or len(frontier_list) == 0:
            print(f"[FRONTIER_RANK] Frontiera vuota")
            return None

        print(f"\n[FRONTIER_RANK] Cerco la cella di rango piu' basso fra {len(frontier_list)} celle di frontiera")

        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        min_rank = float('inf')
        best_cell = None

        for cell in frontier_list:
            # Il rango eventualmente gia' presente si ignora: si rilegge da path.
            if isinstance(cell, tuple):
                if len(cell) >= 2:
                    row, col = cell[0], cell[1]
                    cell_tuple = (row, col)

                    rank = cell_to_rank.get(cell_tuple, float('inf'))

                    print(f"  Cella ({row},{col}): rango={rank}")

                    if rank < min_rank:
                        min_rank = rank
                        best_cell = (row, col, rank)

        if best_cell:
            print(f"[FRONTIER_RANK] ✓ Cella di rango piu' basso: ({best_cell[0]},{best_cell[1]}) rango={best_cell[2]}")
            return best_cell
        else:
            print(f"[FRONTIER_RANK] ⚠️ Nessuna cella valida nella lista")
            return None

    def get_cell_sides_status(self, row, col):
        """Maschera a 4 bit dei lati gia' provati (da 0b0000 a 0b1111)."""
        cell_key = (row, col)
        return self.explored_sides.get(cell_key, 0b0000)

    def set_origin(self, x, y, yaw=0.0, start_row=0, start_col=0):
        """
        Fissa la griglia sulla posa del robot (x, y, yaw nel frame del mondo): il centro di
        (start_row, start_col) cade in (x, y) e la griglia e' allineata con l'orientamento
        yaw. Segna la cella di partenza come visitata.
        """
        self.origin_x = x
        self.origin_y = y
        self.origin_yaw = yaw
        self.start_cell = (start_row, start_col)

        self.map[start_row][start_col] = 1

        print(f"[OK] Origine fissata alle coordinate ({x:.2f}, {y:.2f})")
        print(f"[OK] Orientamento iniziale: {np.rad2deg(yaw):.1f}°")
        print(f"[OK] Cella di partenza ({start_row}, {start_col}) segnata come visitata")

    def update_position(self, x, y):
        """
        Aggiorna la cella corrente dalla posizione (x, y) del robot e la segna visitata se
        era a 0. Restituisce (riga, colonna), oppure None se il centro del robot e' fuori
        dalla griglia.
        """
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y

        # Nel frame della griglia: x = avanti del robot all'accensione (colonne),
        # y = sinistra (righe).
        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        grid_x = delta_x * cos_yaw - delta_y * sin_yaw
        grid_y = delta_x * sin_yaw + delta_y * cos_yaw

        # La cella che contiene il centro del robot, bordi compresi
        half_size = self.cell_size / 2.0

        col = None
        row = None

        for c in range(self.cols):
            cell_center_x = (c - self.start_cell[1]) * self.cell_size
            if abs(grid_x - cell_center_x) <= half_size:
                col = c
                break

        for r in range(self.rows):
            cell_center_y = (r - self.start_cell[0]) * self.cell_size
            if abs(grid_y - cell_center_y) <= half_size:
                row = r
                break

        if row is not None and col is not None:
            if 0 <= row < self.rows and 0 <= col < self.cols:
                if self.map[row][col] == 0:
                    self.map[row][col] = 1
                    print(f"[MAP] Cella ({row}, {col}) segnata come visitata (centro del robot dentro)")

                self.current_cell = (row, col)
                return (row, col)

        print(f"[INFO] Centro del robot in ({x:.2f}, {y:.2f}) -> griglia ({grid_x:.2f}, {grid_y:.2f}): fra due celle o fuori dalla griglia")
        return None

    def get_cell_from_world(self, x, y):
        """
        La cella (riga, colonna) che contiene il punto (x, y) del mondo, senza segnarla
        visitata. Non controlla i limiti: fuori dalla griglia restituisce indici fuori range.
        """
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y

        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        grid_x = delta_x * cos_yaw - delta_y * sin_yaw
        grid_y = delta_x * sin_yaw + delta_y * cos_yaw

        # Centro di cella piu' vicino
        col = self.start_cell[1] + round(grid_x / self.cell_size)
        row = self.start_cell[0] + round(grid_y / self.cell_size)

        return row, col

    def get_cell_status(self, row, col):
        """
        (valore, lati_provati): valore 0 non visitata, 1 visitata, -1 bloccata; lati_provati
        e' la maschera a 4 bit. (None, None) fuori dalla griglia.
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            cell_value = self.map[row][col]
            explored_sides = self.get_cell_sides_status(row, col)
            return cell_value, explored_sides
        return None, None

    def generate_serpentine_path(self, start_cell=None):
        """
        L'ordine di visita a serpentina, lista di (riga, colonna):
            1  2  3  4  5
            10 9  8  7  6
            11 12 13 14 15
        ruotato in modo da partire da start_cell (default: self.start_cell).
        """
        path = []
        for row in range(self.rows):
            if row % 2 == 0:
                # righe pari: da sinistra a destra
                for col in range(self.cols):
                    path.append((row, col))
            else:
                # righe dispari: da destra a sinistra
                for col in range(self.cols - 1, -1, -1):
                    path.append((row, col))

        if not path:
            return path

        effective_start = start_cell if start_cell is not None else self.start_cell
        if effective_start in path:
            start_idx = path.index(effective_start)
            if start_idx > 0:
                path = path[start_idx:] + path[:start_idx]

        return path

    def get_world_position_from_cell(self, row, col):
        """
        Il centro della cella nel frame del mondo, (x, y); None fuori dalla griglia. Il
        centro della cella di partenza e' l'origine, cioe' la posa del robot all'accensione.
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            delta_row = row - self.start_cell[0]
            delta_col = col - self.start_cell[1]

            # Nel frame della griglia: x = colonne, y = righe
            grid_x = delta_col * self.cell_size
            grid_y = delta_row * self.cell_size

            # Nel frame del mondo
            cos_yaw = np.cos(self.origin_yaw)
            sin_yaw = np.sin(self.origin_yaw)

            world_delta_x = grid_x * cos_yaw - grid_y * sin_yaw
            world_delta_y = grid_x * sin_yaw + grid_y * cos_yaw

            x = self.origin_x + world_delta_x
            y = self.origin_y + world_delta_y

            return (x, y)
        return None

    def add_waypoint(self, x, y):
        """Registra un waypoint nel punto (x, y) del mondo."""
        self.waypoints.append((x, y))
        print(f"[WAYPOINT] Registrato il waypoint n. {len(self.waypoints)} in ({x:.3f}, {y:.3f})")

    def print_map(self):
        """Stampa la mappa delle celle."""
        print("\n--- Mappa delle celle ---")
        for row in self.map:
            print(" ".join(str(cell) for cell in row))
        if hasattr(self, 'current_cell'):
            print(f"Cella attuale: {self.current_cell}")
        print("-----------------------\n")
