"""
Probabilistic Road Map (PRM) for Spot autonomous mission.

Builds a graph connecting sampled waypoints with collision-free edges.
Edges are weighted with costs based on distance and terrain slope/gradient.
"""

import numpy as np
from typing import List, Tuple, Dict, Set, Optional
import heapq
import threading


class PRM:
    """
    Probabilistic Road Map for local path planning.

    Connects pre-sampled points (from global sampler) using collision-free edges.
    Uses Dijkstra's algorithm for pathfinding with slope and distance weights.
    """

    def __init__(self, max_edge_length: float = 2.0, connection_radius: float = 3.0,
                 min_edge_length: float = 0.5, alpha: float = 1.0, beta: float = 2.0, gamma: float = 0.0):
        # gamma al momento 0 per non dare peso alla roughness, da testare
        """
        Initialize the PRM.

        Args:
            max_edge_length: Maximum length of an edge in the graph (m)
            connection_radius: Only consider points within this radius for connection (m)
            alpha: Peso associato alla componente di distanza pura nella funzione di costo.
            beta: Peso associato alla componente di pendenza nella funzione di costo.
            gamma: Peso associato alla componente di rugosità nella funzione di costo.
        """
        self.max_edge_length = max_edge_length
        self.connection_radius = connection_radius
        self.min_edge_length = min_edge_length

        # Parametri della funzione di costo
        self.alpha = alpha  # Peso distanza
        self.beta = beta  # Peso pendenza (gradient)
        self.gamma = gamma  # Peso rugosità (roughness)

        self.nodes = {}  # {point_idx: (x, y)}
        self.node_gradients = {}  # {point_idx: slope_value} <-- Memorizza la pendenza del nodo
        self.node_roughness = {}  # <-- Memorizza la rugosità del nodo
        self.edges = {}  # {point_idx: [(neighbor_idx, weight), ...]}
        self.edge_validity = {}  # {(idx1, idx2): bool} - caches edge validity

        # Variabili di appoggio per la griglia locale live
        self.current_grad_2d = None
        self.current_rough_2d = None
        self.grid_origin_x = None
        self.grid_origin_y = None
        self.cell_size = None

        self.built = False

        self.tracker_blocked_edges = set()   # <-- ADD, alongside self.edge_validity = {}

    def add_node(self, point_idx: int, x: float, y: float, gradient: float = 0.0, roughness: float = 0.0):
        """Add a node (waypoint) to the PRM along with its terrain gradient."""
        self.nodes[point_idx] = (x, y)
        self.node_gradients[point_idx] = gradient
        self.node_roughness[point_idx] = roughness
        self.edges[point_idx] = []

    def add_nodes_from_sampler(self, global_sampler, env=None, grad_values=None, rough_values=None):
        """
        Populate PRM with nodes from global sampler and map their gradients.

        Args:
            global_sampler: GlobalSampler instance with sampled points
            env: L'istanza dell'ambiente (EnvironmentMap) per convertire le coordinate in celle della griglia
            grad_values: La griglia 2D o array flat contenente i gradienti calcolati da spotGrid
            rough_values: La griglia 2D o array flat contenente la rugosità calcolata da spotGrid
        """

        # 1. Check della forma degli array PRIMA del loop
        is_grad_2d = grad_values is not None and len(grad_values.shape) == 2
        is_rough_2d = rough_values is not None and len(rough_values.shape) == 2

        points = global_sampler.get_all_points()
        for idx, (x, y) in enumerate(points):
            node_grad = 0.0
            node_rough = 0.0

            if env is not None:
                cell = env.get_cell_from_world(x, y)
                if cell is not None:
                    r, c = cell

                    # 2. Estrazione ottimizzata
                    if grad_values is not None:
                        if is_grad_2d:
                            node_grad = grad_values[r, c]
                        else:
                            idx_flat = r * env.cols + c
                            if idx_flat < grad_values.size:
                                node_grad = grad_values[idx_flat]

                    if rough_values is not None:
                        if is_rough_2d:
                            node_rough = rough_values[r, c]
                        else:
                            idx_flat = r * env.cols + c
                            if idx_flat < rough_values.size:
                                node_rough = rough_values[idx_flat]

            self.add_node(idx, x, y, gradient=node_grad, roughness=node_rough)

        print(f"[PRM] Added {len(self.nodes)} nodes from global sampler with gradient and roughness mapping")

    def update_local_grid_data(self, grad_values_2d, rough_values_2d, grid_origin_x, grid_origin_y, cell_size):
        """
        Salva temporaneamente la mappa locale corrente dei gradienti e delle rugosità
        per permettere il campionamento accurato degli archi.
        """
        self.current_grad_2d = grad_values_2d
        self.current_rough_2d = rough_values_2d
        self.grid_origin_x = grid_origin_x
        self.grid_origin_y = grid_origin_y
        self.cell_size = cell_size

    def refresh_local_edge_weights(self, global_map=None, edge_safety_margin: float = 0.0):
        """
        Fully re-derives all candidate edges among nodes currently inside the local
        grid window -- geometry, occupancy, and weight -- rather than only updating
        weights of edges that already survived a previous pass. This lets an edge
        wrongly invalidated by a transient/noisy occupancy reading become valid
        again once fresher data shows it's actually clear, instead of staying
        permanently removed for the rest of the attempt.
        Bounded to local_nodes x local_nodes, so still much cheaper than build_graph().
        """
        if self.current_grad_2d is None or self.grid_origin_x is None or self.cell_size is None:
            return set()

        num_rows, num_cols = self.current_grad_2d.shape
        local_nodes = []
        for idx, (x, y) in self.nodes.items():
            dx = x - self.grid_origin_x
            dy = y - self.grid_origin_y
            col_idx = int(dx / self.cell_size)
            row_idx = int(dy / self.cell_size)
            if 0 <= row_idx < num_rows and 0 <= col_idx < num_cols:
                local_nodes.append(idx)

        for i, idx1 in enumerate(local_nodes):
            x1, y1 = self.nodes[idx1]
            for idx2 in local_nodes[i + 1:]:
                x2, y2 = self.nodes[idx2]
                dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

                if not (self.connection_radius >= dist >= self.min_edge_length and dist <= self.max_edge_length):
                    continue

                key = (min(idx1, idx2), max(idx1, idx2))
                if key in self.tracker_blocked_edges:
                    continue  # a verified real obstacle (arc-tracker), not just occupancy -- never re-add

                blocked = False
                if global_map is not None and edge_safety_margin > 0.0:
                    num_samples = max(5, int(dist * 5))  # denser sampling, matches _compute_edge_weight
                    for k in range(num_samples):
                        t = k / (num_samples - 1)
                        sx = x1 + t * (x2 - x1)
                        sy = y1 + t * (y2 - y1)
                        if global_map.is_occupied(sx, sy, safety_margin=edge_safety_margin):
                            blocked = True
                            break

                self.edges[idx1] = [(n, w) for n, w in self.edges.get(idx1, []) if n != idx2]
                self.edges[idx2] = [(n, w) for n, w in self.edges.get(idx2, []) if n != idx1]

                if blocked:
                    self.edge_validity[key] = False
                else:
                    weight = self._compute_edge_weight(idx1, idx2, dist)
                    self.edges[idx1].append((idx2, weight))
                    self.edges[idx2].append((idx1, weight))
                    self.edge_validity[key] = True

        return set(local_nodes)

    def _compute_edge_weight(self, idx1: int, idx2: int, dist: float) -> float:
        """
        Funzione di costo multi-layer che campiona l'arco in più punti
        per rilevare pendenze o rugosità nascoste nel mezzo.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]

        # Campioniamo l'arco (es. circa 5 punti per metro, minimo 5 punti totali)
        num_samples = max(5, int(dist * 5))

        sampled_gradients = []
        sampled_roughness = []

        # Verifichiamo se abbiamo a disposizione la griglia locale live
        has_grid = (self.current_grad_2d is not None and
                    self.current_rough_2d is not None and
                    self.grid_origin_x is not None)

        if has_grid:
            num_rows, num_cols = self.current_grad_2d.shape
            for i in range(num_samples):
                t = i / (num_samples - 1)
                cx = x1 + t * (x2 - x1)
                cy = y1 + t * (y2 - y1)

                # Calcolo indici di cella basato sull'origine reale della griglia
                dx = cx - self.grid_origin_x
                dy = cy - self.grid_origin_y
                col_idx = int(dx / self.cell_size)
                row_idx = int(dy / self.cell_size)

                # Controllo confini della matrice locale
                if 0 <= row_idx < num_rows and 0 <= col_idx < num_cols:
                    sampled_gradients.append(self.current_grad_2d[row_idx, col_idx])
                    sampled_roughness.append(self.current_rough_2d[row_idx, col_idx])

        # Se abbiamo campionato dei punti validi nella griglia, prendiamo un valore rappresentativo
        # Altrimenti, facciamo fallback sul valore statico memorizzato precedentemente nei nodi
        if sampled_gradients:
            # Invece del max assoluto (sensibilissimo al rumore), usiamo il 90-esimo percentile.
            # Significa dire: "voglio la pendenza massima, ignorando però il 10% di picchi isolati (rumore)"
            actual_gradient = np.percentile(sampled_gradients, 90)
            actual_roughness = np.percentile(sampled_roughness, 90) if sampled_roughness else 0.0
        else:
            # Fallback statico basato sulla media dei nodi (se fuori griglia locale)
            grad1 = self.node_gradients.get(idx1, 0.0)
            grad2 = self.node_gradients.get(idx2, 0.0)
            actual_gradient = (grad1 + grad2) / 2.0

            r1 = self.node_roughness.get(idx1, 0.0)
            r2 = self.node_roughness.get(idx2, 0.0)
            actual_roughness = (r1 + r2) / 2.0

        # Calcolo costo esteso
        weight = (self.alpha * dist) + (self.beta * actual_gradient) + (self.gamma * actual_roughness)
        return float(weight)

    def get_nearest_node(self, x: float, y: float) -> Optional[int]:
        """Find the ID of the nearest node in the PRM to the given coordinates."""
        if not self.nodes:
            return None
        min_dist = float('inf')
        nearest_idx = None
        for idx, (nx, ny) in self.nodes.items():
            dist = np.sqrt((nx - x)**2 + (ny - y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx
        return nearest_idx

    def build_graph(self, global_map=None, edge_safety_margin: float = 0.0):
        """
        Build the PRM graph by connecting nearby nodes using custom edge weights.
        Se global_map è fornita, scarta gli archi che attraversano celle globalmente occupate.
        """
        print(f"[PRM] Building graph with {len(self.nodes)} nodes...")

        # Reset archi
        for idx in self.nodes.keys():
            self.edges[idx] = []
        self.edge_validity = {}

        node_ids = list(self.nodes.keys())

        for i, idx1 in enumerate(node_ids):
            x1, y1 = self.nodes[idx1]

            for idx2 in node_ids[i + 1:]:
                x2, y2 = self.nodes[idx2]

                dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

                if self.connection_radius >= dist >= self.min_edge_length and dist <= self.max_edge_length:
                    # Se abbiamo una mappa globale, controlliamo che l'arco non passi in zone occupate
                    if global_map is not None and edge_safety_margin > 0.0:
                        # campioniamo qualche punto lungo l'arco
                        num_samples = max(3, int(dist * 3))
                        blocked = False
                        for k in range(num_samples):
                            t = k / (num_samples - 1)
                            sx = x1 + t * (x2 - x1)
                            sy = y1 + t * (y2 - y1)
                            if global_map.is_occupied(sx, sy, safety_margin=edge_safety_margin):
                                blocked = True
                                break
                        if blocked:
                            # non aggiungiamo l'arco
                            continue

                    weight = self._compute_edge_weight(idx1, idx2, dist)

                    self.edges[idx1].append((idx2, weight))
                    self.edges[idx2].append((idx1, weight))

                    self.edge_validity[(idx1, idx2)] = True
                    self.edge_validity[(idx2, idx1)] = True

        self.built = True

        total_edges = sum(len(neighbors) for neighbors in self.edges.values()) // 2
        print(f"[PRM] Built graph with {total_edges} edges using Slope-Distance cost function")

    def mark_edge_invalid(self, idx1: int, idx2: int):
        """Mark an edge as blocked (verified obstacle, e.g. from arc-tracker detection)."""
        key = (min(idx1, idx2), max(idx1, idx2))
        self.edge_validity[key] = False
        self.tracker_blocked_edges.add(key)   # <-- ADD: permanent, never re-added by refresh_local_edge_weights

        if idx2 in [n[0] for n in self.edges.get(idx1, [])]:
            self.edges[idx1] = [(n, w) for n, w in self.edges[idx1] if n != idx2]
        if idx1 in [n[0] for n in self.edges.get(idx2, [])]:
            self.edges[idx2] = [(n, w) for n, w in self.edges[idx2] if n != idx1]

    def is_edge_valid(self, idx1: int, idx2: int) -> bool:
        """Check if an edge is assumed valid (not yet marked as blocked)."""
        key = (min(idx1, idx2), max(idx1, idx2))
        return self.edge_validity.get(key, False)

    def compute_path_cost(self, path_ids):
        """Sum of edge weights along a sequence of node ids, using the graph's current weights."""
        if path_ids is None or len(path_ids) < 2:
            return 0.0
        total = 0.0
        for i in range(len(path_ids) - 1):
            a, b = path_ids[i], path_ids[i + 1]
            weight = None
            for n, w in self.edges.get(a, []):
                if n == b:
                    weight = w
                    break
            if weight is None:
                return float('inf')  # edge no longer exists (e.g. invalidated)
            total += weight
        return total

    def find_path_dijkstra(self, start_idx: int, goal_idx: int,
                            current_path_ids: Optional[List[int]] = None,
                            margin: float = 0.0) -> Optional[List[int]]:
        """
        Find shortest path between two nodes using Dijkstra's algorithm.

        If current_path_ids is given (a plan already being followed, starting at start_idx)
        and margin > 0, the newly-computed path is only returned if it's at least `margin`
        fraction cheaper than sticking with current_path_ids -- this prevents oscillation
        between near-equal routes caused by noisy per-step weight updates. Otherwise
        (current_path_ids=None, the default), behaves exactly as a plain shortest-path search.
        """
        if start_idx not in self.nodes or goal_idx not in self.nodes:
            return None

        distances = {node: float('inf') for node in self.nodes.keys()}
        distances[start_idx] = 0
        parents = {node: None for node in self.nodes.keys()}

        pq = [(0, start_idx)]
        visited = set()
        new_path = None

        while pq:
            current_dist, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_idx:
                path = []
                node = goal_idx
                while node is not None:
                    path.append(node)
                    node = parents[node]
                new_path = path[::-1]
                break

            for neighbor, weight in self.edges.get(current, []):
                if neighbor not in visited:
                    new_dist = current_dist + weight
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        parents[neighbor] = current
                        heapq.heappush(pq, (new_dist, neighbor))

        # --- Plain behavior: no stability check requested ---
        if current_path_ids is None or margin <= 0.0:
            return new_path

        # --- Stability check ---
        current_cost = self.compute_path_cost(current_path_ids)

        if new_path is None:
            return current_path_ids if current_cost != float('inf') else None

        if current_cost == float('inf'):
            return new_path  # current plan is broken (an edge got invalidated) -> must switch

        new_cost = self.compute_path_cost(new_path)
        if new_cost < current_cost * (1.0 - margin):
            return new_path

        return current_path_ids

    def get_node_position(self, node_idx: int) -> Optional[Tuple[float, float]]:
        """Get the (x, y) position of a node."""
        return self.nodes.get(node_idx)

    def get_node_neighbors(self, node_idx: int) -> List[int]:
        """Get indices of neighboring nodes."""
        return [idx for idx, _ in self.edges.get(node_idx, [])]

    def add_temporary_node(self, x: float, y: float, current_gradient: float = 0.0, current_roughness: float = 0.0) -> int:
        """
        Add a temporary node (e.g., current robot position) for pathfinding.

        Returns:
        Temporary node index (negative value)
        """
        temp_idx = -(len(self.nodes) + 1)
        self.nodes[temp_idx] = (x, y)
        self.node_gradients[temp_idx] = current_gradient  # Salva gradiente temporaneo
        self.node_roughness[temp_idx] = current_roughness
        self.edges[temp_idx] = []

        # Connect to nearby permanent nodes
        for perm_idx, (px, py) in self.nodes.items():
            if perm_idx >= 0: # Only permanent nodes
                dist = np.sqrt((px - x)**2 + (py - y)**2)
                if dist <= self.connection_radius:
                    # Calcolo dinamico del costo anche per i nodi temporanei inseriti al volo
                    weight = self._compute_edge_weight(temp_idx, perm_idx, dist)
                    self.edges[temp_idx].append((perm_idx, weight))
                    self.edges[perm_idx].append((temp_idx, weight))
                    key = (min(temp_idx, perm_idx), max(temp_idx, perm_idx))
                    self.edge_validity[key] = True

        return temp_idx

    def remove_temporary_node(self, temp_idx: int):
        """Remove a temporary node and its edges."""
        if temp_idx in self.nodes:

            # 1. Rimuoviamo gli archi di ritorno (Unpacking corretto della tupla!)
            for neighbor_idx, weight in list(self.edges.get(temp_idx, [])):
                if neighbor_idx in self.edges:
                    self.edges[neighbor_idx] = [(n, w) for n, w in self.edges[neighbor_idx] if n != temp_idx]

                key = (min(temp_idx, neighbor_idx), max(temp_idx, neighbor_idx))
                self.edge_validity.pop(key, None)

            # 2. Rimuoviamo le coordinate fisiche
            del self.nodes[temp_idx]

            # 3. Rimuoviamo il nodo dalla lista degli archi
            if temp_idx in self.edges:
                del self.edges[temp_idx]

            # 4. Puliamo i dati ambientali legati al nodo in modo sicuro
            if temp_idx in self.node_gradients:
                del self.node_gradients[temp_idx]
            if temp_idx in self.node_roughness:
                del self.node_roughness[temp_idx]

