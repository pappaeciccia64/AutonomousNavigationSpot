"""
Probabilistic Road Map (PRM) for Spot autonomous mission.

Builds a graph connecting sampled waypoints with collision-free edges.
Edges are weighted with costs based on distance and terrain slope/gradient.
"""

import numpy as np
from typing import List, Tuple, Dict, Set, Optional
import heapq


class PRM:
    """
    Probabilistic Road Map for local path planning.

    Connects pre-sampled points (from global sampler) using collision-free edges.
    Uses Dijkstra's algorithm for pathfinding with slope and distance weights.
    """

    def __init__(self, max_edge_length: float = 2.0, connection_radius: float = 3.0,
                 alpha: float = 1.0, beta: float = 5.0, gamma: float = 0.0):
        # gamma al momento 0 per non dare peso alla roughness, da testare
        """
        Initialize the PRM.

        Args:
            max_edge_length: Maximum length of an edge in the graph (m)
            connection_radius: Only consider points within this radius for connection (m)
            alpha: Peso associato alla componente di distanza pura nella funzione di costo.
            beta: Peso associato alla componente di pendenza nella funzione di costo.
        """
        self.max_edge_length = max_edge_length
        self.connection_radius = connection_radius

        # Parametri della funzione di costo
        self.alpha = alpha  # Peso distanza
        self.beta = beta  # Peso pendenza (gradient)
        self.gamma = gamma  # Peso rugosità (roughness)

        self.nodes = {}  # {point_idx: (x, y)}
        self.node_gradients = {}  # {point_idx: slope_value} <-- [NEW] Memorizza la pendenza del nodo
        self.node_roughness = {}  # <-- [NEW] Memorizza la rugosità del nodo
        self.edges = {}  # {point_idx: [(neighbor_idx, weight), ...]}
        self.edge_validity = {}  # {(idx1, idx2): bool} - caches edge validity

        self.built = False

    def add_node(self, point_idx: int, x: float, y: float, gradient: float = 0.0):
        """Add a node (waypoint) to the PRM along with its terrain gradient."""
        self.nodes[point_idx] = (x, y)
        self.node_gradients[point_idx] = gradient  # [NEW]
        self.edges[point_idx] = []

    def add_nodes_from_sampler(self, global_sampler, env=None, grad_values=None):
        """
        Populate PRM with nodes from global sampler and map their gradients.

        Args:
            global_sampler: GlobalSampler instance with sampled points
            env: L'istanza dell'ambiente (EnvironmentMap) per convertire le coordinate in celle della griglia
            grad_values: La griglia 2D o array flat contenente i gradienti calcolati da spotGrid
        """
        points = global_sampler.get_all_points()
        for idx, (x, y) in enumerate(points):
            # Recuperiamo il gradiente locale se la griglia dei gradienti è fornita
            node_grad = 0.0
            if env is not None and grad_values is not None:
                cell = env.get_cell_from_world(x, y)
                if cell is not None:
                    r, c = cell
                    # Gestione sia di array flat (128x128 ridotto) che di matrici 2D
                    if len(grad_values.shape) == 2:
                        node_grad = grad_values[r, c]
                    else:
                        # Se è flat, usiamo la shape dell'env per ricostruire l'indice
                        idx_flat = r * env.cols + c
                        if idx_flat < grad_values.size:
                            node_grad = grad_values[idx_flat]

            self.add_node(idx, x, y, gradient=node_grad)

        print(f"[PRM] Added {len(self.nodes)} nodes from global sampler with gradient mapping")

    def _compute_edge_weight(self, idx1: int, idx2: int, dist: float) -> float:
        """
        [NEW METHOD] Funzione di costo personalizzata per calcolare il peso di un arco.
        Combina la lunghezza geometrica e la pendenza media rilevata tra i due nodi.
        """
        grad1 = self.node_gradients.get(idx1, 0.0)
        grad2 = self.node_gradients.get(idx2, 0.0)

        # Usiamo la pendenza massima o media tra i due nodi per penalizzare l'arco
        avg_gradient = (grad1 + grad2) / 2.0

        # Estraiamo le rugosità (default 0.0 se non ancora viste)
        r1 = self.node_roughness.get(idx1, 0.0)
        r2 = self.node_roughness.get(idx2, 0.0)
        avg_roughness = (r1 + r2) / 2.0

        # Funzione di costo Multi-Layer estesa:
        # Costo = alpha*Distanza + beta*Pendenza + gamma*Rugosità
        weight = (self.alpha * dist) + (self.beta * avg_gradient) + (self.gamma * avg_roughness)
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

    def build_graph(self):
        """
        Build the PRM graph by connecting nearby nodes using custom edge weights.
        """
        print(f"[PRM] Building graph with {len(self.nodes)} nodes...")

        node_ids = list(self.nodes.keys())

        for i, idx1 in enumerate(node_ids):
            x1, y1 = self.nodes[idx1]

            for idx2 in node_ids[i+1:]:
                x2, y2 = self.nodes[idx2]

                # Calculate distance
                dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                # Only connect if within radius and shorter than max length
                if dist <= self.connection_radius and dist <= self.max_edge_length:
                    # [UPDATED] Calcolo dinamico del peso basato su pendenza e distanza
                    weight = self._compute_edge_weight(idx1, idx2, dist)

                    # Add bidirectional edges
                    self.edges[idx1].append((idx2, weight))
                    self.edges[idx2].append((idx1, weight))

                    # Cache edge as valid (will be updated during motion)
                    self.edge_validity[(idx1, idx2)] = True
                    self.edge_validity[(idx2, idx1)] = True

        self.built = True

        total_edges = sum(len(neighbors) for neighbors in self.edges.values()) // 2
        print(f"[PRM] Built graph with {total_edges} edges using Slope-Distance cost function")

    def mark_edge_invalid(self, idx1: int, idx2: int):
        """Mark an edge as blocked (collision detected)."""
        key = (min(idx1, idx2), max(idx1, idx2))
        self.edge_validity[key] = False

        # Remove from adjacency lists
        if idx2 in [n[0] for n in self.edges.get(idx1, [])]:
            self.edges[idx1] = [(n, w) for n, w in self.edges[idx1] if n != idx2]
        if idx1 in [n[0] for n in self.edges.get(idx2, [])]:
            self.edges[idx2] = [(n, w) for n, w in self.edges[idx2] if n != idx1]

    def is_edge_valid(self, idx1: int, idx2: int) -> bool:
        """Check if an edge is assumed valid (not yet marked as blocked)."""
        key = (min(idx1, idx2), max(idx1, idx2))
        return self.edge_validity.get(key, False)

    def find_path_dijkstra(self, start_idx: int, goal_idx: int) -> Optional[List[int]]:
        """Find shortest path between two nodes using Dijkstra's algorithm (weights are costs)."""
        if start_idx not in self.nodes or goal_idx not in self.nodes:
            return None

        distances = {node: float('inf') for node in self.nodes.keys()}
        distances[start_idx] = 0
        parents = {node: None for node in self.nodes.keys()}

        pq = [(0, start_idx)]  # (distance/cost, node)
        visited = set()

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
                return path[::-1]

            for neighbor, weight in self.edges.get(current, []):
                if neighbor not in visited:
                    new_dist = current_dist + weight

                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        parents[neighbor] = current
                        heapq.heappush(pq, (new_dist, neighbor))

        return None

    def get_node_position(self, node_idx: int) -> Optional[Tuple[float, float]]:
        """Get the (x, y) position of a node."""
        return self.nodes.get(node_idx)

    def get_node_neighbors(self, node_idx: int) -> List[int]:
        """Get indices of neighboring nodes."""
        return [idx for idx, _ in self.edges.get(node_idx, [])]

    def add_temporary_node(self, x: float, y: float, current_gradient: float = 0.0) -> int:
        """
        Add a temporary node (e.g., current robot position) for pathfinding.
        """
        temp_idx = -(len(self.nodes) + 1)
        self.nodes[temp_idx] = (x, y)
        self.node_gradients[temp_idx] = current_gradient  # [NEW] Salva gradiente temporaneo
        self.edges[temp_idx] = []

        # Connect to nearby permanent nodes
        for perm_idx, (px, py) in self.nodes.items():
            if perm_idx >= 0:
                dist = np.sqrt((px - x)**2 + (py - y)**2)
                if dist <= self.connection_radius:
                    # [UPDATED] Calcolo dinamico del costo anche per i nodi temporanei inseriti al volo
                    weight = self._compute_edge_weight(temp_idx, perm_idx, dist)
                    self.edges[temp_idx].append((perm_idx, weight))
                    self.edges[perm_idx].append((temp_idx, weight))

        return temp_idx

    def remove_temporary_node(self, temp_idx: int):
        """Remove a temporary node and its edges."""
        if temp_idx in self.nodes:
            for neighbor in list(self.edges.get(temp_idx, [])):
                if neighbor in self.edges:
                    self.edges[neighbor] = [(n, w) for n, w in self.edges[neighbor] if n != temp_idx]

            del self.nodes[temp_idx]
            del self.node_gradients[temp_idx]  # [NEW]
            del self.edges[temp_idx]