"""
Mission PRM (Probabilistic Road Map): lattice nodes (mission_graph), valid edges with
their cost, path search (Dijkstra).

An edge is valid if it does not pass near obstacles (live scan and global map) and is not
too steep; the cost is 1.0 * length + 2.0 * longitudinal slope + 2.0 * lateral slope
(+ 0 * roughness). Edges are re-evaluated at cell start (mission_graph,
add_edges_to_prm) and during the move around the robot (refresh_local_edge_weights).

Here: the graph, the cost layer (penalties), attaching the robot to a node, the path search.
The evaluation of the edges is in edge_evaluation.py, the veto statistics in edge_stats.py.
"""

import numpy as np
from typing import List, Tuple, Optional
import heapq
from spot_explorer.perception import slopes
from spot_explorer.planning.edge_evaluation import (OCCUPANCY_SAMPLES_MIN, OCCUPANCY_SAMPLES_PER_M,
                                                    EdgeEvaluation)
from spot_explorer.planning.edge_stats import EdgeStats


# --- Attaching the robot position to the graph ---------------------------------------------
# The graph does not grow during the mission: add_stop_node attaches the position to the
# suitable EXISTING node (see anchor_node).
#   ANCHOR_SEARCH_M: radius in which to look for the node (leaves room to discard those behind
#     an obstacle).
#   ANCHOR_ON_NODE_M: within this distance the robot is "on" the node: only then does a block or
#     a failure on the edge leaving it really concern that graph edge.
#   ANCHOR_IGNORE_NEAR_M: around the robot position the global map does not count for
#     attaching (the robot is there); half width plus margin.
ANCHOR_SEARCH_M = 1.0
ANCHOR_ON_NODE_M = 0.15
ANCHOR_IGNORE_NEAR_M = 0.30

# Around the node the robot is in NOW, the global obstacle map is not used
# for the first STOP_NODE_MAP_IGNORE_M of every edge: there the safe frontier judges on the live
# data. Needed for example by a robot that got up after a fall, which is INSIDE the hazard
# disk (0.50 m) marked around the spot: without it, every outgoing edge would be discarded.
# Applies ONLY to the current node (set_robot_node).
STOP_NODE_MAP_IGNORE_M = 0.70


class PRM(EdgeEvaluation, EdgeStats):
    """
    Mission graph: lattice nodes, valid edges with their cost, Dijkstra.
    """

    def __init__(self, max_edge_length: float = 2.0, connection_radius: float = 3.0,
                 min_edge_length: float = 0.5, alpha: float = 1.0, beta: float = 2.0, gamma: float = 0.0,
                 beta_lateral: float = None):
        """
        Args:
            max_edge_length, min_edge_length: maximum and minimum edge length (m)
            connection_radius: radius within which to look for neighbours (m)
            alpha: weight of the length in the cost
            beta: weight of the LONGITUDINAL slope (pitch)
            gamma: weight of the roughness (0: today roughness does not enter the cost)
            beta_lateral: weight of the LATERAL slope (roll); None = beta *
                slopes.LATERAL_SLOPE_COST_MULTIPLIER
        """
        self.max_edge_length = max_edge_length
        self.connection_radius = connection_radius
        self.min_edge_length = min_edge_length

        # Cost function parameters
        self.alpha = alpha  # Distance weight
        self.beta = beta  # Longitudinal slope weight (pitch)
        self.gamma = gamma  # Roughness weight
        self.beta_lateral = (beta_lateral if beta_lateral is not None
                             else beta * slopes.LATERAL_SLOPE_COST_MULTIPLIER)  # Lateral slope weight (roll)

        # Graph
        self.nodes = {}  # {point_idx: (x, y)}
        self.node_gradients = {}  # {point_idx: mean slope}, used only as a last resort
        self.node_roughness = {}  # {point_idx: mean roughness}, same
        self.edges = {}  # {point_idx: [(neighbor_idx, weight), ...]}
        self.edge_validity = {}  # {(idx1, idx2): bool} - caches edge validity
        self.built = False

        # Edges blocked by a confirmed block or a physical failure: excluded forever
        # (see mark_edge_invalid).
        self.tracker_blocked_edges = set()
        # Edges actually travelled by the robot (free by construction).
        self.traversed_edges = set()
        # Node the robot is in now, and radius within which the global map is not used
        # for the edges leaving it (see STOP_NODE_MAP_IGNORE_M).
        self.robot_node = None
        self.robot_node_ignore_m = 0.0
        # Set when an edge is discarded during the loop: the next turn refreshes the edges
        # (see mark_edge_blocked_soft and CellAttempt._scan_and_graph).
        self.needs_refresh = False

        # Path search: nodes it may pass through (None = all; used to keep planning inside
        # the cells already visited, see set_allowed_nodes) and detour cap (a path longer
        # than max_detour_ratio * straight-line distance + detour_slack_m is "no path";
        # None = no cap, see set_detour_limit).
        self.allowed_nodes = None
        self.max_detour_ratio = None
        self.detour_slack_m = 0.0

        # COST LAYER: what is learned during the mission does not remove edges from the graph,
        # it adds a cost (possibly infinite) that Dijkstra adds to the edge weight. Today it is used
        # for the edges of a trap, forbidden for the current attempt only (_ATTEMPT_MARK).
        self.edge_penalty_persistent = {}   # (min,max) -> additional weight (possibly inf)
        self.edge_penalty_transient = {}
        self.node_penalty_persistent = {}   # idx -> additional weight (possibly inf)
        self.node_penalty_transient = {}

        self._init_edge_evaluation()   # live data, global height map, caches (edge_evaluation.py)
        self._init_stats()             # veto statistics (edge_stats.py)

    def clear_penalties_for_attempt(self):
        """
        End of a cell attempt: the traps applied to that attempt, not to the mission.
        ONLY those recorded as such are removed.
        """
        for k in [k for k, v in self.edge_penalty_persistent.items() if v == self._ATTEMPT_MARK]:
            del self.edge_penalty_persistent[k]
        for k in [k for k, v in self.node_penalty_persistent.items() if v == self._ATTEMPT_MARK]:
            del self.node_penalty_persistent[k]

    # Value used to mark "impassable, but only for this cell attempt".
    # It is a distinguishable infinity: it behaves as inf in the cost, and clear_penalties_for_attempt
    # recognizes exactly these entries.
    _ATTEMPT_MARK = float('inf')

    # ---- cost layer: reading ---------------------------------------------------------
    def node_penalty(self, idx: int) -> float:
        return (self.node_penalty_persistent.get(idx, 0.0)
                + self.node_penalty_transient.get(idx, 0.0))

    def edge_penalty(self, idx1: int, idx2: int) -> float:
        key = (min(idx1, idx2), max(idx1, idx2))
        return (self.edge_penalty_persistent.get(key, 0.0)
                + self.edge_penalty_transient.get(key, 0.0))

    def effective_weight(self, idx1: int, idx2: int, weight: float) -> float:
        """Geometric cost + edge penalty + arrival node penalty."""
        return weight + self.edge_penalty(idx1, idx2) + self.node_penalty(idx2)

    def add_node(self, point_idx: int, x: float, y: float, gradient: float = 0.0, roughness: float = 0.0):
        """Add a node with its mean slope and roughness (used only as a last resort)."""
        self.nodes[point_idx] = (x, y)
        self.node_gradients[point_idx] = gradient
        self.node_roughness[point_idx] = roughness
        self.edges[point_idx] = []

    def set_robot_node(self, node_id, ignore_map_within_m: float = STOP_NODE_MAP_IGNORE_M):
        """Record the node the robot is in now (see STOP_NODE_MAP_IGNORE_M)."""
        self.robot_node = node_id
        self.robot_node_ignore_m = float(ignore_map_within_m)

    def set_allowed_nodes(self, node_ids=None):
        """
        Limit the path search to these nodes (None = no limit). Start and end are always
        allowed. Used by mission_graph.restrict_planning_to_explored_cells.
        """
        self.allowed_nodes = None if node_ids is None else set(node_ids)

    def set_detour_limit(self, max_ratio: Optional[float] = None, slack_m: float = 0.0):
        """Cap on path length relative to the straight line (None = none)."""
        self.max_detour_ratio = None if max_ratio is None else float(max_ratio)
        self.detour_slack_m = float(slack_m)

    def path_length(self, path_ids) -> float:
        """Geometric length of a path, in metres."""
        if not path_ids or len(path_ids) < 2:
            return 0.0
        xy = np.array([self.nodes[i] for i in path_ids], dtype=np.float64)
        return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))

    def anchor_node(self, x: float, y: float, global_map=None, edge_safety_margin: float = 0.0,
                    ignore_near_m: float = 0.0) -> Optional[int]:
        """
        The EXISTING node to attach the position (x, y) to, without creating new ones.

        Not simply the nearest (after a stop halfway along an edge the nearest may lie beyond
        what stopped the robot): the nearest, within ANCHOR_SEARCH_M, that
          - has no infinite penalty;
          - is reachable in a straight line without passing over occupied cells of the global
            map (the first ignore_near_m around (x, y) do not count: the robot is there).
        If none passes the checks, the nearest overall: the frontier decides anyway.
        """
        if not self.nodes:
            return None
        ids = np.fromiter(self.nodes.keys(), dtype=np.int64, count=len(self.nodes))
        xy = np.array([self.nodes[int(i)] for i in ids], dtype=np.float64)
        d = np.hypot(xy[:, 0] - x, xy[:, 1] - y)
        order = np.argsort(d, kind='stable')
        for k in order:
            if d[k] > ANCHOR_SEARCH_M:
                break
            nid = int(ids[k])
            if not np.isfinite(self.node_penalty(nid)):
                continue
            if global_map is not None and edge_safety_margin > 0.0 and d[k] > 1e-6:
                nx, ny = xy[k]
                n_s = max(OCCUPANCY_SAMPLES_MIN, int(np.ceil(d[k] * OCCUPANCY_SAMPLES_PER_M)) + 1)
                blocked = False
                for j in range(n_s):
                    t = j / (n_s - 1)
                    sx, sy = x + t * (nx - x), y + t * (ny - y)
                    if np.hypot(sx - x, sy - y) <= ignore_near_m:
                        continue
                    if global_map.is_occupied(sx, sy, safety_margin=edge_safety_margin):
                        blocked = True
                        break
                if blocked:
                    continue
            return nid
        return int(ids[order[0]])

    def add_stop_node(self, x: float, y: float, global_map=None, edge_safety_margin: float = 0.0,
                      label: str = "stop", is_robot: bool = True) -> int:
        """
        The EXISTING node to attach the position (x, y) to (see anchor_node): the graph does
        not grow during the mission. If is_robot, it also becomes the robot node.
        """
        nid = self.anchor_node(x, y, global_map, edge_safety_margin,
                               ignore_near_m=(ANCHOR_IGNORE_NEAR_M if is_robot else 0.0))
        if nid is None:
            raise RuntimeError("add_stop_node: the graph has no nodes")
        if is_robot:
            self.set_robot_node(nid)
        nx, ny = self.nodes[nid]
        print(f"[NODE] {label}: attached to node {nid} at ({nx:.2f}, {ny:.2f}), "
              f"{float(np.hypot(nx - x, ny - y)):.2f} m from ({x:.2f}, {y:.2f}) -- no new node")
        return nid

    def mark_edge_invalid(self, idx1: int, idx2: int):
        """
        Mark an edge as blocked for the rest of the mission: no evaluation puts it back.
        Called by the main loop after a block confirmed UP CLOSE by the safe frontier or a
        physical failure of the move.
        """
        key = (min(idx1, idx2), max(idx1, idx2))
        self.edge_validity[key] = False
        self.tracker_blocked_edges.add(key)   # forever: no refresh puts it back
        self.traversed_edges.discard(key)     # travelled in the past does not mean travelable now

        if idx2 in [n[0] for n in self.edges.get(idx1, [])]:
            self.edges[idx1] = [(n, w) for n, w in self.edges[idx1] if n != idx2]
        if idx1 in [n[0] for n in self.edges.get(idx2, [])]:
            self.edges[idx2] = [(n, w) for n, w in self.edges[idx2] if n != idx1]

    def compute_path_cost(self, path_ids, allow_direct_first_hop: bool = False):
        """
        Sum of the weights (with the penalties) along a sequence of nodes.

        allow_direct_first_hop: the first stretch, from the robot to the first waypoint, may
        not be a graph edge (after a SHORTCUT the robot goes straight to a node further
        ahead, on a stretch confirmed free by the frontier): it is counted as alpha * distance
        instead of as a broken plan.
        """
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
            key = (min(a, b), max(a, b))
            if (weight is None and i == 0 and allow_direct_first_hop and a in self.nodes and b in self.nodes
                    and key not in self.tracker_blocked_edges and self.edge_validity.get(key) is not False):
                # Only for a stretch that has NEVER been discarded: an edge just removed from the
                # graph (occupancy, slope, hazard, block) must trigger a replan.
                (xa, ya), (xb, yb) = self.nodes[a], self.nodes[b]
                weight = self.alpha * float(np.hypot(xb - xa, yb - ya))
            if weight is None:
                return float('inf')  # edge no longer exists (e.g. invalidated)
            # Same costs used by find_path_dijkstra: otherwise the stability check
            # would compare a penalized plan with an unpenalized one.
            total += self.effective_weight(a, b, weight)
        return total

    def find_path_dijkstra(self, start_idx: int, goal_idx: int,
                            current_path_ids: Optional[List[int]] = None,
                            margin: float = 0.0,
                            max_edges: Optional[int] = None,
                            ignore_penalties: bool = False) -> Optional[List[int]]:
        """
        Shortest path between two nodes (Dijkstra), inside allowed_nodes and with the detour
        cap (set_detour_limit). None if there is none.

        With current_path_ids (the plan being followed, starting from start_idx) and margin > 0,
        the new path is returned only if it costs at least `margin` less: avoids oscillating
        between almost equivalent ways because of weight noise. If it keeps the old plan it
        returns current_path_ids unchanged: for the caller it is "stable plan", not "no
        path".
        """
        if start_idx not in self.nodes or goal_idx not in self.nodes:
            return None
        new_path = self._dijkstra(start_idx, goal_idx, ignore_penalties)
        if new_path is not None and not self._within_caps(new_path, max_edges):
            new_path = None
        if current_path_ids is None or margin <= 0.0:
            return new_path
        return self._stable_plan(current_path_ids, new_path, margin)

    def _dijkstra(self, start_idx, goal_idx, ignore_penalties=False):
        """Minimum-cost path start -> goal (list of ids), None if there is none."""
        distances = {node: float('inf') for node in self.nodes.keys()}
        distances[start_idx] = 0
        parents = {node: None for node in self.nodes.keys()}
        pq = [(0, start_idx)]
        visited = set()
        allowed = self.allowed_nodes

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
                if (allowed is not None and neighbor not in allowed
                        and neighbor != goal_idx):
                    continue          # outside the allowed cells (see set_allowed_nodes)
                if neighbor not in visited:
                    # Cost = geometric + cost layer: an impassable edge or node has
                    # infinite penalty and Dijkstra does not cross it.
                    w = weight if ignore_penalties else self.effective_weight(current, neighbor, weight)
                    if not np.isfinite(w):
                        continue
                    new_dist = current_dist + w
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        parents[neighbor] = current
                        heapq.heappush(pq, (new_dist, neighbor))
        return None

    def _within_caps(self, path, max_edges):
        """
        Caps checked on the path FOUND: if the minimum-cost path has too many edges, or is a
        loop too long compared with the straight line (set_detour_limit), the answer is "no
        path": the cell is given up and retried later from another side.
        """
        if max_edges is not None and len(path) - 1 > max_edges:
            return False
        if self.max_detour_ratio is not None:
            start_idx, goal_idx = path[0], path[-1]
            (sx, sy), (gx, gy) = self.nodes[start_idx], self.nodes[goal_idx]
            limit = self.max_detour_ratio * float(np.hypot(gx - sx, gy - sy)) + self.detour_slack_m
            plen = self.path_length(path)
            if plen > limit:
                print(f"[DETOUR] Path {start_idx}->{goal_idx} {plen:.1f} m long, beyond "
                      f"the cap of {limit:.1f} m: I consider it impracticable.")
                return False
        return True

    def _stable_plan(self, current_path_ids, new_path, margin):
        """Stability check: the new path only if it costs at least `margin` less than the plan
        being followed, or if that plan is broken (an edge got invalidated)."""
        current_cost = self.compute_path_cost(current_path_ids, allow_direct_first_hop=True)
        if new_path is None:
            return current_path_ids if current_cost != float('inf') else None
        if current_cost == float('inf'):
            return new_path
        new_cost = self.compute_path_cost(new_path)
        if new_cost < current_cost * (1.0 - margin):
            return new_path
        return current_path_ids

    def reachable_count(self, start_idx: int, limit: int) -> int:
        """
        How many nodes are reachable from start_idx with the same rules as find_path_dijkstra
        (edges with finite cost, inside allowed_nodes), stopping at `limit`. Used to understand,
        when a goal no longer has a path, whether the goal or the robot is isolated.
        """
        if start_idx not in self.nodes:
            return 0
        allowed = self.allowed_nodes
        seen, stack = {start_idx}, [start_idx]
        while stack and len(seen) < limit:
            u = stack.pop()
            for v, w in self.edges.get(u, []):
                if v in seen or (allowed is not None and v not in allowed):
                    continue
                if not np.isfinite(self.effective_weight(u, v, w)):
                    continue
                seen.add(v)
                stack.append(v)
        return min(len(seen), limit)

    def get_node_position(self, node_idx: int) -> Optional[Tuple[float, float]]:
        """Node position (x, y), None if it does not exist."""
        return self.nodes.get(node_idx)


# =========================================================================================
# GRAPH FUNCTIONS used by the navigation loop
# =========================================================================================
def mark_edge_blocked_soft(prm, idx1, idx2):
    """
    Remove an edge from the graph NOW, without condemning it forever.

    PRM.mark_edge_invalid() also puts it in `tracker_blocked_edges`, which is NEVER
    recreated -- neither by the refresh nor by the edge rebuild. Right for a block
    confirmed by three still scans; wrong for [SUBITO], which discards after ONE
    scan: a marginal obstacle seen once would isolate a node for the whole mission.

    Here only the edge and its validity are removed: at the next re-evaluation, if the fresh
    data says it is free, the edge comes back -- which is exactly the return behaviour
    refresh_local_edge_weights was written to have.
    """
    key = (min(idx1, idx2), max(idx1, idx2))
    # Discarding an edge changes the plan, so at the next turn the edge refresh must be
    # done (see EVENT REFRESH in cell_attempt.CellAttempt._scan_and_graph).
    prm.needs_refresh = True
    prm.edge_validity[key] = False
    prm.traversed_edges.discard(key)
    prm.edges[idx1] = [(n, w) for n, w in prm.edges.get(idx1, []) if n != idx2]
    prm.edges[idx2] = [(n, w) for n, w in prm.edges.get(idx2, []) if n != idx1]
