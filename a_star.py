"""
A* instead of Dijkstra for the path search (roadmap.PRM._dijkstra).

The heuristic is alpha x the straight-line distance to the goal. It is never more than the real
cost (an edge costs alpha x its length plus slope terms >= 0), so the path costs the same as
Dijkstra's (equal-cost ties may differ) with fewer nodes explored.
"""
import numpy as np

from spot_explorer.planning import roadmap
from hook import replace

DESCRIPTION = "A* for the path search (same cost as Dijkstra, fewer nodes explored)"


def _path_heuristic(self, goal_idx):
    gx, gy = self.nodes[goal_idx]
    a = min(float(self.alpha), 1.0) if self.alpha > 0 else 0.0
    nodes = self.nodes
    return lambda v: a * float(np.hypot(nodes[v][0] - gx, nodes[v][1] - gy))


def apply():
    return replace(roadmap.PRM, '_path_heuristic', _path_heuristic)
