"""
Path kept on the route of the visit: every node entered by the path search costs WEIGHT x its
distance from the route line (the polyline through the cell centres in visiting order), saturated
at SATURATION_M so that going round a big obstacle stays possible (roadmap.PRM._route_cost). Only
the path search and its stability check use it, not the cell rules nor the reachability searches.
The line is set when the mission is built (exploration.Mission.__init__).
"""
import numpy as np

from spot_explorer.mission import exploration
from spot_explorer.planning import roadmap
from hook import chain, replace

DESCRIPTION = "path search: nodes cost their distance from the route of the visit"
WEIGHT = 1.0
SATURATION_M = 1.0


def set_route_line(prm, points):
    """The route of the visit (list of (x, y)) for the path search of prm."""
    pts = np.asarray([p for p in points if p is not None], dtype=np.float64).reshape(-1, 2)
    prm.route_line = pts if len(pts) >= 2 else None
    prm._route_dist = {}


def route_distance(prm, idx):
    """Distance of node idx from the route line (inf without a line), cached: nodes never move."""
    cache = prm.__dict__.setdefault('_route_dist', {})
    d = cache.get(idx)
    if d is None:
        line = getattr(prm, 'route_line', None)
        if line is None or idx not in prm.nodes:
            return float('inf')
        a, b = line[:-1], line[1:]
        p = np.asarray(prm.nodes[idx], dtype=np.float64)
        ab = b - a
        t = np.clip(np.einsum('ij,ij->i', p - a, ab) / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-12), 0.0, 1.0)
        d = cache[idx] = float(np.min(np.hypot(*(a + t[:, None] * ab - p).T)))
    return d


def _route_cost(self, idx):
    if getattr(self, 'route_line', None) is None:
        return 0.0
    return WEIGHT * min(route_distance(self, idx), SATURATION_M)


def apply():
    init = exploration.Mission.__init__

    def __init__(self, *a, **k):
        init(self, *a, **k)
        set_route_line(self.prm, [self.env.get_world_position_from_cell(r, c) for r, c in self.path])
    return chain(replace(roadmap.PRM, '_route_cost', _route_cost),
                 replace(exploration.Mission, '__init__', __init__))
