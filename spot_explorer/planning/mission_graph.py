"""
mission_graph.py -- the mission graph, built from the MISSION SIZE (rows, columns, cell
side: the same three numbers as SPOT_MISSION_GRID and cells.EnvironmentMap). Nodes are
generated in two ways (P.MISSION_GRAPH_SAMPLING, in config.py like all the P.* mentioned
here):
  - 'reticolo' (default): a rectangular LATTICE of equally spaced nodes, in closed form.
    Two flags: the SPACING between nodes and the connection RINGS.
  - 'casuale': scattered nodes with a chosen distribution, to compare the two methods
    (see RANDOM NODES).
Almost all of the comment below is about the lattice.

WHY A LATTICE
  1. Reproducibility. The graph depends only on the parameters: same parameters, same
     graph, bit for bit. The replay reproduces the planner's choices and two missions in the
     same room are comparable.
  2. Few edges. At 0.714 m spacing there are 1225 nodes on 625 m^2 and, with two rings,
     ~4700 edges: occupancy and slope, the real cost of the evaluation, are computed on these.
  3. Geometry out of the cost. With edges of a few fixed lengths alpha*dist +
     beta*slope says what it means to say; with randomly scattered points a longer detour
     can cost less than a straight one just because of where the points fell.
  4. The pairs are those of the lattice, ready-made: there is no geometric pairing either at
     start-up or at every update (see add_edges_to_prm).

WHY RECTANGULAR AND NOT TRIANGULAR
  The triangular one would be more isotropic and give shorter paths, but it CANNOT CONTAIN
  THE CELL CENTRES, which are the exploration goals and must not move. It would need
  cell_size to be a multiple of both s and s*sqrt(3)/2, and with sqrt(3) irrational that
  does not happen for any spacing. In the rectangular one with spacing cell_size/n the
  centres are nodes by construction. If the question were to be reopened, this is the point
  to attack, not isotropy.

SPACING
  The requested spacing is ALWAYS rounded to cell_size/n with n integer: it is the
  condition that makes every cell centre a lattice node. The [GRAFO] line of the log gives
  the actual spacing. Mission default: P.MISSION_LATTICE_SPACING_M = 0.5 m (= 4/8); without
  it, 0.25 m (DEFAULT_LATTICE_SPACING_M).
  The spacing can be shorter than min_edge_length: neighbours that are too close are simply
  not linked, and the edges start from the outer rings. The edge threshold is for the slope
  (an edge that is too short says nothing about the terrain); the dense spacing serves to
  ALWAYS have a row of nodes inside a corridor (see ALIASING).

RINGS -- how many connections
  Ring k is the square of nodes k index steps away (ring 1 = the 8 around, ring 2 = the 16
  of the next round, ...). `rings` = k links every node to all nodes within the diagonal
  of ring k, i.e. within k*s*sqrt(2), provided the edge is at least min_edge_length long.
  The limit is a circle and not the square for consistency with
  PRM.refresh_local_edge_weights, which recreates the pairs by distance around the robot.
  With spacing 0.25 and min_edge 0.5:
    rings=2 -> edges of 0.50 (straight), 0.56 (the "knight's move", 27 degrees) and 0.71 m
               (diagonal): 16 directions instead of 8, 16 neighbours per node;
    rings=3 -> additionally 0.75, 0.79, 0.90, 1.00, 1.03, 1.06 m: longer stretches, ~2.5
               times the edges.
  Long straight stretches are produced anyway by decisions.shortcut_index, which skips the
  intermediate nodes when the SAFE FRONTIER confirms the stretch on the live data.
  rings=None goes up to min(connection_radius, max_edge_length).

ANCHORING
  The lattice is anchored at the ORIGIN, which by the way cells.EnvironmentMap is written is
  both the robot's power-on pose AND the centre of the starting cell
  (get_world_position_from_cell: "Cell (0,0) center is at the origin"). So there is a node
  exactly under the robot at start-up, see origin_node_id. The indices (i, j) can be
  negative; the rectangle crops them.

ALIASING -- the limit to know
  If a corridor falls between two rows of nodes, it does not exist in the graph. The safe
  frontier requires geometry.FRONTIER_CLEARANCE_M = c from the centre line (0.25 m without
  air, 0.30 with 5 cm), so a usable corridor is at least 2c wide and the allowed centre lines
  are a strip (width - 2c) wide: for a row of nodes to always fall in it the spacing must
  be shorter than the strip. For random nodes coverage <= strip / 2 is enough.
  Guaranteed passage: lattice 2c + spacing, random nodes 2c + 2 * coverage. At spacing 0.25
  every strip wider than 0.25 m contains a row of nodes.

RANDOM NODES (P.MISSION_GRAPH_SAMPLING = 'casuale')
  As an alternative to the lattice, the nodes can be scattered at random over the mission
  area, to compare the two methods. The cell centres ALWAYS stay nodes (they are the goals,
  and the centre of the starting cell is the power-on point); the other nodes follow one of
  these distributions, with P.MISSION_RANDOM_DENSITY nodes per m^2:
    'uniforme'   independent uniform points: the classic PRM, with holes and clumps;
    'poisson'    Poisson-disk: uniform points discarded if closer than a minimum distance
                 to a node already placed (P.MISSION_POISSON_MIN_DIST_M; None = 0.75/sqrt(density),
                 which lets the requested density be reached): no clumps;
    'perturbata' the area divided into squares of 1/density m^2, one random point in each
                 (a few squares removed or points added at random to reach the exact
                 number): no area stays empty for more than one square;
    'halton'     Halton sequence in bases 2 and 3, quasi-random and very uniform; the
                 seed shifts it (Cranley-Patterson rotation), so different seeds give
                 different sets.
  The edges link every pair of nodes between min_edge_length and
  min(max_edge_length, connection_radius) apart; spacing and rings do not count.
  SEED: a fixed P.MISSION_RANDOM_SEED repeats the same graph bit for bit; None = a seed drawn
  at random. In both cases the seed used goes into the log ([GRAFO] Nodi casuali line) and into
  mission_summary.json, so the mission can be repeated and test_replay replays it with
  the same graph.
  The coverage (maximum distance between a point of the area and the nearest node) has no
  formula here: it is measured on a dense grid and goes into the log, because it is the
  number that tells whether a narrow corridor can be left without nodes (see ALIASING).

HOW TO USE IT
      mg = mission_graph.build_mission_graph(env=env)   # ~0.2 s: nodes + adjacency
      mg.into_prm(prm)                                  # the nodes, ids 0..N-1
      gb_sampler = mg.as_sampler()                      # lattice points per cell
      ...
      mg.add_edges_to_prm(prm, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M)
  add_edges_to_prm evaluates every pair with PRM._evaluate_and_add_edge, the same function
  used for all edges: vetoes and weights come from there. On the robot only the vetoes and
  weights are computed, which by construction cannot be precomputed: occupancy comes from
  the global map that fills up along the way, the edge slope from the terrain seen at that
  moment.
  save_npz/load_npz serve to have an IDENTICAL lattice across several missions, not for
  speed: with the default mission (3185 nodes) building from scratch costs ~180 ms,
  reloading ~70 ms (python3 -m spot_explorer.planning.mission_graph --timings).

The module depends ONLY on numpy: it can be imported and tested off the robot, without the
SDK. cells, roadmap, scipy (only for random nodes) and matplotlib are imported inside the
functions that use them.

From the command line:
    python3 -m spot_explorer.planning.mission_graph --compare
    python3 -m spot_explorer.planning.mission_graph --spacing 0.25 --rings 2 --preview p.png --zoom -2 2 -2 2
    python3 -m spot_explorer.planning.mission_graph --graph casuale --distribution poisson --density 16 --seed 7 --preview p.png
    python3 -m spot_explorer.planning.mission_graph --compare-methods methods.png --seed 7 --zoom -2 2 -2 2
    python3 -m spot_explorer.planning.mission_graph --rows 2 --cols 3 --cell 3
    python3 -m spot_explorer.planning.mission_graph --save lattice_2x4x5.npz
    python3 -m spot_explorer.planning.mission_graph --timings
"""

import math
from collections import deque

import numpy as np

from spot_explorer import config as P          # mission size, node method, edges
from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.perception import geometry


# The mission (rows, columns, side, spacing, node method, edges) is chosen in
# config.py, so the mission on the robot and the command-line tests (python3 -m
# spot_explorer.planning.mission_graph) start from the same numbers. These DEFAULT_* are only for compare() and
# as a fallback if a value in config is None (2 x 4 cells of 5 m, spacing 0.25 m).
DEFAULT_MISSION_ROWS = 2
DEFAULT_MISSION_COLS = 4
DEFAULT_CELL_SIZE_M = 5.0
DEFAULT_LATTICE_SPACING_M = 0.25


# ----------------------------------------------------------------------------------------
# HOW THE NODES ARE GENERATED (see RANDOM NODES in the comment at the top).
#   'reticolo' the rectangular lattice, deterministic (default);
#   'casuale'  scattered nodes with P.MISSION_RANDOM_DISTRIBUTION, cell centres always included.
# They are changed for a single mission with the arguments of run_mission.py (--graph,
# --distribution, --density, --seed).
# ----------------------------------------------------------------------------------------
GRAPH_SAMPLINGS = ('reticolo', 'casuale')
RANDOM_DISTRIBUTIONS = ('uniforme', 'poisson', 'perturbata', 'halton')


def mission_settings(rows=None, cols=None, cell_size_m=None, spacing_m=None):
    """
    (rows, cols, cell_size_m, spacing_m) of the mission. For each one, in order, the winner
    is: the value passed here, the one in config.py, the DEFAULT_*.
    """
    def pick(value, chosen, default):
        if value is not None:
            return value
        return chosen if chosen is not None else default
    return (int(pick(rows, P.MISSION_GRID_ROWS, DEFAULT_MISSION_ROWS)),
            int(pick(cols, P.MISSION_GRID_COLS, DEFAULT_MISSION_COLS)),
            float(pick(cell_size_m, P.MISSION_CELL_SIZE_M, DEFAULT_CELL_SIZE_M)),
            float(pick(spacing_m, P.MISSION_LATTICE_SPACING_M, DEFAULT_LATTICE_SPACING_M)))


# Default value of spacing_m in build_mission_graph: "take it from mission_settings()".
# It is not None because None already means suggested_spacing().
_FROM_SETTINGS = "settings"


def covering_radius(spacing):
    """
    Maximum distance between ANY point of the area and the nearest node: half the diagonal
    of the mesh. It is the number that matters for aliasing.
    """
    return spacing / math.sqrt(2.0)


def node_density(spacing):
    """Nodes per square metre."""
    return 1.0 / spacing ** 2


def align_spacing(spacing_m, cell_size_m):
    """
    The spacing rounded to cell_size/n with n integer: the condition that makes every cell
    centre a lattice node. Returns (spacing, n).
    """
    n = max(1, int(round(float(cell_size_m) / float(spacing_m))))
    return float(cell_size_m) / n, n


def suggested_spacing(cell_size_m=5.0, min_edge_length=P.PRM_MIN_EDGE_M):
    """
    The largest aligned spacing whose coverage does not exceed min_edge_length. With 5 m cells
    and min_edge 0.5 m it gives cell_size/7 = 0.714 m (coverage 0.505). If min_edge_length
    changes in the PRM, the suggested spacing is recomputed by itself.
    """
    n = max(1, int(math.ceil(float(cell_size_m) / (float(min_edge_length) * math.sqrt(2.0)))))
    return float(cell_size_m) / n


def _mission_extent(rows, cols, cell_size_m, start_cell):
    """
    Mission rectangle in the GRID FRAME (x = columns, y = rows), with the centre of the
    starting cell at the origin -- the same convention as
    EnvironmentMap.get_world_position_from_cell. The borders are half a cell from the
    outermost centres, like the boundaries used by get_cell_from_world (round on the cells).
    """
    start_row, start_col = start_cell
    half = cell_size_m / 2.0
    return ((0 - start_col) * cell_size_m - half,
            (cols - 1 - start_col) * cell_size_m + half,
            (0 - start_row) * cell_size_m - half,
            (rows - 1 - start_row) * cell_size_m + half)


def _lattice_points(spacing, extent, border_margin_m):
    """
    The lattice nodes inside the rectangle, anchored at the origin. Returns
    (ij, xy_local): integer indices and coordinates in the grid frame. Deterministic
    order: increasing row, then column.
    """
    x_min, x_max, y_min, y_max = extent
    x_min, x_max = x_min + border_margin_m, x_max - border_margin_m
    y_min, y_max = y_min + border_margin_m, y_max - border_margin_m
    eps = 1e-9
    i_lo, i_hi = (int(math.ceil(y_min / spacing - eps)), int(math.floor(y_max / spacing + eps)))
    j_lo, j_hi = (int(math.ceil(x_min / spacing - eps)), int(math.floor(x_max / spacing + eps)))
    ij = [(i, j) for i in range(i_lo, i_hi + 1) for j in range(j_lo, j_hi + 1)]
    xy = [(j * spacing, i * spacing) for i, j in ij]
    return (np.array(ij, dtype=np.int64).reshape(-1, 2),
            np.array(xy, dtype=np.float64).reshape(-1, 2))


def _lattice_pairs(spacing, ij, xy, min_edge_m, reach_m):
    """
    The node pairs that will become edges, with their length. Exploits the fact that the
    nodes lie on integer indices: the neighbours are found by sliding a window of offsets
    (O(N*k) integer arithmetic) instead of querying a k-d tree on all pairs.

    Returns (pairs, lengths, n_below_threshold): the allowed pairs, the lengths, and how many
    pairs were discarded because shorter than min_edge_m. With an aligned spacing
    >= min_edge_m that number is ZERO: if it is not, the first ring is falling below the
    PRM threshold.
    """
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(ij)}
    w = int(math.ceil(reach_m / spacing))
    pairs, lengths, n_short = [], [], 0
    reach2, min2 = reach_m ** 2 + 1e-12, min_edge_m ** 2 - 1e-12
    for k, (i, j) in enumerate(ij):
        x0, y0 = xy[k]
        for di in range(-w, w + 1):
            for dj in range(-w, w + 1):
                if di == 0 and dj == 0:
                    continue
                other = index.get((int(i) + di, int(j) + dj))
                if other is None or other <= k:          # every pair only once
                    continue
                dx, dy = xy[other, 0] - x0, xy[other, 1] - y0
                d2 = dx * dx + dy * dy
                if d2 > reach2:
                    continue
                if d2 < min2:
                    n_short += 1
                    continue
                pairs.append((k, other))
                lengths.append(math.sqrt(d2))
    if not pairs:
        return np.zeros((0, 2), np.int64), np.zeros(0), n_short
    pairs = np.array(pairs, dtype=np.int64)
    lengths = np.array(lengths, dtype=np.float64)
    order = np.lexsort((pairs[:, 1], pairs[:, 0]))
    return pairs[order], lengths[order], n_short


# ----------------------------------------------------------------------------------------
# RANDOM NODES
# ----------------------------------------------------------------------------------------

def draw_seed():
    """A new seed, when none was requested: 31 bits from the system."""
    import random
    return random.SystemRandom().randrange(2 ** 31)


def poisson_min_dist(density_per_m2):
    """
    Minimum Poisson-disk distance to reach the requested density. Scattering points at
    random with minimum distance r stops at about 0.70/r^2 points per m^2 (saturation);
    with r = 0.75/sqrt(density) saturation is at ~1.24 times the requested density, which
    stays reachable.
    """
    return 0.75 / math.sqrt(float(density_per_m2))


def _halton(n, base, start=1):
    """The first n terms of the van der Corput sequence in `base`, from index start."""
    out = np.zeros(n, dtype=np.float64)
    for k in range(n):
        i, f, x = start + k, 1.0, 0.0
        while i > 0:
            f /= base
            x += f * (i % base)
            i //= base
        out[k] = x
    return out


def _poisson_points(n_target, box, fixed_xy, min_dist, rng, max_tries_per_point=50):
    """
    Poisson-disk by dart throwing: uniform points in the rectangle, discarded if closer than
    min_dist to a point already placed (cell centres included). Stops at n_target points or
    when the failed attempts in a row become too many (area saturated). A helper grid with
    side min_dist/sqrt(2) holds at most one point per box, so every check only looks at the
    5 x 5 boxes around.
    """
    x0, x1, y0, y1 = box
    h = min_dist / math.sqrt(2.0)
    nx, ny = max(1, int(math.ceil((x1 - x0) / h))), max(1, int(math.ceil((y1 - y0) / h)))
    grid = -np.ones((ny, nx), dtype=np.int64)
    pts = [tuple(p) for p in fixed_xy]
    r2 = min_dist * min_dist

    def cell_of(x, y):
        return (min(ny - 1, max(0, int((y - y0) / h))), min(nx - 1, max(0, int((x - x0) / h))))

    for k, (x, y) in enumerate(pts):
        grid[cell_of(x, y)] = k
    added, fails, limit = 0, 0, max_tries_per_point * max(1, n_target)
    while added < n_target and fails < limit:
        batch = rng.random((256, 2))
        for u, v in batch:
            x, y = x0 + u * (x1 - x0), y0 + v * (y1 - y0)
            gi, gj = cell_of(x, y)
            ok = True
            for a in range(max(0, gi - 2), min(ny, gi + 3)):
                for b in range(max(0, gj - 2), min(nx, gj + 3)):
                    k = grid[a, b]
                    if k >= 0:
                        dx, dy = pts[k][0] - x, pts[k][1] - y
                        if dx * dx + dy * dy < r2:
                            ok = False
                            break
                if not ok:
                    break
            if not ok:
                fails += 1
                if fails >= limit:
                    break
                continue
            grid[gi, gj] = len(pts)
            pts.append((x, y))
            added += 1
            if added >= n_target:
                break
    return np.array(pts[len(fixed_xy):], dtype=np.float64).reshape(-1, 2)


def _random_points(distribution, density_per_m2, box, fixed_xy, rng, min_dist=None):
    """
    The random nodes in the rectangle box = (x_min, x_max, y_min, y_max), without the fixed
    nodes (cell centres), which poisson respects though. Returns (xy, note): note is a
    string for the log when the requested density was not reached.
    """
    x0, x1, y0, y1 = box
    w, h = x1 - x0, y1 - y0
    n = int(round(float(density_per_m2) * w * h))
    if distribution == 'uniforme':
        u = rng.random((n, 2))
        return np.column_stack((x0 + u[:, 0] * w, y0 + u[:, 1] * h)), ""
    if distribution == 'perturbata':
        side = 1.0 / math.sqrt(float(density_per_m2))
        nx, ny = max(1, int(round(w / side))), max(1, int(round(h / side)))
        hx, hy = w / nx, h / ny                   # squares covering the exact rectangle
        jj, ii = np.meshgrid(np.arange(nx), np.arange(ny))
        u = rng.random((ny * nx, 2))
        xy = np.column_stack((x0 + (jj.ravel() + u[:, 0]) * hx,
                              y0 + (ii.ravel() + u[:, 1]) * hy))
        # The squares are whole and do not always give exactly n points: the extra ones are
        # removed at random, or uniform points are added, so the total is the requested one.
        if len(xy) > n:
            xy = xy[np.sort(rng.choice(len(xy), n, replace=False))]
        elif len(xy) < n:
            e = rng.random((n - len(xy), 2))
            xy = np.vstack((xy, np.column_stack((x0 + e[:, 0] * w, y0 + e[:, 1] * h))))
        return xy, ""
    if distribution == 'halton':
        shift = rng.random(2)
        u = (np.column_stack((_halton(n, 2), _halton(n, 3))) + shift) % 1.0
        return np.column_stack((x0 + u[:, 0] * w, y0 + u[:, 1] * h)), ""
    if distribution == 'poisson':
        r = float(min_dist) if min_dist else poisson_min_dist(density_per_m2)
        xy = _poisson_points(n, box, fixed_xy, r, rng)
        note = ("" if len(xy) >= n else
                f"area satura a {len(xy)} nodi su {n} chiesti con distanza minima {r:.3f} m")
        return xy, note
    raise ValueError(f"distribuzione sconosciuta {distribution!r}: una di {RANDOM_DISTRIBUTIONS}")


def _radius_pairs(xy, min_edge_m, reach_m):
    """
    The node pairs between min_edge_m and reach_m apart, with the length, sorted like those
    of the lattice. Returns (pairs, lengths, n_below_threshold).
    """
    from scipy.spatial import cKDTree
    found = cKDTree(xy).query_pairs(r=float(reach_m), output_type='ndarray')
    if len(found) == 0:
        return np.zeros((0, 2), np.int64), np.zeros(0), 0
    found = np.sort(found.astype(np.int64), axis=1)
    d = xy[found[:, 1]] - xy[found[:, 0]]
    lengths = np.sqrt(d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1])
    keep = lengths >= float(min_edge_m) - 1e-12
    pairs, lengths = found[keep], lengths[keep]
    order = np.lexsort((pairs[:, 1], pairs[:, 0]))
    return pairs[order], lengths[order], int((~keep).sum())


def measured_covering(xy, box, step=0.05):
    """
    Measured coverage: (maximum, 95th percentile) of the distance between the points of a
    dense grid in the rectangle and the nearest node. For the lattice there is the formula
    (covering_radius); for random nodes it must be measured.
    """
    from scipy.spatial import cKDTree
    x0, x1, y0, y1 = box
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step))
    d, _ = cKDTree(xy).query(np.column_stack((gx.ravel(), gy.ravel())))
    return float(d.max()), float(np.percentile(d, 95))


# Slopes of the pairs computed in batch with matrices (see PRM.precompute_arc_slopes).
# False = edge by edge; the result is identical.
BATCH_SLOPES = True


def _special_pairs(prm):
    """Edges travelled by the robot and edges blocked in the attempt, as (min, max) pairs."""
    out = {(min(a, b), max(a, b)) for a, b in getattr(prm, 'traversed_edges', ())}
    out |= {(min(a, b), max(a, b)) for a, b in getattr(prm, 'tracker_blocked_edges', ())}
    return out


def _prm_dirty_margin():
    """roadmap.DIRTY_MARGIN_M, imported only when needed (this module also runs without the SDK)."""
    try:
        from spot_explorer.planning import roadmap as prm_graph
        return prm_graph.DIRTY_MARGIN_M
    except Exception:
        return 1.2


def _frontier_clearance():
    """geometry.FRONTIER_CLEARANCE_M, imported only when needed (this module also runs without the SDK)."""
    try:
        from spot_explorer.perception import geometry
        return geometry.FRONTIER_CLEARANCE_M
    except Exception:
        return 0.25


def _connected_components(n_nodes, pairs):
    """(components, size of the largest, isolated nodes). Geometry only: tells whether the
    DELIVERED graph is connected, before the terrain vetoes thin it out."""
    adj = [[] for _ in range(n_nodes)]
    for a, b in pairs:
        adj[int(a)].append(int(b))
        adj[int(b)].append(int(a))
    seen = np.zeros(n_nodes, dtype=bool)
    sizes = []
    for s in range(n_nodes):
        if seen[s]:
            continue
        seen[s] = True
        q, size = deque([s]), 0
        while q:
            u = q.popleft()
            size += 1
            for v in adj[u]:
                if not seen[v]:
                    seen[v] = True
                    q.append(v)
        sizes.append(size)
    return len(sizes), (max(sizes) if sizes else 0), int(sum(1 for a in adj if not a))


# ========================================================================================
# THE MISSION GRAPH
# ========================================================================================

class MissionGraph:
    """
    The ready graph: nodes, geometric adjacency with the lengths, and the hooks to load it
    into the PRM. It contains NO weights or vetoes: those depend on the terrain and the PRM
    computes them on the local data (see add_edges_to_prm).
    """

    def __init__(self, spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                 ij, xy_local, pairs, lengths, node_cell, cell_center_ids, origin_node_id,
                 min_edge_m, reach_m, rings, n_short_pairs, sampling='reticolo',
                 distribution=None, density_per_m2=None, seed=None, covering=None,
                 sampling_note=""):
        self.spacing_m = float(spacing_m)
        self.rows, self.cols = int(rows), int(cols)
        self.cell_size_m = float(cell_size_m)
        self.start_cell = tuple(int(v) for v in start_cell)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.origin_yaw = float(origin_yaw)
        self.min_edge_m, self.reach_m = float(min_edge_m), float(reach_m)
        self.rings = rings

        self.ij = ij                    # (N, 2) integer lattice indices
        self.xy_local = xy_local        # (N, 2) grid frame
        self.pairs = pairs              # (E, 2) node indices, sorted
        self.lengths = lengths          # (E,)   metres
        self.node_cell = node_cell      # (N, 2) cell (row, column) of every node
        self.cell_center_ids = cell_center_ids    # {(row, col): id of the node at the centre}
        self.origin_node_id = origin_node_id      # node under the robot at start-up
        self.n_short_pairs = int(n_short_pairs)
        # How the nodes were generated (see RANDOM NODES). For random nodes ij is
        # -1 and spacing_m is the mean spacing 1/sqrt(density); the coverage is measured.
        self.sampling = str(sampling)
        self.distribution = distribution
        self.density_per_m2 = None if density_per_m2 is None else float(density_per_m2)
        self.seed = None if seed is None else int(seed)
        self.covering = covering                  # (maximum, 95th percentile) or None
        self.sampling_note = sampling_note

        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        self.xy = np.column_stack((
            self.origin_xy[0] + xy_local[:, 0] * c - xy_local[:, 1] * s,
            self.origin_xy[1] + xy_local[:, 0] * s + xy_local[:, 1] * c,
        ))

        deg = np.zeros(len(xy_local), dtype=np.int64)
        for a, b in self.pairs:
            deg[a] += 1
            deg[b] += 1
        self.degrees = deg
        (self.n_components, self.biggest_component,
         self.n_isolated) = _connected_components(len(xy_local), self.pairs)

    # -- properties --------------------------------------------------------------------
    @property
    def n_nodes(self):
        return int(len(self.xy))

    @property
    def n_edges(self):
        return int(len(self.pairs))

    @property
    def area_m2(self):
        return self.rows * self.cols * self.cell_size_m ** 2

    @property
    def is_connected(self):
        return self.n_components == 1

    @property
    def is_random(self):
        return self.sampling == 'casuale'

    @property
    def covering_m(self):
        if self.covering is not None:
            return float(self.covering[0])
        return covering_radius(self.spacing_m)

    def describe(self):
        """The node method in a dict, for mission_summary.json and to replay the mission."""
        out = dict(metodo=self.sampling, nodi=self.n_nodes, coppie=self.n_edges,
                   arco_min_m=self.min_edge_m, arco_max_m=round(self.reach_m, 6),
                   copertura_m=round(self.covering_m, 4))
        if self.is_random:
            out.update(distribuzione=self.distribution, densita_nodi_m2=self.density_per_m2,
                       seme=self.seed)
        else:
            out.update(passo_m=self.spacing_m, anelli=self.rings)
        return out

    def node_ids(self):
        """The node ids, 0..N-1: the same PRM.nodes will use."""
        return np.arange(self.n_nodes, dtype=np.int64)

    # -- hooking into the PRM ---------------------------------------------------------------
    def into_prm(self, prm):
        """
        Put the NODES into the PRM (prm.add_node), with ids 0..N-1. No edges: the edges
        come from add_edges_to_prm, which needs the terrain data.
        """
        for k in range(self.n_nodes):
            prm.add_node(int(k), float(self.xy[k, 0]), float(self.xy[k, 1]))
        return self.n_nodes

    def add_edges_to_prm(self, prm, global_map=None, edge_safety_margin=0.0, verbose=True,
                         incremental=False):
        """
        Evaluate the pairs ALREADY COMPUTED and add the allowed ones with
        PRM._evaluate_and_add_edge: vetoes (blocked edges, occupancy in the global map,
        longitudinal and lateral slope) and weights. The pairs are those of the lattice, no
        geometric pairing is redone.

        incremental: re-evaluate ONLY the pairs near the areas with new data since the last
        evaluation (prm.dirty_rects, see roadmap.DIRTY_MARGIN_M); the others keep the edge
        and weight they have. The result is the same as the full build: outside those areas
        no data entering the evaluation has changed. It falls back to the full build when it
        cannot be known what changed (see _dirty_pairs).
        """
        todo_all = [(int(a), int(b), float(d)) for (a, b), d in zip(self.pairs, self.lengths)
                    if int(a) in prm.nodes and int(b) in prm.nodes]
        dirty = self._dirty_pairs(prm, global_map, todo_all) if incremental else None
        prm.robot_node = None          # the exemption near the robot applies only to the current node
        if dirty is None:
            for idx in prm.nodes:
                prm.edges[idx] = []
            prm.edge_validity = {}
            todo = todo_all
        else:
            todo = [p for p, m in zip(todo_all, dirty) if m]
            # The edges to re-evaluate are removed; then they are evaluated as in a build from
            # scratch (adjacencies just cleared for these pairs: fresh applies here too).
            drop = {}
            for a, b, _d in todo:
                drop.setdefault(a, set()).add(b)
                drop.setdefault(b, set()).add(a)
                prm.edge_validity.pop((a, b) if a < b else (b, a), None)
            for n, others in drop.items():
                prm.edges[n] = [(m, w) for m, w in prm.edges.get(n, []) if m not in others]
        if hasattr(prm, '_sync_global_slope_cache'):
            prm._sync_global_slope_cache()

        added = 0
        # Occupancy of all pairs in one go and global slope cache
        # filled where there is certainly no data (identical outcomes, see
        # PRM._occupancy_blocked_many and PRM._prefill_global_slope_nodata). The adjacencies have
        # just been cleared and the lattice pairs are distinct: fresh=True.
        batch = hasattr(prm, '_occupancy_blocked_many')
        use_occ = global_map is not None and edge_safety_margin > 0.0
        occ_flags = (prm._occupancy_blocked_many(todo, global_map, edge_safety_margin)
                     if (batch and use_occ) else [None] * len(todo))
        if batch:
            prm._prefill_global_slope_nodata(todo)
        fresh = batch and self._pairs_distinct()
        # AREA WITHOUT DATA: edges on which no source has information get
        # the fallback cost directly, identical to that of the full path (see
        # PRM._add_edge_without_data). Needs gamma = 0: with roughness in the cost, no.
        fast_ok = (fresh and hasattr(prm, '_add_edge_without_data') and prm.gamma == 0.0)
        live_nd = prm._live_slope_certainly_nodata(todo) if fast_ok else None
        tm = getattr(prm, 'global_terrain_map', None)
        cache = getattr(prm, '_gslope_cache', {})
        blocked_set = prm.tracker_blocked_edges
        n_fast = 0
        # Slopes of all the pairs to evaluate in one go (matrices), live and,
        # where there is no live data, from the global map. Same numbers as the edge-by-edge
        # computation, about 13 times faster (see PRM.precompute_arc_slopes).
        live_profs = [None] * len(todo)
        if BATCH_SLOPES and batch and hasattr(prm, 'precompute_arc_slopes'):
            skip = np.array([bool(o) for o in occ_flags], dtype=bool)
            live_profs = prm.precompute_arc_slopes(todo, skip=skip)
        for i, ((a, b, d), occ) in enumerate(zip(todo, occ_flags)):
            if fast_ok and live_nd[i] and not occ:
                key = (a, b) if a < b else (b, a)
                g = cache.get(key) if tm is not None else (0.0, 0.0, False)
                if key not in blocked_set and g is not None and g[2] is False:
                    prm._add_edge_without_data(a, b, d)
                    added += 1
                    n_fast += 1
                    continue
            if batch:
                ok = prm._evaluate_and_add_edge(a, b, d, global_map, edge_safety_margin,
                                                occ_blocked=occ, fresh=fresh, live_prof=live_profs[i])
            else:
                ok = prm._evaluate_and_add_edge(a, b, d, global_map, edge_safety_margin)
            if ok:
                added += 1

        self.last_update = (len(todo), len(todo_all), dirty is not None)

        # The edges travelled by the robot are free by construction and are put back anyway:
        # the stop nodes are often below min_edge_length.
        for a, b in list(getattr(prm, 'traversed_edges', ())):
            if a in prm.nodes and b in prm.nodes:
                if hasattr(prm, '_traversed_edge_ok') and not prm._traversed_edge_ok(a, b, global_map):
                    prm.traversed_edges.discard((a, b))
                    continue
                (xa, ya), (xb, yb) = prm.nodes[a], prm.nodes[b]
                prm._evaluate_and_add_edge(a, b, float(math.hypot(xb - xa, yb - ya)), force=True)
        # The areas just re-evaluated are settled. The current live window stays
        # "to redo": its edges were judged on the live data, and next time,
        # with the robot elsewhere, the full build would judge them on the global map.
        if hasattr(prm, 'dirty_rects'):
            prm.dirty_rects = list(prm.live_rects())
            prm.hazards_seen = len(getattr(global_map, 'hazards', ()) or ())
            prm.special_seen = _special_pairs(prm)
        prm.built = True
        if hasattr(prm, 'flush_slope_log'):
            prm.flush_slope_log("costruzione")
        if verbose and dirty is not None:
            print(f"[GRAFO] Aggiornamento locale: rivalutate {len(todo)} coppie su {len(todo_all)} "
                  f"(vicine alle zone con dati nuovi), {added} ammesse; le altre restano come erano.")
        elif verbose:
            where = (f"nodi casuali {self.distribution}" if self.is_random
                     else f"passo {self.spacing_m:.4f} m")
            print(f"[GRAFO] {added} archi ammessi su {self.n_edges} coppie del "
                  f"{'grafo' if self.is_random else 'reticolo'} ({self.n_nodes} nodi, {where}); "
                  f"{n_fast} in zona senza dati, con il solo costo geometrico")
        return added

    def _dirty_pairs(self, prm, global_map, todo):
        """
        Mask of the pairs to re-evaluate (edge rectangle, widened by
        roadmap.DIRTY_MARGIN_M, touching an area of prm.dirty_rects), or None when the full
        build is needed: graph never built, terrain map reloaded from file (updated areas
        unknown), a new fall (mark_hazard does not go through a scan).

        The edges travelled by the robot and those blocked in the attempt are ALWAYS
        re-evaluated, together with those that were so at the previous update: their outcome
        depends on those sets, not on the new data. They are few.
        """
        if not getattr(prm, 'built', False) or not hasattr(prm, 'dirty_rects'):
            return None
        tm = getattr(prm, 'global_terrain_map', None)
        if tm is not None and getattr(tm, 'update_rects', None) is None:
            return None
        if len(getattr(global_map, 'hazards', ()) or ()) != getattr(prm, 'hazards_seen', 0):
            return None
        mask = np.zeros(len(todo), dtype=bool)
        if not todo:
            return mask
        ia = np.array([p[0] for p in todo]); ib = np.array([p[1] for p in todo])
        a = np.array([prm.nodes[i] for i in ia]); b = np.array([prm.nodes[i] for i in ib])
        m = _prm_dirty_margin()
        xmin, xmax = np.minimum(a[:, 0], b[:, 0]) - m, np.maximum(a[:, 0], b[:, 0]) + m
        ymin, ymax = np.minimum(a[:, 1], b[:, 1]) - m, np.maximum(a[:, 1], b[:, 1]) + m
        for (rx0, rx1, ry0, ry1) in prm.dirty_rects:
            mask |= (xmax >= rx0) & (xmin <= rx1) & (ymax >= ry0) & (ymin <= ry1)
        special = _special_pairs(prm) | getattr(prm, 'special_seen', set())
        if special:
            mask |= np.array([(min(a, b), max(a, b)) in special for a, b in zip(ia.tolist(), ib.tolist())])
        return mask

    def _pairs_distinct(self):
        """True if no lattice pair appears twice (in either direction). Computed once."""
        cached = getattr(self, '_distinct_cache', None)
        if cached is None:
            p = np.sort(np.asarray(self.pairs, dtype=np.int64).reshape(-1, 2), axis=1)
            cached = bool(p.shape[0] == 0 or np.unique(p, axis=0).shape[0] == p.shape[0])
            self._distinct_cache = cached
        return cached

    def as_sampler(self):
        """The lattice points cell by cell (get_point_in_cell), for find_best_point_in_cell."""
        return LatticeSampler(self)

    # -- persistence and diagnostics -----------------------------------------------------
    def save_npz(self, path):
        """
        Save the lattice: indices, LOCAL coordinates, pairs, lengths, parameters. The local
        coordinates do not depend on the robot pose, so the file is valid for any mission of
        that size: it is reloaded with load_npz and only the roto-translation is applied.
        Serves to have an identical lattice across several missions; it is not needed for
        speed (building costs ~30 ms, reloading ~20).
        """
        keys = np.array(list(self.cell_center_ids.keys()), dtype=np.int64).reshape(-1, 2)
        vals = np.array(list(self.cell_center_ids.values()), dtype=np.int64)
        np.savez_compressed(
            path, ij=self.ij, xy_local=self.xy_local, pairs=self.pairs, lengths=self.lengths,
            node_cell=self.node_cell, spacing_m=self.spacing_m, rows=self.rows, cols=self.cols,
            cell_size_m=self.cell_size_m, start_cell=np.array(self.start_cell),
            min_edge_m=self.min_edge_m, reach_m=self.reach_m,
            rings=(-1 if self.rings is None else self.rings),
            origin_node_id=(-1 if self.origin_node_id is None else self.origin_node_id),
            cell_center_keys=keys, cell_center_vals=vals, n_short_pairs=self.n_short_pairs,
            sampling=self.sampling, distribution=str(self.distribution or ''),
            density_per_m2=(-1.0 if self.density_per_m2 is None else self.density_per_m2),
            seed=(-1 if self.seed is None else self.seed),
            covering=(np.array([-1.0, -1.0]) if self.covering is None
                      else np.array(self.covering, dtype=np.float64)),
            sampling_note=self.sampling_note)
        return path

    def report(self):
        """The summary to print in the log, next to the [CONFIG] lines."""
        lo = float(self.lengths.min()) if self.n_edges else float('nan')
        hi = float(self.lengths.max()) if self.n_edges else float('nan')
        uniq = sorted(set(np.round(self.lengths, 3).tolist()))
        rings = ", ".join(f"{v:.2f}" for v in uniq[:6]) + (" ..." if len(uniq) > 6 else "")
        if self.is_random:
            head = [
                f"[GRAFO] Missione {self.rows}x{self.cols} celle da {self.cell_size_m:.1f} m "
                f"= {self.area_m2:.0f} m2; nodi CASUALI, archi fra {self.min_edge_m:.2f} e "
                f"{self.reach_m:.2f} m",
                # test_replay reads distribution, density and seed from this line.
                f"[GRAFO] Nodi casuali: distribuzione {self.distribution}, densita' "
                f"{self.density_per_m2:.2f} nodi/m2, seme {self.seed} "
                f"(per ripetere: --graph casuale --distribution {self.distribution} "
                f"--density {self.density_per_m2:g} --seed {self.seed})",
            ]
            if self.sampling_note:
                head.append(f"[GRAFO] ATTENZIONE: {self.sampling_note}")
            density = self.n_nodes / self.area_m2
            cover = (f"[GRAFO] Copertura misurata {self.covering[0]:.2f} m (95% dell'area entro "
                     f"{self.covering[1]:.2f} m; un corridoio utile al fronte e' largo almeno "
                     f"{2 * _frontier_clearance():.2f} m)")
        else:
            n_div = int(round(self.cell_size_m / self.spacing_m))
            head = [
                f"[GRAFO] Missione {self.rows}x{self.cols} celle da {self.cell_size_m:.1f} m "
                f"= {self.area_m2:.0f} m2; reticolo rettangolare, passo {self.spacing_m:.4f} m "
                f"(= {self.cell_size_m:.1f}/{n_div}), anelli "
                f"{'tutti entro %.2f m' % self.reach_m if self.rings is None else self.rings}",
            ]
            density = node_density(self.spacing_m)
            cover = (f"[GRAFO] Copertura {self.covering_m:.2f} m (un corridoio utile al fronte e' "
                     f"largo almeno {2 * _frontier_clearance():.2f} m)")
        out = head + [
            f"[GRAFO] {self.n_nodes} nodi, {density:.2f} nodi/m2, "
            f"{self.n_edges} archi, grado medio {self.degrees.mean():.1f} "
            f"(min {int(self.degrees.min()) if self.n_nodes else 0}, "
            f"max {int(self.degrees.max()) if self.n_nodes else 0})",
            f"[GRAFO] Lunghezze degli archi {lo:.2f}-{hi:.2f} m"
            f"{'' if self.is_random else ' [%s]' % rings}, "
            f"min_edge_length={self.min_edge_m:.2f} m, "
            f"{self.n_short_pairs} coppie di vicini troppo vicini, non collegate",
            cover,
            f"[GRAFO] Obiettivi di cella: {len(self.cell_center_ids)} su "
            f"{self.rows * self.cols}, tutti nel centro geometrico esatto",
            f"[GRAFO] Connessione geometrica: "
            + ("UNICA componente, nessun nodo isolato"
               if self.is_connected and self.n_isolated == 0 else
               f"{self.n_components} componenti, la piu' grande con {self.biggest_component} "
               f"nodi su {self.n_nodes}, {self.n_isolated} isolati"),
        ]
        if self.origin_node_id is not None:
            out.append(f"[GRAFO] Accensione = centro della cella di partenza = nodo "
                       f"{self.origin_node_id} (nessun nodo di boot da aggiungere)")
        if not self.is_connected or self.n_isolated:
            if self.is_random:
                out.append(f"[GRAFO] ATTENZIONE: il grafo casuale non e' connesso con questi "
                           f"parametri (densita' {self.density_per_m2:g} nodi/m2, archi "
                           f"{self.min_edge_m:.2f}-{self.reach_m:.2f} m): aumenta la densita' "
                           f"o la lunghezza massima degli archi.")
            else:
                out.append(f"[GRAFO] ATTENZIONE: il reticolo non e' connesso con questi parametri "
                           f"(passo {self.spacing_m:.4f} m, archi {self.min_edge_m:.2f}-"
                           f"{self.reach_m:.2f} m): aumenta gli anelli.")
        return "\n".join(out)

    def preview_png(self, path, show_cells=True, zoom=None):
        """
        Draw the graph and its edges into a PNG, to look at it before going out.
        zoom: (x_min, x_max, y_min, y_max) in the grid frame, for the details.
        """
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(9.5, 9.5))
        self.draw(ax, show_cells=show_cells, zoom=zoom)
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
        return path

    def draw(self, ax, show_cells=True, zoom=None, legend=True, title_size=None,
             edge_color='0.78', edge_width=0.45, highlight=None):
        """
        Draw nodes, edges, cell goals and cells on a matplotlib axes.
        highlight: id of a node whose edges to highlight (its neighbours).
        """
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        segs = [[(self.xy_local[a, 0], self.xy_local[a, 1]),
                 (self.xy_local[b, 0], self.xy_local[b, 1])] for a, b in self.pairs]
        ax.add_collection(LineCollection(segs, colors=edge_color, linewidths=edge_width, zorder=1))
        if highlight is not None:
            mine = [s for s, (a, b) in zip(segs, self.pairs) if highlight in (a, b)]
            ax.add_collection(LineCollection(mine, colors='tab:blue', linewidths=0.8, alpha=0.7,
                                             zorder=2, label=f"archi del nodo {highlight}"))
        ax.plot(self.xy_local[:, 0], self.xy_local[:, 1], 'k.', ms=2.6, zorder=3)
        ids = sorted(set(self.cell_center_ids.values()))
        if ids:
            ax.plot(self.xy_local[ids, 0], self.xy_local[ids, 1], 'o', ms=7, mfc='none',
                    mec='tab:orange', mew=1.4, zorder=4, label='obiettivo di cella')
        if self.origin_node_id is not None:
            ax.plot(self.xy_local[self.origin_node_id, 0], self.xy_local[self.origin_node_id, 1],
                    'bo', ms=9, zorder=5, label='accensione / centro cella di partenza')
        if show_cells:
            half = self.cell_size_m / 2.0
            for r in range(self.rows):
                for c in range(self.cols):
                    cx = (c - self.start_cell[1]) * self.cell_size_m
                    cy = (r - self.start_cell[0]) * self.cell_size_m
                    ax.add_patch(plt.Rectangle((cx - half, cy - half), self.cell_size_m,
                                               self.cell_size_m, fill=False, ec='tab:green',
                                               ls='--', lw=1.0, alpha=0.65, zorder=2))
                    ax.text(cx, cy - half + 0.35, f"{r},{c}", ha='center', va='bottom',
                            fontsize=7, color='tab:green', zorder=6, clip_on=True)
        if zoom:
            ax.set_xlim(zoom[0], zoom[1])
            ax.set_ylim(zoom[2], zoom[3])
        else:
            ax.autoscale_view()
        ax.set_aspect('equal', 'box')
        ax.set_xlabel('x griglia [m] -- colonne, davanti al robot')
        ax.set_ylabel('y griglia [m] -- righe, a sinistra del robot')
        what = (f"casuale {self.distribution}, {self.density_per_m2:.3g} nodi/m2, seme {self.seed}"
                if self.is_random else
                f"passo {self.spacing_m:.4f} m, anelli "
                f"{self.rings if self.rings is not None else 'tutti'}")
        ax.set_title(f"{what}\n{self.n_nodes} nodi, {self.n_edges} archi, "
                     f"copertura {self.covering_m:.2f} m", fontsize=title_size)
        if legend:
            ax.legend(loc='upper right', fontsize=8)


class LatticeSampler:
    """The lattice points cell by cell (see MissionGraph.as_sampler)."""

    def __init__(self, mg):
        self._points = [(float(x), float(y)) for x, y in mg.xy]
        self._by_cell = {}
        for k, (r, c) in enumerate(mg.node_cell):
            self._by_cell.setdefault((int(r), int(c)), []).append(k)

    def get_point_in_cell(self, row, col):
        return [self._points[k] for k in self._by_cell.get((int(row), int(col)), ())]

# ========================================================================================
# THE FUNCTION TO CALL
# ========================================================================================

def build_mission_graph(rows=None, cols=None, cell_size_m=None, spacing_m=_FROM_SETTINGS,
                        rings=P.MISSION_LATTICE_RINGS,
                        origin_xy=(0.0, 0.0), origin_yaw=0.0, start_cell=(0, 0),
                        min_edge_length=P.PRM_MIN_EDGE_M,
                        max_edge_length=P.PRM_MAX_EDGE_M,
                        connection_radius=P.PRM_CONNECTION_RADIUS_M,
                        border_margin_m=0.0, env=None, verbose=True,
                        sampling=None, distribution=None, density_per_m2=None, seed=None,
                        poisson_min_dist_m=None):
    """
    Build the mission graph.

    Args:
        rows, cols, cell_size_m: the MISSION SIZE. If not passed: those of
            mission_settings() (MISSION_*, or the DEFAULT_*). Ignored if `env` is passed.
        spacing_m: SPACING FLAG, in metres; it is rounded to cell_size/n (see
            align_spacing). It can be below min_edge_length (see SPACING). If not passed:
            the one of mission_settings() (P.MISSION_LATTICE_SPACING_M, or 0.25 m); None =
            suggested_spacing().
        rings: CONNECTIONS FLAG -- k = all nodes within the diagonal of the k-th ring
            (k*spacing*sqrt(2)), with edges >= min_edge_length (see RINGS). None = all those
            within min(connection_radius, max_edge_length).
        origin_xy, origin_yaw: robot pose at start-up in the VISION frame. The lattice is
            anchored here. Ignored if `env` is passed.
        start_cell: cell where the robot is powered on (like env.start_cell).
        min_edge_length, max_edge_length, connection_radius: the same parameters as the PRM
            that will consume the graph -- they must match those passed to PRM().
        border_margin_m: margin to leave inside the mission border.
        env: an EnvironmentMap already created and with set_origin() done. If passed, size
            and pose come from it: it is the way to guarantee that graph and cell map cannot
            get out of sync.
        sampling: 'reticolo' or 'casuale' (see RANDOM NODES); None = P.MISSION_GRAPH_SAMPLING. With
            'casuale' spacing_m and rings do not count.
        distribution, density_per_m2, seed, poisson_min_dist_m: the parameters of the random
            nodes; None = P.MISSION_RANDOM_DISTRIBUTION, P.MISSION_RANDOM_DENSITY,
            P.MISSION_RANDOM_SEED, P.MISSION_POISSON_MIN_DIST_M. Seed None there too = drawn at random (and written in the log).

    Returns:
        MissionGraph.
    """
    if env is not None:
        rows, cols, cell_size_m = env.rows, env.cols, env.cell_size
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
        start_cell = tuple(env.start_cell)
    rows, cols, cell_size_m, chosen_spacing = mission_settings(rows, cols, cell_size_m)
    if spacing_m is _FROM_SETTINGS:
        spacing_m = chosen_spacing
    if rows < 1 or cols < 1 or cell_size_m <= 0:
        raise ValueError("dimensione della missione non valida")
    sampling = P.MISSION_GRAPH_SAMPLING if sampling is None else sampling
    if sampling not in GRAPH_SAMPLINGS:
        raise ValueError(f"metodo dei nodi sconosciuto {sampling!r}: uno di {GRAPH_SAMPLINGS}")
    if sampling == 'casuale':
        return _build_random_graph(
            rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
            float(min_edge_length), min(float(connection_radius), float(max_edge_length)),
            float(border_margin_m), verbose,
            P.MISSION_RANDOM_DISTRIBUTION if distribution is None else distribution,
            P.MISSION_RANDOM_DENSITY if density_per_m2 is None else density_per_m2,
            P.MISSION_RANDOM_SEED if seed is None else seed,
            P.MISSION_POISSON_MIN_DIST_M if poisson_min_dist_m is None else poisson_min_dist_m)

    if spacing_m is None:
        spacing_m = suggested_spacing(cell_size_m, min_edge_length)
    if float(spacing_m) <= 0:
        raise ValueError("la spaziatura deve essere positiva")
    requested = float(spacing_m)
    spacing_m, n_div = align_spacing(requested, cell_size_m)
    if abs(spacing_m - requested) > 1e-12 and verbose:
        print(f"[GRAFO] Passo allineato al lato cella: {requested:.3f} -> {spacing_m:.4f} m "
              f"(= {cell_size_m:.1f}/{n_div}); e' la condizione per cui ogni centro cella "
              f"e' un nodo del reticolo.")

    prm_reach = min(float(connection_radius), float(max_edge_length))
    if rings is None:
        reach_m = prm_reach
    else:
        rings = int(rings)
        if rings < 1:
            raise ValueError("rings deve essere almeno 1")
        ring_reach = rings * spacing_m * math.sqrt(2.0)      # diagonal of ring k
        if ring_reach > prm_reach + 1e-9 and verbose:
            print(f"[GRAFO] {rings} anelli arriverebbero a {ring_reach:.2f} m, oltre la "
                  f"portata del PRM {prm_reach:.2f} m: mi fermo li'.")
        # Relative tolerance (1e-6): the lattice distances are irrational multiples of the
        # spacing, and without tolerance the ring diagonal could fall outside by one ulp.
        reach_m = min(ring_reach, prm_reach) * (1.0 + 1e-6)
        if reach_m < float(min_edge_length):
            raise ValueError(
                f"con passo {spacing_m:.4f} m e {rings} anelli gli archi arrivano a "
                f"{reach_m:.2f} m, sotto min_edge_length {float(min_edge_length):.2f} m: "
                f"nessun arco sarebbe ammesso. Aumenta gli anelli o il passo.")

    extent = _mission_extent(rows, cols, cell_size_m, start_cell)
    ij, xy_local = _lattice_points(spacing_m, extent, float(border_margin_m))
    if len(xy_local) == 0:
        raise ValueError("nessun nodo nel rettangolo di missione: passo troppo grande "
                         "o margine di bordo troppo largo")
    pairs, lengths, n_short = _lattice_pairs(spacing_m, ij, xy_local,
                                             float(min_edge_length), reach_m)

    # --- The cell centres are lattice nodes by construction (spacing = cell_size/n):
    # here it is checked that this is really so instead of taking it for granted, because it is
    # the property this whole design choice rests on.
    cell_center_ids, tol = {}, max(1e-9, spacing_m * 1e-9)
    by_index = {(int(a), int(b)): k for k, (a, b) in enumerate(ij)}
    for r in range(rows):
        for c in range(cols):
            j = (c - start_cell[1]) * n_div
            i = (r - start_cell[0]) * n_div
            k = by_index.get((i, j))
            if k is None:
                continue                      # cell outside the lattice (border margin)
            if abs(xy_local[k, 0] - (c - start_cell[1]) * cell_size_m) > tol or \
                    abs(xy_local[k, 1] - (r - start_cell[0]) * cell_size_m) > tol:
                raise AssertionError(
                    f"il nodo {k} dovrebbe essere il centro della cella ({r},{c}) ma non lo "
                    f"e': passo {spacing_m} non allineato a cell_size {cell_size_m}")
            cell_center_ids[(r, c)] = k
    if verbose and len(cell_center_ids) < rows * cols:
        print(f"[GRAFO] {rows * cols - len(cell_center_ids)} centri cella fuori dal reticolo "
              f"(margine di bordo {border_margin_m:.2f} m): quelle celle non hanno obiettivo.")

    # Node at the origin: it is there by construction (the lattice is anchored there, and the origin is
    # also the centre of the starting cell), but we look for it instead of assuming it.
    d0 = np.hypot(xy_local[:, 0], xy_local[:, 1])
    origin_node_id = int(np.argmin(d0)) if d0.min() < 1e-9 else None

    node_cell = _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell)

    mg = MissionGraph(spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                      ij, xy_local, pairs, lengths, node_cell, cell_center_ids,
                      origin_node_id, float(min_edge_length), reach_m, rings, n_short)
    if verbose:
        print(mg.report())
    return mg


def _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell):
    """Cell (row, column) of every node: same rule as env.get_cell_from_world (round)."""
    cols_idx = np.clip(np.round(xy_local[:, 0] / cell_size_m).astype(np.int64) + start_cell[1],
                       0, cols - 1)
    rows_idx = np.clip(np.round(xy_local[:, 1] / cell_size_m).astype(np.int64) + start_cell[0],
                       0, rows - 1)
    return np.column_stack((rows_idx, cols_idx))


def _build_random_graph(rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                        min_edge_m, reach_m, border_margin_m, verbose,
                        distribution, density_per_m2, seed, poisson_min_dist_m):
    """
    The graph with random nodes (see RANDOM NODES): first the cell centres, in the order of
    rows and columns, then the nodes of the distribution. Same seed, same graph.
    """
    if distribution not in RANDOM_DISTRIBUTIONS:
        raise ValueError(f"distribuzione sconosciuta {distribution!r}: una di {RANDOM_DISTRIBUTIONS}")
    if density_per_m2 is None or float(density_per_m2) <= 0:
        raise ValueError("la densita' dei nodi deve essere positiva")
    if reach_m < min_edge_m:
        raise ValueError(f"lunghezza massima degli archi {reach_m:.2f} m sotto la minima "
                         f"{min_edge_m:.2f} m: nessun arco sarebbe ammesso")
    seed = draw_seed() if seed is None else int(seed)
    rng = np.random.default_rng(seed)

    x_min, x_max, y_min, y_max = _mission_extent(rows, cols, cell_size_m, start_cell)
    box = (x_min + border_margin_m, x_max - border_margin_m,
           y_min + border_margin_m, y_max - border_margin_m)
    if box[1] <= box[0] or box[3] <= box[2]:
        raise ValueError("margine di bordo troppo largo: nessuna area per i nodi")

    # Cell centres: fixed nodes, the exploration goals. The centre of the starting cell
    # is the origin, i.e. the power-on point.
    centers, keys = [], []
    for r in range(rows):
        for c in range(cols):
            x, y = (c - start_cell[1]) * cell_size_m, (r - start_cell[0]) * cell_size_m
            if box[0] <= x <= box[1] and box[2] <= y <= box[3]:
                centers.append((x, y))
                keys.append((r, c))
    centers = np.array(centers, dtype=np.float64).reshape(-1, 2)
    rand_xy, note = _random_points(distribution, density_per_m2, box, centers, rng,
                                   poisson_min_dist_m)
    xy_local = np.vstack((centers, rand_xy))
    cell_center_ids = {key: k for k, key in enumerate(keys)}
    if verbose and len(cell_center_ids) < rows * cols:
        print(f"[GRAFO] {rows * cols - len(cell_center_ids)} centri cella fuori dall'area "
              f"(margine di bordo {border_margin_m:.2f} m): quelle celle non hanno obiettivo.")
    d0 = np.hypot(xy_local[:, 0], xy_local[:, 1])
    origin_node_id = int(np.argmin(d0)) if len(d0) and d0.min() < 1e-9 else None

    pairs, lengths, n_short = _radius_pairs(xy_local, min_edge_m, reach_m)
    node_cell = _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell)
    mg = MissionGraph(1.0 / math.sqrt(float(density_per_m2)), rows, cols, cell_size_m,
                      start_cell, origin_xy, origin_yaw,
                      -np.ones((len(xy_local), 2), dtype=np.int64), xy_local, pairs, lengths,
                      node_cell, cell_center_ids, origin_node_id, min_edge_m, reach_m, None,
                      n_short, sampling='casuale', distribution=distribution,
                      density_per_m2=float(density_per_m2), seed=seed,
                      covering=measured_covering(xy_local, box), sampling_note=note)
    if verbose:
        print(mg.report())
    return mg


def load_npz(path, origin_xy=(0.0, 0.0), origin_yaw=0.0, env=None):
    """
    Reload a lattice saved with save_npz and bring it into the VISION frame with the given
    pose (or that of `env`): only the roto-translation is applied.
    """
    d = np.load(path, allow_pickle=False)
    if env is not None:
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
    keys = d['cell_center_keys'].reshape(-1, 2)
    vals = d['cell_center_vals'].reshape(-1)
    cc_ids = {(int(r), int(c)): int(v) for (r, c), v in zip(keys, vals)}
    oid, rings = int(d['origin_node_id']), int(d['rings'])
    return MissionGraph(
        float(d['spacing_m']), int(d['rows']), int(d['cols']), float(d['cell_size_m']),
        tuple(int(v) for v in d['start_cell']), origin_xy, origin_yaw, d['ij'], d['xy_local'],
        d['pairs'], d['lengths'], d['node_cell'], cc_ids, None if oid < 0 else oid,
        float(d['min_edge_m']), float(d['reach_m']), None if rings < 0 else rings,
        int(d['n_short_pairs']),
        # files saved before random nodes: lattice
        **({} if 'sampling' not in d.files else dict(
            sampling=str(d['sampling']), distribution=(str(d['distribution']) or None),
            density_per_m2=(None if float(d['density_per_m2']) < 0 else float(d['density_per_m2'])),
            seed=(None if int(d['seed']) < 0 else int(d['seed'])),
            covering=(None if float(d['covering'][0]) < 0 else tuple(float(v) for v in d['covering'])),
            sampling_note=str(d['sampling_note']))))


def compare(rows=DEFAULT_MISSION_ROWS, cols=DEFAULT_MISSION_COLS,
            cell_size_m=DEFAULT_CELL_SIZE_M, divisors=(4, 5, 6, 7, 8, 9, 10),
            rings_list=(1, 2, 3, None), **kw):
    """
    Table of nodes/edges/coverage as the spacing (as cell_size/n) and the rings vary:
    serves to choose the flags by looking at the numbers.
    """
    print(f"Missione {rows}x{cols} celle da {cell_size_m:.1f} m "
          f"= {rows * cols * cell_size_m ** 2:.0f} m2")
    print(f"{'passo':>8}{'(n)':>5}{'anelli':>8}{'nodi':>7}{'archi':>8}{'grado':>7}"
          f"{'lung.min':>10}{'lung.max':>10}{'copert.':>9}{'conn.':>7}{'corti':>7}")
    out = []
    for n in divisors:
        s = cell_size_m / n
        for rg in rings_list:
            try:
                mg = build_mission_graph(rows=rows, cols=cols, cell_size_m=cell_size_m,
                                         spacing_m=s, rings=rg, verbose=False, **kw)
            except ValueError as e:
                print(f"{s:>8.4f}{n:>5}{str(rg):>8}  non costruibile: {e}")
                continue
            lo = float(mg.lengths.min()) if mg.n_edges else float('nan')
            hi = float(mg.lengths.max()) if mg.n_edges else float('nan')
            out.append(dict(spacing=mg.spacing_m, n=n, rings=rg, nodes=mg.n_nodes,
                            edges=mg.n_edges, degree=float(mg.degrees.mean()),
                            covering=mg.covering_m, connected=mg.is_connected))
            print(f"{mg.spacing_m:>8.4f}{n:>5}{str(rg):>8}{mg.n_nodes:>7}{mg.n_edges:>8}"
                  f"{mg.degrees.mean():>7.1f}{lo:>10.2f}{hi:>10.2f}{mg.covering_m:>9.2f}"
                  f"{('si' if mg.is_connected else 'NO'):>7}{mg.n_short_pairs:>7}")
    return out


def nodes_per_cell(mg):
    """
    Nodes in every cell, (rows, columns), counting a node that lies exactly on the boundary
    between two cells as half in each (a quarter at the crossings). The node_cell count
    instead assigns those nodes all to one of the two cells (same rule as the mission), and
    for the lattice, which has whole rows on the boundaries, makes some cells look fuller.
    The nodes on the outer border of the area count fully in their cell.
    """
    out = np.zeros((mg.rows, mg.cols), dtype=np.float64)
    eps = 1e-9

    def owners(u, n):
        # u: position in cell units, with the centres on the integers; boundaries at k + 0.5.
        b = u - 0.5
        if abs(b - round(b)) < eps:
            cand = [int(round(b)), int(round(b)) + 1]
        else:
            cand = [int(round(u))]
        return [k for k in cand if 0 <= k < n] or [min(max(int(round(u)), 0), n - 1)]

    for x, y in mg.xy_local:
        cs = owners(x / mg.cell_size_m + mg.start_cell[1], mg.cols)
        rs = owners(y / mg.cell_size_m + mg.start_cell[0], mg.rows)
        w = 1.0 / (len(cs) * len(rs))
        for r in rs:
            for c in cs:
                out[r, c] += w
    return out


def lattice_matched_density(**kw):
    """
    The random-node density that gives the SAME total number of nodes as the lattice with
    these parameters (outer border included, cell centres apart, as _build_random_graph
    counts them). Serves to compare the methods with the same number of nodes.
    """
    kw = {k: v for k, v in kw.items()
          if k not in ('sampling', 'distribution', 'density_per_m2', 'seed', 'verbose')}
    ref = build_mission_graph(sampling='reticolo', verbose=False, **kw)
    margin = float(kw.get('border_margin_m', 0.0))
    x0, x1, y0, y1 = _mission_extent(ref.rows, ref.cols, ref.cell_size_m, ref.start_cell)
    area = (x1 - x0 - 2 * margin) * (y1 - y0 - 2 * margin)
    return (ref.n_nodes - len(ref.cell_center_ids)) / area


def compare_methods_png(path, zoom=(-2.0, 2.0, -2.0, 2.0), seed=None,
                        methods=('reticolo',) + RANDOM_DISTRIBUTIONS, verbose=False, **kw):
    """
    The node methods side by side (default: the lattice and the four random distributions),
    with the same parameters (mission, edges, density) and the same seed, to see the
    difference at a glance. At the end the table: nodes, edges, mean degree, maximum and
    95% coverage. zoom None = the whole area. Seed None = drawn at random, written in the
    title. verbose: also print the [GRAFO] summary of every graph.
    density_per_m2 None = the density that gives the same number of nodes as the lattice
    (lattice_matched_density).
    Returns (path, graphs).
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    seed = draw_seed() if seed is None else int(seed)
    kw = {k: v for k, v in kw.items() if k not in ('sampling', 'distribution')}
    if kw.get('density_per_m2') is None:
        kw['density_per_m2'] = lattice_matched_density(**kw)
    graphs = []
    for m in methods:
        if verbose:
            print(f"\n--- {m}")
        if m == 'reticolo':
            graphs.append(build_mission_graph(sampling='reticolo', verbose=verbose, **kw))
        else:
            graphs.append(build_mission_graph(sampling='casuale', distribution=m, seed=seed,
                                              verbose=verbose, **kw))
    n_cols = 3 if len(graphs) >= 3 else len(graphs) + 1
    n_rows = int(math.ceil((len(graphs) + 1) / n_cols))
    if zoom:
        panel_w, panel_h = 5.6, 5.6
    else:                          # whole area: panels with the mission proportions
        g0 = graphs[0]
        ratio = (g0.rows * g0.cell_size_m) / (g0.cols * g0.cell_size_m)
        panel_w, panel_h = 6.4, max(3.0, 6.4 * ratio) + 1.0
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(panel_w * n_cols, panel_h * n_rows + 0.8),
                             squeeze=False)
    for k, (ax, mg) in enumerate(zip(axes.ravel(), graphs)):
        # Grey edges, lighter and thinner when there are many; highlighted those leaving
        # the starting node: it shows which neighbours are linked.
        many = mg.n_edges > 20000
        mg.draw(ax, zoom=zoom, legend=(k == 0), title_size=10,
                edge_color='0.88' if many else '0.70', edge_width=0.2 if many else 0.35,
                highlight=mg.origin_node_id)
    for ax in axes.ravel()[len(graphs) + 1:]:
        ax.axis('off')
    # Panel after the last graph: the table of numbers.
    ax = axes.ravel()[len(graphs)]
    ax.axis('off')
    rows = []
    for mg in graphs:
        name = 'reticolo' if not mg.is_random else mg.distribution
        if mg.covering is None:
            # Lattice: the maximum from the formula (the measurement grid may miss its point).
            cov = (mg.covering_m, measured_covering(mg.xy_local, _mission_extent(
                mg.rows, mg.cols, mg.cell_size_m, mg.start_cell))[1])
        else:
            cov = mg.covering
        per_cell = nodes_per_cell(mg)
        rows.append([name, f"{mg.n_nodes}", f"{per_cell.min():.0f}-{per_cell.max():.0f}",
                     f"{mg.n_edges}", f"{mg.degrees.mean():.1f}",
                     f"{cov[0]:.2f}", f"{cov[1]:.2f}"])
    tab = ax.table(cellText=rows, colLabels=['metodo', 'nodi', 'per cella\n(min-max)', 'archi',
                                             'grado\nmedio', 'copertura\nmassima [m]',
                                             '95% area\nentro [m]'],
                   loc='center', cellLoc='center')
    tab.auto_set_font_size(False)
    tab.set_fontsize(10)
    tab.auto_set_column_width(list(range(7)))
    tab.scale(1.0, 2.4)
    g0 = graphs[0]
    rnd = [g for g in graphs if g.is_random]
    lat = [g for g in graphs if not g.is_random]
    fig.suptitle(f"Missione {g0.rows}x{g0.cols} celle da {g0.cell_size_m:g} m, archi "
                 f"{g0.min_edge_m:g}-{g0.reach_m:g} m"
                 + (f", reticolo passo {lat[0].spacing_m:g} m" if lat else "")
                 + (f", casuali {rnd[0].density_per_m2:.2f} nodi/m2, seme {seed}" if rnd else "")
                 + (f" -- riquadro x {zoom[0]:g}..{zoom[1]:g}, y {zoom[2]:g}..{zoom[3]:g} m"
                    if zoom else ""), fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96), h_pad=3.0)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path, graphs


def timing(path=None, **kw):
    """How much building the lattice from scratch costs compared with reloading it from file."""
    import os
    import tempfile
    import time
    t0 = time.perf_counter()
    mg = build_mission_graph(verbose=False, **kw)
    t_build = time.perf_counter() - t0
    path = path or os.path.join(tempfile.gettempdir(), "mission_graph_timing.npz")
    mg.save_npz(path)
    t0 = time.perf_counter()
    mg2 = load_npz(path, origin_xy=(1.0, 2.0), origin_yaw=0.7)
    t_load = time.perf_counter() - t0
    print(f"[TEMPI] {mg.n_nodes} nodi, {mg.n_edges} archi")
    print(f"[TEMPI] costruzione da zero : {t_build * 1000:8.1f} ms")
    print(f"[TEMPI] ricarica da file    : {t_load * 1000:8.1f} ms  "
          f"({os.path.getsize(path) / 1024:.0f} KB su disco)")
    print(f"[TEMPI] file: {path}")
    return t_build, t_load, mg2


def _main():
    import argparse
    p = argparse.ArgumentParser(description="Grafo di missione regolare per Spot")
    s_rows, s_cols, s_cell, s_spacing = mission_settings()
    p.add_argument('--rows', type=int, default=s_rows,
                   help=f"righe di celle, a sinistra del robot (default {s_rows})")
    p.add_argument('--cols', type=int, default=s_cols,
                   help=f"colonne di celle, davanti al robot (default {s_cols})")
    p.add_argument('--cell', type=float, default=s_cell,
                   help=f"lato della cella in m (default {s_cell:g})")
    p.add_argument('--spacing', type=float, default=s_spacing,
                   help=f"passo fra i nodi in m, arrotondato a cell/n (default "
                        f"{s_spacing:g}; 0 = suggested_spacing)")
    p.add_argument('--rings', default=str(P.MISSION_LATTICE_RINGS),
                   help="anelli di vicini collegati (k = entro la diagonale dell'anello k), "
                        "o 'tutti' (default: tutti, come in missione)")
    p.add_argument('--graph', choices=GRAPH_SAMPLINGS, default=P.MISSION_GRAPH_SAMPLING,
                   help=f"come si generano i nodi (default {P.MISSION_GRAPH_SAMPLING})")
    p.add_argument('--distribution', choices=RANDOM_DISTRIBUTIONS, default=P.MISSION_RANDOM_DISTRIBUTION,
                   help=f"nodi casuali: distribuzione (default {P.MISSION_RANDOM_DISTRIBUTION})")
    p.add_argument('--density', type=float, default=P.MISSION_RANDOM_DENSITY,
                   help=f"nodi casuali: nodi per m2 (default {P.MISSION_RANDOM_DENSITY:g})")
    p.add_argument('--seed', type=int, default=P.MISSION_RANDOM_SEED,
                   help="nodi casuali: seme (default: estratto a caso e scritto nel riepilogo)")
    p.add_argument('--min-dist', type=float, default=P.MISSION_POISSON_MIN_DIST_M,
                   help="nodi poisson: distanza minima fra i nodi in m (default 0.75/sqrt(densita'))")
    p.add_argument('--min-edge', type=float, default=P.PRM_MIN_EDGE_M)
    p.add_argument('--max-edge', type=float, default=P.PRM_MAX_EDGE_M)
    p.add_argument('--radius', type=float, default=P.PRM_CONNECTION_RADIUS_M)
    p.add_argument('--margin', type=float, default=0.0, help="margine dal bordo in m")
    p.add_argument('--preview', metavar='PNG', help="salva un disegno del reticolo")
    p.add_argument('--zoom', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                   help="riquadro del disegno, in coordinate griglia")
    p.add_argument('--save', metavar='NPZ', help="salva il reticolo (si ricarica con load_npz)")
    p.add_argument('--compare', action='store_true', help="tabella passo x anelli")
    p.add_argument('--compare-methods', metavar='PNG',
                   help="reticolo e le quattro distribuzioni casuali affiancati in un PNG "
                        "(stessi parametri e seme; riquadro --zoom, default -2 2 -2 2)")
    p.add_argument('--timings', nargs='?', const=True, metavar='NPZ',
                   help="misura costruzione da zero contro ricarica da file")
    a = p.parse_args()

    rings = None if str(a.rings).lower() in ('none', 'tutti', 'all', '-1') else int(a.rings)
    if a.spacing is not None and a.spacing <= 0:
        a.spacing = None                 # 0 = spacing suggested by suggested_spacing()
    common = dict(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                  max_edge_length=a.max_edge, connection_radius=a.radius,
                  border_margin_m=a.margin, sampling=a.graph, distribution=a.distribution,
                  density_per_m2=a.density, seed=a.seed, poisson_min_dist_m=a.min_dist)

    if a.compare_methods:
        common.pop('sampling')
        common.pop('distribution')
        common.pop('seed')
        print(f"[GRAFO] confronto salvato in "
              f"{compare_methods_png(a.compare_methods, zoom=tuple(a.zoom) if a.zoom else (-2.0, 2.0, -2.0, 2.0), seed=a.seed, spacing_m=a.spacing, rings=rings, **common)[0]}")
        return
    if a.comparison:
        compare(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                sampling='reticolo',
                max_edge_length=a.max_edge, connection_radius=a.radius,
                border_margin_m=a.margin)
        return
    if a.timings:
        timing(None if a.timings is True else a.timings, spacing_m=a.spacing, rings=rings, **common)
        return

    mg = build_mission_graph(spacing_m=a.spacing, rings=rings, **common)
    if a.preview:
        print(f"[GRAFO] disegno salvato in "
              f"{mg.preview_png(a.preview, zoom=tuple(a.zoom) if a.zoom else None)}")
    if a.save:
        print(f"[GRAFO] reticolo salvato in {mg.save_npz(a.save)}")


# =========================================================================================
# THE GRAPH DURING THE MISSION: PRM construction, edge update, cells
# allowed for planning
# =========================================================================================
def build_mission_prm(env, verbose=True):
    """
    Create the mission graph (lattice or random nodes, see MISSION_GRAPH_SAMPLING) and the
    PRM with its nodes, from the mission size and the starting pose `env` already holds
    (set_origin must be done BEFORE).

    Returns (prm, sampler, mission_lattice): `sampler` gives the nodes cell by cell (see
    mission_graph.LatticeSampler), `mission_lattice` is the MissionGraph.
    The EDGES are not created here: they need the terrain data, see rebuild_prm_edges.
    """
    mg = build_mission_graph(
        env=env, spacing_m=P.MISSION_LATTICE_SPACING_M, rings=P.MISSION_LATTICE_RINGS,
        min_edge_length=P.PRM_MIN_EDGE_M, max_edge_length=P.PRM_MAX_EDGE_M,
        connection_radius=P.PRM_CONNECTION_RADIUS_M, verbose=verbose,
        sampling=P.MISSION_GRAPH_SAMPLING, distribution=P.MISSION_RANDOM_DISTRIBUTION,
        density_per_m2=P.MISSION_RANDOM_DENSITY, seed=P.MISSION_RANDOM_SEED)
    prm = prm_graph.PRM(min_edge_length=P.PRM_MIN_EDGE_M, max_edge_length=mg.reach_m,
                        connection_radius=P.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg          # rebuild_prm_edges reads it again
    prm.set_detour_limit(P.PLAN_MAX_DETOUR_RATIO, P.PLAN_DETOUR_SLACK_M)
    mg.into_prm(prm)
    return prm, mg.as_sampler(), mg


def node_cells(prm, env):
    """
    {node id: (row, column)}. From the lattice when there is one (mission_graph.node_cell,
    same rule as env.get_cell_from_world), otherwise from env. Cached on the PRM: the nodes
    do not change during the mission.
    """
    cache = getattr(prm, '_node_cell_cache', None)
    if cache is None:
        cache = {}
        prm._node_cell_cache = cache
    mg = getattr(prm, 'mission_graph', None)
    for nid, (x, y) in prm.nodes.items():
        if nid in cache:
            continue
        if mg is not None and 0 <= nid < mg.n_nodes:
            r, c = mg.node_cell[nid]
            cache[nid] = (int(r), int(c))
        else:
            cache[nid] = tuple(int(v) for v in env.get_cell_from_world(x, y))
    return cache


def graph_reaches_cell(prm, env, from_xy, row, col, global_map=None):
    """
    From the point from_xy (the waypoint the cell would be attempted from) does the graph
    reach at least one node of cell (row, col)? Same rules as the attempt: with
    PLAN_ONLY_EXPLORED_CELLS it passes only through the visited cells, the starting cell and
    the goal cell; edges and nodes as they are now (a node with infinite penalty does not
    count). It is a NECESSARY condition: if False the attempt would surely fail with "GRAFO
    DISCONNESSO"; if True the attempt decides (goal point, detour cap, frontier).
    Breadth-first search, no effect on the graph. None if the graph has no nodes.
    """
    start = prm.anchor_node(float(from_xy[0]), float(from_xy[1]), global_map,
                            geometry.PRM_EDGE_SAFETY_MARGIN_M,
                            ignore_near_m=prm_graph.ANCHOR_IGNORE_NEAR_M)
    if start is None:
        return None
    cells = node_cells(prm, env)
    target = (int(row), int(col))
    if P.PLAN_ONLY_EXPLORED_CELLS:
        ok_cells = {(r, c) for r in range(env.rows) for c in range(env.cols)
                    if env.is_cell_visited(r, c)}
        ok_cells.add(target)
        ok_cells.add(tuple(int(v) for v in env.get_cell_from_world(from_xy[0], from_xy[1])))
    else:
        ok_cells = None
    seen, queue = {start}, [start]
    while queue:
        u = queue.pop()
        if cells.get(u) == target and np.isfinite(prm.node_penalty(u)):
            return True
        for v, _w in prm.edges.get(u, ()):
            if v in seen:
                continue
            if ok_cells is not None and cells.get(v) not in ok_cells:
                continue
            seen.add(v)
            queue.append(v)
    return False


def restrict_planning_to_explored_cells(prm, env, target_row, target_col, robot_xy):
    """
    Limit the path search to the nodes of the visited cells, of the robot's cell and of the
    goal cell (see PLAN_ONLY_EXPLORED_CELLS). To be called before every cell attempt: the
    visited cells change from one attempt to the next.
    """
    if not P.PLAN_ONLY_EXPLORED_CELLS:
        prm.set_allowed_nodes(None)
        return
    cells = {(r, c) for r in range(env.rows) for c in range(env.cols) if env.is_cell_visited(r, c)}
    cells.add((int(target_row), int(target_col)))
    cells.add(tuple(int(v) for v in env.get_cell_from_world(robot_xy[0], robot_xy[1])))
    allowed = [nid for nid, cell in node_cells(prm, env).items() if cell in cells]
    prm.set_allowed_nodes(allowed)
    print(f"[PIANO] Percorsi ammessi solo nelle celle {sorted(cells)}: {len(allowed)} nodi su "
          f"{len(prm.nodes)}. Se serve passare da una cella non visitata, rinuncio a questo "
          f"lato e passo a un'altra cella.")


def robot_on_node(prm, node_id, x, y):
    """True if the robot is practically on the node (see roadmap.ANCHOR_ON_NODE_M)."""
    pos = prm.get_node_position(node_id)
    return pos is not None and float(np.hypot(pos[0] - x, pos[1] - y)) <= prm_graph.ANCHOR_ON_NODE_M


def rebuild_prm_edges(prm, global_map, incremental=True):
    """
    Re-evaluate the PRM edges on the current data, on the lattice pairs already computed
    (see mission_graph.MissionGraph.add_edges_to_prm). Vetoes and weights are computed by
    PRM._evaluate_and_add_edge. incremental=False forces the full build (same result:
    test_replay.py checks it).
    """
    return prm.mission_graph.add_edges_to_prm(
        prm, global_map=global_map,
        edge_safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M,
        incremental=incremental)


if __name__ == '__main__':
    _main()