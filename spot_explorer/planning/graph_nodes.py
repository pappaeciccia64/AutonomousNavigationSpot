"""
graph_nodes.py -- the nodes and the geometric pairs of the mission graph (see mission_graph.py):
the rectangular lattice or the random nodes, the coverage, the connected components.

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
  condition that makes every cell centre a lattice node. The [GRAPH] line of the log gives
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

RANDOM NODES (P.MISSION_GRAPH_SAMPLING = 'random')
  As an alternative to the lattice, the nodes can be scattered at random over the mission
  area, to compare the two methods. The cell centres ALWAYS stay nodes (they are the goals,
  and the centre of the starting cell is the power-on point); the other nodes follow one of
  these distributions, with P.MISSION_RANDOM_DENSITY nodes per m^2:
    'uniform'   independent uniform points: the classic PRM, with holes and clumps;
    'poisson'    Poisson-disk: uniform points discarded if closer than a minimum distance
                 to a node already placed (P.MISSION_POISSON_MIN_DIST_M; None = 0.75/sqrt(density),
                 which lets the requested density be reached): no clumps;
    'jittered' the area divided into squares of 1/density m^2, one random point in each
                 (a few squares removed or points added at random to reach the exact
                 number): no area stays empty for more than one square;
    'halton'     Halton sequence in bases 2 and 3, quasi-random and very uniform; the
                 seed shifts it (Cranley-Patterson rotation), so different seeds give
                 different sets.
  The edges link every pair of nodes between min_edge_length and
  min(max_edge_length, connection_radius) apart; spacing and rings do not count.
  SEED: a fixed P.MISSION_RANDOM_SEED repeats the same graph bit for bit; None = a seed drawn
  at random. In both cases the seed used goes into the log ([GRAPH] Random nodes line) and into
  mission_summary.json, so the mission can be repeated and test_replay replays it with
  the same graph.
  The coverage (maximum distance between a point of the area and the nearest node) has no
  formula here: it is measured on a dense grid and goes into the log, because it is the
  number that tells whether a narrow corridor can be left without nodes (see ALIASING).
"""

import math
from collections import deque

import numpy as np

from spot_explorer import config as P


# ----------------------------------------------------------------------------------------
# HOW THE NODES ARE GENERATED (see RANDOM NODES above).
#   'lattice' the rectangular lattice, deterministic (default);
#   'random'  scattered nodes with P.MISSION_RANDOM_DISTRIBUTION, cell centres always included.
# They are changed for a single mission with the arguments of run_mission.py (--graph,
# --distribution, --density, --seed).
# ----------------------------------------------------------------------------------------
GRAPH_SAMPLINGS = ('lattice', 'random')
RANDOM_DISTRIBUTIONS = ('uniform', 'poisson', 'jittered', 'halton')


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


def mission_extent(rows, cols, cell_size_m, start_cell):
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


def lattice_points(spacing, extent, border_margin_m):
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


def lattice_pairs(spacing, ij, xy, min_edge_m, reach_m):
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


def random_points(distribution, density_per_m2, box, fixed_xy, rng, min_dist=None):
    """
    The random nodes in the rectangle box = (x_min, x_max, y_min, y_max), without the fixed
    nodes (cell centres), which poisson respects though. Returns (xy, note): note is a
    string for the log when the requested density was not reached.
    """
    x0, x1, y0, y1 = box
    w, h = x1 - x0, y1 - y0
    n = int(round(float(density_per_m2) * w * h))
    if distribution == 'uniform':
        u = rng.random((n, 2))
        return np.column_stack((x0 + u[:, 0] * w, y0 + u[:, 1] * h)), ""
    if distribution == 'jittered':
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
                f"area saturated at {len(xy)} nodes out of {n} requested with minimum distance {r:.3f} m")
        return xy, note
    raise ValueError(f"unknown distribution {distribution!r}: one of {RANDOM_DISTRIBUTIONS}")


def radius_pairs(xy, min_edge_m, reach_m):
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


def connected_components(n_nodes, pairs):
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
