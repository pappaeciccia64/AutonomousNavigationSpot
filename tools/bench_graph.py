"""
bench_graph.py -- look at the possible mission graphs, without the robot, and pick one.

    python3 tools/bench_graph.py

By default it uses THE SAME parameters as the mission (config.py): area, edges, lattice
spacing, density and seed of the random nodes. It makes an image with the methods side by
side (same area, same edges, same seed), a table with nodes, edges, mean degree and
coverage, and opens it. In the terminal it prints the [GRAPH] summary of each graph, as in
the mission log, the nodes per cell and, at the end, HOW TO USE IN A MISSION each of the
graphs shown.

To try different values without touching config.py, change the TEST block below
(None = the value from config.py). They only apply to the image.

The graph is relative to the robot's pose at power-on: same parameters and same seed give
the same graph, in the same position relative to the starting point.

With options, one graph or one comparison at a time (cli), without the TEST block:
    python3 tools/bench_graph.py --compare
    python3 tools/bench_graph.py --spacing 0.25 --rings 2 --preview p.png --zoom -2 2 -2 2
    python3 tools/bench_graph.py --graph random --distribution poisson --density 16 --seed 7 --preview p.png
    python3 tools/bench_graph.py --compare-methods methods.png --seed 7 --zoom -2 2 -2 2
    python3 tools/bench_graph.py --rows 2 --cols 3 --cell 3
    python3 tools/bench_graph.py --save lattice_2x4x5.npz
    python3 tools/bench_graph.py --timings
"""
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # the spot_explorer package

from spot_explorer.planning.mission_graph import (DEFAULT_CELL_SIZE_M, DEFAULT_MISSION_COLS, DEFAULT_MISSION_ROWS,
                                                  build_mission_graph, load_npz, mission_settings)
from spot_explorer.planning.graph_nodes import (GRAPH_SAMPLINGS, RANDOM_DISTRIBUTIONS, draw_seed,
                                                measured_covering, mission_extent)
from spot_explorer import config as P

# ======================================================================================
# TEST: None = as in config.py
# ======================================================================================
# Area: rows (to the robot's left) x columns (ahead) x cell side.
ROWS = None                 # config.MISSION_GRID_ROWS
COLS = None               # config.MISSION_GRID_COLS
CELL_SIZE_M = None          # config.MISSION_CELL_SIZE_M

# Edges: all pairs of nodes between EDGE_MIN_M and EDGE_MAX_M apart.
EDGE_MIN_M = None            # config.PRM_MIN_EDGE_M
EDGE_MAX_M = None            # config.PRM_MAX_EDGE_M

# Lattice: spacing between nodes, rounded to CELL_SIZE_M / n; rings of linked neighbours.
LATTICE_SPACING_M = None      # config.MISSION_LATTICE_SPACING_M
RINGS = None                # config.MISSION_LATTICE_RINGS (None there too = all)

# Random nodes: nodes per m2 ('match_lattice' = same number of nodes as the lattice) and seed.
# Seed None (also in config.py) = drawn at random: it is written in the image title and
# at the end, to rebuild the same graph in a mission.
NODE_DENSITY_M2 = None       # config.MISSION_RANDOM_DENSITY
SEED = None                  # config.MISSION_RANDOM_SEED

# Which methods to compare: remove the ones you don't want to see.
METHODS = ['lattice', 'uniform', 'poisson', 'jittered', 'halton']

# Region to draw: None = the whole area; or (x_min, x_max, y_min, y_max) in metres,
# with the robot at (0, 0), x forward and y to the left. Example: (-2, 2, -2, 2).
ZOOM = None

# Image to save, and whether to open it at the end.
IMAGE = 'graphs.png'
OPEN = True
# ======================================================================================


def compare(rows=DEFAULT_MISSION_ROWS, cols=DEFAULT_MISSION_COLS,
            cell_size_m=DEFAULT_CELL_SIZE_M, divisors=(4, 5, 6, 7, 8, 9, 10),
            rings_list=(1, 2, 3, None), **kw):
    """
    Table of nodes/edges/coverage as the spacing (as cell_size/n) and the rings vary:
    serves to choose the flags by looking at the numbers.
    """
    print(f"Mission {rows}x{cols} cells of {cell_size_m:.1f} m "
          f"= {rows * cols * cell_size_m ** 2:.0f} m2")
    print(f"{'spacing':>8}{'(n)':>5}{'rings':>8}{'nodes':>7}{'edges':>8}{'degree':>7}"
          f"{'len.min':>10}{'len.max':>10}{'cover.':>9}{'conn.':>7}{'short':>7}")
    out = []
    for n in divisors:
        s = cell_size_m / n
        for rg in rings_list:
            try:
                mg = build_mission_graph(rows=rows, cols=cols, cell_size_m=cell_size_m,
                                         spacing_m=s, rings=rg, verbose=False, **kw)
            except ValueError as e:
                print(f"{s:>8.4f}{n:>5}{str(rg):>8}  cannot be built: {e}")
                continue
            lo = float(mg.lengths.min()) if mg.n_edges else float('nan')
            hi = float(mg.lengths.max()) if mg.n_edges else float('nan')
            out.append(dict(spacing=mg.spacing_m, n=n, rings=rg, nodes=mg.n_nodes,
                            edges=mg.n_edges, degree=float(mg.degrees.mean()),
                            covering=mg.covering_m, connected=mg.is_connected))
            print(f"{mg.spacing_m:>8.4f}{n:>5}{str(rg):>8}{mg.n_nodes:>7}{mg.n_edges:>8}"
                  f"{mg.degrees.mean():>7.1f}{lo:>10.2f}{hi:>10.2f}{mg.covering_m:>9.2f}"
                  f"{('yes' if mg.is_connected else 'NO'):>7}{mg.n_short_pairs:>7}")
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
    ref = build_mission_graph(sampling='lattice', verbose=False, **kw)
    margin = float(kw.get('border_margin_m', 0.0))
    x0, x1, y0, y1 = mission_extent(ref.rows, ref.cols, ref.cell_size_m, ref.start_cell)
    area = (x1 - x0 - 2 * margin) * (y1 - y0 - 2 * margin)
    return (ref.n_nodes - len(ref.cell_center_ids)) / area


def compare_methods_png(path, zoom=(-2.0, 2.0, -2.0, 2.0), seed=None,
                        methods=('lattice',) + RANDOM_DISTRIBUTIONS, verbose=False, **kw):
    """
    The node methods side by side (default: the lattice and the four random distributions),
    with the same parameters (mission, edges, density) and the same seed, to see the
    difference at a glance. At the end the table: nodes, edges, mean degree, maximum and
    95% coverage. zoom None = the whole area. Seed None = drawn at random, written in the
    title. verbose: also print the [GRAPH] summary of every graph.
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
        if m == 'lattice':
            graphs.append(build_mission_graph(sampling='lattice', verbose=verbose, **kw))
        else:
            graphs.append(build_mission_graph(sampling='random', distribution=m, seed=seed,
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
        name = 'lattice' if not mg.is_random else mg.distribution
        if mg.covering is None:
            # Lattice: the maximum from the formula (the measurement grid may miss its point).
            cov = (mg.covering_m, measured_covering(mg.xy_local, mission_extent(
                mg.rows, mg.cols, mg.cell_size_m, mg.start_cell))[1])
        else:
            cov = mg.covering
        per_cell = nodes_per_cell(mg)
        rows.append([name, f"{mg.n_nodes}", f"{per_cell.min():.0f}-{per_cell.max():.0f}",
                     f"{mg.n_edges}", f"{mg.degrees.mean():.1f}",
                     f"{cov[0]:.2f}", f"{cov[1]:.2f}"])
    tab = ax.table(cellText=rows, colLabels=['method', 'nodes', 'per cell\n(min-max)', 'edges',
                                             'mean\ndegree', 'coverage\nmaximum [m]',
                                             '95% of area\nwithin [m]'],
                   loc='center', cellLoc='center')
    tab.auto_set_font_size(False)
    tab.set_fontsize(10)
    tab.auto_set_column_width(list(range(7)))
    tab.scale(1.0, 2.4)
    g0 = graphs[0]
    rnd = [g for g in graphs if g.is_random]
    lat = [g for g in graphs if not g.is_random]
    fig.suptitle(f"Mission {g0.rows}x{g0.cols} cells of {g0.cell_size_m:g} m, edges "
                 f"{g0.min_edge_m:g}-{g0.reach_m:g} m"
                 + (f", lattice spacing {lat[0].spacing_m:g} m" if lat else "")
                 + (f", random {rnd[0].density_per_m2:.2f} nodes/m2, seed {seed}" if rnd else "")
                 + (f" -- region x {zoom[0]:g}..{zoom[1]:g}, y {zoom[2]:g}..{zoom[3]:g} m"
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
    print(f"[TIMES] {mg.n_nodes} nodes, {mg.n_edges} edges")
    print(f"[TIMES] build from scratch : {t_build * 1000:8.1f} ms")
    print(f"[TIMES] reload from file   : {t_load * 1000:8.1f} ms  "
          f"({os.path.getsize(path) / 1024:.0f} KB on disk)")
    print(f"[TIMES] file: {path}")
    return t_build, t_load, mg2


def cli():
    """The command-line options (see the top of this file)."""
    import argparse
    p = argparse.ArgumentParser(description="Regular mission graph for Spot")
    s_rows, s_cols, s_cell, s_spacing = mission_settings()
    p.add_argument('--rows', type=int, default=s_rows,
                   help=f"rows of cells, to the robot's left (default {s_rows})")
    p.add_argument('--cols', type=int, default=s_cols,
                   help=f"columns of cells, ahead of the robot (default {s_cols})")
    p.add_argument('--cell', type=float, default=s_cell,
                   help=f"cell side in m (default {s_cell:g})")
    p.add_argument('--spacing', type=float, default=s_spacing,
                   help=f"spacing between nodes in m, rounded to cell/n (default "
                        f"{s_spacing:g}; 0 = suggested_spacing)")
    p.add_argument('--rings', default=str(P.MISSION_LATTICE_RINGS),
                   help="rings of linked neighbours (k = within the diagonal of ring k), "
                        "or 'all' (default: all, as in the mission)")
    p.add_argument('--graph', choices=GRAPH_SAMPLINGS, default=P.MISSION_GRAPH_SAMPLING,
                   help=f"how the nodes are generated (default {P.MISSION_GRAPH_SAMPLING})")
    p.add_argument('--distribution', choices=RANDOM_DISTRIBUTIONS, default=P.MISSION_RANDOM_DISTRIBUTION,
                   help=f"random nodes: distribution (default {P.MISSION_RANDOM_DISTRIBUTION})")
    p.add_argument('--density', type=float, default=P.MISSION_RANDOM_DENSITY,
                   help=f"random nodes: nodes per m2 (default {P.MISSION_RANDOM_DENSITY:g})")
    p.add_argument('--seed', type=int, default=P.MISSION_RANDOM_SEED,
                   help="random nodes: seed (default: drawn at random and written in the summary)")
    p.add_argument('--min-dist', type=float, default=P.MISSION_POISSON_MIN_DIST_M,
                   help="poisson nodes: minimum distance between nodes in m (default 0.75/sqrt(density))")
    p.add_argument('--min-edge', type=float, default=P.PRM_MIN_EDGE_M)
    p.add_argument('--max-edge', type=float, default=P.PRM_MAX_EDGE_M)
    p.add_argument('--radius', type=float, default=P.PRM_CONNECTION_RADIUS_M)
    p.add_argument('--margin', type=float, default=0.0, help="margin from the border in m")
    p.add_argument('--preview', metavar='PNG', help="save a drawing of the lattice")
    p.add_argument('--zoom', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                   help="region of the drawing, in grid coordinates")
    p.add_argument('--save', metavar='NPZ', help="save the lattice (reload it with load_npz)")
    p.add_argument('--compare', action='store_true', help="spacing x rings table")
    p.add_argument('--compare-methods', metavar='PNG',
                   help="lattice and the four random distributions side by side in a PNG "
                        "(same parameters and seed; region --zoom, default -2 2 -2 2)")
    p.add_argument('--timings', nargs='?', const=True, metavar='NPZ',
                   help="measure building from scratch against reloading from file")
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
        print(f"[GRAPH] comparison saved in "
              f"{compare_methods_png(a.compare_methods, zoom=tuple(a.zoom) if a.zoom else (-2.0, 2.0, -2.0, 2.0), seed=a.seed, spacing_m=a.spacing, rings=rings, **common)[0]}")
        return
    if a.compare:
        compare(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                sampling='lattice',
                max_edge_length=a.max_edge, connection_radius=a.radius,
                border_margin_m=a.margin)
        return
    if a.timings:
        timing(None if a.timings is True else a.timings, spacing_m=a.spacing, rings=rings, **common)
        return

    mg = build_mission_graph(spacing_m=a.spacing, rings=rings, **common)
    if a.preview:
        print(f"[GRAPH] drawing saved in "
              f"{mg.preview_png(a.preview, zoom=tuple(a.zoom) if a.zoom else None)}")
    if a.save:
        print(f"[GRAPH] lattice saved in {mg.save_npz(a.save)}")


def _choice(value, mission):
    return mission if value is None else value


def main():
    rows = _choice(ROWS, P.MISSION_GRID_ROWS)
    cols = _choice(COLS, P.MISSION_GRID_COLS)
    cell = _choice(CELL_SIZE_M, P.MISSION_CELL_SIZE_M)
    e_min = _choice(EDGE_MIN_M, P.PRM_MIN_EDGE_M)
    e_max = _choice(EDGE_MAX_M, P.PRM_MAX_EDGE_M)
    spacing = _choice(LATTICE_SPACING_M, P.MISSION_LATTICE_SPACING_M)
    rings = _choice(RINGS, P.MISSION_LATTICE_RINGS)
    density = _choice(NODE_DENSITY_M2, P.MISSION_RANDOM_DENSITY)
    seed = _choice(SEED, P.MISSION_RANDOM_SEED)
    if seed is None:
        seed = draw_seed()
    print(f"Area {rows} x {cols} cells of {cell:g} m, edges {e_min:g}-{e_max:g} m, "
          f"lattice spacing {spacing}, rings {rings if rings is not None else 'all'}, "
          f"density {density}, seed {seed}"
          + ("" if all(v is None for v in (ROWS, COLS, CELL_SIZE_M, EDGE_MIN_M, EDGE_MAX_M,
                                           LATTICE_SPACING_M, RINGS, NODE_DENSITY_M2))
             else "  (some values from the TEST block, not from config.py)"))

    path, graphs = compare_methods_png(
        IMAGE, zoom=ZOOM, seed=seed, methods=METHODS, verbose=True,
        rows=rows, cols=cols, cell_size_m=cell,
        spacing_m=spacing, rings=rings,
        min_edge_length=e_min, max_edge_length=e_max,
        density_per_m2=None if density == 'match_lattice' else density)
    # Nodes per cell, with nodes on the borders counted as half (see nodes_per_cell).
    print("\nNodes per cell (rows from the top, as in the image; the nodes on the boundaries between "
          "cells count as half):")
    for g in graphs:
        name = g.distribution if g.is_random else 'lattice'
        counts = nodes_per_cell(g)
        print(f"  {name:11s} total {g.n_nodes:5d}, per cell {counts.min():.0f}-{counts.max():.0f}")
        for r in reversed(range(counts.shape[0])):
            print("              " + " ".join(f"{v:6.1f}" for v in counts[r]))

    # How to use each graph in a mission
    area = (rows, cols, cell) != (P.MISSION_GRID_ROWS, P.MISSION_GRID_COLS, P.MISSION_CELL_SIZE_M)
    edges = (e_min, e_max) != (P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M)
    print("\nTO USE ONE OF THESE GRAPHS IN A MISSION")
    if area or edges:
        print("  (area or edges different from config.py: those must be changed too)")
    for g in graphs:
        if g.is_random:
            d = g.describe()
            dens = d.get('node_density_m2', density)
            lines = (f"MISSION_GRAPH_SAMPLING = 'random'   MISSION_RANDOM_DISTRIBUTION = "
                     f"'{g.distribution}'   MISSION_RANDOM_DENSITY = {dens:g}   "
                     f"MISSION_RANDOM_SEED = {seed}")
            cmd = (f"--graph random --distribution {g.distribution} --density {dens:g} "
                   f"--seed {seed}")
            name = g.distribution
        else:
            lines = (f"MISSION_GRAPH_SAMPLING = 'lattice'   MISSION_LATTICE_SPACING_M = "
                     f"{spacing}   MISSION_LATTICE_RINGS = {rings}")
            cmd = f"--graph lattice --spacing {spacing}" + (f" --rings {rings}" if rings else "")
            name = 'lattice'
        extra = ""
        if area:
            extra += f" --rows {rows} --cols {cols} --cell {cell:g}"
        if edges:
            extra += f" --min-edge {e_min:g} --max-edge {e_max:g}"
        print(f"  {name}:")
        print(f"    in config.py:     {lines}")
        print(f"    or just once:     python3 run_mission.py {cmd}{extra}")
    if P.MISSION_RANDOM_SEED is None:
        print("  Careful: in config.py the seed is None, i.e. new at every mission. To "
              "get in a mission a random graph seen here you need its seed.")

    print(f"\nImage saved in {os.path.abspath(path)}")
    if OPEN:
        try:
            subprocess.Popen(['xdg-open', path], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError:
            print("Cannot open it from here: open it by hand.")


if __name__ == '__main__':
    cli() if len(sys.argv) > 1 else main()
