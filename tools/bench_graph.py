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
"""
import os
import subprocess

import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # the spot_explorer package

from spot_explorer.planning import mission_graph
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
        seed = mission_graph.draw_seed()
    print(f"Area {rows} x {cols} cells of {cell:g} m, edges {e_min:g}-{e_max:g} m, "
          f"lattice spacing {spacing}, rings {rings if rings is not None else 'all'}, "
          f"density {density}, seed {seed}"
          + ("" if all(v is None for v in (ROWS, COLS, CELL_SIZE_M, EDGE_MIN_M, EDGE_MAX_M,
                                           LATTICE_SPACING_M, RINGS, NODE_DENSITY_M2))
             else "  (some values from the TEST block, not from config.py)"))

    path, graphs = mission_graph.compare_methods_png(
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
        counts = mission_graph.nodes_per_cell(g)
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
    main()
