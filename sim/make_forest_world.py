#!/usr/bin/env python3
"""
Make a forest world file from the downloaded assets (sim/assets/forest/catalog.json, written
by sim/fetch_forest_assets.py). No Isaac Sim needed: numpy only.

    python3 sim/make_forest_world.py                         # sim/worlds/woods.json, seed 7
    python3 sim/make_forest_world.py --seed 3 --name thicket --tree-gap 1.2 --out sim/worlds/thicket.json

Every tree, shrub and boulder is an obstacle: a cylinder as wide as what the asset has below
the robot's height (for a tree, the trunk), so the grid the mission reads and the forest seen
in Isaac Sim agree. Grass and fallen leaves are only looks: the robot walks through them.

The robot starts at the origin facing +x, in a clearing; x is ahead and y to the left.
"""
import argparse
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# asset: (scale from, scale to, share among the trees). The library trees are park-sized:
# the scales bring the trunks to those of a wood (10-40 cm of radius). Only trees whose
# crown stays above the robot: the obstacle is then the trunk alone, and what the lidar and
# the cameras see at the robot's height is what the grid says. (Red_Ash, Red_Oak and
# Shumard_Oak are in the library too, but some of their branches hang down to the ground.)
TREES = {
    "Yellow_Pine": (0.45, 0.70, 3.0), "Scarlet_Oak": (0.60, 1.00, 2.5), "Sugar_Maple": (0.70, 1.00, 2.0),
    "White_Ash": (1.00, 1.40, 2.5), "Sycamore": (0.35, 0.50, 1.2), "Elm_Sapling": (1.40, 1.80, 1.0),
    "Douglas_Fir": (0.55, 0.80, 0.6),
}
CROWN_CLEARANCE_M = 1.2                 # the foliage must start at least this high
# Tall grass is here too: Spot's cameras see it as an obstacle, and so does the simulated grid.
SHRUBS = {"Barberry": (0.8, 1.2), "Rhododendron": (0.5, 0.8), "White_Pine": (0.5, 0.8),
          "Chinese_Juniper": (0.7, 1.0), "Switchgrass": (0.45, 0.7), "Lupin": (0.5, 0.8)}
ROCK_SCALE = (2.5, 4.5)                 # the library rocks are stones: these make boulders
GRASS = ("Grass_Short_A", "Grass_Short_B", "Grass_Short_C")      # ankle-high: walked through
LEAVES = ("Leaves_Oak", "Leaves_Maple", "Leaves_Cluster")
MIN_OBSTACLE_R_M = 0.05                 # a sapling thinner than this is still seen as an obstacle


def main():
    p = argparse.ArgumentParser(description="Make a forest world for the simulator")
    p.add_argument("--catalog", default=os.path.join(HERE, "assets", "forest", "catalog.json"))
    p.add_argument("--out", default=os.path.join(HERE, "worlds", "woods.json"))
    p.add_argument("--name", default="woods")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--area", type=float, nargs=4, default=(-8.0, 18.0, -9.0, 15.0),
                   metavar=("X0", "X1", "Y0", "Y1"), help="where things are planted, m")
    p.add_argument("--trees", type=int, default=42)
    p.add_argument("--shrubs", type=int, default=20)
    p.add_argument("--rocks", type=int, default=10)
    p.add_argument("--logs", type=int, default=2)
    p.add_argument("--grass", type=int, default=320)
    p.add_argument("--leaves", type=int, default=120)
    p.add_argument("--tree-gap", type=float, default=1.5,
                   help="free space between two obstacles, m (the robot needs about 1 m)")
    p.add_argument("--clearing", type=float, default=1.8, help="free radius around the start, m")
    a = p.parse_args()

    if not os.path.exists(a.catalog):
        raise SystemExit("No catalog in %s: download the assets first (sim/run_isaac.sh --fetch-forest)."
                         % a.catalog)
    with open(a.catalog) as f:
        assets = json.load(f)["assets"]
    rng = np.random.RandomState(a.seed)
    x0, x1, y0, y1 = a.area
    placed = []                         # (x, y, r) of the obstacles so far

    def free(x, y, r, gap):
        if math.hypot(x, y) < a.clearing + r:
            return False
        return all(math.hypot(x - px, y - py) >= r + pr + gap for px, py, pr in placed)

    def drop(r, gap, tries=400):
        for _ in range(tries):
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if free(x, y, r, gap):
                placed.append((x, y, r))
                return round(x, 2), round(y, 2)
        return None

    shapes, counts = [], {}

    def plant(name, scale, r, h):
        at = drop(r, a.tree_gap)
        if at is None:
            return
        shapes.append({"type": "cylinder", "x": at[0], "y": at[1], "r": round(r, 3), "h": round(h, 2),
                       "asset": name, "scale": round(scale, 3), "spin_deg": int(rng.uniform(0, 360))})
        counts[assets[name]["kind"]] = counts.get(assets[name]["kind"], 0) + 1

    for _ in range(a.logs):             # fallen trunks first, they need the most room: boxes, drawn as lying logs
        length, thick = rng.uniform(2.0, 3.2), rng.uniform(0.22, 0.34)
        at = drop(0.5 * length, a.tree_gap)
        if at is not None:
            shapes.append({"type": "box", "x": at[0], "y": at[1], "sx": round(length, 2), "sy": round(thick, 2),
                           "yaw_deg": int(rng.uniform(0, 180)), "h": round(thick, 2), "look": "log"})
            counts["log"] = counts.get("log", 0) + 1

    names = [n for n in TREES if n in assets]
    if names:
        share = np.array([TREES[n][2] for n in names], dtype=float)
        for n in rng.choice(names, size=a.trees, p=share / share.sum()):
            s = rng.uniform(TREES[n][0], TREES[n][1])
            # A tree whose foliage comes down to the robot (a fir) is an obstacle as wide as
            # its lowest branches, not as its trunk.
            high_crown = assets[n].get("foliage_low_m", 0.0) * s >= CROWN_CLEARANCE_M or not assets[n].get("foliage_low_m")
            r = assets[n]["obstacle_r_m" if high_crown else "low_r_m"] * s
            plant(n, s, max(r, MIN_OBSTACLE_R_M), min(assets[n]["height_m"] * s, 3.0))
    names = [n for n in SHRUBS if n in assets]
    for n in (rng.choice(names, size=a.shrubs) if names else ()):
        s = rng.uniform(*SHRUBS[n])
        plant(n, s, assets[n]["obstacle_r_m"] * s, assets[n]["height_m"] * s)
    names = [n for n in assets if assets[n]["kind"] == "rock"]
    for n in (rng.choice(names, size=a.rocks) if names else ()):
        s = rng.uniform(*ROCK_SCALE)
        plant(n, s, assets[n]["obstacle_r_m"] * s, assets[n]["height_m"] * s)
    decor = []
    for group, n_items, scales in ((GRASS, a.grass, (0.8, 1.8)), (LEAVES, a.leaves, (2.0, 5.0))):
        names = [n for n in group if n in assets]
        for n in (rng.choice(names, size=n_items) if names else ()):
            decor.append({"asset": n, "x": round(rng.uniform(x0, x1), 2), "y": round(rng.uniform(y0, y1), 2),
                          "scale": round(rng.uniform(*scales), 2), "spin_deg": int(rng.uniform(0, 360))})

    world = {"name": a.name, "ground_z": 0.0,
             "ground": {"style": "forest", "size_m": round(2.0 * max(abs(v) for v in a.area) + 40.0, 1)},
             "made_by": "sim/make_forest_world.py --seed %d" % a.seed,
             "shapes": shapes, "decor": decor}
    with open(a.out, "w") as f:
        f.write('{"name": %s, "ground_z": 0.0,\n "ground": %s,\n "made_by": %s,\n "shapes": [\n  '
                % (json.dumps(world["name"]), json.dumps(world["ground"]), json.dumps(world["made_by"])))
        f.write(",\n  ".join(json.dumps(s) for s in shapes))
        f.write('\n ],\n "decor": [\n  ')
        f.write(",\n  ".join(json.dumps(d) for d in decor))
        f.write("\n ]}\n")
    print("World '%s' written to %s: %s, %d grass and leaves; area x %.0f..%.0f, y %.0f..%.0f m."
          % (a.name, a.out, ", ".join("%d %ss" % (v, k) for k, v in sorted(counts.items())), len(decor),
             x0, x1, y0, y1))


if __name__ == "__main__":
    main()
