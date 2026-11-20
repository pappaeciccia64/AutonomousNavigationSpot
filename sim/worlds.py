"""
Ready-made KinematicWorld scenes for tools/run_sim_mission.py. Each gives the world and the
mission grid that fits it (rows, columns, cell side). The robot starts in the centre of cell
(0, 0) looking along +x: the columns go along +x, the rows along +y (to the left).
"""
import numpy as np

from sim.kinematic_world import KinematicWorld


def _boxes(seed, w, h, n, keep_free=2.2):
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        sx, sy = rng.uniform(0.4, 1.2, 2)
        x0, y0 = rng.uniform(0, w - sx), rng.uniform(0, h - sy)
        if x0 < keep_free and y0 < keep_free:
            continue
        out.append((x0, x0 + sx, y0, y0 + sy, float(rng.uniform(0.4, 1.5))))
    return out


def scene(name, seed=1):
    """(KinematicWorld, rows, cols, cell) of a scene."""
    cell = 2.0
    if name == 'open':
        rows, cols = 4, 5
        return KinematicWorld(boxes=_boxes(seed, cols * cell, rows * cell, 8), start=(1.0, 1.0, 0.0)), rows, cols, cell
    if name == 'hill':                       # a hill of ~1.2 m with bushes, as the field of 28-10
        rows, cols = 6, 7
        return KinematicWorld(boxes=_boxes(seed, cols * cell, rows * cell, 10),
                              ground=[('hill', 7.0, 6.0, 1.2, 2.5)], start=(1.0, 1.0, 0.0)), rows, cols, cell
    if name == 'slope':                      # a lawn uphill along +y, ~13 degrees, as 29-10
        rows, cols = 5, 6
        return KinematicWorld(boxes=_boxes(seed, cols * cell, rows * cell, 5),
                              ground=[('plane', 0.0, np.tan(np.radians(13.0)))], start=(1.0, 1.0, 0.0)), rows, cols, cell
    if name == 'step':                       # a 0.12 m step across the area
        rows, cols = 3, 5
        return KinematicWorld(boxes=_boxes(seed, cols * cell, rows * cell, 4),
                              ground=[('step', 5.0, 0.12)], start=(1.0, 1.0, 0.0)), rows, cols, cell
    if name == 'walls':                      # a corridor with blocks on its left, as the room of 03-11
        rows, cols = 3, 5
        blocks = [(2.2, 3.6, 2.1, 3.2, 0.8), (5.0, 6.4, 2.1, 3.4, 0.8), (7.8, 9.2, 2.1, 3.2, 0.8)]
        walls = [(-0.2, 10.2, -0.3, -0.1, 1.0), (-0.2, 10.2, 5.9, 6.1, 1.0), (10.1, 10.3, -0.3, 6.1, 1.0)]
        return KinematicWorld(boxes=blocks + walls, start=(1.0, 1.0, 0.0)), rows, cols, cell
    raise ValueError(f"unknown scene {name}; known: {', '.join(SCENES)}")


SCENES = ('open', 'hill', 'slope', 'step', 'walls')
