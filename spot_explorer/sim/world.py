"""
The simulated world: a ground (flat or with hills), obstacles on it, the local grid Spot
would report and the rays of a LiDAR.

The world is described in the simulator's world frame, which the mission sees as the VISION
frame: the robot starts at the origin facing +x, so x is AHEAD and y is to the LEFT (the
same convention as the mission cells: columns ahead, rows to the left).

    {"name": "example", "ground_z": 0.0,
     "terrain": {"max_slope_deg": 8, "wavelength_m": 9, "seed": 3,
                 "ramp": {"dir_deg": 0, "grade_pct": 6},
                 "bumps": [{"x": 6, "y": 2, "h": 0.5, "r": 2.0}]},
     "shapes": [{"type": "box", "x": 3.0, "y": 0.5, "sx": 1.0, "sy": 0.4, "yaw_deg": 20, "h": 0.8},
                {"type": "box", "x0": 5, "x1": 6, "y0": -1, "y1": 1},
                {"type": "cylinder", "x": 2.0, "y": -1.5, "r": 0.15, "h": 2.0}]}

The same file is loaded by the simulator (which builds ground and solids) and served to the
mission over the link, so there is one source of truth.

TERRAIN. The height is an analytic function (height()), so the simulator's mesh, the local
grid and the LiDAR all see exactly the same ground:
  hills   a sum of sine waves in random directions (seed), scaled so that their slope never
          exceeds max_slope_deg; wavelength_m is the distance between crests;
  ramp    a constant grade in one direction (dir_deg: 0 = uphill ahead, 90 = uphill to the left);
  bumps   single mounds (h > 0) or hollows (h < 0) of radius r; their slope (about 1.2 h / r
          at the steepest) adds to the hills'.
Around the origin the ground is flat at ground_z (flat_radius_m), so the robot always starts
level; hills and ramp grow in gradually from there. Shapes stand on the ground under their centre.

WHAT IS SYNTHETIC. The real robot builds obstacle_distance, terrain and terrain_valid from
its cameras. Here they are computed from the world: obstacle_distance is the exact signed
distance from the nearest shape (negative inside), terrain is height() with a few millimetres
of noise, every cell is valid. So: no occlusions, no ghost cells, and a slope is never an
"obstacle" by itself (the mission's own slope checks see it in the terrain layer).
Numpy only: this module is also imported by the simulators.
"""
import json
import numpy as np

# Spot's local grid: 128 x 128 cells of 3 cm, centred on the robot.
GRID_CELLS = 128
GRID_CELL_M = 0.03
# Value far from every obstacle (as in tests/test_e2e_loop.py).
FAR_M = 9.0
# Hills: relative wavelength and weight of each sine wave.
_WAVES = ((1.0, 0.45), (0.62, 0.30), (0.37, 0.25))
FLAT_RADIUS_M = 1.2


# How a shape LOOKS in Isaac Sim (the geometry above is what counts for the mission): the
# library asset drawn in its place, its scale and rotation, or a named look ("log").
LOOK_KEYS = ('asset', 'scale', 'spin_deg', 'look')


def _shape(d):
    """Normalise one shape dict: every box has centre, sizes and yaw; every cylinder a radius."""
    kind = str(d.get('type', 'box')).lower()
    h = float(d.get('h', 1.0))
    look = {k: d[k] for k in LOOK_KEYS if k in d}
    if kind in ('cylinder', 'cyl', 'tree'):
        return dict(look, type='cylinder', x=float(d['x']), y=float(d['y']), r=float(d['r']), h=h)
    if kind != 'box':
        raise ValueError("unknown shape type %r (box or cylinder)" % (kind,))
    if 'x0' in d:
        x0, x1 = sorted((float(d['x0']), float(d['x1'])))
        y0, y1 = sorted((float(d['y0']), float(d['y1'])))
        return dict(look, type='box', x=0.5 * (x0 + x1), y=0.5 * (y0 + y1),
                    sx=x1 - x0, sy=y1 - y0, yaw_deg=0.0, h=h)
    return dict(look, type='box', x=float(d['x']), y=float(d['y']), sx=float(d['sx']),
                sy=float(d['sy']), yaw_deg=float(d.get('yaw_deg', 0.0)), h=h)


def _smoothstep(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


class World:
    def __init__(self, shapes=(), ground_z=0.0, name='', terrain_noise_m=0.004,
                 battery=None, terrain=None, ground=None, decor=()):
        self.shapes = [_shape(s) for s in shapes]
        self.ground_z = float(ground_z)
        self.name = str(name)
        self.terrain_noise_m = float(terrain_noise_m)
        # {'start_pct': 100, 'drain_pct_per_min': 0}: to try the early return on low battery.
        self.battery = dict(battery or {})
        self.terrain = dict(terrain or {})
        # Looks only, used by Isaac Sim: the ground style ({'style': 'forest', 'size_m': 90}) and
        # what the robot walks through (grass, leaves): [{'asset', 'x', 'y', 'scale', 'spin_deg'}].
        self.ground = dict(ground or {})
        self.decor = [dict(d) for d in decor]
        self._setup_terrain()

    # --- files ---------------------------------------------------------------------------
    @classmethod
    def from_dict(cls, d):
        d = d or {}
        return cls(shapes=d.get('shapes', ()), ground_z=d.get('ground_z', 0.0),
                   name=d.get('name', ''), terrain_noise_m=d.get('terrain_noise_m', 0.004),
                   battery=d.get('battery'), terrain=d.get('terrain'), ground=d.get('ground'),
                   decor=d.get('decor', ()))

    @classmethod
    def from_file(cls, path):
        with open(path, 'r') as f:
            return cls.from_dict(json.load(f))

    def to_dict(self, looks=True):
        """looks=False leaves out the decor, which only Isaac Sim needs (the mission asks for
        the world over the link)."""
        d = {'name': self.name, 'ground_z': self.ground_z,
             'terrain_noise_m': self.terrain_noise_m, 'battery': self.battery,
             'terrain': self.terrain, 'ground': self.ground, 'shapes': [dict(s) for s in self.shapes]}
        if looks:
            d['decor'] = [dict(x) for x in self.decor]
        return d

    # --- terrain -------------------------------------------------------------------------
    def _setup_terrain(self):
        t = self.terrain
        self.is_flat = not t
        self._flat_r = float(t.get('flat_radius_m', FLAT_RADIUS_M))
        self._waves = []
        slope = np.tan(np.radians(float(t.get('max_slope_deg', 0.0))))
        wavelength = float(t.get('wavelength_m', 8.0))
        # Hills and bumps grow in over one wavelength from the flat disc at the start.
        self._blend = max(wavelength, 3.0)
        if slope > 0:
            rng = np.random.RandomState(int(t.get('seed', 0)))
            for rel, weight in _WAVES:
                k = 2.0 * np.pi / (wavelength * rel)
                direction = rng.uniform(0.0, 2.0 * np.pi)
                # amplitude * k = this wave's share of the slope: the shares add up to `slope`
                self._waves.append((weight * slope / k, k * np.cos(direction),
                                    k * np.sin(direction), rng.uniform(0.0, 2.0 * np.pi)))
        ramp = t.get('ramp') or {}
        self._ramp_grade = float(ramp.get('grade_pct', 0.0)) / 100.0
        a = np.radians(float(ramp.get('dir_deg', 0.0)))
        self._ramp_dir = (float(np.cos(a)), float(np.sin(a)))
        self._bumps = [(float(b['x']), float(b['y']), float(b['h']), float(b['r']))
                       for b in t.get('bumps', ())]

    def height(self, x, y):
        """Ground height (m, world frame) at the points (x, y): arrays of the same shape."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        if self.is_flat:
            return np.full(np.broadcast(x, y).shape, self.ground_z, dtype=np.float64)
        h = np.zeros(np.broadcast(x, y).shape, dtype=np.float64)
        for amp, kx, ky, phase in self._waves:
            # minus the value at the origin: the blend then starts from zero height
            h += amp * (np.sin(kx * x + ky * y + phase) - np.sin(phase))
        for bx, by, bh, br in self._bumps:
            h += bh * np.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2.0 * (0.5 * br) ** 2))
        h *= _smoothstep((np.hypot(x, y) - self._flat_r) / self._blend)
        if self._ramp_grade:
            # Zero within the flat radius, then the grade, joined smoothly: never steeper.
            u = self._ramp_dir[0] * x + self._ramp_dir[1] * y
            v = np.abs(u) - (self._flat_r + 0.5)
            h += self._ramp_grade * np.sign(u) * 0.5 * (np.sqrt(v * v + 0.04) + v)
        return h + self.ground_z

    def height_at(self, x, y):
        return float(self.height(np.array([x]), np.array([y]))[0])

    def gradient_at(self, x, y, eps=0.05):
        """(dh/dx, dh/dy) at one point."""
        h = self.height(np.array([x + eps, x - eps, x, x]), np.array([y, y, y + eps, y - eps]))
        return float(h[0] - h[1]) / (2 * eps), float(h[2] - h[3]) / (2 * eps)

    # --- obstacles -----------------------------------------------------------------------
    def signed_distance(self, x, y):
        """Signed distance (m) from the nearest shape at the points (x, y): negative inside.
        FAR_M where there is nothing within reach. x and y: arrays of the same shape."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        out = np.full(x.shape, FAR_M, dtype=np.float64)
        for s in self.shapes:
            px, py = x - s['x'], y - s['y']
            if s['type'] == 'cylinder':
                d = np.hypot(px, py) - s['r']
            else:
                a = np.radians(s['yaw_deg'])
                c, sn = np.cos(a), np.sin(a)
                qx = np.abs(c * px + sn * py) - 0.5 * s['sx']     # in the box frame
                qy = np.abs(-sn * px + c * py) - 0.5 * s['sy']
                d = (np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
                     + np.minimum(np.maximum(qx, qy), 0.0))
            np.minimum(out, d, out=out)
        return out

    def clearance(self, x, y):
        """Signed distance at one point (float)."""
        return float(self.signed_distance(np.array([x]), np.array([y]))[0])

    def shape_base_z(self, s):
        """Height of the ground under the centre of a shape: where the solid stands."""
        return self.height_at(s['x'], s['y'])

    # --- local grid ----------------------------------------------------------------------
    def local_grid(self, robot_x, robot_y, n=GRID_CELLS, cell=GRID_CELL_M):
        """
        The window of n x n cells around the robot, as Spot's local grid.

        Returns a dict: origin (x, y) of the corner of cell [0, 0] in the world frame, snapped
        to whole cells like the real grid; obstacle_distance, terrain (height in the world
        frame) and terrain_valid as (n, n) arrays indexed [row = y, column = x], the layout
        spot_grid.LocalGrid expects (cell [r, c] is centred at origin + (c + 0.5, r + 0.5) * cell).
        """
        c0 = int(np.floor(robot_x / cell)) - n // 2
        r0 = int(np.floor(robot_y / cell)) - n // 2
        ox, oy = c0 * cell, r0 * cell
        xs = ox + (np.arange(n) + 0.5) * cell
        ys = oy + (np.arange(n) + 0.5) * cell
        gx, gy = np.meshgrid(xs, ys)
        od = self.signed_distance(gx, gy)
        terrain = self.height(gx, gy)
        if self.terrain_noise_m > 0:
            # Same noise for the same window: two requests in a row give aligned layers.
            seed = (c0 * 73856093 ^ r0 * 19349663) & 0x7FFFFFFF
            terrain = terrain + np.random.RandomState(seed).normal(0.0, self.terrain_noise_m, (n, n))
        return {'origin': (ox, oy), 'cell': float(cell), 'n': int(n),
                'obstacle_distance': od, 'terrain': terrain,
                'terrain_valid': np.ones((n, n), dtype=np.float64)}

    # --- rays (LiDAR) --------------------------------------------------------------------
    def raycast(self, origin, directions, max_range=30.0, march_m=0.25):
        """
        Distance along each ray to the first thing it hits: ground, box or cylinder.

        origin: (3,) world point. directions: (N, 3) unit vectors, world frame.
        Returns (N,) distances; np.inf where nothing is hit within max_range.
        """
        o = np.asarray(origin, dtype=np.float64)
        d = np.asarray(directions, dtype=np.float64)
        n = d.shape[0]
        best = np.full(n, np.inf)

        # Ground: walk along the ray until it goes under, then bisect that step.
        if self.is_flat:
            with np.errstate(divide='ignore', invalid='ignore'):
                t = (self.ground_z - o[2]) / d[:, 2]
            ok = (d[:, 2] < 0) & (t > 0) & (t <= max_range)
            best[ok] = t[ok]
        else:
            steps = np.arange(march_m, max_range + march_m, march_m)
            alive = np.ones(n, dtype=bool)
            lo = np.zeros(n)
            hi = np.full(n, np.inf)
            for t in steps:
                idx = np.nonzero(alive)[0]
                if idx.size == 0:
                    break
                p = o + t * d[idx]
                under = p[:, 2] < self.height(p[:, 0], p[:, 1])
                hit = idx[under]
                hi[hit] = t
                lo[hit] = t - march_m
                alive[hit] = False
            found = np.isfinite(hi)
            a, b = lo[found], hi[found]
            for _ in range(7):
                m = 0.5 * (a + b)
                p = o + m[:, None] * d[found]
                under = p[:, 2] < self.height(p[:, 0], p[:, 1])
                b = np.where(under, m, b)
                a = np.where(under, a, m)
            best[found] = 0.5 * (a + b)

        for s in self.shapes:
            z0 = self.shape_base_z(s) - 0.3          # sunk a little, like the simulator's solids
            z1 = z0 + 0.3 + s['h']
            if s['type'] == 'cylinder':
                px, py = o[0] - s['x'], o[1] - s['y']
                a = d[:, 0] ** 2 + d[:, 1] ** 2
                b = 2.0 * (px * d[:, 0] + py * d[:, 1])
                c = px * px + py * py - s['r'] ** 2
                with np.errstate(divide='ignore', invalid='ignore'):
                    disc = b * b - 4.0 * a * c
                    t = (-b - np.sqrt(disc)) / (2.0 * a)
                z = o[2] + t * d[:, 2]
                ok = (disc >= 0) & (a > 1e-12) & (t > 0) & (z >= z0) & (z <= z1)
            else:
                yaw = np.radians(s['yaw_deg'])
                cs, sn = np.cos(yaw), np.sin(yaw)
                # ray in the box frame
                px, py = o[0] - s['x'], o[1] - s['y']
                lo3 = np.array([cs * px + sn * py, -sn * px + cs * py, o[2]])
                ld = np.stack([cs * d[:, 0] + sn * d[:, 1], -sn * d[:, 0] + cs * d[:, 1], d[:, 2]], axis=1)
                bmin = np.array([-0.5 * s['sx'], -0.5 * s['sy'], z0])
                bmax = np.array([0.5 * s['sx'], 0.5 * s['sy'], z1])
                with np.errstate(divide='ignore', invalid='ignore'):
                    t1 = (bmin - lo3) / ld
                    t2 = (bmax - lo3) / ld
                tnear = np.nanmax(np.minimum(t1, t2), axis=1)
                tfar = np.nanmin(np.maximum(t1, t2), axis=1)
                t = tnear
                ok = (tnear <= tfar) & (tnear > 0)
            ok &= t <= max_range
            best = np.where(ok & (t < best), t, best)
        return best
