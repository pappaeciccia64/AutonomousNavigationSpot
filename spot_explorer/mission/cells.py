"""
cells.py -- the mission cell map.

A rows x cols grid of square cells of side cell_size. For each cell: 0 = not visited,
1 = visited, -1 = blocked (tried from all sides, without success), plus a 4-bit mask of
the sides it has already been tried from (North 0b1000, East 0b0100, South 0b0010,
West 0b0001).

The grid frame is fixed at power-on (set_origin): the centre of the starting cell is at
the origin, the grid x axis is the robot's forward direction at that moment (columns),
the y axis its left (rows).

The starting cell is chosen (start_cell_for, config.MISSION_START_POSITION): the robot can
stand in one of the four corners, in the middle of a side or in the centre of the grid.
Looking at the grid from above with the robot's forward direction up:

    top-left       top-centre     top-right          ^ forward (growing columns)
    centre-left    centre         centre-right       |
    bottom-left    bottom-centre  bottom-right  <----+ left (growing rows)

'bottom-right' is cell (0, 0): the grid opens ahead and to the left of the robot (as it
always was). 'top-...' puts the grid BEHIND the robot, '...-right' to its left,
'...-left' to its right.

The visiting order (generate_visit_path) is a serpentine starting from the robot's corner,
or, if the robot is not in a corner, rings around the starting cell.
"""
import numpy as np


# =========================================================================================
# Starting cell and visiting order
# =========================================================================================
# Named positions of the starting cell: (vertical, horizontal) looking at the grid from
# above with the robot's forward direction up. Vertical -> column (bottom = column 0, the
# grid is ahead of the robot), horizontal -> row (right = row 0, the grid is to the left).
START_POSITIONS = (
    'bottom-right', 'bottom-left', 'top-right', 'top-left', 'centre',
    'bottom-centre', 'top-centre', 'centre-right', 'centre-left',
)
VISIT_ORDERS = ('auto', 'serpentine', 'rings')

_START_ALIASES = {'centre-centre': 'centre', 'center': 'centre'}


def _axis_index(where, n):
    """Index along one axis: 'bottom'/'right' = 0, 'top'/'left' = n-1, 'centre' = n//2."""
    if where in ('bottom', 'right'):
        return 0
    if where in ('top', 'left'):
        return n - 1
    return n // 2


def start_cell_for(position, rows, cols):
    """
    The starting cell (row, column) for a rows x cols grid.

    position: one of START_POSITIONS ('bottom-right', 'centre', ...; spaces and '_' count as
    '-', 'center' as 'centre'), or the cell itself, as a tuple (row, column) or a string
    "row,column". With an even number of rows or columns the centre is not a cell: 'centre'
    takes cell n//2 (the more to the left / more ahead of the two central ones).

    Raises ValueError if the position is not valid or the cell is outside the grid.
    """
    if isinstance(position, (tuple, list)):
        cell = (int(position[0]), int(position[1]))
    else:
        text = str(position).strip().lower().replace(' ', '-').replace('_', '-').replace('center', 'centre')
        text = _START_ALIASES.get(text, text)
        if ',' in text:
            try:
                r, c = text.split(',')
                cell = (int(r), int(c))
            except ValueError:
                raise ValueError(f"starting cell {position!r} not valid: expected 'row,column'")
        elif text == 'centre':
            cell = (rows // 2, cols // 2)
        elif text in START_POSITIONS:
            vert, horiz = text.split('-')
            cell = (_axis_index(horiz, rows), _axis_index(vert, cols))
        else:
            raise ValueError(f"starting position {position!r} unknown: one of "
                             f"{', '.join(START_POSITIONS)}, or 'row,column'")
    if not (0 <= cell[0] < rows and 0 <= cell[1] < cols):
        raise ValueError(f"starting cell {cell} outside the {rows} x {cols} grid")
    return cell


def describe_start_cell(cell, rows, cols):
    """Name of the position of the cell in the grid ('bottom-right', 'centre', ...) or ''."""
    for name in START_POSITIONS:
        if start_cell_for(name, rows, cols) == tuple(cell):
            return name
    return ''


def is_corner_cell(cell, rows, cols):
    """True if the cell is in a corner of the grid (also for one-row grids)."""
    r, c = cell
    return r in (0, rows - 1) and c in (0, cols - 1)


class EnvironmentMap(object):
    def __init__(self, rows=12, cols=12, cell_size=2.0):
        self.map = [[0 for _ in range(cols)] for _ in range(rows)]
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size  # cell side, m
        self.origin_x = 0.0  # grid origin in the world frame
        self.origin_y = 0.0
        self.origin_yaw = 0.0  # robot heading at power-on, rad
        self.start_cell = (0, 0)  # starting cell (row, column)
        self.current_cell = (0, 0)
        self.explored_sides = {}

    def mark_cell_visited(self, row, col):
        """Mark the cell as visited (1). False if it is outside the grid."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.map[row][col] = 1
            print(f"[VISITED] Cell ({row},{col}) marked as visited (value 1)")
            return True
        else:
            print(f"[ERROR] Cannot mark cell ({row},{col}) as visited: outside the grid")
            return False

    def is_cell_visited(self, row, col):
        """True if the cell is visited (1)."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == 1
        return False

    def is_point_in_cell(self, x, y, target_row, target_col):
        """
        True if the world point (x, y) lies inside the cell, borders included. Checks the
        real cell boundaries, rotated with the grid.
        """
        cell_center = self.get_world_position_from_cell(target_row, target_col)
        if cell_center is None:
            return False

        center_x, center_y = cell_center

        half_size = self.cell_size / 2.0

        # Point relative to the cell centre, rotated into the grid frame
        delta_x = x - center_x
        delta_y = y - center_y

        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        local_x = delta_x * cos_yaw - delta_y * sin_yaw
        local_y = delta_x * sin_yaw + delta_y * cos_yaw

        return abs(local_x) <= half_size and abs(local_y) <= half_size

    def mark_cell_blocked(self, row, col):
        """Mark the cell as blocked (-1): tried and not reachable."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.map[row][col] = -1
            print(f"[BLOCKED] Cell ({row},{col}) marked as blocked (value -1)")
        else:
            print(f"[ERROR] Cannot mark cell ({row},{col}) as blocked: outside the grid")

    def get_cell_sides_status(self, row, col):
        """4-bit mask of the sides already tried (from 0b0000 to 0b1111)."""
        cell_key = (row, col)
        return self.explored_sides.get(cell_key, 0b0000)

    def set_origin(self, x, y, yaw=0.0, start_row=0, start_col=0):
        """
        Fix the grid on the robot pose (x, y, yaw in the world frame): the centre of
        (start_row, start_col) falls on (x, y) and the grid is aligned with heading yaw.
        Marks the starting cell as visited.
        """
        self.origin_x = x
        self.origin_y = y
        self.origin_yaw = yaw
        self.start_cell = (start_row, start_col)

        self.map[start_row][start_col] = 1

        print(f"[OK] Origin set at world coordinates ({x:.2f}, {y:.2f})")
        print(f"[OK] Initial orientation: {np.rad2deg(yaw):.1f}°")
        print(f"[OK] Starting cell ({start_row}, {start_col}) marked as visited")

    def update_position(self, x, y):
        """
        Update the current cell from the robot position (x, y) and mark it visited if it
        was 0. Returns (row, column), or None if the robot centre is outside the grid.
        """
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y

        # In the grid frame: x = robot forward at power-on (columns),
        # y = left (rows).
        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        grid_x = delta_x * cos_yaw - delta_y * sin_yaw
        grid_y = delta_x * sin_yaw + delta_y * cos_yaw

        # The cell containing the robot centre, borders included
        half_size = self.cell_size / 2.0

        col = None
        row = None

        for c in range(self.cols):
            cell_center_x = (c - self.start_cell[1]) * self.cell_size
            if abs(grid_x - cell_center_x) <= half_size:
                col = c
                break

        for r in range(self.rows):
            cell_center_y = (r - self.start_cell[0]) * self.cell_size
            if abs(grid_y - cell_center_y) <= half_size:
                row = r
                break

        if row is not None and col is not None:
            if 0 <= row < self.rows and 0 <= col < self.cols:
                if self.map[row][col] == 0:
                    self.map[row][col] = 1

                self.current_cell = (row, col)
                return (row, col)

        print(f"[INFO] Robot centre at ({x:.2f}, {y:.2f}) -> grid ({grid_x:.2f}, {grid_y:.2f}): between two cells or outside the grid")
        return None

    def get_cell_from_world(self, x, y):
        """
        The cell (row, column) containing the world point (x, y), without marking it
        visited. Does not check the limits: outside the grid it returns out-of-range indices.
        """
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y

        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        grid_x = delta_x * cos_yaw - delta_y * sin_yaw
        grid_y = delta_x * sin_yaw + delta_y * cos_yaw

        # Nearest cell centre
        col = self.start_cell[1] + round(grid_x / self.cell_size)
        row = self.start_cell[0] + round(grid_y / self.cell_size)

        return row, col

    def get_cell_status(self, row, col):
        """
        (value, sides_tried): value 0 not visited, 1 visited, -1 blocked; sides_tried is
        the 4-bit mask. (None, None) outside the grid.
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            cell_value = self.map[row][col]
            explored_sides = self.get_cell_sides_status(row, col)
            return cell_value, explored_sides
        return None, None

    def generate_serpentine_path(self, start_cell=None):
        """
        The serpentine visiting order, list of (row, column). A row is walked along the
        columns (forwards or backwards), then the next row, coming back:
            1  2  3  4  5
            10 9  8  7  6
            11 12 13 14 15
        If start_cell (default: self.start_cell) is a corner, the serpentine starts from that
        corner: rows from the one of start_cell, columns from its side. With (0, 0) it is the
        usual serpentine. If it is not a corner, the serpentine from (0, 0) rotated so as to
        start from start_cell (better generate_visit_path('rings')).
        """
        effective_start = tuple(start_cell if start_cell is not None else self.start_cell)
        r0, c0 = effective_start
        corner = is_corner_cell(effective_start, self.rows, self.cols)
        row_order = range(self.rows - 1, -1, -1) if corner and r0 == self.rows - 1 and self.rows > 1 \
            else range(self.rows)
        forward = not (corner and c0 == self.cols - 1 and self.cols > 1)
        path = []
        for row in row_order:
            cols = range(self.cols) if forward else range(self.cols - 1, -1, -1)
            path.extend((row, col) for col in cols)
            forward = not forward

        if not path:
            return path

        if effective_start in path:
            start_idx = path.index(effective_start)
            if start_idx > 0:
                path = path[start_idx:] + path[:start_idx]

        return path

    def generate_ring_path(self, start_cell=None):
        """
        The visiting order in rings, list of (row, column): first the starting cell, then the
        ring of cells at distance 1 (the 8 around it), then the one at distance 2, ...
        (distance = max(|delta row|, |delta column|)). Each ring is walked from one cell to
        the bordering one, counter-clockwise, starting from the cell nearest to the last one
        of the ring before; if the grid border cuts the ring, it starts from one of its ends,
        so no piece is left behind. Suited to a start in the centre or mid-side.
        """
        start = tuple(start_cell if start_cell is not None else self.start_cell)
        r0, c0 = start
        rings = {}
        for r in range(self.rows):
            for c in range(self.cols):
                rings.setdefault(max(abs(r - r0), abs(c - c0)), []).append((r, c))

        n4 = ((-1, 0), (1, 0), (0, 1), (0, -1))

        def angle(cell):
            return float(np.arctan2(cell[0] - r0, cell[1] - c0)) % (2 * np.pi)

        def ccw(a, b):
            # > 0 if going from a to b turns counter-clockwise around the start
            return (a[1] - c0) * (b[0] - r0) - (a[0] - r0) * (b[1] - c0)

        path, cur = [], start
        for d in sorted(rings):
            todo = set(rings[d])
            while todo:
                def degree(cell):
                    return sum((cell[0] + dr, cell[1] + dc) in todo for dr, dc in n4)
                # Entry into the ring (or one piece of it): if the border cuts the ring, the
                # nearest end; if it is closed, the nearest cell.
                nxt = min(todo, key=lambda x: (degree(x) > 1,
                                               abs(x[0] - cur[0]) + abs(x[1] - cur[1]),
                                               angle(x)))
                while nxt is not None:
                    path.append(nxt)
                    todo.discard(nxt)
                    cur = nxt
                    near = [(cur[0] + dr, cur[1] + dc) for dr, dc in n4
                            if (cur[0] + dr, cur[1] + dc) in todo]
                    nxt = max(near, key=lambda x: (ccw(cur, x), -angle(x))) if near else None
        return path

    def visit_order_name(self, order='auto', start_cell=None):
        """The order actually used: 'auto' = 'serpentine' from a corner, 'rings' otherwise."""
        start = tuple(start_cell if start_cell is not None else self.start_cell)
        if order not in VISIT_ORDERS:
            raise ValueError(f"visiting order {order!r} unknown: one of {VISIT_ORDERS}")
        if order == 'auto':
            return 'serpentine' if is_corner_cell(start, self.rows, self.cols) else 'rings'
        return order

    def generate_visit_path(self, order='auto', start_cell=None):
        """
        The visiting order of the cells, list of (row, column), starting from the starting
        cell. order: 'serpentine' (generate_serpentine_path), 'rings' (generate_ring_path) or
        'auto' (see visit_order_name).
        """
        start = tuple(start_cell if start_cell is not None else self.start_cell)
        if self.visit_order_name(order, start) == 'serpentine':
            return self.generate_serpentine_path(start_cell=start)
        return self.generate_ring_path(start_cell=start)

    def get_world_position_from_cell(self, row, col):
        """
        The cell centre in the world frame, (x, y); None outside the grid. The centre of
        the starting cell is the origin, i.e. the robot pose at power-on.
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            delta_row = row - self.start_cell[0]
            delta_col = col - self.start_cell[1]

            # In the grid frame: x = columns, y = rows
            grid_x = delta_col * self.cell_size
            grid_y = delta_row * self.cell_size

            # In the world frame
            cos_yaw = np.cos(self.origin_yaw)
            sin_yaw = np.sin(self.origin_yaw)

            world_delta_x = grid_x * cos_yaw - grid_y * sin_yaw
            world_delta_y = grid_x * sin_yaw + grid_y * cos_yaw

            x = self.origin_x + world_delta_x
            y = self.origin_y + world_delta_y

            return (x, y)
        return None

    def print_map(self):
        """Print the cell map."""
        print("\n--- Cell map ---")
        for row in self.map:
            print(" ".join(str(cell) for cell in row))
        if hasattr(self, 'current_cell'):
            print(f"Current cell: {self.current_cell}")
        print("-----------------------\n")


# =========================================================================================
# Functions on cells (they also accept a fake grid with the same interface)
# =========================================================================================
def cell_border_distance(env, row, col, x, y):
    """
    Distance of (x, y) from the border of cell (row, col), in the rotated grid frame:
    positive inside, negative outside. None if the cell does not exist.
    """
    center = env.get_world_position_from_cell(row, col)
    if center is None:
        return None
    dx, dy = x - center[0], y - center[1]
    yaw = float(getattr(env, 'origin_yaw', 0.0))
    c, s = np.cos(-yaw), np.sin(-yaw)
    lx, ly = dx * c - dy * s, dx * s + dy * c
    return env.cell_size / 2.0 - max(abs(lx), abs(ly))


def cell_seen_fraction(env, global_map, row, col, step=0.05):
    """Share of cell (row, col) already observed in the global map (free or occupied,
    GlobalGrid.is_known), on a lattice of `step` in the rotated grid frame. None if the cell
    does not exist."""
    center = env.get_world_position_from_cell(row, col)
    if center is None:
        return None
    h = env.cell_size / 2.0
    u = np.arange(-h + step / 2.0, h, step)
    U, V = np.meshgrid(u, u)
    yaw = float(getattr(env, 'origin_yaw', 0.0))
    c, s = np.cos(yaw), np.sin(yaw)
    xs = center[0] + U * c - V * s
    ys = center[1] + U * s + V * c
    if hasattr(global_map, 'known_many'):
        return float(np.mean(global_map.known_many(xs, ys)))
    res = global_map.resolution
    keys = zip(np.floor(xs.ravel() / res).astype(int).tolist(), np.floor(ys.ravel() / res).astype(int).tolist())
    grid = global_map.grid
    return float(np.mean([k in grid for k in keys]))
