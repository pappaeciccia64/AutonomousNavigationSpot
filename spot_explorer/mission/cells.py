"""
cells.py -- the mission cell map.

A rows x cols grid of square cells of side cell_size. For each cell: 0 = not visited,
1 = visited, -1 = blocked (tried from all sides, without success), plus a 4-bit mask of
the sides it has already been tried from (North 0b1000, East 0b0100, South 0b0010,
West 0b0001).

The grid frame is fixed at power-on (set_origin): the centre of the starting cell is at
the origin, the grid x axis is the robot's forward direction at that moment (columns),
the y axis its left (rows). The visiting order is a serpentine.
"""
import numpy as np

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

    def is_cell_blocked(self, row, col):
        """True if the cell is blocked (-1)."""
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == -1
        return False

    def get_side_bit_facing_origin(self, origin_row, origin_col, target_row, target_col):
        """
        The robot went (or tried to go) from (origin_row, origin_col) towards (target_row,
        target_col). Returns the bit of TARGET's side facing origin, i.e. the side actually
        tried. 0 if the cells are not adjacent.
        """
        dr, dc = target_row - origin_row, target_col - origin_col
        if dr == -1 and dc == 0:
            return 0b0010  # target north of origin: entered from the SOUTH side
        if dr == 1 and dc == 0:
            return 0b1000  # target south: entered from the NORTH
        if dr == 0 and dc == 1:
            return 0b0001  # target east: entered from the WEST
        if dr == 0 and dc == -1:
            return 0b0100  # target west: entered from the EAST
        return 0b0000

    def get_valid_neighbor_sides_mask(self, row, col):
        """
        Mask of the sides that really have a neighbouring cell inside the grid (border and
        corner cells have fewer than 4): all the sides it could ever be tried from.
        Used by all_sides_explored to know when a cell has run out of options.
        North 0b1000, East 0b0100, South 0b0010, West 0b0001.
        """
        mask = 0b0000
        if row - 1 >= 0:
            mask |= 0b1000  # north
        if row + 1 < self.rows:
            mask |= 0b0010  # south
        if col + 1 < self.cols:
            mask |= 0b0100  # east
        if col - 1 >= 0:
            mask |= 0b0001  # west
        return mask

    def all_sides_explored(self, row, col):
        """
        True only if entry has already been tried from EVERY side that has a neighbour. Only
        then is the cell marked blocked (-1); before that it stays at 0, so exploration
        proposes it again later from a side not yet tried instead of excluding it at the
        first failure.
        """
        return (self.get_cell_sides_status(row, col) & self.get_valid_neighbor_sides_mask(row, col)) == \
               self.get_valid_neighbor_sides_mask(row, col)

    def mark_cell_side_explored(self, row, col, side_bit):
        """
        Record that entry into the cell was tried from side side_bit
        (North 0b1000, East 0b0100, South 0b0010, West 0b0001).
        """
        cell_key = (row, col)
        current = self.explored_sides.get(cell_key, 0b0000)
        self.explored_sides[cell_key] = current | side_bit
        print(f"[SIDE] Cell ({row},{col}): side {bin(side_bit)} marked as tried "
              f"(now {bin(self.explored_sides[cell_key])})")

    def get_adjacent_frontier_cells(self, cell_row, cell_col, path):
        """
        The adjacent cells (north, south, east, west) not yet visited (value 0), except
        those already tried from this same side.

        path: the serpentine, list of (row, column); a cell's rank is its position in path.

        Returns [(row, column, rank), ...] in increasing rank order, [] if there are none.
        """
        if path is None:
            print(f"[ERROR] get_adjacent_frontier_cells needs the path parameter")
            return []

        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        unexplored_neighbors = []
        neighbors = [
            (-1, 0),
            (1, 0),
            (0, 1),
            (0, -1)
        ]

        for dr, dc in neighbors:
            neighbor_row = cell_row + dr
            neighbor_col = cell_col + dc

            if not (0 <= neighbor_row < self.rows and 0 <= neighbor_col < self.cols):
                continue

            if self.map[neighbor_row][neighbor_col] == 0:
                # The cell may still be at 0 but already tried from THIS side without
                # success (the other sides remain to be tried). The same attempt is not
                # proposed again right away: it will come back into play from another side.
                side_bit = self.get_side_bit_facing_origin(cell_row, cell_col, neighbor_row, neighbor_col)
                if self.get_cell_sides_status(neighbor_row, neighbor_col) & side_bit:
                    continue

                neighbor_cell = (neighbor_row, neighbor_col)
                rank = cell_to_rank.get(neighbor_cell, float('inf'))
                unexplored_neighbors.append((neighbor_row, neighbor_col, rank))

        unexplored_neighbors.sort(key=lambda x: x[2])
        return unexplored_neighbors

    def get_lowest_rank_from_frontier_list(self, frontier_list, path):
        """
        Among the cells of frontier_list ((row, column) or (row, column, rank)), the one
        with the lowest rank in the serpentine path. Returns (row, column, rank), None if
        the list is empty or path is missing.
        """
        if path is None:
            print(f"[ERROR] get_lowest_rank_from_frontier_list needs the path parameter")
            return None

        if not frontier_list or len(frontier_list) == 0:
            print(f"[FRONTIER_RANK] Frontier empty")
            return None

        print(f"\n[FRONTIER_RANK] Looking for the lowest-rank cell among {len(frontier_list)} frontier cells")

        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        min_rank = float('inf')
        best_cell = None

        for cell in frontier_list:
            # Any rank already present is ignored: it is re-read from path.
            if isinstance(cell, tuple):
                if len(cell) >= 2:
                    row, col = cell[0], cell[1]
                    cell_tuple = (row, col)

                    rank = cell_to_rank.get(cell_tuple, float('inf'))

                    print(f"  Cell ({row},{col}): rank={rank}")

                    if rank < min_rank:
                        min_rank = rank
                        best_cell = (row, col, rank)

        if best_cell:
            print(f"[FRONTIER_RANK] ✓ Lowest-rank cell: ({best_cell[0]},{best_cell[1]}) rank={best_cell[2]}")
            return best_cell
        else:
            print(f"[FRONTIER_RANK] ⚠️ No valid cell in the list")
            return None

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
        The serpentine visiting order, list of (row, column):
            1  2  3  4  5
            10 9  8  7  6
            11 12 13 14 15
        rotated so as to start from start_cell (default: self.start_cell).
        """
        path = []
        for row in range(self.rows):
            if row % 2 == 0:
                # even rows: left to right
                for col in range(self.cols):
                    path.append((row, col))
            else:
                # odd rows: right to left
                for col in range(self.cols - 1, -1, -1):
                    path.append((row, col))

        if not path:
            return path

        effective_start = start_cell if start_cell is not None else self.start_cell
        if effective_start in path:
            start_idx = path.index(effective_start)
            if start_idx > 0:
                path = path[start_idx:] + path[:start_idx]

        return path

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
# FUNCTIONS ON CELLS (they also accept a fake grid with the same interface)
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


def find_new_borders(env, robot_row, robot_col, path, frontier):
    new_borders = env.get_adjacent_frontier_cells(robot_row, robot_col, path)
    new_borders_cells = []
    if len(new_borders) != 0:
        for new_border in new_borders:
            if new_border not in frontier and env.is_cell_visited(new_border[0], new_border[1]) != 1:
                new_borders_cells.append(new_border)
    return new_borders_cells
