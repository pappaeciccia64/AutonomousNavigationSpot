import numpy as np

class EnvironmentMap(object):
    def __init__(self, rows=12, cols=12, cell_size=2.0):
        self.map = [[0 for _ in range(cols)] for _ in range(rows)]
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size  # Size of each cell in meters
        self.origin_x = 0.0  # World X coordinate of grid origin
        self.origin_y = 0.0  # World Y coordinate of grid origin
        self.origin_yaw = 0.0  # Initial yaw orientation of robot (radians)
        self.start_cell = (0, 0)  # Starting cell in grid coordinates
        self.current_cell = (0, 0)
        self.explored_sides = {}
        self.waypoints = []

    def mark_cell_visited(self, row, col):
        """
        Mark a cell as visited (accessible).
        Sets the map value to 1 to indicate this cell was successfully visited.

        Args:
            row: Row of the cell
            col: Column of the cell

        Returns:
            bool: True if cell was marked successfully, False if out of bounds
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.map[row][col] = 1
            print(f"[VISITED] Cell ({row},{col}) marked as visited (value=1)")
            return True
        else:
            print(f"[ERROR] Cannot mark cell ({row},{col}) as visited - out of bounds")
            return False

    def is_cell_visited(self, row, col):
        """
        Check if a cell is marked as visited.

        Args:
            row: Row of the cell
            col: Column of the cell

        Returns:
            bool: True if cell is visited (value=1), False otherwise
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == 1
        return False

    def is_point_in_cell(self, x, y, target_row, target_col):
        """
        Check if a point in world coordinates is inside a specific cell.

        More precise than world_to_grid_cell because it checks exact cell boundaries,
        not just the center.

        Args:
            x: World X coordinate
            y: World Y coordinate
            target_row: Row of target cell
            target_col: Column of target cell

        Returns:
            bool: True if point (x, y) is inside target cell, False otherwise

        Example:
            # Check if robot is inside cell (2, 3)
            x, y, z, _ = spotUtils.getPosition(robot_state_client)
            if env.is_point_in_cell(x, y, 2, 3):
                print("Robot is inside cell (2,3)")
        """
        # Calculate coordinates of target cell center
        cell_center = self.get_world_position_from_cell(target_row, target_col)
        if cell_center is None:
            return False

        center_x, center_y = cell_center

        # Calculate cell boundaries in world coordinates
        half_size = self.cell_size / 2.0

        # Transform world point to coordinates relative to cell center
        delta_x = x - center_x
        delta_y = y - center_y

        # Rotate delta to align with grid (inverse rotation)
        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        local_x = delta_x * cos_yaw - delta_y * sin_yaw
        local_y = delta_x * sin_yaw + delta_y * cos_yaw

        # Check if point is inside cell boundaries
        is_inside = (abs(local_x) <= half_size and abs(local_y) <= half_size)

        if is_inside:
            print(f"[CHECK] Point ({x:.2f},{y:.2f}) IS inside cell ({target_row},{target_col})")
        else:
            print(f"[CHECK] Point ({x:.2f},{y:.2f}) is OUTSIDE cell ({target_row},{target_col}) by ({local_x:.2f},{local_y:.2f})")

        return is_inside

    def mark_cell_blocked(self, row, col):
        """
        Mark a cell as blocked (obstacle detected, cannot enter).
        Sets the map value to -1 to indicate this cell was attempted but is blocked.

        Args:
            row: Row of the cell
            col: Column of the cell
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.map[row][col] = -1
            print(f"[BLOCKED] Cell ({row},{col}) marked as blocked (value=-1)")
        else:
            print(f"[ERROR] Cannot mark cell ({row},{col}) as blocked - out of bounds")

    def is_cell_blocked(self, row, col):
        """
        Check if a cell is marked as blocked.

        Args:
            row: Row of the cell
            col: Column of the cell

        Returns:
            bool: True if cell is blocked (value=-1), False otherwise
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return self.map[row][col] == -1
        return False

    def get_side_bit_facing_origin(self, origin_row, origin_col, target_row, target_col):
        """
        Robot moved/attempted from (origin_row,origin_col) toward (target_row,target_col).
        Returns the bit on TARGET representing the side facing origin — the side that was
        actually tested by this attempt.
        """
        dr, dc = target_row - origin_row, target_col - origin_col
        if dr == -1 and dc == 0:
            return 0b0010  # target is north of origin -> origin faces target's SOUTH side
        if dr == 1 and dc == 0:
            return 0b1000  # target is south of origin -> origin faces target's NORTH side
        if dr == 0 and dc == 1:
            return 0b0001  # target is east of origin -> origin faces target's WEST side
        if dr == 0 and dc == -1:
            return 0b0100  # target is west of origin -> origin faces target's EAST side
        return 0b0000

    def get_valid_neighbor_sides_mask(self, row, col):
        """
        Bitmask of sides that actually HAVE an in-bounds neighbor for this cell (edge/corner
        cells have fewer than 4). This is "every side that could possibly ever be tried" --
        used by all_sides_explored() to know when a cell has truly run out of options, as
        opposed to just comparing against a fixed 0b1111 that assumes 4 neighbors always exist.
        North=0b1000, East=0b0100, South=0b0010, West=0b0001
        """
        mask = 0b0000
        if row - 1 >= 0:
            mask |= 0b1000  # North neighbor exists
        if row + 1 < self.rows:
            mask |= 0b0010  # South neighbor exists
        if col + 1 < self.cols:
            mask |= 0b0100  # East neighbor exists
        if col - 1 >= 0:
            mask |= 0b0001  # West neighbor exists
        return mask

    def all_sides_explored(self, row, col):
        """
        True only if EVERY side that has an in-bounds neighbor has already been tried
        (mark_cell_side_explored) for this cell. A cell should only be marked permanently
        blocked (mark_cell_blocked, value=-1) once this is True -- otherwise it should stay
        at value 0 (unvisited) so the normal frontier/rank system keeps offering it as a
        legitimate candidate to be retried later, from whichever side hasn't been tried yet,
        instead of being special-cased or excluded from exploration after a single failure.
        """
        return (self.get_cell_sides_status(row, col) & self.get_valid_neighbor_sides_mask(row, col)) == \
               self.get_valid_neighbor_sides_mask(row, col)

    def mark_cell_side_explored(self, row, col, side_bit):
        """
        Record that entry was attempted into this cell from the given side.
        side_bit convention (matches get_blocked_neighbors_with_unexplored_side):
            North=0b1000, East=0b0100, South=0b0010, West=0b0001
        """
        cell_key = (row, col)
        current = self.explored_sides.get(cell_key, 0b0000)
        self.explored_sides[cell_key] = current | side_bit
        print(f"[SIDE] Cell ({row},{col}) side {bin(side_bit)} marked explored "
              f"(now {bin(self.explored_sides[cell_key])})")

    def get_adjacent_frontier_cells(self, cell_row, cell_col, path):
        """
        Returns ONLY adjacent neighbors (distance 1) that are unexplored.

        Checks the 4 neighbors in cardinal directions (North, South, East, West) and returns
        those that have not yet been explored (value == 0).

        Args:
            cell_row: Row of current cell
            cell_col: Column of current cell
            path: List of tuples (row, col) representing the serpentine path (required)

        Returns:
            list: List of tuples (row, col, rank) of adjacent unexplored neighbors,
                  sorted by increasing rank. Empty list [] if no neighbors available.

        Example:
            adjacent = env_map.get_adjacent_frontier_cells(2, 3, path)
            if adjacent:
                # adjacent = [(1,3,5), (2,4,7)]  # Neighbors sorted by rank
                target_row, target_col, rank = adjacent[0]  # Take the one with lowest rank
            else:
                # No neighbors available, must use get_lowest_rank_unexplored_cell
        """
        if path is None:
            print(f"[ERROR] get_adjacent_frontier_cells requires path parameter")
            return []

        print(f"\n[ADJACENT] Searching for adjacent unexplored neighbors from cell ({cell_row},{cell_col})")

        # Create a mapping from (row, col) to path index (rank) for fast lookup
        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        unexplored_neighbors = []
        neighbors = [
            (-1, 0, 'north'),  # North: row - 1
            (1, 0, 'south'),   # South: row + 1
            (0, 1, 'east'),    # East: col + 1
            (0, -1, 'west')    # West: col - 1
        ]

        for dr, dc, direction in neighbors:
            neighbor_row = cell_row + dr
            neighbor_col = cell_col + dc

            # Check if neighbor is within bounds
            if not (0 <= neighbor_row < self.rows and 0 <= neighbor_col < self.cols):
                continue

            # Check if neighbor has NOT been explored (value == 0)
            if self.map[neighbor_row][neighbor_col] == 0:
                # Even if the cell overall is still "unvisited" (0), we may have already
                # tried entering it specifically from THIS direction and failed (a side got
                # marked explored without the cell becoming fully blocked, since other sides
                # remain untried). Don't immediately re-offer the exact same failed approach --
                # let it come back around via a different direction, or via
                # get_lowest_rank_unexplored_cell() once no direct neighbor works.
                side_bit = self.get_side_bit_facing_origin(cell_row, cell_col, neighbor_row, neighbor_col)
                if self.get_cell_sides_status(neighbor_row, neighbor_col) & side_bit:
                    print(f"  ✗ Neighbor {direction}: ({neighbor_row},{neighbor_col}) already tried from "
                          f"this side and failed -- skipping for now")
                    continue

                neighbor_cell = (neighbor_row, neighbor_col)
                # Get rank from path
                rank = cell_to_rank.get(neighbor_cell, float('inf'))
                unexplored_neighbors.append((neighbor_row, neighbor_col, rank))
                print(f"  ✓ Neighbor {direction}: ({neighbor_row},{neighbor_col}) with rank={rank}")

        if unexplored_neighbors:
            # Sort by increasing rank (priority to lowest rank)
            unexplored_neighbors.sort(key=lambda x: x[2])
            print(f"[ADJACENT] ✓ Found {len(unexplored_neighbors)} unexplored neighbors")
            print(f"[ADJACENT] Neighbor with lowest rank: ({unexplored_neighbors[0][0]},{unexplored_neighbors[0][1]}) rank={unexplored_neighbors[0][2]}")
            return unexplored_neighbors
        else:
            print(f"[ADJACENT] No adjacent unexplored neighbors found")
            return []

    def get_lowest_rank_from_frontier_list(self, frontier_list, path):
        """
        Finds the cell with the lowest rank from a list of frontier cells.

        This is useful when you already have a list of cells (not necessarily sorted
        or complete) and want to find the one with the lowest rank in the serpentine path.

        Args:
            frontier_list: List of tuples (row, col) or (row, col, rank) of frontier cells
            path: List of tuples (row, col) representing the serpentine path (required)

        Returns:
            tuple or None: Tuple (row, col, rank) of the cell with lowest rank,
                          or None if the list is empty or the path is not valid.

        Example:
            # You have a list of frontier cells (not sorted)
            frontiers = [(3,5), (1,2), (4,7), (2,3)]

            # Find the one with lowest rank
            best = env.get_lowest_rank_from_frontier_list(frontiers, path)
            if best:
                target_row, target_col, rank = best
                print(f"Cell with lowest rank: ({target_row},{target_col}) rank={rank}")
        """
        if path is None:
            print(f"[ERROR] get_lowest_rank_from_frontier_list requires path parameter")
            return None

        if not frontier_list or len(frontier_list) == 0:
            print(f"[FRONTIER_RANK] Empty frontier list")
            return None

        print(f"\n[FRONTIER_RANK] Searching for cell with lowest rank among {len(frontier_list)} frontier cells")

        # Create a mapping from (row, col) to path index (rank) for fast lookup
        cell_to_rank = {cell: idx for idx, cell in enumerate(path)}

        min_rank = float('inf')
        best_cell = None

        for cell in frontier_list:
            # Handle both tuples (row, col) and (row, col, rank)
            if isinstance(cell, tuple):
                if len(cell) >= 2:
                    row, col = cell[0], cell[1]
                    cell_tuple = (row, col)

                    # Get rank from path
                    rank = cell_to_rank.get(cell_tuple, float('inf'))

                    print(f"  Cell ({row},{col}): rank={rank}")

                    if rank < min_rank:
                        min_rank = rank
                        best_cell = (row, col, rank)

        if best_cell:
            print(f"[FRONTIER_RANK] ✓ Cell with lowest rank: ({best_cell[0]},{best_cell[1]}) rank={best_cell[2]}")
            return best_cell
        else:
            print(f"[FRONTIER_RANK] ⚠️ No valid cell found in list")
            return None

    def get_cell_sides_status(self, row, col):
        """
        Get the exploration status of a cell's sides.

        Args:
            row: Row of the cell
            col: Column of the cell

        Returns:
            int: 4-bit value representing explored sides (0b0000 to 0b1111)
        """
        cell_key = (row, col)
        return self.explored_sides.get(cell_key, 0b0000)

    def set_origin(self, x, y, yaw=0.0, start_row=0, start_col=0):
        """
        Set the world coordinates (x, y, yaw) as the origin of the grid.
        The grid is aligned with the robot's initial orientation.
        Mark the starting cell as visited (value 1).

        Args:
            x: World X coordinate to set as origin
            y: World Y coordinate to set as origin
            yaw: Initial yaw orientation of robot in radians
            start_row: Row index of starting cell (default 0)
            start_col: Column index of starting cell (default 0)
        """
        self.origin_x = x
        self.origin_y = y
        self.origin_yaw = yaw
        self.start_cell = (start_row, start_col)

        # Mark starting cell as visited
        self.map[start_row][start_col] = 1

        print(f"[OK] Origin set at world coordinates ({x:.2f}, {y:.2f})")
        print(f"[OK] Initial orientation: {np.rad2deg(yaw):.1f}°")
        print(f"[OK] Starting cell ({start_row}, {start_col}) marked as visited")

    def update_position(self, x, y):
        """
        Update the map based on new world coordinates.
        Marks cell as visited only when robot center is inside the cell boundaries.
        Coordinates are rotated to align with the robot's initial orientation.

        Args:
            x: Current world X coordinate
            y: Current world Y coordinate

        Returns:
            tuple: (row, col) of the current cell, or None if out of bounds
        """
        # Calculate relative position from origin
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y

        # Rotate coordinates to align with grid (inverse rotation)
        # Grid X-axis should align with robot's initial forward direction
        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        grid_x = delta_x * cos_yaw - delta_y * sin_yaw
        grid_y = delta_x * sin_yaw + delta_y * cos_yaw

        # Convert to grid coordinates by checking which cell contains the robot center
        # Grid: X -> columns (right), Y -> rows (forward)
        # Find the cell index by checking boundaries
        half_size = self.cell_size / 2.0

        # Calculate which cell the robot center is in
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

        # Check if we found a valid cell
        if row is not None and col is not None:
            # Check bounds (should always be true if above logic is correct)
            if 0 <= row < self.rows and 0 <= col < self.cols:
                # Mark cell as visited if not already
                if self.map[row][col] == 0:
                    self.map[row][col] = 1
                    print(f"[MAP] Cell ({row}, {col}) marked as visited (robot center inside)")

                self.current_cell = (row, col)
                return (row, col)

        # Robot center is not inside any cell (between cells or out of bounds)
        print(f"[INFO] Robot center at ({x:.2f}, {y:.2f}) -> grid ({grid_x:.2f}, {grid_y:.2f}) is between cells or out of bounds")
        return None

    def get_cell_from_world(self, x, y):
        """
        Convert world coordinates to grid cell without marking as visited.
        Coordinates are rotated to align with the robot's initial orientation.
        """
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y

        # Rotate coordinates to align with grid (inverse rotation)
        cos_yaw = np.cos(-self.origin_yaw)
        sin_yaw = np.sin(-self.origin_yaw)

        grid_x = delta_x * cos_yaw - delta_y * sin_yaw
        grid_y = delta_x * sin_yaw + delta_y * cos_yaw

        # CORRECT: Use round() to round to the nearest cell center
        # Divide by cell_size to get offset in cells, then round
        col = self.start_cell[1] + round(grid_x / self.cell_size)
        row = self.start_cell[0] + round(grid_y / self.cell_size)

        return row, col

    def get_cell_status(self, row, col):
        """
        Get the status of a cell in the map.

        Args:
            row: Row index
            col: Column index

        Returns:
            tuple: (cell_value, explored_sides) where cell_value is:
                   0 = unvisited
                   1 = visited and accessible
                   -1 = attempted but blocked
                   explored_sides is the 4-bit value representing explored sides.
                   Returns (None, None) if out of bounds.
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            cell_value = self.map[row][col]
            explored_sides = self.get_cell_sides_status(row, col)
            return cell_value, explored_sides
        return None, None

    def generate_serpentine_path(self, start_cell=None):
        """
        Generate a serpentine (lawnmower) path through the grid.
        Pattern:
        1  2  3  4  5
        10 9  8  7  6
        11 12 13 14 15
        ...

        Args:
            start_cell: Optional (row, col). If provided (or if omitted, self.start_cell
                        is used), the path is rotated to start from that cell.

        Returns:
            list: List of (row, col) tuples in order to visit
        """
        path = []
        for row in range(self.rows):
            if row % 2 == 0:
                # Even rows: left to right
                for col in range(self.cols):
                    path.append((row, col))
            else:
                # Odd rows: right to left
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
        Convert grid cell to world coordinates (center of cell).
        Applies rotation to transform from grid frame to world frame.
        Cell (0,0) center is at the origin (robot's starting position).

        Args:
            row: Row index
            col: Column index

        Returns:
            tuple: (x, y) world coordinates, or None if out of bounds
        """
        if 0 <= row < self.rows and 0 <= col < self.cols:
            # Calculate offset from start cell in grid frame
            delta_row = row - self.start_cell[0]
            delta_col = col - self.start_cell[1]

            # Grid coordinates (center of cell)
            # Grid: X -> columns (right), Y -> rows (forward)
            # Note: No offset added because cell (0,0) center should be at origin
            grid_x = delta_col * self.cell_size
            grid_y = delta_row * self.cell_size

            # Rotate to world frame (forward rotation)
            cos_yaw = np.cos(self.origin_yaw)
            sin_yaw = np.sin(self.origin_yaw)

            world_delta_x = grid_x * cos_yaw - grid_y * sin_yaw
            world_delta_y = grid_x * sin_yaw + grid_y * cos_yaw

            # Add to origin
            x = self.origin_x + world_delta_x
            y = self.origin_y + world_delta_y

            return (x, y)
        return None

    def add_waypoint(self, x, y):
        """
        Register a waypoint at the given world coordinates.

        Args:
            x: World X coordinate
            y: World Y coordinate
        """
        self.waypoints.append((x, y))
        print(f"[WAYPOINT] Registered waypoint #{len(self.waypoints)} at ({x:.3f}, {y:.3f})")

    def print_map(self):
        """Print the current state of the map."""
        print("\n--- Environment Map ---")
        for row in self.map:
            print(" ".join(str(cell) for cell in row))
        if hasattr(self, 'current_cell'):
            print(f"Current cell: {self.current_cell}")
        print("-----------------------\n")
