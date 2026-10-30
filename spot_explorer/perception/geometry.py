"""Robot footprint and distances from obstacles: safe frontier, planner, rotation in place."""
import numpy as np

# =========================================================================
# OBSTACLE, SLOPE AND ROUGHNESS THRESHOLDS
#   OBSTACLE_THRESHOLD  a cell is an obstacle if obstacle_distance <= this (m)
#   SLOPE_THRESHOLD     slope above which an edge is discarded (tangent). The slope is measured
#                       over a 0.48 m base (see SLOPE_BASELINE_M), so a SHARP STEP of
#                       height H counts as H / 0.48: 0.6 corresponds to a step of ~29 cm,
#                       Spot's limit (~30 cm). That is why no separate step measurement
#                       is needed.
#   ROUGH_THRESHOLD     roughness (standard deviation of the height, m) above which a cell is
#                       an obstacle
# =========================================================================
OBSTACLE_THRESHOLD = 0.15   # obstacle_distance (m): distance <= this -> blocked

# =========================================================================
# ROBOT FOOTPRINT AND SAFE FRONTIER (see safe_frontier.compute_safe_frontier)
#   FRONTIER_CLEARANCE_M: minimum distance from an obstacle for the CENTRE LINE of the path:
#     half width (0.25) + air (ROBOT_CLEARANCE_AIR_M). Read directly from
#     obstacle_distance. The edge veto comes from here too (PRM_EDGE_SAFETY_MARGIN_M).
#   ROBOT_CLEARANCE_AIR_M: the air between body and obstacle. 0 = only the body footprint: the
#     robot neither plans nor starts where the body does not fit, but with no margin. A usable
#     corridor is at least 2 * FRONTIER_CLEARANCE_M wide, and the smaller the margin the
#     sparser the graph can be to find narrow passages (see mission_graph, ALIASING).
#   FRONTIER_BODY_HALF_LENGTH_M: the nose is 0.55 m ahead of the centre; the centre stops at
#     half length + FRONTIER_MARGIN_M before the first non-free point.
#   FRONTIER_MIN_CLEARANCE_M / FRONTIER_ESCAPE_DIST_M: if the robot is ALREADY closer than
#     FRONTIER_CLEARANCE_M to something (tight space), in the first metre it is enough not
#     to get any closer, never below FRONTIER_MIN_CLEARANCE_M: that way it can get out.
#   FRONTIER_ROBOT_RADIUS_M: within this radius "unseen" cells do not stop the frontier
#     (it is the ground under the robot, covered by the body).
#   FRONTIER_ABORT_AIR_M: it STARTS with FRONTIER_CLEARANCE_M on the sides; while moving it
#     stops only below ROBOT_HALF_WIDTH_M + FRONTIER_ABORT_AIR_M (0.25), i.e. if
#     something really enters the footprint (user's decision: "if it sees it free, go").
# =========================================================================
ROBOT_HALF_LENGTH_M = 0.55          # Spot: 1.10 x 0.50 m
ROBOT_HALF_WIDTH_M = 0.25
ROBOT_CLEARANCE_AIR_M = 0.0         # air between body and obstacle (0.05 = with margin)
FRONTIER_CLEARANCE_M = ROBOT_HALF_WIDTH_M + ROBOT_CLEARANCE_AIR_M      # = 0.25
FRONTIER_MIN_CLEARANCE_M = 0.05
FRONTIER_ESCAPE_DIST_M = 1.0
FRONTIER_BODY_HALF_LENGTH_M = ROBOT_HALF_LENGTH_M                       # = 0.55
FRONTIER_MARGIN_M = 0.10
FRONTIER_ABORT_AIR_M = 0.0

# =========================================================================
# ROTATION IN PLACE (see rotation_is_clear)
# Rotating in place the body corners trace a circle of radius equal to the
# half diagonal, ~0.60 m. The cells the body really sweeps are checked.
#   ROTATION_CLEARANCE_AIR_M: air on the rotation, 0: refused only if the body really
#     penetrates (a false "cannot rotate" is expensive: chains of back-offs).
#   ROTATION_CLEARANCE_M: radius of the prefilter (half diagonal + air).
#   ROTATION_CHECK_MIN_DYAW_DEG: below this rotation the swept space is negligible.
# =========================================================================
ROTATION_CLEARANCE_AIR_M = 0.0
ROTATION_CLEARANCE_M = float(np.hypot(ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M)) + ROTATION_CLEARANCE_AIR_M  # ~0.604
ROTATION_CHECK_MIN_DYAW_DEG = 10.0

# Penetration tolerated in the rotation, against WALLS (see ROTATION_CHECK_PENETRATION for
# thin obstacles). 0.15 m: absorbs the grid registration error (5-10 cm) and the
# obstacle_distance noise; with less, in 6 scans out of 46 of a mission the robot found
# no direction to turn in, while GraphNav rotated and passed at those points.
# If tests show walls being grazed during rotations, set it back to 0.10.
ROTATION_NOISE_TOLERANCE_M = 0.15

# Metric of the rotation check: PENETRATION = depth of the cell inside the body
# minus obstacle_distance. The minimum of obstacle_distance alone is not enough: inside a thin
# obstacle it does not go below half its thickness (a 10 cm pole does not go below -0.05), and
# with the tolerance at 0.15 a pole entering the body would be approved. For a wall the two
# metrics coincide. The cells under the body NOW are excluded: the robot is standing on them.
ROTATION_CHECK_PENETRATION = True


def body_extents(heading, motion_dir):
    """
    Body footprint relative to the direction of travel: (half along the travel,
    half across). heading = body heading, motion_dir = direction of motion, in radians.
    With heading == motion_dir: (0.55, 0.25); sideways: (0.25, 0.55).
    """
    a = motion_dir - heading
    c, s = abs(np.cos(a)), abs(np.sin(a))
    along = ROBOT_HALF_LENGTH_M * c + ROBOT_HALF_WIDTH_M * s
    across = ROBOT_HALF_LENGTH_M * s + ROBOT_HALF_WIDTH_M * c
    return float(along), float(across)
FRONTIER_SAMPLE_STEP_M = 0.03        # one sample per cell
FRONTIER_ROBOT_RADIUS_M = 0.60

# Angular step used to sample the rotation in rotation_is_clear(). 5 degrees: at
# radius 0.60 m that is 5 cm of arc, i.e. less than two cells, so between two sampled poses
# no obstacle the body would cross can hide.
ROTATION_SWEEP_STEP_DEG = 5.0


def rotation_is_clear(obstacle_dist, origin_x, origin_y, cell_size, robot_x, robot_y,
                      robot_yaw, dyaw, half_along=ROBOT_HALF_LENGTH_M,
                      half_across=ROBOT_HALF_WIDTH_M, air=ROTATION_CLEARANCE_AIR_M,
                      step_deg=ROTATION_SWEEP_STEP_DEG):
    """
    Can the robot rotate in place by `dyaw` radians without touching anything?

    Checks the cells the body REALLY sweeps during the rotation (a sector), instead of
    requiring the whole circumscribed circle to be free: obstacles are where they are, and
    the whole circle would refuse many safe rotations.

    The rectangle is already inflated by the air (`air`). It works on a window cropped
    around the robot (~0.3 ms for a normal check, ~0.9 ms for 180 degrees).

    Args:
        obstacle_dist: 2D grid (num_y, num_x) of obstacle_distance, in metres.
        origin_x, origin_y, cell_size: grid geometry in the VISION frame.
        robot_x, robot_y, robot_yaw: current robot pose.
        dyaw: requested rotation, in radians (sign included).
        half_along, half_across: half body footprint (Spot default: 0.55 x 0.25).
        air: air left around the body.
        step_deg: angular sampling step.

    Returns:
        (clear: bool, margin: float) -- margin is the worst penetration (sign flipped) over
        the swept cells, or +inf if the window falls outside the grid: in that case
        nothing is known and the answer is True (the safe frontier will decide).
    """
    if obstacle_dist is None or cell_size is None:
        return True, float('inf')
    reach = float(np.hypot(half_along, half_across)) + air
    num_rows, num_cols = obstacle_dist.shape
    c0 = max(int(np.floor((robot_x - reach - origin_x) / cell_size)), 0)
    c1 = min(int(np.ceil((robot_x + reach - origin_x) / cell_size)), num_cols - 1)
    r0 = max(int(np.floor((robot_y - reach - origin_y) / cell_size)), 0)
    r1 = min(int(np.ceil((robot_y + reach - origin_y) / cell_size)), num_rows - 1)
    if c1 <= c0 or r1 <= r0:
        return True, float('inf')

    rows, cols = np.mgrid[r0:r1 + 1, c0:c1 + 1]
    dx = origin_x + (cols + 0.5) * cell_size - robot_x
    dy = origin_y + (rows + 0.5) * cell_size - robot_y
    window = obstacle_dist[r0:r1 + 1, c0:c1 + 1]

    num_steps = max(2, int(abs(np.degrees(dyaw)) / step_deg) + 2)
    worst = float('inf')
    if ROTATION_CHECK_PENETRATION:
        # Cells under the body NOW: the robot stands on them, they are free by definition.
        c_now, s_now = np.cos(robot_yaw), np.sin(robot_yaw)
        under_now = ((np.abs(dx * c_now + dy * s_now) <= half_along) &
                     (np.abs(-dx * s_now + dy * c_now) <= half_across))
    for t in np.linspace(0.0, float(dyaw), num_steps):
        a = robot_yaw + t
        ca, sa = np.cos(a), np.sin(a)
        along = np.abs(dx * ca + dy * sa)
        across = np.abs(-dx * sa + dy * ca)
        in_body = (along <= half_along + air) & (across <= half_across + air)
        if ROTATION_CHECK_PENETRATION:
            in_body &= ~under_now
            if in_body.any():
                # PENETRATION = depth of the cell inside the body - obstacle_distance.
                depth = np.minimum(half_along + air - along, half_across + air - across)[in_body]
                worst = min(worst, float((window[in_body] - depth).min()))
        elif in_body.any():
            worst = min(worst, float(window[in_body].min()))
    return worst >= -ROTATION_NOISE_TOLERANCE_M, worst

# Distance from obstacles the PLANNER chooses edges with: the frontier's plus
# PLAN_CLEARANCE_EXTRA_M. With the same distance as the frontier the paths passed exactly
# at the limit, and the noise of the live scan (1-2 cm) got them refused: in the test
# of 22-10 15:24, 18 frontier turns out of 25 stopped by an obstacle at 0.23-0.24 m with a
# minimum of 0.25, with advances of 0.2-0.5 m. The narrowest passage is always decided by
# the frontier; the planner leaves a few more centimetres. 0 = same distance as the frontier.
PLAN_CLEARANCE_EXTRA_M = 0.03
PLAN_CLEARANCE_M = FRONTIER_CLEARANCE_M + PLAN_CLEARANCE_EXTRA_M          # = 0.28

# Margin with which the PRM discards edges near occupied cells of the global map. The
# occupied cells have obstacle_distance <= OBSTACLE_THRESHOLD, so an accepted edge passes
# at least OBSTACLE_THRESHOLD + margin = PLAN_CLEARANCE_M from the obstacle.
PRM_EDGE_SAFETY_MARGIN_M = PLAN_CLEARANCE_M - OBSTACLE_THRESHOLD   # = 0.13
