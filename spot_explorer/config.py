"""
Mission parameters: the numbers and options that are changed in the field.

Each parameter is explained next to it; why it has its value (the field tests behind it) is in
CHANGELOG.md. Parameters that belong to one module stay in that module: grids and maps in
perception/, graph in planning/roadmap.py and planning/edge_evaluation.py, frontier in
planning/safe_frontier.py, motion in robot/motion.py.

The command-line arguments of run_mission.py (--rows, --graph, --spacing, ...) override these
values for one mission. The code reads them as config.NAME when it uses them, so the tests
(test_replay.py, test_e2e_loop.py) can change them too.
"""
import numpy as np

from spot_explorer.perception import geometry

# =========================================================================================
# The mission: size, node graph, edges
# =========================================================================================
# Rows (to the robot's left) x columns (ahead) x cell side. The cell map and the graph are both
# built from these values. For one mission: --rows --cols --cell of run_mission.py, or the
# variable SPOT_MISSION_GRID="rows,cols,side".
MISSION_GRID_ROWS = 8
MISSION_GRID_COLS = 7
MISSION_CELL_SIZE_M = 2.0

# The cell the robot starts in, looking at the grid from above with the robot's forward
# direction up (see cells.py):
#     'top-left'      'top-centre'      'top-right'
#     'centre-left'   'centre'          'centre-right'
#     'bottom-left'   'bottom-centre'   'bottom-right'
# 'bottom-right' is cell (0, 0): the grid opens ahead and to the left of the robot.
# 'bottom-left': ahead and to the right; 'top-...': the grid is behind the robot; 'centre': the
# robot in the middle of the area. A cell can also be given as "row,column". With an even number
# of rows or columns the centre is n//2. From the command line: --start.
MISSION_START_POSITION = 'bottom-right'

# Order in which the cells are visited:
#   'serpentine' rows walked back and forth, starting from the robot's corner;
#   'rings'      wider and wider rings around the starting cell;
#   'auto'       serpentine if the robot starts in a corner, rings otherwise.
# From the command line: --order.
MISSION_VISIT_ORDER = 'auto'

# How the graph nodes are generated (see mission_graph.py):
#   'lattice' rectangular lattice; the centre of every cell is a node;
#   'random'  scattered nodes with MISSION_RANDOM_DISTRIBUTION ('uniform', 'poisson',
#             'jittered', 'halton'), cell centres always included.
# From the command line: --graph --distribution --density --seed.
MISSION_GRAPH_SAMPLING = 'random'
MISSION_RANDOM_DISTRIBUTION = 'poisson'
# Nodes per m2: 4 gives as many nodes as the lattice at 0.5 m spacing; a bit more helps in
# narrow passages.
MISSION_RANDOM_DENSITY = 9.0
MISSION_RANDOM_SEED = None              # None = drawn at random, and written in the log
MISSION_POISSON_MIN_DIST_M = None       # None = 0.75 / sqrt(density)

# Lattice: spacing between nodes, rounded to cell_size/n (None = mission_graph.
# suggested_spacing()); rings of linked neighbours (None = all nodes between PRM_MIN_EDGE_M and
# PRM_MAX_EDGE_M). From the command line: --spacing --rings.
MISSION_LATTICE_SPACING_M = 0.5
MISSION_LATTICE_RINGS = None

# Edges: every pair of nodes between PRM_MIN_EDGE_M and PRM_MAX_EDGE_M apart is linked. With
# shorter edges a path becomes a chain of many nodes, each a possible rotation, which is a
# problem in narrow corridors. PRM_MAX_EDGE_M is also the radius within which the graph refresh
# rebuilds the candidate pairs. From the command line: --min-edge --max-edge.
PRM_MIN_EDGE_M = 0.5
PRM_MAX_EDGE_M = 1.25
PRM_CONNECTION_RADIUS_M = 3.0

# --- Where the planner may pass -----------------------------------------------------------
# A path into a cell may use only the cells already visited, the cell the robot is in and the
# goal cell: cells never explored are not crossed. If the only way goes through an unvisited
# cell the attempt fails and the cell rules try the cell again later.
PLAN_ONLY_EXPLORED_CELLS = True
# Detour cap: a path longer than RATIO * straight line + SLACK counts as "no path" (see
# PRM.set_detour_limit). None = no cap.
PLAN_MAX_DETOUR_RATIO = 2.0
PLAN_DETOUR_SLACK_M = 1.0


# ==================================================================
# Navigation with the safe frontier: the loop of a cell attempt (cell_attempt.py,
# decisions.decide_next_move). The frontier distances are in perception.geometry (FRONTIER_*).
#   MIN_PARTIAL_STEP_M           -- shortest step worth a move (a scan costs more than a few cm).
#   FRONTIER_BLOCK_CONFIRM_SCANS -- consecutive scans, with the robot standing, without being
#                                   able to advance the minimum step before a block is declared:
#                                   an edge is condemned only after a look from close by.
#   FRONTIER_RESCAN_WAIT_S       -- pause between two confirmation scans.
#   MAX_BLOCK_REPLANS            -- replans after a block allowed in one attempt; then the
#                                   attempt gives up. Also limits the re-choices of the goal.
#   FRONTIER_ABORT_CONFIRM       -- while moving, how many consecutive frontiers (~0.2 s each)
#                                   must say the robot will not get there before it is stopped.
#                                   2 ignores a single noisy frame; at 0.5 m/s that is ~20 cm.
#   GOAL_REACHED_TOLERANCE_M     -- if the frontier forbids the last centimetres to the goal
#                                   (e.g. a wall just beyond), the robot settles for this.
#   MAX_LOOP_ITERATIONS          -- limit on the turns of the loop.
# ==================================================================
MIN_PARTIAL_STEP_M = 0.40
# Final step: a step shorter than MIN_PARTIAL_STEP_M (but at least FINAL_STEP_MIN_M) is taken
# anyway if it brings the robot, inside the cell, within GOAL_REACHED_TOLERANCE_M of the goal:
# after it the arrival counts.
FINAL_STEP_MIN_M = 0.10
FRONTIER_BLOCK_CONFIRM_SCANS = 3
FRONTIER_RESCAN_WAIT_S = 0.3
MAX_BLOCK_REPLANS = 3
FRONTIER_ABORT_CONFIRM = 2
FRONTIER_ABORT_TOLERANCE_M = 0.05

# The frontier stops the robot's centre FRONTIER_BODY_HALF_LENGTH_M + FRONTIER_MARGIN_M (0.65 m)
# before the first point that is not free (allowed_advance). The arrival tolerance must be the
# same distance: with a smaller one the robot could neither advance nor arrive, and would end
# up declaring a block.
GOAL_REACHED_TOLERANCE_M = geometry.FRONTIER_BODY_HALF_LENGTH_M + geometry.FRONTIER_MARGIN_M
MAX_LOOP_ITERATIONS = 150

# --- Stall checks ---------------------------------------------------------------------------
# The loop can turn with the robot standing still: the rotation towards the next node is
# refused, the cone finds no direction that gets closer, but some directions are usable (so it
# is not a trap); the leg is discarded, the path changes and the new first node is refused again.
#   MAX_ROTATION_DISCARDS -- legs discarded for a refused rotation in one attempt. Counted per
#       attempt, not per pair of nodes: the pair changes at every replan.
#   MAX_IDLE_LOOPS -- consecutive turns without the robot moving IDLE_MOVE_M: then the attempt
#       gives up. It must be above the longest legitimate sequence with the robot standing,
#       otherwise it cuts off the later fallbacks (return with GraphNav):
#         early replans          MAX_EARLY_REPLANS              4
#         goal chosen again      MAX_BLOCK_REPLANS              3
#         block confirmations    FRONTIER_BLOCK_CONFIRM_SCANS x (MAX_BLOCK_REPLANS+1) = 3 x 4 = 12
#         rotation refusals      MAX_ROTATION_DISCARDS          4
#         arrival refusals       MAX_ARRIVAL_REFUSALS           3
#                                                        total 26  -> 30
#       At ~0.7 s per turn that is ~20 s.
MAX_ROTATION_DISCARDS = 4
MAX_IDLE_LOOPS = 30
IDLE_MOVE_M = 0.05

# ==================================================================
# Rotation and back-off.
#
# The robot always points its nose towards the point it walks to: no sideways walking. Spot is
# 1.10 m long and 0.50 m wide, so sideways the frontier would need 0.59 m of obstacle_distance
# instead of 0.30, exactly where space is tight.
#
# When it cannot rotate, the robot backs up in a straight line along its body axis (footprint
# and threshold unchanged) and tries again at the next turn. The second time on the same leg,
# the leg is discarded and the path recomputed.
#   ROTATION_RETREAT_M           -- how far it tries to back up (the frontier may allow less)
#   ROTATION_RETREAT_MIN_M       -- shortest back-off worth a move
#   MAX_ROTATION_FAILS_SAME_EDGE -- back-offs on the same leg before discarding it
# ==================================================================
ROTATION_RETREAT_M = 0.60
ROTATION_RETREAT_MIN_M = 0.20
MAX_ROTATION_FAILS_SAME_EDGE = 2

# --- Fan inside the rotation cone (see plan_cone_move).
CONE_SWEEP_MAX_DEG = 90.0      # how far it looks, to the right and left of the nose
CONE_SWEEP_STEP_DEG = 10.0     # step of the angular sweep (9 directions per side)
CONE_MIN_GAIN_M = 0.20         # a direction counts only if it gets closer to the goal by this much
# The cone aims at the point of the path this far from the robot, not at the goal in a straight
# line: around a block, getting closer in a straight line can lead into a dead end on the other
# side. If that point is behind, no direction gets closer and the robot backs up instead.
CONE_PATH_LOOKAHEAD_M = 1.5
# During a rotation in place the seen area shifts, and an "end of data" frontier can get a few
# cm shorter without any obstacle: it does not interrupt the move until the robot has moved this
# much. A frontier shortened by an obstacle always interrupts.
ABORT_IGNORE_DATA_EDGE_UNTIL_M = 0.15

# Consecutive back-offs allowed, counted per sequence: MAX_ROTATION_FAILS_SAME_EDGE is per
# (robot_node, next_node), and after a back-off the robot may be attached to another node.
MAX_STRAIGHT_RETREATS_IN_A_ROW = 2

# --- 360 degree fan: recognizing a trap (see scan_escape_directions).
ESCAPE_FAN_STEP_DEG = 15.0        # 24 directions
ESCAPE_FAN_MIN_ADVANCE_M = 0.30   # below this space a direction is not a way out
# Check that the arrival pose has a way out (see arrival_has_exit), made only on short steps and
# towards tight spaces, where the robot can walk into a corner.
ARRIVAL_EXIT_CHECK_MAX_STEP_M = 0.40
ARRIVAL_EXIT_CHECK_OD_M = 0.50
MAX_ARRIVAL_REFUSALS = 3          # per cell attempt

# --- Going back with GraphNav ----------------------------------------------------------------
# Walking backwards over the recorded legs is blind and without relocalization. GraphNav walks
# forwards, with Boston Dynamics' localization and obstacle avoidance, along a trail of
# waypoints left on the way (see graphnav_drop_breadcrumb).
GRAPHNAV_WAYPOINT_MIN_SPACING_M = 0.75   # minimum distance between two breadcrumbs
GRAPHNAV_MAX_BACKTRACK_HOPS = 3          # then the attempt gives up the cell

# --- Shortcuts in the GraphNav graph --------------------------------------------------------
# Every waypoint is linked only to the previous one, so the GraphNav graph is a chain along the
# way taken and GraphNav walks only on its edges: going back to a waypoint one metre away but
# recorded long before would retrace the whole chain. Before every GraphNav move
# (graphnav_add_shortcuts) the shortest way to the destination is computed over all the pairs
# of waypoints less than GRAPHNAV_SHORTCUT_MAX_M apart, without a direct edge, whose straight
# line is seen free in the global map with GRAPHNAV_SHORTCUT_MARGIN_M of margin; only the
# shortcuts that way uses are created. Never through areas not seen. GraphNav keeps its own
# obstacle avoidance during the move.
# GRAPHNAV_SHORTCUT_MARGIN_M is measured from the occupied cells (already within 0.15 m of an
# obstacle): 0.30 + 0.15 = 0.45 m from the obstacle, wide because the edges are long.
GRAPHNAV_SHORTCUTS = True
GRAPHNAV_SDK_LOOP_CLOSURE = True
# Spot's full loop closure (with ICP) runs once at the end of the mission, before the anchoring
# and the return to wp_0: its edges, corrected on the scans, are the only ones that correct the
# drift of the downloaded map. GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S is the processing time on Spot's
# side (Params.timeout_seconds); Spot returns the edges found so far when it expires, usually
# 10-20 s late. During the mission the trips use only our shortcuts.
GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S = 45.0
GRAPHNAV_SHORTCUT_MAX_M = 3.0
GRAPHNAV_SHORTCUT_MARGIN_M = 0.30
# Maximum length of the edges Spot's loop closure creates.
GRAPHNAV_SDK_LOOP_MAX_M = 2.0
# A PRM node with less free space than this cannot be reached by the frontier: discarded.
PRM_NODE_MIN_CLEARANCE_M = geometry.FRONTIER_CLEARANCE_M   # 0.30
# Early replans (at the first blocking cell, without waiting for the confirmation scans) allowed
# per cell attempt. See the 'wait' branch.
MAX_EARLY_REPLANS = 4
# Every going back is remembered for the attempt (see _trap_backtrack): the edge the robot
# stopped on stays impassable until the end of the attempt, so the planner does not find the
# same path into the same trap. At the MAX_BACKTRACKS_PER_ATTEMPT-th return the attempt gives up.
MAX_BACKTRACKS_PER_ATTEMPT = 2
# Within this distance a recovery goal is "the same" as the one that just failed.
RECOVERY_SAME_GOAL_M = 0.30

# --- Choice of the cell goal (see find_best_point_in_cell) -----------------------------------
# Preferred free space around the goal, on the global map and on the live scan: 0.08 above the
# frontier threshold. A goal chosen at the minimum would sit on the threshold and its hysteresis
# (0.04 m), and be refused live as soon as the data move a little.
TARGET_CLEARANCE_M = geometry.FRONTIER_CLEARANCE_M + 0.08
# The goal is chosen at least this far from the cell border when possible: a goal on the border
# leaves the robot, stopped within the arrival tolerance, outside the cell.
TARGET_CELL_BORDER_M = 0.30
# Arrival by tolerance with the robot just outside the cell counts if the centre is outside by at
# most ARRIVAL_OUTSIDE_CELL_M (the nose, 0.55 m ahead, is inside anyway). It is the arrival
# tolerance minus the border margin: with a goal TARGET_CELL_BORDER_M inside the cell and the
# robot GOAL_REACHED_TOLERANCE_M from it, the arrival always counts.
ARRIVAL_OUTSIDE_CELL_M = GOAL_REACHED_TOLERANCE_M - TARGET_CELL_BORDER_M      # = 0.35
# A goal refused by the frontier (live free space below the threshold, or a block confirmed
# within this distance of it) stays excluded for its cell until the end of the mission (see
# _refuse_goal). This already applies at the first frontier stop: if the frontier stops for an
# obstacle within this distance of the goal, or beyond the end of the path (in the half body in
# front of the nose), the goal is the problem, not the way, and another one is chosen at once
# (CellAttempt._repick_goal).
REFUSED_GOAL_BLOCK_M = 0.35
# Candidates tried when another goal in the cell is chosen, before giving up if none has a path
# in the graph (CellAttempt._repick_goal).
GOAL_REPICK_CANDIDATES = 3
# Failed replan (the updated graph has no path to the goal): if from the robot the graph still
# reaches at least REPLAN_FAIL_ROBOT_FREE_NODES nodes, the goal is isolated, not the robot, and
# another goal with a path is chosen at once (CellAttempt._repick_goal).
REPICK_ON_REPLAN_FAIL = True
REPLAN_FAIL_ROBOT_FREE_NODES = 6

# ==================================================================
# Mission data.
#   Nothing is drawn during the mission: every figure is saved as data in a <name>_vis.npz (see
#   visualize_grid_with_candidates) and drawn afterwards with tools/mission_viewer.py.
#   SAVE_SCAN_BUNDLES -- one .npz for every scan of the navigation loop, in scans/: the raw
#       layers from Spot (terrain, terrain_valid, obstacle_distance), the derived ones used to
#       decide, the georeferencing (origin, cell, robot pose, estimated ground), path, frontier
#       and decision. Read by mission_viewer.py, mission_report.py and test_replay.py. ~0.3 MB
#       per scan.
# ==================================================================
SAVE_SCAN_BUNDLES = True
# Extra Spot layers saved in the bundles, with a separate request that does not touch the
# navigation (spot_grid.LocalGrid.extra_layers), to tell vegetation from solid obstacles
# afterwards. Each layer has <name>_origin and <name>_cell next to it, since its grid may differ
# from the others. () = off.
SAVE_EXTRA_LAYERS = ('intensity', 'no_step')
# Photos from the five body cameras (robot.photos) at a confirmed block (same name as the
# BLOCKED_n.npz) and when GraphNav stops (STUCK, no route, ...), in MissionMap/<mission>/photos/,
# with the camera pose in VISION and the intrinsics: what the robot had in front. ~0.3 s and
# ~0.3 MB each time. False = off.
SAVE_BLOCK_PHOTOS = True

# Radius of the disk marked as dangerous in the global map around the point of a fall (see
# motion.check_and_recover_fall and GlobalGrid.mark_hazard).
FALL_HAZARD_RADIUS_M = 0.5

# ==================================================================
# Retreat (see retreat_along_traveled_path).
#   RETREAT_MAX_SEGMENTS  -- legs already walked to retrace backwards at most: enough to get out
#                            of a pocket, not to go back to the start.
#   RETREAT_MIN_SEGMENT_M -- shorter legs are skipped (not worth a command of their own).
# ==================================================================
RETREAT_MAX_SEGMENTS = 3
RETREAT_MIN_SEGMENT_M = 0.20
#   RETREAT_CONTINUITY_M  -- it backs up along a recorded leg only if the robot is where that leg
#                            ended. After a GraphNav move (which records no legs) the last leg
#                            may be metres away, and the robot would walk there backwards, blind.
#   RETREAT_MAX_SEGMENT_M -- longer legs are not retraced.
RETREAT_CONTINUITY_M = 0.50
RETREAT_MAX_SEGMENT_M = 2.50

# ==================================================================
# GraphNav moves: between distant cells, going back, and the return to wp_0. They all go
# through navigation.graphnav_navigate_to, which:
#   - has a time limit (GRAPHNAV_TIMEOUT_S), so a fallen robot or one without a route does not
#     block the mission;
#   - limits the speed (GRAPHNAV_MAX_*_VEL): these are the longest moves of the mission;
#   - treats STATUS_LOST as a failure, with the robot standing, after the re-anchoring attempts:
#     it never declares arrival on a waypoint when the robot could be elsewhere.
# ==================================================================
MAX_FALLS_PER_MISSION = 3      # at the third fall the robot gets up and the mission stops

# Battery (see battery.py). The charge is read at every loop turn and before and after the
# GraphNav moves, and goes into battery.csv with the consumption per minute and per metre. Below
# BATTERY_RETURN_PCT, for BATTERY_RETURN_CONFIRM readings in a row, the mission stops exploring
# and closes as usual: final scan, saving, return to wp_0 with GraphNav. 30% is provisional, to
# tune on the measured consumption (battery.csv and the [BATTERY] line of the return). 0 = no
# return (the charge is measured anyway).
BATTERY_RETURN_PCT = 30.0
BATTERY_RETURN_CONFIRM = 2     # readings in a row below the threshold (against an isolated value)
BATTERY_SAMPLE_S = 5.0         # at most one reading every this many seconds in the loop
GRAPHNAV_TIMEOUT_S = 240.0
# A return to wp_0 stopped within this distance (STUCK, timeout) is still not a success, but the
# fiducial and the drift are measured anyway (see graphnav_return_to_start).
GRAPHNAV_RETURN_NEAR_M = 1.0

# A loop leg whose command ends badly (timeout, Spot not settling) without a frontier stop, with
# the robot within this distance of the arrival point, counts as done instead of as a physical
# failure (edge discarded for good). 0 = off.
MOVE_NEAR_GOAL_M = 0.10
GRAPHNAV_MAX_LINEAR_VEL_MPS = 0.5
GRAPHNAV_MAX_ANGULAR_VEL_RPS = 0.6

# --- GraphNav localization -------------------------------------------------------------------
# Outdoors GraphNav's localizer can get lost even with a sound map, and a lost robot has every
# move rejected. So:
#   1. a [LOCALIZATION] line at every breadcrumb when something changes, to see when and why it
#      gets lost (Spot's loss detector);
#   2. re-anchoring: if GraphNav is lost, it relocalizes on the nearest waypoint according to our
#      poses, with the initial guess from odometry (waypoint_tform_ko of the graph times
#      ko_tform_body now: exact up to the drift), refined by Spot's ICP. Then it navigates again;
#   3. a permissive loss detector in the GraphNav moves.
GRAPHNAV_LOST_DETECTOR_STRICTNESS = 'LOST_DETECTOR_STRICTNESS_PERMISSIVE'   # None = Spot's one

# Heading on arrival of the GraphNav trips between cells and of the returns to breadcrumbs (the
# final return to wp_0 always keeps the starting heading):
#   'toward_goal'  facing the centre of the cell to attempt (trips between cells) or the goal of
#                  the attempt (returns): the first move after arrival starts almost straight;
#   'waypoint'     the heading the robot had when it created the waypoint;
#   'free'         GraphNav does not turn at the end (ignore_final_yaw).
GRAPHNAV_ARRIVAL_HEADING = 'toward_goal'
GRAPHNAV_ARRIVAL_HEADING_MIN_M = 0.5   # goal closer than this to the waypoint: direction not reliable, the waypoint's is used
GRAPHNAV_MAX_RELOCALIZATIONS = 3       # re-anchorings per single GraphNav move
GRAPHNAV_RELOC_MAX_DIST_M = 0.5        # ICP search radius around the guess
GRAPHNAV_RELOC_MAX_YAW_RAD = float(np.radians(20.0))
GRAPHNAV_RELOC_MAX_WP_DIST_M = 3.0     # beyond this distance from the nearest waypoint it is not tried


# Starting fiducial. At start-up a fiducial seen by any camera (the rear one too) is searched
# for and remembered: it goes into the snapshot of wp_0, the robot localizes on it, and on the
# return to wp_0 it is searched again to measure the odometry drift of the whole mission. Spot
# only recognizes AprilTag fiducials (family 36h11, the Boston Dynamics ones), not ArUco.
#   START_FIDUCIAL_ID     -- None = the nearest, any ID; a number = only that one.
#   START_FIDUCIAL_WAIT_S -- how long to wait for Spot to see it (right after standing up it
#                            takes a while).
START_FIDUCIAL_ID = None
START_FIDUCIAL_WAIT_S = 3.0
# Correction of the return to wp_0 with the starting fiducial (graphnav_correct_wp0_with_fiducial).
# GraphNav can report arrival while far from the start: it compares the scans with those of the
# waypoint, and in a place with few references it gets it wrong. The tag does not move, so the
# robot position in its frame gives the true offset. Beyond WP0_FIDUCIAL_TOLERANCE_M the robot
# relocalizes on the tag and goes back to wp_0, at most WP0_FIDUCIAL_MAX_CORRECTIONS times.
# False = measurement only.
WP0_FIDUCIAL_CORRECTION = True
WP0_FIDUCIAL_TOLERANCE_M = 0.30
WP0_FIDUCIAL_MAX_CORRECTIONS = 2

SHORTCUT_MAX_DIST_M = 2.5          # beyond it, the leg does not fit in the window anyway
