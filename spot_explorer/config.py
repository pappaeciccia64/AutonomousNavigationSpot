"""
MISSION PARAMETERS: all the switches and numbers that are changed in the field.

A single place to touch. Every parameter has what it does next to it; why it has that
value (the field tests that decided it) is in CHANGELOG.md. The parameters specific to a
module (grids and maps in perception/, graph in planning/roadmap.py and
planning/edge_evaluation.py, frontier in planning/safe_frontier.py, motion in robot/motion.py)
stay in that module.

The command-line arguments of run_mission.py (--rows, --graph, --spacing, ...) change
these values for a single mission. The code reads them as config.NAME at the time of
use, so the tests (test_replay.py, test_e2e_loop.py) can change them too.
"""
import numpy as np

from spot_explorer.perception import geometry
from spot_explorer.perception import spot_grid

# =========================================================================================
# THE MISSION: size, node graph, edges
# =========================================================================================
# Rows (to the robot's left) x columns (ahead) x cell side. Both the cell map and the graph
# are built from here, so they always stay the same. For a single mission,
# without touching the file: arguments --rows --cols --cell of run_mission.py, or the variable
# SPOT_MISSION_GRID="rows,cols,side".
MISSION_GRID_ROWS = 8
MISSION_GRID_COLS = 7
MISSION_CELL_SIZE_M = 2.0

# How the graph nodes are generated (see mission_graph.py, RANDOM NODES):
#   'lattice' rectangular lattice, deterministic; the centre of every cell is a node;
#   'random'  scattered nodes with MISSION_RANDOM_DISTRIBUTION ('uniform', 'poisson',
#              'jittered', 'halton'), cell centres always included.
# From the command line: --graph --distribution --density --seed.
MISSION_GRAPH_SAMPLING = 'random'
MISSION_RANDOM_DISTRIBUTION = 'poisson'
# Nodes per m2: with 4 the number of nodes is that of the lattice at 0.5 m spacing; 5 a bit
# more, for narrow passages.
MISSION_RANDOM_DENSITY = 9.0
MISSION_RANDOM_SEED = None              # None = drawn at random, and written in the log
MISSION_POISSON_MIN_DIST_M = None       # None = 0.75 / sqrt(density)

# Lattice: spacing between nodes, rounded to cell_size/n (None = mission_graph.
# suggested_spacing()); rings of linked neighbours (None = all nodes between PRM_MIN_EDGE_M and
# PRM_MAX_EDGE_M). From the command line: --spacing --rings.
MISSION_LATTICE_SPACING_M = 0.5
MISSION_LATTICE_RINGS = None

# Edges: all pairs of nodes between PRM_MIN_EDGE_M and PRM_MAX_EDGE_M apart are linked.
# Edges up to 1.25 m long: with shorter edges entering a cell becomes a chain
# of many waypoints, and every waypoint is a rotation, right in the narrow corridors where
# rotating is not possible. PRM_MAX_EDGE_M is also the radius within which the PRM refresh
# rebuilds the candidate pairs. From the command line: --min-edge --max-edge.
PRM_MIN_EDGE_M = 0.5
PRM_MAX_EDGE_M = 1.25
PRM_CONNECTION_RADIUS_M = 3.0

# --- Where the planner may pass -----------------------------------------------------------
# To enter a cell the path may use ONLY the cells already visited, the cell
# the robot is in and the goal cell: cells never explored are not crossed. If
# the only way goes through an unvisited cell the attempt fails, the side is marked
# as tried and the cell is retried later from another side.
PLAN_ONLY_EXPLORED_CELLS = True
# Detour cap: a path longer than RATIO * straight line + SLACK counts as "no
# path" (see PRM.set_detour_limit). None = no cap.
PLAN_MAX_DETOUR_RATIO = 2.0
PLAN_DETOUR_SLACK_M = 1.0


# ==================================================================
# NAVIGATION WITH THE SAFE FRONTIER -- see decide_next_move() and the loop
# in attempt_enter_cell_from_position(). The frontier distances are in perception.geometry
# (FRONTIER_*), here only the logic of the loop.
#   MIN_PARTIAL_STEP_M           -- minimum step: below this length it is not worth
#                                   moving (a scan would be paid for a few cm).
#   FRONTIER_BLOCK_CONFIRM_SCANS -- how many consecutive scans, with the robot STILL, without
#                                   being able to advance even the minimum step, before
#                                   declaring a block: the "up close" confirmation, instead of
#                                   condemning an edge at the first glimpse from afar.
#   FRONTIER_RESCAN_WAIT_S       -- pause between two confirmation scans.
#   MAX_BLOCK_REPLANS            -- after a block it replans from where it is; if
#                                   it happens more than this many times in the same attempt,
#                                   it gives up (retreat, cell deferred). The same
#                                   value also limits the re-choices of the goal point.
#   FRONTIER_ABORT_CONFIRM       -- while moving, how many consecutive frontiers (~0.2 s
#                                   each) must say "you will not get where you are going"
#                                   before stopping the robot. 2 avoids braking for a single
#                                   dirty frame; at 0.5 m/s that is ~20 cm.
#   GOAL_REACHED_TOLERANCE_M     -- if the frontier prevents the last centimetres towards
#                                   the goal (e.g. a wall just beyond), it settles for it.
#   MAX_LOOP_ITERATIONS          -- anti-stall safeguard of the loop.
# ==================================================================
MIN_PARTIAL_STEP_M = 0.40
# FINAL STEP: a step below MIN_PARTIAL_STEP_M (but at least FINAL_STEP_MIN_M) is taken
# anyway if it brings the robot within GOAL_REACHED_TOLERANCE_M of the goal, with the robot
# in the cell: it is the last one, after which the arrival counts. Without it, on 28-10 11:36
# the robot stayed still 0.85 m from the goal while able to advance 0.38 m (it would have got to 0.47);
# on 22-10 16:04 it happened up to 15 times, on 26-10 15:11 up to 8.
FINAL_STEP_MIN_M = 0.10
FRONTIER_BLOCK_CONFIRM_SCANS = 3
FRONTIER_RESCAN_WAIT_S = 0.3
MAX_BLOCK_REPLANS = 3
FRONTIER_ABORT_CONFIRM = 2
FRONTIER_ABORT_TOLERANCE_M = 0.05

# Tied to the offset the frontier itself imposes: allowed_advance() stops the robot's CENTRE
# FRONTIER_BODY_HALF_LENGTH_M + FRONTIER_MARGIN_M = 0.65 m before the first non-free
# point. A smaller tolerance would leave a dead band where the robot can neither
# advance nor declare it has arrived, and it would end up declaring a block. The two
# constants must be the same thing.
GOAL_REACHED_TOLERANCE_M = geometry.FRONTIER_BODY_HALF_LENGTH_M + geometry.FRONTIER_MARGIN_M
MAX_LOOP_ITERATIONS = 150

# --- ANTI-STALL GUARDS --------------------------------------------------------------
# The loop can spin idle with the robot motionless: the rotation towards the next node
# is refused, the cone finds no direction that gets closer, but the usable directions
# are not zero (so it is not a "trap"); the leg is discarded, it replans, and at the next
# turn the first waypoint points elsewhere and the rotation is refused again.
#   MAX_ROTATION_DISCARDS -- leg discards for rotation refusal allowed in a
#       cell attempt. Counted per ATTEMPT and not per pair of nodes: the pair changes
#       at every replan, and a per-pair counter would never trigger (that is the
#       limit of MAX_ROTATION_FAILS_SAME_EDGE).
#   MAX_IDLE_LOOPS -- consecutive loop turns without the robot having moved by
#       at least IDLE_MOVE_M. It is the generic net: whatever branch is spinning idle,
#       after this many still turns the cell is given up in an orderly way instead of using up
#       battery. It must be ABOVE the longest legitimate sequence with the robot still,
#       otherwise it cuts off exactly the fallback nets (return with GraphNav) that the
#       sequence must reach. Computed on the limits of the loop:
#         [SUBITO]                MAX_EARLY_REPLANS              4
#         goal chosen again      MAX_BLOCK_REPLANS              3
#         block confirmations    FRONTIER_BLOCK_CONFIRM_SCANS x (MAX_BLOCK_REPLANS+1) = 3 x 4 = 12
#         rotation refusals      MAX_ROTATION_DISCARDS          4
#         arrival refusals       MAX_ARRIVAL_REFUSALS           3
#                                                        total 26  -> 30
#       At ~0.7 s per still turn that is ~20 s. The idle spin on the rotation is caught
#       first by MAX_ROTATION_DISCARDS; this is the net for everything else.
MAX_ROTATION_DISCARDS = 4
MAX_IDLE_LOOPS = 30
IDLE_MOVE_M = 0.05

# ==================================================================
# ROTATION AND BACK-OFF.
#
# User's decision: the robot ALWAYS points its nose towards the point to reach.
# No oblique walking. Sideways Spot is 1.10 m wide instead of 0.50, so the
# frontier requires 0.59 m of obstacle_distance instead of 0.30: going sideways when it
# cannot rotate would make the robot WIDER exactly where space is tight.
#
# When rotation is not possible the robot BACKS UP IN A STRAIGHT LINE along the body
# axis -- which is not oblique walking: the footprint stays 0.50 m and the frontier
# threshold stays 0.30 -- and at the next turn it tries again from there. The second time on
# the same leg the leg is discarded and it replans.
#   ROTATION_RETREAT_M        -- how far it tries to back up (the frontier may allow less)
#   ROTATION_RETREAT_MIN_M    -- below this back-off it is not worth moving
#   MAX_ROTATION_FAILS_SAME_EDGE -- back-off attempts on the same leg before discarding it
# ==================================================================
ROTATION_RETREAT_M = 0.60
ROTATION_RETREAT_MIN_M = 0.20
MAX_ROTATION_FAILS_SAME_EDGE = 2

# --- Fan inside the possible rotation cone -- see plan_cone_move.
CONE_SWEEP_MAX_DEG = 90.0      # how far it looks, to the right and left of the nose
CONE_SWEEP_STEP_DEG = 10.0     # step of the angular sweep (9 directions per side)
CONE_MIN_GAIN_M = 0.20         # a direction counts only if it gets closer to the goal by this much
# The cone aims at the point of the PATH at this distance from the robot, not at the cell
# goal in a straight line: if the path goes around a block, "getting closer" to the goal
# in a straight line can take the robot into a dead end, on the side opposite to the
# path. If that point is behind, no direction of the cone gets closer, and it moves on
# to the back-off -- which is the right move.
CONE_PATH_LOOKAHEAD_M = 1.5
# During the rotation in place a shorter "end of data" frontier does not interrupt the move:
# while rotating the seen area shifts and the data edge gets a few centimetres shorter without
# any obstacle. It holds until
# the robot has moved by at least this much; a frontier shortened by an OBSTACLE always
# interrupts.
ABORT_IGNORE_DATA_EDGE_UNTIL_M = 0.15

# Consecutive back-offs allowed, COUNTED PER SEQUENCE and not per edge:
# MAX_ROTATION_FAILS_SAME_EDGE is indexed on (robot_node, next_node), and after a
# back-off the robot may find itself attached to another node, so on its own it does not stop
# a chain of back-offs.
MAX_STRAIGHT_RETREATS_IN_A_ROW = 2

# --- 360 degree fan: recognizing the trap -- see scan_escape_directions.
ESCAPE_FAN_STEP_DEG = 15.0        # 24 directions; 24 frontier evaluations, ~a fraction of a scan
ESCAPE_FAN_MIN_ADVANCE_M = 0.30   # below this space a direction is not a way out
# Preventive check on the arrival pose (see arrival_has_exit): paid only where needed,
# i.e. on short steps and towards tight spaces, which are the cases where corners are entered.
ARRIVAL_EXIT_CHECK_MAX_STEP_M = 0.40
ARRIVAL_EXIT_CHECK_OD_M = 0.50
MAX_ARRIVAL_REFUSALS = 3          # per cell attempt, so as not to spiral on the refusal

# --- Returns with GraphNav instead of walking backwards ----------------------------------
# Going back on foot, backwards, over the recorded legs is the riskiest manoeuvre
# we make: the robot cannot see where it goes and does not relocalize. GraphNav walks forwards,
# with Boston Dynamics' localization and obstacle avoidance. A trail of waypoints is
# left along the way (see graphnav_drop_breadcrumb) and the robot goes back along it.
GRAPHNAV_WAYPOINT_MIN_SPACING_M = 0.75   # minimum distance between two breadcrumbs
GRAPHNAV_MAX_BACKTRACK_HOPS = 3          # then the cell is deferred and the next priority is taken

# --- Shortcuts in the GraphNav graph ------------------------------------------------------
# Every waypoint links only to the previous one: the GraphNav graph is a CHAIN that traces the
# way taken, and GraphNav only walks on the edges. Without shortcuts, going back to a waypoint
# one metre away but recorded many steps earlier means retracing the whole
# chain. Before every move with GraphNav (graphnav_add_shortcuts) OUR shortcuts are added,
# chosen ON THE WAY (see plan_graphnav_shortcuts): the shortest
# way to the destination is computed considering ALL the pairs of waypoints less than
# GRAPHNAV_SHORTCUT_MAX_M apart, without a direct edge, whose straight line was SEEN free in the
# global map with GRAPHNAV_SHORTCUT_MARGIN_M of margin, and only those the way uses
# are created. Never through areas never seen. At the end of the mission Spot's loop closures
# are added too (GRAPHNAV_SDK_LOOP_CLOSURE). GraphNav still keeps its
# obstacle avoidance during the move.
# GRAPHNAV_SHORTCUT_MARGIN_M: margin from the occupied cells (which are already at obstacle_distance
# <= 0.15): 0.30 + 0.15 = 0.45 m from the obstacle, wide because the edges are long.
GRAPHNAV_SHORTCUTS = True
GRAPHNAV_SDK_LOOP_CLOSURE = True
# Spot's loop closure, full (with ICP), only once at the end of the mission, before
# anchoring and the return to wp_0: its edges, corrected on the scans, are the only ones
# that correct the drift of the downloaded map. GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S is the maximum
# processing time ON SPOT'S SIDE (Params.timeout_seconds): when it expires Spot returns the
# edges found so far. Spot overruns the limit by 10-20 s (28-10: 16 edges in 55 s, 20 in
# 66 s). It is not called during the mission: the trips use our shortcuts (22-10: 23
# successful trips out of 23 with those alone). Bench measurements: bench_loop_closure.py and CHANGELOG 26-10.
GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S = 45.0
GRAPHNAV_SHORTCUT_MAX_M = 3.0
GRAPHNAV_SHORTCUT_MARGIN_M = 0.30
# Maximum length of the edges Spot's loop closure creates.
GRAPHNAV_SDK_LOOP_MAX_M = 2.0
# A PRM node with less free space than this cannot be reached by the frontier: discarded.
PRM_NODE_MIN_CLEARANCE_M = geometry.FRONTIER_CLEARANCE_M   # 0.30
# "Early" replans (at the first blocking cell, without waiting for the confirmation while
# still) allowed per cell attempt. See the 'wait' branch.
MAX_EARLY_REPLANS = 4
# Every going back has memory for the attempt (see _trap_backtrack): the edge on which the
# robot stopped becomes impassable until the end of the attempt, and at the
# MAX_BACKTRACKS_PER_ATTEMPT-th return the robot goes back and gives up the cell from
# this side. Without memory, from the previous breadcrumb the planner would find the
# same path again and the robot would walk back into the same trap.
MAX_BACKTRACKS_PER_ATTEMPT = 2
# Within this distance the recovery goal is "the same" as the one that just failed.
RECOVERY_SAME_GOAL_M = 0.30

# --- CHOICE OF THE CELL GOAL (see find_best_point_in_cell) -----------------------------
# TARGET_CLEARANCE_M: PREFERRED free space around the goal, on the global map and
# on the live scan. The frontier requires FRONTIER_CLEARANCE_M (0.25) and the frontier's
# hysteresis is 0.04 m: a goal chosen at 0.28 (PLAN_CLEARANCE_M, which stays the absolute
# minimum) oscillates around the threshold. Outdoors (22-10 16:04) the goals chosen at
# 0.28 were refused live at 0.24, at the edge of the grass.
TARGET_CLEARANCE_M = 0.33
# TARGET_CELL_BORDER_M: the goal is preferably chosen at least this distance from the
# cell border. A goal on the border leaves the robot, stopped within the arrival
# tolerance, outside the cell ("0.06 m to go ... still outside cell", 44 times on
# 22-10 16:04).
TARGET_CELL_BORDER_M = 0.30
# Arrival by tolerance with the robot just OUTSIDE the cell: it counts if the centre is outside by
# at most ARRIVAL_OUTSIDE_CELL_M (the nose, 0.55 m ahead, is inside anyway). It is the
# arrival tolerance minus the margin from the border: a goal chosen TARGET_CELL_BORDER_M
# inside the cell, with the robot stopped GOAL_REACHED_TOLERANCE_M from it, always counts. On
# 22-10 16:04, at every "outside the cell" refusal the robot was 0.5-0.65 m from the goal,
# outside by 0.01-0.61 m.
ARRIVAL_OUTSIDE_CELL_M = GOAL_REACHED_TOLERANCE_M - TARGET_CELL_BORDER_M      # = 0.35
# A goal refused by the frontier (live free space below the threshold, or a block
# confirmed within this distance of it) stays excluded for its cell until the end
# of the mission: see _refuse_goal. It already applies at the FIRST frontier stop: if the frontier
# stops for an obstacle within this distance of the goal, or beyond the end of the path
# (in the half-body extension for the nose), it is the goal that does not work, not the way,
# and another one is chosen at once (CellAttempt._repick_goal). Before, the legs were
# discarded one by one ([SUBITO]): on 22-10 16:04, 45 [SUBITO] out of 58 had the obstacle
# right next to the goal; on 26-10 15:11, 18 out of 22.
REFUSED_GOAL_BLOCK_M = 0.35
# When another goal in the cell is chosen, how many candidates to try before
# giving up if none has a path in the graph (CellAttempt._repick_goal).
GOAL_REPICK_CANDIDATES = 3
# Failed replan (the updated graph no longer has a path to the goal):
# if from the robot the graph still reaches at least REPLAN_FAIL_ROBOT_FREE_NODES nodes, the
# goal is isolated, not the robot, and another one with a path is chosen at once
# (CellAttempt._repick_goal). Before, the broken plan was kept and the frontier decided:
# on 28-10 14:23, 20 [REPLAN FALLITO]; at 14:37, 14:41 and 14:48 three still scans, block
# and cell deferred with 19-21 free directions out of 24; at 14:27 and 14:34 one more step
# on the broken plan. False = as before.
REPICK_ON_REPLAN_FAIL = True
REPLAN_FAIL_ROBOT_FREE_NODES = 6

# ==================================================================
# MISSION DATA.
#   Nothing is drawn during the mission: every figure is saved as data in a
#   <name>_vis.npz (see visualize_grid_with_candidates) and the figures are made afterwards with
#   tools/mission_viewer.py.
#   SAVE_SCAN_BUNDLES -- one .npz for every scan of the navigation loop, in the
#       scans/ subfolder: RAW layers as they arrive from Spot (terrain, terrain_valid,
#       obstacle_distance), the derived ones used to decide, the georeferencing (origin,
#       cell, robot pose, estimated ground), path, frontier and decision. It is what
#       mission_viewer.py, mission_report.py and test_replay.py use. ~0.3 MB per scan.
# ==================================================================
SAVE_SCAN_BUNDLES = True
# Extra Spot layers saved in the bundles, with a separate request that does not touch
# navigation (spot_grid.LocalGrid.extra_layers): to understand tall grass (22-10 16:04), i.e.
# whether intensity and no_step tell vegetation from a solid obstacle. Every layer has
# <name>_origin and <name>_cell next to it, because its grid may not coincide with the
# others. () = off.
SAVE_EXTRA_LAYERS = ('intensity', 'no_step')

# Radius of the disk marked as dangerous in the global map around the point of a
# fall (see motion.check_and_recover_fall and GlobalGrid.mark_hazard).
FALL_HAZARD_RADIUS_M = 0.5

# ==================================================================
# RETREAT (see retreat_along_traveled_path).
#   RETREAT_MAX_SEGMENTS  -- how many legs already travelled to retrace backwards at
#                            most. Few: it serves to get out of a pocket, not to
#                            go back to the starting point.
#   RETREAT_MIN_SEGMENT_M -- below this length the leg is skipped (not worth a
#                            motion command of its own).
# ==================================================================
RETREAT_MAX_SEGMENTS = 3
RETREAT_MIN_SEGMENT_M = 0.20
#   RETREAT_CONTINUITY_M  -- it backs up along a recorded leg only if the robot really is
#                            where that leg ended. After a move with GraphNav
#                            between cells (which records no legs) the last recorded leg may
#                            be many metres away, and the robot would go there walking
#                            backwards, blind.
#   RETREAT_MAX_SEGMENT_M -- leg longer than this: it is not retraced backwards.
RETREAT_CONTINUITY_M = 0.50
RETREAT_MAX_SEGMENT_M = 2.50

# ==================================================================
# MOVES WITH GRAPHNAV -- between distant cells, for going back and for the return to wp_0.
# They all go through navigation.graphnav_navigate_to, which:
#   - has a time limit (GRAPHNAV_TIMEOUT_S), so a fallen robot or one without a path does not
#     block the mission;
#   - limits the speed (GRAPHNAV_MAX_*_VEL): the moves with GraphNav are the
#     longest of the mission;
#   - treats STATUS_LOST as a failure, with the robot still, after the re-anchoring attempts: it does not
#     force the localization on the destination waypoint declaring "arrived" when the
#     robot could be elsewhere.
# ==================================================================
MAX_FALLS_PER_MISSION = 3      # at the third fall the robot gets up and the mission stops

# BATTERY (see battery.py). The charge is read at every loop turn and before/after the
# GraphNav moves, and goes into battery.csv with the consumption per minute and per metre. Below
# BATTERY_RETURN_PCT, confirmed over BATTERY_RETURN_CONFIRM readings in a row, the mission stops
# exploring and closes as at the end of a mission: final scan, saving, return to wp_0
# with GraphNav. 30% is provisional: it must be tuned on the measured consumption (battery.csv, and the charge
# spent on the return to wp_0, [BATTERIA] line at closure). 0 = return off (it is measured
# anyway).
BATTERY_RETURN_PCT = 30.0
BATTERY_RETURN_CONFIRM = 2     # readings in a row below threshold (against an isolated value)
BATTERY_SAMPLE_S = 5.0         # at most one reading every this many seconds in the loop
GRAPHNAV_TIMEOUT_S = 240.0
# Return to wp_0 stopped within this distance (STUCK, timeout): it stays a return that did NOT
# succeed, but fiducial and drift are measured anyway (see graphnav_return_to_start).
GRAPHNAV_RETURN_NEAR_M = 1.0

# A loop leg whose command ends badly (timeout, Spot not settling)
# without a frontier stop, with the robot within this distance of the arrival point, counts
# as successful instead of as a physical failure (edge discarded forever). 0 = off.
MOVE_NEAR_GOAL_M = 0.10
GRAPHNAV_MAX_LINEAR_VEL_MPS = 0.5
GRAPHNAV_MAX_ANGULAR_VEL_RPS = 0.6

# --- GRAPHNAV LOCALIZATION -------------------------------------------------------------
# Outdoors GraphNav's live localizer can get lost even when the map is
# sound (leg odometry and vision agreeing within ~15 cm over tens of metres), and a
# lost robot has every move rejected. So:
#   1. [LOCALIZZAZIONE] line at every breadcrumb, only when something changes, to see WHEN
#      and WHY it gets lost (Spot's loss detector);
#   2. re-anchoring: if GraphNav is lost, it relocalizes on the nearest waypoint according to
#      our poses, with the initial guess from odometry (waypoint_tform_ko of the graph times
#      ko_tform_body now: exact up to the drift), then Spot's ICP refines it.
#      It does NOT declare "arrived": it tries to navigate again.
#   3. PERMISSIVE loss detector in GraphNav moves.
GRAPHNAV_LOST_DETECTOR_STRICTNESS = 'LOST_DETECTOR_STRICTNESS_PERMISSIVE'   # None = Spot's one

# HEADING ON ARRIVAL of the GraphNav trips between cells and of the returns to breadcrumbs (the
# final return to wp_0 always keeps the starting heading):
#   'toward_goal'  arrives facing the centre of the cell to attempt (trips between cells)
#                      or the goal of the attempt (returns): the first move after
#                      arrival starts almost straight;
#   'waypoint'         the heading the robot had when it created the waypoint (as
#                      until 28-10);
#   'free'           GraphNav does not turn at the end (ignore_final_yaw).
# On 26-10 15:11 (8 arrivals): after arrival the robot rotated in place 79 degrees on average
# (38 s in all); left free it would have been 82, facing the goal 22.
GRAPHNAV_ARRIVAL_HEADING = 'toward_goal'
GRAPHNAV_ARRIVAL_HEADING_MIN_M = 0.5   # goal closer than this to the waypoint: direction not reliable, the waypoint's is used
GRAPHNAV_MAX_RELOCALIZATIONS = 3       # re-anchorings per single GraphNav move
GRAPHNAV_RELOC_MAX_DIST_M = 0.5        # ICP search radius around the guess
GRAPHNAV_RELOC_MAX_YAW_RAD = float(np.radians(20.0))
GRAPHNAV_RELOC_MAX_WP_DIST_M = 3.0     # beyond this distance from the nearest waypoint it is not tried

# Before moving with GraphNav to the waypoint from which to attempt a cell, the graph is checked
# to reach the cell from there (see graph_reaches_cell). In the test of 22-10
# 15:24, three moves out of six ended in an attempt failed in less than a second with
# "GRAPH DISCONNECTED", which the graph already knew before leaving: ~25 s out of 3 minutes.
# False = it leaves anyway and the attempt decides.
GRAPHNAV_PRECHECK = True

# STARTING FIDUCIAL. At start-up a fiducial seen by any camera is searched for
# (the rear one too) and remembered: it goes into the snapshot of wp_0, the robot localizes
# on it, and on return to wp_0 it is searched again to measure the odometry drift of
# the whole mission. Spot only recognizes AprilTag fiducials (family 36h11, the Boston
# Dynamics ones), not ArUco.
#   START_FIDUCIAL_ID     -- None = the nearest, any ID; a number = only that one.
#   START_FIDUCIAL_WAIT_S -- how long to wait for Spot to see it (right after standing up it takes a while).
START_FIDUCIAL_ID = None
START_FIDUCIAL_WAIT_S = 3.0
# CORRECTION OF THE RETURN TO wp_0 with the starting fiducial (graphnav_correct_wp0_with_fiducial).
# GraphNav can say "arrived" while far from the start: it localizes by comparing the
# scans with those of the waypoint, and in a place poor in references it gets it wrong (29-10 10:57:
# 1.2 m, lawn with a single hedge; the odometry drift was 2.8 m). The starting tag does not
# move: the robot position in its frame tells the true offset. Beyond
# WP0_FIDUCIAL_TOLERANCE_M the robot relocalizes on the tag and goes back to wp_0, at most
# WP0_FIDUCIAL_MAX_CORRECTIONS times. False = measurement only, as before.
WP0_FIDUCIAL_CORRECTION = True
WP0_FIDUCIAL_TOLERANCE_M = 0.30
WP0_FIDUCIAL_MAX_CORRECTIONS = 2

SHORTCUT_MAX_DIST_M = 2.5          # beyond it, the leg does not fit in the window anyway
