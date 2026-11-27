# History of changes

Most recent changes at the top. For each one: what changed, why, and the field data that motivated it. The code only describes how it works today; this file tells how it got there.

Missions are given as `day-month time` (for example `13-10 12:40`); their data is in `MissionLogs/` and `MissionMap/`.

Until 29-10-2026 the code was a flat set of Italian-named modules (`easy_walk.py`, `parametri.py`, `spotGrid.py`, `arcVerification.py`, `navGraphUtils.py`, `movements.py`, `registro.py`, `environmentMap.py`, `prm_graph.py`, `batteria.py`, `visualizzatore_live.py`, `prova_*.py`, `test_missione.py`, `test_obiettivi.py`, ...) with Italian logs. The entries below that date use those names, log lines and values as they were at the time; the entry of 30-10 says where each piece went.

---

## 2026-11-26 evening (office PC: simulation and frontier, diagnostics)

- `--frontier-beyond M` (`frontier.FRONTIER_BEYOND_M`, default 0 = off): with `--goals frontier`
  the goal is the node up to M m beyond the frontier node, into the unseen (tried on 20-11
  with 1.0 m in the fake missions). Not yet tried in Isaac Sim.
- `[CRUISE-DIAG]` line, with a scan bundle, when the heading the path asks differs from the
  robot's by more than 45 deg or flips (log only); the scan bundles save the graph of the
  moment (`prm_live_*`), which `live_viewer.py` and `mission_viewer.py` draw.
- Simulated robot (`spot_explorer/sim/controller.py`): a turn on the spot at a node starts only
  once the robot is there, and the side of a turn of nearly 180 deg is kept (the robot spun on
  the spot with frontier goals, 26-11 17:14-17:58; not yet tried in Isaac Sim);
  `SPOT_SIM_TRACE=<file.csv>`. `[SIM]` lines in the mission log: world and start pose.
- Worlds `sim/worlds/campo_prova.json` (woods with a 20% ramp) and `thicket.json` (denser, 25%).

## 2026-11-26 evening (cleanup: catch-alls, log categories, the cruise loop in steps, unused code, docs)

Toward the code tried in Isaac Sim: no behaviour change, checked with the tests and
`tools/behaviour_snapshot.py` at every step.

- **Errors**: in `mission/` nothing catches everything any more outside the emergency procedure
  and the outermost `main()` (`robot/errors.py` rules: robot calls catch `ROBOT_ERRORS`,
  optional parts use `optional()`, decision code catches nothing; the `terrain_errors` count
  goes). The steps that were silent now print what failed.
- **Log categories** (`records.LOG_OFF`, the colleague's idea without one function per message):
  a line whose `[TAG]` is listed is written neither on screen nor in `mission_log.txt`, with the
  untagged lines after it. Empty: nothing changes until categories are chosen. `tests/test_log.py`.
- **The cruise loop in named steps** (`attempt_cruise._cruise`, 299 lines): setup, first send,
  one turn (feedback, checks, take the plan, switch while walking, on the plan), stop, outcome.
  Same statements in the same order.
- **The side mask of the cells** (`explored_sides`, `get_cell_sides_status`) moves into
  `varianti/side_rules.py`, the only rules that use it; `get_cell_status` returns the value alone.
- **Unused code out**: `mission_prm.robot_on_node`, `graphnav_step_back`, `geometry.body_extents`,
  `GlobalTerrainGrid.get_height`, `slopes._slope_profile_from_sampler`,
  `safe_frontier.describe_frontier` and `goal_still_reachable`, `config.SHORTCUT_MAX_DIST_M`,
  `motion.navigate_to_path` (with its test), the segment log (`SegmentLog`, `log_segment`,
  `segments.csv` is no longer written: its readers stay for the old missions), unused
  parameters (`stop_if_moving`, `_planner_to`, `create_vtk_terrain_grid`,
  `visualize_grid_with_candidates`).
- **Docs**: README (cell choice by the cell rules, the cruise, modules, tools and tests of
  today), FIELD_TEST (rows of features that left with the stop-and-go out), TODO (done items
  out), SIMULATOR.

## 2026-11-26 (the colleague's simulation on this code: run_mission.py --sim)

- The simulation of GennaiG00/AutonomousNavigationSim (andrea-branch, 20-25/11) brought onto
  this code: `spot_explorer/sim/` (the simulated Spot over a socket), `sim/` (Isaac Sim and the
  kinematic simulator, worlds, scripts), `tools/sim_viewer.py`, `sim_sensor_viewer.py`,
  `sim_selftest.py`, `sim_turn_check.py`, `tests/test_sim_core.py`; `run_mission.py --sim`, no
  lease / E-Stop with the simulated robot (`robot/connection.py`, `mission/run.py`). See
  `sim/README_SIM.md` and `SIMULATOR.md`. The bridge written here on 20-11 (`sim/backend.py`,
  `kinematic_world.py`, `spot_adapter.py`, `tools/run_sim_mission.py`) is removed.
- The trajectory of a command is followed point by point and in time (andrea-branch 033020c,
  done on the office PC on 25-11): the cruise sends multi-point paths.
- The failures of the simulated services are the SDK's errors (`SimNotRecordingError`,
  `SimGraphNavError`) and a lost link to the simulator is a robot error (`robot/errors.py`):
  the mission catches only those since 25-11.
- Checked with the kinematic simulator (3 x 4 cells, worlds boxes, forest_hills, woods): cells
  12/12 in one cruise each (18.7-19.6 m, 0.32-0.33 m/s), frontier 24-28 goals with none failed,
  100% of the area seen everywhere; `tools/sim_selftest.py --sim` all OK.

## 2026-11-26 (what only the stop-and-go used, out)

- `planning/decisions.py` removed (decide_next_move, rotation_plan, shortcut_index, the cone and
  escape fans, arrival_has_exit: the loop of the stop-and-go) and 27 parameters of `config.py`
  used only by it or by nothing (MIN_PARTIAL_STEP_M, FRONTIER_BLOCK_CONFIRM_SCANS, the stall
  checks, rotation back-offs, cone and escape fans, arrival exit check, GraphNav backtracking,
  MAX_EARLY_REPLANS, ...). Kept: MAX_BLOCK_REPLANS, GOAL_REACHED_TOLERANCE_M,
  GRAPHNAV_WAYPOINT_MIN_SPACING_M, RECOVERY_SAME_GOAL_M, the retreat after a fall.
- The stop-and-go mission can still be run: `varianti/tools/run_stop_and_go_mission.sh`, the code
  of 6dc6cbe in a temporary worktree.

## 2026-11-26 (a bug in the planning thread stops the mission; cruise bundles from a saver thread)

- **Bugs are not hidden** (user, 25-11): in the planning thread a robot error (`ROBOT_ERRORS`)
  is logged and the next cycle tries again; any other exception is a bug, kept with its
  traceback, the thread stops planning and `tracker.raise_if_failed()` raises it again in the
  main thread (cruise start, every turn, after `wait_plan`). The cruise stops the robot and the
  emergency procedure of `run.py` sits it down and saves the maps. Tests: `test_e2e_loop` S19
  (a TypeError in the planning cycle after 1 m), `test_mission` world G, `tests/test_errors.py`.
- **Cruise bundles from a saver thread** (`planning_cycle.BundleSaver`, user 26-11): the extra
  layers (intensity, no_step) are asked to Spot by the saver, not by the planning thread, which
  never waits for Spot or the disk. More than `BUNDLE_QUEUE_MAX` (8) bundles behind: the oldest
  waiting is dropped (`bundle_drops` in the summary). Same content; `extra_time` says when the
  extra layers were taken.
- **`test_e2e_loop` S3** (a person stepping in front): the 0.20 m minimum distance is no longer
  checked for this scenario, as for the 0.8 m box. On the robot the brake for an obstacle
  appearing 0.3 m from the nose is Spot's own body obstacle avoidance, which the fake robot does
  not model; no stop rule of ours for it (user, 26-11). Measured 0.12 m, attempt ended
  "blocked on the spot".

## 2026-11-26 (goals from cells or from the frontier: the mission option --goals)

- `run_mission.py --goals cells|frontier` (`config.MISSION_GOALS`, default `cells`), a mission
  option to compare the two in the simulator (user, 26-11). The navigation is the same.
- `spot_explorer/mission/frontier.py` (back from `varianti/`): the goal is the frontier node
  (seen, free, next to the unseen) cheapest to reach from the robot with Dijkstra, with the
  distance from the route of the visit as a node cost (`PRM.set_route_line`, `ROUTE_WEIGHT`
  1.0, saturated at 1.0 m: only with the frontier goals, the cells' path search is unchanged);
  the end at 97% of the area seen. The forced goal is now in `goal.find_best_point_in_cell`
  (`env._forced_goal`); with the frontier goals `cells.cell_seen_fraction` and
  `mission_prm.planning_cells` do not apply (`env.goal_mode`). No hooks replacing functions.
- Why now: in the fake missions with the obstacle cost of the nodes (8a6115c), the frontier
  goals never left the robot blocked on the spot and kept it >= 0.31 m from the boxes, where
  the cells left it blocked 8-11 times and at 0.28 m in two worlds (a cell goal beside a box);
  the frontier was slower in the worlds with walls (+30-55% time).
- `tests/test_frontier.py` (back), `tools/behaviour_snapshot.py --goals`, `tools/run_sim_mission.py
  -- --goals frontier`, README, SIMULATOR.md.

## 2026-11-25 (stop-and-go removed: the cruise is the only motion)

The user's rule (25-11): the robot never stands "thinking". In three steps:
- **No way on is a block at once**: at the end of the confirmed stretch with no way on, the
  planning thread treats the goal as one without a path and takes another goal in the cell
  (`MAX_BLOCK_REPLANS`), else the next cell, else the cruise ends with the cell left for later.
  A first plan that does not allow to start is the same block.
- **A cruise ended for an exception is followed by a new cruise** from where the robot is
  (`MAX_CRUISE_RESTARTS`, 3, in a row: failure of the cell). A fall: the robot gets up, backs
  away along the way it came and the attempt ends. Low battery: the attempt ends.
- **Blocked on the spot**: `BLOCK_STILL_MAX` (3) blocks with the robot moved less than
  `BLOCK_STILL_M` (0.30 m): no other goal is asked, the attempt ends as a failure of the cell.
  Before, a robot that could take no goal from where it stood asked for one after another until
  the 180 s cap (in the cruise reference a run hung 2.5 h).
- **Removed**: `mission/attempt_legs.py`, `attempt_obstacles.py`, `legacy/stop_and_go_start.py`,
  the stop-and-go loop and phases of `cell_attempt`, `CONTINUOUS_MOTION` and the start outside
  the planning thread; `recovery` keeps the retreat after a fall. The cruise bundles save the
  extra layers again. The old code: 6dc6cbe, runnable with `varianti/tools/run_stop_and_go_mission.sh`.
- Tests: the scenarios of `test_e2e_loop` and the fake missions of `test_mission` run in
  cruise; expectations of the stop-and-go out (S9, S12, S17 removed; the others in the commit
  messages of 03c6fa5, f521845, c03f397).

## 2026-11-25 (no detour cap: the navigation goes wherever a path exists)

- `tests/test_any_goal.py` (new): goals drawn at random among all the mission graph nodes (seen
  or not, near boxes, inside them, behind walls), each one attempt of the real cruise with the
  goal forced on the node, judged against the fake world (a 0.30 m disk reaching the node).
  30 goals in 5 worlds: 23 reachable and reached, 5 unreachable and noticed, 0 hangs, 0
  reached where the body cannot go, closest approach 0.34-0.50 m. The 2 misses were reachable
  goals refused by the detour cap (8.5 m around a wall for 2 m in a straight line).
- Detour cap removed (user, 25-11): `PLAN_MAX_DETOUR_RATIO` / `PLAN_DETOUR_SLACK_M`,
  `PRM.set_detour_limit`, `_within_caps` and the unused `max_edges` of `find_path_dijkstra`.
  Whether a goal is worth a long way round is for the goal choice to say, not the navigation.
  With the cells: a cell behind a wall is reached going round instead of left for later. The
  old behaviour: the `detour_cap` variant of `varianti/`.

## 2026-11-25 (obstacle cost on the nodes)

- The edges check only that the body walks straight along them; turning on the spot at a node
  needs the body's circle (~0.60 m), and nothing told Dijkstra. Every node now has a cost from
  its distance d to the nearest obstacle: `OBSTACLE_COST_W_M * ((D0 - d) / D0)^2` for d < D0 =
  0.60 m, W = 4.0 m, never infinite (a corridor with no alternative is still taken); d from the
  live scan, otherwise the global map; a never-seen node has no cost
  (`roadmap.update_obstacle_costs`, recomputed in `refresh_graph`).
- Fake missions: closest approach 0.28-0.63 m; the blocks on the spot that remain happen at the
  goal of the cell just done with the next cell behind, which a node cost cannot move.

## 2026-11-25 (errors: robot calls catch only the SDK's errors)

- `robot/errors.py`: `ROBOT_ERRORS` (the SDK's `bosdyn.client.exceptions.Error` and the frame
  tree's) and `optional(what)`. In `perception/`, `planning/`, `graphnav/` and `robot/` the
  `except Exception` around robot calls catch `ROBOT_ERRORS`: a bug is no longer taken for a
  robot failure (user, 25-11: optional parts log and go on, bugs in the decisions are not
  hidden). Photos and extra grid layers are optional; shortcuts and arrival heading keep a
  logged fallback; the battery monitor never stops the mission. The fakes of the tests raise
  SDK errors instead of RuntimeError. `tests/test_errors.py`.

## 2026-11-24 evening (cleanup: the alternatives out of the main code, the last switches)

Continues the cleanup of 20-11. The main code has no on/off switches for alternatives any more:
each alternative still worth trying is a local variant on this PC (folder `varianti/`, its own
git, ignored here), which replaces one function of the main code at start-up
(`python3 varianti/run.py --with <variant> -- <arguments of run_mission.py>`; the
`[CONFIG] SWITCHES` line and `mission_summary.json` say which variants ran). Every step checked
with the tests and with `tools/behaviour_snapshot.py` (same decisions as before).

- **Alternatives moved to variants**, with the function of the main code they replace:
  `roadmap.A_STAR_PATH` (`PRM._path_heuristic`, None = Dijkstra); `spot_grid.GROUND_FROM_GRID`
  (`LocalGrid._ground_z_of_grid`); `edge_evaluation.BATCH_ADJACENCY` and the edge-by-edge slopes
  (`EdgeEvaluation._set_edge`); `attempt_cruise.CRUISE_STATE_CACHE` with `robot/state_cache.py`,
  `planning_cycle.PLAN_EVENT_DRIVEN` (`CruiseSteps._cruise_state_client`, `CruisePlanner._skip_grid`,
  `plan_period`); the old 0.2 s sleep of the main thread, for comparison with the wake-up on a new
  plan that is now the normal behaviour (`CruiseSteps._wait_for_plan`); `exploration.GOAL_SOURCE =
  'frontier'` with `mission/frontier.py` (including `FRONTIER_BEYOND_M`) and the route line
  `roadmap.ROUTE_LINE_*` (`PRM._route_cost`, 0 in the main code); `cell_rules.EXPLORE_CELL_ON_THE_WAY`
  (`CellRules._near_choice`); the old cell rules, `mission/legacy/side_rules.py` (they replace
  `exploration.Mission`). Their tests and the one-off tools that used them (`compare_route_line`)
  went with them.
- **Switches removed, the robot's value kept**: `config.PLAN_ONLY_EXPLORED_CELLS`
  (`mission_prm.planning_cells` is the one place that says which cells a path may use; None =
  every cell), `exploration.SEEN_BEFORE_MOVING`, `attempt_common.CELL_SEEN_EXPLORED`. The tests'
  fake sensor sees through walls, so in the fake world no cell counts as seen
  (`test_e2e_loop` replaces `cells.cell_seen_fraction` when it is imported, as it does for the
  robot pose); the scenarios about the seen rule put the real function back for themselves.
- **Summary**: the Cruises line prints `cruise_from_robot`, `cruise_edge_drops`,
  `cruise_stale_plans`; the Cells line prints the cells on the way only when the variant ran.
- **Docs**: README options table without the removed switches; FIELD_TEST.md timing section
  with the variants instead of the switches.
- Known: with the `event_driven_planning` variant, `test_cruise` fails "the attempt ends cleanly"
  in the box-at-0.8 m scenario (3 runs out of 3; it passes with the main code).

## 2026-11-24 afternoon (cruise: the PRM path as usual, from the robot only when the edge walked is closed)

- **From the robot only when needed** (`CruisePlanner.__call__`): Dijkstra from the next
  node, the path of the graph, as before. The robot becomes a temporary node only when the
  edge being walked is closed: out of the graph, or closed by the frontier (log "Edge being
  walked ... closed ...: path from the robot", stat `cruise_from_robot`). The morning
  version planned from the robot at every cycle: 64 path changes in 137 plans, and links
  from the robot cut a box corner inside the under-body exemption of `sample_flags`
  (test_cruise, cell without a reachable goal: 0.26 m from the box with the CPU free).
  User: the plan is the normal PRM one; the temporary node is for blocks.
- **The plan starts from the real pose, not a prediction** (`pose_now` replaces
  `lead_point`): the heavy part of the cycle (maps, edges) runs on the scan; then the robot
  state is read and the plan starts there. The frontier and the robot's links see the body
  at that pose (`GridSnapshot.at`). User: no prediction of where the robot will be.
- **The main thread wakes on a new plan** (`CruiseSteps._wait_for_plan` ->
  `tracker.wait_plan`, at most `CRUISE_POLL_S`): plan read ~1 ms after it is published,
  instead of up to 0.2 s. It was the `CRUISE_WAKE_ON_PLAN` switch, off since 19-11; now
  the normal behaviour.
- Results: test_cruise, test_mission, test_path_command, test_e2e_loop all OK (shared test
  queue). Cell without a reachable goal: 0.30 m from the box in 3 runs. Wall gap: one
  cruise in 3 runs, 0.32-0.35 m.

## 2026-11-24 (cruise: Dijkstra from the robot, the plan starts where the robot will be)

User's goal: exploration all in cruise. Dead ends, turning and backing up are normal motion,
not exceptions, and they come from general rules, not per-scenario code. Before, a cruise
ended when the edge being walked went out of the graph. In the logs of 03, 04 and 09-11
there were 16 such ends: the robot stopped and the stop-and-go took over.

- **The robot as a temporary node** (`edge_evaluation.PRM.connect_point` / `remove_point`,
  `TEMP_NODE`). At every cycle the robot is linked to the nodes within edge reach. Each
  link gets the same checks and cost as a graph edge in the refresh:
  `_occupancy_blocked_many` (live scan + global map), `precompute_arc_slopes`,
  `_refresh_edge`. Nothing is taken as free just because it was walked. The node is
  removed at the end of the cycle; the graph and its statistics keep no trace of it.
- **Dijkstra from the robot** (`CruisePlanner._path`), with the stability margin as
  before: the way on can be the next node, the previous one or another. The graph/frontier
  rule (`closed_edge`) closes the robot's link too, for that cycle only. The end "the edge
  being walked is out of the graph" is removed, together with `CellSwitch.blocked` and the
  `edge_blocked` status.
- **The plan starts where the robot will be** (`CruisePlanner.lead_point`). A plan takes
  about one cycle (0.3-0.5 s), and the robot keeps walking meanwhile. A path that turned
  away from the scan pose was already 20-30 cm off the robot, and the cruise ended with
  "the new path passes 24 cm from the robot" (test_cruise, cell without a reachable goal:
  then the stop-and-go took the centre to 0.27 m from the box). Now the plan starts on
  the stretch being walked, which is already checked, ahead of the robot by what it walked
  between the last two scans. The plan sent is robot -> that point -> the new nodes.
- **A plan still far from the robot is skipped** (`attempt_cruise._cruise`, stat
  `cruise_stale_plans`): the robot keeps walking the stretch sent, which is safe up to its
  frontier, and the next plan starts from where it is. This was already the rule for a
  plan carrying the next cell; now it holds for every plan. Before, the cruise ended.
- Results:
  - test_cruise, wall with a 1.2 m gap, 3 runs: one cruise each, standing 1.4-2.0 s,
    centre at 0.34-0.35 m from the wall. Before: 1-2 cruises, standing up to 4.9 s.
  - The 16 real "out of the graph" ends, replayed (`tools/replay_cruise_ends.py`, new):
    16/16 find a path from the robot, with no edge closed by the frontier. The first node
    is ahead in 12 cases (in 3 the node the robot was heading for) and to the side in 4
    (51-105 deg). The confirmed stretch is 0.25-1.83 m. Never backwards.
  - test_cruise, box appearing 0.8 m ahead (the fake robot has no body obstacle avoidance
    and walks into it): the cruise ends with "rotation not clear" instead of "out of the
    graph", and the stop-and-go sometimes brushes the box with its cone move. User,
    24-11: a scenario impossible in reality, no code for it.
- **`tools/replay_cruise_ends.py`** (new): the cruise ends of the real logs whose reason
  matches `--reason`, planned again with today's code on the mission's scans. Limit: the
  scan used is the last cruise scan saved before the end (up to a few seconds before).

## 2026-11-23 (behaviour snapshot, for the cleanup)

- `tools/behaviour_snapshot.py record|check|compare`: a snapshot of what the code decides --
  real saved scans through the real decoding and processing (`tools/saved_scan.py` packs them
  back into Spot's messages), the replay of 4 missions (every graph update, every path), 5
  stop-and-go fake missions (the log without times). Deterministic: two runs of the same code
  give the same snapshot (checked). Sensitive: OBSTACLE_THRESHOLD 0.15 -> 0.16 shows in every
  part. ~1.5 min with 7 processes. The cruise is not in it (real time, two threads).
- Found while building it: some mission counters (MISSION_STATS, the last gait) are module
  globals not reset between two missions in the same process -- harmless with one mission per
  process, as on the robot; every fake mission of the snapshot runs in its own process.

## 2026-11-23 (cruise: one heading rule, forward or backward)

- **`CruisePlanner._turns_within`** decides the heading on every edge of the confirmed
  stretch. Forward if the rotation to get there is clear (`geometry.rotation_is_clear`,
  swept area); otherwise backward; otherwise the stretch stops there. The start is checked
  too, with the same air as the corners: before, a corner approved ahead was refused once
  it became the start, the cruise stopped and the stop-and-go took over at 0.26 m from the
  box (test_cruise, 1 run in 5).
- The plan carries `backward`, one flag per edge. `PathCommand.send` / `path_trajectory`
  walk those edges tail first (heading + 180 deg, at most `RETREAT_MAX_LINEAR_VEL_MPS`),
  so a dead end is left backwards with no rotation at its tip. `_first_plan` no longer
  checks the start rotation on its own: the check is in the plan.
- `test_cruise.check_heading_rule`: open field, dead end, corridor (0.7 m), node beside
  the robot in the corridor. `test_path_command.test_backward`.

## 2026-11-20 (frontier goal beyond the edge; test plan for the simulator)

- `frontier.FRONTIER_BEYOND_M` (default 0 = off; since 24-11 in the `frontier` variant, `varianti/frontier.py`): the goal is the node up to that far beyond
  the frontier node, into the unseen, furthest along robot -> frontier (through valid edges,
  not near a refused goal). Fake missions (`tools/compare_route_line.py --variants frontier
  --frontier-beyond B`, worlds A, B and 3 random):

  | B | goals | failed | walked | off line | seen | time |
  |---|---|---|---|---|---|---|
  | 0 | 151 | 0 | 220 m | 0.27 m | 98% | 690 s |
  | 1.0 | 92 | 0 | 231 m | 0.32 m | 98% | 688 s |
  | 1.5 | 91 | 25 | 208 m | 0.40 m | 97% | 709 s |

  1 m: 40% fewer goals, same time and area; 1.5 m: goals in obstacles refused. The fake
  robot does not slow down at the end of a command, so the gain in time (if any) is for the
  simulator or the robot: to try there with 1.0.
- SIMULATOR.md: test plan for the first days (bring-up, baseline in cells, timing switches,
  cells against frontier, terrain).

## 2026-11-20 (coverage of the mission area: the progress without the cells)

- **`[COVERAGE]` line** (`exploration.Mission._log_coverage`): the mission area seen in the global map, every time it grows by `COVERAGE_STEP` (5 points) and at the end, with the growth since the last line and since the start and the minutes to `COVERAGE_TARGET` (95%) at the mean rate so far. It does not depend on the cells (the frontier mode has no cells to count); also when a cell is done while walking. The `[PROGRESS]` line of the cells is unchanged.
- **`coverage.csv`** in the mission log folder (time, minutes, % seen, closed cells), and the list in `mission_summary.json` (`mission_stats.progress.coverage`).
- **The area seen counted on the raster of the global map** (`known_many`) instead of point by point on the dictionary: same fraction (`tests/test_coverage.py`, 5 random maps), a few ms on large grids.
- `tools/mission_report.py`: a `Coverage:` line (area seen at the end, when it reached 25, 50, 75%), from the summary or from the `[COVERAGE]` lines; `tools/live_viewer.py` shows the last `[COVERAGE]` line.

## 2026-11-20 (cruise: arrival by distance from the goal; one rule graph/frontier for closed edges)

- **Arrival in cruise**: the robot has arrived when it is within `GOAL_REACHED_TOLERANCE_M`
  (0.65 m, half the body plus a margin; user's choice) of the goal with the stretch ahead
  closed, inside the target cell or not. 09-11 14:54: two minutes back and forth, with the
  goal 0.56 m away but the robot outside the cell. In the cruises of 03-09-11, 78 of the
  89 ends on a short stretch were within 0.65 m of the goal.
- **One rule graph/frontier** (`planning_cycle.closed_edge`): an edge after the one being
  walked on which the frontier closes the path (obstacle, roughness or slope, not within
  the arrival tolerance) is closed in the graph for the attempt, and Dijkstra runs again
  in the same cycle until there is an open path or none. No counters or waits (user: "alla
  fine non fa tutto dijkstra da solo?"). On the real scans, 20 of 533 cruise scans would
  close an edge.
- Switch ids unique over the mission (`_SWITCH_IDS`): a new planner cannot reuse one.
- Not done: the stop-and-go (`cell_attempt`) still refuses an arrival outside the cell
  beyond `ARRIVAL_OUTSIDE_CELL_M`.

## 2026-11-20 (cleanup: comments, switches, one-off tools)

The code says what it does; the history is here. Behaviour unchanged except where said: every
step checked with the tests, and `test_replay --all` printing the same as before. The state
before the cleanup is the tag `switches-before-cleanup-2026-11-20`.

- **Comments**: dates, mission anecdotes and "before it did X" out of the code (they were
  already in this file; the missing 09-11 entry added); no all-caps emphasis; stale names
  fixed (`[EARLY]` for `[SUBITO]`, `[LOCALIZATION]`, the loop of `cell_attempt`).
- **Switches removed, only the branch in use kept**: `low_obstacles.LOW_OBSTACLE_FILTER`,
  `geometry.ROTATION_CHECK_PENETRATION`, `config.REPICK_ON_REPLAN_FAIL`,
  `edge_evaluation.FAST_REFRESH` (the fast helpers take the plain names) and `FRONTIER_EDGES`,
  `motion.PATH_COMMAND_FOR_LEGS`, `edge_evaluation.INCREMENTAL_REFRESH`, `slopes.PLANE_SLOPES`
  (the plane estimate everywhere; gait slope thresholds 0.13 / 0.20; the difference-based slope
  of the global map removed), `goal.GOAL_CENTRE_FIRST`, `mission_prm.PLAN_ALSO_SHARED_NEIGHBOUR_CELLS`.
- **Tests on the behaviour in use**: `test_mission` no longer turns the new behaviours off; the
  whole missions with the old cell rules leave it (they go with the old rules to the local
  variants); the arrival just outside (1,1) is covered by `test_e2e_loop` S14. `test_e2e_loop`:
  the fake robot answers `navigate_to_path`; S15 with graph seeds that give its cases with the
  plane slopes. `test_mission` and `test_cruise` remove their data from /tmp at the end (6.7 GB
  had piled up and filled the disk; `KEEP_TEST_DATA=1` keeps the cruise runs).
- **One-off tools out of the repository** (kept on the PC in the local `varianti/` folder, ignored
  here): `bench_edges`, `bench_refresh`, `compare_cell_rules`, `plane_thresholds`, `slope_check`,
  `slope_percentile_check`, `route_vs_shortest`, `terrain_3d`, `try_path`, `run_legacy_mission`,
  `cruise_timing`.
- **`test_replay --all` and its reference**: the 185 differences from the reference come from the
  0.30 m threshold of 09-11 (`d6699f3`): the reference updates of 06-11, 09-11 and 12-11 refreshed
  only the two default missions. About 0.1-0.5% more edges discarded per cell start, a few paths
  one edge or a few cm different, and on 13-10 11:22 the path to (0,3) (4.8 m, a narrow passage
  the robot walked that day) is gone. To update after the simulator trials (user, 20-11).

## 2026-11-19 evening (the route of the visit in the path; exploration by frontier; simulator plan)

All off by default, to compare on fake missions and in a simulator (the robot was out of order).

- **`roadmap.ROUTE_LINE_PENALTY`** (`ROUTE_LINE_WEIGHT` 1.0, `ROUTE_LINE_SATURATION_M` 1.0): every node entered by the PATH search costs its distance from the route of the visit (polyline through the cell centres in visiting order, `PRM.set_route_line`, set by `exploration.Mission`), saturated so that going round a big obstacle stays possible. Only the path search and its stability check (`compute_path_cost`) use it (`costs_from(..., route=True)`), not the cell rules nor the reachability searches. `tests/test_route_line.py`.
- **Exploration by frontier** (`exploration.GOAL_SOURCE = 'frontier'`, `mission/frontier.py`, new): the goal is the first frontier node (seen, free, with a valid edge towards a node never seen) settled by one Dijkstra search from the robot with the route penalty. `FrontierRules` has the `choose()` of `cell_rules.CellRules`: in cruise the next frontier goal is prepared by the planning thread as the next cell is, and the robot goes on without stopping. The goal is forced on the node through `env._forced_goal` (`goal.find_best_point_in_cell`); the cell rules are off in this mode (cell done when seen, cells closed before moving, paths limited to explored cells) and restored at the end; the first scan by an attempt on the robot's own cell. End: no frontier reachable or 97% of the area seen. `tests/test_frontier.py` (world A of test_mission, in cruise: 94% seen, 28 goals, 25 reached while walking).
- **`tools/compare_route_line.py`** (new): the fake mission of test_mission in cruise, per world and per variant (`cells`, `route`, `frontier`): cells, metres walked, distance of the track from the route, area seen, time.
- Results (`tools/compare_route_line.py`, worlds A and B of test_mission and 4 random of 4 x 5 cells of 2 m, in cruise):
  - *Route penalty*: little effect. Between two cell centres the shortest way is already almost on the line; the distance comes from going round obstacles. Weight 1: off line 0.20 against 0.22 m on the random worlds, 217 m against 207 walked; weight 2: longer detours (world B 45 m against 38). Kept off.
  - *Frontier*: works end to end. With Spot's 3.84 m grid the frontier is ~1 m ahead: goals every 0.5-1 m (22-34 per mission), most of them reached while walking. On the open random worlds it does as well as the cells (204 m against 207, 552 s against 577, area seen 99% against 100%, off line 0.18 against 0.22 m); in the walled worlds of test_mission it is slower (A: 113 s against 75, B: 182 s against 107) for 2-3% more area seen. Meant for a wider sensing (a lidar), to try in a simulator.
- **`SIMULATOR.md`** (new): the four points where a simulator plugs in (local grids, commands and robot state, GraphNav in memory, start-up), as the test fakes do today; steps and what to compare.

## 2026-11-19 (timing of the cruise: switches prepared without the robot, all off)

The robot was out of order: everything here is behind a switch, off, checked offline, to try in the field (FIELD_TEST.md, "Timing of the cruise").

- **What the logs of 04-11 and 09-11 say** (`tools/cruise_timing.py`): the planning thread publishes a plan every 0.4 s, the main thread reads one every ~0.47 s and misses 2-33% of them -- its turn is 0.2 s of sleep plus ~0.25-0.3 s of network requests (command feedback, pose, pose again and the command at every send). The 0.4 s of the planning are ours (`PLAN_PERIOD_S`), not Spot's: the grids came at our request rate, never the same twice, at multiples of 1/15 s. On the CORE I/O the cycle is mostly computation (maps + edges ~0.27 s against ~0.12 s of scan).
- **`attempt_cruise.CRUISE_WAKE_ON_PLAN`**: the main thread waits on `ArcVerificationTracker.wait_plan` (a condition notified at every plan) instead of sleeping `CRUISE_POLL_S`; the feedback is still read at least every 0.2 s. `test_cruise` with fake network latencies: plan read after 0 ms instead of ~115 ms (median).
- **`planning_cycle.PLAN_EVENT_DRIVEN`**: a cycle as soon as the previous one ends (`PLAN_EVENT_MIN_PERIOD_S`); a Spot grid already processed is not processed again (the cycle publishes nothing: `job()` returning None), at most for `PLAN_EVENT_MAX_SKIP_S` so that plans keep coming. Offline timing model on the 09-11 cycles: plan in force ~0.1 s fresher from the laptop, the same on the CORE.
- **`attempt_cruise.CRUISE_STATE_CACHE`** (`robot/state_cache.py`, new): one thread reads the robot state at 20 Hz while somebody asks (lazy: quiet after 2 s without requests) and serves the last reading if younger than 0.12 s, otherwise waits the next one or asks Spot itself; never an old state. Used by the cruise: command feedback pose, paths sent, tilt recorder, fall check, ground height of the planning scans. `tests/test_state_cache.py`; `test_cruise` with latencies: feedback + pose of a turn 65 -> 31 ms. More requests to Spot than the ~15/s of before (20/s), less waiting: `state_cache.PERIOD_S` to tune.
- **`spot_grid.GROUND_FROM_GRID`**: the ground height of the obstacle layer from the grid's own `gpe` frame, when there, instead of a robot state request (the second round trip of every scan); without the frame, the request as before. `tests/test_ground_from_grid.py` on real SDK protos.
- **`roadmap.A_STAR_PATH`**: the path search with A*, heuristic alpha x straight-line distance (no edge of the saved missions, 606 815 checked, costs less than its length: the path is the same). `tests/test_astar.py`: same path in 191/191 searches, 63% less search time. Not to try yet: on `test_replay --all` it breaks equal-cost ties differently (4 attempts with one edge more); Dijkstra stays for now (TODO.md, idea 4).
- **`edge_evaluation.BATCH_ADJACENCY`**: the neighbour lists of the edge refresh rebuilt once per node at the end instead of at every edge (`_refresh_pairs_loop`, `_apply_batch_adjacency`; `_edge_weight` reads the refresh in progress). `tests/test_batch_adjacency.py`: identical graph (order of the lists included) in 89 refreshes, refresh 288 -> 236 ms. The edge refresh is ~70% of the cycle (`tools/profile_cycle.py`, new).
- **Logs**: the `[CRUISE] Plan` line says after how long the plan was read (`read after X ms`); a `[CRUISE] Main thread: turn every …, feedback + pose …, path sent …` line at the end of every cruise; the robot state cache line; the ground height source in the cycle times; the new switches in `[CONFIG] SWITCHES`.
- **Tools**: `tools/grid_rate_check.py` (read only, on the robot standing: how often Spot renews its grids, cost of a grid and of a robot state request, frames carried by the grid), `tools/cruise_timing.py` (timing of the cruise from mission logs, to compare switches), `tools/profile_cycle.py` (the planning cycle function by function, no robot); `tools/route_vs_shortest.py` and `tools/terrain_3d.py` (18-11, for the presentation).

## 2026-11-17 (more checks: tools, legacy runner, repeatability; unreadable terrain)

- **A stretch whose terrain cannot be read gets the most careful gait** (`CruisePlanner._terrain`): an exception there was swallowed, the stretch skipped, i.e. read as flat ground -- the gait could get lighter exactly where the terrain is unknown. Now it counts as above the HARD threshold, said once per planner (`[CRUISE] Terrain of the stretch not readable ...`), counted (`terrain_errors`, in the summary).
- Checked, nothing to change: every tool starts (`--help`) and uses only names that still exist after the moves of 16-11; the log readers do not parse the lines changed on 16-11 (`mission_report` counts `[NO PATH]` too); `tools/preflight.py` consistent; `tools/run_legacy_mission.py --tag legacy-switches-2026-11-11 --set attempt_cruise.CRUISE_SWITCH=False --set planning_cycle.WHOLE_EDGES=False` really runs the code of the tag (`WHOLE_EDGES` exists only there); `test_cruise` three times in a row, all OK (the change of cell while walking depends on thread timing); the counters and the step timer written by both threads are not locked, at worst an increment lost or a `[TIMING]` line mixing two steps, nothing stops; the other silent `except` are diagnostics or optional settings.

## 2026-11-16 (the change of cell while walking in one class)

- **`attempt_cruise.CellSwitch`**: the change of goal or cell while walking, seen from the main thread of a cruise, in one place -- `ask` (the cell is done: the next one asked to the planning thread), `take` (the switch of a plan not applied yet), `blocked` / `late` (a plan while the next cell is prepared: the edge out of the graph stops the robot; beyond `CRUISE_SWITCH_WAIT_S` the hand-off), `at_goal` (the wait standing on the goal, said once), `apply` (new goal or cell for the attempt, or the end of the cruise). Before, the same logic was spread over the loop of `_cruise`: six variables (`switch_pending`, `switch_t0`, `switch_why`, `last_switch_id`, `waiting_logged`, `report_switch`) and four places. Same behaviour (same order: while waiting, the stretch of the plan is sent before the wait is found too long).
- **The `[CRUISE] Plan` line in one function** (`CruiseSteps._log_plan`), also for the plans while the next cell is prepared (`[CRUISE] Plan N (waiting for the next cell): ...`): until now that window, 0.3-3 s, had no line.
- `test_cruise`: `check_cell_switch`, every case of the class with a fake cruise and planner; the cruise scenarios (step C, cell without a reachable goal, cell seen) as before.

## 2026-11-16 (the logs: in the file, the new behaviours visible, the summary of the cruise)

- **Into `mission_log.txt` what went only to the terminal**: the errors of the frontier check while moving (`[FRONTIER] Error ...`, with the traceback; the logger only reached the terminal) and the motors power-on (`[ROBOT]`, was the SDK logger); the start of the frontier/planning thread gets its tag.
- **The stop at the goal while the next cell is prepared is said** (`[CRUISE] At the goal of the cell done, X s after asking the next cell: standing until it is prepared`), once per wait; stat `cruise_switch_waits_at_goal`.
- **The `[CRUISE] Plan` line gives the terrain the gait is chosen from** (`terrain X deg rough Y cm`: worst plane slope along or across and roughness of the first metre of the confirmed stretch), to compare in the log itself with the ground under the feet (gpe in `trajectory.csv`) at the next outdoor mission.
- **The end-of-mission summary has the cruise and the cells**: cruises (metres, time, speed, plans, path changes, deviation), cells chosen near/far, done while walking with the next prepared in the thread (waits at the goal, late beyond the wait), hand-offs, closed because seen (in cruise, at the attempt start, before moving), on the way, attempt starts from the thread / stop-and-go instead / failed (with their cause), cell failures, cruises that did not walk, goals chosen again while walking, ends of the cruises by reason. Until now it listed mostly the stop-and-go counters; these were only in `mission_summary.json`.
- **`[NO PATH]` with the real cause** (`PRM.why_no_path`), instead of `[GRAPH DISCONNECTED]` "the two nodes are in separate components": the path exists but beyond the detour cap, or only through cells not explored (`PLAN_ONLY_EXPLORED_CELLS`), or the graph is really split. `tools/mission_report.py` counts the three tags.

## 2026-11-16 (imports: unused ones out, no import inside functions to dodge cycles)

- **Unused imports removed** (left by the changes of the last days, used by no test nor tool): `config` (`spot_grid`: config is imported by every module, it must not pull a perception module), `exploration` (`geometry`, `goal`, `mission_prm`, `recovery`, `spot_grid`: the side rules), `attempt_scan` (`attempt_cruise`, `motion`, `recovery`: the old start), `run` and `legacy/stop_and_go_start` (`numpy`), `attempt_cruise` (`safe_frontier`), `recovery` (`roadmap`), `goal` (`global_maps`). `spot_grid`: the frame names explicit instead of `from bosdyn.client.frame_helpers import *`, the bosdyn imports in one guarded block.
- **Three cycles removed at the root** (the imports inside functions that dodged them are gone): `CELL_SEEN_EXPLORED` / `CELL_SEEN_FRACTION` move from `cell_attempt` to `attempt_common` (the cruise imported `cell_attempt` inside its functions; `attempt_cruise.cell_attempt_seen_fraction` removed); `save_global_maps` (with `MAPS_SAVE_EVERY_S`) moves from `attempt_scan` to `global_maps`, where the maps are; `prune_prm_nodes_in_obstacles` moves from `safe_frontier` to `roadmap` -- it works on the graph, and it was the only reason the frontier imported the graph, so `edge_evaluation` now imports the frontier at the top.
- **Standard library and bosdyn imports at the top** of their modules instead of inside functions (`time`, `datetime`, `heapq`, `random`, `argparse`, `traceback`, `cKDTree` -- in the goal choice, called often --, `world_object`, `nav_pb2`, `map_processing_pb2`); `matplotlib` stays inside the drawing functions (heavy, only for figures).
- Every module imports alone (no cycle in any order); all suites as before.

## 2026-11-16 (a short timeout on the scan requests of the planning thread)

- **`spot_grid.SCAN_TIMEOUT_S` = 2 s** on every request to Spot of a scan of the planning cycle and of the frontier check (the layers, and the robot state read while decoding them: `fetch_scan` / `return_local_grid(timeout=, max_attempts=)`, `_fetch_layers`, `compute_ground_height_in_vision_frame`). Before: the library default, 30 s, with up to 3 attempts; a stuck request held the cycle, and the main thread waiting for it at an attempt start, that long (09-11 15:32: a scan download of 21 s). A scan arrives in 0.07-0.2 s on the laptop. A request that times out ends the download at once: no other attempt (the next cycle asks again), no fall back to one request per layer (it would wait three times), and it does not count towards giving up the single request for all layers. The cycle ends `no_scan` with the cause (`LocalGrid.last_fetch_problem`, e.g. "no answer from Spot within 2.0 s"), which reaches the log and, for an attempt start that fails, `start_failures`. The stop-and-go scans keep the default.
- `test_e2e_loop` S18: a fake Spot that does not answer -- one request with the short timeout, no fall back, the cause in the plan.

## 2026-11-16 (code review: one function for one job; goal choice with one search)

- **One search on the graph** (`PRM.costs_from`: Dijkstra with the weights of the attempt, nodes allowed, optional stop; `PRM.path_to`): `PRM._dijkstra` (the path), `PRM.reachable_count` and `cell_rules.CellRules._costs` are built on it; before, three copies of the same loop (the third written for the cell rules on 09-11). Same order of the steps: on a graph of 414 nodes with random edges, cells allowed and the detour cap, 1200 paths out of 1200 identical to the old Dijkstra.
- **Goal choice with one search** (`PRM.reachable_from`): `goal.find_best_point_in_cell` asks for every candidate point whether it has a path from the robot (attempt start, goal chosen again, goal chosen while walking); each question was a whole Dijkstra, and with no point reachable (a cell that cannot be entered) one per point of the cell. Now one search from the robot node for all the points, the answer as `find_path_dijkstra` would give it (nodes allowed, detour cap on the path rebuilt from the search; a node outside the allowed ones gets its own search): 1200 answers out of 1200 identical, 4.5x faster on that test. `cell_rules.has_goal` (the same choice made by the cell rules) now applies the detour cap too, as the attempt does.
- **The path of the planner set in one place** (`CruisePlanner.set_path`, before: the constructor, `_start_in_thread`, `_planner_to`), **the edge the robot is walking computed in one place** (`CruisePlanner.robot_edge`, before also `attempt_cruise._robot_edge`), **the planner of a cruise created in one place** (`CruiseSteps._new_planner`, before `_cruise_from_start` and `_cruise`).
- **Empty global maps created in one place** (`global_maps.ensure_global_maps`, used by the attempt start and by `final_scan`, which had a copy since 13-11).
- **The yaw of a quaternion in one place** (`connection.yaw_of`): the formula was written out in 11 places of 10 files.
- **The plane slope sampled in one place** (`slopes.plane_slope_samples`, with `slopes.grid_lookup`): `plane_slope_along` (one edge: the frontier, the tools) is the batch on one edge, and `tools/slope_percentile_check.py` takes its samples from it instead of its own copy of the sampling (same numbers as 13-11).
- **A start in the planning thread that fails is a failure of the cell, with its cause**: without a path from the thread after `START_TRIES` the cell was left as "unreachable now" (skipped once, and if it was the last candidate the mission ended), but that is a fact of the graph, while this may be a slow or stuck computer (09-11 15:32: scan 20 s, graph update 31 s). Now a failure (retried later, set aside after `MAX_FAILURES`); the cause -- cycles in error with the last error, cycles without the scan of Spot, or no cycle ended -- goes into the `[ATTEMPT]` and `[CELL RULES] ... failed (...)` lines, into the summary (`Attempt start failed on cell ...`) and into `mission_summary.json` (`mission_stats.start_failures`).

## 2026-11-16 (code review: side bookkeeping out of the main modules)

- **The side bookkeeping moves into `mission/legacy/side_rules.py`**, the only user: the side methods of `cells.EnvironmentMap` (`mark_cell_side_explored`, `all_sides_explored`, `untried_side_neighbours`, `get_valid_neighbor_sides_mask`, `get_side_bit_facing_origin`, `get_adjacent_frontier_cells`, `get_lowest_rank_from_frontier_list`, `is_cell_blocked`), `cells.find_new_borders`, `recovery.finalize_or_defer_blocked_cell`, `mission_prm.graph_reaches_cell` / `unreached_cell_reason` / `_reach_from`, `RecordingInterface.get_manual_waypoint_by_cell` / `find_nearest_waypoint_cell_to_target`, `config.GRAPHNAV_PRECHECK` (now `side_rules.GRAPHNAV_PRECHECK`). Importing the module adds the methods to the two classes. `EnvironmentMap.explored_sides` and `get_cell_sides_status` stay (data of the saved cell state, 0 with the cell rules).
- **`exploration.Mission` without the frontier list**: with the cell rules the candidates are worked out again at every turn (`CellRules.open_cells`); the list grown by every visit (`self.frontier`, `find_new_borders`), `_last_visited` and `_dropped_cells` were bookkeeping of the sides. `_close_seen_cells(candidates)` takes the open cells (asked again after every cell closed: the cells it opens are checked too); `SideRulesMission` keeps the frontier list, its `_on_visited` and its `_close_seen_cells`.
- **`Mission._chained_visit` removed**: it repeated the waypoint part of `_on_visited`; the cruise calls `_on_visited` for the cells done while walking (marking visited again a cell already visited changes nothing; it also updates the robot cell of the grid, as every other visit).
- All suites as before.

## 2026-11-13 (code review: fixes)

- **While the next cell is prepared the plans still count** (`attempt_cruise._cruise`): after a cell done, the main thread waits up to `CRUISE_SWITCH_WAIT_S` (3 s; usually 0.3-1 s) for the next cell prepared in the planning thread. Until now it skipped every plan meanwhile, with the robot walking the whole last stretch sent: an edge going out of the graph in that window did not stop it. Now the edge being walked out of the graph stops the cruise, and the stretch sent follows the confirmed one of every plan (it may get shorter).
- **Errors of the planning thread in the mission log**: the cycle prints the error with its traceback (`[PLANNING] Error in the planning cycle`, stdout goes into `mission_log.txt`; the logger only reached the terminal), and the wait for the first plan of an attempt writes the plans without data or with an error (`[CRUISE] First plan N: error ...`) instead of skipping them silently (a start that failed read only "no plan within 10 s").
- **`final_scan` with no global maps** (a mission ended before any attempt): the maps are created there, as the first attempt would.
- **`attempt_cruise.CRUISE_HANDOFF_M` 1.0 -> 0.5 m**: the cell counts as done with the robot in it and the goal within 0.5 m (user: the goal must be reached more closely). The 1 m was sized on the hand-off to the next attempt computed by the main thread; with the next cell prepared in the planning thread (~0.3 s) 0.5 m (~1.4 s at 0.35 m/s) is enough. The cell is now done 0.3-0.5 m from the goal (a plan every ~0.35 s, 0.13 m walked between two) and the next cell arrives one or two cycles later: in `test_cruise` step C the robot reached the goal before it and the cruise ended as "arrived", the switch lost. So **arriving at the goal while the next cell is prepared does not end the cruise**: it waits for it (at most `CRUISE_SWITCH_WAIT_S`, then the hand-off as before); at worst a few tenths of a second standing on the goal. Step C again: 2 cells done while walking (0.35 and 0.49 m from the goal), one cruise, 2.3 s still between the cells (the turn on the spot).
- **Bug of 12-11 fixed** (`CellAttempt.run`, the start without the old fallback): a cruise started from the thread that ended at the goal leaves no path, and the start read it as "the planning thread gave no path": the cell reached was left as unreachable. Only a start that did not get going (no cruise) means that now.
- Checked and left: the attempt start updates the edges twice (the rebuild of the areas with new data, then the refresh of the cycle on the same scan). On the laptop the first costs 0.06-0.11 s after the first attempt (09-11 14:36, 14:53), the refresh of the cycle 0.03-0.05 s, and the refresh applies the live rule on the edges of the window that the rebuild does not: not worth removing.

## 2026-11-12 (the 90th percentile along the edge, checked with the plane)

- **`tools/slope_percentile_check.py`**: which summary of the plane slope samples along an edge (mean, p50, p75, p90, max) reads best the ground Spot then walks on (gpe, the ground plane from the legs, from 04-11), on the straight stretches of the trajectories with a scan before them. 12 missions, 186 stretches of 1 m (also 0.6 and 1.25 m: same picture). Against the worst slope under the feet (p90 of the measured, what veto and gait must catch): p90 bias -0.08 deg, MAE 0.54, correlation 0.98 (max the same, mean/p50 -0.65 deg); steep stretches (> 10 deg) caught 12/13 along, 8/10 across, no false alarm (mean/p50 10/13 and 5/10; max 13/13 with 1 false). Against the mean slope (an effort, for the cost) the mean reads a little better (MAE 0.41 against 0.62 deg along). Kept: the 90th percentile, one value for veto, gait and cost. Mostly flat ground (median 0.5 deg): to see again on more slopes. Again on the 13 OUTDOOR missions (body pitch p90 >= 5 deg: 12-10 14:47, 17:16, 17:32; 13-10 12:14, 12:27, 12:40; 22-10 16:04; 26-10 15:11; 28-10 13:50, 14:19, 14:23; 29-10 10:57; 04-11 14:08), `--outdoor --truth body` (the body pitch: only along; on 04-11 14:08 it reads as the legs' ground plane, worst p90 14.6 against 13.7 deg): 297 stretches of 1 m, measured worst median 4.2 deg, p90 14.3, max 22. Against the worst: p90 bias -0.46 deg, MAE 1.22, correlation 0.94 (mean/p50 -1.6 deg, MAE 1.82; max -0.22, 1.22); stretches above 10 deg caught 44/53 (mean/p50 33/53, max 45/53 with 2 false alarms), above 15 deg 17/27 (mean 8/27, max 18/27 with 2 false). On 04-11 14:08 with the legs' ground plane, across: p90 8/10 above 10 deg, mean 5/10. With 0.6 m stretches (809) the same ranking. Against the mean slope the mean is better (MAE 0.92 against 1.35 deg). Kept the 90th percentile: on slopes it reads the worst ground almost without bias, the mean misses a third of the steep stretches; the max adds little (the plane window already smooths) and brings false alarms. The tool also splits the error of p90 by measured slope (outdoors, body pitch): 0-5 deg bias +0.1, MAE 1.0; 5-10 deg -0.9, 1.3; 10-15 deg -0.9, 1.4; above 15 deg -2.4, 2.4 (the estimate reads ~89% of the steepest ground: the plane window of 1.1 m smooths the short steep pieces). On 04-11 14:08 the body pitch and the legs' ground plane agree (4940 samples: mean difference 0.0 deg, MAE 0.7; correlation -0.98, opposite sign convention); the body roll does not measure the ground (under 1 deg with the ground across at 12-17 deg: Spot keeps the body level across).

## 2026-11-12 (the slope of the global map with the plane; the box scenario)

- **The global height map gives the slope with the plane too** (`GlobalTerrainGrid._plane_profiles`, with `slopes.PLANE_SLOPES`): the same estimate as the live scan -- the plane fitted in a window of +/-0.55 m around every cell, along the edge the 90th percentile of the slope along and across it -- on the mean heights of the map. The plane field is kept per fixed tile of 64 x 64 cells (1.92 m, computed with the window margin), recomputed only when a scan changes the map within its margin: a value never depends on which edges are asked together (batch and single edge identical; on the same data map and scan agree to 2e-13). Tried first with a patch around the edges asked: the sums of the integral images grew with the patch and the local graph update differed from the full one by up to 0.002 (test_replay). Until now the map used the differences on a 0.48 m base with the body extension, ~2 deg more on average than the plane (`tools/slope_check.py`): an edge changed cost (and could change veto) only because it left the scan window. `certainly_no_slope_data` with the plane: no seen cell in the edge rectangle widened by the window (integral image of the seen cells, one per map update). `test_replay`: the invariant "global map = live" on the plane (2236/2236 and 1835/1835 edges); "local update = full" compares the weights within 1e-9 (live and map slopes now differ by ~5e-12, rounding both split some); reference updated, 3 differences on 13-10 12:40: two paths shorter (3.07 -> 3.00 m, 2.71 -> 2.56 m), the slopes read lower.
- **`test_cruise`, the box that appears**: now twice. At 1.8 m (its near face; Spot's grid reaches 1.92 m), where it is seen in time: the robot goes around it, the centre never closer than half the body. At 0.8 m, appearing at the last moment: the obstacle filter (median of 3 scans) counts it from its second scan, ~0.6 s, and the robot stops with the nose into the box -- the emergency brake there is Spot's body obstacle avoidance (on in every command; not in the fake robot). Checked: the cruise stops for it, the centre never inside the box, the attempt ends cleanly (in the fake robot, without Spot's avoidance, the body ends against the box, the nodes around are out of the graph and the attempt backs away and leaves the cell for later). At 1.8 m: one cruise without stops, the centre at 0.32-0.34 m from the box. User, 12-11: "se ci fosse bisogno di una frenata di emergenza sarebbe proprio l'anticollisione di Boston Dynamics". Before: one scenario at 1.2 m (face at 1.0 m) with the half-body check, passing or failing by where the box fell on the edges (11-11: 0.13 m).

## 2026-11-12 (the old logic out of the main code)

- **`spot_explorer/mission/legacy/`**: the logic replaced and no longer used in the field, kept runnable (user, 11-11). `side_rules.py`: the rules of the sides tried (`SideRulesMission`: `_explore_sides`, `_attempt_adjacent_cell`, `_retry_after_failure`, `_pick_far_cell`, `_travel_and_attempt`, `_mark_side_failed`, `_start_cell_for`, `_robot_cell`, `_explain_end`, ~370 lines out of `exploration.py`); `stop_and_go_start.py` (was `mission/legacy_stop_and_go_start.py`). `exploration.Mission.explore` is the cell rules only; switch `cell_rules.CELL_RULES` removed (`mission_summary.json`: `exploration.Mission` gives the class, `SideRulesMission` for the old rules).
- **Switches woven into the cruise removed**: `attempt_cruise.CHAIN_CELLS`, `CHAIN_ON_SEEN`, `CRUISE_SWITCH` (with `_cruise_repick` and the main-thread repick branch: the planning thread chooses goal and cell, `_on_no_path`), `THREAD_FIRST_TURN` (with `_can_start_from_thread`), `planning_cycle.WHOLE_EDGES`. A copy of the 300-line cruise for each would only drift: their old behaviour runs from the git tag `legacy-switches-2026-11-11` (094abb7, the code of 11-11 evening).
- **`tools/run_legacy_mission.py`**: `--side-rules`, `--stop-and-go` (cruise off) on the code of today; `--tag legacy-switches-2026-11-11 --set attempt_cruise.CRUISE_SWITCH=False` on a git worktree of the tag in `../legacy_worktrees/`. Everything else goes to `run_mission.py`.
- Kept as they are: the switches with the same results used by the benches (`BATCH_SLOPES`, `REFRESH_BATCH_SLOPES`, `INCREMENTAL_REFRESH`, `FAST_REFRESH`, `FRONTIER_EDGES`: `tools/bench_edges.py`, `bench_refresh.py`), the features and parameters in use (`PLANE_SLOPES`, `LOW_OBSTACLE_FILTER`, `SEEN_BEFORE_MOVING`, `CELL_SEEN_EXPLORED`, `GOAL_CENTRE_FIRST`, `PATH_COMMAND_FOR_LEGS`, ...), the stop-and-go mode (`CONTINUOUS_MOTION`, also the exception handler of the cruise), the side bits of `cells.EnvironmentMap` (data of the map, used by `side_rules.py`).
- Tests: `test_mission` runs the old rules through `SideRulesMission`, as the runner does; all suites as before (`test_cruise`: the box scenario still open).

## 2026-11-11 (attempt start in the planning thread; edge costs and cell choice on Dijkstra)

- **The stop-and-go start out of the main code** (`mission/legacy/stop_and_go_start.py`, mixin `StopAndGoStart`: `_prepare`, `_first_scan`, `_chained_scan`, `CHAIN_REUSE_SCAN`): used only in the stop-and-go mode (cruise off: `test_e2e_loop`, most of `test_mission`), still runnable. With the cruise on there is no fallback to it any more: without a path from the planning thread within `START_PLAN_WAIT_S` the start is tried again (`START_TRIES` = 2), then the cell is left as unreachable now (skipped once, not a failure). The cruise no longer leaves its last scan for the next attempt (`env._chain_scan`): the thread scans by itself. Switch `START_IN_THREAD` removed.
- **The GraphNav waypoint of a cell done while walking after the first command towards the next cell** (`_apply_switch` -> `_visit_after_send`), at the robot position of that moment: before, the call (~0.8 s) came first and the robot could finish the old command and stop meanwhile. `test_cruise` step C: first command towards the next cell 0.00 s after the switch is applied (before 1.44 s), still between the cells only the turn on the spot.
- **`cell_rules.EXPLORE_CELL_ON_THE_WAY`** (off): a near choice whose way to the goal crosses a cell not explored yet takes that cell first, if it has a goal within `NEAR_PATH_M`. Meant with `config.PLAN_ONLY_EXPLORED_CELLS = False` (paths also through the cells never visited; with it on only the shared neighbours can be crossed). User, 11-11: "se si passa da una cella non esplorata avrebbe senso fargli esplorare direttamente quello". `test_mission`: a check of the choice (on: the cell crossed; off: the visiting order) and world A in cruise with both switches flipped (8/8 cells; the rule never triggered there).
- **The attempt start in the planning thread**: the whole attempt start -- scan, cell already seen, local data into the graph and edge update, robot node, goal, Dijkstra path, attempt counters, maps on disk -- is the first cycle of the planning thread (`_start_in_thread`, run as the `switch_job` of that cycle, kind `start`/`seen`/`nogoal`); the main thread only gets up if fallen, creates the global maps at the first attempt (`_ensure_global_maps`) and starts the planner (`_cruise_from_start`, wait `START_PLAN_WAIT_S`). One place for scan and graph, as the user asked ("non ha senso che il primo giro lo faccia il thread principale"). A cell found already seen, or without a goal, at that first cycle sends no command: the robot keeps walking if it was (hand-off), as before. If its first plan does not allow to start (stretch too short, rotation not clear) the stop-and-go loop takes the turn on the path the thread chose (the exceptions). `test_cruise`: every attempt started this way (attempt start ~1-2 s, in the thread).
- **An edge never seen costs its length only** (`edge_evaluation`: the slope sources are the live scan, then the global height map; no third source). Until now the fallback was the mean slope of the two nodes, read only at the attempt start at the node point (also from invalid cells) and meaningless for the stretch in between that nobody has seen; the node loop in `attempt_scan._local_data_into_graph` is gone. User: "finché non sono visti dal robot gli archi hanno solo la lunghezza dell'arco come costo".
- **Cell choice on Dijkstra costs** (`cell_rules`): the way into a cell and the near/far rule (`NEAR_PATH_M`, now a cost: metres on flat ground) use the weights of the attempt (`roadmap.effective_weight`: length, slopes seen, penalties) up to the goal the attempt would choose (`has_goal` returns its cost), not metres to the first node of the cell plus a straight line to the centre.

## 2026-11-11 (a whole mission in cruise in the tests)

- **`test_mission`, world A IN CRUISE** (`run(cruise=True)`: the real-time fake Spot of `test_cruise` as command and state client, its frontier thread with the planning mode): the mission as in the field since 11-11 -- cell rules, the next cell prepared in the planning thread, the first turn of an attempt from the thread, whole edges -- from the start to the return to wp_0. Until now `test_mission` and `test_e2e_loop` ran with the cruise off (stop-and-go): the failures of 09-11 14:30/14:36 (switch lost, GraphNav trip to a cell without goal) were seen only on the robot. Result: 8/8 reachable cells visited in 83 s with 2 cruises in all (from the start, and after the one GraphNav trip to (1,0)), 6 cells done while walking, the closed cell (2,2) left while walking as unreachable, no fallback to the stop-and-go at the start of an attempt.
- `cell_rules.CellRules.choose`: the cells excluded at a choice (left unreachable just before) get their reason in the `[END]` summary ("unreachable: no goal with a path when last tried"; before: "never next to a visited cell").

## 2026-11-11 (the first turn of an attempt in the planning thread)

- **`attempt_cruise.THREAD_FIRST_TURN`** (on): after the attempt start (scan, graph, goal, path) the cruise starts at once and its first command comes from the first plan of the planning thread (`_first_plan`, `_cruise(from_thread=True)`), the same cycle as every other one; before, the main thread made a turn of the stop-and-go first (fresh scan, maps, refresh, shortcut, frontier, decision, rotation check) with its own code, and only then started the cruise. The checks of that turn stay, on the plan: a confirmed stretch of at least `CRUISE_MIN_AHEAD_M`, the rotation towards the first node clear with the body (`decisions.rotation_plan`, standing) or with `CHAIN_TURN_EXTRA_AIR_M` more air while walking; a goal or cell changed by the thread in that first cycle is applied. If the first plan does not allow to start (no plan within `FIRST_PLAN_WAIT_S`, no path, stretch too short, rotation not clear) the planning stops and the stop-and-go takes the turn: the exceptions. Stats `thread_first_turns`, `thread_first_turn_fallbacks`. `test_cruise`: every attempt of every scenario started from the thread, no fallback; the stop-and-go turns left come only after a cruise ended (rotation not clear, end of data, obstacle). `test_e2e_loop` and `test_mission` run with the cruise off (stop-and-go), unchanged.

## 2026-11-11 (an edge is walked only if confirmed free whole)

- **`planning_cycle.WHOLE_EDGES` ON, with the nose only at the goal** (`walk_advance`, used by the planning cycle and by the first send of the cruise): the confirmed stretch ends at the last node reached by edges confirmed free whole on the scan of the cycle (the first point not free or not seen along the path is beyond it), not half a body + margin before that point. The 0.65 m of the nose stay where the robot stops for good: on the edge to the goal (as far as the frontier allows with the nose, if beyond the goal it is not seen yet) and on the edge the robot is already on when it is not confirmed whole (an obstacle appearing). User's point: the padding of the edge (0.30 m) already keeps it away from obstacles, the extra piece beyond a node is needed only to stop there. If the robot ever stops on an intermediate node: 0.30 m around it from the edge, sharp corners checked with the whole body (`_turns_within`), Spot's body obstacle avoidance on. `test_cruise` with edges up to 1.25 m: open field and wall 1 cruise, no stop for end of data; step C 1 cruise (before 10-11's first version: 5-6 cruises, 4 stops); all suites OK. Not tried on the robot.

## 2026-11-10 (where the robot stops: the ends of its edge)

- **After a cruise the robot node is an end of the edge it is on** (`attempt_cruise._stop_on_edge`): the node ahead if that edge is still in the graph (or the robot is on the node), otherwise the node it came from (the part just walked is free). Before: the nearest node with edges (`add_stop_node`), 0.20 m away median and up to 0.31 m on 09-11 14:36 (16 times), also behind or beside the robot, with a first stretch robot -> node that is not an edge. Why the robot stops halfway along an edge at all: the frontier confirms the path up to the first point not free or not seen, minus half a body; an edge seen only in part is walked while the rest comes into view, and when the cruise ends there (obstacle appearing, data not arriving, command expired) the robot is in the middle of it.
- **`planning_cycle.WHOLE_EDGES`** (OFF): the confirmed stretch cut back to the last node reached by edges confirmed whole (user's rule, asked by Marco on 08-10). Off because with it the robot stops at almost every node: to stop on a node the nose (0.65 m) must be confirmed beyond it, so a whole edge of up to 1.25 m needs data up to 1.9 m beyond the node ahead, at the limit of Spot's grid (1.92 m around the robot). `test_cruise`: "step C" from 1 cruise to 6 (6 s still), "three cells" from 3-4 to 7 (10 s still), most ends "the confirmed stretch ends 0.05-0.10 m ahead (end_of_data)". To decide with the user: shorter edges, or partial edges while walking with stops only on nodes.

## 2026-11-09 (missions from the CORE I/O; cruises that do not walk; goals on the cell border)

- **Missions from the CORE I/O without a laptop** (`tools/core_run.sh mission`): the mission keeps its own E-Stop on the CORE (it stops the robot within 9 s if the program dies) and the person following the robot stops it from the tablet by taking control. Checked on the robot: the Spot tablet app never registers an SDK E-Stop endpoint (0 endpoints with the tablet holding the lease and the motors on). `--no-estop` only with `tools/estop_laptop.py`.
- **The E-Stop of the mission is removed at the end** (`SimpleEstop.release`, robot sitting and motors off): before, it stayed registered with nobody answering and the tablet could not power the robot on. `tools/core_check.py --clear-estop` does the same by hand after a mission that died (motors off only).
- **`core_run.sh mission` returns at once**: only the mission goes in the background (`{ nohup ... & }`); a bare `&` took the whole `&&` chain with it, and that chain kept the SSH connection open until the mission ended. The shared SSH connection is opened detached from the script output.
- **Three cruises in a row under 0.10 m end the attempt** (`attempt_cruise.CRUISE_STALL_M`, `MAX_CRUISE_STALLS`). 09-11 14:53: the goal of (0,5) was 9 cm from a wall, no other goal had a path, the plan went back through a node behind the robot (the frontier confirmed 1.26 m of it) and the cruise found the robot already on the blocked last edge (0.00 m ahead): 61 cruises of 2-5 cm in 90 s, every other net reset by the small moves. On all recorded missions it would fire only there and on 04-11 11:39 (same pattern). Ending the attempt as soon as the goal is excluded with no alternative was tried and dropped: walking on often brings the path back (world C of test_mission, old rules: 9/9 cells closed against 4/9).
- **Goals on the cell border after all the others** (`goal.find_best_point_in_cell`): before, the border decided only within a ring of distance from the centre. A border goal leaves the robot in front of it, often outside the cell, where the arrival does not count (`ARRIVAL_OUTSIDE_CELL_M` assumes a goal 0.30 m inside). October-November missions: 28% of border goals ended in an arrival against ~40% of the others; 8 of the 11 goals refused by the frontier were on the border.
- Obstacle stops of the missions of 09-11: 25 of 37 in the zone of the nose beyond the goal, all with the robot within the arrival tolerance anyway; the graph and the frontier did not disagree.

## 2026-11-09 (the edges of the whole scan window updated at every cycle)

- **`edge_evaluation.REFRESH_RADIUS_M` 1.25 -> None** (user's rule: the edges updated are all those the robot sees): weights, slopes and occupancy of every edge with an end in the scan window at every planning cycle, with the incremental reuse of the profiles already computed. Before, only within 1.25 m of the robot node; beyond, up to the border of the window, the edges were only discarded when blocked. `tools/bench_edges.py --b-frontier-edges --b-radius window` on the 392 cruise cycles of 04-11 and 09-11 14:36 (laptop): 36.4 -> 53.8 ms median per cycle, p90 47 -> 72 (on the CORE I/O ~6 times: ~+100 ms per cycle); the frontier never stops on an edge the graph holds valid; edges free for the frontier but out of the graph 3 -> 2 of 305. `test_cruise`, `test_mission`, `test_e2e_loop`, `test_replay`: all OK.

## 2026-11-09 (the lost path handled by the planning thread, without stopping)

- **A new goal in the same cell, chosen by the planning thread** (`attempt_cruise._on_no_path`, `CruisePlanner.no_path_job`, with `CRUISE_SWITCH`): when the path from the node the robot walks to has gone and from there the graph goes on (`REPLAN_FAIL_ROBOT_FREE_NODES`), the thread excludes the goal for the mission and chooses another one in the cell (up to `MAX_BLOCK_REPLANS` per cell, `GOAL_REPICK_CANDIDATES` points each), in the same cycle; the main thread only applies it (`_apply_switch`, kind 'goal'). Before: the main thread paused the planning, chose and started a new planner (`_cruise_repick`, still there with `CRUISE_SWITCH` off). `test_cruise` "goal inside a box": two new goals, 0.07-0.08 s each in the thread.
- **No goal with a path in the cell: the next cell, without stopping, and the cell is NOT failed** (rule 3 of the cell rules: not a candidate now). The thread chooses the next cell (`_prepare_next_cell` with the cell left unmarked and excluded), the cruise goes on; log `[CRUISE] Cell (r,c) left while walking (...: not failed, unreachable now)`. If no next cell can be prepared in cruise the cruise ends and the mission skips the cell at the next choice (`CellRules.skip_once`) instead of counting a failure. Before: the robot stopped and the cell counted towards the 2 failures. `test_cruise` "cell without a reachable goal": a box fills (0,1), the cell is left and the same cruise goes on into (1,1) and (1,0).
- A cruise ended because the robot itself is shut in (from the next node the graph reaches fewer than `REPLAN_FAIL_ROBOT_FREE_NODES` nodes) still ends with a stop and counts as a failure: there the place is the problem, not the goal.

## 2026-11-09 (fixes after the missions of 14:30 and 14:36)

- **The next cell prepared in the thread was lost** (14:30 and 14:36, once each): the tracker publishes only the last plan, and the plan with the switch was overwritten by the next one before the main thread read it; the main thread kept waiting for it, sent nothing, and the robot stood 17 s until the command expired. Now the result stays in every plan until applied (`CruisePlanner.switch_info`, applied once by its id), and after `CRUISE_SWITCH_WAIT_S` = 3 s without it the cruise ends with the hand-off as before (stat `cruise_switch_timeouts`). Right after a switch, a plan computed a moment before (the robot has walked on: 31 cm in one run of `test_cruise`) is skipped instead of ending the cruise.
- **A cell is a candidate only with a goal the attempt would accept** (`CellRules.has_goal`: `goal.find_best_point_in_cell` on the global map, `quiet`, `require_reachable`), checked from the robot if near and from the GraphNav waypoint the trip would go to if far (the choice keeps that waypoint, `Choice.waypoint`). 14:38: the rules saw a way into (1,5) (a node of the cell), the robot went there with GraphNav (22 s) and found no goal reachable. Not verifiable on the replay (the saved scans are not all the mission's: the graph rebuilt differs).
- `tools/compare_cell_rules.py` reads the `[CELL RULES] ... failed` lines of the log: on 14:36 the rules choose the attempted cell at all 5 attempt starts.
- Map download at the end of 14:36: 196 s (5-17 s the other missions from the laptop, 0.7 s from the CORE); the E-Stop errors of the SDK ("RPC took longer than 9.00 seconds", "IncorrectChallengeResponseError") all fall in those 3 minutes, robot already sitting with the motors off: WiFi saturated by the snapshots. During the motion the same would stop the robot (E-Stop timeout 9 s).

## 2026-11-09 (step C: the next cell prepared in the planning thread, one cruise through the cells)

- **`attempt_cruise.CRUISE_SWITCH`** (on, with the cell rules): when the target cell is done (seen 90%, or the goal within `CRUISE_HANDOFF_M`, or the arrival by tolerance) the cruise no longer ends. The main thread sends the whole confirmed stretch (the robot walks it) and asks the planning thread (`CruisePlanner.switch_job`) for the next cell; at its next cycle, on its fresh scan, the thread marks the cell done, chooses the next one with the cell rules (only within `NEAR_PATH_M`), its goal (`goal.find_best_point_in_cell`, reachable from the node the robot walks to) and the path, and goes on planning towards it, keeping the edge being walked (`_prepare_next_cell`). The main thread then makes the GraphNav waypoint of the cell done (`Mission._chained_visit`), moves the attempt onto the new cell (`_apply_switch`) and sends the stretches of the new plans. When the next cell cannot be prepared in cruise (far: GraphNav; none; no goal with a path) the cruise ends with the hand-off as before. Log `[CRUISE] Cell (r,c) done while walking (...): next cell ..., prepared in the planning thread in X s`, `[TIMING] next cell prepared in the planning thread -> first command towards it`, stat `cruise_switches`. The mission counts the outcome of an attempt on the cell the cruise ended in (`env._chain_choice`).
- `test_cruise`, "step C: one cruise through three cells" (real cell grid and cell rules, one attempt on (0,1)): one cruise through (0,1), (1,1), (1,0); the next cell prepared in 0.2-0.4 s in the thread, first command towards it 0.6-1.0 s after the request, with the robot still walking; the only pauses are the 90-degree turns on the spot towards the next cell (~2 s each). Before: one cruise and one attempt start per cell (`test_cruise` "three cells": hand-offs with the next attempt computed from the main thread).

## 2026-11-09 (general cell rules instead of the sides tried)

- **`mission/cell_rules.py`** (new, switch `CELL_RULES`, on; `Mission._explore_rules`; the old loop stays as `_explore_sides`). A cell is done (seen 90% or goal reached), blocked (every candidate point inside a seen obstacle), set aside, candidate (the graph of now has a way from the robot into it, through the cells the attempt may use) or unreachable (no way). The next cell: among the candidates with a way of at most `NEAR_PATH_M` = 4 m, the first in the visiting order, in cruise; otherwise the first in the visiting order among all, with GraphNav to the waypoint nearest to where the way enters the cell. A real failure (the attempt failed, GraphNav did not get there, fall on the trip) counts against the cell with its ENTRY = the cell the way comes from (one of 8; None = robot inside); a failed cell comes after all the others; after `MAX_FAILURES` = 2 it is set aside while the way of now enters where it already failed. No candidate = the end; `[END]` lists the cells left as set aside / unreachable / never next to a visited cell (`mission_summary.json`, `cells_left`). At the mission start (graph without edges yet) the first cell next to the robot is attempted from where it is. Log `[CELL RULES]`. User's decisions (09-11): visiting order first, cruise within ~4 m, 2 failures.
- Out of the new loop: sides tried, deferral, SKIP-ROUND, PRECHECK, cells dropped from the frontier, the retry right after a failure, the approach from the neighbour with a free side, "robot just outside the visited cell" (on 03-04-11: 242 SKIP-ROUND, 102 sides marked, 99 deferrals, 69 cells dropped, 3 blocked, for 208 attempts).
- First version of the entry was the edge into the cell: in `test_mission` world C one cell failed 3 times by 3 edges from the same side; and a failed cell was retried at once from the same side (on 04-11 14:08 (0,5) failed in the tall grass and worked 17 attempts later from another side): hence the cell as entry and the failed cells at the back.
- `tools/compare_cell_rules.py` (new): at every attempt start of a real mission, the choice of the rules against the cell really attempted (graph rebuilt from the scans, visited cells from the log; failures not replayed). 04-11 16:52: 5 of 7 the same, and the rules never go to (0,5) beyond the wall (no way: 14.6 s still in the mission); 16:10: 7 of 11, (0,5) avoided as well; 14:08 outdoors: 45 of 63 the same, never a cell without a way.
- `test_mission`: worlds A, B, C with both rule sets; with the general rules the closed cell (2,2) ends unreachable (not blocked: its inside is free), 8/9 cells closed; world C with the general rules checks failures, set aside and the summary of the end.

## 2026-11-09 (one threshold: 0.30 m)

- **One threshold for frontier, edges, filter, nodes and goals: 0.30 m from the centre line** (`geometry.ROBOT_CLEARANCE_AIR_M` 0 -> 0.05, `PLAN_CLEARANCE_EXTRA_M` 0.03 -> 0; `FRONTIER_EDGES_CLEARANCE_M` = `FRONTIER_CLEARANCE_M`; margin on the global map 0.13 -> 0.15; `TARGET_CLEARANCE_M` = threshold + 0.08 = 0.38). User's choice, after trying 0.28 for the graph and 0.25 for the frontier (two thresholds = a band where they disagree). 5 cm of air cover what was measured: robot off the line in cruise up to 4.4 cm, cells of 3 cm. The emergency stop during a leg of the stop-and-go stays at half the body (0.25, `FRONTIER_ABORT_AIR_M`), the rotation check has its own margin (unchanged). The rule "edge being walked out of the graph, the cruise goes on while the frontier confirms it" (same day) removed: with one threshold it is not needed.
- On the 463 real frontiers of 03-04-11 the confirmed stretch is shorter than with 0.25 in 19% of the cycles, by 7 cm median; 8 cycles would become a stop. `bench_edges --b-frontier-edges` on the 349 cruise cycles of 04-11: the frontier stops on an edge the graph holds valid 2 -> 0; edges free for the frontier but out of the graph 16 -> 3 (2 because the graph checks an edge from both ends, the frontier only in the direction of travel).
- **Cost: narrow passages.** Corridors on 12 random graphs of 5 nodes/m2 (e2e world): 1.0 m 12/12 with 0.25 and 0.30; 0.9 m 11/12 and 11/12; **0.8 m 9/12 with 0.25, 5/12 with 0.30**: the centre of the robot has a band of 0.20 m in a 0.8 m door and often no node falls in it. Next: densify the graph in the narrow passages (user's decision: keep 0.30 and fix it in the graph).
- Tests: `test_e2e_loop` S16 (0.9 m corridor) with 5 nodes/m2 as in the mission (4 with seed 1 had no way); `test_goal_selection` synthetic check with the obstacle between the minimum and the preferred space (on the replayed attempt starts of 22-10: one without goal, 0 before); `test_replay` reference updated (invariants all OK).

## 2026-11-09 (one rule for the graph and the safe frontier)

User's point: the area the scan sees at every cycle is where the edges are re-evaluated and also where the safe frontier lets the robot walk: one thing, not two.

- **`safe_frontier.sample_flags`**: THE rule "can the robot pass here now", point by point on the scan of the cycle (outside the window or never seen; obstacle_distance below the threshold or a hysteresis cell of the filter, except under the body; roughness veto, except next to the robot). `compute_safe_frontier` uses it: identical to before on 430 real frontiers of 04-11 (429 also equal to the one computed in the mission; the other is from 14:08, still with the old slope estimate).
- **The edges of the scan window use the same rule** (`edge_evaluation.FRONTIER_EDGES`, `set_live_snapshot`, `_frontier_blocked_xy`): same samples as the frontier (one per cell, from both ends), obstacle with the hysteresis and roughness; before, only obstacle_distance every 10 cm. Threshold for the graph `FRONTIER_EDGES_CLEARANCE_M` = `PLAN_CLEARANCE_M` (0.28), 3 cm more than the frontier (0.25) for the tracking: with 0.25 for both, `test_cruise` "box appears" had the robot centre 0.24 m from the box (now 0.31). The exemptions of the rule exist only within ~0.61 m of the robot, so only there the full rule per sample: identical to it on 37 600 random edges of real snapshots.
- **Where the scan sees, the scan decides; the global map only where it does not see** (`_global_occupancy_blocked_many`). The spots of a fall stay blocking everywhere (`_hazard_hits`: the terrain there looks normal to the sensors). Measured with `tools/bench_edges.py --b-frontier-edges` on the 349 cruise cycles of 04-11: edges that the frontier confirms free but the graph held out 14 of 279 (13 from the global map) -> 3 (the band 0.25-0.28 m); the frontier stopping on an edge the graph holds valid: 0 before and after.
- **The edge being walked out of the graph no longer ends the cruise while the frontier confirms the robot up to its end** (`planning_cycle.CruisePlanner`, stat `cruise_edge_out_walked`): the graph chooses the way, the frontier says whether to walk. Before: 04-11 16:52, cruise ended after 0.30 m, 1.8 s still.
- Cost: refresh of the cruise ~+11 ms per cycle on the laptop compared with `FAST_REFRESH` alone (the samples every 3 cm instead of 10).
- `test_e2e_loop` S15: the goal 0.25 m from the obstacle is now replaced as soon as the scan sees the obstacle (goal isolated in the graph), before the frontier stops beyond it: the two checks accept both ways (same purpose: new goal at once, never back to the excluded one).

## 2026-11-06 (a cell is done when it is seen)

- **The cruise hands off as soon as the target cell is seen for 90%** (`attempt_cruise.CHAIN_ON_SEEN`, with `cell_attempt.CELL_SEEN_EXPLORED` and `CELL_SEEN_FRACTION`): the planning thread computes at every cycle how much of the cell the global map has seen (`CruisePlanner.seen_cell`, plan `cell_seen`, in the `[CRUISE] Plan` lines as `cell seen N%`), and at the threshold the cell is done wherever the robot is; the arrival at the goal keeps counting as before. User's rule: the cell exists to be seen, the goal is only where to see it from. Stat `cruise_seen_handoffs`.
- **A cell visited from outside gets no GraphNav waypoint of its own** (`Mission._on_visited`): waypoint and robot cell of the grid only if the robot is in the cell or within `ARRIVAL_OUTSIDE_CELL_M`; before, the waypoint was made where the robot was, labelled with the cell, and `update_position` marked the cell the robot was in as visited.
- `test_cruise`: scenario "open field, two cells: the cell is done when seen" (threshold 50% there: cells of 5 m, scan window 3.84 m). The "box appears" scenario fails in about half of the runs, also before today's changes (first run of the morning): to look at.
- `tools/bench_edges.py --b-radius window`: the refresh over the whole scan window instead of 1.25 m costs 35.4 ms median on the laptop (19.6 with the radius), on the 349 cycles of 04-11.

## 2026-11-06 (continuous motion, step B: no scan at the start of a chained attempt)

- **The attempt that starts with the robot still walking reuses the last scan of the cruise** (`attempt_cruise.CHAIN_REUSE_SCAN`, `CHAIN_SCAN_MAX_AGE_S` = 2 s, `attempt_scan._chained_scan`): already processed and in the global maps, with the robot pose of now; no download, processing and folding of a new scan. It serves the graph update and the goal choice; the first move still waits for the fresh scan and frontier of the first turn. Log `[ATTEMPT] Scan of the cruise reused`, stat `chained_scans_reused`. Why: on 04-11 16:52 (CORE I/O) that scan cost ~0.5-1 s of the 4.1-4.7 s hand-offs, on data 0.4 s newer than those the cruise had just put in the maps. First step towards the motion without stops (B: no work done twice; C: the next cell prepared in the planning thread while walking). In `test_cruise` the biggest piece of the hand-off is now the graph update of the attempt start (0.75-1.4 s there, ~0.4 s on the CORE).

## 2026-11-06 (edge refresh of the cruise: fixed costs cut, same result)

- **`edge_evaluation.FAST_REFRESH`** (on): the refresh of the cruise (phase `edges` of the planning cycle: on 04-11 16:52 on the CORE I/O 208 ms per cycle, 33 ms on the laptop) did at every cycle work that does not depend on the new scan. Now: the global slope cache keeps an index of its edges (`_SlopeCache`: every edge indexed once, when it enters, instead of the arrays of all the keys rebuilt at every cycle: `_sync_global_slope_cache` 8.0 -> 0.4 ms); the node positions and the neighbours of every node computed once (`_node_arrays`, `_refresh_pairs_fast`; the nodes never move, `PRM.nodes_version` when one is added: `_refresh_nodes` 2.6 -> 0.1, `_refresh_pairs` 1.3 -> 0.2 ms); the live discard of the valid edges outside the refresh radius picks the nodes near the window on the arrays and computes lengths and samples in one go (`_live_prune_fast`, 3.4 -> 1.8 ms). Refresh 32.2 -> 19.6 ms median on the laptop (p90 40.4 -> 26.6), on 349 cruise cycles of three real missions (04-11 16:52, 16:10, 14:08).
- **Same result, checked to the bit:** `tools/bench_edges.py` (new) replays the scans of a mission into two graphs, old computation and new, and after every cruise cycle compares edges (weights and order), validity, stored live evaluations, global slope cache, veto statistics and counters: 0 differences in 349 cycles. `test_replay`: invariants all OK (local update identical to the full build 22/22); its reference was already out of date before this change (the same 56 differences with `FAST_REFRESH` off: the behaviour changes of 04-11).
- `tests/replay_reference.json` updated (separate commit): the 56 differences are those of the plane slopes and their gait thresholds of 04-11 (more valid edges and slightly shorter paths; on 13-10 12:14 gaits PLAIN 27 -> 17, MODERATE 12 -> 19, HARD 7 -> 10).
- What remains of the refresh is per-edge work on the new scan (slopes of the new edges and the plane field of the scan 5.6, coverage and reuse 3.5, occupancy 2.6, global map without data 2.7, edge by edge 2.0 ms): with the robot walking the scan window moves at every cycle, so almost every edge of the radius gets new data.

## 2026-11-06 (hand-off times measured step by step; global maps saved less often)

- **`[TIMING] hand-off` / `[TIMING] attempt start` line** (`records.StepTimer`, `records.HANDOFF`): from the end of a cruise (or the start of an attempt with the robot standing) to the first command of the next move, every step with its time: end of the attempt, waypoint of the visited cell, progress and seen cells, choice of the next cell, scan download, scan processing, global maps, maps on disk, seen check, scan into the PRM, graph update, allowed cells, robot node, goal choice, breadcrumb, goal node and path, figure data, planning turn, cruise setup. An attempt that fails before moving stays in the same line (the robot waits for all of it); a GraphNav trip closes it. Sums per step in `mission_summary.json`, `handoff_steps_s`. Why: on 04-11 16:52 (CORE I/O) the hand-offs took 4.1-4.7 s against 0.9-1.6 s from the laptop, and between the initial scan and the graph update 1-2 s were explained by no log line.
- **Global maps on disk at most every 60 s, and always at the end** (`attempt_scan.save_global_maps`, `MAPS_SAVE_EVERY_S`): the `.pkl` are read only after the mission (`mission_viewer.py`); before they were written at every attempt start (~50 ms on the laptop with the maps of 16:52, several times that on the CORE, growing with the mission) and what the robot saw after the last attempt start (last cruise, final scan) was never in them. Now also after the final scan and on an interrupted mission (`[EMERGENCY]`).

## 2026-11-06 (cells already seen: closed before moving)

- **Frontier cells already seen are visited before moving** (`exploration.SEEN_BEFORE_MOVING`, `Mission._close_seen_cells`, with `cell_attempt.CELL_SEEN_EXPLORED`): at every turn of the mission, before choosing where to go, a frontier cell with at least 90% of it in the global map becomes visited where it is, in rank order (the neighbours it adds are checked too). GraphNav waypoint of the cell only if the robot is in it (or within `ARRIVAL_OUTSIDE_CELL_M`), as after an attempt; a cell seen from afar gets none (its neighbours are approached from the nearest waypoint, as for a blocked cell). Log `[SEEN] ... before moving there`, stat `cells_seen_before_moving`. Why: on 04-11 16:52 the robot, standing in (1,3) and in (1,0), went with GraphNav to the waypoint of a neighbouring cell (3.1 m in 9 s, 4 s) and only there, at the first scan of the attempt, found them 93% and 100% seen. On the final map of that mission (1,3) is 94%, (1,0) 99.8%.
- `GlobalGrid.known_many`: `is_known` on many points from the state layer of the raster; `cells.cell_seen_fraction` uses it (same values as the dictionary on the 64 cells of 16:52, 0.13 instead of 0.33 ms per cell).
- `test_mission`: check of the case of 16:52 (robot in a seen cell: visited with its waypoint; cell seen from afar: visited without; cell half seen: stays).

## 2026-11-06 (starting cell chosen)

- **Starting cell chosen** (`config.MISSION_START_POSITION`, `MISSION_VISIT_ORDER`, arguments `--start` and `--order` of `run_mission.py`, `cells.start_cell_for`, `generate_visit_path`; ported from `AdL-version` 4fa37ae of 04-11, old code). Before, the robot always started in cell (0,0), the bottom-right corner: the grid opened only ahead and to the left, and the robot had to be put in a corner of the area. Now it can start in any of the four corners, in the centre, mid-side or in a cell given as `row,column`. Graph, cells, waypoints and viewers already used `env.start_cell`: only where it is fixed changes. The visiting order follows the start: serpentine from the robot's corner (from (0,0) the same as before), rings around the start if it is not a corner (the "rotated" serpentine of before, from the centre, explored half the area and then jumped to the other end). The figure data save `env_start_cell` (the viewer's mission frame is centred on it; older missions (0,0)); the replay reads the starting cell from the log (`Starting cell (r, c)`; old missions (0,0)). Tests: `test_mission`, checks on positions and orders and world H (open area, start in the centre: 9/9 cells visited, also those behind the robot). Never tried on the robot.

## 2026-11-04 (first mission run on the CORE I/O, 16:52, same room)

The mission ran on board (Jetson Xavier NX of the CORE I/O, Python 3.8) from start to the return to wp_0 (4 cm of drift), 8 x 8 cells, 7 visited and 1 blocked, the rest beyond the walls. The robot stood still 52 s out of 136 (16:10 from the laptop: 25 out of 139): hand-offs 4.1-4.7 s against 0.9-1.6 s; cycle phases scan x2, maps x2, edges x6 the laptop (0.21 s median), from the attempt start to the graph update 1.2-2.2 s against 0.25 s. Two GraphNav trips (9 s and 4 s) went to cells where the robot already stood and that turned out 93% and 100% seen; GraphNav STUCK towards (0,2) in front of a chair with a jacket moved into the corridor (photos).

- **Tools for the CORE I/O** (`tools/core_setup.sh` once: aarch64/cp38 wheels downloaded on the laptop, venv `~/ans/venv` with the same versions, no internet needed on the CORE; `core_deploy.sh` code + preflight; `core_run.sh` bench / check / credentials / mission (nohup) / log / stop; `core_fetch.sh` logs, maps and graph back to the laptop; `core_check.py` read-only connection check; `core_common.sh`). The software E-Stop stays on the laptop: `tools/estop_laptop.py` (same SimpleEstop as the mission, 9 s): the tablet, with the control released, was not registered as an E-Stop endpoint. To fix: `core_run.sh mission` stays attached to the SSH session instead of returning.
- **Global maps updated with numpy on the raster layers** (`GlobalGrid.update`, `update_clearance`, `GlobalTerrainGrid.update`: the per-cell Python loops over the ~16 000 cells of each scan are gone; `obs_count`, `clear_*` are dictionary views of the layers). Phase `maps` on the Jetson 252 -> 85 ms per cycle in the bench.
- `tools/bench_cruise.py`: the cruise planning cycle against the node density, no robot (the `test_cruise` scenarios).

## 2026-11-04 (indoor mission 16:10, same room as 11:21 and 11:38)

Started 1.6 m further on and 11 deg turned with respect to 11:38, so the grid fell differently on the room: the row 1 is almost all furniture. 8 cells visited (6 on 11:38), 7 stops of 1.5 s or more (14 s in all); hand-offs 0.9-1.6 s; cycle 0.17 s median, 0.27 s at most; only PLAIN.

- **Goal of the cell: nearest to the centre first** (`goal.find_best_point_in_cell`, criterion 3): the rings of 0.25 m from the centre come before "at least 0.33 m of free space", "seen free" and "away from the border", which now decide only within a ring (the points below 0.28 m from obstacles and the ones not reachable in the graph stay out as before). Why: the goal is where the robot looks at the whole cell from (user's choice); with the free space first the goal of (1,2) went 0.94 m from the centre, against the furniture on the border. `test_goal_selection`: the centre 0.30 m from an obstacle is now kept; on the attempt starts of 22-10 16:04 the goals on the cell border go from 11 to 7.
- **A cell already seen almost all counts as explored** (`cell_attempt.CELL_SEEN_EXPLORED`, `CELL_SEEN_FRACTION` = 0.90, `cells.cell_seen_fraction`): after the first scan of the attempt, if 90% of the cell is already in the global map (seen free or obstacle) the attempt ends at once with success, `[SEEN]` in the log; the GraphNav waypoint of the cell is made where the robot is. Why: on 16:10 (1,3), (1,0) and the second attempt on (1,4) were 95-100% seen from the corridor before their attempt (estimate from the saved scans, which are fewer than those in the map), and entering them cost the 7 rotations refused with back-off (0.6 m back, 0.5 m forward), two returns with GraphNav of 0.6 and 0.9 m and two confirmed blocks, ~60 s for two cells; the cells ahead of the robot were 5-45% seen (11:38: none above 82%).
- **The low-obstacle filter keeps the edges of every obstacle that counts** (`LOW_OBSTACLE_NEAR_KEPT_M` 0.15 -> 0.30 m, from any obstacle cell kept, not only the high ones): on 16:10, cell (1,0), it dropped the edge of a piece of furniture whose cells all read the floor height (under a table), up to 0.20 m from its steady cells, and the frontier let the robot on 0.65 m instead of 0.29. Replayed: indoor nothing dropped any more (11:38, 16:10); outdoor 14:08 the stops for obstacle on the saved paths 54 -> 51 only: the grass is mostly next to steady grass, and the height of Spot's terrain does not tell a real obstacle from grass safely.
- Tried and dropped: ending the attempt as soon as no point of the cell is reachable in the graph (for (0,5), behind the wall, 11 goals in 3 rounds): it ended attempts that succeed later, when the robot sees more and the edges come back (`test_cruise` box, `test_e2e_loop`, `test_mission`).
- **`--hostname` and `--no-estop`** (`run.py`): the robot address (default `SPOT_HOSTNAME`, else 192.168.80.3; from the CORE I/O 192.168.50.3) and the mission without our software E-Stop (on the CORE I/O its keepalive runs on board and would never expire when the WiFi drops: the tablet's stays).

## 2026-11-04 (outdoor mission 14:08: stops at every cell, planning cycle growing, gait)

Mission of 22.7 min, 64/64 cells, 8 x 8 cells of 2 m, 5 nodes/m2. 67 stops of 1.5 s or more (190 s): 46 of them (150 s) at the hand-off to the next cell.

- **Global maps: rasters kept up to date cell by cell** (`global_maps._GrowRaster`). `GlobalTerrainGrid` keeps sum, count, mean and seen-mask arrays, updated by `update()` with `np.add.at` (the same sums in the same order: the means of `get_height` to the bit); `GlobalGrid` keeps the occupied cells and, for every margin asked, the count of occupied cells within its disk, updated by `_set_state`. Before, `_mean_height_raster`, `_has_height_raster` and `_get_margin_raster` rebuilt their rasters from the dictionaries over the whole explored area at every change of the map, which in cruise is every planning cycle. Why: the 'edges' phase of the cycle went from 0.07 s at the start to 0.58 s after 20 min, with the same number of edges per cycle; the cycle passed 0.4 s in 47% of the plans (12% in the first 5 min, 77% in the last), the Spot grid was read every 0.73 s at the end, and the next attempt at a hand-off took 2.5 s at the start and 4.6 s at the end. On the final maps of the mission the three rebuilds took 0.32 s per cycle; now a cycle of map updates and rasters takes 0.065 s. Checked equal to the old rasters on random updates and on the final maps.
- **Hand-off to the next cell without stopping** (`attempt_cruise`): the last send of the cruise covers the whole confirmed stretch (dead man of the trajectory time, as a leg, not `PATH_HOLD_S` = 2 s), `CRUISE_HANDOFF_M` 0.6 -> 1.0 m, and the breadcrumb at the start of the attempt only with the usual spacing while the robot still walks (the waypoint of the cell just visited is 10 cm behind; each waypoint costs ~0.4 s). New log line `[CRUISE] Hand-off: this cruise starts X s after...`. Why: the robot had 0.5 m (~2 s) to walk and the next cruise started after 4 s median, and the command expired after 2 s anyway: it stood still at 46 hand-offs out of 58.
- **`PLANE_SLOPES` on, also in the frontier** (`safe_frontier._plane_field`: same estimate and same usable cells as the PRM; before, the frontier measured with the height differences, which read more, and would refuse edges the PRM accepted). Checked against the ground plane of the legs (`gpe`, max of longitudinal and lateral) on 323 cruise stretches of 1 m walked by the robot, ground up to 22 deg: plane |error| 1.1 deg median (bias -0.5, p90 2.7, correlation 0.96), old estimate 2.2 (bias +2.2, p90 5.8, 0.87); below 4 deg the old reads 5.8, the plane 2.2; above 16 deg the plane reads 3-4 deg less. Gait level right against the ground: plane 0.13/0.20 89% (10% too high, 1% too low), old 0.15/0.25 76% (23% too high). The veto stays 0.6.
- **Gait in cruise: hysteresis and the first metre** (`attempt_cruise.GAIT_DOWN_HOLD_S` = 2 s, `planning_cycle.GAIT_LOOKAHEAD_M` = 1 m): a harder level at once, a lighter one only after 2 s of plans asking for it; the terrain is that of the first metre of the confirmed stretch (the robot walks less before the next plan), not of all of it (up to 3 m). Why: 239 gait changes in 20 min (one every 5 s), HARD (0.25 m/s) 48% of the time, 49 stretches in MODERATE or HARD on ground below 5 deg.
- **No more `keep_path`; an attempt without a path ends once** (`attempt_cruise`): when no goal of the cell has a path in the graph, the cruise stops the robot and the attempt ends (FAILED, the side is marked as tried), instead of a new cruise at every turn. And the paths may also use the cells next to both the robot's cell and the goal's (`mission_prm.planning_cells`, switch `PLAN_ALSO_SHARED_NEIGHBOUR_CELLS`, for `PLAN_ONLY_EXPLORED_CELLS`): the way around an obstacle on the border between the two. Why: on 04-11 11:38, cell (1,3) (a gap between two pieces of furniture narrower than the margin), after the goals ran out every move started a cruise that ended at its first plan: eight cruises of 4-9 cm in a row at 0.06 m/s; three cells out of eight were disconnected only by the cells around the border. A first fix (walk the previous path, frontier only: `keep_path`) ran 4 times on 14:08 (2 ended against an obstacle) and was dropped: it skipped the margins of the graph.
- **Cruise scan bundles: no more overwritten, and saved when the frontier changes reason** (`planning_cycle.CruisePlanner._save_bundle`, `CRUISE_BUNDLE_REASON_S` = 2 s): after a new goal while walking the new planner numbered its bundles from 1 again and overwrote those of the same attempt turn (`bundle_seq` now carries the count over); and a bundle was saved only every 0.5 m walked, so turning on the spot or held by the frontier nothing was saved. Why: on 14:08, cell (0,2), 15 s of plans alternating 0.35 m confirmed (slope) and 2.9 m, with the robot turning back and forth, left no scan in the viewer, and the first bundle of the second cruise was overwritten.
- **Photos when a cruise stops on the terrain** (obstacle, slope, roughness, edge out of the graph: `photos.capture` after the robot stands, name `cruise_it..._<time>`, the reason in the JSON). Before only the confirmed blocks had photos: on 14:08 one (tall grass with seed stalks, cell (0,5)) against 23 stops for obstacle.
- **Low intermittent obstacles filtered** (`perception/low_obstacles.py`, `LOW_OBSTACLE_FILTER` on; called in `spot_grid.LocalGrid.process_scan` and in the frontier thread's own snapshot, shared history): Spot's `obstacle_distance` without the obstacle cells that are low (< 0.30 m above the local ground, minimum of the reliable raw heights within 0.30 m), not within 0.15 m of a high obstacle (or one without a reliable height: walls), observed at least 4 times and obstacle in less than 60% of their last 10 observations and not in the last two in a row; `obstacle_distance` is recomputed without them (never below Spot's value, never beyond the border of the window). All the rest uses it (frontier, live edge discard, global maps, goal choice); bundles keep Spot's own in `obstacle_distance_spot` (viewer key `s`). Why: on 14:08 the cells Spot marked as obstacle, seen at least 5 times, were obstacle in 17% of their observations (median) below 10 cm, 45% at 10-30 cm, 87% at 30-60 cm, 100% above 60 cm: the grass comes and goes and edges and frontier said free, blocked as the robot got closer, free again. Replayed on the saved scans in time order: outdoor 2.6% of the obstacle cells dropped, frontier stops for obstacle on the saved paths 54 -> 45; indoor (11:38, 11:21) 0.8% and 0.0%, no stop changed. Without the 0.15 m rule the gap of (1,3) on 11:38 was freed; without the two-in-a-row rule a box appearing where it was free was ignored (`test_cruise`: 0.06 m from it instead of 0.33).
- **Gait: CRAWL only in HARD, and the locomotion_hint changes only after 3 s** (`motion.select_mobility_params`: MODERATE with HINT_AUTO, high step, mu 0.6, 0.35 m/s; `attempt_cruise.GAIT_HINT_HOLD_S`: in cruise the new hint goes only after 3 s of plans asking for it, both ways, the speed/friction/step of the new level at once; the state is on env, over the hand-offs). Why: on 14:08 10 of the 12 sideways jumps of the body in cruise (> 0.30 m/s across the nose) came within 2 s of a locomotion_hint change (17% of the cruise time): 118 changes AUTO <-> CRAWL; in (0,1) MODERATE, PLAIN, MODERATE within one second and then 15 cm sideways in 0.4 s. Ground under MODERATE: 5 deg median (`gpe`), 14 at most. Replayed on the log: 117 hint changes -> 29 with the 3 s hold.
- Tests: `tests/test_mission.py` runs with `PLANE_SLOPES` and `PLAN_ALSO_SHARED_NEIGHBOUR_CELLS` off (its failures are scripted by attempt number: another sequence of attempts moves them onto other cells, and the fall during the GraphNav trip and the return after the block no longer happen); `test_e2e_loop` S15 with the height-difference slopes (the final step below the minimum comes from the path of those seeds); the three tests reset `low_obstacles.HISTORY` with the obstacle history. `test_cruise`, `test_e2e_loop`, `test_mission`: all OK.
- Grass on 14:08, not changed: the roughness alone makes 0.03% of the obstacle cells near the robot (all the others also have `obstacle_distance` <= 0.15 m); 20% of the cells 0.6-2 m from the robot are below 0.15 m of `obstacle_distance`, 29% of the obstacle cells are less than 10 cm above the ground, and 9 of the 41 frontier stops for obstacle are in front of cells that low: Spot's own `obstacle_distance` on grass, to check on the photos of the blocks.

## 2026-11-04 (indoor mission 11:21: the robot attached to a node without edges)

- **The robot is attached to the nearest node WITH edges** (`roadmap.PRM.anchor_node(connected_first=True)`, used by `add_stop_node` for the robot and by the reachability check of `mission_prm`; the goals as before). Why: on 04-11 11:21 the cruise to (0,3) ended along a piece of furniture; the nearest node (0.15 m, 0.21 m from the furniture) lost all its edges in the live refresh (edges closer than 0.25 m to an obstacle), the robot was attached there and every cell from that position was "unreachable": (0,4) and (1,3) failed at once, their sides were marked as tried and the mission ended after 6 cells in 1.4 min. Two connected nodes were at 0.32 and 0.39 m: from them the path to (0,4) exists in the saved graph (3 nodes, checked on `iteration_3_cell_0_4_vis.npz`). The other two failures of the mission ((1,2), (2,0)) were the cell restriction (the paths cross cells never visited), not this.

## 2026-11-04 (gait thresholds for the plane slopes)

- **Gait thresholds of their own for `PLANE_SLOPES`** (`perception/gait.py`: `PLANE_GAIT_PLAIN_SLOPE_MAX` = 0.13, `PLANE_GAIT_MODERATE_SLOPE_MAX` = 0.20, `slope_thresholds()`; used by `motion.select_mobility_params`, the cruise level, `run.py` and `tools/gait_trial.py`). With the switch off nothing changes (0.15 / 0.25). Why: the thresholds were tuned on today's estimate, which reads more than the plane, and with the switch on and the same thresholds the HARD legs would go from 18% to 9%. Measured with `tools/plane_thresholds.py` (new) on 1382 stop-and-go legs of 51 missions: for each leg the last scan before it, both estimates along the leg, against the body pitch in the leg time.
  - Longitudinal error against the median body pitch, on the slopes (6-15 deg): today +4.3/+4.6 deg, plane +1.5/+1.6 deg; flat (below 3 deg) +1.6 against +0.5. Against the 90th percentile of the pitch (which also has the gait oscillation) the plane reads 1.6-2.9 deg less on the slopes, today's estimate 0.7 more: the true ground is between the two measures.
  - Today 0.15 / 0.25: legs with measured pitch > 8.5 deg put in MODERATE or HARD 99%, > 14 deg in HARD 95%; false alarms (calm legs in CRAWL for nothing) 26% and 10%. Plane 0.13 / 0.20: 99% and 94%, false alarms 18% and 10%. Same share of legs above as today would be 0.108 / 0.200.
  - Not checked yet: the lateral decides the level in ~60% of the legs, and the body roll is not a measure of the ground. The ground plane of the legs (`gpe`, from 04-11) is used by the tool when present: to rerun after the field tests.
- **The edge veto stays `SLOPE_THRESHOLD` = 0.6 with `PLANE_SLOPES` too** (~31 deg, Spot's slope limit): the plane is closer to the real slope, so the veto acts near the real limit, while today's estimate, reading more, vetoes earlier (the steepest leg walked, 0.59 on today's estimate, is 0.41 on the plane). The frontier still measures with today's estimate and the same threshold. Replay of every mission with the switch on: 82 paths out of 82. Not changed: the PRM slope cost weights, on today's estimate (with the plane the slopes cost ~20% less).
- `slopes._box_sum` returned one row and one column more than the grid (the window centred just outside it, always NaN in the plane field because under half full): now the grid shape. Same values on every cell of the grid (`tools/plane_thresholds.py` identical).

## 2026-11-03 (photos at the stops, standing capture of a scene)

- **`SAVE_BLOCK_PHOTOS`** (`config.py`, on): at every confirmed block (same name as the `BLOCKED_n.npz`) and every GraphNav stop, the five body cameras are saved in `MissionMap/<mission>/photos/`, with a `.json` giving for each camera the pose in VISION and the intrinsics (`robot/photos.py`). With them the obstacle cells of the scan can be projected onto the image afterwards. Why: on 28-10 the only ground truth on grass was memory; the photo taken by the robot at the moment it stops says what it had in front. Never stops the mission.
- **`tools/capture_scene.py`**: robot standing, a label typed on the PC, one capture = the layers `terrain`, `terrain_valid`, `obstacle_distance`, `intensity`, `no_step` with Spot's ground plane and the five photos, in `SceneCaptures/<date_time>/`. It prints the height above the ground of the obstacle cells. For the standing grass test of `FIELD_TEST.md` (tall grass, bush, solid object): true labels for the rule on grass.
- `spot_grid.LocalGrid.extra_layers` also returns `origin_z`, the z of the layer frame in VISION, needed to turn the decoded `terrain` into a height (not used by the mission).
- `FIELD_TEST.md`: indoor checks before going out (GraphNav reason, photos, Spot's margin, capture tool) and the standing grass capture.

---

## 2026-11-03 (offline tests of the continuous motion; fixes they found)

- **`tests/test_cruise.py`** (new, ~4 min, real time): the cruise with the real planning thread in the worlds of the end-to-end test, the fake Spot moving the robot in real time. Open field, wall with a gap (path changes while walking, distance from the corner), a box appearing 1.2 m ahead during the cruise, goal inside a box (new goal while walking), obstacle beyond the goal and next cell, three cells in a row (hand-offs, big turns).
- **`tests/test_path_command.py`** brought up to date with the path command of today: points along the edges, heading along them, corners while walking and on the spot, start on the spot, back onto the checked line, off the path (and the wider limit while turning at the start), "blocked" as advice while walking, the leg of the stop-and-go, dead man, network errors. `fake_spot.advise_blocked` reports the goal blocked while the robot walks.
- **Corners while walking slowed down to 0.06 m/s** (found by the test): the "sideways step back onto the line" rule (heading of the next segment on a short first segment) also applied with the robot simply less than 0.20 m before the next node along the line. Now only when the robot is beside the line (`first_is_step`, step more than 45 deg off the segment).
- **No expiry while a new goal is chosen while walking**: the stretch confirmed by the last plan is sent again before and after the choice (the command expired after 2 s in two scenarios).
- **Rejoining a changed path**: up to `CRUISE_REJOIN_MAX_M` = 0.20 m (was 0.10) between the robot and the new line before the cruise hands over to the stop-and-go (12 and 15 cm in the world with the gap).
- `test_cruise.py`, three cells in a row: the wall now ends 0.5 m before the border between the cells. With its end ON the border, going around it with 0.30 m of clearance enters a cell that `PLAN_ONLY_EXPLORED_CELLS` forbids, and whether a way inside the allowed cells exists depended on a few cm of the robot pose: 1 run out of 6 failed. Checked with the stop-and-go alone (deterministic), standing under the wall end: 0/6 entered with the restriction, 6/6 without. Not a fault of the cruise; in a mission the side is marked as tried and the cell retried from another side or with GraphNav.
- **Fall check during the cruise** every `CRUISE_FALL_CHECK_S` = 1 s (`motion.check_fall`): the stop-and-go checks it at every turn, the cruise did not; on a fall the cruise ends and the next turn gets the robot up (`_guards`). Scenario in `test_cruise.py`.
- Test harness: the real-time fake Spot moved the robot back to its last trajectory pose after a stop-and-go leg (a jump in the track).

---

## 2026-11-03 (calibration tools ported from the old code; grass work of the online sessions)

- **`tools/calibration.py`** and **`tools/gait_trial.py`**: `taratura.py` and `prova_andatura.py`, written in the online sessions on the old code (branch `AdL-version`, 03-11, commits 0cbd1fd and 1694e8f), ported to the `spot_explorer` package and to English. Same computations; the track trial also records the local-plane estimate before starting and the ground plane of the legs (`gpe`) during the pass, and the calibration compares the inclinometer with edges, plane and gpe. Never tried on the robot: the first time indoors (FIELD_TEST.md, "Gait calibration"). Includes 5bf229f of the online session (slope on the same cells as the mission, `slope_usable_mask`; roughness as the mission, 90th percentile of the samples along the line), checked on saved scans of 22-10 and 26-10. On the 904 stop-and-go legs since 22-10: beta 1.8 [1.1; 3.0] per unit of slope with the level separated (today 2), beta_lat 0.7 [-0.2; 1.9] (today 2), the HARD threshold 0.25 catches 61% of the legs with pitch above 12 deg (0.20: 84%).
- From the `grass` branch (online sessions): photos at the blocks and GraphNav stops (`robot/photos.py`, `SAVE_BLOCK_PHOTOS`), standing scene capture (`tools/capture_scene.py`), the reason why GraphNav stops in the log (entry "why GraphNav stops" below).

---

## 2026-11-03 (why GraphNav stops, behaviour unchanged)

`[GRAPHNAV]` gives the reason when GraphNav stops: `stuck_reason` (e.g. `OBSTACLE`, `GOAL_BLOCKED`), `blockage_status`, `route_following_status` and the metres left; while moving it prints the changes of route and blockage status (e.g. `RETURNING_TO_ROUTE`, `ROUTE_BLOCKED_TEMPORARILY` before a `STUCK`). Why: the return to wp_0 of 28-10 13:50 stopped in the grass at 0.63 m with only `bloccato (STUCK)` in the log, and GraphNav walks with Spot's default mobility parameters (foot obstacle avoidance on, our edges carry no `mobility_params`, see TODO, grass): without the reason we cannot tell if that is the cause.

---

## 2026-11-03 (slopes: measured against the robot, plane estimate behind a switch, off)

- **`tools/slope_check.py`**: for every sample of trajectory.csv the slope under the robot from the last scan saved before it, with several methods, against the measured one (body tilt; from 04-11 also the ground plane from the legs). 46800 samples of 40 missions, longitudinal, against the body pitch: today's differences on a base mean error 2.15 deg, bias +2.0 (the 90th percentile along a segment extended beyond the body inflates it); a plane on a square window as long as the body at the robot centre 0.91 deg, bias -0.1, correlation 0.95 (0.97 signed); on the slopes (22-10, 26-10, 28-10, 29-10) 3.0 -> 1.2 deg. Lateral: not checkable on the body: with the robot walking 45-90 deg off the fall line of a 10-15 deg slope the estimated pitch matches the measured one (3.6 / 3.5 deg) but the body roll stays 0.5 deg whatever the angle: Spot keeps the body level sideways.
- **Ground plane of the legs in trajectory.csv** (`gpe_long_deg`, `gpe_lat_deg`, from Spot's frame `gpe`): the slope the robot really stands on, to check the lateral too.
- **`slopes.PLANE_SLOPES`** (default `False`): the edges on the live scan with the plane estimate (`plane_gradient_field`, integral images, ~3 ms per scan; `plane_slope_profiles_batch`, 11 ms per 2000 edges against 82 of the batch of today). Replay of 13-10 12:40 with it on: paths 11/11, 300-475 fewer edges discarded for slope on the slopes, graph time 31 s against 35. SLOPE_THRESHOLD and the gait levels were tuned on the old estimate (about 2 deg more on slopes): to be retuned before using it.

---

## 2026-11-03 (fewer stops between cells: big turns and failed attempts)

Mission of 03-11 16:02 (other start, same room): 6 cruises out of 6 ended with the hand-off (2 with the last stretch closed by an obstacle), 3 new goals while walking, exploration 40 s (57 at 15:39, 73 with the stop-and-go at 14:23), standing 3% and turning on the spot 7% of the cruise, cycle median 0.17 s with the cruise bundles. Five times out of six, though, the attempt on the next cell failed at once (cells beyond the wall: no goal or no path in the graph) and stopped the robot, then the next attempt started from standing.

- **An attempt that fails before its loop leaves the robot walking** (`CellAttempt.run`): it gave no command, only computations; the mission goes on to the next cell with the robot on the stretch already confirmed. An attempt that failed after moving still stops it.
- **Big turns after a hand-off without stopping first** (`_moving_check`): beyond 35 deg the rotation on the spot is made inside the new command if the swept area, checked at the pose of the scan, is clear with `CHAIN_TURN_EXTRA_AIR_M` = 0.15 m more air (the robot still walks some cm before the command arrives); otherwise it stops and looks again standing as before. Smoke run: +112, -137, +82 deg turned without stopping.

---

## 2026-11-03 (viewer: area really seen; cruise in the viewer and in trajectory.csv)

From the review of 15:39 in the viewer. Cell (0,4) looked explored "far from the centre": the centre is inside a piece of furniture and the only free part reachable is the corridor strip at the corner of the cell, where the goal was chosen (the 10 never-seen candidates are beyond the furniture or outside the room): the choice was right. Two real defects came out instead.

- **The stretch walked after a hand-off was not recorded** (trajectory.csv): the recorder stopped at the hand-off while the robot went on walking to the goal, and the viewer showed an "unrecorded leg". Now it goes on until the next command takes over or the robot is stopped (`attempt_cruise._close_track`).
- **Scan bundles of the cruise**: only the stop-and-go turns saved one (11 bundles at 15:39 for 34 s of cruise). `CruisePlanner` now saves one every `CRUISE_BUNDLE_STEP_M` = 0.5 m walked, same layers and fields as the stop-and-go (`planning_cycle.scan_bundle_arrays`, shared), decision `cruise`, target the end of the confirmed stretch, name `..._<loop>_cruise<cycle>.npz`; the viewer orders them by time with the others.
- **Viewer, global panel**: the light blue square was the local grid window (~3.8 m), aligned with the axes of the fixed frame because that is how Spot gives the local grid, not the area the cameras saw. Now the light blue contour is the area really written by the sensors in that scan (`terrain_valid` and not `unwritten`), the window a thin dashed square.

---

## 2026-11-03 (no stop at the cell goals and new goal while walking, with the cruise)

Mission of 03-11 15:21 (11 cruises, 0.27 m/s, standing 1%): 6 cruises out of 11 ended with "no path to the goal from the next node": the thread saw the goal of the cell isolated (too close to furniture and walls) well before getting there, then the robot stopped only so that the main thread could choose another goal. And every cell goal was a stop of about 1 s.

- **New goal while walking** (`CruiseSteps._cruise_repick`): on a plan 'no_path' the planning pauses (the cycle running ends, the robot goes on along the last stretch confirmed), `_repick_goal` chooses as the stop-and-go does (same limits), and a new `CruisePlanner` starts towards it keeping the edge being walked. `[CRUISE] New goal ... chosen while walking`; statistic `cruise_goal_repicks`.
- **`attempt_cruise.CHAIN_CELLS`** (default `True`, acts only with `CONTINUOUS_MOTION`): inside the target cell and within `CRUISE_HANDOFF_M` = 0.6 m of the goal the attempt ends successfully with the robot still walking towards the goal (`env._moving_pc`); the mission marks the cell and the next attempt starts while it walks. New phase `_moving_check` in the loop: with the robot walking the turn goes on only into a cruise with a turn within 35 deg (rotation clear) or past a path node; anything else (bigger turn, wait, block, partial step) stops the robot first and the turn is made standing. `stop_if_moving` before GraphNav, at the end of the exploration, after an attempt that did not hand off. Statistic `cruise_handoffs`.
- Mission of 03-11 15:39 (3 hand-offs while walking, a new goal chosen while walking, standing 0% of the cruise): 4 cells out of 6 still ended with a stop, "the confirmed stretch ends (obstacle)" and then the stop-and-go accepting the goal within 0.65 m. The **hand-off now also covers that case**: robot inside the cell, last stretch closed by an obstacle (roughness, slope) and goal within `GOAL_REACHED_TOLERANCE_M` along the path, the arrival the stop-and-go would accept standing.
- **Rotation on the spot at the start of a path command**: limit from the line 0.20 m (`PATH_MAX_DEVIATION_TURN_M`) until it ends, then 0.10 m; the body centre moved 9.6 cm while turning 90 degrees at 15:39, near stopping the robot for nothing. That part does not count in `max_deviation`.
- `_fresh_scan` takes the robot pose from the scan (same snapshot of transforms as the grid), as `_first_scan` and the cruise planner already did: the robot may be walking.
- Smoke run only (end-to-end world, real-time fake robot): two cells in a row, hand-off at 0.56 m from the first goal while walking, stop to turn 72 deg towards the second cell, hand-off again; goal inside a box, new goal chosen while walking. With the switches off the logs of the 25 end-to-end scenarios are unchanged, whole-mission test passes.

---

## 2026-11-03 (first field tests of the path command and of the cruise: fixes)

Two indoor missions in the same room as 14:23 (stop-and-go: legs 49 s, standing 36% of it): 14:45 with `PATH_COMMAND_FOR_LEGS` (standing 27%, 3-7 cm at most from the line), 14:48 with the cruise and `INCREMENTAL_REFRESH` (7 cruises, 38 s, 10.9 m, 0.29 m/s, turning on the spot 3% and standing 1% of the time; cycle of the planning thread 0.15-0.30 s, about 1 m confirmed ahead, at most 6 cm from the line; one cruise stopped in front of an obstacle with 0.04 m left, then the stop-and-go accepted the arrival within tolerance).

- **Spot's "final point blocked" is no longer a stop** (`motion.PathCommand.poll`). At 14:45 Spot's feedback (`final_goal_status`) said blocked in 4 legs out of 19 with the robot still walking; stopping on it gave two arrivals at 5-8 cm and two "mechanical failures" (edge discarded for the mission and return with GraphNav). `relative_move` never read it. Now it counts as blocked only with the robot also standing (less than 3 cm and 3 deg in `PATH_BLOCKED_STILL_S` = 1.5 s); a `[PATH]` line says when Spot reports it.
- **GraphNav breadcrumb during the cruise in another thread**: at 14:48 creating the waypoint kept the main thread busy up to 1.6 s, without reading plans nor sending. Also the time of the stale-plan check was read before that call, so a plan read right after looked already old: the cruise ended with "no new plan for 1.4 s" while the thread was planning every 0.4 s.
- **Arrival at the goal of the cruise**: it now waits for the command to end (at most 2 s) before giving back to the stop-and-go; before, the next attempt started (and scanned) with the robot still walking the last 5-10 cm.
- **Times of the trajectory points**: the longer of walking time and turning time, not their sum. At 14:48 a 41-degree turn while walking went at 0.06-0.2 m/s; now about 0.27 m/s. The rotation on the spot is unchanged.
- **Log of the planning thread**: no `[REFRESH]` and `[EDGES]` lines at every cycle (two lines every 0.4 s, two thirds of the log at 14:48); their counts are in the `[CRUISE] Plan` line, and the cycles do not add to the mission statistics of the stop-and-go (`live_arc_vetoes` grew by ~200 per cycle); `cruise_cycles` in the statistics.
- `mission_report.py`: with no stop-and-go leg (14:48) the motion section stopped before reading the trajectory, so neither the cruise nor the GraphNav moves were printed; the cruise line now splits turning on the spot and standing.
- `[TARGET]` said `libero` (Italian) for a goal seen free.
- **The path command goes back onto the checked line** (`PathCommand.send` keeps the projection of the robot, with a short sideways step and the nose on the next segment; the cruise stops if a new plan's line passes more than 10 cm from the robot). The frontier checks the stretch from where the robot was at the scan; when the path changed while walking, the projection of the robot fell beside the new line and was dropped when closer than 0.20 m, so the robot headed for the next node along a line nobody had checked. In the smoke run on the wall with a gap the centre passed 0.24-0.27 m from the wall corner in two runs out of five; after the fix 0.31-0.34 m in four runs out of four.
- **After a stop the cruise waits for the robot to stand** (at most `CRUISE_SETTLE_S` = 1 s) before giving back to the stop-and-go. Mission of 03-11 15:04 (with the fixes above: 9 cruises, 0.33 m/s, standing 1%, no stale plan): a cruise stopped at 0.32 m/s because the goal had become isolated, and 0.28 s later the next one started from a pose still sliding forwards, on a different line, and stopped at once 11 cm off it.
- **No exit rule from a tight space in the cruise** (`compute_safe_frontier(allow_escape=False)` in `CruisePlanner`; the cruise does not start when the frontier of the stop-and-go has it active). Recomputed every 0.4 s from where the robot had got to, the rule lowered the threshold a little at every cycle: in the smoke run on the wall with a gap the centre passed 0.24 m from the wall corner (0.34 m with the stop-and-go); now 0.30 m.

---

## 2026-11-03 (continuous motion: the cruise, behind a switch, off)

Fourth piece of the continuous motion (TODO "Main work" 4): the robot walks along the path without stopping at the nodes, and the planning runs while it walks.

- **`attempt_cruise.CONTINUOUS_MOTION`** (default `False`), `mission/attempt_cruise.py` (`CruiseSteps`, a phase of `CellAttempt`): when the stop-and-go decides a move with at least one whole leg (after the arrival check and the rotation check of the first leg), `_cruise` sends the stretch confirmed by that frontier to `motion.PathCommand` and puts the frontier thread in planning mode with `CruisePlanner`. At every new plan (every 0.4 s) the main thread sends its confirmed stretch, which replaces the previous one without stopping the robot: the robot never goes beyond the stretch confirmed by the last plan. Back to the stop-and-go (a turn with a fresh scan, as after a move) when it arrives at the goal, when the stretch confirmed ahead of the robot is shorter than `CRUISE_MIN_AHEAD_M` = 0.10 m (obstacle, edge of the data, corner where the rotation is not clear), when the edge being walked leaves the graph, with no path, when the command ends badly (off the path, blocked, expired), with no new plan for `CRUISE_PLAN_STALE_S` = 1.2 s or three plans in a row without data, low battery, after `CRUISE_MAX_S`. Then: walked track into the retreat arcs, robot node where the robot is, path from there, breadcrumb (also every second during the cruise).
- **`CruisePlanner`**: corners sharper than `motion.PATH_TURN_IN_PLACE_DEG` inside the confirmed stretch are checked with `geometry.rotation_is_clear` (the confirmed stretch stops at the first one that is not clear); the worst slope and roughness of the stretch go with the plan for the gait, chosen again (and printed) only when the level changes. Stretches of the starting path that are not edges (shortcut of the stop-and-go) do not count as `edge_blocked`.
- Log: `[CRUISE] Start`, a `[CRUISE] Plan` line per plan (metres confirmed ahead, reason, nodes to the goal, path changes, cycle time, distance from the line), `[CRUISE] End` with the reason. Statistics `cruises`, `cruise_s`, `cruise_m`, `cruise_plans`, `cruise_sends`, `cruise_replans`, `cruise_max_dev_cm`, `cruise_exits`; `trajectory.csv` with `move` = `cruise`; `mission_report.py` prints a `Cruise` line (times, metres, time standing).
- Checked only with a smoke run, not yet a test: two worlds of the end-to-end test with a fake robot walking in real time and the real planning thread. Open field: 6.6 m to the goal in one cruise, 45 plans, cycle 0.3-0.45 s, about 1.1 m confirmed ahead. Wall with a gap: 19 path changes while walking, arrived, minimum distance of the centre from the wall 0.28 m. With the switch off the logs of the 25 end-to-end scenarios are identical.

---

## 2026-11-03 (path command in the legs of the mission, behind a switch, off)

To try the path command on the real robot inside a normal mission, with all its safety nets, instead of the fixed shapes of `try_path.py` that do not look at obstacles.

- **`motion.PATH_COMMAND_FOR_LEGS`** (default `False`): every leg goes through `motion.navigate_to_path` instead of `navigate_to`. Same target, same stop halfway from the frontier thread, same time limit and check of the real advance; rotation and walk are one trajectory (`PathCommand`, sent once for the whole leg) instead of two commands each waiting for the body to settle. The rotation stays on the spot at the start beyond 10 deg (`LEG_START_IN_PLACE_DEG` = `geometry.ROTATION_CHECK_MIN_DYAW_DEG`), the one the mission has already checked. A `[PATH]` line per leg (metres, time, points, distance from the line); `path_legs` and `path_legs_max_dev_cm` in the statistics; the switch is in `mission_summary.json`.

---

## 2026-11-03 (planning cycle shared, planning mode of the frontier thread, not used yet)

Third piece of the continuous motion (TODO "Main work" 3): the planning that today runs standing still at every turn of the loop will run in the frontier thread while the robot walks.

- **`mission/planning_cycle.py`**: `scan_into_maps` (scan processing, global maps, local data into the PRM, snapshot with the temporal filter, free-space map) and `refresh_graph` (live discard, edge refresh with its counts and log lines, nodes in obstacles out), moved out of `attempt_scan._fresh_scan` and `_update_graph`, which now call them. The logs of the 25 end-to-end scenarios are identical line by line; whole-mission test passes.
- **`CruisePlanner`** (same file): one call = one cycle while the robot walks: scan with the pose of the same instant, maps, the edge being walked stays committed and the next node becomes the robot node, refresh at every cycle, Dijkstra from the next node with the 10% stability margin, safe frontier and the stretch confirmed free (`confirmed`, the points for `motion.PathCommand`). Status `ok`, `edge_blocked` (the edge being walked is out of the graph), `no_path`, `no_scan`, `error`.
- **`ArcVerificationTracker.start_planning(job, period_s)` / `stop_planning()` / `get_plan()`**: the same thread runs the cycle every 0.4 s instead of the frontier alone and publishes the plan with a version. While it plans it is the only one touching graph and maps; `stop_planning()` returns when the running cycle has ended.

---

## 2026-11-03 (multi-point path command, not used by the mission yet)

Second piece of the continuous motion (TODO "Main work" 1): the command with which the robot will walk along the path without stopping at the nodes. Today every leg is a rotation in place and a straight walk, each waiting for the body to settle: in the mission of 03-11 11:07 the robot stood still 34% of the leg time.

- **`motion.PathCommand`**: one SE2 trajectory along the edges of the path, nose first, without stopping at the nodes. The position stays on the edges (nodes plus points every `PATH_POINT_SPACING_M` = 0.15 m, joined by straight lines); the heading is the direction of the edge along it, so the body stays aligned as the frontier and the edge checks assume, and changes only within `PATH_TURN_ZONE_M` = 0.20 m of a node (turn while walking). Corners sharper than `PATH_TURN_IN_PLACE_DEG` = 60, and a start with the nose more than `PATH_START_IN_PLACE_DEG` = 35 off the first edge, are a rotation on the spot at the node inside the same trajectory. `send()` returns at once and sends only the part of the path ahead of the robot (`path_ahead`, searched in the two segments after where the robot was, so a path that comes back near the start is not cut); `poll()` reads the feedback without waiting and stops the robot farther than `PATH_MAX_DEVIATION_M` = 0.10 m from the line (`off_path`). Point times from the speed limits times `PATH_TIME_FACTOR` = 1.25, so that Spot's reference does not run ahead and cut the corners. Dead man: the command expires `PATH_HOLD_S` = 2 s after the send. Network errors handled like `relative_move`.
- **`tools/try_path.py`**: field test on the real robot outside the mission: line, L, square, zig-zag (53 deg corners) or gentle curve drawn from where the robot stands, path sent again every 0.5 s like the cruise, every poll in `MissionLogs/try_path_*.csv`, summary with time, distance from the line and time standing; `--stop-and-go` runs the same shape with today's legs for the comparison.
- **`tests/fake_spot.py`**: fake Spot with a simulated clock, command and state client, reading and answering with the real protos (trajectory in time, speed limits, stop, dead man, command replaced, point blocked). `tests/test_path_command.py` was written for a first version (heading halfway between the edges at every node) and still has to be brought up to date with the points along the edges.
- The mission does not use it yet: the other tests are unchanged and pass.

---

## 2026-11-03 (incremental edge refresh, behind a switch, off)

First piece of the continuous motion (TODO "Main work" 2): with the scan running while the robot walks, the refresh around the robot runs every ~15 cm instead of every metre, over the whole window ahead and not only within 1.25 m. Most of the edges are then the same as in the previous refresh.

- **`edge_evaluation.INCREMENTAL_REFRESH`** (default `False`): the slope profile of an edge already evaluated on the live scan is kept until the scan sees it better (centre-line coverage up by `REFRESH_COVERAGE_GAIN` = 0.15, or robot `REFRESH_CLOSER_M` = 0.5 m closer to the edge middle); an edge left as that evaluation left it (validity and weight) is not touched at all. Occupancy and live discard are redone at every refresh. The log adds a `[REFRESH]` line with the counts; the switch is in `mission_summary.json`.
- **`tools/bench_refresh.py`**: the robot of the end-to-end test world walks a straight line in 15 cm steps with a refresh at every step. Random nodes 5/m² (the mission of 03-11): 34 -> 28 ms per refresh within 1.25 m, 52 -> 39 ms on the whole window; at 9 nodes/m², 67 -> 50 ms and 126 -> 77 ms. Validity of the edges identical to the switch off; weights apart by the noise the bench puts on the heights at every step (median 0.5% within 1.25 m, 6.5% on the window).
- In the end-to-end test (stop-and-go, ~1 m between refreshes, lattice at 0.25 m) there is no gain: about 200 ms per refresh within 1.25 m in both cases, because most of the edges in the radius are new at every refresh. The rest of the refresh time is not the slopes: rebuilding the neighbour lists edge by edge, the live discard of the edges outside the radius and the global slope cache.
- Switch off: the logs of the 25 end-to-end scenarios identical line by line (`compare_logs.py`); on: end-to-end, whole-mission and replay tests all pass.
- `compare_logs.py` prints `IDENTICAL` (it still said `IDENTICI`).

---

## 2026-11-03 (log messages and report times, behaviour unchanged)

From the indoor mission of 03-11 11:07 (room of about 10 x 3 m, 7 cells closed, all the rest behind the walls), messages that contradicted each other or gave the wrong reason. The robot's behaviour does not change.

- **`[DEFER]`** says from which visited neighbour the cell will be retried, or, if the sides not tried yet face only cells never visited, that it can be retried only if one of them gets visited. Before it always said "will be retried later", and a moment later the same cells left the frontier.
- **End of exploration**: when every frontier cell is skipped in the round the log says so, instead of `[FRONTIER_RANK] Frontier empty` with five cells still in it; `[CELL FRONTIER]` and `[END]` give the real reason (the untried sides face only cells never visited).
- **`[PRECHECK]`** gives the reason from the graph (`mission_prm.unreached_cell_reason`): all the nodes of the cell inside obstacles, or no edge between the reachable area and the cell. Before it always said "obstacles already in the map", also for cells (2,0) and (2,1) with 0 obstacles and 22 points never seen: the edges were vetoed by the wall between.
- `[CONFIG] Mission grid ... source: config.MISSION_GRID_*` (it still said `mission_graph.MISSION_*`).
- **Report times**: the `Smoothness` line mixed two measures (90 s of legs, 32 + 124 s of standing and walking). The trajectory samples of the GraphNav moves have no nodes and fell in a single group spanning from the first to the last GraphNav move (66 s for 25 s of moves). Now `Legs` counts only the legs (03-11: 90 s = 31 standing to rotate or start + 59 walking, 0.24 m/s while walking) and `GraphNav moves` each move on its own (18 s and 7 s); "Measured time" says it is the exploration only, and the distance in `Motion` includes the GraphNav moves.

---

## 2026-10-30 (largest modules split by role, behaviour unchanged)

The three largest modules are split by role, so each file can be read on its own. The robot's behaviour does not change: after every step the logs of the end-to-end tests and of the fake missions are identical line by line, the replay of the reference missions and the goal-selection replay are unchanged, and the mission graphs are the same for 12 sets of parameters (lattice and random, errors included).

- **`CellAttempt`** keeps the current scan as one `Scan` object (before, ~20 attributes `pts_up`, `terrain_up`, `grad_up`, ...) and is split by phase: `mission/cell_attempt.py` the loop, guards, decision and shared actions; `attempt_scan.py` scan, maps, graph update and replan; `attempt_obstacles.py` rotation refused, wait and block; `attempt_legs.py` the legs; `attempt_common.py` the phase outcomes and the battery checks (`battery_return_due`, `battery_low`).
- **PRM**: `planning/roadmap.py` the graph, the penalties, anchoring and the path search (`find_path_dijkstra` = `_dijkstra`, `_within_caps`, `_stable_plan`); `edge_evaluation.py` occupancy, slope, weight and the refresh around the robot (`refresh_local_edge_weights` = `_refresh_nodes`, `_refresh_pairs`, `_refresh_edge`), with the constants `LIVE_PREFILTER_OD_M`, `REFRESH_RADIUS_M`, `REFRESH_BATCH_SLOPES`, `DIRTY_MARGIN_M`; `edge_stats.py` the veto statistics and the `[SLOPES]` line.
- **Mission graph**: `planning/graph_nodes.py` lattice and random nodes, pairs, coverage (with the reasons for the lattice); `mission_graph.py` `MissionGraph` and `build_mission_graph`, with `add_edges_to_prm` and the lattice build in steps; `mission/mission_prm.py` the graph during the mission (`build_mission_prm`, `rebuild_prm_edges`, `restrict_planning_to_explored_cells`, `graph_reaches_cell`, `node_cells`, `robot_on_node`). The command line moves to `tools/bench_graph.py` (`python3 tools/bench_graph.py --compare`, `--timings`, `--preview`, ...); `--compare` failed on a wrong attribute name since the translation, now it works.
- **Imports by the module's own name**: `motion`, `cells`, `connection`, `roadmap`, `records`, `config`, `battery` instead of the old aliases `movements`, `environmentMap`, `spotUtils`, `prm_graph`, `registro`, `parametri`, `batteria`. In `mission_summary.json` the switch keys become `config.X`, `motion.X`, `edge_evaluation.REFRESH_BATCH_SLOPES`; the tools read only the last part of the key, so old and new summaries read the same.
- `tools/preflight.py` follows the PRM methods into `edge_evaluation.py`.
- **Last Italian leftovers**: `mission_viewer.py --background obstacles|terrain|fusion` (the old names still work), plot labels and `scan_itNNN_final.png`; the report of old missions prints the time and closure phases in English (through `legacy.py`); the final scan log label is `scan-final`. `compare_logs.py`, `bench_fiducial.py` and `mission_viewer.py` answer `--help`. Removed `GlobalGrid.clearance_many`, unused since `CLEARANCE_MAP_DECIDES` was removed (29-10).

---

## 2026-10-30 (package, English, behaviour unchanged)

The code becomes the `spot_explorer` package and everything is in English: identifiers, command-line options, comments, docstrings, log messages and tags, internal values and data keys. The robot's behaviour does not change. Every step was checked without the robot: end-to-end and whole-mission test logs identical line by line (up to the step that changed the log text; after it, the same lines with the same numbers on every line), replay of the 10 reference missions identical to the reference, goal-selection replay identical, reports of four field missions with the same numbers, tools run on old and new missions.

- **Layout.** `run_mission.py` starts a mission. `spot_explorer/config.py` (was `parametri.py`), `records.py` (was `registro.py`). `mission/`: `run.py` (start-up, closure, emergency shutdown, command line), `exploration.py` (`Mission`, was `Missione`), `cell_attempt.py` (`CellAttempt`, was `TentativoCella`), `recovery.py`, `cells.py` (was `environmentMap.py`), `summary.py`. `planning/`: `mission_graph.py`, `roadmap.py` (was `prm_graph.py`), `goal.py`, `safe_frontier.py` and `decisions.py` (were `arcVerification.py`). `perception/`: `spot_grid.py`, `global_maps.py`, `slopes.py`, `geometry.py`, `gait.py` (were `spotGrid.py`). `graphnav/`: `map_recording.py`, `navigation.py`, `localization.py`, `map_shortcuts.py`, `apriltags.py` (were `navGraphUtils.py`). `robot/`: `connection.py` (was `spotUtils.py` and `spotLogInUtils.py`), `motion.py` (was `movements.py`), `battery.py` (was `batteria.py`). `tools/`: `mission_report.py`, `mission_viewer.py`, `live_viewer.py` (was `visualizzatore_live.py`), `preflight.py`, `compare_logs.py` (was `confronta_log.py`), `bench_graph.py`, `bench_loop_closure.py`, `bench_fiducial.py` (were `prova_grafo.py`, `prova_anelli.py`, `prova_fiducial.py`). `tests/`: `test_e2e_loop.py`, `test_mission.py`, `test_replay.py` with `replay_reference.json`, `test_goal_selection.py` (were `test_missione.py`, `test_obiettivi.py`, `test_replay_riferimento.json`). `PROVA_CAMPO.md` is `FIELD_TEST.md`.
- **Command line in English**: `--graph lattice|random`, `--distribution uniform|poisson|jittered|halton`, and the tools' options (`--replay`, `--until`, `--missions`, `--all`, `--fast`, `--update-reference`, `--detail`, `--series`, `--background`, `--mission-frame`, ...).
- **Values**: frontier stop reasons `obstacle`, `roughness`, `slope`, `end_of_data`, `end_of_path`; return to wp_0 `arrived`, `near`, `not_arrived`; arrival heading `toward_goal`, `waypoint`, `free`; movement kinds in `trajectory.csv` `cone`, `backoff`, `retreat`, `graphnav_correction`.
- **Log tags**: `[BLOCCO]` → `[BLOCK]`, `[SUBITO]` → `[EARLY]`, `[TEMPO]` → `[TIMING]`, `[RITORNO]` → `[RETURN]`, `[RITIRATA]` → `[RETREAT]`, `[FRONTE]` → `[FRONTIER]`, `[OBIETTIVO]` → `[GOAL]`, `[PROGRESSO]` → `[PROGRESS]`, `[BATTERIA]` → `[BATTERY]`, `[GRAFO]` → `[GRAPH]`, `[CONFIG] INTERRUTTORI` → `[CONFIG] SWITCHES`, and the others (the full list is in `spot_explorer/legacy.py`).
- **Data**: the keys of `mission_summary.json` (`switches`, `graph`, `progress`, ...), downloaded GraphNav maps in `graph/<mission>/Graph_<date>` (was `Grafo_<date>`).
- **Old missions stay readable**: the report, the viewers and the replay tests accept both the Italian and the English logs (regular expressions with both forms) and pass the old values, tags and keys through `spot_explorer/legacy.py`.

---

## 2026-10-29

- **Return to wp_0 corrected with the starting fiducial** (`WP0_FIDUCIAL_CORRECTION`, `WP0_FIDUCIAL_TOLERANCE_M = 0.30`, `WP0_FIDUCIAL_MAX_CORRECTIONS = 2`, `navGraphUtils.graphnav_correct_wp0_with_fiducial`). 29-10 10:57: GraphNav said "arrived" with the robot ~1.2 m to the right of the start (uphill lawn with a single hedge as reference; odometry drift 2.8 m, of which GraphNav corrected 1.86). The starting tag does not move: the robot position in its frame, at the start and on arrival, gives the true offset (line `[WP0] Secondo il tag il robot e' a D m dal punto di partenza`). Beyond 0.30 m: GraphNav is relocalized on the tag (as at start-up) and the robot goes back to wp_0 again, at most twice, with the lines `[WP0] CORREZIONE k/2` and `[WP0] Dopo la correzione k: a D m dalla partenza (prima D0 m)`. If the tag is not visible or the relocalization fails, the robot stays where it is. Only with the outcome "arrived": stopped nearby (STUCK) means the way is blocked. The correction stretch is in `trajectory.csv` as `graphnav_correzione`: red in the after-mission viewer and in the live one; in the report, under "Ritorno a wp_0", the offset and the correction. Test: `test_missione.py`, world G (world A with the return "arrived" 0.8 m to the side: one correction, then 0.00 m) and world A (0.00 m, no correction). In the afternoon field tests the returns stopped 0.04–0.08 m from the start: the correction did not need to trigger.
- **Live viewer**: mission phase at the top (attempt on the cell, GraphNav trip, return to wp_0, correction, emergency, end), a return-to-wp_0 line with the offset from the tag and the correction, start marked, legend below the map.
- **Tests without the robot on any computer.** `test_replay.py --tutte` on a machine in UTC gave 306 differences and a failed invariant ("steep legs not HARD"): the time of `segments.csv` was read with the machine's time zone, so every leg was matched to the wrong scan and the gait came out `None`. Now that time is read as Italian time (`MISSION_TZ`; with Python 3.8, which has no `zoneinfo`, the machine's local time, right on the mission laptop); the mission code does not change, and the replay is all OK again both in UTC and in Italian time. In the same round: `scipy` added to `docker-requirements.txt` (`spotGrid.py` imports it, the mission does not start without it) and `prova_fiducial.py` in the file table of the `README.md`.
- **Navigation loop reordered, identical behaviour** (`TentativoCella`). The long phases are split into named steps and the actions repeated in several places are single functions: `_usa_percorso`, `_scarta_tratto` with the duration of the discard (`'ora'`, `'tentativo'`, `'missione'`; before, four different ways of writing the same thing), `_strada_fatta` and `_sosta`, `_ventaglio` and `_esci_dalla_trappola` (three copies). `_esegui_tratti` (375 lines) is split into `_controlla_posa_d_arrivo`, `_prepara_gamba`, `_cammina_tratto`, `_terreno_del_tratto`, `_muovi_e_registra`; `_prepara` (272 lines) into `_prima_scansione`, `_dati_locali_nel_grafo`, `_obiettivo_e_percorso`, `_azzera_contatori`; `_scansione_e_grafo` into `_scansione_fresca`, `_aggiorna_grafo`, `_ripianifica`; `_attesa` into `_cambia_obiettivo_irraggiungibile` and `_ripianifica_subito`; `_rotazione` into `_avanza_nel_cono` and `_arretra_per_ruotare`. A map of the phases at the top of the class. Check: logs of the end-to-end tests and of the fake missions identical line by line, replay and 40 sweep missions with the same outcomes cell by cell. Counted over the 15 missions since 22-10 (~420 attempts) how often each branch triggers (see `TODO.md`): the mission-level recovery (`Missione._recupero`) looked like a duplicate of the new goal inside the attempt, but removing it ends the mission when the first attempts from the start fail for a passing reason (world C of `test_missione.py`: 1 cell out of 9 instead of 9), and in the field it helped 2 times out of 4: it stays.
- **Cleanup before `main`: superseded switches and a shorter log.** Identical behaviour: the logs of the end-to-end tests and of the fake missions are the same as before minus the lines below (only times and line numbers differ), replay and goal choice unchanged.
  - *Switches removed together with their off branch*: `INCREMENTAL_GRAPH_UPDATE` (the local graph update is the only one; `test_replay.py` keeps comparing it with the full build, requested with `rebuild_prm_edges(..., incremental=False)`); `CLEARANCE_MAP_DECIDES` with the `[SPAZIO]` line, the count of the disagreement and the check in the report (the binary map decides; the free-space map stays, the goal choice uses it); `VERBOSE_SLOPE_LOG` (lines `[SLOPE-VETO]` and `[SLOPE-FLIP]`); the quick loop closure before GraphNav trips, off since 26-10, with `GRAPHNAV_SDK_LOOP_BUDGET_S`, `_FAST`, `_FINAL_PRUNE`, `_MAX_PATH_M`, `_MIN_PATH_M` (the full one at the end of the mission stays; the options stay in `close_loops_checked` for `prova_anelli.py`). In the report the wait before the return to wp_0 no longer counts the final loop closure, which has its own limit (on 29-10 it was marked DA GUARDARE because of this).
  - *Log*: on 29-10 10:57 the log had 5 589 lines. Removed or shortened: the 12-line block for every waypoint created (now one `[WAYPOINT]` line), the progress of the map download (405 lines, now 2), `[GRAPH_CACHE]` (168), the search of the neighbours `[ADJACENT]` with the `Vicina` lines (~370, printed twice per turn), the raw frontier without a tag, `[CHECK]` inside/outside the cell, `[MAP]` maps saved and size, `[INFO] Passo 1/2` and `Spostamento completato` (duplicates of `[MOVIMENTO]`), `[SLOPE-SOURCE]` and `[TILT]` per leg (the same numbers are in `segments.csv`), 436 empty lines.
  - *Files no longer written*: `iteration_*_obstacles.npy` and `_terrain.npy` per attempt, which no tool read (the same data is in the `_vis.npz` views and in the `scans/` bundles).

- **Live viewer** (`visualizzatore_live.py`, new). During the mission there was no way to see what the robot was thinking except reading the log. Now, in a second terminal, a figure that updates itself when a scan or a new log line arrives: map seen so far, latest scan, cells, PRM graph, path taken, planned path, goal, safe frontier, robot, and on the right the state and the last useful log lines. It only reads the files the mission already writes (`scans/*.npz`, `iteration_*_vis.npz`, `mission_log.txt`, `trajectory.csv`): the mission code does not change. It redraws only when there is something new (~0.4 s of CPU per drawing). With `--rigioca` it replays a finished mission, with `--png` it saves a figure (also at a given time, `--fino-a`). Tested on the mission of 28-10 14:23.

## 2026-10-28 evening (after the outdoor tests)

Towards the version on `main`: the small corrections that came out of the outdoor tests, each with its own test without the robot.

- **Failed replan: new goal at once** (`REPICK_ON_REPLAN_FAIL`, `REPLAN_FAIL_ROBOT_FREE_NODES = 6`). 14:23: 20 `[REPLAN FALLITO]` (the updated graph no longer has a path to the goal) out of 178 replans; the broken plan was kept and the frontier decided. At 14:37, 14:41 and 14:48 the goal had been left isolated in the graph with 19-21 free directions out of 24 around the robot: two `[SUBITO]` with "the graph offers no alternatives", three still scans, block and cell deferred (cell (4,2) attempted four times towards the same point, then blocked); at 14:27 and 14:34 one more partial step on the broken plan before realizing the goal was inside an obstacle. Now, if from the robot the graph still reaches at least 6 nodes (it is the goal that is isolated, not the robot), another goal in the cell with a path is chosen at once (`_riscegli_obiettivo`, in the same turn, without an extra scan); if there is none, as before. Tests: e2e all OK (S4 and S5 change: S5 arrives in the cell instead of blocking and retreating 3.6 m); fake missions A, B, C same cells closed with 1-5 fewer turns; sweep test of the loop 160/160 cells (before 158: cases 65 and 88 now succeed), 39 fewer turns, minimum distance from obstacles unchanged (0.256 m); 40 hard missions: all finished and returned, 344 cells visited instead of 342, 341 attempts instead of 344. On the robot on 29-10 10:57: triggered once, worked.
- **Goal chosen again without a path: consistent state.** Sweep test of the navigation loop on 160 random worlds (obstacles, starts and different graphs): no exception, no stall, minimum distance from obstacles 0.256 m in all cases, 158 cells reached out of 160. In one of the two missed cases the centre had 0.24 m of space, the new goal had no path in the graph, and the NEW goal stayed with the OLD path, towards the point just excluded. Now up to `GOAL_REPICK_CANDIDATES` = 3 candidates are tried, and if none has a path goal and path go back to the previous ones. The other missed case is a 0.7 m gap without nodes on its line (graph density in narrow passages, already in `TODO.md`).
- **Emergency shutdown.** If the mission is interrupted with the robot standing (exception, Ctrl+C), before exiting the robot is stopped, sat down and powered off, and the GraphNav map is downloaded (`_chiusura_di_emergenza`); then the error propagates as before. Before, the program just exited: robot standing without commands until the software E-Stop, no longer renewed, cut the power (~9 s), and GraphNav map lost (it is erased at the start of the next mission). If the robot has fallen it is not sat down. Test: `test_missione.py`, world F, interrupted by an exception and by Ctrl+C at the third attempt. On the robot on 29-10 15:31: Ctrl+C halfway through, robot sitting, motors off, map downloaded.
- **`[SUBITO]` repeated on the same leg.** The second time `[SUBITO]` falls on the same leg in the same attempt, the leg leaves the plan for the rest of the attempt (attempt penalty, like the returns and `[ARRIVO]`). 14:23: 8 `[SUBITO]` out of 14 were repetitions (197->569 four times in a row): the "for now" discard was undone by the refresh, because the preliminary discard (one scan, 0.25 m) allows the leg that the frontier (median of three, hysteresis up to 0.29 m) stops. Test: scenario S4.
- **Superfluous code removed** (inventory of 27-10): interpolated slope estimate and sample coverage (lines `[SLOPE-CHECK]`, columns `long_interp`, `lat_interp`, `coverage_along`, `coverage_lateral` of `segments.csv`, which no tool read), `[FOOTPRINT-DIAG]` diagnostics, `env.waypoints` and the `[WAYPOINT]` line, the two never-used `load_map`, `RETREAT_AFTER_ABORT_M`, `NOTE_2026-10-08_grafo_multinodo.md` (contents in the 2026-10-08 section below and in the git history). Only those lines disappear from the test logs.
- **Return to wp_0 with three outcomes** (`navGraphUtils.graphnav_return_to_start`, `GRAPHNAV_RETURN_NEAR_M = 1.0`). 13:50: GraphNav stopped (`STUCK`, tall grass around the start) 0.63 m from wp_0; counted as a failed return and nothing else, without fiducial check or drift measurement. Now: `arrivato` as before; `vicino` if GraphNav stops within 1 m of wp_0, which stays an **unsuccessful** return (in the log `Ritorno a wp_0 NON riuscito ma vicino: fermo a D m da wp_0 (motivo: …)`, in the report "NON riuscito") but with fiducial and drift measured anyway; `non arrivato` beyond, with distance and reason in the log. The distance is GraphNav's if it is localized on wp_0, otherwise that of the VISION poses. `test_missione.py`, world C: fake return stopped at 0.60 m.
- **Leg not completed but arrived** (`MOVE_NEAR_GOAL_M = 0.10`, line `[QUASI]`). 13:50, cell (3,2): 20° climb in HARD, 0.42 m out of 0.44 travelled, but Spot did not settle within the 15 s allowed; the command expired and the leg was treated as a **physical failure**: edge discarded for the whole mission (`mark_edge_invalid`) and going back. Now, when a command ends badly without the frontier having stopped it, and the robot is within 10 cm of the point (and has done at least half the leg), the leg counts as successful: it carries on from the node, the edge stays valid. `segments.csv` keeps the real outcome of the command; the summary counts these legs. 10 cm is a first choice, to be checked in the field. `test_e2e_loop.py`, scenario S17: command expired at 3 cm (counts as arrived) and at 30 cm (physical failure as before).
- **GraphNav trips arriving turned towards where to go** (`GRAPHNAV_ARRIVAL_HEADING = 'verso_obiettivo'`, `navGraphUtils.graphnav_arrival_heading`; it was idea 2 of 28-10). GraphNav took the robot to the waypoint pose, yaw included, i.e. to the heading it had when it created it, often the opposite of where it must go next. Measured on `trajectory.csv` of 26-10 15:11 (8 arrivals): GraphNav hardly rotates at the end (0–23 degrees, because already in the approach it walks backwards or sideways to keep the waypoint heading), but right after the robot rotated in place on average **79 degrees** for the first move (up to 129), 38 s in all. Leaving the final heading free (`ignore_final_yaw`) does not help: it would have been 82 degrees on average, sometimes better sometimes worse. The direction towards the cell goal instead predicts the first move with a mean gap of **22 degrees**. Now in the trips between cells the robot arrives turned towards the centre of the cell to attempt, in the returns to breadcrumbs towards the goal of the attempt (`destination_waypoint_tform_body_goal`, angle only; waypoint position unchanged); the final return to wp_0 keeps the starting heading. `'waypoint'` = as before, `'libero'` = without final rotation. The report has the line **"Rotazione dopo gli arrivi GraphNav"** (mean degrees and seconds in place), for the comparison with the 79 degrees and 38 s of 26-10. Expected gain: 2–4 s per arrival if GraphNav turns while walking; to be checked. `test_missione.py` checks that the trips ask for the heading and the return to wp_0 does not. On 29-10 10:57: 4 degrees on 1 arrival.
- **End of exploration explained in the log.** 14:17: start in the middle of tall grass (obstacles on 54-67% of the cells seen around the robot, 4-15 cm high, intensity equal to the ground), no cell reachable, mission over in 21 s without moving and in the log only "Frontiera vuota". Now a `[FRONTIERA]` line for every cell removed from the frontier, and at the end of exploration `[FINE] Nessuna cella raggiungibile dalla partenza … Spostare il robot su terreno libero` (or how many cells are left and which were removed). `test_missione.py`: world E, start closed all around.
- **Cell choice: no cell lost from the frontier.** 13:50, cells (7,0) and (7,1): (7,2) had been visited with the robot 0.29 m outside (in (6,3)), and the loop looked for the neighbours from the cell the robot was in, not from the one just visited: (7,1), next to (7,2), was never attempted from there. Then, to approach with GraphNav, the first visited neighbour in a fixed order (north, south, west, east) was always taken: for (7,1) it was (6,1), side already tried, so skipped; without practicable candidates the frontier lost one cell per turn, and (7,0) and (7,1) left without other attempts, despite the `[DEFER]` ("it will be retried from another side"). Now: if the last visited cell was accepted with the robot just outside and the robot is still there, the neighbours are looked for from that one (`Missione._cella_del_robot`); and among the visited neighbours the one whose side towards the candidate has not been tried yet is chosen, the nearest to the robot (`Missione._cella_di_partenza_per`). Test: `test_missione.py`, direct check of the 13:50 case on the 8 x 7 grid; world C (forced failures) from 5/9 cells closed (4 never reached, 7 skips) to 9/9 (7 visited, 2 blocked); worlds A and B with one GraphNav trip fewer. The end-to-end tests and the replay do not change (the loop inside the cell is not touched).

## 2026-10-28 evening: battery

- **Battery charge and consumption, with low-battery return** (`batteria.py`, new; it was idea 2 of 28-10 in `TODO.md`). Before there was nothing in the code: `power_state` was only read to know whether the motors were on.
  - *Measurement.* The charge is read from the robot state (`battery_states`, or `locomotion_charge_percentage` if missing) at start-up, at every turn of the navigation loop and of the exploration (at most one reading every `BATTERY_SAMPLE_S = 5` s), after every GraphNav trip between cells, before and after the return to wp_0. Every reading goes into **`battery.csv`** (charge, runtime estimated by Spot, voltage, current, temperature, metres travelled, mean consumption in %/min and %/m); in the log a `[BATTERIA]` line at every point lost. The summary and the report have a "Batteria" line, with how many points the return to wp_0 cost: it is the number to tune the threshold.
  - *Return.* Below `BATTERY_RETURN_PCT = 30%` (provisional), confirmed over 2 readings in a row, the current attempt stops and the mission closes as at the end of exploration: final scan, summary and `mission_summary.json`, loop closure and anchoring, return to wp_0 with GraphNav, sit, map download. The cells not explored yet stay in the frontier (an attempt interrupted for the battery does not mark the side as tried); how many there are goes into the log and into `battery_return_cells_left`. `BATTERY_RETURN_PCT = 0` turns the return off (the measurement stays).
  - If the robot state does not report the charge, the mission goes on without a return (one line in the log). The metres are the sum of the straight-line moves between one reading and the next: on long GraphNav trips they are few, so the consumption per metre is an overestimate.
  - `test_missione.py`: world D, world A with a fake battery losing one point per reading from 36%: the return triggers at 29%, exploration stops with 3 cells in the frontier, the return to wp_0 succeeds. Never tried on the robot.

## 2026-10-28 (after the indoor test of 10:27)

- **`DA_DECIDERE.md` is now `TODO.md`.** Same content and same role, widened from "postponed decisions" to "ideas, open problems and things to do": it was already what it contained, and the old name did not say so. Link in the `README.md` updated. No change to the code.
- **Viewer: the global map panel now also uses the start-of-attempt and final scans.** It rebuilt it only from the `scans/` bundles, which contain the navigation loop scans; the start scan of every attempt went into the robot's map but not there. At 10:27 the attempt on cell (0,5) failed before moving (no way in the graph), and its start scan was the only one that had seen the wall in front: for the robot it was there, in the viewer not. Those scans are rebuilt from the PRM views (`iteration_I_cell_R_C_vis.npz`, `iteration_I_FINAL_SCAN_vis.npz`), with a single approximation (see `_VIS_SCAN_NAME`). Obstacles of the robot's map found again at the end of the mission: 28-10 from 77% to 98%, 26-10 from 87% to 97%, 22-10 from 90% to 96%. The mission code does not change. The same scans can now also be **browsed**, among those of the loop in time order, with the title "inizio del tentativo sulla cella (r,c)" or "scansione finale": at 10:51, between scan 11 (towards the goal of (0,4)) and the old 12 (after GraphNav) there are the two scans from the arrival point, those of the attempts on (0,5) and (1,4) that failed without a way. Robot heading from `trajectory.csv`.
- **Mission progress** (idea 1 of `TODO.md`). A `[PROGRESSO]` line every time a cell is visited or blocked, and one at the end: closed cells (visited + blocked, starting cell included) over the total, mission area already seen in the global map (points every 0.1 m), minutes elapsed, estimate of the minutes left from the mean time per cell, battery read from the robot. In `mission_summary.json` (`mission_stats.progresso`) and in the report (`Avanzamento: …`, with a warning if the mission was interrupted). Log only: no decision depends on it, and in the fake mission only added lines change. The time estimate is an overestimate: unreachable cells leave the frontier without costing time.
- **Check on the arrival pose (`[ARRIVO]`): the unknown is not a wall.** Indoor test of 12:16: the corrections on the goal worked (no `[SUBITO]`, no block, one `[PASSO FINALE]`), but in cells (0,2) and (1,1) the same step was refused three times in a row ("0/24 direzioni utilizzabili") and executed at the third, and it was the right step. The fan from the arrival point counted as closed the forward direction stopping at the edge of the scan (0.09 m, `fine_dati`): in a corridor no exit was left. Now a turnable direction that stops because the data runs out counts as an exit (`arrival_has_exit`); a stop on an obstacle does not. The cap of 3 refusals hid the error: on 22-10 there were 10 refusals (2 up to the third), on 26-10 5 (1). A real refusal now excludes the leg for the rest of the attempt (attempt penalty, like the returns), instead of only until the next refresh, which put it back in the graph. Test: scenario S16 (corridor continuing beyond the data: exit; blind niche all seen: no exit; mission in the 0.9 m wide corridor: no refusal, 7 scans instead of 10). Over 18 corridor and niche cases: refusals from 23 to 0, cells reached 13 instead of 14. The missing one is a path that goes back with an impossible rotation: the limit of the planner that ignores rotations, not of the check.
- **Goal right next to an obstacle: three corrections of the motion logic.** On 28-10 11:36, cell (0,2): the obstacle was 0.17 m BEYOND the goal, where the frontier checks the nose (path extended by half a body). The goal check only looked at the free space on the point, so the legs were blamed: 4 `[SUBITO]` on the same leg (the refresh put it back in the graph), 3 waits, block, goal excluded but path again towards it, impossible U-turn, return with GraphNav: about 50 s. It did not depend on the restructuring: on 22-10 16:04 45 `[SUBITO]` out of 58 had the obstacle within 0.35 m of the goal, on 26-10 15:11 18 out of 22.
  1. At the first frontier stop for an obstacle within `REFUSED_GOAL_BLOCK_M` of the goal, or beyond the end of the path, another goal is chosen at once (`TentativoCella._riscegli_obiettivo`, the same as for the rule on the free space of the point).
  2. Final step: a step below `MIN_PARTIAL_STEP_M` (at least `FINAL_STEP_MIN_M` = 0.10 m) is taken if it brings the robot within the arrival tolerance with the robot in the cell (`[PASSO FINALE]`). At 11:36 the robot was still 0.85 m from the goal while able to advance 0.38 m.
  3. After a block confirmed right next to the goal another one is chosen, instead of discarding the leg and replanning towards the one just excluded.
  Test: scenario S15 of `test_e2e_loop.py` (obstacle right beyond the goal, random nodes). On the same 24 cases the previous code failed the cell 7 times, with 112 `[SUBITO]` and 16 blocks (seed 9: 143 commands, 150 scans); the new one reaches it 24 times out of 24, without `[SUBITO]` or blocks, with a median of 8 scans instead of 11. The other scenarios, the fake mission and the replay of all missions are identical. The limit of the planner remains, which does not take rotations into account (U-turns in narrow corridors): it goes with the smoother motion.
- **`prova_fiducial.py`** (new): which fiducials Spot sees now and which the mission would choose, read only (no lease, robot still, even sitting). Tested: tag 521 at 1.13 m behind the sitting robot, camera `back_fisheye`, in 6 readings out of 6.
- **Any starting fiducial, no longer only 549.** At start-up the nearest fiducial seen by any camera is searched, the rear one too (`navGraphUtils.find_fiducials`; `START_FIDUCIAL_ID = None` = any ID), and written in the log and in `mission_summary.json`. The localization on the fiducial is now requested AFTER creating wp_0, so the tag is in its snapshot: before it was requested on a map where the tag was not there yet, and always gave `STATUS_NO_MATCHING_FIDUCIAL` (26-10 and 28-10; no snapshot of the graphs of 22-10, 26-10 and 28-10 contains a fiducial). Back on wp_0 the tag is searched again: the shift of its position in the VISION frame is the odometry drift of the whole mission (`[FIDUCIAL] Al ritorno su wp_0 …`, statistic `start_fiducial_drift_m`). `test_missione.py`: in world A a fake tag behind the starting point.
- **`prova_grafo.py` uses the mission parameters** (`parametri.py`) instead of its own numbers: the image shows the graphs the mission would really build, and for each one the lines of `parametri.py` or the arguments of `easy_walk.py` to choose it are printed (same seed = same graph relative to the starting point, checked). The TEST block stays to try other values.

## 2026-10-27 (restructuring, behaviour unchanged)

`easy_walk.py` from 5611 to about 2900 lines; the robot's behaviour does not change. Every step checked with: end-to-end test logs **identical line by line** to the reference recording in all 19 scenarios (`E2E_LOG_DIR` + `confronta_log.py`, new), replay of the 10 missions identical to the reference, `test_obiettivi.py` identical, static check of the names, `preflight.py`; the GraphNav parts, which no test exercises, tested with a fake GraphNav (move, block, return to wp_0, loop closure, drift).

- **`parametri.py`** (new): all the mission switches and numbers, including mission size and graph (before in `mission_graph.py`). The code reads them as `parametri.NOME` at the time of use, so command-line arguments and tests can change them.
- **`registro.py`** (new): statistics, copy of the output to file, CSV, tilt recorder, saved bundles and views.
- **Moved into the existing modules:** scan and global maps into `spotGrid.py`; gait, leg with stop check and falls into `movements.py`; graph construction and update into `mission_graph.py`; functions on cells into `environmentMap.py`; decisions on the frontier (next move, rotation, cone, back-off, fan, trap, goal still reachable) into `arcVerification.py`; all the mission's GraphNav into `navGraphUtils.py`; `mark_edge_blocked_soft` into `prm_graph.py`.
- **`spotGrid.py` can be imported without the Spot SDK** (only `LocalGrid` needs it): `mission_graph`, `prm_graph`, `parametri` and the viewer also work where the SDK is not available. The viewer reads the map and threshold constants from `spotGrid` instead of copying them.
- **`preflight.py` rewritten:** the references between modules (`module.name`, `from module import name`, keyword arguments) are checked automatically, without tables to update; only the methods called on objects remain by hand.
- Removed `ROTATION_ROOM_SEARCH_M` and `ROTATION_ROOM_DIRECTIONS_DEG`, which nobody used.
- **`DA_DECIDERE.md`** (new): open questions and decisions to take after the test of 28-10.
- **Entering a cell is the class `TentativoCella`** (before a function of about 1500 lines): `_prepara`, then at every turn the phases `_guardie`, `_scansione_e_grafo`, `_fronte_e_decisione`, `_rotazione`, `_tempi`, `_nodo_o_arrivo`, `_attesa`, `_blocco`, `_esegui_tratti`, and at the end `_fine`. The 7 copies of the return with GraphNav and of the counter reset are the method `_ritorno`. The scan processing chain (before written 4 times) is `LocalGrid.fetch_scan` + `process_scan`.
- **The cell-by-cell mission is the class `Missione`** (before inside `easy_walk()`, 590 lines, now about 90): `esplora`, `_cella_confinante`, `_recupero`, `_scegli_cella_lontana`, `_viaggio_e_tentativo`, `scansione_finale`.
- **`test_missione.py`** (new): the whole mission with the real `easy_walk.main()`, fake robot and GraphNav, in three worlds (the third with forced failures: new attempt after the recovery, no goal, fall during a trip). Logs identical before and after the restructuring.
- **Fix found by the mission test:** after a successful entry with the robot just outside the cell (arrival up to 0.35 m outside), the new candidate cells were looked for around the cell the robot was in instead of around the one just visited.
- Comments: a header for every module, references to moved functions updated, `import *` replaced. The keys of the switches in `mission_summary.json` are now `parametri.NOME` (before `easy_walk.NOME`); the report only uses the final name.

---

## 2026-10-26 evening (after the outdoor test of 15:11)

Outdoor test: 8 × 7 cells of 2 m, **55 cells out of 56 in 25 minutes** (Thursday 59 out of 64 in 42), wait before GraphNav trips about 8 s in all (Thursday 603), no "outside the cell" refusal (Thursday 44), 12 goals excluded for the mission and never proposed again, no fall. Quick closure before the trips 0.0–2.0 s, 13 edges. Only problem: the map closure took 141 s, and the full one gave 2 edges.

### Spot loop closure (`easy_walk.py`, `navGraphUtils.py`)
- **Spot computes without writing, we upload the edges** (`close_loops_checked(upload_result=True)`, `upload_graph` of the edges found). On 26-10 15:36, between the stop of the recording and the line of the full closure, 138 s went by, but the line declared 65: the previous attempts had failed without writing anything in the log, almost certainly with `MapModifiedError`, which Spot gives AT THE END of the processing when it tries to write into the map. Every failed attempt cost the whole processing. Without writing on Spot's side that error cannot happen; if the upload failed, `LoopClosureUploadError` says how many edges were lost.
- **Every failed attempt ends up in the log** with cause and duration (`… tentativo N non riuscito dopo X s (…)`), at most a second attempt and only for `MapModified`. The time reported is the total of all attempts. Statistic `sdk_loop_failures`; the "Attesa prima di GraphNav" check of the report counts them.
- **Quick closure before the trips TURNED OFF** (`GRAPHNAV_SDK_LOOP_BUDGET_S = 0`); the full one at the end of the mission stays (45 s). Series 3 on the bench on the graph of 15:11 (`MissionLogs/prova_anelli_26-10-2026_16-48-03`): the full one finishes by itself in 62–71 s with **0 edges**, with or without `prune_edges` (`GRAPHNAV_SDK_LOOP_FINAL_PRUNE`, left at Spot's value), with 45 or 90 s. Spot looks for pairs close in space but far along the graph: the quick edges and our shortcuts had already shortened those loops, and Spot never rechecks existing edges. The two closures exclude each other. The full one is kept, because its edges are corrected on the scans and are the only ones that correct the drift of the downloaded map (17 in 45 s on the bench, series 2, on a graph without quick edges). The trips use our shortcuts: on 22-10 23 trips out of 23 with those alone; on 26-10 the return to wp_0 used 8 of our shortcuts and none of Spot's.
- **Measurement of the odometry drift at every GraphNav arrival** (`graphnav_log_drift`, line `[DERIVA]`, file `drift.csv`, statistics `drift_*`, check "Deriva dell'odometria" in the report). GraphNav localizes on the scans of the arrival waypoint and says where the body is relative to it; composed with the VISION pose of the waypoint at its creation it gives where the robot would be without drift, and the difference from the current VISION pose is the drift accumulated since then. It is the shift our global maps (in VISION) have over the areas seen earlier, and that no loop closure corrects: the final one only realigns the downloaded GraphNav map. On 26-10 15:11, on arrival at wp_0, the raw offset was 0.66 m after 25 minutes, arrival tolerance included (this measurement excludes it). Measurement only, no correction: measurable on the waypoints we created (`waypoint_poses` now also saves the creation instant).
- **Upload fallback:** with 0 edges series 3 could not test `upload_graph` of the edges on the robot. If the upload were rejected, the second attempt has Spot write the edges itself (as in series 1, where it worked).

---

## 2026-10-26 (after the outdoor test of 22-10 16:04)

Outdoor test: 8 × 8 cells of 2 m, 59 cells out of 64 in 42 minutes, no fall. Three problems.

### GraphNav (`easy_walk.py`)
- **Spot loop closure: quick before every move, full once at the end of the mission.** Thursday: 20 timeouts, **603 s of waiting before the moves against 257 s of travel**, 1 edge in the whole mission. Two series of bench tests (`prova_anelli.py`, graph of 22-10 16:04 reloaded on the still robot, `MissionLogs/prova_anelli_26-10-2026_12-41-53` and `_13-02-58`):
  - **full (with ICP):** without a limit Spot stops by itself after ~30 s, together with the RPC, and that is why on Thursday no result arrived. With a 4 s limit: 1–4 edges in 5–8 s, never finished, at any graph size. Repeated calls do not resume the work and get longer (3, 1, 1, 1 edges in 5.2–12.3 s). The limit is approximate (45 s → 59 s, 120 s → 152 s; once 60 s → over 120 s).
  - **quick (without ICP or feature matching):** FINISHED in 0.4 s at 100 waypoints (6 edges), 2.0 s at 200 (12), 4.0 s at 388 (24). The cost was all in the ICP. The edges are not corrected on the scans (pose from odometry, like our shortcuts) but they pass Spot's collision check: they are extra shortcuts over the whole graph. Without feature matching only, nothing changes; "only loops beyond 8 m" is slower.
  - **at the end of the mission, full:** 45 s → 17 edges, 60 → 18, 90 → 21, 120 → 23. Beyond 45 s about one edge every 10–15 s.

  So: before every GraphNav move quick mode with a 3 s limit (`GRAPHNAV_SDK_LOOP_BUDGET_S = 3`, `GRAPHNAV_SDK_LOOP_FAST = True`); at the end of the mission, **before `optimize_anchoring`**, full with a 45 s limit (`GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S`), so the edges corrected on the scans correct the drift of the downloaded map. RPC limit: three times Spot's, at least +15 s. Line `[GRAPHNAV] Chiusura d'anello di Spot (veloce, senza ICP | completa): N archi in X s (limite Y s, …)`, statistics `sdk_loop_*`. **Not measured:** repeated quick calls on a map that changes (the full one slowed down); it is turned off with `GRAPHNAV_SDK_LOOP_BUDGET_S = 0`.
- The shortcuts from the global map stay: 23 successful trips out of 23, localizations rejected by Spot from 641 to 644 over the whole mission. The odometry of the edges and the VISION frame of the map differ by at most 2.7 cm over 251 waypoints. The drift relative to the world, estimated from GraphNav's localization edges, reaches 0.39–0.47 m between waypoints created 15–20 minutes apart: to keep an eye on in long missions.
- **Return without movement**: if all breadcrumbs are less than 0.40 m from the robot, `graphnav_step_back` does not call GraphNav and replans standing still (line `[RITORNO] La briciola piu' lontana dietro … non mi muovo`). On 22-10 16:22 the robot was on top of wp_138 and the "return" move was 0 m. The other three returns "at 0.0 m" of the log were instead real moves of about 0.6 m: the 0.0 was the distance in the GraphNav graph from the waypoint the robot was localized on. Now every return writes the real distance.

### Choice of the cell goal (`easy_walk.find_best_point_in_cell`)
- **First the points the graph connects to the robot.** The robot node is attached before the choice; the candidates (centre and nodes of the cell) are tried in order of preference until one is reachable. If none is, the best one is chosen as before and the attempt fails as before. **Removed the 3 spare attempts at the start of the attempt** (22-10 evening): they chose the points nearest to the centre, in the same isolated area, and failed 9 times out of 10.
- **Preferred free space 0.33 m** (`TARGET_CLEARANCE_M`), on the global map and live; 0.28 stays the minimum. Goals chosen at 0.28 were refused live at 0.24, at the edge of the grass: the frontier wants 0.25 and the hysteresis is 0.04.
- **At least 0.30 m from the cell border** (`TARGET_CELL_BORDER_M`), when there is something better. On 22-10 there were goals 0.98–1.01 m from the centre of 2 m cells.
- **A goal refused by the frontier stays excluded for its cell until the end of the mission** (`_refuse_goal`): when the live free space is below 0.25, or when a block is confirmed within 0.35 m of it (`REFUSED_GOAL_BLOCK_M`). It is allowed again only if nothing else is left in the cell. On 22-10 node 548 of cell (7,4) was chosen again at every attempt and blocked 3 times at the same point, with 5 GraphNav returns in between.
- Order of the criteria: not refused > reachable > space ≥ 0.33 > centre > seen free > far from the border > close to the centre > close to the robot. The `[TARGET]` line says why the centre was discarded and counts the tight ones, those on the border and the refused ones.

### Arrival (`easy_walk.py`)
- **Arrival by tolerance also counts with the robot just outside the cell, up to 0.35 m** (`ARRIVAL_OUTSIDE_CELL_M` = tolerance 0.65 − margin from the border 0.30; the nose, 0.55 m ahead, is inside). It corrects the rule of 22-10 evening, too strict: outdoors 44 refusals "Mancano X m … ancora fuori dalla cella", in 8 attempts, with the robot outside by 0.01–0.61 m and 0.5–0.65 m from the goal (the frontier stops the centre half a body before). With the new rule 7 attempts out of 8 would have closed. A first version accepted the robot within 0.25 m of the goal: it would have closed 3 cases out of 9, discarded.

### Viewer (`mission_viewer.py`)
- **Starting height visible** in the "terreno" and "fusione" backgrounds: white level line at height 0 (the ground at mission start, heights averaged over 15 cm only for the line, otherwise the noise on the plane fills it with rings) and zero marked on the colour bar, with the negative end. The scale always contains zero. The heights below the start were already there (on 22-10 16:04 down to −0.25 m, about 5% of the cells), but in viridis they could not be told from "the lowest".

### Data for the grass
- **Spot's `intensity` and `no_step` layers in the scan bundles** (`SAVE_EXTRA_LAYERS`), with a separate request (`spotGrid.LocalGrid.extra_layers`) that does not go through the alignment checks: if they are missing or have another grid, the scan used for navigation is not affected. Next to each one `<name>_origin` and `<name>_cell`. They serve to understand whether they tell vegetation from a solid obstacle.

### Tests and tools
- **`prova_anelli.py`** (new, series 1 at 12:41 and series 2 at 13:02 on 26-10, results above): bench test of the loop closure, robot still. Series 2: graph rebuilt at 100/200/388 waypoints, without ICP, without feature matching, only loops beyond 8 m, and at the end of the mission limits from 45 to 120 s. It uploads a saved graph (default the one of 22-10 16:04) and measures time and edges: as on Thursday waiting for the end, with a 4 s limit, with path ≤ 20 m, without collisions, with a 20 s limit, and repeated 4 s calls that modify the map (does the work continue or restart?). Results in `MissionLogs/prova_anelli_<date>.txt/.json`.
- **`test_obiettivi.py`** (new): synthetic checks of the rules, then replay of the scans redoing the goal choice at every attempt start, compared with that of the mission. 22-10 16:04: goals refusable by the frontier 20 → 15, proposed again after a refusal 3 → 0, reachable 63 → 64. Indoor missions: no worsening, goals on the border 7 → 0 (13-10 12:14) and 3 → 0 (13-10 12:40).
- `test_e2e_loop.py`: S1 also checks the saving of the extra layers (one present, one absent); scenario **S14**, cell free only along the border, with obstacles already in the global map (`memory`): the robot stops 0.14 m outside and the arrival counts.
- `mission_report.py`: three new checks. **Wait before GraphNav** (wait against travel, loop closure timeouts), **Goal choice and arrival** (excluded, proposed again, without reachable points, arrivals just outside, refusals outside the cell), **Returns with GraphNav** (real distance, returns standing still). On 22-10 16:04 the first two give DA GUARDARE, as expected.
- New statistics: `goals_refused`, `goals_unreachable`, `arrivals_near_goal`, `backtracks_in_place`; line `[CONFIG] SCELTA OBIETTIVO` at start-up.

### Data in the repository
- **The 13 validation missions are on GitHub** (the 10 of `test_replay.py --tutte` and the three of 22-10): logs and CSV, `scans/`, `global_occupancy_map.pkl`, `global_terrain_map.pkl` (slopes) and the full GraphNav map (`graph`, `waypoint_snapshots/`, `edge_snapshots/`), which can be reloaded on the robot (see `prova_anelli.py`). Also the planner views (`*_vis.npz`, 390 files, 340 MB) for the viewer. About 2.1 GB in all; only the other files saved at every iteration are missing (`iteration_*.npy`, `*_BLOCKED_*.npz`). Added with `git add -f`: `MissionLogs/` and `MissionMap/` stay ignored, so new missions do not get in by themselves.

### To discuss
- **Tall grass**: Spot marks the tufts (+0.15/+0.45 m above the ground) as obstacles; 25% of the map occupied, cells (4,6), (7,2), (7,3), (7,4), (7,7) not reached.

---

## 2026-10-22 (afternoon, after the indoor tests of 15:24 and 15:50)

### Planner and maps (`spotGrid.py`, `prm_graph.py`)
- **The planner leaves 3 cm more than the frontier** (`spotGrid.PLAN_CLEARANCE_EXTRA_M = 0.03`, so `PLAN_CLEARANCE_M = 0.28`). It applies to the edge veto on the global map (margin from 0.10 to 0.13 m), to the choice of the goal point and to the attachment of nodes. Test of 15:24: 18 frontier turns out of 25 stopped by an obstacle at 0.23–0.24 m with a minimum of 0.25, because the paths passed exactly at the limit and the scan noise was enough. On 13-10 12:40 too all the stops for obstacle were within 3 cm of the minimum.
- **Live edge discard from 0.20 to 0.25 m** (`prm_graph.LIVE_PREFILTER_OD_M`, equal to the frontier distance): before it let through edges at 0.20–0.25 m, which the frontier then refused. Not 0.28: from a tight spot the exit edges necessarily pass near obstacles, and the frontier allows them (in the first metre it is enough not to get closer). With 0.28 the replay of mission 13-10 11:22 lost the path from the last cell start (4/5); with 0.25 all 82 are found. The test of 15:50 started with 0.28.
- Replay of the 10 missions: no invariant violated, valid edges −0.7% on average (up to −2.5%), 11 paths a bit longer (+1.5 m in all), gaits and return to wp_0 unchanged. Reference updated.

### Navigation (`easy_walk.py`)
- **The cell centre is also discarded if the live scan sees it less than 0.28 m from an obstacle**, not only if it is occupied in the global map (which confirms an obstacle after two observations). Test of 15:50: the centre of cell (0,2) was 0.15 m from an obstacle just seen and "free" for the map. Its node had no edges, the attempt failed in one second with "GRAFO DISCONNESSO" and the robot did not go further into the corridor. With the new rule, on the same data, a point 0.36 m from the centre is chosen, connected and reachable. The learned free space is not used for the centre, because it is still being tested.
- **If at the start of the attempt the graph does not reach the goal, up to 3 other points of the cell are tried** (`MAX_BLOCK_REPLANS`) before giving up, as the navigation loop already did.
- **Arrival by tolerance counts only inside the goal cell.** Test of 15:24, first attempt on (1,1): fallback point 0.83 m from the centre, robot stopped 0.39 m before, "goal reached" but 6 cm outside the cell, and attempt failed. With 2 m cells a fallback can lie near the border. Outside the cell it carries on as for a blocked frontier.
- **Check on the graph before the GraphNav move** to the waypoint from which to attempt a cell (`GRAPHNAV_PRECHECK`, line `[PRECHECK]`): if from there the graph reaches no node of the cell, the side is marked as tried without moving. Test of 15:24: 3 moves out of 6 ended in a failure in less than a second, about 25 s out of 3 minutes. Tested on the saved graphs: it says "not reachable" exactly for those 3 and "reachable" for the 2 that led to a real attempt. In the test of 15:50 it triggered once.
- New switches `PLAN_CLEARANCE_EXTRA_M` and `GRAPHNAV_PRECHECK` in `mission_switches()`; statistic `graphnav_trips_avoided`.

### Tools
- `mission_report.py` classifies the frontier stops with the distances of the mission (from the recorded switches: frontier 0.25 + air, live discard), no longer with 0.30 and 0.20 written by hand; it shows the GraphNav moves avoided.

### To discuss (motion)
- Test of 15:50: to enter (1,1) and (1,0) rotations of 60–150 degrees were needed inside a corridor about 1 m wide. Turning in place Spot sweeps a circle of about 1.2 m, so the rotations were impossible, and 3 returns with GraphNav followed. Turning while walking would solve it.

---

## 2026-10-22

### Mission graph (`mission_graph.py`)
- **Choice of the node method**: `GRAPH_SAMPLING = 'reticolo'` (default, unchanged) or `'casuale'`, with four distributions (`uniforme`, `poisson`, `perturbata`, `halton`), density in nodes/m² (default 16, like the lattice at 0.25 m) and seed. Seed not given = drawn at random; the one used always goes into the log, with the command to repeat the graph, and into `mission_summary.json` (entry `grafo`). The cell centres stay fixed nodes with random nodes too. The edges link the pairs between minimum and maximum length; a cap on neighbours per node will be evaluated later. The coverage of random nodes is measured on a 5 cm grid. It serves to compare the two methods; the lattice is bit-for-bit identical to before.
- On the default mission (3 × 4 cells of 4 m, edges 0.5–1.25 m, seed 7):

  | method | nodes | pairs | maximum coverage | 95% of the area within |
  |---|---|---|---|---|
  | lattice 0.25 m | 3185 | 105250 | 0.18 m | 0.14 m |
  | uniform | 3084 | 94299 | 0.56 m | 0.25 m |
  | poisson | 3084 | 92194 | 0.25 m | 0.17 m |
  | jittered | 3084 | 93509 | 0.32 m | 0.19 m |
  | halton | 3084 | 93732 | 0.36 m | 0.19 m |

  The lattice has more nodes because it includes the rows on the border of the area.

### Indoor test: new defaults (`mission_graph.py`)
- **10 × 10 mission of 2 m cells** (20 × 20 m) and **poisson random nodes at 5 nodes/m²** (`GRAPH_SAMPLING = 'casuale'`), seed drawn at random and written in the log. On the default mission: 2100 nodes, 20861 edges (the 0.25 m lattice on the 3 × 4 × 4 had 105 thousand), built in 0.6 s, connected, 17–24 nodes per cell, coverage 0.44 m (95% of the area within 0.29 m).
- `test_e2e_loop.py` builds the graph with an explicit `sampling='reticolo'`: the scenarios are tuned on the lattice and must not change with the default.

### Mission graph: lighter default
- **Lattice at 0.5 m** (`MISSION_SPACING_M`, before 0.25) and **random nodes at 4 nodes/m²** (`RANDOM_DENSITY_PER_M2`, before 16), with a similar number of nodes: much lighter graph, suitable outdoors too. With the margin at 0.25 m the lattice guarantees the passages from 1.0 m up; narrower ones are found only if a row of nodes falls in them (user's choice: "if we are unlucky and it does not pass, for now that is fine").
- Replay of the 10 missions (`test_replay.py --tutte --veloce --passo 0.5`, and `--grafo casuale --distribuzione poisson --densita 4 --seme 7`), compared with the lattice at 0.25 m and the margin at 0.25 m:

  | graph | paths found | mean length | edges per path | graph time |
  |---|---|---|---|---|
  | lattice 0.25 m | 82/82 | 4.29 m | 4.5 | 425 s |
  | **lattice 0.5 m** | 79/82 | 4.31 m | 4.8 | **74 s** |
  | poisson 4 nodes/m² | 80/82 | 4.30 m | 4.8 | 98 s |

  The missing paths are real narrow passages. Missions 12-10 14:10 and 14:23, same point: the path of the 0.25 m lattice passes 0.28 m from the obstacles near (-6.0, 1.0), a gap of about 0.6–0.7 m where no row of nodes falls at 0.5 m. Mission 13-10 11:22: robot in a tight space, with a single valid edge, and fallback goal at 1 m. In a mission the robot would give up that side and try the cell from another.
- `test_replay.py --passo`: replays with another lattice spacing. Without it, every mission uses the spacing written in its log, and the reference stays that of the missions recorded at 0.25 m.

### Maps and terrain (`spotGrid.py`)
- **Margin around the body from 0.30 to 0.25 m** (`ROBOT_CLEARANCE_AIR_M` from 0.05 to 0): the path line must stay half of Spot's width from obstacles, without extra air, in the safe frontier and in the edge veto (margin on the edges from 0.15 to 0.10 m). The robot still does not plan where the body does not fit; the usable corridor goes down from 0.60 to 0.50 m, and the 0.25 m lattice guarantees passages from 0.75 m instead of 0.85. Replaying the 10 missions: no invariant violated, valid edges +0.75% on average (up to +2.4%), of 9 changed paths 8 shorter and one longer (-1.4 m in all), gaits and return to wp_0 unchanged. Replay reference updated. To watch in the field: without air, map errors (~3 cm cells) and position errors no longer have a margin, and Spot's body collision avoidance, which stays on, could stop it in passages at the limit. Going back is 0.05.

### Navigation (`easy_walk.py`)
- New arguments: `--grafo`, `--distribuzione`, `--densita`, `--seme`, `--min-edge`, `--max-edge` (before, the edge length could only be changed in the code). Line `[CONFIG] Grafo` at start-up; `MISSION_GRAPH_SAMPLING` among the switches.

### Tools
- **`test_replay.py --grafo …`** replays the real scans with another graph, and prints for every mission paths found, mean length, edges per path and graph time. With a graph different from the mission's the robot attaches to the nearest node, the goal is the nearest node with at least one valid edge, and the comparison with the reference is skipped. Mission 13-10 11:22 had the goal moved from the centre (occupied): the random node nearest to that point was against the obstacle, without edges.
- Replay of all 10 missions, 82 cell starts, seed 7. The times come from 10 processes in parallel and are only indicative; the difference follows mostly the number of pairs:

  | method | paths found | mean length | edges per path | graph time |
  |---|---|---|---|---|
  | lattice | 82/82 | 4.30 m | 4.51 | 582 s |
  | uniform | 82/82 | 4.32 m | 4.68 | 543 s |
  | poisson | 82/82 | 4.33 m | 4.76 | 505 s |
  | jittered | 82/82 | 4.30 m | 4.67 | 487 s |
  | halton | 82/82 | 4.30 m | 4.62 | 512 s |

- **`mission_viewer.py`, end-of-mission view: the real global map** (`global_occupancy_map.pkl`, the planner's one) instead of the mosaic of the saved scans. The start-of-attempt scans do not end up in `scans/`, and an attempt that fails before moving leaves none. In the indoor test of 22-10 15:24, the wall at the end of the corridor, seen at the attempts on (0,5) and (1,4), was in the map (1285 occupied cells) but not in the figure. The scan-by-scan views stay with the mosaic, and the title says which of the two is being looked at.
- `test_e2e_loop.py`: scenario S13, the corridor of S9 with the lattice and with the four distributions. The robot arrives in all cases without grazing the wall.
- `mission_report.py` shows the mission graph (method, distribution, seed).
- `mission_graph.py --preview` with the zoom no longer gives the matplotlib warning: the labels of the cells outside the region are clipped.
- **`prova_grafo.py`**: the graph parameters in a block at the top of the file (area, edges, lattice spacing, density and seed of the random nodes, methods to compare, zoom); `python3 prova_grafo.py` makes and opens the side-by-side image. By default the whole area with a sparser graph (lattice at 0.5 m and random at 4 nodes/m², edges 0.5–1.0 m, about 800 nodes and 3.5–4.7 thousand edges), readable without zoom.
- **Comparison with the same number of nodes**: in `prova_grafo.py` `DENSITA_NODI_M2 = None` (optional; the default stays 4 nodes/m²) gives the random nodes the same total number as the lattice (`mission_graph.lattice_matched_density`): the lattice also has a row of nodes on the outer border, and at equal density it had 6% more (3×3 cells of 4 m, spacing 0.5: 625 against 585). The table has the number of nodes per cell (`nodes_per_cell`), with the nodes on the boundaries between cells counted as half: the mission's count assigns them all to one cell, and the lattice seemed to have from 49 to 81 nodes per cell instead of 64–72. With the same number of nodes (625), per cell: lattice 64–72, halton 68–71, poisson 65–74, jittered 65–74, uniform 60–81. The lattice still has more edges (5710 against ~4900): no pair of its nodes is shorter than 0.5 m.
- The jittered distribution gives exactly the requested number of nodes: the whole squares can be slightly more or fewer, and the extra ones are removed at random (or uniform points are added). With the mission's default density the squares already add up, and the graph does not change.
- **`mission_graph.py --confronta-metodi PNG`**: the lattice and the four distributions side by side, with the same parameters and the same seed, the edges of the starting node highlighted and a table (nodes, edges, mean degree, coverage).
- **Defaults of `mission_graph.py` equal to those of the mission**: edges up to 1.25 m (before 2.0) and all neighbours between 0.5 and 1.25 m (before 2 rings, edges up to 0.71 m). From the command line one saw a graph different from the robot's. Now `easy_walk` reads `PRM_MIN_EDGE_M`, `PRM_MAX_EDGE_M` and `MISSION_LATTICE_RINGS` from `mission_graph` (`DEFAULT_MIN_EDGE_M`, `DEFAULT_MAX_EDGE_M`, `LATTICE_RINGS`), like the mission size already: the values are in a single place.

---

## 2026-10-21

### Planner (`prm_graph.py`)
- **Refresh slopes computed in batch** (`REFRESH_BATCH_SLOPES`, like `mission_graph.BATCH_SLOPES` at cell start). After 14-10 the refresh at every scan had become the slowest part of the graph: replaying mission 13-10 12:40 with today's code, 1.08 s per scan against 1.38 s per cell start. Graph bit-for-bit identical to the edge-by-edge computation (edges, weights, validity, slopes, global slope cache) at every scan of five missions, 340 out of 340. Times on the same replay (59 scans, 11 cell starts, one variant at a time):

  | variant | cell start | refresh | total |
  |---|---|---|---|
  | all edge by edge, full rebuild | 76.8 s | 62.7 s | 141 s |
  | local update | 73.3 s | 62.6 s | 138 s |
  | + slopes in batch at cell start (14-10) | 15.2 s | 63.5 s | 81 s |
  | **+ slopes in batch in the refresh** | **15.2 s** | **17.4 s** | **34 s** |

  In a mission the refresh is done with the robot still between one step and the next: from ~1.1 to ~0.3 s of pause per scan.

### Maps and terrain (`spotGrid.py`)
- **Edge length computed with `x*x` instead of `**2`** in the edge-by-edge slope profile (live and from the global map). On a single number `**2` sometimes differs in the last digit from the operation numpy does on arrays, and from there step and slope: on mission 12-10 14:47 seven edges had slopes differing by 1e-16 between the edge-by-edge and the batch computation. Now the two coincide bit for bit there too (on 14-10 the comparison had been done only on the missions of 13-10).

### Tools
- **Log messages in Italian** in all modules (`easy_walk.py`, `environmentMap.py`, `navGraphUtils.py`, `spotGrid.py`, `spotLogInUtils.py`). The tags in square brackets stay the same, so searches in the logs keep working. `mission_report.py` and `test_replay.py` read both the new messages and the English ones of the missions recorded before.
- **Test checks in `mission_report.py`**, with the list `PROVA_CAMPO.md`: for every feature never tried on the robot, whether it triggered and whether the numbers are those expected.
- **README rewritten** in Italian and in the present tense: start-up, how a mission works, modules, switches, saved data, tools. The previous one (20-05) described the project of that time.
- Default of the mission in `mission_graph.py` written out: 3 × 4 cells of 4 m (before `3 #None`, with the comment of the old default 2 × 4 of 5 m).
- **Switches in the log and in the summary**: line `[CONFIG] INTERRUTTORI` at start-up and entry `interruttori` in `mission_summary.json` (local update, slopes in batch at cell start and in the refresh, free-space map, GraphNav shortcuts, …); the report shows them. It serves the on/off comparisons in the field.
- **"Fluidita'" line in the report**: mean speed and time standing to rotate or start. Mission 13-10 12:40: 0.14 m/s over 356 s of movement, of which 93 s (26%) standing to rotate or start.

---

## 2026-10-20

### Motion (`movements.py`)
- Removed the `stairs` parameter of `relative_move`: it was never read and nobody passed it, so it suggested a stairs mode that does not exist.

---

## 2026-10-14

### Safe frontier (`arcVerification.py`)
- Removed `AsyncArcVerificationTracker`, never used (it called a service that does not exist).

### Planner (`prm_graph.py`)
- **The slope term of the cost stays fixed per edge, not per metre** (user's decision: the planner must prefer long edges, and the per-edge term does that). Replaying the missions of 13-10 (36 cell starts), a per-metre cost changed 32 paths out of 36: exposure to slope −9%, but edges +34% (from 144 to 193). If one day the terrain becomes really steep, the agreed form is slope per metre plus an explicit penalty per edge.
- **Slopes of many edges in one go** (`precompute_arc_slopes`, with `spotGrid.arc_slope_profiles_batch`). Bit-for-bit identical numbers on 421 thousand edges of the missions of 13-10, about 13 times faster. Before, the graph update spent 90% of its time in tens of thousands of calls on arrays of a few dozen numbers.

### Maps and terrain (`spotGrid.py`)
- **`SLOPE_THRESHOLD` also read as a limit on steps.** The slope is measured over a 0.48 m base, so a sharp step of height H counts as H / 0.48, whatever the edge length: 0.6 corresponds to about 29 cm, Spot's limit (~30 cm) without margin. In the field (13-10) the highest step climbed was ~25 cm (slope 0.54). The gait thresholds, expressed this way: PLAIN below ~7 cm (0.15), HARD from ~12 cm (0.25); the step of 13-10 12:47:20 (~17–19 cm) is HARD.
- **The half-body extension (`ARC_BODY_EXTENSION_M`) also applies to the PRM veto**: the robot must not be able to stop on a node with its legs on a slope beyond the threshold. On the maps of 13-10 12:40: edges discarded from 0.77% to 1.19%, isolated nodes from 1 to 5. The frontier stays on the edge between the nodes, so it is more permissive than the PRM and cannot block an edge the PRM accepted for slope.
- Corrected a comment that gave the slope base as ~25 cm: the difference is centred, 8 cells ahead and 8 behind, i.e. ~0.48 m (see also 06-10 evening).

### Navigation (`easy_walk.py`)
- **GraphNav shortcuts chosen on the way** (`plan_graphnav_shortcuts`). Before, up to 30 shortcuts were added between waypoints less than 2 m apart, the shortest first: with 4 m cells and the serpentine loop, between one row of cells and the next almost no pair was within 2 m, and the return to wp_0 of 13-10 12:40 retraced the whole mission: 46.7 m in the graph (straight line 13.1 m), 115 s. Now the shortest way to the destination is computed with all the possible shortcuts up to 3 m and only those the way uses are created. Replayed on the downloaded graphs of 13-10 (`test_replay.py`, check 5): return of 12:40 from 46.7 to 18.5 m, 12:14 from 18.8 to 17.3 m, 12:27 from 18.0 to 15.8 m. Margin from the occupied cells raised to 0.30 m (0.45 m from the obstacle), because the edges are longer.
- **A single place to put a scan into the global maps** (`fold_scan_into_global_maps`), used by the mission and by `test_replay.py`, so the replay checks the real code. The terrain map receives the same mask as the live computation: before it also saved the heights of obstacle cells, and logs and walls became slopes. Replaying the scans of 13-10 12:40, on the same 1885 pairs, edges beyond 0.6: 0.0% live against 10.9% from the global map.

---

## 2026-10-13

### Planner (`prm_graph.py`)
- **Preliminary discard on the live scan** (`LIVE_PREFILTER_OD_M` = 0.20 m). Before, an edge was discarded for occupancy only by the global map, which confirms an obstacle after 2 observations (4 where the robot walked) and does not write the ring around the body: at the first scan of a place the edges crossing it all stayed valid. Mission 13-10 11:22, start of the attempt in cell (0,1): 3160 valid edges out of 9038 in the window passed inside an obstacle (`obstacle_distance` < 0) and 4575 below 0.20 m; at the following turns 20–110. 0.20 m is half the body width minus 5 cm of noise. Of the 85 steps executed in the missions of 12-10 14:23 and 17:32 and of 13-10 11:22 none would have been discarded, not even at 0.25.
- The same discard applies to the valid edges of the window that lie outside the refresh radius (`live_prune_valid_edges`): before, an edge 1.3–1.9 m from the robot crossing an obstacle just seen stayed valid until the robot got closer.
- **Local graph update.** At the start of every cell the ~94 thousand edges are no longer all re-evaluated, only those near the areas where new data arrived (`dirty_rects`, the scan windows).
- **Slope log summarized.** The `[SLOPE-VETO]` and `[SLOPE-FLIP]` lines were 1338 out of 3155 in the mission of 12-10 17:32, 42% of the log. Now there is one `[PENDENZE]` line per build or refresh; the edge-by-edge values stay in the `_vis.npz` bundles.

### Maps and terrain (`spotGrid.py`)
- **Slope "with the body"** (`ARC_BODY_EXTENSION_M`): for cost and gait the edge is extended by half a body at each end, because the robot standing on a node has its legs 0.55 m ahead and behind. Mission 13-10 12:40, 12:47:20: a ~17 cm step was 20 cm beyond the end of the edge, the edge measured 0.17 (MODERATE) and the robot stayed still 9 s with its nose at 27 degrees, front legs on the step. `_body_slope` only reads the added pieces: resampling the extended edge from scratch doubled the graph construction.
- **90th percentile computed by hand**, bit-for-bit identical to `np.percentile`. `np.percentile` cost ~130 µs of overhead alone per call: 121 thousand calls, 15.7 s out of 35 of the graph construction.

### Motion (`movements.py`)
- Explicit message after a rotation in place. Before it said "Arrived at the goal (traveled 0.03m)", and the 3 cm (the body centre moving while turning) looked like a tiny advance.

### Navigation (`easy_walk.py`)
- **GraphNav localization.** Missions of 12-10 17:16 and 17:32, outdoors: at the first use of GraphNav the robot was already lost ("Cannot issue a navigation request when the robot is already lost"); in the first one this ended the exploration (three moves rejected). On the downloaded graphs the map was sound: leg odometry and vision differed by at most 13–14 cm over 34–49 m (as in the mission of 14:47, which came back), and our shortcuts coincided with vision within 6 cm. What gets lost is the live localizer. Added the `[LOCALIZZAZIONE]` line, the re-anchoring on the nearest waypoint with a guess from odometry (without declaring arrival), the permissive loss detector, and the check of the localizer before leaving, instead of having the command rejected three times.
- **Returns with memory for the attempt** (`_trap_backtrack`, `MAX_BACKTRACKS_PER_ATTEMPT` = 2). Mission 13-10 11:22, cell (1,2): after a trap the robot went back with GraphNav to the previous breadcrumb, found exactly the same path again (no edge discarded, counters reset) and walked back into the same trap: 6 loops before we stopped it by hand.
- **Recovery: no second attempt on the same goal** (`RECOVERY_SAME_GOAL_M`). Same mission: the recovery found again the goal that had just failed, reachable "with 1 edge" (precisely the one of the trap), and went straight back into the attempt.
- **Centre rule** (`find_best_point_in_cell`): the geometric centre is the goal, free or unknown; it changes only if seen occupied or already discarded live. Before, a point already seen free was enough to beat the unknown centre: it was the last strip scanned from the previous cell, on the border, ~2 m from the centre. Missions of 13-10 12:14, 12:27 and 12:40: 20 entries out of 38, with three arrivals outside the cell.
- **Goal chosen again at least 0.30 m from obstacles**, also looking at the live scan and the learned free space; at equal distance from the centre the point nearest to the robot wins. 13-10 12:40: the re-choices fell 0.35–0.50 m from the discarded point, still inside the hedge (`obstacle_distance` −0.04, 0.06, 0.27); in cell (2,0) there were two points 0.50 m from the centre, one beyond the hedge and the other on the robot's side, the one it then entered from.
- The scan goes into the preliminary edge discard (`set_live_obstacles`): raw at the start of the attempt, filtered in the loop, removed after a return with GraphNav (it had been taken from another point).

---

## 2026-10-12

### Safe frontier (`arcVerification.py`)
- **Cells under the body never an obstacle** (`_under_body`). Mission 12-10 10:36: the frontier stopped twice, halfway through a move and standing still, on a point 8–9 cm from the robot centre while passing close to a table (`obstacle_distance` under the robot 0.21 m against 0.25 of half width).
- **Two thresholds: 0.30 m to start, 0.25 m to stop halfway through a move** (`air_m`, `FRONTIER_ABORT_AIR_M`). User's request: a leg declared free is travelled; while moving it only stops for a real intrusion.
- **In the log the effective threshold and whether the hysteresis decided.** Mission 12-10 14:47: 23 stops out of 33 said "ostacolo a 0.32 m (minimo 0.30 m)", which looked like a contradiction.
- Exit rule: under the body the reference is the robot's distance now, not the readings under the belly.

### Planner (`prm_graph.py`)
- **Full-fidelity refresh only within 1.25 m of the robot** (`REFRESH_RADIUS_M`). The refresh re-evaluated ~8500 edges per turn, 3.65 s: it was the biggest time entry of the mission (37.5%), because on the lawn 53% of the slope readings find real data against 8.6% indoors. The robot travels one of them in the next step, and advances ~1.2 m at a time. Mission 12-10 14:47 (1 build + 27 refreshes), comparison with the full evaluation:

  | radius | time | different verdicts |
  |---|---|---|
  | 2.00 m | 0.80 s | 1 |
  | 1.50 m | 0.58 s | 79 |
  | **1.25 m** | **0.49 s** | **80** |
  | 1.00 m | 0.38 s | 654 |

  1.25 m is also `PRM_MAX_EDGE_M`: the edge the robot is about to travel always falls entirely in the re-evaluated area.
- **Faster graph construction, identical results.** Mission 12-10 10:36: of the 16 s of construction, 7 were the occupancy check (a million calls to `is_occupied`, almost all on terrain never seen) and the other half the slope from the global map. Now occupancy is read in batch (`_occupancy_blocked_many`), the edges certainly without slope data are cached at once (`_prefill_global_slope_nodata`), and the edges in an area without data get the fallback cost directly, with the same formula (3 edges out of 4 of the lattice).
- With `gamma` = 0 roughness is not sampled for the cost: it was 10–15% of the construction time.
- The centre of the refresh radius is the robot node (if unknown, the centre of the window).
- For occupancy both the binary map and the free-space map are computed; `spotGrid.CLEARANCE_MAP_DECIDES` decides, the other serves to measure the disagreement (`[SPAZIO]` line).

### Maps and terrain (`spotGrid.py`)
- **Cells the robot passed over: more confirmations are needed to declare them occupied** (`GLOBAL_OCC_CONFIRM_WALKED`). Two obstacle observations beat any number of free-space observations. Mission 12-10 14:47, cell (1,1): a cell where the robot had stood still for 16 scans, with 0.35–0.38 m of space under it, was declared occupied by two readings taken from 1.8 and 2.5 m (0.121 and 0.109 m). In 16 scans out of 72 the map declared occupied the cell the robot was in; 35 scans and 5 blocks on that cell. Replaying the 72 scans against the views from less than 0.8 m: false obstacles from 274 to 193 (−30%), obstacles not seen from 377 to 384, "the robot was here and it is occupied" from 16 to 0. The alternative (never resetting the free evidence) gave 173 false but 490 not seen: discarded.
- **Free-space map** (`GlobalGrid` learns the distance from obstacles per cell). The occupancy map only knows "obstacle yes/no": a 0.26 m gap is open lawn for it, while the frontier requires 0.30. Mission 12-10 14:47, cell (1,1): the planner proposed the same gap eight times, one edge next to the other; 336 node discards for 88 really bad nodes. Chosen statistic: mean of the close observations (`CLEARANCE_NEAR_M`), otherwise of all. On the two missions of 12-10 (frontier stops recognized / travelled points that would be forbidden):

  | criterion | outdoor | indoor |
  |---|---|---|
  | bit + margin (current) | 39% / 6.1% | 43% / 16.0% |
  | **close ones, otherwise all** | **52% / 1.9%** | **86% / 16.5%** |
  | mean of all | 52% / 1.3% | 21% / 16.0% |

  For now `CLEARANCE_MAP_DECIDES = False`: the map learns and the disagreement is measured, the binary map still decides. It is switched after a test mission.
- **HARD gait threshold from 0.35 to 0.25**, on the numbers of the first outdoor mission. With 0.35 HARD never triggered (48 steps out of 48 between PLAIN and MODERATE): the maximum gradient on the lawn was 0.330. On the 48 edges travelled: p25 0.132, median 0.171, p75 0.203, p90 0.248. With 0.25: PLAIN 29%, MODERATE 60%, HARD 10%, i.e. HARD on the steepest tenth, as the user wanted.
- `occupied_many` and `certainly_no_slope_data`: the batch versions of `is_occupied` and of the global map's "no data" check, with results identical point by point (98% of the calls during construction ended in "no data").

### GraphNav (`navGraphUtils.py`)
- **No graph downloads just for a name.** The number of the next waypoint is derived from the waypoints we created, and name and id of the last one come from the robot's answer (`last_created_waypoint`). Mission 12-10 14:47: 41 full downloads out of 60 served only this, with a cost growing with the map (70 waypoints at the end of the mission).

### Navigation (`easy_walk.py`)
- **GraphNav and closure times in the `[TEMPO]` summary.** Mission 12-10 14:47: 212 s out of 806 of wall clock (26%) did not end up in any measured phase; the biggest slice was GraphNav (83 s for the return to wp_0 alone), the rest the closure after the summary.
- **MODERATE in CRAWL too** (it was `HINT_AUTO`), user's request after the first outdoor mission. On the lawn MODERATE takes ~60% of the steps, so crawl becomes the prevailing gait and the mission slows down.
- **Nodes in obstacles also judged on the free-space map** (`prune_prm_nodes_in_obstacles`, when `CLEARANCE_MAP_DECIDES`). The scan window contains ~235 nodes out of 3321 (7%), and the discard is soft: at the next refresh the edges came back, 336 discards for 88 really bad nodes.
- **The filtered `obstacle_distance` and the latch in the scan bundles**: without them, an offline replay does not reproduce the blocks that come from the thread's readings during the move (scan 13 of 12-10 10:36).

---

## 2026-10-08

### Planner (`prm_graph.py`)
- **Cost layer instead of removing nodes and edges.** User's decision, with the professors: "do not modify the graph at the level of nodes; what changes instead are the costs". What is learned in the mission becomes a penalty that Dijkstra adds to the geometric cost; impassable means infinite penalty, but the edge stays in the graph. In the missions of 07-10 the removal had condemned forever 8 edges around a node for a marginal obstacle at 9 cm, and the node stayed isolated for the rest of the mission. Persistent penalties (confirmed block, motion failure, trap) and transient ones (recomputed at every scan).
- **Cap on the number of edges of a path.** User's request: "it cannot make paths that are too long over the cells already visited". It is checked on the path found: if the minimum is beyond the cap, the cell is given up.
- **Detour cap.** Mission 08-10 15:09: with the robot 2.4 m from the goal the planner had proposed a loop of ~15 m.
- The cost of the current plan is computed with the same costs as Dijkstra, penalties included.

### Mission graph (`mission_graph.py`)
- **Regular lattice instead of random sampling** (`global_sampler.GlobalSampler`, now removed). Before: 5 points/m², ~3125 nodes on 25×25 m, no seed, a different graph at every mission, ~85 600 edges to evaluate (~10 s per build, 3–4 builds per mission). The lattice at 0.714 m spacing with two rings has ~4700, 18 times fewer, and is reproducible bit for bit.
- **Rectangular, not triangular.** The triangular one cannot have the cell centres as nodes (it would need 2n/√3 to be an even integer). The alternatives were moving the goal to the nearest node (up to 50 cm) or inserting the centres and removing the nearby nodes, which left the lattice irregular around 20 cells out of 25, with edges from 0.57 to 1.44 m.
- **Spacing from 0.5 to 0.25 m, edges from the second ring (≥ 0.5 m).** Mission 08-10 15:40: the starting corridor had ~45 cm of allowed centre lines and the two rows of the lattice at 0.5 spacing both fell outside, so in the graph the corridor did not exist. At 0.25 spacing there are two rows inside. The rings were redefined as a circle of radius k·s·√2, for consistency with the PRM refresh.

### GraphNav (`navGraphUtils.py`)
- **Loop closures during the mission** (`close_loops_checked`), with fiducials and odometry and Spot's collision check on the new edges. Before, they were closed only at the end of the mission and only with fiducials: the graph stayed a chain and every return retraced the whole way.

### Pre-mission check (`preflight.py`)
- Table updated: `MAX_MANEUVERS_SAME_EDGE` no longer exists since sideways walking was removed (07-10), and `preflight` rejected a correct copy of `easy_walk.py`.

### Maps and terrain (`spotGrid.py`)
- **Rotation tolerance from 0.05 to 0.15 m.** User's request: "the padding from the obstacle must not block it in the rotation, sometimes there would be room, with GraphNav it manages". Mission 08-10 10:47, 46 scans, directions of the circle both free and reachable by turning:

  | tolerance | usable directions (median) | scans with zero |
  |---|---|---|
  | 0.00 | 3 | 7/46 |
  | 0.05 | 6 | 6/46 |
  | 0.10 | 6 | 6/46 |
  | 0.15 | 6 | 0/46 |

  The six scans in a trap had the free directions refused at −0.12/−0.15 m. At 0.15 the body is allowed to enter our map by 15 cm: the grid registration error is 5–10 cm, and GraphNav rotates and passes at those points. If tests show grazing during rotations, it is the first number to bring back to 0.10.
- (evening, after the review) **Rotation: the penetration into the body is measured, not the minimum of `obstacle_distance`.** Inside an obstacle `obstacle_distance` does not go below half its thickness, so with the tolerance at 0.15 any obstacle thinner than ~30 cm entering the body was approved (poles of 6, 10, 20 and 30 cm in a synthetic test). The cells under the body now are excluded. On the 47 scans of mission 08-10 16:35, 24 rotations per scan: same verdict in 97.8%, the 24 changed all from approved to refused, almost all at the limit (−0.150 → −0.151).

### Navigation (`easy_walk.py`)
- **Edges between 0.5 and 1.25 m** (`rings=None`, `PRM_MAX_EDGE_M` = 1.25), rule chosen by the user after mission 08-10 16:35. With the cap at 0.707 m a cell entry became a chain of 16–19 waypoints, each one a rotation, in the 1.5 m corridor where rotating was exactly what could not be done (8 loop turns in 3.2 s without moving). Cost measured on the real global map of that mission, 3321 nodes:

  | edges | pairs | allowed | build | refresh |
  |---|---|---|---|---|
  | 0.5–0.707 m | 25 120 | 18 853 | 0.92 s | 0.12 s |
  | **0.5–1.25 m** | **109 474** | **79 229** | **4.72 s** | **0.43 s** |

  On the mission of 16:35 (4 builds, 47 scans) it is ~39 s instead of ~9 s: accepted price, partly given back by the event refresh. `PRM_MAX_EDGE_M` follows the lattice and does not stay at 2.0, because the refresh rebuilds the pairs with the PRM's `max_edge_length` and with 2.0 it would fill up with long edges only around the robot.
- **Paths only in the visited cells** (`PLAN_ONLY_EXPLORED_CELLS`). Mission 08-10 15:09: to enter (0,2) the planner went through row 1, never explored.
- **Anti-stall guards.** Mission 08-10 16:35, iteration 2: eight turns in 3.2 s with the robot motionless. Replaying: the rotation asked for almost 180 degrees (the goal was behind a wall), the sector refused it (−0.17/−0.19 m), the cone found no directions, and the usable directions were 3 out of 24, so no "trap". `MAX_ROTATION_DISCARDS` counts per attempt: `MAX_ROTATION_FAILS_SAME_EDGE`, indexed on the pair of nodes, restarted from 1 at every replan and never triggered. `MAX_IDLE_LOOPS` is the generic net. (evening, after the review) The first version said 12 "above ~8" and was below the longest legitimate sequence with the robot still: recomputed on the limits of the loop, 26 turns, hence 30.
- (evening) **Rotation refusals reset only on progress** (20 cm closer to the goal). With the reset at every move, in test bench S11 (0.80 m corridor with the goal behind) the robot went one node east after another: 35 scans and 4.5 m moving away, with the counter always at zero.
- **Multi-node advance.** The frontier covers the whole path, but only the first node was looked at and the robot stopped to rescan at every node. Mission 08-10 16:35: the frontier confirmed 1.9–3.2 m with edges of 0.5–0.7 m, three or four nodes ahead ignored. (evening) After every leg the true pose is read again: with the heading of the initial scan the test bench measured an 18 cm gap on a single 2-node move.
- **Event refresh.** Before, the edge refresh ran at every loop turn. Mission 08-10 16:35: 22 scans out of 46 (48%) with a move below 25 cm, ~9.5 s recomputed to get the same numbers again.
- **The cone aims at the path, not at the goal** (`CONE_PATH_LOOKAHEAD_M` = 1.5). Mission 08-10 16:11, scan `it001_cell0_2_007`: the path turned west around a block, the goal was east beyond a dead end, and "getting closer" to the goal took the robot 1.36 m into the dead end. In scan 008 the data edge got shorter during a rotation (1.29 m out of 1.38) without any obstacle: hence `ABORT_IGNORE_DATA_EDGE_UNTIL_M`.
- **Arrival point without exits and no alternative: the side is given up.** Before, it went anyway. Mission 08-10 16:11, at (60.05, −31.06): 0/24 exits seen in advance, the robot went in, found itself trapped and went back with GraphNav for 3 waypoints.
- **Area around the body in the global map** (free core, ring without obstacles). Mission 08-10 15:09, around (58.6, −32.8): the same cells read from 1–2 m had `obstacle_distance` 0.18–0.24 (free), read with the robot standing next to them 0.09–0.15 (obstacle), 0.23–0.45 m from the axis, at the height of the rear legs. Two still scans confirmed them, and the PRM discarded the edge the robot had just travelled.
- **Shortcuts in the GraphNav graph** (`graphnav_add_shortcuts`). The graph was a chain and the loops were closed only at the end of the mission and only with fiducials ("Created 0 new edge(s)" in the log of 08-10 15:40). Mission 08-10 16:11: Spot accepts new edges only while recording ("NotRecordingError"), the loop closure must be done with recording stopped and can answer "MapModifiedError" right after.
- **Cell blocked at once if no point to stand on is left** (`cell_has_reachable_target`). Mission 08-10 16:35: the 2×4 grid of 5 m did not fit the space (corridor explored 12×5 m); classifying the centres against the final map, (0,1) had 144 occupied cells out of 169 and (0,2) 123 out of 130, two goals inside the wall, and the whole of row 1 had never been seen: three blocks out of four attempts came from there. With ~400 candidate points per cell the immediate block would not have triggered for any of the eight cells.
- Start and arrival nodes attached to existing nodes: the discard of the refused rotations holds while the robot is where the rotation was refused, and on the first leg, if the robot is not on the node, a confirmed block discards the edge only for now.

---

## 2026-10-07

### Safe frontier (`arcVerification.py`)
- **Temporal filter on `obstacle_distance`: median over 3 scans and 4 cm hysteresis.** Mission 07-10 14:0x, robot still and identical path: a cell read 0.296 and then 0.336 against a threshold of 0.300, and the frontier went from "obstacle" to "free" without anything moving. Noise measured between two consecutive scans: 1.5 cm on average, 7.3 cm peak.
- The shift between scans is measured between consecutive grids and not from an absolute origin: `cell_size` arrives as float32 (0.029999999329447746) and the history was reset at every scan.
- **"Cell seen" compared with `TERRAIN_VALID_MIN` and not with 0.** After scale and offset the 0 of `terrain_valid` becomes ±5.5e-17 with variable sign: the old `> 0.0` declared every cell seen in 10 scans out of 16.

### Maps and terrain (`spotGrid.py`)
- **`TERRAIN_VALID_MIN` = 0.5 in `correct_terrain`** (same error as the `> 0.0`). In the 16 scans of 07-10, in 10 every cell came out valid. The unreliable cells, including those never written, entered the terrain as good data: on average 276 roughness-only vetoes per scan, up to 1138, and in one scan the robot was standing on an obstacle cell.
- **`_cells_never_written` rewritten.** The previous version had never triggered: zero cells in 16 scans, while the never-written cells were 1845–3710 (11–23% of the grid). Two independent errors: the expected height was in the local grid frame and the heights in VISION (0.26–0.35 m of difference against 0.25 mm of tolerance), and those cells do not lie at raw 0 but at an integer that changes at every scan. Now the height is derived from the data: the most frequent one among the cells Spot declares unreliable (98.2–100% of the never-written ones).
- **The cells that `obstacle_distance` already calls obstacle do not enter the slope.** Mission 07-10 13:58, room with furniture: 200 edges vetoed for slope, longitudinal median 1.487 against a 0.600 threshold, maximum 2.092, and the replan failed from every node (225, 227, 228) despite 22–28 links each. The face of a piece of furniture is an obstacle, already handled by the occupancy veto and by the frontier. Those edges become "without slope data", enter the graph with the fallback cost and the frontier checks them.
- **Rotation: the sector the body really sweeps is checked, not the whole 0.60 m circle.** Mission 07-10, iteration 2, cell (1,0): 9 rotations refused out of 10, of which 8 safe; in three cases (scans 8–10) the refusal produced a block on a path free for 1.39 m.
- **Air on the rotation at 0** (explicit choice). A false "cannot rotate" had cost 3.5 m of chained back-off, and 0.05 m was less than the `obstacle_distance` noise. Of the 11 refusals of mission 07-10 14:0x it unblocks a single case: the others are real (body inside the obstacle by 4–19 cm in ~60 cm corridors). It is not the fix of the problem.
- **Noise tolerance in the rotation** (`ROTATION_NOISE_TOLERANCE_M`). Mission 07-10 16:50: from the same pose, in five consecutive scans, `obstacle_distance` at a point 50 cm away oscillated between −0.03 and +0.15 m (+0.45 seen from 2 m); 5 refusals out of 17 were inside that noise. In the passages between the tables the real margin is −0.16/−0.19.

### Navigation (`easy_walk.py`)
- **No sideways walking** (user's decision): the robot always points its nose at the point to reach. Sideways Spot is 1.10 m wide instead of 0.50 and the frontier requires 0.59 m instead of 0.30: six blocks on paths that were free for 1.3–1.4 m with the nose forward came only from this. When it cannot rotate the robot backs up straight along the body axis.
- **Back-offs counted per sequence** (`MAX_STRAIGHT_RETREATS_IN_A_ROW`). The per-edge limit restarted from 1 because every back-off created a new stop node: in the log "tentativo 1/2" seven times in a row. That is how 3.5 m backwards came about.
- **Possible rotation cone** (`plan_cone_move`). Mission 07-10 14:0x, file `iteration_0_cell_0_1_replan_12_vis.npz`, robot at (−2.28, 1.84), goal 1.26 m away: going straight, without rotating, led 0.50 m from the goal (−0.76); the wanted rotation (+22 degrees) was impossible; the code backed up seven times in a row, 3.5 m, and from every arrival point the rotation stayed impossible (margin −0.09 m). Corridor with ~60 cm of free lane: turning around needs 1.21 m of diameter.
- **Back up only if it helps** (`retreat_would_help`): in the two moments examined the rotation after the back-off stayed impossible (margin −0.09 m at (−2.28, 1.84), −0.00 m at (−3.91, 1.74)). The straight back-off was instead available in all 12 scans of iteration 1, from 1.08 to 1.37 m.
- **360 degree fan to recognize the trap** (`scan_escape_directions`, `is_trapped`). Mission 07-10 14:5x, file `iteration_2_cell_1_0_BLOCKED_vis.npz`, robot at (−1.69, 1.87), `obstacle_distance` under it 0.402: the two open lanes (1.30 m at −180 degrees, 1.09 m at 0) required rotations of −73 and +107 degrees, impossible; the other 21 directions had 0–0.24 m. The code had got there by discarding eight edges one by one over some twenty scans; the same pattern at node 281 and, with rotations of ±179 degrees, in cell (0,2), where four edges were condemned forever.
- **Check of the arrival pose** (`arrival_has_exit`). Same mission: the robot entered a corner without exits with a last step of 23 cm, rebuilt from the retreat ((0.40, 1.76) → (−0.71, 1.70) → (−1.47, 1.79) → (−1.69, 1.87)).
- **Returns with GraphNav instead of walking backwards** (user's decision): breadcrumbs (`graphnav_drop_breadcrumb`) every 0.75 m and at the entry of every cell, return one waypoint at a time trying to plan again, walked retreat only as the last net. A breadcrumb for every stop node would have been too many: in the mission of 14:5x there were 37, many 20–60 cm apart.
- **No useless returns** (`backtracking_cannot_help`). Mission 07-10 16:50: 6 returns out of 9 (cells (0,2) and (0,3)) to find out three times in a row that from there the graph offered no paths; the goal had been in another component for several turns.
- **PRM nodes inside obstacles discarded** (`prune_prm_nodes_in_obstacles`). Mission 07-10 14:5x: the goal node of cell (0,1) had `obstacle_distance` −0.21 m and the two fallbacks 0.12 m, the three "goals chosen again" of the mission.
- **Soft discard for [SUBITO] and for refused rotations** (`mark_edge_blocked_soft`). Mission 07-10 14:5x: eight edges around node 3164 ended up out of the graph for the whole rest of the mission because of a marginal obstacle 9 cm from the robot, and the node stayed isolated.
- **Goal chosen again if unreachable on the fresh data** (`goal_still_reachable`, parameter `exclude`). Mission 07-10, iteration 1, cell (0,1): the goal had `obstacle_distance` 0.27 against 0.30 required, it had been chosen from 3 m with the area still unknown, and the robot blocked 63 cm from it after three still scans, with 808 valid cells within 80 cm.
- **Arrival tolerance tied to the frontier** (`GOAL_REACHED_TOLERANCE_M` = 0.65 instead of 0.60): with 0.60 a 5 cm dead band was left. Same iteration: robot still with `path_len` 0.63–0.64 m, three scans and block.
- **Replan at the first blocking cell** ([SUBITO]). Before, it looked again standing still until the confirmation, but between scan 003 and 004, robot still and identical path, the same cell read 0.296 and then 0.336 against a 0.300 threshold: waiting added noise. Now the noise is filtered in `arcVerification`.
- **Shortcuts respecting the discarded edges** (`shortcut_index`): ~40 identical turns on node 225; in the mission of 16:39 the shortcut pointed again at node 218 right after the discard of 3169→218 for rotation.
- **Edges refused for rotation out of the plan until the robot moves.** Mission 07-10 16:39, cell (1,1): robot still on 3169 alternating 3169→142 and 3169→218 for 112 turns ("tentativo 56/2"), without ever getting to the retreat.
- **Scan bundle saved before the rotation block.** Mission 07-10 16:50: none of the 17 rotation refusals had left a scan on disk.
- **Obstacle cells excluded from the slope** (`slope_usable_mask`, see also Maps and terrain). Measured again on the 12 scans of the mission of 14:0x, 3607 edges in the window:

  | | before | after |
  |---|---|---|
  | edges vetoed for slope | 40.3% | 0.1% |
  | edges without slope data | 4.9% | 42.5% |
  | edges allowed in the graph | 59.7% | 99.9% |
  | maximum longitudinal slope | 2.2–3.3 | 0.03–0.10 |

- **GraphNav: an RPC error no longer abandons the move.** The `return False` was outside the `if`: a single transient error abandoned the move while the message said "(1/3)".
- **Footprint diagnostics (question A).** On the data of 07-10 190–191 obstacle cells were measured in the core of the body, all for roughness: never-written cells at the height of the plateau, with a jump of ~0.93 m from the floor. With `_cells_never_written` fixed they go down to zero, so the footprint mask stays a one-off.
- Real trajectory sampled during every move (`trajectory.csv`). Removed the line "[INIT] Initializing Velodyne client...": the Velodyne is not in use and the line suggested an extra sensor.

---

## 2026-10-06

### Safe frontier (`arcVerification.py`)
- **The safe frontier replaces the edge-by-edge check with permanent verdicts.** First outdoor mission, 05-10 16:05: five retreats from free terrain. The edges were judged while the robot walked, from afar, truncated at the edge of the window, and the verdict was never re-examined.
- Cells never written by the sensors treated as not seen: the frontier stops there with `fine_dati`.
- (evening, after the review) **Exit rule from a tight space corrected.** The lowered threshold in the first metre applied to any obstacle: a pole 15 cm from the flank, 70 cm further ahead, was accepted because the robot started near another object. Now "do not get closer than you already are" applies.
- (evening) Tolerance of the exit rule raised to 1.5 cells: with 2 cm, along a wall at 5 degrees from the path, 257 blocks out of 360 positions (before 13).

### Planner (`prm_graph.py`)
- **Occupancy sampled every 10 cm, with a single constant** (`OCCUPANCY_SAMPLES_PER_M`). Before, the construction used 3 samples per metre and the refresh 5: the same edge could be allowed and then refused just because of the density. At 5 per metre, halfway between two samples the margin disk (0.15 m) covered only 0.11 m, and a corner could come 0.26 m from the centre line, less than the frontier's 0.30: the robot blocked on "valid" plans. (evening) The number of samples is rounded up: `int()` gave up to 0.147 m between two samples.
- **The refresh also re-evaluates the edges with only one end in the window.** An edge from a nearby node through a wall just seen to a distant node was not re-examined, and with the safe frontier it produced repeated blocks on the same wall (`test_e2e_loop.py`, scenario S2).
- **`block_edge` is called only by the main loop**, after a block confirmed up close. Before, the old checker called it on verdicts taken from afar, the cause of the retreats of 05-10.
- `_evaluate_and_add_edge` extracted from `build_graph`, to link the stop nodes too with the same logic.
- (evening) **Per-edge cache of the global slope.** It was the dominant cost of `build_graph`: ~80 thousand edges times ~500 dictionary reads.
- (evening) **Direct first leg in the plan cost** (`allow_direct_first_hop`). After a shortcut the first leg is not a graph edge and the plan came out "broken": it replanned at every scan, the confirmation count of a block restarted from zero and a real block was never declared.

### Maps and terrain (`spotGrid.py`)
- **Ground height filter** (`GROUND_HEIGHT_TOLERANCE_M` = 1.2 m, relative to the ground under the robot). Mission 05-10 16:05: the robot had been powered on one floor up, the never-written cells were 3.226 m above the ground, and their roughness (1.585 against a 0.15 threshold) produced 100% of the obstacles of the blocks. Asymmetric on purpose: below the ground there can be a real void, and it is left alone.
- **The slope is measured only on reliable cells** (`valid_2d`): an edge seen for less than half goes back to "without data" and the global map is used.
- **Global obstacle map with confirmation** (`GLOBAL_OCC_CONFIRM_OBS`). Before, a single obstacle observation was enough, even on a cell seen free four times. (evening) The cells not reliable in a scan no longer count as "seen free"; the disk around a fall no longer becomes free again (before it disappeared at the fourth scan).
- Occupied disk around the spot of a fall, without waiting for sensor confirmations.
- **Rotation in place checked**: the corners of the body trace a 0.60 m circle and before nobody checked that it was free.
- PRM margin from obstacles from 0.05 to 0.15 m, to give the same distance as the frontier. Coarse prefilter in `is_occupied`: with 81 cells per check, without the prefilter the construction would go from ~2 s to ~12 s.
- Maximum speeds per gait level: before no limit (Spot up to ~1.6 m/s).
- (evening) **Quick exit in the slope profile** for the edges that do not touch the grid: before each one paid the whole sampling, ~30 s per build.
- (evening) **More robust layer download**: no result if a layer is missing (before `KeyError` and mission ended by a network error) or if the layers are not aligned with each other.
- (evening, review) The slope difference is centred: the real base is ~0.48 m, not 0.25. A 25 cm step gives ~0.52 of slope and ~0.125 of roughness, both below the threshold. Not changed.

### Motion (`movements.py`)
- **Speed limit on every command.** Before nobody set it and Spot walked up to ~1.6 m/s: in the mission of 05-10 the mean over the legs was ~0.3 m/s, but with peaks halfway through the leg too high for outdoors. The limit is always symmetric (`min_vel` = −`max_vel`): with only the maximum the robot can no longer go backwards or to the left, and the retreat would stop working without errors.
- **Stop halfway through a move** (`should_abort`) and feedback every 0.2 s instead of 1.0 (at 0.5 m/s, half a metre between two checks). Before, the command was blocking until the end of the trajectory: in the mission of 05-10 13:12 the checker had seen an obstacle during the move, but nobody could stop the robot.
- Fall detection and automatic self-righting (`check_fall`, `recover_from_fall`).
- (evening, after the review) **Time limit per move.** The command lasted 6000 s: if Spot could not get there, it kept trying for up to 100 minutes. (evening) **Network errors**: before they came out as an exception and ended the mission without stopping the robot, which finished the trajectory by itself; now it retries 3 times, then stops the robot.

### Navigation (`easy_walk.py`)
- **GraphNav moves with a time limit** (`_graphnav_navigate_to`, evening, after the review). `navigate_to_waypoint` and `navigate_to_first_waypoint` of `navGraphUtils.py` had no time limit (they repeated forever even with the robot fallen, and the mission never got to sit down), used empty `TravelParams` (Spot's default speed) and on `STATUS_LOST` forced the localization on the destination and declared "arrived".
- (evening, review) **Retreat only in continuity** (`RETREAT_CONTINUITY_M`): after a move with GraphNav the last recorded leg could be many metres away, and the robot went there walking backwards, blind.
- **Path shortcuts** (`shortcut_index`), checked by the frontier and by the global map (where, for example, the spot of a fall is).
- **Gaits tuned for outdoors**: PLAIN had `SWING_HEIGHT_LOW` (a floor step) and no level had a speed limit.
- **Mission reference height** (`mission_z0`): Spot measures heights from where it was powered on, and taking it to the mission area via the stairs shifted all heights (−3.2 m on 05-10).
- Margin of the goal point equal to that of the PRM and of the frontier (it was 0.05 m).

---

## Before 2026-10-06

### Planner (`prm_graph.py`)
- **Cost with separate longitudinal and lateral slope.** Before the slope was a single value, and an edge across a slope cost as much as a flat one with the same gradient.
- The slope profile of the edge is computed only once and passes from the veto to the cost: before twice per edge, with the same arguments.
- The edges travelled by the robot no longer come back into the graph by force if afterwards they were blocked or pass over the spot of a fall (`_traversed_edge_ok`): before, an edge travelled on the way out and then blocked came back at the next rebuild.
- `find_path_dijkstra` returns the current plan when stability keeps it; the caller must tell it from `None`. In the missions of 05-10 `[REPLAN WARNING]` mistook one case for the other.

### Maps and terrain (`spotGrid.py`)
- **Edge slope over a physical base** (`SLOPE_BASELINE_M`) and not over the cell step (~3 cm): the face of an 80 cm wardrobe gave 13 m/m and a 1.45 m door 24, exactly the maxima in the logs of 05-10.
- **Lateral slice as wide as the robot** (±0.30 m), not as the edge: before, a 2 m edge collected terrain up to 1 m on each side, and the same point was travelable with a 1.5 m edge and forbidden with a 2 m one.
- **Honest `has_data`.** Before it was always true: an edge outside the window came out as a live slope measurement of 0, and the global map was never used (`global_map: 0.0%` in every summary). The global map now uses the same geometry and the same base as the local grid.
- **Early exit on the global map.** Mission 05-10 13:12: 98% of the readings found nothing after ~500 accesses per edge, about 50 s per graph build.
- **Global map keys with `np.floor`, not `np.round`** (occupancy and heights): with the cell centres the round-to-even made different cells collapse on the same key and left keys never written, read as "free".
- **Roughness computed only on valid cells.** Mission 05-10, iteration 6: 29 never-written cells produced 83 false obstacles around them; with the statistics only on the valid ones those vetoes go to 0 (maximum roughness from 1.585 to 0.029) and the 54 real vetoes stay.
- `LATERAL_SLOPE_COST_MULTIPLIER` from 1.5 to 1.0: 1.5 was a guess, and the slope estimate of the time overestimated the small components up to ~1.9 times.

### Pre-mission check (`preflight.py`)
- **Born from the mission of 05-10 15:55**, interrupted right after the graph was built by `AttributeError: 'GlobalGrid' object has no attribute 'is_known'`: `easy_walk.py` was up to date, `spotGrid.py` on the robot was an old copy. No risk (before any movement), but the robot already powered and standing.

### Navigation (`easy_walk.py`)
- **Goal point chosen on the data.** Before, `find_best_point_in_cell` always returned the geometric centre, ignoring the scan: in a furnished room the centre fell inside or right next to a piece of furniture. Mission 05-10 13:12: path 176 → 211 → 3127, more than a metre beyond the target and then back, down to a narrow pocket. (Since 13-10 the centre is the goal again, but it changes when it is seen occupied.)
- **Retreat backwards over its own steps** (`retreat_along_traveled_path`). Mission 05-10 13:12: first `[REPLAN FALLITO]`, then the next edge blocked, in a space where turning around was impossible.
- **`navigate_to` returns the boolean**, not the tuple of `relative_move`: a non-empty tuple is always true, and a failed move came out as successful. It was impossible to notice that the robot was stuck.
- **The graph is no longer built at start-up**: without terrain or occupancy data it carried no information, and the first cell attempt rebuilt it from scratch before any use.
- **A cell that failed from one side is neither retried at once nor blocked** (`finalize_or_defer_blocked_cell`): it stays a candidate and is retried when its turn comes, from the side not yet tried.
- The side tried is that of the cell entry is attempted from, not the one the robot is in after the attempt. Without a waypoint to approach from the cell is skipped (before an index on `None` ended the mission).
