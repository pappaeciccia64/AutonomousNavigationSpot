# Autonomous exploration with Spot

Spot explores on its own a rectangular area divided into cells: it enters one cell after another, builds a map of obstacles and terrain while it walks, chooses its paths on a graph of nodes and travels them only as far as the live scan confirms they are free. At the end of the mission it goes back to the starting point with GraphNav and saves everything needed to understand, afterwards, what it did and why.

The history of the changes, with the measurements of the missions that motivated them, is in [`CHANGELOG.md`](CHANGELOG.md). What to check in the next field test is in [`FIELD_TEST.md`](FIELD_TEST.md); ideas, postponed decisions and work to do, with the data needed to decide them, in [`TODO.md`](TODO.md).

---

## Running a mission

**Requirements** (the ones it runs with today): Python 3.8, the Spot SDK (`bosdyn-client`, `bosdyn-core`, `bosdyn-api` 5.1.x), `numpy`, `scipy`; `matplotlib` only for the viewers. For the SDK see also `docker-requirements.txt`.

1. Copy **the whole folder** to the computer connected to the robot (not a single file: the modules must be the same version).
2. Check that the modules are consistent:
   ```bash
   python3 tools/preflight.py      # must say "All files are consistent with each other."
   ```
3. Put the robot at the starting point, facing the area to explore (optional: any fiducial 1–2 m away, in front or behind; Spot recognizes the Boston Dynamics AprilTag 36h11, not ArUco. See `START_FIDUCIAL_ID` in `spot_explorer/config.py`).
4. Start:
   ```bash
   python3 run_mission.py
   ```
   The robot address is in `main()` of `spot_explorer/mission/run.py` (`192.168.80.3`). The credentials are handled by the SDK (`bosdyn.client.util.authenticate`): with the original SDK they are given with the variables `BOSDYN_CLIENT_USERNAME` and `BOSDYN_CLIENT_PASSWORD`, otherwise it asks for them in the terminal. Careful: on the development computer that SDK file was modified to use fixed credentials; on a new computer, with a freshly installed SDK, the variables are needed.

**Parameters.** All the numbers that are changed in the field are in `spot_explorer/config.py`; those specific to a module (terrain thresholds in `perception/`, graph in `planning/roadmap.py` and `planning/edge_evaluation.py`, frontier in `planning/safe_frontier.py`, speeds in `robot/motion.py`) stay there.

**Mission size.** The default is in `config.py` (`MISSION_GRID_ROWS`, `MISSION_GRID_COLS`, `MISSION_CELL_SIZE_M`): today 8 rows × 7 columns of 2 m, i.e. 16 m to the left and 14 m ahead of the robot. For a single mission, without touching the code (in order of precedence):
```bash
python3 run_mission.py --rows 2 --cols 3 --cell 4      # arguments
SPOT_MISSION_GRID="2,3,4" python3 run_mission.py       # environment variable
```
The `[CONFIG] Mission grid` line of the log says which value was used and where it came from.

**Where the robot starts in the grid.** `config.MISSION_START_POSITION`, or for a single mission `--start`. Looking at the grid from above with the robot's forward direction up:

| | left | centre | right |
|---|---|---|---|
| **top** (grid behind) | `top-left` | `top-centre` | `top-right` |
| **centre** | `centre-left` | `centre` | `centre-right` |
| **bottom** (grid ahead) | `bottom-left` | `bottom-centre` | `bottom-right` (default, cell (0,0)) |

```bash
python3 run_mission.py --start centre              # the robot in the middle of the area
python3 run_mission.py --start bottom-left         # grid ahead and to the RIGHT of the robot
python3 run_mission.py --start 3,2                 # any cell (row,column)
```
The visiting order (`--order`, `config.MISSION_VISIT_ORDER`) is `auto`: a serpentine from the robot's corner, or rings around the start if the robot is not in a corner; `serpentine` or `rings` can be forced. The `[CONFIG] Start` and `[CONFIG] Visiting order` lines of the log give the cell, how far the grid goes ahead, behind, left and right, and the first cells in order. An invalid start (cell outside the grid, unknown name): the mission does not start.

In a simulator (Isaac Sim, or a kinematic one without physics): `run_mission.py --sim`, see `sim/README_SIM.md` and `SIMULATOR.md`.

Where the goals come from (`--goals`, `config.MISSION_GOALS`): `cells` (default) takes the next cell of the visit and a goal near its centre; `frontier` takes the frontier node (seen, free, next to the unseen) that is cheapest to reach from the robot, with the distance from the route of the visit as a cost so that the sweep stays orderly, and ends at 97% of the area seen (`spot_explorer/mission/frontier.py`); `--frontier-beyond M` puts the goal on the node up to M m beyond the frontier node, into the unseen (default 0 = on the frontier node). The navigation is the same in both; the option is there to compare them, in the simulator first.

**Mission graph.** The nodes are generated in two ways (`--graph`, default in `config.MISSION_GRAPH_SAMPLING`):
```bash
python3 run_mission.py --graph random --distribution poisson --density 9 --seed 7     # random nodes (default: poisson, 9 nodes/m², random seed)
python3 run_mission.py --graph lattice --spacing 0.5                                  # lattice
```
| Argument | Default | What it does |
|---|---|---|
| `--min-edge`, `--max-edge` | 0.5, 1.25 m | Minimum and maximum edge length, for both methods |
| `--spacing` | 0.5 m | Lattice: spacing between nodes, rounded to cell side / n |
| `--rings` | all | Lattice: rings of linked neighbours (within min and max) |
| `--distribution` | `poisson` | Random nodes: `uniform` (classic PRM, with holes and clumps), `poisson` (minimum distance between nodes), `jittered` (one random point in every small square), `halton` (quasi-random, very uniform) |
| `--density` | 9 nodes/m² | Random nodes: how many per m² (4 = the density of the lattice at 0.5 m) |
| `--seed` | random | Random nodes: same seed, same graph. If missing one is drawn; the one used is in the log (`[GRAPH] Random nodes`, with the command to repeat it) and in `mission_summary.json` |

With random nodes the cell centres and the starting point are still nodes. The coverage (how far a point of the area can be from the nearest node) is measured and written in the log. To look at the graphs before going out: `python3 tools/bench_graph.py`. It uses the same parameters as the mission (`config.py`) and opens an image with the methods side by side and a table; at the end it prints, for each graph, the lines to put in `config.py` (or the arguments of `run_mission.py`) to use exactly that one in a mission, seed included. To try other values without touching `config.py`, change the TEST block at the top of the file. From the command line, with the same defaults as the mission:
```bash
python3 tools/bench_graph.py --compare-methods methods.png --seed 7 --zoom -2 2 -2 2       # the five methods side by side, with the numbers
python3 tools/bench_graph.py --graph random --distribution halton --seed 7 --preview halton.png --zoom -2 2 -2 2   # just one
```

---

## How a mission works

### Cells and exploration
The area is a grid of cells (`mission/cells.py`) relative to the robot pose at start-up: the **columns** go forward, the **rows** to the left. The cells are visited in a serpentine from the starting corner, or in rings if the robot starts in the centre or mid-side. The next cell (`mission/cell_rules.py`): among the cells the graph can reach from the robot, the first in the visiting order whose way costs at most `NEAR_PATH_M` (4 m on flat ground), reached in cruise without stopping; if none is that near, the first in the visiting order among all, reached with GraphNav. A cell is done when its goal is reached or when 90% of it is already seen in the global map, wherever the robot is (also before moving: no trip towards a cell already seen); it is blocked when every candidate point is inside an obstacle; after `MAX_FAILURES` (2) real failures from the same entry it is set aside and comes back when the graph has a new way into it (`mission/exploration.py`).

### A cell attempt (`CellAttempt` in `mission/cell_attempt.py`)
The attempt is a cruise (`mission/attempt_cruise.py`): its start (scan, maps, graph, goal and path, `attempt_scan.py`) is the first cycle of the planning thread, then the robot walks along the path without stopping while the planning thread re-plans (`mission/planning_cycle.py`). The start and the outcome are in `cell_attempt.py`.
1. **Scan** of Spot's local grid (obstacle_distance, terrain, terrain_valid): corrected terrain, never-written cells discarded, slope and roughness, obstacle mask (`perception/spot_grid.py`).
2. **Global maps** accumulated over the whole mission: occupancy (an obstacle counts only after several observations), terrain, free space (`perception/global_maps.py`).
3. **Graph** (`planning/mission_graph.py`, `planning/graph_nodes.py`, `planning/roadmap.py`, `planning/edge_evaluation.py`): the nodes are random nodes with the cell centres included, or a regular deterministic lattice at 0.5 m spacing with the centre of every cell as a node (see *Mission graph*); the edges go from 0.5 to 1.25 m. Every edge is vetoed or weighted with occupancy, longitudinal and lateral slope (also measured beyond the nodes, by half a body on each side) and a preliminary discard on the live scan. At cell start only the edges near the new data are re-evaluated.
4. **Goal** (`planning/goal.py`): among the points of the cell not inside obstacles, first those reachable from the robot in the graph, with at least 0.33 m of free space and 0.30 m from the cell border; a goal refused by the frontier stays excluded for the whole mission. Arrival also counts with the robot up to 0.35 m outside the cell. **Path** with Dijkstra, only over the cells already visited, the robot's, the goal's and the cells next to both (`mission_prm.planning_cells`; every cell with `--goals frontier`): it goes wherever a path exists, however long the way round.
5. **Cruise** with the **safe frontier** (`planning/safe_frontier.py`, `mission/planning_cycle.py`): every 0.4 s the planning thread takes a fresh scan, updates maps and edges, runs Dijkstra and the frontier, and the stretch confirmed free of each plan is sent to Spot as one path command (`motion.PathCommand`) that replaces the previous one without stopping. No way on is a block at once: another goal in the cell, or the next cell, chosen while walking; blocked on the spot several times without moving, the attempt ends and the cell rules count a failure.
   - The robot walks forward or backward along the path, never sideways; the rotation at the start of a path is made on the spot inside the same command.
   - While walking it drops GraphNav "breadcrumbs" (waypoints) along the way: the GraphNav trips between cells and the return to `wp_0` use them.
6. **Gait** from the terrain of the confirmed stretch (`robot/motion.select_mobility_params`, `perception/gait.py`): PLAIN, MODERATE (crawl), HARD (crawl), with maximum speeds per level; in cruise a lighter level only after the plans have asked for it for a while.

### Safety
- Speed limits on every command, stop halfway through a move, time limit for every move and every GraphNav trip.
- Fall detection: the robot gets up, the spot is marked as dangerous in the map; at the third fall the mission stops.
- **Battery** (`robot/battery.py`): charge read at every turn and saved in `battery.csv` with the consumption per minute and per metre. Below `BATTERY_RETURN_PCT` (today 30%, provisional) the mission stops exploring and closes as at the end of a mission, with the return to `wp_0`; the unexplored cells stay in the frontier.
- GraphNav re-anchors the localization on the nearest waypoint if it gets lost; it never declares "arrived" without being there.
- If the mission is interrupted (exception or Ctrl+C) with the robot powered, the robot is stopped, sat down and powered off, and the GraphNav map is downloaded.

### End of the mission
It gets here when the frontier is empty, or earlier if the battery drops below the return threshold. Final scan, summary (`[MISSION STATS]`, `[TIMING]`), Spot loop closure on the whole GraphNav graph (at most 45 s) and anchoring, return to `wp_0` with GraphNav (with shortcuts chosen on the way), sit, power off, download of the GraphNav graph. If the starting fiducial says the robot stopped more than 0.30 m from the start, GraphNav is relocalized on the tag and the robot goes back to `wp_0` again (at most twice). During exploration Spot's loop closure is off: the trips use our shortcuts (edges seen free in the global map). At every GraphNav arrival the odometry drift is measured (`drift.csv`).

---

## Code layout

| Path | What it does |
|---|---|
| `run_mission.py` | Starts a mission (`spot_explorer.mission.run.main`) |
| `spot_explorer/config.py` | All the mission parameters, read as `P.NAME` |
| `spot_explorer/records.py` | Statistics, logs and saved data (CSV, scan bundles, views); nothing here decides the movement |
| `spot_explorer/legacy.py` | Old Italian names of tags, values and keys, so the tools still read missions recorded before 30-10-2026 |
| `spot_explorer/mission/` | `run.py` start-up, closure and command line; `exploration.py` the cell-by-cell loop (`Mission`); `cell_rules.py` which cell next; `frontier.py` the frontier goals of `--goals frontier`; `cell_attempt.py` one entry into a cell (`CellAttempt`), with its start in `attempt_scan.py`, the cruise in `attempt_cruise.py` and `planning_cycle.py`, outcomes in `attempt_common.py`; `mission_prm.py` the graph during the mission (build, edge update, allowed cells); `recovery.py` the retreat after a fall; `cells.py` the cell grid; `summary.py` options line and end-of-mission summary |
| `spot_explorer/planning/` | `mission_graph.py` the mission graph and its edge update; `graph_nodes.py` lattice and random nodes; `roadmap.py` the PRM (graph, penalties, Dijkstra); `edge_evaluation.py` edge vetoes and weights; `edge_stats.py` veto statistics; `goal.py` goal choice in a cell; `safe_frontier.py` the safe frontier and its thread |
| `spot_explorer/perception/` | `spot_grid.py` the local grid and its processing; `global_maps.py` global maps; `slopes.py` edge slope; `geometry.py` robot footprint and clearances; `gait.py` gait thresholds; `low_obstacles.py` the filter of low intermittent obstacles (grass) on their history |
| `spot_explorer/graphnav/` | `map_recording.py` recording, waypoints, breadcrumbs, loop closure, download; `navigation.py` GraphNav moves and return to `wp_0`; `localization.py` localization, re-anchoring, drift; `map_shortcuts.py` shortcuts; `apriltags.py` starting fiducial and correction of the return |
| `spot_explorer/robot/` | `connection.py` robot connection and E-Stop; `motion.py` motion commands (`PathCommand`), gait, falls; `battery.py` charge and early return; `photos.py` camera photos at blocks; `errors.py` the error rules (which SDK errors are caught) |
| `spot_explorer/sim/` | the simulated Spot of `run_mission.py --sim`: robot and SDK clients, world, controller, GraphNav, lidar, link to Isaac Sim (see `sim/README_SIM.md`) |
| `tools/` | `mission_report.py`, `mission_viewer.py`, `live_viewer.py` (no SDK needed); `preflight.py`, `compare_logs.py`, `profile_cycle.py`; `bench_graph.py` (also the graph command line), `bench_loop_closure.py`, `bench_fiducial.py`, `bench_cruise.py` (planning cycle speed, e.g. on the CORE I/O) bench tests; `grid_rate_check.py` (how often Spot renews its grids), `calibration.py` (cost weights and gait thresholds from the logs), `gait_trial.py` (gait trials on a track, on the robot), `capture_scene.py` (standing capture for the grass); `core_*.sh`, `core_check.py`, `estop_laptop.py` (missions from the CORE I/O); `behaviour_snapshot.py` (what the code decides on saved scans, replays and fake missions: same behaviour before and after a cleanup step), `saved_scan.py` (a saved scan back into Spot's messages), `replay_cruise_ends.py` (the cruise ends of real missions planned again with today's code); `sim_viewer.py`, `sim_sensor_viewer.py`, `sim_selftest.py`, `sim_turn_check.py` (the simulator) |
| `tests/` | `test_e2e_loop.py`, `test_mission.py`, `test_replay.py` (+ `replay_reference.json`), `test_goal_selection.py`, `test_path_command.py` (+ `fake_spot.py`), `test_cruise.py`, `test_frontier.py`, `test_any_goal.py`, `test_coverage.py`, `test_errors.py`, `test_sim_core.py` |

---

## Mission options

The options that change how a mission runs. The log reports them in the `[CONFIG] SWITCHES`
line and `mission_summary.json` in the `switches` entry. Alternatives still being tried are not
in this code: they are variants in the `varianti/` folder (branch `varianti` of this repository),
which replace functions of this code at start-up (`python3 varianti/run.py --with <variant> --
<arguments of run_mission.py>`).

| Option | Default | What it does |
|---|---|---|
| `config.MISSION_GOALS` | `cells` | Where the goals come from: `cells` or `frontier` (`--goals`, see *Running a mission*) |
| `config.MISSION_GRAPH_SAMPLING` | `random` | How the nodes are generated: `lattice` or `random` (see *Mission graph*) |
| `geometry.ROBOT_CLEARANCE_AIR_M` | `0.05` | Air between body and obstacles: the path line stays 0.25 m + this value (0.30 m) from the obstacles, one threshold for frontier, edges, nodes and goals |
| `config.GRAPHNAV_ARRIVAL_HEADING` | `toward_goal` | Heading on arrival of the GraphNav trips between cells and of the returns: facing the cell to attempt (or the goal of the attempt); `waypoint` = the waypoint's, `free` = no final rotation. The return to wp_0 always keeps the starting heading |
| `config.WP0_FIDUCIAL_CORRECTION` | `True` | At the end of the mission, if the starting tag says the robot is more than `WP0_FIDUCIAL_TOLERANCE_M` (0.30 m) from the start, relocalization on the tag and a new return to wp_0 (at most twice) |
| `config.GRAPHNAV_RETURN_NEAR_M` | `1.0` | Return to wp_0 stopped within this distance: it stays unsuccessful, but fiducial and drift are measured anyway |
| `config.GRAPHNAV_SHORTCUTS` | `True` | Shortcuts in the GraphNav graph before every move |
| `config.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S` | `45.0` | Full loop closure at the end of the mission, before anchoring (0 = off) |
| `config.BATTERY_RETURN_PCT` | `30.0` | Below this charge (2 readings in a row) the mission closes and goes back to `wp_0` (0 = return off, the charge is measured anyway) |

---

## What is saved

Every mission has a name `Mission_<day-month-year>_<time>` and three folders:

| Folder | Content | Typical size |
|---|---|---|
| `MissionLogs/<mission>/` | `mission_log.txt` (all the output, with the time on every line), `segments.csv` (one leg per row: slope, gait, real tilt; missions up to 25-11, the stop-and-go legs), `trajectory.csv` (position 5 times per second during the moves), `frontier_live.csv`, `drift.csv`, `battery.csv` (charge and consumption), `mission_summary.json` | 0.5 MB |
| `MissionMap/<mission>/` | `scans/` (one bundle per scan: raw and derived layers, frontier, decision), `*_vis.npz` (graph views), `*_BLOCKED_*.npz`, global maps `.pkl` | 70 MB |
| `graph/<mission>/` | downloaded GraphNav graph (`Graph_<date>`, `Grafo_<date>` in missions before 30-10-2026), with the waypoint snapshots | 150 MB |

---

## During the mission

```bash
python3 tools/live_viewer.py                   # in a second terminal: the latest mission, live
python3 tools/live_viewer.py 28-10-2026_14-23-50 --replay --speed 20   # a finished mission, replayed
```

A single figure with everything on it: map seen so far (red obstacle, green free), latest scan in the light-blue box, cells (green visited, red blocked, blue border = goal), PRM graph, path taken (blue, dashed = GraphNav), planned path (orange), goal (star), point where the frontier stops (X) and allowed advance (thick green), robot (arrow), correction of the return to wp_0 (crimson). On the right phase, cell, decision, frontier, progress, battery, return to wp_0 and the last useful log lines. It only reads the files the mission already writes: it does not slow it down and cannot stop it. Keys: `f` follow the robot / whole mission, space pause, `q` quit.

## After the mission

```bash
python3 tools/mission_report.py                        # report of the latest mission (or: mission name, --all)
python3 tools/mission_viewer.py 13-10-2026_12-40-38    # scans, global map and graph, interactive (--png <dir> to export)
python3 tests/test_replay.py --missions <mission>      # replay the scans through today's code
python3 tests/test_replay.py --all --graph random --distribution halton --seed 7   # same scans, another graph
```

- **`mission_report.py`**: one page with cells, timings, smoothness of the motion, gaits and tilts, frontier stops, return to wp_0, and the **test checks** (OK / CHECK / NOT TRIGGERED / DECISION).
- **`mission_viewer.py`**: the figures are made here, afterwards; during the mission the robot draws nothing.
- **`test_replay.py`**: checks a change without the robot, on the real scans of 10 missions (`--all`, about 5 minutes): invariants that must always hold, and comparison with the reference behaviour (`replay_reference.json`). With `--graph` (and `--distribution`, `--density`, `--seed`, `--min-edge`, `--max-edge`) it replays the same scans with another graph: the final table (paths found, length, edges per path, graph time) is compared with that of the lattice.
- **`test_e2e_loop.py`**: the fake robot and worlds (corridors, traps, walls) the other tests use, and the attempt against them.
- **`test_frontier.py`**, **`test_any_goal.py`**: the frontier goals; any node of the graph as the goal, reached or refused, never a hang.
- **`test_coverage.py`**, **`test_errors.py`**, **`test_sim_core.py`**: the area seen; the error rules; the simulator without the SDK.
- **`test_path_command.py`**: the multi-point path command of the continuous motion (`motion.PathCommand`) against `fake_spot.py`, a fake Spot with a simulated clock that reads the real command protos: corner without stopping, new sends while walking, dead man, stop, goal blocked, network errors.
- **`test_cruise.py`**: the cruise (continuous motion) with the real planning thread in the worlds of `test_e2e_loop.py`, the fake Spot in real time (~4 min): open field, path changes while walking, a box appearing ahead, new goal while walking, hand-offs between cells, big turns.
- **`test_mission.py`**: the whole mission (the real `run.main()`) with a fake robot and GraphNav, in several worlds: walls, closed cell, blocks, returns, forced failures, low-battery return, emergency shutdown, correction of the return to wp_0.
- **`test_goal_selection.py`**: the goal choice replayed on the real missions.
- **`compare_logs.py`**: compares the logs of two versions (`E2E_LOG_DIR=<folder> python3 tests/test_e2e_loop.py`, same for `test_mission.py`), removing times and durations: a change that must not alter the behaviour must give identical logs.
- **`preflight.py`**: checks that the modules are consistent with each other.

Every change is checked with the tests (`tests/`), `test_replay.py --all`, `tools/behaviour_snapshot.py` and `preflight.py` before going on the robot.

---

## Conventions

- **Coordinates**: Spot's VISION frame. x, y and heading are relative to the pose at mission start (`env.set_origin`); heights to the ground height at start-up (`mission_z0`), because Spot measures them from where it was powered on.
- **Cells**: columns forward, rows to the left; the starting cell is (0,0) unless `--start` says otherwise.
- **Sides of a cell**: 4 bits, `0b1000` north, `0b0100` east, `0b0010` south, `0b0001` west.
- **GraphNav waypoints**: `wp_0` at the start, then numbered in order (`wp_1`, `wp_2`, …).
- **Log**: every line has a tag in square brackets (`[ATTEMPT]`, `[FRONTIER]`, `[BLOCK]`, `[GRAPHNAV]`, `[TIMING]`, …), which stays stable: messages may change, tags do not. Missions before 30-10-2026 have Italian logs (`[FRONTE]`, `[BLOCCO]`, `[TEMPO]`, …); the tools read both.
