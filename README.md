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

**Parameters.** All the numbers and switches that are changed in the field are in `spot_explorer/config.py`; those specific to a module (terrain thresholds in `perception/`, graph in `planning/roadmap.py` and `planning/edge_evaluation.py`, frontier in `planning/safe_frontier.py`, speeds in `robot/motion.py`) stay there.

**Mission size.** The default is in `config.py` (`MISSION_GRID_ROWS`, `MISSION_GRID_COLS`, `MISSION_CELL_SIZE_M`): today 8 rows × 7 columns of 2 m, i.e. 16 m to the left and 14 m ahead of the robot. For a single mission, without touching the code (in order of precedence):
```bash
python3 run_mission.py --rows 2 --cols 3 --cell 4      # arguments
SPOT_MISSION_GRID="2,3,4" python3 run_mission.py       # environment variable
```
The `[CONFIG] Mission grid` line of the log says which value was used and where it came from.

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
The area is a grid of cells (`mission/cells.py`) relative to the robot pose at start-up: the **columns** go forward, the **rows** to the left. The cells are visited in a serpentine; at every step the neighbouring frontier cell with the lowest rank is chosen. If none borders the current position, the robot goes with GraphNav to the waypoint nearest to the lowest-rank one and tries from there. A cell that failed from one side stays a candidate and is retried later from another side; it becomes blocked only when all sides have been tried, or at once if the goal turns out to be inside an obstacle (`mission/exploration.py`, `mission/recovery.py`).

### A cell attempt (`CellAttempt` in `mission/cell_attempt.py`)
The attempt prepares scan, maps, graph and path (`_prepare`), then repeats a turn of phases (`_guards`, `_scan_and_graph`, `_frontier_and_decision`, `_rotation`, `_node_or_arrival`, `_wait`, `_block`, `_execute_legs`) until the robot arrives or the attempt fails. The loop and the decision are in `cell_attempt.py`; the scan phases in `attempt_scan.py`, rotation, wait and block in `attempt_obstacles.py`, the legs in `attempt_legs.py`.
1. **Scan** of Spot's local grid (obstacle_distance, terrain, terrain_valid): corrected terrain, never-written cells discarded, slope and roughness, obstacle mask (`perception/spot_grid.py`).
2. **Global maps** accumulated over the whole mission: occupancy (an obstacle counts only after several observations), terrain, free space (`perception/global_maps.py`).
3. **Graph** (`planning/mission_graph.py`, `planning/graph_nodes.py`, `planning/roadmap.py`, `planning/edge_evaluation.py`): the nodes are random nodes with the cell centres included, or a regular deterministic lattice at 0.5 m spacing with the centre of every cell as a node (see *Mission graph*); the edges go from 0.5 to 1.25 m. Every edge is vetoed or weighted with occupancy, longitudinal and lateral slope (also measured beyond the nodes, by half a body on each side) and a preliminary discard on the live scan. At cell start only the edges near the new data are re-evaluated.
4. **Goal** (`planning/goal.py`): among the points of the cell not inside obstacles, first those reachable from the robot in the graph, with at least 0.33 m of free space and 0.30 m from the cell border; a goal refused by the frontier stays excluded for the whole mission. Arrival also counts with the robot up to 0.35 m outside the cell. **Path** with Dijkstra, only over the cells already visited, with a cap on the detour.
5. **Navigation loop** with the **safe frontier** (`planning/safe_frontier.py`, `planning/decisions.py`): at every turn a fresh scan tells how far the path is free now, and the robot advances up to there, even over several nodes in a row. During the move a thread recomputes the frontier about 5 times per second and stops the robot if it gets shorter.
   - The robot always points its nose at the point to reach (no sideways walking). If it cannot rotate: it advances inside the possible rotation cone, or backs up straight if from there the rotation becomes possible, otherwise it discards the leg.
   - A blocking cell is discarded at once and the path replanned; a block confirmed by three still scans becomes permanent.
   - If no direction of the circle is travelable (trap), the robot goes back with GraphNav along the "breadcrumbs" (waypoints dropped every 0.75 m) and replans from there; after two returns in the same attempt it gives up the cell from that side.
6. **Gait** chosen leg by leg (`robot/motion.select_mobility_params`): PLAIN, MODERATE (crawl), HARD (crawl), with maximum speeds per level.

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
| `spot_explorer/config.py` | All the mission parameters and switches, read as `P.NAME` |
| `spot_explorer/records.py` | Statistics, logs and saved data (CSV, scan bundles, views); nothing here decides the movement |
| `spot_explorer/legacy.py` | Old Italian names of tags, values and keys, so the tools still read missions recorded before 30-10-2026 |
| `spot_explorer/mission/` | `run.py` start-up, closure and command line; `exploration.py` the cell-by-cell loop (`Mission`); `cell_attempt.py` one entry into a cell (`CellAttempt`), with its phases in `attempt_scan.py`, `attempt_obstacles.py`, `attempt_legs.py` and `attempt_common.py`; `mission_prm.py` the graph during the mission (build, edge update, allowed cells); `recovery.py` going back, retreat, blocked cells; `cells.py` the cell grid; `summary.py` switches and end-of-mission summary |
| `spot_explorer/planning/` | `mission_graph.py` the mission graph and its edge update; `graph_nodes.py` lattice and random nodes; `roadmap.py` the PRM (graph, penalties, Dijkstra); `edge_evaluation.py` edge vetoes and weights; `edge_stats.py` veto statistics; `goal.py` goal choice in a cell; `safe_frontier.py` the safe frontier and its thread; `decisions.py` what to do given the frontier (step, rotation, cone, back-off, traps) |
| `spot_explorer/perception/` | `spot_grid.py` the local grid and its processing; `global_maps.py` global maps; `slopes.py` edge slope; `geometry.py` robot footprint and clearances; `gait.py` gait thresholds |
| `spot_explorer/graphnav/` | `map_recording.py` recording, waypoints, breadcrumbs, loop closure, download; `navigation.py` GraphNav moves and return to `wp_0`; `localization.py` localization, re-anchoring, drift; `map_shortcuts.py` shortcuts; `apriltags.py` starting fiducial and correction of the return |
| `spot_explorer/robot/` | `connection.py` robot connection and E-Stop; `motion.py` motion commands, gait, falls; `battery.py` charge and early return |
| `tools/` | `mission_report.py`, `mission_viewer.py`, `live_viewer.py` (no SDK needed); `preflight.py`, `compare_logs.py`; `bench_graph.py` (also the graph command line), `bench_loop_closure.py`, `bench_fiducial.py` bench tests |
| `tests/` | `test_e2e_loop.py`, `test_mission.py`, `test_replay.py` (+ `replay_reference.json`), `test_goal_selection.py` |

---

## Switches

Each one allows an on/off comparison in the field; the log reports them in the `[CONFIG] SWITCHES` line and `mission_summary.json` in the `switches` entry.

| Switch | Default | What it does |
|---|---|---|
| `config.MISSION_GRAPH_SAMPLING` | `random` | How the nodes are generated: `lattice` or `random` (see *Mission graph*) |
| `mission_graph.BATCH_SLOPES` | `True` | Slopes at cell start computed in batch (same result, faster) |
| `edge_evaluation.REFRESH_BATCH_SLOPES` | `True` | Slopes of the refresh computed in batch (same result, faster) |
| `geometry.ROBOT_CLEARANCE_AIR_M` | `0.0` | Air between body and obstacles: the path line stays 0.25 m + this value from the obstacles (frontier and edge veto). 0.05 = with margin |
| `geometry.PLAN_CLEARANCE_EXTRA_M` | `0.03` | On the global map and in the goal choice, the planner keeps the edges at (frontier distance + this value) from obstacles, so the scan noise does not make the frontier refuse the legs just chosen. 0 = same distance as the frontier |
| `config.PLAN_ONLY_EXPLORED_CELLS` | `True` | Paths only go through the cells already visited |
| `config.GRAPHNAV_ARRIVAL_HEADING` | `toward_goal` | Heading on arrival of the GraphNav trips between cells and of the returns: facing the cell to attempt (or the goal of the attempt); `waypoint` = the waypoint's, `free` = no final rotation. The return to wp_0 always keeps the starting heading |
| `config.WP0_FIDUCIAL_CORRECTION` | `True` | At the end of the mission, if the starting tag says the robot is more than `WP0_FIDUCIAL_TOLERANCE_M` (0.30 m) from the start, relocalization on the tag and a new return to wp_0 (at most twice) |
| `config.GRAPHNAV_RETURN_NEAR_M` | `1.0` | Return to wp_0 stopped within this distance: it stays unsuccessful, but fiducial and drift are measured anyway |
| `config.MOVE_NEAR_GOAL_M` | `0.10` | A leg whose command does not complete (timeout) with the robot within this distance of the point counts as successful instead of as a physical failure (0 = off) |
| `config.GRAPHNAV_PRECHECK` | `True` | Before going with GraphNav to the waypoint from which to attempt a cell, check on the graph that the cell can be reached from there; if not, mark the side as tried without moving |
| `config.GRAPHNAV_SHORTCUTS` | `True` | Shortcuts in the GraphNav graph before every move |
| `config.GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S` | `45.0` | Full loop closure at the end of the mission, before anchoring (0 = off) |
| `config.BATTERY_RETURN_PCT` | `30.0` | Below this charge (2 readings in a row) the mission closes and goes back to `wp_0` (0 = return off, the charge is measured anyway) |

---

## What is saved

Every mission has a name `Mission_<day-month-year>_<time>` and three folders:

| Folder | Content | Typical size |
|---|---|---|
| `MissionLogs/<mission>/` | `mission_log.txt` (all the output, with the time on every line), `segments.csv` (one leg per row: slope, gait, real tilt), `trajectory.csv` (position 5 times per second during the moves), `frontier_live.csv`, `drift.csv`, `battery.csv` (charge and consumption), `mission_summary.json` | 0.5 MB |
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
- **`test_e2e_loop.py`**: the real navigation loop against a fake robot in synthetic worlds (corridors, traps, walls).
- **`test_mission.py`**: the whole mission (the real `run.main()`) with a fake robot and GraphNav, in several worlds: walls, closed cell, blocks, returns, forced failures, low-battery return, emergency shutdown, correction of the return to wp_0.
- **`test_goal_selection.py`**: the goal choice replayed on the real missions.
- **`compare_logs.py`**: compares the logs of two versions (`E2E_LOG_DIR=<folder> python3 tests/test_e2e_loop.py`, same for `test_mission.py`), removing times and durations: a change that must not alter the behaviour must give identical logs.
- **`preflight.py`**: checks that the modules are consistent with each other.

Every change is checked with `test_replay.py --all`, `test_e2e_loop.py`, `test_mission.py` and `preflight.py` before going on the robot.

---

## Conventions

- **Coordinates**: Spot's VISION frame. x, y and heading are relative to the pose at mission start (`env.set_origin`); heights to the ground height at start-up (`mission_z0`), because Spot measures them from where it was powered on.
- **Cells**: columns forward, rows to the left; the starting cell is (0,0).
- **Sides of a cell**: 4 bits, `0b1000` north, `0b0100` east, `0b0010` south, `0b0001` west.
- **GraphNav waypoints**: `wp_0` at the start, then numbered in order (`wp_1`, `wp_2`, …).
- **Log**: every line has a tag in square brackets (`[ATTEMPT]`, `[FRONTIER]`, `[BLOCK]`, `[GRAPHNAV]`, `[TIMING]`, …), which stays stable: messages may change, tags do not. Missions before 30-10-2026 have Italian logs (`[FRONTE]`, `[BLOCCO]`, `[TEMPO]`, …); the tools read both.
