# History of changes

Most recent changes at the top. For each one: what changed, why, and the field data that motivated it. The code only describes how it works today; this file tells how it got there.

Missions are given as `day-month time` (for example `13-10 12:40`); their data is in `MissionLogs/` and `MissionMap/`.

Until 29-10-2026 the code was a flat set of Italian-named modules (`easy_walk.py`, `parametri.py`, `spotGrid.py`, `arcVerification.py`, `navGraphUtils.py`, `movements.py`, `registro.py`, `environmentMap.py`, `prm_graph.py`, `batteria.py`, `visualizzatore_live.py`, `prova_*.py`, `test_missione.py`, `test_obiettivi.py`, ...) with Italian logs. The entries below that date use those names, log lines and values as they were at the time; the entry of 30-10 says where each piece went.

---

## 2026-10-30 (largest modules split by role, behaviour unchanged)

The three largest modules are split by role, so each file can be read on its own. The robot's behaviour does not change: after every step the logs of the end-to-end tests and of the fake missions are identical line by line, the replay of the reference missions and the goal-selection replay are unchanged, and the mission graphs are the same for 12 sets of parameters (lattice and random, errors included).

- **`CellAttempt`** keeps the current scan as one `Scan` object (before, ~20 attributes `pts_up`, `terrain_up`, `grad_up`, ...) and is split by phase: `mission/cell_attempt.py` the loop, guards, decision and shared actions; `attempt_scan.py` scan, maps, graph update and replan; `attempt_obstacles.py` rotation refused, wait and block; `attempt_legs.py` the legs; `attempt_common.py` the phase outcomes and the battery checks (`battery_return_due`, `battery_low`).
- **PRM**: `planning/roadmap.py` the graph, the penalties, anchoring and the path search (`find_path_dijkstra` = `_dijkstra`, `_within_caps`, `_stable_plan`); `edge_evaluation.py` occupancy, slope, weight and the refresh around the robot (`refresh_local_edge_weights` = `_refresh_nodes`, `_refresh_pairs`, `_refresh_edge`), with the constants `LIVE_PREFILTER_OD_M`, `REFRESH_RADIUS_M`, `REFRESH_BATCH_SLOPES`, `DIRTY_MARGIN_M`; `edge_stats.py` the veto statistics and the `[SLOPES]` line.
- **Mission graph**: `planning/graph_nodes.py` lattice and random nodes, pairs, coverage (with the reasons for the lattice); `mission_graph.py` `MissionGraph` and `build_mission_graph`, with `add_edges_to_prm` and the lattice build in steps; `mission/mission_prm.py` the graph during the mission (`build_mission_prm`, `rebuild_prm_edges`, `restrict_planning_to_explored_cells`, `graph_reaches_cell`, `node_cells`, `robot_on_node`). The command line moves to `tools/bench_graph.py` (`python3 tools/bench_graph.py --compare`, `--timings`, `--preview`, ...); `--compare` failed on a wrong attribute name since the translation, now it works.
- **Imports by the module's own name**: `motion`, `cells`, `connection`, `roadmap`, `records`, `config`, `battery` instead of the old aliases `movements`, `environmentMap`, `spotUtils`, `prm_graph`, `registro`, `parametri`, `batteria`. In `mission_summary.json` the switch keys become `config.X`, `motion.X`, `edge_evaluation.REFRESH_BATCH_SLOPES`; the tools read only the last part of the key, so old and new summaries read the same.
- `tools/preflight.py` follows the PRM methods into `edge_evaluation.py`.

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
