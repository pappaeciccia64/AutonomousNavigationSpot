# Field test: what to check

The changes of 26-10 (rows with that date) went on the robot for the first time on 28-10; those from 14-10 to 22-10 went on the robot on 22-10 indoors and outdoors. Some of 13-10 were in the code of the 12:14–12:40 missions but never triggered, because the situation did not occur. This list says what to look at and what to expect; `tools/mission_report.py` makes the same checks by itself, in the **Test checks** section.

Since 30-10-2026 the code, the logs and this list are in English. Earlier missions have Italian logs (`[BLOCCO]`, `[TEMPO]`, ...): the tools read both.

---

## Continuous motion outdoors (first time on 04-11)

Validated indoors on 03-11 in five missions (room of about 10 x 3 m, always gait PLAIN). Never tried: grass, gait changes in the middle of a cruise (MODERATE / HARD, CRAWL), cruises longer than ~3 m.

**Switches** (all on in the code):

| Switch | File | Off = |
|---|---|---|
| `CONTINUOUS_MOTION` | `spot_explorer/mission/attempt_cruise.py` | stop-and-go as on 03-11 morning (the whole cruise off) |
| `CHAIN_CELLS` | same file | the robot stops at every cell goal |
| `PATH_COMMAND_FOR_LEGS` | `spot_explorer/robot/motion.py` | the legs of the stop-and-go with rotation + walk as before |
| `INCREMENTAL_REFRESH` | `spot_explorer/planning/edge_evaluation.py` | full edge refresh |

To go back at once to the morning's behaviour: `CONTINUOUS_MOTION = False` (and `PATH_COMMAND_FOR_LEGS = False`). The `[CONFIG] SWITCHES` line of the log says which ones were on.

**Plan:** two missions on the same lawn, same grid and start: one with `CONTINUOUS_MOTION = False` as reference, one with it on. Tablet in hand; the first minutes in an open area.

**In the log, during the cruise:**
- `[CRUISE] Plan N: X m confirmed ahead ... | cycle C s, robot at D cm from the line`: `C` around 0.2 s (above 0.6 s the planning is late), `D` a few cm. `X` around 1 m on open ground; on grass it may be shorter (grass read as roughness or obstacle).
- `[GAIT] Terrain level: ...` printed again when the level changes in the middle of a cruise: the robot must change gait without stopping. Watch whether it stumbles or hesitates at the change.
- `[CRUISE] End: <reason>`. Expected: `in the cell ... next cell while walking`, `the confirmed stretch ends ... (obstacle)` near obstacles. To look at: many `(end_of_data)`, `(roughness)` or `(slope)` on open grass; `too far from the path`; `command expired`; `no new plan for ...`.
- `[CRUISE] New goal ... chosen while walking`, `[CRUISE] +N deg to turn ... without stopping first`.
- `[STOP] ... I stop and look again standing`: the turns and the cases where the stop-and-go takes over.

**Stop the robot:** Ctrl+C or the tablet. If the program hangs the command expires and the robot stops by itself within 2 s.

**After:** `python3 tools/mission_report.py` (line `Cruise (continuous motion)` and the next one: hand-offs, new goals, path changes, distance from the line, reasons of the ends) and `python3 tools/mission_viewer.py` (the scans of the cruise every 0.5 m, decision `cruise`; light blue contour = area really seen by the cameras). Slopes: `python3 tools/slope_check.py <mission>` (estimates against the body tilt and the ground plane of the legs, `gpe`, recorded from 04-11).

**Slopes from the local plane** (`PLANE_SLOPES` in `spot_explorer/perception/slopes.py`, on from 04-11 after the outdoor mission 14:08, see CHANGELOG): gait thresholds 0.13 / 0.20, edge veto 0.6, the frontier with the same estimate. To check on the next missions: `[GAIT] Terrain level` lines much rarer than on 14:08 (239 in 20 min), HARD only on the real slopes, frontier stops for `slope` rare. Against the legs: `python3 tools/plane_thresholds.py --since 2026-11-04` and `python3 tools/slope_check.py <mission>`. Off again: `PLANE_SLOPES = False` (everything goes back as before, thresholds included).

---

## Before leaving

All the mission switches and numbers are changed in **`spot_explorer/config.py`**. Those specific to grids and maps stay in `spot_explorer/perception/`, those of the motion in `spot_explorer/robot/motion.py`.

1. **Copy the whole folder** to the PC that commands the robot, not a single file. On 05-10 an old copy of one module stopped the mission right after the graph was built.
2. On the robot's PC: `python3 tools/preflight.py` → it must say **"All files are consistent with each other."**
3. **Mission size.** Chosen in `config.py`: `MISSION_GRID_ROWS`, `MISSION_GRID_COLS`, `MISSION_CELL_SIZE_M = 2`, today 8 × 7, i.e. 16 m to the left × 14 m ahead of the robot, 56 cells. If the area is different, without touching the code: `python3 run_mission.py --rows 6 --cols 6 --cell 2`. The first `[CONFIG] Mission grid` line of the log confirms it.
4. **Graph.** Default: random poisson nodes, 9 nodes/m², seed drawn at random. The `[CONFIG] Graph` and `[GRAPH] Random nodes` lines of the log report the seed and the command to rebuild the same graph.
5. **A fiducial** (AprilTag 36h11, the Boston Dynamics ones; not an ArUco) 1–2 m from the robot at the start, in front or behind: it sees it from any camera. The log confirms it at start-up with `[FIDUCIAL] Starting fiducial: tag N at X m, behind/in front`; without it the mission starts anyway, but the return to wp_0 cannot be corrected. Back on wp_0 the line `[FIDUCIAL] Back on wp_0 I see again tag N … it moved by D m` is the odometry drift of the whole mission: to have it the tag must stay where it is.

---

## Indoors first (04-11 morning, before going out)

1. **Why GraphNav stops.** Put a box on a GraphNav stretch (the return to wp_0, or a move between cells) once the robot is on its way; in a passage it cannot go around (GraphNav may deviate up to 1.5 m from the recorded path). Expected: `[GRAPHNAV] …: route …, blockage ROUTE_BLOCKED_TEMPORARILY …` while it tries, then `interrupted -- blocked (STUCK) (reason OBSTACLE, …)`. If a field shows `-`, the SDK on the PC does not have it: note the `bosdyn` version (`pip show bosdyn-api`).
2. **Photos at the stop.** Same move: `[PHOTO] graphnav_…: 5 photos in X s` and in `MissionMap/<mission>/photos/` five JPEGs plus a `.json`. Also at a confirmed block (`[DATA SAVE] Block data saved in …BLOCKED_n.npz` followed by `[PHOTO]`). Look at one photo: it must show what was in front of the robot.
3. **Spot's own margin.** We send `obstacle_avoidance_padding` = 0, i.e. Spot's default, unknown. With a GraphNav move passing next to a box, note by eye (or tape measure) how close the body passes and whether Spot goes around it by itself.
4. **`tools/capture_scene.py`** once, in front of the box: it must print the layers (5), 5 photos and the obstacle cells with their height above the ground (the box: median close to its real height).
5. **A mission as on 03-11**, everything as it is: `[CRUISE]` lines, `next cell while walking`, `New goal ... chosen while walking`; `trajectory.csv` has the `gpe_long_deg` and `gpe_lat_deg` columns filled (the ground plane of the legs). Then `python3 tools/mission_report.py` and `python3 tools/slope_check.py <mission>` (on the flat floor every estimate and gpe close to 0).
6. **Someone steps in front of the robot during a cruise**, 1-1.5 m ahead. Expected: the `[CRUISE] Plan` lines show the confirmed stretch getting shorter (`(obstacle)`) or `PATH CHANGED`, and the robot goes around or stops before; never closer than the frontier allows. Then a **low box** (15-20 cm) on the path: the same. In the offline test (`test_cruise.py`) a 0.4 m box appearing 1.2 m ahead is passed with the centre 0.30 m from it.
7. **Bigger cells** (`MISSION_GRID_*` in `config.py`, e.g. 3 m): longer cruises; compare the report with the 2 m one.
8. **Gait trial**, standing and on 2 m: `python3 tools/gait_trial.py --terrain floor --true-slope 0 --estimate-only --repetitions 5`, then `--config auto_mu08 --dist 2 --repetitions 1 --backwards --protected-feet` (section "Gait calibration" below).

---

## Grass: standing capture (outdoors, ~10 minutes, before the mission)

The labelled data that decides how to treat grass (TODO, main work 1). Robot standing, moved with the tablet; on the PC `python3 tools/capture_scene.py`, one capture per situation, with a label saying what is in front and how far:
- on short grass, nothing in front (the reference: "free");
- 1 m from tall grass (and the height of the grass, measured);
- 1 m from a bush;
- 1 m from a solid object of similar height (a box, a bag);
- inside the tall grass, as at the start of 28-10 14:17.

Turn the robot by 90 degrees and capture again for one of them: the side cameras see the same thing from another point. Also a photo with the phone of each scene from behind the robot. The captures are in `SceneCaptures/<date_time>/`: load the folder to the repo with the mission.

---

## Gait calibration and cost weights

Two tools: **`tools/calibration.py`** (at the computer: reads the `segments.csv` of the missions and the track trials, prints what the data say) and **`tools/gait_trial.py`** (on the robot: the same straight stretch walked several times with a fixed gait, one row per pass in `GaitTrials/trials.csv`; it also records the slope estimates of the scan, edges and local plane, and the ground plane under the feet from the legs, `gpe`). Ported on 03-11 from `taratura.py` / `prova_andatura.py` of the old code: never tried on the robot, the first time indoors on 2 m of flat floor.

**What the logs already say** (`calibration.py --since 2026-10-22`: 904 stop-and-go legs from 22 missions):
- **Cost weights**: with the gait level separated, the longitudinal slope is worth ~1.8 m of path per unit (today beta = 2: fine); counting also the CRAWL it triggers, ~4.4. The lateral slope only ~0.7 in time (today 2: a safety margin, not a time cost). Roughness: on a rough edge (0.03) about a quarter of a metre: gamma = 0 changes little.
- **Failures**: 80 of the 155 are legs with 0 m travelled (refusals, blocks), which do not depend on the terrain. Those with the robot moving (8.3%) grow only above slope 0.3 (13% against 8%) and roughness 0.02 (11% against 8%).
- **Thresholds**: the roughness thresholds (0.05 and 0.10) almost never trigger (95th percentile 0.028). The HARD threshold at 0.25 catches 61% of the legs with peak pitch above 12 deg; at 0.20, 84%. NB these slopes are today's estimate, which reads ~2 deg too much on slopes (`tools/slope_check.py`).
- **Speeds**: real 0.18 / 0.12 / 0.11 m/s against limits 0.5 / 0.35 / 0.25: the limits slow down nothing.

**Indoors, before going out.** In `gait_trial.py` there is no safe frontier and Spot's foot collision avoidance is off as in the mission: indoors always `--protected-feet`. Space: track + 0.5 m beyond the end and at the sides; for `--back-turn` an area of ~1.6 x 1.6 m free at the start and at the end, otherwise `--backwards`. Before the first pass, check that Ctrl+C in the terminal makes the robot sit.

| What | Command | What to expect |
|---|---|---|
| The script works | `python3 tools/gait_trial.py --terrain floor --config auto_mu08 --dist 2 --repetitions 1 --back-turn --protected-feet` (little room: `--backwards` instead of `--back-turn`) | 2 m out, turn, back, turn; two rows in `GaitTrials/trials.csv` with all the columns filled (`slip_*`, `mu_est_*`, `energy_*`, `gpe_*` included). An empty column = data the robot does not give: note it. |
| Zero of the estimate | `python3 tools/gait_trial.py --terrain floor --true-slope 0 --estimate-only --repetitions 5` | Estimated slope close to 0 (below 3 deg) with both methods; the ground under the feet (gpe) close to 0. |
| Estimate on a ramp (if there is one) | robot still at the foot of the ramp, facing it: `--terrain inner_ramp --true-slope <deg> --estimate-only` | Edges a few degrees above the inclinometer (90th percentile), plane close to it. Then on the ramp: gpe along close to the true slope. |
| Speed limit | `--terrain floor --config auto_mu08 --dist 5 --repetitions 2`, then the same with `--vel 0.3` | Almost equal times = the limit is not what slows the robot. |
| Estimated friction | two passes on different floors (tiles, rubber mat) | Different `mu_est_mean`: an idea of the values Spot estimates. |

**Outdoors: tracks.** Straight tracks of 5 m, free of stones and branches, 1 m free beyond the end and at the sides. In order of importance: `mid_ramp` (constant 10-15 deg uphill: where MODERATE against HARD is decided, 0.15 = 8.5 deg, 0.20 = 11.3, 0.25 = 14), `lawn` (flat lawn: almost the whole mission), `cross` (the same slope walked across: roll, and the check of the lateral estimate against gpe), `steep_ramp` (18-25 deg, if there is one), `clods` (uneven ground: real roughness). For each one: slope with the inclinometer in 3 points with a straight board, cones at start and end, then `--estimate-only --repetitions 3`.

**Outdoors: configurations.** On every track, 3 repetitions with `--back-turn` (on the ramp the way back is the descent) of `auto_mu08` (today's PLAIN), `crawl_mu06` (today's MODERATE), `auto_high_mu06` (MODERATE without crawl: does the crawl really help?); on `steep_ramp` also `crawl_mu04` (HARD) and `auto_high_mu04`, a single pass of `crawl_mu04` first and nothing else if it slips or tilts beyond 25 deg. Change the order of the configurations from one track to the next. Below 35% battery skip `clods`.

**After: how to decide.** `python3 tools/calibration.py --trials GaitTrials/trials.csv`, section 5. If the estimate is systematically off the measured slope, read the thresholds with that correction (or switch to the plane estimate). If `auto_high_mu06` always succeeds with oscillation and slip like `crawl_mu06`, the crawl in MODERATE is not needed (faster mission). The slope where the normal gait starts to give clearly higher RMS or slip, or to fail, is `GAIT_MODERATE_SLOPE_MAX`. `ground_mu_hint` against `mu_est_mean` per ground type. Then the cost weights again on the new missions (`--since <date>`). Every change: `test_replay.py`, then the CHANGELOG with the numbers.

---

## The features to verify

| Feature | Log line | What to expect | If it goes wrong |
|---|---|---|---|
| **Why GraphNav stops** (03-11) | `[GRAPHNAV] …: route X, blockage Y after N s`; `interrupted -- … (reason R, route X, blockage Y, D m to go)` | On every GraphNav stop the reason; in the grass, whether it is `OBSTACLE` (Spot's own obstacle avoidance) or something else. | Log only. |
| **Photos at block and GraphNav stop** (03-11, `SAVE_BLOCK_PHOTOS`) | `[PHOTO] name: 5 photos in X s (photos/)` | One per confirmed block and per GraphNav stop, ~0.3 s; photos in `MissionMap/<mission>/photos/`. Never a mission stopped by it (`photos not taken` is only a message). | `SAVE_BLOCK_PHOTOS = False` in `config.py`. |
| **Return to wp_0 corrected with the fiducial** (29-10 afternoon, `WP0_FIDUCIAL_CORRECTION`) | `[WP0] According to the tag the robot is D m from the starting point`; if D > 0.30: `[WP0] CORRECTION 1/2 ...`, `[GRAPHNAV] correction 1 of the return to wp_0: arrived`, `[WP0] After correction 1: D m from the start (before D0 m)`. In the live viewer: PHASE "CORRECTION of the return to wp_0" and a crimson stretch | The tag must be visible from the start (1-2 m). At the end of the mission the robot stops within ~0.3 m of the start, checked by eye. | `WP0_FIDUCIAL_CORRECTION = False` in `config.py` (the measurement stays). |
| **Emergency shutdown** (28-10 evening) | `[EMERGENCY] Mission interrupted …`, `Robot sitting and motors off.`, `GraphNav map downloaded.` | A deliberate test, short and in a safe place: Ctrl+C halfway through a short mission; the robot sits down and the mission's `graph/` folder has the map. | If it does not sit down: E-Stop from the tablet. |
| **Battery and low-battery return** (28-10 evening, `BATTERY_RETURN_PCT = 30`) | `[BATTERY] start: charge N%`, a line `[BATTERY] loop: …` at every point lost; `battery.csv`; at the end of the mission `[BATTERY] Return to wp_0: from A% to B%` | The charge is read from start-up (if `Charge not available` appears, note it: the return cannot trigger). Sensible and stable consumption per minute and per metre. If the battery drops below 30%: `[BATTERY] RETURN`, then `End of exploration for low battery` and the normal closure up to wp_0. | `BATTERY_RETURN_PCT = 0` in `config.py` turns the return off (the measurement stays). |
| **Local graph update** and **slopes in batch** (14-10) | `[TIMING] local update of the graph: X s` (or `full build`) | A single full build, at the first attempt; then local updates of **1–3 s** (29-10 10:57: 0.3 s on average). | `BATCH_SLOPES = False` in `planning/mission_graph.py`: same results, only slower. |
| **Spot loop closure only at the end of the mission, full** (26-10 evening, `GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S = 45`; edges uploaded by us) | No `Spot loop closure` line before the trips; one line `(full): N edges in X s` before `[ANCHORING]` | GraphNav trips start 1–2 s after the preparation. At the end of the mission about a minute and fifteen or so edges (28-10: 16 in 55 s, 20 in 66 s). No `attempt N failed` line; if `retrying with Spot writing them itself` appears, our upload was rejected: note it. | `GRAPHNAV_SDK_LOOP_CLOSURE = False` in `config.py`. |
| **Odometry drift** (26-10 evening, measurement only) | `[DRIFT] …: localized on wp_N, created X min ago -> the odometry (VISION) is off by D m` at every GraphNav arrival; `drift.csv` | One line per arrival; the drift grows with the age of the waypoint (28-10 14:23: 1.09 m in 26 min on the tag; 29-10 10:57: 2.8 m in 15 min uphill on grass). Needed to decide whether and how to correct the global maps. | — |
| **Extra layers in the scans** (26-10, `SAVE_EXTRA_LAYERS`) | none: afterwards, `intensity` and `no_step` in the `scans/*.npz` bundles | Present in the bundles; `scan` time in the report almost unchanged. | `SAVE_EXTRA_LAYERS = ()`. |
| **Goal choice** (26-10: reachable, 0.33 m, 0.30 m from the border, refused ones excluded) | `[TARGET] Cell (r,c): …`, `[TARGET] Goal … excluded for the rest of the mission`, `NO point of the cell is reachable` | Few "GRAPH DISCONNECTED" at the start of the attempt; never the same goal chosen again after the exclusion (the report counts them). | Note cell and time; `tests/test_goal_selection.py --missions <name> --detail`. |
| **Arrival just outside the cell** (26-10, up to 0.35 m) | `[GOAL] The robot is outside cell … it counts as arrival` | Few `still outside cell` (on 22-10 there were 44). | `ARRIVAL_OUTSIDE_CELL_M = 0` in `config.py` goes back to the rule of 22-10. |
| **Return without movement** (26-10) | `[RETURN] Going back to wp_N, X m from the robot`, `… I do not move, I replan from here` | No GraphNav return of 0 m. | — |
| **Progress** (28-10) | `[PROGRESS] N/56 cells closed (X%) … area seen Y% … ~Z min to the end … battery W%` | One line at every closed cell; the time estimate settles after a few cells; the battery drops regularly. At the end `End of exploration` and the `Progress:` line in the report. | Log only: if a line looks odd, note it. |
| **Check on the arrival pose** (28-10 afternoon: the area not yet seen does not count as a wall) | `[ARRIVAL] The step of … no exits` | Very rare, and never three times in a row on the same step (on 22-10 there were 10, on 26-10 5, indoors at 12:16 6). | Note cell and time: look at the arrival point with the viewer. |
| **Goal right next to an obstacle** (28-10: new goal at the first frontier stop near or beyond the goal; final step below 0.40 m if it is enough to arrive; new goal after a block near the goal) | `[TARGET] The frontier stops … it is the goal that does not work`, `[FINAL STEP]`, `[TARGET] Block …` | Far fewer `[EARLY]` (on 22-10 there were 58, 45 with the obstacle right next to the goal; on 26-10 22, 18 next to it) and fewer blocks; never `[REPLAN] After the block` right after `excluded for the rest of the mission`. | Note cell and time. To go back: `FINAL_STEP_MIN_M = 1.0` (no final step); the goal rule has no switch. |
| **GraphNav arrival facing the cell** (28-10 evening, `GRAPHNAV_ARRIVAL_HEADING`) | `[GRAPHNAV] …: arrival facing (x, y), yaw N deg`; in the report "Rotation after the GraphNav arrivals" | Mean rotation after arrival well below the 79 degrees of 26-10 (expected ~20–30; 29-10 10:57: 4 degrees on 1 arrival), less than the 38 s in place. No trip failed because of the requested pose. | `GRAPHNAV_ARRIVAL_HEADING = 'waypoint'` in `config.py`. |
| **Failed replan: new goal at once** (28-10 evening, `REPICK_ON_REPLAN_FAIL`) | `[TARGET] Replan failed: no path from N to the goal … it is the goal that is isolated. I look for another one`, then `[REPLAN] To the new goal` | Far fewer `[REPLAN FAILED] … I keep the previous path` (on 28-10 14:23 there were 20) and fewer confirmed blocks with many free directions ("Usable directions from here: 21/24" followed by "No alternative"). The robot must not give up cells it used to reach (29-10 10:57: triggered once, worked). | `REPICK_ON_REPLAN_FAIL = False` in `config.py`. |
| **Leg not completed but arrived** (28-10 evening, `MOVE_NEAR_GOAL_M = 0.10`) | `[ALMOST] Command towards (x, y) not completed, but the robot is D m from the point` | Rare, mostly uphill or in HARD; afterwards the robot carries on from the node without going back. Note if after an `[ALMOST]` the next leg starts badly (a sign that 10 cm is too much). | `MOVE_NEAR_GOAL_M = 0` in `config.py`. |
| **Return to wp_0 stopped nearby** (28-10 evening, `GRAPHNAV_RETURN_NEAR_M = 1.0`) | `Return to wp_0 NOT successful but near: stopped D m from wp_0 (reason: …)` | Only if GraphNav gets stuck next to the start: fiducial and drift measured anyway, return marked as unsuccessful anyway. | — |
| **Check before GraphNav** (22-10 evening, `GRAPHNAV_PRECHECK`) | `[PRECHECK] Cell (r,c): from the waypoint …` | No GraphNav move followed by an immediate failure with "GRAPH DISCONNECTED". | `GRAPHNAV_PRECHECK = False` in `config.py`. |
| **Planning at 0.28 m** (22-10 evening, `PLAN_CLEARANCE_EXTRA_M = 0.03`) | `[FRONTIER] … obstacle 0.2x m from the centre line … (minimum 0.25 m)` | Far fewer frontier stops at 0.23–0.24 m (at 15:24 there were 18 turns out of 25). | `PLAN_CLEARANCE_EXTRA_M = 0` in `perception/geometry.py`. |
| **Poisson random nodes** (22-10) | `[GRAPH] Random nodes …`, `[TIMING] … graph …` | Graph built in less than a second; local updates much shorter than those of the 0.25 m lattice. Some narrow passage (below ~1 m) may have no nodes: the robot gives up that side and tries the cell from another one. | `--graph lattice --spacing 0.5` (or `0.25` for narrow passages, slower). |
| **2 m cells** (22-10) | `[ATTEMPT]`, `[TARGET]` | Shorter attempts; goal almost always the centre. | `--cell 4` with fewer rows and columns. |
| **0.25 m margin around the body** (22-10, `ROBOT_CLEARANCE_AIR_M = 0`) | `[FRONTIER]`, `[BLOCK]`, Spot's own stops near obstacles | The robot passes closer to obstacles (line at 0.25 m instead of 0.30) without touching them; no stop by Spot itself in narrow passages. | `ROBOT_CLEARANCE_AIR_M = 0.05` in `perception/geometry.py`. Note where it grazed or stopped. |
| **Refresh with slopes in batch** (21-10) | `[TIMING] scan … \| PRM X \| …` at every turn | PRM below ~0.5 s per turn (on 13-10 12:40 it was 1.35 s on average). | `REFRESH_BATCH_SLOPES = False` in `planning/edge_evaluation.py`: same results, only slower. |
| **Goal at the cell centre** (14-10) | `[TARGET] Cell (r,c): using the geometric centre …` or `centre …, using the … point … D m from the centre` | Almost always the centre; fallbacks within ~1 m of the centre. | Note cell and time: look at it with `tools/mission_viewer.py`. |
| **Slope with the body** in the veto and the gait (14-10) | `[GAIT] Terrain level: …`, `[SLOPES] build: …` | Every leg where the robot tilts more than 18 degrees is **HARD** (the report counts them; 29-10 10:57: 4 out of 5). Some more edges vetoed near steps. | `ARC_BODY_EXTENSION_M` in `perception/slopes.py`. |
| **GraphNav shortcuts on the way** (14-10) | `[GRAPHNAV] …: path to wp_X A m -> B m with N shortcuts` | Much shorter return to wp_0 (29-10 10:57: 85 m → 19.5 m). No `Shortcut … rejected` line. | `GRAPHNAV_SHORTCUTS = False` in `config.py` (back to the chain only). |
| **Terrain map without obstacle cells** (14-10) | no line: checked afterwards, with the replay | `tests/test_replay.py --missions <name>` → invariant 2 OK (global map slope equal to the live one). | — |
| **GraphNav re-anchoring** (13-10) | `[LOCALIZATION] …: LOST`, `[LOCALIZATION] Re-anchoring …: succeeded on wp_N` | If it gets lost, the re-anchoring succeeds and the move continues (29-10 10:57: 1 out of 1). | Note where: `[LOCALIZATION]` reports gait and pitch. |
| **Returns with memory and recovery** (13-10) | `[RETURN] Edge a->b out of the plan …`, `[RECOVERY] The goal … is that of the attempt that just failed` | Never the same loop trap → return → same path more than twice. | — |

---

## During the test

- To follow the mission: in a second terminal `python3 tools/live_viewer.py` (the latest mission, it updates by itself; `f` follows the robot or shows everything). Start it after the mission has started. If it closes or hangs, the mission is not affected.
- Note the **time** of every odd thing (the log has the time on every line): it is found at once in the log and in the scans.
- **Grass**: when the robot stops or goes around something that looks like grass, note the time and take a photo of what is in front. It serves as ground truth to compare the scans (with the `intensity` and `no_step` layers).
- The robot must never stay **still spinning idle**: if it happens, sooner or later `[STALL]` appears; note how long it takes.
- Grazing during rotations: the first number to change is `ROTATION_NOISE_TOLERANCE_M` in `perception/geometry.py` (0.15 → 0.10, see CHANGELOG 08-10).

---

## After the test

1. `python3 tools/mission_report.py` → report of the latest mission, with the **Test checks** at the end (OK / CHECK / NOT TRIGGERED / DECISION).
2. `python3 tools/mission_viewer.py <name>` to look at the points marked CHECK.
3. `python3 tests/test_replay.py --missions <name>` → the invariants on the new scans, including the return to wp_0 if the graph was downloaded. If all is well, the mission is added to `ALL_MISSIONS` in `test_replay.py` and to the reference with `--update-reference`.

---

## Decisions to take with the test data

- **CRAWL in MODERATE**: the report gives the pitch per gait. If the MODERATE legs stay calm (below ~8 degrees), it can go back to the normal gait (`HINT_AUTO` in `select_mobility_params`), which is faster.
- **Smoothness of the motion**: today the robot walks at 0.13–0.14 m/s on average, because every leg waits for the body to be still and rotates separately. The report measures it in the **Legs** line (mean speed, time standing to rotate or start, walking speed; GraphNav moves on their own line). If the change is prepared before the test, it must be compared here, with the switch on and off.
