# Field test: what to check

The changes of 26-10 (rows with that date) went on the robot for the first time on 28-10; those from 14-10 to 22-10 went on the robot on 22-10 indoors and outdoors. Some of 13-10 were in the code of the 12:14–12:40 missions but never triggered, because the situation did not occur. This list says what to look at and what to expect; `tools/mission_report.py` makes the same checks by itself, in the **Test checks** section.

Since 30-10-2026 the code, the logs and this list are in English. Earlier missions have Italian logs (`[BLOCCO]`, `[TEMPO]`, ...): the tools read both.

---

## Before leaving

All the mission switches and numbers are changed in **`spot_explorer/config.py`**. Those specific to grids and maps stay in `spot_explorer/perception/`, those of the motion in `spot_explorer/robot/motion.py`.

1. **Copy the whole folder** to the PC that commands the robot, not a single file. On 05-10 an old copy of one module stopped the mission right after the graph was built.
2. On the robot's PC: `python3 tools/preflight.py` → it must say **"All files are consistent with each other."**
3. **Mission size.** Chosen in `config.py`: `MISSION_GRID_ROWS`, `MISSION_GRID_COLS`, `MISSION_CELL_SIZE_M = 2`, today 8 × 7, i.e. 16 m to the left × 14 m ahead of the robot, 56 cells. If the area is different, without touching the code: `python3 run_mission.py --rows 6 --cols 6 --cell 2`. The first `[CONFIG] Mission grid` line of the log confirms it.
4. **Graph.** Default: random poisson nodes, 9 nodes/m², seed drawn at random. The `[CONFIG] Graph` and `[GRAPH] Random nodes` lines of the log report the seed and the command to rebuild the same graph.
5. **A fiducial** (AprilTag 36h11, the Boston Dynamics ones; not an ArUco) 1–2 m from the robot at the start, in front or behind: it sees it from any camera. The log confirms it at start-up with `[FIDUCIAL] Starting fiducial: tag N at X m, behind/in front`; without it the mission starts anyway, but the return to wp_0 cannot be corrected. Back on wp_0 the line `[FIDUCIAL] Back on wp_0 I see again tag N … it moved by D m` is the odometry drift of the whole mission: to have it the tag must stay where it is.

---

## The features to verify

| Feature | Log line | What to expect | If it goes wrong |
|---|---|---|---|
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
| **Refresh with slopes in batch** (21-10) | `[TIMING] scan … \| PRM X \| …` at every turn | PRM below ~0.5 s per turn (on 13-10 12:40 it was 1.35 s on average). | `REFRESH_BATCH_SLOPES = False` in `planning/roadmap.py`: same results, only slower. |
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
- **Smoothness of the motion**: today the robot walks at 0.13–0.14 m/s on average, because every leg waits for the body to be still and rotates separately. The report measures it in the **Smoothness** line (mean speed, time standing to rotate or start). If the change is prepared before the test, it must be compared here, with the switch on and off.
