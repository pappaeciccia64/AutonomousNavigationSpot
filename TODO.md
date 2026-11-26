# To do

Ideas, open problems and postponed decisions, with the data needed to take them. When
something is done or a decision is taken it is removed from here and goes into `CHANGELOG.md`,
with the reason: progress is read there, here only what is missing.

---

## In the field

Until the grass is handled:

- Start on low or bare ground, never inside tall grass: 28-10 13:50 (return blocked), 14:17 (no cell reachable, 21 s), 14:19 (3 cells). The log now says so: `[END] No cell reachable from the start`.
- Full battery: on 28-10 14:23 it started at 66% and ended at 37%, a few minutes from the automatic return at 30% (consumption 1.15–1.19 %/min, 0.18–0.19 %/m).
- Marker 1–2 m from the start, not moved during the whole mission.

---

## Decisions with the data of 28-10

| Question | Data of 28-10 | Decision |
|---|---|---|
| **Odometry drift** | 14:23: **1.09 m in 26 min on the tag**, 0.92 m on GraphNav; 13:50: 0.19 m after 4 min | Far above the 0.3 m threshold: the correction of the global maps must be done (main work, point 3). |
| **Low-battery return threshold** (30%) | 1.15–1.19 %/min, 0.18–0.19 %/m; return to wp_0 of 14–21 m cost 1 point | The return costs little: 30% has a wide margin, it could go down to 20–25%. To be decided with bigger missions. |
| **Gaits** | HARD on calm legs (below 8°): 6 (13:50), 9 (14:23) | Little time lost; to be reviewed with the cruise data. |
| **GraphNav breadcrumbs every 0.75 m** | Map download 76–94 s at the end of the mission | To try 1.0–1.5 m. |

---

## Main work

1. **Grass and obstacles that "move"** from scan to scan. The main problem of 28-10: vegetation is seen as a wall, in our map and in GraphNav's obstacle avoidance.
   - *Labelled data of 28-10* (the robot knew where the tall grass was):
     - 13:50, start in a patch of grass and cells (7,0)-(7,1) of grass. Obstacles around the start: median height 0.11 m above the ground (95% below 0.48), intensity 19 against 54 of the free ground. In the corner 0.06 m and intensity 19 on the "free" too (all lawn). In the rest of the mission 0.18 m (95% below 0.99), intensity 21 against 70. First attempt blocked next to the start (122 cells out of 800 under the body became obstacle as soon as the robot turned).
     - 14:17, start in the middle of the grass: obstacles on 54–67% of the cells seen around the robot, 4–15 cm high, intensity equal to the ground; no cell reachable.
     - 14:19, grass ahead (cell (0,3): obstacles 34%, median 0.12 m) and bushes on the left ((1,0)-(1,1): 58–76%, median 0.5–0.7 m, 90% above 1.1 m). On the final map, with a 0.25 m margin (the frontier's) no passage to (0,3); with 0.15 m 74% of the cell becomes reachable. It was not the density of the graph.
   - *Spot's side, under our map* (checked 03-11 on the SDK 5.2 protos and docs):
     - **GraphNav does not use our `MobilityParams`.** `navigate_to` takes none; on each edge it uses the edge's `mobility_params`, fixed at recording time by `RecordingEnvironment.edge_environment`. We pass only `waypoint_env` (`graphnav/map_recording.py`): on all 2186 edges of the 15 graphs in the repo `mobility_params` is empty. So during the GraphNav returns the foot obstacle avoidance is ON (Spot's default), the very `no_step` that sees grass as obstacle and that we turn off everywhere else. Possible cause of the `STUCK` of 13:50 (return to wp_0, stopped after 33 s at 0.63 m), not proven: the log had only the status. From 03-11 the log gives `stuck_reason`, `blockage_status` and `route_following_status` (CHANGELOG). Candidate fix, behind a switch: our `obstacle_params` in `edge_environment` (only for maps recorded after it).
     - **`ground_clutter_mode` does NOT help**: `GROUND_CLUTTER_FROM_FOOTFALLS` makes objects below 30 cm not stepped on during recording into obstacles. Leave it unset.
     - **`obstacle_avoidance_padding`** (body): never set by us, so 0 is sent. The docs give no default; the SDK example `fiducial_follow` sends 0.001 to have almost none, which suggests 0 = robot default. The "~7.5 cm" quoted before is not confirmed. Only a field test gives the value.
     - **Hazard Avoidance Service** (`TYPE_PREFER_STEP_ON`): it clears hazards of the same service, not the stereo classification behind `obstacle_distance`; "prefer" types only in autonomous navigation. Not a way for grass.
     - **`MobilityParams.hazard_detection_mode`** (semantic hazards, needs a CORE I/O; `COST` = areas to avoid instead of blocking obstacles): never set by us. To check if it is on by default on our robot, if it has a CORE I/O.
   - *Goal*: low vegetation (a few cm, low intensity) crossable or with a reduced margin; bushes above ~0.5 m stay obstacles. A rule tried first as a variant (`varianti/`), on the replay of these missions. Also on 26-10: 24% of the low obstacles had the intensity of the ground, against 5% of the tall ones.
2. **Drift correction in the global maps**, with `drift.csv` and the starting fiducial (1.09 m in 26 minutes on 28-10, 2.8 m in 15 minutes on 29-10): realign the global map with GraphNav's localizations, or forget old observations faster when the robot passes there again.
3. **Are the cells still needed?** See idea 5 below. On 28-10 they weighed: the "just outside" arrival makes it ambiguous where the robot is (cells (7,0)-(7,1) never attempted at 13:50, fixed); a corner of grass stays "to do" only because the grid splits it into two cells.

---

## Ideas to evaluate (28-10)

Not decided yet: first it must be said what exactly they do and how to prove they work.

1. **Goal chosen by us on the global graph, once exploration is over.** Today the goal is always the point of a cell to explore. A way is needed to tell the robot "go there" on a map already built, using the graph and the global maps instead of exploring again. It is a new way of use, not a change to the current loop: probably a separate script.
2. **Densify the graph in narrow areas, locally and dynamically.** Today the node density is decided once for the whole area (`MISSION_RANDOM_DENSITY`). Narrow gaps need more nodes, elsewhere they are time wasted. To be defined how a "narrow area" is recognized (the measured free space is already available) and whether the new nodes are added to the existing graph or the area is rebuilt. Tested on the replay before the robot. (On 28-10 14:19 the bottleneck was the grass, not the density.) Sweep test of 28-10 evening, case 65: 0.7 m gap between two walls without nodes on its line, cell missed. Case 88: robot stopped under the corner of an obstacle, from there no graph edge towards the cell (3 usable directions out of 24), cell missed although 87% reachable. With the new goal after a failed replan (28-10 evening) cases 65 and 88 succeed. Sweep test of the whole missions (40 hard worlds): the cells left open were all unreachable with a 0.30 m margin, except those of case 8: robot in a 0.65 m corridor between two walls, the grid nodes every 0.5 m fall too close to the walls and the graph does not get out (three cells missed).
3. **Recheck `find_best_point_in_cell`**, and then the big question: **are the cells still needed?** If the goal becomes a trajectory decided beforehand, the cell grid and all the attempt/frontier/rank logic become scaffolding that can be done without. It is the biggest change of the list: first the check of the function (there is already `tests/test_goal_selection.py`), then a written evaluation of what is gained and what is lost, then if anything the code.

4. **A* for the path search (later, not now; 19-11).** Ready as the local variant `a_star` (`varianti/a_star.py`): heuristic alpha x straight-line distance, admissible (no edge of the saved missions costs less than its length). Its test: same path cost in 191/191 random searches, 63% less search time; the search is ~15% of the planning cycle. Why it waits: on the replay of all missions (`test_replay --all`) the cost is always the same but in 4 attempts the path chosen among EQUAL-cost ones has one edge more -- A* explores in another order and breaks the ties differently, against the preference for long edges. Before using it: decide the tie rule (fewer edges at equal cost, or the same order as Dijkstra) and check it on the replay. For now Dijkstra stays (decision of 19-11).
5. **Without cells: a route instead of a grid (discussion of 19-11).** Ideas weighed and where they ended:
   - *Goal on the frontier* (a seen node with an edge to an unseen one, the one closest to the serpentine, chosen with one Dijkstra from the robot): the frontier moves with the robot (~2 m ahead), so the goal runs away at every scan, the path is 1-3 edges and the choice is myopic (a U of bushes). Weighed on 19-11 as the fallback for the zones left behind; since 26-11 it is the option `--goals frontier` (below), to compare in the simulator.
   - *Coverage (new area seen from a node) as a weight in the goal choice*: easy to count, hard to use well (mixed units to tune, overestimated behind obstacles, changes at every scan). Kept as the MEASURE of progress and the end condition: the "area seen" of `[PROGRESS]` does not need cells.
   - *Goal sliding along the serpentine, also through unseen space*: with fixed waypoints every ~2-3 m it is the cell centres in visiting order -- the goal is the same as today. What is really new is only the path: the distance from the route as a cost in Dijkstra, so that with the continuous motion the robot sweeps the strip of the visit while it walks.
   - So the experiment is that alone, with the cells kept: the variant `route_line` (the route cost with `--goals cells`; the frontier goals use it already), compared on fake missions by `varianti/tools/compare_route_line.py`. If it helps, removing the cells becomes a simplification of the code (cell choice, cell rules), not a change of behaviour. First result: little difference (CHANGELOG 19-11 evening).
   - Built anyway, as an alternative goal source for a wider sensing (lidar): since 26-11 the mission option `--goals frontier` (`mission/frontier.py`): Dijkstra from the robot with the route penalty, stop at the first frontier node; goals prepared while walking. With Spot's grid: short goals, slower. To try in the simulator (SIMULATOR.md) and with a lidar window.
---

## Code structure

Done (see CHANGELOG, 27-10 to 30-10): the `spot_explorer` package (`mission/`, `planning/`, `perception/`, `graphnav/`, `robot/`), parameters in `config.py`, records in `records.py`, tools in `tools/`, tests in `tests/`; everything in English, with `legacy.py` so the tools still read the Italian missions; the largest modules split by role (`CellAttempt` by phase, the PRM into graph, edge evaluation and statistics, `mission_graph` into nodes, graph and mission use).

Still possible, not urgent:
- also save in `scans/` the start-of-attempt, recovery and final scans as full bundles (today the viewer rebuilds them from the PRM views).
