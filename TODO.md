# To do

Ideas, open problems and postponed decisions, with the data needed to take them. When
something is done or a decision is taken it is removed from here and goes into `CHANGELOG.md`,
with the reason: progress is read there, here only what is missing.

---

## State at the move to `main` (30-10-2026)

**Field tests of 29-10.**
- *10:57, validation* (8 x 5 cells of 2 m, lawn sloping ~8 degrees uphill, bushes and a hedge along the left side): 40/40 cells at the first attempt, no block, stall or error; the new goal after a failed replan triggered once and worked. The return to wp_0 stopped ~1.2 m from the start (see below).
- *Afternoon* (15:13, 15:27, 16:32; same 8 x 5 grid, in a strip ~3.5 m wide between a wall or hedge ~3 m to the left of the start and another edge on the right): 8–10 cells each, the rest correctly left out (the wall cuts the grid; the cells behind it were never seen, `[FINE] ... Le altre non sono mai state raggiungibili`). Returns to wp_0 0.04–0.08 m from the start according to the tag: the correction with the fiducial did not need to trigger, so it has **never triggered on the robot** yet. 15:31: Ctrl+C halfway through the mission, the robot sat down, motors off, GraphNav map downloaded: the **emergency shutdown works**. 15:27: `KeepaliveMotorsOffError` at power-on (known: restart by hand).

**Return to wp_0 off by ~1.2 m (29-10 10:57)**: correction with the fiducial done on 29-10 afternoon (see CHANGELOG), still to be seen triggering in the field. Analysis: GraphNav said "arrived", but the starting tag seen on arrival (1.59 m, +1.03 m to the left, against 1.08 m and -0.13 m at the start) said the robot was ~1.2 m to the right of the start; confirmed by eye. Odometry drift 2.82 m in 15 min (uphill, grass); GraphNav corrected 1.86 m of it, the residue (~1.3 m) is all along the hedge, the only long straight reference: ICP knows the distance from the hedge but not the position along it. The graph (odometry edges, including our 9 shortcuts) carries all the drift from wp_153 to wp_0. The tag was in the snapshot of wp_0 and visible on arrival, but GraphNav did not use it to correct. Anomaly to understand: right after the initialization on the fiducial GraphNav's loss detector said LOST until the re-anchoring at 11:06 (no effect on the exploration).

**In the field** (until the grass is handled)

- Start on low or bare ground, never inside tall grass: 28-10 13:50 (return blocked), 14:17 (no cell reachable, 21 s), 14:19 (3 cells). The log now says so: `[END] No cell reachable from the start`.
- Full battery: on 28-10 14:23 it started at 66% and ended at 37%, a few minutes from the automatic return at 30% (consumption 1.15–1.19 %/min, 0.18–0.19 %/m).
- Marker 1–2 m from the start, not moved during the whole mission.

---

## Decisions with the data of 28-10

| Question | Data of 28-10 | Decision |
|---|---|---|
| **Final loop closure** (full only, at the end of the mission, limit 45 s) | 16 edges in 55.5 s (13:50), 20 in 65.6 s (14:23); uploaded with `upload_graph` without errors or a second attempt | Stays like this. Spot overruns the limit by 10–20 s, growing with the graph: to keep an eye on, possibly a lower limit. |
| **Returns without the quick closure** | All GraphNav trips successful (3 and 8), shortcuts 116 -> 27 m and 132 -> 70 m | Removed from the code on 29-10. |
| **Odometry drift** | 14:23: **1.09 m in 26 min on the tag**, 0.92 m on GraphNav; 13:50: 0.19 m after 4 min | Far above the 0.3 m threshold: the correction of the global maps must be done (main work, point 3). |
| **Low-battery return threshold** (30%) | 1.15–1.19 %/min, 0.18–0.19 %/m; return to wp_0 of 14–21 m cost 1 point | The return costs little: 30% has a wide margin, it could go down to 20–25%. To be decided with bigger missions. |
| **`CLEARANCE_MAP_DECIDES`** | Median disagreement 7–9% of the edges; no gap proposed more than once | Removed from the code on 29-10: the binary map decides; the free-space map stays for the goal choice. |
| **Gaits** | HARD on calm legs (below 8°): 6 (13:50), 9 (14:23) | Little time lost; to be reviewed together with the smooth motion. |
| **GraphNav breadcrumbs every 0.75 m** | Map download 76–94 s at the end of the mission | To try 1.0–1.5 m when working on the motion. |

---

## Loop decisions: what really triggers (count of 29-10)

Over the 15 missions since 22-10 (~420 attempts): walked retreat 0 times since 12-10 (it stays as the last net and after falls); falls and stall 0; trap 2; multi-node leg interrupted 3; cone 8, straight back-off 18, leg discarded for rotation 19; `[EARLY]` ~100, confirmed blocks 36, returns with GraphNav ~30; mission recovery 10 (2 successful). Possible simplifications that CHANGE the behaviour, to try with the robot after `main`: cone, back-off and trap as a single fan of exit directions (the cone is the directions close to the wanted one, the back-off the one at 180 degrees); a single escalation ladder with one counter (discard the leg, look again standing still, exit direction, return with GraphNav, give up the side) instead of a dozen separate counters; a single "is the goal still fine?" check at the start of the turn.

## Main work, after the version on `main`

1. **Grass and obstacles that "move"** from scan to scan. The main problem of 28-10: vegetation is seen as a wall, in our map and in GraphNav's obstacle avoidance.
   - *Labelled data of 28-10* (the robot knew where the tall grass was):
     - 13:50, start in a patch of grass and cells (7,0)-(7,1) of grass. Obstacles around the start: median height 0.11 m above the ground (95% below 0.48), intensity 19 against 54 of the free ground. In the corner 0.06 m and intensity 19 on the "free" too (all lawn). In the rest of the mission 0.18 m (95% below 0.99), intensity 21 against 70. First attempt blocked next to the start (122 cells out of 800 under the body became obstacle as soon as the robot turned).
     - 14:17, start in the middle of the grass: obstacles on 54–67% of the cells seen around the robot, 4–15 cm high, intensity equal to the ground; no cell reachable.
     - 14:19, grass ahead (cell (0,3): obstacles 34%, median 0.12 m) and bushes on the left ((1,0)-(1,1): 58–76%, median 0.5–0.7 m, 90% above 1.1 m). On the final map, with a 0.25 m margin (the frontier's) no passage to (0,3); with 0.15 m 74% of the cell becomes reachable. It was not the density of the graph.
   - *Goal*: low vegetation (a few cm, low intensity) crossable or with a reduced margin; bushes above ~0.5 m stay obstacles. A rule behind a switch, tested on the replay of these missions. Also on 26-10: 24% of the low obstacles had the intensity of the ground, against 5% of the tall ones.
2. **Smoother motion and non-blocking loop.**
   - *Data of 28-10* (13:50, 14:23, from `trajectory.csv`): during the legs the robot advances 74–75% of the time, rotates in place 8%, and is **still 17–18% (~165 s per mission)**, in short pieces: ~0.6 s per leg halfway through the advance, in all gaits, and ~0.45 s per leg at the end, waiting for `BODY_STATUS_SETTLED` (the settling steps: the "walking in place"). The legs are ~0.9 m: the mean speed per leg is 0.17 m/s in PLAIN with a 0.5 limit, 0.11 m/s in MODERATE and HARD. The robot never reaches cruising speed: it turns, accelerates, slows down, settles, stops.
   - *First stage*: continuous paths instead of points with a stop (multi-point trajectories, turning while walking), no settling on intermediate legs.
   - *Second stage*: motion, more frequent scans, update of weights and graph running while the robot walks, instead of today's sequential loop (the only thing in parallel is the safe frontier). It changes the structure of the loop: after the first stage, with the end-to-end test compared line by line. Both behind a switch, then a few cells in the field before leaving them on.
   - On the same occasion: the planner ignores rotations (U-turns impossible in narrow corridors, 28-10 11:36 and the "funnel" scenario of the end-to-end test).
3. **Drift correction in the global maps**, with `drift.csv` and the starting fiducial (1.09 m in 26 minutes on 28-10, 2.8 m in 15 minutes on 29-10): realign the global map with GraphNav's localizations, or forget old observations faster when the robot passes there again.
4. **Are the cells still needed?** See idea 5 below. On 28-10 they weighed: the "just outside" arrival makes it ambiguous where the robot is (cells (7,0)-(7,1) never attempted at 13:50, fixed); the sides make attempts skip; a corner of grass stays "to do" only because the grid splits it into two cells.

---

## Ideas to evaluate (28-10)

Not decided yet: first it must be said what exactly they do and how to prove they work.

1. **Goal chosen by us on the global graph, once exploration is over.** Today the goal is always the point of a cell to explore. A way is needed to tell the robot "go there" on a map already built, using the graph and the global maps instead of exploring again. It is a new way of use, not a change to the current loop: probably a separate script.
2. **Densify the graph in narrow areas, locally and dynamically.** Today the node density is decided once for the whole area (`MISSION_RANDOM_DENSITY`). Narrow gaps need more nodes, elsewhere they are time wasted. To be defined how a "narrow area" is recognized (the measured free space is already available) and whether the new nodes are added to the existing graph or the area is rebuilt. Tested on the replay before the robot. (On 28-10 14:19 the bottleneck was the grass, not the density.) Sweep test of 28-10 evening, case 65: 0.7 m gap between two walls without nodes on its line, cell missed. Case 88: robot stopped under the corner of an obstacle, from there no graph edge towards the cell (3 usable directions out of 24), cell missed although 87% reachable. With the new goal after a failed replan (28-10 evening) cases 65 and 88 succeed. Sweep test of the whole missions (40 hard worlds): the cells left open were all unreachable with a 0.30 m margin, except those of case 8: robot in a 0.65 m corridor between two walls, the grid nodes every 0.5 m fall too close to the walls and the graph does not get out (three cells missed).
3. **Recheck `find_best_point_in_cell`**, and then the big question: **are the cells still needed?** If the goal becomes a trajectory decided beforehand, the cell grid and all the attempt/frontier/rank logic become scaffolding that can be done without. It is the biggest change of the list: first the check of the function (there is already `tests/test_goal_selection.py`), then a written evaluation of what is gained and what is lost, then if anything the code.

---

## Code structure

Done (see CHANGELOG, 27-10 to 30-10): the `spot_explorer` package (`mission/`, `planning/`, `perception/`, `graphnav/`, `robot/`), parameters in `config.py`, records in `records.py`, tools in `tools/`, tests in `tests/`; everything in English, with `legacy.py` so the tools still read the Italian missions.

Still possible, not urgent:
- split the largest modules further: `mission/cell_attempt.py` (~1750 lines, one class), `planning/mission_graph.py` (~1550), `planning/roadmap.py` (~1350);
- in `CellAttempt` the ~20 scan attributes (`pts_up`, `terrain_up`, `grad_up`, …) gathered into a single `Scan` object;
- also save in `scans/` the start-of-attempt, recovery and final scans as full bundles (today the viewer rebuilds them from the PRM views).
