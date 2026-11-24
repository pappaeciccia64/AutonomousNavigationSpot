# Running the mission in a simulator (plan of 19-11)

The mission talks to Spot only through a few clients of the Boston Dynamics SDK. The tests
already replace them with fakes (`tests/test_mission.py`, `tests/test_cruise.py`,
`tests/fake_spot.py`, `tests/test_e2e_loop.py`) and run the REAL mission code end to end. A
simulator (Isaac Sim or other) plugs in at the same four points: what the fakes do today is
the reference of what the bridge must do.

## What the bridge must provide

| # | Today in the tests | What the simulator must give | Used by |
|---|---|---|---|
| 1 | `test_e2e_loop.FakeLocalGrid.return_local_grid` (the world of rectangles, `window()`) | the local grids around the robot, as Spot gives them: `obstacle_distance` (signed distance from the nearest obstacle, m, negative inside), `terrain` (ground height in VISION, m), `terrain_valid` (1 = height reliable); 128 x 128 cells of 3 cm centred on the robot; the pose of the grid (transform VISION -> grid frame) and, if possible, its `acquisition_time`. From a simulator: a height map around the robot (ray casts or a depth camera) and an obstacle mask, then a distance transform | `perception/spot_grid.py` (`fetch_scan`, `return_local_grid`): every scan of the stop-and-go, of the cruise and of the frontier check |
| 2 | `fake_spot.FakeSpot` (+ `test_cruise.RTSpot`, real time) | `robot_command` with the SE2 trajectories built by `robot/motion.py` (real `bosdyn.api` protos: points with times, speed limits, end time), `robot_command_feedback` (status: moving, at goal, blocked), `get_robot_state` (kinematic snapshot VISION -> body, ideally the ground plane `gpe`, body pitch/roll). In a simulator: turn the trajectory into velocity commands for the walking policy, or move the robot kinematically for the first tests | `robot/motion.py` (`PathCommand`, legs of the stop-and-go), `robot/state_cache.py`, the fall check, the tilt recorder |
| 3 | `test_mission.FakeGraphNav` (graph in memory, `navigate_to` puts the robot on the waypoint) | nothing new: the in-memory GraphNav of the tests can stay as it is in the simulator (it moves the robot through the bridge, point 2) | `graphnav/*`: waypoints, returns, shortcuts, return to wp_0 |
| 4 | login, E-Stop, lease, power: patched in `test_mission.run` | the same patches | `mission/run.py` start-up |

Everything else is the real code: global maps, PRM graph, Dijkstra, safe frontier, planning
thread, cruise, cell or frontier exploration, logs and saved data (`MissionLogs/`,
`MissionMap/`) readable with `tools/mission_viewer.py` and the other tools.

## What is ready (sim/, 20-11)

The bridge is written and runs against a reference backend without physics:

| File | What |
|---|---|
| `sim/backend.py` | `SimBackend`: the methods a simulator must implement (clock, pose, follow a path, path status, teleport, ground height, obstacles, what is seen) |
| `sim/kinematic_world.py` | `KinematicWorld`: complete example backend -- boxes, ground made of planes / hills / steps; the robot follows the path exactly, stops before obstacles and slopes above 30 degrees, body roll and pitch follow the ground |
| `sim/spot_adapter.py` | from a backend, what the mission expects from Spot: the local grids as real `LocalGridResponse` protos (decoded by the mission's own `spot_grid`), `SimSpot` (commands, feedback, robot state with body, gpe, power, battery), `MemoryGraphNav`, `SimRobot` (`ensure_client` by service name) |
| `sim/worlds.py` | scenes: `open`, `hill` (as 28-10), `slope` (13 degrees, as 29-10), `step`, `walls` (as the room of 03-11) |
| `tools/run_sim_mission.py` | the unchanged mission (`mission/run.py`) against a backend: `--world`, `--with` variants of `varianti/`, `--set module.NAME=value`, arguments of run_mission.py after `--`; data under `--out` |
| `tests/test_sim_bridge.py` | grids decoded by the real code, path command on the simulated robot, a whole mission |

    python3 tools/run_sim_mission.py --world hill
    python3 tools/run_sim_mission.py --world walls --with frontier   # a variant of varianti/

## Writing the backend of a real simulator

One class, `SimBackend`, in a new file (e.g. `sim/isaac_backend.py`), then in
`tools/run_sim_mission.py` the backend built from it instead of `worlds.scene(...)`:

- `now()` / `step_to(t)`: the simulator clock in seconds. The mission runs in real time and
  calls `step_to` with the wall clock: a simulator stepping on its own only has to be at least
  that far (or be stepped there).
- `pose()`: body position and yaw, roll, pitch (from the base link of the robot model).
- `follow_path(points, max_linear, max_angular)`: points `(x, y, yaw, t)`. First version:
  the kinematic way of `KinematicWorld._step` (set the base pose along the path). With a
  walking policy: a path follower (target point at time `now()` on the path, velocity command
  towards it, capped) feeding the policy.
- `path_status()`: `moving`, `near_goal`, `at_goal`, `blocked`, `stopped`; `blocked` when the
  robot makes no progress towards the target for a while (it is what Spot reports).
- `teleport(x, y, yaw)`: for the in-memory GraphNav.
- `terrain(xs, ys)` / `obstacles(xs, ys)` / `seen(xs, ys, robot_xy)`: the world around the
  robot at the cell centres of the grid. From a simulator: rays down from above on the grid
  points for the ground height; obstacle where the hit is more than ~0.25 m above the ground
  under the robot's feet or on a collision body tagged as obstacle; `seen` from rays from the
  robot's camera height (occlusions) or simply within range.

Everything else -- grids as protos, commands, robot state, GraphNav -- is `sim/spot_adapter.py`
and stays the same.

## Steps

1. **Kinematic first**: robot moved by the bridge exactly along the commanded trajectory
   (as `FakeSpot` does), grids computed from the simulated scene. It checks the perception
   bridge (point 1) against a known world.
2. **With the walking policy**: the trajectory followed by the simulated legs; errors,
   delays and slips appear, the feedback must report them (deviation, blocked).
3. **Terrain**: slopes, steps, rough ground -- the gait choice and the slope costs get
   exercised, which the rectangle worlds of the tests cannot do.
4. **Wider sensing** (a simulated lidar): the same three layers on a larger window; the
   exploration by frontier (the `frontier` variant, `varianti/frontier.py`) is meant for it.

## What to compare there

`varianti/tools/compare_route_line.py` runs the same mission in each world once per variant
(`cells`, `route`, `frontier`) and prints cells, metres walked, distance from the route,
area seen, time. In a simulator the same comparison, and `tools/cruise_timing.py` on the
mission logs for the timing switches (FIELD_TEST.md, "Timing of the cruise").

## Test plan for the first days in the simulator

Each run: note scene, seed, variant and switches (the `[CONFIG] SWITCHES` line has them),
keep the mission folder. Tools: `tools/mission_report.py` (outcome, `Coverage:`),
`varianti/tools/cruise_timing.py` (timing of the cruise), `tools/mission_viewer.py` (maps,
scans, path), `coverage.csv` (area seen over time). The alternatives live in the local
`varianti/` folder since the cleanup of 20-24/11: turn them on with `--with name,name` of
the runner (`python3 varianti/run.py --list` lists them); parameters with
`--set module.NAME=value`.

**0. Before anything** -- the colleague's bridge (office PC, 20-11, not on GitHub yet): push it,
read how it gives the four points of the table above, compare with `sim/spot_adapter.py`.

**1. Bring-up (an hour).** In order, each a short check:
   - grids: an object of known size and place in the scene; in the mission viewer it must be
     where it is, `obstacle_distance` zero on its border, `terrain` the right height on a slope;
     no strip of never-written cells where the sensor does see (`[GROUND-FILTER]` lines);
   - path: one `PathCommand` to a point 2 m ahead -- `arrived` in about the commanded time,
     `blocked` before a wall; then the same while the next one replaces it (cruise);
   - robot state: body pitch / roll on a slope match the scene, `gpe` under the feet;
   - a 2 x 2 cells mission on flat ground, cells mode: it ends, returns to wp_0.

**2. Baseline, cells mode** (the behaviour of the robot today), three scenes: open ground
with a few boxes, a hill or a slope of 10-15 degrees, a corridor with obstacles on one side.
Measure: cells closed, area seen and its growth (`coverage.csv`), metres, minutes, cruise speed
and stops (`[CRUISE] End`), plans read (`cruise_timing.py`). Expected from the fake missions:
all reachable cells, 98-100% seen, cruise 0.3-0.35 m/s.

**3. Timing variants** (FIELD_TEST.md, "Timing of the cruise"), one at a time on the open
scene: `cruise_state_cache`, `event_driven_planning`, `ground_from_grid` (only if the bridge
puts `gpe` in the grid snapshot), and `sleep_between_turns` (the old fixed sleep: the waking
on a new plan is in the main code since 24-11). Look at: plan read delay, plans read of those
published, main thread turn. The network
latency of a simulator is not the robot's: the order of the gains counts, not the absolute
numbers.

**4. Where the goal comes from**: cells against frontier (variant `frontier`: `--with frontier`
of the runner, or `python3 varianti/run.py --with frontier -- <run_mission args>`; with
`FRONTIER_BEYOND_M` of `varianti/frontier.py` at 0 and at 1.0, the goal beyond the edge), same scenes and
seeds. Expected with Spot's 3.84 m grid: as good as the cells in the open, slower among walls
(goals ~1 m apart). If the simulator can give a wider grid (a simulated lidar: more cells, the
code reads the size from the protos), run the frontier with it: there it should pull ahead.

**5. Terrain and gait** on the slope and on a step of 10-20 cm: gait levels chosen
(`[GAIT]`), edge slopes against the ground under the feet (`tools/slope_check.py`), no stop
in front of a step Spot can take.

**6. Write down** a table per run (scene, variant, cells, % seen, metres, minutes, cruise
speed, stops, failures) -- the comparison of 4 and 3 decides what to try next on the robot.

## What a simulator will not tell

Spot's own internals: how it builds and filters its grids (noise, grass seen as obstacle or
not), its body obstacle avoidance, the real GraphNav (localization, loop closure, drift).
Those stay for the robot.
