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

## Steps

1. **Kinematic first**: robot moved by the bridge exactly along the commanded trajectory
   (as `FakeSpot` does), grids computed from the simulated scene. It checks the perception
   bridge (point 1) against a known world.
2. **With the walking policy**: the trajectory followed by the simulated legs; errors,
   delays and slips appear, the feedback must report them (deviation, blocked).
3. **Terrain**: slopes, steps, rough ground -- the gait choice and the slope costs get
   exercised, which the rectangle worlds of the tests cannot do.
4. **Wider sensing** (a simulated lidar): the same three layers on a larger window; the
   exploration by frontier (`exploration.GOAL_SOURCE = 'frontier'`) is meant for it.

## What to compare there

`tools/compare_route_line.py` runs the same mission in each world once per variant
(`cells`, `route`, `frontier`) and prints cells, metres walked, distance from the route,
area seen, time. In a simulator the same comparison, and `tools/cruise_timing.py` on the
mission logs for the timing switches (FIELD_TEST.md, "Timing of the cruise").

## What a simulator will not tell

Spot's own internals: how it builds and filters its grids (noise, grass seen as obstacle or
not), its body obstacle avoidance, the real GraphNav (localization, loop closure, drift).
Those stay for the robot.
