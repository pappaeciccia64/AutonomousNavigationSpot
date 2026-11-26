# Running the mission in a simulator

The simulation is the colleague's (GennaiG00/AutonomousNavigationSim, andrea-branch of 20-11),
brought onto this code on 26-11: **`sim/README_SIM.md`** says how to start it. In short:

    python3 sim/kinematic_server.py --world sim/worlds/boxes.json     # no physics, any computer
    sim/run_isaac.sh woods                                            # Isaac Sim
    python3 run_mission.py --sim --rows 2 --cols 3 [--goals frontier]

`run_mission.py --sim` runs the unchanged mission against a simulated Spot
(`spot_explorer/sim/robot.py`: fake SDK clients, real SDK messages, talking to the simulator
over a socket). What changed when it was brought onto this code (see CHANGELOG 26-11):

- **the whole trajectory** of a command is followed (`spot_explorer/sim/controller.py`,
  `PathGoal` / `go_to_path`, done on the office PC on 25-11 and taken from andrea-branch 033020c):
  every point in order and in time, as Spot does; before, the robot went straight to the last
  point, cutting the corners the cruise's paths go around;
- **failures as the SDK's errors** (`SimNotRecordingError`, `SimGraphNavError`), and the link to
  the simulator lost counts as a robot error (`robot/errors.py`): the mission catches only those;
- the layers the simulation does not have (intensity, no_step) are simply not saved.

Tests: `tests/test_sim_core.py` (no SDK, no simulator), `tools/sim_selftest.py [--sim HOST:PORT]`.

## Test plan for the first days in the simulator

Each run: note scene, seed, goals and variants (the `[CONFIG] SWITCHES` line has them),
keep the mission folder. Tools: `tools/mission_report.py` (outcome, `Coverage:`),
`varianti/tools/cruise_timing.py` (timing of the cruise), `tools/mission_viewer.py` (maps,
scans, path), `coverage.csv` (area seen over time). The alternatives live in the `varianti/` folder (GitHub branch `varianti`): a mission with
some of them on is `python3 varianti/run.py --with name,name -- --sim ...` (`--list` lists them).

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
scene: `cruise_state_cache`, `event_driven_planning`, `ground_from_grid` (only if the simulator
puts `gpe` in the grid snapshot; `spot_explorer/sim/` does not today), and `sleep_between_turns` (the old fixed sleep: the waking
on a new plan is in the main code since 24-11). Look at: plan read delay, plans read of those
published, main thread turn. The network
latency of a simulator is not the robot's: the order of the gains counts, not the absolute
numbers.

**4. Where the goal comes from**: cells against frontier, the mission option
`run_mission.py --sim --goals cells|frontier`, same scenes and
seeds. Expected with Spot's 3.84 m grid: as good as the cells in the open, slower among walls
(goals ~1 m apart). If the simulator can give a wider grid (a simulated lidar: more cells, the
code reads the size from the protos), run the frontier with it: there it should pull ahead.

**5. Terrain and gait** on the slope and on a step of 10-20 cm: gait levels chosen
(`[GAIT]`), edge slopes against the ground under the feet (`varianti/tools/slope_check.py`), no stop
in front of a step Spot can take.

**6. Write down** a table per run (scene, variant, cells, % seen, metres, minutes, cruise
speed, stops, failures) -- the comparison of 4 and 3 decides what to try next on the robot.

## What a simulator will not tell

Spot's own internals: how it builds and filters its grids (noise, grass seen as obstacle or
not), its body obstacle avoidance, the real GraphNav (localization, loop closure, drift).
Those stay for the robot.
