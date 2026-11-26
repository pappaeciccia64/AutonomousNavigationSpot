# Running the mission in simulation

`python3 run_mission.py --sim` runs the same mission code against a simulated Spot instead of
the robot. Nothing above the SDK changes: perception, graph, frontier, decisions and logs are
the code that runs in the field.

```
run_mission.py --sim  ->  spot_explorer/sim/robot.py  ->  socket  ->  simulator
(mission's Python,        fake SDK clients, real           :8765     sim/isaac_spot_server.py  (Isaac Sim)
 with bosdyn)             SDK messages                               sim/kinematic_server.py   (no physics)
```

## Two simulators, same protocol

| | `sim/isaac_spot_server.py` | `sim/kinematic_server.py` |
|---|---|---|
| Runs with | Isaac Sim's Python (`./python.sh`), RTX GPU | any Python 3 with numpy, any computer |
| Motion | Spot walking with NVIDIA's flat-terrain policy, physics, collisions, real slopes | the commanded velocity integrated; the body follows the ground and tilts with it; a solid stops the body |
| Use it for | seeing the robot do the mission, timing, how it copes with real walking | mission logic, quick checks, the laptop |

## Start

Terminal 1, the simulator:

```bash
# Isaac Sim (in /data/Programs/isaacsim, or where ISAACSIM_PATH says)
sim/run_isaac.sh --fetch-forest     # once: download trees, shrubs, rocks and sky (600 MB)
sim/run_isaac.sh woods              # the forest, with the window following the robot
sim/run_isaac.sh forest_hills --headless   # any world of sim/worlds/, no window
# wait for:  [SIM] Ready: waiting for the mission on 127.0.0.1:8765

# or, without Isaac Sim
python3 sim/kinematic_server.py --world sim/worlds/boxes.json
```

`run_isaac.sh` passes what follows the world to `isaac_spot_server.py`:

| Option | |
|---|---|
| `--headless` | no window: faster; the viewers still show everything |
| `--sensors cameras,lidar,imu` / `none` | which sensors to simulate (default all; `none` is the fastest) |
| `--camera-hz 5`, `--camera-size 320x240` | camera frames per simulated second, image size |
| `--show-lidar` | the lidar points drawn in the window |
| `--view chase` / `free` | the window follows the robot (default) or the camera is yours |
| `--render-hz 25` | frames drawn per simulated second; lower if the real-time factor drops |
| `--device cpu` / `cuda` | physics device (default cpu: with one robot it is faster than the GPU) |
| `--port 8765` | |

The simulator checks the port before its long start: if another one is still running it
says so at once (close it with `pkill -f isaac_spot_server.py`).

Terminal 2, the mission (the Python environment that has bosdyn):

```bash
python3 run_mission.py --sim --rows 2 --cols 3          # 127.0.0.1:8765
python3 run_mission.py --sim 192.168.1.20:8765 ...      # simulator on another computer
python3 tools/live_viewer.py                            # terminal 3, as in the field
python3 tools/sim_viewer.py                             # terminal 4: truth, LiDAR, planner choices
python3 tools/sim_sensor_viewer.py                      # cameras, depth, lidar, IMU (Isaac Sim)
```

Where the goals come from: `--goals cells` (default) or `--goals frontier`, to compare the two
(see the main README). Start small (`--rows 2 --cols 3`): the default grid is 16 x 14 m. Restart the simulator
before every mission, so the robot is back at the origin.

## Checks

```bash
python3 tests/test_sim_core.py          # no SDK, no simulator needed: world, link, controller, GraphNav
python3 tools/sim_selftest.py           # needs bosdyn: the fake SDK clients through the mission's own functions
python3 tools/sim_selftest.py --sim     # the same against the simulator already running (Isaac Sim),
                                        # plus cameras, lidar, IMU, joint states
```

Run `sim_selftest.py` once on every new computer, and once against Isaac Sim before the first
simulated mission.

## Worlds

A world file lists the ground and the solids on it, in the simulator's world frame, which is the
mission's VISION frame: the robot starts at the origin facing +x, so **x is ahead and y is to
the left**, like the mission cells (columns ahead, rows to the left).

```json
{"name": "example", "ground_z": 0.0,
 "shapes": [{"type": "box", "x": 3.0, "y": 0.5, "sx": 1.0, "sy": 0.4, "yaw_deg": 20, "h": 0.8},
            {"type": "box", "x0": 5, "x1": 6, "y0": -1, "y1": 1},
            {"type": "cylinder", "x": 2.0, "y": -1.5, "r": 0.15, "h": 2.0}],
 "battery": {"start_pct": 100, "drain_pct_per_min": 0}}
```

`battery` is optional: a drain (for example 5 per minute) is the way to try the early return
on low battery. The simulator builds ground and solids and serves the file to the mission, so
the two always agree.

| World | What is in it |
|---|---|
| `empty.json`, `boxes.json`, `forest.json` | flat ground; nothing, three boxes, trunks |
| `slope.json` | a constant 12% grade uphill ahead (6.8 deg), two trunks: up, down and across a slope |
| `forest_hills.json` | hills up to ~12 deg, a mound, a hollow, a 5% grade, trunks and a log |
| `woods.json` | a wood with real trees, shrubs, boulders, fallen logs, grass and leaves (Isaac Sim draws them from the downloaded assets; every other simulator sees the same obstacles as plain solids) |

### The forest (`woods`)

`sim/run_isaac.sh --fetch-forest` downloads once from NVIDIA's asset library (into
`sim/assets/forest/`, git-ignored) about 30 trees, shrubs, rocks and leaves, the forest
floor material and a sky, and measures each asset: how wide it is at the robot's height.
`sim/make_forest_world.py` then plants a wood from that catalog:

```bash
python3 sim/make_forest_world.py                                   # sim/worlds/woods.json, seed 7
python3 sim/make_forest_world.py --seed 3 --trees 60 --tree-gap 1.2 --name thicket \
    --out sim/worlds/thicket.json                                  # denser, another layout
```

Every tree, shrub and boulder is also a shape of the world file, a cylinder as wide as what
the asset has below the robot's height (for a tree, the trunk; only trees whose crown starts
above the robot are planted): so the mission's grid, the solids the robot bumps into and what
cameras and lidar see agree. Grass and leaves are only looks: the robot walks through them.
A world file can have hills and a forest together (`terrain` and assets in the same file).
### Terrain

Add a `terrain` block to get hills instead of flat ground:

```json
"terrain": {"max_slope_deg": 7, "wavelength_m": 9, "seed": 4,
            "ramp": {"dir_deg": 0, "grade_pct": 5},
            "bumps": [{"x": 6.5, "y": 4.5, "h": 0.4, "r": 3.5}],
            "extent_m": [-15, 45, -30, 30]}
```

- `max_slope_deg`, `wavelength_m`, `seed`: rolling hills, never steeper than that; the
  wavelength is the distance between crests (longer = higher hills at the same slope).
- `ramp`: a constant grade; `dir_deg` 0 = uphill ahead, 90 = uphill to the left.
- `bumps`: single mounds (`h` > 0) or hollows (`h` < 0); their slope adds to the hills'.
- `extent_m`: the area Isaac Sim builds as a mesh (x min, x max, y min, y max). Outside it
  there is no ground: keep the mission inside.

The ground is always flat at height 0 within 1.2 m of the origin, so the robot starts level.
The height is one analytic function (`World.height`): the mesh in Isaac Sim, the terrain layer
of the local grid, the LiDAR and the viewer all use it, so they agree to the millimetre.

**The locomotion policy was trained on flat ground.** It is the only Spot policy that ships
with Isaac Sim. Start with `slope.json` and watch: if the robot stumbles, lower the grade.
Walking on steeper or rough ground needs a policy trained for it in Isaac Lab (rough-terrain
training), which is a separate piece of work.

## What is simulated, and how

| Mission side | In simulation |
|---|---|
| Robot pose (VISION, odom, body, ground plane) | the simulator's body pose; odom = VISION, **no drift** |
| Trajectory point, stop, stand, sit | a controller (`sim/controller.py`) turns the goal into base velocity, with the speed limits and the deadline of the command; stand and sit only change the reported state (the policy always stands) |
| Gait, step height, friction hint, foot obstacle avoidance | accepted and ignored: one policy, one gait |
| `obstacle_distance` | exact signed distance from the solids of the world file: no occlusion, the robot "sees" behind obstacles and all around |
| `terrain`, `terrain_valid` | the height of the world's ground with ~4 mm of noise, every cell valid: slopes yes (with a `terrain` block), **no rough terrain, no ghost cells** |
| Standing still | NVIDIA's policy turns and creeps slowly with a zero command: the pose is held in closed loop (by the controller after an arrival, by the simulator otherwise), within a few cm and degrees |
| Cameras | Isaac Sim only, not used by the mission: the five body cameras (frontleft, frontright, left, right, back), colour and depth, rendered by RTX; through the SDK's `ImageClient` with the real source names (`frontleft_fisheye_image`, `frontleft_depth_in_visual_frame`, ...). Wide angle but NOT fisheye, pictures upright |
| LiDAR | Isaac Sim only, not used by the mission: an RTX Ouster OS1 (32 beams, 10 Hz) on the back, through the SDK's `PointCloudClient` as `velodyne-point-cloud`; it sees the real tree geometry, leaves included. `tools/sim_viewer.py` also shows an ideal VLP-16 cast against the world file |
| IMU | Isaac Sim only: specific force and angular rate in the body frame, 100 Hz, through `robot_state_client.get_imu_state()` (the packets since the previous call) |
| Body velocity, joints | in the robot state: `velocity_of_body_in_vision` and the 12 joint states with the SDK names (`fl.hx` ...) |
| GraphNav recording | a waypoint every 0.5 m while recording, linked in a chain; `wp_N` and edges on request |
| GraphNav localization | on the nearest waypoint, exact |
| GraphNav navigation | shortest route along the edges, walked waypoint by waypoint; **no obstacle avoidance**, never lost |
| Loop closure, anchoring | answer "nothing to do" |
| Fiducials | none: the mission starts without, and skips the correction of the return |
| Battery | 100%, or the drain set in the world file |
| Falls | detected from the real body tilt in Isaac Sim; the self-right is NOT simulated, so a fall ends the mission |
| Lease, E-Stop, login, time sync | absent |

So the simulation exercises planning, frontier, decisions, recovery, GraphNav trips and logs
on a world with obstacles and, with a `terrain` block, slopes: the mission's slope costs and
slope checks then have something to work on. It says nothing about grass, roughness, camera
noise or drift: those still need the field.

Time: the mission measures its time limits on the clock. The simulator never runs faster
than the clock and counts its time on physics steps; if it runs slower (it prints its
real-time factor every 10 s), moves may time out. Measured on this computer: about 0.94 in
an empty world, 0.8-0.85 in the woods headless with all sensors, lower with the window;
`--sensors none`, `--headless` and `--render-hz 15` help. A 2 x 3 mission completed in the
woods at 0.8.

## Seeing what happens: `tools/sim_viewer.py`

One window, two panels, refreshed twice a second. It reads the true pose and the world from
the simulator (as a second client: it never commands the robot) and the mission's latest PRM
view from `MissionMap/`.

- **Left, the simulated world as it really is**: ground height with contour lines, obstacles,
  the robot and where it has been, the latest LiDAR scan, and the square of Spot's local grid:
  the only area the mission perceives at that moment. With Isaac Sim the scan is the
  simulator's own lidar (`--ideal-lidar` casts the ideal VLP-16 instead): ground hits coloured
  by distance from the robot, so the rings move with it; red = something between 30 cm and
  1 m; green = crowns. The title gives the scan number and its age: a new number at every
  refresh means the lidar is live.
- **Right, the planner**: the PRM with every edge coloured by its cost per metre, the path the
  mission chose (thick blue) and, dashed, the shortest path by distance between the same two
  points. The title gives length and cost of both: the difference is what the cost function
  (slope, roughness) decided.

`tools/live_viewer.py` stays the view of what the mission believes (its map, frontier,
decisions, log); this one adds the truth to compare it with. Keys: `f` follow the robot,
`l` LiDAR on/off, space pause, `q` quit. `--png file.png` writes one figure and exits.

## The sensors: `tools/sim_sensor_viewer.py`

With Isaac Sim, one more window shows what the robot's sensors see, live, with or without a
mission (it only reads, like the other viewers):

- the five cameras in colour, and below each its depth picture;
- the lidar from above in the world frame, every point coloured by its height, with the
  obstacles of the world file as outlines (trunks and outlines must coincide) and the path;
- the IMU over the last 20 s, and pose, speed, simulated time, real-time factor.

`--png file.png` writes one figure and exits. The same data, as SDK messages, from your own
code:

```python
from bosdyn.client.image import ImageClient
from bosdyn.client.point_cloud import PointCloudClient
images = robot.ensure_client(ImageClient.default_service_name)
shots = images.get_image_from_sources(['frontleft_fisheye_image', 'frontleft_depth_in_visual_frame'])
cloud = robot.ensure_client(PointCloudClient.default_service_name).get_point_cloud_from_sources(
    ['velodyne-point-cloud'])[0].point_cloud           # float32 x, y, z in the sensor frame
imu = robot_state_client.get_imu_state()               # ImuState, 100 packets per second
```

## LiDAR and SLAM

The LiDAR in the viewer is computed by casting rays against the world file from the true
sensor pose (`spot_explorer/sim/lidar.py`, 16 rings, tilted with the body). It is an ideal
sensor at one instant: no noise unless asked, no intensity, no motion distortion, no leaves.
It shows what a Velodyne would see.

The RTX lidar of Isaac Sim (above) is closer to a real one: it sees the drawn trees, leaves
included, one full turn at a time. Together with the 100 Hz IMU it is a start for trying a
LiDAR-inertial SLAM, with two limits: the turn comes without per-point timestamps, and the
IMU rate is below the hundreds of Hz such methods prefer.

## CPU and GPU

- Rendering (window, cameras, lidar) is on the GPU. Physics is on the CPU by default
  (`--device cpu`): with a single robot it is faster than `--device cuda` (0.82 against 0.56
  real-time factor, measured).
- `--headless` on `isaac_spot_server.py` drops the 3D window: less GPU and CPU, and the two
  viewers still show what happens.
- The mission, the fake SDK and the viewers are light numpy code: a fraction of a core each.
- The computer's CPU governor was on `powersave` (Isaac Sim warns at start): `performance`
  helps the real-time factor.

## Files

| Path | What it does |
|---|---|
| `sim/run_isaac.sh` | starts the simulator with Isaac Sim's Python; `--fetch-forest` downloads the forest |
| `sim/isaac_spot_server.py` | Isaac Sim side: NVIDIA's `spot_standalone.py` with a socket instead of the scripted commands, pose hold, chase view |
| `sim/isaac_scene.py` | ground (flat, forest floor or hills), lights, solids, trees and decor from the world file |
| `sim/isaac_sensors.py` | cameras, lidar and IMU on the body |
| `sim/fetch_forest_assets.py`, `sim/make_forest_world.py` | download and measure the forest assets; plant a wood |
| `tools/sim_sensor_viewer.py` | cameras, depth, lidar and IMU in one window |
| `sim/kinematic_server.py` | the simulator without physics |
| `spot_explorer/sim/world.py` | terrain height, shapes, signed distance, the local grid window, ray casting |
| `spot_explorer/sim/lidar.py` | the simulated VLP-16 |
| `tools/sim_viewer.py` | truth, LiDAR and planner choices in one window |
| `spot_explorer/sim/server.py`, `link.py` | the socket protocol (one JSON per line), both ends |
| `spot_explorer/sim/controller.py` | pose goal -> velocity command |
| `spot_explorer/sim/graph.py` | the small GraphNav |
| `spot_explorer/sim/robot.py` | `SimRobot` and the fake SDK clients (the only file here that needs bosdyn) |

Changes to the mission code, each marked `FIXME (simulation)`: `spot_explorer/robot/connection.py`
(`setLogInfo` returns the simulated robot with `--sim`; `SimpleEstop` does nothing on it) and
`spot_explorer/mission/run.py` (`--sim` argument, no lease on the simulated robot). Without
`--sim` the mission behaves as before.
