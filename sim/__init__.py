"""
Bridge between the mission and a simulator (see SIMULATOR.md).

    backend.py         SimBackend: the few things a simulator must provide (pose, terrain and
                       obstacles around the robot, following a path, teleport)
    kinematic_world.py KinematicWorld: a reference backend without physics (boxes, slopes,
                       hills, steps), the robot follows the path exactly
    spot_adapter.py    what the mission expects from Spot, built from a backend: the local
                       grids as real SDK protos, commands / feedback / robot state, an
                       in-memory GraphNav, the robot object
    worlds.py          ready-made KinematicWorld scenes

The mission code is untouched: tools/run_sim_mission.py hands it a SimRobot instead of the
robot. A new simulator (Isaac Sim, ...) needs only a SimBackend subclass.
"""
