"""
Simulation back end: runs the mission against a simulated Spot instead of the real robot.

    world.py       obstacles of the simulated world and the synthetic local grid
    link.py        socket client towards the simulator (Isaac Sim or the kinematic one)
    server.py      socket server used by the two simulators (sim/isaac_spot_server.py,
                   sim/kinematic_server.py)
    controller.py  pose goal -> velocity command (what Spot's own controller does)
    graph.py       a small GraphNav: waypoints, edges, route, localization
    robot.py       SimRobot and the fake SDK clients (the only module that needs bosdyn)

This package must stay importable WITHOUT bosdyn (except robot.py): the simulators import
world.py and server.py from Isaac Sim's own Python. See sim/README_SIM.md.
"""
