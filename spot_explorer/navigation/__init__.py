"""
Map navigation: "go there" on an area already explored by a finished mission.

  frames        the mission frame and the transform between the mission's VISION frame and
                today's
  mission_map   reading a finished mission (log, global maps, GraphNav graph) and planning on
                its PRM, without the robot
  goal_picker   choosing the goal with a click on the map
  navigator     the robot side: GraphNav graph upload, localization, following the plan with
                the safe frontier, map updates, battery, return home, logs
  run           command line (run_navigation.py)
"""
