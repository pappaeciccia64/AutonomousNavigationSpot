#!/usr/bin/env python3
"""
Map navigation with Spot: "go there" on an area already explored by a finished mission.

    python3 run_navigation.py [mission] --to X,Y | --cell R,C | --waypoint wp_N | --home | --interactive | --click
    python3 run_navigation.py [mission] --to X,Y --preview plan.png      # without the robot

The arguments are those of spot_explorer.navigation.run (python3 run_navigation.py --help).
"""
import sys

from spot_explorer.navigation.run import main

if __name__ == '__main__':
    sys.exit(0 if main() else 1)
