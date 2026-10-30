#!/usr/bin/env python3
"""
Run an exploration mission with Spot.

    python3 run_mission.py [--rows R --cols C --cell L --graph lattice|random ...]

The arguments are those of spot_explorer.mission.run (python3 run_mission.py --help).
"""
from spot_explorer.mission.run import main

if __name__ == '__main__':
    main()
