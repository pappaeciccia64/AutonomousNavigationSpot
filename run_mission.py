#!/usr/bin/env python3
"""
Lancia una missione di esplorazione con Spot.

    python3 run_mission.py [--rows R --cols C --cell L --grafo reticolo|casuale ...]

Gli argomenti sono quelli di spot_explorer.mission.run (python3 run_mission.py --help).
"""
from spot_explorer.mission.run import main

if __name__ == '__main__':
    main()
