#!/bin/bash
# Start Spot in Isaac Sim for `python3 run_mission.py --sim`.
#
#   sim/run_isaac.sh                      world sim/worlds/forest.json (plain trunks), with the window
#   sim/run_isaac.sh woods                the forest with real trees (needs --fetch-forest once)
#   sim/run_isaac.sh boxes --headless     any world of sim/worlds/ (or a path to a .json), then
#                                         any option of the simulator (see sim/README_SIM.md)
#   sim/run_isaac.sh --fetch-forest       download trees, shrubs, rocks and sky, once (600 MB)
#
# Isaac Sim is looked for in $ISAACSIM_PATH, default /data/Programs/isaacsim.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC="${ISAACSIM_PATH:-/data/Programs/isaacsim}"
if [ "$1" = "--fetch-forest" ]; then      # download the forest assets, once (about 600 MB)
    shift
    [ -x "$ISAAC/python.sh" ] || { echo "Isaac Sim not found in $ISAAC (set ISAACSIM_PATH)" >&2; exit 1; }
    (cd "$ISAAC" && ./python.sh "$HERE/fetch_forest_assets.py" "$@")
    exec python3 "$HERE/make_forest_world.py"
fi
WORLD="${1:-forest}"
[ $# -gt 0 ] && shift
[ -f "$WORLD" ] || WORLD="$HERE/worlds/${WORLD%.json}.json"
[ -f "$WORLD" ] || { echo "World file not found: $WORLD" >&2; exit 1; }
[ -x "$ISAAC/python.sh" ] || { echo "Isaac Sim not found in $ISAAC (set ISAACSIM_PATH)" >&2; exit 1; }
WORLD="$(realpath "$WORLD")"
cd "$ISAAC"
exec ./python.sh "$HERE/isaac_spot_server.py" --world "$WORLD" "$@"
