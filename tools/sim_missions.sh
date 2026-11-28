#!/bin/bash
# Quick check of a change: real missions on the kinematic simulator (sim/kinematic_server.py,
# no physics, a few minutes each), before the validation in Isaac Sim. For each world and goal
# mode: one mission with run_mission.py --sim, then its outcome (end lines, tracebacks).
#
#   tools/sim_missions.sh                                  # boxes and woods, cells and frontier, 3 x 4
#   WORLDS="boxes forest_hills" GOALS=frontier tools/sim_missions.sh
#   ROWS=2 COLS=3 OUT=/some/dir tools/sim_missions.sh
#
# The mission logs go to $OUT (default: a new folder under /tmp). Exit status 1 if a mission
# failed or printed a traceback.
cd "$(dirname "$0")/.." || exit 1
WORLDS=${WORLDS:-"boxes woods"}
GOALS=${GOALS:-"cells frontier"}
ROWS=${ROWS:-3}
COLS=${COLS:-4}
PORT=${PORT:-8771}
OUT=${OUT:-$(mktemp -d /tmp/sim_missions_XXXX)}
mkdir -p "$OUT"
bad=0
for w in $WORLDS; do
  for g in $GOALS; do
    python3 sim/kinematic_server.py --world "sim/worlds/$w.json" --port "$PORT" > "$OUT/server_${g}_$w.log" 2>&1 &
    sp=$!
    sleep 3
    log="$OUT/mission_${g}_$w.log"
    timeout 900 python3 run_mission.py --sim "127.0.0.1:$PORT" --rows "$ROWS" --cols "$COLS" --goals "$g" > "$log" 2>&1
    rc=$?
    kill "$sp" 2>/dev/null; wait "$sp" 2>/dev/null
    tb=$(grep -ac Traceback "$log")
    echo "== $g $w: exit $rc, tracebacks $tb"
    grep -aE "\[FRONTIER\] End|\[PROGRESS\] End|\[COVERAGE\] End|\[EMERGENCY\]|blocked on the spot|^Cruises:" "$log" | cut -c1-160 | tail -5
    [ "$rc" -ne 0 ] || [ "$tb" -ne 0 ] && bad=1
  done
done
echo "Logs in $OUT"
exit $bad
