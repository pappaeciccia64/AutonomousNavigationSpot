#!/usr/bin/env bash
# Run on the CORE I/O, in ~/ans/code with the venv:
#     tools/core_run.sh bench          tools/bench_cruise.py (no robot): is the Jetson fast enough
#     tools/core_run.sh check          tools/core_check.py: connection to the robot, read only
#     tools/core_run.sh mission [args] run_mission.py --hostname 192.168.50.3 --no-estop [args],
#                                      in the background with nohup: it goes on if the WiFi of
#                                      the laptop drops. Log: ~/ans/code/mission_nohup.log
#     tools/core_run.sh log            follow that log (Ctrl+C stops following, not the mission)
#     tools/core_run.sh stop           stop the mission (SIGINT, as Ctrl+C: it closes as on the PC)
# Robot login: ~/ans/robot_credentials on the CORE (two lines, user and password, chmod 600),
# written once by  tools/core_run.sh credentials  (asked here, never on the command line).
set -euo pipefail
source "$(dirname "$0")/core_common.sh"
cmd="${1:-}"; shift || true
PY="~/$CORE_DIR/venv/bin/python"
ENVS="cd ~/$CORE_DIR/code && export SPOT_HOSTNAME=$ROBOT_FROM_CORE && \
{ [ -f ~/$CORE_DIR/robot_credentials ] && export BOSDYN_CLIENT_USERNAME=\$(sed -n 1p ~/$CORE_DIR/robot_credentials) \
BOSDYN_CLIENT_PASSWORD=\$(sed -n 2p ~/$CORE_DIR/robot_credentials) || true; }"
case "$cmd" in
  bench)   "${SSH[@]}" "$DEST" "$ENVS && $PY tools/bench_cruise.py $*" ;;
  check)   "${SSH[@]}" "$DEST" "$ENVS && $PY tools/core_check.py $*" ;;
  mission) "${SSH[@]}" "$DEST" "$ENVS && nohup $PY run_mission.py --hostname $ROBOT_FROM_CORE --no-estop $* \
               > mission_nohup.log 2>&1 < /dev/null & echo \"mission started, pid \$!\"" ;;
  log)     "${SSH[@]}" -t "$DEST" "tail -f ~/$CORE_DIR/code/mission_nohup.log" ;;
  stop)    "${SSH[@]}" "$DEST" "pkill -INT -f 'run_mission.py' && echo 'SIGINT sent' || echo 'no mission running'" ;;
  credentials)
           read -rp "robot user: " u; read -rsp "robot password: " p; echo
           printf '%s\n%s\n' "$u" "$p" | "${SSH[@]}" "$DEST" \
               "umask 077; mkdir -p ~/$CORE_DIR; cat > ~/$CORE_DIR/robot_credentials; echo saved" ;;
  *) sed -n '2,13p' "$0"; exit 1 ;;
esac
