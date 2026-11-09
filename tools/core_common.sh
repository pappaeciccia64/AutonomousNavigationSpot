# Shared settings of the core_*.sh scripts (run from the laptop, on the robot's WiFi).
# Change them with environment variables, e.g.  CORE_HOST=192.168.50.5 tools/core_deploy.sh
CORE_HOST="${CORE_HOST:-192.168.80.3}"     # robot WiFi; the robot forwards port 20022 to the CORE I/O
CORE_PORT="${CORE_PORT:-20022}"
CORE_USER="${CORE_USER:-spot}"
CORE_DIR="${CORE_DIR:-ans}"                 # on the CORE, under /home/spot: code/, venv/, wheels/
ROBOT_FROM_CORE="${ROBOT_FROM_CORE:-192.168.50.3}"   # the robot as seen from the CORE I/O

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SSH=(ssh -p "$CORE_PORT" -o ControlMaster=auto -o ControlPath=/tmp/core-ssh-%r@%h:%p -o ControlPersist=60)
DEST="$CORE_USER@$CORE_HOST"
RSYNC_SSH="ssh -p $CORE_PORT -o ControlMaster=auto -o ControlPath=/tmp/core-ssh-%r@%h:%p -o ControlPersist=60"
# The shared SSH connection is opened here, detached from this script's output: if the first
# command of a script opened it, it would keep that output open for the 60 s of ControlPersist
# and whoever reads it (a pipe, $(...)) would wait that long, as if the script hung.
"${SSH[@]}" -O check "$DEST" >/dev/null 2>&1 || "${SSH[@]}" -o ControlMaster=yes -fN "$DEST" </dev/null >/dev/null 2>&1
