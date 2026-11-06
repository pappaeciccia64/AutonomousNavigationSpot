#!/usr/bin/env bash
# The code of this folder, as it is now (config.py included), to ~/ans/code on the CORE I/O.
# Only the code: spot_explorer/, tools/, tests/, run_mission.py. The mission data on the CORE
# (MissionLogs, MissionMap, graph) stay where they are; files deleted here are deleted there.
#     tools/core_deploy.sh
set -euo pipefail
source "$(dirname "$0")/core_common.sh"
cd "$REPO"
echo "== $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD)$(git diff --quiet || echo ' + uncommitted changes') -> $DEST:~/$CORE_DIR/code"
rsync -a --delete --info=stats1 -e "$RSYNC_SSH" \
    --exclude '__pycache__/' --exclude '*.pyc' \
    spot_explorer tools tests run_mission.py "$DEST:$CORE_DIR/code/"
"${SSH[@]}" "$DEST" "cd ~/$CORE_DIR/code && ~/$CORE_DIR/venv/bin/python tools/preflight.py | tail -3"
