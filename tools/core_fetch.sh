#!/usr/bin/env bash
# The mission data made on the CORE I/O (MissionLogs, MissionMap, graph) into this folder,
# next to those of the missions run from the laptop: mission_report.py and mission_viewer.py
# read them as usual. Nothing is deleted on either side.
#     tools/core_fetch.sh
set -euo pipefail
source "$(dirname "$0")/core_common.sh"
cd "$REPO"
for d in MissionLogs MissionMap graph; do
  rsync -a --info=stats1 -e "$RSYNC_SSH" "$DEST:$CORE_DIR/code/$d/" "$d/" 2>/dev/null || echo "($d: nothing on the CORE)"
done
rsync -a -e "$RSYNC_SSH" "$DEST:$CORE_DIR/code/mission_nohup.log" "MissionLogs/" 2>/dev/null || true
