"""Outcomes of the phases of a cell attempt and the battery checks they share (see cell_attempt.py)."""
from spot_explorer.records import MISSION_CTX


# Outcomes of a phase of the navigation loop (see CellAttempt._loop_once): None = go on
# with the next phase.
CONTINUE = "continue"   # turn finished: start again from the scan
STOP = "stop"           # end of the loop: the final position decides (CellAttempt._finish)
FAILED = "failed"       # the attempt failed

# A cell seen for at least this share in the global map (cells.cell_seen_fraction) counts as
# explored: at the attempt start (CellAttempt._cell_already_seen), in cruise (attempt_cruise),
# before moving (exploration.Mission._close_seen_cells).
CELL_SEEN_FRACTION = 0.90


def battery_return_due(where=None):
    """One battery reading (see battery.BatteryMonitor.check): True if the mission must stop
    exploring and return. False if there is no monitor."""
    monitor = MISSION_CTX.get('battery')
    return monitor.check(where) if monitor is not None else False


def battery_low():
    """Like battery_return_due but without reading: only if the return has already been decided."""
    monitor = MISSION_CTX.get('battery')
    return monitor is not None and monitor.return_due
