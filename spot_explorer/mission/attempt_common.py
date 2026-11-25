"""Outcomes of a cell attempt's cruise and the battery checks it shares (see cell_attempt.py)."""
from spot_explorer.records import MISSION_CTX


# Outcomes of a cruise (attempt_cruise._cruise, read by CellAttempt.run).
CONTINUE = "continue"   # the cruise ended for an exception: a new cruise from here
STOP = "stop"           # the attempt ends: the final position decides (CellAttempt._finish)
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
