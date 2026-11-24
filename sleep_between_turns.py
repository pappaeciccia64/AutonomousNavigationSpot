"""
Cruise: the main thread sleeps CRUISE_POLL_S between two turns of the cruise loop instead of
waking as soon as the planning thread publishes a plan (attempt_cruise.CruiseSteps._wait_for_plan).
The behaviour before 24-11, kept to compare how fresh the plans are when read.
"""
import time

from spot_explorer.mission import attempt_cruise
from hook import replace

DESCRIPTION = "cruise: fixed sleep between turns instead of waking on every new plan"


def _wait_for_plan(self, tracker, last_version):
    time.sleep(attempt_cruise.CRUISE_POLL_S)


def apply():
    return replace(attempt_cruise.CruiseSteps, '_wait_for_plan', _wait_for_plan)
