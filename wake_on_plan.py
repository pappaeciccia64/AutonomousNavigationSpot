"""
Cruise: the main thread wakes as soon as the planning thread publishes a plan, instead of
sleeping CRUISE_POLL_S between two turns (attempt_cruise.CruiseSteps._wait_for_plan). The
feedback is still read at least every CRUISE_POLL_S. The [CRUISE] Plan lines say how long each
plan waited before it was read.
"""
from spot_explorer.mission import attempt_cruise
from hook import replace

DESCRIPTION = "cruise: main thread woken by every new plan (plans read sooner)"


def _wait_for_plan(self, tracker, last_version):
    tracker.wait_plan(last_version, attempt_cruise.CRUISE_POLL_S)


def apply():
    return replace(attempt_cruise.CruiseSteps, '_wait_for_plan', _wait_for_plan)
