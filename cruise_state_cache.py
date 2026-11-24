"""
Cruise: one thread reads the robot state for everybody (state_cache.py here), and the readers
of the cruise (command feedback, paths sent, tilt recorder, fall check, ground height of the
planning thread) take its last reading, at most state_cache.MAX_AGE_S old, instead of a network
request each (attempt_cruise.CruiseSteps._cruise_state_client).
"""
from spot_explorer.mission import attempt_cruise
from hook import replace
import state_cache

DESCRIPTION = "cruise: one shared robot state reader instead of a request per reader"


def _cruise_state_client(self):
    return state_cache.shared(self.robot_state_client)


def apply():
    return replace(attempt_cruise.CruiseSteps, '_cruise_state_client', _cruise_state_client)
