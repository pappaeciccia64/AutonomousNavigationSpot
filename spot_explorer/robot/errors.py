"""
The two rules for errors, used everywhere in the package.

1. A request to the robot can fail (network, robot busy, GraphNav lost, ...): catch only
   ROBOT_ERRORS, the errors of the Boston Dynamics SDK. Every SDK error derives from
   bosdyn.client.exceptions.Error (RpcError, ResponseError, the GraphNav and recording
   errors, TimedOutError), plus the frame-tree errors of bosdyn.client.frame_helpers
   (a transform snapshot from the robot that cannot be read). Anything else is a bug in
   our code and is not hidden.

       try:
           state = robot_state_client.get_robot_state()
       except ROBOT_ERRORS as e:
           print(f"[BATTERY] Reading failed: {describe(e)}")

2. A part that the mission can do without (photos, files, statistics, figures): wrap it in
   optional(). Any error there, bug included, is printed in one clear line with its type and
   the mission goes on.

       with optional("[PHOTO] front camera"):
           save_photo(...)

In the parts that decide the motion a bug is not caught: it reaches the emergency procedure
of mission/run.py (robot stopped and sat down, map saved).
"""
import contextlib

from bosdyn.client.exceptions import Error as _SdkError
from bosdyn.client.frame_helpers import Error as _FrameError

# The SDK's own errors, and a frame tree from the robot that cannot be read.
ROBOT_ERRORS = (_SdkError, _FrameError)


def describe(e):
    """'TypeName: message' -- the type tells a robot error from a bug at a glance."""
    return f"{type(e).__name__}: {e}"


@contextlib.contextmanager
def optional(what):
    """Run an optional part: on any error print one line and go on (see the module docstring)."""
    try:
        yield
    except Exception as e:                  # optional part: on purpose, any error
        print(f"{what}: skipped after an error ({describe(e)}).")
