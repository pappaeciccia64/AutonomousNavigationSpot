#!/usr/bin/env python3
"""
The two error rules of spot_explorer/robot/errors.py, without the robot.

    python3 tests/test_errors.py

  1. ROBOT_ERRORS catches the SDK's errors (RpcError, a GraphNav response error, a frame-tree
     error) and not a bug (TypeError, KeyError);
  2. optional() prints one line with the error type and goes on;
  3. on a real caller: motion.PathCommand.poll counts a robot error as a communication
     error, while a bug inside the robot call is not hidden;
  4. photos.capture is an optional part: a failing camera costs the photos, not the mission.
"""
import contextlib
import io
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE), HERE]

from bosdyn.client.exceptions import RpcError                       # noqa: E402
from bosdyn.client.frame_helpers import ValidateFrameTreeError      # noqa: E402
from bosdyn.client.graph_nav import GraphNavServiceResponseError    # noqa: E402

from fake_spot import FakeSpot                                      # noqa: E402
from spot_explorer.robot import motion, photos                      # noqa: E402
from spot_explorer.robot.errors import ROBOT_ERRORS, describe, optional  # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def caught(exc):
    try:
        raise exc
    except ROBOT_ERRORS:
        return True
    except Exception:
        return False


def main():
    print("== ROBOT_ERRORS ==")
    check(caught(RpcError(None, "net")), "an RpcError is a robot error")
    check(caught(GraphNavServiceResponseError(None, "lost")), "a GraphNav response error is a robot error")
    check(caught(ValidateFrameTreeError("bad tree")), "a frame-tree error is a robot error")
    check(not caught(TypeError("bug")) and not caught(KeyError("bug")), "a TypeError or KeyError is not")
    check(describe(RpcError(None, "net")).startswith("RpcError"), "describe() starts with the type")

    print("== optional() ==")
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        with optional("[TEST] part"):
            raise KeyError("x")
        went_on = True
    check(went_on and "[TEST] part: skipped after an error (KeyError" in out.getvalue(),
          f"one line with the type, then on ({out.getvalue().strip()})")

    print("== PathCommand.poll ==")
    spot = FakeSpot(0.0, 0.0)
    pc = motion.PathCommand(spot, spot)
    pc.send([(2.0, 0.0)], hold_s=None)
    spot.fail_next = 1
    with contextlib.redirect_stdout(io.StringIO()):
        st = pc.poll()['status']
    check(st == 'comm_error', f"a robot error is a communication error ({st})")
    real = spot.robot_command_feedback
    spot.robot_command_feedback = lambda *a, **k: (_ for _ in ()).throw(TypeError("bug in the feedback"))
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            pc.poll()
        hidden = True
    except TypeError:
        hidden = False
    spot.robot_command_feedback = real
    check(not hidden, "a bug in the robot call is not hidden")

    print("== photos.capture ==")
    class BrokenCameras:
        def get_image(self, *a, **k):
            raise RpcError(None, "camera service down")
    photos._STATE.update(client=BrokenCameras(), folder=tempfile.mkdtemp(prefix='test_errors_'))
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        files = photos.capture("test")
    photos._STATE.update(client=None, folder=None)
    check(files == [] and "[PHOTO] test: skipped after an error (RpcError" in out.getvalue(),
          "a failing camera: no photos, one line, the mission goes on")

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
