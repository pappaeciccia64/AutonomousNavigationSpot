#!/usr/bin/env python3
"""
The two error rules of spot_explorer/robot/errors.py, without the robot.

    python3 tests/test_errors.py

  1. ROBOT_ERRORS catches the SDK's errors (RpcError, a GraphNav response error, a frame-tree
     error) and not a bug (TypeError, KeyError);
  2. optional() prints one line with the error type and goes on;
  3. on a real caller: motion.PathCommand.poll counts a robot error as a communication
     error, while a bug inside the robot call is not hidden;
  4. photos.capture is an optional part: a failing camera costs the photos, not the mission;
  5. the planning thread: a robot error is retried, a bug is kept and raised again in the main
     thread (ArcVerificationTracker.raise_if_failed).
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

    print("== planning thread: a robot error is retried, a bug goes up to the main thread ==")
    import threading
    from spot_explorer.planning import safe_frontier
    tr = safe_frontier.ArcVerificationTracker.__new__(safe_frontier.ArcVerificationTracker)
    tr._lock = threading.Lock()
    tr._plan_cond = threading.Condition(tr._lock)
    tr._cycle_lock = threading.Lock()
    tr._plan, tr._plan_version, tr.error, tr._running = None, 0, None, True
    calls = {'n': 0}

    def robot_fails():
        calls['n'] += 1
        raise RpcError(None, "network")
    tr._job = robot_fails
    with contextlib.redirect_stdout(io.StringIO()):
        tr._planning_cycle(robot_fails, 0.0)
    check(tr.error is None and tr._job is robot_fails and tr.get_plan()['status'] == 'error',
          "a robot error: logged, a plan with status 'error', planning goes on")
    tr.raise_if_failed()

    def bug():
        return {}['missing']
    tr._job = bug
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        tr._planning_cycle(bug, 0.0)
    check(tr.error is not None and tr._job is None and not tr._running,
          "a bug: kept, the thread stops planning")
    try:
        tr.raise_if_failed()
        raised = None
    except KeyError as e:
        raised = e
    check(raised is not None and 'planning thread' in str(raised) and 'missing' in str(raised),
          "raise_if_failed re-raises it in the caller's thread, with the thread's traceback")

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
