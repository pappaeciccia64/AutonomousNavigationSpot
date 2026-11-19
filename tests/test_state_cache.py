#!/usr/bin/env python3
"""
robot/state_cache.RobotStateCache without the robot: a fake state client that answers in
LATENCY_S with a state carrying the time it was read.

    python3 tests/test_state_cache.py

Checks: never a state older than MAX_AGE_S (plus the reading itself); the callers wait less
than a request of their own; the thread stops reading when nobody asks and wakes at the next
request; after stop() and when the client fails, the requests go straight to the client.
"""
import os
import sys
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np                                   # noqa: E402
from spot_explorer.robot import state_cache          # noqa: E402

LATENCY_S = 0.03
FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


class FakeClient:
    def __init__(self, fail=False):
        self.calls, self.fail = 0, fail
        self._l = threading.Lock()

    def get_robot_state(self, timeout=None):
        time.sleep(LATENCY_S)
        with self._l:
            self.calls += 1
        if self.fail:
            raise RuntimeError("no answer")
        return SimpleNamespace(read_at=time.time() - LATENCY_S / 2)   # the robot's instant

    def other_request(self):
        return 'other'


def consumers(cache, n_threads, period, seconds):
    """n_threads callers each asking every period: (ages of the states got, wait per request)."""
    ages, waits, l = [], [], threading.Lock()

    def run():
        t_end = time.time() + seconds
        while time.time() < t_end:
            t0 = time.time()
            st = cache.get_robot_state(timeout=2.0)
            t1 = time.time()
            with l:
                ages.append(t1 - st.read_at)
                waits.append(t1 - t0)
            time.sleep(period)
    th = [threading.Thread(target=run) for _ in range(n_threads)]
    [t.start() for t in th]
    [t.join() for t in th]
    return np.array(ages), np.array(waits)


def main():
    print("== five callers, as in the cruise (main 0.2 s, sends, tilt 0.2 s, planner, fall) ==")
    client = FakeClient()
    cache = state_cache.RobotStateCache(client)
    ages, waits = consumers(cache, 5, 0.1, 3.0)
    lim = state_cache.MAX_AGE_S + LATENCY_S
    check(ages.max() <= lim + 0.01, f"never a state older than {1000 * lim:.0f} ms "
                                    f"(max {1000 * ages.max():.0f} ms, p50 {1000 * np.median(ages):.0f})")
    check(np.median(waits) < LATENCY_S / 3, f"the callers wait {1000 * np.median(waits):.1f} ms (p50) "
                                            f"instead of {1000 * LATENCY_S:.0f} ms of a request")
    n_req = len(waits)
    check(client.calls < n_req, f"requests to Spot {client.calls} for {n_req} readings asked")
    print("  " + cache.summary())

    print("== nobody asks: the thread stops; a request wakes it ==")
    time.sleep(state_cache.IDLE_S + 0.3)
    c0 = client.calls
    time.sleep(0.5)
    check(client.calls == c0, f"no reading while idle ({client.calls - c0})")
    t0 = time.time()
    st = cache.get_robot_state()
    check(st is not None and time.time() - t0 < 2 * LATENCY_S + 0.02, "the first request after the idle answers at once")
    time.sleep(0.3)
    check(client.calls > c0 + 2, f"the thread reads again ({client.calls - c0} readings)")
    check(cache.other_request() == 'other', "other requests go to the client")

    print("== after stop(): straight to the client ==")
    cache.stop()
    c0 = client.calls
    cache.get_robot_state()
    check(client.calls == c0 + 1, "one request to the client")

    print("== the client fails: the error reaches the caller, no old state ==")
    bad = FakeClient(fail=True)
    c2 = state_cache.RobotStateCache(bad)
    try:
        c2.get_robot_state()
        check(False, "an exception expected")
    except RuntimeError:
        check(True, "the client's exception reaches the caller")
    time.sleep(0.3)
    check(c2.errors > 0, f"reading errors counted ({c2.errors})")
    c2.stop()

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
