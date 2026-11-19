"""
One thread that reads the robot state for everybody (attempt_cruise.CRUISE_STATE_CACHE).

During the cruise the robot state was asked ~15 times a second, each caller waiting for its
own network round trip: the main thread twice per turn (command feedback, then the pose),
again at every path sent, the body tilt recorder every 0.2 s, the planning thread at every
scan (ground height), the fall check every second. RobotStateCache has the same
get_robot_state() as the client: one thread reads the state every PERIOD_S and the callers
get the last reading if it is younger than MAX_AGE_S, without waiting for the network.

Never an old state: if the last reading is older than MAX_AGE_S the caller waits for the next
one (at most MAX_AGE_S), then asks Spot itself. Lazy: the thread reads only while somebody
asks (a request in the last IDLE_S), so in the stop-and-go and during GraphNav trips it is
quiet; a request wakes it. After stop() every request goes straight to Spot.
"""
import threading
import time

PERIOD_S = 0.05      # one reading every 50 ms while in use (20 Hz)
MAX_AGE_S = 0.12     # a reading older than this is not served
IDLE_S = 2.0         # no request for this long: the thread stops reading


class RobotStateCache:
    def __init__(self, client, period_s=PERIOD_S, max_age_s=MAX_AGE_S, idle_s=IDLE_S):
        self._client = client
        self.period_s, self.max_age_s, self.idle_s = period_s, max_age_s, idle_s
        self._cond = threading.Condition()
        self._state, self._t = None, 0.0          # last reading and when it arrived
        self._seq = 0
        self._asked_t = 0.0                       # last request of a caller
        self._running = True
        self.readings = self.served = self.waited = self.direct = self.errors = 0
        self.ages = []                            # age of the readings served (s), for the stats
        self._thread = threading.Thread(target=self._loop, name="robot_state_cache", daemon=True)
        self._thread.start()

    def __getattr__(self, name):
        # Everything else of the client (other requests) goes to the client.
        return getattr(self._client, name)

    def _loop(self):
        while True:
            with self._cond:
                while self._running and time.time() - self._asked_t > self.idle_s:
                    self._cond.wait()
                if not self._running:
                    return
            t0 = time.time()
            try:
                st = self._client.get_robot_state(timeout=1.0)
            except Exception:
                with self._cond:
                    self.errors += 1
                time.sleep(self.period_s)
                continue
            with self._cond:
                self._state, self._t = st, time.time()
                self._seq += 1
                self.readings += 1
                self._cond.notify_all()
            time.sleep(max(0.0, self.period_s - (time.time() - t0)))

    def get_robot_state(self, timeout=None, **kw):
        """As RobotStateClient.get_robot_state: the last reading if younger than max_age_s,
        else the next one (waited at most max_age_s), else a request of its own."""
        with self._cond:
            now = time.time()
            if not self._running:
                st = None
            else:
                idle = now - self._asked_t > self.idle_s
                self._asked_t = now
                if idle:
                    self._cond.notify_all()             # wake the thread
                st = self._state if self._state is not None and now - self._t <= self.max_age_s else None
                if st is None and not idle:
                    seq = self._seq
                    if self._cond.wait_for(lambda: self._seq > seq, timeout=self.max_age_s):
                        st = self._state
                        self.waited += 1
            if st is not None:
                self.served += 1
                self.ages.append(time.time() - self._t)
                return st
            self.direct += 1
        return (self._client.get_robot_state(**kw) if timeout is None
                else self._client.get_robot_state(timeout=timeout, **kw))

    def stop(self):
        """From now on every request goes to Spot; returns when the reading in flight (if any)
        has ended."""
        with self._cond:
            self._running = False
            self._cond.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)

    def summary(self):
        """One line for the log."""
        import numpy as np
        a = np.array(self.ages) if self.ages else np.zeros(1)
        n = self.served + self.direct
        return (f"robot state: {self.readings} readings, {n} requests served "
                f"({self.served} from the cache, {self.waited} of them after waiting the next reading; "
                f"{self.direct} asked to Spot), age of the state served p50 {1000 * np.median(a):.0f} ms "
                f"p90 {1000 * np.percentile(a, 90):.0f} ms, {self.errors} reading errors")


_SHARED = {}


def shared(client):
    """The one cache of this client for the whole mission (created at the first call)."""
    c = _SHARED.get(id(client))
    if c is None:
        c = _SHARED[id(client)] = RobotStateCache(client)
    return c
