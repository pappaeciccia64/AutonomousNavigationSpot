"""
A small GraphNav for the simulation: waypoints, edges, localization, route, navigation.

What it keeps of the real one, because the mission relies on it:
  - while recording, a waypoint is dropped by itself every AUTO_WAYPOINT_SPACING_M and every
    new waypoint is linked to the previous one (the "chain"); named waypoints (wp_N) are
    created on request, edges between existing waypoints too (the shortcuts);
  - the robot is localized on the nearest waypoint, and reports the body pose relative to it;
  - a navigation command follows the EDGES of the graph (shortest route), is renewed by the
    caller every half second, and expires if it is not.
What it does not have: scans, ICP, loop closure, the lost detector, obstacle avoidance. The
localization is exact (no drift), so the drift the mission measures here is always ~0.

Standard library only (poses are plain tuples); the SDK messages are built in robot.py.
"""
import heapq
import math
import threading
import time

from spot_explorer.sim import controller as ctl

AUTO_WAYPOINT_SPACING_M = 0.5
NAMED_WAYPOINT_BONUS_M = 0.10    # on a tie, localize on a waypoint created by the mission
VIA_TOL_M = 0.12                 # how close to pass by an intermediate waypoint
TURN_FIRST_RAD = math.radians(25.0)
# FIX 25-11: STUCK as Spot means it, no progress for a while, not a leg that
# takes longer than planned. It was a fixed time limit per leg (3 x the expected time + 10 s):
# with Isaac Sim slower than real time (real-time factor 0.4-0.75, see tools/sim_turn_check.py)
# the turns on the spot ran over it and the return to wp_0 ended STUCK 10 cm from the waypoint.
NO_PROGRESS_S = 6.0              # this long without moving or turning = STUCK
PROGRESS_M = 0.03                # progress: moved at least this much ...
PROGRESS_RAD = math.radians(3.0)  # ... or turned at least this much
LEG_MAX_S = 180.0                # safety limit of one leg, far beyond any normal leg

FOLLOWING, REACHED, NO_ROUTE, NO_LOCALIZATION, STUCK, TIMED_OUT, OVERRIDDEN = (
    'following', 'reached', 'no_route', 'no_localization', 'stuck', 'timed_out', 'overridden')


# --- poses: (x, y, z, (w, x, y, z)) ----------------------------------------------------------
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_rotate(q, v):
    w = q_mul(q_mul(q, (0.0, v[0], v[1], v[2])), q_conj(q))
    return (w[1], w[2], w[3])


def q_from_yaw(yaw):
    return (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))


def pose_inverse(p):
    qi = q_conj(p[3])
    x, y, z = q_rotate(qi, (-p[0], -p[1], -p[2]))
    return (x, y, z, qi)


def pose_mul(a, b):
    x, y, z = q_rotate(a[3], (b[0], b[1], b[2]))
    return (a[0] + x, a[1] + y, a[2] + z, q_mul(a[3], b[3]))


def pose_of_state(st):
    return (st['x'], st['y'], st['z'], tuple(st['q']))


class SimGraph:
    def __init__(self, controller):
        self._ctl = controller
        self._lock = threading.RLock()
        self.waypoints = {}          # id -> dict(id, name, pose, auto)
        self.adj = {}                # id -> {id: length}
        self.recording = False
        self.localized = False
        self._last_wp = None
        self._n_created = 0
        self._nav = None             # the navigation command in progress
        self._nav_id = 0
        controller.add_listener(self._on_state)

    # --- map -----------------------------------------------------------------------------
    def clear(self):
        with self._lock:
            self.waypoints, self.adj = {}, {}
            self._last_wp, self.localized = None, False

    def start_recording(self):
        with self._lock:
            self.recording = True
        # Like Spot: recording starts from a waypoint at the current position.
        self.create_waypoint(None, auto=True)

    def stop_recording(self):
        with self._lock:
            self.recording = False

    def create_waypoint(self, name, auto=False, pose=None):
        """New waypoint at the body pose, linked to the previous one. Returns its dict."""
        if pose is None:
            pose = pose_of_state(self._ctl.state())
        with self._lock:
            self._n_created += 1
            wid = 'sim-wp-%04d' % self._n_created
            wp = {'id': wid, 'name': name or 'waypoint_%d' % self._n_created, 'pose': pose,
                  'auto': bool(auto)}
            self.waypoints[wid] = wp
            self.adj.setdefault(wid, {})
            if self._last_wp in self.waypoints:
                self._link(self._last_wp, wid)
            self._last_wp = wid
            self.localized = True
            return wp

    def add_edge(self, a, b):
        with self._lock:
            if a not in self.waypoints or b not in self.waypoints:
                raise KeyError("edge between waypoints that are not in the map")
            self._link(a, b)

    def _link(self, a, b):
        pa, pb = self.waypoints[a]['pose'], self.waypoints[b]['pose']
        d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
        self.adj.setdefault(a, {})[b] = d
        self.adj.setdefault(b, {})[a] = d

    def edges(self):
        """[(from id, to id)] once per pair, in creation order of the first waypoint."""
        with self._lock:
            return [(a, b) for a in self.waypoints for b in self.adj.get(a, {}) if a < b]

    def _on_state(self, st):
        """Controller thread: drop the automatic waypoints of the chain while recording."""
        with self._lock:
            if not self.recording or self._last_wp not in self.waypoints:
                return
            last = self.waypoints[self._last_wp]['pose']
            if math.hypot(st['x'] - last[0], st['y'] - last[1]) < AUTO_WAYPOINT_SPACING_M:
                return
        self.create_waypoint(None, auto=True, pose=pose_of_state(st))

    # --- localization ----------------------------------------------------------------------
    def nearest(self, x, y):
        """Id of the waypoint nearest to (x, y), None if the map is empty."""
        with self._lock:
            best, best_d = None, float('inf')
            for wid, wp in self.waypoints.items():
                d = math.hypot(wp['pose'][0] - x, wp['pose'][1] - y)
                if not wp['auto']:
                    d -= NAMED_WAYPOINT_BONUS_M
                if d < best_d:
                    best, best_d = wid, d
            return best

    def set_localized(self):
        with self._lock:
            if not self.waypoints:
                raise RuntimeError("the map is empty: nothing to localize on")
            self.localized = True

    def localization(self):
        """(waypoint id, body pose relative to the waypoint), or (None, None)."""
        st = self._ctl.state()
        with self._lock:
            if not self.localized or not self.waypoints:
                return None, None
            wid = self.nearest(st['x'], st['y'])
            return wid, pose_mul(pose_inverse(self.waypoints[wid]['pose']), pose_of_state(st))

    # --- route -----------------------------------------------------------------------------
    def route(self, start, goal):
        """Shortest route along the edges: [ids] or None."""
        with self._lock:
            adj = {k: dict(v) for k, v in self.adj.items()}
        dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == goal:
                break
            if d > dist.get(u, float('inf')):
                continue
            for v, w in adj.get(u, {}).items():
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        if goal not in dist:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1]

    # --- navigation ------------------------------------------------------------------------
    def navigate(self, dest_id, duration_s, v_max, w_max, final_yaw_offset=None,
                 ignore_final_yaw=False, command_id=None):
        """
        Start (or renew) a navigation command to a waypoint. The command lives duration_s
        from the last call: the caller renews it with the id it got back. Returns the id.
        """
        with self._lock:
            nav = self._nav
            if (command_id is not None and nav is not None and nav['id'] == command_id
                    and nav['dest'] == dest_id and nav['status'] in (FOLLOWING, REACHED)):
                nav['deadline'] = time.time() + float(duration_s)
                return nav['id']
            if nav is not None and nav['status'] == FOLLOWING:
                nav['status'] = OVERRIDDEN
            self._nav_id += 1
            nav = {'id': self._nav_id, 'dest': dest_id, 'status': FOLLOWING,
                   'deadline': time.time() + float(duration_s), 'v': float(v_max),
                   'w': float(w_max), 'yaw_offset': final_yaw_offset,
                   'free_yaw': bool(ignore_final_yaw)}
            self._nav = nav
        threading.Thread(target=self._follow, args=(nav,), daemon=True,
                         name='sim-graphnav').start()
        return nav['id']

    def navigation_status(self, command_id):
        with self._lock:
            nav = self._nav
            if nav is None or nav['id'] != command_id:
                return OVERRIDDEN
            return nav['status']

    def _finish(self, nav, status):
        with self._lock:
            if nav['status'] == FOLLOWING:
                nav['status'] = status

    def _leg(self, nav, x, y, yaw, pos_tol, yaw_tol=ctl.YAW_TOL_RAD):
        """One controller goal of the navigation. True if reached; otherwise the navigation
        status is already set."""
        st = self._ctl.state()
        expected = (math.hypot(x - st['x'], y - st['y']) / nav['v']
                    + (abs(ctl.wrap(yaw - st['yaw'])) / nav['w'] if yaw is not None else 0.0))
        # FIX 25-11: the leg limit was 3 x expected + 10 s (STUCK when slow); now LEG_MAX_S, and STUCK below
        gid = self._ctl.go_to(x, y, yaw, nav['v'], nav['w'], max(3.0 * expected + 10.0, LEG_MAX_S),
                              pos_tol=pos_tol, yaw_tol=yaw_tol)
        mark, mark_t = (st['x'], st['y'], st['yaw']), time.time()   # last pose with progress
        while True:
            status = self._ctl.status(gid)
            now_st = self._ctl.state()
            if (math.hypot(now_st['x'] - mark[0], now_st['y'] - mark[1]) >= PROGRESS_M
                    or abs(ctl.wrap(now_st['yaw'] - mark[2])) >= PROGRESS_RAD):
                mark, mark_t = (now_st['x'], now_st['y'], now_st['yaw']), time.time()
            elif status == ctl.GOING and time.time() - mark_t > NO_PROGRESS_S:  # FIX 25-11: STUCK = no progress
                self._ctl.stop()                # pushing against something: blocked
                self._finish(nav, STUCK)
                return False
            with self._lock:
                alive = nav['status'] == FOLLOWING
                expired = time.time() > nav['deadline']
            if not alive:                       # replaced by another navigation command
                return False
            if status == ctl.AT_GOAL:
                return True
            if status == ctl.TIMED_OUT:
                self._finish(nav, STUCK)
                return False
            if status != ctl.GOING:             # a robot command took over (stop, a move)
                self._finish(nav, OVERRIDDEN)
                return False
            if expired:                         # the caller stopped renewing the command
                self._ctl.stop()
                self._finish(nav, TIMED_OUT)
                return False
            time.sleep(0.05)

    def _follow(self, nav):
        try:
            st = self._ctl.state()
            with self._lock:
                ok = self.localized and nav['dest'] in self.waypoints
                start = self.nearest(st['x'], st['y']) if ok else None
            if not ok or start is None:
                self._finish(nav, NO_LOCALIZATION if not self.localized else NO_ROUTE)
                return
            path = self.route(start, nav['dest'])
            if path is None:
                self._finish(nav, NO_ROUTE)
                return
            with self._lock:
                pts = [self.waypoints[w]['pose'] for w in path]
            # Already past the first waypoint, on the way to the second: do not walk back to it.
            if len(pts) >= 2 and (math.hypot(pts[1][0] - st['x'], pts[1][1] - st['y'])
                                  <= math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])):
                pts = pts[1:]
            for i, p in enumerate(pts):
                last = i == len(pts) - 1
                st = self._ctl.state()
                dist = math.hypot(p[0] - st['x'], p[1] - st['y'])
                if dist > (ctl.POSITION_TOL_M if last else VIA_TOL_M):
                    heading = math.atan2(p[1] - st['y'], p[0] - st['x'])
                    if abs(ctl.wrap(heading - st['yaw'])) > TURN_FIRST_RAD:
                        if not self._leg(nav, st['x'], st['y'], heading, pos_tol=0.25):
                            return
                    if not self._leg(nav, p[0], p[1], heading,
                                     pos_tol=ctl.POSITION_TOL_M if last else VIA_TOL_M,
                                     yaw_tol=math.radians(15.0)):
                        return
            if not nav['free_yaw']:
                dest = pts[-1]
                yaw = ctl.yaw_of(dest[3]) + (nav['yaw_offset'] or 0.0)
                if not self._leg(nav, dest[0], dest[1], ctl.wrap(yaw), pos_tol=0.10):
                    return
            self._finish(nav, REACHED)
        except Exception:
            self._finish(nav, STUCK)
