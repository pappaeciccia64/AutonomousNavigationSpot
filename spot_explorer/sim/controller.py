"""
From a pose goal to velocity commands: the part of Spot that, on the real robot, takes an
SE2 trajectory and walks it -- one point (go_to) or several in order and in time
(go_to_path, the path command of the continuous motion).

The simulated robot only understands a base velocity (vx, vy, yaw rate) in the body frame:
that is the input of the locomotion policy in Isaac Sim. A thread here reads the pose about
25 times per second, and while a goal is active sends the velocity that closes the error,
within the limits of the command. Spot walks in every direction, so the same law does the
rotation in place, the straight leg and the walk without turning.

Standard library and numpy only.
"""
import math
import os
import threading
import time

from spot_explorer.sim.link import SimLinkError

RATE_HZ = 25.0
KP_LINEAR = 1.5            # 1/s
KP_ANGULAR = 2.0           # 1/s
# Below these the locomotion policy barely moves the robot: outside the tolerance the
# command never goes under them, so the last centimetres do not take forever.
MIN_LINEAR_MPS = 0.12
MIN_ANGULAR_RPS = 0.20
POSITION_TOL_M = 0.04
YAW_TOL_RAD = math.radians(2.0)
SETTLE_S = 0.4             # inside the tolerance for this long = arrived and settled
STATE_MAX_AGE_S = 1.0      # older than this, the pose is not trusted
# After the arrival the robot keeps the goal pose, as Spot does: the walking policy of Isaac Sim
# creeps and turns slowly when left with a zero command. Gentle limits, no minimum speed.
HOLD_MAX_LINEAR_MPS = 0.3
HOLD_MAX_ANGULAR_RPS = 0.5

# ADD 25-11: multi-point trajectories (the path command of the continuous motion): the robot follows the
# poses in order and in time, as Spot does, instead of walking straight to the last one.
PATH_LOOKAHEAD_M = 0.30    # the target never runs further than this ahead of the robot's
                           # projection on the path: a robot behind schedule catches up along
                           # the path, it does not cut the corners
PATH_SEARCH_SEGMENTS = 3   # the projection is searched only a few segments ahead (hairpins)
# The mission sends the path again every ~0.4 s, each time starting from the robot with the
# clock at zero: the robot, which kept walking while the command was built, is always a few
# centimetres "ahead" of the new schedule. Stopping for that made the cruise crawl in Isaac Sim
# (25-11: 0.06-0.08 m/s instead of 0.4). Up to this much ahead it walks at the planned speed.
PATH_AHEAD_TOL_M = 0.25  # FIX 25-11: was wait when ahead: cruise crawled at 0.07 m/s
# FIX 26-11: a turn on the spot at a node (a sharp corner, a U-turn) starts only once the robot
# is at the node, as on Spot, which reaches the node in time before turning. The target used to
# run up to PATH_LOOKAHEAD_M ahead with the heading of the trajectory "for now": with a U-turn
# 20-30 cm ahead the robot started turning towards the way back before getting there, the next
# plan (every 0.4 s) asked forward again, and it spun on the spot for minutes (26-11 17:46,
# frontier goals on the slope: path [19, 361, 1] with the U-turn at node 19).
PATH_NODE_TOL_M = 0.06   # the robot is "at the node" within this along the path

GOING, AT_GOAL, TIMED_OUT, OVERRIDDEN, STOPPED = 'going', 'at_goal', 'timed_out', 'overridden', 'stopped'


def yaw_of(q):
    """Yaw of a quaternion given as (w, x, y, z)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Goal:
    def __init__(self, x, y, yaw, v_max, w_max, deadline, pos_tol=POSITION_TOL_M,
                 yaw_tol=YAW_TOL_RAD):
        self.x, self.y, self.yaw = float(x), float(y), (None if yaw is None else float(yaw))
        self.v_max, self.w_max = abs(float(v_max)) or 0.1, abs(float(w_max)) or 0.1
        self.deadline = float(deadline)
        self.pos_tol, self.yaw_tol = float(pos_tol), float(yaw_tol)
        self.status = GOING
        self.inside_since = None


class PathGoal(Goal):  # ADD 25-11: a multi-point trajectory followed in order and in time
    """A multi-point SE2 trajectory: points (x, y, yaw, t) with t in time.time() seconds.
    The final point is the goal of the base class (arrival and settling as for one point);
    until the last time, the target is the pose the trajectory gives for now, kept on the
    path and at most PATH_LOOKAHEAD_M ahead of the robot."""

    def __init__(self, points, v_max, w_max, deadline, pos_tol=POSITION_TOL_M,
                 yaw_tol=YAW_TOL_RAD):
        last = points[-1]
        super().__init__(last[0], last[1], last[2], v_max, w_max, deadline, pos_tol, yaw_tol)
        self.pts = [(float(x), float(y), float(h), float(t)) for x, y, h, t in points]
        s = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.s = s                   # arc length at every point (equal for turns on the spot)
        self.seg = 0                 # segment of the last projection of the robot

    def s_at_time(self, t):
        """Arc length the trajectory has reached at time t, and the index k of the point
        pair (k, k+1) it is between."""
        pts = self.pts
        if t <= pts[0][3]:
            return 0.0, 0
        for k in range(len(pts) - 1):
            t0, t1 = pts[k][3], pts[k + 1][3]
            if t <= t1:
                f = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
                return self.s[k] + f * (self.s[k + 1] - self.s[k]), k
        return self.s[-1], len(pts) - 2

    def project(self, x, y):
        """Arc length of the robot's projection on the path, searched from the last segment."""
        best, best_s, best_k = float('inf'), self.s[self.seg], self.seg
        for k in range(self.seg, min(self.seg + PATH_SEARCH_SEGMENTS, len(self.pts) - 1)):
            ax, ay = self.pts[k][:2]
            bx, by = self.pts[k + 1][:2]
            L = self.s[k + 1] - self.s[k]
            f = 0.0 if L < 1e-9 else max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (L * L)))
            d = math.hypot(x - ax - f * (bx - ax), y - ay - f * (by - ay))
            if d < best - 1e-9:
                best, best_s, best_k = d, self.s[k] + f * L, k
        self.seg = best_k
        return best_s

    def pose_at_s(self, s_target, k_hint=0):
        """(x, y, yaw) on the path at arc length s_target; the yaw of the segment it is on."""
        pts, S = self.pts, self.s
        k = max(0, min(k_hint, len(pts) - 2))
        while k < len(pts) - 2 and S[k + 1] < s_target - 1e-9:
            k += 1
        while k > 0 and S[k] > s_target + 1e-9:
            k -= 1
        L = S[k + 1] - S[k]
        f = 0.0 if L < 1e-9 else max(0.0, min(1.0, (s_target - S[k]) / L))
        a, b = pts[k], pts[k + 1]
        return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), b[2]


class BaseController:
    def __init__(self, link, rate_hz=RATE_HZ):
        self._link = link
        self._period = 1.0 / float(rate_hz)
        self._lock = threading.Lock()
        self._goals = {}
        self._active = None          # id of the goal being followed
        self._hold = None            # (x, y, yaw) kept after an arrival, until the next command
        self._next_id = 1
        self._state = None
        self._state_wall = 0.0
        self._error = None
        self._listeners = []
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name='sim-controller')
        self._thread.start()

    # --- state ---------------------------------------------------------------------------
    def state(self):
        """Latest body pose: dict x, y, z, q (w, x, y, z), yaw, t. Raises SimLinkError if the
        simulator stopped answering."""
        t_end = time.time() + 5.0
        while True:
            with self._lock:
                st, age, err = self._state, time.time() - self._state_wall, self._error
            if st is not None and age <= STATE_MAX_AGE_S:
                return st
            if err is not None and (st is not None or time.time() > t_end):
                raise SimLinkError("no state from the simulator: %s" % (err,))
            if time.time() > t_end:
                raise SimLinkError("no state from the simulator in 5 s")
            time.sleep(0.02)

    def add_listener(self, fn):
        """fn(state) is called at every new pose (from the controller thread)."""
        self._listeners.append(fn)

    # --- goals ---------------------------------------------------------------------------
    def go_to(self, x, y, yaw, v_max, w_max, timeout_s, pos_tol=POSITION_TOL_M,
              yaw_tol=YAW_TOL_RAD):
        """Start walking to the pose; the previous goal, if any, is overridden. Returns an id
        for status()."""
        return self._start(Goal(x, y, yaw, v_max, w_max, time.time() + float(timeout_s),
                                pos_tol, yaw_tol))

    def go_to_path(self, points, v_max, w_max, timeout_s, pos_tol=POSITION_TOL_M,  # ADD 25-11: follow a multi-point trajectory
                   yaw_tol=YAW_TOL_RAD):
        """Start following a multi-point trajectory: points (x, y, yaw, t), t in time.time()
        seconds (see PathGoal). The previous goal, if any, is overridden. Returns an id."""
        if len(points) < 2:
            x, y, yaw, _ = points[-1]
            return self.go_to(x, y, yaw, v_max, w_max, timeout_s, pos_tol, yaw_tol)
        return self._start(PathGoal(points, v_max, w_max, time.time() + float(timeout_s),
                                    pos_tol, yaw_tol))

    def _start(self, goal):  # ADD 25-11: shared by go_to and go_to_path (was inside go_to)
        with self._lock:
            self._override_active()
            gid = self._next_id
            self._next_id += 1
            self._goals[gid] = goal
            self._active = gid
            for old in [k for k in self._goals if k < gid - 50]:
                del self._goals[old]
        return gid

    def stop(self):
        """Stop the robot now (the equivalent of Spot's stop command)."""
        with self._lock:
            self._override_active(STOPPED)

    def status(self, gid):
        with self._lock:
            goal = self._goals.get(gid)
            return OVERRIDDEN if goal is None else goal.status

    def wait(self, gid, poll_s=0.05):
        """Block until the goal is no longer being followed; returns its final status."""
        while True:
            st = self.status(gid)
            if st != GOING:
                return st
            time.sleep(poll_s)

    def close(self):
        self._running = False

    def _override_active(self, status=OVERRIDDEN):
        self._hold = None
        if self._active is not None:
            goal = self._goals.get(self._active)
            if goal is not None and goal.status == GOING:
                goal.status = status
        self._active = None

    # ADD 26-11: the side of a turn of nearly 180 deg is kept, as on Spot, which goes on turning
    # the way it started. Chosen from scratch at every sample, with the goal behind the robot the
    # shortest side flipped at every new command (the path is resent every 0.4 s and its heading
    # moves by a few degrees): +0.6 / -0.6 rad/s in turn and the robot turned back and forth for
    # minutes (26-11 17:58, frontier goals on the slope, SPOT_SIM_TRACE).
    TURN_KEEP_SIDE_RAD = math.radians(150.0)   # beyond this error the side of the last turn is kept
    TURN_SIDE_MEMORY_S = 1.0                   # ... if that turn was this recent

    def _turn_error(self, target, yaw):  # ADD 26-11
        """Heading error target - yaw in (-pi, pi], on the side of the last turn when nearly 180 deg."""
        e = wrap(target - yaw)
        last = getattr(self, '_turn_side', None)
        now = time.time()
        if (abs(e) > self.TURN_KEEP_SIDE_RAD and last is not None and now - last[1] <= self.TURN_SIDE_MEMORY_S
                and math.copysign(1.0, e) != last[0]):
            e += last[0] * 2.0 * math.pi                       # same angle, taken the other way round
        if abs(e) > math.radians(2.0):
            self._turn_side = (math.copysign(1.0, e), now)
        return e

    # --- loop ----------------------------------------------------------------------------
    def _velocity(self, goal, st, now):
        """Body velocity towards the goal, and whether the robot is inside the tolerance."""
        ex, ey = goal.x - st['x'], goal.y - st['y']
        dist = math.hypot(ex, ey)
        eyaw = 0.0 if goal.yaw is None else self._turn_error(goal.yaw, st['yaw'])  # FIX 26-11: was wrap(): side flipped near 180 deg
        # Hysteresis: once inside, the robot may wobble a little without "leaving".
        k = 1.5 if goal.inside_since is not None else 1.0
        pos_ok = dist <= goal.pos_tol * k
        yaw_ok = abs(eyaw) <= goal.yaw_tol * k
        if pos_ok and yaw_ok:
            return (0.0, 0.0, 0.0), True
        vx = vy = wz = 0.0
        if not pos_ok:
            speed = min(goal.v_max, max(min(MIN_LINEAR_MPS, goal.v_max), KP_LINEAR * dist))
            c, s = math.cos(st['yaw']), math.sin(st['yaw'])
            vx = speed * (c * ex + s * ey) / dist
            vy = speed * (-s * ex + c * ey) / dist
        if not yaw_ok:
            rate = min(goal.w_max, max(min(MIN_ANGULAR_RPS, goal.w_max), KP_ANGULAR * abs(eyaw)))
            wz = math.copysign(rate, eyaw)
        return (vx, vy, wz), False

    def _path_velocity(self, goal, st, now):  # ADD 25-11: velocity along a PathGoal
        """Body velocity along a PathGoal: feed-forward along the path at the speed of the
        trajectory, plus the correction towards the pose the trajectory gives for now (kept
        on the path, at most PATH_LOOKAHEAD_M ahead of the robot). After the last time, or
        once the target is the last point, the arrival is the one of a single goal."""
        s_time, k = goal.s_at_time(now)
        s_proj = goal.project(st['x'], st['y'])
        s_end = goal.s[-1]
        if now >= goal.pts[-1][3] or min(max(s_time, s_proj), s_proj + PATH_LOOKAHEAD_M) >= s_end - 1e-6:
            return self._velocity(goal, st, now)
        s_target = min(max(s_time, s_proj), s_proj + PATH_LOOKAHEAD_M)
        # FIX 26-11: the next turn on the spot (a zero-length step of the trajectory) after the
        # stretch the robot is on: the target stops there and keeps the heading of the approach
        # until the robot has reached it; then the heading follows the trajectory in time.
        S, j = goal.s, goal.seg + 1
        while j < len(S) - 1 and S[j + 1] - S[j] > 1e-6:
            j += 1
        if j < len(S) - 1 and s_proj < S[j] - PATH_NODE_TOL_M:
            s_target = min(s_target, S[j])
            tx, ty, th = goal.pose_at_s(s_target, goal.seg)      # was pose_at_s(s_target, k): turned early
        else:
            tx, ty, th = goal.pose_at_s(s_target, k)
            if goal.s[k + 1] - goal.s[k] <= 1e-6:   # FIX 26-11: in time for the turn on the spot: its heading, at its node
                tx, ty, th = goal.pts[k + 1][0], goal.pts[k + 1][1], goal.pts[k + 1][2]
        # speed of the trajectory on the stretch being walked (0 on a turn on the spot)
        a, b = goal.pts[k], goal.pts[k + 1]
        seg_v = (goal.s[k + 1] - goal.s[k]) / max(b[3] - a[3], 1e-3)
        ahead = s_proj - s_time
        if ahead > PATH_AHEAD_TOL_M:              # well ahead of schedule: slow down, then wait
            v_ff = min(goal.v_max, seg_v) * max(0.0, 1.0 - (ahead - PATH_AHEAD_TOL_M) / PATH_AHEAD_TOL_M)
        elif s_time > s_proj + PATH_LOOKAHEAD_M:  # behind: catch up along the path
            v_ff = goal.v_max
        else:
            v_ff = min(goal.v_max, seg_v)
        goal.dbg = (s_proj, s_time, s_target, th)   # ADD 26-11: for SPOT_SIM_TRACE
        nx, ny, _ = goal.pose_at_s(min(s_end, s_target + 0.05), k)
        dx, dy = nx - tx, ny - ty
        dn = math.hypot(dx, dy)
        ux, uy = (dx / dn, dy / dn) if dn > 1e-9 else (0.0, 0.0)
        wx = v_ff * ux + KP_LINEAR * (tx - st['x'])
        wy = v_ff * uy + KP_LINEAR * (ty - st['y'])
        sp = math.hypot(wx, wy)
        if sp > goal.v_max:
            wx, wy = wx * goal.v_max / sp, wy * goal.v_max / sp
        c, s = math.cos(st['yaw']), math.sin(st['yaw'])
        vx, vy = c * wx + s * wy, -s * wx + c * wy
        wz = max(-goal.w_max, min(goal.w_max, KP_ANGULAR * self._turn_error(th, st['yaw'])))  # FIX 26-11: was wrap(): side flipped near 180 deg
        return (vx, vy, wz), False

    def _hold_velocity(self, st):
        """Body velocity that keeps the robot on the pose of the last arrival."""
        x, y, yaw = self._hold
        ex, ey = x - st['x'], y - st['y']
        c, s = math.cos(st['yaw']), math.sin(st['yaw'])
        vx, vy = KP_LINEAR * (c * ex + s * ey), KP_LINEAR * (-s * ex + c * ey)
        speed = math.hypot(vx, vy)
        if speed > HOLD_MAX_LINEAR_MPS:
            vx, vy = vx * HOLD_MAX_LINEAR_MPS / speed, vy * HOLD_MAX_LINEAR_MPS / speed
        wz = 0.0 if yaw is None else KP_ANGULAR * wrap(yaw - st['yaw'])
        return (vx, vy, max(-HOLD_MAX_ANGULAR_RPS, min(HOLD_MAX_ANGULAR_RPS, wz)))

    # ADD 26-11: trace of what the simulated robot is told, 25 times per second, to a CSV file when
    # the environment variable SPOT_SIM_TRACE names one (SPOT_SIM_TRACE=/tmp/trace.csv python3
    # run_mission.py --sim ...). For the spinning on the spot of 26-11 17:46. Off otherwise.
    def _trace(self, st, goal, cmd):
        path = os.environ.get('SPOT_SIM_TRACE')
        if not path:
            return
        try:
            f = getattr(self, '_trace_file', None)
            if f is None:
                f = self._trace_file = open(path, 'a', buffering=1)
                f.write("wall,sim_t,x,y,yaw_deg,goal,kind,s_proj,s_time,s_target,target_heading_deg,"
                        "goal_x,goal_y,cmd_vx,cmd_vy,cmd_wz,hold\n")
            kind = 'none' if goal is None else ('path' if isinstance(goal, PathGoal) else 'point')
            dbg = getattr(goal, 'dbg', None) if isinstance(goal, PathGoal) else None
            s_proj, s_time, s_target, th = dbg if dbg else (float('nan'),) * 4
            f.write(f"{time.time():.3f},{st.get('t', 0.0):.3f},{st['x']:.3f},{st['y']:.3f},{math.degrees(st['yaw']):.1f},"
                    f"{self._active},{kind},{s_proj:.3f},{s_time:.3f},{s_target:.3f},{math.degrees(th):.1f},"
                    f"{goal.x if goal else float('nan'):.3f},{goal.y if goal else float('nan'):.3f},"
                    f"{cmd[0]:.3f},{cmd[1]:.3f},{cmd[2]:.3f},{int(self._hold is not None)}\n")
        except Exception:
            pass

    def _loop(self):
        cmd = (0.0, 0.0, 0.0)
        while self._running:
            t0 = time.time()
            try:
                raw = self._link.step(cmd)
                q = tuple(raw['q'])
                st = {'x': raw['p'][0], 'y': raw['p'][1], 'z': raw['p'][2], 'q': q,
                      'yaw': yaw_of(q), 't': raw.get('t', 0.0),
                      # what the simulator knows besides the pose (velocity, joints, IMU)
                      'extra': {k: raw[k] for k in ('v', 'w', 'jp', 'jv', 'imu') if k in raw}}
                err = None
            except Exception as e:
                st, err = None, e
            cmd = (0.0, 0.0, 0.0)
            with self._lock:
                self._error = err
                if st is not None:
                    self._state, self._state_wall = st, time.time()
                    goal = self._goals.get(self._active) if self._active is not None else None
                    if goal is not None and goal.status == GOING:
                        now = time.time()
                        if now > goal.deadline:
                            goal.status, self._active = TIMED_OUT, None
                        else:
                            cmd, inside = (self._path_velocity(goal, st, now)
                                           if isinstance(goal, PathGoal)  # FIX 25-11: multi-point paths followed point by point
                                           else self._velocity(goal, st, now))
                            if not inside:
                                goal.inside_since = None
                            elif goal.inside_since is None:
                                goal.inside_since = now
                            elif now - goal.inside_since >= SETTLE_S:
                                goal.status, self._active = AT_GOAL, None
                                self._hold = (goal.x, goal.y, goal.yaw)
                    elif self._hold is not None:
                        cmd = self._hold_velocity(st)
                    self._trace(st, goal, cmd)   # ADD 26-11: SPOT_SIM_TRACE (off unless asked)
            if st is not None:
                for fn in list(self._listeners):
                    try:
                        fn(st)
                    except Exception:
                        pass
            if err is not None:
                time.sleep(0.2)
            dt = time.time() - t0
            if dt < self._period:
                time.sleep(self._period - dt)
