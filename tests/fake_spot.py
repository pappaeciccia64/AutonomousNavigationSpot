"""
Fake Spot driven by a simulated clock: command client and state client in one object,
with the real protos in and out, so the commands built by robot/motion.py are read the
way Spot reads them.

    spot = FakeSpot(x, y, yaw)
    pc = motion.PathCommand(spot, spot)
    pc.send(path); spot.advance(0.2); pc.poll() ...

What it does: follows the SE2 trajectory in time (linear interpolation between the points,
reference_time and end_time relative to when the command arrives), with the speed limits
of the mobility params as a cap; the stop command; the dead man (end_time); a new
command replacing the previous one; a point that cannot be entered (blocked(x, y) true):
the robot stays where it is and reports the final goal blocked. The pose in the
snapshot is vision -> body.
"""
import time

import numpy as np
from bosdyn.api import robot_command_pb2, robot_state_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus, SE2TrajectoryCommand
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.util import duration_to_seconds, timestamp_to_sec

_FB = SE2TrajectoryCommand.Feedback
AT_GOAL_M, AT_GOAL_RAD, NEAR_GOAL_M = 0.03, np.radians(3.0), 0.3


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


class FakeSpot:
    def __init__(self, x, y, yaw=0.0, blocked=None):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.now = 0.0                  # simulated seconds
        self.blocked = blocked          # callable (x, y) -> True if the point cannot be entered
        self.commands = []              # every command received: (sim time, RobotCommand, end_time_secs)
        self.fail_next = 0              # how many of the next feedback/state calls raise
        self.track = [(self.x, self.y)]
        self._cmds = {}                 # id -> dict of the command state
        self._active = None
        self._next_id = 1

    # ---- command client -------------------------------------------------------------
    def robot_command(self, command=None, end_time_secs=None, lease=None, **kw):
        local_now = time.time()
        self.commands.append((self.now, command, end_time_secs))
        cid = self._next_id
        self._next_id += 1
        if self._active is not None and self._cmds[self._active]['status'] == RobotCommandFeedbackStatus.STATUS_PROCESSING:
            self._cmds[self._active]['status'] = RobotCommandFeedbackStatus.STATUS_COMMAND_OVERRIDDEN
        mob = command.synchronized_command.mobility_command
        c = dict(status=RobotCommandFeedbackStatus.STATUS_PROCESSING, points=None,
                 traj_status=_FB.STATUS_GOING_TO_GOAL, final=_FB.FINAL_GOAL_STATUS_ACHIEVABLE,
                 end=None if end_time_secs is None else self.now + (end_time_secs - local_now))
        if mob.HasField('se2_trajectory_request'):
            traj = mob.se2_trajectory_request.trajectory
            t0 = self.now + (timestamp_to_sec(traj.reference_time) - local_now)
            c['points'] = [(p.pose.position.x, p.pose.position.y, p.pose.angle,
                            t0 + duration_to_seconds(p.time_since_reference)) for p in traj.points]
            params = spot_command_pb2.MobilityParams()
            mob.params.Unpack(params)
            c['v'] = params.vel_limit.max_vel.linear.x or 1.6
            c['w'] = params.vel_limit.max_vel.angular or 1.0
        else:                            # stop or anything else: stand still
            c['traj_status'] = _FB.STATUS_STOPPED
        self._cmds[cid] = c
        self._active = cid
        return cid

    def robot_command_feedback(self, cid):
        self._maybe_fail()
        c = self._cmds[cid]
        r = robot_command_pb2.RobotCommandFeedbackResponse()
        m = r.feedback.synchronized_feedback.mobility_command_feedback
        m.status = c['status']
        m.se2_trajectory_feedback.status = c['traj_status']
        m.se2_trajectory_feedback.final_goal_status = c['final']
        m.se2_trajectory_feedback.body_movement_status = (
            _FB.BODY_STATUS_SETTLED if c['traj_status'] in (_FB.STATUS_AT_GOAL, _FB.STATUS_STOPPED)
            else _FB.BODY_STATUS_MOVING)
        return r

    # ---- state client ---------------------------------------------------------------
    def get_robot_state(self):
        self._maybe_fail()
        s = robot_state_pb2.RobotState()
        edges = s.kinematic_state.transforms_snapshot.child_to_parent_edge_map
        edges['vision'].parent_frame_name = ''
        edges['vision'].parent_tform_child.rotation.w = 1.0
        body = edges['body']
        body.parent_frame_name = 'vision'
        body.parent_tform_child.position.x, body.parent_tform_child.position.y = self.x, self.y
        body.parent_tform_child.rotation.w = np.cos(self.yaw / 2)
        body.parent_tform_child.rotation.z = np.sin(self.yaw / 2)
        return s

    def _maybe_fail(self):
        if self.fail_next > 0:
            self.fail_next -= 1
            raise RuntimeError("fake network error")

    # ---- simulation -----------------------------------------------------------------
    def advance(self, dt, step=0.02):
        """Simulated time goes on by dt; the robot follows the active trajectory."""
        end = self.now + dt
        while self.now < end - 1e-9:
            h = min(step, end - self.now)
            self.now += h
            self._step(h)

    def _step(self, h):
        c = self._cmds.get(self._active)
        if c is None or c['points'] is None or c['status'] != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            return
        if c['end'] is not None and self.now > c['end']:
            c['status'] = RobotCommandFeedbackStatus.STATUS_COMMAND_TIMED_OUT
            c['traj_status'] = _FB.STATUS_STOPPED
            return
        rx, ry, rh = self._reference(c['points'])
        dx, dy = rx - self.x, ry - self.y
        d = float(np.hypot(dx, dy))
        k = min(1.0, c['v'] * h / d) if d > 1e-9 else 0.0
        nx, ny = self.x + k * dx, self.y + k * dy
        if self.blocked is not None and self.blocked(nx, ny):
            c['final'] = _FB.FINAL_GOAL_STATUS_BLOCKED
            return
        self.x, self.y = nx, ny
        dyaw = _wrap(rh - self.yaw)
        self.yaw = _wrap(self.yaw + float(np.clip(dyaw, -c['w'] * h, c['w'] * h)))
        self.track.append((self.x, self.y))
        gx, gy, gh, _ = c['points'][-1]
        dist_goal = float(np.hypot(gx - self.x, gy - self.y))
        if dist_goal < AT_GOAL_M and abs(_wrap(gh - self.yaw)) < AT_GOAL_RAD:
            c['traj_status'] = _FB.STATUS_AT_GOAL
        elif dist_goal < NEAR_GOAL_M:
            c['traj_status'] = _FB.STATUS_NEAR_GOAL
        else:
            c['traj_status'] = _FB.STATUS_GOING_TO_GOAL

    def _reference(self, points):
        """Pose of the trajectory at the current simulated time."""
        if self.now <= points[0][3]:
            return points[0][:3]
        for a, b in zip(points, points[1:]):
            if self.now <= b[3]:
                f = (self.now - a[3]) / max(b[3] - a[3], 1e-9)
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
                        a[2] + f * _wrap(b[2] - a[2]))
        return points[-1][:3]
