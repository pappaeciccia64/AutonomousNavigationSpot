"""
What the mission expects from Spot, built from a SimBackend (see SIMULATOR.md).

  grid_responses   the local grids as real bosdyn.api LocalGridResponse protos (128 x 128
                   cells of 3 cm around the robot): the mission decodes them with its own code
                   (perception/spot_grid.py), as on the robot
  SimSpot          command client and state client: SE2 trajectories of robot/motion.py in,
                   feedback and robot state (VISION -> body, gpe, power, battery) out
  MemoryGraphNav   GraphNav, recording and map processing on an in-memory graph; its trips
                   teleport the robot (tests/test_mission.FakeGraphNav, on the backend)
  SimRobot         the robot object: ensure_client() gives the clients above

The mission runs in real time: the simulator clock is advanced to the wall clock at every
request (SimSpot.sync), so a backend with its own clock only has to keep up.
"""
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
from bosdyn.api import local_grid_pb2, robot_command_pb2, robot_state_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus, SE2TrajectoryCommand
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.api import world_object_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.util import duration_to_seconds, timestamp_to_sec
from scipy.ndimage import distance_transform_edt

GRID_CELLS = 128             # cells per side of the local grid, as Spot
GRID_CELL_M = 0.03           # cell size, m, as Spot
OD_MAX_M = 4.0               # obstacle_distance clipped here (int16 with 1 mm steps)
HEIGHT_SCALE = 0.001         # terrain quantization, m
GRID_FRAME = 'grid'
_FB = SE2TrajectoryCommand.Feedback
_ST = RobotCommandFeedbackStatus


def _quat(roll, pitch, yaw):
    """Quaternion (w, x, y, z) of roll about x, pitch about y, yaw about z (z-y-x)."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def _edge(snapshot, child, parent, x=0.0, y=0.0, z=0.0, q=(1.0, 0.0, 0.0, 0.0)):
    e = snapshot.child_to_parent_edge_map[child]
    e.parent_frame_name = parent
    p = e.parent_tform_child
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z = (float(v) for v in q)


def robot_snapshot(snapshot, backend):
    """vision (root), odom (= vision), body and gpe (the ground plane under the robot)."""
    x, y, z, yaw, roll, pitch = backend.pose()
    _edge(snapshot, 'vision', '')
    _edge(snapshot, 'odom', 'vision')
    _edge(snapshot, 'body', 'vision', x, y, z, _quat(roll, pitch, yaw))
    gz = float(np.asarray(backend.terrain(np.array([x]), np.array([y])))[0])
    _edge(snapshot, 'gpe', 'vision', x, y, gz if np.isfinite(gz) else z - 0.5, _quat(roll, pitch, yaw))


def _encode(lg, values, fmt, scale, offset):
    g = lg.local_grid
    g.cell_format = fmt
    g.encoding = local_grid_pb2.LocalGrid.ENCODING_RAW
    g.cell_value_scale, g.cell_value_offset = scale, offset
    g.data = values.tobytes()


def grid_responses(backend, names, wall_time=None):
    """The requested layers around the robot as LocalGridResponse protos (same snapshot):
    obstacle_distance, terrain, terrain_valid, and intensity / no_step if asked. Cell k is
    row k // 128 (y), column k % 128 (x), from the corner of the grid frame."""
    n, cs = GRID_CELLS, GRID_CELL_M
    rx, ry = backend.pose()[:2]
    ox = np.floor(rx / cs) * cs - n * cs / 2.0
    oy = np.floor(ry / cs) * cs - n * cs / 2.0
    xs = ox + (np.arange(n) + 0.5) * cs
    ys = oy + (np.arange(n) + 0.5) * cs
    X, Y = np.meshgrid(xs, ys)
    seen = np.asarray(backend.seen(X, Y, (rx, ry)), dtype=bool)
    h = np.asarray(backend.terrain(X, Y), dtype=float)
    valid = seen & np.isfinite(h)
    # obstacle_distance on a window padded by OD_MAX_M: obstacles just outside count too
    pad = int(np.ceil(OD_MAX_M / cs))
    pxs = ox + (np.arange(-pad, n + pad) + 0.5) * cs
    pys = oy + (np.arange(-pad, n + pad) + 0.5) * cs
    PX, PY = np.meshgrid(pxs, pys)
    obst = np.asarray(backend.obstacles(PX, PY), dtype=bool)
    if obst.any():
        od = np.where(obst, -distance_transform_edt(obst), distance_transform_edt(~obst)) * cs
    else:
        od = np.full(obst.shape, OD_MAX_M)
    od = np.clip(od[pad:pad + n, pad:pad + n], -OD_MAX_M, OD_MAX_M)

    ground = float(np.nanmedian(h[valid])) if valid.any() else 0.0
    offset = round(ground, 2)
    wall = time.time() if wall_time is None else wall_time
    out = []
    for name in names:
        lg = local_grid_pb2.LocalGridResponse(local_grid_type_name=name,
                                              status=local_grid_pb2.LocalGridResponse.STATUS_OK)
        g = lg.local_grid
        g.local_grid_type_name = name
        g.extent.cell_size, g.extent.num_cells_x, g.extent.num_cells_y = cs, n, n
        g.frame_name_local_grid_data = GRID_FRAME
        g.acquisition_time.seconds, g.acquisition_time.nanos = int(wall), int((wall % 1) * 1e9)
        robot_snapshot(g.transforms_snapshot, backend)
        _edge(g.transforms_snapshot, GRID_FRAME, 'vision', ox, oy, 0.0)
        if name == 'obstacle_distance':
            _encode(lg, np.round(od.ravel() / 0.001).astype(np.int16), local_grid_pb2.LocalGrid.CELL_FORMAT_INT16, 0.001, 0.0)
        elif name == 'terrain':
            raw = np.where(valid, np.round((np.nan_to_num(h) - offset) / HEIGHT_SCALE), 0).clip(-32767, 32767)
            _encode(lg, raw.astype(np.int16).ravel(), local_grid_pb2.LocalGrid.CELL_FORMAT_INT16, HEIGHT_SCALE, offset)
        elif name == 'terrain_valid':
            _encode(lg, valid.astype(np.uint8).ravel(), local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8, 1.0, 0.0)
        elif name == 'intensity':
            _encode(lg, np.where(obst[pad:pad + n, pad:pad + n], 30, 120).astype(np.uint8).ravel(),
                    local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8, 1.0, 0.0)
        elif name == 'no_step':
            _encode(lg, np.round(od.ravel() / 0.001).astype(np.int16), local_grid_pb2.LocalGrid.CELL_FORMAT_INT16, 0.001, 0.0)
        else:
            continue
        out.append(lg)
    return out


class SimLocalGridClient:
    """The local-grid service on a SimSpot (which keeps the simulator clock in step)."""

    def __init__(self, spot):
        self.spot = spot

    def get_local_grids(self, names, timeout=None, **kw):
        self.spot.sync()
        return grid_responses(self.spot.backend, list(names))

    def get_local_grid_types(self, **kw):
        return [NS(name=n) for n in ('obstacle_distance', 'terrain', 'terrain_valid', 'intensity', 'no_step')]


class SimSpot:
    """Command client and state client of the simulated robot (one object, as tests/fake_spot)."""

    def __init__(self, backend, battery_pct=100.0):
        self.backend = backend
        self.battery_pct = battery_pct
        self._lock = threading.RLock()
        self._wall0, self._sim0 = time.time(), backend.now()
        self._cmds, self._active, self._next = {}, None, 1
        self.commands = 0

    def sim_time(self, wall):
        return self._sim0 + (wall - self._wall0)

    def sync(self):
        """Advance the simulator to the wall clock."""
        with self._lock:
            self.backend.step_to(self.sim_time(time.time()))

    # --- command client ---------------------------------------------------------------
    def robot_command(self, command=None, end_time_secs=None, lease=None, **kw):
        with self._lock:
            self.sync()
            now = time.time()
            self.commands += 1
            cid, self._next = self._next, self._next + 1
            c = dict(end=None if end_time_secs is None else self.sim_time(end_time_secs), traj=False)
            mob = command.synchronized_command.mobility_command if command is not None else None
            if mob is not None and mob.HasField('se2_trajectory_request'):
                traj = mob.se2_trajectory_request.trajectory
                t0 = timestamp_to_sec(traj.reference_time) if traj.HasField('reference_time') else now
                pts = [(p.pose.position.x, p.pose.position.y, p.pose.angle,
                        self.sim_time(t0 + duration_to_seconds(p.time_since_reference))) for p in traj.points]
                params = spot_command_pb2.MobilityParams()
                mob.params.Unpack(params)
                v = params.vel_limit.max_vel.linear.x or 1.6
                w = params.vel_limit.max_vel.angular or 1.0
                self.backend.follow_path(pts, v, w)
                c['traj'] = True
            else:                       # stop, stand, sit, ...: stand still
                self.backend.stop()
            self._cmds[cid] = c
            self._active = cid
            return cid

    def robot_command_feedback(self, cid, **kw):
        with self._lock:
            self.sync()
            c = self._cmds[cid]
            r = robot_command_pb2.RobotCommandFeedbackResponse()
            m = r.feedback.synchronized_feedback.mobility_command_feedback
            tf = m.se2_trajectory_feedback
            st = self.backend.path_status() if cid == self._active else 'overridden'
            if cid != self._active:
                m.status = _ST.STATUS_COMMAND_OVERRIDDEN
            elif c['end'] is not None and self.backend.now() > c['end'] and st not in ('at_goal', 'stopped'):
                self.backend.stop()
                m.status = _ST.STATUS_COMMAND_TIMED_OUT
                st = 'stopped'
            else:
                m.status = _ST.STATUS_PROCESSING
            tf.status = {'moving': _FB.STATUS_GOING_TO_GOAL, 'near_goal': _FB.STATUS_NEAR_GOAL,
                         'at_goal': _FB.STATUS_AT_GOAL, 'blocked': _FB.STATUS_GOING_TO_GOAL}.get(st, _FB.STATUS_STOPPED)
            tf.final_goal_status = (_FB.FINAL_GOAL_STATUS_BLOCKED if st == 'blocked'
                                    else _FB.FINAL_GOAL_STATUS_ACHIEVABLE)
            tf.body_movement_status = (_FB.BODY_STATUS_SETTLED if st in ('at_goal', 'stopped', 'overridden')
                                       else _FB.BODY_STATUS_MOVING)
            return r

    # --- state client -----------------------------------------------------------------
    def get_robot_state(self, timeout=None, **kw):
        with self._lock:
            self.sync()
            s = robot_state_pb2.RobotState()
            robot_snapshot(s.kinematic_state.transforms_snapshot, self.backend)
            s.power_state.motor_power_state = robot_state_pb2.PowerState.MOTOR_POWER_STATE_ON
            s.behavior_state.state = robot_state_pb2.BehaviorState.STATE_STANDING
            b = s.battery_states.add()
            b.charge_percentage.value = self.battery_pct
            if self.backend.fallen():
                f = s.behavior_fault_state.faults.add()
                f.behavior_fault_id, f.cause = 1, robot_state_pb2.BehaviorFault.CAUSE_FALL
                f.status = robot_state_pb2.BehaviorFault.STATUS_CLEARABLE
            return s


class MemoryGraphNav:
    """Recording + GraphNav + map processing on an in-memory graph (tests/test_mission
    FakeGraphNav, with the pose of the backend); navigate_to teleports the robot."""

    def __init__(self, spot):
        self.spot = spot
        self.graph = map_pb2.Graph()
        self.pos = {}
        self.recording = False
        self.last_wp = None
        self.n = 0
        self.trips = []
        self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL

    def _robot(self):
        self.spot.sync()
        x, y, _z, yaw, _r, _p = self.spot.backend.pose()
        return x, y, yaw

    def _new_waypoint(self, name):
        self.n += 1
        wid = f"wp-sim-{self.n}"
        w = self.graph.waypoints.add()
        w.id = wid
        w.annotations.name = name
        w.annotations.creation_time.seconds = int(time.time())
        x, y, yaw = self._robot()
        self.pos[wid] = (x, y, yaw)
        w.waypoint_tform_ko.CopyFrom(SE3Pose(x, y, 0.0, Quat.from_yaw(yaw)).inverse().to_proto())
        if self.last_wp is not None:
            self._edge(self.last_wp, wid, map_pb2.Edge.EDGE_SOURCE_ODOMETRY)
        self.last_wp = wid
        return w

    def _edge(self, a, b, source):
        e = self.graph.edges.add()
        e.id.from_waypoint, e.id.to_waypoint = a, b
        (xa, ya, ta), (xb, yb, tb) = self.pos[a], self.pos[b]
        e.from_tform_to.CopyFrom((SE3Pose(xa, ya, 0, Quat.from_yaw(ta)).inverse()
                                  * SE3Pose(xb, yb, 0, Quat.from_yaw(tb))).to_proto())
        e.annotations.edge_source = source

    def start_recording(self, **kw):
        self.recording = True
        self.last_wp = self._nearest()
        return recording_pb2.StartRecordingResponse(status=recording_pb2.StartRecordingResponse.STATUS_OK)

    def stop_recording(self, **kw):
        self.recording = False
        return recording_pb2.StopRecordingResponse(status=recording_pb2.StopRecordingResponse.STATUS_OK)

    def get_record_status(self, **kw):
        return NS(is_recording=self.recording)

    def create_waypoint(self, waypoint_name=None, **kw):
        w = self._new_waypoint(waypoint_name or f"auto_{self.n}")
        r = recording_pb2.CreateWaypointResponse(status=recording_pb2.CreateWaypointResponse.STATUS_OK)
        r.created_waypoint.CopyFrom(w)
        return r

    def create_edge(self, edge=None, **kw):
        if not self.recording:
            raise RuntimeError("NotRecordingError (sim)")
        self.graph.edges.add().CopyFrom(edge)

    def _nearest(self):
        if not self.pos:
            return None
        x, y, _ = self._robot()
        return min(self.pos, key=lambda k: np.hypot(self.pos[k][0] - x, self.pos[k][1] - y))

    def download_graph(self, **kw):
        g = map_pb2.Graph()
        g.CopyFrom(self.graph)
        return g

    def get_localization_state(self, **kw):
        st = graph_nav_pb2.GetLocalizationStateResponse()
        wid = self._nearest()
        if wid is not None:
            st.localization.waypoint_id = wid
            x, y, yaw = self.pos[wid]
            rx, ry, ryaw = self._robot()
            st.localization.waypoint_tform_body.CopyFrom(
                (SE3Pose(x, y, 0, Quat.from_yaw(yaw)).inverse() * SE3Pose(rx, ry, 0, Quat.from_yaw(ryaw))).to_proto())
        return st

    def set_localization(self, **kw):
        raise RuntimeError("STATUS_NO_MATCHING_FIDUCIAL (sim: no fiducial in the world)")

    def navigate_to(self, waypoint_id, cmd_duration, command_id=None, travel_params=None,
                    destination_waypoint_tform_body_goal=None, **kw):
        x, y, yaw = self.pos[waypoint_id]
        if destination_waypoint_tform_body_goal is not None:
            yaw += destination_waypoint_tform_body_goal.angle
        self.trips.append((self._nearest(), waypoint_id))
        self.spot.backend.teleport(x, y, yaw)
        self.status = graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        return 1

    def navigation_feedback(self, command_id, **kw):
        return NS(status=self.status)

    def clear_graph(self, **kw):
        self.__init__(self.spot)

    def upload_graph(self, graph=None, **kw):
        for e in graph.edges:
            self.graph.edges.add().CopyFrom(e)

    def download_waypoint_snapshot(self, snapshot_id, **kw):
        return map_pb2.WaypointSnapshot(id=snapshot_id)

    def download_edge_snapshot(self, snapshot_id, **kw):
        return map_pb2.EdgeSnapshot(id=snapshot_id)

    def process_topology(self, params=None, modify_map_on_server=False, **kw):
        return map_processing_pb2.ProcessTopologyResponse(status=map_processing_pb2.ProcessTopologyResponse.STATUS_OK)

    def process_anchoring(self, *a, **kw):
        r = map_processing_pb2.ProcessAnchoringResponse(status=map_processing_pb2.ProcessAnchoringResponse.STATUS_OK)
        r.iteration = 1
        return r


class _NoWorldObjects:
    def list_world_objects(self, object_type=None, **kw):
        return world_object_pb2.ListWorldObjectResponse()


class _NoImages:
    def list_image_sources(self, **kw):
        return []

    def get_image_from_sources(self, *a, **kw):
        raise RuntimeError("no cameras in the simulation")

    def get_image(self, *a, **kw):
        raise RuntimeError("no cameras in the simulation")


class SimRobot:
    """The robot object of the mission: ensure_client() by service name, power, time sync."""

    def __init__(self, backend):
        import logging
        self.backend = backend
        self.spot = SimSpot(backend)
        self.grids = SimLocalGridClient(self.spot)
        self.graph_nav = MemoryGraphNav(self.spot)
        self.logger = logging.getLogger("sim-robot")
        self.time_sync = NS(wait_for_sync=lambda *a, **k: None,
                            get_robot_clock_skew=lambda *a, **k: NS(ToTimedelta=lambda: NS(total_seconds=lambda: 0.0)))
        self._current_user = "sim"
        self.powered = False
        self._clients = {
            'robot-state': self.spot, 'robot-command': self.spot, 'local-grid-service': self.grids,
            'graph-nav-service': self.graph_nav, 'recording-service': self.graph_nav,
            'map-processing-service': self.graph_nav, 'world-objects': _NoWorldObjects(), 'image': _NoImages(),
        }

    def ensure_client(self, name):
        return self._clients.get(name) or NS(name=name)

    def power_on(self, **kw):
        self.powered = True

    def power_off(self, **kw):
        self.powered = False

    def is_powered_on(self):
        return self.powered
