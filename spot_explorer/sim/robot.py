"""
SimRobot: what the mission gets instead of the SDK robot when it runs with --sim.

The mission code talks to Spot through a handful of SDK clients (robot state, robot command,
local grid, GraphNav, recording, map processing, world objects). Here each one has a stand-in
with the same methods, answering with the SAME SDK messages (real protobufs), filled from the
simulator. So everything above this file -- perception, graph, frontier, decisions, logs --
is the code that runs on the robot, unchanged.

    mission code  ->  SimRobot / Sim*Client (this file)  ->  socket  ->  simulator
                      controller.py  graph.py  world.py                   (Isaac Sim or kinematic)

What is simulated and what is not is listed in sim/README_SIM.md. In short: the motion is
real simulated walking (in Isaac Sim), the local grid is computed from the world file, GraphNav
is a small exact one, there are no fiducials, no lease and no E-Stop.

Sensors (Isaac Sim only). The mission does not use them: it reads the world through Spot's
local grid. They are here for whoever wants to work on them in simulation:

    robot.ensure_client(ImageClient.default_service_name)        five cameras, colour and depth
    robot.ensure_client(PointCloudClient.default_service_name)   the lidar ('velodyne-point-cloud')
    robot_state_client.get_robot_state()     body velocity and joint states, besides the pose
    robot_state_client.get_imu_state()       the IMU (on the real robot: the state stream)
"""
import logging
import time

import numpy as np
from bosdyn.api import (basic_command_pb2, geometry_pb2, image_pb2, local_grid_pb2,
                        point_cloud_pb2, robot_command_pb2, robot_state_pb2, world_object_pb2)
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, map_processing_pb2, recording_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME,
                                         ODOM_FRAME_NAME, VISION_FRAME_NAME)
from bosdyn.client.graph_nav import GraphNavClient, GraphNavServiceResponseError
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.point_cloud import PointCloudClient
from bosdyn.client.recording import GraphNavRecordingServiceClient, NotRecordingError
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient

from spot_explorer.sim import controller as ctl
from spot_explorer.sim import graph as simgraph
from spot_explorer.sim.link import SimLink, SimLinkError, parse_address
from spot_explorer.sim.world import World

IDENTITY = (0.0, 0.0, 0.0, (1.0, 0.0, 0.0, 0.0))
# Speed limits when a command carries none (Spot would use its own maximum).
NO_LIMIT_LINEAR_MPS = 1.0
NO_LIMIT_ANGULAR_RPS = 1.5
# The terrain layer stores heights as integers: height = raw * scale + offset. The offset is
# kept well below the ground under the robot so that no real cell has raw == 0 (the "never written" value).
TERRAIN_SCALE_M = 0.001
TERRAIN_OFFSET_BELOW_GROUND_M = 2.0
DISTANCE_SCALE_M = 0.001
# The joints of the simulated model, with the names the SDK uses for Spot's.
SDK_JOINT_NAMES = {'%s_%s' % (leg, j): '%s.%s' % (leg, j)
                   for leg in ('fl', 'fr', 'hl', 'hr') for j in ('hx', 'hy', 'kn')}
LIDAR_SERVICE_NAME = 'velodyne-point-cloud'      # the lidar payload of the real robot
DEPTH_SCALE = 1000.0                             # depth images: millimetres


# FIX 26-11: the failures of a robot service are raised as the SDK's own errors (were RuntimeError /
# KeyError): the mission catches only those around its robot calls (spot_explorer/robot/errors.py),
# anything else is taken for a bug and goes to the emergency procedure.
class SimNotRecordingError(NotRecordingError):
    """The SDK's NotRecordingError: waypoints and edges only while recording."""

    def __init__(self, message):
        super().__init__(None, message)


class SimGraphNavError(GraphNavServiceResponseError):  # ADD 26-11: see the FIX above
    """A GraphNav request the simulation cannot satisfy (no snapshots, no fiducials, an edge
    between unknown waypoints), as the SDK's GraphNav response error."""

    def __init__(self, message):
        super().__init__(None, message)


# --- messages ----------------------------------------------------------------------------------
def _se3(pose):
    x, y, z, (qw, qx, qy, qz) = pose
    return geometry_pb2.SE3Pose(position=geometry_pb2.Vec3(x=x, y=y, z=z),
                                rotation=geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz))


def _snapshot(frames):
    """FrameTreeSnapshot with VISION as the root; frames = {name: pose in VISION}."""
    snap = geometry_pb2.FrameTreeSnapshot()
    root = snap.child_to_parent_edge_map[VISION_FRAME_NAME]
    root.parent_frame_name = ''
    root.parent_tform_child.CopyFrom(_se3(IDENTITY))
    for name, pose in frames.items():
        edge = snap.child_to_parent_edge_map[name]
        edge.parent_frame_name = VISION_FRAME_NAME
        edge.parent_tform_child.CopyFrom(_se3(pose))
    return snap


def _now():
    from google.protobuf import timestamp_pb2
    t = time.time()
    return timestamp_pb2.Timestamp(seconds=int(t), nanos=int((t % 1.0) * 1e9))


def _pose(p, q):
    return (float(p[0]), float(p[1]), float(p[2]), tuple(float(v) for v in q))


class _Core:
    """What the clients share: link, controller, world, graph, power and battery."""

    def __init__(self, link):
        self.link = link
        self.world = World.from_dict(link.world())
        self.controller = ctl.BaseController(link)
        self.graph = simgraph.SimGraph(self.controller)
        self.powered = False
        self.standing = False
        self.t0 = time.time()
        self._sensor_link = None
        self._sensors = None

    def sensor_link(self):
        """A link of its own for the sensors: an image request must not delay the controller."""
        if self._sensor_link is None:
            self._sensor_link = SimLink(self.link.host, self.link.port)
        return self._sensor_link

    def sensors(self):
        if self._sensors is None:
            self._sensors = self.sensor_link().sensors()
        return self._sensors

    def body_frames(self):
        st = self.controller.state()
        body = simgraph.pose_of_state(st)
        return st, {ODOM_FRAME_NAME: IDENTITY, BODY_FRAME_NAME: body,
                    GROUND_PLANE_FRAME_NAME: (st['x'], st['y'],
                                              self.world.height_at(st['x'], st['y']),
                                              simgraph.q_from_yaw(st['yaw']))}

    def battery_pct(self):
        b = self.world.battery
        minutes = (time.time() - self.t0) / 60.0
        return float(np.clip(float(b.get('start_pct', 100.0))
                             - float(b.get('drain_pct_per_min', 0.0)) * minutes, 0.0, 100.0))


# --- robot state -------------------------------------------------------------------------------
class SimRobotStateClient:
    def __init__(self, core):
        self._core = core

    def get_robot_state(self, **kwargs):
        core = self._core
        st, frames = core.body_frames()
        state = robot_state_pb2.RobotState()
        kin = state.kinematic_state
        kin.transforms_snapshot.CopyFrom(_snapshot(frames))
        kin.acquisition_timestamp.CopyFrom(_now())
        extra = st.get('extra') or {}
        if 'v' in extra and 'w' in extra:
            # Odometry = VISION here, so the two velocities are the same.
            for vel in (kin.velocity_of_body_in_vision, kin.velocity_of_body_in_odom):
                vel.linear.x, vel.linear.y, vel.linear.z = extra['v']
                vel.angular.x, vel.angular.y, vel.angular.z = extra['w']
        for i, name in enumerate(core.link.joints):
            if i < len(extra.get('jp', ())):
                joint = kin.joint_states.add()
                joint.name = SDK_JOINT_NAMES.get(name, name)
                joint.position.value = extra['jp'][i]
                if i < len(extra.get('jv', ())):
                    joint.velocity.value = extra['jv'][i]
        PS = robot_state_pb2.PowerState
        state.power_state.motor_power_state = (PS.MOTOR_POWER_STATE_ON if core.powered
                                               else PS.MOTOR_POWER_STATE_OFF)
        pct = core.battery_pct()
        state.power_state.locomotion_charge_percentage.value = pct
        battery = state.battery_states.add()
        battery.identifier = 'sim-battery'
        battery.charge_percentage.value = pct
        BS = robot_state_pb2.BehaviorState
        state.behavior_state.state = BS.STATE_STANDING if core.standing else BS.STATE_NOT_READY
        return state

    def get_imu_state(self, **kwargs):
        """The IMU readings since the previous call (100 per simulated second, at most the
        last second) as the SDK's ImuState: specific force and angular rate in the body frame,
        orientation of the body in the odometry frame. On the real robot these packets come
        with the state stream (RobotStateStreamingClient), not with get_robot_state. None
        where the simulator has no IMU."""
        core = self._core
        if core.sensors().get('imu') is None:
            return None
        st = core.controller.state()
        since = getattr(self, '_imu_last_t', None)
        if since is None or since < st['t'] - 1.0:
            since = st['t'] - 0.1
        rows = core.sensor_link().imu(since)
        out = robot_state_pb2.ImuState(identifier='sim-imu', mounting_link_name=BODY_FRAME_NAME)
        out.packet_rate = float(rows.get('hz', 100.0))
        wall0 = time.time() - st['t']               # simulated time -> clock, roughly
        for t, a, g, q in zip(rows['t'], rows['a'], rows['g'], rows['q']):
            packet = out.packets.add()
            packet.acceleration_rt_odom_in_link_frame.x, packet.acceleration_rt_odom_in_link_frame.y, \
                packet.acceleration_rt_odom_in_link_frame.z = a
            packet.angular_velocity_rt_odom_in_link_frame.x, packet.angular_velocity_rt_odom_in_link_frame.y, \
                packet.angular_velocity_rt_odom_in_link_frame.z = g
            packet.odom_rot_link.w, packet.odom_rot_link.x, packet.odom_rot_link.y, packet.odom_rot_link.z = q
            packet.timestamp.seconds, packet.timestamp.nanos = int(wall0 + t), int(((wall0 + t) % 1.0) * 1e9)
        if rows['t']:
            self._imu_last_t = rows['t'][-1]
        return out


# --- robot command -----------------------------------------------------------------------------
class SimRobotCommandClient:
    """Trajectory point, stand, sit, stop, self-right. Returns the SDK feedback messages."""

    def __init__(self, core):
        self._core = core
        self._commands = {}
        self._next_id = 1

    def robot_command(self, command=None, end_time_secs=None, timesync_endpoint=None,
                      lease=None, **kwargs):
        core = self._core
        cid = self._next_id
        self._next_id += 1
        rec = {'kind': 'other', 'gid': None}
        if command.HasField('full_body_command'):
            full = command.full_body_command
            core.controller.stop()
            if full.HasField('selfright_request'):
                rec['kind'] = 'selfright'
            elif full.HasField('safe_power_off_request'):
                core.powered = core.standing = False
            else:
                rec['kind'] = 'stop'
        elif command.HasField('synchronized_command') and \
                command.synchronized_command.HasField('mobility_command'):
            mob = command.synchronized_command.mobility_command
            if mob.HasField('se2_trajectory_request'):
                rec = self._trajectory(mob, end_time_secs)
            elif mob.HasField('stand_request'):
                core.controller.stop()
                core.standing = True
                rec['kind'] = 'stand'
            elif mob.HasField('sit_request'):
                core.controller.stop()
                core.standing = False
                rec['kind'] = 'sit'
            else:
                core.controller.stop()
                rec['kind'] = 'stop'
        self._commands[cid] = rec
        for old in [k for k in self._commands if k < cid - 50]:
            del self._commands[old]
        return cid

    def _trajectory(self, mob, end_time_secs):
        req = mob.se2_trajectory_request
        if req.se2_frame_name not in (VISION_FRAME_NAME, ODOM_FRAME_NAME):
            raise ValueError("simulated robot: trajectory in frame %r not supported (vision or odom)"
                             % (req.se2_frame_name,))
        if not req.trajectory.points:
            raise ValueError("simulated robot: trajectory without points")
        traj = req.trajectory
        goal = traj.points[-1].pose
        v_max, w_max = NO_LIMIT_LINEAR_MPS, NO_LIMIT_ANGULAR_RPS
        params = spot_command_pb2.MobilityParams()
        if mob.HasField('params') and mob.params.Unpack(params) and params.HasField('vel_limit'):
            lim = params.vel_limit.max_vel
            v_max = max(abs(lim.linear.x), abs(lim.linear.y)) or v_max
            w_max = abs(lim.angular) or w_max
        timeout = 10.0 if end_time_secs is None else max(0.1, float(end_time_secs) - time.time())
        if len(traj.points) > 1:  # FIX 25-11: was only points[-1], straight to the last point
            # Multi-point trajectory (the path command of the continuous motion): every point in
            # order and in time, as Spot does. Taking only the last one cut the corners straight
            # through the obstacles the path went around. Times: reference_time (client clock,
            # the same clock here) + time_since_reference; without a reference, from now.
            ref = traj.reference_time.seconds + traj.reference_time.nanos * 1e-9 \
                if traj.HasField('reference_time') else time.time()
            pts = [(p.pose.position.x, p.pose.position.y, p.pose.angle,
                    ref + p.time_since_reference.seconds + p.time_since_reference.nanos * 1e-9)
                   for p in traj.points]
            gid = self._core.controller.go_to_path(pts, v_max, w_max, timeout)
        else:
            gid = self._core.controller.go_to(goal.position.x, goal.position.y, goal.angle,
                                              v_max, w_max, timeout)
        return {'kind': 'trajectory', 'gid': gid}

    def robot_command_feedback(self, robot_command_id=None, **kwargs):
        rec = self._commands.get(robot_command_id)
        if rec is None:
            raise KeyError("simulated robot: unknown command id %r" % (robot_command_id,))
        ST = basic_command_pb2.RobotCommandFeedbackStatus
        resp = robot_command_pb2.RobotCommandFeedbackResponse()
        if rec['kind'] == 'selfright':
            fb = resp.feedback.full_body_feedback
            fb.status = ST.STATUS_PROCESSING
            fb.selfright_feedback.status = basic_command_pb2.SelfRightCommand.Feedback.STATUS_COMPLETED
            return resp
        mob = resp.feedback.synchronized_feedback.mobility_command_feedback
        mob.status = ST.STATUS_PROCESSING
        if rec['kind'] == 'stand':
            mob.stand_feedback.status = basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING
        elif rec['kind'] == 'sit':
            mob.sit_feedback.status = basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING
        elif rec['kind'] == 'trajectory':
            TF = basic_command_pb2.SE2TrajectoryCommand.Feedback
            status = self._core.controller.status(rec['gid'])
            if status == ctl.AT_GOAL:
                mob.se2_trajectory_feedback.status = TF.STATUS_AT_GOAL
                mob.se2_trajectory_feedback.body_movement_status = TF.BODY_STATUS_SETTLED
            elif status == ctl.GOING:
                mob.se2_trajectory_feedback.status = TF.STATUS_GOING_TO_GOAL
                mob.se2_trajectory_feedback.body_movement_status = TF.BODY_STATUS_MOVING
            elif status == ctl.TIMED_OUT:
                mob.status = ST.STATUS_COMMAND_TIMED_OUT
            else:
                mob.status = ST.STATUS_COMMAND_OVERRIDDEN
        return resp

    def clear_behavior_fault(self, behavior_fault_id=None, **kwargs):
        return True


# --- local grid --------------------------------------------------------------------------------
class SimLocalGridClient:
    """obstacle_distance, terrain and terrain_valid, computed from the world (see world.py).
    A layer the simulation does not have (no_step, intensity) is simply not returned."""

    def __init__(self, core):
        self._core = core

    def get_local_grids(self, local_grid_type_names, **kwargs):
        core = self._core
        # As on the robot, the snapshot of every grid also carries the body, odometry and
        # ground plane frames at the time of the grid: the mission reads the robot pose there.
        st, body_frames = core.body_frames()
        grid = core.world.local_grid(st['x'], st['y'])
        n, cell = grid['n'], grid['cell']
        ox, oy = grid['origin']
        LG = local_grid_pb2.LocalGrid
        # As Spot, every grid says when it was taken (the mission measures how often the grid
        # is new from it); all the layers of one call are the same capture.
        stamp = _now()  # ADD 25-11: one acquisition time for all the layers of this call
        out = []
        for name in local_grid_type_names:
            if name == 'obstacle_distance':
                raw = np.clip(np.round(grid['obstacle_distance'] / DISTANCE_SCALE_M), -32000, 32000)
                data, fmt, scale, offset = raw.astype('<i2'), LG.CELL_FORMAT_INT16, DISTANCE_SCALE_M, 0.0
            elif name == 'terrain':
                offset = core.world.height_at(st['x'], st['y']) - TERRAIN_OFFSET_BELOW_GROUND_M
                raw = np.clip(np.round((grid['terrain'] - offset) / TERRAIN_SCALE_M), -32000, 32000)
                data, fmt, scale = raw.astype('<i2'), LG.CELL_FORMAT_INT16, TERRAIN_SCALE_M
            elif name == 'terrain_valid':
                data, fmt, scale, offset = grid['terrain_valid'].astype('u1'), LG.CELL_FORMAT_UINT8, 1.0, 0.0
            else:
                continue
            frame = 'sim_local_grid_' + name
            resp = local_grid_pb2.LocalGridResponse(local_grid_type_name=name)
            resp.status = local_grid_pb2.LocalGridResponse.STATUS_OK
            lg = resp.local_grid
            lg.local_grid_type_name = name
            lg.frame_name_local_grid_data = frame
            lg.acquisition_time.CopyFrom(stamp)  # FIX 25-11: grids had no time ('Spot grid never new')
            frames = dict(body_frames)
            frames[frame] = (ox, oy, 0.0, (1.0, 0.0, 0.0, 0.0))
            lg.transforms_snapshot.CopyFrom(_snapshot(frames))
            lg.extent.cell_size = cell
            lg.extent.num_cells_x = n
            lg.extent.num_cells_y = n
            lg.cell_format = fmt
            lg.encoding = LG.ENCODING_RAW
            lg.cell_value_scale = scale
            lg.cell_value_offset = offset
            lg.data = data.tobytes()
            out.append(resp)
        return out


# --- GraphNav ----------------------------------------------------------------------------------
_NAV_STATUS = {
    simgraph.FOLLOWING: 'STATUS_FOLLOWING_ROUTE', simgraph.REACHED: 'STATUS_REACHED_GOAL',
    simgraph.NO_ROUTE: 'STATUS_NO_ROUTE', simgraph.NO_LOCALIZATION: 'STATUS_NO_LOCALIZATION',
    simgraph.STUCK: 'STATUS_STUCK', simgraph.TIMED_OUT: 'STATUS_COMMAND_TIMED_OUT',
    simgraph.OVERRIDDEN: 'STATUS_COMMAND_OVERRIDDEN',
}


class SimGraphNavClient:
    def __init__(self, core):
        self._core = core

    def clear_graph(self, **kwargs):
        self._core.graph.clear()

    def download_graph(self, **kwargs):
        g = self._core.graph
        graph = map_pb2.Graph()
        with g._lock:
            wps = dict(g.waypoints)
            edges = g.edges()
        for wp in wps.values():
            w = graph.waypoints.add()
            w.id = wp['id']
            w.annotations.name = wp['name']
            # Pose of the odometry frame seen from the waypoint (here odometry = VISION).
            w.waypoint_tform_ko.CopyFrom(_se3(simgraph.pose_inverse(wp['pose'])))
        for a, b in edges:
            e = graph.edges.add()
            e.id.from_waypoint, e.id.to_waypoint = a, b
            e.from_tform_to.CopyFrom(_se3(simgraph.pose_mul(simgraph.pose_inverse(wps[a]['pose']),
                                                           wps[b]['pose'])))
        return graph

    def upload_graph(self, graph=None, generate_new_anchoring=False, **kwargs):
        for e in (graph.edges if graph is not None else ()):
            try:
                self._core.graph.add_edge(e.id.from_waypoint, e.id.to_waypoint)
            except KeyError as err:                  # FIX 26-11: an SDK error, see SimGraphNavError
                raise SimGraphNavError(str(err)) from err

    def download_waypoint_snapshot(self, waypoint_snapshot_id, **kwargs):
        raise SimGraphNavError("simulated GraphNav: waypoints have no snapshots")  # FIX 26-11: was KeyError

    def download_edge_snapshot(self, edge_snapshot_id, **kwargs):
        raise SimGraphNavError("simulated GraphNav: edges have no snapshots")  # FIX 26-11: was KeyError

    def get_localization_state(self, **kwargs):
        wid, rel = self._core.graph.localization()
        resp = graph_nav_pb2.GetLocalizationStateResponse()
        if wid is not None:
            resp.localization.waypoint_id = wid
            resp.localization.waypoint_tform_body.CopyFrom(_se3(rel))
        return resp

    def set_localization(self, initial_guess_localization=None, ko_tform_body=None,
                         max_distance=None, max_yaw=None, fiducial_init=None, **kwargs):
        SL = graph_nav_pb2.SetLocalizationRequest
        if fiducial_init in (SL.FIDUCIAL_INIT_NEAREST, SL.FIDUCIAL_INIT_SPECIFIC):
            raise SimGraphNavError("simulated GraphNav: there are no fiducials in the simulation")  # FIX 26-11: was RuntimeError
        try:
            self._core.graph.set_localized()
        except RuntimeError as err:          # FIX 26-11: an empty map, nothing to localize on: an SDK error
            raise SimGraphNavError(str(err)) from err

    def navigate_to(self, destination_waypoint_id, cmd_duration, route_params=None,
                    travel_params=None, leases=None, timesync_endpoint=None,
                    destination_waypoint_tform_body_goal=None, command_id=None, **kwargs):
        v_max, w_max = NO_LIMIT_LINEAR_MPS, NO_LIMIT_ANGULAR_RPS
        free_yaw = False
        if travel_params is not None:
            lim = travel_params.velocity_limit.max_vel
            v_max = max(abs(lim.linear.x), abs(lim.linear.y)) or v_max
            w_max = abs(lim.angular) or w_max
            free_yaw = bool(getattr(travel_params, 'ignore_final_yaw', False))
        goal = destination_waypoint_tform_body_goal
        return self._core.graph.navigate(
            destination_waypoint_id, cmd_duration, v_max, w_max,
            final_yaw_offset=None if goal is None else float(goal.angle),
            ignore_final_yaw=free_yaw, command_id=command_id)

    def navigation_feedback(self, command_id=0, **kwargs):
        status = self._core.graph.navigation_status(command_id)
        NF = graph_nav_pb2.NavigationFeedbackResponse
        return NF(status=getattr(NF, _NAV_STATUS[status]))


class SimRecordingClient:
    def __init__(self, core):
        self._core = core

    def start_recording(self, recording_environment=None, **kwargs):
        self._core.graph.start_recording()
        return recording_pb2.StartRecordingResponse.STATUS_OK

    def stop_recording(self, **kwargs):
        self._core.graph.stop_recording()
        return recording_pb2.StopRecordingResponse.STATUS_OK

    def get_record_status(self, **kwargs):
        return recording_pb2.GetRecordStatusResponse(is_recording=self._core.graph.recording)

    def create_waypoint(self, waypoint_name=None, recording_environment=None, **kwargs):
        if not self._core.graph.recording:
            raise SimNotRecordingError("simulated GraphNav: not recording, waypoint not created")
        wp = self._core.graph.create_waypoint(waypoint_name)
        resp = recording_pb2.CreateWaypointResponse()
        resp.status = recording_pb2.CreateWaypointResponse.STATUS_OK
        resp.created_waypoint.id = wp['id']
        resp.created_waypoint.annotations.name = wp['name']
        return resp

    def create_edge(self, edge=None, **kwargs):
        if not self._core.graph.recording:
            raise SimNotRecordingError("simulated GraphNav: not recording, edge not created")
        try:
            self._core.graph.add_edge(edge.id.from_waypoint, edge.id.to_waypoint)
        except KeyError as err:                      # FIX 26-11: an SDK error, see SimGraphNavError
            raise SimGraphNavError(str(err)) from err
        return recording_pb2.CreateEdgeResponse.STATUS_OK


class SimMapProcessingClient:
    """No loop closure and no anchoring to do: the simulated map has no drift."""

    def __init__(self, core):
        self._core = core

    def process_topology(self, params=None, modify_map_on_server=False, **kwargs):
        resp = map_processing_pb2.ProcessTopologyResponse()
        resp.status = map_processing_pb2.ProcessTopologyResponse.STATUS_OK
        return resp

    def process_anchoring(self, params=None, modify_anchoring_on_server=True,
                          stream_intermediate_results=False, **kwargs):
        resp = map_processing_pb2.ProcessAnchoringResponse()
        resp.status = map_processing_pb2.ProcessAnchoringResponse.STATUS_OK
        return resp


class SimWorldObjectClient:
    def __init__(self, core):
        self._core = core

    def list_world_objects(self, object_type=None, **kwargs):
        return world_object_pb2.ListWorldObjectResponse()


# --- cameras -----------------------------------------------------------------------------------
class SimImageClient:
    """The five body cameras of Isaac Sim with the names of the real ones:

        <camera>_fisheye_image            colour, JPEG (wide angle, but NOT a fisheye lens)
        <camera>_depth_in_visual_frame    depth seen from the same camera, 16-bit millimetres
        <camera>_depth                    the same picture: here depth has no viewpoint of its own

    with <camera> = frontleft, frontright, left, right, back. Unlike the real ones the
    pictures are upright (the real front cameras are mounted rotated by 90 degrees).
    """

    def __init__(self, core):
        self._core = core

    def _cameras(self):
        return {c['name']: c for c in self._core.sensors().get('cameras', ())}

    @staticmethod
    def _split(source_name):
        for suffix, depth in (('_fisheye_image', False), ('_depth_in_visual_frame', True),
                              ('_depth', True)):
            if source_name.endswith(suffix):
                return source_name[:-len(suffix)], depth
        return source_name, False

    def _source(self, name, cam, depth):
        src = image_pb2.ImageSource(name=name, cols=cam['w'], rows=cam['h'])
        src.pinhole.intrinsics.focal_length.x, src.pinhole.intrinsics.focal_length.y = cam['fx'], cam['fy']
        src.pinhole.intrinsics.principal_point.x, src.pinhole.intrinsics.principal_point.y = cam['cx'], cam['cy']
        if depth:
            src.image_type = image_pb2.ImageSource.IMAGE_TYPE_DEPTH
            src.depth_scale = DEPTH_SCALE
            src.pixel_formats.append(image_pb2.Image.PIXEL_FORMAT_DEPTH_U16)
            src.image_formats.append(image_pb2.Image.FORMAT_RAW)
        else:
            src.image_type = image_pb2.ImageSource.IMAGE_TYPE_VISUAL
            src.pixel_formats.append(image_pb2.Image.PIXEL_FORMAT_RGB_U8)
            src.image_formats.append(image_pb2.Image.FORMAT_JPEG)
        return src

    def list_image_sources(self, **kwargs):
        out = []
        for cam_name, cam in self._cameras().items():
            for suffix, depth in (('_fisheye_image', False), ('_depth_in_visual_frame', True),
                                  ('_depth', True)):
                out.append(self._source(cam_name + suffix, cam, depth))
        return out

    def get_image_from_sources(self, image_sources, **kwargs):
        cams = self._cameras()
        if not cams:
            raise SimLinkError("this simulator (%s) has no cameras" % (self._core.link.backend,))
        wanted = []
        for name in image_sources:
            cam_name, depth = self._split(name)
            if cam_name not in cams:
                raise KeyError("simulated robot: unknown image source %r (see list_image_sources)" % (name,))
            wanted.append((name, cam_name, depth))
        items = self._core.sensor_link().images([c + ('_depth' if d else '') for _, c, d in wanted])
        out = []
        for (name, cam_name, depth), item in zip(wanted, items):
            cam = cams[cam_name]
            resp = image_pb2.ImageResponse(status=image_pb2.ImageResponse.STATUS_OK)
            resp.source.CopyFrom(self._source(name, cam, depth))
            shot = resp.shot
            shot.acquisition_time.CopyFrom(_now())
            shot.frame_name_image_sensor = cam_name + '_fisheye'
            body = _pose(item['p'], item['q'])
            mount = _pose(cam['mount_p'], cam['mount_q'])
            shot.transforms_snapshot.CopyFrom(_snapshot({
                ODOM_FRAME_NAME: IDENTITY, BODY_FRAME_NAME: body,
                shot.frame_name_image_sensor: simgraph.pose_mul(body, mount)}))
            shot.image.cols, shot.image.rows = item['w'], item['h']
            if depth:
                shot.image.data = item['depth_mm']
                shot.image.format = image_pb2.Image.FORMAT_RAW
                shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
            else:
                shot.image.data = item['jpeg']
                shot.image.format = image_pb2.Image.FORMAT_JPEG
                shot.image.pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
            out.append(resp)
        return out

    def get_image(self, image_requests, **kwargs):
        return self.get_image_from_sources([r.image_source_name for r in image_requests])


# --- lidar -------------------------------------------------------------------------------------
class SimPointCloudClient:
    """The lidar of Isaac Sim as the point cloud service of Spot's lidar payload: one source,
    'velodyne-point-cloud', points as float32 x, y, z in the sensor frame."""

    def __init__(self, core):
        self._core = core

    def _has_lidar(self):
        return self._core.sensors().get('lidar') is not None

    def list_point_cloud_sources(self, **kwargs):
        if not self._has_lidar():
            return []
        return [point_cloud_pb2.PointCloudSource(name=LIDAR_SERVICE_NAME, frame_name_sensor='sensor')]

    def get_point_cloud_from_sources(self, point_cloud_sources, **kwargs):
        if not self._has_lidar():
            raise SimLinkError("this simulator (%s) has no lidar" % (self._core.link.backend,))
        out = []
        for name in point_cloud_sources:
            if name != LIDAR_SERVICE_NAME:
                raise KeyError("simulated robot: unknown point cloud source %r" % (name,))
            scan = self._core.sensor_link().lidar()
            resp = point_cloud_pb2.PointCloudResponse(status=point_cloud_pb2.PointCloudResponse.STATUS_OK)
            cloud = resp.point_cloud
            cloud.source.name = name
            cloud.source.frame_name_sensor = 'sensor'
            cloud.source.acquisition_time.CopyFrom(_now())
            body = _pose(scan['p'], scan['q'])
            cloud.source.transforms_snapshot.CopyFrom(_snapshot({
                ODOM_FRAME_NAME: IDENTITY, BODY_FRAME_NAME: body,
                'sensor': simgraph.pose_mul(body, _pose(scan['mount_p'], scan['mount_q']))}))
            cloud.num_points = scan['n']
            cloud.encoding = point_cloud_pb2.PointCloud.ENCODING_XYZ_32F
            cloud.data = scan['xyz']
            out.append(resp)
        return out

    def get_point_cloud(self, point_cloud_requests, **kwargs):
        return self.get_point_cloud_from_sources([r.point_cloud_source_name for r in point_cloud_requests])


class _NoService:
    """Lease and E-Stop: nothing to hold or to release in the simulation."""

    def __init__(self, core):
        self._core = core


# --- robot -------------------------------------------------------------------------------------
class _TimeSync:
    def wait_for_sync(self, timeout_sec=None):
        return True


class SimRobot:
    is_sim = True

    def __init__(self, host, port):
        self.link = SimLink(host, port)
        self._core = _Core(self.link)
        self.logger = logging.getLogger('sim_robot')
        self.time_sync = _TimeSync()
        self._current_user = 'sim'
        self._clients = {}
        self._factories = {
            RobotStateClient.default_service_name: SimRobotStateClient,
            RobotCommandClient.default_service_name: SimRobotCommandClient,
            LocalGridClient.default_service_name: SimLocalGridClient,
            GraphNavClient.default_service_name: SimGraphNavClient,
            GraphNavRecordingServiceClient.default_service_name: SimRecordingClient,
            MapProcessingServiceClient.default_service_name: SimMapProcessingClient,
            WorldObjectClient.default_service_name: SimWorldObjectClient,
            ImageClient.default_service_name: SimImageClient,
            PointCloudClient.default_service_name: SimPointCloudClient,
            LIDAR_SERVICE_NAME: SimPointCloudClient,
            LeaseClient.default_service_name: _NoService,
            EstopClient.default_service_name: _NoService,
        }

    def ensure_client(self, service_name, **kwargs):
        if service_name not in self._clients:
            if service_name not in self._factories:
                raise KeyError("simulated robot: service %r is not simulated" % (service_name,))
            self._clients[service_name] = self._factories[service_name](self._core)
        return self._clients[service_name]

    def power_on(self, timeout_sec=20, **kwargs):
        self._core.powered = True

    def power_off(self, cut_immediately=False, timeout_sec=20, **kwargs):
        self._core.controller.stop()
        self._core.powered = self._core.standing = False

    def is_powered_on(self, **kwargs):
        return self._core.powered

    def describe(self):
        st = self._core.controller.state()
        w = self._core.world
        return ("simulator '%s' on %s:%d, world '%s' with %d shapes, robot at (%.2f, %.2f), "
                "yaw %.0f deg, body %.2f m above the ground"
                % (self.link.backend, self.link.host, self.link.port, w.name or 'empty',
                   len(w.shapes), st['x'], st['y'], np.degrees(st['yaw']), st['z'] - w.height_at(st['x'], st['y'])))


    def world_summary(self):  # ADD 26-11: the simulated world, for the [SIM] line of the mission log
        """Name of the world and its ground: ramp (grade and direction), hills, bumps, shapes.
        The name alone is not enough: a world file copied from another keeps its name."""
        w = self._core.world
        t = w.terrain or {}
        parts = [f"world '{w.name or 'empty'}'"]
        ramp = t.get('ramp') or {}
        if ramp.get('grade_pct'):
            parts.append(f"ramp {float(ramp['grade_pct']):g}% ({np.degrees(np.arctan(float(ramp['grade_pct']) / 100.0)):.1f} deg) "
                         f"uphill towards {float(ramp.get('dir_deg', 0.0)):g} deg (0 = ahead at the start, 90 = left)")
        if t.get('max_slope_deg'):
            parts.append(f"hills up to {float(t['max_slope_deg']):g} deg (wavelength {float(t.get('wavelength_m', 0)):g} m)")
        if t.get('bumps'):
            parts.append(f"{len(t['bumps'])} bumps")
        if not (ramp.get('grade_pct') or t.get('max_slope_deg') or t.get('bumps')):
            parts.append("flat ground")
        parts.append(f"{len(w.shapes)} obstacles")
        return ", ".join(parts)


def connect(address=''):
    """SimRobot connected to the simulator at 'host:port' (default 127.0.0.1:8765)."""
    host, port = parse_address(address)
    robot = SimRobot(host, port)
    print("[SIM] SIMULATED ROBOT: " + robot.describe() + ".")  # FIX 26-11: was missionLog (not in continuous-motion)
    return robot
