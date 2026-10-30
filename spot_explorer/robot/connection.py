"""Small shared helpers: robot position in the VISION frame, line of sight on the grid."""
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME, ODOM_FRAME_NAME
import numpy as np

def getPosition(robot_state_client):
    robot_state = robot_state_client.get_robot_state()
    transforms_snapshot = robot_state.kinematic_state.transforms_snapshot
    vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)

    return vision_tform_body.position.x, vision_tform_body.position.y, vision_tform_body.position.z, vision_tform_body.rotation

def check_line_of_sight(x1, y1, x2, y2, pts, cells, obstacle_threshold=0.0, max_valid_dist=0.2):
    """
    Line of sight between two points, sampled every 10 cm (at least 10 samples) on the grid
    (pts, cells), with cells = obstacle_distance: below obstacle_threshold is an obstacle.
    Returns 'blocked' at the first sample on an obstacle, 'unseen' if a sample has no grid
    point within max_valid_dist, otherwise 'clear'.
    """
    distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    num_checks = max(10, int(distance * 10))

    for i in range(num_checks):
        t = i / max(1, num_checks - 1)
        check_x = x1 + t * (x2 - x1)
        check_y = y1 + t * (y2 - y1)

        # Nearest grid point
        distances = np.sqrt((pts[:, 0] - check_x) ** 2 + (pts[:, 1] - check_y) ** 2)
        nearest_idx = np.argmin(distances)
        min_dist = distances[nearest_idx]

        if min_dist > max_valid_dist:
            return 'unseen'

        if cells[nearest_idx] < obstacle_threshold:
            return 'blocked'

    return 'clear'


# --- robot connection ---
"""Connection to the robot (SDK, login, lease, state) and E-Stop."""
import os
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive

def setLogInfo(options):
    bosdyn.client.util.setup_logging(options.verbose)
    sdk = bosdyn.client.create_standard_sdk(options.name)
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    session_name = options.recording_session_name
    if session_name == '':
        session_name = os.path.basename(
            '/Users/gianmariagennai/Documents/Unifi/Magistrale/spot/autowalk/TestAuto'
        )
    user_name = options.recording_user_name
    if user_name == '':
        user_name = robot._current_user
    client_metadata = GraphNavRecordingServiceClient.make_client_metadata(
        session_name=session_name,
        client_username=user_name,
        client_id='RecordingClient',
        client_type='Python SDK',
    )
    return robot, lease_client, robot_state_client, client_metadata

class SimpleEstop:
    """
    Software E-Stop without a UI. On creation it registers and lets the robot move;
    stop() stops it, allow() releases it.
    """

    def __init__(self, robot, name='simple_estop'):
        self.estop_client = robot.ensure_client(EstopClient.default_service_name)
        self.endpoint = EstopEndpoint(self.estop_client, name, estop_timeout=9.0)
        self.endpoint.force_simple_setup()
        self.keepalive = EstopKeepAlive(self.endpoint)
        self.keepalive.allow()
        print(f"[OK] E-Stop '{name}' activated and released")

    def stop(self):
        """Engage the E-Stop: the robot stops at once."""
        self.keepalive.stop()
        print("[ESTOP] Robot stopped")

    def allow(self):
        """Release the E-Stop: the robot may move."""
        self.keepalive.allow()
        print("[ESTOP] Robot free to move")
