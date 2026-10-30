"""
motion.py -- Spot's motion commands and motion safety.

  relative_move, move_to_world_point_without_turning   one leg, with speed limit, gait
                                                       and stop halfway (should_abort)
  navigate_to                                          one leg of the navigation loop:
                                                       rotate, then advance (with the
                                                       frontier able to stop it)
  select_mobility_params                               gait, step and speed limits from
                                                       the terrain of the edge
  check_fall, recover_from_fall, check_and_recover_fall   falls: recognize them, get up,
                                                       mark the dangerous spot
  stop_robot
"""
import time
import numpy as np
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_se2_a_tform_b)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client import robot_command as _robot_command
from bosdyn.api import robot_state_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import geometry_pb2
from google.protobuf import wrappers_pb2

from spot_explorer import config as P
from spot_explorer.perception import gait
from spot_explorer.records import MISSION_STATS


# =========================================================================
# SPEED LIMITS.
#
# Every motion command sets a limit: without it, Spot walks up to its own maximum
# (~1.6 m/s), too much for outdoor terrain even when flat. The limits per terrain
# level are in gait.GAIT_VEL_* and arrive through select_mobility_params()
# (below). These are the values used when the caller passes none (commands
# isolated, retreat).
#
# CAREFUL: in Spot the limit is a pair (min_vel, max_vel). Setting only
# max_vel, min_vel stays ZERO and the robot can no longer move backwards or
# sideways to the left: the retreat would stop working WITHOUT any error. That is why
# _build_mobility_params always sets both, symmetric.
# =========================================================================
DEFAULT_MAX_LINEAR_VEL_MPS = 0.5
DEFAULT_MAX_ANGULAR_VEL_RPS = 0.6
RETREAT_MAX_LINEAR_VEL_MPS = 0.3     # backwards the robot sees worse: go slower

# =========================================================================
# MAXIMUM TIME OF A MOVE and NETWORK ERRORS.
#
# If Spot cannot get there (body stopped by its collision avoidance, legs slipping
# on a wet slope) it must not stay there trying forever. Maximum
# time = MOVE_TIMEOUT_FACTOR x (theoretical time at the speed limit) +
# MOVE_TIMEOUT_MARGIN_S; once expired, the robot is stopped and the move counts as
# failed. The same time is the command's deadline inside Spot.
#
# A network error while waiting (WiFi outdoors) must not leave the robot finishing
# the trajectory by itself: it retries up to MOVE_MAX_RPC_ERRORS times in a row, then
# stops the robot and re-raises the exception.
# =========================================================================
MOVE_TIMEOUT_FACTOR = 3.0
MOVE_TIMEOUT_MARGIN_S = 10.0
MOVE_MAX_RPC_ERRORS = 3


# =========================================================================
# SPOT FOOT COLLISION AVOIDANCE -- mission default.
#
# `disable_vision_foot_obstacle_avoidance` turns off the check with which Spot,
# autonomously and at low level, avoids PLACING A FOOT on an obstacle
# it sees. It does NOT concern the body: `disable_vision_body_obstacle_avoidance`
# stays at its default (False), so Spot still refuses to
# take the body into an obstacle.
#
# WHY IT STAYS DISABLED (True): Spot's `no_step` classification considers
# GRASS an obstacle -- the very same reason this project uses
# `obstacle_distance` instead of `no_step` (see the comment in
# spot_grid.LocalGrid). With the protection on, on a lawn Spot
# refuses to place its feet and the mission stops. The intended field of use
# of this system is OUTDOOR exploration: indoor tests are an
# expedient, not the use case, so the default follows outdoors.
#
# CONSEQUENCE TO KEEP IN MIND: with this protection off, the only safety
# net under the feet is our safe frontier (safe_frontier), with the stop
# halfway through a move and the retreat. There are no independent checks below: that is why
# unconfirmed legs are never travelled.
#
# For an indoor test where the extra net is wanted, just pass
# disable_foot_obstacle_avoidance=False to the functions below -- without touching
# this default, which must stay the right one for the field.
# =========================================================================
DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT = True


def _velocity_limit(max_linear_vel, max_angular_vel):
    """SYMMETRIC SE2VelocityLimit: see the caveat above DEFAULT_MAX_LINEAR_VEL_MPS."""
    v, w = abs(float(max_linear_vel)), abs(float(max_angular_vel))
    return geometry_pb2.SE2VelocityLimit(
        max_vel=geometry_pb2.SE2Velocity(linear=geometry_pb2.Vec2(x=v, y=v), angular=w),
        min_vel=geometry_pb2.SE2Velocity(linear=geometry_pb2.Vec2(x=-v, y=-v), angular=-w),
    )


def _build_mobility_params(locomotion_hint=None, ground_mu_hint=None, swing_height=None,
                           disable_foot_obstacle_avoidance=None,
                           max_linear_vel=None, max_angular_vel=None):
    """
    Build MobilityParams in a single place, so the motion commands below cannot diverge
    from each other.

    max_linear_vel / max_angular_vel: limits in m/s and rad/s. None = module default.
    The limit is ALWAYS set: without it, Spot uses its own maximum (~1.6 m/s).
    """
    if disable_foot_obstacle_avoidance is None:
        disable_foot_obstacle_avoidance = DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT
    if max_linear_vel is None:
        max_linear_vel = DEFAULT_MAX_LINEAR_VEL_MPS
    if max_angular_vel is None:
        max_angular_vel = DEFAULT_MAX_ANGULAR_VEL_RPS

    obstacle_params = spot_command_pb2.ObstacleParams(
        disable_vision_foot_obstacle_avoidance=bool(disable_foot_obstacle_avoidance)
    )

    mobility_kwargs = {'obstacle_params': obstacle_params,
                       'vel_limit': _velocity_limit(max_linear_vel, max_angular_vel)}
    if locomotion_hint is not None:
        mobility_kwargs['locomotion_hint'] = locomotion_hint
    if swing_height is not None:
        mobility_kwargs['swing_height'] = swing_height
    if ground_mu_hint is not None:
        mobility_kwargs['terrain_params'] = spot_command_pb2.TerrainParams(
            ground_mu_hint=wrappers_pb2.DoubleValue(value=ground_mu_hint)
        )

    return spot_command_pb2.MobilityParams(**mobility_kwargs)


# =========================================================================
# FALLS -- detection and self-righting.
#
# A fall is recognized if the robot state reports a behaviour fault with
# cause CAUSE_FALL, OR if the body is tilted beyond FALL_TILT_DEG (this also holds if the
# fault is absent, for example a robot overturned halfway through a self-right).
#
# The recovery: clears the clearable faults, self-rights (selfright), checks that the body
# is back within UPRIGHT_TILT_DEG, stands up again, checks that it is really
# standing. At most SELF_RIGHT_MAX_ATTEMPTS attempts.
#
# NOTHING is attempted -- and RobotFallenError is raised -- if a fault is NOT clearable or
# if the motors are off (E-Stop, failure): in those cases the operator is needed, and a robot
# on the ground must not try to do anything else by itself.
# =========================================================================
FALL_TILT_DEG = 50.0
UPRIGHT_TILT_DEG = 20.0
SELF_RIGHT_TIMEOUT_S = 30.0
STAND_TIMEOUT_S = 10.0
SELF_RIGHT_MAX_ATTEMPTS = 2


class RobotFallenError(RuntimeError):
    """The robot fell and could not get up by itself: the operator is needed."""


def _roll_pitch_deg(q):
    roll = np.degrees(np.arctan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)))
    sinp = np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
    return float(roll), float(np.degrees(np.arcsin(sinp)))


def check_fall(robot_state_client):
    """
    Read the robot state and tell whether it has fallen. Returns a dict:
      fallen, reason, roll, pitch, x, y, fall_faults (id), clearable (id), unclearable (id),
      motors_on, standing
    """
    state = robot_state_client.get_robot_state()
    BF = robot_state_pb2.BehaviorFault
    faults = list(state.behavior_fault_state.faults)
    fall_faults = [f.behavior_fault_id for f in faults if f.cause == BF.CAUSE_FALL]
    clearable = [f.behavior_fault_id for f in faults if f.status == BF.STATUS_CLEARABLE]
    unclearable = [f.behavior_fault_id for f in faults if f.status == BF.STATUS_UNCLEARABLE]

    vision_tform_body = get_a_tform_b(state.kinematic_state.transforms_snapshot,
                                      VISION_FRAME_NAME, BODY_FRAME_NAME)
    roll, pitch = _roll_pitch_deg(vision_tform_body.rotation)
    tilted = max(abs(roll), abs(pitch)) > FALL_TILT_DEG

    PS = robot_state_pb2.PowerState
    motors_on = state.power_state.motor_power_state == PS.MOTOR_POWER_STATE_ON
    standing = state.behavior_state.state in (robot_state_pb2.BehaviorState.STATE_STANDING,
                                              robot_state_pb2.BehaviorState.STATE_STEPPING)
    reasons = []
    if fall_faults:
        reasons.append(f"fall fault ({len(fall_faults)})")
    if tilted:
        reasons.append(f"body tilted (roll {roll:.0f}, pitch {pitch:.0f} deg)")
    return dict(fallen=bool(fall_faults) or tilted, reason=", ".join(reasons), roll=roll, pitch=pitch,
                x=vision_tform_body.position.x, y=vision_tform_body.position.y,
                fall_faults=fall_faults, clearable=clearable, unclearable=unclearable,
                motors_on=motors_on, standing=standing)


def recover_from_fall(robot_command_client, robot_state_client, status=None):
    """
    If the robot has fallen, try to get it up. Returns True if in the end it is standing
    (also if it had not fallen). Raises RobotFallenError if the operator is needed.
    """
    st = status or check_fall(robot_state_client)
    if not st['fallen']:
        return True
    print(f"[FALL] Robot on the ground at ({st['x']:.2f}, {st['y']:.2f}): {st['reason']}.")

    for attempt in range(1, SELF_RIGHT_MAX_ATTEMPTS + 1):
        if st['unclearable']:
            raise RobotFallenError(
                f"fault not clearable {st['unclearable']}: the robot cannot get up by itself. "
                f"Operator intervention needed.")
        if not st['motors_on']:
            raise RobotFallenError(
                "motors off after the fall (E-Stop or failure): the robot cannot get up by itself. "
                "Operator intervention needed.")
        try:
            for fid in st['clearable']:
                robot_command_client.clear_behavior_fault(fid)
            print(f"[FALL] Attempt {attempt}/{SELF_RIGHT_MAX_ATTEMPTS}: cleared {len(st['clearable'])} "
                  f"faults, self-righting in progress...")
            _robot_command.blocking_selfright(robot_command_client, timeout_sec=SELF_RIGHT_TIMEOUT_S)
            st = check_fall(robot_state_client)
            if max(abs(st['roll']), abs(st['pitch'])) > UPRIGHT_TILT_DEG:
                print(f"[FALL] After self-righting the body is still tilted "
                      f"(roll {st['roll']:.0f}, pitch {st['pitch']:.0f} deg).")
                continue
            for fid in st['clearable']:
                robot_command_client.clear_behavior_fault(fid)
            _robot_command.blocking_stand(robot_command_client, timeout_sec=STAND_TIMEOUT_S)
            st = check_fall(robot_state_client)
            if not st['fallen'] and st['standing']:
                print(f"[FALL] Got up at attempt {attempt}: standing again at "
                      f"({st['x']:.2f}, {st['y']:.2f}).")
                return True
            print(f"[FALL] After the stand command the robot is not standing ({st['reason'] or 'state'}).")
        except RobotFallenError:
            raise
        except Exception as e:
            print(f"[FALL] Attempt {attempt} failed: {e}")
            st = check_fall(robot_state_client)
    raise RobotFallenError(f"{SELF_RIGHT_MAX_ATTEMPTS} self-righting attempts failed. "
                           f"Operator intervention needed.")


def stop_robot(robot_command_client, reason=""):
    """
    Stop the robot immediately, cancelling the current trajectory.

    Spot stays standing and under control: it is not an emergency stop (that is what the
    E-Stop is for), it is the equivalent of releasing the stick. After this call another
    move can be commanded at once -- for example a backwards retreat.
    """
    try:
        robot_command_client.robot_command(RobotCommandBuilder.stop_command(),
                                           end_time_secs=time.time() + 5.0)
        if reason:
            print(f"[STOP] Move interrupted: {reason}")
        else:
            print("[STOP] Move interrupted.")
        return True
    except Exception as e:
        print(f"[STOP] Could not stop the robot: {e}")
        return False


def relative_move(dx, dy, dyaw, frame_name, robot_command_client, robot_state_client,
                   locomotion_hint=None, ground_mu_hint=None, swing_height=None,
                   disable_foot_obstacle_avoidance=None,
                   should_abort=None, poll_period_s=0.2,
                   max_linear_vel=None, max_angular_vel=None):
    """Move the robot relative to the current pose.

    dx, dy, dyaw are in the BODY frame: dx>0 forward, dx<0 backward, dy sideways,
    dyaw rotation. Spot walks in all directions, so a negative dx with dyaw=0
    is a true backwards walk, without needing to turn around.

    Args:
        locomotion_hint: optional spot_command_pb2.LocomotionHint (e.g. HINT_CRAWL), the gait.
        ground_mu_hint: optional expected friction coefficient (e.g. 0.4-0.8), in terrain_params.
        swing_height: optional spot_command_pb2.SwingHeight (e.g. SWING_HEIGHT_HIGH).
        disable_foot_obstacle_avoidance: if True turns off Spot's FOOT collision avoidance
            (needed on grass, see DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT at the top of the file).
            None = use the mission default.
        should_abort: callable without arguments, queried at every feedback cycle.
            If it returns a true value the move is STOPPED at once and the function
            returns (False, distance_travelled). That is how the safe frontier, running in a
            thread, stops the robot halfway through a move. The REASON for the stop is known
            to the caller, who owns the callback.
        poll_period_s: interval between two feedback readings. At ~0.5 m/s, 0.2 s means
            ~10 cm travelled between one check and the next.
        max_linear_vel, max_angular_vel: speed limits (m/s, rad/s). None = module default
            (DEFAULT_MAX_*). See the caveat at the top of the file about the minimum.

    Returns:
        (success, distance_traveled): success True if the goal is reached, False if the
        move failed or was interrupted; distance_traveled in metres.
    """
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Starting position
    initial_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    initial_x = initial_tform_body.x
    initial_y = initial_tform_body.y

    # Goal in the requested frame: current body pose composed with the displacement.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    mobility_params = _build_mobility_params(
        locomotion_hint=locomotion_hint, ground_mu_hint=ground_mu_hint,
        swing_height=swing_height,
        disable_foot_obstacle_avoidance=disable_foot_obstacle_avoidance,
        max_linear_vel=max_linear_vel, max_angular_vel=max_angular_vel)

    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
        frame_name=frame_name, params=mobility_params)
    v_lim = abs(float(max_linear_vel if max_linear_vel is not None else DEFAULT_MAX_LINEAR_VEL_MPS)) or 0.1
    w_lim = abs(float(max_angular_vel if max_angular_vel is not None else DEFAULT_MAX_ANGULAR_VEL_RPS)) or 0.1
    expected_s = float(np.hypot(dx, dy)) / v_lim + abs(float(dyaw)) / w_lim
    timeout_s = MOVE_TIMEOUT_FACTOR * expected_s + MOVE_TIMEOUT_MARGIN_S
    t_start = time.time()
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=t_start + timeout_s)

    # Wait: arrival, failure, timeout or stop requested by the caller.
    rpc_errors = 0
    distance_traveled = 0.0
    while True:
        if time.time() - t_start > timeout_s:
            stop_robot(robot_command_client,
                       reason=f"time limit {timeout_s:.0f} s exceeded (travelled {distance_traveled:.2f}m)")
            return False, distance_traveled
        try:
            feedback = robot_command_client.robot_command_feedback(cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

            # Current position
            current_state = robot_state_client.get_robot_state()
            current_transforms = current_state.kinematic_state.transforms_snapshot
            current_tform_body = get_se2_a_tform_b(current_transforms, frame_name, BODY_FRAME_NAME)
            rpc_errors = 0
        except Exception as e:
            rpc_errors += 1
            print(f"[MOVE] Communication error during the move ({rpc_errors}/"
                  f"{MOVE_MAX_RPC_ERRORS}): {e}")
            if rpc_errors >= MOVE_MAX_RPC_ERRORS:
                stop_robot(robot_command_client, reason="repeated communication errors")
                raise
            time.sleep(poll_period_s)
            continue

        distance_traveled = np.sqrt((current_tform_body.x - initial_x) ** 2 +
                                    (current_tform_body.y - initial_y) ** 2)

        # Stop requested by the caller (the safe frontier saw something on the leg).
        # A callback that raises an exception is ignored: the move continues.
        if should_abort is not None:
            try:
                abort_now = bool(should_abort())
            except Exception as e:
                print(f"[STOP] Error in the stop condition, ignoring it: {e}")
                abort_now = False
            if abort_now:
                stop_robot(robot_command_client,
                           reason=f"requested by the caller after {distance_traveled:.2f}m")
                return False, distance_traveled

        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print(f"[MOVE] Goal not reached: travelled {distance_traveled:.2f} m out of "
                  f"{float(np.hypot(dx, dy)):.2f} requested (command status "
                  f"{mobility_feedback.status}).")
            return False, distance_traveled

        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            # After a rotation in place the few cm travelled are the body centre moving
            # while turning, not an advance: the message says so.
            if abs(float(dx)) < 1e-6 and abs(float(dy)) < 1e-6:
                turned = float(np.degrees(np.arctan2(
                    np.sin(current_tform_body.angle - initial_tform_body.angle),
                    np.cos(current_tform_body.angle - initial_tform_body.angle))))
                print(f"[MOVE] Rotation completed: {turned:+.0f} deg out of "
                      f"{np.degrees(float(dyaw)):+.0f} requested (the body centre moved "
                      f"{distance_traveled:.2f} m while turning).")
            else:
                print(f"[MOVE] Arrived: travelled {distance_traveled:.2f} m out of "
                      f"{float(np.hypot(dx, dy)):.2f} requested.")
            return True, distance_traveled

        time.sleep(poll_period_s)


def move_to_world_point_without_turning(target_x, target_y, frame_name,
                                        robot_command_client, robot_state_client,
                                        locomotion_hint=None, ground_mu_hint=None,
                                        swing_height=None, disable_foot_obstacle_avoidance=None,
                                        should_abort=None,
                                        max_linear_vel=None, max_angular_vel=None):
    """
    Reach a world point KEEPING the current heading: Spot gets there walking backwards or
    sideways, as needed.
    Default speed: RETREAT_MAX_LINEAR_VEL_MPS.

    Used for the retreat over its own steps: the points it came through are behind the
    robot, and in a tight space there is no way to turn around to face them nose first.
    The delta is converted from the world frame to the body frame, so the commanded
    rotation stays exactly zero.
    """
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)

    dx_world = target_x - out_tform_body.x
    dy_world = target_y - out_tform_body.y

    cos_a, sin_a = np.cos(out_tform_body.angle), np.sin(out_tform_body.angle)
    dx_body = dx_world * cos_a + dy_world * sin_a
    dy_body = -dx_world * sin_a + dy_world * cos_a

    if max_linear_vel is None:
        max_linear_vel = RETREAT_MAX_LINEAR_VEL_MPS
    return relative_move(dx_body, dy_body, 0.0, frame_name, robot_command_client, robot_state_client,
                         locomotion_hint=locomotion_hint, ground_mu_hint=ground_mu_hint,
                         swing_height=swing_height,
                         disable_foot_obstacle_avoidance=disable_foot_obstacle_avoidance,
                         should_abort=should_abort,
                         max_linear_vel=max_linear_vel, max_angular_vel=max_angular_vel)


# =========================================================================================
# ONE MISSION LEG: gait, move with stop check, falls
# =========================================================================================
def check_and_recover_fall(robot_state_client, command_client, global_map, where):
    """
    Check whether the robot has fallen and, if so, get it up (recover_from_fall) and mark
    the spot as dangerous in the global map, so the planner avoids it.

    Returns None if there was no fall, otherwise (x, y) of the point where the robot fell,
    with the robot standing again. Raises RobotFallenError if it cannot get up by itself:
    the mission stops and the operator is needed.
    """
    try:
        st = check_fall(robot_state_client)
    except Exception as e:
        print(f"[FALL] Could not read the robot state ({where}): {e}")
        return None
    if not st['fallen']:
        return None
    MISSION_STATS['falls'] += 1
    print(f"[FALL] Detected {where} (fall no. {MISSION_STATS['falls']} of the mission).")
    recover_from_fall(command_client, robot_state_client, status=st)
    MISSION_STATS['self_rights'] += 1
    if MISSION_STATS['falls'] >= P.MAX_FALLS_PER_MISSION:
        raise RobotFallenError(
            f"{MISSION_STATS['falls']} falls in this mission: the robot got up but the mission "
            f"stops here. Check the terrain and the robot before starting again.")
    if global_map is not None:
        n = global_map.mark_hazard(st['x'], st['y'], P.FALL_HAZARD_RADIUS_M)
        print(f"[FALL] Fall point ({st['x']:.2f}, {st['y']:.2f}) marked as dangerous "
              f"in the map: disk of {P.FALL_HAZARD_RADIUS_M:.1f} m ({n} cells), the planner will avoid it.")
    return (st['x'], st['y'])


def select_mobility_params(gradient, roughness):
    """
    Gait on three levels, chosen from the terrain difficulty of the SINGLE leg about to be
    travelled (see roadmap.PRM.sample_terrain_between_points), instead of a fixed profile
    for the whole mission.

    The worse of slope and roughness decides (exceeding the threshold on ONE of the two is
    enough to move to the harder level) -- the same OR logic as the obstacle veto in
    spot_grid.LocalGrid.fuse_obstacle_mask. Thresholds, frictions and speeds are in gait.GAIT_*.

    Levels:
      PLAIN    -- flat, regular terrain: normal gait (HINT_AUTO), medium step,
                  high ground_mu_hint (good grip)
      MODERATE -- some slope or roughness: CRAWL, high step, medium ground_mu_hint
      HARD     -- difficult but not forbidden terrain (the veto is a separate, much
                  higher threshold, in fuse_obstacle_mask): CRAWL, high step, low
                  ground_mu_hint (less grip, caution)
    """
    # Tuned for OUTDOORS: even a flat lawn is not a floor, so at least a medium
    # step for all levels, and limited speeds at every level.
    if gradient < gait.GAIT_PLAIN_SLOPE_MAX and roughness < gait.GAIT_PLAIN_ROUGH_MAX:
        tier = "PLAIN"
        params = dict(locomotion_hint=spot_command_pb2.HINT_AUTO,
                      swing_height=spot_command_pb2.SWING_HEIGHT_MEDIUM,
                      ground_mu_hint=gait.GAIT_MU_PLAIN,
                      max_linear_vel=gait.GAIT_VEL_PLAIN,
                      max_angular_vel=gait.GAIT_ANG_PLAIN)
    elif gradient < gait.GAIT_MODERATE_SLOPE_MAX and roughness < gait.GAIT_MODERATE_ROUGH_MAX:
        tier = "MODERATE"
        # CRAWL here too, at the user's request. On grass MODERATE takes ~60% of the
        # steps, so crawl is the prevailing gait and the mission is slower;
        # with HINT_AUTO here MODERATE goes back to the normal gait.
        params = dict(locomotion_hint=spot_command_pb2.HINT_CRAWL,
                      swing_height=spot_command_pb2.SWING_HEIGHT_HIGH,
                      ground_mu_hint=gait.GAIT_MU_MODERATE,
                      max_linear_vel=gait.GAIT_VEL_MODERATE,
                      max_angular_vel=gait.GAIT_ANG_MODERATE)
    else:
        tier = "HARD"
        params = dict(locomotion_hint=spot_command_pb2.HINT_CRAWL,
                      swing_height=spot_command_pb2.SWING_HEIGHT_HIGH,
                      ground_mu_hint=gait.GAIT_MU_HARD,
                      max_linear_vel=gait.GAIT_VEL_HARD,
                      max_angular_vel=gait.GAIT_ANG_HARD)

    print(f"[GAIT] Terrain level: {tier} (slope={gradient:.3f}, roughness={roughness:.3f}) -> "
          f"locomotion_hint={params['locomotion_hint']}, swing_height={params['swing_height']}, "
          f"ground_mu_hint={params['ground_mu_hint']}, "
          f"vel_max={params['max_linear_vel']:.2f} m/s, rot_max={params['max_angular_vel']:.2f} rad/s")
    MISSION_STATS['gait_tier_counts'][tier] += 1
    MISSION_STATS['last_tier'] = tier
    return params


def navigate_to(target_x, target_y, robot_x, robot_y, robot_state_client, command_client, vision_tform_body,
                 should_abort=None, keep_heading=False, **mobility_kwargs):
    """
    Take the robot to the point (target_x, target_y): first rotate towards the point, then
    walk straight (two calls to relative_move).

    keep_heading: if True the robot does NOT rotate before moving, it reaches the point with
        the heading it has (even sideways or backwards). The navigation loop always passes
        False: sideways Spot takes 1.10 m instead of 0.50 and the frontier requires
        0.59 m of obstacle_distance instead of 0.30 (see ROTATION AND BACK-OFF in config.py).
        The parameter stays for the tests.

    mobility_kwargs: optional gait passed to relative_move, e.g.
        locomotion_hint=spot_command_pb2.HINT_CRAWL,
        ground_mu_hint=0.4,
        swing_height=spot_command_pb2.SWING_HEIGHT_HIGH

    Returns:
        (success: bool, distance_traveled: float, distance_commanded: float)

    relative_move() returns a TUPLE (success, distance travelled): the boolean is extracted
    here, because a non-empty tuple in Python is always true and a failed move would look
    successful.
    """
    dx, dy = target_x - robot_x, target_y - robot_y
    distance = np.sqrt(dx ** 2 + dy ** 2)
    target_yaw = np.arctan2(dy, dx)

    quat = vision_tform_body.rotation
    current_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2))
    dyaw = np.arctan2(np.sin(target_yaw - current_yaw), np.cos(target_yaw - current_yaw))

    if keep_heading:
        print(f"[INFO] Reaching the point without rotating ({distance:.2f} m, "
              f"{np.degrees(dyaw):.0f} deg from the nose)...")
        move_success, distance_traveled = move_to_world_point_without_turning(
            target_x, target_y, "vision", command_client, robot_state_client,
            should_abort=should_abort, **mobility_kwargs)
    else:
        rotate_success, _ = relative_move(0, 0, dyaw, "vision", command_client, robot_state_client,
                                                    should_abort=should_abort, **mobility_kwargs)
        if not rotate_success:
            print(f"[FAIL] Rotation failed: the forward move does not start.")
            return False, 0.0, distance

        move_success, distance_traveled = relative_move(distance, 0, 0, "vision", command_client,
                                                                   robot_state_client,
                                                                   should_abort=should_abort, **mobility_kwargs)

    # Plausibility check: relative_move() can declare success (trajectory
    # "settled") having travelled almost nothing of the requested distance -- for example with
    # Spot stopped against a physical obstacle, all the more since by default the foot
    # collision avoidance is off (DISABLE_FOOT_OBSTACLE_AVOIDANCE_DEFAULT). A success with
    # an almost null advance on a real move counts as a failure.
    MIN_PROGRESS_FRACTION = 0.5
    MIN_PROGRESS_ABS_M = 0.05
    required_progress = max(MIN_PROGRESS_ABS_M, MIN_PROGRESS_FRACTION * distance)

    if move_success and distance > MIN_PROGRESS_ABS_M and distance_traveled < required_progress:
        print(f"[FAIL] Move reported as successful but only {distance_traveled:.2f}m travelled "
              f"out of {distance:.2f}m requested -- treated as a real mechanical failure (probably "
              f"an undetected physical obstacle).")
        move_success = False

    return move_success, distance_traveled, distance
