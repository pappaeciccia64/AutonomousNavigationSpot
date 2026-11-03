#!/usr/bin/env python3
"""
GAIT TRIAL ON A TRACK: the same straight stretch walked several times with a FIXED gait
configuration, measuring how it goes. It serves to tune the gait levels (perception/gait.py
GAIT_*, motion.select_mobility_params), which the mission logs cannot give: in a mission every
terrain saw only one configuration.

    python3 tools/gait_trial.py --terrain ramp15 --true-slope 14.5 --config crawl_mu06 --dist 5
    python3 tools/gait_trial.py --terrain lawn --config auto_mu08 --repetitions 5 --back-turn
    python3 tools/gait_trial.py --terrain ramp15 --true-slope 14.5 --estimate-only
    python3 tools/gait_trial.py --list          # the configurations available

The robot goes at the START of the track, nose along the track, with free ground for --dist
metres ahead (and behind, with --back-turn, because it turns on the spot at the end). Before
every pass the script waits for Enter: check the track is free and bring the robot back to the
start if needed (or use --back-turn, or --backwards where there is no room to turn: Spot, ~1.1 m
long, needs a circle of ~1.6 m to rotate). E-Stop from the tablet always at hand: here there is
no safe frontier of the mission, the robot goes straight and that is all (Spot's foot collision
avoidance stays off as in the mission, unless --protected-feet; the body one stays on).

For every pass, a row in GaitTrials/trials.csv:
  - terrain, direction (out/back), configuration, slope measured with the inclinometer;
  - slope and roughness ESTIMATED from the scan before starting: with the PRM computation (with
    the body, 90th percentile) and with the local plane (slopes.PLANE_SLOPES), and the level the
    mission would have chosen;
  - success, time, commanded and travelled distance, speed;
  - pitch and roll: mean, peak, and RMS around the mean (the oscillation: on a ramp the mean
    pitch is the slope, the RMS is how much the robot "sways");
  - ground slope under the feet from Spot's legs (frame 'gpe'): mean along and across;
  - foot slip (Spot's foot_slip_distance, foot in contact: peak and mean) and friction estimated
    by Spot under the feet (ground_mu_est), the measure to compare ground_mu_hint with;
  - energy (battery voltage x current, integrated) and charge before and after.
The sampled trajectory goes in GaitTrials/<terrain>_<config>_<time>.npz.

python3 tools/calibration.py reads trials.csv and compares by terrain and configuration.

Ported on 03-11-2026 from prova_andatura.py of the AdL-version branch, never tried on the robot
either: the first time indoors, on 2 m of flat floor (FIELD_TEST.md, "Gait calibration").
"""
import argparse
import csv
import os
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import bosdyn.client.lease                                                   # noqa: E402
from bosdyn.api import robot_state_pb2                                       # noqa: E402
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2            # noqa: E402
from bosdyn.client.frame_helpers import VISION_FRAME_NAME                    # noqa: E402
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, blocking_sit   # noqa: E402

from spot_explorer.perception import gait, slopes, spot_grid                 # noqa: E402
from spot_explorer.planning.edge_evaluation import body_slope                # noqa: E402
from spot_explorer.records import MISSION_STATS, _ground_plane_slopes_deg, _quat_to_roll_pitch_deg  # noqa: E402
from spot_explorer.robot import connection, motion                           # noqa: E402

# Configurations to compare. The first three are today's levels (select_mobility_params); the
# others isolate one factor at a time (gait, step, friction). 'mission' chooses like the
# mission, from the slope estimated before starting.
AUTO, CRAWL = spot_command_pb2.HINT_AUTO, spot_command_pb2.HINT_CRAWL
MED, HIGH = spot_command_pb2.SWING_HEIGHT_MEDIUM, spot_command_pb2.SWING_HEIGHT_HIGH
CONFIG = {
    'auto_mu08':      dict(locomotion_hint=AUTO, swing_height=MED, ground_mu_hint=0.8,
                           max_linear_vel=gait.GAIT_VEL_PLAIN, max_angular_vel=gait.GAIT_ANG_PLAIN),
    'crawl_mu06':     dict(locomotion_hint=CRAWL, swing_height=HIGH, ground_mu_hint=0.6,
                           max_linear_vel=gait.GAIT_VEL_MODERATE, max_angular_vel=gait.GAIT_ANG_MODERATE),
    'crawl_mu04':     dict(locomotion_hint=CRAWL, swing_height=HIGH, ground_mu_hint=0.4,
                           max_linear_vel=gait.GAIT_VEL_HARD, max_angular_vel=gait.GAIT_ANG_HARD),
    'auto_high_mu06': dict(locomotion_hint=AUTO, swing_height=HIGH, ground_mu_hint=0.6,
                           max_linear_vel=gait.GAIT_VEL_MODERATE, max_angular_vel=gait.GAIT_ANG_MODERATE),
    'auto_high_mu04': dict(locomotion_hint=AUTO, swing_height=HIGH, ground_mu_hint=0.4,
                           max_linear_vel=gait.GAIT_VEL_HARD, max_angular_vel=gait.GAIT_ANG_HARD),
    'crawl_mu08':     dict(locomotion_hint=CRAWL, swing_height=HIGH, ground_mu_hint=0.8,
                           max_linear_vel=gait.GAIT_VEL_MODERATE, max_angular_vel=gait.GAIT_ANG_MODERATE),
}
DESCRIPTION = {
    'auto_mu08': "= today's PLAIN (normal gait, medium step, friction 0.8)",
    'crawl_mu06': "= today's MODERATE (crawl, high step, friction 0.6)",
    'crawl_mu04': "= today's HARD (crawl, high step, friction 0.4)",
    'auto_high_mu06': "normal gait with high step and friction 0.6 (MODERATE without crawl)",
    'auto_high_mu04': "normal gait with high step and friction 0.4 (HARD without crawl)",
    'crawl_mu08': "crawl with friction 0.8 (separates the effect of the crawl from that of the friction)",
    'mission': "like the mission: level chosen from the slope estimated before starting",
}
FOLDER = 'GaitTrials'
COLUMNS = ['timestamp', 'terrain', 'direction', 'config', 'repetition', 'true_slope_deg',
           'est_long', 'est_lat', 'est_rough', 'est_has_data', 'est_plane_long_deg', 'est_plane_lat_deg',
           'mission_level', 'locomotion_hint', 'swing_height', 'ground_mu_hint', 'max_linear_vel',
           'success', 'duration_s', 'dist_commanded_m', 'dist_traveled_m', 'speed_mps',
           'pitch_mean_deg', 'pitch_peak_abs_deg', 'pitch_rms_deg',
           'roll_mean_deg', 'roll_peak_abs_deg', 'roll_rms_deg',
           'gpe_long_mean_deg', 'gpe_lat_mean_deg',
           'slip_peak_m', 'slip_mean_m', 'mu_est_mean', 'mu_est_min',
           'energy_j', 'energy_per_m_j', 'charge_start_pct', 'charge_end_pct',
           'samples', 'note']
SAMPLE_COLUMNS = ['t', 'x', 'y', 'yaw', 'roll', 'pitch', 'slip_max', 'mu_est', 'power_w', 'gpe_long', 'gpe_lat']
# How far ahead of the robot the scan sees (the local window is 3.84 m, robot at the centre).
ESTIMATE_MAX_M = 1.8


class Sampler:
    """Robot state every period_s in a thread: pose, tilt, ground plane, feet, power."""

    def __init__(self, robot_state_client, period_s=0.1):
        self._c = robot_state_client
        self._period = period_s
        self._stop = threading.Event()
        self._thread = None
        self.rows = []

    def _one(self):
        try:
            st = self._c.get_robot_state()
        except Exception:
            return
        t = time.time()
        try:
            snap = st.kinematic_state.transforms_snapshot
            x, y, _, q = connection.getPosition(self._c)
            roll, pitch = _quat_to_roll_pitch_deg(q)
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
            g_long, g_lat = _ground_plane_slopes_deg(snap, yaw)
        except Exception:
            return
        slip, mu = [], []
        try:
            for fs in st.foot_state:
                if fs.contact != robot_state_pb2.FootState.CONTACT_MADE:
                    continue
                d = fs.terrain.foot_slip_distance_rt_frame
                slip.append(float(np.sqrt(d.x * d.x + d.y * d.y + d.z * d.z)))
                if fs.terrain.ground_mu_est > 0:
                    mu.append(float(fs.terrain.ground_mu_est))
        except Exception:
            pass
        power = np.nan
        try:
            b = st.battery_states[0]
            if b.HasField('voltage') and b.HasField('current'):
                power = abs(float(b.voltage.value) * float(b.current.value))
        except Exception:
            pass
        self.rows.append((t, x, y, yaw, roll, pitch, max(slip) if slip else np.nan,
                          float(np.mean(mu)) if mu else np.nan, power, g_long, g_lat))

    def _run(self):
        while not self._stop.wait(self._period):
            self._one()

    def start(self):
        self._one()
        self._thread = threading.Thread(target=self._run, name='gait_trial_sampler', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._one()
        return np.array(self.rows, dtype=float) if self.rows else np.zeros((0, len(SAMPLE_COLUMNS)))


def charge(robot_state_client):
    try:
        return float(robot_state_client.get_robot_state().battery_states[0].charge_percentage.value)
    except Exception:
        return np.nan


def estimate_track(local_grid, robot_state_client, dist):
    """Slope (with the body) and roughness of the track ahead of the robot, from the scan of now,
    with the PRM computation and with the local plane. Sees only the first ESTIMATE_MAX_M metres."""
    empty = dict(long=np.nan, lat=np.nan, rough=np.nan, has=False, plane_long=np.nan, plane_lat=np.nan)
    try:
        scan = local_grid.fetch_scan(robot_state_client)
        if scan is None:
            print("[ESTIMATE] Scan not arrived.")
            return empty
        x, y, _, q = connection.getPosition(robot_state_client)
        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        local_grid.process_scan(scan, x, y, yaw)
        shape = (scan.num_y, scan.num_x)
        terrain_2d = scan.terrain.reshape(shape)
        usable = np.asarray(spot_grid.slope_usable_mask(scan.is_valid, scan.od, shape), dtype=bool).reshape(shape)
        ox, oy = local_grid.grid_origin(scan)
        L = min(dist, ESTIMATE_MAX_M)
        x2, y2 = x + L * np.cos(yaw), y + L * np.sin(yaw)
        prof = slopes.compute_arc_slope_profile(terrain_2d, ox, oy, scan.cell_size, x, y, x2, y2,
                                                valid_2d=usable, body_ext_m=slopes.ARC_BODY_EXTENSION_M)
        lo, la, has = body_slope(prof)
        gx, gy, _ = slopes.plane_gradient_field(np.where(usable, terrain_2d, np.nan), usable, scan.cell_size,
                                                half_window_m=slopes.PLANE_EDGE_HALF_WINDOW_M)
        plo, pla, phas = slopes.plane_slope_along(gx, gy, ox, oy, scan.cell_size, x, y, x2, y2)
        t = np.linspace(0.0, 1.0, max(2, int(L / scan.cell_size) + 1))
        r, inside = slopes._gather_heights(np.asarray(scan.rough).reshape(shape), ox, oy, scan.cell_size,
                                           x + t * (x2 - x), y + t * (y2 - y), valid_2d=usable)
        rough = float(np.nanmean(r[inside])) if inside.any() else np.nan
        deg = lambda v: float(np.degrees(np.arctan(v)))
        print(f"[ESTIMATE] First {L:.1f} m: slope long {lo:.3f} ({deg(lo):.1f}°), lat {la:.3f} ({deg(la):.1f}°), "
              f"roughness {rough:.3f}, {'with data' if has else 'NOT ENOUGH DATA'} | local plane: long "
              f"{deg(plo):.1f}°, lat {deg(pla):.1f}°{'' if phas else ' (not enough data)'}")
        return dict(long=float(lo), lat=float(la), rough=rough, has=bool(has),
                    plane_long=deg(plo) if phas else np.nan, plane_lat=deg(pla) if phas else np.nan)
    except Exception as e:
        print(f"[ESTIMATE] Estimate failed: {e}")
        return empty


def summary(a, dist_trav, duration):
    """From the sampler rows to the numbers of the trials.csv row."""
    if len(a) == 0:
        return {}
    roll, pitch, slip, mu, pw, g_long, g_lat = a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8], a[:, 9], a[:, 10]
    nm = lambda v, f: float(f(v[np.isfinite(v)])) if np.isfinite(v).any() else np.nan
    energy = np.nan
    if np.isfinite(pw).sum() >= 2:
        ok = np.isfinite(pw)
        p_, t_ = pw[ok], a[ok, 0]
        energy = float(np.sum(0.5 * (p_[1:] + p_[:-1]) * np.diff(t_)))
    return {
        'pitch_mean_deg': nm(pitch, np.mean), 'pitch_peak_abs_deg': nm(np.abs(pitch), np.max),
        'pitch_rms_deg': nm(pitch - np.nanmean(pitch), lambda v: np.sqrt(np.mean(v * v))),
        'roll_mean_deg': nm(roll, np.mean), 'roll_peak_abs_deg': nm(np.abs(roll), np.max),
        'roll_rms_deg': nm(roll - np.nanmean(roll), lambda v: np.sqrt(np.mean(v * v))),
        'gpe_long_mean_deg': nm(g_long, np.mean), 'gpe_lat_mean_deg': nm(g_lat, np.mean),
        'slip_peak_m': nm(slip, np.max), 'slip_mean_m': nm(slip, np.mean),
        'mu_est_mean': nm(mu, np.mean), 'mu_est_min': nm(mu, np.min),
        'energy_j': energy,
        'energy_per_m_j': energy / dist_trav if dist_trav > 0.1 and np.isfinite(energy) else np.nan,
        'speed_mps': dist_trav / duration if duration > 0 else np.nan,
        'samples': len(a),
    }


def write_row(path, row):
    new = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in row.items()})


def one_pass(a, direction, rep, robot_state_client, command_client, local_grid, csv_path):
    est = estimate_track(local_grid, robot_state_client, a.dist)
    level = ''
    if a.config == 'mission':
        if not np.isfinite(est['long']):
            print("[TRIAL] Config 'mission' without a slope estimate: pass skipped.")
            return
        params = motion.select_mobility_params(max(est['long'], est['lat']),
                                               est['rough'] if np.isfinite(est['rough']) else 0.0)
        level = MISSION_STATS['last_tier']
    else:
        params = dict(CONFIG[a.config])
        if np.isfinite(est['long']):
            g = max(est['long'], est['lat'])
            r = est['rough'] if np.isfinite(est['rough']) else 0.0
            level = ('PLAIN' if g < gait.GAIT_PLAIN_SLOPE_MAX and r < gait.GAIT_PLAIN_ROUGH_MAX
                     else 'MODERATE' if g < gait.GAIT_MODERATE_SLOPE_MAX
                     and r < gait.GAIT_MODERATE_ROUGH_MAX else 'HARD')
    if a.vel is not None:
        params['max_linear_vel'] = a.vel
    if a.protected_feet:
        params['disable_foot_obstacle_avoidance'] = False

    c0 = charge(robot_state_client)
    sampler = Sampler(robot_state_client)
    sampler.start()
    t0 = time.time()
    ok, dist_trav = motion.relative_move(a.dist, 0.0, 0.0, VISION_FRAME_NAME, command_client,
                                         robot_state_client, **params)
    duration = time.time() - t0
    arr = sampler.stop()
    row = dict(timestamp=time.strftime('%Y-%m-%d %H:%M:%S'), terrain=a.terrain, direction=direction,
               config=a.config, repetition=rep,
               true_slope_deg=(a.true_slope if direction == 'out' or a.true_slope is None else -a.true_slope),
               est_long=est['long'], est_lat=est['lat'], est_rough=est['rough'], est_has_data=est['has'],
               est_plane_long_deg=est['plane_long'], est_plane_lat_deg=est['plane_lat'],
               mission_level=level, locomotion_hint=params.get('locomotion_hint'),
               swing_height=params.get('swing_height'), ground_mu_hint=params.get('ground_mu_hint'),
               max_linear_vel=params.get('max_linear_vel'), success=bool(ok), duration_s=duration,
               dist_commanded_m=float(a.dist), dist_traveled_m=float(dist_trav), charge_start_pct=c0,
               charge_end_pct=charge(robot_state_client), note=a.note or '')
    row.update(summary(arr, float(dist_trav), duration))
    write_row(csv_path, row)
    stamp = time.strftime('%H-%M-%S')
    np.savez(os.path.join(a.folder, f"{a.terrain}_{a.config}_{direction}_{stamp}.npz"), samples=arr,
             columns=np.array(SAMPLE_COLUMNS))
    print(f"[TRIAL] {direction} {rep}: {'SUCCEEDED' if ok else 'NOT SUCCEEDED'}, {dist_trav:.2f}/{a.dist:.2f} m "
          f"in {duration:.1f} s ({row.get('speed_mps', np.nan):.2f} m/s), mean pitch "
          f"{row.get('pitch_mean_deg', np.nan):.1f}° RMS {row.get('pitch_rms_deg', np.nan):.2f}°, roll RMS "
          f"{row.get('roll_rms_deg', np.nan):.2f}°, ground under the feet (gpe) "
          f"{row.get('gpe_long_mean_deg', np.nan):+.1f}° / {row.get('gpe_lat_mean_deg', np.nan):+.1f}°, peak slip "
          f"{row.get('slip_peak_m', np.nan):.3f} m, estimated friction {row.get('mu_est_mean', np.nan):.2f}, "
          f"{row.get('energy_per_m_j', np.nan):.0f} J/m")


def turn_around(command_client, robot_state_client, params):
    p = {k: v for k, v in params.items() if k != 'max_linear_vel'}
    ok, _ = motion.relative_move(0.0, 0.0, np.pi, VISION_FRAME_NAME, command_client, robot_state_client, **p)
    return ok


def main():
    p = argparse.ArgumentParser(description="Gait trial on a straight track.")
    p.add_argument('--terrain', help="name of the track, e.g. floor, lawn, ramp10, ramp15, cross, clods")
    p.add_argument('--true-slope', type=float, default=None,
                   help="slope measured with the inclinometer, in degrees, in the outward direction")
    p.add_argument('--config', default='mission', choices=sorted(CONFIG) + ['mission'])
    p.add_argument('--dist', type=float, default=5.0, help="length of the track (m, max 10)")
    p.add_argument('--repetitions', type=int, default=3)
    p.add_argument('--back-turn', action='store_true',
                   help="at the end of every outward pass the robot turns and comes back: the opposite "
                        "direction (descent on a ramp) is measured too")
    p.add_argument('--vel', type=float, default=None, help="speed limit (m/s) instead of the configuration's")
    p.add_argument('--backwards', action='store_true',
                   help="at the end of every outward pass the robot goes back to the start walking "
                        "backwards, without turning and without recording: for tight spaces")
    p.add_argument('--protected-feet', action='store_true',
                   help="turns Spot's foot collision avoidance back on (off in the mission for the grass): "
                        "indoors, where there is no safe frontier, it is the extra net")
    p.add_argument('--estimate-only', action='store_true',
                   help="the robot stands up and estimates the slope ahead, without walking")
    p.add_argument('--note', default='')
    p.add_argument('--folder', default=FOLDER)
    p.add_argument('--hostname', default='192.168.80.3')
    p.add_argument('--list', action='store_true')
    a = p.parse_args()

    if a.list:
        for k in sorted(DESCRIPTION):
            print(f"  {k:15s} {DESCRIPTION[k]}")
        return
    if not a.terrain:
        p.error("--terrain is needed")
    if not (0.5 <= a.dist <= 10.0):
        p.error("--dist between 0.5 and 10 m")
    if a.back_turn and a.backwards:
        p.error("not --back-turn and --backwards together: choose one")
    os.makedirs(a.folder, exist_ok=True)
    csv_path = os.path.join(a.folder, 'trials.csv')

    options = SimpleNamespace(name='gaitTrial', hostname=a.hostname, verbose=False,
                              recording_user_name='', recording_session_name='')
    robot, lease_client, robot_state_client, _ = connection.setLogInfo(options)
    estop = connection.SimpleEstop(robot, options.name + '_estop')
    local_grid = spot_grid.LocalGrid(robot)
    print(f"Terrain '{a.terrain}', configuration '{a.config}' ({DESCRIPTION[a.config]}), "
          f"{a.dist:.1f} m x {a.repetitions}{' out and back' if a.back_turn else ''}.")

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.power_on()
        assert robot.is_powered_on(), 'Robot power-on failed.'
        try:
            blocking_stand(command_client)
            time.sleep(1.0)
            if a.estimate_only:
                for i in range(a.repetitions):
                    st = estimate_track(local_grid, robot_state_client, a.dist)
                    state = robot_state_client.get_robot_state()
                    _, _, _, q = connection.getPosition(robot_state_client)
                    roll, pitch = _quat_to_roll_pitch_deg(q)
                    yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
                    g_long, g_lat = _ground_plane_slopes_deg(state.kinematic_state.transforms_snapshot, yaw)
                    write_row(csv_path, dict(
                        timestamp=time.strftime('%Y-%m-%d %H:%M:%S'), terrain=a.terrain,
                        direction='still', config='estimate_only', repetition=i + 1,
                        true_slope_deg=a.true_slope, est_long=st['long'], est_lat=st['lat'],
                        est_rough=st['rough'], est_has_data=st['has'], est_plane_long_deg=st['plane_long'],
                        est_plane_lat_deg=st['plane_lat'], pitch_mean_deg=pitch, roll_mean_deg=roll,
                        gpe_long_mean_deg=g_long, gpe_lat_mean_deg=g_lat, note=a.note or ''))
                    print(f"[STILL] Body tilt: pitch {pitch:.1f}°, roll {roll:.1f}° | ground under the feet "
                          f"(gpe): along {g_long:+.1f}°, across {g_lat:+.1f}°")
                    time.sleep(1.0)
            else:
                for i in range(1, a.repetitions + 1):
                    r = input(f"\nPass {i}/{a.repetitions}: track free and robot at the start? "
                              f"[Enter = go, s = skip, q = end] ").strip().lower()
                    if r == 'q':
                        break
                    if r == 's':
                        continue
                    one_pass(a, 'out', i, robot_state_client, command_client, local_grid, csv_path)
                    if a.backwards:
                        time.sleep(0.5)
                        ok_b, d_b = motion.relative_move(
                            -a.dist, 0.0, 0.0, VISION_FRAME_NAME, command_client, robot_state_client,
                            max_linear_vel=motion.RETREAT_MAX_LINEAR_VEL_MPS,
                            disable_foot_obstacle_avoidance=(False if a.protected_feet else None))
                        print(f"[TRIAL] Back to the start walking backwards: "
                              f"{'done' if ok_b else 'NOT SUCCEEDED'} ({d_b:.2f} m)")
                        if not ok_b:
                            break
                    if a.back_turn:
                        params = dict(CONFIG.get(a.config, CONFIG['crawl_mu06']))
                        if a.protected_feet:
                            params['disable_foot_obstacle_avoidance'] = False
                        if not turn_around(command_client, robot_state_client, params):
                            print("[TRIAL] Rotation failed: I stop here.")
                            break
                        time.sleep(0.5)
                        one_pass(a, 'back', i, robot_state_client, command_client, local_grid, csv_path)
                        if not turn_around(command_client, robot_state_client, params):
                            print("[TRIAL] Rotation failed: I stop here.")
                            break
            print(f"\nRows saved in {csv_path}. Comparison: python3 tools/calibration.py --trials {csv_path}")
        except KeyboardInterrupt:
            print("\n[TRIAL] Interrupted: the robot sits down.")
        finally:
            try:
                motion.stop_robot(command_client, reason="end of the trial")
            except Exception:
                pass
            try:
                blocking_sit(command_client, timeout_sec=10)
                robot.power_off(cut_immediately=False, timeout_sec=20)
                print("Robot sitting and motors off.")
            except Exception as e:
                print(f"[CAREFUL] Power-off failed ({e}): use the tablet.")
            estop.stop()


if __name__ == '__main__':
    main()
