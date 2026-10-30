"""
MISSION RECORDS: statistics, logs and data saved to disk.

Nothing here decides the robot's behaviour: it serves to understand afterwards what
happened (mission_report.py, mission_viewer.py, test_replay.py).

  MISSION_STATS      counters and times per phase, summarized at the end of the mission
  MISSION_CTX        objects of the current mission for the logs that do not receive them
                     as arguments (PRM, log folder, reference height)
  install_stdout_tee copy of all output into mission_log.txt
  BodyTiltRecorder   samples body position and tilt during a move
  SegmentLog, append_csv, append_trajectory, TrajectoryLog, log_segment
                     the mission CSVs (segments.csv, trajectory.csv, drift.csv, ...)
  save_npz_async     saving of the .npz files on a helper thread, without stopping the loop
"""
import csv
import os
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import numpy as np
from bosdyn.client.frame_helpers import GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b

from spot_explorer.robot import connection

# A helper thread to save data to disk without stopping the navigation loop.
VIS_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vis_worker")


# ==================================================================
# MISSION STATISTICS -- diagnostics/log only, never read to decide the robot's
# behaviour. Never reset: one mission per process (tests that run more than one
# in the same process see the counts summed).
# The veto/slope-source counters live instead on the PRM instance itself
# (see roadmap.PRM.print_mission_summary), because the PRM persists for the whole
# mission while here we only keep what has no natural place there.
# ==================================================================
MISSION_STATS = {
    'gait_tier_counts': {'PLAIN': 0, 'MODERATE': 0, 'HARD': 0},
    'replan_count': 0,
    'near_threshold_saves': 0,
    'segments_logged': 0,
    'last_tier': None,   # tier of the last segment, read for the CSV row
    'time_s': {},        # seconds spent per phase, see timing_add / [TIMING]
    'scan_bundles': 0,
    'frontier_blocks': 0,
    'frontier_aborts': 0,
    'rotation_refusals': 0,   # rotations refused by the sector check
    'straight_retreats': 0,   # straight back-offs after a rotation refusal
    'goal_repicks': 0,        # goal points chosen again inside the loop
    'goals_refused': 0,       # goals excluded for the rest of the mission (_refuse_goal)
    'goals_unreachable': 0,   # goal choices without any reachable point in the graph
    'arrivals_near_goal': 0,  # arrivals accepted with the robot just outside the cell
    'backtracks_in_place': 0, # returns without movement: no breadcrumb far enough
    'sdk_loop_calls': 0,      # successful Spot loop closures (with an answer)
    'sdk_loop_s': 0.0,        # seconds spent in those calls
    'sdk_loop_edges': 0,      # edges created by Spot
    'sdk_loop_partial': 0,    # calls ended at the time limit (partial result)
    'sdk_loop_failures': 0,   # failed loop-closure attempts (error, in the log)
    'drift_samples': 0,       # odometry drift measurements (graphnav_log_drift)
    'drift_max_m': 0.0,       # maximum drift measured
    'drift_last_m': 0.0,      # last measurement (usually on arrival at wp_0)
    'graphnav_trips_avoided': 0,  # GraphNav moves avoided: from the waypoint the graph does not reach the cell
    'cone_moves': 0,          # advances inside the possible rotation cone
    'retreats_suppressed': 0,  # back-offs avoided because useless
    'early_replans': 0,       # replans at the first blocking cell
    'traps_detected': 0,      # traps recognized by the 360 degree fan
    'arrival_refusals': 0,    # steps refused because the arrival pose has no exits
    'graphnav_backtracks': 0,     # one-waypoint returns that succeeded with GraphNav
    'graphnav_backtrack_fail': 0, # failed GraphNav returns (walked fallback)
    'nodes_pruned': 0,            # PRM nodes discarded because inside an obstacle
    'idle_loops': 0,              # idle loop turns (anti-stall guard)
    'multinode_cut': 0,           # multi-node sequences stopped at an intermediate node
    'multinode_moves': 0,         # moves that crossed more than one node
    'multinode_nodes': 0,         # nodes crossed by those moves
    'prm_refresh': 0,             # edge refreshes performed
    'prm_refresh_skipped': 0,     # refreshes skipped (no event, robot still)
    'rotation_maneuvers': 0,
    'shortcuts': 0,
    'falls': 0,
    'self_rights': 0,
    'graphnav_ok': 0,
    'graphnav_fail': 0,
}


# Context of the current mission, for the diagnostic logs that have no way to receive
# these objects as arguments (attempt_enter_cell_from_position is called from several
# places and we do not want to change its signature). Filled in run.easy_walk(), read by
# log_segment() and summary.emit_mission_summary_once(). Diagnostics only: no robot decision reads it.
MISSION_CTX = {
    'prm': None,
    'log_folder': None,
    'segment_log': None,
    'summary_done': False,
    # Ground height at MISSION START in the VISION frame. Spot measures heights
    # relative to where it was POWERED ON: if it is taken to the mission area via
    # stairs, all heights come out shifted (by metres). x, y and heading
    # are already relative to the mission start (env.set_origin with the starting pose); for
    # the height this value is saved in all the data and maps, and the viewer shows heights
    # relative to it. The COMPUTATIONS do not depend on it: they only use height differences and
    # the ground estimated under the robot at every scan (see spot_grid.GROUND_HEIGHT_TOLERANCE_M).
    'mission_z0': None,
    # Fiducial seen at the start (apriltags.find_fiducials): ID and VISION position, to
    # see it again on return to wp_0 (graphnav_check_start_fiducial). None = none.
    'start_fiducial': None,
}


_SEGMENT_COLUMNS = [
    'timestamp', 'iteration', 'cell_row', 'cell_col', 'node_from', 'node_to',
    'x_from', 'y_from', 'x_to', 'y_to', 'dist_m',
    'slope_source', 'long_slope', 'lat_slope',
    'roughness',
    'cost_dist', 'cost_long', 'cost_lat', 'cost_rough', 'cost_total',
    'tier', 'gait_override', 'success', 'dist_traveled_m', 'dist_commanded_m', 'duration_s',
    'roll_start_deg', 'pitch_start_deg', 'roll_end_deg', 'pitch_end_deg',
    'roll_peak_abs_deg', 'pitch_peak_abs_deg', 'tilt_samples', 'tilt_errors',
]


_CSV_LOCK = threading.Lock()


# trajectory.csv columns. 'move' (2026-10-26) tells what kind of movement produced the
# sample: 'prm' (navigation loop leg), 'graphnav', 'cone', 'backoff',
# 'retreat'. Until 2026-10-26 movements other than 'prm' did NOT end up in the file: the
# viewer joined with a straight line the last point before a GraphNav return and the first
# after, a line that cut through obstacles (mission of 22-10, around 16:00).
_TRAJECTORY_COLUMNS = ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                       'x', 'y', 'yaw_deg', 'roll_deg', 'pitch_deg', 'target_x', 'target_y', 'move']


def timing_add(phase, seconds):
    """Accumulate the time spent in a phase ([TIMING] summary at the end of the mission)."""
    MISSION_STATS['time_s'][phase] = MISSION_STATS['time_s'].get(phase, 0.0) + float(seconds)


def save_npz_async(path, **arrays):
    """Compressed save on the service thread: the main loop does not wait for the disk."""
    def _job():
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            np.savez_compressed(path, **arrays)
        except Exception as e:
            print(f"[DATA SAVE] Could not save {os.path.basename(path)}: {e}")
    VIS_EXECUTOR.submit(_job)


def mission_z0():
    """Reference height of the mission (NaN until it is fixed), for the saved files."""
    z0 = MISSION_CTX.get('mission_z0')
    return np.nan if z0 is None else float(z0)


def set_mission_z0(robot_state_client, z_body):
    """
    Fix the reference height at mission start: the ground plane estimated by Spot under
    the feet ('gpe' frame); if not available, the body height minus the nominal standing
    height. Prints how much the mission height differs from the power-on one.
    """
    z0, source = None, "Spot's ground plane"
    try:
        snap = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        z0 = float(get_a_tform_b(snap, VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME).position.z)
    except Exception as e:
        print(f"[HEIGHTS] Ground plane not available ({e}): using the body height.")
    if z0 is None or not np.isfinite(z0):
        z0, source = float(z_body) - 0.5, "body height - 0.5 m"
    MISSION_CTX['mission_z0'] = z0
    note = ""
    if abs(z0) > 1.0:
        note = (" -- the robot was powered on at another height (stairs?). Navigation is not "
                "affected; the value is saved as mission_z0 in every file and mission_viewer "
                "shows the heights relative to the mission start")
    print(f"[HEIGHTS] Height reference = ground at mission start: z={z0:.3f} m in the VISION frame "
          f"({source}), i.e. {z0:+.2f} m relative to power-on{note}.")
    return z0


class _TeeStream:
    """
    Copy everything printed to a file, keeping the normal printing on screen too. In the
    file every line is preceded by a time (HH:MM:SS.mmm), useful to align the logs with a
    video or with notes taken by hand during the test.
    """
    def __init__(self, original, file_path):
        self._orig = original
        self._file = open(file_path, 'a', buffering=1, encoding='utf-8')
        self._lock = threading.Lock()
        self._at_line_start = True

    def write(self, s):
        try:
            self._orig.write(s)
        except Exception:
            pass
        try:
            with self._lock:
                out = []
                for chunk in s.splitlines(keepends=True):
                    if self._at_line_start:
                        out.append(datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' ')
                    out.append(chunk)
                    self._at_line_start = chunk.endswith('\n')
                if out:
                    self._file.write(''.join(out))
        except Exception:
            pass
        return len(s)

    def flush(self):
        try:
            self._orig.flush()
        except Exception:
            pass
        try:
            with self._lock:
                self._file.flush()
        except Exception:
            pass

    def close_file(self):
        try:
            with self._lock:
                self._file.close()
        except Exception:
            pass

    def __getattr__(self, name):
        # isatty, encoding, fileno... go to the original stream
        return getattr(self._orig, name)


def install_stdout_tee(file_path):
    """From here on every print also goes to file_path. Idempotent."""
    try:
        if not isinstance(sys.stdout, _TeeStream):
            sys.stdout = _TeeStream(sys.stdout, file_path)
            print(f"[LOG] Full copy of the output to file: {file_path}")
    except Exception as e:
        print(f"[LOG] Could not enable the copy of the output to file: {e}")


def remove_stdout_tee():
    try:
        if isinstance(sys.stdout, _TeeStream):
            tee = sys.stdout
            sys.stdout = tee._orig
            tee.close_file()
    except Exception:
        pass


def _quat_to_roll_pitch_deg(q):
    """Roll and pitch (degrees) of a quaternion with attributes w, x, y, z. In the VISION
    frame, which is aligned with gravity, they are the real tilt of the body relative to
    the horizontal (same convention the file already uses to compute the yaw)."""
    roll = np.arctan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x ** 2 + q.y ** 2))
    pitch = np.arcsin(np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
    return float(np.degrees(roll)), float(np.degrees(pitch))


class BodyTiltRecorder:
    """
    Record the REAL tilt of Spot's body (roll and pitch) while it executes a move,
    querying the robot state about 5 times per second in a separate thread. Used to
    compare what the planner estimated (terrain slope) with what Spot actually felt: it is
    the data that allows thresholds and weights to be retuned.

    Usage:
        rec = BodyTiltRecorder(robot_state_client); rec.start()
        ... move ...
        stats = rec.stop()   # dict with start, end, peaks, number of samples

    Reading note: Spot can level its body relative to the terrain, so the body tilt is an
    indirect indicator of the slope, not the slope itself. Every reading error is counted
    and ignored.
    """
    def __init__(self, robot_state_client, period_s=0.2):
        self._client = robot_state_client
        self._period = period_s
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._start = None
        self._last = None
        self._peak_roll = 0.0
        self._peak_pitch = 0.0
        self._n = 0
        self._errors = 0
        # Real sampled trajectory: (t, x, y, yaw, roll, pitch). Ends up in
        # trajectory.csv, to compare where the robot really went with what was
        # commanded.
        self.samples = []

    def _sample(self):
        try:
            x, y, _, q = connection.getPosition(self._client)
            roll, pitch = _quat_to_roll_pitch_deg(q)
            yaw = float(np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))))
        except Exception:
            with self._lock:
                self._errors += 1
            return None
        with self._lock:
            if self._start is None:
                self._start = (roll, pitch)
            self._last = (roll, pitch)
            self._peak_roll = max(self._peak_roll, abs(roll))
            self._peak_pitch = max(self._peak_pitch, abs(pitch))
            self._n += 1
            self.samples.append((time.time(), float(x), float(y), yaw, roll, pitch))
        return roll, pitch

    def _run(self):
        while not self._stop.wait(self._period):
            self._sample()

    def start(self):
        try:
            self._sample()
            self._thread = threading.Thread(target=self._run, name="body_tilt_recorder", daemon=True)
            self._thread.start()
        except Exception:
            pass

    def stop(self):
        try:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=1.0)
            self._sample()
        except Exception:
            pass
        with self._lock:
            return {
                'roll_start': self._start[0] if self._start else None,
                'pitch_start': self._start[1] if self._start else None,
                'roll_end': self._last[0] if self._last else None,
                'pitch_end': self._last[1] if self._last else None,
                'roll_peak_abs': self._peak_roll if self._n else None,
                'pitch_peak_abs': self._peak_pitch if self._n else None,
                'samples': self._n,
                'errors': self._errors,
            }


class SegmentLog:
    """CSV with one row for every segment (from one node to the next) actually travelled
    or attempted. Opens in a spreadsheet. A row is written and flushed to disk at once, so
    even an interrupted mission leaves the data up to the last segment."""
    def __init__(self, path):
        self._path = path
        self._lock = threading.Lock()
        with open(path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(_SEGMENT_COLUMNS)

    def write_row(self, row_dict):
        try:
            def fmt(v):
                if v is None:
                    return ''
                if isinstance(v, (float, np.floating)):
                    return f"{float(v):.4f}"
                return v
            with self._lock:
                with open(self._path, 'a', newline='', encoding='utf-8') as f:
                    csv.writer(f).writerow([fmt(row_dict.get(c)) for c in _SEGMENT_COLUMNS])
        except Exception as e:
            print(f"[LOG] Could not write the segment row: {e}")


def append_csv(name, header, rows):
    """Append rows to a CSV in the log folder (header on the first write). Never raises."""
    folder = MISSION_CTX.get('log_folder')
    if not folder or not rows:
        return
    try:
        path = os.path.join(folder, name)
        with _CSV_LOCK:
            new = not os.path.exists(path)
            with open(path, 'a', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                if new:
                    w.writerow(header)
                for r in rows:
                    w.writerow([f"{v:.4f}" if isinstance(v, (float, np.floating)) else v for v in r])
    except Exception as e:
        print(f"[LOG] Could not write {name}: {e}")


def append_trajectory(samples, move, target_xy=(np.nan, np.nan), seg_id=None):
    """Samples of a BodyTiltRecorder into trajectory.csv. seg_id: the 6 columns from iteration to node_to."""
    seg_id = list(seg_id) if seg_id is not None else [''] * 6
    append_csv("trajectory.csv", _TRAJECTORY_COLUMNS,
                [[t] + seg_id + [x, y, yw, r, pt, target_xy[0], target_xy[1], move]
                 for (t, x, y, yw, r, pt) in samples])


class TrajectoryLog:
    """
    Record in trajectory.csv where the robot passes during a move that is not a leg of
    the navigation loop (GraphNav, cone, back-offs). Never raises to the caller.

        with TrajectoryLog(robot_state_client, 'graphnav', (x, y)):
            ... move ...
    """
    def __init__(self, robot_state_client, move, target_xy=(np.nan, np.nan)):
        self._client, self._move, self._target = robot_state_client, move, target_xy
        self._rec = None

    def __enter__(self):
        try:
            self._rec = BodyTiltRecorder(self._client)
            self._rec.start()
        except Exception:
            self._rec = None
        return self

    def __exit__(self, *exc):
        if self._rec is None:
            return False
        try:
            self._rec.stop()
            append_trajectory(list(self._rec.samples), self._move, self._target)
        except Exception as e:
            print(f"[LOG] Trajectory of the move '{self._move}' not recorded: {e}")
        return False


def log_segment(row_dict):
    seg_log = MISSION_CTX.get('segment_log')
    if seg_log is not None:
        seg_log.write_row(row_dict)
        MISSION_STATS['segments_logged'] += 1


def visualize_grid_with_candidates(pts, terrain_real, obstacle_mask, robot_x, robot_y,
                                   candidates, chosen_point, iteration, env, save_path,
                                   prm_graph=None, chosen_path=None,
                                   cells_obstacle_dist=None, intensity_values=None,
                                   valid_values=None, grad_values=None, rough_values=None,
                                   include_diagnostics=True):
    """
    Data of a figure (scan, candidates, chosen point, PRM graph, path) saved in
    <name>_vis.npz. Nothing is drawn during the mission: mission_viewer.py makes the
    figures afterwards, from these files.
    """
    if save_path is None:
        return
    save_visualization_bundle(save_path, pts, terrain_real, obstacle_mask, robot_x, robot_y,
                               candidates, chosen_point, iteration, env, prm_graph, chosen_path,
                               cells_obstacle_dist, valid_values, grad_values, rough_values)


def save_visualization_bundle(save_path, pts, terrain_real, obstacle_mask, robot_x, robot_y,
                               candidates, chosen_point, iteration, env, prm_graph, chosen_path,
                               cells_obstacle_dist, valid_values, grad_values, rough_values):
    """
    Instead of a figure: all the data the figure would have shown, including the PRM graph
    at that moment. Called when the graph changes (attempt start, replan, block), so the
    graph is saved here and not at every scan.
    """
    t0 = time.perf_counter()
    try:
        def _arr(a, dtype=np.float64):
            return np.empty(0, dtype=dtype) if a is None else np.asarray(a, dtype=dtype)

        node_ids = np.array(list(prm_graph.nodes.keys()), dtype=np.int64) if prm_graph else np.empty(0, np.int64)
        node_xy = (np.array([prm_graph.nodes[i] for i in node_ids], dtype=np.float64)
                   if prm_graph and len(node_ids) else np.empty((0, 2)))
        pairs, weights = [], []
        if prm_graph:
            for a, nbrs in prm_graph.edges.items():
                for b, w in nbrs:
                    if a < b:
                        pairs.append((a, b)); weights.append(w)
        slope_keys = list(prm_graph.edge_slope_info.keys()) if prm_graph else []
        slope_vals = [prm_graph.edge_slope_info[k] for k in slope_keys] if prm_graph else []
        traversed = np.array(sorted(getattr(prm_graph, 'traversed_edges', set())), dtype=np.int64)

        cells = []
        if env is not None:
            for row in range(env.rows):
                for col in range(env.cols):
                    wp = env.get_world_position_from_cell(row, col)
                    if wp is None:
                        continue
                    st = env.get_cell_status(row, col)
                    st = st[0] if isinstance(st, (tuple, list)) else st
                    cells.append((row, col, wp[0], wp[1], str(st)))

        def _pts(lst):
            try:
                a = np.asarray([(float(p[0]), float(p[1])) for p in (lst or [])], dtype=np.float64)
                return a.reshape(-1, 2)
            except Exception:
                return np.empty((0, 2))

        out = os.path.splitext(save_path)[0] + "_vis.npz"
        save_npz_async(
            out,
            pts=_arr(pts), terrain=_arr(terrain_real), obstacle_mask=_arr(obstacle_mask),
            obstacle_distance=_arr(cells_obstacle_dist), terrain_valid=_arr(valid_values),
            gradient=_arr(grad_values), roughness=_arr(rough_values),
            robot_xy=np.array([robot_x, robot_y], dtype=np.float64), mission_z0=mission_z0(),
            # instant of the view: mission_viewer matches the view to the scans with this, not
            # with the file date (after a git clone or pull it is the download date)
            time=time.time(),
            chosen_point=np.array([np.nan if v is None else v for v in (chosen_point or (None, None))],
                                  dtype=np.float64),
            chosen_path=_pts(chosen_path), iteration=iteration,
            candidates_valid=_pts((candidates or {}).get('valid')),
            candidates_rejected=_pts((candidates or {}).get('rejected')),
            prm_node_ids=node_ids, prm_node_xy=node_xy,
            prm_edges=np.array(pairs, dtype=np.int64).reshape(-1, 2),
            prm_edge_weights=np.array(weights, dtype=np.float64),
            prm_slope_keys=np.array(slope_keys, dtype=np.int64).reshape(-1, 2),
            prm_slope_values=np.array(slope_vals, dtype=np.float64).reshape(-1, 2),
            prm_traversed_edges=traversed.reshape(-1, 2),
            env_cells=np.array([c[:4] for c in cells], dtype=np.float64).reshape(-1, 4),
            env_cell_status=np.array([c[4] for c in cells]),
            env_cell_size=np.nan if env is None else float(env.cell_size),
            env_origin_yaw=np.nan if env is None else float(env.origin_yaw),
            traveled_arcs=np.array(getattr(env, '_traveled_arcs', []), dtype=np.float64).reshape(-1, 4),
        )
    except Exception as e:
        print(f"[DATA SAVE] Could not prepare the figure data: {e}")
    timing_add('figure data', time.perf_counter() - t0)
