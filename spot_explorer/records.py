"""
Mission records: statistics, logs and data saved to disk.

Nothing here decides the robot's behaviour: it serves to understand afterwards what
happened (mission_report.py, mission_viewer.py, test_replay.py).

  MISSION_STATS      counters and times per phase, summarized at the end of the mission
  MISSION_CTX        objects of the current mission for the logs that do not receive them
                     as arguments (PRM, log folder, reference height)
  install_stdout_tee copy of all output into mission_log.txt
  BodyTiltRecorder   samples body position and tilt during a move
  append_csv, append_trajectory, TrajectoryLog
                     the mission CSVs (trajectory.csv, drift.csv, battery.csv, ...)
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
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b

from spot_explorer.robot import connection

# A helper thread to save data to disk without stopping the navigation loop.
VIS_EXECUTOR = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vis_worker")


# ==================================================================
# Mission statistics -- diagnostics/log only, never read to decide the robot's
# behaviour. Never reset: one mission per process (tests that run more than one
# in the same process see the counts summed).
# The veto/slope-source counters live instead on the PRM instance itself
# (see roadmap.PRM.print_mission_summary), because the PRM persists for the whole
# mission while here we only keep what has no natural place there.
# ==================================================================
MISSION_STATS = {
    'gait_tier_counts': {'PLAIN': 0, 'MODERATE': 0, 'HARD': 0},
    'replan_count': 0,
    'last_tier': None,   # gait level of the last command, for the [LOCALIZATION] line
    'time_s': {},        # seconds spent per phase, see timing_add / [TIMING]
    'scan_bundles': 0,
    'goals_refused': 0,       # goals excluded for the rest of the mission (_refuse_goal)
    'goals_unreachable': 0,   # goal choices without any reachable point in the graph
    'sdk_loop_calls': 0,      # successful Spot loop closures (with an answer)
    'sdk_loop_s': 0.0,        # seconds spent in those calls
    'sdk_loop_edges': 0,      # edges created by Spot
    'sdk_loop_partial': 0,    # calls ended at the time limit (partial result)
    'sdk_loop_failures': 0,   # failed loop-closure attempts (error, in the log)
    'drift_samples': 0,       # odometry drift measurements (graphnav_log_drift)
    'drift_max_m': 0.0,       # maximum drift measured
    'drift_last_m': 0.0,      # last measurement (usually on arrival at wp_0)
    'nodes_pruned': 0,            # PRM nodes discarded because inside an obstacle
    'prm_refresh': 0,             # edge refreshes performed
    'prm_refresh_skipped': 0,     # refreshes skipped (no event, robot still)
    'falls': 0,
    'self_rights': 0,
    'graphnav_ok': 0,
    'graphnav_fail': 0,
}


# Context of the current mission, for the diagnostic logs that have no way to receive
# these objects as arguments (attempt_enter_cell_from_position is called from several
# places and we do not want to change its signature). Filled in run.easy_walk(), read by
# summary.emit_mission_summary_once() and the CSV writers. Diagnostics only: no robot decision reads it.
MISSION_CTX = {
    'prm': None,
    'log_folder': None,
    'summary_done': False,
    # Ground height at mission start in the VISION frame. Spot measures heights
    # relative to where it was powered on: if it is taken to the mission area via
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


_CSV_LOCK = threading.Lock()


# trajectory.csv columns. 'move' tells what kind of movement produced the sample: 'prm'
# (navigation loop leg), 'cruise', 'graphnav', 'cone', 'backoff', 'retreat'.
_TRAJECTORY_COLUMNS = ['time', 'iteration', 'cell_row', 'cell_col', 'loop_iter', 'node_from', 'node_to',
                       'x', 'y', 'yaw_deg', 'roll_deg', 'pitch_deg', 'target_x', 'target_y', 'move',
                       'gpe_long_deg', 'gpe_lat_deg']


def timing_add(phase, seconds):
    """Accumulate the time spent in a phase ([TIMING] summary at the end of the mission)."""
    MISSION_STATS['time_s'][phase] = MISSION_STATS['time_s'].get(phase, 0.0) + float(seconds)


class StepTimer:
    """
    Stopwatch of the steps between two moves, to see where the time of a hand-off goes
    (slower on the CORE I/O than on the laptop). start() at the end of a cruise (or at
    the attempt start, robot standing), mark(step) after every step, report() at the first
    command of the next move: one line [TIMING] <label> with the total and every step, and
    the sums per step in MISSION_STATS['handoff_steps_s'] ({step: [seconds, count]}).
    mark() and report() do nothing when not started: the code that marks needs no checks.
    """

    def __init__(self):
        self.t0 = None
        self.label = ''
        self.steps = []

    def start(self, label, restart=True):
        """Start (restart=False: only if not already running)."""
        if self.t0 is not None and not restart:
            return
        self.t0 = self._last = time.perf_counter()
        self.label, self.steps = label, []

    def mark(self, step):
        if self.t0 is None:
            return
        now = time.perf_counter()
        self.steps.append((step, now - self._last))
        self._last = now

    def report(self, last_step):
        if self.t0 is None:
            return
        self.mark(last_step)
        total = self._last - self.t0
        acc = MISSION_STATS.setdefault('handoff_steps_s', {})
        for step, s in self.steps:
            a = acc.setdefault(step, [0.0, 0])
            a[0] += s
            a[1] += 1
        print(f"[TIMING] {self.label}: {total:.2f} s | "
              + " | ".join(f"{step} {s:.2f}" for step, s in self.steps))
        self.t0 = None


HANDOFF = StepTimer()


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


# Log categories turned off: the lines whose tag ([TIMING], [GAIT], ...) is here are written
# neither on screen nor in mission_log.txt (the colleague's idea, missionLog.LOG_ENABLED of
# andrea-branch). A line without a tag of its own (a table row, a blank line) follows the tag
# of the line before it. Empty = everything written. Turned on and off in one place, without
# touching the code that prints; the tools read some tags (live_viewer, mission_report,
# test_replay): keep those on.
LOG_OFF = set()


def _line_tag(line):
    """The [TAG] a line starts with (after spaces), None if it has none."""
    t = line.lstrip()
    if t.startswith('['):
        end = t.find(']')
        if 0 < end < 40:
            return t[1:end]
    return None


class _TeeStream:
    """
    Copy everything printed to a file, keeping the normal printing on screen too. In the
    file every line is preceded by a time (HH:MM:SS.mmm), useful to align the logs with a
    video or with notes taken by hand during the test. Lines of the categories in LOG_OFF are
    dropped from both; the output goes out line by line (a line waits for its end).
    """
    def __init__(self, original, file_path):
        self._orig = original
        self._file = open(file_path, 'a', buffering=1, encoding='utf-8')
        self._lock = threading.Lock()
        self._pending = ''          # the start of a line not ended yet
        self._last_tag = None       # tag of the last line, for the lines without one
        self._mid_line = False      # a flush wrote the start of the current line already
        self._dropping = False      # ... and that start was dropped (its category is off)

    def _keep(self, line):
        if not LOG_OFF:
            return True
        tag = _line_tag(line)
        if tag is not None:
            self._last_tag = tag
        elif not line.strip():
            return self._last_tag not in LOG_OFF
        return (tag or self._last_tag) not in LOG_OFF

    def write(self, s):
        # The log never stops the mission: a failed write is dropped (an optional part, on
        # purpose any error).
        try:
            with self._lock:
                text = self._pending + s
                lines = text.splitlines(keepends=True)
                self._pending = lines.pop() if lines and not lines[-1].endswith('\n') else ''
                for line in lines:
                    self._emit(line)
        except Exception:
            pass
        return len(s)

    def _emit(self, piece, ended=True):
        """Write a piece of text (a whole line, or with ended=False the start of one written by
        a flush): on screen as it is, in the file with the time at the start of each line."""
        if self._mid_line:                      # the rest of a line whose start went out already
            keep, stamp = not self._dropping, ''
        else:
            keep = self._keep(piece)
            stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' '
        if keep:
            self._orig.write(piece)
            self._file.write(stamp + piece)
        self._mid_line, self._dropping = (not ended), (not keep and not ended)

    def flush(self):
        try:
            with self._lock:
                if self._pending:               # a line without its end yet (print(..., end=''))
                    self._emit(self._pending, ended=False)
                    self._pending = ''
        except Exception:
            pass
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


def _ground_plane_slopes_deg(snap, yaw):
    """
    Slope of the ground under the feet, from Spot's ground plane estimate (frame 'gpe', from
    the legs): (along the heading, across it) in degrees, positive = ground rising ahead /
    to the left. It is what the robot really stands on, unlike the body tilt, which Spot
    partly levels. NaN if the frame is not there.
    """
    try:
        r = get_a_tform_b(snap, VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME).rotation
    except Exception:
        return float('nan'), float('nan')
    # z axis of the gpe frame in VISION = normal of the ground plane.
    nx = 2.0 * (r.x * r.z + r.w * r.y)
    ny = 2.0 * (r.y * r.z - r.w * r.x)
    nz = 1.0 - 2.0 * (r.x * r.x + r.y * r.y)
    if nz <= 1e-6:
        return float('nan'), float('nan')
    gx, gy = -nx / nz, -ny / nz                      # height gradient of the plane
    c, s = np.cos(yaw), np.sin(yaw)
    return (float(np.degrees(np.arctan(gx * c + gy * s))),
            float(np.degrees(np.arctan(-gx * s + gy * c))))


class BodyTiltRecorder:
    """
    Record the real tilt of Spot's body (roll and pitch) while it executes a move,
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
        # Real sampled trajectory: (t, x, y, yaw, roll, pitch, gpe_long, gpe_lat). Ends up
        # in trajectory.csv, to compare where the robot really went with what was
        # commanded, and the slope estimates with the ground the robot stands on.
        self.samples = []

    def _sample(self):
        try:
            snap = self._client.get_robot_state().kinematic_state.transforms_snapshot
            tf = get_a_tform_b(snap, VISION_FRAME_NAME, BODY_FRAME_NAME)
            x, y, q = tf.position.x, tf.position.y, tf.rotation
            roll, pitch = _quat_to_roll_pitch_deg(q)
            yaw = float(np.degrees(connection.yaw_of(q)))
            gpe_long, gpe_lat = _ground_plane_slopes_deg(snap, np.radians(yaw))
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
            self.samples.append((time.time(), float(x), float(y), yaw, roll, pitch, gpe_long, gpe_lat))
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
                [[s[0]] + seg_id + list(s[1:6]) + [target_xy[0], target_xy[1], move] + list(s[6:8])
                 for s in samples])


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


def visualize_grid_with_candidates(pts, terrain_real, obstacle_mask, robot_x, robot_y,
                                   candidates, chosen_point, iteration, env, save_path,
                                   prm_graph=None, chosen_path=None,
                                   cells_obstacle_dist=None,
                                   valid_values=None, grad_values=None, rough_values=None):
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
            env_start_cell=np.array((-1, -1) if env is None else tuple(env.start_cell), dtype=np.int64),
            traveled_arcs=np.array(getattr(env, '_traveled_arcs', []), dtype=np.float64).reshape(-1, 4),
        )
    except Exception as e:
        print(f"[DATA SAVE] Could not prepare the figure data: {e}")
    timing_add('figure data', time.perf_counter() - t0)
