"""
The robot on a map already built (MapNavigator): GraphNav graph upload, localization, the
transform between the mission's VISION frame and today's, following PRM paths with the safe
frontier, today's scans folded into the mission maps, battery and return home, logs and the
state read by the live viewer (tools/navigation_viewer.py).

WHAT IT DOES
  1. Plans on the mission PRM (MissionMap.plan), limited to the seen area.
  2. Follows the path one scan at a time, like the mission loop: safe frontier on the rest of
     the path, decide_next_move, stop mid-move when the frontier shortens, wait, block
     confirmed while standing -> edge dropped and new plan.
  3. Every scan of today goes into the global maps (in the mission frame, see update_maps)
     and the PRM edges are re-evaluated where new data arrived: an obstacle that appeared
     removes edges, one that went away gives them back. If the path breaks, or there is one
     at least 10% shorter, the robot switches at once. The updated maps are saved in the log
     folder, and --maps starts again from there.
  4. If the PRM cannot make it (repeated blocks, no path), fallback: GraphNav to the waypoint
     nearest to the goal, then a last attempt with the PRM from there.
  5. Battery (robot.battery.BatteryMonitor, threshold BATTERY_RETURN_PCT as in the mission):
     below the threshold the robot stops and goes home (wp_0), first with GraphNav, then with
     the PRM.
  6. GraphNav lost (lost detector, or no waypoint for MAX_ROUNDS_WITHOUT_LOC readings): while
     standing, relocalization on the nearest waypoint, then on the fiducial; if that fails the
     robot does not move any more (the transform between the frames would be wrong).
  7. Logs in MissionLogs/Navigation_<date>/: navigation_log.txt, navigation.csv (one round
     per row, mission frame), trajectory.csv and segments.csv as in the mission, battery.csv,
     drift.csv; for the live viewer navigation_info.json, navigation_state.json and
     navigation_map.npz.

WHAT IT DOES NOT DO (yet)
  - no cone, backoff, traps, return on breadcrumbs: a leg that needs an impossible rotation is
    dropped and the robot replans;
  - with UPDATE_MAPS = False (--no-map-update) the maps stay those of the mission: only the
    edges that the current scan sees blocked are dropped, for good (drop_edges_seen_blocked);
  - the drift INSIDE the mission's global map (TODO, main works point 3) stays: an obstacle
    seen at the end of the mission can be tens of cm away from where GraphNav puts it.
"""
import json
import os
import time
from datetime import datetime
from types import SimpleNamespace

import numpy as np

from spot_explorer import config as P
from spot_explorer import records
from spot_explorer.mission import mission_prm
from spot_explorer.navigation.frames import Transform2D, wrap, yaw_of_quat
from spot_explorer.navigation.mission_map import LOG_TAG, limit_to_seen_area, mission_raster, quiet
from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.perception import spot_grid
from spot_explorer.planning import decisions
from spot_explorer.planning import safe_frontier
from spot_explorer.robot import connection
from spot_explorer.robot import motion

# Plans at most for one goal, before the GraphNav fallback. A new plan is made after a block
# confirmed while standing and when the scan sees an edge of the path blocked.
MAX_PLANS = 10
# Today's scans go into the global maps and the PRM is re-evaluated where they change (see
# MapNavigator.update_maps). False = the mission maps stay fixed, only the permanent drop of
# the edges seen blocked (drop_edges_seen_blocked).
UPDATE_MAPS = True
# Replanning along the way: the robot switches if the new path costs at least this fraction
# less (as in the mission, see PRM.find_path_dijkstra).
NEW_PATH_MARGIN = 0.10
# How many cells in common with the terrain map are needed to estimate the height offset
# between today's frame and the mission's (median of the differences).
MIN_HEIGHT_CELLS = 200
# Rounds in a row without GraphNav localization (or with the lost detector saying LOST)
# before stopping to relocalize: a single round can be a missed reading.
MAX_ROUNDS_WITHOUT_LOC = 2
# How often the map image for the viewer is rewritten, and how often the full maps (.pkl,
# ~35 MB) are saved in the log folder.
MAP_IMAGE_EVERY_S = 10.0
PKL_EVERY_S = 300.0
# FRAME JUMP: between two rounds the estimate of where the robot is in the mission map moves
# by more than this (m or degrees). GraphNav corrects the drift a little at a time, so a jump
# is checked with FRAME_JUMP_CONFIRM_READINGS more readings, FRAME_JUMP_CONFIRM_WAIT_S apart:
# if they agree with the new estimate (within FRAME_JUMP_AGREE_M) it is a real correction,
# otherwise the reading is not trusted (it counts as a round without localization).
FRAME_JUMP_M = 0.5
FRAME_JUMP_DEG = 10.0
FRAME_JUMP_CONFIRM_READINGS = 2
FRAME_JUMP_CONFIRM_WAIT_S = 0.3
FRAME_JUMP_AGREE_M = 0.10
# Graph upload: a progress line at most this often (s).
UPLOAD_PROGRESS_EVERY_S = 5.0

# Special goal: back to the mission start (wp_0).
HOME = 'home'

# One row per round of the navigation loop, in the mission frame (see _csv_row).
NAVIGATION_COLUMNS = ['time', 'goal', 'plan', 'round', 'phase', 'x_m', 'y_m', 'yaw_deg',
                      'node', 'next', 'decision', 'frontier', 'advance_m', 'remaining_m',
                      'battery_pct', 'rounds_without_loc', 'T_tx', 'T_ty', 'T_th_deg', 'dz',
                      'scans_in_map']


def graphnav_says_lost(state):
    """True if GraphNav's lost detector says LOST (False if there is none)."""
    try:
        if hasattr(state, 'HasField') and not state.HasField('lost_detector_state'):
            return False
        return bool(getattr(getattr(state, 'lost_detector_state', None), 'is_lost', False))
    except Exception:
        return False


class MapNavigator:
    """
    The robot on a map already built: uploads the GraphNav graph, localizes, keeps the
    transform old VISION -> today's VISION and follows the PRM paths with the safe frontier.
    """

    def __init__(self, mission_map, robot, robot_state_client, command_client, client_metadata, log_folder):
        from spot_explorer.graphnav import map_recording
        self.map = mission_map
        self.robot = robot
        self.rsc = robot_state_client
        self.cc = command_client
        self.ri = map_recording.RecordingInterface(robot, log_folder, client_metadata)
        self.gn = self.ri._graph_nav_client
        self.T = None                       # Transform2D old -> today's VISION
        self.tracker = safe_frontier.ArcVerificationTracker(robot)
        self.local_grid = spot_grid.LocalGrid(robot)
        self.init_maps()
        self.init_session(log_folder)

    def init_maps(self, update=None):
        """State of the map updates (also for the tests, which do not go through __init__)."""
        self.update = UPDATE_MAPS if update is None else bool(update)
        self.dz = None                      # mission height - today's height (m)
        self.n_scans_in_map = 0
        self.t_maps_s = 0.0

    def init_session(self, log_folder=None, battery=None):
        """
        Session state: log folder (None = no files), battery monitor (BatteryMonitor, None = no
        check), why the last goal ended (end_reason: None, 'fallen', 'battery', 'lost'), phase
        and counters for the log.
        """
        self.log_folder = log_folder
        self.battery = battery
        self.end_reason = None
        self.returning = False
        self.rounds_without_loc = 0
        self.phase = 'ready'
        self.goal = None                    # (x, y) mission VISION, description
        self.path = []                      # PRM nodes still to go
        self.n_plan = 0
        self.round = 0
        self.last = {}                      # last decision and frontier, for navigation.csv
        self._t_image = 0.0                 # the first map image at once, for the viewer
        self._t_pkl = time.time()           # the first full save after PKL_EVERY_S
        if log_folder:
            self._write_info()

    # --- graph and localization ----------------------------------------------------------
    def upload_graph(self):
        """
        The graph downloaded at the end of the mission onto Spot. If Spot already has that
        graph (same waypoints: a navigation earlier today, or the mission itself without a
        reboot) it is not cleared: upload_graph answers with the snapshots Spot does NOT have,
        and only those are sent, so a second start takes seconds instead of minutes (the
        snapshots of a mission are ~150 MB). Any other graph is cleared first. A progress line
        with MB sent, speed and time left. Returns (waypoint snapshots sent, MB sent).
        """
        from bosdyn.api.graph_nav import map_pb2
        ng = self.map.ng
        self.ri.stop_recording()
        ours = {w.id for w in ng['graph'].waypoints}
        try:
            on_robot = {w.id for w in self.gn.download_graph().waypoints}
        except Exception:
            on_robot = set()
        if on_robot and on_robot == ours:
            print(f"{LOG_TAG} Spot already has this graph ({len(ours)} waypoints): not cleared, only the "
                  f"missing snapshots are sent.")
        else:
            if on_robot:
                print(f"{LOG_TAG} Spot has another graph ({len(on_robot)} waypoints, {len(on_robot & ours)} "
                      f"in common with this one): cleared.")
            self.gn.clear_graph()
        resp = self.gn.upload_graph(graph=ng['graph'], generate_new_anchoring=not ng['graph'].anchoring.anchors)
        todo = [('waypoint', sid, os.path.join(ng['dir'], 'waypoint_snapshots', sid), map_pb2.WaypointSnapshot)
                for sid in resp.unknown_waypoint_snapshot_ids]
        todo += [('edge', sid, os.path.join(ng['dir'], 'edge_snapshots', sid), map_pb2.EdgeSnapshot)
                 for sid in resp.unknown_edge_snapshot_ids]
        todo = [t for t in todo if os.path.exists(t[2])]
        total_mb = sum(os.path.getsize(t[2]) for t in todo) / 1e6
        if todo:
            print(f"{LOG_TAG} Uploading {len(todo)} snapshots to Spot, {total_mb:.0f} MB...")
        t0 = t_last = time.time()
        sent_mb, n_wp = 0.0, 0
        for i, (kind, sid, path, cls) in enumerate(todo, 1):
            snap = cls()
            with open(path, 'rb') as f:
                snap.ParseFromString(f.read())
            if kind == 'waypoint':
                self.gn.upload_waypoint_snapshot(snap)
                n_wp += 1
            else:
                self.gn.upload_edge_snapshot(snap)
            sent_mb += os.path.getsize(path) / 1e6
            now = time.time()
            if now - t_last >= UPLOAD_PROGRESS_EVERY_S and i < len(todo):
                t_last = now
                speed = sent_mb / max(now - t0, 1e-6)
                print(f"{LOG_TAG} Upload: {i}/{len(todo)} snapshots, {sent_mb:.0f}/{total_mb:.0f} MB, "
                      f"{speed:.1f} MB/s, about {(total_mb - sent_mb) / max(speed, 1e-6):.0f} s left.")
        self.ri.invalidate_graph_cache()
        dt = time.time() - t0
        print(f"{LOG_TAG} Graph on Spot: {n_wp} waypoint snapshots and {len(todo) - n_wp} edge snapshots "
              f"sent ({sent_mb:.0f} MB in {dt:.0f} s).")
        return n_wp, sent_mb

    def localize(self, mode='auto'):
        """
        'fiducial': on the most visible tag that is in the map (the start one);
        'wp_0': the robot stands on the start pose of the mission, guess = wp_0 + ICP;
        'auto': first the fiducial, then wp_0. True if GraphNav is localized.
        """
        from bosdyn.api.graph_nav import graph_nav_pb2, nav_pb2
        from bosdyn.client.frame_helpers import get_odom_tform_body
        ok = False
        if mode in ('auto', 'fiducial'):
            ok = self.ri.initialize_with_fiducial(self.rsc, P.START_FIDUCIAL_ID)
        if not ok and mode in ('auto', 'wp_0'):
            wp0 = self.map.ng['by_name'].get('wp_0')
            if wp0 is None:
                print(f"{LOG_TAG} wp_0 is not in the graph: no localization without a fiducial.")
                return False
            try:
                state = self.rsc.get_robot_state()
                odom_T_body = get_odom_tform_body(state.kinematic_state.transforms_snapshot)
                loc = nav_pb2.Localization()
                loc.waypoint_id = wp0
                loc.waypoint_tform_body.rotation.w = 1.0
                self.gn.set_localization(
                    initial_guess_localization=loc, ko_tform_body=odom_T_body.to_proto(),
                    max_distance=P.GRAPHNAV_RELOC_MAX_DIST_M, max_yaw=P.GRAPHNAV_RELOC_MAX_YAW_RAD,
                    fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL)
                ok = True
                print(f"{LOG_TAG} Localized on wp_0 (robot on the start pose, refined by ICP).")
            except Exception as e:
                print(f"{LOG_TAG} Localization on wp_0 failed: {e}")
        if ok:
            ok = self.update_transform("after localization") is not None
        return ok

    def _transform_now(self, where):
        """(transform from GraphNav's localization now, waypoint name), or (None, why)."""
        try:
            st = self.gn.get_localization_state()
            loc = st.localization
        except Exception as e:
            return None, f"GraphNav state not readable ({where}): {e}"
        if graphnav_says_lost(st):
            return None, f"GraphNav's lost detector says LOST ({where}): keeping the previous estimate."
        w = self.map.ng['wp'].get(loc.waypoint_id)
        if not loc.waypoint_id or w is None:
            return None, f"GraphNav not localized on a known waypoint ({where}): keeping the previous estimate."
        b = loc.waypoint_tform_body
        c, s = np.cos(w['yaw']), np.sin(w['yaw'])
        old = (w['x'] + c * b.position.x - s * b.position.y,
               w['y'] + s * b.position.x + c * b.position.y,
               wrap(w['yaw'] + yaw_of_quat(b.rotation)))
        x, y, _z, q = connection.getPosition(self.rsc)
        return Transform2D.from_poses((x, y, yaw_of_quat(q)), old), w['name']

    def _shift(self, a, b):
        """How far apart two transforms put the robot NOW in the mission map (m, degrees)."""
        x, y, _z, _q = connection.getPosition(self.rsc)
        pa, pb = a.inverse(x, y), b.inverse(x, y)
        return float(np.hypot(pa[0] - pb[0], pa[1] - pb[1])), float(np.degrees(abs(wrap(a.th - b.th))))

    def _jump_confirmed(self, new, where):
        """
        The estimate jumped (FRAME_JUMP_M / FRAME_JUMP_DEG) in one round. A real GraphNav
        correction stays; a wrong localization (a similar-looking place, a moving object in
        the ICP) usually does not. FRAME_JUMP_CONFIRM_READINGS more readings must agree with
        the new estimate. The robot is standing: this is called only between moves.
        """
        dm, dg = self._shift(new, self.T)
        print(f"{LOG_TAG} FRAME JUMP {where}: the robot would be {dm:.2f} m / {dg:.1f} degrees away "
              f"from where it was a round ago. Checking with {FRAME_JUMP_CONFIRM_READINGS} more readings.")
        for _ in range(FRAME_JUMP_CONFIRM_READINGS):
            time.sleep(FRAME_JUMP_CONFIRM_WAIT_S)
            again, _name = self._transform_now(where)
            if again is None or self._shift(again, new)[0] > FRAME_JUMP_AGREE_M:
                print(f"{LOG_TAG} Jump NOT confirmed: the localization is not trusted this round.")
                return False
        print(f"{LOG_TAG} Jump confirmed by the next readings: GraphNav corrected the position, "
              f"using the new estimate.")
        return True

    def update_transform(self, where=""):
        """
        Old VISION -> today's VISION, from: old pose of the waypoint GraphNav is localized on
        * where the body is relative to it (now) = old body pose; new body pose from today's
        odometry. None (and T unchanged) if GraphNav is not localized on a waypoint whose pose
        is known, or if the estimate jumped and the next readings do not confirm it.
        """
        new, name = self._transform_now(where)
        if new is None:
            print(f"{LOG_TAG} {name}")
            return None
        if self.T is None:
            print(f"{LOG_TAG} Mission frame -> today's frame: {new} (on {name}).")
        else:
            dm, dg = self._shift(new, self.T)
            if dm > FRAME_JUMP_M or dg > FRAME_JUMP_DEG:
                if not self._jump_confirmed(new, where):
                    return None
            elif dm > 0.10 or dg > 3.0:
                print(f"{LOG_TAG} Frame estimate changed by {dm:.2f} m and {dg:.1f} degrees {where} "
                      f"(on {name}): GraphNav corrected today's drift.")
        self.T = new
        # Waypoint poses in TODAY's frame: the relocalization and the arrival heading of
        # spot_explorer.graphnav use them, comparing with the current VISION position.
        self.ri.waypoint_poses = {
            v['name']: dict(zip(('x', 'y'), new.apply(v['x'], v['y'])),
                            yaw=new.apply_yaw(v['yaw']), waypoint_id=wid, t=None)
            for wid, v in self.map.ng['wp'].items()}
        return new

    def where_am_i(self):
        """Where the robot is in the mission map now: (x, y, heading in degrees) in the
        mission frame, the nearest GraphNav waypoint and its distance. For the check before
        starting (run.confirm_start)."""
        (x, y, yaw), old = self.robot_pose()
        fr = self.map.frame
        xm, ym = fr.from_vision(*old)
        wid = self.map.nearest_waypoint(old)
        w = self.map.ng['wp'][wid]
        return dict(x=xm, y=ym, heading_deg=float(np.degrees(wrap(yaw - self.T.th - fr.yaw))),
                    waypoint=w['name'], waypoint_m=float(np.hypot(w['x'] - old[0], w['y'] - old[1])),
                    seen=bool(self.map.gm.is_known(*old)),
                    occupied=bool(self.map.gm.is_occupied(*old)))

    def localization_ok(self, where):
        """
        Updates the transform; if that is impossible MAX_ROUNDS_WITHOUT_LOC times in a row, the
        robot (standing: this is called only between moves) tries to relocalize. False if it
        stays lost: then it does not move any more. Without localization the transform stays
        the old one, and with today's drift the robot would go to the wrong point of the map,
        believing it is right.
        """
        if self.update_transform(where) is not None:
            self.rounds_without_loc = 0
            return True
        self.rounds_without_loc += 1
        if self.rounds_without_loc < MAX_ROUNDS_WITHOUT_LOC:
            return True
        print(f"{LOG_TAG} LOST: {self.rounds_without_loc} readings in a row without GraphNav "
              f"localization. Stopped, trying to relocalize.")
        self.phase = 'lost'
        for how, fn in (("on the nearest waypoint", self._relocalize),
                        ("on the fiducial in view", self._relocalize_fiducial)):
            try:
                ok = fn()
            except Exception as e:
                print(f"{LOG_TAG} Relocalization {how} failed: {e}")
                ok = False
            if ok and self.update_transform(f"after relocalization {how}") is not None:
                print(f"{LOG_TAG} Relocalized {how}: resuming.")
                self.rounds_without_loc = 0
                self.phase = 'travelling'
                return True
        print(f"{LOG_TAG} Localization not recovered: the robot stays still. Bring it back near "
              f"the start (or a fiducial of the map) and restart.")
        return False

    def _relocalize(self):
        from spot_explorer.graphnav import localization
        return localization.graphnav_relocalize(self.ri, self.rsc, "during map navigation")

    def _relocalize_fiducial(self):
        return self.ri.initialize_with_fiducial(self.rsc, None)

    def robot_pose(self):
        """(x, y, yaw) in today's frame and (x, y) in the mission's VISION frame."""
        x, y, _z, q = connection.getPosition(self.rsc)
        return (x, y, yaw_of_quat(q)), self.T.inverse(x, y)

    # --- scan and maps -------------------------------------------------------------------
    def scan(self):
        """
        A scan as in the mission loop (attempt_scan): layers, processing, snapshot for the
        frontier; with the map update on it also goes into the maps (update_maps). The
        snapshot, or None if the scan does not arrive.
        """
        from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
        scan = self.local_grid.fetch_scan(self.rsc)
        if scan is None:
            return None
        v_T_body = spot_grid.get_a_tform_b(scan.proto.local_grid.transforms_snapshot,
                                           VISION_FRAME_NAME, BODY_FRAME_NAME)
        rx, ry = v_T_body.position.x, v_T_body.position.y
        ryaw = yaw_of_quat(v_T_body.rotation)
        self.local_grid.process_scan(scan, rx, ry, ryaw)
        ox, oy = self.local_grid.grid_origin(scan)
        snap = safe_frontier.make_grid_snapshot(
            scan.od, scan.rough, scan.is_valid, scan.valid, scan.terrain, scan.num_x, scan.num_y,
            ox, oy, scan.cell_size, rx, ry, unwritten=scan.unwritten, robot_yaw=ryaw)
        if self.update:
            self.update_maps(scan, rx, ry, ryaw)
        return snap

    def _estimate_height_offset(self, xo, yo, h_today, usable):
        """
        Today's heights are relative to where the robot was booted today, the map's to where
        it was booted on the day of the mission: without correcting them, a cell seen on both
        days would average two different heights, i.e. fake steps in the slope. Offset =
        median of (mission height - today's height) over the cells the terrain map already
        has; with fewer than MIN_HEIGHT_CELLS cells in common the previous offset stays (None
        if there is none yet).
        """
        h_old = self.map.tm._sample_heights(xo[usable], yo[usable])
        d = h_old - np.asarray(h_today, dtype=np.float64)[usable]
        d = d[np.isfinite(d)]
        if d.size < MIN_HEIGHT_CELLS:
            return self.dz
        dz = float(np.median(d))
        if self.dz is None:
            print(f"{LOG_TAG} Height: the mission map is {dz:+.3f} m from today's heights "
                  f"(median over {d.size} cells in common, spread "
                  f"{float(np.percentile(np.abs(d - dz), 90)):.3f} m at 90%).")
        elif abs(dz - self.dz) > 0.05:
            print(f"{LOG_TAG} Height: offset changed from {self.dz:+.3f} to {dz:+.3f} m.")
        return dz

    def update_maps(self, scan, rx, ry, ryaw):
        """
        Today's scan into the mission's global maps, with the mission's own function
        (global_maps.fold_scan_into_global_maps): points and robot pose taken to the mission
        frame with the current transform, heights corrected by the estimated offset
        (_estimate_height_offset). Then the PRM is re-evaluated only where new data arrived
        (incremental rebuild_prm_edges, as at the start of a mission attempt) and the seen-area
        filter is redone in that area. The occupied cells of the mission become free only
        after GLOBAL_OCC_CONFIRM_OBS free observations (mission_map.prepare_hysteresis), and new
        ones appear with the same rules as in the mission.
        Returns the updated rectangle (mission VISION) or None.
        """
        t0 = time.perf_counter()
        m = self.map
        sc = self._resample(scan)
        if sc is None:
            return None
        xo, yo, terrain, omask, valid, od, shape = sc
        usable = spot_grid.slope_usable_mask(valid, od, shape).ravel()
        self.dz = self._estimate_height_offset(xo, yo, terrain, usable)
        pose_old = (*self.T.inverse(rx, ry), wrap(ryaw - self.T.th))
        pts_old = np.column_stack([xo, yo, terrain + (self.dz or 0.0)])
        if self.dz is None:
            # No height offset yet: only the occupancy is updated, not the terrain.
            gm_mask, core = global_maps.mask_for_global_map(omask, valid, pts_old, pose_old, return_core=True)
            m.gm.update(pts_old, gm_mask, walked=core)
        else:
            global_maps.fold_scan_into_global_maps(m.gm, m.tm, pts_old, terrain + self.dz, omask, valid,
                                                   od, shape, pose_old)
        rect = (float(xo.min()), float(xo.max()), float(yo.min()), float(yo.max()))
        prm = m.prm
        prm.dirty_rects = list(getattr(prm, 'dirty_rects', [])) + [rect]
        robot_node = prm.robot_node
        with quiet():
            mission_prm.rebuild_prm_edges(prm, m.gm, incremental=True)
        if robot_node is not None:
            prm.set_robot_node(robot_node, 0.0)
        # Edges re-evaluated on ground without data come back without vetoes: out again if not
        # seen. The rectangle grows by the maximum edge length: a re-evaluated edge can leave
        # the window.
        e = m.prm.max_edge_length + 0.5
        limit_to_seen_area(prm, m.gm, rect=(rect[0] - e, rect[1] + e, rect[2] - e, rect[3] + e))
        self.n_scans_in_map += 1
        self.t_maps_s += time.perf_counter() - t0
        return rect

    def _resample(self, scan):
        """
        The scan on the grid of the MAP (mission frame, same resolution). The two grids are
        rotated with respect to each other by the angle between the frames: moving the scan
        points one by one, some map cells would never be hit (and an obstacle that went away
        would stay there for good, as isolated cells). Here each map cell inside the window
        reads the scan cell under it (nearest). Returns (x, y, today's height, obstacle mask,
        valid, obstacle_distance, shape), flattened, or None if the window is empty.
        """
        r = self.map.gm.resolution
        ny, nx, cs = scan.num_y, scan.num_x, scan.cell_size
        ox, oy = self.local_grid.grid_origin(scan)
        cx = np.array([ox, ox + nx * cs, ox, ox + nx * cs])
        cy = np.array([oy, oy, oy + ny * cs, oy + ny * cs])
        bx, by = self.T.inverse_many(cx, cy)
        gx = np.arange(int(np.floor(bx.min() / r)), int(np.ceil(bx.max() / r)))
        gy = np.arange(int(np.floor(by.min() / r)), int(np.ceil(by.max() / r)))
        if not gx.size or not gy.size:
            return None
        GY, GX = np.meshgrid(gy, gx, indexing='ij')
        xo, yo = ((GX + 0.5) * r).ravel(), ((GY + 0.5) * r).ravel()
        xn, yn = self.T.apply_many(xo, yo)
        col = np.floor((xn - ox) / cs).astype(np.int64)
        row = np.floor((yn - oy) / cs).astype(np.int64)
        inside = (row >= 0) & (row < ny) & (col >= 0) & (col < nx)
        k = (np.clip(row, 0, ny - 1) * nx + np.clip(col, 0, nx - 1))
        terrain = np.where(inside, np.asarray(scan.terrain, dtype=np.float64).ravel()[k], np.nan)
        omask = np.where(inside, np.asarray(scan.obstacle_mask, dtype=np.float64).ravel()[k], 0.0)
        valid = inside & np.asarray(scan.is_valid, dtype=bool).ravel()[k]
        od = np.where(inside, np.asarray(scan.od, dtype=np.float64).ravel()[k], 9.0)
        return xo, yo, terrain, omask, valid, od, GX.shape

    def save_maps(self, folder, quiet_=False):
        """The updated maps (mission frame), to reuse with --maps."""
        if not self.n_scans_in_map:
            return
        try:
            for name, m in (('global_occupancy_map.pkl', self.map.gm), ('global_terrain_map.pkl', self.map.tm)):
                path = os.path.join(folder, name)
                m.save_map(path + '.tmp')
                os.replace(path + '.tmp', path)
            if quiet_:
                return
            print(f"{LOG_TAG} Maps updated with {self.n_scans_in_map} scans of today "
                  f"({self.t_maps_s:.1f} s in all) saved in {folder}: to start again from here, "
                  f"--maps {folder}")
        except Exception as e:
            print(f"{LOG_TAG} Updated maps not saved: {e}")

    def drop_edges_seen_blocked(self, snap):
        """
        Only with the map update off (the mission map does not know about obstacles that
        appeared later). The PRM edges that fall in the scan window and pass, on a cell SEEN
        now and not under the body, closer than FRONTIER_CLEARANCE_M to an obstacle are dropped
        (like the mission's live veto, roadmap.set_live_obstacles). The threshold is the
        frontier's, not the planner's: an edge the frontier would accept is not removed, so
        narrow passages stay. Permanent for this session: an obstacle that goes away (a
        person) does not give the edges back. Returns how many edges were dropped.
        """
        prm = self.map.prm
        ny, nx = snap.obstacle_dist.shape
        x0, y0 = snap.origin_x, snap.origin_y
        x1, y1 = x0 + nx * snap.cell_size, y0 + ny * snap.cell_size
        node_new = {}
        for n, (x, y) in prm.nodes.items():
            px, py = self.T.apply(x, y)
            if x0 <= px <= x1 and y0 <= py <= y1:
                node_new[n] = (px, py)
        dropped = 0
        for a, (ax, ay) in node_new.items():
            for b, _w in list(prm.edges.get(a, ())):
                if b <= a or b not in node_new:
                    continue
                bx, by = node_new[b]
                k = max(2, int(np.ceil(np.hypot(bx - ax, by - ay) / 0.05)) + 1)
                t = np.linspace(0.0, 1.0, k)
                xs, ys = ax + t * (bx - ax), ay + t * (by - ay)
                rows, cols, inside = safe_frontier._cells(snap, xs, ys)
                keep = inside & snap.seen[rows, cols] & ~safe_frontier._under_body(snap, xs, ys)
                if np.any(snap.obstacle_dist[rows[keep], cols[keep]] < geometry.FRONTIER_CLEARANCE_M):
                    prm.mark_edge_invalid(a, b)
                    dropped += 1
        if dropped:
            print(f"{LOG_TAG} Current scan: {dropped} PRM edges pass close to obstacles that the "
                  f"mission map does not have (or did not have there): dropped.")
        return dropped

    # --- battery and home ------------------------------------------------------------------
    def battery_low(self, where=None):
        """True if the battery is below the return threshold (BatteryMonitor). During the
        return home it no longer counts: the robot goes home anyway."""
        if self.battery is None or self.returning:
            return False
        return bool(self.battery.check(where))

    def wp0_xy(self):
        wid = self.map.ng['by_name'].get('wp_0')
        if wid is None:
            return None
        w = self.map.ng['wp'][wid]
        return (w['x'], w['y'])

    def go_home(self, why):
        """
        Back to the mission start (wp_0). With a low battery first GraphNav, as at the end of
        the mission (graphnav_return_to_start: a path already walked, no stop at every scan),
        then the PRM if GraphNav does not arrive; asked by hand, first the PRM (with the
        GraphNav fallback). True if it arrives.
        """
        xy = self.wp0_xy()
        if xy is None:
            print(f"{LOG_TAG} Cannot go home: wp_0 is not in the GraphNav graph.")
            return False
        self.returning = True
        print(f"\n{LOG_TAG} ===== Going home (wp_0): {why} =====")
        try:
            if why == 'battery':
                self.phase = 'returning'
                self.goal = (xy, 'home (wp_0)')
                self._write_state()
                if self._graphnav_to_wp0():
                    self.phase = 'arrived'
                    self._row_here()
                    self._write_state()
                    return True
                print(f"{LOG_TAG} GraphNav did not reach wp_0: trying with the PRM.")
            return self.go(xy, 'home (wp_0)')
        finally:
            self.returning = False

    def _graphnav_to_wp0(self):
        from spot_explorer.graphnav import navigation
        return navigation.graphnav_return_to_start(self.ri, self.rsc, self.cc) == 'arrived'

    # --- logs ------------------------------------------------------------------------------
    def _write_json(self, name, data):
        if not self.log_folder:
            return
        try:
            path = os.path.join(self.log_folder, name)
            with open(path + '.tmp', 'w') as f:
                json.dump(data, f)
            os.replace(path + '.tmp', path)        # readers never see a half-written file
        except Exception as e:
            print(f"{LOG_TAG} {name} not written: {e}")

    def _write_info(self):
        """navigation_info.json, once: what the viewer needs and does not change (mission
        frame, GraphNav waypoints in the mission frame, wp_0)."""
        fr = self.map.frame
        wps = [list(fr.from_vision(v['x'], v['y'])) for v in (self.map.ng or {}).get('wp', {}).values()]
        xy = self.wp0_xy() if self.map.ng else None
        self._write_json('navigation_info.json', dict(
            mission=self.map.name, frame=[fr.ox, fr.oy, fr.yaw], waypoints=wps,
            wp0=list(fr.from_vision(*xy)) if xy else None, start=time.time()))

    def _write_state(self):
        """
        navigation_state.json (rewritten every round: where the robot is, what it is doing,
        towards where and along which path, all in the mission frame) and, every
        MAP_IMAGE_EVERY_S, navigation_map.npz with the image of the updated map. Read by
        tools/navigation_viewer.py. Every PKL_EVERY_S also the full maps. Never raises.
        """
        if not self.log_folder:
            return
        try:
            fr = self.map.frame
            robot = None
            if self.T is not None:
                (x, y, yaw), _old = self.robot_pose()
                xm, ym = fr.from_vision(*self.T.inverse(x, y))
                robot = [xm, ym, wrap(yaw - self.T.th - fr.yaw)]
            goal = None
            if self.goal is not None:
                goal = [*fr.from_vision(*self.goal[0]), self.goal[1]]
            prm = self.map.prm
            path = [list(fr.from_vision(*prm.nodes[n])) for n in self.path if n in prm.nodes]
            b = None if self.battery is None else records.MISSION_STATS.get('battery_last_pct')
            self._write_json('navigation_state.json', dict(
                t=time.time(), phase=self.phase, robot=robot, goal=goal, path=path,
                plan=self.n_plan, round=self.round, battery_pct=b,
                decision=self.last.get('decision'), frontier=self.last.get('frontier'),
                lost=self.rounds_without_loc > 0, scans_in_map=self.n_scans_in_map))
            now = time.time()
            if self.n_scans_in_map and now - self._t_image >= MAP_IMAGE_EVERY_S:
                self._t_image = now
                img, ext = mission_raster(self.map.gm, fr)
                path = os.path.join(self.log_folder, 'navigation_map.npz')
                np.savez_compressed(path + '.tmp.npz', img=img, ext=np.array(ext))
                os.replace(path + '.tmp.npz', path)
            if self.n_scans_in_map and now - self._t_pkl >= PKL_EVERY_S:
                self._t_pkl = now
                self.save_maps(self.log_folder, quiet_=True)
        except Exception as e:
            print(f"{LOG_TAG} State for the viewer not written: {e}")

    def _csv_row(self, rx, ry, ryaw):
        """One row of navigation.csv per round, in the mission frame."""
        fr = self.map.frame
        xm, ym = fr.from_vision(*self.T.inverse(rx, ry))
        records.append_csv('navigation.csv', NAVIGATION_COLUMNS, [[
            round(time.time(), 2), self.goal[1] if self.goal else '', self.n_plan, self.round,
            self.phase, round(xm, 3), round(ym, 3), round(np.degrees(wrap(ryaw - self.T.th - fr.yaw)), 1),
            self.path[0] if self.path else '', self.path[1] if len(self.path) > 1 else '',
            self.last.get('decision', ''), self.last.get('frontier', ''),
            self.last.get('advance', ''), self.last.get('remaining', ''),
            records.MISSION_STATS.get('battery_last_pct', ''), self.rounds_without_loc,
            round(self.T.tx, 3), round(self.T.ty, 3), round(np.degrees(self.T.th), 2),
            '' if self.dz is None else round(self.dz, 3), self.n_scans_in_map]])

    def _row_here(self):
        """Row of navigation.csv with the current pose (on arrival, at the end of a goal): the
        round rows are written before moving, and without this the last leg would be missing."""
        if self.T is None:
            return
        try:
            (x, y, yaw), _old = self.robot_pose()
            self.last = dict(decision=self.phase)
            self._csv_row(x, y, yaw)
        except Exception as e:
            print(f"{LOG_TAG} Final row of navigation.csv not written: {e}")

    # --- navigation ------------------------------------------------------------------------
    def go(self, goal_xy, descr, graphnav_fallback=True):
        """
        Takes the robot towards goal_xy (mission VISION). First the PRM, replanning after every
        block (at most MAX_PLANS plans); then, if needed, GraphNav to the waypoint nearest to the
        goal and a last attempt with the PRM from there. True if it arrives.
        """
        print(f"\n{LOG_TAG} ===== Goal: {descr} =====")
        self.end_reason = None
        self.goal = (tuple(goal_xy), descr)
        self.phase = 'returning' if self.returning else 'travelling'
        for n_plan in range(1, MAX_PLANS + 1):
            self.n_plan = n_plan
            if self.battery_low("before the plan"):
                return self._end('battery')
            if not self.localization_ok(f"before plan {n_plan}"):
                return self._end('lost')
            snap = self.scan()
            if snap is not None and not self.update:
                self.drop_edges_seen_blocked(snap)
            _new, old = self.robot_pose()
            plan = self.map.plan(old, goal_xy)
            if plan is None:
                print(f"{LOG_TAG} No PRM path from here towards {descr}.")
                break
            print(f"{LOG_TAG} Plan {n_plan}: {len(plan['path']) - 1} edges, {plan['length']:.1f} m, "
                  f"from node {plan['start']} to node {plan['goal']}"
                  + (f" ({plan['missing_m']:.2f} m from the goal: no closer point on seen ground)"
                     if plan['missing_m'] > 0.3 else "") + ".")
            self.path = list(plan['path'])
            outcome = self._follow(plan)
            if outcome == 'arrived':
                _new, old = self.robot_pose()
                print(f"{LOG_TAG} ARRIVED: {np.hypot(old[0] - goal_xy[0], old[1] - goal_xy[1]):.2f} m "
                      f"from the goal (mission frame).")
                self.path = []
                self.phase = 'arrived'
                self._row_here()
                self._write_state()
                return True
            if outcome in ('fallen', 'battery', 'lost'):
                return self._end(outcome)
        if not graphnav_fallback:
            return self._end(None)
        return self._graphnav_fallback(goal_xy, descr)

    def _end(self, reason):
        """Goal not reached: why (end_reason), state for the viewer."""
        self.end_reason = reason
        self.path = []
        self.phase = {'battery': 'battery low', 'lost': 'lost', 'fallen': 'fallen'}.get(reason, 'not arrived')
        self._row_here()
        self._write_state()
        return False

    def _graphnav_fallback(self, goal_xy, descr):
        from spot_explorer.graphnav import navigation
        wid = self.map.nearest_waypoint(goal_xy)
        if wid is None:
            return False
        w = self.map.ng['wp'][wid]
        print(f"{LOG_TAG} Fallback: GraphNav to {w['name']}, "
              f"{np.hypot(w['x'] - goal_xy[0], w['y'] - goal_xy[1]):.2f} m from the goal.")
        face = self.T.apply(*goal_xy)
        self.phase = 'GraphNav fallback'
        self._write_state()
        if not navigation.graphnav_navigate_to(self.ri, wid, self.rsc, self.cc,
                                               f"fallback towards {descr}", face_xy=face):
            return self._end(None)
        return self.go(goal_xy, descr + " (after the fallback)", graphnav_fallback=False)

    def _follow(self, plan):
        """
        The path, one round per scan: safe frontier on the rest of the path (in today's frame),
        decision as in the mission (decisions.decide_next_move), move with a stop mid-way if
        the frontier shortens. 'arrived', 'blocked' (edge dropped: replan), 'fallen', 'battery'
        (below the return threshold) or 'lost' (GraphNav not localized and relocalization
        failed): the last three stop the goal.
        """
        prm = self.map.prm
        robot_node = plan['start']
        wps_old = [(n, *prm.nodes[n]) for n in plan['path'][1:]]
        no_progress = 0
        while wps_old:
            self.round += 1
            self.path = [robot_node] + [n for n, _x, _y in wps_old]
            if self.battery_low():
                print(f"{LOG_TAG} Battery below the return threshold: stopping, going home.")
                return 'battery'
            if not self.localization_ok("in the round"):
                return 'lost'
            snap = self.scan()
            if snap is None:
                print(f"{LOG_TAG} Scan failed: retrying.")
                time.sleep(0.3)
                continue
            rx, ry, ryaw = snap.robot_x, snap.robot_y, snap.robot_yaw
            seq = [robot_node] + [n for n, _x, _y in wps_old]
            if self.update:
                # Maps just updated: broken path or a clearly better one -> switch now, from the
                # robot's node (as _replan in the mission).
                new = prm.find_path_dijkstra(robot_node, plan['goal'], current_path_ids=seq,
                                             margin=NEW_PATH_MARGIN)
                if new is None:
                    print(f"{LOG_TAG} With the updated maps the path no longer holds and there is no "
                          f"other from here: replanning from scratch.")
                    return 'blocked'
                if new != seq:
                    print(f"{LOG_TAG} Maps updated: new path, {prm.path_length(new):.1f} m "
                          f"instead of {prm.path_length(seq):.1f} m"
                          + (" (the old one used a removed edge)"
                             if not np.isfinite(prm.compute_path_cost(seq, allow_direct_first_hop=True))
                             else "") + ".")
                    wps_old = [(n, *prm.nodes[n]) for n in new[1:]]
                    no_progress = 0
                    if not wps_old:
                        return 'arrived'
            # Like [IMMEDIATE] in the mission: an edge of the rest of the path that the scan sees
            # blocked now makes the robot replan at once, without waiting for the standing frontier.
            elif self.drop_edges_seen_blocked(snap):
                if any((min(a, b), max(a, b)) in prm.tracker_blocked_edges for a, b in zip(seq[:-1], seq[1:])):
                    print(f"{LOG_TAG} The path uses an edge just dropped: replanning at once.")
                    return 'blocked'
            wps = [(n, *self.T.apply(x, y)) for n, x, y in wps_old]
            polyline = [(rx, ry)] + [(x, y) for _, x, y in wps]
            frontier = safe_frontier.compute_safe_frontier(polyline, snap)
            print(safe_frontier.describe_frontier(frontier))
            version = self.tracker.update_path(polyline[1:])
            dec = decisions.decide_next_move((rx, ry), wps, frontier, no_progress, in_target_cell=True)
            self.path = [robot_node] + [n for n, _x, _y in wps_old]
            self.last = dict(decision=dec['kind'], frontier=frontier['reason'],
                             advance=round(safe_frontier.allowed_advance(frontier), 2),
                             remaining=round(frontier['path_len'], 2))
            self._csv_row(rx, ry, ryaw)
            self._write_state()

            if dec['kind'] == 'reached_node':
                robot_node = wps_old.pop(0)[0]
                prm.set_robot_node(robot_node, 0.0)
                continue
            if dec['kind'] == 'arrived':
                print(f"{LOG_TAG} The frontier does not allow the last {frontier['path_len']:.2f} m: "
                      f"within the {P.GOAL_REACHED_TOLERANCE_M:.2f} m tolerance, counts as arrived.")
                return 'arrived'
            if dec['kind'] == 'wait':
                no_progress += 1
                print(f"{LOG_TAG} Cannot advance: looking again while standing "
                      f"({no_progress}/{P.FRONTIER_BLOCK_CONFIRM_SCANS}).")
                time.sleep(P.FRONTIER_RESCAN_WAIT_S)
                continue
            if dec['kind'] == 'blocked':
                k = dec['segment']
                a = robot_node if k == 0 else wps_old[k - 1][0]
                b = wps_old[min(k, len(wps_old) - 1)][0]
                prm.mark_edge_invalid(a, b)
                print(f"{LOG_TAG} Block confirmed while standing on edge {a}->{b}: dropped, replanning.")
                return 'blocked'

            # --- move ---------------------------------------------------------------------
            rot = decisions.rotation_plan(ryaw, (rx, ry), dec['target'], frontier['od_robot'], snap=snap)
            if not rot['turn']:
                a, b = robot_node, wps_old[0][0]
                prm.mark_edge_invalid(a, b)
                print(f"{LOG_TAG} {rot['dyaw_deg']:+.0f} degrees needed towards node {b} but the body "
                      f"would touch ({rot['why']}): dropping edge {a}->{b} and replanning.")
                return 'blocked'
            n_legs = int(dec.get('nodes', 1)) if dec['full'] else 0
            for leg in range(max(1, n_legs)):
                if not wps_old:
                    break
                nid, ox, oy = wps_old[0]
                tx, ty = self.T.apply(ox, oy) if n_legs else dec['target']
                cx, cy, cz, q = connection.getPosition(self.rsc)
                # navigate_to reads only the orientation (rotation) of the body pose
                v_T_body = SimpleNamespace(position=SimpleNamespace(x=cx, y=cy, z=cz), rotation=q)
                # Gait from the terrain of the leg, in the mission frame (map and PRM are there).
                o_from = self.T.inverse(cx, cy)
                o_to = self.T.inverse(tx, ty)
                long_s, lat_s, rough = prm.sample_terrain_between_points(
                    o_from[0], o_from[1], o_to[0], o_to[1], node_idx1=robot_node, node_idx2=nid)
                mob = motion.select_mobility_params(max(long_s, lat_s), rough)
                print(f"{LOG_TAG} Leg {robot_node}->{nid}"
                      + ("" if n_legs else f" (partial step of {dec['dist']:.2f} m)")
                      + f": towards ({tx:.2f}, {ty:.2f}).")
                ok, walked, commanded, why = self._move(
                    tx, ty, cx, cy, v_T_body, version, mob,
                    seg=dict(node_from=robot_node, node_to=nid, long_slope=long_s, lat_slope=lat_s,
                             roughness=rough))
                if motion.check_and_recover_fall(self.rsc, self.cc, None, "during map navigation"):
                    return 'fallen'
                if why is not None:
                    print(f"{LOG_TAG} Stopped mid-way: {why}. Looking again.")
                    break
                if not ok:
                    print(f"{LOG_TAG} Move failed ({walked:.2f} m of {commanded:.2f}): dropping "
                          f"edge {robot_node}->{nid} and replanning.")
                    prm.mark_edge_invalid(robot_node, nid)
                    return 'blocked'
                if n_legs:
                    robot_node = wps_old.pop(0)[0]
                    prm.set_robot_node(robot_node, 0.0)
                if leg + 1 < n_legs:
                    version = self.tracker.update_path([self.T.apply(x, y) for _, x, y in wps_old])
            no_progress = 0
        return 'arrived'

    def _move(self, tx, ty, rx, ry, v_T_body, version, mob, seg=None):
        """motion.navigate_to with the mid-move stop of the mission loop (see
        attempt_legs): (success, walked, commanded, reason or None). As in the mission, the
        leg goes into trajectory.csv (move 'mapnav') and segments.csv, with the real tilt of
        the body."""
        state = {'bad': 0, 'why': None}
        t0 = time.time()

        def _abort():
            f = self.tracker.get_frontier()
            if f is None or f.get('path_version') != version or f['time'] <= t0:
                return False
            frx, fry = f['robot_xy']
            remaining = float(np.hypot(tx - frx, ty - fry))
            reach = safe_frontier.allowed_advance(f)
            if (f['reason'] == 'end_of_data'
                    and float(np.hypot(frx - rx, fry - ry)) < P.ABORT_IGNORE_DATA_EDGE_UNTIL_M):
                state['bad'] = 0
                return False
            if reach + P.FRONTIER_ABORT_TOLERANCE_M < remaining:
                state['bad'] += 1
                if state['bad'] >= P.FRONTIER_ABORT_CONFIRM:
                    state['why'] = (f"the frontier shortened ({f['reason']}): {reach:.2f} m reachable "
                                    f"of {remaining:.2f} m")
                    return True
            else:
                state['bad'] = 0
            return False

        tilt = records.BodyTiltRecorder(self.rsc)
        tilt.start()
        try:
            ok, walked, commanded = motion.navigate_to(tx, ty, rx, ry, self.rsc, self.cc, v_T_body,
                                                       should_abort=_abort, **mob)
        finally:
            ts = tilt.stop() or {}
        try:   # log only: must never stop the navigation
            seg = seg or {}
            records.append_trajectory(list(getattr(tilt, 'samples', []) or []), 'mapnav', (tx, ty),
                                      ['', '', '', self.round, seg.get('node_from', ''), seg.get('node_to', '')])
            records.log_segment(dict(
                timestamp=datetime.now().strftime('%Y-%m-%d %H:%M:%S'), iteration=self.n_plan,
                node_from=seg.get('node_from'), node_to=seg.get('node_to'),
                x_from=rx, y_from=ry, x_to=tx, y_to=ty, dist_m=float(np.hypot(tx - rx, ty - ry)),
                slope_source=getattr(self.map.prm, 'last_slope_source', None),
                long_slope=seg.get('long_slope'), lat_slope=seg.get('lat_slope'),
                roughness=seg.get('roughness'), tier=records.MISSION_STATS.get('last_tier'),
                gait_override=False, success=bool(ok), dist_traveled_m=walked, dist_commanded_m=commanded,
                duration_s=time.time() - t0,
                roll_start_deg=ts.get('roll_start'), pitch_start_deg=ts.get('pitch_start'),
                roll_end_deg=ts.get('roll_end'), pitch_end_deg=ts.get('pitch_end'),
                roll_peak_abs_deg=ts.get('roll_peak_abs'), pitch_peak_abs_deg=ts.get('pitch_peak_abs'),
                tilt_samples=ts.get('samples'), tilt_errors=ts.get('errors')))
        except Exception as e:
            print(f"{LOG_TAG} Leg not logged: {e}")
        return ok, walked, commanded, state['why']
