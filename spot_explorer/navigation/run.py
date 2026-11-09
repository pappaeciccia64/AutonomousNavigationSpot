"""
Command line of map navigation (run_navigation.py): "go there" on an area already explored by a
finished mission, using its global maps, its PRM and its downloaded GraphNav graph.

Without the robot (plan preview; coordinates in the mission frame: x ahead, y to the left of
the start, in metres; headings in degrees from x towards y):
    python3 run_navigation.py 26-10-2026_15-11-21 --to 6,2 --preview plan.png
    python3 run_navigation.py 26-10-2026_15-11-21 --cell 1,3 --from 4,0 --preview plan.png
    python3 run_navigation.py 26-10-2026_15-11-21 --click --preview plan.png
    python3 run_navigation.py 26-10-2026_15-11-21 --route gate,shed,home --preview route.png
Places and routes (saved in MissionMap/Mission_<name>/places.json):
    python3 run_navigation.py 26-10-2026_15-11-21 --add-place gate 6,2,90     # X,Y[,HEADING]
    python3 run_navigation.py 26-10-2026_15-11-21 --add-route patrol gate,shed,home
    python3 run_navigation.py 26-10-2026_15-11-21 --list-places
    python3 run_navigation.py 26-10-2026_15-11-21 --remove-place gate
On the robot (robot at the start of the mission with the start tag in view, or anywhere with
--start-at):
    python3 run_navigation.py 26-10-2026_15-11-21 --to 6,2 [--heading 90]
    python3 run_navigation.py 26-10-2026_15-11-21 --place gate
    python3 run_navigation.py 26-10-2026_15-11-21 --route patrol --loops 3 --dwell 10
    python3 run_navigation.py 26-10-2026_15-11-21 --interactive    # asks for one goal after another
    python3 run_navigation.py 26-10-2026_15-11-21 --click          # goals with a click
    python3 run_navigation.py 26-10-2026_15-11-21 --home           # back to wp_0
    python3 run_navigation.py 26-10-2026_15-11-21 --to 6,2 --start-at 4,1,180   # robot not at the start
    python3 run_navigation.py 26-10-2026_15-11-21 --click --start-at click      # say where it is on the map
Live, in a second terminal: python3 tools/navigation_viewer.py
Without a mission name: the latest one with global maps, log and GraphNav graph.
"""
import argparse
import os
import re
import sys
import time
from datetime import datetime
from types import SimpleNamespace

import numpy as np

from spot_explorer import REPO_ROOT
from spot_explorer import records
from spot_explorer.navigation.mission_map import LOG_TAG, MissionMap, latest_complete_mission
from spot_explorer.navigation.navigator import HOME, MapNavigator
from spot_explorer.navigation.goal_picker import ClickGoalPicker, window_available
from spot_explorer.navigation.places import HOME_NAME, Places

ROUTE_COLUMNS = ['time', 'loop', 'stop', 'place', 'reached', 'seconds', 'missing_m', 'end_reason']


def _pair(s, kind=float):
    a, b = (t.strip() for t in s.split(','))
    return kind(a), kind(b)


def _pose(s):
    """X,Y[,HEADING] -> (x, y, heading or None)."""
    v = [float(t) for t in s.split(',')]
    if len(v) not in (2, 3):
        raise argparse.ArgumentTypeError("X,Y or X,Y,HEADING")
    return v[0], v[1], (v[2] if len(v) == 3 else None)


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description="Navigation on an area already explored, with the "
                                            "end-of-mission maps and the PRM graph")
    p.add_argument('mission', nargs='?', default=None,
                   help="mission name (e.g. 26-10-2026_15-11-21); default: the latest complete one")
    g = p.add_mutually_exclusive_group()
    g.add_argument('--to', type=_pair, metavar='X,Y',
                   help="goal in the mission frame: X ahead, Y to the left of the start (m)")
    g.add_argument('--cell', type=lambda s: _pair(s, int), metavar='R,C', help="cell centre")
    g.add_argument('--waypoint', metavar='wp_N', help="a waypoint of the GraphNav graph")
    g.add_argument('--place', metavar='NAME', help="a named place (see --add-place)")
    g.add_argument('--route', metavar='ROUTE',
                   help="a saved route, or places separated by commas ('home' = wp_0)")
    g.add_argument('--home', action='store_true',
                   help="on the robot: back to the mission start (wp_0)")
    g.add_argument('--interactive', action='store_true',
                   help="on the robot: asks for one goal after another (X,Y | c R,C | wp_N | NAME | "
                        "route R | save NAME | home | q)")
    g.add_argument('--click', action='store_true',
                   help="goal chosen with a click on the map (drag: heading; Enter confirms, p saves a "
                        "place, h home, Esc quits); on the robot one after another, with --preview only the plan")
    g.add_argument('--add-place', nargs=2, metavar=('NAME', 'X,Y[,H]'),
                   help="WITHOUT THE ROBOT: save a named place (mission frame, optional heading)")
    g.add_argument('--add-route', nargs=2, metavar=('NAME', 'A,B,C'),
                   help="WITHOUT THE ROBOT: save a named route of places ('home' allowed)")
    g.add_argument('--remove-place', metavar='NAME', help="WITHOUT THE ROBOT: remove a place or a route")
    g.add_argument('--list-places', action='store_true', help="WITHOUT THE ROBOT: list places and routes")
    p.add_argument('--heading', type=float, metavar='DEG',
                   help="arrival heading in the mission frame (degrees from x towards y); for a place it "
                        "overrides the place's own")
    p.add_argument('--loops', type=int, default=1, help="with --route: how many times (default 1)")
    p.add_argument('--dwell', type=float, default=0.0,
                   help="with --route: seconds to stand at each stop (a place's own dwell wins)")
    p.add_argument('--start-at', metavar='X,Y,H|click',
                   help="on the robot: where it is now (mission frame, heading needed), when it is not at "
                        "the mission start; 'click' to show it on the map")
    p.add_argument('--preview', metavar='PNG',
                   help="WITHOUT THE ROBOT: plan and save the figure of the plan, then exit")
    p.add_argument('--from', dest='from_', type=_pair, metavar='X,Y',
                   help="with --preview: start in the mission frame (default: the start, 0,0)")
    p.add_argument('--localization', choices=('auto', 'fiducial', 'wp_0'), default='auto',
                   help="how to localize on the uploaded graph (default: fiducial, then wp_0)")
    p.add_argument('--with-scans', action='store_true',
                   help="adds the scans saved in scans/ to the .pkl maps (more complete, slower)")
    p.add_argument('--maps', metavar='FOLDER',
                   help=".pkl maps from this folder instead of the end-of-mission ones (for example "
                        "those updated by a navigation: MissionLogs/Navigation_<date>/)")
    p.add_argument('--no-map-update', action='store_true',
                   help="today's scans do not go into the maps (only the edges seen blocked are dropped)")
    p.add_argument('--return-at-end', action='store_true',
                   help="on the robot: when the goals are done, back to wp_0 before sitting")
    p.add_argument('--no-fallback', action='store_true',
                   help="no GraphNav fallback when the PRM cannot make it")
    p.add_argument('--yes', action='store_true',
                   help="on the robot: start without asking to confirm where the robot is on the map")
    p.add_argument('--hostname', default='192.168.80.3')
    a = p.parse_args(argv)
    if a.loops < 1:
        p.error("--loops must be at least 1")
    if a.start_at and a.start_at != 'click':
        try:
            a.start_at = _pose(a.start_at)
        except (ValueError, argparse.ArgumentTypeError):
            p.error("--start-at wants X,Y,HEADING (mission frame) or 'click'")
        if a.start_at[2] is None:
            p.error("--start-at needs the heading too (X,Y,HEADING): GraphNav cannot guess it")
    return a


def goal_from_text(mission_map, s):
    """A goal typed in --interactive: X,Y | c R,C | wp_N | NAME (a place) | home. An optional
    heading after '@' (for example '6,2 @ 90' or 'gate @ 180')."""
    s = s.strip()
    heading = None
    if '@' in s:
        s, h = (t.strip() for t in s.split('@', 1))
        heading = float(h)
    if s.lower() in ('home', 'h', 'wp0'):
        return HOME
    if re.fullmatch(r'wp_\d+', s):
        return mission_map.goal_vision(waypoint=s, heading_deg=heading)
    if s in mission_map.places.places:
        return mission_map.goal_vision(place=s, heading_deg=heading)
    if s.lower().startswith('c') and ',' in s:
        return mission_map.goal_vision(cell=_pair(s[1:], int), heading_deg=heading)
    return mission_map.goal_vision(to=_pair(s), heading_deg=heading)


def route_goals(mission_map, spec, heading=None):
    """The stops of a route as goals: (name, goal) with goal = HOME or (xy, descr, heading)."""
    out = []
    for name in mission_map.places.route_stops(spec):
        out.append((name, HOME if name.lower() == HOME_NAME else mission_map.goal_vision(place=name)))
    return out


def preview(args):
    """Without the robot: plan from the start (or from --from) to the goal, or along the whole
    route, and figure."""
    mission_map = MissionMap.load(args.mission, with_scans=args.with_scans, maps_folder=args.maps)
    start = mission_map.frame.to_vision(*(args.from_ or (0.0, 0.0)))
    if args.route:
        return _preview_route(mission_map, start, args)
    if args.click:
        picker = ClickGoalPicker(mission_map, start, title=f"Mission {mission_map.name} (without the robot)")
        r = picker.choose()
        if r in (None, HOME):
            print(f"{LOG_TAG} No goal confirmed.")
            return False
        goal, descr, heading = r
    else:
        goal, descr, heading = mission_map.goal_vision(args.to, args.cell, args.waypoint, args.place, args.heading)
    plan = mission_map.plan(start, goal)
    if plan is None:
        print(f"{LOG_TAG} No path towards {descr}.")
    else:
        print(f"{LOG_TAG} Plan towards {descr}: {len(plan['path']) - 1} edges, {plan['length']:.1f} m "
              f"(straight line {np.hypot(goal[0] - start[0], goal[1] - start[1]):.1f} m), arriving "
              f"{plan['missing_m']:.2f} m from the goal"
              + (f", then facing {heading:+.0f} degrees" if heading is not None else "") + ".")
    mission_map.preview(args.preview, plan, start, goal, title=f"Mission {mission_map.name}: towards {descr}")
    return plan is not None


def _preview_route(mission_map, start, args):
    """The route leg by leg, from the start, each leg planned from where the previous ended."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    stops = route_goals(mission_map, args.route)
    wp0 = mission_map.ng['wp'][mission_map.ng['by_name']['wp_0']] if mission_map.ng and 'wp_0' in mission_map.ng['by_name'] else None
    fig, ax = plt.subplots(figsize=(10, 10))
    mission_map.draw_map(ax)
    here, total, ok = start, 0.0, True
    for loop in range(args.loops):
        for name, g in stops:
            xy = (wp0['x'], wp0['y']) if g == HOME else g[0]
            if xy is None:
                continue
            plan = mission_map.plan(here, xy)
            if plan is None:
                print(f"{LOG_TAG} Route: no path to {name}.")
                ok = False
                continue
            total += plan['length']
            if loop == 0:
                mission_map.draw_plan(ax, plan, None, xy, heading_deg=None if g == HOME else g[2])
            print(f"{LOG_TAG} Route leg to {name}: {plan['length']:.1f} m.")
            here = mission_map.prm.nodes[plan['goal']]
    print(f"{LOG_TAG} Route {args.route} x {args.loops}: {total:.1f} m in all.")
    ax.set_title(f"Mission {mission_map.name}: route {args.route} ({total:.1f} m"
                 + (f", {args.loops} loops" if args.loops > 1 else "") + ")")
    fig.tight_layout()
    fig.savefig(args.preview, dpi=130)
    plt.close(fig)
    print(f"{LOG_TAG} Preview saved to {args.preview}")
    return ok


def edit_places(args):
    """--add-place, --add-route, --remove-place, --list-places: only the places.json of the
    mission, no maps loaded."""
    name = args.mission or latest_complete_mission()
    if name is None:
        print(f"{LOG_TAG} No mission found.")
        return False
    name = name[len('Mission_'):] if name.startswith('Mission_') else name
    folder = os.path.join(REPO_ROOT, 'MissionMap', f'Mission_{name}')
    if not os.path.isdir(folder):
        print(f"{LOG_TAG} No folder {folder}.")
        return False
    places = Places(os.path.join(folder, 'places.json'))
    try:
        if args.add_place:
            x, y, h = _pose(args.add_place[1])
            places.add(args.add_place[0], x, y, heading_deg=h)
            print(f"{LOG_TAG} Place '{args.add_place[0]}' saved.")
        elif args.add_route:
            places.add_route(args.add_route[0], [t.strip() for t in args.add_route[1].split(',') if t.strip()])
            print(f"{LOG_TAG} Route '{args.add_route[0]}' saved.")
        elif args.remove_place:
            places.remove(args.remove_place)
            print(f"{LOG_TAG} '{args.remove_place}' removed.")
    except (ValueError, KeyError, argparse.ArgumentTypeError) as e:
        print(f"{LOG_TAG} {e}")
        return False
    print(f"{LOG_TAG} Mission {name}: {places.describe()}")
    return True


def confirm_start(nav, args, ask=input):
    """
    Before the first move: where the robot believes it is on the mission map, from GraphNav's
    localization, and a yes from the operator. A wrong localization (the wrong tag, a
    similar-looking place) would send the robot to the wrong point of the map believing it is
    right: this is the moment to see it, with the robot still standing at the start. The
    same position is in the viewer (navigation_state.json). --yes skips the question; without
    a terminal to ask on, and without --yes, the robot does not move.
    """
    w = nav.where_am_i()
    print(f"{LOG_TAG} Localized: the robot is at ({w['x']:.2f}, {w['y']:.2f}) of the mission frame "
          f"(x ahead, y to the left of the mission start), heading {w['heading_deg']:+.0f} degrees, "
          f"{w['waypoint_m']:.2f} m from {w['waypoint']}"
          + ("" if w['seen'] else "; NOT on ground seen by the mission")
          + ("; ON AN OCCUPIED CELL of the map" if w['occupied'] else "") + ".")
    nav._write_state()
    if getattr(args, 'yes', False):
        return True
    if not sys.stdin.isatty() and ask is input:
        print(f"{LOG_TAG} No terminal to confirm on: not moving (use --yes to start without asking).")
        return False
    try:
        answer = ask(f"{LOG_TAG} Is that where the robot really is? Start? [y/N] ")
    except EOFError:
        answer = ''
    if answer.strip().lower() in ('y', 'yes', 's', 'si'):
        return True
    print(f"{LOG_TAG} Not confirmed: not moving.")
    return False


def emergency_shutdown(robot, command_client, nav, log_folder, exc):
    """
    Session interrupted (exception or Ctrl+C) with the robot powered and the lease still held:
    as the mission does (mission.run._emergency_shutdown), first stop it, then sit it down and
    power off the motors -- unless it has fallen (it is already on the ground) -- and only then
    save the updated maps, which take seconds. Each step is tried on its own: never raises.
    """
    from spot_explorer.robot import motion
    from bosdyn.client.robot_command import RobotCommandBuilder
    fallen = isinstance(exc, motion.RobotFallenError)
    print(f"\n[EMERGENCY] Map navigation interrupted ({type(exc).__name__}): "
          + ("the robot is on the ground, " if fallen else "I stop the robot, sit it down, ")
          + "I save the maps.")
    if nav is not None:
        try:
            nav.tracker.clear_path()
            nav.tracker.stop()
        except BaseException:
            pass
    if not fallen and command_client is not None:
        try:
            motion.stop_robot(command_client, reason="map navigation interrupted")
            command_client.robot_command(RobotCommandBuilder.synchro_sit_command(),
                                         end_time_secs=time.time() + 20)
            time.sleep(3)
            robot.power_off(cut_immediately=False)
            print("[EMERGENCY] Robot sitting and motors off.")
        except BaseException as e:
            print(f"[EMERGENCY] I could not sit the robot down ({e!r}): use the tablet.")
    if nav is not None and log_folder:
        try:
            nav.phase = 'interrupted'
            nav._write_state()
            nav.save_maps(log_folder)
        except BaseException as e:
            print(f"[EMERGENCY] Maps not saved ({e!r}).")


def close_session(robot, command_client, nav, log_folder, monitor):
    """Normal end: sit down and power off first, then the saves (they take seconds)."""
    from bosdyn.client.robot_command import RobotCommandBuilder
    if nav is not None:
        nav.tracker.stop()
    try:
        command_client.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
        time.sleep(3)
        robot.power_off(cut_immediately=False)
    except Exception as e:
        print(f"{LOG_TAG} Sitting down / power off failed: {e}")
    if monitor is not None:
        monitor.check("end")
    if nav is not None:
        nav.save_maps(log_folder)


def on_robot(args):
    """
    Connection, graph on Spot (before powering the motors: it can take minutes), power on,
    localization, check of where the robot is, one or more goals, sitting down. On an
    exception or Ctrl+C: emergency_shutdown, then the error goes up.
    """
    import bosdyn.client.lease
    from bosdyn.client import robot_command
    from spot_explorer.robot import battery
    from spot_explorer.robot import connection
    from spot_explorer.records import MISSION_CTX, SegmentLog, install_stdout_tee, remove_stdout_tee

    mission_map = MissionMap.load(args.mission, with_scans=args.with_scans, maps_folder=args.maps)
    goals = []
    if args.home:
        goals.append(HOME)
    elif args.route:
        goals.append(('route', args.route))
        route_goals(mission_map, args.route)            # unknown places: error now, not on the robot
    elif not (args.interactive or args.click):
        goals.append(mission_map.goal_vision(args.to, args.cell, args.waypoint, args.place, args.heading))
    start_pose = args.start_at
    if start_pose == 'click':
        start_pose = ClickGoalPicker(mission_map, None, start=True,
                                     title=f"Mission {mission_map.name}: where is the robot now?").choose()
        if start_pose is None:
            print(f"{LOG_TAG} Start position not given: not starting.")
            return False

    log_folder = os.path.join(REPO_ROOT, 'MissionLogs',
                              f"Navigation_{datetime.now().strftime('%d-%m-%Y_%H-%M-%S')}")
    os.makedirs(log_folder, exist_ok=True)
    MISSION_CTX['log_folder'] = log_folder
    install_stdout_tee(os.path.join(log_folder, 'navigation_log.txt'))
    MISSION_CTX['segment_log'] = SegmentLog(os.path.join(log_folder, 'segments.csv'))
    print(f"{LOG_TAG} Map of mission {mission_map.name}; log in {log_folder}")
    print(f"{LOG_TAG} To follow it live, in another terminal: python3 tools/navigation_viewer.py {log_folder}")

    options = SimpleNamespace(name="mapNavigation", hostname=args.hostname, verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, lease_client, rsc, client_metadata = connection.setLogInfo(options)
    estop = connection.SimpleEstop(robot, options.name + "_estop")
    nav = cc = None
    try:
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            cc = robot.ensure_client(robot_command.RobotCommandClient.default_service_name)
            nav = MapNavigator(mission_map, robot, rsc, cc, client_metadata, log_folder)
            nav.init_maps(update=not args.no_map_update)
            monitor = battery.BatteryMonitor(rsc)
            nav.init_session(log_folder, battery=monitor)
            monitor.check("start")
            try:
                nav.upload_graph()
                robot.power_on()
                assert robot.is_powered_on(), "Robot power-on failed."
                robot_command.blocking_stand(cc)
                if not nav.localize(args.localization, start_pose=start_pose):
                    print(f"{LOG_TAG} Not localized on the map: not moving.")
                    ok = False
                elif not confirm_start(nav, args):
                    ok = False
                else:
                    nav.tracker.start()
                    ok = run_goals(nav, mission_map, args, goals)
            except BaseException as exc:
                emergency_shutdown(robot, cc, nav, log_folder, exc)
                raise
            close_session(robot, cc, nav, log_folder, monitor)
            return ok
    finally:
        try:
            estop.stop()
        except Exception:
            pass
        remove_stdout_tee()


def run_route(nav, mission_map, spec, args):
    """
    A route: its stops in order, --loops times, standing --dwell seconds (or the place's own
    dwell) at each stop reached. A stop not reached is skipped and the route goes on; battery
    low, robot lost or fallen end it. One row per stop in route.csv. True if every stop was
    reached.
    """
    stops = route_goals(mission_map, spec)
    reached = total = 0
    print(f"{LOG_TAG} Route {spec}: {' -> '.join(n for n, _ in stops)}"
          + (f", {args.loops} loops" if args.loops > 1 else "") + ".")
    for loop in range(1, args.loops + 1):
        for i, (name, g) in enumerate(stops, 1):
            if nav.battery_low("before a route stop"):
                nav.end_reason = 'battery'
                return False
            t0 = time.time()
            total += 1
            if g == HOME:
                ok = nav.go_home(f"route {spec}")
                xy = nav.wp0_xy()
            else:
                ok = nav.go(g[0], f"{g[1]} (route {spec}, loop {loop}, stop {i})",
                            graphnav_fallback=not args.no_fallback, heading_deg=g[2])
                xy = g[0]
            _here, old = nav.robot_pose() if nav.T is not None else (None, None)
            missing = float(np.hypot(old[0] - xy[0], old[1] - xy[1])) if old is not None and xy else ''
            records.append_csv('route.csv', ROUTE_COLUMNS, [[
                round(time.time(), 2), loop, i, name, int(bool(ok)), round(time.time() - t0, 1),
                round(missing, 2) if missing != '' else '', nav.end_reason or '']])
            if nav.end_reason in ('lost', 'fallen', 'battery'):
                return False
            if not ok:
                print(f"{LOG_TAG} Route {spec}: stop {name} NOT reached, going on with the next one.")
                continue
            reached += 1
            dwell = (mission_map.places.places.get(name, {}).get('dwell_s') or args.dwell or 0.0)
            if dwell > 0:
                print(f"{LOG_TAG} At {name}: standing {dwell:g} s.")
                nav.phase = f'at {name}'
                nav._write_state()
                time.sleep(dwell)
    print(f"{LOG_TAG} Route {spec}: {reached} of {total} stops reached.")
    return reached == total


def run_goals(nav, mission_map, args, goals):
    """
    The round of goals, robot already standing and localized: from the command line, from the
    terminal (--interactive) or with a click (--click). A goal is HOME, (xy, descr, heading)
    or ('route', spec). Battery low: back home and end. Robot lost or fallen: end, it stays
    where it is. True if the last goal (or route) was done.
    """
    ok = True
    while True:
        if nav.battery_low("before a goal"):
            nav.end_reason = 'battery'
        elif args.click:
            _new, here = nav.robot_pose()
            r = ClickGoalPicker(mission_map, here, title=f"Mission {mission_map.name}: where do I go?",
                                from_label='robot now').choose()
            if r is None:
                break
            goals = [r]
        elif args.interactive:
            try:
                s = input("\nGoal (X,Y | c R,C | wp_N | PLACE, optional '@ HEADING' | route R | save NAME | "
                          "home | q to finish): ")
            except EOFError:
                break
            s = s.strip()
            if s.lower() in ('q', 'quit', 'exit', ''):
                break
            try:
                if s.lower().startswith('save '):
                    nav.save_place_here(s[5:].strip())
                    continue
                if s.lower().startswith('route '):
                    route_goals(mission_map, s[6:].strip())
                    goals = [('route', s[6:].strip())]
                else:
                    goals = [goal_from_text(mission_map, s)]
            except (ValueError, KeyError) as e:
                print(f"{LOG_TAG} Not valid: {e}")
                continue
        if nav.end_reason != 'battery':
            if not goals:
                break
            g = goals.pop(0)
            if g == HOME:
                ok = nav.go_home("asked")
            elif g[0] == 'route':
                ok = run_route(nav, mission_map, g[1], args)
            else:
                ok = nav.go(g[0], g[1], graphnav_fallback=not args.no_fallback,
                            heading_deg=g[2] if len(g) > 2 else None)
                if not ok:
                    print(f"{LOG_TAG} Goal {g[1]} NOT reached.")
        if nav.end_reason == 'battery':
            print(f"{LOG_TAG} Battery below {nav.battery.threshold_pct:.0f}%: no more goals, going home.")
            nav.go_home('battery')
            return False          # not all the goals asked for were done
        if nav.end_reason in ('lost', 'fallen'):
            print(f"{LOG_TAG} Session interrupted ({nav.end_reason}): the robot stays where it is.")
            return False
        if not (args.interactive or args.click):
            break
    if getattr(args, 'return_at_end', False) and nav.end_reason is None:
        ok = nav.go_home("end of the goals") and ok
    return ok


def main(argv=None):
    args = _parse_args(argv)
    if args.add_place or args.add_route or args.remove_place or args.list_places:
        return edit_places(args)
    if (args.click or args.start_at == 'click') and not window_available():
        return False
    if args.preview:
        if not (args.to or args.cell or args.waypoint or args.place or args.route or args.click):
            print("With --preview a goal is needed: --to, --cell, --waypoint, --place, --route or --click.")
            return False
        return preview(args)
    if not (args.to or args.cell or args.waypoint or args.place or args.route or args.home
            or args.interactive or args.click):
        print("A goal is needed (--to, --cell, --waypoint, --place, --route, --home) or --interactive "
              "or --click.")
        return False
    from spot_explorer.robot import motion
    try:
        return on_robot(args)
    except motion.RobotFallenError as exc:
        print("\n" + "=" * 70)
        print(f"[FALL] MAP NAVIGATION INTERRUPTED: {exc}")
        print("[FALL] The robot is on the ground and has NOT tried to go back home.")
        print("[FALL] Take control from the tablet, check the robot, then stand it up by hand.")
        print("=" * 70)
        return False


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
