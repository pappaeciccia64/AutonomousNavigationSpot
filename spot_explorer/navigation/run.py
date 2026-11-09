"""
Command line of map navigation (run_navigation.py): "go there" on an area already explored by a
finished mission, using its global maps, its PRM and its downloaded GraphNav graph.

Without the robot (plan preview; coordinates in the mission frame: x ahead, y to the left of
the start, in metres):
    python3 run_navigation.py 26-10-2026_15-11-21 --to 6,2 --preview plan.png
    python3 run_navigation.py 26-10-2026_15-11-21 --cell 1,3 --from 4,0 --preview plan.png
    python3 run_navigation.py 26-10-2026_15-11-21 --click --preview plan.png
On the robot (robot at the start of the mission, with the start tag in view):
    python3 run_navigation.py 26-10-2026_15-11-21 --to 6,2
    python3 run_navigation.py 26-10-2026_15-11-21 --interactive    # asks for one goal after another
    python3 run_navigation.py 26-10-2026_15-11-21 --click          # goals with a click
    python3 run_navigation.py 26-10-2026_15-11-21 --home           # back to wp_0
Live, in a second terminal: python3 tools/navigation_viewer.py
Goals: --to X,Y (mission frame), --cell R,C (cell centre), --waypoint wp_N (a waypoint of the
GraphNav graph), --home, or --click: left click on the map, the path appears at once, Enter
confirms, h goes home, Esc quits. Without a mission name: the latest one with global maps, log
and GraphNav graph.
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
from spot_explorer.navigation.mission_map import LOG_TAG, MissionMap
from spot_explorer.navigation.navigator import HOME, MapNavigator
from spot_explorer.navigation.goal_picker import ClickGoalPicker, window_available


def _pair(s, kind=float):
    a, b = (t.strip() for t in s.split(','))
    return kind(a), kind(b)


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
    g.add_argument('--home', action='store_true',
                   help="on the robot: back to the mission start (wp_0)")
    g.add_argument('--interactive', action='store_true',
                   help="on the robot: asks for one goal after another (X,Y | c R,C | wp_N | home | q)")
    g.add_argument('--click', action='store_true',
                   help="goal chosen with a click on the map (Enter confirms, h home, Esc quits); on "
                        "the robot one after another, with --preview only the plan")
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
    p.add_argument('--hostname', default='192.168.80.3')
    return p.parse_args(argv)


def goal_from_text(mission_map, s):
    """A goal typed in --interactive: X,Y | c R,C | wp_N | home."""
    s = s.strip()
    if s.lower() in ('home', 'h', 'wp0'):
        return HOME
    if re.fullmatch(r'wp_\d+', s):
        return mission_map.goal_vision(waypoint=s)
    if s.lower().startswith('c'):
        return mission_map.goal_vision(cell=_pair(s[1:], int))
    return mission_map.goal_vision(to=_pair(s))


def preview(args):
    """Without the robot: plan from the start (or from --from) to the goal, and figure."""
    mission_map = MissionMap.load(args.mission, with_scans=args.with_scans, maps_folder=args.maps)
    start = mission_map.frame.to_vision(*(args.from_ or (0.0, 0.0)))
    if args.click:
        picker = ClickGoalPicker(mission_map, start, title=f"Mission {mission_map.name} (without the robot)")
        if picker.choose() in (None, HOME):
            print(f"{LOG_TAG} No goal confirmed.")
            return False
        goal = picker.goal
        descr = "the point chosen with a click"
    else:
        goal, descr = mission_map.goal_vision(args.to, args.cell, args.waypoint)
    plan = mission_map.plan(start, goal)
    if plan is None:
        print(f"{LOG_TAG} No path towards {descr}.")
    else:
        print(f"{LOG_TAG} Plan towards {descr}: {len(plan['path']) - 1} edges, {plan['length']:.1f} m "
              f"(straight line {np.hypot(goal[0] - start[0], goal[1] - start[1]):.1f} m), arriving "
              f"{plan['missing_m']:.2f} m from the goal.")
    mission_map.preview(args.preview, plan, start, goal, title=f"Mission {mission_map.name}: towards {descr}")
    return plan is not None


def on_robot(args):
    """Connection, graph on Spot, localization, one or more goals, sitting down."""
    import bosdyn.client.lease
    from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
    from spot_explorer.robot import battery
    from spot_explorer.robot import connection
    from spot_explorer.records import MISSION_CTX, SegmentLog, install_stdout_tee, remove_stdout_tee

    mission_map = MissionMap.load(args.mission, with_scans=args.with_scans, maps_folder=args.maps)
    goals = []
    if args.home:
        goals.append(HOME)
    elif not (args.interactive or args.click):
        goals.append(mission_map.goal_vision(args.to, args.cell, args.waypoint))

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
    try:
        with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            cc = robot.ensure_client(RobotCommandClient.default_service_name)
            robot.power_on()
            assert robot.is_powered_on(), "Robot power-on failed."
            blocking_stand(cc)
            nav = MapNavigator(mission_map, robot, rsc, cc, client_metadata, log_folder)
            nav.init_maps(update=not args.no_map_update)
            monitor = battery.BatteryMonitor(rsc)
            nav.init_session(log_folder, battery=monitor)
            monitor.check("start")
            nav.upload_graph()
            if not nav.localize(args.localization):
                print(f"{LOG_TAG} Not localized on the map: not moving.")
                return False
            nav.tracker.start()
            try:
                return run_goals(nav, mission_map, args, goals)
            finally:
                nav.tracker.stop()
                nav.save_maps(log_folder)
                monitor.check("end")
                try:
                    cc.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
                    time.sleep(3)
                    robot.power_off(cut_immediately=False)
                except Exception as e:
                    print(f"{LOG_TAG} Sitting down / power off failed: {e}")
    finally:
        try:
            estop.stop()
        except Exception:
            pass
        remove_stdout_tee()


def run_goals(nav, mission_map, args, goals):
    """
    The round of goals, robot already standing and localized: from the command line, from the
    terminal (--interactive) or with a click (--click). Battery low: back home and end. Robot
    lost or fallen: end, it stays where it is. True if the last goal was reached.
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
                s = input("\nGoal (X,Y ahead/left | c R,C | wp_N | home | q to finish): ")
            except EOFError:
                break
            if s.strip().lower() in ('q', 'quit', 'exit', ''):
                break
            try:
                goals = [goal_from_text(mission_map, s)]
            except Exception as e:
                print(f"{LOG_TAG} Goal not valid: {e}")
                continue
        if nav.end_reason != 'battery':
            if not goals:
                break
            g = goals.pop(0)
            if g == HOME:
                ok = nav.go_home("asked")
            else:
                ok = nav.go(g[0], g[1], graphnav_fallback=not args.no_fallback)
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
    if args.click and not window_available():
        return False
    if args.preview:
        if not (args.to or args.cell or args.waypoint or args.click):
            print("With --preview a goal is needed: --to, --cell, --waypoint or --click.")
            return False
        return preview(args)
    if not (args.to or args.cell or args.waypoint or args.home or args.interactive or args.click):
        print("A goal is needed (--to, --cell, --waypoint, --home) or --interactive or --click.")
        return False
    return on_robot(args)


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
