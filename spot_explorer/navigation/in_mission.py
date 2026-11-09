"""
Navigation phase inside the mission (run_mission.py --then-navigate).

When the exploration is over (and the final scan and summary are done), instead of going
back to wp_0 at once the robot waits for commands on the map it has just built: goals,
places, routes, the click window. With 'end' the mission closes as always (map closure,
return to wp_0, sitting down, GraphNav map downloaded), and everything seen during the
phase is in the maps the mission saves.

Same session as the exploration, so none of the work of run_navigation.py on an old
mission is needed: the maps, the robot and GraphNav are in the same VISION frame (no graph
upload, no localization, identity transform). The rest is the map navigation code
(navigator.MapNavigator, run.run_route, goal_picker.ClickGoalPicker).

While it waits for a command the robot sits down after IDLE_SIT_S seconds, to save the
battery, and stands up again at the next command.
"""
import threading
import time
from types import SimpleNamespace

from spot_explorer.navigation.mission_map import LOG_TAG, MissionMap
from spot_explorer.navigation.navigator import HOME, MapNavigator

# Without a command for this long the robot sits down (it stands up at the next command).
IDLE_SIT_S = 120.0
# How the commands are read (the tests put their own script here).
ASK = input

HELP = f"""{LOG_TAG} NAVIGATION PHASE: the robot waits for commands on the map it has just explored.
  X,Y [@ H]       go to X ahead, Y to the left of the mission start (m), optional heading H (degrees)
  c R,C           centre of cell (R, C)
  NAME [@ H]      a named place          places        list places and routes
  save NAME       save where the robot stands as a place
  route R [xN]    a saved route, or places separated by commas ('home' allowed), N loops
  map             choose the goal with a click on the map (needs a screen)
  home            go to wp_0 and wait there
  end             end the phase: the mission closes as usual (return to wp_0, saves)"""


class IdleSitter:
    """While waiting for a command: after idle_s the robot sits down (a thread sends the sit
    command), and on leaving the block it stands up again if it had sat."""

    def __init__(self, command_client, idle_s=None):
        self.cc = command_client
        self.idle_s = IDLE_SIT_S if idle_s is None else idle_s
        self.sat = False
        self._stop = threading.Event()

    def _run(self):
        if self._stop.wait(self.idle_s):
            return
        try:
            from bosdyn.client.robot_command import RobotCommandBuilder
            self.cc.robot_command(RobotCommandBuilder.synchro_sit_command(), end_time_secs=time.time() + 20)
            self.sat = True
            print(f"\n{LOG_TAG} No command for {self.idle_s:.0f} s: the robot sits down to save the battery "
                  f"(it stands up at the next command).")
        except Exception as e:
            print(f"\n{LOG_TAG} Could not sit down while waiting: {e}")

    def __enter__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self.sat:
            from bosdyn.client import robot_command
            robot_command.blocking_stand(self.cc)
            print(f"{LOG_TAG} Standing up again.")
        return False


def _route_args(text):
    """'route R [xN]' -> (spec, loops)."""
    parts = text.split()
    loops = 1
    if len(parts) > 1 and parts[-1].lower().startswith('x') and parts[-1][1:].isdigit():
        loops = int(parts[-1][1:])
        parts = parts[:-1]
    return " ".join(parts), loops


def navigation_phase(robot, robot_state_client, command_client, recordingInterface, local_grid,
                     global_grid, env, prm, tracker, mission_folder, log_folder, battery=None):
    """
    The phase, between the end of the exploration and the mission closure. Returns a summary
    dict: goals asked, goals reached, why it ended ('end', 'battery', 'fallen', 'eof').
    Exceptions and Ctrl+C go up to the mission, which does its emergency closure.
    """
    from spot_explorer.navigation import run as nrun
    from spot_explorer.navigation.goal_picker import ClickGoalPicker, window_available

    t0 = time.perf_counter()
    mm = MissionMap.from_live(env, global_grid.global_occupancy_map, global_grid.global_terrain_map,
                              recordingInterface.waypoint_poses, getattr(prm, 'mission_graph', None),
                              mission_folder)
    nav = MapNavigator.in_mission(mm, robot, robot_state_client, command_client, recordingInterface,
                                  tracker, local_grid, log_folder, mission_folder, battery=battery)
    print(f"{LOG_TAG} Map of the exploration ready for navigation in {time.perf_counter() - t0:.1f} s.")
    print(HELP)
    nav._write_state()
    out = dict(asked=0, reached=0, ended='end')
    args = SimpleNamespace(no_fallback=False, loops=1, dwell=0.0, click=False, interactive=True,
                           return_at_end=False)
    while True:
        if nav.battery_low("navigation phase"):
            print(f"{LOG_TAG} Battery low: end of the navigation phase, the mission closes.")
            out['ended'] = 'battery'
            break
        nav.phase = 'waiting for a command'
        nav._write_state()
        try:
            with IdleSitter(command_client):
                text = ASK(f"\n{LOG_TAG} Command (help for the list): ").strip()
                picked = None
                if text.lower() == 'map':
                    if not window_available("map"):
                        continue
                    _new, here = nav.robot_pose()
                    picked = ClickGoalPicker(mm, here, title=f"Mission {mm.name}: where do I go?",
                                             from_label='robot now').choose()
        except EOFError:
            out['ended'] = 'eof'
            break
        low = text.lower()
        if low in ('end', 'fine', 'quit', 'q'):
            break
        if low in ('', 'help', '?'):
            if low:
                print(HELP)
            continue
        try:
            if low == 'places':
                print(f"{LOG_TAG} {mm.places.describe()}")
                continue
            if low.startswith('save '):
                nav.save_place_here(text[5:].strip())
                continue
            if low == 'map':
                if picked is None:
                    continue
                goal = picked
            elif low.startswith('route '):
                spec, args.loops = _route_args(text[6:])
                nrun.route_goals(mm, spec)            # unknown places: said now
                goal = ('route', spec)
            else:
                goal = nrun.goal_from_text(mm, text)
        except (ValueError, KeyError) as e:
            print(f"{LOG_TAG} Not valid: {e}")
            continue
        out['asked'] += 1
        if goal == HOME:
            ok = nav.go_home("asked in the navigation phase")
        elif goal[0] == 'route':
            ok = nrun.run_route(nav, mm, goal[1], args)
        else:
            ok = nav.go(goal[0], goal[1], heading_deg=goal[2] if len(goal) > 2 else None)
        out['reached'] += int(bool(ok))
        if nav.end_reason in ('battery', 'fallen', 'lost'):
            print(f"{LOG_TAG} End of the navigation phase ({nav.end_reason}): the mission closes.")
            out['ended'] = nav.end_reason
            break
        if nav.end_reason is None and not ok:
            print(f"{LOG_TAG} Not reached: waiting for the next command.")
        nav.end_reason = None
    nav.phase = 'navigation phase over'
    nav._write_state()
    nav.save_maps(mission_folder, quiet_=True)
    print(f"{LOG_TAG} Navigation phase over ({out['ended']}): {out['reached']} of {out['asked']} goals "
          f"reached; maps updated with {nav.n_scans_in_map} scans. Now the usual mission closure.")
    return out
