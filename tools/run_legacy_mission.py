#!/usr/bin/env python3
"""
A mission with an OLD logic, out of the main code (see spot_explorer/mission/legacy/__init__.py).

On the code of today:
    python3 tools/run_legacy_mission.py --side-rules  [arguments of run_mission.py]
        the rules of the sides tried (before 09-11) instead of the cell rules
    python3 tools/run_legacy_mission.py --stop-and-go [arguments of run_mission.py]
        no cruise: the stop-and-go loop with its own attempt start (legacy/stop_and_go_start.py)

On the code of a tag (the switches removed from the code, see legacy/__init__.py):
    python3 tools/run_legacy_mission.py --tag legacy-switches-2026-11-11 \\
        --set attempt_cruise.CRUISE_SWITCH=False [--set ...] [arguments of run_mission.py]
    the tag is checked out (once) as a git worktree in ../legacy_worktrees/<tag>, the switches
    are set on its modules (spot_explorer.mission.<module>.<NAME>, or planning./perception.
    with the full path) and its run_mission main() runs there, with the config.py of the tag:
    grid and density of the day as arguments (--rows --cols --cell ..., or SPOT_MISSION_GRID).

Everything after the options goes to run_mission.py (python3 run_mission.py --help).
"""
import argparse
import ast
import importlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _worktree(tag):
    path = os.path.join(os.path.dirname(ROOT), 'legacy_worktrees', tag)
    if not os.path.isdir(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        subprocess.run(['git', '-C', ROOT, 'worktree', 'add', '--detach', path, tag], check=True)
    return path


def _set(assignment):
    name, value = assignment.split('=', 1)
    mod, attr = name.rsplit('.', 1)
    if not mod.startswith('spot_explorer.'):
        mod = 'spot_explorer.mission.' + mod
    module = importlib.import_module(mod)
    if not hasattr(module, attr):
        sys.exit(f"{mod} has no {attr}")
    setattr(module, attr, ast.literal_eval(value))
    print(f"[LEGACY] {mod}.{attr} = {value}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--side-rules', action='store_true', help="the rules of the sides tried")
    ap.add_argument('--stop-and-go', action='store_true', help="no cruise: the stop-and-go loop")
    ap.add_argument('--tag', help="run the code of this git tag (worktree)")
    ap.add_argument('--set', action='append', default=[], metavar='MODULE.NAME=VALUE',
                    help="set a switch before the mission (with --tag: on the code of the tag)")
    args, rest = ap.parse_known_args()

    root = _worktree(args.tag) if args.tag else ROOT
    sys.path.insert(0, root)
    os.chdir(root)
    for a in args.set:
        _set(a)
    if args.side_rules:
        from spot_explorer.mission import exploration
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        import side_rules
        side_rules.apply()
        print("[LEGACY] Cell choice: the rules of the sides tried (the side_rules variant).")
    if args.stop_and_go:
        from spot_explorer.mission import attempt_cruise
        attempt_cruise.CONTINUOUS_MOTION = False
        print("[LEGACY] No cruise: the stop-and-go loop (legacy/stop_and_go_start.py).")
    from spot_explorer.mission.run import main as mission_main
    sys.argv = [os.path.join(root, 'run_mission.py')] + rest
    return mission_main()


if __name__ == '__main__':
    main()
