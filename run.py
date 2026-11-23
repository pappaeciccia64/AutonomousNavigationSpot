#!/usr/bin/env python3
"""
A mission with some variants on: the main code with some of its functions replaced at start-up.

    python3 varianti/run.py --list
    python3 varianti/run.py --with a_star,ground_from_grid -- --rows 8 --cols 8

Everything after `--` goes to run_mission.py. The variants on are written in the mission log
([CONFIG] SWITCHES, key 'variants') and in mission_summary.json.
"""
import argparse
import importlib
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT]


def available():
    """Name -> description of every variant module in this folder."""
    out = {}
    for f in sorted(os.listdir(HERE)):
        if f.endswith('.py') and f not in ('run.py', 'check.py', 'hook.py'):
            mod = importlib.import_module(f[:-3])
            if hasattr(mod, 'apply'):
                out[f[:-3]] = getattr(mod, 'DESCRIPTION', '')
    return out


def apply(names):
    """Turn the variants on; returns one function that turns them all off."""
    from hook import chain
    restores = [importlib.import_module(n).apply() for n in names]
    from spot_explorer.mission import summary
    original = summary.mission_switches
    summary.mission_switches = lambda: dict(original(), variants=','.join(names))
    restores.append(lambda: setattr(summary, 'mission_switches', original))
    return chain(*restores)


def main():
    argv = sys.argv[1:]
    mission_args = argv[argv.index('--') + 1:] if '--' in argv else []
    own = argv[:argv.index('--')] if '--' in argv else argv
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--with', dest='names', default='', help='comma-separated variant names')
    ap.add_argument('--list', action='store_true', help='list the variants and exit')
    a = ap.parse_args(own)
    variants = available()
    if a.list or not a.names:
        for n, d in variants.items():
            print(f"  {n:24s} {d}")
        return 0
    names = [n.strip() for n in a.names.split(',') if n.strip()]
    unknown = [n for n in names if n not in variants]
    if unknown:
        sys.exit(f"Unknown variants: {', '.join(unknown)} (see --list)")
    apply(names)
    print(f"[VARIANTS] On: {', '.join(names)}")
    os.chdir(ROOT)
    sys.argv = [os.path.join(ROOT, 'run_mission.py')] + mission_args
    runpy.run_path(sys.argv[0], run_name='__main__')


if __name__ == '__main__':
    sys.exit(main())
