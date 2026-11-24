#!/usr/bin/env python3
"""
The side_rules variant (the old cell rules) on the fake missions of tests/test_mission.py,
worlds A and B: the mission ends without errors, every reachable cell is visited and the
closed cell (2,2) ends blocked, as with these rules before 09-11.

    python3 varianti/variant_tests/test_side_rules.py
"""
import contextlib
import io
import os
import shutil
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))    # varianti/
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT, os.path.join(ROOT, 'tests')]
os.chdir(ROOT)

with contextlib.redirect_stdout(io.StringIO()):
    import test_mission as M      # noqa: E402
import side_rules                 # noqa: E402
from spot_explorer.mission import exploration    # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def main():
    off = side_rules.apply()
    try:
        for name in ('A', 'B'):
            print(f"== world {name}, old cell rules ==")
            r = M.run(M.WORLDS[name], f"side_rules_{name}")
            log = r['log']
            grid = log.split("--- Cell map ---")[-1].splitlines()[1:M.ROWS + 1]
            cells = [line.split()[1:] for line in grid]
            visited = sum(v == '1' for row in cells for v in row)
            check('Traceback' not in log and 'SideRulesMission' in log, "no traceback, the side rules ran")
            check(visited == 8, f"all 8 reachable cells visited ({visited})")
            check(cells and cells[2][2] == '-1', "the closed cell (2,2) is blocked")
            shutil.rmtree(r['root'], ignore_errors=True)
    finally:
        off()
    check(exploration.Mission.__name__ == 'Mission', "variant off: exploration.Mission again")
    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
