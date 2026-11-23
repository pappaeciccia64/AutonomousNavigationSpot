#!/usr/bin/env python3
"""
Do the variants still fit the main code? Every variant is turned on and off (its hooks must
still exist), then its test in tests/ runs, if it has one.

    python3 varianti/check.py              # all
    python3 varianti/check.py a_star       # some
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]
import run   # noqa: E402


def main():
    names = sys.argv[1:] or list(run.available())
    bad = []
    for n in names:
        try:
            run.apply([n])()
            hooked = True
        except Exception as e:
            print(f"  [FAIL] {n}: {e}")
            bad.append(n)
            hooked = False
        test = os.path.join(HERE, 'tests', f'test_{n}.py')
        if hooked and os.path.exists(test):
            ok = subprocess.run([sys.executable, test], cwd=os.path.dirname(HERE),
                                capture_output=True, text=True)
            last = (ok.stdout.strip().splitlines() or [''])[-1]
            print(f"  [{'OK ' if ok.returncode == 0 else 'FAIL'}] {n}: {last}")
            if ok.returncode:
                bad.append(n)
        elif hooked:
            print(f"  [OK ] {n}: hooks in place (no test)")
    print("ALL OK" if not bad else f"FAILED: {', '.join(bad)}")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
