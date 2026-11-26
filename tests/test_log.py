#!/usr/bin/env python3
"""
The mission log (records._TeeStream, records.LOG_OFF) without the robot.

    python3 tests/test_log.py

  1. everything printed goes on screen and in the file, each file line with its time;
  2. a category in LOG_OFF is dropped from both, with the lines without a tag that follow it
     (table rows), and the next tagged line is written again;
  3. a line written in pieces (print(..., end=''), flush in the middle) comes out whole, with
     one time stamp, and is dropped whole if its category is off.
"""
import io
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from spot_explorer import records                                    # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def run(off, writes):
    screen = io.StringIO()
    path = os.path.join(tempfile.mkdtemp(prefix='test_log_'), 'mission_log.txt')
    records.LOG_OFF = set(off)
    tee = records._TeeStream(screen, path)
    for w in writes:
        if w is None:
            tee.flush()
        else:
            tee.write(w)
    tee.flush()
    tee.close_file()
    records.LOG_OFF = set()
    return screen.getvalue(), open(path).read()


def main():
    print("== everything on ==")
    scr, fil = run((), ["[CRUISE] Plan 1\n", "[GAIT] plain\n", "  row of a table\n"])
    check(scr == "[CRUISE] Plan 1\n[GAIT] plain\n  row of a table\n", "screen: every line")
    stamped = fil.splitlines()
    check(len(stamped) == 3 and all(re.match(r'\d\d:\d\d:\d\d\.\d{3} ', l) for l in stamped),
          "file: every line with its time")

    print("== [GAIT] off ==")
    scr, fil = run({'GAIT'}, ["[CRUISE] Plan 1\n", "[GAIT] plain\n", "  row of the gait table\n",
                              "\n", "[CRUISE] Plan 2\n", "line without a tag\n"])
    check(scr == "[CRUISE] Plan 1\n[CRUISE] Plan 2\nline without a tag\n",
          f"the [GAIT] line and the rows after it dropped, the rest kept ({scr!r})")
    check('GAIT' not in fil and 'row of the gait' not in fil and fil.count('Plan') == 2,
          "the same in the file")

    print("== a line in pieces ==")
    scr, fil = run((), ["[MAP DOWNLOAD] Downloading", None, "... done\n"])
    check(scr == "[MAP DOWNLOAD] Downloading... done\n", f"screen: one line ({scr!r})")
    check(len(fil.splitlines()) == 1 and fil.count(':') >= 2 and fil.rstrip().endswith('done'),
          f"file: one line, one time stamp ({fil!r})")
    scr, fil = run({'MAP DOWNLOAD'}, ["[MAP DOWNLOAD] Downloading", None, "... done\n", "[CRUISE] x\n"])
    check(scr == "[CRUISE] x\n" and 'Downloading' not in fil, f"dropped whole when off ({scr!r})")

    print(f"\n{'ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
