#!/usr/bin/env python3
"""
Compare two recordings of the end-to-end test logs (E2E_LOG_DIR), ignoring what changes
from one run to the next without changing behaviour: timings, temporary folders, clock
times. Used to check that a code restructure changes nothing.

    E2E_LOG_DIR=/tmp/before python3 tests/test_e2e_loop.py
    ... change ...
    E2E_LOG_DIR=/tmp/after  python3 tests/test_e2e_loop.py
    python3 tools/compare_logs.py /tmp/before /tmp/after

Exit 0 = identical, 1 = differences (the first ones of each scenario are printed).
"""
import difflib
import os
import re
import sys

_RULES = [
    (re.compile(r'\d+(\.\d+)?\s*s\b'), '<T>s'),                 # durations
    (re.compile(r'\d+(\.\d+)?\s*ms\b'), '<T>ms'),
    (re.compile(r'/tmp/[^\s\'",)]+'), '<TMP>'),                  # temporary folders
    (re.compile(r'\b\d{2}:\d{2}:\d{2}(\.\d+)?\b'), '<TIME>'),     # clock times
    (re.compile(r'0x[0-9a-f]+'), '<ADDR>'),                      # object addresses
    (re.compile(r'\d{2}-\d{2}-\d{4}_\d{2}-\d{2}-\d{2}'), '<DATA>'),   # folder names
    (re.compile(r'\d+(\.\d+)?\s*min\b'), '<T>min'),               # waypoint ages
    (re.compile(r'\d+ failed readings'), '<N> failed readings'),  # battery reads (timing)
    (re.compile(r'(File "[^"]+", line )\d+'), r'\1<N>'),    # traceback line numbers
]


def normalize(text):
    out = []
    for line in text.splitlines():
        if line.startswith('[TIMING]') or '[TIMING]' in line:
            continue                                            # timing-only lines
        for rx, rep in _RULES:
            line = rx.sub(rep, line)
        if re.search(r'<T>s\s+\(\s*[\d.]+%\)', line):
            continue                                            # timing breakdown
        out.append(line)
    return out


def main(a, b):
    names = sorted(set(os.listdir(a)) | set(os.listdir(b)))
    bad = 0
    for n in names:
        pa, pb = os.path.join(a, n), os.path.join(b, n)
        if not (os.path.exists(pa) and os.path.exists(pb)):
            print(f"[MISSING] {n}: only in {'before' if os.path.exists(pa) else 'after'}")
            bad += 1
            continue
        la, lb = normalize(open(pa).read()), normalize(open(pb).read())
        if la != lb:
            bad += 1
            diff = list(difflib.unified_diff(la, lb, 'before', 'after', n=1, lineterm=''))
            print(f"[DIFFERENT] {n}: {sum(1 for d in diff if d[:1] in '+-') - 2} lines")
            print("\n".join(diff[:30]))
        else:
            print(f"[SAME] {n} ({len(la)} lines)")
    print("\nIDENTICI" if not bad else f"\n{bad} SCENARIOS DIFFERENT")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
