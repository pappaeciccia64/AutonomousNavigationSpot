#!/usr/bin/env python3
"""
Timing of the cruise from the logs of real missions: to compare the timing variants
(event_driven_planning, wake_on_plan, cruise_state_cache; before 24-11 the switches
planning_cycle.PLAN_EVENT_DRIVEN, attempt_cruise.CRUISE_WAKE_ON_PLAN and CRUISE_STATE_CACHE)
mission against mission.

    python3 varianti/tools/cruise_timing.py 09-11-2026_14-53-46 09-11-2026_15-42-07 ...
    python3 varianti/tools/cruise_timing.py 09-11-2026_*      # a shell pattern on MissionLogs/

Per mission: the variants (or the old switches) it ran with ([CONFIG] SWITCHES), plans per second while cruising,
cycle and phases (median, p90), interval between Spot grids, grids read twice / not
processed, how long a plan waited before the main thread read it ("read after", logged from
19-11), the turn of the main thread and the cost of its requests ([CRUISE] Main thread, from
19-11), the robot state cache line, and how often the cruise stopped at the end of the
confirmed stretch (the fresher the plan, the fewer).
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SWITCH_KEYS = ('variants', 'planning_cycle.PLAN_EVENT_DRIVEN', 'attempt_cruise.CRUISE_WAKE_ON_PLAN',
               'attempt_cruise.CRUISE_STATE_CACHE', 'planning_cycle.PLAN_PERIOD_S')
PLAN_RE = re.compile(r'^(\d\d):(\d\d):(\d\d\.\d+) \[CRUISE\] Plan (\d+).*?\| cycle ([\d.]+) s(?: \(([^)]*)\))?')


def _secs(h, m, s):
    return int(h) * 3600 + int(m) * 60 + float(s)


def _p(a, q):
    return float(np.percentile(a, q)) if len(a) else float('nan')


def report(log):
    switches, cycles, phases, grid, read, times = {}, [], {}, [], [], []
    not_proc, read_twice, n_grids, cache_lines, ends = 0, 0, 0, [], []
    plans_read = cycles_done = 0
    main_turn, main_poll, main_send = [], [], []
    for line in open(log, errors='ignore'):
        if '[CONFIG] SWITCHES' in line:
            for k in SWITCH_KEYS:
                m = re.search(re.escape(k) + r'=(\S+)', line)
                if m:
                    switches[k.split('.')[-1]] = m.group(1)
        m = PLAN_RE.match(line)
        if m:
            times.append(_secs(*m.groups()[:3]))
            cycles.append(float(m.group(5)))
            if m.group(6):
                parts = m.group(6).split()
                for k, v in zip(parts[::2], parts[1::2]):
                    try:
                        phases.setdefault(k, []).append(float(v))
                    except ValueError:
                        pass
            g = re.search(r'Spot grid \+([\d.]+) s', line)
            if g:
                grid.append(float(g.group(1)))
            r = re.search(r'read after (\d+) ms', line)
            if r:
                read.append(float(r.group(1)) / 1000)
        c = re.search(r'Cycle times .*?(\d+) read twice out of (\d+)(?: \((\d+) not processed\))?', line)
        if c:
            read_twice += int(c.group(1)); n_grids += int(c.group(2)); not_proc += int(c.group(3) or 0)
        mt = re.search(r'\[CRUISE\] Main thread: turn every p50/p90 (\d+)/\d+ ms, feedback \+ pose (\d+)/\d+ ms, '
                       r'path sent (\S+?)/', line)
        if mt:
            main_turn.append(float(mt.group(1))); main_poll.append(float(mt.group(2)))
            if mt.group(3) != '-':
                main_send.append(float(mt.group(3)))
        if '[CRUISE] robot state:' in line:
            cache_lines.append(line.split('[CRUISE] ', 1)[1].strip())
        e = re.search(r'\[CRUISE\] End.*?: (.*)$', line)
        if e:
            ends.append(e.group(1))
            pc = re.search(r'(\d+) plans \((\d+) cycles', line)
            if pc:
                plans_read += int(pc.group(1)); cycles_done += int(pc.group(2))
    if not cycles:
        return None
    t = np.array(times)
    gaps = np.diff(t)
    gaps = gaps[(gaps > 0) & (gaps < 2.0)]              # within a cruise
    out = [f"  switches: " + (", ".join(f"{k}={v}" for k, v in switches.items()) or "not in the log (before 19-11: all off, period 0.4)")]
    out.append(f"  plans {len(cycles)}, {1 / np.median(gaps):.1f} per second while cruising (median interval {np.median(gaps):.2f} s)"
               if len(gaps) else f"  plans {len(cycles)}")
    out.append(f"  cycle p50/p90 {_p(cycles, 50):.2f}/{_p(cycles, 90):.2f} s; phases p50 "
               + " ".join(f"{k} {_p(v, 50):.2f}" for k, v in phases.items()))
    if grid:
        out.append(f"  Spot grid every p50 {_p(grid, 50):.2f} s (min {min(grid):.2f})"
                   + (f"; {read_twice} of {n_grids} grids read twice, {not_proc} cycles not processed" if n_grids else ""))
    out.append(f"  plan read by the main thread after p50/p90 {1000 * _p(read, 50):.0f}/{1000 * _p(read, 90):.0f} ms"
               if read else "  plan read delay: not logged (before 19-11)")
    if main_turn:
        out.append(f"  main thread (median over the cruises): turn every {np.median(main_turn):.0f} ms, "
                   f"feedback + pose {np.median(main_poll):.0f} ms"
                   + (f", path sent {np.median(main_send):.0f} ms" if main_send else ""))
    for cl in cache_lines[-1:]:
        out.append(f"  {cl} (last cruise)")
    if cycles_done:
        out.append(f"  plans read by the main thread {plans_read} of {cycles_done} published "
                   f"({100 * plans_read / cycles_done:.0f}%: the others were replaced before it looked)")
    short = sum(1 for e in ends if 'confirmed stretch ends' in e)
    if ends:
        out.append(f"  cruises {len(ends)}, {short} ended at the end of the confirmed stretch")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('missions', nargs='+', help='e.g. 09-11-2026_14-53-46, or a pattern 09-11-2026_*')
    a = ap.parse_args()
    for pat in a.missions:
        for d in sorted(glob.glob(os.path.join(ROOT, 'MissionLogs', f'Mission_{pat}'))):
            log = os.path.join(d, 'mission_log.txt')
            if not os.path.exists(log):
                continue
            r = report(log)
            print(f"== {os.path.basename(d)[8:]}")
            print(r if r else "  no cruise")


if __name__ == '__main__':
    sys.exit(main())
