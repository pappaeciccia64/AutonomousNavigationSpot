"""
The attempt start of the stop-and-go mode (stop_and_go_start.py: scan standing, graph, goal,
path in the main thread), used by CellAttempt when the cruise is off
(attempt_cruise.CONTINUOUS_MOTION = False; test_e2e_loop, part of test_mission).

The old cell rules (the sides tried, before 09-11) and the logic of the switches removed on
12-11 are not in this repository any more: the first is a local variant, the second runs from
the tag legacy-switches-2026-11-11.
"""
