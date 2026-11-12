"""
OLD LOGIC, out of the main code but kept runnable (user, 11-11-2026: "le cose vecchie non le
cancellerei ... script diversi, in modo da tenerli e poterli usare, ma togliendo dal codice
principale tutte le parti che non usiamo attivamente").

Here, runnable on the code of today:
  side_rules.py         the rules of the sides tried (before 09-11): SideRulesMission;
                        tools/run_legacy_mission.py --side-rules
  stop_and_go_start.py  the attempt start of the stop-and-go mode (scan standing, graph,
                        goal, path in the main thread): used by CellAttempt when the cruise is
                        off (attempt_cruise.CONTINUOUS_MOTION = False, tools/run_legacy_mission.py
                        --stop-and-go; test_e2e_loop, most of test_mission)

From the tag legacy-switches-2026-11-11 (the code of 11-11 evening, with the switches still in
it), the switches removed on 12-11 -- woven into the cruise, a copy here would only drift:
  attempt_cruise.CHAIN_CELLS = False      stop at every cell goal, no hand-off while walking
  attempt_cruise.CHAIN_ON_SEEN = False    a cell is done only at its goal, not when 90% seen
  attempt_cruise.CRUISE_SWITCH = False    the next cell prepared by the next attempt after a
                                          hand-off, not in the planning thread (step C); goal
                                          isolated: new goal chosen by the main thread
                                          (_cruise_repick)
  attempt_cruise.THREAD_FIRST_TURN = False  a stop-and-go turn before every cruise
  planning_cycle.WHOLE_EDGES = False      the stretch walked as far as the frontier confirms
                                          (safe_frontier.allowed_advance), edges in part
To run one of them:  tools/run_legacy_mission.py --tag legacy-switches-2026-11-11 --set
attempt_cruise.CRUISE_SWITCH=False [arguments of run_mission.py] (a git worktree of the tag in
../legacy_worktrees/, the switch set before main()).
"""
