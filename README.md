# varianti (local only, not on GitHub)

Alternatives and one-off tools kept out of the main code (see the main repository's
CHANGELOG, 2026-11-20). This folder is ignored by the main repository and has its own git.

- `tools/`: analyses and comparisons already used for a decision, and tools that compare
  switches no longer in the main code. Run them from the main repository folder, e.g.
  `python3 varianti/tools/slope_check.py`. The links here (spot_explorer, tests, MissionLogs,
  MissionMap, graph) let them find the code and the data as they did in tools/.
- Tools that compared switches since removed (bench_edges, bench_refresh, compare_cell_rules,
  plane_thresholds) only work on the code of the tag `switches-before-cleanup-2026-11-20`.

## Launch and check

    python3 varianti/run.py --list
    python3 varianti/run.py --with a_star,ground_from_grid -- <arguments of run_mission.py>
    flock ~/spotSDK/test_results/.lock python3 varianti/check.py     # all variants: hooks + tests

Each variant replaces one or more functions ("hooks") of the main code at start-up; check.py says
when the main code no longer has them. Variant tests are in variant_tests/ (tests/ is a link to
the main repository's tests).

Known (24-11): with event_driven_planning, test_cruise fails "the attempt ends cleanly" in the
0.8 m box scenario (3 runs out of 3), which passes with the main code. The scenario was judged
impossible by the user (a box appearing 0.8 m ahead); kept as a note for when this variant is tried.
