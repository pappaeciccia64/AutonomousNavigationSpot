# varianti (local only, not on GitHub)

Alternatives and one-off tools kept out of the main code (see the main repository's
CHANGELOG, 2026-11-20). This folder is ignored by the main repository and has its own git.

- `tools/`: analyses and comparisons already used for a decision, and tools that compare
  switches no longer in the main code. Run them from the main repository folder, e.g.
  `python3 varianti/tools/slope_check.py`. The links here (spot_explorer, tests, MissionLogs,
  MissionMap, graph) let them find the code and the data as they did in tools/.
- Tools that compared switches since removed (bench_edges, bench_refresh, compare_cell_rules,
  plane_thresholds) only work on the code of the tag `switches-before-cleanup-2026-11-20`.
