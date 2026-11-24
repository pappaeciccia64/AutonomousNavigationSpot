"""
Event-driven planning: a cycle starts as soon as the previous one ends (at most every
MIN_PERIOD_S, planning_cycle.plan_period) and a Spot grid already processed is not processed
again: the cycle publishes nothing (planning_cycle.CruisePlanner._skip_grid), unless the last
grid processed is older than MAX_SKIP_S, so plans keep coming if Spot stops renewing its grids.
tools/grid_rate_check.py measures how often Spot renews them.
"""
from spot_explorer.mission import planning_cycle
from hook import chain, replace

DESCRIPTION = "planning cycle as soon as possible, skipping grids already processed"
MIN_PERIOD_S = 0.03
MAX_SKIP_S = 0.6


def _skip_grid(self, grid_dt, now):
    return (grid_dt is not None and grid_dt <= 1e-6 and self._processed_t is not None
            and now - self._processed_t < MAX_SKIP_S)


def apply():
    return chain(replace(planning_cycle, 'plan_period', lambda: MIN_PERIOD_S),
                 replace(planning_cycle.CruisePlanner, '_skip_grid', _skip_grid))
