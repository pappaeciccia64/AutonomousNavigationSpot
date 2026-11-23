"""
Edge slopes computed edge by edge, instead of all the pairs together with matrices
(edge_evaluation.EdgeEvaluation.precompute_arc_slopes), both at the attempt start and in the
refresh. Same numbers, slower: kept to compare the two computations.
"""
from spot_explorer.planning import edge_evaluation as ee
from hook import replace

DESCRIPTION = "edge slopes computed edge by edge (same result as the batch, slower)"


def _no_batch(self, pairs, skip=None):
    return [None] * len(pairs)


def apply():
    return replace(ee.EdgeEvaluation, 'precompute_arc_slopes', _no_batch)
