"""
Edge refresh with the neighbour lists rebuilt once per node at the end, instead of at every edge
(edge_evaluation.EdgeEvaluation._refresh_pairs_loop, _set_edge, _edge_weight).

In the main code every refreshed edge rewrites both neighbour lists, so a node with k refreshed
edges has its list rebuilt k times. Here the edges refreshed are collected and the lists rebuilt
once: the same lists in the same order (each refreshed edge removed, then appended at the end if
valid, in refresh order). Reads during the refresh see the edges refreshed so far.
"""
from spot_explorer.planning import edge_evaluation as ee
from hook import chain, replace

DESCRIPTION = "edge refresh: neighbour lists rebuilt once per node (same lists, faster)"


def apply():
    cls = ee.EdgeEvaluation
    loop, set_edge, edge_weight = cls._refresh_pairs_loop, cls._set_edge, cls._edge_weight

    def _refresh_pairs_loop(self, *a, **k):
        self._batch_adj = {}
        try:
            return loop(self, *a, **k)
        finally:
            batch, self._batch_adj = self._batch_adj, None
            if batch:
                _apply_batch(self, batch)

    def _set_edge(self, idx1, idx2, weight):
        batch = getattr(self, '_batch_adj', None)
        if batch is None:
            return set_edge(self, idx1, idx2, weight)
        batch.pop((idx2, idx1), None)
        batch[(idx1, idx2)] = weight

    def _edge_weight(self, idx1, idx2):
        batch = getattr(self, '_batch_adj', None)
        if batch:
            if (idx1, idx2) in batch:
                return batch[(idx1, idx2)]
            if (idx2, idx1) in batch:
                return batch[(idx2, idx1)]
        return edge_weight(self, idx1, idx2)

    return chain(replace(cls, '_refresh_pairs_loop', _refresh_pairs_loop),
                 replace(cls, '_set_edge', _set_edge),
                 replace(cls, '_edge_weight', _edge_weight))


def _apply_batch(self, batch):
    done, add = {}, {}
    for (a, b), w in batch.items():
        done.setdefault(a, set()).add(b)
        done.setdefault(b, set()).add(a)
        if w is not None:
            add.setdefault(a, []).append((b, w))
            add.setdefault(b, []).append((a, w))
    for node, gone in done.items():
        self.edges[node] = [(n, w) for n, w in self.edges.get(node, []) if n not in gone] + add.get(node, [])
