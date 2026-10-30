"""
mission_graph.py -- the mission graph, built from the MISSION SIZE (rows, columns, cell
side: the same three numbers as SPOT_MISSION_GRID and cells.EnvironmentMap). Nodes are
generated in two ways (P.MISSION_GRAPH_SAMPLING, in config.py like all the P.* mentioned
here):
  - 'lattice' (default): a rectangular LATTICE of equally spaced nodes, in closed form.
    Two flags: the SPACING between nodes and the connection RINGS.
  - 'random': scattered nodes with a chosen distribution, to compare the two methods.

The nodes, the reasons for the lattice and its limits: graph_nodes.py. The graph during
the mission (PRM construction, edge update, allowed cells): mission/mission_prm.py.
Comparing graphs from the command line: tools/bench_graph.py.

HOW TO USE IT
      mg = mission_graph.build_mission_graph(env=env)   # ~0.2 s: nodes + adjacency
      mg.into_prm(prm)                                  # the nodes, ids 0..N-1
      gb_sampler = mg.as_sampler()                      # lattice points per cell
      ...
      mg.add_edges_to_prm(prm, global_map, geometry.PRM_EDGE_SAFETY_MARGIN_M)
  add_edges_to_prm evaluates every pair with PRM._evaluate_and_add_edge, the same function
  used for all edges: vetoes and weights come from there. On the robot only the vetoes and
  weights are computed, which by construction cannot be precomputed: occupancy comes from
  the global map that fills up along the way, the edge slope from the terrain seen at that
  moment.
  save_npz/load_npz serve to have an IDENTICAL lattice across several missions, not for
  speed: with the default mission (3185 nodes) building from scratch costs ~180 ms,
  reloading ~70 ms (python3 tools/bench_graph.py --timings).
"""

import math

import numpy as np

from spot_explorer import config as P          # mission size, node method, edges
from spot_explorer.perception import geometry
from spot_explorer.planning import edge_evaluation
from spot_explorer.planning.graph_nodes import (GRAPH_SAMPLINGS, RANDOM_DISTRIBUTIONS, align_spacing,
                                                connected_components, covering_radius, draw_seed,
                                                lattice_pairs, lattice_points, measured_covering,
                                                mission_extent, node_density, radius_pairs,
                                                random_points, suggested_spacing)


# The mission (rows, columns, side, spacing, node method, edges) is chosen in
# config.py, so the mission on the robot and the command-line tests (tools/bench_graph.py)
# start from the same numbers. These DEFAULT_* are only for bench_graph.compare() and
# as a fallback if a value in config is None (2 x 4 cells of 5 m, spacing 0.25 m).
DEFAULT_MISSION_ROWS = 2
DEFAULT_MISSION_COLS = 4
DEFAULT_CELL_SIZE_M = 5.0
DEFAULT_LATTICE_SPACING_M = 0.25


def mission_settings(rows=None, cols=None, cell_size_m=None, spacing_m=None):
    """
    (rows, cols, cell_size_m, spacing_m) of the mission. For each one, in order, the winner
    is: the value passed here, the one in config.py, the DEFAULT_*.
    """
    def pick(value, chosen, default):
        if value is not None:
            return value
        return chosen if chosen is not None else default
    return (int(pick(rows, P.MISSION_GRID_ROWS, DEFAULT_MISSION_ROWS)),
            int(pick(cols, P.MISSION_GRID_COLS, DEFAULT_MISSION_COLS)),
            float(pick(cell_size_m, P.MISSION_CELL_SIZE_M, DEFAULT_CELL_SIZE_M)),
            float(pick(spacing_m, P.MISSION_LATTICE_SPACING_M, DEFAULT_LATTICE_SPACING_M)))


# Default value of spacing_m in build_mission_graph: "take it from mission_settings()".
# It is not None because None already means suggested_spacing().
_FROM_SETTINGS = "settings"


# Slopes of the pairs computed in batch with matrices (see PRM.precompute_arc_slopes).
# False = edge by edge; the result is identical.
BATCH_SLOPES = True


def _special_pairs(prm):
    """Edges travelled by the robot and edges blocked in the attempt, as (min, max) pairs."""
    out = {(min(a, b), max(a, b)) for a, b in getattr(prm, 'traversed_edges', ())}
    out |= {(min(a, b), max(a, b)) for a, b in getattr(prm, 'tracker_blocked_edges', ())}
    return out


# ========================================================================================
# THE MISSION GRAPH
# ========================================================================================

class MissionGraph:
    """
    The ready graph: nodes, geometric adjacency with the lengths, and the hooks to load it
    into the PRM. It contains NO weights or vetoes: those depend on the terrain and the PRM
    computes them on the local data (see add_edges_to_prm).
    """

    def __init__(self, spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                 ij, xy_local, pairs, lengths, node_cell, cell_center_ids, origin_node_id,
                 min_edge_m, reach_m, rings, n_short_pairs, sampling='lattice',
                 distribution=None, density_per_m2=None, seed=None, covering=None,
                 sampling_note=""):
        self.spacing_m = float(spacing_m)
        self.rows, self.cols = int(rows), int(cols)
        self.cell_size_m = float(cell_size_m)
        self.start_cell = tuple(int(v) for v in start_cell)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.origin_yaw = float(origin_yaw)
        self.min_edge_m, self.reach_m = float(min_edge_m), float(reach_m)
        self.rings = rings

        self.ij = ij                    # (N, 2) integer lattice indices
        self.xy_local = xy_local        # (N, 2) grid frame
        self.pairs = pairs              # (E, 2) node indices, sorted
        self.lengths = lengths          # (E,)   metres
        self.node_cell = node_cell      # (N, 2) cell (row, column) of every node
        self.cell_center_ids = cell_center_ids    # {(row, col): id of the node at the centre}
        self.origin_node_id = origin_node_id      # node under the robot at start-up
        self.n_short_pairs = int(n_short_pairs)
        # How the nodes were generated (see RANDOM NODES). For random nodes ij is
        # -1 and spacing_m is the mean spacing 1/sqrt(density); the coverage is measured.
        self.sampling = str(sampling)
        self.distribution = distribution
        self.density_per_m2 = None if density_per_m2 is None else float(density_per_m2)
        self.seed = None if seed is None else int(seed)
        self.covering = covering                  # (maximum, 95th percentile) or None
        self.sampling_note = sampling_note

        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        self.xy = np.column_stack((
            self.origin_xy[0] + xy_local[:, 0] * c - xy_local[:, 1] * s,
            self.origin_xy[1] + xy_local[:, 0] * s + xy_local[:, 1] * c,
        ))

        deg = np.zeros(len(xy_local), dtype=np.int64)
        for a, b in self.pairs:
            deg[a] += 1
            deg[b] += 1
        self.degrees = deg
        (self.n_components, self.biggest_component,
         self.n_isolated) = connected_components(len(xy_local), self.pairs)

    # -- properties --------------------------------------------------------------------
    @property
    def n_nodes(self):
        return int(len(self.xy))

    @property
    def n_edges(self):
        return int(len(self.pairs))

    @property
    def area_m2(self):
        return self.rows * self.cols * self.cell_size_m ** 2

    @property
    def is_connected(self):
        return self.n_components == 1

    @property
    def is_random(self):
        return self.sampling == 'random'

    @property
    def covering_m(self):
        if self.covering is not None:
            return float(self.covering[0])
        return covering_radius(self.spacing_m)

    def describe(self):
        """The node method in a dict, for mission_summary.json and to replay the mission."""
        out = dict(method=self.sampling, nodes=self.n_nodes, pairs=self.n_edges,
                   edge_min_m=self.min_edge_m, edge_max_m=round(self.reach_m, 6),
                   coverage_m=round(self.covering_m, 4))
        if self.is_random:
            out.update(distribution=self.distribution, node_density_m2=self.density_per_m2,
                       seed=self.seed)
        else:
            out.update(spacing_m=self.spacing_m, rings=self.rings)
        return out

    def node_ids(self):
        """The node ids, 0..N-1: the same PRM.nodes will use."""
        return np.arange(self.n_nodes, dtype=np.int64)

    # -- hooking into the PRM ---------------------------------------------------------------
    def into_prm(self, prm):
        """
        Put the NODES into the PRM (prm.add_node), with ids 0..N-1. No edges: the edges
        come from add_edges_to_prm, which needs the terrain data.
        """
        for k in range(self.n_nodes):
            prm.add_node(int(k), float(self.xy[k, 0]), float(self.xy[k, 1]))
        return self.n_nodes

    def add_edges_to_prm(self, prm, global_map=None, edge_safety_margin=0.0, verbose=True,
                         incremental=False):
        """
        Evaluate the pairs ALREADY COMPUTED and add the allowed ones with
        PRM._evaluate_and_add_edge: vetoes (blocked edges, occupancy in the global map,
        longitudinal and lateral slope) and weights. The pairs are those of the lattice, no
        geometric pairing is redone.

        incremental: re-evaluate ONLY the pairs near the areas with new data since the last
        evaluation (prm.dirty_rects, see edge_evaluation.DIRTY_MARGIN_M); the others keep the edge
        and weight they have. The result is the same as the full build: outside those areas
        no data entering the evaluation has changed. It falls back to the full build when it
        cannot be known what changed (see _dirty_pairs).
        """
        todo_all = [(int(a), int(b), float(d)) for (a, b), d in zip(self.pairs, self.lengths)
                    if int(a) in prm.nodes and int(b) in prm.nodes]
        dirty = self._dirty_pairs(prm, global_map, todo_all) if incremental else None
        prm.robot_node = None          # the exemption near the robot applies only to the current node
        todo = self._clear_pairs(prm, todo_all, dirty)
        prm._sync_global_slope_cache()
        added, n_fast = self._evaluate_pairs(prm, todo, global_map, edge_safety_margin)
        self.last_update = (len(todo), len(todo_all), dirty is not None)
        self._restore_traversed_edges(prm, global_map)

        # The areas just re-evaluated are settled. The current live window stays
        # "to redo": its edges were judged on the live data, and next time,
        # with the robot elsewhere, the full build would judge them on the global map.
        prm.dirty_rects = list(prm.live_rects())
        prm.hazards_seen = len(getattr(global_map, 'hazards', ()) or ())
        prm.special_seen = _special_pairs(prm)
        prm.built = True
        prm.flush_slope_log("build")
        if verbose and dirty is not None:
            print(f"[GRAPH] Local update: re-evaluated {len(todo)} pairs out of {len(todo_all)} "
                  f"(near the areas with new data), {added} allowed; the others stay as they were.")
        elif verbose:
            where = (f"random nodes {self.distribution}" if self.is_random
                     else f"spacing {self.spacing_m:.4f} m")
            print(f"[GRAPH] {added} edges allowed out of {self.n_edges} pairs of the "
                  f"{'graph' if self.is_random else 'lattice'} ({self.n_nodes} nodes, {where}); "
                  f"{n_fast} in an area without data, with the geometric cost only")
        return added

    @staticmethod
    def _clear_pairs(prm, todo_all, dirty):
        """
        Remove from the PRM the edges about to be evaluated and return their pairs: all of
        them (dirty None, full build) or those of the dirty mask. They are then evaluated as in
        a build from scratch (adjacencies just cleared: fresh applies here too).
        """
        if dirty is None:
            for idx in prm.nodes:
                prm.edges[idx] = []
            prm.edge_validity = {}
            return todo_all
        todo = [p for p, m in zip(todo_all, dirty) if m]
        drop = {}
        for a, b, _d in todo:
            drop.setdefault(a, set()).add(b)
            drop.setdefault(b, set()).add(a)
            prm.edge_validity.pop((a, b) if a < b else (b, a), None)
        for n, others in drop.items():
            prm.edges[n] = [(m, w) for m, w in prm.edges.get(n, []) if m not in others]
        return todo

    def _evaluate_pairs(self, prm, todo, global_map, edge_safety_margin):
        """
        Evaluate the pairs with PRM._evaluate_and_add_edge, with the batch computations done
        first. Returns (edges added, of which in an area without data).
        """
        # Occupancy of all pairs in one go and global slope cache
        # filled where there is certainly no data (identical outcomes, see
        # PRM._occupancy_blocked_many and PRM._prefill_global_slope_nodata). The adjacencies have
        # just been cleared and the lattice pairs are distinct: fresh=True.
        use_occ = global_map is not None and edge_safety_margin > 0.0
        occ_flags = (prm._occupancy_blocked_many(todo, global_map, edge_safety_margin)
                     if use_occ else [None] * len(todo))
        prm._prefill_global_slope_nodata(todo)
        fresh = self._pairs_distinct()
        # AREA WITHOUT DATA: edges on which no source has information get
        # the fallback cost directly, identical to that of the full path (see
        # PRM._add_edge_without_data). Needs gamma = 0: with roughness in the cost, no.
        fast_ok = fresh and prm.gamma == 0.0
        live_nd = prm._live_slope_certainly_nodata(todo) if fast_ok else None
        tm = prm.global_terrain_map
        cache = prm._gslope_cache
        # Slopes of all the pairs to evaluate in one go (matrices), live and,
        # where there is no live data, from the global map. Same numbers as the edge-by-edge
        # computation, about 13 times faster (see PRM.precompute_arc_slopes).
        live_profs = [None] * len(todo)
        if BATCH_SLOPES:
            skip = np.array([bool(o) for o in occ_flags], dtype=bool)
            live_profs = prm.precompute_arc_slopes(todo, skip=skip)
        added, n_fast = 0, 0
        for i, ((a, b, d), occ) in enumerate(zip(todo, occ_flags)):
            if fast_ok and live_nd[i] and not occ:
                key = (a, b) if a < b else (b, a)
                g = cache.get(key) if tm is not None else (0.0, 0.0, False)
                if key not in prm.tracker_blocked_edges and g is not None and g[2] is False:
                    prm._add_edge_without_data(a, b, d)
                    added += 1
                    n_fast += 1
                    continue
            if prm._evaluate_and_add_edge(a, b, d, global_map, edge_safety_margin,
                                          occ_blocked=occ, fresh=fresh, live_prof=live_profs[i]):
                added += 1
        return added, n_fast

    @staticmethod
    def _restore_traversed_edges(prm, global_map):
        """The edges travelled by the robot are free by construction and are put back anyway
        (the stop nodes are often below min_edge_length), unless blocked afterwards."""
        for a, b in list(prm.traversed_edges):
            if a in prm.nodes and b in prm.nodes:
                if not prm._traversed_edge_ok(a, b, global_map):
                    prm.traversed_edges.discard((a, b))
                    continue
                (xa, ya), (xb, yb) = prm.nodes[a], prm.nodes[b]
                prm._evaluate_and_add_edge(a, b, float(math.hypot(xb - xa, yb - ya)), force=True)

    def _dirty_pairs(self, prm, global_map, todo):
        """
        Mask of the pairs to re-evaluate (edge rectangle, widened by
        edge_evaluation.DIRTY_MARGIN_M, touching an area of prm.dirty_rects), or None when the full
        build is needed: graph never built, terrain map reloaded from file (updated areas
        unknown), a new fall (mark_hazard does not go through a scan).

        The edges travelled by the robot and those blocked in the attempt are ALWAYS
        re-evaluated, together with those that were so at the previous update: their outcome
        depends on those sets, not on the new data. They are few.
        """
        if not getattr(prm, 'built', False) or not hasattr(prm, 'dirty_rects'):
            return None
        tm = getattr(prm, 'global_terrain_map', None)
        if tm is not None and getattr(tm, 'update_rects', None) is None:
            return None
        if len(getattr(global_map, 'hazards', ()) or ()) != getattr(prm, 'hazards_seen', 0):
            return None
        mask = np.zeros(len(todo), dtype=bool)
        if not todo:
            return mask
        ia = np.array([p[0] for p in todo]); ib = np.array([p[1] for p in todo])
        a = np.array([prm.nodes[i] for i in ia]); b = np.array([prm.nodes[i] for i in ib])
        m = edge_evaluation.DIRTY_MARGIN_M
        xmin, xmax = np.minimum(a[:, 0], b[:, 0]) - m, np.maximum(a[:, 0], b[:, 0]) + m
        ymin, ymax = np.minimum(a[:, 1], b[:, 1]) - m, np.maximum(a[:, 1], b[:, 1]) + m
        for (rx0, rx1, ry0, ry1) in prm.dirty_rects:
            mask |= (xmax >= rx0) & (xmin <= rx1) & (ymax >= ry0) & (ymin <= ry1)
        special = _special_pairs(prm) | getattr(prm, 'special_seen', set())
        if special:
            mask |= np.array([(min(a, b), max(a, b)) in special for a, b in zip(ia.tolist(), ib.tolist())])
        return mask

    def _pairs_distinct(self):
        """True if no lattice pair appears twice (in either direction). Computed once."""
        cached = getattr(self, '_distinct_cache', None)
        if cached is None:
            p = np.sort(np.asarray(self.pairs, dtype=np.int64).reshape(-1, 2), axis=1)
            cached = bool(p.shape[0] == 0 or np.unique(p, axis=0).shape[0] == p.shape[0])
            self._distinct_cache = cached
        return cached

    def as_sampler(self):
        """The lattice points cell by cell (get_point_in_cell), for find_best_point_in_cell."""
        return LatticeSampler(self)

    # -- persistence and diagnostics -----------------------------------------------------
    def save_npz(self, path):
        """
        Save the lattice: indices, LOCAL coordinates, pairs, lengths, parameters. The local
        coordinates do not depend on the robot pose, so the file is valid for any mission of
        that size: it is reloaded with load_npz and only the roto-translation is applied.
        Serves to have an identical lattice across several missions; it is not needed for
        speed (building costs ~30 ms, reloading ~20).
        """
        keys = np.array(list(self.cell_center_ids.keys()), dtype=np.int64).reshape(-1, 2)
        vals = np.array(list(self.cell_center_ids.values()), dtype=np.int64)
        np.savez_compressed(
            path, ij=self.ij, xy_local=self.xy_local, pairs=self.pairs, lengths=self.lengths,
            node_cell=self.node_cell, spacing_m=self.spacing_m, rows=self.rows, cols=self.cols,
            cell_size_m=self.cell_size_m, start_cell=np.array(self.start_cell),
            min_edge_m=self.min_edge_m, reach_m=self.reach_m,
            rings=(-1 if self.rings is None else self.rings),
            origin_node_id=(-1 if self.origin_node_id is None else self.origin_node_id),
            cell_center_keys=keys, cell_center_vals=vals, n_short_pairs=self.n_short_pairs,
            sampling=self.sampling, distribution=str(self.distribution or ''),
            density_per_m2=(-1.0 if self.density_per_m2 is None else self.density_per_m2),
            seed=(-1 if self.seed is None else self.seed),
            covering=(np.array([-1.0, -1.0]) if self.covering is None
                      else np.array(self.covering, dtype=np.float64)),
            sampling_note=self.sampling_note)
        return path

    def report(self):
        """The summary to print in the log, next to the [CONFIG] lines."""
        lo = float(self.lengths.min()) if self.n_edges else float('nan')
        hi = float(self.lengths.max()) if self.n_edges else float('nan')
        uniq = sorted(set(np.round(self.lengths, 3).tolist()))
        rings = ", ".join(f"{v:.2f}" for v in uniq[:6]) + (" ..." if len(uniq) > 6 else "")
        if self.is_random:
            head = [
                f"[GRAPH] Mission {self.rows}x{self.cols} cells of {self.cell_size_m:.1f} m "
                f"= {self.area_m2:.0f} m2; RANDOM nodes, edges between {self.min_edge_m:.2f} and "
                f"{self.reach_m:.2f} m",
                # test_replay reads distribution, density and seed from this line.
                f"[GRAPH] Random nodes: distribution {self.distribution}, density "
                f"{self.density_per_m2:.2f} nodes/m2, seed {self.seed} "
                f"(to repeat: --graph random --distribution {self.distribution} "
                f"--density {self.density_per_m2:g} --seed {self.seed})",
            ]
            if self.sampling_note:
                head.append(f"[GRAPH] CAREFUL: {self.sampling_note}")
            density = self.n_nodes / self.area_m2
            cover = (f"[GRAPH] Measured coverage {self.covering[0]:.2f} m (95% of the area within "
                     f"{self.covering[1]:.2f} m; a corridor usable by the frontier is at least "
                     f"{2 * geometry.FRONTIER_CLEARANCE_M:.2f} m wide)")
        else:
            n_div = int(round(self.cell_size_m / self.spacing_m))
            head = [
                f"[GRAPH] Mission {self.rows}x{self.cols} cells of {self.cell_size_m:.1f} m "
                f"= {self.area_m2:.0f} m2; rectangular lattice, spacing {self.spacing_m:.4f} m "
                f"(= {self.cell_size_m:.1f}/{n_div}), rings "
                f"{'all within %.2f m' % self.reach_m if self.rings is None else self.rings}",
            ]
            density = node_density(self.spacing_m)
            cover = (f"[GRAPH] Coverage {self.covering_m:.2f} m (a corridor usable by the frontier is "
                     f"at least {2 * geometry.FRONTIER_CLEARANCE_M:.2f} m wide)")
        out = head + [
            f"[GRAPH] {self.n_nodes} nodes, {density:.2f} nodes/m2, "
            f"{self.n_edges} edges, mean degree {self.degrees.mean():.1f} "
            f"(min {int(self.degrees.min()) if self.n_nodes else 0}, "
            f"max {int(self.degrees.max()) if self.n_nodes else 0})",
            f"[GRAPH] Edge lengths {lo:.2f}-{hi:.2f} m"
            f"{'' if self.is_random else ' [%s]' % rings}, "
            f"min_edge_length={self.min_edge_m:.2f} m, "
            f"{self.n_short_pairs} pairs of neighbours too close, not linked",
            cover,
            f"[GRAPH] Cell goals: {len(self.cell_center_ids)} out of "
            f"{self.rows * self.cols}, all at the exact geometric centre",
            f"[GRAPH] Geometric connectivity: "
            + ("SINGLE component, no isolated node"
               if self.is_connected and self.n_isolated == 0 else
               f"{self.n_components} components, the largest with {self.biggest_component} "
               f"nodes out of {self.n_nodes}, {self.n_isolated} isolated"),
        ]
        if self.origin_node_id is not None:
            out.append(f"[GRAPH] Power-on = centre of the starting cell = node "
                       f"{self.origin_node_id} (no boot node to add)")
        if not self.is_connected or self.n_isolated:
            if self.is_random:
                out.append(f"[GRAPH] CAREFUL: the random graph is not connected with these "
                           f"parameters (density {self.density_per_m2:g} nodes/m2, edges "
                           f"{self.min_edge_m:.2f}-{self.reach_m:.2f} m): increase the density "
                           f"or the maximum edge length.")
            else:
                out.append(f"[GRAPH] CAREFUL: the lattice is not connected with these parameters "
                           f"(spacing {self.spacing_m:.4f} m, edges {self.min_edge_m:.2f}-"
                           f"{self.reach_m:.2f} m): increase the rings.")
        return "\n".join(out)

    def preview_png(self, path, show_cells=True, zoom=None):
        """
        Draw the graph and its edges into a PNG, to look at it before going out.
        zoom: (x_min, x_max, y_min, y_max) in the grid frame, for the details.
        """
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(9.5, 9.5))
        self.draw(ax, show_cells=show_cells, zoom=zoom)
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
        return path

    def draw(self, ax, show_cells=True, zoom=None, legend=True, title_size=None,
             edge_color='0.78', edge_width=0.45, highlight=None):
        """
        Draw nodes, edges, cell goals and cells on a matplotlib axes.
        highlight: id of a node whose edges to highlight (its neighbours).
        """
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        segs = [[(self.xy_local[a, 0], self.xy_local[a, 1]),
                 (self.xy_local[b, 0], self.xy_local[b, 1])] for a, b in self.pairs]
        ax.add_collection(LineCollection(segs, colors=edge_color, linewidths=edge_width, zorder=1))
        if highlight is not None:
            mine = [s for s, (a, b) in zip(segs, self.pairs) if highlight in (a, b)]
            ax.add_collection(LineCollection(mine, colors='tab:blue', linewidths=0.8, alpha=0.7,
                                             zorder=2, label=f"edges of node {highlight}"))
        ax.plot(self.xy_local[:, 0], self.xy_local[:, 1], 'k.', ms=2.6, zorder=3)
        ids = sorted(set(self.cell_center_ids.values()))
        if ids:
            ax.plot(self.xy_local[ids, 0], self.xy_local[ids, 1], 'o', ms=7, mfc='none',
                    mec='tab:orange', mew=1.4, zorder=4, label='cell goal')
        if self.origin_node_id is not None:
            ax.plot(self.xy_local[self.origin_node_id, 0], self.xy_local[self.origin_node_id, 1],
                    'bo', ms=9, zorder=5, label='power-on / starting cell centre')
        if show_cells:
            half = self.cell_size_m / 2.0
            for r in range(self.rows):
                for c in range(self.cols):
                    cx = (c - self.start_cell[1]) * self.cell_size_m
                    cy = (r - self.start_cell[0]) * self.cell_size_m
                    ax.add_patch(plt.Rectangle((cx - half, cy - half), self.cell_size_m,
                                               self.cell_size_m, fill=False, ec='tab:green',
                                               ls='--', lw=1.0, alpha=0.65, zorder=2))
                    ax.text(cx, cy - half + 0.35, f"{r},{c}", ha='center', va='bottom',
                            fontsize=7, color='tab:green', zorder=6, clip_on=True)
        if zoom:
            ax.set_xlim(zoom[0], zoom[1])
            ax.set_ylim(zoom[2], zoom[3])
        else:
            ax.autoscale_view()
        ax.set_aspect('equal', 'box')
        ax.set_xlabel('grid x [m] -- columns, ahead of the robot')
        ax.set_ylabel('grid y [m] -- rows, to the left of the robot')
        what = (f"random {self.distribution}, {self.density_per_m2:.3g} nodes/m2, seed {self.seed}"
                if self.is_random else
                f"spacing {self.spacing_m:.4f} m, rings "
                f"{self.rings if self.rings is not None else 'all'}")
        ax.set_title(f"{what}\n{self.n_nodes} nodes, {self.n_edges} edges, "
                     f"coverage {self.covering_m:.2f} m", fontsize=title_size)
        if legend:
            ax.legend(loc='upper right', fontsize=8)


class LatticeSampler:
    """The lattice points cell by cell (see MissionGraph.as_sampler)."""

    def __init__(self, mg):
        self._points = [(float(x), float(y)) for x, y in mg.xy]
        self._by_cell = {}
        for k, (r, c) in enumerate(mg.node_cell):
            self._by_cell.setdefault((int(r), int(c)), []).append(k)

    def get_point_in_cell(self, row, col):
        return [self._points[k] for k in self._by_cell.get((int(row), int(col)), ())]

# ========================================================================================
# THE FUNCTION TO CALL
# ========================================================================================

def build_mission_graph(rows=None, cols=None, cell_size_m=None, spacing_m=_FROM_SETTINGS,
                        rings=P.MISSION_LATTICE_RINGS,
                        origin_xy=(0.0, 0.0), origin_yaw=0.0, start_cell=(0, 0),
                        min_edge_length=P.PRM_MIN_EDGE_M,
                        max_edge_length=P.PRM_MAX_EDGE_M,
                        connection_radius=P.PRM_CONNECTION_RADIUS_M,
                        border_margin_m=0.0, env=None, verbose=True,
                        sampling=None, distribution=None, density_per_m2=None, seed=None,
                        poisson_min_dist_m=None):
    """
    Build the mission graph.

    Args:
        rows, cols, cell_size_m: the MISSION SIZE. If not passed: those of
            mission_settings() (MISSION_*, or the DEFAULT_*). Ignored if `env` is passed.
        spacing_m: SPACING FLAG, in metres; it is rounded to cell_size/n (see
            align_spacing). It can be below min_edge_length (see SPACING). If not passed:
            the one of mission_settings() (P.MISSION_LATTICE_SPACING_M, or 0.25 m); None =
            suggested_spacing().
        rings: CONNECTIONS FLAG -- k = all nodes within the diagonal of the k-th ring
            (k*spacing*sqrt(2)), with edges >= min_edge_length (see RINGS). None = all those
            within min(connection_radius, max_edge_length).
        origin_xy, origin_yaw: robot pose at start-up in the VISION frame. The lattice is
            anchored here. Ignored if `env` is passed.
        start_cell: cell where the robot is powered on (like env.start_cell).
        min_edge_length, max_edge_length, connection_radius: the same parameters as the PRM
            that will consume the graph -- they must match those passed to PRM().
        border_margin_m: margin to leave inside the mission border.
        env: an EnvironmentMap already created and with set_origin() done. If passed, size
            and pose come from it: it is the way to guarantee that graph and cell map cannot
            get out of sync.
        sampling: 'lattice' or 'random' (see RANDOM NODES); None = P.MISSION_GRAPH_SAMPLING. With
            'random' spacing_m and rings do not count.
        distribution, density_per_m2, seed, poisson_min_dist_m: the parameters of the random
            nodes; None = P.MISSION_RANDOM_DISTRIBUTION, P.MISSION_RANDOM_DENSITY,
            P.MISSION_RANDOM_SEED, P.MISSION_POISSON_MIN_DIST_M. Seed None there too = drawn at random (and written in the log).

    Returns:
        MissionGraph.
    """
    if env is not None:
        rows, cols, cell_size_m = env.rows, env.cols, env.cell_size
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
        start_cell = tuple(env.start_cell)
    rows, cols, cell_size_m, chosen_spacing = mission_settings(rows, cols, cell_size_m)
    if spacing_m is _FROM_SETTINGS:
        spacing_m = chosen_spacing
    if rows < 1 or cols < 1 or cell_size_m <= 0:
        raise ValueError("invalid mission size")
    sampling = P.MISSION_GRAPH_SAMPLING if sampling is None else sampling
    if sampling not in GRAPH_SAMPLINGS:
        raise ValueError(f"unknown node method {sampling!r}: one of {GRAPH_SAMPLINGS}")
    if sampling == 'random':
        return _build_random_graph(
            rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
            float(min_edge_length), min(float(connection_radius), float(max_edge_length)),
            float(border_margin_m), verbose,
            P.MISSION_RANDOM_DISTRIBUTION if distribution is None else distribution,
            P.MISSION_RANDOM_DENSITY if density_per_m2 is None else density_per_m2,
            P.MISSION_RANDOM_SEED if seed is None else seed,
            P.MISSION_POISSON_MIN_DIST_M if poisson_min_dist_m is None else poisson_min_dist_m)

    return _build_lattice_graph(
        rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw, spacing_m, rings,
        float(min_edge_length), min(float(connection_radius), float(max_edge_length)),
        float(border_margin_m), verbose)


def _build_lattice_graph(rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw, spacing_m,
                         rings, min_edge_length, prm_reach, border_margin_m, verbose):
    """The lattice graph (see build_mission_graph): spacing aligned to the cell, reach of
    the rings, nodes, pairs, cell centres."""
    if spacing_m is None:
        spacing_m = suggested_spacing(cell_size_m, min_edge_length)
    if float(spacing_m) <= 0:
        raise ValueError("the spacing must be positive")
    requested = float(spacing_m)
    spacing_m, n_div = align_spacing(requested, cell_size_m)
    if abs(spacing_m - requested) > 1e-12 and verbose:
        print(f"[GRAPH] Spacing aligned to the cell side: {requested:.3f} -> {spacing_m:.4f} m "
              f"(= {cell_size_m:.1f}/{n_div}); it is the condition for every cell centre "
              f"to be a lattice node.")
    if rings is not None:
        rings = int(rings)
    reach_m = _lattice_reach(spacing_m, rings, prm_reach, min_edge_length, verbose)

    extent = mission_extent(rows, cols, cell_size_m, start_cell)
    ij, xy_local = lattice_points(spacing_m, extent, border_margin_m)
    if len(xy_local) == 0:
        raise ValueError("no node in the mission rectangle: spacing too large "
                         "or border margin too wide")
    pairs, lengths, n_short = lattice_pairs(spacing_m, ij, xy_local, min_edge_length, reach_m)

    cell_center_ids = _lattice_cell_centres(ij, xy_local, rows, cols, cell_size_m, start_cell,
                                            spacing_m, n_div)
    if verbose and len(cell_center_ids) < rows * cols:
        print(f"[GRAPH] {rows * cols - len(cell_center_ids)} cell centres outside the lattice "
              f"(border margin {border_margin_m:.2f} m): those cells have no goal.")

    # Node at the origin: it is there by construction (the lattice is anchored there, and the origin is
    # also the centre of the starting cell), but we look for it instead of assuming it.
    d0 = np.hypot(xy_local[:, 0], xy_local[:, 1])
    origin_node_id = int(np.argmin(d0)) if d0.min() < 1e-9 else None

    node_cell = _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell)

    mg = MissionGraph(spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                      ij, xy_local, pairs, lengths, node_cell, cell_center_ids,
                      origin_node_id, min_edge_length, reach_m, rings, n_short)
    if verbose:
        print(mg.report())
    return mg


def _lattice_reach(spacing_m, rings, prm_reach, min_edge_length, verbose):
    """How far the edges reach: the diagonal of ring `rings`, at most the PRM reach (None =
    the PRM reach)."""
    if rings is None:
        return prm_reach
    if rings < 1:
        raise ValueError("rings must be at least 1")
    ring_reach = rings * spacing_m * math.sqrt(2.0)      # diagonal of ring k
    if ring_reach > prm_reach + 1e-9 and verbose:
        print(f"[GRAPH] {rings} rings would reach {ring_reach:.2f} m, beyond the "
              f"PRM reach {prm_reach:.2f} m: I stop there.")
    # Relative tolerance (1e-6): the lattice distances are irrational multiples of the
    # spacing, and without tolerance the ring diagonal could fall outside by one ulp.
    reach_m = min(ring_reach, prm_reach) * (1.0 + 1e-6)
    if reach_m < min_edge_length:
        raise ValueError(
            f"with spacing {spacing_m:.4f} m and {rings} rings the edges reach "
            f"{reach_m:.2f} m, below min_edge_length {min_edge_length:.2f} m: "
            f"no edge would be allowed. Increase the rings or the spacing.")
    return reach_m


def _lattice_cell_centres(ij, xy_local, rows, cols, cell_size_m, start_cell, spacing_m, n_div):
    """
    {(row, col): id of the node at the centre}. The cell centres are lattice nodes by
    construction (spacing = cell_size/n): here it is checked that this is really so instead
    of taking it for granted, because it is the property this whole design choice rests on.
    """
    cell_center_ids, tol = {}, max(1e-9, spacing_m * 1e-9)
    by_index = {(int(a), int(b)): k for k, (a, b) in enumerate(ij)}
    for r in range(rows):
        for c in range(cols):
            j = (c - start_cell[1]) * n_div
            i = (r - start_cell[0]) * n_div
            k = by_index.get((i, j))
            if k is None:
                continue                      # cell outside the lattice (border margin)
            if abs(xy_local[k, 0] - (c - start_cell[1]) * cell_size_m) > tol or \
                    abs(xy_local[k, 1] - (r - start_cell[0]) * cell_size_m) > tol:
                raise AssertionError(
                    f"node {k} should be the centre of cell ({r},{c}) but it is "
                    f"not: spacing {spacing_m} not aligned with cell_size {cell_size_m}")
            cell_center_ids[(r, c)] = k
    return cell_center_ids


def _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell):
    """Cell (row, column) of every node: same rule as env.get_cell_from_world (round)."""
    cols_idx = np.clip(np.round(xy_local[:, 0] / cell_size_m).astype(np.int64) + start_cell[1],
                       0, cols - 1)
    rows_idx = np.clip(np.round(xy_local[:, 1] / cell_size_m).astype(np.int64) + start_cell[0],
                       0, rows - 1)
    return np.column_stack((rows_idx, cols_idx))


def _build_random_graph(rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                        min_edge_m, reach_m, border_margin_m, verbose,
                        distribution, density_per_m2, seed, poisson_min_dist_m):
    """
    The graph with random nodes (see RANDOM NODES): first the cell centres, in the order of
    rows and columns, then the nodes of the distribution. Same seed, same graph.
    """
    if distribution not in RANDOM_DISTRIBUTIONS:
        raise ValueError(f"unknown distribution {distribution!r}: one of {RANDOM_DISTRIBUTIONS}")
    if density_per_m2 is None or float(density_per_m2) <= 0:
        raise ValueError("the node density must be positive")
    if reach_m < min_edge_m:
        raise ValueError(f"maximum edge length {reach_m:.2f} m below the minimum "
                         f"{min_edge_m:.2f} m: no edge would be allowed")
    seed = draw_seed() if seed is None else int(seed)
    rng = np.random.default_rng(seed)

    x_min, x_max, y_min, y_max = mission_extent(rows, cols, cell_size_m, start_cell)
    box = (x_min + border_margin_m, x_max - border_margin_m,
           y_min + border_margin_m, y_max - border_margin_m)
    if box[1] <= box[0] or box[3] <= box[2]:
        raise ValueError("border margin too wide: no area for the nodes")

    # Cell centres: fixed nodes, the exploration goals. The centre of the starting cell
    # is the origin, i.e. the power-on point.
    centers, keys = [], []
    for r in range(rows):
        for c in range(cols):
            x, y = (c - start_cell[1]) * cell_size_m, (r - start_cell[0]) * cell_size_m
            if box[0] <= x <= box[1] and box[2] <= y <= box[3]:
                centers.append((x, y))
                keys.append((r, c))
    centers = np.array(centers, dtype=np.float64).reshape(-1, 2)
    rand_xy, note = random_points(distribution, density_per_m2, box, centers, rng,
                                   poisson_min_dist_m)
    xy_local = np.vstack((centers, rand_xy))
    cell_center_ids = {key: k for k, key in enumerate(keys)}
    if verbose and len(cell_center_ids) < rows * cols:
        print(f"[GRAPH] {rows * cols - len(cell_center_ids)} cell centres outside the area "
              f"(border margin {border_margin_m:.2f} m): those cells have no goal.")
    d0 = np.hypot(xy_local[:, 0], xy_local[:, 1])
    origin_node_id = int(np.argmin(d0)) if len(d0) and d0.min() < 1e-9 else None

    pairs, lengths, n_short = radius_pairs(xy_local, min_edge_m, reach_m)
    node_cell = _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell)
    mg = MissionGraph(1.0 / math.sqrt(float(density_per_m2)), rows, cols, cell_size_m,
                      start_cell, origin_xy, origin_yaw,
                      -np.ones((len(xy_local), 2), dtype=np.int64), xy_local, pairs, lengths,
                      node_cell, cell_center_ids, origin_node_id, min_edge_m, reach_m, None,
                      n_short, sampling='random', distribution=distribution,
                      density_per_m2=float(density_per_m2), seed=seed,
                      covering=measured_covering(xy_local, box), sampling_note=note)
    if verbose:
        print(mg.report())
    return mg


def load_npz(path, origin_xy=(0.0, 0.0), origin_yaw=0.0, env=None):
    """
    Reload a lattice saved with save_npz and bring it into the VISION frame with the given
    pose (or that of `env`): only the roto-translation is applied.
    """
    d = np.load(path, allow_pickle=False)
    if env is not None:
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
    keys = d['cell_center_keys'].reshape(-1, 2)
    vals = d['cell_center_vals'].reshape(-1)
    cc_ids = {(int(r), int(c)): int(v) for (r, c), v in zip(keys, vals)}
    oid, rings = int(d['origin_node_id']), int(d['rings'])
    return MissionGraph(
        float(d['spacing_m']), int(d['rows']), int(d['cols']), float(d['cell_size_m']),
        tuple(int(v) for v in d['start_cell']), origin_xy, origin_yaw, d['ij'], d['xy_local'],
        d['pairs'], d['lengths'], d['node_cell'], cc_ids, None if oid < 0 else oid,
        float(d['min_edge_m']), float(d['reach_m']), None if rings < 0 else rings,
        int(d['n_short_pairs']),
        # files saved before random nodes: lattice
        **({} if 'sampling' not in d.files else dict(
            sampling=str(d['sampling']), distribution=(str(d['distribution']) or None),
            density_per_m2=(None if float(d['density_per_m2']) < 0 else float(d['density_per_m2'])),
            seed=(None if int(d['seed']) < 0 else int(d['seed'])),
            covering=(None if float(d['covering'][0]) < 0 else tuple(float(v) for v in d['covering'])),
            sampling_note=str(d['sampling_note']))))


