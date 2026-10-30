"""Veto statistics of the mission PRM (see roadmap.py): the [SLOPES] line per build or refresh
and the end-of-mission summary. Only diagnostics: they never decide."""

from spot_explorer.perception import slopes


class EdgeStats:
    """Mixin of roadmap.PRM: statistics for the logs and the end-of-mission summary, never used
    to decide."""

    def _init_stats(self):
        self.slope_source_counts = {'live': 0, 'global_map': 0, 'fallback_nodes': 0}
        self.last_slope_source = None  # last source used, read by the mission code for the per-edge log
        # These three count veto EVALUATIONS (the same edge is re-evaluated many times); the
        # DISTINCT edges are in the vetoed_arcs_* sets below.
        self.veto_count_longitudinal = 0  # veto evaluations for longitudinal slope
        self.veto_count_lateral = 0       # veto evaluations for lateral slope
        self.veto_count_occupancy = 0     # veto evaluations for global occupancy (not slope)
        self.vetoed_arcs_longitudinal = set()  # keys (min_id, max_id) of DISTINCT edges vetoed for long.
        self.vetoed_arcs_lateral = set()       # same for lateral
        self.vetoed_arcs_occupancy = set()     # same for occupancy
        self.max_slope_seen = {'long': 0.0, 'lat': 0.0}  # maxima measured on any evaluated edge
        self.slope_flip_count = 0         # times an edge near the threshold changed state between two refreshes
        self.edge_slope_info = {}         # {(idx1,idx2): (long_slope, lat_slope)} -- for visualization, no recomputation
        self._slope_log_pending = None    # vetoes since the last [SLOPES] line (see flush_slope_log)

    def _record_slope_veto(self, key, long_slope: float, lat_slope: float) -> bool:
        """
        Slope veto (slopes.SLOPE_THRESHOLD, in both directions) with the statistics.
        Returns True if the edge must be discarded. The vetoes are summarized in one [SLOPES]
        line per build or refresh (flush_slope_log); the edge-by-edge values stay in the _vis.npz.
        """
        self.max_slope_seen['long'] = max(self.max_slope_seen['long'], long_slope)
        self.max_slope_seen['lat'] = max(self.max_slope_seen['lat'], lat_slope)

        vetoed = False
        pend = self._slope_pending()
        if long_slope > slopes.SLOPE_THRESHOLD:
            vetoed = True
            self.veto_count_longitudinal += 1
            pend['long'] += 1
            pend['max_long'] = max(pend['max_long'], long_slope)
            if key not in self.vetoed_arcs_longitudinal:
                self.vetoed_arcs_longitudinal.add(key)
                pend['new'] += 1
        if lat_slope > slopes.SLOPE_THRESHOLD:
            vetoed = True
            self.veto_count_lateral += 1
            pend['lat'] += 1
            pend['max_lat'] = max(pend['max_lat'], lat_slope)
            if key not in self.vetoed_arcs_lateral:
                self.vetoed_arcs_lateral.add(key)
                pend['new'] += 1
        return vetoed

    def _slope_pending(self):
        p = self._slope_log_pending
        if p is None:
            p = self._slope_log_pending = {'long': 0, 'lat': 0, 'new': 0, 'flip': 0,
                                           'max_long': 0.0, 'max_lat': 0.0}
        return p

    def flush_slope_log(self, where=""):
        """
        One [SLOPES] line with the vetoes since the last call, then reset.
        Prints nothing if there were no vetoes.
        """
        p = self._slope_pending()
        if p['long'] or p['lat'] or p['flip']:
            print(f"[SLOPES] {where}: edges vetoed for slope -- longitudinal {p['long']} "
                  f"(max {p['max_long']:.2f}), lateral {p['lat']} (max {p['max_lat']:.2f}), "
                  f"{p['new']} never vetoed before; {p['flip']} verdict changes near the threshold "
                  f"(threshold {slopes.SLOPE_THRESHOLD:.2f}).")
        self._slope_log_pending = None

    def _record_occupancy_veto(self, key):
        """Debug count of the global occupancy vetoes (behaviour unchanged)."""
        self.veto_count_occupancy += 1
        self.vetoed_arcs_occupancy.add(key)

    def get_mission_summary_dict(self) -> dict:
        """The same statistics as print_mission_summary, in serializable form (JSON)."""
        return {
            'slope_source_counts': dict(self.slope_source_counts),
            'veto_evaluations': {
                'longitudinal': self.veto_count_longitudinal,
                'lateral': self.veto_count_lateral,
                'occupancy': self.veto_count_occupancy,
            },
            'veto_distinct_arcs': {
                'longitudinal': len(self.vetoed_arcs_longitudinal),
                'lateral': len(self.vetoed_arcs_lateral),
                'occupancy': len(self.vetoed_arcs_occupancy),
            },
            'max_slope_seen': dict(self.max_slope_seen),
            'slope_flip_count': self.slope_flip_count,
            'slope_threshold': slopes.SLOPE_THRESHOLD,
            'slope_baseline_m': slopes.SLOPE_BASELINE_M,
            'lateral_slice_half_width_m': slopes.LATERAL_SLICE_HALF_WIDTH_M,
            'beta': self.beta,
            'beta_lateral': self.beta_lateral,
            'gamma': self.gamma,
            'alpha': self.alpha,
        }

    def print_mission_summary(self):
        """
        Print a cumulative summary for the whole mission -- meant to be called once at the
        end of the mission, instead of having to read thousands of log lines to understand
        how often each mechanism really stepped in.
        Purely diagnostic, it does not affect the robot's behaviour.
        """
        total_sources = sum(self.slope_source_counts.values())
        print("\n================ [PRM MISSION SUMMARY] ================")
        print(f"Slope source used in the edge cost (total {total_sources} queries):")
        for src, count in self.slope_source_counts.items():
            pct = (100.0 * count / total_sources) if total_sources > 0 else 0.0
            print(f"  {src:15s}: {count:6d}  ({pct:5.1f}%)")
        print("Vetoes -- DISTINCT edges (and, in brackets, total evaluations, which count the same edge "
              "at every refresh):")
        print(f"  longitudinal: {len(self.vetoed_arcs_longitudinal)} ({self.veto_count_longitudinal})  "
              f"lateral: {len(self.vetoed_arcs_lateral)} ({self.veto_count_lateral})  "
              f"occupancy: {len(self.vetoed_arcs_occupancy)} ({self.veto_count_occupancy})")
        print(f"Maximum slope measured on an edge -- longitudinal: {self.max_slope_seen['long']:.3f}, "
              f"lateral: {self.max_slope_seen['lat']:.3f} (veto threshold {slopes.SLOPE_THRESHOLD:.3f})")
        print(f"Edge profile geometry -- base {slopes.SLOPE_BASELINE_M:.2f} m, "
              f"lateral slice +/-{slopes.LATERAL_SLICE_HALF_WIDTH_M:.2f} m")
        print(f"Cost weights -- alpha={self.alpha}, beta(long)={self.beta}, "
              f"beta_lateral={self.beta_lateral}, gamma={self.gamma}")
        print(f"Slope veto flips near the threshold (possible oscillation): {self.slope_flip_count}")
        print("=========================================================\n")
