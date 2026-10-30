#!/usr/bin/env python3
"""
BENCH test of Spot's loop closure (ProcessTopology), on a real saved graph.

The robot does NOT move: it must be on and connected, even sitting or charging, with the
motors off. The script does not power the motors and does not create the E-Stop; it takes
the lease, replaces the GraphNav map the robot has in memory with the given graph (the
mission clears it at start-up anyway) and clears it at the end.

SERIES 1 (run on 26-10 at 12:41, see CHANGELOG), on the whole graph:
  A. as on 22-10, but waiting for the end: how long it really takes and how many edges it finds;
  B-E. with the limit on Spot's side (timeout_seconds), with fewer candidate pairs
     (max_loop_closure_path_length), without collision checking;
  F. the call with a 4 s limit repeated, modifying the map: does the work continue or restart?

SERIES 3 (default; on the graph of 26-10 15:11, which already has the edges of the quick
closure done before every move): the FULL end-of-mission closure, which on 26-10 gave 2
edges in 65 s. With Spot's prune_edges (one edge per area: the areas were already
connected) and without (prune_edges=False: edges corrected on the scans even next to the
quick ones), with 45 and 90 s. Then the mode used in the mission since the evening of 26-10:
Spot computes without writing and the edges are uploaded with upload_graph (no
MapModifiedError); the map's edges are counted before and after.

SERIES 2:
  1. Graph rebuilt as at the start, middle and end of the mission (the first 100, 200, all
     388 waypoints in creation order): at each size, the ideas for a quick loop closure
     BEFORE moves -- 2 s limit, without ICP (scan alignment: if the cost is there, it
     becomes fast, but the edges are no longer verified), without feature matching, only
     true loops (waypoints at least 8 m apart along the graph) -- and a 60 s call to see up
     to which size the processing FINISHES.
  2. Whole graph, AT THE END OF THE MISSION: limits 45, 60, 90, 120 s. How many more edges,
     and whether Spot stops around 30 s anyway as in series 1.
All measurements without modifying the map (modify_map_on_server=False).

Usage (asks for the robot's user and password, if not in BOSDYN_CLIENT_USERNAME/PASSWORD):
    python3 tools/bench_loop_closure.py               # series 3, about 10 minutes
    python3 tools/bench_loop_closure.py --series 2    # the series of 26-10 13:02, about 20 minutes
    python3 tools/bench_loop_closure.py --series 1    # the series of 26-10 12:41
    python3 tools/bench_loop_closure.py --graph graph/Mission_<name>/Grafo_<...> --hostname 192.168.80.3

Results on screen and in MissionLogs/prova_anelli_<date>.txt (+ .json).
"""
import argparse
import datetime
import json
import os
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # project folder
sys.path.insert(0, HERE)

DEFAULT_GRAPH = 'graph/Mission_22-10-2026_16-04-22/Grafo_22-10-2026_16-48-11'
DEFAULT_GRAPH_3 = 'graph/Mission_26-10-2026_15-11-21/Grafo_26-10-2026_15-39-18'

# --- Series 1 -------------------------------------------------------------------------
# (name, close_loops_checked parameters). For A, without a limit on Spot's side, the RPC
# waits up to 300 s.
CONFIGS_1 = [
    ("A. as on Thursday, no limit", dict(timeout_s=None)),
    ("B. limit 4 s", dict(timeout_s=4.0)),
    ("C. limit 4 s, path <= 20 m", dict(timeout_s=4.0, max_path_m=20.0)),
    ("D. limit 4 s, no collisions", dict(timeout_s=4.0, check_collision=False)),
    ("E. limit 20 s (return to wp_0)", dict(timeout_s=20.0)),
]
REPEAT_CALLS = 8

# --- Series 2 -------------------------------------------------------------------------
STAGES = [100, 200, None]          # waypoints of the rebuilt graph; None = all
CONFIGS_BEFORE = [
    ("limit 2 s", dict(timeout_s=2.0)),
    ("limit 4 s", dict(timeout_s=4.0)),
    ("limit 4 s, no ICP", dict(timeout_s=4.0, icp_iters=0)),
    ("limit 4 s, no feature matching", dict(timeout_s=4.0, feature_matching=False)),
    ("limit 4 s, no ICP or features", dict(timeout_s=4.0, icp_iters=0, feature_matching=False)),
    ("limit 4 s, only loops beyond 8 m", dict(timeout_s=4.0, min_path_m=8.0)),
    ("limit 60 s (does it finish?)", dict(timeout_s=60.0)),
]
CONFIGS_FINAL = [
    ("end of mission, limit 45 s", dict(timeout_s=45.0)),
    ("end of mission, limit 60 s", dict(timeout_s=60.0)),
    ("end of mission, limit 90 s", dict(timeout_s=90.0)),
    ("end of mission, limit 120 s", dict(timeout_s=120.0)),
]


def log(fh, msg=""):
    print(msg, flush=True)
    fh.write(msg + "\n")
    fh.flush()


def subgraph(graph, n):
    """The first n waypoints in creation order and the edges between them (n=None: everything)."""
    from bosdyn.api.graph_nav import map_pb2
    if n is None or n >= len(graph.waypoints):
        return graph
    wps = sorted(graph.waypoints, key=lambda w: (w.annotations.creation_time.seconds,
                                                 w.annotations.creation_time.nanos))[:n]
    keep = {w.id for w in wps}
    g = map_pb2.Graph()
    g.waypoints.extend(wps)
    g.edges.extend(e for e in graph.edges
                   if e.id.from_waypoint in keep and e.id.to_waypoint in keep)
    return g


def upload(gn, gdir, graph, fh, done_wp=None, done_edges=None):
    """
    Upload the empty map + graph to the robot, with the snapshots. Returns (seconds,
    waypoints uploaded, edges uploaded). With done_wp/done_edges (already on the robot) it
    uploads only the new part; if the robot rejects it, it clears and uploads everything.
    """
    from bosdyn.api.graph_nav import map_pb2
    t0 = time.perf_counter()
    done_wp, done_edges = set(done_wp or ()), set(done_edges or ())
    if done_wp:
        try:
            gn.upload_graph(graph=graph, generate_new_anchoring=False)
        except Exception as e:
            log(fh, f"[LOOPS-BENCH] adding to the graph rejected ({type(e).__name__}): clearing and uploading everything")
            gn.clear_graph()
            done_wp, done_edges = set(), set()
    if not done_wp:
        gn.clear_graph()
        gn.upload_graph(graph=graph, generate_new_anchoring=False)
    n = 0
    for w in graph.waypoints:
        if not w.snapshot_id or w.id in done_wp:
            continue
        snap = map_pb2.WaypointSnapshot()
        with open(os.path.join(gdir, 'waypoint_snapshots', w.snapshot_id), 'rb') as f:
            snap.ParseFromString(f.read())
        gn.upload_waypoint_snapshot(snap)
        n += 1
        if n % 50 == 0:
            log(fh, f"[LOOPS-BENCH] waypoint snapshots: {n} ({time.perf_counter() - t0:.0f} s)")
    for e in graph.edges:
        key = (e.id.from_waypoint, e.id.to_waypoint)
        if not e.snapshot_id or key in done_edges:
            continue
        path = os.path.join(gdir, 'edge_snapshots', e.snapshot_id)
        if not os.path.exists(path):
            continue
        snap = map_pb2.EdgeSnapshot()
        with open(path, 'rb') as f:
            snap.ParseFromString(f.read())
        gn.upload_edge_snapshot(snap)
    return (time.perf_counter() - t0, {w.id for w in graph.waypoints},
            {(e.id.from_waypoint, e.id.to_waypoint) for e in graph.edges})


def measure(ri, fh, name, kw, n_wp, rows):
    """One call without modifying the map; log line and result line."""
    kw = dict(kw)
    t = time.perf_counter()
    try:
        if kw.get('timeout_s') is None:
            # Without a limit on Spot's side: the RPC waits up to 300 s.
            kw.pop('timeout_s', None)
            orig = ri._map_processing_client.process_topology
            ri._map_processing_client.process_topology = lambda *a, **k: orig(*a, timeout=300.0, **k)
            try:
                n, partial = ri.close_loops_checked(max_edge_length_m=2.0, modify_map_on_server=False, **kw)
            finally:
                ri._map_processing_client.process_topology = orig
        else:
            n, partial = ri.close_loops_checked(max_edge_length_m=2.0, modify_map_on_server=False, **kw)
        dt = time.perf_counter() - t
        row = dict(waypoint=n_wp, run=name, seconds=round(dt, 1), edges=n, partial=partial)
        log(fh, f"[LOOPS-BENCH] {n_wp:4d} wp  {name:38s} {dt:6.1f} s  {n:3d} edges  "
                f"{'PARTIAL (limit expired)' if partial else 'FINISHED'}")
    except Exception as e:
        dt = time.perf_counter() - t
        row = dict(waypoint=n_wp, run=name, seconds=round(dt, 1), error=f"{type(e).__name__}: {e}")
        log(fh, f"[LOOPS-BENCH] {n_wp:4d} wp  {name:38s} {dt:6.1f} s  ERROR {type(e).__name__}: {e}")
    rows.append(row)


def series_1(ri, gn, gdir, graph, fh, results):
    dt, _, _ = upload(gn, gdir, graph, fh)
    results['upload_s'] = round(dt, 1)
    log(fh, f"[LOOPS-BENCH] Uploaded in {dt:.0f} s\n")
    results['runs'] = []
    for name, kw in CONFIGS_1:
        measure(ri, fh, name, kw, len(graph.waypoints), results['runs'])
    log(fh, f"\n[LOOPS-BENCH] F. limit 4 s repeated {REPEAT_CALLS} times, the map changes:")
    rep = []
    for k in range(1, REPEAT_CALLS + 1):
        t = time.perf_counter()
        try:
            n, partial = ri.close_loops_checked(max_edge_length_m=2.0, timeout_s=4.0)
            dt = time.perf_counter() - t
            rep.append(dict(call=k, seconds=round(dt, 1), edges=n, partial=partial))
            log(fh, f"[LOOPS-BENCH]   call {k}: {dt:5.1f} s  {n:3d} edges  "
                    f"{'partial' if partial else 'complete'}")
            if not partial:
                break
        except Exception as e:
            rep.append(dict(call=k, error=f"{type(e).__name__}: {e}"))
            log(fh, f"[LOOPS-BENCH]   call {k}: ERROR {type(e).__name__}: {e}")
            break
    results['ripetute'] = rep
    log(fh, f"[LOOPS-BENCH]   edges in the graph after F: {len(gn.download_graph().edges)} "
            f"(before {len(graph.edges)})")


def series_2(ri, gn, gdir, graph, fh, results):
    results['runs'] = []
    results['uploads_s'] = []
    done_wp, done_edges = None, None
    for stage in STAGES:
        g = subgraph(graph, stage)
        log(fh, f"\n[LOOPS-BENCH] --- graph with {len(g.waypoints)} waypoints and {len(g.edges)} edges ---")
        dt, done_wp, done_edges = upload(gn, gdir, g, fh, done_wp, done_edges)
        results['uploads_s'].append(round(dt, 1))
        on_robot = len(gn.download_graph().waypoints)
        log(fh, f"[LOOPS-BENCH] uploaded in {dt:.0f} s; {on_robot} waypoints on the robot")
        if on_robot != len(g.waypoints):
            log(fh, "[LOOPS-BENCH] the graph on the robot is not the expected one: clearing and uploading everything")
            dt, done_wp, done_edges = upload(gn, gdir, g, fh)
        for name, kw in CONFIGS_BEFORE:
            measure(ri, fh, name, kw, len(g.waypoints), results['runs'])
    log(fh, "")
    for name, kw in CONFIGS_FINAL:
        measure(ri, fh, name, kw, len(graph.waypoints), results['runs'])


CONFIGS_3 = [
    ("full 45 s, as on 26-10", dict(timeout_s=45.0)),
    ("full 45 s, no prune", dict(timeout_s=45.0, prune_edges=False)),
    ("full 90 s, no prune", dict(timeout_s=90.0, prune_edges=False)),
]


def series_3(ri, gn, gdir, graph, fh, results):
    dt, _, _ = upload(gn, gdir, graph, fh)
    results['upload_s'] = round(dt, 1)
    src = {}
    for e in graph.edges:
        src[e.annotations.edge_source] = src.get(e.annotations.edge_source, 0) + 1
    log(fh, f"[LOOPS-BENCH] Uploaded in {dt:.0f} s; edges by source: {src}\n")
    results['runs'] = []
    for name, kw in CONFIGS_3:
        measure(ri, fh, name, kw, len(graph.waypoints), results['runs'])
    # The mission mode: compute without writing + upload_graph of the edges found.
    log(fh, "")
    before = len(gn.download_graph().edges)
    t = time.perf_counter()
    try:
        n, partial = ri.close_loops_checked(max_edge_length_m=2.0, timeout_s=45.0, upload_result=True,
                                            prune_edges=False)
        dt = time.perf_counter() - t
        after = len(gn.download_graph().edges)
        row = dict(run="upload with upload_graph (45 s, no prune)", seconds=round(dt, 1),
                   edges=n, partial=partial, map_edges_before=before, map_edges_after=after)
        log(fh, f"[LOOPS-BENCH] upload with upload_graph: {n} edges computed in {dt:.1f} s; edges "
                f"in the map {before} -> {after} ({'OK' if after - before == n else 'DOES NOT ADD UP'})")
    except Exception as e:
        row = dict(run="upload with upload_graph", error=f"{type(e).__name__}: {e}")
        log(fh, f"[LOOPS-BENCH] upload with upload_graph: ERROR {type(e).__name__}: {e}")
    results['runs'].append(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--graph', default=None)
    ap.add_argument('--hostname', default="192.168.80.3")
    ap.add_argument('--series', type=int, choices=(1, 2, 3), default=3)
    args = ap.parse_args()
    if args.graph is None:
        args.graph = DEFAULT_GRAPH_3 if args.series == 3 else DEFAULT_GRAPH

    from bosdyn.api.graph_nav import map_pb2
    import bosdyn.client.lease
    from spot_explorer.robot import connection
    from spot_explorer.graphnav import map_recording

    gdir = args.graph if os.path.isabs(args.graph) else os.path.join(HERE, args.graph)
    graph = map_pb2.Graph()
    with open(os.path.join(gdir, 'graph'), 'rb') as f:
        graph.ParseFromString(f.read())
    stamp = datetime.datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
    out_txt = os.path.join(HERE, 'MissionLogs', f'prova_anelli_{stamp}.txt')
    fh = open(out_txt, 'w')
    log(fh, f"[LOOPS-BENCH] Series {args.series}. Graph {gdir}: {len(graph.waypoints)} waypoints, "
            f"{len(graph.edges)} edges")

    options = SimpleNamespace(name="provaAnelli", hostname=args.hostname, verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, lease_client, _rsc, client_metadata = connection.setLogInfo(options)
    ri = map_recording.RecordingInterface(robot, HERE, client_metadata)
    gn = ri._graph_nav_client
    results = dict(series=args.series, graph=gdir, waypoint=len(graph.waypoints), edges=len(graph.edges))

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        try:
            ri.stop_recording()
        except Exception:
            pass
        try:
            {1: series_1, 2: series_2, 3: series_3}[args.series](ri, gn, gdir, graph, fh, results)
        finally:
            gn.clear_graph()
            log(fh, "[LOOPS-BENCH] Robot map cleared.")

    with open(out_txt.replace('.txt', '.json'), 'w') as f:
        json.dump(results, f, indent=1)
    log(fh, f"[LOOPS-BENCH] Results in {out_txt} and .json")
    fh.close()


if __name__ == '__main__':
    main()
