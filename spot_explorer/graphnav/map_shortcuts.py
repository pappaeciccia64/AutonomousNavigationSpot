"""Shortcuts in the GraphNav graph: new edges on stretches seen free, chosen on the way to the destination."""
import time
import numpy as np
from spot_explorer import config as P
from spot_explorer.records import MISSION_CTX, MISSION_STATS
from spot_explorer.graphnav.localization import graphnav_localization
from spot_explorer.graphnav.map_recording import graphnav_is_recording
from spot_explorer.graphnav.localization import wp_name_by_id


def segment_seen_free(global_map, x1, y1, x2, y2, margin_m, step_m=0.05):
    """True if the segment was observed at every point and does not pass near occupied cells."""
    n = max(2, int(np.ceil(np.hypot(x2 - x1, y2 - y1) / step_m)) + 1)
    for t in np.linspace(0.0, 1.0, n):
        x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
        if not global_map.is_known(x, y) or global_map.is_occupied(x, y, safety_margin=margin_m):
            return False
    return True


def graph_route(adj, start, goal):
    """Dijkstra on {wp: {neighbour: length}}: (length, [wp...]) or (None, None)."""
    import heapq
    dist, prev, pq = {start: 0.0}, {}, [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, float('inf')):
            continue
        for v, w in adj.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, float('inf')):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    if goal not in dist:
        return None, None
    path = [goal]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return dist[goal], path[::-1]


def plan_graphnav_shortcuts(poses, base_edges, start_id, goal_id, global_map,
                            max_m=None, margin_m=None):
    """
    Shortcuts to add to the GraphNav graph to go from start_id to goal_id.
    Pure function (no robot), tested offline on the downloaded graphs: see test_replay.py.

    poses: {wp_id: (x, y)} of the waypoints whose position is known (global map frame);
        base_edges: [(wp_a, wp_b, length_m)] the edges the graph already has.
    Candidates: pairs of waypoints less than max_m apart, without a direct edge, whose
    straight line was SEEN free in the global map with margin_m (segment_seen_free).
    The shortest path is computed with the graph plus ALL the candidates, and only the
    candidates that path uses are returned.

    Returns (shortcuts [(a, b, d)], length today, length with the shortcuts); the lengths
    are None if goal is not reachable.
    """
    max_m = P.GRAPHNAV_SHORTCUT_MAX_M if max_m is None else max_m
    margin_m = P.GRAPHNAV_SHORTCUT_MARGIN_M if margin_m is None else margin_m
    adj = {}
    for a, b, w in base_edges:
        w = float(w)
        if w < adj.get(a, {}).get(b, float('inf')):
            adj.setdefault(a, {})[b] = w
            adj.setdefault(b, {})[a] = w
    before, _ = graph_route(adj, start_id, goal_id)
    ids = list(poses)
    shortcut = {}
    for i in range(len(ids)):
        xa, ya = poses[ids[i]]
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if b in adj.get(a, {}):
                continue
            xb, yb = poses[b]
            d = float(np.hypot(xa - xb, ya - yb))
            if d > max_m or d < 0.05:
                continue
            if segment_seen_free(global_map, xa, ya, xb, yb, margin_m):
                shortcut[frozenset((a, b))] = d
    full = {k: dict(v) for k, v in adj.items()}
    for k, d in shortcut.items():
        a, b = tuple(k)
        full.setdefault(a, {})[b] = d
        full.setdefault(b, {})[a] = d
    after, path = graph_route(full, start_id, goal_id)
    used = []
    if path:
        for a, b in zip(path[:-1], path[1:]):
            k = frozenset((a, b))
            if k in shortcut and b not in adj.get(a, {}):
                used.append((a, b, shortcut[k]))
    return used, before, after


def graphnav_add_shortcuts(recordingInterface, label="", map_edges=True, loop_closure_s=0.0,
                           target_wp_id=None, robot_xy=None):
    """
    Add shortcuts to the GraphNav graph before a move (see GRAPHNAV_SHORTCUTS).
    Never raises: without shortcuts GraphNav works anyway, along the chain.

    Spot accepts new edges (create_edge) ONLY while recording ("NotRecordingError"),
    while loop closure (process_topology) must be done with recording stopped, and right
    after stopping it may answer "MapModifiedError" until the map has finished processing.
    So:
      map_edges -- the shortcuts from the global map: call BEFORE stop_recording
                   (skipped automatically if recording is stopped). They need
                   target_wp_id (the destination) and, if the localizer does not say where
                   the robot is, robot_xy; only those on the way are created (see
                   plan_graphnav_shortcuts).
      loop_closure_s -- if > 0, Spot's full loop closure with this limit on Spot's side
                   (s): call AFTER stop_recording (skipped automatically while
                   recording), with a second attempt on MapModifiedError. Done only once,
                   at the end of the mission (see GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
    """
    if not P.GRAPHNAV_SHORTCUTS or recordingInterface is None:
        return 0
    added = 0
    recording = graphnav_is_recording(recordingInterface)
    gg = MISSION_CTX.get('global_grid')
    global_map = getattr(gg, 'global_occupancy_map', None) if gg is not None else None
    poses = getattr(recordingInterface, 'waypoint_poses', None) or {}
    if (map_edges and target_wp_id and recording is not False and global_map is not None
            and len(poses) >= 2):
        try:
            graph = recordingInterface._get_graph(force_refresh=True)
            by_id = {w.id: w for w in graph.waypoints}
            xy = {p['waypoint_id']: (float(p['x']), float(p['y'])) for p in poses.values()
                  if p.get('waypoint_id') in by_id}
            base = []
            for e in graph.edges:
                t = e.from_tform_to.position
                base.append((e.id.from_waypoint, e.id.to_waypoint,
                             float(np.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2))))
            start = (graphnav_localization(recordingInterface) or {}).get('wp_id')
            if start not in by_id:
                if robot_xy is not None and xy:
                    start = min(xy, key=lambda k: np.hypot(xy[k][0] - robot_xy[0], xy[k][1] - robot_xy[1]))
                else:
                    start = None
            if start is not None and target_wp_id in by_id:
                used, before, after = plan_graphnav_shortcuts(xy, base, start, target_wp_id, global_map)
                for a, b, d in used:
                    try:
                        recordingInterface.create_edge_between_waypoint_objs(by_id[a], by_id[b])
                        added += 1
                    except Exception as e:
                        print(f"[GRAPHNAV] Scorciatoia {wp_name_by_id(recordingInterface, a)}-"
                              f"{wp_name_by_id(recordingInterface, b)} rifiutata ({e}).")
                if before is not None and after is not None:
                    print(f"[GRAPHNAV] {label}: strada verso {wp_name_by_id(recordingInterface, target_wp_id)} "
                          f"{before:.1f} m -> {after:.1f} m con {len(used)} scorciatoie "
                          f"(fino a {P.GRAPHNAV_SHORTCUT_MAX_M:.1f} m, viste libere con "
                          f"{P.GRAPHNAV_SHORTCUT_MARGIN_M:.2f} m di margine).")
        except Exception as e:
            print(f"[GRAPHNAV] Scorciatoie dalla mappa non aggiunte ({e}).")
        finally:
            if added:
                recordingInterface.invalidate_graph_cache()
    n_sdk = 0
    budget = loop_closure_s
    if P.GRAPHNAV_SDK_LOOP_CLOSURE and budget > 0 and recording is not True:
        # Spot computes without writing and we upload the edges ourselves (upload_result): no
        # MapModifiedError at the end of processing. Every failed attempt ends up in the log with
        # its duration, and the time reported is the TOTAL (on 26-10 15:36 the failed attempts
        # did not show up and the time was only that of the last one: 65 s reported out of 138).
        what = 'completa'
        t_all = time.perf_counter()
        write_on_spot = False
        for attempt in range(1, 3):
            t0 = time.perf_counter()
            try:
                n_sdk, timed_out = recordingInterface.close_loops_checked(
                    max_edge_length_m=P.GRAPHNAV_SDK_LOOP_MAX_M, timeout_s=budget,
                    upload_result=not write_on_spot)
            except Exception as e:
                dt = time.perf_counter() - t0
                MISSION_STATS['sdk_loop_failures'] += 1
                retry = attempt == 1 and ('MapModified' in type(e).__name__ or 'modified' in str(e)
                                          or 'LoopClosureUpload' in type(e).__name__)
                # Edge upload rejected: on the second attempt Spot writes them itself
                # (never tried on the robot with edges to upload: series 3 found 0).
                write_on_spot = 'LoopClosureUpload' in type(e).__name__
                print(f"[GRAPHNAV] Chiusura d'anello di Spot ({what}): tentativo {attempt} non riuscito "
                      f"dopo {dt:.1f} s ({type(e).__name__}: {e})"
                      + (("; riprovo facendoli scrivere a Spot." if write_on_spot else "; riprovo.")
                         if retry else "; proseguo senza."))
                if retry:
                    time.sleep(1.0)
                    continue
                n_sdk = 0
                break
            dt_all = time.perf_counter() - t_all
            MISSION_STATS['sdk_loop_calls'] += 1
            MISSION_STATS['sdk_loop_s'] += dt_all
            MISSION_STATS['sdk_loop_edges'] += n_sdk
            MISSION_STATS['sdk_loop_partial'] += int(timed_out)
            print(f"[GRAPHNAV] Chiusura d'anello di Spot ({what}): {n_sdk} archi in {dt_all:.1f} s "
                  f"(limite {budget:.0f} s, "
                  f"{'SCADUTO: risultato parziale' if timed_out else 'finita'}"
                  + (f", {attempt} tentativi" if attempt > 1 else "") + ").")
            break
    if added or n_sdk:
        MISSION_STATS['graphnav_shortcuts'] = MISSION_STATS.get('graphnav_shortcuts', 0) + added + n_sdk
        print(f"[GRAPHNAV] {label}: {added} scorciatoie dalla mappa globale + {n_sdk} da Spot "
              f"(chiusure d'anello con controllo di collisione).")
    return added + n_sdk
