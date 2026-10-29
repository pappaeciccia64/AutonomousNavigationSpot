"""Scorciatoie nel grafo GraphNav: archi nuovi sui tratti visti liberi, scelti sulla strada verso la destinazione."""
import time
import numpy as np
from spot_explorer import config as P
from spot_explorer.records import MISSION_CTX, MISSION_STATS
from spot_explorer.graphnav.localization import graphnav_localization
from spot_explorer.graphnav.map_recording import graphnav_is_recording
from spot_explorer.graphnav.localization import wp_name_by_id


def segment_seen_free(global_map, x1, y1, x2, y2, margin_m, step_m=0.05):
    """True se il segmento e' stato osservato in ogni punto e non passa vicino a celle occupate."""
    n = max(2, int(np.ceil(np.hypot(x2 - x1, y2 - y1) / step_m)) + 1)
    for t in np.linspace(0.0, 1.0, n):
        x, y = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
        if not global_map.is_known(x, y) or global_map.is_occupied(x, y, safety_margin=margin_m):
            return False
    return True


def graph_route(adj, start, goal):
    """Dijkstra su {wp: {vicino: lunghezza}}: (lunghezza, [wp...]) o (None, None)."""
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
    Scorciatoie da aggiungere al grafo GraphNav per andare da start_id a goal_id.
    Funzione pura (niente robot), provata offline sui grafi scaricati: vedi test_replay.py.

    poses: {wp_id: (x, y)} dei waypoint di cui si conosce la posizione (frame della mappa
        globale); base_edges: [(wp_a, wp_b, lunghezza_m)] gli archi che il grafo ha gia'.
    Candidate: coppie di waypoint con posizione a meno di max_m, senza arco diretto, la cui
    linea retta e' stata VISTA libera nella mappa globale con margin_m (segment_seen_free).
    Si calcola la strada piu' corta col grafo piu' TUTTE le candidate, e si restituiscono solo
    le candidate che quella strada usa.

    Restituisce (scorciatoie [(a, b, d)], lunghezza oggi, lunghezza con le scorciatoie); le
    lunghezze sono None se goal non e' raggiungibile.
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
    Aggiunge scorciatoie al grafo GraphNav prima di uno spostamento (vedi GRAPHNAV_SHORTCUTS).
    Non solleva mai: senza scorciatoie GraphNav funziona lo stesso, sulla catena.

    Spot accetta archi nuovi (create_edge) SOLO mentre registra ("NotRecordingError"),
    mentre la chiusura d'anello (process_topology) va fatta a registrazione ferma, e appena
    fermata puo' rispondere "MapModifiedError" finche' la mappa non e' finita di elaborare.
    Quindi:
      map_edges -- le scorciatoie dalla mappa globale: da chiamare PRIMA di stop_recording
                   (si saltano da sole se la registrazione e' ferma). Servono
                   target_wp_id (la destinazione) e, se il localizzatore non dice dove si
                   trova il robot, robot_xy; si creano solo quelle sulla strada (vedi
                   plan_graphnav_shortcuts).
      loop_closure_s -- se > 0, la chiusura d'anello completa di Spot con questo limite lato
                   Spot (s): da chiamare DOPO stop_recording (si salta da sola se si sta
                   registrando), con un secondo tentativo su MapModifiedError. Si fa una volta
                   sola, a fine missione (vedi GRAPHNAV_SDK_LOOP_BUDGET_FINAL_S).
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
        # Spot calcola senza scrivere e gli archi li carichiamo noi (upload_result): niente
        # MapModifiedError a fine elaborazione. Ogni tentativo fallito finisce nel log con la
        # sua durata, e il tempo riportato e' il TOTALE (il 26-10 15:36 i tentativi falliti non
        # comparivano e il tempo era solo quello dell'ultimo: 65 s dichiarati su 138).
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
                # Caricamento degli archi rifiutato: al secondo tentativo li scrive Spot stesso
                # (mai provato sul robot con archi da caricare: la serie 3 ne ha trovati 0).
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
