#!/usr/bin/env python3
"""
Prova AL BANCO della chiusura d'anello di Spot (ProcessTopology), su un grafo vero salvato.

Il robot NON si muove: serve acceso e collegato, anche seduto o in carica, con i motori spenti.
Lo script non accende i motori e non crea l'E-Stop; prende il lease, sostituisce la mappa
GraphNav che il robot ha in memoria con quella del grafo indicato (la missione la svuota
comunque all'avvio) e alla fine la svuota.

SERIE 1 (eseguita il 26-10 alle 12:41, vedi CHANGELOG), sul grafo intero:
  A. come il 22-10, ma aspettando la fine: quanto ci mette davvero e quanti archi trova;
  B-E. con il limite lato Spot (timeout_seconds), con meno coppie candidate
     (max_loop_closure_path_length), senza controllo di collisione;
  F. la chiamata con limite 4 s ripetuta, modificando la mappa: il lavoro prosegue o riparte?

SERIE 3 (default; sul grafo del 26-10 15:11, che ha gia' gli archi della chiusura veloce fatta
prima di ogni spostamento): la chiusura COMPLETA di fine missione, che il 26-10 ha dato 2 archi
in 65 s. Con prune_edges di Spot (un solo arco per zona: le zone erano gia' collegate) e senza
(prune_edges=False: archi corretti sulle scansioni anche accanto a quelli veloci), con 45 e 90 s.
Poi il modo usato in missione dal 26-10 sera: Spot calcola senza scrivere e gli archi si
caricano con upload_graph (niente MapModifiedError); si contano gli archi della mappa prima e
dopo.

SERIE 2:
  1. Grafo ricostruito come a inizio, meta' e fine missione (i primi 100, 200, tutti i 388
     waypoint in ordine di creazione): a ogni dimensione, le idee per una chiusura d'anello
     veloce PRIMA degli spostamenti -- limite 2 s, senza ICP (l'allineamento delle scansioni:
     se il costo e' li', diventa veloce, ma gli archi non sono piu' verificati), senza feature
     matching, solo anelli veri (waypoint ad almeno 8 m lungo il grafo) -- e una chiamata da
     60 s per vedere fino a che dimensione l'elaborazione FINISCE.
  2. Grafo intero, A FINE MISSIONE: limiti 45, 60, 90, 120 s. Quanti archi in piu', e se Spot
     si ferma comunque verso i 30 s come nella serie 1.
Tutte le misure senza modificare la mappa (modify_map_on_server=False).

Uso (chiede utente e password del robot, se non sono in BOSDYN_CLIENT_USERNAME/PASSWORD):
    python3 prova_anelli.py                 # serie 3, circa 10 minuti
    python3 prova_anelli.py --serie 2       # la serie del 26-10 13:02, circa 20 minuti
    python3 prova_anelli.py --serie 1       # la serie del 26-10 12:41
    python3 prova_anelli.py --grafo graph/Mission_<nome>/Grafo_<...> --hostname 192.168.80.3

Risultati a schermo e in MissionLogs/prova_anelli_<data>.txt (+ .json).
"""
import argparse
import datetime
import json
import os
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT_GRAPH = 'graph/Mission_22-10-2026_16-04-22/Grafo_22-10-2026_16-48-11'
DEFAULT_GRAPH_3 = 'graph/Mission_26-10-2026_15-11-21/Grafo_26-10-2026_15-39-18'

# --- Serie 1 --------------------------------------------------------------------------
# (nome, parametri di close_loops_checked). Per A, senza limite lato Spot, la RPC aspetta
# fino a 300 s.
CONFIGS_1 = [
    ("A. come giovedi', senza limite", dict(timeout_s=None)),
    ("B. limite 4 s", dict(timeout_s=4.0)),
    ("C. limite 4 s, strada <= 20 m", dict(timeout_s=4.0, max_path_m=20.0)),
    ("D. limite 4 s, senza collisioni", dict(timeout_s=4.0, check_collision=False)),
    ("E. limite 20 s (ritorno a wp_0)", dict(timeout_s=20.0)),
]
REPEAT_CALLS = 8

# --- Serie 2 --------------------------------------------------------------------------
STAGES = [100, 200, None]          # waypoint del grafo ricostruito; None = tutti
CONFIGS_BEFORE = [
    ("limite 2 s", dict(timeout_s=2.0)),
    ("limite 4 s", dict(timeout_s=4.0)),
    ("limite 4 s, senza ICP", dict(timeout_s=4.0, icp_iters=0)),
    ("limite 4 s, senza feature matching", dict(timeout_s=4.0, feature_matching=False)),
    ("limite 4 s, senza ICP ne' feature", dict(timeout_s=4.0, icp_iters=0, feature_matching=False)),
    ("limite 4 s, solo anelli oltre 8 m", dict(timeout_s=4.0, min_path_m=8.0)),
    ("limite 60 s (finisce?)", dict(timeout_s=60.0)),
]
CONFIGS_FINAL = [
    ("fine missione, limite 45 s", dict(timeout_s=45.0)),
    ("fine missione, limite 60 s", dict(timeout_s=60.0)),
    ("fine missione, limite 90 s", dict(timeout_s=90.0)),
    ("fine missione, limite 120 s", dict(timeout_s=120.0)),
]


def log(fh, msg=""):
    print(msg, flush=True)
    fh.write(msg + "\n")
    fh.flush()


def subgraph(graph, n):
    """I primi n waypoint in ordine di creazione e gli archi fra loro (n=None: tutto)."""
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
    Carica sul robot la mappa vuota + graph, con le istantanee. Restituisce (secondi, waypoint
    caricati, archi caricati). Con done_wp/done_edges (gia' sul robot) carica solo la parte
    nuova; se il robot la rifiuta, svuota e ricarica tutto.
    """
    from bosdyn.api.graph_nav import map_pb2
    t0 = time.perf_counter()
    done_wp, done_edges = set(done_wp or ()), set(done_edges or ())
    if done_wp:
        try:
            gn.upload_graph(graph=graph, generate_new_anchoring=False)
        except Exception as e:
            log(fh, f"[ANELLI-BANCO] aggiunta al grafo rifiutata ({type(e).__name__}): svuoto e ricarico tutto")
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
            log(fh, f"[ANELLI-BANCO] istantanee dei waypoint: {n} ({time.perf_counter() - t0:.0f} s)")
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
    """Una chiamata senza modificare la mappa; riga di log e riga di risultato."""
    kw = dict(kw)
    t = time.perf_counter()
    try:
        if kw.get('timeout_s') is None:
            # Senza limite lato Spot: la RPC aspetta fino a 300 s.
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
        row = dict(waypoint=n_wp, prova=name, secondi=round(dt, 1), archi=n, parziale=partial)
        log(fh, f"[ANELLI-BANCO] {n_wp:4d} wp  {name:38s} {dt:6.1f} s  {n:3d} archi  "
                f"{'PARZIALE (limite scaduto)' if partial else 'FINITA'}")
    except Exception as e:
        dt = time.perf_counter() - t
        row = dict(waypoint=n_wp, prova=name, secondi=round(dt, 1), errore=f"{type(e).__name__}: {e}")
        log(fh, f"[ANELLI-BANCO] {n_wp:4d} wp  {name:38s} {dt:6.1f} s  ERRORE {type(e).__name__}: {e}")
    rows.append(row)


def serie_1(ri, gn, gdir, graph, fh, results):
    dt, _, _ = upload(gn, gdir, graph, fh)
    results['caricamento_s'] = round(dt, 1)
    log(fh, f"[ANELLI-BANCO] Caricato in {dt:.0f} s\n")
    results['prove'] = []
    for name, kw in CONFIGS_1:
        measure(ri, fh, name, kw, len(graph.waypoints), results['prove'])
    log(fh, f"\n[ANELLI-BANCO] F. limite 4 s ripetuto {REPEAT_CALLS} volte, la mappa si modifica:")
    rep = []
    for k in range(1, REPEAT_CALLS + 1):
        t = time.perf_counter()
        try:
            n, partial = ri.close_loops_checked(max_edge_length_m=2.0, timeout_s=4.0)
            dt = time.perf_counter() - t
            rep.append(dict(chiamata=k, secondi=round(dt, 1), archi=n, parziale=partial))
            log(fh, f"[ANELLI-BANCO]   chiamata {k}: {dt:5.1f} s  {n:3d} archi  "
                    f"{'parziale' if partial else 'completa'}")
            if not partial:
                break
        except Exception as e:
            rep.append(dict(chiamata=k, errore=f"{type(e).__name__}: {e}"))
            log(fh, f"[ANELLI-BANCO]   chiamata {k}: ERRORE {type(e).__name__}: {e}")
            break
    results['ripetute'] = rep
    log(fh, f"[ANELLI-BANCO]   archi nel grafo dopo F: {len(gn.download_graph().edges)} "
            f"(prima {len(graph.edges)})")


def serie_2(ri, gn, gdir, graph, fh, results):
    results['prove'] = []
    results['caricamenti_s'] = []
    done_wp, done_edges = None, None
    for stage in STAGES:
        g = subgraph(graph, stage)
        log(fh, f"\n[ANELLI-BANCO] --- grafo con {len(g.waypoints)} waypoint e {len(g.edges)} archi ---")
        dt, done_wp, done_edges = upload(gn, gdir, g, fh, done_wp, done_edges)
        results['caricamenti_s'].append(round(dt, 1))
        on_robot = len(gn.download_graph().waypoints)
        log(fh, f"[ANELLI-BANCO] caricato in {dt:.0f} s; sul robot {on_robot} waypoint")
        if on_robot != len(g.waypoints):
            log(fh, "[ANELLI-BANCO] il grafo sul robot non e' quello atteso: svuoto e ricarico tutto")
            dt, done_wp, done_edges = upload(gn, gdir, g, fh)
        for name, kw in CONFIGS_BEFORE:
            measure(ri, fh, name, kw, len(g.waypoints), results['prove'])
    log(fh, "")
    for name, kw in CONFIGS_FINAL:
        measure(ri, fh, name, kw, len(graph.waypoints), results['prove'])


CONFIGS_3 = [
    ("completa 45 s, come il 26-10", dict(timeout_s=45.0)),
    ("completa 45 s, senza prune", dict(timeout_s=45.0, prune_edges=False)),
    ("completa 90 s, senza prune", dict(timeout_s=90.0, prune_edges=False)),
]


def serie_3(ri, gn, gdir, graph, fh, results):
    dt, _, _ = upload(gn, gdir, graph, fh)
    results['caricamento_s'] = round(dt, 1)
    src = {}
    for e in graph.edges:
        src[e.annotations.edge_source] = src.get(e.annotations.edge_source, 0) + 1
    log(fh, f"[ANELLI-BANCO] Caricato in {dt:.0f} s; archi per origine: {src}\n")
    results['prove'] = []
    for name, kw in CONFIGS_3:
        measure(ri, fh, name, kw, len(graph.waypoints), results['prove'])
    # Il modo della missione: calcolo senza scrivere + upload_graph degli archi trovati.
    log(fh, "")
    before = len(gn.download_graph().edges)
    t = time.perf_counter()
    try:
        n, partial = ri.close_loops_checked(max_edge_length_m=2.0, timeout_s=45.0, upload_result=True,
                                            prune_edges=False)
        dt = time.perf_counter() - t
        after = len(gn.download_graph().edges)
        row = dict(prova="caricamento con upload_graph (45 s, senza prune)", secondi=round(dt, 1),
                   archi=n, parziale=partial, archi_mappa_prima=before, archi_mappa_dopo=after)
        log(fh, f"[ANELLI-BANCO] caricamento con upload_graph: {n} archi calcolati in {dt:.1f} s; archi "
                f"nella mappa {before} -> {after} ({'OK' if after - before == n else 'NON TORNA'})")
    except Exception as e:
        row = dict(prova="caricamento con upload_graph", errore=f"{type(e).__name__}: {e}")
        log(fh, f"[ANELLI-BANCO] caricamento con upload_graph: ERRORE {type(e).__name__}: {e}")
    results['prove'].append(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--grafo', default=None)
    ap.add_argument('--hostname', default="192.168.80.3")
    ap.add_argument('--serie', type=int, choices=(1, 2, 3), default=3)
    args = ap.parse_args()
    if args.grafo is None:
        args.grafo = DEFAULT_GRAPH_3 if args.serie == 3 else DEFAULT_GRAPH

    from bosdyn.api.graph_nav import map_pb2
    import bosdyn.client.lease
    import spotLogInUtils
    import navGraphUtils

    gdir = args.grafo if os.path.isabs(args.grafo) else os.path.join(HERE, args.grafo)
    graph = map_pb2.Graph()
    with open(os.path.join(gdir, 'graph'), 'rb') as f:
        graph.ParseFromString(f.read())
    stamp = datetime.datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
    out_txt = os.path.join(HERE, 'MissionLogs', f'prova_anelli_{stamp}.txt')
    fh = open(out_txt, 'w')
    log(fh, f"[ANELLI-BANCO] Serie {args.serie}. Grafo {gdir}: {len(graph.waypoints)} waypoint, "
            f"{len(graph.edges)} archi")

    options = SimpleNamespace(name="provaAnelli", hostname=args.hostname, verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, lease_client, _rsc, client_metadata = spotLogInUtils.setLogInfo(options)
    ri = navGraphUtils.RecordingInterface(robot, HERE, client_metadata)
    gn = ri._graph_nav_client
    results = dict(serie=args.serie, grafo=gdir, waypoint=len(graph.waypoints), archi=len(graph.edges))

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        try:
            ri.stop_recording()
        except Exception:
            pass
        try:
            {1: serie_1, 2: serie_2, 3: serie_3}[args.serie](ri, gn, gdir, graph, fh, results)
        finally:
            gn.clear_graph()
            log(fh, "[ANELLI-BANCO] Mappa del robot svuotata.")

    with open(out_txt.replace('.txt', '.json'), 'w') as f:
        json.dump(results, f, indent=1)
    log(fh, f"[ANELLI-BANCO] Risultati in {out_txt} e .json")
    fh.close()


if __name__ == '__main__':
    main()
