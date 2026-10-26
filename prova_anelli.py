#!/usr/bin/env python3
"""
Prova AL BANCO della chiusura d'anello di Spot (ProcessTopology), su un grafo vero salvato.

Il robot NON si muove: serve acceso e collegato, anche seduto, con i motori spenti. Lo script
sostituisce la mappa GraphNav che il robot ha in memoria con quella del grafo indicato (la
missione la svuota comunque all'avvio) e alla fine la svuota.

Perche': il 22-10 16:04 la chiusura d'anello prima di ogni spostamento GraphNav e' andata in
timeout 20 volte (Spot elabora fino a 45 s se non gli si da' un limite, la RPC si arrende a
30 s). Qui si misura, sullo stesso grafo (388 waypoint, 560 MB di istantanee):
  A. come giovedi', ma aspettando la fine: quanto ci mette davvero e quanti archi trova;
  B-E. con il limite lato Spot (timeout_seconds) e con meno coppie candidate
     (max_loop_closure_path_length), senza controllo di collisione per vederne il costo;
  F. la stessa chiamata con limite di 4 s ripetuta, modificando la mappa: il lavoro prosegue
     da una chiamata all'altra (come promette map_processing.proto) o riparte da capo?
A-E non modificano la mappa (modify_map_on_server=False): misurano tutte lo stesso grafo.

Uso (chiede utente e password del robot, se non sono in BOSDYN_CLIENT_USERNAME/PASSWORD):
    python3 prova_anelli.py
    python3 prova_anelli.py --grafo graph/Mission_<nome>/Grafo_<...> --hostname 192.168.80.3

Risultati a schermo e in MissionLogs/prova_anelli_<data>.txt (+ .json). Dura circa 10-20
minuti, quasi tutti per caricare le istantanee.
"""
import argparse
import datetime
import glob
import json
import os
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT_GRAPH = 'graph/Mission_22-10-2026_16-04-22/Grafo_22-10-2026_16-48-11'

# (nome, parametri di close_loops_checked). Il limite della RPC e' timeout_s + 10 s; per A, che
# non ha limite lato Spot, la RPC aspetta fino a 300 s.
CONFIGS = [
    ("A. come giovedi', senza limite", dict(timeout_s=None)),
    ("B. limite 4 s", dict(timeout_s=4.0)),
    ("C. limite 4 s, strada <= 20 m", dict(timeout_s=4.0, max_path_m=20.0)),
    ("D. limite 4 s, senza collisioni", dict(timeout_s=4.0, check_collision=False)),
    ("E. limite 20 s (ritorno a wp_0)", dict(timeout_s=20.0)),
]
REPEAT_CALLS = 8


def log(fh, msg=""):
    print(msg, flush=True)
    fh.write(msg + "\n")
    fh.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--grafo', default=DEFAULT_GRAPH)
    ap.add_argument('--hostname', default="192.168.80.3")
    args = ap.parse_args()

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
    log(fh, f"[ANELLI-BANCO] Grafo {gdir}: {len(graph.waypoints)} waypoint, {len(graph.edges)} archi")

    options = SimpleNamespace(name="provaAnelli", hostname=args.hostname, verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, lease_client, _rsc, client_metadata = spotLogInUtils.setLogInfo(options)
    ri = navGraphUtils.RecordingInterface(robot, HERE, client_metadata)
    gn = ri._graph_nav_client
    results = dict(grafo=gdir, waypoint=len(graph.waypoints), archi=len(graph.edges), prove=[])

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        try:
            ri.stop_recording()
        except Exception:
            pass
        gn.clear_graph()

        # --- Caricamento -------------------------------------------------------------
        t0 = time.perf_counter()
        gn.upload_graph(graph=graph, generate_new_anchoring=False)
        n_up = 0
        for w in graph.waypoints:
            if not w.snapshot_id:
                continue
            snap = map_pb2.WaypointSnapshot()
            with open(os.path.join(gdir, 'waypoint_snapshots', w.snapshot_id), 'rb') as f:
                snap.ParseFromString(f.read())
            gn.upload_waypoint_snapshot(snap)
            n_up += 1
            if n_up % 50 == 0:
                log(fh, f"[ANELLI-BANCO] istantanee dei waypoint: {n_up} ({time.perf_counter() - t0:.0f} s)")
        for e in graph.edges:
            if not e.snapshot_id:
                continue
            path = os.path.join(gdir, 'edge_snapshots', e.snapshot_id)
            if not os.path.exists(path):
                continue
            snap = map_pb2.EdgeSnapshot()
            with open(path, 'rb') as f:
                snap.ParseFromString(f.read())
            gn.upload_edge_snapshot(snap)
        results['caricamento_s'] = round(time.perf_counter() - t0, 1)
        log(fh, f"[ANELLI-BANCO] Caricato in {results['caricamento_s']:.0f} s ({n_up} istantanee dei waypoint)")

        # --- A-E: la mappa non cambia ---------------------------------------------------
        log(fh, "")
        for name, kw in CONFIGS:
            kw = dict(kw)
            t = time.perf_counter()
            try:
                if kw.get('timeout_s') is None:
                    # Come giovedi' ma senza arrendersi a 30 s: la RPC aspetta fino a 300 s.
                    kw.pop('timeout_s')
                    orig = ri._map_processing_client.process_topology
                    ri._map_processing_client.process_topology = \
                        lambda *a, **k: orig(*a, timeout=300.0, **k)
                    try:
                        n, partial = ri.close_loops_checked(max_edge_length_m=2.0,
                                                            modify_map_on_server=False, **kw)
                    finally:
                        ri._map_processing_client.process_topology = orig
                else:
                    n, partial = ri.close_loops_checked(max_edge_length_m=2.0,
                                                        modify_map_on_server=False, **kw)
                dt = time.perf_counter() - t
                row = dict(prova=name, secondi=round(dt, 1), archi=n, parziale=partial)
                log(fh, f"[ANELLI-BANCO] {name:36s} {dt:6.1f} s  {n:3d} archi  "
                        f"{'PARZIALE (limite scaduto)' if partial else 'completa'}")
            except Exception as e:
                dt = time.perf_counter() - t
                row = dict(prova=name, secondi=round(dt, 1), errore=f"{type(e).__name__}: {e}")
                log(fh, f"[ANELLI-BANCO] {name:36s} {dt:6.1f} s  ERRORE {type(e).__name__}: {e}")
            results['prove'].append(row)

        # --- F: chiamate ripetute col limite, modificando la mappa ------------------------
        log(fh, "")
        log(fh, f"[ANELLI-BANCO] F. limite 4 s ripetuto {REPEAT_CALLS} volte, la mappa si modifica:")
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

        gn.clear_graph()
        log(fh, "[ANELLI-BANCO] Mappa del robot svuotata.")

    with open(out_txt.replace('.txt', '.json'), 'w') as f:
        json.dump(results, f, indent=1)
    log(fh, f"[ANELLI-BANCO] Risultati in {out_txt} e .json")
    fh.close()


if __name__ == '__main__':
    main()
