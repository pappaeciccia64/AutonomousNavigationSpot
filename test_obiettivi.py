"""
Prova della SCELTA DELL'OBIETTIVO DELLA CELLA sulle scansioni vere salvate in missione.

test_replay.py rigioca le scansioni ma usa l'obiettivo che la missione aveva scelto. Qui, a
ogni inizio di tentativo, si rifa' la scelta con il codice di oggi (easy_walk.
find_best_point_in_cell, con la raggiungibilita' nel grafo e la memoria degli obiettivi
rifiutati) e la si mette accanto a quella della missione, con le stesse misure:

  raggiungibile  -- il grafo, ricostruito come in missione, collega il robot all'obiettivo;
  rifiutabile    -- in una delle scansioni del tentativo l'obiettivo ha obstacle_distance
                    (filtrata, quella su cui decide il fronte) sotto FRONTIER_CLEARANCE_M,
                    oppure il fronte ha confermato un blocco entro REFUSED_GOAL_BLOCK_M da lui:
                    il robot non ci arriverebbe mai. Si giudica solo dove le scansioni lo
                    coprono, cioe' lungo la strada fatta in missione (vedi limiti). Molti
                    rifiutabili sono centri ancora ignoti all'inizio del tentativo: li riscopre
                    il ciclo di navigazione, nessuna scelta iniziale li puo' evitare;
  sul bordo      -- a meno di TARGET_CELL_BORDER_M dal bordo della cella;
  riproposto     -- uguale (entro 0.30 m) a un obiettivo della stessa cella gia' risultato
                    rifiutabile in un tentativo precedente.

Per il codice di oggi un obiettivo rifiutabile viene escluso per il resto della missione
(easy_walk._refuse_goal), come farebbe il ciclo di navigazione.

Uso:
    python3 test_obiettivi.py                          # la missione all'aperto del 22-10
    python3 test_obiettivi.py --missioni 13-10-2026_12-40-38 22-10-2026_15-50-23
    python3 test_obiettivi.py --dettaglio              # una riga per tentativo

Prima del rigioco, controlli sintetici delle regole su casi costruiti a mano.

Limiti: il robot rigiocato fa la strada della missione, non quella verso l'obiettivo nuovo;
per l'obiettivo nuovo "rifiutabile" si vede solo se le scansioni lo coprono. La scansione presa
all'inizio del tentativo non e' salvata: la mappa globale rigiocata ne ha meno, e un centro
che in missione risultava occupato qui puo' risultare libero (22-10 16:04, cella (5,2)). Esce con codice 1
se un controllo sintetico fallisce, o se il codice di oggi fa peggio della missione su
raggiungibili, rifiutabili o riproposti.
"""
import argparse
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_replay as tr          # noqa: E402  (importa anche easy_walk in silenzio)
from test_replay import ew, sg, P    # noqa: E402

DEFAULT = ['22-10-2026_16-04-22']


def _od_at(a, x, y):
    """obstacle_distance filtrata nel punto, None fuori dalla scansione."""
    c = int((x - a['ox']) / a['cs'])
    r = int((y - a['oy']) / a['cs'])
    if 0 <= r < a['ny'] and 0 <= c < a['nx']:
        return float(a['od_filt'][r, c])
    return None


def _bad(g_):
    """Il fronte rifiuterebbe l'obiettivo: spazio libero dal vivo sotto soglia, o blocco a ridosso."""
    return g_['xy'] is not None and (
        g_.get('blocked', False) or (bool(g_['od']) and min(g_['od']) < sg.FRONTIER_CLEARANCE_M))


def replay_goals(name):
    m = tr.mission_setup(name)
    if m is None:
        return None
    with tr._quiet():
        env = tr.environmentMap.EnvironmentMap(rows=m['rows'], cols=m['cols'], cell_size=m['cell'])
        env.set_origin(m['origin'][0], m['origin'][1], math.radians(m['yaw_deg']))
        P.MISSION_LATTICE_SPACING_M = m['spacing']
        g = m['graph']
        P.MISSION_GRAPH_SAMPLING = g['sampling']
        P.PRM_MIN_EDGE_M, P.PRM_MAX_EDGE_M = g['min_edge'], g['max_edge']
        if g['sampling'] == 'casuale':
            P.MISSION_RANDOM_DISTRIBUTION = g['distribution']
            P.MISSION_RANDOM_DENSITY = g['density']
            P.MISSION_RANDOM_SEED = g['seed']
        prm, sampler, mg = tr.mission_graph.build_mission_prm(env, verbose=False)
        gm = sg.GlobalGrid()
        tm = sg.GlobalTerrainGrid()
        tm.update_rects = []
        prm.set_global_terrain_map(tm)
    node_cells = None
    done_cells = []
    attempts = []          # un dict per tentativo
    cur = None
    for f in m['scans']:
        d = np.load(f)
        it, rr, cc = (int(v) for v in re.search(r'scan_it(\d+)_cell(\d+)_(\d+)', f).groups())
        a = tr.scan_arrays(d)
        rx, ry, _ = a['robot']
        with tr._quiet():
            tr.feed_scan(a, prm, gm, tm)
        if cur is None or cur['it'] != it:
            with tr._quiet():
                tr.mission_graph.rebuild_prm_edges(prm, gm)
                if node_cells is None:
                    node_cells = tr.mission_graph.node_cells(prm, env)
                cells = set(done_cells) | {(rr, cc)} | {tuple(int(v) for v in env.get_cell_from_world(rx, ry))}
                prm.set_allowed_nodes([n for n, c in node_cells.items() if c in cells])
                robot_node = prm.anchor_node(rx, ry, gm, sg.PRM_EDGE_SAFETY_MARGIN_M,
                                             ignore_near_m=prm_graph_ignore())
                prm.set_robot_node(robot_node, 0.0)

                def reach(px, py, _rn=robot_node):
                    nid = prm.anchor_node(px, py, gm, sg.PRM_EDGE_SAFETY_MARGIN_M)
                    return nid is not None and prm.find_path_dijkstra(_rn, nid) is not None

                nx_, ny_ = ew.find_best_point_in_cell(
                    rx, ry, env, rr, cc, a['pts'], a['od_raw'].ravel(), sampler,
                    global_map=gm, reachable=reach)[:2]
                mx, my = map(float, d['goal_xy'])
                cur = dict(it=it, cell=(rr, cc),
                           old=dict(xy=(mx, my), reach=reach(mx, my), od=[]),
                           new=dict(xy=None if nx_ is None else (float(nx_), float(ny_)),
                                    reach=(nx_ is not None and reach(nx_, ny_)), od=[]))
                prm.set_allowed_nodes(None)
            attempts.append(cur)
            done_cells.append((rr, cc))
        for k in ('old', 'new'):
            if cur[k]['xy'] is not None:
                v = _od_at(a, *cur[k]['xy'])
                if v is not None:
                    cur[k]['od'].append(v)
        # Blocco confermato a ridosso dell'obiettivo della missione (la strada rigiocata e'
        # quella verso di lui): vale per l'obiettivo nuovo solo se e' lo stesso punto.
        if (str(d['decision_kind']) == 'blocked'
                and str(d['frontier_reason']) in ('ostacolo', 'rugosita', 'pendenza')):
            sxy = tuple(map(float, d['frontier_stop_xy']))
            if math.dist(sxy, cur['old']['xy']) <= P.REFUSED_GOAL_BLOCK_M:
                cur['old']['blocked'] = True
                if (cur['new']['xy'] is not None
                        and math.dist(cur['new']['xy'], cur['old']['xy']) < P.RECOVERY_SAME_GOAL_M):
                    cur['new']['blocked'] = True
        # L'obiettivo nuovo rifiutabile si esclude per la missione, come fa il ciclo.
        if _bad(cur['new']):
            with tr._quiet():
                ew._refuse_goal(env, rr, cc, *cur['new']['xy'], "rigioco")
    # Misure
    out = {}
    for k in ('old', 'new'):
        refused_by_cell = {}
        n = dict(tentativi=0, senza_obiettivo=0, raggiungibili=0, rifiutabili=0, sul_bordo=0,
                 riproposti=0)
        for at in attempts:
            g_ = at[k]
            n['tentativi'] += 1
            if g_['xy'] is None:
                n['senza_obiettivo'] += 1
                continue
            n['raggiungibili'] += bool(g_['reach'])
            bad = _bad(g_)
            n['rifiutabili'] += bad
            b = tr.environmentMap.cell_border_distance(env, *at['cell'], *g_['xy'])
            n['sul_bordo'] += b is not None and b < P.TARGET_CELL_BORDER_M
            prev = refused_by_cell.setdefault(at['cell'], [])
            n['riproposti'] += any(math.dist(g_['xy'], p) < P.RECOVERY_SAME_GOAL_M for p in prev)
            if bad:
                prev.append(g_['xy'])
        out[k] = n
    return dict(misure=out, tentativi=attempts, env=env)


class _Map:
    """Mappa globale finta: ostacoli puntiformi, celle occupate entro OBSTACLE_THRESHOLD."""

    def __init__(self, obstacles):
        self.obs = [tuple(o) for o in obstacles]

    def _d(self, x, y):
        return min((math.dist((x, y), o) for o in self.obs), default=99.0)

    def is_occupied(self, x, y, safety_margin=0.0):
        return self._d(x, y) <= sg.OBSTACLE_THRESHOLD + safety_margin

    def is_known(self, x, y):
        return True

    def clearance(self, x, y):
        return self._d(x, y)


def synthetic_checks():
    """Le regole di find_best_point_in_cell su casi costruiti a mano. Lista di fallimenti."""
    fails = []

    def check(cond, msg):
        print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
        if not cond:
            fails.append(msg)

    def pick(obstacles, reachable=None, env=None, robot=(-3.0, 0.0)):
        env = env or _env()
        pts = [(x, y) for x in np.linspace(-0.9, 0.9, 7) for y in np.linspace(-0.9, 0.9, 7)]
        sampler = type('S', (), {'get_point_in_cell': lambda self, r, c: pts})()
        with tr._quiet():
            x, y, _, _ = ew.find_best_point_in_cell(robot[0], robot[1], env, 0, 0, None, None, sampler,
                                                    global_map=_Map(obstacles), reachable=reachable)
        return env, (x, y)

    def _env():
        e = tr.environmentMap.EnvironmentMap(rows=1, cols=1, cell_size=2.0)
        with tr._quiet():
            e.set_origin(0.0, 0.0, 0.0)
        return e

    print("Controlli sintetici della scelta dell'obiettivo (cella da 2 m centrata in (0, 0)):")
    e = _env()
    with tr._quiet():
        e30 = tr.environmentMap.EnvironmentMap(rows=1, cols=1, cell_size=2.0)
        e30.set_origin(0.0, 0.0, math.radians(30))
    check(abs(tr.environmentMap.cell_border_distance(e, 0, 0, 0.0, 0.0) - 1.0) < 1e-9
          and abs(tr.environmentMap.cell_border_distance(e, 0, 0, 0.9, 0.2) - 0.1) < 1e-9
          and abs(tr.environmentMap.cell_border_distance(e, 0, 0, 1.2, 0.0) + 0.2) < 1e-9,
          "distanza dal bordo: 1.0 al centro, 0.1 a 0.9 m, -0.2 fuori")
    p = (0.9 * math.cos(math.radians(30)), 0.9 * math.sin(math.radians(30)))
    check(abs(tr.environmentMap.cell_border_distance(e30, 0, 0, *p) - 0.1) < 1e-9,
          "distanza dal bordo con la griglia ruotata di 30 gradi")

    _, g = pick([])
    check(g == (0.0, 0.0), f"campo libero: il centro {g}")
    _, g = pick([], reachable=lambda x, y: math.hypot(x, y) > 0.5)
    check(g != (0.0, 0.0) and math.hypot(*g) > 0.5,
          f"centro non raggiungibile: un punto raggiungibile {tuple(round(v, 2) for v in g)}")
    _, g = pick([], reachable=lambda x, y: False)
    check(g == (0.0, 0.0), "nessun punto raggiungibile: comunque il migliore (il centro)")
    _, g = pick([(0.30, 0.0)])
    check(g != (0.0, 0.0) and math.dist(g, (0.30, 0.0)) >= P.TARGET_CLEARANCE_M,
          f"centro a 0.30 m da un ostacolo (sotto {P.TARGET_CLEARANCE_M:.2f}): un punto con piu' "
          f"spazio {tuple(round(v, 2) for v in g)}")
    _, g = pick([(0.0, 0.0)])
    b = tr.environmentMap.cell_border_distance(e, 0, 0, *g)
    check(b >= P.TARGET_CELL_BORDER_M,
          f"centro occupato: il ripiego sta ad almeno {P.TARGET_CELL_BORDER_M:.2f} m dal bordo ({b:.2f})")
    env, g1 = pick([(0.0, 0.0)])
    with tr._quiet():
        ew._refuse_goal(env, 0, 0, *g1, "prova")
    env, g2 = pick([(0.0, 0.0)], env=env)
    check(math.dist(g1, g2) >= P.RECOVERY_SAME_GOAL_M,
          f"obiettivo rifiutato non riproposto: {tuple(round(v, 2) for v in g1)} -> "
          f"{tuple(round(v, 2) for v in g2)}")
    env2 = _env()
    with tr._quiet():
        for x in np.linspace(-0.9, 0.9, 7):
            for y in np.linspace(-0.9, 0.9, 7):
                ew._refuse_goal(env2, 0, 0, x, y, "prova")
        ew._refuse_goal(env2, 0, 0, 0.0, 0.0, "prova")
    _, g = pick([], env=env2)
    check(g[0] is not None, "tutti i punti rifiutati: si riprova il migliore invece di arrendersi")
    return fails


def prm_graph_ignore():
    import prm_graph
    return prm_graph.ANCHOR_IGNORE_NEAR_M


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--missioni', nargs='+', default=DEFAULT)
    ap.add_argument('--dettaglio', action='store_true')
    args = ap.parse_args()
    worse = bool(synthetic_checks())
    for name in args.missioni:
        res = replay_goals(name)
        if res is None:
            print(f"{name}: scansioni o log non trovati su questo PC, salto.")
            continue
        o, n = res['misure']['old'], res['misure']['new']
        print(f"\n=== {name}: {o['tentativi']} inizi di tentativo ===")
        print(f"  {'':22s}{'missione':>10s}{'oggi':>8s}")
        for k, label in (('raggiungibili', 'raggiungibili'), ('rifiutabili', 'rifiutabili dal fronte'),
                         ('riproposti', 'riproposti dopo rifiuto'), ('sul_bordo', 'sul bordo della cella'),
                         ('senza_obiettivo', 'senza obiettivo')):
            print(f"  {label:22s}{o[k]:10d}{n[k]:8d}")
        if args.dettaglio:
            for at in res['tentativi']:
                def fmt(g_):
                    if g_['xy'] is None:
                        return "nessuno".ljust(38)
                    od = f"{min(g_['od']):.2f}" if g_['od'] else " -- "
                    return (f"({g_['xy'][0]:6.2f},{g_['xy'][1]:7.2f}) "
                            f"{'R' if g_['reach'] else '-'} od_min {od}").ljust(38)
                print(f"   it{at['it']:3d} cella {at['cell']}: missione {fmt(at['old'])} oggi {fmt(at['new'])}")
        if (n['raggiungibili'] < o['raggiungibili'] or n['rifiutabili'] > o['rifiutabili']
                or n['riproposti'] > o['riproposti']):
            worse = True
            print("  ATTENZIONE: il codice di oggi fa peggio della missione su almeno una misura.")
    sys.exit(1 if worse else 0)


if __name__ == '__main__':
    main()
