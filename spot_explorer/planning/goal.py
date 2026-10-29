"""Scelta del punto obiettivo in una cella (find_best_point_in_cell) e obiettivi esclusi."""
import numpy as np

from spot_explorer.perception import geometry
from spot_explorer.perception import global_maps
from spot_explorer.mission import cells as environmentMap


from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS



# =========================================================================
# SCELTA DEL PUNTO OBIETTIVO in una cella
# =========================================================================


def _refused_goals(env, row, col):
    """Gli obiettivi della cella rifiutati dal fronte in questa missione (lista di (x, y))."""
    store = getattr(env, '_refused_goals', None)
    if store is None:
        store = env._refused_goals = {}
    return store.setdefault((int(row), int(col)), [])


def _refuse_goal(env, row, col, x, y, why):
    """
    Esclude l'obiettivo (x, y) della cella fino alla fine della missione. Il 22-10 16:04 il
    nodo 548 della cella (7,4) e' stato riscelto a ogni tentativo e bloccato tre volte nello
    stesso punto: la mappa globale lo dava libero a 0.28 m, il fronte dal vivo a 0.24.
    """
    lst = _refused_goals(env, row, col)
    if any(np.hypot(x - ex, y - ey) < P.RECOVERY_SAME_GOAL_M for ex, ey in lst):
        return
    lst.append((float(x), float(y)))
    MISSION_STATS['goals_refused'] = MISSION_STATS.get('goals_refused', 0) + 1
    print(f"[TARGET] Obiettivo ({x:.2f}, {y:.2f}) della cella ({row},{col}) escluso per il "
          f"resto della missione: {why}.")


def find_best_point_in_cell(robot_x, robot_y, env, cell_row, cell_col, pts, cells_obstacle_dist,
                            sampler, global_map=None, safety_margin=None, exclude=None,
                            reachable=None):
    """
    Sceglie il punto obiettivo dentro la cella fra il centro geometrico e i nodi del grafo
    nella cella.

    Scartati sempre (mai scelti): i punti in `exclude` (gia' provati in questo tentativo),
    quelli occupati nella mappa globale (con safety_margin) e quelli a meno di
    geometry.PLAN_CLEARANCE_M (0.28) da un ostacolo nella scansione dal vivo o nello spazio
    libero imparato. Fra i rimasti, nell'ordine:
      1. non rifiutati dal fronte in questa missione (_refuse_goal); i rifiutati tornano
         solo se non c'e' altro;
      2. RAGGIUNGIBILI dal robot nel grafo (reachable(x, y) -> bool, provato nell'ordine
         dei criteri seguenti finche' uno risponde si'). Non basta ripiegare sui successivi
         del piu' vicino al centro quando il grafo non ci arriva: stanno nella stessa zona
         isolata (il 22-10 16:04 quel ripiego e' fallito 9 volte su 10). Se nessun punto e'
         raggiungibile si sceglie comunque il migliore, e il tentativo fallisce;
      3. con almeno TARGET_CLEARANCE_M (0.33) di spazio libero;
      4. il centro geometrico, libero o ignoto che sia (REGOLA DEL CENTRO: un punto gia'
         visto libero di solito e' l'ultimo lembo scansionato dalla cella precedente, cioe'
         sul bordo);
      5. visti liberi prima che mai visti: se il centro e' scartato la zona e' nota, e
         tornare a guardare un ignoto accanto all'ostacolo sarebbe un giro a vuoto;
      6. ad almeno TARGET_CELL_BORDER_M dal bordo della cella;
      7. piu' vicino al centro (a passi di 0.25 m), poi piu' vicino al robot: a parita',
         quello dalla sua parte dell'ostacolo.

    Se non resta nessun punto la cella non ha un obiettivo valido e la funzione
    restituisce (None, None, ...): il chiamante lo tratta gia' come fallimento di
    ingresso e passa alla logica dei lati non ancora provati.

    global_map: global_maps.GlobalGrid accumulata, aggiornata con la scansione corrente appena
    prima di questa chiamata. Se None si usa il centro senza controlli.

    Returns:
        (target_x, target_y, valid_samples, rejected_samples)
        Le due liste alimentano la visualizzazione (gialli = ammessi, rossi = scartati).
    """
    if safety_margin is None:
        safety_margin = geometry.PRM_EDGE_SAFETY_MARGIN_M
    cell_center = env.get_world_position_from_cell(cell_row, cell_col)
    if cell_center is None:
        return None, None, [], []

    cell_center_x, cell_center_y = cell_center
    sampled = list(sampler.get_point_in_cell(cell_row, cell_col))

    if global_map is None:
        # Nessuna informazione di occupazione: il centro della cella.
        return cell_center_x, cell_center_y, sampled, []

    # min_clear e' la distanza con cui si pianificano gli archi (geometry.PLAN_CLEARANCE_M):
    # un obiettivo piu' vicino agli ostacoli non avrebbe archi. La mappa globale dichiara
    # occupata una cella solo dopo due osservazioni, quindi un ostacolo appena visto c'e'
    # solo nella scansione dal vivo (22-10 15:50, cella (0,2)).
    min_clear = geometry.PLAN_CLEARANCE_M
    pref_margin = max(safety_margin, P.TARGET_CLEARANCE_M - geometry.OBSTACLE_THRESHOLD)
    refused = _refused_goals(env, cell_row, cell_col)
    live_od = None
    if pts is not None and cells_obstacle_dist is not None:
        try:
            from scipy.spatial import cKDTree
            p2 = np.asarray(pts, dtype=np.float64).reshape(-1, np.shape(pts)[-1])[:, :2]
            od = np.asarray(cells_obstacle_dist, dtype=np.float64).ravel()
            if len(p2) == len(od) and len(p2):
                live_od = (cKDTree(p2), od)
        except Exception:
            live_od = None

    def _live_clearance(px, py):
        """obstacle_distance dal vivo nel punto, o None se fuori dalla scansione."""
        if live_od is None:
            return None
        d, i = live_od[0].query((px, py))
        return float(live_od[1][i]) if d <= 0.05 else None

    def _near(p, pts_list):
        return bool(pts_list) and any(np.hypot(p[0] - ex, p[1] - ey) < P.RECOVERY_SAME_GOAL_M
                                      for ex, ey in pts_list)

    center = (cell_center_x, cell_center_y)
    candidates = [center] + [p for p in sampled
                             if np.hypot(p[0] - cell_center_x, p[1] - cell_center_y) > 0.01]
    rows, blocked = [], []
    center_reason = None
    for p in candidates:
        px, py = p
        is_center = p is center
        if _near(p, exclude):
            blocked.append(p)
            center_reason = center_reason or ("gia' scartato dal vivo" if is_center else None)
            continue
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            blocked.append(p)
            center_reason = center_reason or ("occupato nella mappa" if is_center else None)
            continue
        c_live = _live_clearance(px, py)
        # Per il centro conta la sola scansione dal vivo: lo spazio libero imparato, che
        # raccoglie anche letture da lontano, cambierebbe il centro piu' del dovuto.
        c_mem = None if is_center else (global_map.clearance(px, py)
                                        if hasattr(global_map, 'clearance') else None)
        known_c = [c for c in (c_live, c_mem) if c is not None]
        if known_c and min(known_c) < min_clear:
            blocked.append(p)
            if is_center:
                center_reason = f"a {min(known_c):.2f} m da un ostacolo nella scansione dal vivo"
            continue
        tight = (global_map.is_occupied(px, py, safety_margin=pref_margin)
                 or (bool(known_c) and min(known_c) < P.TARGET_CLEARANCE_M))
        border = environmentMap.cell_border_distance(env, cell_row, cell_col, px, py)
        rows.append(dict(
            p=(float(px), float(py)), center=is_center, refused=_near(p, refused),
            seen=(c_live is not None or global_map.is_known(px, py)), tight=tight,
            clear=(min(known_c) if known_c else None),
            edge=(border is not None and border < P.TARGET_CELL_BORDER_M),
            dc=float(np.hypot(px - cell_center_x, py - cell_center_y)),
            dr=float(np.hypot(px - robot_x, py - robot_y))))

    valid = [r['p'] for r in rows]
    if not rows:
        print(f"[TARGET] Cella ({cell_row},{cell_col}): centro {center_reason or 'scartato'} e "
              f"nessun punto della cella ad almeno {min_clear:.2f} m dagli ostacoli. "
              f"Nessun obiettivo valido.")
        return None, None, [], blocked

    step = 0.25
    rows.sort(key=lambda r: (r['tight'], not r['center'], not r['seen'], r['edge'],
                             round(r['dc'] / step), r['dr']))
    fresh = [r for r in rows if not r['refused']]
    pool = fresh if fresh else rows
    chosen, reach_note = None, ""
    if reachable is None:
        chosen = pool[0]
    else:
        for r in pool:
            if reachable(*r['p']):
                chosen = r
                break
        if chosen is None:
            chosen = pool[0]
            reach_note = "; NESSUN punto della cella e' raggiungibile dal robot nel grafo"
            MISSION_STATS['goals_unreachable'] = MISSION_STATS.get('goals_unreachable', 0) + 1
    if not fresh:
        reach_note += "; tutti i punti erano gia' stati rifiutati dal fronte, riprovo il migliore"

    bx, by = chosen['p']
    flags = []
    if chosen['tight']:
        flags.append(f"meno di {P.TARGET_CLEARANCE_M:.2f} m di spazio libero")
    if chosen['edge']:
        flags.append(f"a meno di {P.TARGET_CELL_BORDER_M:.2f} m dal bordo della cella")
    flag_txt = f" -- {', '.join(flags)}" if flags else ""
    if chosen['center']:
        state = "libero" if chosen['seen'] else "ignoto"
        print(f"[TARGET] Cella ({cell_row},{cell_col}): uso il centro geometrico "
              f"({bx:.2f}, {by:.2f}), {state}{flag_txt}{reach_note}.")
        return bx, by, valid, blocked

    if center_reason is None:
        crow = next((r for r in rows if r['center']), None)
        if crow is None:
            center_reason = "scartato"
        elif crow['refused']:
            center_reason = "rifiutato dal fronte in questa missione"
        elif crow['tight'] and not chosen['tight']:
            center_reason = (f"con solo {crow['clear']:.2f} m di spazio libero"
                             if crow['clear'] is not None else "troppo vicino a un ostacolo nella mappa")
        else:
            center_reason = "non raggiungibile dal robot nel grafo"
    n_seen = sum(1 for r in rows if r['seen'])
    print(f"[TARGET] Cella ({cell_row},{cell_col}): centro {center_reason}, uso il punto "
          f"{'libero' if chosen['seen'] else 'ignoto'} ({bx:.2f}, {by:.2f}), a {chosen['dc']:.2f} m "
          f"dal centro{flag_txt}{reach_note}  [liberi={n_seen} ignoti={len(rows) - n_seen} "
          f"scartati={len(blocked)} stretti={sum(r['tight'] for r in rows)} "
          f"sul bordo={sum(r['edge'] for r in rows)} rifiutati={sum(r['refused'] for r in rows)}]")
    return bx, by, valid, blocked
