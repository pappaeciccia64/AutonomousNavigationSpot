"""Decisioni sul fronte: il passo da fare (decide_next_move), la rotazione, il ventaglio delle direzioni d'uscita, cono, arretramento, uscite all'arrivo, scorciatoie."""
import numpy as np
from spot_explorer import config as P
from spot_explorer.planning import roadmap as prm_graph
from spot_explorer.perception import geometry
from spot_explorer.planning.safe_frontier import (GridSnapshot, allowed_advance, compute_safe_frontier, point_along)


def decide_next_move(robot_xy, path_waypoints, frontier, no_progress_count, in_target_cell=True):
    """
    Cosa fare adesso, dato il fronte sicuro. Funzione pura: niente robot, niente I/O, quindi
    provabile fuori dal campo.

    path_waypoints: [(node_id, x, y), ...] -- i nodi ancora da raggiungere, il primo e' il
        prossimo. La posizione del robot NON e' inclusa.
    frontier: risultato di arcVerification.compute_safe_frontier sul percorso
        [robot] + waypoint.
    no_progress_count: scansioni consecutive in cui non si e' potuto avanzare.
    in_target_cell: il robot e' gia' dentro la cella obiettivo, o fuori al massimo di
        ARRIVAL_OUTSIDE_CELL_M (lo decide il chiamante). L'arrivo per tolleranza vale solo
        li': fermarsi 0.65 m prima di un obiettivo vicino al bordo puo' voler dire restare
        nella cella precedente. Altrimenti si prosegue come per un fronte bloccato (attesa,
        poi blocco).

    Restituisce un dict con 'kind':
      'reached_node'  il prossimo nodo e' gia' sotto il robot (< 5 cm): consumarlo e basta
      'move'          muoversi verso 'target'; 'full' = True se il target e' il nodo stesso.
                      'final' = True: passo sotto MIN_PARTIAL_STEP_M che porta entro la
                      tolleranza d'arrivo (vedi FINAL_STEP_MIN_M).
                      'nodes' = quanti nodi del percorso stanno dentro l'avanzamento
                      consentito dal fronte: il ciclo li percorre uno dopo l'altro senza
                      riscansionare (vedi AVANZAMENTO MULTINODO). Vale 1 o piu' quando
                      'full', e non c'e' quando il passo e' parziale.
      'arrived'       non si puo' avanzare, ma l'obiettivo finale e' entro tolleranza e il
                      robot e' dentro la cella obiettivo
      'wait'          non si puo' avanzare: riscansionare da fermi prima di concludere
      'blocked'       non si puo' avanzare da FRONTIER_BLOCK_CONFIRM_SCANS scansioni
                      consecutive: blocco confermato da vicino. 'segment' = indice del
                      tratto del percorso in cui si e' fermato il fronte (0 = dal robot al
                      primo nodo).
    """
    rx, ry = robot_xy
    _, nx, ny = path_waypoints[0]
    seg_len = float(np.hypot(nx - rx, ny - ry))
    if seg_len < 0.05:
        return {'kind': 'reached_node'}

    adv = allowed_advance(frontier)
    if adv >= seg_len - 0.02:
        # --- AVANZAMENTO MULTINODO ----------------------------------------------------
        # Il fronte sicuro percorre TUTTO il percorso pianificato, non il primo tratto:
        # restituisce fin dove e' confermato libero adesso, spesso piu' nodi avanti.
        # Si contano TUTTI i nodi che stanno dentro l'avanzamento consentito, e il
        # ciclo li percorre uno dopo l'altro SENZA scansione, refresh ne' ripianificazione
        # in mezzo, invece di fermarsi e riscansionare a ogni nodo. Si cammina sempre sulla
        # polilinea, un tratto per volta: NON si taglia la corda fra due nodi, che
        # passerebbe su terreno non verificato (e' quello che fa shortcut_index, ma solo
        # dopo avere verificato la corda col fronte).
        # Il controllo NON diventa un verdetto ricordato: durante il movimento il thread
        # ricalcola il fronte a ~5 Hz sul percorso rimanente e ferma il robot se si
        # accorcia (vedi _should_abort_move e FRONTIER_ABORT_CONFIRM). Si eliminano le
        # FERMATE, non le verifiche.
        cum, nodes = 0.0, 0
        px, py = rx, ry
        for (_, wx, wy) in path_waypoints:
            cum += float(np.hypot(wx - px, wy - py))
            if cum > adv + 0.02:
                break
            px, py, nodes = wx, wy, nodes + 1
        return {'kind': 'move', 'target': (nx, ny), 'full': True, 'dist': seg_len,
                'nodes': max(1, nodes)}
    # PASSO FINALE (vedi FINAL_STEP_MIN_M): anche sotto il passo minimo, se dopo il passo
    # l'obiettivo e' entro la tolleranza d'arrivo e il robot e' nella cella.
    final = (in_target_cell and adv >= P.FINAL_STEP_MIN_M
             and frontier['path_len'] > P.GOAL_REACHED_TOLERANCE_M
             and frontier['path_len'] - adv <= P.GOAL_REACHED_TOLERANCE_M)
    if adv >= P.MIN_PARTIAL_STEP_M or final:
        f = adv / seg_len
        out = {'kind': 'move', 'target': (rx + f * (nx - rx), ry + f * (ny - ry)),
               'full': False, 'dist': adv}
        if final and adv < P.MIN_PARTIAL_STEP_M:
            out['final'] = True
        return out

    if frontier['path_len'] <= P.GOAL_REACHED_TOLERANCE_M and in_target_cell:
        return {'kind': 'arrived'}
    if no_progress_count + 1 < P.FRONTIER_BLOCK_CONFIRM_SCANS:
        return {'kind': 'wait'}

    polyline = [(rx, ry)] + [(x, y) for _, x, y in path_waypoints]
    _, seg_idx = point_along(polyline, frontier['dist'])
    return {'kind': 'blocked', 'segment': int(seg_idx)}


def rotation_plan(robot_yaw, robot_xy, target_xy, od_robot, snap=None):
    """
    Si puo' ruotare sul posto per guardare il punto da raggiungere?

    Ruotando di un certo angolo il corpo spazza un settore, non tutto il cerchio di raggio
    0.60 attorno al robot: si controllano le celle davvero spazzate. Chiedere libero tutto
    il cerchio (obstacle_distance sotto il robot >= ROTATION_CLEARANCE_M) e' il caso
    peggiore su tutte le direzioni e tutti gli angoli, e rifiuterebbe molte rotazioni sicure.

    Ordine dei controlli, dal piu' economico:
      1. rotazione sotto ROTATION_CHECK_MIN_DYAW_DEG -- trascurabile, si ruota
      2. obstacle_distance sotto il robot >= ROTATION_CLEARANCE_M -- cerchio circoscritto
         libero, si ruota senza altri calcoli (prefiltro)
      3. geometry.rotation_is_clear() sulle celle davvero spazzate (0.26 ms)

    Senza `snap` (nessuna fotografia della griglia) si ricade sul solo confronto scalare.

    Funzione pura, provabile fuori dal campo. Restituisce dict con:
      turn (bool), dyaw_deg, motion_dir (rad), margin (margine del settore spazzato, in
      metri, o None se non calcolato).
    """
    rx, ry = robot_xy
    tx, ty = target_xy
    motion_dir = float(np.arctan2(ty - ry, tx - rx))
    dyaw = float(np.degrees(np.arctan2(np.sin(motion_dir - robot_yaw), np.cos(motion_dir - robot_yaw))))

    if abs(dyaw) < geometry.ROTATION_CHECK_MIN_DYAW_DEG:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'rotazione trascurabile'}
    if od_robot >= geometry.ROTATION_CLEARANCE_M:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'cerchio circoscritto libero'}
    if snap is not None:
        ok, margin = geometry.rotation_is_clear(
            snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
            rx, ry, robot_yaw, float(np.radians(dyaw)))
        return {'turn': bool(ok), 'dyaw_deg': dyaw, 'motion_dir': motion_dir,
                'margin': float(margin) if np.isfinite(margin) else None,
                'why': 'settore spazzato'}
    return {'turn': False, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
            'why': 'nessun dato di griglia'}


def shortcut_index(robot_xy, path_waypoints, snap, global_map=None, prm=None, robot_node_id=None):
    """
    Scorciatoia: l'indice del waypoint PIU' LONTANO raggiungibile in linea retta
    dal robot con il tratto interamente confermato libero dal fronte sicuro (muso compreso).
    0 = nessuna scorciatoia. Toglie le svolte inutili fra nodi del PRM.
    Il controllo e' il fronte stesso sui dati dal vivo: ostacoli, rugosita', pendenza.

    E anche la MAPPA GLOBALE, con lo stesso margine del PRM: alcune cose stanno solo li' --
    il punto di una caduta, per esempio, che ai sensori puo' sembrare terreno normale (e' per
    questo che il robot ci e' caduto). Gli archi del PRM le evitano gia'; una scorciatoia
    controllata solo dal vivo potrebbe riportarci sopra il robot.

    E GLI ARCHI GIA' SCARTATI. Senza questo controllo la scorciatoia riaggancerebbe un
    waypoint il cui collegamento diretto e' appena stato invalidato, e il ciclo non
    uscirebbe piu': il tratto viene scartato, Dijkstra trova una strada che passa comunque
    per quel nodo attraverso i nodi intermedi, e la scorciatoia ci ripunta dritto. Servono
    `prm` e `robot_node_id`; senza, il controllo si salta.
    """
    rx, ry = robot_xy
    for k in range(len(path_waypoints) - 1, 0, -1):
        wid, wx, wy = path_waypoints[k]
        if prm is not None and robot_node_id is not None:
            key = (min(robot_node_id, wid), max(robot_node_id, wid))
            if key in getattr(prm, 'tracker_blocked_edges', ()):
                continue
            # Anche gli scarti soft (mark_edge_blocked_soft: [SUBITO], rotazione rifiutata),
            # altrimenti la scorciatoia ripunta il nodo appena scartato per rotazione.
            if getattr(prm, 'edge_validity', {}).get(key) is False:
                continue
        d = float(np.hypot(wx - rx, wy - ry))
        if d > P.SHORTCUT_MAX_DIST_M:
            continue
        if global_map is not None:
            n = max(prm_graph.OCCUPANCY_SAMPLES_MIN, int(np.ceil(d * prm_graph.OCCUPANCY_SAMPLES_PER_M)) + 1)
            if any(global_map.is_occupied(rx + t * (wx - rx), ry + t * (wy - ry),
                                          safety_margin=geometry.PRM_EDGE_SAFETY_MARGIN_M)
                   for t in np.linspace(0.0, 1.0, n)[1:]):
                continue
        f = compute_safe_frontier([(rx, ry), (wx, wy)], snap)
        if f['reason'] == 'fine_percorso':
            return k
    return 0


def plan_straight_retreat(robot_xy, robot_yaw, snap, max_dist=P.ROTATION_RETREAT_M):
    """
    Quanto puo' arretrare il robot IN LINEA RETTA, all'indietro, senza girarsi.

    Non e' marcia obliqua: muovendosi lungo il proprio asse l'ingombro resta 0.50 m di
    larghezza e il fronte applica la soglia normale FRONTIER_CLEARANCE_M, non i 0.59 della
    marcia di traverso. E' la via d'uscita quando la rotazione verso il prossimo punto non e'
    possibile: ci si stacca da cio' che impedisce di girarsi e al giro dopo si riprova da
    una posizione diversa.

    Funzione pura. Restituisce (x, y, distanza) del punto raggiungibile, oppure None se non
    si puo' arretrare di almeno ROTATION_RETREAT_MIN_M.
    """
    rx, ry = robot_xy
    back_x = rx - max_dist * np.cos(robot_yaw)
    back_y = ry - max_dist * np.sin(robot_yaw)
    body = geometry.body_extents(robot_yaw, robot_yaw + np.pi)   # all'indietro: (0.55, 0.25)
    f = compute_safe_frontier([(rx, ry), (back_x, back_y)], snap, body=body)
    adv = allowed_advance(f)
    if adv < P.ROTATION_RETREAT_MIN_M:
        return None
    adv = min(adv, max_dist)
    return (float(rx - adv * np.cos(robot_yaw)), float(ry - adv * np.sin(robot_yaw)), float(adv))


def scan_escape_directions(robot_xy, robot_yaw, snap, step_deg=P.ESCAPE_FAN_STEP_DEG,
                           min_advance_m=P.ESCAPE_FAN_MIN_ADVANCE_M, probe_m=3.0):
    """
    Il giro completo: in quali direzioni il robot puo' DAVVERO andare da qui?

    Per ciascuna direzione del giro (passo step_deg) misura due cose indipendenti:
      - quanto spazio concede il fronte sicuro andando in linea retta in quella direzione;
      - se il robot puo' girarsi tanto da puntarci il muso (geometry.rotation_is_clear).
    Una direzione e' UTILIZZABILE solo se entrambe rispondono si'. Il robot non cammina di
    traverso (vedi ROTAZIONE E ARRETRAMENTO in parametri.py), quindi una corsia larga che richiede una
    rotazione impossibile NON e' una via d'uscita, per quanto larga sia.

    Serve a riconoscere una trappola (zero direzioni utilizzabili: robot in un angolo, con
    le corsie aperte dietro rotazioni che li' non si fanno) con 24 valutazioni del fronte,
    invece di scoprirlo scartando gli archi uno per uno, una scansione dopo l'altra.

    Funzione pura. Restituisce la lista delle direzioni esaminate, ciascuna con:
    dir_deg (assoluta), rel_deg (rotazione necessaria), advance, reason, rotatable, usable.
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    out = []
    for deg in np.arange(-180.0, 180.0, abs(step_deg)):
        a = float(np.radians(deg))
        f = compute_safe_frontier(
            [(rx, ry), (rx + probe_m * np.cos(a), ry + probe_m * np.sin(a))], snap)
        adv = allowed_advance(f)
        rel = float(np.degrees(np.arctan2(np.sin(a - robot_yaw), np.cos(a - robot_yaw))))
        if abs(rel) < geometry.ROTATION_CHECK_MIN_DYAW_DEG:
            rotatable = True
        else:
            rotatable = bool(geometry.rotation_is_clear(
                snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
                rx, ry, float(robot_yaw), float(np.radians(rel)))[0])
        out.append({'dir_deg': float(deg), 'rel_deg': rel, 'advance': float(adv),
                    'reason': f['reason'], 'rotatable': rotatable,
                    'usable': bool(rotatable and adv >= min_advance_m)})
    return out


def describe_escape_fan(dirs, max_rows=4):
    """Riga di log compatta: quante direzioni utilizzabili e le migliori, per i log."""
    usable = [d for d in dirs if d['usable']]
    best = sorted(dirs, key=lambda d: -d['advance'])[:max_rows]
    txt = "; ".join(f"{d['dir_deg']:+.0f} gradi: {d['advance']:.2f} m "
                    f"({'ruotabile' if d['rotatable'] else 'NON ruotabile'})" for d in best)
    return len(usable), txt


def is_trapped(robot_xy, robot_yaw, snap):
    """
    True se NESSUNA direzione del giro e' insieme percorribile e raggiungibile
    girandosi: la condizione che l'utente ha posto per tornare indietro ("solo se non puo'
    piu' andare avanti ne' ruotare"), verificata in una scansione invece che in venti.
    Restituisce (trapped, dirs) -- dirs serve per il log, cosi' nei dati si vede perche'.
    """
    dirs = scan_escape_directions(robot_xy, robot_yaw, snap)
    return (not any(d['usable'] for d in dirs)), dirs


def arrival_has_exit(snap, arrival_xy, heading, min_advance_m=P.ESCAPE_FAN_MIN_ADVANCE_M):
    """
    Dal punto in cui sto per fermarmi, ci sara' un'uscita?

    Il fronte sicuro guarda la strada DAVANTI, non se dalla posa d'arrivo si potra'
    ripartire: con un passo corto il robot puo' entrare in un angolo senza uscite.

    Si valuta il ventaglio nel punto d'arrivo usando la fotografia CORRENTE: e' una stima
    (da li' il robot vedra' di piu'), ma e' la stessa informazione su cui si sta decidendo
    di andarci, quindi la decisione e' almeno coerente con se stessa. Per non pagarla su
    ogni movimento la chiama solo il ciclo principale, e solo per i passi corti o verso
    spazi stretti -- vedi ARRIVAL_EXIT_CHECK_MAX_STEP_M / _OD_M.

    Restituisce (ha_uscita, dirs).
    """
    ax, ay = float(arrival_xy[0]), float(arrival_xy[1])
    # Stessa griglia, ma con il robot dichiarato nel punto d'arrivo: e' cio' che cambia il
    # riferimento della regola di uscita del fronte e il centro delle rotazioni provate.
    probe = GridSnapshot(
        snap.obstacle_dist, snap.rough_veto, snap.seen, snap.terrain,
        snap.origin_x, snap.origin_y, snap.cell_size, ax, ay,
        timestamp=snap.time, valid=snap.valid, blocked_latch=snap.blocked_latch)
    dirs = scan_escape_directions((ax, ay), float(heading), probe, min_advance_m=min_advance_m)
    # Una direzione girabile che si ferma perche' FINISCONO I DATI conta come uscita: dal
    # punto d'arrivo il robot vedra' di piu', e "non visto" non vuol dire "chiuso". Senza,
    # in un corridoio la direzione avanti finiva sul bordo della scansione (0.09 m) e il
    # passo veniva rifiutato; al terzo rifiuto si andava lo stesso, ed era il passo giusto
    # (28-10 12:16, e lo scenario S16 di test_e2e_loop.py).
    return any(d['usable'] or (d['rotatable'] and d['reason'] == 'fine_dati') for d in dirs), dirs


def plan_cone_move(robot_xy, robot_yaw, snap, goal_xy, max_dyaw_deg=P.CONE_SWEEP_MAX_DEG,
                   step_deg=P.CONE_SWEEP_STEP_DEG, min_gain_m=P.CONE_MIN_GAIN_M):
    """
    "Ruota quanto PUOI, non quanto vorresti, e cammina."

    Quando la rotazione verso il prossimo nodo e' rifiutata, la domanda giusta non e' piu'
    "posso girarmi verso il nodo?" ma "qual e' la rotazione piu' grande che posso fare qui,
    e in quale direzione dentro quel cono conviene andare?". In un corridoio stretto girarsi
    richiede 1.21 m di diametro e non si puo', ma camminare si': spesso andare dritti, o
    quasi, avvicina all'obiettivo piu' di qualunque arretramento.

    Criterio di scelta: fra le direzioni che AVVICINANO all'obiettivo
    di almeno min_gain_m, si prende quella con piu' spazio confermato libero. Non la piu'
    libera in assoluto: quella porterebbe il robot via dalla cella.

    Funzione pura. Restituisce None se nessuna direzione del cono avvicina, altrimenti
    dict con: dyaw_deg (rotazione da fare, gradi, puo' essere 0), target (x, y), dist
    (avanzamento), gain (metri guadagnati verso l'obiettivo), reason (motivo del fronte).
    """
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    d0 = float(np.hypot(gx - rx, gy - ry))

    best = None
    rels = np.arange(-abs(max_dyaw_deg), abs(max_dyaw_deg) + 1e-9, abs(step_deg))
    for rel in rels:
        # La rotazione deve essere possibile: sotto la soglia di trascurabilita' non si
        # controlla nulla (e' quello che fa anche rotation_plan), sopra si misura il settore.
        if abs(rel) >= geometry.ROTATION_CHECK_MIN_DYAW_DEG:
            ok, _ = geometry.rotation_is_clear(
                snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
                rx, ry, robot_yaw, float(np.radians(rel)))
            if not ok:
                continue

        heading = robot_yaw + float(np.radians(rel))
        far_x, far_y = rx + 3.0 * np.cos(heading), ry + 3.0 * np.sin(heading)
        f = compute_safe_frontier([(rx, ry), (far_x, far_y)], snap)
        adv = allowed_advance(f)
        if adv < P.MIN_PARTIAL_STEP_M:
            continue

        nx, ny = rx + adv * np.cos(heading), ry + adv * np.sin(heading)
        gain = d0 - float(np.hypot(gx - nx, gy - ny))
        if gain < min_gain_m:
            continue
        cand = {'dyaw_deg': float(rel), 'target': (float(nx), float(ny)), 'dist': float(adv),
                'gain': float(gain), 'reason': f['reason']}
        # Piu' spazio libero vince; a pari spazio, la rotazione piu' piccola.
        if best is None or (cand['dist'], -abs(cand['dyaw_deg'])) > (best['dist'], -abs(best['dyaw_deg'])):
            best = cand
    return best


def retreat_would_help(snap, back_xy, robot_yaw, dyaw_deg):
    """
    L'arretramento di 0.60 m renderebbe possibile la rotazione che serve?

    In un corridoio la geometria 0.60 m piu' indietro e' la stessa: arretrare non apre
    spazio, lo sposta. Si arretra solo se da li' la rotazione diventa possibile.

    Si valuta la rotazione nel punto di ARRIVO usando la fotografia corrente -- e' una stima,
    perche' da li' il robot vedra' qualcosa di piu', ma e' la stessa informazione su cui si
    e' deciso di arretrare, quindi almeno la decisione e' coerente con se stessa.
    """
    if abs(dyaw_deg) < geometry.ROTATION_CHECK_MIN_DYAW_DEG:
        return True, None
    ok, margin = geometry.rotation_is_clear(
        snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
        float(back_xy[0]), float(back_xy[1]), float(robot_yaw), float(np.radians(dyaw_deg)))
    return bool(ok), margin
