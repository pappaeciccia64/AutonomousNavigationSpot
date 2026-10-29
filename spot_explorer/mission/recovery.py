"""Uscire da un blocco: ritorno indietro con GraphNav, ritirata a zampe, celle bloccate o rimandate."""
import numpy as np

from spot_explorer.graphnav import graphnav as navGraphUtils
from spot_explorer.robot import motion as movements
from spot_explorer.perception import grids as spotGrid
from spot_explorer.robot import connection as spotUtils
from spot_explorer.planning import safe_frontier as arcVerification

from spot_explorer.planning import roadmap as prm_graph

from spot_explorer import config as P
from spot_explorer.records import MISSION_STATS, TrajectoryLog



# =========================================================================
# RITORNO INDIETRO CON GRAPHNAV, quando dal punto in cui si e' il grafo non porta
# alla cella
# =========================================================================


def backtracking_cannot_help(env, prm_graph, goal_id, robot_state_client, max_hops,
                             min_dist_m=0.40):
    """
    Tornare indietro con GraphNav puo' servire a qualcosa?

    escape_by_backtracking torna a una briciola e SOLO DOPO chiede al grafo un percorso da
    li'. Ma ogni briciola ha gia' il suo nodo PRM: la stessa domanda si puo' fare prima di
    muoversi. Se l'obiettivo sta in un'altra componente del grafo, nessun waypoint dietro
    puo' cambiare la risposta, e i ritorni sarebbero solo strada sprecata.

    Risponde True solo se OGNI briciola che verrebbe visitata ha un nodo PRM collegato nel
    grafo e da nessuno di essi c'e' un percorso. Se anche una sola non si puo' giudicare
    (nodo mancante o isolato, es. scartato dentro un ostacolo) risponde False e si torna
    indietro: nel dubbio non si toglie la via d'uscita.
    """
    trail = getattr(env, '_graphnav_trail', None) or []
    if not trail or goal_id not in prm_graph.nodes:
        return False
    try:
        rx, ry, _, _ = spotUtils.getPosition(robot_state_client)
    except Exception:
        return False
    candidates = [c for c in reversed(trail)
                  if float(np.hypot(c['x'] - rx, c['y'] - ry)) >= min_dist_m][:int(max_hops)]
    if not candidates:
        return False
    for crumb in candidates:
        nid = crumb.get('prm_node')
        if nid is None or nid not in prm_graph.nodes or not prm_graph.edges.get(nid):
            return False
        if prm_graph.find_path_dijkstra(nid, goal_id) is not None:
            return False
    return True


def _trap_backtrack(trap_state, block_arc, env, prm_graph, goal_id, recordingInterface,
                    robot_state_client, command_client, global_map, mobility_kwargs, pts,
                    obstacle_mask, label="", **kwargs):
    """
    Ritorno indietro (escape_by_backtracking) con MEMORIA per il tentativo.

    trap_state: dict del tentativo ({'n': ritorni gia' fatti}), che non si azzera mai dentro
    il tentativo. block_arc: (a, b), l'arco su cui il robot si e' fermato.

    Prima di ripianificare, l'arco diventa impercorribile per il resto del tentativo
    (penalita' del tentativo: sopravvive a refresh e ricostruzioni e si toglie solo
    all'inizio del tentativo successivo, vedi prm_graph.clear_penalties_for_attempt). Da li'
    il pianificatore deve proporre un'altra strada, o ammettere che non c'e'. Al
    MAX_BACKTRACKS_PER_ATTEMPT-esimo ritorno il robot torna comunque indietro -- la' dov'e'
    non puo' restare -- ma il tentativo finisce: la cella si rimanda e si prova da un altro
    lato. Senza questa memoria il robot rifarebbe lo stesso giro all'infinito.
    """
    trap_state['n'] = trap_state.get('n', 0) + 1
    if block_arc is not None and block_arc[0] is not None and block_arc[1] is not None:
        a, b = int(block_arc[0]), int(block_arc[1])
        if a != b:
            key = (min(a, b), max(a, b))
            prm_graph.edge_penalty_persistent[key] = prm_graph._ATTEMPT_MARK
            print(f"[RITORNO] Arco {a}->{b} fuori dal piano per il resto del tentativo "
                  f"(ritorno {trap_state['n']}/{P.MAX_BACKTRACKS_PER_ATTEMPT}).")
    esc = escape_by_backtracking(
        env, prm_graph, goal_id, recordingInterface, robot_state_client, command_client,
        global_map, mobility_kwargs, pts, obstacle_mask, label=label, **kwargs)
    if trap_state['n'] >= P.MAX_BACKTRACKS_PER_ATTEMPT and esc.get('ok'):
        MISSION_STATS['backtrack_giveups'] = MISSION_STATS.get('backtrack_giveups', 0) + 1
        print(f"[RITORNO] {trap_state['n']} ritorni in questo tentativo: sono tornato "
              f"indietro, ma non riprovo. Rimando la cella e la provo da un altro lato.")
        esc = dict(esc, ok=False, how='rinuncia dopo ritorni ripetuti')
    return esc


def escape_by_backtracking(env, prm_graph, goal_id, recordingInterface, robot_state_client,
                           command_client, global_map, mobility_kwargs, pts, obstacle_mask,
                           max_hops=P.GRAPHNAV_MAX_BACKTRACK_HOPS, label="",
                           skip_if_graph_cannot_help=False):
    """
    La via d'uscita quando da qui non si va da nessuna parte.

    Si torna indietro di un waypoint con GraphNav e si prova a ripianificare verso
    l'obiettivo da li'; se non si puo', si torna indietro di un altro, e cosi' via fino a
    max_hops. Dopo max_hops la cella si rimanda e si passa alla priorita' successiva.

    Se GraphNav non ce la fa (traccia vuota, non localizzato, STUCK, tempo scaduto) si ricade
    sulla ritirata a zampe: e' peggiore, ma e' l'ultima rete e non la togliamo. In una
    trappola dove ruotare e' impossibile GraphNav, che per percorrere un arco vuole girare il
    muso, puo' tornare STUCK proprio dove serve.

    Restituisce dict: ok (si puo' continuare la missione), ids (nuovo percorso o None),
    robot_node (nuovo nodo del robot o None), how (cosa e' successo, per il log).
    """
    # Solo quando il robot NON e' intrappolato (chiamanti "nessuna alternativa nel grafo"):
    # in una trappola o dopo un fallimento fisico tornare indietro serve anche a liberarlo.
    if skip_if_graph_cannot_help and backtracking_cannot_help(
            env, prm_graph, goal_id, robot_state_client, max_hops):
        print(f"[RITORNO] {label}: dai nodi delle prossime {max_hops} briciole il grafo non "
              f"arriva comunque all'obiettivo (non dipende da dove sono io). Non torno "
              f"indietro: rimando subito la cella.")
        MISSION_STATS['backtracks_skipped'] = MISSION_STATS.get('backtracks_skipped', 0) + 1
        return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}

    for hop in range(1, int(max_hops) + 1):
        # Si arriva girati verso l'obiettivo del tentativo: da li' si ripianifica verso di lui.
        ok, crumb = navGraphUtils.graphnav_step_back(env, recordingInterface, robot_state_client, command_client,
                                                     face_xy=prm_graph.get_node_position(goal_id))
        if not ok:
            if crumb is None:
                print(f"[RITORNO] {label}: nessuna briciola GraphNav disponibile.")
            else:
                print(f"[RITORNO] {label}: GraphNav non e' riuscito a tornare a "
                      f"{crumb['wp_name']}.")
            MISSION_STATS['graphnav_backtrack_fail'] += 1
            print("[RITORNO] Ripiego sulla ritirata a zampe sui tratti percorsi.")
            retreat_along_traveled_path(env, robot_state_client, command_client,
                                        mobility_kwargs=mobility_kwargs,
                                        pts=pts, obstacle_mask=obstacle_mask)
            return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'ritirata a zampe'}

        if not crumb.get('in_place'):
            MISSION_STATS['graphnav_backtracks'] += 1
        prm_graph.needs_refresh = True       # il robot si e' spostato: vedi REFRESH A EVENTI
        # La scansione dello scarto preliminare e' stata fatta da un altro punto (e
        # l'esclusione del corpo starebbe dove il robot era): via, fino alla prossima scansione.
        prm_graph.set_live_obstacles(None, None, 0, 0, 1, 0, 0)
        try:
            ax, ay, _, _ = spotUtils.getPosition(robot_state_client)
        except Exception:
            ax, ay = crumb['x'], crumb['y']
        new_node = prm_graph.add_stop_node(
            ax, ay, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
            label=f"sosta (ritorno GraphNav a {crumb['wp_name']})")
        # Niente deroga alla mappa globale attorno a questo nodo: vedi il commento al
        # chiamante di set_robot_node nel ciclo principale.
        prm_graph.set_robot_node(new_node, 0.0)

        ids = prm_graph.find_path_dijkstra(new_node, goal_id)
        if ids is not None and len(ids) > 1:
            print(f"[RITORNO] {label}: tornato a {crumb['wp_name']} ({hop}/{max_hops}) e da qui "
                  f"c'e' un percorso con {len(ids) - 1} tratti. Riprendo.")
            return {'ok': True, 'ids': ids, 'robot_node': new_node, 'how': 'ripianificato'}
        print(f"[RITORNO] {label}: tornato a {crumb['wp_name']} ({hop}/{max_hops}) ma da qui il "
              f"grafo non offre percorsi verso l'obiettivo.")
        if crumb.get('in_place'):
            break           # non ci sono briciole piu' lontane: altri giri sarebbero uguali

    print(f"[RITORNO] {label}: {hop} ritorni senza trovare un percorso. Rimando la cella.")
    return {'ok': False, 'ids': None, 'robot_node': None, 'how': 'cella rimandata'}



# =========================================================================
# RITIRATA A ZAMPE all'indietro: l'ultima rete
# =========================================================================


def retreat_along_traveled_path(env, robot_state_client, command_client,
                                max_segments=P.RETREAT_MAX_SEGMENTS,
                                mobility_kwargs=None,
                                pts=None, obstacle_mask=None):
    """
    Ritirata: torna indietro sui propri passi camminando ALL'INDIETRO, senza mai girarsi.
    E' l'ultima rete, quando il ritorno con GraphNav non riesce (vedi escape_by_backtracking).

    Quando il robot si infila in uno spazio stretto e li' non esiste piu' un percorso
    verso l'obiettivo, girarsi non e' una via d'uscita --
    serve spazio che in quel punto non c'e'. Arretrare invece ripercorre il varco da cui
    si e' entrati, che per costruzione era abbastanza largo da passarci. E arretrare lungo
    l'asse del corpo non e' marcia di traverso: l'ingombro resta 0.50 m di larghezza.

    I punti da cui si e' passati sono gia' registrati in env._traveled_arcs, riempito a
    ogni spostamento completato. Si ripercorrono a ritroso, uno per uno, mantenendo
    l'orientamento attuale (vedi movements.move_to_world_point_without_turning).

    Se vengono passati `pts` e `obstacle_mask` della scansione corrente, ogni tratto
    viene prima controllato sui dati freschi: il fatto di esserci passati poco fa e' un
    ottimo indizio, non una garanzia, e non ha senso arretrare dentro qualcosa che nel
    frattempo si vede occupato. In quel caso la ritirata si ferma li' e lo dichiara.

    Returns:
        (segmenti_percorsi: int, distanza_totale: float)
    """
    mobility_kwargs = mobility_kwargs or {}
    traveled = getattr(env, '_traveled_arcs', None)
    if not traveled:
        print("[RITIRATA] Nessun tratto percorso registrato: non so da dove sono arrivato, "
              "non arretro.")
        return 0, 0.0

    done, total = 0, 0.0
    for _ in range(max_segments):
        if not traveled:
            print("[RITIRATA] Esauriti i tratti percorsi registrati.")
            break

        from_x, from_y, to_x, to_y = traveled[-1]
        robot_x, robot_y, _, _ = spotUtils.getPosition(robot_state_client)
        seg_len = float(np.hypot(from_x - robot_x, from_y - robot_y))

        gap = float(np.hypot(to_x - robot_x, to_y - robot_y))
        if gap > P.RETREAT_CONTINUITY_M:
            print(f"[RITIRATA] Non sono dove finiva l'ultimo tratto registrato (distanza {gap:.2f} m, "
                  f"es. dopo uno spostamento con GraphNav): non arretro alla cieca.")
            del traveled[:]
            break
        if seg_len > P.RETREAT_MAX_SEGMENT_M:
            print(f"[RITIRATA] Il tratto di rientro e' lungo {seg_len:.2f} m (oltre "
                  f"{P.RETREAT_MAX_SEGMENT_M:.1f} m): non lo ripercorro all'indietro.")
            break

        if seg_len < P.RETREAT_MIN_SEGMENT_M:
            # Tratto troppo corto per valere un comando a se': lo consumiamo e proseguiamo.
            traveled.pop()
            continue

        if pts is not None and obstacle_mask is not None:
            try:
                if arcVerification.is_arc_in_fov(robot_x, robot_y, from_x, from_y, pts):
                    status = arcVerification.verify_arc_safety(robot_x, robot_y, from_x, from_y,
                                                               pts, obstacle_mask)
                    if status == 'blocked':
                        print(f"[RITIRATA] Il tratto di rientro verso ({from_x:.2f}, {from_y:.2f}) "
                              f"risulta ostruito nella scansione corrente. Mi fermo qui.")
                        break
            except Exception as e:
                print(f"[RITIRATA] Controllo del tratto di rientro non riuscito ({e}); "
                      f"procedo comunque, e' la via da cui sono venuto.")

        print(f"[RITIRATA] Tratto {done + 1}/{max_segments}: rientro verso "
              f"({from_x:.2f}, {from_y:.2f}), {seg_len:.2f} m all'indietro.")
        with TrajectoryLog(robot_state_client, 'ritirata', (from_x, from_y)):
            success, dist = movements.move_to_world_point_without_turning(
                from_x, from_y, "vision", command_client, robot_state_client, **mobility_kwargs)
        total += dist
        traveled.pop()

        if not success:
            print(f"[RITIRATA] Il rientro si e' fermato dopo {dist:.2f} m. "
                  f"Interrompo la ritirata.")
            break
        done += 1

    if done:
        print(f"[RITIRATA] Completata: {done} tratti, {total:.2f} m percorsi all'indietro.")
    else:
        print("[RITIRATA] Nessun tratto percorso.")
    return done, total


# =========================================================================
# CELLE BLOCCATE O RIMANDATE
# =========================================================================


def cell_has_reachable_target(env, sampler, global_map, row, col, safety_margin=None):
    """
    Nella cella c'e' ANCORA un punto dove il robot potrebbe stare, secondo la
    mappa globale accumulata finora? Restituisce (si_no, liberi, ignoti, occupati).

    Serve a non ritentare una cella il cui obiettivo e' fisicamente dentro un ostacolo
    (per esempio quando la griglia di missione non sta nello spazio):
    finalize_or_defer_blocked_cell marca bloccata una cella solo dopo averne provati tutti
    i lati, ma se l'obiettivo e' in un muro i lati non c'entrano niente.

    Si guarda solo cio' che e' stato VISTO: ignoto non e' occupato (e' la stessa regola del
    fronte, che distingue 'fine_dati' da 'ostacolo'). Quindi una cella mai vista risponde
    sempre si', e questo controllo non puo' impedire una prima esplorazione.
    """
    if global_map is None:
        return True, 0, 0, 0
    if safety_margin is None:
        safety_margin = spotGrid.PRM_EDGE_SAFETY_MARGIN_M
    centro = env.get_world_position_from_cell(row, col)
    if centro is None:
        return False, 0, 0, 0
    punti = [centro]
    try:
        punti += list(sampler.get_point_in_cell(row, col))
    except Exception:
        pass
    liberi = ignoti = occupati = 0
    for (px, py) in punti:
        if global_map.is_occupied(px, py, safety_margin=safety_margin):
            occupati += 1
        elif global_map.is_known(px, py):
            liberi += 1
        else:
            ignoti += 1
    return (liberi + ignoti) > 0, liberi, ignoti, occupati


def finalize_or_defer_blocked_cell(env, row, col, sampler=None, global_map=None):
    """
    Da chiamare subito dopo env.mark_cell_side_explored(row, col, side_bit), quando un
    tentativo di entrare in (row, col) e' appena fallito da un lato.

    La cella si marca bloccata per sempre (env.mark_cell_blocked -> valore -1) solo quando
    OGNI lato che ha un vicino dentro la griglia e' stato provato ed e' fallito
    (env.all_sides_explored). Altrimenti resta al valore di non visitata (0) e NON si
    ritenta subito: resta una candidata normale della frontiera.
    get_adjacent_frontier_cells e get_lowest_rank_from_frontier_list trattano ogni cella a
    0 allo stesso modo, ordinata per la sua posizione nella serpentina, come una cella mai
    tentata. Cosi' una cella fallita una volta compete per la priorita' con la STESSA
    logica di tutte le altre, e si ritenta quando tocca a lei, dal lato non ancora provato:
    ne' subito fuori turno, ne' scartata dopo un solo lato.
    """
    # Se nella cella non esiste piu' un punto dove il robot possa stare, provare da un
    # altro lato e' inutile -- l'obiettivo e' dentro un ostacolo, non dietro un varco
    # che si apre da un'altra parte. Vedi cell_has_reachable_target.
    if sampler is not None and global_map is not None:
        ok, liberi, ignoti, occupati = cell_has_reachable_target(env, sampler, global_map, row, col)
        # Diagnostica: dice subito se la cella sta dentro un ostacolo o non e' mai stata
        # vista, cioe' se la griglia di missione non sta nello spazio.
        # ATTENZIONE a come si legge: i punti candidati per cella sono decine (reticolo a
        # 0.5 m) o centinaia (a 0.25 m), quindi "occupati" alto NON vuol dire cella irraggiungibile; vuol
        # dire che gran parte della cella e' dentro un ostacolo. Il blocco immediato sotto
        # scatta solo nel caso estremo in cui NON resta un solo punto libero o ignoto.
        print(f"[CELLA] ({row},{col}) dopo il tentativo fallito: dei punti candidati, "
              f"{liberi} visti liberi, {ignoti} mai visti, {occupati} dentro un ostacolo "
              f"nella mappa globale.")
        if not ok:
            env.mark_cell_blocked(row, col)
            print(f"[BLOCKED] Cella ({row},{col}) bloccata subito: tutti i {occupati} punti "
                  f"candidati risultano dentro un ostacolo nella mappa globale (nessuno libero, "
                  f"nessuno ignoto). Non c'e' un lato da cui entrare: l'obiettivo e' nel muro.")
            return
    if env.all_sides_explored(row, col):
        env.mark_cell_blocked(row, col)
        print(f"[BLOCKED] Cella ({row},{col}) bloccata per sempre: ogni lato raggiungibile "
              f"e' stato provato senza successo.")
    else:
        print(f"[DEFER] Cella ({row},{col}) non ancora provata da tutti i lati (lati provati: "
              f"{bin(env.get_cell_sides_status(row, col))}): resta candidata della "
              f"frontiera e si ritentera' piu' avanti, nell'ordine normale, "
              f"da un lato non ancora provato.")
