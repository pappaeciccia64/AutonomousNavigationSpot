"""
PRM (Probabilistic Road Map) della missione: nodi del reticolo (mission_graph), archi validi
con il loro costo, ricerca del percorso (Dijkstra).

Un arco e' valido se non passa vicino a ostacoli (scansione dal vivo e mappa globale) e non e'
troppo ripido; il costo e' 1.0 * lunghezza + 2.0 * pendenza longitudinale + 2.0 * pendenza
laterale (+ 0 * rugosita'). Gli archi si rivalutano a inizio cella (mission_graph,
add_edges_to_prm) e durante il movimento attorno al robot (refresh_local_edge_weights).
"""

import numpy as np
from typing import List, Tuple, Optional
import heapq
import threading
from spot_explorer.perception import grids as spotGrid
try:
    from scipy.spatial import cKDTree
except Exception:          # scipy c'e' gia' (spotGrid usa scipy.ndimage); ripiego per sicurezza
    cKDTree = None


# Campioni per metro con cui si controlla l'occupazione lungo un arco (minimo 5), uguali
# ovunque. Ogni campione controlla un disco di raggio spotGrid.PRM_EDGE_SAFETY_MARGIN_M
# (0.13): con un campione ogni 10 cm, a meta' fra due campioni il disco copre ancora 0.12 m,
# cioe' la linea dell'arco resta a ~0.27 m dagli ostacoli (spotGrid.PLAN_CLEARANCE_M = 0.28;
# il fronte ne pretende 0.25).
OCCUPANCY_SAMPLES_PER_M = 10
OCCUPANCY_SAMPLES_MIN = 5

# --- SCARTO PRELIMINARE SULLA SCANSIONE DAL VIVO ----------------------------------------
# La mappa globale dichiara occupata una cella solo dopo 2 conferme, quindi alla prima
# scansione di un posto non sa ancora niente. Prima di tutto il resto, un arco si scarta se
# un suo campione (ogni 10 cm) cade su una cella VISTA nella scansione attuale con
# obstacle_distance sotto LIVE_PREFILTER_OD_M e non sotto il corpo del robot.
#   - La soglia e' la distanza del fronte (spotGrid.FRONTIER_CLEARANCE_M, 0.25 m): toglie
#     gli archi che il fronte rifiuterebbe. Con una soglia sotto il fronte (0.20 m) il
#     pianificatore sceglieva archi a 0.20-0.25 m che il fronte poi rifiutava.
#   - NON la distanza di pianificazione (0.28, quella della mappa globale): da un punto
#     stretto gli archi d'uscita passano per forza vicino agli ostacoli, e il fronte li
#     ammette (nel primo metro basta non avvicinarsi, vedi FRONTIER_ESCAPE_DIST_M). A 0.28
#     il rigioco della missione 13-10 11:22 perdeva il percorso dall'ultimo inizio di cella.
#   - E' una lettura vettoriale, costa pochissimo, e viene prima della pendenza.
#   - Non e' permanente: si rifa' a ogni refresh con la scansione nuova.
# None = disattivato.
LIVE_PREFILTER_OD_M = spotGrid.FRONTIER_CLEARANCE_M     # = 0.25

# --- Aggancio della posizione del robot al grafo -------------------------------------------
# Il grafo non cresce durante la missione: add_stop_node aggancia la posizione al nodo
# ESISTENTE adatto (vedi anchor_node).
#   ANCHOR_SEARCH_M: raggio in cui cercare il nodo (lascia spazio per scartare quelli dietro
#     un ostacolo).
#   ANCHOR_ON_NODE_M: entro questa distanza il robot e' "sul" nodo: solo allora un blocco o
#     un fallimento sull'arco che ne parte riguarda davvero quell'arco del grafo.
#   ANCHOR_IGNORE_NEAR_M: attorno alla posizione del robot la mappa globale non conta per
#     l'aggancio (li' c'e' il robot); mezza larghezza piu' margine.
ANCHOR_SEARCH_M = 1.0
ANCHOR_ON_NODE_M = 0.15
ANCHOR_IGNORE_NEAR_M = 0.30

# Attorno al nodo in cui il robot si trova ADESSO, la mappa globale degli ostacoli non si usa
# per i primi STOP_NODE_MAP_IGNORE_M di ogni arco: li' giudica il fronte sicuro sui dati dal
# vivo. Serve per esempio a un robot rialzato dopo una caduta, che sta DENTRO il disco
# pericoloso (0.50 m) segnato attorno al punto: senza, ogni arco in uscita verrebbe scartato.
# Vale SOLO per il nodo corrente (set_robot_node).
STOP_NODE_MAP_IGNORE_M = 0.70

# Raggio entro cui refresh_local_edge_weights rivaluta gli archi durante il movimento. Il
# robot nel passo successivo percorre pochi archi vicini; fuori da qui gli archi tengono i
# pesi dell'ultima valutazione (la sicurezza resta al fronte, sui dati dal vivo). 1.25 m e'
# la lunghezza massima degli archi: l'arco che il robot sta per percorrere ricade sempre per
# intero nella zona rivalutata. None = nessun limite.
REFRESH_RADIUS_M = 1.25

# Pendenze delle coppie del refresh calcolate in blocco con le matrici (vedi
# precompute_arc_slopes), come a inizio cella (mission_graph.BATCH_SLOPES).
# False = arco per arco; il risultato e' identico.
REFRESH_BATCH_SLOPES = True


def body_slope(prof):
    """
    (long, lat, has) da usare per veto, costo e andatura, dato il risultato di
    compute_arc_slope_profile: con 6 valori (body=True) quelli col corpo (il peggiore fra arco
    e arco allungato di mezzo corpo, vedi spotGrid._body_slope); con 3 valori quelli
    dell'arco. Se l'arco non ha dati suoi restano quelli dell'arco: l'allungamento non
    trasforma un arco mai visto in uno misurato.
    """
    if prof[2] and len(prof) == 6:
        return prof[3], prof[4], True
    return prof[0], prof[1], prof[2]


# AGGIORNAMENTO LOCALE DEL GRAFO (vedi mission_graph.MissionGraph.add_edges_to_prm). A inizio
# cella si rivalutano solo gli archi vicini alle zone con dati nuovi dall'ultima valutazione
# (dirty_rects: le finestre delle scansioni, l'unico posto in cui cambiano le mappe).
# DIRTY_MARGIN_M e' fin dove la valutazione di un arco legge i dati oltre il suo rettangolo:
# mezzo corpo oltre gli estremi, fetta laterale della pendenza, margine d'occupazione, piu'
# un decimetro di sicurezza. Ridurlo rompe l'uguaglianza con la costruzione completa
# (test_replay.py lo controlla).
DIRTY_MARGIN_M = (spotGrid.ARC_BODY_EXTENSION_M + spotGrid.LATERAL_SLICE_HALF_WIDTH_M
                  + spotGrid.PRM_EDGE_SAFETY_MARGIN_M + 0.10)


class PRM:
    """
    Grafo della missione: nodi del reticolo, archi validi con il loro costo, Dijkstra.
    """

    def __init__(self, max_edge_length: float = 2.0, connection_radius: float = 3.0,
                 min_edge_length: float = 0.5, alpha: float = 1.0, beta: float = 2.0, gamma: float = 0.0,
                 beta_lateral: float = None):
        """
        Args:
            max_edge_length, min_edge_length: lunghezza massima e minima degli archi (m)
            connection_radius: raggio entro cui cercare i vicini (m)
            alpha: peso della lunghezza nel costo
            beta: peso della pendenza LONGITUDINALE (beccheggio)
            gamma: peso della rugosita' (0: oggi la rugosita' non entra nel costo)
            beta_lateral: peso della pendenza LATERALE (rollio); None = beta *
                spotGrid.LATERAL_SLOPE_COST_MULTIPLIER
        """
        self.max_edge_length = max_edge_length
        self.connection_radius = connection_radius
        self.min_edge_length = min_edge_length

        # Parametri della funzione di costo
        self.alpha = alpha  # Peso distanza
        self.beta = beta  # Peso pendenza longitudinale (beccheggio)
        self.gamma = gamma  # Peso rugosità (roughness)
        self.beta_lateral = (beta_lateral if beta_lateral is not None
                             else beta * spotGrid.LATERAL_SLOPE_COST_MULTIPLIER)  # Peso pendenza laterale (rollio)

        self.nodes = {}  # {point_idx: (x, y)}
        self.node_gradients = {}  # {point_idx: slope_value} <-- Memorizza la pendenza del nodo
        self.node_roughness = {}  # <-- Memorizza la rugosità del nodo
        self.edges = {}  # {point_idx: [(neighbor_idx, weight), ...]}
        self.edge_validity = {}  # {(idx1, idx2): bool} - caches edge validity

        # Variabili di appoggio per la griglia locale live
        self.current_grad_2d = None
        self.current_rough_2d = None
        self.current_terrain_2d = None  # quote corrette della scansione, per la pendenza degli archi
        self.grid_origin_x = None
        self.grid_origin_y = None
        self.cell_size = None

        # Mappa globale delle quote (spotGrid.GlobalTerrainGrid): pendenza degli archi fuori
        # dalla finestra di adesso ma gia' visti prima (vedi set_global_terrain_map).
        self.global_terrain_map = None
        # Cache della pendenza dalla mappa globale, per arco: una voce resta valida finche' la
        # mappa non cambia vicino all'arco (vedi _sync_global_slope_cache).
        self._gslope_cache = {}
        self._gslope_map_id = None
        self._gslope_seen_updates = 0

        self.built = False
        # Zone (xmin, xmax, ymin, ymax) con dati nuovi dall'ultima valutazione degli archi:
        # vedi DIRTY_MARGIN_M e mission_graph.MissionGraph.add_edges_to_prm(incremental=True).
        self.dirty_rects = []
        self.hazards_seen = 0

        # Archi bloccati da un blocco confermato o da un fallimento fisico: esclusi per sempre
        # (vedi mark_edge_invalid).
        self.tracker_blocked_edges = set()
        # Archi effettivamente percorsi dal robot (liberi per costruzione).
        self.traversed_edges = set()
        # Nodo in cui si trova il robot adesso, e raggio entro cui la mappa globale non si usa
        # per gli archi che ne partono (vedi STOP_NODE_MAP_IGNORE_M).
        self.robot_node = None
        self.robot_node_ignore_m = 0.0
        # Nodi su cui la ricerca del percorso puo' passare: None = tutti. Serve a
        # tenere la pianificazione dentro le celle gia' visitate (vedi set_allowed_nodes).
        self.allowed_nodes = None
        # Tetto alla deviazione: un percorso piu' lungo di
        # max_detour_ratio * distanza in linea d'aria + detour_slack_m e' "nessun percorso".
        # None = nessun tetto. Vedi set_detour_limit.
        self.max_detour_ratio = None
        self.detour_slack_m = 0.0

        # ==================================================================
        # STATISTICHE DI MISSIONE: solo per il riepilogo di fine missione
        # (print_mission_summary), mai usate per decidere.
        # ==================================================================
        self.slope_source_counts = {'live': 0, 'global_map': 0, 'fallback_nodes': 0}
        self.last_slope_source = None  # ultima fonte usata, letta da easy_walk.py per il log per-arco
        # Questi tre contano VALUTAZIONI di veto (lo stesso arco si rivaluta molte volte); gli
        # archi DIVERSI sono nei set vetoed_arcs_* qui sotto.
        self.veto_count_longitudinal = 0  # valutazioni di veto per pendenza longitudinale
        self.veto_count_lateral = 0       # valutazioni di veto per pendenza laterale
        self.veto_count_occupancy = 0     # valutazioni di veto per occupazione globale (non pendenza)
        self.vetoed_arcs_longitudinal = set()  # chiavi (min_id, max_id) di archi DISTINTI vetati per long.
        self.vetoed_arcs_lateral = set()       # idem per laterale
        self.vetoed_arcs_occupancy = set()     # idem per occupazione
        self.max_slope_seen = {'long': 0.0, 'lat': 0.0}  # massimi misurati su qualunque arco valutato
        self.slope_flip_count = 0         # volte in cui un arco vicino soglia ha cambiato stato tra due refresh
        self.edge_slope_info = {}         # {(idx1,idx2): (long_slope, lat_slope)} -- per visualizzazione, niente ricalcolo

        # Scansione dal vivo per lo scarto preliminare (vedi LIVE_PREFILTER_OD_M e
        # set_live_obstacles). None = nessuna scansione: nessuno scarto.
        self.live_obstacles = None
        self.live_veto_last = 0       # archi scartati dalla scansione all'ultima chiamata
        self.live_prune_last = 0      # archi validi fuori raggio tolti all'ultimo refresh

        # STRATO DEI COSTI: cio' che si impara durante la missione non toglie archi dal grafo,
        # aggiunge un costo (anche infinito) che Dijkstra somma al peso dell'arco. Oggi si usa
        # per gli archi di una trappola, vietati per il solo tentativo (_ATTEMPT_MARK).
        self.edge_penalty_persistent = {}   # (min,max) -> peso aggiuntivo (anche inf)
        self.edge_penalty_transient = {}
        self.node_penalty_persistent = {}   # idx -> peso aggiuntivo (anche inf)
        self.node_penalty_transient = {}

    def clear_penalties_for_attempt(self):
        """
        Fine di un tentativo di cella: le trappole valevano per quel tentativo, non per la
        missione. Si tolgono SOLO quelle registrate come tali.
        """
        for k in [k for k, v in self.edge_penalty_persistent.items() if v == self._ATTEMPT_MARK]:
            del self.edge_penalty_persistent[k]
        for k in [k for k, v in self.node_penalty_persistent.items() if v == self._ATTEMPT_MARK]:
            del self.node_penalty_persistent[k]

    # Valore usato per marcare "impercorribile, ma solo per questo tentativo di cella".
    # E' un infinito distinguibile: si comporta come inf nel costo, e clear_penalties_for_attempt
    # riconosce proprio queste voci.
    _ATTEMPT_MARK = float('inf')

    # ---- strato dei costi: lettura ---------------------------------------------------
    def node_penalty(self, idx: int) -> float:
        return (self.node_penalty_persistent.get(idx, 0.0)
                + self.node_penalty_transient.get(idx, 0.0))

    def edge_penalty(self, idx1: int, idx2: int) -> float:
        key = (min(idx1, idx2), max(idx1, idx2))
        return (self.edge_penalty_persistent.get(key, 0.0)
                + self.edge_penalty_transient.get(key, 0.0))

    def effective_weight(self, idx1: int, idx2: int, weight: float) -> float:
        """Costo geometrico + penalita' dell'arco + penalita' del nodo di arrivo."""
        return weight + self.edge_penalty(idx1, idx2) + self.node_penalty(idx2)

    def add_node(self, point_idx: int, x: float, y: float, gradient: float = 0.0, roughness: float = 0.0):
        """Aggiunge un nodo con la sua pendenza e rugosita' media (usate solo come ultimo ripiego)."""
        self.nodes[point_idx] = (x, y)
        self.node_gradients[point_idx] = gradient
        self.node_roughness[point_idx] = roughness
        self.edges[point_idx] = []

    def set_global_terrain_map(self, global_terrain_map):
        """
        Inietta la mappa globale persistente delle altezze (spotGrid.GlobalTerrainGrid).
        Va richiamata una volta che l'oggetto esiste (tipicamente subito dopo averlo
        creato/recuperato in easy_walk.py) -- puo' essere richiamata piu' volte, e'
        idempotente, serve solo ad aggiornare il riferimento.
        """
        self.global_terrain_map = global_terrain_map

    def update_local_grid_data(self, grad_values_2d, rough_values_2d, grid_origin_x, grid_origin_y, cell_size,
                               terrain_values_2d=None, valid_2d=None):
        """
        Salva temporaneamente la mappa locale corrente dei gradienti, delle rugosità e
        (opzionale) delle altezze del terreno grezze (già corrette, vedi correct_terrain)
        per permettere il campionamento accurato degli archi -- incluso il profilo di
        pendenza denso per-arco calcolato direttamente dalle altezze reali.
        """
        self.current_grad_2d = grad_values_2d
        self.current_rough_2d = rough_values_2d
        self.current_terrain_2d = terrain_values_2d
        # celle attendibili (is_valid di correct_terrain): le quote riempite non entrano
        # nel profilo di pendenza -- vedi spotGrid.compute_arc_slope_profile
        self.current_valid_2d = valid_2d
        self.grid_origin_x = grid_origin_x
        self.grid_origin_y = grid_origin_y
        self.cell_size = cell_size
        self.dirty_rects.extend(self.live_rects())

    def live_rects(self):
        """Rettangoli coperti dai dati dal vivo correnti (griglia locale e scansione degli ostacoli)."""
        out = []
        if self.current_grad_2d is not None and self.grid_origin_x is not None and self.cell_size:
            nr, nc = np.shape(self.current_grad_2d)[:2]
            out.append((float(self.grid_origin_x), float(self.grid_origin_x + nc * self.cell_size),
                        float(self.grid_origin_y), float(self.grid_origin_y + nr * self.cell_size)))
        lo = getattr(self, 'live_obstacles', None)
        if lo is not None:
            nr, nc = lo['od'].shape[:2]
            out.append((lo['ox'], lo['ox'] + nc * lo['cs'], lo['oy'], lo['oy'] + nr * lo['cs']))
        return out

    def set_live_obstacles(self, obstacle_dist_2d, seen_2d, origin_x, origin_y, cell_size,
                           robot_x, robot_y, robot_yaw=None):
        """
        La scansione appena fatta, per lo scarto preliminare degli archi (vedi
        LIVE_PREFILTER_OD_M). Array 2D (num_y, num_x), riga = y. obstacle_dist_2d: nel ciclo di
        navigazione quella FILTRATA della fotografia del fronte (mediana + isteresi); all'inizio
        del tentativo quella grezza. seen_2d: celle viste dal sensore.
        """
        if obstacle_dist_2d is None:
            self.live_obstacles = None
            return
        od = np.asarray(obstacle_dist_2d, dtype=np.float64)
        seen = (np.ones(od.shape, dtype=bool) if seen_2d is None
                else np.asarray(seen_2d, dtype=bool).reshape(od.shape))
        self.live_obstacles = {
            'od': od, 'seen': seen, 'ox': float(origin_x), 'oy': float(origin_y),
            'cs': float(cell_size), 'rx': float(robot_x), 'ry': float(robot_y),
            'yaw': None if robot_yaw is None else float(robot_yaw)}
        nr, nc = od.shape[:2]
        self.dirty_rects.append((float(origin_x), float(origin_x + nc * cell_size),
                                 float(origin_y), float(origin_y + nr * cell_size)))

    def _live_blocked_samples(self, sx, sy):
        """
        Campioni (array di qualunque forma) -> True dove la scansione dal vivo dice "qui il
        fianco del robot entra in un ostacolo": cella dentro la finestra, vista, con
        obstacle_distance sotto LIVE_PREFILTER_OD_M, fuori dal rettangolo del corpo.
        """
        lo = self.live_obstacles
        if lo is None or LIVE_PREFILTER_OD_M is None:
            return np.zeros(np.shape(sx), dtype=bool)
        od = lo['od']
        ny, nx = od.shape
        cols = np.floor((sx - lo['ox']) / lo['cs']).astype(np.int64)
        rows = np.floor((sy - lo['oy']) / lo['cs']).astype(np.int64)
        inside = (rows >= 0) & (rows < ny) & (cols >= 0) & (cols < nx)
        rows = np.clip(rows, 0, ny - 1)
        cols = np.clip(cols, 0, nx - 1)
        v = od[rows, cols]
        hit = inside & lo['seen'][rows, cols] & np.isfinite(v) & (v < LIVE_PREFILTER_OD_M)
        dx, dy = sx - lo['rx'], sy - lo['ry']
        if lo['yaw'] is None:
            body = np.hypot(dx, dy) <= spotGrid.ROBOT_HALF_WIDTH_M
        else:
            c, s = np.cos(lo['yaw']), np.sin(lo['yaw'])
            body = ((np.abs(c * dx + s * dy) <= spotGrid.ROBOT_HALF_LENGTH_M)
                    & (np.abs(-s * dx + c * dy) <= spotGrid.ROBOT_HALF_WIDTH_M))
        return hit & ~body

    def _live_blocked_many(self, pairs):
        """[(idx1, idx2, dist), ...] -> array di bool: arco scartato dalla scansione dal vivo."""
        n_pairs = len(pairs)
        out = np.zeros(n_pairs, dtype=bool)
        if n_pairs == 0 or self.live_obstacles is None or LIVE_PREFILTER_OD_M is None:
            return out
        xy1 = np.array([self.nodes[p[0]] for p in pairs], dtype=np.float64)
        xy2 = np.array([self.nodes[p[1]] for p in pairs], dtype=np.float64)
        d = np.array([p[2] for p in pairs], dtype=np.float64)
        ns = np.maximum(OCCUPANCY_SAMPLES_MIN,
                        np.ceil(d * OCCUPANCY_SAMPLES_PER_M).astype(np.int64) + 1)
        for m in np.unique(ns):
            sel = np.nonzero(ns == m)[0]
            t = np.arange(int(m)) / (int(m) - 1)
            sx = xy1[sel, 0, None] + t[None, :] * (xy2[sel, 0] - xy1[sel, 0])[:, None]
            sy = xy1[sel, 1, None] + t[None, :] * (xy2[sel, 1] - xy1[sel, 1])[:, None]
            out[sel] = self._live_blocked_samples(sx, sy).any(axis=1)
        return out

    def live_prune_valid_edges(self, skip_keys=()):
        """
        Scarto preliminare sugli archi GIA' validi che il refresh non rivaluta (fuori da
        REFRESH_RADIUS_M ma dentro la finestra della scansione): un arco a 1.3-1.9 m dal robot che
        attraversa un ostacolo appena visto non resta valido finche' il robot non si avvicina.
        Si tolgono solo dal grafo (validita' False): alla prossima valutazione piena tornano se i
        dati lo dicono. Restituisce quanti archi ha tolto.
        """
        lo = self.live_obstacles
        if lo is None or LIVE_PREFILTER_OD_M is None:
            return 0
        ny, nx = lo['od'].shape
        x0, y0 = lo['ox'], lo['oy']
        x1, y1 = x0 + nx * lo['cs'], y0 + ny * lo['cs']
        reach = self.max_edge_length
        cand = []
        for a, nbrs in self.edges.items():
            xa, ya = self.nodes.get(a, (None, None))
            if xa is None or not (x0 - reach <= xa <= x1 + reach and y0 - reach <= ya <= y1 + reach):
                continue
            for b, _w in nbrs:
                if b <= a:
                    continue
                key = (a, b)
                if key in skip_keys or key in self.traversed_edges:
                    continue
                xb, yb = self.nodes[b]
                cand.append((a, b, float(np.hypot(xb - xa, yb - ya))))
        if not cand:
            return 0
        flags = self._live_blocked_many(cand)
        n = 0
        for (a, b, _d), f in zip(cand, flags):
            if not f:
                continue
            self.edge_validity[(a, b)] = False
            self.edges[a] = [(k, w) for k, w in self.edges.get(a, []) if k != b]
            self.edges[b] = [(k, w) for k, w in self.edges.get(b, []) if k != a]
            self._record_occupancy_veto((a, b))
            n += 1
        return n

    def refresh_local_edge_weights(self, global_map=None, edge_safety_margin: float = 0.05):
        """
        Rivaluta da capo (occupazione, pendenza, peso) gli archi entro REFRESH_RADIUS_M dal robot
        che hanno almeno UN estremo nella finestra della scansione, anche quelli scartati prima:
        un arco tolto per una lettura rumorosa torna valido appena i dati lo dicono, e un arco che
        attraversa un muro appena visto verso un nodo lontano viene riesaminato subito. Gli archi
        percorsi dal robot (traversed_edges) non si toccano. Restituisce i nodi rivalutati.
        """
        if self.current_grad_2d is None or self.grid_origin_x is None or self.cell_size is None:
            return set()

        num_rows, num_cols = self.current_grad_2d.shape
        # Il centro da cui si misura il raggio e' il nodo del robot; se non e' noto, il centro
        # della finestra (che e' comunque il robot).
        center = None
        if REFRESH_RADIUS_M is not None:
            if self.robot_node is not None and self.robot_node in self.nodes:
                center = self.nodes[self.robot_node]
            else:
                center = (self.grid_origin_x + num_cols * self.cell_size / 2.0,
                          self.grid_origin_y + num_rows * self.cell_size / 2.0)
        local_nodes = []
        for idx, (x, y) in self.nodes.items():
            col_idx = int(np.floor((x - self.grid_origin_x) / self.cell_size))
            row_idx = int(np.floor((y - self.grid_origin_y) / self.cell_size))
            if not (0 <= row_idx < num_rows and 0 <= col_idx < num_cols):
                continue
            if center is not None and np.hypot(x - center[0], y - center[1]) > REFRESH_RADIUS_M:
                continue        # fuori dal raggio: tiene i pesi dell'ultima valutazione
            local_nodes.append(idx)
        local_set = set(local_nodes)

        all_ids = list(self.nodes.keys())
        all_xy = np.array([self.nodes[i] for i in all_ids], dtype=np.float64) if all_ids else np.empty((0, 2))
        reach = min(self.connection_radius, self.max_edge_length)
        self._sync_global_slope_cache()

        pairs = []
        for idx1 in local_nodes:
            x1, y1 = self.nodes[idx1]
            d_all = np.hypot(all_xy[:, 0] - x1, all_xy[:, 1] - y1)
            for j in np.nonzero((d_all >= self.min_edge_length) & (d_all <= reach))[0]:
                idx2 = all_ids[j]
                if idx2 == idx1 or (idx2 in local_set and idx2 < idx1):
                    continue           # coppia locale-locale gia' considerata dall'altro estremo
                pairs.append((idx1, idx2, float(d_all[j])))

        # Occupazione di tutte le coppie in un colpo solo (esito identico a _occupancy_blocked)
        # e cache della pendenza globale riempita dove certamente non ci sono dati.
        use_occ = global_map is not None and edge_safety_margin > 0.0
        occ_flags = (self._occupancy_blocked_many(pairs, global_map, edge_safety_margin)
                     if use_occ else np.zeros(len(pairs), dtype=bool))
        self._prefill_global_slope_nodata(pairs)

        # Pendenze dal vivo di tutte le coppie da valutare in un colpo solo (stessi numeri del
        # calcolo arco per arco). Si saltano quelle che il ciclo sotto non valuta: scartate per
        # occupazione, bloccate da vicino, percorse dal robot (se una di queste ultime risulta
        # da rivalutare, il suo profilo si calcola arco per arco).
        live_profs = [None] * len(pairs)
        if REFRESH_BATCH_SLOPES and pairs:
            skip = np.array([bool(o) or (min(a, b), max(a, b)) in self.tracker_blocked_edges
                             or (min(a, b), max(a, b)) in self.traversed_edges
                             for (a, b, _d), o in zip(pairs, occ_flags)], dtype=bool)
            live_profs = self.precompute_arc_slopes(pairs, skip=skip)

        for (idx1, idx2, dist), occ_flag, live_prof in zip(pairs, occ_flags, live_profs):
            x1, y1 = self.nodes[idx1]
            x2, y2 = self.nodes[idx2]
            key = (min(idx1, idx2), max(idx1, idx2))
            if key in self.tracker_blocked_edges:
                continue  # blocco confermato da vicino (fronte sicuro) -- mai ricreato
            if key in self.traversed_edges:
                if self._traversed_edge_ok(idx1, idx2, global_map):
                    continue  # percorso dal robot: libero per costruzione
                self.traversed_edges.discard(key)   # ora attraversa il punto di una caduta

            old_valid = self.edge_validity.get(key)  # None se l'arco non era mai stato valutato prima

            # Inizializzate qui e non dentro `if not blocked:` perche' vengono
            # rilette piu' sotto per passare il profilo gia' calcolato a
            # _compute_edge_weight (vedi sample_terrain_between_points).
            long_slope, lat_slope, has_slope_data = 0.0, 0.0, False
            prof = (long_slope, lat_slope, has_slope_data)

            blocked = False
            if use_occ:
                if occ_flag:
                    blocked = True
                    self._record_occupancy_veto(key)

            if not blocked:
                # Una sola lettura del terreno; veto e costo usano lo stesso valore, quello
                # col corpo (vedi body_slope e spotGrid.ARC_BODY_EXTENSION_M).
                prof = (live_prof if live_prof is not None
                        else self.compute_arc_slope_profile(x1, y1, x2, y2, body=True))
                long_slope, lat_slope, has_slope_data = body_slope(prof)
                if has_slope_data:
                    self.edge_slope_info[key] = (long_slope, lat_slope)  # per visualizzazione, no ricalcolo
                    if self._record_slope_veto(key, long_slope, lat_slope):
                        blocked = True

                    # --- Solo diagnostico: un arco vicino soglia che cambia stato tra due
                    # refresh consecutivi puo' indicare oscillazione dovuta a rumore del
                    # sensore piu' che a un cambiamento reale del terreno -- non influisce
                    # su nessuna decisione, serve solo a farlo emergere nei log.
                    near_thresh = spotGrid.SLOPE_THRESHOLD * (1.0 - spotGrid.NEAR_THRESHOLD_MARGIN_FRACTION)
                    is_near_threshold = (long_slope >= near_thresh) or (lat_slope >= near_thresh)
                    new_valid = not blocked
                    if is_near_threshold and old_valid is not None and old_valid != new_valid:
                        self.slope_flip_count += 1
                        self._slope_pending()['flip'] += 1

            self.edges[idx1] = [(n, w) for n, w in self.edges.get(idx1, []) if n != idx2]
            self.edges[idx2] = [(n, w) for n, w in self.edges.get(idx2, []) if n != idx1]

            if blocked:
                self.edge_validity[key] = False
            else:
                # Il profilo dal vivo e' gia' calcolato (anche quando dice "nessun dato"):
                # non lo si rifa' dentro il calcolo del peso.
                weight = self._compute_edge_weight(
                    idx1, idx2, dist, precomputed_slope=prof)
                self.edges[idx1].append((idx2, weight))
                self.edges[idx2].append((idx1, weight))
                self.edge_validity[key] = True

        # Gli archi validi della finestra fuori dal raggio di refresh passano almeno dallo
        # scarto preliminare sulla scansione dal vivo (live_prune_valid_edges).
        done = {(min(a, b), max(a, b)) for a, b, _d in pairs}
        self.live_prune_last = self.live_prune_valid_edges(skip_keys=done)
        self.flush_slope_log("refresh")

        return set(local_nodes)

    def compute_arc_slope_profile(self, x1: float, y1: float, x2: float, y2: float,
                                   perp_sample_spacing_m: float = 0.10, body: bool = False):
        """
        spotGrid.compute_arc_slope_profile sulla griglia dal vivo di questo ciclo (vedi
        update_local_grid_data). Restituisce (pendenza longitudinale, laterale, has_data);
        con body=True l'arco e' allungato di mezzo corpo per parte.
        """
        return spotGrid.compute_arc_slope_profile(
            self.current_terrain_2d, self.grid_origin_x, self.grid_origin_y, self.cell_size,
            x1, y1, x2, y2, perp_sample_spacing_m=perp_sample_spacing_m,
            valid_2d=getattr(self, 'current_valid_2d', None),
            body_ext_m=(spotGrid.ARC_BODY_EXTENSION_M if body else 0.0)
        )

    def sample_terrain_between_points(self, x1: float, y1: float, x2: float, y2: float,
                                       node_idx1: Optional[int] = None, node_idx2: Optional[int] = None,
                                       precomputed_slope: Optional[Tuple[float, float, bool]] = None,
                                       need_roughness: bool = True):
        """
        Difficolta' del tratto (x1, y1) -> (x2, y2): (pendenza longitudinale, pendenza laterale,
        rugosita'). Serve al costo degli archi e alla scelta dell'andatura.

        Pendenza: direzionale, col corpo (body_slope su compute_arc_slope_profile). Fonti, dalla
        migliore alla piu' grezza:
          1. la scansione attuale (self.current_terrain_2d);
          2. la mappa globale delle quote, per un tratto gia' visto ma ora fuori dalla finestra;
          3. la media della pendenza dei due nodi, tutta sulla componente longitudinale.
        precomputed_slope: profilo gia' calcolato dal chiamante (3 o 6 valori), per non rifarlo.

        Rugosita': 90 percentile del campo per cella lungo il tratto (OCCUPANCY_SAMPLES_PER_M
        campioni per metro); fuori dalla scansione, la media dei nodi. need_roughness=False la
        salta (con gamma = 0 non entra nel costo).

        node_idx1/node_idx2: id dei nodi, per la cache della mappa globale e per il ripiego.
        """
        dist = float(np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
        num_samples = max(OCCUPANCY_SAMPLES_MIN, int(dist * OCCUPANCY_SAMPLES_PER_M))

        # --- Rugosita': isotropa, dal campo per cella della scansione ---
        sampled_roughness = []
        has_rough_grid = (need_roughness and self.current_rough_2d is not None
                          and self.grid_origin_x is not None and self.cell_size is not None)
        if has_rough_grid:
            num_rows, num_cols = self.current_rough_2d.shape
            for i in range(num_samples):
                t = i / (num_samples - 1)
                cx = x1 + t * (x2 - x1)
                cy = y1 + t * (y2 - y1)

                dx = cx - self.grid_origin_x
                dy = cy - self.grid_origin_y
                col_idx = int(np.floor(dx / self.cell_size))   # floor: int() sbaglia sotto zero
                row_idx = int(np.floor(dy / self.cell_size))

                if 0 <= row_idx < num_rows and 0 <= col_idx < num_cols:
                    sampled_roughness.append(self.current_rough_2d[row_idx, col_idx])

        if sampled_roughness:
            actual_roughness = spotGrid.p90(sampled_roughness)
        else:
            r1 = self.node_roughness.get(node_idx1, 0.0)
            r2 = self.node_roughness.get(node_idx2, 0.0)
            actual_roughness = float((r1 + r2) / 2.0)

        # --- Pendenza: direzionale e col corpo (body_slope), dalla scansione dal vivo ---
        if precomputed_slope is None:
            precomputed_slope = self.compute_arc_slope_profile(x1, y1, x2, y2, body=True)
        longitudinal_slope, lateral_slope, has_slope_data = body_slope(precomputed_slope)
        source = 'live'

        if not has_slope_data and self.global_terrain_map is not None:
            # Fuori dalla scansione attuale: la mappa globale, se il tratto era gia' stato visto.
            g_long, g_lat, g_has_data = self._global_slope(x1, y1, x2, y2, node_idx1, node_idx2)
            if g_has_data:
                longitudinal_slope, lateral_slope, has_slope_data = g_long, g_lat, True
                source = 'global_map'

        if not has_slope_data:
            # Nessun dato da nessuna fonte: media dei nodi, tutta sul longitudinale.
            grad1 = self.node_gradients.get(node_idx1, 0.0)
            grad2 = self.node_gradients.get(node_idx2, 0.0)
            longitudinal_slope = float((grad1 + grad2) / 2.0)
            lateral_slope = 0.0
            source = 'fallback_nodes'

        # Statistiche: solo per il riepilogo, non decidono niente.
        self.slope_source_counts[source] += 1
        self.last_slope_source = source

        return float(longitudinal_slope), float(lateral_slope), actual_roughness

    def _record_slope_veto(self, key, long_slope: float, lat_slope: float) -> bool:
        """
        Veto di pendenza (spotGrid.SLOPE_THRESHOLD, in entrambe le direzioni) con le statistiche.
        Restituisce True se l'arco va scartato. I veti si riassumono in una riga [PENDENZE] per
        costruzione o refresh (flush_slope_log); i valori arco per arco restano nei _vis.npz.
        """
        self.max_slope_seen['long'] = max(self.max_slope_seen['long'], long_slope)
        self.max_slope_seen['lat'] = max(self.max_slope_seen['lat'], lat_slope)

        vetoed = False
        pend = self._slope_pending()
        if long_slope > spotGrid.SLOPE_THRESHOLD:
            vetoed = True
            self.veto_count_longitudinal += 1
            pend['long'] += 1
            pend['max_long'] = max(pend['max_long'], long_slope)
            if key not in self.vetoed_arcs_longitudinal:
                self.vetoed_arcs_longitudinal.add(key)
                pend['new'] += 1
        if lat_slope > spotGrid.SLOPE_THRESHOLD:
            vetoed = True
            self.veto_count_lateral += 1
            pend['lat'] += 1
            pend['max_lat'] = max(pend['max_lat'], lat_slope)
            if key not in self.vetoed_arcs_lateral:
                self.vetoed_arcs_lateral.add(key)
                pend['new'] += 1
        return vetoed

    def _slope_pending(self):
        p = getattr(self, '_slope_log_pending', None)
        if p is None:
            p = self._slope_log_pending = {'long': 0, 'lat': 0, 'new': 0, 'flip': 0,
                                           'max_long': 0.0, 'max_lat': 0.0}
        return p

    def flush_slope_log(self, where=""):
        """
        Una riga [PENDENZE] con i veti dall'ultima chiamata, poi azzera.
        Non stampa niente se non ci sono stati veti.
        """
        p = self._slope_pending()
        if p['long'] or p['lat'] or p['flip']:
            print(f"[PENDENZE] {where}: archi vetati per pendenza -- longitudinale {p['long']} "
                  f"(max {p['max_long']:.2f}), laterale {p['lat']} (max {p['max_lat']:.2f}), "
                  f"{p['new']} mai vetati prima; {p['flip']} cambi di verdetto vicino soglia "
                  f"(soglia {spotGrid.SLOPE_THRESHOLD:.2f}).")
        self._slope_log_pending = None

    def _record_occupancy_veto(self, key):
        """Conteggio di debug dei veti per occupazione globale (comportamento invariato)."""
        self.veto_count_occupancy += 1
        self.vetoed_arcs_occupancy.add(key)

    def get_mission_summary_dict(self) -> dict:
        """Le stesse statistiche di print_mission_summary, in forma serializzabile (JSON)."""
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
            'slope_threshold': spotGrid.SLOPE_THRESHOLD,
            'slope_baseline_m': spotGrid.SLOPE_BASELINE_M,
            'lateral_slice_half_width_m': spotGrid.LATERAL_SLICE_HALF_WIDTH_M,
            'beta': self.beta,
            'beta_lateral': self.beta_lateral,
            'gamma': self.gamma,
            'alpha': self.alpha,
        }

    def print_mission_summary(self):
        """
        Stampa un riepilogo cumulativo per tutta la missione -- pensato per essere
        richiamato una volta a fine missione, invece di dover leggere migliaia di righe
        di log per capire quanto spesso ciascun meccanismo è intervenuto davvero.
        Puramente diagnostico, non influisce sul comportamento del robot.
        """
        total_sources = sum(self.slope_source_counts.values())
        print("\n================ [PRM MISSION SUMMARY] ================")
        print(f"Fonte pendenza usata nel costo degli archi (totale {total_sources} query):")
        for src, count in self.slope_source_counts.items():
            pct = (100.0 * count / total_sources) if total_sources > 0 else 0.0
            print(f"  {src:15s}: {count:6d}  ({pct:5.1f}%)")
        print("Veti -- archi DISTINTI (e, tra parentesi, valutazioni totali, che contano lo stesso arco "
              "a ogni refresh):")
        print(f"  longitudinale: {len(self.vetoed_arcs_longitudinal)} ({self.veto_count_longitudinal})  "
              f"laterale: {len(self.vetoed_arcs_lateral)} ({self.veto_count_lateral})  "
              f"occupazione: {len(self.vetoed_arcs_occupancy)} ({self.veto_count_occupancy})")
        print(f"Pendenza massima misurata su un arco -- longitudinale: {self.max_slope_seen['long']:.3f}, "
              f"laterale: {self.max_slope_seen['lat']:.3f} (soglia veto {spotGrid.SLOPE_THRESHOLD:.3f})")
        print(f"Geometria del profilo d'arco -- base {spotGrid.SLOPE_BASELINE_M:.2f} m, "
              f"fetta laterale +/-{spotGrid.LATERAL_SLICE_HALF_WIDTH_M:.2f} m")
        print(f"Pesi del costo -- alpha={self.alpha}, beta(long)={self.beta}, "
              f"beta_lateral={self.beta_lateral}, gamma={self.gamma}")
        print(f"Flip del veto pendenza vicino soglia (possibile oscillazione): {self.slope_flip_count}")
        print("=========================================================\n")

    def _compute_edge_weight(self, idx1: int, idx2: int, dist: float,
                             precomputed_slope: Optional[Tuple[float, float, bool]] = None) -> float:
        """
        Costo dell'arco: alpha * lunghezza + beta * pendenza longitudinale + beta_lateral *
        pendenza laterale + gamma * rugosita', con le pendenze di sample_terrain_between_points.
        Il termine di pendenza e' fisso per arco (non per metro): favorisce archi lunghi, che e'
        voluto (vedi CHANGELOG 2026-10-14). precomputed_slope evita di ricalcolare il profilo.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]

        # Con gamma = 0 la rugosita' non entra nel costo, quindi non la si campiona (l'andatura
        # la chiede a sample_terrain_between_points direttamente).
        longitudinal_slope, lateral_slope, actual_roughness = self.sample_terrain_between_points(
            x1, y1, x2, y2, node_idx1=idx1, node_idx2=idx2,
            precomputed_slope=precomputed_slope, need_roughness=(self.gamma != 0.0)
        )

        # Costo dell'arco
        weight = ((self.alpha * dist) + (self.beta * longitudinal_slope) +
                 (self.beta_lateral * lateral_slope) + (self.gamma * actual_roughness))
        return float(weight)

    def _global_slope(self, x1, y1, x2, y2, idx1=None, idx2=None):
        """
        Pendenza dalla mappa globale, con cache per arco. La cache si usa solo quando i punti
        sono davvero i due nodi (non per un tratto parziale verso un punto intermedio).
        """
        tm = self.global_terrain_map

        def _with_body(x1, y1, x2, y2):
            # Come per il dal vivo (sample_terrain_between_points): il peggiore fra arco e
            # arco allungato di mezzo corpo, solo se l'arco ha dati suoi.
            ext = spotGrid.ARC_BODY_EXTENSION_M
            if ext <= 0:
                return tm.compute_arc_slope_profile(x1, y1, x2, y2)
            r = tm.compute_arc_slope_profile(x1, y1, x2, y2, body_ext_m=ext)
            return (r[3], r[4], True) if r[2] else r[:3]

        cacheable = (idx1 is not None and idx2 is not None and idx1 in self.nodes and idx2 in self.nodes
                     and self.nodes[idx1] == (x1, y1) and self.nodes[idx2] == (x2, y2))
        if not cacheable:
            return _with_body(x1, y1, x2, y2)
        key = (min(idx1, idx2), max(idx1, idx2))
        hit = self._gslope_cache.get(key)
        if hit is None:
            hit = _with_body(x1, y1, x2, y2)
            self._gslope_cache[key] = hit
        return hit

    def _sync_global_slope_cache(self):
        """
        Butta le voci della cache su cui la mappa globale e' cambiata: gli archi il cui
        rettangolo (allargato della fetta laterale) interseca una zona aggiornata dopo il
        calcolo. Se la mappa non registra le zone aggiornate, svuota tutto (sempre corretto).
        """
        tm = self.global_terrain_map
        if tm is None:
            self._gslope_cache.clear()
            return
        if id(tm) != self._gslope_map_id:
            self._gslope_cache.clear()
            self._gslope_map_id = id(tm)
            self._gslope_seen_updates = len(getattr(tm, 'update_rects', None) or [])
            return
        rects = getattr(tm, 'update_rects', None)
        if rects is None:
            self._gslope_cache.clear()
            return
        new = rects[self._gslope_seen_updates:]
        self._gslope_seen_updates = len(rects)
        if not new or not self._gslope_cache:
            return
        keys = [k for k in self._gslope_cache if k[0] in self.nodes and k[1] in self.nodes]
        if len(keys) != len(self._gslope_cache):
            self._gslope_cache = {k: self._gslope_cache[k] for k in keys}
        if not keys:
            return
        a = np.array([self.nodes[k[0]] for k in keys]); b = np.array([self.nodes[k[1]] for k in keys])
        # + ARC_BODY_EXTENSION_M: il valore in cache guarda anche oltre gli estremi.
        m = spotGrid.LATERAL_SLICE_HALF_WIDTH_M + 2 * tm.resolution + spotGrid.ARC_BODY_EXTENSION_M
        exmin, exmax = np.minimum(a[:, 0], b[:, 0]) - m, np.maximum(a[:, 0], b[:, 0]) + m
        eymin, eymax = np.minimum(a[:, 1], b[:, 1]) - m, np.maximum(a[:, 1], b[:, 1]) + m
        stale = np.zeros(len(keys), dtype=bool)
        for (rxmin, rxmax, rymin, rymax) in new:
            stale |= (exmax >= rxmin) & (exmin <= rxmax) & (eymax >= rymin) & (eymin <= rymax)
        for j in np.nonzero(stale)[0]:
            del self._gslope_cache[keys[j]]

    def _traversed_edge_ok(self, a, b, global_map) -> bool:
        """
        Un arco percorso dal robot resta nel grafo senza controlli (e' libero per costruzione),
        TRANNE se e' stato bloccato dopo (blocco confermato o fallimento fisico) o se ora passa
        sopra il punto di una caduta: un arco percorso all'andata e poi bloccato non deve
        tornare nel grafo alla ricostruzione successiva.
        """
        key = (min(a, b), max(a, b))
        if key in self.tracker_blocked_edges:
            return False
        if global_map is not None and hasattr(global_map, 'segment_hits_hazard'):
            (xa, ya), (xb, yb) = self.nodes[a], self.nodes[b]
            if global_map.segment_hits_hazard(xa, ya, xb, yb, spotGrid.PRM_EDGE_SAFETY_MARGIN_M):
                return False
        return True

    def set_robot_node(self, node_id, ignore_map_within_m: float = STOP_NODE_MAP_IGNORE_M):
        """Registra il nodo in cui si trova il robot adesso (vedi STOP_NODE_MAP_IGNORE_M)."""
        self.robot_node = node_id
        self.robot_node_ignore_m = float(ignore_map_within_m)

    def _occupancy_blocked(self, idx1, idx2, dist, global_map, edge_safety_margin,
                           use_live=True) -> bool:
        """
        True se l'arco e' bloccato per occupazione: dallo scarto preliminare sulla scansione dal
        vivo (LIVE_PREFILTER_OD_M) o dalla mappa globale, con un campione ogni 10 cm. I campioni
        entro robot_node_ignore_m dal nodo del robot non contano per la mappa globale.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]
        if use_live and self.live_obstacles is not None:
            if self._live_blocked_many([(idx1, idx2, float(dist))])[0]:
                return True
        ignore_xy = None
        if self.robot_node is not None and self.robot_node in (idx1, idx2) and self.robot_node_ignore_m > 0:
            ignore_xy = self.nodes[self.robot_node]
        # ceil + 1: passo fra campioni <= 1/OCCUPANCY_SAMPLES_PER_M.
        num_samples = max(OCCUPANCY_SAMPLES_MIN, int(np.ceil(dist * OCCUPANCY_SAMPLES_PER_M)) + 1)
        for k in range(num_samples):
            t = k / (num_samples - 1)
            sx = x1 + t * (x2 - x1)
            sy = y1 + t * (y2 - y1)
            if ignore_xy is not None and np.hypot(sx - ignore_xy[0], sy - ignore_xy[1]) <= self.robot_node_ignore_m:
                continue
            if global_map.is_occupied(sx, sy, safety_margin=edge_safety_margin):
                return True
        return False

    def _occupancy_blocked_many(self, pairs, global_map, edge_safety_margin):
        """
        _occupancy_blocked per molti archi insieme: [(idx1, idx2, dist), ...] -> array di bool,
        IDENTICO arco per arco (stessi campioni, stessa deroga attorno al nodo del robot, stesso
        test di GlobalGrid tramite occupied_many).
        """
        # Scarto preliminare sulla scansione dal vivo (LIVE_PREFILTER_OD_M) e mappa globale:
        # l'esito e' l'OR dei due.
        live = self._live_blocked_many(pairs)
        self.live_veto_last = int(live.sum())
        out = self._global_occupancy_blocked_many(pairs, global_map, edge_safety_margin)
        return out | live

    def _global_occupancy_blocked_many(self, pairs, global_map, edge_safety_margin):
        """Il controllo sulla mappa globale di _occupancy_blocked_many, senza la scansione dal vivo."""
        n_pairs = len(pairs)
        out = np.zeros(n_pairs, dtype=bool)
        if n_pairs == 0 or global_map is None or edge_safety_margin <= 0.0:
            return out
        if not hasattr(global_map, 'occupied_many'):
            for i, (a, b, d) in enumerate(pairs):
                out[i] = self._occupancy_blocked(a, b, d, global_map, edge_safety_margin,
                                                 use_live=False)
            return out
        a_ids = [p[0] for p in pairs]
        b_ids = [p[1] for p in pairs]
        xy1 = np.array([self.nodes[a] for a in a_ids], dtype=np.float64)
        xy2 = np.array([self.nodes[b] for b in b_ids], dtype=np.float64)
        d = np.array([p[2] for p in pairs], dtype=np.float64)
        ns = np.maximum(OCCUPANCY_SAMPLES_MIN,
                        np.ceil(d * OCCUPANCY_SAMPLES_PER_M).astype(np.int64) + 1)
        ignore_xy = None
        touches = np.zeros(n_pairs, dtype=bool)
        if self.robot_node is not None and self.robot_node_ignore_m > 0 and self.robot_node in self.nodes:
            ignore_xy = self.nodes[self.robot_node]
            touches = np.array([(a == self.robot_node) or (b == self.robot_node)
                                for a, b in zip(a_ids, b_ids)], dtype=bool)
        for m in np.unique(ns):
            sel = np.nonzero(ns == m)[0]
            t = np.arange(int(m)) / (int(m) - 1)
            sx = xy1[sel, 0, None] + t[None, :] * (xy2[sel, 0] - xy1[sel, 0])[:, None]
            sy = xy1[sel, 1, None] + t[None, :] * (xy2[sel, 1] - xy1[sel, 1])[:, None]
            keep = None
            if ignore_xy is not None and touches[sel].any():
                near = np.hypot(sx - ignore_xy[0], sy - ignore_xy[1]) <= self.robot_node_ignore_m
                keep = ~(near & touches[sel, None])
            occ = global_map.occupied_many(sx, sy, safety_margin=edge_safety_margin)
            if keep is not None:
                occ &= keep
            out[sel] = occ.any(axis=1)
        return out

    def _prefill_global_slope_nodata(self, pairs):
        """
        Mette in cache (0, 0, False) -- "nessun dato" -- per gli archi su cui la mappa globale del
        terreno risponderebbe certamente cosi' (GlobalTerrainGrid.certainly_no_slope_data): e' lo
        stesso valore, nello stesso verso, che _global_slope calcolerebbe alla prima chiamata.
        """
        tm = self.global_terrain_map
        if tm is None or not hasattr(tm, 'certainly_no_slope_data') or not pairs:
            return 0
        todo = [(a, b) for a, b, _ in pairs
                if (min(a, b), max(a, b)) not in self._gslope_cache]
        if not todo:
            return 0
        xy1 = np.array([self.nodes[a] for a, _ in todo], dtype=np.float64)
        xy2 = np.array([self.nodes[b] for _, b in todo], dtype=np.float64)
        nodata = tm.certainly_no_slope_data(xy1[:, 0], xy1[:, 1], xy2[:, 0], xy2[:, 1])
        n = 0
        for (a, b), nd in zip(todo, nodata):
            if nd:
                self._gslope_cache[(min(a, b), max(a, b))] = (0.0, 0.0, False)
                n += 1
        return n

    def precompute_arc_slopes(self, pairs, skip=None):
        """
        Pendenza di molte coppie in un colpo solo, con le matrici di
        spotGrid.arc_slope_profiles_batch: numeri IDENTICI al calcolo arco per arco.

        pairs: [(idx1, idx2, dist)], nel verso in cui verranno valutate; skip: maschera delle
        coppie da non calcolare (es. gia' scartate per occupazione).
        Restituisce, per ogni coppia, il profilo dal vivo (6 valori, come
        compute_arc_slope_profile(..., body=True)) o None se saltata. In piu' riempie la cache
        della pendenza globale (_gslope_cache) per le coppie senza dati dal vivo, con lo stesso
        valore e lo stesso verso che _global_slope calcolerebbe alla prima chiamata.
        """
        n = len(pairs)
        out = [None] * n
        if n == 0:
            return out
        use = np.ones(n, dtype=bool) if skip is None else ~np.asarray(skip, dtype=bool)
        idx = np.nonzero(use)[0]
        if idx.size == 0:
            return out
        xy1 = np.array([self.nodes[pairs[i][0]] for i in idx], dtype=np.float64)
        xy2 = np.array([self.nodes[pairs[i][1]] for i in idx], dtype=np.float64)
        ext = spotGrid.ARC_BODY_EXTENSION_M
        live = spotGrid.compute_arc_slope_profiles_batch(
            self.current_terrain_2d, self.grid_origin_x, self.grid_origin_y, self.cell_size,
            xy1[:, 0], xy1[:, 1], xy2[:, 0], xy2[:, 1],
            valid_2d=getattr(self, 'current_valid_2d', None), body_ext_m=ext)
        for j, i in enumerate(idx):
            r = live[j]
            out[i] = (float(r[0]), float(r[1]), bool(r[2]), float(r[3]), float(r[4]), bool(r[5]))
        tm = self.global_terrain_map
        if tm is not None and hasattr(tm, 'compute_arc_slope_profiles_batch') and ext > 0:
            need = [j for j, i in enumerate(idx) if not live[j, 2]
                    and (min(pairs[i][0], pairs[i][1]), max(pairs[i][0], pairs[i][1])) not in self._gslope_cache]
            if need:
                need = np.array(need)
                g = tm.compute_arc_slope_profiles_batch(xy1[need, 0], xy1[need, 1], xy2[need, 0], xy2[need, 1],
                                                        body_ext_m=ext)
                for j, r in zip(need, g):
                    a, b = pairs[idx[j]][0], pairs[idx[j]][1]
                    # come _global_slope._with_body: col corpo se l'arco ha dati suoi
                    self._gslope_cache[(min(a, b), max(a, b))] = (
                        (float(r[3]), float(r[4]), True) if r[2] else (float(r[0]), float(r[1]), False))
        return out

    def _live_slope_certainly_nodata(self, pairs):
        """
        Per ogni coppia: True se compute_arc_slope_profile sulla scansione dal vivo uscirebbe
        subito con "nessun dato" (la sua stessa uscita rapida, stessi numeri).
        """
        n = len(pairs)
        out = np.ones(n, dtype=bool)
        if (n == 0 or self.current_terrain_2d is None or self.grid_origin_x is None
                or self.cell_size is None):
            return out
        xy1 = np.array([self.nodes[a] for a, _, _ in pairs], dtype=np.float64)
        xy2 = np.array([self.nodes[b] for _, b, _ in pairs], dtype=np.float64)
        num_rows, num_cols = self.current_terrain_2d.shape
        pad = spotGrid.LATERAL_SLICE_HALF_WIDTH_M + self.cell_size
        x_hi = self.grid_origin_x + num_cols * self.cell_size
        y_hi = self.grid_origin_y + num_rows * self.cell_size
        out = ((np.maximum(xy1[:, 0], xy2[:, 0]) + pad < self.grid_origin_x)
               | (np.minimum(xy1[:, 0], xy2[:, 0]) - pad > x_hi)
               | (np.maximum(xy1[:, 1], xy2[:, 1]) + pad < self.grid_origin_y)
               | (np.minimum(xy1[:, 1], xy2[:, 1]) - pad > y_hi))
        return out

    def _add_edge_without_data(self, idx1: int, idx2: int, dist: float) -> bool:
        """
        ZONA SENZA DATI: arco su cui nessuna fonte ha informazione (fuori dalla scansione, mappa
        globale del terreno certamente senza dati, occupazione libera, non bloccato, gamma = 0).
        _evaluate_and_add_edge arriverebbe al costo di ripiego (lunghezza + media della pendenza
        dei nodi): qui lo si calcola direttamente, con la stessa formula e gli stessi contatori,
        risultato identico al bit. Solo per adiacenze appena svuotate (fresh).
        """
        grad1 = self.node_gradients.get(idx1, 0.0)
        grad2 = self.node_gradients.get(idx2, 0.0)
        longitudinal_slope = float((grad1 + grad2) / 2.0)
        lateral_slope = 0.0
        r1 = self.node_roughness.get(idx1, 0.0)
        r2 = self.node_roughness.get(idx2, 0.0)
        actual_roughness = float((r1 + r2) / 2.0)
        self.slope_source_counts['fallback_nodes'] += 1
        self.last_slope_source = 'fallback_nodes'
        weight = float((self.alpha * dist) + (self.beta * float(longitudinal_slope)) +
                       (self.beta_lateral * float(lateral_slope)) + (self.gamma * actual_roughness))
        self.edges.setdefault(idx1, []).append((idx2, weight))
        self.edges.setdefault(idx2, []).append((idx1, weight))
        self.edge_validity[(min(idx1, idx2), max(idx1, idx2))] = True
        return True

    def _evaluate_and_add_edge(self, idx1: int, idx2: int, dist: float,
                               global_map=None, edge_safety_margin: float = 0.0,
                               force: bool = False, occ_blocked=None,
                               fresh: bool = False, live_prof=None) -> bool:
        """
        Decide se l'arco idx1-idx2 e' ammesso e, se si', lo aggiunge con il suo peso.
        Controlli in ordine: arco bloccato, occupazione, pendenza col corpo; poi il peso.

        force=True salta i veti e aggiunge comunque l'arco: SOLO per il tratto che il robot ha
        appena percorso con le proprie zampe, libero per costruzione.
        occ_blocked: esito gia' calcolato da _occupancy_blocked_many (None = lo calcola qui).
        fresh=True: le adiacenze sono appena state svuotate, non c'e' un arco vecchio da togliere.
        live_prof: profilo di pendenza gia' calcolato in blocco (precompute_arc_slopes).
        Restituisce True se l'arco e' stato aggiunto.
        """
        x1, y1 = self.nodes[idx1]
        x2, y2 = self.nodes[idx2]
        key = (min(idx1, idx2), max(idx1, idx2))

        long_slope, lat_slope, has_slope_data = 0.0, 0.0, False
        prof = (long_slope, lat_slope, has_slope_data)
        if not force:
            if key in self.tracker_blocked_edges:
                self.edge_validity[key] = False
                return False

            # Occupazione (scansione dal vivo e mappa globale).
            if global_map is not None and edge_safety_margin > 0.0:
                blocked_now = (bool(occ_blocked) if occ_blocked is not None else
                               self._occupancy_blocked(idx1, idx2, dist, global_map, edge_safety_margin))
                if blocked_now:
                    self._record_occupancy_veto(key)
                    self.edge_validity[key] = False
                    return False

            # Pendenza col corpo, in avanti o di lato. live_prof = profilo gia' calcolato in
            # blocco (precompute_arc_slopes).
            prof = (live_prof if live_prof is not None
                    else self.compute_arc_slope_profile(x1, y1, x2, y2, body=True))
            long_slope, lat_slope, has_slope_data = body_slope(prof)
            if has_slope_data:
                self.edge_slope_info[key] = (long_slope, lat_slope)  # per visualizzazione
                if self._record_slope_veto(key, long_slope, lat_slope):
                    self.edge_validity[key] = False
                    return False

        weight = self._compute_edge_weight(
            idx1, idx2, dist,
            precomputed_slope=(None if force else prof))

        if fresh:
            self.edges.setdefault(idx1, []).append((idx2, weight))
            self.edges.setdefault(idx2, []).append((idx1, weight))
        else:
            self.edges[idx1] = [(n, w) for n, w in self.edges.get(idx1, []) if n != idx2]
            self.edges[idx2] = [(n, w) for n, w in self.edges.get(idx2, []) if n != idx1]
            self.edges[idx1].append((idx2, weight))
            self.edges[idx2].append((idx1, weight))
        self.edge_validity[key] = True
        return True

    def set_allowed_nodes(self, node_ids=None):
        """
        Limita la ricerca del percorso a questi nodi (None = nessun limite). Partenza e arrivo
        sono sempre ammessi. Usato da mission_graph.restrict_planning_to_explored_cells.
        """
        self.allowed_nodes = None if node_ids is None else set(node_ids)

    def set_detour_limit(self, max_ratio: Optional[float] = None, slack_m: float = 0.0):
        """Tetto alla lunghezza dei percorsi rispetto alla linea d'aria (None = nessuno)."""
        self.max_detour_ratio = None if max_ratio is None else float(max_ratio)
        self.detour_slack_m = float(slack_m)

    def path_length(self, path_ids) -> float:
        """Lunghezza geometrica di un percorso, in metri."""
        if not path_ids or len(path_ids) < 2:
            return 0.0
        xy = np.array([self.nodes[i] for i in path_ids], dtype=np.float64)
        return float(np.sum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))))

    def anchor_node(self, x: float, y: float, global_map=None, edge_safety_margin: float = 0.0,
                    ignore_near_m: float = 0.0) -> Optional[int]:
        """
        Il nodo ESISTENTE a cui agganciare la posizione (x, y), senza crearne di nuovi.

        Non semplicemente il piu' vicino (dopo un arresto a meta' arco il piu' vicino puo' stare
        oltre cio' che ha fermato il robot): il piu' vicino, entro ANCHOR_SEARCH_M, che
          - non ha penalita' infinita;
          - e' raggiungibile in linea retta senza passare su celle occupate della mappa globale
            (i primi ignore_near_m attorno a (x, y) non contano: li' sta il robot).
        Se nessuno passa i controlli, il piu' vicino in assoluto: decide comunque il fronte.
        """
        if not self.nodes:
            return None
        ids = np.fromiter(self.nodes.keys(), dtype=np.int64, count=len(self.nodes))
        xy = np.array([self.nodes[int(i)] for i in ids], dtype=np.float64)
        d = np.hypot(xy[:, 0] - x, xy[:, 1] - y)
        order = np.argsort(d, kind='stable')
        for k in order:
            if d[k] > ANCHOR_SEARCH_M:
                break
            nid = int(ids[k])
            if not np.isfinite(self.node_penalty(nid)):
                continue
            if global_map is not None and edge_safety_margin > 0.0 and d[k] > 1e-6:
                nx, ny = xy[k]
                n_s = max(OCCUPANCY_SAMPLES_MIN, int(np.ceil(d[k] * OCCUPANCY_SAMPLES_PER_M)) + 1)
                blocked = False
                for j in range(n_s):
                    t = j / (n_s - 1)
                    sx, sy = x + t * (nx - x), y + t * (ny - y)
                    if np.hypot(sx - x, sy - y) <= ignore_near_m:
                        continue
                    if global_map.is_occupied(sx, sy, safety_margin=edge_safety_margin):
                        blocked = True
                        break
                if blocked:
                    continue
            return nid
        return int(ids[order[0]])

    def add_stop_node(self, x: float, y: float, global_map=None, edge_safety_margin: float = 0.0,
                      label: str = "sosta", is_robot: bool = True) -> int:
        """
        Il nodo ESISTENTE a cui agganciare la posizione (x, y) (vedi anchor_node): il grafo
        non cresce durante la missione. Se is_robot, diventa anche il nodo del robot.
        """
        nid = self.anchor_node(x, y, global_map, edge_safety_margin,
                               ignore_near_m=(ANCHOR_IGNORE_NEAR_M if is_robot else 0.0))
        if nid is None:
            raise RuntimeError("add_stop_node: il grafo non ha nodi")
        if is_robot:
            self.set_robot_node(nid)
        nx, ny = self.nodes[nid]
        print(f"[NODO] {label}: agganciato al nodo {nid} in ({nx:.2f}, {ny:.2f}), a "
              f"{float(np.hypot(nx - x, ny - y)):.2f} m da ({x:.2f}, {y:.2f}) -- nessun nodo nuovo")
        return nid

    def mark_edge_invalid(self, idx1: int, idx2: int):
        """
        Segna un arco come bloccato per il resto della missione: nessuna valutazione lo rimette.
        La chiama il ciclo principale dopo un blocco confermato DA VICINO dal fronte sicuro o un
        fallimento fisico del movimento.
        """
        key = (min(idx1, idx2), max(idx1, idx2))
        self.edge_validity[key] = False
        self.tracker_blocked_edges.add(key)   # per sempre: nessun refresh lo rimette
        self.traversed_edges.discard(key)     # percorso in passato non vuol dire percorribile ora

        if idx2 in [n[0] for n in self.edges.get(idx1, [])]:
            self.edges[idx1] = [(n, w) for n, w in self.edges[idx1] if n != idx2]
        if idx1 in [n[0] for n in self.edges.get(idx2, [])]:
            self.edges[idx2] = [(n, w) for n, w in self.edges[idx2] if n != idx1]

    def compute_path_cost(self, path_ids, allow_direct_first_hop: bool = False):
        """
        Somma dei pesi (con le penalita') lungo una sequenza di nodi.

        allow_direct_first_hop: il primo tratto, dal robot al primo waypoint, puo' non essere un
        arco del grafo (dopo una SCORCIATOIA il robot va dritto verso un nodo piu' avanti, su un
        tratto confermato libero dal fronte): lo si conta come alpha * distanza invece che
        come piano rotto.
        """
        if path_ids is None or len(path_ids) < 2:
            return 0.0
        total = 0.0
        for i in range(len(path_ids) - 1):
            a, b = path_ids[i], path_ids[i + 1]
            weight = None
            for n, w in self.edges.get(a, []):
                if n == b:
                    weight = w
                    break
            key = (min(a, b), max(a, b))
            if (weight is None and i == 0 and allow_direct_first_hop and a in self.nodes and b in self.nodes
                    and key not in self.tracker_blocked_edges and self.edge_validity.get(key) is not False):
                # Solo per un tratto che NON e' mai stato scartato: un arco appena tolto dal
                # grafo (occupazione, pendenza, pericolo, blocco) deve far ripianificare.
                (xa, ya), (xb, yb) = self.nodes[a], self.nodes[b]
                weight = self.alpha * float(np.hypot(xb - xa, yb - ya))
            if weight is None:
                return float('inf')  # edge no longer exists (e.g. invalidated)
            # Stessi costi usati da find_path_dijkstra: altrimenti il controllo
            # di stabilita' confronterebbe un piano penalizzato con uno non penalizzato.
            total += self.effective_weight(a, b, weight)
        return total

    def find_path_dijkstra(self, start_idx: int, goal_idx: int,
                            current_path_ids: Optional[List[int]] = None,
                            margin: float = 0.0,
                            max_edges: Optional[int] = None,
                            ignore_penalties: bool = False) -> Optional[List[int]]:
        """
        Percorso piu' corto fra due nodi (Dijkstra), dentro allowed_nodes e col tetto alla
        deviazione (set_detour_limit). None se non c'e'.

        Con current_path_ids (il piano che si sta seguendo, che parte da start_idx) e margin > 0,
        il percorso nuovo si restituisce solo se costa almeno `margin` in meno: evita di
        oscillare fra strade quasi equivalenti per il rumore dei pesi. Se tiene il piano vecchio
        restituisce current_path_ids invariato: per il chiamante e' "piano stabile", non "nessun
        percorso".
        """
        if start_idx not in self.nodes or goal_idx not in self.nodes:
            return None

        distances = {node: float('inf') for node in self.nodes.keys()}
        distances[start_idx] = 0
        parents = {node: None for node in self.nodes.keys()}

        pq = [(0, start_idx)]
        visited = set()
        new_path = None
        allowed = self.allowed_nodes

        while pq:
            current_dist, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)

            if current == goal_idx:
                path = []
                node = goal_idx
                while node is not None:
                    path.append(node)
                    node = parents[node]
                new_path = path[::-1]
                break

            for neighbor, weight in self.edges.get(current, []):
                if (allowed is not None and neighbor not in allowed
                        and neighbor != goal_idx):
                    continue          # fuori dalle celle ammesse (vedi set_allowed_nodes)
                if neighbor not in visited:
                    # Costo = geometrico + strato dei costi: un arco o un nodo impercorribile ha
                    # penalita' infinita e Dijkstra non lo attraversa.
                    w = weight if ignore_penalties else self.effective_weight(current, neighbor, weight)
                    if not np.isfinite(w):
                        continue
                    new_dist = current_dist + w
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        parents[neighbor] = current
                        heapq.heappush(pq, (new_dist, neighbor))

        # Tetto sul numero di archi, verificato sul percorso TROVATO: se il cammino di costo
        # minimo e' troppo lungo si risponde "nessun percorso" e si rinuncia alla cella.
        if new_path is not None and max_edges is not None and len(new_path) - 1 > max_edges:
            new_path = None

        # Tetto alla deviazione: un giro troppo lungo rispetto alla linea d'aria vale "nessun
        # percorso"; si rinuncia alla cella e la si riprova piu' tardi da un altro lato.
        if new_path is not None and self.max_detour_ratio is not None:
            (sx, sy), (gx, gy) = self.nodes[start_idx], self.nodes[goal_idx]
            limit = self.max_detour_ratio * float(np.hypot(gx - sx, gy - sy)) + self.detour_slack_m
            plen = self.path_length(new_path)
            if plen > limit:
                print(f"[DEVIAZIONE] Percorso {start_idx}->{goal_idx} lungo {plen:.1f} m, oltre "
                      f"il tetto di {limit:.1f} m: lo considero impraticabile.")
                new_path = None

        # --- Senza controllo di stabilita' ---
        if current_path_ids is None or margin <= 0.0:
            return new_path

        # --- Controllo di stabilita' ---
        current_cost = self.compute_path_cost(current_path_ids, allow_direct_first_hop=True)

        if new_path is None:
            return current_path_ids if current_cost != float('inf') else None

        if current_cost == float('inf'):
            return new_path  # current plan is broken (an edge got invalidated) -> must switch

        new_cost = self.compute_path_cost(new_path)
        if new_cost < current_cost * (1.0 - margin):
            return new_path

        return current_path_ids

    def reachable_count(self, start_idx: int, limit: int) -> int:
        """
        Quanti nodi si raggiungono da start_idx con le stesse regole di find_path_dijkstra
        (archi con costo finito, dentro allowed_nodes), fermandosi a `limit`. Serve a capire,
        quando un obiettivo non ha piu' un percorso, se e' isolato l'obiettivo o il robot.
        """
        if start_idx not in self.nodes:
            return 0
        allowed = self.allowed_nodes
        seen, stack = {start_idx}, [start_idx]
        while stack and len(seen) < limit:
            u = stack.pop()
            for v, w in self.edges.get(u, []):
                if v in seen or (allowed is not None and v not in allowed):
                    continue
                if not np.isfinite(self.effective_weight(u, v, w)):
                    continue
                seen.add(v)
                stack.append(v)
        return min(len(seen), limit)

    def get_node_position(self, node_idx: int) -> Optional[Tuple[float, float]]:
        """Posizione (x, y) del nodo, None se non esiste."""
        return self.nodes.get(node_idx)


# =========================================================================================
# FUNZIONI SUL GRAFO usate dal ciclo di navigazione
# =========================================================================================
def mark_edge_blocked_soft(prm, idx1, idx2):
    """
    Toglie un arco dal grafo ADESSO, senza condannarlo per sempre.

    prm_graph.mark_edge_invalid() lo mette anche in `tracker_blocked_edges`, che non viene
    MAI ricreato -- ne' dal refresh ne' dalla ricostruzione degli archi. Giusto per un
    blocco confermato da tre scansioni ferme; sbagliato per [SUBITO], che scarta dopo UNA
    scansione: un ostacolo marginale visto una volta isolerebbe un nodo per tutta la missione.

    Qui si tolgono solo l'arco e la sua validita': alla prossima rivalutazione, se i dati
    freschi dicono che e' libero, l'arco torna -- che e' esattamente il comportamento di
    rientro che refresh_local_edge_weights e' stato scritto per avere.
    """
    key = (min(idx1, idx2), max(idx1, idx2))
    # Scartare un arco cambia il piano, quindi al prossimo giro il refresh degli archi va
    # fatto (vedi REFRESH A EVENTI in easy_walk.TentativoCella._scansione_e_grafo).
    prm.needs_refresh = True
    prm.edge_validity[key] = False
    prm.traversed_edges.discard(key)
    prm.edges[idx1] = [(n, w) for n, w in prm.edges.get(idx1, []) if n != idx2]
    prm.edges[idx2] = [(n, w) for n, w in prm.edges.get(idx2, []) if n != idx1]
