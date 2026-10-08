"""
mission_graph.py -- il grafo di missione, REGOLARE e DETERMINISTICO.

Sostituisce il campionamento casuale di global_sampler.GlobalSampler (np.random.uniform,
5 punti/m^2, ~3125 nodi su 25x25 m, grafo diverso a ogni missione) con un RETICOLO
rettangolare di nodi equidistanziati, costruito in forma chiusa a partire dalla DIMENSIONE
DELLA MISSIONE -- (righe, colonne, lato_cella), gli stessi tre numeri di SPOT_MISSION_GRID
e di environmentMap.EnvironmentMap.

Due flag per l'utente: la SPAZIATURA fra i nodi e gli ANELLI di collegamento.

PERCHE' UN RETICOLO
  1. Riproducibilita'. Oggi non c'e' nessun seed: ogni missione parte da un grafo diverso,
     quindi mission_replay non puo' riprodurre le scelte del pianificatore e due missioni
     nella stessa stanza non sono confrontabili. Qui il grafo e' funzione solo dei
     parametri: stessi parametri, stesso grafo, bit per bit.
  2. Costa meno, non di piu'. A passo 0.714 m sono 1225 nodi su 625 m^2 contro i 3125
     attuali, e con due anelli ~4700 archi contro gli ~85600 da valutare oggi: 18 volte
     meno archi su cui calcolare occupazione e pendenza, che e' il costo vero di
     build_graph (~10 s per costruzione, 3-4 per missione).
  3. Geometria fuori dal costo. Con archi di due sole lunghezze, alpha*dist +
     beta*pendenza dice quello che intende dire; con punti casuali un giro piu' lungo puo'
     costare meno di uno dritto solo per dove sono caduti i punti.
  4. Niente appaiamento geometrico all'avvio ne' a ogni ricostruzione: le coppie sono
     quelle del reticolo, gia' pronte (vedi add_edges_to_prm).

PERCHE' SOLO RETTANGOLARE -- e perche' il triangolare e' stato scartato (2026-10-08)
  Il triangolare sembra la scelta naturale per "nodi equidistanziati": sei vicini alla
  stessa distanza, direzioni a 60 gradi, isotropo, percorsi piu' corti (detour massimo
  ~2.6% contro ~8% del rettangolare a 8 vicini). E' stato scartato per un motivo che vince
  su quelli: NON PUO' CONTENERE I CENTRI DELLE CELLE, che sono gli obiettivi
  dell'esplorazione e non devono spostarsi di un centimetro.
  Il triangolare ha passo s in x e s*sqrt(3)/2 fra le file; perche' i centri (multipli di
  cell_size in entrambe le direzioni) cadano su nodi servirebbe che cell_size sia multiplo
  sia di s sia di s*sqrt(3)/2, cioe' che 2n/sqrt(3) sia un intero pari. sqrt(3) e'
  irrazionale: non succede per nessun passo (provati n=4..11: 4.62, 5.77, 6.93, 8.08...).
  Le due vie d'uscita erano entrambe peggiori: spostare l'obiettivo sul nodo piu' vicino
  (fino a 50 cm: inaccettabile, e' il bersaglio della missione), oppure inserire il centro
  e togliere i nodi di reticolo troppo vicini -- che funziona, ma lascia il reticolo
  localmente irregolare attorno a 20 celle su 25, con archi da 0.57 a 1.44 m invece di due
  sole lunghezze. Nel rettangolare con passo cell_size/n i centri sono nodi per
  costruzione: zero nodi inseriti, zero togliti, zero eccezioni.
  Se un giorno si volesse riaprire la questione, il punto da attaccare e' questo, non
  l'isotropia.

SPAZIATURA
  Il passo richiesto viene SEMPRE arrotondato a cell_size/n con n intero: e' la condizione
  che rende ogni centro cella un nodo del reticolo. La riga [GRAFO] del log dice il passo
  effettivo. Default: DEFAULT_LATTICE_SPACING_M = 0.25 m (= 5/20).

  PASSO SOTTO min_edge_length (2026-10-08, missione 08-10 15:40). Il passo PUO' essere piu'
  corto di min_edge_length: i vicini troppo vicini semplicemente non si collegano, e gli
  archi partono dagli anelli piu' esterni (vedi ANELLI). La soglia sugli archi resta 0.5 m
  perche' serve alla pendenza (un arco troppo corto non dice nulla del terreno); a cosa
  serve il passo fitto e' un'altra cosa: avere SEMPRE una fila di nodi dentro un corridoio.
  Nella missione del 08-10 15:40 il corridoio di partenza aveva ~45 cm di linee centrali
  ammesse (0.30 m di spazio per lato) e le due file del reticolo a passo 0.5 cadevano
  entrambe fuori: il corridoio nel grafo non esisteva. A passo 0.25 ogni striscia larga
  piu' di 0.25 m contiene una fila.

ANELLI -- quante connessioni (ridefiniti il 2026-10-08)
  L'anello k e' il quadrato di nodi a k passi di indice (anello 1 = gli 8 intorno, anello 2
  = i 16 del giro dopo, ...). `rings` = k collega ogni nodo a tutti i nodi entro la
  diagonale dell'anello k, cioe' entro k*s*sqrt(2), purche' l'arco sia lungo almeno
  min_edge_length. Il limite e' un cerchio e non il quadrato per coerenza con
  PRM.refresh_local_edge_weights, che attorno al robot ricrea le coppie per distanza.
  Con passo 0.25 e min_edge 0.5:
    rings=2 -> archi da 0.50 (dritto), 0.56 (la "mossa del cavallo", 27 gradi) e 0.71 m
               (diagonale): 16 direzioni invece di 8, 16 vicini per nodo;
    rings=3 -> in piu' 0.75, 0.79, 0.90, 1.00, 1.03, 1.06 m: tratti piu' lunghi, ~2.5
               volte gli archi.
  I tratti lunghi in rettilineo li produce comunque easy_walk.shortcut_index, che salta i
  nodi intermedi fino a 2.5 m quando il FRONTE SICURO conferma il tratto sui dati dal vivo.
  rings=None arriva fino a min(connection_radius, max_edge_length).

ANCORAGGIO
  Il reticolo e' ancorato all'ORIGINE, che per come e' scritta environmentMap e' insieme la
  posa di accensione del robot E il centro della cella di partenza
  (get_world_position_from_cell: "Cell (0,0) center is at the origin"). Quindi c'e' un nodo
  esattamente sotto il robot all'avvio -- niente "boot node" da aggiungere a mano, vedi
  origin_node_id. Gli indici (i, j) possono essere negativi; il rettangolo li ritaglia.

ALIASING -- il limite da conoscere
  Un reticolo puo' perdere un varco che il caso trovava: se un corridoio cade fra due file
  di nodi, nel grafo non esiste. E' l'unico modo in cui la regolarita' e' peggiore del
  caso, ed e' il motivo per cui la spaziatura e' un parametro. Riferimento: il fronte
  sicuro pretende spotGrid.FRONTIER_CLEARANCE_M = 0.30 m di obstacle_distance dalla linea
  centrale, quindi un corridoio utile e' largo almeno 0.60 m. Le linee centrali ammesse
  sono una striscia larga (larghezza - 0.60 m): perche' ci cada sempre una fila di nodi il
  passo deve essere piu' corto di quella striscia. Missione del 08-10 15:40: striscia di
  ~45 cm, file a passo 0.5 entrambe fuori, corridoio inesistente nel grafo; a passo 0.25
  (default dal 2026-10-08) ci sono due file dentro.

COME SI PASSA A SPOT
      mg = mission_graph.build_mission_graph(env=env)   # ~30 ms: nodi + adiacenza
      mg.into_prm(prm)                                  # i nodi, id 0..N-1
      gb_sampler = mg.as_sampler()                      # al posto di GlobalSampler
      ...
      mg.add_edges_to_prm(prm, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
                                                        # al posto di prm.build_graph()
  add_edges_to_prm chiama lo stesso PRM._evaluate_and_add_edge di build_graph, quindi veti
  e pesi sono identici a oggi; quello che salta e' l'appaiamento geometrico. Cio' che resta
  da calcolare sul robot sono i veti e i pesi, che per costruzione non sono precalcolabili:
  l'occupazione viene dalla mappa globale che si riempie strada facendo, la pendenza d'arco
  dal terreno visto in quel momento. E' esattamente "Spot ci aggiunge solo i dati locali".
  save_npz/load_npz servono ad avere un reticolo IDENTICO fra piu' missioni, non per la
  velocita': misurato, costruire da zero costa ~30 ms e ricaricare ~20 ms.

Il modulo dipende SOLO da numpy: si importa e si prova fuori dal robot, senza l'SDK.
environmentMap, prm_graph e matplotlib sono importati dentro le funzioni che li usano.

Da riga di comando:
    python3 mission_graph.py --confronto
    python3 mission_graph.py --spacing 0.25 --rings 2 --preview p.png --zoom -2 2 -2 2
    python3 mission_graph.py --rows 2 --cols 3 --cell 3
    python3 mission_graph.py --save reticolo_2x4x5.npz
    python3 mission_graph.py --tempi
"""

import math
from collections import deque

import numpy as np

# ----------------------------------------------------------------------------------------
# La flag di default sugli anelli. La spaziatura, per default, la deriva
# suggested_spacing() dai parametri del PRM e dal lato della cella.
# ----------------------------------------------------------------------------------------
LATTICE_RINGS = 2

# La missione e il passo di default, in UN solo posto: easy_walk li importa da qui, quindi
# la missione sul robot e le prove da riga di comando (python3 mission_graph.py) partono
# dagli stessi numeri. Per cambiarli per una sola missione: argomenti di easy_walk.py
# (--rows --cols --cell --spacing --rings) o SPOT_MISSION_GRID="righe,colonne,lato".
#   2 x 4 celle da 5 m = 10 m a sinistra x 20 m davanti al robot (colonne in avanti).
#   Passo 0.25 m = 5/20 con archi da 0.5 m in su (vedi SPAZIATURA e ANELLI): 3321 nodi,
#   ~26 mila archi sulla 2x4 con rings=2.
DEFAULT_MISSION_ROWS = 2
DEFAULT_MISSION_COLS = 4
DEFAULT_CELL_SIZE_M = 5.0
DEFAULT_LATTICE_SPACING_M = 0.25

# Stessi default del PRM in uso (prm_graph.PRM(min_edge_length=0.5, max_edge_length=2,
# connection_radius=3)). Passati come argomenti, non importati, per non trascinarsi dietro
# l'SDK: se li cambi nel PRM, cambiali anche nella chiamata.
DEFAULT_MIN_EDGE_M = 0.5
DEFAULT_MAX_EDGE_M = 2.0
DEFAULT_CONNECTION_RADIUS_M = 3.0


def covering_radius(spacing):
    """
    Distanza massima fra un punto QUALSIASI dell'area e il nodo piu' vicino: mezza diagonale
    della maglia. E' il numero che conta per l'aliasing.
    """
    return spacing / math.sqrt(2.0)


def node_density(spacing):
    """Nodi per metro quadro (per confronto con i 5/m^2 del campionamento casuale)."""
    return 1.0 / spacing ** 2


def align_spacing(spacing_m, cell_size_m):
    """
    Il passo arrotondato a cell_size/n con n intero: la condizione che rende ogni centro
    cella un nodo del reticolo. Restituisce (passo, n).
    """
    n = max(1, int(round(float(cell_size_m) / float(spacing_m))))
    return float(cell_size_m) / n, n


def suggested_spacing(cell_size_m=5.0, min_edge_length=DEFAULT_MIN_EDGE_M):
    """
    Il passo allineato piu' grande la cui copertura non supera min_edge_length. Con celle da
    5 m e min_edge 0.5 m da' cell_size/7 = 0.714 m (copertura 0.505). Se cambia
    min_edge_length nel PRM, il passo consigliato si ricalcola da solo.
    """
    n = max(1, int(math.ceil(float(cell_size_m) / (float(min_edge_length) * math.sqrt(2.0)))))
    return float(cell_size_m) / n


def ring_distances(spacing, n_rings=6):
    """
    Le distanze dei primi n_rings gusci di vicini: s, s*sqrt(2), 2s, s*sqrt(5), 2s*sqrt(2)...
    Calcolate dalla geometria vera, non da una tabella scritta a mano.
    """
    w = int(n_rings) + 2
    d = {round(spacing * math.hypot(di, dj), 6)
         for di in range(-w, w + 1) for dj in range(-w, w + 1) if di or dj}
    return sorted(d)[:int(n_rings)]


def _mission_extent(rows, cols, cell_size_m, start_cell):
    """
    Rettangolo della missione nel FRAME DELLA GRIGLIA (x = colonne, y = righe), con il
    centro della cella di partenza nell'origine -- la stessa convenzione di
    EnvironmentMap.get_world_position_from_cell. I bordi stanno a mezza cella dai centri
    estremi, come i confini usati da get_cell_from_world (round sulle celle).
    """
    start_row, start_col = start_cell
    half = cell_size_m / 2.0
    return ((0 - start_col) * cell_size_m - half,
            (cols - 1 - start_col) * cell_size_m + half,
            (0 - start_row) * cell_size_m - half,
            (rows - 1 - start_row) * cell_size_m + half)


def _lattice_points(spacing, extent, border_margin_m):
    """
    I nodi del reticolo dentro il rettangolo, ancorati all'origine. Restituisce
    (ij, xy_local): indici interi e coordinate nel frame della griglia. Ordine
    deterministico: fila crescente, poi colonna.
    """
    x_min, x_max, y_min, y_max = extent
    x_min, x_max = x_min + border_margin_m, x_max - border_margin_m
    y_min, y_max = y_min + border_margin_m, y_max - border_margin_m
    eps = 1e-9
    i_lo, i_hi = (int(math.ceil(y_min / spacing - eps)), int(math.floor(y_max / spacing + eps)))
    j_lo, j_hi = (int(math.ceil(x_min / spacing - eps)), int(math.floor(x_max / spacing + eps)))
    ij = [(i, j) for i in range(i_lo, i_hi + 1) for j in range(j_lo, j_hi + 1)]
    xy = [(j * spacing, i * spacing) for i, j in ij]
    return (np.array(ij, dtype=np.int64).reshape(-1, 2),
            np.array(xy, dtype=np.float64).reshape(-1, 2))


def _lattice_pairs(spacing, ij, xy, min_edge_m, reach_m):
    """
    Le coppie di nodi che diventeranno archi, con la loro lunghezza. Sfrutta il fatto che i
    nodi stanno su indici interi: i vicini si trovano scorrendo una finestra di offset
    (O(N*k) di aritmetica intera) invece di interrogare un albero k-d su tutte le coppie,
    che e' quello che fa PRM.build_graph a ogni ricostruzione.

    Restituisce (pairs, lengths, n_sotto_soglia): le coppie ammesse, le lunghezze, e quante
    coppie sono state scartate perche' piu' corte di min_edge_m. Con un passo allineato e
    >= min_edge_m quel numero e' ZERO: se non lo e', il primo anello sta finendo sotto la
    soglia del PRM.
    """
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(ij)}
    w = int(math.ceil(reach_m / spacing))
    pairs, lengths, n_short = [], [], 0
    reach2, min2 = reach_m ** 2 + 1e-12, min_edge_m ** 2 - 1e-12
    for k, (i, j) in enumerate(ij):
        x0, y0 = xy[k]
        for di in range(-w, w + 1):
            for dj in range(-w, w + 1):
                if di == 0 and dj == 0:
                    continue
                other = index.get((int(i) + di, int(j) + dj))
                if other is None or other <= k:          # ogni coppia una volta sola
                    continue
                dx, dy = xy[other, 0] - x0, xy[other, 1] - y0
                d2 = dx * dx + dy * dy
                if d2 > reach2:
                    continue
                if d2 < min2:
                    n_short += 1
                    continue
                pairs.append((k, other))
                lengths.append(math.sqrt(d2))
    if not pairs:
        return np.zeros((0, 2), np.int64), np.zeros(0), n_short
    pairs = np.array(pairs, dtype=np.int64)
    lengths = np.array(lengths, dtype=np.float64)
    order = np.lexsort((pairs[:, 1], pairs[:, 0]))
    return pairs[order], lengths[order], n_short


def _connected_components(n_nodes, pairs):
    """(componenti, dimensione della piu' grande, nodi isolati). Solo geometria: dice se il
    grafo CONSEGNATO e' connesso, prima che i veti del terreno lo assottiglino."""
    adj = [[] for _ in range(n_nodes)]
    for a, b in pairs:
        adj[int(a)].append(int(b))
        adj[int(b)].append(int(a))
    seen = np.zeros(n_nodes, dtype=bool)
    sizes = []
    for s in range(n_nodes):
        if seen[s]:
            continue
        seen[s] = True
        q, size = deque([s]), 0
        while q:
            u = q.popleft()
            size += 1
            for v in adj[u]:
                if not seen[v]:
                    seen[v] = True
                    q.append(v)
        sizes.append(size)
    return len(sizes), (max(sizes) if sizes else 0), int(sum(1 for a in adj if not a))


# ========================================================================================
# IL GRAFO DI MISSIONE
# ========================================================================================

class MissionGraph:
    """
    Il grafo pronto: nodi, adiacenza geometrica con le lunghezze, e i ganci per caricarlo
    nel PRM. NON contiene pesi ne' veti: quelli dipendono dal terreno e li calcola il PRM
    sui dati locali (vedi add_edges_to_prm).
    """

    def __init__(self, spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                 ij, xy_local, pairs, lengths, node_cell, cell_center_ids, origin_node_id,
                 min_edge_m, reach_m, rings, n_short_pairs):
        self.spacing_m = float(spacing_m)
        self.rows, self.cols = int(rows), int(cols)
        self.cell_size_m = float(cell_size_m)
        self.start_cell = tuple(int(v) for v in start_cell)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.origin_yaw = float(origin_yaw)
        self.min_edge_m, self.reach_m = float(min_edge_m), float(reach_m)
        self.rings = rings

        self.ij = ij                    # (N, 2) indici interi del reticolo
        self.xy_local = xy_local        # (N, 2) frame della griglia
        self.pairs = pairs              # (E, 2) indici di nodo, ordinate
        self.lengths = lengths          # (E,)   metri
        self.node_cell = node_cell      # (N, 2) cella (riga, colonna) di ogni nodo
        self.cell_center_ids = cell_center_ids    # {(riga, col): id del nodo nel centro}
        self.origin_node_id = origin_node_id      # nodo sotto il robot all'avvio
        self.n_short_pairs = int(n_short_pairs)

        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        self.xy = np.column_stack((
            self.origin_xy[0] + xy_local[:, 0] * c - xy_local[:, 1] * s,
            self.origin_xy[1] + xy_local[:, 0] * s + xy_local[:, 1] * c,
        ))

        deg = np.zeros(len(xy_local), dtype=np.int64)
        for a, b in self.pairs:
            deg[a] += 1
            deg[b] += 1
        self.degrees = deg
        (self.n_components, self.biggest_component,
         self.n_isolated) = _connected_components(len(xy_local), self.pairs)

    # -- proprieta' --------------------------------------------------------------------
    @property
    def n_nodes(self):
        return int(len(self.xy))

    @property
    def n_edges(self):
        return int(len(self.pairs))

    @property
    def area_m2(self):
        return self.rows * self.cols * self.cell_size_m ** 2

    @property
    def is_connected(self):
        return self.n_components == 1

    @property
    def covering_m(self):
        return covering_radius(self.spacing_m)

    def node_ids(self):
        """Gli id dei nodi, 0..N-1: gli stessi che useranno PRM.nodes."""
        return np.arange(self.n_nodes, dtype=np.int64)

    def target_for_cell(self, row, col):
        """
        (x, y) in coordinate mondo del nodo che sta nel centro geometrico della cella --
        esatto per costruzione. (None, None) se la cella non e' nella griglia.
        """
        nid = self.cell_center_ids.get((int(row), int(col)))
        if nid is None:
            return None, None
        return float(self.xy[nid, 0]), float(self.xy[nid, 1])

    # -- aggancio al PRM ---------------------------------------------------------------
    def into_prm(self, prm):
        """
        Mette i NODI nel PRM (prm.add_node), con gli id 0..N-1. Nessun arco: gli archi
        arrivano da add_edges_to_prm, che ha bisogno dei dati di terreno.
        """
        for k in range(self.n_nodes):
            prm.add_node(int(k), float(self.xy[k, 0]), float(self.xy[k, 1]))
        return self.n_nodes

    def add_edges_to_prm(self, prm, global_map=None, edge_safety_margin=0.0, verbose=True):
        """
        Valuta le coppie GIA' CALCOLATE e aggiunge quelle ammesse, usando la stessa funzione
        di build_graph (PRM._evaluate_and_add_edge): identici veti -- archi bloccati,
        occupazione nella mappa globale, pendenza longitudinale e laterale -- e identici
        pesi. Quello che non si rifa' e' l'appaiamento geometrico.

        Da chiamare al posto di prm.build_graph(...) quando i nodi vengono da qui.
        """
        for idx in prm.nodes:
            prm.edges[idx] = []
        prm.edge_validity = {}
        prm.robot_node = None          # come build_graph: il permesso vale per il nodo attuale
        if hasattr(prm, '_sync_global_slope_cache'):
            prm._sync_global_slope_cache()

        added = 0
        for (a, b), d in zip(self.pairs, self.lengths):
            a, b = int(a), int(b)
            if a in prm.nodes and b in prm.nodes:
                if prm._evaluate_and_add_edge(a, b, float(d), global_map, edge_safety_margin):
                    added += 1

        # Gli archi percorsi dal robot sono liberi per costruzione e vanno rimessi comunque,
        # come fa build_graph: i nodi di sosta stanno spesso sotto min_edge_length.
        for a, b in list(getattr(prm, 'traversed_edges', ())):
            if a in prm.nodes and b in prm.nodes:
                if hasattr(prm, '_traversed_edge_ok') and not prm._traversed_edge_ok(a, b, global_map):
                    prm.traversed_edges.discard((a, b))
                    continue
                (xa, ya), (xb, yb) = prm.nodes[a], prm.nodes[b]
                prm._evaluate_and_add_edge(a, b, float(math.hypot(xb - xa, yb - ya)), force=True)
        prm.built = True
        if verbose:
            print(f"[GRAFO] {added} archi ammessi su {self.n_edges} coppie del reticolo "
                  f"({self.n_nodes} nodi, passo {self.spacing_m:.4f} m)")
        return added

    def as_sampler(self):
        """
        Oggetto con la stessa interfaccia di global_sampler.GlobalSampler, da passare a
        find_best_point_in_cell senza toccarla: get_all_points, get_point_in_cell,
        get_points_in_cell, get_nearest_points, global_points, point_cell_map. La ricerca
        per cella e' a indice (O(1)) invece di scorrere tutti i punti a ogni chiamata.
        """
        return LatticeSampler(self)

    # -- persistenza e diagnostica -----------------------------------------------------
    def save_npz(self, path):
        """
        Salva il reticolo: indici, coordinate LOCALI, coppie, lunghezze, parametri. Le
        coordinate locali non dipendono dalla posa del robot, quindi il file vale per
        qualunque missione di quella dimensione: si ricarica con load_npz e si applica solo
        la rototraslazione. Serve ad avere un reticolo identico fra piu' missioni; per la
        velocita' non serve (costruire costa ~30 ms, ricaricare ~20).
        """
        keys = np.array(list(self.cell_center_ids.keys()), dtype=np.int64).reshape(-1, 2)
        vals = np.array(list(self.cell_center_ids.values()), dtype=np.int64)
        np.savez_compressed(
            path, ij=self.ij, xy_local=self.xy_local, pairs=self.pairs, lengths=self.lengths,
            node_cell=self.node_cell, spacing_m=self.spacing_m, rows=self.rows, cols=self.cols,
            cell_size_m=self.cell_size_m, start_cell=np.array(self.start_cell),
            min_edge_m=self.min_edge_m, reach_m=self.reach_m,
            rings=(-1 if self.rings is None else self.rings),
            origin_node_id=(-1 if self.origin_node_id is None else self.origin_node_id),
            cell_center_keys=keys, cell_center_vals=vals, n_short_pairs=self.n_short_pairs)
        return path

    def report(self):
        """Il riepilogo da stampare nel log, accanto alle righe [CONFIG]."""
        lo = float(self.lengths.min()) if self.n_edges else float('nan')
        hi = float(self.lengths.max()) if self.n_edges else float('nan')
        uniq = sorted(set(np.round(self.lengths, 3).tolist()))
        rings = ", ".join(f"{v:.2f}" for v in uniq[:6]) + (" ..." if len(uniq) > 6 else "")
        n_div = int(round(self.cell_size_m / self.spacing_m))
        out = [
            f"[GRAFO] Missione {self.rows}x{self.cols} celle da {self.cell_size_m:.1f} m "
            f"= {self.area_m2:.0f} m2; reticolo rettangolare, passo {self.spacing_m:.4f} m "
            f"(= {self.cell_size_m:.1f}/{n_div}), anelli "
            f"{'tutti entro %.2f m' % self.reach_m if self.rings is None else self.rings}",
            f"[GRAFO] {self.n_nodes} nodi, {node_density(self.spacing_m):.2f} nodi/m2, "
            f"{self.n_edges} archi, grado medio {self.degrees.mean():.1f} "
            f"(min {int(self.degrees.min()) if self.n_nodes else 0}, "
            f"max {int(self.degrees.max()) if self.n_nodes else 0})",
            f"[GRAFO] Lunghezze degli archi {lo:.2f}-{hi:.2f} m [{rings}], "
            f"min_edge_length={self.min_edge_m:.2f} m, "
            f"{self.n_short_pairs} coppie di vicini troppo vicini, non collegate",
            f"[GRAFO] Copertura {self.covering_m:.2f} m (un corridoio utile al fronte e' "
            f"largo almeno 0.60 m)",
            f"[GRAFO] Obiettivi di cella: {len(self.cell_center_ids)} su "
            f"{self.rows * self.cols}, tutti nel centro geometrico esatto",
            f"[GRAFO] Connessione geometrica: "
            + ("UNICA componente, nessun nodo isolato"
               if self.is_connected and self.n_isolated == 0 else
               f"{self.n_components} componenti, la piu' grande con {self.biggest_component} "
               f"nodi su {self.n_nodes}, {self.n_isolated} isolati"),
        ]
        if self.origin_node_id is not None:
            out.append(f"[GRAFO] Accensione = centro della cella di partenza = nodo "
                       f"{self.origin_node_id} (nessun nodo di boot da aggiungere)")
        if not self.is_connected or self.n_isolated:
            out.append(f"[GRAFO] ATTENZIONE: il reticolo non e' connesso con questi parametri "
                       f"(passo {self.spacing_m:.4f} m, archi {self.min_edge_m:.2f}-"
                       f"{self.reach_m:.2f} m): aumenta gli anelli.")
        return "\n".join(out)

    def preview_png(self, path, show_cells=True, zoom=None):
        """
        Disegna il reticolo e i suoi archi in un PNG, per guardarlo prima di uscire.
        zoom: (x_min, x_max, y_min, y_max) nel frame della griglia, per i dettagli.
        """
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        fig, ax = plt.subplots(figsize=(9.5, 9.5))
        segs = [[(self.xy_local[a, 0], self.xy_local[a, 1]),
                 (self.xy_local[b, 0], self.xy_local[b, 1])] for a, b in self.pairs]
        ax.add_collection(LineCollection(segs, colors='0.78', linewidths=0.45, zorder=1))
        ax.plot(self.xy_local[:, 0], self.xy_local[:, 1], 'k.', ms=2.6, zorder=3)
        ids = sorted(set(self.cell_center_ids.values()))
        if ids:
            ax.plot(self.xy_local[ids, 0], self.xy_local[ids, 1], 'o', ms=7, mfc='none',
                    mec='tab:orange', mew=1.4, zorder=4, label='obiettivo di cella')
        if self.origin_node_id is not None:
            ax.plot(self.xy_local[self.origin_node_id, 0], self.xy_local[self.origin_node_id, 1],
                    'bo', ms=9, zorder=5, label='accensione / centro cella di partenza')
        if show_cells:
            half = self.cell_size_m / 2.0
            for r in range(self.rows):
                for c in range(self.cols):
                    cx = (c - self.start_cell[1]) * self.cell_size_m
                    cy = (r - self.start_cell[0]) * self.cell_size_m
                    ax.add_patch(plt.Rectangle((cx - half, cy - half), self.cell_size_m,
                                               self.cell_size_m, fill=False, ec='tab:green',
                                               ls='--', lw=1.0, alpha=0.65, zorder=2))
                    ax.text(cx, cy - half + 0.35, f"{r},{c}", ha='center', va='bottom',
                            fontsize=7, color='tab:green', zorder=6)
        if zoom:
            ax.set_xlim(zoom[0], zoom[1])
            ax.set_ylim(zoom[2], zoom[3])
        ax.set_aspect('equal', 'box')
        ax.set_xlabel('x griglia [m] -- colonne, davanti al robot')
        ax.set_ylabel('y griglia [m] -- righe, a sinistra del robot')
        ax.set_title(f"passo {self.spacing_m:.4f} m, anelli "
                     f"{self.rings if self.rings is not None else 'tutti'} -- "
                     f"{self.n_nodes} nodi, {self.n_edges} archi, "
                     f"copertura {self.covering_m:.2f} m")
        ax.legend(loc='upper right', fontsize=8)
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
        return path


class LatticeSampler:
    """Interfaccia GlobalSampler sopra un MissionGraph (vedi MissionGraph.as_sampler)."""

    def __init__(self, mg):
        self._mg = mg
        self.global_points = [(float(x), float(y)) for x, y in mg.xy]
        self.point_cell_map = {k: (int(r), int(c)) for k, (r, c) in enumerate(mg.node_cell)}
        self.point_density = node_density(mg.spacing_m)
        self.sampled = True
        self._by_cell = {}
        for k, cell in self.point_cell_map.items():
            self._by_cell.setdefault(cell, []).append(k)

    def sample_global_grid(self):
        """Il reticolo e' gia' costruito: non c'e' nulla da campionare."""
        return self.global_points

    def get_all_points(self):
        return self.global_points

    def get_point_in_cell(self, row, col):
        return [self.global_points[k] for k in self._by_cell.get((int(row), int(col)), ())]

    def get_points_in_cell(self, row, col):
        return [(k, self.global_points[k]) for k in self._by_cell.get((int(row), int(col)), ())]

    def get_nearest_points(self, x, y, k=5):
        d = np.hypot(self._mg.xy[:, 0] - x, self._mg.xy[:, 1] - y)
        order = np.argsort(d)[:max(0, int(k))]
        return [(int(i), float(d[i]), self.global_points[int(i)]) for i in order]

    def update_density(self, point_density):
        raise NotImplementedError(
            "la densita' di un reticolo si cambia dal passo: ricostruisci con "
            "build_mission_graph(spacing_m=...)")


# ========================================================================================
# LA FUNZIONE DA CHIAMARE
# ========================================================================================

def build_mission_graph(rows=DEFAULT_MISSION_ROWS, cols=DEFAULT_MISSION_COLS,
                        cell_size_m=DEFAULT_CELL_SIZE_M, spacing_m=DEFAULT_LATTICE_SPACING_M,
                        rings=LATTICE_RINGS,
                        origin_xy=(0.0, 0.0), origin_yaw=0.0, start_cell=(0, 0),
                        min_edge_length=DEFAULT_MIN_EDGE_M,
                        max_edge_length=DEFAULT_MAX_EDGE_M,
                        connection_radius=DEFAULT_CONNECTION_RADIUS_M,
                        border_margin_m=0.0, env=None, verbose=True):
    """
    Costruisce il grafo di missione.

    Args:
        rows, cols, cell_size_m: la DIMENSIONE DELLA MISSIONE (default DEFAULT_MISSION_*,
            2 x 4 celle da 5 m, gli stessi di easy_walk). Ignorati se si passa `env`.
        spacing_m: FLAG DELLA SPAZIATURA, in metri; viene arrotondata a cell_size/n (vedi
            align_spacing). Puo' essere sotto min_edge_length (vedi SPAZIATURA). Default
            DEFAULT_LATTICE_SPACING_M (0.25 m); None = suggested_spacing().
        rings: FLAG DELLE CONNESSIONI -- k = tutti i nodi entro la diagonale del k-esimo
            anello (k*passo*sqrt(2)), con archi >= min_edge_length (vedi ANELLI). None =
            tutti quelli entro min(connection_radius, max_edge_length).
        origin_xy, origin_yaw: posa del robot all'avvio nel frame VISION. Il reticolo e'
            ancorato qui. Ignorati se si passa `env`.
        start_cell: cella in cui il robot si accende (come env.start_cell).
        min_edge_length, max_edge_length, connection_radius: gli stessi parametri del PRM
            che consumera' il grafo -- devono coincidere con quelli passati a PRM().
        border_margin_m: margine da lasciare dentro il bordo della missione.
        env: una EnvironmentMap gia' creata e con set_origin() fatto. Se passata, dimensione
            e posa vengono da lei: e' il modo di garantire che grafo e mappa delle celle non
            possano andare fuori sincrono.

    Returns:
        MissionGraph.
    """
    if env is not None:
        rows, cols, cell_size_m = env.rows, env.cols, env.cell_size
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
        start_cell = tuple(env.start_cell)
    rows, cols, cell_size_m = int(rows), int(cols), float(cell_size_m)
    if rows < 1 or cols < 1 or cell_size_m <= 0:
        raise ValueError("dimensione della missione non valida")

    if spacing_m is None:
        spacing_m = suggested_spacing(cell_size_m, min_edge_length)
    if float(spacing_m) <= 0:
        raise ValueError("la spaziatura deve essere positiva")
    requested = float(spacing_m)
    spacing_m, n_div = align_spacing(requested, cell_size_m)
    if abs(spacing_m - requested) > 1e-12 and verbose:
        print(f"[GRAFO] Passo allineato al lato cella: {requested:.3f} -> {spacing_m:.4f} m "
              f"(= {cell_size_m:.1f}/{n_div}); e' la condizione per cui ogni centro cella "
              f"e' un nodo del reticolo.")

    prm_reach = min(float(connection_radius), float(max_edge_length))
    if rings is None:
        reach_m = prm_reach
    else:
        rings = int(rings)
        if rings < 1:
            raise ValueError("rings deve essere almeno 1")
        ring_reach = rings * spacing_m * math.sqrt(2.0)      # diagonale dell'anello k
        if ring_reach > prm_reach + 1e-9 and verbose:
            print(f"[GRAFO] {rings} anelli arriverebbero a {ring_reach:.2f} m, oltre la "
                  f"portata del PRM {prm_reach:.2f} m: mi fermo li'.")
        # Tolleranza relativa (1e-6): le distanze del reticolo sono multipli irrazionali del
        # passo, e senza tolleranza la diagonale dell'anello potrebbe cadere fuori per un ulp.
        reach_m = min(ring_reach, prm_reach) * (1.0 + 1e-6)
        if reach_m < float(min_edge_length):
            raise ValueError(
                f"con passo {spacing_m:.4f} m e {rings} anelli gli archi arrivano a "
                f"{reach_m:.2f} m, sotto min_edge_length {float(min_edge_length):.2f} m: "
                f"nessun arco sarebbe ammesso. Aumenta gli anelli o il passo.")

    extent = _mission_extent(rows, cols, cell_size_m, start_cell)
    ij, xy_local = _lattice_points(spacing_m, extent, float(border_margin_m))
    if len(xy_local) == 0:
        raise ValueError("nessun nodo nel rettangolo di missione: passo troppo grande "
                         "o margine di bordo troppo largo")
    pairs, lengths, n_short = _lattice_pairs(spacing_m, ij, xy_local,
                                             float(min_edge_length), reach_m)

    # --- I centri cella sono nodi del reticolo per costruzione (passo = cell_size/n):
    # qui si verifica che sia davvero cosi' invece di darlo per buono, perche' e' la
    # proprieta' su cui si regge tutta questa scelta di progetto.
    cell_center_ids, tol = {}, max(1e-9, spacing_m * 1e-9)
    by_index = {(int(a), int(b)): k for k, (a, b) in enumerate(ij)}
    for r in range(rows):
        for c in range(cols):
            j = (c - start_cell[1]) * n_div
            i = (r - start_cell[0]) * n_div
            k = by_index.get((i, j))
            if k is None:
                continue                      # cella fuori dal reticolo (margine di bordo)
            if abs(xy_local[k, 0] - (c - start_cell[1]) * cell_size_m) > tol or \
                    abs(xy_local[k, 1] - (r - start_cell[0]) * cell_size_m) > tol:
                raise AssertionError(
                    f"il nodo {k} dovrebbe essere il centro della cella ({r},{c}) ma non lo "
                    f"e': passo {spacing_m} non allineato a cell_size {cell_size_m}")
            cell_center_ids[(r, c)] = k
    if verbose and len(cell_center_ids) < rows * cols:
        print(f"[GRAFO] {rows * cols - len(cell_center_ids)} centri cella fuori dal reticolo "
              f"(margine di bordo {border_margin_m:.2f} m): quelle celle non hanno obiettivo.")

    # Nodo nell'origine: c'e' per costruzione (il reticolo e' ancorato la', e l'origine e'
    # anche il centro della cella di partenza), ma lo cerchiamo invece di assumerlo.
    d0 = np.hypot(xy_local[:, 0], xy_local[:, 1])
    origin_node_id = int(np.argmin(d0)) if d0.min() < 1e-9 else None

    # Cella di appartenenza di ogni nodo: stessa regola di env.get_cell_from_world (round).
    cols_idx = np.clip(np.round(xy_local[:, 0] / cell_size_m).astype(np.int64) + start_cell[1],
                       0, cols - 1)
    rows_idx = np.clip(np.round(xy_local[:, 1] / cell_size_m).astype(np.int64) + start_cell[0],
                       0, rows - 1)
    node_cell = np.column_stack((rows_idx, cols_idx))

    mg = MissionGraph(spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                      ij, xy_local, pairs, lengths, node_cell, cell_center_ids,
                      origin_node_id, float(min_edge_length), reach_m, rings, n_short)
    if verbose:
        print(mg.report())
    return mg


def load_npz(path, origin_xy=(0.0, 0.0), origin_yaw=0.0, env=None):
    """
    Ricarica un reticolo salvato con save_npz e lo riporta nel frame VISION con la posa
    passata (o quella di `env`): si applica solo la rototraslazione.
    """
    d = np.load(path, allow_pickle=False)
    if env is not None:
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
    keys = d['cell_center_keys'].reshape(-1, 2)
    vals = d['cell_center_vals'].reshape(-1)
    cc_ids = {(int(r), int(c)): int(v) for (r, c), v in zip(keys, vals)}
    oid, rings = int(d['origin_node_id']), int(d['rings'])
    return MissionGraph(
        float(d['spacing_m']), int(d['rows']), int(d['cols']), float(d['cell_size_m']),
        tuple(int(v) for v in d['start_cell']), origin_xy, origin_yaw, d['ij'], d['xy_local'],
        d['pairs'], d['lengths'], d['node_cell'], cc_ids, None if oid < 0 else oid,
        float(d['min_edge_m']), float(d['reach_m']), None if rings < 0 else rings,
        int(d['n_short_pairs']))


def compare(rows=DEFAULT_MISSION_ROWS, cols=DEFAULT_MISSION_COLS,
            cell_size_m=DEFAULT_CELL_SIZE_M, divisors=(4, 5, 6, 7, 8, 9, 10),
            rings_list=(1, 2, 3, None), **kw):
    """
    Tabella nodi/archi/copertura al variare del passo (come cell_size/n) e degli anelli:
    serve a scegliere le flag guardando i numeri.
    """
    print(f"Missione {rows}x{cols} celle da {cell_size_m:.1f} m "
          f"= {rows * cols * cell_size_m ** 2:.0f} m2")
    print(f"{'passo':>8}{'(n)':>5}{'anelli':>8}{'nodi':>7}{'archi':>8}{'grado':>7}"
          f"{'lung.min':>10}{'lung.max':>10}{'copert.':>9}{'conn.':>7}{'corti':>7}")
    out = []
    for n in divisors:
        s = cell_size_m / n
        for rg in rings_list:
            try:
                mg = build_mission_graph(rows=rows, cols=cols, cell_size_m=cell_size_m,
                                         spacing_m=s, rings=rg, verbose=False, **kw)
            except ValueError as e:
                print(f"{s:>8.4f}{n:>5}{str(rg):>8}  non costruibile: {e}")
                continue
            lo = float(mg.lengths.min()) if mg.n_edges else float('nan')
            hi = float(mg.lengths.max()) if mg.n_edges else float('nan')
            out.append(dict(spacing=mg.spacing_m, n=n, rings=rg, nodes=mg.n_nodes,
                            edges=mg.n_edges, degree=float(mg.degrees.mean()),
                            covering=mg.covering_m, connected=mg.is_connected))
            print(f"{mg.spacing_m:>8.4f}{n:>5}{str(rg):>8}{mg.n_nodes:>7}{mg.n_edges:>8}"
                  f"{mg.degrees.mean():>7.1f}{lo:>10.2f}{hi:>10.2f}{mg.covering_m:>9.2f}"
                  f"{('si' if mg.is_connected else 'NO'):>7}{mg.n_short_pairs:>7}")
    return out


def timing(path=None, **kw):
    """Quanto costa costruire il reticolo da zero contro ricaricarlo da file."""
    import os
    import tempfile
    import time
    t0 = time.perf_counter()
    mg = build_mission_graph(verbose=False, **kw)
    t_build = time.perf_counter() - t0
    path = path or os.path.join(tempfile.gettempdir(), "mission_graph_timing.npz")
    mg.save_npz(path)
    t0 = time.perf_counter()
    mg2 = load_npz(path, origin_xy=(1.0, 2.0), origin_yaw=0.7)
    t_load = time.perf_counter() - t0
    print(f"[TEMPI] {mg.n_nodes} nodi, {mg.n_edges} archi")
    print(f"[TEMPI] costruzione da zero : {t_build * 1000:8.1f} ms")
    print(f"[TEMPI] ricarica da file    : {t_load * 1000:8.1f} ms  "
          f"({os.path.getsize(path) / 1024:.0f} KB su disco)")
    print(f"[TEMPI] file: {path}")
    return t_build, t_load, mg2


def _main():
    import argparse
    p = argparse.ArgumentParser(description="Grafo di missione regolare per Spot")
    p.add_argument('--rows', type=int, default=DEFAULT_MISSION_ROWS,
                   help=f"righe di celle, a sinistra del robot (default {DEFAULT_MISSION_ROWS})")
    p.add_argument('--cols', type=int, default=DEFAULT_MISSION_COLS,
                   help=f"colonne di celle, davanti al robot (default {DEFAULT_MISSION_COLS})")
    p.add_argument('--cell', type=float, default=DEFAULT_CELL_SIZE_M,
                   help=f"lato della cella in m (default {DEFAULT_CELL_SIZE_M:g})")
    p.add_argument('--spacing', type=float, default=DEFAULT_LATTICE_SPACING_M,
                   help=f"passo fra i nodi in m, arrotondato a cell/n (default "
                        f"{DEFAULT_LATTICE_SPACING_M:g}; 0 = suggested_spacing)")
    p.add_argument('--rings', default=str(LATTICE_RINGS),
                   help="anelli di vicini collegati (k = entro la diagonale dell'anello k), "
                        "o 'tutti' (default 2)")
    p.add_argument('--min-edge', type=float, default=DEFAULT_MIN_EDGE_M)
    p.add_argument('--max-edge', type=float, default=DEFAULT_MAX_EDGE_M)
    p.add_argument('--radius', type=float, default=DEFAULT_CONNECTION_RADIUS_M)
    p.add_argument('--margin', type=float, default=0.0, help="margine dal bordo in m")
    p.add_argument('--preview', metavar='PNG', help="salva un disegno del reticolo")
    p.add_argument('--zoom', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                   help="riquadro del disegno, in coordinate griglia")
    p.add_argument('--save', metavar='NPZ', help="salva il reticolo (si ricarica con load_npz)")
    p.add_argument('--confronto', action='store_true', help="tabella passo x anelli")
    p.add_argument('--tempi', nargs='?', const=True, metavar='NPZ',
                   help="misura costruzione da zero contro ricarica da file")
    a = p.parse_args()

    rings = None if str(a.rings).lower() in ('none', 'tutti', 'all', '-1') else int(a.rings)
    if a.spacing is not None and a.spacing <= 0:
        a.spacing = None                 # 0 = passo suggerito da suggested_spacing()
    common = dict(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                  max_edge_length=a.max_edge, connection_radius=a.radius,
                  border_margin_m=a.margin)

    if a.confronto:
        compare(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                max_edge_length=a.max_edge, connection_radius=a.radius,
                border_margin_m=a.margin)
        return
    if a.tempi:
        timing(None if a.tempi is True else a.tempi, spacing_m=a.spacing, rings=rings, **common)
        return

    mg = build_mission_graph(spacing_m=a.spacing, rings=rings, **common)
    if a.preview:
        print(f"[GRAFO] disegno salvato in "
              f"{mg.preview_png(a.preview, zoom=tuple(a.zoom) if a.zoom else None)}")
    if a.save:
        print(f"[GRAFO] reticolo salvato in {mg.save_npz(a.save)}")


if __name__ == '__main__':
    _main()