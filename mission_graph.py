"""
mission_graph.py -- il grafo di missione, costruito dalla DIMENSIONE DELLA MISSIONE (righe,
colonne, lato cella: gli stessi tre numeri di SPOT_MISSION_GRID e di
environmentMap.EnvironmentMap). I nodi si generano in due modi (P.MISSION_GRAPH_SAMPLING,
in parametri.py come tutti i P.* citati qui):
  - 'reticolo' (default): un RETICOLO rettangolare di nodi equidistanziati, in forma
    chiusa. Due flag: la SPAZIATURA fra i nodi e gli ANELLI di collegamento.
  - 'casuale': nodi sparsi con una distribuzione scelta, per confrontare i due metodi
    (vedi NODI CASUALI).
Quasi tutto il commento qui sotto parla del reticolo.

PERCHE' UN RETICOLO
  1. Riproducibilita'. Il grafo dipende solo dai parametri: stessi parametri, stesso grafo,
     bit per bit. Il rigioco riproduce le scelte del pianificatore e due missioni nella
     stessa stanza sono confrontabili.
  2. Pochi archi. A passo 0.714 m sono 1225 nodi su 625 m^2 e, con due anelli, ~4700 archi:
     occupazione e pendenza, il costo vero della valutazione, si calcolano su questi.
  3. Geometria fuori dal costo. Con archi di poche lunghezze fisse alpha*dist +
     beta*pendenza dice quello che intende dire; con punti sparsi a caso un giro piu' lungo
     puo' costare meno di uno dritto solo per dove sono caduti i punti.
  4. Le coppie sono quelle del reticolo, gia' pronte: non c'e' appaiamento geometrico ne'
     all'avvio ne' a ogni aggiornamento (vedi add_edges_to_prm).

PERCHE' RETTANGOLARE E NON TRIANGOLARE
  Il triangolare sarebbe piu' isotropo e darebbe percorsi piu' corti, ma NON PUO' CONTENERE
  I CENTRI DELLE CELLE, che sono gli obiettivi dell'esplorazione e non devono spostarsi.
  Servirebbe che cell_size fosse multiplo sia di s sia di s*sqrt(3)/2, e con sqrt(3)
  irrazionale non succede per nessun passo. Nel rettangolare con passo cell_size/n i centri
  sono nodi per costruzione. Se si volesse riaprire la questione, il punto da attaccare e'
  questo, non l'isotropia.

SPAZIATURA
  Il passo richiesto viene SEMPRE arrotondato a cell_size/n con n intero: e' la condizione
  che rende ogni centro cella un nodo del reticolo. La riga [GRAFO] del log dice il passo
  effettivo. Default della missione: P.MISSION_LATTICE_SPACING_M = 0.5 m (= 4/8); senza, 0.25 m
  (DEFAULT_LATTICE_SPACING_M).
  Il passo puo' essere piu' corto di min_edge_length: i vicini troppo vicini semplicemente
  non si collegano, e gli archi partono dagli anelli piu' esterni. La soglia sugli archi
  serve alla pendenza (un arco troppo corto non dice nulla del terreno); il passo fitto
  serve ad avere SEMPRE una fila di nodi dentro un corridoio (vedi ALIASING).

ANELLI -- quante connessioni
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
  I tratti lunghi in rettilineo li produce comunque arcVerification.shortcut_index, che salta i
  nodi intermedi quando il FRONTE SICURO conferma il tratto sui dati dal vivo.
  rings=None arriva fino a min(connection_radius, max_edge_length).

ANCORAGGIO
  Il reticolo e' ancorato all'ORIGINE, che per come e' scritta environmentMap e' insieme la
  posa di accensione del robot E il centro della cella di partenza
  (get_world_position_from_cell: "Cell (0,0) center is at the origin"). Quindi c'e' un nodo
  esattamente sotto il robot all'avvio, vedi origin_node_id. Gli indici (i, j) possono
  essere negativi; il rettangolo li ritaglia.

ALIASING -- il limite da conoscere
  Se un corridoio cade fra due file di nodi, nel grafo non esiste. Il fronte sicuro
  pretende spotGrid.FRONTIER_CLEARANCE_M = c dalla linea centrale (0.25 m senza aria, 0.30
  con 5 cm), quindi un corridoio utile e' largo almeno 2c e le linee centrali ammesse sono
  una striscia larga (larghezza - 2c): perche' ci cada sempre una fila di nodi il passo deve
  essere piu' corto della striscia. Per i nodi casuali basta copertura <= striscia / 2.
  Passaggio garantito: reticolo 2c + passo, nodi casuali 2c + 2 * copertura. A passo 0.25
  ogni striscia piu' larga di 0.25 m contiene una fila di nodi.

NODI CASUALI (P.MISSION_GRAPH_SAMPLING = 'casuale')
  In alternativa al reticolo, i nodi si possono spargere a caso nell'area della missione,
  per confrontare i due metodi. I centri delle celle restano SEMPRE nodi (sono gli
  obiettivi, e il centro della cella di partenza e' il punto di accensione); gli altri
  nodi seguono una di queste distribuzioni, con P.MISSION_RANDOM_DENSITY nodi per m^2:
    'uniforme'   punti indipendenti e uniformi: il PRM classico, con buchi e grumi;
    'poisson'    Poisson-disk: punti uniformi scartati se piu' vicini di una distanza
                 minima a un nodo gia' messo (P.MISSION_POISSON_MIN_DIST_M; None = 0.75/sqrt(densita'),
                 che lascia raggiungere la densita' chiesta): niente grumi;
    'perturbata' l'area divisa in quadretti da 1/densita' m^2, un punto a caso in ognuno
                 (pochi quadretti tolti o punti aggiunti a caso per arrivare al numero
                 esatto): nessuna zona resta vuota piu' di un quadretto;
    'halton'     sequenza di Halton in basi 2 e 3, quasi casuale e molto uniforme; il
                 seme la trasla (rotazione di Cranley-Patterson), cosi' semi diversi danno
                 insiemi diversi.
  Gli archi collegano ogni coppia di nodi distante fra min_edge_length e
  min(max_edge_length, connection_radius); passo e anelli non contano.
  SEME: P.MISSION_RANDOM_SEED fisso ripete lo stesso grafo bit per bit; None = un seme estratto a
  caso. In entrambi i casi il seme usato va nel log (riga [GRAFO] Nodi casuali) e in
  mission_summary.json, cosi' la missione si puo' ripetere e test_replay la rigioca con
  lo stesso grafo.
  La copertura (distanza massima fra un punto dell'area e il nodo piu' vicino) qui non ha
  una formula: si misura su una griglia fitta e va nel log, perche' e' il numero che dice
  se un corridoio stretto puo' restare senza nodi (vedi ALIASING).

COME SI USA
      mg = mission_graph.build_mission_graph(env=env)   # ~0.2 s: nodi + adiacenza
      mg.into_prm(prm)                                  # i nodi, id 0..N-1
      gb_sampler = mg.as_sampler()                      # punti del reticolo per cella
      ...
      mg.add_edges_to_prm(prm, global_map, spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
  add_edges_to_prm valuta ogni coppia con PRM._evaluate_and_add_edge, la stessa funzione
  usata per tutti gli archi: veti e pesi vengono da li'. Sul robot si calcolano solo i veti
  e i pesi, che per costruzione non sono precalcolabili: l'occupazione viene dalla mappa
  globale che si riempie strada facendo, la pendenza d'arco dal terreno visto in quel
  momento.
  save_npz/load_npz servono ad avere un reticolo IDENTICO fra piu' missioni, non per la
  velocita': con la missione di default (3185 nodi) costruire da zero costa ~180 ms,
  ricaricare ~70 ms (python3 mission_graph.py --tempi).

Il modulo dipende SOLO da numpy: si importa e si prova fuori dal robot, senza l'SDK.
environmentMap, prm_graph, scipy (solo per i nodi casuali) e matplotlib sono importati
dentro le funzioni che li usano.

Da riga di comando:
    python3 mission_graph.py --confronto
    python3 mission_graph.py --spacing 0.25 --rings 2 --preview p.png --zoom -2 2 -2 2
    python3 mission_graph.py --grafo casuale --distribuzione poisson --densita 16 --seme 7 --preview p.png
    python3 mission_graph.py --confronta-metodi metodi.png --seme 7 --zoom -2 2 -2 2
    python3 mission_graph.py --rows 2 --cols 3 --cell 3
    python3 mission_graph.py --save reticolo_2x4x5.npz
    python3 mission_graph.py --tempi
"""

import math
from collections import deque

import numpy as np

import parametri as P          # dimensione della missione, metodo dei nodi, archi
import prm_graph
import spotGrid


# La missione (righe, colonne, lato, passo, metodo dei nodi, archi) si sceglie in
# parametri.py, cosi' la missione sul robot e le prove da riga di comando (python3
# mission_graph.py) partono dagli stessi numeri. Questi DEFAULT_* servono solo a compare() e
# come ripiego se in parametri un valore e' None (2 x 4 celle da 5 m, passo 0.25 m).
DEFAULT_MISSION_ROWS = 2
DEFAULT_MISSION_COLS = 4
DEFAULT_CELL_SIZE_M = 5.0
DEFAULT_LATTICE_SPACING_M = 0.25


# ----------------------------------------------------------------------------------------
# COME SI GENERANO I NODI (vedi NODI CASUALI nel commento in cima).
#   'reticolo' il reticolo rettangolare, deterministico (default);
#   'casuale'  nodi sparsi con P.MISSION_RANDOM_DISTRIBUTION, centri cella sempre compresi.
# Si cambiano per una sola missione con gli argomenti di easy_walk.py (--grafo,
# --distribuzione, --densita, --seme).
# ----------------------------------------------------------------------------------------
GRAPH_SAMPLINGS = ('reticolo', 'casuale')
RANDOM_DISTRIBUTIONS = ('uniforme', 'poisson', 'perturbata', 'halton')


def mission_settings(rows=None, cols=None, cell_size_m=None, spacing_m=None):
    """
    (rows, cols, cell_size_m, spacing_m) della missione. Per ognuno vince, nell'ordine:
    il valore passato qui, quello di parametri.py, il DEFAULT_*.
    """
    def pick(value, chosen, default):
        if value is not None:
            return value
        return chosen if chosen is not None else default
    return (int(pick(rows, P.MISSION_GRID_ROWS, DEFAULT_MISSION_ROWS)),
            int(pick(cols, P.MISSION_GRID_COLS, DEFAULT_MISSION_COLS)),
            float(pick(cell_size_m, P.MISSION_CELL_SIZE_M, DEFAULT_CELL_SIZE_M)),
            float(pick(spacing_m, P.MISSION_LATTICE_SPACING_M, DEFAULT_LATTICE_SPACING_M)))


# Valore di default di spacing_m in build_mission_graph: "prendilo da mission_settings()".
# Non e' None perche' None vuol gia' dire suggested_spacing().
_FROM_SETTINGS = "settings"


def covering_radius(spacing):
    """
    Distanza massima fra un punto QUALSIASI dell'area e il nodo piu' vicino: mezza diagonale
    della maglia. E' il numero che conta per l'aliasing.
    """
    return spacing / math.sqrt(2.0)


def node_density(spacing):
    """Nodi per metro quadro."""
    return 1.0 / spacing ** 2


def align_spacing(spacing_m, cell_size_m):
    """
    Il passo arrotondato a cell_size/n con n intero: la condizione che rende ogni centro
    cella un nodo del reticolo. Restituisce (passo, n).
    """
    n = max(1, int(round(float(cell_size_m) / float(spacing_m))))
    return float(cell_size_m) / n, n


def suggested_spacing(cell_size_m=5.0, min_edge_length=P.PRM_MIN_EDGE_M):
    """
    Il passo allineato piu' grande la cui copertura non supera min_edge_length. Con celle da
    5 m e min_edge 0.5 m da' cell_size/7 = 0.714 m (copertura 0.505). Se cambia
    min_edge_length nel PRM, il passo consigliato si ricalcola da solo.
    """
    n = max(1, int(math.ceil(float(cell_size_m) / (float(min_edge_length) * math.sqrt(2.0)))))
    return float(cell_size_m) / n


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
    (O(N*k) di aritmetica intera) invece di interrogare un albero k-d su tutte le coppie.

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


# ----------------------------------------------------------------------------------------
# NODI CASUALI
# ----------------------------------------------------------------------------------------

def draw_seed():
    """Un seme nuovo, quando non ne e' stato chiesto uno: 31 bit dal sistema."""
    import random
    return random.SystemRandom().randrange(2 ** 31)


def poisson_min_dist(density_per_m2):
    """
    Distanza minima del Poisson-disk per arrivare alla densita' chiesta. Spargendo punti a
    caso con distanza minima r ci si ferma a circa 0.70/r^2 punti per m^2 (saturazione);
    con r = 0.75/sqrt(densita') la saturazione sta a ~1.24 volte la densita' chiesta, che
    resta raggiungibile.
    """
    return 0.75 / math.sqrt(float(density_per_m2))


def _halton(n, base, start=1):
    """I primi n termini della sequenza di van der Corput in `base`, dall'indice start."""
    out = np.zeros(n, dtype=np.float64)
    for k in range(n):
        i, f, x = start + k, 1.0, 0.0
        while i > 0:
            f /= base
            x += f * (i % base)
            i //= base
        out[k] = x
    return out


def _poisson_points(n_target, box, fixed_xy, min_dist, rng, max_tries_per_point=50):
    """
    Poisson-disk per lancio di freccette: punti uniformi nel rettangolo, scartati se piu'
    vicini di min_dist a un punto gia' messo (centri cella compresi). Si ferma a n_target
    punti o quando i tentativi falliti di fila diventano troppi (area satura). Una griglia
    di appoggio con lato min_dist/sqrt(2) tiene al massimo un punto per casella, cosi'
    ogni controllo guarda solo le 5 x 5 caselle attorno.
    """
    x0, x1, y0, y1 = box
    h = min_dist / math.sqrt(2.0)
    nx, ny = max(1, int(math.ceil((x1 - x0) / h))), max(1, int(math.ceil((y1 - y0) / h)))
    grid = -np.ones((ny, nx), dtype=np.int64)
    pts = [tuple(p) for p in fixed_xy]
    r2 = min_dist * min_dist

    def cell_of(x, y):
        return (min(ny - 1, max(0, int((y - y0) / h))), min(nx - 1, max(0, int((x - x0) / h))))

    for k, (x, y) in enumerate(pts):
        grid[cell_of(x, y)] = k
    added, fails, limit = 0, 0, max_tries_per_point * max(1, n_target)
    while added < n_target and fails < limit:
        batch = rng.random((256, 2))
        for u, v in batch:
            x, y = x0 + u * (x1 - x0), y0 + v * (y1 - y0)
            gi, gj = cell_of(x, y)
            ok = True
            for a in range(max(0, gi - 2), min(ny, gi + 3)):
                for b in range(max(0, gj - 2), min(nx, gj + 3)):
                    k = grid[a, b]
                    if k >= 0:
                        dx, dy = pts[k][0] - x, pts[k][1] - y
                        if dx * dx + dy * dy < r2:
                            ok = False
                            break
                if not ok:
                    break
            if not ok:
                fails += 1
                if fails >= limit:
                    break
                continue
            grid[gi, gj] = len(pts)
            pts.append((x, y))
            added += 1
            if added >= n_target:
                break
    return np.array(pts[len(fixed_xy):], dtype=np.float64).reshape(-1, 2)


def _random_points(distribution, density_per_m2, box, fixed_xy, rng, min_dist=None):
    """
    I nodi casuali nel rettangolo box = (x_min, x_max, y_min, y_max), senza i nodi fissi
    (centri cella), che il poisson pero' rispetta. Restituisce (xy, note): note e' una
    stringa per il log quando la densita' chiesta non e' stata raggiunta.
    """
    x0, x1, y0, y1 = box
    w, h = x1 - x0, y1 - y0
    n = int(round(float(density_per_m2) * w * h))
    if distribution == 'uniforme':
        u = rng.random((n, 2))
        return np.column_stack((x0 + u[:, 0] * w, y0 + u[:, 1] * h)), ""
    if distribution == 'perturbata':
        side = 1.0 / math.sqrt(float(density_per_m2))
        nx, ny = max(1, int(round(w / side))), max(1, int(round(h / side)))
        hx, hy = w / nx, h / ny                   # quadretti che coprono il rettangolo esatto
        jj, ii = np.meshgrid(np.arange(nx), np.arange(ny))
        u = rng.random((ny * nx, 2))
        xy = np.column_stack((x0 + (jj.ravel() + u[:, 0]) * hx,
                              y0 + (ii.ravel() + u[:, 1]) * hy))
        # I quadretti sono interi e non danno sempre esattamente n punti: si tolgono a caso
        # quelli in piu', o si aggiungono punti uniformi, cosi' il totale e' quello chiesto.
        if len(xy) > n:
            xy = xy[np.sort(rng.choice(len(xy), n, replace=False))]
        elif len(xy) < n:
            e = rng.random((n - len(xy), 2))
            xy = np.vstack((xy, np.column_stack((x0 + e[:, 0] * w, y0 + e[:, 1] * h))))
        return xy, ""
    if distribution == 'halton':
        shift = rng.random(2)
        u = (np.column_stack((_halton(n, 2), _halton(n, 3))) + shift) % 1.0
        return np.column_stack((x0 + u[:, 0] * w, y0 + u[:, 1] * h)), ""
    if distribution == 'poisson':
        r = float(min_dist) if min_dist else poisson_min_dist(density_per_m2)
        xy = _poisson_points(n, box, fixed_xy, r, rng)
        note = ("" if len(xy) >= n else
                f"area satura a {len(xy)} nodi su {n} chiesti con distanza minima {r:.3f} m")
        return xy, note
    raise ValueError(f"distribuzione sconosciuta {distribution!r}: una di {RANDOM_DISTRIBUTIONS}")


def _radius_pairs(xy, min_edge_m, reach_m):
    """
    Le coppie di nodi distanti fra min_edge_m e reach_m, con la lunghezza, ordinate come
    quelle del reticolo. Restituisce (pairs, lengths, n_sotto_soglia).
    """
    from scipy.spatial import cKDTree
    found = cKDTree(xy).query_pairs(r=float(reach_m), output_type='ndarray')
    if len(found) == 0:
        return np.zeros((0, 2), np.int64), np.zeros(0), 0
    found = np.sort(found.astype(np.int64), axis=1)
    d = xy[found[:, 1]] - xy[found[:, 0]]
    lengths = np.sqrt(d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1])
    keep = lengths >= float(min_edge_m) - 1e-12
    pairs, lengths = found[keep], lengths[keep]
    order = np.lexsort((pairs[:, 1], pairs[:, 0]))
    return pairs[order], lengths[order], int((~keep).sum())


def measured_covering(xy, box, step=0.05):
    """
    Copertura misurata: (massima, 95-esimo percentile) della distanza fra i punti di una
    griglia fitta nel rettangolo e il nodo piu' vicino. Per il reticolo c'e' la formula
    (covering_radius); per i nodi casuali serve misurarla.
    """
    from scipy.spatial import cKDTree
    x0, x1, y0, y1 = box
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step))
    d, _ = cKDTree(xy).query(np.column_stack((gx.ravel(), gy.ravel())))
    return float(d.max()), float(np.percentile(d, 95))


# Pendenze delle coppie calcolate in blocco con le matrici (vedi PRM.precompute_arc_slopes).
# False = arco per arco; il risultato e' identico.
BATCH_SLOPES = True


def _special_pairs(prm):
    """Archi percorsi dal robot e archi bloccati nel tentativo, come coppie (min, max)."""
    out = {(min(a, b), max(a, b)) for a, b in getattr(prm, 'traversed_edges', ())}
    out |= {(min(a, b), max(a, b)) for a, b in getattr(prm, 'tracker_blocked_edges', ())}
    return out


def _prm_dirty_margin():
    """prm_graph.DIRTY_MARGIN_M, importato solo quando serve (questo modulo gira anche senza SDK)."""
    try:
        import prm_graph
        return prm_graph.DIRTY_MARGIN_M
    except Exception:
        return 1.2


def _frontier_clearance():
    """spotGrid.FRONTIER_CLEARANCE_M, importato solo quando serve (questo modulo gira anche senza SDK)."""
    try:
        import spotGrid
        return spotGrid.FRONTIER_CLEARANCE_M
    except Exception:
        return 0.25


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
                 min_edge_m, reach_m, rings, n_short_pairs, sampling='reticolo',
                 distribution=None, density_per_m2=None, seed=None, covering=None,
                 sampling_note=""):
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
        # Come sono stati generati i nodi (vedi NODI CASUALI). Per i nodi casuali ij vale
        # -1 e spacing_m e' il passo medio 1/sqrt(densita'); la copertura e' misurata.
        self.sampling = str(sampling)
        self.distribution = distribution
        self.density_per_m2 = None if density_per_m2 is None else float(density_per_m2)
        self.seed = None if seed is None else int(seed)
        self.covering = covering                  # (massima, 95-esimo percentile) o None
        self.sampling_note = sampling_note

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
    def is_random(self):
        return self.sampling == 'casuale'

    @property
    def covering_m(self):
        if self.covering is not None:
            return float(self.covering[0])
        return covering_radius(self.spacing_m)

    def describe(self):
        """Il metodo dei nodi in un dict, per mission_summary.json e per rigiocare la missione."""
        out = dict(metodo=self.sampling, nodi=self.n_nodes, coppie=self.n_edges,
                   arco_min_m=self.min_edge_m, arco_max_m=round(self.reach_m, 6),
                   copertura_m=round(self.covering_m, 4))
        if self.is_random:
            out.update(distribuzione=self.distribution, densita_nodi_m2=self.density_per_m2,
                       seme=self.seed)
        else:
            out.update(passo_m=self.spacing_m, anelli=self.rings)
        return out

    def node_ids(self):
        """Gli id dei nodi, 0..N-1: gli stessi che useranno PRM.nodes."""
        return np.arange(self.n_nodes, dtype=np.int64)

    # -- aggancio al PRM ---------------------------------------------------------------
    def into_prm(self, prm):
        """
        Mette i NODI nel PRM (prm.add_node), con gli id 0..N-1. Nessun arco: gli archi
        arrivano da add_edges_to_prm, che ha bisogno dei dati di terreno.
        """
        for k in range(self.n_nodes):
            prm.add_node(int(k), float(self.xy[k, 0]), float(self.xy[k, 1]))
        return self.n_nodes

    def add_edges_to_prm(self, prm, global_map=None, edge_safety_margin=0.0, verbose=True,
                         incremental=False):
        """
        Valuta le coppie GIA' CALCOLATE e aggiunge quelle ammesse con
        PRM._evaluate_and_add_edge: veti (archi bloccati, occupazione nella mappa globale,
        pendenza longitudinale e laterale) e pesi. Le coppie sono quelle del reticolo, non
        si rifa' nessun appaiamento geometrico.

        incremental: rivaluta SOLO le coppie vicine alle zone con dati nuovi
        dall'ultima valutazione (prm.dirty_rects, vedi prm_graph.DIRTY_MARGIN_M); le altre
        tengono arco e peso che hanno. Il risultato e' lo stesso della costruzione completa:
        fuori da quelle zone nessun dato che entra nella valutazione e' cambiato. Si torna
        alla costruzione completa quando non si puo' sapere cosa e' cambiato (vedi
        _dirty_pairs).
        """
        todo_all = [(int(a), int(b), float(d)) for (a, b), d in zip(self.pairs, self.lengths)
                    if int(a) in prm.nodes and int(b) in prm.nodes]
        dirty = self._dirty_pairs(prm, global_map, todo_all) if incremental else None
        prm.robot_node = None          # il permesso vicino al robot vale solo per il nodo attuale
        if dirty is None:
            for idx in prm.nodes:
                prm.edges[idx] = []
            prm.edge_validity = {}
            todo = todo_all
        else:
            todo = [p for p, m in zip(todo_all, dirty) if m]
            # Si tolgono gli archi da rivalutare; poi si valutano come in una costruzione da
            # zero (adiacenze appena svuotate per queste coppie: fresh vale anche qui).
            drop = {}
            for a, b, _d in todo:
                drop.setdefault(a, set()).add(b)
                drop.setdefault(b, set()).add(a)
                prm.edge_validity.pop((a, b) if a < b else (b, a), None)
            for n, others in drop.items():
                prm.edges[n] = [(m, w) for m, w in prm.edges.get(n, []) if m not in others]
        if hasattr(prm, '_sync_global_slope_cache'):
            prm._sync_global_slope_cache()

        added = 0
        # Occupazione di tutte le coppie in un colpo solo e cache della pendenza globale
        # riempita dove certamente non ci sono dati (esiti identici, vedi
        # PRM._occupancy_blocked_many e PRM._prefill_global_slope_nodata). Le adiacenze sono
        # appena state svuotate e le coppie del reticolo sono distinte: fresh=True.
        batch = hasattr(prm, '_occupancy_blocked_many')
        use_occ = global_map is not None and edge_safety_margin > 0.0
        occ_flags = (prm._occupancy_blocked_many(todo, global_map, edge_safety_margin)
                     if (batch and use_occ) else [None] * len(todo))
        if batch:
            prm._prefill_global_slope_nodata(todo)
        fresh = batch and self._pairs_distinct()
        # ZONA SENZA DATI: archi su cui nessuna fonte ha informazione prendono
        # direttamente il costo di ripiego, identico a quello del percorso completo (vedi
        # PRM._add_edge_without_data). Serve gamma = 0: con la rugosita' nel costo, no.
        fast_ok = (fresh and hasattr(prm, '_add_edge_without_data') and prm.gamma == 0.0)
        live_nd = prm._live_slope_certainly_nodata(todo) if fast_ok else None
        tm = getattr(prm, 'global_terrain_map', None)
        cache = getattr(prm, '_gslope_cache', {})
        blocked_set = prm.tracker_blocked_edges
        n_fast = 0
        # Pendenze di tutte le coppie da valutare in un colpo solo (matrici), dal vivo e,
        # dove dal vivo non ci sono dati, dalla mappa globale. Stessi numeri del calcolo arco
        # per arco, circa 13 volte piu' in fretta (vedi PRM.precompute_arc_slopes).
        live_profs = [None] * len(todo)
        if BATCH_SLOPES and batch and hasattr(prm, 'precompute_arc_slopes'):
            skip = np.array([bool(o) for o in occ_flags], dtype=bool)
            live_profs = prm.precompute_arc_slopes(todo, skip=skip)
        for i, ((a, b, d), occ) in enumerate(zip(todo, occ_flags)):
            if fast_ok and live_nd[i] and not occ:
                key = (a, b) if a < b else (b, a)
                g = cache.get(key) if tm is not None else (0.0, 0.0, False)
                if key not in blocked_set and g is not None and g[2] is False:
                    prm._add_edge_without_data(a, b, d)
                    added += 1
                    n_fast += 1
                    continue
            if batch:
                ok = prm._evaluate_and_add_edge(a, b, d, global_map, edge_safety_margin,
                                                occ_blocked=occ, fresh=fresh, live_prof=live_profs[i])
            else:
                ok = prm._evaluate_and_add_edge(a, b, d, global_map, edge_safety_margin)
            if ok:
                added += 1

        self.last_update = (len(todo), len(todo_all), dirty is not None)

        # Gli archi percorsi dal robot sono liberi per costruzione e vanno rimessi comunque:
        # i nodi di sosta stanno spesso sotto min_edge_length.
        for a, b in list(getattr(prm, 'traversed_edges', ())):
            if a in prm.nodes and b in prm.nodes:
                if hasattr(prm, '_traversed_edge_ok') and not prm._traversed_edge_ok(a, b, global_map):
                    prm.traversed_edges.discard((a, b))
                    continue
                (xa, ya), (xb, yb) = prm.nodes[a], prm.nodes[b]
                prm._evaluate_and_add_edge(a, b, float(math.hypot(xb - xa, yb - ya)), force=True)
        # Le zone appena rivalutate sono a posto. Resta "da rifare" la finestra dal vivo
        # corrente: i suoi archi sono stati giudicati sui dati dal vivo, e la prossima volta,
        # col robot altrove, la costruzione completa li giudicherebbe sulla mappa globale.
        if hasattr(prm, 'dirty_rects'):
            prm.dirty_rects = list(prm.live_rects())
            prm.hazards_seen = len(getattr(global_map, 'hazards', ()) or ())
            prm.special_seen = _special_pairs(prm)
        prm.built = True
        if hasattr(prm, 'flush_slope_log'):
            prm.flush_slope_log("costruzione")
        if verbose and dirty is not None:
            print(f"[GRAFO] Aggiornamento locale: rivalutate {len(todo)} coppie su {len(todo_all)} "
                  f"(vicine alle zone con dati nuovi), {added} ammesse; le altre restano come erano.")
        elif verbose:
            where = (f"nodi casuali {self.distribution}" if self.is_random
                     else f"passo {self.spacing_m:.4f} m")
            print(f"[GRAFO] {added} archi ammessi su {self.n_edges} coppie del "
                  f"{'grafo' if self.is_random else 'reticolo'} ({self.n_nodes} nodi, {where}); "
                  f"{n_fast} in zona senza dati, con il solo costo geometrico")
        return added

    def _dirty_pairs(self, prm, global_map, todo):
        """
        Maschera delle coppie da rivalutare (rettangolo dell'arco, allargato di
        prm_graph.DIRTY_MARGIN_M, che tocca una zona di prm.dirty_rects), o None quando serve
        la costruzione completa: grafo mai costruito, mappa del terreno ricaricata da file
        (zone aggiornate ignote), una caduta nuova (mark_hazard non passa da una scansione).

        Gli archi percorsi dal robot e quelli bloccati nel tentativo si rivalutano SEMPRE,
        insieme a quelli che lo erano all'aggiornamento precedente: il loro esito dipende da
        quegli insiemi, non dai dati nuovi. Sono pochi.
        """
        if not getattr(prm, 'built', False) or not hasattr(prm, 'dirty_rects'):
            return None
        tm = getattr(prm, 'global_terrain_map', None)
        if tm is not None and getattr(tm, 'update_rects', None) is None:
            return None
        if len(getattr(global_map, 'hazards', ()) or ()) != getattr(prm, 'hazards_seen', 0):
            return None
        mask = np.zeros(len(todo), dtype=bool)
        if not todo:
            return mask
        ia = np.array([p[0] for p in todo]); ib = np.array([p[1] for p in todo])
        a = np.array([prm.nodes[i] for i in ia]); b = np.array([prm.nodes[i] for i in ib])
        m = _prm_dirty_margin()
        xmin, xmax = np.minimum(a[:, 0], b[:, 0]) - m, np.maximum(a[:, 0], b[:, 0]) + m
        ymin, ymax = np.minimum(a[:, 1], b[:, 1]) - m, np.maximum(a[:, 1], b[:, 1]) + m
        for (rx0, rx1, ry0, ry1) in prm.dirty_rects:
            mask |= (xmax >= rx0) & (xmin <= rx1) & (ymax >= ry0) & (ymin <= ry1)
        special = _special_pairs(prm) | getattr(prm, 'special_seen', set())
        if special:
            mask |= np.array([(min(a, b), max(a, b)) in special for a, b in zip(ia.tolist(), ib.tolist())])
        return mask

    def _pairs_distinct(self):
        """True se nessuna coppia del reticolo compare due volte (in nessun verso). Calcolato una volta."""
        cached = getattr(self, '_distinct_cache', None)
        if cached is None:
            p = np.sort(np.asarray(self.pairs, dtype=np.int64).reshape(-1, 2), axis=1)
            cached = bool(p.shape[0] == 0 or np.unique(p, axis=0).shape[0] == p.shape[0])
            self._distinct_cache = cached
        return cached

    def as_sampler(self):
        """I punti del reticolo cella per cella (get_point_in_cell), per find_best_point_in_cell."""
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
            cell_center_keys=keys, cell_center_vals=vals, n_short_pairs=self.n_short_pairs,
            sampling=self.sampling, distribution=str(self.distribution or ''),
            density_per_m2=(-1.0 if self.density_per_m2 is None else self.density_per_m2),
            seed=(-1 if self.seed is None else self.seed),
            covering=(np.array([-1.0, -1.0]) if self.covering is None
                      else np.array(self.covering, dtype=np.float64)),
            sampling_note=self.sampling_note)
        return path

    def report(self):
        """Il riepilogo da stampare nel log, accanto alle righe [CONFIG]."""
        lo = float(self.lengths.min()) if self.n_edges else float('nan')
        hi = float(self.lengths.max()) if self.n_edges else float('nan')
        uniq = sorted(set(np.round(self.lengths, 3).tolist()))
        rings = ", ".join(f"{v:.2f}" for v in uniq[:6]) + (" ..." if len(uniq) > 6 else "")
        if self.is_random:
            head = [
                f"[GRAFO] Missione {self.rows}x{self.cols} celle da {self.cell_size_m:.1f} m "
                f"= {self.area_m2:.0f} m2; nodi CASUALI, archi fra {self.min_edge_m:.2f} e "
                f"{self.reach_m:.2f} m",
                # test_replay legge distribuzione, densita' e seme da questa riga.
                f"[GRAFO] Nodi casuali: distribuzione {self.distribution}, densita' "
                f"{self.density_per_m2:.2f} nodi/m2, seme {self.seed} "
                f"(per ripetere: --grafo casuale --distribuzione {self.distribution} "
                f"--densita {self.density_per_m2:g} --seme {self.seed})",
            ]
            if self.sampling_note:
                head.append(f"[GRAFO] ATTENZIONE: {self.sampling_note}")
            density = self.n_nodes / self.area_m2
            cover = (f"[GRAFO] Copertura misurata {self.covering[0]:.2f} m (95% dell'area entro "
                     f"{self.covering[1]:.2f} m; un corridoio utile al fronte e' largo almeno "
                     f"{2 * _frontier_clearance():.2f} m)")
        else:
            n_div = int(round(self.cell_size_m / self.spacing_m))
            head = [
                f"[GRAFO] Missione {self.rows}x{self.cols} celle da {self.cell_size_m:.1f} m "
                f"= {self.area_m2:.0f} m2; reticolo rettangolare, passo {self.spacing_m:.4f} m "
                f"(= {self.cell_size_m:.1f}/{n_div}), anelli "
                f"{'tutti entro %.2f m' % self.reach_m if self.rings is None else self.rings}",
            ]
            density = node_density(self.spacing_m)
            cover = (f"[GRAFO] Copertura {self.covering_m:.2f} m (un corridoio utile al fronte e' "
                     f"largo almeno {2 * _frontier_clearance():.2f} m)")
        out = head + [
            f"[GRAFO] {self.n_nodes} nodi, {density:.2f} nodi/m2, "
            f"{self.n_edges} archi, grado medio {self.degrees.mean():.1f} "
            f"(min {int(self.degrees.min()) if self.n_nodes else 0}, "
            f"max {int(self.degrees.max()) if self.n_nodes else 0})",
            f"[GRAFO] Lunghezze degli archi {lo:.2f}-{hi:.2f} m"
            f"{'' if self.is_random else ' [%s]' % rings}, "
            f"min_edge_length={self.min_edge_m:.2f} m, "
            f"{self.n_short_pairs} coppie di vicini troppo vicini, non collegate",
            cover,
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
            if self.is_random:
                out.append(f"[GRAFO] ATTENZIONE: il grafo casuale non e' connesso con questi "
                           f"parametri (densita' {self.density_per_m2:g} nodi/m2, archi "
                           f"{self.min_edge_m:.2f}-{self.reach_m:.2f} m): aumenta la densita' "
                           f"o la lunghezza massima degli archi.")
            else:
                out.append(f"[GRAFO] ATTENZIONE: il reticolo non e' connesso con questi parametri "
                           f"(passo {self.spacing_m:.4f} m, archi {self.min_edge_m:.2f}-"
                           f"{self.reach_m:.2f} m): aumenta gli anelli.")
        return "\n".join(out)

    def preview_png(self, path, show_cells=True, zoom=None):
        """
        Disegna il grafo e i suoi archi in un PNG, per guardarlo prima di uscire.
        zoom: (x_min, x_max, y_min, y_max) nel frame della griglia, per i dettagli.
        """
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(9.5, 9.5))
        self.draw(ax, show_cells=show_cells, zoom=zoom)
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
        return path

    def draw(self, ax, show_cells=True, zoom=None, legend=True, title_size=None,
             edge_color='0.78', edge_width=0.45, highlight=None):
        """
        Disegna nodi, archi, obiettivi di cella e celle su un riquadro matplotlib.
        highlight: id di un nodo di cui disegnare gli archi in evidenza (i suoi vicini).
        """
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        segs = [[(self.xy_local[a, 0], self.xy_local[a, 1]),
                 (self.xy_local[b, 0], self.xy_local[b, 1])] for a, b in self.pairs]
        ax.add_collection(LineCollection(segs, colors=edge_color, linewidths=edge_width, zorder=1))
        if highlight is not None:
            mine = [s for s, (a, b) in zip(segs, self.pairs) if highlight in (a, b)]
            ax.add_collection(LineCollection(mine, colors='tab:blue', linewidths=0.8, alpha=0.7,
                                             zorder=2, label=f"archi del nodo {highlight}"))
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
                            fontsize=7, color='tab:green', zorder=6, clip_on=True)
        if zoom:
            ax.set_xlim(zoom[0], zoom[1])
            ax.set_ylim(zoom[2], zoom[3])
        else:
            ax.autoscale_view()
        ax.set_aspect('equal', 'box')
        ax.set_xlabel('x griglia [m] -- colonne, davanti al robot')
        ax.set_ylabel('y griglia [m] -- righe, a sinistra del robot')
        what = (f"casuale {self.distribution}, {self.density_per_m2:.3g} nodi/m2, seme {self.seed}"
                if self.is_random else
                f"passo {self.spacing_m:.4f} m, anelli "
                f"{self.rings if self.rings is not None else 'tutti'}")
        ax.set_title(f"{what}\n{self.n_nodes} nodi, {self.n_edges} archi, "
                     f"copertura {self.covering_m:.2f} m", fontsize=title_size)
        if legend:
            ax.legend(loc='upper right', fontsize=8)


class LatticeSampler:
    """I punti del reticolo cella per cella (vedi MissionGraph.as_sampler)."""

    def __init__(self, mg):
        self._points = [(float(x), float(y)) for x, y in mg.xy]
        self._by_cell = {}
        for k, (r, c) in enumerate(mg.node_cell):
            self._by_cell.setdefault((int(r), int(c)), []).append(k)

    def get_point_in_cell(self, row, col):
        return [self._points[k] for k in self._by_cell.get((int(row), int(col)), ())]

# ========================================================================================
# LA FUNZIONE DA CHIAMARE
# ========================================================================================

def build_mission_graph(rows=None, cols=None, cell_size_m=None, spacing_m=_FROM_SETTINGS,
                        rings=P.MISSION_LATTICE_RINGS,
                        origin_xy=(0.0, 0.0), origin_yaw=0.0, start_cell=(0, 0),
                        min_edge_length=P.PRM_MIN_EDGE_M,
                        max_edge_length=P.PRM_MAX_EDGE_M,
                        connection_radius=P.PRM_CONNECTION_RADIUS_M,
                        border_margin_m=0.0, env=None, verbose=True,
                        sampling=None, distribution=None, density_per_m2=None, seed=None,
                        poisson_min_dist_m=None):
    """
    Costruisce il grafo di missione.

    Args:
        rows, cols, cell_size_m: la DIMENSIONE DELLA MISSIONE. Se non passati: quelli di
            mission_settings() (MISSION_*, o i DEFAULT_*). Ignorati se si passa `env`.
        spacing_m: FLAG DELLA SPAZIATURA, in metri; viene arrotondata a cell_size/n (vedi
            align_spacing). Puo' essere sotto min_edge_length (vedi SPAZIATURA). Se non
            passato: quello di mission_settings() (P.MISSION_LATTICE_SPACING_M, o 0.25 m); None =
            suggested_spacing().
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
        sampling: 'reticolo' o 'casuale' (vedi NODI CASUALI); None = P.MISSION_GRAPH_SAMPLING. Con
            'casuale' spacing_m e rings non contano.
        distribution, density_per_m2, seed, poisson_min_dist_m: i parametri dei nodi
            casuali; None = P.MISSION_RANDOM_DISTRIBUTION, P.MISSION_RANDOM_DENSITY,
            P.MISSION_RANDOM_SEED, P.MISSION_POISSON_MIN_DIST_M. Seme None anche li' = estratto a caso (e scritto nel log).

    Returns:
        MissionGraph.
    """
    if env is not None:
        rows, cols, cell_size_m = env.rows, env.cols, env.cell_size
        origin_xy = (env.origin_x, env.origin_y)
        origin_yaw = env.origin_yaw
        start_cell = tuple(env.start_cell)
    rows, cols, cell_size_m, chosen_spacing = mission_settings(rows, cols, cell_size_m)
    if spacing_m is _FROM_SETTINGS:
        spacing_m = chosen_spacing
    if rows < 1 or cols < 1 or cell_size_m <= 0:
        raise ValueError("dimensione della missione non valida")
    sampling = P.MISSION_GRAPH_SAMPLING if sampling is None else sampling
    if sampling not in GRAPH_SAMPLINGS:
        raise ValueError(f"metodo dei nodi sconosciuto {sampling!r}: uno di {GRAPH_SAMPLINGS}")
    if sampling == 'casuale':
        return _build_random_graph(
            rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
            float(min_edge_length), min(float(connection_radius), float(max_edge_length)),
            float(border_margin_m), verbose,
            P.MISSION_RANDOM_DISTRIBUTION if distribution is None else distribution,
            P.MISSION_RANDOM_DENSITY if density_per_m2 is None else density_per_m2,
            P.MISSION_RANDOM_SEED if seed is None else seed,
            P.MISSION_POISSON_MIN_DIST_M if poisson_min_dist_m is None else poisson_min_dist_m)

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

    node_cell = _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell)

    mg = MissionGraph(spacing_m, rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                      ij, xy_local, pairs, lengths, node_cell, cell_center_ids,
                      origin_node_id, float(min_edge_length), reach_m, rings, n_short)
    if verbose:
        print(mg.report())
    return mg


def _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell):
    """Cella (riga, colonna) di ogni nodo: stessa regola di env.get_cell_from_world (round)."""
    cols_idx = np.clip(np.round(xy_local[:, 0] / cell_size_m).astype(np.int64) + start_cell[1],
                       0, cols - 1)
    rows_idx = np.clip(np.round(xy_local[:, 1] / cell_size_m).astype(np.int64) + start_cell[0],
                       0, rows - 1)
    return np.column_stack((rows_idx, cols_idx))


def _build_random_graph(rows, cols, cell_size_m, start_cell, origin_xy, origin_yaw,
                        min_edge_m, reach_m, border_margin_m, verbose,
                        distribution, density_per_m2, seed, poisson_min_dist_m):
    """
    Il grafo con nodi casuali (vedi NODI CASUALI): prima i centri cella, nell'ordine delle
    righe e delle colonne, poi i nodi della distribuzione. Stesso seme, stesso grafo.
    """
    if distribution not in RANDOM_DISTRIBUTIONS:
        raise ValueError(f"distribuzione sconosciuta {distribution!r}: una di {RANDOM_DISTRIBUTIONS}")
    if density_per_m2 is None or float(density_per_m2) <= 0:
        raise ValueError("la densita' dei nodi deve essere positiva")
    if reach_m < min_edge_m:
        raise ValueError(f"lunghezza massima degli archi {reach_m:.2f} m sotto la minima "
                         f"{min_edge_m:.2f} m: nessun arco sarebbe ammesso")
    seed = draw_seed() if seed is None else int(seed)
    rng = np.random.default_rng(seed)

    x_min, x_max, y_min, y_max = _mission_extent(rows, cols, cell_size_m, start_cell)
    box = (x_min + border_margin_m, x_max - border_margin_m,
           y_min + border_margin_m, y_max - border_margin_m)
    if box[1] <= box[0] or box[3] <= box[2]:
        raise ValueError("margine di bordo troppo largo: nessuna area per i nodi")

    # Centri cella: nodi fissi, gli obiettivi dell'esplorazione. Il centro della cella di
    # partenza e' l'origine, cioe' il punto di accensione.
    centers, keys = [], []
    for r in range(rows):
        for c in range(cols):
            x, y = (c - start_cell[1]) * cell_size_m, (r - start_cell[0]) * cell_size_m
            if box[0] <= x <= box[1] and box[2] <= y <= box[3]:
                centers.append((x, y))
                keys.append((r, c))
    centers = np.array(centers, dtype=np.float64).reshape(-1, 2)
    rand_xy, note = _random_points(distribution, density_per_m2, box, centers, rng,
                                   poisson_min_dist_m)
    xy_local = np.vstack((centers, rand_xy))
    cell_center_ids = {key: k for k, key in enumerate(keys)}
    if verbose and len(cell_center_ids) < rows * cols:
        print(f"[GRAFO] {rows * cols - len(cell_center_ids)} centri cella fuori dall'area "
              f"(margine di bordo {border_margin_m:.2f} m): quelle celle non hanno obiettivo.")
    d0 = np.hypot(xy_local[:, 0], xy_local[:, 1])
    origin_node_id = int(np.argmin(d0)) if len(d0) and d0.min() < 1e-9 else None

    pairs, lengths, n_short = _radius_pairs(xy_local, min_edge_m, reach_m)
    node_cell = _node_cells_of(xy_local, rows, cols, cell_size_m, start_cell)
    mg = MissionGraph(1.0 / math.sqrt(float(density_per_m2)), rows, cols, cell_size_m,
                      start_cell, origin_xy, origin_yaw,
                      -np.ones((len(xy_local), 2), dtype=np.int64), xy_local, pairs, lengths,
                      node_cell, cell_center_ids, origin_node_id, min_edge_m, reach_m, None,
                      n_short, sampling='casuale', distribution=distribution,
                      density_per_m2=float(density_per_m2), seed=seed,
                      covering=measured_covering(xy_local, box), sampling_note=note)
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
        int(d['n_short_pairs']),
        # file salvati prima dei nodi casuali: reticolo
        **({} if 'sampling' not in d.files else dict(
            sampling=str(d['sampling']), distribution=(str(d['distribution']) or None),
            density_per_m2=(None if float(d['density_per_m2']) < 0 else float(d['density_per_m2'])),
            seed=(None if int(d['seed']) < 0 else int(d['seed'])),
            covering=(None if float(d['covering'][0]) < 0 else tuple(float(v) for v in d['covering'])),
            sampling_note=str(d['sampling_note']))))


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


def nodes_per_cell(mg):
    """
    Nodi in ogni cella, (righe, colonne), contando un nodo che sta esattamente sul confine
    fra due celle per meta' in ognuna (un quarto negli incroci). Il conteggio di node_cell
    invece assegna quei nodi tutti a una delle due celle (stessa regola della missione), e
    per il reticolo, che ha intere file sui confini, fa sembrare piu' piene alcune celle.
    I nodi sul bordo esterno dell'area contano per intero nella loro cella.
    """
    out = np.zeros((mg.rows, mg.cols), dtype=np.float64)
    eps = 1e-9

    def owners(u, n):
        # u: posizione in unita' di cella, con i centri sugli interi; confini a k + 0.5.
        b = u - 0.5
        if abs(b - round(b)) < eps:
            cand = [int(round(b)), int(round(b)) + 1]
        else:
            cand = [int(round(u))]
        return [k for k in cand if 0 <= k < n] or [min(max(int(round(u)), 0), n - 1)]

    for x, y in mg.xy_local:
        cs = owners(x / mg.cell_size_m + mg.start_cell[1], mg.cols)
        rs = owners(y / mg.cell_size_m + mg.start_cell[0], mg.rows)
        w = 1.0 / (len(cs) * len(rs))
        for r in rs:
            for c in cs:
                out[r, c] += w
    return out


def lattice_matched_density(**kw):
    """
    La densita' dei nodi casuali che da' lo STESSO numero totale di nodi del reticolo con
    questi parametri (bordo esterno compreso, centri cella a parte, come li conta
    _build_random_graph). Serve a confrontare i metodi a parita' di nodi.
    """
    kw = {k: v for k, v in kw.items()
          if k not in ('sampling', 'distribution', 'density_per_m2', 'seed', 'verbose')}
    ref = build_mission_graph(sampling='reticolo', verbose=False, **kw)
    margin = float(kw.get('border_margin_m', 0.0))
    x0, x1, y0, y1 = _mission_extent(ref.rows, ref.cols, ref.cell_size_m, ref.start_cell)
    area = (x1 - x0 - 2 * margin) * (y1 - y0 - 2 * margin)
    return (ref.n_nodes - len(ref.cell_center_ids)) / area


def compare_methods_png(path, zoom=(-2.0, 2.0, -2.0, 2.0), seed=None,
                        methods=('reticolo',) + RANDOM_DISTRIBUTIONS, verbose=False, **kw):
    """
    I metodi dei nodi affiancati (default: il reticolo e le quattro distribuzioni casuali),
    con gli stessi parametri (missione, archi, densita') e lo stesso seme, per vederne la
    differenza a colpo d'occhio. In fondo la tabella: nodi, archi, grado medio, copertura
    massima e al 95%. zoom None = tutta l'area. Seme None = estratto a caso, scritto nel
    titolo. verbose: stampa anche il riepilogo [GRAFO] di ogni grafo.
    density_per_m2 None = la densita' che da' lo stesso numero di nodi del reticolo
    (lattice_matched_density).
    Restituisce (path, grafi).
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    seed = draw_seed() if seed is None else int(seed)
    kw = {k: v for k, v in kw.items() if k not in ('sampling', 'distribution')}
    if kw.get('density_per_m2') is None:
        kw['density_per_m2'] = lattice_matched_density(**kw)
    graphs = []
    for m in methods:
        if verbose:
            print(f"\n--- {m}")
        if m == 'reticolo':
            graphs.append(build_mission_graph(sampling='reticolo', verbose=verbose, **kw))
        else:
            graphs.append(build_mission_graph(sampling='casuale', distribution=m, seed=seed,
                                              verbose=verbose, **kw))
    n_cols = 3 if len(graphs) >= 3 else len(graphs) + 1
    n_rows = int(math.ceil((len(graphs) + 1) / n_cols))
    if zoom:
        panel_w, panel_h = 5.6, 5.6
    else:                          # tutta l'area: riquadri con le proporzioni della missione
        g0 = graphs[0]
        ratio = (g0.rows * g0.cell_size_m) / (g0.cols * g0.cell_size_m)
        panel_w, panel_h = 6.4, max(3.0, 6.4 * ratio) + 1.0
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(panel_w * n_cols, panel_h * n_rows + 0.8),
                             squeeze=False)
    for k, (ax, mg) in enumerate(zip(axes.ravel(), graphs)):
        # Archi grigi, piu' chiari e sottili quando sono tanti; in evidenza quelli che partono
        # dal nodo di partenza: si vede quali vicini sono collegati.
        many = mg.n_edges > 20000
        mg.draw(ax, zoom=zoom, legend=(k == 0), title_size=10,
                edge_color='0.88' if many else '0.70', edge_width=0.2 if many else 0.35,
                highlight=mg.origin_node_id)
    for ax in axes.ravel()[len(graphs) + 1:]:
        ax.axis('off')
    # Riquadro dopo l'ultimo grafo: la tabella dei numeri.
    ax = axes.ravel()[len(graphs)]
    ax.axis('off')
    rows = []
    for mg in graphs:
        name = 'reticolo' if not mg.is_random else mg.distribution
        if mg.covering is None:
            # Reticolo: la massima dalla formula (la griglia di misura puo' mancarne il punto).
            cov = (mg.covering_m, measured_covering(mg.xy_local, _mission_extent(
                mg.rows, mg.cols, mg.cell_size_m, mg.start_cell))[1])
        else:
            cov = mg.covering
        per_cell = nodes_per_cell(mg)
        rows.append([name, f"{mg.n_nodes}", f"{per_cell.min():.0f}-{per_cell.max():.0f}",
                     f"{mg.n_edges}", f"{mg.degrees.mean():.1f}",
                     f"{cov[0]:.2f}", f"{cov[1]:.2f}"])
    tab = ax.table(cellText=rows, colLabels=['metodo', 'nodi', 'per cella\n(min-max)', 'archi',
                                             'grado\nmedio', 'copertura\nmassima [m]',
                                             '95% area\nentro [m]'],
                   loc='center', cellLoc='center')
    tab.auto_set_font_size(False)
    tab.set_fontsize(10)
    tab.auto_set_column_width(list(range(7)))
    tab.scale(1.0, 2.4)
    g0 = graphs[0]
    rnd = [g for g in graphs if g.is_random]
    lat = [g for g in graphs if not g.is_random]
    fig.suptitle(f"Missione {g0.rows}x{g0.cols} celle da {g0.cell_size_m:g} m, archi "
                 f"{g0.min_edge_m:g}-{g0.reach_m:g} m"
                 + (f", reticolo passo {lat[0].spacing_m:g} m" if lat else "")
                 + (f", casuali {rnd[0].density_per_m2:.2f} nodi/m2, seme {seed}" if rnd else "")
                 + (f" -- riquadro x {zoom[0]:g}..{zoom[1]:g}, y {zoom[2]:g}..{zoom[3]:g} m"
                    if zoom else ""), fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96), h_pad=3.0)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path, graphs


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
    s_rows, s_cols, s_cell, s_spacing = mission_settings()
    p.add_argument('--rows', type=int, default=s_rows,
                   help=f"righe di celle, a sinistra del robot (default {s_rows})")
    p.add_argument('--cols', type=int, default=s_cols,
                   help=f"colonne di celle, davanti al robot (default {s_cols})")
    p.add_argument('--cell', type=float, default=s_cell,
                   help=f"lato della cella in m (default {s_cell:g})")
    p.add_argument('--spacing', type=float, default=s_spacing,
                   help=f"passo fra i nodi in m, arrotondato a cell/n (default "
                        f"{s_spacing:g}; 0 = suggested_spacing)")
    p.add_argument('--rings', default=str(P.MISSION_LATTICE_RINGS),
                   help="anelli di vicini collegati (k = entro la diagonale dell'anello k), "
                        "o 'tutti' (default: tutti, come in missione)")
    p.add_argument('--grafo', choices=GRAPH_SAMPLINGS, default=P.MISSION_GRAPH_SAMPLING,
                   help=f"come si generano i nodi (default {P.MISSION_GRAPH_SAMPLING})")
    p.add_argument('--distribuzione', choices=RANDOM_DISTRIBUTIONS, default=P.MISSION_RANDOM_DISTRIBUTION,
                   help=f"nodi casuali: distribuzione (default {P.MISSION_RANDOM_DISTRIBUTION})")
    p.add_argument('--densita', type=float, default=P.MISSION_RANDOM_DENSITY,
                   help=f"nodi casuali: nodi per m2 (default {P.MISSION_RANDOM_DENSITY:g})")
    p.add_argument('--seme', type=int, default=P.MISSION_RANDOM_SEED,
                   help="nodi casuali: seme (default: estratto a caso e scritto nel riepilogo)")
    p.add_argument('--dist-min', type=float, default=P.MISSION_POISSON_MIN_DIST_M,
                   help="nodi poisson: distanza minima fra i nodi in m (default 0.75/sqrt(densita'))")
    p.add_argument('--min-edge', type=float, default=P.PRM_MIN_EDGE_M)
    p.add_argument('--max-edge', type=float, default=P.PRM_MAX_EDGE_M)
    p.add_argument('--radius', type=float, default=P.PRM_CONNECTION_RADIUS_M)
    p.add_argument('--margin', type=float, default=0.0, help="margine dal bordo in m")
    p.add_argument('--preview', metavar='PNG', help="salva un disegno del reticolo")
    p.add_argument('--zoom', type=float, nargs=4, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                   help="riquadro del disegno, in coordinate griglia")
    p.add_argument('--save', metavar='NPZ', help="salva il reticolo (si ricarica con load_npz)")
    p.add_argument('--confronto', action='store_true', help="tabella passo x anelli")
    p.add_argument('--confronta-metodi', metavar='PNG',
                   help="reticolo e le quattro distribuzioni casuali affiancati in un PNG "
                        "(stessi parametri e seme; riquadro --zoom, default -2 2 -2 2)")
    p.add_argument('--tempi', nargs='?', const=True, metavar='NPZ',
                   help="misura costruzione da zero contro ricarica da file")
    a = p.parse_args()

    rings = None if str(a.rings).lower() in ('none', 'tutti', 'all', '-1') else int(a.rings)
    if a.spacing is not None and a.spacing <= 0:
        a.spacing = None                 # 0 = passo suggerito da suggested_spacing()
    common = dict(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                  max_edge_length=a.max_edge, connection_radius=a.radius,
                  border_margin_m=a.margin, sampling=a.grafo, distribution=a.distribuzione,
                  density_per_m2=a.densita, seed=a.seme, poisson_min_dist_m=a.dist_min)

    if a.confronta_metodi:
        common.pop('sampling')
        common.pop('distribution')
        common.pop('seed')
        print(f"[GRAFO] confronto salvato in "
              f"{compare_methods_png(a.confronta_metodi, zoom=tuple(a.zoom) if a.zoom else (-2.0, 2.0, -2.0, 2.0), seed=a.seme, spacing_m=a.spacing, rings=rings, **common)[0]}")
        return
    if a.confronto:
        compare(rows=a.rows, cols=a.cols, cell_size_m=a.cell, min_edge_length=a.min_edge,
                sampling='reticolo',
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


# =========================================================================================
# IL GRAFO DURANTE LA MISSIONE: costruzione del PRM, aggiornamento degli archi, celle
# ammesse per la pianificazione
# =========================================================================================
def build_mission_prm(env, verbose=True):
    """
    Crea il grafo della missione (reticolo o nodi casuali, vedi MISSION_GRAPH_SAMPLING) e il
    PRM con i suoi nodi, dalla dimensione della missione e dalla posa di partenza che `env`
    ha gia' dentro (set_origin va fatto PRIMA).

    Restituisce (prm, sampler, mission_lattice): `sampler` da' i nodi cella per cella (vedi
    mission_graph.LatticeSampler), `mission_lattice` e' il MissionGraph.
    Gli ARCHI non vengono creati qui: servono i dati di terreno, vedi rebuild_prm_edges.
    """
    mg = build_mission_graph(
        env=env, spacing_m=P.MISSION_LATTICE_SPACING_M, rings=P.MISSION_LATTICE_RINGS,
        min_edge_length=P.PRM_MIN_EDGE_M, max_edge_length=P.PRM_MAX_EDGE_M,
        connection_radius=P.PRM_CONNECTION_RADIUS_M, verbose=verbose,
        sampling=P.MISSION_GRAPH_SAMPLING, distribution=P.MISSION_RANDOM_DISTRIBUTION,
        density_per_m2=P.MISSION_RANDOM_DENSITY, seed=P.MISSION_RANDOM_SEED)
    prm = prm_graph.PRM(min_edge_length=P.PRM_MIN_EDGE_M, max_edge_length=mg.reach_m,
                        connection_radius=P.PRM_CONNECTION_RADIUS_M)
    prm.mission_graph = mg          # lo rilegge rebuild_prm_edges
    prm.set_detour_limit(P.PLAN_MAX_DETOUR_RATIO, P.PLAN_DETOUR_SLACK_M)
    mg.into_prm(prm)
    return prm, mg.as_sampler(), mg


def node_cells(prm, env):
    """
    {id nodo: (riga, colonna)}. Dal reticolo quando c'e' (mission_graph.node_cell, stessa
    regola di env.get_cell_from_world), altrimenti da env. In cache sul PRM: i nodi non
    cambiano durante la missione.
    """
    cache = getattr(prm, '_node_cell_cache', None)
    if cache is None:
        cache = {}
        prm._node_cell_cache = cache
    mg = getattr(prm, 'mission_graph', None)
    for nid, (x, y) in prm.nodes.items():
        if nid in cache:
            continue
        if mg is not None and 0 <= nid < mg.n_nodes:
            r, c = mg.node_cell[nid]
            cache[nid] = (int(r), int(c))
        else:
            cache[nid] = tuple(int(v) for v in env.get_cell_from_world(x, y))
    return cache


def graph_reaches_cell(prm, env, from_xy, row, col, global_map=None):
    """
    Dal punto from_xy (il waypoint da cui si tenterebbe la cella) il grafo arriva ad almeno un
    nodo della cella (row, col)? Stesse regole del tentativo: con PLAN_ONLY_EXPLORED_CELLS si
    passa solo dalle celle visitate, dalla cella di partenza e dalla cella obiettivo; archi e
    nodi come sono adesso (un nodo con penalita' infinita non conta). E' una condizione
    NECESSARIA: se e' False il tentativo fallirebbe di sicuro con "GRAFO DISCONNESSO"; se e'
    True decide il tentativo (punto obiettivo, tetto alla deviazione, fronte).
    Ricerca in ampiezza, nessun effetto sul grafo. None se il grafo non ha nodi.
    """
    start = prm.anchor_node(float(from_xy[0]), float(from_xy[1]), global_map,
                            spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
                            ignore_near_m=prm_graph.ANCHOR_IGNORE_NEAR_M)
    if start is None:
        return None
    cells = node_cells(prm, env)
    target = (int(row), int(col))
    if P.PLAN_ONLY_EXPLORED_CELLS:
        ok_cells = {(r, c) for r in range(env.rows) for c in range(env.cols)
                    if env.is_cell_visited(r, c)}
        ok_cells.add(target)
        ok_cells.add(tuple(int(v) for v in env.get_cell_from_world(from_xy[0], from_xy[1])))
    else:
        ok_cells = None
    seen, queue = {start}, [start]
    while queue:
        u = queue.pop()
        if cells.get(u) == target and np.isfinite(prm.node_penalty(u)):
            return True
        for v, _w in prm.edges.get(u, ()):
            if v in seen:
                continue
            if ok_cells is not None and cells.get(v) not in ok_cells:
                continue
            seen.add(v)
            queue.append(v)
    return False


def restrict_planning_to_explored_cells(prm, env, target_row, target_col, robot_xy):
    """
    Limita la ricerca del percorso ai nodi delle celle visitate, della cella del robot e
    della cella obiettivo (vedi PLAN_ONLY_EXPLORED_CELLS). Da chiamare prima di ogni
    tentativo di cella: le celle visitate cambiano fra un tentativo e l'altro.
    """
    if not P.PLAN_ONLY_EXPLORED_CELLS:
        prm.set_allowed_nodes(None)
        return
    cells = {(r, c) for r in range(env.rows) for c in range(env.cols) if env.is_cell_visited(r, c)}
    cells.add((int(target_row), int(target_col)))
    cells.add(tuple(int(v) for v in env.get_cell_from_world(robot_xy[0], robot_xy[1])))
    allowed = [nid for nid, cell in node_cells(prm, env).items() if cell in cells]
    prm.set_allowed_nodes(allowed)
    print(f"[PIANO] Percorsi ammessi solo nelle celle {sorted(cells)}: {len(allowed)} nodi su "
          f"{len(prm.nodes)}. Se serve passare da una cella non visitata, rinuncio a questo "
          f"lato e passo a un'altra cella.")


def robot_on_node(prm, node_id, x, y):
    """True se il robot e' praticamente sul nodo (vedi prm_graph.ANCHOR_ON_NODE_M)."""
    pos = prm.get_node_position(node_id)
    return pos is not None and float(np.hypot(pos[0] - x, pos[1] - y)) <= prm_graph.ANCHOR_ON_NODE_M


def rebuild_prm_edges(prm, global_map):
    """
    Rivaluta gli archi del PRM sui dati correnti, sulle coppie del reticolo gia' calcolate
    (vedi mission_graph.MissionGraph.add_edges_to_prm). Veti e pesi li calcola
    PRM._evaluate_and_add_edge.
    """
    return prm.mission_graph.add_edges_to_prm(
        prm, global_map=global_map,
        edge_safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M,
        incremental=P.INCREMENTAL_GRAPH_UPDATE)


if __name__ == '__main__':
    _main()