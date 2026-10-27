"""
Griglia locale di Spot e mappe globali: lettura delle griglie, correzione del terreno, pendenza
degli archi, maschera degli ostacoli, controllo di rotazione, mappe di occupazione, di spazio
libero e del terreno. Qui stanno anche tutte le soglie condivise da pianificatore, fronte e
scelta dell'andatura: si cambiano SOLO qui.
"""


import numpy as np
from scipy.ndimage import uniform_filter, distance_transform_edt, binary_dilation
# L'SDK di Spot serve solo a LocalGrid (la lettura delle griglie dal robot). Senza SDK il resto
# del modulo funziona: mappe globali, pendenze, soglie, cosi' lo possono importare anche
# mission_graph, prm_graph, parametri e mission_viewer su una macchina senza SDK.
try:
    import bosdyn.client
    from bosdyn.client.frame_helpers import *
    from bosdyn.client.frame_helpers import get_a_tform_b
    from bosdyn.api import local_grid_pb2
    from bosdyn.client.local_grid import LocalGridClient
except ImportError:
    get_a_tform_b = local_grid_pb2 = LocalGridClient = None

import pickle
import time
import os

# =========================================================================
# SOGLIE DI OSTACOLO, PENDENZA E RUGOSITA'
#   OBSTACLE_THRESHOLD  una cella e' ostacolo se obstacle_distance <= questo (m)
#   SLOPE_THRESHOLD     pendenza oltre cui un arco si scarta (tangente). La pendenza si misura
#                       su una base di 0.48 m (vedi SLOPE_BASELINE_M), quindi un GRADINO NETTO
#                       di altezza H vale H / 0.48: 0.6 corrisponde a un gradino di ~29 cm,
#                       il limite di Spot (~30 cm). Per questo non serve una misura separata
#                       dei gradini.
#   ROUGH_THRESHOLD     rugosita' (deviazione standard della quota, m) oltre cui una cella e'
#                       ostacolo
# =========================================================================
OBSTACLE_THRESHOLD = 0.15   # obstacle_distance (m): distance <= this -> blocked
SLOPE_THRESHOLD = 0.6       # gradient (tan of slope): tan(28deg)=~0.53, tan(25deg)=~0.46
ROUGH_THRESHOLD = 0.15      # local roughness (std-dev of terrain height, m)

# Il layer `terrain_valid` di Spot contiene 0 e 1, ma dopo scala e offset lo 0 diventa
# +/-5.5e-17 con segno variabile: si confronta con 0.5, insensibile al residuo.
TERRAIN_VALID_MIN = 0.5

# =========================================================================
# FILTRO SULL'ALTEZZA DEL SUOLO (vedi LocalGrid.correct_terrain)
#
# Le quote di `terrain` sono rispetto al riferimento fissato all'accensione. Il suolo su cui
# il robot sta in piedi si stima come mediana delle celle valide entro GROUND_REF_RADIUS_M;
# le celle oltre GROUND_HEIGHT_TOLERANCE_M dal suolo si trattano in modo ASIMMETRICO:
#   - SOPRA: non e' superficie calpestabile (muro, albero, cella mai scritta). Esclusa da
#     pendenza e rugosita' e riempita col suolo vicino; l'ostacolo resta in obstacle_distance.
#   - SOTTO: puo' essere un vuoto vero (muretto, scala che scende, fosso): non si tocca,
#     riempirla nasconderebbe un dirupo.
# 1.2 m: a ~1.9 m dal robot (bordo della finestra) sono gia' ~32 gradi, oltre il limite di
# Spot, quindi nessuna salita affrontabile viene scartata.
# =========================================================================
GROUND_HEIGHT_TOLERANCE_M = 1.2
GROUND_REF_RADIUS_M = 0.7            # celle sotto/attorno al robot usate per stimare il suolo
GROUND_REF_FALLBACK_RADIUS_M = 1.2   # se sotto il robot ci sono troppe poche celle valide
GROUND_REF_MIN_CELLS = 50

# -------------------------------------------------------------------------
# CELLE MAI SCRITTE (vedi LocalGrid._cells_never_written)
# Le celle che i sensori non hanno mai scritto restano tutte alla stessa quota esatta (un
# "plateau"). Spot le dichiara quasi sempre non valide; questa regola cattura il resto.
# -------------------------------------------------------------------------
UNWRITTEN_MATCH_FRACTION = 0.25    # tolleranza: 0.25 * passo di quantizzazione
UNWRITTEN_MATCH_FALLBACK_M = 1e-4  # se il passo non e' noto
UNWRITTEN_MIN_CELLS = 100          # celle che devono condividere la quota perche' sia un plateau
UNWRITTEN_MIN_SPOT_INVALID = 0.80  # frazione del plateau che Spot deve marcare non valida

# =========================================================================
# INGOMBRO DEL ROBOT E FRONTE SICURO (vedi arcVerification.compute_safe_frontier)
#   FRONTIER_CLEARANCE_M: distanza minima da un ostacolo per la LINEA CENTRALE del percorso:
#     mezza larghezza (0.25) + aria (ROBOT_CLEARANCE_AIR_M). Si legge direttamente da
#     obstacle_distance. Da qui viene anche il veto degli archi (PRM_EDGE_SAFETY_MARGIN_M).
#   ROBOT_CLEARANCE_AIR_M: l'aria fra corpo e ostacolo. 0 = solo l'ingombro del corpo: il
#     robot non pianifica ne' parte dove il corpo non entra, ma senza margine. Un corridoio
#     utile e' largo almeno 2 * FRONTIER_CLEARANCE_M, e piu' e' piccolo il margine piu' rado
#     puo' essere il grafo per trovare i passaggi stretti (vedi mission_graph, ALIASING).
#   FRONTIER_BODY_HALF_LENGTH_M: il muso sta 0.55 m davanti al centro; il centro si ferma a
#     meta' lunghezza + FRONTIER_MARGIN_M prima del primo punto non libero.
#   FRONTIER_MIN_CLEARANCE_M / FRONTIER_ESCAPE_DIST_M: se il robot e' GIA' piu' vicino di
#     FRONTIER_CLEARANCE_M a qualcosa (spazio stretto), nel primo metro basta non
#     avvicinarsi di piu', mai sotto FRONTIER_MIN_CLEARANCE_M: cosi' puo' uscirne.
#   FRONTIER_ROBOT_RADIUS_M: entro questo raggio le celle "non viste" non fermano il fronte
#     (e' il terreno sotto il robot, coperto dal corpo).
#   FRONTIER_ABORT_AIR_M: si PARTE con FRONTIER_CLEARANCE_M ai lati; a movimento in corso ci
#     si ferma solo sotto ROBOT_HALF_WIDTH_M + FRONTIER_ABORT_AIR_M (0.25), cioe' se
#     qualcosa entra davvero nell'ingombro (decisione dell'utente: "se lo vede libero va").
# =========================================================================
ROBOT_HALF_LENGTH_M = 0.55          # Spot: 1.10 x 0.50 m
ROBOT_HALF_WIDTH_M = 0.25
ROBOT_CLEARANCE_AIR_M = 0.0         # aria fra corpo e ostacolo (0.05 = con margine)
FRONTIER_CLEARANCE_M = ROBOT_HALF_WIDTH_M + ROBOT_CLEARANCE_AIR_M      # = 0.25
FRONTIER_MIN_CLEARANCE_M = 0.05
FRONTIER_ESCAPE_DIST_M = 1.0
FRONTIER_BODY_HALF_LENGTH_M = ROBOT_HALF_LENGTH_M                       # = 0.55
FRONTIER_MARGIN_M = 0.10
FRONTIER_ABORT_AIR_M = 0.0

# =========================================================================
# ROTAZIONE SUL POSTO (vedi rotation_is_clear)
# Ruotando sul posto gli spigoli del corpo descrivono un cerchio di raggio pari alla
# semidiagonale, ~0.60 m. Si controllano le celle che il corpo spazza davvero.
#   ROTATION_CLEARANCE_AIR_M: aria sulla rotazione, 0: si rifiuta solo se il corpo compenetra
#     davvero (un falso "non posso ruotare" costa caro: arretramenti in catena).
#   ROTATION_CLEARANCE_M: raggio del prefiltro (semidiagonale + aria).
#   ROTATION_CHECK_MIN_DYAW_DEG: sotto questa rotazione lo spazio spazzato e' trascurabile.
# =========================================================================
ROTATION_CLEARANCE_AIR_M = 0.0
ROTATION_CLEARANCE_M = float(np.hypot(ROBOT_HALF_LENGTH_M, ROBOT_HALF_WIDTH_M)) + ROTATION_CLEARANCE_AIR_M  # ~0.604
ROTATION_CHECK_MIN_DYAW_DEG = 10.0

# Compenetrazione tollerata nella rotazione, contro i MURI (vedi ROTATION_CHECK_PENETRATION per
# gli ostacoli sottili). 0.15 m: assorbe l'errore di registrazione della griglia (5-10 cm) e il
# rumore di obstacle_distance; con meno, in 6 scansioni su 46 di una missione il robot non
# trovava nessuna direzione in cui girarsi, mentre GraphNav in quei punti ruotava e passava.
# Se in prova si vedono sfioramenti dei muri durante le rotazioni, va riportato a 0.10.
ROTATION_NOISE_TOLERANCE_M = 0.15

# Metrica del controllo di rotazione: PENETRAZIONE = profondita' della cella dentro il corpo
# meno obstacle_distance. Il minimo di obstacle_distance da solo non basta: dentro un ostacolo
# sottile non scende sotto meta' del suo spessore (un palo da 10 cm non va sotto -0.05), e con
# la tolleranza a 0.15 un palo che entra nel corpo verrebbe approvato. Per un muro le due
# metriche coincidono. Le celle sotto il corpo ADESSO si escludono: il robot ci sta sopra.
ROTATION_CHECK_PENETRATION = True


def body_extents(heading, motion_dir):
    """
    Ingombro del corpo rispetto alla direzione di marcia: (meta' lungo la marcia,
    meta' di traverso). heading = orientamento del corpo, motion_dir = direzione del
    moto, in radianti. Con heading == motion_dir: (0.55, 0.25); di lato: (0.25, 0.55).
    """
    a = motion_dir - heading
    c, s = abs(np.cos(a)), abs(np.sin(a))
    along = ROBOT_HALF_LENGTH_M * c + ROBOT_HALF_WIDTH_M * s
    across = ROBOT_HALF_LENGTH_M * s + ROBOT_HALF_WIDTH_M * c
    return float(along), float(across)
FRONTIER_SAMPLE_STEP_M = 0.03        # un campione per cella
FRONTIER_ROBOT_RADIUS_M = 0.60
FRONTIER_SLOPE_MIN_LENGTH_M = 0.50   # sotto questa lunghezza la pendenza non e' stimabile

# Passo angolare con cui si campiona la rotazione in rotation_is_clear(). 5 gradi: a
# raggio 0.60 m sono 5 cm di arco, cioe' meno di due celle, quindi fra due pose campionate
# non puo' nascondersi un ostacolo che il corpo attraverserebbe.
ROTATION_SWEEP_STEP_DEG = 5.0


def rotation_is_clear(obstacle_dist, origin_x, origin_y, cell_size, robot_x, robot_y,
                      robot_yaw, dyaw, half_along=ROBOT_HALF_LENGTH_M,
                      half_across=ROBOT_HALF_WIDTH_M, air=ROTATION_CLEARANCE_AIR_M,
                      step_deg=ROTATION_SWEEP_STEP_DEG):
    """
    Il robot puo' ruotare sul posto di `dyaw` radianti senza toccare niente?

    Controlla le celle che il corpo spazza DAVVERO durante la rotazione (un settore), invece
    di chiedere che sia libero tutto il cerchio circoscritto: gli ostacoli stanno dove stanno,
    e il cerchio intero rifiuterebbe molte rotazioni sicure.

    Il rettangolo e' gia' gonfiato dell'aria (`air`). Si lavora su una finestra ritagliata
    attorno al robot (~0.3 ms per un controllo normale, ~0.9 ms per 180 gradi).

    Args:
        obstacle_dist: griglia 2D (num_y, num_x) di obstacle_distance, in metri.
        origin_x, origin_y, cell_size: geometria della griglia nel frame VISION.
        robot_x, robot_y, robot_yaw: posa attuale del robot.
        dyaw: rotazione richiesta, in radianti (segno compreso).
        half_along, half_across: meta' ingombro del corpo (default Spot: 0.55 x 0.25).
        air: aria lasciata attorno al corpo.
        step_deg: passo angolare di campionamento.

    Returns:
        (libero: bool, margine: float) -- margine e' la peggiore penetrazione (col segno
        cambiato) sulle celle spazzate, oppure +inf se la finestra cade fuori dalla griglia:
        in quel caso non si sa nulla e si risponde True (decidera' il fronte sicuro).
    """
    if obstacle_dist is None or cell_size is None:
        return True, float('inf')
    reach = float(np.hypot(half_along, half_across)) + air
    num_rows, num_cols = obstacle_dist.shape
    c0 = max(int(np.floor((robot_x - reach - origin_x) / cell_size)), 0)
    c1 = min(int(np.ceil((robot_x + reach - origin_x) / cell_size)), num_cols - 1)
    r0 = max(int(np.floor((robot_y - reach - origin_y) / cell_size)), 0)
    r1 = min(int(np.ceil((robot_y + reach - origin_y) / cell_size)), num_rows - 1)
    if c1 <= c0 or r1 <= r0:
        return True, float('inf')

    rows, cols = np.mgrid[r0:r1 + 1, c0:c1 + 1]
    dx = origin_x + (cols + 0.5) * cell_size - robot_x
    dy = origin_y + (rows + 0.5) * cell_size - robot_y
    window = obstacle_dist[r0:r1 + 1, c0:c1 + 1]

    num_steps = max(2, int(abs(np.degrees(dyaw)) / step_deg) + 2)
    worst = float('inf')
    if ROTATION_CHECK_PENETRATION:
        # Celle sotto il corpo ADESSO: il robot ci sta sopra, sono libere per definizione.
        c_now, s_now = np.cos(robot_yaw), np.sin(robot_yaw)
        under_now = ((np.abs(dx * c_now + dy * s_now) <= half_along) &
                     (np.abs(-dx * s_now + dy * c_now) <= half_across))
    for t in np.linspace(0.0, float(dyaw), num_steps):
        a = robot_yaw + t
        ca, sa = np.cos(a), np.sin(a)
        along = np.abs(dx * ca + dy * sa)
        across = np.abs(-dx * sa + dy * ca)
        in_body = (along <= half_along + air) & (across <= half_across + air)
        if ROTATION_CHECK_PENETRATION:
            in_body &= ~under_now
            if in_body.any():
                # PENETRAZIONE = profondita' della cella dentro il corpo - obstacle_distance.
                depth = np.minimum(half_along + air - along, half_across + air - across)[in_body]
                worst = min(worst, float((window[in_body] - depth).min()))
        elif in_body.any():
            worst = min(worst, float(window[in_body].min()))
    return worst >= -ROTATION_NOISE_TOLERANCE_M, worst

# Distanza dagli ostacoli con cui il PIANIFICATORE sceglie gli archi: quella del fronte piu'
# PLAN_CLEARANCE_EXTRA_M. Con la stessa distanza del fronte i percorsi passavano esattamente
# al limite, e il rumore della scansione dal vivo (1-2 cm) li faceva rifiutare: nella prova
# del 22-10 15:24, 18 giri del fronte su 25 fermati da un ostacolo a 0.23-0.24 m con minimo
# 0.25, con avanzamenti da 0.2-0.5 m. Il passaggio piu' stretto lo decide sempre il fronte;
# il pianificatore lascia qualche centimetro in piu'. 0 = stessa distanza del fronte.
PLAN_CLEARANCE_EXTRA_M = 0.03
PLAN_CLEARANCE_M = FRONTIER_CLEARANCE_M + PLAN_CLEARANCE_EXTRA_M          # = 0.28

# Margine con cui il PRM scarta gli archi vicini a celle occupate della mappa globale. Le
# celle occupate hanno obstacle_distance <= OBSTACLE_THRESHOLD, quindi un arco ammesso passa
# ad almeno OBSTACLE_THRESHOLD + margine = PLAN_CLEARANCE_M dall'ostacolo.
PRM_EDGE_SAFETY_MARGIN_M = PLAN_CLEARANCE_M - OBSTACLE_THRESHOLD   # = 0.13

# =========================================================================
# MAPPA GLOBALE DEGLI OSTACOLI (vedi GlobalGrid.update)
#   GLOBAL_OCC_CONFIRM_OBS: una cella diventa occupata dopo tante osservazioni CONSECUTIVE di
#     ostacolo: la mappa ricorda gli ostacoli fra un tentativo e l'altro e non puo' credere a
#     una scansione isolata. La prima osservazione di ostacolo azzera le prove di "libero"
#     (reazione rapida a un ostacolo nuovo).
#   GLOBAL_OCC_CONFIRM_WALKED: conferme richieste dove il robot e' passato col corpo: averci
#     camminato e' una prova piu' forte di una lettura da lontano. Le celle non diventano
#     immuni (il mondo cambia), ma servono piu' conferme.
#   GLOBAL_OCC_MAX_COUNT: tetto del contatore di prove.
# =========================================================================
GLOBAL_OCC_CONFIRM_OBS = 2
GLOBAL_OCC_MAX_COUNT = 4
GLOBAL_OCC_COARSE_CELLS = 8          # lato del blocco dell'indice a grana grossa (8 x 3 cm = 24 cm)
GLOBAL_OCC_CONFIRM_WALKED = 4

# =========================================================================
# MAPPA DELLO SPAZIO LIBERO (vedi GlobalGrid.update_clearance)
# Per ogni cella ricorda QUANTO spazio c'era (obstacle_distance), non solo "ostacolo si/no":
# cosi' il pianificatore potrebbe applicare lo stesso test del fronte sulla stessa grandezza.
# Statistica: media delle osservazioni prese entro CLEARANCE_NEAR_M dal robot; dove non ce ne
# sono, media di tutte.
# CLEARANCE_MAP_DECIDES = False: la mappa impara e la riga [SPAZIO] del log misura il
# disaccordo, ma decide la mappa binaria (sulle missioni del 12-13/10 lo spazio libero non ha
# dato vantaggi: vedi CHANGELOG 2026-10-14).
# =========================================================================
CLEARANCE_NEAR_M = 1.0
CLEARANCE_MAP_DECIDES = False

# =========================================================================
# SCELTA DELL'ANDATURA (vedi easy_walk.select_mobility_params)
# Tre livelli dalla pendenza (col corpo) e dalla rugosita' del tratto da percorrere; decide
# la peggiore delle due. In altezza di gradino netto (H / 0.48): PLAIN sotto ~7 cm, HARD da
# ~12 cm. Sono soglie di "quanta cura", molto sotto quelle di ostacolo.
# =========================================================================
GAIT_PLAIN_SLOPE_MAX = 0.15     # below this on BOTH -> PLAIN tier (flat ground)
GAIT_PLAIN_ROUGH_MAX = 0.05
GAIT_MODERATE_SLOPE_MAX = 0.25  # below this on BOTH -> MODERATE tier; at/above -> HARD tier
GAIT_MODERATE_ROUGH_MAX = 0.10

# ground_mu_hint per livello (bosdyn.api.spot.TerrainParams): 0.8 e' il default di Spot (buona
# aderenza); valori piu' bassi gli dicono di aspettarsi meno attrito e appoggiare con piu' cura.
GAIT_MU_PLAIN = 0.8
GAIT_MU_MODERATE = 0.6
GAIT_MU_HARD = 0.4

# Velocita' massime per livello (m/s lineare, rad/s rotazione), pensate per l'esterno. Vedi
# movements.DEFAULT_MAX_LINEAR_VEL_MPS per l'avvertenza su min/max.
GAIT_VEL_PLAIN = 0.5
GAIT_VEL_MODERATE = 0.35
GAIT_VEL_HARD = 0.25
GAIT_ANG_PLAIN = 0.6
GAIT_ANG_MODERATE = 0.5
GAIT_ANG_HARD = 0.4

# Peso della pendenza LATERALE (rollio) rispetto a quella longitudinale nel costo degli archi
# (beta_lateral = beta * questo). 1.0 = stesso peso. Non scarta archi: serve solo a preferire,
# fra due archi ammessi, quello meno esposto al rollio.
LATERAL_SLOPE_COST_MULTIPLIER = 1.0

# Quanto vicino a SLOPE_THRESHOLD conta come "quasi al limite" nei log e nei salvataggi di
# diagnostica. Non influisce su nessuna decisione.
NEAR_THRESHOLD_MARGIN_FRACTION = 0.15

# =========================================================================
# PROFILO DI PENDENZA DI UN ARCO (vedi compute_arc_slope_profile)
#   SLOPE_BASELINE_M: MEZZA base della differenza di quota. La differenza e' centrata, k celle
#     avanti e k indietro con k = round(0.25 / 0.03) = 8: il dislivello si misura su ~0.48 m,
#     circa un passo. Su una base piu' corta la faccia di un mobile darebbe pendenze enormi;
#     su questa un gradino netto di H vale H / 0.48.
#   LATERAL_SLICE_HALF_WIDTH_M: mezza larghezza della fetta perpendicolare per il rollio:
#     larga quanto il robot (0.50 m) piu' margine. Oltre quella fascia non e' rollio.
#   MIN_ARC_SAMPLE_FRACTION: frazione minima di campioni con dati perche' l'arco abbia una
#     pendenza (has_data); sotto, l'arco e' "senza dati", che non vuol dire piatto.
#   ARC_BODY_EXTENSION_M: di quanto si allunga l'arco a OGNI estremo per la pendenza "col
#     corpo", usata per veto, costo e andatura: il robot fermo su un nodo ha le zampe 0.55 m
#     davanti e dietro, quindi sente il terreno oltre i due nodi. Il fronte resta sull'arco fra
#     i nodi (piu' permissivo del PRM). 0 = solo l'arco.
# =========================================================================
SLOPE_BASELINE_M = 0.25
LATERAL_SLICE_HALF_WIDTH_M = 0.30
MIN_ARC_SAMPLE_FRACTION = 0.5
ARC_BODY_EXTENSION_M = ROBOT_HALF_LENGTH_M


def fill_invalid_nearest(values_2d, invalid_mask):
    """
    Riempie le celle non attendibili (invalid_mask True) con il valore della cella
    attendibile piu' vicina: celle che Spot dichiara non valide e impronta del robot.

    Restituisce una copia; se tutto e' valido, o tutto non valido, i valori invariati.
    """
    if not np.any(invalid_mask) or not np.any(~invalid_mask):
        return values_2d.copy()
    nearest_indices = distance_transform_edt(invalid_mask, return_distances=False, return_indices=True)
    return values_2d[tuple(nearest_indices)]


def _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size, xs, ys, valid_2d=None):
    """
    Quote della griglia nelle posizioni (xs, ys), array di qualunque forma.

    Restituisce (quote, dentro): dove non c'e' dato la quota e' NaN e il campione resta al
    suo posto (non viene compattato via, che inventerebbe pendenze fra punti non vicini).
    Con valid_2d i campioni su celle non attendibili valgono NaN come quelli fuori griglia,
    e `dentro` vuol dire "dentro E attendibile". Usa np.floor e non int(), che sbaglia cella
    per coordinate negative.
    """
    num_rows, num_cols = terrain_2d.shape
    cols = np.floor((xs - grid_origin_x) / cell_size).astype(np.int64)
    rows = np.floor((ys - grid_origin_y) / cell_size).astype(np.int64)
    inside = (rows >= 0) & (rows < num_rows) & (cols >= 0) & (cols < num_cols)
    out = np.full(np.shape(xs), np.nan, dtype=np.float64)
    if valid_2d is not None:
        ok = inside.copy()
        ok[inside] = np.asarray(valid_2d, dtype=bool)[rows[inside], cols[inside]]
        inside = ok
    if np.any(inside):
        out[inside] = terrain_2d[rows[inside], cols[inside]]
    return out, inside


def p90(values):
    """
    90 percentile, IDENTICO a float(np.percentile(values, 90)) (interpolazione lineare) per
    valori finiti, ma senza il costo fisso di np.percentile, che qui si chiama molto spesso.
    """
    a = np.asarray(values, dtype=np.float64).ravel()
    n = a.size
    if n == 1:
        return float(a[0])
    v = (n - 1) * 0.9
    lo = int(np.floor(v))
    hi = min(lo + 1, n - 1)
    part = np.partition(a, (lo, hi)) if hi != lo else np.partition(a, lo)
    x0, x1 = float(part[lo]), float(part[hi])
    t = v - lo
    d = x1 - x0
    return x1 - d * (1.0 - t) if t >= 0.5 else x0 + d * t


def _baseline_gradient(heights, step, baseline_m):
    """
    Pendenza come differenza centrata su una base FISICA (vedi SLOPE_BASELINE_M), lungo
    l'ultimo asse: va bene per un profilo 1D (longitudinale) e per una pila di fette 2D
    (laterale), che vengono messe in comune. Dove manca un campione il risultato e' NaN e
    viene scartato. Restituisce l'array 1D dei moduli di pendenza validi (anche vuoto).
    """
    n = heights.shape[-1]
    k = max(1, int(round(baseline_m / step)))
    if n <= 2 * k:
        k = max(1, (n - 1) // 2)
    if n <= 2 * k:
        return np.empty(0)
    g = np.abs(heights[..., 2 * k:] - heights[..., :-2 * k]) / (2 * k * step)
    return g[np.isfinite(g)]


def compute_arc_slope_profile(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                              x1, y1, x2, y2, perp_sample_spacing_m=0.10,
                              baseline_m=SLOPE_BASELINE_M,
                              lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M,
                              min_sample_fraction=MIN_ARC_SAMPLE_FRACTION, valid_2d=None,
                              obstacle_2d=None, body_ext_m=0.0):
    """
    Pendenza che Spot sentirebbe percorrendo l'arco (x1, y1) -> (x2, y2), misurata sulle
    quote della griglia passata. Funzione pura dei dati: la usano il PRM (sulla scansione
    del momento) e il fronte (sulla sua fotografia).

      - LONGITUDINALE (beccheggio): quote ogni cella lungo la linea dell'arco, differenze
        su SLOPE_BASELINE_M (vedi _baseline_gradient).
      - LATERALE (rollio): ogni perp_sample_spacing_m lungo l'arco una fetta perpendicolare
        larga quanto il robot (+/- lateral_half_width_m), con le stesse differenze; i
        campioni di tutte le fette vanno insieme.

    Ognuna delle due si riassume col 90 percentile: una cella sbagliata isolata non conta,
    ma un tratto ripido non viene annacquato dalla media dell'arco.

    has_data e' True solo se almeno min_sample_fraction dei campioni ha un dato, in
    entrambe le direzioni: un arco visto per meno di meta' e' "senza dati", che non vuol
    dire piatto (il PRM passa allora alla mappa globale).

    valid_2d: maschera delle celle attendibili; le altre (riempite, filtrate, mai scritte)
    non entrano. obstacle_2d: celle gia' ostacolo per obstacle_distance, escluse anche loro:
    la faccia di un mobile non e' un pendio, ed e' gia' gestita come ostacolo dal PRM e dal
    fronte. Un arco che perde cosi' i dati diventa "senza dati", non "libero": entra nel
    grafo con un costo di ripiego e chi lo percorre lo verifica col fronte.

    body_ext_m > 0: in piu' i tre valori "col corpo" (vedi _body_slope).

    Returns:
        (long, lat, has_data), oppure con body_ext_m > 0
        (long, lat, has_data, long col corpo, lat col corpo, has col corpo)
    """
    if terrain_2d is None or grid_origin_x is None or cell_size is None:
        return _no_slope_data(body_ext_m)

    # x*x e non **2: e' l'operazione che numpy fa sugli array, quindi la lunghezza (e da
    # li' passo e pendenza) e' identica bit per bit a quella del calcolo in blocco
    # (arc_slope_profiles_batch); **2 su un numero singolo a volte differisce all'ultima cifra.
    dist = float(np.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))
    if dist < 1e-6:
        return _no_slope_data(body_ext_m)

    # Uscita rapida: se l'arco, allargato della fetta laterale, non tocca la griglia, nessun
    # campione vi cade dentro -> "senza dati", identico al calcolo completo ma senza pagarlo
    # (la maggior parte degli archi sta fuori dalla finestra di 3.84 m).
    num_rows, num_cols = terrain_2d.shape
    pad = lateral_half_width_m + cell_size
    if (max(x1, x2) + pad < grid_origin_x or min(x1, x2) - pad > grid_origin_x + num_cols * cell_size or
            max(y1, y2) + pad < grid_origin_y or min(y1, y2) - pad > grid_origin_y + num_rows * cell_size):
        return _no_slope_data(body_ext_m)

    dir_x, dir_y = (x2 - x1) / dist, (y2 - y1) / dist   # along-arc unit vector
    perp_x, perp_y = -dir_y, dir_x                      # across-arc unit vector

    # Celle attendibili E non ostacolo: una sola maschera per entrambe le direzioni.
    usable_2d = valid_2d
    if obstacle_2d is not None:
        obs = np.asarray(obstacle_2d, dtype=bool)
        usable_2d = (~obs) if valid_2d is None else (np.asarray(valid_2d, dtype=bool) & ~obs)

    # --- LONGITUDINALE: lungo la linea dell'arco ---
    num_along = max(2, int(dist / cell_size) + 1)
    t = np.linspace(0.0, 1.0, num_along)
    step_along = dist / (num_along - 1)
    h_along, inside_along = _gather_heights(
        terrain_2d, grid_origin_x, grid_origin_y, cell_size,
        x1 + t * (x2 - x1), y1 + t * (y2 - y1), valid_2d=usable_2d
    )
    grad_long = _baseline_gradient(h_along, step_along, baseline_m)
    longitudinal_slope = p90(grad_long) if grad_long.size else 0.0

    # --- LATERALE: fette perpendicolari larghe quanto il ROBOT, non quanto l'arco ---
    num_slices = max(1, int(dist / perp_sample_spacing_m) + 1)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    step_perp = 2 * lateral_half_width_m / (num_perp - 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    ts = np.linspace(0.0, 1.0, num_slices) if num_slices > 1 else np.array([0.5])
    xs_perp = (x1 + ts * (x2 - x1))[:, None] + u[None, :] * perp_x
    ys_perp = (y1 + ts * (y2 - y1))[:, None] + u[None, :] * perp_y
    h_perp, inside_perp = _gather_heights(
        terrain_2d, grid_origin_x, grid_origin_y, cell_size, xs_perp, ys_perp, valid_2d=usable_2d
    )
    grad_lat = _baseline_gradient(h_perp, step_perp, baseline_m)
    lateral_slope = p90(grad_lat) if grad_lat.size else 0.0

    has_data = (float(inside_along.mean()) >= min_sample_fraction
                and float(inside_perp.mean()) >= min_sample_fraction
                and grad_long.size > 0 and grad_lat.size > 0)

    base = (longitudinal_slope, lateral_slope, has_data)
    if body_ext_m <= 0:
        return base
    gather = lambda xs, ys: _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                            xs, ys, valid_2d=usable_2d)
    return base + _body_slope(gather, base, x1, y1, x2, y2, dir_x, dir_y, perp_x, perp_y,
                              h_along, inside_along, step_along, grad_lat, inside_perp, u,
                              step_perp, num_slices, dist, perp_sample_spacing_m, baseline_m,
                              min_sample_fraction, body_ext_m)


# =========================================================================
# PENDENZA DI MOLTI ARCHI INSIEME
#
# Stesso calcolo di compute_arc_slope_profile / _slope_profile_from_sampler / _body_slope,
# fatto con matrici: una riga per arco. Gli archi si raggruppano per numero di campioni (che
# dipende solo dalla lunghezza: sul reticolo le lunghezze sono poche decine), e per ogni
# gruppo posizioni, letture delle quote, differenze e 90 percentile si fanno in un colpo
# solo. Le operazioni sono le stesse e nello stesso ordine, quindi i numeri sono IDENTICI al
# bit a quelli arco per arco: va mantenuto cosi' (lo controlla test_replay.py).
# =========================================================================
def p90_rows(m):
    """p90 riga per riga di una matrice, ignorando i NaN; IDENTICO a p90 sui valori finiti di
    ogni riga. Restituisce (valori, quanti finiti); 0.0 dove la riga non ha valori finiti."""
    m = np.asarray(m, dtype=np.float64)
    if m.ndim == 1:
        m = m[None, :]
    cnt = np.isfinite(m).sum(axis=1)
    out = np.zeros(m.shape[0], dtype=np.float64)
    ok = cnt > 0
    if not ok.any() or m.shape[1] == 0:
        return out, cnt
    srt = np.sort(m[ok], axis=1)                      # i NaN vanno in fondo
    n = cnt[ok]
    v = (n - 1) * 0.9
    lo = np.floor(v).astype(np.int64)
    hi = np.minimum(lo + 1, n - 1)
    r = np.arange(srt.shape[0])
    x0, x1 = srt[r, lo], srt[r, hi]
    t = v - lo
    d = x1 - x0
    out[ok] = np.where(t >= 0.5, x1 - d * (1.0 - t), x0 + d * t)
    return out, cnt


def _baseline_gradient_rows(h, step, baseline_m):
    """_baseline_gradient su un gruppo di righe con lo STESSO k (step: un valore per riga).
    Restituisce la matrice delle pendenze con NaN dove manca un campione (nessuna
    compattazione: ci pensa p90_rows), o None se la riga e' troppo corta."""
    n = h.shape[-1]
    k = max(1, int(round(baseline_m / float(step[0]))))
    if n <= 2 * k:
        k = max(1, (n - 1) // 2)
    if n <= 2 * k:
        return None
    den = (2 * k * step).reshape((-1,) + (1,) * (h.ndim - 1))
    return np.abs(h[..., 2 * k:] - h[..., :-2 * k]) / den


def _k_of(baseline_m, step):
    """Il k di _baseline_gradient per ogni riga (serve a raggruppare)."""
    return np.array([max(1, int(round(baseline_m / float(s)))) for s in step], dtype=np.int64)


def arc_slope_profiles_batch(sample, x1, y1, x2, y2, cell_size, perp_sample_spacing_m=0.10,
                             baseline_m=SLOPE_BASELINE_M,
                             lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M,
                             min_sample_fraction=MIN_ARC_SAMPLE_FRACTION, body_ext_m=0.0,
                             sampler_mode=False):
    """
    Profilo di pendenza di molti archi insieme. sample(xs, ys) -> (quote, dentro) su array di
    qualunque forma (NaN / False dove non c'e' dato).

    sampler_mode=False: come compute_arc_slope_profile (griglia dal vivo).
    sampler_mode=True:  come _slope_profile_from_sampler (mappa globale): stesse uscite
        anticipate -- i tre punti inizio/meta'/fine senza dati, copertura della mezzeria
        sotto min_sample_fraction -- che danno "nessun dato" con pendenze 0.

    Restituisce un array (N, 6): long, lat, has, long col corpo, lat col corpo, has col corpo
    (has come 0.0 / 1.0). Senza body_ext_m le ultime tre colonne ripetono le prime tre.
    """
    x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
    x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
    N = x1.size
    out = np.zeros((N, 6), dtype=np.float64)
    if N == 0:
        return out
    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    live = dist >= 1e-6
    if sampler_mode:
        h3, _ = sample(np.stack([x1, 0.5 * (x1 + x2), x2], -1), np.stack([y1, 0.5 * (y1 + y2), y2], -1))
        live &= np.isfinite(h3).any(axis=1)
    idx_all = np.nonzero(live)[0]
    if idx_all.size == 0:
        return out
    d = dist[idx_all]
    num_along = np.maximum(2, np.floor(d / cell_size).astype(np.int64) + 1)
    num_slices = np.maximum(1, np.floor(d / perp_sample_spacing_m).astype(np.int64) + 1)
    step_along = d / (num_along - 1)
    k_along = _k_of(baseline_m, step_along)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    step_perp = 2 * lateral_half_width_m / (num_perp - 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    if body_ext_m > 0:
        kb = np.array([max(1, int(round(body_ext_m / float(s)))) for s in step_along], dtype=np.int64)
        ds = np.where(num_slices > 1, d / np.maximum(num_slices - 1, 1), perp_sample_spacing_m)
        kl = np.array([max(1, int(round(body_ext_m / float(s)))) for s in ds], dtype=np.int64)
    else:
        kb = kl = np.zeros(idx_all.size, dtype=np.int64)
        ds = np.zeros(idx_all.size)
    keys = np.stack([num_along, num_slices, k_along, kb, kl], axis=1)
    _, group = np.unique(keys, axis=0, return_inverse=True)
    group = np.asarray(group).ravel()
    for g in range(int(group.max()) + 1):
        sel = np.nonzero(group == g)[0]
        rows = idx_all[sel]
        n_al, n_sl = int(num_along[sel[0]]), int(num_slices[sel[0]])
        ax1, ay1, ax2, ay2 = x1[rows], y1[rows], x2[rows], y2[rows]
        dd = d[sel]
        st = step_along[sel]
        dir_x, dir_y = (ax2 - ax1) / dd, (ay2 - ay1) / dd
        perp_x, perp_y = -dir_y, dir_x
        # --- longitudinale
        t = np.linspace(0.0, 1.0, n_al)
        h_al, in_al = sample(ax1[:, None] + t[None, :] * (ax2 - ax1)[:, None],
                             ay1[:, None] + t[None, :] * (ay2 - ay1)[:, None])
        cov_al = in_al.mean(axis=1)
        go = np.ones(rows.size, dtype=bool)
        if sampler_mode:
            go = cov_al >= min_sample_fraction
        g_long = _baseline_gradient_rows(h_al, st, baseline_m)
        if g_long is None:
            lo, n_lo = np.zeros(rows.size), np.zeros(rows.size, dtype=np.int64)
        else:
            lo, n_lo = p90_rows(g_long)
        # --- laterale
        ts = np.linspace(0.0, 1.0, n_sl) if n_sl > 1 else np.array([0.5])
        cx = ax1[:, None] + ts[None, :] * (ax2 - ax1)[:, None]
        cy = ay1[:, None] + ts[None, :] * (ay2 - ay1)[:, None]
        h_pp, in_pp = sample(cx[:, :, None] + u[None, None, :] * perp_x[:, None, None],
                             cy[:, :, None] + u[None, None, :] * perp_y[:, None, None])
        g_lat = _baseline_gradient_rows(h_pp, np.full(rows.size, step_perp), baseline_m)
        if g_lat is None:
            g_lat_flat = np.zeros((rows.size, 0))
        else:
            g_lat_flat = g_lat.reshape(rows.size, -1)
        la, n_la = p90_rows(g_lat_flat)
        cov_pp = in_pp.reshape(rows.size, -1).mean(axis=1)
        has = (cov_al >= min_sample_fraction) & (cov_pp >= min_sample_fraction) & (n_lo > 0) & (n_la > 0)
        base = np.stack([lo, la, has.astype(np.float64)], axis=1)
        base[~go] = 0.0
        has &= go
        res = np.concatenate([base, base], axis=1)
        if body_ext_m > 0 and has.any():
            k_b, k_l = int(kb[sel[0]]), int(kl[sel[0]])
            off = np.arange(1, k_b + 1)[None, :] * st[:, None]
            xs = np.concatenate((ax1[:, None] - dir_x[:, None] * off[:, ::-1], ax2[:, None] + dir_x[:, None] * off), axis=1)
            ys = np.concatenate((ay1[:, None] - dir_y[:, None] * off[:, ::-1], ay2[:, None] + dir_y[:, None] * off), axis=1)
            h_ex, in_ex = sample(xs, ys)
            h_full = np.concatenate((h_ex[:, :k_b], h_al, h_ex[:, k_b:]), axis=1)
            cov_b = (in_ex.sum(axis=1).astype(np.float64) + in_al.sum(axis=1).astype(np.float64)) / (2 * k_b + n_al)
            gb_long = _baseline_gradient_rows(h_full, st, baseline_m)
            offl = np.arange(1, k_l + 1)[None, :] * ds[sel][:, None]
            bx = np.concatenate((ax1[:, None] - dir_x[:, None] * offl[:, ::-1], ax2[:, None] + dir_x[:, None] * offl), axis=1)
            by = np.concatenate((ay1[:, None] - dir_y[:, None] * offl[:, ::-1], ay2[:, None] + dir_y[:, None] * offl), axis=1)
            h_l, in_l = sample(bx[:, :, None] + u[None, None, :] * perp_x[:, None, None],
                               by[:, :, None] + u[None, None, :] * perp_y[:, None, None])
            gl_ext = _baseline_gradient_rows(h_l, np.full(rows.size, step_perp), baseline_m)
            parts = [g_lat_flat] + ([gl_ext.reshape(rows.size, -1)] if gl_ext is not None else [])
            gb_lat = np.concatenate(parts, axis=1)
            cov_lb = ((in_l.reshape(rows.size, -1).sum(axis=1).astype(np.float64)
                       + in_pp.reshape(rows.size, -1).sum(axis=1).astype(np.float64))
                      / (in_l[0].size + in_pp[0].size))
            if gb_long is None:
                blo, n_blo = np.zeros(rows.size), np.zeros(rows.size, dtype=np.int64)
            else:
                blo, n_blo = p90_rows(gb_long)
            bla, n_bla = p90_rows(gb_lat)
            ok = (has & (cov_b >= min_sample_fraction) & (cov_lb >= min_sample_fraction)
                  & (n_blo > 0) & (n_bla > 0))
            res[ok, 3] = np.maximum(lo[ok], blo[ok])
            res[ok, 4] = np.maximum(la[ok], bla[ok])
            res[ok, 5] = 1.0
        out[rows] = res
    return out


def compute_arc_slope_profiles_batch(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                     x1, y1, x2, y2, valid_2d=None, body_ext_m=0.0, **kw):
    """compute_arc_slope_profile per molti archi (array), dalla griglia dal vivo. Vedi
    arc_slope_profiles_batch per il formato del risultato."""
    n = np.size(x1)
    if terrain_2d is None or grid_origin_x is None or cell_size is None:
        return np.zeros((n, 6))
    x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
    x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
    out = np.zeros((n, 6))
    num_rows, num_cols = terrain_2d.shape
    pad = kw.get('lateral_half_width_m', LATERAL_SLICE_HALF_WIDTH_M) + cell_size
    near = ~((np.maximum(x1, x2) + pad < grid_origin_x) | (np.minimum(x1, x2) - pad > grid_origin_x + num_cols * cell_size)
             | (np.maximum(y1, y2) + pad < grid_origin_y) | (np.minimum(y1, y2) - pad > grid_origin_y + num_rows * cell_size))
    if near.any():
        sample = lambda xs, ys: _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                               xs, ys, valid_2d=valid_2d)
        out[near] = arc_slope_profiles_batch(sample, x1[near], y1[near], x2[near], y2[near], cell_size,
                                             body_ext_m=body_ext_m, **kw)
    return out


def _no_slope_data(body_ext_m):
    """Risultato "nessun dato": 3 valori, o 6 se e' stata chiesta anche la misura col corpo."""
    return (0.0, 0.0, False) if body_ext_m <= 0 else (0.0, 0.0, False, 0.0, 0.0, False)


def _body_slope(gather, base, x1, y1, x2, y2, dir_x, dir_y, perp_x, perp_y,
                h_along, inside_along, step_along, grad_lat, inside_perp, u, step_perp,
                num_slices, dist, perp_spacing, baseline_m, min_sample_fraction, ext_m):
    """
    Pendenza "col corpo": come quella dell'arco, ma sul tratto allungato di ext_m a entrambi
    gli estremi (vedi ARC_BODY_EXTENSION_M). Riusa i campioni gia' letti per l'arco e legge
    solo i pezzi aggiunti, allo stesso passo (ext arrotondato a un multiplo del passo:
    +/- 1.5 cm lungo l'arco, +/- 5 cm per le fette laterali).

    Restituisce (long, lat, has) gia' presi come il PEGGIORE fra arco e arco allungato, cosi'
    il tratto aggiunto non puo' diluire una salita che sta dentro l'arco. Se l'arco non ha
    dati, o il tratto allungato non ne ha abbastanza, restituisce i valori dell'arco.
    """
    lo, la, has = base
    if not has:
        return base
    # Longitudinale: profilo continuo pezzo prima + arco + pezzo dopo.
    k = max(1, int(round(ext_m / step_along)))
    off = np.arange(1, k + 1) * step_along
    xs = np.concatenate((x1 - dir_x * off[::-1], x2 + dir_x * off))
    ys = np.concatenate((y1 - dir_y * off[::-1], y2 + dir_y * off))
    h_ext, in_ext = gather(xs, ys)
    h_full = np.concatenate((h_ext[:k], h_along, h_ext[k:]))
    cov_along = (float(np.count_nonzero(in_ext)) + float(np.count_nonzero(inside_along))) / (2 * k + h_along.size)
    g_long = _baseline_gradient(h_full, step_along, baseline_m)
    # Laterale: fette aggiunte allo stesso passo delle fette dell'arco.
    ds = dist / (num_slices - 1) if num_slices > 1 else perp_spacing
    kl = max(1, int(round(ext_m / ds)))
    offl = np.arange(1, kl + 1) * ds
    cx = np.concatenate((x1 - dir_x * offl[::-1], x2 + dir_x * offl))
    cy = np.concatenate((y1 - dir_y * offl[::-1], y2 + dir_y * offl))
    h_l, in_l = gather(cx[:, None] + u[None, :] * perp_x, cy[:, None] + u[None, :] * perp_y)
    g_lat = np.concatenate((grad_lat, _baseline_gradient(h_l, step_perp, baseline_m)))
    cov_lat = ((float(np.count_nonzero(in_l)) + float(np.count_nonzero(inside_perp)))
               / (in_l.size + np.size(inside_perp)))
    if (cov_along < min_sample_fraction or cov_lat < min_sample_fraction
            or g_long.size == 0 or g_lat.size == 0):
        return base
    return max(lo, p90(g_long)), max(la, p90(g_lat)), True


def _bilinear_height(terrain_2d, grid_origin_x, grid_origin_y, cell_size, px, py):
    """
    Quota del terreno nel punto (px, py) per interpolazione bilineare tra i centri delle
    4 celle vicine (il centro della cella (r, c) sta a origine + (c + 0.5) * cell_size,
    coerente con offset_grid_pixels). None se uno dei 4 vicini cade fuori griglia o non e'
    finito. SOLO DIAGNOSTICA: usata da compute_arc_slope_profile_interp().
    """
    num_rows, num_cols = terrain_2d.shape
    fx = (px - grid_origin_x) / cell_size - 0.5
    fy = (py - grid_origin_y) / cell_size - 0.5
    c0 = int(np.floor(fx))
    r0 = int(np.floor(fy))
    if r0 < 0 or c0 < 0 or r0 + 1 >= num_rows or c0 + 1 >= num_cols:
        return None
    wx = fx - c0
    wy = fy - r0
    h = (terrain_2d[r0, c0] * (1 - wx) * (1 - wy) + terrain_2d[r0, c0 + 1] * wx * (1 - wy) +
         terrain_2d[r0 + 1, c0] * (1 - wx) * wy + terrain_2d[r0 + 1, c0 + 1] * wx * wy)
    return float(h) if np.isfinite(h) else None


def compute_arc_slope_profile_interp(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                     x1, y1, x2, y2, perp_sample_spacing_m=0.10):
    """
    STIMA DIAGNOSTICA, NON USATA PER DECIDERE NULLA. Variante interpolata bilinearmente,
    tenuta per il confronto loggato in `[SLOPE-CHECK]` e nelle colonne long_interp /
    lat_interp del CSV per segmento.

    NOTA (2026-10-05): questa funzione nacque per correggere la lettura "a scalini" di
    compute_arc_slope_profile (CONTEXT.md, punto A). Quel problema e' ora risolto nella
    funzione principale per altra via -- base fisica SLOPE_BASELINE_M invece del passo
    nativo, piu' campionamento NaN-aware -- quindi questa resta solo come riferimento
    indipendente. Attenzione nel confronto: usa ancora la fetta laterale lunga quanto
    l'arco e la differenziazione sul passo di campionamento, quindi sui mobili restituira'
    valori molto piu' alti della stima in uso. E' atteso, non e' un disaccordo da indagare.

    Returns:
        (longitudinal_slope, lateral_slope, has_data)
    """
    if terrain_2d is None or grid_origin_x is None or cell_size is None:
        return 0.0, 0.0, False
    dist = float(np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    if dist < 1e-6:
        return 0.0, 0.0, False

    dir_x, dir_y = (x2 - x1) / dist, (y2 - y1) / dist
    perp_x, perp_y = -dir_y, dir_x

    num_along = max(2, int(dist / cell_size) + 1)
    step_along = dist / (num_along - 1)
    heights_along = []
    for i in range(num_along):
        t = i / (num_along - 1)
        h = _bilinear_height(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                             x1 + t * (x2 - x1), y1 + t * (y2 - y1))
        if h is not None:
            heights_along.append(h)
    if len(heights_along) >= 2:
        # Se qualche campione e' caduto fuori griglia la serie e' accorciata ma resta contigua
        # per costruzione (si perdono solo gli estremi), quindi il passo costante vale ancora.
        longitudinal_slope = float(np.percentile(
            np.abs(np.gradient(np.array(heights_along), step_along)), 90))
    else:
        longitudinal_slope = 0.0

    num_slices = max(1, int(dist / perp_sample_spacing_m) + 1)
    num_perp = max(2, int(dist / cell_size) + 1)
    step_perp = dist / (num_perp - 1)
    pooled = []
    for s in range(num_slices):
        t = s / max(1, num_slices - 1) if num_slices > 1 else 0.5
        cx = x1 + t * (x2 - x1)
        cy = y1 + t * (y2 - y1)
        heights_perp = []
        for j in range(num_perp):
            u = (j / (num_perp - 1) - 0.5) * dist
            h = _bilinear_height(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                 cx + u * perp_x, cy + u * perp_y)
            if h is not None:
                heights_perp.append(h)
        if len(heights_perp) >= 2:
            pooled.extend(np.abs(np.gradient(np.array(heights_perp), step_perp)).tolist())
    lateral_slope = float(np.percentile(pooled, 90)) if pooled else 0.0

    has_data = len(heights_along) >= 2 and len(pooled) > 0
    return longitudinal_slope, lateral_slope, has_data


def compute_arc_sample_coverage(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                x1, y1, x2, y2, perp_sample_spacing_m=0.10,
                                lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M):
    """
    SOLO DIAGNOSTICA (colonne coverage_* del CSV dei segmenti). Frazione dei campioni di un
    arco che cadono dentro la griglia: (lungo l'arco, nelle fette laterali), in [0, 1], con
    la stessa geometria di compute_arc_slope_profile.
    """
    if terrain_2d is None or grid_origin_x is None or cell_size is None:
        return 0.0, 0.0
    dist = float(np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2))
    if dist < 1e-6:
        return 0.0, 0.0
    dir_x, dir_y = (x2 - x1) / dist, (y2 - y1) / dist
    perp_x, perp_y = -dir_y, dir_x

    num_along = max(2, int(dist / cell_size) + 1)
    t = np.linspace(0.0, 1.0, num_along)
    _, inside_along = _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                      x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    num_slices = max(1, int(dist / perp_sample_spacing_m) + 1)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    ts = np.linspace(0.0, 1.0, num_slices) if num_slices > 1 else np.array([0.5])
    xs_perp = (x1 + ts * (x2 - x1))[:, None] + u[None, :] * perp_x
    ys_perp = (y1 + ts * (y2 - y1))[:, None] + u[None, :] * perp_y
    _, inside_perp = _gather_heights(terrain_2d, grid_origin_x, grid_origin_y, cell_size,
                                     xs_perp, ys_perp)

    return float(inside_along.mean()), float(inside_perp.mean())


class Scan:
    """
    Una scansione della griglia locale (vedi LocalGrid.fetch_scan e process_scan). Array piatti,
    nell'ordine delle celle della griglia; (num_y, num_x) per rimetterli in 2D.
    """

    def __init__(self, **fields):
        self.__dict__.update(fields)


class LocalGrid:
    def __init__(self, robot):
        self.local_grid_client = robot.ensure_client(LocalGridClient.default_service_name)
        self.footprint_correction_pending = True
        # Esito dell'ultimo filtro sull'altezza del suolo (vedi _cells_above_ground).
        self.last_ground_filter = {'ground_z': None, 'n_above': 0, 'max_above': 0.0,
                                   'source': 'non ancora eseguito', 'n_ref': 0}

    def create_vtk_no_step_grid(self, proto, robot_state_client):
        """
        Layer "no_step" della risposta: (punti (x, y, z) nel frame VISION, valori, colori RGB:
        rosso = non calpestabile, blu = calpestabile). Array vuoti se il layer manca.
        """
        local_grid_proto = None
        cell_size = 0.0
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == 'no_step':
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        # If no relevant local grid found, return empty arrays (caller can handle)
        if local_grid_proto is None:
            return np.empty((0, 3), dtype=np.float32), np.array([], dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

        # Unpack the data field for the local grid.
        cells_no_step = self.unpack_grid(local_grid_proto).astype(np.float32)
        # Populate the x,y values with a complete combination of all possible pairs for the dimensions in the grid extent.
        ys, xs = np.mgrid[0:local_grid_proto.local_grid.extent.num_cells_x,
                          0:local_grid_proto.local_grid.extent.num_cells_y]
        # Get the estimated height (z value) of the ground in the vision frame as if the robot was standing.
        transforms_snapshot = local_grid_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        z_ground_in_vision_frame = self.compute_ground_height_in_vision_frame(robot_state_client)
        # Numpy vstack makes it so that each column is (x,y,z) for a single no step grid point. The height values come
        # from the estimated height of the ground plane.
        cell_count = local_grid_proto.local_grid.extent.num_cells_x * local_grid_proto.local_grid.extent.num_cells_y
        cells_est_height = np.ones(cell_count) * z_ground_in_vision_frame
        pts = np.vstack(
            [np.ravel(xs).astype(np.float32),
             np.ravel(ys).astype(np.float32), cells_est_height]).T
        pts[:, [0, 1]] *= (local_grid_proto.local_grid.extent.cell_size,
                           local_grid_proto.local_grid.extent.cell_size)
        # Determine the coloration based on whether or not the region is steppable. The regions that Spot considers it
        # cannot safely step are colored red, and the regions that are considered safe to step are colored blue.
        color = np.zeros([cell_count, 3], dtype=np.uint8)
        color[:, 0] = (cells_no_step <= 0.0)
        color[:, 2] = (cells_no_step > 0.0)
        color *= 255
        # Offset the grid points to be in the vision frame instead of the local grid frame.
        vision_tform_local_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME,
                                                local_grid_proto.local_grid.frame_name_local_grid_data)
        pts = self.offset_grid_pixels(pts, vision_tform_local_grid, cell_size)

        return pts, cells_no_step, color


    def create_vtk_obstacle_grid(self, proto, robot_state_client):
        """
        Layer "obstacle_distance": distanza col segno (m) dall'ostacolo piu' vicino, negativa
        dentro un ostacolo. A differenza di no_step non tratta l'erba come ostacolo, per
        questo e' il layer usato per gli ostacoli.

        Returns:
            (punti (N, 3) nel frame VISION, distanze (N,), colori RGB (N, 3): rosso dentro un
            ostacolo, blu fuori). Array vuoti se il layer manca.
        """
        local_grid_proto = None
        cell_size = 0.0
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == 'obstacle_distance':
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        # If no relevant local grid found, return empty arrays (caller can handle)
        if local_grid_proto is None:
            return np.empty((0, 3), dtype=np.float32), np.array([], dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

        # Unpack the raw distance values.
        cells_obstacle_dist = self.unpack_grid(local_grid_proto).astype(np.float32)

        # Build (x, y) grid coordinates.
        ys, xs = np.mgrid[0:local_grid_proto.local_grid.extent.num_cells_x,
                          0:local_grid_proto.local_grid.extent.num_cells_y]

        # Use ground-plane height for the z coordinate.
        transforms_snapshot = local_grid_proto.local_grid.transforms_snapshot
        z_ground_in_vision_frame = self.compute_ground_height_in_vision_frame(robot_state_client)
        cell_count = local_grid_proto.local_grid.extent.num_cells_x * local_grid_proto.local_grid.extent.num_cells_y
        z = np.ones(cell_count, dtype=np.float32) * z_ground_in_vision_frame

        pts = np.vstack([np.ravel(xs).astype(np.float32),
                         np.ravel(ys).astype(np.float32), z]).T
        pts[:, [0, 1]] *= (local_grid_proto.local_grid.extent.cell_size,
                           local_grid_proto.local_grid.extent.cell_size)

        # Colour coding (zero-padding - no border zone):
        #   red  -> strictly inside obstacle  (dist < 0)
        #   blue -> passable: border or free  (dist >= 0)
        color = np.zeros([cell_count, 3], dtype=np.uint8)
        color[:, 0] = (cells_obstacle_dist < 0.0)   # red  = blocked
        color[:, 2] = (cells_obstacle_dist >= 0.0)  # blue = passable
        color *= 255

        # Offset to VISION frame.
        vision_tform_local_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME,
                                                local_grid_proto.local_grid.frame_name_local_grid_data)
        pts = self.offset_grid_pixels(pts, vision_tform_local_grid, cell_size)

        return pts, cells_obstacle_dist, color


    def compute_ground_height_in_vision_frame(self, robot_state_client):
        """Get the z-height of the ground plane in vision frame from the current robot state."""
        robot_state = robot_state_client.get_robot_state()
        vision_tform_ground_plane = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                                  VISION_FRAME_NAME, GROUND_PLANE_FRAME_NAME)
        return vision_tform_ground_plane.position.z

    def offset_grid_pixels(self, pts, vision_tform_local_grid, cell_size):
        """
        Porta i punti dal frame della griglia locale al frame VISION (rotazione e traslazione
        complete), riferendoli al CENTRO di ogni cella.
        """
        pts_centered = pts.copy()
        pts_centered[:, 0] += cell_size * 0.5
        pts_centered[:, 1] += cell_size * 0.5

        # Se l'oggetto della trasformazione supporta transform_cloud, usalo direttamente
        if hasattr(vision_tform_local_grid, 'transform_cloud'):
            return vision_tform_local_grid.transform_cloud(pts_centered)
        else:
            # Fallback manuale tramite matrice di rotazione R e traslazione T
            rot_mat = vision_tform_local_grid.rotation.to_matrix()
            trans = np.array([
                vision_tform_local_grid.position.x,
                vision_tform_local_grid.position.y,
                vision_tform_local_grid.position.z
            ], dtype=np.float32)
            return (pts_centered @ rot_mat.T) + trans


    def unpack_grid(self, local_grid_proto):
        """Unpack the local grid proto."""
        # Determine the data type for the bytes data.
        data_type = self.get_numpy_data_type(local_grid_proto.local_grid)
        if data_type is None:
            print('[GRID] Formato dei dati della griglia locale non riconosciuto.')
            return None
        # Decode the local grid.
        if local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
            full_grid = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        elif local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
            full_grid = self.expand_data_by_rle_count(local_grid_proto, data_type=data_type)
        else:
            # Return nothing if there is no encoding type set.
            return None
        # Apply the offset and scaling to the local grid.
        if local_grid_proto.local_grid.cell_value_scale == 0:
            return full_grid
        full_grid_float = full_grid.astype(np.float64)
        full_grid_float *= local_grid_proto.local_grid.cell_value_scale
        full_grid_float += local_grid_proto.local_grid.cell_value_offset
        return full_grid_float


    def get_numpy_data_type(self, local_grid_proto):
        """Convert the cell format of the local grid proto to a numpy data type."""
        if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
            return np.uint16
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
            return np.int16
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
            return np.uint8
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
            return np.int8
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
            return np.float64
        elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
            return np.float32
        else:
            return None


    def expand_data_by_rle_count(self, local_grid_proto, data_type=np.int16):
        """Expand local grid data to full bytes data using the RLE count."""
        cells_pz = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
        cells_pz_full = []
        # For each value of rle_counts, we expand the cell data at the matching index
        # to have that many repeated, consecutive values.
        for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
            for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
                cells_pz_full.append(cells_pz[i])
        return np.array(cells_pz_full)

    def _fetch_layers(self, types_of_grid):
        """
        Scarica i layer richiesti con UNA sola richiesta (stessa fotografia per tutti); se non
        funziona ripiega su una richiesta per layer.
        """
        if getattr(self, '_multi_request_ok', True):
            try:
                out = list(self.local_grid_client.get_local_grids(list(types_of_grid)))
                self._multi_request_fails = 0
                return out
            except Exception as e:
                # Dopo 3 fallimenti di fila non la si riprova piu' (una richiesta che fallisce
                # sempre costerebbe un giro di rete a ogni scansione); un singolo errore di
                # rete invece non la spegne.
                self._multi_request_fails = getattr(self, '_multi_request_fails', 0) + 1
                if self._multi_request_fails >= 3:
                    print(f"[WARNING] Richiesta unica dei layer non riuscita 3 volte di fila ({e}): da ora "
                          f"li chiedo uno alla volta (con controllo di allineamento).")
                    self._multi_request_ok = False
        p = []
        for t in types_of_grid:
            try:
                p.extend(self.local_grid_client.get_local_grids([t]))
            except Exception as e:
                print(f"[WARNING] Fallito recupero layer '{t}': {e}")
        return p

    @staticmethod
    def _layer_geometry(lg):
        """(origine x, origine y, yaw in gradi, nx, ny, cella) del layer nel frame VISION."""
        g = lg.local_grid
        tf = get_a_tform_b(g.transforms_snapshot, VISION_FRAME_NAME, g.frame_name_local_grid_data)
        q = tf.rotation
        yaw = np.degrees(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        return (tf.position.x, tf.position.y, float(yaw), g.extent.num_cells_x, g.extent.num_cells_y,
                g.extent.cell_size)

    def fetch_scan(self, robot_state_client):
        """
        Scarica i layer di una scansione (obstacle_distance, terrain, terrain_valid) e ne legge
        valori e geometria. Restituisce una Scan, o None se i layer non arrivano. L'elaborazione
        (correzione del terreno, pendenza, maschera degli ostacoli) la fa process_scan.
        """
        grids_data, proto, _ = self.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'], robot_state_client)
        if grids_data is None or 'obstacle_distance' not in grids_data:
            return None
        ext = proto.local_grid.extent
        return Scan(grids_data=grids_data, proto=proto,
                    pts=grids_data['obstacle_distance']['pts'],
                    od=grids_data['obstacle_distance']['values'],
                    terrain_raw=grids_data['terrain']['values'],
                    valid=grids_data['terrain_valid']['values'],
                    num_x=ext.num_cells_x, num_y=ext.num_cells_y, cell_size=ext.cell_size)

    @staticmethod
    def grid_origin(scan):
        """Origine (x, y) della griglia della scansione nel frame VISION."""
        g = scan.proto.local_grid
        tf = get_a_tform_b(g.transforms_snapshot, VISION_FRAME_NAME, g.frame_name_local_grid_data)
        return tf.position.x, tf.position.y

    def process_scan(self, scan, robot_x, robot_y, robot_yaw, footprint=True, fuse=True,
                     ground_label=None, diag_label=None):
        """
        Elabora una scansione letta con fetch_scan, riempiendo la Scan:
          footprint_mask  celle sotto il corpo del robot (footprint=False: nessuna)
          terrain, is_valid, unwritten  terreno corretto (vedi correct_terrain): da qui in
                          poi si usa solo questo
          grad, rough     pendenza e rugosita' sul terreno corretto
          obstacle_mask   veto per cella, pura logica di soglia (fuse=False: non calcolata).
                          La pendenza non entra in questo veto: vedi fuse_obstacle_mask
        ground_label e diag_label: etichette delle righe [GROUND-FILTER] e [FOOTPRINT-DIAG]
        (None = niente riga). Restituisce la Scan.
        """
        s = scan
        s.footprint_mask = (self.compute_robot_footprint_mask(s.pts, robot_x, robot_y, robot_yaw,
                                                              s.num_x, s.num_y) if footprint else None)
        s.terrain, s.is_valid, s.unwritten = self.correct_terrain(
            s.terrain_raw, s.valid, s.num_x, s.num_y, robot_footprint_mask=s.footprint_mask,
            cell_size=s.cell_size, unwritten_value=raw_zero(s.grids_data),
            unwritten_scale=raw_scale(s.grids_data), return_unwritten=True)
        if ground_label:
            log_ground_filter(self, ground_label)
        s.grad, s.rough = self.compute_gradient_and_roughness(
            s.terrain, s.valid, s.num_x, s.num_y, s.cell_size, is_valid=s.is_valid)
        s.obstacle_mask = None
        if fuse:
            s.obstacle_mask = self.fuse_obstacle_mask(
                cells_obstacle_dist=s.od, rough_values=s.rough, is_valid=s.is_valid,
                obstacle_threshold=OBSTACLE_THRESHOLD, rough_threshold=ROUGH_THRESHOLD)
        if diag_label:
            log_footprint_diagnostic(self, s.pts, robot_x, robot_y, robot_yaw, s.num_x, s.num_y,
                                     s.obstacle_mask, diag_label, cells_obstacle_dist=s.od,
                                     rough_values=s.rough, is_valid=s.is_valid)
        return s

    def extra_layers(self, names):
        """
        Layer in piu' da SALVARE soltanto (es. 'intensity', 'no_step'), con una richiesta a
        parte: non entrano nei controlli di allineamento di return_local_grid, quindi se
        mancano o hanno un'altra geometria la scansione usata per navigare non ne risente.
        Non solleva mai. Restituisce {nome: dict(values (ny, nx) float32, origin (x, y) nel
        frame VISION, cell)}; {} se la richiesta fallisce.
        """
        try:
            protos = list(self.local_grid_client.get_local_grids(list(names)))
        except Exception:
            return {}
        out = {}
        for lg in protos:
            try:
                ox, oy, _, nx, ny, cell = self._layer_geometry(lg)
                vals = self.unpack_grid(lg)
                if vals is None or np.size(vals) != nx * ny:
                    continue
                out[lg.local_grid_type_name] = dict(
                    values=np.asarray(vals, dtype=np.float32).reshape((ny, nx)),
                    origin=np.array([ox, oy]), cell=float(cell))
            except Exception:
                continue
        return out

    def return_local_grid(self, types_of_grid, robot_state_client, max_attempts=3):
        """
        Scarica e decodifica i layer richiesti.

        Restituisce (grids_data, main_proto, protos), oppure (None, None, None) se dopo
        max_attempts tentativi manca anche un solo layer o i layer non sono allineati fra
        loro (origine, dimensioni o cella diverse: se il robot si muove fra una richiesta e
        l'altra la griglia puo' ricentrarsi). main_proto e' il primo layer richiesto: la sua
        origine indicizza tutti i layer.

        Avvisa una volta se la griglia e' ruotata rispetto a VISION: tutto il codice assume
        che non lo sia (indici di cella calcolati con la sola traslazione).
        """
        if isinstance(types_of_grid, str):
            types_of_grid = [types_of_grid]

        p, problem = [], None
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                time.sleep(0.05)
            p = self._fetch_layers(types_of_grid)
            names = [lg.local_grid_type_name for lg in p]
            missing = [t for t in types_of_grid if t not in names]
            if missing:
                problem = f"layer mancanti {missing}"
                continue
            try:
                geoms = {lg.local_grid_type_name: self._layer_geometry(lg) for lg in p}
            except Exception as e:
                problem = f"geometria dei layer non leggibile ({e})"
                continue
            ref = geoms[types_of_grid[0]]
            bad = [n for n, gm in geoms.items()
                   if gm[3:5] != ref[3:5] or abs(gm[5] - ref[5]) > 1e-6
                   or np.hypot(gm[0] - ref[0], gm[1] - ref[1]) > 0.5 * ref[5]]
            if bad:
                problem = f"layer non allineati a {types_of_grid[0]}: {bad}"
                continue
            if abs(ref[2]) > 0.5 and not getattr(self, '_warned_grid_yaw', False):
                print(f"[WARNING] La griglia locale e' ruotata di {ref[2]:.2f} gradi rispetto a VISION: "
                      f"gli indici di cella assumono rotazione nulla. Da segnalare.")
                self._warned_grid_yaw = True
            problem = None
            break

        if problem is not None or not p:
            self.rejected_scans = getattr(self, 'rejected_scans', 0) + 1
            print(f"[ERROR] Griglie locali non utilizzabili dopo {max_attempts} tentativi: "
                  f"{problem or 'nessun layer scaricato'} (scansioni scartate finora: {self.rejected_scans}).")
            return None, None, None

        grids_data = {}
        # main_proto = il primo layer richiesto (obstacle_distance in tutti i chiamanti):
        # la sua origine e' quella con cui si indicizzano TUTTI i layer.
        main_proto = next(lg for lg in p if lg.local_grid_type_name == types_of_grid[0])

        for lg in p:
            grid_type = lg.local_grid_type_name
            if grid_type == 'obstacle_distance':
                pts, vals, color = self.create_vtk_obstacle_grid(p, robot_state_client)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
            elif grid_type == 'no_step':
                pts, vals, color = self.create_vtk_no_step_grid(p, robot_state_client)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
            elif grid_type in ['terrain', 'intensity', 'terrain_valid']:
                pts, vals, color = self.create_vtk_terrain_grid(p, robot_state_client, layer_name=grid_type)
                grids_data[grid_type] = {'pts': pts, 'values': vals, 'color': color}
                if grid_type == 'terrain':
                    # Passo di quantizzazione delle quote di QUESTA scansione: tolleranza per
                    # _cells_never_written. 'raw_zero' fa solo da interruttore di quella
                    # regola (None = spenta).
                    grids_data[grid_type]['raw_zero'] = float(np.float32(lg.local_grid.cell_value_offset))
                    grids_data[grid_type]['scale'] = float(lg.local_grid.cell_value_scale)

        return grids_data, main_proto, p

    def create_vtk_terrain_grid(self, proto, robot_state_client, layer_name='terrain'):
        """
        Layer di tipo terreno (terrain, terrain_valid, intensity): (punti nel frame VISION,
        valori, colori). Per `terrain` il valore restituito e' la quota in VISION.
        """
        local_grid_proto = None
        cell_size = 0.0

        for local_grid_found in proto:
            # Check against the dynamic layer_name instead of a hardcoded string
            if local_grid_found.local_grid_type_name == layer_name:
                local_grid_proto = local_grid_found
                cell_size = local_grid_found.local_grid.extent.cell_size

        if local_grid_proto is None:
            return np.empty((0, 3), dtype=np.float32), np.array([], dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

        # Decode values
        terrain_values = self.unpack_grid(local_grid_proto).astype(np.float32)

        # Build XY grid
        ys, xs = np.mgrid[
                 0:local_grid_proto.local_grid.extent.num_cells_x,
                 0:local_grid_proto.local_grid.extent.num_cells_y
                 ]

        z_vals = np.ravel(terrain_values)
        pts = np.vstack([
            np.ravel(xs).astype(np.float32),
            np.ravel(ys).astype(np.float32),
            z_vals
        ]).T

        pts[:, [0, 1]] *= (cell_size, cell_size)

        transforms_snapshot = local_grid_proto.local_grid.transforms_snapshot
        vision_tform_local_grid = get_a_tform_b(
            transforms_snapshot,
            VISION_FRAME_NAME,
            local_grid_proto.local_grid.frame_name_local_grid_data
        )

        pts = self.offset_grid_pixels(pts, vision_tform_local_grid, cell_size)

        # Le quote decodificate sono nel frame della griglia locale; dopo offset_grid_pixels
        # pts[:, 2] e' la quota nel frame VISION, che e' quella che si restituisce.
        terrain_values_vision = pts[:, 2].astype(np.float32)

        color = np.zeros([len(z_vals), 3], dtype=np.uint8)
        color[:, 1] = 255  # green

        return pts, terrain_values_vision, color

    def correct_terrain(self, terrain_values, terrain_valid_values, num_cells_x, num_cells_y,
                        robot_footprint_mask=None, cell_size=0.03, robot_rc=None,
                        unwritten_value=None, unwritten_scale=None, return_unwritten=False):
        """
        Unico punto di correzione del terreno. Le celle non attendibili -- non valide per
        Spot, sotto il corpo del robot, troppo sopra il suolo (_cells_above_ground), mai
        scritte dai sensori (_cells_never_written) -- vengono riempite con la quota della
        cella attendibile piu' vicina e marcate non valide. Tutto il resto del codice usa
        questo terreno corretto, e is_valid impedisce che un valore riempito decida qualcosa.

        Args:
            terrain_values: quote grezze (piatte o (num_cells_y, num_cells_x))
            terrain_valid_values: layer terrain_valid di Spot (attendibile se >= TERRAIN_VALID_MIN)
            robot_footprint_mask: maschera 2D facoltativa, True sotto il corpo del robot
            unwritten_value: interruttore della regola delle celle mai scritte
                (grids_data['terrain']['raw_zero']); None = regola spenta
            unwritten_scale: passo di quantizzazione (grids_data['terrain']['scale'])
            return_unwritten: restituisce anche la maschera delle celle mai scritte

        Returns:
            terrain_corrected: quote corrette, 1D
            is_valid: 1D, True dove la lettura ORIGINALE era attendibile
            unwritten: (solo con return_unwritten) 1D, celle mai scritte, da trattare come
                NON VISTE (vedi arcVerification.make_grid_snapshot)
        """
        terrain_2d = terrain_values.reshape((num_cells_y, num_cells_x)).astype(np.float32)

        if terrain_valid_values is not None and terrain_valid_values.size > 0:
            valid_2d = (terrain_valid_values.reshape((num_cells_y, num_cells_x)) >= TERRAIN_VALID_MIN)
        else:
            valid_2d = np.ones((num_cells_y, num_cells_x), dtype=bool)

        sensor_invalid_2d = ~valid_2d
        fill_mask = sensor_invalid_2d.copy()
        if robot_footprint_mask is not None:
            fill_mask = fill_mask | robot_footprint_mask

        # Filtro sull'altezza del suolo -- vedi GROUND_HEIGHT_TOLERANCE_M.
        above_ground = self._cells_above_ground(terrain_2d, ~fill_mask, cell_size, robot_rc)
        # Celle mai scritte: si passa valid_2d (la bandiera di Spot), non ~fill_mask: vedi
        # _cells_never_written.
        unwritten = self._cells_never_written(terrain_2d, valid_2d, unwritten_value, unwritten_scale)
        fill_mask = fill_mask | above_ground | unwritten

        terrain_corrected = fill_invalid_nearest(terrain_2d, fill_mask)
        is_valid = ~fill_mask

        if return_unwritten:
            return terrain_corrected.ravel(), is_valid.ravel(), unwritten.ravel()
        return terrain_corrected.ravel(), is_valid.ravel()

    def _cells_never_written(self, terrain_2d, sensor_valid_2d, unwritten_value, unwritten_scale):
        """
        Celle che i sensori non hanno mai scritto: restano tutte alla STESSA quota esatta.

        La quota non si prevede (dipende dal frame e Spot sposta l'offset a ogni scansione): si
        RICAVA come la quota esatta piu' frequente fra le celle che Spot stesso dichiara non
        attendibili. Se almeno UNWRITTEN_MIN_CELLS la condividono (e almeno
        UNWRITTEN_MIN_SPOT_INVALID di quelle a quella quota sono non valide per Spot), ogni
        cella a quella quota e' mai scritta, comprese le poche che Spot marca buone per errore.
        E' una rete: quasi tutte queste celle Spot le marca gia' non valide.

        unwritten_value fa solo da interruttore (None = regola spenta).
        Aggiorna self.last_ground_filter['n_unwritten'] e ['n_unwritten_below'].
        """
        none = np.zeros_like(sensor_valid_2d, dtype=bool)
        lgf = self.last_ground_filter
        lgf['n_unwritten'] = 0
        lgf['n_unwritten_below'] = 0
        if unwritten_value is None:
            return none

        tol = (UNWRITTEN_MATCH_FRACTION * abs(float(unwritten_scale)) if unwritten_scale
               else UNWRITTEN_MATCH_FALLBACK_M)
        tol = max(tol, 1e-9)

        finite = np.isfinite(terrain_2d)
        cand = terrain_2d[(~sensor_valid_2d) & finite]
        if cand.size < UNWRITTEN_MIN_CELLS:
            return none
        keys, counts = np.unique(np.round(cand / tol).astype(np.int64), return_counts=True)
        i = int(np.argmax(counts))
        if counts[i] < UNWRITTEN_MIN_CELLS:
            return none
        quota = float(keys[i]) * tol

        unwritten = finite & (np.abs(terrain_2d - quota) <= tol)
        # Un plateau vero e' fatto di celle che Spot stesso dichiara non attendibili. Se
        # quella quota coincidesse con una superficie REALE (il piano di un tavolo
        # esattamente a quell'altezza) le celle sarebbero in gran parte valide: non si tocca.
        if not unwritten.any() or \
                float(np.mean(~sensor_valid_2d[unwritten])) < UNWRITTEN_MIN_SPOT_INVALID:
            return none

        lgf['n_unwritten'] = int(unwritten.sum())
        lgf['quota_unwritten'] = quota
        ground_z = lgf.get('ground_z')
        if ground_z is not None:
            lgf['n_unwritten_below'] = int((unwritten & (terrain_2d < ground_z)).sum())
        return unwritten

    def _cells_above_ground(self, terrain_2d, trusted_2d, cell_size, robot_rc=None):
        """
        Celle che stanno piu' di GROUND_HEIGHT_TOLERANCE_M SOPRA il suolo su cui il robot
        e' appoggiato. Solo sopra: vedi il commento alla costante per il perche'.

        Il suolo e' la mediana delle celle attendibili entro GROUND_REF_RADIUS_M dal robot.
        La griglia locale di Spot e' centrata sul robot (entro ~5 cm), quindi di default il
        robot e' al centro; robot_rc=(riga, colonna) lo sovrascrive.

        Registra l'esito in self.last_ground_filter per i log e il salvataggio dati.
        Restituisce una maschera 2D booleana (tutta False se il suolo non e' stimabile).
        """
        num_rows, num_cols = terrain_2d.shape
        none = np.zeros_like(trusted_2d, dtype=bool)
        if robot_rc is None:
            cr, cc = (num_rows - 1) / 2.0, (num_cols - 1) / 2.0
        else:
            cr, cc = robot_rc

        rr, cc_grid = np.ogrid[:num_rows, :num_cols]
        dist = np.hypot((rr - cr) * cell_size, (cc_grid - cc) * cell_size)
        finite = np.isfinite(terrain_2d)

        source = 'sotto il robot'
        ref = trusted_2d & finite & (dist <= GROUND_REF_RADIUS_M)
        if ref.sum() < GROUND_REF_MIN_CELLS:
            source = 'anello esteso'
            ref = trusted_2d & finite & (dist <= GROUND_REF_FALLBACK_RADIUS_M)
        if ref.sum() < GROUND_REF_MIN_CELLS:
            self.last_ground_filter = {'ground_z': None, 'n_above': 0, 'max_above': 0.0,
                                       'source': 'non stimabile', 'n_ref': int(ref.sum())}
            return none

        ground_z = float(np.median(terrain_2d[ref]))
        height_above = terrain_2d - ground_z
        above = trusted_2d & finite & (height_above > GROUND_HEIGHT_TOLERANCE_M)

        self.last_ground_filter = {
            'ground_z': ground_z,
            'n_above': int(above.sum()),
            'max_above': float(height_above[above].max()) if above.any() else 0.0,
            'source': source,
            'n_ref': int(ref.sum()),
        }
        return above

    def compute_gradient_and_roughness(self, terrain_corrected, terrain_valid_values, num_cells_x, num_cells_y, cell_size,
                                       slope_baseline_m=SLOPE_BASELINE_M, is_valid=None):
        """
        Layer di pendenza e rugosita' per cella, da un terreno GIA' corretto (vedi
        correct_terrain). Le celle che Spot dichiara non valide valgono 0 nell'uscita.

        PENDENZA: differenza centrata su una base fisica (vedi SLOPE_BASELINE_M: ~0.48 m,
        circa un passo) invece che sulla cella da 3 cm, su cui un piccolo bordo varrebbe
        quanto una salita vera. Questo campo per cella serve ai nodi e alle figure; la
        pendenza che decide e' quella per arco (compute_arc_slope_profile).

        RUGOSITA': deviazione standard della quota su una finestra corta (5 celle, 15 cm),
        per il terreno irregolare sotto il piede: allargarla nasconderebbe proprio quello.
        Le statistiche usano solo celle attendibili (is_valid).
        """
        terrain_2d = terrain_corrected.reshape((num_cells_y, num_cells_x)).astype(np.float64)

        if terrain_valid_values is not None and terrain_valid_values.size > 0:
            sensor_invalid_mask = ~(terrain_valid_values.reshape((num_cells_y, num_cells_x)) >= TERRAIN_VALID_MIN)
        else:
            sensor_invalid_mask = np.zeros((num_cells_y, num_cells_x), dtype=bool)

        window_size = 5

        # Celle che possono entrare nelle statistiche di vicinato. Con is_valid (uscita di
        # correct_terrain) restano fuori anche le celle riempite e quelle scartate dal
        # filtro sull'altezza del suolo: altrimenti una cella non attendibile "contamina" la
        # rugosita' delle vicine valide nella finestra 5x5.
        if is_valid is not None:
            weight_2d = np.asarray(is_valid).reshape((num_cells_y, num_cells_x)).astype(np.float64)
        else:
            weight_2d = (~sensor_invalid_mask).astype(np.float64)

        # 1. Smoothing "normalizzato": media delle sole celle valide della finestra.
        #    Dove nella finestra non c'e' nessuna cella valida si tiene il terreno corretto.
        n_valid = uniform_filter(weight_2d, size=window_size)
        sum_t = uniform_filter(terrain_2d * weight_2d, size=window_size)
        sum_t2 = uniform_filter((terrain_2d ** 2) * weight_2d, size=window_size)
        has_support = n_valid > (0.5 / (window_size * window_size))
        with np.errstate(invalid='ignore', divide='ignore'):
            mean_t = np.where(has_support, sum_t / n_valid, terrain_2d)
            mean_t2 = np.where(has_support, sum_t2 / n_valid, terrain_2d ** 2)

        # 2. Calcolo Gradiente (Pendenza) su una base fisica, non sulla spaziatura nativa della cella
        k = max(1, round(slope_baseline_m / cell_size))  # es. 0.25m / 0.03m =~ 8 celle

        def _centered_diff(arr, k, axis):
            # Differenza centrata a distanza k celle, con bordo "clampato" (edge padding)
            # cosi' non introduciamo pendenze finte ai margini della griglia locale.
            pad = [(0, 0), (0, 0)]
            pad[axis] = (k, k)
            padded = np.pad(arr, pad, mode='edge')
            sl_ahead = [slice(None), slice(None)]
            sl_ahead[axis] = slice(2 * k, 2 * k + arr.shape[axis])
            sl_behind = [slice(None), slice(None)]
            sl_behind[axis] = slice(0, arr.shape[axis])
            return padded[tuple(sl_ahead)] - padded[tuple(sl_behind)]

        grad_x = _centered_diff(mean_t, k, axis=1) / (2 * k * cell_size)
        grad_y = _centered_diff(mean_t, k, axis=0) / (2 * k * cell_size)
        gradient_2d = np.sqrt(grad_x ** 2 + grad_y ** 2)

        # 3. Calcolo Rugosita' (Deviazione Standard locale, solo celle valide) -- finestra corta invariata
        variance = mean_t2 - (mean_t ** 2)
        roughness_2d = np.sqrt(np.maximum(0.0, variance))
        # Con meno di 3 celle valide nella finestra la deviazione standard non significa nulla.
        roughness_2d[n_valid * window_size * window_size < 3] = 0.0

        # Azzeriamo SOLO le zone realmente cieche nell'output finale.
        # L'impronta del robot mantiene invece la stima interpolata realistica.
        gradient_2d[sensor_invalid_mask] = 0.0
        roughness_2d[sensor_invalid_mask] = 0.0

        return gradient_2d.ravel(), roughness_2d.ravel()


    def fuse_obstacle_mask(self, cells_obstacle_dist,
                        rough_values,
                        is_valid,
                        obstacle_threshold=OBSTACLE_THRESHOLD,
                        rough_threshold=ROUGH_THRESHOLD):
        """
        Maschera degli ostacoli per cella: -1 ostacolo, +1 libero. Una cella attendibile e'
        ostacolo se obstacle_distance <= obstacle_threshold oppure rugosita' >
        rough_threshold; una cella non attendibile (riempita) non e' mai ostacolo.

        La pendenza non entra qui: si valuta per ARCO (compute_arc_slope_profile), su tutta
        la sua lunghezza e nelle due direzioni. La rugosita' invece e' una proprieta' locale.
        """
        cells_obstacle_dist = cells_obstacle_dist.ravel()
        rough_values = rough_values.ravel()
        is_valid = is_valid.ravel()

        assert cells_obstacle_dist.shape == rough_values.shape == is_valid.shape, \
            "Errore: I layer ambientali hanno dimensioni disallineate!"

        # Calcolo dei VETI applicato SOLO alle zone con dati validi
        # Se una zona NON e' valida (punto cieco o impronta robot), non deve attivare il veto di ostacolo
        v1 = (cells_obstacle_dist <= obstacle_threshold) & is_valid
        v3 = (rough_values > rough_threshold) & is_valid

        # Maschera binaria finale degli ostacoli
        obstacle_mask = v1 | v3
        obstacle_mask = np.where(obstacle_mask, -1.0, 1.0)

        return obstacle_mask

    def compute_robot_footprint_mask(self, pts, robot_x, robot_y, robot_yaw, num_cells_x, num_cells_y, margin=0.05,
                                     diagnostic_only=False,
                                     body_length=1.10, body_width=0.50):
        """
        Maschera 2D delle celle sotto il corpo del robot (rettangolo orientato, piu' margin).
        Viene restituita UNA SOLA VOLTA per missione, alla prima chiamata (alla prima
        scansione il terreno sotto il robot non e' mai stato visto); dopo restituisce None e
        vale il terrain_valid di Spot.

        diagnostic_only=True la calcola senza consumare quell'unica volta (diagnostica
        [FOOTPRINT-DIAG] in easy_walk). body_length / body_width: dimensioni del corpo, da
        cambiare solo per la diagnostica (confronto col solo nucleo fra le zampe).

        Attenzione: la chiama anche il thread del fronte (arcVerification), quindi la maschera
        "una volta sola" puo' consumarla lui per primo.
        """
        if not diagnostic_only:
            if not self.footprint_correction_pending:
                return None
            self.footprint_correction_pending = False

        dx = pts[:, 0] - robot_x
        dy = pts[:, 1] - robot_y
        cos_yaw, sin_yaw = np.cos(robot_yaw), np.sin(robot_yaw)
        x_body = dx * cos_yaw + dy * sin_yaw
        y_body = -dx * sin_yaw + dy * cos_yaw

        half_length = (body_length / 2.0) + margin
        half_width = (body_width / 2.0) + margin
        mask_flat = (np.abs(x_body) <= half_length) & (np.abs(y_body) <= half_width)
        return mask_flat.reshape((num_cells_y, num_cells_x))


class GlobalGrid:
    def __init__(self, resolution=0.03):
        self.resolution = resolution
        # Dizionario globale: la chiave e' (grid_x, grid_y), il valore e' lo stato:
        #  1.0 -> Libero/Sicuro
        # -1.0 -> Occupato/Ostacolo
        self.grid = {}
        self.obs_count = {}
        self._cached_margin = None
        self._cached_offsets = []
        self.global_occupancy_map = None
        self.global_terrain_map = None  # spotGrid.GlobalTerrainGrid, lazy-init altrove
        # Indice a grana grossa: blocco (GLOBAL_OCC_COARSE_CELLS celle di lato) -> numero di
        # celle occupate al suo interno. Serve a is_occupied() per rispondere "libero" senza
        # guardare cella per cella quando nell'intorno non c'e' nessun blocco con ostacoli,
        # che e' il caso di gran lunga piu' comune (vedi PRM_EDGE_SAFETY_MARGIN_M).
        self._coarse_occ = {}
        # Celle dei dischi pericolosi (mark_hazard): restano occupate per tutta la missione.
        self.hazard_cells = set()
        self.hazards = []
        # Celle su cui il CORPO del robot e' passato: per dichiararle occupate servono
        # GLOBAL_OCC_CONFIRM_WALKED conferme invece di GLOBAL_OCC_CONFIRM_OBS.
        self.walked_cells = set()
        # MAPPA DELLO SPAZIO LIBERO (vedi CLEARANCE_NEAR_M): media delle obstacle_distance
        # osservate, tenuta separata fra viste ravvicinate e tutte.
        self.clear_near_sum = {}
        self.clear_near_n = {}
        self.clear_all_sum = {}
        self.clear_all_n = {}
        # Versione delle celle occupate: cresce a ogni cambio da/verso -1. Serve
        # a riusare il raster dilatato di occupied_many finche' la mappa non cambia.
        self._occ_version = 0
        self._margin_raster = None      # (versione, steps, raster, gx0, gy0)

    def _coarse_key(self, gx, gy):
        c = GLOBAL_OCC_COARSE_CELLS
        return (gx // c, gy // c)

    def _set_state(self, key, new_state):
        """Unico punto in cui cambia lo stato di una cella: tiene allineato l'indice grosso."""
        old_state = self.grid.get(key)
        if old_state == new_state:
            return
        self.grid[key] = new_state
        if new_state == -1.0 or old_state == -1.0:
            self._occ_version = getattr(self, '_occ_version', 0) + 1
        ck = self._coarse_key(*key)
        if new_state == -1.0:
            self._coarse_occ[ck] = self._coarse_occ.get(ck, 0) + 1
        elif old_state == -1.0:
            n = self._coarse_occ.get(ck, 0) - 1
            if n > 0:
                self._coarse_occ[ck] = n
            else:
                self._coarse_occ.pop(ck, None)

    def _rebuild_coarse_index(self):
        self._coarse_occ = {}
        self._occ_version = getattr(self, '_occ_version', 0) + 1
        for key, state in self.grid.items():
            if state == -1.0:
                ck = self._coarse_key(*key)
                self._coarse_occ[ck] = self._coarse_occ.get(ck, 0) + 1


    def clearance(self, x, y):
        """
        Spazio libero ricordato in quel punto (m), o None se mai osservato. Media delle
        viste ravvicinate quando ce ne sono, altrimenti media di tutte (vedi CLEARANCE_NEAR_M).
        """
        key = (int(np.floor(x / self.resolution)), int(np.floor(y / self.resolution)))
        n = self.clear_near_n.get(key, 0)
        if n:
            return self.clear_near_sum[key] / n
        n = self.clear_all_n.get(key, 0)
        return (self.clear_all_sum[key] / n) if n else None

    def clearance_many(self, xs, ys):
        """clearance() su molti punti: array di float, NaN dove mai osservato."""
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        out = np.full(xs.shape, np.nan)
        if xs.size == 0 or not self.clear_all_n:
            return out
        gx = np.floor(xs / self.resolution).astype(np.int64)
        gy = np.floor(ys / self.resolution).astype(np.int64)
        ns, na = self.clear_near_sum, self.clear_near_n
        as_, an = self.clear_all_sum, self.clear_all_n
        flat = out.ravel()
        for i, k in enumerate(zip(gx.ravel().tolist(), gy.ravel().tolist())):
            n = na.get(k, 0)
            if n:
                flat[i] = ns[k] / n
            else:
                n = an.get(k, 0)
                if n:
                    flat[i] = as_[k] / n
        return out

    def clearance_blocked(self, x, y, min_clearance):
        """
        True se in quel punto lo spazio ricordato e' sotto min_clearance. Una cella mai
        osservata non blocca: ignoto non e' occupato, stessa regola del fronte sicuro
        ('fine_dati' non e' 'ostacolo').
        """
        c = self.clearance(x, y)
        return c is not None and c < min_clearance

    def update_clearance(self, pts, obstacle_distance, usable, near=None):
        """
        Impara lo spazio libero per cella. Chiamata accanto a update().

        obstacle_distance: le distanze (gia' filtrate) della scansione, allineate a pts.
        usable: celle da cui si puo' imparare (attendibili, oppure sotto il corpo).
        near: osservazioni prese entro CLEARANCE_NEAR_M dal robot -- vanno in un
            accumulatore separato, che ha la precedenza (vedi clearance()).
        """
        p = np.asarray(pts).reshape(-1, pts.shape[-1] if np.ndim(pts) > 1 else 2)
        od = np.asarray(obstacle_distance, dtype=np.float64).ravel()
        us = np.asarray(usable, dtype=bool).ravel()
        if od.size != len(p) or us.size != len(p):
            return
        gx = np.floor(p[:, 0] / self.resolution).astype(np.int64)
        gy = np.floor(p[:, 1] / self.resolution).astype(np.int64)
        nr = (np.zeros(len(p), dtype=bool) if near is None
              else np.asarray(near, dtype=bool).ravel())
        for k, v, ok, isnear in zip(zip(gx.tolist(), gy.tolist()), od, us, nr):
            if not ok:
                continue
            self.clear_all_sum[k] = self.clear_all_sum.get(k, 0.0) + float(v)
            self.clear_all_n[k] = self.clear_all_n.get(k, 0) + 1
            if isnear:
                self.clear_near_sum[k] = self.clear_near_sum.get(k, 0.0) + float(v)
                self.clear_near_n[k] = self.clear_near_n.get(k, 0) + 1

    def update(self, pts, obstacle_mask, walked=None):
        """
        Aggiorna la mappa con una scansione. obstacle_mask: -1 ostacolo, +1 vista libera,
        0 nessuna informazione (cella non attendibile in questa scansione: si salta).

        Isteresi per cella (contatore in obs_count):
          - occupata dopo GLOBAL_OCC_CONFIRM_OBS osservazioni di ostacolo consecutive (la prima
            azzera le prove di "libero");
          - libera quando il contatore torna a 0 o sotto;
          - nel mezzo resta com'era. Una cella mai vista e osservata libera diventa libera
            (serve a is_known).
        Le celle di un disco pericoloso (mark_hazard) non tornano mai libere.

        walked: maschera, allineata a pts, delle celle coperte dal CORPO del robot adesso.
        Vengono ricordate (walked_cells) e per dichiararle occupate servono
        GLOBAL_OCC_CONFIRM_WALKED osservazioni invece di GLOBAL_OCC_CONFIRM_OBS.
        """
        pts_flat = pts.reshape(-1, 3)
        mask_flat = obstacle_mask.ravel()

        # Chiave di cella = floor(x / resolution), come in is_occupied (np.round sui centri
        # delle celle arrotonderebbe al pari e lascerebbe chiavi mai scritte).
        gx_array = np.floor(pts_flat[:, 0] / self.resolution).astype(int)
        gy_array = np.floor(pts_flat[:, 1] / self.resolution).astype(int)

        hazard_cells = self.hazard_cells
        walked_cells = self.walked_cells
        if walked is not None:
            w = np.asarray(walked, dtype=bool).ravel()
            if w.size == gx_array.size:
                walked_cells.update(zip(gx_array[w].tolist(), gy_array[w].tolist()))
        for gx, gy, state in zip(gx_array, gy_array, mask_flat):
            if state == 0:
                continue
            key = (int(gx), int(gy))
            if key in hazard_cells:
                continue
            curr_count = self.obs_count.get(key, 0)

            if state == -1.0:
                new_count = min(curr_count + 1 if curr_count >= 0 else 1, GLOBAL_OCC_MAX_COUNT)
                self.obs_count[key] = new_count
                confirm = (GLOBAL_OCC_CONFIRM_WALKED if key in walked_cells
                           else GLOBAL_OCC_CONFIRM_OBS)
                if new_count >= confirm:
                    self._set_state(key, -1.0)
                elif key not in self.grid:
                    self._set_state(key, 1.0)    # vista, ma non ancora confermata occupata
            else:
                new_count = max(curr_count - 1, -GLOBAL_OCC_MAX_COUNT)
                self.obs_count[key] = new_count
                if new_count <= 0:
                    self._set_state(key, 1.0)
                elif key not in self.grid:
                    self._set_state(key, 1.0)

    def is_occupied(self, x, y, safety_margin=0.0):
        """
        True se (x, y) cade in una cella occupata o, con safety_margin > 0, se c'e' una
        cella occupata entro safety_margin metri (disco). Una cella mai vista e' libera.
        """
        gx_center = int(np.floor(x / self.resolution))
        gy_center = int(np.floor(y / self.resolution))

        if safety_margin <= 0.0:
            return self.grid.get((gx_center, gy_center), 1.0) == -1.0

        # Prefiltro a grana grossa: se nessun blocco che tocca l'intorno contiene celle
        # occupate, la risposta e' "libero" senza guardare le singole celle (stesso risultato).
        steps = int(np.ceil(safety_margin / self.resolution))
        c = GLOBAL_OCC_COARSE_CELLS
        any_coarse = False
        for cx in range((gx_center - steps) // c, (gx_center + steps) // c + 1):
            for cy in range((gy_center - steps) // c, (gy_center + steps) // c + 1):
                if (cx, cy) in self._coarse_occ:
                    any_coarse = True
                    break
            if any_coarse:
                break
        if not any_coarse:
            return False

        # Rigenera la maschera di vicinato solo se il margine cambia
        if self._cached_margin != safety_margin:
            self._cached_margin = safety_margin
            steps = int(np.ceil(safety_margin / self.resolution))
            offsets = []
            for dx in range(-steps, steps + 1):
                for dy in range(-steps, steps + 1):
                    if dx ** 2 + dy ** 2 <= steps ** 2:
                        offsets.append((dx, dy))
            self._cached_offsets = offsets

        for dx, dy in self._cached_offsets:
            if self.grid.get((gx_center + dx, gy_center + dy), 1.0) == -1.0:
                return True

        return False

    def _get_margin_raster(self, safety_margin):
        """
        Raster delle celle per cui is_occupied(x, y, safety_margin) e' True: le
        celle occupate dilatate con LO STESSO disco di is_occupied (dx^2 + dy^2 <= steps^2,
        steps = ceil(margin / resolution)). Restituisce (raster, gx0, gy0) oppure None se non
        c'e' nessuna cella occupata. Ricostruito solo quando la mappa cambia (_occ_version).
        """
        steps = int(np.ceil(safety_margin / self.resolution)) if safety_margin > 0.0 else 0
        ver = getattr(self, '_occ_version', 0)
        cached = getattr(self, '_margin_raster', None)
        if cached is not None and cached[0] == ver and cached[1] == steps:
            return cached[2]
        occ = [k for k, v in self.grid.items() if v == -1.0]
        if not occ:
            result = None
        else:
            k = np.array(occ, dtype=np.int64)
            gx0, gy0 = int(k[:, 0].min()) - steps, int(k[:, 1].min()) - steps
            nx = int(k[:, 0].max()) - gx0 + steps + 1
            ny = int(k[:, 1].max()) - gy0 + steps + 1
            raster = np.zeros((nx, ny), dtype=bool)
            raster[k[:, 0] - gx0, k[:, 1] - gy0] = True
            if steps > 0:
                d = np.arange(-steps, steps + 1)
                disk = (d[:, None] ** 2 + d[None, :] ** 2) <= steps ** 2
                raster = binary_dilation(raster, structure=disk)
            result = (raster, gx0, gy0)
        self._margin_raster = (ver, steps, result)
        return result

    def occupied_many(self, xs, ys, safety_margin=0.0):
        """
        is_occupied su molti punti insieme, con risultato IDENTICO punto per punto (stesse
        chiavi floor(x / resolution), stesso disco): serve alla costruzione del grafo.
        """
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        out = np.zeros(xs.shape, dtype=bool)
        r = self._get_margin_raster(safety_margin)
        if r is None or xs.size == 0:
            return out
        raster, gx0, gy0 = r
        gx = np.floor(xs / self.resolution).astype(np.int64) - gx0
        gy = np.floor(ys / self.resolution).astype(np.int64) - gy0
        inside = (gx >= 0) & (gx < raster.shape[0]) & (gy >= 0) & (gy < raster.shape[1])
        out[inside] = raster[gx[inside], gy[inside]]
        return out

    def mark_hazard(self, x, y, radius=0.5):
        """
        Segna come occupato un disco attorno a un punto pericoloso (oggi: dove il robot e'
        CADUTO). Il terreno li' puo' sembrare normale ai sensori (scivoloso, cedevole, una
        buca coperta d'erba), quindi non si aspetta conferma dalle scansioni, e le scansioni
        successive non lo liberano: resta occupato per tutta la missione. Restituisce il
        numero di celle segnate.
        """
        if not hasattr(self, 'hazards'):
            self.hazards = []
        if not hasattr(self, 'hazard_cells'):
            self.hazard_cells = set()
        self.hazards.append((float(x), float(y), float(radius)))
        r = self.resolution
        steps = int(np.ceil(radius / r))
        gx0, gy0 = int(np.floor(x / r)), int(np.floor(y / r))
        n = 0
        for dx in range(-steps, steps + 1):
            for dy in range(-steps, steps + 1):
                if dx * dx + dy * dy <= steps * steps:
                    key = (gx0 + dx, gy0 + dy)
                    self.obs_count[key] = GLOBAL_OCC_MAX_COUNT
                    self._set_state(key, -1.0)
                    self.hazard_cells.add(key)
                    n += 1
        return n

    def segment_hits_hazard(self, x1, y1, x2, y2, margin=0.0):
        """True se il segmento passa entro (raggio + margin) dal centro di un disco pericoloso."""
        for hx, hy, hr in getattr(self, 'hazards', []):
            dx, dy = x2 - x1, y2 - y1
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((hx - x1) * dx + (hy - y1) * dy) / L2))
            if np.hypot(x1 + t * dx - hx, y1 + t * dy - hy) <= hr + margin:
                return True
        return False

    def is_known(self, x, y):
        """
        True se questa posizione e' stata OSSERVATA almeno una volta (libera o occupata).
        is_occupied da sola non distingue "vista libera" da "mai vista" (entrambe False): qui
        si', e serve per esempio a scegliere un obiettivo visto libero piuttosto che ignoto
        (find_best_point_in_cell) e a non creare scorciatoie GraphNav su terreno mai visto.
        """
        gx = int(np.floor(x / self.resolution))
        gy = int(np.floor(y / self.resolution))
        return (gx, gy) in self.grid

    def save_map(self, file_path):
        """Salva la mappa globale di occupazione in un file pickle."""
        with open(file_path, 'wb') as f:
            pickle.dump({
                'resolution': self.resolution,
                'grid': self.grid,
                'hazards': list(getattr(self, 'hazards', [])),   # dischi delle cadute (x, y, raggio)
                'walked_cells': list(getattr(self, 'walked_cells', ())),  # celle su cui il corpo e' passato
                # mappa dello spazio libero (vedi clearance()): somme e conteggi
                'clear_near_sum': getattr(self, 'clear_near_sum', {}),
                'clear_near_n': getattr(self, 'clear_near_n', {}),
                'clear_all_sum': getattr(self, 'clear_all_sum', {}),
                'clear_all_n': getattr(self, 'clear_all_n', {}),
            }, f)
        print(f"[MAP] Mappa globale salvata correttamente in {file_path}")

    def load_map(self, file_path):
        """Ricarica una mappa precedentemente salvata."""
        if os.path.exists(file_path):
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
                self.resolution = data['resolution']
                self.grid = data['grid']
                self.hazards = list(data.get('hazards', []))
                self.walked_cells = {tuple(k) for k in data.get('walked_cells', ())}
                self.clear_near_sum = dict(data.get('clear_near_sum', {}))
                self.clear_near_n = dict(data.get('clear_near_n', {}))
                self.clear_all_sum = dict(data.get('clear_all_sum', {}))
                self.clear_all_n = dict(data.get('clear_all_n', {}))
            self.hazard_cells = set()
            for hx, hy, hr in self.hazards:          # i dischi delle cadute restano permanenti
                st = int(np.ceil(hr / self.resolution))
                gx0, gy0 = int(np.floor(hx / self.resolution)), int(np.floor(hy / self.resolution))
                self.hazard_cells.update((gx0 + dx, gy0 + dy) for dx in range(-st, st + 1)
                                         for dy in range(-st, st + 1) if dx * dx + dy * dy <= st * st)
            self._rebuild_coarse_index()
            print(f"[MAP] Mappa globale caricata con {len(self.grid)} celle occupate/libere.")
        else:
            print(f"[WARNING] File {file_path} non trovato. Inizializzo mappa vuota.")


def _slope_profile_from_sampler(sample_height, x1, y1, x2, y2, cell_size,
                                perp_sample_spacing_m=0.10,
                                baseline_m=SLOPE_BASELINE_M,
                                lateral_half_width_m=LATERAL_SLICE_HALF_WIDTH_M,
                                min_sample_fraction=MIN_ARC_SAMPLE_FRACTION,
                                sample_array=None, body_ext_m=0.0):
    """
    Pendenza di un arco su una mappa SPARSA (la mappa globale del terreno, vedi
    GlobalTerrainGrid.compute_arc_slope_profile). sample_height(px, py) da' la quota in un
    punto o None; sample_array fa lo stesso su un array (NaN dove manca).

    Stessa geometria e stessa base di compute_arc_slope_profile: le due fonti devono
    misurare la stessa grandezza, altrimenti un arco cambierebbe pendenza entrando o uscendo
    dalla finestra scansionata (test_replay.py lo controlla). has_data dice se ci sono
    abbastanza campioni VERI.

    Returns:
        (longitudinal_slope, lateral_slope, has_data)
    """
    # x*x e non **2: e' l'operazione che numpy fa sugli array, quindi la lunghezza (e da
    # li' passo e pendenza) e' identica bit per bit a quella del calcolo in blocco
    # (arc_slope_profiles_batch); **2 su un numero singolo a volte differisce all'ultima cifra.
    dist = float(np.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))
    if dist < 1e-6:
        return _no_slope_data(body_ext_m)

    dir_x, dir_y = (x2 - x1) / dist, (y2 - y1) / dist
    perp_x, perp_y = -dir_y, dir_x

    # Uscita anticipata: se inizio, meta' e fine dell'arco non hanno dati, "senza dati"
    # (la maggior parte degli archi sta fuori da tutto cio' che e' stato visto). Se almeno
    # uno risponde si fa il calcolo completo, e decide min_sample_fraction.
    if sample_array is not None:
        h3 = sample_array(np.array([x1, 0.5 * (x1 + x2), x2]), np.array([y1, 0.5 * (y1 + y2), y2]))
        if not np.isfinite(h3).any():
            return _no_slope_data(body_ext_m)
    elif (sample_height(x1, y1) is None
            and sample_height(0.5 * (x1 + x2), 0.5 * (y1 + y2)) is None
            and sample_height(x2, y2) is None):
        return _no_slope_data(body_ext_m)

    def _sample_array(xs, ys):
        """Campiona un array di posizioni, NaN dove la mappa non ha dati."""
        if sample_array is not None:
            return sample_array(xs, ys)
        flat_x = np.ravel(xs)
        flat_y = np.ravel(ys)
        out = np.empty(flat_x.size, dtype=np.float64)
        for i in range(flat_x.size):
            h = sample_height(flat_x[i], flat_y[i])
            out[i] = np.nan if h is None else float(h)
        return out.reshape(np.shape(xs))

    # --- LONGITUDINALE ---
    num_along = max(2, int(dist / cell_size) + 1)
    t = np.linspace(0.0, 1.0, num_along)
    step_along = dist / (num_along - 1)
    h_along = _sample_array(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    cov_along = float(np.isfinite(h_along).mean())

    # Il laterale costa ~6 volte il longitudinale (una fetta ogni 10 cm, ciascuna
    # larga quanto il robot). Se la copertura longitudinale e' gia' insufficiente,
    # has_data sarebbe False comunque: inutile pagarlo.
    if cov_along < min_sample_fraction:
        return _no_slope_data(body_ext_m)

    grad_long = _baseline_gradient(h_along, step_along, baseline_m)
    longitudinal_slope = p90(grad_long) if grad_long.size else 0.0

    # --- LATERALE ---
    num_slices = max(1, int(dist / perp_sample_spacing_m) + 1)
    num_perp = max(2, int(2 * lateral_half_width_m / cell_size) + 1)
    step_perp = 2 * lateral_half_width_m / (num_perp - 1)
    u = np.linspace(-lateral_half_width_m, lateral_half_width_m, num_perp)
    ts = np.linspace(0.0, 1.0, num_slices) if num_slices > 1 else np.array([0.5])
    xs_perp = (x1 + ts * (x2 - x1))[:, None] + u[None, :] * perp_x
    ys_perp = (y1 + ts * (y2 - y1))[:, None] + u[None, :] * perp_y
    h_perp = _sample_array(xs_perp, ys_perp)
    cov_lat = float(np.isfinite(h_perp).mean())
    grad_lat = _baseline_gradient(h_perp, step_perp, baseline_m)
    lateral_slope = p90(grad_lat) if grad_lat.size else 0.0

    has_data = (cov_along >= min_sample_fraction and cov_lat >= min_sample_fraction
                and grad_long.size > 0 and grad_lat.size > 0)

    base = (longitudinal_slope, lateral_slope, has_data)
    if body_ext_m <= 0:
        return base

    def gather(xs, ys):
        h = _sample_array(xs, ys)
        return h, np.isfinite(h)
    return base + _body_slope(gather, base, x1, y1, x2, y2, dir_x, dir_y, perp_x, perp_y,
                              h_along, np.isfinite(h_along), step_along, grad_lat,
                              np.isfinite(h_perp), u, step_perp, num_slices, dist,
                              perp_sample_spacing_m, baseline_m, min_sample_fraction, body_ext_m)


class GlobalTerrainGrid:
    """
    Mappa globale e persistente della QUOTA del terreno (media delle osservazioni per
    cella), non della pendenza: la pendenza dipende da come si percorre un arco e si
    ricalcola al volo (compute_arc_slope_profile). Stessa risoluzione della griglia locale;
    se in missioni lunghe la memoria diventasse un problema, la leva e' `resolution`.

    Si alimenta solo con quote corrette (LocalGrid.correct_terrain) e con la stessa maschera
    del calcolo dal vivo (easy_walk.slope_usable_mask: celle attendibili e non ostacolo),
    tramite easy_walk.fold_scan_into_global_maps.

    Limite: se l'odometria (frame VISION) derivasse, letture della "stessa" cella verrebbero
    da punti un po' diversi. Sulle missioni del 13-10 lo scarto fra scansioni successive e'
    0.1 cm in mediana.
    """
    def __init__(self, resolution=0.03):
        self.resolution = resolution
        self.height_sum = {}    # (gx, gy) -> somma delle altezze valide osservate
        self.height_count = {}  # (gx, gy) -> numero di osservazioni valide
        # Rettangoli (xmin, xmax, ymin, ymax) aggiornati, uno per chiamata a update: servono
        # al PRM per sapere quali pendenze in cache rifare (prm_graph._sync_global_slope_cache).
        self.update_rects = []

    def update(self, pts, heights, is_valid):
        """
        Args:
            pts: (N,3) o (N,2) -- solo le colonne x,y vengono usate (posizione mondo)
            heights: array 1D di N altezze GIA' corrette (vedi correct_terrain)
            is_valid: array 1D booleano di N elementi, True dove la quota va usata
        """
        pts_flat = np.asarray(pts).reshape(-1, pts.shape[-1] if pts.ndim > 1 else 2)
        x_array = pts_flat[:, 0]
        y_array = pts_flat[:, 1]
        heights_flat = np.asarray(heights).ravel()
        valid_flat = np.asarray(is_valid).ravel()

        # Chiave = floor(x / resolution) come in get_height (np.round sui centri delle celle
        # arrotonderebbe al pari e lascerebbe chiavi mai scritte).
        gx_array = np.floor(x_array / self.resolution).astype(int)
        gy_array = np.floor(y_array / self.resolution).astype(int)

        if valid_flat.any():
            xv, yv = x_array[valid_flat.astype(bool)], y_array[valid_flat.astype(bool)]
            if not hasattr(self, 'update_rects'):
                self.update_rects = []
            self.update_rects.append((float(xv.min()), float(xv.max()), float(yv.min()), float(yv.max())))

        for gx, gy, h, v in zip(gx_array, gy_array, heights_flat, valid_flat):
            if not v:
                continue
            key = (gx, gy)
            self.height_sum[key] = self.height_sum.get(key, 0.0) + float(h)
            self.height_count[key] = self.height_count.get(key, 0) + 1

    def get_height(self, x, y):
        """Altezza media osservata in quella posizione, o None se mai vista."""
        gx = int(np.floor(x / self.resolution))
        gy = int(np.floor(y / self.resolution))
        key = (gx, gy)
        count = self.height_count.get(key, 0)
        if count == 0:
            return None
        return self.height_sum[key] / count

    def compute_arc_slope_profiles_batch(self, x1, y1, x2, y2, perp_sample_spacing_m=0.10, body_ext_m=0.0):
        """compute_arc_slope_profile della mappa globale per molti archi (array), stesse uscite
        anticipate. Vedi arc_slope_profiles_batch per il formato del risultato."""
        x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
        x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
        out = np.zeros((x1.size, 6))
        rects = getattr(self, 'update_rects', None)
        near = np.ones(x1.size, dtype=bool)
        if rects is not None:
            if not rects:
                return out
            if not hasattr(self, '_seen_bbox') or self._seen_bbox[4] != len(rects):
                r = np.array(rects)
                self._seen_bbox = (r[:, 0].min(), r[:, 1].max(), r[:, 2].min(), r[:, 3].max(), len(rects))
            bxmin, bxmax, bymin, bymax, _ = self._seen_bbox
            pad = LATERAL_SLICE_HALF_WIDTH_M + 2 * self.resolution
            near = ~((np.maximum(x1, x2) + pad < bxmin) | (np.minimum(x1, x2) - pad > bxmax)
                     | (np.maximum(y1, y2) + pad < bymin) | (np.minimum(y1, y2) - pad > bymax))
        if near.any():
            def sample(xs, ys):
                h = self._sample_heights(xs, ys)
                return h, np.isfinite(h)
            out[near] = arc_slope_profiles_batch(sample, x1[near], y1[near], x2[near], y2[near],
                                                 self.resolution, perp_sample_spacing_m,
                                                 body_ext_m=body_ext_m, sampler_mode=True)
        return out

    def compute_arc_slope_profile(self, x1, y1, x2, y2, perp_sample_spacing_m=0.10, body_ext_m=0.0):
        """
        Come spotGrid.compute_arc_slope_profile, ma sulla mappa globale: serve per gli archi
        fuori dalla finestra di adesso in zone gia' viste prima (vedi
        _slope_profile_from_sampler).

        Returns:
            (longitudinal_slope, lateral_slope, has_data)
        """
        rects = getattr(self, 'update_rects', None)
        if rects is not None:
            if not rects:
                return _no_slope_data(body_ext_m)
            # Uscita rapida: arco tutto fuori dal rettangolo che contiene tutto cio' che e'
            # stato visto -> nessun campione puo' avere dati (stesso risultato, senza leggere).
            if not hasattr(self, '_seen_bbox') or self._seen_bbox[4] != len(rects):
                r = np.array(rects)
                self._seen_bbox = (r[:, 0].min(), r[:, 1].max(), r[:, 2].min(), r[:, 3].max(), len(rects))
            bxmin, bxmax, bymin, bymax, _ = self._seen_bbox
            pad = LATERAL_SLICE_HALF_WIDTH_M + 2 * self.resolution
            if (max(x1, x2) + pad < bxmin or min(x1, x2) - pad > bxmax or
                    max(y1, y2) + pad < bymin or min(y1, y2) - pad > bymax):
                return _no_slope_data(body_ext_m)
        return _slope_profile_from_sampler(
            self.get_height, x1, y1, x2, y2, self.resolution, perp_sample_spacing_m,
            sample_array=self._sample_heights, body_ext_m=body_ext_m
        )

    def _mean_height_raster(self):
        """
        Raster delle altezze medie (height_sum / height_count, NaN dove mai vista),
        ricostruito solo quando la mappa cambia. Stessa divisione di get_height, quindi stessi
        valori al bit.
        """
        ver = (len(self.height_count), len(getattr(self, 'update_rects', None) or []))
        cached = getattr(self, '_mh_raster', None)
        if cached is not None and cached[0] == ver:
            return cached[1]
        keys = [key for key, c in self.height_count.items() if c > 0]
        if not keys:
            result = None
        else:
            k = np.array(keys, dtype=np.int64)
            s = np.array([self.height_sum[key] for key in keys], dtype=np.float64)
            c = np.array([self.height_count[key] for key in keys], dtype=np.float64)
            gx0, gy0 = int(k[:, 0].min()), int(k[:, 1].min())
            raster = np.full((int(k[:, 0].max()) - gx0 + 1, int(k[:, 1].max()) - gy0 + 1), np.nan)
            raster[k[:, 0] - gx0, k[:, 1] - gy0] = s / c
            result = (raster, gx0, gy0)
        self._mh_raster = (ver, result)
        return result

    def _sample_heights(self, xs, ys):
        """get_height su un array di punti (NaN dove mai vista), stessa chiave floor(x / res)."""
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        out = np.full(xs.shape, np.nan)
        rr = self._mean_height_raster()
        if rr is None or xs.size == 0:
            return out
        raster, gx0, gy0 = rr
        gx = np.floor(xs / self.resolution).astype(np.int64) - gx0
        gy = np.floor(ys / self.resolution).astype(np.int64) - gy0
        ok = (gx >= 0) & (gx < raster.shape[0]) & (gy >= 0) & (gy < raster.shape[1])
        out[ok] = raster[gx[ok], gy[ok]]
        return out

    def _has_height_raster(self):
        """Raster booleano delle celle con almeno un'altezza (height_count > 0), con cache."""
        ver = (len(self.height_count), len(getattr(self, 'update_rects', None) or []))
        cached = getattr(self, '_hh_raster', None)
        if cached is not None and cached[0] == ver:
            return cached[1]
        if not self.height_count:
            result = None
        else:
            k = np.array([key for key, c in self.height_count.items() if c > 0], dtype=np.int64)
            if k.size == 0:
                result = None
            else:
                gx0, gy0 = int(k[:, 0].min()), int(k[:, 1].min())
                raster = np.zeros((int(k[:, 0].max()) - gx0 + 1, int(k[:, 1].max()) - gy0 + 1), dtype=bool)
                raster[k[:, 0] - gx0, k[:, 1] - gy0] = True
                result = (raster, gx0, gy0)
        self._hh_raster = (ver, result)
        return result

    def certainly_no_slope_data(self, x1, y1, x2, y2, min_sample_fraction=MIN_ARC_SAMPLE_FRACTION):
        """
        Per molti archi insieme (array): True dove compute_arc_slope_profile risponderebbe
        CERTAMENTE "nessun dato". Riproduce esattamente le sue uscite anticipate (mappa vuota,
        arco fuori dal rettangolo del visto, inizio/meta'/fine senza dati, copertura della
        linea sotto min_sample_fraction) con gli stessi campioni. Dove risponde False si
        chiama la funzione vera: il risultato e' identico, si evitano solo le chiamate inutili.
        """
        x1 = np.asarray(x1, dtype=np.float64); y1 = np.asarray(y1, dtype=np.float64)
        x2 = np.asarray(x2, dtype=np.float64); y2 = np.asarray(y2, dtype=np.float64)
        n = x1.size
        out = np.ones(n, dtype=bool)
        if n == 0:
            return out
        rects = getattr(self, 'update_rects', None)
        if rects is not None and not rects:
            return out
        rr = self._has_height_raster()
        if rr is None:
            return out
        raster, gx0, gy0 = rr
        res = self.resolution

        def has(px, py):
            gx = np.floor(px / res).astype(np.int64) - gx0
            gy = np.floor(py / res).astype(np.int64) - gy0
            ok = (gx >= 0) & (gx < raster.shape[0]) & (gy >= 0) & (gy < raster.shape[1])
            h = np.zeros(px.shape, dtype=bool)
            h[ok] = raster[gx[ok], gy[ok]]
            return h

        dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        three = has(x1, y1) | has(0.5 * (x1 + x2), 0.5 * (y1 + y2)) | has(x2, y2)
        cand = (dist >= 1e-6) & three
        if not cand.any():
            return out
        idx = np.nonzero(cand)[0]
        num_along = np.maximum(2, (dist[idx] / res).astype(np.int64) + 1)
        for m in np.unique(num_along):
            sel = idx[num_along == m]
            t = np.linspace(0.0, 1.0, int(m))
            px = x1[sel, None] + t[None, :] * (x2[sel] - x1[sel])[:, None]
            py = y1[sel, None] + t[None, :] * (y2[sel] - y1[sel])[:, None]
            cov = has(px, py).mean(axis=1)
            out[sel] = cov < min_sample_fraction
        return out

    def save_map(self, file_path):
        """Salva la mappa globale di altezza in un file pickle."""
        with open(file_path, 'wb') as f:
            pickle.dump({
                'resolution': self.resolution,
                'height_sum': self.height_sum,
                'height_count': self.height_count,
                # quote nel frame VISION (rispetto all'accensione); sottrarre questo valore
                # per averle rispetto al suolo all'avvio missione. None se non noto.
                'mission_z0': getattr(self, 'mission_z0', None),
            }, f)
        print(f"[MAP] Mappa globale del terreno salvata correttamente in {file_path}")

    def load_map(self, file_path):
        """Ricarica una mappa di altezza precedentemente salvata."""
        if os.path.exists(file_path):
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
                self.resolution = data['resolution']
                self.height_sum = data['height_sum']
                self.height_count = data['height_count']
                self.mission_z0 = data.get('mission_z0')
                # zone aggiornate sconosciute per una mappa ricaricata: niente uscite rapide
                self.update_rects = None
                self._hh_raster = None
                self._mh_raster = None
            print(f"[MAP] Mappa globale del terreno caricata con {len(self.height_count)} celle osservate.")
        else:
            print(f"[WARNING] File {file_path} non trovato. Inizializzo mappa terreno vuota.")


# =========================================================================================
# DALLA SCANSIONE ALLE MAPPE GLOBALI, e righe di log sullo stato di griglie e mappe
# =========================================================================================
# --- Zona attorno al corpo nella mappa GLOBALE ------------------------------------------
# Le celle a ridosso del fianco del robot risultano "ostacolo" SOLO quando il robot ci sta
# accanto: lette da 1-2 m sono libere, lette con il robot fermo li' accanto scendono sotto
# soglia, a 0.2-0.45 m dall'asse, all'altezza delle zampe posteriori. Due scansioni da fermo
# basterebbero a confermarle nella mappa globale (GLOBAL_OCC_CONFIRM_OBS = 2), e il PRM
# scarterebbe l'arco su cui il robot e' APPENA passato.
#   - NUCLEO (il rettangolo del corpo): il robot ci sta sopra, quindi e' libero per
#     costruzione -> si scrive "visto libero".
#   - ANELLO (fino a GLOBAL_MAP_SELF_RING_M oltre il corpo): da qui la lettura non e'
#     attendibile -> un ostacolo NON si scrive (nessuna informazione); il libero si'.
# Vale solo per la mappa globale: il fronte sicuro usa la scansione dal vivo, intera, quindi
# un ostacolo vero accanto al robot continua a fermarlo. E un ostacolo vero viene comunque
# scritto nella mappa globale dalle scansioni fatte da un altro punto.
GLOBAL_MAP_SELF_BODY_LENGTH_M = 1.10

GLOBAL_MAP_SELF_BODY_WIDTH_M = 0.50

GLOBAL_MAP_SELF_RING_M = 0.25


def mask_for_global_map(obstacle_mask, is_valid, pts=None, robot_pose=None,
                         return_core=False):
    """
    Maschera da passare a GlobalGrid.update: -1 ostacolo, +1 vista libera, 0 nessuna
    informazione (cella non attendibile in questa scansione, che quindi non conta come
    "vista libera": vedi GlobalGrid.update).

    pts, robot_pose=(x, y, yaw): se passati, applica la zona attorno al corpo (vedi
    GLOBAL_MAP_SELF_RING_M): nucleo libero, anello senza ostacoli.

    return_core: restituisce anche la maschera del NUCLEO, cioe' le celle
    coperte dal corpo adesso. Va passata a GlobalGrid.update come `walked`: sono celle su
    cui il robot e' fisicamente passato, e per dichiararle occupate serviranno piu'
    conferme (vedi spotGrid.GLOBAL_OCC_CONFIRM_WALKED).
    """
    m = np.asarray(obstacle_mask, dtype=np.float64).ravel()
    v = np.asarray(is_valid, dtype=bool).ravel()
    out = np.where(m < 0, -1.0, np.where(v, 1.0, 0.0))
    if pts is None or robot_pose is None:
        return (out, np.zeros(out.shape, dtype=bool)) if return_core else out
    rx, ry, ryaw = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    if len(p) != len(out):
        return (out, np.zeros(out.shape, dtype=bool)) if return_core else out
    dx, dy = p[:, 0] - rx, p[:, 1] - ry
    c, s_ = np.cos(ryaw), np.sin(ryaw)
    xb, yb = np.abs(dx * c + dy * s_), np.abs(-dx * s_ + dy * c)
    hl, hw = GLOBAL_MAP_SELF_BODY_LENGTH_M / 2.0, GLOBAL_MAP_SELF_BODY_WIDTH_M / 2.0
    core = (xb <= hl) & (yb <= hw)
    ring = ~core & (xb <= hl + GLOBAL_MAP_SELF_RING_M) & (yb <= hw + GLOBAL_MAP_SELF_RING_M)
    out[ring & (out < 0)] = 0.0
    out[core] = 1.0
    return (out, core) if return_core else out


def fold_scan_into_global_maps(global_map, global_terrain_map, pts, terrain, obstacle_mask, is_valid,
                               cells_obstacle_dist, shape, robot_pose):
    """
    Una scansione nelle due mappe globali: occupazione (con la zona attorno al
    corpo, vedi _mask_for_global_map) e terreno. Unico punto usato dalla missione (inizio
    tentativo, ogni giro, scansione finale) e da test_replay.py, cosi' la prova di rigioco
    verifica il codice vero e non una sua copia.

    La mappa del terreno riceve la STESSA maschera del calcolo dal vivo (slope_usable_mask:
    attendibili E non ostacolo): le quote delle celle ostacolo trasformerebbero tronchi e
    muri in "pendenze" nella mappa globale.

    Restituisce la maschera del nucleo (celle sotto il corpo), che serve alla mappa dello
    spazio libero.
    """
    gm_mask, gm_core = mask_for_global_map(obstacle_mask, is_valid, pts, robot_pose, return_core=True)
    global_map.update(pts, gm_mask, walked=gm_core)
    global_terrain_map.update(pts, terrain, slope_usable_mask(is_valid, cells_obstacle_dist, shape))
    return gm_core


def update_clearance_map(global_map, pts, snap, is_valid, unwritten, core, robot_xy):
    """
    Alimenta la mappa dello spazio libero con la scansione corrente.

    Usa la obstacle_distance FILTRATA (quella su cui il fronte decide davvero, mediana +
    isteresi: vedi arcVerification), non la lettura grezza. Imparano solo le celle
    attendibili; sotto il corpo il valore non puo' essere minore di mezza larghezza, perche'
    il robot ci sta sopra (stesso principio di _under_body nel fronte).
    """
    if not hasattr(global_map, 'update_clearance') or snap is None:
        return
    try:
        od = np.asarray(snap.obstacle_dist, dtype=np.float64).ravel()
        v = np.asarray(is_valid, dtype=bool).ravel()
        if unwritten is not None and np.size(unwritten) == v.size:
            v = v & ~np.asarray(unwritten, dtype=bool).ravel()
        c = np.asarray(core, dtype=bool).ravel()
        if od.size != v.size or c.size != v.size:
            return
        usable = v | c
        od = np.where(c, np.maximum(od, ROBOT_HALF_WIDTH_M), od)
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        near = np.hypot(p[:, 0] - robot_xy[0], p[:, 1] - robot_xy[1]) <= CLEARANCE_NEAR_M
        global_map.update_clearance(p, od, usable, near=near)
    except Exception as e:      # mai fatale: e' una mappa di appoggio
        print(f"[SPAZIO] Mappa dello spazio libero non aggiornata: {e}")


def slope_usable_mask(is_valid_flat, cells_obstacle_dist, shape):
    """
    Maschera delle celle le cui QUOTE possono
    entrare nel calcolo della pendenza di un arco: attendibili (is_valid di correct_terrain)
    E non-ostacolo.

    La faccia verticale di un mobile non e' terreno in pendenza, e' un ostacolo -- ed e' gia'
    gestita come tale due volte: dal veto di occupazione del PRM e dal fronte sicuro, che si
    ferma a FRONTIER_CLEARANCE_M da qualunque cella con obstacle_distance bassa. Contarla
    ANCHE come pendenza la conta la terza volta, e con il peggiore degli effetti: non ferma
    il robot davanti al mobile (ci pensa il fronte), scollega il grafo PRM in tutta la stanza.

    Gli archi che perdono il dato NON diventano "liberi per decreto": diventano "senza dati",
    e un arco senza dati non viene vetato (entra nel grafo con un costo di ripiego, vedi
    PRM._evaluate_and_add_edge); chi lo percorre lo verifica metro per metro col fronte
    sicuro. E' una scelta esplicita: su quegli archi la sicurezza la da' il fronte, non una
    pendenza misurata su un armadio.

    La maschera si combina QUI e non dentro prm_graph perche' e' qui che si hanno in mano
    entrambi gli array, e perche' in prm_graph `current_valid_2d` serve a una cosa sola -- il
    profilo di pendenza per-arco (compute_arc_slope_profile) -- quindi restringerla non ha
    altri effetti. Chi legge prm_graph.update_local_grid_data trova `valid_2d` documentato
    come "celle attendibili": qui significa "attendibili come TERRENO".
    """
    valid = np.asarray(is_valid_flat, dtype=bool).reshape(shape)
    if cells_obstacle_dist is None:
        return valid
    od = np.asarray(cells_obstacle_dist, dtype=np.float64).reshape(shape)
    return valid & ~(od < OBSTACLE_THRESHOLD)


def raw_zero(grids_data):
    """Quota del valore grezzo 0 del layer terrain di questa scansione (None se assente)."""
    try:
        return grids_data['terrain'].get('raw_zero')
    except (KeyError, TypeError, AttributeError):
        return None


def raw_scale(grids_data):
    try:
        return grids_data['terrain'].get('scale')
    except (KeyError, TypeError, AttributeError):
        return None


def segment_diagnostics(terrain_2d, origin_x, origin_y, cell_size, x1, y1, x2, y2):
    """Stima alternativa (interpolata) e copertura dei campioni per un segmento. Solo log:
    la stima in uso per decidere resta quella di PRM.sample_terrain_between_points."""
    out = {'long_interp': None, 'lat_interp': None, 'coverage_along': None, 'coverage_lateral': None}
    try:
        li, lai, ok = compute_arc_slope_profile_interp(
            terrain_2d, origin_x, origin_y, cell_size, x1, y1, x2, y2)
        if ok:
            out['long_interp'], out['lat_interp'] = li, lai
        out['coverage_along'], out['coverage_lateral'] = compute_arc_sample_coverage(
            terrain_2d, origin_x, origin_y, cell_size, x1, y1, x2, y2)
    except Exception as e:
        print(f"[LOG] Diagnostica del segmento non disponibile: {e}")
    return out


def log_ground_filter(local_grid, label):
    """
    Una riga solo quando il filtro sull'altezza del suolo ha scartato qualcosa o non ha
    potuto stimare il suolo -- nel caso normale tace. Vedi spotGrid.GROUND_HEIGHT_TOLERANCE_M.
    """
    info = getattr(local_grid, 'last_ground_filter', None) or {}
    if info.get('ground_z') is None:
        print(f"[GROUND-FILTER] {label}: suolo non stimabile ({info.get('source')}, "
              f"{info.get('n_ref', 0)} celle di riferimento) -- filtro non applicato")
    elif info.get('n_above', 0) > 0:
        print(f"[GROUND-FILTER] {label}: {info['n_above']} celle scartate, fino a "
              f"{info['max_above']:.2f} m sopra il suolo (suolo z={info['ground_z']:.3f}, "
              f"stimato {info['source']} su {info['n_ref']} celle)")
    if info.get('n_unwritten', 0) > 0:
        print(f"[GROUND-FILTER] {label}: {info['n_unwritten']} celle mai scritte dai sensori "
              f"({info.get('n_unwritten_below', 0)} sotto il suolo) -> trattate come non viste"
              + (f", quota {info['quota_unwritten']:+.4f} m" if info.get('quota_unwritten') is not None else ""))


def log_footprint_diagnostic(local_grid, pts, robot_x, robot_y, robot_yaw,
                              num_x, num_y, obstacle_mask, label,
                              cells_obstacle_dist=None, rough_values=None, is_valid=None):
    """
    Misura quante celle sotto l'impronta del robot risultano ostacolo, e PERCHE'.
    PURAMENTE OSSERVATIVO: usa diagnostic_only=True, quindi non consuma il flag
    one-shot di compute_robot_footprint_mask e non cambia nessun comportamento.

    La maschera dell'impronta si applica una volta sola: applicarla a ogni scansione
    nasconderebbe muri veri in un corridoio stretto. Celle ostacolo NEL NUCLEO, quando
    compaiono, sono di solito celle mai scritte dai sensori (sotto il corpo non possono
    scrivere) lasciate alla quota sbagliata, che fanno esplodere la rugosita': le toglie
    spotGrid._cells_never_written, non questa maschera.

    PERCHE' -- i due veti di fuse_obstacle_mask, ricalcolati identici:
      - "prossimita'": cells_obstacle_dist <= OBSTACLE_THRESHOLD. NON vuol dire
        "qui c'e' un ostacolo", vuol dire "qui sei a meno di 15 cm da un ostacolo".
        Se domina questo, il footprint NON c'entra: il robot sta davvero passando
        vicino a qualcosa, e mascherare cancellerebbe proprio quell'avviso.
      - "rugosita'": rough_values > ROUGH_THRESHOLD su celle che il sensore dichiara
        valide.

    DOVE -- rettangolo intero (1.10 x 0.50 + margine) contro il solo NUCLEO
    (0.90 x 0.35), cioe' la fascia entro la carreggiata delle zampe. Un ostacolo
    dentro il nucleo significherebbe che il robot ci sta gia' sopra, quindi mascherare li'
    non puo' nascondere nulla di raggiungibile, mentre mascherare l'intero rettangolo in un
    corridoio stretto puo' cancellare un muro vero.

    Nota: anche arcVerification.py chiama compute_robot_footprint_mask, quindi in
    condizioni di corsa puo' essere il thread di background a consumare l'unica
    maschera prevista, lasciando senza il ciclo principale.
    """
    try:
        fp = local_grid.compute_robot_footprint_mask(
            pts, robot_x, robot_y, robot_yaw, num_x, num_y, diagnostic_only=True)
        if fp is None:
            return
        fp_flat = fp.ravel()
        obs_flat = np.asarray(obstacle_mask).ravel()
        n_fp = int(fp_flat.sum())
        if n_fp == 0:
            return
        blocked = (obs_flat < 0) & fp_flat
        n_obs = int(blocked.sum())

        line = (f"[FOOTPRINT-DIAG] {label}: celle sotto il corpo marcate ostacolo: "
                f"{n_obs}/{n_fp} ({100.0 * n_obs / n_fp:.1f}%)")

        # --- PERCHE': i due veti separati, stessa identica logica di fuse_obstacle_mask ---
        if n_obs > 0 and cells_obstacle_dist is not None and rough_values is not None \
                and is_valid is not None:
            d = np.asarray(cells_obstacle_dist).ravel()
            r = np.asarray(rough_values).ravel()
            v = np.asarray(is_valid).ravel()
            if d.shape == fp_flat.shape and r.shape == fp_flat.shape and v.shape == fp_flat.shape:
                v1 = (d <= OBSTACLE_THRESHOLD) & v   # prossimita' a un ostacolo
                v3 = (r > ROUGH_THRESHOLD) & v       # terreno irregolare
                n_prox = int(np.sum(v1 & fp_flat & ~v3))
                n_rough = int(np.sum(v3 & fp_flat & ~v1))
                n_both = int(np.sum(v1 & v3 & fp_flat))
                n_invalid = int(np.sum(~v & fp_flat))
                line += (f"\n                 perche': prossimita'={n_prox}  rugosita'={n_rough}  "
                         f"entrambi={n_both}   (celle non valide sotto il corpo: {n_invalid})")

        # --- DOVE: nucleo (dentro la carreggiata delle zampe) contro corona esterna ---
        if n_obs > 0:
            core = local_grid.compute_robot_footprint_mask(
                pts, robot_x, robot_y, robot_yaw, num_x, num_y,
                diagnostic_only=True, body_length=0.90, body_width=0.35)
            if core is not None:
                core_flat = core.ravel()
                n_core_fp = int(core_flat.sum())
                n_core_obs = int(np.sum(blocked & core_flat))
                n_ring_obs = n_obs - n_core_obs
                line += (f"\n                 dove: nucleo={n_core_obs}/{n_core_fp}  "
                         f"corona esterna={n_ring_obs}/{n_fp - n_core_fp}")

        print(line)
    except Exception as e:
        print(f"[FOOTPRINT-DIAG] diagnostica non disponibile: {e}")


def log_clearance_gap(prm, global_map):
    """
    Una riga per giro sul divario fra i due criteri di percorribilita':
    la mappa binaria (bit occupato + margine) e la mappa dello spazio libero (stessa soglia
    del fronte). Tace quando sono d'accordo. Serve a decidere sul campo se commutare
    spotGrid.CLEARANCE_MAP_DECIDES.
    """
    d = getattr(prm, 'clearance_disagreement', None)
    if not d:
        return
    only_bin, only_clear, tot = d
    if not (only_bin or only_clear) or not tot:
        return
    chi = "lo spazio libero" if CLEARANCE_MAP_DECIDES else "la mappa binaria"
    n_cells = len(getattr(global_map, 'clear_all_n', ()) or ())
    print(f"[SPAZIO] Archi su cui i due criteri non concordano: {only_bin + only_clear} su "
          f"{tot} ({100.0 * (only_bin + only_clear) / tot:.1f}%) -- "
          f"{only_bin} bloccati solo dalla mappa binaria, {only_clear} solo dallo spazio "
          f"libero. Adesso decide {chi}. Celle con spazio libero noto: {n_cells}.")
