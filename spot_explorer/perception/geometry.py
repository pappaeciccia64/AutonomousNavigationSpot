"""Ingombro del robot e distanze dagli ostacoli: fronte sicuro, pianificatore, rotazione sul posto."""
import numpy as np

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
