"""
Fronte sicuro: fin dove il percorso pianificato e' confermato libero ADESSO.

A ogni scansione, partendo dalla posizione del robot, si percorre la spezzata del percorso
(prolungata oltre l'obiettivo di mezzo corpo, per il muso) e ci si ferma al primo punto non
libero. Il risultato si ricalcola da zero a ogni scansione: nessun verdetto sopravvive alle
condizioni in cui e' stato preso.

Motivi per cui il fronte si ferma:
  'ostacolo'      obstacle_distance sotto la distanza minima dalla linea del percorso
  'rugosita'      cella vetata per rugosita' (stesso criterio di fuse_obstacle_mask)
  'pendenza'      tratto con pendenza longitudinale o laterale oltre SLOPE_THRESHOLD
  'fine_dati'     fuori dalla finestra, o cella mai vista dal sensore
  'fine_percorso' tutto il percorso e' libero

'fine_dati' non vuol dire "bloccato" ma "avvicinati e guarda".

Il modulo contiene anche il thread che ricalcola il fronte durante il movimento
(ArcVerificationTracker), usato per fermare il robot se qualcosa entra nel percorso.

Nota sul nome "FOV" in is_arc_in_fov: e' l'estensione della griglia locale (3.84 m di lato
attorno al robot), non il campo visivo delle telecamere.
"""
import threading
import time
import logging

import numpy as np

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, BODY_FRAME_NAME

import parametri as P
import prm_graph
import spotUtils
import spotGrid


LOGGER = logging.getLogger(__name__)


# =========================================================================
# FILTRO TEMPORALE SU obstacle_distance
#
# obstacle_distance oscilla di qualche centimetro fra due scansioni anche a robot fermo
# (1.5 cm in media, 7.3 cm di punta), cioe' quanto il margine d'aria con cui lo si
# confronta: senza filtro un percorso che sfiora un ostacolo passa da "libero" a "bloccato"
# e ritorno a ogni scansione. Due rimedi:
#   - MEDIANA sulle ultime OBSTACLE_MEDIAN_FRAMES letture della stessa cella di mondo: una
#     punta isolata non decide da sola. Costa al massimo una scansione di ritardo.
#   - ISTERESI: una cella diventa bloccante sotto FRONTIER_CLEARANCE_M e torna libera solo
#     sopra FRONTIER_CLEARANCE_M + OBSTACLE_HYSTERESIS_M.
#
# Le origini delle griglie di Spot sono multipli interi della cella, quindi riallineare due
# scansioni e' uno scorrimento di indici, senza interpolazione. Se lo scorrimento non e'
# intero o cambia la dimensione della cella, la cronologia si azzera.
# =========================================================================
OBSTACLE_MEDIAN_FRAMES = 3
OBSTACLE_HYSTERESIS_M = 0.04


class _ObstacleHistory:
    """
    Cronologia di obstacle_distance allineata al mondo, condivisa fra il ciclo principale
    e il thread del fronte (le scansioni vengono dallo stesso robot a frazioni di secondo
    di distanza: piu' campioni rendono la mediana migliore, non peggiore).

    update() restituisce (mediana, latch): la mediana da usare al posto della lettura
    grezza, e la maschera booleana delle celle che l'isteresi considera bloccanti.
    """

    def __init__(self, frames=OBSTACLE_MEDIAN_FRAMES, hysteresis_m=OBSTACLE_HYSTERESIS_M):
        self._lock = threading.Lock()
        self._frames = max(1, int(frames))
        self._hyst = float(hysteresis_m)
        self._hist = []          # [(od 2D, ix, iy)] -- ix/iy = origine in CELLE di mondo
        self._latch = None       # 2D bool, allineata all'ultimo elemento di _hist
        self._latch_ij = None
        self._cell_size = None

    def reset(self):
        with self._lock:
            self._hist = []
            self._latch = None
            self._latch_ij = None
            self._cell_size = None

    @staticmethod
    def _align(src, dix, diy, shape):
        """
        src riletta negli indici di una griglia spostata di (dix, diy) celle: il valore
        che la nuova griglia vede in (r, c) sta in src[r + diy, c + dix]. Fuori da src:
        NaN per i float, False per i booleani.
        """
        rows, cols = shape
        fill = False if src.dtype == bool else np.nan
        out = np.full(shape, fill, dtype=src.dtype if src.dtype == bool else np.float64)
        r0, c0 = max(0, diy), max(0, dix)                       # primo indice valido in src
        rr0, cc0 = max(0, -diy), max(0, -dix)                   # dove finisce in out
        nr = min(src.shape[0] - r0, rows - rr0)
        nc = min(src.shape[1] - c0, cols - cc0)
        if nr > 0 and nc > 0:
            out[rr0:rr0 + nr, cc0:cc0 + nc] = src[r0:r0 + nr, c0:c0 + nc]
        return out

    # Tolleranza sul resto dello scorrimento, in frazione di cella. Lo scorrimento si misura
    # fra due griglie CONSECUTIVE, mai da un'origine assoluta: cell_size arriva come float32
    # (0.029999999329447746), e su un'origine lontana l'errore supererebbe la tolleranza.
    _SHIFT_TOL_CELLS = 0.05

    def update(self, od, origin_x, origin_y, cell_size):
        od = np.asarray(od, dtype=np.float64)
        cs = float(cell_size)
        ox, oy = float(origin_x), float(origin_y)

        def shift_from(old_ox, old_oy):
            """(dix, diy) in celle, oppure None se lo scorrimento non e' intero."""
            fx, fy = (ox - old_ox) / cs, (oy - old_oy) / cs
            dix, diy = int(round(fx)), int(round(fy))
            if abs(fx - dix) > self._SHIFT_TOL_CELLS or abs(fy - diy) > self._SHIFT_TOL_CELLS:
                return None
            return dix, diy

        with self._lock:
            if self._cell_size is not None and abs(self._cell_size - cs) > 1e-9:
                self._hist = []
                self._latch = None
                self._latch_ij = None
            self._cell_size = cs

            stack = [od]
            for (old, old_ox, old_oy) in self._hist:
                if old.shape != od.shape:
                    continue
                sh = shift_from(old_ox, old_oy)
                if sh is None:
                    continue
                stack.append(self._align(old, sh[0], sh[1], od.shape))
            if len(stack) > 1:
                with np.errstate(invalid='ignore'):
                    med = np.nanmedian(np.stack(stack, axis=0), axis=0)
                med = np.where(np.isfinite(med), med, od)
            else:
                med = od.copy()

            thr = spotGrid.FRONTIER_CLEARANCE_M
            latch = med < thr
            if self._latch is not None and self._latch_ij is not None:
                sh = shift_from(self._latch_ij[0], self._latch_ij[1])
                if sh is not None:
                    prev = self._align(self._latch, sh[0], sh[1], od.shape)
                    latch |= prev & (med < thr + self._hyst)

            self._hist.append((od, ox, oy))
            if len(self._hist) > self._frames:
                self._hist.pop(0)
            self._latch = latch
            self._latch_ij = (ox, oy)

        return med, latch


OBSTACLE_HISTORY = _ObstacleHistory()


class GridSnapshot:
    """
    Una scansione pronta per il fronte. Tutti gli array sono 2D (num_y, num_x), con lo
    stesso ordinamento usato ovunque nel progetto: riga = y, colonna = x, origine nel
    vertice in basso a sinistra (vedi spotGrid._gather_heights).

    blocked_latch: maschera dell'isteresi (vedi _ObstacleHistory); None = nessun filtro.
    """
    __slots__ = ('obstacle_dist', 'rough_veto', 'seen', 'terrain', 'origin_x', 'origin_y',
                 'cell_size', 'robot_x', 'robot_y', 'time', 'valid', 'blocked_latch', 'robot_yaw')

    def __init__(self, obstacle_dist, rough_veto, seen, terrain, origin_x, origin_y, cell_size,
                 robot_x, robot_y, timestamp=None, valid=None, blocked_latch=None, robot_yaw=None):
        self.blocked_latch = blocked_latch
        # Imbardata del robot nella stessa fotografia, per riconoscere le celle sotto il
        # corpo (vedi _under_body). None = non nota: si usa il cerchio inscritto.
        self.robot_yaw = None if robot_yaw is None else float(robot_yaw)
        self.obstacle_dist = obstacle_dist
        self.rough_veto = rough_veto
        self.seen = seen
        self.terrain = terrain
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size = float(cell_size)
        self.robot_x = float(robot_x)
        self.robot_y = float(robot_y)
        self.time = time.time() if timestamp is None else timestamp
        self.valid = valid      # is_valid 2D (None = tutte attendibili): per il profilo di pendenza


def make_grid_snapshot(cells_obstacle_dist, rough_values, is_valid, terrain_valid_raw,
                       terrain_corrected, num_x, num_y, origin_x, origin_y, cell_size,
                       robot_x, robot_y, unwritten=None, robot_yaw=None):
    """
    Costruisce la fotografia dai vettori piatti che il codice calcola gia' a ogni scansione.

    - rough_veto: stesso criterio di fuse_obstacle_mask (rugosita' > soglia, su cella valida).
    - seen: il sensore ha dato una quota per quella cella (terrain_valid grezzo, confrontato
      con spotGrid.TERRAIN_VALID_MIN e non con 0: dopo scala e offset lo 0 diventa +/-5e-17).
      Non e' is_valid: is_valid esclude anche le celle scartate dal filtro sull'altezza del
      suolo (muri), che invece sono state viste; l'ostacolo lo segnala obstacle_distance.
    - unwritten: celle mai scritte dai sensori (correct_terrain con return_unwritten=True):
      contano come NON viste, quindi il fronte si ferma li' con 'fine_dati'.
    """
    shape = (num_y, num_x)
    rough = np.asarray(rough_values, dtype=np.float64).reshape(shape)
    valid = np.asarray(is_valid, dtype=bool).reshape(shape)
    if terrain_valid_raw is not None and np.size(terrain_valid_raw) == num_x * num_y:
        seen = np.asarray(terrain_valid_raw).reshape(shape) >= spotGrid.TERRAIN_VALID_MIN
    else:
        seen = np.ones(shape, dtype=bool)
    if unwritten is not None and np.size(unwritten) == num_x * num_y:
        seen = seen & ~np.asarray(unwritten, dtype=bool).reshape(shape)
    # Filtro temporale + isteresi su obstacle_distance (vedi _ObstacleHistory): fronte e
    # controllo di rotazione leggono questa fotografia, non la lettura grezza.
    od_raw = np.asarray(cells_obstacle_dist, dtype=np.float64).reshape(shape)
    od_filtered, latch = OBSTACLE_HISTORY.update(od_raw, origin_x, origin_y, cell_size)

    return GridSnapshot(
        obstacle_dist=od_filtered,
        rough_veto=(rough > spotGrid.ROUGH_THRESHOLD) & valid,
        seen=seen,
        terrain=np.asarray(terrain_corrected, dtype=np.float64).reshape(shape),
        origin_x=origin_x, origin_y=origin_y, cell_size=cell_size,
        robot_x=robot_x, robot_y=robot_y, valid=valid, blocked_latch=latch, robot_yaw=robot_yaw)


def _under_body(snap, xs, ys):
    """
    True per i punti che stanno SOTTO IL CORPO del robot adesso: rettangolo
    +/-ROBOT_HALF_LENGTH_M x +/-ROBOT_HALF_WIDTH_M attorno alla posa, orientato con
    l'imbardata della fotografia. Senza imbardata, il cerchio inscritto (raggio mezza
    larghezza), che sta sotto il corpo in qualunque orientamento.

    Li' c'e' il robot, quindi non c'e' un ostacolo in cui entrare: quei punti non fermano
    mai il fronte (stesso principio di spotGrid.rotation_is_clear).
    """
    dx = np.asarray(xs, dtype=np.float64) - snap.robot_x
    dy = np.asarray(ys, dtype=np.float64) - snap.robot_y
    yaw = getattr(snap, 'robot_yaw', None)
    if yaw is None:
        return np.hypot(dx, dy) <= spotGrid.ROBOT_HALF_WIDTH_M
    c, s = np.cos(yaw), np.sin(yaw)
    along = c * dx + s * dy
    across = -s * dx + c * dy
    return (np.abs(along) <= spotGrid.ROBOT_HALF_LENGTH_M) & (np.abs(across) <= spotGrid.ROBOT_HALF_WIDTH_M)


def _cells(snap, xs, ys):
    """Indici di cella per indicizzazione diretta (niente ricerca del vicino) + maschera 'dentro'."""
    ny, nx = snap.obstacle_dist.shape
    cols = np.floor((xs - snap.origin_x) / snap.cell_size).astype(np.int64)
    rows = np.floor((ys - snap.origin_y) / snap.cell_size).astype(np.int64)
    inside = (rows >= 0) & (rows < ny) & (cols >= 0) & (cols < nx)
    return np.clip(rows, 0, ny - 1), np.clip(cols, 0, nx - 1), inside


def _polyline_samples(polyline, step):
    """Campioni equispaziati lungo la spezzata: (xs, ys, s, indice del segmento)."""
    xs, ys, ss, seg = [], [], [], []
    s0 = 0.0
    for k in range(len(polyline) - 1):
        (x1, y1), (x2, y2) = polyline[k], polyline[k + 1]
        L = float(np.hypot(x2 - x1, y2 - y1))
        if L < 1e-9:
            continue
        n = max(1, int(np.ceil(L / step)))
        t = np.arange(n) / n                      # l'ultimo punto e' il primo del segmento dopo
        xs.append(x1 + t * (x2 - x1)); ys.append(y1 + t * (y2 - y1))
        ss.append(s0 + t * L); seg.append(np.full(n, k))
        s0 += L
    if not xs:
        x, y = polyline[0]
        return np.array([x]), np.array([y]), np.array([0.0]), np.array([0]), 0.0
    x_last, y_last = polyline[-1]
    xs.append([x_last]); ys.append([y_last]); ss.append([s0]); seg.append([len(polyline) - 2])
    return (np.concatenate(xs), np.concatenate(ys), np.concatenate(ss),
            np.concatenate(seg).astype(int), s0)


def compute_safe_frontier(polyline, snap, extend_end_m=None, check_slope=True, body=None,
                          air_m=None):
    """
    Fin dove il percorso e' confermato libero adesso.

    Args:
        polyline: [(x, y), ...] -- il PRIMO punto e' la posizione del robot, poi i waypoint.
        snap: GridSnapshot della scansione.
        extend_end_m: prolunga l'ultimo segmento di questa lunghezza oltre l'obiettivo, per
            verificare anche il terreno sotto il muso quando il robot ci arriva. Default
            FRONTIER_BODY_HALF_LENGTH_M.
        check_slope: controlla anche la pendenza dei tratti liberi.
        body: (meta' ingombro lungo la marcia, meta' ingombro di traverso), vedi
            spotGrid.body_extents. Default: robot che guarda dove va (0.55, 0.25).
        air_m: aria fra fianco e ostacolo. None = ROBOT_CLEARANCE_AIR_M (0.05), cioe' la
            distanza minima 0.30 con cui si DECIDE di partire. Il thread di arresto passa
            spotGrid.FRONTIER_ABORT_AIR_M (0.0): a movimento in corso ci si ferma solo per
            un'intrusione vera, a meno di mezza larghezza (0.25). I 5 cm fra le due soglie
            evitano di fermarsi su un tratto appena dichiarato libero. Con un'aria diversa
            dalla nominale l'isteresi non si applica (e' tarata sulla soglia 0.30).

        I campioni sotto il corpo del robot adesso non sono mai ostacolo (vedi _under_body).

    Returns: dict con
        dist        distanza lungo il percorso del primo punto NON libero (m)
        reason      vedi docstring del modulo
        stop_xy     dove si ferma il fronte
        value       il valore che ha fermato il fronte (obstacle_distance, pendenza, ...)
        path_len    lunghezza del percorso vero, senza prolungamento
        od_robot    obstacle_distance sotto il robot
        clearance   distanza minima applicata nel primo tratto
        robot_xy, time
    """
    half_along, half_across = body if body is not None else (spotGrid.FRONTIER_BODY_HALF_LENGTH_M,
                                                             spotGrid.ROBOT_HALF_WIDTH_M)
    use_latch = air_m is None
    clearance_nominal = half_across + (spotGrid.ROBOT_CLEARANCE_AIR_M if air_m is None else float(air_m))
    if extend_end_m is None:
        extend_end_m = half_along
    pl = [(float(x), float(y)) for x, y in polyline]

    # Lunghezza vera e prolungamento oltre l'obiettivo
    path_len = float(sum(np.hypot(pl[k + 1][0] - pl[k][0], pl[k + 1][1] - pl[k][1])
                         for k in range(len(pl) - 1)))
    if len(pl) >= 2 and extend_end_m > 0:
        (xa, ya), (xb, yb) = pl[-2], pl[-1]
        L = np.hypot(xb - xa, yb - ya)
        if L > 1e-6:
            pl.append((xb + extend_end_m * (xb - xa) / L, yb + extend_end_m * (yb - ya) / L))

    xs, ys, ss, seg, _ = _polyline_samples(pl, spotGrid.FRONTIER_SAMPLE_STEP_M)
    rows, cols, inside = _cells(snap, xs, ys)

    # obstacle_distance sotto il robot: decide se siamo gia' in uno spazio stretto
    r0, c0, in0 = _cells(snap, np.array([snap.robot_x]), np.array([snap.robot_y]))
    od_robot = float(snap.obstacle_dist[r0[0], c0[0]]) if in0[0] else np.inf
    clearance_escape = max(spotGrid.FRONTIER_MIN_CLEARANCE_M,
                           min(clearance_nominal, od_robot - 0.02))
    thr = np.where(ss < spotGrid.FRONTIER_ESCAPE_DIST_M, clearance_escape, clearance_nominal)

    od = snap.obstacle_dist[rows, cols]
    near_robot = np.hypot(xs - snap.robot_x, ys - snap.robot_y) <= spotGrid.FRONTIER_ROBOT_RADIUS_M
    under_body = _under_body(snap, xs, ys)

    no_data = ~inside | (~snap.seen[rows, cols] & ~near_robot)
    obstacle = inside & (od < thr) & ~under_body
    # Isteresi: una cella gia' bloccante resta tale finche' non risale sopra
    # FRONTIER_CLEARANCE_M + OBSTACLE_HYSTERESIS_M (vedi _ObstacleHistory). Solo dove vale la
    # soglia nominale: dove la regola di uscita l'ha abbassata il robot sta uscendo da uno
    # spazio stretto, e l'isteresi lo inchioderebbe li'.
    if snap.blocked_latch is not None and use_latch:
        nominal = thr >= clearance_nominal - 1e-9
        obstacle |= inside & nominal & snap.blocked_latch[rows, cols] & ~under_body
    # Regola di uscita da uno spazio stretto: nel primo metro (FRONTIER_ESCAPE_DIST_M) la
    # soglia scende a od_robot - 2 cm, ma vale la promessa "non avvicinarsi piu' di quanto
    # gia' si sia": un campione sotto la soglia nominale e' ammesso solo se non scende sotto
    # il meglio raggiunto finora lungo il percorso (meno la tolleranza). Allontanarsi si',
    # riavvicinarsi no: un palo sul fianco poco piu' avanti resta bloccante.
    if clearance_escape < clearance_nominal:
        od_capped = np.minimum(np.where(inside, od, clearance_nominal), clearance_nominal)
        # Sotto il corpo il riferimento e' la distanza del robot adesso, non le letture sotto
        # la pancia (che alzerebbero "il meglio raggiunto" senza motivo).
        od_capped = np.where(under_body, min(od_robot, clearance_nominal), od_capped)
        best_so_far = np.maximum.accumulate(od_capped)
        in_escape = inside & (ss < spotGrid.FRONTIER_ESCAPE_DIST_M) & (od < clearance_nominal) & ~under_body
        # Tolleranza: almeno 1.5 celle. obstacle_distance va a gradini di una cella (3 cm), e
        # lungo un muro quasi parallelo al percorso un gradino non deve contare come
        # riavvicinamento.
        tol = max(0.02, 1.5 * snap.cell_size)
        obstacle = obstacle | (in_escape & (od < best_so_far - tol))
    rough = inside & snap.rough_veto[rows, cols] & ~near_robot

    bad = no_data | obstacle | rough
    result = dict(path_len=path_len, od_robot=od_robot, clearance=clearance_escape,
                  clearance_nominal=clearance_nominal, half_along=half_along,
                  robot_xy=(snap.robot_x, snap.robot_y), time=snap.time)

    if bad.any():
        k = int(np.argmax(bad))
        if no_data[k]:
            reason, value = 'fine_dati', None
        elif obstacle[k]:
            reason, value = 'ostacolo', float(od[k])
            # La soglia EFFETTIVA in quel punto e se ha deciso l'isteresi, per il log: senza,
            # "ostacolo a 0.27 m (minimo 0.25 m)" sembrerebbe una contraddizione.
            result['stop_threshold'] = float(thr[k])
            result['stop_by_latch'] = bool(
                snap.blocked_latch is not None and use_latch
                and snap.blocked_latch[rows[k], cols[k]] and od[k] >= thr[k])
        else:
            reason, value = 'rugosita', None
        dist, stop_xy = float(ss[k]), (float(xs[k]), float(ys[k]))
    else:
        reason, value = 'fine_percorso', None
        dist, stop_xy = float(ss[-1]), (float(xs[-1]), float(ys[-1]))

    # Pendenza sui tratti gia' confermati liberi, segmento per segmento: se un tratto e'
    # troppo ripido il fronte arretra al suo inizio. Stessa funzione e stessa soglia del PRM
    # sull'arco fra i nodi (il PRM usa in piu' il valore col corpo, quindi e' piu' severo:
    # il fronte non puo' rifiutare per pendenza un arco che il PRM ha ammesso).
    if check_slope and spotGrid.FRONTIER_SLOPE_MIN_LENGTH_M > 0:
        s_start = 0.0
        for k in range(len(pl) - 1):
            (x1, y1), (x2, y2) = pl[k], pl[k + 1]
            L = float(np.hypot(x2 - x1, y2 - y1))
            if s_start >= dist:
                break
            free_len = min(L, dist - s_start)
            if free_len >= spotGrid.FRONTIER_SLOPE_MIN_LENGTH_M:
                f = free_len / L
                long_s, lat_s, has = spotGrid.compute_arc_slope_profile(
                    snap.terrain, snap.origin_x, snap.origin_y, snap.cell_size,
                    x1, y1, x1 + f * (x2 - x1), y1 + f * (y2 - y1), valid_2d=snap.valid)
                if has and max(long_s, lat_s) > spotGrid.SLOPE_THRESHOLD:
                    reason, value = 'pendenza', float(max(long_s, lat_s))
                    dist, stop_xy = s_start, (x1, y1)
                    break
            s_start += L

    result.update(dist=dist, reason=reason, stop_xy=stop_xy, value=value)
    return result


def allowed_advance(frontier):
    """
    Quanto puo' avanzare il CENTRO del robot lungo il percorso.

    Se tutto il percorso (prolungamento compreso) e' libero, fino all'obiettivo. Altrimenti
    il centro si ferma meta' lunghezza del corpo + margine prima del primo punto non libero,
    cosi' anche il muso resta su terreno confermato. Vale anche per 'fine_dati': il muso
    non deve finire su terreno mai visto.
    """
    if frontier['reason'] == 'fine_percorso':
        return frontier['path_len']
    half_along = frontier.get('half_along', spotGrid.FRONTIER_BODY_HALF_LENGTH_M)
    return max(0.0, frontier['dist'] - half_along - spotGrid.FRONTIER_MARGIN_M)


def point_along(polyline, s):
    """Il punto a distanza s lungo la spezzata (troncato agli estremi) e l'indice del segmento."""
    s_acc = 0.0
    for k in range(len(polyline) - 1):
        (x1, y1), (x2, y2) = polyline[k], polyline[k + 1]
        L = float(np.hypot(x2 - x1, y2 - y1))
        if s <= s_acc + L or k == len(polyline) - 2:
            t = 0.0 if L < 1e-9 else min(1.0, max(0.0, (s - s_acc) / L))
            return (x1 + t * (x2 - x1), y1 + t * (y2 - y1)), k
        s_acc += L
    return polyline[-1], max(0, len(polyline) - 2)


def describe_frontier(frontier):
    """Una riga leggibile per il log."""
    r = frontier['reason']
    adv = allowed_advance(frontier)
    sx, sy = frontier['stop_xy']
    base = (f"{frontier['dist']:.2f} m confermati su {frontier['path_len']:.2f} m di percorso, "
            f"avanzamento consentito {adv:.2f} m")
    if r == 'fine_percorso':
        return f"[FRONTE] {base} -- percorso libero fino all'obiettivo"
    if r == 'fine_dati':
        return f"[FRONTE] {base} -- si ferma sul bordo dei dati in ({sx:.2f}, {sy:.2f}): avanzo e riguardo"
    if r == 'ostacolo':
        thr = frontier.get('stop_threshold', frontier.get('clearance_nominal',
                                                          spotGrid.FRONTIER_CLEARANCE_M))
        if frontier.get('stop_by_latch'):
            why = (f"(minimo {thr:.2f} m; qui decide l'isteresi, che tiene bloccante la cella "
                   f"fino a {thr + OBSTACLE_HYSTERESIS_M:.2f} m)")
        else:
            why = f"(minimo {thr:.2f} m)"
        return (f"[FRONTE] {base} -- ostacolo a {frontier['value']:.2f} m dalla linea centrale in "
                f"({sx:.2f}, {sy:.2f}) {why}")
    if r == 'rugosita':
        return f"[FRONTE] {base} -- terreno troppo irregolare in ({sx:.2f}, {sy:.2f})"
    return (f"[FRONTE] {base} -- tratto troppo ripido da ({sx:.2f}, {sy:.2f}) "
            f"(pendenza {frontier['value']:.3f} > {spotGrid.SLOPE_THRESHOLD:.3f})")


# =============================================================================
# Verificatore in background: tiene aggiornato il fronte mentre il robot cammina.
# =============================================================================
class ArcVerificationTracker:
    """
    Thread in background che ricalcola il fronte sicuro sul percorso corrente a ogni
    scansione (~5 Hz), con dati propri e freschi.

    Serve DURANTE il movimento: il ciclo principale lo interroga per fermare il robot se il
    fronte si accorcia sotto il punto verso cui sta andando. Prima di partire il ciclo
    principale calcola il fronte da se', sulla propria scansione.

    Il risultato porta la versione del percorso su cui e' stato calcolato: dopo una
    ripianificazione un fronte vecchio non puo' essere scambiato per uno nuovo.
    """

    def __init__(self, robot):
        self.robot = robot
        self._robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._local_grid = spotGrid.LocalGrid(self.robot)
        self._lock = threading.Lock()
        self._waypoints = []          # [(x, y), ...] SENZA la posizione del robot
        self._extend_end_m = None
        self._body = None
        self._path_version = 0
        self._latest = None           # ultimo fronte calcolato
        self._running = False
        self._thread = None

    # --- interfaccia per il ciclo principale -------------------------------------------
    def update_path(self, waypoints_xy, extend_end_m=None, body=None):
        """
        Imposta il percorso da seguire (solo i waypoint). Restituisce la nuova versione.
        body: ingombro del corpo rispetto alla marcia (vedi compute_safe_frontier), da passare
        quando il robot cammina senza girarsi.
        """
        with self._lock:
            self._waypoints = [(float(x), float(y)) for x, y in waypoints_xy]
            self._extend_end_m = extend_end_m
            self._body = body
            self._path_version += 1
            self._latest = None
            return self._path_version

    def clear_path(self):
        self.update_path([])

    def get_frontier(self):
        """Copia dell'ultimo fronte (dict) con 'path_version', oppure None."""
        with self._lock:
            return None if self._latest is None else dict(self._latest)

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._process_loop, daemon=True)
        self._thread.start()
        print("ArcVerificationTracker (fronte sicuro) avviato in background.")

    def stop(self):
        self._running = False
        print('ArcVerificationTracker stopped.')

    # --- ciclo in background -------------------------------------------------------------
    def _process_loop(self):
        while self._running:
            with self._lock:
                waypoints = list(self._waypoints)
                extend = self._extend_end_m
                body = self._body
                version = self._path_version
            if not waypoints:
                time.sleep(0.2)
                continue
            try:
                snap = self._fresh_snapshot()
                if snap is None:
                    time.sleep(0.2)
                    continue
                # Questo fronte serve SOLO all'arresto a meta' movimento: soglia d'intrusione
                # (mezza larghezza, senza aria), vedi spotGrid.FRONTIER_ABORT_AIR_M.
                frontier = compute_safe_frontier([(snap.robot_x, snap.robot_y)] + waypoints, snap,
                                                 extend_end_m=extend, body=body,
                                                 air_m=spotGrid.FRONTIER_ABORT_AIR_M)
                frontier['path_version'] = version
                with self._lock:
                    if version == self._path_version:   # nel frattempo non e' cambiato il percorso
                        self._latest = frontier
            except Exception as e:
                LOGGER.error(f"Errore nel calcolo del fronte: {e}")
            time.sleep(0.2)

    def _fresh_snapshot(self):
        grids_data, main_proto, _ = self._local_grid.return_local_grid(
            ['obstacle_distance', 'terrain', 'terrain_valid'],
            robot_state_client=self._robot_state_client)
        if grids_data is None or 'obstacle_distance' not in grids_data:
            return None

        pts = grids_data['obstacle_distance']['pts']
        cells_obs = grids_data['obstacle_distance']['values']
        terrain_vals = grids_data['terrain']['values']
        valid_vals = grids_data['terrain_valid']['values']
        num_x = main_proto.local_grid.extent.num_cells_x
        num_y = main_proto.local_grid.extent.num_cells_y
        cell_size = main_proto.local_grid.extent.cell_size

        # Posa del robot NELLO STESSO istante della griglia (stessa fotografia di trasformate)
        transforms_snapshot = main_proto.local_grid.transforms_snapshot
        vision_tform_body = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME)
        robot_x, robot_y = vision_tform_body.position.x, vision_tform_body.position.y
        q = vision_tform_body.rotation
        robot_yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        footprint_mask_2d = self._local_grid.compute_robot_footprint_mask(
            pts, robot_x, robot_y, robot_yaw, num_x, num_y)
        terrain_corrected, is_valid, unwritten = self._local_grid.correct_terrain(
            terrain_vals, valid_vals, num_x, num_y,
            robot_footprint_mask=footprint_mask_2d, cell_size=cell_size,
            unwritten_value=grids_data['terrain'].get('raw_zero'),
            unwritten_scale=grids_data['terrain'].get('scale'), return_unwritten=True)
        _, rough_vals = self._local_grid.compute_gradient_and_roughness(
            terrain_corrected, valid_vals, num_x, num_y, cell_size, is_valid=is_valid)

        grid_frame_name = main_proto.local_grid.frame_name_local_grid_data
        vision_tform_grid = get_a_tform_b(transforms_snapshot, VISION_FRAME_NAME, grid_frame_name)
        return make_grid_snapshot(cells_obs, rough_vals, is_valid, valid_vals, terrain_corrected,
                                  num_x, num_y, vision_tform_grid.position.x,
                                  vision_tform_grid.position.y, cell_size, robot_x, robot_y,
                                  unwritten=unwritten, robot_yaw=robot_yaw)


def is_arc_in_fov(x1, y1, x2, y2, pts, fov_margin=0.0):
    """
    True se l'arco sta TUTTO dentro la finestra della griglia locale (~3.8 m). Non e' il
    campo visivo delle telecamere. Usata dalla ritirata.
    """
    if len(pts) == 0:
        return False

    x_min, x_max = pts[:, 0].min() - fov_margin, pts[:, 0].max() + fov_margin
    y_min, y_max = pts[:, 1].min() - fov_margin, pts[:, 1].max() + fov_margin

    p1_in_fov = (x_min <= x1 <= x_max) and (y_min <= y1 <= y_max)
    p2_in_fov = (x_min <= x2 <= x_max) and (y_min <= y2 <= y_max)
    mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
    mid_in_fov = (x_min <= mid_x <= x_max) and (y_min <= mid_y <= y_max)

    return p1_in_fov and p2_in_fov and mid_in_fov


def verify_arc_safety(x1, y1, x2, y2, pts, obstacle_mask):
    """Controllo puntuale di un segmento sulla maschera fusa. Usato dalla ritirata."""
    return spotUtils.check_line_of_sight(x1, y1, x2, y2, pts, obstacle_mask)


# =========================================================================================
# LE DECISIONI SUL FRONTE: cosa fare adesso dato il fronte sicuro e la scansione fresca.
# Funzioni pure (niente robot, niente I/O), provate dal test end-to-end e dal rigioco.
# =========================================================================================
def goal_still_reachable(snap, goal_x, goal_y):
    """
    Il punto obiettivo e' ancora raggiungibile sui dati FRESCHI?

    Il fronte sicuro pretende che la linea centrale del percorso stia ad almeno
    FRONTIER_CLEARANCE_M da un ostacolo. Un punto obiettivo con obstacle_distance sotto
    quella soglia non potra' MAI essere raggiunto, per quante scansioni si facciano: il
    robot aspetterebbe e dichiarerebbe un blocco. Succede perche' il punto si sceglie da
    lontano, quando la zona e' ancora ignota.

    Returns:
        True se il punto e' raggiungibile o se non lo si puo' giudicare (fuori dalla
        finestra scansionata: non sappiamo, e non si tocca niente sulla base di nulla).
    """
    if snap is None:
        return True
    rows, cols, inside = _cells(snap, np.array([float(goal_x)]), np.array([float(goal_y)]))
    if not bool(inside[0]):
        return True
    return float(snap.obstacle_dist[rows[0], cols[0]]) >= spotGrid.FRONTIER_CLEARANCE_M


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
      3. spotGrid.rotation_is_clear() sulle celle davvero spazzate (0.26 ms)

    Senza `snap` (nessuna fotografia della griglia) si ricade sul solo confronto scalare.

    Funzione pura, provabile fuori dal campo. Restituisce dict con:
      turn (bool), dyaw_deg, motion_dir (rad), margin (margine del settore spazzato, in
      metri, o None se non calcolato).
    """
    rx, ry = robot_xy
    tx, ty = target_xy
    motion_dir = float(np.arctan2(ty - ry, tx - rx))
    dyaw = float(np.degrees(np.arctan2(np.sin(motion_dir - robot_yaw), np.cos(motion_dir - robot_yaw))))

    if abs(dyaw) < spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'rotazione trascurabile'}
    if od_robot >= spotGrid.ROTATION_CLEARANCE_M:
        return {'turn': True, 'dyaw_deg': dyaw, 'motion_dir': motion_dir, 'margin': None,
                'why': 'cerchio circoscritto libero'}
    if snap is not None:
        ok, margin = spotGrid.rotation_is_clear(
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
                                          safety_margin=spotGrid.PRM_EDGE_SAFETY_MARGIN_M)
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
    body = spotGrid.body_extents(robot_yaw, robot_yaw + np.pi)   # all'indietro: (0.55, 0.25)
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
      - se il robot puo' girarsi tanto da puntarci il muso (spotGrid.rotation_is_clear).
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
        if abs(rel) < spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
            rotatable = True
        else:
            rotatable = bool(spotGrid.rotation_is_clear(
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
    return any(d['usable'] for d in dirs), dirs


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
        if abs(rel) >= spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
            ok, _ = spotGrid.rotation_is_clear(
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
    if abs(dyaw_deg) < spotGrid.ROTATION_CHECK_MIN_DYAW_DEG:
        return True, None
    ok, margin = spotGrid.rotation_is_clear(
        snap.obstacle_dist, snap.origin_x, snap.origin_y, snap.cell_size,
        float(back_xy[0]), float(back_xy[1]), float(robot_yaw), float(np.radians(dyaw_deg)))
    return bool(ok), margin


def prune_prm_nodes_in_obstacles(prm, snap, min_clearance_m=P.PRM_NODE_MIN_CLEARANCE_M,
                                 global_map=None):
    """
    Scarta dal grafo i nodi che cadono DENTRO un ostacolo.

    Gli archi che attraversano celle occupate sono vetati (_occupancy_blocked), ma i nodi
    nascono dal reticolo prima che il robot abbia visto qualcosa: un nodo dentro un
    ostacolo resterebbe destinazione o punto di passaggio, da scoprire solo sul posto.

    Soglia min_clearance_m = FRONTIER_CLEARANCE_M (0.30): sotto quella il centro del corpo
    non ci sta e il fronte non ci arrivera' mai, quindi il nodo e' inutile sia come
    destinazione sia come punto di passaggio.

    Si scarta SOLO dove la cella e' stata davvero osservata: ignoto non e' occupato (e' la
    stessa regola per cui il fronte distingue 'fine_dati' da 'ostacolo'). Un nodo fuori
    dalla finestra o mai visto resta dov'e' e verra' giudicato quando lo si vedra'.

    Si rivaluta a ogni ciclo, quindi un nodo che si scopre dentro un muro cade subito; e
    siccome gli archi vengono tolti in modo NON permanente, un nodo giudicato male da una
    lettura rumorosa torna in gioco quando i dati freschi lo smentiscono.

    La finestra della scansione contiene solo una piccola parte del grafo (~7%), e lo
    scarto e' soft: al refresh successivo gli archi tornano. Con `global_map` (e
    spotGrid.CLEARANCE_MAP_DECIDES) si guarda ANCHE la mappa dello spazio libero, che copre
    tutto il grafo e ricorda, cosi' il giudizio su un nodo lontano non va perso a ogni giro.
    """
    ids = [nid for nid in prm.nodes if prm.edges.get(nid)]   # gia' isolati: niente da togliere
    if not ids:
        return []
    xy = np.array([prm.nodes[nid] for nid in ids], dtype=np.float64)
    rows, cols, inside = _cells(snap, xy[:, 0], xy[:, 1])
    bad = np.zeros(len(ids), dtype=bool)
    if np.any(inside):
        r, c = rows[inside], cols[inside]
        bad[inside] = snap.seen[r, c] & (snap.obstacle_dist[r, c] < min_clearance_m)
    if (spotGrid.CLEARANCE_MAP_DECIDES and global_map is not None
            and hasattr(global_map, 'clearance_many')):
        cl = global_map.clearance_many(xy[:, 0], xy[:, 1])
        bad |= np.isfinite(cl) & (cl < min_clearance_m)
    pruned = []
    for k in np.nonzero(bad)[0]:
        nid = ids[int(k)]
        for nb in [n for n, _ in prm.edges.get(nid, [])]:
            prm_graph.mark_edge_blocked_soft(prm, nid, nb)
        pruned.append(nid)
    return pruned
