"""Mappe globali accumulate per tutta la missione: occupazione, spazio libero, terreno."""
import numpy as np
from scipy.ndimage import binary_dilation
import pickle
from spot_explorer.perception.geometry import ROBOT_HALF_WIDTH_M
from spot_explorer.perception.slopes import (LATERAL_SLICE_HALF_WIDTH_M, MIN_ARC_SAMPLE_FRACTION, _no_slope_data, _slope_profile_from_sampler, arc_slope_profiles_batch)
from spot_explorer.perception.spot_grid import slope_usable_mask

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
# Per ogni cella ricorda QUANTO spazio c'era (obstacle_distance), non solo "ostacolo si/no".
# La usa la scelta dell'obiettivo della cella (easy_walk.find_best_point_in_cell); archi e
# nodi del grafo si giudicano sulla mappa binaria. Statistica: media delle osservazioni prese
# entro CLEARANCE_NEAR_M dal robot; dove non ce ne sono, media di tutte.
# =========================================================================
CLEARANCE_NEAR_M = 1.0


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
        self.global_terrain_map = None  # GlobalTerrainGrid, lazy-init altrove
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


class GlobalTerrainGrid:
    """
    Mappa globale e persistente della QUOTA del terreno (media delle osservazioni per
    cella), non della pendenza: la pendenza dipende da come si percorre un arco e si
    ricalcola al volo (compute_arc_slope_profile). Stessa risoluzione della griglia locale;
    se in missioni lunghe la memoria diventasse un problema, la leva e' `resolution`.

    Si alimenta solo con quote corrette (LocalGrid.correct_terrain) e con la stessa maschera
    del calcolo dal vivo (slope_usable_mask: celle attendibili e non ostacolo),
    tramite fold_scan_into_global_maps.

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
        Come slopes.compute_arc_slope_profile, ma sulla mappa globale: serve per gli archi
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
    conferme (vedi GLOBAL_OCC_CONFIRM_WALKED).
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
