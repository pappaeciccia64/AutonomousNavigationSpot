"""Pendenza degli archi: profili longitudinale e laterale su una base di 0.48 m, dal vivo e dalla mappa del terreno."""
import numpy as np
from spot_explorer.perception.geometry import ROBOT_HALF_LENGTH_M
SLOPE_THRESHOLD = 0.6       # gradient (tan of slope): tan(28deg)=~0.53, tan(25deg)=~0.46
FRONTIER_SLOPE_MIN_LENGTH_M = 0.50   # sotto questa lunghezza la pendenza non e' stimabile

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
