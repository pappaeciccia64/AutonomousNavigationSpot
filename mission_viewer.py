#!/usr/bin/env python3
"""
Visualizzatore dei dati di missione. Gira su qualunque computer: servono solo
numpy e matplotlib, NON l'SDK di Spot.

Durante la missione il robot non disegna figure (vedi visualize_grid_with_candidates in
easy_walk.py): salva i dati grezzi, e le figure si fanno qui, dopo.

    python3 mission_viewer.py                                  # (o tasto Run) l'ultima missione in MissionMap/
    python3 mission_viewer.py <cartella_missione>              # interattivo, tutte le scansioni
    python3 mission_viewer.py 13-10-2026_12-40-38              # oppure solo il nome della missione
    python3 mission_viewer.py <file.npz>                       # una scansione o una vista PRM
    python3 mission_viewer.py <cartella_missione> --png <dir>  # esporta tutte le figure in PNG
    python3 mission_viewer.py <cartella_missione> --overview   # solo il riepilogo della missione
    ... --sfondo terreno | fusione                             # sfondo iniziale del pannello globale

Tasti (modalita' interattiva):
    A sinistra la scansione (layer a scelta), a destra la MAPPA GLOBALE in quel momento:
    mosaico delle scansioni fatte fin li' (rosso ostacolo, verde chiaro libero, bianco mai visto),
    celle di missione (verde visitata, rossa bloccata), grafo PRM colorato per pendenza
    (verde in avanti, rosso di traverso), percorso reale, percorso pianificato, finestra
    attuale (azzurro), blocchi (quadrati rossi).
    freccia destra / sinistra   scansione successiva / precedente
    freccia su / giu'            10 scansioni avanti / indietro
    1..9, 0                      layer del pannello sinistro (vedi LAYERS)
    o                            pannello destro: fine missione <-> momento della scansione
    g                            grafo PRM sopra la scansione: mostra / nascondi
    m                            pannello destro: mappa del robot (ostacoli confermati, default)
                                 <-> mosaico grezzo (l'ultima scansione vince)
    t                            sfondo del pannello destro: ostacoli -> terreno -> fusione
                                 (terreno colorato per quota, ostacoli in rosso sopra)
    il valore sotto il mouse compare in basso a destra, con le coordinate VISION

File letti:
    scans/scan_*.npz    un pacchetto per scansione del ciclo di navigazione
    *_vis.npz           al posto delle vecchie figure: dati + grafo PRM in quel momento
    *_BLOCKED_*.npz     i dati che hanno deciso un blocco
"""
import glob
import os
import pickle
import sys

import threading
import numpy as np

PNG_MODE = "--png" in sys.argv
if PNG_MODE:
    import matplotlib
    matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.collections import LineCollection    # noqa: E402
from matplotlib.colors import ListedColormap, Normalize   # noqa: E402
import matplotlib.patches as patches                  # noqa: E402
import csv                                            # noqa: E402
import matplotlib.patheffects as pe                   # noqa: E402

SLOPE_THRESHOLD = 0.6   # come spotGrid.SLOPE_THRESHOLD: serve solo a colorare gli archi

# MAPPA GLOBALE COME LA COSTRUIVA IL ROBOT (2026-10-26). Copia di easy_walk._mask_for_global_map
# e spotGrid.GlobalGrid.update, che qui non si possono importare (servono l'SDK di Spot).
# Prima il pannello di destra mostrava il mosaico "l'ultima scansione vince": un ostacolo
# letto UNA volta, anche accanto alle zampe, ricopriva le celle su cui il robot era appena
# passato, e il percorso reale sembrava attraversare gli ostacoli, soprattutto alla fine
# (missione del 22-10, 16:00 circa). Il robot quelle letture le scartava o le pretendeva
# confermate. Il tasto 'm' torna al mosaico grezzo.
SELF_BODY_LENGTH_M = 1.10      # easy_walk.GLOBAL_MAP_SELF_BODY_LENGTH_M
SELF_BODY_WIDTH_M = 0.50       # easy_walk.GLOBAL_MAP_SELF_BODY_WIDTH_M
SELF_RING_M = 0.25             # easy_walk.GLOBAL_MAP_SELF_RING_M
OCC_CONFIRM_OBS = 2            # spotGrid.GLOBAL_OCC_CONFIRM_OBS
OCC_CONFIRM_WALKED = 4         # spotGrid.GLOBAL_OCC_CONFIRM_WALKED
OCC_MAX_COUNT = 4              # spotGrid.GLOBAL_OCC_MAX_COUNT

# Due campioni consecutivi di trajectory.csv piu' lontani di cosi': in mezzo c'e' un
# movimento non registrato (fino al 2026-10-26 GraphNav, cono e arretramenti non scrivevano
# il file). Si disegna tratteggiato, non come percorso reale.
TRAJ_GAP_M = 0.5

# SFONDO DEL PANNELLO GLOBALE (2026-10-26), tasto 't'.
#   ostacoli -- la mappa di occupazione (rosso ostacolo, verde chiaro libero, bianco mai visto)
#   terreno  -- la mappa globale della QUOTA, come la teneva il robot (media delle quote
#               corrette, solo celle attendibili e non ostacolo: vedi easy_walk.slope_usable_mask),
#               rispetto al suolo all'avvio della missione, con ombreggiatura del rilievo
#   fusione  -- il terreno, e sopra in rosso gli ostacoli della mappa di occupazione
# La scala dei colori della quota e' fissa per tutta la missione, cosi' scorrendo le
# scansioni lo stesso colore vuol dire sempre la stessa quota.
BACKGROUNDS = ('ostacoli', 'terreno', 'fusione')
TERRAIN_CMAP = 'viridis'                # niente rossi: nella fusione il rosso e' solo degli ostacoli
OBSTACLE_RGB = (0.84, 0.15, 0.16)       # come MOSAIC_CMAP
NO_TERRAIN_RGB = (0.80, 0.80, 0.80)     # fusione: cella vista libera ma senza quota attendibile
OBSTACLE_THRESHOLD = 0.15               # spotGrid.OBSTACLE_THRESHOLD (maschera del terreno)
TERRAIN_PERCENTILES = (2, 98)           # estremi della scala dei colori della quota
HILLSHADE_STRENGTH = 0.45               # 0 = niente ombreggiatura del rilievo

# Le mappe si costruiscono scansione per scansione; tornando indietro si riparte dall'ultima
# copia salvata invece che dalla prima scansione (343 scansioni nella missione del 22-10 16:04).
CHECKPOINT_EVERY = 25

REASON_COLORS = {'fine_percorso': 'tab:green', 'fine_dati': 'tab:blue', 'ostacolo': 'tab:red',
                 'rugosita': 'tab:orange', 'pendenza': 'tab:purple'}

# nome, funzione che estrae la matrice, mappa colori, vmin, vmax, etichetta
LAYERS = [
    ("quota rispetto al suolo", lambda d: d['terrain_raw'] - _ground(d), 'terrain', -0.5, 0.5,
     "quota grezza - suolo stimato (m)"),
    ("terrain_valid grezzo", lambda d: d['terrain_valid_raw'], 'gray_r', 0, 1, "1 = valido"),
    ("obstacle_distance", lambda d: d['obstacle_distance'], 'plasma', 0, 1.0, "distanza dall'ostacolo (m)"),
    ("rugosita'", lambda d: d['roughness'], 'coolwarm', 0, 0.3, "deviazione standard (m)"),
    ("gradiente", lambda d: d['gradient'], 'viridis', 0, 1.0, "tan(pendenza)"),
    ("maschera fusa", lambda d: d['obstacle_mask'], 'RdYlGn', -1, 1, "-1 = ostacolo"),
    ("celle usate (is_valid)", lambda d: d['is_valid'].astype(float), 'gray_r', 0, 1,
     "0 = riempita/scartata"),
    ("quota corretta", lambda d: d['terrain_corrected'] - _ground(d), 'terrain', -0.5, 0.5,
     "quota corretta - suolo (m)"),
    ("quota rispetto all'avvio missione", lambda d: d['terrain_corrected'] - _mission_z0(d), 'terrain',
     -2.0, 2.0, "quota corretta - suolo all'avvio missione (m)"),
    ("celle mai scritte", lambda d: d['unwritten'].astype(float) if 'unwritten' in d
     else np.zeros_like(d['terrain_raw']), 'gray_r', 0, 1, "1 = mai scritta: trattata come non vista"),
]


def _ground(d):
    g = float(d['ground_z']) if 'ground_z' in d else np.nan
    if not np.isfinite(g):
        t = d['terrain_raw']
        g = float(np.median(t[np.isfinite(t)]))
    return g


def _mission_z0(d):
    """Suolo all'avvio missione; per i file vecchi (senza il campo) il suolo della scansione."""
    z0 = float(d['mission_z0']) if 'mission_z0' in d else np.nan
    return z0 if np.isfinite(z0) else _ground(d)


def _load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def _extent(d):
    ny, nx = d['terrain_raw'].shape
    ox, oy = d['grid_origin']
    cs = float(d['cell_size'])
    return [ox, ox + nx * cs, oy, oy + ny * cs]


def _hover(ax, mat, extent):
    ny, nx = mat.shape
    x0, x1, y0, y1 = extent
    cs = (x1 - x0) / nx

    def fmt(x, y):
        c, r = int((x - x0) / cs), int((y - y0) / cs)
        if 0 <= r < ny and 0 <= c < nx:
            return f"x={x:.2f} y={y:.2f} | riga {r} col {c} | valore {mat[r, c]:.4f}"
        return f"x={x:.2f} y={y:.2f}"
    ax.format_coord = fmt


def draw_scan(ax, d, layer_idx, title_prefix=""):
    ax.clear()
    name, fn, cmap, vmin, vmax, label = LAYERS[layer_idx]
    mat = np.asarray(fn(d), dtype=float)
    ext = _extent(d)
    im = ax.imshow(mat, origin='lower', extent=ext, cmap=cmap, vmin=vmin, vmax=vmax,
                   interpolation='nearest')
    _hover(ax, mat, ext)

    # contorno della distanza minima dal centro del percorso
    od = d['obstacle_distance']
    clear = float(d.get('frontier_clearance', 0.30))
    xs = np.linspace(ext[0], ext[1], od.shape[1]); ys = np.linspace(ext[2], ext[3], od.shape[0])
    ax.contour(xs, ys, od, levels=[clear], colors='white', linewidths=0.8, linestyles='--')

    pl = d['polyline']
    ax.plot(pl[:, 0], pl[:, 1], '-', color='magenta', lw=2, label='percorso')
    rx, ry = d['robot_xyz'][:2]
    yaw = float(d['robot_yaw'])
    ax.plot(rx, ry, 'o', color='blue', ms=9)
    ax.arrow(rx, ry, 0.35 * np.cos(yaw), 0.35 * np.sin(yaw), color='blue', width=0.02)
    reason = str(d['frontier_reason'])
    sx, sy = d['frontier_stop_xy']
    ax.plot(sx, sy, 'X', color=REASON_COLORS.get(reason, 'k'), ms=13, mec='white',
            label=f"fronte: {reason}")
    tx, ty = d['decision_target']
    if np.isfinite(tx):
        ax.plot(tx, ty, '*', color='lime', ms=14, mec='k', label='punto comandato')
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3]); ax.set_aspect('equal')
    ax.set_xlabel("X [m] (VISION)"); ax.set_ylabel("Y [m] (VISION)")
    ax.set_title(f"{title_prefix}{name}\n"
                 f"fronte {float(d['frontier_dist']):.2f} m ({reason}), avanzamento "
                 f"{float(d['allowed_advance']):.2f} m, decisione: {str(d['decision_kind'])}"
                 + (" [senza ruotare]" if bool(d.get('keep_heading', False)) else ""), fontsize=9)
    ax.legend(loc='upper left', fontsize=7, framealpha=0.8)
    return im, label


def draw_vis_bundle(ax, d):
    """Le vecchie figure 'Path Visualization': scansione, grafo PRM, percorso, celle."""
    ax.clear()
    pts = d['pts']
    if pts.size:
        mask = d['obstacle_mask']
        ax.scatter(pts[:, 0], pts[:, 1], c=np.where(mask < 0, 1.0, 0.0), cmap='RdYlGn_r', s=1,
                   vmin=0, vmax=1, alpha=0.5)
    xy = {int(i): tuple(p) for i, p in zip(d['prm_node_ids'], d['prm_node_xy'])}
    segs = [(xy[a], xy[b]) for a, b in d['prm_edges'] if a in xy and b in xy]
    if segs:
        ax.add_collection(LineCollection(segs, colors='gray', linewidths=0.3, alpha=0.4))
    trav = [(xy[a], xy[b]) for a, b in d['prm_traversed_edges'] if a in xy and b in xy]
    if trav:
        ax.add_collection(LineCollection(trav, colors='tab:blue', linewidths=2))
    # nodi di sosta: solo nelle missioni fino al 08-10, prima che il grafo restasse fisso
    stops = np.array([xy[int(i)] for i in _arr(d, 'prm_stop_nodes', ()) if int(i) in xy]).reshape(-1, 2)
    if len(stops):
        ax.plot(stops[:, 0], stops[:, 1], 'o', color='tab:orange', ms=5, label='nodi di sosta')
    cp = d['chosen_path']
    if len(cp):
        ax.plot(cp[:, 0], cp[:, 1], '-', color='magenta', lw=2.5, label='percorso scelto')
    ax.plot(*d['robot_xy'], 'o', color='blue', ms=10, label='robot')
    if np.all(np.isfinite(d['chosen_point'])):
        ax.plot(*d['chosen_point'], '*', color='green', ms=16, label='obiettivo')
    for (r, c, x, y), st in zip(d['env_cells'], d['env_cell_status']):
        ax.text(x, y, f"{int(r)},{int(c)}", fontsize=7, ha='center')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.autoscale_view()
    ax.set_title(f"Iterazione {int(d['iteration'])}: {len(segs)} archi PRM, {len(stops)} nodi di sosta",
                 fontsize=10)
    ax.legend(fontsize=7)



# =============================================================================
# VISTA GLOBALE.
# Tutto e' ricostruito "com'era in quel momento": la mappa e' il mosaico delle scansioni
# fatte FINO alla scansione mostrata, il grafo e' l'ultima vista PRM salvata prima.
# =============================================================================
MOSAIC_CMAP = ListedColormap(['#ffffff', '#dfe8d8', '#d62728'])   # ignoto, libero, ostacolo


class MissionContext:
    """Dati di tutta la missione, caricati una volta: scansioni, viste PRM, traiettoria."""

    def __init__(self, folder, scans, scan_files, vis_files):
        self.scans, self.scan_files = scans, scan_files
        self.vis = []
        for f in vis_files:
            try:
                self.vis.append((os.path.getmtime(f), os.path.basename(f), _load(f)))
            except Exception as e:
                print(f"[viewer] vista PRM non leggibile {os.path.basename(f)}: {e}")
        self.vis.sort(key=lambda t: t[0])
        self.traj = self._load_trajectory(folder)
        occ = self._load_pkl_cells(folder, "global_occupancy_map.pkl", 'grid')
        ter = self._load_pkl_cells(folder, "global_terrain_map.pkl", 'height_count')
        self.final_map = self.final_terrain = None
        self._cache = {}
        self._terrain_norm = None
        if not scans:
            return
        # Griglia comune: tutte le scansioni piu' le mappe salvate a fine missione (che
        # contengono anche le scansioni d'inizio tentativo). L'origine e' un multiplo della
        # risoluzione, cosi' le celle coincidono con quelle delle mappe del robot (floor(x/res)).
        self.res = float(scans[0]['cell_size'])
        ext = np.array([_extent(d) for d in scans])
        xmin, xmax, ymin, ymax = ext[:, 0].min(), ext[:, 1].max(), ext[:, 2].min(), ext[:, 3].max()
        for cells in (occ, ter):
            if cells is not None:
                k, res = cells[0], cells[2]
                xmin, xmax = min(xmin, k[:, 0].min() * res), max(xmax, (k[:, 0].max() + 1) * res)
                ymin, ymax = min(ymin, k[:, 1].min() * res), max(ymax, (k[:, 1].max() + 1) * res)
        pad = 1.0
        self.kx0 = int(np.floor((xmin - pad) / self.res))
        self.ky0 = int(np.floor((ymin - pad) / self.res))
        self.x0, self.y0 = self.kx0 * self.res, self.ky0 * self.res
        self.nx = int(np.ceil((xmax + pad - self.x0) / self.res))
        self.ny = int(np.ceil((ymax + pad - self.y0) / self.res))
        # Suolo all'avvio della missione: riferimento delle quote.
        self.z0 = np.nan
        if ter is not None and ter[3] is not None:
            self.z0 = float(ter[3])
        if not np.isfinite(self.z0):
            self.z0 = _mission_z0(scans[0])
        if occ is not None:
            self.final_map = self._raster(occ[0], np.where(occ[1] < 0, 2, 1), np.uint8, 0)
        if ter is not None:
            self.final_terrain = self._raster(ter[0], ter[1], np.float32, np.nan) - self.z0

    @staticmethod
    def _load_pkl_cells(folder, name, key):
        """
        Celle di una mappa globale salvata a fine missione (.pkl di easy_walk): (chiavi (N, 2)
        come (gx, gy), valori, risoluzione, mission_z0). Per l'occupazione i valori sono
        -1 ostacolo / +1 libero; per il terreno la quota media assoluta. None se manca.
        """
        path = os.path.join(folder, name)
        if not os.path.exists(path):
            return None
        try:
            with open(path, 'rb') as f:
                g = pickle.load(f)
            cells = g[key]
            if not cells:
                return None
            keys = np.array(list(cells.keys()), dtype=np.int64).reshape(-1, 2)
            if key == 'grid':
                vals = np.array(list(cells.values()), dtype=float)
            else:
                hs = g['height_sum']
                vals = np.array([hs[k] / n for k, n in cells.items()], dtype=float)
            return keys, vals, float(g['resolution']), g.get('mission_z0')
        except Exception as e:
            print(f"[viewer] mappa globale non leggibile ({path}): {e}")
            return None

    def _raster(self, keys, vals, dtype, fill):
        """Celle (gx, gy) -> matrice sulla griglia comune."""
        img = np.full((self.ny, self.nx), fill, dtype=dtype)
        c, r = keys[:, 0] - self.kx0, keys[:, 1] - self.ky0
        ok = (c >= 0) & (c < self.nx) & (r >= 0) & (r < self.ny)
        img[r[ok], c[ok]] = np.asarray(vals)[ok]
        return img

    @property
    def extent(self):
        return [self.x0, self.x0 + self.nx * self.res, self.y0, self.y0 + self.ny * self.res]

    @staticmethod
    def _load_trajectory(folder):
        """trajectory.csv in MissionLogs/<stessa missione>, se c'e' (le missioni piu' vecchie non l'hanno)."""
        name = os.path.basename(os.path.normpath(folder))
        for cand in (os.path.join(os.path.dirname(os.path.dirname(os.path.normpath(folder))), "MissionLogs", name,
                                  "trajectory.csv"),
                     os.path.join(folder, "trajectory.csv")):
            if os.path.exists(cand):
                try:
                    with open(cand, newline='', encoding='utf-8') as f:
                        rows = list(csv.DictReader(f))
                    # quarta colonna: 1 se il campione viene da uno spostamento GraphNav
                    # (colonna 'move', dal 2026-10-26), altrimenti 0
                    return np.array([(float(r['time']), float(r['x']), float(r['y']),
                                      float(r.get('move') == 'graphnav')) for r in rows]).reshape(-1, 4)
                except Exception as e:
                    print(f"[viewer] trajectory.csv non leggibile: {e}")
        return None

    def _window(self, d):
        """Slice della griglia comune e della scansione d per la parte che si sovrappone."""
        gox, goy = d['grid_origin']
        r0 = int(np.floor((goy - self.y0) / self.res))
        c0 = int(np.floor((gox - self.x0) / self.res))
        h, w = d['terrain_raw'].shape
        rs, cs_ = slice(max(r0, 0), min(r0 + h, self.ny)), slice(max(c0, 0), min(c0 + w, self.nx))
        return rs, cs_, slice(rs.start - r0, rs.stop - r0), slice(cs_.start - c0, cs_.stop - c0)

    @staticmethod
    def _robot_mask(d):
        """
        Come easy_walk._mask_for_global_map: -1 ostacolo, +1 libero, 0 nessuna informazione;
        sotto il corpo libero, nell'anello attorno nessun ostacolo. Anche la maschera del corpo.
        """
        om = np.asarray(d['obstacle_mask']).reshape(d['terrain_raw'].shape)
        out = np.where(om < 0, -1, np.where(np.asarray(d['is_valid'], bool), 1, 0)).astype(np.int8)
        core = np.zeros(om.shape, bool)
        if 'robot_yaw' not in d:
            return out, core
        h, w = om.shape
        cs = float(d['cell_size'])
        gox, goy = d['grid_origin']
        rx, ry = d['robot_xyz'][:2]
        yaw = float(d['robot_yaw'])
        X, Y = np.meshgrid(gox + (np.arange(w) + 0.5) * cs - rx, goy + (np.arange(h) + 0.5) * cs - ry)
        xb = np.abs(X * np.cos(yaw) + Y * np.sin(yaw))
        yb = np.abs(-X * np.sin(yaw) + Y * np.cos(yaw))
        hl, hw = SELF_BODY_LENGTH_M / 2.0, SELF_BODY_WIDTH_M / 2.0
        core = (xb <= hl) & (yb <= hw)
        ring = ~core & (xb <= hl + SELF_RING_M) & (yb <= hw + SELF_RING_M)
        out[ring & (out < 0)] = 0
        out[core] = 1
        return out, core

    def _build(self, name, upto, init, step):
        """
        Stato di una mappa dopo le scansioni 0..upto, costruito una scansione alla volta con
        step(stato, scansione). Avanti si continua da dove si era; indietro si riparte dalla
        copia salvata piu' vicina (una ogni CHECKPOINT_EVERY scansioni).
        """
        c = self._cache.setdefault(name, {'upto': -1, 'state': None, 'ckpt': {}})
        if c['state'] is None or upto < c['upto']:
            ks = [k for k in c['ckpt'] if k <= upto]
            if ks:
                c['upto'] = max(ks)
                c['state'] = tuple(a.copy() for a in c['ckpt'][c['upto']])
            else:
                c['upto'], c['state'] = -1, init()
        for i in range(c['upto'] + 1, upto + 1):
            step(c['state'], self.scans[i])
            if (i + 1) % CHECKPOINT_EVERY == 0 and i not in c['ckpt']:
                c['ckpt'][i] = tuple(a.copy() for a in c['state'])
        c['upto'] = upto
        return c['state']

    def _zeros(self, *dtypes):
        return lambda: tuple(np.zeros((self.ny, self.nx), dt) for dt in dtypes)

    def robot_map(self, upto):
        """
        Mappa 0/1/2 (ignoto/libero/ostacolo) delle scansioni 0..upto come la teneva il robot:
        un ostacolo vale solo dopo OCC_CONFIRM_OBS letture consecutive (OCC_CONFIRM_WALKED sulle
        celle su cui il corpo e' passato), e torna libero quando le letture libere lo pareggiano.
        Mancano i dischi delle cadute e la scansione iniziale e finale (non salvate in scans/).
        """
        def step(st_all, d):
            state, count, walked = st_all
            m, core = self._robot_mask(d)
            rs, cs_, sr, sc = self._window(d)
            m, core = m[sr, sc], core[sr, sc]
            st, cnt, wk = state[rs, cs_], count[rs, cs_], walked[rs, cs_]   # viste: scrivono nelle mappe
            wk |= core
            obs, free = m < 0, m > 0
            c_obs = np.minimum(np.where(cnt >= 0, cnt + 1, 1), OCC_MAX_COUNT)
            c_free = np.maximum(cnt - 1, -OCC_MAX_COUNT)
            new = np.where(obs, c_obs, np.where(free, c_free, cnt)).astype(np.int8)
            confirm = np.where(wk, OCC_CONFIRM_WALKED, OCC_CONFIRM_OBS)
            to_obs = obs & (new >= confirm)
            to_free = free & (new <= 0)
            seen_new = (obs | free) & (st == 0) & ~to_obs
            st[to_obs] = 2
            st[to_free | seen_new] = 1
            cnt[...] = new
        return self._build('robot', upto, self._zeros(np.uint8, np.int8, bool), step)[0]

    def mosaic(self, upto):
        """Mosaico 0/1/2 (ignoto/libero/ostacolo) delle scansioni 0..upto: l'ultima vince."""
        def step(st_all, d):
            om = np.asarray(d['obstacle_mask']).reshape(d['terrain_raw'].shape)
            known = np.asarray(d['is_valid'], bool) | (om < 0)
            if 'unwritten' in d:
                known &= ~np.asarray(d['unwritten'], bool) | (om < 0)
            val = np.where(om < 0, 2, 1).astype(np.uint8)
            rs, cs_, sub_r, sub_c = self._window(d)
            tgt = st_all[0][rs, cs_]
            k = known[sub_r, sub_c]
            tgt[k] = val[sub_r, sub_c][k]
        return self._build('raw', upto, self._zeros(np.uint8), step)[0]

    def terrain(self, upto):
        """
        Quota media (m, rispetto al suolo all'avvio missione) delle scansioni 0..upto, come
        spotGrid.GlobalTerrainGrid: quote corrette, solo celle attendibili e non ostacolo
        (easy_walk.slope_usable_mask). NaN dove non c'e' quota.
        """
        def step(st_all, d):
            hsum, hn = st_all
            shape = d['terrain_raw'].shape
            t = np.asarray(d['terrain_corrected'], dtype=np.float64).reshape(shape)
            ok = np.asarray(d['is_valid'], bool).reshape(shape) & np.isfinite(t)
            if 'obstacle_distance' in d:
                ok &= ~(np.asarray(d['obstacle_distance']).reshape(shape) < OBSTACLE_THRESHOLD)
            rs, cs_, sr, sc = self._window(d)
            t, ok = t[sr, sc], ok[sr, sc]
            hsum[rs, cs_][ok] += t[ok]
            hn[rs, cs_][ok] += 1
        hsum, hn = self._build('terrain', upto, self._zeros(np.float64, np.uint32), step)
        with np.errstate(invalid='ignore', divide='ignore'):
            return np.where(hn > 0, hsum / np.maximum(hn, 1), np.nan) - self.z0

    def terrain_norm(self):
        """Scala dei colori della quota, fissa per la missione (vedi TERRAIN_PERCENTILES)."""
        if self._terrain_norm is None:
            h = self.final_terrain if self.final_terrain is not None else self.terrain(len(self.scans) - 1)
            h = h[np.isfinite(h)]
            lo, hi = (np.percentile(h, TERRAIN_PERCENTILES) if h.size else (-0.5, 0.5))
            if hi - lo < 0.05:
                lo, hi = lo - 0.025, hi + 0.025
            self._terrain_norm = Normalize(vmin=float(lo), vmax=float(hi))
        return self._terrain_norm

    def vis_before(self, t):
        """Ultima vista PRM salvata prima dell'istante t (None se nessuna)."""
        best = None
        for mt, name, d in self.vis:
            if t is None or mt <= t + 2.0:
                best = (name, d)
        return best if best is not None else ((self.vis[0][1], self.vis[0][2]) if self.vis else None)


def _scalar(d, key, default):
    """Valore scalare da un npz, robusto a campi mancanti, vuoti o NaN (file vecchi o di prova)."""
    try:
        v = np.asarray(d[key], dtype=float).ravel()
        return float(v[0]) if v.size and np.isfinite(v[0]) else default
    except Exception:
        return default


def _arr(d, key, shape_tail=None):
    """Array da un npz, vuoto se manca (file di versioni precedenti)."""
    if key in d:
        return np.asarray(d[key])
    return np.empty((0,) + tuple(shape_tail or ()))


# La mappa di colori si prende UNA volta: plt.get_cmap(...)(v) ricostruisce la tavola a ogni
# chiamata, e con migliaia di archi era quasi tutto il tempo di un ridisegno.
_SLOPE_CMAP = plt.get_cmap('RdYlGn_r')
_SLOPE_LUT_CACHE = {}   # ripiego quando il bundle non accetta attributi


def _slope_colors_many(keys, long_s, lat_s):
    """
    _slope_color su molti archi in una volta: stessi colori, calcolati con operazioni su
    array invece che uno per uno.
    """
    long_s = np.asarray(long_s, dtype=np.float64)
    lat_s = np.asarray(lat_s, dtype=np.float64)
    total = long_s + lat_s
    frac = np.where(total < 1e-6, 0.5, lat_s / np.where(total < 1e-6, 1.0, total))
    rgba = np.asarray(_SLOPE_CMAP(frac), dtype=np.float64).reshape(-1, 4)
    alpha = 0.3 + 0.5 * np.minimum(1.0, np.maximum(long_s, lat_s) / SLOPE_THRESHOLD)
    rgba[:, 3] = np.where(total < 1e-6, 0.25, alpha)
    rgba[total < 1e-6, :3] = 0.6
    return rgba


# --- ARCHI COME IMMAGINE ------------------------------------------------------------------
# Gli archi si disegnano UNA volta in un'immagine RGBA (dove si sovrappongono la trasparenza
# si somma) e la si mostra con imshow, che costa come la mappa: con decine di migliaia di
# archi, disegnarli come linee rendeva lento ogni ridisegno, compresi trascinamento e zoom.
# L'immagine si tiene in cache per vista PRM ed estensione.
_EDGE_RASTER_CACHE = {}
_EDGE_RASTER_CACHE_MAX = 256
# Lato massimo dell'immagine degli archi, in pixel: oltre, la risoluzione si allarga.
EDGE_RASTER_MAX_PX = 1600
# Opacita' di un arco in un pixel, tarata a occhio perche' l'immagine somigli a linee da
# 0.5 punti.
EDGE_RASTER_COVERAGE = 0.45
EDGE_RASTER_COVERAGE_GLOBAL = 0.22   # nel globale gli archi per pixel sono molti di piu'


def _slope_table(vd):
    """Chiavi (a, b) ordinate come codici interi + valori, per cercare le pendenze con searchsorted."""
    tab = getattr(vd, '_slope_tab', None) if not isinstance(vd, dict) else vd.get('_slope_tab')
    if tab is None:
        tab = _SLOPE_LUT_CACHE.get(('tab', id(vd)))
    if tab is None:
        k = np.asarray(_arr(vd, 'prm_slope_keys', (2,)), dtype=np.int64).reshape(-1, 2)
        v = np.asarray(_arr(vd, 'prm_slope_values', (2,)), dtype=np.float64).reshape(-1, 2)
        k = np.sort(k, axis=1)
        code = k[:, 0] * (1 << 32) + k[:, 1]
        order = np.argsort(code)
        tab = (code[order], v[order])
        try:
            vd._slope_tab = tab
        except Exception:
            _SLOPE_LUT_CACHE[('tab', id(vd))] = tab
    return tab


def _edge_colors(vd, e, grey_rgba, alpha=None):
    """Colore di ogni arco come _slope_color (grigio se non c'e' la pendenza), tutto vettoriale."""
    cols = np.tile(np.asarray(grey_rgba, dtype=np.float64), (len(e), 1))
    codes, vals = _slope_table(vd)
    if len(codes) and len(e):
        ee = np.sort(e, axis=1)
        q = ee[:, 0] * (1 << 32) + ee[:, 1]
        j = np.clip(np.searchsorted(codes, q), 0, len(codes) - 1)
        have = codes[j] == q
        if have.any():
            cols[have] = _slope_colors_many(None, vals[j[have], 0], vals[j[have], 1])
    if alpha is not None:
        cols[:, 3] = alpha
    return cols


def _edges_image(segs, cols, x0, x1, y0, y1, max_px=EDGE_RASTER_MAX_PX, min_res=None,
                 coverage=EDGE_RASTER_COVERAGE):
    """
    Segmenti (N, 2, 2) con colori RGBA (N, 4) -> immagine RGBA (ny, nx, 4) sull'estensione
    data. Ogni arco si campiona a passo di un pixel; in un pixel toccato da k archi il colore
    e' la media e l'opacita' 1 - prod(1 - alpha), come sovrapponendo linee trasparenti.
    """
    w, h = max(x1 - x0, 1e-6), max(y1 - y0, 1e-6)
    res = max(w, h) / float(max_px)
    if min_res is not None:
        res = max(res, float(min_res))
    nx, ny = max(1, int(np.ceil(w / res))), max(1, int(np.ceil(h / res)))
    img = np.zeros((ny, nx, 4), dtype=np.float32)
    if len(segs) == 0:
        return img
    p0, p1 = segs[:, 0, :], segs[:, 1, :]
    L = np.hypot(p1[:, 0] - p0[:, 0], p1[:, 1] - p0[:, 1])
    n = np.maximum(2, np.ceil(L / res).astype(np.int64) + 1)
    idx = np.repeat(np.arange(len(segs)), n)
    start = np.cumsum(n) - n
    t = (np.arange(int(n.sum())) - np.repeat(start, n)) / np.repeat(n - 1, n)
    px = p0[idx, 0] + t * (p1[idx, 0] - p0[idx, 0])
    py = p0[idx, 1] + t * (p1[idx, 1] - p0[idx, 1])
    c = np.floor((px - x0) / res).astype(np.int64)
    r = np.floor((py - y0) / res).astype(np.int64)
    ok = (c >= 0) & (c < nx) & (r >= 0) & (r < ny)
    flat, idx = r[ok] * nx + c[ok], idx[ok]
    # un arco conta una volta per pixel: i suoi campioni sono consecutivi, quindi basta
    # scartare quelli che ricadono nello stesso pixel del precedente (niente ordinamento).
    if len(flat) > 1:
        keep = np.empty(len(flat), dtype=bool)
        keep[0] = True
        keep[1:] = (flat[1:] != flat[:-1]) | (idx[1:] != idx[:-1])
        flat, idx = flat[keep], idx[keep]
    m = nx * ny
    cnt = np.bincount(flat, minlength=m).astype(np.float64)
    hit = cnt > 0
    out = img.reshape(-1, 4)
    for ch in range(3):
        sch = np.bincount(flat, weights=cols[idx, ch], minlength=m)
        out[hit, ch] = (sch[hit] / cnt[hit]).astype(np.float32)
    # coverage: frazione di pixel che una linea sottile (0.5 punti) copre davvero; senza, un
    # pixel dell'immagine "toccato" diventerebbe pieno e il disegno molto piu' scuro di prima.
    logt = np.bincount(flat, weights=np.log1p(-np.clip(coverage * cols[idx, 3], 0.0, 0.999)),
                       minlength=m)
    out[hit, 3] = (1.0 - np.exp(logt[hit])).astype(np.float32)
    return img


def _cached_edges_image(key, build):
    img = _EDGE_RASTER_CACHE.get(key)
    if img is None:
        img = build()
        if len(_EDGE_RASTER_CACHE) >= _EDGE_RASTER_CACHE_MAX:
            _EDGE_RASTER_CACHE.pop(next(iter(_EDGE_RASTER_CACHE)))
        _EDGE_RASTER_CACHE[key] = img
    return img


def _draw_cells(ax, vd):
    """Griglia di missione: verde = visitata, rosso = bloccata, tratteggio = da esplorare."""
    if vd is None or 'env_cells' not in vd or not len(vd['env_cells']):
        return
    size = _scalar(vd, 'env_cell_size', 5.0)
    yaw = _scalar(vd, 'env_origin_yaw', 0.0)
    h = size / 2.0
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    base = np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) @ R.T
    for (r, c, x, y), st in zip(vd['env_cells'], vd['env_cell_status']):
        st = str(st)
        if st == '1':
            kw = dict(edgecolor='darkgreen', facecolor='lightgreen', alpha=0.25, lw=1.2)
        elif st == '-1':
            kw = dict(edgecolor='darkred', facecolor='lightcoral', alpha=0.35, lw=1.2)
        else:
            kw = dict(edgecolor='gray', facecolor='none', alpha=0.6, lw=0.8, ls='--')
        ax.add_patch(patches.Polygon(base + [x, y], zorder=1, **kw))
        ax.text(x, y, f"{int(r)},{int(c)}", ha='center', va='center', fontsize=7, color='0.3', zorder=1)


def overlay_prm_local(ax, ctx, i):
    """Archi e nodi del grafo (ultima vista PRM prima della scansione) sopra la finestra locale."""
    try:
        _overlay_prm_local(ax, ctx, i)
    except Exception as e:
        print(f"[viewer] grafo locale non disegnato: {e}")


def _overlay_prm_local(ax, ctx, i):
    d = ctx.scans[i]
    vis = ctx.vis_before(float(d['time']) if 'time' in d else None)
    if not vis:
        return
    vd = vis[1]
    x0, x1, y0, y1 = _extent(d)
    # Si tengono solo gli archi che toccano la finestra, con operazioni su array: gli archi
    # del PRM sono decine di migliaia e si ricontrollano a ogni ridisegno.
    ids = np.asarray(_arr(vd, 'prm_node_ids', ()), dtype=np.int64).ravel()
    nxy = np.asarray(_arr(vd, 'prm_node_xy', (2,)), dtype=np.float64).reshape(-1, 2)
    edges = np.asarray(_arr(vd, 'prm_edges', (2,)), dtype=np.int64).reshape(-1, 2)
    if len(ids) == 0 or len(nxy) != len(ids):
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
        return
    pos = np.full((int(ids.max()) + 2, 2), np.nan)
    pos[ids] = nxy
    inside_node = np.zeros(len(pos), dtype=bool)
    inside_node[ids] = ((nxy[:, 0] >= x0 - 1) & (nxy[:, 0] <= x1 + 1)
                        & (nxy[:, 1] >= y0 - 1) & (nxy[:, 1] <= y1 + 1))
    if len(edges):
        ok = (edges[:, 0] < len(pos)) & (edges[:, 1] < len(pos))
        e = edges[ok]
        e = e[inside_node[e[:, 0]] & inside_node[e[:, 1]]]
    else:
        e = edges
    if len(e):
        # Come immagine, vedi ARCHI COME IMMAGINE.
        def build():
            segs = np.stack([pos[e[:, 0]], pos[e[:, 1]]], axis=1)
            cols = _edge_colors(vd, e, (0.2, 0.2, 0.2, 0.35), alpha=0.35)
            return _edges_image(segs, cols, x0, x1, y0, y1, max_px=480)
        img = _cached_edges_image(('loc', vis[0], round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)), build)
        ax.imshow(img, origin='lower', extent=[x0, x0 + img.shape[1] * ((x1 - x0) / img.shape[1]),
                                              y0, y0 + img.shape[0] * ((y1 - y0) / img.shape[0])],
                  interpolation='nearest', zorder=3)
    pts = nxy[inside_node[ids]]
    if len(pts):
        ax.plot(pts[:, 0], pts[:, 1], '.', color='k', ms=3, alpha=0.6, zorder=3)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)


def _global_edges_image(ctx, name, vd):
    """
    Immagine degli archi del pannello globale per una vista PRM, in cache (vedi ARCHI COME
    IMMAGINE). Tutti gli archi, senza assottigliamento. (img, extent) o (None, None).
    """
    gx0, gx1 = ctx.x0, ctx.x0 + ctx.nx * ctx.res
    gy0, gy1 = ctx.y0, ctx.y0 + ctx.ny * ctx.res

    def build():
        ids = np.asarray(_arr(vd, 'prm_node_ids', ()), dtype=np.int64).ravel()
        nxy = np.asarray(_arr(vd, 'prm_node_xy', (2,)), dtype=np.float64).reshape(-1, 2)
        edges = np.asarray(_arr(vd, 'prm_edges', (2,)), dtype=np.int64).reshape(-1, 2)
        if not (len(ids) and len(nxy) == len(ids) and len(edges)):
            return None
        pos = np.full((int(ids.max()) + 2, 2), np.nan)
        pos[ids] = nxy
        known = np.zeros(len(pos), dtype=bool)
        known[ids] = True
        ok = (edges[:, 0] < len(pos)) & (edges[:, 1] < len(pos)) & (edges[:, 0] >= 0) & (edges[:, 1] >= 0)
        e = edges[ok]
        e = e[known[e[:, 0]] & known[e[:, 1]]]
        if not len(e):
            return None
        segs = np.stack([pos[e[:, 0]], pos[e[:, 1]]], axis=1)
        cols = _edge_colors(vd, e, (0.5, 0.5, 0.5, 0.25))
        return _edges_image(segs, cols, gx0, gx1, gy0, gy1, min_res=ctx.res,
                            coverage=EDGE_RASTER_COVERAGE_GLOBAL)

    img = _cached_edges_image(('glob', name, gx0, gy0, gx1, gy1), build)
    if img is None:
        return None, None
    return img, [gx0, gx0 + img.shape[1] * ((gx1 - gx0) / img.shape[1]),
                 gy0, gy0 + img.shape[0] * ((gy1 - gy0) / img.shape[0])]


def _prebuild_global_images(ctx):
    """In sottofondo, all'apertura: le immagini di tutte le viste PRM, cosi' scorrere non aspetta."""
    for _mt, name, vd in list(ctx.vis):
        try:
            _global_edges_image(ctx, name, vd)
        except Exception as e:
            print(f"[viewer] immagine archi di {name} non preparata: {e}")


def _traj_runs(tr):
    """
    Spezza la traiettoria (t, x, y, graphnav) dove fra due campioni consecutivi c'e' un salto
    oltre TRAJ_GAP_M, e dove cambia il tipo di movimento. Restituisce
    (tratti registrati come (punti, graphnav), salti come coppie di punti).
    """
    if len(tr) == 0:
        return [], []
    gn = tr[:, 3] > 0.5 if tr.shape[1] > 3 else np.zeros(len(tr), bool)
    jump = np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])) > TRAJ_GAP_M
    cuts = np.flatnonzero(jump | (gn[1:] != gn[:-1])) + 1
    runs = []
    for idx in np.split(np.arange(len(tr)), cuts):
        if len(idx):
            # il tratto parte dall'ultimo punto del precedente, se non c'e' salto in mezzo
            k = idx[0]
            if k > 0 and not jump[k - 1]:
                idx = np.concatenate(([k - 1], idx))
            runs.append((tr[idx, 1:3], bool(gn[idx[-1]])))
    gaps = [(tr[k - 1, 1:3], tr[k, 1:3]) for k in np.flatnonzero(jump) + 1]
    return runs, gaps


def _hillshade(h, res):
    """Ombreggiatura del rilievo (luce da nord-ovest, 45 gradi): 1 = piano. NaN -> 1."""
    gy, gx = np.gradient(h, res)
    alt, az = np.radians(45.0), np.radians(315.0)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    sh = (np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)) / np.sin(alt)
    return np.where(np.isfinite(sh), sh, 1.0)


def _background_rgb(occ, height, mode, norm, res):
    """Immagine RGB dello sfondo 'terreno' o 'fusione' (vedi BACKGROUNDS)."""
    rgb = plt.get_cmap(TERRAIN_CMAP)(norm(np.nan_to_num(height, nan=norm.vmin)))[..., :3]
    if HILLSHADE_STRENGTH > 0:
        shade = 1.0 + HILLSHADE_STRENGTH * (np.clip(_hillshade(height, res), 0.0, 2.0) - 1.0)
        rgb = np.clip(rgb * shade[..., None], 0.0, 1.0)
    has_h = np.isfinite(height)
    rgb[~has_h] = 1.0                                         # mai visto: bianco
    if mode == 'fusione' and occ is not None:
        rgb[~has_h & (occ == 1)] = NO_TERRAIN_RGB
        rgb[occ == 2] = OBSTACLE_RGB
    return rgb


def _hover_global(ax, ctx, occ, height):
    """Coordinate, stato della cella e quota sotto il mouse nel pannello globale."""
    names = {0: 'mai vista', 1: 'libera', 2: 'OSTACOLO'}

    def fmt(x, y):
        c, r = int(np.floor((x - ctx.x0) / ctx.res)), int(np.floor((y - ctx.y0) / ctx.res))
        out = f"x={x:.2f} y={y:.2f}"
        if 0 <= r < ctx.ny and 0 <= c < ctx.nx:
            if occ is not None:
                out += f" | {names.get(int(occ[r, c]), '?')}"
            if height is not None:
                h = height[r, c]
                out += f" | quota {h:+.3f} m" if np.isfinite(h) else " | quota: nessuna"
        return out
    ax.format_coord = fmt


def draw_global(ax, ctx, i, final=False, raw=False, bg='ostacoli'):
    """
    Vista globale alla scansione i (o a fine missione con final=True). raw: mosaico grezzo.
    bg: sfondo, vedi BACKGROUNDS. Restituisce l'oggetto per la barra dei colori della quota
    (None per lo sfondo 'ostacoli').
    """
    ax.clear()
    upto = len(ctx.scans) - 1 if final else i
    d = ctx.scans[upto]
    t_now = float(d['time']) if 'time' in d else None
    use_final = final and not raw and ctx.final_map is not None
    if use_final:
        # Fine missione: la mappa globale salvata dal pianificatore, che contiene anche le
        # scansioni d'inizio tentativo (non salvate in scans/).
        occ = ctx.final_map
    else:
        occ = ctx.mosaic(upto) if raw else ctx.robot_map(upto)
    height = None
    if bg != 'ostacoli':
        height = ctx.final_terrain if (final and ctx.final_terrain is not None) else ctx.terrain(upto)
    mappable = None
    if bg == 'ostacoli':
        ax.imshow(occ, origin='lower', cmap=MOSAIC_CMAP, vmin=0, vmax=2, interpolation='nearest',
                  extent=ctx.extent, zorder=0)
    else:
        norm = ctx.terrain_norm()
        ax.imshow(_background_rgb(occ, height, bg, norm, ctx.res), origin='lower',
                  interpolation='nearest', extent=ctx.extent, zorder=0)
        mappable = plt.cm.ScalarMappable(norm=norm, cmap=TERRAIN_CMAP)
    _hover_global(ax, ctx, occ, height)

    vis = ctx.vis_before(None if final else t_now)
    vd = vis[1] if vis else None
    try:
        _draw_cells(ax, vd)
    except Exception as e:
        print(f"[viewer] celle non disegnate: {e}")
    if vd is not None:
        xy = {int(k): tuple(p) for k, p in zip(_arr(vd, 'prm_node_ids', ()), _arr(vd, 'prm_node_xy', (2,)))}
        # Questo pannello disegna TUTTI gli archi del grafo (fino a ~90 000), come immagine
        # in cache preparata in sottofondo all'apertura (vedi ARCHI COME IMMAGINE).
        img, gext = _global_edges_image(ctx, vis[0], vd)
        if img is not None:
            ax.imshow(img, origin='lower', interpolation='nearest', zorder=2, extent=gext)
        stops = np.array([xy[int(k)] for k in _arr(vd, 'prm_stop_nodes', ()) if int(k) in xy]).reshape(-1, 2)
        if len(stops):
            ax.plot(stops[:, 0], stops[:, 1], '.', color='tab:orange', ms=5, zorder=4, label='nodi di sosta')
        cp = _arr(vd, 'chosen_point').astype(float).ravel()
        if cp.size == 2 and np.all(np.isfinite(cp)):
            ax.plot(*cp, '*', color='green', ms=15, mec='k', zorder=6, label='obiettivo cella')

    # traiettoria reale (trajectory.csv) o, se manca, le posizioni alle scansioni
    if ctx.traj is not None and len(ctx.traj):
        tr = ctx.traj if (final or t_now is None) else ctx.traj[ctx.traj[:, 0] <= t_now + 0.5]
        runs, gaps = _traj_runs(tr)
        halo = [pe.Stroke(linewidth=3.2, foreground='white', alpha=0.8), pe.Normal()]
        labelled = set()
        for r, is_gn in runs:
            lab = 'percorso reale (GraphNav)' if is_gn else 'percorso reale'
            ax.plot(r[:, 0], r[:, 1], '-', color='darkorange' if is_gn else 'tab:blue', lw=1.5,
                    zorder=5, path_effects=halo, label=None if lab in labelled else lab)
            labelled.add(lab)
        for k, (a, b) in enumerate(gaps):
            ax.plot([a[0], b[0]], [a[1], b[1]], ':', color='tab:blue', lw=1.2, zorder=5, path_effects=halo,
                    label='tratto non registrato (GraphNav o arretramento)' if k == 0 else None)
    else:
        track = np.array([s['robot_xyz'][:2] for s in ctx.scans[:upto + 1]])
        ax.plot(track[:, 0], track[:, 1], '-', color='tab:blue', lw=1.5, alpha=0.8, zorder=5,
                label='robot (alle scansioni)')

    for s in ctx.scans[:upto + 1]:
        if str(s['decision_kind']) == 'blocked':
            ax.plot(*s['frontier_stop_xy'], 's', color='red', ms=11, mfc='none', mew=2, zorder=7)
    if not final:
        pl = d['polyline']
        ax.plot(pl[:, 0], pl[:, 1], '-', color='magenta', lw=2.5, zorder=6, label='percorso pianificato')
        x0, x1, y0, y1 = _extent(d)
        ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor='cyan', lw=2, zorder=6))
        reason = str(d['frontier_reason'])
        ax.plot(*d['frontier_stop_xy'], 'X', color=REASON_COLORS.get(reason, 'k'), ms=11, mec='white', zorder=8)
    rx, ry = d['robot_xyz'][:2]
    ax.plot(rx, ry, 'o', color='blue', ms=9, mec='white', zorder=9)

    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
    n_blk = sum(str(s['decision_kind']) == 'blocked' for s in ctx.scans[:upto + 1])
    if use_final:
        title = "Fine missione -- mappa globale salvata dal pianificatore ('m': mosaico grezzo)"
    else:
        title = ("Fine missione" if final else f"Mappa globale alla scansione {upto + 1}/{len(ctx.scans)}")
        title += (" -- mosaico grezzo, l'ultima scansione vince ('m')" if raw
                  else " -- mappa del robot, ostacoli confermati ('m')")
    legend = {
        'ostacoli': "rosso = ostacolo visto, verde chiaro = libero, bianco = mai visto",
        'terreno': "sfondo: quota del terreno rispetto all'avvio missione; bianco = nessuna quota",
        'fusione': "sfondo: quota del terreno, rosso = ostacolo, grigio = libero senza quota, bianco = mai visto",
    }[bg]
    ax.set_title(f"{title}\n{legend} ('t'); quadrati rossi = blocchi ({n_blk})"
                 + (f"\ngrafo: {vis[0]}" if vis else ""), fontsize=8)
    ax.set_xlabel("X [m] (VISION)")
    ax.legend(loc='upper left', fontsize=7, framealpha=0.8)
    return mappable


def _latest_mission_folder():
    """
    Senza argomenti (es. tasto Run dell'editor): l'ultima missione in MissionMap/, cercata
    accanto a questo file e nella cartella corrente. Se non c'e', finestra per sceglierla.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands = []
    for base in (here, os.getcwd(), os.path.dirname(here)):
        cands += [d for d in glob.glob(os.path.join(base, "MissionMap", "Mission_*")) if os.path.isdir(d)]
    if cands:
        return max(set(cands), key=os.path.getmtime)
    try:
        import tkinter
        from tkinter import filedialog
        root = tkinter.Tk()
        root.withdraw()
        return filedialog.askdirectory(title="Cartella della missione (dentro MissionMap)") or None
    except Exception:
        return None


def main():
    skip = {sys.argv.index(o) + 1 for o in ("--png", "--sfondo") if o in sys.argv}
    args = [a for k, a in enumerate(sys.argv[1:], 1) if not a.startswith("--") and k not in skip]
    if not args:
        found = _latest_mission_folder()
        if not found:
            print(__doc__)
            print("Nessuna cartella MissionMap/Mission_* trovata accanto a questo file.")
            return 1
        print(f"Nessuna cartella indicata: apro l'ultima missione, {found}")
        args = [found]
    target = args[0]
    # Basta anche il nome della missione (es. 13-10-2026_12-40-38), come per mission_report.py.
    by_name = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MissionMap",
                           "Mission_" + target.replace("Mission_", ""))
    if not os.path.exists(target) and os.path.isdir(by_name):
        target = by_name
    out_dir = sys.argv[sys.argv.index("--png") + 1] if PNG_MODE else None
    bg0 = 'ostacoli'
    if "--sfondo" in sys.argv:
        bg0 = sys.argv[sys.argv.index("--sfondo") + 1]
        if bg0 not in BACKGROUNDS:
            print(f"--sfondo: uno fra {', '.join(BACKGROUNDS)}")
            return 1

    if os.path.isdir(target):
        scan_files = sorted(glob.glob(os.path.join(target, "scans", "scan_*.npz")))
        vis_files = sorted(glob.glob(os.path.join(target, "*_vis.npz")))
    else:
        scan_files = [target] if os.path.basename(target).startswith("scan_") else []
        vis_files = [target] if target.endswith("_vis.npz") else []
    scans = [_load(f) for f in scan_files]
    print(f"{len(scans)} scansioni, {len(vis_files)} viste PRM")
    folder = target if os.path.isdir(target) else os.path.dirname(os.path.dirname(os.path.abspath(target)))
    ctx = MissionContext(folder, scans, scan_files, vis_files) if scans else None

    if PNG_MODE:
        os.makedirs(out_dir, exist_ok=True)
        for k, (f, d) in enumerate(zip(scan_files, scans)):
            fig, (axl, axr) = plt.subplots(1, 2, figsize=(20, 9.5))
            im, label = draw_scan(axl, d, 5)          # maschera degli ostacoli
            overlay_prm_local(axl, ctx, k)
            fig.colorbar(im, ax=axl, label=label, shrink=0.8)
            draw_global(axr, ctx, k, bg=bg0)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, os.path.basename(f).replace(".npz", ".png")), dpi=100)
            plt.close(fig)
        for f in vis_files:
            fig, ax = plt.subplots(figsize=(11, 11))
            draw_vis_bundle(ax, _load(f))
            fig.savefig(os.path.join(out_dir, os.path.basename(f).replace(".npz", ".png")), dpi=110)
            plt.close(fig)
        for bg in (BACKGROUNDS if scans else ()):
            fig, ax = plt.subplots(figsize=(12, 12))
            m = draw_global(ax, ctx, len(scans) - 1, final=True, bg=bg)
            if m is not None:
                fig.colorbar(m, ax=ax, label="quota rispetto all'avvio missione (m)", shrink=0.7)
            fig.savefig(os.path.join(out_dir, "overview.png" if bg == 'ostacoli' else f"overview_{bg}.png"),
                        dpi=110)
            plt.close(fig)
        print(f"Figure salvate in {out_dir}")
        return 0

    if "--overview" in sys.argv or not scans:
        fig, ax = plt.subplots(figsize=(11, 11))
        if scans:
            m = draw_global(ax, ctx, len(scans) - 1, final=True, bg=bg0)
            if m is not None:
                fig.colorbar(m, ax=ax, label="quota rispetto all'avvio missione (m)", shrink=0.7)
        elif vis_files:
            draw_vis_bundle(ax, _load(vis_files[0]))
        plt.show()
        return 0

    # I tasti del visualizzatore non devono fare anche altro: per matplotlib 'g' e' la griglia
    # e 'o' lo strumento zoom (premendo 'o' restava attivo lo zoom).
    for km in ('keymap.grid', 'keymap.zoom'):
        plt.rcParams[km] = [k for k in plt.rcParams[km] if k not in ('g', 'o', 'm', 't')]
    state = {'i': 0, 'layer': 0, 'overview': False, 'cbar': None, 'prm': True, 'raw': False,
             'bg': bg0}
    fig, (ax, axg) = plt.subplots(1, 2, figsize=(18, 8.5))
    # Barra della quota del pannello destro: creata una volta sola e nascosta con lo sfondo
    # 'ostacoli'. Toglierla e rimetterla a ogni cambio rimpiccioliva il pannello ogni volta.
    cbar_g = fig.colorbar(plt.cm.ScalarMappable(norm=ctx.terrain_norm(), cmap=TERRAIN_CMAP), ax=axg,
                          label="quota rispetto all'avvio missione (m)", shrink=0.8)

    def redraw():
        if state['cbar'] is not None:
            state['cbar'].remove(); state['cbar'] = None
        d = scans[state['i']]
        im, label = draw_scan(ax, d, state['layer'],
                              title_prefix=f"[{state['i'] + 1}/{len(scans)}] "
                                           f"{os.path.basename(scan_files[state['i']])}\n")
        state['cbar'] = fig.colorbar(im, ax=ax, label=label, shrink=0.8)
        if state['prm']:
            overlay_prm_local(ax, ctx, state['i'])
        m = draw_global(axg, ctx, state['i'], final=state['overview'], raw=state['raw'], bg=state['bg'])
        cbar_g.ax.set_visible(m is not None)
        fig.canvas.draw_idle()

    def on_key(ev):
        if ev.key == 'right':
            state['i'] = min(len(scans) - 1, state['i'] + 1); state['overview'] = False
        elif ev.key == 'left':
            state['i'] = max(0, state['i'] - 1); state['overview'] = False
        elif ev.key == 'up':
            state['i'] = min(len(scans) - 1, state['i'] + 10); state['overview'] = False
        elif ev.key == 'down':
            state['i'] = max(0, state['i'] - 10); state['overview'] = False
        elif ev.key in [str((k + 1) % 10) for k in range(min(len(LAYERS), 10))]:
            state['layer'] = (int(ev.key) - 1) % 10   # '0' = decimo layer
        elif ev.key == 'o':
            state['overview'] = not state['overview']
        elif ev.key == 'g':
            state['prm'] = not state['prm']
        elif ev.key == 'm':
            state['raw'] = not state['raw']
        elif ev.key == 't':
            state['bg'] = BACKGROUNDS[(BACKGROUNDS.index(state['bg']) + 1) % len(BACKGROUNDS)]
        else:
            return
        redraw()

    fig.canvas.mpl_connect('key_press_event', on_key)
    print("Frecce sinistra/destra: scansione | su/giu': 10 scansioni | 1-9, 0: layer a sinistra | "
          "o: a destra fine missione / momento della scansione | g: grafo a sinistra si'/no | "
          "m: a destra mappa del robot / mosaico grezzo | t: sfondo ostacoli / terreno / fusione | "
          "valore sotto il mouse in basso a destra")
    for k, layer in enumerate(LAYERS):
        print(f"  {k + 1}: {layer[0]}")
    redraw()
    # Le immagini degli archi del pannello globale si preparano in sottofondo.
    threading.Thread(target=_prebuild_global_images, args=(ctx,), daemon=True).start()
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())