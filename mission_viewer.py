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
    ... --navgraph                                             # nav graph visibile dall'inizio
    ... --frame-missione                                       # pannello globale nel frame della missione

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
                                 (terreno colorato per quota, ostacoli in rosso sopra;
                                 linea bianca = quota 0, il suolo all'avvio missione, segnata
                                 anche sulla barra)
    r                            pannello destro nel frame della MISSIONE: griglia delle celle dritta,
                                 partenza nell'origine, muso del robot alla partenza verso l'alto
                                 (sotto il mouse restano anche le coordinate VISION)
    n                            nav graph di GraphNav sul pannello destro: mostra / nascondi
                                 (serve l'SDK di Spot per leggerlo, vedi NAV GRAPH)
    lo zoom del pannello destro resta quando si cambia scansione o sfondo
    il valore sotto il mouse compare in basso a destra, con le coordinate VISION

File letti:
    scans/scan_*.npz    un pacchetto per scansione del ciclo di navigazione
    *_vis.npz           al posto delle vecchie figure: dati + grafo PRM in quel momento
    *_BLOCKED_*.npz     i dati che hanno deciso un blocco
"""
import glob
import os
import re
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
import matplotlib.patheffects as patheffects          # noqa: E402
import csv                                            # noqa: E402
import matplotlib.patheffects as pe                   # noqa: E402
from matplotlib.transforms import Affine2D            # noqa: E402

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
TERRAIN_PERCENTILES = (2, 98)           # estremi della scala dei colori della quota (con lo 0 sempre dentro)
ZERO_LEVEL_COLOR = 'white'              # linea di livello e riga sulla barra a quota 0 (partenza)
ZERO_LEVEL_SMOOTH_CELLS = 5             # media su 5 celle (15 cm) solo per tracciare quella linea
HILLSHADE_STRENGTH = 0.45               # 0 = niente ombreggiatura del rilievo

# Le mappe si costruiscono scansione per scansione; tornando indietro si riparte dall'ultima
# copia salvata invece che dalla prima scansione (343 scansioni nella missione del 22-10 16:04).
CHECKPOINT_EVERY = 25

# NAV GRAPH (2026-10-26), tasto 'n'. Il grafo GraphNav scaricato a fine missione in
# graph/<missione>/Grafo_*/: per ogni waypoint la posizione e' quella del corpo nel frame
# VISION quando il waypoint e' stato creato (dallo snapshot), cioe' lo stesso frame della
# mappa. Dagli snapshot (558 MB nella missione del 22-10 16:04, quasi tutto nuvole di punti)
# si legge solo lo stato del robot, senza decodificare il resto: 0.3 s invece di 51.
# Leggere il formato di Boston Dynamics richiede l'SDK (bosdyn); senza, il tasto non fa nulla.
NAVGRAPH_EDGE_STYLE = {    # origine dell'arco -> (colore, spessore, tratto, etichetta)
    'EDGE_SOURCE_ODOMETRY': ('#303030', 0.9, '-', 'nav graph: arco registrato'),
    'EDGE_SOURCE_USER_REQUEST': ('deepskyblue', 1.2, '-', 'nav graph: scorciatoia nostra'),
    'EDGE_SOURCE_LOCALIZATION': ('darkviolet', 1.2, '--', "nav graph: chiusura d'anello"),
    'EDGE_SOURCE_SMALL_LOOP_CLOSURE': ('darkviolet', 1.2, '--', "nav graph: chiusura d'anello"),
    'EDGE_SOURCE_ALTERNATE_ROUTE_FINDING': ('darkviolet', 1.2, '--', "nav graph: chiusura d'anello"),
}
NAVGRAPH_OTHER_STYLE = ('gray', 0.9, ':', 'nav graph: altro arco')

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


def _body_masks(d):
    """Celle sotto il corpo (nucleo) e nell'anello attorno, per la scansione d. Vedi SELF_*."""
    h, w = d['terrain_raw'].shape
    if 'robot_yaw' not in d:
        z = np.zeros((h, w), bool)
        return z, z
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
    return core, ring


# QUOTA SOTTO IL CORPO (2026-10-26). Alla prima scansione di una missione il robot si e'
# appena alzato e il terreno sotto il corpo e' "mai scritto": nei layer di quota restava una
# macchia bianca. Il robot quel punto lo rivede dopo, da lontano: nei layer di quota del
# pannello sinistro e nel terreno del pannello globale quelle celle prendono la quota della
# mappa del terreno di fine missione, tratteggiate. Missione del 26-10 15:11: tutte le 90
# celle coperte, 1.5 cm dal terreno attorno. Solo disegno: il robot le tratta come non viste.
# Layer di quota -> riferimento sottratto (stesso del layer).
QUOTA_LAYER_REF = {
    "quota rispetto al suolo": lambda d: _ground(d),
    "quota corretta": lambda d: _ground(d),
    "quota rispetto all'avvio missione": lambda d: _mission_z0(d),
}


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


def draw_scan(ax, d, layer_idx, title_prefix="", ctx=None):
    ax.clear()
    name, fn, cmap, vmin, vmax, label = LAYERS[layer_idx]
    mat = np.asarray(fn(d), dtype=float)
    ext = _extent(d)
    # quota sotto il corpo vista dopo (vedi QUOTA SOTTO IL CORPO)
    hole = None
    if ctx is not None and name in QUOTA_LAYER_REF:
        q, hole = ctx.body_quota(d)
        if hole is not None:
            mat = mat.copy()
            mat[hole] = q[hole] - QUOTA_LAYER_REF[name](d)
    im = ax.imshow(mat, origin='lower', extent=ext, cmap=cmap, vmin=vmin, vmax=vmax,
                   interpolation='nearest')
    _hover(ax, mat, ext)
    if hole is not None:
        cs = float(d['cell_size'])
        xs_c = ext[0] + (np.arange(mat.shape[1]) + 0.5) * cs
        ys_c = ext[2] + (np.arange(mat.shape[0]) + 0.5) * cs
        ax.contourf(xs_c, ys_c, hole.astype(float), levels=[0.5, 1.5], colors='none', hatches=['////'])
        ax.add_patch(patches.Rectangle((0, 0), 0, 0, fill=False, hatch='////', edgecolor='k', lw=0,
                                       label='sotto il corpo: quota vista dopo'))

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
    # Ingombro del corpo (come la zona del corpo della mappa globale). Alla prima scansione
    # di una missione il terreno sotto il corpo e' spesso "mai scritto" (bianco): il robot si
    # e' appena alzato li' e i sensori non l'hanno mai visto. E' trattato come non visto,
    # non come ostacolo.
    ax.add_patch(patches.Rectangle((rx - SELF_BODY_LENGTH_M / 2, ry - SELF_BODY_WIDTH_M / 2),
                                   SELF_BODY_LENGTH_M, SELF_BODY_WIDTH_M, angle=float(np.degrees(yaw)),
                                   rotation_point=(rx, ry), fill=False, edgecolor='blue', lw=1.2, ls='--',
                                   zorder=6))
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


def _navgraph_folder(folder):
    """graph/<missione>/Grafo_* piu' recente, cercato accanto a MissionMap/ e a questo file."""
    name = os.path.basename(os.path.normpath(folder))
    bases = {os.path.dirname(os.path.dirname(os.path.abspath(folder))),
             os.path.dirname(os.path.abspath(__file__))}
    cands = []
    for b in bases:
        cands += [d for d in glob.glob(os.path.join(b, "graph", name, "Grafo_*")) if os.path.isdir(d)]
    return max(cands, key=lambda d: os.path.basename(d)) if cands else None


def _proto_field(buf, number):
    """
    Il campo `number` (length-delimited) al primo livello di un messaggio protobuf serializzato,
    senza decodificare gli altri: negli snapshot dei waypoint permette di saltare le nuvole di
    punti. None se non c'e'.
    """
    def varint(i):
        r = sh = 0
        while True:
            c = buf[i]; i += 1
            r |= (c & 0x7f) << sh; sh += 7
            if c < 0x80:
                return r, i
    i = 0
    while i < len(buf):
        tag, i = varint(i)
        f, wt = tag >> 3, tag & 7
        if wt == 2:
            n, i = varint(i)
            if f == number:
                return buf[i:i + n]
            i += n
        elif wt == 0:
            _, i = varint(i)
        elif wt == 1:
            i += 8
        elif wt == 5:
            i += 4
        else:
            return None
    return None


def load_navgraph(folder):
    """
    Il nav graph della missione: dict con 'names', 'xy' (N, 2) in VISION, 'time' (N,) istante
    di creazione (orologio del robot), 'edges' (M, 2) indici, 'source' (M,) origine dell'arco,
    'folder'. None se il grafo manca o non c'e' l'SDK per leggerlo.
    """
    gdir = _navgraph_folder(folder)
    if gdir is None:
        print("[viewer] nav graph: nessuna cartella graph/<missione>/Grafo_* per questa missione.")
        return None
    try:
        from bosdyn.api.graph_nav import map_pb2
        from bosdyn.api import robot_state_pb2
        from bosdyn.client.frame_helpers import get_a_tform_b
    except Exception:
        print("[viewer] nav graph: per leggerlo serve l'SDK di Spot (bosdyn-client), qui non c'e'.")
        return None
    try:
        g = map_pb2.Graph()
        with open(os.path.join(gdir, "graph"), 'rb') as f:
            g.ParseFromString(f.read())
        fn = map_pb2.WaypointSnapshot.DESCRIPTOR.fields_by_name['robot_state'].number
        ids, names, xy, times = [], [], [], []
        for w in g.waypoints:
            path = os.path.join(gdir, "waypoint_snapshots", w.snapshot_id)
            if not os.path.exists(path):
                continue
            with open(path, 'rb') as f:
                raw = _proto_field(f.read(), fn)
            if raw is None:
                continue
            rs = robot_state_pb2.RobotState()
            rs.ParseFromString(raw)
            ks = rs.kinematic_state
            p = get_a_tform_b(ks.transforms_snapshot, 'vision', 'body')
            if p is None:
                continue
            ids.append(w.id); names.append(w.annotations.name or w.id[:12])
            xy.append((p.x, p.y))
            ts = ks.acquisition_timestamp
            times.append(ts.seconds + ts.nanos * 1e-9 if ts.seconds else np.nan)
        index = {k: n for n, k in enumerate(ids)}
        edges, source = [], []
        for e in g.edges:
            a, b = index.get(e.id.from_waypoint), index.get(e.id.to_waypoint)
            if a is None or b is None:
                continue
            edges.append((a, b))
            try:
                source.append(map_pb2.Edge.EdgeSource.Name(e.annotations.edge_source))
            except Exception:
                source.append('')
        print(f"[viewer] nav graph: {len(ids)} waypoint, {len(edges)} archi ({os.path.basename(gdir)}).")
        return {'names': np.array(names), 'xy': np.array(xy, float).reshape(-1, 2),
                'time': np.array(times, float), 'edges': np.array(edges, int).reshape(-1, 2),
                'source': np.array(source), 'folder': gdir}
    except Exception as e:
        print(f"[viewer] nav graph non leggibile ({gdir}): {e}")
        return None


_VIS_NAME = re.compile(r'iteration_(\d+)(?:_cell_\d+_\d+)?(?:_replan_(\d+)|_(BLOCKED)|_(FINAL_SCAN))?_vis\.npz$')


def _vis_times(vis, scans):
    """
    Istante di ogni vista PRM, per abbinarla alle scansioni (vedi vis_before), e lista
    ordinata. Dal 2026-10-26 la vista ha il campo 'time'. Prima no, e la data del file non
    serve: dopo un git clone o un pull e' quella del download, uguale per tutte, e si vedeva
    sempre la prima vista. Allora l'istante si ricostruisce dal nome e dalla posizione:
      iteration_I_cell_R_C          inizio del tentativo I: poco prima della sua prima scansione
      ..._replan_K                  ripianificazione durante il giro: la scansione dell'iterazione
                                    I in cui il robot stava dove dice la vista (robot_xy)
      ..._BLOCKED                   fine del tentativo bloccato: dopo la sua ultima scansione
      iteration_I_FINAL_SCAN        scansione finale della missione: dopo tutte
    Un tentativo senza scansioni (fallito prima di muoversi) va dopo l'ultima scansione
    precedente. Gli istanti sono resi crescenti nell'ordine dei nomi.
    """
    if not scans:
        return sorted(vis, key=lambda v: v[1])
    t_sc = np.array([float(d['time']) if 'time' in d else np.nan for d in scans])
    it_sc = np.array([int(d['iteration']) if 'iteration' in d else -1 for d in scans])
    xy_sc = np.array([np.asarray(d['robot_xyz'], float)[:2] for d in scans])
    if not np.isfinite(t_sc).all():
        return sorted(vis, key=lambda v: v[1])
    keyed, last_match = [], {}
    for _t, name, d in vis:
        m = _VIS_NAME.search(name)
        it = int(m.group(1)) if m else -1
        phase, sub = ((3, 0) if m and m.group(4) else (2, 0) if m and m.group(3)
                      else (1, int(m.group(2))) if m and m.group(2) else (0, 0))
        keyed.append(((it, phase, sub, name), name, d))
    keyed.sort(key=lambda k: k[0])
    out, t_prev = [], -np.inf
    for (it, phase, sub, _n), name, d in keyed:
        if 'time' in d and np.isfinite(_scalar(d, 'time', np.nan)):
            t = _scalar(d, 'time', np.nan)
        else:
            idx = np.flatnonzero(it_sc == it)
            before = t_sc[it_sc < it]
            t_after_prev = (before.max() if before.size else t_sc[0] - 1.0) + 0.01
            if phase == 3:
                t = t_sc.max() + 1.0
            elif not idx.size:
                t = t_after_prev
            elif phase == 0:
                t = max(t_sc[idx[0]] - 0.5, min(t_after_prev, t_sc[idx[0]] - 1e-3))
            elif phase == 2:
                t = t_sc[idx[-1]] + 0.01
            else:
                # la scansione dell'iterazione piu' vicina a robot_xy, non prima di quella
                # della ripianificazione precedente
                idx = idx[idx >= last_match.get(it, -1)]
                rxy = _arr(d, 'robot_xy').astype(float).ravel()
                k = idx[int(np.argmin(np.hypot(*(xy_sc[idx] - rxy).T)))] if rxy.size == 2 else idx[0]
                last_match[it] = k
                t = t_sc[k]
        t = max(t, t_prev + 1e-3)
        t_prev = t
        out.append((t, name, d))
    return out


class MissionContext:
    """Dati di tutta la missione, caricati una volta: scansioni, viste PRM, traiettoria."""

    def __init__(self, folder, scans, scan_files, vis_files):
        self.scans, self.scan_files = scans, scan_files
        self.vis = []
        for f in vis_files:
            try:
                self.vis.append((np.nan, os.path.basename(f), _load(f)))
            except Exception as e:
                print(f"[viewer] vista PRM non leggibile {os.path.basename(f)}: {e}")
        self.vis = _vis_times(self.vis, scans)
        self.traj = self._load_trajectory(folder)
        occ = self._load_pkl_cells(folder, "global_occupancy_map.pkl", 'grid')
        ter = self._load_pkl_cells(folder, "global_terrain_map.pkl", 'height_count')
        self.final_map = self.final_terrain = None
        self._cache = {}
        self._terrain_norm = None
        self._folder = folder
        self._navgraph = False          # False = non ancora letto, None = non disponibile
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
        core, ring = _body_masks(d)
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

    def navgraph(self):
        """Il nav graph (vedi load_navgraph), letto alla prima richiesta."""
        if self._navgraph is False:
            self._navgraph = load_navgraph(self._folder)
            ng = self._navgraph
            if ng is not None and self.scans:
                # Se l'orologio del robot non e' confrontabile con quello delle scansioni,
                # niente filtro per istante: si mostra sempre tutto il grafo.
                t = ng['time']
                t_scan = [float(d['time']) for d in (self.scans[0], self.scans[-1]) if 'time' in d]
                if (not np.isfinite(t).any() or len(t_scan) < 2
                        or np.nanmin(t) > t_scan[1] + 60 or np.nanmax(t) < t_scan[0] - 60):
                    ng['time'] = np.full(len(t), np.nan)
        return self._navgraph

    def walked(self, upto):
        """Celle su cui il corpo e' passato nelle scansioni 0..upto (dalla mappa del robot)."""
        self.robot_map(upto)
        return self._cache['robot']['state'][2]

    def body_quota(self, d):
        """
        Quota assoluta della mappa del terreno di fine missione sulle celle della scansione d
        che stanno sotto il corpo e non hanno quota (vedi QUOTA SOTTO IL CORPO), e la loro
        maschera. (None, None) se non serve o non c'e'.
        """
        if self.final_terrain is None:
            return None, None
        core, _ = _body_masks(d)
        shape = d['terrain_raw'].shape
        hole = core & ~np.asarray(d['is_valid'], bool).reshape(shape)
        if not hole.any():
            return None, None
        h, w = shape
        cs = float(d['cell_size'])
        gox, goy = d['grid_origin']
        X, Y = np.meshgrid(gox + (np.arange(w) + 0.5) * cs, goy + (np.arange(h) + 0.5) * cs)
        c = np.floor((X - self.x0) / self.res).astype(int)
        r = np.floor((Y - self.y0) / self.res).astype(int)
        inside = (c >= 0) & (c < self.nx) & (r >= 0) & (r < self.ny)
        q = np.full(shape, np.nan)
        q[inside] = self.final_terrain[r[inside], c[inside]] + self.z0
        hole &= np.isfinite(q)
        return (q, hole) if hole.any() else (None, None)

    def mission_frame(self):
        """
        (x, y, yaw) del frame della missione: origine nel centro della cella (0, 0), che e' il
        punto di partenza, e orientamento della griglia delle celle (env_origin_yaw, il muso
        del robot alla partenza). Dalle viste PRM; senza, dalla prima scansione.
        """
        for _t, _n, vd in self.vis:
            cells = _arr(vd, 'env_cells', (4,))
            yaw = _scalar(vd, 'env_origin_yaw', np.nan)
            k = np.flatnonzero((cells[:, 0] == 0) & (cells[:, 1] == 0)) if len(cells) else []
            if len(k) and np.isfinite(yaw):
                return float(cells[k[0], 2]), float(cells[k[0], 3]), float(yaw)
        d = self.scans[0]
        return float(d['robot_xyz'][0]), float(d['robot_xyz'][1]), float(d.get('robot_yaw', 0.0))

    def terrain_norm(self):
        """Scala dei colori della quota, fissa per la missione (vedi TERRAIN_PERCENTILES)."""
        if self._terrain_norm is None:
            h = self.final_terrain if self.final_terrain is not None else self.terrain(len(self.scans) - 1)
            h = h[np.isfinite(h)]
            lo, hi = (np.percentile(h, TERRAIN_PERCENTILES) if h.size else (-0.5, 0.5))
            # La quota di partenza (0) sta sempre nella scala, cosi' si vede sulla barra.
            lo, hi = min(lo, 0.0), max(hi, 0.0)
            if hi - lo < 0.05:
                lo, hi = lo - 0.025, hi + 0.025
            self._terrain_norm = Normalize(vmin=float(lo), vmax=float(hi))
        return self._terrain_norm

    def vis_before(self, t):
        """Ultima vista PRM salvata entro l'istante t (None se nessuna). Vedi _vis_times."""
        best = None
        for mt, name, d in self.vis:
            if t is None or not (mt > t):
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


def _box_sum(a, k):
    """Somma su una finestra k x k centrata (bordi a zero), con le somme cumulate: solo numpy."""
    r = k // 2
    p = np.pad(a, ((r + 1, r), (r + 1, r)))
    c = p.cumsum(0).cumsum(1)
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


def _draw_zero_level(ax, height, extent):
    """
    Linea di livello a quota 0 (il suolo all'avvio missione) sopra lo sfondo del terreno:
    separa cio' che sta sotto la partenza da cio' che sta sopra. Niente linea se la quota non
    attraversa lo zero.
    """
    if height is None:
        return
    # Media su ZERO_LEVEL_SMOOTH_CELLS celle, solo per la linea: sul terreno piatto attorno
    # alla partenza il rumore della quota (qualche cm) la riempirebbe di piccoli anelli.
    k = ZERO_LEVEL_SMOOTH_CELLS
    ok = np.isfinite(height)
    num = _box_sum(np.where(ok, height, 0.0), k)
    den = _box_sum(ok.astype(np.float64), k)
    with np.errstate(invalid='ignore', divide='ignore'):
        smooth = np.where(ok & (den > 0), num / np.maximum(den, 1), np.nan)
    h = np.ma.masked_invalid(smooth)
    if h.count() == 0 or not (h.min() < 0.0 < h.max()):
        return
    ny, nx = h.shape
    xs = np.linspace(extent[0], extent[1], nx, endpoint=False) + (extent[1] - extent[0]) / nx / 2
    ys = np.linspace(extent[2], extent[3], ny, endpoint=False) + (extent[3] - extent[2]) / ny / 2
    cs = ax.contour(xs, ys, h, levels=[0.0], colors=ZERO_LEVEL_COLOR, linewidths=1.4, zorder=1)
    stroke = [patheffects.withStroke(linewidth=3.0, foreground='black')]
    # Il bordo nero si mette dopo: contour non accetta path_effects nelle versioni vecchie, e
    # nelle nuove non ha piu' .collections.
    for art in (getattr(cs, 'collections', None) or [cs]):
        art.set_path_effects(stroke)
    ax.plot([], [], '-', color=ZERO_LEVEL_COLOR, lw=1.4,
            path_effects=[patheffects.withStroke(linewidth=3.0, foreground='black')],
            label="quota 0 (suolo all'avvio missione)")


def _mark_zero(cbar):
    """Lo zero sulla barra della quota: tacca e riga, se sta nella scala."""
    lo, hi = cbar.mappable.norm.vmin, cbar.mappable.norm.vmax
    if not (lo <= 0.0 <= hi):
        return
    ticks = [t for t in cbar.get_ticks() if lo <= t <= hi and abs(t) > 1e-9 * max(1.0, hi - lo)]
    extra = [0.0]
    if lo < 0.0 and all(abs(t - lo) > 0.25 * (hi - lo) / max(len(ticks), 1) for t in ticks + extra):
        extra.append(round(lo, 2))      # l'estremo negativo, se non c'e' gia' una tacca vicina
    cbar.set_ticks(sorted(ticks + extra))
    cbar.ax.axhline(0.0, color=ZERO_LEVEL_COLOR, lw=2.0,
                    path_effects=[patheffects.withStroke(linewidth=3.5, foreground='black')])


def _hover_global(ax, ctx, occ, height, ng=None):
    """Coordinate, stato della cella, quota e waypoint vicino sotto il mouse nel pannello globale."""
    names = {0: 'mai vista', 1: 'libera', 2: 'OSTACOLO'}

    def fmt(x, y):
        out = _fmt_cell(x, y)
        if ng is not None and len(ng['xy']):
            dist = np.hypot(ng['xy'][:, 0] - x, ng['xy'][:, 1] - y)
            k = int(np.argmin(dist))
            if dist[k] < 0.3:
                out += f" | {ng['names'][k]}"
        return out

    def _fmt_cell(x, y):
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


def draw_navgraph(ax, ng, t_now=None):
    """Nav graph sul pannello globale; con t_now solo i waypoint gia' creati a quell'istante."""
    shown = np.ones(len(ng['xy']), bool)
    if t_now is not None and np.isfinite(ng['time']).any():
        shown = ~(ng['time'] > t_now + 1.0)
    xy, e = ng['xy'], ng['edges']
    keep = shown[e[:, 0]] & shown[e[:, 1]] if len(e) else np.zeros(0, bool)
    styles = {}
    for (a, b), src in zip(e[keep], ng['source'][keep]):
        styles.setdefault(NAVGRAPH_EDGE_STYLE.get(src, NAVGRAPH_OTHER_STYLE), []).append((xy[a], xy[b]))
    for (col, lw, ls, lab), segs in styles.items():
        ax.add_collection(LineCollection(segs, colors=col, linewidths=lw, linestyles=ls, zorder=3,
                                         label=lab))
    names = ng['names']
    ours = shown & np.char.startswith(names.astype(str), 'wp_')
    auto = shown & ~ours
    if auto.any():
        ax.plot(xy[auto, 0], xy[auto, 1], '.', color='dimgray', ms=3, zorder=3,
                label='waypoint automatici di GraphNav')
    if ours.any():
        ax.plot(xy[ours, 0], xy[ours, 1], 'o', color='black', mfc='white', ms=3.5, mew=0.8, zorder=3,
                label='waypoint della missione (briciole, celle)')
    k0 = np.flatnonzero(shown & (names == 'wp_0'))
    if len(k0):
        ax.plot(*xy[k0[0]], 'P', color='black', mfc='gold', ms=11, zorder=8, label='wp_0 (partenza)')


def _to_mission_frame(ax, ctx, fmt_vision):
    """
    Ruota e trasla tutto cio' che e' disegnato su ax nel frame della missione (vedi
    MissionContext.mission_frame): x laterale, y avanti, origine nella partenza. I dati
    restano in VISION; cambia solo la trasformazione del disegno.
    """
    ox, oy, yaw = ctx.mission_frame()
    T = Affine2D().translate(-ox, -oy).rotate(np.pi / 2 - yaw)
    for art in list(ax.images) + list(ax.lines) + list(ax.collections) + list(ax.patches) + list(ax.texts):
        art.set_transform(T + ax.transData)
    x0, x1, y0, y1 = ctx.extent
    corners = T.transform(np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]))
    ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
    ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
    inv = T.inverted()

    def fmt(x, y):
        xv, yv = inv.transform((x, y))
        return f"missione ({x:.2f}, {y:.2f}) | VISION " + fmt_vision(xv, yv)
    ax.format_coord = fmt
    ax.set_xlabel("laterale [m] (frame missione: origine alla partenza)")
    ax.set_ylabel("avanti [m]")


def draw_global(ax, ctx, i, final=False, raw=False, bg='ostacoli', navgraph=False, mframe=False):
    """
    Vista globale alla scansione i (o a fine missione con final=True). raw: mosaico grezzo.
    bg: sfondo, vedi BACKGROUNDS. navgraph: disegna il nav graph (vedi load_navgraph).
    mframe: frame della missione invece di VISION (vedi _to_mission_frame). Restituisce l'oggetto per la barra dei colori della quota
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
        if final and ctx.final_terrain is not None:
            height = ctx.final_terrain
        else:
            height = ctx.terrain(upto)
            if ctx.final_terrain is not None:
                # dove il corpo e' passato e la quota manca: quella vista dopo (QUOTA SOTTO IL CORPO)
                fill = np.isnan(height) & ctx.walked(upto) & np.isfinite(ctx.final_terrain)
                height[fill] = ctx.final_terrain[fill]
    mappable = None
    if bg == 'ostacoli':
        ax.imshow(occ, origin='lower', cmap=MOSAIC_CMAP, vmin=0, vmax=2, interpolation='nearest',
                  extent=ctx.extent, zorder=0)
    else:
        norm = ctx.terrain_norm()
        ax.imshow(_background_rgb(occ, height, bg, norm, ctx.res), origin='lower',
                  interpolation='nearest', extent=ctx.extent, zorder=0)
        mappable = plt.cm.ScalarMappable(norm=norm, cmap=TERRAIN_CMAP)
        _draw_zero_level(ax, height, ctx.extent)
    ng = ctx.navgraph() if navgraph else None
    if ng is not None:
        draw_navgraph(ax, ng, None if final else t_now)
    _hover_global(ax, ctx, occ, height, ng)

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
            # senza bordo bianco: sotto un tratteggio il bordo continuo lo fa sembrare una linea piena
            ax.plot([a[0], b[0]], [a[1], b[1]], linestyle=(0, (1.5, 2.5)), color='navy', lw=1.3, zorder=5,
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
    if mframe:
        _to_mission_frame(ax, ctx, ax.format_coord)
    ax.legend(loc='upper left', fontsize=7, framealpha=0.8)
    return mappable


# Posizioni [sinistra, basso, larghezza, altezza] in frazioni della finestra (vedi main).
LAYOUT = {
    'scan': [0.035, 0.055, 0.405, 0.835],
    'scan_cbar': [0.447, 0.12, 0.008, 0.70],
    'global': [0.535, 0.055, 0.405, 0.835],
    'global_cbar': [0.947, 0.12, 0.008, 0.70],
}


def _maximize_window(fig):
    """Apre la finestra massimizzata, se il backend lo permette (Qt, Tk, wx)."""
    try:
        mng = fig.canvas.manager
        win = getattr(mng, 'window', None)
        if win is None:
            return
        if hasattr(win, 'showMaximized'):          # Qt
            win.showMaximized()
        elif hasattr(win, 'state'):                # Tk
            try:
                win.state('zoomed')                # Windows, macOS
            except Exception:
                win.attributes('-zoomed', True)    # Linux
        elif hasattr(win, 'Maximize'):             # wx
            win.Maximize(True)
    except Exception:
        pass


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
    ng0 = "--navgraph" in sys.argv
    mf0 = "--frame-missione" in sys.argv
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
            im, label = draw_scan(axl, d, 5, ctx=ctx)          # maschera degli ostacoli
            overlay_prm_local(axl, ctx, k)
            fig.colorbar(im, ax=axl, label=label, shrink=0.8)
            draw_global(axr, ctx, k, bg=bg0, navgraph=ng0, mframe=mf0)
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
            m = draw_global(ax, ctx, len(scans) - 1, final=True, bg=bg, navgraph=ng0, mframe=mf0)
            if m is not None:
                _mark_zero(fig.colorbar(m, ax=ax, label="quota rispetto all'avvio missione (m)", shrink=0.7))
            fig.savefig(os.path.join(out_dir, "overview.png" if bg == 'ostacoli' else f"overview_{bg}.png"),
                        dpi=110)
            plt.close(fig)
        print(f"Figure salvate in {out_dir}")
        return 0

    if "--overview" in sys.argv or not scans:
        fig, ax = plt.subplots(figsize=(11, 11))
        if scans:
            m = draw_global(ax, ctx, len(scans) - 1, final=True, bg=bg0, navgraph=ng0, mframe=mf0)
            if m is not None:
                _mark_zero(fig.colorbar(m, ax=ax, label="quota rispetto all'avvio missione (m)", shrink=0.7))
        elif vis_files:
            draw_vis_bundle(ax, _load(vis_files[0]))
        plt.show()
        return 0

    # I tasti del visualizzatore non devono fare anche altro: per matplotlib 'g' e' la griglia
    # e 'o' lo strumento zoom (premendo 'o' restava attivo lo zoom), 'r' la vista iniziale.
    for km in ('keymap.grid', 'keymap.zoom', 'keymap.home'):   # 'r' e' anche "vista iniziale"
        plt.rcParams[km] = [k for k in plt.rcParams[km] if k not in ('g', 'o', 'm', 't', 'n', 'r')]
    state = {'i': 0, 'layer': 0, 'overview': False, 'prm': True, 'raw': False,
             'bg': bg0, 'nav': ng0, 'rot': mf0, 'auto_lim': None}
    # Disposizione fissa (2026-10-26): i due pannelli e le loro barre dei colori in posizioni
    # date, che usano quasi tutta la finestra. Con plt.subplots e le barre aggiunte "accanto"
    # a schermo intero (1920 x 1080) i pannelli erano 541 x 541 px; cosi' circa 800 x 800.
    # Le barre hanno assi propri: ridisegnarle non rimpicciolisce i pannelli.
    fig = plt.figure(figsize=(18, 9.5))
    ax = fig.add_axes(LAYOUT['scan'])
    cax = fig.add_axes(LAYOUT['scan_cbar'])
    axg = fig.add_axes(LAYOUT['global'])
    cax_g = fig.add_axes(LAYOUT['global_cbar'])
    for a in (ax, axg):
        a.set_anchor('C')
    _maximize_window(fig)
    # Barra della quota del pannello destro: creata una volta sola e nascosta con lo sfondo
    # 'ostacoli'.
    cbar_g = fig.colorbar(plt.cm.ScalarMappable(norm=ctx.terrain_norm(), cmap=TERRAIN_CMAP), cax=cax_g,
                          label="quota rispetto all'avvio missione (m)")
    _mark_zero(cbar_g)

    def redraw():
        d = scans[state['i']]
        im, label = draw_scan(ax, d, state['layer'], ctx=ctx,
                              title_prefix=f"[{state['i'] + 1}/{len(scans)}] "
                                           f"{os.path.basename(scan_files[state['i']])}\n")
        cax.clear()
        fig.colorbar(im, cax=cax, label=label)
        if state['prm']:
            overlay_prm_local(ax, ctx, state['i'])
        # Lo zoom del pannello destro resta: se i limiti sono diversi da quelli automatici del
        # disegno precedente li ha cambiati l'utente, e si rimettono dopo il ridisegno.
        lim = (axg.get_xlim(), axg.get_ylim())
        zoomed = state['auto_lim'] is not None and lim != state['auto_lim']
        m = draw_global(axg, ctx, state['i'], final=state['overview'], raw=state['raw'], bg=state['bg'],
                        navgraph=state['nav'], mframe=state['rot'])
        state['auto_lim'] = (axg.get_xlim(), axg.get_ylim())
        if zoomed:
            axg.set_xlim(lim[0]); axg.set_ylim(lim[1])
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
        elif ev.key == 'n':
            state['nav'] = not state['nav']
        elif ev.key == 'r':
            state['rot'] = not state['rot']
            state['auto_lim'] = None        # coordinate diverse: lo zoom non si conserva
        elif ev.key == 't':
            state['bg'] = BACKGROUNDS[(BACKGROUNDS.index(state['bg']) + 1) % len(BACKGROUNDS)]
        else:
            return
        redraw()

    fig.canvas.mpl_connect('key_press_event', on_key)
    print("Frecce sinistra/destra: scansione | su/giu': 10 scansioni | 1-9, 0: layer a sinistra | "
          "o: a destra fine missione / momento della scansione | g: grafo a sinistra si'/no | "
          "m: a destra mappa del robot / mosaico grezzo | t: sfondo ostacoli / terreno / fusione | "
          "n: nav graph si'/no | r: frame della missione si'/no | "
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