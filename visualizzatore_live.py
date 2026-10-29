#!/usr/bin/env python3
"""
Visualizzatore DAL VIVO della missione: una sola figura che si aggiorna da sola, per capire a
grandi linee che cosa sta facendo il robot. Gira in un terminale a parte, accanto alla
missione, e legge soltanto i file che la missione scrive gia':
  MissionMap/Mission_<nome>/scans/*.npz          una per giro del ciclo (scansione, percorso,
                                                 obiettivo, fronte, decisione)
  MissionMap/Mission_<nome>/iteration_*_vis.npz  una per tentativo (celle, grafo PRM)
  MissionLogs/Mission_<nome>/mission_log.txt     il log, riga per riga
  MissionLogs/Mission_<nome>/trajectory.csv      la traiettoria reale, a fine di ogni tratto
Non tocca la missione e non usa l'SDK: se si chiude o si blocca, la missione non se ne accorge.

    python3 visualizzatore_live.py                       # l'ultima missione, dal vivo
    python3 visualizzatore_live.py 28-10-2026_14-23-50   # una missione precisa, dal vivo
    python3 visualizzatore_live.py 28-10-2026_14-23-50 --rigioca [--velocita 20]
                                                         # missione finita, rigiocata nel tempo
    python3 visualizzatore_live.py <nome> --png <file.png> [--fino-a HH:MM:SS]
                                                         # una figura sola, senza finestra

Nella figura (un solo strato con tutto sopra):
  sfondo      mappa vista finora: rosso ostacolo, verde chiaro libero, bianco mai visto
              (mosaico delle scansioni del ciclo, l'ultima vince); dentro il riquadro azzurro
              l'ultima scansione, piu' scura
  celle       verde visitata, rossa bloccata, tratteggio da esplorare, bordo blu = obiettivo
  grafo PRM   grigio chiaro
  blu         strada fatta (tratteggio = GraphNav)
  arancione   percorso pianificato adesso; stella = obiettivo nella cella
  X           dove si ferma il fronte sicuro (rossa ostacolo, viola fine dei dati, verde fine
              del percorso); linea verde spessa = avanzamento consentito
  freccia     il robot
A destra: stato (cella, decisione, fronte, avanzamento, batteria) e le ultime righe utili del log.

Tasti: f segui il robot / tutta la missione,  spazio pausa,  q esci.
"""
import argparse
import csv
import glob
import os
import re
import sys
import textwrap
import time

import numpy as np
import matplotlib
import matplotlib.patches as patches

HERE = os.path.dirname(os.path.abspath(__file__))
MOSAIC_RES = 0.10           # lato delle celle del mosaico, m
REFRESH_S = 1.0             # ogni quanto si rilegge, dal vivo
LOG_TAGS = ('[PROGRESSO]', '[TARGET]', '[ATTEMPT]', '[VISITED]', '[BLOCKED]', '[BLOCCO]',
            '[SUBITO]', '[REPLAN]', '[REPLAN FALLITO]', '[ARRIVO]', '[OBIETTIVO]', '[STOP]',
            '[GRAPHNAV]', '[BATTERIA]', '[FINE]', '[FRONTIERA]', '[STALLO]', '[EMERGENZA]',
            '[QUASI]', '[PASSO FINALE]', '[RITORNO]', '[MULTINODO]', '[DEFER]', '[PRECHECK]')
N_LOG_LINES = 12
REASON_COLORS = {'ostacolo': 'red', 'rugosita': 'darkorange', 'pendenza': 'darkorange',
                 'fine_dati': 'purple', 'fine_percorso': 'green'}


def _cartelle(nome):
    """(cartella MissionMap, cartella MissionLogs) della missione; l'ultima se nome e' None."""
    if nome is None:
        cands = [d for d in glob.glob(os.path.join(HERE, "MissionMap", "Mission_*")) if os.path.isdir(d)]
        if not cands:
            sys.exit("Nessuna missione in MissionMap/.")
        mm = max(cands, key=os.path.getmtime)
    elif os.path.isdir(nome):
        mm = os.path.abspath(nome)
    else:
        mm = os.path.join(HERE, "MissionMap", nome if nome.startswith("Mission_") else f"Mission_{nome}")
    ml = os.path.join(HERE, "MissionLogs", os.path.basename(mm))
    return mm, ml


def _carica(path):
    """npz come dict; None se il file non si legge (per esempio e' ancora in scrittura)."""
    try:
        with np.load(path, allow_pickle=False) as z:
            return {k: z[k] for k in z.files}
    except Exception:
        return None


def _ostacoli(d):
    """Celle ostacolo della scansione: obstacle_mask vale -1 sugli ostacoli, 1 sul libero."""
    return np.asarray(d['obstacle_mask']) < 0


def _hms(s):
    h, m, x = s.split(':')
    return int(h) * 3600 + int(m) * 60 + float(x)


def _ora_del_giorno(t):
    lt = time.localtime(t)
    return lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec + (t % 1.0)


class Stato:
    """Tutto quello che si sa della missione fino all'istante `fino_a` (None = adesso)."""

    def __init__(self, mm, ml):
        self.mm, self.ml = mm, ml
        self.mosaico = {}            # (ix, iy) -> 1 ostacolo, 0 libero
        self.scan_lette = set()
        self.ultima = None           # ultima scansione del ciclo (dict)
        self.vis = None              # ultima vista PRM (celle e grafo)
        self.vis_path = None
        self.log_righe = []
        self.log_pos = 0
        self.stato_log = {}

    # --- scansioni del ciclo ------------------------------------------------------------
    def _scans(self):
        return sorted(glob.glob(os.path.join(self.mm, "scans", "scan_*.npz")), key=os.path.getmtime)

    def _aggiungi_al_mosaico(self, d):
        obst = _ostacoli(d)
        visto = ~np.asarray(d['unwritten'], dtype=bool) if 'unwritten' in d else np.ones_like(obst)
        ny, nx = obst.shape
        ox, oy = (float(v) for v in d['grid_origin'])
        cs = float(d['cell_size'])
        jj, ii = np.nonzero(visto)
        if not len(jj):
            return
        ix = np.floor((ox + (ii + 0.5) * cs) / MOSAIC_RES).astype(np.int64)
        iy = np.floor((oy + (jj + 0.5) * cs) / MOSAIC_RES).astype(np.int64)
        val = obst[jj, ii]
        # Per ogni cella del mosaico: ostacolo se lo e' almeno il 30% dei punti della scansione.
        key = ix * 1_000_003 + iy
        order = np.argsort(key)
        key, ix, iy, val = key[order], ix[order], iy[order], val[order]
        starts = np.r_[0, np.flatnonzero(np.diff(key)) + 1]
        counts = np.diff(np.r_[starts, len(key)])
        anyobst = np.add.reduceat(val.astype(np.int32), starts) >= 0.3 * counts
        for a, b, v in zip(ix[starts], iy[starts], anyobst):
            self.mosaico[(int(a), int(b))] = int(v)

    def aggiorna_scansioni(self, fino_a=None):
        nuove = 0
        for p in self._scans():
            if p in self.scan_lette:
                continue
            d = _carica(p)
            if d is None:
                continue                      # ancora in scrittura: al prossimo giro
            if fino_a is not None and float(d['time']) > fino_a:
                continue
            self.scan_lette.add(p)
            self._aggiungi_al_mosaico(d)
            if self.ultima is None or float(d['time']) >= float(self.ultima['time']):
                self.ultima = d
            nuove += 1
        return nuove

    # --- viste PRM (celle e grafo) ---------------------------------------------------------
    def aggiorna_vis(self, fino_a=None):
        cands = glob.glob(os.path.join(self.mm, "iteration_*_vis.npz"))
        if fino_a is not None:
            cands = [p for p in cands if os.path.getmtime(p) <= fino_a]
        if not cands:
            return False
        p = max(cands, key=os.path.getmtime)
        if p == self.vis_path:
            return False
        d = _carica(p)
        if d is not None:
            self.vis, self.vis_path = d, p
        return d is not None

    # --- log -------------------------------------------------------------------------------
    def aggiorna_log(self, fino_a=None):
        path = os.path.join(self.ml, "mission_log.txt")
        if not os.path.exists(path):
            return False
        prima = self.log_pos
        limite = None if fino_a is None else _ora_del_giorno(fino_a)
        with open(path, encoding='utf-8', errors='replace') as f:
            f.seek(self.log_pos)
            while True:
                pos = f.tell()
                riga = f.readline()
                if not riga or not riga.endswith('\n'):
                    f.seek(pos)               # riga a meta': si rilegge al prossimo giro
                    break
                m = re.match(r'(\d\d:\d\d:\d\d\.\d+) (.*)', riga.rstrip('\n'))
                if m and limite is not None and _hms(m.group(1)) > limite:
                    f.seek(pos)
                    break
                if m:
                    self._leggi_riga(m.group(1)[:8], m.group(2))
            self.log_pos = f.tell()
        return self.log_pos != prima

    def _leggi_riga(self, ora, testo):
        s = self.stato_log
        if testo.startswith('[PROGRESSO]'):
            s['progresso'] = testo[len('[PROGRESSO] '):]
        elif testo.lstrip().startswith(('min trascorsi', '0.', '1', '2', '3', '4', '5', '6', '7', '8', '9')) \
                and 'min trascorsi' in testo:
            s['tempo'] = testo.strip()
        elif testo.startswith('[ATTEMPT]'):
            s['tentativo'] = testo[len('[ATTEMPT] '):]
        elif testo.startswith('[BATTERIA]'):
            m = re.search(r'(\d+(?:\.\d+)?)\s*%', testo)
            if m:
                s['batteria'] = m.group(1) + '%'
        elif testo.startswith('[GRAPHNAV]'):
            s['graphnav'] = f"{ora} {testo[len('[GRAPHNAV] '):]}"
        if testo.startswith(LOG_TAGS):
            self.log_righe.append(f"{ora} {testo}")
            del self.log_righe[:-N_LOG_LINES]

    def traiettoria(self, fino_a=None):
        """Array (t, x, y, graphnav) dalla traiettoria registrata."""
        path = os.path.join(self.ml, "trajectory.csv")
        if not os.path.exists(path):
            return np.zeros((0, 4))
        rows = []
        try:
            with open(path, newline='') as f:
                for r in csv.DictReader(f):
                    try:
                        t = float(r['time'])
                        if fino_a is not None and t > fino_a:
                            break
                        rows.append((t, float(r['x']), float(r['y']), 1.0 if r.get('move') == 'graphnav' else 0.0))
                    except (TypeError, ValueError, KeyError):
                        continue
        except OSError:
            pass
        return np.array(rows) if rows else np.zeros((0, 4))


# =========================================================================================
# DISEGNO
# =========================================================================================
def _immagine_mosaico(mosaico):
    keys = np.array(list(mosaico.keys()))
    vals = np.array(list(mosaico.values()))
    x0, y0 = keys.min(axis=0)
    x1, y1 = keys.max(axis=0)
    img = np.ones((y1 - y0 + 1, x1 - x0 + 1, 3))
    img[keys[:, 1] - y0, keys[:, 0] - x0] = np.where(vals[:, None] > 0, [0.92, 0.35, 0.35], [0.85, 0.95, 0.85])
    ext = [x0 * MOSAIC_RES, (x1 + 1) * MOSAIC_RES, y0 * MOSAIC_RES, (y1 + 1) * MOSAIC_RES]
    return img, ext


def _celle(ax, vis, cella_obiettivo):
    if vis is None or 'env_cells' not in vis or not len(vis['env_cells']):
        return
    size = float(vis.get('env_cell_size', 5.0))
    yaw = float(vis.get('env_origin_yaw', 0.0))
    h = size / 2.0
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    base = np.array([[-h, -h], [h, -h], [h, h], [-h, h]]) @ R.T
    for (r, c, x, y), st in zip(vis['env_cells'], vis['env_cell_status']):
        st = str(st)
        if st == '1':
            kw = dict(edgecolor='darkgreen', facecolor='lightgreen', alpha=0.25, lw=1.0)
        elif st == '-1':
            kw = dict(edgecolor='darkred', facecolor='lightcoral', alpha=0.35, lw=1.0)
        else:
            kw = dict(edgecolor='gray', facecolor='none', alpha=0.6, lw=0.7, ls='--')
        if cella_obiettivo is not None and (int(r), int(c)) == cella_obiettivo:
            kw.update(edgecolor='blue', lw=2.2, alpha=0.9)
        ax.add_patch(patches.Polygon(base + [x, y], zorder=4, **kw))
        ax.text(x, y, f"{int(r)},{int(c)}", ha='center', va='center', fontsize=7, color='0.25', zorder=4,
                clip_on=True)


def _grafo(ax, vis):
    if vis is None or 'prm_edges' not in vis or not len(vis['prm_edges']):
        return
    pos = {int(n): xy for n, xy in zip(vis['prm_node_ids'], vis['prm_node_xy'])}
    segs = [(pos[int(a)], pos[int(b)]) for a, b in vis['prm_edges'] if int(a) in pos and int(b) in pos]
    from matplotlib.collections import LineCollection
    ax.add_collection(LineCollection(segs, colors=(0.5, 0.5, 0.5, 0.18), linewidths=0.4, zorder=2))


def disegna(fig, ax_map, ax_txt, st, segui, fino_a=None):
    ax_map.clear()
    ax_txt.clear()
    ax_txt.axis('off')
    d = st.ultima
    if st.mosaico:
        img, ext = _immagine_mosaico(st.mosaico)
        ax_map.imshow(img, origin='lower', extent=ext, interpolation='nearest', zorder=0)
    cella = None
    if d is not None:
        cella = (int(d['cell'][0]), int(d['cell'][1]))
        # Ultima scansione, piu' scura, dentro il suo riquadro.
        ny, nx = d['obstacle_mask'].shape
        ox, oy = (float(v) for v in d['grid_origin'])
        cs = float(d['cell_size'])
        ext_l = [ox, ox + nx * cs, oy, oy + ny * cs]
        loc = np.zeros((ny, nx, 4))
        visto = ~np.asarray(d['unwritten'], dtype=bool) if 'unwritten' in d else np.ones((ny, nx), bool)
        obst = _ostacoli(d)
        loc[visto & ~obst] = [0.55, 0.80, 0.55, 0.55]
        loc[visto & obst] = [0.75, 0.05, 0.05, 0.85]
        ax_map.imshow(loc, origin='lower', extent=ext_l, interpolation='nearest', zorder=3)
        ax_map.add_patch(patches.Rectangle((ext_l[0], ext_l[2]), nx * cs, ny * cs, fill=False,
                                           edgecolor='deepskyblue', lw=1.2, zorder=4))
    _celle(ax_map, st.vis, cella)
    _grafo(ax_map, st.vis)

    tr = st.traiettoria(fino_a)
    if len(tr):
        gn = tr[:, 3] > 0.5
        salto = np.r_[False, np.hypot(np.diff(tr[:, 1]), np.diff(tr[:, 2])) > 1.5]
        for sel, ls in ((~gn, '-'), (gn, '--')):
            xy = tr[:, 1:3].copy()
            xy[~sel | salto] = np.nan
            ax_map.plot(xy[:, 0], xy[:, 1], ls, color='royalblue', lw=1.4, zorder=5)

    righe = []
    if d is not None:
        rx, ry, _ = (float(v) for v in d['robot_xyz'])
        yaw = float(d['robot_yaw'])
        poly = np.asarray(d['polyline'])
        if len(poly) > 1:
            ax_map.plot(poly[:, 0], poly[:, 1], '-o', color='darkorange', lw=2, ms=3, zorder=6)
        gx, gy = (float(v) for v in d['goal_xy'])
        ax_map.plot(gx, gy, '*', color='gold', mec='k', ms=16, zorder=8)
        reason = str(d['frontier_reason'])
        sx, sy = (float(v) for v in d['frontier_stop_xy'])
        ax_map.plot(sx, sy, 'x', color=REASON_COLORS.get(reason, 'k'), ms=12, mew=3, zorder=8)
        adv = float(d['allowed_advance'])
        if len(poly) > 1 and adv > 0:
            pts, rest = [poly[0]], adv
            for a, b in zip(poly[:-1], poly[1:]):
                L = float(np.hypot(*(b - a)))
                if L >= rest:
                    pts.append(a + (b - a) * (rest / max(L, 1e-9)))
                    break
                pts.append(b)
                rest -= L
            pts = np.array(pts)
            ax_map.plot(pts[:, 0], pts[:, 1], '-', color='limegreen', lw=5, alpha=0.7, zorder=7)
        ax_map.add_patch(patches.FancyArrow(rx, ry, 0.6 * np.cos(yaw), 0.6 * np.sin(yaw), width=0.12,
                                            head_width=0.35, head_length=0.25, color='k', zorder=9))
        if segui:
            ax_map.set_xlim(rx - 6, rx + 6)
            ax_map.set_ylim(ry - 6, ry + 6)
        eta = time.time() - float(d['time']) if fino_a is None else fino_a - float(d['time'])
        righe += [
            f"Cella obiettivo  ({cella[0]},{cella[1]})   giro {int(d['loop_iter'])}, tentativo {int(d['iteration'])}",
            f"Decisione        {str(d['decision_kind'])}",
            f"Fronte           {reason}: {float(d['frontier_dist']):.2f} m confermati su "
            f"{float(d['frontier_path_len']):.2f} m, avanzo {adv:.2f} m",
            f"Robot            ({rx:.2f}, {ry:.2f})  muso {np.degrees(yaw):.0f} gradi"
            + (f"   fermo da {int(float(d['no_progress']))} scansioni" if float(d['no_progress']) > 0 else ""),
            f"Obiettivo        ({gx:.2f}, {gy:.2f}), a {np.hypot(gx - rx, gy - ry):.2f} m in linea d'aria",
            f"Ultima scansione {eta:.0f} s fa",
        ]
    s = st.stato_log
    if 'progresso' in s:
        righe.append(f"Avanzamento      {s['progresso']}")
    if 'tempo' in s:
        righe.append(f"                 {s['tempo']}")
    if 'batteria' in s:
        righe.append(f"Batteria         {s['batteria']}")
    if 'graphnav' in s:
        righe.append(f"Ultimo GraphNav  {s['graphnav'][:90]}")
    righe = [w for r in righe for w in (textwrap.wrap(r, 66, subsequent_indent=' ' * 17) or [''])]
    ax_txt.text(0.0, 1.0, "\n".join(righe), va='top', ha='left', family='monospace', fontsize=9,
                transform=ax_txt.transAxes)
    log = "\n".join(w for r in st.log_righe for w in textwrap.wrap(r, 92, subsequent_indent='    '))
    ax_txt.text(0.0, 0.52, "Ultime righe del log\n" + log, va='top', ha='left', family='monospace',
                fontsize=7.5, transform=ax_txt.transAxes, color='0.15')
    ax_map.set_aspect('equal')
    if not segui and st.mosaico:
        ax_map.autoscale()
    nome = os.path.basename(st.mm).replace('Mission_', '')
    quando = time.strftime('%H:%M:%S', time.localtime(fino_a if fino_a is not None else time.time()))
    ax_map.set_title(f"Missione {nome} -- {quando}" + ("   (segue il robot: f)" if segui else "   (tutta: f)"),
                     fontsize=10)
    fig.canvas.draw_idle()


def main():
    ap = argparse.ArgumentParser(description="Visualizzatore dal vivo della missione.")
    ap.add_argument('missione', nargs='?', help="nome (es. 28-10-2026_14-23-50) o cartella; default l'ultima")
    ap.add_argument('--rigioca', action='store_true', help="rigioca una missione finita nel suo tempo")
    ap.add_argument('--velocita', type=float, default=20.0, help="velocita' del rigioco (default 20x)")
    ap.add_argument('--png', help="salva una figura e esce (con --fino-a: a quell'ora)")
    ap.add_argument('--fino-a', help="HH:MM:SS, con --png")
    ap.add_argument('--tutta', action='store_true', help="parte mostrando tutta la missione invece di seguire il robot")
    args = ap.parse_args()

    if args.png:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    mm, ml = _cartelle(args.missione)
    if not os.path.isdir(mm):
        sys.exit(f"Cartella non trovata: {mm}")
    st = Stato(mm, ml)
    fig = plt.figure(figsize=(15, 8.5))
    ax_map = fig.add_axes([0.04, 0.05, 0.55, 0.9])
    ax_txt = fig.add_axes([0.61, 0.03, 0.38, 0.94])
    stato = {'segui': not args.tutta, 'pausa': False, 'esci': False, 'ridisegna': True}

    def tasto(ev):
        stato['ridisegna'] = True
        if ev.key == 'f':
            stato['segui'] = not stato['segui']
        elif ev.key == ' ':
            stato['pausa'] = not stato['pausa']
        elif ev.key in ('q', 'escape'):
            stato['esci'] = True
    fig.canvas.mpl_connect('key_press_event', tasto)
    fig.canvas.mpl_connect('close_event', lambda ev: stato.update(esci=True))

    # Istante d'inizio per rigioco e --fino-a: la prima scansione della missione.
    scans = st._scans()
    t_inizio = None
    if scans:
        d0 = _carica(scans[0])
        t_inizio = float(d0['time']) if d0 is not None else None

    if args.png:
        fino_a = None
        if args.fino_a and t_inizio is not None:
            base = time.localtime(t_inizio)
            h, m, s = (int(v) for v in args.fino_a.split(':'))
            fino_a = time.mktime((base.tm_year, base.tm_mon, base.tm_mday, h, m, s, 0, 0, -1))
        elif scans:
            dl = _carica(scans[-1])
            fino_a = float(dl['time']) + 60.0 if dl is not None else None
        st.aggiorna_scansioni(fino_a)
        st.aggiorna_vis(fino_a)
        st.aggiorna_log(fino_a)
        disegna(fig, ax_map, ax_txt, st, stato['segui'], fino_a)
        fig.savefig(args.png, dpi=110)
        print(f"Salvata {args.png}")
        return

    if args.rigioca and t_inizio is None:
        sys.exit("Nessuna scansione da rigiocare.")
    t_rig, t_wall = t_inizio, time.time()
    print(f"Missione {os.path.basename(mm)}: " + ("rigioco" if args.rigioca else "dal vivo") +
          ". Tasti: f segui/tutta, spazio pausa, q esci.")
    while not stato['esci']:
        now = time.time()
        if not stato['pausa']:
            fino_a = None
            if args.rigioca:
                t_rig += (now - t_wall) * args.velocita
                fino_a = t_rig
            # Si ridisegna solo se c'e' qualcosa di nuovo (o un tasto): il disegno costa ~0.4 s
            # di CPU, sullo stesso computer della missione.
            nuovo = st.aggiorna_scansioni(fino_a) > 0
            nuovo = st.aggiorna_vis(fino_a) or nuovo
            nuovo = st.aggiorna_log(fino_a) or nuovo
            if nuovo or stato['ridisegna'] or args.rigioca:
                disegna(fig, ax_map, ax_txt, st, stato['segui'], fino_a)
                stato['ridisegna'] = False
        t_wall = now
        plt.pause(REFRESH_S)
        if not plt.fignum_exists(fig.number):
            break


if __name__ == '__main__':
    main()
