"""
Prova SENZA ROBOT di naviga_mappa.py: il ciclo vero (NavigatoreRobot.vai) contro il robot e la
griglia locale finti di test_e2e_loop.py.

Il mondo finto e' nel frame di OGGI. La mappa della missione (occupazione, terreno, PRM,
waypoint GraphNav) e' costruita nel frame VECCHIO, spostato e ruotato rispetto a oggi di
T_VERA: e' la situazione dopo un riavvio del robot. Un GraphNav finto localizza il robot sul
waypoint piu' vicino, come quello vero; tutto il resto (stima del frame, PRM sulla mappa,
fronte sicuro, decisioni, scarto degli archi e nuova pianificazione) e' il codice vero.

Scenari:
  1  muro con un varco: il percorso lo aggira e il robot arriva;
  2  un ostacolo che a fine missione non c'era: entra nella mappa, il percorso cambia per
     strada e il robot arriva lo stesso (2b: con le mappe fisse, scartando gli archi);
  3  obiettivo dentro un ostacolo: il robot si ferma sul nodo raggiungibile piu' vicino;
  5  un ostacolo della mappa che oggi non c'e' piu': con le mappe aggiornate la cella torna
     libera e il robot va dritto, senza (--senza-aggiornare) fa il giro;
  6  le mappe aggiornate: quota di oggi riportata a quella della missione, l'ostacolo nuovo
     dello scenario 2 occupato nella mappa, salvataggio e ricarica da --mappe;
  4  scelta dell'obiettivo con un clic (SceltaConClic), con eventi finti al posto del mouse:
     punto giusto, percorso subito, conferma con Invio, clic ignorati fuori dalla mappa e
     senza conferma nessun obiettivo.

    python3 test_naviga.py
"""
import contextlib
import io
import sys
from types import SimpleNamespace as NS

import numpy as np

import test_e2e_loop as T          # mondo, robot e griglia finti; applica le sue patch
import arcVerification as av
import naviga_mappa as nm
import spotGrid as sg

FAILS = []
T_VERA = nm.Trasformazione2D(2.0, -1.0, 0.5)     # frame della missione -> frame di oggi
RES = 0.03


def check(cond, msg):
    print(f"  [{'OK ' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _vecchio_da_nuovo(x, y):
    c, s = np.cos(T_VERA.th), np.sin(T_VERA.th)
    dx, dy = x - T_VERA.tx, y - T_VERA.ty
    return c * dx + s * dy, -s * dx + c * dy


def _nuovo_da_vecchio(x, y):
    c, s = np.cos(T_VERA.th), np.sin(T_VERA.th)
    return T_VERA.tx + c * x - s * y, T_VERA.ty + s * x + c * y


def mappa_dal_mondo(world, start_xy, start_yaw):
    """La mappa di fine missione, nel frame vecchio, come se l'esplorazione avesse visto tutto
    il mondo `world`. Celle della mappa vecchia -> punto di oggi -> stato nel mondo."""
    ext = T.W * T.CS
    corners = np.array([_vecchio_da_nuovo(x, y) for x in (0, ext) for y in (0, ext)])
    gx = np.arange(int(np.floor(corners[:, 0].min() / RES)), int(np.ceil(corners[:, 0].max() / RES)))
    gy = np.arange(int(np.floor(corners[:, 1].min() / RES)), int(np.ceil(corners[:, 1].max() / RES)))
    GX, GY = np.meshgrid(gx, gy, indexing='ij')
    nx, ny = _nuovo_da_vecchio((GX + 0.5) * RES, (GY + 0.5) * RES)
    c, r = np.floor(nx / T.CS).astype(int), np.floor(ny / T.CS).astype(int)
    inside = (c >= 0) & (c < T.W) & (r >= 0) & (r < T.W)
    occ = np.zeros(GX.shape, bool)
    # Come la missione: occupata la cella con obstacle_distance sotto soglia (spotGrid.
    # mask_for_global_map), non solo l'ostacolo fisico.
    occ[inside] = world.od[r[inside], c[inside]] <= sg.OBSTACLE_THRESHOLD
    gm = sg.GlobalGrid(resolution=RES)
    tm = sg.GlobalTerrainGrid(resolution=RES)
    for k1, k2, o in zip(GX[inside].tolist(), GY[inside].tolist(), occ[inside].tolist()):
        gm._set_state((k1, k2), -1.0 if o else 1.0)
        tm.height_sum[(k1, k2)] = 0.0
        tm.height_count[(k1, k2)] = 1
    tm.update_rects = [(gx[0] * RES, (gx[-1] + 1) * RES, gy[0] * RES, (gy[-1] + 1) * RES)]
    ox, oy = _vecchio_da_nuovo(*start_xy)
    cfg = dict(name='finta', origin=(ox, oy), yaw=start_yaw - T_VERA.th, rows=5, cols=5, cell=2.0,
               spacing=0.5, sampling='reticolo', min_edge=0.5, max_edge=1.25,
               distribution=None, density=None, seed=None)
    # Waypoint GraphNav ogni 1.5 m sul libero, orientati a caso (nel frame vecchio).
    rng = np.random.default_rng(3)
    wps = {}
    for x in np.arange(1.0, ext - 0.5, 1.5):
        for y in np.arange(1.0, ext - 0.5, 1.5):
            if world.od_at(x, y) > 0.5:
                wx, wy = _vecchio_da_nuovo(x, y)
                wid = f"id{len(wps)}"
                wps[wid] = dict(name=f"wp_{len(wps)}", x=wx, y=wy, yaw=float(rng.uniform(-np.pi, np.pi)))
    ng = dict(dir=None, graph=None, wp=wps, by_name={v['name']: k for k, v in wps.items()})
    with contextlib.redirect_stdout(io.StringIO()):
        return nm.MappaMissione('finta', cfg, gm, tm, ng)


class FakeGraphNav:
    """Localizzazione sul waypoint piu' vicino, con la posa del corpo rispetto a lui."""

    def __init__(self, wps):
        self.wps = wps

    def get_localization_state(self):
        ox, oy = _vecchio_da_nuovo(T.ROBOT.x, T.ROBOT.y)
        oyaw = T.ROBOT.yaw - T_VERA.th
        wid = min(self.wps, key=lambda k: np.hypot(self.wps[k]['x'] - ox, self.wps[k]['y'] - oy))
        w = self.wps[wid]
        c, s = np.cos(w['yaw']), np.sin(w['yaw'])
        dx, dy = ox - w['x'], oy - w['y']
        pos = NS(x=c * dx + s * dy, y=-s * dx + c * dy, z=0.0)
        return NS(localization=NS(waypoint_id=wid, waypoint_tform_body=NS(position=pos,
                                                                         rotation=T.quat(oyaw - w['yaw']))))


def _snapshot():
    pts, od, terr, valid, _ = T.window()
    lg = T.FakeLocalGrid()
    tc, iv = lg.correct_terrain(terr, valid, T.N, T.N, cell_size=T.CS)
    _, rough = lg.compute_gradient_and_roughness(tc, valid, T.N, T.N, T.CS, is_valid=iv)
    c0 = int(np.floor(T.ROBOT.x / T.CS)) - T.N // 2
    r0 = int(np.floor(T.ROBOT.y / T.CS)) - T.N // 2
    return av.make_grid_snapshot(od, rough, iv, valid, tc, T.N, T.N, c0 * T.CS, r0 * T.CS, T.CS,
                                 T.ROBOT.x, T.ROBOT.y, robot_yaw=T.ROBOT.yaw)


def navigatore(mappa, aggiorna=True):
    nav = object.__new__(nm.NavigatoreRobot)     # senza robot: niente RecordingInterface
    nav.mappa, nav.robot, nav.rsc, nav.cc = mappa, None, None, None
    nav.ri = NS(waypoint_poses={})
    nav.gn = FakeGraphNav(mappa.ng['wp'])
    nav.T = None
    nav.tracker = T.FakeTracker()
    nav.tracker._fresh_snapshot = _snapshot
    nav.local_grid = T.FakeLocalGrid()
    nav._init_mappe(aggiorna=aggiorna)
    return nav


def run(name, rects_mappa, rects_oggi, start, goal_new, aggiorna=True):
    print(f"\n=== {name} ===")
    av.OBSTACLE_HISTORY.reset()
    mappa = mappa_dal_mondo(T.World(rects_mappa, ghosts=False), start[:2], start[2])
    T.WORLD = T.World(rects_oggi, ghosts=False)
    T.ROBOT = T.Robot(*start)
    nav = navigatore(mappa, aggiorna)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ok_loc = nav.aggiorna_trasformazione("prova") is not None
        ok = nav.vai(_vecchio_da_nuovo(*goal_new), f"({goal_new[0]}, {goal_new[1]}) di oggi",
                     ripiego_graphnav=False)
    log = out.getvalue()
    if '--log' in sys.argv:
        print(log)
    dm, dg = nav.T.scarto(T_VERA)
    check(ok_loc and dm < 0.01 and dg < 0.1,
          f"frame stimato {nav.T} contro il vero {T_VERA} (scarto {dm:.3f} m, {dg:.2f} gradi)")
    d_goal = float(np.hypot(T.ROBOT.x - goal_new[0], T.ROBOT.y - goal_new[1]))
    return dict(ok=ok, d_goal=d_goal, min_clear=T.ROBOT.min_clear, walked=T.ROBOT.walked, nav=nav,
                log=log, piani=log.count('] Piano '), scartati=log.count('scarto') + log.count('scartato'))


def main():
    np.seterr(all='ignore')
    # 1. Muro da x = 5 a 5.4 con un varco in alto (y > 8): da (1.5, 1.5) a (9, 2).
    muro = [(5.0, 5.4, 0.0, 8.0)]
    r = run("1 muro con varco", muro, muro, (1.5, 1.5, 0.0), (9.0, 2.0))
    check(r['ok'], f"arrivato ({r['piani']} piani, {r['walked']:.1f} m percorsi)")
    check(r['d_goal'] < 0.8, f"a {r['d_goal']:.2f} m dall'obiettivo")
    check(r['min_clear'] > 0.2, f"distanza minima dagli ostacoli {r['min_clear']:.2f} m")

    # 2. Un blocco nuovo sul corridoio sotto il varco, che la mappa non ha.
    nuovo = muro + [(5.4, 7.0, 8.3, 8.7)]
    r = run("2 ostacolo nuovo", muro, nuovo, (1.5, 1.5, 0.0), (9.0, 2.0))
    check(r['ok'], f"arrivato lo stesso ({r['piani']} piani)")
    check(r['piani'] >= 2 or 'nuovo percorso' in r['log'],
          f"strada cambiata dopo l'ostacolo nuovo ({r['piani']} piani, "
          f"{r['log'].count('nuovo percorso')} cambi di percorso per strada)")
    check(r['min_clear'] > 0.2, f"distanza minima dagli ostacoli {r['min_clear']:.2f} m")

    r0 = run("2b ostacolo nuovo, mappe fisse", muro, nuovo, (1.5, 1.5, 0.0), (9.0, 2.0), aggiorna=False)
    check(r0['ok'], f"arrivato anche con le mappe fisse ({r0['piani']} piani)")
    gm = r['nav'].mappa.gm
    ox, oy = _vecchio_da_nuovo(6.2, 8.5)                       # dentro l'ostacolo nuovo
    check(gm.is_occupied(ox, oy), "6: l'ostacolo nuovo e' occupato nella mappa aggiornata")
    check(r['nav'].dz is not None and abs(r['nav'].dz - 3.23) < 0.02,
          f"6: scarto di quota stimato {r['nav'].dz:+.3f} m (vero +3.23: quote finte di oggi a -3.23)")
    check(r['nav'].n_scansioni_in_mappa > 10,
          f"6: {r['nav'].n_scansioni_in_mappa} scansioni nelle mappe, {r['nav'].t_mappe_s:.1f} s in tutto")
    import os, tempfile
    tmp = tempfile.mkdtemp(prefix="naviga_mappe_")
    with contextlib.redirect_stdout(io.StringIO()):
        r['nav'].salva_mappe(tmp)
    gm2 = nm.carica_mappa_occupazione(os.path.join(tmp, 'global_occupancy_map.pkl'))
    tm2 = nm.carica_mappa_terreno(os.path.join(tmp, 'global_terrain_map.pkl'))
    check(gm2.is_occupied(ox, oy) and len(gm2.grid) == len(gm.grid) and len(tm2.height_count) == len(r['nav'].mappa.tm.height_count),
          "6: mappe salvate e ricaricate uguali")
    h = tm2.get_height(*_vecchio_da_nuovo(8.0, 1.0))
    check(h is not None and abs(h) < 0.02, f"6: quota della mappa invariata dove il robot e' passato ({h:+.3f} m)")

    # 5. Ostacolo nella mappa, oggi sparito: davanti alla partenza, sulla retta verso (9, 1.5).
    # Tutto dentro la finestra della scansione dalla partenza (3.8 m di lato attorno al robot):
    # una parte lontana resterebbe occupata finche' il robot non ci arriva vicino.
    vecchio = [(2.6, 3.2, 0.5, 2.5)]
    r = run("5 ostacolo sparito, mappe aggiornate", vecchio, [], (1.5, 1.5, 0.0), (9.0, 1.5))
    r0 = run("5b ostacolo sparito, mappe fisse", vecchio, [], (1.5, 1.5, 0.0), (9.0, 1.5), aggiorna=False)
    check(r['ok'] and r0['ok'], "arrivato in tutti e due i casi")
    check(r['walked'] < 8.0, f"mappe aggiornate: {r['walked']:.1f} m percorsi (linea d'aria 7.5 m)")
    check(r0['walked'] > r['walked'] + 0.3, f"mappe fisse: {r0['walked']:.1f} m, il giro attorno "
                                            f"all'ostacolo che non c'e' piu'")
    check('nuovo percorso' in r['log'], "con le mappe aggiornate il percorso cambia per strada")

    # 3. Obiettivo dentro un ostacolo pieno.
    blocco = [(7.0, 9.0, 3.0, 5.0)]
    r = run("3 obiettivo dentro un ostacolo", blocco, blocco, (1.5, 1.5, 0.0), (8.0, 4.0))
    check(r['ok'], "arrivato al nodo raggiungibile piu' vicino")
    check(1.0 < r['d_goal'] < 2.0, f"fermo fuori dall'ostacolo, a {r['d_goal']:.2f} m dal centro")

    # 4. Scelta con un clic: eventi finti, figura senza finestra (backend Agg).
    print("\n=== 4 obiettivo con un clic ===")
    mappa = mappa_dal_mondo(T.World(muro, ghosts=False), (1.5, 1.5), 0.0)
    da = _vecchio_da_nuovo(1.5, 1.5)
    with contextlib.redirect_stdout(io.StringIO()):
        sc = nm.SceltaConClic(mappa, da)
        # Obiettivo (9, 2) di oggi = (7.5, 0.5) nel frame della missione (partenza in (1.5, 1.5),
        # muso lungo +x); in figura il punto (xm, ym) sta in (ym, xm).
        sc._clic(NS(inaxes=sc.ax, button=1, xdata=0.5, ydata=7.5))
    gx, gy = _nuovo_da_vecchio(*sc.goal)
    check(np.hypot(gx - 9.0, gy - 2.0) < 1e-6, f"clic in (7.5, 0.5) del frame della missione -> "
                                               f"({gx:.2f}, {gy:.2f}) di oggi")
    check(sc.piano is not None and sc.piano['lunghezza'] > 9.0,
          f"percorso pianificato subito, attorno al muro ({sc.piano['lunghezza']:.1f} m)")
    with contextlib.redirect_stdout(io.StringIO()):
        sc._clic(NS(inaxes=None, button=1, xdata=None, ydata=None))           # fuori dagli assi
        sc._clic(NS(inaxes=sc.ax, button=3, xdata=3.0, ydata=3.0))            # tasto destro
    check(np.hypot(*np.subtract(_nuovo_da_vecchio(*sc.goal), (9.0, 2.0))) < 1e-6,
          "clic fuori dalla mappa e col tasto destro ignorati")
    check(len(sc._artisti) == 3, "un solo percorso disegnato (il vecchio tolto)")
    sc._tasto(NS(key='enter'))
    r = sc.scegli()
    check(r is not None and r[0] == sc.goal, "Invio: obiettivo confermato")
    with contextlib.redirect_stdout(io.StringIO()):
        sc2 = nm.SceltaConClic(mappa, da)
        sc2._clic(NS(inaxes=sc2.ax, button=1, xdata=2.0, ydata=2.0))
    sc2._tasto(NS(key='escape'))
    check(sc2.scegli() is None, "Esc: nessun obiettivo")

    # 5. Le trasformazioni: andata e ritorno.
    t = nm.Trasformazione2D.da_pose((3.0, 4.0, 1.0), (1.0, -2.0, 0.3))
    x, y = t.inversa(*t.applica(5.0, 6.0))
    check(np.hypot(x - 5.0, y - 6.0) < 1e-9 and np.hypot(*np.subtract(t.applica(1.0, -2.0), (3.0, 4.0))) < 1e-9,
          "Trasformazione2D: andata e ritorno")
    fr = nm.FrameMissione(10.0, 20.0, np.radians(90))
    check(np.allclose(fr.a_vision(1.0, 0.0), (10.0, 21.0)) and np.allclose(fr.da_vision(*fr.a_vision(2, 3)), (2, 3)),
          "FrameMissione: x avanti, y a sinistra")

    print(f"\n{'TUTTO OK' if not FAILS else f'{len(FAILS)} CONTROLLI FALLITI'}")
    return not FAILS


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
