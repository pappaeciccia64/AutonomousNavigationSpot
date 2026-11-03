#!/usr/bin/env python3
"""
PROVA D'ANDATURA SU TRACCIATO: lo stesso tratto rettilineo percorso piu' volte con una
configurazione d'andatura FISSA, misurando come va. Serve a tarare i livelli d'andatura
(spotGrid.GAIT_*, movements.select_mobility_params), che dai log di missione non si possono
ricavare: in missione ogni terreno ha visto una sola configurazione.

    python3 prova_andatura.py --terreno rampa15 --pendenza-vera 14.5 --config crawl_mu06 --dist 5
    python3 prova_andatura.py --terreno prato --config auto_mu08 --ripetizioni 5 --ritorno
    python3 prova_andatura.py --terreno rampa15 --pendenza-vera 14.5 --solo-stima
    python3 prova_andatura.py --elenco          # le configurazioni disponibili

Il robot va messo all'INIZIO del tracciato, con il muso lungo il tracciato, a terra libera
per --dist metri davanti (e dietro, con --ritorno, perche' gira su se stesso alla fine).
Prima di ogni passaggio lo script aspetta Invio: si controlla che il tracciato sia libero e
si riporta il robot all'inizio se serve (o si usa --ritorno). E-Stop dal tablet sempre a
portata di mano: qui non c'e' il fronte sicuro della missione, il robot va dritto e basta
(l'anticollisione di Spot resta quella della missione).

Per ogni passaggio, una riga in ProveAndatura/prove.csv:
  - terreno, verso (andata/ritorno), configurazione, pendenza misurata con l'inclinometro;
  - pendenza e rugosita' STIMATE dalla scansione prima di partire, con lo stesso calcolo del
    PRM (col corpo, 90° percentile) e il livello che la missione avrebbe scelto;
  - riuscita, tempo, distanza comandata e percorsa, velocita';
  - beccheggio e rollio: medi, di picco, e RMS attorno alla media (l'oscillazione: su una rampa
    il beccheggio medio e' la pendenza, l'RMS e' quanto il robot "balla");
  - slittamento dei piedi (foot_slip_distance di Spot, piede in appoggio: picco e media) e
    attrito stimato da Spot sotto i piedi (ground_mu_est), che e' la misura con cui confrontare
    ground_mu_hint;
  - energia (tensione x corrente della batteria integrata) e carica prima e dopo.
La traiettoria campionata va in ProveAndatura/<terreno>_<config>_<ora>.npz.

python3 taratura.py legge prove.csv e fa il confronto per terreno e configurazione.

NOTA: lo script usa le stesse funzioni della missione (movements.relative_move, LocalGrid,
spotGrid.compute_arc_slope_profile), ma non e' mai stato provato sul robot: la prima volta
farlo al chiuso, su 2 m in piano (vedi PROVA_CAMPO.md, "Taratura dell'andatura").
"""
import argparse
import csv
import os
import threading
import time
from types import SimpleNamespace

import numpy as np
import bosdyn.client.lease
from bosdyn.api import robot_state_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, blocking_sit

import movements
import prm_graph
import spotGrid
import spotLogInUtils
import spotUtils
from registro import _quat_to_roll_pitch_deg

# Configurazioni da confrontare. Le prime tre sono i livelli di oggi (select_mobility_params);
# le altre isolano un fattore alla volta (andatura, passo, attrito). 'missione' sceglie come
# la missione, dalla pendenza stimata prima di partire.
AUTO, CRAWL = spot_command_pb2.HINT_AUTO, spot_command_pb2.HINT_CRAWL
MED, HIGH = spot_command_pb2.SWING_HEIGHT_MEDIUM, spot_command_pb2.SWING_HEIGHT_HIGH
CONFIG = {
    'auto_mu08':      dict(locomotion_hint=AUTO, swing_height=MED, ground_mu_hint=0.8,
                           max_linear_vel=spotGrid.GAIT_VEL_PLAIN, max_angular_vel=spotGrid.GAIT_ANG_PLAIN),
    'crawl_mu06':     dict(locomotion_hint=CRAWL, swing_height=HIGH, ground_mu_hint=0.6,
                           max_linear_vel=spotGrid.GAIT_VEL_MODERATE, max_angular_vel=spotGrid.GAIT_ANG_MODERATE),
    'crawl_mu04':     dict(locomotion_hint=CRAWL, swing_height=HIGH, ground_mu_hint=0.4,
                           max_linear_vel=spotGrid.GAIT_VEL_HARD, max_angular_vel=spotGrid.GAIT_ANG_HARD),
    'auto_alto_mu06': dict(locomotion_hint=AUTO, swing_height=HIGH, ground_mu_hint=0.6,
                           max_linear_vel=spotGrid.GAIT_VEL_MODERATE, max_angular_vel=spotGrid.GAIT_ANG_MODERATE),
    'auto_alto_mu04': dict(locomotion_hint=AUTO, swing_height=HIGH, ground_mu_hint=0.4,
                           max_linear_vel=spotGrid.GAIT_VEL_HARD, max_angular_vel=spotGrid.GAIT_ANG_HARD),
    'crawl_mu08':     dict(locomotion_hint=CRAWL, swing_height=HIGH, ground_mu_hint=0.8,
                           max_linear_vel=spotGrid.GAIT_VEL_MODERATE, max_angular_vel=spotGrid.GAIT_ANG_MODERATE),
}
DESCRIZIONE = {
    'auto_mu08': "= PLAIN di oggi (andatura normale, passo medio, attrito 0.8)",
    'crawl_mu06': "= MODERATE di oggi (crawl, passo alto, attrito 0.6)",
    'crawl_mu04': "= HARD di oggi (crawl, passo alto, attrito 0.4)",
    'auto_alto_mu06': "andatura normale con passo alto e attrito 0.6 (MODERATE senza crawl)",
    'auto_alto_mu04': "andatura normale con passo alto e attrito 0.4 (HARD senza crawl)",
    'crawl_mu08': "crawl con attrito 0.8 (separa l'effetto del crawl da quello dell'attrito)",
    'missione': "come la missione: livello scelto dalla pendenza stimata prima di partire",
}
CARTELLA = 'ProveAndatura'
COLONNE = ['timestamp', 'terreno', 'verso', 'config', 'ripetizione', 'pendenza_vera_gradi',
           'stima_long', 'stima_lat', 'stima_rough', 'stima_has_data', 'livello_missione',
           'locomotion_hint', 'swing_height', 'ground_mu_hint', 'max_linear_vel',
           'success', 'duration_s', 'dist_commanded_m', 'dist_traveled_m', 'speed_mps',
           'pitch_mean_deg', 'pitch_peak_abs_deg', 'pitch_rms_deg',
           'roll_mean_deg', 'roll_peak_abs_deg', 'roll_rms_deg',
           'slip_peak_m', 'slip_mean_m', 'mu_est_mean', 'mu_est_min',
           'energy_j', 'energy_per_m_j', 'charge_start_pct', 'charge_end_pct',
           'samples', 'note']
# Fin dove la scansione vede davanti al robot (la finestra locale e' 3.84 m, robot al centro).
STIMA_MAX_M = 1.8


class Campionatore:
    """Stato del robot ogni period_s in un thread: posa, inclinazione, piedi, potenza."""

    def __init__(self, robot_state_client, period_s=0.1):
        self._c = robot_state_client
        self._period = period_s
        self._stop = threading.Event()
        self._thread = None
        self.righe = []   # t, x, y, yaw, roll, pitch, slip_max, mu_mean, potenza_w

    def _uno(self):
        try:
            st = self._c.get_robot_state()
        except Exception:
            return
        t = time.time()
        try:
            x, y, _, q = spotUtils.getPosition(self._c)
            roll, pitch = _quat_to_roll_pitch_deg(q)
            yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        except Exception:
            return
        slip, mu = [], []
        try:
            for fs in st.foot_state:
                if fs.contact != robot_state_pb2.FootState.CONTACT_MADE:
                    continue
                d = fs.terrain.foot_slip_distance_rt_frame
                slip.append(float(np.sqrt(d.x * d.x + d.y * d.y + d.z * d.z)))
                if fs.terrain.ground_mu_est > 0:
                    mu.append(float(fs.terrain.ground_mu_est))
        except Exception:
            pass
        potenza = np.nan
        try:
            b = st.battery_states[0]
            if b.HasField('voltage') and b.HasField('current'):
                potenza = abs(float(b.voltage.value) * float(b.current.value))
        except Exception:
            pass
        self.righe.append((t, x, y, yaw, roll, pitch,
                           max(slip) if slip else np.nan,
                           float(np.mean(mu)) if mu else np.nan, potenza))

    def _run(self):
        while not self._stop.wait(self._period):
            self._uno()

    def start(self):
        self._uno()
        self._thread = threading.Thread(target=self._run, name='campionatore', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._uno()
        return np.array(self.righe, dtype=float) if self.righe else np.zeros((0, 9))


def carica(robot_state_client):
    try:
        b = robot_state_client.get_robot_state().battery_states[0]
        return float(b.charge_percentage.value)
    except Exception:
        return np.nan


def stima_tracciato(local_grid, robot_state_client, dist):
    """Pendenza (col corpo) e rugosita' del tracciato davanti al robot, dalla scansione di
    adesso, con lo stesso calcolo del PRM. Vede solo i primi STIMA_MAX_M metri."""
    vuota = dict(long=np.nan, lat=np.nan, rough=np.nan, has=False)
    try:
        scan = local_grid.fetch_scan(robot_state_client)
        if scan is None:
            print("[STIMA] Scansione non arrivata.")
            return vuota
        x, y, _, q = spotUtils.getPosition(robot_state_client)
        yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        local_grid.process_scan(scan, x, y, yaw)
        shape = (scan.num_y, scan.num_x)
        terrain_2d = scan.terrain.reshape(shape)
        valid_2d = np.asarray(scan.is_valid).reshape(shape)
        obst_2d = (np.asarray(scan.obstacle_mask).reshape(shape)
                   if scan.obstacle_mask is not None else None)
        ox, oy = local_grid.grid_origin(scan)
        L = min(dist, STIMA_MAX_M)
        x2, y2 = x + L * np.cos(yaw), y + L * np.sin(yaw)
        prof = spotGrid.compute_arc_slope_profile(
            terrain_2d, ox, oy, scan.cell_size, x, y, x2, y2, valid_2d=valid_2d,
            obstacle_2d=obst_2d, body_ext_m=spotGrid.ARC_BODY_EXTENSION_M)
        lo, la, has = prm_graph.body_slope(prof)
        t = np.linspace(0.0, 1.0, max(2, int(L / scan.cell_size) + 1))
        r, inside = spotGrid._gather_heights(np.asarray(scan.rough).reshape(shape), ox, oy,
                                             scan.cell_size, x + t * (x2 - x), y + t * (y2 - y),
                                             valid_2d=valid_2d)
        rough = float(np.nanmean(r[inside])) if inside.any() else np.nan
        print(f"[STIMA] Primi {L:.1f} m: pendenza long {lo:.3f} ({np.degrees(np.arctan(lo)):.1f}°), "
              f"lat {la:.3f} ({np.degrees(np.arctan(la)):.1f}°), rugosita' {rough:.3f}, "
              f"{'con dati' if has else 'SENZA DATI SUFFICIENTI'}")
        return dict(long=float(lo), lat=float(la), rough=rough, has=bool(has))
    except Exception as e:
        print(f"[STIMA] Stima non riuscita: {e}")
        return vuota


def riassunto(a, dist_cmd, dist_trav, durata):
    """Dalle righe del campionatore ai numeri della riga di prove.csv."""
    if len(a) == 0:
        return {}
    roll, pitch, slip, mu, pot = a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8]
    nm = lambda v, f: float(f(v[np.isfinite(v)])) if np.isfinite(v).any() else np.nan
    energia = np.nan
    if np.isfinite(pot).sum() >= 2:
        ok = np.isfinite(pot)
        p_, t_ = pot[ok], a[ok, 0]
        energia = float(np.sum(0.5 * (p_[1:] + p_[:-1]) * np.diff(t_)))
    return {
        'pitch_mean_deg': nm(pitch, np.mean), 'pitch_peak_abs_deg': nm(np.abs(pitch), np.max),
        'pitch_rms_deg': nm(pitch - np.nanmean(pitch), lambda v: np.sqrt(np.mean(v * v))),
        'roll_mean_deg': nm(roll, np.mean), 'roll_peak_abs_deg': nm(np.abs(roll), np.max),
        'roll_rms_deg': nm(roll - np.nanmean(roll), lambda v: np.sqrt(np.mean(v * v))),
        'slip_peak_m': nm(slip, np.max), 'slip_mean_m': nm(slip, np.mean),
        'mu_est_mean': nm(mu, np.mean), 'mu_est_min': nm(mu, np.min),
        'energy_j': energia,
        'energy_per_m_j': energia / dist_trav if dist_trav > 0.1 and np.isfinite(energia) else np.nan,
        'speed_mps': dist_trav / durata if durata > 0 else np.nan,
        'samples': len(a),
    }


def scrivi_riga(path, riga):
    nuovo = not os.path.exists(path)
    with open(path, 'a', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=COLONNE, extrasaction='ignore')
        if nuovo:
            w.writeheader()
        w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in riga.items()})


def passaggio(a, verso, rip, robot_state_client, command_client, local_grid, csv_path):
    stima = stima_tracciato(local_grid, robot_state_client, a.dist)
    livello = ''
    if a.config == 'missione':
        if not np.isfinite(stima['long']):
            print("[PROVA] Config 'missione' senza stima della pendenza: passaggio saltato.")
            return
        params = movements.select_mobility_params(max(stima['long'], stima['lat']),
                                                  stima['rough'] if np.isfinite(stima['rough']) else 0.0)
        livello = movements.MISSION_STATS['last_tier']
    else:
        params = dict(CONFIG[a.config])
        if np.isfinite(stima['long']):
            g = max(stima['long'], stima['lat'])
            r = stima['rough'] if np.isfinite(stima['rough']) else 0.0
            livello = ('PLAIN' if g < spotGrid.GAIT_PLAIN_SLOPE_MAX and r < spotGrid.GAIT_PLAIN_ROUGH_MAX
                       else 'MODERATE' if g < spotGrid.GAIT_MODERATE_SLOPE_MAX
                       and r < spotGrid.GAIT_MODERATE_ROUGH_MAX else 'HARD')
    if a.vel is not None:
        params['max_linear_vel'] = a.vel

    c0 = carica(robot_state_client)
    camp = Campionatore(robot_state_client)
    camp.start()
    t0 = time.time()
    ok, dist_trav = movements.relative_move(a.dist, 0.0, 0.0, VISION_FRAME_NAME, command_client,
                                            robot_state_client, **params)
    durata = time.time() - t0
    arr = camp.stop()
    riga = dict(timestamp=time.strftime('%Y-%m-%d %H:%M:%S'), terreno=a.terreno, verso=verso,
                config=a.config, ripetizione=rip,
                pendenza_vera_gradi=(a.pendenza_vera if verso == 'andata' or a.pendenza_vera is None
                                     else -a.pendenza_vera),
                stima_long=stima['long'], stima_lat=stima['lat'], stima_rough=stima['rough'],
                stima_has_data=stima['has'], livello_missione=livello,
                locomotion_hint=params.get('locomotion_hint'), swing_height=params.get('swing_height'),
                ground_mu_hint=params.get('ground_mu_hint'), max_linear_vel=params.get('max_linear_vel'),
                success=bool(ok), duration_s=durata, dist_commanded_m=float(a.dist),
                dist_traveled_m=float(dist_trav), charge_start_pct=c0,
                charge_end_pct=carica(robot_state_client), note=a.note or '')
    riga.update(riassunto(arr, a.dist, float(dist_trav), durata))
    scrivi_riga(csv_path, riga)
    stamp = time.strftime('%H-%M-%S')
    np.savez(os.path.join(a.cartella, f"{a.terreno}_{a.config}_{verso}_{stamp}.npz"), campioni=arr,
             colonne=np.array(['t', 'x', 'y', 'yaw', 'roll', 'pitch', 'slip_max', 'mu_est', 'potenza_w']))
    print(f"[PROVA] {verso} {rip}: {'RIUSCITO' if ok else 'NON RIUSCITO'}, {dist_trav:.2f}/{a.dist:.2f} m "
          f"in {durata:.1f} s ({riga.get('speed_mps', np.nan):.2f} m/s), beccheggio medio "
          f"{riga.get('pitch_mean_deg', np.nan):.1f}° RMS {riga.get('pitch_rms_deg', np.nan):.2f}°, "
          f"rollio RMS {riga.get('roll_rms_deg', np.nan):.2f}°, slittamento picco "
          f"{riga.get('slip_peak_m', np.nan):.3f} m, attrito stimato {riga.get('mu_est_mean', np.nan):.2f}, "
          f"{riga.get('energy_per_m_j', np.nan):.0f} J/m")


def gira(command_client, robot_state_client, params):
    p = {k: v for k, v in params.items() if k != 'max_linear_vel'}
    ok, _ = movements.relative_move(0.0, 0.0, np.pi, VISION_FRAME_NAME, command_client,
                                    robot_state_client, **p)
    return ok


def chiedi(msg):
    r = input(msg).strip().lower()
    return r


def main():
    p = argparse.ArgumentParser(description="Prova d'andatura su tracciato rettilineo.")
    p.add_argument('--terreno', help="nome del tracciato, es. prato, rampa10, rampa15, laterale, zolle")
    p.add_argument('--pendenza-vera', type=float, default=None,
                   help="pendenza misurata con l'inclinometro, in gradi, nel verso di andata")
    p.add_argument('--config', default='missione', choices=sorted(CONFIG) + ['missione'])
    p.add_argument('--dist', type=float, default=5.0, help="lunghezza del tracciato (m, max 10)")
    p.add_argument('--ripetizioni', type=int, default=3)
    p.add_argument('--ritorno', action='store_true',
                   help="alla fine di ogni andata il robot si gira e torna: anche il verso opposto "
                        "(discesa su una rampa) viene misurato")
    p.add_argument('--vel', type=float, default=None, help="limite di velocita' (m/s) al posto di "
                                                          "quello della configurazione")
    p.add_argument('--solo-stima', action='store_true',
                   help="il robot si alza e stima la pendenza davanti, senza camminare")
    p.add_argument('--note', default='')
    p.add_argument('--cartella', default=CARTELLA)
    p.add_argument('--hostname', default='192.168.80.3')
    p.add_argument('--elenco', action='store_true')
    a = p.parse_args()

    if a.elenco:
        for k in sorted(DESCRIZIONE):
            print(f"  {k:15s} {DESCRIZIONE[k]}")
        return
    if not a.terreno:
        p.error("serve --terreno")
    if not (0.5 <= a.dist <= 10.0):
        p.error("--dist fra 0.5 e 10 m")
    os.makedirs(a.cartella, exist_ok=True)
    csv_path = os.path.join(a.cartella, 'prove.csv')

    options = SimpleNamespace(name='provaAndatura', hostname=a.hostname, verbose=False,
                              recording_user_name='', recording_session_name='')
    robot, lease_client, robot_state_client, _ = spotLogInUtils.setLogInfo(options)
    estop = spotLogInUtils.SimpleEstop(robot, options.name + '_estop')
    local_grid = spotGrid.LocalGrid(robot)
    print(f"Terreno '{a.terreno}', configurazione '{a.config}' ({DESCRIZIONE[a.config]}), "
          f"{a.dist:.1f} m x {a.ripetizioni}{' andata e ritorno' if a.ritorno else ''}.")

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        robot.time_sync.wait_for_sync()
        robot.power_on()
        assert robot.is_powered_on(), 'Accensione del robot non riuscita.'
        try:
            blocking_stand(command_client)
            time.sleep(1.0)
            if a.solo_stima:
                for i in range(a.ripetizioni):
                    st = stima_tracciato(local_grid, robot_state_client, a.dist)
                    _, _, _, q = spotUtils.getPosition(robot_state_client)
                    roll, pitch = _quat_to_roll_pitch_deg(q)
                    scrivi_riga(csv_path, dict(
                        timestamp=time.strftime('%Y-%m-%d %H:%M:%S'), terreno=a.terreno,
                        verso='fermo', config='solo_stima', ripetizione=i + 1,
                        pendenza_vera_gradi=a.pendenza_vera, stima_long=st['long'],
                        stima_lat=st['lat'], stima_rough=st['rough'], stima_has_data=st['has'],
                        pitch_mean_deg=pitch, roll_mean_deg=roll, note=a.note or ''))
                    print(f"[FERMO] Inclinazione del corpo: beccheggio {pitch:.1f}°, rollio {roll:.1f}°")
                    time.sleep(1.0)
            else:
                for i in range(1, a.ripetizioni + 1):
                    r = chiedi(f"\nPassaggio {i}/{a.ripetizioni}: tracciato libero e robot all'inizio? "
                               f"[Invio = parti, s = salta, q = fine] ")
                    if r == 'q':
                        break
                    if r == 's':
                        continue
                    passaggio(a, 'andata', i, robot_state_client, command_client, local_grid, csv_path)
                    if a.ritorno:
                        params = dict(CONFIG.get(a.config, CONFIG['crawl_mu06']))
                        if not gira(command_client, robot_state_client, params):
                            print("[PROVA] Rotazione non riuscita: mi fermo qui.")
                            break
                        time.sleep(0.5)
                        passaggio(a, 'ritorno', i, robot_state_client, command_client, local_grid,
                                  csv_path)
                        if not gira(command_client, robot_state_client, params):
                            print("[PROVA] Rotazione non riuscita: mi fermo qui.")
                            break
            print(f"\nRighe salvate in {csv_path}. Confronto: python3 taratura.py --prove {csv_path}")
        except KeyboardInterrupt:
            print("\n[PROVA] Interrotta: il robot si siede.")
        finally:
            try:
                movements.stop_robot(command_client, reason="fine della prova")
            except Exception:
                pass
            try:
                blocking_sit(command_client, timeout_sec=10)
                robot.power_off(cut_immediately=False, timeout_sec=20)
                print("Robot seduto e motori spenti.")
            except Exception as e:
                print(f"[ATTENZIONE] Spegnimento non riuscito ({e}): usare il tablet.")
            estop.stop()


if __name__ == '__main__':
    main()
