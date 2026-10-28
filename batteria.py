"""
BATTERIA: carica, consumo durante la missione e rientro anticipato a batteria bassa.

  leggi_batteria      una lettura dallo stato del robot (carica, autonomia stimata da Spot,
                      tensione, corrente, temperatura), None se lo stato non la contiene
  MonitorBatteria     campiona la carica nei punti fissi della missione (a ogni giro del
                      ciclo di navigazione, prima e dopo gli spostamenti GraphNav, a fine
                      missione), la scrive in battery.csv, ricava il consumo per minuto e per
                      metro e decide il RIENTRO: sotto P.BATTERY_RETURN_PCT, confermato su
                      P.BATTERY_RETURN_CONFIRM letture di fila, la missione smette di
                      esplorare e chiude come a fine missione (scansione finale, riepilogo,
                      chiusura della mappa, ritorno a wp_0 con GraphNav, salvataggi).

Il monitor non muove il robot e non solleva mai eccezioni verso chi lo chiama: una lettura
che fallisce vale "carica sconosciuta" e non fa scattare il rientro. Le letture si fanno
nel thread della missione, al massimo una ogni P.BATTERY_SAMPLE_S secondi (tranne quelle
con etichetta, che si fanno sempre): niente thread in piu'.

La distanza percorsa e' la somma degli spostamenti in linea retta fra una lettura e la
successiva (posa VISION): sui tratti a zampe e' quasi esatta, sugli spostamenti GraphNav
lunghi la sottostima, quindi il consumo per metro e' per eccesso.
"""
import time

import numpy as np

import parametri as P
import spotUtils
from registro import MISSION_STATS, append_csv

_BATTERY_COLUMNS = ['t_s', 'wall_time', 'charge_pct', 'spot_runtime_min', 'voltage_v',
                    'current_a', 'temp_max_c', 'dist_m', 'x', 'y', 'where', 'pct_per_min',
                    'pct_per_m', 'below_threshold']


def leggi_batteria(robot_state_client):
    """Una lettura della batteria: dict con charge_pct (0-100) e, quando ci sono,
    spot_runtime_min (autonomia stimata da Spot), voltage_v, current_a, temp_max_c.
    None se lo stato del robot non dice la carica. Mai eccezioni."""
    try:
        st = robot_state_client.get_robot_state()
    except Exception:
        return None
    try:
        out = {}
        bats = list(getattr(st, 'battery_states', []) or [])
        if bats and bats[0].HasField('charge_percentage'):
            b = bats[0]
            out['charge_pct'] = float(b.charge_percentage.value)
            if b.HasField('estimated_runtime'):
                out['spot_runtime_min'] = (b.estimated_runtime.seconds
                                           + b.estimated_runtime.nanos * 1e-9) / 60.0
            if b.HasField('voltage'):
                out['voltage_v'] = float(b.voltage.value)
            if b.HasField('current'):
                out['current_a'] = float(b.current.value)
            if len(b.temperatures):
                out['temp_max_c'] = float(max(b.temperatures))
            return out
        # Ripiego: la carica "di locomozione" nello stato di alimentazione.
        ps = st.power_state
        if ps.HasField('locomotion_charge_percentage'):
            out['charge_pct'] = float(ps.locomotion_charge_percentage.value)
            if ps.HasField('locomotion_estimated_runtime'):
                rt = ps.locomotion_estimated_runtime
                out['spot_runtime_min'] = (rt.seconds + rt.nanos * 1e-9) / 60.0
            return out
    except Exception:
        pass
    return None


class MonitorBatteria:
    """Carica e consumo della missione, e la decisione di rientrare (vedi il modulo).

    controlla(where)  una lettura (se e' passato abbastanza tempo, o sempre con where);
                      True se la missione deve rientrare
    rientro           True da quando la soglia e' stata superata (non torna mai False)
    """

    def __init__(self, robot_state_client, soglia_pct=None, conferme=None, intervallo_s=None):
        self.robot_state_client = robot_state_client
        self.soglia_pct = P.BATTERY_RETURN_PCT if soglia_pct is None else soglia_pct
        self.conferme = P.BATTERY_RETURN_CONFIRM if conferme is None else conferme
        self.intervallo_s = P.BATTERY_SAMPLE_S if intervallo_s is None else intervallo_s
        self.rientro = False
        self.motivo = None
        self._t0 = time.time()
        self._t_last = None
        self._xy_last = None
        self._dist_m = 0.0
        self._samples = []         # (t_s, carica, distanza)
        self._sotto = 0            # letture di fila sotto la soglia
        self._pct_stampata = None  # ultima carica intera stampata nel log
        self._fallite = 0

    # --- Consumi ---------------------------------------------------------------------

    def consumi(self):
        """(percento al minuto, percento al metro) dalla prima lettura all'ultima, positivi
        quando la carica scende; None dove non si puo' ancora dire."""
        if len(self._samples) < 2:
            return None, None
        t0, c0, d0 = self._samples[0]
        t1, c1, d1 = self._samples[-1]
        per_min = (c0 - c1) / ((t1 - t0) / 60.0) if t1 - t0 >= 60.0 else None
        per_m = (c0 - c1) / (d1 - d0) if d1 - d0 >= 5.0 else None
        return per_min, per_m

    def minuti_alla_soglia(self):
        """Minuti stimati prima di arrivare alla soglia, al consumo medio finora."""
        per_min, _ = self.consumi()
        if not self._samples or not per_min or per_min <= 0:
            return None
        return max(0.0, (self._samples[-1][1] - self.soglia_pct) / per_min)

    # --- Lettura ---------------------------------------------------------------------

    def controlla(self, where=None):
        """Una lettura della batteria. Senza etichetta si salta se l'ultima e' piu' recente
        di intervallo_s. True se la missione deve rientrare. Mai eccezioni."""
        try:
            now = time.time()
            if where is None and self._t_last is not None and now - self._t_last < self.intervallo_s:
                return self.rientro
            self._t_last = now
            try:
                x, y, _, _ = spotUtils.getPosition(self.robot_state_client)
                if self._xy_last is not None:
                    self._dist_m += float(np.hypot(x - self._xy_last[0], y - self._xy_last[1]))
                self._xy_last = (x, y)
            except Exception:
                x, y = float('nan'), float('nan')
            b = leggi_batteria(self.robot_state_client)
            if b is None:
                self._fallite += 1
                MISSION_STATS['battery_reads_failed'] = self._fallite
                if self._fallite == 1:
                    print("[BATTERIA] Carica non disponibile nello stato del robot: niente "
                          "rientro a batteria bassa finche' non si legge.")
                return self.rientro
            self._registra(now, b, x, y, where or 'ciclo')
            return self.rientro
        except Exception as e:
            print(f"[BATTERIA] Lettura non riuscita ({type(e).__name__}: {e}).")
            return self.rientro

    def _registra(self, now, b, x, y, where):
        c = b['charge_pct']
        t_s = now - self._t0
        self._samples.append((t_s, c, self._dist_m))
        per_min, per_m = self.consumi()
        sotto = self.soglia_pct > 0 and c <= self.soglia_pct
        self._sotto = self._sotto + 1 if sotto else 0

        MISSION_STATS['battery_samples'] = len(self._samples)
        MISSION_STATS.setdefault('battery_start_pct', c)
        MISSION_STATS['battery_last_pct'] = c
        MISSION_STATS['battery_min_pct'] = min(MISSION_STATS.get('battery_min_pct', c), c)
        MISSION_STATS['battery_dist_m'] = round(self._dist_m, 1)
        MISSION_STATS['battery_elapsed_min'] = round(t_s / 60.0, 1)
        MISSION_STATS['battery_pct_per_min'] = None if per_min is None else round(per_min, 3)
        MISSION_STATS['battery_pct_per_m'] = None if per_m is None else round(per_m, 3)

        append_csv("battery.csv", _BATTERY_COLUMNS,
                   [[round(t_s, 1), round(now, 1), round(c, 1), b.get('spot_runtime_min', ''),
                     b.get('voltage_v', ''), b.get('current_a', ''), b.get('temp_max_c', ''),
                     round(self._dist_m, 2), x, y, where,
                     '' if per_min is None else round(per_min, 3),
                     '' if per_m is None else round(per_m, 3), int(sotto)]])

        # Nel log solo quando la carica scende di un punto intero, o con un'etichetta.
        pct_int = int(np.floor(c))
        if where != 'ciclo' or self._pct_stampata is None or pct_int < self._pct_stampata:
            self._pct_stampata = pct_int
            print(f"[BATTERIA] {where}: {self.descrivi(b)}")

        if sotto and not self.rientro and self._sotto >= self.conferme:
            self.rientro = True
            self.motivo = (f"carica {c:.0f}% <= soglia {self.soglia_pct:.0f}% "
                           f"({self._sotto} letture di fila)")
            MISSION_STATS['battery_return'] = True
            MISSION_STATS['battery_return_pct'] = c
            MISSION_STATS['battery_return_min'] = round(t_s / 60.0, 1)
            print(f"[BATTERIA] RIENTRO: {self.motivo}. Smetto di esplorare e chiudo la missione "
                  f"come a fine esplorazione: scansione finale, salvataggi, ritorno a wp_0 con "
                  f"GraphNav.")

    def descrivi(self, b=None):
        """Una riga leggibile: carica, consumi, autonomia stimata da Spot, minuti alla soglia."""
        if not self._samples:
            return "nessuna lettura"
        c = self._samples[-1][1]
        per_min, per_m = self.consumi()
        parts = [f"carica {c:.0f}%"]
        if per_min is not None:
            parts.append(f"{per_min:.2f} %/min")
        if per_m is not None:
            parts.append(f"{per_m:.3f} %/m su {self._dist_m:.0f} m")
        if b is not None and b.get('spot_runtime_min') is not None:
            parts.append(f"autonomia stimata da Spot {b['spot_runtime_min']:.0f} min")
        m = self.minuti_alla_soglia()
        if m is not None and not self.rientro:
            parts.append(f"~{m:.0f} min alla soglia del {self.soglia_pct:.0f}%")
        return ", ".join(parts)

    def riepilogo(self):
        """La riga del riepilogo di fine missione."""
        if not self._samples:
            return "Batteria: carica mai letta" + (
                f" ({self._fallite} letture fallite)" if self._fallite else "")
        c0, c1 = self._samples[0][1], self._samples[-1][1]
        line = (f"Batteria: da {c0:.0f}% a {c1:.0f}% in {self._samples[-1][0] / 60.0:.1f} min e "
                f"{self._dist_m:.0f} m; {self.descrivi()}")
        if self.rientro:
            line += f" | RIENTRO ANTICIPATO: {self.motivo}"
        return line + " (dettaglio in battery.csv)"
