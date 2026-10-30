"""
BATTERY: charge, consumption during the mission and early return on low battery.

  read_battery        one reading from the robot state (charge, runtime estimated by Spot,
                      voltage, current, temperature), None if the state does not contain it
  BatteryMonitor      samples the charge at fixed points of the mission (at every turn of
                      the navigation loop, before and after GraphNav moves, at the end of
                      the mission), writes it to battery.csv, derives consumption per minute
                      and per metre and decides the RETURN: below P.BATTERY_RETURN_PCT,
                      confirmed over P.BATTERY_RETURN_CONFIRM readings in a row, the mission
                      stops exploring and closes as at the end of a mission (final scan,
                      summary, map closure, return to wp_0 with GraphNav, saving).

The monitor does not move the robot and never raises to its caller: a failed reading
counts as "charge unknown" and does not trigger the return. Readings are made in the
mission thread, at most one every P.BATTERY_SAMPLE_S seconds (except labelled ones,
which are always made): no extra thread.

The distance travelled is the sum of the straight-line moves between one reading and the
next (VISION pose): on walked legs it is almost exact, on long GraphNav moves it
underestimates, so the consumption per metre is an overestimate.
"""
import time

import numpy as np

from spot_explorer import config as P
from spot_explorer.robot import connection
from spot_explorer.records import MISSION_STATS, append_csv

_BATTERY_COLUMNS = ['t_s', 'wall_time', 'charge_pct', 'spot_runtime_min', 'voltage_v',
                    'current_a', 'temp_max_c', 'dist_m', 'x', 'y', 'where', 'pct_per_min',
                    'pct_per_m', 'below_threshold']


def read_battery(robot_state_client):
    """One battery reading: dict with charge_pct (0-100) and, when available,
    spot_runtime_min (runtime estimated by Spot), voltage_v, current_a, temp_max_c.
    None if the robot state does not report the charge. Never raises."""
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
        # Fallback: the "locomotion" charge in the power state.
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


class BatteryMonitor:
    """Mission charge and consumption, and the decision to return (see the module).

    check(where)      one reading (if enough time has passed, or always with where);
                      True if the mission must return
    return_due        True once the threshold has been crossed (never goes back to False)
    """

    def __init__(self, robot_state_client, threshold_pct=None, confirmations=None, interval_s=None):
        self.robot_state_client = robot_state_client
        self.threshold_pct = P.BATTERY_RETURN_PCT if threshold_pct is None else threshold_pct
        self.confirmations = P.BATTERY_RETURN_CONFIRM if confirmations is None else confirmations
        self.interval_s = P.BATTERY_SAMPLE_S if interval_s is None else interval_s
        self.return_due = False
        self.reason_txt = None
        self._t0 = time.time()
        self._t_last = None
        self._xy_last = None
        self._dist_m = 0.0
        self._samples = []         # (t_s, charge, distance)
        self._below_count = 0            # readings in a row below the threshold
        self._printed_pct = None  # last whole charge printed in the log
        self._failed_reads = 0

    # --- Consumption ------------------------------------------------------------------

    def consumption(self):
        """(percent per minute, percent per metre) from the first reading to the last, positive
        when the charge drops; None where it cannot be told yet."""
        if len(self._samples) < 2:
            return None, None
        t0, c0, d0 = self._samples[0]
        t1, c1, d1 = self._samples[-1]
        per_min = (c0 - c1) / ((t1 - t0) / 60.0) if t1 - t0 >= 60.0 else None
        per_m = (c0 - c1) / (d1 - d0) if d1 - d0 >= 5.0 else None
        return per_min, per_m

    def minutes_to_threshold(self):
        """Estimated minutes before reaching the threshold, at the average consumption so far."""
        per_min, _ = self.consumption()
        if not self._samples or not per_min or per_min <= 0:
            return None
        return max(0.0, (self._samples[-1][1] - self.threshold_pct) / per_min)

    # --- Reading -----------------------------------------------------------------------

    def check(self, where=None):
        """One battery reading. Without a label it is skipped if the last one is more recent
        than interval_s. True if the mission must return. Never raises."""
        try:
            now = time.time()
            if where is None and self._t_last is not None and now - self._t_last < self.interval_s:
                return self.return_due
            self._t_last = now
            try:
                x, y, _, _ = connection.getPosition(self.robot_state_client)
                if self._xy_last is not None:
                    self._dist_m += float(np.hypot(x - self._xy_last[0], y - self._xy_last[1]))
                self._xy_last = (x, y)
            except Exception:
                x, y = float('nan'), float('nan')
            b = read_battery(self.robot_state_client)
            if b is None:
                self._failed_reads += 1
                MISSION_STATS['battery_reads_failed'] = self._failed_reads
                if self._failed_reads == 1:
                    print("[BATTERY] Charge not available in the robot state: no "
                          "low-battery return until it can be read.")
                return self.return_due
            self._record(now, b, x, y, where or 'loop')
            return self.return_due
        except Exception as e:
            print(f"[BATTERY] Reading failed ({type(e).__name__}: {e}).")
            return self.return_due

    def _record(self, now, b, x, y, where):
        c = b['charge_pct']
        t_s = now - self._t0
        self._samples.append((t_s, c, self._dist_m))
        per_min, per_m = self.consumption()
        below = self.threshold_pct > 0 and c <= self.threshold_pct
        self._below_count = self._below_count + 1 if below else 0

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
                     '' if per_m is None else round(per_m, 3), int(below)]])

        # In the log only when the charge drops by a whole point, or with a label.
        pct_int = int(np.floor(c))
        if where != 'loop' or self._printed_pct is None or pct_int < self._printed_pct:
            self._printed_pct = pct_int
            print(f"[BATTERY] {where}: {self.describe(b)}")

        if below and not self.return_due and self._below_count >= self.confirmations:
            self.return_due = True
            self.reason_txt = (f"charge {c:.0f}% <= threshold {self.threshold_pct:.0f}% "
                           f"({self._below_count} readings in a row)")
            MISSION_STATS['battery_return'] = True
            MISSION_STATS['battery_return_pct'] = c
            MISSION_STATS['battery_return_min'] = round(t_s / 60.0, 1)
            print(f"[BATTERY] RETURN: {self.reason_txt}. I stop exploring and close the mission "
                  f"as at the end of exploration: final scan, saving, return to wp_0 with "
                  f"GraphNav.")

    def describe(self, b=None):
        """One readable line: charge, consumption, runtime estimated by Spot, minutes to the threshold."""
        if not self._samples:
            return "no reading"
        c = self._samples[-1][1]
        per_min, per_m = self.consumption()
        parts = [f"charge {c:.0f}%"]
        if per_min is not None:
            parts.append(f"{per_min:.2f} %/min")
        if per_m is not None:
            parts.append(f"{per_m:.3f} %/m over {self._dist_m:.0f} m")
        if b is not None and b.get('spot_runtime_min') is not None:
            parts.append(f"runtime estimated by Spot {b['spot_runtime_min']:.0f} min")
        m = self.minutes_to_threshold()
        if m is not None and not self.return_due:
            parts.append(f"~{m:.0f} min to the {self.threshold_pct:.0f}% threshold")
        return ", ".join(parts)

    def summary(self):
        """The end-of-mission summary line."""
        if not self._samples:
            return "Battery: charge never read" + (
                f" ({self._failed_reads} failed readings)" if self._failed_reads else "")
        c0, c1 = self._samples[0][1], self._samples[-1][1]
        line = (f"Battery: from {c0:.0f}% to {c1:.0f}% in {self._samples[-1][0] / 60.0:.1f} min and "
                f"{self._dist_m:.0f} m; {self.describe()}")
        if self.return_due:
            line += f" | EARLY RETURN: {self.reason_txt}"
        return line + " (details in battery.csv)"
