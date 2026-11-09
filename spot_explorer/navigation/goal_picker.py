"""
Choosing on the map with the mouse (ClickGoalPicker), in a matplotlib window.

  goal mode   press on the goal (the path is planned at once); drag before releasing to give
              the arrival heading (the arrow); Enter confirms, p saves it as a named place, h
              goes home, Esc quits
  start mode  where the robot is now, for --start-at click: press on its position and drag in
              the direction it faces (the heading is needed); Enter confirms
"""
import os
import sys

import numpy as np

from spot_explorer.navigation.mission_map import LOG_TAG, describe_plan, heading_arrow
from spot_explorer.navigation.navigator import HOME

# A drag shorter than this (m, on the map) is a plain click: no heading.
MIN_DRAG_M = 0.3


class ClickGoalPicker:
    """
    Goal mode: left press = goal at that point, with the path planned at once (another press
    moves it); drag while pressed = arrival heading; Enter = confirm; p = save the goal as a
    named place (the name is asked in the terminal); h = go home (wp_0); Esc, q or window
    closed = nothing. Start mode (start=True): press on the robot and drag in its direction.
    With the zoom or pan of the toolbar active, the clicks are theirs and choose nothing.

    from_xy: start of the plan (mission VISION): the robot now, or the mission start.
    ask: how to ask for the name of a place (input; the tests pass their own).
    """

    def __init__(self, mission_map, from_xy=None, title="", from_label='start', start=False, ask=input):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.map, self.from_xy = mission_map, from_xy
        self.start_mode = start
        self.ask = ask
        self.title = title or f"Mission {mission_map.name}"
        self.goal = self.plan = None
        self.goal_m = None                  # (xm, ym) of the goal in the mission frame
        self.heading_deg = None
        self.confirmed = False
        self.home = False
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        mission_map.draw_map(self.ax)
        self._artists = mission_map.draw_plan(self.ax, None, from_xy, None, from_label=from_label) if from_xy else []
        self.from_label = from_label
        self.ax.legend(loc='upper right', fontsize=8)
        self._set_title("press on the robot and drag in the direction it faces | Enter: confirm | Esc: quit"
                        if start else
                        "press: goal (drag: arrival heading) | Enter: confirm | p: save as a place | "
                        "h: go home | Esc: quit")
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect('button_press_event', self._press)
        self.fig.canvas.mpl_connect('button_release_event', self._release)
        self.fig.canvas.mpl_connect('key_press_event', self._key)

    def _set_title(self, line):
        self.ax.set_title(f"{self.title}\n{line}", fontsize=10)

    def _tool_active(self):
        tb = getattr(getattr(self.fig.canvas, 'manager', None), 'toolbar', None)
        return bool(getattr(tb, 'mode', ''))

    def _usable(self, ev):
        return ev.inaxes is self.ax and ev.button == 1 and ev.xdata is not None and not self._tool_active()

    def _press(self, ev):
        if not self._usable(ev):
            return
        self.goal_m = (float(ev.ydata), float(ev.xdata))          # (y, x) in the figure
        self.heading_deg = None
        self._update()

    def _release(self, ev):
        if self.goal_m is None or ev.inaxes is not self.ax or ev.xdata is None or self._tool_active():
            return
        dx, dy = float(ev.ydata) - self.goal_m[0], float(ev.xdata) - self.goal_m[1]
        if np.hypot(dx, dy) >= MIN_DRAG_M:
            self.heading_deg = float(np.degrees(np.arctan2(dy, dx)))
            self._update()

    def _update(self):
        xm, ym = self.goal_m
        self.goal = self.map.frame.to_vision(xm, ym)
        for art in self._artists:
            art.remove()
        h = "" if self.heading_deg is None else f", heading {self.heading_deg:+.0f} degrees"
        if self.start_mode:
            self._artists = self.map.draw_plan(self.ax, None, self.goal, None, from_label='robot (your guess)')
            if self.heading_deg is not None:
                self._artists.append(heading_arrow(self.ax, xm, ym, self.heading_deg, 'k'))
            self._set_title(f"robot at ({xm:.2f}, {ym:.2f}){h}\n"
                            + ("Enter: confirm | press again: another point | Esc: quit" if self.heading_deg is not None
                               else "drag in the direction the robot faces | Esc: quit"))
        else:
            self.plan = self.map.plan(self.from_xy, self.goal)
            self._artists = self.map.draw_plan(self.ax, self.plan, self.from_xy, self.goal,
                                               from_label=self.from_label, heading_deg=self.heading_deg)
            text = describe_plan(self.plan, self.from_xy, self.goal)
            print(f"{LOG_TAG} Goal at ({xm:.2f}, {ym:.2f}){h} in the mission frame: {text}.")
            self._set_title(f"goal ({xm:.2f}, {ym:.2f}){h}: {text}\n"
                            + ("Enter: confirm | p: save as a place | press: another point | Esc: quit"
                               if self.plan is not None else "press: another point | Esc: quit"))
        self.ax.legend(loc='upper right', fontsize=8)
        self.fig.canvas.draw_idle()

    def _key(self, ev):
        if ev.key == 'enter' and self.goal is not None and (
                (self.start_mode and self.heading_deg is not None) or (not self.start_mode and self.plan is not None)):
            self.confirmed = True
            self.plt.close(self.fig)
        elif ev.key == 'p' and not self.start_mode and self.goal_m is not None:
            self._save_place()
        elif ev.key == 'h' and not self.start_mode:
            self.home = True
            self.plt.close(self.fig)
        elif ev.key in ('escape', 'q'):
            self.confirmed = False
            self.plt.close(self.fig)

    def _save_place(self):
        try:
            name = self.ask(f"{LOG_TAG} Name of the place at ({self.goal_m[0]:.2f}, {self.goal_m[1]:.2f}): ").strip()
            self.map.places.add(name, *self.goal_m, heading_deg=self.heading_deg)
        except (ValueError, EOFError) as e:
            print(f"{LOG_TAG} Place not saved: {e}")
            return
        print(f"{LOG_TAG} Place '{name}' saved.")
        from spot_explorer.navigation.mission_map import draw_places
        draw_places(self.ax, {name: self.map.places.places[name]})
        self.fig.canvas.draw_idle()

    def choose(self):
        """Opens the window and waits. Goal mode: (goal VISION, description, heading in the
        mission frame or None), HOME or None. Start mode: (xm, ym, heading_deg) or None."""
        self.plt.show()
        if self.home:
            return HOME
        if not self.confirmed:
            return None
        xm, ym = self.goal_m
        if self.start_mode:
            return xm, ym, self.heading_deg
        return (self.goal, f"({xm:.2f}, {ym:.2f}) in the mission frame, chosen with a click",
                self.heading_deg)

    def save(self, png):
        self.fig.savefig(png, dpi=130)


def window_available(what="--click"):
    """A backend with windows is needed: on Linux without DISPLAY matplotlib opens none."""
    import matplotlib
    if sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print(f"{LOG_TAG} {what} opens a window, but there is no screen here (DISPLAY empty).")
        return False
    if matplotlib.get_backend().lower() in ('agg', 'pdf', 'svg', 'ps', 'cairo', 'template'):
        print(f"{LOG_TAG} {what} opens a window, but matplotlib uses the "
              f"'{matplotlib.get_backend()}' backend, without windows.")
        return False
    return True
