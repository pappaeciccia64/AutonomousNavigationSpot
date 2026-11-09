"""
The goal chosen with a click on the map (ClickGoalPicker), in a matplotlib window.
"""
import os
import sys

from spot_explorer.navigation.mission_map import LOG_TAG, describe_plan
from spot_explorer.navigation.navigator import HOME


class ClickGoalPicker:
    """
    Left click: goal at that point, with the path planned at once (another click moves it);
    Enter: confirm; h: go home (wp_0); Esc, q or window closed: no goal. With the zoom or pan
    of the toolbar active, the clicks are theirs and choose nothing.

    from_xy: start of the plan (mission VISION): the robot now, or the mission start.
    """

    def __init__(self, mission_map, from_xy, title="", from_label='start'):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.map, self.from_xy = mission_map, from_xy
        self.title = title or f"Mission {mission_map.name}"
        self.goal = self.plan = None
        self.confirmed = False
        self.home = False
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        mission_map.draw_map(self.ax)
        self._artists = mission_map.draw_plan(self.ax, None, from_xy, None, from_label=from_label)
        self.from_label = from_label
        self.ax.legend(loc='upper right', fontsize=8)
        self._set_title("left click: goal | Enter: confirm | h: go home | Esc: quit")
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect('button_press_event', self._click)
        self.fig.canvas.mpl_connect('key_press_event', self._key)

    def _set_title(self, line):
        self.ax.set_title(f"{self.title}\n{line}", fontsize=10)

    def _tool_active(self):
        tb = getattr(getattr(self.fig.canvas, 'manager', None), 'toolbar', None)
        return bool(getattr(tb, 'mode', ''))

    def _click(self, ev):
        if ev.inaxes is not self.ax or ev.button != 1 or ev.xdata is None or self._tool_active():
            return
        xm, ym = float(ev.ydata), float(ev.xdata)          # (y, x) in the figure
        self.goal = self.map.frame.to_vision(xm, ym)
        self.plan = self.map.plan(self.from_xy, self.goal)
        for art in self._artists:
            art.remove()
        self._artists = self.map.draw_plan(self.ax, self.plan, self.from_xy, self.goal,
                                           from_label=self.from_label)
        text = describe_plan(self.plan, self.from_xy, self.goal)
        print(f"{LOG_TAG} Click at ({xm:.2f}, {ym:.2f}) in the mission frame: {text}.")
        self._set_title(f"goal ({xm:.2f}, {ym:.2f}): {text}\n"
                        + ("Enter: confirm | click: another point | Esc: quit" if self.plan is not None
                           else "click: another point | Esc: quit"))
        self.ax.legend(loc='upper right', fontsize=8)
        self.fig.canvas.draw_idle()

    def _key(self, ev):
        if ev.key == 'enter' and self.goal is not None and self.plan is not None:
            self.confirmed = True
            self.plt.close(self.fig)
        elif ev.key == 'h':
            self.home = True
            self.plt.close(self.fig)
        elif ev.key in ('escape', 'q'):
            self.confirmed = False
            self.plt.close(self.fig)

    def choose(self):
        """Opens the window and waits. (goal VISION, description), HOME or None."""
        self.plt.show()
        if self.home:
            return HOME
        if not self.confirmed:
            return None
        xm, ym = self.map.frame.from_vision(*self.goal)
        return self.goal, f"({xm:.2f}, {ym:.2f}) in the mission frame, chosen with a click"

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
