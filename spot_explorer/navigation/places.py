"""
Named places and routes on a mission map (Places), kept in places.json next to the mission
maps (MissionMap/Mission_<name>/places.json). Everything in the mission frame: x ahead, y to
the left of the mission start, metres; heading in degrees from x towards y (None = any).

    {"places": {"gate": {"x": 6.0, "y": 2.0, "heading_deg": 90.0, "dwell_s": 0.0}, ...},
     "routes": {"patrol": ["gate", "shed", "home"], ...}}

A place is created from the command line (--add-place), with a click on the map (key p in
the window) or on the robot where it stands (save NAME in --interactive). A route is a list
of place names; 'home' is the mission start (wp_0).
"""
import json
import os
import re

HOME_NAME = 'home'
_NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')


class Places:
    """Places and routes of one mission. path None: in memory only (tests, maps without a
    folder)."""

    def __init__(self, path=None):
        self.path = path
        self.places, self.routes = {}, {}
        if path and os.path.exists(path):
            with open(path) as f:
                d = json.load(f)
            self.places = {k: dict(v) for k, v in d.get('places', {}).items()}
            self.routes = {k: list(v) for k, v in d.get('routes', {}).items()}

    def save(self):
        if not self.path:
            return
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(dict(places=self.places, routes=self.routes), f, indent=1, sort_keys=True)
        os.replace(tmp, self.path)

    @staticmethod
    def check_name(name):
        if not _NAME_RE.match(name or '') or name.lower() == HOME_NAME:
            raise ValueError(f"place/route name '{name}' not valid: letters, digits, _ and -, "
                             f"at most 40, not '{HOME_NAME}'")
        return name

    def add(self, name, x, y, heading_deg=None, dwell_s=0.0):
        self.check_name(name)
        self.places[name] = dict(x=float(x), y=float(y),
                                 heading_deg=None if heading_deg is None else float(heading_deg),
                                 dwell_s=float(dwell_s))
        self.save()
        return self.places[name]

    def remove(self, name):
        if name in self.places:
            del self.places[name]
        elif name in self.routes:
            del self.routes[name]
        else:
            raise KeyError(f"no place or route called '{name}'")
        self.save()

    def add_route(self, name, stops):
        self.check_name(name)
        for s in stops:
            if s.lower() != HOME_NAME and s not in self.places:
                raise KeyError(f"route {name}: no place called '{s}'")
        self.routes[name] = list(stops)
        self.save()

    def route_stops(self, spec):
        """The stops of a route: a saved route name, or a comma-separated list of places
        (and 'home'). KeyError for an unknown name."""
        stops = self.routes[spec] if spec in self.routes else [s.strip() for s in spec.split(',') if s.strip()]
        if not stops:
            raise KeyError(f"route '{spec}' is empty")
        for s in stops:
            if s.lower() != HOME_NAME and s not in self.places:
                raise KeyError(f"no place called '{s}' (places: {', '.join(sorted(self.places)) or 'none'})")
        return stops

    def describe(self):
        lines = [f"{len(self.places)} places, {len(self.routes)} routes"
                 + (f" ({self.path})" if self.path else "")]
        for k, p in sorted(self.places.items()):
            lines.append(f"  {k:20s} ({p['x']:6.2f}, {p['y']:6.2f})"
                         + (f"  heading {p['heading_deg']:+.0f} deg" if p.get('heading_deg') is not None else "")
                         + (f"  dwell {p['dwell_s']:g} s" if p.get('dwell_s') else ""))
        for k, r in sorted(self.routes.items()):
            lines.append(f"  route {k}: {' -> '.join(r)}")
        return "\n".join(lines)
