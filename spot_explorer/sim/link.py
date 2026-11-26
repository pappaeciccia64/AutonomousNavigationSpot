"""Socket client towards the simulator (protocol in server.py). Standard library only."""
import base64
import json
import socket
import threading
import zlib

from spot_explorer.sim.server import DEFAULT_PORT, PROTO


class SimLinkError(RuntimeError):
    """The simulator does not answer, or answers with an error."""


def parse_address(text):
    """'host:port', 'host' or ':port' -> (host, port). Empty = 127.0.0.1:DEFAULT_PORT."""
    text = (text or '').strip()
    host, port = '127.0.0.1', DEFAULT_PORT
    if text:
        if ':' in text:
            h, p = text.rsplit(':', 1)
            host = h or host
            port = int(p) if p else port
        else:
            host = text
    return host, port


class SimLink:
    def __init__(self, host='127.0.0.1', port=DEFAULT_PORT, timeout_s=3.0):
        self.host, self.port = host, int(port)
        self._lock = threading.Lock()
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=timeout_s)
        except OSError as e:
            raise SimLinkError(
                "no simulator listening on %s:%d (%s). Start sim/isaac_spot_server.py (or "
                "sim/kinematic_server.py) first." % (self.host, self.port, e))
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._file = self._sock.makefile('rwb')
        hello = self._request({'op': 'hello'})
        if hello.get('proto') != PROTO:
            raise SimLinkError("simulator protocol %r, expected %r: the simulator script and "
                               "the mission code are not the same version."
                               % (hello.get('proto'), PROTO))
        self.backend = hello.get('backend', '?')
        self.ops = tuple(hello.get('ops', ()))          # requests beyond hello, world, step
        self.joints = tuple(hello.get('joints', ()))    # order of "jp" and "jv" in the state

    def _request(self, req):
        with self._lock:
            try:
                self._file.write((json.dumps(req) + '\n').encode('utf-8'))
                self._file.flush()
                line = self._file.readline()
            except OSError as e:
                raise SimLinkError("link to the simulator lost (%s)" % (e,))
        if not line:
            raise SimLinkError("the simulator closed the link")
        reply = json.loads(line.decode('utf-8'))
        if not reply.get('ok'):
            raise SimLinkError(reply.get('error', 'request refused by the simulator'))
        return reply

    def world(self):
        return self._request({'op': 'world'}).get('world') or {}

    def step(self, cmd=None):
        """Send a velocity command (or None to leave it as it is) and read the state:
        dict with t, p = [x, y, z], q = [w, x, y, z]."""
        req = {'op': 'step'}
        if cmd is not None:
            req['cmd'] = [float(cmd[0]), float(cmd[1]), float(cmd[2])]
        return self._request(req)

    # --- sensors (Isaac Sim only: the kinematic simulator answers with an error) ----------
    def sensors(self):
        """What the simulator carries: {'cameras': [...], 'lidar': {...} or None, 'imu': ...}.
        Empty where there are no sensors."""
        if 'sensors' not in self.ops:
            return {'cameras': [], 'lidar': None, 'imu': None}
        return self._request({'op': 'sensors'})

    def images(self, sources=None, quality=80):
        """Latest camera frames, decoded: a list of dicts with name, camera, t, p, q (body pose
        at the capture), w, h and either 'jpeg' (bytes) or 'depth_mm' (bytes of h x w uint16,
        little endian, 0 = nothing within range)."""
        req = {'op': 'image', 'quality': int(quality)}
        if sources is not None:
            req['sources'] = list(sources)
        out = []
        for item in self._request(req).get('images', ()):
            raw = base64.b64decode(item.pop('data'))
            if item.get('format') == 'jpeg':
                item['jpeg'] = raw
            else:
                item['depth_mm'] = zlib.decompress(raw)
            out.append(item)
        return out

    def lidar(self):
        """Latest full turn of the lidar: dict with t, p, q (body pose at the capture),
        mount_p, mount_q, n and 'xyz' (bytes of n x 3 float32, little endian, lidar frame)."""
        reply = self._request({'op': 'lidar'})
        reply['xyz'] = zlib.decompress(base64.b64decode(reply.pop('data')))
        return reply

    def imu(self, since=-1.0):
        """IMU readings later than `since` (simulated seconds), 100 per second, the last few
        seconds at most: dict with hz and the lists t, a (specific force), g (angular rate),
        q (orientation), all in the body frame."""
        return self._request({'op': 'imu', 'since': float(since)})

    def screenshot(self, path):
        """Ask the simulator to save its window to a file (written a moment later)."""
        return self._request({'op': 'screenshot', 'path': str(path)}).get('path')

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass
