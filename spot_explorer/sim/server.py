"""
Socket server shared by the two simulators (sim/isaac_spot_server.py and
sim/kinematic_server.py). Standard library only: it runs inside Isaac Sim's Python too.

Protocol: one JSON object per line, one answer per request.

    {"op": "hello"}                      -> {"ok": true, "backend": "...", "proto": 1}
    {"op": "world"}                      -> {"ok": true, "world": {...}}        (see world.py)
    {"op": "step", "cmd": [vx, vy, wz]}  -> {"ok": true, "t": sim time, "p": [x, y, z],
                                             "q": [w, x, y, z]}
    {"op": "step"}                       -> the same, without changing the command

A simulator may add to the state what it knows besides the pose: "v" and "w" (linear and
angular velocity of the body in the world frame), "jp" and "jv" (joint positions and
velocities, in the order of "joints" in the hello answer), "imu" ({"a": specific force,
"g": angular rate, both in the body frame, "q": orientation}).

Sensors, only where the simulator has them (Isaac Sim; see sim/isaac_sensors.py):

    {"op": "sensors"}                    -> {"cameras": [...], "lidar": {...}, "imu": {...}}
    {"op": "image", "sources": ["frontleft", "frontleft_depth"]}
                                         -> {"images": [{"name", "format", "data" (base64), ...}]}
    {"op": "lidar"}                      -> {"n", "data" (base64 of zlib of float32 x, y, z), ...}
    {"op": "screenshot", "path": "..."}  -> the window of the simulator saved to a file

A request the simulator does not know is answered with ok = false and the reason.

cmd is the base velocity in the BODY frame (m/s, m/s, rad/s): forward, left, counterclockwise.
p and q are the body pose in the world frame. If no command arrives for WATCHDOG_S seconds
the simulator stops the robot: a mission that crashed must not leave it walking.
"""
import json
import socket
import threading
import time

PROTO = 1
DEFAULT_PORT = 8765
WATCHDOG_S = 0.6


class SimServer:
    """
    Holds the latest command received and the latest state published by the simulator.

    The simulator calls set_state() from its own loop and command() to read what to apply;
    the socket threads only touch these two values, never the simulator.
    """

    def __init__(self, backend, world_dict=None, host='127.0.0.1', port=DEFAULT_PORT):
        self.backend = str(backend)
        self.world_dict = world_dict or {}
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_wall = 0.0
        self._state = None
        # Requests beyond the three above: op -> function(request) -> dict merged in the answer.
        # Called from the socket threads: they must only read what the simulator publishes.
        self.providers = {}
        self.info = {}                  # added to the hello answer
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, int(port)))
        self._sock.listen(4)
        self.address = self._sock.getsockname()
        self._running = True
        self.clients = 0
        threading.Thread(target=self._accept_loop, daemon=True).start()

    # --- simulator side ------------------------------------------------------------------
    def set_state(self, t, position, quat_wxyz, extra=None):
        state = {'t': float(t), 'p': [float(v) for v in position],
                 'q': [float(v) for v in quat_wxyz]}
        if extra:
            state.update(extra)
        with self._lock:
            self._state = state

    def command(self):
        """(vx, vy, wz) to apply now: zero if the last command is older than WATCHDOG_S."""
        with self._lock:
            if time.time() - self._cmd_wall > WATCHDOG_S:
                return (0.0, 0.0, 0.0)
            return self._cmd

    def close(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass

    # --- socket side ---------------------------------------------------------------------
    def _accept_loop(self):
        while self._running:
            try:
                conn, _ = self._sock.accept()
            except Exception:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        self.clients += 1
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            f = conn.makefile('rwb')
            for line in f:
                try:
                    reply = self._handle(json.loads(line.decode('utf-8')))
                except Exception as e:      # a bad request must not close the link
                    reply = {'ok': False, 'error': '%s: %s' % (type(e).__name__, e)}
                f.write((json.dumps(reply) + '\n').encode('utf-8'))
                f.flush()
        except Exception:
            pass
        finally:
            self.clients -= 1
            try:
                conn.close()
            except Exception:
                pass

    def _handle(self, req):
        op = req.get('op')
        if op == 'hello':
            out = {'ok': True, 'backend': self.backend, 'proto': PROTO,
                   'ops': sorted(self.providers)}
            out.update(self.info)
            return out
        if op == 'world':
            return {'ok': True, 'world': self.world_dict}
        if op == 'step':
            cmd = req.get('cmd')
            with self._lock:
                if cmd is not None:
                    self._cmd = (float(cmd[0]), float(cmd[1]), float(cmd[2]))
                    self._cmd_wall = time.time()
                state = self._state
            if state is None:
                return {'ok': False, 'error': 'simulator not ready yet'}
            out = {'ok': True}
            out.update(state)
            return out
        if op in self.providers:
            out = {'ok': True}
            out.update(self.providers[op](req) or {})
            return out
        return {'ok': False, 'error': 'this simulator (%s) does not answer %r' % (self.backend, op)}
