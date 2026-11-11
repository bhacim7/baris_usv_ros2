"""
TCP transport to RoboCommand (RoboBoat 2026: 10.10.10.1:50000 on the tent's network).

Non-blocking, polled from the GCS's thread; reconnects by itself every retry_s while the
cable or the server is away. Carries framed payloads (framing.py) both ways.
"""

import socket
import time

from robocommand.framing import Deframer, frame


class TcpTransport:

    name = 'tcp'

    def __init__(self, host='10.10.10.1', port=50000, retry_s=2.0):
        self.host, self.port, self.retry_s = host, int(port), retry_s
        self.sock = None
        self.deframer = Deframer()
        self.next_try = 0.0
        self.error = ''
        self.sent = self.received = 0

    @property
    def connected(self):
        return self.sock is not None

    def _connect(self, now):
        self.next_try = now + self.retry_s
        try:
            s = socket.create_connection((self.host, self.port), timeout=0.5)
        except OSError as e:
            self.error = str(e)
            return
        s.setblocking(False)
        self.sock, self.error = s, ''
        self.deframer = Deframer()

    def _drop(self, e):
        self.error = str(e)
        try:
            self.sock.close()
        except OSError:
            pass
        self.sock = None

    def send(self, payload, now=None):
        """Send one payload; False if not connected (the caller decides what is lost)."""
        now = time.monotonic() if now is None else now
        if self.sock is None:
            if now >= self.next_try:
                self._connect(now)
            if self.sock is None:
                return False
        try:
            self.sock.sendall(frame(payload))
            self.sent += 1
            return True
        except OSError as e:
            self._drop(e)
            return False

    def poll(self, now=None):
        """Return the payloads received since the last call (connects when due)."""
        now = time.monotonic() if now is None else now
        if self.sock is None:
            if now >= self.next_try:
                self._connect(now)
            return []
        out = []
        while True:
            try:
                data = self.sock.recv(4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as e:
                self._drop(e)
                break
            if not data:
                self._drop(ConnectionError('closed by RoboCommand'))
                break
            out += self.deframer.feed(data)
        self.received += len(out)
        return out

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
