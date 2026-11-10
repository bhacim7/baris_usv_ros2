"""
The GCS link: the 2026 radio protocol, now over two links at once (RoboBoat 2027, step 5a).

The same newline-terminated JSON lines travel over the Ubiquiti network (UDP) and the RFD900x
radio (serial); every command and every telemetry packet goes over BOTH. ROS-free and
dependency-free (no pyserial), so the boat's telemetry_bridge and the GCS (plain Python,
no ROS) share it. Unit tested (test/test_gcs_link.py).

Protocol (unchanged from 2026, usv_legacy utils/telem.py and core/nav_process.py):
  boat -> GCS  telemetry, short keys: i boat number, q packet counter (0-99999), a b c d s
               PWM, v speed, h heading, th advised course, ea angle error, ce control
               error, k task, la lo position, xa xo target, ds distance, m manual, f FPS,
               r relay (-1 unknown), tc ts Task 3 colour and its source, aq the last
               command done; waypoint mini packets {i, q, w, wa, wo}
  GCS -> boat  {target_id, cmd, ..., cq}: set_manual, set_task, set_gps, set_target_color,
               set_relay, emergency_stop, (2027) task; cq = the command's number, the boat
               answers with aq in its telemetry; the GCS resends until it sees it

Two links change two things:
  * a command arrives twice, in either order and late: the boat does each cq once
    (CommandFilter: anything at or below the last one done is a copy) and acks on both
  * telemetry arrives twice: the GCS keeps the newest per boat (NewestFilter on q)
The GCS numbers each boat's commands on their own (ReliableSender per boat), since the
radio is shared and each boat acks only its own.
"""

import fcntl
import json
import math
import os
import socket
import termios
import time

SEQ_MOD = 100000                  # q wraps here (2026)
COPY_WINDOW = 1000                # a cq this far below the last one is a new GCS, not a copy
RESEND_S = 0.6                    # the 2026 GCS's resend period and tries
MAX_TRIES = 8
MAX_LINE = 8192                   # a line longer than this lost its newline: resync
MARKERS = ('{"i"', '{"target_id"', '{"cmd"', '{"id"')

# 2026 TEKNOFEST task names travel as numbers; anything else (RoboBoat phases) as text
TASK_CODES = {
    'TASK1_APPROACH': 0, 'TASK1_STATE_ENTER': 1, 'TASK1_STATE_MID': 2, 'TASK1_STATE_EXIT': 3,
    'TASK2_START': 4, 'TASK2_GO_TO_MID': 5, 'TASK2_GO_TO_END': 6, 'T3_START': 7,
    'T3_MID': 8, 'TASK3_SEARCH_KAMIKAZE': 9, 'FINISHED': 10, 'TASK_UNKNOWN': 11,
}


def encode(obj):
    """Return one wire line: compact JSON and a newline (spaces are airtime)."""
    return (json.dumps(obj, separators=(',', ':'), ensure_ascii=False) + '\n').encode('utf-8')


class LineReader:
    """
    Turn a byte stream (or datagrams) into dicts, surviving the radio's cuts.

    The RFD splits long lines wherever its buffer fills and another node's bytes can land in
    between (2026: GCSv1000._parse_or_recover). A line that does not parse is searched for
    the start of a packet (one of MARKERS); what comes before it is dropped.
    """

    def __init__(self):
        self.buf = b''
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            obj = self._parse(line.strip())
            if obj is not None:
                out.append(obj)
        if len(self.buf) > MAX_LINE:
            self.buf = b''
            self.bad += 1
        return out

    def _parse(self, line):
        if not line:
            return None
        text = line.decode('utf-8', errors='replace')
        try:
            obj = json.loads(text)
            return obj if isinstance(obj, dict) else None
        except ValueError:
            pass
        for marker in MARKERS:
            k = text.rfind(marker)
            if k > 0:
                try:
                    obj = json.loads(text[k:])
                    return obj if isinstance(obj, dict) else None
                except ValueError:
                    continue
        self.bad += 1
        return None


class CommandFilter:
    """The boat's side of the two links: do each command number once, whatever arrives."""

    def __init__(self):
        self.last = -1                # the last cq done (sent back as aq)

    def is_new(self, cq):
        """Return True for a command not done yet; copies and old resends are False."""
        if cq is None:
            return True               # an un-numbered command (old GCS): always done
        cq = int(cq)
        if cq > self.last:
            return True
        # Far below: a restarted GCS numbering from scratch (it jumps past our aq as soon
        # as it hears it, but its first commands may come first)
        return self.last - cq > COPY_WINDOW

    def done(self, cq):
        if cq is not None:
            self.last = int(cq)


class NewestFilter:
    """The GCS's side for telemetry: per boat, only packets newer than the last shown."""

    def __init__(self, restart_s=3.0):
        self.last = {}                # boat -> (q, time)
        self.restart_s = restart_s

    def is_new(self, boat, q, now):
        if q is None:
            return True
        q = int(q)
        prev = self.last.get(boat)
        newer = prev is None or 0 < (q - prev[0]) % SEQ_MOD < SEQ_MOD // 2
        # Nothing for a while: the boat may have restarted its counter
        if not newer and prev is not None and now - prev[1] >= self.restart_s:
            newer = True
        if newer:
            self.last[boat] = (q, now)
        return newer


class ReliableSender:
    """
    The GCS's commands to one boat: numbered, sent on every link, resent until acked.

    Acks are cumulative (the boat does commands in order): aq >= cq means that command and
    all before it landed. A boat that remembers a larger number (it outlived a GCS restart)
    moves our numbering past it, so new commands are not taken for copies.
    """

    def __init__(self, boat, send):
        self.boat = boat
        self.send = send              # send(dict): out on every link
        self.seq = 0
        self.last_aq = -1
        self.pending = []
        self.tries = 0
        self.last_tx = 0.0
        self.failed = []              # commands given up on (shown to the operator)

    def command(self, cmd, now, **fields):
        self.seq += 1
        payload = {'target_id': self.boat, 'cmd': cmd, **fields, 'cq': self.seq}
        self.pending.append(payload)
        if len(self.pending) == 1:
            self.service(now, force=True)
        return self.seq

    def on_ack(self, aq, now):
        aq = int(aq)
        if aq > self.seq:
            self.seq = aq
        self.last_aq = aq
        self.service(now)

    def service(self, now, force=False):
        """Drop what is acked; (re)send the head every RESEND_S; give up after MAX_TRIES."""
        while self.pending and self.last_aq >= self.pending[0]['cq']:
            self.pending.pop(0)
            self.tries, self.last_tx = 0, 0.0
            force = bool(self.pending)
        if not self.pending:
            return
        if force or now - self.last_tx >= RESEND_S:
            if self.tries >= MAX_TRIES:
                self.failed.append(self.pending.pop(0))
                self.tries = 0
                if not self.pending:
                    return
            self.send(self.pending[0])
            self.last_tx = now
            self.tries += 1


def _r(value, digits):
    """Round for the wire; a missing, NaN or infinite value goes as 0 (JSON has no NaN)."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0
    return round(value, digits) if math.isfinite(value) else 0


def telemetry(state, boat, q, aq):
    """
    Return the 2026 telemetry packet from the boat's state.

    state: dict with pwm (rear_left, rear_right, front_left, front_right, steer), speed,
    heading, lat, lon, task, manual, advised_course, angle_error, control_error,
    target_lat, target_lon, target_dist, fps, relay, target_color, target_color_source;
    missing values are sent as 0 (relay as -1, unknown).
    """
    pwm = state.get('pwm', {})
    task = state.get('task', '')
    return {
        'i': int(boat), 'q': int(q) % SEQ_MOD,
        'a': int(pwm.get('rear_left', 0)), 'b': int(pwm.get('rear_right', 0)),
        'c': int(pwm.get('front_left', 0)), 'd': int(pwm.get('front_right', 0)),
        's': int(pwm.get('steer', 0)),
        'v': _r(state.get('speed'), 1), 'h': _r(state.get('heading'), 0),
        'th': int(_r(state.get('advised_course'), 0)),
        'ea': int(_r(state.get('angle_error'), 0)),
        'ce': int(_r(state.get('control_error'), 0)),
        'k': TASK_CODES.get(task, task),
        'la': _r(state.get('lat'), 6), 'lo': _r(state.get('lon'), 6),
        'xa': _r(state.get('target_lat'), 6), 'xo': _r(state.get('target_lon'), 6),
        'ds': int(_r(state.get('target_dist'), 0)),
        'm': 1 if state.get('manual') else 0,
        'f': int(_r(state.get('fps'), 0)),
        'r': int(state.get('relay', -1)),
        'tc': state.get('target_color', ''),
        'ts': 1 if state.get('target_color_source') == 'drone' else 0,
        'aq': int(aq),
    }


# --------------------------------------------------------------------------- links
class UdpLink:
    """
    The Ubiquiti network: datagrams to fixed peers, one line per datagram.

    UDP rather than TCP: a lost datagram is simply lost (the commands have their own resend,
    telemetry is only worth its newest packet), so nothing waits behind it, and there is no
    connection to rebuild after the link drops or a boat restarts.
    """

    name = 'udp'

    def __init__(self, listen_port, peers, host='0.0.0.0'):
        self.peers = [(h, int(p)) for h, p in peers]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # No SO_REUSEADDR: a second program on the same port must fail, not silently take
        # the datagrams (two bridges for one boat each got half of the commands)
        try:
            self.sock.bind((host, int(listen_port)))
        except OSError as e:
            self.sock.close()
            raise OSError(f'UDP port {listen_port} is taken ({e.strerror}): is another '
                          'telemetry_bridge or GCS already running?') from e
        self.sock.setblocking(False)
        self.rx_bytes = self.tx_bytes = 0

    def send(self, data):
        for peer in self.peers:
            try:
                self.sock.sendto(data, peer)
                self.tx_bytes += len(data)
            except OSError:
                pass                  # network down: the radio still carries it

    def read(self):
        chunks = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            chunks.append(data if data.endswith(b'\n') else data + b'\n')
            self.rx_bytes += len(data)
        return b''.join(chunks)

    def close(self):
        self.sock.close()


BAUDS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
         57600: termios.B57600, 115200: termios.B115200}


class SerialLink:
    """
    The RFD900x radio: a serial port, raw, non-blocking (also a pseudo-terminal in sim).

    A line larger than what the port clears in budget_s is not started (2026: a write cut
    by the timeout left half a line that poisoned the next packet too). Linux/macOS only
    (termios); on Windows the GCS would use pyserial for this one class.
    """

    name = 'rfd'

    def __init__(self, path, baud=57600, budget_s=0.5):
        self.path = path
        self.baud = int(baud)
        self.limit = int(self.baud / 10.0 * budget_s)
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        # One reader per radio: a second program reading the port would take half the bytes
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as e:
            os.close(self.fd)
            raise OSError(f'{path} is in use by another program (telemetry_bridge or GCS '
                          'already running?)') from e
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0                                    # iflag: raw
            attrs[1] = 0                                    # oflag
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0                                    # lflag: no echo, no canon
            speed = BAUDS.get(self.baud, termios.B57600)
            attrs[4] = attrs[5] = speed
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except termios.error:
            pass                      # not a tty (a test pipe): bytes still flow
        self.rx_bytes = self.tx_bytes = 0
        self.dropped = 0

    def send(self, data):
        if len(data) > self.limit:
            self.dropped += 1
            return
        try:
            os.write(self.fd, data)
            self.tx_bytes += len(data)
        except (BlockingIOError, OSError):
            self.dropped += 1

    def read(self):
        chunks = []
        while True:
            try:
                data = os.read(self.fd, 4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            if not data:
                break
            chunks.append(data)
            self.rx_bytes += len(data)
        return b''.join(chunks)

    def close(self):
        os.close(self.fd)


class Links:
    """Every link at once: send on all, read from all; when each last heard something."""

    def __init__(self, links):
        self.links = list(links)
        self.readers = {link.name: LineReader() for link in self.links}
        self.heard = {link.name: 0.0 for link in self.links}

    def send(self, obj):
        data = encode(obj)
        for link in self.links:
            link.send(data)

    def read(self, now=None):
        """Return [(link name, dict)] of everything that arrived."""
        now = time.monotonic() if now is None else now
        out = []
        for link in self.links:
            data = link.read()
            if data:
                for obj in self.readers[link.name].feed(data):
                    self.heard[link.name] = now
                    out.append((link.name, obj))
        return out

    def close(self):
        for link in self.links:
            link.close()
