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
               command done, kq (2027) the tasks still to come, '>'-joined; waypoint
               mini packets {i, q, w, wa, wo}
  GCS -> boat  {target_id, cmd, ..., cq}: set_manual, set_task, set_gps, set_target_color,
               set_relay, emergency_stop, (2027) task (see task_request); cq = the
               command's number, the boat answers with aq in its telemetry; the GCS
               resends until it sees it

Two links change two things:
  * a command arrives twice, in either order and late: the boat does each cq once
    (CommandFilter: anything at or below the last one done is a copy) and acks on both
  * telemetry arrives twice: the GCS keeps the newest per boat (NewestFilter on q)
The GCS numbers each boat's commands on their own (ReliableSender per boat), since the
radio is shared and each boat acks only its own.
"""

import json
import math
import os
import socket
import time

try:                              # Linux / macOS; on Windows the GCS uses pyserial instead
    import fcntl
    import termios
except ImportError:
    fcntl = termios = None

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

    The RFD (Mavlink framing off) cuts long lines wherever its frame fills, and another
    node's turn can fall between the pieces, so its packet lands inside ours (2026,
    GCSv1000._parse_or_recover):
        {"i":1,"a":16{"id":3,"drone_color":"KIRMIZI"}
        77,"b":1722,...}
    The complete packet at the end of a line is taken (it starts with one of MARKERS); the
    head before it gets ONE chance to meet its tail, the next line (the radio delivers
    bytes in order). Keeping heads longer made garbage valid JSON now and then (2026: the
    kilometre jumps on the map).
    """

    def __init__(self):
        self.buf = b''
        self.pending = ''
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            text = line.decode('utf-8', errors='replace').strip()
            if not text:
                continue
            glued = bool(self.pending)
            if glued:
                text, self.pending = self.pending + text, ''
            obj, head = parse_or_recover(text)
            self.pending = '' if glued or len(head) >= 2048 else head
            if isinstance(obj, dict):
                out.append(obj)
            else:
                self.bad += 1
        if len(self.buf) > MAX_LINE:
            self.buf = b''
            self.bad += 1
        return out


def parse_or_recover(text):
    """Return (packet or None, the unparsed head before it) for one received line."""
    try:
        return json.loads(text), ''
    except ValueError:
        pass
    for marker in MARKERS:
        start = text.find(marker, 1)
        while start != -1:
            try:
                return json.loads(text[start:]), text[:start]
            except ValueError:
                start = text.find(marker, start + 1)
    return None, ''


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


def report_packet(boat, q, report):
    """
    Return the wire packet of a task report (boat -> GCS).

    report: {id, task, type, lat, lon, fields: {key: value}}. Wire: {i, q, rp: id, rk: task,
    rt: type, ra, ro (position; absent if none), rf: fields}. Not the telemetry's la / lo:
    the GCS would take a report for the boat's position.
    """
    out = {'i': int(boat), 'q': int(q) % SEQ_MOD, 'rp': int(report['id']),
           'rk': str(report.get('task', '')), 'rt': str(report['type'])}
    lat, lon = report.get('lat'), report.get('lon')
    if lat is not None and lon is not None and math.isfinite(lat) and math.isfinite(lon):
        out['ra'], out['ro'] = round(float(lat), 7), round(float(lon), 7)
    if report.get('fields'):
        out['rf'] = {str(k): str(v) for k, v in report['fields'].items()}
    return out


def report_from_packet(pkt):
    """Return the report in a report packet (GCS side), as report_packet took it."""
    return {'id': int(pkt['rp']), 'task': pkt.get('rk', ''), 'type': pkt.get('rt', ''),
            'lat': pkt.get('ra'), 'lon': pkt.get('ro'), 'fields': dict(pkt.get('rf') or {})}


class ReportSender:
    """
    The boat's reports to the GCS: each resent every RESEND_S until the GCS acks it.

    The GCS acks with {target_id, cmd: report_ack, n}; it keeps the reports it has had,
    so a resend that crosses the ack is not passed on twice.
    """

    def __init__(self):
        self.pending = {}             # id -> (report, last sent)

    def add(self, report):
        self.pending[int(report['id'])] = (report, None)

    def ack(self, n):
        self.pending.pop(int(n), None)

    def due(self, now):
        """Return the oldest report due to be (re)sent now, or None; marks it sent."""
        for n in sorted(self.pending):
            report, sent = self.pending[n]
            if sent is None or now - sent >= RESEND_S * 2:
                self.pending[n] = (report, now)
                return report
        return None


def task_request(obj):
    """
    Return the fields of a `task` command for usv_interfaces/TaskRequest.

    Wire: {target_id, cmd: "task", task, urgent (0/1), src ("robocommand", default "gcs"),
    lat, lon (optional), p: {key: value, ...} (optional), cq}. Raises ValueError for a
    command without a task name. Parameters become text, sorted by key.
    """
    task = str(obj.get('task', '')).strip()
    if not task:
        raise ValueError('a task command needs a task name')
    params = obj.get('p') or {}
    if not isinstance(params, dict):
        raise ValueError('task parameters (p) must be key: value pairs')

    def number(key):
        try:
            value = float(obj[key])
        except (KeyError, TypeError, ValueError):
            return float('nan')
        return value if math.isfinite(value) else float('nan')
    keys = sorted(str(k) for k in params)
    return {'request_id': int(obj.get('cq') or 0), 'source': str(obj.get('src', 'gcs')),
            'task': task, 'urgent': bool(obj.get('urgent')),
            'latitude': number('lat'), 'longitude': number('lon'),
            'keys': keys, 'values': [str(params[k]) for k in keys]}


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
        # 2027: the tasks still to come, short (one radio frame); '' for the readiness run
        'kq': '>'.join(str(t) for t in state.get('task_queue', ()))[:48],
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
        if termios is None:
            raise OSError('SerialLink needs termios (Linux/macOS); use pyserial on Windows')
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
            termios.tcflush(self.fd, termios.TCIFLUSH)      # what piled up unread is old
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
