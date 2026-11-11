"""
GCS RoboCommand layers that need no protobuf: framing, TCP transport, the bridge's timing.

Run (on the GCS computer): ~/gcs-venv/bin/python -m pytest ~/usv_ros2/gcs/test -v
"""

import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robocommand.bridge import Bridge  # noqa: E402
from robocommand.framing import Deframer, frame  # noqa: E402
from robocommand.tcp import TcpTransport  # noqa: E402


def test_frame_and_deframe_with_junk_and_cuts():
    a, b = frame(b'hello'), frame(b'\x00\x01!!$R')      # payload may hold the markers
    # Junk between frames, and a false header whose length points past a footer
    stream = b'xx' + a + b'junk' + b'$R\x02ab??' + b
    d = Deframer()
    got = []
    for k in range(0, len(stream), 3):                   # arrives in small pieces
        got += d.feed(stream[k:k + 3])
    assert got == [b'hello', b'\x00\x01!!$R']
    try:
        frame(b'x' * 256)
        assert False, 'a 256 B payload must be refused'
    except ValueError:
        pass


class FakeCodec:
    def heartbeat(self, boat, seq, stamp, tel):
        return f'hb {boat} {seq} {tel.get("m")}'.encode()

    def report(self, boat, seq, stamp, report):
        return f'{report["type"]} {boat} {seq}'.encode()


def test_bridge_beats_each_boat_once_a_second_and_respects_the_rate():
    sent = []
    br = Bridge(FakeCodec(), lambda d: sent.append(d) or True, max_per_s=5)
    br.on_telemetry(1, {'m': 1}, 0.0)
    br.on_telemetry(2, {'m': 0}, 0.0)
    for k in range(6):
        br.add_report(1, {'type': 'gate'})
    br.tick(0.0)
    # two heartbeats first, then reports up to five in that second
    assert sent == [b'hb 1 1 1', b'hb 2 1 0', b'gate 1 2', b'gate 1 3', b'gate 1 4']
    br.tick(0.5)
    assert len(sent) == 5                              # nothing due, budget used
    br.on_telemetry(1, {'m': 1}, 1.0)
    br.on_telemetry(2, {'m': 0}, 1.0)
    br.tick(1.0)
    assert sent[5:7] == [b'hb 1 5 1', b'hb 2 2 0'] and len(sent) == 10


def test_bridge_keeps_fields_a_packet_lacks():
    sent = []
    br = Bridge(FakeCodec(), lambda d: sent.append(d) or True)
    br.on_telemetry(1, {'m': 1}, 0.0)
    br.on_telemetry(1, {'w': 3}, 0.1)                  # a waypoint mini packet
    br.tick(0.1)
    assert sent == [b'hb 1 1 1']


def test_bridge_does_not_report_a_boat_it_no_longer_hears():
    sent = []
    br = Bridge(FakeCodec(), lambda d: sent.append(d) or True)
    br.on_telemetry(1, {'m': 1}, 0.0)
    br.tick(0.0)
    br.tick(5.0)                                       # 5 s of silence
    assert sent == [b'hb 1 1 1']


def test_tcp_transport_talks_to_a_server_and_reconnects():
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    t = TcpTransport('127.0.0.1', port, retry_s=0.1)
    assert t.send(b'one')
    conn, _ = srv.accept()
    conn.settimeout(1.0)
    assert Deframer().feed(conn.recv(100)) == [b'one']
    conn.sendall(frame(b'cmd'))
    got = []
    for _ in range(100):
        got += t.poll()
        if got:
            break
        time.sleep(0.01)
    assert got == [b'cmd']
    conn.close()                                       # server goes away
    for _ in range(100):
        t.poll()
        if not t.connected:
            break
        time.sleep(0.01)
    assert not t.connected
    time.sleep(0.15)
    assert t.send(b'two')                              # connects again by itself
    conn2, _ = srv.accept()
    conn2.settimeout(1.0)
    assert Deframer().feed(conn2.recv(100)) == [b'two']
    conn2.close()
    t.close()
    srv.close()
