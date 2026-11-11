"""
RoboBoat 2026 RoboCommand messages, the GCS client and the fake server's checks (protobuf).

Run (on the GCS computer): ~/gcs-venv/bin/python -m pytest ~/usv_ros2/gcs/test -v
"""

import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robocommand.client import Client  # noqa: E402
from robocommand.codec_roboboat2026 import Codec, pb  # noqa: E402
from robocommand.fake_server import Checker, line  # noqa: E402
from robocommand.framing import Deframer  # noqa: E402

TEL = {'MEVCUT_KONUM': {'lat': 27.3712, 'lon': -82.4523}, 'spd': 1.2, 'hdg': 271,
       'mod': 0, 'relay': 1, 'task': 'task2'}


def test_heartbeat_from_gcs_telemetry():
    c = Codec('DHO', {1: 'S1', 2: 'S2'})
    r = Codec.decode(c.heartbeat(2, 7, 1794400000.25, TEL))
    assert (r.team_id, r.vehicle_id, r.seq) == ('DHO', 'S2', 7)
    assert r.sent_at.ToNanoseconds() == 1794400000250000000
    hb = r.heartbeat
    assert hb.state == pb.STATE_AUTO and hb.current_task == pb.TASK_NAV_CHANNEL
    assert abs(hb.position.latitude - 27.3712) < 1e-9 and abs(hb.heading_deg - 271) < 1e-4
    manual = Codec.decode(c.heartbeat(1, 1, 0.0, dict(TEL, mod=1))).heartbeat
    killed = Codec.decode(c.heartbeat(1, 1, 0.0, dict(TEL, relay=0))).heartbeat
    assert manual.state == pb.STATE_MANUAL and killed.state == pb.STATE_KILLED
    assert len(c.heartbeat(1, 1, 0.0, TEL)) < 255             # fits one frame


def test_reports():
    c = Codec('DHO', {1: 'S1'})
    g = Codec.decode(c.report(1, 2, 0.0, {'type': 'gate', 'gate': 'exit', 'lat': 1.0,
                                          'lon': 2.0}))
    assert g.gate_pass.type == pb.GATE_EXIT and g.gate_pass.position.longitude == 2.0
    o = Codec.decode(c.report(1, 3, 0.0, {'type': 'object', 'object': 'light_beacon',
                                          'color': 'green', 'id': 12, 'task': 'task2'}))
    assert (o.object_detected.object_type, o.object_detected.color,
            o.object_detected.object_id) == (pb.OBJECT_LIGHT_BEACON, pb.COLOR_GREEN, 12)
    d = Codec.decode(c.report(1, 4, 0.0, {'type': 'docking', 'dock': 'N', 'slip': '2'}))
    assert (d.docking.dock, d.docking.slip) == ('N', '2')
    w = Codec.decode(c.report(1, 5, 0.0, {'type': 'delivery', 'color': 'yellow',
                                          'kind': 'water'}))
    assert w.object_delivery.delivery_type == pb.DELIVERY_WATER
    try:
        c.report(1, 6, 0.0, {'type': 'selfie'})
        assert False, 'an unknown report type must be refused'
    except ValueError:
        pass
    assert line(g).startswith('DHO/S1 #2 gate_pass: type=GATE_EXIT @ 1.000000,2.000000')


def test_checker_finds_the_broken_rules():
    c, chk = Codec('DHO', {1: 'S1'}), Checker()
    wall = 1794400000.0

    def hb(seq, stamp):
        return Codec.decode(c.heartbeat(1, seq, stamp, TEL))
    assert chk.check(hb(1, wall), 0.0, wall) == []
    assert chk.check(hb(2, wall + 1), 1.0, wall + 1) == []
    late = chk.check(hb(3, wall + 3), 3.0, wall + 3)
    assert any('heartbeat 2.0 s' in w for w in late)
    assert any('seq 3 after 3' in w for w in chk.check(hb(3, wall + 3.2), 3.2, wall + 3.2))
    assert any('off the clock' in w for w in chk.check(hb(4, wall), 3.4, wall + 3.4))
    many = [chk.check(hb(5 + k, wall + 3.5), 3.5, wall + 3.5) for k in range(4)]
    assert any('messages in one second' in w for w in many[-1])
    assert chk.silent(10.0) == ['S1']


def test_client_sends_heartbeats_to_a_server():
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(1)
    rc = Client('127.0.0.1', srv.getsockname()[1], 'DHO')
    rc.on_telemetry(1, TEL, 0.0)
    rc.on_telemetry(2, dict(TEL, mod=1), 0.0)
    rc.tick(0.0)
    conn, _ = srv.accept()
    conn.settimeout(1.0)
    got, d = [], Deframer()
    t0 = time.monotonic()
    while len(got) < 2 and time.monotonic() - t0 < 2.0:
        got += d.feed(conn.recv(4096))
    reports = [Codec.decode(p) for p in got]
    assert [r.vehicle_id for r in reports] == ['S1', 'S2']
    assert all(r.WhichOneof('body') == 'heartbeat' for r in reports)
    assert rc.status().startswith('BAĞLI')
    conn.close()
    rc.close()
    srv.close()
