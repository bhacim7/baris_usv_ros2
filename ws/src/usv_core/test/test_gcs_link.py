"""
usv_core.gcs_link: the 2026 GCS protocol over two links at once (RoboBoat 2027, step 5a).

Run: cd /ws/src/usv_core && python3 -m pytest test/test_gcs_link.py -v
"""

import json
import os
import pty
import tty

import pytest
from usv_core import gcs_link


def test_lines_survive_cuts_and_junk():
    r = gcs_link.LineReader()
    pkt = gcs_link.encode({'i': 1, 'q': 7, 'h': 90})
    assert r.feed(pkt[:5]) == []                     # half a line waits
    assert r.feed(pkt[5:]) == [{'i': 1, 'q': 7, 'h': 90}]
    # Another radio's bytes cut into our line: the packet after the junk is recovered
    junk = b'{"i":1,"q":8,"h":9{"id":3,"drone_color":"SIYAH"}\n'
    assert r.feed(junk) == [{'id': 3, 'drone_color': 'SIYAH'}]
    assert r.feed(b'garbage\n') == [] and r.bad == 1


def test_a_packet_cut_by_another_radio_is_glued_back():
    r = gcs_link.LineReader()
    got = r.feed(b'{"i":1,"q":5,"a":16{"id":3,"drone_color":"KIRMIZI"}\n'
                 b'00,"b":1722}\n')
    assert got == [{'id': 3, 'drone_color': 'KIRMIZI'}, {'i': 1, 'q': 5, 'a': 1600,
                                                         'b': 1722}]
    # A head gets one chance only: two cut lines in a row are not glued into garbage
    got = r.feed(b'{"i":1,"q":6,"a":1{"id":3}\n{"i":1,"q":7,"a":2{"id":3}\n9}\n')
    assert got == [{'id': 3}, {'id': 3}] and r.pending == ''


def test_encode_is_compact():
    line = gcs_link.encode({'target_id': 2, 'cmd': 'set_manual', 'value': True, 'cq': 4})
    assert line == b'{"target_id":2,"cmd":"set_manual","value":true,"cq":4}\n'


def test_boat_does_each_command_once_whatever_the_order():
    f = gcs_link.CommandFilter()
    done = []
    # cq 5 over UDP, 6 over UDP, then the RFD's late copies of 5 and 6
    for cq in (5, 6, 5, 6, 6):
        if f.is_new(cq):
            done.append(cq)
            f.done(cq)
    assert done == [5, 6] and f.last == 6
    assert f.is_new(None)                            # old GCS without numbers
    # A restarted GCS numbering from 1 is not taken for copies
    f.done(5000)
    assert f.is_new(1)


def test_gcs_keeps_the_newest_telemetry_per_boat():
    n = gcs_link.NewestFilter()
    assert n.is_new(1, 10, 0.0)
    assert not n.is_new(1, 10, 0.1)                  # the same packet over the other link
    assert not n.is_new(1, 9, 0.2)                   # an older one, late
    assert n.is_new(1, 11, 0.3)
    assert n.is_new(2, 3, 0.3)                       # each boat on its own
    assert n.is_new(1, 2, 0.4) is False
    assert n.is_new(1, 0, 4.0)                       # silence: the boat restarted
    assert n.is_new(1, 99999, 5.0) is False and n.is_new(1, 1, 5.1)
    n.last[1] = (99998, 6.0)
    assert n.is_new(1, 3, 6.1)                       # wrapped round


def test_sender_resends_until_acked_and_numbers_per_boat():
    sent = []
    s = gcs_link.ReliableSender(2, sent.append)
    cq1 = s.command('set_manual', 0.0, value=False)
    cq2 = s.command('set_relay', 0.0, value=1)
    assert sent == [{'target_id': 2, 'cmd': 'set_manual', 'value': False, 'cq': 1}]
    s.service(0.3)
    assert len(sent) == 1                            # not yet time to resend
    s.service(0.7)
    assert len(sent) == 2 and sent[1]['cq'] == cq1   # resent
    s.on_ack(cq1, 0.8)
    assert sent[-1]['cq'] == cq2                     # the next goes at once
    s.on_ack(cq2, 0.9)
    assert s.pending == []
    # A boat that remembers a larger number moves ours past it
    s.on_ack(40, 1.0)
    assert s.command('set_task', 1.0, task_name='T') == 41


def test_sender_gives_up_after_its_tries():
    sent = []
    s = gcs_link.ReliableSender(1, sent.append)
    s.command('emergency_stop', 0.0)
    for k in range(1, 20):
        s.service(k * 0.7)
    assert len(sent) == gcs_link.MAX_TRIES and len(s.failed) == 1


def test_telemetry_packet_has_the_2026_keys():
    pkt = gcs_link.telemetry({'pwm': {'rear_left': 1600, 'steer': 1500}, 'heading': 271.6,
                              'lat': 27.3712345678, 'lon': -82.45, 'task': 'TASK1_STATE_ENTER',
                              'manual': True, 'relay': 1}, boat=2, q=100003, aq=7)
    assert pkt['i'] == 2 and pkt['q'] == 3 and pkt['aq'] == 7
    assert pkt['a'] == 1600 and pkt['s'] == 1500 and pkt['b'] == 0
    assert pkt['h'] == 272 and pkt['la'] == 27.371235 and pkt['k'] == 1 and pkt['m'] == 1
    assert gcs_link.telemetry({'task': 'T1_ENTRY'}, 1, 0, -1)['k'] == 'T1_ENTRY'
    assert gcs_link.telemetry({}, 1, 0, -1)['r'] == -1
    # Before the run the navigator reports NaN (no advised course yet): sent as 0, not a crash
    nan = gcs_link.telemetry({'advised_course': float('nan'), 'heading': float('inf'),
                              'target_dist': float('nan')}, 1, 0, -1)
    assert (nan['th'], nan['h'], nan['ds']) == (0, 0, 0)
    assert b'NaN' not in gcs_link.encode(nan)
    assert len(gcs_link.encode(pkt)) < 250           # one radio frame (2026)


def test_udp_link_both_ways():
    a = gcs_link.UdpLink(0, [], host='127.0.0.1')
    b = gcs_link.UdpLink(0, [('127.0.0.1', a.sock.getsockname()[1])], host='127.0.0.1')
    a.peers = [('127.0.0.1', b.sock.getsockname()[1])]
    la, lb = gcs_link.Links([a]), gcs_link.Links([b])
    lb.send({'i': 1, 'q': 1})
    got = []
    for _ in range(200):
        got += la.read()
        if got:
            break
    assert got == [('udp', {'i': 1, 'q': 1})]
    la.close()
    lb.close()


def test_serial_link_over_a_pseudo_terminal():
    master, slave = pty.openpty()
    tty.setraw(slave)
    link = gcs_link.SerialLink(os.ttyname(slave), 57600)
    os.write(master, b'{"target_id":1,"cmd":"set_manual","value":false,"cq":3}\n')
    links = gcs_link.Links([link])
    got = []
    for _ in range(200):
        got += links.read()
        if got:
            break
    assert got[0][1]['cmd'] == 'set_manual'
    links.send({'i': 1, 'q': 0, 'aq': 3})
    assert json.loads(os.read(master, 4096)) == {'i': 1, 'q': 0, 'aq': 3}
    link.send(b'x' * (link.limit + 1))               # too long for the radio: not started
    assert link.dropped == 1
    links.close()
    os.close(master)
    os.close(slave)


def test_a_second_program_cannot_take_the_same_link():
    a = gcs_link.UdpLink(0, [], host='127.0.0.1')
    with pytest.raises(OSError, match='taken'):
        gcs_link.UdpLink(a.sock.getsockname()[1], [], host='127.0.0.1')
    a.close()
    master, slave = pty.openpty()
    first = gcs_link.SerialLink(os.ttyname(slave))
    with pytest.raises(OSError, match='in use'):
        gcs_link.SerialLink(os.ttyname(slave))
    first.close()
    gcs_link.SerialLink(os.ttyname(slave)).close()      # free again once closed
    os.close(master)
    os.close(slave)


def test_two_links_one_command_done_once_end_to_end():
    """Check a GCS and a boat on a network and a radio: one command done once, acked."""
    m1, s1 = pty.openpty()
    tty.setraw(s1)
    gcs_udp = gcs_link.UdpLink(0, [], host='127.0.0.1')
    boat_udp = gcs_link.UdpLink(0, [('127.0.0.1', gcs_udp.sock.getsockname()[1])],
                                host='127.0.0.1')
    gcs_udp.peers = [('127.0.0.1', boat_udp.sock.getsockname()[1])]
    gcs_rfd = gcs_link.SerialLink(os.ttyname(s1))
    gcs = gcs_link.Links([gcs_udp, gcs_rfd])
    boat = gcs_link.Links([boat_udp])
    sender = gcs_link.ReliableSender(1, gcs.send)
    cq = sender.command('set_manual', 0.0, value=False)
    # The radio's copy reaches the boat through the pty master
    radio_copy = os.read(m1, 4096)
    f, done = gcs_link.CommandFilter(), []
    arrived = []
    for _ in range(200):
        arrived += [obj for _, obj in boat.read()]
        if arrived:
            break
    arrived += gcs_link.LineReader().feed(radio_copy)
    for obj in arrived:
        if f.is_new(obj['cq']):
            done.append(obj['cmd'])
            f.done(obj['cq'])
    assert done == ['set_manual'] and len(arrived) == 2
    boat.send(gcs_link.telemetry({}, 1, 0, f.last))
    for _ in range(200):
        for _, pkt in gcs.read():
            sender.on_ack(pkt['aq'], 0.1)
        if not sender.pending:
            break
    assert sender.pending == [] and sender.last_aq == cq
    gcs.close()
    boat.close()
    os.close(m1)
    os.close(s1)
