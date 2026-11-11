#!/usr/bin/env python3
"""
A pretend RoboCommand server (RoboBoat 2026 protocol) that also checks the rules.

Listens like the course server (TCP, $R + length + protobuf + !!), prints every Report, and
warns when a rule is broken: a vehicle's heartbeat late (> 1.5 s, the rule is 1 Hz), more
than 5 messages in a second, a sequence number not counting up, a timestamp off the clock
by more than 2 s (the course syncs everyone with NTP).

    ~/gcs-venv/bin/python ~/usv_ros2/gcs/robocommand/fake_server.py [--port 50000]
then in the GCS: RoboCommand host 127.0.0.1.
"""

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from robocommand.codec_roboboat2026 import Codec  # noqa: E402
from robocommand.framing import Deframer  # noqa: E402


class Checker:
    """The rules, as warnings; ROS-free and clock-injectable for the tests."""

    def __init__(self, beat_s=1.5, max_per_s=5, clock_tol_s=2.0):
        self.beat_s, self.max_per_s, self.clock_tol_s = beat_s, max_per_s, clock_tol_s
        self.last_beat, self.last_seq, self.times = {}, {}, []

    def check(self, report, now, wall):
        warn = []
        vid = report.vehicle_id
        if vid in self.last_seq and report.seq <= self.last_seq[vid]:
            warn.append(f'{vid}: seq {report.seq} after {self.last_seq[vid]}')
        self.last_seq[vid] = report.seq
        sent = report.sent_at.ToNanoseconds() / 1e9
        if abs(sent - wall) > self.clock_tol_s:
            warn.append(f'{vid}: timestamp {sent - wall:+.1f} s off the clock (NTP?)')
        self.times = [t for t in self.times if now - t < 1.0] + [now]
        if len(self.times) > self.max_per_s:
            warn.append(f'{len(self.times)} messages in one second (max {self.max_per_s})')
        if report.WhichOneof('body') == 'heartbeat':
            if vid in self.last_beat and now - self.last_beat[vid] > self.beat_s:
                warn.append(f'{vid}: heartbeat {now - self.last_beat[vid]:.1f} s after '
                            'the last one')
            self.last_beat[vid] = now
        return warn

    def silent(self, now):
        """Return the vehicles whose heartbeat is overdue."""
        return [v for v, t in self.last_beat.items() if now - t > 3 * self.beat_s]


def line(report):
    body = report.WhichOneof('body')
    msg = getattr(report, body)
    def show(f, v):                               # enums by name: STATE_AUTO, not 3
        return f.enum_type.values_by_number[v].name if f.enum_type else v
    fields = ', '.join(f'{f.name}={show(f, v)}' for f, v in msg.ListFields()
                       if f.name != 'position')
    pos = ''
    if msg.HasField('position') if hasattr(msg, 'position') else False:
        pos = f' @ {msg.position.latitude:.6f},{msg.position.longitude:.6f}'
    return f'{report.team_id}/{report.vehicle_id} #{report.seq} {body}: {fields}{pos}'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=50000)
    args = ap.parse_args()
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f'fake RoboCommand on {args.host}:{args.port}, waiting for the GCS', flush=True)
    while True:
        conn, addr = srv.accept()
        print(f'connected: {addr[0]}:{addr[1]}', flush=True)
        conn.settimeout(1.0)
        d, chk, quiet_t = Deframer(), Checker(), 0.0
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                data = None
            except OSError:
                data = b''
            if data == b'':
                print('disconnected', flush=True)
                break
            now = time.monotonic()
            for payload in d.feed(data or b''):
                try:
                    rep = Codec.decode(payload)
                except Exception as e:  # noqa: B902 (a bad message must not stop the server)
                    print(f'  !! cannot decode {len(payload)} B: {e}', flush=True)
                    continue
                print(line(rep), flush=True)
                for w in chk.check(rep, now, time.time()):
                    print(f'  !! {w}', flush=True)
            if now - quiet_t > 2.0:
                quiet_t = now
                for v in chk.silent(now):
                    print(f'  !! {v}: no heartbeat for {now - chk.last_beat[v]:.0f} s',
                          flush=True)
        conn.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
