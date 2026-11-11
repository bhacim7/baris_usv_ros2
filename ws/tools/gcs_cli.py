#!/usr/bin/env python3
"""
A text GCS for testing the two-link GCS link before the GUI (RoboBoat 2027, step 5a).

Speaks usv_core.gcs_link over UDP (the Ubiquiti network) and/or a serial port (the RFD):
every command goes out on every link given, telemetry is shown per boat with the link that
brought the newest packet. No ROS needed.

    # in the simulation (sim.launch.py link:=true), both links:
    python3 gcs_cli.py --udp --rfd /tmp/usv_rfd_gcs watch
    python3 gcs_cli.py --udp --rfd /tmp/usv_rfd_gcs send azure set_manual false
    # one link only, to see that it works alone:
    python3 gcs_cli.py --rfd /tmp/usv_rfd_gcs send azure set_manual true
    # a task for the mission manager (usv_interfaces/TaskRequest on /<boat>/task_request):
    python3 gcs_cli.py --udp send azure task task=delivery urgent=1 p.shape=triangle

Boats by name or number (azure 1, marine 2); their addresses and ports from
ws/src/usv_nodes/config/boats/*.yaml, with --host for all of them (default 127.0.0.1, the
simulation; on the water drop it to use the profiles' Ubiquiti addresses).
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'usv_core'))
from usv_core import gcs_link  # noqa: E402
import yaml  # noqa: E402

BOATS_DIR = os.path.join(HERE, '..', 'src', 'usv_nodes', 'config', 'boats')
GCS_PORT = 15600


def boats():
    out = {}
    for f in sorted(os.listdir(BOATS_DIR)):
        if f.endswith('.yaml'):
            with open(os.path.join(BOATS_DIR, f)) as fh:
                d = yaml.safe_load(fh)
            out[d['name']] = d
    return out


def value(text):
    """Turn a command-line value into true/false, a number or text."""
    low = text.lower()
    if low in ('true', 'false'):
        return low == 'true'
    try:
        return float(text) if '.' in text else int(text)
    except ValueError:
        return text


def open_links(args, profiles):
    links = []
    if args.udp:
        peers = [(args.host or p['ip'], p['udp_port']) for p in profiles.values()]
        links.append(gcs_link.UdpLink(GCS_PORT, peers))
    if args.rfd:
        links.append(gcs_link.SerialLink(args.rfd, args.baud))
    if not links:
        sys.exit('give --udp and/or --rfd PORT')
    return gcs_link.Links(links)


def show(boat, link, pkt, age_by_link):
    ages = '  '.join(f'{name} {age:4.1f}s' if age is not None else f'{name}   -  '
                     for name, age in age_by_link.items())
    print(f"boat {boat} via {link:<3}  q {pkt.get('q', '-'):>5}  "
          f"{pkt.get('la', 0):.6f},{pkt.get('lo', 0):.6f}  hdg {pkt.get('h', 0):>4}  "
          f"manual {pkt.get('m', '-')}  task {pkt.get('k', '-')}  aq {pkt.get('aq', '-')}  "
          f"| last heard: {ages}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--udp', action='store_true', help='use the network link')
    ap.add_argument('--rfd', default='', help='serial port of the RFD (sim: /tmp/usv_rfd_gcs)')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--host', default='127.0.0.1',
                    help="boats' address for UDP; '' = each profile's ip")
    sub = ap.add_subparsers(dest='what', required=True)
    w = sub.add_parser('watch', help='show telemetry')
    w.add_argument('--every', type=float, default=1.0, help='one line per boat this often (s)')
    s = sub.add_parser('send', help='send one command and wait for its ack')
    s.add_argument('boat')
    s.add_argument('cmd', help='set_manual, set_relay, set_task, set_gps, emergency_stop ...')
    s.add_argument('args', nargs='*', help='value, or key=value pairs (task_name=..., '
                   'index=1 lat=.. lon=..)')
    args = ap.parse_args()

    profiles = boats()
    try:
        links = open_links(args, profiles)
    except OSError as e:
        sys.exit(str(e))
    newest = gcs_link.NewestFilter()
    last_print = {}

    def heard_ages(now):
        return {name: (now - t if t else None) for name, t in links.heard.items()}

    if args.what == 'watch':
        try:
            while True:
                now = time.monotonic()
                for link, pkt in links.read(now):
                    if 'i' not in pkt or not newest.is_new(pkt['i'], pkt.get('q'), now):
                        continue
                    if now - last_print.get(pkt['i'], 0.0) >= args.every and 'w' not in pkt:
                        last_print[pkt['i']] = now
                        show(pkt['i'], link, pkt, heard_ages(now))
                time.sleep(0.02)
        except KeyboardInterrupt:
            return 0

    by_name = {n: p['number'] for n, p in profiles.items()}
    number = by_name.get(args.boat) or int(args.boat)
    fields = {}
    for a in args.args:
        if a.startswith('p.') and '=' in a:      # task parameters: p.shape=triangle
            k, v = a[2:].split('=', 1)
            fields.setdefault('p', {})[k] = value(v)
        elif '=' in a:
            k, v = a.split('=', 1)
            fields[k] = value(v)
        else:
            fields['value'] = value(a)
    if args.cmd in ('set_manual', 'set_relay') and not isinstance(fields.get('value'), bool):
        sys.exit(f'{args.cmd} needs true or false, not {fields.get("value")!r}')
    sender = gcs_link.ReliableSender(number, links.send)
    # Hear the boat first: if it remembers a larger number (an earlier GCS run), start
    # past it, so this command is not taken for a copy
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        for link, pkt in links.read():
            if pkt.get('i') == number and 'aq' in pkt:
                sender.seq = max(sender.seq, int(pkt['aq']))
        time.sleep(0.02)
    cq = sender.command(args.cmd, time.monotonic(), **fields)
    print(f'sent {args.cmd} {fields} to boat {number} as cq {cq} on '
          f"{', '.join(link.name for link in links.links)}", flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        now = time.monotonic()
        for link, pkt in links.read(now):
            if pkt.get('i') == number and 'aq' in pkt and int(pkt['aq']) >= cq:
                print(f'acked by boat {number} via {link} after {now - t0:.2f} s '
                      f"(aq {pkt['aq']}, manual {pkt.get('m')})")
                return 0
        sender.service(now)
        if sender.failed:
            break
        time.sleep(0.02)
    print(f'NO ACK from boat {number}: is sim.launch.py running with link:=true?')
    return 1


if __name__ == '__main__':
    sys.exit(main())
