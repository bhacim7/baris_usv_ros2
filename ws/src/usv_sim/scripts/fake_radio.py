#!/usr/bin/env python3
"""
A pretend RFD900x network for the simulation: one pseudo-terminal per radio.

Every radio hears every other one (the RFDs share one channel), at the radio's speed: the
bytes of all radios together go out no faster than baud / 10 per second, so a busy channel
queues as the real one does. Optionally a share of the chunks is lost (loss).

    python3 fake_radio.py gcs azure marine [--baud 57600] [--loss 0.0] [--dir /tmp]

makes /tmp/usv_rfd_gcs, /tmp/usv_rfd_azure, /tmp/usv_rfd_marine: open them as serial
ports (telemetry_bridge serial_port, the GCS's RFD port). Plain Python, no ROS, Linux.
sim.launch.py starts it with link:=true.
"""

import argparse
import os
import pty
import random
import select
import sys
import time
import tty


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('names', nargs='+')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--loss', type=float, default=0.0, help='share of chunks lost (0-1)')
    ap.add_argument('--dir', default='/tmp')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    rng = random.Random(args.seed or None)

    radios = {}
    for name in args.names:
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        link = os.path.join(args.dir, f'usv_rfd_{name}')
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(os.ttyname(slave), link)
        radios[master] = (name, slave, link)
        print(f'fake_radio: {name} at {link}', flush=True)
    rate = args.baud / 10.0
    queue = []                    # (time due, sender fd, bytes)
    busy_until = time.monotonic()
    try:
        while True:
            ready, _, _ = select.select(list(radios), [], [], 0.005)
            now = time.monotonic()
            for fd in ready:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    continue
                if not data or rng.random() < args.loss:
                    continue
                # On the air after everything before it, at the channel's speed
                busy_until = max(busy_until, now) + len(data) / rate
                queue.append((busy_until, fd, data))
            while queue and queue[0][0] <= now:
                _, sender, data = queue.pop(0)
                for fd in radios:
                    if fd != sender:
                        try:
                            os.write(fd, data)
                        except OSError:
                            pass  # nobody reading that radio: its buffer is full
    except KeyboardInterrupt:
        pass
    finally:
        for fd, (_, slave, link) in radios.items():
            if os.path.lexists(link):
                os.remove(link)
            os.close(fd)
            os.close(slave)
    return 0


if __name__ == '__main__':
    sys.exit(main())
