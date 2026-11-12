"""
The GCS's RoboCommand client: transport + codec + bridge, polled from the GUI's timer.

    rc = Client(host, port, team_id)       # RoboBoat 2026 TCP, codec_roboboat2026
    rc.on_telemetry(boat, packet)          # every telemetry packet the GCS shows
    rc.add_report(boat, {...})             # a task report (codec_roboboat2026.Codec.report)
    rc.tick()                              # often (e.g. every 100 ms): sends, receives
    rc.status()                            # one line for the GCS's RoboCommand box
    rc.tasks                               # tasks from RoboCommand, for the boats (2027)
"""

import time

from robocommand.bridge import Bridge
from robocommand.codec_roboboat2026 import Codec
from robocommand.tcp import TcpTransport

VEHICLE_IDS = {1: 'S1', 2: 'S2'}       # RoboNation's "S" = surface vessel; Azure 1, Marine 2


class Client:

    def __init__(self, host='10.10.10.1', port=50000, team_id='TEAM'):
        self.transport = TcpTransport(host, port)
        self.codec = Codec(team_id, VEHICLE_IDS)
        self.bridge = Bridge(self.codec, self.transport.send)
        self.tasks = []

    def on_telemetry(self, boat, packet, now=None):
        self.bridge.on_telemetry(boat, packet, time.monotonic() if now is None else now)

    def add_report(self, boat, report):
        """Queue a report; False if this year's protocol has no message for it."""
        if not self.codec.knows(report):
            return False
        self.bridge.add_report(boat, report)
        return True

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        for payload in self.transport.poll(now):
            task = self.codec.command(payload)
            if task is not None:
                self.tasks.append(task)
        self.bridge.tick(now)

    def status(self):
        t = self.transport
        state = 'BAĞLI' if t.connected else f'bağlanıyor ({t.error or "..."})'
        return (f'{state} · gönderilen {t.sent} · düşen {self.bridge.dropped}'
                f'{" · son: " + self.bridge.last_sent if self.bridge.last_sent else ""}')

    def close(self):
        self.transport.close()
