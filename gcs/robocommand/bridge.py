"""
What the GCS sends to RoboCommand, and when; protocol-free (the codec builds the bytes).

  * a heartbeat for every boat once a second, from its latest telemetry (position, mode),
    only while that telemetry is fresh (a boat the GCS does not hear is not reported alive)
  * reports (gate passed, object found, ...) as the boats send them, queued
  * at most max_per_s messages a second in all (RoboBoat 2026: 5); heartbeats first,
    so a burst of reports never makes a boat look dead
  * one sequence number per boat, counting up

The codec is any object with heartbeat(boat, seq, stamp, telemetry) and report(boat, seq,
stamp, report) returning bytes.
"""

import time
from collections import deque

HEARTBEAT_S = 1.0
FRESH_S = 3.0


class Bridge:

    def __init__(self, codec, send, max_per_s=5, clock=time.time):
        self.codec, self.send, self.max_per_s, self.clock = codec, send, max_per_s, clock
        self.telemetry = {}           # boat -> (monotonic time heard, dict)
        self.next_beat = {}
        self.seq = {}
        self.reports = deque()
        self.sent_times = deque()
        self.dropped = 0
        self.last_sent = ''

    def on_telemetry(self, boat, telemetry, now):
        """Note a boat's packet; fields it lacks keep their last value (waypoint packets)."""
        merged = dict(self.telemetry.get(boat, (0.0, {}))[1])
        merged.update(telemetry)
        self.telemetry[boat] = (now, merged)
        self.next_beat.setdefault(boat, now)

    def add_report(self, boat, report):
        """Queue a task report (a dict the codec understands) for a boat."""
        self.reports.append((boat, report))

    def _next_seq(self, boat):
        self.seq[boat] = self.seq.get(boat, 0) + 1
        return self.seq[boat]

    def _budget(self, now):
        while self.sent_times and now - self.sent_times[0] >= 1.0:
            self.sent_times.popleft()
        return self.max_per_s - len(self.sent_times)

    def _out(self, data, now, what):
        if self.send(data):
            self.sent_times.append(now)
            self.last_sent = what
            return True
        self.dropped += 1
        return False

    def tick(self, now):
        """Send what is due within the rate limit; call it often (e.g. every 0.1 s)."""
        for boat in sorted(self.next_beat):
            heard, tel = self.telemetry[boat]
            if now < self.next_beat[boat]:
                continue
            self.next_beat[boat] += HEARTBEAT_S
            if self.next_beat[boat] < now:
                self.next_beat[boat] = now + HEARTBEAT_S
            if now - heard > FRESH_S or self._budget(now) <= 0:
                continue
            data = self.codec.heartbeat(boat, self._next_seq(boat), self.clock(), tel)
            self._out(data, now, f'heartbeat {boat}')
        while self.reports and self._budget(now) > 0:
            boat, report = self.reports.popleft()
            try:
                data = self.codec.report(boat, self._next_seq(boat), self.clock(), report)
            except ValueError:            # a report the protocol has no message for
                self.dropped += 1
                continue
            self._out(data, now, f"{report.get('type', 'report')} {boat}")
