"""
GCS command intake, ported from legacy core/nav_process.py section B (commit 5df3f5b).

CommandFilter keeps legacy's rules around the dispatch itself:

  target_id   the RFD link is shared with the drone; a command addressed to another vehicle
              is dropped
  cq          R1 reliable commands: the GCS resends until our telemetry echoes the sequence
              number; a resend of the last processed one is skipped
  ack         set only after the dispatch returned, so a command whose handling failed is
              not confirmed and the GCS retries it
  exception   one bad command ends this cycle's intake (legacy wrapped the whole queue loop
              in one try); the commands behind it wait for the next cycle

What a command does is the caller's business: drain() hands each accepted command to
dispatch(cmd).
"""

from collections import deque


class CommandFilter:

    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.pending = deque()
        self.last_cq = None
        self.last_ack = None

    def submit(self, cmd):
        self.pending.append(cmd)

    def drain(self, dispatch):
        try:
            while self.pending:
                cmd = self.pending.popleft()
                tgt = cmd.get('target_id')
                if tgt is not None and int(tgt) != self.vehicle_id:
                    continue
                cq = cmd.get('cq')
                if cq is not None and cq == self.last_cq:
                    continue
                dispatch(cmd)
                if cq is not None:
                    self.last_cq = int(cq)
                    self.last_ack = self.last_cq
        except Exception:
            pass
