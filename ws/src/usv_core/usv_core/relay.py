"""
Motor power relay retries, ported from legacy core/nav_process.py (commit 5df3f5b).

MAV_CMD_DO_SET_RELAY is sent with command_long_send, which waits for no ACK, so a dropped
safety command would go unnoticed. Legacy therefore resends the request every
RELAY_RETRY_INTERVAL_S, at most RELAY_COMMAND_RETRIES times, and stops as soon as
RELAY_STATUS reports the state it asked for. The retries are spread over control cycles
instead of slept through, so the loop keeps running.

RelayRetry holds that state; whoever owns the flight controller link (fc_bridge) calls, once
per cycle and in legacy's order:

  update(reported, now)   reported = relay state read back from RELAY_STATUS (None = never
                          reported); returns the state to send this cycle, or None
  request(want)           a new command; it goes out on the next update()

The transmitter can switch this relay too (RC7_OPTION=28), so the truth always comes from
the vehicle, not from what was last sent.
"""


class RelayRetry:

    def __init__(self, max_retries=5, retry_interval_s=0.25, log=None):
        self.max_retries = max_retries
        self.retry_interval_s = retry_interval_s
        self.log = log or (lambda msg: None)
        self.target = None          # state we are trying to reach, None = nothing pending
        self.retries = 0
        self.last_tx = 0.0

    def request(self, want):
        self.target = 1 if want else 0
        self.retries = self.max_retries
        self.last_tx = 0.0          # due at once
        self.log(f"Relay command received: {'ON' if self.target else 'OFF'}")

    def update(self, reported, now):
        if self.target is None:
            return None
        if reported is not None and int(reported) == self.target:
            self.target = None      # vehicle agrees, done
        elif self.retries <= 0:
            self.log(f'Relay command not confirmed after {self.max_retries} attempts.')
            self.target = None
        elif (now - self.last_tx) >= self.retry_interval_s:
            self.last_tx = now
            self.retries -= 1
            return self.target
        return None
