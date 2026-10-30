"""
Port check for usv_core.relay against the legacy relay retry code.

The legacy code is inline in nav_worker, so LegacyRelay below re-types it as written
(nav_process.py lines 372-379, 444-459 and 498-502, commit 5df3f5b), keeping its order
within a cycle: read RELAY_STATUS and retry first (section A), take commands after
(section B).

Run: cd /ws/src/usv_core && python3 -m pytest test/test_relay.py -v
"""

import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))

import config as legacy_cfg  # noqa: E402
import pytest  # noqa: E402
from usv_core.relay import RelayRetry  # noqa: E402

RELAY_MAX_RETRIES = getattr(legacy_cfg, 'RELAY_COMMAND_RETRIES', 5)
RELAY_RETRY_INTERVAL_S = getattr(legacy_cfg, 'RELAY_RETRY_INTERVAL_S', 0.25)


class LegacyRelay:

    def __init__(self):
        self.relay_target = None
        self.relay_retries = 0
        self.relay_last_tx = 0.0

    def cycle(self, relay_reported, start_time, command):
        sent = None
        if self.relay_target is not None:
            if relay_reported is not None and int(relay_reported) == self.relay_target:
                self.relay_target = None
            elif self.relay_retries <= 0:
                self.relay_target = None
            elif (start_time - self.relay_last_tx) >= RELAY_RETRY_INTERVAL_S:
                self.relay_last_tx = start_time
                self.relay_retries -= 1
                sent = self.relay_target
        if command is not None:
            want = 1 if command else 0
            self.relay_target = want
            self.relay_retries = RELAY_MAX_RETRIES
            self.relay_last_tx = 0.0
        return sent


def new_cycle(relay, reported, now, command):
    sent = relay.update(reported, now)
    if command is not None:
        relay.request(command)
    return sent


def test_defaults_are_the_legacy_config():
    r = RelayRetry()
    assert (r.max_retries, r.retry_interval_s) == (RELAY_MAX_RETRIES, RELAY_RETRY_INTERVAL_S)


@pytest.mark.parametrize('seed', range(20))
def test_matches_legacy(seed):
    """Drive both with a lossy, late vehicle link and a transmitter that flips the relay."""
    rng = random.Random(seed)
    old, new = LegacyRelay(), RelayRetry()
    state = rng.choice([None, 0, 1])          # what RELAY_STATUS reports
    pending = []                              # (apply_at, value) sends in flight
    t = 1000.0
    for _ in range(2000):
        t += rng.choice([0.04, 0.04, 0.04, 0.2])
        for item in [p for p in pending if p[0] <= t]:
            pending.remove(item)
            state = item[1]
        if rng.random() < 0.003:
            state = rng.choice([0, 1])        # RC7 switch
        command = rng.choice([True, False]) if rng.random() < 0.01 else None
        a = old.cycle(state, t, command)
        b = new_cycle(new, state, t, command)
        assert a == b
        if a is not None and rng.random() < 0.6:      # 40 % of sends lost on the link
            pending.append((t + rng.uniform(0.05, 0.6), a))


def test_retries_until_confirmed_then_stops():
    r = RelayRetry()
    r.request(True)
    sends = [r.update(0, 1000.0 + 0.04 * k) for k in range(40)]
    assert [s for s in sends if s is not None] == [1] * 5      # five attempts, 0.25 s apart
    assert r.target is None                                     # then gives up

    r.request(False)
    assert r.update(1, 2000.0) == 0
    assert r.update(0, 2000.04) is None                         # confirmed: done
    assert r.target is None
