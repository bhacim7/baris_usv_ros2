"""Port check for usv_core.mixer against apply_motor_mixer extracted from legacy nav_process.py."""

import os
import random
import sys

import pytest

from usv_core.mixer import DEFAULT_CONFIG, NEUTRAL, Mixer

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))
import config as legacy_config  # noqa: E402
import mixer_extract as legacy  # noqa: E402

CHANNELS = (legacy.CH_REAR_LEFT, legacy.CH_REAR_RIGHT, legacy.CH_FRONT_LEFT,
            legacy.CH_FRONT_RIGHT, legacy.CH_STEER)


class FakeController:
    """Mimics USVController's PWM cache: starts at 1500, set_servo stores int(pwm)."""

    def __init__(self):
        self.pwms = {ch: 1500 for ch in CHANNELS}
        self.force = False

    def get_servo_pwm(self, channel):
        return self.pwms.get(channel, 1500)

    def set_servo(self, channel, pwm, force=False):
        self.pwms[channel] = int(pwm)
        self.force = force

    def output(self):
        return tuple(self.pwms[ch] for ch in CHANNELS) + (self.force,)


def run_both(commands):
    """Feed (forward, yaw, slew) commands to both versions; assert equality on every step."""
    ctrl = FakeController()
    mixer = Mixer()
    for step, (forward, yaw, slew) in enumerate(commands):
        legacy.apply_motor_mixer(ctrl, forward, yaw, slew=slew)
        new = mixer.mix(forward, yaw, slew=slew)
        assert tuple(new) == ctrl.output(), f'step {step}: {forward}, {yaw}, {slew}'


MISSION_LIKE = (
    [(1680, 0, True)] * 20 +                      # cruise
    [(1680, 50, True)] * 10 +                     # gentle right
    [(1680, -300, True)] * 10 +                   # hard left, saturating
    [(1500, 200, True)] * 15 +                    # spot turn right (SPOT_TURN_PWM)
    [(1500, -140, True)] * 10 +                   # spot turn left (T3_SPOT_TURN_PWM)
    [(1500 + 180 * 0.4, 75.5, True)] * 5 +        # Pure Pursuit taper: float forward
    [(1250, 400, False), (1200, -400, False)] +   # emergency braking, no slew
    [(1500, 0, True)] * 20 +                      # back to neutral, slewed
    [(1500, 0, False)] +                          # neutralise
    [(1850, 10, True)] + [(1900, 0, True)] * 20 +
    [(1449.9, 30, True), (1450, 30, True), (1500, 0.0, True)]
)


# --- Settings ------------------------------------------------------------------------

def test_default_config_matches_legacy():
    assert DEFAULT_CONFIG.base_pwm == legacy_config.BASE_PWM
    assert DEFAULT_CONFIG.steer_max_pwm == legacy_config.STEER_MAX_PWM
    assert DEFAULT_CONFIG.steer_min_pwm == legacy_config.STEER_MIN_PWM
    assert DEFAULT_CONFIG.max_pwm_change == legacy_config.MAX_PWM_CHANGE


# --- Port equivalence ----------------------------------------------------------------

def test_mission_like_sequence_matches_legacy():
    run_both(MISSION_LIKE)


@pytest.mark.parametrize('seed', range(10))
def test_random_sequence_matches_legacy(seed):
    rng = random.Random(seed)
    commands = []
    for _ in range(300):
        forward = rng.choice([1500, 1500, rng.uniform(1100, 1900), rng.randint(1100, 1900)])
        yaw = rng.choice([0, rng.uniform(-500, 500), rng.randint(-400, 400)])
        commands.append((forward, yaw, rng.random() < 0.9))
    run_both(commands)


# --- Known behaviour ------------------------------------------------------------------

def test_starts_neutral():
    assert Mixer().last == NEUTRAL


def test_slew_limits_60_per_cycle():
    mixer = Mixer()
    rear_left = [mixer.mix(1900, 0).rear_left for _ in range(8)]
    assert rear_left == [1560, 1620, 1680, 1740, 1800, 1860, 1900, 1900]


def test_cruise_with_right_turn():
    # Multiplier at 1680 is 1 + 170/350; rear gets the full differential, front half
    out = Mixer().mix(1680, 100, slew=False)
    assert tuple(out) == (1828, 1531, 1754, 1605, 1600, True)


def test_spot_turn_is_symmetric():
    out = Mixer().mix(1500, 200, slew=False)
    assert tuple(out)[:5] == (1700, 1300, 1700, 1300, 1700)


def test_braking_has_no_differential():
    out = Mixer().mix(1300, 400, slew=False)
    assert tuple(out)[:5] == (1300, 1300, 1300, 1300, 1900)


def test_neutral_bypasses_slew():
    mixer = Mixer()
    for _ in range(10):
        mixer.mix(1900, 300)
    assert mixer.neutral() == (1500, 1500, 1500, 1500, 1500, True)
