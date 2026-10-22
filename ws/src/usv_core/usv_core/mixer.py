"""Thruster mixer, ported from legacy core/nav_process.py apply_motor_mixer (commit 5df3f5b).

What changed in the port, and only this:
  * The slew limiter's "previous PWM" used to be read back from USVController's cache
    (controller.get_servo_pwm). The Mixer now remembers its own last output, so it does not
    know about the flight controller at all. Like the controller cache, it starts at 1500.
  * It returns a ThrusterCommand instead of calling controller.set_servo(); whoever drives
    the hardware or the simulator applies it. `force` carries the legacy `force=not slew`.
  * Settings come from MixerConfig instead of the global `config` module.

The arithmetic is unchanged; test/test_mixer.py drives this class and the frozen legacy
function with the same command sequences and requires identical PWM on every step.
"""

from dataclasses import dataclass
from typing import NamedTuple

import numpy as np


@dataclass(frozen=True)
class MixerConfig:
    """Defaults are the legacy config.py values at commit 5df3f5b."""

    base_pwm: int = 1500            # BASE_PWM
    steer_max_pwm: int = 1900       # STEER_MAX_PWM
    steer_min_pwm: int = 1100       # STEER_MIN_PWM
    max_pwm_change: int = 60        # MAX_PWM_CHANGE, per call (per 25 Hz cycle)


DEFAULT_CONFIG = MixerConfig()


class ThrusterCommand(NamedTuple):
    rear_left: int
    rear_right: int
    front_left: int
    front_right: int
    steer: int
    force: bool         # True = emergency / neutralise: bypass downstream write thinning


NEUTRAL = ThrusterCommand(1500, 1500, 1500, 1500, 1500, False)


def _slew(target, prev, max_delta):
    """Limit one channel's per-cycle change to +/- max_delta."""
    if max_delta is None or max_delta <= 0 or prev is None:
        return int(target)
    if target > prev + max_delta:
        return int(prev + max_delta)
    if target < prev - max_delta:
        return int(prev - max_delta)
    return int(target)


class Mixer:
    """Distributes forward and yaw effort across four thrusters and the steering servo."""

    def __init__(self, cfg=DEFAULT_CONFIG):
        self.cfg = cfg
        self.last = NEUTRAL

    def mix(self, forward_pwm, yaw_pwm, slew=True):
        """
        forward_pwm: absolute PWM-style forward effort (1500 = stop).
        yaw_pwm: turn effort, positive = turn right.
        slew=False bypasses the rate limiter (emergency braking, neutralise).
        """
        cfg = self.cfg
        base = cfg.base_pwm

        # 1. Steering servo: direct proportion
        steer_out = base + yaw_pwm
        steer_out = np.clip(steer_out, cfg.steer_min_pwm, cfg.steer_max_pwm)

        # 2. Differential thrust
        is_spot_turn = (forward_pwm == 1500 and yaw_pwm != 0)

        if is_spot_turn:
            diff_thrust = yaw_pwm * 1.0
        else:
            # Boost differential at low speed where the rudder is weak:
            # x2.0 at 1500, ~x1.4 at 1700, x1.0 at 1850 and above.
            diff_multiplier = 1.0 + max(0.0, (1850.0 - forward_pwm) / 350.0)
            diff_thrust = yaw_pwm * diff_multiplier

        # 3. Individual thrusters
        if forward_pwm < 1450:  # Actively reversing: no differential
            rear_left = int(forward_pwm)
            rear_right = int(forward_pwm)
            front_left = int(forward_pwm)
            front_right = int(forward_pwm)
        elif is_spot_turn:
            # Pure pivot: front and rear get identical differential
            rear_left = int(forward_pwm + diff_thrust)
            rear_right = int(forward_pwm - diff_thrust)
            front_left = int(forward_pwm + diff_thrust)
            front_right = int(forward_pwm - diff_thrust)
        else:
            # Turning while moving: full differential at the rear, half at the front
            rear_left = int(forward_pwm + diff_thrust)
            rear_right = int(forward_pwm - diff_thrust)
            front_left = int(forward_pwm + (diff_thrust * 0.5))
            front_right = int(forward_pwm - (diff_thrust * 0.5))

        rear_left = int(np.clip(rear_left, 1100, 1900))
        rear_right = int(np.clip(rear_right, 1100, 1900))
        front_left = int(np.clip(front_left, 1100, 1900))
        front_right = int(np.clip(front_right, 1100, 1900))
        steer_out = int(steer_out)

        # Slew-rate limit against the previous output (legacy: the controller's PWM cache)
        if slew:
            max_delta = cfg.max_pwm_change
            prev = self.last
            rear_left = _slew(rear_left, prev.rear_left, max_delta)
            rear_right = _slew(rear_right, prev.rear_right, max_delta)
            front_left = _slew(front_left, prev.front_left, max_delta)
            front_right = _slew(front_right, prev.front_right, max_delta)
            steer_out = _slew(steer_out, prev.steer, max_delta)

        self.last = ThrusterCommand(rear_left, rear_right, front_left, front_right, steer_out,
                                    not slew)
        return self.last

    def neutral(self):
        """Immediate neutral on every channel (legacy apply_motor_mixer(c, 1500, 0, slew=False))."""
        return self.mix(1500, 0, slew=False)
