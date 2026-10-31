"""
The survey pilot: drives a course's survey route from the boat's true pose (sim only).

ROS-free, unit tested (test/test_survey.py); survey_drive.py runs it in the simulation.

For the map-quality measurements (phase 6 R1.6) every run must take the same path, whatever
the sensors and the map say, so a change to perception or the mapper is compared on equal
terms. So this is not the navigator: it is given Gazebo's exact position and heading, and
the route from the course file's `survey` section (usv_sim/course.py).

Steps:
  {east, north}   follow the straight leg from where the last step ended to this point:
                  aim lookahead_m ahead of the closest point on the leg, so the boat stays
                  on the line (narrow channel gates) instead of cutting corners. Done when
                  closer than reach_m, or level with the point and within 2 * reach_m.
  {spin_deg: N}   turn on the spot until the heading has turned N degrees (+ clockwise)

Output is the navigator's (forward_pwm, yaw_pwm) pair, for usv_core.mixer.Mixer.mix:
forward 1500 = stop, yaw + = turn right. A large heading error is turned on the spot first.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PilotConfig:
    cruise_pwm: int = 1680        # navigator base_pwm + cruise_pwm
    slow_pwm: int = 1580          # heading error between slow_deg and spot_deg
    slow_deg: float = 30.0
    spot_deg: float = 70.0        # larger error: turn on the spot first
    spot_pwm: int = 200           # navigator spot_turn_pwm
    gain_pwm_per_deg: float = 5.0
    max_yaw_pwm: int = 250
    lookahead_m: float = 2.5
    reach_m: float = 1.0
    step_timeout_s: float = 90.0


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def compass_bearing(from_e, from_n, to_e, to_n):
    return math.degrees(math.atan2(to_e - from_e, to_n - from_n)) % 360.0


class SurveyPilot:

    def __init__(self, steps, cfg=PilotConfig()):
        self.steps = list(steps)
        self.cfg = cfg
        self.index = 0
        self.leg_start = None         # (east, north) where the current leg began
        self.step_started = None      # time
        self.turned = 0.0             # spin: degrees turned so far
        self.last_heading = None
        self.log = []                 # (time, message) of finished / skipped steps

    @property
    def done(self):
        return self.index >= len(self.steps)

    def step(self, now, east, north, heading_deg):
        """One control cycle from the hull centre (ENU m) and compass heading (deg)."""
        if self.done:
            return 1500, 0
        cfg = self.cfg
        if self.step_started is None:
            self.step_started = now
            self.leg_start = self.leg_start or (east, north)
            self.turned, self.last_heading = 0.0, heading_deg
        if now - self.step_started > cfg.step_timeout_s:
            self._next(now, east, north, 'timed out')
            return 1500, 0
        s = self.steps[self.index]
        if 'spin_deg' in s:
            return self._spin(now, east, north, heading_deg, s['spin_deg'])
        return self._leg(now, east, north, heading_deg, s['east'], s['north'])

    def _next(self, now, east, north, how):
        self.log.append((now, f'step {self.index + 1}/{len(self.steps)} {how}'))
        self.index += 1
        self.leg_start = (east, north)
        self.step_started = None

    def _spin(self, now, east, north, heading_deg, spin_deg):
        self.turned += wrap180(heading_deg - self.last_heading)
        self.last_heading = heading_deg
        if abs(self.turned) >= abs(spin_deg):
            self._next(now, east, north, f'turned {self.turned:+.0f} deg')
            return 1500, 0
        return 1500, int(math.copysign(self.cfg.spot_pwm, spin_deg))

    def _leg(self, now, east, north, heading_deg, to_e, to_n):
        cfg = self.cfg
        x0, y0 = self.leg_start
        dx, dy = to_e - x0, to_n - y0
        length = math.hypot(dx, dy)
        dist = math.hypot(to_e - east, to_n - north)
        if length < 1e-6:
            along, ux, uy = 0.0, 0.0, 0.0
        else:
            ux, uy = dx / length, dy / length
            along = (east - x0) * ux + (north - y0) * uy
        if dist < cfg.reach_m or (along >= length and dist < 2.0 * cfg.reach_m):
            self._next(now, east, north, f'reached ({dist:.1f} m off)')
            return 1500, 0
        if length < 1e-6 or along >= length:
            aim_e, aim_n = to_e, to_n
        else:
            a = max(0.0, along) + cfg.lookahead_m
            a = min(a, length)
            aim_e, aim_n = x0 + a * ux, y0 + a * uy
        err = wrap180(compass_bearing(east, north, aim_e, aim_n) - heading_deg)
        if abs(err) > cfg.spot_deg:
            return 1500, int(math.copysign(cfg.spot_pwm, err))
        yaw = int(max(-cfg.max_yaw_pwm, min(cfg.max_yaw_pwm, cfg.gain_pwm_per_deg * err)))
        forward = cfg.cruise_pwm if abs(err) < cfg.slow_deg else cfg.slow_pwm
        return forward, yaw
