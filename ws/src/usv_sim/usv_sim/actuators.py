"""What the boat's hardware does with a PWM command, for the simulation.

ROS-free so it can be unit tested. sim_bridge stands in for fc_bridge + Orange Cube + ESCs +
steering servos, so each class here copies one link of that chain:

  ServoWriteThinning  USVController.set_servo (legacy MainSystem2.py): a DO_SET_SERVO is
                      only sent when the value moved by SERVO_MIN_PWM_DELTA or
                      SERVO_REFRESH_S passed; the FC holds (latches) the last value it got.
  PwmToThrust         ESC + Degz Utras thruster: PWM (us) -> thrust (N), from a table.
  steer_angle_rad     steering PWM -> front thruster angle (team: 1100 = 45 deg left,
                      1900 = 45 deg right; Gazebo: + = counter-clockwise = left).
  SlewLimiter         MG995 servo: the angle follows the command at a limited rate.
"""

from bisect import bisect_right


class PwmToThrust:
    """Piecewise-linear table PWM (us) -> thrust (N), clamped at both ends."""

    def __init__(self, pwm_points, thrust_points):
        if len(pwm_points) != len(thrust_points) or len(pwm_points) < 2:
            raise ValueError('need at least two (pwm, thrust) points of equal count')
        if any(b <= a for a, b in zip(pwm_points, pwm_points[1:])):
            raise ValueError('pwm points must increase')
        self.pwm = [float(p) for p in pwm_points]
        self.thrust = [float(t) for t in thrust_points]

    def __call__(self, pwm):
        if pwm <= self.pwm[0]:
            return self.thrust[0]
        if pwm >= self.pwm[-1]:
            return self.thrust[-1]
        i = bisect_right(self.pwm, pwm) - 1
        f = (pwm - self.pwm[i]) / (self.pwm[i + 1] - self.pwm[i])
        return self.thrust[i] + f * (self.thrust[i + 1] - self.thrust[i])


def steer_angle_rad(pwm, center_us=1500.0, half_range_us=400.0, max_angle_rad=0.785,
                    sign=-1.0):
    """Steering PWM -> front thruster angle in Gazebo's convention, clamped to the stops.

    sign = -1 because the team's PWM above 1500 turns the boat right, while a positive
    (counter-clockwise) angle on a front thruster turns it left (measured, see model.sdf).
    """
    angle = (pwm - center_us) / half_range_us * max_angle_rad
    angle = max(-max_angle_rad, min(max_angle_rad, angle))
    return sign * angle


class SlewLimiter:
    """Moves a value towards its target by at most rate * dt per step."""

    def __init__(self, rate_per_s, initial=0.0):
        self.rate = float(rate_per_s)
        self.value = float(initial)

    def step(self, target, dt):
        limit = self.rate * max(0.0, dt)
        delta = max(-limit, min(limit, target - self.value))
        self.value += delta
        return self.value


class ServoWriteThinning:
    """The value the flight controller actually holds for one output channel.

    Same rule as the legacy USVController.set_servo: an update smaller than min_delta_us is
    not sent unless refresh_s passed since the last write; force always goes out.
    """

    def __init__(self, min_delta_us=3, refresh_s=0.5, initial_us=1500):
        self.min_delta = min_delta_us
        self.refresh = refresh_s
        self.requested = None          # last value asked for (legacy self.pwms[channel])
        self.held = initial_us         # last value written to the FC
        self.last_tx = None

    def write(self, pwm, now, force=False):
        prev = self.requested
        self.requested = pwm
        if (not force and prev is not None and self.last_tx is not None
                and abs(pwm - prev) < self.min_delta and (now - self.last_tx) < self.refresh):
            return self.held
        self.held = pwm
        self.last_tx = now
        return self.held
