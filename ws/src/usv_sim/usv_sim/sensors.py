"""How the boat's sensors distort the truth, for the simulation.

ROS-free so it can be unit tested. Gazebo gives exact position, velocity and attitude;
sim_bridge passes them through these models so the software sees what it would see on the
boat (rates and delays are handled in sim_bridge):

  GaussMarkov     a slowly wandering error, e.g. the metre-level drift of a GPS fix
                  without RTK (Here Pro on the Orange Cube)
  offset_latlon   moves a latitude/longitude by metres north and east
  quantize        MAVLink integer units, e.g. 1e-7 deg in GPS_RAW_INT
  DelayLine       a value becomes visible only after a fixed delay
  compass_deg     Gazebo ENU yaw (counter-clockwise from east) -> compass heading
                  (clockwise from north, 0-360)
"""

import math
import random
from collections import deque

from usv_sim.geo import latlon


class GaussMarkov:
    """First-order Gauss-Markov process: standard deviation sigma, correlation time tau.

    x(t + dt) = x(t) * exp(-dt / tau) + sigma * sqrt(1 - exp(-2 dt / tau)) * N(0, 1)
    Stationary, so it starts from a random draw instead of zero.
    """

    def __init__(self, sigma, tau_s, rng):
        self.sigma = float(sigma)
        self.tau = float(tau_s)
        self.rng = rng
        self.value = self.rng.gauss(0.0, self.sigma) if self.sigma > 0 else 0.0

    def step(self, dt):
        if self.sigma <= 0 or dt <= 0:
            return self.value
        a = math.exp(-dt / self.tau)
        self.value = a * self.value + self.sigma * math.sqrt(1.0 - a * a) * self.rng.gauss(0, 1)
        return self.value


def offset_latlon(lat_deg, lon_deg, north_m, east_m):
    return latlon(east_m, north_m, lat_deg, lon_deg)


def quantize(value, step):
    return round(value / step) * step if step > 0 else value


class DelayLine:
    """Holds (time, value) pairs; latest(now) is the newest value at least delay_s old."""

    def __init__(self, delay_s):
        self.delay = float(delay_s)
        self.items = deque()
        self.current = None

    def push(self, t, value):
        self.items.append((t, value))

    def latest(self, now):
        while self.items and self.items[0][0] <= now - self.delay:
            self.current = self.items.popleft()[1]
        return self.current


def yaw_from_quaternion(x, y, z, w):
    """Rotation about z (rad) of a quaternion, ENU: 0 = east, counter-clockwise positive."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def roll_pitch_from_quaternion(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def compass_deg(enu_yaw_rad):
    """ENU yaw -> heading in degrees clockwise from north, in [0, 360)."""
    return (90.0 - math.degrees(enu_yaw_rad)) % 360.0


def make_rng(seed):
    """seed 0 = different every run; any other value = the same noise every run."""
    return random.Random(None if seed == 0 else seed)
