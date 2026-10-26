"""What the ZED 2i measures, for the simulation (ROS-free, unit tested).

  world_field_enu       the Earth's field at the course (declination, inclination,
                        strength from the course file), as an ENU vector. Gazebo's own
                        magnetometer is NOT used: with spherical coordinates set, gz-sim
                        computes the field from a coarse built-in table (4.8 deg declination
                        at Golcuk instead of ~6), in gauss, and stores north/east/down in the
                        east/north/up axes (measured 2026-10-26: z came out +0.40 G, i.e. up).
  body_field            what a magnetometer fixed to the boat reads from a world field
  magnetic_heading_deg  heading from the magnetometer, as the ZED SDK's magnetic_heading:
                        the field is levelled with the attitude (tilt compensation), then
                        the angle of its horizontal part gives the heading clockwise from
                        MAGNETIC north. Declination is added later by usv_core.heading.
                        A hard-iron offset (a field fixed to the boat: motors, batteries,
                        steel) bends it by an amount that depends on the heading, as a
                        badly calibrated compass does.
  FrameClock            when the camera delivers frames: 30 fps with spells of ~15 fps, as
                        the FPS counter in the 2026-10-14 video showed.
"""

import math


def rotate(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w * t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def world_field_enu(declination_deg, inclination_deg, strength_uT):
    """ENU field vector (T): declination east of north, inclination downwards."""
    total = strength_uT * 1e-6
    inc = math.radians(inclination_deg)
    dec = math.radians(declination_deg)
    horizontal = total * math.cos(inc)
    return (horizontal * math.sin(dec), horizontal * math.cos(dec), -total * math.sin(inc))


def body_field(field_enu, q_body_to_enu):
    """The world field as seen from the body frame (inverse rotation)."""
    x, y, z, w = q_body_to_enu
    return rotate((-x, -y, -z, w), field_enu)


def magnetic_heading_deg(field_body, q_body_to_enu, hard_iron_body=(0.0, 0.0, 0.0)):
    """Magnetic heading in [0, 360) from a body-frame field (x forward, y left, z up).

    q_body_to_enu: attitude from the IMU. Only its roll and pitch matter: the field is
    turned into the world frame and back about the vertical by the IMU's own yaw, which
    leaves it expressed in a level frame that points where the boat points.
    """
    m = tuple(f + b for f, b in zip(field_body, hard_iron_body))
    mx, my, _ = rotate(q_body_to_enu, m)
    x, y, z, w = q_body_to_enu
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    c, s = math.cos(-yaw), math.sin(-yaw)
    level_x = c * mx - s * my
    level_y = s * mx + c * my
    # Magnetic north is at angle atan2(level_y, level_x) counter-clockwise from the bow,
    # so the bow is that far clockwise from north
    return math.degrees(math.atan2(level_y, level_x)) % 360.0


class FrameClock:
    """Two-state frame rate: normal fps, with occasional slow spells (exponential lengths)."""

    def __init__(self, rng, fps_normal=30.0, fps_slow=15.0, normal_mean_s=20.0,
                 slow_mean_s=2.0):
        self.rng = rng
        self.fps = {False: fps_normal, True: fps_slow}
        self.mean = {False: normal_mean_s, True: slow_mean_s}
        self.slow = False
        self.state_left = self._draw()

    def _draw(self):
        mean = self.mean[self.slow]
        return self.rng.expovariate(1.0 / mean) if mean > 0 else float('inf')

    def next_period(self):
        """Seconds until the next frame, advancing the slow/normal state."""
        period = 1.0 / self.fps[self.slow]
        self.state_left -= period
        if self.state_left <= 0:
            self.slow = not self.slow
            self.state_left = self._draw()
        return period
