"""Unit tests for the ZED 2i models of fake_perception (no ROS needed).

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_zed.py -v
"""

import math

import pytest

from usv_sim.sensors import make_rng
from usv_sim.zed import FrameClock, magnetic_heading_deg, rotate

# Earth field in ENU (T): declination +6 deg, inclination 56.5 deg, 46.5 uT
_H = 46.5e-6 * math.cos(math.radians(56.5))
FIELD_ENU = (_H * math.sin(math.radians(6.0)), _H * math.cos(math.radians(6.0)),
             -46.5e-6 * math.sin(math.radians(56.5)))


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    """Body -> ENU quaternion (x, y, z, w) from roll, pitch, yaw in degrees (ZYX)."""
    r, p, y = (math.radians(a) / 2 for a in (roll, pitch, yaw))
    return (math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
            math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
            math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
            math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y))


def body_field(q):
    """What a body-fixed magnetometer reads: the world field turned into the body frame."""
    x, y, z, w = q
    return rotate((-x, -y, -z, w), FIELD_ENU)


def compass_to_enu_yaw(heading_deg):
    return 90.0 - heading_deg


@pytest.mark.parametrize('true_heading', [0.0, 45.0, 90.0, 146.0, 200.0, 359.0])
def test_level_boat_reads_true_heading_minus_declination(true_heading):
    q = quat(yaw=compass_to_enu_yaw(true_heading))
    expected = (true_heading - 6.0) % 360.0
    got = magnetic_heading_deg(body_field(q), q)
    assert min(abs(got - expected), 360 - abs(got - expected)) < 1e-6


@pytest.mark.parametrize('roll, pitch', [(10, 0), (0, -8), (15, 12), (-20, 5)])
def test_rolling_and_pitching_do_not_move_the_heading(roll, pitch):
    q = quat(roll=roll, pitch=pitch, yaw=compass_to_enu_yaw(146.0))
    assert magnetic_heading_deg(body_field(q), q) == pytest.approx(140.0, abs=1e-6)


def test_hard_iron_error_depends_on_heading():
    # A field fixed to the boat, 5 uT to the left: bends the reading differently per heading
    errors = []
    for h in (0.0, 90.0, 180.0, 270.0):
        q = quat(yaw=compass_to_enu_yaw(h))
        got = magnetic_heading_deg(body_field(q), q, hard_iron_body=(0.0, 5e-6, 0.0))
        err = (got - (h - 6.0) + 180.0) % 360.0 - 180.0
        errors.append(err)
    assert max(abs(e) for e in errors) > 5.0
    assert max(errors) - min(errors) > 1.0      # not a constant offset


def test_rotate_is_a_proper_rotation():
    q = quat(roll=30, pitch=-20, yaw=75)
    v = (1.0, 2.0, 3.0)
    out = rotate(q, v)
    assert math.hypot(*out) == pytest.approx(math.hypot(*v))
    x, y, z, w = q
    assert rotate((-x, -y, -z, w), out) == pytest.approx(v)


def test_frame_clock_spends_most_time_at_full_rate():
    clock = FrameClock(make_rng(5), fps_normal=30.0, fps_slow=15.0,
                       normal_mean_s=20.0, slow_mean_s=2.0)
    t_fast = t_slow = 0.0
    for _ in range(200000):
        period = clock.next_period()
        if period == pytest.approx(1 / 30.0):
            t_fast += period
        else:
            t_slow += period
    share_slow = t_slow / (t_fast + t_slow)
    assert 0.04 < share_slow < 0.16             # expected 2 / 22 = 9 %


def test_frame_clock_without_slow_spells():
    clock = FrameClock(make_rng(5), normal_mean_s=0.0)
    assert all(clock.next_period() == pytest.approx(1 / 30.0) for _ in range(1000))
