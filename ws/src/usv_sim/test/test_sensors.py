"""Unit tests for the sensor error models of sim_bridge (no ROS needed).

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_sensors.py -v
"""

import math
import statistics

import pytest

from usv_sim.sensors import (
    compass_deg, DelayLine, GaussMarkov, make_rng, offset_latlon, quantize,
    roll_pitch_from_quaternion, yaw_from_quaternion)


# ---------------------------------------------------------------- headings
def test_compass_from_enu_yaw():
    assert compass_deg(0.0) == pytest.approx(90.0)             # facing east
    assert compass_deg(math.pi / 2) == pytest.approx(0.0)      # facing north
    assert compass_deg(-math.pi / 2) == pytest.approx(180.0)   # facing south
    assert compass_deg(math.pi) == pytest.approx(270.0)        # facing west


def test_turning_right_increases_the_compass_heading():
    # Turning right = clockwise from above = ENU yaw decreasing
    assert compass_deg(math.radians(80)) > compass_deg(math.radians(90))


def _quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def test_yaw_from_quaternion_round_trip():
    for deg in (-170, -90, -30, 0, 45, 135, 179):
        assert yaw_from_quaternion(*_quat_from_yaw(math.radians(deg))) == pytest.approx(
            math.radians(deg))


def test_roll_and_pitch_from_quaternion():
    a = math.radians(10)
    roll, pitch = roll_pitch_from_quaternion(math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))
    assert roll == pytest.approx(a)
    assert pitch == pytest.approx(0.0)


# ---------------------------------------------------------------- position
def test_one_metre_north_and_east():
    lat, lon = offset_latlon(40.8, 29.26, 1.0, 1.0)
    assert (lat - 40.8) * 111320 == pytest.approx(1.0, rel=0.01)
    assert (lon - 29.26) * 111320 * math.cos(math.radians(40.8)) == pytest.approx(1.0, rel=0.01)


def test_mavlink_quantization():
    assert quantize(40.80907354, 1e-7) == pytest.approx(40.8090735, abs=1e-12)
    assert quantize(359.996, 0.01) == pytest.approx(360.0)
    assert quantize(1.23, 0) == 1.23


# ---------------------------------------------------------------- noise
def test_gauss_markov_has_the_requested_spread_and_memory():
    rng = make_rng(1)
    gm = GaussMarkov(sigma=1.0, tau_s=60.0, rng=rng)
    values = [gm.step(0.5) for _ in range(40000)]          # 20 000 s of samples
    assert statistics.pstdev(values) == pytest.approx(1.0, rel=0.1)
    # Correlated: two samples 0.5 s apart are nearly the same (exp(-0.5/60) = 0.99)
    diffs = [b - a for a, b in zip(values, values[1:])]
    assert statistics.pstdev(diffs) < 0.2


def test_zero_sigma_means_no_error():
    gm = GaussMarkov(sigma=0.0, tau_s=60.0, rng=make_rng(1))
    assert gm.step(1.0) == 0.0


def test_seed_makes_runs_repeatable():
    a = GaussMarkov(1.0, 60.0, make_rng(7))
    b = GaussMarkov(1.0, 60.0, make_rng(7))
    assert [a.step(0.5) for _ in range(5)] == [b.step(0.5) for _ in range(5)]


# ---------------------------------------------------------------- delay
def test_delay_line_hides_fresh_values():
    d = DelayLine(0.1)
    assert d.latest(0.0) is None
    d.push(1.00, 'a')
    assert d.latest(1.05) is None           # not old enough yet
    assert d.latest(1.10) == 'a'
    d.push(1.20, 'b')
    assert d.latest(1.25) == 'a'            # keeps the last visible value
    assert d.latest(1.31) == 'b'
