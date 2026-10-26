"""Unit tests for the actuator models of sim_bridge (no ROS needed).

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_actuators.py -v
"""

import pytest

from usv_sim.actuators import PwmToThrust, ServoWriteThinning, SlewLimiter, steer_angle_rad

TABLE = PwmToThrust([1100.0, 1500.0, 1900.0], [-64.0, 0.0, 91.0])


# ---------------------------------------------------------------- PwmToThrust
def test_table_points_are_exact():
    assert TABLE(1100) == -64.0
    assert TABLE(1500) == 0.0
    assert TABLE(1900) == 91.0


def test_cruise_pwm_gives_the_thrust_used_for_the_drag_fit():
    # CRUISE_PWM 180 -> 180 / 400 * 91 N = 40.95 N (model.sdf surge drag header)
    assert TABLE(1680) == pytest.approx(40.95)


def test_reverse_is_linear_on_its_own_side():
    assert TABLE(1300) == pytest.approx(-32.0)


def test_outside_the_table_is_clamped():
    assert TABLE(900) == -64.0
    assert TABLE(2100) == 91.0


def test_table_must_increase():
    with pytest.raises(ValueError):
        PwmToThrust([1500.0, 1100.0], [0.0, -64.0])
    with pytest.raises(ValueError):
        PwmToThrust([1500.0], [0.0])


# ---------------------------------------------------------------- steering
def test_steering_sign_matches_the_measured_model():
    # 1900 = team "thrusters right, boat turns right" = -0.785 rad in Gazebo
    assert steer_angle_rad(1900) == pytest.approx(-0.785)
    assert steer_angle_rad(1100) == pytest.approx(0.785)
    assert steer_angle_rad(1500) == 0.0


def test_steering_stops_at_the_mechanical_limit():
    assert steer_angle_rad(2000) == pytest.approx(-0.785)
    assert steer_angle_rad(1000) == pytest.approx(0.785)


def test_half_steer_is_half_angle():
    assert steer_angle_rad(1700) == pytest.approx(-0.3925)


# ---------------------------------------------------------------- servo speed
def test_servo_needs_time_to_reach_full_lock():
    servo = SlewLimiter(5.2)
    for _ in range(5):                  # 5 x 20 ms = 0.1 s -> at most 0.52 rad
        servo.step(-0.785, 0.02)
    assert servo.value == pytest.approx(-0.52)
    for _ in range(50):
        servo.step(-0.785, 0.02)
    assert servo.value == pytest.approx(-0.785)


def test_servo_does_not_overshoot_and_ignores_negative_dt():
    servo = SlewLimiter(5.2, initial=0.1)
    assert servo.step(0.105, 0.02) == pytest.approx(0.105)
    assert servo.step(1.0, -0.5) == pytest.approx(0.105)


# ---------------------------------------------------------------- write thinning
def test_first_write_always_goes_out():
    ch = ServoWriteThinning()
    assert ch.write(1680, now=0.0) == 1680


def test_small_steps_are_held_until_the_refresh():
    ch = ServoWriteThinning(min_delta_us=3, refresh_s=0.5)
    ch.write(1680, now=0.0)
    assert ch.write(1682, now=0.1) == 1680       # 2 us: not sent
    assert ch.write(1684, now=0.2) == 1680       # 2 us from the last REQUEST: not sent
    assert ch.write(1684, now=0.6) == 1684       # refresh due


def test_big_steps_and_force_go_out_at_once():
    ch = ServoWriteThinning(min_delta_us=3, refresh_s=0.5)
    ch.write(1680, now=0.0)
    assert ch.write(1690, now=0.01) == 1690
    assert ch.write(1691, now=0.02, force=True) == 1691


def test_creeping_command_lags_like_the_legacy_code():
    # Legacy compares with the last REQUESTED value, so a 2 us/cycle ramp is only written
    # by the 0.5 s refresh. This is what the boat really does.
    ch = ServoWriteThinning(min_delta_us=3, refresh_s=0.5)
    ch.write(1500, now=0.0)
    held = [ch.write(1500 + 2 * k, now=0.02 * k) for k in range(1, 25)]   # up to 0.48 s
    assert held == [1500] * 24                   # 48 us asked for, nothing written
    assert ch.write(1550, now=0.5) == 1550       # the refresh catches up
