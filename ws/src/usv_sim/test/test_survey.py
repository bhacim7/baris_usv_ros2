"""
Unit tests for the survey pilot (usv_sim/survey.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_survey.py -v
"""

import math
import os

from usv_sim import course as course_mod
from usv_sim.survey import compass_bearing, SurveyPilot, wrap180
import yaml

COURSES = os.path.join(os.path.dirname(__file__), '..', 'courses')


def drive(pilot, east, north, heading, seconds=600.0, dt=0.04):
    """Drive a rough boat: speed from the forward PWM, turn rate from the yaw PWM."""
    track = []
    t = 0.0
    while not pilot.done and t < seconds:
        forward, yaw = pilot.step(t, east, north, heading)
        speed = max(0.0, (forward - 1500) / 180.0 * 1.3)
        heading = (heading + yaw * 0.25 * dt) % 360.0         # 200 PWM = 50 deg/s
        east += speed * math.sin(math.radians(heading)) * dt
        north += speed * math.cos(math.radians(heading)) * dt
        track.append((t, east, north, heading))
        t += dt
    return track


def test_helpers():
    assert wrap180(350.0) == -10.0
    assert wrap180(-190.0) == 170.0
    assert compass_bearing(0, 0, 1, 0) == 90.0
    assert compass_bearing(0, 0, 0, -1) == 180.0


def test_reaches_every_point_and_spins():
    steps = [{'east': 0.0, 'north': 10.0}, {'spin_deg': 360.0},
             {'east': 10.0, 'north': 10.0}, {'spin_deg': -90.0}, {'east': 10.0, 'north': 0.0}]
    pilot = SurveyPilot(steps)
    track = drive(pilot, 0.0, 0.0, 0.0)
    assert pilot.done
    assert [m for _, m in pilot.log if 'timed out' in m] == []
    assert len(pilot.log) == len(steps)
    assert 'turned +36' in pilot.log[1][1]
    assert 'turned -9' in pilot.log[3][1]
    _, e, n, _ = track[-1]
    assert math.hypot(e - 10.0, n - 0.0) < 2.0


def test_stays_on_the_leg():
    """Starting 2 m beside a long leg, the boat joins the line instead of cutting across."""
    pilot = SurveyPilot([{'east': 0.0, 'north': 30.0}])
    pilot.leg_start = (0.0, 0.0)       # the leg runs north along east = 0
    track = drive(pilot, 2.0, 0.0, 0.0)
    assert pilot.done
    late = [abs(e) for _, e, n, _ in track if n > 12.0]
    assert max(late) < 0.3


def test_roboboat_survey_route_finishes():
    with open(os.path.join(COURSES, 'roboboat_2027.yaml')) as f:
        course = course_mod.load(yaml.safe_load(f))
    pilot = SurveyPilot(course.survey)
    drive(pilot, course.boat['east'], course.boat['north'], course.boat['heading_deg'])
    assert pilot.done
    assert [m for _, m in pilot.log if 'timed out' in m] == []
