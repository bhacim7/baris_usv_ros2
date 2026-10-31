"""Unit tests for the course file reader (usv_sim/course.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_course.py -v
"""

from collections import Counter
import math
import os

import pytest
from usv_core.classes import ROBOBOAT
from usv_sim import course as course_mod
from usv_sim.camera import Buoy, Panel
from usv_sim.geo import enu
import yaml

COURSES = os.path.join(os.path.dirname(__file__), '..', 'courses')


def read(name):
    with open(os.path.join(COURSES, name + '.yaml')) as f:
        return yaml.safe_load(f)


def by_class(course):
    return Counter(course.class_names[t.class_id] for t in course.targets)


def roboboat_local():
    """The RoboBoat course as laid out in the file, before the anchor moves it."""
    data = read('roboboat_2026')
    del data['anchor']
    return course_mod.load(data)


def test_anchor_moves_and_turns_the_layout():
    origin = {'lat': 27.367256, 'lon': -82.451120}
    anchor = {'east': -185.0, 'north': 1088.0, 'rotation_deg': 90.0}
    e, n = course_mod.position({'east': 10.0, 'north': 0.0}, origin, anchor)
    assert (e, n) == (pytest.approx(-185.0), pytest.approx(1098.0))   # east turned to north
    assert course_mod.heading({'heading_deg': 90.0}, anchor) == 0.0
    c = course_mod.load(read('roboboat_2026'))
    assert c.venue == 'nbpark'
    assert c.boat['east'] == pytest.approx(-185.0 + 2.0)
    assert c.boat['north'] == pytest.approx(1088.0 - 8.0)


def test_golcuk_targets_are_what_fake_perception_built_before():
    data = read('golcuk_2026_10_14')
    c = course_mod.load(data)
    assert c.profile == 'teknofest' and c.models == []
    o = data['origin']
    expected = []
    for b in data['buoys']:
        spec = data['buoy_types'][b['type']]
        x, y = enu(b['lat'], b['lon'], o['lat'], o['lon'])
        expected.append(Buoy(x, y, spec['diameter'] / 2.0, spec['height'], spec['class_id']))
    assert c.targets == expected


def test_roboboat_course_has_every_element():
    c = course_mod.load(read('roboboat_2026'))
    assert c.profile == 'roboboat' and c.class_names == ROBOBOAT
    assert len(c.buoys) == 28
    assert len(c.models) == 8                  # 4 vessels, 2 docks, 2 vessels in slips
    assert by_class(c) == {
        'pole_red': 2, 'pole_green': 2, 'buoy_red': 5, 'buoy_green': 5, 'buoy_black': 9,
        'buoy_yellow': 2, 'indicator_red': 6, 'indicator_green': 3,
        'triangle': 6, 'plus': 6, 'digit_1': 2, 'digit_2': 2, 'digit_3': 2}


def test_positions_in_metres_or_degrees():
    origin = {'lat': 27.367256, 'lon': -82.451120}
    assert course_mod.position({'east': 3.0, 'north': -4.0}, origin) == (3.0, -4.0)
    lat, lon = 27.367256 + 1e-4, -82.451120
    e, n = course_mod.position({'lat': lat, 'lon': lon}, origin)
    assert e == pytest.approx(0.0, abs=1e-6) and n == pytest.approx(11.07, abs=0.05)


def test_dock_banners_face_into_their_slips():
    c = roboboat_local()
    banners = [t for t in c.targets
               if isinstance(t, Panel) and c.class_names[t.class_id].startswith('digit')]
    south = sorted((t for t in banners if t.x < 23.0), key=lambda t: t.y)
    north = sorted((t for t in banners if t.x > 23.0), key=lambda t: t.y)
    # S dock: spine along x = 16 going north, slips open to the east
    assert [c.class_names[t.class_id] for t in south] == ['digit_1', 'digit_2', 'digit_3']
    assert all(t.normal_deg % 360.0 == pytest.approx(0.0) for t in south)
    assert all(t.x == pytest.approx(16.0 + course_mod.SPINE_WIDTH / 2.0) for t in south)
    # N dock: spine along x = 30, slips open to the west
    assert all(t.normal_deg % 360.0 == pytest.approx(180.0) for t in north)
    assert south[1].y - south[0].y == pytest.approx(course_mod.CUBE + course_mod.SLIP_WIDTH)


def test_moored_vessel_lies_in_its_slip_bow_to_the_spine():
    c = roboboat_local()
    m = next(m for m in c.models if m.name == 'dock_S_slip1_yellow')
    first_slip_y = -14.0 + course_mod.FIRST_TINE_X + course_mod.CUBE + \
        course_mod.SLIP_WIDTH / 2.0
    assert m.y == pytest.approx(first_slip_y)
    assert m.x > 16.0 + course_mod.SPINE_WIDTH / 2.0           # out in the slip, east
    assert math.degrees(m.yaw) % 360.0 == pytest.approx(180.0)  # pointing west, at the spine


def test_vessel_marks_are_on_both_sides():
    yaw = course_mod.compass_to_yaw(40.0)
    _, panels = course_mod.vessel('v', 'yellow', 10.0, 5.0, yaw, ROBOBOAT)
    assert [ROBOBOAT[p.class_id] for p in panels] == ['triangle', 'triangle']
    normals = sorted(p.normal_deg % 360.0 for p in panels)
    assert normals[1] - normals[0] == pytest.approx(180.0)
    assert all(math.hypot(p.x - 10.0, p.y - 5.0) == pytest.approx(
        course_mod.VESSEL_WIDTH / 2.0 + 0.003) for p in panels)
    _, panels = course_mod.vessel('v', 'black', 0.0, 0.0, 0.0, ROBOBOAT)
    assert {ROBOBOAT[p.class_id] for p in panels} == {'plus'}


def test_indicator_buoy_target_is_the_cylinder_on_top():
    c = roboboat_local()
    ind = [t for t in c.targets if isinstance(t, Buoy)
           and c.class_names[t.class_id] == 'indicator_green' and t.x == 13.0]
    (t,) = ind
    assert t.radius == course_mod.INDICATOR_RADIUS
    assert t.height - t.z0 == pytest.approx(course_mod.INDICATOR_HEIGHT)
