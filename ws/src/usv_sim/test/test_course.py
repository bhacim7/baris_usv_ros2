"""
Unit tests for the course file reader (usv_sim/course.py), no ROS needed.

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
# The whole 2026 course (Task 1-6: docks, vessels, yellow buoys), as roboboat_2026.yaml was
# before it became roboboat_2027: every element course.py can build
FULL_2026 = os.path.join(os.path.dirname(__file__), 'data', 'roboboat_2026_full.yaml')


def read(name):
    with open(os.path.join(COURSES, name + '.yaml')) as f:
        return yaml.safe_load(f)


def read_full_2026():
    with open(FULL_2026) as f:
        return yaml.safe_load(f)


def by_class(course):
    return Counter(course.class_names[t.class_id] for t in course.targets)


def roboboat_local(data=None):
    """Load the RoboBoat course as laid out in the file, before the anchor moves it."""
    data = data or read('roboboat_2027')
    data.pop('anchor', None)
    data.pop('placement', None)
    return course_mod.load(data)


def test_anchor_moves_and_turns_the_layout():
    origin = {'lat': 27.367256, 'lon': -82.451120}
    anchor = {'east': -185.0, 'north': 1088.0, 'rotation_deg': 90.0}
    e, n = course_mod.position({'east': 10.0, 'north': 0.0}, origin, anchor)
    assert (e, n) == (pytest.approx(-185.0), pytest.approx(1098.0))   # east turned to north
    assert course_mod.heading({'heading_deg': 90.0}, anchor) == 0.0


def test_placement_puts_the_boat_start_where_asked():
    data = read('roboboat_2027')
    c = course_mod.load(data)
    assert c.venue == 'nbpark'
    p = data['placement']
    assert c.boat['east'] == pytest.approx(p['east'])
    assert c.boat['north'] == pytest.approx(p['north'])
    assert c.boat['heading_deg'] == pytest.approx(p['heading_deg'])
    # Everything keeps its place relative to the boat: the first Task 1 gate stays 8 m
    # straight ahead of it
    local = roboboat_local()
    gate = [t for t in c.targets if c.class_names[t.class_id].startswith('pole')][:2]
    gate_local = [t for t in local.targets
                  if local.class_names[t.class_id].startswith('pole')][:2]
    for t, tl in zip(gate, gate_local):
        assert math.dist((t.x, t.y), (c.boat['east'], c.boat['north'])) == pytest.approx(
            math.dist((tl.x, tl.y), (local.boat['east'], local.boat['north'])))
    mid = ((gate[0].x + gate[1].x) / 2, (gate[0].y + gate[1].y) / 2)
    bearing = math.degrees(math.atan2(mid[0] - c.boat['east'], mid[1] - c.boat['north']))
    assert bearing % 360.0 == pytest.approx(p['heading_deg'], abs=1.0)


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


def test_roboboat_2027_has_the_readiness_tasks_and_the_task1_field():
    c = course_mod.load(read('roboboat_2027'))
    assert c.profile == 'roboboat' and c.class_names == ROBOBOAT
    # Readiness: Task 1 4 poles; Task 2 10 channel, 7 debris. 2027: 10 RoboBuoys
    assert len(c.buoys) == 31
    assert [m.name for m in c.models] == ['vessel_00_black']
    beacons = Counter(n for n in by_class(c).elements() if n.startswith('beacon'))
    assert sum(beacons.values()) == 10 and beacons['beacon_blue'] == 2
    assert {n: k for n, k in by_class(c).items() if not n.startswith('beacon')} == {
        'pole_red': 2, 'pole_green': 2, 'buoy_red': 4, 'buoy_green': 4, 'buoy_black': 7,
        'indicator_red': 1, 'indicator_green': 1, 'plus': 2}
    assert c.mission == {'roboboat.task_order': ['task2'], 'roboboat.hint.task2': '21.1,-1.5'}


def test_select_start_and_mission():
    data = read('roboboat_2027')
    data['missions']['roboboat_2027'] = {'roboboat.task_order': []}
    base = course_mod.load(data)
    chosen = course_mod.select(data, 'competition', 'roboboat_2027')
    assert 'starts' not in chosen and 'missions' not in chosen and 'default' not in chosen
    c = course_mod.load(chosen)
    # The layout stays where the placement puts it; only the boat starts elsewhere
    assert c.targets == base.targets and c.survey == base.survey
    assert c.mission == {'roboboat.task_order': []}
    # Where Baris picked it in Gazebo, heading west like the readiness start
    assert c.boat['east'] == pytest.approx(-197.80)
    assert c.boat['north'] == pytest.approx(809.98)
    assert c.boat['heading_deg'] % 360.0 == pytest.approx(270.0)
    assert base.boat['east'] == pytest.approx(-189.17)
    assert data['starts'] and data['placement']['start'] == 'readiness'   # input untouched


def test_task1_field_stands_ahead_of_the_competition_start():
    c = course_mod.load(course_mod.select(read('roboboat_2027'), 'competition'))
    field = [b for b in c.targets if getattr(b, 'light', None)]
    entry = next(b for b, (name, *_) in zip(c.targets, c.buoys) if name.endswith('entry'))
    # 12 m ahead (west in the park) of the start, the field further west
    assert entry.x == pytest.approx(c.boat['east'] - 12.0, abs=0.01)
    assert entry.y == pytest.approx(c.boat['north'], abs=0.01)
    assert all(b.x <= entry.x + 0.01 for b in field)
    assert all(abs(b.y - entry.y) <= 12.5 for b in field)
    # Far from the readiness course
    readiness = [b for b in c.targets if getattr(b, 'light', None) is None]
    assert min(abs(b.y - entry.y) for b in readiness) > 100.0


def test_select_refuses_what_the_course_does_not_have():
    data = read('roboboat_2027')
    with pytest.raises(ValueError, match='start finish: the course has competition, readiness'):
        course_mod.select(data, start='finish')
    with pytest.raises(ValueError, match='mission roboboat_2027'):
        course_mod.select(data, mission='roboboat_2027')
    golcuk = read('golcuk_2026_10_14')
    assert course_mod.select(golcuk) == golcuk
    with pytest.raises(ValueError, match='one start only'):
        course_mod.select(golcuk, start='readiness')


def test_full_2026_course_has_every_element():
    c = course_mod.load(read_full_2026())
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
    c = roboboat_local(read_full_2026())
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
    c = roboboat_local(read_full_2026())
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
           and c.class_names[t.class_id] == 'indicator_green' and t.x == 14.52]
    (t,) = ind
    assert t.radius == course_mod.INDICATOR_RADIUS
    assert t.height - t.z0 == pytest.approx(course_mod.INDICATOR_HEIGHT)


def test_survey_route_follows_the_placement():
    course = course_mod.load(read('roboboat_2027'))
    local = roboboat_local()
    assert len(course.survey) == len(local.survey) > 5
    assert any('spin_deg' in s for s in course.survey)
    # Same distances between the route and the buoys wherever the course is placed
    first = course.survey[0]
    first_local = local.survey[0]
    t, tl = course.targets[0], local.targets[0]
    assert math.hypot(first['east'] - t.x, first['north'] - t.y) == pytest.approx(
        math.hypot(first_local['east'] - tl.x, first_local['north'] - tl.y))
    assert first_local == {'east': 2.05, 'north': -1.0}


def test_survey_route_keeps_clear_of_the_buoys():
    """Straight legs between the points pass no buoy closer than its radius + 0.6 m."""
    course = roboboat_local()
    points = [(s['east'], s['north']) for s in course.survey if 'east' in s]
    points.insert(0, (course.boat['east'], course.boat['north']))
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        for b in course.targets:
            if not isinstance(b, Buoy):
                continue
            dx, dy = x2 - x1, y2 - y1
            f = max(0.0, min(1.0, ((b.x - x1) * dx + (b.y - y1) * dy) / (dx * dx + dy * dy)))
            gap = math.hypot(x1 + f * dx - b.x, y1 + f * dy - b.y)
            assert gap > b.radius + 0.6, (x1, y1, x2, y2, b)


def test_far_variant_hides_the_green_from_the_channel():
    """
    Check roboboat_2027_far: only the green indicator moves, out of reach from the channel.

    The channel counts as ended 3 m past gate 4 (Task2.run_on_m); from there the green has
    to be more than 14 m away (the mapper's 12 m and the turn there), and within reach of
    the debris scan: its far row runs 13.5 m past gate 4, 7 m to each side (DebrisConfig).
    """
    base, far = read('roboboat_2027'), read('roboboat_2027_far')
    assert {k: v for k, v in base.items() if k != 'buoys'} == \
        {k: v for k, v in far.items() if k != 'buoys'}
    moved = [(a, b) for a, b in zip(base['buoys'], far['buoys']) if a != b]
    assert len(base['buoys']) == len(far['buoys']) and len(moved) == 1
    assert moved[0][1]['type'] == 'indicator_green'
    green = (moved[0][1]['east'], moved[0][1]['north'])
    chan = [(b['east'], b['north']) for b in base['buoys'] if b['type'] in ('a0_red', 'a0_green')]
    g3 = ((chan[4][0] + chan[5][0]) / 2.0, (chan[4][1] + chan[5][1]) / 2.0)
    g4 = ((chan[6][0] + chan[7][0]) / 2.0, (chan[6][1] + chan[7][1]) / 2.0)
    n = math.hypot(g4[0] - g3[0], g4[1] - g3[1])
    ahead = ((g4[0] - g3[0]) / n, (g4[1] - g3[1]) / n)
    ended = (g4[0] + 3.0 * ahead[0], g4[1] + 3.0 * ahead[1])
    assert math.hypot(green[0] - ended[0], green[1] - ended[1]) > 14.0
    d = (green[0] - g4[0], green[1] - g4[1])
    along, side = d[0] * ahead[0] + d[1] * ahead[1], d[1] * ahead[0] - d[0] * ahead[1]
    assert math.hypot(along - 13.5, abs(side) - 7.0) <= 3.0
