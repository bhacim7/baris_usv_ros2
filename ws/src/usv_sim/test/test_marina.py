"""
Unit tests for the RoboBoat 2027 marina (usv_sim/marina.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_marina.py -v
"""

import math

import pytest
from usv_core.classes import ROBOBOAT
from usv_sim import course as course_mod, lights, marina

SPEC = {'name': 'M', 'bays': [{'indicator': 'red'}, {'indicator': 'green', 'window': 2},
                              {'indicator': 'red', 'vessel': 'yellow'}]}


def build(spec=SPEC, heading=270.0):
    return marina.build(spec, 0.0, 0.0, heading, ROBOBOAT, course_mod.Part, course_mod.Model)


def test_dock_size_follows_the_drawing():
    dock, _, _ = build()
    deck = dock.parts[0]
    assert deck.size[1] == pytest.approx(6.596)               # 13 cubes long
    assert deck.size[0] == pytest.approx(2 * 6.596 / 13)      # 2 cubes across
    fingers = dock.parts[1:5]
    assert all(f.size[0] == pytest.approx(4 * 6.596 / 13) for f in fingers)   # 2.0 m bays
    gaps = sorted(f.pose[1] for f in fingers)
    assert gaps[1] - gaps[0] - 6.596 / 13 == pytest.approx(1.522, abs=0.01)   # 1.5 m wide


def test_bays_numbered_from_the_left_seen_from_the_water():
    # Bays open west: standing in the water looking east, left is north
    _, targets, _ = build()
    indicators = [t for t in targets if ROBOBOAT[t.class_id].startswith('indicator')]
    assert [ROBOBOAT[t.class_id] for t in indicators] == [
        'indicator_red', 'indicator_green', 'indicator_red']
    ys = [t.y for t in indicators]
    assert ys[0] > ys[1] > ys[2]                                # bay 1 north
    assert all(t.x == pytest.approx(-0.08) for t in indicators)  # just out from the front


def test_windows_face_into_the_bays_at_the_drawn_heights():
    _, targets, _ = build()
    windows = [t for t in targets if ROBOBOAT[t.class_id].startswith('window')]
    assert len(windows) == 6
    assert all(w.normal_deg % 360.0 == pytest.approx(180.0) for w in windows)   # west
    w1, w2 = windows[0], windows[1]
    assert (w1.z0, w1.z1) == (pytest.approx(0.15 + 0.605), pytest.approx(0.15 + 0.895))
    assert (w1.width, w2.width) == (pytest.approx(0.21), pytest.approx(0.28))
    assert w1.y > w2.y                         # window 1 on the left seen from the bay


def test_only_the_green_bays_chosen_window_lights():
    _, targets, _ = build()
    windows = [t for t in targets if ROBOBOAT[t.class_id].startswith('window')]
    lit = [k for k, w in enumerate(windows) if w.light.changes]
    assert lit == [3]                          # bay 2, window 2


def test_fire_sequence_core_and_disruptive():
    _, targets, _ = build()
    w = next(t for t in targets if getattr(t, 'light', None) and t.light.changes)
    bay = w.light.changes[0].near[:2]
    assert w.light.state(0.0, bay) == 'off'
    assert w.light.state(3.0, bay) == 'red'
    lights.fire(targets, 'fire_out')
    assert w.light.state(10.0, bay) == 'green'
    assert w.light.state(15.0, bay) == 'off'
    assert w.light.state(500.0, bay) == 'off'
    _, targets, _ = build(dict(SPEC, level='disruptive', sequence=['blue', 'red']))
    w = next(t for t in targets if getattr(t, 'light', None) and t.light.changes)
    w.light.state(0.0, bay)
    w.light.state(3.0, bay)
    lights.fire(targets, 'fire_out')
    seq = [w.light.state(10.0 + k, bay) for k in range(12)]
    assert seq == ['green'] * 5 + ['off', 'blue', 'off', 'red', 'off', 'off', 'blue']


def test_vessel_moored_in_its_bay_and_wrong_specs_refused():
    _, _, moored = build()
    (name, color, x, y, yaw), = moored
    assert color == 'yellow' and name == 'marina_bay3_yellow'
    assert x == pytest.approx(-0.95) and y < 0.0
    assert math.cos(yaw) == pytest.approx(1.0)         # bow east, at the building
    with pytest.raises(ValueError):
        build({'bays': SPEC['bays'][:2]})
    with pytest.raises(ValueError):
        build(dict(SPEC, level='advanced'))
