"""
Unit tests for the random Task 1 field (usv_sim/task1_field.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_task1_field.py -v
"""

from collections import Counter
import math

import pytest
from usv_sim import task1_field as tf

SEEDS = range(60)


def side(pts, b):
    """+1 if the buoy is left of the path where it stands, -1 if right."""
    c, left = tf.along(pts, min(max(b['x'], 0.0), pts[-1][0]))
    return 1 if (b['x'] - c[0]) * left[0] + (b['y'] - c[1]) * left[1] > 0 else -1


@pytest.mark.parametrize('level', tf.LEVELS)
def test_every_field_has_its_buoys_and_a_clear_path(level):
    for seed in SEEDS:
        pts, buoys = tf.layout(seed, level=level)
        roles = Counter(b['role'] for b in buoys)
        assert len(buoys) == 10 and roles['entry'] == 1 and roles['exit'] == 1
        assert roles['red'] >= 2 and roles['green'] >= 2
        for b in buoys:
            assert 0.0 <= b['x'] <= 40.0 and abs(b['y']) <= 12.5
            others = [o for o in buoys if o is not b]
            assert min(math.hypot(o['x'] - b['x'], o['y'] - b['y']) for o in others) >= 4.0
            if b['role'] == 'green':
                assert side(pts, b) == 1 and tf.dist_to_path((b['x'], b['y']), pts) >= 1.6
            elif b['role'] == 'red':
                assert side(pts, b) == -1 and tf.dist_to_path((b['x'], b['y']), pts) >= 1.6
            elif b['role'] in ('off', 'next_exit'):
                assert tf.dist_to_path((b['x'], b['y']), pts) >= 3.0
        for b in buoys:
            if b['role'] not in ('entry', 'exit'):
                for e in buoys[:2]:
                    assert math.hypot(e['x'] - b['x'], e['y'] - b['y']) >= 6.0


def test_lights_follow_the_rules():
    _, buoys = tf.layout(3)
    light = {b['role']: b['light'] for b in buoys}
    assert light['entry']['color'] == 'blue' and light['entry']['flash']
    assert light['exit']['color'] == 'blue' and not light['exit']['flash']
    assert light['red']['flash'] and light['green']['flash']
    assert light['off']['color'] == 'off'


def test_same_seed_same_field_other_seed_other_field():
    assert tf.layout(7) == tf.layout(7)
    assert tf.layout(7)[1] != tf.layout(8)[1]


def test_advanced_moves_the_exit_when_the_boat_reaches_the_middle():
    for seed in SEEDS:
        _, buoys = tf.layout(seed, level='advanced')
        nxt = [b for b in buoys if b['role'] == 'next_exit']
        assert len(nxt) == 1 and nxt[0]['x'] >= 24.0
        assert nxt[0]['light']['color'] == 'off'
        assert nxt[0]['light']['changes'][0]['color'] == 'blue'
        assert buoys[1]['light']['changes'][0]['color'] == 'off'
        assert nxt[0]['light']['changes'][0]['near'] == buoys[1]['light']['changes'][0]['near']


def test_disruptive_swaps_at_most_two_colours_past_the_middle():
    swapped = 0
    for seed in SEEDS:
        _, buoys = tf.layout(seed, level='disruptive')
        swaps = [b for b in buoys if 'swaps_to' in b]
        assert len(swaps) <= 2
        for b in swaps:
            assert b['x'] > 20.0 and b['swaps_to'] != b['role']
            assert b['light']['changes'][0]['color'] == b['swaps_to']
        swapped += len(swaps)
    assert swapped >= len(SEEDS)              # most fields get some


def test_course_entries_turn_the_field_into_the_file_frame():
    field = {'seed': 2, 'entry': {'east': 100.0, 'north': 50.0}, 'heading_deg': 90.0,
             'level': 'advanced'}
    entries = tf.course_entries(field)
    _, buoys = tf.layout(2, level='advanced')
    assert len(entries) == 10 and entries[0]['name'] == 'task1_00_entry'
    assert (entries[0]['east'], entries[0]['north']) == (100.0, 50.0)
    # Heading east: ahead = +east, left = +north
    for e, b in zip(entries, buoys):
        assert e['east'] == pytest.approx(100.0 + b['x'], abs=1e-3)
        assert e['north'] == pytest.approx(50.0 + b['y'], abs=1e-3)
    near = entries[1]['light']['changes'][0]['near']
    assert near['east'] == pytest.approx(120.0, abs=1e-3)


def test_wrong_settings_are_refused():
    with pytest.raises(ValueError):
        tf.layout(1, level='expert')
    with pytest.raises(ValueError):
        tf.layout(1, buoys=4)
