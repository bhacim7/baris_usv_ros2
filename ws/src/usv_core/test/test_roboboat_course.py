"""
The course model (usv_core.roboboat_course): course elements recognised on a world map.

The layout is the roboboat_2027 course file's (east / north metres, drawn heading north),
turned and moved to show that nothing depends on the course's orientation.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_roboboat_course.py -v
"""

import math
import random

import pytest
from usv_core.roboboat_course import CourseConfig, CourseModel, GateFrame

START = (2.0, -8.0)
LAYOUT = [
    ('pole_red', -2.28, 0.0), ('pole_green', 6.38, 0.0),
    ('pole_red', -2.28, 7.95), ('pole_green', 6.38, 7.95),
    ('buoy_red', 2.32, 21.13), ('buoy_green', 4.82, 21.13),
    ('buoy_red', 2.92, 27.13), ('buoy_green', 5.42, 27.13),
    ('buoy_red', 3.52, 33.13), ('buoy_green', 6.02, 33.13), ('buoy_black', 4.12, 33.33),
    ('buoy_red', 4.52, 39.13), ('buoy_green', 7.02, 39.13), ('buoy_black', 5.12, 39.43),
    ('buoy_black', 7.52, 45.13), ('buoy_black', 12.52, 47.13), ('buoy_black', 16.52, 52.13),
    ('buoy_black', 20.52, 55.13), ('buoy_black', 9.52, 54.13),
    ('indicator', 14.52, 49.13, 'green'), ('indicator', 18.52, 58.13, 'red'),
    ('buoy_red', 28.0, 28.25), ('buoy_green', 28.0, 25.75),
    ('indicator', 36.0, 27.0, 'red'), ('buoy_yellow', 48.0, 27.0),
    ('buoy_black', 45.0, 24.5), ('buoy_black', 40.0, 22.0),
    ('buoy_yellow', 23.0, -2.0),
    # Docks: S (x 16.5, slips open east), N (x 29.5, open west); banners 2 m apart from
    # y -12.3; indicator beside each banner; vessels 1 m in front of S1 and N3
    ('digit_1', 16.5, -12.3), ('digit_2', 16.5, -10.3), ('digit_3', 16.5, -8.3),
    ('indicator', 16.4, -11.85, 'red'), ('indicator', 16.4, -9.85, 'green'),
    ('indicator', 16.4, -7.85, 'red'), ('triangle', 17.5, -12.3),
    ('digit_1', 29.5, -12.3), ('digit_2', 29.5, -10.3), ('digit_3', 29.5, -8.3),
    ('indicator', 29.6, -11.85, 'green'), ('indicator', 29.6, -9.85, 'red'),
    ('indicator', 29.6, -7.85, 'red'), ('plus', 28.5, -8.3),
]


def landmarks(turn_deg=0.0, shift=(0.0, 0.0), noise=0.0, seed=1, drop=()):
    rng = random.Random(seed)
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    out = []
    for i, item in enumerate(LAYOUT):
        if i in drop:
            continue
        kind, x, y = item[:3]
        x, y = x + rng.gauss(0, noise), y + rng.gauss(0, noise)
        out.append({'id': i, 'kind': kind, 'color': item[3] if len(item) > 3 else '',
                    'east': shift[0] + x * c - y * s, 'north': shift[1] + x * s + y * c,
                    'confirmed': True, 'sightings': 50})
    return out


def placed(p, turn_deg=0.0, shift=(0.0, 0.0)):
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    return (shift[0] + p[0] * c - p[1] * s, shift[1] + p[0] * s + p[1] * c)


@pytest.mark.parametrize('turn', [0.0, 90.0, 213.0])
def test_task1_gates_and_frame(turn):
    shift = (-180.0, 950.0)
    v = CourseModel().view(landmarks(turn, shift, noise=0.3), start=placed(START, turn, shift))
    assert len(v.t1_gates) == 2
    g1, g2 = v.t1_gates
    assert {g1.red['id'], g1.green['id']} == {0, 1}           # the first gate is the near one
    ox, oy = placed((2.05, 0.0), turn, shift)
    assert math.hypot(v.frame.origin[0] - ox, v.frame.origin[1] - oy) < 0.6
    ahead_true = placed((0.0, 1.0), turn)
    assert v.frame.ahead[0] * ahead_true[0] + v.frame.ahead[1] * ahead_true[1] > 0.99
    # Hints read in the gate frame land where the course file says. With 0.3 m landmark
    # noise the frame's direction is off by ~1-2 deg: 21 m away that is 0.5 m on average,
    # 1.5 m at worst (200 draws); a hint only says where to start looking
    e, n = v.expected((21.1, -1.5))
    tx, ty = placed((3.57, 21.13), turn, shift)
    assert math.hypot(e - tx, n - ty) < 1.5


def test_gate_frame_round_trip():
    f = GateFrame((10.0, 20.0), (0.0, 1.0))           # ahead = north, left = west
    assert f.to_map(5.0, 2.0) == pytest.approx((8.0, 25.0))
    assert f.from_map(8.0, 25.0) == pytest.approx((5.0, 2.0))


def test_only_one_task1_gate_seen_points_through_it():
    v = CourseModel().view(landmarks(drop=(2, 3)), start=START)
    assert len(v.t1_gates) == 1 and 't1_second_gate' in v.missing
    assert v.frame.ahead == pytest.approx((0.0, 1.0), abs=1e-6)


def test_channel_and_debris_field():
    v = CourseModel().view(landmarks(37.0, noise=0.2), start=placed(START, 37.0))
    assert len(v.channel) == 4
    first = v.channel[0].centre
    assert math.hypot(first[0] - placed((3.57, 21.13), 37.0)[0],
                      first[1] - placed((3.57, 21.13), 37.0)[1]) < 0.6
    debris_ids = {lm['id'] for lm in v.debris}
    assert {14, 15, 16, 17, 18, 19, 20} <= debris_ids        # field past the channel
    assert 10 not in debris_ids and 13 not in debris_ids      # black buoys inside the channel


def test_task3_gate_yellow_and_indicator():
    v = CourseModel().view(landmarks(), start=START)
    assert {v.task3_gate.red['id'], v.task3_gate.green['id']} == {21, 22}
    assert v.task3_yellow['id'] == 24 and v.task3_indicator == 'red'
    assert v.zones['task6_zone_t3']['id'] == 24


def test_marina_slips_and_the_best_one():
    v = CourseModel().view(landmarks(250.0, noise=0.15), start=placed(START, 250.0))
    slips = {(s.dock, s.number): s for s in v.slips}
    assert len(slips) == 6
    # Which dock is "N" depends on the compass; turned 250 deg, the x 29.5 dock is south
    north_dock = 'N' if placed((29.5, -10.0), 250.0)[1] > placed((16.5, -10.0), 250.0)[1] \
        else 'S'
    south_dock = 'S' if north_dock == 'N' else 'N'
    assert slips[(south_dock, 1)].occupied and slips[(north_dock, 3)].occupied
    assert slips[(south_dock, 2)].indicator == 'green' and not slips[(south_dock, 2)].occupied
    assert v.marina_complete and 'marina_incomplete' not in v.missing
    assert v.best_slip.number == 1 and v.best_slip.dock == north_dock
    assert v.zones['task6_zone_marina']['id'] == 27


def test_unconfirmed_landmarks_are_left_out_and_missing_listed():
    lms = landmarks()
    for lm in lms:
        if lm['kind'].startswith('pole'):
            lm['confirmed'] = False
    v = CourseModel().view(lms, start=START)
    assert v.frame is None and 't1_gates' in v.missing and v.expected((1.0, 1.0)) is None
    assert CourseModel(CourseConfig(use_unconfirmed=True)).view(lms, START).frame is not None
    empty = CourseModel().view([], start=START)
    assert set(empty.missing) >= {'t1_gates', 'channel', 'task3', 'marina'}


def test_one_dock_seen_has_no_name_yet():
    north_dock = [i for i, item in enumerate(LAYOUT) if item[0] in
                  ('digit_1', 'digit_2', 'digit_3', 'indicator', 'plus') and item[1] > 29.0]
    v = CourseModel().view(landmarks(drop=north_dock), start=START)
    assert len(v.slips) == 3 and {s.dock for s in v.slips} == {''}
    assert 'marina_second_dock' in v.missing and 'marina_incomplete' in v.missing
    # S2 is green and free, but the unseen dock could have a lower free number: no answer
    assert v.best_slip is None and v.best_slip_so_far.number == 2
