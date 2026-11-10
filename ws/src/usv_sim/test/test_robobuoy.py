"""
Unit tests for the RoboBuoy light (usv_sim/robobuoy.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_robobuoy.py -v
"""

import pytest
from usv_core.classes import ROBOBOAT
from usv_sim import robobuoy
from usv_sim.camera import Buoy


def test_sizes_follow_the_build_guide():
    assert robobuoy.PANEL_BASE == pytest.approx(0.4318)
    assert 0.24 < robobuoy.PANEL_HEIGHT < 0.27          # 12 in panels leaning in
    assert 0.40 < robobuoy.TOP < 0.50                   # top of the sun hat


def test_flashing_light_is_off_half_of_the_time():
    light = robobuoy.Light('green', flash=True)
    states = [light.state(k * 0.1) for k in range(40)]   # 4 s
    assert states.count('green') == 20 and states.count('off') == 20
    assert light.state(0.5) == 'green' and light.state(1.5) == 'off'
    late = robobuoy.Light('green', flash=True, phase_s=1.0)
    assert late.state(0.5) == 'off'


def test_solid_and_off_lights():
    assert all(robobuoy.Light('blue').state(t) == 'blue' for t in (0.0, 1.2, 7.7))
    assert robobuoy.Light('off', flash=True).state(0.2) == 'off'
    with pytest.raises(ValueError):
        robobuoy.Light('yellow')


def test_changes_by_time_and_by_the_boat_coming_near():
    light = robobuoy.parse_light(
        {'color': 'blue', 'changes': [{'color': 'off', 'near': {'east': 10.0, 'north': 0.0,
                                                                'within_m': 3.0}}]},
        lambda p: (p['east'] + 100.0, p['north']))          # file -> world
    assert light.state(5.0, (100.0, 0.0)) == 'blue'           # 10 m away
    assert light.state(6.0, (108.0, 0.0)) == 'off'            # within 3 m of (110, 0)
    assert light.state(7.0, (100.0, 0.0)) == 'off'            # and it stays so
    timed = robobuoy.parse_light({'color': 'red', 'changes': [
        {'color': 'green', 'flash': True, 'at_s': 30.0}]}, None)
    assert timed.state(29.0) == 'red' and timed.state(30.2) == 'green'


def test_lit_targets_give_the_class_of_the_moment():
    names = ROBOBOAT
    light = robobuoy.Light('red', flash=True)
    b = Buoy(0.0, 0.0, 0.24, 0.44, names.index('beacon_red'), light=light)
    other = Buoy(1.0, 0.0, 0.1, 0.15, names.index('buoy_black'))
    (on, same), = [robobuoy.lit_targets([b, other], names, 0.3)]
    assert names[on.class_id] == 'beacon_red' and same is other
    off = robobuoy.lit_targets([b], names, 1.3)[0]
    assert names[off.class_id] == 'beacon_off'
    assert b.class_id == names.index('beacon_red')         # the course's target unchanged
