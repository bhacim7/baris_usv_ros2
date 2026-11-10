"""
Unit tests for the course lights (usv_sim/lights.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_lights.py -v
"""

import pytest
from usv_core.classes import ROBOBOAT
from usv_sim import lights
from usv_sim.camera import Buoy, Panel


def test_flashing_light_is_off_half_of_the_time():
    light = lights.Light('green', flash=True)
    states = [light.state(k * 0.1) for k in range(40)]   # 4 s
    assert states.count('green') == 20 and states.count('off') == 20
    assert light.state(0.5) == 'green' and light.state(1.5) == 'off'
    late = lights.Light('green', flash=True, phase_s=1.0)
    assert late.state(0.5) == 'off'


def test_solid_and_off_lights():
    assert all(lights.Light('blue').state(t) == 'blue' for t in (0.0, 1.2, 7.7))
    assert lights.Light('off', flash=True).state(0.2) == 'off'
    with pytest.raises(ValueError):
        lights.Light('yellow')


def test_changes_by_time_and_by_the_boat_coming_near():
    light = lights.parse_light(
        {'color': 'blue', 'changes': [{'color': 'off', 'near': {'east': 10.0, 'north': 0.0,
                                                                'within_m': 3.0}}]},
        lambda p: (p['east'] + 100.0, p['north']))          # file -> world
    assert light.state(5.0, (100.0, 0.0)) == 'blue'           # 10 m away
    assert light.state(6.0, (108.0, 0.0)) == 'off'            # within 3 m of (110, 0)
    assert light.state(7.0, (100.0, 0.0)) == 'off'            # and it stays so
    timed = lights.parse_light({'color': 'red', 'changes': [
        {'color': 'green', 'flash': True, 'at_s': 30.0}]}, None)
    assert timed.state(29.0) == 'red' and timed.state(30.2) == 'green'


def test_near_with_hold_needs_the_boat_to_stay():
    light = lights.Light('off', changes=[lights.Change([('red', None)], near=(0.0, 0.0, 1.0),
                                                       hold_s=3.0)])
    assert light.state(0.0, (0.5, 0.0)) == 'off'
    assert light.state(2.0, (5.0, 0.0)) == 'off'             # left: the clock restarts
    assert light.state(3.0, (0.2, 0.0)) == 'off'
    assert light.state(5.9, (0.2, 0.0)) == 'off'
    assert light.state(6.0, (0.2, 0.0)) == 'red'


def test_program_steps_and_event_after_an_earlier_change():
    light = lights.Light('off', changes=[
        lights.Change([('red', None)], at_s=10.0),
        lights.Change([('green', 5.0), ('blue', 1.0), ('off', None)], event='fire_out',
                      after=0)])
    light.fire('fire_out')                    # too early: waits for change 0
    assert light.state(5.0) == 'off'
    assert light.state(10.0) == 'green'       # change 0 and then at once change 1
    assert light.state(14.9) == 'green'
    assert light.state(15.5) == 'blue'
    assert light.state(16.5) == 'off'


def test_lit_targets_give_the_class_of_the_moment():
    names = ROBOBOAT
    light = lights.Light('red', flash=True)
    b = Buoy(0.0, 0.0, 0.24, 0.44, names.index('beacon_red'), light=light)
    w = Panel(1.0, 0.0, 0.5, 0.8, 0.2, 180.0, names.index('window_off'),
              light=lights.Light('blue'))
    other = Buoy(1.0, 0.0, 0.1, 0.15, names.index('buoy_black'))
    on, win, same = lights.lit_targets([b, w, other], names, 0.3)
    assert names[on.class_id] == 'beacon_red' and same is other
    assert names[win.class_id] == 'window_blue'
    off = lights.lit_targets([b], names, 1.3)[0]
    assert names[off.class_id] == 'beacon_off'
    assert b.class_id == names.index('beacon_red')         # the course's target unchanged


def test_fire_reaches_every_light():
    a, b = lights.Light('off'), lights.Light('red')
    lights.fire([Buoy(0, 0, 0.1, 0.1, 0, light=a), Buoy(0, 0, 0.1, 0.1, 0),
                 Buoy(0, 0, 0.1, 0.1, 0, light=b)], 'fire_out')
    assert a.events == {'fire_out'} and b.events == {'fire_out'}
