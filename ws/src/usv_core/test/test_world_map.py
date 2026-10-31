"""
usv_core.world_map (the persistent map) and detection.measure_boxes (what feeds it).

Run: cd /ws/src/usv_core && python3 -m pytest test/test_world_map.py -v
"""

import math

import pytest
from usv_core.classes import ROBOBOAT
from usv_core.detection import intrinsics_from_hfov, measure_boxes
from usv_core.world_map import enu, latlon, MapConfig, WorldMap

O_LAT, O_LON = 27.3757, -82.4530          # somewhere in Nathan Benderson Park


def sighting(cls, east, north, dist=5.0, ok=True):
    lat, lon = latlon(east, north, O_LAT, O_LON)
    return {'class_name': cls, 'position_ok': ok, 'distance_m': dist, 'lat': lat, 'lon': lon}


def fresh_map(**cfg):
    m = WorldMap(MapConfig(**cfg))
    m.set_origin(O_LAT, O_LON)
    return m


def test_enu_and_latlon_are_inverse():
    lat, lon = latlon(12.5, -40.0, O_LAT, O_LON)
    e, n = enu(lat, lon, O_LAT, O_LON)
    assert (e, n) == (pytest.approx(12.5, abs=1e-6), pytest.approx(-40.0, abs=1e-6))


def test_repeated_sightings_make_one_landmark():
    m = fresh_map()
    for k in range(5):
        m.add(sighting('buoy_red', 10.0 + 0.2 * (k % 2), 3.0), now=k)
    (lm,) = m.snapshot()
    assert lm['kind'] == 'buoy_red' and lm['sightings'] == 5 and lm['confirmed']
    assert lm['east'] == pytest.approx(10.08, abs=1e-6)
    assert lm['first_seen'] == 0 and lm['last_seen'] == 4


def test_close_sightings_outweigh_far_ones():
    m = fresh_map(max_range_m=15.0)
    m.add(sighting('buoy_black', 0.0, 0.0, dist=2.0), now=0)     # sigma 0.5
    m.add(sighting('buoy_black', 1.0, 0.0, dist=12.0), now=1)    # sigma 1.5, 9x less weight
    (lm,) = m.snapshot()
    assert lm['east'] == pytest.approx(0.1)
    assert lm['closest'] == 2.0


def test_far_or_unmeasured_sightings_are_not_mapped():
    m = fresh_map()
    assert m.add(sighting('buoy_red', 0.0, 0.0, dist=14.0), now=0) is None
    assert m.add(sighting('buoy_red', 0.0, 0.0, dist=float('nan')), now=0) is None
    assert m.add(sighting('buoy_red', 0.0, 0.0, ok=False), now=0) is None
    assert m.snapshot() == []


def test_neighbours_of_the_same_kind_stay_apart():
    m = fresh_map()
    for k in range(3):
        m.add(sighting('indicator_red', 0.0, 0.0), now=k)
        m.add(sighting('indicator_green', 2.03, 0.0), now=k)    # next dock slip
    assert len(m.snapshot()) == 2


def test_different_kinds_never_merge():
    m = fresh_map()
    m.add(sighting('buoy_red', 0.0, 0.0), now=0)
    m.add(sighting('buoy_green', 0.3, 0.0), now=0)
    assert sorted(lm['kind'] for lm in m.snapshot()) == ['buoy_green', 'buoy_red']


def test_indicator_keeps_its_place_when_its_colour_changes():
    m = fresh_map(color_window=5)
    for k in range(5):
        m.add(sighting('indicator_red', 4.0, 4.0), now=k)
    assert m.snapshot()[0]['color'] == 'red'
    for k in range(5):                               # changed before the next run
        m.add(sighting('indicator_green', 4.0, 4.0), now=10 + k)
    (lm,) = m.snapshot()
    assert lm['kind'] == 'indicator' and lm['color'] == 'green' and lm['sightings'] == 10


def test_confirmation_needs_enough_sightings():
    m = fresh_map(confirm_sightings=3)
    m.add(sighting('triangle', 0.0, 0.0), now=0)
    m.add(sighting('triangle', 0.0, 0.0), now=1)
    assert not m.snapshot()[0]['confirmed']
    m.add(sighting('triangle', 0.0, 0.0), now=2)
    assert m.snapshot()[0]['confirmed']


def test_origin_is_the_first_fix_only():
    m = WorldMap()
    m.set_origin(0.0, 0.0)                           # no fix yet: ignored
    assert m.origin is None
    m.set_origin(O_LAT, O_LON)
    m.set_origin(O_LAT + 1.0, O_LON)
    assert m.origin == (O_LAT, O_LON)


# ---------------------------------------------------------------- measure_boxes
FX, CX = intrinsics_from_hfov(1280, 110.0)


def test_measure_boxes_gives_bearing_distance_and_position():
    depth = (lambda u, v: 5.0)
    (d,) = measure_boxes([(630, 300, 650, 340, ROBOBOAT.index('buoy_red'))], depth, 1280, 720,
                         CX, FX, 90.0, O_LAT, O_LON, ROBOBOAT)
    assert d['class_name'] == 'buoy_red' and d['position_ok']
    assert d['distance_m'] == 5.0 and d['bearing_deg'] == pytest.approx(90.0)
    e, n = enu(d['lat'], d['lon'], O_LAT, O_LON)
    assert e == pytest.approx(5.0, abs=0.05) and abs(n) < 0.05      # 5 m due east


def test_measure_boxes_keeps_what_it_cannot_place():
    no_depth = (lambda u, v: float('nan'))
    (d,) = measure_boxes([(600, 300, 640, 340, 0)], no_depth, 1280, 720, CX, FX, 0.0,
                         O_LAT, O_LON, ROBOBOAT)
    assert not d['position_ok'] and math.isnan(d['distance_m']) and math.isnan(d['lat'])
    (d,) = measure_boxes([(600, 300, 640, 340, 0)], lambda u, v: 5.0, 1280, 720, CX, FX,
                         None, O_LAT, O_LON, ROBOBOAT)
    assert not d['position_ok'] and math.isnan(d['bearing_deg'])
