"""Port check for usv_core.navigation against the frozen legacy copy."""

import math
import os
import sys

import pytest

from usv_core import navigation as nav

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))
import navigasyon as legacy  # noqa: E402

# Course waypoints from legacy config.py, plus far-away and Southern Hemisphere points
POINTS = [
    (40.8090735, 29.2622975),   # T1_GATE_ENTER
    (40.8089036, 29.2622449),   # T1_GATE_MID
    (40.8088117, 29.2624693),   # T1_GATE_EXIT
    (40.8095851, 29.2622612),   # T2_ZONE_MID
    (40.809666, 29.2622792),    # T3_START
    (41.0, 29.0),
    (-33.72, 150.67),           # Sydney (VRX)
]
PAIRS = [(a, b) for a in POINTS for b in POINTS]
TRIPLES = [(POINTS[0], POINTS[1], POINTS[3]), (POINTS[2], POINTS[4], POINTS[0]),
           (POINTS[1], POINTS[3], POINTS[4])]


# --- Port equivalence: results must be bit-for-bit identical ---------------------

@pytest.mark.parametrize('a,b', PAIRS)
def test_haversine_matches_legacy(a, b):
    assert nav.haversine(*a, *b) == legacy.haversine(*a, *b)


@pytest.mark.parametrize('a,b', PAIRS)
def test_bearing_matches_legacy(a, b):
    assert nav.calculate_bearing(*a, *b) == legacy.calculate_bearing(*a, *b)


@pytest.mark.parametrize('start,current,target', TRIPLES)
def test_cross_track_matches_legacy(start, current, target):
    assert (nav.calculate_cross_track_error(*start, *current, *target)
            == legacy.calculate_cross_track_error(*start, *current, *target))


@pytest.mark.parametrize('dist', [0.0, 5.0, 12.5, 30.0])
@pytest.mark.parametrize('bearing', [0.0, 45.0, 90.0, 180.0, 270.0, 359.0])
def test_obj_gps_matches_legacy(dist, bearing):
    lat, lon = POINTS[0]
    assert nav.calculate_obj_gps(lat, lon, dist, bearing) == legacy.calculate_obj_gps(lat, lon, dist, bearing)


@pytest.mark.parametrize('a1', [0.0, 10.0, 179.0, 181.0, 350.0, -30.0])
@pytest.mark.parametrize('a2', [0.0, 10.0, 180.0, 359.0, 720.5])
def test_angle_difference_matches_legacy(a1, a2):
    assert nav.signed_angle_difference(a1, a2) == legacy.signed_angle_difference(a1, a2)


@pytest.mark.parametrize('err', [-90.0, -10.0, 0.0, 30.0, 170.0])
def test_hybrid_point_matches_legacy(err):
    assert nav.get_hybrid_point(1.0, -2.0, 0.7, err, 2.0) == legacy.get_hybrid_point(1.0, -2.0, 0.7, err, 2.0)


# --- Known values: what the functions mean -------------------------------------

def test_one_degree_of_latitude():
    expected = 6371000 * math.pi / 180   # 111194.93 m
    assert nav.haversine(0.0, 0.0, 1.0, 0.0) == pytest.approx(expected, rel=1e-12)


def test_bearing_north_and_east():
    assert nav.calculate_bearing(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0)
    assert nav.calculate_bearing(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0)


def test_angle_difference_sign_and_wrap():
    assert nav.signed_angle_difference(350.0, 10.0) == 20.0     # right turn through north
    assert nav.signed_angle_difference(10.0, 350.0) == -20.0    # left turn
    assert nav.signed_angle_difference(0.0, 180.0) == -180.0    # exactly behind: left by convention


def test_cross_track_sign():
    # Line due north; a point slightly east is to the right (positive)
    assert nav.calculate_cross_track_error(40.0, 29.0, 40.0005, 29.0001, 40.001, 29.0) > 0


def test_radius_mismatch_is_documented():
    lat, lon = nav.calculate_obj_gps(40.0, 29.0, 10.0, 0.0)
    measured = nav.haversine(40.0, 29.0, lat, lon)
    assert measured == pytest.approx(10.0 * 6371000.0 / 6378137.0, abs=1e-6)   # 9.989 m