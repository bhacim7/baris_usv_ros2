"""Port check for usv_core.heading against the frozen legacy copies."""

import os
import random
import sys

import pytest

from usv_core.heading import HeadingFilter, LEGACY_DECLINATION_DEG, select_heading, zed_heading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))
from utils.headingFilter import KalmanFilter as LegacyFilter  # noqa: E402
from utils import navigasyon as legacy_nav  # noqa: E402


def raw_heading_stream(seed, n=3000):
    """Magnetometer-like headings: slow turns through north, noise, occasional jumps."""
    rng = random.Random(seed)
    h = rng.uniform(0, 360)
    out = []
    for _ in range(n):
        h = (h + rng.gauss(0.0, 1.5)) % 360          # turning, crossing 0/360 now and then
        sample = (h + rng.gauss(0.0, 3.0)) % 360     # magnetometer noise
        if rng.random() < 0.01:
            sample = rng.uniform(0, 360)             # disturbance spike
        out.append(sample)
    return out


@pytest.mark.parametrize('seed', range(5))
def test_filter_matches_legacy_bit_for_bit(seed):
    ours, theirs = HeadingFilter(1e-3, 1e-1), LegacyFilter(process_variance=1e-3,
                                                         measurement_variance=1e-1)
    for raw in raw_heading_stream(seed):
        assert ours.update(raw) == theirs.update(raw)


def test_filter_crosses_north_the_short_way():
    f = HeadingFilter()
    f.update(355.0)
    for _ in range(200):
        out = f.update(5.0)
        assert out >= 355.0 - 1e-9 or out <= 5.0 + 1e-9    # never swings through 180
    assert out == pytest.approx(5.0, abs=0.5)


def test_zed_heading_is_filter_then_declination():
    # camera_process: zed_heading = magnetic_filter.update(raw); (zed_heading + 6) % 360
    legacy = LegacyFilter(process_variance=1e-3, measurement_variance=1e-1)
    ours = HeadingFilter()
    for raw in raw_heading_stream(11, 500):
        expected = (legacy.update(raw) + 6) % 360
        assert zed_heading(ours, raw) == expected
    assert LEGACY_DECLINATION_DEG == 6.0


def test_declination_is_a_parameter():
    assert zed_heading(HeadingFilter(), 10.0, declination_deg=-6.0) == pytest.approx(4.0)
    assert zed_heading(HeadingFilter(), 3.0, declination_deg=-6.0) == pytest.approx(357.0)


def legacy_select(heading_source, zed, fc_hdg):
    """nav_process lines 465-471, verbatim apart from the variable names."""
    if heading_source == 'FC' and fc_hdg is not None:
        magnetic_heading = fc_hdg
    elif heading_source == 'FUSED' and fc_hdg is not None:
        diff = legacy_nav.signed_angle_difference(zed, fc_hdg)
        magnetic_heading = (zed + (diff * 0.5)) % 360
    else:
        magnetic_heading = zed
    return magnetic_heading


@pytest.mark.parametrize('source', ['ZED', 'FC', 'FUSED', 'something else'])
def test_select_heading_matches_legacy(source):
    rng = random.Random(3)
    for _ in range(2000):
        zed = rng.uniform(0, 360)
        fc = None if rng.random() < 0.1 else rng.uniform(0, 360)
        assert select_heading(source, zed, fc) == legacy_select(source, zed, fc)


def test_fused_averages_across_north():
    assert select_heading('FUSED', 350.0, 10.0) == pytest.approx(0.0)
