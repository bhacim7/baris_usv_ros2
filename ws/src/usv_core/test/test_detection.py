"""
Port check for usv_core.detection against the legacy camera_process formulas.

The legacy code is inline in the camera loop, so it cannot be imported; the reference
functions below repeat it line for line (camera_process.py, commit 5df3f5b).
"""

import math
import os
import random
import sys

import numpy as np
import pytest

from usv_core import detection as det

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))
from utils import navigasyon as legacy_nav  # noqa: E402

WIDTH, HEIGHT = 1280, 720    # ZED 2i at HD720, as configured in camera_process


def legacy_depth(depth_at, x1, y1, x2, y2, width, height):
    """camera_process lines 559-580."""
    cx = int((x1 + x2) / 2)
    box_h = y2 - y1
    target_cy = int(y2 - (box_h * 0.15))
    cy = max(0, min(target_cy, height - 1))
    roi_size = 2
    depth_values = []
    for dy in range(-roi_size, roi_size + 1):
        for dx in range(-roi_size, roi_size + 1):
            sample_x = max(0, min(cx + dx, width - 1))
            sample_y = max(0, min(cy + dy, height - 1))
            d_val = depth_at(sample_x, sample_y)
            if not np.isnan(d_val) and not np.isinf(d_val) and d_val > 0.1:
                depth_values.append(d_val)
    if depth_values:
        dist_m = float(np.median(depth_values))
    else:
        dist_m = float('inf')
    return cx, cy, dist_m


def noisy_depth_image(seed):
    """Depth lookup with water-reflection holes (NaN), inf and near-zero junk."""
    rng = random.Random(seed)
    cache = {}

    def depth_at(x, y):
        if (x, y) not in cache:
            r = rng.random()
            if r < 0.15:
                cache[(x, y)] = float('nan')
            elif r < 0.2:
                cache[(x, y)] = float('inf')
            elif r < 0.25:
                cache[(x, y)] = rng.uniform(-1.0, 0.1)
            else:
                cache[(x, y)] = rng.uniform(0.3, 20.0)
        return cache[(x, y)]
    return depth_at


def random_box(rng):
    x1 = rng.randint(-20, WIDTH - 2)
    y1 = rng.randint(-20, HEIGHT - 2)
    return x1, y1, x1 + rng.randint(2, 300), y1 + rng.randint(2, 300)


@pytest.mark.parametrize('seed', range(5))
def test_depth_matches_legacy(seed):
    rng = random.Random(seed)
    depth_at = noisy_depth_image(seed)
    for _ in range(300):
        x1, y1, x2, y2 = random_box(rng)
        cx, cy = det.depth_sample_pixel(x1, y1, x2, y2, HEIGHT)
        dist = det.median_depth(depth_at, cx, cy, WIDTH, HEIGHT)
        assert (cx, cy, dist) == legacy_depth(depth_at, x1, y1, x2, y2, WIDTH, HEIGHT)


def test_no_valid_depth_is_inf_and_rejected():
    assert det.median_depth(lambda x, y: float('nan'), 640, 360, WIDTH, HEIGHT) == math.inf
    assert not det.distance_is_valid(math.inf)


@pytest.mark.parametrize('d, ok', [(0.5, False), (0.51, True), (14.99, True), (15.0, False),
                                   (math.nan, False)])
def test_distance_window(d, ok):
    # camera_process: not nan and not inf and 0.5 < dist_m < 15.0
    assert det.distance_is_valid(d) is ok


def test_hfov_fallback_matches_legacy():
    fx, cx = det.intrinsics_from_hfov(WIDTH, 110.0)
    assert fx == (WIDTH / 2.0) / math.tan(math.radians(110.0) / 2.0)
    assert cx == WIDTH / 2.0
    # the image edge is exactly half the field of view away from the centre
    assert det.bearing_deg(WIDTH, cx, fx, 0.0) == pytest.approx(55.0)


def test_bearing_matches_legacy_and_right_is_larger():
    rng = random.Random(4)
    fx, cam_cx = det.intrinsics_from_hfov(WIDTH, 110.0)
    for _ in range(1000):
        cx = rng.randint(0, WIDTH - 1)
        heading = rng.uniform(0, 360)
        legacy = (heading + math.degrees(math.atan((cx - cam_cx) / fx))) % 360
        assert det.bearing_deg(cx, cam_cx, fx, heading) == legacy
    assert det.bearing_deg(900, cam_cx, fx, 90.0) > 90.0


def test_position_uses_the_legacy_projection():
    for d, b in [(1.0, 0.0), (7.5, 123.4), (14.9, 359.9)]:
        assert det.detection_position(40.8090735, 29.2622975, d, b) == \
            legacy_nav.calculate_obj_gps(40.8090735, 29.2622975, d, b)
