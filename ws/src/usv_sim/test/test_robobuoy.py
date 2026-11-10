"""
Unit tests for the RoboBuoy sizes (usv_sim/robobuoy.py), no ROS needed.

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_robobuoy.py -v
"""

import math

import pytest
from usv_sim import robobuoy


def test_sizes_follow_the_build_guide():
    assert robobuoy.PANEL_BASE == pytest.approx(0.4318)
    assert 0.24 < robobuoy.PANEL_HEIGHT < 0.27          # 12 in panels leaning in
    assert 0.40 < robobuoy.TOP < 0.50                   # top of the sun hat


def test_face_middle_lies_on_the_sloping_face():
    out, z = robobuoy.FACE_MID
    frac = (z - robobuoy.PANEL_BOTTOM) / robobuoy.PANEL_HEIGHT
    half = robobuoy.PANEL_BASE / 2 + frac * (robobuoy.PANEL_TOP - robobuoy.PANEL_BASE) / 2
    assert out == pytest.approx(half)
    assert math.degrees(robobuoy.FACE_TILT) == pytest.approx(32.8, abs=0.5)
