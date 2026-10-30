"""
NavConfig defaults must be the settings the legacy loop actually ran with.

NavConfig.from_module() reads the frozen legacy config.py the way nav_process did (same names,
same getattr fallbacks); NavConfig() must come out identical, field by field.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_nav_config.py -v
"""

import os
import sys

LEGACY = os.path.join(os.path.dirname(__file__), 'legacy')
if LEGACY not in sys.path:
    sys.path.insert(0, LEGACY)

import config as legacy_cfg  # noqa: E402
import pytest  # noqa: E402
from usv_core.nav_config import NavConfig, WAYPOINT_NAMES  # noqa: E402

FIELDS = NavConfig().field_names()


@pytest.mark.parametrize('name', FIELDS)
def test_default_equals_legacy(name):
    assert getattr(NavConfig(), name) == getattr(NavConfig.from_module(legacy_cfg), name)


def test_every_waypoint_is_set():
    wp = NavConfig().waypoints
    assert list(wp) == list(WAYPOINT_NAMES)
    assert all(lat and lon for lat, lon in wp.values())


def test_missing_names_fall_back_like_nav_process():
    class Empty:
        pass
    nc = NavConfig.from_module(Empty())
    assert nc.cruise_pwm == 80          # nav_process: getattr(cfg, 'CRUISE_PWM', 80)
    assert nc.spot_turn_threshold_deg == 45.0
    assert nc.waypoints['T2_ZONE_MID'] == (0, 0)
