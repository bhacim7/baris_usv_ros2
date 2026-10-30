"""
navigator node conversions (usv_nodes.navigator_io), testable without a ROS graph.

Run: cd /ws/src/usv_nodes && python3 -m pytest test -v
"""

import dataclasses
import os
from types import SimpleNamespace

import pytest
from usv_core.nav_config import NavConfig
from usv_nodes.navigator_io import (buoy_to_object, nav_config_from, path_points,
                                    target_color)
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'navigator.yaml')
NODE_ONLY = {'start_manual', 'status_hz'}


def yaml_params():
    with open(CONFIG) as f:
        params = yaml.safe_load(f)['navigator']['ros__parameters']
    flat = {k: v for k, v in params.items() if k != 'waypoints'}
    flat.update({'waypoints.' + k: v for k, v in params['waypoints'].items()})
    return flat


def from_dict(values):
    return nav_config_from(lambda name, default: values.get(name, default))


def test_yaml_is_the_legacy_config():
    assert from_dict(yaml_params()) == NavConfig()


def test_yaml_lists_every_setting_and_nothing_else():
    fields = {f.name for f in dataclasses.fields(NavConfig)} - {'waypoints'}
    params = yaml_params()
    names = {k for k in params if not k.startswith('waypoints.')} - NODE_ONLY
    assert names == fields
    assert {k[len('waypoints.'):] for k in params if k.startswith('waypoints.')} == \
        set(NavConfig().waypoints)


def test_types_follow_navconfig_not_yaml():
    cfg = from_dict({'cruise_pwm': 180.0, 'loop_hz': 25, 't3_v_round_deg': [45, 70, 100]})
    assert cfg.cruise_pwm == 180 and isinstance(cfg.cruise_pwm, int)
    assert cfg.loop_hz == 25.0 and isinstance(cfg.loop_hz, float)
    assert cfg.t3_v_round_deg == (45.0, 70.0, 100.0)


def test_override_waypoint_and_task():
    cfg = from_dict({'start_task': 'TASK2_START', 'waypoints.T3_MID': [40.7, 29.8]})
    assert cfg.start_task == 'TASK2_START'
    assert cfg.waypoints['T3_MID'] == (40.7, 29.8)
    assert cfg.waypoints['T3_START'] == NavConfig().waypoints['T3_START']


def buoy(**kw):
    b = dict(id=4, class_id=0, latitude=40.72, longitude=29.82, position_ok=True,
             seen_count=5, last_seen=SimpleNamespace(sec=1000, nanosec=500_000_000),
             distance_m=3.25, cx=700, cy=400, area_px=12000)
    b.update(kw)
    return SimpleNamespace(**b)


def test_buoy_becomes_the_legacy_dict():
    assert buoy_to_object(buoy()) == {
        'id': 4, 'cid': 0, 'lat': 40.72, 'lon': 29.82, 'pos_ok': True, 'seen': 5,
        'last_seen': pytest.approx(1000.5), 'dist': 3.25, 'cx': 700, 'cy': 400,
        'area': 12000}


def test_unknown_markers_become_none():
    o = buoy_to_object(buoy(class_id=255, distance_m=float('nan'), cx=-1, cy=-1))
    assert (o['cid'], o['dist'], o['cx'], o['cy']) == (None, None, None, None)


def test_path_is_published_east_north():
    assert path_points([(10.0, 2.0), (12.0, -1.0)]) == [(2.0, 10.0), (-1.0, 12.0)]
    assert path_points(None) == []


def test_target_colour_source():
    cfg = NavConfig()                                    # drone_active, black
    assert target_color(cfg, None) == ('black', 'cfg')
    assert target_color(cfg, 'red') == ('red', 'drone')
    off = dataclasses.replace(cfg, drone_active=False)
    assert target_color(off, 'red') == ('black', 'cfg')
