"""
navigator node conversions (usv_nodes.navigator_io), testable without a ROS graph.

Run: cd /ws/src/usv_nodes && python3 -m pytest test -v
"""

import dataclasses
import os
from types import SimpleNamespace

import pytest
from usv_core.nav_config import NavConfig
from usv_core.roboboat_drive import DriveConfig
from usv_core.roboboat_run import HINTED, RunConfig
from usv_core.roboboat_tasks import DebrisConfig
from usv_nodes.navigator_io import (buoy_to_object, debris_config_from, detections_to_dicts,
                                    drive_config_from, landmark_to_dict, nav_config_from,
                                    parse_hint, path_points, run_config_from, target_color)
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'navigator.yaml')
NODE_ONLY = {'start_manual', 'status_hz', 'competition'}


def yaml_params():
    with open(CONFIG) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    flat = {k: v for k, v in params.items() if k not in ('waypoints', 'roboboat')}
    flat.update({'waypoints.' + k: v for k, v in params['waypoints'].items()})
    return flat


def roboboat_params():
    with open(CONFIG) as f:
        rb = yaml.safe_load(f)['/**']['ros__parameters']['roboboat']
    flat = {'roboboat.' + k: v for k, v in rb.items() if k not in ('hint', 'drive', 'debris')}
    flat.update({'roboboat.hint.' + k: v for k, v in rb['hint'].items()})
    flat.update({'roboboat.drive.' + k: v for k, v in rb['drive'].items()})
    flat.update({'roboboat.debris.' + k: v for k, v in rb['debris'].items()})
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


# ---------------------------------------------------------------- RoboBoat profile
def test_roboboat_yaml_is_the_run_defaults_with_no_hints():
    params = roboboat_params()
    cfg = run_config_from(lambda name, default: params.get(name, default))
    assert cfg == RunConfig()
    assert {k[len('roboboat.hint.'):] for k in params if k.startswith('roboboat.hint.')} \
        == set(HINTED)


def test_hints_and_task_order_from_parameters():
    params = {'roboboat.task_order': ['task3', 'task2'], 'roboboat.hint.task2': '21, -1.5',
              'roboboat.hint.task5': ''}
    cfg = run_config_from(lambda name, default: params.get(name, default))
    assert cfg.task_order == ('task3', 'task2')
    assert cfg.hints == {'task2': (21.0, -1.5)}
    assert parse_hint('') is None
    with pytest.raises(ValueError):
        parse_hint('3,4,5')


def stamp(t):
    return SimpleNamespace(sec=int(t), nanosec=int((t % 1) * 1e9))


def test_landmark_and_detections_become_dicts():
    lm = SimpleNamespace(id=7, kind='pole_red', color='', latitude=27.1, longitude=-82.4,
                         east_m=3.0, north_m=4.0, sigma_m=0.2, sightings=12, confirmed=True,
                         closest_m=4.5, first_seen=stamp(10.5), last_seen=stamp(20.25))
    d = landmark_to_dict(lm)
    assert d['kind'] == 'pole_red' and d['confirmed'] and d['last_seen'] == 20.25
    det = SimpleNamespace(class_name='buoy_red', class_id=2, x1=1, y1=2, x2=3, y2=4,
                          depth_m=5.0, distance_m=5.1, bearing_deg=90.0, position_ok=True,
                          latitude=27.1, longitude=-82.4)
    cut = SimpleNamespace(**{**vars(det), 'x1': 1200, 'x2': 1279})
    frame = SimpleNamespace(header=SimpleNamespace(stamp=stamp(33.5)), image_width=1280,
                            detections=[det, cut])
    dets, t = detections_to_dicts(frame)
    assert t == 33.5 and dets[0]['box'] == (1, 2, 3, 4) and dets[0]['distance_m'] == 5.1
    assert dets[0]['edge'] and dets[1]['edge']             # x1 = 1 and x2 = 1279 touch
    frame.detections[0] = SimpleNamespace(**{**vars(det), 'x1': 5, 'x2': 30})
    assert not detections_to_dicts(frame)[0][0]['edge']


def test_drive_settings_from_yaml_and_override():
    params = roboboat_params()
    assert drive_config_from(lambda name, default: params.get(name, default)) == DriveConfig()
    assert drive_config_from(lambda name, default: {'roboboat.drive.lookahead_m': 4}.get(
        name, default)).lookahead_m == 4.0


def test_debris_settings_from_yaml_and_override():
    params = roboboat_params()
    assert debris_config_from(lambda name, default: params.get(name, default)) == DebrisConfig()
    cfg = debris_config_from(lambda name, default: {'roboboat.debris.half_width_m': 10,
                                                    'roboboat.debris.green_clockwise': True}.get(
        name, default))
    assert cfg.half_width_m == 10.0 and cfg.green_clockwise is True and cfg.depth_m == 15.0
