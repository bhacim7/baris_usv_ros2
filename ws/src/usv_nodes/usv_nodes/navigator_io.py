"""
Conversions between the ROS side and usv_core.teknofest_navigator.TeknofestNavigator.

Kept apart from the node so they can be tested without a running ROS graph:

  nav_config_from(lookup)   NavConfig from parameter values; lookup(name, default) returns
                            the parameter (the node declares it with that default). Values
                            are cast to the type of the NavConfig default, so a YAML 180
                            for a float field or 1.0 for an int field cannot break startup
  parameter_default(v)      a NavConfig default as a ROS parameter value (tuples -> lists)
  buoy_to_object(b)         one usv_interfaces/Buoy as the dict legacy's object memory
                            handed to the navigation (keys: id, cid, lat, lon, pos_ok, seen,
                            last_seen, dist, cx, cy, area); "unknown" markers back to None
  path_points(path)         the A* path (local x = North, y = East) as ENU (east, north)
  target_color(cfg, drone)  the colour Task 3 hunts and where it came from ('cfg'/'drone')

RoboBoat profile (phase 6 R2):
  run_config_from(lookup)   usv_core.roboboat_run.RunConfig from the roboboat.* parameters
  parse_hint(text)          'ahead,left' (metres from the Task 1 entry gates) -> tuple;
                            '' = no hint
  landmark_to_dict(lm)      one usv_interfaces/Landmark as the dict usv_core.world_map
                            snapshots give (id, kind, color, lat, lon, east, north, sigma,
                            sightings, confirmed, closest, first_seen, last_seen)
  detections_to_dicts(msg)  a DetectionArray as measure_boxes dicts, plus its frame time
"""

import dataclasses
import math

from usv_core.nav_config import NavConfig, WAYPOINT_NAMES
from usv_core.roboboat_run import HINTED, RunConfig

UNKNOWN_CLASS = 255             # fake_perception / perception: class id not decided


def parameter_default(value):
    if isinstance(value, tuple):
        return [float(v) for v in value]
    return value


def _cast(value, default):
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, int):
        return int(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, tuple):
        return tuple(float(v) for v in value)
    return str(value)


def nav_config_from(lookup):
    defaults = NavConfig()
    kwargs = {}
    for f in dataclasses.fields(NavConfig):
        if f.name == 'waypoints':
            continue
        default = getattr(defaults, f.name)
        kwargs[f.name] = _cast(lookup(f.name, parameter_default(default)), default)
    waypoints = {}
    for name in WAYPOINT_NAMES:
        lat, lon = lookup('waypoints.' + name, list(defaults.waypoints[name]))
        waypoints[name] = (float(lat), float(lon))
    kwargs['waypoints'] = waypoints
    return NavConfig(**kwargs)


def buoy_to_object(b):
    return {
        'id': int(b.id),
        'cid': None if b.class_id == UNKNOWN_CLASS else int(b.class_id),
        'lat': float(b.latitude),
        'lon': float(b.longitude),
        'pos_ok': bool(b.position_ok),
        'seen': int(b.seen_count),
        'last_seen': b.last_seen.sec + b.last_seen.nanosec * 1e-9,
        'dist': None if math.isnan(b.distance_m) else float(b.distance_m),
        'cx': None if b.cx < 0 else int(b.cx),
        'cy': None if b.cy < 0 else int(b.cy),
        'area': int(b.area_px),
    }


def path_points(path):
    """Local (x = North, y = East) metres -> (east, north) for an ENU nav_msgs/Path."""
    return [(float(y), float(x)) for x, y in (path or [])]


def target_color(cfg, drone_target_color):
    if cfg.drone_active and drone_target_color:
        return str(drone_target_color), 'drone'
    return str(cfg.task3_kamikaze_color), 'cfg'


# ---------------------------------------------------------------- RoboBoat profile
def parse_hint(text):
    text = str(text).strip()
    if not text:
        return None
    parts = [float(v) for v in text.replace(' ', '').split(',')]
    if len(parts) != 2:
        raise ValueError(f'hint {text!r}: want "ahead_m,left_m"')
    return tuple(parts)


def run_config_from(lookup):
    defaults = RunConfig()
    hints = {}
    for name in HINTED:
        hint = parse_hint(lookup('roboboat.hint.' + name, ''))
        if hint is not None:
            hints[name] = hint
    return RunConfig(
        task_order=tuple(str(t) for t in lookup('roboboat.task_order',
                                                list(defaults.task_order))),
        task_timeout_s=float(lookup('roboboat.task_timeout_s', defaults.task_timeout_s)),
        run_timeout_s=float(lookup('roboboat.run_timeout_s', defaults.run_timeout_s)),
        hints=hints)


def _stamp(t):
    return t.sec + t.nanosec * 1e-9


def landmark_to_dict(lm):
    return {'id': int(lm.id), 'kind': lm.kind, 'color': lm.color,
            'lat': float(lm.latitude), 'lon': float(lm.longitude),
            'east': float(lm.east_m), 'north': float(lm.north_m), 'sigma': float(lm.sigma_m),
            'sightings': int(lm.sightings), 'confirmed': bool(lm.confirmed),
            'closest': float(lm.closest_m), 'first_seen': _stamp(lm.first_seen),
            'last_seen': _stamp(lm.last_seen)}


def detections_to_dicts(msg):
    """Return (detections, frame time): one dict per box, as measure_boxes gives them."""
    out = [{'class_name': d.class_name, 'class_id': int(d.class_id),
            'box': (int(d.x1), int(d.y1), int(d.x2), int(d.y2)),
            'depth_m': float(d.depth_m), 'distance_m': float(d.distance_m),
            'bearing_deg': float(d.bearing_deg), 'position_ok': bool(d.position_ok),
            'lat': float(d.latitude), 'lon': float(d.longitude)} for d in msg.detections]
    return out, _stamp(msg.header.stamp)
