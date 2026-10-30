"""
Conversions between the ROS side and usv_core.navigator.Navigator.

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
"""

import dataclasses
import math

from usv_core.nav_config import NavConfig, WAYPOINT_NAMES

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
