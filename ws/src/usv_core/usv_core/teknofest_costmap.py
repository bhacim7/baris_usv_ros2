"""
TEKNOFEST local frame, costmap drawing and Task 2 corridor helpers.

Ported from legacy core/nav_process.py (commit 5df3f5b); moved here from usv_core.costmap
unchanged on 2026-11-01 (phase 6 R2.3), when the plain grid stayed there for both profiles:

  LocalFrame            section C: x = North, y = East, metres from the first valid fix,
                        through haversine + bearing (legacy's own maths, kept)
  latlon_to_local       _latlon_to_local
  task2_axis_local      _task2_axis_local: the Task 2 entry -> end line in the local frame
  confirmed_boundary_local
                        _confirmed_boundary_local: orange buoys trustworthy enough to bound
                        the corridor
  TeknofestCostmap      section D: the grid (usv_core.costmap.LocalCostmap) rebuilt every
                        cycle from the legacy object memory

Grid values: 127 unknown/free, 0 buoy (planner.get_inflated_nav_map inflates them).
"""

import math

import cv2
import numpy as np

from usv_core import navigation as nav
from usv_core.costmap import LocalCostmap


class LocalFrame:
    """Section C: the boat's local position, from the first valid GPS fix."""

    def __init__(self):
        self.origin = None          # (lat, lon)
        self.x = 0.0                # North, m
        self.y = 0.0                # East, m

    def update(self, lat, lon):
        if self.origin is None and lat != 0.0 and lon != 0.0:
            self.origin = (lat, lon)
        if self.origin is not None and lat != 0.0 and lon != 0.0:
            d = nav.haversine(self.origin[0], self.origin[1], lat, lon)
            b = math.radians(nav.calculate_bearing(self.origin[0], self.origin[1], lat, lon))
            self.x = d * math.cos(b)
            self.y = d * math.sin(b)


def latlon_to_local(origin_lat, origin_lon, lat, lon):
    """(North, East) metres from the local frame origin."""
    d = nav.haversine(origin_lat, origin_lon, lat, lon)
    b = math.radians(nav.calculate_bearing(origin_lat, origin_lon, lat, lon))
    return d * math.cos(b), d * math.sin(b)


def task2_axis_local(origin_lat, origin_lon, waypoints):
    """Task 2 entry -> end in local coordinates, or None if unset or shorter than 5 m."""
    e_lat, e_lon = waypoints.get('T2_ZONE_ENTRY', (0.0, 0.0))
    x_lat, x_lon = waypoints.get('T2_ZONE_END', (0.0, 0.0))
    if not (e_lat and e_lon and x_lat and x_lon):
        return None
    if nav.haversine(e_lat, e_lon, x_lat, x_lon) < 5.0:
        return None
    p0 = np.array(latlon_to_local(origin_lat, origin_lon, e_lat, e_lon), dtype=float)
    p1 = np.array(latlon_to_local(origin_lat, origin_lon, x_lat, x_lon), dtype=float)
    return p0, p1


def confirmed_boundary_local(vision_objects, boat_lat, boat_lon, robot_x, robot_y,
                             min_seen, max_age, now):
    """Local (x, y) of the orange buoys seen min_seen times, the last time within max_age."""
    if not vision_objects or not boat_lat:
        return []
    min_seen = int(min_seen)
    max_age = float(max_age)
    out = []
    for obj in vision_objects:
        if obj.get('cid') != 3:          # orange = boundary
            continue
        if not obj.get('pos_ok', True):
            continue
        if int(obj.get('seen', 0)) < min_seen:
            continue
        last_seen = obj.get('last_seen')
        if last_seen is None or (now - last_seen) > max_age:
            continue
        o_lat, o_lon = obj.get('lat'), obj.get('lon')
        if not o_lat or not o_lon:
            continue
        dist = nav.haversine(boat_lat, boat_lon, o_lat, o_lon)
        if not (0.0 < dist < 20.0):
            continue
        brg = math.radians(nav.calculate_bearing(boat_lat, boat_lon, o_lat, o_lon))
        out.append((robot_x + dist * math.cos(brg), robot_y + dist * math.sin(brg)))
    return out


class TeknofestCostmap(LocalCostmap):
    """Section D: the grid, redrawn every cycle from the legacy object memory."""

    def rebuild(self, robot_x, robot_y, mappable_objects, task, boat_lat, boat_lon):
        """Redraw the buoys around the boat. In Task 2 only yellow and orange count."""
        self.center_m = (robot_x, robot_y)
        self.img.fill(127)
        if not mappable_objects:
            return
        for obj in mappable_objects:
            if 'TASK2' in task and obj.get('cid') not in [1, 3]:
                continue
            obj_lat = obj.get('lat')
            obj_lon = obj.get('lon')
            if obj_lat and obj_lon and boat_lat != 0:
                real_dist = nav.haversine(boat_lat, boat_lon, obj_lat, obj_lon)
                real_bearing = nav.calculate_bearing(boat_lat, boat_lon, obj_lat, obj_lon)
                if 0 < real_dist < 15.0:
                    angle_in_world = math.radians(real_bearing)
                    obj_world_x = robot_x + (real_dist * math.cos(angle_in_world))
                    obj_world_y = robot_y + (real_dist * math.sin(angle_in_world))
                    p = self.world_to_pixel(obj_world_x, obj_world_y)
                    if p:
                        cv2.circle(self.img, p, self.buoy_radius_px, 0, -1)
