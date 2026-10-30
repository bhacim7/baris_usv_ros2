"""
Local frame, local costmap and Task 2 corridor helpers.

Ported from legacy core/nav_process.py (commit 5df3f5b):

  LocalFrame            section C: x = North, y = East, metres from the first valid fix,
                        through haversine + bearing (legacy's own maths, kept)
  latlon_to_local       _latlon_to_local
  task2_axis_local      _task2_axis_local: the Task 2 entry -> end line in the local frame
  confirmed_boundary_local
                        _confirmed_boundary_local: orange buoys trustworthy enough to bound
                        the corridor
  LocalCostmap          section D: a robot-centred grid rebuilt every cycle from the
                        object memory; plus the crop the A* planner runs on (section G)

Grid values: 127 unknown/free, 0 buoy (planner.get_inflated_nav_map inflates them).
"""

import math

import cv2
import numpy as np

from usv_core import navigation as nav


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


class LocalCostmap:
    """Section D: robot-centred grid of A_STAR_CROP_RADIUS_M around the boat."""

    def __init__(self, res_m_per_px, crop_radius_m, buoy_radius_m):
        self.res = res_m_per_px
        self.crop_radius_m = crop_radius_m
        size = int((crop_radius_m * 2.0) / res_m_per_px)
        self.img = np.full((size, size), 127, dtype=np.uint8)
        self.size_px = (size, size)            # (width, height)
        self.center_m = (0.0, 0.0)
        # Physical footprint only; the safety margin is the planner's inflation
        self.buoy_radius_px = max(1, int(round(buoy_radius_m / res_m_per_px)))

    def world_to_pixel(self, world_x, world_y):
        w, h = self.size_px
        dx_m = world_x - self.center_m[0]
        dy_m = world_y - self.center_m[1]
        px = int((w // 2) + (dx_m / self.res))
        py = int((h // 2) - (dy_m / self.res))
        if 0 <= px < w and 0 <= py < h:
            return (px, py)
        return None

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

    def crop(self, robot_x, robot_y):
        """Cut the A* window around the boat: (grid, centre in metres, (width, height))."""
        crop_radius_px = int(self.crop_radius_m / self.res)
        map_w, map_h = self.size_px
        cw, ch = map_w // 2, map_h // 2
        rx_px = int(cw + ((robot_x - self.center_m[0]) / self.res))
        ry_px = int(ch - ((robot_y - self.center_m[1]) / self.res))
        x_min = max(0, rx_px - crop_radius_px)
        x_max = min(map_w, rx_px + crop_radius_px)
        y_min = max(0, ry_px - crop_radius_px)
        y_max = min(map_h, ry_px + crop_radius_px)
        cropped = self.img[y_min:y_max, x_min:x_max]
        center = (self.center_m[0] + ((x_min + x_max) / 2 - cw) * self.res,
                  self.center_m[1] - ((y_min + y_max) / 2 - ch) * self.res)
        return cropped, center, (x_max - x_min, y_max - y_min)
