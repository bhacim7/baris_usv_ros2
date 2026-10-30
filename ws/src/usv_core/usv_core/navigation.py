"""
Geodesy helpers ported from legacy utils/navigasyon.py (commit 5df3f5b).

Function bodies are unchanged on purpose; test/test_navigation.py checks that every
function returns exactly the same value as the legacy copy in test/legacy/.

Known legacy inconsistency, kept deliberately: calculate_obj_gps uses an Earth radius of
6378137 m, haversine and calculate_cross_track_error use 6371000 m. A point projected 10 m
away measures 9.989 m with haversine. Gains were tuned with this in place.
"""

import math


def calculate_obj_gps(lat1, lon1, dist_m, bearing_deg):
    """
    Point reached from (lat1, lon1) after dist_m metres along bearing_deg.

    Returns (lat, lon) in degrees. Bearing is clockwise from north.
    """
    R = 6378137.0
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    bearing_rad = math.radians(bearing_deg)

    lat2_rad = math.asin(math.sin(lat1_rad) * math.cos(dist_m / R) +
                         math.cos(lat1_rad) * math.sin(dist_m / R) * math.cos(bearing_rad))

    lon2_rad = lon1_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(dist_m / R) * math.cos(lat1_rad),
        math.cos(dist_m / R) - math.sin(lat1_rad) * math.sin(lat2_rad))

    return math.degrees(lat2_rad), math.degrees(lon2_rad)


def get_hybrid_point(robot_x, robot_y, robot_yaw, angle_error_deg, step_dist=2.0):
    """Local point step_dist ahead of the robot, rotated by -angle_error_deg."""
    target_angle = robot_yaw - math.radians(angle_error_deg)
    tx = robot_x + (step_dist * math.cos(target_angle))
    ty = robot_y + (step_dist * math.sin(target_angle))
    return tx, ty


def haversine(lat1, lon1, lat2, lon2):
    """Great-circle distance between two points, in metres."""
    R = 6371000
    phi_1 = math.radians(lat1)
    phi_2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0) ** 2 + \
        math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def calculate_cross_track_error(start_lat, start_lon, current_lat, current_lon,
                                target_lat, target_lon):
    """
    Signed distance in metres from the current position to the start->target line.

    Positive means the boat is right of the line, negative means left.
    """
    R = 6371000

    theta_13 = math.radians(calculate_bearing(start_lat, start_lon, target_lat, target_lon))
    theta_12 = math.radians(calculate_bearing(start_lat, start_lon, current_lat, current_lon))
    dist_13 = haversine(start_lat, start_lon, current_lat, current_lon) / R

    xte_rad = math.asin(math.sin(dist_13) * math.sin(theta_12 - theta_13))

    return xte_rad * R


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Return the initial bearing from point 1 to point 2, degrees in [0, 360)."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    lam1 = math.radians(lon1)
    lam2 = math.radians(lon2)
    y = math.sin(lam2 - lam1) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(lam2 - lam1)
    theta = math.atan2(y, x)
    return (math.degrees(theta) + 360) % 360


def signed_angle_difference(angle1, angle2):
    """
    Shortest signed turn from angle1 to angle2, degrees in [-180, 180).

    Positive means angle2 is clockwise (to the right) of angle1.
    """
    diff = (angle2 - angle1 + 180) % 360 - 180
    return diff
