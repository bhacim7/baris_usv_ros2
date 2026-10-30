"""Latitude/longitude <-> local metres around a course origin, on the WGS84 ellipsoid.

Gazebo's NavSat sensor converts with the WGS84 ellipsoid, so the world generator and
fake_perception must too: with a sphere of 6378137 m the north scale is 0.25 % off at 40.7 N,
i.e. 15 cm over a 60 m course between the buoys and /usv/fix. Local (tangent plane) radii
are exact to well under a centimetre over a few hundred metres.
"""

import math

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def radii(lat_deg):
    """(north metres per radian of latitude, east metres per radian of longitude)."""
    s = math.sin(math.radians(lat_deg))
    w = 1.0 - WGS84_E2 * s * s
    meridian = WGS84_A * (1.0 - WGS84_E2) / (w * math.sqrt(w))
    prime = WGS84_A / math.sqrt(w)
    return meridian, prime * math.cos(math.radians(lat_deg))


def enu(lat, lon, origin_lat, origin_lon):
    """Metres (east, north) of a point from the origin."""
    north_r, east_r = radii(origin_lat)
    return (math.radians(lon - origin_lon) * east_r, math.radians(lat - origin_lat) * north_r)


def latlon(east, north, origin_lat, origin_lon):
    north_r, east_r = radii(origin_lat)
    return (origin_lat + math.degrees(north / north_r), origin_lon + math.degrees(east / east_r))
