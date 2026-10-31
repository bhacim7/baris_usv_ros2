"""
The world map: every object perception has seen since power-on, one landmark per object.

Unlike the legacy object memory (usv_core.object_memory), which forgets a buoy a few seconds
after it leaves the view (a list of obstacles around the boat), the world map forgets
nothing: RoboBoat tasks are found from it (no waypoints are set from the GCS), and Task 2
reports the position of every debris buoy from it. Mapping starts at power-on, also while
the boat is driven by hand.

How a sighting (one measured box, usv_core.detection.measure_boxes) is used:
  * only sightings with a position (valid depth, heading, fix) within max_range_m count
  * it joins the nearest landmark of the same kind within merge_radius_m, or starts one;
    the two indicator classes are one kind, 'indicator', with a colour voted over the
    last color_window sightings (the colour may change between runs, the object does not)
  * positions are fused as a weighted mean, weight 1 / sigma^2 with
    sigma = sigma_base_m + sigma_per_m * range: close sightings dominate (the ZED's depth
    error grows with distance and a heading error moves far objects more)
  * a landmark is confirmed after confirm_sightings sightings

Positions are kept in a local east/north frame (metres) from the map origin, the first GPS
fix (set_origin), and given back in latitude/longitude too. All numbers in MapConfig are
first values, to be tuned from the map-quality report (run_report, phase 6 R1.6).

Known limit: a GPS without RTK drifts about a metre a minute, so an old landmark and the
boat's present position carry different GPS errors. The map says where to look; the camera
says exactly where it is.
"""

from collections import Counter, deque
from dataclasses import dataclass, field
import math

from usv_core.classes import INDICATOR_COLOR

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def radii(lat_deg):
    """(north, east) metres per radian at a latitude, WGS84."""
    s = math.sin(math.radians(lat_deg))
    w = 1.0 - WGS84_E2 * s * s
    return (WGS84_A * (1.0 - WGS84_E2) / (w * math.sqrt(w)),
            WGS84_A / math.sqrt(w) * math.cos(math.radians(lat_deg)))


def enu(lat, lon, origin_lat, origin_lon):
    """Return metres (east, north) of a point from the origin."""
    north_r, east_r = radii(origin_lat)
    return (math.radians(lon - origin_lon) * east_r, math.radians(lat - origin_lat) * north_r)


def latlon(east, north, origin_lat, origin_lon):
    north_r, east_r = radii(origin_lat)
    return (origin_lat + math.degrees(north / north_r),
            origin_lon + math.degrees(east / east_r))


def kind_of(class_name):
    """Return the map kind of a class: the indicator colours are one kind."""
    return 'indicator' if class_name in INDICATOR_COLOR else class_name


@dataclass(frozen=True)
class MapConfig:
    merge_radius_m: float = 1.5       # dock indicators stand 2 m apart
    max_range_m: float = 12.0         # legacy MAP_MAX_RANGE_M: farther positions not trusted
    sigma_base_m: float = 0.3         # ASSUMED sighting uncertainty: base + per metre range
    sigma_per_m: float = 0.1
    confirm_sightings: int = 3
    color_window: int = 15


@dataclass
class Landmark:
    landmark_id: int
    kind: str
    first_seen: float
    sum_w: float = 0.0
    sum_we: float = 0.0
    sum_wn: float = 0.0
    sightings: int = 0
    last_seen: float = 0.0
    closest_m: float = math.inf
    colors: deque = field(default_factory=deque)

    @property
    def east(self):
        return self.sum_we / self.sum_w

    @property
    def north(self):
        return self.sum_wn / self.sum_w

    @property
    def sigma(self):
        return 1.0 / math.sqrt(self.sum_w)

    @property
    def color(self):
        if not self.colors:
            return ''
        return Counter(self.colors).most_common(1)[0][0]


class WorldMap:

    def __init__(self, cfg=MapConfig()):
        self.cfg = cfg
        self.origin = None            # (lat, lon)
        self.landmarks = []
        self._next_id = 1

    def set_origin(self, lat, lon):
        """Fix the local frame at the first GPS fix; later calls change nothing."""
        if self.origin is None and lat and lon:
            self.origin = (float(lat), float(lon))

    def add(self, det, now):
        """Fuse one sighting (a measure_boxes dict); return its Landmark or None."""
        cfg = self.cfg
        dist = det['distance_m']
        if not det['position_ok'] or not dist <= cfg.max_range_m:
            return None
        self.set_origin(det['lat'], det['lon'])
        e, n = enu(det['lat'], det['lon'], *self.origin)
        kind = kind_of(det['class_name'])
        best, best_d = None, cfg.merge_radius_m
        for lm in self.landmarks:
            if lm.kind == kind:
                d = math.hypot(lm.east - e, lm.north - n)
                if d <= best_d:
                    best, best_d = lm, d
        if best is None:
            best = Landmark(self._next_id, kind, first_seen=now,
                            colors=deque(maxlen=cfg.color_window))
            self._next_id += 1
            self.landmarks.append(best)
        sigma = cfg.sigma_base_m + cfg.sigma_per_m * dist
        w = 1.0 / (sigma * sigma)
        best.sum_w += w
        best.sum_we += w * e
        best.sum_wn += w * n
        best.sightings += 1
        best.last_seen = now
        best.closest_m = min(best.closest_m, dist)
        if kind == 'indicator':
            best.colors.append(INDICATOR_COLOR[det['class_name']])
        return best

    def add_frame(self, detections, now):
        return [self.add(d, now) for d in detections]

    def snapshot(self):
        """Return every landmark as a plain dict (for the ROS message and the map file)."""
        out = []
        for lm in self.landmarks:
            lat, lon = latlon(lm.east, lm.north, *self.origin)
            out.append({
                'id': lm.landmark_id, 'kind': lm.kind, 'color': lm.color, 'lat': lat, 'lon': lon,
                'east': lm.east, 'north': lm.north, 'sigma': lm.sigma,
                'sightings': lm.sightings,
                'confirmed': lm.sightings >= self.cfg.confirm_sightings,
                'closest': lm.closest_m, 'first_seen': lm.first_seen,
                'last_seen': lm.last_seen})
        return out
