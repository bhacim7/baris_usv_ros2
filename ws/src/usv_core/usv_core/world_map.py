"""
The world map: every object perception has seen since power-on, one landmark per object.

Unlike the legacy object memory (usv_core.teknofest_object_memory), which forgets a buoy a
few seconds after it leaves the view (a list of obstacles around the boat), the world map
forgets nothing: RoboBoat tasks are found from it (no waypoints are set from the GCS), and Task 2
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

Two clean-ups (phase 6 R1.7), each off when its setting is 0:
  D4 turning   the heading filter lags in turns (measured: mean 9 deg above 30 deg/s), so a
               sighting taken while turning fast is placed sideways by about
               distance * tan(heading_lag_s * yaw rate). That is added to its sigma (less
               weight), and above new_landmark_max_turn_dps it may join a landmark but not
               start one: in a turn a buoy lands metres aside and would open a duplicate.
  D5 confusion YOLO now and then gives a box the wrong class, which opens a landmark of
               another kind on top of the real one. Such a landmark is not confirmed while
               a landmark of another kind within confusion_radius_m has at least
               confusion_ratio times its sightings and the two were never seen in the same
               camera frame. A real neighbour (a black buoy beside a red one) is seen in the
               same frames as a box of its own, so it is confirmed as usual; a wrong label
               replaces the real box in its frame, so it never is. Nothing is deleted.
               Needs the sightings of a frame together (add_frame).

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
    heading_lag_s: float = 0.0            # D4: 0 = off
    new_landmark_max_turn_dps: float = 0.0    # D4: 0 = off
    confusion_ratio: float = 0.0          # D5: 0 = off
    confusion_radius_m: float = 1.0


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
    co_seen: dict = field(default_factory=dict)   # other landmark id -> frames seen together

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

    def add(self, det, now, yaw_rate_dps=0.0):
        """Fuse one sighting (a measure_boxes dict); return its Landmark or None."""
        cfg = self.cfg
        turn = abs(yaw_rate_dps)
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
            if cfg.new_landmark_max_turn_dps > 0 and turn > cfg.new_landmark_max_turn_dps:
                return None           # D4: no new landmark while turning fast
            best = Landmark(self._next_id, kind, first_seen=now,
                            colors=deque(maxlen=cfg.color_window))
            self._next_id += 1
            self.landmarks.append(best)
        sigma = cfg.sigma_base_m + cfg.sigma_per_m * dist
        if cfg.heading_lag_s > 0:     # D4: sideways error of the lagging heading
            sigma = math.hypot(sigma, dist * math.tan(math.radians(
                min(80.0, cfg.heading_lag_s * turn))))
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

    def add_frame(self, detections, now, yaw_rate_dps=0.0):
        """Fuse the sightings of one camera frame; remember which landmarks it showed together."""
        hits = [self.add(d, now, yaw_rate_dps) for d in detections]
        seen = {lm.landmark_id: lm for lm in hits if lm is not None}
        for a in seen.values():
            for b_id in seen:
                if b_id != a.landmark_id:
                    a.co_seen[b_id] = a.co_seen.get(b_id, 0) + 1
        return hits

    def suspect(self, lm):
        """D5: True if lm looks like a wrong class on top of a landmark of another kind."""
        cfg = self.cfg
        if cfg.confusion_ratio <= 0:
            return False
        for other in self.landmarks:
            if other.kind == lm.kind or lm.co_seen.get(other.landmark_id, 0) > 0:
                continue
            if other.sightings >= cfg.confusion_ratio * lm.sightings and math.hypot(
                    other.east - lm.east, other.north - lm.north) <= cfg.confusion_radius_m:
                return True
        return False

    def confirmed(self, lm):
        return lm.sightings >= self.cfg.confirm_sightings and not self.suspect(lm)

    def snapshot(self):
        """Return every landmark as a plain dict (for the ROS message and the map file)."""
        out = []
        for lm in self.landmarks:
            lat, lon = latlon(lm.east, lm.north, *self.origin)
            out.append({
                'id': lm.landmark_id, 'kind': lm.kind, 'color': lm.color, 'lat': lat, 'lon': lon,
                'east': lm.east, 'north': lm.north, 'sigma': lm.sigma,
                'sightings': lm.sightings,
                'confirmed': self.confirmed(lm),
                'closest': lm.closest_m, 'first_seen': lm.first_seen,
                'last_seen': lm.last_seen})
        return out
