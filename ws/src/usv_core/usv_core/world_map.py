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
  * the four light beacon classes are one kind, 'beacon' (RoboBoat 2027): its light may
    change during the run and red/green flash 1 s on, 1 s off, so its colour is read from
    the last beacon_window_s seconds before it was last seen: the most common lit colour,
    'off' only if it was never lit in that time. One frame does not change it; a real
    change shows within beacon_window_s.
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
               confusion_ratio times its sightings and the two were seen in the same camera
               frame fewer than CO_SEEN_REAL times. A real neighbour (a black buoy beside a
               red one) is seen in the same frames as a box of its own, so it is confirmed
               as usual; a wrong label replaces the real box in its frame, so it hardly
               ever is (once can happen: another buoy's wrong label in the frame). Nothing
               is deleted.
               Needs the sightings of a frame together (add_frame).

One more (phase 6 R3.5), off when its setting is 0:
  D7 copies    the GPS drifts (about a metre a minute without RTK), so a buoy seen again
               after a while can land more than merge_radius_m from its landmark and open a
               second one; a sighting joins only the nearest, so the old copy stays where
               it was, even once the drift has come back and the two are close again
               (t2d_s1: the black in channel gate 4 four times, confirmed, 0.6-1.9 m apart,
               two of them in the middle of the gate). After each frame, a landmark the
               frame showed absorbs any landmark of its kind within copy_merge_radius_m that
               it was not seen in CO_SEEN_REAL frames together with (two real buoys that
               close are seen together); the one with fewer sightings goes into the other.
               D5 suspects are not folded: wrong labels scattered round a buoy would add up
               past confirm_sightings, move off the buoy, out of D5's reach, and be
               confirmed. With D5 counting one shared frame as chance (co_seen summed over
               folded copies made them common), maps rebuilt from three bags with 2.0 m:
               objects mapped more than once t2d_s1 16 -> 0, t2c_s1 11 -> 0, survey1_d6
               10 -> 0, the dock indicators 2 m apart still two, wrong confirmed landmarks
               as without D7 (0, 2, 3), no object lost.

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

from usv_core.classes import BEACON_COLOR, INDICATOR_COLOR

# D7: seen in this many frames together, two landmarks are two buoys (one such frame can
# be a box with the wrong class: t2c_s1 had copies of two buoys seen together once); in
# the frame itself two landmarks seen together are never folded
CO_SEEN_REAL = 2
# D5: even seen together, a landmark beside one of another kind with this many times its
# sightings is a wrong label (one between two buoys is fed by both and seen with both:
# t2e_s1, a 'pole_green' with 3 sightings in channel gate 3 between its red and its black)
CONFUSION_STRONG = 30.0

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
    """Return the map kind of a class: the indicator colours, the beacon lights are one kind."""
    if class_name in INDICATOR_COLOR:
        return 'indicator'
    return 'beacon' if class_name in BEACON_COLOR else class_name


@dataclass(frozen=True)
class MapConfig:
    merge_radius_m: float = 1.5       # dock indicators stand 2 m apart
    max_range_m: float = 12.0         # legacy MAP_MAX_RANGE_M: farther positions not trusted
    sigma_base_m: float = 0.3         # ASSUMED sighting uncertainty: base + per metre range
    sigma_per_m: float = 0.1
    confirm_sightings: int = 3
    color_window: int = 15
    beacon_window_s: float = 2.5      # more than one flash cycle (1 s on, 1 s off)
    heading_lag_s: float = 0.0            # D4: 0 = off
    new_landmark_max_turn_dps: float = 0.0    # D4: 0 = off
    confusion_ratio: float = 0.0          # D5: 0 = off
    confusion_radius_m: float = 1.0
    copy_merge_radius_m: float = 0.0      # D7: 0 = off


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
    lights: deque = field(default_factory=lambda: deque(maxlen=400))   # beacon (time, colour)
    light_window_s: float = 2.5
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
        if self.kind == 'beacon':
            recent = [c for t, c in self.lights if t >= self.last_seen - self.light_window_s]
            lit = [c for c in recent if c != 'off']
            if lit:
                return Counter(lit).most_common(1)[0][0]
            return 'off' if recent else ''
        if not self.colors:
            return ''
        return Counter(self.colors).most_common(1)[0][0]


class WorldMap:

    def __init__(self, cfg=MapConfig()):
        self.cfg = cfg
        self.origin = None            # (lat, lon)
        self.landmarks = []
        self._next_id = 1
        self.copies_merged = 0        # D7

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
                            colors=deque(maxlen=cfg.color_window),
                            light_window_s=cfg.beacon_window_s)
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
        elif kind == 'beacon':
            best.lights.append((now, BEACON_COLOR[det['class_name']]))
        return best

    def add_frame(self, detections, now, yaw_rate_dps=0.0):
        """Fuse the sightings of one camera frame; remember which landmarks it showed together."""
        hits = [self.add(d, now, yaw_rate_dps) for d in detections]
        seen = {lm.landmark_id: lm for lm in hits if lm is not None}
        for a in seen.values():
            for b_id in seen:
                if b_id != a.landmark_id:
                    a.co_seen[b_id] = a.co_seen.get(b_id, 0) + 1
        if self.cfg.copy_merge_radius_m > 0:
            for a in seen.values():
                self._merge_copies(a, seen)
        return hits

    def _merge_copies(self, a, seen):
        """D7: fold into a (or a into it) every copy of its kind close by, never seen with it."""
        for b in list(self.landmarks):
            if b is a or b.kind != a.kind or b not in self.landmarks or \
                    a not in self.landmarks or b.landmark_id in seen:
                continue                      # (in this frame together: two, for now)
            if max(a.co_seen.get(b.landmark_id, 0), b.co_seen.get(a.landmark_id, 0)) \
                    >= CO_SEEN_REAL:
                continue
            if math.hypot(a.east - b.east, a.north - b.north) > self.cfg.copy_merge_radius_m:
                continue
            if self.suspect(a) or self.suspect(b):
                continue                      # wrong labels stay apart (and unconfirmed)
            keep, gone = (a, b) if a.sightings >= b.sightings else (b, a)
            self._fold(keep, gone)
            a = keep

    def _fold(self, keep, gone):
        keep.sum_w += gone.sum_w
        keep.sum_we += gone.sum_we
        keep.sum_wn += gone.sum_wn
        keep.sightings += gone.sightings
        keep.first_seen = min(keep.first_seen, gone.first_seen)
        keep.last_seen = max(keep.last_seen, gone.last_seen)
        keep.closest_m = min(keep.closest_m, gone.closest_m)
        keep.colors.extend(gone.colors)
        keep.lights = deque(sorted(list(keep.lights) + list(gone.lights)),
                            maxlen=keep.lights.maxlen)
        self.landmarks.remove(gone)
        for lm in self.landmarks:             # who was seen with the copy was seen with it
            n = lm.co_seen.pop(gone.landmark_id, 0)
            if n and lm is not keep:
                lm.co_seen[keep.landmark_id] = lm.co_seen.get(keep.landmark_id, 0) + n
        for other_id, n in gone.co_seen.items():
            if other_id != keep.landmark_id:
                keep.co_seen[other_id] = keep.co_seen.get(other_id, 0) + n
        self.copies_merged += 1

    def suspect(self, lm):
        """D5: True if lm looks like a wrong class on top of a landmark of another kind."""
        cfg = self.cfg
        if cfg.confusion_ratio <= 0:
            return False
        for other in self.landmarks:
            if other.kind == lm.kind:
                continue
            ratio = CONFUSION_STRONG if lm.co_seen.get(other.landmark_id, 0) >= CO_SEEN_REAL \
                else cfg.confusion_ratio
            if other.sightings >= ratio * lm.sightings and math.hypot(
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
