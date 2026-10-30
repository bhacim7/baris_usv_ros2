"""
The world both navigators are run in for the golden comparison.

Everything here is deterministic: the same scenario always produces the same inputs, so
the legacy nav_worker and the new navigator can be compared cycle by cycle. Nothing in here
tries to be realistic physics (that is Gazebo's job); it only has to react to the PWMs so
the navigation is exercised in closed loop.

  BoatModel        3-DOF kinematics driven by the five PWM channels: surge with the fitted
                   drag of the Gazebo model, yaw from differential thrust and the steered
                   front pair, no sway
  Perception       what camera_process's object memory would hand to the navigation: every
                   buoy within 15 m and the 110 deg view, exact positions, legacy fields
  Scenario         waypoints, buoys, start pose, scripted GCS commands, run length
  SCENARIOS        the named scenarios the tests use
"""

from dataclasses import dataclass, field
import math

# ---------------------------------------------------------------- geometry (WGS84 local)
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _radii(lat_deg):
    s = math.sin(math.radians(lat_deg))
    w = 1.0 - WGS84_E2 * s * s
    return (WGS84_A * (1.0 - WGS84_E2) / (w * math.sqrt(w)),
            WGS84_A / math.sqrt(w) * math.cos(math.radians(lat_deg)))


def offset(lat, lon, north_m, east_m):
    n_r, e_r = _radii(lat)
    return lat + math.degrees(north_m / n_r), lon + math.degrees(east_m / e_r)


def local(lat, lon, origin_lat, origin_lon):
    """(east, north) metres of a point from an origin."""
    n_r, e_r = _radii(origin_lat)
    return (math.radians(lon - origin_lon) * e_r, math.radians(lat - origin_lat) * n_r)


def along(lat, lon, dist_m, bearing_deg):
    b = math.radians(bearing_deg)
    return offset(lat, lon, dist_m * math.cos(b), dist_m * math.sin(b))


# ---------------------------------------------------------------- boat
THRUST_MAX_N = 91.0          # 1900 us, Degz Utras on 7S (model.sdf)
THRUST_MIN_N = -64.0         # 1100 us
MASS_KG = 45.0
IZZ = 7.14
X_U, X_UU = 70.5, 39.2       # surge drag, fitted from the 2026-10-14 video
N_R, N_RR = 28.0, 35.0       # yaw drag
Y_SIDE = 0.245               # thrusters off the centre line
X_FRONT = 0.363              # front pair ahead of the centre
STEER_MAX_DEG = 45.0


def thrust(pwm):
    d = (pwm - 1500) / 400.0
    return max(THRUST_MIN_N, min(THRUST_MAX_N, d * (THRUST_MAX_N if d >= 0 else -THRUST_MIN_N)))


@dataclass
class BoatState:
    east: float
    north: float
    heading: float           # compass degrees
    surge: float = 0.0       # m/s
    yaw_rate: float = 0.0    # deg/s, clockwise positive


class BoatModel:

    def __init__(self, origin_lat, origin_lon, start_lat, start_lon, heading_deg):
        self.origin = (origin_lat, origin_lon)
        e, n = local(start_lat, start_lon, origin_lat, origin_lon)
        self.s = BoatState(e, n, heading_deg % 360.0)

    def step(self, pwm, dt):
        """pwm: dict rear_left, rear_right, front_left, front_right, steer (us)."""
        s = self.s
        t_rl, t_rr = thrust(pwm['rear_left']), thrust(pwm['rear_right'])
        t_fl, t_fr = thrust(pwm['front_left']), thrust(pwm['front_right'])
        delta = math.radians(max(-STEER_MAX_DEG, min(
            STEER_MAX_DEG, (pwm['steer'] - 1500) / 400.0 * STEER_MAX_DEG)))
        fx = t_rl + t_rr + (t_fl + t_fr) * math.cos(delta)
        # left thrusters pushing harder, or the bow pushed right, turn the boat clockwise
        moment = (t_rl - t_rr + (t_fl - t_fr) * math.cos(delta)) * Y_SIDE \
            + (t_fl + t_fr) * math.sin(delta) * X_FRONT
        u = s.surge
        r = math.radians(s.yaw_rate)
        u += (fx - (X_U * u + X_UU * u * abs(u))) / MASS_KG * dt
        r += (moment - (N_R * r + N_RR * r * abs(r))) / IZZ * dt
        s.surge = u
        s.yaw_rate = math.degrees(r)
        s.heading = (s.heading + s.yaw_rate * dt) % 360.0
        h = math.radians(s.heading)
        s.east += u * math.sin(h) * dt
        s.north += u * math.cos(h) * dt

    def latlon(self):
        return offset(self.origin[0], self.origin[1], self.s.north, self.s.east)


def mavlink_deg(value):
    """GPS_RAW_INT carries degrees * 1e7 as integers."""
    return round(value * 1e7) / 1e7


# ---------------------------------------------------------------- perception
CID_TO_COLOR = {0: 3, 1: 1, 2: 2, 3: 5, 4: 4}
CAM_FX = 640.0 / math.tan(math.radians(55.0))
CAM_AHEAD_M = 0.29
VIEW_HALF_DEG = 55.0
VALID_RANGE_M = 15.0         # camera_process depth gate
MAP_MAX_RANGE_M = 12.0       # object memory trust gate
MEMORY_S = 20.0


@dataclass
class Buoy:
    lat: float
    lon: float
    cid: int
    radius: float = 0.15
    height: float = 0.43


class Perception:
    """A deterministic stand-in for the object memory list ('vision_detected_objects')."""

    def __init__(self, buoys):
        self.buoys = buoys
        self.tracks = {}

    def update(self, boat, now):
        lat, lon = boat.latlon()
        h = math.radians(boat.s.heading)
        cam_lat, cam_lon = offset(lat, lon, CAM_AHEAD_M * math.cos(h), CAM_AHEAD_M * math.sin(h))
        for i, b in enumerate(self.buoys):
            e, n = local(b.lat, b.lon, cam_lat, cam_lon)
            rng = math.hypot(e, n) - b.radius
            rel = (math.degrees(math.atan2(e, n)) - boat.s.heading + 180.0) % 360.0 - 180.0
            if not (0.5 < rng < VALID_RANGE_M and abs(rel) <= VIEW_HALF_DEG):
                continue
            t = self.tracks.setdefault(i + 1, {
                'type': 3, 'color': CID_TO_COLOR[b.cid], 'lat': b.lat, 'lon': b.lon,
                'id': i + 1, 'cid': b.cid, 'seen': 0, 'pos_ok': False})
            px_w = CAM_FX * 2 * b.radius / rng
            px_h = CAM_FX * b.height / rng
            t.update(cx=int(640 + CAM_FX * math.tan(math.radians(rel))), cy=400, dist=rng,
                     area=int(px_w * px_h), last_seen=now)
            if rng <= MAP_MAX_RANGE_M:
                t['seen'] += 1
                t['pos_ok'] = True
        for k in [k for k, t in self.tracks.items() if now - t['last_seen'] >= MEMORY_S]:
            del self.tracks[k]
        return [dict(t) for t in self.tracks.values()]


# ---------------------------------------------------------------- scenarios
@dataclass
class Scenario:
    name: str
    waypoints: dict              # config attribute -> (lat, lon), e.g. 'T2_ZONE_MID'
    start: tuple                 # (lat, lon, heading_deg)
    start_task: str
    duration_s: float
    buoys: list = field(default_factory=list)
    manual: bool = False
    mission_started: bool = True
    target_color: str = 'red'
    commands: list = field(default_factory=list)    # (time_s, command dict) as the GCS sends
    stop_when_finished: bool = True
    heading_offset_deg: float = 0.0                 # compass error added to every heading


# Golcuk 2026-10-14 (HUD) and the provisional course of usv_sim/courses/golcuk_2026_10_14.yaml
G_T2_ENTRY = (40.724038, 29.825102)
G_T2_MID = (40.723589, 29.825499)
G_T3_START = (40.723576, 29.825515)
G_T3_MID = (40.723547, 29.825561)
G_START = (40.724066, 29.825080, 146.0)
G_RED = (40.7234713, 29.8256045)
G_GREEN = (40.7234745, 29.8255374)
G_BLACK = (40.7235116, 29.8256511)
G_CORRIDOR = [
    # left boundary (orange)
    (40.7240530, 29.8251315, 3), (40.7239858, 29.8251909, 3), (40.7239187, 29.8252503, 3),
    (40.7238552, 29.8253064, 3), (40.7237918, 29.8253625, 3), (40.7237246, 29.8254219, 3),
    (40.7236649, 29.8254747, 3), (40.7236052, 29.8255274, 3),
    # right boundary (orange)
    (40.7239631, 29.8250398, 3), (40.7238960, 29.8250992, 3), (40.7238288, 29.8251585, 3),
    (40.7237617, 29.8252179, 3), (40.7236945, 29.8252773, 3), (40.7236273, 29.8253367, 3),
    (40.7235602, 29.8253961, 3),
    # obstacles (yellow)
    (40.7239111, 29.8251928, 1), (40.7238574, 29.8252360, 1), (40.7238202, 29.8252375, 1),
    (40.7237892, 29.8253148, 1), (40.7237360, 29.8253377, 1), (40.7236722, 29.8253612, 1),
    (40.7236310, 29.8254504, 1),
]
G_TARGETS = [(G_RED[0], G_RED[1], 0), (G_GREEN[0], G_GREEN[1], 4), (G_BLACK[0], G_BLACK[1], 2)]


def _buoys(rows, radius=0.15, height=0.43):
    return [Buoy(lat, lon, cid, radius, height) for lat, lon, cid in rows]


def _golcuk_buoys():
    return _buoys(G_CORRIDOR) + _buoys(G_TARGETS, radius=0.32, height=0.85)


def _golcuk_waypoints():
    # Task 1 gates are not in the video: three points leading into the Task 2 entry from the
    # north-west, 8 m apart, so a full mission can start at Task 1.
    g3 = along(G_T2_ENTRY[0], G_T2_ENTRY[1], 8.0, 326.0)
    g2 = along(g3[0], g3[1], 8.0, 326.0)
    g1 = along(g2[0], g2[1], 8.0, 326.0)
    return {
        'T1_GATE_ENTER': g1, 'T1_GATE_MID': g2, 'T1_GATE_EXIT': g3,
        'T2_ZONE_ENTRY': G_T2_ENTRY, 'T2_ZONE_MID': G_T2_MID, 'T2_ZONE_END': G_T2_MID,
        'T3_START': G_T3_START, 'T3_MID': G_T3_MID,
    }


def _scenarios():
    wp = _golcuk_waypoints()
    t1_start = along(wp['T1_GATE_ENTER'][0], wp['T1_GATE_ENTER'][1], 6.0, 326.0)
    far = along(G_T3_MID[0], G_T3_MID[1], 45.0, 60.0)      # a red target out of the box
    return {s.name: s for s in [
        Scenario('task1_gates', wp, (t1_start[0], t1_start[1], 100.0), 'TASK1_STATE_ENTER',
                 40.0, buoys=_golcuk_buoys(), stop_when_finished=False),
        Scenario('golcuk_task2', wp, G_START, 'TASK2_START', 90.0, buoys=_golcuk_buoys(),
                 stop_when_finished=False),
        Scenario('golcuk_task3', wp, (G_T3_START[0], G_T3_START[1], 150.0), 'T3_START', 60.0,
                 buoys=_golcuk_buoys()),
        Scenario('golcuk_full', wp, G_START, 'TASK2_START', 150.0, buoys=_golcuk_buoys()),
        Scenario('full_mission', wp, (t1_start[0], t1_start[1], 140.0), 'TASK1_STATE_ENTER',
                 200.0, buoys=_golcuk_buoys()),
        Scenario('task3_search', wp, (G_T3_START[0], G_T3_START[1], 150.0), 'T3_START', 90.0,
                 buoys=_buoys(G_CORRIDOR) + _buoys([(far[0], far[1], 0)], 0.32, 0.85)),
        Scenario('manual_rollback', wp, G_START, 'TASK2_START', 20.0, buoys=_golcuk_buoys(),
                 manual=True),
        Scenario('manual_then_auto', wp, G_START, 'TASK2_START', 40.0, buoys=_golcuk_buoys(),
                 manual=True, commands=[(5.0, {'cmd': 'set_manual', 'value': False}),
                                        (25.0, {'cmd': 'set_manual', 'value': True})]),
        Scenario('gcs_commands', wp, G_START, 'TASK2_START', 30.0, buoys=_golcuk_buoys(),
                 commands=[
                     (3.0, {'cmd': 'set_gps', 'index': 5, 'lat': G_T3_START[0],
                            'lon': G_T3_START[1], 'cq': 1}),
                     (3.04, {'cmd': 'set_gps', 'index': 5, 'lat': 0.0, 'lon': 0.0, 'cq': 1}),
                     (10.0, {'cmd': 'set_task', 'task_name': 'T3_START', 'cq': 2}),
                     (12.0, {'cmd': 'set_target_color', 'color': 'black', 'cq': 3}),
                     (20.0, {'cmd': 'set_task', 'task_name': 'TASK2_GO_TO_MID',
                             'target_id': 99, 'cq': 4})]),
        Scenario('compass_error', wp, G_START, 'TASK2_START', 60.0, buoys=_golcuk_buoys(),
                 heading_offset_deg=8.0, stop_when_finished=False),
    ]}


SCENARIOS = _scenarios()
