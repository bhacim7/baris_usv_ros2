"""
RoboBoat moves: the building blocks the run's phases are made of (phase 6 R2.3).

Every move turns its job into a PATH (map east / north points) and one PathFollower drives
it, with the legacy Pure Pursuit (usv_core.planner) and the TEKNOFEST tuning: cruise PWM,
spot turn above pp_spot_turn_enter_deg (back below pp_spot_turn_exit_deg), throttle taper.
So every move steers the same way, and a steering fix is made in one place.

  DriveTo   to a point, round obstacles: A* (usv_core.planner) on a grid of the world
            map's landmarks around the boat (LandmarkGrid), replanned every replan_s; a
            straight line when it is clear; straight at the goal when A* finds nothing
  PassGate  through a gate: a straight line from approach_m before its centre, square to
            it, to exit_m past it. Starts from the map's gate; whenever the camera sees
            the gate's poles / buoys, their positions in THAT frame replace it. A frame's
            detections and the boat's position come from the same GPS fix, so the gate is
            right relative to the boat even when the map is off by the GPS drift
  Circle    round an object at radius_m, clockwise or counter-clockwise, until laps x 360
            deg are swept round it; the object refined from the camera as for PassGate
  Search    look for something: one turn on the spot, then an expanding square round a
            point, until found(ctx) is true (DONE) or the pattern is used up (SKIPPED)

A move has start(ctx) and step(ctx) -> (state, command), as a run phase: state RUNNING /
DONE / SKIPPED (usv_core.roboboat_run), command (forward_pwm, yaw_pwm) for the mixer or
None for neutral. Moves read ctx.boat_xy, ctx.heading, ctx.speed, ctx.now, ctx.landmarks,
ctx.detections and ctx.map_origin; results for later phases go to self.result.

Frames: map east / north metres here; the planner's own x = North, y = East inside
(to_planner / from_planner).
"""

import math

import cv2
from usv_core import planner
from usv_core.costmap import LocalCostmap
from usv_core.roboboat_run import DONE, RUNNING, SKIPPED
from usv_core.world_map import enu

# Obstacle radius drawn on the grid per map kind (m); the planner inflates it by the
# boat's radius and its margin. Vessel marks stand for the whole vessel (1.7 x 0.7 m),
# slip banners for the dock end they stand on.
OBSTACLE_RADIUS_M = {
    'pole_red': 0.23, 'pole_green': 0.23, 'buoy_red': 0.10, 'buoy_green': 0.10,
    'buoy_black': 0.10, 'buoy_yellow': 0.18, 'indicator': 0.23, 'triangle': 0.85,
    'plus': 0.85, 'digit_1': 0.5, 'digit_2': 0.5, 'digit_3': 0.5,
}
DETECTION_MAX_AGE_S = 0.5
MATCH_M = 2.0                 # a detection replaces a map position within this


def to_planner(p):
    return (p[1], p[0])


def from_planner(p):
    return (p[1], p[0])


def _wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def detections_xy(ctx):
    """Return (class_name, (east, north)) of the fresh, placed detections of this frame."""
    if ctx.map_origin is None or ctx.now - ctx.detections_time > DETECTION_MAX_AGE_S:
        return []
    return [(d['class_name'], enu(d['lat'], d['lon'], *ctx.map_origin))
            for d in ctx.detections if d['position_ok']]


def refine(position, class_name, ctx, alpha=0.5):
    """Move a map position towards this frame's nearest detection of that class."""
    best = None
    for name, xy in detections_xy(ctx):
        if name == class_name and _dist(xy, position) <= MATCH_M:
            if best is None or _dist(xy, position) < _dist(best, position):
                best = xy
    if best is None:
        return position, False
    return ((1 - alpha) * position[0] + alpha * best[0],
            (1 - alpha) * position[1] + alpha * best[1]), True


class LandmarkGrid(LocalCostmap):
    """The planner's grid, drawn from world map landmarks around the boat."""

    def rebuild(self, boat_xy, landmarks, min_sightings=2):
        self.center_m = to_planner(boat_xy)
        self.img.fill(127)
        reach = self.crop_radius_m * 1.5
        for lm in landmarks:
            if not (lm['confirmed'] or lm['sightings'] >= min_sightings):
                continue
            xy = (lm['east'], lm['north'])
            if _dist(xy, boat_xy) > reach:
                continue
            p = self.world_to_pixel(*to_planner(xy))
            if p:
                r = OBSTACLE_RADIUS_M.get(lm['kind'], 0.25)
                cv2.circle(self.img, p, max(1, int(round(r / self.res))), 0, -1)


class PathFollower:
    """Pure Pursuit along a map path, with the TEKNOFEST throttle and spot-turn rules."""

    def __init__(self, cfg, planner_cfg=planner.DEFAULT_CONFIG):
        self.cfg = cfg
        self.pcfg = planner_cfg
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.spot_turn = False
        self.path_idx = 0
        self.path_id = None

    def follow(self, path, boat_xy, heading, speed, dt, cruise_offset=None):
        """Return (forward_pwm, yaw_pwm) for the next cycle along `path` (map points)."""
        cfg = self.cfg
        if id(path) != self.path_id:
            self.path_id, self.path_idx = id(path), 0
        offset = cfg.cruise_pwm if cruise_offset is None else cruise_offset
        rx, ry = to_planner(boat_xy)
        pts = [to_planner(p) for p in path]
        _, steer, _, err, self.path_idx = planner.pure_pursuit_control(
            rx, ry, math.radians(heading), pts, current_speed=speed,
            base_speed=cfg.base_pwm + offset, prev_error=self.prev_error, dt=dt,
            start_idx=self.path_idx, cfg=self.pcfg)
        self.prev_error = err
        if self.spot_turn:
            if abs(err) < cfg.pp_spot_turn_exit_deg:
                self.spot_turn = False
        elif abs(err) > cfg.pp_spot_turn_enter_deg:
            self.spot_turn = True
        if self.spot_turn:
            spot = cfg.spot_turn_pwm
            return cfg.base_pwm, int(math.copysign(spot, err)) if err else 0
        taper = max(cfg.pp_throttle_taper_min, math.cos(math.radians(min(abs(err), 90.0))))
        return cfg.base_pwm + offset * taper, steer


class Move:
    """Common bookkeeping: the follower, the cycle time, a result dict."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.follower = PathFollower(cfg)
        self.last_t = None
        self.result = {}

    def start(self, ctx):
        self.follower.reset()
        self.last_t = ctx.now
        self.result = {}

    def _dt(self, ctx):
        dt = 0.04 if self.last_t is None else min(0.5, max(0.001, ctx.now - self.last_t))
        self.last_t = ctx.now
        return dt

    def _follow(self, path, ctx, dt):
        return self.follower.follow(path, ctx.boat_xy, ctx.heading, ctx.speed, dt)


class DriveTo(Move):

    def __init__(self, cfg, goal, arrive_m=1.5, replan_s=0.2, avoid=True):
        super().__init__(cfg)
        self.goal, self.arrive_m, self.replan_s, self.avoid = goal, arrive_m, replan_s, avoid
        self.grid = LandmarkGrid(cfg.costmap_res_m_per_px, cfg.crop_radius_m,
                                 cfg.buoy_radius_m) if avoid else None
        self.path = None
        self.planned_at = None

    def start(self, ctx):
        super().start(ctx)
        self.path, self.planned_at = None, None

    def step(self, ctx):
        dt = self._dt(ctx)
        if ctx.boat_xy is None:
            return RUNNING, None
        if _dist(ctx.boat_xy, self.goal) <= self.arrive_m:
            return DONE, None
        if self.path is None or ctx.now - self.planned_at >= self.replan_s:
            self.path = self._plan(ctx)
            self.planned_at = ctx.now
        return RUNNING, self._follow(self.path, ctx, dt)

    def _plan(self, ctx):
        straight = [ctx.boat_xy, self.goal]
        if not self.avoid:
            return straight
        self.grid.rebuild(ctx.boat_xy, ctx.landmarks)
        start, goal = to_planner(ctx.boat_xy), to_planner(self.goal)
        crop, centre, size = self.grid.crop(*start)
        nav_map, _ = planner.get_inflated_nav_map(crop)
        res = self.grid.res
        if planner.check_line_of_sight(start, goal, nav_map, centre, res, size):
            return straight
        path = planner.get_path_plan(start, goal, nav_map, centre, res, size,
                                     heuristic_weight=self.cfg.a_star_heuristic_weight,
                                     max_expansions=40000, time_budget_s=0.15)
        if not path or len(path) < 2:
            self.result['blind'] = self.result.get('blind', 0) + 1
            return straight
        return [from_planner(p) for p in path]


class PassGate(Move):

    def __init__(self, cfg, gate, reverse=False, approach_m=4.0, exit_m=2.0):
        super().__init__(cfg)
        self.red = (gate.red['east'], gate.red['north'])
        self.green = (gate.green['east'], gate.green['north'])
        self.red_class = 'pole_red' if gate.red['kind'] == 'pole_red' else 'buoy_red'
        self.green_class = 'pole_green' if gate.green['kind'] == 'pole_green' else 'buoy_green'
        self.reverse, self.approach_m, self.exit_m = reverse, approach_m, exit_m
        self.path = None

    def geometry(self):
        """Return (centre, through unit vector, left unit vector, width) of the gate now."""
        c = ((self.red[0] + self.green[0]) / 2.0, (self.red[1] + self.green[1]) / 2.0)
        lx, ly = self.red[0] - self.green[0], self.red[1] - self.green[1]
        w = math.hypot(lx, ly) or 1.0
        left = (lx / w, ly / w)                   # red on the left going through
        through = (left[1], -left[0])
        if self.reverse:                          # the other way: red on the right
            through, left = (-through[0], -through[1]), (-left[0], -left[1])
        return c, through, left, w

    def start(self, ctx):
        super().start(ctx)
        self.path = None

    def step(self, ctx):
        dt = self._dt(ctx)
        if ctx.boat_xy is None:
            return RUNNING, None
        self.red, seen_r = refine(self.red, self.red_class, ctx)
        self.green, seen_g = refine(self.green, self.green_class, ctx)
        if seen_r and seen_g:
            self.result['camera'] = self.result.get('camera', 0) + 1
        c, t, left, w = self.geometry()
        d = (ctx.boat_xy[0] - c[0], ctx.boat_xy[1] - c[1])
        along = d[0] * t[0] + d[1] * t[1]
        side = d[0] * left[0] + d[1] * left[1]
        if 'crossed_at' not in self.result and along >= 0.0:
            self.result['crossed_at'] = side       # left of centre at the gate line
            self.result['through'] = abs(side) < w / 2.0
        if along >= self.exit_m:
            return DONE, None
        self.path = [(c[0] - t[0] * self.approach_m, c[1] - t[1] * self.approach_m), c,
                     (c[0] + t[0] * (self.exit_m + 2.0), c[1] + t[1] * (self.exit_m + 2.0))]
        if along < -self.approach_m:
            self.path.insert(0, ctx.boat_xy)
        return RUNNING, self._follow(self.path, ctx, dt)


class Circle(Move):

    def __init__(self, cfg, target, class_name, radius_m=2.0, clockwise=True, laps=1.0,
                 step_deg=15.0):
        super().__init__(cfg)
        self.target, self.class_name = target, class_name
        self.radius_m, self.clockwise, self.laps = radius_m, clockwise, laps
        self.step_deg = step_deg
        self.swept = 0.0
        self.last_angle = None
        self.path = None
        self.path_target = None

    def start(self, ctx):
        super().start(ctx)
        self.swept, self.last_angle, self.path, self.path_target = 0.0, None, None, None

    def _angle(self, xy):
        return math.degrees(math.atan2(xy[1] - self.target[1], xy[0] - self.target[0]))

    def step(self, ctx):
        dt = self._dt(ctx)
        if ctx.boat_xy is None:
            return RUNNING, None
        self.target, _ = refine(self.target, self.class_name, ctx, alpha=0.2)
        a = self._angle(ctx.boat_xy)
        if self.last_angle is not None:
            delta = _wrap180(a - self.last_angle)       # + = counter-clockwise (ENU)
            self.swept += -delta if self.clockwise else delta
        self.last_angle = a
        if self.swept >= 360.0 * self.laps:
            self.result['swept_deg'] = self.swept
            return DONE, None
        if self.path is None or _dist(self.path_target, self.target) > 0.3 or \
                self.follower.path_idx >= len(self.path) - 3:
            self.path = self._ring(a)
            self.path_target = self.target
        return RUNNING, self._follow(self.path, ctx, dt)

    def _ring(self, from_angle):
        """Points round the target from the boat's angle, one turn and a bit, in order."""
        sign = -1.0 if self.clockwise else 1.0
        n = int(400.0 / self.step_deg)
        pts = []
        for k in range(1, n + 1):
            ang = math.radians(from_angle + sign * k * self.step_deg)
            pts.append((self.target[0] + self.radius_m * math.cos(ang),
                        self.target[1] + self.radius_m * math.sin(ang)))
        return pts


class Search(Move):

    def __init__(self, cfg, centre, found, leg_m=4.0, max_m=16.0, spin_deg=360.0):
        super().__init__(cfg)
        self.centre, self.found = centre, found
        self.leg_m, self.max_m, self.spin_deg = leg_m, max_m, spin_deg
        self.turned = 0.0
        self.last_heading = None
        self.path = None

    def start(self, ctx):
        super().start(ctx)
        self.turned, self.last_heading, self.path = 0.0, None, None

    def step(self, ctx):
        dt = self._dt(ctx)
        if self.found(ctx):
            return DONE, None
        if ctx.boat_xy is None:
            return RUNNING, None
        if self.turned < self.spin_deg:               # look round first
            if self.last_heading is not None:
                self.turned += abs(_wrap180(ctx.heading - self.last_heading))
            self.last_heading = ctx.heading
            return RUNNING, (self.cfg.base_pwm, self.cfg.spot_turn_pwm)
        if self.path is None:
            self.path = self._square(ctx.boat_xy)
        if _dist(ctx.boat_xy, self.path[-1]) < 1.5:
            return SKIPPED, None
        return RUNNING, self._follow(self.path, ctx, dt)

    def _square(self, boat_xy):
        """Return an expanding square round the centre: east, north, west, south, legs growing."""
        pts = [boat_xy, self.centre]
        x, y = self.centre
        leg, k = self.leg_m, 0
        dirs = ((1, 0), (0, 1), (-1, 0), (0, -1))
        while leg <= 2 * self.max_m:
            dx, dy = dirs[k % 4]
            x, y = x + dx * leg, y + dy * leg
            pts.append((x, y))
            if k % 2 == 1:
                leg += self.leg_m
            k += 1
        return pts
