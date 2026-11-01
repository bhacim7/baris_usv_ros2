"""
RoboBoat moves: the building blocks the run's phases are made of (phase 6 R2.3).

Every move turns its job into a PATH (map east / north points) and one PathFollower drives
it, with the legacy Pure Pursuit (usv_core.planner) and the TEKNOFEST tuning: cruise PWM,
spot turn above pp_spot_turn_enter_deg (back below pp_spot_turn_exit_deg), throttle taper.
So every move steers the same way, and a steering fix is made in one place.

  DriveTo   to a point, round obstacles: A* (usv_core.planner) on a grid of the world
            map's landmarks around the boat (LandmarkGrid), replanned every replan_s; a
            straight line when it is clear; straight at the goal when A* finds nothing
  PassGate  through a gate: a straight line from approach_m before it, square to it, to
            exit_m past it. Starts from the map's gate; whenever the camera sees the gate's
            poles / buoys, their positions in THAT frame replace it. A frame's detections
            and the boat's position come from the same GPS fix, so the gate is right
            relative to the boat even when the map is off by the GPS drift. Obstacles of
            other kinds near the gate's line (a black buoy beside the red one, as in the
            channel) narrow it: the line goes through the middle of the widest gap left for
            the hull (free_gap), not the gate's centre. With avoid (the channel, R3.2c)
            the way there and through is planned round every obstacle near the gate by
            A* on a visibility graph (usv_core.roboboat_avoid), with walls along the gate's
            line beyond its buoys so the path cannot go round the gate. Planned round
            the map's confirmed objects only (a single frame's boxes can be mislabelled:
            t2c_s1 saw digits on the channel buoys), each put where this frame's camera
            sees one of its kind within OBJECT_MATCH_M, else moved by the camera's shift
            of the gate (the GPS drift since it was mapped). The camera refines the gate
            only with boxes clear of the image's sides, within 0.4 x its width of a
            marker, keeping its width within WIDTH_TOL_M of the map's (t2c_s1: the black
            and the red, each once labelled green, pulled the green 1.4 m in), and not
            in the last GATE_FREEZE_M before the line
  Circle    round an object at radius_m, clockwise or counter-clockwise, until laps x 360
            deg are swept round it; the object refined from the camera as for PassGate
  Search    look for something: one turn on the spot, then an expanding square round a
            point, until found(ctx) is true (DONE) or the pattern is used up (SKIPPED)

A move has start(ctx) and step(ctx) -> (state, command), as a run phase: state RUNNING /
DONE / SKIPPED (usv_core.roboboat_run), command (forward_pwm, yaw_pwm) for the mixer or
None for neutral. Moves read ctx.boat_xy, ctx.heading, ctx.speed, ctx.now, ctx.landmarks,
ctx.detections and ctx.map_origin; results for later phases go to self.result. ctx.boat_xy
is the GPS antenna's place; the moves steer, plan and measure from the hull's centre,
DriveConfig.centre_ahead_m ahead of it (Move.here): planned from the antenna, the bow, 1.3 m
ahead of it, brushed a black buoy in a turn (t2e_s1).

DriveTo and PassGate are watched for being stuck (Move._guard): no STUCK_M of progress
(along the gate's way; path left to the goal) in STUCK_S, whatever the commands (t2c_s1
pivoted round a buoy, then turned on the spot, for 35 s), the boat backs off BACK_S
astern, a bump goes where the bow was and the path is planned again; past MAX_STUCK the
move is SKIPPED. A spot turn counts SPOT_SHARE of its time: turning round after the
channel's end took 6 s and was taken for stuck (t2e_s1).

Frames: map east / north metres here; the planner's own x = North, y = East inside
(to_planner / from_planner).
"""

from dataclasses import dataclass, replace
import math

import cv2
from usv_core import planner
from usv_core.costmap import LocalCostmap
from usv_core.roboboat_avoid import Obstacles, plan_through
from usv_core.roboboat_run import DONE, RUNNING, SKIPPED
from usv_core.world_map import enu, kind_of

# Obstacle radius drawn on the grid per map kind (m); the planner inflates it by the
# boat's radius and its margin. Vessel marks stand for the whole vessel (1.7 x 0.7 m),
# slip banners for the dock end they stand on.
OBSTACLE_RADIUS_M = {
    'pole_red': 0.23, 'pole_green': 0.23, 'buoy_red': 0.10, 'buoy_green': 0.10,
    'buoy_black': 0.10, 'buoy_yellow': 0.18, 'indicator': 0.23, 'triangle': 0.85,
    'plus': 0.85, 'digit_1': 0.5, 'digit_2': 0.5, 'digit_3': 0.5,
}
DETECTION_MAX_AGE_S = 0.5
MATCH_M = 2.0                 # a detection replaces a map position within this ...
GATE_MATCH_SHARE = 0.4        # ... and within this share of a gate's width
WIDTH_TOL_M = 0.6             # a gate refined to a width further from the map's is not
GATE_FREEZE_M = 1.0           # the gate is not refined in the last metre before its line
OBJECT_MATCH_M = 1.0          # avoid: a detection places a map object of its kind within
HALF_BEAM_M = 0.39            # half the hull's width (0.78 m)
GAP_MARGIN_M = 0.15           # kept clear of each side of a gap in a gate
GATE_LINE_M = 1.5             # obstacles this close to a gate's line narrow it
# Clearance kept by PassGate's avoid paths, hull centre line to an object's edge: a gate's
# (channel's) red / green markers as in free_gap, anything else a little more (both drop to
# the gate's if no path keeps it). Walls go WALL_M out from a gate's buoys.
KEEP_GATE_M = HALF_BEAM_M + GAP_MARGIN_M
KEEP_OTHER_M = HALF_BEAM_M + 0.35
GATE_KINDS = ('pole_red', 'pole_green', 'buoy_red', 'buoy_green')
WALL_M = 3.0
AVOID_RANGE_M = 12.0          # objects further from the boat are not planned round
BENT_CRUISE = 0.6             # share of the cruise PWM while an avoid path bends round
PATH_DETOUR = 1.5             # avoid paths longer than this x straight + 2 m are refused
# Stuck watch (Move._guard)
STUCK_S = 6.0                 # this long ...
STUCK_M = 0.3                 # ... without this much progress is stuck
SPOT_SHARE = 0.4              # a second turning on the spot counts this much
BACK_S = 1.5                  # then astern this long (the mixer reverses below 1450)
BACK_PWM = 1350
MAX_STUCK = 3                 # back-offs before the move gives up (SKIPPED)
BUMP_AHEAD_M = 0.9            # a bump is put this far ahead of the hull's centre
BUMP_RADIUS_M = 0.15


@dataclass(frozen=True)
class DriveConfig:
    """
    RoboBoat path-following settings (navigator.yaml roboboat.drive.*).

    lookahead_m: Pure Pursuit aims this far ahead along the path. The TEKNOFEST tuning
    (0.8-2.5 m with the speed, ~1.3 m cruising) made the Gazebo boat weave round straight
    legs, ~10 s period, +/-30-40 deg (t1_run1/2, 2026-11-01): with its yaw inertia a near
    aim point is overshot. Measured on Task 1 (seed 1): 1.3 m 54.6 s, 2.5 m 46.0 s,
    4.0 m 43.5 s and the smoothest (entry yaw rate 7.4 deg/s, gates within 0.3 m). To be
    checked again in the 2.5 m channel gates (R3). Circle uses at most 0.8 x its radius.

    avoid_lookahead_m: used by DriveTo while A* bends the path round obstacles: a far aim
    point cuts the corner (4.0 m brushed the end of a buoy wall in the test, 2.5 m did not).
    """

    lookahead_m: float = 4.0
    avoid_lookahead_m: float = 2.0
    # channel_lookahead_m: through the Task 2 channel (2.5 m gates ~6 m apart), where a far
    # aim point may cut from gate to gate; to be measured (R3.2)
    channel_lookahead_m: float = 2.5
    # centre_ahead_m: the hull's centre ahead of the GPS antenna (boat_xy), m; measured
    # 0.46 (antenna 0.75 m behind the ZED, model.sdf navsat at -0.46)
    centre_ahead_m: float = 0.46


def to_planner(p):
    return (p[1], p[0])


def from_planner(p):
    return (p[1], p[0])


def _wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _length(path):
    return sum(_dist(a, b) for a, b in zip(path, path[1:]))


def detections_xy(ctx):
    """
    Return (class_name, (east, north)) of the fresh, placed detections of this frame.

    Boxes touching the image's side are left out: cut off, their centre (the bearing) is
    off, more so the nearer the object.
    """
    if ctx.map_origin is None or ctx.now - ctx.detections_time > DETECTION_MAX_AGE_S:
        return []
    return [(d['class_name'], enu(d['lat'], d['lon'], *ctx.map_origin))
            for d in ctx.detections if d['position_ok'] and not d.get('edge')]


def refine(position, class_name, ctx, alpha=0.5, match_m=MATCH_M):
    """Move a map position towards this frame's nearest detection of that class."""
    best = None
    for name, xy in detections_xy(ctx):
        if name == class_name and _dist(xy, position) <= match_m:
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

    def __init__(self, cfg, lookahead_m=DriveConfig().lookahead_m):
        self.cfg = cfg
        self.lookahead_m = lookahead_m
        self.pcfg = self._planner_cfg(lookahead_m)
        self.reset()

    @staticmethod
    def _planner_cfg(lookahead_m):
        return replace(planner.DEFAULT_CONFIG, pure_pursuit_min_lookahead=lookahead_m,
                       pure_pursuit_max_lookahead=lookahead_m)

    def set_lookahead(self, lookahead_m):
        if lookahead_m != self.lookahead_m:
            self.lookahead_m = lookahead_m
            self.pcfg = self._planner_cfg(lookahead_m)

    def reset(self):
        self.prev_error = 0.0
        self.spot_turn = False
        self.spot_dir = 1.0
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
            self.spot_dir = 1.0 if err > 0 else -1.0
        if self.spot_turn:
            # One way until done: an aim point near behind flips err's sign from cycle to
            # cycle, and the boat turned left, right, left on the spot (t2c_s1)
            return cfg.base_pwm, int(self.spot_dir * cfg.spot_turn_pwm)
        taper = max(cfg.pp_throttle_taper_min, math.cos(math.radians(min(abs(err), 90.0))))
        return cfg.base_pwm + offset * taper, steer


class Move:
    """Common bookkeeping: the follower, the cycle time, a result dict."""

    def __init__(self, cfg, drive=None, lookahead_m=None):
        self.cfg = cfg
        self.drive = drive or DriveConfig()
        self.follower = PathFollower(cfg, lookahead_m or self.drive.lookahead_m)
        self.last_t = None
        self.result = {}

    def start(self, ctx):
        self.follower.reset()
        self.last_t = ctx.now
        self.result = {}
        self.bumps = []               # where the bow was held, for the moves that avoid
        self.stuck = 0
        self._back_until = None
        self._progress = None         # [best progress, seconds since it last grew]

    def replan_now(self):
        """Drop the current path (after a back-off); the moves that plan override it."""

    def progress(self, ctx):
        """Return how far on the move is (m, growing as it goes); moves override it."""
        return 0.0

    def _guard(self, ctx, cmd, dt):
        """
        Return (state, command) after the stuck watch.

        No STUCK_M of progress (self.progress) for STUCK_S, whatever was commanded, means
        the hull is held (by a buoy the map has not got, or has in the wrong place) or the
        boat goes round in circles: back off BACK_S straight astern, put a bump
        BUMP_AHEAD_M ahead of where the boat was and plan again. One hold more than
        MAX_STUCK and the move gives up: SKIPPED, result 'stuck'.
        """
        if self._back_until is not None:
            if ctx.now < self._back_until:
                return RUNNING, (BACK_PWM, 0)
            self._back_until, self._progress = None, None
            self.replan_now()
        p = self.progress(ctx)
        if self._progress is None or p >= self._progress[0] + STUCK_M:
            self._progress = [p, 0.0]
        else:
            self._progress[1] += dt * (SPOT_SHARE if self.follower.spot_turn else 1.0)
            if self._progress[1] >= STUCK_S:
                self.stuck += 1
                self.result['stuck'] = self.stuck
                if self.stuck > MAX_STUCK:
                    return SKIPPED, None
                h = math.radians(ctx.heading)
                c = self.here(ctx)
                self.bumps.append((c[0] + BUMP_AHEAD_M * math.sin(h),
                                   c[1] + BUMP_AHEAD_M * math.cos(h)))
                self._back_until = ctx.now + BACK_S
                return RUNNING, (BACK_PWM, 0)
        return RUNNING, cmd

    def here(self, ctx):
        """Return the hull's centre: centre_ahead_m ahead of the antenna (ctx.boat_xy)."""
        h = math.radians(ctx.heading or 0.0)
        k = self.drive.centre_ahead_m
        return (ctx.boat_xy[0] + k * math.sin(h), ctx.boat_xy[1] + k * math.cos(h))

    def _dt(self, ctx):
        dt = 0.04 if self.last_t is None else min(0.5, max(0.001, ctx.now - self.last_t))
        self.last_t = ctx.now
        return dt

    def _follow(self, path, ctx, dt):
        return self.follower.follow(path, self.here(ctx), ctx.heading, ctx.speed, dt)


class DriveTo(Move):

    def __init__(self, cfg, goal, arrive_m=1.5, replan_s=0.2, avoid=True, drive=None,
                 lookahead_m=None):
        super().__init__(cfg, drive, lookahead_m)
        self.straight_lookahead = lookahead_m or self.drive.lookahead_m
        self.goal, self.arrive_m, self.replan_s, self.avoid = goal, arrive_m, replan_s, avoid
        self.grid = LandmarkGrid(cfg.costmap_res_m_per_px, cfg.crop_radius_m,
                                 cfg.buoy_radius_m) if avoid else None
        self.path = None
        self.planned_at = None

    def start(self, ctx):
        super().start(ctx)
        self.path, self.planned_at = None, None

    def replan_now(self):
        self.path = None

    def progress(self, ctx):
        """Return minus the path left (it starts at the boat), else minus the distance."""
        if self.path and len(self.path) >= 2:
            return -(_dist(self.here(ctx), self.path[1])
                     + sum(_dist(a, b) for a, b in zip(self.path[1:], self.path[2:])))
        return -_dist(self.here(ctx), self.goal)

    def step(self, ctx):
        dt = self._dt(ctx)
        if ctx.boat_xy is None:
            return RUNNING, None
        if _dist(self.here(ctx), self.goal) <= self.arrive_m:
            return DONE, None
        if self.path is None or ctx.now - self.planned_at >= self.replan_s:
            self.path = self._plan(ctx)
            self.planned_at = ctx.now
        # A bent (A*) path is followed closely; a straight one smoothly
        self.follower.set_lookahead(self.straight_lookahead if len(self.path) <= 2
                                    else min(self.straight_lookahead,
                                             self.drive.avoid_lookahead_m))
        return self._guard(ctx, self._follow(self.path, ctx, dt), dt)

    def _plan(self, ctx):
        here = self.here(ctx)
        straight = [here, self.goal]
        if not self.avoid:
            return straight
        bumps = [{'kind': 'buoy_black', 'east': b[0], 'north': b[1], 'confirmed': True,
                  'sightings': 1} for b in self.bumps]
        self.grid.rebuild(here, list(ctx.landmarks) + bumps)
        start, goal = to_planner(here), to_planner(self.goal)
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

    def __init__(self, cfg, gate, reverse=False, approach_m=4.0, exit_m=2.0, drive=None,
                 lookahead_m=None, avoid=False, replan_s=0.25, then=(), prefer=None):
        super().__init__(cfg, drive, lookahead_m)
        self.straight_lookahead = lookahead_m or self.drive.lookahead_m
        self.red = (gate.red['east'], gate.red['north'])
        self.green = (gate.green['east'], gate.green['north'])
        self.map_red, self.map_green = self.red, self.green
        self.map_width = _dist(self.red, self.green)
        self.match_m = min(MATCH_M, GATE_MATCH_SHARE * self.map_width)
        self.red_class = 'pole_red' if gate.red['kind'] == 'pole_red' else 'buoy_red'
        self.green_class = 'pole_green' if gate.green['kind'] == 'pole_green' else 'buoy_green'
        self.reverse, self.approach_m, self.exit_m = reverse, approach_m, exit_m
        self.avoid, self.replan_s = avoid, replan_s
        self.then = list(then)        # avoid: where the way goes on after (the next gate)
        self.prefer = prefer          # avoid: the way taken so far (the last gate's path)
        self.path = None
        self.planned_at = None
        self.bent = False

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
        self.planned_at = None
        self.offset = 0.0
        self.along = None

    def replan_now(self):
        self.planned_at = None

    def progress(self, ctx):
        return self.along if self.along is not None else 0.0

    def shift(self):
        """Return how far the camera has moved the gate from the map (the GPS drift)."""
        return ((self.red[0] - self.map_red[0] + self.green[0] - self.map_green[0]) / 2.0,
                (self.red[1] - self.map_red[1] + self.green[1] - self.map_green[1]) / 2.0)

    def step(self, ctx):
        dt = self._dt(ctx)
        if ctx.boat_xy is None:
            return RUNNING, None
        if self.along is None or self.along < -GATE_FREEZE_M:
            self._refine(ctx)
        c, t, left, w = self.geometry()
        here = self.here(ctx)
        d = (here[0] - c[0], here[1] - c[1])
        along = self.along = d[0] * t[0] + d[1] * t[1]
        side = d[0] * left[0] + d[1] * left[1]
        if 'crossed_at' not in self.result and along >= 0.0:
            self.result['crossed_at'] = side       # left of centre at the gate line
            self.result['through'] = abs(side) < w / 2.0
        if along >= self.exit_m:
            return DONE, None
        # Aim through the widest free gap (decided before the line, then kept)
        if along < -0.5:
            gap = free_gap(w, OBSTACLE_RADIUS_M.get(self.red_class, 0.1),
                           OBSTACLE_RADIUS_M.get(self.green_class, 0.1),
                           self._obstacles(ctx, c, t, left, w))
            self.offset = gap[0] if gap else 0.0
            self.result['gap_offset'] = self.offset
        p = (c[0] + left[0] * self.offset, c[1] + left[1] * self.offset)
        self.gap_point = p
        a = (p[0] - t[0] * self.approach_m, p[1] - t[1] * self.approach_m)
        e = (p[0] + t[0] * (self.exit_m + 2.0), p[1] + t[1] * (self.exit_m + 2.0))
        if not self.avoid:
            self.path = [a, p, e]
            if along < -self.approach_m:
                self.path.insert(0, here)
        elif self.planned_at is None or ctx.now - self.planned_at >= self.replan_s:
            self.path = self._plan(ctx, self._waypoints(along, t, a, p, e))
            self.planned_at = ctx.now
        # Slower round a bend: the same turn rate makes a tighter turn
        cruise = int(self.cfg.cruise_pwm * BENT_CRUISE) if self.bent else None
        return self._guard(ctx, self.follower.follow(self.path, here, ctx.heading,
                                                     ctx.speed, dt, cruise), dt)

    def _refine(self, ctx, alpha=0.5):
        """
        Move the gate towards where this frame's camera sees it.

        Both markers seen: the pair (each within MATCH_M) whose width is the map's within
        WIDTH_TOL_M and whose two shifts agree within WIDTH_TOL_M, the smallest shift
        first: the GPS drift moves both alike, a mislabelled box does not. One marker
        seen: within GATE_MATCH_SHARE of the gate's width, and the width still right.
        """
        dets = detections_xy(ctx)
        reds = [xy for n, xy in dets if n == self.red_class and _dist(xy, self.red) <= MATCH_M]
        greens = [xy for n, xy in dets
                  if n == self.green_class and _dist(xy, self.green) <= MATCH_M]
        pairs = []
        for r in reds:
            for g in greens:
                sr = (r[0] - self.red[0], r[1] - self.red[1])
                sg = (g[0] - self.green[0], g[1] - self.green[1])
                if abs(_dist(r, g) - self.map_width) <= WIDTH_TOL_M and \
                        _dist(sr, sg) <= WIDTH_TOL_M:
                    pairs.append((math.hypot(*sr) + math.hypot(*sg), r, g))
        if pairs:
            _, r, g = min(pairs)
            self.result['camera'] = self.result.get('camera', 0) + 1
        else:
            r = min((xy for xy in reds if _dist(xy, self.red) <= self.match_m),
                    key=lambda xy: _dist(xy, self.red), default=None)
            g = min((xy for xy in greens if _dist(xy, self.green) <= self.match_m),
                    key=lambda xy: _dist(xy, self.green), default=None)
            if r is not None and g is not None:
                g = None if _dist(g, self.green) > _dist(r, self.red) else g
                r = None if g is not None else r          # one at a time, the nearer
        if r is None and g is None:
            if reds or greens:
                self.result['refused'] = self.result.get('refused', 0) + 1
            return
        red = self.red if r is None else ((1 - alpha) * self.red[0] + alpha * r[0],
                                          (1 - alpha) * self.red[1] + alpha * r[1])
        green = self.green if g is None else ((1 - alpha) * self.green[0] + alpha * g[0],
                                              (1 - alpha) * self.green[1] + alpha * g[1])
        if abs(_dist(red, green) - self.map_width) > WIDTH_TOL_M:
            self.result['refused'] = self.result.get('refused', 0) + 1
            return
        self.red, self.green = red, green

    def _waypoints(self, along, t, a, p, e):
        """
        Return the points to go through from here: approach, gap, exit, as far as left.

        With `then` (the next gate's approach and centre) the way is planned on to there,
        so an obstacle just past this gate is gone round from before it; the exit is then
        1 m past the line, enough to cross it square.
        """
        if self.then:
            e = (p[0] + t[0], p[1] + t[1])
        if along < -self.approach_m:
            pts = [a, p, e]
        elif along < -0.3:
            # Back onto the line square to the gate, 1 m on, so it is crossed straight
            if along + 1.0 < -0.3:
                q = (p[0] + t[0] * (along + 1.0), p[1] + t[1] * (along + 1.0))
                pts = [q, p, e]
            else:
                pts = [p, e]
        else:
            pts = [e] if along < 1.0 or not self.then else []
        return pts + self.then

    def _plan(self, ctx, waypoints):
        """
        Return the path from the boat through the waypoints round the obstacles.

        A waypoint inside an obstacle's clearance is left out, except the gap itself: the
        approach and exit points are only there to cross the gate square. The last path
        (or the last gate's) is preferred, so the way round an obstacle keeps its side.
        """
        here = self.here(ctx)
        straight = [here] + waypoints
        prefer = self.path if self.path is not None else self.prefer
        for keep_other in (KEEP_OTHER_M, KEEP_GATE_M):
            obstacles = self._avoid_obstacles(ctx, keep_other)
            points = [here] + [w for w in waypoints
                               if w == self.gap_point or not obstacles.inside(w)]
            path = plan_through(points, obstacles, prefer)
            if path is not None and _length(path) > PATH_DETOUR * _length(points) + 2.0:
                path = None                       # round the outside of a wall, or worse
            if path is not None:
                self.bent = len(path) > len(points)
                self.follower.set_lookahead(
                    min(self.straight_lookahead, self.drive.avoid_lookahead_m) if self.bent
                    else self.straight_lookahead)
                if self.bent:
                    self.result['bent'] = self.result.get('bent', 0) + 1
                return path
        self.result['blind'] = self.result.get('blind', 0) + 1
        self.bent = False
        self.follower.set_lookahead(self.straight_lookahead)
        return straight

    def _avoid_obstacles(self, ctx, keep_other):
        """Return the Obstacles to plan round: the gate, its walls, the others, the bumps."""
        r_red = OBSTACLE_RADIUS_M.get(self.red_class, 0.1)
        r_green = OBSTACLE_RADIUS_M.get(self.green_class, 0.1)
        circles = [(self.red, r_red + KEEP_GATE_M), (self.green, r_green + KEEP_GATE_M)]
        for kind, xy in self._others(ctx):
            keep = KEEP_GATE_M if kind in GATE_KINDS else keep_other
            circles.append((xy, OBSTACLE_RADIUS_M.get(kind, 0.25) + keep))
        circles += [(b, BUMP_RADIUS_M + keep_other) for b in self.bumps]
        # Outward from each buoy along the gate's line (not `left`: reversed, it points green)
        gx, gy = self.red[0] - self.green[0], self.red[1] - self.green[1]
        n = math.hypot(gx, gy) or 1.0
        u = (gx / n * WALL_M, gy / n * WALL_M)
        walls = [(self.red, (self.red[0] + u[0], self.red[1] + u[1]), KEEP_GATE_M),
                 (self.green, (self.green[0] - u[0], self.green[1] - u[1]), KEEP_GATE_M)]
        return Obstacles(circles, walls)

    def _others(self, ctx):
        """
        Return [(kind, (east, north))] near the boat other than the gate's own two markers.

        The map's confirmed landmarks only; each is put where this frame's camera sees one
        of its kind within OBJECT_MATCH_M, else moved by the camera's shift of the gate, so
        it sits where this frame's GPS puts it.
        """
        own = (self.red_class, self.green_class)
        seen = [(kind_of(name), xy) for name, xy in detections_xy(ctx)]
        sx, sy = self.shift()
        out = []
        for lm in ctx.landmarks:
            if not lm['confirmed']:
                continue
            kind = kind_of(lm['kind'])            # indicator_red / _green -> indicator
            xy = (lm['east'] + sx, lm['north'] + sy)
            near = [s for k, s in seen if k == kind and _dist(xy, s) <= OBJECT_MATCH_M]
            if near:
                xy = min(near, key=lambda s: _dist(xy, s))
            if _dist(xy, ctx.boat_xy) > AVOID_RANGE_M:
                continue
            if kind in own and min(_dist(xy, self.red), _dist(xy, self.green)) <= MATCH_M:
                continue                          # the gate's own markers (and their copies)
            out.append((kind, xy))
        return out

    def _obstacles(self, ctx, c, t, left, w):
        """Return [(side, radius)] of the other objects on the gate's line (map, camera)."""
        out = []
        for kind, xy in self._others(ctx):
            d = (xy[0] - c[0], xy[1] - c[1])
            along = d[0] * t[0] + d[1] * t[1]
            side = d[0] * left[0] + d[1] * left[1]
            if abs(along) <= GATE_LINE_M and abs(side) <= w / 2.0 + 0.5:
                out.append((side, OBSTACLE_RADIUS_M.get(kind, 0.25)))
        return out


def free_gap(width, red_r, green_r, obstacles):
    """
    Return (offset, gap) of the widest gap for the hull's centre in a gate, or None.

    Positions across the gate: + towards red, the centre at 0, red at width/2, green at
    -width/2. obstacles: [(side, radius)] on the gate's line. The hull's half beam and a
    margin are kept from every edge; offset is the middle of the widest gap left.
    """
    keep = HALF_BEAM_M + GAP_MARGIN_M
    lo, hi = -width / 2.0 + green_r + keep, width / 2.0 - red_r - keep
    gaps = [(lo, hi)] if hi > lo else []
    for side, r in obstacles:
        a, b = side - r - keep, side + r + keep
        nxt = []
        for g0, g1 in gaps:
            if b <= g0 or a >= g1:
                nxt.append((g0, g1))
                continue
            if a > g0:
                nxt.append((g0, a))
            if b < g1:
                nxt.append((b, g1))
        gaps = nxt
    if not gaps:
        return None
    g0, g1 = max(gaps, key=lambda g: g[1] - g[0])
    return (g0 + g1) / 2.0, g1 - g0


class Circle(Move):

    def __init__(self, cfg, target, class_name, radius_m=2.0, clockwise=True, laps=1.0,
                 step_deg=15.0, drive=None):
        drive = drive or DriveConfig()
        super().__init__(cfg, drive, lookahead_m=min(drive.lookahead_m, 0.8 * radius_m))
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
        a = self._angle(self.here(ctx))
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

    def __init__(self, cfg, centre, found, leg_m=4.0, max_m=16.0, spin_deg=360.0,
                 drive=None):
        super().__init__(cfg, drive)
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
            self.path = self._square(self.here(ctx))
        if _dist(self.here(ctx), self.path[-1]) < 1.5:
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
