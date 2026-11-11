"""
The first tasks of the 2027 mission manager (step 6c): rally, hold, home.

  rally  go to a point and keep station there (2027 Task 5 Core: "rally at the GPS point
         RoboCommand gives"). Parameters lat, lon (degrees); hold_s (optional, s): DONE
         after holding that long, else it holds until the manager moves on (the next
         request, the task's or the run's timeout)
  hold   keep station where the boat is when the task starts
  home   back to where the run started (memory start_xy) and keep station there: what
         the boat does when nothing is left or time is up (decided 2026-11-11)

Keeping station (Hold): the boat has no brakes and drifts, so it is a band, not a point:
inside hold_m the thrusters are neutral; drifted out past leave_m it drives back (DriveTo,
round obstacles on the map) to within hold_m. ASSUMED 2.0 / 4.0 m, to be measured.
Every task here notes what happened in `result` (arrived, times driven back), for the
RoboCommand reports (step 6d). ROS-free, unit tested (test/test_mission_tasks.py).
"""

import math

from usv_core.roboboat_drive import DriveTo
from usv_core.roboboat_run import DONE, RUNNING, SKIPPED
from usv_core.world_map import enu

HOLD_M = 2.0
LEAVE_M = 4.0


class Hold:
    """Keep station at a point (map east / north metres), or where the boat is if None."""

    def __init__(self, cfg, point=None, hold_s=0.0, hold_m=HOLD_M, leave_m=LEAVE_M,
                 drive=None):
        self.cfg, self.drive_cfg = cfg, drive
        self.point, self.hold_s, self.hold_m, self.leave_m = point, hold_s, hold_m, leave_m
        self.move = None
        self.arrived_at = None
        self.result = {}

    def start(self, ctx):
        if self.point is None and ctx.boat_xy is not None:
            self.point = ctx.boat_xy
        self.move, self.arrived_at = None, None
        self.result = {'returns': 0}

    def step(self, ctx):
        if ctx.boat_xy is None or self.point is None:
            return RUNNING, None              # no map yet: wait, neutral
        d = math.dist(ctx.boat_xy, self.point)
        if self.move is None and (self.arrived_at is None or d > self.leave_m):
            if self.arrived_at is not None:
                self.result['returns'] += 1
            self.move = DriveTo(self.cfg, self.point, arrive_m=self.hold_m,
                                drive=self.drive_cfg)
            self.move.start(ctx)
        if self.move is not None:
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            self.move = None
            if self.arrived_at is None:
                self.arrived_at = ctx.now
                self.result['arrived_at'] = ctx.now
        if self.hold_s > 0 and ctx.now - self.arrived_at >= self.hold_s:
            return DONE, None
        return RUNNING, None                  # in the band: neutral, drift

    def summary(self, ctx):
        if self.arrived_at is None:
            return 'not there yet'
        returns = self.result.get('returns', 0)
        return f'held {ctx.now - self.arrived_at:.0f} s, driven back {returns} times'


class Rally(Hold):
    """Go to a latitude / longitude and keep station there (Task 5 Core)."""

    def __init__(self, cfg, lat, lon, hold_s=0.0, drive=None):
        super().__init__(cfg, None, hold_s, drive=drive)
        self.lat, self.lon = float(lat), float(lon)

    def start(self, ctx):
        super().start(ctx)
        self.point = None
        self._place(ctx)

    def _place(self, ctx):
        if self.point is None and ctx.map_origin is not None:
            self.point = enu(self.lat, self.lon, *ctx.map_origin)

    def step(self, ctx):
        self._place(ctx)
        if self.point is None:
            return RUNNING, None              # no map origin yet: wait, neutral
        return super().step(ctx)


class Home(Hold):
    """Back to where the run started (memory start_xy) and keep station there."""

    def start(self, ctx):
        super().start(ctx)
        self.point = ctx.memory.get('start_xy') or ctx.boat_xy


class Refused:
    """A task whose parameters are wrong: skipped at once, saying why."""

    def __init__(self, why):
        self.why = why

    def start(self, ctx):
        pass

    def step(self, ctx):
        return SKIPPED, None

    def summary(self, ctx):
        return self.why


def factories(cfg, drive=None):
    """Return {name: factory(params)} for rally and hold, and the home factory."""
    def rally(params):
        try:
            lat, lon = float(params['lat']), float(params['lon'])
        except (KeyError, TypeError, ValueError):
            return Refused('rally needs lat and lon')
        if not (math.isfinite(lat) and math.isfinite(lon)):
            return Refused('rally needs lat and lon')
        return Rally(cfg, lat, lon, float(params.get('hold_s', 0.0)), drive)

    def hold(params):
        return Hold(cfg, None, float(params.get('hold_s', 0.0)), drive=drive)

    def home(params):
        return Home(cfg, drive=drive)
    return {'rally': rally, 'hold': hold}, home
