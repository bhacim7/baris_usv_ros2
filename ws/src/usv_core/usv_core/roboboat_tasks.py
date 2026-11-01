"""
The RoboBoat run's phases, made of the moves in usv_core.roboboat_drive (phase 6 R2.4).

Each phase has start(ctx) / step(ctx) -> (state, command), as usv_core.roboboat_run
expects; what later phases and the reports (R4) need goes to ctx.memory.

Task 1 (handbook 3.2.1, 3.5): the run enters through the two Task 1 gates and leaves
through the same two, the other way.

  T1Entry        through the first gate, then the second, each with PassGate (camera
                 refined). The second gate is 8-16 m from the start, past the map's 12 m
                 range: if the map has not got it yet, the boat goes on 6 m along the way
                 through the first and takes it from there. While running straight between
                 the gates the heading is sampled: its mean is the gate frame's direction
                 (memory 't1_heading'), from the compass, free of the GPS drift that tilts a
                 frame made of mapped gates. Memory: 't1_gates' (where the poles were),
                 't1_entry' (time, lat, lon at the second gate: the entry report)
  ReturnToGates  DriveTo (A* round the map's landmarks) to approach_m past the second gate
                 on the course side, facing nothing in particular
  T1Exit         PassGate through the second gate, then the first, both reversed (red on
                 the right). Memory: 't1_exit' (time, lat, lon at the first gate)

A gate kept in memory is matched again to the course view (centre within MATCH_GATE_M) so
the newest map position is used; the camera refines it further inside PassGate.
"""

import math

from usv_core.roboboat_course import Gate
from usv_core.roboboat_drive import DriveTo, PassGate, Search
from usv_core.roboboat_run import DONE, RUNNING, SKIPPED

MATCH_GATE_M = 3.0
PHASE_LIMIT_S = 240.0         # RETURN and T1_EXIT are not cut by the run's task timeout


def _centre(g):
    return ((g.red['east'] + g.green['east']) / 2.0, (g.red['north'] + g.green['north']) / 2.0)


def _gate_dict(g):
    return {'red': (g.red['east'], g.red['north']), 'green': (g.green['east'], g.green['north']),
            'kind': g.red['kind']}


def _gate_from(stored, ctx):
    """Return the course view's gate at a stored gate's place, else the stored one."""
    c = ((stored['red'][0] + stored['green'][0]) / 2.0,
         (stored['red'][1] + stored['green'][1]) / 2.0)
    view = ctx.course
    for g in (view.t1_gates if view is not None else []):
        if math.hypot(_centre(g)[0] - c[0], _centre(g)[1] - c[1]) <= MATCH_GATE_M:
            return g
    green_kind = stored['kind'].replace('red', 'green')
    return Gate({'kind': stored['kind'], 'east': stored['red'][0], 'north': stored['red'][1]},
                {'kind': green_kind, 'east': stored['green'][0],
                 'north': stored['green'][1]})


def circular_mean_deg(values):
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c)) % 360.0


class T1Entry:

    def __init__(self, cfg, steady_deg_per_s=8.0, min_samples=25):
        self.cfg = cfg
        self.steady = steady_deg_per_s
        self.min_samples = min_samples

    def start(self, ctx):
        self.stage = 'first'
        self.move = None
        self.headings = []
        self.last = None              # (time, heading) for the turn rate

    def step(self, ctx):
        gates = ctx.course.t1_gates if ctx.course is not None else []
        if self.stage == 'first':
            if not gates:
                return self._search(ctx)
            if not isinstance(self.move, PassGate):
                self.first = _gate_dict(gates[0])
                self.move = PassGate(self.cfg, gates[0])
                self.move.start(ctx)
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            self.first = {'red': self.move.red, 'green': self.move.green,
                          'kind': self.first['kind']}
            self.stage, self.move = 'second', None
        if self.stage == 'second':
            self._sample(ctx)
            second = [g for g in gates if math.hypot(
                _centre(g)[0] - (self.first['red'][0] + self.first['green'][0]) / 2.0,
                _centre(g)[1] - (self.first['red'][1] + self.first['green'][1]) / 2.0)
                > MATCH_GATE_M]
            if not second:
                return self._go_on(ctx)
            if not isinstance(self.move, PassGate):
                self.move = PassGate(self.cfg, second[0])
                self.move.start(ctx)
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            self._finish(ctx)
            return DONE, None
        return SKIPPED, None

    def _search(self, ctx):
        if not isinstance(self.move, Search):
            h = math.radians(ctx.heading)
            ahead = (ctx.boat_xy[0] + 8.0 * math.sin(h), ctx.boat_xy[1] + 8.0 * math.cos(h)) \
                if ctx.boat_xy else (0.0, 0.0)
            self.move = Search(self.cfg, ahead,
                               lambda c: c.course is not None and bool(c.course.t1_gates))
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        if state == SKIPPED:
            return SKIPPED, None
        return RUNNING, cmd if state == RUNNING else None

    def _go_on(self, ctx):
        """Run on through the first gate until the map has the second one."""
        if not isinstance(self.move, DriveTo):
            g = _gate_from(self.first, ctx)
            pg = PassGate(self.cfg, g)
            c, t, _, _ = pg.geometry()
            self.move = DriveTo(self.cfg, (c[0] + 8.0 * t[0], c[1] + 8.0 * t[1]),
                                arrive_m=1.0, avoid=False)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        return RUNNING, cmd if state == RUNNING else None

    def _sample(self, ctx):
        if self.last is not None and ctx.now > self.last[0]:
            rate = abs((ctx.heading - self.last[1] + 180.0) % 360.0 - 180.0) / \
                (ctx.now - self.last[0])
            if rate <= self.steady:
                self.headings.append(ctx.heading)
        self.last = (ctx.now, ctx.heading)

    def _finish(self, ctx):
        mem = ctx.memory
        mem['t1_gates'] = [self.first, {'red': self.move.red, 'green': self.move.green,
                                        'kind': self.first['kind']}]
        mem['t1_entry'] = {'time': ctx.now, 'lat': ctx.lat, 'lon': ctx.lon,
                           'through': self.move.result.get('through')}
        if len(self.headings) >= self.min_samples:
            mem['t1_heading'] = circular_mean_deg(self.headings)


class ReturnToGates:

    def __init__(self, cfg, approach_m=6.0):
        self.cfg = cfg
        self.approach_m = approach_m

    def start(self, ctx):
        self.move = None
        self.started = ctx.now

    def step(self, ctx):
        stored = ctx.memory.get('t1_gates')
        if not stored or ctx.now - self.started > PHASE_LIMIT_S:
            return SKIPPED, None
        if self.move is None:
            second = _gate_from(stored[1], ctx)
            c, t, _, _ = PassGate(self.cfg, second).geometry()   # the entry direction
            goal = (c[0] + self.approach_m * t[0], c[1] + self.approach_m * t[1])
            self.move = DriveTo(self.cfg, goal, arrive_m=2.0)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        return state, cmd


class T1Exit:

    def __init__(self, cfg):
        self.cfg = cfg

    def start(self, ctx):
        self.order = [1, 0]           # second gate first, the other way
        self.move = None
        self.started = ctx.now
        self.through = []

    def step(self, ctx):
        stored = ctx.memory.get('t1_gates')
        if not stored or ctx.now - self.started > PHASE_LIMIT_S:
            return SKIPPED, None
        if self.move is None:
            self.move = PassGate(self.cfg, _gate_from(stored[self.order[0]], ctx),
                                 reverse=True)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        if state == RUNNING:
            return RUNNING, cmd
        self.through.append(self.move.result.get('through'))
        self.order.pop(0)
        self.move = None
        if self.order:
            return RUNNING, None
        ctx.memory['t1_exit'] = {'time': ctx.now, 'lat': ctx.lat, 'lon': ctx.lon,
                                 'through': all(self.through)}
        return DONE, None


def default_phases(cfg):
    """Return the phases written so far, by run phase name; the rest stay Pending."""
    return {'T1_ENTRY': T1Entry(cfg), 'RETURN': ReturnToGates(cfg), 'T1_EXIT': T1Exit(cfg)}
