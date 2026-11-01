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

Task 2 (handbook 3.2.2): through the channel into the debris field, scan it, circle the
green indicator, report every debris, back out through the channel.

  Task2          (R3.1-R3.2 so far) find the channel: on the map already, else go to the
                 team's hint (read in the Task 1 gate frame) and search round it, else search
                 ahead of the gates. Then to channel_approach_m before the entry gate (the end
                 nearer to Task 1) and through the gates one by one, each a PassGate refined
                 from the camera, the gates further on joining the map as the boat comes
                 near; between gates the boat runs on along the channel. The way through a
                 gate is the channel's direction, whatever side red is on. The channel ends
                 when no next gate shows up within channel_end_m past the last one. Memory
                 'task2': entry time and position, gates passed (and whether cleanly)

A gate kept in memory is matched again to the course view (centre within MATCH_GATE_M) so
the newest map position is used; the camera refines it further inside PassGate.
"""

import math

from usv_core.roboboat_course import Gate
from usv_core.roboboat_drive import DriveConfig, DriveTo, PassGate, Search
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

    def __init__(self, cfg, steady_deg_per_s=8.0, min_samples=25, drive=None):
        self.cfg = cfg
        self.drive = drive
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
                self.move = PassGate(self.cfg, gates[0], drive=self.drive)
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
                self.move = PassGate(self.cfg, second[0], drive=self.drive)
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
                               lambda c: c.course is not None and bool(c.course.t1_gates),
                               drive=self.drive)
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
                                arrive_m=1.0, avoid=False, drive=self.drive)
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

    def __init__(self, cfg, approach_m=6.0, drive=None):
        self.cfg = cfg
        self.drive = drive
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
            self.move = DriveTo(self.cfg, goal, arrive_m=2.0, drive=self.drive)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        return state, cmd


class T1Exit:

    def __init__(self, cfg, drive=None):
        self.cfg = cfg
        self.drive = drive

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
                                 reverse=True, drive=self.drive)
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


class Task2:

    def __init__(self, cfg, hint=None, drive=None, channel_approach_m=4.0, channel_end_m=9.0,
                 gate_approach_m=2.5, gate_exit_m=1.5):
        self.cfg = cfg
        self.hint = hint
        self.drive = drive or DriveConfig()
        self.channel_approach_m = channel_approach_m
        self.channel_end_m = channel_end_m
        self.gate_approach_m = gate_approach_m
        self.gate_exit_m = gate_exit_m

    def start(self, ctx):
        self.stage = 'find'
        self.move = None
        self.passed = []              # centres of the gates passed
        self.through = []             # each gate passed cleanly (PassGate's own reading)
        self.direction = None         # unit vector along the channel, entry to exit
        self.channel_in = None

    # ------------------------------------------------------------------ helpers
    def _channel(self, ctx):
        return ctx.course.channel if ctx.course is not None else []

    def _next_gate(self, ctx):
        """Return the nearest channel gate ahead of the last passed one, not yet passed."""
        ref = self.passed[-1] if self.passed else None
        best = None
        for g in self._channel(ctx):
            c = _centre(g)
            if any(math.hypot(c[0] - p[0], c[1] - p[1]) < 1.5 for p in self.passed):
                continue
            if ref is not None:
                d = (c[0] - ref[0], c[1] - ref[1])
                along = d[0] * self.direction[0] + d[1] * self.direction[1]
                if along < 1.0 or along > self.channel_end_m:
                    continue
                dist = along
            else:
                dist = math.hypot(c[0] - ctx.boat_xy[0], c[1] - ctx.boat_xy[1])
            if best is None or dist < best[0]:
                best = (dist, g)
        return best[1] if best else None

    # ------------------------------------------------------------------ stages
    def step(self, ctx):
        if ctx.boat_xy is None:
            return RUNNING, None
        if self.stage == 'find':
            chan = self._channel(ctx)
            if len(chan) >= 2:
                a, b = _centre(chan[0]), _centre(chan[1])
                n = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
                self.direction = ((b[0] - a[0]) / n, (b[1] - a[1]) / n)
                self.stage, self.move = 'approach', None
            else:
                return self._find(ctx)
        if self.stage == 'approach':
            c = _centre(self._channel(ctx)[0]) if self._channel(ctx) else None
            if c is None:
                self.stage = 'find'
                return RUNNING, None
            if not isinstance(self.move, DriveTo):
                goal = (c[0] - self.direction[0] * self.channel_approach_m,
                        c[1] - self.direction[1] * self.channel_approach_m)
                self.move = DriveTo(self.cfg, goal, arrive_m=1.5, drive=self.drive)
                self.move.start(ctx)
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            self.stage, self.move = 'channel', None
            self.channel_in = {'time': ctx.now, 'lat': ctx.lat, 'lon': ctx.lon}
        if self.stage == 'channel':
            return self._channel_step(ctx)
        return SKIPPED, None

    def _find(self, ctx):
        view = ctx.course
        if not isinstance(self.move, (DriveTo, Search)):
            target = view.expected(self.hint) if view is not None else None
            if target is not None:
                self.move = DriveTo(self.cfg, target, arrive_m=3.0, drive=self.drive)
            else:
                centre = view.frame.to_map(15.0, 0.0) if view is not None and view.frame \
                    else ctx.boat_xy
                self.move = Search(self.cfg, centre, lambda c: len(self._channel(c)) >= 2,
                                   drive=self.drive)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        if state == DONE and isinstance(self.move, DriveTo):
            self.move = Search(self.cfg, self.move.goal, lambda c: len(self._channel(c)) >= 2,
                               drive=self.drive)
            self.move.start(ctx)
            return RUNNING, None
        if state == SKIPPED:
            return SKIPPED, None
        return RUNNING, cmd if state == RUNNING else None

    def _channel_step(self, ctx):
        if isinstance(self.move, PassGate):
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            c, _, _, _ = self.move.geometry()
            self.passed.append(c)
            self.through.append(bool(self.move.result.get('through')))
            self.move = None
        gate = self._next_gate(ctx)
        if gate is not None:
            reverse = (gate.through[0] * self.direction[0]
                       + gate.through[1] * self.direction[1]) < 0
            self.move = PassGate(self.cfg, gate, reverse=reverse,
                                 approach_m=self.gate_approach_m, exit_m=self.gate_exit_m,
                                 drive=self.drive, lookahead_m=self.drive.channel_lookahead_m)
            self.move.start(ctx)
            return RUNNING, self.move.step(ctx)[1]
        # No next gate on the map: run on along the channel; past channel_end_m it has ended
        last = self.passed[-1] if self.passed else ctx.boat_xy
        d = (ctx.boat_xy[0] - last[0], ctx.boat_xy[1] - last[1])
        if self.passed and d[0] * self.direction[0] + d[1] * self.direction[1] \
                > self.channel_end_m:
            ctx.memory['task2'] = {'channel_in': self.channel_in, 'gates': len(self.passed),
                                   'through': self.through,
                                   'channel_out_xy': ctx.boat_xy,
                                   'direction': self.direction}
            return DONE, None
        if not isinstance(self.move, DriveTo):
            goal = (last[0] + self.direction[0] * (self.channel_end_m + 2.0),
                    last[1] + self.direction[1] * (self.channel_end_m + 2.0))
            self.move = DriveTo(self.cfg, goal, arrive_m=1.0, avoid=False, drive=self.drive,
                                lookahead_m=self.drive.channel_lookahead_m)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        return RUNNING, cmd if state == RUNNING else None


def default_phases(cfg, drive=None, hints=None):
    """Return the phases written so far, by run phase name; the rest stay Pending."""
    hints = hints or {}
    return {'T1_ENTRY': T1Entry(cfg, drive=drive), 'RETURN': ReturnToGates(cfg, drive=drive),
            'T1_EXIT': T1Exit(cfg, drive=drive),
            'task2': Task2(cfg, hint=hints.get('task2'), drive=drive)}
