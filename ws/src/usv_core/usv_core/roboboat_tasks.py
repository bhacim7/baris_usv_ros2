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

  Task2          (R3.1-R3.2, R3.4 so far) find the channel: on the map already, else go to
                 the team's hint (read in the Task 1 gate frame) and search round it, else
                 search ahead of the gates. Then to channel_approach_m before the entry gate
                 (the end nearer to Task 1) and through the gates one by one ('in'), each a
                 PassGate refined from the camera and planned round every obstacle near it
                 (avoid: A* inside walls along the gate's line), the gates further on joining
                 the map as the boat comes near; between gates the boat runs on along the
                 channel. The way through a gate is the channel's direction, whatever side
                 red is on. The channel ends when no next gate (within channel_end_m) is on
                 the map once the boat is run_on_m past the last one. Then the debris field
                 ('debris', DebrisField). Then back through the same gates the other way
                 ('out', handbook: the ASV returns through the channel), ending back_end_m
                 past the entry gate. A gate the
                 boat stays stuck at is left (not between the buoys) and the next one taken.
                 Memory 'task2': entry time and position, gates passed each way (and whether
                 cleanly), back-offs; 'task2_debris': DebrisField.report
  DebrisField    (R3.3) past the channel's last gate. The green indicator first: once it is
                 on the map (confirmed), Circle round it, radius_m, counter-clockwise,
                 planned round the other buoys. Until then a scan: DriveTo (avoiding) along
                 a zigzag in the channel's frame, depth_m ahead of the last gate and
                 half_width_m to each side. The red indicator is never aimed at (an obstacle
                 like the rest, 1 m clear) and noted when seen. The scan stops when nothing
                 new has been confirmed for quiet_s of scanning, or after limit_s of
                 scanning, or when the zigzag is done; and once the green is circled and the
                 red seen. Then back to the channel from wherever the boat is: the turn is
                 made in the field, not beside its first buoy (t2h_s2 brushed it)

A gate kept in memory is matched again to the course view (centre within MATCH_GATE_M) so
the newest map position is used; the camera refines it further inside PassGate.
"""

from dataclasses import dataclass
import math

from usv_core.roboboat_course import Gate, GateFrame
from usv_core.roboboat_drive import Circle, DriveConfig, DriveTo, PassGate, Search
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
                                arrive_m=1.0, drive=self.drive)
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
        # Where the run started, for good: 8 m before the first gate on the gates' line (the
        # course model tells the first gate by it; a start taken late would flip the frame)
        c1 = ((self.first['red'][0] + self.first['green'][0]) / 2.0,
              (self.first['red'][1] + self.first['green'][1]) / 2.0)
        g2 = mem['t1_gates'][1]
        c2 = ((g2['red'][0] + g2['green'][0]) / 2.0, (g2['red'][1] + g2['green'][1]) / 2.0)
        n = math.hypot(c2[0] - c1[0], c2[1] - c1[1]) or 1.0
        mem['start_xy'] = (c1[0] - 8.0 * (c2[0] - c1[0]) / n, c1[1] - 8.0 * (c2[1] - c1[1]) / n)

    def summary(self, ctx):
        h = ctx.memory.get('t1_heading')
        return (f'gate frame from the compass {h:.1f} deg, {len(self.headings)} samples'
                if h is not None else
                f'gate frame from the mapped gates (only {len(self.headings)} compass samples)')


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


@dataclass(frozen=True)
class DebrisConfig:
    """Task 2 debris field settings (navigator.yaml roboboat.debris.*), decided 2026-11-01."""

    depth_m: float = 15.0          # scanned this far past the channel's last gate
    half_width_m: float = 7.0      # and this far to each side of the channel's line
    quiet_s: float = 30.0          # scanning stops after this long with nothing new
    limit_s: float = 150.0         # or after this long scanning in all
    green_radius_m: float = 2.5    # the green indicator is circled at this radius
    green_clockwise: bool = False


def _debris(ctx):
    return [lm for lm in (ctx.course.debris if ctx.course is not None else [])
            if lm['confirmed']]


def _xy(lm):
    return (lm['east'], lm['north'])


def _place(lm):
    return {'kind': lm['kind'], 'color': lm.get('color', ''), 'east': lm['east'],
            'north': lm['north'], 'lat': lm.get('lat'), 'lon': lm.get('lon')}


class DebrisField:

    def __init__(self, cfg, drive=None, debris=None):
        self.cfg = cfg
        self.drive = drive or DriveConfig()
        self.dcfg = debris or DebrisConfig()

    def start(self, ctx, exit_xy, direction):
        """Start from the channel's last gate (exit_xy), direction along the channel."""
        d = self.dcfg
        frame = GateFrame(exit_xy, direction)
        near, far, w = 0.4 * d.depth_m, 0.9 * d.depth_m, d.half_width_m
        self.legs = [frame.to_map(a, side) for a, side in
                     ((near, -w), (near, w), (far, w), (far, -w))]
        self.move = None
        self.known = {lm['id'] for lm in _debris(ctx)}
        self.scan_s = 0.0             # time spent scanning (circling not counted)
        self.last_new_s = 0.0
        self.last_t = ctx.now
        self.green = self.red = None  # landmark dicts
        self.circle = None            # {'done', 'swept_deg'} once circled
        self.stopped = ''

    def _look(self, ctx):
        """Note new confirmed debris and the indicators; return True if any is new."""
        new = False
        for lm in _debris(ctx):
            if lm['id'] not in self.known:
                self.known.add(lm['id'])
                new = True
            if lm['kind'] == 'indicator' and lm.get('color') == 'green':
                self.green = lm
            elif lm['kind'] == 'indicator' and lm.get('color') == 'red':
                self.red = lm
        return new

    def step(self, ctx):
        dt = max(0.0, ctx.now - self.last_t)
        self.last_t = ctx.now
        if self._look(ctx):
            self.last_new_s = self.scan_s
        if self.green is not None and self.circle is None:
            return self._circle(ctx)
        if self.circle is not None and self.red is not None:
            self.stopped = 'green circled, red seen'
            return DONE, None
        d = self.dcfg
        if self.scan_s >= d.limit_s:
            self.stopped = f'scan limit {d.limit_s:.0f} s'
        elif self.scan_s - self.last_new_s >= d.quiet_s:
            self.stopped = f'nothing new for {d.quiet_s:.0f} s'
        elif not self.legs:
            self.stopped = 'zigzag done'
        if self.stopped:
            return DONE, None
        self.scan_s += dt
        if not isinstance(self.move, DriveTo):
            self.move = DriveTo(self.cfg, self.legs[0], arrive_m=2.0, drive=self.drive)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        if state != RUNNING:                      # there, or stuck: the next leg
            self.legs.pop(0)
            self.move = None
            return RUNNING, None
        return RUNNING, cmd

    def _circle(self, ctx):
        d = self.dcfg
        if not isinstance(self.move, Circle):
            self.move = Circle(self.cfg, _xy(self.green), 'indicator_green',
                               radius_m=d.green_radius_m, clockwise=d.green_clockwise,
                               drive=self.drive, avoid=True)
            self.move.start(ctx)
        state, cmd = self.move.step(ctx)
        if state == RUNNING:
            return RUNNING, cmd
        self.circle = {'done': state == DONE, 'swept_deg': self.move.result.get('swept_deg', 0.0),
                       'bulged': self.move.result.get('bulged', 0)}
        self.move = None
        self.last_new_s = self.scan_s             # the scan, if it goes on, starts afresh
        return RUNNING, None

    def report(self, ctx):
        """Return what the field held (the map's latest places) and what was done."""
        latest = {lm['id']: lm for lm in _debris(ctx)}
        green = red = None
        if self.green is not None:
            green = _place(latest.get(self.green['id'], self.green))
            green.update(self.circle or {'done': False})
        if self.red is not None:
            red = _place(latest.get(self.red['id'], self.red))
        return {'debris': [_place(lm) for lm in latest.values()], 'green': green, 'red': red,
                'scan_s': self.scan_s, 'stopped': self.stopped}

    def summary(self):
        if self.green is None:
            g = 'green not found'
        elif self.circle and self.circle['done']:
            g = f'green circled {self.circle["swept_deg"]:.0f} deg'
        else:
            g = 'green found, not circled'
        r = 'red seen' if self.red is not None else 'red not seen'
        return f'{len(self.known)} debris, {g}, {r}, scan {self.scan_s:.0f} s ({self.stopped})'


class Task2:

    def __init__(self, cfg, hint=None, drive=None, channel_approach_m=4.0, channel_end_m=9.0,
                 gate_approach_m=2.5, gate_exit_m=1.5, back_end_m=4.0, debris=None,
                 run_on_m=3.0):
        self.cfg = cfg
        self.field = DebrisField(cfg, drive, debris)
        self.hint = hint
        self.drive = drive or DriveConfig()
        self.channel_approach_m = channel_approach_m
        self.channel_end_m = channel_end_m
        self.gate_approach_m = gate_approach_m
        self.gate_exit_m = gate_exit_m
        self.back_end_m = back_end_m
        # Past the last gate, run on this far before the channel counts as ended: the map
        # sees 12 m ahead, so a next gate (6 m on) would be on it already. It was
        # channel_end_m, 9 m, and the boat ran on past the green it had already seen
        self.run_on_m = run_on_m

    def start(self, ctx):
        self.stage = 'find'
        self.move = None
        self.direction = None         # unit vector along the channel, entry to exit
        self.channel_in = None
        self.channel_out_xy = None
        # Each way through the channel: centres of the gates passed, each passed cleanly
        # (PassGate's own reading), back-offs from being stuck
        self.legs = {way: {'passed': [], 'through': [], 'stuck': 0} for way in ('in', 'out')}

    # ------------------------------------------------------------------ helpers
    def _channel(self, ctx):
        return ctx.course.channel if ctx.course is not None else []

    def _way(self, name):
        d = self.direction
        return d if name == 'in' else (-d[0], -d[1])

    def _next_gate(self, ctx, passed, direction):
        """Return the nearest channel gate ahead of the last passed one, not yet passed."""
        ref = passed[-1] if passed else None
        best = None
        for g in self._channel(ctx):
            c = _centre(g)
            if any(math.hypot(c[0] - p[0], c[1] - p[1]) < 1.5 for p in passed):
                continue
            if ref is not None:
                d = (c[0] - ref[0], c[1] - ref[1])
                along = d[0] * direction[0] + d[1] * direction[1]
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
            self.stage, self.move = 'in', None
            self.channel_in = {'time': ctx.now, 'lat': ctx.lat, 'lon': ctx.lon}
        if self.stage in ('in', 'out'):
            return self._channel_step(ctx, self.stage)
        if self.stage == 'debris':
            state, cmd = self.field.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            ctx.memory['task2_debris'] = self.field.report(ctx)
            self.stage, self.move = 'out', None
            return RUNNING, None
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
        if state != RUNNING and isinstance(self.move, DriveTo):     # there, or stuck
            self.move = Search(self.cfg, self.move.goal, lambda c: len(self._channel(c)) >= 2,
                               drive=self.drive)
            self.move.start(ctx)
            return RUNNING, None
        if state == SKIPPED:
            return SKIPPED, None
        return RUNNING, cmd if state == RUNNING else None

    def summary(self, ctx):
        legs = self.legs
        if not legs['in']['passed']:
            return f'channel not reached (stage {self.stage})'
        text = ', '.join(f'{way} {len(legs[way]["passed"])} gates '
                         f'({sum(legs[way]["through"])} between the buoys'
                         + (f', {legs[way]["stuck"]} back-offs' if legs[way]['stuck'] else '')
                         + ')' for way in ('in', 'out') if legs[way]['passed'])
        if self.stage in ('debris', 'out') or 'task2_debris' in ctx.memory:
            text += f'; debris field: {self.field.summary()}'
        return f'channel {text}'

    def _channel_step(self, ctx, way):
        """Go through the channel one way, gate by gate; past its end, turn back or finish."""
        leg, direction = self.legs[way], self._way(way)
        prefer = None
        if isinstance(self.move, PassGate):
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            prefer = self.move.path
            c, _, _, _ = self.move.geometry()
            leg['passed'].append(c)
            leg['through'].append(state == DONE and bool(self.move.result.get('through')))
            leg['stuck'] += self.move.result.get('stuck', 0)
            self.move = None
        gate = self._next_gate(ctx, leg['passed'], direction)
        if gate is not None:
            reverse = (gate.through[0] * direction[0] + gate.through[1] * direction[1]) < 0
            # The way is planned on to the gate after it, if the map has that one yet
            after = self._next_gate(ctx, leg['passed'] + [_centre(gate)], direction)
            then = []
            if after is not None:
                c2 = _centre(after)
                then = [(c2[0] - direction[0] * self.gate_approach_m,
                         c2[1] - direction[1] * self.gate_approach_m), c2]
            self.move = PassGate(self.cfg, gate, reverse=reverse,
                                 approach_m=self.gate_approach_m, exit_m=self.gate_exit_m,
                                 drive=self.drive, lookahead_m=self.drive.channel_lookahead_m,
                                 avoid=True, then=then, prefer=prefer)
            self.move.start(ctx)
            return RUNNING, self.move.step(ctx)[1]
        # No next gate on the map: run on along the channel; past end_m it has ended
        end_m = self.run_on_m if way == 'in' else self.back_end_m
        last = leg['passed'][-1] if leg['passed'] else ctx.boat_xy
        d = (ctx.boat_xy[0] - last[0], ctx.boat_xy[1] - last[1])
        ended = bool(leg['passed']) and d[0] * direction[0] + d[1] * direction[1] > end_m
        if not ended:
            if not isinstance(self.move, DriveTo):
                goal = (last[0] + direction[0] * (end_m + 2.0),
                        last[1] + direction[1] * (end_m + 2.0))
                # round obstacles: past the channel the debris field starts (t2f_s1 ran
                # into its first buoy here, straight)
                self.move = DriveTo(self.cfg, goal, arrive_m=1.0, drive=self.drive,
                                    lookahead_m=self.drive.channel_lookahead_m)
                self.move.start(ctx)
            state, cmd = self.move.step(ctx)
            if state == RUNNING:
                return RUNNING, cmd
            ended = bool(leg['passed'])               # there, or stuck on the way
            if not ended:
                return RUNNING, None
        self.move = None
        if way == 'in':
            # The debris field, then back out through the channel
            self.channel_out_xy = ctx.boat_xy
            self.stage = 'debris'
            self.field.start(ctx, leg['passed'][-1], direction)
            return RUNNING, None
        ctx.memory['task2'] = {'channel_in': self.channel_in,
                               'gates': len(self.legs['in']['passed']),
                               'through': self.legs['in']['through'],
                               'back_gates': len(leg['passed']), 'back_through': leg['through'],
                               'stuck': self.legs['in']['stuck'] + leg['stuck'],
                               'channel_out_xy': self.channel_out_xy,
                               'direction': self.direction}
        return DONE, None


def default_phases(cfg, drive=None, hints=None, debris=None):
    """Return the phases written so far, by run phase name; the rest stay Pending."""
    hints = hints or {}
    return {'T1_ENTRY': T1Entry(cfg, drive=drive), 'RETURN': ReturnToGates(cfg, drive=drive),
            'T1_EXIT': T1Exit(cfg, drive=drive),
            'task2': Task2(cfg, hint=hints.get('task2'), drive=drive, debris=debris)}
