"""
The RoboBoat course as the world map shows it (phase 6 R2.2).

No GPS points come from the GCS (decision 2026-11-01): the course elements are recognised
on the world map (usv_core.world_map landmarks, /usv/map) from their classes and geometry,
with the handbook's sizes (2026, section 3.2) as the only prior:

  Task 1 gates   a red and a green Sur-Mark pole (pole_red / pole_green) a gate's width
                 apart; poles appear nowhere else. Two gates one behind the other; the
                 first is the one nearer to where the run started (the boat is put down at
                 least 6 ft before it, facing it). Red is on the port side going in.
  gate frame     origin at the first gate's centre, `ahead` towards the second gate (or
                 through the only gate seen), `left` 90 deg counter-clockwise. The hints the
                 team gives before a run ("ahead_m,left_m") are read in this frame; it is
                 made of the gates themselves, so it carries no GPS offset of its own
  Task 2 channel small red / green buoy pairs (buoy_red / buoy_green, 6-10 ft apart) one
                 after the other; the longest such chain. Its far end leads into the debris
                 field: the black buoys and indicators beyond it
  Task 3         a red / green buoy pair that is not part of the channel, with a yellow
                 buoy 40-100 ft beyond it; the indicator between them gives the direction
                 round the yellow buoy (red: counter-clockwise, green: clockwise)
  marina         the slip number banners (digit_1..3); for each, the indicator next to it
                 and whether a vessel lies in the slip. The best slip: green indicator, no
                 vessel, lowest number (handbook 3.2.5), over ALL six slips: given only
                 once the marina is complete (best_slip_so_far before; 'marina_incomplete'
                 missing). Banners closer than dock_link_m belong to one dock; with two
                 docks seen, the northern one (by compass) is N. With one dock seen its
                 name is not known yet ('', 'marina_second_dock' missing)
  Task 6 zones   the yellow buoy of Task 3 and the yellow buoy nearest the marina

Only confirmed landmarks are used unless use_unconfirmed. Everything works in the map's
local east / north metres (landmark 'east' / 'north'); what is not found yet is listed in
CourseView.missing, so the run knows what still has to be looked for.

ROS-free; test/test_roboboat_course.py, and ws/tools/course_report.py on recorded maps.
"""

from dataclasses import dataclass, field
import math

from usv_core.classes import DIGIT_NUMBER


@dataclass(frozen=True)
class CourseConfig:
    t1_gate_width_m: tuple = (3.0, 14.0)      # ASSUMED: not in the handbook; 8.7 m placed
    t1_gate_spacing_m: tuple = (3.0, 20.0)    # between the two Task 1 gates
    buoy_gate_width_m: tuple = (1.2, 4.0)     # 6-10 ft, plus map error
    buoy_gate_nominal_m: float = 2.5
    channel_step_m: tuple = (2.5, 12.0)       # gate to gate along the channel
    channel_turn_deg: float = 50.0            # largest bend between channel gates
    task3_yellow_m: tuple = (8.0, 40.0)       # 40-100 ft past the gate, plus map error
    task3_line_m: float = 4.0                 # indicator / yellow off the gate's axis
    debris_range_m: float = 30.0              # past the channel's far end
    # Dock geometry (handbook: 60 in slips, 20 in cubes -> banners 2.0 m apart): an
    # indicator sits ~0.5 m from its banner, a moored vessel's centre ~1 m in front of it
    # and ~2 m from the next banner; each goes to its NEAREST banner within these
    slip_indicator_m: float = 1.5
    slip_vessel_m: float = 1.8
    dock_link_m: float = 4.5                  # banners of one dock: 2 m apart, 3 in a row
    use_unconfirmed: bool = False


# ---------------------------------------------------------------- geometry
def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _norm(a):
    return math.hypot(a[0], a[1])


def _unit(a):
    n = _norm(a) or 1.0
    return (a[0] / n, a[1] / n)


def _pos(lm):
    return (lm['east'], lm['north'])


@dataclass
class Gate:
    red: dict
    green: dict

    @property
    def centre(self):
        r, g = _pos(self.red), _pos(self.green)
        return ((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0)

    @property
    def width(self):
        return _norm(_sub(_pos(self.red), _pos(self.green)))

    @property
    def through(self):
        """Return the unit vector of the way through, red on the left (port)."""
        left = _unit(_sub(_pos(self.red), _pos(self.green)))
        return (left[1], -left[0])

    @property
    def ids(self):
        return (self.red['id'], self.green['id'])


@dataclass
class GateFrame:
    """Task 1 gate frame: origin at the first gate's centre, ahead towards the second."""

    origin: tuple
    ahead: tuple

    @property
    def left(self):
        return (-self.ahead[1], self.ahead[0])

    def to_map(self, ahead_m, left_m):
        a, lft = self.ahead, self.left
        return (self.origin[0] + ahead_m * a[0] + left_m * lft[0],
                self.origin[1] + ahead_m * a[1] + left_m * lft[1])

    def from_map(self, east, north):
        d = (east - self.origin[0], north - self.origin[1])
        return (_dot(d, self.ahead), _dot(d, self.left))


@dataclass
class Slip:
    number: int
    banner: dict
    indicator: str = ''          # 'red' / 'green' / '' not seen
    occupied: bool = False
    dock: str = ''               # 'N' / 'S'


@dataclass
class CourseView:
    t1_gates: list = field(default_factory=list)      # Gate, first one first
    frame: GateFrame = None
    channel: list = field(default_factory=list)       # Gate, from the entry end
    debris: list = field(default_factory=list)        # landmarks past the channel
    task3_gate: Gate = None
    task3_yellow: dict = None
    task3_indicator: str = ''                         # 'red' / 'green' / ''
    slips: list = field(default_factory=list)         # Slip
    zones: dict = field(default_factory=dict)         # 'task6_zone_t3' / '_marina' -> landmark
    missing: list = field(default_factory=list)

    @property
    def marina_complete(self):
        """Return True once both docks and all six slips, each with its indicator, are seen."""
        named = {(s.dock, s.number) for s in self.slips if s.dock and s.indicator}
        return named >= {(d, n) for d in 'NS' for n in (1, 2, 3)}

    @property
    def best_slip_so_far(self):
        """Return the best slip among those seen (green, no vessel, lowest number)."""
        free = [s for s in self.slips if s.indicator == 'green' and not s.occupied]
        return min(free, key=lambda s: (s.number, s.dock)) if free else None

    @property
    def best_slip(self):
        """
        Return the slip to dock in (handbook 3.2.5), or None while the marina is incomplete.

        The lowest number available counts over every slip: an unseen slip could be better
        than all the seen ones.
        """
        return self.best_slip_so_far if self.marina_complete else None

    def expected(self, hint):
        """Return the map position of a hint (ahead_m, left_m); None without the gates."""
        if self.frame is None or hint is None:
            return None
        return self.frame.to_map(*hint)


# ---------------------------------------------------------------- recognition
def _pairs(reds, greens, width_range, nominal):
    """Pair red and green landmarks a gate's width apart, each in one pair at most."""
    lo, hi = width_range
    cands = []
    for r in reds:
        for g in greens:
            w = _norm(_sub(_pos(r), _pos(g)))
            if lo <= w <= hi:
                cands.append((abs(w - nominal) / nominal - 0.001 * (r['sightings']
                                                                    + g['sightings']), r, g))
    cands.sort(key=lambda c: c[0])
    used, gates = set(), []
    for _, r, g in cands:
        if r['id'] in used or g['id'] in used:
            continue
        used.update((r['id'], g['id']))
        gates.append(Gate(r, g))
    return gates


def _chain(gates, cfg):
    """Return the longest run of gates one after the other (a channel), in order."""
    lo, hi = cfg.channel_step_m
    cos_turn = math.cos(math.radians(cfg.channel_turn_deg))

    def follows(a, b):
        step = _sub(b.centre, a.centre)
        d = _norm(step)
        if not lo <= d <= hi:
            return False
        u = _unit(step)
        return abs(_dot(u, a.through)) >= cos_turn and abs(_dot(u, b.through)) >= cos_turn

    best = []
    for start in gates:
        for sign in (1.0, -1.0):
            chain = [start]
            while True:
                last = chain[-1]
                nxt = [g for g in gates if g not in chain and follows(last, g)
                       and _dot(_sub(g.centre, last.centre), last.through) * sign > 0]
                if not nxt:
                    break
                chain.append(min(nxt, key=lambda g: _norm(_sub(g.centre, last.centre))))
            if len(chain) > len(best):
                best = chain
    return best if len(best) >= 2 else []


class CourseModel:

    def __init__(self, cfg=CourseConfig()):
        self.cfg = cfg

    def view(self, landmarks, start=None, frame_heading=None):
        """
        Recognise the course on a map snapshot.

        landmarks: world map dicts (kind, color, east, north, confirmed, sightings, id);
        start: map (east, north) where the run started, to tell the first Task 1 gate;
        frame_heading: compass heading (deg) of the way through the Task 1 gates measured
        by the boat (usv_core.roboboat_tasks.T1Entry): when given, it is the gate frame's
        direction instead of the mapped gates' (no GPS drift in it).
        """
        cfg = self.cfg
        lms = [lm for lm in landmarks if lm['confirmed'] or cfg.use_unconfirmed]
        by_kind = {}
        for lm in lms:
            by_kind.setdefault(lm['kind'], []).append(lm)
        v = CourseView()
        self._task1(v, by_kind, start, frame_heading)
        small = _pairs(by_kind.get('buoy_red', []), by_kind.get('buoy_green', []),
                       cfg.buoy_gate_width_m, cfg.buoy_gate_nominal_m)
        self._channel(v, small, lms)
        self._task3(v, [g for g in small if g not in v.channel], by_kind)
        self._marina(v, by_kind)
        self._zones(v, by_kind)
        return v

    # ------------------------------------------------------------------ Task 1
    def _task1(self, v, by_kind, start, frame_heading=None):
        cfg = self.cfg
        lo, hi = cfg.t1_gate_width_m
        gates = _pairs(by_kind.get('pole_red', []), by_kind.get('pole_green', []),
                       (lo, hi), (lo + hi) / 2.0)
        if not gates:
            v.missing.append('t1_gates')
            return
        if start is not None:
            gates.sort(key=lambda g: _norm(_sub(g.centre, start)))
        g1 = gates[0]
        second = [g for g in gates[1:] if cfg.t1_gate_spacing_m[0]
                  <= _norm(_sub(g.centre, g1.centre)) <= cfg.t1_gate_spacing_m[1]]
        if second:
            g2 = min(second, key=lambda g: _norm(_sub(g.centre, g1.centre)))
            v.t1_gates = [g1, g2]
            # Three estimates of the same direction, averaged: gate to gate, and the way
            # through each gate (perpendicular to its poles, pointed the same way)
            along = _unit(_sub(g2.centre, g1.centre))
            ahead = along
            for g in (g1, g2):
                t = g.through
                if _dot(t, along) < 0:
                    t = (-t[0], -t[1])
                ahead = (ahead[0] + t[0], ahead[1] + t[1])
            ahead = _unit(ahead)
        else:
            v.t1_gates = [g1]
            ahead = g1.through
            if start is not None and _dot(_sub(g1.centre, start), ahead) < 0:
                ahead = (-ahead[0], -ahead[1])
            v.missing.append('t1_second_gate')
        if frame_heading is not None:
            h = math.radians(frame_heading)
            ahead = (math.sin(h), math.cos(h))
        v.frame = GateFrame(g1.centre, ahead)

    # ------------------------------------------------------------------ Task 2
    def _channel(self, v, small, lms):
        chain = _chain(small, self.cfg)
        if not chain:
            v.missing.append('channel')
            return
        if v.frame is not None and _norm(_sub(chain[-1].centre, v.frame.origin)) < \
                _norm(_sub(chain[0].centre, v.frame.origin)):
            chain.reverse()                 # enter from the end nearer to Task 1
        v.channel = chain
        end = chain[-1]
        out = _unit(_sub(end.centre, chain[-2].centre))
        in_channel = {i for g in chain for i in g.ids}
        # Past the last gate's line by a metre: obstacles beside the last gate are the
        # channel's, not the field's
        v.debris = [lm for lm in lms
                    if lm['kind'] in ('buoy_black', 'indicator') and lm['id'] not in in_channel
                    and _dot(_sub(_pos(lm), end.centre), out) > 1.0
                    and _norm(_sub(_pos(lm), end.centre)) <= self.cfg.debris_range_m]

    # ------------------------------------------------------------------ Task 3
    def _task3(self, v, gates, by_kind):
        cfg = self.cfg
        lo, hi = cfg.task3_yellow_m
        best = None
        for g in gates:
            for y in by_kind.get('buoy_yellow', []):
                d = _sub(_pos(y), g.centre)
                along = _dot(d, g.through)
                off = abs(_dot(d, (-g.through[1], g.through[0])))
                if lo <= abs(along) <= hi and off <= cfg.task3_line_m:
                    score = off + 0.01 * abs(along)
                    if best is None or score < best[0]:
                        best = (score, g, y, 1.0 if along > 0 else -1.0)
        if best is None:
            v.missing.append('task3')
            return
        _, g, y, sign = best
        v.task3_gate, v.task3_yellow = g, y
        axis = (g.through[0] * sign, g.through[1] * sign)
        length = _norm(_sub(_pos(y), g.centre))
        inds = []
        for lm in by_kind.get('indicator', []):
            d = _sub(_pos(lm), g.centre)
            along = _dot(d, axis)
            if 0.0 < along < length and abs(_dot(d, (-axis[1], axis[0]))) <= cfg.task3_line_m:
                inds.append(lm)
        if inds:
            v.task3_indicator = max(inds, key=lambda lm: lm['sightings'])['color']
        else:
            v.missing.append('task3_indicator')

    # ------------------------------------------------------------------ marina
    def _marina(self, v, by_kind):
        cfg = self.cfg
        banners = [lm for k in DIGIT_NUMBER for lm in by_kind.get(k, [])]
        if not banners:
            v.missing.append('marina')
            return
        docks = []                          # banners linked within dock_link_m
        for b in banners:
            near = [d for d in docks if any(_norm(_sub(_pos(b), _pos(o))) <= cfg.dock_link_m
                                            for o in d)]
            merged = [b] + [o for d in near for o in d]
            docks = [d for d in docks if d not in near] + [merged]
        names = {}
        if len(docks) >= 2:
            docks.sort(key=lambda d: -sum(_pos(o)[1] for o in d) / len(d))
            names = {id(docks[0]): 'N', id(docks[-1]): 'S'}
        else:
            v.missing.append('marina_second_dock')
        slips = {b['id']: Slip(DIGIT_NUMBER[b['kind']], b, dock=names.get(id(d), ''))
                 for d in docks for b in d}

        def nearest(lm, limit):
            b = min(banners, key=lambda b: _norm(_sub(_pos(lm), _pos(b))))
            return slips[b['id']] if _norm(_sub(_pos(lm), _pos(b))) <= limit else None

        for ind in sorted(by_kind.get('indicator', []), key=lambda lm: lm['sightings']):
            slip = nearest(ind, cfg.slip_indicator_m)
            if slip is not None:
                slip.indicator = ind['color'] or ''    # the most seen one is written last
        for mark in by_kind.get('triangle', []) + by_kind.get('plus', []):
            slip = nearest(mark, cfg.slip_vessel_m)
            if slip is not None:
                slip.occupied = True
        v.slips = sorted(slips.values(), key=lambda s: (s.dock, s.number))
        if not v.marina_complete:
            v.missing.append('marina_incomplete')

    # ------------------------------------------------------------------ Task 6 zones
    def _zones(self, v, by_kind):
        yellows = list(by_kind.get('buoy_yellow', []))
        if v.task3_yellow is not None:
            v.zones['task6_zone_t3'] = v.task3_yellow
            yellows = [y for y in yellows if y['id'] != v.task3_yellow['id']]
        if v.slips and yellows:
            c = (sum(_pos(s.banner)[0] for s in v.slips) / len(v.slips),
                 sum(_pos(s.banner)[1] for s in v.slips) / len(v.slips))
            v.zones['task6_zone_marina'] = min(yellows, key=lambda y: _norm(_sub(_pos(y), c)))
        for name in ('task6_zone_t3', 'task6_zone_marina'):
            if name not in v.zones:
                v.missing.append(name)
