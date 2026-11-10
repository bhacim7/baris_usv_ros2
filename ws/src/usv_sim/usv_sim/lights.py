"""
Lights on the RoboBoat 2027 course: RoboBuoy beacons and the marina windows.

A camera target (camera.Buoy or camera.Panel) may carry a Light; the camera then reports it
with the class of the moment, `<prefix>_<colour>` (beacon_red, window_off, ...), as the
dataset labels the object and its light together. ROS-free, unit tested
(test/test_lights.py).

A light runs a program: steps (colour, seconds), the last one held (seconds None). A plain
colour is the program [(colour, None)]. A flashing light is dark 1 s of every 2 (RobotX
2026: red and green beacons, the ENTRY blue). Changes replace the program when they happen:
  at_s       at that time (s since the simulation started)
  near       when the boat comes within (east, north, radius_m), staying hold_s seconds
  event      when the event is fired (fake_perception: topic sim/event, e.g. fire_out:
             the water cannon hit the window; to come from the turret, roadmap step 7)
  after      only once change number `after` of the same light has happened
"""

from dataclasses import dataclass, field, replace
import math

COLORS = ('red', 'green', 'blue', 'off')
FLASH_PERIOD_S = 2.0              # 1 s on, 1 s off
FLASH_ON_S = 1.0


def _check(program):
    for color, _ in program:
        if color not in COLORS:
            raise ValueError(f'light colour {color!r}: one of {", ".join(COLORS)}')
    return program


@dataclass
class Change:
    """One change of a light: its new program, and what makes it happen."""

    program: list
    flash: bool = False
    at_s: float = None
    near: tuple = None            # (east, north, radius_m), world ENU
    hold_s: float = 0.0
    event: str = None
    after: int = None
    since: float = None           # the boat came within `near` at this time


@dataclass
class Light:
    color: str                    # the colour it starts with (what Gazebo draws)
    flash: bool = False
    phase_s: float = 0.0          # where in its flash cycle the light is at t = 0
    changes: list = field(default_factory=list)
    done: set = field(default_factory=set)
    events: set = field(default_factory=set)
    program: list = None
    started: float = 0.0

    def __post_init__(self):
        if self.program is None:
            self.program = [(self.color, None)]
        _check(self.program)
        for ch in self.changes:
            _check(ch.program)

    def fire(self, event):
        """Note an event (e.g. 'fire_out'); changes waiting for it happen at the next look."""
        self.events.add(event)

    def _due(self, ch, t, boat_xy):
        if ch.after is not None and ch.after not in self.done:
            return False
        if ch.at_s is not None and t >= ch.at_s:
            return True
        if ch.event is not None and ch.event in self.events:
            return True
        if ch.near is not None and boat_xy is not None:
            if math.hypot(boat_xy[0] - ch.near[0], boat_xy[1] - ch.near[1]) <= ch.near[2]:
                ch.since = t if ch.since is None else ch.since
                return t - ch.since >= ch.hold_s
            ch.since = None
        return False

    def state(self, t, boat_xy=None):
        """Return what the light shows at t (s): a colour, or 'off' (dark in a flash too)."""
        for k, ch in enumerate(self.changes):
            if k not in self.done and self._due(ch, t, boat_xy):
                self.done.add(k)
                self.program, self.flash, self.started = ch.program, ch.flash, t
        elapsed = t - self.started
        color = self.program[-1][0]
        for c, seconds in self.program:
            if seconds is None or elapsed < seconds:
                color = c
                break
            elapsed -= seconds
        if color == 'off':
            return 'off'
        if self.flash and (t + self.phase_s) % FLASH_PERIOD_S >= FLASH_ON_S:
            return 'off'
        return color


def parse_program(spec):
    """Return a program from a colour or a list of [colour, seconds] (last seconds None)."""
    if isinstance(spec, str):
        return [(spec, None)]
    return [(c, None if s is None else float(s)) for c, s in spec]


def parse_light(spec, to_world):
    """
    Build a Light from a course file entry.

    spec: a colour ('red') or {color, flash, phase_s, changes: [...]}; a change has its new
    look as `color` (and `flash`) or `program` ([[colour, seconds], ...]) and one or more
    of at_s, near {east, north, within_m, hold_s}, event, after. to_world maps a file
    position (dict) to world ENU (east, north).
    """
    if isinstance(spec, str):
        spec = {'color': spec}
    changes = []
    for ch in spec.get('changes', []):
        near, hold = None, 0.0
        if 'near' in ch:
            x, y = to_world(ch['near'])
            near = (x, y, float(ch['near']['within_m']))
            hold = float(ch['near'].get('hold_s', 0.0))
        program = parse_program(ch['program'] if 'program' in ch else ch['color'])
        changes.append(Change(program, bool(ch.get('flash', False)), at_s=ch.get('at_s'),
                              near=near, hold_s=hold, event=ch.get('event'),
                              after=ch.get('after')))
    return Light(spec['color'], bool(spec.get('flash', False)),
                 float(spec.get('phase_s', 0.0)), changes)


def lit_targets(targets, names, t, boat_xy=None):
    """Return the camera targets with every light's class as it shows at t."""
    out = []
    for target in targets:
        light = getattr(target, 'light', None)
        if light is None:
            out.append(target)
        else:
            prefix = names[target.class_id].rsplit('_', 1)[0]
            out.append(replace(target, class_id=names.index(
                f'{prefix}_{light.state(t, boat_xy)}')))
    return out


def fire(targets, event):
    """Fire an event on every light of the course (fake_perception's sim/event)."""
    for target in targets:
        if getattr(target, 'light', None) is not None:
            target.light.fire(event)
