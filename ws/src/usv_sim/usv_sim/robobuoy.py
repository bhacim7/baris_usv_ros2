"""
RoboBuoy with a Light Beacon (RoboBoat 2027 Task 1).

Its size, its light, and the light's class at a given moment. ROS-free, unit tested
(test/test_robobuoy.py).

Sizes from the RobotX 2026 task build guides (RoboBuoy build guide, Light Beacon
specifications; RoboBoat 2027 uses the same elements); what they leave open is ASSUMED:
  * four white coroplast side panels, 17 x 12 in, leaning in to a flat top (a frustum);
    the top width (4 in) is ASSUMED: the guide gives the panels, not the assembled shape
  * a black diamond on each face, about 8 in tall, starting 2.5 in up the panel; its width
    (0.75 of its height) is ASSUMED
  * under the panels a square PVC frame wrapped in 3 in pool noodles (two 38 in noodles
    mitred round it: about 19 in a side), half in the water (ASSUMED)
  * on top the Light Beacon: RGB LED strip round a 3.5 in OD PVC pipe, 5.25 in long, under
    a 10 in black sun hat

Light (RobotX 2026 Task 1): red and green FLASH, 1 s on and 1 s off; flashing blue marks
the ENTRY, solid blue the EXIT; off (black) is an obstacle passed on either side. The
camera reports the body and the light as one class (`beacon_red`, ..., `beacon_off`), as
Baris labels the dataset, so a flashing beacon is `beacon_off` half of the time.
"""

from dataclasses import dataclass, field, replace
import math

INCH = 0.0254
PANEL_BASE = 17.0 * INCH          # 0.432 m
PANEL_TOP = 4.0 * INCH            # ASSUMED
PANEL_SLANT = 12.0 * INCH         # 0.305 m along the face
PANEL_HEIGHT = math.sqrt(PANEL_SLANT ** 2 - ((PANEL_BASE - PANEL_TOP) / 2.0) ** 2)
NOODLE_D = 3.0 * INCH
FLOAT_SIDE = 19.0 * INCH          # outer side of the noodle square
PANEL_BOTTOM = NOODLE_D / 2.0     # panels stand on the noodles, half of which are under water
DIAMOND_START = 2.5 * INCH        # up the face, from its bottom edge
DIAMOND_LENGTH = 8.0 * INCH       # along the face
DIAMOND_WIDTH = 0.75 * DIAMOND_LENGTH     # ASSUMED
BEACON_RADIUS = 3.5 * INCH / 2.0
BEACON_LENGTH = 5.25 * INCH
FLANGE = 0.01                     # ASSUMED: printed flange under the pipe
HAT_RADIUS = 10.0 * INCH / 2.0
HAT_THICKNESS = 0.005
BEACON_BOTTOM = PANEL_BOTTOM + PANEL_HEIGHT + FLANGE
TOP = BEACON_BOTTOM + BEACON_LENGTH + HAT_THICKNESS      # ~0.45 m above the water
CAMERA_RADIUS = FLOAT_SIDE / 2.0  # what a box spans: the float frame at the water

COLORS = ('red', 'green', 'blue', 'off')
FLASH_PERIOD_S = 2.0              # 1 s on, 1 s off
FLASH_ON_S = 1.0


@dataclass
class Change:
    """A change of the light: at at_s seconds, or when the boat comes within near's radius."""

    color: str
    flash: bool
    at_s: float = None
    near: tuple = None            # (east, north, radius_m), world ENU


@dataclass
class Light:
    color: str                    # red, green, blue or off
    flash: bool = False
    phase_s: float = 0.0          # where in its flash cycle the light is at t = 0
    changes: list = field(default_factory=list)
    done: set = field(default_factory=set)        # changes that have happened

    def __post_init__(self):
        for c in [self.color] + [ch.color for ch in self.changes]:
            if c not in COLORS:
                raise ValueError(f'light colour {c!r}: one of {", ".join(COLORS)}')

    def state(self, t, boat_xy=None):
        """Return what the light shows at t (s): its colour, or 'off' (dark in a flash too)."""
        for k, ch in enumerate(self.changes):
            if k in self.done:
                continue
            due = ch.at_s is not None and t >= ch.at_s
            if ch.near is not None and boat_xy is not None:
                due = due or math.hypot(boat_xy[0] - ch.near[0],
                                        boat_xy[1] - ch.near[1]) <= ch.near[2]
            if due:
                self.done.add(k)
                self.color, self.flash = ch.color, ch.flash
        if self.color == 'off':
            return 'off'
        if self.flash and (t + self.phase_s) % FLASH_PERIOD_S >= FLASH_ON_S:
            return 'off'
        return self.color


def parse_light(spec, to_world):
    """
    Build a Light from a course file entry.

    spec: a colour ('red') or {color, flash, phase_s, changes: [{color, flash, at_s} or
    {color, flash, near: {east, north, within_m}}]}; to_world maps a file position (dict)
    to world ENU (east, north).
    """
    if isinstance(spec, str):
        spec = {'color': spec}
    changes = []
    for ch in spec.get('changes', []):
        near = None
        if 'near' in ch:
            x, y = to_world(ch['near'])
            near = (x, y, float(ch['near']['within_m']))
        changes.append(Change(ch['color'], bool(ch.get('flash', False)),
                              at_s=ch.get('at_s'), near=near))
    return Light(spec['color'], bool(spec.get('flash', False)),
                 float(spec.get('phase_s', 0.0)), changes)


def class_name(state):
    return f'beacon_{state}'


def lit_targets(targets, names, t, boat_xy=None):
    """Return the camera targets with every beacon's class as its light shows it at t."""
    out = []
    for target in targets:
        light = getattr(target, 'light', None)
        if light is None:
            out.append(target)
        else:
            out.append(replace(target, class_id=names.index(class_name(
                light.state(t, boat_xy)))))
    return out
