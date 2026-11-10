"""
The RoboBoat 2027 marina (Task 3): a floating dock with three bays, a building in each.

Sizes from the RobotX 2026 build guide "Docking Bay Structure" (the same elements; drawings
in docs, RobotX Resources PDF):
  * dock: 500 mm cubes, 6596 mm long; main deck 13 x 2 cubes, four fingers of 4 cubes
    with 3 cubes between them: bays 1.5 m wide and 2.0 m deep
  * building: PVC frame 1.0 m wide (front), 0.75 m deep, 1.0 m tall, white panels; it
    stands on the deck at the back of its bay, its front panel facing into the bay
  * front panel, seen from the bay (mm from its left and bottom edges):
      window 1  210 x 290, left edge 175, bottom 605
      window 2  280 x 360, left edge 615, bottom 355
      both with a 25 mm black frame and a 16x16 RGB LED panel behind (the water target,
      210 x 280 mm); a 160 x 160 opening at the bottom middle for the Color Indicator
  * Color Indicator (the 2026 one) at the base, facing into the bay: GREEN = free bay,
    RED = not safe
Bays are numbered 1-3 from the left as seen from the water, facing the bays. ASSUMED: the
deck's height above the water, the panels' thickness, the top panel flat (it slopes ~10
deg).

The window lights (usv_sim.lights), as in RobotX 2026 Task 3: once the boat has stayed in
the green bay for hold_s seconds, one window lights RED (the fire); the event fire_out
(the water cannon hit it) turns it GREEN for 5 s, then off. At level disruptive the green
is followed by 1 s off and, for 60 s, the two-colour sequence: first colour 1 s, off 1 s,
second colour 1 s, off 2 s (RoboBoat 2027 Primer: first = what to deliver, red water / blue
ball; second = where). The camera reports each window as window_<colour>.

ROS-free, unit tested (test/test_marina.py).
"""

import math

from usv_sim import lights
from usv_sim.camera import Buoy, Panel

CUBE = 6.596 / 13.0               # cube pitch (m)
DECK = (13, 2)                    # cubes along, across
FINGER = 4                        # cubes out from the deck
FREEBOARD = 0.15                  # ASSUMED: cube top above the water
BUILDING = (1.0, 0.75, 1.0)       # width (front), depth, height
WINDOWS = ((0.175, 0.605, 0.210, 0.290), (0.615, 0.355, 0.280, 0.360))   # left, bottom, w, h
FRAME = 0.025
CUTOUT = (0.420, 0.160, 0.160)    # left, width, height
INDICATOR_RADIUS = 0.0635         # the 2026 Color Indicator (5 in wide, 5.2 in tall)
INDICATOR_HEIGHT = 0.132
GREEN_S, OFF_S, SEQUENCE_S = 5.0, 1.0, 60.0
LEVELS = ('core', 'disruptive')


def bay_centres():
    """Return each bay's centre (along the deck, out into the bay) in the marina frame."""
    return [((1 - j) * 4 * CUBE, FINGER * CUBE / 2.0) for j in range(3)]


def window_program(level, sequence):
    """Return the program a window runs once the fire is out."""
    program = [('green', GREEN_S)]
    if level == 'disruptive':
        first, second = sequence
        program.append(('off', OFF_S))
        for _ in range(int(SEQUENCE_S / 5.0)):
            program += [(first, 1.0), ('off', 1.0), (second, 1.0), ('off', 2.0)]
    return program + [('off', None)]


def build(spec, x0, y0, heading_deg, names, part, model_cls):
    """
    Build the marina: the dock model, camera targets (windows, indicators), moored vessels.

    spec: the course file's marina entry {name, bays: [{indicator, window (1|2, the one
    that lights; the green bay only), vessel (optional, moored in the bay)}] x 3, level,
    sequence [first, second], hold_s}. (x0, y0): world ENU of the middle of the deck's
    bay-side edge; heading_deg: compass direction the bays open to. part / model_cls:
    course.Part and course.Model (passed in, course.py imports this module). Returns
    (dock model, targets, [(name, colour, x, y, yaw, shape) of vessels moored in bays]).
    A bay's vessel may carry a 2027 shape (`shape`).
    """
    if len(spec['bays']) != 3:
        raise ValueError('a marina has three bays')
    level = spec.get('level', 'core')
    if level not in LEVELS:
        raise ValueError(f'marina level {level!r}: one of {", ".join(LEVELS)}')
    v_yaw = math.radians(90.0 - heading_deg)               # ENU yaw of "out into the bays"
    v = (math.cos(v_yaw), math.sin(v_yaw))
    u = (v[1], -v[0])                                       # along the deck, bay 1 side

    def world(du, dv):
        return x0 + du * u[0] + dv * v[0], y0 + du * u[1] + dv * v[1]

    # The dock model's frame: x out into the bays (v), y = -u; a point (du, dv) is (dv, -du)
    beige, white, black = (0.80, 0.72, 0.58), (0.92, 0.92, 0.90), (0.04, 0.04, 0.04)
    dock = model_cls(f"marina_{spec.get('name', 'M')}", x0, y0, v_yaw)
    z = (FREEBOARD - 0.25) / 2.0
    h = FREEBOARD + 0.25
    dock.parts.append(part('box', (DECK[1] * CUBE, DECK[0] * CUBE, h),
                           (-DECK[1] * CUBE / 2.0, 0.0, z, 0.0), beige))
    for k in (0, 4, 8, 12):
        du = (k - 6) * CUBE
        dock.parts.append(part('box', (FINGER * CUBE, CUBE, h),
                               (FINGER * CUBE / 2.0, -du, z, 0.0), beige))
    targets, moored = [], []
    bw, bd, bh = BUILDING
    for j, (bay, (du, dv)) in enumerate(zip(spec['bays'], bay_centres())):
        number = j + 1
        base = FREEBOARD
        if bay['indicator'] not in ('red', 'green'):
            raise ValueError(f'bay {number}: indicator red or green')
        # Building on the deck, front panel flush with the deck edge
        dock.parts.append(part('box', (bd, bw, bh), (-bd / 2.0, -du, base + bh / 2.0, 0.0),
                               white))
        # Windows: seen from the bay, "left" is +u
        for w, (left, bottom, ww, wh) in enumerate(WINDOWS):
            cu = du + bw / 2.0 - left - ww / 2.0
            cz = base + bottom + wh / 2.0
            dock.parts.append(part('box', (0.01, ww + 2 * FRAME, wh + 2 * FRAME),
                                   (0.005, -cu, cz, 0.0), black, False))
            dock.parts.append(part('box', (0.01, ww, wh), (0.011, -cu, cz, 0.0),
                                   (0.12, 0.12, 0.12), False))
            fire = bay['indicator'] == 'green' and bay.get('window', 1) == w + 1
            light = lights.Light('off')
            if fire:
                near = world(du, dv) + (0.8,)
                light.changes = [
                    lights.Change([('red', None)], near=near, hold_s=spec.get('hold_s', 3.0)),
                    lights.Change(window_program(level, spec.get('sequence', ('red', 'blue'))),
                                  event='fire_out', after=0)]
            px, py = world(cu, 0.012)
            targets.append(Panel(px, py, cz - wh / 2.0, cz + wh / 2.0, ww,
                                 math.degrees(v_yaw), names.index('window_off'),
                                 light=light))
        # Color Indicator in the opening at the bottom middle
        color = bay['indicator']
        cu = du + bw / 2.0 - CUTOUT[0] - CUTOUT[1] / 2.0
        dock.parts.append(part('cylinder', (INDICATOR_RADIUS, INDICATOR_HEIGHT),
                               (0.08, -cu, base + INDICATOR_HEIGHT / 2.0, 0.0),
                               {'red': (0.75, 0.02, 0.02), 'green': (0.0, 0.55, 0.12)}[color],
                               False))
        ix, iy = world(cu, 0.08)
        targets.append(Buoy(ix, iy, INDICATOR_RADIUS, base + INDICATOR_HEIGHT,
                            names.index(f'indicator_{color}'), z0=base))
        if bay.get('vessel'):
            # Moored in the bay, bow to the building (course.vessel builds it)
            mx, my = world(du, 0.95)
            moored.append((f"marina_bay{number}_{bay['vessel']}", bay['vessel'], mx, my,
                           v_yaw + math.pi, bay.get('shape')))
    return dock, targets, moored
