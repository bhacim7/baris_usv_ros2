"""
A random Task 1 (Safe Passage) field for RoboBoat 2027, the same for the same seed.

RobotX 2026 Task 1, which RoboBoat 2027 follows (Primer): ten RoboBuoys with light
beacons; the boat circles the flashing blue ENTRY clockwise, crosses the field keeping
greens to port and reds to starboard (a buoy counts when it is the nearest on that side
abeam), passes off (black) ones on either side, and circles the solid blue EXIT
counter-clockwise. The layout changes every run. Neither handbook gives the field's size
or spacing: length_m, width_m and min_gap_m are ASSUMED.

How a valid field is made: a smooth random path is drawn from the ENTRY to the EXIT first;
greens go to its left, reds to its right, 1.8-3.0 m off it, offs at least 3 m from it,
every buoy min_gap_m from the others and 6 m from the ENTRY and EXIT (room to circle). So
the path is always there; the boat has to find it (or a better one).

Levels (RoboBoat 2027 Primer):
  core        nothing changes
  advanced    the EXIT moves while the boat is in the field: when it comes within
              trigger_m of the field's middle, the solid blue goes off and another buoy
              near the far end (off until then) turns solid blue
  disruptive  advanced, and at the same moment up to two reds/greens swap colour; only
              ones with room on their other side (the boat must now pass them there) are
              chosen. That room is checked against the buoys, not proved by a plan.

ROS-free, unit tested (test/test_task1_field.py). Positions come out in the course file's
east/north frame, as course file buoy entries.
"""

import math
import random

CLEAR_M = 1.6             # path to a red/green: half beam 0.39 + float 0.26 + margin
SIDE_M = (1.8, 3.0)       # red/green distance from the path
OFF_CLEAR_M = 3.0
CIRCLE_ROOM_M = 6.0       # around the ENTRY and the EXIT
LEVELS = ('core', 'advanced', 'disruptive')


def path_points(rng, length, width):
    """Return the reference path in the field frame (x ahead from the ENTRY, y left)."""
    pts = [(0.0, 0.0), (5.0, 0.0)]
    y = 0.0
    for k in (1, 2, 3):
        y = max(-width / 3.0, min(width / 3.0, y + rng.uniform(-width / 4.0, width / 4.0)))
        pts.append((length * k / 4.0, y))
    exit_y = rng.uniform(-width / 4.0, width / 4.0)
    pts += [(length - 5.0, exit_y), (length, exit_y)]
    return pts


def dist_to_path(p, pts):
    best = math.inf
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        dx, dy = x2 - x1, y2 - y1
        f = max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x1 + f * dx - p[0], y1 + f * dy - p[1]))
    return best


def along(pts, x):
    """Return the point and unit left normal of the path where it is x ahead."""
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        if x1 <= x <= x2 and x2 > x1:
            f = (x - x1) / (x2 - x1)
            n = math.hypot(x2 - x1, y2 - y1)
            return (x1 + f * (x2 - x1), y1 + f * (y2 - y1)), (-(y2 - y1) / n, (x2 - x1) / n)
    raise ValueError(f'{x} is not on the path')


def layout(seed, length_m=40.0, width_m=25.0, buoys=10, min_gap_m=4.0, level='core'):
    """
    Return the field in its own frame: (path, buoys), buoys as dicts {x, y, role, light}.

    role: entry, exit, red, green, off, next_exit (advanced: off until the EXIT moves)
    light: a course file light entry (lights.parse_light); changes give `near` in the
    field frame ({x, y, within_m}), turned into file positions by `course_entries`.
    """
    if level not in LEVELS:
        raise ValueError(f'level {level!r}: one of {", ".join(LEVELS)}')
    if buoys < 6:
        raise ValueError('a field needs at least 6 buoys (ENTRY, EXIT, 2 red, 2 green)')
    rng = random.Random(seed)
    for _ in range(200):
        out = _try(rng, length_m, width_m, buoys, min_gap_m, level)
        if out:
            return out
    raise RuntimeError(f'no field found for seed {seed}: too many buoys for the space?')


def _try(rng, length, width, n, gap, level):
    pts = path_points(rng, length, width)
    entry, exit_ = {'x': 0.0, 'y': 0.0, 'role': 'entry'}, \
        {'x': pts[-1][0], 'y': pts[-1][1], 'role': 'exit'}
    placed = [entry, exit_]
    rest = n - 2
    offs = max(1, rest // 4)
    if level != 'core':
        offs = max(offs, 2)                 # one of them is the EXIT to be
    reds = (rest - offs) // 2 + rng.randint(0, (rest - offs) % 2)
    roles = ['red'] * reds + ['green'] * (rest - offs - reds) + ['off'] * offs
    rng.shuffle(roles)

    def free(p, need=gap):
        if not (0.0 <= p[0] <= length and -width / 2.0 <= p[1] <= width / 2.0):
            return False
        for b in placed:
            d = math.hypot(b['x'] - p[0], b['y'] - p[1])
            if d < need or (b['role'] in ('entry', 'exit') and d < CIRCLE_ROOM_M):
                return False
        return True

    for role in roles:
        for _ in range(100):
            if role == 'off':
                p = (rng.uniform(0.0, length), rng.uniform(-width / 2.0, width / 2.0))
                ok = dist_to_path(p, pts) >= OFF_CLEAR_M
            else:
                c, left = along(pts, rng.uniform(CIRCLE_ROOM_M, length - CIRCLE_ROOM_M))
                side = 1.0 if role == 'green' else -1.0
                d = side * rng.uniform(*SIDE_M)
                p = (c[0] + d * left[0], c[1] + d * left[1])
                ok = dist_to_path(p, pts) >= CLEAR_M
            if ok and free(p):
                placed.append({'x': p[0], 'y': p[1], 'role': role})
                break
        else:
            return None

    for b in placed:
        b['light'] = {'color': {'entry': 'blue', 'exit': 'blue', 'off': 'off'}.get(
            b['role'], b['role']), 'flash': b['role'] in ('entry', 'red', 'green'),
            'phase_s': round(rng.uniform(0.0, 2.0), 2)}
    if level == 'core':
        return pts, placed

    # The EXIT moves when the boat reaches the middle of the field
    mid, _ = along(pts, length / 2.0)
    near = {'x': mid[0], 'y': mid[1], 'within_m': 6.0}
    far = [b for b in placed if b['role'] == 'off' and b['x'] >= 0.6 * length
           and math.hypot(b['x'] - exit_['x'], b['y'] - exit_['y']) >= CIRCLE_ROOM_M]
    if not far:
        return None
    nxt = max(far, key=lambda b: b['x'])
    nxt['role'] = 'next_exit'
    nxt['light']['changes'] = [{'color': 'blue', 'flash': False, 'near': near}]
    exit_['light']['changes'] = [{'color': 'off', 'flash': False, 'near': near}]
    if level == 'advanced':
        return pts, placed

    # Disruptive: up to two reds/greens past the middle swap colour, if the boat has room
    # on their other side
    flips = 0
    for b in sorted((b for b in placed if b['role'] in ('red', 'green')
                     and b['x'] > length / 2.0), key=lambda b: rng.random()):
        if flips == 2:
            break
        c, left = along(pts, b['x'])
        away = 1.0 if (b['x'] - c[0]) * left[0] + (b['y'] - c[1]) * left[1] > 0 else -1.0
        beyond = (b['x'] + away * 2.0 * CLEAR_M * left[0],
                  b['y'] + away * 2.0 * CLEAR_M * left[1])
        others = [o for o in placed if o is not b]
        if not (-width / 2.0 <= beyond[1] <= width / 2.0) or any(
                math.hypot(o['x'] - beyond[0], o['y'] - beyond[1]) < CLEAR_M for o in others):
            continue
        other = 'green' if b['role'] == 'red' else 'red'
        b['light']['changes'] = [{'color': other, 'flash': True, 'near': near}]
        b['swaps_to'] = other
        flips += 1
    return pts, placed


def course_entries(field):
    """
    Return the course file buoy entries of a course file's task1_field section.

    field: {seed, entry {east, north}, heading_deg, length_m, width_m, buoys, min_gap_m,
    level}; positions in the file's east/north frame, ENTRY -> EXIT along heading_deg.
    """
    _, placed = layout(field.get('seed', 1), field.get('length_m', 40.0),
                       field.get('width_m', 25.0), field.get('buoys', 10),
                       field.get('min_gap_m', 4.0), field.get('level', 'core'))
    h = math.radians(float(field.get('heading_deg', 0.0)))
    ahead, left = (math.sin(h), math.cos(h)), (-math.cos(h), math.sin(h))
    e0, n0 = float(field['entry']['east']), float(field['entry']['north'])

    def to_file(x, y):
        return (round(e0 + x * ahead[0] + y * left[0], 3),
                round(n0 + x * ahead[1] + y * left[1], 3))

    out = []
    for k, b in enumerate(placed):
        e, n = to_file(b['x'], b['y'])
        light = dict(b['light'])
        if 'changes' in light:
            light['changes'] = []
            for ch in b['light']['changes']:
                ch = dict(ch)
                ne, nn = to_file(ch['near']['x'], ch['near']['y'])
                ch['near'] = {'east': ne, 'north': nn, 'within_m': ch['near']['within_m']}
                light['changes'].append(ch)
        out.append({'type': 'robobuoy', 'east': e, 'north': n, 'light': light,
                    'name': f"task1_{k:02d}_{b['role']}"})
    return out
