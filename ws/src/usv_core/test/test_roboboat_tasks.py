"""
Task 1 phases (usv_core.roboboat_tasks) through a whole run, on the rough boat model.

The map is built as the boat goes: a landmark appears once the boat has been within 12 m
of it (the mapper's range), so at the start only the first gate is on the map, as in the
simulation. The course model reads it every step, with the compass frame once measured.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_roboboat_tasks.py -v
"""

import math

from usv_core.nav_config import NavConfig
from usv_core.roboboat_course import CourseModel
from usv_core.roboboat_run import FINISHED, RoboBoatRun, RunConfig, RunContext
from usv_core.roboboat_tasks import circular_mean_deg, default_phases
from usv_core.world_map import latlon

CFG = NavConfig()
ORIGIN = (27.37, -82.45)
POLES = [('pole_red', -4.33, 0.0), ('pole_green', 4.33, 0.0),
         ('pole_red', -4.33, 7.95), ('pole_green', 4.33, 7.95)]


# The roboboat_2026 channel, from the first Task 1 gate's centre: 2.5 m gates, 6 m apart
CHANNEL = []
for cx, y in ((1.52, 21.13), (2.12, 27.13), (2.72, 33.13), (3.72, 39.13)):
    CHANNEL += [('buoy_red', cx - 1.25, y), ('buoy_green', cx + 1.25, y)]
BLACKS = [('buoy_black', 2.07, 33.33), ('buoy_black', 3.07, 39.43)]   # inside gates 3 and 4
OFF_LINE = [('buoy_black', 1.82, 24.13)]      # between gates 1 and 2, on the way
DEBRIS = [('buoy_black', 4.30, 45.13)]        # 6 m past gate 4, on the channel's line
# The roboboat_2026 debris field, from the first Task 1 gate's centre (course east - 2.05):
# five black buoys, the green indicator (a black 2.8 m from it) and the red one
FIELD = [('buoy_black', 5.47, 45.13), ('buoy_black', 10.47, 47.13), ('buoy_black', 14.47, 52.13),
         ('buoy_black', 18.47, 55.13), ('buoy_black', 7.47, 54.13),
         ('indicator', 12.47, 49.13, 'green'), ('indicator', 16.47, 58.13, 'red')]


def landmarks(turn_deg, items=POLES):
    """Return the items (kind, x, y[, colour]) as confirmed map landmarks, turned."""
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    return [{'id': i, 'kind': it[0], 'color': it[3] if len(it) > 3 else '',
             'east': it[1] * c - it[2] * s, 'north': it[1] * s + it[2] * c,
             'confirmed': True, 'sightings': 30} for i, it in enumerate(items)]


def run_task1(turn_deg=0.0, seconds=200.0, dt=0.04, items=POLES, task_order=(),
              hints=None, until=FINISHED, see_m=None, drift=None, drift_north=None,
              camera=False):
    """
    Start 8 m before the first gate, facing it; course drawn heading north, turned.

    see_m: {kind: range} for kinds mapped from nearer than the mapper's 12 m. drift: once
    the boat has been north of drift_north, the GPS is off by this (east, north): the boat's
    position and what it maps from then on move with it (the landmarks it then sees again
    get copies there, as the mapper made them), what it mapped before stays. camera: every
    step a frame of what is within 10 m and 55 deg of the bow, placed by the (drifted) GPS.
    """
    see_m = see_m or {}
    every = landmarks(turn_deg, items)
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    boat = (0.0 * c - (-8.0) * s, 0.0 * s + (-8.0) * c)
    heading = (0.0 - turn_deg) % 360.0                 # north, turned counter-clockwise
    run = RoboBoatRun(RunConfig(task_order=task_order), default_phases(CFG, hints=hints))
    model = CourseModel()
    ctx = RunContext(now=0.0, boat_xy=boat, heading=heading,
                     map_origin=ORIGIN if camera else None)
    seen, copies, last, track = set(), {}, {}, [boat]
    true_xy, off = boat, (0.0, 0.0)
    while ctx.now < seconds and run.phase != until:
        if drift is not None and true_xy[1] > drift_north:
            off = drift
        for lm in every:
            if math.hypot(lm['east'] - true_xy[0], lm['north'] - true_xy[1]) <= \
                    see_m.get(lm['kind'], 12.0):
                if off == (0.0, 0.0):
                    seen.add(lm['id'])
                    last[lm['id']] = ctx.now
                else:                             # mapped where the drifted GPS puts it
                    copies.setdefault(lm['id'], dict(lm, id=1000 + lm['id'],
                                                     east=lm['east'] + off[0],
                                                     north=lm['north'] + off[1]))
                    last[1000 + lm['id']] = ctx.now
        ctx.landmarks = [dict(lm, last_seen=last[lm['id']]) for lm in
                         [lm for lm in every if lm['id'] in seen] + list(copies.values())]
        ctx.boat_xy = (true_xy[0] + off[0], true_xy[1] + off[1])
        if camera:
            ctx.detections, ctx.detections_time = [], ctx.now
            for lm in every:
                dx, dy = lm['east'] - true_xy[0], lm['north'] - true_xy[1]
                bearing = math.degrees(math.atan2(dx, dy))
                if math.hypot(dx, dy) <= 10.0 and \
                        abs((bearing - ctx.heading + 180.0) % 360.0 - 180.0) <= 55.0:
                    name = f"indicator_{lm['color']}" if lm['kind'] == 'indicator' \
                        else lm['kind']
                    lat, lon = latlon(lm['east'] + off[0], lm['north'] + off[1], *ORIGIN)
                    ctx.detections.append({'class_name': name, 'position_ok': True,
                                           'lat': lat, 'lon': lon})
        ctx.course = model.view(ctx.landmarks, run.memory.get('start_xy') or boat,
                                run.memory.get('t1_heading'))
        cmd = run.step(ctx, True)
        fwd, yaw = cmd or (1500, 0)
        speed = (fwd - 1500) / 180.0 * 1.3 if fwd >= 1500 else (fwd - 1500) / 150.0 * 0.7
        h = (ctx.heading + max(-60.0, min(60.0, yaw * 0.25)) * dt) % 360.0
        true_xy = (true_xy[0] + speed * math.sin(math.radians(h)) * dt,
                   true_xy[1] + speed * math.cos(math.radians(h)) * dt)
        ctx.heading, ctx.speed = h, speed
        ctx.now += dt
        track.append(true_xy)
    return run, track, every


def crossings(track, red, green):
    """Return [(direction, offset from the centre)] each time the track crosses the gate."""
    lx, ly = red['east'] - green['east'], red['north'] - green['north']
    w = math.hypot(lx, ly)
    left = (lx / w, ly / w)
    through = (left[1], -left[0])
    cx, cy = (red['east'] + green['east']) / 2.0, (red['north'] + green['north']) / 2.0
    out = []
    for a, b in zip(track, track[1:]):
        da = (a[0] - cx) * through[0] + (a[1] - cy) * through[1]
        db = (b[0] - cx) * through[0] + (b[1] - cy) * through[1]
        if (da < 0) != (db < 0):
            side = (b[0] - cx) * left[0] + (b[1] - cy) * left[1]
            out.append(('in' if db > da else 'out', side, w))
    return out


def test_circular_mean_wraps_round_north():
    assert abs((circular_mean_deg([350.0, 10.0]) + 180.0) % 360.0 - 180.0) < 1e-9
    assert abs(circular_mean_deg([80.0, 100.0]) - 90.0) < 1e-9


def test_task1_entry_return_exit():
    for turn in (0.0, 120.0):
        run, track, every = run_task1(turn)
        assert run.phase == FINISHED, (turn, run.events[-3:])
        kinds = [(k, d.get('phase')) for _, k, d in run.events]
        for phase in ('T1_ENTRY', 'RETURN', 'T1_EXIT'):
            assert ('phase_done', phase) in kinds, (turn, phase, kinds)
        for red, green in ((every[0], every[1]), (every[2], every[3])):
            xs = crossings(track, red, green)
            assert [d for d, _, _ in xs] == ['in', 'out'], (turn, xs)
            assert all(abs(side) < w / 2.0 - 0.5 for _, side, w in xs), (turn, xs)
        mem = run.memory
        assert mem['t1_entry']['through'] and mem['t1_exit']['through']
        true_heading = (0.0 - turn) % 360.0
        assert abs((mem['t1_heading'] - true_heading + 180.0) % 360.0 - 180.0) < 3.0


def test_task2_through_the_channel_and_back():
    """
    Go in through all four channel gates and back out through all four.

    With an exact hint and with one 3 m off; a black buoy between gates 1 and 2 on the
    way, one inside gates 3 and 4 beside the red, and one where the debris field starts.
    """
    items = POLES + CHANNEL + BLACKS + OFF_LINE + DEBRIS
    for hint in ((21.1, -1.5), (21.1 + 2.1, -1.5 + 2.1)):
        run, track, every = run_task1(0.0, seconds=400.0, items=items,
                                      task_order=('task2',), hints={'task2': hint},
                                      until='RETURN')
        assert run.phase == 'RETURN', (hint, run.events[-3:])
        assert ('phase_done', 'task2') in [(k, d.get('phase')) for _, k, d in run.events]
        for k in range(4):
            red, green = every[4 + 2 * k], every[5 + 2 * k]
            xs = crossings(track, red, green)
            assert [d for d, _, _ in xs] == ['in', 'out'], (hint, k, xs)
            # the whole hull between the buoys: 0.39 m half beam + 0.10 m buoy radius
            assert all(abs(side) < w / 2.0 - 0.49 for _, side, w in xs), (hint, k, xs)
        mem = run.memory['task2']
        assert mem['gates'] == 4 and mem['back_gates'] == 4 and mem['stuck'] == 0
        assert all(mem['through']) and all(mem['back_through'])
        for b in every[12:16]:                # no touch: 0.39 half beam + 0.10 buoy
            assert min(math.hypot(p[0] - b['east'], p[1] - b['north']) for p in track) > 0.6
        field = run.memory['task2_debris']    # no indicators: scanned, then out
        assert field['green'] is None and field['red'] is None and field['stopped']


def test_task2_debris_field():
    """
    Past the channel: circle the green indicator 360 deg, note the red, map the blacks.

    The course's field: a black 2.8 m from the green, inside the 2.5 m ring, which bulges
    round it. No buoy touched, the red never come near; then back out through the channel.
    """
    items = POLES + CHANNEL + BLACKS + FIELD
    run, track, every = run_task1(0.0, seconds=500.0, items=items, task_order=('task2',),
                                  hints={'task2': (21.1, -1.5)}, until='RETURN')
    assert run.phase == 'RETURN', run.events[-3:]
    field = run.memory['task2_debris']
    assert field['green']['done'] and field['green']['swept_deg'] >= 360.0, field['green']
    assert field['red'] is not None and field['red']['color'] == 'red'
    assert len(field['debris']) == 7
    assert run.memory['task2']['back_gates'] == 4 and all(run.memory['task2']['back_through'])
    green, red = every[-2], every[-1]
    for b in every[12:-2]:                    # the blacks: 0.39 half beam + 0.10 buoy
        assert min(math.hypot(p[0] - b['east'], p[1] - b['north']) for p in track) > 0.6, b
    assert min(math.hypot(p[0] - green['east'], p[1] - green['north']) for p in track) > 1.2
    assert min(math.hypot(p[0] - red['east'], p[1] - red['north']) for p in track) > 1.0


def test_task2_debris_scan_finds_the_green():
    """The indicators are seen only from 5 m: the zigzag has to find the green."""
    items = POLES + CHANNEL + BLACKS + FIELD
    run, track, every = run_task1(0.0, seconds=600.0, items=items, task_order=('task2',),
                                  hints={'task2': (21.1, -1.5)}, until='RETURN',
                                  see_m={'indicator': 5.0})
    assert run.phase == 'RETURN', run.events[-3:]
    field = run.memory['task2_debris']
    assert field['scan_s'] > 0.0
    assert field['green']['done'] and field['green']['swept_deg'] >= 360.0, field
    for b in every[12:-2]:
        assert min(math.hypot(p[0] - b['east'], p[1] - b['north']) for p in track) > 0.6, b
    assert run.memory['task2']['back_gates'] == 4


def test_task2_debris_scan_far_green():
    """Find the green by the scan where roboboat_2026_far puts it, 14.8 m past the channel."""
    green = ('indicator', 1.50 - 2.05, 56.06, 'green')
    field = [it for it in FIELD if it[-1] != 'green'] + [green]
    run, track, every = run_task1(0.0, seconds=700.0, items=POLES + CHANNEL + BLACKS + field,
                                  task_order=('task2',), hints={'task2': (21.1, -1.5)},
                                  until='RETURN')
    assert run.phase == 'RETURN', run.events[-3:]
    result = run.memory['task2_debris']
    assert result['scan_s'] > 10.0 and result['red'] is not None
    assert result['green']['done'] and result['green']['swept_deg'] >= 360.0, result
    for b in every[12:-2]:
        assert min(math.hypot(p[0] - b['east'], p[1] - b['north']) for p in track) > 0.6, b


def test_task2_back_through_the_gates_passed_not_their_copies():
    """
    Go back through the 4 gates passed going in, not the copies the GPS drift makes.

    Past the channel the GPS drifts 2.8 m north: the gates seen again are mapped again,
    2.8 m on, and the course model sees a longer channel.
    """
    run, track, every = run_task1(0.0, seconds=500.0, items=POLES + CHANNEL + BLACKS,
                                  task_order=('task2',), hints={'task2': (21.1, -1.5)},
                                  until='RETURN', drift=(0.0, 2.8), drift_north=44.0,
                                  camera=True)
    assert run.phase == 'RETURN', run.events[-3:]
    assert run.memory['task2']['back_gates'] == 4, run.memory['task2']
    for k in range(4):
        xs = crossings(track, every[4 + 2 * k], every[5 + 2 * k])
        assert [d for d, _, _ in xs] == ['in', 'out'], (k, xs)
        assert all(abs(side) < w / 2.0 - 0.49 for _, side, w in xs), (k, xs)
