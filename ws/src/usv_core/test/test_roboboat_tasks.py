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

CFG = NavConfig()
POLES = [('pole_red', -4.33, 0.0), ('pole_green', 4.33, 0.0),
         ('pole_red', -4.33, 7.95), ('pole_green', 4.33, 7.95)]


# The roboboat_2026 channel, from the first Task 1 gate's centre: 2.5 m gates, 6 m apart
CHANNEL = []
for cx, y in ((1.52, 21.13), (2.12, 27.13), (2.72, 33.13), (3.72, 39.13)):
    CHANNEL += [('buoy_red', cx - 1.25, y), ('buoy_green', cx + 1.25, y)]
BLACKS = [('buoy_black', 2.07, 33.33), ('buoy_black', 3.07, 39.43)]   # inside gates 3 and 4
OFF_LINE = [('buoy_black', 1.82, 24.13)]      # between gates 1 and 2, on the way
DEBRIS = [('buoy_black', 4.30, 45.13)]        # 6 m past gate 4, on the channel's line


def landmarks(turn_deg, items=POLES):
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    return [{'id': i, 'kind': k, 'color': '', 'east': x * c - y * s, 'north': x * s + y * c,
             'confirmed': True, 'sightings': 30} for i, (k, x, y) in enumerate(items)]


def run_task1(turn_deg=0.0, seconds=200.0, dt=0.04, items=POLES, task_order=(),
              hints=None, until=FINISHED):
    """Start 8 m before the first gate, facing it; course drawn heading north, turned."""
    every = landmarks(turn_deg, items)
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    boat = (0.0 * c - (-8.0) * s, 0.0 * s + (-8.0) * c)
    heading = (0.0 - turn_deg) % 360.0                 # north, turned counter-clockwise
    run = RoboBoatRun(RunConfig(task_order=task_order), default_phases(CFG, hints=hints))
    model = CourseModel()
    ctx = RunContext(now=0.0, boat_xy=boat, heading=heading)
    seen, track = set(), [boat]
    while ctx.now < seconds and run.phase != until:
        for lm in every:
            if math.hypot(lm['east'] - ctx.boat_xy[0], lm['north'] - ctx.boat_xy[1]) <= 12.0:
                seen.add(lm['id'])
        ctx.landmarks = [lm for lm in every if lm['id'] in seen]
        ctx.course = model.view(ctx.landmarks, run.memory.get('start_xy') or boat,
                                run.memory.get('t1_heading'))
        cmd = run.step(ctx, True)
        fwd, yaw = cmd or (1500, 0)
        speed = (fwd - 1500) / 180.0 * 1.3 if fwd >= 1500 else (fwd - 1500) / 150.0 * 0.7
        h = (ctx.heading + max(-60.0, min(60.0, yaw * 0.25)) * dt) % 360.0
        ctx.boat_xy = (ctx.boat_xy[0] + speed * math.sin(math.radians(h)) * dt,
                       ctx.boat_xy[1] + speed * math.cos(math.radians(h)) * dt)
        ctx.heading, ctx.speed = h, speed
        ctx.now += dt
        track.append(ctx.boat_xy)
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
