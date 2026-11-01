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


def landmarks(turn_deg):
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    return [{'id': i, 'kind': k, 'color': '', 'east': x * c - y * s, 'north': x * s + y * c,
             'confirmed': True, 'sightings': 30} for i, (k, x, y) in enumerate(POLES)]


def run_task1(turn_deg=0.0, seconds=200.0, dt=0.04):
    """Start 8 m before the first gate, facing it; course drawn heading north, turned."""
    every = landmarks(turn_deg)
    c, s = math.cos(math.radians(turn_deg)), math.sin(math.radians(turn_deg))
    boat = (0.0 * c - (-8.0) * s, 0.0 * s + (-8.0) * c)
    heading = (0.0 - turn_deg) % 360.0                 # north, turned counter-clockwise
    run = RoboBoatRun(RunConfig(task_order=()), default_phases(CFG))
    model = CourseModel()
    ctx = RunContext(now=0.0, boat_xy=boat, heading=heading)
    seen, track = set(), [boat]
    while ctx.now < seconds and run.phase != FINISHED:
        for lm in every:
            if math.hypot(lm['east'] - ctx.boat_xy[0], lm['north'] - ctx.boat_xy[1]) <= 12.0:
                seen.add(lm['id'])
        ctx.landmarks = [lm for lm in every if lm['id'] in seen]
        ctx.course = model.view(ctx.landmarks, run.memory.get('start_xy') or boat,
                                run.memory.get('t1_heading'))
        cmd = run.step(ctx, True)
        fwd, yaw = cmd or (1500, 0)
        speed = max(0.0, (fwd - 1500) / 180.0 * 1.3)
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
