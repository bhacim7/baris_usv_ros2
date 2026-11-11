"""
usv_core.mission_tasks: rally, hold and home (step 6c), on a rough boat model.

The boat: speed from the forward PWM, turn rate from the yaw PWM, as in
test_roboboat_drive; a drift (m/s, east / north) stands for wind and current.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_mission_tasks.py -v
"""

import math

from usv_core.mission import competition_factories, HOME, MissionConfig, MissionManager, Request
from usv_core.mission_tasks import factories, Hold, Rally, Refused
from usv_core.nav_config import NavConfig
from usv_core.roboboat_run import DONE, RunContext, RUNNING, SKIPPED
from usv_core.world_map import latlon

CFG = NavConfig()
ORIGIN = (27.37, -82.45)


def boat_speed(fwd):
    return (fwd - 1500) / 180.0 * 1.3 if fwd >= 1500 else (fwd - 1500) / 150.0 * 0.7


def sail(step, ctx, seconds, dt=0.04, drift=(0.0, 0.0)):
    """Run step(ctx) -> (state, cmd) on the rough boat; returns the states and the track."""
    states, track = [], []
    t_end = ctx.now + seconds
    while ctx.now < t_end:
        state, cmd = step(ctx)
        states.append(state)
        if state != RUNNING:
            break
        fwd, yaw = cmd or (1500, 0)
        rate = max(-60.0, min(60.0, yaw * 0.25))
        ctx.heading = (ctx.heading + rate * dt) % 360.0
        v = boat_speed(fwd)
        h = math.radians(ctx.heading)
        ctx.boat_xy = (ctx.boat_xy[0] + (v * math.sin(h) + drift[0]) * dt,
                       ctx.boat_xy[1] + (v * math.cos(h) + drift[1]) * dt)
        ctx.speed = v
        ctx.now += dt
        track.append(ctx.boat_xy)
    return states, track


def ctx_at(xy, heading=0.0):
    return RunContext(now=0.0, boat_xy=xy, heading=heading, map_origin=ORIGIN, memory={})


def test_rally_goes_to_the_point_and_keeps_station_against_a_drift():
    lat, lon = latlon(30.0, 20.0, *ORIGIN)              # 30 m east, 20 m north
    task = Rally(CFG, lat, lon)
    ctx = ctx_at((0.0, 0.0))
    task.start(ctx)
    states, track = sail(task.step, ctx, 120.0, drift=(0.0, -0.15))
    assert all(s == RUNNING for s in states)             # holds until told otherwise
    assert 'arrived_at' in task.result
    late = track[len(track) // 2:]
    assert max(math.dist(p, (30.0, 20.0)) for p in late) < 4.5
    assert task.result['returns'] >= 1                   # the drift took it out, it came back


def test_rally_with_hold_time_is_done_after_it():
    lat, lon = latlon(10.0, 0.0, *ORIGIN)
    task = Rally(CFG, lat, lon, hold_s=5.0)
    ctx = ctx_at((0.0, 0.0), heading=90.0)
    task.start(ctx)
    states, _ = sail(task.step, ctx, 60.0)
    assert states[-1] == DONE


def test_rally_waits_for_the_map_origin():
    lat, lon = latlon(10.0, 0.0, *ORIGIN)
    task = Rally(CFG, lat, lon)
    ctx = RunContext(now=0.0, boat_xy=None, heading=0.0, memory={})
    task.start(ctx)
    assert task.step(ctx) == (RUNNING, None)


def test_hold_keeps_station_where_it_started():
    task = Hold(CFG)
    ctx = ctx_at((5.0, 5.0))
    task.start(ctx)
    _, track = sail(task.step, ctx, 90.0, drift=(0.1, 0.0))
    assert max(math.dist(p, (5.0, 5.0)) for p in track) < 4.5


def test_wrong_rally_parameters_are_skipped_saying_why():
    written, _ = factories(CFG)
    t = written['rally']({'lat': 'x'})
    assert isinstance(t, Refused) and t.step(None) == (SKIPPED, None)
    assert t.summary(None) == 'rally needs lat and lon'


def test_mission_goes_home_after_an_urgent_rally():
    written, home = factories(CFG)
    m = MissionManager(MissionConfig(plan=('task1',)), competition_factories(written),
                       home=home)
    ctx = ctx_at((0.0, 0.0))
    m.step(ctx, True)                                     # task1 skipped, home at once
    assert m.phase == HOME
    lat, lon = latlon(15.0, 0.0, *ORIGIN)
    m.request(Request('rally', urgent=True, params={'lat': lat, 'lon': lon, 'hold_s': 3.0}),
              now=ctx.now)

    def step(c):
        return RUNNING, m.step(c, True)
    sail(step, ctx, 80.0)
    assert m.phase == HOME                                # rally done, home again
    _, track = sail(step, ctx, 60.0)
    assert math.dist(track[-1], (0.0, 0.0)) < 4.5         # back where the run started
