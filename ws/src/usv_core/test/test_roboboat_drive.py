"""
RoboBoat moves (usv_core.roboboat_drive), driven on a rough boat model.

The boat: speed from the forward PWM (1680 = 1.3 m/s), turn rate from the yaw PWM
(200 = 50 deg/s, clipped at 60), as in test_survey. Each test drives a move to its end
and checks what came out of it: got there, went between the poles, went round the right
way, found what was hidden.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_roboboat_drive.py -v
"""

import math

import pytest
from usv_core.nav_config import NavConfig
from usv_core.roboboat_course import Gate
from usv_core.roboboat_drive import Circle, DriveTo, PassGate, Search
from usv_core.roboboat_run import DONE, RunContext, RUNNING, SKIPPED
from usv_core.world_map import latlon

CFG = NavConfig()
ORIGIN = (27.37, -82.45)


def lm(i, kind, east, north):
    return {'id': i, 'kind': kind, 'color': '', 'east': east, 'north': north,
            'confirmed': True, 'sightings': 30}


def drive(move, boat, heading, seconds=120.0, dt=0.04, landmarks=(), seen=None):
    """Drive a move on the rough boat; seen(t, xy) -> detections dicts for that frame."""
    ctx = RunContext(now=0.0, boat_xy=boat, heading=heading, speed=0.0,
                     landmarks=list(landmarks), map_origin=ORIGIN)
    move.start(ctx)
    track, state = [boat], RUNNING
    while ctx.now < seconds:
        if seen is not None:
            ctx.detections, ctx.detections_time = seen(ctx.now, ctx.boat_xy), ctx.now
        state, cmd = move.step(ctx)
        if state != RUNNING:
            break
        fwd, yaw = cmd or (1500, 0)
        speed = max(0.0, (fwd - 1500) / 180.0 * 1.3)
        rate = max(-60.0, min(60.0, yaw * 0.25))
        h = (ctx.heading + rate * dt) % 360.0
        e = ctx.boat_xy[0] + speed * math.sin(math.radians(h)) * dt
        n = ctx.boat_xy[1] + speed * math.cos(math.radians(h)) * dt
        ctx.heading, ctx.speed, ctx.boat_xy = h, speed, (e, n)
        ctx.now += dt
        track.append(ctx.boat_xy)
    return state, track, ctx.now


def detection(name, xy):
    lat, lon = latlon(xy[0], xy[1], *ORIGIN)
    return {'class_name': name, 'position_ok': True, 'lat': lat, 'lon': lon}


def test_drive_to_a_point_in_the_open():
    state, track, t = drive(DriveTo(CFG, (15.0, 20.0), avoid=False), (0.0, 0.0), 0.0)
    assert state == DONE and math.hypot(track[-1][0] - 15.0, track[-1][1] - 20.0) <= 1.5
    assert t < 40.0                                       # ~25 m at ~1.3 m/s, plus the turn


def test_drive_to_goes_round_a_wall_of_buoys():
    pytest.importorskip('cv2')
    wall = [lm(i, 'buoy_black', -3.0 + i * 0.5, 10.0) for i in range(13)]   # x -3 .. 3
    state, track, _ = drive(DriveTo(CFG, (0.0, 20.0)), (0.0, 0.0), 0.0, landmarks=wall)
    assert state == DONE
    closest = min(math.hypot(p[0] - b['east'], p[1] - b['north']) for p in track for b in wall)
    assert closest > 0.5


def gate(red, green):
    return Gate(lm(1, 'pole_red', *red), lm(2, 'pole_green', *green))


def test_pass_gate_goes_between_the_poles():
    g = gate((-4.3, 10.0), (4.3, 10.0))                   # through = north, red on the west
    move = PassGate(CFG, g)
    state, track, _ = drive(move, (3.0, 0.0), 40.0)
    assert state == DONE and move.result['through']
    assert abs(move.result['crossed_at']) < 1.0          # near the middle of 8.6 m
    assert track[-1][1] >= 12.0                           # 2 m past the gate line


def test_pass_gate_the_other_way():
    move = PassGate(CFG, gate((-4.3, 10.0), (4.3, 10.0)), reverse=True)
    state, track, _ = drive(move, (-2.0, 22.0), 180.0)
    assert state == DONE and move.result['through'] and track[-1][1] <= 8.0


def test_pass_gate_trusts_the_camera_over_the_map():
    """The map has the gate 1.5 m east of where it is; the camera sees it right."""
    true_red, true_green = (-1.25, 10.0), (1.25, 10.0)    # a 2.5 m channel gate
    g = Gate(lm(1, 'buoy_red', true_red[0] + 1.5, 10.0),
             lm(2, 'buoy_green', true_green[0] + 1.5, 10.0))
    move = PassGate(CFG, g)

    def seen(t, boat):
        return [detection('buoy_red', true_red), detection('buoy_green', true_green)] \
            if boat[1] < 10.0 else []

    state, track, _ = drive(move, (0.0, 0.0), 0.0, seen=seen)
    assert state == DONE and move.result['camera'] > 10
    crossing = min(track, key=lambda p: abs(p[1] - 10.0))
    assert abs(crossing[0]) < 0.6                         # true centre, not the map's 1.5


@pytest.mark.parametrize('clockwise', [True, False])
def test_circle_goes_round_the_right_way(clockwise):
    move = Circle(CFG, (0.0, 10.0), 'buoy_yellow', radius_m=2.5, clockwise=clockwise)
    state, track, _ = drive(move, (0.0, 0.0), 0.0)
    assert state == DONE and move.result['swept_deg'] >= 360.0
    # Signed area of the loop: negative = clockwise (east / north axes)
    loop = [p for p in track if math.hypot(p[0], p[1] - 10.0) < 5.0]
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(loop, loop[1:]))
    assert (area < 0) == clockwise
    radii = [math.hypot(p[0], p[1] - 10.0) for p in loop[len(loop) // 4:]]
    assert 1.2 < min(radii) and max(radii) < 4.5


def test_search_finds_what_is_hidden():
    hidden = (6.0, 9.0)

    def found(ctx):
        return math.hypot(ctx.boat_xy[0] - hidden[0], ctx.boat_xy[1] - hidden[1]) < 4.0

    state, track, t = drive(Search(CFG, (0.0, 6.0), found), (0.0, 0.0), 0.0, seconds=300.0)
    assert state == DONE and t < 300.0


def test_search_gives_up_when_nothing_is_there():
    state, _, t = drive(Search(CFG, (0.0, 6.0), lambda ctx: False, max_m=8.0), (0.0, 0.0),
                        0.0, seconds=600.0)
    assert state == SKIPPED and t < 600.0
