#!/usr/bin/env python3
"""
Replay a recorded RoboBoat run through the navigator: what the boat thought, when.

The bag holds everything the navigator node was given: the GPS fix, the heading, the
speed, the world map (/usv/map) and the camera frames (/usv/detections). Fed again, in
order, to usv_core's RoboBoatNavigator (the code as it is now, not as it was), it makes
the same decisions, and every move, path, gate and turn can be looked at at any moment.
Used to find why a run went wrong (phase 6 R3): the Circle that went round twice, the
gate copies on the way back.

Open loop: the boat in the bag goes where it went. With the code as recorded the replay
matches the run; with changed code it shows what the new code would have decided in the
same place, but not where the boat would then have gone.

Runs on the host with plain python3 (no ROS; numpy and OpenCV stood in for if missing)
and in the container. Times are seconds from the run's start (leaving WAIT_START), as in
roboboat_report.py.

    python3 ws/tools/dev/replay.py BAG                    # the navigator's log
    python3 ws/tools/dev/replay.py BAG moves              # every move, when, and its result
    python3 ws/tools/dev/replay.py BAG trace 60 90        # the move each second from 60 to 90 s
        [--every S] [--path]                              #   (--path: the planned path too)
    python3 ws/tools/dev/replay.py BAG channel 70 150     # the course model's channel then
    python3 ws/tools/dev/replay.py BAG map [--map KEY=VALUE ...] [--course NAME]
        # the world map built again from the camera frames with mapper.yaml's settings (each
        # --map changes one, e.g. copy_merge_radius_m=3.0), against the course's truth

BAG: a name under ws/bags (t3d_s1) or a bag folder's path.
"""

import argparse
import math
import os
import sys
import types

import bag_read

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(os.path.dirname(HERE))
INPUTS = {'/usv/fix', '/usv/heading', '/usv/speed', '/usv/map', '/usv/detections',
          '/usv/nav_status'}


def _import_usv():
    """Put the workspace's packages on the path; stand in for numpy / cv2 if missing."""
    for pkg in ('usv_core', 'usv_sim', 'usv_nodes'):
        path = os.path.join(WS, 'src', pkg)
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        import numpy  # noqa: F401
    except ImportError:
        np = types.ModuleType('numpy')                  # the mixer's clip is all it needs
        np.clip = lambda v, lo, hi: max(lo, min(hi, v))
        sys.modules['numpy'] = np
    try:
        import cv2  # noqa: F401
    except ImportError:
        sys.modules['cv2'] = types.ModuleType('cv2')     # the grid planner, unused here


def bag_file(name):
    folder = name if os.path.isdir(name) else os.path.join(WS, 'bags', name)
    files = [f for f in sorted(os.listdir(folder)) if f.endswith('.mcap')]
    if not files:
        sys.exit(f'{folder}: no .mcap file')
    return os.path.join(folder, files[0])


def _r(p):
    return None if p is None else (round(p[0], 2), round(p[1], 2))


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Replay:
    """The navigator fed a bag's inputs at the node's 25 Hz; hooks see every cycle."""

    def __init__(self, bag, hints=None, dt=0.04):
        _import_usv()
        from usv_core.roboboat_navigator import Perception, RoboBoatNavigator
        from usv_core.roboboat_run import RunConfig
        import usv_core.roboboat_run as run_mod

        self.msgs = bag_read.read(bag_file(bag), INPUTS)
        starts = [t for t, tp, d in self.msgs
                  if tp == '/usv/nav_status' and bag_read.status(d) not in ('WAIT_START', '')]
        if not starts:
            sys.exit('no run start in /usv/nav_status (never left WAIT_START)')
        self.t0, self.dt, self.logs = starts[0], dt, []
        self.clock = 0.0
        self.ctx = None
        original = run_mod.RoboBoatRun.step

        def step(run, ctx, active):                     # keep the cycle's RunContext
            self.ctx = ctx
            return original(run, ctx, active)
        run_mod.RoboBoatRun.step = step
        hints = {'task2': (21.1, -1.5)} if hints is None else hints
        self.nav = RoboBoatNavigator(
            run_cfg=RunConfig(task_order=('task2', 'task3', 'task5'), hints=hints),
            log=lambda m: self.logs.append((self.clock - self.t0, m)))
        self.perception = Perception()

    def run(self, hook=None, until=None):
        """Replay to the end (or until s from the start); hook(t, nav, ctx) each cycle."""
        per, gps, heading, speed = self.perception, (0.0, 0.0), 0.0, 0.0
        i, t = 0, self.msgs[0][0]
        end = self.msgs[-1][0] if until is None else self.t0 + until
        while t < end:
            while i < len(self.msgs) and self.msgs[i][0] <= t:
                _, topic, data = self.msgs[i]
                i += 1
                if topic == '/usv/fix':
                    gps = bag_read.fix(data)
                elif topic == '/usv/heading':
                    heading = bag_read.float32(data)
                elif topic == '/usv/speed':
                    speed = bag_read.float32(data)
                elif topic == '/usv/map':
                    per.origin, per.landmarks = bag_read.landmarks(data)
                elif topic == '/usv/detections':
                    per.detections_time, per.detections = bag_read.detections(data)
            self.clock = t
            self.nav.step(t, gps, speed, heading, per)
            if hook is not None and self.ctx is not None:
                hook(t - self.t0, self.nav, self.ctx)
            t += self.dt


def current_move(nav):
    """Return (phase object, its stage, the move running now)."""
    phase = nav.run.phases.get(nav.run.phase)
    stage = getattr(phase, 'stage', '')
    move = phase.field.move if stage == 'debris' else getattr(phase, 'move', None)
    return phase, stage, move


def show_log(args):
    replay = Replay(args.bag)
    replay.run()
    for t, m in replay.logs:
        if args.all or not ('marina' in m or 'slip' in m):
            print(f'{t:7.1f}  {m}')


def show_moves(args):
    replay = Replay(args.bag)
    moves = {}

    def hook(t, nav, ctx):
        _, stage, move = current_move(nav)
        if move is None:
            return
        entry = moves.setdefault(id(move), [t, t, nav.run.phase, stage, move])
        entry[1] = t
    replay.run(hook)
    for start, end, phase, stage, move in sorted(moves.values(), key=lambda m: m[0]):
        where = f'{phase}/{stage}' if stage else phase
        print(f'{start:7.1f}-{end:7.1f}  {where:<14} {type(move).__name__:<9} {move.result}')


def _trace_line(t, nav, ctx, with_path):
    from usv_core import roboboat_drive as drive
    _, stage, move = current_move(nav)
    cmd = nav.mixer.last
    here = move.here(ctx) if move is not None and ctx.boat_xy else ctx.boat_xy
    where = f'{nav.run.phase}/{stage}' if stage else nav.run.phase
    line = (f'{t:7.1f}  {where:<14} {type(move).__name__:<9} here={_r(here)} '
            f'hdg={ctx.heading:4.0f} pwm L{cmd.rear_left} R{cmd.rear_right}')
    if isinstance(move, (drive.PassGate, drive.DriveTo, drive.Circle)):
        line += f' spot={move.follower.spot_turn} stuck={move.stuck}'
    if isinstance(move, drive.PassGate):
        line += (f' along={move.along and round(move.along, 2)} gap={move.offset:+.2f}'
                 f' width={_dist(move.red, move.green):.2f} bent={move.bent}')
    elif isinstance(move, drive.Circle):
        line += (f' target={_r(move.target)} swept={move.swept:.0f} dist='
                 f'{move.dist and round(move.dist, 2)} on_ring={move.on_ring}')
    elif isinstance(move, drive.DriveTo):
        line += f' goal={_r(move.goal)}'
    if here is not None:
        near = sorted(((_dist((lm['east'], lm['north']), here), lm['id'], lm)
                       for lm in ctx.landmarks if lm['confirmed']), key=lambda x: x[0])[:2]
        line += ' near=' + ', '.join(f"{lm['kind']}{_r((lm['east'], lm['north']))} {d:.2f}"
                                     for d, _, lm in near)
    if with_path and move is not None and getattr(move, 'path', None):
        line += '\n           path=' + str([_r(p) for p in move.path[:8]])
    return line


def show_trace(args):
    replay = Replay(args.bag)
    last = [-1e9]

    def hook(t, nav, ctx):
        if args.start <= t <= args.end and t - last[0] >= args.every:
            last[0] = t
            print(_trace_line(t, nav, ctx, args.path))
    replay.run(hook, until=args.end + replay.dt)


def show_channel(args):
    replay = Replay(args.bag)
    todo = sorted(args.times)

    def hook(t, nav, ctx):
        if todo and t >= todo[0]:
            todo.pop(0)
            gates = nav.course.channel if nav.course is not None else []
            print(f'{t:7.1f} s: {len(gates)} channel gates, boat {_r(ctx.boat_xy)}')
            for g in gates:
                red, green = g.red, g.green
                print(f'   centre {_r(g.centre)} width {g.width:.2f}  '
                      f"red #{red['id']} x{red['sightings']} {_r((red['east'], red['north']))}  "
                      f"green #{green['id']} x{green['sightings']} "
                      f"{_r((green['east'], green['north']))}")
    replay.run(hook, until=max(args.times) + replay.dt)


def _map_settings(changes):
    import yaml
    from usv_core.world_map import MapConfig
    with open(os.path.join(WS, 'src', 'usv_nodes', 'config', 'mapper.yaml')) as f:
        params = yaml.safe_load(f)['mapper']['ros__parameters']
    fields = MapConfig.__dataclass_fields__
    values = {k: v for k, v in params.items() if k in fields}
    for change in changes:
        key, _, value = change.partition('=')
        if key not in fields:
            sys.exit(f'--map: no setting {key}; have {", ".join(fields)}')
        values[key] = type(getattr(MapConfig(), key))(float(value))
    return MapConfig(**values)


def show_map(args):
    """Build the world map again from the bag's camera frames and compare it with the truth."""
    _import_usv()
    import yaml
    from usv_core.world_map import enu, kind_of, WorldMap
    from usv_sim import course as course_mod

    cfg = _map_settings(args.map)
    world = WorldMap(cfg)
    rate = 0.0
    for _, topic, data in bag_read.read(bag_file(args.bag), {'/usv/detections', '/usv/imu'}):
        if topic == '/usv/imu':
            rate = math.degrees(bag_read.gyro_z(data))
        else:
            stamp, frame = bag_read.detections(data)
            world.add_frame(frame, stamp, rate)
    with open(os.path.join(WS, 'src', 'usv_sim', 'courses', args.course + '.yaml')) as f:
        course = course_mod.load(yaml.safe_load(f))
    shift = enu(*world.origin, course.origin['lat'], course.origin['lon'])
    mine = [dict(lm, east=lm['east'] + shift[0], north=lm['north'] + shift[1])
            for lm in world.snapshot() if lm['confirmed']]
    truth = []
    for t in course.targets:
        name = course.class_names[t.class_id]
        truth.append((name, kind_of(name), name.split('_')[1] if name.startswith('indicator')
                      else '', (t.x, t.y)))

    def same(lm, kind, color):
        return lm['kind'] == kind and (not color or lm['color'] == color)
    print(f'settings: {cfg}')
    print(f'{len(world.landmarks)} landmarks, {len(mine)} confirmed, '
          f'{getattr(world, "copies_merged", 0)} copies folded')
    twice, never = [], 0
    for name, kind, color, xy in truth:
        found = [lm for lm in mine if same(lm, kind, color)
                 and _dist((lm['east'], lm['north']), xy) <= args.near]
        if not found:
            never += 1
        elif len(found) > 1:
            twice.append((name, _r(xy), len(found)))
    false = [lm for lm in mine
             if not any(same(lm, kind, color) and _dist((lm['east'], lm['north']), xy)
                        <= args.near for _, kind, color, xy in truth)]
    print(f'objects mapped more than once (within {args.near} m): {len(twice)}')
    for name, xy, n in twice:
        print(f'   {name:<16} {xy}  x{n}')
    print(f'confirmed with no such object within {args.near} m: {len(false)}')
    for lm in false:
        print(f"   {lm['kind']} {lm['color']} {_r((lm['east'], lm['north']))} "
              f"x{lm['sightings']}")
    print(f'objects never mapped (most never seen in this run): {never}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('bag')
    sub = ap.add_subparsers(dest='what')
    log = sub.add_parser('log', help="the navigator's log (the default)")
    log.add_argument('--all', action='store_true', help='with the marina lines')
    sub.add_parser('moves', help='every move, when, and its result')
    tr = sub.add_parser('trace', help='the move running now, from START to END s')
    tr.add_argument('start', type=float)
    tr.add_argument('end', type=float)
    tr.add_argument('--every', type=float, default=1.0)
    tr.add_argument('--path', action='store_true')
    ch = sub.add_parser('channel', help="the course model's channel gates at the times")
    ch.add_argument('times', type=float, nargs='+')
    mp = sub.add_parser('map', help='the map built again from the frames, against the truth')
    mp.add_argument('--map', action='append', default=[], metavar='KEY=VALUE')
    mp.add_argument('--course', default='roboboat_2026')
    mp.add_argument('--near', type=float, default=2.5)
    args = ap.parse_args()
    if args.what in (None, 'log'):
        args.all = getattr(args, 'all', False)
        show_log(args)
    else:
        {'moves': show_moves, 'trace': show_trace, 'channel': show_channel,
         'map': show_map}[args.what](args)


if __name__ == '__main__':
    main()
