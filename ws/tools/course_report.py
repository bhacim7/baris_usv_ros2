#!/usr/bin/env python3
"""
What the course model makes of a recorded map (phase 6 R2.2).

Usage (inside the container, after `source /ws/install/setup.bash`):
    python3 /ws/tools/course_report.py BAG_DIR | MAP.json [--course NAME]
                                       [--course-file PATH] [--gps-true]

usv_core.roboboat_course.CourseModel runs twice: on the course file's TRUE object
positions (what it should find) and on the map (what it does find), and the two are
compared element by element: Task 1 gates and the gate frame, the channel, the debris
field, Task 3, the marina slips and the Task 6 zones; then the course file's hints
(mission section) read in the map's gate frame, against where those tasks really are.

The map is the mapper's last /usv/map of a bag, or a map JSON the mapper wrote on
shutdown (/ws/bags/map_<course>.json). With --gps-true (bags only) the sightings are
mapped again with the GPS swapped for the truth, as map_report.py's map 3c: the course
model on a map with only the measurement chain's error.
"""

import argparse
import json
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from usv_core.roboboat_course import CourseModel
from usv_core.world_map import kind_of
from usv_sim import course as course_mod
from usv_sim.geo import enu
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_report  # noqa: E402

HINT_TARGETS = {     # what a hint points at, in the true course view
    'task2': lambda v: v.channel[0].centre if v.channel else None,
    'task3': lambda v: v.task3_gate.centre if v.task3_gate else None,
    'task5': lambda v: _mean([_xy(s.banner) for s in v.slips]) if v.slips else None,
    'task6_zone_t3': lambda v: _xy(v.zones['task6_zone_t3'])
    if 'task6_zone_t3' in v.zones else None,
    'task6_zone_marina': lambda v: _xy(v.zones['task6_zone_marina'])
    if 'task6_zone_marina' in v.zones else None,
}


def _xy(lm):
    return (lm['east'], lm['north'])


def _mean(points):
    return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_course(name, path):
    path = path or os.path.join(get_package_share_directory('usv_sim'), 'courses',
                                name + '.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return course_mod.load(data), data.get('mission', {})


def truth_landmarks(course):
    out = []
    for i, t in enumerate(course.targets):
        name = course.class_names[t.class_id]
        out.append({'id': i, 'kind': kind_of(name),
                    'color': name.split('_')[1] if name.startswith('indicator') else '',
                    'east': t.x, 'north': t.y, 'confirmed': True, 'sightings': 100})
    return out


def map_landmarks(source, course, gps_true, course_file):
    origin = course.origin
    if source.endswith('.json'):
        with open(source) as f:
            raw = json.load(f)['landmarks']
        out = []
        for lm in raw:
            e, n = enu(lm['lat'], lm['lon'], origin['lat'], origin['lon'])
            out.append({'id': lm['id'], 'kind': lm['kind'], 'color': lm['color'], 'east': e,
                        'north': n, 'confirmed': lm['confirmed'],
                        'sightings': lm['sightings']})
        return out, 'map JSON written by the mapper'
    data = map_report.read_bag(source)
    if gps_true:
        setup = map_report.load_setup(course.name, course_file)
        objects = map_report.truths(setup.course)
        _, every, _, _, _ = map_report.sightings(data, setup, objects, setup.map_cfg.max_range_m)
        return (map_report.rebuild_map(every, setup, ('gps',), setup.map_cfg),
                'sightings mapped again with the GPS true (map_report 3c)')
    if not data['/usv/map']:
        sys.exit('bag has no /usv/map')
    return map_report.mapper_landmarks(data, map_report.load_setup(course.name, course_file)), \
        "the mapper's last /usv/map"


def show_point(label, found, truth):
    if truth is None and found is None:
        return
    if found is None:
        print(f'   {label:<26} NOT FOUND')
    elif truth is None:
        print(f'   {label:<26} found, but not in the true course')
    else:
        print(f'   {label:<26} {_dist(found, truth):5.2f} m off')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('source', help='bag directory or map JSON')
    ap.add_argument('--course', default='roboboat_2026')
    ap.add_argument('--course-file', default='')
    ap.add_argument('--gps-true', action='store_true')
    args = ap.parse_args()

    course, mission = load_course(args.course, args.course_file)
    start = (course.boat['east'], course.boat['north'])
    model = CourseModel()
    want = model.view(truth_landmarks(course), start)
    lms, what = map_landmarks(args.source, course, args.gps_true, args.course_file)
    got = model.view(lms, start)
    print(f'== {args.source}: {what}, {len(lms)} landmarks '
          f'({sum(1 for lm in lms if lm["confirmed"])} confirmed)')
    if want.missing:
        print(f'   (on the true course the model misses: {want.missing})')

    print('\n-- Task 1 gates and the gate frame')
    print(f'   gates found: {len(got.t1_gates)} of {len(want.t1_gates)}')
    for k, (g, w) in enumerate(zip(got.t1_gates, want.t1_gates)):
        show_point(f'gate {k + 1} centre', g.centre, w.centre)
    if got.frame and want.frame:
        dot = max(-1.0, min(1.0, got.frame.ahead[0] * want.frame.ahead[0]
                            + got.frame.ahead[1] * want.frame.ahead[1]))
        print(f'   frame direction            {math.degrees(math.acos(dot)):5.1f} deg off')

    print('\n-- Task 2')
    print(f'   channel gates: {len(got.channel)} of {len(want.channel)}')
    for k, (g, w) in enumerate(zip(got.channel, want.channel)):
        show_point(f'channel gate {k + 1}', g.centre, w.centre)
    print(f'   debris field: {len(got.debris)} landmarks (true course: {len(want.debris)}); '
          f"indicators {sorted(lm['color'] for lm in got.debris if lm['kind'] == 'indicator')}"
          f" (true {sorted(lm['color'] for lm in want.debris if lm['kind'] == 'indicator')})")

    print('\n-- Task 3')
    show_point('gate centre', got.task3_gate.centre if got.task3_gate else None,
               want.task3_gate.centre if want.task3_gate else None)
    show_point('yellow buoy', _xy(got.task3_yellow) if got.task3_yellow else None,
               _xy(want.task3_yellow) if want.task3_yellow else None)
    print(f'   indicator colour: {got.task3_indicator or "-"} (true {want.task3_indicator})')

    print('\n-- Task 5 marina')
    true_slips = {(s.dock, s.number): s for s in want.slips}
    for s in got.slips:
        # The true slip is the nearest true banner with the same number (the dock may not
        # be named yet when only one dock has been seen)
        same = [t for t in want.slips if t.number == s.number]
        t = min(same, key=lambda t: _dist(_xy(t.banner), _xy(s.banner))) if same else None
        if t is not None and _dist(_xy(t.banner), _xy(s.banner)) > 5.0:
            t = None
        flag = '' if t and (t.indicator, t.occupied) == (s.indicator, s.occupied) else \
            f'   <- true: {t.indicator or "-"}{", occupied" if t and t.occupied else ""}' \
            if t else '   <- not in the true course'
        print(f'   dock {s.dock or "?"} slip {s.number}: indicator {s.indicator or "-":<6} '
              f'{"occupied" if s.occupied else "free":<9} '
              f'{_dist(_xy(s.banner), _xy(t.banner)) if t else float("nan"):5.2f} m off{flag}')
    found = set()
    for s in got.slips:
        same = [t for t in want.slips if t.number == s.number]
        if same:
            t = min(same, key=lambda t: _dist(_xy(t.banner), _xy(s.banner)))
            found.add((t.dock, t.number))
    missing = sorted(set(true_slips) - found)
    if missing:
        print(f'   slips not found: {missing}')
    b, wb, sf = got.best_slip, want.best_slip, got.best_slip_so_far
    so_far = '' if b or not sf else f', so far {sf.dock or "?"}{sf.number}: marina not complete'
    print(f'   best slip: {f"{b.dock}{b.number}" if b else "-"}{so_far} '
          f'(true {f"{wb.dock}{wb.number}" if wb else "-"})')

    print('\n-- Task 6 zones')
    for name in ('task6_zone_t3', 'task6_zone_marina'):
        show_point(name, _xy(got.zones[name]) if name in got.zones else None,
                   _xy(want.zones[name]) if name in want.zones else None)

    hints = {k.split('roboboat.hint.')[1]: v for k, v in mission.items()
             if k.startswith('roboboat.hint.') and v}
    if hints:
        print('\n-- hints of the course file, read in this map\'s gate frame')
        for name, text in hints.items():
            hint = tuple(float(x) for x in str(text).split(','))
            target = HINT_TARGETS.get(name, lambda v: None)(want)
            show_point(f'{name} ({text})', got.expected(hint), target)
    print(f'\n   still missing on this map: {got.missing or "nothing"}')


if __name__ == '__main__':
    main()
