#!/usr/bin/env python3
"""
Report on one simulated RoboBoat run, from Gazebo's ground truth (phase 6 R2.4).

Usage (inside the container, after `source /ws/install/setup.bash`):
    python3 /ws/tools/roboboat_report.py BAG_DIR [--course NAME] [--course-file PATH]

Record with sim.launch.py course:=roboboat_2026 auto:=true record:=<name>. What the boat
believed does not count here; where it really was does (/usv/sim/navsat + /usv/sim/imu,
the hull outline as in run_report.py):

  phases     the run's phases from /usv/nav_status, with times from the run's start
  Task 1     every crossing of each Task 1 gate line: direction (in = the entry way, red
             on the left), where along the gate the hull centre crossed, and whether the
             WHOLE hull passed between the poles (handbook 3.2.1: "the entire ASV must
             pass through"); entry = gate 1 in, then gate 2 in; exit = gate 2 out, then
             gate 1 out
  contacts   the closest the hull came to every buoy; 0 m or less is a contact (the
             handbook's obstacle strike, -25 points each)
  steering   per phase: how much longer the true path was than a straight line, how
             often the steering command changed side, the true yaw rate, how far the
             heading the boat used (/usv/heading) was from the true one, and the GPS
             fix's steps: to tell weaving caused by the compass lag, the 2 Hz fix or the
             commands themselves

--plot OUT.png: the true track and the GPS fixes over the gates and buoys; true heading vs
the heading used; the steering command; each against time, phases marked. The map panel
shows the true objects as rings (the judge's view, from the course file) and, as filled
dots, the boat's OWN map at the end of the run (the last /usv/map: what it saw; faint =
not confirmed yet). The boat never reads the course file.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import course_report  # noqa: E402
import run_report  # noqa: E402

from rclpy.serialization import deserialize_message  # noqa: E402
import rosbag2_py  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from usv_core.roboboat_course import CourseModel  # noqa: E402
from usv_sim.geo import enu  # noqa: E402
from usv_sim.sensors import compass_deg, yaw_from_quaternion  # noqa: E402

EXTRA = ('/usv/fix', '/usv/heading', '/usv/map')


def read_extra(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    data = {t: [] for t in EXTRA}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in data:
            data[topic].append((t_ns * 1e-9, deserialize_message(raw, types[topic])))
    return data


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def steering(data, extra, track, changes, origin, t0):
    """Print, per phase: path / straight, steering side changes, yaw rate, heading error."""
    imu = [(t, math.degrees(m.angular_velocity.z), compass_deg(yaw_from_quaternion(
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)))
        for t, m in data['/usv/sim/imu']]
    pwm = [(t, m.steer - 1500) for t, m in data['/usv/thrusters/pwm']]
    used = extra['/usv/heading']
    fixes = [(t, enu(m.latitude, m.longitude, origin['lat'], origin['lon']))
             for t, m in extra['/usv/fix']]
    print('\n-- steering (true values; "used" = /usv/heading)')
    print('   phase         path/straight  side changes/min  |yaw rate|  |used-true hdg|')
    for k, (ts, task, manual, _) in enumerate(changes):
        te = changes[k + 1][0] if k + 1 < len(changes) else track[-1][0]
        if manual or te - ts < 2.0 or task in ('WAIT_START', 'FINISHED'):
            continue
        seg = [p for p in track if ts <= p[0] <= te]
        length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(seg, seg[1:]))
        net = math.hypot(seg[-1][1] - seg[0][1], seg[-1][2] - seg[0][2]) if seg else 0.0
        sides = [1 if v > 30 else -1 for t, v in pwm if ts <= t <= te and abs(v) > 30]
        flips = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
        rates = [abs(r) for t, r, _ in imu if ts <= t <= te]
        errs = []
        j = 0
        for t, m in used:
            if not ts <= t <= te:
                continue
            while j + 1 < len(imu) and imu[j + 1][0] <= t:
                j += 1
            errs.append(abs(wrap180(m.data - imu[j][2])))
        mean = (lambda v: sum(v) / len(v) if v else float('nan'))
        print(f'   {task:<12} {length / net if net > 0.5 else float("nan"):10.2f} '
              f'{flips / (te - ts) * 60.0:14.0f} {mean(rates):12.1f} {mean(errs):14.1f}')
    steps = [math.hypot(b[1][0] - a[1][0], b[1][1] - a[1][1]) for a, b in zip(fixes, fixes[1:])
             if b[0] >= t0]
    if steps:
        gaps = [b[0] - a[0] for a, b in zip(fixes, fixes[1:]) if b[0] >= t0]
        print(f'   GPS fix: every {sum(gaps) / len(gaps):.2f} s, mean step '
              f'{sum(steps) / len(steps):.2f} m (the boat sees its position jump by this)')
    return imu, pwm, fixes


def boat_map(extra, origin):
    """Return the boat's own landmarks at the end of the run, in course east / north."""
    if not extra['/usv/map']:
        return []
    _, last = extra['/usv/map'][-1]
    out = []
    for lm in last.landmarks:
        e, n = enu(lm.latitude, lm.longitude, origin['lat'], origin['lon'])
        out.append({'kind': lm.kind, 'color': lm.color, 'east': e, 'north': n,
                    'confirmed': lm.confirmed, 'sightings': lm.sightings})
    return out


def plot(path, track, fixes, imu, pwm, used, changes, view, buoys, t0, seen=()):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from usv_nodes.mapper import COLORS

    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_axes([0.04, 0.07, 0.48, 0.86])
    x0, y0 = track[0][1], track[0][2]
    phases = [(t, task) for t, task, manual, _ in changes]
    colors = {}
    palette = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']
    for k, (ts, task) in enumerate(phases):
        te = phases[k + 1][0] if k + 1 < len(phases) else track[-1][0]
        seg = [p for p in track if ts <= p[0] <= te]
        if len(seg) > 1:
            c = colors.setdefault(task, palette[len(colors) % len(palette)])
            ax.plot([p[1] - x0 for p in seg], [p[2] - y0 for p in seg], '-', color=c,
                    lw=1.5, label=task if task not in ax.get_legend_handles_labels()[1] else None)
    ax.plot([f[1][0] - x0 for f in fixes], [f[1][1] - y0 for f in fixes], '.', color='0.4',
            ms=3, label='GPS fixes')
    for g in view.t1_gates:
        for lm, c in ((g.red, 'red'), (g.green, 'green')):
            ax.plot(lm['east'] - x0, lm['north'] - y0, 'o', color=c, ms=8)
    for b in buoys:
        ax.plot(b['east'] - x0, b['north'] - y0, 'o', mfc='none', mec='0.5', ms=5)
    for lm in seen:
        rgb = COLORS.get(lm['color'] or lm['kind'], (0.5, 0.5, 0.5))
        ax.plot(lm['east'] - x0, lm['north'] - y0, 'o', color=rgb,
                alpha=1.0 if lm['confirmed'] else 0.3, ms=5 if lm['confirmed'] else 3)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='best')
    ax.set_title('rings: true objects (judge); filled dots: the boat\'s own map '
                 '(faint = unconfirmed); lines: true track by phase; grey: GPS fixes',
                 fontsize=9)
    ax.set_xlabel('east of the start, m')
    ax.set_ylabel('north of the start, m')

    ax2 = fig.add_axes([0.58, 0.55, 0.39, 0.38])
    ax2.plot([t - t0 for t, _, h in imu], [h for _, _, h in imu], '-', lw=1, label='true')
    ax2.plot([t - t0 for t, m in used], [m.data for _, m in used], '-', lw=1, label='used')
    ax2.set_ylabel('heading, deg')
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    ax3 = fig.add_axes([0.58, 0.08, 0.39, 0.38], sharex=ax2)
    ax3.plot([t - t0 for t, _ in pwm], [v for _, v in pwm], '-', lw=1, color='tab:purple')
    ax3.set_ylabel('steering command (PWM - 1500)')
    ax3.set_xlabel('time from the start, s')
    ax3.grid(alpha=0.3)
    for axx in (ax2, ax3):
        for ts, task in phases:
            axx.axvline(ts - t0, color='0.7', lw=0.8)
    for ts, task in phases:
        ax2.text(ts - t0, ax2.get_ylim()[1], task, fontsize=7, rotation=90, va='top')
    fig.savefig(path, dpi=110)
    print(f'\nplot: {path}')


def gate_crossings(track, gate, t0, marker_r=0.457 / 2.0):
    """Return [(t, 'in'/'out', offset_m, whole_hull_between)] for one gate line."""
    r = (gate.red['east'], gate.red['north'])
    g = (gate.green['east'], gate.green['north'])
    w = math.hypot(r[0] - g[0], r[1] - g[1])
    left = ((r[0] - g[0]) / w, (r[1] - g[1]) / w)
    through = (left[1], -left[0])
    c = ((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0)
    pole_r = marker_r
    out = []
    for (ta, ea, na, _), (tb, eb, nb, yaw) in zip(track, track[1:]):
        da = (ea - c[0]) * through[0] + (na - c[1]) * through[1]
        db = (eb - c[0]) * through[0] + (nb - c[1]) * through[1]
        if (da < 0) == (db < 0):
            continue
        side = (eb - c[0]) * left[0] + (nb - c[1]) * left[1]
        # The hull's half extent along the gate line at this heading
        fwd, port = (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw))
        across = (abs(fwd[0] * left[0] + fwd[1] * left[1]) * run_report.HALF_LENGTH
                  + abs(port[0] * left[0] + port[1] * left[1]) * run_report.HALF_WIDTH)
        clean = abs(side) + across < w / 2.0 - pole_r
        out.append((tb - t0, 'in' if db > da else 'out', side, clean))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('bag')
    ap.add_argument('--course', default='roboboat_2026')
    ap.add_argument('--course-file', default='')
    ap.add_argument('--antenna-behind', type=float, default=run_report.ANTENNA_BEHIND_M)
    ap.add_argument('--plot', default='')
    args = ap.parse_args()

    course, _ = course_report.load_course(args.course, args.course_file)
    data = run_report.read_bag(args.bag)
    track = run_report.truth_track(data, course.origin, args.antenna_behind)
    changes = run_report.timeline(data['/usv/nav_status'])
    if not track or not changes:
        sys.exit('no ground truth or no /usv/nav_status in the bag')
    started = [t for t, task, manual, _ in changes if not manual and task != 'WAIT_START']
    t0 = started[0] if started else changes[0][0]
    print(f'== {args.bag}: t = 0 when the run started (left WAIT_START in AUTO)')

    print('\n-- phases')
    for k, (t, task, manual, emergency) in enumerate(changes):
        end = changes[k + 1][0] if k + 1 < len(changes) else track[-1][0]
        flags = (' MANUAL' if manual else '') + (' EMERGENCY' if emergency else '')
        print(f'   {t - t0:7.1f} s  {task:<12} {end - t:6.1f} s{flags}')

    view = CourseModel().view(course_report.truth_landmarks(course),
                              (course.boat['east'], course.boat['north']))
    print('\n-- Task 1 gates (true hull; offset + = left of centre going in)')
    seq = []
    for k, gate in enumerate(view.t1_gates):
        w = math.hypot(gate.red['east'] - gate.green['east'],
                       gate.red['north'] - gate.green['north'])
        xs = gate_crossings(track, gate, t0)
        print(f'   gate {k + 1} ({w:.1f} m wide):' + ('' if xs else ' never crossed'))
        for t, d, side, clean in xs:
            print(f'     {t:7.1f} s  {d:<3} {side:+6.2f} m  '
                  f'{"whole hull between the poles" if clean else "NOT CLEAN"}')
            seq.append((t, k + 1, d, clean))
    seq.sort()
    order = [(gate, d) for _, gate, d, _ in seq]
    entry = order[:2] == [(1, 'in'), (2, 'in')]
    exit_ = order[-2:] == [(2, 'out'), (1, 'out')]
    print(f'   entry (gate 1 in, gate 2 in): {"YES" if entry else "no"}'
          f'{", clean" if entry and all(c for _, _, _, c in seq[:2]) else ""}')
    print(f'   exit (gate 2 out, gate 1 out): {"YES" if exit_ else "no"}'
          f'{", clean" if exit_ and all(c for _, _, _, c in seq[-2:]) else ""}')

    if view.channel:
        print('\n-- Task 2 channel (true hull; in = towards the debris field)')
        rows = []
        for k, gate in enumerate(view.channel):
            xs = gate_crossings(track, gate, t0, marker_r=0.203 / 2.0)
            through = gate.through
            if k + 1 < len(view.channel):
                nxt = view.channel[k + 1]
                d = (nxt.red['east'] - gate.red['east'], nxt.red['north'] - gate.red['north'])
                if d[0] * through[0] + d[1] * through[1] < 0:     # red on the right going in
                    xs = [(t, 'out' if dd == 'in' else 'in', -side, c) for t, dd, side, c in xs]
            text = ', '.join(f'{t:.1f} s {dd} {side:+.2f} m{"" if c else " NOT CLEAN"}'
                             for t, dd, side, c in xs) or 'never crossed'
            print(f'   gate {k + 1}: {text}')
            rows.append(xs)
        ins = [next((x for x in xs if x[1] == 'in'), None) for xs in rows]
        ok_in = all(ins) and all(x[3] for x in ins) and \
            all(a[0] < b[0] for a, b in zip(ins, ins[1:]))
        outs = [next((x for x in xs if x[1] == 'out'), None) for xs in rows]
        ok_out = all(outs) and all(x[3] for x in outs)
        print(f'   channel in (every gate, in order, clean): {"YES" if ok_in else "no"}')
        print(f'   channel out (every gate the other way, clean): {"YES" if ok_out else "no"}'
              f'{"" if any(outs) else " (not attempted yet)"}')

    _, buoys = run_report.load_course(args.course) if not args.course_file else (None, [])
    if buoys:
        print('\n-- closest approach (true hull outline to buoy edge), run only')
        rows = []
        for b in buoys:
            gap, when = min((run_report.clearance(b, e, n, yaw), t) for t, e, n, yaw in track
                            if t >= t0)
            rows.append((gap, when, b))
        rows.sort(key=lambda r: r[0])
        for gap, when, b in rows[:8]:
            print(f'   #{b["index"]:<2} {b["type"]:<11} {gap:+6.2f} m at {when - t0:6.1f} s')
        contacts = [r for r in rows if r[0] <= 0.0]
        print(f'   contacts: {len(contacts)}')
    extra = read_extra(args.bag)
    imu, pwm, fixes = steering(data, extra, track, changes, course.origin, t0)
    seen = boat_map(extra, course.origin)
    print(f"\n   the boat's own map at the end: {len(seen)} landmarks, "
          f"{sum(1 for lm in seen if lm['confirmed'])} confirmed (course_report.py compares "
          'it with the course)')
    print(f'\n   run length: {track[-1][0] - t0:.0f} s of truth after the start')
    if args.plot:
        plot(args.plot, track, fixes, imu, pwm, extra['/usv/heading'], changes, view,
             buoys, t0, seen)


if __name__ == '__main__':
    main()
