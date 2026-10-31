#!/usr/bin/env python3
"""Report on one simulated run, recorded with ros2 bag (phase 5.6).

Usage (inside the container, after `source /ws/install/setup.bash`):
    python3 /ws/tools/run_report.py BAG_DIR [--course NAME] [--plot OUT.png]
                                            [--reference REF.yaml]

Needs these topics in the bag:
    /usv/nav_status     task timeline, manual/auto, emergency
    /usv/thrusters/pwm  what the navigator commanded
    /usv/sim/navsat     Gazebo's TRUE antenna position (no GPS error)
    /usv/sim/imu        Gazebo's true attitude and yaw rate

Everything about where the boat really was comes from Gazebo's ground truth, not from
/usv/fix: a contact is a contact whatever the GPS believed. The hull is the catamaran's
outline (1.182 x 0.78 m) around the model origin; the antenna sits 0.60 m behind it. A
buoy's clearance is the gap between that outline and the buoy's edge (course file
diameter); 0 or less is a contact. Expect +/- 5 cm from the outline approximation.

--reference compares with what was read from the legacy run's video
(ws/bags/video/run_2026-10-14_reference.yaml).
"""

import argparse
import bisect
import math
import os
import statistics
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from usv_sim.geo import enu
from usv_sim.sensors import yaw_from_quaternion
import yaml

HALF_LENGTH = 1.182 / 2
HALF_WIDTH = 0.78 / 2
ANTENNA_BEHIND_M = 0.60
TARGET_TYPES = ('red_target', 'green_target', 'black_target')
TOPICS = ('/usv/nav_status', '/usv/thrusters/pwm', '/usv/sim/navsat', '/usv/sim/imu')


# ---------------------------------------------------------------- reading
def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in types]
    if missing:
        sys.exit(f'bag has no {", ".join(missing)} - record them (see daily-startup.md)')
    data = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in data:
            data[topic].append((t_ns * 1e-9, deserialize_message(raw, types[topic])))
    return data


def load_course(name):
    path = os.path.join(get_package_share_directory('usv_sim'), 'courses', name + '.yaml')
    with open(path) as f:
        course = yaml.safe_load(f)
    o = course['origin']
    buoys = []
    for i, b in enumerate(course['buoys']):
        spec = course['buoy_types'][b['type']]
        e, n = enu(b['lat'], b['lon'], o['lat'], o['lon'])
        buoys.append({'index': i, 'type': b['type'], 'east': e, 'north': n,
                      'radius': spec['diameter'] / 2.0})
    return course, buoys


# ---------------------------------------------------------------- ground truth
def truth_track(data, origin):
    """[(t, east, north, yaw_rad)] of the hull centre, from the true antenna and yaw."""
    imu = data['/usv/sim/imu']
    imu_t = [t for t, _ in imu]
    yaws = [yaw_from_quaternion(m.orientation.x, m.orientation.y, m.orientation.z,
                                m.orientation.w) for _, m in imu]
    track = []
    for t, fix in data['/usv/sim/navsat']:
        k = bisect.bisect_right(imu_t, t) - 1
        if k < 0:
            continue
        yaw = yaws[k]
        ae, an = enu(fix.latitude, fix.longitude, origin['lat'], origin['lon'])
        track.append((t, ae + ANTENNA_BEHIND_M * math.cos(yaw),
                      an + ANTENNA_BEHIND_M * math.sin(yaw), yaw))
    return track


def clearance(buoy, east, north, yaw):
    dx, dy = buoy['east'] - east, buoy['north'] - north
    c, s = math.cos(yaw), math.sin(yaw)
    along, across = dx * c + dy * s, -dx * s + dy * c
    qx = max(abs(along) - HALF_LENGTH, 0.0)
    qy = max(abs(across) - HALF_WIDTH, 0.0)
    return math.hypot(qx, qy) - buoy['radius']


# ---------------------------------------------------------------- analysis
def timeline(status):
    """[(t, task, manual, emergency)] at every change."""
    out = []
    last = None
    for t, m in status:
        key = (m.task_state, m.manual, m.emergency)
        if key != last:
            out.append((t,) + key)
            last = key
    return out


def task_at(changes, t):
    task = changes[0][1] if changes else '?'
    for tc, name, _, _ in changes:
        if tc > t:
            break
        task = name
    return task


def speeds(track, window_s=1.0):
    """[(t, m/s)] over a sliding window, from the true track."""
    out = []
    j = 0
    for i, (t, e, n, _) in enumerate(track):
        while track[j][0] < t - window_s:
            j += 1
        if t - track[j][0] >= 0.5 * window_s:
            out.append((t, math.hypot(e - track[j][1], n - track[j][2]) / (t - track[j][0])))
    return out


def pwm_at(pwm, t):
    k = bisect.bisect_right([p[0] for p in pwm], t) - 1
    return pwm[k][1] if k >= 0 else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--course', default='golcuk_2026_10_14')
    ap.add_argument('--plot')
    ap.add_argument('--reference')
    args = ap.parse_args()

    data = read_bag(args.bag)
    course, buoys = load_course(args.course)
    status = data['/usv/nav_status']
    pwm = data['/usv/thrusters/pwm']
    track = truth_track(data, course['origin'])
    if not status or not track:
        sys.exit('no nav_status or no ground truth in the bag')

    changes = timeline(status)
    auto = [t for t, task, manual, emerg in changes if not manual]
    t0 = auto[0] if auto else changes[0][0]
    print(f'== {args.bag}')
    print(f'   t = 0 at the first AUTO cycle; run length {track[-1][0] - t0:.1f} s after it')

    # --- task timeline
    print('\n-- task timeline')
    for i, (t, task, manual, emerg) in enumerate(changes):
        end = changes[i + 1][0] if i + 1 < len(changes) else status[-1][0]
        flags = ' MANUAL' if manual else ''
        flags += ' EMERGENCY' if emerg else ''
        print(f'   {t - t0:7.1f} s  {task:<24} {end - t:6.1f} s{flags}')
    finished = [t for t, task, _, _ in changes if task == 'FINISHED']
    if finished:
        print(f'   FINISHED at {finished[0] - t0:.1f} s')
    else:
        print('   not FINISHED in this recording')

    # --- clearances
    print('\n-- closest approach to each buoy (true hull outline to buoy edge)')
    rows = []
    for b in buoys:
        best = min(((clearance(b, e, n, y), t) for t, e, n, y in track), key=lambda x: x[0])
        rows.append((best[0], best[1], b))
    contacts = [r for r in rows if r[0] <= 0.0]
    for gap, t, b in sorted(rows, key=lambda r: r[0])[:10]:
        mark = ''
        if gap <= 0.0:
            mark = '  <-- target hit' if b['type'] in TARGET_TYPES else '  <-- CONTACT'
        print(f'   #{b["index"]:<2} {b["type"]:<12} {gap:+6.2f} m at {t - t0:6.1f} s '
              f'({task_at(changes, t)}){mark}')
    obstacle_contacts = [r for r in contacts if r[2]['type'] not in TARGET_TYPES]
    print(f'   contacts with corridor buoys: {len(obstacle_contacts)}')

    # --- speed and turning
    sp = speeds(track)
    imu = data['/usv/sim/imu']
    cruise, spot = [], []
    for t, v in sp:
        cmd = pwm_at(pwm, t)
        if cmd is None or t < t0:
            continue
        fwd = (cmd.rear_left + cmd.rear_right + cmd.front_left + cmd.front_right) / 4.0
        diff = abs(cmd.rear_left - cmd.rear_right)
        if fwd >= 1650 and diff < 60:
            cruise.append(v)
    for t, m in imu:
        cmd = pwm_at(pwm, t)
        if cmd is None or t < t0:
            continue
        fwd = (cmd.rear_left + cmd.rear_right + cmd.front_left + cmd.front_right) / 4.0
        if abs(fwd - 1500) < 5 and abs(cmd.rear_left - cmd.rear_right) >= 280:
            spot.append(abs(math.degrees(m.angular_velocity.z)))
    print('\n-- speed and turning (true values)')
    t2_fwd = [(p.rear_left + p.rear_right + p.front_left + p.front_right) / 4.0
              for t, p in pwm if t >= t0 and task_at(changes, t).startswith('TASK2')]
    t2_speed = [v for t, v in sp if t >= t0 and task_at(changes, t).startswith('TASK2')]
    if t2_fwd and t2_speed:
        print(f'   Task 2 averages: forward PWM {statistics.mean(t2_fwd):.0f}, speed '
              f'{statistics.mean(t2_speed):.2f} m/s (max {max(t2_speed):.2f})')
    if cruise:
        print(f'   cruising (forward >= 1650, little differential): median '
              f'{statistics.median(cruise):.2f} m/s, n = {len(cruise)}')
    if spot:
        print(f'   spot turns (forward 1500, |differential| >= 280): median '
              f'{statistics.median(spot):.0f} deg/s, max {max(spot):.0f}')
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(track, track[1:])
               if a[0] >= t0)
    print(f'   distance travelled after t = 0: {dist:.0f} m')

    # --- left/right differential flips (the ~1 s steering oscillation in the video)
    flips = 0
    sign = 0
    n_t2 = [p for t, p in pwm if t >= t0 and task_at(changes, t).startswith('TASK2')]
    for p in n_t2:
        d = p.rear_left - p.rear_right
        s = (d > 40) - (d < -40)
        if s and s != sign:
            if sign:
                flips += 1
            sign = s
    t2_time = sum(
        (changes[i + 1][0] if i + 1 < len(changes) else status[-1][0]) - t
        for i, (t, task, _, _) in enumerate(changes) if task.startswith('TASK2'))
    if t2_time > 0:
        print(f'   Task 2: left/right differential flips {flips} in {t2_time:.0f} s '
              f'({flips / t2_time:.2f} per s)')

    # --- reference from the legacy video
    if args.reference:
        with open(args.reference) as f:
            ref = yaml.safe_load(f)
        print('\n-- legacy run (video) vs this run')
        for key, value in ref.get('metrics', {}).items():
            print(f'   {key:<34} video: {value}')
        for ev in ref.get('events', []):
            print(f'   video {ev["t"]:6.1f} s  {ev["what"]}')

    if args.plot:
        plot(args.plot, track, buoys, changes, rows, t0)
        print(f'\nplot: {args.plot}')


def plot(path, track, buoys, changes, rows, t0):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'orange_flag': 'tab:orange', 'yellow_flag': 'gold', 'red_target': 'red',
              'green_target': 'green', 'black_target': 'black'}
    fig, ax = plt.subplots(figsize=(9, 9))
    for b in buoys:
        ax.add_patch(plt.Circle((b['east'], b['north']), max(b['radius'], 0.15),
                                color=colors.get(b['type'], 'grey')))
    tasks = []
    for t, e, n, _ in track:
        if t < t0:
            continue
        name = task_at(changes, t)
        if not tasks or tasks[-1][0] != name:
            tasks.append((name, [], []))
        tasks[-1][1].append(e)
        tasks[-1][2].append(n)
    seen = set()
    for name, es, ns in tasks:
        ax.plot(es, ns, '-', lw=1.2, label=None if name in seen else name)
        seen.add(name)
    for gap, t, b in rows:
        if gap <= 0.0 and b['type'] not in TARGET_TYPES:
            ax.plot(b['east'], b['north'], 'x', color='magenta', ms=14, mew=3)
    ax.set_aspect('equal')
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title('true track (hull centre); x = contact')
    fig.savefig(path, dpi=120, bbox_inches='tight')


if __name__ == '__main__':
    main()
