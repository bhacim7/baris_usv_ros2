#!/usr/bin/env python3
"""Map quality of one simulated run: every sighting and the final map against the truth
(phase 6 R1.6).

Usage (inside the container, after `source /ws/install/setup.bash`):
    python3 /ws/tools/map_report.py BAG_DIR [--course NAME | --course-file PATH]
                                            [--plot OUT.png] [--max-range M]

Record the run with sim.launch.py record:=<name>; survey:=true drives the same route every
time and seed:=<n> keeps the same sensor noise, so two runs can be compared. Needs
/usv/detections, /usv/fix, /usv/sim/navsat, /usv/sim/imu; /usv/map for part 3. Camera and
antenna mounts come from the installed fake_perception.yaml, the fix delay from
sim_bridge.yaml.

1. Sightings: each measured box of /usv/detections (position_ok, within --max-range, the
   mapper's limit) is tied to the course object it came from: the object of its class
   whose TRUE direction from the camera matches the box's pixel column and whose true depth
   is near the measured one. That uses neither the GPS nor the heading, the things checked.
   Its position is then worked out again with inputs swapped for the truth:
     gps      the fix used (/usv/fix)           -> the true antenna position at the frame
     heading  the heading used (/usv/heading)   -> the true heading
     range    the ZED depth                     -> true horizontal distance camera - object
     origin   placed from the GPS antenna       -> placed from the camera (lever arm)
   "only X fixed": as recorded but X true: what fixing X alone would gain.
   "only X wrong": all true but X: the error X brings by itself.
2. Inputs: how wrong each input was, and when. GPS error = receiver error (drift + noise)
   + age (the fix was measured up to 1/rate + delay before the frame); heading error
   against the yaw rate; range error = ZED noise + geometry (the ZED gives the depth of the
   object's front along the optical axis, not the distance to its centre).
3. The map (last /usv/map message): each object the camera saw - found, confirmed, error,
   duplicates; landmarks with no object of their kind within 3 m ("ghosts"); and the
   target: confirmed landmarks seen from within 6 m, mean error < 0.5 m, none above 1 m.
   Objects of one kind closer than 1.5 m to each other (the two marks of a vessel) count as
   one, as the mapper merges them.
"""

import argparse
import bisect
from dataclasses import dataclass
import math
import os
import statistics
import sys

from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from usv_core.world_map import kind_of
from usv_sim import course as course_mod
from usv_sim.camera import Buoy, camera_position, CameraModel, target_view, to_camera
from usv_sim.geo import enu
from usv_sim.sensors import compass_deg, yaw_from_quaternion
from usv_sim.zed import rotate
import yaml

NEEDED = ('/usv/detections', '/usv/fix', '/usv/sim/navsat', '/usv/sim/imu')
OPTIONAL = ('/usv/map',)
FACTORS = ('gps', 'heading', 'range', 'origin')
MATCH_DEG = 4.0               # box column vs true direction
GHOST_M = 3.0                 # landmark farther than this from any object of its kind
GROUP_M = 1.5                 # objects of one kind this close are one map object
TARGET_SEEN_M = 6.0
TARGET_MEAN_M = 0.5
TARGET_MAX_M = 1.0


# ---------------------------------------------------------------- reading
def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    missing = [t for t in NEEDED if t not in types]
    if missing:
        sys.exit(f'bag has no {", ".join(missing)} - record with sim.launch.py record:=<name>')
    data = {t: [] for t in NEEDED + OPTIONAL}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in data:
            data[topic].append((t_ns * 1e-9, deserialize_message(raw, types[topic])))
    return data


def ros_params(share, name):
    with open(os.path.join(share, 'config', name + '.yaml')) as f:
        return yaml.safe_load(f)[name]['ros__parameters']


@dataclass
class Setup:
    course: object
    cam: CameraModel
    antenna: tuple            # GNSS antenna, body frame
    fix_delay: float


def load_setup(course_name, course_file):
    share = get_package_share_directory('usv_sim')
    path = course_file or os.path.join(share, 'courses', course_name + '.yaml')
    with open(path) as f:
        course = course_mod.load(yaml.safe_load(f))
    fp = ros_params(share, 'fake_perception')
    cam = CameraModel(width=fp['image_width'], height=fp['image_height'],
                      hfov_deg=fp['camera_hfov_deg'], mount=tuple(fp['camera_mount_m']))
    return Setup(course, cam, tuple(fp['gnss_antenna_mount_m']),
                 ros_params(share, 'sim_bridge')['fix_delay_s'])


@dataclass
class Truth:
    index: int
    target: object            # usv_sim.camera Buoy / Panel
    class_name: str
    kind: str
    east: float
    north: float
    zc: float                 # height of its middle, for the direction


def truths(course):
    out = []
    for i, t in enumerate(course.targets):
        zc = (t.z0 + t.height) / 2.0 if isinstance(t, Buoy) else (t.z0 + t.z1) / 2.0
        name = course.class_names[t.class_id]
        out.append(Truth(i, t, name, kind_of(name), t.x, t.y, zc))
    return out


class Series:
    """Samples sorted by time; latest(t) = the last one at or before t (as a node sees it)."""

    def __init__(self, samples):
        self.t = [s[0] for s in samples]
        self.v = [s[1] for s in samples]

    def latest(self, t):
        i = bisect.bisect_right(self.t, t) - 1
        return self.v[i] if i >= 0 else None


# ---------------------------------------------------------------- geometry
def place(origin_e, origin_n, dist, bearing_deg):
    b = math.radians(bearing_deg)
    return origin_e + dist * math.sin(b), origin_n + dist * math.cos(b)


def body_to_enu(dx, dy, heading_deg):
    yaw = math.radians(90.0 - heading_deg)
    return (dx * math.cos(yaw) - dy * math.sin(yaw), dx * math.sin(yaw) + dy * math.cos(yaw))


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def pixel_angle(cx, cam):
    """Degrees right of the optical axis, as usv_core.detection.bearing_deg adds it."""
    return math.degrees(math.atan((cx - cam.cx) / cam.fx))


# ---------------------------------------------------------------- 1. sightings
def sightings(data, setup, objects, max_range):
    cam, origin = setup.cam, setup.course.origin
    navsat = Series([(t, enu(m.latitude, m.longitude, origin['lat'], origin['lon'])
                      + (m.altitude,)) for t, m in data['/usv/sim/navsat']])
    imu = Series([(t, m) for t, m in data['/usv/sim/imu']])
    fixes = [(t, m.latitude, m.longitude) for t, m in data['/usv/fix']]
    fix_t = [f[0] for f in fixes]
    lever_body = (cam.mount[0] - setup.antenna[0], cam.mount[1] - setup.antenna[1])

    recs, confused, unmatched, skipped = [], {}, 0, 0
    for t, frame in data['/usv/detections']:
        h_used = frame.heading_deg
        if math.isnan(h_used) or not frame.boat_latitude:
            continue
        # The /usv/fix message this frame used (same numbers), and when it arrived
        i = bisect.bisect_right(fix_t, t) - 1
        t_fix = None
        while i >= 0 and fix_t[i] > t - 3.0:
            if fixes[i][1] == frame.boat_latitude and fixes[i][2] == frame.boat_longitude:
                t_fix = fix_t[i]
                break
            i -= 1
        antenna, imu_msg = navsat.latest(t), imu.latest(t)
        if t_fix is None or antenna is None or imu_msg is None:
            skipped += len(frame.detections)
            continue
        o = imu_msg.orientation
        q = (o.x, o.y, o.z, o.w)
        lever = rotate(q, setup.antenna)
        boat = tuple(a - lv for a, lv in zip(antenna, lever))
        cam_pos = camera_position(boat, q, cam)
        h_true = compass_deg(yaw_from_quaternion(*q))
        yaw_rate = math.degrees(imu_msg.angular_velocity.z)
        a_fix = enu(frame.boat_latitude, frame.boat_longitude, origin['lat'], origin['lon'])
        sampled = navsat.latest(t_fix - setup.fix_delay) or antenna

        for d in frame.detections:
            if not d.position_ok or not d.distance_m <= max_range:
                continue
            cx = int((d.x1 + d.x2) / 2)
            alpha = pixel_angle(cx, cam)
            obj, front = match(objects, d.class_name, alpha, d.distance_m, cam_pos, q, cam)
            if obj is None:
                other, _ = match(objects, None, alpha, d.distance_m, cam_pos, q, cam)
                if other is None:
                    unmatched += 1
                else:
                    key = (other.class_name, d.class_name)
                    confused[key] = confused.get(key, 0) + 1
                continue
            r_true = math.hypot(obj.east - cam_pos[0], obj.north - cam_pos[1])
            recs.append({
                't': t, 'obj': obj, 'alpha': alpha, 'd': d.distance_m, 'front': front,
                'r_true': r_true, 'h_used': h_used, 'h_true': h_true, 'yaw_rate': yaw_rate,
                'a_fix': a_fix, 'a_true': antenna[:2], 'a_sampled': sampled[:2],
                'fix_age': t - t_fix + setup.fix_delay, 'cam': cam_pos[:2],
                'lever_body': lever_body,
                'recorded': enu(d.latitude, d.longitude, origin['lat'], origin['lon'])})
    return recs, confused, unmatched, skipped


def match(objects, class_name, alpha, depth, cam_pos, q, cam):
    """(object, its true front depth) the box most likely shows, or (None, None)."""
    best, best_cost, best_front = None, math.inf, None
    for obj in objects:
        if class_name is not None and obj.class_name != class_name:
            continue
        if math.hypot(obj.east - cam_pos[0], obj.north - cam_pos[1]) > depth + 6.0:
            continue
        p = to_camera((obj.east, obj.north, obj.zc), cam_pos, q)
        if p[0] <= 0.2:
            continue
        d_alpha = abs(alpha - math.degrees(math.atan(-p[1] / p[0])))
        if d_alpha > MATCH_DEG:
            continue
        view = target_view(obj.target, cam_pos, q, cam)
        if view is None:
            continue
        front = view[1]
        if abs(depth - front) > max(1.0, 0.4 * front):
            continue
        cost = d_alpha / 2.0 + abs(depth - front) / max(0.5, 0.2 * front)
        if cost < best_cost:
            best, best_cost, best_front = obj, cost, front
    return best, best_front


def position(rec, fixed):
    """Where the sighting lands with the inputs in `fixed` swapped for the truth."""
    ae, an = rec['a_true'] if 'gps' in fixed else rec['a_fix']
    h = rec['h_true'] if 'heading' in fixed else rec['h_used']
    if 'origin' in fixed:
        le, ln = body_to_enu(*rec['lever_body'], h)
        ae, an = ae + le, an + ln
    dist = rec['r_true'] if 'range' in fixed else rec['d']
    return place(ae, an, dist, h + rec['alpha'])


def error(rec, pos):
    """(total, along, cross): along + = placed too far, cross + = placed to the right."""
    obj = rec['obj']
    ex, ey = pos[0] - obj.east, pos[1] - obj.north
    ux, uy = obj.east - rec['cam'][0], obj.north - rec['cam'][1]
    n = math.hypot(ux, uy) or 1.0
    ux, uy = ux / n, uy / n
    return math.hypot(ex, ey), ex * ux + ey * uy, ex * uy - ey * ux


# ---------------------------------------------------------------- printing helpers
def summary(values):
    if not values:
        return '     -'
    v = sorted(values)
    p90 = v[min(len(v) - 1, int(0.9 * len(v)))]
    return (f'{statistics.mean(v):6.2f} {statistics.median(v):6.2f} {p90:6.2f} '
            f'{v[-1]:6.2f}')


def mean(values):
    return statistics.mean(values) if values else float('nan')


def rms(values):
    return math.sqrt(statistics.mean([v * v for v in values])) if values else float('nan')


def binned(recs, key, edges, columns):
    """Print one row per bin of rec[key]: count and each column's mean."""
    for lo, hi in zip(edges, edges[1:]):
        rows = [r for r in recs if lo <= key(r) < hi]
        label = f'{lo:g}-{hi:g}' if hi != math.inf else f'{lo:g}+'
        cells = ' '.join(f'{mean([f(r) for r in rows]):8.2f}' if rows else '       -'
                         for _, f in columns)
        print(f'   {label:>9} {len(rows):6d} {cells}')


def report_sightings(recs, confused, unmatched, skipped, max_range):
    print(f'\n-- 1. sightings within {max_range:g} m (each measured box, before the map)')
    total_conf = sum(confused.values())
    print(f'   tied to their object: {len(recs)}; wrong class: {total_conf}; '
          f'no object found: {unmatched}; frames without fix/pose: {skipped} boxes')
    if confused:
        worst = sorted(confused.items(), key=lambda kv: -kv[1])[:6]
        print('   wrong classes (object -> reported): ' + ', '.join(
            f'{a}->{b} {n}' for (a, b), n in worst))
    if not recs:
        return {}
    for r in recs:
        r['err'] = {}
    combos = {'as recorded': ()}
    for f in FACTORS:
        combos[f'only {f} fixed'] = (f,)
    for f in FACTORS:
        combos[f'only {f} wrong'] = tuple(g for g in FACTORS if g != f)
    combos['all true (residual)'] = FACTORS
    for name, fixed in combos.items():
        for r in recs:
            r['err'][name] = error(r, position(r, fixed))
    recorded = []
    for r in recs:
        (e1, n1), (e2, n2) = r['recorded'], position(r, ())
        recorded.append(math.hypot(e1 - e2, n1 - n2))
    print(f'   recorded vs recomputed position: mean {100 * mean(recorded):.1f} cm '
          '(the legacy formula uses a spherical Earth)')

    print('\n   position error, m              mean median    p90    max   along  cross')
    means = {}
    for name in combos:
        tot = [r['err'][name][0] for r in recs]
        along = mean([r['err'][name][1] for r in recs])
        cross = mean([r['err'][name][2] for r in recs])
        means[name] = mean(tot)
        print(f'   {name:<28} {summary(tot)} {along:+6.2f} {cross:+6.2f}')
        if name in ('as recorded', f'only {FACTORS[-1]} fixed', f'only {FACTORS[-1]} wrong'):
            print()
    print('   along + = placed beyond the object, cross + = placed to its right '
          '(seen from the boat)')
    near = [r for r in recs if r['r_true'] <= TARGET_SEEN_M]
    print(f'   within {TARGET_SEEN_M:g} m: {len(near)} sightings, as recorded mean '
          f'{mean([r["err"]["as recorded"][0] for r in near]):.2f} m')
    return means


def report_inputs(recs):
    if not recs:
        return
    print('\n-- 2. inputs (recorded minus true)')
    gps = [math.hypot(r['a_fix'][0] - r['a_true'][0], r['a_fix'][1] - r['a_true'][1])
           for r in recs]
    recv = [math.hypot(r['a_fix'][0] - r['a_sampled'][0], r['a_fix'][1] - r['a_sampled'][1])
            for r in recs]
    age = [math.hypot(r['a_sampled'][0] - r['a_true'][0], r['a_sampled'][1] - r['a_true'][1])
           for r in recs]
    print('   GPS position error, m        mean median    p90    max')
    print(f'     total                     {summary(gps)}')
    print(f'     receiver (drift + noise)  {summary(recv)}')
    print(f'     age (boat moved since)    {summary(age)}   fix age mean '
          f'{mean([r["fix_age"] for r in recs]):.2f} s')
    print('     by fix age (s)    n   total  receiver    age')
    binned(recs, lambda r: r['fix_age'], [0.0, 0.2, 0.35, 0.5, 0.65, math.inf], [
        ('total', lambda r: math.hypot(r['a_fix'][0] - r['a_true'][0],
                                       r['a_fix'][1] - r['a_true'][1])),
        ('receiver', lambda r: math.hypot(r['a_fix'][0] - r['a_sampled'][0],
                                          r['a_fix'][1] - r['a_sampled'][1])),
        ('age', lambda r: math.hypot(r['a_sampled'][0] - r['a_true'][0],
                                     r['a_sampled'][1] - r['a_true'][1]))])

    herr = [wrap180(r['h_used'] - r['h_true']) for r in recs]
    print(f'\n   heading error, deg: mean {mean(herr):+.2f} (constant part), rms {rms(herr):.2f}, '
          f'max {max(abs(h) for h in herr):.1f}')
    print('     by |yaw rate| (deg/s)  n    mean  mean|e|  pos err')
    binned(recs, lambda r: abs(r['yaw_rate']), [0.0, 5.0, 15.0, 30.0, math.inf], [
        ('mean', lambda r: wrap180(r['h_used'] - r['h_true'])),
        ('abs', lambda r: abs(wrap180(r['h_used'] - r['h_true']))),
        ('pos', lambda r: r['err']['as recorded'][0])])

    print('\n   range, m: measured - true distance = ZED noise + geometry')
    print('     by true distance  n   total  ZED noise geometry  pos err')
    binned(recs, lambda r: r['r_true'], [0.0, 3.0, 6.0, 9.0, 12.0, math.inf], [
        ('total', lambda r: r['d'] - r['r_true']),
        ('zed', lambda r: r['d'] - r['front']),
        ('geom', lambda r: r['front'] - r['r_true']),
        ('pos', lambda r: r['err']['as recorded'][0])])
    print('     by |pixel angle| (deg)')
    binned(recs, lambda r: abs(r['alpha']), [0.0, 15.0, 30.0, 45.0, math.inf], [
        ('total', lambda r: r['d'] - r['r_true']),
        ('zed', lambda r: r['d'] - r['front']),
        ('geom', lambda r: r['front'] - r['r_true']),
        ('pos', lambda r: r['err']['as recorded'][0])])
    lever = math.hypot(*recs[0]['lever_body'])
    print(f'\n   origin: positions are placed from the GPS antenna, the camera is {lever:.2f} m '
          'away from it')


# ---------------------------------------------------------------- 3. the map
def group_objects(objects):
    """Objects of one kind within GROUP_M of each other -> one map object (mean position)."""
    groups = []
    for obj in objects:
        for g in groups:
            if g['kind'] == obj.kind and any(
                    math.hypot(o.east - obj.east, o.north - obj.north) < GROUP_M
                    for o in g['members']):
                g['members'].append(obj)
                break
        else:
            groups.append({'kind': obj.kind, 'members': [obj]})
    for g in groups:
        g['east'] = mean([o.east for o in g['members']])
        g['north'] = mean([o.north for o in g['members']])
        g['label'] = f"#{g['members'][0].index:<3} {g['kind']}" + (
            f" x{len(g['members'])}" if len(g['members']) > 1 else '')
    return groups


def report_map(data, setup, objects, recs):
    if not data['/usv/map']:
        print('\n-- 3. map: no /usv/map in the bag')
        return None
    origin = setup.course.origin
    _, last = data['/usv/map'][-1]
    lms = []
    for lm in last.landmarks:
        e, n = enu(lm.latitude, lm.longitude, origin['lat'], origin['lon'])
        lms.append({'id': lm.id, 'kind': lm.kind, 'color': lm.color, 'east': e, 'north': n,
                    'confirmed': lm.confirmed, 'sightings': lm.sightings,
                    'closest': lm.closest_m, 'group': None, 'dist': math.inf})
    groups = group_objects(objects)
    seen = {}
    for r in recs:
        seen[r['obj'].index] = min(seen.get(r['obj'].index, math.inf), r['r_true'])
    for g in groups:
        g['seen'] = min(seen.get(o.index, math.inf) for o in g['members'])
        g['lms'] = []
    for lm in lms:
        for g in groups:
            if g['kind'] == lm['kind']:
                dist = math.hypot(lm['east'] - g['east'], lm['north'] - g['north'])
                if dist < lm['dist']:
                    lm['group'], lm['dist'] = g, dist
        if lm['group'] is not None and lm['dist'] <= GHOST_M:
            lm['group']['lms'].append(lm)
        else:
            lm['group'] = None

    confirmed = [lm for lm in lms if lm['confirmed']]
    print(f'\n-- 3. map: {len(lms)} landmarks, {len(confirmed)} confirmed '
          '(last /usv/map message)')
    print('   object              seen from  found  error  sightings  duplicates (confirmed)')
    errors = []
    for g in sorted(groups, key=lambda g: g['members'][0].index):
        if g['seen'] == math.inf and not g['lms']:
            continue
        lm_sorted = sorted(g['lms'], key=lambda lm: (not lm['confirmed'], -lm['sightings']))
        main = lm_sorted[0] if lm_sorted else None
        dups = lm_sorted[1:]
        seen_txt = f'{g["seen"]:5.1f} m' if g['seen'] < math.inf else '    -  '
        if main is None:
            print(f'   {g["label"]:<22} {seen_txt}   none')
            continue
        state = 'yes' if main['confirmed'] else 'unconf'
        print(f'   {g["label"]:<22} {seen_txt}  {state:>6} {main["dist"]:5.2f} m '
              f'{main["sightings"]:6d}      {len(dups)} '
              f'({sum(1 for d in dups if d["confirmed"])})')
        if main['confirmed']:
            errors.append((g, main))

    ghosts = [lm for lm in lms if lm['group'] is None]
    print(f'   ghosts (no object of their kind within {GHOST_M:g} m): {len(ghosts)}, '
          f'confirmed {sum(1 for lm in ghosts if lm["confirmed"])}')
    for lm in sorted(ghosts, key=lambda lm: -lm['sightings'])[:5]:
        if lm['confirmed']:
            print(f'     id {lm["id"]} {lm["kind"]} x{lm["sightings"]}, nearest '
                  f'{lm["kind"]} {lm["dist"]:.1f} m')

    if errors:
        de = mean([lm['east'] - g['east'] for g, lm in errors])
        dn = mean([lm['north'] - g['north'] for g, lm in errors])
        resid = [math.hypot(lm['east'] - g['east'] - de, lm['north'] - g['north'] - dn)
                 for g, lm in errors]
        print(f'   confirmed landmarks: mean error {mean([lm["dist"] for _, lm in errors]):.2f}'
              f' m; common offset {de:+.2f} m east {dn:+.2f} m north, '
              f'{mean(resid):.2f} m left after removing it')
    near = [lm['dist'] for g, lm in errors if lm['closest'] <= TARGET_SEEN_M]
    if near:
        ok = mean(near) < TARGET_MEAN_M and max(near) <= TARGET_MAX_M
        print(f'\n   TARGET (confirmed, seen from <= {TARGET_SEEN_M:g} m: mean < '
              f'{TARGET_MEAN_M} m, max <= {TARGET_MAX_M} m): n = {len(near)}, mean '
              f'{mean(near):.2f} m, max {max(near):.2f} m -> {"PASS" if ok else "FAIL"}')
    return {'groups': groups, 'lms': lms}


# ---------------------------------------------------------------- plot
def plot(path, data, setup, objects, recs, means, map_part):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from usv_nodes.mapper import COLORS

    origin, boat = setup.course.origin, setup.course.boat
    x0, y0 = boat['east'], boat['north']
    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_axes([0.04, 0.06, 0.55, 0.88])
    gps = [enu(m.latitude, m.longitude, origin['lat'], origin['lon'])
           for _, m in data['/usv/fix']]
    true = [enu(m.latitude, m.longitude, origin['lat'], origin['lon'])
            for _, m in data['/usv/sim/navsat']][::5]
    ax.plot([e - x0 for e, _ in true], [n - y0 for _, n in true], '-', color='0.6', lw=1,
            label='true antenna track')
    ax.plot([e - x0 for e, _ in gps], [n - y0 for _, n in gps], '.', color='tab:blue',
            ms=2, label='GPS fixes')
    for obj in objects:
        rgb = COLORS.get(obj.kind, COLORS.get(obj.class_name.split('_')[-1], (0.5, 0.5, 0.5)))
        ax.plot(obj.east - x0, obj.north - y0, 'o', mfc='none', mec=rgb, ms=9, mew=1.5)
    if map_part:
        for lm in map_part['lms']:
            rgb = COLORS.get(lm['color'] or lm['kind'], (0.5, 0.5, 0.5))
            ax.plot(lm['east'] - x0, lm['north'] - y0, 'o' if lm['confirmed'] else '.',
                    color=rgb, alpha=1.0 if lm['confirmed'] else 0.35,
                    ms=5 if lm['confirmed'] else 4)
            if lm['group'] is not None and lm['confirmed']:
                g = lm['group']
                ax.plot([lm['east'] - x0, g['east'] - x0], [lm['north'] - y0, g['north'] - y0],
                        '-', color='k', lw=0.8)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('east of the boat start, m')
    ax.set_ylabel('north of the boat start, m')
    ax.set_title('rings: true objects; dots: map (faint = unconfirmed); line: error')
    ax.legend(loc='best', fontsize=8)

    ax2 = fig.add_axes([0.66, 0.56, 0.32, 0.38])
    if recs:
        sc = ax2.scatter([r['r_true'] for r in recs],
                         [r['err']['as recorded'][0] for r in recs],
                         c=[min(40.0, abs(r['yaw_rate'])) for r in recs], s=3, cmap='viridis')
        fig.colorbar(sc, ax=ax2, label='|yaw rate|, deg/s')
    ax2.set_xlabel('true distance, m')
    ax2.set_ylabel('sighting position error, m')
    ax2.grid(alpha=0.3)

    ax3 = fig.add_axes([0.77, 0.08, 0.21, 0.36])
    if means:
        names = ['as recorded'] + [f'only {f} wrong' for f in FACTORS] + ['all true (residual)']
        colors = ['0.6'] + ['tab:orange'] * len(FACTORS) + ['tab:red']
        ax3.barh(names[::-1], [means[n] for n in names[::-1]], color=colors[::-1])
        ax3.set_xlabel('mean sighting error, m')
        ax3.set_title('error each input brings by itself')
        ax3.grid(alpha=0.3, axis='x')
    fig.savefig(path, dpi=110)
    print(f'\nplot: {path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('--course', default='roboboat_2026')
    ap.add_argument('--course-file', default='',
                    help='a placed course file (/tmp/usv_sim_<course>_placed.yaml) when '
                    'the run used course_east/north/heading')
    ap.add_argument('--max-range', type=float, default=12.0,
                    help="sightings farther than this are left out (the mapper's max_range_m)")
    ap.add_argument('--plot', default='')
    args = ap.parse_args()

    setup = load_setup(args.course, args.course_file)
    objects = truths(setup.course)
    data = read_bag(args.bag)
    duration = data['/usv/detections'][-1][0] - data['/usv/detections'][0][0] \
        if data['/usv/detections'] else 0.0
    print(f'== {args.bag}: {len(data["/usv/detections"])} camera frames over {duration:.0f} s, '
          f'course {setup.course.name} ({len(objects)} objects)')

    recs, confused, unmatched, skipped = sightings(data, setup, objects, args.max_range)
    means = report_sightings(recs, confused, unmatched, skipped, args.max_range)
    report_inputs(recs)
    map_part = report_map(data, setup, objects, recs)
    if args.plot:
        plot(args.plot, data, setup, objects, recs, means, map_part)


if __name__ == '__main__':
    main()
