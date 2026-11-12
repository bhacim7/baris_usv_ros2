#!/usr/bin/env python3
"""
Generate a Gazebo world for a course file (courses/*.yaml).

The course file is the single source of truth: this script builds the world from it, and
fake_perception reads the same file for the buoy positions. Base: worlds/nbpark.sdf (VRX
physics, sensors, water, waves and our boat), with these changes:

  * the Nathan Benderson Park land model is kept only for courses at that venue
    (`venue: nbpark`); elsewhere (e.g. Golcuk) open water only
  * the VRX wind plugin is left out (it only acts on the WAM-V)
  * spherical_coordinates: the course origin, so /usv/fix reads the course's lat/lon
  * NO magnetic field: with spherical coordinates gz-sim ignores <magnetic_field> and
    uses a coarse built-in table; fake_perception computes the ZED 2i magnetometer from
    the course's magnetic_field instead (see usv_sim/zed.py)
  * the boat starts at the course's start pose
  * one static model per buoy, shaped from the course's buoy_types (sphere, pear,
    cylinder, pole, indicator, robobuoy); one per vessel and dock (RoboBoat), geometry
    from usv_sim/course.py, which fake_perception also uses
  * a RoboBuoy (RoboBoat 2027) is drawn with its light as it starts: Gazebo does not show
    later changes or the flashing (what the camera sees does: fake_perception)

Gazebo's world frame is ENU here: x east, y north, z up.

One model per boat (RoboBoat 2027: Azure and Marine), named usv_<boat> (usv_azure, ...),
copied from models/usv_boat/model.sdf with its Gazebo topics under /usv_<boat>/ instead of
/usv_boat/ (sim.launch.py's bridge follows); the first boat on the course's start, the
others SPACING_M to its right, one after the other.

Usage (inside the container):
    python3 /ws/src/usv_sim/scripts/make_course_world.py COURSE.yaml OUTPUT.sdf [BOATS]
BOATS: comma separated names, default azure.
"""

import math
import os
import re
import sys
import xml.etree.ElementTree as ET

# usv_sim.geo from the source tree, so the script also runs before a build
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from usv_sim import course as course_mod  # noqa: E402
from usv_sim import robobuoy as rb  # noqa: E402
from usv_sim.geo import enu as geo_enu  # noqa: E402
import yaml  # noqa: E402

BASE_WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds',
                          'nbpark.sdf')
COLOURS = {   # ambient/diffuse RGB
    'orange_flag': (1.0, 0.42, 0.0),
    'yellow_flag': (1.0, 0.82, 0.0),
    'red_target': (0.75, 0.02, 0.02),
    'green_target': (0.0, 0.55, 0.12),
    'black_target': (0.04, 0.04, 0.04),
}
FLAG_RGB = (0.85, 0.1, 0.05)
BOAT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models',
                          'usv_boat', 'model.sdf')
SPACING_M = 6.0             # boats side by side at the start (beam 0.78 m)
LIGHT_RGB = {'red': (1.0, 0.08, 0.05), 'green': (0.1, 1.0, 0.2), 'blue': (0.15, 0.35, 1.0),
             'off': (0.12, 0.12, 0.12)}
NOODLE_RGB = (0.25, 0.45, 0.8)      # ASSUMED: pool noodles come in any colour
SUBMERGED_M = 0.25          # body extends this far below the water line


def enu(lat, lon, origin):
    """Metres east and north of the origin (WGS84, as Gazebo's NavSat)."""
    return geo_enu(lat, lon, origin['lat'], origin['lon'])


def sub(parent, tag, text=None, **attrib):
    e = ET.SubElement(parent, tag, attrib)
    if text is not None:
        e.text = text
    return e


def material(parent, rgb):
    m = sub(parent, 'material')
    sub(m, 'ambient', f'{rgb[0]} {rgb[1]} {rgb[2]} 1')
    sub(m, 'diffuse', f'{rgb[0]} {rgb[1]} {rgb[2]} 1')


def visual(link, name, pose, shape, size, rgb, emissive=None, uri=None):
    v = sub(link, 'visual', name=name)
    sub(v, 'pose', pose)
    g = sub(v, 'geometry')
    if shape == 'box':
        sub(sub(g, 'box'), 'size', ' '.join(f'{s:.4f}' for s in size))
    elif shape == 'cylinder':
        c = sub(g, 'cylinder')
        sub(c, 'radius', f'{size[0]:.4f}')
        sub(c, 'length', f'{size[1]:.4f}')
    else:
        sub(sub(g, 'mesh'), 'uri', uri)
    material(v, rgb)
    if emissive:
        sub(v.find('material'), 'emissive', f'{emissive[0]} {emissive[1]} {emissive[2]} 1')
    return v


def robobuoy_model(name, spec, x, y):
    """Build a RoboBuoy (usv_sim/robobuoy.py): float frame, panels, diamonds, beacon, hat."""
    model = ET.Element('model', name=name)
    sub(model, 'static', 'true')
    sub(model, 'pose', f'{x:.3f} {y:.3f} 0 0 0 0')
    link = sub(model, 'link', name='link')
    white, black = course_mod.RGB['white'], course_mod.RGB['black']
    # Pool noodles round the square frame, centred on the water line
    half = (rb.FLOAT_SIDE - rb.NOODLE_D) / 2.0
    for k, (px, py, yaw) in enumerate(((half, 0, 0), (-half, 0, 0),
                                       (0, half, 1.5708), (0, -half, 1.5708))):
        visual(link, f'noodle{k}', f'{px:.4f} {py:.4f} 0 1.5708 0 {yaw}', 'cylinder',
               (rb.NOODLE_D / 2.0, rb.FLOAT_SIDE), NOODLE_RGB)
    visual(link, 'panels', '0 0 0 0 0 0', 'mesh', None, white,
           uri='file://robobuoy/meshes/panels.obj')
    symbol, top = spec.get('symbol', 'diamond'), spec.get('top', 'beacon')
    if symbol == 'diamond':
        visual(link, 'diamonds', '0 0 0 0 0 0', 'mesh', None, black,
               uri='file://robobuoy/meshes/diamonds.obj')
    else:
        # A number 1-3 drawn as that many bars, a letter as one wide bar (looks only)
        bars = int(spec.get('number', 1)) if symbol == 'digit' else 1
        out, zc = rb.FACE_MID
        for k in range(4):
            a = k * math.pi / 2.0
            for n in range(bars):
                s = (n - (bars - 1) / 2.0) * 0.04
                bx = (out + 0.004) * math.cos(a) - s * math.sin(a)
                by = (out + 0.004) * math.sin(a) + s * math.cos(a)
                width = 0.02 if symbol == 'digit' else 0.10
                visual(link, f'mark{k}_{n}', f'{bx:.4f} {by:.4f} {zc:.4f} 0 {-rb.FACE_TILT:.4f} '
                       f'{a:.4f}', 'box', (0.004, width, 0.16), black)
    visual(link, 'flange', f'0 0 {rb.BEACON_BOTTOM - rb.FLANGE / 2.0:.4f} 0 0 0', 'cylinder',
           (rb.BEACON_RADIUS + 0.01, rb.FLANGE), black)
    if top == 'beacon':
        color = spec.get('light', 'off')
        lit = color != 'off'
        visual(link, 'beacon', f'0 0 {rb.BEACON_BOTTOM + rb.BEACON_LENGTH / 2.0:.4f} 0 0 0',
               'cylinder', (rb.BEACON_RADIUS, rb.BEACON_LENGTH), LIGHT_RGB[color],
               emissive=LIGHT_RGB[color] if lit else None)
        visual(link, 'hat', f'0 0 {rb.TOP - rb.HAT_THICKNESS / 2.0:.4f} 0 0 0', 'cylinder',
               (rb.HAT_RADIUS, rb.HAT_THICKNESS), black)
    elif top == 'indicator':
        h = course_mod.INDICATOR_HEIGHT
        visual(link, 'indicator', f'0 0 {rb.BEACON_BOTTOM + h / 2.0:.4f} 0 0 0', 'cylinder',
               (course_mod.INDICATOR_RADIUS, h), course_mod.RGB[spec['indicator']])
    col = sub(link, 'collision', name='collision')
    sub(col, 'pose', f'0 0 {(rb.TOP - SUBMERGED_M) / 2.0:.3f} 0 0 0')
    sub(sub(sub(col, 'geometry'), 'box'), 'size',
        f'{rb.FLOAT_SIDE:.3f} {rb.FLOAT_SIDE:.3f} {rb.TOP + SUBMERGED_M:.3f}')
    return model


def buoy_model(name, kind, spec, x, y):
    """Build a static buoy: body (sphere or cylinder) spec['height'] above the water."""
    if spec['shape'] == 'robobuoy':
        return robobuoy_model(name, spec, x, y)
    model = ET.Element('model', name=name)
    sub(model, 'static', 'true')
    sub(model, 'pose', f'{x:.3f} {y:.3f} 0 0 0 0')
    link = sub(model, 'link', name='link')
    r = spec['diameter'] / 2.0
    h = spec['height']
    rgb = COLOURS.get(kind) or course_mod.RGB.get(spec.get('color'), (0.5, 0.5, 0.5))

    body = sub(link, 'visual', name='body')
    if spec['shape'] == 'indicator':
        # RoboBoat color indicator: white float frame, coloured cylinder on top
        base_h = h - course_mod.INDICATOR_HEIGHT
        sub(body, 'pose', f'0 0 {(base_h - 0.10) / 2:.3f} 0 0 0')
        sub(sub(sub(body, 'geometry'), 'box'), 'size',
            f'{2 * r:.3f} {2 * r:.3f} {base_h + 0.10:.3f}')
        material(body, course_mod.RGB['white'])
        body = sub(link, 'visual', name='indicator')
        sub(body, 'pose', f'0 0 {h - course_mod.INDICATOR_HEIGHT / 2:.3f} 0 0 0')
        cyl = sub(sub(body, 'geometry'), 'cylinder')
        sub(cyl, 'radius', f'{course_mod.INDICATOR_RADIUS:.4f}')
        sub(cyl, 'length', f'{course_mod.INDICATOR_HEIGHT:.3f}')
    elif spec['shape'] == 'pole':
        # RoboBoat Task 1 marker (Taylor Made Sur-Mark): tall can, narrower top third
        low = 2.0 * h / 3.0
        sub(body, 'pose', f'0 0 {(low - SUBMERGED_M) / 2:.3f} 0 0 0')
        cyl = sub(sub(body, 'geometry'), 'cylinder')
        sub(cyl, 'radius', f'{r:.3f}')
        sub(cyl, 'length', f'{low + SUBMERGED_M:.3f}')
        material(body, rgb)
        body = sub(link, 'visual', name='top')
        sub(body, 'pose', f'0 0 {(low + h) / 2:.3f} 0 0 0')
        cyl = sub(sub(body, 'geometry'), 'cylinder')
        sub(cyl, 'radius', f'{0.6 * r:.3f}')
        sub(cyl, 'length', f'{h - low:.3f}')
    elif spec['shape'] == 'sphere':
        # Top of the ball at h; the rest under water
        sub(body, 'pose', f'0 0 {h - r:.3f} 0 0 0')
        sub(sub(sub(body, 'geometry'), 'sphere'), 'radius', f'{r:.3f}')
    elif spec['shape'] == 'pear':
        # Ball at the top, narrowing to the water (and below): the TEKNOFEST pear buoy
        sub(body, 'pose', f'0 0 {h - r:.3f} 0 0 0')
        sub(sub(sub(body, 'geometry'), 'sphere'), 'radius', f'{r:.3f}')
        material(body, rgb)
        body = sub(link, 'visual', name='neck')
        neck = h - r + SUBMERGED_M
        sub(body, 'pose', f'0 0 {h - r - neck / 2:.3f} 0 0 0')
        cyl = sub(sub(body, 'geometry'), 'cylinder')
        sub(cyl, 'radius', f'{0.6 * r:.3f}')
        sub(cyl, 'length', f'{neck:.3f}')
    else:
        length = h + SUBMERGED_M
        sub(body, 'pose', f'0 0 {h - length / 2:.3f} 0 0 0')
        cyl = sub(sub(body, 'geometry'), 'cylinder')
        sub(cyl, 'radius', f'{r:.3f}')
        sub(cyl, 'length', f'{length:.3f}')
    material(body, rgb)

    if spec.get('flag'):
        pole_len = 0.8
        pole = sub(link, 'visual', name='pole')
        sub(pole, 'pose', f'0 0 {h + pole_len / 2:.3f} 0 0 0')
        cyl = sub(sub(pole, 'geometry'), 'cylinder')
        sub(cyl, 'radius', '0.012')
        sub(cyl, 'length', f'{pole_len:.3f}')
        material(pole, (0.1, 0.1, 0.1))
        flag = sub(link, 'visual', name='flag')
        sub(flag, 'pose', f'0.16 0 {h + pole_len - 0.12:.3f} 0 0 0')
        sub(sub(sub(flag, 'geometry'), 'box'), 'size', '0.30 0.005 0.20')
        material(flag, FLAG_RGB)

    col = sub(link, 'collision', name='collision')
    length = h + SUBMERGED_M
    sub(col, 'pose', f'0 0 {h - length / 2:.3f} 0 0 0')
    cyl = sub(sub(col, 'geometry'), 'cylinder')
    sub(cyl, 'radius', f'{r:.3f}')
    sub(cyl, 'length', f'{length:.3f}')
    return model


def geometry(parent, part):
    g = sub(parent, 'geometry')
    if part.shape == 'box':
        sub(sub(g, 'box'), 'size', ' '.join(f'{v:.3f}' for v in part.size))
    elif part.shape == 'cylinder':
        cyl = sub(g, 'cylinder')
        sub(cyl, 'radius', f'{part.size[0]:.4f}')
        sub(cyl, 'length', f'{part.size[1]:.3f}')
    else:
        sub(sub(g, 'sphere'), 'radius', f'{part.size[0]:.3f}')


def course_model(m):
    """Build a vessel or dock from usv_sim.course: static, one link, a visual per part."""
    model = ET.Element('model', name=m.name)
    sub(model, 'static', 'true')
    sub(model, 'pose', f'{m.x:.3f} {m.y:.3f} 0 0 0 {m.yaw:.4f}')
    link = sub(model, 'link', name='link')
    for k, part in enumerate(m.parts):
        rpy = part.pose[3:] if len(part.pose) == 6 else (0.0, 0.0, part.pose[3])
        pose = (f'{part.pose[0]:.3f} {part.pose[1]:.3f} {part.pose[2]:.3f} '
                f'{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}')
        vis = sub(link, 'visual', name=f'v{k}')
        sub(vis, 'pose', pose)
        geometry(vis, part)
        material(vis, part.rgb)
        if part.collide:
            col = sub(link, 'collision', name=f'c{k}')
            sub(col, 'pose', pose)
            geometry(col, part)
    return model


def gz_name(boat):
    """Return a boat's Gazebo model name (its topics live under /<name>/)."""
    return f'usv_{boat}'


def boat_model(boat, x, y, yaw):
    """Return a boat's model: usv_boat with its own name and topics, at (x, y, yaw)."""
    with open(BOAT_MODEL) as f:
        text = f.read()
    name = gz_name(boat)
    # Topics only (the hull mesh uri file://usv_boat/... stays: one mesh for all)
    text = re.sub(r'(<(?:topic|odom_topic)>)/usv_boat/', rf'\1/{name}/', text)
    text = text.replace('<namespace>usv_boat</namespace>', f'<namespace>{name}</namespace>')
    model = ET.fromstring(text).find('model')
    model.set('name', name)
    pose = model.find('pose')
    if pose is None:
        pose = ET.Element('pose')
        model.insert(0, pose)
    pose.text = f'{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}'
    return model


def main():
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    boats = sys.argv[3].split(',') if len(sys.argv) == 4 else ['azure']
    with open(sys.argv[1]) as f:
        course = course_mod.load(yaml.safe_load(f))
    origin = course.origin

    tree = ET.parse(BASE_WORLD)
    world = tree.getroot().find('world')

    bx, by = course.boat['east'], course.boat['north']
    yaw = math.radians(90.0 - course.boat['heading_deg'])       # compass -> ENU yaw
    for inc in list(world.findall('include')):
        uri = inc.findtext('uri', '')
        if 'nathan_benderson_park' in uri:
            if course.venue != 'nbpark':
                world.remove(inc)
        elif inc.findtext('name') == 'usv_boat':
            world.remove(inc)             # one model per boat below
    # Side by side: the k-th boat k * SPACING_M to the right of the start
    right = (math.sin(yaw), -math.cos(yaw))
    for k, boat in enumerate(boats):
        world.append(boat_model(boat, bx + k * SPACING_M * right[0],
                                by + k * SPACING_M * right[1], yaw))

    # GUI camera 12 m behind the boat, 6 m up, looking down the course
    cam = world.find('gui').find('.//camera_pose')
    if cam is not None:
        cam.text = (f'{bx - 12 * math.cos(yaw):.1f} {by - 12 * math.sin(yaw):.1f} 6 '
                    f'0 0.35 {yaw:.3f}')
    for plugin in list(world.findall('plugin')):
        if plugin.get('name') == 'vrx::USVWind':
            world.remove(plugin)

    sc = world.find('spherical_coordinates')
    sc.find('latitude_deg').text = f"{origin['lat']:.7f}"
    sc.find('longitude_deg').text = f"{origin['lon']:.7f}"

    for name, kind, spec, x, y in course.buoys:
        world.append(buoy_model(name, kind, spec, x, y))
    for m in course.models:
        world.append(course_model(m))

    ET.indent(tree, space='  ')
    header = ('<?xml version="1.0"?>\n<!-- GENERATED by scripts/make_course_world.py from '
              f'{os.path.basename(sys.argv[1])}. Edit the course file, not this one. -->\n')
    with open(sys.argv[2], 'w') as f:
        f.write(header + ET.tostring(tree.getroot(), encoding='unicode') + '\n')
    print(f'{sys.argv[2]}: {len(course.buoys)} buoys, {len(course.models)} vessels/docks, '
          f"{len(course.targets)} camera targets, origin {origin['lat']}, {origin['lon']}; "
          f"boat start x {bx:.2f} y {by:.2f} heading {course.boat['heading_deg']:.1f} deg; "
          f"boats {', '.join(gz_name(b) for b in boats)}")


if __name__ == '__main__':
    main()
