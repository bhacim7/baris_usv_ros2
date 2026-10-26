#!/usr/bin/env python3
"""Generate a Gazebo world for a course file (courses/*.yaml).

The course file is the single source of truth: this script builds the world from it, and
fake_perception reads the same file for the buoy positions. Base: worlds/nbpark.sdf (VRX
physics, sensors, water, waves and our boat), with these changes:

  * the Nathan Benderson Park land model is left out: open water only
  * the VRX wind plugin is left out (it only acts on the WAM-V)
  * spherical_coordinates: the course origin, so /usv/fix reads the course's lat/lon
  * magnetic_field: the site's field (declination, inclination, strength), so the
    simulated magnetometer points where a real one would
  * the boat starts at the course's start pose
  * one static model per buoy, shaped from the course's buoy_types

Gazebo's world frame is ENU here: x east, y north, z up.

Usage (inside the container):
    python3 /ws/src/usv_sim/scripts/make_course_world.py COURSE.yaml OUTPUT.sdf
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

EARTH_RADIUS_M = 6378137.0
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
SUBMERGED_M = 0.25          # body extends this far below the water line


def enu(lat, lon, origin):
    """Metres east and north of the origin (flat Earth, fine over a few hundred metres)."""
    east = math.radians(lon - origin['lon']) * EARTH_RADIUS_M * math.cos(
        math.radians(origin['lat']))
    north = math.radians(lat - origin['lat']) * EARTH_RADIUS_M
    return east, north


def magnetic_vector_tesla(field):
    """ENU field vector from declination (east of north), inclination (down) and strength."""
    total = field['strength_uT'] * 1e-6
    inc = math.radians(field['inclination_deg'])
    dec = math.radians(field['declination_deg'])
    horizontal = total * math.cos(inc)
    return (horizontal * math.sin(dec), horizontal * math.cos(dec), -total * math.sin(inc))


def sub(parent, tag, text=None, **attrib):
    e = ET.SubElement(parent, tag, attrib)
    if text is not None:
        e.text = text
    return e


def material(parent, rgb):
    m = sub(parent, 'material')
    sub(m, 'ambient', f'{rgb[0]} {rgb[1]} {rgb[2]} 1')
    sub(m, 'diffuse', f'{rgb[0]} {rgb[1]} {rgb[2]} 1')


def buoy_model(name, kind, spec, x, y):
    """Static buoy: body (sphere or cylinder) standing spec['height'] above the water."""
    model = ET.Element('model', name=name)
    sub(model, 'static', 'true')
    sub(model, 'pose', f'{x:.3f} {y:.3f} 0 0 0 0')
    link = sub(model, 'link', name='link')
    r = spec['diameter'] / 2.0
    h = spec['height']
    rgb = COLOURS.get(kind, (0.5, 0.5, 0.5))

    body = sub(link, 'visual', name='body')
    if spec['shape'] == 'sphere':
        # Top of the ball at h; the rest under water
        sub(body, 'pose', f'0 0 {h - r:.3f} 0 0 0')
        sub(sub(sub(body, 'geometry'), 'sphere'), 'radius', f'{r:.3f}')
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


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    with open(sys.argv[1]) as f:
        course = yaml.safe_load(f)
    origin = course['origin']

    tree = ET.parse(BASE_WORLD)
    world = tree.getroot().find('world')

    boat = course['boat']
    bx, by = enu(boat['lat'], boat['lon'], origin)
    yaw = math.radians(90.0 - boat['heading_deg'])              # compass -> ENU yaw
    for inc in list(world.findall('include')):
        uri = inc.findtext('uri', '')
        if 'nathan_benderson_park' in uri:
            world.remove(inc)
        elif inc.findtext('name') == 'usv_boat':
            inc.find('pose').text = f'{bx:.3f} {by:.3f} 0 0 0 {yaw:.4f}'

    # GUI camera 12 m behind the boat, 6 m up, looking down the course
    cam = world.find('gui').find(".//camera_pose")
    if cam is not None:
        cam.text = (f'{bx - 12 * math.cos(yaw):.1f} {by - 12 * math.sin(yaw):.1f} 6 '
                    f'0 0.35 {yaw:.3f}')
    for plugin in list(world.findall('plugin')):
        if plugin.get('name') == 'vrx::USVWind':
            world.remove(plugin)

    sc = world.find('spherical_coordinates')
    sc.find('latitude_deg').text = f"{origin['lat']:.7f}"
    sc.find('longitude_deg').text = f"{origin['lon']:.7f}"

    mf = world.find('magnetic_field')
    if mf is None:
        mf = ET.Element('magnetic_field')
        world.insert(list(world).index(sc) + 1, mf)
    mf.text = '%.4e %.4e %.4e' % magnetic_vector_tesla(course['magnetic_field'])

    types = course['buoy_types']
    for i, b in enumerate(course['buoys']):
        x, y = enu(b['lat'], b['lon'], origin)
        world.append(buoy_model(f"buoy_{i:02d}_{b['type']}", b['type'], types[b['type']], x, y))

    ET.indent(tree, space='  ')
    header = (f"<?xml version=\"1.0\"?>\n<!-- GENERATED by scripts/make_course_world.py from "
              f"{os.path.basename(sys.argv[1])}. Edit the course file, not this one. -->\n")
    with open(sys.argv[2], 'w') as f:
        f.write(header + ET.tostring(tree.getroot(), encoding='unicode') + '\n')
    print(f"{sys.argv[2]}: {len(course['buoys'])} buoys, origin {origin['lat']}, "
          f"{origin['lon']}, magnetic field {mf.text} T (ENU)")


if __name__ == '__main__':
    main()
