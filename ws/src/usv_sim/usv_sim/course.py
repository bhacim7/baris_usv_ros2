"""Course files (courses/*.yaml): one reader for the world generator and fake_perception.

Both take the course's geometry from here, so what Gazebo draws and what the simulated
camera sees cannot drift apart. ROS-free, unit tested (test/test_course.py).

A course file has:
  name, origin {lat, lon}, magnetic_field, boat {lat/lon or east/north, heading_deg}
  venue          optional: 'nbpark' keeps the Nathan Benderson Park land model in the world
  anchor         optional {east, north, rotation_deg}: east/north positions in the file are
                 laid out around this point of the world, turned by rotation_deg
                 (counter-clockwise), so a course drawn north-up can be fitted into the lake;
                 headings in the file turn with it
  classes        'teknofest' (default) or 'roboboat': the class list (usv_core.classes)
  mission        optional, navigator overrides (see sim.launch.py)
  buoy_types     shapes: sphere, pear, cylinder (legacy), pole, indicator; class by name
                 (`class: pole_red`) or legacy id (`class_id: 3`)
  buoys          {type, lat/lon or east/north}
  vessels        {color: yellow|black, heading_deg, position}: RoboBoat stationary vessels
                 with a triangle (yellow) or plus (black) on both sides
  docks          {name, heading_deg, tines: left|right, slips: [{indicator, vessel}] x 3,
                 position of the start of the dock's spine}: RoboBoat floating docks
Positions: `lat`/`lon`, or `east`/`north` in metres from the origin.

Dimensions are the RoboBoat 2026 handbook's where it gives them; everything marked
ASSUMED is a guess to be replaced when known.
"""

from dataclasses import dataclass, field
import math

from usv_core.classes import class_names
from usv_sim.camera import Buoy, Panel
from usv_sim.geo import enu

# --- RoboBoat elements (handbook 2026, section 3.2) ---------------------------------
INDICATOR_RADIUS = 0.0635         # dock indicators: 5 in wide, 5.2 in tall; the ones on
INDICATOR_HEIGHT = 0.132          # buoys taken the same (ASSUMED)
VESSEL_LENGTH = 1.70              # 5.5 in stern + 42 in body + ~0.5 m bow (drawing)
VESSEL_WIDTH = 0.686              # 27 in
VESSEL_FREEBOARD = 0.30           # ASSUMED: side height above the water
MARK_SIZE = 0.30                  # ASSUMED: triangle / plus panels
CUBE = 0.508                      # dock cube 20 x 20 in
DOCK_FREEBOARD = 0.15             # ASSUMED: cube top above the water (16 in cube, floating)
SPINE_LENGTH = 8.636              # 340 in
SPINE_WIDTH = 2 * CUBE            # 40 in
TINE_LENGTH = 1.83                # 4-7 ft; 6 ft used
SLIP_WIDTH = 1.524                # 60 in
FIRST_TINE_X = 0.60               # ASSUMED: spine before the first tine
BANNER_SIZE = 0.61                # 24 x 24 in
BANNER_BOTTOM = 0.30              # ASSUMED: banner on a post, bottom edge above the water

RGB = {
    'red': (0.75, 0.02, 0.02), 'green': (0.0, 0.55, 0.12), 'black': (0.04, 0.04, 0.04),
    'yellow': (1.0, 0.82, 0.0), 'white': (0.95, 0.95, 0.95), 'beige': (0.80, 0.72, 0.58),
    'grey': (0.5, 0.5, 0.5),
}


@dataclass
class Part:
    """One SDF primitive of a model, in the model's frame (x forward, y left, z up)."""

    shape: str                    # box, cylinder or sphere
    size: tuple                   # box (sx, sy, sz); cylinder (radius, length); sphere (r,)
    pose: tuple                   # (x, y, z, yaw)
    rgb: tuple
    collide: bool = True


@dataclass
class Model:
    name: str
    x: float                      # ENU
    y: float
    yaw: float                    # rad, ENU
    parts: list = field(default_factory=list)


@dataclass
class Course:
    name: str
    origin: dict
    magnetic_field: dict
    boat: dict                    # east, north, heading_deg
    profile: str
    class_names: tuple
    mission: dict
    venue: str                    # '' = open water only
    buoys: list                   # (name, type, spec, east, north) for the world generator
    models: list                  # Model, for vessels and docks
    targets: list                 # Buoy / Panel: what the camera can report

    def class_ids(self):
        return tuple(range(len(self.class_names)))


def compass_to_yaw(heading_deg):
    return math.radians(90.0 - heading_deg)


def position(item, origin, anchor=None):
    """World ENU metres of an item: lat/lon as they are, east/north around the anchor."""
    if 'east' in item:
        e, n = float(item['east']), float(item['north'])
        if anchor:
            r = math.radians(anchor.get('rotation_deg', 0.0))
            e, n = (anchor['east'] + e * math.cos(r) - n * math.sin(r),
                    anchor['north'] + e * math.sin(r) + n * math.cos(r))
        return e, n
    return enu(item['lat'], item['lon'], origin['lat'], origin['lon'])


def heading(item, anchor=None):
    """Compass heading of an item, turned with the anchor (counter-clockwise = smaller)."""
    return float(item['heading_deg']) - (anchor or {}).get('rotation_deg', 0.0)


def local_to_enu(x0, y0, yaw, lx, ly):
    c, s = math.cos(yaw), math.sin(yaw)
    return x0 + lx * c - ly * s, y0 + lx * s + ly * c


def load(data):
    """Build a Course from a parsed course file (dict)."""
    origin = data['origin']
    anchor = data.get('anchor')
    profile = data.get('classes', 'teknofest')
    names = class_names(profile)

    def cid(spec_or_name):
        if isinstance(spec_or_name, str):
            return names.index(spec_or_name)
        if 'class' in spec_or_name:
            return names.index(spec_or_name['class'])
        return int(spec_or_name['class_id'])

    bx, by = position(data['boat'], origin, anchor)
    boat = {'east': bx, 'north': by, 'heading_deg': heading(data['boat'], anchor)}

    buoys, targets, models = [], [], []
    types = data.get('buoy_types', {})
    for i, b in enumerate(data.get('buoys', [])):
        spec = types[b['type']]
        x, y = position(b, origin, anchor)
        buoys.append((f"buoy_{i:02d}_{b['type']}", b['type'], spec, x, y))
        if spec['shape'] == 'indicator':
            top = spec['height']
            targets.append(Buoy(x, y, INDICATOR_RADIUS, top, cid(spec),
                                z0=top - INDICATOR_HEIGHT))
        else:
            targets.append(Buoy(x, y, spec['diameter'] / 2.0, spec['height'], cid(spec)))

    for i, v in enumerate(data.get('vessels', [])):
        x, y = position(v, origin, anchor)
        model, panels = vessel(f'vessel_{i:02d}_{v["color"]}', v['color'], x, y,
                               compass_to_yaw(heading(v, anchor)), names)
        models.append(model)
        targets.extend(panels)

    for d in data.get('docks', []):
        x, y = position(d, origin, anchor)
        dock_models, dock_targets = dock(d, x, y, heading(d, anchor), names)
        models.extend(dock_models)
        targets.extend(dock_targets)

    return Course(name=data['name'], origin=origin, magnetic_field=data['magnetic_field'],
                  boat=boat, profile=profile, class_names=names,
                  mission=data.get('mission', {}), venue=data.get('venue', ''),
                  buoys=buoys, models=models, targets=targets)


def vessel(name, color, x, y, yaw, names):
    """A stationary vessel and the two marks on its sides (camera panels)."""
    mark = 'triangle' if color == 'yellow' else 'plus'
    model = Model(name, x, y, yaw)
    h = VESSEL_FREEBOARD
    model.parts.append(Part('box', (VESSEL_LENGTH, VESSEL_WIDTH, h + 0.10),
                            (0.0, 0.0, (h - 0.10) / 2.0, 0.0), RGB[color]))
    panels = []
    for side in (1, -1):
        sy = side * (VESSEL_WIDTH / 2.0 + 0.003)
        z0, z1 = 0.03, 0.03 + MARK_SIZE
        model.parts.extend(mark_parts(mark, sy, (z0 + z1) / 2.0,
                                      white_back=(color == 'black')))
        px, py = local_to_enu(x, y, yaw, 0.0, sy)
        panels.append(Panel(px, py, z0, z1, MARK_SIZE,
                            math.degrees(yaw) + 90.0 * side, names.index(mark)))
    return model, panels


def mark_parts(mark, y, zc, white_back):
    """Thin boxes drawing a triangle or a plus on a vessel side (looks only)."""
    parts = []
    t = 0.004
    if white_back:
        parts.append(Part('box', (MARK_SIZE + 0.06, t, MARK_SIZE + 0.06), (0.0, y, zc, 0.0),
                          RGB['white'], collide=False))
        y = y + math.copysign(t, y)
    if mark == 'plus':
        bar = MARK_SIZE / 3.0
        parts.append(Part('box', (MARK_SIZE, t, bar), (0.0, y, zc, 0.0), RGB['black'], False))
        parts.append(Part('box', (bar, t, MARK_SIZE), (0.0, y, zc, 0.0), RGB['black'], False))
    else:
        rows = 5
        for k in range(rows):
            w = MARK_SIZE * (rows - k) / rows
            z = zc - MARK_SIZE / 2.0 + (k + 0.5) * MARK_SIZE / rows
            parts.append(Part('box', (w, t, MARK_SIZE / rows), (0.0, y, z, 0.0),
                              RGB['black'], False))
    return parts


def dock(d, x0, y0, heading_deg, names):
    """A floating dock: spine, four tines making three slips, banners, indicators."""
    yaw = compass_to_yaw(heading_deg)
    s = 1 if d.get('tines', 'left') == 'left' else -1
    model = Model(f"dock_{d['name']}", x0, y0, yaw)
    h = DOCK_FREEBOARD
    zc = (h - 0.25) / 2.0                       # cubes reach 0.25 m below the water
    model.parts.append(Part('box', (SPINE_LENGTH, SPINE_WIDTH, h + 0.25),
                            (SPINE_LENGTH / 2.0, 0.0, zc, 0.0), RGB['beige']))
    pitch = CUBE + SLIP_WIDTH
    for k in range(4):
        tx = FIRST_TINE_X + k * pitch + CUBE / 2.0
        ty = s * (SPINE_WIDTH / 2.0 + TINE_LENGTH / 2.0)
        model.parts.append(Part('box', (CUBE, TINE_LENGTH, h + 0.25), (tx, ty, zc, 0.0),
                                RGB['beige']))
    models, targets = [model], []
    face = math.degrees(yaw) + 90.0 * s          # banners look into the slips
    for k, slip in enumerate(d['slips']):
        number = k + 1
        sx = FIRST_TINE_X + CUBE + SLIP_WIDTH / 2.0 + k * pitch
        edge = s * SPINE_WIDTH / 2.0
        # Banner on a post at the spine edge
        z0, z1 = BANNER_BOTTOM, BANNER_BOTTOM + BANNER_SIZE
        model.parts.append(Part('cylinder', (0.02, z1), (sx, edge - s * 0.05, z1 / 2.0, 0.0),
                                RGB['grey'], collide=False))
        model.parts.append(Part('box', (BANNER_SIZE, 0.01, BANNER_SIZE),
                                (sx, edge, (z0 + z1) / 2.0, 0.0), RGB['black'], False))
        for n in range(number):                  # white bars: 1, 2 or 3 (looks only)
            bx = sx + (n - (number - 1) / 2.0) * 0.12
            model.parts.append(Part('box', (0.05, 0.004, 0.36),
                                    (bx, edge + s * 0.007, (z0 + z1) / 2.0, 0.0),
                                    RGB['white'], False))
        px, py = local_to_enu(x0, y0, yaw, sx, edge)
        targets.append(Panel(px, py, z0, z1, BANNER_SIZE, face,
                             names.index(f'digit_{number}')))
        # Color indicator on the spine, beside the banner
        color = slip['indicator']
        ix, iy = sx + 0.45, edge - s * 0.12
        model.parts.append(Part('cylinder', (INDICATOR_RADIUS, INDICATOR_HEIGHT),
                                (ix, iy, h + INDICATOR_HEIGHT / 2.0, 0.0), RGB[color], False))
        px, py = local_to_enu(x0, y0, yaw, ix, iy)
        targets.append(Buoy(px, py, INDICATOR_RADIUS, h + INDICATOR_HEIGHT,
                            names.index(f'indicator_{color}'), z0=h))
        # A vessel moored in the slip, bow to the spine
        if slip.get('vessel'):
            vx, vy = local_to_enu(x0, y0, yaw, sx,
                                  s * (SPINE_WIDTH / 2.0 + VESSEL_LENGTH / 2.0 + 0.10))
            vyaw = yaw - s * math.pi / 2.0
            v_model, v_panels = vessel(f"dock_{d['name']}_slip{number}_{slip['vessel']}",
                                       slip['vessel'], vx, vy, vyaw, names)
            models.append(v_model)
            targets.extend(v_panels)
    return models, targets
