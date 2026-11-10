"""
The object classes perception reports, one list per competition profile.

Code works with class NAMES; a class id is only the position of the name in the list the
YOLO model was trained with. On the boat the ids come from the model's own name list; in
simulation from the list here. Both lists must therefore hold the same names.

  TEKNOFEST   the legacy model's classes (camera_process: red 0, yellow 1, black 2,
              orange 3, green 4); the ids the legacy navigation code uses
  ROBOBOAT    the RoboBoat dataset, renamed to English (2026-11-01), in the alphabetical
              order a dataset export gives them

RoboBoat groups (handbook 2026, section 3.2):
  pole_red / pole_green          Task 1 entry/exit gates (Taylor Made Sur-Mark, 18 x 39 in)
  buoy_red / buoy_green          Task 2 channel and Task 3 gate (Polyform A-0 / A-2)
  buoy_black                     obstacles, debris
  buoy_yellow                    Task 3 turning buoy; Task 6 zones
  indicator_red/indicator_green  color indicators (one cylinder, red OR green per run)
  triangle / plus                marks on the yellow (water) and black (ball) vessels
  digit_1 .. digit_3             Task 5 slip number banners

RoboBoat 2027 (added 2026-11-10, names as Baris will label them; to change if the dataset
names them otherwise):
  beacon_red / beacon_green      RoboBuoy with its Light Beacon lit red / green, body and
  beacon_blue / beacon_off       light labelled together (like indicator_*); off = not lit.
                                 Task 1 (RobotX 2026 rules): red and green flash 1 s on,
                                 1 s off, so a flashing beacon is beacon_off half the time;
                                 flashing blue = ENTRY, solid blue = EXIT
  window_red / window_green      Task 3 marina: a building window (LED panel) by its light
  window_blue / window_off       (the sequences: usv_sim/marina.py). Whether YOLO gives
                                 the colour or YOLO finds the building and OpenCV reads the
                                 windows is a perception choice; navigation sees these names
  buoy_blue / buoy_white         Task 2 Polyform buoys in the colours 2026 did not have
"""

TEKNOFEST = ('red', 'yellow', 'black', 'orange', 'green')

ROBOBOAT = ('beacon_blue', 'beacon_green', 'beacon_off', 'beacon_red', 'buoy_black',
            'buoy_blue', 'buoy_green', 'buoy_red', 'buoy_white', 'buoy_yellow', 'digit_1',
            'digit_2', 'digit_3', 'indicator_green', 'indicator_red', 'plus', 'pole_green',
            'pole_red', 'triangle', 'window_blue', 'window_green', 'window_off', 'window_red')

PROFILES = {'teknofest': TEKNOFEST, 'roboboat': ROBOBOAT}

GATE_POLES = ('pole_red', 'pole_green')
CHANNEL_BUOYS = ('buoy_red', 'buoy_green')
INDICATOR_COLOR = {'indicator_red': 'red', 'indicator_green': 'green'}
BEACON_COLOR = {'beacon_red': 'red', 'beacon_green': 'green', 'beacon_blue': 'blue',
                'beacon_off': 'off'}
WINDOW_COLOR = {'window_red': 'red', 'window_green': 'green', 'window_blue': 'blue',
                'window_off': 'off'}
VESSEL_MARK_COLOR = {'triangle': 'yellow', 'plus': 'black'}   # mark -> vessel colour
DIGIT_NUMBER = {'digit_1': 1, 'digit_2': 2, 'digit_3': 3}

# Horizontal radius (m) of what the camera sees of each class: the ZED measures the object's
# front, the map wants its centre (usv_core.detection.measure_boxes, add_radius; phase 6
# R1.7 D6). Handbook 2026 sizes. Red, green and black buoys come as Polyform A-0 (20.3 cm,
# channel, debris) and A-2 (36.8 cm, Task 3): the A-0 value is used, the more common one.
# Flat marks (vessel marks, slip banners) are seen face on: 0.
OBJECT_RADIUS_M = {
    'pole_red': 0.229, 'pole_green': 0.229,                 # Sur-Mark, 18 in
    'buoy_red': 0.102, 'buoy_green': 0.102, 'buoy_black': 0.102,    # A-0
    'buoy_yellow': 0.184,                                   # A-2
    'indicator_red': 0.064, 'indicator_green': 0.064,       # 5 in cylinder
    'triangle': 0.0, 'plus': 0.0, 'digit_1': 0.0, 'digit_2': 0.0, 'digit_3': 0.0,
    # RoboBuoy: its float frame at the water, ~19 in square (as the simulation measures
    # it); on the real buoy the ZED may hit the sloping face, further in: to be measured
    'beacon_red': 0.24, 'beacon_green': 0.24, 'beacon_blue': 0.24, 'beacon_off': 0.24,
    'window_red': 0.0, 'window_green': 0.0, 'window_blue': 0.0, 'window_off': 0.0,
    'buoy_blue': 0.184, 'buoy_white': 0.184,            # ASSUMED A-2 (Task 2 drawing)
}


def class_names(profile):
    """Return the class name list of a profile ('teknofest' or 'roboboat')."""
    try:
        return PROFILES[profile]
    except KeyError:
        raise ValueError(f'unknown class profile {profile!r}; one of {sorted(PROFILES)}')


def class_id(profile, name):
    """Return the class id of a name in a profile; ValueError if it has no such class."""
    names = class_names(profile)
    if name not in names:
        raise ValueError(f'{name!r} is not a {profile} class')
    return names.index(name)
