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
"""

TEKNOFEST = ('red', 'yellow', 'black', 'orange', 'green')

ROBOBOAT = ('buoy_black', 'buoy_green', 'buoy_red', 'buoy_yellow', 'digit_1', 'digit_2',
            'digit_3', 'indicator_green', 'indicator_red', 'plus', 'pole_green', 'pole_red',
            'triangle')

PROFILES = {'teknofest': TEKNOFEST, 'roboboat': ROBOBOAT}

GATE_POLES = ('pole_red', 'pole_green')
CHANNEL_BUOYS = ('buoy_red', 'buoy_green')
INDICATOR_COLOR = {'indicator_red': 'red', 'indicator_green': 'green'}
VESSEL_MARK_COLOR = {'triangle': 'yellow', 'plus': 'black'}   # mark -> vessel colour
DIGIT_NUMBER = {'digit_1': 1, 'digit_2': 2, 'digit_3': 3}


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
