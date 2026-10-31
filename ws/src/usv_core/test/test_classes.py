"""
usv_core.classes: the class lists perception, simulation and navigation share.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_classes.py -v
"""

import pytest
from usv_core import classes


def test_teknofest_ids_are_the_legacy_ones():
    # camera_process / object memory: red 0, yellow 1, black 2, orange 3, green 4
    assert classes.TEKNOFEST.index('red') == 0
    assert classes.TEKNOFEST.index('orange') == 3
    assert classes.class_id('teknofest', 'green') == 4


def test_roboboat_has_the_thirteen_dataset_classes_sorted():
    assert len(classes.ROBOBOAT) == 13
    assert list(classes.ROBOBOAT) == sorted(classes.ROBOBOAT)
    assert len(set(classes.ROBOBOAT)) == 13


def test_groups_only_name_existing_classes():
    names = set(classes.ROBOBOAT)
    for group in (classes.GATE_POLES, classes.CHANNEL_BUOYS, classes.INDICATOR_COLOR,
                  classes.VESSEL_MARK_COLOR, classes.DIGIT_NUMBER):
        assert set(group) <= names


def test_unknown_names_are_errors():
    with pytest.raises(ValueError):
        classes.class_id('roboboat', 'orange')
    with pytest.raises(ValueError):
        classes.class_names('suas')
