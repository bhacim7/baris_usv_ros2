"""
usv_core.allocation: which boat gets a task from RoboCommand (step 6e).

Run: cd /ws/src/usv_core && python3 -m pytest test/test_allocation.py -v
"""

from usv_core.allocation import allocate, BoatState, needs, share_out


def boats(**changes):
    azure = BoatState(1, 'azure', (), heard_s=0.2, manual=False, task='HOME')
    marine = BoatState(2, 'marine', ('water', 'balls', 'turret_camera'), heard_s=0.3,
                       manual=False, task='HOME')
    for name, fields in changes.items():
        for k, v in fields.items():
            setattr(azure if name == 'azure' else marine, k, v)
    return [azure, marine]


def test_water_goes_to_marine_even_when_azure_is_free():
    assert needs('task4') == 'water' and needs('rally') is None
    assert allocate('task4', boats()) == (2, '')
    assert allocate('task4', boats(marine={'task': 'task1', 'upcoming': ['a', 'b']})) == (2, '')


def test_free_boat_first_then_fewer_tasks_then_the_lower_number():
    assert allocate('rally', boats()) == (1, '')                       # both free: Azure
    assert allocate('rally', boats(azure={'task': 'task1'})) == (2, '')  # Marine free
    busy = boats(azure={'task': 'task1', 'upcoming': ['x', 'y']},
                 marine={'task': 'marina', 'upcoming': ['z']})
    assert allocate('rally', busy) == (2, '')


def test_boats_not_heard_or_in_manual_get_nothing():
    assert allocate('rally', boats(azure={'heard_s': 9.0})) == (2, '')
    assert allocate('rally', boats(azure={'manual': True})) == (2, '')
    number, reason = allocate('task4', boats(marine={'heard_s': None}))
    assert number is None and 'marine not heard' in reason and 'azure cannot do water' in reason


def test_a_named_boat_and_unknown_tasks():
    assert allocate('rally', boats(), named=2) == (2, '')
    assert allocate('task4', boats(), named=1) == (None, 'azure cannot do water')
    assert allocate('rally', boats(), named=7) == (None, 'no boat number 7')
    assert allocate('fly', boats()) == (None, 'unknown task fly')


def test_a_badge_is_shared_out_between_the_boats():
    out = share_out(['task1', 'marina', 'task5'], boats())
    assert [n for _, n, _ in out] == [1, 2, 1]
    out = share_out(['task4', 'task1'], boats())
    assert [n for _, n, _ in out] == [2, 1]
