"""
Step 5.2 of the navigator port: the new Navigator against the legacy loop, cycle by cycle.

Both run the same scenario in the same deterministic world. Every cycle's record (task,
the five PWMs, target, angle and control error, target distance, boat pose) must be
identical - not close, identical: the same arithmetic in the same order gives the same
floats. A run stops at the first TASK3_SEARCH_KAMIKAZE cycle (the search is phase 5.3).

On a mismatch the test prints the first differing cycle and the cycles around it.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_golden_compare.py -v
(about half a minute: every scenario runs through both loops)
"""

import contextlib
import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'golden'))

from legacy_runner import run_legacy  # noqa: E402
from new_runner import run_new  # noqa: E402
import pytest  # noqa: E402
from scenario import SCENARIOS  # noqa: E402

FIELDS = ('task', 'pwm', 'target', 'angle_error', 'control_error', 'target_dist', 'boat')


def first_difference(legacy, new):
    for a, b in zip(legacy, new):
        diff = [f for f in FIELDS if a[f] != b[f]]
        if diff:
            return a['cycle'], diff
    return None


def describe(legacy, new, cycle, fields):
    lines = [f'first difference at cycle {cycle} (t={legacy[cycle]["t"]:.2f} s) in {fields}']
    for c in range(max(0, cycle - 2), min(len(new), cycle + 3)):
        lines.append(f'  cycle {c}')
        for f in FIELDS:
            mark = '  <--' if legacy[c][f] != new[c][f] else ''
            lines.append(f'    {f:<14} legacy {legacy[c][f]!s:<44} new {new[c][f]!s}{mark}')
    return '\n'.join(lines)


@pytest.mark.parametrize('name', sorted(SCENARIOS))
def test_same_as_legacy(name):
    sc = SCENARIOS[name]
    with contextlib.redirect_stdout(io.StringIO()):
        legacy = run_legacy(sc)
    new = run_new(sc)

    assert len(new) <= len(legacy)
    if new[-1]['task'] != 'TASK3_SEARCH_KAMIKAZE':
        assert len(new) == len(legacy)
    diff = first_difference(legacy, new)
    assert diff is None, describe(legacy, new, *diff)


def test_stops_where_the_task3_search_begins():
    new = run_new(SCENARIOS['golcuk_task3'])
    assert new[-1]['task'] == 'TASK3_SEARCH_KAMIKAZE'
    assert all(r['task'] != 'TASK3_SEARCH_KAMIKAZE' for r in new[:-1])
