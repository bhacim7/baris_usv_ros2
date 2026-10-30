"""
Steps 5.2-5.3 of the navigator port: the new Navigator against the legacy loop.

Both run the same scenario in the same deterministic world, to the end. Every cycle's
record (task, the five PWMs, target, angle and control error, target distance, boat pose)
must be identical - not close, identical: the same arithmetic in the same order gives the
same floats.

On a mismatch the test prints the first differing cycle and the cycles around it. The
coverage tests below check that the Task 3 scenarios really reach the branches they were
written for; if one of those fails, the scenario missed its branch - not a port error.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_golden_compare.py -v
(about a minute: every scenario runs through both loops)
"""

import contextlib
import functools
import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'golden'))

from legacy_runner import run_legacy  # noqa: E402
from new_runner import run_new  # noqa: E402
import pytest  # noqa: E402
from scenario import emergency_cycle, SCENARIOS  # noqa: E402

FIELDS = ('task', 'pwm', 'target', 'angle_error', 'control_error', 'target_dist', 'boat')
NEUTRAL_PWM = (1500, 1500, 1500, 1500, 1500)


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


@functools.lru_cache(maxsize=None)
def new_run(name):
    return run_new(SCENARIOS[name])


def t3_modes(name):
    return {r['t3'] for r in new_run(name) if r['t3'] is not None}


@pytest.mark.parametrize('name', sorted(SCENARIOS))
def test_same_as_legacy(name):
    with contextlib.redirect_stdout(io.StringIO()):
        legacy = run_legacy(SCENARIOS[name])
    new = new_run(name)

    stop = emergency_cycle(SCENARIOS[name])
    if stop is not None:
        # Legacy ran its emergency cycle with normal commands, then shut down; the new
        # navigator is neutral from that very cycle on (latch). Equal up to it.
        legacy, new = legacy[:stop], new[:stop]
    diff = first_difference(legacy, new)
    assert diff is None, describe(legacy, new, *diff)
    assert len(new) == len(legacy)


def test_emergency_stop_latches_neutral_to_the_end():
    sc = SCENARIOS['emergency_stop']
    new = new_run('emergency_stop')
    stop = emergency_cycle(sc)
    assert new[stop - 1]['pwm'] != NEUTRAL_PWM          # it was under way
    assert all(r['pwm'] == NEUTRAL_PWM for r in new[stop:])
    assert len(new) == round(sc.duration_s / 0.04)      # and kept running, neutral


def test_task3_locks_on_and_hits_the_target():
    assert {'servo', 'finished'} <= t3_modes('golcuk_task3')
    assert new_run('golcuk_task3')[-1]['task'] == 'FINISHED'


def test_task3_dropouts_exercise_grace_and_hold():
    assert {'servo', 'grace', 'hold'} <= t3_modes('task3_dropout')


def test_task3_search_flies_the_pattern():
    assert {'PAN_A', 'PAN_B', 'PAN_C', 'V_LEG'} <= t3_modes('task3_search')
