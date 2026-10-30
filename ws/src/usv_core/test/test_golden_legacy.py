"""Step 5.1 of the navigator port: the legacy nav_worker runs unchanged in the golden harness.

These tests only check the harness itself - the legacy loop runs every scenario to the end
without crashing, repeatably, and reacts to the inputs it is given. The comparison with the
new navigator is added in step 5.2 onwards, on the same scenarios.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_golden_legacy.py -v
(slow: several minutes, the legacy loop builds a costmap and plans A* in pure Python)
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'golden'))
from legacy_runner import run_legacy  # noqa: E402
from scenario import SCENARIOS  # noqa: E402

PERIOD = 1.0 / 25.0


def run(name, capsys):
    records = run_legacy(SCENARIOS[name])
    out = capsys.readouterr().out
    assert 'Brain crashed' not in out, out[-2000:]
    assert 'Iteration failed' not in out, out[-2000:]
    return records


@pytest.mark.parametrize('name', sorted(SCENARIOS))
def test_scenario_runs_to_the_end(name, capsys):
    sc = SCENARIOS[name]
    records = run(name, capsys)
    expected = round(sc.duration_s / PERIOD)
    finished = [r for r in records if r['task'] == 'FINISHED']
    if sc.stop_when_finished and finished:
        assert len(records) <= expected
    else:
        assert len(records) == expected
    assert [r['cycle'] for r in records] == list(range(len(records)))
    assert records[1]['t'] == pytest.approx(PERIOD)


def test_runs_are_repeatable(capsys):
    a = run('gcs_commands', capsys)
    b = run('gcs_commands', capsys)
    assert a == b


def test_manual_mode_never_moves_and_never_advances(capsys):
    records = run('manual_rollback', capsys)
    assert {r['task'] for r in records} == {'TASK2_START'}
    assert {r['pwm'] for r in records} == {(1500, 1500, 1500, 1500, 1500)}


def test_auto_mode_drives_and_manual_stops_again(capsys):
    records = run('manual_then_auto', capsys)
    before = [r for r in records if r['t'] < 5.0]
    during = [r for r in records if 8.0 <= r['t'] < 25.0]
    after = [r for r in records if r['t'] >= 26.0]
    assert {r['pwm'] for r in before} == {(1500, 1500, 1500, 1500, 1500)}
    assert any(r['pwm'] != (1500, 1500, 1500, 1500, 1500) for r in during)
    assert after[-1]['pwm'] == (1500, 1500, 1500, 1500, 1500)


def test_gcs_set_task_is_obeyed_and_foreign_commands_ignored(capsys):
    records = run('gcs_commands', capsys)
    assert any(r['task'] == 'T3_START' for r in records if 10.0 < r['t'] < 11.0)
    # the command addressed to vehicle 99 must not have sent the boat back to Task 2
    assert all(not r['task'].startswith('TASK2') for r in records if r['t'] > 20.5)
