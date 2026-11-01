"""
The RoboBoat run (usv_core.roboboat_run) and navigator (usv_core.roboboat_navigator).

Run: cd /ws/src/usv_core && python3 -m pytest test/test_roboboat_run.py -v
"""

import pytest
from usv_core.mixer import NEUTRAL
from usv_core.roboboat_navigator import Perception, RoboBoatNavigator
from usv_core.roboboat_run import (DONE, FINISHED, Pending, RETURN, RoboBoatRun, RunConfig,
                                   RunContext, RUNNING, T1_ENTRY, T1_EXIT, WAIT_START)


class Scripted:
    """A phase that drives for `cycles` steps, then reports `end`."""

    def __init__(self, cycles=3, end=DONE, command=(1680, 0)):
        self.cycles, self.end, self.command = cycles, end, command
        self.starts = 0
        self.steps = 0

    def start(self, ctx):
        self.starts += 1
        self.steps = 0

    def step(self, ctx):
        self.steps += 1
        if self.steps > self.cycles:
            return self.end, None
        return RUNNING, self.command


def run_until(run, active=True, t0=0.0, dt=0.04, limit=10000):
    t = t0
    commands = []
    for _ in range(limit):
        commands.append(run.step(RunContext(now=t), active))
        t += dt
        if run.phase == FINISHED:
            break
    return commands, t


def kinds(run):
    return [(kind, d.get('phase')) for _, kind, d in run.events]


def test_unwritten_phases_are_skipped_through_to_finished():
    run = RoboBoatRun(RunConfig(task_order=('task2', 'task3')))
    assert run.plan == [T1_ENTRY, 'task2', 'task3', RETURN, T1_EXIT]
    commands, _ = run_until(run)
    assert run.phase == FINISHED and all(c is None for c in commands)
    assert kinds(run)[0] == ('run_started', None)
    assert ('phase_skipped', 'task3') in kinds(run)
    assert kinds(run)[-1] == ('run_finished', None)


def test_nothing_happens_until_the_mission_is_active():
    run = RoboBoatRun()
    for k in range(50):
        assert run.step(RunContext(now=k * 0.04), active=False) is None
    assert run.phase == WAIT_START and run.events == []


def test_a_running_phase_drives_then_hands_over():
    entry = Scripted(cycles=3, command=(1680, 40))
    run = RoboBoatRun(RunConfig(task_order=()), phases={T1_ENTRY: entry})
    commands, _ = run_until(run)
    assert commands[:3] == [(1680, 40)] * 3 and entry.starts == 1
    assert ('phase_done', T1_ENTRY) in kinds(run)


def test_a_task_that_takes_too_long_is_skipped():
    stuck = Scripted(cycles=10 ** 9)
    run = RoboBoatRun(RunConfig(task_order=('task2',), task_timeout_s=5.0),
                      phases={'task2': stuck})
    _, t = run_until(run)
    assert ('phase_timeout', 'task2') in kinds(run)
    assert 5.0 <= t < 5.5 and run.phase == FINISHED


def test_run_timeout_returns_but_never_cuts_the_exit_short():
    stuck = Scripted(cycles=10 ** 9)
    exit_gates = Scripted(cycles=500)                 # 20 s of exit, past every timeout
    run = RoboBoatRun(RunConfig(task_order=('task2',), task_timeout_s=60.0,
                                run_timeout_s=10.0),
                      phases={'task2': stuck, T1_EXIT: exit_gates})
    run_until(run)
    assert ('run_timeout', 'task2') in kinds(run)
    assert ('phase_done', T1_EXIT) in kinds(run) and exit_gates.steps == 501


def test_paused_in_manual_keeps_the_phase():
    task = Scripted(cycles=5)
    run = RoboBoatRun(RunConfig(task_order=('task2',)), phases={'task2': task})
    run.step(RunContext(now=0.0), True)               # T1_ENTRY skipped -> task2
    run.step(RunContext(now=0.04), True)
    assert run.phase == 'task2' and task.steps == 1
    for k in range(10):
        assert run.step(RunContext(now=0.1 + k), False) is None
    assert run.phase == 'task2' and task.steps == 1


def test_jump_and_restart():
    task = Scripted(cycles=100)
    run = RoboBoatRun(RunConfig(task_order=('task2', 'task3')), phases={'task3': task})
    assert run.jump('task3', 1.0) and run.phase == 'task3'
    run.step(RunContext(now=1.04), True)
    assert task.starts == 1
    run.restart_phase()
    run.step(RunContext(now=1.08), True)
    assert task.starts == 2
    assert not run.jump('task9', 2.0)
    with pytest.raises(ValueError):
        RoboBoatRun(RunConfig(task_order=('task1',)))


# ---------------------------------------------------------------- navigator
def make_nav(phases=None, **kw):
    """Navigator with the written phases left out, unless phases says otherwise."""
    logs = []
    skeleton = {name: Pending(name) for name in (T1_ENTRY, 'task2', RETURN, T1_EXIT)}
    skeleton.update(phases or {})
    nav = RoboBoatNavigator(run_cfg=RunConfig(task_order=('task2',)), manual=True,
                            log=logs.append, phases=skeleton, **kw)
    return nav, logs


def drive(nav, t0, cycles, perception=None):
    t = t0
    out = None
    for _ in range(cycles):
        out = nav.step(t, (27.37, -82.45), 1.0, 90.0, perception or Perception())
        t += 0.04
    return out, t


def test_navigator_waits_in_manual_then_runs():
    nav, _ = make_nav()
    out, t = drive(nav, 0.0, 10)
    assert out == NEUTRAL and nav.task == WAIT_START
    nav.submit({'cmd': 'set_manual', 'value': False})
    out, t = drive(nav, t, 20)
    assert nav.task == FINISHED and out == NEUTRAL
    assert nav.status['current_task'] == FINISHED and nav.status['manual_mode'] is False


def test_navigator_emergency_latch_and_release_restart_the_phase():
    entry = Scripted(cycles=10 ** 6)
    nav, logs = make_nav(phases={T1_ENTRY: entry})
    nav.submit({'cmd': 'set_manual', 'value': False})
    out, t = drive(nav, 0.0, 5)
    assert out.rear_left > 1500                       # driving
    nav.submit({'cmd': 'emergency_stop'})
    out, t = drive(nav, t, 3)
    assert out.rear_left == 1500 and out.force and nav.status['emergency']
    nav.relay_on = 0
    nav.submit({'cmd': 'set_task', 'task_name': T1_ENTRY})
    out, t = drive(nav, t, 3)
    assert nav.emergency.active                       # relay off: stays latched
    nav.relay_on = 1
    nav.submit({'cmd': 'set_task', 'task_name': T1_ENTRY})
    out, t = drive(nav, t, 3)
    assert not nav.emergency.active and entry.starts >= 2 and out.rear_left > 1500


def test_navigator_ignores_gps_points_and_unknown_phases():
    nav, logs = make_nav()
    nav.submit({'cmd': 'set_gps', 'index': 1, 'lat': 1.0, 'lon': 2.0})
    nav.submit({'cmd': 'set_task', 'task_name': 'TASK2_START'})
    drive(nav, 0.0, 2)
    assert any('ignored' in m for m in logs) and any('no such phase' in m for m in logs)
    assert nav.task == WAIT_START


def test_navigator_reads_the_course_from_the_map():
    poles = [('pole_red', -2.28, 0.0), ('pole_green', 6.38, 0.0),
             ('pole_red', -2.28, 7.95), ('pole_green', 6.38, 7.95)]
    lms = [{'id': i, 'kind': k, 'color': '', 'east': x, 'north': y, 'confirmed': True,
            'sightings': 40} for i, (k, x, y) in enumerate(poles)]
    nav, logs = make_nav()
    origin = (27.37, -82.45)                          # the boat sits on the map origin
    drive(nav, 0.0, 3, Perception(landmarks=lms, origin=origin))
    assert nav.course.frame is not None and len(nav.course.t1_gates) == 2
    assert nav.course.frame.ahead == pytest.approx((0.0, 1.0), abs=0.01)
    assert any('Task 1 gates: 2' in m for m in logs)
    nav.submit({'cmd': 'set_manual', 'value': False})
    drive(nav, 1.0, 2, Perception(landmarks=lms, origin=origin))
    assert nav.run.memory['start_xy'] == pytest.approx((0.0, 0.0), abs=0.01)
