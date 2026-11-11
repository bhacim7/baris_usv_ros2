"""
The RoboBoat navigator with the 2027 mission manager (step 6b), and the readiness run kept.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_mission_navigator.py -v
"""

from usv_core.mission import competition_factories, HOME, MissionConfig, MissionManager, Request
from usv_core.roboboat_navigator import Perception, RoboBoatNavigator
from usv_core.roboboat_run import RoboBoatRun

FIX = (27.37, -82.45)


def navigator(plan=('task1', 'marina'), capabilities=()):
    logs = []
    mm = MissionManager(MissionConfig(plan=plan), competition_factories(), capabilities)
    nav = RoboBoatNavigator(mission=mm, log=logs.append, mission_started=True, manual=False)
    return nav, logs


def test_competition_plan_skips_tasks_not_written_and_goes_home():
    nav, logs = navigator()
    assert nav.upcoming() == ['task1', 'marina']
    for k in range(3):
        nav.step(float(k), FIX, 0.0, 270.0, Perception())
    assert nav.task == HOME and nav.upcoming() == []
    assert 'task1: skipped (not written yet)' in logs
    assert nav.status['current_task'] == HOME


def test_requests_reach_the_mission_manager_and_capabilities_count():
    nav, logs = navigator(plan=('task1', 'marina', 'task5'))
    assert not nav.request(Request('task4', urgent=True, source='robocommand'))
    assert any('cannot do water' in line for line in logs)
    marine, _ = navigator(capabilities=('water', 'balls'))
    assert marine.request(Request('task4'))
    assert marine.upcoming()[0] == 'task4'


def test_readiness_run_is_the_frozen_one_and_takes_no_requests():
    nav = RoboBoatNavigator(log=lambda m: None)
    assert isinstance(nav.run, RoboBoatRun)
    assert not nav.request(Request('rally')) and nav.upcoming() == []
