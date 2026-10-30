"""
Emergency stop latch and pre-flight check (usv_core.safety), and how the Navigator uses them.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_safety.py -v
"""

from usv_core.nav_config import NavConfig
from usv_core.navigator import Navigator
from usv_core.safety import EmergencyLatch, preflight_ready

NEUTRAL = (1500, 1500, 1500, 1500, 1500)
BOAT = (40.8085, 29.2615)        # ~70 m south-west of the legacy T1_GATE_ENTER
DT = 0.04


def pwm(cmd):
    return (cmd.rear_left, cmd.rear_right, cmd.front_left, cmd.front_right, cmd.steer)


class Run:
    """A Navigator in AUTO, Task 1, standing still at BOAT and heading north."""

    def __init__(self):
        self.t = 100.0
        self.nav = Navigator(NavConfig(), start_time=self.t, mission_started=True,
                             manual=False)

    def step(self, *commands):
        for c in commands:
            self.nav.submit(c)
        self.t += DT
        return self.nav.step(self.t, BOAT, 0.0, 0.0, [])


def test_latch_keeps_the_first_reason():
    latch = EmergencyLatch()
    assert not latch.active
    latch.trip('first', 1.0)
    latch.trip('second', 2.0)
    assert (latch.active, latch.reason, latch.tripped_at) == (True, 'first', 1.0)
    latch.reset()
    assert (latch.active, latch.reason, latch.tripped_at) == (False, None, None)


def test_preflight_is_the_legacy_check():
    assert not preflight_ready(0.0, 0.0)
    assert not preflight_ready(40.7, 0.0)        # heading source silent (it publishes 0.0)
    assert not preflight_ready(0.0, 146.0)       # no GPS fix yet
    assert preflight_ready(40.7, 146.0)


def test_emergency_neutralises_in_the_same_cycle_and_holds():
    run = Run()
    for _ in range(10):
        cmd = run.step()
    assert pwm(cmd) != NEUTRAL                   # spot-turning toward the gate

    cmd = run.step({'cmd': 'emergency_stop'})
    assert pwm(cmd) == NEUTRAL
    assert cmd.force                             # bypasses slew limiting and write thinning
    assert run.nav.status['emergency'] is True
    assert run.nav.status['emergency_reason'] == 'GCS emergency_stop'

    # AUTO and task commands are still taken in, but nothing moves
    for c in ({'cmd': 'set_manual', 'value': False},
              {'cmd': 'set_task', 'task_name': 'TASK2_START'}):
        cmd = run.step(c)
        assert pwm(cmd) == NEUTRAL
    for _ in range(50):
        assert pwm(run.step()) == NEUTRAL
    assert run.nav.task == 'TASK2_START'


def test_reset_comes_back_in_manual():
    run = Run()
    for _ in range(10):
        run.step()
    run.step({'cmd': 'emergency_stop'})
    run.nav.reset_emergency()
    assert run.nav.manual is True
    assert run.nav.status['emergency'] is True   # status refreshes on the next step

    cmd = run.step()
    assert pwm(cmd) == NEUTRAL                   # manual: stays still
    assert run.nav.status['emergency'] is False

    for _ in range(5):
        cmd = run.step({'cmd': 'set_manual', 'value': False})
    assert pwm(cmd) != NEUTRAL                   # auto again only on command


def test_emergency_stop_resend_is_acked_once():
    run = Run()
    run.step({'cmd': 'emergency_stop', 'cq': 7})
    run.step({'cmd': 'emergency_stop', 'cq': 7})
    assert run.nav.status['last_cmd_ack'] == 7
    assert run.nav.emergency.active


def test_reset_without_emergency_changes_nothing():
    run = Run()
    run.step()
    run.nav.reset_emergency()
    assert run.nav.manual is False
