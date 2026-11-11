"""
usv_core.mission: the RoboBoat 2027 mission manager (step 6a).

Run: cd /ws/src/usv_core && python3 -m pytest test/test_mission.py -v
"""

from usv_core.mission import HOME, Idle, MissionConfig, MissionManager, Pending, Request
from usv_core.roboboat_run import DONE, RunContext, RUNNING


class Steps:
    """A task that runs for `n` steps, then is DONE; counts its starts."""

    def __init__(self, n, record):
        self.n, self.record, self.left = n, record, n

    def start(self, ctx):
        self.left = self.n
        self.record.append(('start', ctx.now))

    def step(self, ctx):
        self.left -= 1
        return (DONE, None) if self.left <= 0 else (RUNNING, (1600, 1500))


def factory(n, record, needs=None):
    def make(params):
        record.append(('made', dict(params)))
        return Steps(n, record)
    make.needs = needs
    return make


def run(m, t0, t1, dt=1.0, active=True):
    t, out = t0, []
    while t <= t1:
        cmd = m.step(RunContext(now=t, boat_xy=(0.0, 0.0)), active)
        out.append((t, m.phase, cmd))            # the task in charge after this step
        t += dt
    return out


def phases(trace):
    out = []
    for _, phase, _ in trace:
        if not out or out[-1] != phase:
            out.append(phase)
    return out


def test_plan_runs_in_order_then_home():
    rec = []
    tasks = {'a': factory(2, rec), 'b': factory(3, rec)}
    m = MissionManager(MissionConfig(plan=('a', 'b')), tasks)
    trace = run(m, 0, 8)
    assert phases(trace) == ['a', 'b', HOME]
    assert m.status() == {'task': HOME, 'suspended': [], 'queue': [], 'plan': []}
    kinds = [k for _, k, _ in m.events]
    assert kinds[:3] == ['run_started', 'task_started', 'task_done']
    assert 'going_home' in kinds


def test_urgent_request_puts_the_task_aside_and_it_starts_again_after():
    rec = []
    m = MissionManager(MissionConfig(plan=('long', 'next')),
                       {'long': factory(10, rec), 'next': factory(1, rec),
                        'rally': factory(2, rec)})
    run(m, 0, 2)                                           # long has started
    assert m.request(Request('rally', urgent=True, params={'lat': 1.0}, source='robocommand'),
                     now=2.5)
    trace = run(m, 3, 20)
    assert phases(trace) == ['rally', 'long', 'next', HOME]
    assert ('made', {'lat': 1.0}) in rec                  # the parameters reach the task
    starts = [t for kind, t in rec if kind == 'start']
    assert len(starts) == 4                                # long twice: begun, resumed
    kinds = [k for _, k, _ in m.events]
    assert 'task_suspended' in kinds and 'task_resumed' in kinds


def test_request_that_is_not_urgent_waits_for_the_current_task():
    rec = []
    m = MissionManager(MissionConfig(plan=('a', 'b')), {
        'a': factory(3, rec), 'b': factory(1, rec), 'x': factory(1, rec)})
    run(m, 0, 0)
    m.request(Request('x'), now=0.5)
    assert m.status()['queue'] == ['x']
    assert phases(run(m, 1, 8)) == ['a', 'x', 'b', HOME]


def test_requests_this_boat_cannot_do_are_refused():
    rec = []
    m = MissionManager(MissionConfig(plan=('a',)), {
        'a': factory(5, rec), 'deliver': factory(1, rec, needs='water')},
        capabilities=('balls',))
    run(m, 0, 0)
    assert not m.request(Request('deliver', urgent=True), now=0.5)
    assert not m.request(Request('fly'), now=0.6)
    reasons = [d['reason'] for _, k, d in m.events if k == 'request_refused']
    assert reasons == ['this boat cannot do water', 'unknown task']
    assert m.phase == 'a'
    marine = MissionManager(MissionConfig(), {'deliver': factory(1, rec, needs='water')},
                            capabilities=('water',))
    assert marine.request(Request('deliver'))


def test_timeouts():
    rec = []
    m = MissionManager(MissionConfig(plan=('slow', 'b'), task_timeout_s=3.0,
                                     timeouts={'b': 100.0}, run_timeout_s=10.0),
                       {'slow': factory(99, rec), 'b': factory(99, rec)})
    trace = run(m, 0, 12)
    # slow skipped after 3 s, b would go on but the run is up at 10 s
    assert phases(trace) == ['slow', 'b', HOME]
    kinds = [k for _, k, _ in m.events]
    assert 'task_timeout' in kinds and 'run_timeout' in kinds


def test_home_then_a_request_still_runs():
    rec = []
    m = MissionManager(MissionConfig(), {'rally': factory(2, rec)})
    run(m, 0, 1)
    assert m.phase == HOME
    m.request(Request('rally'), now=1.5)
    assert phases(run(m, 2, 6)) == ['rally', HOME]


def test_request_before_the_start_is_the_first_thing_done():
    rec = []
    m = MissionManager(MissionConfig(plan=('a',)), {'a': factory(2, rec),
                                                    'rally': factory(2, rec)})
    m.request(Request('rally', urgent=True), now=0.0)
    assert phases(run(m, 1, 4)) == ['rally', 'a', HOME]


def test_manual_pauses_and_pending_tasks_are_skipped():
    m = MissionManager(MissionConfig(plan=('task1', 'marina')),
                       {'task1': Pending, 'marina': Pending}, home=Idle)
    assert run(m, 0, 2, active=False)[-1][2] is None and m.phase == 'WAIT_START'
    run(m, 3, 5)
    assert m.phase == HOME
    skipped = [d['task'] for _, k, d in m.events if k == 'task_skipped']
    assert skipped == ['task1', 'marina']


def test_jump_and_unknown_plan_tasks():
    rec = []
    tasks = {'a': factory(9, rec), 'b': factory(1, rec)}
    m = MissionManager(MissionConfig(plan=('a', 'b')), tasks)
    run(m, 0, 1)
    assert m.jump('b', now=1.5) and not m.jump('nope')
    assert m.phase == 'b'
    run(m, 2, 2)
    assert m.phase == 'b' or m.phase == HOME   # b (1 step) done: the plan had only b left
    assert [d['task'] for _, k, d in m.events if k == 'jump'] == ['b']
    try:
        MissionManager(MissionConfig(plan=('ghost',)), {})
        assert False, 'a plan task without a factory must be refused'
    except ValueError:
        pass
