"""
The mission manager: what the boat does next, in the RoboBoat 2027 competition (step 6a).

The 2027 run is no longer a fixed line (2026: entry gates, tasks, exit gates; that run is
kept, frozen, for the Proof of Readiness video: usv_core.roboboat_run). Tasks come from a
plan, and from outside during the run (RoboCommand through the GCS, usv_interfaces/
TaskRequest), some of them urgent (2027 Task 4 delivery, Task 5 rally):

    WAIT_START -> plan tasks, requests in between ... -> HOME

  plan        the tasks of the mission, in order (course file: mission roboboat_2027)
  request     a task asked during the run; urgent: the current task is put aside (pushed
              on a stack) and the new one starts now; not urgent: it runs when the current
              task ends, before the rest of the plan
  resume      a task put aside starts again from its beginning (start() once more) with
              the run's shared memory, so it can skip what it had already done (decided
              with Baris, 2026-11-11: resuming mid-step would be written, and go wrong,
              task by task)
  refused     a request for a task this boat does not know, or cannot do (its profile:
              usv_core.boat, e.g. water on Azure), is written to the events, not run
  timeouts    a task running longer than its timeout is SKIPPED; after run_timeout_s
              everything left is dropped and the boat goes HOME
  HOME        when nothing is left (or time is up): the home task (back to where the run
              started and hold there, decided 2026-11-11; the drive comes with step 6c)
  MANUAL      the run pauses (neutral), as before; AUTO carries on

A task is an object with start(ctx) and step(ctx) -> (state, command), as the 2026 phases
(usv_core.roboboat_run: RUNNING / DONE / SKIPPED, command (forward, yaw) or None); it may
have `needs` (a capability, usv_core.boat.CAPABILITIES) and summary(ctx). Tasks are made by
factories, factory(params) -> task, so a request's parameters (a rally point, a shape)
reach its task. Every change goes to `events` (time, kind, details): the RoboCommand
reports are built from them (step 6d). ROS-free, unit tested (test/test_mission.py).
"""

from collections import deque
from dataclasses import dataclass, field

from usv_core.roboboat_run import DONE, RUNNING, SKIPPED

WAIT_START, HOME = 'WAIT_START', 'HOME'


@dataclass(frozen=True)
class MissionConfig:
    plan: tuple = ()
    task_timeout_s: float = 360.0
    timeouts: dict = field(default_factory=dict)      # task name -> its own timeout (s)
    run_timeout_s: float = 1200.0                     # then HOME; 0 = never


@dataclass
class Request:
    task: str
    urgent: bool = False
    params: dict = field(default_factory=dict)
    source: str = ''                  # 'robocommand', 'gcs', 'plan'
    request_id: int = 0


class Pending:
    """A task whose behaviour is not written yet: skipped at once (says so in the log)."""

    summary_text = 'not written yet'

    def __init__(self, params=None):
        self.params = params or {}

    def start(self, ctx):
        pass

    def step(self, ctx):
        return SKIPPED, None

    def summary(self, ctx):
        return self.summary_text


class Idle:
    """Neutral thrusters for ever: the home task until the real one (step 6c)."""

    def __init__(self, params=None):
        pass

    def start(self, ctx):
        pass

    def step(self, ctx):
        return RUNNING, None


class MissionManager:

    def __init__(self, cfg, factories, capabilities=(), home=Idle, log=None):
        self.cfg = cfg
        self.factories = dict(factories)  # task name -> factory(params) -> task
        self.capabilities = set(capabilities)
        self.home_factory = home
        self.log = log or (lambda msg: None)
        unknown = [t for t in cfg.plan if t not in self.factories]
        if unknown:
            raise ValueError(f'plan tasks without a factory: {unknown}')
        self.plan = deque(Request(t, source='plan') for t in cfg.plan)
        self.queue = deque()              # requests waiting, before the rest of the plan
        self.stack = []                   # (request, task) put aside by urgent requests
        self.phase = WAIT_START
        self.current = None               # (request, task)
        self.phase_started = None
        self.run_started = None
        self.memory = {}
        self.events = []
        self.reports = []                 # (task, report dict) since the navigator last took
        self._start_pending = False

    # ------------------------------------------------------------------ outside
    def request(self, req, now=None):
        """Take a task asked during the run; returns False (and logs why) if refused."""
        why = self._refusal(req.task)
        if why:
            self._event(now, 'request_refused', task=req.task, source=req.source,
                        request_id=req.request_id, reason=why)
            self.log(f'Request {req.task} from {req.source or "?"} refused: {why}')
            return False
        self._event(now, 'request', task=req.task, urgent=req.urgent, source=req.source,
                    request_id=req.request_id, params=dict(req.params))
        if self.phase == WAIT_START:
            # Not started yet: urgent or not, it is the first thing to do
            (self.queue.appendleft if req.urgent else self.queue.append)(req)
            return True
        if req.urgent:
            if self.current is not None and self.phase != HOME:
                self.stack.append(self.current)
                self._event(now, 'task_suspended', task=self.phase)
                self.log(f'{self.phase} put aside for urgent {req.task}')
            self._begin(req, now)
        else:
            self.queue.append(req)
            if self.phase == HOME:        # nothing was left: do it now
                self._advance(now)
        return True

    def _refusal(self, name):
        if name not in self.factories:
            return 'unknown task'
        needs = getattr(self.factories[name], 'needs', None)
        if needs and needs not in self.capabilities:
            return f'this boat cannot do {needs}'
        return ''

    def jump(self, name, now=None):
        """Start task `name` now, dropping the current one (set_task from the GCS, tests)."""
        if name == WAIT_START:
            self.__init__(self.cfg, self.factories, self.capabilities, self.home_factory,
                          self.log)
            return True
        if name == HOME:
            self._go_home(now, 'jump')
            return True
        if name not in self.factories:
            return False
        self._event(now, 'jump', task=name)
        self._begin(Request(name, source='jump'), now)
        return True

    def restart_task(self):
        """Start the current task over at the next step (after an emergency stop)."""
        if self.current is not None:
            self._start_pending = True

    restart_phase = restart_task      # the name RoboBoatNavigator calls (RoboBoatRun's)

    def upcoming(self):
        """Return the tasks still to come, in the order they will run."""
        return ([r.task for r, _ in reversed(self.stack)] + [r.task for r in self.queue]
                + [r.task for r in self.plan])

    def status(self):
        """Return what the GCS shows: the task now, put aside, waiting, left in the plan."""
        return {'task': self.phase, 'suspended': [r.task for r, _ in self.stack],
                'queue': [r.task for r in self.queue], 'plan': [r.task for r in self.plan]}

    # ------------------------------------------------------------------ cycle
    def step(self, ctx, active):
        """One control cycle; returns (forward_pwm, yaw_pwm) or None (= neutral)."""
        ctx.memory = self.memory
        if not active:
            return None
        now = ctx.now
        if self.phase == WAIT_START:
            self.run_started = now
            self.memory['start_xy'] = ctx.boat_xy
            self._event(now, 'run_started', start_xy=ctx.boat_xy)
            self._advance(now, ctx)
        if self.memory.get('start_xy') is None and ctx.boat_xy is not None:
            self.memory['start_xy'] = ctx.boat_xy
        if self._start_pending:
            self._start_pending = False
            self.phase_started = now
            self.current[1].start(ctx)
        if (self.cfg.run_timeout_s > 0 and now - self.run_started > self.cfg.run_timeout_s
                and self.phase != HOME):
            self._event(now, 'run_timeout', task=self.phase)
            self.log(f'Run time is up ({self.cfg.run_timeout_s:.0f} s): going home')
            self._go_home(now, 'run_timeout', ctx)
        req, task = self.current
        state, command = task.step(ctx)
        for report in getattr(task, 'take_reports', lambda: [])():
            self.reports.append((self.phase, report))
        if state == RUNNING and self.phase != HOME:
            limit = self.cfg.timeouts.get(self.phase, self.cfg.task_timeout_s)
            if now - self.phase_started > limit:
                state = SKIPPED
                self._event(now, 'task_timeout', task=self.phase)
        if state == RUNNING:
            return command
        self._event(now, 'task_done' if state == DONE else 'task_skipped', task=self.phase,
                    request_id=req.request_id, source=req.source)
        summary = getattr(task, 'summary', None)
        self.log(f'{self.phase}: {state.lower()}' + (f' ({summary(ctx)})' if summary else ''))
        self._advance(now, ctx)
        return None

    def _advance(self, now, ctx=None):
        """Start what comes next: a task put aside, a request, the plan, else HOME."""
        if self.stack:
            req, task = self.stack.pop()
            self._event(now, 'task_resumed', task=req.task)
            self.log(f'Resuming {req.task}')
            self._enter(req, task, now, ctx)
        elif self.queue:
            self._begin(self.queue.popleft(), now, ctx)
        elif self.plan:
            self._begin(self.plan.popleft(), now, ctx)
        else:
            self._go_home(now, 'done', ctx)

    def _begin(self, req, now, ctx=None):
        self._enter(req, self.factories[req.task](dict(req.params)), now, ctx)

    def _go_home(self, now, why, ctx=None):
        self.plan.clear()
        self.queue.clear()
        self.stack.clear()
        self._event(now, 'going_home', reason=why)
        self._enter(Request(HOME, source=why), self.home_factory({}), now, ctx)

    def _enter(self, req, task, now, ctx=None):
        self.current = (req, task)
        self.phase = req.task
        self.phase_started = now
        if self.run_started is None:
            self.run_started = now
        self._event(now, 'task_started', task=req.task, source=req.source,
                    request_id=req.request_id)
        self.log(f'Task {req.task}' + (f' ({req.source})' if req.source not in ('', 'plan')
                                       else ''))
        if ctx is not None:
            task.start(ctx)
        else:
            self._start_pending = True    # start() once the next cycle brings a context

    def _event(self, now, kind, **details):
        self.events.append((now, kind, details))


def needs(capability, factory):
    """Return the factory marked as needing a capability (usv_core.boat.CAPABILITIES)."""
    def make(params):
        return factory(params)
    make.needs = capability
    return make


def competition_factories(written=None):
    """
    Return the RoboBoat 2027 task factories: the behaviours written so far, else Pending.

    task1 Safe Passage, marina (Task 3), task5 Dynamic Response, task2 Emergency Response
    Sprint, task4 Delivery Run (needs the water cannon); rally (Task 5 Core: go to a point
    and hold) and hold (usv_core.mission_tasks.factories gives them). written: {name:
    factory} replacing a Pending.
    """
    out = {'task1': Pending, 'marina': Pending, 'task5': Pending, 'task2': Pending,
           'task4': needs('water', Pending), 'rally': Pending, 'hold': Pending}
    out.update(written or {})
    return out
