"""
The RoboBoat run: which phase the boat is in and what it asks of the thrusters (phase 6 R2).

A run (handbook 2026, section 3.5) is: through the Task 1 gates (entry), the tasks of the
team's choice in any order, back through the same gates the other way (exit). RoboBoatRun
sequences that as phases:

    WAIT_START -> T1_ENTRY -> <task_order ...> -> RETURN -> T1_EXIT -> FINISHED

  WAIT_START   until the mission is active (pre-flight passed and AUTO); then run_started
  a phase      runs until it reports DONE or SKIPPED, or until task_timeout_s (SKIPPED,
               'timeout'); then the next one starts
  run timeout  after run_timeout_s the boat gives up what it is doing and goes RETURN, so
               it still exits through the gates (worth 300 points with the report)
  jump         set_task(<phase>) from the GCS / launch starts that phase (tests, practice)
  MANUAL       the run pauses: neutral, the phase is kept; AUTO again carries on

Every phase change is written to `events` (time, kind, details): what the RoboCommand
reports (R4) are built from (entry gate time and position, ...).

A phase is an object with start(ctx) and step(ctx) -> (state, command); command is a
(forward_pwm, yaw_pwm) pair for usv_core.mixer.Mixer.mix, or None for neutral. The phases
themselves come in R2.4 (Task 1) and R3 (Tasks 2-6); until then a phase is Pending: it is
skipped at once with the reason 'not written yet'.

`ctx` (RunContext) carries the cycle's inputs: time, GPS fix (also as map metres), heading,
speed, the world map (/usv/map landmarks), what the course model makes of it (course), the
latest camera frame (/usv/detections), plus `memory`, a dict the phases share across the
run: start_xy (where the run started, map metres), and what phases leave for later ones.
"""

from dataclasses import dataclass, field

RUNNING, DONE, SKIPPED = 'RUNNING', 'DONE', 'SKIPPED'
WAIT_START, T1_ENTRY, RETURN, T1_EXIT, FINISHED = (
    'WAIT_START', 'T1_ENTRY', 'RETURN', 'T1_EXIT', 'FINISHED')
TASKS = ('task2', 'task3', 'task4', 'task5')      # chosen per run; Task 6 interrupts (R3)
HINTED = TASKS + ('task6_zone_t3', 'task6_zone_marina')


@dataclass(frozen=True)
class RunConfig:
    task_order: tuple = ('task2', 'task3', 'task5')
    task_timeout_s: float = 240.0
    run_timeout_s: float = 900.0          # then RETURN and exit; 0 = never
    # Where to start looking for a task, from the Task 1 entry gates: (ahead_m, left_m),
    # ahead = from the first gate's centre towards the second's. Seen by the team from the
    # shore before the run; no hint = explore (R3).
    hints: dict = field(default_factory=dict)


@dataclass
class RunContext:
    now: float
    lat: float = 0.0
    lon: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    landmarks: list = field(default_factory=list)     # /usv/map, dicts
    detections: list = field(default_factory=list)    # latest /usv/detections frame, dicts
    detections_time: float = 0.0
    memory: dict = field(default_factory=dict)
    boat_xy: tuple = None           # the fix in the map's east / north metres (None: no map)
    map_origin: tuple = None        # the map's origin (lat, lon), for the camera detections
    course: object = None           # usv_core.roboboat_course.CourseView, updated at 2 Hz


class Pending:
    """A phase not written yet: skipped at once."""

    def __init__(self, name):
        self.name = name

    def start(self, ctx):
        pass

    def step(self, ctx):
        return SKIPPED, None


class RoboBoatRun:

    def __init__(self, cfg=RunConfig(), phases=None, log=None):
        self.cfg = cfg
        self.log = log or (lambda msg: None)
        unknown = [t for t in cfg.task_order if t not in TASKS]
        if unknown:
            raise ValueError(f'unknown tasks {unknown}; RoboBoat tasks are {TASKS}')
        self.plan = [T1_ENTRY, *cfg.task_order, RETURN, T1_EXIT]
        self.phases = {name: Pending(name) for name in self.plan}
        self.phases.update(phases or {})
        self.phase = WAIT_START
        self.phase_started = None
        self.run_started = None
        self.events = []
        self.memory = {}
        self._start_pending = False       # call the phase's start() at the next step

    # ------------------------------------------------------------------ control
    def jump(self, name, now=None):
        """Start phase `name` now (from set_task); False if the run has no such phase."""
        if name not in self.plan and name not in (WAIT_START, FINISHED):
            return False
        self._event(now, 'jump', phase=name)
        self._enter(name, now)
        return True

    def restart_phase(self):
        """Start the current phase over at the next step (after an emergency stop)."""
        if self.phase not in (WAIT_START, FINISHED):
            self._start_pending = True

    # ------------------------------------------------------------------ cycle
    def step(self, ctx, active):
        """One control cycle; returns (forward_pwm, yaw_pwm) or None (= neutral)."""
        ctx.memory = self.memory
        if not active or self.phase == FINISHED:
            return None
        now = ctx.now
        if self.phase == WAIT_START:
            self.run_started = now
            self.memory['start_xy'] = ctx.boat_xy
            self._event(now, 'run_started', start_xy=ctx.boat_xy)
            self._enter(self.plan[0], now, ctx)
        if self._start_pending:
            self._start_pending = False
            self.phase_started = now
            self.phases[self.phase].start(ctx)

        if (self.cfg.run_timeout_s > 0 and now - self.run_started > self.cfg.run_timeout_s
                and self.phase not in (RETURN, T1_EXIT)):
            self._event(now, 'run_timeout', phase=self.phase)
            self.log(f'Run time is up ({self.cfg.run_timeout_s:.0f} s): returning to the gates')
            self._enter(RETURN, now, ctx)

        state, command = self.phases[self.phase].step(ctx)
        if state == RUNNING and now - self.phase_started > self.cfg.task_timeout_s \
                and self.phase not in (RETURN, T1_EXIT):
            state = SKIPPED
            self._event(now, 'phase_timeout', phase=self.phase)
        if state == RUNNING:
            return command
        self._event(now, 'phase_done' if state == DONE else 'phase_skipped', phase=self.phase)
        self.log(f'{self.phase}: {state.lower()}')
        self._enter(self._next(self.phase), now, ctx)
        return None

    def _next(self, phase):
        i = self.plan.index(phase)
        return self.plan[i + 1] if i + 1 < len(self.plan) else FINISHED

    def _enter(self, name, now, ctx=None):
        self.phase = name
        self.phase_started = now
        if name == FINISHED:
            self._event(now, 'run_finished')
            self.log('Run FINISHED')
            return
        if name == WAIT_START:
            self.run_started = None
            return
        if self.run_started is None:
            self.run_started = now
        self._event(now, 'phase_started', phase=name)
        self.log(f'Phase {name}')
        if ctx is not None:
            self.phases[name].start(ctx)
        else:
            self._start_pending = True    # start() once the next cycle brings a context

    def _event(self, now, kind, **details):
        self.events.append((now, kind, details))
