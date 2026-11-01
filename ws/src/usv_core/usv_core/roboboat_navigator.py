"""
The RoboBoat rule book of the navigator (phase 6 R2).

Same driver as the TEKNOFEST navigator (usv_core.nav_base.NavigatorBase: GCS commands,
manual / auto, emergency latch, relay request, mixer); what differs is the mission:

  inputs   the GPS fix, heading and speed as before; instead of the legacy object memory
           (/usv/buoys) the world map (/usv/map, with its origin) and the latest camera
           frame (/usv/detections), handed in as a Perception
  course   usv_core.roboboat_course.CourseModel on the map every 0.5 s: Task 1 gates and
           the gate frame, channel, Task 3, marina, zones; new finds are logged
  mission  usv_core.roboboat_run.RoboBoatRun: entry gates, the chosen tasks, return, exit
  set_task the name of a run phase (T1_ENTRY, task2 ..., RETURN, T1_EXIT, FINISHED, or
           WAIT_START to start over) jumps there; as in TEKNOFEST it also releases the
           emergency latch while the relay is on
  set_gps  not used: RoboBoat tasks are found on the map (decision 2026-11-01)

step() keeps the TEKNOFEST order where it applies: commands first, then the emergency latch
(neutral, nothing else runs), then the mission; any exception neutralises the thrusters and
the loop carries on.
"""

from dataclasses import dataclass, field

from usv_core.nav_base import NavigatorBase
from usv_core.nav_config import NavConfig
from usv_core.roboboat_course import CourseModel
from usv_core.roboboat_run import RoboBoatRun, RunConfig, RunContext
from usv_core.world_map import enu

COURSE_PERIOD_S = 0.5


@dataclass
class Perception:
    landmarks: list = field(default_factory=list)     # /usv/map
    origin: tuple = None                              # /usv/map origin (lat, lon)
    detections: list = field(default_factory=list)    # latest /usv/detections frame
    detections_time: float = 0.0


def course_summary(view):
    """Return what the course model has found, as short labels (for the log)."""
    found = set()
    if view.t1_gates:
        found.add(f'Task 1 gates: {len(view.t1_gates)}')
    if view.channel:
        found.add(f'channel gates: {len(view.channel)}')
    if view.task3_gate is not None:
        found.add(f'Task 3 (indicator {view.task3_indicator or "?"})')
    if view.slips:
        found.add(f'marina slips: {len(view.slips)}')
    if view.best_slip is not None:
        found.add(f'best slip {view.best_slip.dock}{view.best_slip.number}')
    elif view.best_slip_so_far is not None:
        s = view.best_slip_so_far
        found.add(f'best slip so far {s.dock or "?"}{s.number} (marina not complete)')
    found.update(f'zone {name[len("task6_zone_"):]}' for name in view.zones)
    return found


class RoboBoatNavigator(NavigatorBase):

    def __init__(self, cfg=None, run_cfg=None, start_time=0.0, mission_started=True,
                 manual=False, log=None, phases=None):
        cfg = cfg or NavConfig()
        super().__init__(cfg, start_time, mission_started, manual, log)
        self.run = RoboBoatRun(run_cfg or RunConfig(), phases, self.log)
        self.course_model = CourseModel()
        self.course = None                  # latest CourseView
        self.course_found = set()
        self.last_course_time = None
        self.status = {'current_task': self.run.phase, 'angle_error': None,
                       'control_error': None, 'adviced_course': None, 'target_dist': None,
                       'target_lat': None, 'target_lon': None}

    @property
    def task(self):
        return self.run.phase

    def plan_path(self):
        return []

    # ------------------------------------------------------------------ profile hooks
    def _select_task(self, task):
        if self.run.jump(task, self.now):
            self.log(f'Run phase set to {task}')
        else:
            self.log(f'set_task {task!r}: no such phase in this run ({self.run.plan})')

    def set_waypoint(self, index, lat, lon):
        self.log(f'GPS point {index} ignored: RoboBoat tasks are found on the map')

    def _fresh_guidance(self):
        self.run.restart_phase()

    # ------------------------------------------------------------------ loop
    def step(self, now, gps, speed, heading, perception):
        """One control cycle. gps = (lat, lon), 0 = no fix; perception: a Perception."""
        try:
            self._step(now, gps, speed, heading, perception)
        except Exception as e:
            self.log(f'Iteration failed: {e}')
            self.mixer.neutral()
        self.status['current_task'] = self.run.phase
        self.status.update(self._status_common())
        return self.mixer.last

    def _step(self, now, gps, speed, heading, perception):
        self.now = now
        self.last_loop_time = now
        if speed is not None:
            self.speed_mps = float(speed)
        self.commands.drain(self._dispatch)
        if self.emergency.active:
            self.mixer.neutral()
            return
        perception = perception or Perception()
        boat_xy = None
        if perception.origin is not None and gps[0]:
            boat_xy = enu(gps[0], gps[1], *perception.origin)
        if self.last_course_time is None or now - self.last_course_time >= COURSE_PERIOD_S:
            self.last_course_time = now
            self._update_course(perception.landmarks,
                                self.run.memory.get('start_xy') or boat_xy)
        ctx = RunContext(now=now, lat=gps[0], lon=gps[1], heading=heading,
                         speed=self.speed_mps, landmarks=perception.landmarks,
                         detections=perception.detections,
                         detections_time=perception.detections_time,
                         boat_xy=boat_xy, map_origin=perception.origin,
                         course=self.course)
        command = self.run.step(ctx, self.mission_active)
        if command is None:
            self.mixer.mix(1500, 0)
        else:
            self.mixer.mix(*command)

    def _update_course(self, landmarks, start):
        self.course = self.course_model.view(landmarks, start)
        found = course_summary(self.course)
        new = sorted(found - self.course_found)
        if new:
            self.log('Course map: ' + ', '.join(new))
        self.course_found = found
