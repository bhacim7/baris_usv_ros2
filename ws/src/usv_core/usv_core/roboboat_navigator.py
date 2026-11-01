"""
The RoboBoat rule book of the navigator (phase 6 R2).

Same driver as the TEKNOFEST navigator (usv_core.nav_base.NavigatorBase: GCS commands,
manual / auto, emergency latch, relay request, mixer); what differs is the mission:

  inputs   the GPS fix, heading and speed as before; instead of the legacy object memory
           (/usv/buoys) the world map (/usv/map) and the latest camera frame
           (/usv/detections), handed in as a Perception
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
from usv_core.roboboat_run import RoboBoatRun, RunConfig, RunContext


@dataclass
class Perception:
    landmarks: list = field(default_factory=list)     # /usv/map
    detections: list = field(default_factory=list)    # latest /usv/detections frame
    detections_time: float = 0.0


class RoboBoatNavigator(NavigatorBase):

    def __init__(self, cfg=None, run_cfg=None, start_time=0.0, mission_started=True,
                 manual=False, log=None, phases=None):
        cfg = cfg or NavConfig()
        super().__init__(cfg, start_time, mission_started, manual, log)
        self.run = RoboBoatRun(run_cfg or RunConfig(), phases, self.log)
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
        ctx = RunContext(now=now, lat=gps[0], lon=gps[1], heading=heading,
                         speed=self.speed_mps, landmarks=perception.landmarks,
                         detections=perception.detections,
                         detections_time=perception.detections_time)
        command = self.run.step(ctx, self.mission_active)
        if command is None:
            self.mixer.mix(1500, 0)
        else:
            self.mixer.mix(*command)
