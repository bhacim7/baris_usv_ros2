"""
TEKNOFEST navigation loop, ported from legacy core/nav_process.py nav_worker (5df3f5b).

TeknofestNavigator.step() is one pass of legacy's while loop, minus everything that touched the
outside world:

  inputs    the caller passes the fix, ground speed, the selected heading and the object
            memory list; legacy read them from the flight controller and shared_state
  output    the ThrusterCommand of the cycle; legacy wrote the servos itself
  status    a dict with the values legacy published to shared_state (task, target, errors,
            mission points, last ack ...), for the ROS node to publish

Inside, the order of legacy's sections is kept, because the golden tests compare cycle by
cycle: commands (B), local frame (C), vision read at VISION_READ_HZ, costmap (D), mission
router (E), guidance values (F), control (G). Any exception neutralises the thrusters and
the loop carries on, as legacy did.

GCS commands, manual / auto, the emergency latch and the relay request are shared with the
RoboBoat navigator and live in usv_core.nav_base (moved there unchanged in phase 6 R2.1).

Two things differ from legacy on purpose (ARCHITECTURE.md section 9):
  emergency_stop   latches (usv_core.safety.EmergencyLatch): neutral from the cycle the
                   command arrives; commands are still taken in, nothing else runs. Legacy
                   ended the program instead. fc_bridge / sim_bridge also switch the relay
                   off. The latch is released only by a set_task received while the relay
                   reports on (relay_on, from RELAY_STATUS): relay on alone never makes the
                   boat move, and a task chosen before the relay is on does not either.
  set_relay        is only recorded in relay_request; the retries live with whoever owns the
                   flight controller link (usv_core.relay.RelayRetry, used by fc_bridge).
"""

from usv_core import teknofest_mission as mission_mod
from usv_core.nav_base import NavigatorBase
from usv_core.nav_config import NavConfig, WAYPOINT_NAMES
from usv_core.teknofest_costmap import LocalFrame, TeknofestCostmap
from usv_core.teknofest_guidance import WaypointGuidance
from usv_core.teknofest_mission import TeknofestMission

TASK3_SEARCH = mission_mod.TASK3_SEARCH


class TeknofestNavigator(NavigatorBase):
    """The TEKNOFEST rule book; commands, modes and the emergency latch: NavigatorBase."""

    def __init__(self, cfg=None, start_time=0.0, mission_started=True, manual=False, log=None):
        cfg = cfg or NavConfig()
        super().__init__(cfg, start_time, mission_started, manual, log)
        self.frame = LocalFrame()
        self.costmap = TeknofestCostmap(cfg.costmap_res_m_per_px, cfg.crop_radius_m,
                                        cfg.buoy_radius_m)
        self.guidance = WaypointGuidance(cfg, self.mixer, self.log)
        self.mission = TeknofestMission(cfg, self.mixer, self.log)

        self.waypoints = {name: tuple(cfg.waypoints[name]) for name in WAYPOINT_NAMES}
        self.task = cfg.start_task
        self.vision_period = 1.0 / max(1.0, cfg.vision_read_hz)
        self.last_vision_read = 0.0
        self.vision_objects = []
        self.mappable_objects = []
        self.target = None
        self.status = {'current_task': self.task, 'angle_error': None, 'control_error': None,
                       'adviced_course': None, 'target_dist': None, 'target_lat': None,
                       'target_lon': None}

    # ------------------------------------------------------------------ profile hooks
    def _select_task(self, task):
        self.task = task
        self.log(f'Task updated to {task}')

    def set_waypoint(self, index, lat, lon):
        """GCS index 1..8 = GPS1..GPS8."""
        for i, name in enumerate(WAYPOINT_NAMES, 1):
            if index == i:
                self.waypoints[name] = (lat, lon)
        self.log(f'Updated GPS Point {index}')

    def _fresh_guidance(self):
        """Start alignment, PID and path over, as a restarted legacy program would."""
        self.guidance = WaypointGuidance(self.cfg, self.mixer, self.log)

    def plan_path(self):
        """Return the A* path being followed (local x = North, y = East), for /usv/plan."""
        return self.guidance.path

    def mission_points(self):
        return {f'GPS{i}': (float(self.waypoints[n][0]), float(self.waypoints[n][1]))
                for i, n in enumerate(WAYPOINT_NAMES, 1)}

    # ------------------------------------------------------------------ loop
    def step(self, now, gps, speed, heading, vision_objects):
        """One control cycle. gps = (lat, lon), 0 = no fix; heading in compass degrees."""
        try:
            self._step(now, gps, speed, heading, vision_objects)
        except Exception as e:
            self.log(f'Iteration failed: {e}')
            self.mixer.neutral()
        self.status.update(self._status_common())
        return self.mixer.last

    def _step(self, now, gps, speed, heading, vision_objects):
        self.now = now
        loop_dt = min(0.5, max(0.001, now - self.last_loop_time))
        self.last_loop_time = now

        # A. sensors
        lat, lon = gps
        if speed is not None:
            self.speed_mps = float(speed)

        # B. commands
        self.commands.drain(self._dispatch)

        if self.emergency.active:
            self.mixer.neutral()
            return

        # C. local frame
        task = self.task
        frame = self.frame
        frame.update(lat, lon)

        if (now - self.last_vision_read) >= self.vision_period:
            self.last_vision_read = now
            self.vision_objects = vision_objects
            self.mappable_objects = [o for o in vision_objects if o.get('pos_ok', True)]

        # D. costmap
        self.costmap.rebuild(frame.x, frame.y, self.mappable_objects, task, lat, lon)

        # E. mission router
        mission_active = self.mission_active
        task, target, skip_default_nav = self.mission.route(
            task, lat, lon, heading, self.waypoints, mission_active, self.vision_objects,
            self.drone_target_color, now)
        self.task = task
        self.target = target
        self.status['current_task'] = task

        # F. guidance values
        published = self.guidance.observe(target, task, lat, lon, heading, frame.x, frame.y,
                                          now)
        if published is not None:
            self.status.update(published)

        # G. control
        if self.manual or not self.mission_started:
            self.mixer.mix(1500, 0)
        elif not skip_default_nav and target is not None:
            self.guidance.drive(target, task, frame.x, frame.y, heading, self.speed_mps,
                                loop_dt, now, self.costmap, frame, self.vision_objects,
                                lat, lon, self.waypoints)
        elif not skip_default_nav:
            self.mixer.mix(1500, 0)
