"""
The navigation loop, ported from legacy core/nav_process.py nav_worker (commit 5df3f5b).

Navigator.step() is one pass of legacy's while loop, minus everything that touched the
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

from usv_core import mission as mission_mod
from usv_core.commands import CommandFilter
from usv_core.costmap import LocalCostmap, LocalFrame
from usv_core.guidance import WaypointGuidance
from usv_core.mission import Mission
from usv_core.mixer import Mixer, MixerConfig
from usv_core.nav_config import NavConfig, WAYPOINT_NAMES
from usv_core.safety import EmergencyLatch

TASK3_SEARCH = mission_mod.TASK3_SEARCH


class Navigator:

    def __init__(self, cfg=None, start_time=0.0, mission_started=True, manual=False, log=None):
        self.cfg = cfg = cfg or NavConfig()
        self.log = log or (lambda msg: None)
        self.mixer = Mixer(MixerConfig(base_pwm=cfg.base_pwm, steer_max_pwm=cfg.steer_max_pwm,
                                       steer_min_pwm=cfg.steer_min_pwm,
                                       max_pwm_change=cfg.max_pwm_change))
        self.frame = LocalFrame()
        self.costmap = LocalCostmap(cfg.costmap_res_m_per_px, cfg.crop_radius_m,
                                    cfg.buoy_radius_m)
        self.guidance = WaypointGuidance(cfg, self.mixer, self.log)
        self.mission = Mission(cfg, self.mixer, self.log)
        self.commands = CommandFilter(cfg.vehicle_id)

        self.waypoints = {name: tuple(cfg.waypoints[name]) for name in WAYPOINT_NAMES}
        self.task = cfg.start_task
        self.manual = manual
        self.mission_started = mission_started
        self.drone_target_color = None
        self.emergency = EmergencyLatch()
        self.relay_on = None                # 1/0 read back from RELAY_STATUS, None = unknown
        self.relay_request = None           # 1/0 from set_relay, for whoever owns the relay
        self.send_telemetry = False

        self.now = start_time
        self.speed_mps = 0.0
        self.last_loop_time = start_time
        self.vision_period = 1.0 / max(1.0, cfg.vision_read_hz)
        self.last_vision_read = 0.0
        self.vision_objects = []
        self.mappable_objects = []
        self.target = None
        self.status = {'current_task': self.task, 'angle_error': None, 'control_error': None,
                       'adviced_course': None, 'target_dist': None, 'target_lat': None,
                       'target_lon': None}

    # ------------------------------------------------------------------ API
    @property
    def mission_active(self):
        return self.mission_started and not self.manual

    def submit(self, cmd):
        """Queue a GCS command (dict, legacy format); it is handled at the next step()."""
        self.commands.submit(cmd)

    def set_task(self, task):
        if not task:
            return
        self.task = task
        self.log(f'Task updated to {task}')
        if self.emergency.active:
            if self.relay_on == 1:
                self.reset_emergency()
            else:
                self.log('Emergency stop stays latched: turn the relay on, then set the task '
                         'again.')

    def set_manual(self, value):
        if value is True:
            self.manual = True
            self.log('Switched to MANUAL mode.')
        elif value is False:
            self.manual = False
            self.log('Switched to AUTO mode.')

    def set_waypoint(self, index, lat, lon):
        """GCS index 1..8 = GPS1..GPS8."""
        for i, name in enumerate(WAYPOINT_NAMES, 1):
            if index == i:
                self.waypoints[name] = (lat, lon)
        self.log(f'Updated GPS Point {index}')

    def set_target_color(self, color):
        if color:
            self.drone_target_color = color
            self.log(f'Drone target color updated to {color}')

    def reset_emergency(self):
        """
        Release the emergency latch (set_task does this once the relay is on).

        Guidance starts from fresh state - initial alignment, PID, path - as a restarted
        legacy program would have; task, waypoints and the manual/auto mode are kept.
        """
        if not self.emergency.active:
            return
        self.emergency.reset()
        self.guidance = WaypointGuidance(self.cfg, self.mixer, self.log)
        self.log('Emergency stop released.')

    def mission_points(self):
        return {f'GPS{i}': (float(self.waypoints[n][0]), float(self.waypoints[n][1]))
                for i, n in enumerate(WAYPOINT_NAMES, 1)}

    def _dispatch(self, cmd):
        cmd_str = cmd.get('cmd')
        if cmd_str == 'set_relay':
            self.relay_request = 1 if cmd.get('value') else 0
            self.log(f"Relay command received: {'ON' if self.relay_request else 'OFF'}")
        elif cmd_str == 'emergency_stop':
            self.log('Emergency Stop Received!')
            self.emergency.trip('GCS emergency_stop', self.now)
        elif cmd_str == 'report_status':
            self.send_telemetry = True
        elif cmd_str == 'set_gps':
            self.set_waypoint(cmd.get('index'), cmd.get('lat'), cmd.get('lon'))
        elif cmd_str == 'set_task':
            self.set_task(cmd.get('task_name'))
        elif cmd_str == 'set_manual':
            self.set_manual(cmd.get('value'))
        elif cmd_str == 'set_target_color':
            self.set_target_color(cmd.get('color'))

    # ------------------------------------------------------------------ loop
    def step(self, now, gps, speed, heading, vision_objects):
        """One control cycle. gps = (lat, lon), 0 = no fix; heading in compass degrees."""
        try:
            self._step(now, gps, speed, heading, vision_objects)
        except Exception as e:
            self.log(f'Iteration failed: {e}')
            self.mixer.neutral()
        self.status.update(manual_mode=self.manual, mission_started=self.mission_started,
                           last_cmd_ack=self.commands.last_ack,
                           drone_target_color=self.drone_target_color,
                           horizontal_speed=self.speed_mps,
                           emergency=self.emergency.active,
                           emergency_reason=self.emergency.reason)
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
