"""
What every navigator profile shares (phase 6 R2.1).

The TEKNOFEST navigator (usv_core.teknofest_navigator.TeknofestNavigator, ported cycle for
cycle from legacy) and the RoboBoat navigator
(usv_core.roboboat_navigator.RoboBoatNavigator) are two rule books for one driver.
Everything that is not a rule of the competition lives here, once:

  GCS commands     queued by the node, drained at the start of a cycle (CommandFilter)
  manual / auto    set_manual; mission_active = pre-flight passed and not manual
  emergency stop   latches (ARCHITECTURE.md section 9); released by set_task while the
                   relay reports on
  relay request    set_relay is only recorded; whoever owns the flight controller link acts
  mixer            the thruster mixer both profiles drive

The methods were moved here from TeknofestNavigator unchanged; the golden tests (legacy loop vs
TeknofestNavigator, cycle by cycle) guard that. What differs per profile is behind three hooks:
_select_task (what set_task means), set_waypoint (TEKNOFEST GPS points) and
_fresh_guidance (state rebuilt when the emergency latch is released).
"""

from usv_core.commands import CommandFilter
from usv_core.mixer import Mixer, MixerConfig
from usv_core.safety import EmergencyLatch


class NavigatorBase:

    def __init__(self, cfg, start_time=0.0, mission_started=True, manual=False, log=None):
        self.cfg = cfg
        self.log = log or (lambda msg: None)
        self.mixer = Mixer(MixerConfig(base_pwm=cfg.base_pwm, steer_max_pwm=cfg.steer_max_pwm,
                                       steer_min_pwm=cfg.steer_min_pwm,
                                       max_pwm_change=cfg.max_pwm_change))
        self.commands = CommandFilter(cfg.vehicle_id)
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
        self._select_task(task)
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

    def set_target_color(self, color):
        if color:
            self.drone_target_color = color
            self.log(f'Drone target color updated to {color}')

    def reset_emergency(self):
        """
        Release the emergency latch (set_task does this once the relay is on).

        Guidance starts from fresh state, as a restarted legacy program would have; task,
        waypoints and the manual/auto mode are kept.
        """
        if not self.emergency.active:
            return
        self.emergency.reset()
        self._fresh_guidance()
        self.log('Emergency stop released.')

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

    # ------------------------------------------------------------------ profile hooks
    def _select_task(self, task):
        raise NotImplementedError

    def set_waypoint(self, index, lat, lon):
        raise NotImplementedError

    def _fresh_guidance(self):
        raise NotImplementedError

    def _status_common(self):
        """Return the status fields every profile publishes."""
        return {'manual_mode': self.manual, 'mission_started': self.mission_started,
                'last_cmd_ack': self.commands.last_ack,
                'drone_target_color': self.drone_target_color,
                'horizontal_speed': self.speed_mps, 'emergency': self.emergency.active,
                'emergency_reason': self.emergency.reason}
