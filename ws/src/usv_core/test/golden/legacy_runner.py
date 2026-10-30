"""
Run the legacy nav_worker unchanged and record every control cycle.

The frozen copy (test/legacy/core/nav_process.py) runs with fake hardware and a virtual
clock.

What the legacy code thinks it is talking to, and what it really gets:
  hardware.MainSystem2.USVController   FakeController: PWMs go to a BoatModel, position,
                                       speed and heading come back from it (the legacy
                                       pymavlink module is never imported)
  time.time / time.sleep               VirtualClock: exactly 1 / NAV_LOOP_HZ per cycle; A*'s
                                       time budget never runs out, so planning is repeatable
  shared_state                         a plain dict (the Manager dict has the same get/[]/=)
  hf_data                              objects with a .value, like multiprocessing.Value
  command_queue / lidar_queue          queue.Queue; GCS commands are injected at their times

One record per cycle, taken in the cycle's closing time.sleep(): the task state, the five
PWM channels as the controller holds them, and the guidance values legacy publishes.
"""

import os
import queue
import signal
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
LEGACY = os.path.join(HERE, '..', 'legacy')
if LEGACY not in sys.path:
    sys.path.insert(0, LEGACY)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from scenario import BoatModel, mavlink_deg, Perception  # noqa: E402

_current = {}                # the harness of the run in progress (the fake controller finds it)


class FakeController:
    """Stand-in for hardware.MainSystem2.USVController, call for call."""

    def __init__(self, port='/dev/ttyACM0', baud=57600):
        self.h = _current['harness']
        self.pwms = {}               # legacy: last REQUESTED value per channel
        self.h.controller = self

    def set_mode(self, mode):
        pass

    def get_current_position(self):
        return self.h.gps

    def get_horizontal_speed(self):
        return abs(self.h.boat.s.surge)

    def get_heading(self):
        return self.h.heading

    def get_relay_state(self):
        return 1

    def set_relay(self, state, relay=None):
        pass

    def get_servo_pwm(self, channel):
        return self.pwms.get(channel, 1500)

    def set_servo(self, channel, pwm, force=False):
        self.pwms[channel] = int(pwm)

    def disarm_vehicle(self):
        pass


class Value:
    def __init__(self, v=0.0):
        self.value = v


class VirtualClock:
    """The legacy loop's time module: time() and sleep() on a clock we advance."""

    def __init__(self, harness):
        self.t = 1_000_000.0
        self.h = harness

    def time(self):
        return self.t

    def sleep(self, dt):
        self.h.end_of_cycle()
        self.t += self.h.period


def _import_legacy():
    if 'hardware.MainSystem2' not in sys.modules:
        hw = types.ModuleType('hardware')
        ms2 = types.ModuleType('hardware.MainSystem2')
        ms2.USVController = FakeController
        hw.MainSystem2 = ms2
        sys.modules['hardware'] = hw
        sys.modules['hardware.MainSystem2'] = ms2
    import config as cfg               # noqa: E402  (legacy config, frozen copy)
    from core import nav_process       # noqa: E402
    import utils.planner as planner    # noqa: E402
    return cfg, nav_process, planner


class LegacyHarness:

    def __init__(self, scenario):
        self.sc = scenario
        self.cfg, self.nav, self.planner = _import_legacy()
        self.period = 1.0 / max(1.0, getattr(self.cfg, 'NAV_LOOP_HZ', 25.0))
        origin = scenario.waypoints['T2_ZONE_ENTRY']
        self.boat = BoatModel(origin[0], origin[1], *scenario.start)
        self.perception = Perception(scenario.buoys)
        self.records = []
        self.controller = None
        self.cycle = 0
        self.finished_cycles = 0
        self._sense()

    # ----------------------------------------------------------------- inputs to legacy
    def _sense(self):
        lat, lon = self.boat.latlon()
        self.gps = (mavlink_deg(lat), mavlink_deg(lon))
        self.heading = (self.boat.s.heading + self.sc.heading_offset_deg) % 360.0

    def _channels(self):
        c = self.cfg
        return {'rear_left': c.SOL_MOTOR, 'rear_right': c.SAG_MOTOR,
                'front_left': c.FRONT_SOL_MOTOR, 'front_right': c.FRONT_SAG_MOTOR,
                'steer': c.FRONT_STEER_SERVO}

    def end_of_cycle(self):
        """Record the cycle, move the boat, refresh the inputs (the loop's closing sleep)."""
        s = self.shared
        pwm = {k: self.controller.pwms.get(ch, 1500) for k, ch in self._channels().items()}
        self.records.append({
            'cycle': self.cycle,
            't': round(self.clock.t - 1_000_000.0, 6),
            'task': s.get('current_task'),
            'pwm': (pwm['rear_left'], pwm['rear_right'], pwm['front_left'],
                    pwm['front_right'], pwm['steer']),
            'target': (s.get('target_lat'), s.get('target_lon')),
            'angle_error': s.get('angle_error'),
            'control_error': s.get('control_error'),
            'target_dist': s.get('target_dist'),
            'boat': (round(self.boat.s.east, 4), round(self.boat.s.north, 4),
                     round(self.boat.s.heading, 4)),
        })
        self.cycle += 1
        self.boat.step(pwm, self.period)
        self._sense()
        now = self.clock.t + self.period
        s['zed_heading'] = self.heading
        s['vision_detected_objects'] = self.perception.update(self.boat, now)
        for t_cmd, cmd in self.sc.commands:
            if abs((self.cycle * self.period) - t_cmd) < self.period / 2:
                self.cq.put(dict(cmd))
        if s.get('current_task') == 'FINISHED':
            self.finished_cycles += 1
        if self.cycle * self.period >= self.sc.duration_s or \
                (self.sc.stop_when_finished and self.finished_cycles >= 10):
            s['shutdown'] = True

    # ----------------------------------------------------------------- run
    def run(self):
        cfg, nav = self.cfg, self.nav
        saved = {k: getattr(cfg, k) for k in dir(cfg) if k.isupper()}
        old_sigterm = signal.getsignal(signal.SIGTERM)
        old_nav_time, old_planner_time = nav.time, self.planner.time
        try:
            for name, (lat, lon) in self.sc.waypoints.items():
                setattr(cfg, name + '_LAT', lat)
                setattr(cfg, name + '_LON', lon)
            cfg.RECORD_COSTMAP = False
            cfg.DRONE_ACTIVE = False
            cfg.TASK3_KAMIKAZE_COLOR = self.sc.target_color
            cfg.HEADING_SOURCE = 'ZED'

            self.clock = VirtualClock(self)
            nav.time = self.clock
            self.planner.time = self.clock
            _current['harness'] = self

            self.shared = {
                'shutdown': False,
                'current_task': self.sc.start_task,
                'manual_mode': self.sc.manual,
                'mission_started': self.sc.mission_started,
                'zed_heading': self.heading,
                'vision_detected_objects': self.perception.update(self.boat, self.clock.t),
            }
            self.hf = {'gps_lat': Value(), 'gps_lon': Value(), 'magnetic_heading': Value()}
            self.cq = queue.Queue()
            self.lq = queue.Queue()
            nav.nav_worker(self.shared, self.cq, self.hf, self.lq)
        finally:
            nav.time, self.planner.time = old_nav_time, old_planner_time
            for k in [k for k in dir(cfg) if k.isupper()]:
                if k not in saved:
                    delattr(cfg, k)
            for k, v in saved.items():
                setattr(cfg, k, v)
            signal.signal(signal.SIGTERM, old_sigterm)
            _current.clear()
        return self.records


def run_legacy(scenario):
    return LegacyHarness(scenario).run()
