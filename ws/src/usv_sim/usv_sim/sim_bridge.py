"""sim_bridge: the simulated boat's flight-controller side. Stands in for fc_bridge.

Command side (this file so far):
  /usv/thrusters/pwm   usv_interfaces/ThrusterPwm, from navigator (or by hand)
      -> /usv/sim/thrust/<thruster>   std_msgs/Float64, N
      -> /usv/sim/steer/<thruster>    std_msgs/Float64, rad (both front thrusters)
      ros_gz_bridge (config/ros_gz_bridge.yaml) forwards these to the usv_boat Gazebo topics.
  /usv/set_relay       std_srvs/SetBool: motor power relay. Off = no thrust; the steering
                       servos keep working (the relay cuts thruster power only).
  /usv/emergency_stop  std_srvs/Trigger: outputs to 1500 at once and relay off. Leaving the
                       emergency: relay on, then a new command (see ARCHITECTURE.md, safety).
  /usv/relay_state     std_msgs/Int8 at 2 Hz (the legacy code asks the FC for RELAY_STATUS
                       at 2 Hz): 0 off, 1 on.

What happens to a command, as on the boat (models in actuators.py, values in
config/sim_bridge.yaml):
  1. write thinning of the legacy USVController.set_servo; the FC holds the last value
  2. no ThrusterPwm for command_timeout_s -> all outputs to 1500 (fc_bridge design, 0.5 s)
  3. thrust from the PWM -> N table, 0 N while the relay is off
  4. steering angle follows the command at the MG995 speed
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, Int8
from std_srvs.srv import SetBool, Trigger

from usv_interfaces.msg import ThrusterPwm
from usv_sim.actuators import PwmToThrust, ServoWriteThinning, SlewLimiter, steer_angle_rad

THRUSTERS = ('rear_left', 'rear_right', 'front_left', 'front_right')
STEERED = ('front_left', 'front_right')
NEUTRAL = 1500


class SimBridge(Node):

    def __init__(self):
        super().__init__('sim_bridge')
        p = self.declare_parameter
        self.rate_hz = p('output_rate_hz', 50.0).value
        self.timeout_s = p('command_timeout_s', 0.5).value
        self.relay_on = p('relay_on_at_start', True).value
        min_delta = p('min_pwm_delta_us', 3).value
        refresh = p('servo_refresh_s', 0.5).value
        self.to_thrust = PwmToThrust(
            p('thrust_table_pwm', [1100.0, 1500.0, 1900.0]).value,
            p('thrust_table_newton', [-64.0, 0.0, 91.0]).value)
        self.steer_args = dict(
            center_us=p('steer_center_us', 1500.0).value,
            half_range_us=p('steer_half_range_us', 400.0).value,
            max_angle_rad=p('steer_max_angle_rad', 0.785).value,
            sign=p('steer_sign', -1.0).value)
        self.steer_servo = SlewLimiter(p('steer_rate_rad_s', 5.2).value)

        # One FC output channel per ThrusterPwm field
        self.channels = {name: ServoWriteThinning(min_delta, refresh)
                         for name in THRUSTERS + ('steer',)}
        self.last_command = None        # time.monotonic() of the last ThrusterPwm
        self.timed_out = False

        self.thrust_pub = {n: self.create_publisher(Float64, f'/usv/sim/thrust/{n}', 10)
                           for n in THRUSTERS}
        self.steer_pub = {n: self.create_publisher(Float64, f'/usv/sim/steer/{n}', 10)
                          for n in STEERED}
        self.relay_pub = self.create_publisher(Int8, '/usv/relay_state', 10)

        self.create_subscription(ThrusterPwm, '/usv/thrusters/pwm', self.on_pwm, 10)
        self.create_service(SetBool, '/usv/set_relay', self.on_set_relay)
        self.create_service(Trigger, '/usv/emergency_stop', self.on_emergency_stop)

        self.last_tick = time.monotonic()
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.create_timer(0.5, self.publish_relay)
        self.get_logger().info(
            f'sim_bridge ready: {self.rate_hz:.0f} Hz outputs, timeout {self.timeout_s} s, '
            f'relay {"on" if self.relay_on else "off"}')

    # ------------------------------------------------------------------ inputs
    def on_pwm(self, msg):
        now = time.monotonic()
        self.last_command = now
        if self.timed_out:
            self.timed_out = False
            self.get_logger().info('ThrusterPwm back, outputs follow commands again')
        for name in THRUSTERS + ('steer',):
            self.channels[name].write(getattr(msg, name), now, force=msg.force)

    def neutralise(self, now):
        for ch in self.channels.values():
            ch.write(NEUTRAL, now, force=True)

    def on_set_relay(self, request, response):
        self.relay_on = bool(request.data)
        self.publish_relay()
        response.success = True
        response.message = f'relay {"on" if self.relay_on else "off"}'
        self.get_logger().info(response.message)
        return response

    def on_emergency_stop(self, request, response):
        self.neutralise(time.monotonic())
        self.relay_on = False
        self.publish_relay()
        response.success = True
        response.message = 'emergency stop: outputs 1500, relay off'
        self.get_logger().warn(response.message)
        return response

    # ------------------------------------------------------------------ outputs
    def tick(self):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now

        stale = self.last_command is None or (now - self.last_command) > self.timeout_s
        if stale:
            if self.last_command is not None and not self.timed_out:
                self.get_logger().warn(
                    f'no ThrusterPwm for {self.timeout_s} s: outputs to {NEUTRAL}')
            self.timed_out = self.last_command is not None
            self.neutralise(now)

        for name in THRUSTERS:
            thrust = self.to_thrust(self.channels[name].held) if self.relay_on else 0.0
            self.thrust_pub[name].publish(Float64(data=float(thrust)))

        target = steer_angle_rad(self.channels['steer'].held, **self.steer_args)
        angle = self.steer_servo.step(target, dt)
        for name in STEERED:
            self.steer_pub[name].publish(Float64(data=float(angle)))

    def publish_relay(self):
        self.relay_pub.publish(Int8(data=1 if self.relay_on else 0))


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
