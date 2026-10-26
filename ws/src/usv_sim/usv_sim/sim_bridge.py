"""sim_bridge: the simulated boat's flight-controller side. Stands in for fc_bridge.

It publishes what fc_bridge publishes on the boat, no more: the Orange Cube's view of the
world. The ZED's view (its heading, attitude, buoys) comes from fake_perception, as it
comes from perception on the boat.

Sensor side: Gazebo NavSat, IMU and odometry (bridged to /usv/sim/*) ->
  /usv/fix         sensor_msgs/NavSatFix   GPS_RAW_INT: Here Pro without RTK, at the rate
                   seen in the 2026-10-14 video, delayed, drifting by metres, 1e-7 deg steps
  /usv/speed       std_msgs/Float32 (m/s)  ground speed (VFR_HUD), GPS-velocity noise
  /usv/heading/fc  std_msgs/Float32 (deg)  EKF heading (GLOBAL_POSITION_INT.hdg),
                   clockwise from north, 0.01 deg steps, compass error + noise
Models in sensors.py, numbers in config/sim_bridge.yaml.

Command side:
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

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Float64, Int8
from std_srvs.srv import SetBool, Trigger

from usv_interfaces.msg import ThrusterPwm
from usv_sim.actuators import PwmToThrust, ServoWriteThinning, SlewLimiter, steer_angle_rad
from usv_sim.sensors import (
    compass_deg, DelayLine, GaussMarkov, make_rng, offset_latlon, quantize, yaw_from_quaternion)

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

        self.setup_sensors()

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

    # ------------------------------------------------------------------ sensors
    def setup_sensors(self):
        p = self.declare_parameter
        rng = make_rng(p('random_seed', 0).value)
        self.fix_delay = DelayLine(p('fix_delay_s', 0.1).value)
        tau = p('gps_bias_tau_s', 60.0).value
        bias_sigma = p('gps_bias_sigma_m', 1.0).value
        self.gps_bias = (GaussMarkov(bias_sigma, tau, rng), GaussMarkov(bias_sigma, tau, rng))
        self.gps_white = p('gps_white_sigma_m', 0.15).value
        self.speed_sigma = p('speed_sigma_mps', 0.05).value
        self.hdg_offset = p('heading_fc_offset_deg', 0.0).value
        self.hdg_sigma = p('heading_fc_sigma_deg', 1.0).value
        self.rng = rng
        self.sigma_total = math.hypot(bias_sigma, self.gps_white)

        self.true_fix = None             # latest Gazebo NavSat message
        self.true_speed = None           # m/s
        self.true_yaw = None             # ENU rad
        self.last_bias_step = None

        self.create_subscription(NavSatFix, '/usv/sim/navsat', self.on_navsat, 10)
        self.create_subscription(Odometry, '/usv/sim/odometry', self.on_odometry, 10)
        self.create_subscription(Imu, '/usv/sim/imu', self.on_imu, 10)

        self.fix_pub = self.create_publisher(NavSatFix, '/usv/fix', 10)
        self.speed_pub = self.create_publisher(Float32, '/usv/speed', 10)
        self.hdg_pub = self.create_publisher(Float32, '/usv/heading/fc', 10)

        self.create_timer(1.0 / p('fix_rate_hz', 2.0).value, self.publish_fix)
        self.create_timer(1.0 / p('speed_rate_hz', 5.0).value, self.publish_speed)
        self.create_timer(1.0 / p('heading_fc_rate_hz', 5.0).value, self.publish_heading_fc)

    def on_navsat(self, msg):
        now = time.monotonic()
        self.true_fix = msg
        # The receiver's error wanders with time, whether or not we publish
        if self.last_bias_step is not None:
            dt = now - self.last_bias_step
            for b in self.gps_bias:
                b.step(dt)
        self.last_bias_step = now
        north = self.gps_bias[0].value + self.rng.gauss(0.0, self.gps_white)
        east = self.gps_bias[1].value + self.rng.gauss(0.0, self.gps_white)
        lat, lon = offset_latlon(msg.latitude, msg.longitude, north, east)
        self.fix_delay.push(now, (quantize(lat, 1e-7), quantize(lon, 1e-7), msg.altitude))

    def on_odometry(self, msg):
        v = msg.twist.twist.linear
        self.true_speed = math.hypot(v.x, v.y)

    def on_imu(self, msg):
        q = msg.orientation
        self.true_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def publish_fix(self):
        sample = self.fix_delay.latest(time.monotonic())
        if sample is None:
            return
        out = NavSatFix()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'gps'
        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        out.latitude, out.longitude, out.altitude = sample
        var = self.sigma_total ** 2
        out.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4.0 * var]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.fix_pub.publish(out)

    def publish_speed(self):
        if self.true_speed is None:
            return
        speed = max(0.0, self.true_speed + self.rng.gauss(0.0, self.speed_sigma))
        self.speed_pub.publish(Float32(data=float(speed)))

    def publish_heading_fc(self):
        if self.true_yaw is None:
            return
        hdg = compass_deg(self.true_yaw) + self.hdg_offset + self.rng.gauss(0.0, self.hdg_sigma)
        self.hdg_pub.publish(Float32(data=float(quantize(hdg, 0.01) % 360.0)))


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
