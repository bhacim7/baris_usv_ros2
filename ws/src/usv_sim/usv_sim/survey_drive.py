"""
survey_drive: drives the course's survey route by itself, from Gazebo's TRUE pose (sim only).

For the map-quality measurements (phase 6 R1.6): the same route every run, so a change to
perception or the mapper is compared on equal terms. The pilot is usv_sim.survey; the
route is the course file's `survey` section.

  in   /usv/sim/navsat  sensor_msgs/NavSatFix  true antenna position (no GPS error)
       /usv/sim/imu     sensor_msgs/Imu        true attitude
  out  /usv/thrusters/pwm  usv_interfaces/ThrusterPwm, 25 Hz, through usv_core.mixer like
                           the navigator's

It does not read /usv/fix or /usv/heading: the sensors are what is being measured, not what
steers. Start it without the navigator (sim.launch.py survey:=true does both). It waits
start_delay_s after the first true pose (Gazebo, perception and the mapper settle), drives
the route, then holds neutral until Ctrl+C.

Topic and service names are relative to the boat's namespace (sim.launch.py
boat:=azure|marine): the /usv/... names above are /azure/... on Azure, /marine/... on Marine.
"""

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from usv_core.mixer import Mixer
from usv_interfaces.msg import ThrusterPwm
from usv_sim import course as course_mod
from usv_sim.geo import enu
from usv_sim.sensors import compass_deg, yaw_from_quaternion
from usv_sim.survey import PilotConfig, SurveyPilot
from usv_sim.zed import rotate
import yaml


class SurveyDrive(Node):

    def __init__(self):
        super().__init__('survey_drive')
        p = self.declare_parameter
        course_name = p('course', 'roboboat_2026').value
        course_path = p('course_file', '').value or os.path.join(
            get_package_share_directory('usv_sim'), 'courses', course_name + '.yaml')
        with open(course_path) as f:
            self.course = course_mod.load(yaml.safe_load(f))
        if not self.course.survey:
            raise RuntimeError(f'course {course_name} has no survey route')
        self.antenna_mount = tuple(p('gnss_antenna_mount_m', [-0.46, 0.0, 0.68]).value)
        self.start_delay = p('start_delay_s', 8.0).value
        defaults = PilotConfig()
        self.pilot = SurveyPilot(self.course.survey, PilotConfig(
            cruise_pwm=p('cruise_pwm', defaults.cruise_pwm).value,
            lookahead_m=p('lookahead_m', defaults.lookahead_m).value,
            reach_m=p('reach_m', defaults.reach_m).value))
        self.mixer = Mixer()
        self.origin = self.course.origin

        self.antenna = None           # true antenna ENU (x, y, z)
        self.orientation = None       # true attitude (x, y, z, w)
        self.first_pose = None        # time of the first complete true pose
        self.started = None
        self.reported = 0
        self.finished = False

        self.create_subscription(NavSatFix, 'sim/navsat', self.on_navsat, 10)
        self.create_subscription(Imu, 'sim/imu', self.on_imu, 10)
        self.pwm_pub = self.create_publisher(ThrusterPwm, 'thrusters/pwm', 10)
        self.create_timer(1.0 / 25.0, self.tick)
        self.get_logger().info(
            f'survey_drive: {len(self.course.survey)} steps on {course_name}, '
            f'starting {self.start_delay:.0f} s after the first true pose')

    def on_navsat(self, msg):
        x, y = enu(msg.latitude, msg.longitude, self.origin['lat'], self.origin['lon'])
        self.antenna = (x, y, msg.altitude)

    def on_imu(self, msg):
        q = msg.orientation
        self.orientation = (q.x, q.y, q.z, q.w)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.antenna is None or self.orientation is None:
            return
        now = self.now()
        if self.first_pose is None:
            self.first_pose = now
        if now - self.first_pose < self.start_delay:
            return                    # nothing published: the boat floats as launched
        if self.started is None:
            self.started = now
            self.get_logger().info('survey started')
        lever = rotate(self.orientation, self.antenna_mount)
        east, north = self.antenna[0] - lever[0], self.antenna[1] - lever[1]
        heading = compass_deg(yaw_from_quaternion(*self.orientation))
        forward, yaw = self.pilot.step(now - self.started, east, north, heading)
        for t, message in self.pilot.log[self.reported:]:
            self.get_logger().info(f'{t:6.1f} s  {message}')
        self.reported = len(self.pilot.log)
        if self.pilot.done:
            cmd = self.mixer.neutral()
            if not self.finished:
                self.finished = True
                self.get_logger().info(
                    f'survey done in {now - self.started:.0f} s; holding neutral, Ctrl+C '
                    'to stop (the bag and the map are written then)')
        else:
            cmd = self.mixer.mix(forward, yaw)
        msg = ThrusterPwm()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.rear_left, msg.rear_right = cmd.rear_left, cmd.rear_right
        msg.front_left, msg.front_right = cmd.front_left, cmd.front_right
        msg.steer = cmd.steer
        msg.force = cmd.force
        self.pwm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SurveyDrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
