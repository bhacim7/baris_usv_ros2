"""fake_perception: the simulated ZED 2i side of the boat. Stands in for perception.

Publishes what perception publishes on the boat, from Gazebo's exact sensors plus the
camera's own behaviour (models in zed.py, numbers in config/fake_perception.yaml):

  /usv/heading/zed       std_msgs/Float32 (deg)  once per camera frame, like camera_process:
                         magnetometer -> magnetic heading (tilt compensated) -> the legacy
                         Kalman filter -> + declination (usv_core.heading, unchanged code).
                         The magnetometer reading is computed here from the course file's
                         Earth field and the IMU attitude; Gazebo's magnetometer is not used
                         (see zed.py: it gets the field wrong).
  /usv/attitude          geometry_msgs/Vector3Stamped, x = roll, y = pitch (deg), 5 Hz;
                         ROS convention: roll + = starboard down, pitch + = bow down
  /usv/perception/fps    std_msgs/Float32, frames in the last second, 1 Hz

Inputs: /usv/sim/imu (ros_gz_bridge); the course file (parameter `course`, a name in
usv_sim/courses/), the same file the Gazebo world was generated from.
Buoys (/usv/buoys) come in the next step.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import yaml

from usv_core.heading import HeadingFilter, LEGACY_DECLINATION_DEG, zed_heading
from usv_sim.sensors import make_rng, roll_pitch_from_quaternion
from usv_sim.zed import body_field, FrameClock, magnetic_heading_deg, world_field_enu


class FakePerception(Node):

    def __init__(self):
        super().__init__('fake_perception')
        p = self.declare_parameter
        self.rng = make_rng(p('random_seed', 0).value)
        course_name = p('course', 'golcuk_2026_10_14').value
        course_path = os.path.join(get_package_share_directory('usv_sim'), 'courses',
                                   course_name + '.yaml')
        with open(course_path) as f:
            self.course = yaml.safe_load(f)
        mf = self.course['magnetic_field']
        self.field_enu = world_field_enu(mf['declination_deg'], mf['inclination_deg'],
                                         mf['strength_uT'])
        self.declination = p('magnetic_declination_deg', LEGACY_DECLINATION_DEG).value
        self.heading_noise = p('magnetometer_noise_deg', 1.0).value
        hard_iron_uT = list(p('hard_iron_body_uT', [0.0, 0.0, 0.0]).value)
        self.hard_iron = [v * 1e-6 for v in hard_iron_uT]
        self.compass_offset = p('compass_offset_deg', 0.0).value
        self.clock = FrameClock(
            self.rng,
            fps_normal=p('fps_normal', 30.0).value,
            fps_slow=p('fps_slow', 15.0).value,
            normal_mean_s=p('fps_normal_mean_s', 20.0).value,
            slow_mean_s=p('fps_slow_mean_s', 2.0).value)
        self.heading_filter = HeadingFilter()   # camera_process: process 1e-3, measurement 1e-1

        self.orientation = None       # (x, y, z, w) body -> ENU
        self.frame_times = []
        self.next_frame = time.monotonic()

        self.create_subscription(Imu, '/usv/sim/imu', self.on_imu, 10)
        self.heading_pub = self.create_publisher(Float32, '/usv/heading/zed', 10)
        self.attitude_pub = self.create_publisher(Vector3Stamped, '/usv/attitude', 10)
        self.fps_pub = self.create_publisher(Float32, '/usv/perception/fps', 10)

        self.create_timer(0.002, self.poll_frame)     # frames are due at irregular times
        self.create_timer(0.2, self.publish_attitude)
        self.create_timer(1.0, self.publish_fps)
        self.get_logger().info(
            f'fake_perception ready: course {course_name}, site declination '
            f'{mf["declination_deg"]:+.1f} deg; perception adds {self.declination:+.1f} deg, '
            f'compass offset {self.compass_offset:+.1f} deg, hard iron {hard_iron_uT} uT')

    def on_imu(self, msg):
        q = msg.orientation
        self.orientation = (q.x, q.y, q.z, q.w)

    def poll_frame(self):
        now = time.monotonic()
        if now < self.next_frame:
            return
        self.next_frame = max(self.next_frame + self.clock.next_period(), now)
        self.frame_times.append(now)
        self.on_frame()

    def on_frame(self):
        """One camera frame: as camera_process, the heading is updated once per frame."""
        if self.orientation is None:
            return
        field = body_field(self.field_enu, self.orientation)
        raw = magnetic_heading_deg(field, self.orientation, self.hard_iron)
        raw = (raw + self.compass_offset + self.rng.gauss(0.0, self.heading_noise)) % 360.0
        heading = zed_heading(self.heading_filter, raw, self.declination)
        self.heading_pub.publish(Float32(data=float(heading)))

    def publish_attitude(self):
        if self.orientation is None:
            return
        roll, pitch = roll_pitch_from_quaternion(*self.orientation)
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # FLU body: + rotation about x lifts the left side (starboard down), + about y
        # (pointing left) turns the bow down. That is already the ROS reading.
        msg.vector.x = math.degrees(roll)
        msg.vector.y = math.degrees(pitch)
        self.attitude_pub.publish(msg)

    def publish_fps(self):
        now = time.monotonic()
        self.frame_times = [t for t in self.frame_times if now - t <= 1.0]
        self.fps_pub.publish(Float32(data=float(len(self.frame_times))))


def main(args=None):
    rclpy.init(args=args)
    node = FakePerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
