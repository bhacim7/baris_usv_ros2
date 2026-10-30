"""fake_perception: the simulated ZED 2i side of the boat. Stands in for perception.

Publishes what perception publishes on the boat, from Gazebo's exact sensors plus the
camera's own behaviour (models in zed.py and camera.py, numbers in
config/fake_perception.yaml):

  /usv/buoys             usv_interfaces/BuoyArray, once per camera frame. Simulated YOLO
                         boxes and ZED depth (camera.py) go through the boat's own code,
                         usv_core.detection.process_frame + the object memory, with the
                         heading the navigation uses (/usv/heading) and the GPS fix
                         (/usv/fix), exactly as camera_process does it.
  /usv/heading/zed       std_msgs/Float32 (deg)  once per camera frame, like camera_process:
                         magnetometer -> magnetic heading (tilt compensated) -> the legacy
                         Kalman filter -> + declination (usv_core.heading, unchanged code).
                         The magnetometer reading is computed here from the course file's
                         Earth field and the IMU attitude; Gazebo's magnetometer is not used
                         (see zed.py: it gets the field wrong).
  /usv/attitude          geometry_msgs/Vector3Stamped, x = roll, y = pitch (deg), 5 Hz;
                         ROS convention: roll + = starboard down, pitch + = bow down
  /usv/perception/fps    std_msgs/Float32, frames in the last second, 1 Hz

Inputs: /usv/sim/imu and /usv/sim/navsat (exact, from Gazebo through ros_gz_bridge) give the
boat's true pose for the camera; /usv/heading and /usv/fix are what the boat's perception
would use. The course file (parameter `course`, a name in usv_sim/courses/) gives the buoys
and the Earth's field: use the course the Gazebo world was generated from.
"""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
import yaml

from usv_core.detection import memory_snapshot, process_frame
from usv_core.heading import HeadingFilter, LEGACY_DECLINATION_DEG, zed_heading
from usv_core.object_memory import ObjectMemoryManager
from usv_interfaces.msg import Buoy as BuoyMsg, BuoyArray
from usv_sim.camera import Buoy, CameraModel, Frame, Settings
from usv_sim.geo import enu
from usv_sim.sensors import make_rng, roll_pitch_from_quaternion
from usv_sim.zed import body_field, FrameClock, magnetic_heading_deg, rotate, world_field_enu


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

        # Camera and YOLO
        self.cam = CameraModel(
            width=p('image_width', 1280).value,
            height=p('image_height', 720).value,
            hfov_deg=p('camera_hfov_deg', 110.0).value,
            mount=tuple(p('camera_mount_m', [0.29, 0.06, 0.18]).value))
        self.settings = Settings(
            max_range_m=p('max_range_m', 20.0).value,
            min_box_px=p('min_box_px', 4.0).value,
            full_detection_px=p('full_detection_px', 25.0).value,
            detect_prob=p('detect_prob', 0.98).value,
            box_jitter_px=p('box_jitter_px', 2.0).value,
            class_confusion=p('class_confusion', 0.02).value,
            occlusion_fraction=p('occlusion_fraction', 0.6).value,
            depth_error_coeff=p('depth_error_coeff', 0.0033).value,
            depth_pixel_noise=p('depth_pixel_noise', 0.005).value,
            depth_nan_prob=p('depth_nan_prob', 0.10).value)
        self.antenna_mount = tuple(p('gnss_antenna_mount_m', [-0.60, 0.0, 0.75]).value)
        self.origin = self.course['origin']
        types = self.course['buoy_types']
        self.buoys = []
        for b in self.course['buoys']:
            spec = types[b['type']]
            x, y = enu(b['lat'], b['lon'], self.origin['lat'], self.origin['lon'])
            self.buoys.append(Buoy(x, y, spec['diameter'] / 2.0, spec['height'],
                                   spec['class_id']))
        self.memory = ObjectMemoryManager()

        self.orientation = None       # (x, y, z, w) body -> ENU, exact
        self.antenna_enu = None       # exact antenna position (x, y, z)
        self.heading = None           # /usv/heading, what perception uses on the boat
        self.fix = None               # /usv/fix (lat, lon)
        self.frame_times = []
        self.next_frame = time.monotonic()

        self.create_subscription(Imu, '/usv/sim/imu', self.on_imu, 10)
        self.create_subscription(NavSatFix, '/usv/sim/navsat', self.on_true_navsat, 10)
        self.create_subscription(Float32, '/usv/heading', self.on_heading, 10)
        self.create_subscription(NavSatFix, '/usv/fix', self.on_fix, 10)
        self.heading_pub = self.create_publisher(Float32, '/usv/heading/zed', 10)
        self.attitude_pub = self.create_publisher(Vector3Stamped, '/usv/attitude', 10)
        self.fps_pub = self.create_publisher(Float32, '/usv/perception/fps', 10)
        self.buoys_pub = self.create_publisher(BuoyArray, '/usv/buoys', 10)

        self.create_timer(0.002, self.poll_frame)     # frames are due at irregular times
        self.create_timer(0.2, self.publish_attitude)
        self.create_timer(1.0, self.publish_fps)
        self.get_logger().info(
            f'fake_perception ready: course {course_name} ({len(self.buoys)} buoys), site '
            f'declination {mf["declination_deg"]:+.1f} deg; perception adds '
            f'{self.declination:+.1f} deg, compass offset {self.compass_offset:+.1f} deg, '
            f'hard iron {hard_iron_uT} uT')

    # ------------------------------------------------------------------ inputs
    def on_imu(self, msg):
        q = msg.orientation
        self.orientation = (q.x, q.y, q.z, q.w)

    def on_true_navsat(self, msg):
        x, y = enu(msg.latitude, msg.longitude, self.origin['lat'], self.origin['lon'])
        self.antenna_enu = (x, y, msg.altitude)

    def on_heading(self, msg):
        self.heading = msg.data

    def on_fix(self, msg):
        self.fix = (msg.latitude, msg.longitude)

    # ------------------------------------------------------------------ frames
    def poll_frame(self):
        now = time.monotonic()
        if now < self.next_frame:
            return
        self.next_frame = max(self.next_frame + self.clock.next_period(), now)
        self.frame_times.append(now)
        self.on_frame()

    def on_frame(self):
        """One camera frame, in camera_process order: heading sample, then detections."""
        if self.orientation is None:
            return
        field = body_field(self.field_enu, self.orientation)
        raw = magnetic_heading_deg(field, self.orientation, self.hard_iron)
        raw = (raw + self.compass_offset + self.rng.gauss(0.0, self.heading_noise)) % 360.0
        heading = zed_heading(self.heading_filter, raw, self.declination)
        self.heading_pub.publish(Float32(data=float(heading)))

        if self.antenna_enu is None:
            return
        # Boat origin from the exact antenna position and attitude
        lever = rotate(self.orientation, self.antenna_mount)
        boat = tuple(a - l for a, l in zip(self.antenna_enu, lever))
        frame = Frame(self.buoys, boat, self.orientation, self.cam, self.settings, self.rng)
        now = self.get_clock().now()
        lat, lon = self.fix if self.fix is not None else (0, 0)
        process_frame(frame.boxes(), frame.depth_at, self.cam.width, self.cam.height,
                      self.cam.cx, self.cam.fx, self.heading, lat, lon, self.memory,
                      now=now.nanoseconds * 1e-9)
        self.publish_buoys(now)

    def publish_buoys(self, now):
        out = BuoyArray()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'map'
        out.image_width = self.cam.width
        for obj in memory_snapshot(self.memory):
            b = BuoyMsg()
            b.id = int(obj['id'])
            b.class_id = int(obj['cid']) if obj['cid'] is not None else 255
            b.latitude = float(obj['lat'])
            b.longitude = float(obj['lon'])
            b.position_ok = obj['pos_ok']
            b.seen_count = int(obj['seen'])
            seen = obj['last_seen']
            b.last_seen = Time(sec=int(seen), nanosec=int((seen % 1.0) * 1e9))
            b.distance_m = float(obj['dist']) if obj['dist'] is not None else float('nan')
            b.cx = int(obj['cx']) if obj['cx'] is not None else -1
            b.cy = int(obj['cy']) if obj['cy'] is not None else -1
            b.area_px = int(obj['area']) if obj['area'] is not None else 0
            out.buoys.append(b)
        self.buoys_pub.publish(out)

    # ------------------------------------------------------------------ other outputs
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
