"""
navigator: the autonomy loop - legacy core/nav_process.py nav_worker as a ROS 2 node.

All decisions are made by usv_core.navigator.Navigator (ported cycle for cycle and tested
against the legacy loop); this node only feeds it and publishes what it decides.

  in   /usv/fix           sensor_msgs/NavSatFix      GPS fix (fc_bridge / sim_bridge)
       /usv/speed         std_msgs/Float32           ground speed, m/s
       /usv/heading       std_msgs/Float32           selected heading (heading_source)
       /usv/buoys         usv_interfaces/BuoyArray   object memory (perception)
       /usv/relay_state   std_msgs/Int8              -1 unknown, 0 off, 1 on
  out  /usv/thrusters/pwm usv_interfaces/ThrusterPwm every cycle (loop_hz, 25 Hz)
       /usv/nav_status    usv_interfaces/NavStatus   status_hz (10 Hz)
       /usv/mission/waypoints  WaypointArray, latched, on start and on every change
       /usv/plan          nav_msgs/Path              A* path, ENU metres from the first fix
                                                     (frame usv_local), status_hz
  services (the GCS commands; each is queued and handled at the start of the next cycle,
  where legacy handled its command queue):
       /usv/set_task  /usv/set_manual  /usv/set_waypoint  /usv/set_target_color
       /usv/navigator/emergency_stop   (fc_bridge / sim_bridge serve /usv/emergency_stop;
                                        two servers cannot share one service name)

Start-up as legacy main_orchestrator: MANUAL mode (parameter start_manual), and the mission
counts as started only once the pre-flight check has passed (GPS latitude and heading both
non-zero). On shutdown (Ctrl+C or SIGTERM from launch) one neutral command goes out.

Every time comes from the node clock; the buoys' last_seen uses the same clock.
"""

import signal

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int8
from std_srvs.srv import SetBool, Trigger
from usv_core.mixer import NEUTRAL
from usv_core.nav_config import WAYPOINT_NAMES
from usv_core.navigator import Navigator
from usv_core.safety import preflight_ready
from usv_interfaces.msg import (BuoyArray, NavStatus, ThrusterPwm, Waypoint,
                                WaypointArray)
from usv_interfaces.srv import SetTargetColor, SetTask, SetWaypoint

from usv_nodes.navigator_io import buoy_to_object, nav_config_from, path_points, target_color

NAN = float('nan')


def _f(value):
    return NAN if value is None else float(value)


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.cfg = nav_config_from(
            lambda name, default: self.declare_parameter(name, default, any_type).value)
        start_manual = self.declare_parameter('start_manual', True).value
        status_hz = self.declare_parameter('status_hz', 10.0).value

        self.nav = Navigator(self.cfg, start_time=self.now_s(), mission_started=False,
                             manual=start_manual, log=self.get_logger().info)
        self.fix = None
        self.speed = None
        self.heading = None
        self.buoys = []
        self.published_waypoints = None

        self.create_subscription(NavSatFix, '/usv/fix', self.on_fix, 10)
        self.create_subscription(Float32, '/usv/speed', self.on_speed, 10)
        self.create_subscription(Float32, '/usv/heading', self.on_heading, 10)
        self.create_subscription(BuoyArray, '/usv/buoys', self.on_buoys, 10)
        self.create_subscription(Int8, '/usv/relay_state', self.on_relay, 10)

        self.pwm_pub = self.create_publisher(ThrusterPwm, '/usv/thrusters/pwm', 10)
        self.status_pub = self.create_publisher(NavStatus, '/usv/nav_status', 10)
        self.plan_pub = self.create_publisher(Path, '/usv/plan', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoints_pub = self.create_publisher(WaypointArray, '/usv/mission/waypoints',
                                                   latched)

        self.create_service(SetTask, '/usv/set_task', self.srv_set_task)
        self.create_service(SetBool, '/usv/set_manual', self.srv_set_manual)
        self.create_service(SetWaypoint, '/usv/set_waypoint', self.srv_set_waypoint)
        self.create_service(SetTargetColor, '/usv/set_target_color', self.srv_set_color)
        self.create_service(Trigger, '/usv/navigator/emergency_stop', self.srv_emergency)

        self.create_timer(1.0 / max(1.0, self.cfg.loop_hz), self.tick)
        self.create_timer(1.0 / status_hz, self.publish_status)
        self.publish_waypoints()
        self.get_logger().info(
            f'navigator: {self.cfg.loop_hz:.0f} Hz, task {self.nav.task}, '
            f"{'MANUAL' if start_manual else 'AUTO'}, waiting for the pre-flight check "
            '(GPS fix and heading)')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def on_fix(self, msg):
        self.fix = (msg.latitude, msg.longitude)

    def on_speed(self, msg):
        self.speed = msg.data

    def on_heading(self, msg):
        self.heading = msg.data

    def on_buoys(self, msg):
        self.buoys = [buoy_to_object(b) for b in msg.buoys]

    def on_relay(self, msg):
        self.nav.relay_on = None if msg.data < 0 else int(msg.data)

    # ------------------------------------------------------------------ services
    def srv_set_task(self, request, response):
        response.success = bool(request.task)
        if response.success:
            self.nav.submit({'cmd': 'set_task', 'task_name': request.task})
        response.message = f'queued: {request.task}' if response.success else 'empty task'
        return response

    def srv_set_manual(self, request, response):
        self.nav.submit({'cmd': 'set_manual', 'value': bool(request.data)})
        response.success = True
        response.message = 'queued: ' + ('MANUAL' if request.data else 'AUTO')
        return response

    def srv_set_waypoint(self, request, response):
        response.success = 1 <= request.index <= len(WAYPOINT_NAMES)
        if response.success:
            self.nav.submit({'cmd': 'set_gps', 'index': int(request.index),
                             'lat': request.latitude, 'lon': request.longitude})
            response.message = f'queued: GPS{request.index}'
        else:
            response.message = f'index must be 1..{len(WAYPOINT_NAMES)}'
        return response

    def srv_set_color(self, request, response):
        response.success = bool(request.color)
        if response.success:
            self.nav.submit({'cmd': 'set_target_color', 'color': request.color})
        response.message = f'queued: {request.color}' if response.success else 'empty color'
        return response

    def srv_emergency(self, request, response):
        self.nav.submit({'cmd': 'emergency_stop'})
        response.success = True
        response.message = 'queued: emergency stop (navigator latched from the next cycle)'
        return response

    # ------------------------------------------------------------------ loop
    def tick(self):
        lat, lon = self.fix if self.fix is not None else (0.0, 0.0)
        heading = self.heading if self.heading is not None else 0.0
        if not self.nav.mission_started and preflight_ready(lat, heading):
            self.nav.mission_started = True
            self.get_logger().info('Pre-flight check passed: GPS fix and heading online.')
        cmd = self.nav.step(self.now_s(), (lat, lon), self.speed, heading, self.buoys)
        self.publish_pwm(cmd)
        if self.nav.waypoints != self.published_waypoints:
            self.publish_waypoints()

    def publish_pwm(self, cmd):
        msg = ThrusterPwm()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.rear_left = int(cmd.rear_left)
        msg.rear_right = int(cmd.rear_right)
        msg.front_left = int(cmd.front_left)
        msg.front_right = int(cmd.front_right)
        msg.steer = int(cmd.steer)
        msg.force = bool(cmd.force)
        self.pwm_pub.publish(msg)

    def publish_status(self):
        s = self.nav.status
        msg = NavStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_state = str(s['current_task'])
        msg.manual = bool(self.nav.manual)
        msg.mission_active = bool(self.nav.mission_active)
        msg.emergency = bool(self.nav.emergency.active)
        msg.target_latitude = _f(s['target_lat'])
        msg.target_longitude = _f(s['target_lon'])
        msg.target_distance_m = _f(s['target_dist'])
        msg.advised_course_deg = _f(s['adviced_course'])
        msg.angle_error_deg = _f(s['angle_error'])
        msg.control_error_deg = _f(s['control_error'])
        msg.target_color, msg.target_color_source = target_color(
            self.cfg, self.nav.drone_target_color)
        self.status_pub.publish(msg)

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = 'usv_local'
        for east, north in path_points(self.nav.guidance.path):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = east
            pose.pose.position.y = north
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.plan_pub.publish(path)

    def publish_waypoints(self):
        msg = WaypointArray()
        for i, name in enumerate(WAYPOINT_NAMES, 1):
            lat, lon = self.nav.waypoints[name]
            msg.waypoints.append(Waypoint(index=i, latitude=float(lat or 0.0),
                                          longitude=float(lon or 0.0)))
        self.waypoints_pub.publish(msg)
        self.published_waypoints = dict(self.nav.waypoints)

    def publish_neutral(self):
        self.publish_pwm(NEUTRAL._replace(force=True))


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # rclpy's own signal handling would shut the context down before we could send the
    # neutral command; Ctrl+C and launch's SIGTERM both end up in the `finally` instead.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_neutral()
            node.get_logger().info('navigator stopping: neutral sent')
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
