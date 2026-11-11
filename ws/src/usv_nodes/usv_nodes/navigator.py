"""
navigator: the autonomy loop - legacy core/nav_process.py nav_worker as a ROS 2 node.

All decisions are made in usv_core; this node only feeds them and publishes what they
decide. Parameter `competition` picks the rule book (phase 6 R2.1):
  teknofest   usv_core.teknofest_navigator.TeknofestNavigator, ported cycle for cycle and
              tested against the legacy loop; GPS points from the GCS, the object memory
              (/usv/buoys)
  roboboat    usv_core.roboboat_navigator.RoboBoatNavigator: the run (entry gates, the
              chosen tasks, return, exit gates) from the world map (/usv/map) and the camera
              frames (/usv/detections); roboboat.* parameters; no GPS points
Both share the GCS services, manual / auto and the emergency latch (usv_core.nav_base).

  in   /usv/fix           sensor_msgs/NavSatFix      GPS fix (fc_bridge / sim_bridge)
       /usv/speed         std_msgs/Float32           ground speed, m/s
       /usv/heading       std_msgs/Float32           selected heading (heading_source)
       /usv/buoys         usv_interfaces/BuoyArray   object memory (perception), teknofest
       /usv/map           usv_interfaces/LandmarkArray  world map (mapper), roboboat
       /usv/detections    usv_interfaces/DetectionArray camera frames (perception), roboboat
       /usv/relay_state   std_msgs/Int8              -1 unknown, 0 off, 1 on
  out  /usv/thrusters/pwm usv_interfaces/ThrusterPwm every cycle (loop_hz, 25 Hz)
       /usv/nav_status    usv_interfaces/NavStatus   status_hz (10 Hz)
       /usv/mission/waypoints  WaypointArray, latched, on start and on every change
                               (teknofest)
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

Topic and service names are relative to the boat's namespace (sim.launch.py
boat:=azure|marine): the /usv/... names above are /azure/... on Azure, /marine/... on Marine.
"""

import math
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
from usv_core.mission import competition_factories, MissionManager, Request
from usv_core.mission_tasks import factories as mission_task_factories
from usv_core.nav_config import WAYPOINT_NAMES
from usv_core.roboboat_navigator import Perception, RoboBoatNavigator
from usv_core.safety import preflight_ready
from usv_core.teknofest_navigator import TeknofestNavigator
from usv_interfaces.msg import (BuoyArray, DetectionArray, LandmarkArray, NavStatus,
                                TaskReport, TaskRequest, ThrusterPwm, Waypoint,
                                WaypointArray)
from usv_interfaces.srv import SetTargetColor, SetTask, SetWaypoint

from usv_nodes.navigator_io import (buoy_to_object, debris_config_from, detections_to_dicts,
                                    drive_config_from, landmark_to_dict,
                                    mission_config_from, nav_config_from, path_points,
                                    run_config_from, target_color)

NAN = float('nan')


def _f(value):
    return NAN if value is None else float(value)


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')
        any_type = ParameterDescriptor(dynamic_typing=True)

        def lookup(name, default):
            # Several readers may ask for the same parameter (roboboat.task_timeout_s: the
            # readiness run and the 2027 mission): declare it once, then read it
            if self.has_parameter(name):
                return self.get_parameter(name).value
            return self.declare_parameter(name, default, any_type).value

        self.cfg = nav_config_from(lookup)
        start_manual = self.declare_parameter('start_manual', True).value
        status_hz = self.declare_parameter('status_hz', 10.0).value
        self.competition = self.declare_parameter('competition', 'teknofest').value
        if self.competition not in ('teknofest', 'roboboat'):
            raise ValueError(f'competition {self.competition!r}: teknofest or roboboat')
        self.roboboat = self.competition == 'roboboat'

        if self.roboboat:
            run_cfg = run_config_from(lookup)
            self.drive_cfg = drive_config_from(lookup)
            self.debris_cfg = debris_config_from(lookup)
            # readiness (frozen 2026 run, video) or roboboat_2027 (the mission manager)
            self.mission_name, mission_cfg = mission_config_from(lookup)
            capabilities = [c for c in str(lookup('boat_capabilities', '')).split(',') if c]
            mission = None
            if self.mission_name == 'roboboat_2027':
                written, home = mission_task_factories(self.cfg, self.drive_cfg)
                mission = MissionManager(mission_cfg, competition_factories(written),
                                         capabilities, home=home)
            self.nav = RoboBoatNavigator(self.cfg, run_cfg, start_time=self.now_s(),
                                         mission_started=False, manual=start_manual,
                                         log=self.get_logger().info,
                                         drive_cfg=self.drive_cfg, debris_cfg=self.debris_cfg,
                                         mission=mission)
        else:
            self.nav = TeknofestNavigator(self.cfg, start_time=self.now_s(),
                                          mission_started=False, manual=start_manual,
                                          log=self.get_logger().info)
        self.fix = None
        self.speed = None
        self.heading = None
        self.buoys = []
        self.perception = Perception()
        self.published_waypoints = None

        self.create_subscription(NavSatFix, 'fix', self.on_fix, 10)
        self.create_subscription(Float32, 'speed', self.on_speed, 10)
        self.create_subscription(Float32, 'heading', self.on_heading, 10)
        if self.roboboat:
            self.create_subscription(LandmarkArray, 'map', self.on_map, 10)
            self.create_subscription(DetectionArray, 'detections', self.on_detections,
                                     10)
        else:
            self.create_subscription(BuoyArray, 'buoys', self.on_buoys, 10)
        self.create_subscription(Int8, 'relay_state', self.on_relay, 10)
        if self.roboboat:
            # Tasks asked during the run (RoboCommand via the GCS; telemetry_bridge)
            self.create_subscription(TaskRequest, 'task_request', self.on_task_request, 10)
            # What the run reports (gates passed, points reached): to the GCS, RoboCommand
            self.report_pub = self.create_publisher(TaskReport, 'task_report', 10)

        self.pwm_pub = self.create_publisher(ThrusterPwm, 'thrusters/pwm', 10)
        self.status_pub = self.create_publisher(NavStatus, 'nav_status', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoints_pub = self.create_publisher(WaypointArray, 'mission/waypoints',
                                                   latched)

        self.create_service(SetTask, 'set_task', self.srv_set_task)
        self.create_service(SetBool, 'set_manual', self.srv_set_manual)
        self.create_service(SetWaypoint, 'set_waypoint', self.srv_set_waypoint)
        self.create_service(SetTargetColor, 'set_target_color', self.srv_set_color)
        self.create_service(Trigger, 'navigator/emergency_stop', self.srv_emergency)

        self.create_timer(1.0 / max(1.0, self.cfg.loop_hz), self.tick)
        self.create_timer(1.0 / status_hz, self.publish_status)
        if self.roboboat and self.mission_name == 'roboboat_2027':
            m = self.nav.run
            self.get_logger().info(
                f'navigator: RoboBoat 2027 mission, plan {list(m.cfg.plan)}, task timeout '
                f'{m.cfg.task_timeout_s:.0f} s, run timeout {m.cfg.run_timeout_s:.0f} s, '
                f'boat can: {", ".join(sorted(m.capabilities)) or "nothing extra"}')
        elif self.roboboat:
            run = self.nav.run
            self.get_logger().info(
                f'navigator: RoboBoat readiness run {run.plan}, task timeout '
                f'{run.cfg.task_timeout_s:.0f} s, run timeout {run.cfg.run_timeout_s:.0f} s, '
                f'hints {run.cfg.hints or "none"}, look-ahead '
                f'{self.drive_cfg.lookahead_m:.1f} m, debris field '
                f'{self.debris_cfg.depth_m:.0f} m x +/-{self.debris_cfg.half_width_m:.0f} m')
        else:
            self.publish_waypoints()
        self.get_logger().info(
            f'navigator: {self.competition}, {self.cfg.loop_hz:.0f} Hz, task {self.nav.task}, '
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

    def on_map(self, msg):
        self.perception.landmarks = [landmark_to_dict(lm) for lm in msg.landmarks]
        self.perception.origin = (msg.origin_latitude, msg.origin_longitude)

    def on_task_request(self, msg):
        params = dict(zip(msg.keys, msg.values))
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            params.update(lat=msg.latitude, lon=msg.longitude)
        self.nav.request(Request(msg.task, urgent=msg.urgent, params=params,
                                 source=msg.source, request_id=msg.request_id))

    def on_detections(self, msg):
        self.perception.detections, self.perception.detections_time = \
            detections_to_dicts(msg)

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
        seen = self.perception if self.roboboat else self.buoys
        cmd = self.nav.step(self.now_s(), (lat, lon), self.speed, heading, seen)
        self.publish_pwm(cmd)
        if self.roboboat:
            for report in self.nav.take_reports():
                self.publish_report(report)
        if not self.roboboat and self.nav.waypoints != self.published_waypoints:
            self.publish_waypoints()

    def publish_report(self, report):
        msg = TaskReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.report_id = int(report['id'])
        msg.task = str(report.get('task', ''))
        msg.type = str(report['type'])
        msg.latitude = _f(report.get('lat'))
        msg.longitude = _f(report.get('lon'))
        fields = report.get('fields') or {}
        msg.keys = [str(k) for k in fields]
        msg.values = [str(v) for v in fields.values()]
        self.report_pub.publish(msg)
        self.get_logger().info(f'Report {msg.report_id}: {msg.type} {dict(fields)} '
                               f'({msg.task})')

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
        if self.roboboat:
            msg.task_queue = [str(t) for t in self.nav.upcoming()]
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
        for east, north in path_points(self.nav.plan_path()):
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
