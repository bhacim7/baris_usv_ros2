"""
telemetry_bridge: the boat's end of the GCS link, over the Ubiquiti network and the RFD900x.

Replaces the legacy telem_process (and the command part of nav_process). The protocol is
the 2026 one (usv_core.gcs_link); what is new is that it runs over two links at once and
every packet goes over both (RoboBoat 2027, step 5a):

  telemetry  every broadcast_s (and at once when a command was done, so the GCS sees its
             ack quickly): the 2026 short-key packet; waypoint mini packets replace a tick
             when the waypoints change and every waypoint_refresh_s
  commands   a command addressed to this boat (target_id = the boat profile's number) is
             done once, whichever link brings it first (gcs_link.CommandFilter), and acked
             (aq in the telemetry) once its service has answered

  GCS command       service called (relative to the boat's namespace)
  set_manual        set_manual (std_srvs/SetBool)
  set_task          set_task (usv_interfaces/SetTask)
  set_gps           set_waypoint (usv_interfaces/SetWaypoint)
  set_target_color  set_target_color (usv_interfaces/SetTargetColor)
  set_relay         set_relay (std_srvs/SetBool; fc_bridge, sim: sim_bridge)
  emergency_stop    emergency_stop (fc_bridge / sim_bridge) and navigator/emergency_stop
  task              no service: published on task_request (usv_interfaces/TaskRequest)
                    for the mission manager (2027: judges' and operator's tasks)
  others            acked and ignored (report_status: telemetry runs on its own clock;
                    manual_control: the 2026 boat ignored it too)

Subscribes: nav_status, thrusters/pwm, fix, heading, speed, relay_state, perception/fps,
mission/waypoints. Links: udp_listen_port / gcs_host:gcs_port (Ubiquiti; empty gcs_host
= off) and serial_port at serial_baud (RFD; empty = off).
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int8
from std_srvs.srv import SetBool, Trigger
from usv_core import gcs_link
from usv_interfaces.msg import NavStatus, TaskRequest, ThrusterPwm, WaypointArray
from usv_interfaces.srv import SetTargetColor, SetTask, SetWaypoint


class TelemetryBridge(Node):

    def __init__(self):
        super().__init__('telemetry_bridge')
        p = self.declare_parameter
        self.boat = int(p('boat_number', 1).value)
        self.period = float(p('broadcast_s', 0.3).value)
        self.wp_refresh = float(p('waypoint_refresh_s', 10.0).value)
        links = []
        gcs_host = p('gcs_host', '').value
        if gcs_host:
            try:
                links.append(gcs_link.UdpLink(p('udp_listen_port', 15601).value,
                                              [(gcs_host, p('gcs_port', 15600).value)]))
            except OSError as e:
                self.get_logger().error(f'Ubiquiti link: {e}; going on without it')
        serial_port = p('serial_port', '').value
        if serial_port:
            try:
                links.append(gcs_link.SerialLink(serial_port, p('serial_baud', 57600).value))
            except OSError as e:
                self.get_logger().error(f'RFD {serial_port}: {e}; going on without it')
        self.links = gcs_link.Links(links)
        self.filter = gcs_link.CommandFilter()
        self.in_flight = None         # cq whose service call has not answered yet
        self.state = {}
        self.q = 0
        self.waypoints, self.wp_sent, self.wp_sent_t, self.wp_cycle = {}, {}, 0.0, 0
        self.acked = -1

        self.create_subscription(NavStatus, 'nav_status', self.on_status, 10)
        self.create_subscription(ThrusterPwm, 'thrusters/pwm', self.on_pwm, 10)
        self.create_subscription(NavSatFix, 'fix', self.on_fix, 10)
        self.create_subscription(Float32, 'heading', self.on_value('heading'), 10)
        self.create_subscription(Float32, 'speed', self.on_value('speed'), 10)
        self.create_subscription(Float32, 'perception/fps', self.on_value('fps'), 10)
        self.create_subscription(Int8, 'relay_state', self.on_value('relay'), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(WaypointArray, 'mission/waypoints', self.on_waypoints,
                                 latched)
        self.task_pub = self.create_publisher(TaskRequest, 'task_request', 10)
        self.srv_clients = {
            'set_manual': self.create_client(SetBool, 'set_manual'),
            'set_task': self.create_client(SetTask, 'set_task'),
            'set_gps': self.create_client(SetWaypoint, 'set_waypoint'),
            'set_target_color': self.create_client(SetTargetColor, 'set_target_color'),
            'set_relay': self.create_client(SetBool, 'set_relay'),
            'emergency_stop': self.create_client(Trigger, 'emergency_stop'),
            'nav_emergency_stop': self.create_client(Trigger, 'navigator/emergency_stop'),
        }
        self.create_timer(0.02, self.poll)
        self.create_timer(self.period, self.broadcast)
        names = ', '.join(link.name for link in self.links.links) or 'NONE'
        self.get_logger().info(f'telemetry_bridge: boat {self.boat}, links {names}, '
                               f'telemetry every {self.period:.1f} s on each')

    # ------------------------------------------------------------------ boat state
    def on_status(self, m):
        self.state.update(task=m.task_state, manual=m.manual, target_lat=m.target_latitude,
                          target_lon=m.target_longitude, target_dist=m.target_distance_m,
                          advised_course=m.advised_course_deg, angle_error=m.angle_error_deg,
                          control_error=m.control_error_deg, target_color=m.target_color,
                          target_color_source=m.target_color_source)

    def on_pwm(self, m):
        self.state['pwm'] = {'rear_left': m.rear_left, 'rear_right': m.rear_right,
                             'front_left': m.front_left, 'front_right': m.front_right,
                             'steer': m.steer}

    def on_fix(self, m):
        self.state.update(lat=m.latitude, lon=m.longitude)

    def on_value(self, key):
        def store(m):
            self.state[key] = m.data
        return store

    def on_waypoints(self, m):
        self.waypoints = {w.index: (w.latitude, w.longitude) for w in m.waypoints}

    # ------------------------------------------------------------------ commands
    def poll(self):
        for link, obj in self.links.read():
            if 'cmd' not in obj:
                continue              # another boat's telemetry, the drone, an echo
            target = obj.get('target_id')
            if target is not None and int(target) != self.boat:
                continue              # the radio is shared: not for us
            cq = obj.get('cq')
            if cq is not None and cq == self.in_flight:
                continue              # its copy from the other link, still being done
            if not self.filter.is_new(cq):
                continue              # done already (the ack is on its way)
            self.do(obj, cq, link)

    def do(self, obj, cq, link):
        cmd = obj['cmd']
        calls = []
        if cmd == 'set_manual':
            calls.append(('set_manual', SetBool.Request(data=bool(obj.get('value')))))
        elif cmd == 'set_task':
            calls.append(('set_task', SetTask.Request(task=str(obj.get('task_name', '')))))
        elif cmd == 'set_gps':
            calls.append(('set_gps', SetWaypoint.Request(
                index=int(obj.get('index', 0)), latitude=float(obj.get('lat', 0.0)),
                longitude=float(obj.get('lon', 0.0)))))
        elif cmd == 'set_target_color':
            calls.append(('set_target_color',
                          SetTargetColor.Request(color=str(obj.get('color', '')))))
        elif cmd == 'set_relay':
            calls.append(('set_relay', SetBool.Request(data=bool(obj.get('value')))))
        elif cmd == 'emergency_stop':
            calls += [('emergency_stop', Trigger.Request()),
                      ('nav_emergency_stop', Trigger.Request())]
        elif cmd == 'task':
            try:
                fields = gcs_link.task_request(obj)
            except ValueError as e:
                self.get_logger().warn(f'GCS ({link}): task refused: {e}')
                self.finish(cq)       # acked: resending the same bad command helps nobody
                return
            msg = TaskRequest(**fields)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.task_pub.publish(msg)
            self.get_logger().info(
                f'GCS ({link}): task {msg.task}{" URGENT" if msg.urgent else ""} from '
                f'{msg.source} {dict(zip(msg.keys, msg.values))} cq {cq}')
            self.finish(cq)
            return
        self.get_logger().info(f'GCS ({link}): {cmd} {obj.get("value", "")} cq {cq}')
        if not calls:
            self.finish(cq)           # nothing to call: done
            return
        ready = [(name, req) for name, req in calls if self.srv_clients[name].service_is_ready()]
        if not ready:
            self.get_logger().warn(f'{cmd}: no service yet; the GCS will resend',
                                   throttle_duration_sec=2.0)
            return
        self.in_flight = cq
        left = {'n': len(ready)}

        def answered(future, name):
            left['n'] -= 1
            try:
                res = future.result()
                if not getattr(res, 'success', True):
                    self.get_logger().warn(f'{name}: {getattr(res, "message", "")}')
            except Exception as e:     # noqa: B902 (a failed call must not stop the bridge)
                self.get_logger().error(f'{name}: {e}')
            if left['n'] == 0:
                self.finish(cq)

        for name, req in ready:
            self.srv_clients[name].call_async(req).add_done_callback(
                lambda f, n=name: answered(f, n))

    def finish(self, cq):
        self.filter.done(cq)
        self.in_flight = None
        if self.filter.last != self.acked:
            self.broadcast()          # fast ack

    # ------------------------------------------------------------------ telemetry
    def broadcast(self):
        now = time.monotonic()
        packet = None
        if self.waypoints and (self.waypoints != self.wp_sent
                               or now - self.wp_sent_t >= self.wp_refresh):
            index = sorted(self.waypoints)[self.wp_cycle % len(self.waypoints)]
            lat, lon = self.waypoints[index]
            packet = {'i': self.boat, 'q': self.q, 'w': index, 'wa': round(lat, 6),
                      'wo': round(lon, 6)}
            self.wp_cycle += 1
            if self.wp_cycle % len(self.waypoints) == 0:
                self.wp_sent, self.wp_sent_t = dict(self.waypoints), now
        if packet is None or self.filter.last != self.acked:
            try:
                packet = gcs_link.telemetry(self.state, self.boat, self.q, self.filter.last)
            except (TypeError, ValueError) as e:
                # A bad value must not take the link down (the boat would go deaf to the
                # GCS, emergency stop included): skip this packet, say so
                self.get_logger().error(f'telemetry packet: {e}', throttle_duration_sec=5.0)
                return
            self.acked = self.filter.last
        self.links.send(packet)
        self.q = (self.q + 1) % gcs_link.SEQ_MOD

    def destroy_node(self):
        self.links.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
