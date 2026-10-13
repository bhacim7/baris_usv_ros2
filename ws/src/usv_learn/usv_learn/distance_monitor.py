import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from usv_learn import navigation as nav


class DistanceMonitor(Node):
    def __init__(self):
        super().__init__('distance_monitor')

        # Parameters: name and default value (the role config.py used to play).
        self.declare_parameter('target_lat', 40.8090735)
        self.declare_parameter('target_lon', 29.2622975)
        self.declare_parameter('arrival_radius_m', 3.0)

        self.manual = False
        self.arrived = False
        self.count = 0

        self.subscription = self.create_subscription(NavSatFix, '/usv/fix', self.on_fix, 10)
        self.publisher = self.create_publisher(Float64, '/usv/target_distance', 10)

        # Service: on_set_manual() runs when a /usv/set_manual request arrives.
        self.service = self.create_service(SetBool, '/usv/set_manual', self.on_set_manual)

    def on_set_manual(self, request, response):
        self.manual = request.data
        response.success = True
        response.message = 'manual mode' if self.manual else 'autonomous mode'
        self.get_logger().info(f'Command received: {response.message}')
        return response

    def on_fix(self, msg):
        if self.manual:
            return

        # Read on every message so runtime changes take effect immediately.
        target_lat = self.get_parameter('target_lat').value
        target_lon = self.get_parameter('target_lon').value
        radius = self.get_parameter('arrival_radius_m').value

        distance = nav.haversine(msg.latitude, msg.longitude, target_lat, target_lon)
        bearing = nav.calculate_bearing(msg.latitude, msg.longitude, target_lat, target_lon)

        out = Float64()
        out.data = distance
        self.publisher.publish(out)

        if distance < radius and not self.arrived:
            self.arrived = True
            self.get_logger().info(f'TARGET REACHED ({distance:.1f} m < {radius:.1f} m)')
        elif distance >= radius:
            self.arrived = False

        self.count += 1
        if self.count % 5 == 0:
            self.get_logger().info(f'Target {distance:.1f} m, bearing {bearing:.0f} deg')


def main():
    rclpy.init()
    node = DistanceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
