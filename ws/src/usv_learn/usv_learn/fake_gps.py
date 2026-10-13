import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class FakeGps(Node):
    def __init__(self):
        super().__init__('fake_gps')

        # Publisher: sends NavSatFix messages on /usv/fix (queue depth 10).
        self.publisher = self.create_publisher(NavSatFix, '/usv/fix', 10)

        # Timer: calls publish_fix() every 0.2 s (5 Hz).
        self.timer = self.create_timer(0.2, self.publish_fix)

        # About 64 m south of T1_GATE_ENTER
        self.lat = 40.8085000
        self.lon = 29.2622975
        self.count = 0

    def publish_fix(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = self.lat
        msg.longitude = self.lon
        self.publisher.publish(msg)

        # Log once per second (every 5th message)
        self.count += 1
        if self.count % 5 == 0:
            self.get_logger().info(f'Position: {self.lat:.7f}, {self.lon:.7f}')

        # Move 0.2 m north: 0.2 m / 111139 m per degree = 0.0000018 deg
        self.lat += 0.0000018


def main():
    rclpy.init()
    node = FakeGps()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
