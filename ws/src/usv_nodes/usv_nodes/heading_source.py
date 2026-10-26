"""heading_source: the heading the navigation steers by.

Replaces the heading selection at the top of the legacy nav loop (nav_process lines
462-471), using usv_core.heading.select_heading:

  /usv/heading/zed  std_msgs/Float32  ZED 2i magnetometer, filtered, declination added
                                      (perception on the boat, fake_perception in simulation)
  /usv/heading/fc   std_msgs/Float32  Orange Cube EKF heading (fc_bridge / sim_bridge)
  /usv/heading      std_msgs/Float32  the selected heading, at the legacy loop rate

Legacy behaviour kept on purpose:
  * before the first ZED sample the ZED heading counts as 0.0 (shared_state default),
    which is published as a real heading. A warning says so every 5 s.
  * the newest value of each source is used however old it is (no staleness check).
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from usv_core.heading import select_heading, SOURCES


class HeadingSource(Node):

    def __init__(self):
        super().__init__('heading_source')
        self.source = self.declare_parameter('source', 'ZED').value
        rate_hz = self.declare_parameter('rate_hz', 50.0).value
        if self.source not in SOURCES:
            self.get_logger().warn(
                f'unknown source {self.source!r}: legacy falls back to the ZED heading')

        self.zed = None
        self.fc = None
        self.create_subscription(Float32, '/usv/heading/zed', self.on_zed, 10)
        self.create_subscription(Float32, '/usv/heading/fc', self.on_fc, 10)
        self.pub = self.create_publisher(Float32, '/usv/heading', 10)
        self.create_timer(1.0 / rate_hz, self.publish)
        self.get_logger().info(f'heading_source: {self.source} at {rate_hz:.0f} Hz')

    def on_zed(self, msg):
        self.zed = msg.data

    def on_fc(self, msg):
        self.fc = msg.data

    def publish(self):
        zed = self.zed
        if zed is None:
            zed = 0.0
            if self.source != 'FC' or self.fc is None:
                self.get_logger().warn(
                    'no ZED heading yet: using 0.0 as the legacy code does',
                    throttle_duration_sec=5.0)
        heading = select_heading(self.source, zed, self.fc)
        self.pub.publish(Float32(data=float(heading)))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
