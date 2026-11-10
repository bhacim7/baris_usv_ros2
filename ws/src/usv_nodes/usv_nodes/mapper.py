"""
mapper: the world map, from power-on (usv_core.world_map).

  in   /usv/detections     usv_interfaces/DetectionArray  every camera frame's measured boxes
       /usv/fix            sensor_msgs/NavSatFix          map origin (first fix), boat marker
       /usv/heading        std_msgs/Float32               boat marker
       /usv/imu            sensor_msgs/Imu                the ZED's gyro: yaw rate for the
                                                          turning clean-up (D4); none = 0
  out  /usv/map            usv_interfaces/LandmarkArray   every landmark, publish_hz
       /usv/map/markers    visualization_msgs/MarkerArray the same for RViz, frame usv_map
                           (east/north metres from the map origin), plus the boat and its
                           track

It runs whether the boat is in MANUAL or AUTO: what the boat sees while it is driven to the
start is already on the map. On shutdown the map is written to map_file (JSON) if set.

Times are the detections' own stamps (the frame time), so a replayed bag maps the same way.

Topic and service names are relative to the boat's namespace (sim.launch.py
boat:=azure|marine): the /usv/... names above are /azure/... on Azure, /marine/... on Marine.
"""

import json
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import ColorRGBA, Float32
from usv_core.world_map import enu, MapConfig, WorldMap
from usv_interfaces.msg import DetectionArray, Landmark, LandmarkArray
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {    # RGB per kind; indicators take their current colour
    'buoy_red': (0.9, 0.1, 0.1), 'buoy_green': (0.1, 0.75, 0.2),
    'pole_red': (0.6, 0.0, 0.0), 'pole_green': (0.0, 0.45, 0.1),
    'buoy_black': (0.15, 0.15, 0.15), 'buoy_yellow': (1.0, 0.85, 0.0),
    'triangle': (1.0, 0.85, 0.0), 'plus': (0.05, 0.05, 0.05),
    'digit_1': (0.9, 0.9, 0.9), 'digit_2': (0.9, 0.9, 0.9), 'digit_3': (0.9, 0.9, 0.9),
    'red': (0.9, 0.1, 0.1), 'green': (0.1, 0.75, 0.2),
    'blue': (0.1, 0.3, 1.0), 'off': (0.25, 0.25, 0.25),      # light beacons
}
FRAME = 'usv_map'


def stamp(seconds):
    sec = int(math.floor(seconds))
    return Time(sec=sec, nanosec=int((seconds - sec) * 1e9))


class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper')
        p = self.declare_parameter
        defaults = MapConfig()
        self.map = WorldMap(MapConfig(
            merge_radius_m=p('merge_radius_m', defaults.merge_radius_m).value,
            max_range_m=p('max_range_m', defaults.max_range_m).value,
            sigma_base_m=p('sigma_base_m', defaults.sigma_base_m).value,
            sigma_per_m=p('sigma_per_m', defaults.sigma_per_m).value,
            confirm_sightings=p('confirm_sightings', defaults.confirm_sightings).value,
            color_window=p('color_window', defaults.color_window).value,
            beacon_window_s=p('beacon_window_s', defaults.beacon_window_s).value,
            heading_lag_s=p('heading_lag_s', defaults.heading_lag_s).value,
            new_landmark_max_turn_dps=p('new_landmark_max_turn_dps',
                                        defaults.new_landmark_max_turn_dps).value,
            confusion_ratio=p('confusion_ratio', defaults.confusion_ratio).value,
            confusion_radius_m=p('confusion_radius_m', defaults.confusion_radius_m).value,
            copy_merge_radius_m=p('copy_merge_radius_m',
                                  defaults.copy_merge_radius_m).value))
        self.map_file = p('map_file', '').value
        publish_hz = p('publish_hz', 2.0).value

        self.fix = None
        self.heading = None
        self.yaw_rate = 0.0           # deg/s, from /usv/imu
        self.track = []
        self.create_subscription(DetectionArray, 'detections', self.on_detections, 10)
        self.create_subscription(NavSatFix, 'fix', self.on_fix, 10)
        self.create_subscription(Float32, 'heading', self.on_heading, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 50)
        self.map_pub = self.create_publisher(LandmarkArray, 'map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'map/markers', 10)
        self.create_timer(1.0 / publish_hz, self.publish)
        self.get_logger().info('mapper: mapping from power-on; origin = first GPS fix')

    # ------------------------------------------------------------------ inputs
    def on_fix(self, msg):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        if self.map.origin is None:
            self.map.set_origin(msg.latitude, msg.longitude)
            self.get_logger().info(
                f'map origin {msg.latitude:.7f}, {msg.longitude:.7f}')
        self.fix = (msg.latitude, msg.longitude)
        e, n = enu(msg.latitude, msg.longitude, *self.map.origin)
        if not self.track or math.hypot(e - self.track[-1][0], n - self.track[-1][1]) > 0.3:
            self.track.append((e, n))

    def on_heading(self, msg):
        self.heading = msg.data

    def on_imu(self, msg):
        self.yaw_rate = math.degrees(msg.angular_velocity.z)

    def on_detections(self, msg):
        if self.map.origin is None:
            return              # no fix yet: nothing could be placed anyway
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.map.add_frame([{'class_name': d.class_name, 'position_ok': d.position_ok,
                             'distance_m': d.distance_m, 'lat': d.latitude,
                             'lon': d.longitude} for d in msg.detections], now, self.yaw_rate)

    # ------------------------------------------------------------------ outputs
    def publish(self):
        if self.map.origin is None:
            return
        snapshot = self.map.snapshot()
        out = LandmarkArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = FRAME
        out.origin_latitude, out.origin_longitude = self.map.origin
        for lm in snapshot:
            out.landmarks.append(Landmark(
                id=lm['id'], kind=lm['kind'], color=lm['color'], latitude=lm['lat'],
                longitude=lm['lon'], east_m=float(lm['east']), north_m=float(lm['north']),
                sigma_m=float(lm['sigma']), sightings=lm['sightings'],
                confirmed=lm['confirmed'], closest_m=float(lm['closest']),
                first_seen=stamp(lm['first_seen']), last_seen=stamp(lm['last_seen'])))
        self.map_pub.publish(out)
        self.marker_pub.publish(self.markers(snapshot, out.header.stamp))

    def markers(self, snapshot, now):
        arr = MarkerArray()
        # Landmarks can go (D7 folds copies together): clear RViz, then draw them all again
        clear = Marker()
        clear.header.frame_id, clear.header.stamp = FRAME, now
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for lm in snapshot:
            rgb = COLORS.get(lm['color'] or lm['kind'], (0.6, 0.6, 0.6))
            alpha = 1.0 if lm['confirmed'] else 0.35
            m = self.marker(now, 'landmarks', lm['id'], Marker.CYLINDER)
            m.pose.position.x, m.pose.position.y = float(lm['east']), float(lm['north'])
            tall = lm['kind'].startswith('pole')
            m.scale.x = m.scale.y = 0.6
            m.scale.z = 1.0 if tall else 0.4
            m.pose.position.z = m.scale.z / 2.0
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
            arr.markers.append(m)
            t = self.marker(now, 'labels', lm['id'], Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y = float(lm['east']), float(lm['north'])
            t.pose.position.z = 1.6
            t.scale.z = 0.5
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=alpha)
            name = lm['kind'] + (f" {lm['color']}" if lm['color'] else '')
            t.text = f"{name} x{lm['sightings']}"
            arr.markers.append(t)
        if self.fix is not None:
            e, n = enu(self.fix[0], self.fix[1], *self.map.origin)
            boat = self.marker(now, 'boat', 0, Marker.ARROW)
            boat.pose.position.x, boat.pose.position.y = e, n
            yaw = math.radians(90.0 - (self.heading or 0.0))
            boat.pose.orientation.z = math.sin(yaw / 2.0)
            boat.pose.orientation.w = math.cos(yaw / 2.0)
            boat.scale.x, boat.scale.y, boat.scale.z = 1.5, 0.5, 0.5
            boat.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0)
            arr.markers.append(boat)
            track = self.marker(now, 'track', 0, Marker.LINE_STRIP)
            track.scale.x = 0.1
            track.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.6)
            track.points = [Point(x=x, y=y, z=0.0) for x, y in self.track[-3000:]]
            arr.markers.append(track)
        return arr

    def marker(self, now, ns, mid, kind):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = now
        m.ns, m.id, m.type, m.action = ns, int(mid), kind, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def save(self):
        if not self.map_file or self.map.origin is None:
            return
        with open(self.map_file, 'w') as f:
            json.dump({'origin': self.map.origin, 'landmarks': self.map.snapshot()}, f,
                      indent=1, default=float)
        self.get_logger().info(f'map saved: {self.map_file} '
                               f'({len(self.map.landmarks)} landmarks)')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        except Exception as e:
            print(f'mapper: could not save the map: {e}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
