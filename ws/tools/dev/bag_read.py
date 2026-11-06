"""
Read a recorded run (rosbag2, MCAP storage, no compression) without ROS.

The bags sim.launch.py records (record:=NAME) are plain MCAP files with CDR messages, so
the few topics the replay needs can be decoded by hand: no rclpy, no rosbag2_py, and it
runs on the host as well as in the container. Only uncompressed chunks are read (the
recorder's default here); a compressed bag stops with an error.

Topics are named /usv/<name> whatever the boat: a run of Azure (/azure/fix) reads as an old
one (/usv/fix), see ../bag_topics.py.

  read(path, topics)      [(log time s, /usv/ topic, raw CDR bytes)] sorted by time
  status(data)            /usv/nav_status task_state
  fix(data)               /usv/fix and /usv/sim/navsat (lat, lon)
  float32(data)           std_msgs/Float32 (/usv/heading, /usv/speed)
  gyro_z(data)            sensor_msgs/Imu angular_velocity.z (rad/s)
  detections(data)        (frame time, [dict]) as usv_nodes.navigator_io.detections_to_dicts
  landmarks(data)         ((origin lat, lon), [dict]) as navigator_io.landmark_to_dict
"""

import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import bag_topics  # noqa: E402


def _records(buf, off, end):
    while off + 9 <= end:
        op = buf[off]
        length = struct.unpack_from('<Q', buf, off + 1)[0]
        yield op, off + 9, length
        off += 9 + length


def _string(buf, off):
    n = struct.unpack_from('<I', buf, off)[0]
    return buf[off + 4:off + 4 + n].decode('utf-8', 'replace'), off + 4 + n


def read(path, topics, boat=None):
    """Return [(log time in s, /usv/ topic, CDR bytes)] of the given topics, by time."""
    with open(path, 'rb') as f:
        buf = f.read()
    relative = {t[len(bag_topics.PREFIX):] for t in topics}
    channels, out = {}, []

    def handle(op, off, length):
        if op == 0x04:                                    # Channel
            channels[struct.unpack_from('<H', buf, off)[0]] = _string(buf, off + 4)[0]
        elif op == 0x05:                                  # Message
            cid, _, log_time, _ = struct.unpack_from('<HIQQ', buf, off)
            name = channels.get(cid, '')
            if name.lstrip('/').partition('/')[2] in relative:
                out.append((log_time * 1e-9, name, buf[off + 22:off + length]))
    for op, off, length in _records(buf, 8, len(buf)):
        if op == 0x06:                                    # Chunk
            compression, p = _string(buf, off + 28)
            if compression:
                raise ValueError(f'{path}: compressed chunks ({compression}) not supported')
            size = struct.unpack_from('<Q', buf, p)[0]
            for record in _records(buf, p + 8, p + 8 + size):
                handle(*record)
        else:
            handle(op, off, length)
    topic_of = bag_topics.names(set(channels.values()), topics, boat)
    out = [(t, topic_of[name], data) for t, name, data in out if name in topic_of]
    out.sort(key=lambda m: m[0])
    return out


class _Cdr:
    """Little-endian CDR, alignment counted from after the 4-byte encapsulation header."""

    def __init__(self, data):
        self.d, self.o = data, 4

    def _align(self, n):
        self.o += (-(self.o - 4)) % n

    def _unpack(self, fmt, size):
        self._align(size)
        v = struct.unpack_from(fmt, self.d, self.o)[0]
        self.o += size
        return v

    def u8(self):
        v = self.d[self.o]
        self.o += 1
        return v

    def u16(self):
        return self._unpack('<H', 2)

    def u32(self):
        return self._unpack('<I', 4)

    def i32(self):
        return self._unpack('<i', 4)

    def f32(self):
        return self._unpack('<f', 4)

    def f64(self):
        return self._unpack('<d', 8)

    def string(self):
        n = self.u32()
        v = self.d[self.o:self.o + n - 1].decode('utf-8', 'replace')
        self.o += n
        return v

    def time(self):
        return self.i32() + self.u32() * 1e-9

    def header(self):
        stamp = self.time()
        self.string()                                     # frame_id
        return stamp


def status(data):
    c = _Cdr(data)
    c.header()
    return c.string()


def fix(data):
    c = _Cdr(data)
    c.header()
    c.u8()                                                # status.status
    c.u16()                                               # status.service
    return c.f64(), c.f64()


def float32(data):
    return _Cdr(data).f32()


def gyro_z(data):
    c = _Cdr(data)
    c.header()
    for _ in range(4 + 9 + 2):                            # orientation, its covariance, x, y
        c.f64()
    return c.f64()


def detections(data):
    """Return (frame time, [dict]) of a DetectionArray, as the navigator node gets it."""
    c = _Cdr(data)
    stamp = c.header()
    width = c.u32()
    c.f64(), c.f64(), c.f32(), c.f32(), c.f32()           # boat lat, lon, heading, origin
    out = []
    for _ in range(c.u32()):
        name = c.string()
        cid = c.u8()
        c.f32()                                           # confidence
        box = tuple(c.i32() for _ in range(4))
        depth, distance, bearing = c.f32(), c.f32(), c.f32()
        ok = bool(c.u8())
        lat, lon = c.f64(), c.f64()
        out.append({'class_name': name, 'class_id': cid, 'box': box, 'depth_m': depth,
                    'distance_m': distance, 'bearing_deg': bearing, 'position_ok': ok,
                    'lat': lat, 'lon': lon,
                    'edge': width > 0 and (box[0] <= 1 or box[2] >= width - 2)})
    return stamp, out


def landmarks(data):
    """Return ((origin lat, lon), [dict]) of a LandmarkArray, as the navigator node gets it."""
    c = _Cdr(data)
    c.header()
    origin = (c.f64(), c.f64())
    out = []
    for _ in range(c.u32()):
        lid = c.u32()
        kind, color = c.string(), c.string()
        lat, lon = c.f64(), c.f64()
        east, north, sigma = c.f32(), c.f32(), c.f32()
        sightings = c.u32()
        confirmed = bool(c.u8())
        closest = c.f32()
        first_seen, last_seen = c.time(), c.time()
        out.append({'id': lid, 'kind': kind, 'color': color, 'lat': lat, 'lon': lon,
                    'east': east, 'north': north, 'sigma': sigma, 'sightings': sightings,
                    'confirmed': confirmed, 'closest': closest, 'first_seen': first_seen,
                    'last_seen': last_seen})
    return origin, out
