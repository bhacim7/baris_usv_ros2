"""The ZED 2i's eye and YOLO, for the simulation (ROS-free, unit tested).

Turns the known buoys (course file) and the boat's exact pose into what perception gets
from the hardware on the boat: YOLO boxes and a depth image. Everything after that (depth
window, bearing, position, object memory) is the boat's own code, usv_core.detection.

Frames: world ENU (x east, y north, z up, water at z = 0); body FLU (x forward, y left,
z up); the camera looks along body x, so pixel u grows to the right (-y) and v downwards
(-z), and the ZED depth is the distance along the optical axis (body x).

What is modelled, all numbers in config/fake_perception.yaml:
  * pinhole HD720 camera at the left lens of the ZED (it reports the left image)
  * buoys as upright cylinders from the water line to their height; a box is the
    projection of their outline, clipped to the image
  * not seen: behind the camera, beyond max range, too few pixels, mostly hidden behind a
    nearer buoy, or missed by YOLO (more often when small)
  * YOLO box edges jitter; now and then the class is wrong
  * depth: the buoy's front surface, with an error growing with distance squared (ZED 2i
    datasheet: <1 % at 3 m, <5 % at 15 m), per-pixel noise and reflection holes (NaN);
    pixels off the buoy see the water surface, or nothing above the horizon (inf)
"""

import math
from dataclasses import dataclass

from usv_sim.zed import rotate

CLASS_IDS = (0, 1, 2, 3, 4)


@dataclass
class Buoy:
    x: float              # ENU, m
    y: float
    radius: float
    height: float         # above the water
    class_id: int


@dataclass
class CameraModel:
    width: int = 1280
    height: int = 720
    hfov_deg: float = 110.0
    mount: tuple = (0.29, 0.06, 0.18)     # left lens, body frame

    @property
    def fx(self):
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def cx(self):
        return self.width / 2.0

    @property
    def cy(self):
        return self.height / 2.0


def conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def camera_position(boat_pos, q, cam):
    off = rotate(q, cam.mount)
    return tuple(b + o for b, o in zip(boat_pos, off))


def to_camera(point_enu, cam_pos, q):
    """World point -> camera frame (FLU axes), metres."""
    rel = tuple(p - c for p, c in zip(point_enu, cam_pos))
    return rotate(conj(q), rel)


def project(p_cam, cam):
    """Camera-frame point -> pixel (u, v); None when not in front of the camera."""
    x, y, z = p_cam
    if x <= 0.05:
        return None
    return cam.cx - cam.fx * y / x, cam.cy - cam.fx * z / x


def buoy_view(buoy, cam_pos, q, cam, ring=12):
    """(box, front_depth) of a buoy, box = (x1, y1, x2, y2) unclipped; None if behind."""
    us, vs, depths = [], [], []
    for k in range(ring):
        a = 2.0 * math.pi * k / ring
        px = buoy.x + buoy.radius * math.cos(a)
        py = buoy.y + buoy.radius * math.sin(a)
        for z in (0.0, buoy.height):
            p_cam = to_camera((px, py, z), cam_pos, q)
            uv = project(p_cam, cam)
            if uv is None:
                return None
            us.append(uv[0])
            vs.append(uv[1])
            depths.append(p_cam[0])
    return (min(us), min(vs), max(us), max(vs)), min(depths)


def clip_box(box, cam):
    x1, y1, x2, y2 = box
    return (max(0.0, x1), max(0.0, y1), min(cam.width - 1.0, x2), min(cam.height - 1.0, y2))


def box_area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def overlap(a, b):
    return box_area((max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])))


@dataclass
class Settings:
    max_range_m: float = 20.0
    min_box_px: float = 4.0
    full_detection_px: float = 25.0       # boxes at least this tall are found at detect_prob
    detect_prob: float = 0.98
    box_jitter_px: float = 2.0
    class_confusion: float = 0.02
    occlusion_fraction: float = 0.6       # hidden when this share is behind a nearer buoy
    depth_error_coeff: float = 0.0033     # sigma = coeff * depth^2 per buoy per frame
    depth_pixel_noise: float = 0.005      # sigma = this * depth per pixel
    depth_nan_prob: float = 0.10


class Frame:
    """One simulated camera frame: YOLO boxes plus a depth image to sample."""

    def __init__(self, buoys, boat_pos, q, cam, settings, rng):
        self.cam, self.settings, self.rng = cam, settings, rng
        self.q = q
        self.cam_pos = camera_position(boat_pos, q, cam)
        self.visible = []      # (clipped box, front depth, class_id), nearest first
        candidates = []
        for b in buoys:
            if math.hypot(b.x - self.cam_pos[0], b.y - self.cam_pos[1]) > settings.max_range_m:
                continue
            view = buoy_view(b, self.cam_pos, q, cam)
            if view is None:
                continue
            box, depth = view
            clipped = clip_box(box, cam)
            if clipped[2] - clipped[0] < settings.min_box_px or \
                    clipped[3] - clipped[1] < settings.min_box_px:
                continue
            candidates.append((depth, clipped, b.class_id))
        candidates.sort(key=lambda c: c[0])
        for depth, box, cid in candidates:
            hidden = any(overlap(box, near) >= settings.occlusion_fraction * box_area(box)
                         for near, _, _ in self.visible)
            if not hidden:
                # One depth error per buoy and frame: stereo errors are spatially correlated
                err = rng.gauss(0.0, settings.depth_error_coeff * depth * depth)
                self.visible.append((box, depth + err, cid))
        self.depth_cache = {}

    def boxes(self):
        """What YOLO reports: (x1, y1, x2, y2, class_id), with misses, jitter, confusions."""
        s, rng = self.settings, self.rng
        out = []
        for (x1, y1, x2, y2), _, cid in self.visible:
            p = s.detect_prob * min(1.0, (y2 - y1) / s.full_detection_px)
            if rng.random() >= p:
                continue
            j = [rng.gauss(0.0, s.box_jitter_px) for _ in range(4)]
            box = clip_box((x1 + j[0], y1 + j[1], x2 + j[2], y2 + j[3]), self.cam)
            if box[2] <= box[0] or box[3] <= box[1]:
                continue
            if rng.random() < s.class_confusion:
                cid = rng.choice([c for c in CLASS_IDS if c != cid])
            out.append((*box, cid))
        return out

    def depth_at(self, u, v):
        """ZED depth (m) at a pixel, like depth.get_value(u, v)."""
        key = (u, v)
        if key not in self.depth_cache:
            self.depth_cache[key] = self._depth(u, v)
        return self.depth_cache[key]

    def _depth(self, u, v):
        s, rng = self.settings, self.rng
        if rng.random() < s.depth_nan_prob:
            return float('nan')
        for (x1, y1, x2, y2), depth, _ in self.visible:     # nearest buoy first
            if x1 <= u <= x2 and y1 <= v <= y2:
                return max(0.0, depth + rng.gauss(0.0, s.depth_pixel_noise * depth))
        # Off every buoy: the ray meets the water, or the sky
        cam = self.cam
        ray_body = (1.0, -(u - cam.cx) / cam.fx, -(v - cam.cy) / cam.fx)
        ray = rotate(self.q, ray_body)
        if ray[2] >= -1e-6 or self.cam_pos[2] <= 0:
            return float('inf')
        t = self.cam_pos[2] / -ray[2]           # ray_body has x = 1, so t is the depth
        return t + rng.gauss(0.0, s.depth_pixel_noise * t)
