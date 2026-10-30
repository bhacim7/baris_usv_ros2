"""
From a YOLO box and the ZED depth image to a buoy position.

Ported from legacy core/camera_process.py (commit 5df3f5b).

This is everything perception does AFTER the hardware: the boat runs it on real YOLO boxes
and the real ZED depth, the simulation (fake_perception) on boxes and depth it draws from
the known buoy positions. Same code, so both see the same geometry and the same quirks.

  intrinsics_from_hfov   lines 417-420, the fallback when the ZED factory intrinsics are
                         not available (the boat uses left_cam.fx / left_cam.cx)
  depth_sample_pixel     lines 559-564: the box centre column, 15 % above the box bottom
  median_depth           lines 566-580: median of the valid depths in a 5 x 5 window
  distance_is_valid      line 582: 0.5 m < distance < 15 m
  bearing_deg            lines 591-594: heading + angle of the pixel column
  detection_position     line 596: navigation.calculate_obj_gps from the boat's GPS fix
  process_frame          lines 534-605: one frame of boxes through all of the above into
                         the object memory (usv_core.object_memory)
  memory_snapshot        lines 648-675: what is handed to the navigation every frame

The boat's GPS fix is the antenna's position, not the camera's: legacy places every buoy
relative to the antenna, which on this boat is ~0.9 m behind the ZED. Kept.
"""

import math
import statistics

from usv_core.navigation import calculate_obj_gps
from usv_core.object_memory import CID_TO_COLOR

DEPTH_SAMPLE_FROM_BOTTOM = 0.15
ROI_HALF_SIZE = 2            # 5 x 5 window
MIN_VALID_DEPTH_M = 0.1      # samples at or below this are rejected
MIN_DISTANCE_M = 0.5
MAX_DISTANCE_M = 15.0


def intrinsics_from_hfov(width, hfov_deg):
    """(fx, cx) of a pinhole camera with the given horizontal field of view."""
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return fx, width / 2.0


def depth_sample_pixel(x1, y1, x2, y2, height):
    """(cx, cy) where the depth of a box is read: centre column, near the waterline."""
    cx = int((x1 + x2) / 2)
    box_h = y2 - y1
    target_cy = int(y2 - (box_h * DEPTH_SAMPLE_FROM_BOTTOM))
    cy = max(0, min(target_cy, height - 1))
    return cx, cy


def median_depth(depth_at, cx, cy, width, height):
    """
    Median of the valid depths around (cx, cy); inf when none is valid.

    depth_at(x, y) returns the depth in metres at a pixel (NaN / inf allowed), e.g. a
    wrapper around the ZED's depth.get_value or a simulated depth image.
    """
    values = []
    for dy in range(-ROI_HALF_SIZE, ROI_HALF_SIZE + 1):
        for dx in range(-ROI_HALF_SIZE, ROI_HALF_SIZE + 1):
            sample_x = max(0, min(cx + dx, width - 1))
            sample_y = max(0, min(cy + dy, height - 1))
            d = depth_at(sample_x, sample_y)
            if not math.isnan(d) and not math.isinf(d) and d > MIN_VALID_DEPTH_M:
                values.append(d)
    return float(statistics.median(values)) if values else float('inf')


def distance_is_valid(dist_m):
    return not math.isnan(dist_m) and not math.isinf(dist_m) and \
        MIN_DISTANCE_M < dist_m < MAX_DISTANCE_M


def bearing_deg(cx, cam_cx, cam_fx, heading_deg):
    """Compass bearing of a pixel column: right of the image centre = larger bearing."""
    angle_offset_deg = math.degrees(math.atan((cx - cam_cx) / cam_fx))
    return (heading_deg + angle_offset_deg) % 360


def detection_position(boat_lat, boat_lon, dist_m, bearing):
    return calculate_obj_gps(boat_lat, boat_lon, dist_m, bearing)


OBJECT_BUOY = 3              # legacy ProtoEnum.OBJECT_BUOY
BUOY_CLASS_IDS = (0, 1, 2, 3, 4)    # YOLO data.yaml: red, yellow, black, orange, green


def process_frame(boxes, depth_at, width, height, cam_cx, cam_fx, heading_deg, boat_lat,
                  boat_lon, memory, now=None):
    """
    Handle one camera frame, as the legacy camera loop does.

    boxes: [(x1, y1, x2, y2, class_id), ...] in pixels (YOLO's xyxy, any number type).
    depth_at(x, y): depth in metres at a pixel (see median_depth).
    heading_deg: the heading the navigation uses (/usv/heading); None = no heading yet.
    boat_lat, boat_lon: the GPS fix (/usv/fix); a latitude of 0 means no fix yet.
    memory: an ObjectMemoryManager, updated in place. now: its clock (None = time.time()).

    Legacy skips the whole frame without a heading or a fix, and only class ids 0-4 count.
    Note the memory drops expired tracks only inside update_and_get_id, so a frame with no
    usable detection leaves stale tracks in place (kept).
    """
    if not boxes or heading_deg is None or boat_lat == 0:
        return
    for box in boxes:
        cid = box[4]
        if cid not in BUOY_CLASS_IDS:
            continue
        color = CID_TO_COLOR[cid]
        x1, y1, x2, y2 = map(int, box[:4])
        cx, cy = depth_sample_pixel(x1, y1, x2, y2, height)
        dist_m = median_depth(depth_at, cx, cy, width, height)
        if not distance_is_valid(dist_m):
            continue
        bearing = bearing_deg(cx, cam_cx, cam_fx, heading_deg)
        obj_lat, obj_lon = detection_position(boat_lat, boat_lon, dist_m, bearing)
        memory.update_and_get_id(obj_lat, obj_lon, OBJECT_BUOY, color, cid=cid, cx=cx, cy=cy,
                                 obj_dist=dist_m, area=(x2 - x1) * (y2 - y1), now=now)


def memory_snapshot(memory):
    """Every tracked object, with the fields (and defaults) legacy sends to the navigation."""
    return [{
        'id': obj.get('id'),
        'type': obj.get('type'),
        'color': obj.get('color'),
        'cid': obj.get('cid'),
        'lat': obj.get('lat'),
        'lon': obj.get('lon'),
        'cx': obj.get('cx'),
        'cy': obj.get('cy'),
        'dist': obj.get('dist', 10.0),
        'area': obj.get('area', 0),
        'last_seen': obj.get('last_seen', 0.0),
        'seen': obj.get('seen', 1),
        'pos_ok': bool(obj.get('pos_ok', True)),
    } for obj in memory.memory]
