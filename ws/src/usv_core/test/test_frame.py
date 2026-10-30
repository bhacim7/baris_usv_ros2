"""Port check for detection.process_frame / memory_snapshot against the legacy camera loop.

legacy_frame() below is camera_process.py lines 534-605 and 648-675 (commit 5df3f5b) with
only these changes: inputs are arguments instead of camera/YOLO objects, the HUD drawing and
the task-context list (which never reaches the memory) are left out.
"""

import math
import os
import random
import sys

import numpy as np
import pytest

from usv_core.detection import intrinsics_from_hfov, memory_snapshot, process_frame
from usv_core.object_memory import ObjectMemoryManager

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))
import object_memory_extract as legacy_memory  # noqa: E402
from utils.navigasyon import calculate_obj_gps  # noqa: E402

WIDTH, HEIGHT = 1280, 720
CAM_FX, CAM_CX = intrinsics_from_hfov(WIDTH, 110.0)


class ProtoEnum:
    OBJECT_UNKNOWN = 0
    OBJECT_BUOY = 3
    COLOR_UNKNOWN = 0
    COLOR_YELLOW = 1
    COLOR_BLACK = 2
    COLOR_RED = 3
    COLOR_GREEN = 4
    COLOR_ORANGE = 5


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def time(self):
        return self.t


def legacy_frame(coords, cids, depth_at, magnetic_heading, ida_enlem, ida_boylam,
                 obj_manager, width=WIDTH, height=HEIGHT, cam_cx=CAM_CX, cam_fx=CAM_FX):
    detections = len(cids) > 0
    if detections and magnetic_heading is not None and ida_enlem != 0:
        for i, cid in enumerate(cids):
            r_type = ProtoEnum.OBJECT_UNKNOWN
            r_color = ProtoEnum.COLOR_UNKNOWN
            if cid in [0, 1, 2, 3, 4]:
                r_type = ProtoEnum.OBJECT_BUOY
            else:
                continue
            if cid == 0:
                r_color = ProtoEnum.COLOR_RED
            elif cid == 1:
                r_color = ProtoEnum.COLOR_YELLOW
            elif cid == 2:
                r_color = ProtoEnum.COLOR_BLACK
            elif cid == 3:
                r_color = ProtoEnum.COLOR_ORANGE
            elif cid == 4:
                r_color = ProtoEnum.COLOR_GREEN
            x1, y1, x2, y2 = map(int, coords[i])
            cx = int((x1 + x2) / 2)
            box_h = y2 - y1
            target_cy = int(y2 - (box_h * 0.15))
            cy = max(0, min(target_cy, height - 1))
            roi_size = 2
            depth_values = []
            for dy in range(-roi_size, roi_size + 1):
                for dx in range(-roi_size, roi_size + 1):
                    sample_x = max(0, min(cx + dx, width - 1))
                    sample_y = max(0, min(cy + dy, height - 1))
                    d_val = depth_at(sample_x, sample_y)
                    if not np.isnan(d_val) and not np.isinf(d_val) and d_val > 0.1:
                        depth_values.append(d_val)
            if depth_values:
                dist_m = float(np.median(depth_values))
            else:
                dist_m = float('inf')
            if not np.isnan(dist_m) and not np.isinf(dist_m) and 0.5 < dist_m < 15.0:
                angle_offset_deg = math.degrees(math.atan((cx - cam_cx) / cam_fx))
                obj_bearing = (magnetic_heading + angle_offset_deg) % 360
                obj_lat, obj_lon = calculate_obj_gps(ida_enlem, ida_boylam, dist_m, obj_bearing)
                obj_manager.update_and_get_id(obj_lat, obj_lon, r_type, r_color, cid=cid,
                                              cx=cx, cy=cy, obj_dist=dist_m,
                                              area=(x2 - x1) * (y2 - y1))
    memory_objects = []
    for obj in obj_manager.memory:
        memory_objects.append({
            "type": obj.get('type'),
            "color": obj.get('color'),
            "lat": obj.get('lat'),
            "lon": obj.get('lon'),
            "id": obj.get('id'),
            "cid": obj.get('cid'),
            "cx": obj.get('cx'),
            "cy": obj.get('cy'),
            "dist": obj.get('dist', 10.0),
            "area": obj.get('area', 0),
            "last_seen": obj.get('last_seen', 0.0),
            "seen": obj.get('seen', 1),
            "pos_ok": bool(obj.get('pos_ok', True)),
        })
    return memory_objects


def depth_image(rng):
    """Mostly sensible depths around a few buoys, with holes, inf and junk."""
    base = rng.uniform(1.0, 16.0)
    cache = {}

    def depth_at(x, y):
        if (x, y) not in cache:
            r = rng.random()
            if r < 0.1:
                cache[(x, y)] = float('nan')
            elif r < 0.13:
                cache[(x, y)] = float('inf')
            elif r < 0.16:
                cache[(x, y)] = 0.05
            else:
                cache[(x, y)] = base + rng.gauss(0.0, 0.3)
        return cache[(x, y)]
    return depth_at


def frames(seed, n=250):
    """A run past a buoy field: drifting boat, a few boxes per frame, odd frames mixed in."""
    rng = random.Random(seed)
    lat, lon, heading, t = 40.7240380, 29.8251020, 146.0, 1000.0
    out = []
    for i in range(n):
        t += rng.choice([1 / 30, 1 / 30, 1 / 15, 0.5])
        if i == n // 2:
            t += 25.0                                  # long gap: every track expires
        lat += rng.gauss(-4e-6, 1e-6)
        lon += rng.gauss(3e-6, 1e-6)
        heading = (heading + rng.gauss(0.0, 2.0)) % 360
        boxes = []
        for _ in range(rng.randint(0, 4)):
            x1 = rng.uniform(-10, WIDTH)
            y1 = rng.uniform(300, HEIGHT)
            boxes.append((x1, y1, x1 + rng.uniform(1, 200), y1 + rng.uniform(1, 150),
                          rng.choice([0, 1, 2, 3, 4, 3, 3, 5, 7])))
        h = None if rng.random() < 0.03 else heading
        la = 0 if rng.random() < 0.03 else lat
        out.append((t, boxes, depth_image(rng), h, la, lon))
    return out


@pytest.mark.parametrize('seed', range(6))
def test_frames_match_legacy(seed, monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(legacy_memory, 'time', clock)
    old, new = legacy_memory.ObjectMemoryManager(), ObjectMemoryManager()
    for t, boxes, depth_at, heading, la, lo in frames(seed):
        clock.t = t
        expected = legacy_frame([b[:4] for b in boxes], [b[4] for b in boxes], depth_at,
                                heading, la, lo, old)
        process_frame(boxes, depth_at, WIDTH, HEIGHT, CAM_CX, CAM_FX, heading, la, lo, new,
                      now=t)
        got = memory_snapshot(new)
        for e in expected:
            e.pop('ctx', None)
        assert got == expected
    assert new.memory == old.memory


def test_no_heading_or_no_fix_skips_the_frame():
    mem = ObjectMemoryManager()
    box = [(600, 400, 680, 470, 3)]
    process_frame(box, lambda x, y: 5.0, WIDTH, HEIGHT, CAM_CX, CAM_FX, None,
                  40.72, 29.82, mem, now=1.0)
    process_frame(box, lambda x, y: 5.0, WIDTH, HEIGHT, CAM_CX, CAM_FX, 146.0,
                  0, 29.82, mem, now=1.0)
    assert mem.memory == []


def test_one_buoy_ahead_lands_where_it_should():
    mem = ObjectMemoryManager()
    # centred box, 5 m deep, boat heading north: the buoy is 5 m north of the GPS antenna
    process_frame([(620, 400, 660, 470, 0)], lambda x, y: 5.0, WIDTH, HEIGHT, CAM_CX, CAM_FX,
                  0.0, 40.72, 29.82, mem, now=1.0)
    (obj,) = memory_snapshot(mem)
    assert obj['cid'] == 0 and obj['color'] == ProtoEnum.COLOR_RED
    assert (obj['lat'] - 40.72) * 111320 == pytest.approx(5.0, rel=0.01)
    assert obj['lon'] == pytest.approx(29.82, abs=1e-9)
    assert obj['dist'] == 5.0 and obj['area'] == 40 * 70 and obj['pos_ok'] is True
