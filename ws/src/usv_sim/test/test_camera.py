"""Unit tests for the simulated ZED 2i camera and YOLO (no ROS needed).

Run: cd /ws/src/usv_sim && python3 -m pytest test/test_camera.py -v
"""

import math

import pytest

from usv_core.detection import (
    bearing_deg, depth_sample_pixel, distance_is_valid, median_depth)
from usv_sim.camera import Buoy, CameraModel, Frame, Settings
from usv_sim.sensors import make_rng

CAM = CameraModel()
EXACT = Settings(detect_prob=1.0, full_detection_px=1.0, box_jitter_px=0.0,
                 class_confusion=0.0, depth_error_coeff=0.0, depth_pixel_noise=0.0,
                 depth_nan_prob=0.0)


def yaw_quat(yaw_deg):
    a = math.radians(yaw_deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


NORTH = yaw_quat(90.0)          # ENU yaw 90 deg = facing north


def frame(buoys, q=NORTH, pos=(0.0, 0.0, 0.0), settings=EXACT, seed=1):
    return Frame(buoys, pos, q, CAM, settings, make_rng(seed))


def test_buoy_dead_ahead_is_centred_and_measured_to_its_front():
    # 10 m north of the boat, on the boat's axis; the ZED left lens is 6 cm left of it
    f = frame([Buoy(0.0, 10.0, 0.225, 0.75, 0)])
    ((x1, y1, x2, y2, cid),) = f.boxes()
    assert cid == 0
    u = (x1 + x2) / 2
    assert u > CAM.cx                          # seen slightly right of centre (left lens)
    assert u - CAM.cx == pytest.approx(CAM.fx * 0.06 / (10.0 - 0.29), rel=0.05)
    cx, cy = depth_sample_pixel(x1, y1, x2, y2, CAM.height)
    d = median_depth(f.depth_at, cx, cy, CAM.width, CAM.height)
    assert d == pytest.approx(10.0 - 0.29 - 0.225, abs=0.02)


def test_full_pipeline_puts_the_buoy_back_where_it_is():
    # The boat's own code turns the simulated box + depth into a bearing and a distance
    f = frame([Buoy(3.0, 8.0, 0.225, 0.4, 3)])
    ((x1, y1, x2, y2, cid),) = f.boxes()
    cx, cy = depth_sample_pixel(x1, y1, x2, y2, CAM.height)
    d = median_depth(f.depth_at, cx, cy, CAM.width, CAM.height)
    assert distance_is_valid(d)
    true_bearing = math.degrees(math.atan2(3.0, 8.0 - 0.29))
    assert bearing_deg(cx, CAM.cx, CAM.fx, 0.0) == pytest.approx(true_bearing, abs=1.5)


def test_right_of_the_bow_is_right_in_the_image():
    f = frame([Buoy(2.0, 6.0, 0.225, 0.4, 3), Buoy(-2.0, 6.0, 0.225, 0.4, 3)])
    boxes = sorted(f.boxes())
    assert (boxes[0][0] + boxes[0][2]) / 2 < CAM.cx < (boxes[1][0] + boxes[1][2]) / 2


def test_behind_far_and_outside_the_view_are_not_seen():
    buoys = [Buoy(0.0, -5.0, 0.225, 0.4, 3),      # behind
             Buoy(0.0, 30.0, 0.225, 0.4, 3),      # beyond 20 m
             Buoy(8.0, 1.0, 0.225, 0.4, 3)]       # 83 deg right: outside the 110 deg view
    assert frame(buoys).boxes() == []


def test_a_nearer_buoy_hides_the_one_behind_it():
    f = frame([Buoy(0.0, 4.0, 0.25, 0.75, 0), Buoy(0.0, 9.0, 0.225, 0.4, 3)])
    assert [b[4] for b in f.boxes()] == [0]


def test_horizon_and_water_depth():
    f = frame([])
    assert math.isinf(f.depth_at(640, 200))               # above the horizon: sky
    d = f.depth_at(640, 600)                               # below: the water surface
    assert d == pytest.approx(0.18 * CAM.fx / (600 - CAM.cy), rel=0.01)


def test_small_far_boxes_are_missed_more_often():
    settings = Settings(box_jitter_px=0.0, class_confusion=0.0)
    near = sum(len(frame([Buoy(0.0, 4.0, 0.225, 0.4, 3)], settings=settings, seed=s).boxes())
               for s in range(300))
    far = sum(len(frame([Buoy(0.0, 18.0, 0.225, 0.4, 3)], settings=settings, seed=s).boxes())
              for s in range(300))
    assert near > 280 and far < near * 0.8


def test_depth_error_grows_with_distance():
    settings = Settings(detect_prob=1.0, depth_nan_prob=0.0, depth_pixel_noise=0.0)

    def spread(distance):
        values = [frame([Buoy(0.0, distance, 0.225, 0.4, 3)], settings=settings,
                        seed=s).visible[0][1] for s in range(400)]
        mean = sum(values) / len(values)
        return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

    assert spread(3.0) < 0.05
    assert spread(14.0) == pytest.approx(0.0033 * (14.0 - 0.29 - 0.225) ** 2, rel=0.2)
