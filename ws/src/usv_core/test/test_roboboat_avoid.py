"""
A* on a visibility graph round circles and walls (usv_core.roboboat_avoid).

Every path is checked against the obstacles it was planned round: each segment keeps every
clearance (hull centre line to the object), and a gate with walls is crossed between its
buoys.

Run: cd /ws/src/usv_core && python3 -m pytest test/test_roboboat_avoid.py -v
"""

import math

import pytest
from usv_core.roboboat_avoid import (Obstacles, plan, plan_through, point_segment_dist,
                                     segment_segment_dist)


def length(path):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def keeps_clear(path, obstacles):
    return all(point_segment_dist(c, a, b) >= r - 1e-9
               for a, b in zip(path, path[1:]) for c, r in obstacles.circles) and \
        all(segment_segment_dist(a, b, w0, w1) >= r - 1e-9
            for a, b in zip(path, path[1:]) for w0, w1, r in obstacles.walls)


def test_distances():
    assert point_segment_dist((0.0, 1.0), (-1.0, 0.0), (1.0, 0.0)) == pytest.approx(1.0)
    assert point_segment_dist((3.0, 0.0), (-1.0, 0.0), (1.0, 0.0)) == pytest.approx(2.0)
    assert segment_segment_dist((0, -1), (0, 1), (-1, 0), (1, 0)) == 0.0        # crossing
    assert segment_segment_dist((0, 0), (2, 0), (0, 1), (2, 1)) == pytest.approx(1.0)


def test_straight_when_nothing_is_in_the_way():
    obs = Obstacles([((3.0, 5.0), 0.84)])
    assert plan((0.0, 0.0), (0.0, 10.0), obs) == [(0.0, 0.0), (0.0, 10.0)]


def test_goes_round_a_buoy_on_the_line():
    obs = Obstacles([((0.0, 5.0), 0.84)])
    path = plan((0.0, 0.0), (0.0, 10.0), obs)
    assert path is not None and len(path) > 2 and keeps_clear(path, obs)
    assert length(path) < 10.6                    # round it, not far out


def test_walls_force_the_way_between_the_buoys():
    red, green = (-1.25, 5.0), (1.25, 5.0)
    obs = Obstacles([(red, 0.64), (green, 0.64)],
                    [(red, (-4.25, 5.0), 0.54), (green, (4.25, 5.0), 0.54)])
    path = plan((2.0, 0.0), (2.0, 10.0), obs)     # straight up would pass outside green
    assert path is not None and keeps_clear(path, obs)
    xs = [a[0] + (b[0] - a[0]) * (5.0 - a[1]) / (b[1] - a[1])
          for a, b in zip(path, path[1:]) if a[1] < 5.0 <= b[1]]
    assert len(xs) == 1 and abs(xs[0]) <= 1.25 - 0.64 + 1e-9


def test_none_when_the_way_is_shut():
    box = [(-2.0, 8.0), (2.0, 8.0), (2.0, 12.0), (-2.0, 12.0)]    # walls all round the goal
    obs = Obstacles([], [(a, b, 0.54) for a, b in zip(box, box[1:] + box[:1])])
    assert plan((0.0, 0.0), (0.0, 10.0), obs) is None


def test_gets_out_when_already_too_close():
    obs = Obstacles([((0.0, 0.5), 0.84)])         # the boat at (0, 0), 0.5 m from a buoy
    path = plan((0.0, 0.0), (0.0, 10.0), obs)
    assert path is not None and len(path) > 2


def test_goal_on_a_buoy_is_moved_off_it():
    obs = Obstacles([((0.0, 10.0), 0.84)])
    path = plan((0.0, 0.0), (0.0, 10.2), obs)
    assert path is not None and math.hypot(path[-1][0], path[-1][1] - 10.0) >= 0.84


def test_through_points_in_order():
    obs = Obstacles([((0.0, 5.0), 0.84)])
    path = plan_through([(0.0, 0.0), (0.0, 3.0), (0.0, 8.0), (0.0, 12.0)], obs)
    assert path is not None and keeps_clear(path, obs)
    assert path[0] == (0.0, 0.0) and path[-1] == (0.0, 12.0)
    assert (0.0, 3.0) in path and (0.0, 8.0) in path
