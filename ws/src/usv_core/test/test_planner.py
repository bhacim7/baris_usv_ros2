"""Port check for usv_core.planner against the frozen legacy copy in test/legacy/."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))

import config as legacy_config  # noqa: E402
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
from usv_core import planner  # noqa: E402
from usv_core.planner import DEFAULT_CONFIG  # noqa: E402
from utils import planner as legacy  # noqa: E402

# Same geometry nav_process uses: 20 m crop radius at 0.10 m/px, map centred on the boat
RES = 0.10
SIZE = (400, 400)
CENTER = (0.0, 0.0)
BUDGET = 10.0   # generous A* time budget so no result depends on CPU speed


def to_px(x, y):
    return (int(SIZE[0] // 2 + x / RES), int(SIZE[1] // 2 - y / RES))


def costmap_with_buoys(buoys_m):
    """Raw costmap as nav_process builds it: 127 background, buoys as filled 0 discs."""
    img = np.full((SIZE[1], SIZE[0]), 127, dtype=np.uint8)
    radius_px = max(1, int(round(0.25 / RES)))   # BUOY_RADIUS_M
    for x, y in buoys_m:
        cv2.circle(img, to_px(x, y), radius_px, 0, -1)
    return img


RING = [(2.0 * np.cos(a), 2.0 * np.sin(a)) for a in np.radians(np.arange(0, 360, 15))]

# name: (buoys, start, goal)
SCENARIOS = {
    'open_water': ([], (0.0, 0.0), (10.0, 0.0)),
    'single_buoy': ([(5.0, 0.0)], (0.0, 0.0), (10.0, 0.0)),
    'wall_with_gap': ([(5.0, y) for y in (-4, -3, -2, -1, 2, 3, 4)], (0.0, 0.0), (10.0, 0.0)),
    'diagonal_field': ([(3.0, 1.0), (4.0, -1.5), (6.0, 0.5), (7.5, -0.5)], (0.0, 0.0), (9.0, 2.0)),
    'start_in_inflation': ([(1.0, 0.0)], (0.5, 0.0), (10.0, 0.0)),
    'goal_in_obstacle': ([(10.0, 0.0)], (0.0, 0.0), (10.0, 0.0)),
    'goal_outside_window': ([(5.0, 1.0)], (0.0, 0.0), (40.0, 5.0)),
    'enclosed': (RING, (0.0, 0.0), (10.0, 0.0)),
}

# Planner field -> legacy config.py name
CONFIG_NAMES = {
    'costmap_res_m_per_px': 'COSTMAP_RES_M_PER_PX',
    'robot_radius_m': 'ROBOT_RADIUS_M',
    'buoy_radius_m': 'BUOY_RADIUS_M',
    'inflation_margin_m': 'INFLATION_MARGIN_M',
    'a_star_time_budget_s': 'A_STAR_TIME_BUDGET_S',
    'corridor_window_back_m': 'CORRIDOR_WINDOW_BACK_M',
    'corridor_window_ahead_m': 'CORRIDOR_WINDOW_AHEAD_M',
    'corridor_bin_m': 'CORRIDOR_BIN_M',
    'corridor_require_both_sides': 'CORRIDOR_REQUIRE_BOTH_SIDES',
    'corridor_margin_m': 'CORRIDOR_MARGIN_M',
    'corridor_penalty': 'CORRIDOR_PENALTY',
    'pure_pursuit_min_lookahead': 'PURE_PURSUIT_MIN_LOOKAHEAD',
    'pure_pursuit_max_lookahead': 'PURE_PURSUIT_MAX_LOOKAHEAD',
    'pure_pursuit_k_speed': 'PURE_PURSUIT_K_SPEED',
    'pure_pursuit_kp': 'PURE_PURSUIT_KP',
    'pure_pursuit_kd': 'PURE_PURSUIT_KD',
}


# --- Settings ------------------------------------------------------------------------

@pytest.mark.parametrize('field,name', CONFIG_NAMES.items())
def test_default_config_matches_legacy_config(field, name):
    assert getattr(DEFAULT_CONFIG, field) == getattr(legacy_config, name)


def test_nominal_dt_unchanged():
    assert planner.NOMINAL_DT == legacy.NOMINAL_DT == 0.02


# --- Port equivalence ----------------------------------------------------------------

@pytest.mark.parametrize('dt', [None, -1.0, 0.0, 0.001, 0.02, 0.04, 0.1, 0.5])
def test_dt_scale_matches_legacy(dt):
    assert planner.dt_scale(dt) == legacy.dt_scale(dt)


@pytest.mark.parametrize('a,b', [((0, 0), (10, 3)), ((5, 5), (5, 5)), ((-3, 8), (12, -4))])
@pytest.mark.parametrize('w', [1.0, 2.5])
def test_heuristic_matches_legacy(a, b, w):
    assert planner.heuristic(a, b, w) == legacy.heuristic(a, b, w)


@pytest.mark.parametrize('name', SCENARIOS)
def test_inflation_matches_legacy(name):
    raw = costmap_with_buoys(SCENARIOS[name][0])
    new_map, new_obs = planner.get_inflated_nav_map(raw)
    old_map, old_obs = legacy.get_inflated_nav_map(raw)
    assert np.array_equal(new_map, old_map)
    assert np.array_equal(new_obs, old_obs)


@pytest.mark.parametrize('name', SCENARIOS)
def test_line_of_sight_matches_legacy(name):
    buoys, start, goal = SCENARIOS[name]
    nav_map, _ = legacy.get_inflated_nav_map(costmap_with_buoys(buoys))
    for a, b in [(start, goal), ((0.0, 0.0), (0.0, 10.0)), ((-5.0, -5.0), (8.0, 3.0)),
                 ((0.0, 0.0), (25.0, 0.0))]:
        assert (planner.check_line_of_sight(a, b, nav_map, CENTER, RES, SIZE)
                == legacy.check_line_of_sight(a, b, nav_map, CENTER, RES, SIZE))


@pytest.mark.parametrize('name', SCENARIOS)
def test_free_cell_and_goal_nudge_match_legacy(name):
    buoys, start, goal = SCENARIOS[name]
    nav_map, _ = legacy.get_inflated_nav_map(costmap_with_buoys(buoys))
    cells = [to_px(*start), to_px(*goal), (200, 200), (5, 5)]
    # Both versions index the map directly, so only in-window cells are valid input
    cells = [c for c in cells if 0 <= c[0] < SIZE[0] and 0 <= c[1] < SIZE[1]]
    for cell in cells:
        assert (planner._find_free_cell(nav_map, cell, 13)
                == legacy._find_free_cell(nav_map, cell, 13))
        assert (planner._nudge_goal_to_free(cell, (200, 200), nav_map)
                == legacy._nudge_goal_to_free(cell, (200, 200), nav_map))


@pytest.mark.parametrize('name', SCENARIOS)
def test_path_plan_matches_legacy(name):
    buoys, start, goal = SCENARIOS[name]
    nav_map, _ = legacy.get_inflated_nav_map(costmap_with_buoys(buoys))
    new = planner.get_path_plan(start, goal, nav_map, CENTER, RES, SIZE, time_budget_s=BUDGET)
    old = legacy.get_path_plan(start, goal, nav_map, CENTER, RES, SIZE, time_budget_s=BUDGET)
    assert new == old


BOUNDARIES = {
    'both_sides': [(x, 3.0) for x in (2, 6, 10, 14)] + [(x, -3.0) for x in (2, 6, 10, 14)],
    'uneven': [(1.0, 2.5), (4.0, 3.5), (9.0, 2.0), (3.0, -3.0), (12.0, -4.0)],
    'one_side_only': [(2.0, 3.0), (6.0, 3.0)],
    'none': [],
}


@pytest.mark.parametrize('bname', BOUNDARIES)
@pytest.mark.parametrize('robot', [(1.0, 0.0), (8.0, 0.5), (-10.0, 0.0)])
def test_corridor_cost_matches_legacy(bname, robot):
    args = ((SIZE[1], SIZE[0]), CENTER, RES, (0.0, 0.0), (20.0, 0.0), BOUNDARIES[bname], robot)
    new = planner.build_corridor_cost(*args)
    old = legacy.build_corridor_cost(*args)
    if old is None:
        assert new is None
    else:
        assert np.array_equal(new, old)


def test_path_plan_with_corridor_matches_legacy():
    nav_map, _ = legacy.get_inflated_nav_map(costmap_with_buoys([(6.0, 0.5), (9.0, -1.0)]))
    args = ((SIZE[1], SIZE[0]), CENTER, RES, (0.0, 0.0), (20.0, 0.0),
            BOUNDARIES['both_sides'], (0.0, 0.0))
    cost = legacy.build_corridor_cost(*args)
    new = planner.get_path_plan((0.0, 0.0), (12.0, 0.0), nav_map, CENTER, RES, SIZE,
                                time_budget_s=BUDGET, cost_map=cost, los_max_cost=3.0)
    old = legacy.get_path_plan((0.0, 0.0), (12.0, 0.0), nav_map, CENTER, RES, SIZE,
                               time_budget_s=BUDGET, cost_map=cost, los_max_cost=3.0)
    assert new == old


PATH = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (10.0, 5.0)]
POSES = [(0.0, 0.0, 0.0), (2.0, 1.0, 0.3), (5.0, 2.0, 1.2), (7.0, 5.0, 1.57),
         (10.0, 5.0, 0.0), (-3.0, -3.0, 2.0)]


@pytest.mark.parametrize('pose', POSES)
@pytest.mark.parametrize('lookahead', [0.0, 0.8, 2.5, 30.0])
@pytest.mark.parametrize('start_idx', [0, 1, 2, 5])
def test_lookahead_matches_legacy(pose, lookahead, start_idx):
    x, y, _ = pose
    assert (planner.find_lookahead_point(x, y, PATH, lookahead, start_idx)
            == legacy.find_lookahead_point(x, y, PATH, lookahead, start_idx))


@pytest.mark.parametrize('pose', POSES)
@pytest.mark.parametrize('speed', [0.0, 1.0, 3.0])
@pytest.mark.parametrize('start_idx', [0, 2])
@pytest.mark.parametrize('path', [PATH, [], [(4.0, 4.0)]])
def test_pure_pursuit_matches_legacy(pose, speed, start_idx, path):
    x, y, yaw = pose
    kwargs = {'current_speed': speed, 'base_speed': 1680, 'prev_error': 5.0, 'dt': 0.04,
              'start_idx': start_idx}
    assert (planner.pure_pursuit_control(x, y, yaw, path, **kwargs)
            == legacy.pure_pursuit_control(x, y, yaw, path, **kwargs))


# --- Known behaviour ------------------------------------------------------------------

def test_dt_scale_at_25_hz_is_two():
    # The loop runs at 25 Hz but NOMINAL_DT is 50 Hz: D and I terms are scaled by 2.
    assert planner.dt_scale(0.04) == 2.0


def test_inflation_square_size():
    # One obstacle pixel grows to a square of side 2 * int((0.4 + 0.5) / 0.1) + 1 = 19 px
    raw = np.full((SIZE[1], SIZE[0]), 127, dtype=np.uint8)
    raw[200, 200] = 0
    nav_map, _ = planner.get_inflated_nav_map(raw)
    assert int((nav_map == 0).sum()) == 19 * 19


def test_open_water_path_is_straight():
    nav_map, _ = planner.get_inflated_nav_map(costmap_with_buoys([]))
    path = planner.get_path_plan((0.0, 0.0), (10.0, 0.0), nav_map, CENTER, RES, SIZE,
                                 time_budget_s=BUDGET)
    assert path is not None and len(path) == 2


def test_buoy_forces_a_detour():
    nav_map, _ = planner.get_inflated_nav_map(costmap_with_buoys([(5.0, 0.0)]))
    path = planner.get_path_plan((0.0, 0.0), (10.0, 0.0), nav_map, CENTER, RES, SIZE,
                                 time_budget_s=BUDGET)
    assert path is not None and len(path) > 2


def test_enclosed_start_has_no_path():
    nav_map, _ = planner.get_inflated_nav_map(costmap_with_buoys(RING))
    assert planner.get_path_plan((0.0, 0.0), (10.0, 0.0), nav_map, CENTER, RES, SIZE,
                                 time_budget_s=BUDGET) is None


def test_corridor_needs_both_sides():
    args = ((SIZE[1], SIZE[0]), CENTER, RES, (0.0, 0.0), (20.0, 0.0),
            BOUNDARIES['one_side_only'], (1.0, 0.0))
    assert planner.build_corridor_cost(*args) is None


def test_pure_pursuit_sign_target_right_is_positive():
    # Heading North (yaw 0); a path going East is to the right
    _, correction, _, err, _ = planner.pure_pursuit_control(
        0.0, 0.0, 0.0, [(0.0, 0.0), (0.0, 10.0)], current_speed=1.0, prev_error=0.0, dt=0.04)
    assert err == pytest.approx(90.0)
    assert correction > 0
