"""
Local path planning and path following, ported from legacy utils/planner.py.

Source: commit 5df3f5b. What changed in the port, and only this:
  * Settings are no longer read from a global `config` module. They come from a
    PlannerConfig passed as the `cfg` argument; its defaults are the legacy config.py
    values, so calling any function without `cfg` behaves exactly like the legacy code.
  * The unused `navigasyon` import was dropped.

Function bodies are otherwise unchanged. test/test_planner.py runs every function next to
the frozen legacy copy in test/legacy/ and requires identical results.

Frame convention (inherited, do not change): world x = North, y = East, in metres.
Grid: column grows with x, row grows with -y.
"""

from dataclasses import dataclass
import heapq
import math
import time

import cv2
import numpy as np

# Nominal nav loop period. The PID/PP derivative and integral terms are scaled by
# (dt / NOMINAL_DT). The legacy loop now runs at 25 Hz (dt = 0.04), so dt_scale() returns
# 2.0 in normal operation. Gains were tuned with that in place: keep 0.02.
NOMINAL_DT = 0.02


@dataclass(frozen=True)
class PlannerConfig:
    """Planner settings. Defaults are the legacy config.py values at commit 5df3f5b."""

    costmap_res_m_per_px: float = 0.10
    robot_radius_m: float = 0.4
    buoy_radius_m: float = 0.25
    inflation_margin_m: float = 0.5
    a_star_time_budget_s: float = 0.12
    corridor_window_back_m: float = 5.0
    corridor_window_ahead_m: float = 15.0
    corridor_bin_m: float = 2.0
    corridor_require_both_sides: bool = True
    corridor_margin_m: float = 0.5
    corridor_penalty: float = 3.0
    pure_pursuit_min_lookahead: float = 0.8
    pure_pursuit_max_lookahead: float = 2.5
    pure_pursuit_k_speed: float = 1.0
    pure_pursuit_kp: float = 2.0
    pure_pursuit_kd: float = 0.8


DEFAULT_CONFIG = PlannerConfig()


def dt_scale(dt):
    if dt is None or dt <= 0.0:
        return 1.0
    return min(5.0, max(0.2, dt / NOMINAL_DT))


# --- UTILITY & HYBRID ---
def get_hybrid_point(robot_x, robot_y, robot_yaw, angle_error_deg, step_dist=2.0):
    target_angle = robot_yaw - math.radians(angle_error_deg)
    tx = robot_x + (step_dist * math.cos(target_angle))
    ty = robot_y + (step_dist * math.sin(target_angle))
    return tx, ty


def get_inflated_nav_map(raw_costmap, cfg=DEFAULT_CONFIG):
    """
    Prepare the map for A* by inflating obstacles to account for robot radius.

    Returns (nav_map, inflated_obstacles): nav_map is 255 where free and 0 where blocked.
    """
    if raw_costmap is None or raw_costmap.size == 0:
        return None, None

    nav_map = raw_costmap.copy()

    # Obstacles are < 100 in the grayscale costmap
    obstacles_mask = (nav_map < 100).astype(np.uint8) * 255

    inflation_m = cfg.inflation_margin_m
    robot_radius = cfg.robot_radius_m
    res = cfg.costmap_res_m_per_px

    kernel_size = (int((robot_radius + inflation_m) / res) * 2) + 1
    inflated_obstacles = cv2.dilate(obstacles_mask, np.ones((kernel_size, kernel_size), np.uint8),
                                    iterations=1)

    nav_map[:] = 255  # Assume clear everywhere
    nav_map[inflated_obstacles > 0] = 0  # Mark inflated obstacles as black

    return nav_map, inflated_obstacles


def _segment_is_clear(nav_map, p1, p2, cost_map=None, max_cost=None):
    """
    Check a pixel segment for obstacles in O(L).

    Samples one pixel per unit of the dominant axis instead of rasterising the line into a
    full-size mask.
    """
    h, w = nav_map.shape[:2]

    x0, y0 = float(p1[0]), float(p1[1])
    x1, y1 = float(p2[0]), float(p2[1])

    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    if n < 2:
        n = 2

    xi = np.rint(np.linspace(x0, x1, n)).astype(np.int32)
    yi = np.rint(np.linspace(y0, y1, n)).astype(np.int32)

    # Leaving the planning window is treated as blocked (we cannot vouch for it).
    if xi.min() < 0 or yi.min() < 0 or xi.max() >= w or yi.max() >= h:
        return False

    if np.any(nav_map[yi, xi] == 0):
        return False

    # A straight-line shortcut must also respect the soft cost layer, otherwise the
    # line-of-sight fast path tunnels straight through the corridor cap that A* itself
    # would have paid to avoid.
    if cost_map is not None and max_cost is not None:
        if np.any(cost_map[yi, xi] > max_cost):
            return False

    return True


def check_line_of_sight(start, end, nav_map, center_m, res, size_px,
                        cost_map=None, max_cost=None):
    """
    Raycast on the grid: is the straight line from start to end obstacle-free.

    If a soft cost layer is supplied, the line must also not cross anything above
    `max_cost`.
    """
    if nav_map is None:
        return True

    def w2p(x, y):
        cw, ch = size_px[0] // 2, size_px[1] // 2
        dx = x - center_m[0]
        dy = y - center_m[1]
        px = int(cw + (dx / res))
        py = int(ch - (dy / res))
        if 0 <= px < size_px[0] and 0 <= py < size_px[1]:
            return px, py
        return None

    p1 = w2p(start[0], start[1])
    p2 = w2p(end[0], end[1])

    if p1 is None or p2 is None:
        return False

    return _segment_is_clear(nav_map, p1, p2, cost_map, max_cost)


# --- A* PATH PLANNING ---
def heuristic(a, b, weight=1.0):
    # Octile distance (Chebyshev variant)
    dx = abs(b[0] - a[0])
    dy = abs(b[1] - a[1])
    return weight * (dx + dy + (1.414 - 2) * min(dx, dy))


def string_pulling(path, nav_map, center_m, res, size_px, cost_map=None, max_cost=None):
    """
    Smooth the jagged A* grid path.

    Draws straight lines between the furthest mutually visible nodes.
    """
    if not path or len(path) <= 2:
        return path

    smoothed_path = [path[0]]
    current_idx = 0

    while current_idx < len(path) - 1:
        furthest_visible = current_idx + 1
        for test_idx in range(current_idx + 2, len(path)):
            if check_line_of_sight(path[current_idx], path[test_idx], nav_map, center_m, res,
                                   size_px, cost_map, max_cost):
                furthest_visible = test_idx
            else:
                break

        smoothed_path.append(path[furthest_visible])
        current_idx = furthest_visible

    return smoothed_path


def build_corridor_cost(shape, center_m, res, axis_p0, axis_p1, boundary_pts, robot_xy,
                        cfg=DEFAULT_CONFIG):
    """
    Soft "do not pass outside a boundary buoy" cost layer for Task 2.

    Returns a float32 array the same shape as the nav map, 0 everywhere the boat is free to
    go and corridor_penalty per cell beyond the boundary. None if the cap cannot be
    established, in which case no constraint is applied at all.

    Lateral offsets are measured against axis_p0 -> axis_p1 (first to last Task 2
    waypoint). Only buoys within a longitudinal window around the boat contribute; the cap
    is built per longitudinal bin; if either side is missing the whole cap is abandoned.
    """
    if not boundary_pts:
        return None

    ax0, ay0 = axis_p0
    ax1, ay1 = axis_p1
    dx, dy = ax1 - ax0, ay1 - ay0
    axis_len = math.hypot(dx, dy)
    if axis_len < 1.0:
        return None
    ux, uy = dx / axis_len, dy / axis_len       # along
    nx, ny = -uy, ux                            # left-positive normal

    def project(px, py):
        rx, ry = px - ax0, py - ay0
        return rx * ux + ry * uy, rx * nx + ry * ny

    boat_s, _ = project(*robot_xy)
    back = cfg.corridor_window_back_m
    ahead = cfg.corridor_window_ahead_m
    s_lo, s_hi = boat_s - back, boat_s + ahead

    bin_m = max(0.5, cfg.corridor_bin_m)
    nbins = max(1, int(math.ceil((s_hi - s_lo) / bin_m)))
    left = [None] * nbins     # most negative lateral allowed (nearest left buoy)
    right = [None] * nbins    # most positive lateral allowed

    for px, py in boundary_pts:
        s, d = project(px, py)
        if not (s_lo <= s <= s_hi):
            continue
        b = min(nbins - 1, max(0, int((s - s_lo) / bin_m)))
        if d >= 0:
            if right[b] is None or d < right[b]:
                right[b] = d
        else:
            if left[b] is None or d > left[b]:
                left[b] = d

    if cfg.corridor_require_both_sides:
        if not any(v is not None for v in left) or not any(v is not None for v in right):
            return None

    def fill(seq):
        out = list(seq)
        last = None
        for i in range(len(out)):
            if out[i] is None:
                out[i] = last
            else:
                last = out[i]
        last = None
        for i in range(len(out) - 1, -1, -1):
            if out[i] is None:
                out[i] = last
            else:
                last = out[i]
        return out

    left = fill(left)
    right = fill(right)
    if any(v is None for v in left) or any(v is None for v in right):
        return None

    h, w = shape[:2]
    cw, ch = w // 2, h // 2
    yy, xx = np.mgrid[0:h, 0:w]
    wx = center_m[0] + (xx - cw) * res
    wy = center_m[1] - (yy - ch) * res
    rx, ry = wx - ax0, wy - ay0
    s_grid = rx * ux + ry * uy
    d_grid = rx * nx + ry * ny

    idx = np.clip(((s_grid - s_lo) / bin_m).astype(np.int32), 0, nbins - 1)
    margin = cfg.corridor_margin_m
    left_arr = np.asarray(left, dtype=np.float32) + margin
    right_arr = np.asarray(right, dtype=np.float32) - margin

    outside = (d_grid < left_arr[idx]) | (d_grid > right_arr[idx])
    # Beyond the longitudinal window we know nothing, so charge nothing.
    outside &= (s_grid >= s_lo) & (s_grid <= s_hi)

    cost = np.zeros((h, w), dtype=np.float32)
    cost[outside] = float(cfg.corridor_penalty)
    return cost


def _find_free_cell(nav_map, cell, max_radius_px):
    """Nearest free cell to `cell`, searched outward in Chebyshev rings. None if none."""
    h, w = nav_map.shape[:2]
    cx, cy = int(cell[0]), int(cell[1])

    if 0 <= cx < w and 0 <= cy < h and nav_map[cy, cx] != 0:
        return (cx, cy)

    for r in range(1, int(max_radius_px) + 1):
        best = None
        best_d2 = None
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue  # only the new ring
                x, y = cx + dx, cy + dy
                if 0 <= x < w and 0 <= y < h and nav_map[y, x] != 0:
                    d2 = dx * dx + dy * dy
                    if best_d2 is None or d2 < best_d2:
                        best_d2 = d2
                        best = (x, y)
        if best is not None:
            return best
    return None


def _nudge_goal_to_free(goal, start, nav_map, max_radius_px=None, cfg=DEFAULT_CONFIG):
    """If the goal cell sits inside an inflated obstacle, move it to the NEAREST free cell."""
    if nav_map[goal[1], goal[0]] != 0:
        return goal

    if max_radius_px is None:
        res = cfg.costmap_res_m_per_px
        max_radius_px = int((cfg.buoy_radius_m +
                             cfg.robot_radius_m +
                             cfg.inflation_margin_m) / res) * 2 + 6

    return _find_free_cell(nav_map, goal, max_radius_px)


def get_path_plan(start_world, end_world, nav_map, center_m, res, size_px,
                  heuristic_weight=2.5, max_expansions=20000, time_budget_s=None,
                  goal_tolerance_px=3, cost_map=None, los_max_cost=None,
                  cfg=DEFAULT_CONFIG):
    """
    Run A* over the local costmap grid; return a smoothed list of world (x, y), or None.

    max_expansions / time_budget_s bound the search. Note the time budget makes the result
    depend on CPU speed when the search is long; tests pass a generous budget.
    """
    if nav_map is None:
        return None

    if time_budget_s is None:
        time_budget_s = cfg.a_star_time_budget_s

    def w2p(x, y):
        cw, ch = size_px[0] // 2, size_px[1] // 2
        px = int(cw + ((x - center_m[0]) / res))
        py = int(ch - ((y - center_m[1]) / res))
        return (px, py)

    def p2w(px, py):
        cw, ch = size_px[0] // 2, size_px[1] // 2
        x = center_m[0] + ((px - cw) * res)
        y = center_m[1] - ((py - ch) * res)
        return (x, y)

    start = w2p(*start_world)
    raw_goal = w2p(*end_world)

    # Clamp the goal into the window so A* never searches for an impossible point.
    goal = (max(0, min(size_px[0] - 1, raw_goal[0])),
            max(0, min(size_px[1] - 1, raw_goal[1])))

    if not (0 <= start[0] < size_px[0] and 0 <= start[1] < size_px[1]):
        return None

    # Boat inside an inflated obstacle: plan from the nearest free cell instead of giving up.
    inflation_px = int((cfg.robot_radius_m +
                        cfg.inflation_margin_m) / res) + 4
    start = _find_free_cell(nav_map, start, inflation_px)
    if start is None:
        return None

    goal = _nudge_goal_to_free(goal, start, nav_map, cfg=cfg)
    if goal is None:
        return None

    tol_sq = goal_tolerance_px * goal_tolerance_px
    deadline = time.time() + time_budget_s

    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal, heuristic_weight)}

    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]

    # Entry format: (f_score, (px, py), (dx_from_parent, dy_from_parent))
    open_set = []
    heapq.heappush(open_set, (f_score[start], start, (0, 0)))

    expansions = 0

    while open_set:
        current_f, current, prev_dir = heapq.heappop(open_set)

        # Skip stale heap entries
        if current_f > f_score.get(current, float('inf')) + 1e-9:
            continue

        dgx = current[0] - goal[0]
        dgy = current[1] - goal[1]
        if (dgx * dgx + dgy * dgy) <= tol_sq:
            path = []
            node = current
            while node in came_from:
                path.append(p2w(node[0], node[1]))
                node = came_from[node]
            path.append(p2w(start[0], start[1]))
            path.reverse()
            return string_pulling(path, nav_map, center_m, res, size_px,
                                  cost_map, los_max_cost)

        expansions += 1
        if expansions >= max_expansions:
            return None
        if (expansions & 0x3FF) == 0 and time.time() > deadline:
            return None

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < size_px[0] and 0 <= neighbor[1] < size_px[1]:
                if nav_map[neighbor[1], neighbor[0]] == 0:
                    continue  # Hit obstacle

                step_cost = 1 if dx == 0 or dy == 0 else 1.414

                # Soft cost layer (Task 2 corridor cap): charged, not forbidden.
                if cost_map is not None:
                    step_cost += float(cost_map[neighbor[1], neighbor[0]])

                # Kinematic penalty for sharp turns (biases the search, not strictly enforced)
                if prev_dir != (0, 0):
                    dot_product = prev_dir[0] * dx + prev_dir[1] * dy
                    mag_prev = math.sqrt(prev_dir[0] ** 2 + prev_dir[1] ** 2)
                    mag_curr = math.sqrt(dx ** 2 + dy ** 2)
                    if mag_prev > 0 and mag_curr > 0:
                        cos_angle = min(1.0, max(-1.0, dot_product / (mag_prev * mag_curr)))
                        angle_diff = math.degrees(math.acos(cos_angle))

                        if angle_diff > 45.0:
                            step_cost += (angle_diff * 0.1)
                        if angle_diff >= 90.0:
                            step_cost += 5.0

                tentative_g_score = g_score[current] + step_cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_n = tentative_g_score + heuristic(neighbor, goal, heuristic_weight)
                    f_score[neighbor] = f_n
                    heapq.heappush(open_set, (f_n, neighbor, (dx, dy)))

    return None


# --- PURE PURSUIT CONTROLLER ---
def _closest_point_on_segment(px, py, ax, ay, bx, by):
    """Perpendicular foot of (px, py) on segment a->b, clamped. Returns (cx, cy, d2)."""
    dx = bx - ax
    dy = by - ay
    seg2 = dx * dx + dy * dy
    if seg2 < 1e-12:
        return ax, ay, (px - ax) ** 2 + (py - ay) ** 2

    t = ((px - ax) * dx + (py - ay) * dy) / seg2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0

    cx = ax + t * dx
    cy = ay + t * dy
    return cx, cy, (px - cx) ** 2 + (py - cy) ** 2


def find_lookahead_point(x, y, path, lookahead_dist, start_idx=0):
    """
    Return (target_point, segment_index).

    Projects the boat onto the path polyline (closest segment, not closest node) and walks
    lookahead_dist forward from that projection. start_idx keeps progress monotonic.
    """
    n = len(path)
    if n == 0:
        return None, start_idx
    if n == 1:
        return path[0], 0

    start_idx = max(0, min(start_idx, n - 2))

    # 1. Closest point on any segment at or after our committed progress point.
    proj_idx = start_idx
    proj = (path[start_idx][0], path[start_idx][1])
    best_d2 = float('inf')
    for i in range(start_idx, n - 1):
        cx, cy, d2 = _closest_point_on_segment(
            x, y, path[i][0], path[i][1], path[i + 1][0], path[i + 1][1])
        if d2 < best_d2:
            best_d2 = d2
            proj_idx = i
            proj = (cx, cy)

    # 2. Walk lookahead_dist forward along the polyline from that projection.
    remaining = float(lookahead_dist)
    cx, cy = proj
    i = proj_idx
    while i < n - 1:
        nx, ny = path[i + 1]
        seg_len = math.hypot(nx - cx, ny - cy)
        if seg_len >= remaining:
            if seg_len < 1e-9:
                return (nx, ny), proj_idx
            r = remaining / seg_len
            return (cx + (nx - cx) * r, cy + (ny - cy) * r), proj_idx
        remaining -= seg_len
        cx, cy = nx, ny
        i += 1

    # 3. Ran off the end of the path - aim at its final node.
    return path[n - 1], proj_idx


def pure_pursuit_control(rx, ry, ryaw, path, current_speed=0.0, base_speed=1500,
                         prev_error=0.0, dt=NOMINAL_DT, start_idx=0, cfg=DEFAULT_CONFIG):
    """
    Pure Pursuit: returns (base_speed, correction, target_pt, heading_err, next_start_idx).

    current_speed is ground speed in m/s. heading_err is in degrees, positive = target to
    the right. The path is never modified; progress is carried in next_start_idx.
    """
    if not path:
        return base_speed, 0, None, 0.0, start_idx

    # 1. Dynamic lookahead distance
    min_ld = cfg.pure_pursuit_min_lookahead
    max_ld = cfg.pure_pursuit_max_lookahead
    k_ld = cfg.pure_pursuit_k_speed

    lookahead_dist = float(np.clip(float(current_speed) * k_ld, min_ld, max_ld))

    # 2. Find target point
    target_pt, t_idx = find_lookahead_point(rx, ry, path, lookahead_dist, start_idx)
    if target_pt is None:
        return base_speed, 0, None, 0.0, start_idx

    # 3. Steering error
    tx, ty = target_pt

    # Sitting exactly on the target: hold heading instead of a bogus due-North turn.
    if math.hypot(tx - rx, ty - ry) < 1e-3:
        return base_speed, 0, target_pt, 0.0, t_idx

    # World frame is x = North, y = East, so atan2(dy, dx) is the compass bearing.
    target_bearing_deg = math.degrees(math.atan2(ty - ry, tx - rx)) % 360
    current_heading_deg = math.degrees(ryaw) % 360

    heading_err = (target_bearing_deg - current_heading_deg + 180) % 360 - 180

    # 4. PD (dt-normalised - see NOMINAL_DT)
    kp = cfg.pure_pursuit_kp
    kd = cfg.pure_pursuit_kd

    scale = dt_scale(dt)
    P = heading_err * kp
    D = ((heading_err - prev_error) / scale) * kd
    correction = P + D

    return base_speed, correction, target_pt, heading_err, t_idx
