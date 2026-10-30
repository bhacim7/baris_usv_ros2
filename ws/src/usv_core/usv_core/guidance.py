"""
Going to a target point, ported from legacy core/nav_process.py sections F and G.

Source: commit 5df3f5b.

WaypointGuidance keeps the state those sections kept in nav_worker's locals and splits them
in the two places the loop runs them:

  observe()   section F, every cycle: a new target restarts the initial alignment and drops
              the path; the start point for LOS is captured; bearing, distance, LOS heading
              and the two errors are computed; the A* goal is projected into the local map
  drive()     section G for "mission active, default navigation, a target": the initial
              spot-turn alignment, else A* + Pure Pursuit (Task 2, or every task with LOS
              guidance off), else the direct-drive PID with its own spot-turn threshold

Every branch ends in one Mixer.mix() call, as apply_motor_mixer() was called in legacy.
The arithmetic, the order of the state updates and the thresholds are unchanged; the golden
tests run this against the legacy loop cycle by cycle.
"""

import math

from usv_core import navigation as nav
from usv_core import planner
from usv_core.costmap import confirmed_boundary_local, task2_axis_local

TASK3_SEARCH = 'TASK3_SEARCH_KAMIKAZE'


def _is_t3(task):
    """Legacy uses two spellings of the same test in different places; both kept."""
    return 'TASK3' in task or task.startswith('T3_')


class WaypointGuidance:

    def __init__(self, cfg, mixer, log=None):
        self.cfg = cfg
        self.mixer = mixer
        self.log = log or (lambda msg: None)
        # PID (direct drive)
        self.integral = 0.0
        self.prev_error = 0.0
        # A* + Pure Pursuit
        self.path = None
        self.path_ts = 0.0
        self.path_idx = 0
        self.active_controller = None       # 'PP' or 'PID'
        self.pp_spot_turn = False
        self.plan_timer = 0
        self.prev_heading_error = 0.0
        self.last_astar_fail_log = 0.0
        # Target bookkeeping
        self.force_alignment = False
        self.alignment_started_at = None
        self.prev_target_lat = None
        self.prev_target_lon = None
        self.start_lat = None
        self.start_lon = None
        # This cycle's section F results
        self.aci_farki = 0.0
        self.control_error = 0.0
        self.adviced_course = 0.0
        self.target_dist = None
        self.goal_world = (None, None)

    def _drop_path(self):
        self.path = None
        self.path_ts = 0.0
        self.path_idx = 0

    # ------------------------------------------------------------------ section F
    def observe(self, target, task, lat, lon, heading, robot_x, robot_y, now):
        """Compute this cycle's errors; return what legacy published, or None without a target."""
        cfg = self.cfg
        self.aci_farki = 0.0
        self.control_error = 0.0
        self.adviced_course = 0.0
        self.goal_world = (None, None)
        if target is None:
            return None
        target_lat, target_lon = target

        if target_lat != self.prev_target_lat or target_lon != self.prev_target_lon:
            self.force_alignment = True
            self.alignment_started_at = now
            self.prev_target_lat = target_lat
            self.prev_target_lon = target_lon
            self.start_lat = None
            self.start_lon = None
            self._drop_path()

        # Start point for LOS: not while the GPS is unset, not during the alignment turn
        if self.start_lat is None and lat != 0.0 and lon != 0.0 and not self.force_alignment:
            self.start_lat = lat
            self.start_lon = lon

        adviced_course = nav.calculate_bearing(lat, lon, target_lat, target_lon)
        hedefe_mesafe = nav.haversine(lat, lon, target_lat, target_lon)

        base_pwm_current = cfg.base_pwm
        if _is_t3(task):
            base_pwm_current += cfg.t3_speed_pwm
        else:
            base_pwm_current += cfg.cruise_pwm
        speed_factor = max(0.0, (base_pwm_current - 1500) / 100.0)
        dynamic_los_cutoff = 3.5 + (speed_factor * 1.5)

        if self.start_lat is not None and self.start_lon is not None and \
                cfg.enable_los_guidance and hedefe_mesafe > dynamic_los_cutoff:
            xte = nav.calculate_cross_track_error(self.start_lat, self.start_lon, lat, lon,
                                                  target_lat, target_lon)
            los_lookahead = max(4.0, min(10.0, hedefe_mesafe * 0.5))
            path_bearing = nav.calculate_bearing(self.start_lat, self.start_lon,
                                                 target_lat, target_lon)
            correction_angle = math.degrees(math.atan2(cfg.los_kp * xte, los_lookahead))
            correction_angle = max(-30.0, min(30.0, correction_angle))
            los_heading = (path_bearing - correction_angle) % 360
            control_error = nav.signed_angle_difference(heading, los_heading)
            aci_farki = nav.signed_angle_difference(heading, adviced_course)
        else:
            aci_farki = nav.signed_angle_difference(heading, adviced_course)
            control_error = aci_farki

        self.aci_farki = aci_farki
        self.control_error = control_error
        self.adviced_course = adviced_course
        self.target_dist = hedefe_mesafe

        # A* goal in the local map, plotted the way the buoys are plotted
        projection_dist = min(hedefe_mesafe, cfg.goal_projection_m)
        angle_in_world = math.radians(adviced_course)
        self.goal_world = (robot_x + (projection_dist * math.cos(angle_in_world)),
                           robot_y + (projection_dist * math.sin(angle_in_world)))
        return {
            'angle_error': float(aci_farki),
            'control_error': float(control_error),
            'adviced_course': float(adviced_course),
            'target_dist': float(hedefe_mesafe),
            'target_lat': float(target_lat),
            'target_lon': float(target_lon),
        }

    # ------------------------------------------------------------------ section G
    def drive(self, target, task, robot_x, robot_y, heading, speed_mps, loop_dt, now,
              costmap, frame, vision_objects, lat, lon, waypoints):
        cfg = self.cfg
        aci_farki = self.aci_farki

        if self.force_alignment:
            if abs(aci_farki) < 15.0:
                self.force_alignment = False
                self.alignment_started_at = None
            elif self.alignment_started_at is not None and \
                    (now - self.alignment_started_at) > cfg.alignment_timeout_s:
                self.log(f'Initial alignment timed out after {cfg.alignment_timeout_s:.0f}s '
                         f'(err={aci_farki:.0f} deg). Proceeding.')
                self.force_alignment = False
                self.alignment_started_at = None

        if self.force_alignment:
            # Proportional spot turn that slows over the last 45 deg
            if 'T3' in task or 'TASK3' in task:
                base_spot_pwm = cfg.t3_spot_turn_pwm
            else:
                base_spot_pwm = cfg.spot_turn_pwm
            hiz_carpani = min(1.0, abs(aci_farki) / 45.0)
            dinamik_spot_pwm = int(60 + (base_spot_pwm - 60) * hiz_carpani)
            if aci_farki > 0:
                self.mixer.mix(1500, dinamik_spot_pwm)
            else:
                self.mixer.mix(1500, -dinamik_spot_pwm)
            return

        use_astar_planner = ('TASK2' in task) or \
            (not cfg.enable_los_guidance and task != TASK3_SEARCH)
        if not use_astar_planner:
            self._drop_path()
        if use_astar_planner:
            self._plan(task, robot_x, robot_y, now, costmap, frame, vision_objects, lat, lon,
                       waypoints)

        if self.path and len(self.path) >= 2:
            self._pure_pursuit(task, robot_x, robot_y, heading, speed_mps, loop_dt)
        else:
            self._drop_path()
            self.pp_spot_turn = False
            if target is not None and target[0] is not None and target[1] is not None:
                self._direct_drive(task, loop_dt)

    def _plan(self, task, robot_x, robot_y, now, costmap, frame, vision_objects, lat, lon,
              waypoints):
        cfg = self.cfg
        res = costmap.res
        cropped, cropped_center_m, cropped_size_px = costmap.crop(robot_x, robot_y)
        nav_map, _ = planner.get_inflated_nav_map(cropped)

        # Task 2 corridor cap (soft): keep A* from routing round the outside of the course
        corridor_cost = None
        if cfg.enable_task2_corridor and 'TASK2' in task and frame.origin is not None:
            axis = task2_axis_local(frame.origin[0], frame.origin[1], waypoints)
            if axis is not None:
                boundary = confirmed_boundary_local(
                    vision_objects, lat, lon, robot_x, robot_y,
                    cfg.corridor_confirm_sightings, cfg.corridor_confirm_max_age_s, now)
                corridor_cost = planner.build_corridor_cost(
                    nav_map.shape, cropped_center_m, res, axis[0], axis[1], boundary,
                    (robot_x, robot_y))

        tx_world, ty_world = self.goal_world
        plan_divisor = max(1, int(cfg.a_star_plan_divisor))
        self.plan_timer += 1
        if (self.plan_timer >= plan_divisor or not self.path) and tx_world is not None:
            self.plan_timer = 0
            new_path = None
            los_cost = cfg.corridor_los_block_cost
            if planner.check_line_of_sight((robot_x, robot_y), (tx_world, ty_world), nav_map,
                                           cropped_center_m, res, cropped_size_px,
                                           corridor_cost, los_cost):
                new_path = [(robot_x, robot_y), (tx_world, ty_world)]
            else:
                new_path = planner.get_path_plan(
                    (robot_x, robot_y), (tx_world, ty_world), nav_map, cropped_center_m, res,
                    cropped_size_px, heuristic_weight=cfg.a_star_heuristic_weight,
                    max_expansions=40000, time_budget_s=0.15,
                    cost_map=corridor_cost, los_max_cost=los_cost)
                # Fallback: an obstacle-avoiding path outside the corridor beats the blind PID
                if new_path is None and corridor_cost is not None:
                    new_path = planner.get_path_plan(
                        (robot_x, robot_y), (tx_world, ty_world), nav_map, cropped_center_m,
                        res, cropped_size_px, heuristic_weight=cfg.a_star_heuristic_weight)
                    if new_path:
                        self.log('Corridor cap left no route - replanning without it.')
            if new_path and len(new_path) >= 2:
                self.path = new_path
                self.path_ts = now
                self.path_idx = 0
            else:
                self._drop_path()
                if (now - self.last_astar_fail_log) >= 1.0:
                    self.last_astar_fail_log = now
                    self.log('A* found no path - falling back to obstacle-blind PID toward '
                             'target.')

        # Never follow a path older than A_STAR_MAX_PATH_AGE_S
        if self.path and (now - self.path_ts) > cfg.a_star_max_path_age_s:
            self._drop_path()

    def _pure_pursuit(self, task, robot_x, robot_y, heading, speed_mps, loop_dt):
        cfg = self.cfg
        cruise_offset = cfg.cruise_pwm
        if task.startswith('T3_') or 'TASK3' in task:
            cruise_offset = cfg.t3_speed_pwm
        base_pwm = cfg.base_pwm + cruise_offset

        if self.active_controller != 'PP':
            self.prev_heading_error = 0.0       # no derivative kick on handover
            self.active_controller = 'PP'
            self.pp_spot_turn = False

        robot_yaw = math.radians(heading)
        p_base, p_steer, _raw_target, current_error, self.path_idx = \
            planner.pure_pursuit_control(
                robot_x, robot_y, robot_yaw, self.path, current_speed=speed_mps,
                base_speed=base_pwm, prev_error=self.prev_heading_error, dt=loop_dt,
                start_idx=self.path_idx)
        self.prev_heading_error = current_error

        # Throttle taper + spot-turn guard (hysteresis)
        abs_err = abs(current_error)
        if self.pp_spot_turn:
            if abs_err < cfg.pp_spot_turn_exit_deg:
                self.pp_spot_turn = False
        elif abs_err > cfg.pp_spot_turn_enter_deg:
            self.pp_spot_turn = True

        if self.pp_spot_turn:
            p_base = cfg.base_pwm
        else:
            taper = math.cos(math.radians(min(abs_err, 90.0)))
            taper = max(cfg.pp_throttle_taper_min, taper)
            p_base = cfg.base_pwm + (cruise_offset * taper)

        self.mixer.mix(p_base, p_steer)

        # Path consumed: plan afresh next cycle
        end_pt = self.path[-1]
        if math.hypot(end_pt[0] - robot_x, end_pt[1] - robot_y) < 0.5:
            self._drop_path()

    def _direct_drive(self, task, loop_dt):
        cfg = self.cfg
        if self.active_controller != 'PID':
            self.prev_error = self.control_error   # no derivative kick on handover
            self.active_controller = 'PID'

        aci_farki = self.aci_farki
        if abs(aci_farki) > cfg.spot_turn_threshold_deg:
            if 'T3' in task or 'TASK3' in task:
                spot_pwm = cfg.t3_spot_turn_pwm
            else:
                spot_pwm = cfg.spot_turn_pwm
            if aci_farki > 0:
                self.mixer.mix(1500, spot_pwm)
            else:
                self.mixer.mix(1500, -spot_pwm)
            return

        base_pwm = cfg.base_pwm
        if 'TASK3' in task or task.startswith('T3_'):
            base_pwm += cfg.t3_speed_pwm
        else:
            base_pwm += cfg.cruise_pwm

        kp = cfg.direct_drive_kp
        ki = cfg.direct_drive_ki
        speed_factor_pid = max(0.0, (base_pwm - 1500) / 100.0)
        kd = cfg.direct_drive_kd + (speed_factor_pid * 0.3)

        error = self.control_error
        dts = planner.dt_scale(loop_dt)

        # Anti-windup: integrate only in the linear region, decay outside it
        if abs(error) < cfg.anti_windup_deg:
            self.integral += error * dts
        else:
            self.integral *= 0.9
        if (error > 0 and self.integral < 0) or (error < 0 and self.integral > 0):
            self.integral *= 0.5
        windup_limit = cfg.anti_windup_limit
        self.integral = max(-windup_limit, min(windup_limit, self.integral))

        derivative = (error - self.prev_error) / dts
        self.prev_error = error

        steering_correction = (error * kp) + (self.integral * ki) + (derivative * kd)
        self.mixer.mix(base_pwm, steering_correction)
