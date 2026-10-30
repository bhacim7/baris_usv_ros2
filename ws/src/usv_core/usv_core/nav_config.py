"""
Navigation settings, replacing the global `config` module of legacy nav_process.

nav_process read getattr(cfg, NAME, default) all over the loop (commit 5df3f5b).

  NavConfig            one field per setting; the defaults are the values of the legacy
                       config.py, so NavConfig() behaves like the boat did
  NavConfig.from_module(cfg)
                       reads a config module exactly the way nav_process did: the same names,
                       and nav_process's own getattr fallback where a name is missing. The
                       golden tests use it to give the new navigator the very settings the
                       legacy loop runs with.

Waypoints are the eight GCS points GPS1..GPS8 (config T1_GATE_ENTER .. T3_MID), as
(lat, lon); legacy treats a latitude of 0 as "not set".
"""

from dataclasses import dataclass, field, fields

# GCS index -> waypoint name, as nav_process's set_gps handler
WAYPOINT_NAMES = ('T1_GATE_ENTER', 'T1_GATE_MID', 'T1_GATE_EXIT', 'T2_ZONE_ENTRY',
                  'T2_ZONE_MID', 'T2_ZONE_END', 'T3_START', 'T3_MID')

# field -> (legacy config name, nav_process's getattr fallback). Fields without an entry were
# local constants in nav_process.
LEGACY_NAMES = {
    'loop_hz': ('NAV_LOOP_HZ', 25.0),
    'vision_read_hz': ('VISION_READ_HZ', 10.0),
    'costmap_res_m_per_px': ('COSTMAP_RES_M_PER_PX', 0.10),
    'crop_radius_m': ('A_STAR_CROP_RADIUS_M', 20.0),
    'buoy_radius_m': ('BUOY_RADIUS_M', 0.25),
    'base_pwm': ('BASE_PWM', 1500),
    'cruise_pwm': ('CRUISE_PWM', 80),
    't3_speed_pwm': ('T3_SPEED_PWM', 100),
    'spot_turn_pwm': ('SPOT_TURN_PWM', 200),
    't3_spot_turn_pwm': ('T3_SPOT_TURN_PWM', 140),
    'spot_turn_threshold_deg': ('SPOT_TURN_THRESHOLD', 45.0),
    'enable_los_guidance': ('ENABLE_LOS_GUIDANCE', True),
    'los_kp': ('LOS_KP', 1.5),
    'goal_projection_m': ('A_STAR_GOAL_PROJECTION_M', 10.0),
    'enable_task2_corridor': ('ENABLE_TASK2_CORRIDOR', True),
    'corridor_los_block_cost': ('CORRIDOR_LOS_BLOCK_COST', 3.0),
    'a_star_plan_divisor': ('A_STAR_PLAN_DIVISOR', 5),
    'a_star_heuristic_weight': ('A_STAR_HEURISTIC_WEIGHT', 2.5),
    'a_star_max_path_age_s': ('A_STAR_MAX_PATH_AGE_S', 0.6),
    'pp_spot_turn_enter_deg': ('PP_SPOT_TURN_ENTER_DEG', 60.0),
    'pp_spot_turn_exit_deg': ('PP_SPOT_TURN_EXIT_DEG', 35.0),
    'pp_throttle_taper_min': ('PP_THROTTLE_TAPER_MIN', 0.25),
    'direct_drive_kp': ('DIRECT_DRIVE_KP', 1.5),
    'direct_drive_ki': ('DIRECT_DRIVE_KI', 0.05),
    'direct_drive_kd': ('DIRECT_DRIVE_KD', 0.8),
    'anti_windup_deg': ('ANTI_WINDUP_DEG', 15.0),
    'anti_windup_limit': ('ANTI_WINDUP_LIMIT', 500.0),
    'corridor_confirm_sightings': ('CORRIDOR_CONFIRM_SIGHTINGS', 3),
    'corridor_confirm_max_age_s': ('CORRIDOR_CONFIRM_MAX_AGE_S', 1.5),
    'task2_approach_offset_m': ('TASK2_APPROACH_OFFSET_M', 12.0),
    'task2_approach_reached_m': ('TASK2_APPROACH_REACHED_M', 3.0),
    'task2_approach_lateral_m': ('TASK2_APPROACH_LATERAL_M', 3.0),
    'enable_task3': ('ENABLE_TASK3', True),
    'task3_kamikaze_color': ('TASK3_KAMIKAZE_COLOR', 'red'),
    'drone_active': ('DRONE_ACTIVE', False),
    'kp_pixel': ('Kp_PIXEL', 0.3),
    'task3_invert_steering': ('TASK3_INVERT_STEERING', False),
    't3_lock_grace_s': ('T3_LOCK_GRACE_S', 0.6),
    't3_hold_max_dist_m': ('T3_HOLD_MAX_DIST_M', 5.0),
    't3_hold_s': ('T3_HOLD_S', 2.0),
    't3_bound_lateral_m': ('T3_BOUND_LATERAL_M', 30.0),
    't3_bound_forward_m': ('T3_BOUND_FORWARD_M', 20.0),
    't3_anchor_enabled': ('T3_ANCHOR_ENABLED', True),
    't3_standoff_m': ('T3_STANDOFF_M', 8.0),
    't3_pan_wide_deg': ('T3_PAN_WIDE_DEG', 90.0),
    't3_pan_tol_deg': ('T3_PAN_TOL_DEG', 10.0),
    't3_v_round_deg': ('T3_V_ROUND_DEG', (45.0, 70.0, 100.0)),
    't3_v_round_leg_m': ('T3_V_ROUND_LEG_M', (10.0, 20.0, 30.0)),
    't3_v_probe_m': ('T3_V_PROBE_M', 8.0),
    't3_v_tip_pan_deg': ('T3_V_TIP_PAN_DEG', 45.0),
    't3_patrol_first_leg_m': ('T3_PATROL_FIRST_LEG_M', 15.0),
    't3_patrol_second_leg_m': ('T3_PATROL_SECOND_LEG_M', 30.0),
    'start_task': ('MEVCUT_GOREV', 'TASK1_STATE_ENTER'),
    'vehicle_id': ('VEHICLE_ID', 1),
    'max_pwm_change': ('MAX_PWM_CHANGE', 60),
    'steer_max_pwm': ('STEER_MAX_PWM', 1900),
    'steer_min_pwm': ('STEER_MIN_PWM', 1100),
}


def _legacy_waypoints():
    return {
        'T1_GATE_ENTER': (40.8090735, 29.2622975),
        'T1_GATE_MID': (40.8089036, 29.2622449),
        'T1_GATE_EXIT': (40.8088117, 29.2624693),
        'T2_ZONE_ENTRY': (40.8090289, 29.262524),
        'T2_ZONE_MID': (40.8095851, 29.2622612),
        'T2_ZONE_END': (40.8095851, 29.2622612),
        'T3_START': (40.809666, 29.2622792),
        'T3_MID': (40.8096887, 29.2622795),
    }


@dataclass
class NavConfig:
    """Defaults = legacy config.py at commit 5df3f5b (test_nav_config.py checks this)."""

    loop_hz: float = 25.0
    vision_read_hz: float = 10.0
    costmap_res_m_per_px: float = 0.1
    crop_radius_m: float = 20.0
    buoy_radius_m: float = 0.25
    base_pwm: int = 1500
    cruise_pwm: int = 180
    t3_speed_pwm: int = 200
    spot_turn_pwm: int = 200
    t3_spot_turn_pwm: int = 140
    spot_turn_threshold_deg: float = 30.0
    enable_los_guidance: bool = True
    los_kp: float = 1.2
    goal_projection_m: float = 10.0
    enable_task2_corridor: bool = True
    corridor_los_block_cost: float = 3.0
    a_star_plan_divisor: int = 5
    a_star_heuristic_weight: float = 2.5
    a_star_max_path_age_s: float = 0.6
    pp_spot_turn_enter_deg: float = 60.0
    pp_spot_turn_exit_deg: float = 35.0
    pp_throttle_taper_min: float = 0.25
    direct_drive_kp: float = 1.5
    direct_drive_ki: float = 0.05
    direct_drive_kd: float = 1.2
    anti_windup_deg: float = 15.0
    anti_windup_limit: float = 500.0
    corridor_confirm_sightings: int = 3
    corridor_confirm_max_age_s: float = 6.0
    task2_approach_offset_m: float = 5.0
    task2_approach_reached_m: float = 2.0
    task2_approach_lateral_m: float = 8.0
    enable_task3: bool = True
    task3_kamikaze_color: str = 'black'
    drone_active: bool = True
    kp_pixel: float = 0.3
    task3_invert_steering: bool = False
    t3_lock_grace_s: float = 0.6
    t3_hold_max_dist_m: float = 5.0
    t3_hold_s: float = 2.0
    t3_bound_lateral_m: float = 30.0
    t3_bound_forward_m: float = 20.0
    t3_anchor_enabled: bool = True
    t3_standoff_m: float = 8.0
    t3_pan_wide_deg: float = 90.0
    t3_pan_tol_deg: float = 10.0
    t3_v_round_deg: tuple = (45.0, 70.0, 100.0)
    t3_v_round_leg_m: tuple = (10.0, 20.0, 30.0)
    t3_v_probe_m: float = 8.0
    t3_v_tip_pan_deg: float = 45.0
    t3_patrol_first_leg_m: float = 15.0
    t3_patrol_second_leg_m: float = 30.0
    start_task: str = 'TASK1_STATE_ENTER'
    vehicle_id: int = 1
    max_pwm_change: int = 60
    steer_max_pwm: int = 1900
    steer_min_pwm: int = 1100
    # nav_process local constants
    alignment_timeout_s: float = 12.0
    t3_max_detection_age_s: float = 0.3
    image_width: int = 1280               # Task 3 pixel error is cx - 1280 / 2
    waypoints: dict = field(default_factory=_legacy_waypoints)

    @classmethod
    def from_module(cls, cfg):
        kwargs = {name: getattr(cfg, legacy, fallback)
                  for name, (legacy, fallback) in LEGACY_NAMES.items()}
        kwargs['waypoints'] = {w: (getattr(cfg, w + '_LAT', 0), getattr(cfg, w + '_LON', 0))
                               for w in WAYPOINT_NAMES}
        return cls(**kwargs)

    def field_names(self):
        return [f.name for f in fields(self)]
