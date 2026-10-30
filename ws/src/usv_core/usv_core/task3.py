"""
Task 3 search and kamikaze run, ported from legacy core/nav_process.py (commit 5df3f5b).

Task3Search.step() is the TASK3_SEARCH_KAMIKAZE branch of the mission router, one call per
cycle while the mission is active. In priority order:

  servo   a fresh detection of the target colour (seen within T3_MAX_DETECTION_AGE_S):
          the nearest one is locked by track id and steered on by its pixel column at
          T3 speed; closer than 1 m, or a box larger than 300000 px, ends the run (FINISHED)
  grace   the locked track missed its freshness window: keep steering on its last pixel
          column for T3_LOCK_GRACE_S before considering any other candidate
  hold    the lock is gone but the last servo distance was within T3_HOLD_MAX_DIST_M and
          less than T3_HOLD_S ago: hold course at T3 speed
  search  the pattern below

The search pattern, framed on the T3_MID waypoint and the heading frozen at the first search
entry: a wide pan (PAN_A/B/C), then an expanding V fan (V_CALC, V_LEG to the arm tip and a
probe point beyond it, V_TIPPAN_A/B, V_HOME); a bounding box around T3_MID (RETURN, then a
mirrored restart); and an anchor shortcut - any fresh Task 3 buoy seen while scanning sends
the boat to a standoff point and a patrol parallel to the buoy line (GOTO_STANDOFF,
PATROL_1/2).

step() returns (task, target). A target (lat, lon) is driven by the normal guidance (initial
alignment, LOS, direct-drive PID), as in legacy; with no target the thrusters were already
commanded here - or, in transition cycles (V_CALC, reaching a pattern waypoint, back at
T3_MID), not at all, so they hold their last command for one cycle (kept from legacy).

The anchor/patrol branch is not covered by a golden scenario (see docs/open-items.md).
"""

import math

from usv_core import navigation as nav

TASK3_SEARCH = 'TASK3_SEARCH_KAMIKAZE'
SCAN_STATES = ('PAN_A', 'PAN_B', 'PAN_C', 'V_LEG', 'V_TIPPAN_A', 'V_TIPPAN_B', 'V_HOME')
ANCHOR_CIDS = (0, 2, 4)          # red, black, green: the Task 3 buoys


class Task3Search:

    def __init__(self, cfg, mixer, log=None):
        self.cfg = cfg
        self.mixer = mixer
        self.log = log or (lambda msg: None)
        # Search pattern
        self.state = 'INIT'
        self.heading0 = None         # search frame axis, frozen at the first search entry
        self.move_target = None      # (lat, lon) of the standoff / patrol waypoint
        self.patrol_side = 1         # first V-arm / pan side; flipped on a mirrored restart
        self.v_round = 0
        self.v_arm = 0
        self.v_wps = None            # [tip, probe] of the arm being flown
        self.v_idx = 0
        # Visual servo lock
        self.lock_id = None
        self.lock_seen_ts = 0.0
        self.lock_cx = None
        self.last_servo_dist = None
        self.last_servo_ts = None
        # What this cycle did (servo / grace / hold / finished / a pattern state)
        self.mode = None

    def step(self, lat, lon, heading, vision_objects, waypoints, target_cid, now):
        cfg = self.cfg
        t3_base_pwm = cfg.base_pwm + cfg.t3_speed_pwm

        # Target selection: nearest fresh candidate, held by an id lock
        fresh = []
        for obj in vision_objects:
            if obj.get('cid') != target_cid or obj.get('cx') is None:
                continue
            ls = obj.get('last_seen')
            if ls is None or (now - ls) > cfg.t3_max_detection_age_s:
                continue
            fresh.append(obj)

        target_obj = None
        grace_steer = False
        if self.lock_id is not None:
            for o in fresh:
                if o.get('id') == self.lock_id:
                    target_obj = o
                    break
            if target_obj is None:
                if (self.lock_cx is not None
                        and (now - self.lock_seen_ts) <= cfg.t3_lock_grace_s):
                    grace_steer = True
                else:
                    self.lock_id = None
        if target_obj is None and not grace_steer and fresh:
            target_obj = min(fresh, key=lambda o: float(o.get('dist', 1e9)))
            self.lock_id = target_obj.get('id')
            self.log(f'Locked target id={self.lock_id} at '
                     f"{float(target_obj.get('dist', -1)):.1f} m")

        if target_obj is not None:
            dist_m = float(target_obj.get('dist', 10.0))
            self.last_servo_dist = dist_m
            self.last_servo_ts = now
            self.lock_seen_ts = now
            self.lock_cx = target_obj.get('cx')
            self._servo(target_obj.get('cx'), t3_base_pwm)
            self.mode = 'servo'
            if dist_m < 1.0 or target_obj.get('area', 0) > 300000:
                self.log('KAMIKAZE COLLISION CONFIRMED! STOPPING VEHICLE.')
                self.mixer.mix(1500, 0)
                self.mode = 'finished'
                return 'FINISHED', None
            return TASK3_SEARCH, None

        if grace_steer:
            self._servo(self.lock_cx, t3_base_pwm)
            self.mode = 'grace'
            return TASK3_SEARCH, None

        if (self.last_servo_ts is not None
                and self.last_servo_dist is not None
                and self.last_servo_dist <= cfg.t3_hold_max_dist_m
                and (now - self.last_servo_ts) <= cfg.t3_hold_s):
            self.mixer.mix(t3_base_pwm, 0)
            self.mode = 'hold'
            return TASK3_SEARCH, None

        target = self._search(lat, lon, heading, vision_objects, waypoints, now)
        self.mode = self.state
        return TASK3_SEARCH, target

    def _servo(self, cx, base_pwm):
        cfg = self.cfg
        pixel_error = cx - (cfg.image_width / 2)
        kp_pixel = cfg.kp_pixel
        if cfg.task3_invert_steering:
            steering_correction = -pixel_error * kp_pixel
        else:
            steering_correction = pixel_error * kp_pixel
        self.mixer.mix(base_pwm, steering_correction)

    def _pan(self, heading, target_h):
        """Turn on the spot toward target_h, slowing over the last 45 deg; True when there."""
        cfg = self.cfg
        spot_pwm = cfg.t3_spot_turn_pwm
        diff = nav.signed_angle_difference(heading, target_h)
        if abs(diff) < cfg.t3_pan_tol_deg:
            self.mixer.mix(1500, 0)
            return True
        ramp = min(1.0, abs(diff) / 45.0)
        pan_pwm = int(60 + (spot_pwm - 60) * ramp)
        self.mixer.mix(1500, pan_pwm if diff > 0 else -pan_pwm)
        return False

    def _search(self, lat, lon, heading, vision_objects, waypoints, now):
        cfg = self.cfg
        target = None
        mid_lat, mid_lon = waypoints['T3_MID']
        have_mid = bool(mid_lat and mid_lon)

        if self.state == 'INIT':
            if self.heading0 is None:
                self.heading0 = heading
            self.v_round = 0
            self.v_arm = 0
            self.v_wps = None
            self.v_idx = 0
            self.state = 'PAN_A'
            self.log(f'Search start: axis {self.heading0:.0f} deg, '
                     f"first side {'L' if self.patrol_side < 0 else 'R'}")

        # Bounds: stay inside the plausible field around T3_MID
        if have_mid and self.heading0 is not None and self.state != 'RETURN':
            d_mid = nav.haversine(mid_lat, mid_lon, lat, lon)
            brg_mid = nav.calculate_bearing(mid_lat, mid_lon, lat, lon)
            rel_mid = math.radians(brg_mid - self.heading0)
            s_fwd = d_mid * math.cos(rel_mid)
            d_side = abs(d_mid * math.sin(rel_mid))
            if (d_side > cfg.t3_bound_lateral_m
                    or s_fwd > cfg.t3_bound_forward_m
                    or s_fwd < -10.0):
                self.log('Search bound reached - returning to T3_MID.')
                self.state = 'RETURN'

        # Anchor scan: any fresh Task 3 buoy seen while scanning (pos_ok not required)
        if cfg.t3_anchor_enabled and self.state in SCAN_STATES:
            anchor = None
            a_best = 1e9
            for obj in vision_objects:
                if obj.get('cid') not in ANCHOR_CIDS:
                    continue
                ls = obj.get('last_seen')
                if ls is None or (now - ls) > cfg.t3_max_detection_age_s:
                    continue
                if int(obj.get('seen', 0)) < 3:
                    continue
                o_lat, o_lon = obj.get('lat'), obj.get('lon')
                if not o_lat or not o_lon:
                    continue
                a_d = float(obj.get('dist', 1e9))
                if a_d < a_best:
                    a_best = a_d
                    anchor = (o_lat, o_lon)
            if anchor is not None:
                brg_away = nav.calculate_bearing(anchor[0], anchor[1], lat, lon)
                self.move_target = nav.calculate_obj_gps(anchor[0], anchor[1],
                                                         cfg.t3_standoff_m, brg_away)
                self.state = 'GOTO_STANDOFF'
                self.log(f'Anchor buoy at {a_best:.1f} m - moving to standoff on the line.')

        state = self.state
        if state in ('PAN_A', 'PAN_B', 'PAN_C'):
            pan_deg = cfg.t3_pan_wide_deg
            if state == 'PAN_A':
                target_h = (self.heading0 - self.patrol_side * pan_deg) % 360
            elif state == 'PAN_B':
                target_h = (self.heading0 + self.patrol_side * pan_deg) % 360
            else:
                target_h = self.heading0
            if self._pan(heading, target_h):
                self.state = {'PAN_A': 'PAN_B', 'PAN_B': 'PAN_C', 'PAN_C': 'V_CALC'}[state]

        elif state == 'V_CALC':
            degs = cfg.t3_v_round_deg
            legs = cfg.t3_v_round_leg_m
            if not have_mid or self.heading0 is None or self.v_round >= len(degs):
                self.state = 'RETURN'
            else:
                v_deg = float(degs[self.v_round])
                v_leg = float(legs[self.v_round])
                v_sgn = self.patrol_side if self.v_arm == 0 else -self.patrol_side
                tip = nav.calculate_obj_gps(mid_lat, mid_lon, v_leg,
                                            (self.heading0 + v_sgn * v_deg) % 360)
                probe = nav.calculate_obj_gps(tip[0], tip[1], cfg.t3_v_probe_m, self.heading0)
                self.v_wps = [tip, probe]
                self.v_idx = 0
                self.state = 'V_LEG'
                self.log(f'V round {self.v_round + 1}: '
                         f"{'+' if v_sgn > 0 else '-'}{v_deg:.0f} deg / {v_leg:.0f} m arm")

        elif state == 'V_LEG':
            if not self.v_wps or self.v_idx >= len(self.v_wps):
                self.state = 'V_TIPPAN_A'
            else:
                target = self.v_wps[self.v_idx]
                if nav.haversine(lat, lon, target[0], target[1]) < 2.0:
                    self.v_idx += 1
                    target = None

        elif state in ('V_TIPPAN_A', 'V_TIPPAN_B'):
            tp = cfg.t3_v_tip_pan_deg
            target_h = (self.heading0 - tp if state == 'V_TIPPAN_A'
                        else self.heading0 + tp) % 360
            if self._pan(heading, target_h):
                self.state = 'V_TIPPAN_B' if state == 'V_TIPPAN_A' else 'V_HOME'

        elif state == 'V_HOME':
            if not have_mid:
                self.state = 'RETURN'
            else:
                target = (mid_lat, mid_lon)
                if nav.haversine(lat, lon, mid_lat, mid_lon) < 2.5:
                    target = None
                    if self.v_arm == 0:
                        self.v_arm = 1
                        self.state = 'V_CALC'
                    else:
                        self.v_arm = 0
                        self.v_round += 1
                        if self.v_round >= len(cfg.t3_v_round_deg):
                            # Whole fan flown empty: mirror and start over
                            self.v_round = 0
                            self.patrol_side = -self.patrol_side
                            self.state = 'INIT'
                            self.log('V fan exhausted - mirrored restart.')
                        else:
                            self.state = 'V_CALC'

        elif state == 'GOTO_STANDOFF':
            target = self.move_target
            if nav.haversine(lat, lon, target[0], target[1]) < 2.0:
                # Patrol parallel to the buoy line: first leg one way, second twice as far
                lat_brg = (self.heading0 + self.patrol_side * 90.0) % 360
                self.move_target = nav.calculate_obj_gps(lat, lon, cfg.t3_patrol_first_leg_m,
                                                         lat_brg)
                target = None
                self.state = 'PATROL_1'

        elif state == 'PATROL_1':
            target = self.move_target
            if nav.haversine(lat, lon, target[0], target[1]) < 2.0:
                lat_brg = (self.heading0 - self.patrol_side * 90.0) % 360
                self.move_target = nav.calculate_obj_gps(lat, lon, cfg.t3_patrol_second_leg_m,
                                                         lat_brg)
                target = None
                self.state = 'PATROL_2'

        elif state == 'PATROL_2':
            target = self.move_target
            if nav.haversine(lat, lon, target[0], target[1]) < 2.0:
                target = None
                self.state = 'RETURN'

        elif state == 'RETURN':
            # Come home to T3_MID and run the whole pattern again, mirrored
            if have_mid:
                target = (mid_lat, mid_lon)
                if nav.haversine(lat, lon, mid_lat, mid_lon) < 2.5:
                    target = None
                    self.patrol_side = -self.patrol_side
                    self.state = 'INIT'
                    self.log('Back at T3_MID - restarting mirrored.')
            else:
                self.patrol_side = -self.patrol_side
                self.state = 'INIT'

        return target
