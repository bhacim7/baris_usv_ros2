"""
Mission state router, ported from legacy core/nav_process.py section E (commit 5df3f5b).

Mission.route() decides, once per cycle, which task state the boat is in and which point it
steers to:

  Task 1    three gate points in a row, 3 m acceptance
  Task 2    entry -> mid -> end, 3 m acceptance; on the way to the entry an approach point set
            back along the corridor axis, latched once reached (t2_aligned)
  Task 3    T3_START -> T3_MID, 2 m acceptance, then TASK3_SEARCH_KAMIKAZE; on the T3_MID leg
            a fresh, confirmed sighting of the target colour switches to the search early
  FINISHED  neutral on the mixer (and once more in the control section, as in legacy)

When the mission is not active (not started, or manual) any state change is rolled back but
the target is still returned, so the GCS keeps seeing distance and bearing to the next point.

The Task 3 search itself (pattern, lock, visual servo, the collision that ends the run) is
usv_core.task3.Task3Search, run only while the mission is active.
"""

import math

from usv_core import navigation as nav
from usv_core.task3 import TASK3_SEARCH, Task3Search
COLOR_TO_CID = {'red': 0, 'yellow': 1, 'black': 2, 'orange': 3, 'green': 4}


class Mission:

    def __init__(self, cfg, mixer, log=None):
        self.cfg = cfg
        self.mixer = mixer
        self.log = log or (lambda msg: None)
        self.t2_aligned = False         # Task 2 entrance alignment latch
        self.finished_printed = False
        self.search = Task3Search(cfg, mixer, log)

    def target_cid(self, drone_target_color):
        """Return the colour Task 3 hunts: config, or the drone's report when active."""
        color = self.cfg.task3_kamikaze_color
        if self.cfg.drone_active:
            color = (color if drone_target_color is None else drone_target_color) or color
        return COLOR_TO_CID.get(str(color).lower(), 0)

    def route(self, task, lat, lon, heading, waypoints, mission_active, vision_objects,
              drone_target_color, now):
        """Advance the task state; return (task, target (lat, lon) or None, skip_default_nav)."""
        target = None
        skip_default_nav = False
        task_before = task

        if 'TASK1' in task or task == 'FINISHED':
            if task == 'FINISHED':
                if not self.finished_printed:
                    self.log('ALL TASKS COMPLETE')
                    self.finished_printed = True
                self.mixer.mix(1500, 0)
            else:
                task, target = self._task1(task, lat, lon, waypoints)

        elif 'TASK2' in task:
            task, target = self._task2(task, lat, lon, waypoints)

        elif 'T3' in task or 'TASK3' in task:
            if task == TASK3_SEARCH and not mission_active:
                # The search drives the thrusters itself; never while disarmed or in manual
                skip_default_nav = True
            elif task == TASK3_SEARCH:
                task, target = self.search.step(lat, lon, heading, vision_objects, waypoints,
                                                self.target_cid(drone_target_color), now)
                if target is None:
                    skip_default_nav = True
            else:
                task, target = self._task3(task, lat, lon, waypoints)
                if task == 'T3_MID' and mission_active:
                    task, target = self._early_engage(task, target, vision_objects,
                                                      drone_target_color, now)

        if not mission_active:
            task = task_before
        return task, target, skip_default_nav

    # ------------------------------------------------------------------ tasks
    def _task1(self, task, lat, lon, waypoints):
        # Back in Task 1: a fresh run at the Task 2 mouth is still to come
        self.t2_aligned = False
        if task == 'TASK1_APPROACH':
            return 'TASK1_STATE_ENTER', None
        targets = {
            'TASK1_STATE_ENTER': waypoints['T1_GATE_ENTER'],
            'TASK1_STATE_MID': waypoints['T1_GATE_MID'],
            'TASK1_STATE_EXIT': waypoints['T1_GATE_EXIT'],
        }
        t_lat, t_lon = targets.get(task, (None, None))
        if t_lat and nav.haversine(lat, lon, t_lat, t_lon) < 3.0:
            if task == 'TASK1_STATE_ENTER':
                task = 'TASK1_STATE_MID'
            elif task == 'TASK1_STATE_MID':
                task = 'TASK1_STATE_EXIT'
            elif task == 'TASK1_STATE_EXIT':
                task = 'TASK2_START'
        return task, (None if t_lat is None else (t_lat, t_lon))

    def _task2(self, task, lat, lon, waypoints):
        cfg = self.cfg
        targets = {
            'TASK2_START': tuple(waypoints['T2_ZONE_ENTRY']) + ('TASK2_GO_TO_MID',),
            'TASK2_GO_TO_MID': tuple(waypoints['T2_ZONE_MID']) + ('TASK2_GO_TO_END',),
            'TASK2_GO_TO_END': tuple(waypoints['T2_ZONE_END']) + ('T3_START',),
        }
        if task not in targets:
            return task, None
        t_lat, t_lon, next_state = targets[task]
        if nav.haversine(lat, lon, t_lat, t_lon) < 3.0:
            return next_state, (t_lat, t_lon)

        # Entrance approach: aim at a point set back along the corridor axis while off to
        # the side of the mouth or still behind that point, so the last leg runs along the
        # corridor instead of slipping between two boundary buoys.
        if task == 'TASK2_START':
            end_lat, end_lon = waypoints['T2_ZONE_END']
            if end_lat and end_lon and nav.haversine(t_lat, t_lon, end_lat, end_lon) > 5.0:
                offset = cfg.task2_approach_offset_m
                reached = cfg.task2_approach_reached_m
                axis_brg = nav.calculate_bearing(t_lat, t_lon, end_lat, end_lon)
                a_lat, a_lon = nav.calculate_obj_gps(t_lat, t_lon, offset,
                                                     (axis_brg + 180.0) % 360.0)
                brg_to_boat = nav.calculate_bearing(t_lat, t_lon, lat, lon)
                r_boat = nav.haversine(t_lat, t_lon, lat, lon)
                rel = math.radians(brg_to_boat - axis_brg)
                s_along = r_boat * math.cos(rel)
                d_across = abs(r_boat * math.sin(rel))
                lateral_tol = cfg.task2_approach_lateral_m
                dist_to_approach = nav.haversine(lat, lon, a_lat, a_lon)
                if dist_to_approach <= reached:
                    self.t2_aligned = True
                if (not self.t2_aligned
                        and s_along < 0.0
                        and dist_to_approach > reached
                        and (s_along < -offset or d_across > lateral_tol)):
                    return task, (a_lat, a_lon)
        return task, (t_lat, t_lon)

    def _task3(self, task, lat, lon, waypoints):
        if task == 'TASK3_APPROACH':
            return 'T3_START', None
        targets = {
            'T3_START': tuple(waypoints['T3_START']) + (
                'T3_MID' if self.cfg.enable_task3 else 'FINISHED',),
            'T3_MID': tuple(waypoints['T3_MID']) + (TASK3_SEARCH,),
        }
        if task not in targets:
            return task, None
        t_lat, t_lon, next_state = targets[task]
        if nav.haversine(lat, lon, t_lat, t_lon) < 2.0:
            task = next_state
        return task, (t_lat, t_lon)

    def _early_engage(self, task, target, vision_objects, drone_target_color, now):
        """T3_MID leg only: a target seen 3+ times, freshly, in a committed colour."""
        committed = (not self.cfg.drone_active) or bool(drone_target_color)
        if not committed:
            return task, target
        tc = self.target_cid(drone_target_color)
        for obj in vision_objects:
            if (obj.get('cid') == tc
                    and obj.get('cx') is not None
                    and int(obj.get('seen', 0)) >= 3
                    and obj.get('last_seen') is not None
                    and (now - obj['last_seen']) <= self.cfg.t3_max_detection_age_s):
                self.log('Target sighted on the T3_MID leg - engaging early.')
                return TASK3_SEARCH, None
        return task, target
