"""
Run a golden scenario through the new usv_core.navigator.Navigator.

Every cycle is recorded exactly as legacy_runner records the legacy loop, so the two
record lists can be compared with ==.

The world is the same (scenario.BoatModel, scenario.Perception, the same virtual clock
arithmetic, commands injected at the same cycles). The settings are the legacy config module
read through NavConfig.from_module(), with the overrides legacy_runner applies.

An emergency_stop ends the legacy run (shutdown); the new navigator latches and keeps
running, neutral, to the scenario's end.

Each record also carries 't3', what the Task 3 search did in that cycle (servo, grace,
hold, a pattern state, ...). Legacy records have no such field and the comparison ignores
it; tests use it to check that a scenario really exercises the branch it was written for.
"""

import dataclasses
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from legacy_runner import _import_legacy  # noqa: E402
from scenario import BoatModel, mavlink_deg, Perception  # noqa: E402

from usv_core import planner  # noqa: E402
from usv_core.nav_config import NavConfig  # noqa: E402
from usv_core.navigator import Navigator  # noqa: E402

T0 = 1_000_000.0


def nav_config_for(scenario):
    """Return the settings legacy_runner gives the legacy loop for this scenario."""
    cfg, _, _ = _import_legacy()
    nc = NavConfig.from_module(cfg)
    waypoints = dict(nc.waypoints)
    waypoints.update({name: tuple(v) for name, v in scenario.waypoints.items()})
    return dataclasses.replace(nc, waypoints=waypoints, drone_active=False,
                               task3_kamikaze_color=scenario.target_color)


class _Clock:
    """Stand-in for the time module inside usv_core.planner (A*'s time budget)."""

    def __init__(self):
        self.t = T0

    def time(self):
        return self.t


def run_new(scenario, log=None):
    sc = scenario
    nc = nav_config_for(sc)
    period = 1.0 / max(1.0, nc.loop_hz)
    origin = sc.waypoints['T2_ZONE_ENTRY']
    boat = BoatModel(origin[0], origin[1], *sc.start)
    perception = Perception(sc.buoys, sc.dropouts)

    def sense():
        lat, lon = boat.latlon()
        return (mavlink_deg(lat), mavlink_deg(lon)), \
            (boat.s.heading + sc.heading_offset_deg) % 360.0

    clock = _Clock()
    old_planner_time = planner.time
    planner.time = clock
    try:
        t = T0
        gps, heading = sense()
        nav = Navigator(nc, start_time=t, mission_started=sc.mission_started,
                        manual=sc.manual, log=log)
        nav.task = sc.start_task
        vision = perception.update(boat, t)
        records = []
        cycle = 0
        finished_cycles = 0
        while True:
            clock.t = t
            nav.mission.search.mode = None
            cmd = nav.step(t, gps, abs(boat.s.surge), heading, vision)
            s = nav.status
            records.append({
                'cycle': cycle,
                't': round(t - T0, 6),
                'task': s['current_task'],
                'pwm': (cmd.rear_left, cmd.rear_right, cmd.front_left, cmd.front_right,
                        cmd.steer),
                'target': (s['target_lat'], s['target_lon']),
                'angle_error': s['angle_error'],
                'control_error': s['control_error'],
                'target_dist': s['target_dist'],
                'boat': (round(boat.s.east, 4), round(boat.s.north, 4),
                         round(boat.s.heading, 4)),
                't3': nav.mission.search.mode,
            })
            cycle += 1
            boat.step({'rear_left': cmd.rear_left, 'rear_right': cmd.rear_right,
                       'front_left': cmd.front_left, 'front_right': cmd.front_right,
                       'steer': cmd.steer}, period)
            gps, heading = sense()
            vision = perception.update(boat, t + period)
            for t_cmd, gcs_cmd in sc.commands:
                if abs((cycle * period) - t_cmd) < period / 2:
                    nav.submit(dict(gcs_cmd))
            if s['current_task'] == 'FINISHED':
                finished_cycles += 1
            if cycle * period >= sc.duration_s or \
                    (sc.stop_when_finished and finished_cycles >= 10):
                break
            t += period
    finally:
        planner.time = old_planner_time
    return records
