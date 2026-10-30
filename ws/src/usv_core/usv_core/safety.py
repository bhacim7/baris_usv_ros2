"""
Emergency stop latch and the pre-flight check.

EmergencyLatch is a deliberate change from legacy (decided at the start of phase 5). In
legacy, emergency_stop set shared_state['shutdown']: that cycle still ran with normal motor
commands, then the loop ended, its `finally` neutralised and disarmed, and the orchestrator
shut every process down - the program had to be restarted. Under ROS a node that exits can
be respawned, so the stop is a latch instead: the navigator neutralises in the very cycle the
command arrives and stays neutral until it is released. fc_bridge / sim_bridge switch the
motor relay off at the same time; the latch is released by a set_task received while the
relay is on again (ARCHITECTURE.md section 9, see Navigator.set_task).

preflight_ready() is legacy main_orchestrator.pre_flight_check unchanged: the mission may
start once the GPS latitude and the heading are both non-zero. It does not check fix
quality, data age or waypoints (see ARCHITECTURE.md section 14).
"""


class EmergencyLatch:

    def __init__(self):
        self.active = False
        self.reason = None
        self.tripped_at = None

    def trip(self, reason, now=None):
        """Latch the stop; a second trip keeps the first reason and time."""
        if not self.active:
            self.active = True
            self.reason = reason
            self.tripped_at = now

    def reset(self):
        self.active = False
        self.reason = None
        self.tripped_at = None


def preflight_ready(lat, heading):
    """Return True once the GPS fix and the heading have both reported (legacy check)."""
    return lat != 0.0 and heading != 0.0
