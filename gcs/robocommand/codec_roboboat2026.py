"""
RoboBoat 2026 RoboCommand messages (msgs/roboboat2026/report.proto, RoboNation).

Turns what the GCS knows into Report messages and back (the fake server reads them):
  heartbeat  from a boat's telemetry as the GCS shows it (gcs.py LinkWorker._expand):
             MEVCUT_KONUM, spd, hdg, mod (1 = manual), relay (0 = motor power off ->
             KILLED), task (the boat's phase)
  report     a dict: {type: gate, gate: entry|exit|speed_start|speed_end, lat, lon},
             {type: object, object: boat|light_beacon|buoy, color, lat, lon, id, task},
             {type: delivery, color, lat, lon, kind: water|ball},
             {type: docking, dock, slip}
2026 has no messages from RoboCommand to the boats: command() always returns None.
The 2027 tasks have no TaskType here yet; the boat's phases map to the nearest 2026 ones.
"""

from robocommand.msgs.roboboat2026 import report_pb2 as pb

# The boat's run phase (usv_core.roboboat_run) -> 2026 TaskType
TASKS = {'T1_ENTRY': pb.TASK_ENTRY_EXIT, 'T1_EXIT': pb.TASK_ENTRY_EXIT,
         'RETURN': pb.TASK_ENTRY_EXIT, 'task2': pb.TASK_NAV_CHANNEL,
         'task3': pb.TASK_SPEED_CHALLENGE, 'task4': pb.TASK_OBJECT_DELIVERY,
         'task5': pb.TASK_DOCKING, 'WAIT_START': pb.TASK_NONE, 'FINISHED': pb.TASK_NONE}
GATES = {'entry': pb.GATE_ENTRY, 'exit': pb.GATE_EXIT, 'speed_start': pb.GATE_SPEED_START,
         'speed_end': pb.GATE_SPEED_END}
OBJECTS = {'boat': pb.OBJECT_BOAT, 'light_beacon': pb.OBJECT_LIGHT_BEACON,
           'buoy': pb.OBJECT_BUOY}
COLORS = {'yellow': pb.COLOR_YELLOW, 'black': pb.COLOR_BLACK, 'red': pb.COLOR_RED,
          'green': pb.COLOR_GREEN}
DELIVERIES = {'water': pb.DELIVERY_WATER, 'ball': pb.DELIVERY_BALL}


def _num(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class Codec:

    name = 'roboboat2026'

    def __init__(self, team_id, vehicle_ids):
        self.team_id = team_id
        self.vehicle_ids = vehicle_ids          # boat number -> 'S1', 'S2'

    def _report(self, boat, seq, stamp):
        r = pb.Report(team_id=self.team_id, vehicle_id=self.vehicle_ids.get(boat, f'S{boat}'),
                      seq=seq)
        secs = int(stamp)                       # split: stamp * 1e9 loses nanoseconds
        r.sent_at.seconds, r.sent_at.nanos = secs, min(999999999, round((stamp - secs) * 1e9))
        return r

    def heartbeat(self, boat, seq, stamp, tel):
        r = self._report(boat, seq, stamp)
        hb = r.heartbeat
        if int(_num(tel.get('relay'), -1)) == 0:
            hb.state = pb.STATE_KILLED
        elif 'mod' in tel:
            hb.state = pb.STATE_MANUAL if int(_num(tel['mod'], 1)) else pb.STATE_AUTO
        else:
            hb.state = pb.STATE_UNKNOWN
        pos = tel.get('MEVCUT_KONUM') or {}
        hb.position.latitude = _num(pos.get('lat'))
        hb.position.longitude = _num(pos.get('lon'))
        hb.spd_mps = _num(tel.get('spd'))
        hb.heading_deg = _num(tel.get('hdg')) % 360.0
        hb.current_task = TASKS.get(str(tel.get('task', '')), pb.TASK_UNKNOWN)
        return r.SerializeToString()

    def report(self, boat, seq, stamp, rep):
        r = self._report(boat, seq, stamp)
        kind = rep.get('type')
        if kind == 'gate':
            r.gate_pass.type = GATES.get(rep.get('gate'), pb.GATE_UNKNOWN)
            r.gate_pass.position.latitude = _num(rep.get('lat'))
            r.gate_pass.position.longitude = _num(rep.get('lon'))
        elif kind == 'object':
            od = r.object_detected
            od.object_type = OBJECTS.get(rep.get('object'), pb.OBJECT_UNKNOWN)
            od.color = COLORS.get(rep.get('color'), pb.COLOR_UNKNOWN)
            od.position.latitude = _num(rep.get('lat'))
            od.position.longitude = _num(rep.get('lon'))
            od.object_id = int(_num(rep.get('id'), 0))
            od.task_context = TASKS.get(str(rep.get('task', '')), pb.TASK_UNKNOWN)
        elif kind == 'delivery':
            d = r.object_delivery
            d.vessel_color = COLORS.get(rep.get('color'), pb.COLOR_UNKNOWN)
            d.position.latitude = _num(rep.get('lat'))
            d.position.longitude = _num(rep.get('lon'))
            d.delivery_type = DELIVERIES.get(rep.get('kind'), pb.DELIVERY_UNKNOWN)
        elif kind == 'docking':
            r.docking.dock = str(rep.get('dock', ''))
            r.docking.slip = str(rep.get('slip', ''))
        else:
            raise ValueError(f'unknown report type {kind!r}')
        return r.SerializeToString()

    def command(self, payload):
        """Return a task for a boat from a RoboCommand message: none exist in 2026."""
        return None

    @staticmethod
    def decode(payload):
        """Return the Report in a payload (for the fake server and tests)."""
        return pb.Report.FromString(payload)
