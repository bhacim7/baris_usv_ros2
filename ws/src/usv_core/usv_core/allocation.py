"""
Which boat gets a task from RoboCommand (RoboBoat 2027, step 6e): the GCS decides, alone.

The judges' tasks reach the GCS (RoboCommand over the tent's Ethernet); the GCS picks a boat
and sends it the task over the GCS link (Ubiquiti + RFD). The operator does not touch it
(decided with Baris, 2026-11-11: inside the autonomous run). Always the same rules, so the
same situation always gives the same boat:

  1. the boat RoboCommand named, if it did (2027 may address a vehicle); refused if it
     cannot do the task or is not heard
  2. else among the boats the GCS hears (telemetry fresh), in AUTO, that can do it (the
     task's `needs`, usv_core.mission.competition_factories, against the boat's profile:
     water -> Marine):
       a boat with nothing to do (HOME, WAIT_START) before a busy one,
       then the one with fewer tasks still to come,
       then the lower number (Azure 1 before Marine 2)
  3. no boat: refused, with the reason (logged by the GCS)

A list of tasks (a Mission Badge: tasks for both boats together) is shared out one by one,
each assignment counted as a task to come for the next. ROS-free, unit tested
(test/test_allocation.py); the GCS (gcs/gcs.py) uses it.
"""

from dataclasses import dataclass, field

from usv_core.mission import competition_factories, HOME, WAIT_START

FRESH_S = 3.0
IDLE = (HOME, WAIT_START)


@dataclass
class BoatState:
    number: int
    name: str
    capabilities: tuple = ()
    heard_s: float = None             # seconds since its last telemetry; None = never
    manual: bool = True
    task: str = ''
    upcoming: list = field(default_factory=list)


def needs(task):
    """Return the capability required by a task (one of usv_core.boat.CAPABILITIES) or None."""
    factory = competition_factories().get(task)
    return getattr(factory, 'needs', None) if factory else None


def why_not(boat, task):
    """Return why a boat cannot take a task now, or '' if it can."""
    if boat.heard_s is None or boat.heard_s > FRESH_S:
        return f'{boat.name} not heard'
    if boat.manual:
        return f'{boat.name} in MANUAL'
    need = needs(task)
    if need and need not in boat.capabilities:
        return f'{boat.name} cannot do {need}'
    return ''


def allocate(task, boats, named=None, extra=None):
    """
    Return (boat number, '') for a task, or (None, reason).

    boats: BoatState list; named: the boat number RoboCommand gave, if any; extra: {number:
    tasks already given out in this round} (badges), counted as tasks to come.
    """
    extra = extra or {}
    if task not in competition_factories():
        return None, f'unknown task {task}'
    if named is not None:
        boat = next((b for b in boats if b.number == named), None)
        if boat is None:
            return None, f'no boat number {named}'
        reason = why_not(boat, task)
        return (None, reason) if reason else (boat.number, '')
    reasons, able = [], []
    for b in boats:
        reason = why_not(b, task)
        if reason:
            reasons.append(reason)
        else:
            able.append(b)
    if not able:
        return None, '; '.join(reasons) or 'no boats'

    def rank(b):
        busy = b.task not in IDLE or extra.get(b.number, 0) > 0
        return (busy, len(b.upcoming) + extra.get(b.number, 0), b.number)
    return min(able, key=rank).number, ''


def share_out(tasks, boats):
    """Return [(task, boat number or None, reason)] for a list of tasks, in order."""
    given, out = {}, []
    for task in tasks:
        number, reason = allocate(task, boats, extra=given)
        if number is not None:
            given[number] = given.get(number, 0) + 1
        out.append((task, number, reason))
    return out
