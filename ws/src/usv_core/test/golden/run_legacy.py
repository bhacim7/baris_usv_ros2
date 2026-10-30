#!/usr/bin/env python3
"""Run golden scenarios through the legacy nav_worker and print what happened.

Usage (inside the container):
    python3 /ws/src/usv_core/test/golden/run_legacy.py               # every scenario
    python3 /ws/src/usv_core/test/golden/run_legacy.py golcuk_full   # one
    ... --save DIR                                                   # also write the records

The legacy loop prints its own log lines to stdout; they are captured here and only the
summary (state changes, closest buoy approaches, any legacy error lines) is shown.
"""

import contextlib
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from legacy_runner import run_legacy  # noqa: E402
from scenario import local, SCENARIOS  # noqa: E402


def closest_approaches(scenario, records):
    """Smallest distance from the boat's centre to each buoy over the run (m)."""
    origin = scenario.waypoints['T2_ZONE_ENTRY']
    pts = [local(b.lat, b.lon, *origin) + (b.cid, b.radius) for b in scenario.buoys]
    best = [math.inf] * len(pts)
    for r in records:
        e, n, _ = r['boat']
        for i, (be, bn, _, rad) in enumerate(pts):
            best[i] = min(best[i], math.hypot(be - e, bn - n) - rad)
    return [(i, pts[i][2], d) for i, d in enumerate(best)]


def summary(name, records):
    sc = SCENARIOS[name]
    print(f'== {name}: {len(records)} cycles, {records[-1]["t"] if records else 0:.2f} s')
    last = None
    for r in records:
        if r['task'] != last:
            e, n, h = r['boat']
            print(f'   t={r["t"]:7.2f}s  {r["task"]:<24} boat ({e:6.1f} E, {n:6.1f} N, {h:5.1f} deg)')
            last = r['task']
    pwms = sorted({r['pwm'] for r in records})
    print(f'   distinct PWM tuples: {len(pwms)}, last {records[-1]["pwm"] if records else None}')
    close = [c for c in closest_approaches(sc, records) if c[2] < 1.0]
    names = {0: 'red', 1: 'yellow', 2: 'black', 3: 'orange', 4: 'green'}
    if close:
        print('   passed within 1 m of: ' + ', '.join(
            f'#{i} {names[cid]} {d:+.2f} m' for i, cid, d in close))


def main():
    args = sys.argv[1:]
    save = None
    if '--save' in args:
        save = args[args.index('--save') + 1]
        args = [a for a in args if a not in ('--save', save)]
    names = args or list(SCENARIOS)
    for name in names:
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            records = run_legacy(SCENARIOS[name])
        summary(name, records)
        errors = [ln for ln in log.getvalue().splitlines() if 'ERROR' in ln or 'crashed' in ln]
        if errors:
            print('   legacy reported errors:')
            for ln in errors[:5]:
                print('     ' + ln)
        if save:
            os.makedirs(save, exist_ok=True)
            with open(os.path.join(save, name + '.json'), 'w') as f:
                json.dump(records, f)


if __name__ == '__main__':
    main()
