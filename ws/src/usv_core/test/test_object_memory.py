"""Port check for usv_core.object_memory against code extracted from legacy camera_process.py."""

import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'legacy'))

import config as legacy_config  # noqa: E402
import object_memory_extract as legacy  # noqa: E402
import pytest  # noqa: E402
from usv_core import object_memory  # noqa: E402
from usv_core.object_memory import (  # noqa: E402
    CID_TO_COLOR, DEFAULT_CONFIG, ObjectMemoryManager)

BUOY = 3   # ProtoEnum.OBJECT_BUOY
T1 = (40.8090735, 29.2622975)


class FakeClock:
    """Stands in for the `time` module inside a module under test."""

    def __init__(self, t=1000.0):
        self.t = t

    def time(self):
        return self.t


def sighting_sequence(seed, n=400):
    """Repeatable stream of noisy sightings of a few buoys, like a run past a buoy field."""
    rng = random.Random(seed)
    buoys = [(T1[0] + rng.uniform(-3e-4, 3e-4), T1[1] + rng.uniform(-3e-4, 3e-4),
              rng.choice([0, 1, 2, 3, 4])) for _ in range(8)]
    t = 1000.0
    seq = []
    for i in range(n):
        t += rng.choice([0.033, 0.1, 0.3, 0.7])
        if i == n // 2:
            t += 25.0                                   # longer than memory_s: all tracks expire
        lat, lon, true_cid = rng.choice(buoys)
        lat += rng.gauss(0.0, 2e-6)                     # ~0.2 m of position noise
        lon += rng.gauss(0.0, 2e-6)
        cid = true_cid if rng.random() < 0.85 else rng.choice([0, 1, 2, 3, 4])
        if rng.random() < 0.05:
            cid = None
        dist = rng.uniform(1.0, 18.0) if rng.random() > 0.05 else None
        seq.append({'t': t, 'lat': lat, 'lon': lon, 'color': CID_TO_COLOR.get(cid, 0),
                    'cid': cid, 'cx': rng.randint(0, 1279), 'cy': rng.randint(0, 719),
                    'dist': dist, 'area': rng.randint(100, 50000)})
    return seq


# --- Settings ------------------------------------------------------------------------

def test_default_config_matches_legacy():
    assert DEFAULT_CONFIG.memory_s == legacy_config.OBJECT_MEMORY_S
    assert DEFAULT_CONFIG.vel_min_dt_s == legacy_config.OBJECT_VEL_MIN_DT_S
    assert DEFAULT_CONFIG.vel_max_predict_s == legacy_config.OBJECT_VEL_MAX_PREDICT_S
    assert DEFAULT_CONFIG.map_max_range_m == legacy_config.MAP_MAX_RANGE_M
    assert DEFAULT_CONFIG.merge_distance_m == legacy.ObjectMemoryManager().MERGE_DISTANCE


def test_colour_table_matches_legacy():
    assert CID_TO_COLOR == legacy.CID_TO_COLOR


# --- Port equivalence ----------------------------------------------------------------

@pytest.mark.parametrize('seed', range(8))
def test_sequence_matches_legacy(seed, monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(legacy, 'time', clock)          # legacy reads time.time()
    old = legacy.ObjectMemoryManager()
    new = ObjectMemoryManager()
    for s in sighting_sequence(seed):
        clock.t = s['t']
        r_old = old.update_and_get_id(s['lat'], s['lon'], BUOY, s['color'], cid=s['cid'],
                                      cx=s['cx'], cy=s['cy'], obj_dist=s['dist'], area=s['area'])
        r_new = new.update_and_get_id(s['lat'], s['lon'], BUOY, s['color'], cid=s['cid'],
                                      cx=s['cx'], cy=s['cy'], obj_dist=s['dist'], area=s['area'],
                                      now=s['t'])
        assert r_new == r_old
    assert new.memory == old.memory
    assert new.id_counter == old.id_counter


def test_without_now_reads_the_clock(monkeypatch):
    monkeypatch.setattr(object_memory, 'time', FakeClock(1234.5))
    mem = ObjectMemoryManager()
    mem.update_and_get_id(*T1, BUOY, 5, cid=3, obj_dist=5.0)
    assert mem.memory[0]['last_seen'] == 1234.5


# --- Known behaviour ------------------------------------------------------------------

def test_repeated_sightings_make_one_track():
    mem = ObjectMemoryManager()
    for k in range(5):
        mem.update_and_get_id(*T1, BUOY, 5, cid=3, obj_dist=6.0, now=100.0 + k * 0.1)
    assert len(mem.memory) == 1
    assert mem.memory[0]['seen'] == 5
    assert mem.memory[0]['pos_ok'] is True


def test_distant_first_sighting_is_replaced_by_a_close_one():
    mem = ObjectMemoryManager()
    mem.update_and_get_id(T1[0], T1[1], BUOY, 5, cid=3, obj_dist=15.0, now=100.0)
    assert mem.memory[0]['pos_ok'] is False
    close = (T1[0] + 5e-6, T1[1])                       # ~0.6 m away: same track
    mem.update_and_get_id(*close, BUOY, 5, cid=3, obj_dist=8.0, now=100.5)
    track = mem.memory[0]
    assert track['pos_ok'] is True
    assert (track['lat'], track['lon']) == close        # taken outright, not averaged


def test_distant_sighting_neither_votes_nor_counts():
    mem = ObjectMemoryManager()
    mem.update_and_get_id(*T1, BUOY, 5, cid=3, obj_dist=5.0, now=100.0)
    mem.update_and_get_id(*T1, BUOY, 1, cid=1, obj_dist=14.0, now=100.1)
    track = mem.memory[0]
    assert track['cid_votes'] == {3: 1}
    assert track['seen'] == 1
    assert track['dist'] == 14.0                        # per-frame fields still refresh


def test_colour_is_voted_not_matched():
    mem = ObjectMemoryManager()
    for k, cid in enumerate([3, 3, 1, 3]):
        mem.update_and_get_id(*T1, BUOY, CID_TO_COLOR[cid], cid=cid, obj_dist=5.0, now=100.0 + k)
    assert len(mem.memory) == 1
    assert mem.memory[0]['cid'] == 3


def test_track_expires_after_memory_s():
    mem = ObjectMemoryManager()
    first, _, _ = mem.update_and_get_id(*T1, BUOY, 5, cid=3, obj_dist=5.0, now=100.0)
    again, _, _ = mem.update_and_get_id(*T1, BUOY, 5, cid=3, obj_dist=5.0, now=100.0 + 20.0)
    assert again != first


def test_short_baseline_gives_no_velocity():
    mem = ObjectMemoryManager()
    mem.update_and_get_id(*T1, BUOY, 5, cid=3, obj_dist=5.0, now=100.0)
    mem.update_and_get_id(T1[0] + 2e-6, T1[1], BUOY, 5, cid=3, obj_dist=5.0, now=100.1)
    assert mem.memory[0]['v_lat'] == 0.0
