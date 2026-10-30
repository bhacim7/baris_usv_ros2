"""
Buoy track memory, ported from legacy core/camera_process.py (commit 5df3f5b).

What changed in the port, and only this:
  * Settings come from ObjectMemoryConfig instead of the global `config` module. Defaults
    are the legacy config.py values.
  * update_and_get_id() accepts an optional `now` (seconds). Without it the clock is read
    with time.time() exactly as before; with it, tests are repeatable and simulation time
    can be used.

The method body is otherwise unchanged. test/test_object_memory.py replays sighting
sequences through this class and the frozen legacy copy and requires identical results.
"""

from dataclasses import dataclass
import math
import time

# Raw YOLO/data.yaml class id -> ProtoEnum colour. Single source of truth so the voted
# class id and the reported colour can never drift apart.
CID_TO_COLOR = {
    0: 3,  # ProtoEnum.COLOR_RED
    1: 1,  # ProtoEnum.COLOR_YELLOW
    2: 2,  # ProtoEnum.COLOR_BLACK
    3: 5,  # ProtoEnum.COLOR_ORANGE
    4: 4,  # ProtoEnum.COLOR_GREEN
}


@dataclass(frozen=True)
class ObjectMemoryConfig:
    """Defaults are the legacy values at commit 5df3f5b."""

    merge_distance_m: float = 2.5           # hardcoded in the legacy class
    memory_s: float = 20.0                  # OBJECT_MEMORY_S
    vel_min_dt_s: float = 0.5               # OBJECT_VEL_MIN_DT_S
    vel_max_predict_s: float = 1.0          # OBJECT_VEL_MAX_PREDICT_S
    map_max_range_m: float = 12.0           # MAP_MAX_RANGE_M


DEFAULT_CONFIG = ObjectMemoryConfig()


class ObjectMemoryManager:
    """
    One persistent track per physical buoy.

    Matching is by predicted position and type only; colour is voted. Sightings beyond
    map_max_range_m keep a track alive and refresh its per-frame fields (cx, cy, dist, area)
    but may not move its position, vote on colour or count as a sighting. Tracks unseen for
    memory_s are dropped. See the legacy config.py comments for the field measurements
    behind each value.
    """

    def __init__(self, cfg=DEFAULT_CONFIG):
        # Format: [{'id': 1, 'lat': 0.0, 'lon': 0.0, 'type': 0, 'color': 0, 'last_seen': 0.0, ...}]
        self.memory = []
        self.id_counter = 1
        self.MERGE_DISTANCE = cfg.merge_distance_m
        self.memory_s = float(cfg.memory_s)
        self.vel_min_dt_s = float(cfg.vel_min_dt_s)
        self.vel_max_predict_s = float(cfg.vel_max_predict_s)
        self.map_max_range_m = float(cfg.map_max_range_m)

    def update_and_get_id(self, lat, lon, obj_type, color, cid=None, cx=None, cy=None,
                          obj_dist=None, area=None, now=None):
        """
        Match one sighting to a track (or open a new one). Returns (id, lat, lon).

        cx/cy/obj_dist/area are per-frame values and are always refreshed; lat/lon/id are
        the persistent memory.
        """
        current_time = time.time() if now is None else now
        best_match = None
        min_dist = float('inf')

        # Drop tracks that have not been seen for memory_s.
        self.memory = [obj for obj in self.memory
                       if (current_time - obj['last_seen']) < self.memory_s]

        for obj in self.memory:
            # Predict current position from velocity, never further than vel_max_predict_s.
            dt = current_time - obj['last_seen']
            dt_pred = min(dt, self.vel_max_predict_s)
            pred_lat = obj['lat'] + (obj['v_lat'] * dt_pred)
            pred_lon = obj['lon'] + (obj['v_lon'] * dt_pred)

            # Distance to the predicted position (legacy flat-earth factors kept as they are)
            dy = (pred_lat - lat) * 111139
            dx = (pred_lon - lon) * 85000
            dist = math.sqrt(dx * dx + dy * dy)

            # Position + type only - colour is voted below, never a matching key.
            if dist < self.MERGE_DISTANCE and obj['type'] == obj_type:
                if dist < min_dist:
                    min_dist = dist
                    best_match = obj

        near = (obj_dist is None) or (float(obj_dist) <= self.map_max_range_m)

        if best_match:
            # Velocity only over a long enough baseline to mean anything.
            dt = current_time - best_match['last_seen']
            if near and dt >= self.vel_min_dt_s:
                best_match['v_lat'] = \
                    best_match['v_lat'] * 0.8 + ((lat - best_match['lat']) / dt) * 0.2
                best_match['v_lon'] = \
                    best_match['v_lon'] * 0.8 + ((lon - best_match['lon']) / dt) * 0.2

            if near:
                if best_match.get('pos_ok'):
                    # Smooth position (alpha filter)
                    best_match['lat'] = best_match['lat'] * 0.8 + lat * 0.2
                    best_match['lon'] = best_match['lon'] * 0.8 + lon * 0.2
                else:
                    # First trustworthy sighting of a track created from a distant one:
                    # take it outright instead of averaging in the bad projection.
                    best_match['lat'] = lat
                    best_match['lon'] = lon
                    best_match['pos_ok'] = True
            best_match['last_seen'] = current_time
            if near:
                best_match['seen'] = best_match.get('seen', 1) + 1

            # Colour vote: the winning class id decides both 'cid' and 'color'.
            if cid is not None and near:
                votes = best_match.setdefault('cid_votes', {})
                votes[cid] = votes.get(cid, 0) + 1
                winner = max(votes, key=votes.get)
                best_match['cid'] = winner
                best_match['color'] = CID_TO_COLOR.get(winner, best_match.get('color', 0))

            # Per-frame fields: always the latest reading, never smoothed
            best_match['cx'] = cx
            best_match['cy'] = cy
            best_match['dist'] = obj_dist
            best_match['area'] = area

            return best_match['id'], best_match['lat'], best_match['lon']
        else:
            new_id = self.id_counter
            self.id_counter += 1
            self.memory.append({
                'id': new_id,
                'lat': lat,
                'lon': lon,
                'type': obj_type,
                'color': color,
                'last_seen': current_time,
                'seen': 1,
                'v_lat': 0.0,
                'v_lon': 0.0,
                'cid': cid,
                'cid_votes': {cid: 1} if cid is not None else {},
                # False = position from a sighting too far away to trust; map consumers skip it.
                'pos_ok': near,
                'cx': cx,
                'cy': cy,
                'dist': obj_dist,
                'area': area,
            })
            return new_id, lat, lon
