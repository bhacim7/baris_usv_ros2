import math
import time

import config as cfg


# Raw YOLO/data.yaml class id -> ProtoEnum colour. Single source of truth so the voted
# class id and the reported colour can never drift apart.
CID_TO_COLOR = {
    0: 3,  # ProtoEnum.COLOR_RED
    1: 1,  # ProtoEnum.COLOR_YELLOW
    2: 2,  # ProtoEnum.COLOR_BLACK
    3: 5,  # ProtoEnum.COLOR_ORANGE
    4: 4,  # ProtoEnum.COLOR_GREEN
}


class ObjectMemoryManager:
    def __init__(self):
        # Format: [{'id': 1, 'lat': 0.0, 'lon': 0.0, 'type': 0, 'color': 0, 'last_seen': 0.0, 'v_lat': 0.0, 'v_lon': 0.0}]
        self.memory = []
        self.id_counter = 1
        self.MERGE_DISTANCE = 2.5  # 2.5 meters merge distance

        # How long a track survives without a sighting.
        #
        # This was a hardcoded 5.0 s, which is shorter than a buoy routinely spends out of
        # frame. Every time one came back it was allocated a NEW id and its position
        # re-converged to a slightly different answer - measured 292 tracks for the 25-30
        # buoys of the 2026-08-12 run. A* re-reads the map every planning cycle, so each of
        # those 292 recycles was a chance for an obstacle to jump ~0.5 m and for the planner
        # to change which side it passes on.
        #
        # Only raise this together with the velocity guards below. Extrapolating a noisy
        # velocity over 20 s throws the predicted position metres away from the buoy, the
        # match fails anyway, and the stale track lingers as a phantom for the full window -
        # strictly worse than the 5 s it replaced.
        self.memory_s = float(getattr(cfg, 'OBJECT_MEMORY_S', 20.0))

        # Velocity is estimated as (position change / dt). At the 10 Hz publish rate dt is
        # ~0.1 s, so the ~0.2 m position noise divides up into an apparent 2 m/s - on a buoy
        # that is anchored to the seabed. Smoothing damps it but leaves 0.3-0.5 m/s of pure
        # noise standing. Requiring a longer baseline stops dividing by a tiny number, and
        # capping how far ahead that velocity is projected bounds the damage when it is
        # wrong. Genuinely moving objects keep a second of prediction, which is most of the
        # benefit; anchored ones can no longer wander off on noise.
        self.vel_min_dt_s = float(getattr(cfg, 'OBJECT_VEL_MIN_DT_S', 0.5))
        self.vel_max_predict_s = float(getattr(cfg, 'OBJECT_VEL_MAX_PREDICT_S', 1.0))

        # Beyond this range a sighting is not trusted to place a buoy on the map.
        #
        # The projected position is (boat GPS) + (distance) at (heading + pixel offset), so a
        # heading error becomes a TANGENTIAL position error that grows linearly with range.
        # Measured on the 2026-08-17 run by taking the principal axis of each buoy's sighting
        # cloud and comparing it with the line of sight: 6 of 9 orange clouds were elongated
        # ACROSS the line of sight (median 73.5 deg, 2.4x elongation, 0.68 m at 1 sigma), not
        # along it. So the dominant error is heading, not ZED depth.
        #
        # What that costs at range, at the measured ~2.6 deg:
        #     5 m -> 0.23 m     12 m -> 0.55 m     20 m -> 0.91 m     30 m -> 1.36 m
        # Two encounters differ by roughly twice that, so buoys seen far away land 2-3 m
        # apart - past the 2.5 m merge radius - and one real buoy becomes several tracks. That
        # is why the run recorded 33 orange "buoys" whose median nearest-neighbour spacing was
        # 2.91 m, and why 6 of them sat within hull-contact distance of the boat's own track:
        # positions it is not physically possible for a buoy to occupy.
        #
        # Yellow, seen at 4-11 m, stayed under the merge radius and produced 7 clean tracks
        # from the same code. The gate puts orange into that same regime.
        #
        # This also improves ACCURACY rather than merely tidying the count: the stored
        # position stops being an average that includes 30 m sightings with 1.4 m of error.
        self.map_max_range_m = float(getattr(cfg, 'MAP_MAX_RANGE_M', 12.0))

    def update_and_get_id(self, lat, lon, obj_type, color, cid=None, cx=None, cy=None, obj_dist=None, area=None):
        """
        cx/cy/obj_dist/area are PER-FRAME (transient) values from the current detection.
        They are refreshed on every sighting so Task3's visual servoing always reads the
        latest pixel/distance data, while lat/lon/id remain the persistent GPS memory.

        Colour (and therefore `cid`) is a VOTED attribute, not a matching key.

        Matching used to require `obj['color'] == color` as well as position and type. In
        backlit water a single frame where YOLO calls a yellow buoy orange (or vice versa)
        then fails to match, allocates a NEW id, and leaves BOTH entries alive for the full
        5 s memory window. Task 2's costmap filter accepts cid 1 and 3, so one real buoy
        became two overlapping obstacles a couple of metres apart - which is exactly the
        smeared cloud visible in final_costmap.png, and exactly the kind of jitter that
        makes A* flip its avoidance side between replans.

        Matching on position + type and voting on colour keeps one track per buoy and lets
        the colour settle to whatever the detector says most of the time.
        """
        current_time = time.time()
        best_match = None
        min_dist = float('inf')

        # Drop tracks that have not been seen for memory_s (see __init__).
        self.memory = [obj for obj in self.memory
                       if (current_time - obj['last_seen']) < self.memory_s]

        for obj in self.memory:
            # Predict current position based on velocity, but never project further than
            # vel_max_predict_s. Beyond that the term is dominated by velocity noise rather
            # than by motion, and with a 20 s memory an unclamped projection puts the buoy
            # metres from where it is - so the match fails and the track is duplicated.
            dt = current_time - obj['last_seen']
            dt_pred = min(dt, self.vel_max_predict_s)
            pred_lat = obj['lat'] + (obj['v_lat'] * dt_pred)
            pred_lon = obj['lon'] + (obj['v_lon'] * dt_pred)

            # Calculate distance to predicted position
            dy = (pred_lat - lat) * 111139
            dx = (pred_lon - lon) * 85000
            dist = math.sqrt(dx * dx + dy * dy)

            # Position + type only - colour is voted below, never a matching key.
            if dist < self.MERGE_DISTANCE and obj['type'] == obj_type:
                if dist < min_dist:
                    min_dist = dist
                    best_match = obj

        # A sighting from beyond map_max_range_m may keep a track ALIVE and refresh the
        # per-frame fields Task 3 steers on, but it may not move the stored position, vote on
        # the colour, or count towards the corridor's sighting confirmation. Task 3's visual
        # servoing works off 'cx' in pixels and never reads lat/lon, so it is unaffected by
        # this; the Task 2 costmap is exactly what needs protecting from it.
        near = (obj_dist is None) or (float(obj_dist) <= self.map_max_range_m)

        if best_match:
            # Update velocity, but only over a long enough baseline to mean anything.
            # dt used to be whatever the frame interval happened to be (~0.1 s), and
            # dividing the position noise by that manufactured metres per second of motion
            # for an anchored buoy.
            dt = current_time - best_match['last_seen']
            if near and dt >= self.vel_min_dt_s:
                best_match['v_lat'] = best_match['v_lat'] * 0.8 + ((lat - best_match['lat']) / dt) * 0.2
                best_match['v_lon'] = best_match['v_lon'] * 0.8 + ((lon - best_match['lon']) / dt) * 0.2

            if near:
                if best_match.get('pos_ok'):
                    # Smooth position (Alpha filter)
                    best_match['lat'] = best_match['lat'] * 0.8 + lat * 0.2
                    best_match['lon'] = best_match['lon'] * 0.8 + lon * 0.2
                else:
                    # First trustworthy sighting of a track that was created from a distant
                    # one. Take the new measurement outright instead of easing towards it -
                    # the stored position is a bad projection and averaging it in only drags
                    # the good measurement backwards.
                    best_match['lat'] = lat
                    best_match['lon'] = lon
                    best_match['pos_ok'] = True
            best_match['last_seen'] = current_time
            # Sighting count. nav_process only lets a buoy constrain the Task 2 corridor
            # once it has been seen a few times - a spurious boundary buoy narrows the
            # corridor, and one that is too narrow is as unhelpful as none at all.
            if near:
                best_match['seen'] = best_match.get('seen', 1) + 1

            # Colour vote: the winning class id decides both 'cid' and 'color'.
            if cid is not None and near:
                votes = best_match.setdefault('cid_votes', {})
                votes[cid] = votes.get(cid, 0) + 1
                winner = max(votes, key=votes.get)
                best_match['cid'] = winner
                best_match['color'] = CID_TO_COLOR.get(winner, best_match.get('color', 0))

            # Refresh transient per-frame fields (NOT smoothed - always the latest reading)
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
                # False = this position came from a sighting too far away to trust. The track
                # exists (so Task 3 can see the object) but consumers that place it on a map
                # must skip it until a close sighting arrives and sets this True.
                'pos_ok': near,
                'cx': cx,
                'cy': cy,
                'dist': obj_dist,
                'area': area,
            })
            return new_id, lat, lon
