"""
Obstacle-aware paths for the RoboBoat moves (phase 6 R3.2c): A* on a visibility graph.

DriveTo's grid A* (usv_core.planner) grows every landmark by the robot radius and a 0.5 m
margin with a square kernel: ~1.0 m round a channel buoy, which leaves ~0.5 m of a 2.5 m
gate, and it knows nothing of the channel, so it takes the shortest way, round the outside
(t2b_cl25's return). Here instead:

  - obstacles are circles and walls (segments), each with its own clearance: how close the
    hull's centre line may come to its centre (circle) or its line (wall). For a buoy that
    is its radius + half the beam + a margin; exact, no grid and no square kernel
  - the graph's nodes are the start, the goal and NODE_RING points round every circle and
    every wall end, NODE_EXTRA_M outside its clearance; an edge is a straight segment that
    keeps every clearance, checked only when A* reaches it
  - A* (edge length, straight-line heuristic) finds the shortest path through them. Given
    the path followed so far (prefer), every node further than STICK_M from it costs
    SWITCH_COST_M more: going round the other side of an obstacle has to be that much
    shorter, so two near-equal ways do not swap from one plan to the next (the boat swung
    one way, then the other, and hit the buoy between)

A gate's walls close its sides: segments from each of its buoys outward along the gate's
line, so the only way past is between the buoys.

Pure geometry on map east / north metres, no numpy: plan() returns a list of points.
"""

import heapq
import math

NODE_RING = 8                 # points round each circle / wall end
NODE_EXTRA_M = 0.25           # how far outside the clearance those points sit
NEAR_M = 4.0                  # obstacles further than this from start-goal are left out
STICK_M = 0.6
SWITCH_COST_M = 1.0


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_dist(p, a, b):
    """Return the distance from point p to the segment a-b."""
    ax, ay = b[0] - a[0], b[1] - a[1]
    n2 = ax * ax + ay * ay
    if n2 == 0.0:
        return _dist(p, a)
    u = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / n2))
    return math.hypot(p[0] - a[0] - u * ax, p[1] - a[1] - u * ay)


def _nearest_on(p, a, b):
    """Return the point of the segment a-b nearest to p."""
    ax, ay = b[0] - a[0], b[1] - a[1]
    n2 = ax * ax + ay * ay
    u = 0.0 if n2 == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / n2))
    return (a[0] + u * ax, a[1] + u * ay)


def _away(a, b, c, r):
    """Return True when a is within r of c and the segment a-b heads away from c."""
    return _dist(a, c) < r and (b[0] - a[0]) * (a[0] - c[0]) + (b[1] - a[1]) * (a[1] - c[1]) >= 0


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def segment_segment_dist(a, b, c, d):
    """Return the distance between the segments a-b and c-d (0 when they cross)."""
    d1, d2 = _cross(c, d, a), _cross(c, d, b)
    d3, d4 = _cross(a, b, c), _cross(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4:
        return 0.0
    return min(point_segment_dist(a, c, d), point_segment_dist(b, c, d),
               point_segment_dist(c, a, b), point_segment_dist(d, a, b))


class Obstacles:
    """Circles [((x, y), clearance)] and walls [((x, y), (x, y), clearance)]."""

    def __init__(self, circles=(), walls=()):
        self.circles = list(circles)
        self.walls = list(walls)

    def near(self, a, b, reach=NEAR_M):
        """Return the obstacles that could matter for a path from a to b."""
        return Obstacles(
            [c for c in self.circles if point_segment_dist(c[0], a, b) <= reach + c[1]],
            [w for w in self.walls
             if segment_segment_dist(a, b, w[0], w[1]) <= reach + w[2]])

    def inside(self, p):
        """Return True when p breaks some clearance."""
        return any(_dist(p, c) < r for c, r in self.circles) or \
            any(point_segment_dist(p, w0, w1) < r for w0, w1, r in self.walls)

    def clear(self, a, b):
        """
        Return True when the segment a-b keeps every clearance.

        An obstacle whose clearance a already breaks (the boat pushed against a buoy) is
        ignored as long as the segment leads straight away from it (never nearer than at
        a), so the boat can get out.
        """
        for c, r in self.circles:
            if point_segment_dist(c, a, b) < r and not _away(a, b, c, r):
                return False
        for w0, w1, r in self.walls:
            if segment_segment_dist(a, b, w0, w1) < r and \
                    not _away(a, b, _nearest_on(a, w0, w1), r):
                return False
        return True

    def push_out(self, p):
        """Return p moved out of any circle whose clearance it breaks (a goal on a buoy)."""
        for c, r in self.circles:
            d = _dist(p, c)
            if d < r:
                if d < 1e-6:
                    return (c[0] + r + 0.05, c[1])
                k = (r + 0.05) / d
                p = (c[0] + (p[0] - c[0]) * k, c[1] + (p[1] - c[1]) * k)
        return p

    def nodes(self):
        """Return the graph's nodes round the obstacles, those breaking no clearance."""
        centres = [(c, r) for c, r in self.circles]
        for w0, w1, r in self.walls:
            centres += [(w0, r), (w1, r)]
        out = []
        for c, r in centres:
            rr = r + NODE_EXTRA_M
            for k in range(NODE_RING):
                a = 2.0 * math.pi * k / NODE_RING
                p = (c[0] + rr * math.cos(a), c[1] + rr * math.sin(a))
                if not self.inside(p):
                    out.append(p)
        return out


def polyline_dist(p, line):
    """Return the distance from p to a polyline (a list of points)."""
    if len(line) == 1:
        return _dist(p, line[0])
    return min(point_segment_dist(p, a, b) for a, b in zip(line, line[1:]))


def plan(start, goal, obstacles, prefer=None):
    """
    Return the shortest path start -> goal keeping every clearance, or None.

    The direct segment is tried first; then A* over the nodes round the obstacles near
    the way (every edge checked against all of them). The goal is pushed out of any circle
    it falls in first.
    """
    goal = obstacles.push_out(goal)
    if obstacles.clear(start, goal):
        return [start, goal]
    pts = [start, goal] + obstacles.near(start, goal).nodes()
    extra = [0.0, 0.0] + [SWITCH_COST_M if prefer and polyline_dist(p, prefer) > STICK_M
                          else 0.0 for p in pts[2:]]
    g = {0: 0.0}
    came = {}
    heap = [(_dist(start, goal), 0)]
    closed = set()
    while heap:
        _, u = heapq.heappop(heap)
        if u in closed:
            continue
        if u == 1:
            path = [pts[1]]
            while u in came:
                u = came[u]
                path.append(pts[u])
            return path[::-1]
        closed.add(u)
        for v in range(1, len(pts)):
            if v in closed or v == u:
                continue
            cost = g[u] + _dist(pts[u], pts[v]) + extra[v]
            if cost >= g.get(v, math.inf):
                continue
            if not obstacles.clear(pts[u], pts[v]):
                continue
            g[v], came[v] = cost, u
            heapq.heappush(heap, (cost + _dist(pts[v], goal), v))
    return None


def plan_through(points, obstacles, prefer=None):
    """Return a path through the given points in order, each leg by plan(), or None."""
    path = [points[0]]
    for b in points[1:]:
        leg = plan(path[-1], b, obstacles, prefer)
        if leg is None:
            return None
        path += leg[1:]
    return path
