#!/usr/bin/env python3
"""Generate a light hull mesh (OBJ) for one catamaran hull of usv_boat from the team's STL.

Input: ROBO27_MAIN_BODY_SIM.STL, the SolidWorks export of the boat body before the build
(binary STL, millimetres, translated to positive space). It is ONE closed shell: both hulls
plus the old central box on top. Only the hull shape is used; the box was replaced on the
real boat (see the photos) and model.sdf draws today's parts separately.

Method: slice the right-hand hull at many stations along x, take the convex outline of each
slice below the deck (the hull sections are convex: flat bottom, rounded chines, sides
flaring out to the deck), resample every outline to the same number of points (deck
corners pinned) and loft the rings into a closed mesh. ~160 000 triangles in, ~8 000 out.

Measured from the STL (mm): length 1181.6, deck 320 above the keel, widest 247 at 0.6 L,
flat bottom from x ~60 to ~1000, raked stern, bottom rising to 200 above the keel at the bow.
The hulls are symmetric about their own centre lines, 600 apart in the STL. model.sdf puts
them at y = +/-0.26 (technical drawing; overall beam 0.52 + 0.247 = 0.77, drawing 0.78).

Output frame: hull-local, x forward, y left, z up, origin on the hull centre line half way
along the hull, deck at z = +0.19. That puts the keel at z = -0.13: the STL hulls displace
45 kg of sea water (1025 kg/m^3) at a draft of ~0.13 m (42.7 kg fresh water), in line with
the ANSYS draft of 0.11-0.13 m.

Usage (pure Python, no numpy; runs on the host or in the container):
    python3 make_hull_mesh.py ROBO27_MAIN_BODY_SIM.STL OUTPUT.obj
"""

import math
import os
import struct
import sys

Y_HULL = 131.75        # right hull centre line in the STL (mm); the left one is at +600
Y_HALF_SPAN = 135.0    # slice points further than this from Y_HULL belong to something else
Y_SPLIT = 430.0        # triangles entirely below this y belong to the right hull
Z_DECK = 334.9         # hull deck in the STL (mm); the old central box sits above it
X_MID = 591.74         # half way along the hull in the STL (mm)
Z_DECK_OUT = 0.19      # deck height in the output frame (m)
RING_DECK = 6          # section points on the deck line
RING_SIDES = 50        # section points on the sides and bottom
RAYS = RING_DECK + RING_SIDES
# Stations (mm): dense at the raked stern and at the bow where the sections change fast
STATIONS = ([1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 14.0, 19.0, 25.0, 32.0, 40.0, 50.0, 60.0]
            + [75.0 + 25.0 * i for i in range(37)]
            + [1000.0 + 10.0 * i for i in range(1, 17)]
            + [1165.0, 1170.0, 1174.0, 1177.0, 1179.0, 1180.5, 1181.5])


def load_right_hull(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    if len(data) != 84 + 50 * n:
        sys.exit('Not a binary STL (size does not match the triangle count)')
    tris = []
    for i in range(n):
        v = struct.unpack('<9f', data[84 + 50 * i + 12:84 + 50 * i + 48])
        t = ((v[0], v[1], v[2]), (v[3], v[4], v[5]), (v[6], v[7], v[8]))
        if max(p[1] for p in t) < Y_SPLIT:
            tris.append(t)
    tris.sort(key=lambda t: min(p[0] for p in t))
    return tris


def slice_points(tris, x):
    pts = []
    for t in tris:
        if min(p[0] for p in t) > x:
            break                       # sorted by min x: nothing further can cross x
        if max(p[0] for p in t) < x:
            continue
        for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            if a[0] != b[0] and (a[0] - x) * (b[0] - x) <= 0:
                s = (x - a[0]) / (b[0] - a[0])
                y = a[1] + s * (b[1] - a[1])
                z = a[2] + s * (b[2] - a[2])
                if z <= Z_DECK + 0.01 and abs(y - Y_HULL) < Y_HALF_SPAN:
                    pts.append((y, min(z, Z_DECK)))
    return pts


def convex_outline(pts):
    """Counter-clockwise convex hull (monotone chain)."""
    pts = sorted(set((round(y, 3), round(z, 3)) for y, z in pts))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def resample(path, count):
    """`count` points evenly spaced by arc length along an open polyline, first point
    included, last point excluded."""
    seg = [math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)]
    total = sum(seg)
    out, i, done = [], 0, 0.0
    for k in range(count):
        s = total * k / count
        while i < len(seg) - 1 and done + seg[i] < s:
            done += seg[i]
            i += 1
        f = 0.0 if seg[i] == 0 else (s - done) / seg[i]
        a, b = path[i], path[i + 1]
        out.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
    return out


def ring(outline):
    """RING_DECK + RING_SIDES points on the outline. The two deck corners are always ring
    points, so the sharp deck edge lofts cleanly; the rest is spaced by arc length, so point
    k of every ring sits at the same place around the hull (no twist in the loft)."""
    z_top = max(p[1] for p in outline)
    top = [i for i, p in enumerate(outline) if p[1] >= z_top - 0.5]
    right = max(top, key=lambda i: outline[i][0])
    left = min(top, key=lambda i: outline[i][0])
    n = len(outline)

    def walk(start, end):               # counter-clockwise, start..end inclusive
        out, i = [outline[start]], start
        while i != end:
            i = (i + 1) % n
            out.append(outline[i])
        return out

    # Counter-clockwise from the right deck corner runs along the deck to the left corner,
    # then down the left side, along the bottom and up the right side.
    return resample(walk(right, left), RING_DECK) + resample(walk(left, right), RING_SIDES)


def to_output(x, y, z):
    return ((x - X_MID) / 1000.0, (y - Y_HULL) / 1000.0, (z - Z_DECK) / 1000.0 + Z_DECK_OUT)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    tris = load_right_hull(sys.argv[1])

    rings = []
    for x in STATIONS:
        outline = convex_outline(slice_points(tris, x))
        if len(outline) < 3:
            sys.exit(f'No hull section at x = {x} mm')
        rings.append([to_output(x, y, z) for y, z in ring(outline)])

    verts = [v for r in rings for v in r]

    def vid(i, k):
        return i * RAYS + (k % RAYS)

    faces = []
    for i in range(len(rings) - 1):
        for k in range(RAYS):
            a, b, c, d = vid(i, k), vid(i, k + 1), vid(i + 1, k + 1), vid(i + 1, k)
            faces += [(a, b, c), (a, c, d)]
    caps = []
    for i, outward in ((0, -1.0), (len(rings) - 1, 1.0)):
        verts.append(tuple(sum(p[j] for p in rings[i]) / RAYS for j in range(3)))
        centre = len(verts) - 1
        caps += [((centre, vid(i, k), vid(i, k + 1)), outward) for k in range(RAYS)]

    # Section centres in the output frame, for the outward-normal test of the side faces
    centres = [(r[0][0], sum(p[2] for p in r) / RAYS) for r in rings]

    def oriented(tri, outward_x=None):
        p0, p1, p2 = (verts[j] for j in tri)
        e1 = [p1[j] - p0[j] for j in range(3)]
        e2 = [p2[j] - p0[j] for j in range(3)]
        n = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
             e1[0] * e2[1] - e1[1] * e2[0])
        if outward_x is not None:
            ok = n[0] * outward_x > 0
        else:
            cx = (p0[0] + p1[0] + p2[0]) / 3.0
            cz = min(centres, key=lambda c: abs(c[0] - cx))[1]
            m = ((p0[1] + p1[1] + p2[1]) / 3.0, (p0[2] + p1[2] + p2[2]) / 3.0 - cz)
            ok = n[1] * m[0] + n[2] * m[1] > 0
        if not ok:
            tri = (tri[0], tri[2], tri[1])
            n = (-n[0], -n[1], -n[2])
        length = math.sqrt(sum(c * c for c in n))
        return tri, (tuple(c / length for c in n) if length > 1e-15 else (0.0, 0.0, 1.0))

    out = [oriented(t) for t in faces] + [oriented(t, o) for t, o in caps]
    out = [(t, n) for t, n in out if len(set(t)) == 3]

    path = sys.argv[2]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write('# usv_boat hull from ROBO27_MAIN_BODY_SIM.STL, generated by make_hull_mesh.py\n')
        for x, y, z in verts:
            f.write(f'v {x:.5f} {y:.5f} {z:.5f}\n')
        for _, n in out:
            f.write(f'vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}\n')
        for j, (t, _) in enumerate(out, start=1):
            a, b, c = (i + 1 for i in t)
            f.write(f'f {a}//{j} {b}//{j} {c}//{j}\n')

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    print(f'{path}: {len(verts)} vertices, {len(out)} triangles; '
          f'x {min(xs):.3f}..{max(xs):.3f}, y {min(ys):.3f}..{max(ys):.3f}, '
          f'z {min(zs):.3f}..{max(zs):.3f} m')


if __name__ == '__main__':
    main()
