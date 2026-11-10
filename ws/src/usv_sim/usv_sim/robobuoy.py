"""
RoboBuoy (RoboBoat 2027 Task 1 and Task 2): its size.

Its light is a usv_sim.lights.Light. ROS-free, unit tested (test/test_robobuoy.py).

Sizes from the RobotX 2026 task build guides (RoboBuoy build guide, Light Beacon
specifications; RoboBoat 2027 uses the same elements); what they leave open is ASSUMED:
  * four white coroplast side panels, 17 x 12 in, leaning in to a flat top (a frustum);
    the top width (4 in) is ASSUMED: the guide gives the panels, not the assembled shape
  * a black diamond on each face, about 8 in tall, starting 2.5 in up the panel; its width
    (0.75 of its height) is ASSUMED
  * under the panels a square PVC frame wrapped in 3 in pool noodles (two 38 in noodles
    mitred round it: about 19 in a side), half in the water (ASSUMED)
  * on top the Light Beacon: RGB LED strip round a 3.5 in OD PVC pipe, 5.25 in long, under
    a 10 in black sun hat

Light (RobotX 2026 Task 1): red and green FLASH, 1 s on and 1 s off; flashing blue marks
the ENTRY, solid blue the EXIT; off (black) is an obstacle passed on either side. The
camera reports the body and the light as one class (`beacon_red`, ..., `beacon_off`), as
Baris labels the dataset, so a flashing beacon is `beacon_off` half of the time.

Task 2 RoboBuoys carry a Color Indicator (red or green, the 2026 one) instead, and a number
1-3 on their faces (Primer drawing); the course markers carry a letter and nothing on top.
"""

import math

INCH = 0.0254
PANEL_BASE = 17.0 * INCH          # 0.432 m
PANEL_TOP = 4.0 * INCH            # ASSUMED
PANEL_SLANT = 12.0 * INCH         # 0.305 m along the face
PANEL_HEIGHT = math.sqrt(PANEL_SLANT ** 2 - ((PANEL_BASE - PANEL_TOP) / 2.0) ** 2)
NOODLE_D = 3.0 * INCH
FLOAT_SIDE = 19.0 * INCH          # outer side of the noodle square
PANEL_BOTTOM = NOODLE_D / 2.0     # panels stand on the noodles, half of which are under water
DIAMOND_START = 2.5 * INCH        # up the face, from its bottom edge
DIAMOND_LENGTH = 8.0 * INCH       # along the face
DIAMOND_WIDTH = 0.75 * DIAMOND_LENGTH     # ASSUMED
BEACON_RADIUS = 3.5 * INCH / 2.0
BEACON_LENGTH = 5.25 * INCH
FLANGE = 0.01                     # ASSUMED: printed flange under the pipe
HAT_RADIUS = 10.0 * INCH / 2.0
HAT_THICKNESS = 0.005
BEACON_BOTTOM = PANEL_BOTTOM + PANEL_HEIGHT + FLANGE
TOP = BEACON_BOTTOM + BEACON_LENGTH + HAT_THICKNESS      # ~0.45 m above the water
CAMERA_RADIUS = FLOAT_SIDE / 2.0  # what a box spans: the float frame at the water
# Task 2: a face's slope (from vertical) and where its middle is, for the number marks
FACE_TILT = math.atan2((PANEL_BASE - PANEL_TOP) / 2.0, PANEL_HEIGHT)
FACE_MID = ((PANEL_BASE + PANEL_TOP) / 4.0, PANEL_BOTTOM + PANEL_HEIGHT / 2.0)  # (out, z)
