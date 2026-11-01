"""
The local costmap grid the A* planner (usv_core.planner) runs on; shared by both profiles.

Ported from legacy core/nav_process.py sections D and G (commit 5df3f5b): a robot-centred
grid of crop_radius_m around the boat, and the crop the planner gets. What is drawn on it is
the profile's business: usv_core.teknofest_costmap.TeknofestCostmap draws the legacy object
memory, usv_core.roboboat_drive the world map's landmarks.

Frame (planner's, inherited): world x = North, y = East, metres. Grid values: 127
unknown/free, 0 obstacle (planner.get_inflated_nav_map inflates them).
"""

import numpy as np


class LocalCostmap:
    """Section D: robot-centred grid of A_STAR_CROP_RADIUS_M around the boat."""

    def __init__(self, res_m_per_px, crop_radius_m, buoy_radius_m):
        self.res = res_m_per_px
        self.crop_radius_m = crop_radius_m
        size = int((crop_radius_m * 2.0) / res_m_per_px)
        self.img = np.full((size, size), 127, dtype=np.uint8)
        self.size_px = (size, size)            # (width, height)
        self.center_m = (0.0, 0.0)
        # Physical footprint only; the safety margin is the planner's inflation
        self.buoy_radius_px = max(1, int(round(buoy_radius_m / res_m_per_px)))

    def world_to_pixel(self, world_x, world_y):
        w, h = self.size_px
        dx_m = world_x - self.center_m[0]
        dy_m = world_y - self.center_m[1]
        px = int((w // 2) + (dx_m / self.res))
        py = int((h // 2) - (dy_m / self.res))
        if 0 <= px < w and 0 <= py < h:
            return (px, py)
        return None

    def crop(self, robot_x, robot_y):
        """Cut the A* window around the boat: (grid, centre in metres, (width, height))."""
        crop_radius_px = int(self.crop_radius_m / self.res)
        map_w, map_h = self.size_px
        cw, ch = map_w // 2, map_h // 2
        rx_px = int(cw + ((robot_x - self.center_m[0]) / self.res))
        ry_px = int(ch - ((robot_y - self.center_m[1]) / self.res))
        x_min = max(0, rx_px - crop_radius_px)
        x_max = min(map_w, rx_px + crop_radius_px)
        y_min = max(0, ry_px - crop_radius_px)
        y_max = min(map_h, ry_px + crop_radius_px)
        cropped = self.img[y_min:y_max, x_min:x_max]
        center = (self.center_m[0] + ((x_min + x_max) / 2 - cw) * self.res,
                  self.center_m[1] - ((y_min + y_max) / 2 - ch) * self.res)
        return cropped, center, (x_max - x_min, y_max - y_min)
