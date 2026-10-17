import math

import numpy as np
import pygame

BEAM_COLOUR = (0, 140, 220)


class Lidar:
    def __init__(self, cfg):
        lid = cfg["lidar"]
        self.n_beams = lid["n_beams"]
        self.fov = lid["fov_rad"]
        self.max_range = float(lid["max_range_m"])
        self.step = lid["step_m"]
        # beam 0 = far right, last = far left, middle = straight ahead
        self.angles = np.linspace(-self.fov / 2, self.fov / 2, self.n_beams)

    def scan(self, track, x, y, yaw):
        ranges = np.full(self.n_beams, self.max_range)
        for i, a in enumerate(self.angles):
            theta = yaw + a
            c, s = math.cos(theta), math.sin(theta)
            d = 0.0
            while d < self.max_range:
                if track.is_wall(x + d * c, y + d * s):
                    break
                d += self.step
            ranges[i] = min(d, self.max_range)
        return ranges

    def draw(self, screen, track, x, y, yaw, ranges):
        start = track.world_to_px(x, y)
        for a, r in zip(self.angles, ranges):
            theta = yaw + a
            end = track.world_to_px(x + r * math.cos(theta), y + r * math.sin(theta))
            pygame.draw.line(screen, BEAM_COLOUR, start, end)
