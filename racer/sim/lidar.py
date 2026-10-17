import math

import numpy as np
import pygame

from racer.interface import beam_angles

BEAM_COLOUR = (0, 140, 220)


class Lidar:
    def __init__(self, cfg, seed=0):
        lid = cfg["lidar"]
        self.max_range = float(lid["max_range_m"])
        self.step = lid["step_m"]
        self.noise = lid["noise_m"]
        self.angles = beam_angles(cfg)
        self.rng = np.random.default_rng(seed)

    def scan(self, track, x, y, yaw):
        ranges = np.full(len(self.angles), self.max_range)
        for i, a in enumerate(self.angles):
            theta = yaw + a
            c, s = math.cos(theta), math.sin(theta)
            d = 0.0
            while d < self.max_range:
                if track.is_wall(x + d * c, y + d * s):
                    break
                d += self.step
            ranges[i] = min(d, self.max_range)

        hit = ranges < self.max_range
        ranges[hit] += self.rng.normal(0.0, self.noise, int(hit.sum()))
        return np.clip(ranges, 0.0, self.max_range)

    def draw(self, screen, track, x, y, yaw, ranges):
        start = track.world_to_px(x, y)
        for a, r in zip(self.angles, ranges):
            theta = yaw + a
            end = track.world_to_px(x + r * math.cos(theta), y + r * math.sin(theta))
            pygame.draw.line(screen, BEAM_COLOUR, start, end)
