import math

import numpy as np
import pygame

from racer.interface import beam_angles

BEAM_COLOUR = (0, 140, 220, 70)
CHUNK = 100  # samples per march step (2 m at 2 cm)


class Lidar:
    def __init__(self, cfg, seed=0):
        lid = cfg["lidar"]
        self.max_range = float(lid["max_range_m"])
        self.step = lid["step_m"]
        self.noise = lid["noise_m"]
        self.angles = beam_angles(cfg)
        self.rng = np.random.default_rng(seed)
        # sample distances 0, step, 2*step, ... added up one at a time like a loop would
        n = int(self.max_range / self.step) + 2
        d = np.add.accumulate(np.full(n, float(self.step)))
        d = np.concatenate(([0.0], d))
        self.d = d[d < self.max_range]

    def scan(self, track, x, y, yaw):
        # all beams march together, a chunk of samples at a time; beams that hit drop out
        c = np.array([math.cos(yaw + a) for a in self.angles])
        s = np.array([math.sin(yaw + a) for a in self.angles])
        ranges = np.full(len(self.angles), self.max_range)
        todo = np.arange(len(self.angles))
        for i in range(0, len(self.d), CHUNK):
            d = self.d[i:i + CHUNK]
            col = ((x + d * c[todo, None]) * track.px_per_m).astype(int)  # truncates like int()
            row = (track.h_px - (y + d * s[todo, None]) * track.px_per_m).astype(int)
            inside = (col >= 0) & (col < track.w_px) & (row >= 0) & (row < track.h_px)
            hit = ~inside
            hit[inside] = track.wall[row[inside], col[inside]]
            done = hit.any(axis=1)
            ranges[todo[done]] = d[hit[done].argmax(axis=1)]
            todo = todo[~done]
            if len(todo) == 0:
                break

        hit = ranges < self.max_range
        ranges[hit] += self.rng.normal(0.0, self.noise, int(hit.sum()))
        return np.clip(ranges, 0.0, self.max_range)

    def draw(self, screen, track, x, y, yaw, ranges):
        """Beams as faint lines, drawn on a see-through layer so the track stays readable."""
        if getattr(self, "_layer", None) is None or self._layer.get_size() != screen.get_size():
            self._layer = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        self._layer.fill((0, 0, 0, 0))
        start = track.to_screen(x, y)
        for a, r in zip(self.angles, ranges):
            theta = yaw + a
            end = track.to_screen(x + r * math.cos(theta), y + r * math.sin(theta))
            pygame.draw.line(self._layer, BEAM_COLOUR, start, end)
        screen.blit(self._layer, (0, 0))
