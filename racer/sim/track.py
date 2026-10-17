from pathlib import Path

import numpy as np
import pygame
import yaml

TRACKS_DIR = Path(__file__).resolve().parents[2] / "tracks"


class Track:
    def __init__(self, name, default_px_per_m):
        image = pygame.image.load(TRACKS_DIR / f"{name}.png")
        rgb = pygame.surfarray.array3d(image).transpose(1, 0, 2)
        self.wall = rgb[:, :, 0] < 128
        self.h_px, self.w_px = self.wall.shape

        with open(TRACKS_DIR / f"{name}.yaml") as f:
            meta = yaml.safe_load(f)
        self.px_per_m = meta.get("px_per_m", default_px_per_m)
        p = meta["start_pose"]
        self.start_pose = (p["x_m"], p["y_m"], p["yaw_rad"])

        self.surface = image
        self._converted = False

    def world_to_px(self, x_m, y_m):
        col = int(x_m * self.px_per_m)
        row = int(self.h_px - y_m * self.px_per_m)
        return col, row

    def is_wall(self, x_m, y_m):
        col, row = self.world_to_px(x_m, y_m)
        if col < 0 or col >= self.w_px or row < 0 or row >= self.h_px:
            return True
        return bool(self.wall[row, col])

    def px_to_world(self, col, row):
        x_m = col / self.px_per_m
        y_m = (self.h_px - row) / self.px_per_m
        return x_m, y_m

    def draw(self, screen):
        if not self._converted:
            self.surface = self.surface.convert()
            self._converted = True
        screen.blit(self.surface, (0, 0))
