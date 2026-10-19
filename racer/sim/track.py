from pathlib import Path

import pygame
import yaml

TRACKS_DIR = Path(__file__).resolve().parents[2] / "tracks"


def track_names():
    return sorted(p.stem for p in TRACKS_DIR.glob("*.png"))


class Track:
    def __init__(self, name, default_px_per_m):
        self.name = name
        image = pygame.image.load(TRACKS_DIR / f"{name}.png")
        rgb = pygame.surfarray.array3d(image).transpose(1, 0, 2)
        self.wall = rgb[:, :, 0] < 128
        self.h_px, self.w_px = self.wall.shape

        with open(TRACKS_DIR / f"{name}.yaml") as f:
            meta = yaml.safe_load(f)
        self.px_per_m = meta.get("px_per_m", default_px_per_m)
        p = meta["start_pose"]
        self.start_pose = (p["x_m"], p["y_m"], p["yaw_rad"])
        # generated tracks carry their own boxes (see racer/sim/trackgen.py)
        self.boxes_known = meta.get("boxes_known", []) + meta.get("slalom", [])
        self.boxes_drop = [(d["box"], d["at_lap"]) for d in meta.get("boxes_drop", [])]

        self.surface = image
        self._converted = False

    def world_to_px(self, x_m, y_m):
        col = int(x_m * self.px_per_m)
        row = int(self.h_px - y_m * self.px_per_m)
        return col, row

    def px_to_world(self, col, row):
        return col / self.px_per_m, (self.h_px - row) / self.px_per_m

    def is_wall(self, x_m, y_m):
        col, row = self.world_to_px(x_m, y_m)
        if col < 0 or col >= self.w_px or row < 0 or row >= self.h_px:
            return True
        return bool(self.wall[row, col])

    def add_box(self, x_m, y_m, w_m, h_m):
        """Paint a black box into the wall mask so the lidar and the crash check both see it."""
        left, bottom = self.world_to_px(x_m, y_m)
        right, top = self.world_to_px(x_m + w_m, y_m + h_m)
        left, right = max(left, 0), min(right, self.w_px)        # clip, or numpy would wrap negative rows
        top, bottom = max(top, 0), min(bottom, self.h_px)
        if left >= right or top >= bottom:
            return
        self.wall[top:bottom, left:right] = True
        pygame.draw.rect(self.surface, (0, 0, 0), (left, top, right - left, bottom - top))

    def draw(self, screen):
        if not self._converted:
            self.surface = self.surface.convert()
            self._converted = True
        screen.blit(self.surface, (0, 0))
