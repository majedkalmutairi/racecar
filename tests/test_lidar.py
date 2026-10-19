import math

import numpy as np

from racer.sim.lidar import Lidar

CFG = {"lidar": {"n_beams": 108, "fov_rad": 4.71, "max_range_m": 12, "step_m": 0.02, "noise_m": 0.0}}


class GridTrack:
    """Wall mask only, same pixel rules as Track."""

    def __init__(self, wall, px_per_m=100):
        self.wall = wall
        self.h_px, self.w_px = wall.shape
        self.px_per_m = px_per_m

    def is_wall(self, x_m, y_m):
        col = int(x_m * self.px_per_m)
        row = int(self.h_px - y_m * self.px_per_m)
        if col < 0 or col >= self.w_px or row < 0 or row >= self.h_px:
            return True
        return bool(self.wall[row, col])


def loop_scan(track, x, y, yaw, angles, max_range=12.0, step=0.02):
    """The original one-beam-at-a-time ray march, kept as the reference."""
    ranges = np.full(len(angles), max_range)
    for i, a in enumerate(angles):
        c, s = math.cos(yaw + a), math.sin(yaw + a)
        d = 0.0
        while d < max_range:
            if track.is_wall(x + d * c, y + d * s):
                break
            d += step
        ranges[i] = min(d, max_range)
    return ranges


def make_track(seed):
    rng = np.random.default_rng(seed)
    wall = np.zeros((700, 1000), bool)
    wall[:5, :] = wall[-5:, :] = True
    for _ in range(30):
        r, c = rng.integers(0, 680), rng.integers(0, 980)
        wall[r:r + rng.integers(3, 40), c:c + rng.integers(3, 40)] = True
    return GridTrack(wall)


def test_lidar_matches_loop_exactly():
    rng = np.random.default_rng(1)
    for seed in range(3):
        track = make_track(seed)
        lidar = Lidar(CFG)
        for _ in range(40):
            x, y, yaw = rng.uniform(-1, 11), rng.uniform(-1, 8), rng.uniform(-7, 7)
            assert np.array_equal(lidar.scan(track, x, y, yaw), loop_scan(track, x, y, yaw, lidar.angles))


def test_lidar_open_space_is_max_range():
    track = GridTrack(np.zeros((3000, 3000), bool))
    r = Lidar(CFG).scan(track, 15.0, 15.0, 0.3)
    assert np.all(r == 12.0)


def test_lidar_wall_ahead():
    wall = np.zeros((700, 1000), bool)
    wall[:, 600:] = True  # wall from x = 6 m
    r = Lidar(CFG).scan(GridTrack(wall), 1.0, 3.5, 0.0)
    assert abs(r[54] - 5.0) < 0.03  # beams 53 and 54 straddle straight ahead
