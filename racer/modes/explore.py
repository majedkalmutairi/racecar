import numpy as np


class ExploreController:
    """EXPLORE mode: drive slowly, centred between the walls, using only the lidar scan."""

    def __init__(self, cfg):
        ex = cfg["explore"]
        self.speed = ex["speed_mps"]
        self.k = ex["steer_k"]
        self.side_beams = ex["side_beams"]
        self.front_slow_m = ex["front_slow_m"]
        self.front_slow_factor = ex["front_slow_factor"]
        side = ex["side_angle_rad"]

        n = cfg["lidar"]["n_beams"]
        fov = cfg["lidar"]["fov_rad"]
        angles = np.linspace(-fov / 2, fov / 2, n)
        self.i_left = int(np.argmin(np.abs(angles - side)))
        self.i_right = int(np.argmin(np.abs(angles + side)))
        self.i_front = n // 2
        self.last = (0.0, 0.0, 0.0, 0.0)   # left, right, front, steer: for the on-screen readout

    def side_mean(self, scan, centre_idx):
        half = self.side_beams // 2
        return float(np.mean(scan[centre_idx - half : centre_idx + half + 1]))

    def step(self, scan):
        left = self.side_mean(scan, self.i_left)
        right = self.side_mean(scan, self.i_right)
        front = float(scan[self.i_front])

        # positive steer turns left, so a farther left wall (left > right) steers left
        steer = self.k * (left - right)

        speed = self.speed
        if front < self.front_slow_m:
            speed *= self.front_slow_factor

        self.last = (left, right, front, steer)
        return speed, steer
