import numpy as np

from racer.interface import beam_angles


class ExploreController:
    """First lap: drive slowly, stay centred between the walls, steer away from anything close ahead."""

    def __init__(self, cfg):
        ex = cfg["explore"]
        self.speed = ex["speed_mps"]
        self.k = ex["steer_k"]
        self.side_beams = ex["side_beams"]
        self.front_slow_m = ex["front_slow_m"]
        self.front_slow_factor = ex["front_slow_factor"]
        self.avoid_m = ex["avoid_m"]
        self.avoid_k = ex["avoid_k"]

        angles = beam_angles(cfg)
        side = ex["side_angle_rad"]
        self.i_left = int(np.argmin(np.abs(angles - side)))
        self.i_right = int(np.argmin(np.abs(angles + side)))
        self.i_front = len(angles) // 2
        self.ahead = np.abs(angles) <= ex["avoid_fov_rad"]
        self.ahead_sign = np.sign(angles[self.ahead])
        self.last = (0.0, 0.0, 0.0, 0.0)

    def side_mean(self, scan, centre_idx):
        half = self.side_beams // 2
        return float(np.mean(scan[centre_idx - half : centre_idx + half + 1]))

    def step(self, scan):
        left = self.side_mean(scan, self.i_left)
        right = self.side_mean(scan, self.i_right)
        front = float(scan[self.i_front])

        steer = self.k * (left - right)

        too_close = np.maximum(0.0, self.avoid_m - scan[self.ahead])
        steer -= self.avoid_k * float(np.sum(self.ahead_sign * too_close)) / len(too_close)

        speed = self.speed
        if front < self.front_slow_m:
            speed *= self.front_slow_factor

        self.last = (left, right, front, steer)
        return speed, steer
