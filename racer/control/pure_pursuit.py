import math

import numpy as np


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def lateral_error(pose, line, idx):
    """Sideways distance from the car to the line segment starting at idx. Positive = car is left of the line."""
    p0 = line[idx]
    p1 = line[(idx + 1) % len(line)]
    t = (p1 - p0) / np.linalg.norm(p1 - p0)
    d = np.array(pose[:2]) - p0
    return float(t[0] * d[1] - t[1] * d[0])


class PurePursuit:
    """Steer toward a point a little way ahead on the racing line; speed comes from the profile."""

    def __init__(self, line, speed_profile, cfg):
        self.line = line
        self.v_profile = speed_profile
        self.n = len(line)
        self.L = cfg["car"]["wheelbase_m"]
        self.max_steer = cfg["car"]["max_steer_rad"]
        race = cfg["race"]
        self.ld_k = race["ld_k"]
        self.ld_min = race["ld_min_m"]
        self.ld_max = race["ld_max_m"]
        self.window = race["search_window"]
        self.ds = cfg["plan"]["ds_m"]
        self.idx = 0
        self.multiplier = np.ones(self.n)
        self.target = line[0]
        self.v_target = 0.0

    def relocate(self, pose):
        """Search the whole line for the nearest point; used at the start and after a teleport."""
        self.idx = int(np.argmin(np.linalg.norm(self.line - pose[:2], axis=1)))

    def nearest_index(self, x, y):
        candidates = (self.idx + np.arange(-5, self.window)) % self.n
        d = np.linalg.norm(self.line[candidates] - (x, y), axis=1)
        self.idx = int(candidates[np.argmin(d)])
        return self.idx

    def lookahead_index(self, idx, v):
        ld = float(np.clip(self.ld_k * v, self.ld_min, self.ld_max))
        return (idx + int(ld / self.ds)) % self.n

    def step(self, pose, v):
        """Returns (speed, steer) for the car."""
        x, y, yaw = pose
        idx = self.nearest_index(x, y)
        i_target = self.lookahead_index(idx, v)
        target = self.line[i_target]

        dx, dy = target[0] - x, target[1] - y
        ld = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)
        steer = math.atan2(2 * self.L * math.sin(alpha), ld)
        steer = max(-self.max_steer, min(self.max_steer, steer))

        speed = min(self.v_profile[idx], self.v_profile[i_target]) * self.multiplier[idx]
        self.target = target
        self.v_target = float(speed)
        return self.v_target, steer
