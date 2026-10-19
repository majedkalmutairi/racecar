import math

import numpy as np

from racer.planning.speed_profile import backward_pass


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
        self.a_brake = cfg["plan"]["a_brake"]
        self.idx = 0
        self.multiplier = np.ones(self.n)
        self.target = line[0]
        self.v_target = 0.0

    @property
    def multiplier(self):
        return self._mult

    @multiplier.setter
    def multiplier(self, mult):
        """Learned speed multiplier per point. The target speed is the profile times the multiplier, braked again,
        so the car slows down in time for a section that learned to be slow instead of arriving at its neighbour's speed."""
        self._mult = np.asarray(mult, dtype=float)
        self.v_target_line = backward_pass(list(self.v_profile * self._mult), self.a_brake, self.ds)

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

    def predict(self, pose, v, n):
        """Where this controller will take the car over the next n steps of ds metres at speed v:
        the same steering law run forward on the bicycle model. Includes the corner cutting the real car does."""
        x, y, yaw = pose
        idx = self.idx
        path = np.empty((n, 2))
        for i in range(n):
            near = (idx + np.arange(-2, 10)) % self.n
            idx = int(near[np.argmin(np.hypot(self.line[near, 0] - x, self.line[near, 1] - y))])
            tx, ty = self.line[self.lookahead_index(idx, v)]
            alpha = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
            steer = math.atan2(2 * self.L * math.sin(alpha), max(math.hypot(tx - x, ty - y), 1e-6))
            steer = max(-self.max_steer, min(self.max_steer, steer))
            x += self.ds * math.cos(yaw)
            y += self.ds * math.sin(yaw)
            yaw += self.ds / self.L * math.tan(steer)
            path[i] = x, y
        return path

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

        speed = min(self.v_target_line[idx], self.v_target_line[i_target])
        self.target = target
        self.v_target = float(speed)
        return self.v_target, steer
