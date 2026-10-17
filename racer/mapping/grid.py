import numpy as np


def scan_to_world(scan, pose, beam_angles):
    """Where each beam ends, as (N, 2) world coordinates."""
    x, y, yaw = pose
    a = yaw + beam_angles
    return np.c_[x + scan * np.cos(a), y + scan * np.sin(a)]


class OccupancyGrid:
    """A square of small cells; each cell counts how many beams ended inside it."""

    def __init__(self, size_m, res_m):
        self.res = res_m
        self.n = int(round(size_m / res_m))
        self.origin = -size_m / 2
        self.hits = np.zeros((self.n, self.n), dtype=np.int32)

    def world_to_cell(self, x, y):
        col = np.floor((np.asarray(x) - self.origin) / self.res).astype(int)
        row = np.floor((np.asarray(y) - self.origin) / self.res).astype(int)
        return np.clip(row, 0, self.n - 1), np.clip(col, 0, self.n - 1)

    def cell_to_world(self, row, col):
        x = self.origin + (np.asarray(col) + 0.5) * self.res
        y = self.origin + (np.asarray(row) + 0.5) * self.res
        return x, y

    def add_scan(self, scan, pose, beam_angles, max_range):
        """Count one hit in every cell a beam ended in. Returns the (rows, cols) that were hit."""
        hit = scan < max_range
        points = scan_to_world(scan[hit], pose, beam_angles[hit])
        row, col = self.world_to_cell(points[:, 0], points[:, 1])
        np.add.at(self.hits, (row, col), 1)
        return row, col

    def occupied(self, min_hits):
        return self.hits >= min_hits

    def save(self, path):
        np.save(path, self.hits)

    @classmethod
    def load(cls, path, size_m, res_m):
        grid = cls(size_m, res_m)
        grid.hits = np.load(path)
        return grid
