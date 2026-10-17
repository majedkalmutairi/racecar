import numpy as np
from scipy.ndimage import binary_dilation

from racer.mapping.grid import scan_to_world


class ChangeDetector:
    """Watches whether live scans still land on the saved map; says so once they have not for hold_s."""

    def __init__(self, grid, beam_angles, cfg, max_range):
        tc = cfg["track_change"]
        tol = tc["cell_tol"]
        walls = grid.occupied(cfg["mapping"]["min_hits"])
        self.near_wall = binary_dilation(walls, np.ones((2 * tol + 1, 2 * tol + 1), dtype=bool))
        self.grid = grid
        self.angles = beam_angles
        self.max_range = max_range
        self.threshold = tc["threshold"]
        self.hold_s = tc["hold_s"]
        self.bad_since = None
        self.mismatch = 0.0

    def mismatch_fraction(self, scan, pose):
        """Fraction of beams that hit something where the map has no wall nearby."""
        valid = scan < self.max_range
        if not valid.any():
            return 0.0
        points = scan_to_world(scan[valid], pose, self.angles[valid])
        row, col = self.grid.world_to_cell(points[:, 0], points[:, 1])
        return float(np.mean(~self.near_wall[row, col]))

    def update(self, scan, pose, t_now):
        """Returns True once the mismatch has stayed above the threshold for hold_s."""
        self.mismatch = self.mismatch_fraction(scan, pose)
        if self.mismatch <= self.threshold:
            self.bad_since = None
            return False
        if self.bad_since is None:
            self.bad_since = t_now
        return t_now - self.bad_since >= self.hold_s
