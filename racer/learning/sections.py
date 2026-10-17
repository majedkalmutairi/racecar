from pathlib import Path

import numpy as np


class SectionLearner:
    """One speed multiplier per section of the racing line, nudged up or down after every lap."""

    def __init__(self, n_points, cfg, path):
        lr = cfg["learning"]
        self.n_sections = lr["n_sections"]
        self.up = lr["up"]
        self.down = lr["down"]
        self.min_mult = lr["min_mult"]
        self.max_mult = lr["max_mult"]
        self.err_ok = lr["err_ok_m"]
        self.path = Path(path)
        self.remap(n_points)
        if self.path.exists():
            self.mult = np.load(self.path)
        else:
            self.mult = np.full(self.n_sections, float(lr["start_mult"]))
        self._new_lap()

    def remap(self, n_points):
        """Point -> section lookup; called again when the racing line is rebuilt with a different length."""
        self.section_of = np.arange(n_points) * self.n_sections // n_points

    def _new_lap(self):
        self.max_err = np.zeros(self.n_sections)
        self.problem = np.zeros(self.n_sections, dtype=bool)
        self.visited = np.zeros(self.n_sections, dtype=bool)

    def observe(self, idx, lateral_err_m, safety_fired, crashed):
        """Called every RACE frame with the nearest racing-line index."""
        s = self.section_of[idx]
        self.max_err[s] = max(self.max_err[s], abs(lateral_err_m))
        self.problem[s] |= bool(safety_fired or crashed)
        self.visited[s] = True

    def end_lap(self):
        """Grade every section driven this lap, save, and start a fresh lap. Returns the clean mask."""
        clean = (self.max_err < self.err_ok) & ~self.problem & self.visited
        bad = ~clean & self.visited
        self.mult[clean] *= 1 + self.up
        self.mult[bad] *= 1 - self.down
        self.mult = np.clip(self.mult, self.min_mult, self.max_mult)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        np.save(self.path, self.mult)
        self._new_lap()
        return clean

    def multiplier_per_point(self):
        return self.mult[self.section_of]
