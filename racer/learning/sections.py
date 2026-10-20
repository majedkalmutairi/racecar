from pathlib import Path

import numpy as np


class SectionLearner:
    """One speed multiplier per section of the racing line, pushed up or pulled back after every lap.

    Confidence = clean laps in a row. The more confident a section is, the bigger its next step up.
    A problem resets the confidence and remembers the multiplier it happened at as a ceiling: the section
    then climbs back to just under it, and tries a little past it again after a few clean laps.
    """

    def __init__(self, n_points, cfg, path):
        lr = cfg["learning"]
        lap_m = n_points * cfg["plan"]["ds_m"]
        self.n_sections = max(lr["n_sections"], int(round(lap_m / lr["section_m"])))
        self.up = lr["up"]
        self.up_max = lr["up_max"]
        self.down = lr["down"]
        self.down_crash = lr["down_crash"]
        self.below_ceiling = lr["below_ceiling"]
        self.probe_laps = lr["probe_laps"]
        self.probe = lr["probe"]
        self.min_mult = lr["min_mult"]
        self.max_mult = lr["max_mult"]
        self.err_ok = lr["err_ok_m"]
        self.path = Path(path)
        self.remap(n_points)
        self.mult = np.full(self.n_sections, float(lr["start_mult"]))
        if self.path.exists():
            saved = np.load(self.path)
            if saved.shape == self.mult.shape:
                self.mult = saved
        self.streak = np.zeros(self.n_sections, dtype=int)
        self.ceiling = np.full(self.n_sections, np.inf)
        self.trouble = np.full(self.n_sections, np.inf)   # multiplier of the last real problem (probing never moves it)
        self._new_lap()

    def remap(self, n_points):
        """Point -> section lookup; called again when the racing line is rebuilt with a different length."""
        self.section_of = np.arange(n_points) * self.n_sections // n_points

    def _new_lap(self):
        self.max_err = np.zeros(self.n_sections)
        self.problem = np.zeros(self.n_sections, dtype=bool)
        self.crashed = np.zeros(self.n_sections, dtype=bool)
        self.visited = np.zeros(self.n_sections, dtype=bool)

    def observe(self, idx, lateral_err_m, safety_fired, crashed):
        """Called every RACE frame with the nearest racing-line index."""
        s = self.section_of[idx]
        self.max_err[s] = max(self.max_err[s], abs(lateral_err_m))
        self.problem[s] |= bool(safety_fired or crashed)
        self.crashed[s] |= bool(crashed)
        self.visited[s] = True

    def end_lap(self):
        """Grade every section driven this lap, save, and start a fresh lap. Returns the clean mask."""
        clean = (self.max_err < self.err_ok) & ~self.problem & self.visited
        bad = ~clean & self.visited

        # bad: remember where it went wrong, back off (harder after a crash), confidence back to zero
        self.ceiling[bad] = self.trouble[bad] = self.mult[bad]
        self.mult[bad] *= 1 - np.where(self.crashed[bad], self.down_crash, self.down)
        self.streak[bad] = 0

        # clean: one more lap of confidence, a bigger step, but stay under a known ceiling...
        self.streak[clean] += 1
        step = np.minimum(self.up * self.streak[clean], self.up_max)
        self.mult[clean] = np.minimum(self.mult[clean] * (1 + step), self.ceiling[clean] * self.below_ceiling)
        # ...until enough clean laps say the ceiling was too low: then try a little past it
        retry = clean & (self.streak % self.probe_laps == 0) & np.isfinite(self.ceiling)
        self.ceiling[retry] *= 1 + self.probe

        self.mult = np.clip(self.mult, self.min_mult, self.max_mult)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        np.save(self.path, self.mult)
        self._new_lap()
        return clean

    def crashed_at(self, idx):
        """The car hit something and was put back at the start: grade this partial lap now, or the restarted lap
        would never reach end_lap and the car would crash in the same place forever. The crash usually starts
        earlier (too fast into the corner), so every section that already had trouble this lap backs off too,
        and so does the one just before the crash."""
        s = self.section_of[idx]
        self.problem[(s - 1) % self.n_sections] = True
        self.problem[s] = self.crashed[s] = True
        bad = self.problem
        self.ceiling[bad] = self.trouble[bad] = self.mult[bad]
        self.mult[bad] *= 1 - np.where(self.crashed[bad], self.down_crash, self.down)
        self.streak[bad] = 0
        self.mult = np.clip(self.mult, self.min_mult, self.max_mult)
        np.save(self.path, self.mult)
        self._new_lap()

    def confidence(self):
        """0..1 per section: clean laps in a row, full at the lap the step reaches up_max."""
        return np.minimum(self.streak * self.up / self.up_max, 1.0)

    def multiplier_per_point(self):
        return self.mult[self.section_of]
