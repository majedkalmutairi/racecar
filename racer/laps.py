import csv
import math
from pathlib import Path

LAP_FIELDS = ["lap_number", "lap_time_s", "crashes", "mode"]


def open_lap_log(path):
    """Open the lap CSV for appending; write the header only if the file is new or empty."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    f = open(path, "a", newline="")
    writer = csv.DictWriter(f, fieldnames=LAP_FIELDS)
    if f.tell() == 0:
        writer.writeheader()
        f.flush()
    return f, writer


class LapCounter:
    """Counts laps from the pose alone, so it works the same in the sim and on the car."""

    def __init__(self, start_pose, cfg):
        self.sx, self.sy = start_pose[0], start_pose[1]
        self.radius = cfg["laps"]["radius_m"]
        self.min_lap_s = cfg["laps"]["min_lap_s"]
        self.reset()

    def reset(self, t_now=None):
        """Forget all laps; the clock restarts at t_now (or on the next update)."""
        self.count = 0
        self.last_lap_time = None
        self.restart(t_now)

    def restart(self, t_now=None):
        """Throw away the running lap (e.g. the sim put the car back at the start)."""
        self.inside = True      # we (re)start inside the circle
        self.lap_start_t = t_now

    def _dist_to_start(self, x, y):
        return math.hypot(x - self.sx, y - self.sy)

    def update(self, pose, t_now):
        """Returns the lap time (s) on the frame a lap completes, else None."""
        now_inside = self._dist_to_start(pose[0], pose[1]) < self.radius
        if self.lap_start_t is None:
            self.lap_start_t = t_now    # first call starts the clock

        result = None
        if now_inside and not self.inside and t_now - self.lap_start_t >= self.min_lap_s:
            result = t_now - self.lap_start_t
            self.count += 1
            self.lap_start_t = t_now
            self.last_lap_time = result

        self.inside = now_inside
        return result

    def current_lap_s(self, t_now):
        if self.lap_start_t is None:
            return 0.0
        return t_now - self.lap_start_t
