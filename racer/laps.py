import math

LAP_FIELDS = ["track", "lap_number", "lap_time_s", "crashes", "mode"]


class LapCounter:
    """Counts laps from the pose alone, so it works the same in the sim and on the car."""

    def __init__(self, start_pose, cfg, t_now=None):
        self.sx, self.sy = start_pose[0], start_pose[1]
        self.radius = cfg["laps"]["radius_m"]
        self.min_lap_s = cfg["laps"]["min_lap_s"]
        self.count = 0
        self.last_lap_time = None
        self.best_lap_time = None
        self.restart(t_now)

    def restart(self, t_now=None):
        """Throw away the running lap, e.g. after the sim put the car back at the start."""
        self.inside = True
        self.lap_start_t = t_now

    def _dist_to_start(self, x, y):
        return math.hypot(x - self.sx, y - self.sy)

    def update(self, pose, t_now):
        """Returns the lap time on the frame a lap completes, else None."""
        now_inside = self._dist_to_start(pose[0], pose[1]) < self.radius
        if self.lap_start_t is None:
            self.lap_start_t = t_now

        result = None
        if now_inside and not self.inside and t_now - self.lap_start_t >= self.min_lap_s:
            result = t_now - self.lap_start_t
            self.count += 1
            self.lap_start_t = t_now
            self.last_lap_time = result
            if self.best_lap_time is None or result < self.best_lap_time:
                self.best_lap_time = result

        self.inside = now_inside
        return result

    def current_lap_s(self, t_now):
        if self.lap_start_t is None:
            return 0.0
        return t_now - self.lap_start_t
