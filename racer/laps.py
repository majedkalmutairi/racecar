import math

LAP_FIELDS = ["track", "lap_number", "lap_time_s", "crashes", "mode"]


class LapCounter:
    """Counts laps from the pose alone, so it works the same in the sim and on the car.

    The finish line is a gate across the track through the start pose, square to the start direction. A lap ends
    when the car crosses it going forward, close enough to the start (gate_half_m to either side), and only after
    min_lap_s and min_lap_m, so backing up across it or brushing it right after the start never counts.
    """

    def __init__(self, start_pose, cfg, t_now=None):
        self.sx, self.sy = start_pose[0], start_pose[1]
        yaw = start_pose[2] if len(start_pose) > 2 else 0.0
        self.hx, self.hy = math.cos(yaw), math.sin(yaw)          # forward through the gate
        self.gate_half = cfg["laps"]["gate_half_m"]
        self.min_lap_s = cfg["laps"]["min_lap_s"]
        self.min_lap_m = cfg["laps"]["min_lap_m"]
        self.count = 0
        self.last_lap_time = None
        self.best_lap_time = None
        self.restart(t_now)

    def restart(self, t_now=None):
        """Throw away the running lap, e.g. after the sim put the car back at the start."""
        self.lap_start_t = t_now
        self.driven_m = 0.0
        self.last_xy = None
        self.last_ahead = None

    def update(self, pose, t_now):
        """Returns the lap time on the frame a lap completes, else None."""
        dx, dy = pose[0] - self.sx, pose[1] - self.sy
        ahead = dx * self.hx + dy * self.hy                      # metres past the finish line
        side = -dx * self.hy + dy * self.hx                      # metres beside the start, across the track
        if self.lap_start_t is None:
            self.lap_start_t = t_now
        if self.last_xy is not None:
            self.driven_m += math.hypot(pose[0] - self.last_xy[0], pose[1] - self.last_xy[1])
        self.last_xy = pose[:2]

        result = None
        crossed = self.last_ahead is not None and self.last_ahead < 0 <= ahead and abs(side) < self.gate_half
        long_enough = t_now - self.lap_start_t >= self.min_lap_s and self.driven_m >= self.min_lap_m
        if crossed and long_enough:
            result = t_now - self.lap_start_t
            self.count += 1
            self.lap_start_t = t_now
            self.driven_m = 0.0
            self.last_lap_time = result
            if self.best_lap_time is None or result < self.best_lap_time:
                self.best_lap_time = result

        self.last_ahead = ahead
        return result

    def current_lap_s(self, t_now):
        if self.lap_start_t is None:
            return 0.0
        return t_now - self.lap_start_t
