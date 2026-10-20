import math

import numpy as np


class SafetyReflex:
    """Brake and steer toward open space when the car's predicted path hits something within ttc_s.

    Runs last, in every mode, so no controller can drive the car into a wall. Mirrored in C++ on the real car.
    """

    def __init__(self, beam_angles, cfg):
        s = cfg["safety"]
        car = cfg["car"]
        self.ttc_s = s["ttc_s"]
        self.v_min = s["v_min_mps"]
        self.brake_speed = s["brake_speed_mps"]
        self.hold_s = s["hold_s"]
        self.escape_clear_m = s["escape_clear_m"]
        self.stop_clear_m = s["stop_clear_m"]
        self.escape_speed = s["escape_speed_mps"]
        self.gap_range_m = s["gap_range_m"]
        self.bubble_m = s["bubble_m"]
        self.L = car["wheelbase_m"]
        self.max_steer = car["max_steer_rad"]
        self.nose = car["wheelbase_m"] / 2 + car["length_m"] / 2
        self.half_w = car["width_m"] / 2
        self.margin = s["body_margin_m"]
        self.angles = beam_angles
        self.max_range = cfg["lidar"]["max_range_m"]
        self.fwd = np.abs(beam_angles) <= math.radians(s["fov_deg"] / 2)
        self.cos = np.cos(beam_angles)
        self.sin = np.sin(beam_angles)
        self.steer_options = np.linspace(-self.max_steer, self.max_steer, s["steer_options"])
        self.hold_until = -1.0
        self.min_ttc = math.inf

    def path_clearance(self, scan, steer):
        """Metres the car can travel along this steer's arc before its body comes within the margin of something.

        Going straight the body sweeps a strip; turning it sweeps a ring between the inner rear
        corner and the outer front corner. Only beam hits inside that shape count.
        """
        px = scan * self.cos
        py = scan * self.sin
        if abs(math.tan(steer)) < 1e-3:
            on_path = np.abs(py) < self.half_w + self.margin
            along = px
        else:
            r = self.L / math.tan(steer)
            d = np.hypot(px, py - r)
            inner = abs(r) - self.half_w - self.margin
            outer = math.hypot(abs(r) + self.half_w, self.nose) + self.margin
            on_path = (d > inner) & (d < outer)
            along = abs(r) * np.abs(np.arctan2(r * px, r * (r - py)))
        along = np.where(on_path & self.fwd & (px > 0), along - self.nose, np.inf)
        return float(max(along.min(), 0.0))

    def line_clearance(self, scan, pose, ahead):
        """Metres the car can follow the planned line before its body comes within the margin of a lidar hit.

        ahead: the next racing-line points, starting at the car. The body passes over a hit near line point i
        while the rear axle is up to `nose` behind it, hence the subtraction at the end.
        """
        x, y, yaw = pose
        use = self.fwd & (scan < self.max_range)
        a = yaw + self.angles[use]
        hits = np.c_[x + scan[use] * np.cos(a), y + scan[use] * np.sin(a)]
        if len(hits) == 0:
            return math.inf
        s = np.concatenate([[math.hypot(ahead[0, 0] - x, ahead[0, 1] - y)],
                            np.linalg.norm(np.diff(ahead, axis=0), axis=1)]).cumsum()
        d = np.linalg.norm(hits[:, None, :] - ahead[None, :, :], axis=2)
        nearest = d.argmin(axis=1)                      # where along the line each hit sits...
        on_line = d.min(axis=1) < self.half_w + self.margin     # ...and whether it is close enough sideways
        if not on_line.any():
            return math.inf
        return float(max(s[nearest[on_line]].min() - self.nose, 0.0))

    def roomiest_steer(self, scan):
        """The steer with the most room ahead. On a tie (often: none has any) the one turning hardest away from
        the nearest hit, so a car still rolling to a stop does not swing its nose into it."""
        room = np.array([self.path_clearance(scan, s) for s in self.steer_options])
        away = -np.sign(self.angles[int(np.argmin(np.where(self.fwd, scan, np.inf)))])
        best = np.flatnonzero(room >= room.max() - 1e-9)
        return float(self.steer_options[best[np.argmax(away * self.steer_options[best])]])

    def escape_steer(self, scan):
        """Follow the gap: blank the beams around the nearest hit, find the widest run of open beams, aim at its middle."""
        ahead = np.where(self.fwd, scan, np.inf)
        i_near = int(np.argmin(ahead))
        bubble = self.bubble_m / max(ahead[i_near], 0.05)
        is_open = self.fwd & (scan > self.gap_range_m) & (np.abs(self.angles - self.angles[i_near]) > bubble)

        best_len, best_mid, start = 0, None, None
        for i, open_beam in enumerate(np.r_[is_open, False]):
            if open_beam and start is None:
                start = i
            elif not open_beam and start is not None:
                if i - start > best_len:
                    best_len, best_mid = i - start, (start + i - 1) / 2
                start = None
        if best_mid is None:
            return self.roomiest_steer(scan)
        angle = np.interp(best_mid, np.arange(len(self.angles)), self.angles)
        steer = math.atan2(2 * self.L * math.sin(angle), self.gap_range_m)
        return max(-self.max_steer, min(self.max_steer, steer))

    def apply(self, cmd, scan, v, t_now, pose=None, ahead=None):
        """cmd = (speed, steer) from the mode controller. Returns (cmd, active).

        On the plan (ahead = the racing line in front of the car): only something on that line counts, and the
        answer is to slow down to the speed that keeps it ttc_s away; the plan keeps steering.
        Off the plan, or the line itself blocked: judge the arc the current steer would drive. Too fast for it:
        slow down. Arc blocked: head for open space instead. No room at all: stop.
        """
        if ahead is not None:
            clearance = self.line_clearance(scan, pose, ahead)
            if clearance >= self.escape_clear_m:
                self.min_ttc = clearance / max(v, self.v_min)
                if self.min_ttc < self.ttc_s:
                    self.hold_until = t_now + self.hold_s
                if t_now >= self.hold_until:
                    return cmd, False
                return (min(cmd[0], clearance / self.ttc_s), cmd[1]), True
        clearance = self.path_clearance(scan, cmd[1])
        self.min_ttc = clearance / max(v, self.v_min)
        if self.min_ttc < self.ttc_s or clearance < self.escape_clear_m:
            self.hold_until = t_now + self.hold_s
        if t_now >= self.hold_until:
            return cmd, False
        speed, steer = cmd
        if clearance < self.escape_clear_m:
            steer = self.escape_steer(scan)
            clearance = self.path_clearance(scan, steer)
            if clearance < self.stop_clear_m:          # the gap is there but this arc still hits: take the roomiest arc
                roomy = self.roomiest_steer(scan)
                if self.path_clearance(scan, roomy) >= clearance:
                    steer, clearance = roomy, self.path_clearance(scan, roomy)
            speed = min(speed, self.escape_speed)
        speed = min(speed, clearance / self.ttc_s) if clearance >= self.stop_clear_m else self.brake_speed
        return (speed, steer), True
