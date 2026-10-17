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

    def roomiest_steer(self, scan):
        room = [self.path_clearance(scan, s) for s in self.steer_options]
        return float(self.steer_options[int(np.argmax(room))])

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

    def apply(self, cmd, scan, v, t_now):
        """cmd = (speed, steer) from the mode controller. Returns (cmd, active).

        Too fast for the path ahead: slow to the speed at which it stays ttc_s away.
        Path ahead blocked: head for open space instead. No room at all: stop.
        """
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
            speed = min(speed, self.escape_speed)
        speed = min(speed, clearance / self.ttc_s) if clearance >= self.stop_clear_m else self.brake_speed
        return (speed, steer), True
