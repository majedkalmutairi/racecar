import math

import numpy as np

from racer.interface import beam_angles
from racer.safety.ttc import SafetyReflex


class ExploreController:
    """First lap: drive slowly and safely round a track nobody has mapped yet.

    method "centre": stay centred between the walls and steer away from anything close ahead (best on narrow
    tracks). method "gap": follow the gap (best on wide tracks and round boxes). method "auto": centre while the
    side room is under wide_m, follow the gap where the track is wider. Wherever the scan jumps from near to far (the edge of a box or a
    wall), the near range is copied over the next beams, as wide as half the car plus a margin, so a gap the car
    does not fit through looks closed (disparity extender). Then aim at the deepest open direction ahead.
    Whatever the method, a box in the car's straight path with room on both sides is passed by the smallest
    sidestep (kept to that side until past), and no steer is used whose arc would hit something within arc_clear_m.
    """

    def __init__(self, cfg):
        ex = cfg["explore"]
        car = cfg["car"]
        self.method = ex["method"]
        self.speed = ex["speed_mps"]
        self.k = ex["steer_k"]
        self.ref_width = ex["ref_width_m"]
        self.side_beams = ex["side_beams"]
        self.front_slow_m = ex["front_slow_m"]
        self.front_slow_factor = ex["front_slow_factor"]
        self.avoid_m = ex["avoid_m"]
        self.avoid_k = ex["avoid_k"]
        self.max_steer = car["max_steer_rad"]
        self.L = car["wheelbase_m"]
        self.half_w = car["width_m"] / 2 + ex["gap_margin_m"]
        self.nose = car["wheelbase_m"] / 2 + car["length_m"] / 2      # rear axle to front bumper
        self.disparity = ex["disparity_m"]
        self.aim_max = ex["gap_aim_max_m"]
        self.wide = ex["wide_m"]
        self.block_m = ex["block_m"]
        self.arc_clear = ex["arc_clear_m"]
        self.step_half_w = car["width_m"] / 2 + ex["sidestep_margin_m"]
        self.past_m = ex["sidestep_past_m"]
        self.shifts = np.arange(0.0, ex["sidestep_max_m"], 0.025)

        self.angles = beam_angles(cfg)
        self.step_rad = float(self.angles[1] - self.angles[0])
        side = ex["side_angle_rad"]
        self.i_left = int(np.argmin(np.abs(self.angles - side)))
        self.i_right = int(np.argmin(np.abs(self.angles + side)))
        self.i_front = len(self.angles) // 2
        self.ahead = np.abs(self.angles) <= ex["avoid_fov_rad"]
        self.ahead_sign = np.sign(self.angles[self.ahead])
        self.fwd = np.abs(self.angles) <= ex["gap_fov_rad"]
        self.cos, self.sin = np.cos(self.angles), np.sin(self.angles)
        self.arcs = SafetyReflex(self.angles, cfg)          # only its swept-body check is used
        self.side = 0                       # the side a box is being passed on (+1 left, -1 right), 0 = none
        self.last = (0.0, 0.0, 0.0, 0.0)

    def side_mean(self, scan, centre_idx):
        half = self.side_beams // 2
        return float(np.mean(scan[centre_idx - half : centre_idx + half + 1]))

    def step(self, scan):
        left = self.side_mean(scan, self.i_left)
        right = self.side_mean(scan, self.i_right)
        front = float(scan[self.i_front])

        ahead = self.path_clear(scan)
        blocked = ahead < self.block_m                      # something in the car's way: find a way past
        if not blocked:
            self.side = 0
        shift = self.sidestep(scan, ahead) if blocked else None
        if shift is not None:
            # pure pursuit to just beside it, reached by the time the nose (not the rear axle) gets there
            reach = max(ahead - self.nose, 0.3)
            steer = math.atan(2 * self.L * shift / (reach ** 2 + shift ** 2))
        elif self.method == "gap" or (self.method == "auto" and (left + right > self.wide or blocked)):
            steer = self.gap_steer(scan)
        else:
            # centre by the relative difference, scaled to a normal track: on a wide track the same offset is a
            # smaller share of the room, so the car does not swing to full lock and hug one wall
            steer = self.k * self.ref_width * (left - right) / max(left + right, 1e-6)
            too_close = np.maximum(0.0, self.avoid_m - scan[self.ahead])
            steer -= self.avoid_k * float(np.sum(self.ahead_sign * too_close)) / len(too_close)

        speed = self.speed
        if front < self.front_slow_m:
            speed *= self.front_slow_factor

        steer = self.clear_arc(scan, max(-self.max_steer, min(self.max_steer, steer)))
        self.last = (left, right, front, steer)
        return speed, steer

    def sidestep(self, scan, ahead):
        """Smallest sideways shift (+ left) that gets the car past what blocks its path `ahead` metres away, when
        there is a way past on both sides (a box, not the wall of a corner); else None.

        A shift is clear when no hit from here to a little past the blocker lies within half the car plus a margin
        of it. The search on a side stops at anything beside the car before the blocker (a wall).
        """
        px, py = scan * self.cos, scan * self.sin
        near = (px > 0) & (px < ahead + self.past_m)
        beside = near & (px < ahead - 0.1)
        found = {}
        for sign in (1, -1):
            for o in sign * self.shifts:
                band = np.abs(py - o) < self.step_half_w
                if np.any(beside & band):
                    break
                if not np.any(near & band):
                    found[sign] = o
                    break
        if found.get(self.side):                    # already going round one side: keep to it until past
            return found[self.side]
        if len(found) < 2 or min(abs(o) for o in found.values()) == 0:     # a wall on one side, or nothing in the way
            return None
        shift = min(found.values(), key=abs)
        self.side = 1 if shift > 0 else -1
        return shift

    def clear_arc(self, scan, steer):
        """The steer nearest the wanted one whose arc the car body can drive for arc_clear_m, so the car never
        turns its side into the corner of a box it is passing. No arc is that free: the roomiest one."""
        if self.arcs.path_clearance(scan, steer) >= self.arc_clear:
            return steer
        options = self.arcs.steer_options
        room = np.array([self.arcs.path_clearance(scan, s) for s in options])
        free = room >= self.arc_clear
        if not free.any():
            return self.arcs.roomiest_steer(scan)
        return float(options[free][np.argmin(np.abs(options[free] - steer))])

    def extended(self, scan):
        """The scan with every near edge widened by half the car plus a margin (disparity extender)."""
        out = scan.copy()
        jumps = np.flatnonzero(np.abs(np.diff(scan)) > self.disparity)
        for i in jumps:
            near = min(scan[i], scan[i + 1])
            k = int(math.ceil(math.atan2(self.half_w, max(near, 0.05)) / self.step_rad))
            if scan[i] < scan[i + 1]:               # the edge is on the left of the jump: widen it to the left
                out[i + 1:i + 1 + k] = np.minimum(out[i + 1:i + 1 + k], near)
            else:
                out[max(i - k + 1, 0):i + 1] = np.minimum(out[max(i - k + 1, 0):i + 1], near)
        return out

    def gap_steer(self, scan):
        """Steer toward the deepest open direction ahead, after the edges are widened."""
        ext = np.where(self.fwd, self.extended(scan), 0.0)
        deepest = np.flatnonzero(ext >= ext.max() - 0.05)          # all about equally deep: take the one nearest ahead
        i = deepest[np.argmin(np.abs(self.angles[deepest]))]
        # aim no farther than the first thing in the car's straight path, so it turns as hard as that needs
        aim = min(ext[i], self.aim_max, max(self.path_clear(scan), 0.3))
        return math.atan2(2 * self.L * math.sin(self.angles[i]), aim)

    def path_clear(self, scan):
        """How far the car can drive straight on before its body hits something."""
        ahead = (self.cos > 0) & (np.abs(scan * self.sin) < self.step_half_w)
        return float((scan * self.cos)[ahead].min()) if ahead.any() else self.aim_max
