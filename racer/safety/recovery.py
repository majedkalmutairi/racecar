import math

import numpy as np


class StuckRecovery:
    """Backs the car out when it wants to drive but cannot move, e.g. parked nose-in by the safety reflex.

    The lidar cannot see straight behind the car, so every back-up is short and checked against the map
    built so far. A few tries at the same spot, then it gives up and says so.
    """

    def __init__(self, cfg, safety):
        r = cfg["recovery"]
        car = cfg["car"]
        self.safety = safety
        self.stuck_s = r["stuck_s"]
        self.stuck_v = r["stuck_v_mps"]
        self.want_v = r["want_v_mps"]
        self.reverse_v = r["reverse_mps"]
        self.back_m = r["back_m"]
        self.back_step_m = r["back_step_m"]
        self.max_attempts = r["max_attempts"]
        self.spot_m = r["spot_m"]
        self.clear_m = r["clear_m"]
        self.grace_s = r["grace_s"]
        self.rear_beam_m = r["rear_beam_m"]
        self.check_m = r["rear_check_m"]
        self.no_progress_s = r["no_progress_s"]
        self.progress_m = r["progress_m"]
        self.min_hits = cfg["mapping"]["min_hits"]
        self.L = car["wheelbase_m"]
        self.max_steer = car["max_steer_rad"]
        self.left = safety.fwd & (safety.angles > 0)
        self.right = safety.fwd & (safety.angles < 0)
        self.outline = body_outline(car["length_m"], car["width_m"], car["wheelbase_m"] / 2, 0.0)
        self.margin = r["map_margin_m"] + cfg["mapping"]["res_m"] / 2     # cell centres sit half a cell inside the wall
        self.reset()

    def reset(self):
        self.state = "drive"
        self.slow_since = None
        self.spot = None
        self.attempts = 0
        self.grace_until = -1.0
        self.gave_up = False
        self.progress_ref = None
        self.progress_t = None
        self.started = False

    @property
    def reversing(self):
        return self.state == "reverse"

    def step(self, want, out, scan, pose, v, t_now, grid):
        """want = what the mode controller asked for, out = after the safety reflex. Returns the command to send."""
        self.started = False
        self._watch_progress(pose, t_now)
        if self.spot is not None and math.hypot(pose[0] - self.spot[0], pose[1] - self.spot[1]) > self.clear_m:
            self.spot = None  # got well away: the next stuck is a fresh one
        if self.reversing:
            return self._reverse(scan, pose, t_now, grid)
        if self.gave_up:
            return out

        if want[0] > self.want_v and abs(v) < self.stuck_v and t_now >= self.grace_until:
            if self.slow_since is None:
                self.slow_since = t_now
            elif t_now - self.slow_since >= self.stuck_s:
                return self._start(want, scan, pose, t_now, out)
        else:
            self.slow_since = None
        return out

    def _start(self, want, scan, pose, t_now, out):
        self.slow_since = None
        if self.spot is None or math.hypot(pose[0] - self.spot[0], pose[1] - self.spot[1]) > self.spot_m:
            self.spot = pose[:2]
            self.attempts = 0
        if self.attempts >= self.max_attempts:
            self.gave_up = True
            return out
        self.attempts += 1
        self.started = True
        self.state = "reverse"
        self.back_from = pose[:2]
        self.back_dist = self.back_m + (self.attempts - 1) * self.back_step_m
        self.give_up_t = t_now + self.back_dist / self.reverse_v + 2.0
        # backing up with the wheels turned the other way swings the nose toward the open side
        self.steer = -self.open_side(want, scan) * self.max_steer
        return -self.reverse_v, self.steer

    def open_side(self, want, scan):
        """+1 if the way out is to the left, -1 if to the right. The controller knows where it is trying to go
        (pure pursuit aims through the gap the plan found); the reflex's gap search is the fallback."""
        if abs(want[1]) > 0.05:
            return math.copysign(1.0, want[1])
        escape = self.safety.escape_steer(scan)
        if abs(escape) > 0.05:
            return math.copysign(1.0, escape)
        return 1.0 if scan[self.left].mean() >= scan[self.right].mean() else -1.0

    def _reverse(self, scan, pose, t_now, grid):
        moved = math.hypot(pose[0] - self.back_from[0], pose[1] - self.back_from[1])
        if moved >= self.back_dist or t_now >= self.give_up_t or self.rear_blocked(scan, pose, grid):
            self.state = "drive"
            self.grace_until = t_now + self.grace_s
            return 0.0, 0.0
        return -self.reverse_v, self.steer

    def rear_blocked(self, scan, pose, grid):
        """True if the last beams on either side see something close, or backing up would bring any part of the
        body closer to a mapped wall while it is already within the margin. Parts that are close may only move
        away: the nose is often right against the thing the car is backing out from."""
        if min(scan[:2].min(), scan[-2:].min()) < self.rear_beam_m:
            return True
        walls = self.walls_near(pose, grid)
        if len(walls) == 0:
            return False
        now = self.clearance(pose, walls)
        after = self.clearance(back_along_arc(pose, self.check_m, self.steer, self.L), walls)
        return bool(np.any((after < self.margin) & (after < now - 1e-3)))

    def walls_near(self, pose, grid, reach=1.2):
        """Centres of the mapped wall cells within reach metres of the car."""
        r0, c0 = grid.world_to_cell(pose[0] - reach, pose[1] - reach)
        r1, c1 = grid.world_to_cell(pose[0] + reach, pose[1] + reach)
        rows, cols = np.nonzero(grid.hits[r0:r1 + 1, c0:c1 + 1] >= self.min_hits)
        x, y = grid.cell_to_world(rows + r0, cols + c0)
        return np.c_[x, y]

    def clearance(self, pose, walls):
        """Distance from every point of the body outline to the nearest wall cell centre."""
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        pts = np.c_[x + self.outline[:, 0] * c - self.outline[:, 1] * s, y + self.outline[:, 0] * s + self.outline[:, 1] * c]
        return np.linalg.norm(pts[:, None, :] - walls[None, :, :], axis=2).min(axis=1)

    def _watch_progress(self, pose, t_now):
        """Gives up if the car has not got progress_m away from where it was for no_progress_s."""
        if self.progress_ref is None or math.hypot(pose[0] - self.progress_ref[0],
                                                   pose[1] - self.progress_ref[1]) > self.progress_m:
            self.progress_ref = pose[:2]
            self.progress_t = t_now
        elif t_now - self.progress_t > self.no_progress_s:
            self.gave_up = True

    def status(self):
        if self.gave_up:
            return "STUCK"
        if self.reversing:
            return f"RECOVER {self.attempts}/{self.max_attempts}"
        return None


def body_outline(length, width, centre_ahead, margin, step=0.05):
    """Points around the car body (grown by margin), in the rear-axle frame."""
    hl, hw = length / 2 + margin, width / 2 + margin
    xs = np.arange(-hl, hl + 1e-9, step) + centre_ahead
    ys = np.arange(-hw, hw + 1e-9, step)
    edges = [np.c_[xs, np.full_like(xs, hw)], np.c_[xs, np.full_like(xs, -hw)],
             np.c_[np.full_like(ys, xs[0]), ys], np.c_[np.full_like(ys, xs[-1]), ys]]
    return np.vstack(edges)


def back_along_arc(pose, dist, steer, wheelbase, n=5):
    """Pose after reversing dist metres with this steer (bicycle model, small steps)."""
    x, y, yaw = pose
    ds = dist / n
    for _ in range(n):
        x -= ds * math.cos(yaw)
        y -= ds * math.sin(yaw)
        yaw -= ds / wheelbase * math.tan(steer)
    return x, y, yaw
