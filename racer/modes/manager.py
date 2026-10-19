from enum import Enum
from pathlib import Path

import numpy as np

from racer.control.pure_pursuit import PurePursuit, lateral_error
from racer.csvlog import open_csv_log
from racer.interface import beam_angles
from racer.laps import LAP_FIELDS, LapCounter
from racer.learning.sections import SectionLearner
from racer.mapping.centerline import build_centerline
from racer.mapping.change_detector import ChangeDetector
from racer.mapping.grid import OccupancyGrid
from racer.modes.explore import ExploreController
from racer.planning.racing_line import build_racing_line
from racer.planning.speed_profile import build_speed_profile
from racer.safety.recovery import StuckRecovery
from racer.safety.ttc import SafetyReflex

EVENT_FIELDS = ["t", "lap", "section", "mode", "min_ttc", "steer", "kind"]


class Mode(Enum):
    EXPLORE = "EXPLORE"
    PLAN = "PLAN"
    RACE = "RACE"
    TRACK_CHANGED = "TRACK_CHANGED"


class ModeManager:
    """Decides what the car does each frame: explore the first lap, plan once, then race and learn."""

    def __init__(self, cfg, start_pose, track_name, root):
        self.cfg = cfg
        self.root = Path(root)
        self.track_name = track_name
        self.angles = beam_angles(cfg)
        self.max_range = cfg["lidar"]["max_range_m"]
        self.explore = ExploreController(cfg)
        self.safety = SafetyReflex(self.angles, cfg)
        self.recovery = StuckRecovery(cfg, self.safety)
        self.lap_file, self.lap_log = open_csv_log(self.root / cfg["laps"]["csv_path"], LAP_FIELDS)
        self.event_file, self.event_log = open_csv_log(self.root / cfg["safety"]["events_csv"], EVENT_FIELDS)
        self.lap_total = 0
        self.last_event = None
        self.safety_active = False
        self.was_crashed = False
        self.lap_crashes = 0
        self.cmd = (0.0, 0.0)
        self.sent = (0.0, 0.0)
        self.settle_until = 0.0
        self.lap_history = []      # (lap number, seconds, mode) for the dashboard
        self.counts = {}           # events per kind, for the dashboard
        self.replans = 0
        self.ahead = None
        self.track_err = 0.0
        self.last_pose = self.last_t = self.last_v = None
        self.slipping = False
        self.mode = None
        self._start_explore(start_pose, None)

    def _enter(self, mode):
        if self.mode is not None:
            print(f"[mode] {self.mode.value} -> {mode.value}")
        self.mode = mode

    def _start_explore(self, pose, t_now):
        m = self.cfg["mapping"]
        self.grid = OccupancyGrid(m["size_m"], m["res_m"])
        self.path = []
        self.laps = LapCounter(pose, self.cfg, t_now)
        self.center = self.width = self.line = self.speed = None
        self.racer = self.learner = self.detector = None
        self.new_cells = 0
        self.last_replan_t = -1e9
        self.files = []
        self._enter(Mode.EXPLORE)

    def step(self, car, t_now):
        """One frame: read the car, run the current mode, let the safety reflex veto. Returns (speed, steer)."""
        scan, pose, v = car.get_scan(), car.get_pose(), car.get_speed()
        if car.crashed and not self.was_crashed:
            self.lap_crashes += 1
            self.note("crash", t_now)
        self.was_crashed = car.crashed

        lap_time = self.laps.update(pose, t_now)
        if lap_time is not None:
            self._lap_done(lap_time)

        if self.mode is Mode.EXPLORE:
            cmd = self._explore(scan, pose)
        elif self.mode is Mode.PLAN:
            cmd = self._plan(scan, pose, t_now)
        elif self.mode is Mode.RACE:
            cmd = self._race(scan, pose, v, t_now)
        else:
            cmd = self._track_changed(pose, t_now)
        self.cmd = cmd

        if self.recovery.reversing:  # the reflex only looks forward
            out, active = cmd, False
        else:
            self.ahead = self._line_ahead(pose, v)
            out, active = self.safety.apply(cmd, scan, v, t_now, pose, self.ahead)
        if active and not self.safety_active:
            self._log_event(t_now, out[1])
        self.safety_active = active

        gave_up = self.recovery.gave_up
        out = self.recovery.step(cmd, out, scan, pose, v, t_now, self.grid)
        if self.recovery.started:
            self.note("recover", t_now, out[1])
        if self.recovery.gave_up and not gave_up:
            self.note("stuck", t_now)
            print(f"[recover] gave up at ({pose[0]:.2f}, {pose[1]:.2f})")

        self.slipping = self._slipping(pose, v, t_now)
        if self.slipping:  # lift off, like a driver would: less speed, less sideways load, grip comes back
            out = (min(out[0], v * self.cfg["slip"]["lift"]), out[1])
        if self.racer is not None:
            err = lateral_error(pose, self.line, self.racer.idx)
            trouble = active or self.recovery.reversing or self.slipping
            self.track_err = err
            if t_now < self.settle_until:
                err = 0.0                   # still getting onto a new line: being off it is not the section's fault
            self.learner.observe(self.racer.idx, err, trouble, car.crashed)
        self.sent = out
        return out

    def _slipping(self, pose, v, t_now):
        """The car turns clearly less than the steering it was given asks for: the tyres are sliding.
        Uses only the pose and the last command, so the same check works on the real car (yaw from the IMU)."""
        last_pose, last_t, last_v = self.last_pose, self.last_t, self.last_v
        self.last_pose, self.last_t, self.last_v = pose, t_now, v
        if last_pose is None or t_now <= last_t:
            return False
        v = (v + last_v) / 2                     # the turn just measured happened at the speed of the last frame
        c = self.cfg["slip"]
        car = self.cfg["car"]
        steer = max(-car["max_steer_rad"], min(car["max_steer_rad"], self.sent[1]))
        asked = v * np.tan(steer) / car["wheelbase_m"]
        got = ((pose[2] - last_pose[2] + np.pi) % (2 * np.pi) - np.pi) / (t_now - last_t)
        return v > c["min_v_mps"] and abs(asked) > c["min_rate"] and abs(got) < c["ratio"] * abs(asked)

    def _explore(self, scan, pose):
        self.grid.add_scan(scan, pose, self.angles, self.max_range)
        self.path.append(pose[:2])
        if self.laps.count >= 1:
            self._enter(Mode.PLAN)
        return self.explore.step(scan)

    def _plan(self, scan, pose, t_now):
        self.detector = ChangeDetector(self.grid, self.angles, self.cfg, self.max_range)
        self._build_plan(self.path, pose, t_now)
        self._enter(Mode.RACE)
        return self.explore.step(scan)

    def _build_plan(self, seed, pose, t_now):
        """Map + an ordered loop of seed points -> centerline, racing line, speed profile, and the objects that use them."""
        cfg = self.cfg
        self.center, self.width = build_centerline(self.grid, seed, cfg)
        self.line = build_racing_line(self.center, self.width, cfg, self.grid)
        self.speed = build_speed_profile(self.line, cfg)
        self._save_plan()

        self.racer = PurePursuit(self.line, self.speed, cfg)
        self.settle_until = t_now + cfg["learning"]["settle_s"]
        self.racer.relocate(pose)
        if self.learner is None:
            self.learner = SectionLearner(len(self.line), cfg, self._file("mult.npy"))
        else:
            self.learner.remap(len(self.line))
        self.racer.multiplier = self.learner.multiplier_per_point()
        self.new_cells = 0

    def _race(self, scan, pose, v, t_now):
        speed, steer = self.racer.step(pose, v)
        if self.detector.update(scan, pose, t_now):
            self._enter(Mode.TRACK_CHANGED)
        elif self.detector.mismatch > self.detector.threshold:
            speed = min(speed, self.explore.speed)
        else:
            self._watch_for_obstacles(scan, pose, t_now)
        return speed, steer

    def _line_ahead(self, pose, v):
        """The racing line in front of the car, if the car is on it; None means the reflex should not trust the plan."""
        if self.mode is not Mode.RACE or self.racer is None:
            return None
        r = self.cfg["race"]
        idx = self.racer.idx
        nxt = self.line[(idx + 1) % len(self.line)] - self.line[idx]
        heading_err = (pose[2] - np.arctan2(nxt[1], nxt[0]) + np.pi) % (2 * np.pi) - np.pi
        if abs(lateral_error(pose, self.line, idx)) > r["trust_err_m"] or abs(heading_err) > r["trust_heading_rad"]:
            return None
        horizon = v * self.cfg["safety"]["ttc_s"] + r["trust_horizon_m"]
        # where the car will really go (pure pursuit cuts corners), not the line itself
        ahead = self.racer.predict(pose, v, int(horizon / self.cfg["plan"]["ds_m"]) + 1)
        return np.vstack([pose[:2], ahead])

    def _watch_for_obstacles(self, scan, pose, t_now):
        """Keep mapping while racing. A cluster of new wall cells away from the walls the explore lap saw is a new
        obstacle: rebuild the plan around it. The change detector keeps judging against the explore-lap map."""
        row, col = self.grid.add_scan(scan, pose, self.angles, self.max_range)
        just_filled = self.grid.hits[row, col] == self.cfg["mapping"]["min_hits"]
        self.new_cells += int(np.sum(just_filled & ~self.detector.near_wall[row, col]))
        plan = self.cfg["plan"]
        if self.new_cells >= plan["replan_cells"] and t_now - self.last_replan_t >= plan["replan_cooldown_s"]:
            print(f"[plan] {self.new_cells} new wall cells away from the known walls: replanning around them")
            self.replans += 1
            self._build_plan(self.center, pose, t_now)
            self.last_replan_t = t_now

    def _track_changed(self, pose, t_now):
        self.reset(pose, t_now)
        return 0.0, 0.0

    def reset(self, pose, t_now):
        """Throw away the map, the plan and the learned multipliers; explore again from this pose."""
        for f in self.files:
            Path(f).unlink(missing_ok=True)
        self.recovery.reset()
        self._start_explore(pose, t_now)

    def car_was_reset(self, pose, t_now):
        """The sim put the car back at the start after a crash."""
        self.recovery.reset()
        if self.mode is Mode.EXPLORE:
            self.path = []
            self.laps = LapCounter(pose, self.cfg, t_now)
        else:
            self.laps.restart(t_now)
            if self.racer is not None:
                self.settle_until = t_now + self.cfg["learning"]["settle_s"]
                self.learner.crashed_at(self.racer.idx)
                self.racer.multiplier = self.learner.multiplier_per_point()
                self.racer.relocate(pose)

    def _lap_done(self, lap_time):
        self.lap_total += 1
        self.lap_history.append((self.lap_total, lap_time, self.mode.value))
        self.lap_log.writerow({"track": self.track_name, "lap_number": self.lap_total,
                               "lap_time_s": round(lap_time, 2), "crashes": self.lap_crashes,
                               "mode": self.mode.value})
        self.lap_file.flush()
        self.lap_crashes = 0
        if self.mode is Mode.RACE:
            clean = self.learner.end_lap()
            self.racer.multiplier = self.learner.multiplier_per_point()
            print(f"[learn] lap {self.lap_total}: {lap_time:.2f} s, {int(clean.sum())}/{self.learner.n_sections} "
                  f"sections clean, mean multiplier {self.learner.mult.mean():.2f}")

    def _log_event(self, t_now, steer):
        self.last_event = self.note("reflex", t_now, steer, self.safety.min_ttc)

    def note(self, kind, t_now, steer=0.0, min_ttc=float("nan")):
        """One row in events.csv: reflex, recover, stuck, crash, drop or swap."""
        section = int(self.learner.section_of[self.racer.idx]) if self.racer is not None else -1
        self.counts[kind] = self.counts.get(kind, 0) + 1
        row = {"t": round(t_now, 2), "lap": self.lap_total + 1, "section": section, "mode": self.mode.value,
               "min_ttc": round(min_ttc, 2), "steer": round(steer, 2), "kind": kind}
        self.event_log.writerow(row)
        self.event_file.flush()
        return row

    def _file(self, suffix):
        return self.root / self.cfg["mapping"]["map_dir"] / f"{self.track_name}_{suffix}"

    def _save_plan(self):
        self.grid.save(self._file("map.npy"))
        np.savez(self._file("center.npz"), xy=self.center, width=self.width, path=np.array(self.path))
        np.save(self._file("line.npy"), self.line)
        np.save(self._file("speed.npy"), self.speed)
        self.files = [self._file(s) for s in ("map.npy", "center.npz", "line.npy", "speed.npy", "mult.npy")]

    def status(self, t_now):
        """Everything the dashboard shows, as plain values. Read-only."""
        return {
            "mode": self.mode.value,
            "track": self.track_name,
            "lap": self.lap_total + 1,
            "lap_s": self.laps.current_lap_s(t_now),
            "last_lap_s": self.laps.last_lap_time,
            "best_lap_s": self.laps.best_lap_time,
            "v_target": self.cmd[0],
            "center": self.center,
            "line": self.line,
            "mult_per_point": self.learner.multiplier_per_point() if self.learner is not None else None,
            "lookahead": self.racer.target if self.racer is not None else None,
            "mismatch": self.detector.mismatch if self.detector is not None else None,
            "min_ttc": self.safety.min_ttc,
            "safety_active": self.safety_active,
            "last_event": self.last_event,
            "recovery": self.recovery.status(),
            "confidence": float(self.learner.confidence().mean()) if self.learner is not None else None,
            # for the bigger dashboard and its debug section
            "lap_history": list(self.lap_history),
            "counts": dict(self.counts),
            "replans": self.replans,
            "ahead": self.ahead,
            "track_err": self.track_err,
            "slipping": self.slipping,
            "lookahead_m": self.racer.ld if self.racer is not None else None,
            "v_line": self.racer.v_target_line if self.racer is not None else None,
            "plan_lap_s": float(np.sum(self.cfg["plan"]["ds_m"] / self.racer.v_target_line)) if self.racer is not None else None,
            "section": int(self.learner.section_of[self.racer.idx]) if self.racer is not None else None,
            "grip": self.cfg["plan"]["a_lat_max"] * self.learner.mult ** 2 if self.learner is not None else None,
            "ceiling_grip": self.cfg["plan"]["a_lat_max"] * self.learner.ceiling ** 2 if self.learner is not None else None,
            "trouble_grip": self.cfg["plan"]["a_lat_max"] * self.learner.trouble ** 2 if self.learner is not None else None,
            "section_conf": self.learner.confidence() if self.learner is not None else None,
            "mult": self.learner.mult if self.learner is not None else None,
            "section_of": self.learner.section_of if self.learner is not None else None,
            "section_corner": self.section_corners() if self.racer is not None else None,
        }

    def section_corners(self):
        """True for sections with a real corner in them: only there does driving faster test the grip."""
        bend = np.zeros(self.learner.n_sections)
        np.maximum.at(bend, self.learner.section_of, self.racer.kappa)
        return bend > self.cfg["dashboard"]["corner_kappa"]

    def close(self):
        self.lap_file.close()
        self.event_file.close()
