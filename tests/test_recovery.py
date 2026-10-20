import math

import numpy as np

from racer.laps import LapCounter
from racer.mapping.grid import OccupancyGrid
from racer.safety.recovery import StuckRecovery
from racer.safety.ttc import SafetyReflex
from racer.sim.car import Car

CFG = {
    "car": {"wheelbase_m": 0.33, "max_steer_rad": 0.4, "length_m": 0.55, "width_m": 0.3, "max_speed_mps": 8,
            "max_reverse_mps": 0.5, "max_accel": 4, "max_brake": 8, "friction_decel": 1.0, "grip_mps2": 8.0},
    "lidar": {"max_range_m": 12},
    "safety": {"ttc_s": 0.6, "fov_deg": 180, "body_margin_m": 0.1, "v_min_mps": 0.05,
               "brake_speed_mps": 0.0, "hold_s": 0.5, "steer_options": 17,
               "escape_clear_m": 0.8, "stop_clear_m": 0.4, "escape_speed_mps": 1.0,
               "gap_range_m": 1.5, "bubble_m": 0.4},
    "recovery": {"stuck_s": 1.0, "stuck_v_mps": 0.05, "want_v_mps": 0.1, "reverse_mps": 0.4, "back_m": 0.5,
                 "back_step_m": 0.25, "max_attempts": 3, "spot_m": 0.5, "clear_m": 2.0, "grace_s": 1.5,
                 "rear_beam_m": 0.2, "rear_check_m": 0.1, "map_margin_m": 0.05, "no_progress_s": 30,
                 "progress_m": 1.0},
    "mapping": {"min_hits": 3, "res_m": 0.05},
    "laps": {"gate_half_m": 1.6, "min_lap_s": 3, "min_lap_m": 5},
}
ANGLES = np.linspace(-np.radians(135), np.radians(135), 108)
NOSE_IN = np.where(np.abs(ANGLES) < 0.5, 0.35, 3.0)          # something right in front, open everywhere else
DT = 1 / 60


def make():
    return StuckRecovery(CFG, SafetyReflex(ANGLES, CFG)), OccupancyGrid(20, 0.05)


def stall(rec, grid, t, pose=(0.0, 0.0, 0.0), want=(0.8, 0.2)):
    """Car wants to drive but is not moving, until recovery takes over. Returns (command, time)."""
    while t < 100:
        cmd = rec.step(want, (0.0, 0.0), NOSE_IN, pose, 0.0, t, grid)
        t += DT
        if rec.reversing or rec.gave_up:
            return cmd, t
    raise AssertionError("never reacted")


def test_stuck_car_backs_up_turning_away_from_the_open_side():
    rec, grid = make()
    cmd, t = stall(rec, grid, 0.0)
    assert 1.0 <= t < 1.1
    assert cmd[0] < 0                                          # reversing
    assert cmd[1] < 0                                          # controller wants left, so wheels right going back
    assert rec.status() == "RECOVER 1/3"


def test_back_up_stops_after_back_m_then_hands_back():
    rec, grid = make()
    _, t = stall(rec, grid, 0.0)
    assert rec.step((0.8, 0.2), (0.0, 0.0), NOSE_IN, (-0.3, 0.0, 0.0), -0.4, t, grid)[0] < 0
    cmd = rec.step((0.8, 0.2), (0.0, 0.0), NOSE_IN, (-0.55, 0.0, 0.0), -0.4, t + DT, grid)
    assert cmd == (0.0, 0.0) and not rec.reversing


def test_mapped_wall_behind_stops_the_back_up():
    rec, grid = make()
    _, t = stall(rec, grid, 0.0)
    row, col = grid.world_to_cell(np.full(9, -0.3), np.linspace(-0.2, 0.2, 9))
    grid.hits[row, col] = 3                                    # wall 0.19 m behind the rear bumper
    rec.step((0.8, 0.2), (0.0, 0.0), NOSE_IN, (-0.05, 0.0, 0.0), -0.4, t, grid)
    assert not rec.reversing


def test_gives_up_after_three_tries_at_the_same_spot():
    rec, grid = make()
    t = 0.0
    for attempt in range(3):
        _, t = stall(rec, grid, t)
        assert rec.attempts == attempt + 1
        rec.step((0.8, 0.2), (0.0, 0.0), NOSE_IN, (-1.0, 0.0, 0.0), -0.4, t, grid)   # backed up, ends
        t += 2.0                                               # past the grace time, back at the same spot
    stall(rec, grid, t)
    assert rec.gave_up and rec.status() == "STUCK"


def test_a_new_spot_far_away_starts_counting_again():
    rec, grid = make()
    _, t = stall(rec, grid, 0.0)
    rec.step((0.8, 0.2), (0.0, 0.0), NOSE_IN, (-1.0, 0.0, 0.0), -0.4, t, grid)
    rec.step((0.8, 0.2), (0.8, 0.2), NOSE_IN, (5.0, 0.0, 0.0), 1.0, t + 2.0, grid)
    _, t = stall(rec, grid, t + 2.1, pose=(5.0, 0.0, 0.0))
    assert rec.attempts == 1


def test_target_zero_never_rolls_backwards():
    car = Car(CFG)
    car.v = 0.3
    for _ in range(600):
        car.step(3.0 * (0.0 - car.v), 0.0, DT)
        assert car.v >= 0.0
    for _ in range(60):
        car.step(0.0, 0.0, DT)                                 # no command at all: friction stops at zero
        assert car.v == 0.0


def test_reverse_is_capped():
    car = Car(CFG)
    for _ in range(300):
        car.step(3.0 * (-2.0 - car.v), 0.0, DT)
    assert math.isclose(car.v, -0.5)


def test_backing_up_near_the_start_is_not_a_lap():
    laps = LapCounter((0.0, 0.0), CFG, 0.0)
    t = 0.0
    for x in list(np.linspace(0, 0.8, 30)) + list(np.linspace(0.8, 0.0, 30)) * 3:
        t += 0.2
        assert laps.update((x, 0.0, 0.0), t) is None
    assert laps.count == 0


def test_too_fast_for_the_grip_slides_wide():
    car = Car(CFG)
    car.v = 5.0
    car.step(0.0, 0.3, DT)                                      # asks for 5^2 * tan(0.3) / 0.33 = 23.4 m/s^2
    assert car.sliding
    slow = Car(CFG)
    slow.v = 1.0
    slow.step(0.0, 0.3, DT)                                     # 0.9 m/s^2: well inside the grip
    assert not slow.sliding


def test_braking_takes_its_grip_first():
    car = Car(CFG)
    car.v = 3.0
    car.step(-8.0, 0.2, DT)                                     # full brake (capped at the grip) leaves no grip to turn
    assert car.sliding and math.isclose(car.v, 3.0 - 8.0 * DT)
