import math

import numpy as np

from racer.control.pure_pursuit import PurePursuit, lateral_error, wrap_angle

CFG = {
    "car": {"wheelbase_m": 0.33, "max_steer_rad": 0.4},
    "race": {"ld_k": 0.4, "ld_min_m": 0.5, "ld_max_m": 2.0, "search_window": 30},
    "plan": {"ds_m": 0.1, "a_brake": 6.0},
}
STRAIGHT = np.c_[np.arange(0, 20, 0.1), np.zeros(200)]


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9 or abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9
    assert abs(wrap_angle(0.3) - 0.3) < 1e-9

def test_on_the_line_steers_straight():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    speed, steer = pp.step((0.0, 0.0, 0.0), 1.0)
    assert abs(steer) < 1e-9
    assert speed == 3.0

def test_left_of_the_line_steers_right():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    _, steer = pp.step((0.0, 0.5, 0.0), 1.0)
    assert steer < 0

def test_multiplier_scales_speed():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    pp.multiplier = np.full(200, 0.5)
    speed, _ = pp.step((1.0, 0.0, 0.0), 1.0)
    assert abs(speed - 1.5) < 1e-9

def test_brakes_in_time_for_a_slow_section():
    pp = PurePursuit(STRAIGHT, np.full(200, 6.0), CFG)
    mult = np.ones(200)
    mult[100:] = 0.2                                     # 1.2 m/s from x = 10 m
    pp.multiplier = mult
    v = pp.v_target_line
    assert v[99] < 6.0 and abs(v[99] - math.sqrt(1.2 ** 2 + 2 * 6.0 * 0.1)) < 1e-9
    assert np.all(np.diff(v[40:100]) <= 1e-9)             # slowing down before it, not at it

def test_relocate_finds_far_point():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    pp.relocate((15.0, 0.1, 0.0))
    assert pp.idx == 150

def test_lateral_error_sign():
    assert lateral_error((1.0, 0.3, 0.0), STRAIGHT, 10) > 0
    assert lateral_error((1.0, -0.3, 0.0), STRAIGHT, 10) < 0
