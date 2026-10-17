import math

import numpy as np

from racer.control.pure_pursuit import PurePursuit, lateral_error, wrap_angle

CFG = {
    "car": {"wheelbase_m": 0.33, "max_steer_rad": 0.4},
    "race": {"ld_k": 0.4, "ld_min_m": 0.5, "ld_max_m": 2.0, "search_window": 30},
    "plan": {"ds_m": 0.1},
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
    pp.multiplier[:] = 0.5
    speed, _ = pp.step((1.0, 0.0, 0.0), 1.0)
    assert abs(speed - 1.5) < 1e-9

def test_relocate_finds_far_point():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    pp.relocate((15.0, 0.1, 0.0))
    assert pp.idx == 150

def test_lateral_error_sign():
    assert lateral_error((1.0, 0.3, 0.0), STRAIGHT, 10) > 0
    assert lateral_error((1.0, -0.3, 0.0), STRAIGHT, 10) < 0
