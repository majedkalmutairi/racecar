import math

import numpy as np

from racer.control.pure_pursuit import PurePursuit, lateral_error, wrap_angle

CFG = {
    "car": {"wheelbase_m": 0.33, "max_steer_rad": 0.4, "max_speed_mps": 8.0},
    "race": {"ld_k": 0.4, "ld_min_m": 0.5, "ld_max_m": 2.0, "search_window": 30, "speed_lead_s": 0.12, "cut_max_m": 0.06, "ld_floor_m": 0.3},
    "plan": {"ds_m": 0.1, "a_brake": 5.0, "a_accel": 4.0, "a_lat_max": 6.0, "curv_span_m": 0.6, "kappa_eps": 1e-3},
}
STRAIGHT = np.c_[np.arange(0, 20, 0.1), np.zeros(200)]


def test_wrap_angle():
    assert abs(wrap_angle(3 * math.pi) - math.pi) < 1e-9 or abs(wrap_angle(3 * math.pi) + math.pi) < 1e-9
    assert abs(wrap_angle(0.3) - 0.3) < 1e-9

def test_on_the_line_steers_straight():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    _, steer = pp.step((0.0, 0.0, 0.0), 1.0)
    assert abs(steer) < 1e-9

def test_left_of_the_line_steers_right():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    _, steer = pp.step((0.0, 0.5, 0.0), 1.0)
    assert steer < 0

def circle(radius, ds=0.1):
    n = int(round(2 * np.pi * radius / ds))
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[radius * np.cos(t), radius * np.sin(t)]

def test_multiplier_scales_corner_speed():
    ring = circle(2.0)
    pp = PurePursuit(ring, np.full(len(ring), 3.0), CFG)
    pp.multiplier = np.full(len(ring), 0.5)               # half the speed = a quarter of the grip
    assert np.allclose(pp.v_target_line, 0.5 * math.sqrt(6.0 * 2.0), rtol=0.02)

def test_brakes_in_time_for_a_slow_section():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    mult = np.ones(200)
    pp.multiplier = mult
    fast = pp.v_target_line.copy()
    mult[100:] = 0.0001                                    # a section that learned to all but stop
    pp.multiplier = mult
    v = pp.v_target_line
    assert v[99] < 0.2 and v[50] < fast[50]                # slows well before the slow section
    assert np.all(np.diff(v[60:100]) <= 1e-9)

def test_relocate_finds_far_point():
    pp = PurePursuit(STRAIGHT, np.full(200, 3.0), CFG)
    pp.relocate((15.0, 0.1, 0.0))
    assert pp.idx == 150

def test_lateral_error_sign():
    assert lateral_error((1.0, 0.3, 0.0), STRAIGHT, 10) > 0
    assert lateral_error((1.0, -0.3, 0.0), STRAIGHT, 10) < 0
