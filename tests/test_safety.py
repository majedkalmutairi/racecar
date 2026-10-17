import math

import numpy as np

from racer.safety.ttc import SafetyReflex

CFG = {
    "car": {"wheelbase_m": 0.33, "max_steer_rad": 0.4, "length_m": 0.55, "width_m": 0.3},
    "safety": {"ttc_s": 0.6, "fov_deg": 180, "body_margin_m": 0.1, "v_min_mps": 0.05,
               "brake_speed_mps": 0.0, "hold_s": 0.5, "steer_options": 17,
               "escape_clear_m": 0.8, "stop_clear_m": 0.4, "escape_speed_mps": 1.0,
               "gap_range_m": 1.5, "bubble_m": 0.4},
}
ANGLES = np.linspace(-np.radians(135), np.radians(135), 108)
NOSE = 0.33 / 2 + 0.55 / 2


def beam(deg):
    return int(np.argmin(np.abs(ANGLES - np.radians(deg))))


def test_safety_far_walls_do_not_fire():
    r = SafetyReflex(ANGLES, CFG)
    assert r.path_clearance(np.full(108, 10.0), 0.0) / 2.0 > 0.6

def test_safety_wall_ahead_gives_the_right_ttc():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 10.0)
    scan[54] = 1.0
    expected = 1.0 * math.cos(ANGLES[54]) - NOSE
    assert abs(r.path_clearance(scan, 0.0) - expected) < 1e-9

def test_safety_slow_is_safe():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 10.0)
    scan[54] = 1.0
    assert r.path_clearance(scan, 0.0) / 0.1 > 0.6

def test_safety_walls_beside_the_path_do_not_count():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 10.0)
    scan[beam(45)] = 1.0
    scan[beam(-45)] = 1.0
    assert r.path_clearance(scan, 0.0) / 4.0 > 0.6

def test_safety_steering_into_a_wall_counts():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 10.0)
    scan[beam(30)] = 1.0
    assert r.path_clearance(scan, 0.0) / 2.0 > 0.6
    assert r.path_clearance(scan, 0.3) / 2.0 < 0.6

def test_safety_slows_to_the_speed_that_keeps_the_path_clear():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 10.0)
    scan[beam(-10):beam(10)] = 1.5
    cmd, active = r.apply((3.0, 0.0), scan, 3.0, t_now=1.0)
    assert active and cmd[1] == 0.0
    assert abs(cmd[0] - r.path_clearance(scan, 0.0) / 0.6) < 1e-9 and 1.5 < cmd[0] < 2.0


def test_safety_escapes_toward_open_space_when_the_path_is_blocked():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 10.0)
    scan[beam(-10):beam(10)] = 1.0
    scan[beam(-90):beam(-20)] = 0.7
    cmd, active = r.apply((3.0, 0.0), scan, 2.0, t_now=1.0)
    assert active and cmd[1] > 0.1 and 0 < cmd[0] <= 1.0
    cmd, active = r.apply((3.0, 0.0), np.full(108, 10.0), 0.0, t_now=1.6)
    assert not active and cmd == (3.0, 0.0)


def test_safety_escape_goes_straight_when_everything_is_open():
    r = SafetyReflex(ANGLES, CFG)
    assert abs(r.escape_steer(np.full(108, 10.0))) < 0.05


def test_safety_escape_turns_away_from_the_nearest_wall():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 3.0)
    scan[beam(-60):beam(-20)] = 0.5
    assert r.escape_steer(scan) > 0.1


def test_safety_stops_when_nothing_is_clear():
    r = SafetyReflex(ANGLES, CFG)
    scan = np.full(108, 0.5)
    cmd, active = r.apply((3.0, 0.0), scan, 0.0, t_now=1.0)
    assert active and cmd[0] == 0.0
