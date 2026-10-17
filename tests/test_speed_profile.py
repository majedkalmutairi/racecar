import numpy as np

from racer.planning.speed_profile import backward_pass, curvature, forward_pass, max_speed_from_curvature


def circle(radius, n=100):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[radius * np.cos(t), radius * np.sin(t)]


def test_curvature_of_circle():
    k = curvature(circle(2.0))
    assert np.all(np.abs(np.abs(k) - 0.5) < 0.01)

def test_corner_speed_is_sqrt_of_a_lat_times_radius():
    v = max_speed_from_curvature(np.array([0.5]), 3.0, 8.0, 1e-3)
    assert abs(v[0] - np.sqrt(3.0 * 2.0)) < 1e-9

def test_straight_is_capped():
    v = max_speed_from_curvature(np.array([0.0]), 3.0, 8.0, 1e-3)
    assert v[0] == 8.0

def test_passes_limit_speed_change_between_points():
    v = np.array([8.0] * 20 + [1.0] + [8.0] * 20)
    v = backward_pass(v.copy(), 4.0, 0.1)
    v = forward_pass(v, 2.5, 0.1)
    dv2 = np.abs(np.diff(np.r_[v, v[0]] ** 2))
    assert np.all(dv2 <= 2 * 4.0 * 0.1 + 1e-6)
