import numpy as np

from racer.planning.racing_line import build_racing_line, clamp_to_track

CFG = {"plan": {"ds_m": 0.1, "margin_m": 0.2, "smooth_iters": 200, "smooth_alpha": 0.3}}


def circle(radius, n=100):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[radius * np.cos(t), radius * np.sin(t)]


def test_clamp_keeps_points_inside():
    center = circle(2.0)
    width = np.full(100, 1.0)
    line = center * 1.5
    clamped = clamp_to_track(line, center, width, 0.2)
    assert np.all(np.linalg.norm(clamped - center, axis=1) <= 0.3 + 1e-9)

def test_racing_line_cuts_inside_and_stays_inside():
    center = circle(2.0, 126)
    width = np.full(len(center), 1.2)
    line = build_racing_line(center, width, CFG)
    r = np.linalg.norm(line, axis=1)
    assert np.all(r >= 2.0 - 0.4 - 0.01)
    assert r.mean() < 2.0
