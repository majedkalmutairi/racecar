import numpy as np

from racer.loops import resample, smooth_step


def circle(radius, n=100):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[radius * np.cos(t), radius * np.sin(t)]


def test_resample_spacing():
    pts = resample(circle(2.0, 50), 0.1)
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    assert abs(seg.mean() - 0.1) < 0.01 and seg.std() < 0.01
    assert np.allclose(pts[0], circle(2.0, 50)[0])

def test_smoothing_shrinks_a_wiggle():
    line = circle(2.0)
    wiggly = line + 0.05 * np.sin(np.arange(100))[:, None]
    smoother = smooth_step(wiggly, 0.3)
    assert np.linalg.norm(smoother - line) < np.linalg.norm(wiggly - line)
