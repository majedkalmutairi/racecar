import numpy as np


def resample(points, ds):
    """Points spaced every ds metres along a closed loop, starting at points[0]."""
    loop = np.vstack([points, points[:1]])
    seg = np.linalg.norm(np.diff(loop, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(round(s[-1] / ds)), 4)
    s_new = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.c_[np.interp(s_new, s, loop[:, 0]), np.interp(s_new, s, loop[:, 1])]


def smooth_step(loop, alpha):
    """Pull every point a fraction alpha toward the midpoint of its two neighbours."""
    mid = (np.roll(loop, 1, axis=0) + np.roll(loop, -1, axis=0)) / 2
    return loop + alpha * (mid - loop)


def normals(loop):
    """Unit vector pointing left of the direction of travel at every point."""
    tangent = np.roll(loop, -1, axis=0) - np.roll(loop, 1, axis=0)
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    return np.c_[-tangent[:, 1], tangent[:, 0]]
