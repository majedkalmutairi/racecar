import math

import numpy as np


def curvature(line, eps=1e-9):
    """Signed curvature (1/radius) at every point of a closed loop, from each point and its neighbours."""
    prev = np.roll(line, 1, axis=0)
    nxt = np.roll(line, -1, axis=0)
    d1 = line - prev
    d2 = nxt - line
    cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    lengths = np.linalg.norm(d1, axis=1) * np.linalg.norm(d2, axis=1) * np.linalg.norm(nxt - prev, axis=1)
    return 2 * cross / (lengths + eps)


def max_speed_from_curvature(kappa, a_lat_max, v_cap, kappa_eps):
    return np.minimum(np.sqrt(a_lat_max / np.maximum(np.abs(kappa), kappa_eps)), v_cap)


def backward_pass(v, a_brake, ds):
    """Lower each speed so the car can still brake to the next point's speed."""
    n = len(v)
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            v[i] = min(v[i], math.sqrt(v[(i + 1) % n] ** 2 + 2 * a_brake * ds))
    return v


def forward_pass(v, a_accel, ds):
    """Lower each speed so the car can actually reach it from the previous point."""
    n = len(v)
    for _ in range(2):
        for i in range(n):
            j = (i + 1) % n
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * a_accel * ds))
    return v


def build_speed_profile(line, cfg):
    plan = cfg["plan"]
    kappa = curvature(line)
    v = max_speed_from_curvature(kappa, plan["a_lat_max"], cfg["car"]["max_speed_mps"], plan["kappa_eps"])
    v = backward_pass(v, plan["a_brake"], plan["ds_m"])
    v = forward_pass(v, plan["a_accel"], plan["ds_m"])
    return v


def ideal_lap_time(v, ds):
    return float(np.sum(ds / v))
