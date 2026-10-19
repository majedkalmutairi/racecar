import math

import numpy as np


def curvature(line, eps=1e-9, k=1):
    """Signed curvature (1/radius) at every point of a closed loop, from each point and the points k either side."""
    prev = np.roll(line, k, axis=0)
    nxt = np.roll(line, -k, axis=0)
    d1 = line - prev
    d2 = nxt - line
    cross = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    lengths = np.linalg.norm(d1, axis=1) * np.linalg.norm(d2, axis=1) * np.linalg.norm(nxt - prev, axis=1)
    return 2 * cross / (lengths + eps)


def max_speed_from_curvature(kappa, grip, v_cap, kappa_eps):
    """v = sqrt(grip * R): the fastest a corner can be taken. grip is one number or one per point."""
    return np.minimum(np.sqrt(grip / np.maximum(np.abs(kappa), kappa_eps)), v_cap)


def grip_left(a_max, v, kappa, grip):
    """Friction ellipse: of a_max for braking or accelerating, what is left once the corner takes v^2 * kappa
    of the grip. A corner taken at the limit leaves nothing; a straight leaves all of it."""
    used = min(v * v * abs(kappa) / grip, 1.0)
    return a_max * math.sqrt(1.0 - used * used)


def backward_pass(v, a_brake, ds, kappa=None, grip=None):
    """Lower each speed so the car can still brake to the next point's speed. With kappa and grip, braking only
    gets the grip the corner leaves."""
    n = len(v)
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            a = a_brake if kappa is None else grip_left(min(a_brake, grip[j]), v[j], kappa[j], grip[j])
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * a * ds))
    return v


def forward_pass(v, a_accel, ds, kappa=None, grip=None):
    """Lower each speed so the car can actually reach it from the previous point (same grip sharing)."""
    n = len(v)
    for _ in range(2):
        for i in range(n):
            j = (i + 1) % n
            a = a_accel if kappa is None else grip_left(min(a_accel, grip[i]), v[i], kappa[i], grip[i])
            v[j] = min(v[j], math.sqrt(v[i] ** 2 + 2 * a * ds))
    return v


def build_speed_profile(line, cfg, grip=None):
    """Fastest speed at every point of the line for this grip (m/s^2, one number or one per point)."""
    plan = cfg["plan"]
    grip = np.broadcast_to(plan["a_lat_max"] if grip is None else grip, (len(line),)).astype(float)
    # read the curvature over the distance the car actually turns over (pure pursuit aims >= 0.5 m ahead),
    # so a few-centimetre kink in the line is not mistaken for a hairpin
    kappa = np.abs(curvature(line, k=max(int(round(plan["curv_span_m"] / (2 * plan["ds_m"]))), 1)))
    v = list(max_speed_from_curvature(kappa, grip, cfg["car"]["max_speed_mps"], plan["kappa_eps"]))
    v = backward_pass(v, plan["a_brake"], plan["ds_m"], kappa, grip)
    v = forward_pass(v, plan["a_accel"], plan["ds_m"], kappa, grip)
    return np.array(v)


def ideal_lap_time(v, ds):
    return float(np.sum(ds / v))
