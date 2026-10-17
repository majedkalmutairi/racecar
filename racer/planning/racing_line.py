import numpy as np

from racer.loops import resample, smooth_step


def clamp_to_track(line, center, width, margin):
    """Keep every point within width/2 - margin of its centerline point."""
    d = line - center
    dist = np.linalg.norm(d, axis=1)
    limit = np.maximum(width / 2 - margin, 0.0)
    too_far = dist > limit
    d[too_far] *= (limit[too_far] / dist[too_far])[:, None]
    return center + d


def build_racing_line(center, width, cfg):
    plan = cfg["plan"]
    line = center.copy()
    for _ in range(plan["smooth_iters"]):
        line = smooth_step(line, plan["smooth_alpha"])
        line = clamp_to_track(line, center, width, plan["margin_m"])
    return resample(line, plan["ds_m"])
