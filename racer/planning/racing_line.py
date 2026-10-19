import numpy as np

from racer.loops import resample, smooth_step
from racer.mapping.centerline import distance_field


def clamp_to_track(line, center, width, margin):
    """Keep every point within width/2 - margin of its centerline point."""
    d = line - center
    dist = np.linalg.norm(d, axis=1)
    limit = np.maximum(width / 2 - margin, 0.0)
    too_far = dist > limit
    d[too_far] *= (limit[too_far] / dist[too_far])[:, None]
    return center + d


def push_off_walls(line, dist, grid, clear):
    """Move every point that is closer than `clear` to a wall straight away from it, by the missing distance.
    The width clamp only knows the track width at the centerline; this knows the real distance to every wall."""
    row, col = grid.world_to_cell(line[:, 0], line[:, 1])
    d = dist[row, col]
    close = d < clear
    if not close.any():
        return line
    r, c = row[close], col[close]
    n = dist.shape[0]
    gx = (dist[r, np.minimum(c + 1, n - 1)] - dist[r, np.maximum(c - 1, 0)]) / 2    # downhill = toward the wall
    gy = (dist[np.minimum(r + 1, n - 1), c] - dist[np.maximum(r - 1, 0), c]) / 2
    g = np.c_[gx, gy]
    norm = np.linalg.norm(g, axis=1, keepdims=True)
    line = line.copy()
    line[close] += g / np.maximum(norm, 1e-9) * (clear - d[close])[:, None]
    return line


def build_racing_line(center, width, cfg, grid=None):
    plan = cfg["plan"]
    dist = distance_field(grid.occupied(cfg["mapping"]["min_hits"]), grid.res) if grid is not None else None
    line = center.copy()
    for _ in range(plan["smooth_iters"]):
        line = smooth_step(line, plan["smooth_alpha"])
        line = clamp_to_track(line, center, width, plan["margin_m"])
        if dist is not None:
            line = push_off_walls(line, dist, grid, plan["wall_clear_m"])
    return resample(line, plan["ds_m"])
