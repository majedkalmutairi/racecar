import numpy as np
from scipy.optimize import lsq_linear
from scipy.sparse import diags, vstack

from racer.loops import normals, resample, smooth_step
from racer.mapping.centerline import distance_field


def clamp_to_track(line, center, width, margin):
    """Keep every point within width/2 - margin of its centerline point."""
    d = line - center
    dist = np.linalg.norm(d, axis=1)
    limit = np.maximum(width / 2 - margin, 0.0)
    too_far = dist > limit
    d[too_far] *= (limit[too_far] / dist[too_far])[:, None]
    return center + d


def smoothed_line(center, width, cfg):
    """The simple racing line: pull every point toward its neighbours, stay inside the track. Used without a map."""
    plan = cfg["plan"]
    line = center.copy()
    for _ in range(plan["smooth_iters"]):
        line = smooth_step(line, plan["smooth_alpha"])
        line = clamp_to_track(line, center, width, plan["margin_m"])
    return resample(line, plan["ds_m"])


def sideways_room(ref, nrm, dist, grid, clear, reach=1.0):
    """How far every point may move left (hi) and right (lo) along its normal and still be `clear` from every
    mapped wall or box. Only the free stretch the point is in counts, so it never jumps into the next corridor."""
    offsets = np.arange(-reach, reach + 1e-9, grid.res / 2)
    k0 = len(offsets) // 2
    pts = ref[:, None, :] + offsets[None, :, None] * nrm[:, None, :]
    row, col = grid.world_to_cell(pts[..., 0], pts[..., 1])
    ok = dist[row, col] >= clear
    lo, hi = np.zeros(len(ref)), np.zeros(len(ref))
    for i in np.flatnonzero(ok[:, k0]):          # a point already too close to a wall stays where it is
        a = b = k0
        while a > 0 and ok[i, a - 1]:
            a -= 1
        while b < len(offsets) - 1 and ok[i, b + 1]:
            b += 1
        lo[i], hi[i] = offsets[a], offsets[b]
    return lo, hi


def min_curvature_line(center, grid, cfg):
    """Racing line that bends as little as possible inside the room the map leaves (Heilmeier et al., TUM).

    The line is the centerline moved sideways by d at every point. Its bending is the second difference of the
    points, which is linear in d, so the whole lap is one bounded least-squares problem. A small weight on the
    first difference also keeps it short. Solved again around its own result a few times, because moving the
    points changes their spacing.
    """
    plan, mapping = cfg["plan"], cfg["mapping"]
    dist = distance_field(grid.occupied(mapping["min_hits"]), grid.res)
    line = center
    for _ in range(plan["mincurv_iters"]):
        ref = resample(line, plan["mincurv_step_m"])
        nrm = normals(ref)
        n = len(ref)
        lo, hi = sideways_room(ref, nrm, dist, grid, plan["wall_clear_m"] + plan["mincurv_buffer_m"])
        bend = diags([1, -2, 1], [-1, 0, 1], shape=(n, n)).tolil()
        bend[0, n - 1] = bend[n - 1, 0] = 1                         # closed loop
        step = diags([-1, 1], [0, 1], shape=(n, n)).tolil()
        step[n - 1, 0] = 1
        bend, step = bend.tocsr(), step.tocsr() * plan["mincurv_length_w"]
        nx, ny = diags(nrm[:, 0]), diags(nrm[:, 1])
        a = vstack([bend @ nx, bend @ ny, step @ nx, step @ ny]).tocsr()
        b = -np.concatenate([bend @ ref[:, 0], bend @ ref[:, 1], step @ ref[:, 0], step @ ref[:, 1]])
        d = lsq_linear(a, b, bounds=(lo, np.maximum(hi, lo + 1e-9))).x
        line = ref + nrm * d[:, None]
    return resample(line, plan["ds_m"])


def build_racing_line(center, width, cfg, grid=None):
    if grid is None:
        return smoothed_line(center, width, cfg)
    return min_curvature_line(center, grid, cfg)
