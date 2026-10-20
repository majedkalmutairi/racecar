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
    mapped wall or box. Only the free stretch the point is in counts, so it never jumps into the next corridor.

    A stretch of points that is itself too close to something (a box dropped onto the line after the map was made)
    gets the free room beside it instead, all on the same side, so the line goes round one side of the box."""
    offsets = np.arange(-reach, reach + 1e-9, grid.res / 2)
    k0 = len(offsets) // 2
    pts = ref[:, None, :] + offsets[None, :, None] * nrm[:, None, :]
    row, col = grid.world_to_cell(pts[..., 0], pts[..., 1])
    d = dist[row, col]
    ok = d >= clear
    start = np.full(len(ref), k0)
    for run in runs(~ok[:, k0]):
        sides = {step: [first_free(ok[i], d[i] <= 0, k0, step) for i in run] for step in (1, -1)}
        # the side where more of the stretch has room, and then the nearer room
        step = max(sides, key=lambda s: (sum(j is not None for j in sides[s]),
                                         -sum(abs(j - k0) for j in sides[s] if j is not None)))
        for i, j in zip(run, sides[step]):
            if j is not None:
                start[i] = j
    lo, hi = np.zeros(len(ref)), np.zeros(len(ref))
    for i in np.flatnonzero(ok[np.arange(len(ref)), start]):     # no room on either side: the point stays put
        a = b = start[i]
        while a > 0 and ok[i, a - 1]:
            a -= 1
        while b < len(offsets) - 1 and ok[i, b + 1]:
            b += 1
        lo[i], hi[i] = offsets[a], offsets[b]
    return lo, hi


def runs(mask):
    """The stretches of a closed loop where mask is True, as index arrays."""
    if not mask.any() or mask.all():
        return []
    idx = np.roll(np.arange(len(mask)), -int(np.argmin(mask)))     # start the loop on a False point
    edges = np.flatnonzero(np.diff(np.r_[0, mask[idx].astype(int), 0]))
    return [idx[a:b] for a, b in zip(edges[::2], edges[1::2])]


def first_free(ok, wall, k0, step):
    """Index of the first ok cell going from k0 in direction step (+1 left, -1 right), or None. It may leave the
    box the point is in, but never crosses a wall: beyond one lies the next corridor or the unmapped outside."""
    j = k0
    while 0 <= j < len(ok) and wall[j]:
        j += step
    while 0 <= j < len(ok) and not wall[j]:
        if ok[j]:
            return j
        j += step
    return None


def solve_offsets(ref, nrm, lo, hi, length_w):
    """One bounded least-squares solve: sideways offsets d that make ref + d * nrm bend least, plus a little length,
    with lo <= d <= hi."""
    n = len(ref)
    bend = diags([1, -2, 1], [-1, 0, 1], shape=(n, n)).tolil()
    bend[0, n - 1] = bend[n - 1, 0] = 1                             # closed loop
    bend = bend.tocsr()
    step = diags([-1, 1], [0, 1], shape=(n, n)).tolil()
    step[n - 1, 0] = 1
    step = step.tocsr() * length_w
    nx, ny = diags(nrm[:, 0]), diags(nrm[:, 1])
    a = vstack([bend @ nx, bend @ ny, step @ nx, step @ ny]).tocsr()
    b = -np.concatenate([bend @ ref[:, 0], bend @ ref[:, 1], step @ ref[:, 0], step @ ref[:, 1]])
    return lsq_linear(a, b, bounds=(lo, np.maximum(hi, lo + 1e-9))).x


def min_curvature_line(center, grid, cfg):
    """Racing line that bends as little as possible inside the room the map leaves (Heilmeier et al., TUM).

    The line is the centerline moved sideways by d at every point. Its bending is the second difference of the
    points, which is linear in d, so the whole lap is one bounded least-squares problem. A small weight on the
    first difference also keeps it short. Solved again around its own result a few times, because moving the
    points changes their spacing. (Weighting the bending by how slow the car is there planned up to 2 % faster
    on twisty tracks but drove no faster, so it was left out.)
    """
    plan, mapping = cfg["plan"], cfg["mapping"]
    dist = distance_field(grid.occupied(mapping["min_hits"]), grid.res)
    line = center
    for _ in range(plan["mincurv_iters"]):
        ref = resample(line, plan["mincurv_step_m"])
        nrm = normals(ref)
        lo, hi = sideways_room(ref, nrm, dist, grid, plan["wall_clear_m"] + plan["mincurv_buffer_m"],
                               plan["room_search_m"])
        line = ref + nrm * solve_offsets(ref, nrm, lo, hi, plan["mincurv_length_w"])[:, None]
    return resample(line, plan["ds_m"])


def build_racing_line(center, width, cfg, grid=None):
    """Without a map: the smoothed line. With one: the minimum-curvature line."""
    if grid is None:
        return smoothed_line(center, width, cfg)
    return min_curvature_line(center, grid, cfg)
