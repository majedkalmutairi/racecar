import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.optimize import lsq_linear
from scipy.sparse import diags, identity, vstack

from racer.loops import normals, resample, smooth_step


def distance_field(occupied, res_m):
    """Metres from every cell to the nearest wall cell."""
    return distance_transform_edt(~occupied) * res_m


def snap_to_ridge(points, dist, grid, search_m, smooth_w):
    """Slide every point sideways toward the cell farthest from the walls. Returns (points, width).

    Each point may only move within the free stretch it is in: the search stops at the first wall on each side,
    so it never jumps across a wall or a box. Where the middle of the room jumps sideways (just before a box)
    neighbouring points would step apart, so the offsets are solved together: as close to the middle as possible
    while neighbours stay close (one bounded least-squares problem). The line then ramps round the box.
    A point that is itself inside a wall (an obstacle that appeared later) searches the whole window.
    """
    offsets = np.arange(-search_m, search_m + grid.res / 2, grid.res)
    k0 = len(offsets) // 2
    nrm = normals(points)
    candidates = points[:, None, :] + offsets[None, :, None] * nrm[:, None, :]
    row, col = grid.world_to_cell(candidates[..., 0], candidates[..., 1])
    d = dist[row, col]

    blocked = d <= 0
    inside_wall = blocked[:, k0].copy()
    blocked[:, k0] = False
    past_left_wall = np.cumsum(blocked[:, k0:], axis=1) > 0
    past_right_wall = np.cumsum(blocked[:, :k0 + 1][:, ::-1], axis=1)[:, ::-1] > 0
    d[:, k0:][past_left_wall & ~inside_wall[:, None]] = -1
    d[:, :k0 + 1][past_right_wall & ~inside_wall[:, None]] = -1

    best = np.argmax(d, axis=1)
    rows = np.arange(len(points))
    # the free stretch around each point's best cell bounds where it may go
    idx = np.arange(len(offsets))[None, :]
    wall = d <= 0
    lo = np.where(wall & (idx < best[:, None]), idx + 1, 0).max(axis=1)
    hi = np.where(wall & (idx > best[:, None]), idx - 1, len(offsets) - 1).min(axis=1)
    off = smooth_offsets(offsets[best], offsets[lo], offsets[hi], smooth_w)
    return points + off[:, None] * nrm, 2 * d[rows, best]


def smooth_offsets(target, lo, hi, w):
    """Offsets near target, within lo..hi, with neighbours (on a closed loop) pulled together by weight w."""
    n = len(target)
    step = diags([-1, 1], [0, 1], shape=(n, n)).tolil()
    step[n - 1, 0] = 1
    a = vstack([identity(n), w * step.tocsr()]).tocsr()
    b = np.concatenate([target, np.zeros(n)])
    return lsq_linear(a, b, bounds=(lo, np.maximum(hi, lo + 1e-9))).x


def remove_loops(xy, window):
    """Cut out every small loop: where the line crosses itself within `window` points, drop the points between.
    Snapping can tie one between two boxes close together, where the middle of the room jumps sideways."""
    while len(xy) > 2 * window:
        a, b = xy, np.roll(xy, -1, axis=0)                      # segment i runs from a[i] to b[i]
        k = np.arange(2, window)
        j = (np.arange(len(xy))[:, None] + k[None, :]) % len(xy)
        hit = np.argwhere(crosses(a[:, None], b[:, None], a[j], b[j]))
        if len(hit) == 0:
            break
        i, kk = hit[0]
        keep = np.ones(len(xy), bool)
        keep[(i + 1 + np.arange(k[kk])) % len(xy)] = False      # the loop: points i+1 .. j
        xy = xy[keep]
    return xy


def crosses(p1, p2, p3, p4):
    """True where segment p1-p2 crosses segment p3-p4."""
    def side(o, u, v):
        return (u[..., 0] - o[..., 0]) * (v[..., 1] - o[..., 1]) - (u[..., 1] - o[..., 1]) * (v[..., 0] - o[..., 0])
    return (side(p3, p4, p1) * side(p3, p4, p2) < 0) & (side(p1, p2, p3) * side(p1, p2, p4) < 0)


def build_centerline(grid, path, cfg):
    """Explore path + map -> (centerline xy, track width at each point), spaced ds apart."""
    plan, mapping = cfg["plan"], cfg["mapping"]
    dist = distance_field(grid.occupied(mapping["min_hits"]), grid.res)
    path = np.asarray(path)
    # the lap ends where the car crosses the finish line, which can be well to one side of where it started; closing
    # the loop there would leave a sideways step in the centerline, so the last metres back into the start are dropped
    # and the loop closes with a short straight join that the snapping below straightens out
    back = np.linalg.norm(path - path[0], axis=1) < plan["seam_m"]
    tail = len(path) - np.argmax(~back[::-1]) if (~back).any() else len(path)
    xy = resample(path[:tail], plan["ds_m"])
    window = int(plan["loop_max_m"] / plan["ds_m"])
    for _ in range(plan["snap_iters"]):
        xy, _ = snap_to_ridge(xy, dist, grid, plan["snap_search_m"], plan["snap_smooth_w"])
        xy = resample(remove_loops(xy, window), plan["ds_m"])
    for _ in range(plan["center_smooth_iters"]):
        xy = smooth_step(xy, plan["smooth_alpha"])
    xy = resample(remove_loops(xy, window), plan["ds_m"])
    width = 2 * dist[grid.world_to_cell(xy[:, 0], xy[:, 1])]
    return xy, width
