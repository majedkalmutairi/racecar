import numpy as np
from scipy.ndimage import distance_transform_edt

from racer.loops import normals, resample, smooth_step


def distance_field(occupied, res_m):
    """Metres from every cell to the nearest wall cell."""
    return distance_transform_edt(~occupied) * res_m


def snap_to_ridge(points, dist, grid, search_m):
    """Slide every point sideways to the cell farthest from the walls. Returns (points, width).

    The search stops at the first wall on each side, so a point never jumps across a wall.
    A point that is itself inside a wall (an obstacle that appeared later) searches the whole window.
    """
    offsets = np.arange(-search_m, search_m + grid.res / 2, grid.res)
    k0 = len(offsets) // 2
    candidates = points[:, None, :] + offsets[None, :, None] * normals(points)[:, None, :]
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
    return candidates[rows, best], 2 * d[rows, best]


def build_centerline(grid, path, cfg):
    """Explore path + map -> (centerline xy, track width at each point), spaced ds apart."""
    plan, mapping = cfg["plan"], cfg["mapping"]
    dist = distance_field(grid.occupied(mapping["min_hits"]), grid.res)
    xy = resample(np.asarray(path), plan["ds_m"])
    for _ in range(plan["snap_iters"]):
        xy, _ = snap_to_ridge(xy, dist, grid, plan["snap_search_m"])
        xy = resample(xy, plan["ds_m"])
    for _ in range(plan["center_smooth_iters"]):
        xy = smooth_step(xy, plan["smooth_alpha"])
    xy = resample(xy, plan["ds_m"])
    width = 2 * dist[grid.world_to_cell(xy[:, 0], xy[:, 1])]
    return xy, width
