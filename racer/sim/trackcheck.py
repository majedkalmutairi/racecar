"""Can the car drive this layout? Every check returns the names of the rules it breaks; [] means drivable.

The numbers come from the car: it turns no tighter than 0.78 m, it is 0.3 m wide and the planner and
the safety reflex each want room on both sides, and the lap counter must not see the start twice.
"""
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from racer.loops import normals
from racer.mapping.centerline import build_centerline
from racer.mapping.grid import OccupancyGrid
from racer.planning.racing_line import build_racing_line
from racer.planning.speed_profile import build_speed_profile, curvature
from racer.sim.lidar import Lidar
from racer.sim.trackgen import PX_PER_M, paint_box, render


class MaskTrack:
    """Just enough of Track for the lidar: a wall mask."""

    def __init__(self, wall):
        self.wall = wall
        self.h_px, self.w_px = wall.shape
        self.px_per_m = PX_PER_M


def check_layout(layout, cfg):
    g = cfg["trackgen"]
    reasons = []
    ds = cfg["plan"]["ds_m"]
    xy, width = layout.centre, layout.width_m

    reasons += check_shape(layout, g, ds)
    if reasons:
        return reasons                              # no point painting a track that crosses itself

    wall = render(layout)
    reasons += check_corridor(wall, xy, width)
    if reasons:
        return reasons

    known = layout.boxes_known + layout.slalom
    wall_known = render(layout, known)
    reasons += check_boxes(layout, wall_known, g, ds)
    if layout.boxes_drop:
        reasons += check_boxes_dropped(layout, wall_known, cfg)
    if not reasons:
        reasons += check_plan(layout, render(layout, layout.all_boxes()), cfg)
    return reasons


def check_shape(layout, g, ds):
    """Corner radii, lap length, fits in the image, no two parts of the loop too close."""
    reasons = []
    xy, width = layout.centre, layout.width_m
    if min(layout.radii) < g["min_radius_m"] - 1e-6:
        reasons.append("corner_radius")
    if np.abs(curvature(xy)).max() > 1.1 / g["min_radius_m"]:
        reasons.append("curvature")
    lap = len(xy) * ds
    if not g["lap_m"][0] <= lap <= g["lap_m"][1]:
        reasons.append("lap_length")
    w, h = layout.size_m
    edge = g["border_m"] + width / 2
    if xy[:, 0].min() < edge or xy[:, 0].max() > w - edge or xy[:, 1].min() < edge or xy[:, 1].max() > h - edge:
        reasons.append("off_image")

    # two points far apart along the loop must be at least a corridor plus a wall apart in space;
    # "far apart" starts where even the tightest legal corner could not bring them that close
    sep = width + g["wall_gap_m"]
    r = g["min_radius_m"]
    far_s = 2 * r * math.asin(min(sep / (2 * r), 1.0)) if sep < 2 * r else math.pi * r
    n = len(xy)
    d = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=2)
    k = np.arange(n)
    along = np.abs(k[:, None] - k[None, :])
    along = np.minimum(along, n - along) * ds
    if np.any((along >= far_s) & (d < sep)):
        reasons.append("passes_too_close")

    # the lap counter fires within radius_m of the start: no other part of the loop may come near it
    near_start = (along[0] > 2.0) & (d[0] < g["start_clear_m"])
    if near_start.any():
        reasons.append("start_clear")
    return reasons


def check_corridor(wall, xy, width):
    """The free space is one ring of the expected width."""
    reasons = []
    free_parts = label(~wall)[1]
    wall_parts = label(wall)[1]
    if free_parts != 1 or wall_parts != 2:
        reasons.append("topology")
    dist = distance_transform_edt(~wall) / PX_PER_M
    row, col = to_px(wall, xy)
    if dist[row, col].min() < 0.95 * width / 2:
        reasons.append("narrow")
    return reasons


def check_boxes(layout, wall_known, g, ds):
    """Every box leaves a gap the car and the planner fit through."""
    boxes = layout.all_boxes()
    if not boxes:
        return []
    xy = layout.centre
    gap_min = g["difficulty"][layout.info["difficulty"]]["gap_m"]
    wall_all = render(layout, boxes)
    if gap_profile(wall_all, xy, layout.width_m).min() < gap_min - 0.02:
        return ["box_gap"]
    return []


def gap_profile(wall, xy, width):
    """Width of the widest free passage across the track at every centreline point:
    twice the largest wall distance on a cross-section of the corridor."""
    dist = distance_transform_edt(~wall) / PX_PER_M
    nrm = normals(xy)
    offsets = np.linspace(-width / 2, width / 2, 61)
    pts = xy[:, None, :] + offsets[None, :, None] * nrm[:, None, :]
    row, col = to_px(wall, pts.reshape(-1, 2))
    d = dist[row, col].reshape(len(xy), len(offsets))
    return 2 * d.max(axis=1)


def check_boxes_dropped(layout, wall_known, cfg):
    """Dropped boxes must look like an obstacle to the change detector, not like a new track."""
    g = cfg["trackgen"]
    lidar_cfg = {"lidar": {**cfg["lidar"], "noise_m": 0.0}}
    lidar = Lidar(lidar_cfg)
    wall_drop = wall_known.copy()
    for box, _ in layout.boxes_drop:
        paint_box(wall_drop, box)
    before, after = MaskTrack(wall_known), MaskTrack(wall_drop)
    xy = layout.centre
    worst = 0.0
    for i in np.linspace(0, len(xy) - 1, 40).astype(int):
        t = xy[(i + 1) % len(xy)] - xy[i - 1]
        pose = (xy[i, 0], xy[i, 1], math.atan2(t[1], t[0]))
        r0, r1 = lidar.scan(before, *pose), lidar.scan(after, *pose)
        valid = r1 < lidar.max_range
        if valid.any():
            worst = max(worst, float(np.mean(np.abs(r1[valid] - r0[valid]) > 0.1)))
    layout.info["drop_mismatch"] = round(worst, 2)
    return ["drop_mismatch"] if worst >= g["drop_mismatch_max"] else []


def check_plan(layout, wall_all, cfg):
    """Run the real planner on the true walls. Its racing line must be one the car can follow."""
    g = cfg["trackgen"]
    grid = grid_from_mask(wall_all, cfg)
    try:
        center, width = build_centerline(grid, layout.centre, cfg)
        line = build_racing_line(center, width, cfg)
        build_speed_profile(line, cfg)
    except Exception:
        return ["plan_failed"]
    reasons = []
    # pure pursuit aims at least ld_min ahead, so judge the turn the car makes over that distance
    r_min = cfg["car"]["wheelbase_m"] / math.tan(cfg["car"]["max_steer_rad"])
    every = max(int(round(cfg["race"]["ld_min_m"] / cfg["plan"]["ds_m"])), 1)
    kappa = max(np.abs(curvature(line[k::every])).max() for k in range(every))
    layout.info["line_radius_m"] = round(1 / kappa, 2)
    if kappa > 1 / (r_min * g["line_radius_factor"]):
        reasons.append("line_too_tight")
    dist = distance_transform_edt(~wall_all) / PX_PER_M
    row, col = to_px(wall_all, line)
    clear = dist[row, col].min()
    layout.info["line_clear_m"] = round(float(clear), 2)
    if clear < g["line_clear_m"]:
        reasons.append("line_clearance")
    return reasons


def grid_from_mask(wall, cfg):
    """An occupancy grid that already knows every wall, as if a perfect explore lap had mapped it."""
    m = cfg["mapping"]
    grid = OccupancyGrid(m["size_m"], m["res_m"])
    rows, cols = np.nonzero(wall)
    x = (cols + 0.5) / PX_PER_M
    y = (wall.shape[0] - rows - 0.5) / PX_PER_M
    r, c = grid.world_to_cell(x, y)
    grid.hits[r, c] = m["min_hits"]
    return grid


def to_px(wall, xy):
    h, w = wall.shape
    col = np.clip((xy[:, 0] * PX_PER_M).astype(int), 0, w - 1)
    row = np.clip((h - xy[:, 1] * PX_PER_M).astype(int), 0, h - 1)
    return row, col

