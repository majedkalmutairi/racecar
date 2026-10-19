"""Track geometry and the random track generator.

A track is a closed centreline painted as a white corridor on black. A random track is a random polygon
with rounded corners; trackcheck.check_layout() decides whether the car can really drive it.

Run from the repo root:  python -m racer.sim.trackgen --gen-seed 42 --difficulty hard --size medium --preview
"""
import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import label

from racer.loops import normals, resample
from racer.planning.speed_profile import curvature

ROOT = Path(__file__).resolve().parents[2]
TRACKS_DIR = ROOT / "tracks"
CONFIG_PATH = ROOT / "racer" / "config.yaml"
PX_PER_M = 100
MODES = ("known", "drop", "slalom")


def corridor(draw, points_m, width_m, h_px, px_per_m=PX_PER_M):
    """Paint a white band of constant width along a closed centreline (list of (x, y) in metres)."""
    r = width_m / 2 * px_per_m
    pts = list(points_m) + [points_m[0]]
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.02))     # a disc every 2 cm
        for t in np.linspace(0, 1, n, endpoint=False):
            cx, cy = (x0 + t * (x1 - x0)) * px_per_m, h_px - (y0 + t * (y1 - y0)) * px_per_m
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill="white")


def arc(cx, cy, r, a0, a1, n=200):
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


def fillets(corners, r, max_frac=0.45):
    """Every corner of a closed polygon as an arc: (centre, radius, start angle, sweep).

    r is one radius or one per corner. A radius shrinks where the sides are too short for it (an arc may use
    at most max_frac of each side), which is why the effective radius is returned: the checker needs the real one.
    """
    n = len(corners)
    rs = list(r) if np.ndim(r) else [r] * n
    out = []
    for i in range(n):
        p = np.array(corners[i], float)
        u = np.array(corners[i - 1]) - p
        v = np.array(corners[(i + 1) % n]) - p
        len_u, len_v = np.linalg.norm(u), np.linalg.norm(v)
        u, v = u / len_u, v / len_v
        half = math.acos(np.clip(np.dot(u, v), -1, 1)) / 2
        t = min(rs[i] / math.tan(half), max_frac * len_u, max_frac * len_v)
        radius = t * math.tan(half)
        bisector = (u + v) / np.linalg.norm(u + v)
        centre = p + bisector * radius / math.sin(half)
        a0 = math.atan2(*(p + u * t - centre)[::-1])
        a1 = math.atan2(*(p + v * t - centre)[::-1])
        sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        out.append((centre, radius, a0, sweep))
    return out


def rounded(corners, r, step_m=None, max_frac=0.45):
    """Closed polygon -> the same loop with every corner replaced by an arc of radius r (smaller if the sides are short).
    Each arc gets 12 points, or one every step_m if given (smooth enough to measure curvature on)."""
    pts = []
    for centre, radius, a0, sweep in fillets(corners, r, max_frac):
        n = 12 if step_m is None else max(12, int(abs(sweep) * radius / step_m) + 1)
        pts += [tuple(centre + radius * np.array([math.cos(a0 + f * sweep), math.sin(a0 + f * sweep)]))
                for f in np.linspace(0, 1, n)]
    return pts


@dataclass
class Layout:
    name: str
    size_m: tuple
    width_m: float
    centre: np.ndarray            # closed centreline, ds_m apart, in driving order
    radii: list                   # effective radius of every rounded corner
    start_pose: tuple
    boxes_known: list = field(default_factory=list)   # [x, y, w, h], in place before the explore lap
    boxes_drop: list = field(default_factory=list)    # ([x, y, w, h], at_lap)
    slalom: list = field(default_factory=list)        # [x, y, w, h], known at start unless slalom_drop
    info: dict = field(default_factory=dict)

    @property
    def px(self):
        return int(round(self.size_m[0] * PX_PER_M)), int(round(self.size_m[1] * PX_PER_M))

    def all_boxes(self):
        return self.boxes_known + self.slalom + [b for b, _ in self.boxes_drop]


def render(layout, boxes=()):
    """Layout -> wall mask [row, col], True = wall, same pixel rules as Track."""
    w_px, h_px = layout.px
    img = Image.new("RGB", (w_px, h_px), "black")
    corridor(ImageDraw.Draw(img), [tuple(p) for p in layout.centre], layout.width_m, h_px)
    wall = np.asarray(img)[:, :, 0] < 128
    for box in boxes:
        paint_box(wall, box)
    return wall


def paint_box(wall, box, px_per_m=PX_PER_M):
    """Axis-aligned box [x, y, w, h] in metres into a wall mask, clipped to the image."""
    h_px, w_px = wall.shape
    x, y, w, h = box
    left = max(int(x * px_per_m), 0)
    right = min(int((x + w) * px_per_m), w_px)
    top = max(int(h_px - (y + h) * px_per_m), 0)
    bottom = min(int(h_px - y * px_per_m), h_px)
    if left < right and top < bottom:
        wall[top:bottom, left:right] = True


def star_corners(rng, n, lo, hi, reach_range):
    """Easy and medium tracks: n jittered angles around the middle, each corner a random way out toward the edge."""
    centre = (lo + hi) / 2
    half = (hi - lo) / 2
    theta = rng.uniform(0, 2 * math.pi) + 2 * math.pi * (np.arange(n) + rng.uniform(-0.25, 0.25, n)) / n
    corners = []
    for t in theta:
        d = np.array([math.cos(t), math.sin(t)])
        reach = min(half[0] / max(abs(d[0]), 1e-9), half[1] / max(abs(d[1]), 1e-9))   # to the box edge
        corners.append(tuple(centre + d * reach * rng.uniform(*reach_range)))
    return corners


def lattice(rng, lo, hi, cell):
    """Grid lines from lo to hi with random spacings in [cell[0], cell[1]], placed at a random offset."""
    span = hi - lo
    n = int(span // cell[0])
    if n < 1:
        return None
    gaps = rng.uniform(cell[0], cell[1], n)
    if gaps.sum() > span:
        gaps = cell[0] + (gaps - cell[0]) * (span - n * cell[0]) / (gaps.sum() - n * cell[0])
    return lo + rng.uniform(0, max(span - gaps.sum(), 0.0)) + np.concatenate([[0.0], np.cumsum(gaps)])


def grow_cells(rng, nx, ny, k):
    """A random blob of k grid cells with no holes and no two cells touching only at a corner,
    so its outline is one simple loop that never comes closer to itself than one cell."""
    cells = {(int(rng.integers(nx)), int(rng.integers(ny)))}
    for _ in range(40 * k):
        if len(cells) >= k:
            break
        cx, cy = list(cells)[int(rng.integers(len(cells)))]
        dx, dy = [(1, 0), (-1, 0), (0, 1), (0, -1)][int(rng.integers(4))]
        new = (cx + dx, cy + dy)
        if 0 <= new[0] < nx and 0 <= new[1] < ny and new not in cells and blob_ok(cells | {new}, nx, ny):
            cells.add(new)
    return cells


def blob_ok(cells, nx, ny):
    occ = np.zeros((nx + 2, ny + 2), bool)
    for x, y in cells:
        occ[x + 1, y + 1] = True
    a, b, c, d = occ[:-1, :-1], occ[1:, :-1], occ[:-1, 1:], occ[1:, 1:]
    if np.any((a & d & ~b & ~c) | (b & c & ~a & ~d)):          # diagonal-only contact
        return False
    return label(~occ, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])[1] == 1     # no holes


def outline(cells, xs, ys):
    """Corners of the blob's outline, counter-clockwise, as (x, y) in metres; straight-through points dropped."""
    edges = set()
    for x, y in cells:
        for a, b in [((x, y), (x + 1, y)), ((x + 1, y), (x + 1, y + 1)),
                     ((x + 1, y + 1), (x, y + 1)), ((x, y + 1), (x, y))]:
            if (b, a) in edges:
                edges.remove((b, a))                            # shared with a neighbour: inside the blob
            else:
                edges.add((a, b))
    succ = dict(edges)
    start = min(succ)
    loop, p = [start], succ[start]
    while p != start:
        loop.append(p)
        p = succ[p]
    corners = []
    for i, p in enumerate(loop):
        q, r = loop[i - 1], loop[(i + 1) % len(loop)]
        if (p[0] - q[0]) * (r[1] - p[1]) - (p[1] - q[1]) * (r[0] - p[0]) != 0:
            corners.append((float(xs[p[0]]), float(ys[p[1]])))
    return corners


def grid_corners(rng, d, lo, hi, width, wall_gap):
    """Medium/hard tracks: the outline of a random blob of cells, corners nudged so it is not all right angles."""
    xs, ys = lattice(rng, lo[0], hi[0], d["cell_m"]), lattice(rng, lo[1], hi[1], d["cell_m"])
    if xs is None or ys is None:
        return None
    nx, ny = len(xs) - 1, len(ys) - 1
    k = int(round(rng.uniform(*d["cells"]) * nx * ny))          # a fraction of the grid, never all of it
    cells = grow_cells(rng, nx, ny, min(max(k, 3), nx * ny - 1))
    if len(cells) < 3:
        return None
    nudge = max(min(d["jitter"] * d["cell_m"][0], (d["cell_m"][0] - width - wall_gap) / 2 - 0.05), 0.0)
    return [tuple(np.clip((x + rng.uniform(-nudge, nudge), y + rng.uniform(-nudge, nudge)), lo, hi))
            for x, y in outline(cells, xs, ys)]


def straight_runs(xy, ds, kappa_max):
    """(start index, length in points) of every run of near-zero curvature, longest first, wrapping around."""
    straight = np.abs(curvature(xy)) < kappa_max
    if straight.all():
        return [(0, len(xy))]
    shift = int(np.argmin(straight))                 # start the scan on a curved point so no run is split
    s = np.roll(straight, -shift)
    runs, i = [], 0
    while i < len(s):
        if s[i]:
            j = i
            while j < len(s) and s[j]:
                j += 1
            runs.append(((i + shift) % len(s), j - i))
            i = j
        else:
            i += 1
    return sorted(runs, key=lambda r: -r[1])


def place_box(xy, nrm, i, side, width, gap, box_wh):
    """Axis-aligned box next to centreline point i, on side +1 (left) or -1 (right), leaving `gap` metres
    between the box and the far wall."""
    t = xy[(i + 1) % len(xy)] - xy[i - 1]
    bw, bh = box_wh if abs(t[0]) >= abs(t[1]) else box_wh[::-1]   # long side along the track
    n = nrm[i]
    extent = abs(n[0]) * bw / 2 + abs(n[1]) * bh / 2               # half the box, measured across the track
    off = gap - width / 2 + extent
    cx, cy = xy[i] + side * off * n
    return [round(float(cx - bw / 2), 3), round(float(cy - bh / 2), 3), float(bw), float(bh)]


def generate(seed, difficulty, size, modes, cfg, size_m=None):
    """Try seed-derived candidates until one passes check_layout. Returns (layout, rejection counts)."""
    from racer.sim.trackcheck import check_layout

    g = cfg["trackgen"]
    size_m = tuple(size_m) if size_m else tuple(g["sizes"][size])
    rejected = {}
    for attempt in range(g["max_tries"]):
        rng = np.random.default_rng([seed, attempt])
        layout = candidate(rng, seed, difficulty, size, size_m, modes, cfg)
        reasons = ["geometry"] if layout is None else check_layout(layout, cfg)
        if not reasons:
            layout.info["attempts"] = attempt + 1
            return layout, rejected
        for r in reasons:
            rejected[r] = rejected.get(r, 0) + 1
    raise RuntimeError(f"no valid {difficulty} {size} track for seed {seed} in {g['max_tries']} tries: {rejected}")


def candidate(rng, seed, difficulty, size, size_m, modes, cfg):
    g = cfg["trackgen"]
    d = g["difficulty"][difficulty]
    ds = cfg["plan"]["ds_m"]
    width = round(float(rng.uniform(*d["width_m"])), 2)
    inset = g["border_m"] + width / 2
    lo, hi = np.array([inset, inset]), np.array(size_m) - inset
    if d["shape"] == "star":
        corners = star_corners(rng, int(rng.integers(d["corners"][0], d["corners"][1] + 1)), lo, hi, d["reach"])
    else:
        corners = grid_corners(rng, d, lo, hi, width, g["wall_gap_m"])
    if not corners or len(corners) < 3:
        return None
    n = len(corners)
    radii = rng.uniform(*d["radius_m"], n)
    with np.errstate(all="raise"):
        try:
            arcs = fillets(corners, radii, g["arc_frac"])
        except (ValueError, ZeroDivisionError, FloatingPointError):
            return None
    xy = resample(np.array(rounded(corners, radii, step_m=0.02, max_frac=g["arc_frac"])), ds)
    if rng.random() < 0.5:
        xy = xy[::-1].copy()                                     # drive it clockwise half the time

    runs = straight_runs(xy, ds, g["straight_kappa"])
    i0, run = runs[0]
    i_start = (i0 + run // 2) % len(xy)
    xy = np.roll(xy, -i_start, axis=0)                          # index 0 = start, middle of the longest straight
    t = xy[1] - xy[-1]
    start = (round(float(xy[0, 0]), 3), round(float(xy[0, 1]), 3), round(math.atan2(t[1], t[0]), 4))

    name = f"gen_{difficulty}_{size}_{seed}" + ("_" + "".join(m[0] for m in modes) if modes else "")
    layout = Layout(name, size_m, width, xy, [float(c[1]) for c in arcs], start,
                    info={"difficulty": difficulty, "size": size, "gen_seed": seed, "modes": list(modes),
                          "corners": n, "lap_m": round(len(xy) * ds, 1)})
    add_obstacles(layout, rng, modes, d, cfg)
    return layout


def add_obstacles(layout, rng, modes, d, cfg):
    """Pick box spots along the centreline: away from the start, off tight corners, spaced out."""
    g = cfg["trackgen"]
    ds = cfg["plan"]["ds_m"]
    xy, width = layout.centre, layout.width_m
    nrm = normals(xy)
    n = len(xy)
    kappa = np.abs(curvature(xy))
    win = int(0.5 / ds)
    corner_ok = np.array([kappa[np.arange(i - win, i + win + 1) % n].max() <= 1 / g["box_corner_radius_m"]
                          for i in range(n)])
    s = np.arange(n) * ds
    away = (s >= g["box_start_clear_m"]) & (s <= n * ds - g["box_start_clear_m"])
    taken = []

    def free(i):
        return all(min(abs(i - j), n - abs(i - j)) * ds >= g["box_spacing_m"] for j in taken)

    def gap():
        return float(min(rng.uniform(d["gap_m"], d["gap_m"] + 0.15), width - 0.1))

    def size():
        return [round(float(rng.uniform(lo, hi)), 2) for lo, hi in zip(g["box_m"], g["box_m_max"])]

    if "slalom" in modes and d["slalom"][1] > 0:
        spacing = float(rng.uniform(*g["slalom_spacing_m"]))
        step = int(round(spacing / ds))
        gentle = kappa < g["slalom_kappa"]
        for k in range(int(rng.integers(d["slalom"][0], d["slalom"][1] + 1)), 1, -1):   # fewer boxes if it must
            span = (k - 1) * step
            ok = [i for i in range(n)
                  if all(away[(i + j * step) % n] for j in range(k))
                  and gentle[np.arange(i - win, i + span + win + 1) % n].all()]
            if not ok:
                continue
            first = ok[int(rng.integers(len(ok)))]
            idx = [(first + j * step) % n for j in range(k)]
            side = 1 if rng.random() < 0.5 else -1
            boxes = [place_box(xy, nrm, i, side * (-1) ** j, width, gap(), size()) for j, i in enumerate(idx)]
            taken += idx
            if g["slalom_drop"]:
                layout.boxes_drop += [(b, g["drop_from_lap"]) for b in boxes]
            else:
                layout.slalom += boxes
            break

    spots = [i for i in rng.permutation(n) if away[i] and corner_ok[i]]
    wanted = []
    if "drop" in modes:
        wanted += ["drop"] * int(rng.integers(d["drop"][0], d["drop"][1] + 1))
    if "known" in modes:
        wanted += ["known"] * int(rng.integers(d["known"][0], d["known"][1] + 1))
    lap = g["drop_from_lap"]
    for kind in wanted:
        i = next((i for i in spots if free(i)), None)
        if i is None:
            continue
        taken.append(i)
        box = place_box(xy, nrm, i, 1 if rng.random() < 0.5 else -1, width, gap(), size())
        if kind == "known":
            layout.boxes_known.append(box)
        else:
            layout.boxes_drop.append((box, lap))
            lap += 1                                             # one drop per lap
    placed = {"known": layout.boxes_known, "drop": layout.boxes_drop, "slalom": layout.slalom or g["slalom_drop"]}
    layout.info["missing"] = [m for m in modes if not placed[m]]     # asked for, but not even one box fitted here
    layout.info["boxes"] = {"known": len(layout.boxes_known), "drop": len(layout.boxes_drop), "slalom": len(layout.slalom)}


def save(layout, tracks_dir=TRACKS_DIR):
    w_px, h_px = layout.px
    img = Image.fromarray(np.where(render(layout)[:, :, None], 0, 255).astype(np.uint8).repeat(3, axis=2))
    img.save(tracks_dir / f"{layout.name}.png")
    x, y, yaw = layout.start_pose
    meta = {"start_pose": {"x_m": x, "y_m": y, "yaw_rad": yaw}, "px_per_m": PX_PER_M,
            "boxes_known": layout.boxes_known,
            "boxes_drop": [{"box": b, "at_lap": lap} for b, lap in layout.boxes_drop],
            "slalom": layout.slalom,
            "gen": {**layout.info, "width_m": layout.width_m, "size_m": list(layout.size_m)}}
    with open(tracks_dir / f"{layout.name}.yaml", "w") as f:
        yaml.safe_dump(plain(meta), f, default_flow_style=None, sort_keys=False)


def plain(x):
    """numpy numbers -> Python numbers, all the way down, so yaml can write them."""
    if isinstance(x, dict):
        return {k: plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    return x.item() if isinstance(x, np.generic) else x


def preview(layout, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    w, h = layout.size_m
    fig, ax = plt.subplots(figsize=(w * 0.8 + 1, h * 0.8 + 0.6))
    draw_layout(ax, layout)
    ax.set_title(f"{layout.name}  ({layout.info['lap_m']} m, {layout.width_m} m wide)", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


BOX_COLOURS = {"known": "#d62728", "drop": "#ff7f0e", "slalom": "#9467bd"}


def draw_layout(ax, layout, legend=True):
    from matplotlib.patches import Rectangle

    w, h = layout.size_m
    ax.imshow(~render(layout), cmap="gray", extent=(0, w, 0, h), vmin=0, vmax=1)
    ax.plot(*np.vstack([layout.centre, layout.centre[:1]]).T, color="#1f77b4", lw=0.8)
    x, y, yaw = layout.start_pose
    ax.annotate("", (x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)), (x, y),
                arrowprops={"arrowstyle": "-|>", "color": "#2ca02c", "lw": 2})
    groups = [("known", layout.boxes_known), ("drop", [b for b, _ in layout.boxes_drop]), ("slalom", layout.slalom)]
    for kind, boxes in groups:
        for i, (bx, by, bw, bh) in enumerate(boxes):
            ax.add_patch(Rectangle((bx, by), bw, bh, color=BOX_COLOURS[kind], label=kind if i == 0 else None))
    if legend and any(b for _, b in groups):
        ax.legend(loc="upper right", fontsize=7)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])


def parse_modes(text):
    modes = [m for m in (text or "").split(",") if m]
    bad = [m for m in modes if m not in MODES]
    if bad:
        raise SystemExit(f"unknown obstacle mode(s) {bad}; use any of {', '.join(MODES)}")
    return tuple(m for m in MODES if m in modes)


def main():
    p = argparse.ArgumentParser(description="generate a random track")
    p.add_argument("--gen-seed", type=int, default=0)
    p.add_argument("--difficulty", default="medium", choices=["easy", "medium", "hard"])
    p.add_argument("--size", default="medium", choices=["small", "medium", "large"])
    p.add_argument("--size-m", type=float, nargs=2, metavar=("W", "H"), help="overrides --size, at most 10 x 10")
    p.add_argument("--obstacles", default="", help="comma list of known, drop, slalom")
    p.add_argument("--preview", action="store_true", help="also write docs/plots/gen_preview_<name>.png")
    args = p.parse_args()
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    layout, rejected = generate(args.gen_seed, args.difficulty, args.size, parse_modes(args.obstacles), cfg, args.size_m)
    save(layout)
    print(f"saved tracks/{layout.name}.png: {layout.info['lap_m']} m, {layout.width_m} m wide, "
          f"{len(layout.radii)} corners, boxes {layout.info['boxes']}, after {layout.info['attempts']} tries; "
          f"rejected: {rejected or 'none'}")
    if layout.info["missing"]:
        print(f"no room on this track for: {', '.join(layout.info['missing'])}")
    if args.preview:
        out = ROOT / "docs" / "plots" / f"gen_preview_{layout.name}.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        preview(layout, out)
        print(f"saved {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
