"""Generate the hand-made tracks: tracks/<name>.png (white = drivable, black = wall) + tracks/<name>.yaml.

The original five are drawn 2.5 times bigger than they used to be (about 25 x 17.5 m), with sharp corners kept
sharp, and two new 30 x 20 m tracks are added. Every track's width wanders smoothly along the lap. Each one is
checked with the same rules as the random tracks before it is saved.
Random tracks come from racer/sim/trackgen.py, which also holds the corridor and rounded-corner geometry.

Run from the repo root:  python tools/make_tracks.py          (--plot also draws docs/plots/tracks.png)
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from racer.loops import resample  # noqa: E402
from racer.planning.speed_profile import curvature  # noqa: E402
from racer.sim.trackcheck import check_layout  # noqa: E402
from racer.sim.trackgen import (Layout, arc, fillets, rounded, save, straight_runs,  # noqa: E402
                                width_profile)

CFG = yaml.safe_load(open(Path(__file__).resolve().parent.parent / "racer" / "config.yaml"))
DS = CFG["plan"]["ds_m"]
S = 2.5                               # how much bigger the original five are drawn now
OLD = (25.0, 17.5)                    # their area in metres
NEW = (30.0, 20.0)
PX = 50                               # 2 cm per pixel


def oval():
    cx, cy, a, b = 5.0 * S, 3.5 * S, 4.0 * S, 2.5 * S
    return [(cx + a * math.cos(t), cy + b * math.sin(t)) for t in np.linspace(0, 2 * math.pi, 600, endpoint=False)], []


def hairpin():
    """A big turn on the left and a tight 1 m turn on the right, joined by two straights."""
    c1, r1 = (3.3 * S, 3.5 * S), 2.4 * S
    c2, r2 = (8.2 * S, 3.5 * S), 1.0
    alpha = math.acos((r1 - r2) / (c2[0] - c1[0]))          # where the straights touch the circles
    return arc(*c2, r2, -alpha, alpha) + arc(*c1, r1, alpha, 2 * math.pi - alpha), [r2]


def chicane():
    """A stadium whose top straight dips and comes back."""
    left, right, r_end, cy, dip = 2.8 * S, 7.2 * S, 1.9 * S, 3.5 * S, 0.7 * S
    top = cy + r_end
    pts = [(x, cy - r_end) for x in np.linspace(left, right, 400, endpoint=False)]
    pts += arc(right, cy, r_end, -math.pi / 2, math.pi / 2)[:-1]
    for u in np.linspace(0, 1, 400, endpoint=False):
        pts.append((right - u * (right - left), top - dip * (1 - math.cos(2 * math.pi * u)) / 2))
    pts += arc(left, cy, r_end, math.pi / 2, 3 * math.pi / 2)[:-1]
    return pts, []


def polygon(corners, radii):
    pts = rounded(corners, radii, step_m=0.02, max_frac=0.5)
    return pts, [f[1] for f in fillets(corners, radii, 0.5)]


# twisty: a loop with a deep pocket from the top and a diagonal on the left, nine sharp corners
TWISTY = [(2.5, 1.0), (9.0, 1.0), (9.0, 6.1), (6.5, 6.1), (6.5, 3.9), (4.3, 3.9), (4.3, 6.1), (1.0, 6.1), (1.0, 3.6)]
# snake: four columns joined by three hairpins, return straight along the top
SNAKE = [(1.0, 6.1), (1.0, 1.0), (3.2, 1.0), (3.2, 4.5), (5.4, 4.5), (5.4, 1.0), (7.6, 1.0), (7.6, 6.1)]
# speedway: a 22 m straight, two fast ends and a V chicane in the top straight
SPEEDWAY = [(4, 3), (26, 3), (26, 17), (18, 17), (14, 13), (10, 17), (4, 17)]
SPEEDWAY_R = [3.0, 3.0, 2.0, 1.2, 1.2, 1.2, 3.0]
# circuit: a 24 m straight, three pockets (two U-turns and a hairpin pair) and sharp corners everywhere else
CIRCUIT = [(3, 3), (27, 3), (27, 9), (21, 9), (21, 12), (27, 12), (27, 17), (17, 17), (17, 12), (14, 12), (14, 17),
           (3, 17), (3, 11), (9, 11), (9, 7), (3, 7)]
CIRCUIT_R = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0]

TRACKS = {
    # name: (centreline and corner radii, area, width range, seed for the width profile)
    "oval": (oval, OLD, (1.4, 3.0), 1),
    "hairpin": (hairpin, OLD, (1.2, 2.5), 2),
    "chicane": (chicane, OLD, (1.3, 2.6), 3),
    "twisty": (lambda: polygon([(x * S, y * S) for x, y in TWISTY], 1.0), OLD, (1.2, 2.4), 4),
    "snake": (lambda: polygon([(x * S, y * S) for x, y in SNAKE], 1.0), OLD, (1.2, 2.4), 5),
    "speedway": (lambda: polygon(SPEEDWAY, SPEEDWAY_R), NEW, (1.2, 3.0), 6),
    "circuit": (lambda: polygon(CIRCUIT, CIRCUIT_R), NEW, (1.0, 2.4), 7),
}


def build(name):
    make, size, (w_lo, w_hi), seed = TRACKS[name]
    pts, radii = make()
    xy = resample(np.array(pts), DS)
    widths = width_profile(np.random.default_rng(seed), len(xy), DS, w_lo, w_hi, CFG["trackgen"]["width_every_m"])
    runs = straight_runs(xy, DS, CFG["trackgen"]["straight_kappa"])
    # start in the middle of the longest straight, or where it bends least if it never runs straight (oval)
    first = (runs[0][0] + runs[0][1] // 2) % len(xy) if runs else int(np.argmin(np.abs(curvature(xy))))
    xy, widths = np.roll(xy, -first, axis=0), np.roll(widths, -first)
    t = xy[1] - xy[-1]
    start = (round(float(xy[0, 0]), 3), round(float(xy[0, 1]), 3), round(math.atan2(t[1], t[0]), 4))
    return Layout(name, size, widths, xy, radii or [99.0], start, px_per_m=PX,
                  info={"difficulty": "medium", "lap_m": round(len(xy) * DS, 1), "hand_made": True,
                        "width_range_m": [round(float(widths.min()), 2), round(float(widths.max()), 2)]})


def plot(layouts):
    """All the tracks side by side: docs/plots/tracks.png"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from racer.sim.trackgen import draw_layout

    cols = 4
    rows = math.ceil(len(layouts) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(16, rows * 3.6))
    for ax, layout in zip(axes.flat, layouts):
        draw_layout(ax, layout, legend=False)
        lo, hi = layout.info["width_range_m"]
        ax.set_title(f"{layout.name}: {layout.info['lap_m']:.0f} m, {lo:.1f}-{hi:.1f} m wide", fontsize=10)
        ax.set_axis_off()
    for ax in list(axes.flat)[len(layouts):]:
        ax.set_axis_off()
    fig.tight_layout()
    out = Path(__file__).resolve().parent.parent / "docs" / "plots" / "tracks.png"
    fig.savefig(out, dpi=100)
    print(f"saved docs/plots/{out.name}")


if __name__ == "__main__":
    names = [a for a in sys.argv[1:] if not a.startswith("--")] or list(TRACKS)
    layouts = []
    for name in names:
        layout = build(name)
        problems = check_layout(layout, CFG)
        if problems:
            raise SystemExit(f"{name}: {problems}")
        save(layout)
        layouts.append(layout)
        print(f"saved tracks/{name}.png  {layout.info['lap_m']} m lap, {layout.size_m[0]:g} x {layout.size_m[1]:g} m, "
              f"width {layout.info['width_range_m'][0]}-{layout.info['width_range_m'][1]} m, start {layout.start_pose}")
    if "--plot" in sys.argv:
        plot(layouts)
