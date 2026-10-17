"""Generate the test tracks: tracks/<name>.png (white = drivable, black = wall) + tracks/<name>.yaml.

Run from the repo root:  python tools/make_tracks.py
"""
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

W, H = 1000, 700
PX_PER_M = 100
TRACKS_DIR = Path(__file__).resolve().parent.parent / "tracks"


def to_px(x_m, y_m):
    return x_m * PX_PER_M, H - y_m * PX_PER_M


def save_track(name, draw_fn, start_pose):
    img = Image.new("RGB", (W, H), "black")
    draw_fn(ImageDraw.Draw(img))
    img.save(TRACKS_DIR / f"{name}.png")
    x, y, yaw = start_pose
    meta = {"start_pose": {"x_m": round(x, 3), "y_m": round(y, 3), "yaw_rad": round(yaw, 4)},
            "px_per_m": PX_PER_M}
    with open(TRACKS_DIR / f"{name}.yaml", "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print(f"saved tracks/{name}.png  start {meta['start_pose']}")


def corridor(draw, points_m, width_m):
    """Paint a white band of constant width along a closed centreline (list of (x, y) in metres)."""
    r = width_m / 2 * PX_PER_M
    pts = list(points_m) + [points_m[0]]
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.02))     # a disc every 2 cm
        for t in np.linspace(0, 1, n, endpoint=False):
            cx, cy = to_px(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill="white")


def arc(cx, cy, r, a0, a1, n=200):
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


def rounded(corners, r):
    """Closed polygon -> the same loop with every corner replaced by an arc of radius r (smaller if the sides are short)."""
    pts = []
    n = len(corners)
    for i in range(n):
        p = np.array(corners[i], float)
        u = np.array(corners[i - 1]) - p
        v = np.array(corners[(i + 1) % n]) - p
        len_u, len_v = np.linalg.norm(u), np.linalg.norm(v)
        u, v = u / len_u, v / len_v
        half = math.acos(np.clip(np.dot(u, v), -1, 1)) / 2
        t = min(r / math.tan(half), 0.45 * len_u, 0.45 * len_v)
        radius = t * math.tan(half)
        bisector = (u + v) / np.linalg.norm(u + v)
        centre = p + bisector * radius / math.sin(half)
        a0 = math.atan2(*(p + u * t - centre)[::-1])
        a1 = math.atan2(*(p + v * t - centre)[::-1])
        sweep = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        pts += [tuple(centre + radius * np.array([math.cos(a0 + f * sweep), math.sin(a0 + f * sweep)]))
                for f in np.linspace(0, 1, 12)]
    return pts


# oval: two ellipses, 1.4 m lane
def draw_oval(draw):
    draw.ellipse((30, 30, 970, 670), fill="white")
    draw.ellipse((170, 170, 830, 530), fill="black")


# hairpin: 2.4 m turn on the left, 1.0 m turn on the right, 1.2 m wide
C1, R1 = (3.3, 3.5), 2.4
C2, R2 = (8.2, 3.5), 1.0
ALPHA = math.acos((R1 - R2) / (C2[0] - C1[0]))   # where the straights touch the circles


def hairpin_centreline():
    return (arc(*C2, R2, -ALPHA, ALPHA)                      # tight turn (CCW)
            + arc(*C1, R1, ALPHA, 2 * math.pi - ALPHA))      # big turn; the straights join them


def hairpin_start():
    p1 = (C1[0] + R1 * math.cos(-ALPHA), C1[1] + R1 * math.sin(-ALPHA))
    p2 = (C2[0] + R2 * math.cos(-ALPHA), C2[1] + R2 * math.sin(-ALPHA))
    return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, math.atan2(p2[1] - p1[1], p2[0] - p1[0]))


# chicane: stadium whose top straight dips 0.7 m and comes back, 1.3 m wide
CL, CR, R_END, CY = 2.8, 7.2, 1.9, 3.5
DIP_M = 0.7


def chicane_centreline():
    top = CY + R_END
    pts = [(x, CY - R_END) for x in np.linspace(CL, CR, 200, endpoint=False)]          # bottom, left -> right
    pts += arc(CR, CY, R_END, -math.pi / 2, math.pi / 2)[:-1]                           # right end
    for u in np.linspace(0, 1, 200, endpoint=False):                                    # top, right -> left
        pts.append((CR - u * (CR - CL), top - DIP_M * (1 - math.cos(2 * math.pi * u)) / 2))
    pts += arc(CL, CY, R_END, math.pi / 2, 3 * math.pi / 2)[:-1]                        # left end
    return pts


# twisty: a loop with a deep pocket from the top and a diagonal on the left, nine corners, 1.2 m wide
TWISTY = [(2.5, 1.0), (9.0, 1.0), (9.0, 6.1), (6.5, 6.1), (6.5, 3.9), (4.3, 3.9), (4.3, 6.1), (1.0, 6.1), (1.0, 3.6)]

# snake: four columns joined by three hairpins, return straight along the top, 1.2 m wide
SNAKE = [(1.0, 6.1), (1.0, 1.0), (3.2, 1.0), (3.2, 4.5), (5.4, 4.5), (5.4, 1.0), (7.6, 1.0), (7.6, 6.1)]


if __name__ == "__main__":
    save_track("oval", draw_oval, (5.0, 1.0, 0.0))
    save_track("hairpin", lambda d: corridor(d, hairpin_centreline(), 1.2), hairpin_start())
    save_track("chicane", lambda d: corridor(d, chicane_centreline(), 1.3), ((CL + CR) / 2, CY - R_END, 0.0))
    save_track("twisty", lambda d: corridor(d, rounded(TWISTY, 1.0), 1.2), (5.5, 1.0, 0.0))
    save_track("snake", lambda d: corridor(d, rounded(SNAKE, 1.0), 1.2), (4.5, 6.1, math.pi))
