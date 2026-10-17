"""Draw the recorded map, explore path, centerline, racing line and speed profile of one track.

Run from the repo root after a run:  python tools/plot_plan.py oval
Writes docs/plots/plan_<track>.png
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from racer.mapping.grid import OccupancyGrid

ROOT = Path(__file__).resolve().parent.parent


def main(track):
    cfg = yaml.safe_load(open(ROOT / "racer/config.yaml"))
    m = cfg["mapping"]
    grid = OccupancyGrid.load(ROOT / f"logs/{track}_map.npy", m["size_m"], m["res_m"])
    center = np.load(ROOT / f"logs/{track}_center.npz")
    line = np.load(ROOT / f"logs/{track}_line.npy")
    v = np.load(ROOT / f"logs/{track}_speed.npy")

    rows, cols = np.nonzero(grid.occupied(m["min_hits"]))
    wx, wy = grid.cell_to_world(rows, cols)

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(9, 10), gridspec_kw={"height_ratios": [3, 1]})
    ax.scatter(wx, wy, s=2, c="black", label="map")
    ax.plot(center["path"][:, 0], center["path"][:, 1], c="grey", lw=0.8, label="explore path")
    ax.plot(*np.vstack([center["xy"], center["xy"][:1]]).T, c="tab:blue", lw=1.5, label="centerline")
    sc = ax.scatter(line[:, 0], line[:, 1], c=v, cmap="RdYlGn", s=12, label="racing line (colour = speed)")
    ax.plot(center["xy"][0, 0], center["xy"][0, 1], "k*", ms=12, label="start")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{track}: map, centerline and racing line")
    ax.legend(loc="upper right", fontsize=8)
    fig.colorbar(sc, ax=ax, label="target speed [m/s]")

    s = np.arange(len(v)) * cfg["plan"]["ds_m"]
    ax2.plot(s, v)
    ax2.set_xlabel("distance along racing line [m]")
    ax2.set_ylabel("target speed [m/s]")
    ax2.set_title("speed profile")
    ax2.grid(True, alpha=0.3)

    out = ROOT / "docs/plots" / f"plan_{track}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"saved {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "oval")
