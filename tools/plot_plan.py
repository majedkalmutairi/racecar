"""Draw the recorded map, explore path, centerline, racing line and speed profile of one track.

Run from the repo root after a run:  python tools/plot_plan.py oval [log_dir]
log_dir defaults to logs/ (tools/run_experiments.py leaves one per run in logs/exp/<track>_seed0/).
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
from racer.learning.sections import SectionLearner  # noqa: E402
from racer.mapping.grid import OccupancyGrid  # noqa: E402
from racer.planning.speed_profile import build_speed_profile  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent


def main(track, log_dir):
    cfg = yaml.safe_load(open(ROOT / "racer/config.yaml"))
    m, plan = cfg["mapping"], cfg["plan"]
    grid = OccupancyGrid.load(log_dir / f"{track}_map.npy", m["size_m"], m["res_m"])
    center = np.load(log_dir / f"{track}_center.npz")
    line = np.load(log_dir / f"{track}_line.npy")

    # the speed it planned for its first race lap (cautious start grip) and the speed it has learned since
    start = build_speed_profile(line, cfg, plan["a_lat_max"] * cfg["learning"]["start_mult"] ** 2)
    mult = SectionLearner(len(line), cfg, log_dir / f"{track}_mult.npy").multiplier_per_point()
    learned = build_speed_profile(line, cfg, plan["a_lat_max"] * mult ** 2)

    rows, cols = np.nonzero(grid.occupied(m["min_hits"]))
    wx, wy = grid.cell_to_world(rows, cols)

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(10, 10), gridspec_kw={"height_ratios": [2.3, 1]})
    ax.scatter(wx, wy, s=1, c="black", label="map (lidar hits)")
    ax.plot(center["path"][:, 0], center["path"][:, 1], c="grey", lw=0.8, label="explore path")
    ax.plot(*np.vstack([center["xy"], center["xy"][:1]]).T, c="tab:blue", lw=1, label="centerline")
    sc = ax.scatter(line[:, 0], line[:, 1], c=learned, cmap="RdYlGn", s=6, label="racing line (colour = learned speed)")
    ax.plot(center["xy"][0, 0], center["xy"][0, 1], "k*", ms=12, label="start")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{track}: map, centerline and racing line")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3, fontsize=8)
    fig.colorbar(sc, ax=ax, label="learned speed [m/s]", shrink=0.8)

    s = np.arange(len(line)) * plan["ds_m"]
    ax2.plot(s, start, c="grey", label="first race lap plan")
    ax2.plot(s, learned, c="tab:red", label="after learning")
    ax2.set_xlabel("distance along racing line from the start [m]")
    ax2.set_ylabel("target speed [m/s]")
    ax2.set_title("speed profile")
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    out = ROOT / "docs/plots" / f"plan_{track}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"saved {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "oval", Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "logs")
