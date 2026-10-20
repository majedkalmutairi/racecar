"""Drive many random tracks headless and report how often the car gets round them.

Run from the repo root:  python tools/bench_random.py --n 30
Writes logs/bench/summary.csv (one row per run) and docs/plots/bench_success.png, bench_improvement.png,
bench_gallery.png. Every run gets its own log folder, so the runs go in parallel.
"""
import argparse
import csv
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from racer.sim.trackgen import MODES, draw_layout, generate, parse_modes  # noqa: E402

OUT = ROOT / "logs" / "bench"
PLOTS = ROOT / "docs" / "plots"
LEVELS = ["easy", "medium", "hard"]
RESULTS = ["OK", "STUCK", "CRASH", "TIMEOUT", "FALSE_CHANGE", "GEN_FAIL"]
COLOURS = {"OK": "#2ca02c", "STUCK": "#d62728", "CRASH": "#9467bd", "TIMEOUT": "#ff7f0e",
           "FALSE_CHANGE": "#8c564b", "GEN_FAIL": "#7f7f7f"}
FIELDS = ["name", "gen_seed", "difficulty", "size", "obstacles", "result", "explore_s", "first_race_s", "best_s",
          "laps", "crashes", "reflex", "recoveries", "track_changed", "lap_m", "width_m",
          "boxes_known", "boxes_drop", "boxes_slalom"]


def run_one(job):
    seed, level, args = job
    log_dir = OUT / "runs" / f"{level}_{args.size}_{seed}"
    cmd = [sys.executable, "-m", "racer.sim.main", "--headless", "--random", "--gen-seed", str(seed),
           "--difficulty", level, "--size", args.size, "--laps", str(args.laps), "--max-s", str(args.max_s),
           "--log-dir", str(log_dir)]
    if args.obstacles:
        cmd += ["--obstacles", args.obstacles]
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    row = {"gen_seed": seed, "difficulty": level, "size": args.size, "obstacles": args.obstacles or "none"}
    gen = re.search(r"\[gen\] (\S+):", p.stdout)
    if not gen:
        return {**row, "name": "-", "result": "GEN_FAIL"}
    row["name"] = gen.group(1)
    meta = yaml.safe_load(open(ROOT / "tracks" / f"{row['name']}.yaml"))["gen"]
    row.update(lap_m=meta["lap_m"], width_m=meta["width_m"], boxes_known=meta["boxes"]["known"],
               boxes_drop=meta["boxes"]["drop"], boxes_slalom=meta["boxes"]["slalom"])

    laps = list(csv.DictReader(open(log_dir / "laps.csv"))) if (log_dir / "laps.csv").exists() else []
    events = list(csv.DictReader(open(log_dir / "events.csv"))) if (log_dir / "events.csv").exists() else []
    race = [float(r["lap_time_s"]) for r in laps if r["mode"] == "RACE"]
    explore = [float(r["lap_time_s"]) for r in laps if r["mode"] == "EXPLORE"]
    kinds = [e["kind"] for e in events]
    changed = p.stdout.count("-> TRACK_CHANGED")
    crashes = kinds.count("crash")
    row.update(explore_s=explore[0] if explore else "", first_race_s=race[0] if race else "",
               best_s=min(race) if race else "", laps=len(laps), crashes=crashes, reflex=kinds.count("reflex"),
               recoveries=kinds.count("recover"), track_changed=changed)
    if p.returncode == 3:
        row["result"] = "STUCK"
    elif crashes:
        row["result"] = "CRASH"
    elif changed:
        row["result"] = "FALSE_CHANGE"           # nothing swapped the track, so any change detection is wrong
    elif len(laps) < args.laps:
        row["result"] = "TIMEOUT"
    else:
        row["result"] = "OK"
    return row


def plot_success(rows):
    fig, ax = plt.subplots(figsize=(6, 4))
    bottom = np.zeros(len(LEVELS))
    for result in RESULTS:
        frac = np.array([np.mean([r["result"] == result for r in rows if r["difficulty"] == lv] or [0])
                         for lv in LEVELS])
        if frac.any():
            ax.bar(LEVELS, frac * 100, bottom=bottom, color=COLOURS[result], label=result)
            bottom += frac * 100
    ax.set_ylabel("% of runs")
    n = len(rows) // max(len({r["difficulty"] for r in rows}), 1)
    ax.set_title(f"random tracks: {n} per difficulty")
    ax.legend(fontsize=8, loc="lower right")
    savefig(fig, "bench_success.png")


def plot_improvement(rows):
    fig, ax = plt.subplots(figsize=(5, 5))
    top = 0
    for lv, colour in zip(LEVELS, ["#2ca02c", "#1f77b4", "#d62728"]):
        pts = [(float(r["first_race_s"]), float(r["best_s"])) for r in rows
               if r["difficulty"] == lv and r["result"] == "OK"]
        if pts:
            x, y = np.array(pts).T
            ax.scatter(x, y, s=14, color=colour, label=lv, alpha=0.8)
            top = max(top, x.max())
    ax.plot([0, top * 1.05], [0, top * 1.05], "k--", lw=0.8, label="no improvement")
    ax.set_xlabel("first race lap [s]")
    ax.set_ylabel("best lap [s]")
    ax.set_title("every dot is one random track")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    savefig(fig, "bench_improvement.png")


def plot_gallery(args, cfg):
    modes = MODES if args.obstacles == "all" else parse_modes(args.obstacles)
    w, h = cfg["trackgen"]["sizes"][args.size]
    fig, axes = plt.subplots(3, 4, figsize=(14, 3 * (3.5 * h / w + 0.4)))      # each panel keeps the area's shape
    for i, lv in enumerate(LEVELS):
        for j in range(4):
            ax = axes[i, j]
            try:
                layout, _ = generate(j, lv, args.size, modes, cfg)
                draw_layout(ax, layout, legend=(i == 0 and j == 0))
                ax.set_title(f"{lv}, seed {j}: {layout.info['lap_m']} m", fontsize=9)
            except RuntimeError:
                ax.set_axis_off()
    savefig(fig, "bench_gallery.png")


def savefig(fig, name):
    PLOTS.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(PLOTS / name, dpi=120)
    plt.close(fig)
    print(f"saved docs/plots/{name}")


def summary(rows):
    for lv in LEVELS:
        mine = [r for r in rows if r["difficulty"] == lv]
        if not mine:
            continue
        ok = [r for r in mine if r["result"] == "OK"]
        faster = [r for r in ok if float(r["best_s"]) < float(r["first_race_s"])]
        gain = [1 - float(r["best_s"]) / float(r["first_race_s"]) for r in ok]
        bad = {res: sum(r["result"] == res for r in mine) for res in RESULTS[1:]}
        print(f"{lv:6s}: {len(ok)}/{len(mine)} OK ({100 * len(ok) / len(mine):.0f} %), "
              f"best < first race lap on {len(faster)}/{len(ok)}, mean gain {100 * np.mean(gain or [0]):.0f} %, "
              f"recoveries {sum(int(r.get('recoveries') or 0) for r in mine)}, "
              f"false track changes {sum(int(r.get('track_changed') or 0) for r in mine)}, "
              f"failures {({k: v for k, v in bad.items() if v}) or 'none'}")


def main():
    p = argparse.ArgumentParser(description="benchmark the car on random tracks")
    p.add_argument("--n", type=int, default=30, help="tracks per difficulty")
    p.add_argument("--first-seed", type=int, default=0)
    p.add_argument("--difficulty", default="easy,medium,hard")
    p.add_argument("--size", default="medium", choices=["small", "medium", "large", "xl"])
    p.add_argument("--obstacles", default="all", help="comma list of known, drop, slalom; 'all'; or '' for none")
    p.add_argument("--laps", type=int, default=10)
    p.add_argument("--max-s", type=float, default=400)
    p.add_argument("--jobs", type=int, default=max((os.cpu_count() or 2) - 1, 1))
    p.add_argument("--no-gallery", action="store_true")
    args = p.parse_args()
    cfg = yaml.safe_load(open(ROOT / "racer" / "config.yaml"))

    levels = [lv for lv in args.difficulty.split(",") if lv]
    jobs = [(seed, lv, args) for lv in levels for seed in range(args.first_seed, args.first_seed + args.n)]
    print(f"{len(jobs)} runs on {args.jobs} workers", flush=True)
    rows = []
    with ThreadPoolExecutor(args.jobs) as pool:
        for row in pool.map(run_one, jobs):
            rows.append(row)
            if row["result"] != "OK":
                print(f"   {row['difficulty']} seed {row['gen_seed']}: {row['result']}", flush=True)

    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows({k: r.get(k, "") for k in FIELDS} for r in rows)
    print(f"wrote {OUT.relative_to(ROOT) / 'summary.csv'}")
    summary(rows)
    plot_success(rows)
    plot_improvement(rows)
    if not args.no_gallery:
        plot_gallery(args, cfg)


if __name__ == "__main__":
    main()
