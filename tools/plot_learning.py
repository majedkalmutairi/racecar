"""Turn the CSVs from tools/run_experiments.py into the learning plots for the README.

Run from the repo root:  python tools/plot_learning.py
Writes docs/plots/laptime_vs_lap_<track>.png, multipliers_<track>.png, track_change.png
and prints the numbers for docs/results_sim.md.
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
EXP = ROOT / "logs/exp"
PLOTS = ROOT / "docs/plots"
TRACKS = ["oval", "hairpin", "chicane"]
SEEDS = [0, 1, 2]


def load_laps(path):
    """-> list of rows as dicts with numbers, in lap order."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["lap_number"] = int(r["lap_number"])
        r["lap_time_s"] = float(r["lap_time_s"])
        r["crashes"] = int(r["crashes"])
    return rows


def race_times(rows):
    return np.array([r["lap_time_s"] for r in rows if r["mode"] == "RACE"])


def savefig(name):
    PLOTS.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(PLOTS / name, dpi=150)
    plt.close()
    print(f"saved docs/plots/{name}")


def plot_laptime_vs_lap(track):
    runs = [race_times(load_laps(EXP / f"{track}_seed{s}.csv")) for s in SEEDS]
    n = min(len(r) for r in runs)
    times = np.vstack([r[:n] for r in runs])
    laps = np.arange(2, n + 2)
    plt.figure(figsize=(6, 4))
    plt.fill_between(laps, times.min(axis=0), times.max(axis=0), alpha=0.3, label="min-max over 3 seeds")
    plt.plot(laps, times.mean(axis=0), marker="o", label="mean")
    plt.xlabel("lap (lap 1 is the explore lap)")
    plt.ylabel("lap time [s]")
    plt.title(f"{track}: lap time while learning")
    plt.ylim(0, times.max() * 1.15)
    plt.grid(True, alpha=0.3)
    plt.legend()
    savefig(f"laptime_vs_lap_{track}.png")
    return times


def plot_multipliers(track):
    mults = np.vstack([np.load(EXP / f"{track}_seed{s}_mult.npy") for s in SEEDS])
    sections = np.arange(len(mults[0]))
    mean = mults.mean(axis=0)
    plt.figure(figsize=(6, 4))
    plt.bar(sections, mean, yerr=np.maximum([mean - mults.min(axis=0), mults.max(axis=0) - mean], 0), capsize=3)
    plt.axhline(1.0, color="grey", linestyle="--", label="speed profile as planned")
    plt.xlabel("section of the racing line")
    plt.ylabel("learned speed multiplier")
    plt.title(f"{track}: multipliers after 15 laps (3 seeds)")
    plt.legend()
    savefig(f"multipliers_{track}.png")
    return mults


def plot_track_change():
    rows = load_laps(EXP / "swap_oval_to_chicane.csv")
    laps = [r["lap_number"] for r in rows]
    times = [r["lap_time_s"] for r in rows]
    colours = ["tab:orange" if r["mode"] == "EXPLORE" else ("tab:blue" if r["track"] == "oval" else "tab:green")
               for r in rows]
    swap_lap = next(r["lap_number"] for r in rows if r["track"] != "oval")
    plt.figure(figsize=(6, 4))
    plt.bar(laps, times, color=colours)
    plt.axvline(swap_lap - 0.5, color="red", linestyle="--", label="track swapped")
    plt.scatter([], [], c="tab:orange", label="explore lap")
    plt.scatter([], [], c="tab:blue", label="oval race laps")
    plt.scatter([], [], c="tab:green", label="chicane race laps")
    plt.xlabel("lap")
    plt.ylabel("lap time [s]")
    plt.title("track swapped under the car: re-explore, re-learn")
    plt.legend(fontsize=8)
    savefig("track_change.png")
    return rows


def summary(track, times, mults):
    first, best = times[:, 0].mean(), times.min(axis=1).mean()
    within = [int(np.argmax(r <= 1.05 * r.min())) + 1 for r in times]
    return (f"| {track:8s} | {first:6.2f} | {best:6.2f} | {np.mean(within):4.1f} | {mults.mean():5.2f} | "
            f"{100 * (1 - times[:, -1].mean() / first):4.0f} % |")


def main():
    lines = ["| track | first race lap [s] | best lap [s] | laps to within 5 % of best | mean multiplier at lap 15 | lap 15 vs first race lap |",
             "|---|---|---|---|---|---|"]
    for track in TRACKS:
        times = plot_laptime_vs_lap(track)
        mults = plot_multipliers(track)
        lines.append(summary(track, times, mults))
    rows = plot_track_change()
    swap = next(i for i, r in enumerate(rows) if r["track"] != "oval")
    print()
    print("\n".join(lines))
    print(f"\nTrack change: last oval lap {rows[swap - 1]['lap_time_s']} s, explore lap {rows[swap]['lap_time_s']} s, "
          f"first chicane race lap {rows[swap + 1]['lap_time_s']} s, "
          f"best chicane lap {min(r['lap_time_s'] for r in rows[swap + 1:])} s.")
    n_events = n_first = 0
    for p in EXP.glob("*_events.csv"):
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                if r.get("kind", "reflex") != "reflex":     # crashes, drops, swaps, recoveries are logged too
                    continue
                n_events += 1
                n_first += r["mode"] == "RACE" and r["lap"] == "2"
    print(f"Safety events over all runs: {n_events} (of which {n_first} in the first race lap).")


if __name__ == "__main__":
    main()
