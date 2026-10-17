"""Plot lap time vs lap number, one line per track: logs/laps.csv -> docs/plots/explore_laps.png

Run from the repo root:  python tools/plot_laps.py
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")                 # no window; writes a file
import matplotlib.pyplot as plt

CSV_PATH = "logs/laps.csv"
OUT_PATH = "docs/plots/explore_laps.png"


def load(path):
    """-> {track: ([lap numbers], [lap times])}, as numbers, not strings."""
    data = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            laps, times = data.setdefault(row["track"], ([], []))
            laps.append(int(row["lap_number"]))
            times.append(float(row["lap_time_s"]))
    return data


def main():
    data = load(CSV_PATH)
    plt.figure(figsize=(7, 4))
    for track, (laps, times) in data.items():
        plt.plot(laps, times, marker="o", label=track)
    plt.xlabel("lap")
    plt.ylabel("lap time (s)")
    plt.title("Explore laps")
    plt.ylim(0, 1.15 * max(max(t) for _, t in data.values()))
    plt.xticks(range(1, max(max(l) for l, _ in data.values()) + 1))
    plt.legend()
    plt.grid(True, alpha=0.3)
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    plt.savefig(OUT_PATH, dpi=150, bbox_inches="tight")
    print(f"saved {OUT_PATH}")


if __name__ == "__main__":
    main()
