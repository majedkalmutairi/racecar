"""Run the simulator headless many times and keep every run's logs under logs/exp/.

Run from the repo root:  python tools/run_experiments.py
Three seeds per track for 15 laps, plus one track-swap run, in parallel. Each run keeps its own folder
(logs/exp/<track>_seed<n>/) with the map and plan, for tools/plot_plan.py. Takes a few minutes.
"""
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EXP = ROOT / "logs" / "exp"
TRACKS = ["oval", "hairpin", "chicane", "twisty", "snake", "speedway", "circuit"]
SEEDS = [0, 1, 2]
LAPS = 15


def run(job):
    name, args, mult = job
    log_dir = EXP / name
    shutil.rmtree(log_dir, ignore_errors=True)
    cmd = [sys.executable, "-m", "racer.sim.main", "--headless", "--fresh", "--max-s", "1000",
           "--log-dir", str(log_dir)] + args
    result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    print(f"{name}: {result.stdout.strip().splitlines()[-1] if result.stdout.strip() else result.stderr[-300:]}",
          flush=True)
    shutil.copy(log_dir / "laps.csv", EXP / f"{name}.csv")
    shutil.copy(log_dir / "events.csv", EXP / f"{name}_events.csv")
    if mult and (log_dir / mult).exists():
        shutil.copy(log_dir / mult, EXP / f"{name}_mult.npy")


def main():
    EXP.mkdir(parents=True, exist_ok=True)
    jobs = [(f"{track}_seed{seed}", ["--track", track, "--laps", str(LAPS), "--seed", str(seed)], f"{track}_mult.npy")
            for track in TRACKS for seed in SEEDS]
    jobs.append(("swap_oval_to_chicane", ["--track", "oval", "--swap-track", "chicane", "--swap-at-lap", "8",
                                          "--laps", "16", "--seed", "0"], None))
    with ThreadPoolExecutor(max((os.cpu_count() or 2) - 1, 1)) as pool:
        list(pool.map(run, jobs))
    print("done:", len(list(EXP.glob("*.csv"))), "csv files in logs/exp/")


if __name__ == "__main__":
    main()
