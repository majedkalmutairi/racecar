"""Run the simulator headless many times and keep every run's logs under logs/exp/.

Run from the repo root:  python tools/run_experiments.py
Three seeds per track for 15 laps, plus one track-swap run. Takes a few minutes.
"""
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "logs"
EXP = LOGS / "exp"
TRACKS = ["oval", "hairpin", "chicane"]
SEEDS = [0, 1, 2]
LAPS = 15


def run(args, laps_to, mult_from=None, mult_to=None):
    for f in LOGS.glob("*_*.np*"):
        f.unlink()
    cmd = [sys.executable, "-m", "racer.sim.main", "--headless", "--fresh", "--max-s", "600"] + args
    print("running:", " ".join(args), flush=True)
    result = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    print("   ", result.stdout.strip().splitlines()[-1] if result.stdout.strip() else result.stderr[-300:])
    shutil.copy(LOGS / "laps.csv", laps_to)
    shutil.copy(LOGS / "events.csv", laps_to.with_name(laps_to.stem + "_events.csv"))
    if mult_from and (LOGS / mult_from).exists():
        shutil.copy(LOGS / mult_from, mult_to)


def main():
    EXP.mkdir(parents=True, exist_ok=True)
    for track in TRACKS:
        for seed in SEEDS:
            run(["--track", track, "--laps", str(LAPS), "--seed", str(seed)],
                EXP / f"{track}_seed{seed}.csv", f"{track}_mult.npy", EXP / f"{track}_seed{seed}_mult.npy")
    run(["--track", "oval", "--swap-track", "chicane", "--swap-at-lap", "8", "--laps", "16", "--seed", "0"],
        EXP / "swap_oval_to_chicane.csv")
    print("done:", len(list(EXP.glob("*.csv"))), "csv files in logs/exp/")


if __name__ == "__main__":
    main()
