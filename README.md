# racecar

A 1/10 scale RC car that learns to race by itself.

Put it on a track it has never seen. It drives one slow lap to map the track, then goes faster every lap after that. If I move the walls around it notices and starts over from the slow lap.

Only sensor is a 2D lidar. All the compute is on the car (Jetson Orin Nano).

Plan:

- write a small 2D simulator first and get everything working there
- then run the same code on the real car

Status: the simulator part is done (below). Ordering parts for the car, see [shopping-list.md](shopping-list.md).

![dashboard](docs/sim_dashboard.gif)

## How it drives

Every frame the mode manager reads the lidar, the pose and the speed through one small interface
(`CarInterface`: `get_scan`, `get_pose`, `get_speed`, `set_drive`) and decides what to do:

1. **EXPLORE**: 0.8 m/s, stay centred between the walls, count every lidar hit into an occupancy grid.
2. **PLAN** (once, when the car is back at its start): slide the explore path onto the ridge of the distance
   transform to get the centerline, smooth it into a racing line that keeps 0.3 m from the walls, and compute a
   speed profile from curvature (`v = sqrt(a_lat_max / |kappa|)`) with braking and acceleration limits.
3. **RACE**: pure pursuit on the racing line. The line is cut into 20 sections and each section has a speed
   multiplier that starts at 0.6 and a confidence: its clean laps in a row. A clean lap pushes the section up by
   5 % times its confidence (up to 15 % a lap); a problem (the car slid, left the line by more than 0.12 m, the
   reflex had to brake, or it crashed) backs it off 10 % (25 % for a crash), resets the confidence and remembers
   that speed as a ceiling to stay just under, and to try again past after three clean laps. Slides are spotted by
   comparing the turn rate the pose shows with what the steering asked for, which works with an IMU too.
4. **Safety reflex**, in every mode, after the controller. On the plan: run pure pursuit forward on the bicycle
   model to see where the car will really go, and if a lidar hit lies on that path within 0.6 s, slow down to keep
   it 0.6 s away. Off the plan: predict the shape the body sweeps along the current steering arc and do the same,
   and if that path is blocked head for the widest gap instead. This is the part that becomes a C++ node on the
   real car.
5. **TRACK_CHANGED**: while racing, if more than 35 % of the beams land where the map has no wall for 2 s, throw
   away the map, the plan and the multipliers and explore again from wherever the car is.
6. **Reverse recovery**, in every mode: if the car wants to drive but has not moved for 1 s (the reflex has parked
   it nose-in against something), back up 0.5 m with the wheels turned away from the way out, then try again.
   The lidar cannot see straight behind, so the back-up is checked against the map built so far. Three tries at
   the same spot, then it stops and reports STUCK instead of sitting there forever.

A box that is on the track during the explore lap is just more wall: it ends up in the map and the plan goes
around it. A box dropped after planning is caught by the reflex, mapped while racing, and the plan is rebuilt
around it a few frames later.

Nothing in `racer/mapping`, `planning`, `control`, `learning`, `safety` or `modes` imports pygame, so the same
code will run on the car with a `RosCar` behind the interface.

## Results (simulation)

15 laps, three lidar-noise seeds per track. Lap 1 is the explore lap. No run crashed. The sim car has a grip limit
(about 0.8 g), so the speed it learns is the speed the tyres allow.

| track   | explore lap | first race lap | lap 15 |
|---------|------------:|---------------:|-------:|
| oval    | 26 s | 8.2 s | 4.4 s |
| hairpin | 29 s | 9.2 s | 5.4 s |
| chicane | 28 s | 9.2 s | 5.4 s |

![oval learning curve](docs/plots/laptime_vs_lap_oval.png)

Swapping the oval for the chicane under the car at lap 8: the mismatch is noticed in 2 s, the car re-explores and
learns the new track.

![track change](docs/plots/track_change.png)

More numbers, the multiplier plots and the obstacle runs: [docs/results_sim.md](docs/results_sim.md).

## Random tracks

`--random` builds a new track from a seed and drives it. Easy tracks are a few big sweeping corners, medium ones
are tighter and narrower, hard ones are the outline of a random blob of grid cells (L, U and S shapes, 1.0 to
1.2 m corners, 1.2 m wide). Boxes come three ways: known before the explore lap, dropped mid-race, or a slalom.
Every candidate is checked before it is used (corner radius, gaps between passes, room next to every box, what
a dropped box does to the change detector, and a run of the real planner on the true walls); about a third to
nine tenths of candidates pass, and the generator just tries the next one.

![random tracks](docs/plots/bench_gallery.png)

90 random tracks (30 per difficulty, 10 laps each, all three kinds of boxes) all finish without a crash, and on
every one the best lap beats the first race lap by 45 to 47 %. 300 larger tracks: also 300 of 300. With reverse
recovery switched off, 57 of those 300 runs end with the car parked against a box or a wall (27 % of the hard ones).

![improvement](docs/plots/bench_improvement.png)

## Tracks

Five hand-made tracks in `tools/make_tracks.py`, all 1.2 to 1.4 m wide: `oval`, `hairpin` (one 1 m radius turn),
`chicane` (two S-bends), `twisty` (a deep pocket and a diagonal, nine corners) and `snake` (four columns joined by
three hairpins). Corners are rounded to a 1 m radius, which is about the tightest this car turns cleanly. On `twisty` the car goes from a 45 s explore lap to 7.3 s,
on `snake` from 50 s to 8.1 s.

## Running it

```
python -m racer.sim.main --track oval                 # window with the dashboard; R reset, SPACE pause, N next track, A manual
python -m racer.sim.main --track hairpin --headless --laps 15      # no window, as fast as the CPU goes
python -m racer.sim.main --track oval --swap-track chicane --swap-at-lap 4 --laps 10
python -m racer.sim.main --track oval --obstacles --laps 6         # boxes from config.yaml are there from the start
python -m racer.sim.main --track oval --drop-box 6.0 1.3 0.6 0.3 --drop-at-lap 3 --laps 8   # repeat --drop-box for more
python -m racer.sim.main --random --gen-seed 7 --difficulty hard --obstacles known,drop,slalom
python -m racer.sim.trackgen --gen-seed 7 --difficulty hard --preview   # just make the track and a picture of it
python tools/bench_random.py --n 30                   # 90 random tracks headless, about 30 s on 24 cores
python -m pytest tests                                # 61 unit tests
python tools/plot_plan.py oval                        # map, centerline, racing line, speed profile of the last run
python tools/run_experiments.py && python tools/plot_learning.py   # the plots above, a few minutes
python tools/make_tracks.py                           # regenerate the five tracks
```

Every number lives in [racer/config.yaml](racer/config.yaml). Runs write `logs/laps.csv`, `logs/events.csv` (reflex,
recover, stuck, crash, drop and swap events) and the map, centerline, racing line, speed profile and multipliers as
`logs/<track>_*.npy`; `--log-dir` puts them somewhere else. A headless run that gets stuck exits with code 3.

## Layout

```
racer/
  interface.py   CarInterface and the lidar beam angles
  sim/           track from PNG, bicycle model, lidar, SimCar, main loop, overlays, gif,
                 random track generator (trackgen) and its checker (trackcheck)
  mapping/       occupancy grid, centerline, change detector
  planning/      racing line, speed profile
  control/       pure pursuit
  learning/      per-section multipliers
  safety/        the reflex, reverse recovery
  modes/         explore controller, mode manager
  dashboard/     the side panel
tests/           one file per module
tools/           hand-made tracks, experiments, random-track benchmark, plots
```
