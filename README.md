# racecar

A 1/10 scale RC car that learns to race by itself.

Put it on a track it has never seen. It drives one slow lap to map the track, then goes faster every lap after that.
If I move the walls around, it notices and starts over from the slow lap.

The only sensor is a 2D lidar, and all the compute is on the car (Jetson Orin Nano).

The plan is to write a small 2D simulator first, get everything working there, and then run the same code on the
real car. The simulator part is done (below). I'm ordering the parts for the car now; see
[shopping-list.md](shopping-list.md).

![dashboard](docs/sim_dashboard.gif)

## How it drives

Every frame the mode manager reads the lidar, the pose and the speed through one small interface
(`CarInterface`: `get_scan`, `get_pose`, `get_speed`, `set_drive`) and decides what to do.

1. EXPLORE: drive at 0.8 m/s, stay centred between the walls, and count every lidar hit into an occupancy grid.
2. PLAN, once the car is back at its start: slide the explore path onto the ridge of the distance transform to get
   the centerline. Then solve for the racing line that bends least while keeping 0.35 m from every wall and box
   (one bounded least-squares problem), and build a speed profile from grip with `v = sqrt(grip * R)`. Braking and
   accelerating only get the grip a corner leaves over (friction ellipse).
3. RACE: pure pursuit on the racing line. In bends the lookahead shrinks so the car cuts a corner by at most 10 cm,
   and it asks for the planned speed a moment early because the speed controller lags.
4. Learning, after every race lap. The line is cut into 60 sections, each with a speed multiplier (the grip that
   section trusts, rebuilt into the speed profile) and a confidence, which is its clean laps in a row. Multipliers
   start at 0.6. A clean lap raises a section by 5 % times its confidence, up to 15 %. If the car slid there, left
   the line by more than 0.12 m or needed the reflex, the section drops 10 % (25 % after a crash), its confidence
   resets, and that speed becomes a ceiling it stays just under, trying 6 % past it after two clean laps. The car
   spots a slide when it turns less than its steering asked for; on the real car the IMU can measure the same thing.
5. Safety reflex, in every mode, after the controller. While the car follows its plan, the reflex runs pure pursuit
   forward on the bicycle model to see where the car will really go. If a lidar hit lies on that path less than
   0.6 s away, it slows down to keep it 0.6 s away. Off the plan it predicts the shape the body sweeps along the
   current steering arc instead, and if that path is blocked it heads for the widest gap. This is the part that
   becomes a C++ node on the real car.
6. TRACK_CHANGED: while racing, if more than 35 % of the beams land where the map has no wall for 2 s, the car throws
   away the map, the plan and the multipliers and explores again from wherever it is.
7. Reverse recovery, in every mode: if the car wants to drive but has not moved for 1 s (usually because the reflex
   parked it nose-in against something), it backs up 0.5 m with the wheels turned away from the way out and tries
   again. The lidar cannot see straight behind, so each back-up is checked against the map built so far. After
   three tries at the same spot it stops and reports STUCK.

A box that is on the track during the explore lap is just more wall: it ends up in the map and the plan goes around
it. A box dropped after planning is caught by the reflex, mapped while racing, and the plan is rebuilt around it a few
frames later.

Nothing in `racer/mapping`, `planning`, `control`, `learning`, `safety` or `modes` imports pygame, so the same code will
run on the car with a `RosCar` behind the interface.

## Results (simulation)

15 laps, three lidar-noise seeds per track, and lap 1 is the explore lap. No run crashed. The sim car has a grip limit
of about 0.8 g, so the speed it learns is the speed its tyres allow.

| track   | explore lap | first race lap | lap 15 |
|---------|------------:|---------------:|-------:|
| oval    | 26 s | 8.0 s | 4.2 s |
| hairpin | 29 s | 8.5 s | 4.5 s |
| chicane | 28 s | 9.7 s | 4.8 s |

![oval learning curve](docs/plots/laptime_vs_lap_oval.png)

Every track ends within 0 to 1.4 % of the fastest lap its racing line allows at that grip. One long run on a random
hard track went 473 laps without a crash: 12.4 s on the first race lap, 6.6 s by lap 8, then between 6.2 and 6.4 s
for the remaining 460 or so laps.

When the oval is swapped for the chicane under the car at lap 8, it notices the mismatch within 2 s, explores again
and learns the new track.

![track change](docs/plots/track_change.png)

More numbers, the multiplier plots and the obstacle runs are in [docs/results_sim.md](docs/results_sim.md).

## Reading the dashboard

The window shows the track on the left and a panel on the right. The panel reads the car's state and never changes
how it drives.

- Track: the racing line is coloured by planned speed, blue for slow and red for fast. Press C to colour it by the
  grip each section trusts, or by its confidence. The thin blue line is the centerline, the white line is where the
  car actually drove over the last lap, the cyan line ahead of the car is the path the reflex is checking, and the
  yellow dot is where pure pursuit is aiming. Red dots are mapped walls, bright while exploring. The car turns orange
  while its tyres slide and gets a red ring while the reflex brakes.
- Lap times: this lap, the last one, the best one, and "what it thinks it can do now", which is the lap time its
  current plan and learned grip add up to. The bars show every race lap; the best is green and the last is white.
- Speed: actual speed against what the controller asks for, on a bar from 0 to top speed, with a short history.
- What it has learned: one bar per section of the line. Height is the grip that section trusts in m/s², colour is how
  sure it is (clean laps in a row), and a white tick marks where it last had trouble. Small boxes are straights,
  where speed never tests the grip. "Trouble at ... g" is the median grip at which corners last slid, left the line
  or made the reflex brake.
- Safety: whether the reflex is braking, how long until something on the checked path would be hit, and counts of
  reflex events, back-ups and crashes.
- Debug, toggled with D: tracking error, lookahead distance, whether the reflex is checking the planned path or the
  steering arc, map mismatch (what the track-change detector sees), the current section's multiplier and ceiling,
  replans, dropped boxes, track swaps, and how much faster than real time the sim is running.

## Random tracks

`--random` builds a new track from a seed and drives it. Easy tracks have a few big sweeping corners, and medium ones
are tighter and narrower. Hard ones follow the outline of a random blob of grid cells, which gives L, U and S shapes
with 1.0 to 1.2 m corners on a 1.2 m wide track. Boxes come three ways: placed before the explore lap, dropped
mid-race, or set out as a slalom.

Every candidate track is checked before it is used: corner radius, gaps between passes, room next to every box, what
a dropped box does to the change detector, and a run of the real planner on the true walls. Between a third and nine
tenths of candidates pass, and the generator just tries the next one.

![random tracks](docs/plots/bench_gallery.png)

90 random tracks (30 per difficulty, 10 laps each, all three kinds of boxes) all finish without a crash, and on
every one the best lap is 45 to 47 % faster than the first race lap. Of 300 larger tracks, 299 finish and none
crash. With reverse recovery switched off, 53 of those 300 runs end with the car parked against a box or a wall,
including 30 % of the hard ones.

![improvement](docs/plots/bench_improvement.png)

## Tracks

Five hand-made tracks live in `tools/make_tracks.py`, all 1.2 to 1.4 m wide: `oval`, `hairpin` (one 1 m radius turn),
`chicane` (two S-bends), `twisty` (a deep pocket and a diagonal, nine corners) and `snake` (four columns joined by
three hairpins). Corners are rounded to a 1 m radius, about the tightest this car turns cleanly. On `twisty` the car
goes from a 45 s explore lap to 6.45 s, and on `snake` from 50 s to 7.0 s.

## Running it

```
python -m racer.sim.main --track oval                 # window with the dashboard; keys are listed at its bottom
python -m racer.sim.main --track hairpin --headless --laps 15      # no window, as fast as the CPU goes
python -m racer.sim.main --track oval --swap-track chicane --swap-at-lap 4 --laps 10
python -m racer.sim.main --track oval --obstacles --laps 6         # boxes from config.yaml are there from the start
python -m racer.sim.main --track oval --drop-box 6.0 1.3 0.6 0.3 --drop-at-lap 3 --laps 8   # repeat --drop-box for more
python -m racer.sim.main --random --gen-seed 7 --difficulty hard --obstacles known,drop,slalom
python -m racer.sim.trackgen --gen-seed 7 --difficulty hard --preview   # just make the track and a picture of it
python tools/bench_random.py --n 30                   # 90 random tracks headless, about 30 s on 24 cores
python -m pytest tests                                # 63 unit tests
python tools/plot_plan.py oval                        # map, centerline, racing line, speed profile of the last run
python tools/run_experiments.py && python tools/plot_learning.py   # the plots above, a few minutes
python tools/make_tracks.py                           # regenerate the five tracks
```

Every number lives in [racer/config.yaml](racer/config.yaml). A run writes `logs/laps.csv`, `logs/events.csv`
(reflex, recover, stuck, crash, drop and swap events) and the map, centerline, racing line, speed profile and
multipliers as `logs/<track>_*.npy`; `--log-dir` puts them somewhere else. A headless run that gets stuck exits with
code 3.

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
