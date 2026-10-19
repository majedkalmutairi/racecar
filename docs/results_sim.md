# Simulation results

All numbers come from `logs/laps.csv` and `logs/events.csv` written by `python tools/run_experiments.py`
(three seeds per track, 15 laps each, plus one track-swap run) and summarised by `python tools/plot_learning.py`.
Seeds only change the 1 cm lidar noise. No run crashed.

## Learning (15 laps, 3 seeds)

Lap 1 is the explore lap (26 to 29 s on every track). Lap 2 is the first race lap at 60 % of the planned speed
profile. From there every clean section gets 8 % faster per lap until it hits the 1.1 cap or a problem shows up.

| track   | first race lap [s] | best lap [s] | laps to within 5 % of best | mean multiplier at lap 15 | lap 15 vs first race lap |
|---------|-------------------:|-------------:|---------------------------:|--------------------------:|-------------------------:|
| oval    | 11.23 | 6.17 | 9 | 1.10 | 45 % faster |
| hairpin | 12.16 | 6.69 | 9 | 1.09 | 45 % faster |
| chicane | 12.68 | 6.92 | 9 | 1.10 | 45 % faster |

The three seeds land within 0.1 s of each other on every lap, so the min-max band on the plots is barely visible.
The ideal lap time from the speed profile alone (`sum(ds / v)`) is 6.3 s on the oval, 6.8 s on the hairpin and
7.3 s on the chicane; the car ends up slightly faster than that because the multiplier cap is 1.1.

![oval](plots/laptime_vs_lap_oval.png) ![hairpin](plots/laptime_vs_lap_hairpin.png) ![chicane](plots/laptime_vs_lap_chicane.png)

![oval multipliers](plots/multipliers_oval.png) ![hairpin multipliers](plots/multipliers_hairpin.png) ![chicane multipliers](plots/multipliers_chicane.png)

## Track change (oval to chicane at lap 8)

The chicane PNG replaces the oval under the car while it is racing. Nothing is told to the car.

- last oval race lap 7.1 s, swap, map mismatch jumps from 0.00 to about 0.9
- the car slows to explore speed while the mismatch is above 0.35 and resets 2.0 s after the swap
  (`RACE -> TRACK_CHANGED -> EXPLORE`), deleting map, plan and multipliers
- explore lap on the chicane 27.9 s, first chicane race lap 12.6 s, best chicane lap after 7 race laps 8.0 s

![track change](plots/track_change.png)

## Obstacles

- **Known boxes** (`--obstacles`, one 0.3 m box per track from `config.yaml`, there before the explore lap): the box
  ends up in the occupancy grid like any wall and the centerline and racing line route around it. All five tracks
  finish 6 laps with no crash. No special code: mapping and planning never knew the difference between a wall and a
  box. (Until 2026-10-19 the oval's box sat inside the outer wall, so the oval run tested nothing, and the hairpin,
  chicane and twisty boxes left gaps of 0.62 to 0.75 m in which the explore lap parked against the box for good.
  Every box now leaves at least 0.9 m and is at least 2.5 m from the start.)
- **Dropped box** (`--drop-box 6.33 0.65 0.3 0.3 --drop-at-lap 3`, a box on the racing line after planning): the
  safety reflex fires once (`min_ttc` 0.55 s) and slows the car; the new wall cells are mapped while racing, which
  triggers a replan, and the racing line now passes the box on the inside. Lap time before the drop 10.23 s, the lap
  with the drop 9.65 s (the reflex cost about 0.4 s against the expected 9.3 s), then 9.25, 8.63, 8.03 and 7.47 s.
  Zero crashes over 8 laps.
- The track-change detector reported a mismatch of 0.05 to 0.13 while the box was in view, far below the 0.35
  threshold: one box is an obstacle to route around, not a new track.

## Safety events

8 reflex episodes over all 10 runs, none in the first race lap. They happen in the two tight sections of the
hairpin and chicane once the multiplier reaches 1.1, cost 0.1 to 0.2 s each, and mark their section as a problem so
the multiplier backs off 15 % before climbing again.

## Reverse recovery

The car can now reverse (at most 0.5 m/s). When it wants to drive but has not moved for 1 s, it backs up 0.5 m
(0.75 m, 1.0 m on later tries at the same spot) with the wheels turned away from the side the controller is aiming
for, so the nose swings toward the way out. The back-up stops early if the body would touch a wall already in the
map or the outermost beams see something within 0.2 m. After three tries at one spot, or 30 s without getting 1 m
anywhere, it gives up: headless runs end with `STUCK` and exit code 3 instead of running until `--max-s`.

The old, too-tight config boxes make good test cases, dropped at lap 0 (`--drop-box X Y 0.3 0.3 --drop-at-lap 0`):

| box | gap | before | with recovery |
|---|---|---|---|
| chicane (4.0, 1.3) | 0.64 m | parked for good at 26 s of the explore lap | 6 laps, 6 back-ups, no crash; laps stay near 18 s because every lap needs a back-up |
| twisty (6.0, 0.55) | 0.75 m | parked for good | 6 laps, 5 back-ups, no crash |
| hairpin (5.0, 1.25) | 0.62 m | parked for good | `STUCK` after three tries at the same spot, at 107 s |

Recovery gets the car out; it does not make a gap the planner cannot use drivable. That is the generator's job: it
never puts a box closer than 0.85 m to the far wall.

## Random tracks

`python tools/bench_random.py --n 30` generates 30 tracks per difficulty (10 x 7 m area, known, dropped and slalom
boxes) and drives each for 10 laps headless, in parallel, each run in its own log folder.

| difficulty | runs OK | lap | explore lap | first race lap (mean) | best lap (mean) | best vs first | back-ups |
|---|---|---|---|---|---|---|---|
| easy   | 30 / 30 | 12.6 - 18.0 m | 16 - 26 s | 9.9 s  | 5.8 s | 41 % faster | 5 |
| medium | 30 / 30 | 12.2 - 18.2 m | 18 - 35 s | 10.5 s | 6.4 s | 39 % faster | 9 |
| hard   | 30 / 30 | 16.8 - 22.1 m | 28 - 42 s | 12.7 s | 8.0 s | 36 % faster | 5 |

No crash, no false track change, and the best lap beats the first race lap on all 90. On 300 tracks of the large
size (10 x 10 m, seeds 100 to 199): 300 of 300 OK. The same 300 with recovery switched off
(`recovery.max_attempts: 0`): easy 97, medium 79, hard 75 OK; the other 49 park against a box or a wall.

![success](plots/bench_success.png) ![improvement](plots/bench_improvement.png)

![gallery](plots/bench_gallery.png)

What the generator found out on the way:

- A corner drawn as 12 points reads as a much tighter corner than it is once resampled, so generated arcs get a
  point every 2 cm.
- Star-shaped polygons with corners pulled far in make narrow V-shapes; the grid-cell outline cannot, because a blob
  with no holes and no corner-only contacts never comes within one cell of itself.
- A box makes the planner's racing line kink sideways over a few centimetres. Pure pursuit never follows that
  literally (it aims at least 0.5 m ahead), so the line is judged on points 0.5 m apart.
- Asking for every kind of box on every track pushed the hard tracks toward plain rectangles (only they had room);
  boxes are now placed where they fit, and a slalom only appears where there is a long enough straight
  (most easy tracks, a few medium and hard ones).
