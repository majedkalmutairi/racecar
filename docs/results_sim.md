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
| hairpin | 12.21 | 6.72 | 9 | 1.10 | 44 % faster |
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

- **Known box** (`--obstacles`, a 0.3 m box on the oval straight before the explore lap): the box ends up in the
  occupancy grid like any wall, the centerline and racing line route around it, 5 race laps with no crash and no
  safety event. No special code: mapping and planning never knew the difference between a wall and a box.
- **Dropped box** (`--drop-box 6.33 0.65 0.3 0.3 --drop-at-lap 3`, a box on the racing line after planning): the
  safety reflex fires once (`min_ttc` 0.55 s) and slows the car; the new wall cells are mapped while racing, which
  triggers a replan, and the racing line now passes the box on the inside. Lap time before the drop 10.23 s, the lap
  with the drop 9.65 s (the reflex cost about 0.4 s against the expected 9.3 s), then 9.25, 8.63, 8.03 and 7.47 s.
  Zero crashes over 8 laps.
- The track-change detector reported a mismatch of 0.05 to 0.13 while the box was in view, far below the 0.35
  threshold: one box is an obstacle to route around, not a new track.

## Safety events

9 reflex episodes over all 10 runs, none in the first race lap. They happen in the two tight sections of the
hairpin and chicane once the multiplier reaches 1.1, cost 0.1 to 0.2 s each, and mark their section as a problem so
the multiplier backs off 15 % before climbing again.
