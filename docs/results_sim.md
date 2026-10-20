# Simulation results

All numbers come from `logs/laps.csv` and `logs/events.csv` written by `python tools/run_experiments.py`
(three seeds per track, 15 laps each, plus one track-swap run) and summarised by `python tools/plot_learning.py`.
Seeds only change the 1 cm lidar noise. No run crashed.

The sim car has a grip limit (8 m/s², about 0.8 g): a corner taken too fast makes it slide wide, and braking or
accelerating uses up some of that grip. The racing line is a minimum-curvature line, the speed profile is built
from grip with a friction ellipse, and each section learns its own grip, starting from a cautious guess.

## Learning (15 laps, 3 seeds, all seven tracks)

Lap 1 is the explore lap (60 to 122 s). Lap 2 is the first race lap at 60 % of the planned speed
profile. Every section then has a **confidence**: its clean laps in a row. Each clean lap pushes the section up by
5 % times its confidence (5, 10, then 15 % a lap). A problem (the car slid, left the line by more than 0.12 m, or
the safety reflex had to brake) backs the section off 10 %, resets its confidence and remembers the multiplier it
happened at as a ceiling; the section climbs back to just under it and tries 6 % past it after two clean laps.
The multiplier m stands for the section's grip, 6 m/s² x m², and the speed profile is rebuilt from those grips
after every lap.

| track   | first race lap [s] | best lap [s] | laps to within 5 % of best | mean multiplier at lap 15 | lap 15 vs first race lap |
|---------|-------------------:|-------------:|---------------------------:|--------------------------:|-------------------------:|
| oval     | 12.76 |  6.78 | 7 | 1.73 | 46 % faster |
| hairpin  | 16.12 |  7.20 | 7 | 1.71 | 55 % faster |
| chicane  | 14.45 |  7.52 | 7 | 1.73 | 47 % faster |
| twisty   | 25.51 | 12.11 | 7 | 1.82 | 52 % faster |
| snake    | 26.30 | 12.88 | 7 | 1.91 | 50 % faster |
| speedway | 20.89 |  9.98 | 7 | 1.88 | 52 % faster |
| circuit  | 42.38 | 19.51 | 7 | 1.71 | 53 % faster |

These are the tracks drawn 2.5 times bigger (see "Bigger tracks" at the end). On the old small versions they ended at
4.16, 4.44 and 4.69 s, and before 2026-10-19 at 6.17, 6.69 and 6.92 s (a 1.1 multiplier cap, a 3 m/s² plan, and a
reflex that braked for walls the plan never went near). On the long straights the limit is now how fast the car can get and still brake for the next corner (top speed
12 m/s since 2026-10-21, reached nowhere); the corners settle at the grip limit.

![all tracks](plots/laptime_all_tracks.png)

![oval](plots/laptime_vs_lap_oval.png) ![hairpin](plots/laptime_vs_lap_hairpin.png) ![chicane](plots/laptime_vs_lap_chicane.png)
![twisty](plots/laptime_vs_lap_twisty.png) ![snake](plots/laptime_vs_lap_snake.png) ![speedway](plots/laptime_vs_lap_speedway.png)
![circuit](plots/laptime_vs_lap_circuit.png)

![oval multipliers](plots/multipliers_oval.png) ![hairpin multipliers](plots/multipliers_hairpin.png) ![chicane multipliers](plots/multipliers_chicane.png)
![twisty multipliers](plots/multipliers_twisty.png) ![snake multipliers](plots/multipliers_snake.png) ![speedway multipliers](plots/multipliers_speedway.png)
![circuit multipliers](plots/multipliers_circuit.png)

The map, centerline, racing line (coloured by learned speed) and speed profile of every track after 15 laps
(`python tools/plot_plan.py <track> logs/exp/<track>_seed0`):

![oval plan](plots/plan_oval.png) ![hairpin plan](plots/plan_hairpin.png)
![chicane plan](plots/plan_chicane.png) ![twisty plan](plots/plan_twisty.png)
![snake plan](plots/plan_snake.png) ![speedway plan](plots/plan_speedway.png)
![circuit plan](plots/plan_circuit.png)

## Track change (oval to chicane at lap 8)

The chicane PNG replaces the oval under the car while it is racing. Nothing is told to the car.

- last oval race lap 7.07 s, swap, map mismatch jumps from 0.00 to about 0.9
- the car slows to explore speed while the mismatch is above 0.35 and resets 2.0 s after the swap
  (`RACE -> TRACK_CHANGED -> EXPLORE`), deleting map, plan and multipliers
- explore lap on the chicane 64.2 s, first chicane race lap 19.1 s, best chicane lap after 7 race laps 9.0 s
- the swap waits until the car is 0.3 m clear of the new track's walls (`sim.swap_margin_m`; it was 0.4, which the
  minimum-curvature line on the oval never offered, so the swap silently never happened)

![track change](plots/track_change.png)

## Obstacles

(Measured on the small tracks, before 2026-10-20; the box positions have moved with the bigger tracks. The bigger
tracks' obstacle runs are at the end.)

- **Known boxes** (`--obstacles`, one 0.3 m box per track from `config.yaml`, there before the explore lap): the box
  ends up in the occupancy grid like any wall and the centerline and racing line route around it, keeping at least
  0.35 m from it. All five tracks finish 15 laps with no crash (oval 4.20 s best, hairpin 4.55, chicane 5.03,
  twisty 6.75, snake 7.87). No special code: mapping and planning never knew the difference between a wall and a
  box. (Until 2026-10-19 the oval's box sat inside the outer wall, so the oval run tested nothing, and the hairpin,
  chicane and twisty boxes left gaps of 0.62 to 0.75 m in which the explore lap parked against the box for good.
  Every box now leaves at least 0.9 m and is at least 2.5 m from the start.)
- **Dropped boxes**: a box dropped at lap 3 is mapped while racing, which triggers a replan around it. The box at
  `--drop-box 6.33 0.65 0.3 0.3` used to sit on the racing line; the minimum-curvature line passes it anyway, so it
  is mapped and planned around without the reflex firing and the laps keep dropping (4.15 s best). A longer box
  right on the line (`--drop-box 6.0 1.3 0.6 0.3`) makes the reflex brake once, the car swerves round it from the
  next lap and settles at 4.7 s. Zero crashes.
- The track-change detector reported a mismatch of 0.05 to 0.13 while the box was in view, far below the 0.35
  threshold: one box is an obstacle to route around, not a new track.

## Safety events

20 reflex episodes over all 22 runs (7 tracks x 3 seeds + the swap), 18 of them in the first race lap. While the car is on its plan the reflex checks the
path pure pursuit will really drive (the controller run forward on the bicycle model, corner cutting included)
against the live scan, and only brakes for something on that path; off the plan it falls back to the arc check.
Each episode marks its section as a problem, so the multiplier backs off.

## Bolder racing (2026-10-19)

On a random hard track the old car's best lap was 12.09 s and it got slower again (14.4 s by lap 20): four sections
sat at the 0.2 multiplier floor. The reflex fired there every lap even at low speed, because it judged the arc the
current steering would drive for 0.6 s and that arc ran into walls the plan never went near; every firing cost the
section 15 %, every clean lap gave back 8 %. What changed, each found by tracing a failure:

| change | why |
|---|---|
| grip limit in the sim (8 m/s², brakes and throttle take theirs first) | without it the only limits were guesses |
| plan assumes 6 m/s² cornering, 5 braking, 4 accelerating (was 3, 4, 2.5) | real numbers for a 1/10 brushless car |
| confidence-sized steps, ceilings, no 1.1 cap | the old fixed steps saw-toothed and never got near the limit |
| a crash grades the partial lap at once | the lap restarted after every crash, so the section was never slowed and the car crashed there forever |
| target speed = profile x multiplier, then braked again | a slowed section was entered at its neighbour's speed |
| slip detector: yaw rate from the pose vs `v tan(steer) / L` | slides were invisible until they hit a wall; same check works with the car's IMU |
| lift off while sliding | a driver would |
| reflex trusts the plan, checks the path pure pursuit will drive | removes the false alarms, catches corner cutting into a box |
| racing line keeps 0.35 m from every mapped wall or box | 0.3 from the centerline width was 0.25 in practice, less than half the car plus tracking error |
| speed profile reads curvature over 0.6 m | a few-cm kink next to a box read as a 0.2 m hairpin |

Same track afterwards: first race lap 12.5 s, best 7.57 s, steady at 7.8 s, no crash. With the plan driven
perfectly at the full grip the lap would be about 7.0 s, so most of what is left is the margin kept around the
boxes. On the 90-track benchmark the best lap got 34 to 36 % faster (easy 5.82 -> 3.82 s, medium 6.40 -> 4.08 s,
hard 8.05 -> 5.24 s), still with no crash.

## Racing line and control, round 2 (2026-10-20)

Four ideas from the autonomous-racing literature, each measured on the six named tracks over 30 laps (best lap, s):

| step | gen_hard_large_3 | oval | hairpin | chicane | snake | twisty |
|---|---|---|---|---|---|---|
| after "bolder racing" | 7.57 | 4.32 | 4.97 | 5.22 | 8.08 | 7.47 |
| 1. minimum-curvature line, room taken from the distance field | 7.40 | 4.30 | 4.93 | 5.20 | 8.02 | 7.40 |
| ...plus 1.5 s of settling after a new plan before tracking error counts | 7.40 | 4.30 | 4.75 | 5.20 | 8.02 | 7.40 |
| 2+4. grip learned per section, friction-ellipse speed profile | 7.62 | 4.27 | 4.82 | 5.33 | 8.20 | 7.17 |
| speed feedforward (command the planned speed 0.12 s ahead) | 7.88 | 4.03 | 4.90 | 5.35 | 8.30 | 7.28 |
| 3. regulated lookahead (shorter in bends) | **6.68** | **4.15** | **4.63** | **4.78** | **7.52** | **6.67** |

- **Minimum-curvature line** ([Heilmeier et al., TUM](https://arxiv.org/pdf/1902.00606); used by the
  [ForzaETH F1TENTH stack](https://arxiv.org/pdf/2403.11784)): the line is the centerline moved sideways by d at
  every point; its bending (second difference) is linear in d, so the lap is one bounded least-squares problem,
  solved three times around its own result. The bounds come from the map's distance field, so the line keeps 0.35 m
  from walls and boxes alike. Planned laps got 6 to 11 % faster on five of the six tracks.
- **Settling**: the first race lap starts on the explore path, 0.13 m off the new line; that was graded as a tracking
  problem and left the start section under a ceiling of 0.6 for good.
- **Grip per section and the friction ellipse**: braking and accelerating only get the grip a corner leaves, which
  is how the sim tyre works too. On its own about neutral, but it makes the multiplier a physical quantity.
- **Feedforward**: the car averaged 90 % of its planned speed, 0.56 m/s short while accelerating (a proportional
  speed controller lags a ramp by a / kp). Now 99.4 %.
- **Regulated lookahead**: pure pursuit settles about ld^2 * kappa / 2 inside a bend (0.56 m with a 1.5 m lookahead
  in a 2 m bend). The lookahead now shrinks in bends so that is at most 6 cm. Mean tracking error 6 cm -> 6 mm, and
  the corners stopped being punished for an error slowing down could not fix. This was the big one.

After all of them every track drives within 0 to 5 % of the plan with the true grip (8 m/s²), and the median corner
grip learned is about 8 m/s². A smaller wall clearance (0.25 to 0.30 m) bought about 3 % on plain tracks but cost
time next to boxes, and the real car's pose will be a few centimetres off, so it stays 0.35 m.

## Tuning round (2026-10-20)

With the car driving its plan closely, the remaining questions were which numbers to trust. Each change was run on
the six tracks for 30 laps against the previous version (best lap, s); the full set of tried variants is in the commit
message.

| change | gen_hard_large_3 | oval | hairpin | chicane | snake | twisty |
|---|---|---|---|---|---|---|
| start of the round | 6.68 | 4.15 | 4.63 | 4.78 | 7.52 | 6.67 |
| plan uses the car's real acceleration and braking (6 / 6 m/s²) and 60 learning sections | 6.38 | 4.15 | 4.48 | 4.73 | 7.27 | 6.60 |
| probe past a ceiling after 2 clean laps by 6 %, allow 10 cm of corner cutting | **6.20** | **4.12** | **4.45** | **4.48** | **7.02** | **6.45** |

Every track now drives within 0 to 1.4 % of what its line allows at the true grip. Tried and dropped: bigger learning
steps (overshoot), other length weights for the line (one track better, another worse), 3 cm and 15 cm corner cutting,
and a line weighted toward minimum time (planned up to 2 % faster, drove no faster). The closest the car body came to
any wall or box over 20 laps on the tightest tracks was 21 cm.

## Reverse recovery

The car can now reverse (at most 0.5 m/s). When it wants to drive but has not moved for 1 s, it backs up 0.5 m
(0.75 m, 1.0 m on later tries at the same spot) with the wheels turned away from the side the controller is aiming
for, so the nose swings toward the way out. The back-up stops early if the body would touch a wall already in the
map or the outermost beams see something within 0.2 m. After three tries at one spot, or 30 s without getting 1 m
anywhere, it gives up: headless runs end with `STUCK` and exit code 3 instead of running until `--max-s`.

The old, too-tight config boxes make good test cases, dropped at lap 0 (`--drop-box X Y 0.3 0.3 --drop-at-lap 0`):

| box | gap | before | with recovery |
|---|---|---|---|
| chicane (4.0, 1.3) | 0.64 m | parked for good at 26 s of the explore lap | one back-up in the explore lap, then 6 laps down to 7.85 s, no crash |
| twisty (6.0, 0.55) | 0.75 m | parked for good | one back-up, then 6 laps down to 9.95 s, no crash |
| hairpin (5.0, 1.25) | 0.62 m | parked for good | `STUCK` after three tries at the same spot in the second lap, at 69 s |

Recovery gets the car out; it does not make a gap the planner cannot use drivable. That is the generator's job: it
never puts a box closer than 0.85 m to the far wall.

## Random tracks

(Small and medium areas, before 2026-10-20. The numbers for the bigger areas are at the end.)

`python tools/bench_random.py --n 30` generates 30 tracks per difficulty (10 x 7 m area, known, dropped and slalom
boxes) and drives each for 10 laps headless, in parallel, each run in its own log folder.

| difficulty | runs OK | lap | explore lap | first race lap (mean) | best lap (mean) | best vs first | back-ups |
|---|---|---|---|---|---|---|---|
| easy   | 30 / 30 | 12.6 - 16.7 m | 16 - 25 s | 6.8 s | 3.7 s | 45 % faster | 5 |
| medium | 30 / 30 | 12.2 - 18.2 m | 18 - 30 s | 6.9 s | 3.7 s | 46 % faster | 4 |
| hard   | 30 / 30 | 16.8 - 22.1 m | 28 - 41 s | 9.4 s | 4.9 s | 47 % faster | 4 |

No crash, no false track change, and the best lap beats the first race lap on all 90. On 300 tracks of the large
size (10 x 10 m, seeds 100 to 199): 299 of 300 OK, no crash, one stuck for good. The same 300 with recovery
switched off (`recovery.max_attempts: 0`): easy 97, medium 80, hard 70 OK; the other 53 park against a box or a wall.

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

## Bigger tracks (2026-10-20)

The tracks were too small to show what the car can do: the longest straight on the old oval was about 5 m. They are
now as big as the 12 m lidar sensibly allows.

- The five hand-made tracks are drawn 2.5 times bigger (25 x 17.5 m), and two new ones are 30 x 20 m: `speedway`
  (two long straights and a V) and `circuit` (101 m, sixteen corners). Random tracks get an `xl` size, 30 x 20 m.
  Big areas are drawn at 2 cm per pixel instead of 1 cm, and the map grid covers 64 m.
- The width changes along every lap: new random targets every 4 to 10 m, blended smoothly, between 1.0 and 3.0 m
  depending on difficulty. Every check that used one width (pass gaps, box room, the planner check) uses the local
  one.
- The window zooms to fit the track on screen.
- Learning sections are about 0.4 m long (at least 60), so a 100 m lap gets 250 of them.

![tracks](plots/tracks.png)

30 laps per track, then 15 with a box on each, plus a box dropped on the oval at lap 3 and the oval swapped for the
chicane at lap 8 (best lap, s; no crash and no stuck run in any of the 16):

| track | lap | explore lap | first race lap | best | with a box |
|---|---|---|---|---|---|
| oval | 52 m | 60.4 | 12.8 | 6.77 | 6.85 |
| hairpin | 49 m | 59.5 | 16.1 | 7.05 | 7.15 |
| chicane | 53 m | 64.5 | 14.5 | 7.42 | 7.58 |
| twisty | 71 m | 84.9 | 25.5 | 11.70 | 12.03 |
| snake | 73 m | 88.2 | 26.4 | 12.65 | 12.87 |
| speedway | 70 m | 82.1 | 20.9 | 9.95 | 10.37 |
| circuit | 101 m | 122.3 | 42.4 | 19.07 | 19.45 |

With the old 8 m/s top speed the car hit it on every track by lap 10 (best laps 7.07 / 7.25 / 7.65 / 11.85 / 12.85 /
10.28 / 19.42 s). With 12 m/s (2026-10-21) it peaks at 10.0 to 10.6 m/s on oval, speedway and circuit: the straights
are too short to go faster and still brake for the next corner at 6 m/s². The best laps above got 1 to 3 % faster,
with no new reflex event or crash. Stopping from 12 m/s at 6 m/s² takes 12 m, the lidar's range, which is why it
stops there.

What the bigger tracks broke, and the fixes:

| problem | fix |
|---|---|
| the lap counter wanted the car within 0.5 m of its start; on a wide track the racing line passes 1 m away | a lap is crossing a finish line through the start (1.6 m each side), forwards, after at least 3 s and 5 m |
| explore centring swung to full lock on wide tracks and hugged a wall | centre on the relative difference, scaled to a 1.5 m track |
| boxes in the middle of a wide track | follow the gap where the track is wider than 2 m or something is in the car's path within 2 m |
| slaloms on the explore lap: the gap steering aimed 2 m out and turned too gently for a box 1 m ahead | aim no further than the first thing in the car's path; a box with room on both sides gets the smallest sidestep that clears it by 20 cm, aimed from the nose, kept to one side until past |
| the car turned its side into a box corner it was passing | never pick a steer whose arc hits something within 1 m (the reflex's body check) |
| the last metres of the explore path, after the finish line, folded back into the start | drop the path's last 2 m back into the start before building the centerline |
| centerline knots between two close boxes, where the middle of the room jumps sideways | snap all points together: as close to the middle as possible, each inside its own free stretch, neighbours pulled together (one bounded least-squares solve); cut out any small loop left over |
| a replan round a dropped box re-snapped the centerline through the box; the knots grew every replan, 17 km of line on an 80 m track | a replan keeps the centerline and only rebuilds the racing line; points that are now too close to something get the free room on one side, the same side for the whole stretch, never across a wall |
| replans waited 3 s, so a box seen just after another replan was planned round too late | 1 s |
| boxes dropped right in front of a fast car | a drop waits until the car is 1.5 m plus its braking distance plus 1 s away |
| when every arc was blocked the reflex kept steering toward the box while the car rolled to a stop | on a tie, steer away from the nearest hit |
| wide corners looked like two passes of the track too close together | a second pass only counts if it runs more than 120 degrees the other way |

Random tracks, 10 laps each, all three kinds of boxes:

| size | tracks | finished | crashes | best vs first race lap |
|---|---|---|---|---|
| medium (10 x 7 m) | 90 | 89 | 0 | 46 % faster |
| large (10 x 10 m) | 90 | 90 | 0 | 45 to 47 % faster |
| xl (30 x 20 m) | 60 | 57 | 0 | 45 to 48 % faster |

The four that did not finish all stopped on the explore lap: the car reached the corner of a slalom box at a slight
angle, nose within a few centimetres of it, and three back-ups at the same spot did not get it out. Before the fixes
above, xl was 47 of 60 (and an earlier version 37 of 60).

![success](plots/bench_success.png) ![improvement](plots/bench_improvement.png)

![gallery](plots/bench_gallery.png)
