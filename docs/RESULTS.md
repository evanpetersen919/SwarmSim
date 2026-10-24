# Results: the quadrotor swarm

These results are for the simulator, with the parameter sets in [MODELS.md](MODELS.md). They are for the quadrotor swarm of the original research question. The fixed-wing aircraft has only pilots so far (see the end of this page).

Research question: under bursty packet loss and weather-degraded sensing, how much does neighbor prediction with a fallback state machine preserve formation integrity compared with holding the last reported state?

## What was run

| Experiment file | Cells | Seeds | Runs | What it varies |
|---|---|---|---|---|
| `experiments/baselines.yaml` | 48 | 30 | 1,440 | Neighbor predictor (hold-last, dead reckoning, Kalman, oracle), independent loss (10, 30, 50 percent), jamming, a drone failure |
| `experiments/baselines_burst.yaml` | 48 | 30 | 1,440 | The same, with bursty loss (mean burst of 6 messages) at the same average rates |
| `experiments/ablation.yaml` | 48 | 30 | 1,440 | The full controller and three variants that each disable one component (prediction, fallback, separation) |
| `experiments/ablation_fog.yaml` | 16 | 30 | 480 | The full controller and the variant without the fallback, in fog thick enough to lose the position fix (four visibilities, 200 to 110 m), with and without pulsed jamming |
| `experiments/matrix.yaml` | 432 | 30 | 12,960 | Loss, weather, jamming, swarm size, a drone failure, and the scene, with the default controller |

Every run succeeded (none failed). The result tables include the share of alive drones in each fallback state (the `fallback_*` columns, computed from the logs). Every 200th run was re-executed from its log (8 runs of each 1,440-run experiment and 65 of the matrix) and all matched. A smaller run of `matrix.yaml` with 5 seeds (2,160 runs) was done first and is not used here.

Setup: Windows, MSVC, the release preset, `swarm_batch` 0.1.0 with 16 jobs, on an Intel Core i9-13900K with 32 GB of memory. Seeds 1 to 30. The simulation code (`core/`, `apps/`, `net/`) is the code of commit `a472c9e`, identical to `c9decf8`. The tables do not depend on the job count. Results from another compiler or platform may differ, because floating-point behavior is not guaranteed across them (see [DETERMINISM.md](DETERMINISM.md)).

To regenerate them:

```sh
cmake --preset release
cmake --build --preset release --target swarm_batch
for e in baselines baselines_burst ablation ablation_fog matrix; do
  build/release/apps/swarm_batch experiments/$e.yaml --out results/${e}_quad_30.csv \
    --jobs 16 --seeds 30 --verify-every 200
done
cd analysis && python results_quad.py ../results
```

The metrics are computed from the run logs. The formation error is the settled root-mean-square distance of the drones from their slots, after the first 20 s of a 60 s run. The minimum separation is the closest approach of any two drones. Intervals are 95 percent Student t intervals for the mean over runs. A paired difference is a - b over the runs that share every other axis level and the seed. Each baselines experiment pools its jamming and failure levels, so each group is 360 runs.

## Neighbor prediction

Settled formation error and minimum separation, by predictor (means, with 95 percent intervals; 360 runs each):

| Predictor | Formation error, independent loss | Formation error, bursty loss | Minimum separation, independent loss | Minimum separation, bursty loss |
|---|---|---|---|---|
| Hold-last | 3.61 m [3.42, 3.81] | 3.96 m [3.76, 4.17] | 3.46 m [3.37, 3.55] | 3.09 m [2.98, 3.20] |
| Dead reckoning | 1.47 m [1.42, 1.53] | 1.52 m [1.46, 1.59] | 4.16 m [4.08, 4.24] | 4.11 m [4.02, 4.21] |
| Kalman | 1.47 m [1.41, 1.53] | 1.53 m [1.46, 1.60] | 4.01 m [3.93, 4.08] | 4.05 m [3.96, 4.15] |
| Oracle | 3.52 m [3.30, 3.74] | 3.52 m [3.30, 3.74] | 3.79 m [3.72, 3.86] | 3.79 m [3.72, 3.86] |

Paired differences, Kalman minus hold-last (360 pairs):

| Loss | Formation error | Minimum separation |
|---|---|---|
| Independent | -2.14 m [-2.34, -1.95] | +0.55 m [0.50, 0.59] |
| Bursty | -2.44 m [-2.64, -2.24] | +0.96 m [0.86, 1.06] |

- With the runs that do and do not have a failed drone pooled, prediction reduces the settled formation error compared with holding the last state by about 2.1 m under independent loss and 2.4 m under bursty loss, from about 3.6 and 4.0 m. Most of that is a failure effect (the next section).
- The hold-last predictor gets worse as the loss rises, and faster under bursty loss. Its formation error goes from 3.62 to 4.32 m and its minimum separation from 3.49 to 2.63 m between 10 and 50 percent bursty loss. The Kalman predictor changes little over the same range (formation error 1.45 to 1.66 m, minimum separation 4.03 to 4.18 m).
- Dead reckoning is as good as the Kalman filter on the formation error (a paired difference of 0.00 m for independent loss, and 0.004 m [-0.004, 0.012] for bursty loss), and its minimum separation is slightly higher (by 0.16 m for independent loss and 0.06 m for bursty loss). The Kalman filter is not shown to be better than the simpler predictor.
- The oracle, which reads the neighbors' true state, is an upper bound when no drone fails (1.44 m, the same as dead reckoning and the Kalman filter) but not when one does. Its pooled 3.52 m mixes the two cases (next section).

### Where the difference comes from: a failed drone

The same runs split by whether one drone fails (settled formation error; 180 runs per predictor in each row, pooled over loss level and jamming):

| Failure | Loss | Hold-last | Dead reckoning | Kalman | Oracle | Kalman minus hold-last (paired) |
|---|---|---|---|---|---|---|
| None | Independent | 1.74 m | 1.44 m | 1.44 m | 1.44 m | -0.30 m [-0.32, -0.27] |
| None | Bursty | 2.09 m | 1.49 m | 1.50 m | 1.44 m | -0.60 m [-0.66, -0.53] |
| One drone | Independent | 5.49 m | 1.50 m | 1.50 m | 5.60 m | -3.99 m [-4.05, -3.93] |
| One drone | Bursty | 5.83 m | 1.55 m | 1.56 m | 5.60 m | -4.28 m [-4.36, -4.20] |

- With no failure, prediction helps by 0.30 m under independent loss and 0.60 m under bursty loss, and the oracle equals dead reckoning and the Kalman filter.
- When one drone fails, hold-last and the oracle go to about 5.5 m and dead reckoning and the Kalman filter stay near 1.5 m. The cause, from the code: the formation target averages the position of every neighbor less its slot offset, weighted by the inverse of its uncertainty. A failed drone stops and goes silent. The hold-last predictor reports the uncertainty of its last report unchanged (`neighbor_predictor.cpp`) and the oracle reports zero, so a failed drone keeps its full weight and the surviving drones keep averaging in a frozen position while the formation moves on. Dead reckoning and the Kalman filter let the uncertainty of a silent neighbor grow, so its weight falls and the survivors stop following it.
- So most of the pooled prediction effect is a property of how the formation consensus treats a silent neighbor, not of tracking healthy neighbors better. A rule that drops a silent neighbor from the consensus removes most of that gap; it was tested after this release and is described in the next section.
- The minimum separation does not depend on the failure level (it is the same in both halves), so its gain from prediction (0.55 m under independent loss and 0.96 m under bursty loss) is a separate effect.

### A rule that drops a silent neighbor

The controller has an optional setting, `control.drop_stale_after_ticks` (0 = off, the default, so every table above is unchanged). When set, a neighbor whose latest report is older than that many ticks is left out of the formation consensus; separation still keeps clear of its prediction. `experiments/baselines_drop.yaml` and `experiments/baselines_burst_drop.yaml` repeat the baselines with the rule off (`keep`) and on at 100 ticks (`drop100`, 2 s at the 0.02 s step, 20 report periods). 100 is the only threshold tried and was fixed before the run. Each file is 96 cells, 30 seeds, 2,880 runs, none failed, with the same setup as above.

Settled formation error (180 runs per predictor in each row, pooled over loss level and jamming):

| Failure | Loss | Hold-last, keep | Hold-last, drop100 | Dead reckoning and Kalman | Oracle | Kalman minus hold-last, drop100 (paired) |
|---|---|---|---|---|---|---|
| None | Independent | 1.74 m | 1.74 m | 1.44 m | 1.44 m | -0.30 m [-0.32, -0.27] |
| None | Bursty | 2.09 m | 2.02 m | 1.49 and 1.50 m | 1.44 m | -0.53 m [-0.59, -0.47] |
| One drone | Independent | 5.49 m | 1.80 m | 1.50 m | 5.60 m | -0.30 m [-0.32, -0.28] |
| One drone | Bursty | 5.83 m | 2.08 m | 1.55 and 1.56 m | 5.60 m | -0.52 m [-0.59, -0.46] |

- The rule changes hold-last and nothing else. Paired by seed, drop100 minus keep for hold-last with a failed drone is -3.69 m [-3.74, -3.63] under independent loss and -3.76 m [-3.82, -3.70] under bursty loss. For dead reckoning and the Kalman filter the difference is zero to the printed precision, because a silent neighbor's weight has already fallen with its growing uncertainty.
- With the rule on, the Kalman advantage over hold-last with a failed drone falls from 3.99 and 4.28 m to 0.30 and 0.52 m, which is about the no-failure advantage. The large failure effect in the tables above comes from the consensus treating a silent neighbor at full weight, as the section above says, and not from prediction quality.
- The oracle is unchanged at 5.60 m with a failure. Its estimates are supplied from outside and counted as always fresh, so the rule does not apply to them. The oracle's failure result is a property of reading the truth of a drone that no longer moves, not of the rule.
- Minimum separation and collision episodes do not change by more than the intervals: the largest paired difference is +0.014 m [-0.001, +0.030] for hold-last under bursty loss.
- The rule does not improve the proposed method (Kalman) on these scenarios. It makes the hold-last baseline a fairer comparison. Reproduce with `python results_quad.py ../results` after running the two experiments with the command above.

## Ablation

| Variant | Formation error | Minimum separation | Collision episodes per run |
|---|---|---|---|
| Full controller | 1.47 m [1.41, 1.53] | 4.01 m [3.93, 4.08] | 0.00 |
| No prediction (hold-last) | 3.61 m [3.42, 3.81] | 3.46 m [3.37, 3.55] | 0.01 [0.00, 0.02] |
| No fallback | 1.44 m [1.39, 1.49] | 4.01 m [3.93, 4.08] | 0.00 |
| No separation | 1.55 m [1.50, 1.60] | 1.01 m [0.94, 1.07] | 0.58 [0.52, 0.64] |

Paired differences against the full controller (360 pairs): removing prediction raises the formation error by 2.14 m [1.95, 2.34]; removing separation lowers the minimum separation by 3.00 m [2.91, 3.09] and adds 0.58 collision episodes per run [0.52, 0.64]; removing the fallback state machine changes the formation error by -0.03 m [-0.04, -0.02] and the minimum separation by 0.00 m.

- Prediction and the separation rule each do what they are for. The no-prediction row carries the failed-drone effect of the previous section.
- The fallback state machine is not shown to help in these scenarios, but these scenarios barely use it. The alive drones were in the degraded-communication state 4 percent of the time (2, 4, and 6 percent at 10, 30, and 50 percent loss, and 9 percent under pulsed jamming) and in the degraded-navigation and safe-hold states 0.00 percent of the time. So this ablation tests only the degraded-communication state, where the fallback changes nothing. The next section tests the other states.

## The fallback state machine in thick fog

`experiments/ablation_fog.yaml` runs the full controller and the variant without the fallback in fog thick enough to lose the LiDAR position fix, so that the degraded-navigation and safe-hold states are used. With the fallback on, a drone that has lost its fix goes into safe hold and stops; the mission reference moves on, so the formation error measured against the moving slots grows. Without the fallback the drones keep flying on their dead-reckoned position. 480 runs; 60 per variant and visibility (30 seeds with and without pulsed jamming).

| Visibility | Share in degraded navigation and in safe hold (fallback on) | Formation error, fallback on | Formation error, fallback off | Minimum separation, on and off |
|---|---|---|---|---|
| 200 m | 1 percent and 1 percent | 5.2 m | 1.9 m | 4.09 m and 4.09 m |
| 150 m | 53 percent and 40 percent | 97.6 m | 21.4 m | 5.91 m and 4.21 m |
| 130 m | 48 percent and 46 percent | 104.9 m | 22.3 m | 6.22 m and 4.37 m |
| 110 m | 46 percent and 49 percent | 108.5 m | 23.2 m | 6.29 m and 4.34 m |

Paired differences, fallback off minus on (60 pairs per visibility): the formation error is 76.2 m [73.6, 78.8], 82.5 m [80.2, 84.9], and 85.4 m [82.9, 87.8] lower without the fallback at 150, 130, and 110 m, and 3.3 m [1.1, 5.5] lower at 200 m. The minimum separation is 1.70 m [0.99, 2.41], 1.85 m [1.21, 2.50], and 1.95 m [1.26, 2.64] lower without the fallback at 150, 130, and 110 m, and the same at 200 m.

- In thick fog the fallback trades formation error for separation. Stopping in safe hold costs about 76 to 85 m of formation error against the moving slots and keeps the drones about 1.7 to 1.9 m farther apart at their closest.
- The formation error with the fallback on is the cost of stopping, not a failure to hold position: the drones are meant to stop when they cannot navigate.
- Collision episodes are 0.03 or fewer per run, with and without the fallback, and the differences are not distinguishable from zero. In these runs the fallback is not shown to prevent collisions; none happen without it.

## The matrix

The default controller across the full matrix (12,960 runs; the axes are fully crossed, so each row pools the other axes). Settled formation error, with the minimum separation:

| Axis | Level | Formation error | Minimum separation |
|---|---|---|---|
| Loss | 0 / 10 / 30 / 50 percent | 6.37 / 6.62 / 7.08 / 7.57 m | 4.43 / 4.44 / 4.49 / 4.53 m |
| Weather | clear / fog / rain | 4.05 / 12.63 / 4.05 m | 4.44 / 4.53 / 4.44 m |
| Jamming | none / constant / pulsed | 6.24 / 8.11 / 6.39 m | 4.47 m in each |
| Swarm size | 5 / 10 / 20 drones | 2.63 / 3.43 / 14.67 m | 5.51 / 4.04 / 3.86 m |
| Failure | none / one drone | 6.88 / 6.94 m | 4.47 m in each |
| Scene | field / hills | 6.02 / 7.80 m | 4.47 m in each |

- Fog, constant jamming, a swarm of 20, and hills each raise the formation error. The swarm of 20 raises it most (14.67 m against 3.43 m for 10).
- Loss raises the formation error steadily, by about 1.2 m from none to 50 percent.
- The rain level of the matrix (8 mm/h) changes nothing: its results equal clear air in every digit. The weather model gives 8 mm/h an extinction of about 0.0016 per meter (placeholder rain constants, see [MODELS.md](MODELS.md), plus the clear-air visibility of 20 km), a two-way transmission of about 0.83 over the LiDAR's 60 m maximum range. The LiDAR loses a return only when its relative intensity falls below a threshold of 0.05, which this level does not approach, and that is consistent with identical runs. A rain level that matters would need a higher rate or a larger coefficient.
- Collision episodes average below 0.005 per run in every row of the matrix. The ablation scenario is the one that shows the separation rule.

## What these results do not show

- They come from one scene family, one controller design, and one simulator. The weather, loss, and jamming models are parametric and are not validated against real data, apart from one published visibility curve.
- The intervals are for the mean over runs, treating runs as independent. The pooled groups mix scenarios (jamming and failure levels); the per-cell numbers are in the result tables.
- The hold-last baseline as implemented keeps a silent neighbor at constant certainty, so the comparison with it is partly a comparison of how a silent neighbor is treated (the drop rule above removes most of that, for hold-last). It is not a measurement of how well a predictor tracks healthy neighbors, which is the 0.30 and 0.60 m of the no-failure rows.
- The fallback result is for one kind of failure (a lost LiDAR position fix in fog) and one degraded-communication state in the other scenarios. The state machine was not tested against other kinds of navigation failure.

## The fixed-wing aircraft

There are no benchmark results for the fixed-wing aircraft, only three pilots, described in [phases/flight-realism.md](phases/flight-realism.md). They found that fog (blinding the camera position fix) is the only effect that reaches the flight, that packet loss and the neighbor predictor have no measurable effect on it, and that the radar altimeter did not reduce the worst dip towards the ground when the terrain map was present (the experiment below, with a wrong map, is where it and the forward sensor show an effect). The research question is a question about the quadrotor swarm for that reason.

### Sensors that look down and ahead, when the prior map is wrong

ADR 0013 asked: when the map the aircraft carries is wrong, does a sensor that measures the ground keep the aircraft off it, and what does that cost? `experiments/fixed_wing_forward.yaml` crosses eight map levels, four sensor arms (none, the radar altimeter, the forward obstacle sensor, both) and two weather levels (clear, fog of 750 m visibility): 64 cells, 30 seeds, 1,920 runs, none failed. The scenario is `base_fixed_wing_forward.yaml`: twelve aircraft on one 80 m orbit over 120 m hills at a mission altitude of 20 m, below the 60 m clearance, so the altitude flown is set by the prior map. At the 100 m of the dense base the map never decided anything and a wrong map changed no run (found by the probe `probe_fixed_wing_map_errors.yaml`, five seeds, not a result). The levels were fixed from the probe before the experiment and committed before it ran. Same setup as above: Windows, MSVC, the release preset, 16 jobs, seeds 1 to 30. Reproduce with `swarm_batch experiments/fixed_wing_forward.yaml --out results/fixed_wing_forward_30.csv --jobs 16 --seeds 30` and `python results_forward.py ../results` in the analysis directory.

Safety is `ground_contacts`, the number of the twelve aircraft that reached the ground. Cost is `mean_agl_m`, the mean height above the surface, and `orbit_altitude_rmse_m` and `time_on_station`. Means over 30 runs; differences are paired by seed with 95 percent intervals; clear air.

| Map | None: contacts / mean height | Altimeter | Forward | Both |
|---|---|---|---|---|
| Exact | 0 / 88.6 m | 0 / 88.6 m | 0 / 89.1 m | 0 / 89.1 m |
| Offset -30 m | 0 / 58.6 m | 0 / 65.3 m | 0 / 68.8 m | 0 / 72.3 m |
| Offset -60 m | 3.03 / 25.6 m | 0 / 54.8 m | 0 / 62.7 m | 0 / 67.5 m |
| Offset -80 m | 10.23 / 6.8 m | 3.63 / 38.3 m | 3.07 / 46.6 m | 3.03 / 49.9 m |
| One square omitted | 9.20 / 10.1 m | 3.53 / 39.8 m | 3.07 / 47.1 m | 3.07 / 50.2 m |
| Random error, 20, 60, 120 m | 0 | 0 | 0 | 0 |

- **With a wrong map, either sensor keeps aircraft off the ground.** At an offset of -60 m the paired difference in contacts against no sensor is -3.03 [-4.73, -1.34] for the altimeter and for the forward sensor. At -80 m it is -6.60 [-8.08, -5.12] for the altimeter and -7.17 [-8.37, -5.96] for the forward sensor, and with the omitted square -5.67 [-7.20, -4.13] and -6.13 [-7.57, -4.69]. With an exact map they do nothing useful: contacts stay 0, and the forward sensor adds 0.52 m [0.21, 0.84] to the mean height.
- **The forward sensor is not shown to be safer than the altimeter.** Forward minus altimeter in contacts is 0.00 at -60 m, -0.57 [-1.28, +0.15] at -80 m and -0.47 [-1.09, +0.16] with the omitted square: the intervals include zero. Both together are no different from the forward sensor alone.
- **They differ in cost.** The forward sensor holds the orbit better and flies higher. Against the altimeter, time on station is +0.24 [0.16, 0.33] at -60 m and +0.33 [0.24, 0.42] at -80 m, and mean height is +7.96 m [5.50, 10.41] and +8.30 m [5.77, 10.83]. The altimeter reacts after the aircraft is already low and climbs in bursts, which breaks the station: its time on station falls by 0.23 and 0.40 against no sensor, the forward sensor's by 0 and 0.07.
- **Staying clear is expensive.** At -60 m the altimeter adds 29.2 m [25.8, 32.6] to the mean height and 26.0 m to the altitude error, the forward sensor 37.1 m [33.3, 40.9] and 32.5 m. For reference, aircraft with the exact map fly at a mean height of 88.6 m, so at -60 m the altimeter recovers about half of the height that the wrong map took away (54.8 m against 25.6 m without a sensor) and the forward sensor about three fifths (62.7 m). The sensors do not restore the clearance; they keep the aircraft off the ground.
- **A random error in the map costs height and not safety.** The map holds the highest height under the aircraft's footprint, so a zero-mean error raises the aircraft more than it lowers them. No run has a contact at 20, 60 or 120 m; the mean height rises from 88.6 m to 99.0, 145.6 and 227.3 m. The sensors change nothing there, to the printed precision at 60 and 120 m: they never need to raise a command that is already high.
- **Fog of 750 m visibility does not change the safety results or the comparison between sensors.** At -80 m the contacts are 10.23, 3.63, 3.03 and 3.03 in fog, as in clear air, and the paired differences match to within their intervals. The forward sensor's detection distance falls from its 800 m range to about 400 m, and the climb it needs is still started in time, so this fog level did not test the sensor's limit. Fog does lower time on station by about 0.06 where it was checked (random error of 120 m: 0.93 to 0.87, with and without sensors), through the camera position fix.
- **Not every loss is the sensor's to prevent.** About 3 of the twelve aircraft are lost in the -80 m and omitted-square cells whatever the sensor does, and the number does not change with the forward sensor's range (400, 800, 1500 m) or its dropout probability (5, 25, 50 percent) in `fixed_wing_forward_sensitivity.yaml` (36 cells, 1,080 runs, none failed; the same 3.07 in every cell). In a half-second run, before any sensor can act, the -80 m cells already have 1.9 aircraft on the ground: these aircraft launch below the real ground, because the start altitude comes from the wrong map. The remaining 1.2 per run were not examined. The sensors cannot help an aircraft that starts underground, and this experiment does not separate that from a climb that is too slow.

What these results do not show. They are for one scenario family (12 aircraft, one orbit, hills), one sensor model whose parameters are all assumed, and map errors of three kinds and a few sizes that a scenario author chose; the sizes were chosen to bracket contact, so the table says what happens near that edge and not how often real maps are wrong by that much. The forward sensor reads the scene's true surface and only raises the altitude command, so ground that rises faster than the aircraft can climb is still reached; turning and slowing down are built and measured in the next section. The aircraft is a model, and none of this was tested on hardware.

### Turning and slowing for what climbing cannot clear

ADR 0014 asked: where the ground rises faster than an aircraft can climb, does turning and slowing keep it off the ground, and what does that cost the formation? `experiments/fixed_wing_avoid.yaml` crosses four terrain steepness levels (hills of 40, 240, 400, and 480 m amplitude), four map levels (none, exact, 60 m too low, an omitted square), three arms (no sensor, the forward sensor with its altitude floor only, the forward sensor with avoidance) and two weather levels (clear, fog of 375 m visibility): 96 cells, 30 seeds, 2,880 runs, none failed. The scenario is `base_fixed_wing_avoid.yaml`: six ALTIUS-700-class aircraft (figures supplied without a source) at 31.3 m/s on a 300 m orbit at 100 m over generated hills. The levels were fixed from a probe (`probe_fixed_wing_avoid.yaml`, five seeds) before the experiment and committed before it ran; the generator's hills peak at about a third of their amplitude under the orbit, so 40 m is a gentle control and 480 m puts most aircraft without a sensor on the ground. A sensitivity sweep (`fixed_wing_avoid_sensitivity.yaml`, 12 cells, 360 runs, none failed) varies the two assumed numbers of the rule. Same setup as above. Reproduce with `swarm_batch experiments/fixed_wing_avoid.yaml --out results/fixed_wing_avoid_30.csv --jobs 16 --seeds 30` and `python results_avoid.py ../results`.

Safety is `late_contacts`, the aircraft of six that reached the ground after the first record (`ground_contacts` less `launch_contacts`, so aircraft that start under the ground do not count); cost is `time_on_station`, `orbit_radial_rmse_m`, `mean_agl_m`, and `collision_episodes`. Clear air, no map, means over 30 runs:

| Hills | Late contacts: none / forward / avoid | Avoid minus forward, paired | Time on station: forward / avoid | Radial error: forward / avoid | Collision episodes: none / forward / avoid |
|---|---|---|---|---|---|
| 40 m | 0 / 0 / 0 | 0 | 1.00 / 1.00 | 1.3 m / 1.3 m | 0 / 0 / 0 |
| 240 m | 0.53 / 0.07 / 0.03 | -0.03 [-0.10, +0.03] | 0.98 / 0.23 | 2.0 m / 86.2 m | 0.37 / 0.00 / 0.00 |
| 400 m | 4.57 / 2.03 / 0.30 | -1.73 [-2.33, -1.14] | 0.30 / 0.07 | 11.7 m / 201.6 m | 2.17 / 0.83 / 0.00 |
| 480 m | 5.03 / 3.13 / 0.53 | -2.60 [-3.22, -1.98] | 0.19 / 0.06 | 19.2 m / 225.4 m | 1.60 / 0.67 / 0.00 |

- **Avoidance keeps most of the remaining aircraft off the ground, where climbing cannot.** On the steep hills it takes the late contacts from 2.03 to 0.30 of six at 400 m and from 3.13 to 0.53 at 480 m, against 4.57 and 5.03 with no sensor. With the omitted square the numbers are nearly the same (2.03 to 0.30 and 3.13 to 0.57), as the map is no help there either. At 240 m the altitude floor alone already leaves 0.07, and avoidance is not shown to add to it.
- **It does this by giving up the formation.** Time on station falls to 0.23, 0.07, and 0.06, and the radial error to the orbit rises to 86, 202, and 225 m, against 0.98, 0.30, and 0.19 and 2, 12, and 19 m with the forward sensor alone. The aircraft that dodges does not come back to its circle within the 90 s of the run. At the mildest level, 240 m, the formation is lost for no gain in safety that the intervals can show. The data do not say why the aircraft do not return; the hold and the orbit guidance are as described in docs/MODELS.md, and a likely cause, which was not tested, is that the orbit takes the aircraft straight back toward the ground it turned away from, so it is turned away again.
- **Collisions between aircraft fall.** Collision episodes (two aircraft closer than 10 m) are 2.17 and 1.60 with no sensor, 0.83 and 0.67 with the forward sensor alone, and 0.00 with avoidance at 400 and 480 m. Part of the no-sensor figure is aircraft that crashed in the same place.
- **With an exact map nothing reaches the ground in any arm, and avoidance still costs the formation.** The map plans the climb in time. Avoidance triggers anyway, because it judges a return unclearable against the climb budget (0.6 of the vehicle's climb limit) and not against what the vehicle could do: time on station is 1.00, 0.96, and 0.94 with the forward sensor and 0.89, 0.75, and 0.71 with avoidance at 240, 400, and 480 m. These are false alarms, and a cost with no benefit.
- **With a map 60 m too low** the altitude floor alone already leaves almost nothing on the ground (late contacts 0.00 in the forward arm at every level) and avoidance costs the formation (time on station 0.24, 0.15, and 0.16 against 1.00, 0.93, and 0.86).
- **Fog of 375 m visibility, which cuts the detection distance to about 200 m, hurts the forward sensor and avoidance too.** With no map at 400 and 480 m the late contacts are 3.40 and 4.50 with the forward sensor alone (against 2.03 and 3.13 in clear air) and 0.67 and 1.50 with avoidance (against 0.30 and 0.53); avoid minus forward is -2.73 [-3.56, -1.91] and -3.00 [-3.67, -2.33]. A turn at 31.3 m/s has a radius of 143 m, so it has little room at that range, and avoidance still helps more than it fails. With an exact map the two arms are identical in fog.
- **The safety result does not depend on the two assumed numbers, the cost does.** In the sweep (400 m hills, clear air) the late contacts are 0.30 to 0.43 for every horizon (6, 12, and 20 s) and bank fraction (0.5 and 0.9) with no map, and 0.00 with the map 60 m too low. The horizon changes the cost: the radial error is 63 to 66 m with a horizon of 6 s and 189 to 222 m with 12 and 20 s, and time on station is 0.10 against 0.07 (0.29 against 0.13 to 0.15 with the map 60 m too low). A horizon of 6 s keeps the safety at a third of the formation error of the default; the default of 12 s was fixed before the experiment and is not changed here.
- **What is left.** With avoidance about 0.3 and 0.5 of six aircraft still reach the ground at 400 and 480 m in clear air with no map, and 0.67 and 1.50 in fog. They were not examined; the all-blocked case of the rule and a threat first seen closer than a turn needs are the likely ones.

What this does not show. One scenario family (six aircraft on one orbit, generated hills), one sensor model and one avoidance rule whose values are all assumed, and hills whose steepness was chosen to bracket contact, so the table says what happens near that edge and not how often real terrain is that steep. The rule is a simple one chosen so that a reader can follow it; it is not shown to be the best way to use the sensor, and a planner was not tried. None of this was tested on hardware.

### Getting the formation back after avoiding

ADR 0015 looked at what the experiment above found to be the cost: avoidance keeps the aircraft off the ground by giving up the formation. A trace of the flown paths showed that the aircraft do return to their circle, after excursions of 500 to 770 m, and the rule had three things that made them large or needless: a tie that turned outward, a column chosen nearest the aircraft's own heading, and a judgment of what climbing can clear that was both a straight line at 0.6 of the climb limit and measured against the planning clearance of 60 m. `experiments/fixed_wing_avoid_rejoin.yaml` runs seven arms (the forward sensor alone; avoidance as built; with a rejoin heading from the guidance; with the kinematic judgment; with an avoidance clearance of 30 m; with all three; and avoidance as built with a horizon of 6 s) over the steepness levels of the experiment above, two maps (none, exact) and two weather levels: 112 cells, 30 seeds, 3,360 runs, none failed, on the same scenario. The criteria for an arm being better than avoidance as built were written in ADR 0015 before the run and are applied by `analysis/results_avoid_rejoin.py`. A sweep of the avoidance clearance (`fixed_wing_avoid_clearance.yaml`, 6 cells, 180 runs, none failed) follows. Reproduce with `swarm_batch experiments/fixed_wing_avoid_rejoin.yaml --out results/fixed_wing_avoid_rejoin_30.csv --jobs 16 --seeds 30` and `python results_avoid_rejoin.py ../results`.

Clear air, no map, means over 30 runs: late contacts of six aircraft, share of samples within 100 m of the aircraft's circle, radial error, time on station.

| Hills | Arm | Late contacts | Corridor share | Radial error | Time on station |
|---|---|---|---|---|---|
| 240 m | Forward sensor alone | 0.07 | 1.00 | 2.0 m | 0.98 |
| | Avoidance as built | 0.03 | 0.84 | 86.3 m | 0.23 |
| | All three | 0.03 | 1.00 | 5.1 m | 0.93 |
| | Horizon of 6 s | 0.03 | 0.98 | 27.6 m | 0.43 |
| 400 m | Forward sensor alone | 2.03 | 1.00 | 11.7 m | 0.30 |
| | Avoidance as built | 0.30 | 0.49 | 201.6 m | 0.07 |
| | All three | 0.37 | 0.96 | 34.2 m | 0.14 |
| | Horizon of 6 s | 0.33 | 0.89 | 66.0 m | 0.10 |
| 480 m | Forward sensor alone | 3.13 | 1.00 | 19.2 m | 0.19 |
| | Avoidance as built | 0.53 | 0.41 | 225.4 m | 0.06 |
| | All three | 0.57 | 0.93 | 50.1 m | 0.08 |
| | Horizon of 6 s | 0.53 | 0.83 | 86.1 m | 0.07 |

- **The three changes together keep the safety and nearly all of the formation.** All three against avoidance as built, paired by seed: late contacts +0.07 [-0.03, +0.16] at 400 m and +0.03 [-0.03, +0.10] at 480 m (no loss of safety that the intervals show, though a loss of up to 0.16 of six aircraft is not excluded), corridor share +0.476 [+0.442, +0.511] and +0.513 [+0.471, +0.555], radial error -167.4 m [-180.8, -154.1] and -175.2 m [-191.0, -159.5]. At 240 m the formation is back to the forward sensor's (corridor share 1.00, radial error 5.1 m against 2.0 m, time on station 0.93 against 0.98) with the same 0.03 contacts. The corridor share of the forward sensor alone, 1.00 at every level, is inflated where it loses aircraft: an aircraft that has crashed stays where it crashed and is counted in its corridor, so the share is not comparable between arms that lose different numbers of aircraft, but it is between the avoidance arms, whose contacts are alike.
- **Each change helps alone, and the clearance helps most.** At 400 m, against avoidance as built, the rejoin heading adds 0.313 [0.274, 0.352] to the corridor share (radial error -106.9 m), the kinematic judgment 0.183 (-60.1 m), the clearance of 30 m 0.389 (-136.0 m, late contacts +0.03 [-0.03, +0.10]). Their sum is more than any alone. The horizon of 6 s alone (0.406, -135.6 m) is about as good as the clearance alone and much worse than all three (0.476, -167.4 m); all three at the default horizon beats the short horizon of ADR 0014 (corridor share 0.96 against 0.89, radial error 34 m against 66 m).
- **With an exact map the false-alarm cost is gone.** Nothing reaches the ground in any arm. Time on station with all three is 1.00, 0.96, and 0.92 at 240, 400, and 480 m against 0.89, 0.75, and 0.71 for avoidance as built and 1.00, 0.96, and 0.94 for the forward sensor alone; at 240 m the radial error is the forward sensor's to the printed precision (2.27 m), because avoidance never triggers.
- **The criteria of ADR 0015.** Three arms are better than avoidance as built on every criterion: the clearance of 30 m alone, all three, and the horizon of 6 s. The rejoin heading alone and the kinematic judgment alone are better on the steep hills and fail the exact-map criterion (time on station at least 0.90 at every level; their minimum is 0.80 and 0.78), as the false alarms are mostly the clearance's.
- **Fog of 375 m visibility.** With no map at 400 and 480 m, all three has late contacts of 0.60 and 1.57 against 0.67 and 1.50 for avoidance as built (within the noise) and a corridor share of 0.89 and 0.80 against 0.78 and 0.65 (radial error 58.6 and 85.1 m against 94.0 and 144.6 m). The fog still costs safety (the forward sensor alone loses 3.40 and 4.50 of six), and the formation is still not whole in it.
- **The clearance sweep** (all three, 400 m hills, clear air). With no map, clearances of 15, 30, and 60 m give late contacts of 0.37, 0.37, and 0.30, corridor shares of 0.97, 0.96, and 0.87 and lowest heights of 5.1, 7.0, and 7.7 m: a smaller clearance keeps more of the formation at no loss of safety that 30 runs can show, and passes a little lower. With the map 60 m too low nothing reaches the ground at any of them, and time on station is 0.87, 0.79, and 0.29, so a clearance of 60 m, the planning one, keeps most of the cost of the first experiment where the aircraft have a map to follow.
- **What is left.** On the steep hills the formation is not whole: with all three the radial error is 34 and 50 m against the forward sensor's 12 and 19 m, and time on station is 0.14 and 0.08 against 0.30 and 0.19, because an aircraft that has dodged is off its place on the circle and the phase it lost (the reference is a function of time) is only recovered by a speed adjustment that its speed limits bound; this ADR did not change that. About 0.4 and 0.6 of six aircraft still reach the ground at 400 and 480 m, and 0.60 and 1.57 in fog.

What this does not show. One scenario family and one avoidance rule, as above; the clearance of 30 m and the turn time of 1.5 s are assumed; an avoidance clearance below the planning clearance lets an aircraft pass lower when it cannot climb, which the lowest heights above (5 to 8 m with no map) show and which a real aircraft's margin for its own altitude error would have to cover. The three changes were tried together because separately each was only partly enough; the experiment does not separate them further than the arms above. None of this was tested on hardware.

### Telling each other what the sensors see

ADR 0016 asked what it is worth for the aircraft to share what their forward sensors have seen, and how much of that survives a bad link. Aircraft on the same 300 m circle reach a hill 10.0 s apart; the sensor sees 25.6 s ahead in clear air and only 6.4 s in the fog of these experiments, less than a quarter turn takes (8.0 s). Terrain does not move, so part of any benefit is memory, which needs no link; the experiment has memory without communication as its own arm. `experiments/fixed_wing_share.yaml` crosses three steepness levels (hills of 40, 400, and 480 m, no map), two weather levels, three loss levels (0.05, 0.30, 0.60), two jam levels (none, and one jammer over the whole area from 20 s to 60 s at 0.97 loss, which cuts the aircraft off for 44 percent of the run) and three arms (avoidance with the three changes of ADR 0015; the same with memory; the same with sharing): 108 cells, 30 seeds, 3,240 runs, none failed. A sensitivity sweep (`fixed_wing_share_sensitivity.yaml`, 6 cells, 180 runs) and a trust experiment (`fixed_wing_share_trust.yaml`, 32 cells, 960 runs; all none failed) follow. The criteria were written in ADR 0016 before the runs and are applied by `analysis/results_share.py`. Same setup as above. Reproduce with `swarm_batch experiments/fixed_wing_share.yaml --out results/fixed_wing_share_30.csv --jobs 16 --seeds 30` (and the other two) and `python results_share.py ../results`.

Fog of 375 m visibility, no jam, loss 0.05, means over 30 runs: late contacts of six aircraft, corridor share, mean height.

| Hills | Arm | Late contacts | Corridor share | Mean height |
|---|---|---|---|---|
| 400 m | Avoidance | 0.60 | 0.89 | 114.3 m |
| | Memory | 0.60 | 0.82 | 121.4 m |
| | Sharing | 0.30 | 0.87 | 137.9 m |
| 480 m | Avoidance | 1.57 | 0.80 | 103.7 m |
| | Memory | 1.27 | 0.72 | 109.9 m |
| | Sharing | 0.67 | 0.77 | 134.9 m |

- **The link is worth something exactly where the sensor is blind.** In fog, sharing against memory, paired by seed: late contacts -0.30 [-0.52, -0.08] at 400 m and -0.60 [-0.89, -0.31] at 480 m, with the corridor share +0.057 [+0.024, +0.090] and +0.051 [+0.000, +0.101]. In clear air, where the sensor sees far enough, sharing and memory are alike (late contacts 0.33 against 0.33 at 400 m and 0.53 against 0.53 at 480 m); the link adds nothing there. On the 40 m control it changes nothing. All the criteria that were about benefit and harm are met (C1, C2, C3).
- **Memory alone is not the benefit.** Against avoidance without memory, memory has late contacts +0.00 [-0.26, +0.26] and -0.30 [-0.77, +0.17] in fog (no benefit that the intervals show) and a lower corridor share (-0.069 [-0.103, -0.036] and -0.082 [-0.117, -0.047]): a remembered point keeps turning the aircraft aside on later laps. Sharing against avoidance has late contacts -0.30 [-0.54, -0.06] and -0.90 [-1.35, -0.45] and a corridor share that does not differ beyond the intervals (-0.012 and -0.032). So what the link gives is what the aircraft ahead saw before the aircraft behind could.
- **The link costs height and a little formation.** Memory and sharing both fly higher: mean height 137.9 m with sharing against 114.3 m without it at 400 m in fog, and 134.9 m against 103.7 m at 480 m, 20 to 30 m more, because every remembered or reported point raises the altitude floor. In clear air the same is 165.8 m and 168.7 m against 142.2 m and 147.2 m. Time on station is lower with sharing (0.18 against 0.27 at 400 m in fog, 0.13 against 0.22 at 480 m).
- **Loss costs the advantage less than it might.** Sharing against memory in fog, late contacts, at loss 0.05, 0.30, and 0.60: -0.30, -0.33, and -0.23 at 400 m, and -0.60, -0.63, and -0.47 at 480 m; with 60 percent of the messages lost, three quarters of the advantage remains (0.23 of 0.30, 0.47 of 0.60). That is the redundancy of sending a point again with every report: over 2 s at 10 reports a second, a point is sent 20 times.
- **A jam costs it more.** With the aircraft cut off for 44 percent of the run, the advantage in fog falls to -0.13, -0.17, and -0.20 at 400 m (against -0.30, -0.33, and -0.23) and -0.40, -0.47, and -0.30 at 480 m (against -0.60, -0.63, and -0.47): between 43 and 87 percent of it remains across the six cells, around the share of the run that the link was up (56 percent). The intervals of single cells are wide (30 runs); the pattern is across the six.
- **What it costs the channel.** Sharing sends 32,332 messages in a 90 s run of six aircraft against 24,015 without it (400 m hills, fog, loss 0.05), 35 percent more; the delivery ratio of the pooled messages is the channel's (0.95 and 0.40 at loss 0.05 and 0.60), the same in every arm.
- **The sensitivity sweep.** Late contacts 0.33 to 0.40 and corridor share 0.88 to 0.90 across a ttl of 30, 120, and 240 s and reports of 2 and 8 points; a report of 8 points has 0.33 or 0.37 against 0.40 for 2. A ttl of 120 s and one of 240 s are the same by construction, because a run is 90 s and nothing expires.

Trust. One aircraft of six (drone 2) sends false hazards from 20 s: eight points on the circle at its own altitude in every report. Late contacts and the corridor share, the measures of the criteria written in advance, do not show it: on the 40 m hills the liar changes the corridor share by +0.000 [+0.000, +0.000] and the late contacts by +0.00, so criterion T1 (that the liar does measurable harm to undefended sharing by those measures) is not met as written. The harm is there in the other measures, which the criterion did not name. On the 40 m hills, with undefended sharing, the liar takes time on station from 1.00 to 0.43 (paired difference -0.57 [-0.57, -0.57] in clear air and -0.57 [-0.58, -0.57] in fog), adds 8.3 m [7.8, 8.8] and 10.2 m [9.7, 10.7] to the radial error, and makes the aircraft fly 22.9 m [22.0, 23.8] and 22.2 m [21.3, 23.1] higher, where there was nothing to avoid; the flown paths differ by up to about 60 m. On 400 m hills the harm to height is 18.5 m [14.3, 22.6] in clear air and 16.6 m [12.5, 20.6] in fog, and the rest is within the intervals. The corridor share does not see it because the deviations stay inside the 100 m corridor; a measure that does not saturate (time on station, radial error, mean height) is the one to read. The confirmation rule of two defends: with `min_sources: 2` the liar makes no difference to any measure to the printed precision on the 40 m hills (time on station +0.00, radial error +0.00 m, height -0.00 m) and none beyond the intervals on 400 m hills (T2 is met on every cell, by its own measures and by these). The memory arm, which never receives, is moved by a liar by 0.000000, as it must. The rule costs the benefit: in fog on 400 m hills, with no liar, late contacts are 0.47 with the rule against 0.30 without it (+0.17 [+0.03, +0.31]) and 0.60 for memory, so it keeps about 43 percent of the benefit (T3), because the aircraft behind the one that saw a hill has only one source for it and gets the warning one aircraft later.

What this does not show. One scenario (six aircraft on one circle, so every aircraft is a neighbor of every other and there is nothing to relay); one liar of six, at the worst place for its points, and not one that lies in other ways; a rule that defends against one faulty sender and not several; a store that keeps a point where it was placed, so a navigation error that grows in fog (the aircraft's own position estimate, and so the points it reports) is inherited, whose effect on these results was not separated; and bandwidth, which the channel limits in messages and not in bytes. The criteria were applied as written, and one of them, T1, turned out to be built on a measure too coarse to see the effect it was written to find; the result is reported as it came out and the finding from the other measures is stated as that. None of this was tested on hardware.

### Why aircraft still reach the ground

Three ADRs left a residual unexamined: with avoidance, its fixes, and sharing, about 0.3 to 1.6 of six aircraft still reach the ground on the steep hills. `scripts/crash_report.py` runs a scenario for seeds 1 to 30, reads the flown positions (`swarm_sim positions`) and scene (`swarm_sim export-scene`), and puts each aircraft that stops on the ground in a class from its path: `launch` (stopped at the first step, so it started on the ground), `early` (within 5 s of the start), `separation` (another aircraft that is still flying within 60 m in the last 3 s; one that has already stopped on the ground is a wreck and does not count), `climb` (climbing at 80 percent of its limit or more with the ground ahead rising faster), `turn` (turning at 70 percent of its sideways limit or more), `blind` (the impact point was outside the sensor's 20 degrees half-field 3 s before), and `other`. The classes are tests on the path and not on what the brain sensed, which the log does not hold. Run on the avoidance of ADR 0015 with all three changes (the arm `all3`), no map, hills of 400 or 480 m, 30 seeds, 180 aircraft each:

| Case | On the ground | Launch | Early | Separation | Climb | Turn | Blind | Other |
|---|---|---|---|---|---|---|---|---|
| Clear air, 400 m | 18 | 12 | 5 | 0 | 0 | 0 | 0 | 1 |
| Fog, 400 m | 25 | 12 | 6 | 0 | 0 | 0 | 0 | 7 |
| Fog, 480 m | 63 | 23 | 9 | 2 | 2 | 2 | 0 | 25 |

- **Most of the residual in clear air is not an avoidance failure.** Of the 6 aircraft that were not on the ground at launch, 5 stopped within 4 s of the start: they began close to the face of a hill with nothing yet sensed, and the distance to the impact point 3 s before was 58 to 79 m, where clearing it needed 4.8 to 5.9 times the climb the vehicle can make in the time that was left. The metric `launch_contacts` counts only aircraft on the ground in the first record, so these are in `late_contacts` although no avoidance could have saved them. One aircraft in 180 is a contact after that.
- **In fog the early ones are a quarter of the rest, and most of the remainder is `other`.** Fog, 480 m: 9 of the 40 aircraft that were not on the ground at launch stopped within 5 s; 25 are in `other`. A first version of this script counted a neighbor lying on the ground as a close aircraft and put 12 in `separation`; in 10 of those the neighbor was a wreck already stopped, so the class was an artifact. With wrecks excluded, 2 remain, and aircraft taking the same free column is not shown as a cause here.
- **The `other` aircraft were not using what they had.** Nearly all were heading straight at the ground and had time to turn: 3 s before the impact they were 68 to 145 m from it (6 s before, 152 to 297 m), the impact point was within 11 degrees of the heading, and clearing it needed 3.0 to 5.0 times the climb that was possible, so climbing could not save them. Their sideways acceleration over the last 3 s was 1.6 to 4.5 m/s^2, against the 6.2 m/s^2 that the rule may use (0.9 of the vehicle's 6.85): they were turning at a half or less of the bank they had. That fits the rule: the turn command is speed x angle / 1.5 s toward the nearest free column, so a free column 10 degrees off gives about 3.6 m/s^2 at 31 m/s whatever the time left, and the next column may be blocked as well. Only 2 aircraft in the fog-480 case were at their sideways limit and 2 at their climb limit. Their speed over the last 3 s was 23 to 49 m/s, mean 36 m/s over the 31 late crashes of this case against 31.3 m/s on the mission, and 77 percent were above 31.3: why the speed is above the mission speed was not examined, and it shortens the time to the threat and widens the turn (the radius at 36 m/s and 6.2 m/s^2 is 209 m, at 31.3 m/s 158 m).
- **So the next change is the use of the turn, not more sensing or more climb.** When the time to the threat is short, the turn should use all the bank there is, not an amount proportional to the angle. The speed on the way down is a second target. Nothing was tried here; this section is a diagnosis.

What this does not show. Three cases of one scenario family with the avoidance of ADR 0015; classes that are tests on the flown path with thresholds chosen here (60 m, 5 s, 80 and 70 percent), so a different threshold moves aircraft between classes; and no view into what the brain sensed or decided at the time. Reproduce with `python scripts/crash_report.py build/release/apps/swarm_sim.exe SCENARIO.yaml --seeds 30` for a scenario file with the `all3` settings (`sensors.forward`, and `control.avoid` with `rejoin`, `feasibility: kinematic`, and `clearance_m: 30`).



### The whole turn when time is short

ADR 0017 asked whether, when the time to a threat is short, turning with the whole bank and slowing keeps more aircraft off the ground. `experiments/fixed_wing_avoid_urgent.yaml` crosses weather (clear, fog of 375 m visibility), hills of 40, 400, and 480 m, a map (none, exact), and four arms (`all3`, the avoidance of ADR 0015, and the same with `urgent_s` of 4, 6, and 8 s): 48 cells, 30 seeds, 1,440 runs, none failed. The criteria were written in the ADR before the run and apply to `urgent6` against `all3` as paired differences with 95 percent t intervals. Reproduce with `swarm_batch experiments/fixed_wing_avoid_urgent.yaml --out results/fixed_wing_avoid_urgent_30.csv --jobs 16 --seeds 30` and `python results_avoid_urgent.py ../results`.

- **U1 is not met.** In fog with no map, late contacts of six aircraft fall from 0.60 to 0.57 on 400 m hills (difference -0.03 [-0.35, +0.28]) and from 1.57 to 1.43 on 480 m hills (-0.13 [-0.45, +0.19]). The intervals include zero, so the experiment does not show that the whole bank keeps aircraft off the ground. The sweep gives the same: `urgent4` is +0.07 and -0.27 and `urgent8` is -0.03 and -0.10, none with an interval below zero.
- **U2, U3, and U4 are met.** On the 40 m control and with the exact map the urgent arms are identical to `all3` in every one of those cells but one (the 480 m clear cell with the exact map differs in the third decimal place of the radial error), with no late contacts. In clear air with no map the late contacts are not higher (-0.07 [-0.16, +0.03] and -0.03 [-0.10, +0.03]). No cell has an increase of collision episodes whose interval is above zero, but the mean rises from 0.00 to 0.07 in clear air on 480 m hills and falls from 0.10 to 0.07 in fog on 480 m hills, and the closest approach falls there (88 to 76 m in clear air).
- **The formation is held somewhat better, which U1 did not ask about.** The share of samples within 100 m of the circle is higher by 0.039 [0.016, 0.062] on 400 m hills and 0.056 [0.027, 0.085] on 480 m hills in fog, and 0.010 [0.005, 0.014] on 400 m hills in clear air, and the radial error is lower (59 to 46 m and 85 to 66 m in fog). This was not a criterion, was seen after the run, and is reported as an observation: harder, shorter turns make shorter detours.
- **The cause named in the diagnosis is not supported.** The crash classes of `scripts/crash_report.py` on `urgent6` in fog on 480 m hills (30 seeds, 180 aircraft): 59 aircraft stopped on the ground against 63 with `all3` (23 at launch, 9 early, 0 separation, 0 climb, 1 turn, 0 blind, 26 other, against 25 other). The sideways acceleration of the `other` aircraft over their last 3 s is still about 1.5 m/s^2 on average, as before. So these aircraft are not turning hard in the last seconds even with the rule that makes them. Either the rule does not fire for them (the sensor does not see the hill in time in fog, the column nearest the heading is not judged blocked, or the return is judged clearable), or it fires too late. The log does not hold what the brain sensed, so this is not decided here.

What this does not show. One scenario family, one value of `urgent_s` in the criteria (6 s), and crash classes that are tests on the flown path. The time to the threat the rule uses is the least over the points of the column nearest the heading, so it is a time to the first return that is unclearable.

### The turn keeps its share of the acceleration limit

ADR 0018 found why the whole bank of ADR 0017 changed nothing: while slowing, the braking command and the turn together went over the brain's acceleration limit (10 m/s^2), `combine_accel` scaled the whole command, and the achieved turn was about half the commanded one (in one traced flight: braking -17, turn -6.17, climb +2, a norm of 18.5, a scale of 0.54, and an achieved turn of about 3.3 m/s^2). `control.avoid.turn_first` bounds the braking by what the limit leaves after the turn and the climb. `experiments/fixed_wing_avoid_turnfirst.yaml` has the cells of the previous experiment with three arms (`all3`, `turnfirst`, `turnfirst_urgent6`): 36 cells, 30 seeds, 1,080 runs, none failed. The criteria were written in the ADR before the run and apply to `turnfirst` against `all3`. Reproduce with `swarm_batch experiments/fixed_wing_avoid_turnfirst.yaml --out results/fixed_wing_avoid_turnfirst_30.csv --jobs 16 --seeds 30` and `python results_avoid_turnfirst.py ../results`.

- **V1 is met on 480 m hills and not on 400 m hills, so it is not met as written.** In fog with no map the late contacts of six aircraft fall from 1.57 to 1.10 on 480 m hills (-0.47 [-0.83, -0.10]) and from 0.60 to 0.43 on 400 m hills (-0.17 [-0.34, +0.01]); the upper end of the second interval is just above zero.
- **V2, V3, and V4 are met.** No late contacts on the 40 m control or with the exact map, where `turnfirst` is identical to `all3`. In clear air with no map the late contacts are not higher (-0.07 [-0.16, +0.03] and -0.03 [-0.10, +0.03]). No cell has an increase of collision episodes whose interval is above zero; in fog on 480 m hills they fall from 0.10 to 0.00.
- **The cost is none that the measures show.** The corridor share changes by -0.002 [-0.008, +0.005] and +0.003 [-0.013, +0.019] in fog; the radial error is 59 and 87 m against 59 and 85 m.
- **The crash classes show where the gain comes from.** `scripts/crash_report.py` on fog, 480 m hills, 30 seeds, 180 aircraft: 49 on the ground against 63 (`launch` 23, `early` 9, `separation` 2, `climb` 1, `turn` 8, `other` 6; with `all3`: 23, 9, 2, 2, 2, 25). The `other` class, the aircraft that were heading at the ground with sideways acceleration well under the limit, falls from 25 to 6, and the 6 that remain have an average sideways acceleration of 2.6 m/s^2 in their last 3 s. The `turn` class rises from 2 to 8: those aircraft now turn at 70 percent or more of their limit and still hit the ground. They are the next limit, and the traced flight says why it is likely: the threat is first seen about 3.4 s ahead at 39 m/s, a turn radius of 247 m at 6.17 m/s^2. This is a reading of one trace and of the flown paths, not a test of it.
- **With `urgent_s` of 6 s on top (reported, not a criterion),** `turnfirst_urgent6` against `turnfirst`: late contacts -0.27 [-0.48, -0.05] in fog on 480 m hills and -0.07 [-0.20, +0.07] on 400 m hills, corridor share +0.049 [+0.024, +0.075] and +0.053 [+0.030, +0.077], and radial error 87 to 65 m on 480 m hills. The whole bank adds something once the turn is no longer scaled down; before, it added nothing in late contacts (ADR 0017). In clear air on 480 m hills the closest approach falls from 88 to 78 m (collision episodes 0.03 for both arms).

What this does not show. One scenario family; the criterion V1 as written needed both steepness levels; the earlier results (ADRs 0014 to 0017) were measured with the scaled-down turn and are not repeated with `turn_first`, so their numbers understate what the rule can do. The traced flight is one flight.

### Limiting the catch-up speed after a detour

ADR 0019 asked whether the speed that the orbit's phase correction asks for after a detour (up to 15 m/s more per radian of lost phase) is what shortens the warning, and whether a cap on it keeps more aircraft off the ground. A probe (`experiments/probe_fixed_wing_phase.yaml`, 10 seeds, fog, no map) varied only the phase gain and gave late contacts on 480 m hills of 1.00, 0.60, and 0.50 at gains of 0.05, 0.02, and 0. `experiments/fixed_wing_avoid_catchup.yaml` crosses weather (clear, fog), hills of 40, 400, and 480 m, a map (none, exact), and four arms (`base`, the avoidance with `turn_first`; and `cap6`, `cap3`, and `cap0`, the same with `catchup_cap_mps` of 6, 3, and 0 m/s): 48 cells, 30 seeds, 1,440 runs, none failed. The criteria were written in the ADR before the run and apply to `cap3` against `base`. Reproduce with `swarm_batch experiments/fixed_wing_avoid_catchup.yaml --out results/fixed_wing_avoid_catchup_30.csv --jobs 16 --seeds 30` and `python results_avoid_catchup.py ../results`.

- **X1 is met on 480 m hills and not on 400 m hills, so it is not met as written.** In fog with no map the late contacts of six aircraft fall from 1.10 to 0.63 on 480 m hills (-0.47 [-0.72, -0.21]) and from 0.43 to 0.37 on 400 m hills (-0.07 [-0.16, +0.03]). The sweep on 480 m hills: `cap6` -0.17 [-0.34, +0.01], `cap0` -0.50 [-0.75, -0.25]; on 400 m hills `cap0` is -0.10 [-0.21, +0.01].
- **X2, X3, and X4 are met.** No late contacts on the 40 m control or with the exact map, where the arms are identical. In clear air with no map the late contacts are the same for all arms (0.30 and 0.53). No cell has an increase of collision episodes whose interval is above zero.
- **The cost is in the closest approach and, at a cap of 0, in time on station.** The mean closest approach of `cap3` against `base`: 135 against 145 m in clear air on 400 m hills, 85 against 88 m on 480 m hills, 86 against 99 m in fog on 400 m hills, and 56 against 55 m in fog on 480 m hills; at `cap0` it is 125, 75, 80, and 55 m. Time on station in fog on 400 m hills is 0.27, 0.26, and 0.23 for `base`, `cap3`, and `cap0`. The corridor share is the same or higher (+0.008 [0.000, 0.015] at `cap3` and +0.036 [0.020, 0.052] at `cap0` in fog on 400 m hills). This is what the ADR expected: aircraft stay bunched on the circle for longer.
- **The crash classes and speeds, as the ADR said they would be reported.** `scripts/crash_report.py` on fog, 480 m hills, 30 seeds, 180 aircraft: `base` has 49 aircraft on the ground (23 at launch, 9 early, 2 separation, 1 climb, 8 turn, 6 other), `cap3` has 35 (23 at launch, 9 early, 0 separation, 0 climb, 1 turn, 2 other). So 32 of the 35 are launch and early cases that no avoidance acts on, and 3 are late. The mean speed over the last 3 s of the late ones is 35.9 m/s for `base` (17 aircraft; 36.8 for the 8 in `turn`) and 23.5 m/s for `cap3` (3 aircraft). This agrees with the reading that the speed above the mission speed was shortening the warning; it is not a separate test of it. The counts of this script (aircraft stopped) and the metric `late_contacts` (ground contacts) are different measures, so 17 to 3 here is not the 1.10 to 0.63 per six aircraft above.

What this does not show. One scenario family and one cap in the criteria (3 m/s); a cap leaves the aircraft bunched longer and costs 3 to 13 m of closest approach in the cells above, on separations well above the 30 m margin, but the scenario has six aircraft on one circle and nothing is said about denser formations. The probe is 10 seeds. The aircraft that stop within seconds of the start (32 of the 35 here) are not addressed by this or any change since ADR 0014.

### Starting above the ground

ADR 0020 asked whether starting above the true ground removes the launch and early losses of the arms that fly without a map, and whether the aircraft that then fly survive the hills. Without a map an aircraft started on its orbit slot at the nominal altitude, whatever the terrain under it. For the 180 aircraft of fog on 480 m hills in the previous section, the start height above the true ground against what happened: 8 started at or below the ground (all 8 stopped at the first step), 22 within 30 m (13 stopped at the first step), 22 within 30 to 60 m (2 at the first step, 3 within 5 s), and 128 above 60 m (none at the first step, 6 within 5 s). `swarm.start.clearance_m` places a fixed-wing aircraft at least that far above the highest surface at its start point; the builder reads the scene, not the map. `experiments/fixed_wing_avoid_start.yaml` crosses weather (clear, fog), hills of 40, 400, and 480 m, a map (none, exact), and two start rules (`free`, as before; `cleared`, 60 m), with the best avoidance so far in both (`turn_first`, a catch-up cap of 3 m/s): 24 cells, 30 seeds, 720 runs, none failed. The criteria were written in the ADR before the run. Reproduce with `swarm_batch experiments/fixed_wing_avoid_start.yaml --out results/fixed_wing_avoid_start_30.csv --jobs 16 --seeds 30` and `python results_avoid_start.py ../results`.

- **Z1 to Z4 are met.** Launch contacts are 0 for `cleared` in every run. Total ground contacts of six aircraft fall, with no map, by -0.27 [-0.43, -0.10] (400 m hills) and -0.57 [-0.86, -0.28] (480 m) in fog, and by -0.27 [-0.43, -0.10] and -0.63 [-0.94, -0.33] in clear air; in fog on 480 m hills from 1.17 to 0.60. On the 40 m control and with the exact map there are no ground contacts and the arms are the same.
- **The aircraft that now fly do not crash more.** Late contacts are -0.03 [-0.24, +0.17] in fog on 480 m hills, -0.03 [-0.15, +0.09] on 400 m hills, -0.10 [-0.28, +0.08] and -0.03 [-0.15, +0.09] in clear air. This was the thing Z5 was there to show: that removing the launch losses did not just move them later.
- **The costs are none that the measures show, and the closest approach rises.** The corridor share differs by -0.000 to +0.008, the radial error is the same within a few meters, and the mean height rises by 6 to 20 m, because the aircraft start higher. The mean closest approach rises from 56 to 75 m in fog on 480 m hills and from 135 to 161 m in clear air on 400 m hills.
- **The crash classes, as the ADR said they would be reported.** `scripts/crash_report.py` on fog, 480 m hills, 30 seeds, 180 aircraft with `cleared`: 18 on the ground against 35 (1 launch, 11 early, 0 separation, 0 climb, 2 turn, 0 blind, 4 other; with the free start: 23, 9, 0, 0, 1, 0, 2). Of the 11 early ones, 8 stopped within 2.4 s of the start; the impact point was 17 to 79 m ahead 3 s before (or at the start), and clearing it needed 4.7 to 19 times the climb the vehicle can make in the time. They started at least 60 m above the ground under them and next to a hill face on their circle, and no avoidance acts on a face 17 to 66 m ahead at 31 m/s (0.5 to 2 s). The 4 `other` and 2 `turn` are the late ones.

What this does not show. The clearance of 60 m is one level, fixed from the planning clearance; a different level would move aircraft between the first-step cases and the early ones. The early cases are a property of where the orbit passes in these generated hills, and a scenario with a launch site chosen away from the faces would not have them. One scenario family.

### The whole ladder on one setup

The avoidance experiments above were each run with the options of their time and with the aircraft starting on their slots at the nominal altitude, so their numbers are not comparable to each other. `experiments/fixed_wing_final.yaml` runs every stage on one setup, all with a cleared start (`swarm.start.clearance_m` 60, ADR 0020): `forward` (the forward sensor and its altitude floor only, ADR 0013), `avoid` (turning and slowing, ADR 0014), `all3` (rejoin, kinematic feasibility, clearance 30 m, ADR 0015), `turnfirst` (ADR 0018), `cap3` (a catch-up cap of 3 m/s, ADR 0019), `urgent6` (the whole bank under 6 s, ADR 0017), and `share` (the aircraft also share what they see, ADR 0016). Weather (clear, fog of 375 m visibility), hills of 40, 400, and 480 m, a map (none, exact): 84 cells, 30 seeds, 2,520 runs, none failed. It describes; it tests no new claim. Reproduce with `swarm_batch experiments/fixed_wing_final.yaml --out results/fixed_wing_final_30.csv --jobs 16 --seeds 30` and `python results_final.py ../results`.

Ground contacts of six aircraft over the run (means over 30 seeds), steep hills with no map:

| Arm | Clear, 400 m | Clear, 480 m | Fog, 400 m | Fog, 480 m |
|---|---|---|---|---|
| forward | 1.87 | 3.13 | 3.47 | 5.03 |
| avoid | 0.27 | 0.40 | 0.73 | 1.70 |
| all3 | 0.33 | 0.43 | 0.70 | 1.57 |
| turnfirst | 0.27 | 0.40 | 0.43 | 0.93 |
| cap3 | 0.27 | 0.43 | 0.33 | 0.60 |
| urgent6 | 0.27 | 0.37 | 0.30 | 0.57 |
| share | 0.27 | 0.37 | 0.27 | 0.40 |

- **Most of the gain is the first step.** Turning and slowing against the sensor with an altitude floor alone: -1.60 [-2.28, -0.92] (clear, 400 m), -2.73 [-3.45, -2.02] (clear, 480 m), -2.73 [-3.63, -1.84] (fog, 400 m), and -3.33 [-3.96, -2.70] (fog, 480 m). The whole stack against the sensor alone, in fog on 480 m hills: -4.63 [-5.11, -4.16], from 5.03 to 0.40.
- **In clear air nothing after the first step moves the contacts that the intervals show** (every step from `all3` on is within about 0.1 of zero, with intervals that include it). In fog the later steps do: `turnfirst` against `all3` -0.63 [-0.98, -0.29] on 480 m hills (-0.27 [-0.54, +0.01] on 400 m), `cap3` against `turnfirst` -0.33 [-0.58, -0.09] (-0.10 [-0.21, +0.01]), and `urgent6` and `share` each smaller than their intervals (`share` against `urgent6` -0.17 [-0.36, +0.03] on 480 m hills).
- **The rejoin step is about the formation, not the contacts.** `all3` against `avoid`: the share of time within 100 m of the circle rises by +0.45 [+0.42, +0.49] (clear, 400 m), +0.48 [+0.44, +0.53] (clear, 480 m), +0.11 [+0.07, +0.15] (fog, 400 m), and +0.14 [+0.10, +0.18] (fog, 480 m), with contacts unchanged.
- **The formation is still not the benchmark's.** The radial error to the circle is 31 to 67 m with `share` against 9 to 18 m with the sensor alone; time on station is 0.06 to 0.18 against 0.14 to 0.75; and the mean height is higher by 56 m (clear, 400 m) to 101 m (fog, 480 m) than with the sensor alone. With an exact map every arm has no contacts, and `avoid` alone costs time on station (0.75 against 0.96 on 400 m hills), which the later stages give back.
- **The costs in closest approach.** In fog on 480 m hills the closest approach of the aircraft is 7.8 m with the sensor alone and 57 to 84 m with the stages (82 m for `share`), and collision episodes fall from 1.57 to 0.00 per run.

What this does not show. One scenario family and one aircraft parameter set, six aircraft on one circle, a cleared start of 60 m in every arm (a different start moves the early losses), and no test of whether the stages interact; each step was judged against its own criteria in its own ADR. The aircraft that still stop on the ground in fog on 480 m hills (0.40 of six) are mostly early cases next to a hill face (previous section).
