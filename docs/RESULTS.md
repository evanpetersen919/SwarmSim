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

What these results do not show. They are for one scenario family (12 aircraft, one orbit, hills), one sensor model whose parameters are all assumed, and map errors of three kinds and a few sizes that a scenario author chose; the sizes were chosen to bracket contact, so the table says what happens near that edge and not how often real maps are wrong by that much. The forward sensor reads the scene's true surface and only raises the altitude command, so ground that rises faster than the aircraft can climb is still reached; turning or slowing down is not built. The aircraft is a model, and none of this was tested on hardware.
