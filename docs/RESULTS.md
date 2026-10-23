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
- So most of the pooled prediction effect is a property of how the formation consensus treats a silent neighbor, not of tracking healthy neighbors better. A rule that drops a neighbor that has not been heard from within the staleness time would probably remove most of the gap between hold-last and the other predictors. That rule is not in the controller and has not been tested.
- The minimum separation does not depend on the failure level (it is the same in both halves), so its gain from prediction (0.55 m under independent loss and 0.96 m under bursty loss) is a separate effect.

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
- The hold-last baseline as implemented keeps a silent neighbor at constant certainty, so the comparison with it is partly a comparison of how a silent neighbor is treated. It is not a measurement of how well a predictor tracks healthy neighbors, which is the 0.30 and 0.60 m of the no-failure rows.
- The fallback result is for one kind of failure (a lost LiDAR position fix in fog) and one degraded-communication state in the other scenarios. The state machine was not tested against other kinds of navigation failure.

## The fixed-wing aircraft

There are no benchmark results for the fixed-wing aircraft, only three pilots, described in [phases/flight-realism.md](phases/flight-realism.md). They found that fog (blinding the camera position fix) is the only effect that reaches the flight, that packet loss and the neighbor predictor have no measurable effect on it, and that the radar altimeter did not reduce the worst dip towards the ground when the terrain map was present. The research question is a question about the quadrotor swarm for that reason.
