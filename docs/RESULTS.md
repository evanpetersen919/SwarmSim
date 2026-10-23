# Results: the quadrotor swarm

These results are for the simulator, with the parameter sets in [MODELS.md](MODELS.md). They are for the quadrotor swarm of the original research question. The fixed-wing aircraft has only pilots so far (see the end of this page).

Research question: under bursty packet loss and weather-degraded sensing, how much does neighbor prediction with a fallback state machine preserve formation integrity compared with holding the last reported state?

## What was run

| Experiment file | Cells | Seeds | Runs | What it varies |
|---|---|---|---|---|
| `experiments/baselines.yaml` | 48 | 30 | 1,440 | Neighbor predictor (hold-last, dead reckoning, Kalman, oracle), independent loss (10, 30, 50 percent), jamming, a drone failure |
| `experiments/baselines_burst.yaml` | 48 | 30 | 1,440 | The same, with bursty loss (mean burst of 6 messages) at the same average rates |
| `experiments/ablation.yaml` | 48 | 30 | 1,440 | The full controller and three variants that each disable one component (prediction, fallback, separation) |
| `experiments/matrix.yaml` | 432 | 30 | 12,960 | Loss, weather, jamming, swarm size, a drone failure, and the scene, with the default controller |

Every run succeeded (none failed). Every 200th run was re-executed from its log (8 runs of each 1,440-run experiment and 65 of the matrix) and all matched. A smaller run of `matrix.yaml` with 5 seeds (2,160 runs) was done first and is not used here.

Setup: Windows, MSVC, the release preset, `swarm_batch` 0.1.0 with 16 jobs, on an Intel Core i9-13900K with 32 GB of memory. Seeds 1 to 30. The simulation code (`core/`, `apps/`, `net/`) is the code of commit `a472c9e`, identical to `c9decf8`. The tables do not depend on the job count. Results from another compiler or platform may differ, because floating-point behavior is not guaranteed across them (see [DETERMINISM.md](DETERMINISM.md)).

To regenerate them:

```sh
cmake --preset release
cmake --build --preset release --target swarm_batch
for e in baselines baselines_burst ablation matrix; do
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

- Prediction reduces the settled formation error compared with holding the last state, by about 2.1 m under independent loss and 2.4 m under bursty loss, from about 3.6 and 4.0 m.
- The hold-last predictor gets worse as the loss rises, and faster under bursty loss. Its formation error goes from 3.62 to 4.32 m and its minimum separation from 3.49 to 2.63 m between 10 and 50 percent bursty loss. The Kalman predictor changes little over the same range (formation error 1.45 to 1.66 m, minimum separation 4.03 to 4.18 m).
- Dead reckoning is as good as the Kalman filter on the formation error (a paired difference of 0.00 m for independent loss, and 0.004 m [-0.004, 0.012] for bursty loss), and its minimum separation is slightly higher (by 0.16 m for independent loss and 0.06 m for bursty loss). The Kalman filter is not shown to be better than the simpler predictor.
- The oracle, which reads the neighbors' true state, is not an upper bound here: its formation error (3.52 m) is the level of hold-last, not of the Kalman filter. The reason has not been investigated.

## Ablation

| Variant | Formation error | Minimum separation | Collision episodes per run |
|---|---|---|---|
| Full controller | 1.47 m [1.41, 1.53] | 4.01 m [3.93, 4.08] | 0.00 |
| No prediction (hold-last) | 3.61 m [3.42, 3.81] | 3.46 m [3.37, 3.55] | 0.01 [0.00, 0.02] |
| No fallback | 1.44 m [1.39, 1.49] | 4.01 m [3.93, 4.08] | 0.00 |
| No separation | 1.55 m [1.50, 1.60] | 1.01 m [0.94, 1.07] | 0.58 [0.52, 0.64] |

Paired differences against the full controller (360 pairs): removing prediction raises the formation error by 2.14 m [1.95, 2.34]; removing separation lowers the minimum separation by 3.00 m [2.91, 3.09] and adds 0.58 collision episodes per run [0.52, 0.64]; removing the fallback state machine changes the formation error by -0.03 m [-0.04, -0.02] and the minimum separation by 0.00 m.

- Prediction and the separation rule each do what they are for.
- The fallback state machine is not shown to help in these scenarios. Without it the formation error is very slightly lower, and the minimum separation is the same.

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
- The rain level of the matrix (8 mm/h) changes nothing: its results equal clear air in every digit. Why has not been investigated; the rain constants of the weather model are placeholders (see [MODELS.md](MODELS.md)).
- Collision episodes average below 0.005 per run in every row of the matrix. The ablation scenario is the one that shows the separation rule.

## What these results do not show

- They come from one scene family, one controller design, and one simulator. The weather, loss, and jamming models are parametric and are not validated against real data, apart from one published visibility curve.
- The intervals are for the mean over runs, treating runs as independent. The pooled groups mix scenarios (jamming and failure levels); the per-cell numbers are in the result tables.
- The oracle result is unexplained (above), so the hold-last to oracle range is not a bound on what a predictor can do.
- The fallback result says that the state machine did not help these scenarios. Whether these scenarios often put a drone into a fallback state was not measured, so it is not a result about the state machine in general.

## The fixed-wing aircraft

There are no benchmark results for the fixed-wing aircraft, only three pilots, described in [phases/flight-realism.md](phases/flight-realism.md). They found that fog (blinding the camera position fix) is the only effect that reaches the flight, that packet loss and the neighbor predictor have no measurable effect on it, and that the radar altimeter did not reduce the worst dip towards the ground when the terrain map was present. The research question is a question about the quadrotor swarm for that reason.
