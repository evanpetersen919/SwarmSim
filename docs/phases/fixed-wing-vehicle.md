# Fixed-wing vehicle

Decision: [0010](../decisions/0010-fixed-wing-vehicle.md). Models: [MODELS.md](../MODELS.md), the sections on the fixed-wing vehicle, the camera position fix, orbit guidance, the turning neighbor predictor, and orbit metrics.

## What was built

A second vehicle type behind a vehicle interface, with the quadrotor as the first. A fixed-wing aircraft of an ALTIUS-class parameter set flies a coordinated-turn point mass that cannot hover, fixes its position with a downward camera instead of a LiDAR, and flies a circle around a shared center with its phase set by a shared clock blended with a consensus over its neighbors. Separation uses a closest-approach rule. A turning neighbor predictor was added for aircraft on circles. Metrics for orbits were added. The scenario format went to version 2 for the vehicle, camera, and orbit sections; a scenario that uses none of them is written as version 1, so every earlier log still verifies.

Two changes were made after a first probe, because the probe showed the first design could not answer the project's question:

- In the first design the phase of each aircraft came from the shared clock alone, so nothing an aircraft heard changed where it flew. All four neighbor predictors, jamming, and packet loss gave the same results. A phase consensus over the neighbors, blended with the clock by a weight, was added. A weight of 0 is the first design.
- In the first design a failed aircraft was frozen in the air, as a failed quad is. The others flew through it, and one failure cut the minimum separation from about 109 m to 15 m, or 0.5 m without altitude layers. A failed fixed-wing aircraft now flies on straight, silent, at its last speed.

## How to reproduce

The experiment files are in `experiments/`: `base_fixed_wing.yaml` is the scenario they vary, `matrix_fixed_wing.yaml` has 216 cells, `baselines_fixed_wing.yaml` 72, and `ablation_fixed_wing.yaml` 48. `probe_fixed_wing.yaml` is the probe described above, with 384 cells. Five seeds per cell.

```
build/release/apps/swarm_batch experiments/matrix_fixed_wing.yaml --out results/matrix_fixed_wing.csv --jobs 28
build/release/apps/swarm_batch experiments/baselines_fixed_wing.yaml --out results/baselines_fixed_wing.csv --jobs 28
build/release/apps/swarm_batch experiments/ablation_fixed_wing.yaml --out results/ablation_fixed_wing.csv --jobs 28
python -m swarmsim_analysis summarize ../results/baselines_fixed_wing.csv --by consensus,predictor,jamming --metrics orbit_phase_rmse_m,min_separation_m
```

The three tables hold 1,080, 360, and 240 runs. On a 32-thread machine with the release build they took about 34, 5, and 4 seconds. The table does not depend on the job count. Every tenth run was re-executed and its log compared, and all of those matched. The numbers below are means over those runs with Student t 95 percent intervals, from the command above. They are for the simulator, with the parameters in MODELS.md, most of which are assumptions.

## Results

### The default controller (the 216-cell matrix)

The default controller is Kalman prediction, a consensus weight of 0.5, and altitude layers 10 m apart.

| Weather | Fraction of time on station | Orbit phase error (m) | Minimum separation (m) | Near misses per run (under 10 m) |
|---|---|---|---|---|
| clear | 1.00 | 2.1 | 92.9 | 0.00 |
| haze (1,000 m) | 1.00 | 2.3 | 92.9 | 0.00 |
| fog (350 m) | 0.76 | 40.0 | 51.5 | 0.16 |
| blind (200 m) | 0.42 | 73.0 | 24.1 | 0.21 |

Weather is the dominant effect. In fog the camera fix is lost above about 165 m, and the upper altitude layers lose it; in 200 m visibility every aircraft loses it, and dead reckoning drifts.

Jamming and packet loss have no measurable effect on the flight of the default controller. The orbit phase error is 29.37, 29.35, and 29.34 m for constant jamming, no jamming, and pulsed jamming, and 29.33, 29.33, and 29.39 m for loss of 0, 0.3, and 0.5 (pooled over everything else, so dominated by the foggy runs). The delivery ratio does fall, from 0.73 with no jamming to 0.56 pulsed and 0.38 constant. The default controller is robust to the loss of communication, because the clock anchors the phase and a stale neighbor is given a low weight.

Minimum separation falls with swarm size, 108 m for six aircraft, 56 m for ten, and 31 m for sixteen, as the arc between neighbors on a circle shrinks. One failed aircraft does not reduce the minimum separation (66.1 m against 64.6 m with none), because it flies away from the swarm; the longest recovery time rises from 13.5 to 25.7 s.

### Where the predictors separate (the 72-cell baselines)

With the clock alone (weight 0) every predictor flies identically, an orbit phase error of about 1.9 m (hold-last 2.0 m). The predictors only matter when the aircraft depend on what they hear. The table is the orbit phase error in meters under 60 seconds of constant jamming, with each row pooling five seeds and two loss levels (10 runs); the interval is half-width.

| Predictor | Weight 0.5 | Weight 1.0 (neighbors only) |
|---|---|---|
| turning | 1.87 +- 0.18 | 1.95 +- 0.21 |
| Kalman | 1.95 +- 0.21 | 3.31 +- 0.94 |
| dead reckoning | 2.95 +- 0.78 | 8.62 +- 3.94 |
| hold-last | 23.00 +- 5.29 | 85.21 +- 18.11 |

With no jamming the neighbors-only differences mostly disappear (turning 2.0, Kalman 2.0, dead reckoning 2.0, hold-last 7.9 m); under pulsed jamming hold-last gives 16.2 m and the others about 2.0 m. The order turning, Kalman, dead reckoning, hold-last is the order of how well each predicts an aircraft on a circle. The mechanism is visible in the weights: the first three let a neighbor's uncertainty grow with the age of its last report, so a silent neighbor is given a low weight and the aircraft fall back on the clock; hold-last keeps trusting a stale report. The mean minimum separation was 109 to 110 m for every predictor, and no aircraft came near another.

This answers the project's research question for the fixed-wing swarm in these conditions: prediction that tracks the neighbor's motion, and uncertainty that grows with age, protect the formation when it depends on communication, and hold-last does not. It does not show a gain in collision safety, because there were no near collisions to prevent.

### The ablations (48 cells)

There were no near misses in any ablation cell. Removing the separation rule changed the minimum separation by an amount inside the intervals (81.5 m against 81.5 m with flat altitudes, 63.8 against 62.9 m with layers, intervals about 14 m): the rule did not engage in these scenarios. Removing the fallback state machine did not hurt. In 200 m visibility the fraction of time on station was 0.52 +- 0.04 without it and 0.47 +- 0.04 with it, and in fog 0.91 against 0.90. Replacing the prediction with hold-last made no measurable difference without jamming.

The altitude layers have a cost in fog: the minimum separation is 52.3 +- 7.3 m with layers and 109.4 +- 0.7 m without, because the upper layers are higher than the camera can see through fog. In clear air the layers make no difference (110.0 and 109.6 m).

## What is not shown

- The separation rule has not been shown to prevent anything. In every scenario here the swarm was either far apart or blind, and when it was blind the aircraft do not know they are close: each judges closeness from reported positions, which have all drifted, and nothing in the model senses a neighbor directly.
- The fallback state machine has not been shown to help the fixed-wing swarm. Its safe hold is a circle through the aircraft's position, and it may slightly hurt under blindness.
- The results depend on assumptions in MODELS.md: the airframe limits, the camera model and its thresholds, the orbit radii and layer spacing, the consensus weights, the failure model, the jammer geometry, and the near-miss radius of 10 m. None was validated against a real aircraft or sensor. The quad matrix of 432 cells has not been run.
- Five seeds give narrow intervals here because runs within a cell differ only in noise; the planned final run uses 30. Results are for the release build of this repository; they have not been compared with other compilers.
- Real launch, joining an orbit, wind, and aircraft that leave the orbit were not studied.
