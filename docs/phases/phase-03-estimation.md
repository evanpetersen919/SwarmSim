# Phase 3: Estimation

Status: planned. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md), [MODELS.md](../MODELS.md), [phase 2](phase-02-vehicles-and-sensing.md).

## Goal

Each drone estimates its own pose from its IMU and a LiDAR-derived position fix, and keeps an estimate of every neighbor that survives gaps in communication. Three neighbor predictors (hold last state, dead reckoning, Kalman) share one interface, so the research question (does Kalman prediction beat hold-last-state?) can be answered later with the same code paths.

No formation control, fallback logic, or drone agent in this phase. Estimators are built and tested standalone, then proven together on the lockstep loop in the last commit.

## Non-goals

Heading (yaw) estimation, attitude, scan matching or any real localization from point clouds, map building, covariance intersection and other fusion of correlated estimates, consistency statistics such as NEES, and anything that reads ground truth.

## What the "position fix" is, stated plainly

The project does not implement scan-matching localization. The LiDAR contributes a **simulated position fix** whose availability and accuracy depend on what the real scan saw:

- The fix sensor lives on the sensor side of the boundary, like the IMU, so it may use ground truth to generate a measurement: `measured position = true position + noise`.
- The noise level and whether a fix exists at all are derived from the scan. A scan with few valid returns (fog, rain, open sky) gives a poor fix or none; variance scales inversely with the number of valid returns.
- The estimator only ever sees the resulting `PositionFix` (a position and a standard deviation). It never sees the scan or ground truth.

This couples weather and terrain to estimation quality without claiming localization performance the project does not have. The README's limitations section already says this; `MODELS.md` will say it in the fix model's section.

## Decisions

These are the choices a production team would have to make explicitly. Each is recorded so it can be revisited.

### Eigen, fixed-size only, pinned

Filters use Eigen 3.4.0 fixed-size vectors and matrices, fetched at configure time at a pinned version.

- Why Eigen: it is the standard in robotics, its linear algebra is reviewed far beyond anything a solo project could match, and using it keeps the filter code readable.
- Why fixed-size only: state dimensions are small and known at compile time (3, 6, 9), so there is no heap allocation in the filters and loops unroll.
- Rejected alternative: a hand-written small matrix class. It would remove the dependency but add numerical code to test and maintain, for no benefit.
- Isolation: Eigen is used only in `core/sim`. It never appears in `core/format` or any header the Unreal plugin includes (see [0002](../decisions/0002-unreal-integration.md)). Eigen headers are added as system includes so strict warnings do not apply to them.
- Fetching: the CMake file declares Eigen with a source subdirectory that does not exist, so Eigen's own build (tests, docs, many targets) is never configured, and exposes an interface target pointing at the headers.
- Determinism: results are reproducible for one binary on one platform, for any thread count, because each filter runs on a single thread. As with the libm functions, they are not promised to match across platforms.

### Joseph form and a gate on every fix

- The covariance update uses the Joseph form with explicit symmetrization, which keeps the covariance symmetric and positive semi-definite despite rounding.
- Innovation covariance is solved with an LDLT decomposition, not an explicit inverse. If it is not positive definite the update is skipped and counted.
- Position fixes are gated: an update whose normalized innovation squared exceeds the 0.999 quantile of chi-square with three degrees of freedom (about 16.27) is rejected and counted. A test checks empirically that about 0.1% of valid fixes are rejected, which validates the constant instead of trusting it.

### Estimators see only sensor outputs

Sensor output types (`ImuSample`, `PositionFix`) move to one header, `sensor_data.hpp`, that includes nothing about the simulated truth. Estimator sources may include only that header, vector and matrix types, and each other. `scripts/check_core_invariants.sh` enforces this with an include whitelist, so a violation fails CI. This turns the rule "estimators must not read ground truth" from a convention into a check.

### The launch pose is configuration

A pose estimator starts from a launch position, velocity, and uncertainty passed to its constructor. This models a drone that knows where it was deployed. It is a one-time input, not a per-tick leak, and tests show the estimator's output depends only on that and the sensor stream.

### Neighbor data on the wire

Drones broadcast their own estimate, not ground truth. A neighbor message is a small fixed binary layout (version byte, send tick, position, velocity, position standard deviation) with explicit little-endian encoding, no struct copying. A receiver accepts only a message newer than the latest it holds for that sender, so reordered and duplicate messages are ignored. Malformed payloads are counted and dropped.

### Delay compensation

A received state describes the sender at its send tick, and arrives later. The Kalman predictor extrapolates the report forward by the delay before fusing it, and inflates its uncertainty by the matching process noise. The hold-last and dead-reckoning baselines deliberately do not compensate except where dead reckoning extrapolates from the report time, so the comparison shows what the delay handling is worth.

### Tuning is documented, and checked on synthetic data

Process-noise values come from the IMU model's own parameters for pose estimation, and from a stated acceleration spectral density for neighbors (`q = 1 m^2/s^3`, meaning a velocity uncertainty of about 1 m/s after one second without information). These are defaults for a swarm that maneuvers at a few m/s^2. They are not claimed to be optimal; they can be tuned in Phase 7 ablations.

## Components and contracts

### Linear Kalman filter (commit 21)

`LinearKalmanFilter<N>` holds state `x` and covariance `P` as fixed-size Eigen types.

- `predict(F, Q)`; `predict(F, Q, B, u)` with a control input.
- `update<M>(z, H, R)` returns an `UpdateResult` with the innovation, the innovation covariance, the normalized innovation squared, and whether the update was applied. An optional gate rejects an update by its NIS.
- Rejects non-finite input by throwing `std::invalid_argument` (a programming error); a non-positive-definite innovation covariance is reported in the result instead.
- Constant-velocity helpers: `constant_velocity_transition(dt)` and `constant_velocity_process_noise(dt, q)` for a six-state position-velocity model.

### Sensor output types and the position fix sensor (commit 22)

- `sensor_data.hpp`: `ImuSample` (moved from `imu.hpp`) and `PositionFix { bool available; Vec3 position; double sigma; }`.
- `PositionFixSensor`: configured with a base sigma at a reference valid-return fraction, the smallest fraction that still yields a fix, and a sigma ceiling. Given the true position and a `Scan`, it computes the valid-return fraction `f`. If `f` is below the minimum there is no fix. Otherwise `sigma = base_sigma * sqrt(reference_fraction / f)`, capped, and the reported position is the truth plus Gaussian noise of that sigma.
- Random draws: three normals per call whatever the result, on stream `kStreamFixBase + drone id` (`0x4000 + id`).

### Pose estimator (commit 23)

A nine-state Kalman filter over position, velocity, and accelerometer bias, in the world frame.

```
a_meas = specific_force + gravity           (world-frame acceleration reading)
p' = p + v dt + 0.5 (a_meas - b) dt^2
v' = v + (a_meas - b) dt
b' = b
```

- Prediction takes an `ImuSample`; process noise comes from the IMU's accelerometer noise density and bias walk (a known sensor specification, passed in as configuration).
- Update takes a `PositionFix`, uses `H = [I 0 0]` and `R = sigma^2 I`, and is gated as above.
- Output: `PoseEstimate { position, velocity, accel_bias, position_sigma, velocity_sigma }` plus counters for fixes applied and rejected.
- The gyro and yaw are not used.

### Neighbor messages and tracking (commit 24)

- `encode_neighbor_state` / `decode_neighbor_state` for the layout above.
- `NeighborTable`: for each neighbor id, the last accepted state, its send tick, and its receipt tick. `age(id, now)` is `now - receipt_tick`. Reordered or duplicate messages (send tick not newer than the held one) are ignored and counted; malformed payloads are counted and dropped.

### Neighbor predictors (commit 25)

```
class INeighborPredictor {
  void on_report(const NeighborReport& report, std::uint64_t receipt_tick);
  NeighborEstimate predict(std::uint64_t now_tick) const;   // position, velocity, sigma, age
};
```

- `HoldLastPredictor`: reports the last received position and velocity unchanged.
- `DeadReckoningPredictor`: `p = p_report + v_report * (now - send_tick) * dt`.
- `KalmanNeighborPredictor`: six-state constant-velocity filter. A report is extrapolated by its delay, then fused; between reports the filter predicts each tick, so its covariance grows during dropout.

All three are driven by the same calls, so a scenario can switch between them from configuration.

### Boundary enforcement (commit 26)

- `scripts/check_core_invariants.sh` gains an include whitelist for the estimation sources and accepts an optional root directory so it can be tested.
- A test runs the script against a fixture directory that contains a violating file and expects failure, and against the real tree and expects success.

### Integration (commit 27)

A test agent runs IMU, LiDAR, fix sensor, pose estimator, neighbor messaging and predictors on the lockstep loop in a generated scene with weather and jamming.

## Commit-by-commit specification

Each commit builds warning-free, adds the listed tests, and passes `ctest`, the invariants script, and the format check. `MODELS.md` gains a section in the commit that adds each model.

### 21. `estimation: add Eigen and a linear Kalman filter`

Files: `cmake/Dependencies.cmake` (Eigen), `swarm/estimation/kalman.hpp`, `tests/unit/test_kalman.cpp`.

Tests:
- Textbook one-dimensional case: prior `x = 0, P = 1`, measurement `z = 1, R = 1` gives `x = 0.5, P = 0.5`.
- Steady-state gain and covariance of a scalar random walk match the closed-form solution of its Riccati equation.
- A constant-velocity tracker driven by noisy position measurements converges to the true velocity.
- The covariance stays symmetric and positive semi-definite over 10,000 steps, and the Joseph form matches the textbook form for the optimal gain.
- An update never increases the covariance trace; a prediction never decreases it.
- An unobserved state's covariance is not changed by an update.
- A measurement with enormous noise changes the state negligibly.
- NIS is reported correctly, and gating rejects an update above the threshold.
- With correct models, about 0.1% of valid three-dimensional updates exceed the 16.27 gate (checked over many seeded trials).
- Non-finite input throws; a non-positive-definite innovation covariance is reported, not thrown.

### 22. `sensors: add sensor output types and the position fix model`

Tests:
- No fix when the valid fraction is below the minimum; a fix when at or above it.
- `sigma` equals `base_sigma` at the reference fraction and scales as the inverse square root of the fraction; it never exceeds the ceiling.
- Measurement noise standard deviation matches the reported sigma over many calls.
- Heavy fog on a real scan removes the fix, clear air gives one (the coupling to the LiDAR actually works).
- Random draws are fixed per call: the noise value for the same call index is identical whether or not earlier calls produced a fix.
- Two drones are independent; the same seed and id reproduce exactly.
- Invalid configuration is rejected.
- The IMU tests still pass after `ImuSample` moves.

### 23. `estimation: add the pose estimator`

Tests:
- A stationary drone with a biased accelerometer and regular fixes: the bias estimate converges to the true bias and position error stays bounded.
- Without fixes, position error grows (dead reckoning drift), and the reported position sigma grows with it; with fixes both stay bounded.
- Between fixes the covariance grows, and a fix shrinks it.
- A grossly wrong fix is rejected by the gate and counted; the estimate is unchanged by it.
- Over many seeded runs of a moving drone, position error is within three reported sigma for at least 95% of the samples (a basic consistency check without NEES).
- Identical inputs give identical outputs; two interleaved estimators do not affect each other.
- Invalid configuration and non-finite inputs are rejected.

### 24. `estimation: add neighbor messages and tracking with age`

Tests:
- Encode then decode round-trips every field exactly; the byte layout is checked against hand-written bytes.
- Truncated, oversized, wrong-version, and non-finite payloads are rejected.
- Age increases by one each tick with no message and resets when one arrives.
- A message with an older send tick than the held one is ignored; an equal one is a duplicate and ignored; a newer one replaces it.
- Several neighbors are tracked independently; an unknown neighbor reports "no data".

### 25. `estimation: add neighbor predictors`

Tests:
- All three predictors satisfy the same contract and can be created from configuration.
- Constant-velocity target, exact reports, then a dropout: dead reckoning and Kalman are accurate, hold-last error grows linearly with the gap.
- Noisy reports (fixed seeds, 300 trials): after the same dropout, mean position error orders as Kalman below dead reckoning below hold-last.
- Kalman covariance grows monotonically during a dropout and is within three sigma for at least 95% of trials.
- Delay compensation: with reports delayed by a known number of ticks, the Kalman predictor's error is smaller than the same predictor with compensation disabled.
- Reordered reports do not corrupt the Kalman state.
- A maneuvering target (a constant-acceleration segment) increases every predictor's error, and the result is reported honestly in the test comments.

### 26. `estimation: enforce that estimators cannot read ground truth`

Tests:
- The invariants script fails on a fixture containing a forbidden include (for example the dynamics or scene headers) in an estimation source.
- It passes on the real tree.
- The pose estimator and the predictors compile against only the whitelisted headers (checked by the script on the real tree).

### 27. `test: run estimation on the lockstep loop`

Tests:
- Final and per-tick hashes identical for 1, 2, 4, and 8 threads with fog, burst loss, jamming, and all three predictors active.
- Estimated pose error over a one-minute run stays below a stated bound when fixes are available, and is larger when fog removes them.
- Neighbor prediction error during jamming orders as Kalman below dead reckoning below hold-last, on the whole-swarm average.
- Same seed reproduces; a different seed differs.

## Documentation produced

- `docs/MODELS.md` sections: Kalman filter conventions, position fix, pose estimator, neighbor messages and predictors, each with equations, parameters, assumptions, limitations, and validation status.
- An ADR for the Eigen decision (commit 21).

## Risks

| Risk | Handling |
|---|---|
| Eigen headers trip strict warnings or the C++20 build | Added as system includes; a C++20 CI job already exists |
| Eigen fetch fails on a clean machine | Pinned URL; CI exercises a clean fetch on every run |
| Filter tuning looks arbitrary | Defaults derived from the IMU model and a stated acceleration density, checked by consistency tests, documented as tunable |
| The synthetic fix is mistaken for real localization | Stated in the README limitations, `MODELS.md`, and this document |
| Statistical tests flake | Fixed seeds, tolerances sized from the trial counts, written in the tests |
| Ground truth leaks into an estimator later | Include whitelist enforced by the invariants script in CI |
| Plain weighted fusion double-counts correlated estimates | Not applicable yet: each neighbor report is treated as an independent measurement, and the limitation is documented |

## Exit criteria

- All seven commits made, each green locally and in CI.
- The three neighbor predictors can be compared by tests and by the integration run.
- MODELS.md and the Eigen decision record written; PLAN.md change log and README status updated.
