# Phases

Each row is one commit. Every commit adds at least one new test, builds without warnings, and leaves CI green. Phases are ordered so each one is usable on its own; later phases build on earlier ones. Scope and cuts are in [PLAN.md](PLAN.md).

## Rules for every commit

1. One behavior per commit. Write the test with the code, or before it.
2. Before committing, run:
   ```sh
   cmake --build --preset debug && ctest --preset debug
   bash scripts/check_core_invariants.sh
   clang-format --dry-run -Werror $(git ls-files '*.cpp' '*.hpp')
   ```
3. Commit message: `area: what changed` (for example `channel: add burst loss model`). No emojis, no trailers.
4. Push only when the local checks pass. Fix a red CI run before starting the next row.
5. A phase ends with a short tag-free checkpoint: update the status line in the README and the change log in PLAN.md.

## Phase 1: Deterministic runtime

Goal: a lockstep simulation loop whose state hash does not depend on thread count, with a log that replays exactly.

| # | Commit | New tests |
|---|---|---|
| 1 | `Message` and a delivery queue ordered by (deliver tick, sender id, sequence) | Ordering, tie-break by sender, stable for equal keys |
| 2 | `IChannel` interface and `DeterministicChannel` with fixed latency | Delivery arrives exactly `latency` ticks later; nothing arrives early |
| 3 | Random loss on a dedicated RNG stream | Observed loss rate near target; same seed gives identical drops |
| 4 | Burst loss (two-state) | Mean loss matches target; mean burst length matches target |
| 5 | Latency jitter and a rate limit | Jitter bounds; queue drops past the rate limit |
| 6 | Lockstep thread pool with a barrier per tick | Every task runs once per tick; tick count correct; no deadlock; clean under TSan |
| 7 | World state and per-tick state hash | Hash identical with 1, 2, and 8 threads |
| 8 | Binary log writer and reader in `core/format` with a version header | Round trip; wrong version rejected; truncated file rejected |
| 9 | Replay with `--verify` | Replay hash equals recorded hash; a corrupted log fails verification |

Exit check: `swarm_sim` records a run, replays it, and the hashes match on the CI determinism job.

## Phase 2: Vehicles and sensing

Goal: drones that move and sense a generated scene, with weather affecting the sensor.

| # | Commit | New tests |
|---|---|---|
| 10 | `Vec3` and point-mass drone dynamics with acceleration and speed limits | Limits respected; free motion matches analytic result |
| 11 | IMU model: white noise and bias random walk | Noise variance and bias drift match parameters |
| 12 | `IScene`, seeded heightfield and obstacle generator | Same seed same scene; obstacles inside bounds; spawn area clear |
| 13 | Ray casting against the scene | Hits on known primitives and heightfield at analytic distances |
| 14 | LiDAR scan pattern and range noise | Beam directions correct; noise statistics match |
| 15 | Weather: visibility threshold dropout and rain range reduction | Returns beyond visibility dropped; clear weather unchanged |
| 16 | Jamming zones in the channel: constant, pulsed, localized | Loss only inside the zone and active window; duty cycle correct |

## Phase 3: Estimation

Goal: each drone knows its own pose and predicts neighbors when packets stop.

| # | Commit | New tests |
|---|---|---|
| 17 | Linear Kalman filter (Eigen added here) | Converges on a known system; covariance behaves as expected |
| 18 | Pose estimator from IMU and a LiDAR-derived position fix | Error bounded on a synthetic trajectory |
| 19 | Neighbor state tracking with message age | Age increments each tick without a message and resets on one |
| 20 | Neighbor prediction during dropout | Beats hold-last-state on a synthetic dropout |
| 21 | Estimator interface that cannot see ground truth | Compile-time check, plus a property test that estimates are unchanged when ground truth is perturbed |

## Phase 4: Control

Goal: the swarm holds formation, stays separated, and degrades in defined steps.

| # | Commit | New tests |
|---|---|---|
| 22 | Formation offsets (virtual structure) | Targets match the geometry for each formation |
| 23 | Separation rule with an age-widened margin | Minimum separation holds over many seeds |
| 24 | Fallback state machine with hysteresis | Each transition fires at its threshold; no flapping near a threshold |
| 25 | Mission waypoints and path following | Reaches waypoints within tolerance |
| 26 | Drone agent tick loop wiring all of the above | Two drones with no loss reach formation |

## Phase 5: Scenarios, faults, metrics, baselines

Goal: scenario-driven runs that produce the numbers in the plan.

| # | Commit | New tests |
|---|---|---|
| 27 | YAML scenario loader (yaml-cpp added here) | Valid files load; missing and invalid fields rejected with a clear error |
| 28 | Fault injection: drone failure at a given time | Failed drone stops sending and moving |
| 29 | Metrics from logs: formation RMSE, minimum separation, collisions | Hand-built logs give known values |
| 30 | Recovery time metric | Known recovery on a synthetic error trace; censored case handled |
| 31 | Latency and delivery-ratio metrics | Known values on a synthetic message log |
| 32 | CSV export | Header and row format; round trip through a parser |
| 33 | Baselines: hold-last-state, dead reckoning, oracle | Oracle error is zero; baselines differ from the predictor on a dropout |
| 34 | Scenario runner end to end | Zero loss and clear weather reproduce the oracle result |

## Phase 6: Real network backend

Goal: the same drone code runs over real UDP and TCP sockets.

| # | Commit | New tests |
|---|---|---|
| 35 | Wire protocol serialization | Round trip; malformed and truncated input rejected |
| 36 | Portable socket wrapper (Windows and Linux) | Open, bind, send, receive on loopback |
| 37 | `UdpChannel` for state beacons | Loopback delivery between two endpoints |
| 38 | `TcpChannel` for commands | Ordered, acknowledged delivery on loopback |
| 39 | Loss shim in front of the real channel | Observed loss matches the shim setting |
| 40 | Real versus simulated channel comparison tool | Runs and emits latency and loss statistics |

## Phase 7: Batch runs and analysis

Goal: reproducible result tables and figures.

| # | Commit | New tests |
|---|---|---|
| 41 | Experiment matrix expansion from a config | Cell count is 432 for the planned matrix; seeds enumerated |
| 42 | Batch driver running cells in parallel | Output identical regardless of how many cells run at once |
| 43 | Ablation configurations | Each ablation disables exactly one component |
| 44 | Python analysis: aggregation and confidence intervals (pytest) | Known inputs give known intervals |
| 45 | Figure scripts | Run on a small fixture and produce the expected files |
| 46 | Fog visibility fit against published numbers | Model output within tolerance of the cited curve |

## Phase 8: Unreal viewer

Goal: replay a recorded run in Unreal 5.4. CI cannot build Unreal, so each row has a manual check in addition to a C++ test where one applies.

| # | Commit | New tests or checks |
|---|---|---|
| 47 | Log reader used from the plugin | Same reader sources pass a core test; plugin builds |
| 48 | Terrain mesh from the scenario heightfield | Manual: matches core geometry at sampled points |
| 49 | Obstacles and drones spawned from code | Manual: counts match the log |
| 50 | Replay playback and camera | Manual: playback state hash matches the log at the final tick |
| 51 | Weather and jamming visuals (optional) | Manual |

## Phase 9: Release

| # | Commit | Check |
|---|---|---|
| 52 | Benchmark run with 5 seeds, results table in README | Numbers regenerate from one command |
| 53 | 30-seed run if time allows | Same command, config change only |
| 54 | Hero GIF, Docker verified, limitations reviewed | `docker run` works on a clean machine |
| 55 | Tag `v0.1.0` | CI green on the tagged commit |
