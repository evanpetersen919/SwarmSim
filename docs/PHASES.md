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

Goal: a lockstep simulation loop whose state hash does not depend on thread count, with a log that replays exactly. Detailed design: [phases/phase-01-deterministic-runtime.md](phases/phase-01-deterministic-runtime.md). Each later phase gets the same kind of document before its first commit.

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

Goal: drones that move and sense a generated scene, with weather affecting the sensor. Detailed design: [phases/phase-02-vehicles-and-sensing.md](phases/phase-02-vehicles-and-sensing.md).

| # | Commit | New tests |
|---|---|---|
| 10 | `Vec3` and point-mass drone dynamics with acceleration and speed limits | Exact discrete closed form; limits respected; yaw wrap |
| 11 | IMU model: white noise and bias random walk | Noise variance and bias drift match parameters; independent streams |
| 12 | Scene data, serialization, and hash in `core/format` | Round trip; byte layout; truncation; hash sensitivity |
| 13 | Seeded terrain generator and exact height queries | Determinism; amplitude bound; exact plane; golden hash |
| 14 | Seeded obstacle placement | Spacing, bounds, spawn clearance, termination when saturated |
| 15 | Terrain ray casting | Analytic hits; brute-force agreement on random rays |
| 16 | Obstacle ray casting and unified traversal | Face-by-face analytic hits; brute-force agreement; cell-size independence |
| 17 | LiDAR scan pattern, range noise, dropouts | Pattern geometry; noise and dropout statistics; fixed draws per ray |
| 18 | Weather extinction and detection threshold | Closed-form maximum range; boundary flips; noise unaffected |
| 19 | Jamming zones in the channel | Zone, window, pulse duty, boundary; stream independence; thread invariance |
| 20 | Sensors and dynamics on the lockstep loop | Hashes identical for 1, 2, 4, 8 threads; concurrent scene queries |

## Phase 3: Estimation

Goal: each drone knows its own pose and predicts neighbors when packets stop. Detailed design: [phases/phase-03-estimation.md](phases/phase-03-estimation.md).

| # | Commit | New tests |
|---|---|---|
| 21 | Eigen and a linear Kalman filter | Textbook case; Riccati steady state; covariance stays symmetric and PSD; gating rate matches the chi-square constant |
| 22 | Sensor output types and the position fix model | No fix below the valid-return minimum; sigma scaling; fog removes the fix; fixed draws |
| 23 | Pose estimator from IMU and position fix | Bias converges; drift without fixes; gate rejects outliers; 95% within three sigma |
| 24 | Neighbor messages and tracking with age | Byte layout; malformed input; age resets on a message; reordered and duplicate reports ignored |
| 25 | Neighbor predictors: hold-last, dead reckoning, Kalman | Error ordering over seeded trials; covariance growth; delay compensation |
| 26 | Estimators cannot read ground truth, enforced | Invariants script fails on a fixture with a forbidden include, passes on the tree |
| 27 | Estimation on the lockstep loop | Hashes identical for 1, 2, 4, 8 threads; error bounds; fog removes fixes |

## Phase 4: Control

Goal: the swarm holds formation, stays separated, and degrades in defined steps.

| # | Commit | New tests |
|---|---|---|
| 28 | Formation offsets (virtual structure) | Targets match the geometry for each formation |
| 29 | Separation rule with an age-widened margin | Minimum separation holds over many seeds |
| 30 | Fallback state machine with hysteresis | Each transition fires at its threshold; no flapping near a threshold |
| 31 | Mission waypoints and path following | Reaches waypoints within tolerance |
| 32 | Drone agent tick loop wiring all of the above | Two drones with no loss reach formation |

## Phase 5: Scenarios, faults, metrics, baselines

Goal: scenario-driven runs that produce the numbers in the plan.

| # | Commit | New tests |
|---|---|---|
| 33 | YAML scenario loader (yaml-cpp added here) | Valid files load; missing and invalid fields rejected with a clear error |
| 34 | Fault injection: drone failure at a given time | Failed drone stops sending and moving |
| 35 | Metrics from logs: formation RMSE, minimum separation, collisions | Hand-built logs give known values |
| 36 | Recovery time metric | Known recovery on a synthetic error trace; censored case handled |
| 37 | Latency and delivery-ratio metrics | Known values on a synthetic message log |
| 38 | CSV export | Header and row format; round trip through a parser |
| 39 | Baselines: hold-last-state, dead reckoning, oracle | Oracle error is zero; baselines differ from the predictor on a dropout |
| 40 | Scenario runner end to end | Zero loss and clear weather reproduce the oracle result |

## Phase 6: Real network backend

Goal: the same drone code runs over real UDP and TCP sockets.

| # | Commit | New tests |
|---|---|---|
| 41 | Wire protocol serialization | Round trip; malformed and truncated input rejected |
| 42 | Portable socket wrapper (Windows and Linux) | Open, bind, send, receive on loopback |
| 43 | `UdpChannel` for state beacons | Loopback delivery between two endpoints |
| 44 | `TcpChannel` for commands | Ordered, acknowledged delivery on loopback |
| 45 | Loss shim in front of the real channel | Observed loss matches the shim setting |
| 46 | Real versus simulated channel comparison tool | Runs and emits latency and loss statistics |

## Phase 7: Batch runs and analysis

Goal: reproducible result tables and figures.

| # | Commit | New tests |
|---|---|---|
| 47 | Experiment matrix expansion from a config | Cell count is 432 for the planned matrix; seeds enumerated |
| 48 | Batch driver running cells in parallel | Output identical regardless of how many cells run at once |
| 49 | Ablation configurations | Each ablation disables exactly one component |
| 50 | Python analysis: aggregation and confidence intervals (pytest) | Known inputs give known intervals |
| 51 | Figure scripts | Run on a small fixture and produce the expected files |
| 52 | Fog visibility fit against published numbers | Model output within tolerance of the cited curve |

## Phase 8: Unreal viewer

Goal: replay a recorded run in Unreal 5.4. CI cannot build Unreal, so each row has a manual check in addition to a C++ test where one applies.

| # | Commit | New tests or checks |
|---|---|---|
| 53 | Log reader used from the plugin | Same reader sources pass a core test; plugin builds |
| 54 | Terrain mesh from the scenario heightfield | Manual: matches core geometry at sampled points |
| 55 | Obstacles and drones spawned from code | Manual: counts match the log |
| 56 | Replay playback and camera | Manual: playback state hash matches the log at the final tick |
| 57 | Weather and jamming visuals (optional) | Manual |

## Phase 9: Release

| # | Commit | Check |
|---|---|---|
| 58 | Benchmark run with 5 seeds, results table in README | Numbers regenerate from one command |
| 59 | 30-seed run if time allows | Same command, config change only |
| 60 | Hero GIF, Docker verified, limitations reviewed | `docker run` works on a clean machine |
| 61 | Tag `v0.1.0` | CI green on the tagged commit |
