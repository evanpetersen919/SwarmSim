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

Goal: the swarm holds formation, stays separated, and degrades in defined steps. Detailed design: [phases/phase-04-control.md](phases/phase-04-control.md).

| # | Commit | New tests |
|---|---|---|
| 28 | Formation shapes, and the ground-truth boundary extended to controllers | Count, centering, spacing; the invariants script fails on a controller fixture that includes truth |
| 29 | Mission reference | Constant speed along a polyline; continuity at corners; hold and loop |
| 30 | Formation target and control law | No-neighbor case; inverse-variance weighting; blend extremes; convergence without oscillation |
| 31 | Separation rule | Margin, direction, cap, widening with uncertainty; closest approach over 500 seeded pairs |
| 32 | Fallback state machine | Thresholds, hysteresis, dwell, hold and recovery timing |
| 33 | Drone brain | Driven by synthetic sensor data; fallback transitions; output depends only on inputs |
| 34 | Drone agent and a formation swarm | Two drones reach formation; eight converge with bounded separation; thread invariance |

## Phase 5: Scenarios, faults, metrics, baselines

Goal: scenario-driven runs that produce the numbers in the plan. Detailed design: [phases/phase-05-scenarios-and-metrics.md](phases/phase-05-scenarios-and-metrics.md).

| # | Commit | New tests |
|---|---|---|
| 35 | Strict YAML scenario loader (yaml-cpp added here) | Valid files load with defaults; unknown and missing keys, wrong types, and bad ranges rejected with a path and line; canonical round trip |
| 36 | Build the scene and agents from a scenario | Counts and offsets match; same seed same world; start bounds; thread-invariant hashes |
| 37 | Message outcomes from the channel | Behavior unchanged with no sink; one event per message; drop reasons match the statistics |
| 38 | Run snapshot codec | Byte-exact layout; round trip; truncated and malformed input rejected |
| 39 | Run a scenario, record it, and verify it | Log decodes; thread-invariant bytes; verification detects one corrupted byte; demo logs still verify |
| 40 | Drone failure in the drone agent | Frozen and silent after the failure tick; survivors measured against a stated bound |
| 41 | Metrics: formation error, minimum separation, collisions | Hand-built logs give known values; collision episodes counted once |
| 42 | Metrics: recovery time | Known recovery on synthetic traces; hold time; censored case; several disturbances |
| 43 | Metrics: latency and delivery ratio | Nearest-rank percentiles; in-flight excluded; observed ratio near 0.7 at 30 percent loss |
| 44 | CSV export and the metrics command | Exact header and row text; round trip through a parser |
| 45 | Baselines: oracle, and selectable predictors | Oracle trajectories independent of the channel; non-oracle agents unchanged by the peer view |
| 46 | Scenario runner end to end | Kalman within a measured margin of the oracle at zero loss; fog removes fixes; examples run and verify |

## Phase 6: Real network backend

Goal: the same drone code runs over real UDP and TCP sockets, and the result is bit-identical to the simulated channel. Detailed design: [phases/phase-06-network-backend.md](phases/phase-06-network-backend.md).

| # | Commit | New tests |
|---|---|---|
| 47 | Wire protocol | Byte-exact layout; round trip; every truncation and each malformed header rejected; stream decoder reassembles any chunking |
| 48 | Channel model extracted; injectable channel; reliable message class | Hash of a harsh run unchanged; model matches channel; reliable messages never dropped and consume no random draws |
| 49 | Hold and resume commands in the brain, and a ground station agent | Hold stops and holds; resume re-forms; stale and malformed commands ignored; commands survive heavy loss |
| 50 | Scenario `commands` list | Loader and canonical round trip; no change without commands; snapshots and metrics ignore the station |
| 51 | Portable socket wrapper (Windows and Linux) | Loopback UDP and TCP; poll timeout is a status; RAII closes; invariants forbid sockets in the core |
| 52 | UDP transport for beacons | Exact delivery between endpoints; burst of thousands; foreign and malformed datagrams counted |
| 53 | TCP transport for commands | Ordered, whole, lossless over 10,000 messages; one-byte writes reassemble; connections opened once per pair |
| 54 | Socket channel (model plus real transport, barrier delivery) | State hash equal to the deterministic channel for every thread count; events equal; observed loss matches the setting |
| 55 | Run a scenario over sockets | Log byte-identical to the simulated run; verifies on the simulated channel |
| 56 | Real versus simulated comparison tool | Identical deliveries; documented fields; ordered percentiles; no absolute latency asserted |

## Phase 7: Batch runs and analysis

Goal: reproducible result tables and figures.

| # | Commit | New tests |
|---|---|---|
| 57 | Experiment matrix expansion from a config | Cell count is 432 for the planned matrix; seeds enumerated |
| 58 | Batch driver running cells in parallel | Output identical regardless of how many cells run at once |
| 59 | Ablation configurations | Each ablation disables exactly one component |
| 60 | Python analysis: aggregation and confidence intervals (pytest) | Known inputs give known intervals |
| 61 | Figure scripts | Run on a small fixture and produce the expected files |
| 62 | Fog visibility fit against published numbers | Model output within tolerance of the cited curve |

## Phase 8: Unreal viewer

Goal: replay a recorded run in Unreal 5.4. CI cannot build Unreal, so each row has a manual check in addition to a C++ test where one applies.

| # | Commit | New tests or checks |
|---|---|---|
| 63 | Log reader used from the plugin | Same reader sources pass a core test; plugin builds |
| 64 | Terrain mesh from the scenario heightfield | Manual: matches core geometry at sampled points |
| 65 | Obstacles and drones spawned from code | Manual: counts match the log |
| 66 | Replay playback and camera | Manual: playback state hash matches the log at the final tick |
| 67 | Weather and jamming visuals (optional) | Manual |

## Phase 9: Release

| # | Commit | Check |
|---|---|---|
| 68 | Benchmark run with 5 seeds, results table in README | Numbers regenerate from one command |
| 69 | 30-seed run if time allows | Same command, config change only |
| 70 | Hero GIF, Docker verified, limitations reviewed | `docker run` works on a clean machine |
| 71 | Tag `v0.1.0` | CI green on the tagged commit |
