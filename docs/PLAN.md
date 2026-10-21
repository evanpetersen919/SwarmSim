# SwarmSim: Project Plan

Status: approved scope, timeline flexible. Last updated 2026-10-20.

This document is the log of what the project is, what is in scope, what is deliberately cut, and in what order work happens. Changes to scope go here first, with a dated entry in the change log at the bottom.

## Problem

Drone swarms fail when the conditions they were designed for degrade. Rain and fog thin and corrupt LiDAR returns. Jamming and terrain cause packet loss that arrives in bursts, not as independent events. Indoors or underground there is no GPS. Most swarm simulations model loss as a flat probability, ignore the effect of weather on the sensor itself, and cannot be rerun to an identical result, so their curves cannot be compared with anyone else's.

## Goal

Build a reproducible C++17 simulation of a decentralized drone swarm and measure how formation and safety degrade as comms and sensing get worse. Visualize runs in Unreal Engine 5.4.

## Research question

Under bursty packet loss and weather-degraded LiDAR, how much does neighbor prediction with a fallback state machine preserve formation integrity compared with hold-last-state?

## Scope

This is a simulation study. Comms and jamming models are parameterized, not measured against real radios. Results are claimed only for the simulator.

Vehicles: a quadrotor and an ALTIUS-class fixed-wing aircraft, behind a vehicle interface (ADR 0010). The fixed-wing parameters come from public press figures and the literature, each marked published, literature, or assumed. The aircraft is not claimed to match any real model, and strike behavior is out of scope.

## Architecture

```
scenario.yaml + seed
        |
  SceneGenerator  (heightfield, obstacles, spawns)
        |
  Core library (C++17, no Unreal dependency)
    SimClock (fixed step) -> lockstep thread pool (barrier per tick)
    Drone agents: dynamics, IMU, LiDAR + weather, pose Kalman,
                  neighbor Kalman, formation, separation rule, fallback FSM
    IChannel:  DeterministicChannel (default, benchmarks)
               UdpTcpChannel        (real sockets, demo)
    Metrics + binary run log + replay
        |
  Batch runner -> CSV -> Python analysis
  UE5.4 plugin: builds the scene from the scenario, replays logs
```

Design rules:

1. One clock. All timestamps use simulation time. The core never reads wall-clock time.
2. Lockstep concurrency. Drone logic runs on a thread pool with a barrier each tick. Messages are delivered in (sim time, sender id) order, so thread scheduling cannot change results.
3. Same seed produces the same state hash, independent of thread count.
4. The estimator never reads ground truth. Ground truth is used only for evaluation.
5. Every model is behind an interface chosen from the scenario file.
6. Metrics are computed from logs, not inside the simulation.
7. Nothing is placed by hand in the Unreal editor. The scene is generated from the scenario.

## Components

| Component | Behavior |
|---|---|
| SimClock | Fixed-step simulation time |
| Lockstep pool | Worker threads, barrier per tick, mutex-protected queues |
| Drone dynamics | Kinematic point mass with acceleration and speed limits |
| IMU | White noise plus bias random walk |
| LiDAR | Ray-cast against generated scene geometry, configurable beams, range noise |
| Weather | Visibility sets a detection threshold, returns below it drop, rain shortens range |
| Comms model | Random and burst loss, fixed latency plus jitter, constant / pulsed / localized jamming, simple rate limit |
| Pose estimate | Kalman filter over IMU plus a LiDAR-derived position fix |
| Neighbor estimate | Kalman filter predicting neighbor state while packets are missing |
| Formation | Consensus or virtual-structure offsets along a mission path |
| Separation rule | Repel inside a margin, margin widened by estimate age |
| Fallback FSM | Nominal, degraded comms, perception only, safe hold, with hysteresis |
| Logger and replay | Binary per-tick log, bit-exact replay, CSV metrics export |
| UDP/TCP backend | Simple binary serialization, UDP beacons, TCP commands, blocking sockets, loss shim |

## Scenes

Generated from code and a seed: open field, hills (noise heightfield). Urban blocks if time. Cave is out of scope for now.

## Evaluation

Axes: packet loss (0, 10, 30, 50 percent; random and burst at equal mean), weather (clear, fog, rain), jamming (none, constant, pulsed), drones (5, 10, 20), faults (0 or 1 failure), scene (field, hills).

Metrics: formation error RMSE, minimum separation and collisions, recovery time, latency p50 and p95, delivery ratio, tick time versus N.

Baselines: hold-last-state, dead reckoning, oracle. Ablations: no prediction, no fallback, no separation rule.

Seeds: start at 5 per cell. The final run should raise this to 30; it is one configuration value. 432 cells at 5 seeds is 2,160 runs.

Validation: one sourced fog visibility fit, a real-socket versus simulated-channel latency comparison, and sanity tests (zero loss and clear weather reproduce the oracle; filter error matches its stated covariance on synthetic data).

## Testing

Target about 100 tests, and report the real number. Unit, property (minimum separation, no ground-truth access, deterministic message order), determinism (same seed same hash, thread-count invariance, no wall-clock calls), and integration. Sanitizers (ASan, UBSan, TSan) in CI.

## Timeline

Flexible. The commit-by-commit breakdown with the test for each step is in [PHASES.md](PHASES.md). Rough order of work:

1. Core scaffolding, SimClock, deterministic channel, lockstep pool, log and replay, CI
2. Dynamics, IMU, LiDAR, weather, comms model
3. Scene generator, pose Kalman, neighbor Kalman
4. Fallback FSM, formation, separation rule, metrics
5. Wire protocol and real UDP/TCP channel, baselines
6. Batch runs, ablations, figures, UE5.4 scene builder and replay
7. UE5 demo clip, README with GIF, Docker, release

Estimated 6 to 7 weeks, plus buffer for Unreal build and threading issues. Dropping the real-socket backend saves about 4 to 5 days.

## Cut list

Cut for now: multi-process mode, epoll, SCHED_FIFO and affinity, tc netem, lock-free queue, checksums, Parquet, Slurm scripts, error-state EKF, ICP, ORCA/CBF, Gilbert-Elliott, SINR, NEES/NIS, ns-3, FAST-LIO2, PX4 SITL, ROS2, paper.

First upgrades if time allows: 30-seed rerun, Covariance Intersection, Gilbert-Elliott channel, ICP estimator with a FAST-LIO2 comparison.

Later: multi-process drones with epoll, real-time tuning, ORCA/CBF, fault injection, learned predictor versus Kalman, 50 drones, cave scene.

## Known risks

| Risk | Mitigation |
|---|---|
| Nondeterminism creeps in | Hash-compare CI, thread-count invariance test, no wall-clock reads, ordered delivery |
| Unreal integration eats the schedule | Core never depends on Unreal; the visualizer is built last and only replays logs |
| Pose estimate is not real localization | Define the LiDAR measurement as a position fix and say so, or drop GPS-denied wording |
| Weighted fusion double-counts correlated estimates | Document the limit; Covariance Intersection is the first upgrade |
| 5 seeds is too few for tight intervals | Raise the seed count for the final run |
| Models unvalidated | One sourced fog fit and the real-versus-simulated channel comparison; label the rest as assumptions |
| No clear winner in the results | Still a valid result; report where each method breaks |
| Fixed-wing control grows beyond its limit | v0.1 is loiter orbits, offset phases, speed matching, and one separation rule; nothing else |
| Fixed-wing parameters and the camera-fix model are unvalidated | Parameter table with provenance; sensitivity runs on the assumed values; label them as assumptions |

## Honest limits

The project demonstrates multithreading, sockets and Linux tooling. It does not demonstrate kernel-level or real-time work. Describe it that way. Novelty was checked with only a few searches; Swarm-LIO2 and ns-3-based co-simulators are the closest prior work and should be cited in the README.

## Change log

- 2026-10-16: Initial approved plan. Multi-process mode, epoll, real-time tuning, tc netem, lock-free queue, checksums, Parquet and Slurm cut. Unreal Engine version fixed at 5.4. Timeline marked flexible.
- 2026-10-16: Project renamed from SwarmResilience to SwarmSim. Unreal plugin and module are now SwarmSim.
- 2026-10-17: Phase 1 (deterministic runtime) complete: delivery queue, deterministic channel with latency, loss, burst loss, jitter and rate limit, lockstep pool, simulation loop with state hash, binary log, record and replay verification. See docs/phases/phase-01-deterministic-runtime.md.
- 2026-10-17: Phase 2 (vehicles and sensing) complete: dynamics, IMU, scene data, terrain and obstacle generation, ray casting, LiDAR, weather, jamming, and a lockstep integration test. See docs/phases/phase-02-vehicles-and-sensing.md.
- 2026-10-18: Phase 3 (estimation) complete: Eigen and a Kalman filter, a simulated position fix, a pose estimator, neighbor messages and tracking, three neighbor predictors, and an enforced boundary keeping estimators away from ground truth. Finding: Kalman prediction beats dead reckoning for straight flight but not for maneuvering neighbors at the default tuning. See docs/phases/phase-03-estimation.md.
- 2026-10-18: Phase 4 (control) complete: formation shapes, mission reference, formation target and control law, separation rule, fallback state machine, drone brain, and drone agent, with the ground-truth boundary extended to controllers. Eight drones form up from a scattered start under loss and jamming. Finding: the separation margin is a soft preference; closest approaches during assembly were 1.89 m against an 8 m margin, and the margin cannot stop fast closing speeds. See docs/phases/phase-04-control.md.
- 2026-10-18: Phase 5 (scenarios, faults, metrics, baselines) complete: a strict YAML scenario loader, a scenario runner that records a verifiable log with per-tick snapshots and message events, drone failure, metrics computed from the log alone, CSV export, and an oracle baseline. Finding: in steady straight flight, packet loss up to 50 percent changes the settled formation error by under 0.05 m, and fog turns the position fix off abruptly between 200 m and 150 m visibility. A failed lead drone in single file is flown into unless silent neighbors keep their last known position as an obstacle. See docs/phases/phase-05-scenarios-and-metrics.md.
- 2026-10-19: Phase 6 (network backend) complete: a versioned wire format, a channel model separated from the channel, reliable hold and resume commands from a ground station, UDP and TCP transports on loopback, a socket channel that delivers at a barrier, a `--transport sockets` option for `swarm_sim run`, and a `swarm_netcmp` comparison tool. Finding: a scenario run over sockets writes a log byte-identical to the simulated run, and a network that loses messages produces a log that fails verification. Scenario running takes a channel factory because `core/` cannot include `net/`. See docs/phases/phase-06-network-backend.md.
- 2026-10-19: Phase 7 (batch runs and analysis) complete: experiment files that expand into cells (the planned matrix is 432), a parallel batch driver whose table does not depend on the job count, ablation switches for the fallback and the separation rule, a Python analysis package with Student t intervals and paired differences, figure scripts, and a fog check against one published relation. Findings: the position fix is lost at 45 m altitude, so experiments fly at 20 m; the default fog constant is 1.304 times the published one, a difference in visibility convention, and a setting exists to use the published value. See docs/phases/phase-07-batch-and-analysis.md.
- 2026-10-20: Scope change: add an ALTIUS-class fixed-wing vehicle behind a vehicle interface, with a camera position-fix sensor, orbit-based control, and a turn-aware neighbor predictor, and keep the quadrotor as a second vehicle (ADR 0010). The benchmark matrix is recomputed for two vehicles and swarm sizes of 6, 10, and 16, with 28 as an extra case; the 432-cell figure above is for the quadrotor only. The local tag `quad-baseline` marks the code before this change. Work order: vehicle interface with no change to existing logs, fixed-wing model, camera fix, orbit control, predictor, metrics, benchmark, viewer.
