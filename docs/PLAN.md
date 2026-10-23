# SwarmSim: Project Plan

Status: approved scope, timeline flexible. Last updated 2026-10-22.

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

Target about 100 tests, and report the real number. Unit, determinism (same seed same hash, thread-count invariance, no wall-clock calls), and integration; the property group (minimum separation, no ground-truth access, deterministic message order) is an empty directory, and those properties are checked by tests in the other groups and by `scripts/check_core_invariants.sh`. Sanitizers (ASan, UBSan, TSan) in CI.

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
- 2026-10-21: The fixed-wing vehicle (ADR 0010) is built through the benchmark: vehicle interface, coordinated-turn model, camera position fix, orbit guidance with closest-approach separation, a turning neighbor predictor, and orbit metrics. A first probe showed that with phase from a shared clock alone nothing a drone heard changed where it flew, and that a failed aircraft frozen in the air distorted the separation results; a blended phase consensus over the neighbors and an uncontrolled-flight failure model were added. Three fixed-wing experiments (216, 72, and 48 cells, five seeds) were run; the quad matrix of 432 cells has not been run. Finding: weather that blinds the camera is the dominant effect; jamming and loss do not affect the default controller; the predictors separate (turning, Kalman, dead reckoning, hold-last, in that order) only when the aircraft depend on what they hear; the separation rule and the fallback machine were not shown to help. See docs/phases/fixed-wing-vehicle.md.
- 2026-10-21: Scope change: flight realism for the fixed-wing vehicle (ADR 0011). After watching a replay in the Electric Dreams level, where the aircraft flew through cliffs at an altitude chosen without the terrain, four additions: wind and gusts, an energy-based vehicle model with roll dynamics and a load-factor stall, terrain-aware altitude guidance on a scene read from the level, and racetrack and lawnmower patterns with staggered starts. Scenario version 3; version 1 and 2 logs still verify. The patterns are standard ones and assumptions: public sources do not describe what the real aircraft fly. Work order: wind, vehicle model, scenario keys, terrain guidance and ground-contact metrics, patterns, experiments, viewer. The 30-seed fixed-wing runs and the quad matrix wait until this is done, because the new model changes the fixed-wing results.
- 2026-10-21: The flight realism work of ADR 0011 is built: wind and gusts, the energy model, a terrain map with terrain-aware altitude, racetrack and lanes patterns with staggered starts, ground contact, and height-above-ground metrics (two new CSV columns). Running a pattern on the exported Electric Dreams level found a climb response that was a fiftieth of the command in the energy model and a start altitude that ignored obstacles; both are fixed. The experiments of ADR 0010 have not been rerun under the new model. See docs/phases/flight-realism.md.
- 2026-10-22: Added a formation pattern for the fixed-wing vehicle: a lead track with smooth offsets for trail, right echelon, left echelon, and vee, with shape changes over a set time. Spacing, altitude, and change times are assumptions. See docs/MODELS.md and scenarios/fixed_wing_formation.yaml.
- 2026-10-22: Added a rake shape to the formation pattern: a line abreast with spacing from the sensor swath, W = 2 h tan(hfov / 2) (1 - overlap), and an optional altitude stagger for any shape. A 15-aircraft scenario flies one pass. Geometry and parameters are assumptions from an unsourced description. See docs/MODELS.md and scenarios/fixed_wing_rake.yaml.
- 2026-10-22: Fixed two faults in terrain-aware guidance found by an audit: the planned climb now comes from what the energy model's propulsion can hold at the mission speed, and an aircraft starts at the floor of its own path instead of the ground under its start point. Proposed in ADR 0012: a radar altimeter as the first added sensor, with the controller raising its altitude when the measured height above ground is below the clearance; the forward camera is deferred. See docs/decisions/0012-radar-altimeter.md.
- 2026-10-22: Built the radar altimeter of ADR 0012 (`sensors.radar_altimeter`, scenario version 3): a nadir height sensor on the fixed-wing vehicle, used by the controller as a floor on the present. Off unless the section is present, so earlier runs are unchanged. With no map it keeps aircraft off hills that otherwise catch them (a test), and it does not help where the ground rises faster than the aircraft can climb. The forward camera is the next sensor and needs its own decision. See docs/MODELS.md.
- 2026-10-22: Ran a pilot of the fixed-wing aircraft on the current model (`experiments/pilot_fixed_wing.yaml`, 8 cells, 5 seeds). Fog is the only effect that reaches the flight; packet loss and the radar altimeter changed nothing, because the aircraft stay apart and terrain guidance already clears the ground. The fixed-wing benchmark therefore needs scenarios in which comms matter before it can address the research question. The forward camera is on hold until it is decided whether the first release needs it; the proposal is to freeze the sensors at the camera fix and the radar altimeter. See docs/phases/flight-realism.md.
- 2026-10-22: Ran the quadrotor benchmark with 30 seeds per cell on the release build: the matrix (432 cells, 12,960 runs), the baselines, the baselines with bursty loss, and the ablation (48 cells and 1,440 runs each). None failed. Kalman prediction lowers the settled formation error compared with hold-last, by 2.14 m under independent loss and 2.44 m under bursty loss, and dead reckoning does as well as the Kalman filter; the separation rule matters; the fallback state machine is not shown to help; the oracle is not an upper bound and that is unexplained. Added `experiments/baselines_burst.yaml`, `analysis/results_quad.py`, and docs/RESULTS.md. The fixed-wing aircraft has pilots only.
- 2026-10-23: Examined the residual ground contacts (`scripts/crash_report.py`, new; 30 seeds, 180 aircraft for each of three cases of the ADR 0015 avoidance on steep hills). In clear air on 400 m hills, of 18 aircraft on the ground 12 were on it at launch and 5 more stopped within 5 s of the start, with no time to sense or react (`launch_contacts` counts only the first record, so these are in `late_contacts`); one is a later contact. In fog on 480 m hills, of 63, 23 launch, 9 early, 12 separation conflicts (aircraft dodging the same hill together), 2 at the climb limit, 1 at the turn limit, and 16 'other': heading straight at the ground with time to turn, at half or less of the sideways acceleration the rule may use, because the turn command is proportional to the angle to the nearest free column whatever the time left. The next change is to use the full bank when time is short; not built. Diagnosis in docs/RESULTS.md.
- 2026-10-23: Built and ran ADR 0016 (`MessageType::kHazard`, `control.share` with modes off, memory, and share, `min_sources`, and the fault kind `false_hazards`; `experiments/fixed_wing_share.yaml`, 108 cells, 3,240 runs, `fixed_wing_share_sensitivity.yaml`, 180 runs, and `fixed_wing_share_trust.yaml`, 32 cells, 960 runs; none failed; criteria written before the runs). Sharing helps where the sensor is blind: in fog on 400 and 480 m hills, late ground contacts of six aircraft against memory -0.30 [-0.52, -0.08] and -0.60 [-0.89, -0.31], and nothing in clear air; memory alone gives no benefit that the intervals show and costs corridor share; 60 percent loss keeps about three quarters of the advantage and a jam over 44 percent of the run keeps 43 to 87 percent; sharing costs 20 to 30 m of height and 35 percent more messages. One faulty sender does harm that the pre-registered measures (late contacts, corridor share) did not see and time on station (1.00 to 0.43), radial error and height did; the confirmation rule of two removes it and keeps about 43 percent of the benefit. 1,029 tests pass. Results in docs/RESULTS.md.
- 2026-10-23: Found why the crashes did not move (a temporary trace, not committed): while slowing, the braking command (-17 m/s^2) and the turn (-6.17) together exceed the brain's limit of 10, `combine_accel` scales the whole command, and the achieved turn is about 3.3 m/s^2, since ADR 0014. Proposed ADR 0018: the turn keeps its share (`control.avoid.turn_first`), judged by a pre-fixed 36-cell experiment. Not built.
- 2026-10-23: Accepted, built, and ran ADR 0017 (`control.avoid.urgent_s`; `experiments/fixed_wing_avoid_urgent.yaml`, 48 cells, 30 seeds, 1,440 runs, none failed; committed before the run). U1 is not met: late contacts in fog with no map are -0.03 [-0.35, +0.28] and -0.13 [-0.45, +0.19] against `all3`. U2 to U4 are met; the corridor share is 0.04 to 0.06 higher in fog (observation, not a criterion). The crash classes do not move, so the unused bank was not the cause; the next step is to record what the brain senses and decides in the last seconds. Results in docs/RESULTS.md. 1,030 tests pass.
- 2026-10-23: Corrected the residual-contact diagnosis: the `separation` class of `scripts/crash_report.py` counted neighbors already stopped on the ground, so it is now 0 in fog 400 m and 2 of 63 in fog 480 m, and same-column dodging is not a target. ADR 0017 is the next entry.
- 2026-10-23: Accepted ADR 0016: sharing what the forward sensor sees (a lossy hazard message, a per-cell store with a ttl, modes off, memory, and share so that memory is separated from the link, a `false_hazards` fault and a `min_sources` confirmation rule for trust), judged by a pre-fixed 108-cell experiment and a 32-cell trust experiment with the criteria written before the runs. Building it next.
- 2026-10-23: Built and ran ADR 0015 (`control.avoid.rejoin`, `feasibility`, `clearance_m`; `orbit_corridor_share`; `experiments/fixed_wing_avoid_rejoin.yaml`, 112 cells, 30 seeds, 3,360 runs, and `fixed_wing_avoid_clearance.yaml`, 6 cells, 180 runs; none failed; criteria written before the run). With all three changes the share of samples within 100 m of the aircraft's circle goes from 0.49 to 0.96 (400 m hills, clear air, no map), the radial error from 202 m to 34 m and time on station from 0.07 to 0.14 against 0.30 for the forward sensor alone, with late contacts of six aircraft 0.37 against 0.30 for avoidance as built (+0.07 [-0.03, +0.16]), and with an exact map avoidance no longer costs the formation (time on station 0.92 to 1.00 against 0.71 to 0.89). The clearance of 30 m, all three, and a 6 s horizon meet every criterion; the rejoin heading alone and the kinematic judgment alone fail the exact-map one. The formation is still not whole on the steep hills, because the phase lost in a detour is not recovered. Results in docs/RESULTS.md. 1,005 tests pass.
- 2026-10-23: Accepted ADR 0015: rejoining the guidance after avoiding and judging a threat against the vehicle's kinematics (`control.avoid.rejoin`: the preferred heading comes from the guidance's own acceleration, for every pattern; `feasibility: kinematic`: the exact altitude reachable under the climb and vertical acceleration limits; `clearance_m`: a margin of its own for what counts as clearing; a new metric `orbit_corridor_share`), judged by a pre-fixed 112-cell, 30-seed experiment with the criteria for better written before the run. Found by tracing the flown paths of ADR 0014: the aircraft do return, after excursions of 500 to 770 m. Building it next.
- 2026-10-23: Built and ran ADR 0014 (`control.avoid`, `launch_contacts`, `first_ground_contact_s`; `experiments/fixed_wing_avoid.yaml`, 96 cells, 30 seeds, 2,880 runs, and `fixed_wing_avoid_sensitivity.yaml`, 12 cells, 360 runs; none failed; levels fixed from a probe and committed before the run). On hills that climbing cannot clear, avoidance takes the late ground contacts of six aircraft from 2.03 to 0.30 (400 m hills) and from 3.13 to 0.53 (480 m) against the forward sensor's altitude floor alone, and removes the aircraft collisions, and it gives up the formation to do it (time on station 0.07 against 0.30 at 400 m, radial error 202 m against 12 m); with an exact map it costs the formation (0.71 to 0.89 time on station against 0.94 to 1.00) for no safety gain. The safety result holds for horizons of 6, 12, and 20 s and bank fractions of 0.5 and 0.9; a 6 s horizon costs a third of the formation error. Results in docs/RESULTS.md. 999 tests pass.
- 2026-10-23: Accepted ADR 0014: avoiding terrain and obstacles by turning and slowing, driven by the forward sensor (`control.avoid`, a column rule: turn toward the free column nearest the heading, slow while time is short, hand back to the orbit after a hold), judged by a pre-fixed 30-seed experiment on generated hills with cost measures next to safety measures, and two new metrics (`launch_contacts`, `first_ground_contact_s`). Building it next.
- 2026-10-23: Added an audit of the scene export: `-SwarmAuditScene=PATH` in the viewer writes the top surface of the level at every export grid point, and `scripts/audit_export.py` compares it with the scene file and the aircraft's map. On the 2 km Electric Dreams scene the level is more than 5 m above the simulation's surface at 14.62 percent of points (hornbeam crowns, up to 66.1 m; the scene holds a tree as a thin trunk cylinder) and, along the demo's orbit corridor, at 0.91 percent (up to 39.2 m); the demo aircraft's flown path stayed at least 108 m above the real level (`swarm_sim positions`, new). The causes were found and fixed: the first export matched only `EuropeanHornbeam_Forest` (the `Field` variants were never exported), trees had no crowns (`-SwarmExportCrown`), and 871 obstacles floated because the exporter also took the level's distant background scenery (`PCGDemo_BackgroundBP_C`, no collision, `-SwarmExportSkipActors=Background` now skips it). Re-exported, the level is above the simulation's surface at 1.00 percent of points (14.62 before), nothing floats, and the simulation's lowest clearance agrees with the real level's. Results in docs/MODELS.md.
- 2026-10-23: Ran the experiment of ADR 0013 (`experiments/fixed_wing_forward.yaml`, 64 cells, 30 seeds, 1,920 runs, and `fixed_wing_forward_sensitivity.yaml`, 36 cells, 1,080 runs; none failed), with its levels fixed from a probe and committed before the run. With a wrong map either sensor keeps aircraft off the ground (at -60 m: 3.03 aircraft of twelve lost without a sensor, 0 with either), the forward sensor is not shown to be safer than the altimeter but holds the orbit better (time on station +0.24 to +0.33), staying clear costs 29 to 37 m of mean height, a random map error costs height and not safety, and fog of 750 m changes neither. About 3 of twelve aircraft are lost at -80 m and with an omitted square whatever the sensor does; 1.9 per run of those are already on the ground half a second in, because they launch below the real ground from a start altitude computed from the wrong map. Results in docs/RESULTS.md. Not built: a launch altitude that does not come from the map, and turning or slowing for terrain.
- 2026-10-23: Built the first part of ADR 0013: the forward obstacle sensor (`sensors.forward`, scenario version 3) with its second altitude floor in the brain, the three prior-map errors (`offset_m`, `error_sigma_m` with `error_correlation_m`, `omit` under `control.terrain`), and the `mean_agl_m` metric, which adds a column to the metrics CSV (33 metric columns). 981 tests pass. In a one-seed scratch run (4 aircraft, 100 m hills, no map) the sensor takes the losses from 2 to 0 at the cost of a 37 m altitude error. The 30-seed dose-response experiment of the ADR is not run yet.
- 2026-10-23: Accepted ADR 0013: a forward obstacle sensor for the fixed-wing aircraft (a ray fan along the heading, weather-dependent range, a second altitude floor from measured hit points) and a prior map that can be wrong (offset, smooth random error, omitted regions), judged by a pre-fixed 30-seed dose-response experiment with cost measures next to safety measures. Not built yet. Turning or slowing for terrain stays out of scope until the result is in.
- 2026-10-23: Added the optional stale-neighbor rule `control.drop_stale_after_ticks` (default 0, off; formation guidance only) and tested it with `experiments/baselines_drop.yaml` and `experiments/baselines_burst_drop.yaml` (96 cells each, 30 seeds, none failed). It lowers hold-last's formation error with a failed drone from 5.49 to 1.80 m (independent loss) and from 5.83 to 2.08 m (bursty loss), leaves dead reckoning, Kalman, and the oracle unchanged, and shrinks the Kalman advantage with a failure from about 4 m to 0.3 to 0.5 m. The v0.1.0 tables are unchanged because the rule is off by default. See docs/RESULTS.md.
- 2026-10-22: Explained the oracle result of the quadrotor benchmark. Hold-last and the oracle keep a failed drone in the formation consensus at full weight, so with one failed drone their settled formation error is about 5.5 m against about 1.5 m for dead reckoning and the Kalman filter; with no failure the oracle equals them (1.44 m) and Kalman beats hold-last by 0.30 m (independent loss) and 0.60 m (bursty loss). The pooled prediction effect in docs/RESULTS.md is mostly a failure effect. A rule that drops a neighbor that has not been heard from within the staleness time is untested and would change the comparison.
- 2026-10-22: Added fallback occupancy to the metrics (the share of alive drones in the degraded-communication, degraded-navigation, and safe-hold states; three CSV columns from the logs) and an experiment for the fallback in thick fog (`experiments/ablation_fog.yaml`, 16 cells, 30 seeds). In the ablation the drones only reach the degraded-communication state, which is why the fallback seemed to do nothing. In fog thick enough to lose the position fix the fallback stops drones, which costs about 76 to 85 m of formation error against the moving slots and gains about 1.7 to 1.9 m of minimum separation, with no collisions either way. Also found that the rain level of the matrix gives an extinction of about 0.0016 per meter, too small to change the LiDAR. See docs/RESULTS.md.
- 2026-10-22: Viewer: the propeller of the ALTIUS model now turns the small tail blades and the model is imported with its node transforms baked (`scripts/bake_gltf_transforms.py`), because Unreal's import drew the tail surfaces about 4.5 times too large; the state lamp is replaced by overlay text (speed of the followed drone, fallback states, message counts, red when out of nominal); message lines show delivered, lost, and jammed messages (`M`, `-SwarmMessages`); a mini map (`N`, `-SwarmNoMap`) shows positions, trails, and the same message lines from above; the overlay adds altitude, climb rate, and heading. A drone's-eye picture-in-picture was tried with a scene capture component and dropped: in the Electric Dreams level the capture draws the sky but the landscape and forest come out white, which needs a fix in the Unreal project's materials, not in the plugin. The hero GIF and screenshots show an ALTIUS-700-class run at 70 mph with a three-second jam. See docs/phases/phase-08-unreal-viewer.md.
