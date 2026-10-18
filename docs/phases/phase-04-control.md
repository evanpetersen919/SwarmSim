# Phase 4: Control

Status: complete (commits 28 to 34). Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md), [MODELS.md](../MODELS.md), [phase 3](phase-03-estimation.md).

## Goal

A real drone agent: it senses, estimates, decides, and sends, flying a mission in formation with its neighbors, keeping apart from them, and degrading in defined steps when information is lost. After this phase the project can run a whole swarm end to end, which is what the experiments in later phases need.

## Non-goals

Terrain and obstacle avoidance (drones avoid each other, not the scene), yaw control and formation rotation, collision avoidance guarantees (the separation rule is a safety margin, not a proof), the scenario file loader, metrics computed from logs, and the Unreal viewer.

## The architectural decision: a brain that cannot see the truth

The drone agent is split in two.

- **Vehicle side** (`DroneAgent`, in `core/sim`): owns the simulated truth (`DroneState`), the dynamics, and the sensor models, which are allowed to read truth. It produces sensor outputs and applies a command.
- **Brain** (`DroneBrain`, under `core/sim/**/control/`): pure decision code. Inputs: an `ImuSample`, a `PositionFix`, received message payloads, and the tick. Outputs: an acceleration command and the report to broadcast. It holds the pose estimator, the neighbor table and predictors, the formation and separation logic, and the fallback state machine.

The brain has no access to truth by construction. The Phase 3 include whitelist is extended to the `control/` directories, so a controller that reaches for dynamics, the scene, or the simulation fails CI, exactly as for estimators. Because the brain is a plain object with plain inputs, it is also testable without a simulation: unit tests drive it with synthetic sensor data.

The command type moves into its own header (`drone_command.hpp`) so the brain can name its output without including the truth-side dynamics header.

Vehicle capabilities the brain needs (maximum acceleration and speed) are passed in as configuration, as a controller on a real drone would know its airframe limits.

## Decisions

### Formation: leaderless consensus plus a shared mission reference

Every drone `i` has a slot offset `o_i` from a formation shape and a copy of the mission. From its own estimate `p_i` it forms `c_i = p_i - o_i`, its belief about where the formation center is. From each usable neighbor `j`, using the predictor's estimate `p_j`, it forms `c_j = p_j - o_j`.

```
c_consensus = inverse-variance weighted mean of c_i and the usable c_j
c_ref       = mission reference r(t)
target      = alpha * c_ref + (1 - alpha) * c_consensus      (alpha = 1 with no usable neighbors)
accel       = k_p * (target + o_i - p_i) + k_d * (v_target - v_i)
```

with the velocity target blended the same way. The inverse-variance weights use each predictor's reported uncertainty, so a stale neighbor counts for less.

Why this form:

- With good communication the consensus term tightens the formation. With none, the mission term alone keeps every drone aimed at the same moving reference, so the formation degrades gradually and does not collapse. That is the behavior the experiments are meant to measure.
- No leader means no single point of failure and no re-election logic. A partitioned swarm splits into groups that each hold formation around the shared reference.
- Rejected alternatives: leader-follower (a single point of failure and a re-election problem), and pure consensus with no reference (the formation would drift anywhere and never follow a mission).

Formation offsets stay fixed in the world frame. They do not rotate with the direction of travel. Rotation is a later refinement.

### Separation: a potential-field margin that widens with uncertainty

For each neighbor with an estimate, the margin is `m_j = d_safe + kappa * sigma_j`, where `sigma_j` is the predictor's position uncertainty. A neighbor closer than `m_j` produces a repulsive acceleration along the line between them, growing linearly with penetration and capped. Because stale predictions carry larger sigma, the margin widens as information ages, which is the "age-widened margin" of the plan. The repulsion is added to the formation command and the sum is clamped to the vehicle limit, with the repulsion gain high enough that it dominates inside the margin.

This is a safety margin, not a guarantee: it relies on estimates, so the tests measure the true minimum separation over many seeded runs instead of assuming it.

### The fallback machine is a pure function of two conditions

Two conditions drive it, each with its own enter and exit thresholds:

- **comms bad:** too few neighbors have a report fresher than a staleness limit.
- **navigation bad:** no position fix for too long, or the pose uncertainty above a limit.

| State | Entered when | Behavior |
|---|---|---|
| Nominal | both fine | Full formation gain, mission weight `alpha` at its default |
| Degraded comms | comms bad | More weight on the mission reference, wider separation margin, reduced speed |
| Degraded navigation | navigation bad | Lower speed, wider margin by own uncertainty, lower formation gain |
| Safe hold | both bad for a hold time, or uncertainty above an abort limit | Decelerate to a stop and hold altitude |

Enter and exit thresholds differ, and each state has a minimum dwell, so noise around a threshold cannot make the drone flap between states. Leaving safe hold requires the conditions to stay good for a recovery window. The machine itself takes numbers in and returns a state, so every transition and the hysteresis are unit-tested without a simulation.

The plan's third state was named perception-only. Drones are not objects in the scene, so the LiDAR cannot see neighbors, and a state that relied on that would be fiction. The state is named for what it really is, degraded navigation.

### Mission: waypoints and a deterministic reference

The mission is a list of waypoints and a cruise speed. The reference `r(t)` starts at the first waypoint at tick 0 and moves along the path at constant speed, then holds at the last waypoint or loops. It is a pure function of the tick, so every drone computes the same reference with no messages. Zero-length segments and a single waypoint are valid.

### Gains

Default `k_p = 1.0` per second squared and `k_d = 1.8` per second give a damping ratio of 0.9. These are starting values, stated as such, and the tests check that the formation converges and does not oscillate from a perturbed start with perfect communication.

## Components and contracts

### Formation shapes (commit 28)

`make_formation(shape, count, spacing)` returns offsets, centered on the origin, in the world frame. Shapes: line (along y), wedge (a V trailing in -x), grid (square-ish). No trigonometry is used, so the offsets are identical on every platform. A circle needs `sin`/`cos` and is left for later.

### Mission reference (commit 29)

`Mission { waypoints, speed, loop }` and `mission_reference(mission, time)` returning position and velocity. Position is continuous at corners; velocity changes at corners.

### Formation target (commit 30)

A pure function: own estimate, a list of neighbor estimates with their slots, the mission reference, `alpha`, and gains in; target position, target velocity, and acceleration out. Includes the inverse-variance consensus, the exclusion of neighbors with no usable estimate, and the PD law.

### Separation (commit 31)

A pure function: own position, neighbor positions with sigma, parameters in; acceleration out. Handles coincident positions by choosing a deterministic direction.

### Fallback state machine (commit 32)

`FallbackFsm::update(inputs, tick)` returns the state. Inputs: the number of neighbors with fresh reports and the total, ticks since the last fix, and the pose uncertainty. Parameters: thresholds, minimum dwell, hold time, recovery window, and the per-state behavior scales.

### Drone brain (commit 33)

`DroneBrain` wires the pose estimator, neighbor table, one predictor per neighbor (kind selected by configuration), the fallback machine, formation target, and separation. `step(imu, fix, inbox, tick)` returns the command and, every report period, a payload to broadcast. It exposes read-only introspection (state, estimates) for tests and later for logging.

### Drone agent (commit 34)

`DroneAgent : IAgent` owns the truth and sensors, calls the brain, and applies the command through the dynamics. Its `hash_state` covers truth and the brain's state, so the lockstep determinism checks cover control too.

## Commit-by-commit specification

Each commit builds warning-free, adds the listed tests, and passes `ctest`, the invariants script, and the format check. Linux CI must be green before the next phase starts. `MODELS.md` gains a section in the commit that adds each part.

### 28. `control: add formation shapes and extend the ground-truth boundary to controllers`

Files: `swarm/drone_command.hpp` (moved out of `dynamics.hpp`), `swarm/control/formation.hpp`, invariants script and fixtures, tests.

Tests:
- Every shape gives the requested count, centered on the origin, with the closest pair of slots at least the spacing apart.
- Shapes are deterministic and distinct slots never coincide.
- Count 0 and 1 and invalid spacing are handled as specified.
- The invariants script now covers `control/` and fails on a fixture controller that includes dynamics or the scene, and passes on a clean one.
- Existing dynamics and agent code still compile against the moved command type.

### 29. `control: add the mission reference`

Tests:
- Constant speed along a polyline, with position continuous at corners and velocity matching the segment direction.
- Reaches each waypoint at the time implied by the distances and speed.
- Holds at the last waypoint, or wraps when looping.
- A single waypoint, repeated waypoints, and zero speed are handled as specified; invalid input is rejected.
- Pure function of time: evaluating at the same time twice, and in any order, gives identical results.

### 30. `control: add the formation target and control law`

Tests:
- No usable neighbors gives exactly the mission reference plus the slot offset.
- Neighbors that agree with the reference leave the target unchanged for any `alpha`.
- A neighbor whose center estimate is off pulls the target toward it in proportion to its inverse-variance weight; a neighbor with huge uncertainty barely does.
- Neighbors without a valid estimate are excluded.
- `alpha = 1` ignores neighbors; `alpha = 0` ignores the reference.
- The control law is zero at the target, points toward it, and a point-mass simulation from a perturbed start converges without oscillation (damping ratio check) with perfect information.

### 31. `control: add the separation rule`

Tests:
- Zero outside the margin; directed away from the neighbor inside it.
- Magnitude rises with penetration and is capped.
- The margin widens with the neighbor's sigma.
- Two drones receive equal and opposite repulsion; several neighbors sum.
- Coincident positions give a finite, deterministic direction.
- Property test: over 500 seeded pairs on converging courses with perfect information, the closest approach never goes below a stated bound.

### 32. `control: add the fallback state machine`

Tests:
- Each transition fires at its threshold and not before, for both conditions.
- Hysteresis: a signal wobbling around the enter threshold does not flap; exit needs the lower threshold.
- Minimum dwell is respected.
- Both conditions bad for the hold time enters safe hold; an uncertainty above the abort limit enters it immediately.
- Leaving safe hold needs the recovery window of good conditions.
- The behavior scales for each state match the table.
- Invalid thresholds (exit above enter, negative times) are rejected.

### 33. `control: add the drone brain`

Tests (driven by synthetic sensor data, no simulation):
- A single brain with no neighbors flies toward the mission reference and holds formation slot offset.
- Fed neighbor reports in a consistent formation, it commands nearly zero acceleration at the target.
- When reports stop, its fallback state moves to degraded comms after the staleness limit, and back after reports resume.
- Without fixes it goes through degraded navigation toward safe hold, and recovers when fixes return.
- The brain's output depends only on its inputs: identical input sequences give bit-identical commands, and two interleaved brains do not interfere.
- The predictor kind is taken from configuration.
- Every piece of the brain's state that affects behavior is included in its hash.

### 34. `sim: add the drone agent and run a formation swarm`

Tests:
- Two drones with no loss reach formation and stay there.
- Eight drones from a scattered start converge to the formation: formation error falls below a stated bound and the true minimum separation stays above a stated bound.
- A jamming window drives the fallback machine through degraded comms and back, the formation survives it, and separation holds.
- Hashes identical for 1, 2, 4, and 8 threads with loss, jamming, fog, and all predictor kinds.
- Same seed reproduces; a different seed differs.
- The ThreadSanitizer job runs all of it.

## Documentation produced

- `docs/MODELS.md` sections: formation, mission, separation, fallback machine, and the brain, each with equations, parameters, assumptions, limitations, and validation status.
- An ADR for the brain/vehicle split (commit 28).
- PLAN.md change log and README status at the end of the phase.

## Risks

| Risk | Handling |
|---|---|
| Gain tuning produces oscillation or sluggish formation | Tests check convergence and damping with perfect information before any degradation is applied |
| Consensus and the mission term fight each other | A single blend weight, with both extremes tested; the blend is in the fallback table |
| The fallback machine flaps | Separate enter/exit thresholds, minimum dwell, and a dedicated hysteresis test |
| Separation fails when estimates are wrong | The margin widens with uncertainty; tests measure true minimum separation over many seeds and report it |
| Drones fly into terrain or obstacles | Out of scope; scenarios keep the corridor clear, and the limitation is stated in the README and MODELS.md |
| Controllers start reading truth | Include whitelist extended to `control/`, tested with a deliberate violation |
| Results look better than they are | Tests assert measured bounds, and the docs report what the measurements show, including unfavorable ones |

## Exit criteria

- All seven commits made, each green locally and in CI (GCC, Clang, sanitizers, C++20).
- A swarm of eight drones flies a mission in formation with the fallback machine active, deterministically for any thread count.
- MODELS.md and the decision record written; PLAN.md change log and README status updated.

## Outcome

### Results

Numbers come from `tests/integration/test_formation_swarm.cpp` and `tests/unit/test_separation.cpp`; the scenario is described in the Drone agent section of [MODELS.md](../MODELS.md).

- **Formation.** Eight drones starting scattered converge to their slots on the mission reference. Over 10 seeds with 5 percent burst loss, the largest final slot error is 0.43 m.
- **Separation is a soft margin.** The smallest true separation over 10 seeds is 1.89 m, reached while the formation assembles, inside the 8 m margin. A jam of all links overlapping assembly gives 1.92 m. Two drones on head-on courses at 5 m/s each stay at least 3.67 m apart over 500 seeds; at 15 m/s each they come within 0.13 m, because the closing speed exceeds what the acceleration limit can stop inside the margin.
- **Fallback.** A 6 s jam during assembly takes the machine into degraded comms on every seed, every drone ends nominal, and the formation still forms. Determinism holds: hashes are identical for 1, 2, 4, and 8 threads.

### Problems found by the tests and fixed in the phase

- A clamped sum of formation pull and repulsion let the pull cancel the repulsion; head-on drones at 5 m/s reached 0.03 m. The separation term now has priority and the formation term gets only the remaining headroom (`combine_accel`).
- The margin widened with a neighbor's sigma without bound. After reports stopped, a drone in safe hold was pushed away at the acceleration limit up to its speed limit. The margin is now capped (`max_margin`), and the brain rejects formations whose slots are not farther apart than the cap.
- A speed limiter at the cruise speed stopped a lone drone from closing its gap to the reference. The limit is now a multiple of the cruise speed (`speed_limit_factor`), which also bounds the closing speed during assembly: in one run the closest approach rose from 0.86 m to 2.64 m.
- With no packet loss, drones started in degraded comms because no report had arrived at tick 0. Comms is not judged for the first staleness window.

### Changes from the plan above

- The controller include whitelist also allows `swarm/format/hash.hpp`, so the fallback machine and the brain can fold their state into the run hash. This is a hashing utility and carries no vehicle state.
- `FallbackFsm` gained `hash_state`, and `separation_accel` gained `max_margin` and `combine_accel`. `BrainConfig` gained `speed_limit_factor`.
- The fallback margin scales are 1.25 for all degraded states and safe hold, not 1.5 and 2.0, because the cap makes larger values meaningless.
- The jam in the swarm test overlaps assembly. A jam in steady flight leaves error and separation unchanged because the formation is already in place.

### Open items

- The brain's speed-aware margin: the margin does not depend on closing speed, so safety relies on the speed limit.
- All thresholds, gains, and scales are starting values and have not been tuned or ablated (Phase 7).
- Terrain and obstacle avoidance, and yaw control, are not implemented. Scenarios must keep the corridor clear.
- Linux CI for commits 33 and 34 appears in the Actions history after the push.
- Benchmark numbers are still not recorded.

