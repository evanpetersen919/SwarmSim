# Phase 5: Scenarios, faults, metrics, baselines

Status: planned. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md), [MODELS.md](../MODELS.md), [phase 4](phase-04-control.md).

## Goal

Run a whole experiment from one file and get numbers out of the log. After this phase a scenario file plus a seed fully describes a run, the run writes a log that contains enough to compute every reported metric without re-running anything, and the three baselines the plan compares against exist.

## Non-goals

Experiment matrices, batch execution, ablation switches, and plots (Phase 7). The real UDP and TCP backend (Phase 6). The Unreal viewer (Phase 8), although the snapshot format defined here is the one it will read. Wall-clock measurements such as tick time versus drone count, which are benchmarks and not log metrics. Reassigning the slot of a failed drone.

## What exists and what is missing

The runtime, the sensors, the estimators, the brain, and the drone agent exist and run deterministically on the lockstep loop. Three things stand between that and the plan's evaluation section:

1. Runs are configured from C++ or from the demo command line. The plan says every run is described by a scenario file.
2. The log records a tick and a state hash per record. Its blob field is reserved and empty. Formation error, separation, and message latency cannot be computed from a hash.
3. The channel reports totals only. Latency percentiles and a delivery ratio need per-message outcomes.

## Decisions

### Scenario files: strict, versioned YAML, scalars parsed by this project

A scenario is a YAML file with `version: 1` at the top level. The loader reads it with yaml-cpp (pinned to 0.8.0 with a SHA-256 of the archive, like Eigen and GoogleTest) and does three things that make it safe to depend on for results:

- **Strict keys.** An unknown key is an error, not a warning, so a misspelled `loss:` cannot silently leave a default in place. A missing required key is an error.
- **All errors at once, with locations.** The loader returns a list of errors, each as a dotted path with a line number where yaml-cpp has one, for example `comms.loss (line 14): must be a number in [0, 1], got 1.5`. Fixing a file should not take one run per mistake.
- **Scalars parsed by the project.** The loader takes each scalar as text and converts it with `std::strtod` or `std::strtoull`, requiring that the whole text is consumed and the value is finite. yaml-cpp's own conversions accept forms the project does not want (`.inf`, `0x10`, `yes` for booleans in some contexts) and are a dependency on library behavior that results then rest on. Booleans are `true` or `false` only.

Times in the file are in seconds. The loader converts a time to a tick with `round(t / dt)` and rejects the file if `t / dt` is not within `1e-6` of an integer, so no rounding happens unnoticed. The thread count is not part of a scenario: it must never change results, so it is a command-line option only.

yaml-cpp is linked only into the scenario library. The invariants script already forbids it in `core/format`, which is compiled into Unreal, and keeps doing so.

Schema, version 1. Required keys are marked; everything else has the stated default.

```yaml
version: 1                      # required
name: hills_jam_demo            # default: the file name
seed: 7                         # required
dt: 0.02                        # seconds per tick, default 0.02
duration_s: 60                  # required

scene:
  type: field                   # field | hills, default field
  size_m: [1000, 1000]          # default [1000, 1000]
  cell_m: 8                     # default 8
  amplitude_m: 4                # default 4 for field, 30 for hills
  obstacles:
    density_per_m2: 0.0         # default 0
    spawn_clear_radius_m: 0

swarm:
  drones: 8                     # required, 1 to 4096
  formation: {shape: wedge, spacing_m: 20}     # line | wedge | grid; default wedge, 20
  start: {columns: 4, spacing_m: 35, jitter_m: 8, altitude_m: 20, altitude_jitter_m: 3}

mission:                        # required
  waypoints: [[-150, 0, 20], [150, 0, 20]]
  speed_mps: 4
  loop: false

weather: {visibility_m: 20000, rain_mm_h: 0}   # default clear

sensors:
  lidar: {beams: 8, azimuth_samples: 36}
  fix_period_ticks: 5

comms:
  latency_ticks: 2
  jitter_ticks: 1
  loss_model: burst             # none | independent | burst
  loss: 0.05
  mean_burst: 2.0
  rate_limit_per_tick: 0
  jammers:
    - {center: [0, 0, 0], radius_m: inf, mode: constant, from_s: 2, to_s: 8, loss: 1.0, affects: either}

control:
  predictor: kalman             # kalman | dead_reckoning | hold_last | oracle
  report_period_ticks: 5
  staleness_ticks: 50

faults:
  - {drone: 3, at_s: 30}        # the drone fails at that time

metrics:
  settle_s: 20                  # formation error is also reported from this time on
  collision_radius_m: 1.0
  recovery_threshold_m: 1.0
  recovery_hold_s: 2.0

log:
  snapshot_every_ticks: 1
```

`inf` is accepted as a jammer radius only, matching the existing jammer text format. Pulsed jammers carry `period_ticks`, `duty`, and `offset_ticks` as in `swarm/jammer.hpp`. Anything the existing configuration structs validate is validated again by the loader, with the same limits, so a loaded scenario cannot construct an invalid simulation.

A scenario is a plain C++ struct (`Scenario`) with no yaml-cpp types in it. A second function writes the canonical text of a scenario back out, and the loader reads its own output to the same struct; that round trip is tested.

### The log carries a snapshot and a message event list per tick

Metrics are computed from logs (a design rule of the plan), so the log has to hold what the metrics need. The log container already has an opaque blob per record, reserved for exactly this. Phase 5 defines the contents without changing the container layout, so the container version stays 1 and its byte-exact layout test is untouched. The blob has its own version byte and its own byte-exact layout test.

Blob layout (little-endian, fixed width), written by the runner every `snapshot_every_ticks` ticks:

```
u8  snapshot_version = 1
u32 agent_count
agent_count x { u8 flags | f64 position[3] | f64 velocity[3] }
u32 event_count
event_count x { u8 kind | u32 sender | u32 receiver | u64 send_tick | u64 deliver_tick }
```

`flags` bit 0 is alive; bits 1 and 2 are the fallback state (0 nominal, 1 degraded comms, 2 degraded navigation, 3 safe hold). Positions and velocities are the true state, which is allowed here: this is the evaluation record, and the brain never sees it. Event kinds: scheduled (will be delivered at `deliver_tick`), dropped by loss, dropped by rate limit, dropped by jamming. Events are recorded for every tick regardless of the snapshot stride, attached to the next written record, so a stride greater than 1 never loses a message.

The snapshot codec lives in `core/format`, because the Unreal viewer will read the same bytes. It follows the format library's rules: no exceptions, status codes, no Eigen, no yaml-cpp. Decoding checks the version, the counts against the remaining bytes, and the kind values, and rejects anything else.

Size, for planning only: eight drones is about 400 bytes of snapshot per tick plus 17 bytes per message event, so a 60 s run at 50 Hz is a little over a megabyte. The runner builds the log in memory; the batch phase will compute metrics from the in-memory bytes and discard them instead of writing every run to disk.

### Message events come from the channel, not from the agents

`DeterministicChannel` gets an optional event sink. When one is set, `send` appends an event for every message: scheduled with its delivery tick, or dropped with the reason. The channel already makes these decisions in a fixed order on the barrier thread, so the event list is deterministic and thread-count invariant. With no sink set the channel behaves exactly as before and the existing channel tests are unchanged. The sink is on the concrete class and not on `IChannel`, because the real-socket backend of Phase 6 will report what it can observe in its own way.

Latency is `deliver_tick - send_tick` of the scheduled events. Because every scheduled message is delivered at its tick, the scheduled delivery tick is the delivery tick.

### Fault injection: a failed drone is frozen and silent

A fault is a drone index and a tick. From that tick the drone's agent does not step: its position and velocity stay as they were, it sends nothing, and it ignores its inbox. It stays in the world as a physical object, so the others can still collide with it and the separation metric still counts it. It does not fall and does not hold a hover; it is a simple, stated failure model, not a vehicle failure model. The snapshot marks it not alive.

What the other drones do with a silent neighbor is the interesting part, and the design does not assume the answer. Their predictors keep extrapolating the last report, the sigma grows, and the weight in the formation consensus falls. If the remaining drones keep a slot error under 2 m after the failure, nothing else is needed. If the ghost of the failed drone drags them further, the commit adds one parameter, a sigma above which a neighbor is left out of the consensus, with a test; the measurement decides, and the result is reported either way. The failed drone's slot stays empty. Reassigning slots is a non-goal.

### Metrics are defined once, here, and computed from the log only

All metrics read a log. The scenario they need (mission, formation offsets, metric settings, fault and jam times) comes from the scenario text stored in the log header, so a log is self-describing and the metrics function takes bytes and nothing else.

- **Reference.** The formation reference for slot `i` at tick `k` is `r(k * dt) + o_i`, where `r` is the mission reference and `o_i` the slot offset, computed with the same `MissionPath` and `make_formation` the drones use.
- **Slot error** of an alive drone is the distance from its true position to its reference. **Formation RMSE** is the root mean square of slot error over alive drones and over the ticks considered. It is reported over the whole run and over ticks from `settle_s` on, because the whole-run number includes the assembly transient. Failed drones are excluded from both; the number of alive drones at the end is reported.
- **Minimum separation** is the smallest true distance between any two drones, alive or failed, over the whole run.
- **Collisions** are counted as episodes: a pair starts an episode when its distance falls below `collision_radius_m` and ends it when the distance is back at or above that radius. The count is the number of episodes over all pairs.
- **Recovery time.** A disturbance is the end of a finite jammer window (`to_s`) or a fault. Let `E(k)` be the formation RMSE over alive drones at tick `k`. For a disturbance at tick `t_d`, recovery is the first tick `t_r >= t_d` such that `E <= recovery_threshold_m` for `recovery_hold_s` without a break, and the recovery time is `(t_r - t_d) * dt`. If there is no such tick before the run ends, the result is censored: it is reported as the time remaining with a censored flag set, never as a number that looks like a recovery. A run with several disturbances reports each one, and the summary row reports the longest and whether any was censored. A run with none reports no recovery time.
- **Latency.** The p50 and p95 of `(deliver_tick - send_tick) * dt` over scheduled messages, by the nearest-rank rule: the value at index `ceil(p * n) - 1` of the sorted list. Messages whose delivery tick is after the last logged tick are still in flight and are excluded. No messages means no value, not zero.
- **Delivery ratio** is scheduled messages delivered by the end divided by those plus all dropped messages, with the drops also reported by reason.

Percentiles, ratios, and RMSE use only addition, division, and `sqrt` on values read from the log, in a fixed order, so the metrics are reproducible on every platform.

### CSV: one row per run, fixed columns

The CSV has a header row and one row per run with a fixed column order, written by one function so that the batch phase concatenates files without translating. Numbers are written with `%.9g`; missing values are empty cells, not zeros. The project never calls `setlocale`, so formatting is locale independent, and the test checks the exact text of a row.

### Baselines: the predictor is selectable, and the oracle is outside the brain

The plan compares hold-last-state, dead reckoning, and an oracle against the Kalman predictor. The first two are already `PredictorKind` values, so a scenario selects them with `control.predictor`.

The oracle is a neighbor estimate that equals the neighbor's true state, which a brain must never be able to read. So the oracle lives on the vehicle side. The simulation already collects every agent's true position at the barrier for the jamming check; it will also publish positions and velocities to the agents as a read-only view of the previous tick's end state. A `DroneAgent` configured for the oracle uses that view and nothing else uses it. The agent passes the values to the brain through an optional field in the brain's input (`neighbor_override`), and the brain uses an override in place of its predictors when one is present. The brain's include whitelist is unchanged: the override is plain `NeighborEstimate` values, and the brain cannot tell where they came from.

Oracle definition: for every other drone, valid, true position and velocity as of the end of the previous tick, sigma zero, age zero. Fresh-neighbor count is the full count, so the comms condition never triggers. The oracle therefore ignores the channel entirely, and two properties follow that are tested directly:

- Under the oracle, true trajectories are identical, bit for bit, for any loss, jitter, latency, and jamming setting, because nothing the channel does reaches the control law.
- With zero loss, one tick of latency, and clear weather, the Kalman predictor's formation RMSE lies within a stated margin of the oracle's. The margin is measured in the commit and recorded, not assumed.

A test also checks that agents not configured for the oracle behave identically whether or not the view is published, so the access cannot leak into the other predictors.

## Components and contracts

### Scenario loader (commit 35)

`load_scenario(text) -> LoadResult { bool ok; Scenario scenario; std::vector<ScenarioError> errors; }` and `canonical_text(scenario)`. `ScenarioError` has a dotted path, a line (0 if unknown), and a message. No exceptions escape; yaml-cpp exceptions are caught and reported as a parse error.

### Scenario to simulation (commit 36)

`build_world(scenario) -> ScenarioWorld`, owning the `Scene` and holding the `SimConfig` and every `DroneAgentConfig`. `make_simulation(world, threads)` returns the `Simulation`; the world must outlive it. Start positions come from a new random stream, `kStreamStart = 0x105`, so changing the start layout never shifts any other stream. The terrain and obstacles use the existing seeded generators, so the golden scene hashes still hold.

### Channel events (commit 37)

`ChannelEvent { kind, sender, receiver, send_tick, deliver_tick }`, a sink pointer set on `DeterministicChannel`, appended to in `send`.

### Snapshot codec (commit 38)

`encode_snapshot` and `decode_snapshot` over `AgentSnapshot` and `MessageEvent` structs in `core/format`.

### Runner and verification (commit 39)

`run_scenario(scenario, threads, log_bytes)` steps the simulation, builds a snapshot after each step, and appends it to the log. The log header's config text is the line `swarmsim-scenario 1` followed by the scenario text verbatim. `verify_log` dispatches on that first line: the existing `key=value` form still means the demo run, and the scenario form re-runs the scenario and compares the hash and the blob bytes of every record. Comparing blobs as well as hashes means a divergence in anything the metrics read is caught, not only a divergence in what the hash covers. The command line gains `swarm_sim run SCENARIO --out FILE`; the existing demo options keep working.

### Faults (commit 40)

`DroneAgentConfig` gets an optional `fail_tick`. The scenario's `faults` list sets it per drone.

### Metrics (commits 41 to 43)

`analyze_log(bytes) -> MetricsResult`, with each metric group in its own commit and its own tests. The function reads the header and the records and nothing else.

### CSV and command line (commit 44)

`metrics_csv_header()` and `metrics_csv_row(result)`; `swarm_sim metrics LOG --csv FILE`.

### Baselines (commit 45)

The peer view in `AgentContext`, `BrainInput::neighbor_override`, and the oracle mode in `DroneAgent`.

## Commit-by-commit specification

Each commit builds warning-free, adds the listed tests, and passes `ctest`, the invariants script, and the format check. Linux CI must be green before the phase is closed. `MODELS.md` gains a section in the commit that adds each part.

### 35. `scenario: add the strict YAML loader`

Files: `cmake/Dependencies.cmake` (yaml-cpp 0.8.0 with the SHA-256 of the archive), a `swarm_scenario` library, `scenario.hpp`, the loader, and an example under `scenarios/`.

Tests:
- A complete valid file loads to the expected struct; a minimal file with only the required keys loads with every documented default.
- An unknown key at the top level and inside each section is an error naming the path.
- Each required key missing is an error. Wrong types (a string for a number, a map for a list) are errors.
- Range checks: negative or zero `dt`, loss outside `[0, 1]`, zero drones, more than 4096 drones, an empty waypoint list, a speed that is not positive, a jammer window that ends before it starts.
- Several mistakes in one file produce several errors, in file order, each with its line.
- A time that is not a whole number of ticks is rejected; one that is, converts exactly.
- `inf` is accepted for a jammer radius and rejected anywhere else; `nan`, `0x10`, and `1e999` are rejected; `yes` is not a boolean.
- Malformed YAML is a parse error and not an exception.
- `canonical_text` followed by `load_scenario` returns an equal struct.
- The shipped example scenarios all load.

### 36. `scenario: build the scene and agents from a scenario`

Tests:
- Drone count, formation offsets, mission, and channel configuration in the built world match the scenario.
- The same scenario gives the same scene hash and the same start positions on repeated builds; a different seed gives different ones.
- Start positions fall within the stated layout bounds and no two start closer than `spacing - 2 * jitter` (the stated bound, checked over 100 seeds).
- Changing the number of drones does not change the terrain or the obstacles.
- An invalid combination the loader cannot see alone (formation spacing not greater than the separation margin) is rejected with a message.
- A built simulation steps and gives thread-count-invariant hashes.

### 37. `sim: report message outcomes from the channel`

Tests:
- With no sink set, the existing channel tests pass unchanged and the channel's behavior is bit-identical to before (hash comparison over a loss and jam configuration).
- Every sent message produces exactly one event; scheduled plus dropped equals sent.
- Drop reasons match the channel statistics for loss, rate limit, and jamming.
- Scheduled events carry `deliver_tick = send_tick + latency + jitter draw`, and the messages arrive at those ticks.
- The event list is identical across runs and independent of the number of agent threads.

### 38. `format: add the run snapshot codec`

Files: `swarm/format/snapshot.hpp` and source in `core/format`, and an ADR (0006) for logs carrying snapshots.

Tests:
- A byte-exact layout test: a hand-built snapshot with known values encodes to a fixed byte string and decodes back.
- Round trip over randomized snapshots, including zero agents and zero events.
- Truncated input at every length is rejected without reading out of bounds; an unsupported version, an event kind out of range, and counts larger than the buffer are rejected with distinct statuses.
- The codec compiles without Eigen or yaml-cpp (the invariants script) and as C++20.

### 39. `scenario: run a scenario, record it, and verify it`

Tests:
- Running a scenario produces a log whose header holds the scenario text and whose record count equals the tick count; every record decodes.
- Snapshot positions match the agents' true positions at that tick.
- The log hash sequence is identical for 1, 2, 4, and 8 threads; the whole log bytes are identical too.
- `verify_log` passes on a fresh log, fails with a tick-specific message on a log with one corrupted snapshot byte, and still passes the existing demo logs.
- The command-line `run` writes a file that `--replay --verify` accepts, and the existing round-trip test still passes.

### 40. `sim: add drone failure to the drone agent`

Tests:
- A drone with `fail_tick` flies normally before it and is frozen after: position and velocity constant, no messages sent, inbox ignored.
- The snapshot marks it not alive from that tick and not before.
- Other drones' receipt counters for the failed sender stop growing after the delivery latency.
- The failed drone still counts in the separation metric and in jamming-zone membership.
- Seeded run with eight drones and one failure: the remaining seven are measured against the 2 m criterion above. The measured error is recorded in the test, and if the criterion fails the commit adds the sigma exclusion parameter and a test that it restores the criterion.
- The scenario `faults` list builds the right `fail_tick` values; a drone index out of range is rejected.

### 41. `metrics: add formation error, minimum separation, and collisions`

Tests (hand-built logs, no simulation):
- A log in which every drone sits exactly on its reference gives zero RMSE; a known constant offset gives exactly that RMSE; a known mix gives the hand-computed value.
- The whole-run and settled values differ as expected when the first part of the log has a larger error.
- A failed drone is excluded from RMSE and included in minimum separation; the alive count at the end is correct.
- Minimum separation matches a brute-force minimum over a random log.
- Collision episodes: a pair that dips inside the radius twice counts two; touching exactly the radius does not count; a pair that stays inside for many ticks counts one.
- A log with no records, or a header with a scenario that does not parse, returns a clear error.

### 42. `metrics: add recovery time`

Tests (synthetic error traces):
- An error that returns under the threshold and stays gives the known recovery time.
- An error that dips under the threshold for less than the hold time and rises again does not count as recovered.
- A trace already under the threshold at the disturbance recovers in zero time.
- A trace that never recovers is censored, with the time remaining and the flag set.
- Two disturbances give two results; an infinite jammer gives none; the summary reports the longest and the censoring.

### 43. `metrics: add latency and delivery ratio`

Tests (synthetic event lists):
- Known latencies give known p50 and p95 under the nearest-rank rule, including one, two, and 100 samples.
- Messages still in flight at the end are excluded; no messages gives no value.
- Delivery ratio and the per-reason drop counts match the lists.
- On a real run with the independent loss model at 30 percent, the observed delivery ratio is within a stated tolerance of 0.7 over a stated number of messages.

### 44. `metrics: add CSV export and the metrics command`

Tests:
- The header text and the exact text of a row for a known result.
- Missing values are empty cells; a result with censoring writes the flag.
- A round trip through a small parser in the test returns the same numbers to nine significant digits.
- `swarm_sim metrics LOG --csv FILE` on a recorded log writes a header and one row; a corrupt log exits with the usage-or-I/O status.

### 45. `sim: add the oracle baseline and selectable predictors`

Tests:
- The scenario's `predictor` selects each of the four kinds; an unknown name is a loader error.
- Under the oracle, true trajectories are bit-identical across a loss-free run and a run with 50 percent loss and a jam.
- An agent not set to the oracle is bit-identical with and without the peer view published.
- Hold-last, dead reckoning, and Kalman produce different neighbor estimates than the oracle during a dropout, and the oracle's estimate error is exactly zero against the previous tick's truth.
- The brain with an override ignores its predictors, and without one is unchanged (the existing brain tests pass untouched).

### 46. `scenario: check the scenario runner end to end`

Tests:
- Zero loss, one tick of latency, clear weather: the Kalman formation RMSE is within the margin of the oracle's, with the margin set from the measurement and recorded.
- Increasing loss (0, 10, 30, 50 percent, random and burst at equal mean) raises or leaves unchanged the neighbor-estimate error, and does not break the run.
- A fog scenario reduces the number of fixes applied compared to clear air.
- All shipped example scenarios run for a short duration and verify.
- A full example (eight drones, burst loss, a jam, one fault) records a log, verifies it, and yields a metrics row with every column filled except the ones the scenario cannot produce.

## Documentation produced

- `docs/MODELS.md` sections: the scenario format, the snapshot format, message events, fault model, the metric definitions, and the baselines, each with parameters, assumptions, limitations, and validation status.
- ADR 0006 (logs carry snapshots and message events; metrics read only logs) and ADR 0007 (strict versioned scenario files with project-parsed scalars).
- A `scenarios/README.md` listing the shipped scenarios and what each exercises.
- PLAN.md change log, the PHASES.md rows, and the README status at the end of the phase.

## Risks

| Risk | Handling |
|---|---|
| yaml-cpp behavior leaks into results | Scalars are parsed by the project from text; yaml-cpp only supplies structure. Pinned version and archive hash |
| A typo in a scenario silently uses a default | Unknown keys are errors; all errors are reported with paths and lines |
| Logs grow too large for batch runs | Estimated per run; batch computes metrics from the in-memory log and discards it; the snapshot stride is configurable |
| Metrics depend on something not in the log | The metric function takes bytes only, and the scenario text is in the header; tests run it on hand-built logs |
| Metric definitions are ambiguous | Defined once in this document and in MODELS.md, with edge cases (censoring, no data, in flight, nearest rank) each tested |
| The oracle leaks truth into the other predictors | Oracle access is on the vehicle side and optional in the brain input; a test checks that non-oracle agents are unchanged by the published view |
| Failed drones drag the survivors | The design measures first and adds an exclusion parameter only if the 2 m criterion fails; the result is reported either way |
| Verification becomes slow with snapshots | It compares blob bytes during the re-run it already does; no extra pass |
| Results look better than they are | Tests assert measured bounds, and the docs report unfavorable numbers |

## Exit criteria

- All twelve commits made, each green locally and in CI (GCC, Clang, sanitizers, C++20).
- A scenario file alone reproduces a run; the log it writes verifies on re-execution for any thread count.
- Every metric in the plan's evaluation section except tick time versus drone count is computed from a log, with its definition documented and its edge cases tested.
- The oracle, dead-reckoning, and hold-last baselines run from the scenario file.
- MODELS.md, the decision records, and the scenario README written; PLAN.md change log and README status updated.
