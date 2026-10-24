# 0020. Starting above the ground

Date: 2026-10-23. Status: accepted.

## Context

After ADR 0019, in fog on 480 m hills with the best avoidance so far (`turn_first`, a catch-up cap of 3 m/s), 35 of 180 aircraft are on the ground. 32 of the 35 are launch or early cases: 23 stopped at the first step and 9 within 5 s. They are not avoidance failures, and nothing since ADR 0014 acts on them.

Why, from the code and from the saved runs. Without a map, an aircraft starts on its slot of the orbit at the nominal altitude (100 m plus its layer), whatever the terrain under it (the start positions in `world.cpp`). The start is only raised to clear the terrain when terrain guidance is on (`start_altitude`, which reads the map). The hills of the experiments peak at about a third of their amplitude under the orbit, so some starts are inside or at the surface of a hill. For the 180 aircraft of the 30 runs of ADR 0019 in fog on 480 m hills, the start height above the true ground (read from each run's exported scene) against what happened:

| Start height above the ground | Aircraft | Stopped at the first step | Stopped within 5 s |
|---|---|---|---|
| at or below 0 m | 8 | 8 | 0 |
| 0 to 30 m | 22 | 13 | 0 |
| 30 to 60 m | 22 | 2 | 3 |
| above 60 m | 128 | 0 | 6 |

So 23 launch cases are the 8 that start in the ground and 15 that start within 60 m of it, and 3 of the 9 early cases also start within 60 m. Six early cases start with more than 60 m and hit a hill face within 5 s; those are an avoidance question and are not addressed here. The metric `launch_contacts` already keeps the first-step cases out of `late_contacts`, so earlier results were not misreported; but the arms that fly without a map are being compared on aircraft that were placed in the ground.

A launched aircraft is not placed inside a hill: a launch site is a place where the ground is known, and an aircraft leaves it above the ground. That is a property of the setup, not of the controller; no controller sees the scene or reads the true height.

Everything here is a modeling choice, as in ADR 0014.

## Decision

1. **A start clearance.** `swarm.start.clearance_m` (default empty, no change). With it set, a fixed-wing aircraft starts at the larger of its former start altitude and the highest solid surface (terrain or an obstacle) at its start point plus the clearance. The position in x and y does not change, and the start velocity is as before. The brain's launch state follows the start, as it already does when terrain guidance raises it.
2. **It reads the scene in the setup.** The builder reads the scene's surface height at the start point, as a launch site would be known. It does not go through the map, so it is the same with no map, an exact map, or a wrong one, and nothing in `core/sim/**/estimation/` or the brain is given it.
3. **Version and vehicle.** The key needs scenario version 3 and a fixed-wing vehicle; a quad or an older version is refused. It is written to the canonical text only when set, and raises the canonical version to 3 like the other flight keys. With it empty, runs are unchanged to the byte.
4. **The level.** 60 m in the experiment, the planning clearance of the terrain guidance in these scenarios, fixed here and not from the result.

## Experiment

`experiments/fixed_wing_avoid_start.yaml`, with levels and criteria fixed here and committed before any run. The base is `base_fixed_wing_avoid.yaml` with the best avoidance so far (the settings of ADR 0015, `turn_first`, and `catchup_cap_mps` 3). Axes: weather (clear, fog of 375 m visibility), steepness (hills of 40, 400, and 480 m), map (none, exact), and start (`free`, as before; `cleared`, `swarm.start.clearance_m` 60): 24 cells, 30 seeds, 720 runs.

- **Criteria**, `cleared` against `free` as paired differences with 95 percent t intervals over the 30 seeds, applied as written:
  - **Z1.** Launch contacts are 0 for `cleared` in every cell.
  - **Z2.** In fog with no map on 400 and 480 m hills, total ground contacts (`ground_contacts`) lower with an interval entirely below zero.
  - **Z3.** In clear air with no map on 400 and 480 m hills, total ground contacts lower with an interval entirely below zero.
  - **Z4.** On the 40 m control and with the exact map, no ground contacts of `cleared` and a corridor share not lower than `free` by more than 0.02.
  - **Z5.** Reported: late contacts of both arms, so that aircraft which used to be lost at launch and now fly into the hills later are visible, and the costs (corridor share, time on station, radial error, mean height, closest approach, collision episodes).
- **Reported whatever the result:** the crash classes of `scripts/crash_report.py` for fog on 480 m hills with `cleared`, which say how many aircraft are still lost within 5 s of the start with 60 m or more below them.

## Alternatives considered

- **Leave the setup and report the cases separately.** `launch_contacts` already does this and the earlier ADRs used it. It does not remove the early cases that also start close to the ground, and it leaves the no-map arm comparing aircraft some of which began in the ground.
- **Start from the map's idea of the ground.** The map can be wrong by design (the map-error experiments), and the launch site is not what the map is about; a start that depended on the map's error would mix the two questions.
- **Exclude seeds or aircraft in the analysis.** Selecting runs after the fact by their outcome is what the pre-fixed criteria exist to avoid.

## Not in scope

The early crashes with 60 m or more below the aircraft; the range at which a threat is first seen; denser formations; any claim about a real aircraft; making `clearance_m` or the earlier options the default.

## Consequences

If Z1 to Z4 hold, the no-map arm is measured on aircraft that start above the ground, and the previously reported contacts are re-read as a mix of start placement and avoidance. Z2 and Z3 are close to true by construction for the launch cases; what is tested beyond that is Z5, whether the aircraft that now fly survive. If late contacts rise, those aircraft are the cause and the avoidance is the thing to improve next. A null result is reported as one.

## Outcome

Recorded on 2026-10-23 after the experiment ran; the numbers are in docs/RESULTS.md. The criteria were applied as written and all are met. Z1: no launch contacts in any run. Z2 and Z3: total ground contacts of six aircraft with no map fall by 0.27 to 0.63 in fog and clear air on 400 and 480 m hills, every interval below zero (fog, 480 m: -0.57 [-0.86, -0.28]). Z4: no ground contacts on the control or with the exact map. Z5: late contacts do not rise (-0.03 [-0.24, +0.17] in fog on 480 m hills), so the aircraft that used to be lost at launch and now fly are not lost later in greater numbers. In fog on 480 m hills 18 of 180 aircraft are on the ground against 35: 11 of them early, 8 within 2.4 s of the start next to a hill face on their circle 17 to 79 m ahead, which no avoidance can clear at 31 m/s. The option stays off by default, since the earlier results were measured without it; the results of ADRs 0014 to 0019 are re-read as including launch placement. What was not examined: a launch site chosen away from the faces of the generated hills, other clearances, and the late crashes that remain (6 of the 18).
