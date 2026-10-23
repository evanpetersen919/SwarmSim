# 0014. Avoiding terrain and obstacles by turning and slowing, driven by the forward sensor

Date: 2026-10-23. Status: proposed.

## Context

ADR 0013 gave the fixed-wing aircraft a forward obstacle sensor and measured what it does when the prior map is wrong (docs/RESULTS.md). It keeps most aircraft off the ground, and it cannot keep them all off, for a reason built into the guidance: every avoidance the aircraft has changes only its altitude command. Terrain guidance, the radar altimeter, and the forward sensor each raise the command, and the vehicle then climbs at no more than its climb limit (3 m/s for the default parameters, and less under the energy model at speed). Ground that rises faster than the aircraft can climb is reached whatever the sensor sees, and ADR 0012 and ADR 0013 both list turning or slowing down as not built.

What the vehicle can do sideways is much more than what it can do upward. A coordinated turn at the bank limit gives `g * tan(max_bank)` of sideways acceleration (`FixedWingParams::max_lateral_accel`), 6.85 m/s^2 at the 35 degree default, and a turn radius of `v^2 / a`: 42 m at 17 m/s, 91 m at 25 m/s, and 143 m at 31.3 m/s (70 mph, the speed of the ALTIUS-700 demo). Climbing 60 m at 3 m/s takes 20 s, which at 17 m/s is 340 m of flight and at 31.3 m/s 626 m. The sensor's default range is 800 m. So where an obstacle is seen early enough, a turn clears it by a wide margin and a climb may not.

What is not known. In the 1,920-run experiment of ADR 0013, about 3 of twelve aircraft are lost at -80 m and with an omitted square whatever the sensor does, and 1.9 per run of those are on the ground half a second in, because they launch below the real ground from a start altitude taken from the wrong map. The other 1.2 per run were not examined, so it is not shown how much of the remaining loss is the climb limit and how much is something else. The scene export of the Electric Dreams level was also found to be wrong in ways that made some clearance results wrong (docs/MODELS.md, Audit), and was fixed; experiments for this decision must not depend on an exported scene, which is not in the repository.

Everything about the maneuver is a modeling choice. Public sources say nothing about the ALTIUS aircraft's avoidance behavior, so the rule below is a simple, readable one and not a description of any real autopilot.

## Decision

1. **Climbing alone is judged feasible or not, from the sensor's own returns.** For each detected return the brain has the point it implies (ADR 0013, decision 6), the horizontal distance `d` to it, and the time to reach it, `t = d / speed`. The altitude needed to clear it is the point's height plus the clearance; the climb rate needed is `(needed - altitude) / t`. A return is *unclearable* when that rate is above the climb budget, `climb_fraction * climb_rate` of the terrain guidance parameters, the same quantity the altitude floor is built from, and `t` is below a horizon (`control.avoid.horizon_s`). A return that climbing can clear is left to the altitude floor, as now.
2. **Which way to turn comes from the fan's columns.** The fan has `azimuth_rays` columns. A column is blocked if any ray in it has an unclearable return. The aircraft turns toward the free column nearest its heading (ties go to the right, as the separation rule's head-on case does), by commanding a sideways acceleration toward that column's azimuth, limited to `bank_fraction` (default 0.9) of the vehicle's maximum lateral acceleration. It only turns toward columns the sensor can see: it does not turn into the part of the sky it is blind to.
3. **Slowing helps the turn and the time.** While a return is unclearable and the time to it is shorter than the time the turn needs (the heading change over the turn rate `a_lateral / speed`), the speed command is lowered toward the vehicle's minimum speed plus its margin. A lower speed shrinks the turn radius and lengthens the time, and the vehicle's own limits clamp the result.
4. **When every column is blocked,** the aircraft turns away from the column whose nearest unclearable return is farthest, at full bank, at minimum speed, with the altitude floor still applying. This is the last resort and the case most likely to still end in contact; the experiment counts it.
5. **Priority and hand-off.** The avoidance acceleration replaces the orbit tracking term while it is active; separation from the other aircraft keeps its priority within the acceleration limit, as in orbit guidance. Avoidance stays on until no unclearable return has been seen for `control.avoid.hold_s`, then the orbit guidance takes the aircraft back to its circle, which it does from any position already. The altitude floors are unchanged and apply throughout.
6. **Scenario keys.** `control.avoid: {horizon_s, hold_s, bank_fraction}`, written to the canonical text only when used, in the scenario version of the other flight keys. It needs `sensors.forward`, because it acts on the sensor's returns; a scenario that sets it without the sensor is refused, as an altimeter or sensor key is refused for a quadrotor.
7. **Metrics.** Two new columns computed from the log and the scene as `min_agl_m` is: `launch_contacts`, the number of aircraft at or below the surface in the first record that holds drone state, and `first_ground_contact_s`, the time of the first record in which any aircraft is at or below the surface (empty when none is). They tell contacts at launch from contacts later, which the experiment of ADR 0013 could not. The binary log is unchanged.
8. **The brain's avoidance state** is not logged, as the sensor's output is not (ADR 0013, decision 8). The share of time spent avoiding cannot be computed from the log in this step.
9. **Estimator rule.** Unchanged: the forward sensor is not fused into the pose estimator.

## How it will be judged

A probe first, then an experiment with its levels fixed and committed before the first run, 30 seeds per cell, paired by seed, on generated scenes only (the scene file of a real level is not in the repository, and an export is only as good as its audit).

- **Test bed.** `scene: hills` with the amplitude and feature scale chosen so that the slope is above the climb limit in places. The probe finds the amplitudes at which aircraft without any avoidance reach the ground: none, some, all of them. A second test bed has the map wrong, as in ADR 0013, so the sensor has something to correct.
- **Arms.** No sensor, forward sensor (ADR 0013, the altitude floor only), forward sensor with avoidance.
- **Levels.** Terrain steepness at three levels bracketing the probe; map exact and map wrong (an offset and an omitted rectangle); clear air and fog that cuts the sensor's detection distance to the range where a turn at 31.3 m/s is still possible (about 200 m), so the experiment reaches the sensor's limit, which the fog of ADR 0013 did not.
- **Safety measures.** `ground_contacts`, `launch_contacts` (so that contacts after launch are `ground_contacts - launch_contacts`), `first_ground_contact_s`, `min_agl_m`.
- **Cost measures.** `time_on_station`, `orbit_radial_rmse_m`, `orbit_altitude_rmse_m`, `mean_agl_m`, `collision_episodes` and `min_separation_m`, because a turn that ignores the formation can end in a collision, which is a worse result than the contact it avoided.
- **The exact-map, gentle-terrain control** shows whether avoidance does harm where nothing needs avoiding.
- **A sensitivity sweep** of `horizon_s` and `bank_fraction` (the two assumed numbers of the rule), since a result that holds for one setting only is not a result.

Report paired differences with 95 percent intervals, null results included, and say which cells avoidance cannot save, which is what decides whether anything planning-like is the next decision.

## Alternatives considered

- **A three-dimensional planner** (sampling-based or model-predictive). It would use the sensor better and would put many assumed parameters, and a planner whose behavior no reader can follow, between the sensor and the result. The rule above is one a reader can check by hand; if it shows a clear remaining gap, a planner is the next decision and has a baseline to beat.
- **A potential field** that pushes the aircraft away from every return. Simple, and it fails in the usual way, by pushing into a local minimum in front of a wall; the column rule at least names the case where it has no answer (decision 4).
- **Speed only.** An aircraft cannot stop (its speed has a floor), so slowing alone only delays the contact.
- **Sharing detections with the other aircraft** so that one's sighting warns the rest. It fits the project's question about communications better than anything here, and it needs this decision's avoidance to act on what it receives, so it is the decision after this one.

## Not in scope

Launch altitude (an aircraft that starts below the real ground because its start altitude comes from the wrong map; it is counted by `launch_contacts` and is its own fix), planning around the whole orbit, cooperative sensing, a sensor with more rays or a wider fan, avoidance of other aircraft beyond the separation rule that exists, a quadrotor version, and any claim about the real aircraft.

## Consequences

- Avoidance can break the formation: an aircraft that leaves its circle to dodge a ridge is off its slot until it re-acquires. `time_on_station` and the orbit errors measure the cost, and `collision_episodes` the worst case.
- A turn needs room: at 31.3 m/s the turn radius is 143 m, so an obstacle first seen closer than that, or seen only by a fan that does not cover the direction of the turn, cannot be avoided by this rule. Narrow-fan and fog levels in the experiment exist to show this.
- Terrain guidance gains a lateral output and a mode, so its tests must cover the choice of column, the all-blocked case, the hand-off back to the orbit, the hold time, a return that climbing can clear (no turn), a dropout in the middle of a maneuver, and separation taking priority.
- Runs with `control.avoid` are not comparable with runs without it. Runs without it are unchanged, and the new metric columns add two columns to the metrics CSV (35 metric columns), which changes the column count that some tests check.
- All the values of the rule, and the rule itself, are modeling choices. The results say what such a rule would do in this simulator, not what the real aircraft does.
