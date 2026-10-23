# 0015. Rejoining the orbit after avoiding, and judging a threat against what the vehicle can climb

Date: 2026-10-23. Status: proposed.

## Context

ADR 0014 built avoidance by turning and slowing, and measured it (docs/RESULTS.md). It keeps most of the remaining aircraft off the ground where climbing cannot, and it costs the formation: on 400 m hills with no map the late ground contacts of six aircraft go from 2.03 to 0.30, while time on station goes from 0.30 to 0.07 and the radial error to the orbit from 12 to 202 m. With an exact map, where nothing was in danger, it still costs the formation (time on station 0.71 to 0.89 against 0.94 to 1.00 with the forward sensor alone). The outcome section of that ADR names two causes to look at, and this ADR looks at them.

What the flown path shows. A trace of three aircraft in one run (400 m hills, no map, seed 1, avoidance on; positions from `swarm_sim positions`) shows that the aircraft do come back to the orbit, but only after wide excursions. Their distance from the orbit center swings between about 300 m and 500 to 770 m and back, with a period of about 50 s: drone 1, for example, is at 303, 451, 432, 552, 589, 769, and 364 m at 0, 18, 27, 36, 48, 63, and 84 s. All of them also climb from 100 m to about 200 m, which is the altitude floor and not the turning. So the formation is not lost for good, as the results section guessed: it is torn open by a series of detours, and the aircraft are never at their own place on the circle for long.

Why each detour is large, from the rule and the geometry. Three things in the rule of ADR 0014 add to it:

- **The tie goes outward.** When the free columns on both sides are equally near the heading, the aircraft turns right. The orbit is counterclockwise, so the center is on the left and right is away from it. Every tie dodges outward, and the orbit then has to bring the aircraft back in against a hill it is still near.
- **The free column is chosen nearest the aircraft's heading, not nearest where the orbit wants it to go.** An aircraft on a circle that must dodge a hill ahead picks the smallest turn, which is not the turn that keeps it on the circle.
- **A return is judged against the climb budget, not the climb the vehicle has.** The budget is 0.6 of the climb limit (`climb_fraction` of the terrain guidance, set for planning with margin). A return that climbing at the full limit could still clear is called unclearable, so the aircraft turns away from ground it could have climbed over. With an exact map this is every trigger there is, and it is the false-alarm cost in the results.

The first two make a detour larger than it needs to be; the third makes there be detours that are not needed. The phase of the aircraft on its circle (the orbit reference is a function of time) is lost during any detour, and the aircraft then catches up with a speed adjustment that the vehicle's speed limits bound, which is why even a short detour costs time on station; this ADR does not change that.

Everything here is a modeling choice, as in ADR 0014.

## Decision

1. **A rejoin preference for the choice of column.** For a circle pattern (an orbit), the brain knows the heading that the orbit guidance would ask for now: the circle's tangent, cut toward the circle in proportion to the radial error, as in `orbit_guidance`. When `control.avoid.rejoin` is on, the free column is the one whose direction is nearest that heading, not the one nearest the aircraft's own; a tie goes toward the center of the orbit and not to the right. The trigger is unchanged: the column nearest the aircraft's own heading is blocked. For the patterns that are not a circle (racetrack, lanes, formation) there is no single such heading and the rule of ADR 0014 stays as it is, and a scenario that sets `rejoin` with one of them is refused.
2. **A climb fraction of its own for the judgment** (0.9 is tried in the experiment, not the full limit). `control.avoid.climb_fraction` is the part of the vehicle's climb limit that the judgment of an unclearable return may count on, in (0, 1]. The default is the terrain guidance's `climb_fraction`, 0.6, so that a scenario that sets neither new key behaves exactly as under ADR 0014. The vehicle's climb limit is the one the scenario builder already gives the terrain guidance (the aircraft's maximum, or what its energy model sustains at the mission speed, whichever is less).
3. **Both are options that default to the old behavior.** Runs without them are unchanged to the byte, as for every option so far; the canonical text writes the two keys only when they differ from the defaults. They are written in the same scenario version as `control.avoid`.
4. **The hold time stays at its default.** It is already a parameter of ADR 0014 and its sensitivity was measured there; a shorter horizon cut the formation error to a third at no loss of safety, and this ADR does not change the defaults of that rule. The experiment includes the 6 s horizon as an arm so that the two kinds of change can be told apart.
5. **No new log content.** The binary log and the metrics are unchanged. The cost of a detour is read from `orbit_radial_rmse_m`, `time_on_station`, and `mean_agl_m`, as before.
6. **Estimator rule.** Unchanged.

## How it will be judged

The same scenario, steepness levels, and measures as ADR 0014, so that the new arms can be compared with its results directly, with the levels fixed and committed before the first run, 30 seeds per cell, paired by seed.

- **Arms.** The forward sensor alone (the baseline of ADR 0014); avoidance as built (`avoid`); avoidance with the rejoin preference only; with a climb fraction of 0.9 only; with both; and, to separate the horizon, avoidance as built with a 6 s horizon.
- **Levels.** Hills of 40, 240, 400, and 480 m; the map none and exact; clear air and the fog of 375 m visibility. Sixteen combinations of steepness, map, and weather for each of the six arms: 4 x 2 x 2 x 6 = 96 cells, 2,880 runs.
- **What would count as better, fixed now.** The new setting is better than the one built if, on the steep hills with no map (400 and 480 m, clear air), its late contacts are not higher than those of `avoid` by more than the paired interval allows (an increase whose interval is above zero is a loss of safety), and its radial error and time on station are better with intervals that do not include zero; and, with an exact map, if its time on station is at least 0.90 at every steepness level (the forward sensor alone has 0.94 to 1.00). A setting that is better on one of these and worse on another is reported as that.
- **Measures.** As in ADR 0014: `late_contacts`, `ground_contacts`, `launch_contacts`, `min_agl_m`, and `first_ground_contact_s` for safety; `time_on_station`, `orbit_radial_rmse_m`, `mean_agl_m`, `collision_episodes`, and `min_separation_m` for cost.
- **A null result is a result.** If the rejoin preference changes nothing, the cause of the wide excursions is something else, and the next step is a closer look at the path (the trace above is for one run), not a third rule.

## Alternatives considered

- **A stronger pull back to the circle after a detour** (a larger radial gain while the aircraft is off the circle after avoiding). It would shrink the excursions at the cost of bank, and the aircraft would turn back toward the hill sooner; it is a gain to tune, and the choice of column is the part that sets where the detour goes. It can follow if this is not enough.
- **A different tie-break only** (toward the center). It is the cheaper half of decision 1 and is what the experiment's `rejoin` arm would show if the column choice by the orbit heading adds nothing; the two are not separated, because the tie is rare when the choice is by the orbit heading, and adding an arm for it would double the cells without a different question.
- **Dropping the hold.** The hold keeps the heading for 3 s after the last threat so that the aircraft clears the ground before the orbit turns it back; removing it would be a change to ADR 0014's rule rather than to the return.
- **Planning the whole return** (a path from the aircraft to its slot that avoids the sensed ground). It is a planner, and the question of a planner is left until a simple rule shows what is left to gain.
- **Sharing detections between aircraft.** Still the decision after these, and it needs avoidance that works.

## Not in scope

Racetrack, lanes, and formation patterns (the rule of ADR 0014 stays for them), the aircraft's phase on its circle after a detour, launch altitude, the altitude floor's own climb to about 200 m, a sensor with more rays or a wider fan, a quadrotor version, and any claim about the real aircraft.

## Consequences

- The brain computes the heading the orbit asks for in a second place; a test must show that it agrees with `orbit_guidance`'s own direction, so the two cannot drift apart.
- A rejoin preference can turn the aircraft toward ground that the old rule would have turned it away from: the free column nearest the orbit's heading is free, not safe in general, because the sensor sees only its fan. The all-blocked case and the trigger are unchanged and keep that guard, and the experiment counts the contacts that this could add.
- A full climb fraction takes away the margin of 0.4 of the climb limit: an aircraft that judges a return clearable at the full limit and then cannot climb as fast (the energy model's thrust at that speed, a gust, the vehicle's own lag) will reach it. The arm at 0.9 is in the experiment so that this is seen; the full limit (1.0) is not tried, because it has no margin at all.
- Two new keys under `control.avoid`; scenarios without them are unchanged, and the canonical text of a scenario that sets them to their defaults is the same as one that sets neither.
- Everything about the rule is a modeling choice. The results say what it does in this simulator, not what the real aircraft does.
