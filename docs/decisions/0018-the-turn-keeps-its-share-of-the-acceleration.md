# 0018. The turn keeps its share of the acceleration limit while slowing

Date: 2026-10-23. Status: proposed.

## Context

ADR 0017 asked whether using the whole bank when time is short keeps more aircraft off the ground. It did not (docs/RESULTS.md, The whole turn when time is short): late contacts in fog changed by -0.03 [-0.35, +0.28] and -0.13 [-0.45, +0.19]. The crash classes did not move either, and the aircraft that reached the ground were still turning at about 1.5 to 3 m/s^2 in their last seconds although the rule commanded 6.17.

To find out why, a temporary trace (not committed) printed what the brain decided each tick in one such flight: fog, 480 m hills, `urgent6`, seed 3, drone 0, which stopped on the ground at 75.7 s. The facts, from that trace and the flown positions:

- The threat is first seen 3.4 s before the impact (about 130 m at 39 m/s). The brain then commands a sideways acceleration of -6.17 m/s^2 (0.9 of 6.85) at every step from 3560 to 3770, and `slow` is set.
- The achieved sideways acceleration over the same steps, from the flown positions, is -1.7 to -3.4 m/s^2, about half the command.
- The command that reaches the controller is `along = -17.3 to -15.6`, `sideways = -6.17`, `vertical = +2.0` m/s^2. The slow-down command is `-(speed - target)` with no bound: at 39 m/s against a target near 22 m/s it asks for 17 m/s^2 of braking, more than the brain can give.
- The brain's final step is `combine_accel(track, push, max_accel)`, which limits the norm of the whole command to `max_accel` (10 m/s^2 in this scenario). A command of norm 18.5 against 10 is scaled by 0.54, and 0.54 times 6.17 is 3.3 m/s^2, which is the achieved turn.

So the slow-down term takes the share of the limit that the turn needs, and it does so in exactly the case the rule calls slow: when time is short. This is the same kind of fault as the climb that took the turn's share in ADR 0014, which was fixed for the vertical command only (the vertical command is clamped to the vehicle's vertical limit while avoiding). It has been in the rule since ADR 0014 and it affects every avoidance result so far in the cases where `slow` was set. Those results stand as measurements of the rule as built; this ADR changes the rule.

The speed also matters (the late crashes were at 36 m/s on average, against 31.3 on the mission) but is not part of this decision.

Everything here is a modeling choice, as in ADR 0014.

## Decision

1. **A priority for the turn.** `control.avoid.turn_first` (default false). While avoiding, the along-track component of the command (the braking) is limited so that the three components together are within the brain's acceleration limit `A` (the one `combine_accel` applies) with the sideways and the vertical components first: `|along| <= sqrt(max(0, A^2 - sideways^2 - vertical^2))`. The sideways and vertical components are as they are today (the vertical is already clamped to the vehicle's vertical limit). Nothing else changes: the trigger, the choice of column, `urgent_s`, the hold. If `sideways^2 + vertical^2` is at least `A^2` the braking is zero. With the option off the command is as in ADR 0017, byte for byte.
2. **Why this and not a lower braking target or a larger limit.** The limit is a fact of the brain's acceleration budget, and the order in which it is spent is the rule's choice. Turning is what keeps the aircraft off the hill, and braking helps by shrinking the radius and lengthening the time; when they compete, the turn should win. The braking is not removed, only bounded by what is left. A higher `A` would change every other behavior.
3. **The key** is validated as a boolean, written to the canonical text only when true, in the same scenario version as `control.avoid`.

## Experiment

`experiments/fixed_wing_avoid_turnfirst.yaml`, with the levels and criteria fixed here and committed before any run. The base and the axes are those of `fixed_wing_avoid_urgent.yaml` (weather clear and fog; hills of 40, 400, and 480 m; map none and exact) with three arms, all with the settings of ADR 0015 (`all3`): `all3` (as before), `turnfirst` (`turn_first: true`), and `turnfirst_urgent6` (`turn_first: true` and `urgent_s: 6`): 36 cells, 30 seeds, 1,080 runs. The primary comparison is `turnfirst` against `all3`.

- **Criteria**, as paired differences with 95 percent t intervals over the 30 seeds, applied as written:
  - **V1.** In fog with no map on 400 and 480 m hills, `turnfirst` has late contacts lower with an interval entirely below zero.
  - **V2.** On the 40 m control and with the exact map, `turnfirst` has no late contacts and a corridor share not lower than `all3` by more than 0.02.
  - **V3.** In clear air with no map on 400 and 480 m hills, `turnfirst` has late contacts not higher beyond the paired interval (an interval entirely above zero is a harm).
  - **V4.** Collision episodes over every cell: no increase whose interval is entirely above zero.
  - **V5.** The costs (corridor share, time on station, radial error, mean height, closest approach) are reported for all cells, and `turnfirst_urgent6` against `turnfirst` is reported to say whether the whole-bank rule adds anything once the turn is no longer squeezed. It is not a criterion.
- **Reported whatever the result:** the crash classes of `scripts/crash_report.py` for fog on 480 m hills with `turnfirst`, and the sideways acceleration achieved by the `other` class in their last 3 s, which is the quantity this ADR claims to have been too low.

## Alternatives considered

- **Bound the braking by the vehicle's own deceleration limit.** The vehicle model already limits what it does with the command; the fault is in the brain's own norm limit, which scales the sideways part. A limit in the rule that mirrors the vehicle would need the vehicle's numbers in the controller, which the scenario builder gives it only for the lateral and vertical limits.
- **Fix it without an option.** It would change the results of ADRs 0014 to 0017 under the same names and make the earlier numbers unreproducible. The option keeps them reproducible and lets the experiment measure the change.
- **Slow earlier and gently.** A different rule; the trace shows the aircraft first sees the threat 3.4 s out, and what is asked of this decision is only that the turn be made as commanded.

## Not in scope

Why the speed is above the mission speed at the impact; the range at which the threat is first seen in fog; the path-planning alternative of ADR 0017; the launch altitude taken from a wrong map; recovering the phase after a detour; more than one faulty sender; any claim about a real aircraft.

## Consequences

If V1 to V4 hold, the rule has the turn it was meant to have and the earlier results are re-read as a lower bound on what the rule can do. If V1 fails with the achieved turn now at the command, the limit is the sensing (3.4 s of warning at 39 m/s is short for a radius of 247 m at 6.17 m/s^2), and the next steps are the speed and the range, not the turn. A null result is reported as one.
