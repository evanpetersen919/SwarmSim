# 0017. Using the whole turn and the lowest speed when the time to a threat is short

Date: 2026-10-23. Status: proposed.

## Context

ADR 0016 left a residual: with avoidance, its fixes, and sharing, some aircraft of six still reach the ground on the steep hills. `scripts/crash_report.py` put the stops in classes from the flown paths (docs/RESULTS.md, Why aircraft still reach the ground). In clear air on 400 m hills 18 of 180 aircraft stopped, 12 of them on the ground at launch and 5 within 4 s of the start, where no avoidance could have acted. In fog on 480 m hills 63 stopped; 23 at launch, 9 within 5 s, and 25 in `other`, the largest group after those that cannot be helped.

The `other` aircraft were heading at the ground with time to turn. From the flown paths of the fog 480 m case:

- Three seconds before the impact they were 68 to 145 m from it, and 6 s before, 152 to 297 m. The impact point was within 11 degrees of the heading.
- Clearing it by climbing needed 3.0 to 5.0 times the climb that was possible, so climbing could not save them; only a turn could.
- Their sideways acceleration over the last 3 s was 1.6 to 4.5 m/s^2, against the 6.2 m/s^2 that the rule may use (0.9 of the vehicle's 6.85). Two of the 25 were at their sideways limit.
- Their speed over the last 3 s was 23 to 49 m/s, mean 36 m/s over the 31 late crashes of that case, against 31.3 m/s on the mission; 77 percent were above 31.3. The radius of a turn at 6.2 m/s^2 is 209 m at 36 m/s and 158 m at 31.3 m/s, so the speed matters as much as the bank. Why the speed is above the mission speed was not examined.

The rule explains the unused bank. In `assess_avoidance` the sideways command toward the chosen free column is `speed * angle / turn_time_s`, clamped to the limit. It depends on the angle and not on the time left: a free column 10 degrees off at 31 m/s gives about 3.6 m/s^2 whether the hill is 300 m away or 80 m. The slow-down is only requested when the time to the threat is less than the angle divided by the turn rate of that command, so an aircraft that is turning too gently for the time it has is not told to slow either.

An earlier version of the diagnosis named a second cause, aircraft taking the same free column and getting in each other's way. That was an error of the script, which counted a neighbor already lying on the ground as a close aircraft; with wrecks excluded the class has 0 aircraft in fog 400 m and 2 of 63 in fog 480 m. It is corrected in docs/RESULTS.md and ADR 0016 and is not a target here.

Everything here is a modeling choice, as in ADR 0014.

## Decision

1. **An urgency time.** `control.avoid.urgent_s` (default 0, meaning off). When a threat is seen and the time to it is less than `urgent_s`, the sideways command is the whole of what the rule may use, `bank_fraction * max_lateral_accel`, in the direction of the chosen column, and the aircraft slows toward its minimum speed plus margin as it does today when it is told to slow. Bang-bang is the time-optimal way to change heading under a bound on the sideways acceleration, and the proportional command is a smooth approximation that is right when there is time. When the time to the threat is `urgent_s` or more, the command is the proportional one, unchanged, so the effect is confined to the case the diagnosis found. The time to the threat is the existing one: the least time over the points of the column nearest the heading (`time_to_threat`).
2. **Why a time and not an angle.** The time is what an aircraft lacks, and it is already computed. A rule on the angle would be tied to the fan's column spacing. `urgent_s` is one assumed number; its default for the experiment is 6 s, because the crashes were 152 to 297 m from the impact point 6 s before they happened and 68 to 145 m 3 s before, so a threat with less than 6 s left is the one a gentle turn has stopped being able to resolve. It is swept at 4, 6, and 8 s.
3. **Nothing else changes.** The choice of column, the rejoin preference, the hold, the vertical clamp, and the all-blocked rule are as in ADR 0015. With `urgent_s: 0` runs are unchanged to the byte; the canonical text writes the key only when it is not 0, in the same scenario version as `control.avoid`, and the key is validated to be at least 0.
4. **A measure for what the rule does to speed.** None is added. `mean_agl_m`, `time_on_station`, `orbit_corridor_share`, `orbit_radial_rmse_m`, and `min_separation_m` already show the cost; the speed is in the positions log.

## Experiment

`experiments/fixed_wing_avoid_urgent.yaml`, with the levels and the criteria fixed here and committed before any run. The base is `base_fixed_wing_avoid.yaml` with the all-three settings of ADR 0015 (the arm `all3`: rejoin, kinematic feasibility, clearance 30 m) and no sharing. Axes: weather (clear, fog of 375 m visibility), steepness (hills of 40, 400, and 480 m), map (none, exact), and arm (`all3`, `urgent4`, `urgent6`, `urgent8`): 48 cells, 30 seeds, 1,440 runs. The exact-map level is where every trigger is a false alarm, so it measures what the rule costs when nothing is in danger.

- **Safety measures.** `late_contacts` (ground contacts after launch), `ground_contacts`, `launch_contacts`, `first_ground_contact_s`, `min_agl_m`. **Cost measures.** `orbit_corridor_share`, `time_on_station`, `orbit_radial_rmse_m`, `mean_agl_m`, `collision_episodes`, `min_separation_m`.
- **Criteria**, applied as written to the arm `urgent6` against `all3` as paired differences with 95 percent t intervals over the 30 seeds. `urgent4` and `urgent8` are reported and do not enter the criteria, so the default is not chosen by the result.
  - **U1.** In fog with no map on 400 and 480 m hills, `urgent6` has late contacts lower with an interval entirely below zero.
  - **U2.** On the 40 m control and with the exact map, `urgent6` has no late contacts and a corridor share not lower than `all3` by more than 0.02.
  - **U3.** In clear air with no map on 400 and 480 m hills, `urgent6` has late contacts not higher beyond the paired interval (an interval entirely above zero is a harm).
  - **U4.** Collision episodes of `urgent6` against `all3` over every cell: no increase whose interval is entirely above zero. The turn is harder and the speed lower, which draws the aircraft toward one another; this is where that would show.
  - **U5.** The costs (corridor share, time on station, radial error, mean height) are reported for all cells, with the sweep over `urgent_s`.
- **Reported whatever the result:** that the all-blocked and early crashes are not addressed (they are in `late_contacts` as before), and the crash classes of the script run on `urgent6` against `all3` for fog 480 m, so that the claim about the `other` class is tested and not assumed.

## Alternatives considered

- **Treat the proportional command as the problem and raise the gain** (a shorter `turn_time_s`). It scales every turn, including those with time to spare, and costs the formation in the cases that are not urgent. The time-conditioned rule leaves them alone. `turn_time_s` is not swept here.
- **Plan the turn as a path** (a Dubins or lateral-displacement solution that clears the width of the blocked region by a given time). This is the better model, and it needs the lateral extent of the obstacle, which the 7-column fan gives only coarsely. It is a larger change to the sensing model and is a candidate if the result of this one shows that the bank was not the limit.
- **A fixed speed cap in avoidance.** It would lower the radius for every threat. Not tried; the slow-down is already requested in the cases where the turn is too gentle, and this ADR makes that request apply in the urgent case.
- **Separation de-confliction in the choice of column.** Not supported by the diagnosis (see Context).

## Not in scope

The launch altitude taken from a wrong map; recovering the phase after a detour; more than one faulty sender; the effect of the navigation error in fog on shared points; why the speed is above the mission speed at the impact; a picture-in-picture camera; any claim about a real aircraft.

## Consequences

If U1 to U4 hold, the avoidance has a time-aware turn and the residual in fog shrinks by some amount the experiment reports; if U1 fails, the unused bank was not the limit, and the next step is the path-planning alternative above or a look at the sensing (the fan's coverage and the points' height). If U2 or U3 fail, the cost of a harder turn is where it is not needed, and the rule needs a narrower trigger. A null result is reported as one, as before.
