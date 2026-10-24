# 0019. Limiting the catch-up speed after a detour

Date: 2026-10-23. Status: accepted.

## Context

ADR 0018 let the turn use the acceleration it commands. In fog on 480 m hills the aircraft on the ground fell from 63 to 49 of 180, and the aircraft that turned at their limit and still hit the ground rose from 2 to 8. Those 8 are limited by something other than the turn. In the traced flight of ADR 0018 the threat is first seen about 3.4 s ahead at 39 m/s, and a turn at 6.17 m/s^2 has a radius of 247 m at that speed. The late crashes of the diagnosis (docs/RESULTS.md) averaged 36 m/s over their last 3 s, against the mission speed of 31.3 m/s, and 77 percent of them were above it.

Where the speed comes from, from the rule in `orbit_guidance` and not from a run: the desired speed is `clamp(speed + phase_gain * radius * phase_error, min + margin, max - margin)`, with `phase_gain` 0.05 per second and radius 300 m, so each radian of phase the aircraft is behind asks for 15 m/s more, up to the vehicle's limit (51 m/s here, less the margin). A detour to avoid a hill puts an aircraft behind its slot on the circle, and the guidance then asks for the speed to catch up. The warning time of a fixed sensor range falls with speed (the effective range in fog is about 200 m: 6.4 s at 31.3 m/s, 5.1 s at 39 m/s) and the turn radius grows with its square. So the catch-up that follows one detour makes the next one harder. That is a reading of the rule; whether it is the cause of the 36 m/s is what the experiment tests.

A probe (`experiments/probe_fixed_wing_phase.yaml`, 10 seeds, fog, no map, the avoidance of ADR 0015 with `turn_first`) varied only the phase gain. Late contacts of six aircraft on 480 m hills were 1.00 at the gain of 0.05, 0.60 at 0.02, and 0.50 at 0; on 400 m hills 0.20 for all three. The closest approach was 94, 93, and 91 m. Ten seeds do not show a difference, and a gain of 0 is not a rule: an aircraft ahead of its slot should still be able to slow down, and one behind should still be able to close the gap in time.

Everything here is a modeling choice, as in ADR 0014.

## Decision

1. **A cap on the speed the phase correction may ask for above the commanded speed.** `control.avoid.catchup_cap_mps` (default empty, no cap). With it set, the desired speed of `orbit_guidance` is `clamp(speed + phase_gain * radius * phase_error, min + margin, min(max - margin, speed + cap))`. Slowing down to correct the phase is not capped, and the phase gain is not changed, so an aircraft ahead of its slot behaves as before. The cap is on the commanded speed (the mission speed on its circle), not on the speed the aircraft is flying.
2. **Why a cap and not a lower gain.** A lower gain slows every correction, in both directions and in the cases with nothing to avoid; a cap bounds the one thing the diagnosis points at, the speed above the mission speed. The price is a slower recovery of the phase, which the experiment measures.
3. **Where the key lives.** Under `control.avoid`, so it needs `sensors.forward` and scenario version 3 like the rest of avoidance, and it is applied by the scenario builder to the orbit parameters of the brain. Without avoidance nothing changes. It is validated as finite and not negative, and written to the canonical text only when set. With it empty, runs are unchanged to the byte.
4. **Levels.** The primary arm has a cap of 3 m/s (about a tenth of the mission speed, so at most 34.3 m/s commanded). It is fixed here, before the experiment, from the diagnosis (the late crashes were at about 5 m/s above the mission speed on average) and not from the result. A cap of 0 (no catching up above the mission speed) and of 6 m/s are swept and reported.

## Experiment

`experiments/fixed_wing_avoid_catchup.yaml`, with the levels and criteria fixed here and committed before any run. The base is `base_fixed_wing_avoid.yaml` with the settings of ADR 0015 and `turn_first` (the arm `turnfirst` of ADR 0018). Axes: weather (clear, fog of 375 m visibility), steepness (hills of 40, 400, and 480 m), map (none, exact), and arm (`base`, `cap6`, `cap3`, `cap0`): 48 cells, 30 seeds, 1,440 runs.

- **Criteria**, for the arm `cap3` against `base` as paired differences with 95 percent t intervals over the 30 seeds, applied as written:
  - **X1.** In fog with no map on 400 and 480 m hills, late contacts lower with an interval entirely below zero.
  - **X2.** On the 40 m control and with the exact map, no late contacts and a corridor share not lower than `base` by more than 0.02.
  - **X3.** In clear air with no map on 400 and 480 m hills, late contacts not higher beyond the paired interval (an interval entirely above zero is a harm).
  - **X4.** Collision episodes over every cell: no increase whose interval is entirely above zero. A cap leaves aircraft bunched on the circle longer, so this is where a harm would show.
  - **X5.** The costs are reported for all cells with the sweep over the cap: corridor share, time on station, radial error, mean height, closest approach.
- **Reported whatever the result:** the crash classes of `scripts/crash_report.py` for fog on 480 m hills with `base` and `cap3`, and the mean speed of the aircraft over their last 3 s in each class, since this ADR claims that the speed is what limits the warning.

## Alternatives considered

- **Lower the phase gain.** Slower in both directions and in all scenarios; see the Decision.
- **Stop catching up while the avoidance is armed.** It would remove the speed only for the seconds of the hold and leave the aircraft fast right after it, when the next hill may come.
- **A speed cap from the sensor range** (speed no higher than the range divided by the time a turn needs). The better rule in principle, and it needs a model of the sensor's range in the weather in the controller. Not tried; if the result of this ADR shows the speed is the limit, it is the next candidate.

## Not in scope

The range at which the threat is first seen in fog; the aircraft that stop within seconds of the start; the launch altitude taken from a wrong map; the path-planning alternative of ADR 0017; more than one faulty sender; any claim about a real aircraft.

## Consequences

If X1 to X4 hold, the speed after a detour is part of the residual, and the cap is a cheap way to take it. If X1 fails, the speed above the mission speed is not what limits the warning, and the next steps are the sensing range and the early crashes. If X4 fails, the cap costs the spacing, and the better rule is the one tied to the sensor range. A null result is reported as one.

## Outcome

Recorded on 2026-10-23 after the experiment ran; the numbers are in docs/RESULTS.md. The criteria were applied as written. X1 is met on 480 m hills (late contacts in fog with no map -0.47 [-0.72, -0.21]) and not on 400 m hills (-0.07 [-0.16, +0.03]), so it is not met as written, as was V1 of ADR 0018. X2, X3, and X4 are met. The cost is a smaller closest approach (3 to 13 m at a cap of 3 m/s in the cells examined) and, at a cap of 0, less time on station. The crash classes in fog on 480 m hills: 35 aircraft of 180 on the ground against 49, of which 32 are launch and early cases; the late ones were at a mean of 35.9 m/s with `base` and 23.5 m/s with `cap3`. The option stays off by default because the earlier results were measured without it. What was not examined: the aircraft that stop within seconds of the start, which are now nearly all of what remains in this case; the range at which the threat is first seen; and the better rule in principle, a cap tied to the sensor range.
