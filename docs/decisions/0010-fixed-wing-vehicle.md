# 0010. A second vehicle type: an ALTIUS-class fixed-wing aircraft, behind a vehicle interface

Date: 2026-10-20. Status: accepted.

## Context

Every result so far is for quadrotors: a point mass that can stop and hover, with a formation controller, a separation rule, and a safe-hold fallback that all rely on that. Swarm work that matters for loitering and ISR aircraft concerns fixed-wing vehicles, which cannot hover, must keep a minimum airspeed, and turn by banking.

Public information on the Anduril ALTIUS family is limited. Press reports give the ALTIUS-600 a wingspan of about 2.5 m, a mass of about 12 kg, a cruise speed of about 60 km/h, and a pusher propeller. Stall speed, bank limit, rate limits, engine type, datalink, and the patterns the real aircraft flies are not public. The research notes are kept locally and are not part of the repository.

ADR 0005 already separates a drone's brain from its vehicle. The brain sees estimates and messages and outputs a command; the vehicle owns the truth, the sensors, and the dynamics.

## Decision

1. **Add a vehicle interface** at the seam ADR 0005 made. The quadrotor becomes the first implementation and must reproduce every existing log byte for byte. The scenario file selects the vehicle, which is a new key and a scenario version bump (ADR 0007).
2. **Add a fixed-wing vehicle** as a coordinated-turn point mass with an airspeed range, a bank limit, and a climb-rate limit. It cannot hover. A hold command means orbit at the current position. Wind is not modeled in this version.
3. **The airframe is "ALTIUS-class", not an ALTIUS-700M.** Its parameters live in a file, and every value is marked as published (with a source), taken from the literature, or assumed. The README says that results are for the simulator and for this parameter set.
4. **Add a camera position-fix sensor** for the fixed-wing vehicle. It emits the same position-fix type as the LiDAR sensor, so the estimators do not change. Its noise and availability depend on altitude and weather visibility. Its formulas are assumptions and are labeled so.
5. **Add orbit-based control for the fixed-wing vehicle:** a loiter circle per drone with slot-dependent radius, speed adjustment to hold phase spacing, and a priority-and-speed-corridor separation rule. The fallback state machine keeps its states; safe hold becomes an orbit. This version stops there: no path planning, no wind estimation, no area-search planner.
6. **Keep the quadrotor** as a second vehicle in the benchmark. The comparison is a result, and it shows the interface is real.
7. **Out of scope:** warheads, seekers, targeting, and strike behavior. The study is about coordination, communication, and navigation under degradation.
8. **The log header records the vehicle type and a hash of its parameters.** This is a format change: a version bump and an update to the byte-exact layout test.

## Consequences

- The runtime, the channel, jamming, the socket backend, and the rule that metrics come from logs are unchanged.
- The controllers are the large part of the work. The risk is scope growth, and item 5 is the limit.
- Prediction of a neighbor's motion needs a turn-aware predictor, and the earlier finding about Kalman prediction versus dead reckoning may not hold for fixed-wing.
- The benchmark matrix changes size. The real cell count is computed and recorded before the first run.
- The viewer model is the CC-BY-4.0 model "Anduril ALTIUS 700M" by KillCaptureDestroy. Its credit line goes in the README. The model is not stored in the repository.
