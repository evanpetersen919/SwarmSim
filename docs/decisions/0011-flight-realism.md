# 0011. Flight realism for the fixed-wing vehicle: wind, energy dynamics, terrain-aware guidance, search patterns

Date: 2026-10-21. Status: accepted.

## Context

ADR 0010 gave the fixed-wing aircraft a coordinated-turn point mass: speed and bank limits, no wind, no energy, instant roll, no terrain knowledge, and one pattern (a circle per aircraft, all starting at tick 0). That was enough to measure formation error under degradation. Looking at the replay in the Electric Dreams level showed what it leaves out:

- The level's ground runs from 41 m to 254 m over the exported kilometer, and the demo flew at 100 m absolute altitude, so aircraft flew through cliffs. Nothing in the core knows the terrain it flies over.
- Speed and heading change as the command says, with no thrust, drag, or load factor, so a turn costs nothing and a climb costs nothing.
- There is no wind, so ground speed equals airspeed and an orbit is a perfect circle.
- Every aircraft starts on its circle at tick 0 and nothing else is flown.

Public sources do not describe the patterns the real aircraft fly. The only formation fact is that launches were spaced 10 to 15 seconds apart (press, EDGE22). The patterns added here are standard fixed-wing search and loiter patterns and are labeled assumptions, not a copy of any real behavior.

## Decision

1. **Scenario version 3** adds the keys below. A file that uses none of them is canonicalized at the version it needs, so every log written before version 3 verifies, and quad logs are byte-identical.
2. **Wind.** `environment.wind` sets a steady wind vector and a gust intensity. Gusts are a sum of a few traveling waves with seeded phases, carried by the mean wind, so every aircraft sees the same air and the field is smooth in space and time. It is a pure function of position, time, and seed, so thread count does not matter. The vehicle flies in the air mass: ground velocity is air velocity plus wind.
3. **Energy dynamics.** `vehicle.dynamics: energy` selects a three-degree-of-freedom point mass with airspeed, heading, flight-path angle, a thrust limit, a drag polar, roll dynamics (a roll-rate limit and a first-order lag), and a stall speed that grows with load factor. Speed trades against altitude: a climb or a hard turn costs speed, and the autopilot adds thrust up to the limit. The default `dynamics: simple` is the model of ADR 0010. Every parameter is assumed or derived from the published mass, wingspan, and speeds, and docs/MODELS.md says which.
4. **Terrain-aware guidance.** The aircraft carries a terrain database, the heightfield of the scene it flies in, treated as prior map data, not a sensor. `control.terrain` sets a clearance above the highest ground in a footprint. The altitude command is raised so that a climb at a fraction of the climb limit clears the highest ground ahead, along the straight line and along the circle or leg it is flying. The map is perfect by default; a map error setting is future work. Obstacles (trees, rocks as boxes and cylinders) are covered by the clearance, not modeled separately. The sim ends a run's aircraft at ground contact and the metrics report minimum height above ground and ground contacts.
5. **Patterns.** `mission.pattern` is `orbit` (ADR 0010, unchanged), `racetrack`, or `lawnmower`, with staggered starts along the pattern. Path following is cross-track and along-track control on a path of straight legs and constant-radius turns, with the phase blend of ADR 0010 along the path.
6. **Viewer.** Pitch comes from the flight-path angle and roll from the logged turn rate; the propeller speed follows airspeed.

## Not in scope

Six-degree-of-freedom aerodynamics, control-surface models, launch and recovery, landing, wind estimation by the aircraft, terrain maps with errors, replicating any real aircraft's behavior, and anything weapon-related (ADR 0010, item 7).

## Consequences

- Results with the energy model, wind, or patterns are not comparable with the ADR 0010 results. The old experiments stay as they were run, with their version 2 scenarios.
- The log format does not change; everything new is in the scenario text in the header. Ground contact and altitude above ground are computed by the metrics from the log and the scene.
- The orbit controller was tuned without wind or energy. Retuning it, and showing that the formation still holds, is part of this work, not assumed.
- The controller headers gain one new header for the terrain map, kept inside `core/sim/include/swarm/control/` so the existing include rule holds.
