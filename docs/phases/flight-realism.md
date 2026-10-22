# Flight realism for the fixed-wing vehicle

Decision record: [0011](../decisions/0011-flight-realism.md). Models: [MODELS.md](../MODELS.md), sections Wind, Fixed-wing energy model, Terrain-aware guidance, and Patterns.

## Why

The first fixed-wing results (ADR 0010) came from a point mass with no wind, no energy, instant roll, no knowledge of the ground, and one pattern. Playing a run back in the Electric Dreams level showed the gaps: aircraft flew through cliffs, turns and climbs cost nothing, and every aircraft sat on a circle from the first tick. The work below closes those gaps. It does not copy any real aircraft's behavior: the sources do not describe it.

## What was built

| Part | Where | What it does |
|---|---|---|
| Wind | `wind.hpp` | A steady wind and a gust field of eight traveling waves, a function of position, time, and seed |
| Energy model | `fixed_wing_energy.hpp` | Thrust, drag polar, roll-rate limit and lag, bank limited by stall, stall below the stall speed, climb limited by the thrust left |
| Terrain map and guidance | `control/terrain_map.hpp`, `drone_brain.cpp` | The aircraft carries the scene's heightfield with obstacle tops raised into it, and raises its altitude to clear the ground along its path |
| Patterns | `control/path.hpp` | A stadium path (racetrack, parallel lanes), path following, staggered starts |
| Ground contact | `drone_agent.cpp` | With `environment.ground_contact_crashes`, an aircraft that reaches the ground stops there |
| Metrics | `metrics.cpp` | Cross-track and along-path error for patterns, `min_agl_m`, `ground_contacts` |
| Viewer | `ue5-plugin` | Pitch from the flight-path angle, a faster propeller, a state lamp, cameras sized to the swarm |

Everything is opt-in through scenario version 3. A scenario that uses none of it is written as version 1 or 2, so every earlier log verifies and the quad is unchanged.

## What a run shows

`scenarios/fixed_wing_flight.yaml`: eight aircraft on a racetrack launched 12 s apart over generated hills of 40 m amplitude, in a 4 and -2 m/s wind with 1.2 m/s gusts, 10 percent packet loss, the energy model, and terrain guidance with a 60 m clearance. A 300 s run (`swarm_sim run` then `swarm_sim metrics`) gave:

| Metric | Value |
|---|---|
| Aircraft alive at the end | 8 |
| Collision episodes | 0 |
| Closest approach | 161.5 m |
| Cross-track error (RMS) | 1.17 m |
| Phase error along the path (RMS) | 6.28 m |
| Time on station | 1.00 |
| Lowest height above the ground | 82.7 m |
| Ground contacts | 0 |
| Message delivery ratio | 0.90 |

A test runs the first 90 s of this scenario and checks the same properties.

On the scene exported from the Electric Dreams level (17 849 trees and 102 819 rocks, a 1 km square, not in the repository) a similar scenario with a racetrack of 300 m legs ran for 180 s with all eight aircraft alive, no collisions, a cross-track error of 2.9 m, and a lowest height of 80.1 m above the exported ground. Time on station was 0.60 in that run; the wind and gusts put the phase error at about 35 m, outside the 30 m station radius.

## Two defects found by running it

- **The climb response was a fiftieth of what was asked.** The energy model read a vertical acceleration command as `vz + a * dt` and then passed it through the pitch lag, so a 2 m/s^2 command gave about 0.07 m/s^2. Aircraft sat at their starting altitude while the controller asked for a 10 m/s^2 climb. The unit tests did not catch it because they ran for long enough to saturate. The target is now `vz + a * tau`, and a test checks that 1 m/s^2 held for 2 s gives 1 to 2.5 m/s.
- **The start altitude ignored obstacles.** Aircraft started at the bare ground plus the clearance, under rock tops that the map included. They now start at the map height plus the clearance.

## What this does not show

- The orbit controller, its gains, and the consensus weights were tuned without wind or energy. They fly the scenarios above; they were not retuned or optimized, and no sweep was run.
- The elevation map is exact. A real aircraft's would have errors.
- Terrain guidance flies at about the altitude of the highest ground on the loop plus the clearance. It does not follow terrain down into valleys, so on high, uneven ground the aircraft is much higher above the valleys than the clearance.
- Trees and rocks are boxes and cylinders, and their tops are raised into the terrain map, so the guidance clears them as it clears ground.
- Phase consensus over the neighbors is used only by the orbit pattern. With a racetrack or lanes the neighbor predictors and the comms conditions affect only separation, which is the situation the first probe of ADR 0010 found and fixed for orbits.
- The energy model's parameters are assumed, except the mass and wingspan. No claim is made that it flies like an ALTIUS.
- The experiments of ADR 0010 (216, 72, and 48 cells) were run with the simple model and no wind. Their results are not comparable with a run under the energy model, and they have not been rerun. The 30-seed fixed-wing runs and the quad's 432-cell matrix are still to do.
