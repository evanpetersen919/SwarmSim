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
- **The start altitude ignored obstacles.** Aircraft started at the bare ground plus the clearance, under rock tops that the map included. They then started at the map height plus the clearance, and a later fix (below) made them start at the floor of their whole path.

## Later changes

- **Formation and rake.** A formation pattern (trail, echelon, vee) and a rake shape, with an altitude stagger that applies to any shape. See `docs/MODELS.md`.
- **Obstacle-aware ground contact.** Ground contact and the height-above-ground metric use `Scene::surface_height`, the highest of the terrain and any obstacle top above the point, instead of the terrain alone.
- **Climb budget from the energy model.** Terrain guidance used a fixed fraction of the vehicle's climb limit at any speed. It now uses the climb the propulsion can hold at the mission speed (`EnergyParams::steady_climb_rate`), and a scenario whose aircraft cannot climb at that speed is refused when terrain guidance is on.
- **Start altitude from the whole path.** An aircraft started at the altitude that cleared only the ground under its start point. On hills that rise faster than it can climb, that left it too low to recover. It now starts at the floor of its own path. In scratch runs of `fixed_wing_flight.yaml` with hills of 400 and 500 m (`swarm_sim run`, then `metrics`), an aircraft was lost before this change and none after, and the lowest height above the surface was 64 to 79 m for hills up to 700 m.
- **Radar altimeter.** A nadir height sensor used as a floor on the present (ADR 0012).

## Pilot on the current model

A pilot, not a result: `experiments/pilot_fixed_wing.yaml` on `experiments/base_fixed_wing_energy.yaml` (eight aircraft on stacked orbits over 120 m hills, with the energy model, wind, ground contact, and terrain guidance) crosses weather (clear, fog at 350 m visibility), the radar altimeter (off, on), and packet loss (0, 30 percent): 8 cells, 5 seeds, 40 runs.

```sh
swarm_batch experiments/pilot_fixed_wing.yaml --out pilot.csv --jobs 4
```

Means over the 20 runs of each weather (the debug build; a rerun from the committed files gave the same table):

| Weather | Settled formation error | Orbit radial error | Orbit altitude error | Minimum separation | Aircraft lost | Lowest height above the surface |
|---|---|---|---|---|---|---|
| Clear | 8.4 m | 1.7 m | 3.1 m | 103.8 m | 0 | 70.4 m |
| Fog | 41.8 m | 18.4 m | 20.2 m | 71.1 m | 0 | 70.5 m |

- Weather is the only effect. Fog blinds the camera position fix and the formation error rises about five times.
- The radar altimeter changed nothing: terrain guidance already kept every aircraft at least 70 m above the surface, so the measured clearance never reached the floor. Paired runs (same weather, loss, and seed) differ by at most 0.013 m in the lowest height and 0.0004 m in the formation error.
- Packet loss changed nothing about the flight. The loss was applied (delivery ratio 1.0 at no loss and 0.7 at 30 percent), but paired runs differ by at most 0.004 m in the formation and orbit errors. Orbiting aircraft hold their phase from the shared clock and only use what they hear when separation or the consensus acts, and here the aircraft stay apart.
- No aircraft was lost and there were no collisions in any run.

Limits of this first pilot: one scenario, five seeds, means without intervals, and aircraft that stay well apart. This scenario does not exercise the research question (neighbor prediction and the fallback state machine under loss) for the fixed-wing aircraft. A scenario in which the aircraft are close enough for separation to act, or whose navigation is degraded enough that neighbors' reports carry information, would.

### Two more pilots: comms and the altimeter

Both use `experiments/base_fixed_wing_dense.yaml`: twelve aircraft on one 80 m orbit, about 41 m apart, inside the separation margin, over 120 m hills. They ran in the debug build.

`pilot_fixed_wing_comms.yaml` crosses weather (clear, blind at 200 m visibility), the neighbor predictor (hold-last, dead reckoning, Kalman, turning), and loss (none, 30 percent independent, 30 percent bursty with a mean burst of 6): 24 cells, 5 seeds, 120 runs.

| Weather | Minimum separation | Orbit radial error | Lowest height above the surface | Runs with a collision episode | Aircraft lost |
|---|---|---|---|---|---|
| Clear | 28.7 m | 2.2 m | 67.8 m | 0 of 60 | 0 |
| Blind | 15.5 m | 13.5 m | 54.1 m | 16 of 60 | 0 |

- In clear air the minimum separation is 28.7 m in every predictor and loss cell.
- In blind weather the cells vary (13.8 to 17.1 m minimum separation, 0 to 2 of 5 runs with a collision episode) with no pattern by loss or predictor; that is noise at five seeds.
- A single seed had suggested that loss hurts hold-last and not Kalman. It did not hold up across seeds.
- In the controller, what an aircraft hears feeds only its neighbor predictors, and those feed only separation. An aircraft's own position, which is what blind weather corrupts, does not improve from what it hears. So packet loss and the predictor are second-order for the fixed-wing aircraft here. The research question about prediction and fallback under loss is for the quadrotor matrix, where the neighbors' estimates are the formation.

`pilot_fixed_wing_altimeter.yaml` crosses weather (clear, blind) and the radar altimeter (off, on): 4 cells, 10 seeds, 40 runs.

| Weather, altimeter | Lowest height above the surface (mean, sd) | Runs below the 60 m clearance | Runs with a collision episode |
|---|---|---|---|
| Clear, off | 68.7 m, 9.1 m | 3 of 10 | 0 |
| Clear, on | 68.7 m, 9.1 m | 3 of 10 | 0 |
| Blind, off | 58.8 m, 8.5 m | 6 of 10 | 3 |
| Blind, on | 58.8 m, 8.5 m | 6 of 10 | 3 |

- The paired difference in the lowest height (blind, on minus off) is 0.02 m (sd 0.05 m, 10 pairs). The altimeter did not reduce the worst dip.
- It does act. In seed 1 of the blind case the two runs drift apart by up to about 9 m, and their hashes differ; the lowest height is 44.3 m with and without it.
- The dips come where the ground rises faster than the aircraft can climb, and a reactive sensor cannot prevent that. The altimeter helped only where there was no map (the `AltimeterFlight` test), and the map already clears the ground in these runs.

## What this does not show

- The orbit controller, its gains, and the consensus weights were tuned without wind or energy. They fly the scenarios above; they were not retuned or optimized, and no sweep was run.
- The elevation map is exact. A real aircraft's would have errors.
- Terrain guidance flies at about the altitude of the highest ground on the loop plus the clearance. It does not follow terrain down into valleys, so on high, uneven ground the aircraft is much higher above the valleys than the clearance.
- Trees and rocks are boxes and cylinders, and their tops are raised into the terrain map, so the guidance clears them as it clears ground.
- Phase consensus over the neighbors is used only by the orbit pattern. With a racetrack or lanes the neighbor predictors and the comms conditions affect only separation, which is the situation the first probe of ADR 0010 found and fixed for orbits.
- The energy model's parameters are assumed, except the mass and wingspan. No claim is made that it flies like an ALTIUS.
- The experiments of ADR 0010 (216, 72, and 48 cells) were run with the simple model and no wind. Their results are not comparable with a run under the energy model, and they have not been rerun. The 30-seed fixed-wing runs and the quad's 432-cell matrix are still to do.
