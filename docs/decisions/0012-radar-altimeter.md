# 0012. A radar altimeter as the first added sensor, and what the guidance does with it

Date: 2026-10-22. Status: proposed.

## Context

The fixed-wing aircraft has one sensor, the camera position fix of ADR 0010. It reports a position, or nothing. The aircraft knows nothing else about the world while it flies: the terrain it avoids comes from a map it carries (ADR 0011), which is prior data and not a sensor, and the map is exact.

An audit of the flight code on 2026-10-22 found three things that bear on this decision.

- Terrain guidance plans a climb ahead of the aircraft, but it can only change altitude. It does not turn or slow down for terrain.
- The climb it planned was a fixed fraction of the vehicle's climb limit at any speed. The energy model's propulsion can hold less than that at higher speeds, and with the default propulsion it cannot climb at all at 33 m/s. The budget now comes from the energy model at the mission speed (`terrain_climb_rate`), and a scenario whose aircraft cannot climb at its mission speed is refused when terrain guidance is on.
- An aircraft started at the altitude that cleared the ground under its start point, not the ground along the whole lap it was about to fly. On hills steeper than its climb gradient it started too low to recover. It now starts at the floor of its own path.

With the map exact and those two faults fixed, nothing in the loop tests the map against the world. A sensor that measures the height above the surface directly is the smallest addition that does, and it gives the controller a reason to use a measurement.

Public sources describe the ALTIUS payload only as an EO/IR camera. They say nothing about a radar altimeter, its accuracy, or how the aircraft navigates without GPS. Everything about this sensor is a modeling choice and is labeled that way in docs/MODELS.md, as the camera position fix is.

## Decision

1. **A radar altimeter sensor** in the fixed-wing vehicle: a nadir range, the height of the aircraft above the highest solid surface below it (the terrain, or the top of an obstacle, `Scene::surface_height`), with noise and dropouts. It reads the truth, as sensors may. Weather does not affect it; it is not an optical sensor, so fog does not blind it as it blinds the camera fix.
2. **Output type** `AltimeterSample` in `swarm/sensor_data.hpp`: whether a reading is available, the height above ground, and its standard deviation. That header is the only place a sensor output can live and still be visible to controllers.
3. **Noise and dropout** are drawn from a new Rng stream with a fixed number of draws per call, so results do not depend on thread scheduling. The parameters are `sigma_floor_m`, `sigma_per_m`, `max_range_m`, `dropout_probability`, and `period_ticks`, set under `sensors.radar_altimeter`. All of them are assumed.
4. **Use by the controller.** The brain keeps the map floor as the plan. The altimeter adds a floor on the present: if the measured height above ground, less a margin of its own standard deviation, is below the clearance, the altitude command is raised at the climb budget now, whatever the map says. It never lowers the command. When no reading is available the plan alone applies. The measurement also lets the controller notice a map that disagrees with the world, which the metrics can count.
5. **Scenario and log.** New keys are written to the canonical text only when used, so earlier scenarios and logs are unchanged. The scenario stays at version 3. The binary log format does not change: the sensor's output reaches the log only through the aircraft's state, as the camera fix does. Metrics are computed from the log and the scene as before.
6. **Estimator rule.** The altimeter is not fused into the pose estimator in this step. If it is later, the filter lives under `core/sim/**/estimation/` and sees only `AltimeterSample`.

## Not in scope

A forward-looking EO/IR camera with a field of view and a gimbal, obstacle detection from imagery, turning or slowing down for terrain, a map with errors (the altimeter is what would expose one), and any claim about the real aircraft's payload. The forward camera is the next sensor and gets its own decision: it needs an attitude, a field of view, a detection model, and a decision on what is logged.

## Consequences

- Results with the altimeter are not comparable with runs without it. Runs without the section are unchanged.
- Terrain guidance gains a second input, so its tests must cover a map that disagrees with the world, a dropout, and a failed reading.
- The altimeter does not remove the limit of vertical-only avoidance: terrain that rises faster than the aircraft can climb still needs a turn or a speed change, which is not built.
- The 30-seed fixed-wing runs and the quad matrix wait until this and the guidance fixes are in, because they change the fixed-wing results.
