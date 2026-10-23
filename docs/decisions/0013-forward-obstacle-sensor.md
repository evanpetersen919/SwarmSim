# 0013. A forward obstacle sensor for the fixed-wing aircraft, and a prior map that can be wrong

Date: 2026-10-23. Status: proposed.

## Context

ADR 0012 added the first sensor beyond the camera position fix, a radar altimeter that looks straight down, and left the next one to its own decision: "a forward-looking EO/IR camera with a field of view and a gimbal ... needs an attitude, a field of view, a detection model, and a decision on what is logged."

Where the fixed-wing aircraft stands:

- **It cannot see ahead.** Terrain and obstacles reach it only through the terrain map it carries (ADR 0011), which is prior data. The map is exact, so the aircraft never meets a surprise. The radar altimeter measures only what is already under it.
- **Avoidance is vertical only.** Terrain guidance and the altimeter floor raise the altitude command. Nothing turns or slows the aircraft for ground that rises faster than it can climb (ADR 0012, consequences).
- **The altimeter has no demonstrated value yet.** The pilot in `docs/phases/flight-realism.md` found no benefit with the exact map present. ADR 0012 said it needs a map with errors to show anything.
- **The quadrotor has a LiDAR** (`core/sim/include/swarm/lidar.hpp`) that sees all around with rays against `IScene::raycast`. The fixed-wing aircraft has no equivalent, and the sensor there would be optical, so weather affects it.
- **Public sources describe the ALTIUS payload only as an EO/IR camera.** They say nothing about obstacle detection, a field of view, or range. Every parameter below is an assumption and is labeled so in `docs/MODELS.md`, as the camera position fix and the altimeter are.

Two things are missing for a sensor like this to be worth building: the sensor itself, and a world where it can matter, which is a prior map that disagrees with the scene.

## Decision

1. **A forward obstacle sensor** on the fixed-wing vehicle, modeled as what a forward EO/IR camera plus an obstacle detector would give, not as imagery. It casts a small fan of rays from the aircraft along its heading and reports, per ray, the range to the nearest solid surface (terrain or obstacle) with noise, or nothing. The heading is the direction of the horizontal velocity, as the vehicle defines it (`core/sim/include/swarm/fixed_wing.hpp`), so no attitude model is needed. It reads the truth through `IScene::raycast`, as sensors may.
2. **Geometry.** `azimuth_rays` by `elevation_rays` rays over a horizontal and a vertical field of view centered on the heading, with a pitch offset so the fan can look slightly down. Fixed small numbers (a handful of rays, tens of degrees), so the cost per reading is tens of ray casts. Parameters under `sensors.forward`: `horizontal_fov_deg`, `vertical_fov_deg`, `pitch_deg`, `azimuth_rays`, `elevation_rays`, `max_range_m`, `sigma_floor_m`, `sigma_per_m`, `dropout_probability`, `min_transmission`, `period_ticks`. All assumed.
3. **Weather.** The sensor is optical. With alpha the weather's extinction (fog and rain, `swarm/weather.hpp`), a ray is detected only if `exp(-alpha * range) >= min_transmission`, so fog shortens the distance at which a return is seen before it removes the sensor, the same model the camera fix uses for its ground range. The noise standard deviation grows with range and with the inverse of the transmission.
4. **Output type** `ForwardSample` in `swarm/sensor_data.hpp`: whether a reading is available, and for each ray its unit direction in the world frame, the measured range or no-return, and the standard deviation. That header stays the only place sensor output can live and still be visible to controllers; estimators and controllers never see the scene.
5. **Noise and dropout** come from a new Rng stream (`kStreamForwardBase + drone id`) with a fixed number of draws per ray, whether or not the ray hits anything, as the LiDAR and altimeter do, so results do not depend on scene contents or thread scheduling.
6. **Use by the controller, first step.** The brain keeps the map as the plan and the altimeter as the floor on the present. The forward sensor adds a second floor on the present: each return gives a measured hit point; if a hit lies within `lookahead` of the aircraft and its height is above the aircraft's altitude less the clearance, the altitude command is raised to the hit height plus the clearance, never lowered. The existing altitude control and climb limit set how fast. This is vertical avoidance only, the same limit as before, now driven by a measurement and so correct where the map is wrong. Turning or slowing for terrain is not in this decision (see below).
7. **A prior map with errors.** Terrain guidance gains optional map faults under `terrain`: an `offset_m` added to every height of the map and a list of `omit` rectangles in which the map shows no obstacle and the terrain flat at its edge height. Both are written to the canonical text only when used. Without them the map is exact, as now. This is the test bed: without a wrong map the altimeter and the forward sensor have nothing to correct.
8. **Scenario and log.** New keys are written to the canonical text only when used, in the same scenario version as the altimeter, so earlier scenarios and logs are unchanged. The binary log format does not change in this step: the sensor affects the run only through the aircraft's motion, as the camera fix does. Consequence: detections cannot be counted from the log. Metrics that matter here (`min_agl_m`, `ground_contacts`, crashes) come from the logged state and the scene as before. If detection statistics are wanted later, that is a log version bump and its own decision.
9. **Estimator rule.** The sensor is not fused into the pose estimator. Any later filter lives under `core/sim/**/estimation/` and sees only `ForwardSample`.

## How it will be judged

A pilot before anything is called a benefit, paired by seed on the dense fixed-wing scenario: forward sensor off and on, map exact and map wrong (an `offset_m` of -30 m, and an `omit` rectangle over the ridge the aircraft flies at), clear air and fog at a visibility that shortens the sensor's range. Measured: `min_agl_m`, `ground_contacts`, crashes, time below the clearance. Report paired differences with intervals, including a null result. The altimeter is run alone and together with the forward sensor, since the altimeter's own value was undecided. The offset and rectangle are chosen before the run and not tuned afterward.

## Not in scope

Imagery, target detection or classification, a gimbal, tracking, any claim about the real aircraft's payload or range, turning or slowing down for terrain (a separate decision once a forward sensor exists), cooperative sensing between aircraft, a forward sensor for the quadrotor (it has the LiDAR), and viewer drawing of the field of view. A drone's-eye picture in the viewer was tried with a scene capture and dropped (the level's terrain did not render in a second capture); it is a presentation matter and unrelated to this sensor.

## Consequences

- Runs with `sensors.forward` are not comparable with runs without it. Runs without the section are unchanged, and the new scenario keys do not change any existing canonical text or golden.
- Cost per reading is tens of ray casts at `period_ticks`. A 30-seed fixed-wing benchmark, not yet run, will show what that does to batch time.
- Vertical-only avoidance remains the limit: ground that rises faster than the aircraft can climb still ends in contact. The pilot will show how often, and that result decides whether a turn or speed change is the next decision.
- Terrain guidance gets a third input and a way to be wrong, so its tests must cover an offset map, an omitted rectangle, a dropout, a return at the edge of the range, and fog that removes the sensor.
- Everything about the sensor's accuracy, range, and field of view is a modeling choice. The results say what such a sensor would do in this simulator, not what the real aircraft's payload does.
