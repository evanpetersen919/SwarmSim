# Models

Each model used by the simulation: what it computes, its parameters, its assumptions, and how far it has been validated. A model is documented in the commit that adds it. Status values are `unit-tested` (behavior checked against analytic results), `validated` (compared with external data), or `placeholder` (values chosen without a source).

## Conventions

- SI units: meters, seconds, radians.
- World frame is right-handed with +z up. Yaw is rotation about +z, counter-clockwise, 0 along +x.
- Gravity is `(0, 0, -9.81)` m/s^2. The dynamics model ignores it (thrust compensates); the IMU model accounts for it.
- The Unreal plugin converts to Unreal's left-handed centimeter coordinates at its boundary. The core never uses Unreal units.

## Drone dynamics

Source: `core/sim/include/swarm/dynamics.hpp`. Status: unit-tested.

A kinematic point mass with a yaw angle. There is no attitude, thrust, drag, or motor model.

Per step of `dt` seconds, with command acceleration `a_c`:

```
a  = clamp_norm(a_c, max_accel)
v' = clamp_norm(v + a*dt, max_speed)
p' = p + v'*dt
accel_applied = (v' - v)/dt
yaw_rate' = clamp(yaw_rate_command, -max_yaw_rate, max_yaw_rate)
yaw' = wrap(yaw + yaw_rate'*dt)            wrapped into (-pi, pi]
```

This is semi-implicit Euler. For constant acceleration the position error against the continuous solution is `0.5*a*dt*t`, first order in `dt`; tests check the exact discrete closed form.

| Parameter | Default | Meaning |
|---|---|---|
| `max_speed` | 15 m/s | Speed limit, direction preserved |
| `max_accel` | 8 m/s^2 | Acceleration limit, direction preserved |
| `max_yaw_rate` | pi rad/s | Yaw rate limit |

Limitations: no vertical-specific limits, no wind, no ground contact. Collision with terrain and obstacles is handled by control in a later phase.

## IMU

Source: `core/sim/include/swarm/imu.hpp`. Status: unit-tested; default parameters are placeholders.

An orientation-free inertial sensor: a world-frame accelerometer and a yaw-rate gyro. It reports specific force, so a hovering drone reads +9.81 m/s^2 on z.

```
accel_meas = (accel_true - gravity) + accel_bias + accel_noise
gyro_meas  = yaw_rate_true + gyro_bias + gyro_noise
```

Per sample of `dt` seconds, with continuous-time parameters converted to discrete time:

```
noise ~ N(0, (density / sqrt(dt))^2)         white noise, per axis
bias' = bias + N(0, (walk * sqrt(dt))^2)     bias random walk, per axis
bias(0) ~ N(0, initial_sigma^2)
```

| Parameter | Unit | Meaning |
|---|---|---|
| `accel_noise_density`, `gyro_noise_density` | (m/s^2)/sqrt(Hz), (rad/s)/sqrt(Hz) | White-noise density |
| `accel_bias_walk`, `gyro_bias_walk` | (m/s^2)/sqrt(s), (rad/s)/sqrt(s) | Bias random-walk density |
| `accel_initial_bias_sigma`, `gyro_initial_bias_sigma` | m/s^2, rad/s | Spread of the initial bias |

`ImuParams::typical_mems()` gives order-of-magnitude values for a consumer MEMS part. They are placeholders and are not taken from a datasheet. Replace them with datasheet or Allan-variance values before drawing conclusions about a specific sensor.

Random draws: each IMU has its own stream (`kStreamImuBase + drone id`) and draws eight normals per sample whatever the parameters, so changing one channel's parameters never shifts another channel's noise.

Limitations: no attitude, so no tilt-dependent gravity coupling and no cross-axis effects. No scale-factor error, misalignment, temperature dependence, or saturation. No time-correlated (Gauss-Markov) bias.

## Scene data and terrain

Source: `core/format/include/swarm/format/scene.hpp`, `core/sim/include/swarm/terrain.hpp`. Status: unit-tested.

A scene is a terrain heightfield plus box and vertical-cylinder obstacles, stored as plain data (`SceneData`) in `core/format` so that the Unreal plugin can rebuild the exact geometry the LiDAR saw.

Terrain is a regular grid of height samples, centered on the origin. Each cell is split into two triangles, `(a, b, c)` and `(a, c, d)` for corners `a=(i,j)`, `b=(i+1,j)`, `c=(i+1,j+1)`, `d=(i,j+1)`. Height queries, ray casts, and the Unreal mesh all use this triangulation. Bilinear interpolation is not used, because it disagrees with the mesh inside a cell.

Generation:

- `field`: exactly flat at height 0.
- `hills`: fractal 2D gradient noise. Each octave doubles the frequency and multiplies the amplitude by `persistence`; the sum is divided by the total amplitude, so every height stays within `+-amplitude`. Gradients come from a table of eight fixed directions selected by a splitmix64 hash of (seed, lattice x, lattice y). No random-number stream or libm function is involved, so the result is identical across compilers and platforms, and a golden hash in the tests enforces that.

| Parameter | Meaning |
|---|---|
| `size_x`, `size_y` | Extent in meters, rounded up to whole cells |
| `cell_size` | Grid spacing in meters (at most 4096 cells per axis) |
| `amplitude` | Peak height in meters |
| `feature_scale` | Wavelength of the first octave in meters |
| `octaves`, `persistence` | Detail levels (1 to 16) and amplitude ratio between them |

Limitations: terrain is a single-valued height surface, so there are no overhangs, caves, or water.

## Obstacles

Source: `core/sim/include/swarm/obstacles.hpp`. Status: unit-tested.

Boxes (axis-aligned in x and y) and vertical cylinders placed on the terrain by seeded rejection sampling on stream `kStreamScene`.

- Target count is `round(density * terrain area)`.
- Each attempt draws six uniform values (position, kind, three size parameters) whether or not it is accepted, then is rejected if its bounding circle leaves the allowed area, falls inside the spawn clearance, or comes within `min_spacing` of another obstacle's bounding circle. Neighbor checks use a spatial grid, so cost grows linearly with the number of obstacles.
- The attempt budget is `attempts_per_obstacle * target`. A crowded scene ends with fewer obstacles than the target and the call still returns.
- A box's bounding circle passes through its corners. This is conservative: boxes keep more distance than the spacing parameter strictly requires.
- The base of each obstacle is the lowest terrain height under its footprint minus `embed`. The height is sampled at the center, around the perimeter at no more than a quarter cell apart, and at every terrain vertex inside the footprint. Sampling is dense but not exhaustive; `embed` absorbs the remainder.

Known behavior: when the scene is crowded, large obstacles are rejected more often than small ones, so the realized mix drifts toward small obstacles (for example cylinders). Keep density low enough to reach the target if the mix matters.

Limitations: obstacles are axis-aligned, so there are no rotated buildings, and they do not interact with terrain slope beyond the base height.

## Ray casting

Source: `core/sim/include/swarm/scene.hpp`, `core/sim/include/swarm/geometry.hpp`. Status: unit-tested, and checked against brute force on 70,000 random rays.

`Scene::raycast` finds the nearest hit along a unit-direction ray within a maximum range.

- Terrain: the triangulated surface from the scene-data section. It is two-sided, so a ray starting below the ground hits it from below. Triangles are tested with Moller-Trumbore, with a small slack on the barycentric bounds so rays through shared edges or vertices do not slip between triangles.
- Obstacles: boxes by the slab method, vertical cylinders by a side-wall quadratic plus two cap discs. Obstacles are solid: a ray that starts inside one ignores it. A ray that grazes an edge, or is tangent to a cylinder, counts as a hit.
- Traversal: the ray is clipped to the terrain's xy extent and walked cell by cell. Each cell tests its two terrain triangles and the obstacles registered in it. Traversal stops once the best hit is no farther than the end of the cell just walked, which makes the first hit found the nearest one. An obstacle is registered in every cell its footprint overlaps.
- Geometry outside the terrain's xy extent does not exist.

Queries are `const` and keep no state, so many threads can query one scene at once.

## LiDAR

Source: `core/sim/include/swarm/lidar.hpp`. Status: unit-tested.

A spinning LiDAR mounted level at the drone's position, yawing with it.

- Pattern: beam `b` has elevation spread evenly over the vertical field of view (a single beam sits at the middle). Sample `k` of `N` has azimuth `-fov/2 + (k + 0.5) * fov / N` relative to the heading. The default is 16 beams by 90 samples over 360 degrees.
- Per ray: cast against the scene out to `max_range`; discard a hit closer than `min_range`; discard a hit whose relative intensity falls below `intensity_threshold`; discard it with probability `dropout_probability`. A kept return has range `true + sigma * N(0, 1)` with `sigma = range_noise_sigma + range_noise_sigma_per_m * true`. Measured range can fall slightly outside the limits because of noise.
- Random draws: one normal and one uniform per ray, in scan order, whether or not the ray hits. Scene or weather changes therefore never shift the noise on other rays.
- The whole scan is taken at one pose. Motion during a revolution is ignored.

Beam directions use `sin` and `cos`, and weather uses `exp`, so scans are reproducible on the same binary and platform and not guaranteed bit-identical across platforms.

Limitations: no beam divergence, no multiple returns, no incidence-angle dependence of intensity, no rolling-shutter distortion, and no specular or retro-reflective targets.

## Weather

Source: `core/sim/include/swarm/weather.hpp`. Status: unit-tested. The fog relation was checked against one published relation (see below); rain constants are placeholders.

Weather sets an extinction coefficient `alpha` in 1/m:

```
alpha_fog  = fog_constant / visibility              (default 3.912: Koschmieder, 2% contrast threshold)
alpha_rain = rain_coefficient * rate^rain_exponent  (rate in mm/h)
alpha      = alpha_fog + alpha_rain
intensity  = reflectivity * exp(-2 * alpha * range)  (out and back)
```

A return is detected when its intensity reaches `intensity_threshold`, so the maximum detectable range of a surface is `ln(reflectivity / threshold) / (2 * alpha)`. With the default reflectivity of 0.6 for obstacles, a threshold of 0.05, and a visibility of 200 m, that is about 63 m.

The rain constants (`rain_coefficient = 4e-4`, `rain_exponent = 0.6`) are placeholders chosen to give an extinction of a few per kilometer in moderate rain. They are not from a cited source and have not been checked against data. With them, rain has a modest effect on strong targets within 100 m (100 mm/h pushes the detection limit of a 0.6-reflectivity target to roughly 190 m), while fog is the dominant effect. Do not draw conclusions about rain until the constants are replaced with sourced values.

### Fog check against a published relation

Source: R. H. Rasshofer, M. Spies, and H. Spies, "Influences of weather phenomena on automotive laser radar systems", Advances in Radio Science 9, 49-60 (2011), [doi:10.5194/ars-9-49-2011](https://doi.org/10.5194/ars-9-49-2011). Test: `tests/unit/test_fog_fit.cpp`.

The paper compares laser radar return signals measured in fog at visibilities of 50, 100, 145, and 205 m with a simulation at 50, 100, 150, and 200 m, over 0 to 30 m, for a sensor whose droplet refractive index is given for 905 nm. The caption of its Fig. 7 states that the simulation assumed `alpha = 3 / visibility` (and a backscatter coefficient of 0.046 / visibility per steradian). The text reports "very good agreement" between measurement and simulation and gives no numeric error, so no numeric tolerance is claimed here.

What was found:

- The constant 3 is the 5 percent convention (`ln(1/0.05) = 2.996`), and this project's default 3.912 is the 2 percent one (`ln(1/0.02)`). The two conventions define visibility differently; the paper does not say which it used, so this is an inference from the number. The default extinction is 1.304 times the published one at every visibility, and returns are weaker by `exp(-2 * (3.912 - 3) * range / visibility)`. At 200 m visibility the detection limit of a 0.6-reflectivity surface at threshold 0.05 is 63.5 m by default and 82.8 m with the published constant.
- The model reproduces the published relation exactly when `fog_constant` is set to 3.0 (`weather.fog_constant` in a scenario, a value in (0, 20]). The test checks the extinction and the two-way transmission at the four visibilities and ranges of the figure. The default is unchanged, so all existing results stand; the constant is the setting to use for a sensitivity check.
- Effect on the results: `experiments/fog_convention.yaml` runs the base scenario (ten drones, 20 m altitude) at six visibilities under both constants, three seeds. The position fix is lost, and the settled formation error rises from under 1 m to over 90 m, between 200 m and 150 m visibility with the default constant and between 150 m and 120 m with 3.0.

This is one sourced relation for one sensor type and one fog type. It does not validate the rain constants, the detection threshold, the reflectivities, or the sensor model, and a single paper's measurements do not settle which visibility convention a given deployment would use.

Limitations: weather is uniform in space and time, there is no backscatter clutter or false returns from rain or fog, and it does not affect the IMU or communications.

## Jamming

Source: `core/sim/include/swarm/jammer.hpp`, `core/sim/src/deterministic_channel.cpp`. Status: unit-tested.

A jammer is a zone in space and a schedule in time. While it is active, messages whose link it covers are dropped with probability `loss_inside` (default 1).

- Zone: a sphere of `radius` meters around `center`. A point at exactly the radius is inside. An infinite radius covers everything.
- Schedule: active from `start_tick` up to but not including `end_tick`. A constant jammer is on for the whole window. A pulsed jammer is on when `(tick - start + offset) % period < on_ticks`, with `on_ticks = round(duty * period)`; both the period and `on_ticks` must be at least 1.
- Which links: by default a message is affected when its receiver is inside the zone, which models jamming raising the noise floor at the receiver. With `affects = either`, a sender inside the zone also counts.
- Order within the channel: rate limit, base loss model, jammers (in configuration order, stopping at the first that drops the message), then jitter. Jamming draws from its own random stream, and a jammer with `loss_inside = 1` draws nothing. Adding a jammer therefore does not change which messages outside its zone are lost.
- Positions: the channel is given ground-truth positions each tick, after the agents have stepped and before their messages are sent. Jamming is evaluated at send time, so a receiver that moves into or out of the zone while a message is in flight is judged by where it was when the message was sent. For latencies of a few ticks this is a small approximation.

Command line: `--jam cx,cy,cz,radius,mode,start,end,period,duty,offset,loss,affects`, repeatable. A recorded log stores each jammer as a `jammer=` line in its configuration.

Limitations: a jammer is a hard-edged sphere with no distance falloff, no signal-to-noise model, and no effect on the sender's own reception beyond what `either` gives. Jamming does not depend on terrain or obstacles blocking the signal.

## Kalman filter conventions

Source: `core/sim/include/swarm/estimation/kalman.hpp`. Status: unit-tested.

A linear Kalman filter on fixed-size Eigen types, used by the pose estimator and the neighbor predictor.

```
predict:  x' = F x (+ B u),   P' = F P F' + Q
update:   y = z - H x,  S = H P H' + R,  K = P H' S^-1
          x' = x + K y,  P' = (I - K H) P (I - K H)' + K R K'        (Joseph form)
```

- The covariance is symmetrized after every prediction and update. S is factored with LDLT and no matrix is inverted explicitly.
- `update` returns the innovation, its covariance, and the normalized innovation squared (NIS, `y' S^-1 y`). A measurement whose NIS exceeds an optional gate is rejected and the filter is left unchanged. A positive-definite check on S failing is reported as `kSingular`, also leaving the filter unchanged.
- Invalid arguments (non-finite values, asymmetric or negative covariances) throw `std::invalid_argument`. They are programming errors, unlike an implausible measurement.
- Constant-velocity model for a state `[position (3), velocity (3)]`: `F = [I dt I; 0 I]` and `Q = q [dt^3/3 I, dt^2/2 I; dt^2/2 I, dt I]`, where `q` (m^2/s^3) is the spectral density of white acceleration noise. After `t` seconds without information the velocity uncertainty is about `sqrt(q t)`.
- Gate constant: for three measurement dimensions, the 0.999 quantile of chi-square is about 16.27. A test checks that about one in a thousand consistent measurements exceeds it.

Limitations: linear models only, no process noise adaptation, no smoothing, and no consistency statistic over time (NEES) beyond the checks in the tests.

## Position fix

Source: `core/sim/include/swarm/position_fix.hpp`, `core/sim/include/swarm/sensor_data.hpp`. Status: unit-tested. This is a sensor model, not localization.

The project does not implement scan-matching localization. The LiDAR contributes a simulated position fix whose availability and accuracy depend on what the real scan saw.

- The fix sensor sits on the sensor side of the boundary, like the IMU, so it may read the true position and reports `true + noise`.
- From the scan it takes only how informative the scan was. With `f` the fraction of rays that returned: there is no fix if `f < min_fraction`; otherwise `sigma = min(max_sigma, base_sigma * sqrt(reference_fraction / f))`. Variance is inversely proportional to the number of valid returns, so fog, rain, and open sky degrade or remove the fix.
- The estimator receives only a `PositionFix { available, position, sigma }`. It never sees the scan or the true position.
- Each call draws three normals whether or not a fix results, on stream `kStreamFixBase + drone id`, so one call's noise does not depend on earlier outcomes.

| Parameter | Default | Meaning |
|---|---|---|
| `base_sigma` | 0.5 m | Per-axis sigma when the valid fraction equals the reference |
| `reference_fraction` | 0.5 | Valid-return fraction at which sigma equals `base_sigma` |
| `min_fraction` | 0.05 | Below this fraction there is no fix |
| `max_sigma` | 20 m | Ceiling on the reported sigma; also reported when there is no fix |

The defaults are placeholders chosen so a clear-air scan at 10 m altitude gives a fix of about half a meter. They are not derived from a localization algorithm.

Limitations: the fix is unbiased and independent from call to call, which real scan matching is not. It has no failure mode other than too few returns, and no dependence on scene geometry beyond which rays return.

## Pose estimator

Source: `core/sim/include/swarm/estimation/pose_estimator.hpp`. Status: unit-tested, including a consistency check over 40 seeded runs.

A nine-state Kalman filter over position, velocity, and accelerometer bias in the world frame.

```
a_meas = specific_force + gravity                 (world-frame acceleration reading)
p' = p + v dt + 0.5 (a_meas - b) dt^2
v' = v + (a_meas - b) dt
b' = b
```

- Prediction takes one `ImuSample` per call, so the filter advances by `dt` each time. Process noise comes from the accelerometer's noise density and bias random walk, which are sensor-specification constants supplied as configuration, not read from the simulated truth.
- A `PositionFix` is fused with `H = [I 0 0]` and `R = sigma^2 I`. A fix whose normalized innovation squared exceeds `fix_nis_gate` (default 16.27, the 0.999 chi-square quantile for three degrees of freedom) is rejected and counted. Unavailable fixes are counted and ignored.
- The launch state (position, velocity, and their uncertainty) is a one-time input that models a drone that knows where it was deployed.
- Output: position, velocity, accelerometer bias, and per-axis standard deviations, plus counters for fixes applied, gated, singular, and unavailable.

What the tests show: with regular fixes the bias of a hovering drone is learned to within 0.01 m/s^2 and position error stays under a meter; without fixes both the error and the reported uncertainty grow roughly quadratically, as expected from an uncorrected bias; a grossly wrong fix is rejected without changing the estimate; and across 40 seeded runs of a maneuvering drone the position error lies within three reported standard deviations in at least 95% of samples.

Limitations: yaw and attitude are not estimated, which is acceptable because the IMU reports in the world frame. The filter assumes the process noise it is told about. A fix is treated as independent of earlier fixes.

## Neighbor messages and tracking

Source: `core/sim/include/swarm/estimation/neighbor_message.hpp`, `neighbor_table.hpp`. Status: unit-tested.

Drones broadcast their own estimate, never ground truth. A report is a fixed 65-byte layout: `u8 version | u64 send_tick | f64 position[3] | f64 velocity[3] | f64 position_sigma`, little-endian, with explicit encoding and no struct copying (the shared helpers in `swarm/format/byte_io.hpp`). The sender's identity comes from the message envelope.

A receiving drone keeps a `NeighborTable`: the latest accepted report from each neighbor and the tick it was delivered.

- Age is `now - receipt_tick`, in ticks: it grows by one each tick without a message and resets when a newer report arrives.
- A report is accepted only if its send tick is newer than the one held. An older one is counted as stale and an equal one as a duplicate, and neither changes the table, so channel jitter cannot make a neighbor's state go backwards or reset its age.
- A payload of the wrong size or version, or with a non-finite value or negative sigma, is counted as malformed and dropped. A sender outside the table is counted and ignored.

Limitations: a report carries only position, velocity, and a scalar position sigma, not a full covariance, and nothing authenticates the sender.

## Neighbor predictors

Source: `core/sim/include/swarm/estimation/neighbor_predictor.hpp`. Status: unit-tested. The comparison below is the project's central question in miniature.

Three predictors share one interface (`on_report`, `predict`) and can be chosen from configuration:

- **Hold last state:** reports the last received position and velocity unchanged.
- **Dead reckoning:** `p = p_report + v_report * (now - send_tick) * dt`, extrapolating from the time the report was made. Its uncertainty grows with the assumed velocity sigma.
- **Kalman:** a six-state constant-velocity filter. Each report is carried forward by its delivery delay (the extrapolation adds the process noise of the delay to the measurement noise) and fused as a measurement. Between reports the filter predicts, so its uncertainty grows during a dropout. Reports are trusted; there is no gating, since a neighbor's report is its own filtered estimate.

All three ignore a report whose send tick is not newer than the one held, so reordered and duplicate messages cannot corrupt them.

What the tests measure: a target flying straight at 3.7 m/s, reports every 0.5 s carrying 0.5 m position noise and 0.3 m/s velocity noise, delivered 0.1 s late, then a 3 s dropout. Mean position error at the end of the dropout (seeded trials; reproduce with `swarm_tests --gtest_filter='NeighborPredictors.*' --gtest_output=json:out.json` and read the recorded properties):

| Predictor | Mean error |
|---|---|
| Hold last state | 11.4 m |
| Dead reckoning | 1.66 m |
| Kalman, process noise 1 m^2/s^3 (the default) | 1.32 m |

The Kalman advantage depends on the process noise matching the neighbor. For the same straight-flying target:

| Process noise q (m^2/s^3) | Kalman mean error |
|---|---|
| 0.01 | 0.71 m |
| 0.1 | 1.05 m |
| 1 (default) | 1.35 m |
| 10 | 1.86 m |

So a badly chosen `q` makes the Kalman predictor worse than dead reckoning. The default of 1 is chosen for a swarm that maneuvers at a few m/s^2, which is why it is only modestly better on a straight-flying target. When the target accelerates during the dropout, every predictor's error rises and none can anticipate it. The right `q` is a tuning question for the ablation phase, not something the Kalman filter settles by itself.

Further checks in the tests: the Kalman uncertainty grows monotonically during a dropout and covers the true error within three sigma in at least 95% of trials; delay compensation at least halves the error when reports are 0.5 s late; and a straggler report delivered out of order leaves the state bit-identical.

### Result in the integrated swarm test

The integration test runs eight drones that maneuver (a new random horizontal acceleration of about 0.7 m/s^2 every second), exchange reports at 5 Hz, and lose all messages for 3 seconds. Mean position error of what each drone believes about each neighbor, over the last second of the jam (reproduce with `swarm_tests --gtest_filter='EstimatingSwarm.*' --gtest_output=json:out.json`):

| Predictor | Mean error |
|---|---|
| Hold last state | 12.4 m |
| Dead reckoning | 1.57 m |
| Kalman, default process noise | 1.61 m |

Both extrapolating predictors are far better than holding the last state. The Kalman predictor is **not** better than dead reckoning here: with maneuvering neighbors and the default process noise it is about 3% worse. The advantage in the straight-flight table above does not carry over to this scenario. This is a finding, not something tuned away: whether prediction beats dead reckoning, and by how much, depends on how well the process noise matches the neighbors' maneuvering, and it is left to the ablation phase to map that out.

Limitations: the constant-velocity model, no gating or outlier handling, the report treated as independent of the filter's prior (slightly overconfident), and a single scalar position sigma in each report.

## Formation shapes

Source: `core/sim/include/swarm/control/formation.hpp`. Status: unit-tested.

A formation is a list of slot offsets, one per drone, in the world frame and centered so that they average to the origin. The closest pair of slots is exactly `spacing` apart. Offsets use only basic arithmetic and fixed constants, so they are identical on every platform.

| Shape | Layout |
|---|---|
| Line | One row along y, in slot order |
| Wedge | A V: slot 0 is the apex at the front (largest x), later slots alternate sides and fall back one row per pair, with the arms making 60 degrees |
| Grid | Rows of `c` columns, where `c` is the smallest integer with `c * c >= count`; the last row may be partial |

Offsets do not rotate with the direction of travel. A circle needs trigonometry and is left for later.

## Mission reference

Source: `core/sim/include/swarm/control/mission.hpp`. Status: unit-tested.

A mission is a list of waypoints, a cruise speed, and an optional loop flag. The reference is a point that starts at the first waypoint at time 0 and travels along the polyline through the waypoints at constant speed. It is a pure function of time, so every drone that has the same mission computes the same reference with no messages, which keeps the formation anchored when communication fails.

- Position is continuous everywhere. Velocity is the direction of the current segment times the speed, so it changes at corners.
- A non-looping mission stops at the last waypoint with zero velocity and reports `finished`. A looping mission includes the closing segment from the last waypoint back to the first and repeats; it is never finished.
- Repeated waypoints take no time. A single waypoint, or a loop whose waypoints are all identical, holds in place with zero velocity.
- Times before zero are treated as zero.

Limitations: the reference has no acceleration limit, so its velocity jumps at corners and a drone following it has to smooth that itself. There is no terrain or obstacle awareness: the corridor must be kept clear by the scenario.

## Formation target and control law

Source: `core/sim/include/swarm/control/formation_control.hpp`. Status: unit-tested.

Each drone occupies a slot and estimates the formation center as its own position minus its slot offset. The formation center it steers toward is

    alpha * reference + (1 - alpha) * consensus

where the consensus is the inverse-variance weighted mean of the center estimates of this drone and of every usable neighbor. A neighbor's weight is `1 / (sigma^2 + sigma_floor^2)`, so an old prediction, which carries a larger sigma, counts for less. The floor keeps a very confident estimate from taking all of the weight. The target position is that center plus the drone's own offset; the target velocity is blended the same way.

The control law is proportional-derivative, `a = kp * (target_position - position) + kd * (target_velocity - velocity)`, clamped in norm to `max_accel`. The default gains give a damping ratio of 0.9.

- With no usable neighbor the weight on the reference becomes 1 whatever `alpha` is, so a drone that has lost all communication still holds its slot against the shared mission.
- A neighbor is unusable if its estimate is not valid, not finite, has a negative sigma, or names this drone's own slot or one outside the formation.
- `alpha = 1` ignores neighbors and `alpha = 0` ignores the reference.

Limitations: the consensus is a single-round weighted mean, not an iterated protocol, so drones with different neighbor sets can disagree transiently. The estimate used for the drone's own position is whatever the pose estimator provides; a biased estimate shifts the drone's slot. There is no separation term here; that is a separate rule.

## Separation

Source: `core/sim/include/swarm/control/separation.hpp`. Status: unit-tested.

Each neighbor with a position estimate and uncertainty sigma has the margin `min(d_safe + kappa * sigma, max_margin)`. A neighbor outside its margin contributes nothing. Inside it, the contribution points away from the neighbor with magnitude `gain * (margin - distance)`, capped at `max_accel` per neighbor; the sum is capped at `max_accel`. Stale predictions carry a larger sigma, so the margin widens as information ages. When two positions coincide, the higher slot is pushed along +x and the lower along -x, so the pair is still pushed apart in opposite directions.

`combine_accel` adds the formation command to the separation command with the separation term taking priority: the separation term is kept whole and the formation term is limited to the headroom left under `max_accel`. A plain clamped sum does not work. With the default parameters, two drones flying head-on at 5 m/s each reached 0.03 m apart under a clamped sum, because the formation pull cancelled most of the repulsion at the point where it was needed.

Validation: 500 seeded pairs on head-on courses with perfect information, each sent to the other's start point, run in `tests/unit/test_separation.cpp`. With the priority combination and a 5 m/s speed limit the smallest separation across all runs is 3.67 m, and the test requires more than 3.0 m.

The cap exists because of a failure found while testing the brain: after reports stopped, the neighbors' predicted sigma grew without bound, the margin grew with it, and a drone in safe hold was pushed away at the acceleration limit until it reached its speed limit. Formation slots must therefore be farther apart than `max_margin` (default 12 m); the drone brain rejects a formation that is not.

Limitations: the margin does not depend on closing speed. At a 15 m/s speed limit, a closing speed of 30 m/s exceeds what 12 m/s^2 can stop within the 8 m margin, and the same test measures 0.13 m; a test records this. The rule uses estimates, so it is a margin and not a guarantee. It acts along the line between the pair only, so two drones on an exact head-on course are held apart but do not pass each other.

## Fallback state machine

Source: `core/sim/include/swarm/control/fallback.hpp`. Status: unit-tested.

Two conditions are tracked, each as a latch with separate enter and exit thresholds:

- Comms bad: the fraction of expected neighbors with a fresh report falls below `comms_enter_fraction` (default 0.5) and is good again only at `comms_exit_fraction` (0.75). With no expected neighbors comms is good.
- Navigation bad: ticks since the last position fix exceed `fix_enter_ticks` (100) or the pose sigma exceeds `sigma_enter` (3.0 m). It is good again only when ticks since the fix are at most `fix_exit_ticks` (25) and the sigma is at most `sigma_exit` (1.5 m).

| State | Entered when | reference_boost | speed | margin | gain | hold |
|---|---|---|---|---|---|---|
| Nominal | both good | 0 | 1.0 | 1.0 | 1.0 | no |
| Degraded comms | comms bad only | 0.5 | 0.6 | 1.25 | 1.0 | no |
| Degraded navigation | navigation bad (alone, or with comms bad before the hold time) | 0 | 0.5 | 1.25 | 0.5 | no |
| Safe hold | both bad for `hold_ticks` (150), or pose sigma above `sigma_abort` (15 m) | 0 | 0 | 1.25 | 0 | yes |

`reference_boost` is the fraction of `(1 - alpha)` moved onto the mission reference. The scales are starting values, not tuned.

Every transition other than into safe hold waits for `min_dwell_ticks` (50) in the current state, except the first transition from the initial state. Safe hold is left only after both conditions have stayed good for `recovery_ticks` (150); a relapse restarts that window. A property test over random inputs checks that no transition other than into safe hold comes sooner than the dwell after the previous one.

Limitations: the thresholds are expressed in ticks and fractions and are not derived from the channel or sensor models. The machine does not distinguish a jammed link from a failed neighbor. Navigation is judged from fix age and pose sigma only; it does not compare against neighbor-reported positions.

## Drone brain

Source: `core/sim/include/swarm/control/drone_brain.hpp`. Status: unit-tested.

The decision-making half of a drone (ADR 0005). Its inputs each tick are the IMU sample, the position fix, and the payloads delivered to it; it never sees the vehicle's true state. In order, each step:

1. Advances the pose estimator with the IMU sample and fuses the fix if there is one.
2. Files the delivered neighbor reports in the neighbor table and passes accepted ones to that neighbor's predictor. The predictor kind comes from the configuration. Malformed, stale, duplicate, and unknown-sender payloads are counted and ignored.
3. Counts neighbors whose latest report is no older than `staleness_ticks` and updates the fallback machine with that count, the ticks since the last fix, and the pose sigma. Comms is not judged during the first `staleness_ticks` ticks, because the first reports are still in flight; without this grace a drone with no packet loss started in degraded comms.
4. Computes the command. The formation pull comes from the formation target (the mission reference at `tick * dt` blended with the neighbor consensus, with the reference weight raised by the state's `reference_boost`) and the PD law with the gains scaled by the state's `gain_scale`. The separation term uses neighbor sigma plus the drone's own sigma and the margin scaled by the state. The two are combined with separation taking priority under the acceleration limit.
5. Every `report_period_ticks`, encodes its own estimate (not truth) as a neighbor report for broadcast.

In safe hold the target is the position held when the state was entered, at zero velocity. A speed limiter removes forward acceleration above the limit and brakes the excess. The limit is the mission's cruise speed times `speed_limit_factor` (default 2, so a drone can close a gap) times the state's speed scale. Without a limit below the vehicle's maximum, drones assembling from a scattered start closed on each other fast enough that the true separation fell to 0.86 m in one run; with the limit the closest approach in the same run is 2.64 m. `separation.max_accel` is also the acceleration limit of the command and should equal the airframe limit. A neighbor not heard from for `last_known_after_ticks` (default 150, three seconds at 50 Hz) also contributes its last reported position as a second point to stay clear of, because its prediction keeps moving while a failed drone does not (see the fault model).

`hash_state` folds in the fallback machine, the pose estimate and counters, the neighbor table counters and receipt ticks, each neighbor's current estimate, the hold position, the last target, and the last command. The pose estimator's covariance and the predictors' internal state enter only through the sigmas they report, so two brains whose hidden covariances differ but whose reported values agree would hash equal.

Validation: driven by synthetic sensor data with a point-mass stand-in for the vehicle. A lone drone follows the mission; a drone in place among consistent neighbors commands under 0.1 m/s^2; silence moves it to degraded comms after the staleness limit and back; losing both fixes and reports passes through degraded navigation to safe hold, where it stops, and recovers to nominal when both return; identical inputs give bit-identical commands and hashes, and interleaved brains do not interfere.

Limitations: the thresholds, scales, and gains are starting values. The brain does not avoid terrain or obstacles and does not control yaw. Position fixes are the only correction to the pose estimate, so a long fix outage lets the estimate drift as the accelerometer bias allows.

## Drone agent

Source: `core/sim/include/swarm/drone_agent.hpp`. Status: integration-tested.

The vehicle half of a drone (ADR 0005). It owns the true state and the sensor models and implements `IAgent`. Each tick it samples the IMU from the truth; on the fix period it takes a LiDAR scan through the configured weather and derives a position fix; it passes those and the delivered payloads to the brain; applies the brain's command through the dynamics; and sends the brain's report, if any, to every other drone. The brain's launch state is set from the start state, and the pose estimator's noise constants are copied from the IMU parameters, which are sensor specification values.

`hash_state` folds in the true state, the IMU bias, and the brain's hash. The sensors' random generators are not hashed directly: their draws reach the hash through the brain's estimates.

Validation (`tests/integration/test_formation_swarm.cpp`): wedge formation, 20 m spacing, mission of 300 m east at 4 m/s, flat terrain with 4 m of relief, 50 Hz, burst loss of 5 percent with latency 2 ticks and jitter 1 tick, starts scattered on a 35 m grid with up to 8 m of offset. Measured by the test run:

| Case | Result |
|---|---|
| 2 drones, no loss, 60 s | largest slot error 0.23 m, closest approach above 10 m, only the nominal state visited |
| 8 drones, seed 7, 60 s | mean slot error 0.14 m, largest 0.25 m, closest approach 2.64 m |
| 8 drones, 10 seeds | largest final slot error 0.43 m, smallest closest approach 1.89 m |
| 8 drones, 10 seeds, 6 s jam of all links starting at 2 s | largest final slot error 0.43 m, smallest closest approach 1.92 m; every seed visits degraded comms and every drone ends nominal |

Slot error is the distance from a drone's true position to its slot on the mission reference at the end of the run. The closest approaches happen while the formation is assembled, and they are well inside the 8 m separation margin, so the margin is a soft preference and not a clearance guarantee. The jam in this scenario overlaps assembly and not steady flight; a jam during steady flight leaves the error and separation unchanged because the formation is already in place. Hashes are identical for 1, 2, 4, and 8 threads.

Limitations: one scenario family, one formation shape, flat terrain with no obstacles on the path, and no terrain or obstacle avoidance. Yaw is not controlled, so the LiDAR always faces +x.

## Scenario format

Source: `core/scenario/include/swarm/scenario/scenario.hpp`. Status: unit-tested.

A scenario is a YAML file with `version: 1` and a seed. It holds the scene (field or hills, size, cell size, relief, obstacle density), the swarm (drone count, formation shape and spacing, start layout), the mission, the weather, the sensor settings, the channel (latency, jitter, loss model, rate limit, jammers), the control settings (predictor, report period, staleness), faults, metric settings, and the log stride. Required keys are `version`, `seed`, `duration_s`, `swarm.drones`, and `mission.waypoints`; every other key has a default, listed in [the phase 5 design](phases/phase-05-scenarios-and-metrics.md) and shown by `scenarios/minimal.yaml`. Examples are in `scenarios/`.

Rules, from [ADR 0007](decisions/0007-strict-versioned-scenario-files.md): unknown keys are errors; all problems are reported with a path and line; scalars are parsed by the project, decimal only; times are in seconds and must be a whole number of ticks; the existing model validators run on the result. `canonical_text` writes a scenario back out with enough digits to round-trip, and the loader reads that text to an identical scenario.

Limitations: the schema covers what Phase 5 needs. Terrain relief (`amplitude_m`) applies to hills only; a field is flat. Obstacle size ranges, the separation and fallback parameters, and the sensor noise parameters are not in the file and use the defaults of their structs. Ablation switches are not in version 1.

## Scenario world

Source: `core/scenario/include/swarm/scenario/world.hpp`. Status: unit-tested.

`build_world` turns a scenario into a runnable world: the scene, the simulation configuration, and one drone configuration per drone. The scene is generated from the scenario seed (terrain, then obstacles). The obstacle-free area is centered on the first waypoint, so `scene.obstacles.spawn_clear_radius_m` clears the area where the swarm assembles. Terrain and obstacles do not depend on the number of drones.

A scenario can take its scene from a file instead of generating it:

```
scene:
  source: file
  path: scenes/level.swscene
  hash: 0123456789abcdef   # optional in the file
```

The file is in the scene format (`swarm/format/scene.hpp`), for example one written by `swarm_sim export-scene` or by the Unreal exporter. A file defines the whole scene, so `type`, `size_m`, `cell_m`, `amplitude_m`, and `obstacles` are errors with `source: file`. The path is relative to the directory the program runs in. `build_world` reads the file, checks it, and takes its content hash (`scene_hash`). If the scenario gives a hash and it differs, building fails. The returned world's scenario has the hash filled in, and `run_scenario` writes that scenario into the log header, so the header always names the exact scene the run used. `verify_scenario_log` builds the world from the header, so verifying a log whose file is missing, invalid, or changed fails with a configuration error that names the hash. A scenario that generates its scene has the same canonical text as before, so existing logs and golden files do not change. Heights in a scene file are absolute: mission waypoints and the start altitude are heights in the file's own coordinates, so a scenario that uses a level exported at an elevation of several hundred meters needs waypoints at that elevation. Nothing keeps the drones above the ground except the mission's heights and the brain's separation; the simulation has no terrain-following. A run on a file that holds the scene `build_world` generated for a scenario has the same final hash as the run that generated it (tested).

Start positions are a grid of `start.columns` columns centered on the first waypoint, with each drone moved by a random amount up to `jitter_m` in x and y and up to `altitude_jitter_m` in z. The draws come from their own stream (`kStreamStart`), three per drone in drone order regardless of the jitter amounts, so changing the jitter changes the positions but never the draws. Two drones in adjacent cells start at least `spacing_m - 2 * jitter_m` apart; the test checks this over 100 seeds.

`build_world` rejects a formation whose slots are not farther apart than the separation margin (12 m by default), with a message in terms of `swarm.formation.spacing_m`, and rejects the oracle predictor until it is implemented. `make_simulation(threads)` returns a fresh simulation at tick 0 that refers to the world's scene; the world must outlive it. The hashes of a built simulation are identical for 1, 2, 4, and 8 threads.

Limitations: the start layout is a flat-grid generator only, in absolute altitude; it does not follow the terrain, so a scenario over hills must set `altitude_m` above the relief. Obstacles are not avoided by the drones.

## Channel message events

Source: `core/sim/include/swarm/channel.hpp`, `deterministic_channel.hpp`. Status: unit-tested.

`DeterministicChannel::set_event_sink` (and `Simulation::set_event_sink`) makes the channel append one `ChannelEvent` for every message it is given: scheduled with its delivery tick, or dropped by the loss model, the rate limit, or a jammer. Events carry the sender, the receiver, the send tick, and the delivery tick (zero for a dropped message). They are appended on the barrier thread in send order, so the list is identical for every thread count. With no sink set the channel is unchanged: a test compares a watched and an unwatched channel under loss, jitter, a rate limit, and a jammer and finds the same arrivals and statistics.

Latency of a delivered message is `deliver_tick - send_tick`. Every scheduled message arrives at its scheduled tick, so the scheduled tick is the delivery tick; a test matches the scheduled ticks against the ticks at which messages actually arrive. A message scheduled for a tick after the end of a run is still in flight at the end; metrics exclude those.

The sink is on the concrete class and not on `IChannel`: the real-socket backend will report what it can observe in its own way.

## Run snapshot

Source: `core/format/include/swarm/format/snapshot.hpp`. Status: unit-tested. Decision record: [0006](decisions/0006-run-snapshots-in-the-log.md).

The content of a log record's blob. Little-endian, fixed width: a version byte, the agent count, one 49-byte entry per agent (flags, position, velocity), the event count, and one 25-byte entry per message event (kind, sender, receiver, send tick, delivery tick). The flags hold the alive bit and the two-bit fallback state; the other bits are reserved and must be zero. The exact layout is in the header and in `tests/unit/test_snapshot.cpp`, which checks the encoder and decoder against hand-written bytes.

The encoder rejects a fallback state above 3, an unknown event kind, a non-finite number, and more than 4096 agents or 2^20 events. The decoder rejects the same, plus an unsupported version, a buffer that ends early, and extra bytes, and it checks the counts against the remaining length before allocating. A test cuts the sample at every length and decodes each prefix. On any error the output is empty.

Limitations: positions and velocities are written as 64-bit floats for exactness, which is larger than a viewer needs. The format has no per-agent estimate fields; metrics about estimation error would need a new version.

## Scenario run and log verification

Source: `core/scenario/include/swarm/scenario/run.hpp`. Status: integration-tested.

`run_scenario` builds the world, steps the simulation for the scenario's duration, and writes a log with one record per tick. The header holds the seed, `dt`, and the drone count, and its config text is the line `swarmsim-scenario 1` followed by the scenario in canonical form (not the original file text, so comments and layout are not kept; the content is). Each record holds the tick, the state hash, and a snapshot blob ([run snapshot](#run-snapshot)). The blob always lists the messages sent in that tick. It lists every drone's true state on ticks divisible by `log.snapshot_every_ticks` and on the last tick, so a stride saves space without losing any message.

`verify_scenario_log` reads the scenario back out of the header, checks that the header's seed, `dt`, and drone count agree with it, re-runs it, and compares for every record the tick, the hash, and the blob bytes. A divergence reports the tick and whether it was the hash or the snapshot. The thread count is free. `verify_run_log` is the single entry point: it dispatches on the first line of the config, so demo logs (the `key=value` form) still verify as before. The runner and the verifier build each record with the same function, so they cannot disagree about the bytes.

The command line has a `run` subcommand, `swarm_sim run SCENARIO --out FILE [--threads N]`, which prints the scenario name, tick and drone counts, and the final hash, and exits 2 with every loader error if the file is invalid. `--replay FILE --verify` accepts both kinds of log.

Validation: a recorded log decodes at every tick, its snapshots equal the true state of an independent run bit for bit, the log bytes and final hash are identical for 1, 2, 4, and 8 threads, a flipped snapshot byte is found at its tick, a flipped hash is found, and the message events in the log add up to the channel's own statistics with every jam drop inside the jam window.

Limitations: the log holds the whole run in memory before it is written. The runner needs the drones to be `DroneAgent`s, which is the only agent a scenario builds. The alive flag in the snapshot is false from a drone's failure tick (see the fault model).

## Fault model

Source: `core/sim/include/swarm/drone_agent.hpp`. Status: integration-tested.

A fault is a drone index and a tick, set in the scenario's `faults` list. From that tick the drone has failed: it stops where it is with zero velocity and acceleration, samples no sensors, ignores its inbox, and sends nothing. It stays in the world as a physical object. Its frozen position still counts for jamming zones and for the separation metric, and the snapshot marks it not alive from its failure tick and not before. Until the failure tick the run is identical, hash for hash, to the same run without the fault. It does not fall and does not hover; this is a stated, simple model and not a vehicle failure model. Nothing reassigns its slot.

What the others do with a silent neighbor, measured in `tests/integration/test_drone_failure.cpp` (eight drones, failure at 30 s into a 60 s run, after the formation has settled):

- Wedge flying along its axis of travel, any of drones 0, 2, or 7 failing: the other seven stay within 0.60 m of their slots from the failure to the end, against the 2 m criterion in the phase design, and the closest approach of any two drones is 9.76 m. Followers pass beside the frozen drone at the slot spacing, so nothing more is needed here, and the consensus is not dragged: the prediction of a dead drone keeps moving with the formation, and its weight falls as its uncertainty grows.
- Line flying along its own axis (single file), drone 1, 4, or 7 failing: each follower's path leads through the failed drone. With the prediction alone, the closest approach was 0.08 to 0.28 m, which is a collision, because the followers' separation rule sees a neighbor that has moved on. With the last reported position of a silent neighbor kept as a second point to avoid (`last_known_after_ticks`, default 150), the closest approach is 5.9 to 6.2 m.
- The price is that the followers behind the failed drone stop there and never reach their slots: the worst slot error afterwards is 21.9 m (failure of drone 4). Nothing routes around the drone or reassigns the slot. This is documented behavior and a stated limit of the project, not a goal of this phase.

A neighbor that is silent because of a jam, and not failed, also gets the last-position rule once it has been silent for three seconds. In the jam scenarios of the swarm tests this changed neither the smallest separation (1.92 m) nor the final slot error (0.43 m). A longer jam would make drones keep clear of positions where their neighbors no longer are, which is the conservative choice.

Limitations: the failure is instant and total. Failures during assembly were not studied separately: the slot error while a formation is still assembling is dominated by assembly, and the measurements above fail a drone after it has settled.

## Metrics: formation error, separation, collisions

Source: `core/scenario/include/swarm/scenario/metrics.hpp`. Status: unit-tested on hand-built logs and integration-tested on a recorded run.

`analyze_log` takes the bytes of a scenario log and nothing else. The scenario it needs (mission, formation, metric settings) is read from the log header, and the per-tick state from the snapshots. Records without drone state, which a snapshot stride produces, are skipped for these metrics. Sums are taken in tick order and then drone order, with only addition, division, and `sqrt`, so the result is reproducible.

- **Slot error and formation RMSE.** The slot of drone `i` is `r(t) + o_i`, with `r` the mission reference and `o_i` the formation offset, computed with the same `MissionPath` and `make_formation` the drones use. The state in the record of tick `k` is the state after that tick's step, so it is evaluated at time `(k + 1) * dt`. The RMSE is over alive drones and over records. It is reported for the whole run and, separately, for records at or after `metrics.settle_s`, because the whole-run value includes the assembly transient. Failed drones are excluded. A value that cannot be computed (no record at or after the settle time) is empty and not zero.
- **Minimum separation.** The smallest true distance between any two drones over all records with state, failed drones included, since a failed drone is still an object in the way.
- **Collisions.** Counted as episodes per pair: a pair starts one when its distance falls below `metrics.collision_radius_m` and ends it when the distance is back at or above the radius. A distance equal to the radius is not inside. A pair already inside in the first record starts an episode there. The count is over all pairs.
- **Alive at end.** Alive drones in the last record with state.
- **Latency and delivery ratio.** From the message events, which every record carries whatever the snapshot stride. Latency is `(delivery tick - send tick) * dt` of each delivered message, and `latency_p50_s` and `latency_p95_s` are percentiles by the nearest-rank rule: the value at index `ceil(p * n) - 1` of the sorted list, computed with integer arithmetic (`ceil(0.5 n)` is `(50 n + 99) / 100`, and the same with 95), so there is no floating-point edge. For one sample both percentiles are that sample, for two the median is the lower and the 95th the higher. A scheduled message whose delivery tick is after the last record is still in flight when the log ends: it is counted as in flight and left out of the latency and the ratio. The delivery ratio is delivered messages divided by delivered plus all drops, with the drops also reported by reason (loss, rate limit, jamming). With nothing delivered there is no latency; with nothing decided there is no ratio. A message delivered before it was sent marks the snapshot bad.
- **Recovery time.** A disturbance is the end of a finite jammer window (the first tick after the window) or a drone failing. Let `E(k)` be the formation RMSE over alive drones at tick `k`, computed from the records with drone state, and `t_d` the disturbance tick. Recovery is the first tick `t_r >= t_d` from which `E` stays at or below `metrics.recovery_threshold_m` (a value equal to the threshold counts as within) for `metrics.recovery_hold_s`, which is converted to a whole number of ticks, at least one. The recovery time is `(t_r - t_d) * dt`; it is zero if `E` is already within the threshold at `t_d` and stays there. If no such run is completed before the log ends, the result is censored: `seconds` is the time from `t_d` to the end of the log, which is a lower bound, and the `censored` flag is set, so a run that never recovers cannot look like a fast recovery. A disturbance at or after the end of the log is not observed and gives no result. A jammer with no end gives none. Results come in tick order, a jam end before a fault at the same tick. The summary is the longest `seconds` and whether any result was censored; a run with no disturbances has no recovery time at all, not zero.

Unusable logs return a status and a message and no values: not a log, a demo log, a scenario that does not load or disagrees with the header, a snapshot that does not decode (the message names the tick) or has the wrong drone count, and a log with no records.

Validation: hand-built logs with known positions give zero error, an exact constant error, a hand-computed mix, and the expected whole-run and settled values; the minimum separation equals a brute-force minimum over a random log; collision episodes are counted once however long they last, twice for two dips, and not at all for a distance equal to the radius. On a recorded run the values match a computation done separately from the simulation's own state to 1e-9, and a stride can only miss the closest approach, never report a smaller one. Recovery is checked on synthetic traces whose error is set tick by tick: a return that stays gives the known time; a dip shorter than the hold does not count; a trace already within the threshold recovers in zero time; a trace that never recovers, or whose log ends before the hold has elapsed, is censored with the time remaining; the hold is measured in ticks; several disturbances give several results; a failed drone's error is not part of the trace. On a recorded run with independent loss of 30 percent (four drones, 1,440 messages), the counts equal the channel's own statistics, the delivery ratio is 0.691 against the set 0.7, the latencies fall between the channel latency and latency plus jitter, and a snapshot stride changes nothing about messages. On a recorded run with a jam and a failure at the same tick (six drones, 36 s), the formation recovers 12.54 s after the disturbance, which matches a separate scan of the simulation's state; the first recording, 18 s long, was censored, because the formation was still assembling.

Limitations: with a snapshot stride the closest approach between recorded ticks is not seen, so minimum separation is an upper bound on the true minimum and collisions can be missed; the default stride of 1 sees every tick. The pairwise loops are quadratic in the drone count, which is fine at the sizes used here.

## Metrics CSV and the metrics command

Source: `core/scenario/include/swarm/scenario/metrics_csv.hpp`. Status: unit-tested; the command is covered by the command-line script.

`swarm_sim metrics LOG [--csv FILE]` computes the metrics of a scenario log and writes a header row and one data row, to the file or, without `--csv`, to standard output. It exits with status 2 for a file it cannot read, a log that is not a scenario log, a scenario or snapshot that does not decode, or bad arguments, and prints the reason.

The header and the row come from two functions, `metrics_csv_header` and `metrics_csv_row`, so the columns cannot drift apart and files from different runs can be concatenated as they are (the batch phase relies on this). The 23 columns, in order: `scenario, seed, drones, dt_s, ticks, duration_s, formation_rmse_m, formation_rmse_settled_m, alive_at_end, min_separation_m, collision_episodes, recovery_count, recovery_longest_s, recovery_censored, messages_sent, messages_delivered, messages_in_flight, dropped_loss, dropped_rate, dropped_jam, delivery_ratio, latency_p50_s, latency_p95_s`.

Rules: lines end with a line feed. Whole numbers are written in decimal, exactly. Other numbers use `%.9g`, so they round-trip to nine significant digits; formatting does not depend on the locale because the project never sets one. A value that does not exist is an empty cell, never zero. `recovery_censored` is 1 if any recovery was censored, 0 if none was, and empty when the run had no disturbance. A scenario name containing a comma, a quote, or a line break is quoted with quotes doubled; a test reads such names back unchanged.

Validation: the exact text of the header and of the row for a known result, empty cells for a result with nothing in it, the same number of cells in every row as in the header, a round trip through a small reader, and the command run on a recorded scenario log.

Limitations: one row per run and no per-tick output. The scenario name and seed identify a run only if each combination is unique, which the batch phase has to ensure.

## Oracle baseline and selectable predictors

Source: `core/sim/include/swarm/agent.hpp`, `drone_agent.hpp`, `control/drone_brain.hpp`. Status: integration-tested.

`control.predictor` in a scenario selects how each drone estimates its neighbors: `kalman`, `dead_reckoning`, `hold_last`, or `oracle`. The first three are the predictors of Phase 3. The oracle is an evaluation baseline: each drone is told every other drone's true position and velocity as of the end of the previous tick, with zero uncertainty and zero age. It is an upper bound on what any predictor could achieve, not a realistic drone.

Where the truth lives. A brain must never read the truth, so the oracle is on the vehicle side. The simulation publishes the true position and velocity of every agent to all agents in `AgentContext::peers`, updated between ticks and never during one, so reading it from the parallel section is safe. Only a `DroneAgent` with `oracle` set reads it; it passes the values to the brain in `BrainInput::neighbor_override`, plain `NeighborEstimate` values the brain cannot trace to a source. With an override the brain uses it in place of its predictors, counts every neighbor as fresh (so the comms condition never triggers), and skips the last-known-position rule, so the channel plays no part in what it does. The brain's include whitelist is unchanged.

Validation:

- Under the oracle the true trajectories are bit-identical for a channel that drops nothing and for one with 50 percent burst loss, a rate limit of two, and a jam of every link. Without the oracle the same two channels give different trajectories, so the comparison is able to see a difference.
- A brain under the oracle holds exactly the previous tick's truth for every neighbor, with sigma zero and age zero, and never leaves the nominal state because of the channel.
- An agent that is not the oracle is bit-identical, in state and hash, whether the published truth is accurate or nonsense; an oracle agent does use it.
- The simulation publishes the previous tick's state: at tick 0 the initial state, at tick t the state after t steps.

Estimate error during a 3 s jam of every link, averaged over the last second, every ordered pair, against the previous tick's truth (six drones, no other loss):

| Case | Oracle | Hold last | Dead reckoning | Kalman |
|---|---|---|---|---|
| Jam at 30 s, formation settled | 0 m | 9.51 m | 0.274 m | 0.264 m |
| Jam at 2 s, formation assembling | 0 m | 13.4 m | 5.92 m | 6.33 m |

Holding the last state is much worse than extrapolating it in both cases. Kalman is a little better than dead reckoning in steady flight and a little worse during assembly, where the drones are converging fast and slowing, so a constant-velocity extrapolation overshoots. The test asserts the oracle's zero error and the wide gap to hold-last in steady flight, and records the rest.

Limitations: the oracle sees every drone regardless of range. It reports a failed drone's frozen position, as truth. The published truth is one tick old, so the oracle's own estimate is one tick behind the present, which matches the one-tick latency of the shortest channel.

## End-to-end validation of the scenario runner

Source: `tests/integration/test_scenario_end_to_end.cpp`. All numbers below are recorded by that file's tests (eight drones, wedge, 20 m spacing, 50 Hz, 60 s, clear air unless stated, formation error settled from 20 s).

- **Kalman against the oracle at zero loss** (one tick of latency, no jitter, no jam): settled slot RMSE 0.439 m for the Kalman predictor and 0.448 m for the oracle, so the predictor costs nothing measurable here. The Kalman neighbor estimate is 0.177 m from the previous tick's truth on average; the oracle's is exactly 0. Neither run has a collision, and the fallback machine never leaves nominal.
- **Packet loss** (0, 10, 30, 50 percent, independent and burst with a mean burst of four, latency 2 ticks, jitter 1 tick): the delivery ratio matches one minus the loss to within 0.05 in every case (measured 0.495 at 50 percent independent loss, 0.500 at 50 percent burst). The neighbor estimate error grows with loss in both models and is larger for bursts than for independent drops at the same mean (0.1798 m against 0.1784 m at 50 percent, from 0.1770 m with no loss). The settled slot RMSE stays within 0.05 m of its no-loss value (0.439 m) at every level. The effect of loss on the formation is therefore small in this scenario. Steady straight flight is easy to predict, and the shared mission reference anchors every slot, so a drone that hears nothing still flies to the right place. The scenario does not test loss during maneuvers.
- **Fog** (visibility 20,000 m, 200 m, 150 m, 40 m; 1,500 ticks): the loss of position fixes is a cliff and not a slope. At 200 m all 2,399 fixes that clear air gives survive and the settled RMSE is 0.83 m against 0.80 m. At 150 m only 24 survive, at 40 m only 4, and the settled RMSE is 42.9 m and 46.3 m. With the fixes gone the drones pass through degraded navigation, and the formation breaks. The cliff follows from the detection model: the steepest beam meets the ground 40 m away at the 20 m flight altitude, and its return falls below the detection threshold between the visibilities of 200 m and 150 m.
- **Examples**: every file in `scenarios/` runs for five seconds, verifies on three threads, yields metrics, and gives a CSV row with the right number of cells.
- **Full scenario**: eight drones with burst loss, a jam from 2 s to 8 s, and drone 3 failing at 20 s record, verify on another thread count, and give a metrics row in which no cell is empty (two recovery results, jam drops and loss drops both present).

Limitations: one seed per case, so the differences between loss levels are small and the ordering is not a statistical result; the batch phase repeats cases over seeds. The cases were chosen to be the sanity checks the plan names, not to span the evaluation matrix.

## Wire protocol

Source: `net/include/swarm/net/wire.hpp`. Status: unit-tested.

The format shared by the UDP and TCP transports. A message is a 30-byte header followed by its payload, little-endian: two magic bytes `S` and `N`, a version byte (1), a type byte (1 for a beacon, 2 for a command), the sender, the receiver, the sequence number, the send tick, and a 16-bit payload length. The payload is at most 1,100 bytes, which keeps a datagram well under a typical path MTU. A UDP datagram holds one message. A TCP stream holds messages back to back, and the length in the header is enough to find the end of each. There is no checksum, because UDP and TCP have their own. The delivery tick is not on the wire: it is a property of the channel's model and the receiving side learns it from its own record of the send. The exact layout is in the header and in `tests/unit/test_wire.cpp`, which checks the encoder and decoder against hand-written bytes.

`decode_message` takes exactly one datagram and reports one of: truncated, bad magic, bad version, unknown type, oversized payload, or trailing bytes. `FrameDecoder` takes a byte stream in pieces of any size, including one byte at a time, and returns whole messages as they complete. A header problem cannot be skipped over on a stream, because there is no way to find the next message, so the decoder fails and stays failed; messages completed before the failure are kept. It checks each header field as soon as its bytes have arrived, so garbage is noticed at its first byte, and it never holds more than one incomplete message.

`Message` gained a `type` (default beacon) and a `send_tick`, which the deterministic channel now sets when a message is sent. Existing senders are unchanged.

Validation: a byte-exact layout test; 500 randomized round trips including the empty and the largest payload and the extreme values of every field; every truncation of a datagram is rejected; each malformed field has its own status; the stream decoder returns the same messages for whole, one-byte, random, and every-possible-split chunkings.

Limitations: no versioning beyond the version byte, no compression, no authentication or encryption, no fragmentation of a message larger than the limit (such a message is refused at encoding).

## Channel model and message classes

Source: `core/sim/include/swarm/channel_model.hpp`. Status: unit-tested.

What happens to a message on a channel is decided by `ChannelModel`: the rate limit, the loss model and its burst state, the jammers, and the latency and jitter, with their seeded random streams. It decides one message at a time, in the order it is given, and the outcome depends only on the configuration, the seed, and that order. `DeterministicChannel` is the model plus a queue that holds messages until their delivery tick. The socket channel of Phase 6 uses the same model, so the two cannot drift apart. `IChannel` gained `endpoint_count` and `set_event_sink`, and `Simulation` can be given a channel built by the caller.

Messages have two classes. A beacon is unreliable, as before. A command (`MessageType::kCommand`) models a transport that retransmits: the rate limit, the loss model, and the jammers do not drop it, it is delivered after exactly the base latency with no jitter, it does not count toward the rate limit, and it draws no random numbers, so adding commands to a run never changes the fate of any beacon. Commands from one sender to one receiver arrive in the order sent. The model does not represent retransmission delay, so a command on a jammed link is treated as if it got through at the base latency. That is a stated simplification, not a measurement.

Validation: the refactor changed nothing. A harsh run (burst loss, jitter, a rate limit of one, a pulsed and a global jammer, eight demo agents, 400 ticks) has the state hash 13965888112468217684, the message-event digest 17948217017054799834, and 6,400 sent, 1,861 delivered, 637 dropped by loss, 3,200 by the rate limit, and 680 by jamming, and these were recorded from the channel before the model was extracted and are asserted unchanged. The model alone gives the same decision as the channel for 16,800 messages. Reliable messages survive loss of 1.0, a jammer over everything, and a rate limit of one, and arrive in order; interleaving them leaves every beacon's outcome and delivery tick unchanged. A simulation uses a channel it is given, passes it the event sink, and rejects one with the wrong number of endpoints.

Limitations: the reliable class is a model of a retransmitting transport with unlimited patience, not of TCP. A command is never late because the link is bad.

## Hold and resume commands

Source: `core/sim/include/swarm/control/command.hpp`, `ground_station.hpp`. Status: unit- and integration-tested.

A ground station tells drones to hold or to resume. A command is a 10-byte payload (version, kind, and the tick it was issued at) sent as a reliable message (see the channel model) from a `GroundStationAgent`, which is placed after the drones so that its endpoint is the drone count. It sends each scheduled command once, to every drone, on its tick; the schedule must be in tick order.

A drone applies a command only if it was issued later than the last command it applied, so a reordered pair or a duplicate cannot leave it in the wrong state: a hold issued at tick 100 and a resume at tick 50 leave it holding whichever arrives first, and a hold at 10 with a resume at 20 leave it flying in either arrival order. A command that does not decode is counted as malformed and ignored, and a command payload sent as a beacon is not a command. The brain accepts commands from any sender; there is no authentication.

A hold stops the drone where it is and keeps it there, with the same stopping behavior and wide margin as the fallback machine's safe hold. It is not a fallback state, and the machine is unaffected: a drone under a hold in clear conditions stays nominal. A resume returns it to flying the formation. Because the mission reference keeps moving during a hold, a drone resumes behind its slot and catches up at up to twice the cruise speed.

Two rules make a hold a promise to stay put. While holding, separation counts only neighbors heard from within the staleness window, because a neighbor that has gone silent is extrapolated at the speed it had, which for a swarm told to hold together is wrong and pushed the holding drones at about 1 m/s in a first version. And the last-known-position rule is off, because it protects a moving drone and a holding one cannot fly into anything. Both apply to safe hold as well.

Validation: the codec round-trips and rejects each malformed form; one drone stops within 12 m of where it was told to hold, stays within 0.05 m afterwards, stays nominal, and after a resume is back within 1.5 m of its slot; commands arrive in either order, duplicates are ignored, and the commanded state is part of the hash. In a swarm of six drones with half the beacons lost in bursts and every link jammed for 7 s around the hold, every drone received and applied both commands, the worst speed 4 s into the hold was 0.27 m/s and 0.12 m/s at 6.6 s, no drone moved more than 1 m between those times, and after the resume every drone was within 2 m of its slot. Hashes are identical for 1, 2, 4, and 8 threads. A station with an empty schedule leaves the drones bit-identical under independent loss; under burst loss it would not, because the number of links seeds the burst states, so a scenario without commands has no station.

Limitations: commands are never lost or delayed by the model (a stated simplification), the slot of a drone is not reassigned, and one station commands all drones at once.

## Scenario commands

Source: `core/scenario/include/swarm/scenario/scenario.hpp`, `world.hpp`. Status: unit- and integration-tested.

A scenario may list commands for a ground station to send to every drone:

    commands:
      - {at_s: 30, kind: hold}
      - {at_s: 36, kind: resume}

Each entry needs `at_s`, which must be a whole number of ticks like every time in a file, and `kind`, which is `hold` or `resume`. Entries may be written in any order; the loader sorts them by time, and entries at the same time keep the order they were written in. At most 1,000 entries. A scenario without the key has no commands, and canonical text writes `commands: []`.

When the list is not empty the built world has a ground station, placed after the drones so that its endpoint is the drone count. The station sits at the origin for the jamming zones. A scenario without commands has no station and exactly one endpoint per drone, so every existing scenario and its hashes are unchanged: a test pins the state hash of a 300-tick run of `formation_jam.yaml`, recorded before this change. The station is not a drone for anything that reads a log: the log header and every snapshot hold the drones only, the verifier and the metrics ignore it, and the brain's neighbors are the drones.

Measured on `scenarios/hold_resume.yaml` (six drones, a hold at 30 s and a resume at 36 s, half the beacons lost in bursts, every link jammed from 28 s to 35 s): the formation RMSE is 0.27 m just before the hold, 22.97 m at the end of the hold (the reference has moved about 24 m on), and 0.20 m at the end of the run, 28 s after the resume. The log verifies on 2 and 4 threads, its bytes are identical for 1 and 4 threads, and the metrics count six drones and one recovery (the end of the jam). A hold and a resume are not disturbances in the recovery metric.

Limitations: one station for all drones, at the origin; commands go to every drone at once; there is no way to address a single drone.

## Socket wrapper

Source: `net/include/swarm/net/socket.hpp`. Status: unit-tested on Windows; the POSIX branch is built and tested by the Linux CI job, and was also checked for warnings against stand-in POSIX headers before the commit.

A small portable layer over UDP and TCP, for Winsock2 and POSIX systems. It does only what the transports need: IPv4 on the loopback interface, blocking sockets, and a poll with a timeout. The header includes no platform header, so the Windows macros stay out of the code that uses it.

- **Addresses and ports.** Sockets bind to 127.0.0.1. Asking for port 0 lets the operating system choose a free port, which the tests use throughout so that runs in parallel never collide; `local_address()` reports the port chosen. Network byte order is built by hand, not with `htons` and `htonl`, which some C libraries expand into code that draws conversion warnings.
- **UDP.** `bind_loopback`, `send_to`, and `receive` with a timeout. A datagram larger than the buffer is reported as truncated (with `MSG_TRUNC` on POSIX, and with the `WSAEMSGSIZE` error on Windows) and is dropped, not delivered again. A zero-byte datagram is a datagram. The receive and send buffers are enlarged on a best-effort basis. On Windows the socket option that turns an ICMP "port unreachable" into a receive error is switched off.
- **TCP.** `TcpListener` with `accept` and a timeout, `TcpStream::connect`, `send_all` (loops until the operating system has taken every byte; reports a closed peer; writing to a closed peer is an error and never a signal), `receive_some` (reads what is there), and `receive_exact` (reads exactly `n` bytes within a total time, and says how many arrived if time runs out). `TCP_NODELAY` is set. TCP preserves no write boundaries, which is why the wire protocol has its own framing.
- **Waiting.** `wait_readable` takes several sockets and returns the positions of those that can be read without blocking, which for a listener means a connection is waiting and for a connection means data or a close.
- **Errors.** Setup failures throw `NetError`, which carries the operating system's error code and text (`bind() to 127.0.0.1:5000 failed: ... (error 10048)`). A timeout and a closed connection are statuses. Using a closed or moved-from socket throws and does not touch a dead descriptor.
- **Ownership.** Sockets are move-only and close on destruction; closing frees the port, and closing twice is harmless.

The core must stay independent of all this. The invariants script now fails if anything under `core/` includes a socket header (POSIX or Windows) or anything from `net/`, and its tests run it on fixtures with each kind of violation, and on a fixture showing that `net/` itself may read the clock and include socket headers.

Validation: two sockets get different ports; datagrams carry the sender's address and keep their boundaries and order; truncation is reported; a receive times out as a status after about the time asked, polls with a zero timeout, and returns at once when data is waiting; a second bind to a port in use throws with the reason; a freed port can be bound again; four MiB crosses a TCP connection exactly while the sender writes from another thread; reads may return pieces of a write; a closed peer is reported after the bytes sent before the close have been read; a write to a closed peer reports closed and does not kill the process; a timed-out exact read says how much arrived; accepts happen in connection order and time out cleanly; connecting to a port nobody listens on throws with the reason.

Limitations: IPv4 loopback only, blocking calls only, no `epoll` or `io_uring`, no IPv6, no TLS. Poll restarts its timeout if a signal interrupts it on POSIX. The Windows and POSIX code paths are small and parallel, but a behavior that differs between systems (buffer sizes, which errors a closed peer produces) is handled where it was observed or documented and no further.

## UDP transport

Source: `net/include/swarm/net/udp_endpoints.hpp`. Status: unit-tested.

The transport for state beacons. A set of endpoints exchange messages as UDP datagrams over the loopback interface, one socket per endpoint, each on a port the operating system chose. `send` serializes a message with the wire format and sends it from the sender's socket to the receiver's. `receive` returns the valid messages that have arrived, waiting up to a timeout if none have. UDP is unreliable by nature and this class does not hide that: it neither retransmits nor orders.

What it checks on every arrival, in this order, counting what it throws away and carrying on:

1. The datagram decodes as exactly one wire message and was not cut short (otherwise it is `malformed`).
2. It came from the address of the endpoint named as its sender (otherwise `foreign`). A program that is not one of the endpoints cannot deliver a message by sending to a port.
3. It arrived at the socket of the endpoint named as its receiver (otherwise `misdelivered`).

These checks catch accidents. They are not a defense against anything malicious on the machine, and there is no authentication.

A UDP receiver silently drops datagrams once its queue is full, and the queue holds far fewer small datagrams than its size in bytes suggests, because each datagram carries kernel overhead. A sender that writes thousands of datagrams before the receiver reads would lose most of them on Linux with default settings. So `send` drains the sockets every 16 sends into an internal list, which `receive` returns, and a burst of thousands of messages does not overflow a queue. The kernel buffers are also asked to be large, as a best effort.

Endpoints are limited to 512, because each is a descriptor and many systems allow 1,024 per process.

Validation: two endpoints exchange beacons and commands with every field intact; payloads of 0, 1, 2, 100, 1,000, 1,099, and 1,100 bytes arrive whole; 32 endpoints each send one message to every other (992 messages) and each message arrives once, at its addressee, with its payload; a burst of 6,000 messages from one endpoint to another all arrive; garbage, an empty datagram, and a datagram of 3,000 bytes are counted as malformed, a message with a false sender as foreign, a message at the wrong socket as misdelivered, a valid message from a program outside the set as foreign, and the transport delivers a good message after each; a receive times out empty after about the time given and polls with zero; misuse (no endpoints, too many, a missing endpoint, an oversized payload) is rejected and sends nothing.

Limitations: loopback only, IPv4, and one thread. Delivery order between two endpoints is not promised by UDP (it holds on loopback in practice, and nothing here depends on it: the socket channel reassembles order from its own ledger). A message is lost if the operating system drops it, which the statistics make visible as `sent` minus `received`.

## TCP transport

Source: `net/include/swarm/net/tcp_connections.hpp`. Status: unit-tested.

The transport for commands: bytes arrive whole, in order, and once. Each endpoint listens on a port the operating system chose. The first time a message goes from one endpoint to another, a connection is opened from the sender to the receiver's listener, and it is reused for every later message in that direction. A ground station commanding 40 drones holds 40 connections (a test checks the count), not a mesh. Messages use the wire format back to back on the stream. TCP keeps no write boundaries, so the receiving side reassembles them with the stream decoder, and a test writes messages one byte at a time and gets them all back.

On the receiving side the transport accepts connections and reads from them in one loop (`receive`, and `send` does the same every 16 sends). What it checks, counting what it throws away:

- The stream decodes. A corrupt stream cannot be resynchronized, so the connection is closed and counted as `broken`; the messages completed before the corruption are kept.
- A message arrived on the listener of the endpoint it is addressed to (otherwise `misdelivered`).
- Every message on one connection names the same sender, the one named by the first message (otherwise `misattributed`). Without a handshake the first message cannot be checked, so a connection can claim a false sender only on its first message.

A connection the peer closes is counted as `closed` and forgotten, and the next send in that direction opens a new one. A write that finds the connection closed opens one new connection and writes again; if that also fails, `NetError` is thrown. After the receiver closes a broken connection, the sender learns of it only from a failed write, so the first message written right after may be lost to the closed connection and later messages arrive. That follows from how TCP reports a close, and it only arises when the stream was corrupted, which needs something other than this transport writing to the port.

A sender that writes a lot before anything reads would fill the socket buffers and block, with nobody to read. Draining every 16 sends prevents it: a test writes 20,000 messages before the first receive.

Validation: a command arrives whole; 10,000 messages of random sizes arrive in the same order, byte for byte, none lost; messages written a byte at a time are reassembled; one connection is opened per pair and direction and reused; order is kept per pair when several pairs send to one receiver; a closed connection is reported, the other connections carry on, and sending reopens the closed direction; a corrupt stream is reported and closed while others carry on; a message for another endpoint and a false sender are counted and dropped; misuse is rejected and opens no connection.

Limitations: loopback and IPv4 only; endpoints are limited to 512 listeners and each used pair costs two descriptors; no TLS and no authentication; a false sender on a connection's first message is not detected; the receive side is polled, so a message is delivered when the owner next calls `send` or `receive`.

## Socket channel

Source: `net/include/swarm/net/socket_channel.hpp`. Status: integration-tested. Decision record: [0008](decisions/0008-real-sockets-under-the-channel-model.md).

A channel that carries its messages over real UDP and TCP sockets and still behaves exactly like the simulated one. Each message goes through the same `ChannelModel` as on the deterministic channel, which decides whether the rate limit, the loss model, or a jammer drops it and the tick it is delivered at. A dropped message never touches a socket. One the model lets through is serialized and sent at once, a beacon as a UDP datagram and a command over TCP. When the simulation reaches the delivery tick the channel hands it to the receiver's inbox, in the order the deterministic channel would, waiting up to a timeout (one second by default) for the bytes if they have not arrived. The order comes from a `DeliveryQueue` that holds a placeholder for each message in flight.

If a scheduled message does not arrive in time it is counted as `dropped_network` (in the channel statistics and in `timed_out`) and not delivered, and the run continues. Datagrams that arrive twice are counted as `duplicates`, and ones that were not awaited (late, or never sent by this channel) as `unexpected`. The transports' own statistics are available through `net_stats()`. A payload the wire format cannot carry is refused before anything changes, so it leaves no placeholder behind to time out later. At most 256 endpoints, since each takes two descriptors.

Validation:

- A 400-tick run of eight demo agents under burst loss, jitter, a rate limit, a pulsed jammer, and a global jammer, over real sockets, gives the final state hash 13965888112468217684 and the message-event digest 17948217017054799834, with 6,400 sent, 1,861 delivered, and 637, 3,200, and 680 dropped by loss, the rate limit, and jamming. These are the numbers recorded from the deterministic channel before its model was extracted, so the real transport reproduces the simulated channel exactly. Only the messages the model let through touched a socket; the number sent as datagrams equals the number of scheduled events.
- The state hash at every tick and every message event equal those of the deterministic channel for 1, 2, 4, and 8 threads, which also exercises reading inboxes from several threads.
- Driven with the same traffic, the two channels deliver the same messages, with the same fields and payloads, in the same order at every tick. Over 20,000 messages at a set loss of 0.3, the observed loss is 0.2929 with independent loss and 0.3019 with bursts of mean length four.
- Beacons travel as UDP and commands over TCP, counted by the transports; commands survive total beacon loss and arrive in order, matching the deterministic channel.
- With a test hook that makes the network lose every tenth message, 10 of 100 are counted as network drops, 90 are delivered, and the channel carries on and delivers 20 more afterwards.

Limitations: loopback only, so it says nothing about real latency or loss; the receive timeout is wall-clock time, which is the one way a real network can change a run; a message lost by the operating system makes the run differ from the simulated one.

## Running a scenario over sockets

Source: `core/scenario/include/swarm/scenario/run.hpp`, `world.hpp`, `apps/swarm_sim.cpp`. Status: integration-tested.

`swarm_sim run SCENARIO --out FILE --transport sockets` runs the scenario with its messages carried over real UDP and TCP sockets on the loopback interface (see the socket channel), and writes the same kind of log as a simulated run. `--receive-timeout-ms N` (default 1000) is how long the channel waits for a message's bytes at its delivery tick. The command prints the number of network drops, which is zero in a normal run, and warns on standard error if it is not.

The core cannot include the network library, so the runner takes the channel from the caller: `run_scenario` and `ScenarioWorld::make_simulation` accept a `ChannelFactory`, a function from the number of endpoints and the scenario's seeded channel configuration to a channel. Without one the run is over the deterministic channel. The program that wants sockets, here `swarm_sim`, passes a factory that makes a socket channel. A factory that returns no channel, or one with the wrong number of endpoints, is rejected. `run_scenario` can also report the channel's statistics at the end of the run.

The log does not record the transport, in the same way that it does not record the thread count: the transport is not part of what defines the run. So:

- A scenario run over sockets writes a log that is byte-identical to the log of the same run over the simulated channel. Tests compare the whole byte string for a scenario with burst loss and a jam, and for `hold_resume.yaml`, whose commands travel over TCP (twelve commands over six connections, one per drone, and the beacons as more than a thousand datagrams).
- That log verifies by re-execution on the simulated channel, on 1, 3, and 8 threads, and the log bytes are the same for 1 and 4 threads of the recording run.
- If the operating system loses a message so that the run differs, the channel counts it as a network drop and the run reports it. The log of such a run fails verification: a test makes the network lose every fiftieth message and finds a hash or snapshot mismatch.
- The command-line script records both scenarios over sockets and compares the files byte for byte with the simulated recordings, checks that bad options (an unknown transport, a missing value, a timeout that is not a number or is negative) are usage errors, and runs the verifier on the log.

Limitations: the whole run still happens in one process on one machine; sockets carry the messages but the drones are not separate programs. Loopback timing says nothing about a radio.

## Real versus simulated comparison

Source: `net/include/swarm/net/compare.hpp`, `apps/swarm_netcmp.cpp`. Status: unit and command-line tested; timings are not asserted.

`swarm_netcmp` pushes the same traffic through the deterministic channel and the socket channel and reports two things.

Equivalence. Each tick, every endpoint sends a beacon to its next two neighbors, and every 25 ticks endpoint 0 sends a command to endpoint 1 (over TCP). At every tick the messages delivered to every endpoint are compared field by field, payload included. The report says whether they were identical and, if not, where they first differed. Exit status is 0 for identical, 1 for a difference or a network loss, 2 for a usage error. `--lose-every N` makes the operating system "lose" every Nth message, to test the tool.

Cost. For UDP and for TCP separately, one message at a time on one thread, the time from the call that sends a message to the call that returns it at the receiver (after one warm-up message), as nearest-rank p50, p95, p99 and maximum in microseconds; and the rate at which a batch is sent and then received. The report prints the platform, compiler and number of logical cores, and the channel model's configured latency in milliseconds beside the measurements.

Limits: these are loopback timings on one machine and say nothing about a radio. The model's latency is a parameter, not a measurement. Run it in a Release build, and quote the platform line with any number taken from it. No test asserts a timing.

## Experiments and cell expansion

Source: `core/experiment/include/swarm/experiment/experiment.hpp`, `experiments/`. Status: unit-tested, including the planned matrix.

An experiment file (version 1, strict YAML) names a base scenario file, a first seed and a seed count, and a list of axes. An axis has a lowercase name and levels; a level has a label (letters, digits, `_` and `.`) and a `set` mapping written in the scenario format. The cells are the product of the axes' levels in file order, the last axis varying fastest; with no axes there is one cell, `base`. A cell's scenario is the base with each chosen level's `set` merged over it (maps key by key, any other value replaced), passed through the ordinary scenario loader, and named `<experiment>.<labels joined by '-'>`. Replicate `r` of a cell runs with seed `first_seed + r`, the same for every cell, so comparisons between cells are paired.

Rejected with a path and, where there is one, a line: unknown keys, a wrong version, missing keys, duplicate axis names or labels, an empty axis, a fragment that is not a mapping or sets `seed`, `name`, or `version`, two axes that write the same value, a base that cannot be read, a fragment that makes a cell's scenario invalid (reported with the cell's name and the scenario loader's message), and more than 100,000 cells. Loading takes a function that reads files, so no file access happens inside the library.

`experiments/matrix.yaml` is the planned matrix: 4 loss levels (0, 10, 30, 50 percent, independent), 3 weather (clear, fog at 200 m visibility, rain at 8 mm/h), 3 jamming (none, constant, pulsed), 3 drone counts (5, 10, 20), 2 fault settings (none, drone 2 fails at 30 s), and 2 scenes (field, hills): 432 cells, 2,160 runs at five seeds. The level values are assumptions chosen for this study, not measurements.

Limits: the cells are not run by this library; the batch driver does that. Because seeds are shared, replicates of different cells are not independent of each other.

## Batch driver

Source: `core/experiment/include/swarm/experiment/batch.hpp`, `apps/swarm_batch.cpp`. Status: unit-tested and covered by a command-line script.

`swarm_batch EXPERIMENT --out FILE` expands the experiment and runs every cell for every seed (task index = cell x seeds + replicate). Each task builds its scenario with the replicate's seed, runs it on one thread over the deterministic channel, optionally verifies the log by re-execution, and computes the metrics from the log. `--jobs N` sets the worker threads, `--seeds N` overrides the seed count, `--verify-every N` re-executes the log of every Nth task by index (default 10; 0 none, 1 all), and `--keep-logs DIR` writes each log. Progress goes to standard error.

The table has one row per task, in task order: `experiment, cell, <one column per axis>, replicate, status, verified`, then the 23 columns of the metrics CSV. The axis columns hold the level labels. A task that fails becomes a row whose status says what failed (`run failed`, `verification failed`, or `metrics failed`) and whose metric cells are empty; the other tasks still run, and the exit status is 1. A text file `FILE.meta` records the platform, compiler, counts, and the experiment text. The table contains no timing or host information.

The table is identical for any number of jobs: tasks share no state, each writes its own slot, and rows are written in index order. A unit test and the command-line script compare 1 job with several, byte for byte.

Limits: logs are held in memory per running task. Sampled verification checks the determinism of the sampled tasks only; the determinism tests cover the rest.

## Ablation switches

Source: `core/sim/include/swarm/control/drone_brain.hpp` (`fallback_enabled`, `separation_enabled`), scenario keys `control.fallback` and `control.separation`. Status: unit and integration tested.

Both default to true. `control.fallback: false` makes a drone behave as in the nominal state whatever the fallback state machine decides. The machine still runs and the drone still reports its state, so the effect of removing it can be measured against what it would have done. A commanded hold is not a fallback and still works. `control.separation: false` removes the repulsion term, so the drone feels only the formation pull. Removing prediction is not a switch: it is `control.predictor: hold_last`, which treats a neighbor as stationary at its last report, and it is the same configuration as the hold-last baseline.

The canonical scenario text, which is stored in each log header, writes a switch only when it is false, so logs of scenarios that leave the switches alone are byte-identical to what earlier builds wrote. `experiments/ablation.yaml` compares the full controller with each variant under loss, jamming, and a drone failure, and `experiments/baselines.yaml` compares the four predictors, hold-last, dead reckoning, Kalman, and the oracle.

Limits: an ablation measures what the component does in these scenarios, not what it would do in others. With fallback off there is no safe hold, so a drone that has lost its position fixes keeps flying on a drifting estimate.

## Analysis statistics

Source: `analysis/swarmsim_analysis/`. Status: tested in `analysis/tests/` on inputs whose answers are known by hand, and on a small table written by `swarm_batch`.

The table is read as `swarm_batch` writes it. The axes are the columns between `cell` and `replicate`. A metric cell that is empty means the value does not exist for that run and is read as NaN; it is never turned into zero. A run whose status is not `ok` is counted in the group's `failed` column and kept out of every statistic.

For each group (the runs sharing the levels of the chosen axes) and metric the summary has: `n`, the number of runs that have the value; `missing`, the number that do not; the mean; the standard deviation with n - 1 in the denominator; a two-sided 95 percent Student t interval for the mean, `mean +- t(0.975, n - 1) * std / sqrt(n)`; the median; and the minimum and maximum. With one value the standard deviation and interval are NaN, because one value says nothing about spread; with none, everything is NaN.

Choices and limits:

- The t interval assumes the per-seed values are roughly normal. Several metrics are not (collision episodes are counts, recovery time is censored), so the median and range are reported next to the interval, and the interval for such a metric is a rough guide.
- Recovery time exists only for runs with a disturbance, and only the runs that recovered contribute a time. `recovery_censored` is 1 when any recovery result in the run was censored; its mean over a group is the fraction of runs with a censored recovery, which belongs beside any recovery time that is quoted.
- Replicates of a cell use consecutive seeds and are treated as independent. The same seed is used in every cell, so runs of different cells are not independent. A comparison between two levels of an axis therefore uses `paired_difference`: runs are matched when they agree on every other axis and on the replicate, only pairs in which both runs succeeded and both have the metric are used, and the differences are summarized in the same way. The number of pairs is the summary's `n`.
- No significance test is run across the matrix. With hundreds of comparisons, a count of "significant" cells would mostly measure the number of comparisons.

## Figures

Source: `analysis/swarmsim_analysis/figures.py`, `make.py`. Status: tested on synthetic tables and on a real batch table; no test compares pixels.

`python -m swarmsim_analysis figures TABLE --out DIR [--where axis=level,...]` writes the figures that the table's axes support, as PNG files of 640 by 400 pixels with a fixed font, palette (Okabe and Ito), and layout, so the same table gives the same bytes:

| File | Needs | Shows |
|---|---|---|
| `error_vs_loss.png`, `delivery_vs_loss.png` | a `loss` axis | settled formation error and delivery ratio against loss level |
| `separation_vs_jamming.png`, `recovery_vs_jamming.png` | a `jamming` axis | closest approach and longest recovery time against jamming level |
| `ablation_error.png`, `ablation_separation.png` | `variant` and `loss` axes | the controller variants as bars per loss level |
| `baselines_error.png`, `baselines_separation.png` | `predictor` and `loss` axes | the same for the four predictors |

A figure whose metric has no value in any run is skipped and reported, not drawn empty. Rules every figure follows: a point is the mean over its group with a 95 percent Student t interval, a point with one run has no interval, a group with no value is left out so a line joins only neighbouring levels that were measured, series are shifted slightly sideways so equal values stay visible, and axes that the figure does not show are pooled. The subtitle states the axes that are fixed with `--where`, the axes pooled over, and the number of runs. Pooling mixes conditions, so a figure from a full matrix should be made with the other axes fixed.

Limits: level labels are placed at equal spacing in table order, not at their numeric value. Intervals inherit the limits of the statistics above.

## Replay reader and scene export

Source: `core/format/include/swarm/format/replay.hpp`, `apps/swarm_sim.cpp`. Status: unit-tested in the core and built into the Unreal plugin. It is part of `core/format`, so it has no Eigen or yaml-cpp, reports errors as status codes, and never throws.

`Replay::load` reads a whole run log from memory. It checks the container with `LogReader` (magic, version, record framing, the trailer's record count and final hash), decodes every record's snapshot, and keeps the frames: tick, state hash, and snapshot. A snapshot holds the drones' true state only on the ticks the scenario's snapshot stride selects and on the last tick; the other frames keep their message events and have no drone state. It reports, with the index of the record where there is one: a bad container or trailer (`kBadLog`, with the log status), a snapshot that does not decode (`kBadSnapshot`, with the snapshot status), record ticks that do not strictly increase, a snapshot whose drone count differs from the header's, and a log with no drone state at all. After any failure the replay is empty.

`sample(tick, out)` gives every drone's state at a fractional tick. Between two frames that hold state, the position is a cubic Hermite curve through the two logged positions with the two logged velocities, and the velocity is the curve's derivative in meters per second. A logged tick is returned exactly as logged, without going through the curve. Alive and fallback state come from the earlier frame. Nothing is extrapolated: a time before the first or after the last state frame gives that frame. Frames without state are skipped. The tests check exactness at logged ticks, that a cubic path is reproduced exactly, that constant-velocity flight is a straight line that never goes backward, and that position and velocity are continuous across a frame.

The Hermite curve is a display interpolation. It is smooth, but it is not what the simulation did between the frames, and with a snapshot stride above 1 a quick maneuver between two logged ticks is smoothed. The curve uses the logged velocity, which is the drone's true velocity.

`swarm_sim export-scene LOG --out FILE` writes the terrain and obstacles of a scenario log as a scene file (the existing `SWSC` format, see `scene.hpp`). The scene is not in the log; it is generated from the scenario in the log's header, so the file always matches the log, and exporting the same log twice gives the same bytes. A test checks that the scene rebuilt from the header equals the scene the run used (same content hash) and survives serialization. The command exits 2 for a file it cannot read, a log that is not a scenario log, or bad arguments.

## Unreal viewer: terrain

Source: `ue5-plugin/SwarmSim/Source/SwarmSim/`. Status: automation-tested in Unreal 5.4.4 without a window; manual look-and-feel checks are done by the author.

Units and axes. The plugin converts the simulation's meters and right-handed frame to Unreal's centimeters and left-handed frame in `SwarmCoordinates.h`, and nowhere else: Unreal X is the simulation's x, Unreal Y is its -y, and Z is z. The terrain test checks the orientation against numbers written out in the test, so a mirrored or transposed terrain fails.

The terrain is a Landscape actor with one vertex per heightfield sample. A Landscape's size must be a whole number of components of one or two subsections of 7, 15, 31, 63, 127, or 255 quads, so the plugin picks the layout with the least surplus (a 150 by 150 grid is exactly 10 by 10 components of 15 quads) and repeats the edge heights over any surplus, which lies outside the simulation's world and is cosmetic. Heights are stored as 16-bit values at a Z scale of 100 (0.78 cm per step, a range of plus or minus 256 m), or at a larger scale for a taller terrain. The measured difference between the stored heights and the heightfield is under 0.5 cm, and the test requires under 1 cm.

Ground layers. When a material and layer assets are given in the SwarmSim settings (the defaults name the Procedural Nature Pack Vol. 2 by PurePolygons, which is not in this repository), each vertex gets weights for grass, rock, mud, and two forest floors. Rock follows slope (the settings give the slope and the blend width), mud takes the lowest 12 percent of the height range, and the rest is grass with patches of the forest floors from value noise seeded by the scene's content hash, so the same scene always gives the same ground. Weights sum to 255 at every vertex. A layer that is missing gives its share to grass. With no assets the Landscape has the right shape and no texture.

Limits. A Landscape triangulates its quads its own way, so between grid points its surface differs slightly from the simulation's triangulation; at the grid points the heights are equal. The engine's height query returns nothing, or a neighbouring height, for a point exactly on the Landscape's outer edge, so the placement check leaves the edge samples out and the stored-height check covers them. Landscapes can only be built in the editor, so the viewer runs in the editor, not in a packaged game.

## Unreal viewer: obstacles, wind and snow

Source: `ue5-plugin/SwarmSim/Source/SwarmSim/Public/SwarmScene.h`, `Private/SwarmObstaclePlan.h`. Status: automation-tested in Unreal 5.4.4 without a window.

`ASwarmScene::BuildFromSceneFile(path)` builds the terrain (see above), then one tree for each cylinder obstacle and one rock for each box obstacle, then sets the wind and snow.

A tree is the mesh from `TreeMeshes` whose height is closest to the cylinder's, scaled uniformly by the ratio of the heights, limited to 0.4 to 3 times. Its base is put on the cylinder's base at the cylinder's x and y, and it is turned about the vertical to a yaw from a hash of its position and the scene's content hash. A rock is scaled separately in x, y, and z so that the mesh's bounds are the box, and it is not turned, because the box is axis-aligned. A box more than 1.6 times as tall as it is wide (by the mesh's own proportions) is filled with several rocks stacked up to its height, each squashed by the same amount, so that a tall box is not a stretched pillar; the stack together fills the box exactly. Meshes are instanced, one component per mesh. A mesh's center and extent are taken from its bounds, so a mesh whose pivot is not at its center is still placed correctly. With no tree or rock meshes the viewer uses the engine's cone and cube, the cone scaled to the cylinder's radius as well as its height.

The defaults of `TreeMeshes` and `RockMeshes`, and of the wind and snow collections, name assets of the Procedural Nature Pack Vol. 2 (three pines, one rock, `global_wind`, `master_snow_adjustment`). The pack is not in the repository. The wind starts at the settings' values (the pack's own defaults: intensity 2, speed 12.5 and 5, tiling 5), and `SetWind` and `SetSnowCoverage` change it at run time. Snow coverage is clamped to 0 to 1.

Limits. The simulation's cylinders are thin, so a pine looks bulkier than the cylinder the LiDAR sees, and a tall box (up to 20 m in the default obstacle settings) becomes a stack of rocks, which reads as a cairn and not as a building. Wind is a global parameter, uniform across the scene. The pack's grass and bushes are scattered by its landscape material and appear within a few tens of meters of the camera.

## Unreal viewer: playback

`ASwarmPlayback` plays a run log. It reads the log with the replay reader (`swarm::Replay`) and moves one drone for each agent along the logged path: between two logged frames the position is the replay's cubic Hermite curve, and on a logged tick it is the logged position. The viewer computes nothing about the simulation.

Display only. The log holds no attitude, so heading is the direction of the horizontal velocity (kept when the speed is under 0.5 m/s), and lean comes from the horizontal acceleration over gravity, taken over 5 ticks, split into pitch and roll, limited to 22 degrees, and smoothed with a 0.2 s time constant. Rotors spin at a fixed rate while a drone is alive. Drone color follows the logged fallback state (nominal green, degraded comms yellow, degraded navigation orange, safe hold red, failed dark gray). The drones are drawn 4 times life size by default (`-SwarmScale`) so they can be seen across a level. The drone is a stand-in built from engine cubes and cylinders, unless `-SwarmDroneModel=/Game/FOLDER` names a content folder. Every static mesh in that folder is then drawn as one aircraft, kept at the position it was authored at, centered, turned so that its +Y axis points along the heading, and scaled to a wingspan of 3.66 m before `-SwarmScale`. The pieces are not animated and take no color from the fallback state, so a model has no spinning propeller and does not show the fallback state; those need the model's parts identified, which has not been done. The folder is scanned when the drones are built, so a game finds assets that were never opened.

Coordinates are converted with `SwarmCoordinates.h`: meters to centimeters, and y negated because Unreal is left-handed. `-SwarmAnchor=X,Y` puts the simulation's origin at that point of the level, in centimeters; heights are used as they are.

The heads-up display shows the tick, the time, the speed, the number of drones alive, the camera, the seed, and the log's state hash for the frame at or before the current tick. At the last tick that hash is the trailer's final hash, which `swarm_sim` prints for the same log.

Controls: Space pauses, Up and Down double and halve the speed (1/64 to 64), Left and Right step 10 s, R restarts, 1 to 4 choose the free, orbit, follow, and cinematic cameras, Tab chooses the drone to follow.

Cameras. The playback does not use the level's own camera manager. A level can bring its own: Electric Dreams uses a Blueprint manager (`BP_OpalPlayerCamManager_C`) that ignores the view target and reported the world origin, which put the view under the terrain. The playback installs a plain `APlayerCameraManager` and sets the view target to itself. The free camera spawns a flying pawn (`ADefaultPawn`) near the first drone.

Command line, used by `USwarmPlaybackSubsystem` in a game world: `-SwarmLog`, `-SwarmSceneFile` (editor builds only), `-SwarmAnchor`, `-SwarmSpeed`, `-SwarmScale`, `-SwarmCamera`, `-SwarmDroneModel`. For checking a run without a person at the machine, `-SwarmShotDir`, `-SwarmShotDelay`, `-SwarmShotEvery`, and `-SwarmShotCount` save renders of the game by the engine's `HighResShot` command and then quit. They capture the game's own render, not the desktop. `-SwarmFrameDir`, `-SwarmFrameFps` (default 30), and `-SwarmFrameSeconds` (default 30) write numbered PNG frames for a video: the playback advances by exactly 1/fps of play time for each frame and waits until the frame is saved, so the video does not depend on how fast the machine renders. Encode them with `ffmpeg -framerate 30 -i frame_%05d.png -c:v libx264 -pix_fmt yuv420p out.mp4`. Frames are the size of the game window, and a long capture needs several GB of memory; a 703-frame capture at 3440 by 1440 ended with an out-of-memory error while a screen-recording service held most of the machine's commit limit.

Tests (`SwarmPlayback.*`, headless): the attitude, smoothing, tick, color, hash, command-line, and screenshot-schedule functions, and an actor test with a hand-made log of three drones that checks positions with y mirrored, the hash on screen, the anchor, timing, looping, and speed limits. The tests do not check how anything looks; that was checked by playing a log in the Electric Dreams level.

## Unreal viewer: scene export

`ASwarmSceneExporter` writes the scene file of the level it is in, so the simulation can run on the geometry the viewer shows (a scenario's `scene: {source: file}`). It is started by the playback subsystem when `-SwarmExportScene=PATH` is on the command line, in a game (not the editor), and it quits when done.

What it does. It moves the player's pawn to the anchor (`-SwarmAnchor=X,Y`, the level position of the simulation's origin, in centimeters), so that a World Partition level streams the area in, and waits `-SwarmExportWait` seconds (default 45). It then samples a square of `-SwarmExportSize` meters (default 1000) centered on the anchor, at a spacing of `-SwarmExportCell` meters (default 8), two rows of the grid per frame so the game keeps running, and writes a scene whose origin is the anchor. Coordinates are converted with `SwarmCoordinates.h` (y negated), and heights are the level's own, in meters.

Ground. At each sample a trace goes straight down. A trace stops at the first thing that blocks it, a tree or a boulder above the ground, so when a hit is not ground the trace starts again just below it, up to 400 times. Ground is the Landscape, or a static mesh whose name contains one of `-SwarmExportGround=A,B`: a level can have holes in its Landscape with terrain meshes (cliffs, embankments) in them. Complex collision is used, because those meshes may have none of the simple kind. A sample with no ground is filled from its neighbors; more than `-SwarmExportMaxGap` of them (default 0.01) is an error.

Obstacles. Every static mesh component that touches the area, and every instance of an instanced one, is looked at. A mesh whose name contains one of `-SwarmExportTrees` becomes a cylinder from the bottom of its bounds to the top, with a radius of `-SwarmExportTrunk` (default 0.15) of its half width, limited to 0.2 to 1.5 m. A tree under `-SwarmExportMinTree` meters (default 3) is left out. A mesh whose name contains one of `-SwarmExportRocks` becomes the box of its bounds if its largest side is between `-SwarmExportMinRock` (default 1) and 60 m. Meshes are looked up by name once for each component, because a level's instanced components hold millions of instances. The log lists the meshes in the area by count, with their class, so patterns can be chosen for a level the exporter has not seen.

Limits. A tree's crown is much wider than the cylinder the LiDAR sees; the trunk radius is a fraction of the width and does not come from the mesh. Rocks are boxes of their axis-aligned bounds, so a turned or leaning rock is a larger box than the rock. Vegetation other than trees and rocks (bushes, ferns, leaves) is not exported. The result is only as complete as what the level had streamed in and generated within the wait. A quad has no terrain following, so a quad scenario sets its waypoint heights from the exported ground. A fixed-wing scenario can use terrain-aware guidance (see below), which raises its altitude over high ground.

Checked: an automation test (`SwarmSim.Export.RoundTrip`) builds a scene into a test level with the viewer, exports it again, and compares the ground heights (within 5 cm), and the position, height, and base of each tree. `SwarmSim.Export.Plan` tests the options, name matching, grid points (with numbers written out), hole filling, the obstacles from bounds, and the assembled scene. On Electric Dreams Environment (UE 5.4.4), with the anchor at (81187, -14778) cm and a 1000 m area, the export found ground at all but 34 of 15 876 samples (ground from 10.2 to 221.8 m), 35 130 trees (hornbeam meshes at least 8 m tall) and 46 063 rocks (boulders and rock formations of at least 3 m), and the file's hash is recorded in the log of a run on it. A six-drone scenario on that file ran for 140 s in 2 s of wall time, verified on 8 threads, and its drones stayed at least 42.8 m above the exported ground and never entered a tree cylinder (nearest pass 2.0 m); both figures come from a script over the log and the scene, sampled every 25 ticks.

## Fixed-wing vehicle and camera position fix

Source: `core/sim/include/swarm/fixed_wing.hpp`, `camera_fix.hpp`. Status: unit-tested; scenario version 2 (ADR 0010). The vehicle is an ALTIUS-class parameter set, not any real aircraft, and the results are for the simulator.

Vehicle. A coordinated-turn point mass that cannot hover. Horizontal speed is held in [min_speed, max_speed]. The command's acceleration is split into along-heading, sideways, and vertical parts; the sideways part is clamped to g * tan(max_bank) and turns the heading at rate a_sideways / speed. The command's yaw rate is ignored. A drone starts already flying toward the first waypoint. There is no wind. A test checks that a constant sideways acceleration traces a circle of radius V^2 / a within 1 percent.

| Parameter | Value | Basis |
|---|---|---|
| max_speed | 25 m/s | Published: press reports give the ALTIUS-600 a maximum of 90 km/h |
| min_speed | 12 m/s | Assumed. Public sources give no stall or minimum speed |
| max_accel, max_decel | 2 m/s^2 | Assumed |
| max_bank | 35 degrees | Assumed. Typical of small fixed-wing guidance limits, not from a source for this aircraft |
| max_climb_rate, max_descent_rate | 3 and 4 m/s | Assumed |
| max_vertical_accel | 2 m/s^2 | Assumed |
| gravity | 9.80665 m/s^2 | Standard |

The ALTIUS-600 figures reported in the press (wingspan about 2.5 m, mass about 12 kg, cruise about 60 km/h) were not used as inputs, because the model is a point mass. Figures for the ALTIUS-700M conflict between sources and are not used.

Camera position fix. A fixed-wing aircraft has no LiDAR in this model. Public reports describe an EO/IR camera payload; they do not describe how the aircraft navigates without GPS, so this sensor is a modeling choice and not a description of the aircraft. The fix is the true position plus noise, or no fix. With h the height above ground and alpha the weather's extinction, the one-way transmission straight down is T = exp(-alpha * h). There is no fix when h is outside [min_agl, max_agl], when T is below min_transmission, or when a dropout draw hits. Otherwise sigma = min(max_sigma, max(sigma_floor, sigma_per_m_agl * h) / T) per axis. The extinction is the same fog and rain model the LiDAR uses (Koschmieder for fog; the rain constants are placeholders).

| Parameter | Value | Basis |
|---|---|---|
| sigma_floor_m | 3 m | Assumed. Published visual-localization systems report errors of a few meters to under 10 m (AnyVisLoc, arXiv 2503.10692; a 200 m altitude result under 8 m) |
| sigma_per_m_agl | 0.02 | Assumed. Stands for ground sample size growing with height |
| min_agl_m, max_agl_m | 10 m, 1500 m | Assumed |
| min_transmission | 0.2 | Assumed |
| max_sigma_m | 100 m | Assumed |
| dropout_probability | 0.05 | Assumed. Published systems fail on some fixes even in clear conditions |

Limits. The failure is independent per fix, not correlated in time, so a long fog bank is not modeled as a persistent bad state. The camera points straight down. There is no latency and no motion blur. No source ties airborne feature matching to visibility in meters, so the transmission threshold is the part with the least support.

## Fixed-wing orbit guidance

Source: `core/sim/include/swarm/control/orbit.hpp`, `drone_brain.hpp` (`Guidance::kOrbit`). Status: unit and integration tested; scenario version 2 (ADR 0010). A drone's guidance is `kOrbit` when its vehicle is fixed-wing; the formation guidance of a quad is unchanged and every earlier log is byte-identical.

Reference. Every drone flies a circle around a shared center, which is the scenario's one mission waypoint. With omega = speed / radius, slot i of n at time t has radius R_i = radius + (i mod 2) * radius_step, altitude z_i = center altitude + i * altitude_step, angle 2 pi i / n + omega t (counterclockwise), and speed omega * R_i. Every drone has the same angular rate, so the angular spacing does not drift. The reference is a pure function of the tick, as the mission path is, so every drone computes everyone's position without a message and nominal flight needs no communication. This is a shared-clock scheme. It differs from guidance-field methods in which phase comes from communication (for example Garcia de Marina et al., arXiv 1703.07736). No source was found that compares the two under message loss.

Tracking. From its own pose estimate a drone steers toward the tangent of its circle, cut inward or outward in proportion to its radial error (at most 45 degrees), adjusts its speed in proportion to its phase error (behind the reference means faster), and adds the centripetal acceleration of a steady circle as feedforward. Altitude is a damped spring to its slot altitude. The vehicle's own limits clamp the result.

Phase consensus. Added after the first probe runs showed that nothing a drone heard ever changed where it flew (see below). The phase error that a drone steers out is a blend, (1 - w) * clock + w * consensus, with w the `consensus_weight` (default 0.5; 0 flies the clock alone and 1 the neighbors alone). The consensus term takes each usable neighbor's estimated angle around the center, subtracts the angle that neighbor is expected ahead of this drone on the reference, and compares the result with this drone's own angle. The terms are combined as a circular mean weighted by 1 / (sigma^2 + floor^2), with sigma the neighbor's position uncertainty and the floor 5 m (an assumption). A neighbor that has gone silent is predicted by the neighbor predictor. A predictor whose uncertainty grows with the age of the report (Kalman, dead reckoning, turning) gives a stale neighbor a low weight, which hands control back to the clock; hold-last keeps the report's own sigma and so keeps trusting a stale report. With no usable neighbor the clock gets all of the weight.

Separation. For vehicles that cannot stop, repulsion is replaced by a closest-approach rule. For each neighbor estimate the closest approach inside the lookahead is predicted, and when the predicted miss distance is inside the margin (the safe distance widened by the neighbor's position uncertainty, capped) the drone is pushed away from it. The higher slot applies the whole push and the lower slot half, so a pair does not both dodge the same way. An exactly head-on encounter is resolved by each aircraft turning right. Every neighbor with an estimate counts, silent or not, because an aircraft keeps moving.

Fallback. The states and thresholds are the quad's. The state's gain scale and margin scale apply; its speed scale and reference boost do not. Safe hold, and a ground-station hold command, fly a circle of the orbit's radius through the aircraft's position at the moment the hold starts, with the center to the left of its heading and the altitude held. A resume returns to the shared orbit.

Start. A fixed-wing scenario starts every aircraft on its orbit slot with the jitter added, flying along the orbit. The grid keys of the `start` section do not apply. Launch and joining the orbit are not studied; a swarm launched in a grid at one altitude had to cross itself to reach its circles and came within 2 m of itself in the first seconds.

| Parameter | Default | Basis |
|---|---|---|
| radius | 150 m | Assumed. Must give a sideways acceleration within 80 percent of the bank limit; the loader checks |
| radius_step | 12 m | Assumed |
| altitude_step | 10 m | Assumed. Setting it to 0 removes the stacking and is a planned ablation |
| phase_gain, radial_gain, velocity gain, altitude gains | 0.05, 1, 1, 0.5 and 1.5 | Assumed; not tuned beyond the first working values |
| safe_distance_m | 30 m | Assumed; the margin cap is twice this |
| lookahead_s | 8 s | Assumed |

Why the consensus and the failure model exist. In a first probe (eight aircraft, three seeds, phase from the clock alone, a failed aircraft frozen in the air) the four predictors, jamming, and packet loss changed nothing about how the swarm flew, because nothing a drone heard entered its control except the separation rule, which rarely engaged. One failure also dropped the minimum separation from about 109 m to 15 m, or 0.5 m without altitude layers, because the others flew through an aircraft hanging in the air, which a real aircraft cannot do. The consensus term and the uncontrolled-flight failure model are the two changes made in response. The results with both are in [docs/phases/fixed-wing-vehicle.md](phases/fixed-wing-vehicle.md).

Limits. No path planning, wind, or area search; the orbit center does not move; the mission may have only one waypoint. A failed aircraft flies on straight at its last horizontal speed and climb rate, silent and with no command, and never descends, turns, or stalls, which is also a simple model. Nothing in the model senses a neighbor directly: separation and consensus use only what neighbors report, so a swarm whose members are all blind has no independent view of how close they are. The formation metrics are explained under orbit metrics.

## Turning neighbor predictor

Source: `core/sim/src/estimation/neighbor_predictor.cpp` (`PredictorKind::kTurning`, scenario `control.predictor: turning`). Status: unit-tested. The report format is unchanged.

The Kalman and dead-reckoning predictors assume straight flight, which is wrong for an aircraft on an orbit. This predictor assumes constant speed and constant turn rate. Reports carry no turn rate, so it is taken from the reports themselves: the change of the reported heading (the direction of the horizontal velocity) from the oldest report in a 3 second window to the newest, divided by the time between them. It is used once the reports span 1 second and is limited to 0.8 rad/s; until then the neighbor is predicted as flying straight. The prediction extrapolates the newest report along the arc of that turn rate (horizontal) and at constant climb rate. The position uncertainty is the report's sigma, the velocity-noise drift as in dead reckoning, and a term for the turn-rate uncertainty, 0.5 * speed * sigma_omega * t^2, which dominates after a few seconds. The window, the minimum span, the rate limit, and sigma_omega (0.03 rad/s) are assumptions.

Checked against a neighbor on a true circle (radius 150 m, speed 17 m/s) after 4 seconds of reports every 0.1 s: with exact reports the predicted position stays within 0.5 m of the circle for 10 seconds, while dead reckoning is off by more than ten times that. By calculation a straight-line prediction of that circle is about 93 m off after 10 seconds. With noisy reports (position 2 m, velocity 0.3 m/s) the turning prediction is closer than dead reckoning at 5 and 10 seconds, but the error is larger than with exact reports, because the turn rate from noisy headings is itself noisy.

Limits. The turn rate is a single constant over the prediction; an aircraft that changes its turn rate (leaving an orbit, avoiding a neighbor) is mispredicted, and the uncertainty term is not calibrated against such changes. The predictor has not yet been compared with the others inside a swarm run; the benchmark does that.

## Orbit metrics

Source: `core/scenario/src/metrics.cpp`, `metrics.hpp`. Status: tested on real runs and on the CSV layout. Computed from the log alone, like every other metric.

For a fixed-wing scenario the formation error is measured against each aircraft's point on the orbit at the time of the state, not against a slot on the mission path, because a quad's formation slots mean nothing for aircraft on circles. The columns `formation_rmse_m` and `formation_rmse_settled_m` keep their names and now hold that error, and the recovery metrics use the same error, so a recovery after a jam or a failure is the time for the swarm to get back within the threshold of its orbit references. Four columns are added at the end of the CSV and are empty for a quad scenario:

| Column | Meaning |
|---|---|
| `orbit_radial_rmse_m` | RMS of the horizontal distance from the center minus the aircraft's orbit radius |
| `orbit_altitude_rmse_m` | RMS of the altitude minus the aircraft's slot altitude |
| `orbit_phase_rmse_m` | RMS of the arc length of the phase error, radius times the angle from the aircraft to its reference; positive means behind |
| `time_on_station` | The fraction of (aircraft, record) samples within 30 m of the orbit reference point; with terrain guidance only the horizontal distance counts, because the altitude is raised over high ground |
| `min_agl_m` | The lowest height above the terrain of any aircraft in any record. Needs the scene: generated from the scenario, or read from the file it names; empty when a scene file is missing |
| `ground_contacts` | The number of aircraft that were at or below the ground in some record; with `ground_contact_crashes` it is the number that crashed |

The first four are over every alive aircraft and every record with state. The 30 m station radius is a constant, not a scenario setting, and is an assumption. `min_separation_m` and `collision_episodes` are the same for both vehicles. Failed aircraft are excluded from the error, as for a quad, but a failed aircraft is frozen where it stopped and still counts in the separation, which is the stated simple failure model and not what a real aircraft would do.

The first columns of the CSV are unchanged, so existing tables still parse; code that reads columns by position past `latency_p95_s` is the only thing affected. The batch tables and the Python analysis package (`swarmsim_analysis`) carry the new columns, and the figure labels know their names.

What the first runs show (`scenarios/fixed_wing_orbit.yaml`, 60 seconds, from the tests): an established swarm has formation error under 30 m, radial and altitude errors under 20 m, phase error under 30 m, and is on station more than 90 percent of the time; dense fog that blinds the camera raises the phase error. These are the thresholds the tests assert, not measured benchmark results.

## Wind

Source: `core/sim/include/swarm/wind.hpp`. Status: unit-tested; scenario version 3 (ADR 0011).

The air is a steady wind vector plus a gust field. The gust field is a sum of eight traveling waves. Each wave has a seeded direction, a seeded phase, and a wavelength drawn log-uniformly between a fifth of `gust_scale` and `gust_scale`. The amplitudes are set so that each horizontal gust component has the rms `gust_sigma`, and the vertical rms is `vertical_ratio` times that. The waves are carried along by the steady wind (frozen turbulence) and also change slowly on their own, at a rate proportional to their wavenumber and the sigma. The result is a function of position, time, and the run seed alone, so every aircraft sees the same air and the thread count cannot matter. Every aircraft builds the same field from the seed; nothing is shared.

This is not a turbulence spectrum. It has the requested rms and a largest scale and nothing more; it is not Dryden or von Karman, and the vertical component is independent of the horizontal ones. `mean_mps` is the direction the air moves toward, in the simulation's x and y. The default is no wind, and a scenario without wind steps its vehicles as before.

A quad is not moved by wind. A fixed-wing aircraft flies in the air mass: its air velocity is its ground velocity minus the wind at its position and tick, and the wind is added back after the step. A gust therefore changes the airspeed at once, and a headwind lost to a gust costs the aircraft lift. The aircraft has no wind estimate, and its controller works in ground velocity, so a crosswind makes the ground track drift until the controller corrects it.

Checked: the rms of the field over many seeds and places is within 25 percent of the sigma; the field is smooth within bounds that follow from the amplitude and shortest wavelength; the gusts move with the mean wind; the field is identical for equal seeds and differs for different seeds.

## Fixed-wing energy model

Source: `core/sim/include/swarm/fixed_wing_energy.hpp`. Status: unit-tested; scenario version 3, `vehicle.dynamics: energy` (ADR 0011). The default `simple` is the model of the section above and is unchanged.

A point mass flown in the air mass with thrust, drag, and a lift limit. From the air velocity it takes the airspeed V, the heading, and the flight-path angle. The command's acceleration is the controller's wish, read in the frame of the heading:

- The sideways part sets a bank angle atan(a / g), within the bank limit. The bank follows its command with a first-order lag (`roll_tau`) and a roll-rate limit, and starts each step from the bank that the last turn rate implies. The heading changes at g tan(bank) / V.
- The bank is also held below acos((V_s / V)^2), where V_s is the stall speed in level flight, so the load factor 1 / cos(bank) does not stall the wing at the current speed. The limit matters only between the stall speed and about 1.1 times it, because the autopilot speeds up quickly there.
- The vertical part becomes a climb-rate wish, then a flight-path angle that follows its command with a lag (`pitch_tau`) and a rate limit. The angle is capped by the thrust left after drag, with a small allowance for spending speed above the hold speed, so an aircraft that cannot climb is not asked to stall in the attempt.
- Thrust makes up the drag and the weight along the path, plus the speed change that the along part of the command asks for, within min(max_thrust, max_power / V). Below the hold speed for the current load factor the autopilot raises thrust to recover speed.
- Below the stall speed for the load factor the wing cannot carry the weight and the path angle falls.
- Drag is q S (CD0 + k CL^2), with CL = n W / (q S), k = 1 / (pi AR e), and AR = wingspan^2 / wing area. Airspeed above `max_speed` is cut back, as if by brakes.

The minimum speed of the aircraft is not a free setting under this model. It is the hold speed, `stall_margin` times the stall speed sqrt(2 m g / (rho S CLmax)), and the loader writes it into the vehicle's `min_speed`, which the orbit controller uses as its lower speed limit. A scenario that sets `min_speed_mps` together with `dynamics: energy` is an error.

| Parameter | Value | Basis |
|---|---|---|
| mass | 12 kg | Published: press reports for the ALTIUS-600 give about 12 kg |
| wingspan | 2.54 m | Published: press reports for the ALTIUS-600 |
| wing area | 0.9 m^2 | Assumed. Chosen with the maximum lift coefficient so that the stall speed is about 12.4 m/s, below the published cruise of 60 km/h |
| cl_max | 1.4 | Assumed |
| cd0, oswald | 0.035, 0.8 | Assumed. Typical of a small clean fixed-wing UAV, not from a source for this aircraft |
| max_thrust, max_power | 60 N, 600 W | Assumed. Chosen so that the power left over at cruise supports a climb of a few meters per second, near the 3 m/s climb limit of the simple model |
| stall_margin | 1.15 | Assumed |
| max_roll_rate, roll_tau | 1.2 rad/s, 0.3 s | Assumed |
| pitch_tau, max_path_rate, max_path_angle | 0.6 s, 0.4 rad/s, 0.35 rad | Assumed |
| max_speed, max_bank, climb and descent limits | as the simple model | See the section above |
| air density | 1.225 kg/m^3 | Standard sea level |

Everything marked assumed was chosen to give plausible behavior, not fitted to the aircraft. The model has no propeller model, no angle of attack, no sideslip, no ground effect, and no altitude dependence of density. The roll-in is a lag on the bank, not a roll-moment model. A test checks level flight at 17 m/s for 30 s (speed within 0.05 m/s), the coordinated turn rate of a held bank, that roll does not jump, that a turn costs speed when the thrust is at its limit, that a climb is limited by the thrust left after drag, the nose dropping below the stall speed, the drift of the ground track in wind, and the effect of a gust on the airspeed.

## Terrain-aware guidance

Source: `core/sim/include/swarm/control/terrain_map.hpp`, `drone_brain.cpp`. Status: unit and integration tested; scenario version 3, `control.terrain` (ADR 0011).

The aircraft carries a map: a copy of the scene's heightfield, treated as prior elevation data and not a sensor. The scenario builder makes it from the scene the run uses, and it is exact. The map answers one question, the highest sample whose cell touches a disc, which bounds the height of the mesh in the disc (a cell's mesh never exceeds its highest corner).

The altitude command of an orbiting aircraft is the larger of its nominal altitude and a terrain floor, plus the slot's offset from the bottom layer so the layers stay apart. The floor is the maximum, over points along the path ahead, of

    highest ground within `footprint` of the point + `clearance` - climb_budget * distance / speed

where distance is along the path to the point and climb_budget is `climb_fraction` times the vehicle's climb limit. A ridge far ahead lowers the floor by the height that a climb at the budget rate gains before reaching it, so the aircraft starts climbing early enough and no earlier. The path is the whole circle (or stadium) once around from the aircraft's nearest point. While the aircraft is more than a quarter of the radius off its path, the straight line along its heading is checked too. Safe hold uses the same floor over its circle. A pattern uses the stadium path.

Because the path is the whole loop, a loop over uneven ground is flown at about the altitude of its highest point plus the clearance, less what the budget climb gains over the loop. It is not terrain following: the aircraft does not descend into valleys to keep a constant height above ground. An aircraft starts at the altitude that clears the ground under its start.

| Parameter | Default | Basis |
|---|---|---|
| clearance_m | 60 | Assumed. Covers trees and the error of an aircraft's altitude estimate |
| footprint_m | 40 | Assumed |
| climb_fraction | 0.6 | Assumed. The part of the climb limit a planned climb may use |
| sample_spacing_m, lookahead_m | 20, 600 | Assumed |

Limits. The map has no error and no obstacle: trees and rocks are covered only by the clearance, and a scene's boxes and cylinders are not in the map. A ridge steeper than the budget climb that appears within the aircraft's turning circle cannot be cleared. The orbit altitude error metric includes the climb. With `environment.ground_contact_crashes` an aircraft that reaches the ground stops there for good and stays silent, which is how a run shows that a scenario without terrain guidance flew into the ground.

## Patterns

Source: `core/sim/include/swarm/control/path.hpp`. Status: unit and integration tested; scenario version 3, `control.pattern` (ADR 0011). Public sources do not describe what the real aircraft fly, so these are standard patterns and assumptions. The only formation fact used is that launches were reported 10 to 15 s apart (press).

A stadium path is two straights of `leg_length` joined by semicircles of radius R; with a zero leg length it is the circle of the orbit. Position, tangent, and curvature are closed-form in the distance along the path, and the nearest point of a path to a position is found per part.

- `racetrack`: every aircraft flies a stadium around the center, counterclockwise. Odd slots fly a stadium wider by `orbit.radius_step`, and the layers stack by `orbit.altitude_step`, as for orbits.
- `lanes`: each aircraft flies its own stadium, centered across the heading `lane_spacing` apart, with adjacent lanes flown in opposite directions. The loader requires the spacing to exceed twice the turn radius plus the safe distance.

Reference. Every slot completes a loop in the same time, the length of slot 0's loop over the mission speed, so a longer loop is flown faster and the spacing along the loops does not drift. With `stagger_s` above zero, aircraft i is i times that many seconds of flying behind aircraft 0; with zero, the aircraft are spread evenly along the loop. The loader checks that each slot's speed is inside the vehicle's limits and each turn needs under 80 percent of the bank limit.

Tracking. A path-following law like the orbit's: the nearest point gives the tangent, the curvature, and the cross-track error e. The desired direction is the tangent turned toward the path in proportion to the error (at most 45 degrees); the desired speed is the slot's speed adjusted in proportion to the phase error (the distance along the path to the reference point); the horizontal acceleration is the centripetal term v^2 times the curvature plus a velocity-error term; the vertical part is the orbit's. An aircraft more than `join_fraction` times the turn radius from its path flies at the nominal speed toward it and does not try to hold its phase. Separation is the orbit's closest-approach rule.

Limits. Phase consensus over the neighbors is used only by the orbit pattern: a racetrack and lanes fly their phase from the shared clock, so with them the neighbor predictors and the comms conditions affect only separation. There is no launch: aircraft start on their paths. There is no lawnmower with a return leg, which would need a path planner (ADR 0010 and 0011 leave it out). Metrics measure the cross-track error and the distance along the path to the reference point; the altitude reference is the nominal layer.
