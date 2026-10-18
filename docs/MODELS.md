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

Source: `core/sim/include/swarm/weather.hpp`. Status: unit-tested. Fog relation standard but not yet validated against data; rain constants are placeholders.

Weather sets an extinction coefficient `alpha` in 1/m:

```
alpha_fog  = 3.912 / visibility                    (Koschmieder, 2% contrast threshold)
alpha_rain = rain_coefficient * rate^rain_exponent  (rate in mm/h)
alpha      = alpha_fog + alpha_rain
intensity  = reflectivity * exp(-2 * alpha * range)  (out and back)
```

A return is detected when its intensity reaches `intensity_threshold`, so the maximum detectable range of a surface is `ln(reflectivity / threshold) / (2 * alpha)`. With the default reflectivity of 0.6 for obstacles, a threshold of 0.05, and a visibility of 200 m, that is about 63 m.

The rain constants (`rain_coefficient = 4e-4`, `rain_exponent = 0.6`) are placeholders chosen to give an extinction of a few per kilometer in moderate rain. They are not from a cited source and have not been checked against data. With them, rain has a modest effect on strong targets within 100 m (100 mm/h pushes the detection limit of a 0.6-reflectivity target to roughly 190 m), while fog is the dominant effect. Do not draw conclusions about rain until the constants are replaced with sourced values.

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
