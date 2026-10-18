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
