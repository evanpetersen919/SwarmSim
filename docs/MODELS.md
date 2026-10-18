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
