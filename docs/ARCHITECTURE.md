# Architecture

The scope and order of work are in [PLAN.md](PLAN.md). This document describes how the pieces fit together and where they live.

## Layout

| Path | Contents |
|---|---|
| `core/format` | Run log, scene data (heightfield and obstacles) with serialization and hashing. No Eigen, no yaml-cpp, no exceptions. Also compiled into the Unreal plugin. |
| `core/sim` | Clock, RNG, lockstep pool, channel with loss, jitter, rate limit, and jamming, drone dynamics, IMU, terrain and obstacle generation, ray casting, LiDAR, weather. Estimation, control, and metrics come in later phases. |
| `net` | Wire protocol and the UDP/TCP channel. |
| `apps` | `swarm_sim` runner. Batch driver and replay tool come later. |
| `ue5-plugin/SwarmSim` | Unreal Engine 5.4 plugin: builds the scene from a scenario, replays logs. |
| `UnrealHost` | Minimal Unreal project used to develop and run the plugin. |
| `scenarios` | YAML scenario files. |
| `analysis` | Python scripts that read CSV metrics and produce figures. |
| `tests` | `unit`, `property`, `determinism`, `integration`. |

## Dependency direction

```
apps -> net -> core/sim -> core/format
ue5-plugin -> core/format (sources only)
analysis reads CSV output, nothing else
```

Nothing in `core` depends on `net`, `apps`, or Unreal.

## Tick order

Each drone runs the same steps every tick on a worker thread, then all threads meet at a barrier:

1. Deliver messages whose delivery time is at or before the current tick, ordered by (sim time, sender id).
2. Sense (IMU, LiDAR with weather).
3. Estimate (pose Kalman, neighbor Kalman).
4. Decide (formation, separation rule, fallback state machine).
5. Send.

At the barrier the clock advances, the state is logged, and the state hash is updated.

## Interfaces

Models sit behind interfaces chosen from the scenario file: channel, LiDAR model, estimator, formation, avoidance, scene. A new implementation is a new class, not a change to its callers.

## Unreal integration

See [decisions/0002-unreal-integration.md](decisions/0002-unreal-integration.md).
