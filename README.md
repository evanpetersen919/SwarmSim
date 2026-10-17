# SwarmSim

Deterministic C++17 simulation and benchmark for drone-swarm behavior under degraded communication and perception: bursty packet loss, weather-degraded LiDAR, neighbor state prediction, and a fallback state machine. Runs are reproducible from a seed. An Unreal Engine 5.4 plugin visualizes recorded runs.

[![ci](https://github.com/evanpetersen919/SwarmSim/actions/workflows/ci.yml/badge.svg)](https://github.com/evanpetersen919/SwarmSim/actions/workflows/ci.yml) [![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

<!-- TODO: hero GIF from the UE5.4 plugin, about 800 px wide, under 5 MB.
     Caption: scenario, drone count, seed.
![N-drone formation under burst loss in fog, seed 42](docs/img/hero.gif) -->

Status: early development. The repository currently contains the build system, CI, the simulation clock, a seeded random number generator, state hashing, and their tests. Everything else described here is planned; see [docs/PLAN.md](docs/PLAN.md) for scope and order of work.

## Results

<!-- TODO: fill only from the output of scripts/run_benchmarks. Name the hardware,
     compiler, commit, and seed count. Delete this section if numbers are not final.

| Scenario | Loss model | Weather | Drones | Formation error, median (IQR) |
|---|---|---|---|---|
-->

No results yet.

## Build and run

Requirements: CMake 3.22 or newer and a C++17 compiler (gcc, clang, or MSVC).

```sh
cmake --preset release
cmake --build --preset release
ctest --preset release
```

Presets for sanitizer builds are `asan-ubsan` and `tsan` (GCC or Clang). See [CMakePresets.json](CMakePresets.json). Setup notes for Windows and Unreal are in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

The runner is a placeholder that draws seeded random numbers and prints a state hash:

```sh
./build/release/apps/swarm_sim --seed 42
```

The same seed prints the same hash on the same platform. The scenario runner, recording, and replay are not implemented yet.

Docker: `docker build -t swarmsim . && docker run --rm swarmsim`.

## Design

This section describes the intended design. Most of it is not implemented yet.

Each tick runs in a fixed order: deliver messages, sense, estimate, decide, send. Drone logic runs on a thread pool and all threads meet at a barrier at the end of the tick. Messages are delivered in order of simulation time, then sender id, so thread scheduling does not change the result.

Determinism comes from a single fixed-step simulation clock, seeded random streams per subsystem, ordered message delivery, and no wall-clock reads in the core. It is not guaranteed across different compilers, optimization flags, or CPU architectures that change floating-point behavior.

| Component | What it does | Location |
|---|---|---|
| Simulation core | Clock, thread pool, drones, sensors, comms model, estimation, control, metrics | [core/](core/) |
| Network backend | Wire protocol and a UDP/TCP channel with the same interface as the simulated one | [net/](net/) |
| Apps | Scenario runner, batch driver, replay tool | [apps/](apps/) |
| Unreal plugin | Builds the scene from a scenario file and replays run logs; visualization only | [ue5-plugin/](ue5-plugin/) |
| Analysis | Python scripts that turn CSV metrics into figures and tables | [analysis/](analysis/) |

More detail: [docs/PLAN.md](docs/PLAN.md), [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), [docs/DETERMINISM.md](docs/DETERMINISM.md).

## Testing

Tests use GoogleTest and are grouped as unit, property, determinism, and integration. CI builds with gcc and clang on Linux and runs the sanitizer presets.

Not covered by CI: the Unreal plugin, which needs a local UE 5.4 install.

## Limitations

- The LiDAR weather model is a parametric degradation. It is not fitted to or validated against real sensor data, apart from one published visibility curve.
- Packet loss and jamming are modeled at the message level, not as a physical radio channel.
- The neighbor predictor is a Kalman filter with a constant-velocity assumption and degrades under maneuvers.
- Pose estimation uses a LiDAR-derived position fix, not full scan-matching localization.
- Performance numbers will come from one machine.
- There is no flight testing, hardware-in-the-loop, or autopilot integration.

## Repository layout

```
core/         simulation library (no Unreal dependency)
net/          wire protocol and UDP/TCP channel
apps/         command-line programs
ue5-plugin/   Unreal Engine 5.4 plugin
scenarios/    scenario definitions (YAML)
analysis/     Python analysis scripts
tests/        unit, property, determinism, integration tests
docs/         plan, architecture, decisions
```

## Related work

Swarm-LIO2 (decentralized LiDAR-inertial swarm odometry) and ns-3 based UAV co-simulators such as CORNET and FANS cover nearby ground. This project differs in focus: a seeded, replayable benchmark of how prediction and fallback behavior hold up under burst loss and weather-degraded sensing.

## License

MIT. See [LICENSE](LICENSE).
