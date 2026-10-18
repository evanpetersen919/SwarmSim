# 0005. A drone agent is a vehicle plus a brain that cannot see the truth

Date: 2026-10-18. Status: accepted.

## Context

The project's central claims depend on controllers acting on what a drone could actually know: its own estimate and what its neighbors told it. A simulated truth is always available inside a simulator, and it is easy for a controller to read it by accident or for convenience, which would silently inflate every result. Phase 3 enforced this for estimators with an include whitelist. Phase 4 adds controllers and a drone agent that has both truth and decisions in one object.

## Decision

Split the drone agent in two.

- **Vehicle side** (`DroneAgent`): owns the simulated truth and the dynamics, and runs the sensor models, which are allowed to read truth. It produces sensor outputs and applies a command.
- **Brain** (`DroneBrain`, under `control/`): estimation, neighbor tracking, formation, separation, and the fallback state machine. Its inputs are an IMU sample, a position fix, received message payloads, and the tick. Its outputs are a command and a report to broadcast.

The include whitelist that already keeps estimators away from truth now covers `core/sim/**/control/` as well. Controllers may include control and estimation headers, the sensor output types, vector types, the byte helpers, and the command type, and nothing else from the project. `scripts/check_core_invariants.sh` fails CI otherwise, and tests run it against fixtures with a deliberate violation.

The command type moved to its own header, `drone_command.hpp`, so a controller can name its output without including the truth-side `dynamics.hpp`.

## Consequences

- A controller cannot read truth without failing CI, so a result cannot depend on it by accident.
- The brain is a plain object with plain inputs, so it is unit-tested with synthetic sensor data and no simulation.
- Vehicle limits the brain needs (maximum acceleration and speed) are configuration, as they would be on a real airframe.
- The one-time launch pose given to the pose estimator is configuration and remains the only truth-derived input.
- The agent is a little more code than a single class, in exchange for the boundary being structural.
