# 0001. Deterministic headless core, Unreal as viewer

Date: 2026-10-16. Status: accepted.

## Context

The project needs reproducible benchmark numbers, headless runs by the hundreds, and a visual demo. Unreal ticks at a variable rate and cannot run thousands of seeded simulations quickly.

## Decision

All simulation, sensing, comms, estimation, control, and metrics live in a C++17 core with no Unreal dependency. The core runs on a fixed-step clock and produces a binary log. The Unreal plugin builds the scene from the scenario file and replays the log.

## Consequences

- Runs are repeatable and can be batched without a GPU or editor.
- The scene exists in two forms (core geometry for LiDAR and collision, Unreal meshes for display), generated from the same scenario and seed.
- Unreal features such as physics and its own ray tracing are not used.
