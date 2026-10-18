# 0003. Scene data lives in core/format, scene logic in core/sim

Date: 2026-10-17. Status: accepted.

## Context

The LiDAR casts rays against a generated scene of terrain and obstacles. The Unreal plugin has to draw the same scene. The plugin compiles only `core/format` (see [0002](0002-unreal-integration.md)), which must stay free of third-party libraries and C++ exceptions, so it cannot call the generator or the ray caster in `core/sim`.

A second risk: if the viewer builds terrain with a different triangulation than the LiDAR uses, the viewer shows ground the sensor never saw.

## Decision

- `core/format` holds `SceneData` (a heightfield and a list of box and cylinder obstacles), its binary serialization, validation, and a content hash. Errors are status codes, and the code works on in-memory buffers.
- `core/sim` holds the terrain generator, obstacle placement, and `Scene`, which wraps a `SceneData` with a spatial index for ray casting.
- Terrain is a triangle mesh with one fixed layout: each cell splits into `(a, b, c)` and `(a, c, d)`. Height queries, ray casts, and the Unreal mesh all use it. Bilinear interpolation is not used anywhere.
- Terrain noise uses integer hashing and basic arithmetic only, so a scene generated on any compiler or platform has the same content hash. A golden-hash test enforces this.

## Consequences

- The plugin can rebuild the exact scene from serialized data without the generator.
- Embedding the scene in the run log is deferred to the scenario runner (Phase 5) and will bump the log version then.
- Anything that uses `sin`, `cos`, `log`, or `exp` (LiDAR beam directions, Gaussian noise, attenuation) is reproducible on one binary and platform but not guaranteed identical across platforms. [DETERMINISM.md](../DETERMINISM.md) says so.
- Obstacles are limited to axis-aligned boxes and vertical cylinders, and terrain to a single-valued height surface. Caves and overhangs would need a different representation.
