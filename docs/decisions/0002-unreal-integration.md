# 0002. Compile only core/format into the Unreal plugin

Date: 2026-10-16. Status: accepted.

## Context

Unreal 5.4 builds modules with its own build tool, a C++20 default, and RTTI and exceptions off. The simulation core uses Eigen and yaml-cpp, and a prebuilt static library built outside Unreal risks CRT and RTTI mismatches. The plugin only needs to read run logs and scenario geometry.

## Decision

Split the core into `swarm_format` (log schema and hashing, no third-party dependencies) and `swarm_core` (everything else). The plugin compiles the `swarm_format` sources directly, through a directory junction at `Private/swarm_format` created by `scripts/link_ue.ps1`. The plugin does not link `swarm_core`.

If the plugin later needs the full simulation, build it as a static library with the dynamic CRT and consume it through an external module.

## Consequences

- No second build system inside Unreal and no CRT mismatch.
- `core/format` must stay free of Eigen and yaml-cpp and must compile as C++20.
- Windows macros such as `check`, `verify`, `min`, `max` can clash with core headers. Include core headers only from `.cpp` files in the plugin, never from headers.
- The junction approach was checked against UBT 5.4.4: it follows the junction and compiles the sources. Compiling them in Unreal showed that the module must not use unity builds (`bUseUnity = false`), because `log.cpp` and `scene.cpp` define `kMagic` in anonymous namespaces and collide when merged, so `core/format` must stay buildable as separate translation units.
