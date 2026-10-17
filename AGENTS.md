# SwarmSim

C++17 simulation core in `core/` with no Unreal dependency. Unreal Engine 5.4 viewer plugin in `ue5-plugin/`. Scope and order of work are in `docs/PLAN.md`; read it before changing scope.

## Commands

    cmake --preset debug && cmake --build --preset debug && ctest --preset debug
    cmake --preset asan-ubsan && cmake --build --preset asan-ubsan && ctest --preset asan-ubsan
    cmake --preset tsan && cmake --build --preset tsan && ctest --preset tsan
    bash scripts/check_core_invariants.sh
    clang-format -i <changed files>

## Invariants

- No wall-clock time, `random_device`, `rand`, or global RNG in `core/`. Time is the simulation tick only. Use `swarm::Rng` with an explicit seed and stream id. `scripts/check_core_invariants.sh` enforces this; a deliberate exception needs an `allow-wallclock: reason` comment on the same line.
- Results must not depend on thread scheduling or thread count. Messages are delivered in (sim time, sender id) order.
- Do not iterate `unordered_*` containers in any code path that affects results. Sorts need a total order.
- Never use `-ffast-math`. Keep `-ffp-contract=off` (`/fp:precise` on MSVC).
- Estimators must not read ground truth. Ground truth is for metrics only.
- `core/` never includes Unreal headers. Unreal code never includes Eigen or yaml-cpp. Only `core/format` is compiled into the plugin, and it must stay free of Eigen and yaml-cpp.
- Core headers must also compile as C++20, because Unreal builds them that way.
- Any change to the binary log format needs a version bump and an update to the golden files.
- Models are selected through interfaces from the scenario file. Metrics are computed from logs, not inside the simulation.

## Style

2-space indent, 100 columns, warnings are errors, no raw `new` or `delete`. Match the surrounding code. No emojis anywhere, including docs and commit messages.

## Documentation

Write plainly: no hype words, no marketing tone. State numbers only when a command in the repo produces them. Update `docs/PLAN.md` change log when scope changes.

## Do not touch without asking

`tests/golden/`, anything under `**/Binaries`, `**/Intermediate`, `**/Saved`, and the junctions created by `scripts/link_ue.ps1`.
