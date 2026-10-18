# 0004. Eigen for estimation, fixed-size types only

Date: 2026-10-18. Status: accepted.

## Context

Phase 3 adds Kalman filters with states of 3 to 9 dimensions. They need matrix products, a symmetric solve, and a way to check positive semi-definiteness in tests. The options were a dependency, or a small matrix class written for the project.

## Decision

Use Eigen 3.4.0 with fixed-size vectors and matrices only.

- Fetched at configure time from a pinned URL and checked against a SHA-256 of the archive. The archive was downloaded twice to confirm it is byte-stable before the hash was recorded.
- Header-only, exposed through one interface target (`swarm::eigen`) as system includes, so the project's strict warnings and `-Werror` do not apply to Eigen's headers. `EIGEN_MPL2_ONLY` is defined so only MPL-2 licensed parts are used.
- Eigen's own build is not configured: the fetch declares a source subdirectory that does not exist, which downloads the sources without adding Eigen's tests and docs targets.
- Used only in `core/sim`. `scripts/check_core_invariants.sh` fails if anything under `core/format` includes Eigen or yaml-cpp, since that directory is compiled into the Unreal plugin.

## Alternatives considered

- A hand-written small matrix class: removes the dependency, but adds numerical code to test and maintain, and gives up Eigen's reviewed implementations.
- Dynamic-size Eigen types: unnecessary, since every state dimension is known at compile time, and they would allocate on the heap in the per-tick path.

## Consequences

- Results are reproducible for one binary on one platform and any thread count, because each filter runs on a single thread. Eigen's vectorized kernels can differ between platforms, so results are not promised to match across them, consistent with [DETERMINISM.md](../DETERMINISM.md).
- Compile times for files that include Eigen increase.
- Changing the Eigen version means changing the URL and the hash together.
