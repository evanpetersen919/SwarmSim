# Determinism

## Guarantee

Given the same binary, platform, scenario, and seed, a run produces the same per-tick state hashes, independent of thread count.

## Not guaranteed

Identical results across different compilers, optimization levels, or CPU architectures. Floating-point behavior can differ between them. CI compares runs of one binary against itself, not across compilers.

## Rules that provide it

| Rule | Reason |
|---|---|
| Time is the integer tick count times a fixed step | No host clock, no accumulated error |
| Own PCG32 generator, one stream per subsystem | Standard distributions differ between libraries; separate streams keep subsystems independent |
| Messages delivered in (sim time, sender id) order | Thread scheduling cannot reorder them |
| No iteration over unordered containers in result paths | Iteration order is unspecified |
| Sorts use a total order | `std::sort` is not stable |
| `-ffp-contract=off` (`/fp:precise` on MSVC), no `-ffast-math` | Prevents fused multiply-add and reassociation changes |
| State hashed each tick with FNV-1a over fixed little-endian bytes | Exact comparison between runs |

## Enforcement

- `scripts/check_core_invariants.sh` fails the build on wall-clock calls, unseeded randomness, or fast-math flags in the core.
- Tests in `tests/determinism` check that equal seeds reproduce and unequal seeds or streams diverge.
- CI runs the runner twice with the same seed and diffs the output.
- A thread-count invariance test is added with the thread pool.

## Floating-point library calls

Not everything is bit-identical across platforms.

| Part | Uses | Guarantee |
|---|---|---|
| Terrain noise and scene content hash | Integer hashing and basic IEEE arithmetic | Identical on every compiler and platform; a golden hash in the tests enforces it |
| Obstacle placement | The PCG stream, `sqrt`, and basic arithmetic | Identical on every compiler and platform; also pinned by a golden hash |
| LiDAR beam directions, Gaussian noise (Box-Muller), weather attenuation | `sin`, `cos`, `log`, `exp`, `pow` | Same binary and platform. The last bit of these functions can differ between math libraries |

CI compares a run against itself on one platform. It does not compare results between platforms for the second group.
