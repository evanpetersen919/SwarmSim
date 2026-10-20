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

## Real sockets

The socket channel (`net/`) carries messages over real UDP and TCP sockets on the loopback interface. It does not give up the guarantees above, because the model decides and the sockets only carry.

- What happens to each message, and the tick it is delivered at, is decided by the same channel model as on the simulated channel, from the seed and the order of calls. The operating system decides nothing about it.
- A message the model lets through is put on a socket at once. At its delivery tick the channel waits for the bytes and delivers them in the order of the simulated channel's queue.
- So when the operating system loses and delays nothing, a run over sockets is bit-identical to the run on the deterministic channel: the same hash at every tick, the same message events, the same log, for any thread count. Tests assert this, including against numbers recorded before the model was extracted.
- The one way the network can change a run is a datagram that does not arrive within the receive timeout. The message is then counted as a network drop and not delivered, the run differs from the simulated one, and its log fails verification. That is the guarantee failing visibly, not silently.
- Wall-clock time is used only for that timeout, and only in `net/` and the apps. `core/` never includes anything from `net/` or any socket header, and `scripts/check_core_invariants.sh` fails the build if it does.

What is not guaranteed: that the operating system delivers every datagram. On loopback it almost always does; under heavy load it may not.

## Batch runs

`swarm_batch` runs many scenarios at once, on worker threads that take tasks from a shared counter. The results do not depend on that: each task runs its own simulation on one thread with no shared state, writes its row to its own slot, and the table is written in task order after all tasks finish. Which worker ran which task, and in what order the tasks finished, is not visible in the output. The progress callback and the log sink are the only code that runs concurrently with other tasks, and neither can change a row. Tests compare the table for 1, 2, 3, 6, and 40 jobs byte for byte, and the command-line script does the same through the tool.
