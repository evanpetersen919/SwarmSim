# Phase 1: Deterministic runtime

Status: in progress. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md).

## Goal

A lockstep simulation loop in which agents exchange messages through a lossy, delayed channel, whose per-tick state hash does not depend on thread count, and whose runs can be recorded to a binary log and verified by re-execution.

No drone physics, sensors, or estimation are in this phase. A placeholder `DemoAgent` exercises the loop, the channel, and the hash so the machinery can be proven before domain code depends on it.

## Non-goals

Vehicles, sensors, scenes, jamming zones, Kalman filters, scenario files, and Unreal. Broadcast addressing (callers send one message per receiver). Network sockets (Phase 6).

## The tick

For tick `t`, on a fixed step `dt`:

1. Single thread: `channel.deliver(t)` moves every message with `deliver_tick <= t` into per-receiver inboxes, in key order.
2. Parallel, on the pool: for each agent `i`, `step(ctx)` where the context holds the agent's inbox (taken from the channel), its own RNG stream, and its own outbox. Agents touch only their own state, inbox, and outbox.
3. Barrier: `for_each` returns only when every agent has finished.
4. Single thread: for sender `0..N-1` in id order, for each message in that sender's outbox in order, `channel.send(t, msg)`. All channel randomness (loss, jitter) is drawn here, in this fixed order.
5. Single thread: fold the tick number and every agent's state hash (agent id order) into the running hash. Append a log record if recording. Advance the clock.

Why this is deterministic: the only parallel section (step 2) has no shared mutable state and no randomness outside per-agent streams. Everything order-dependent runs on one thread in a fixed order.

Minimum latency is one tick. A message sent in tick `t` is first visible in tick `t+1` at the earliest, so no agent can observe another agent's step within the same tick.

## Ordering key and tie-breaking

Messages are ordered by `(deliver_tick, sender, seq, receiver, insertion_index)`.

- `seq` is assigned by the channel, per sender, increasing.
- `receiver` is in the key so two messages from the same send call to different receivers have a total order.
- `insertion_index` is an internal counter that makes ordering total even for caller-supplied duplicate keys.

## RNG streams

All streams use the same seed and different stream ids, defined in one header (`swarm/streams.hpp`).

| Stream | Id | Used for |
|---|---|---|
| Channel loss | `0x100` | Loss and burst decisions |
| Channel jitter | `0x101` | Extra delay |
| Agent `i` | `0x1000 + i` | Per-agent randomness |

Keeping loss and jitter on separate streams means enabling jitter does not change which messages are lost. A test asserts this.

## Components and contracts

### Message and delivery queue (commit 1)

- `Message { deliver_tick, sender, receiver, seq, payload }` with a byte-vector payload.
- `DeliveryQueue::push(Message)`, `pop_ready(tick)` returning messages with `deliver_tick <= tick` in key order, `size()`, `empty()`.
- Backed by an ordered container keyed on the full key. No unordered containers.

### Channel interface and deterministic channel (commits 2 to 5)

```
class IChannel {
  void send(tick, Message);          // barrier only, single thread
  void deliver(tick);                // barrier only, single thread
  std::vector<Message> take_inbox(receiver);  // safe concurrently for distinct receivers
  const ChannelStats& stats() const;
};
```

`DeterministicChannel(num_endpoints, ChannelConfig, seed)`. Inboxes are sized once at construction and never resized, which makes `take_inbox` safe from different threads for different receivers.

`ChannelConfig`: `latency_ticks` (at least 1), loss model (none, independent, burst), `jitter_ticks`, `max_msgs_per_sender_per_tick` (0 means unlimited).

Loss models:

- Independent: each message dropped with probability `p`.
- Burst: a two-state chain per directed link (sender, receiver). The bad state drops everything, the good state drops nothing. With mean loss `p` and mean burst length `L >= 1`: `P(bad to good) = 1/L`, `P(good to bad) = p * (1/L) / (1 - p)`. The stationary bad-state probability is `p`. This is the simple two-state special case of the Gilbert-Elliott model; the plan's cut of full Gilbert-Elliott refers to per-state loss probabilities.

Order of draws in `send`: rate limit check (no draw), loss draw (loss stream), jitter draw (jitter stream). A dropped message does not consume a jitter draw, so jitter stream position depends only on which messages survive. Both properties are tested.

`ChannelStats`: sent, delivered, dropped by loss, dropped by rate limit.

### Lockstep pool (commit 6)

`LockstepPool(num_threads)` where `num_threads` includes the calling thread. `for_each(count, fn)` runs `fn(i)` for each `i` in `[0, count)`, exactly once, and returns after all complete. Indices are claimed from an atomic counter, so which thread runs which index is not fixed and must not matter.

- `num_threads == 1` runs inline with no worker threads.
- If tasks throw, the exception from the lowest index is rethrown after the barrier. The pool remains usable.
- C++17 has no `std::barrier`, so the pool uses a mutex, two condition variables, and a generation counter.

### Simulation and hash (commit 7)

- `IAgent::step(AgentContext&)` and `IAgent::hash_state(Fnv1a64&)`.
- `Simulation(SimConfig, agents)` owns the clock, the channel, the pool, and per-agent RNGs and outboxes.
- Running hash: `h_t = fold(h_{t-1}, t, agent state hashes in id order)`. The per-tick value is exposed so logs can record it.
- `DemoAgent`: placeholder ring-echo agent. Each tick it reads its inbox, folds payloads into an accumulator, draws a random value, and sends a message to the next agent. It is replaced by the drone agent in Phase 4.

### Binary log (commit 8, in `core/format`)

Little-endian, fixed-width, no struct memcpy.

```
header:  "SWRM" | u32 version | u64 seed | f64 dt | u32 agent_count | u32 config_len | config bytes
record:  u64 tick | u64 hash | u32 blob_len | blob bytes      (repeated)
trailer: "ENDS" | u64 record_count | u64 final_hash
```

- The blob is empty in this phase. It is reserved for state snapshots (positions) that the Unreal viewer reads in Phase 8, so the record layout will not change then.
- `config_len` and the config bytes carry an opaque text blob. The format layer does not interpret it; the runner stores its run configuration there so a log is self-describing.
- Readers reject: bad magic, unsupported version, truncation anywhere, trailer record count that does not match, and trailer final hash that does not match the last record.
- Version is 1. Any layout change bumps it, and the inline golden-bytes test must be updated deliberately.

### Replay and verify (commit 9)

- `SimConfig` has a text form (`key=value` lines) stored in the log header. Unknown keys and malformed values are errors.
- `record_run(config, ticks, threads, LogWriter&)` runs and writes the log.
- `verify_log(LogReader&, threads)` rebuilds the simulation from the header config, re-executes, and compares every recorded hash. It reports the first mismatching tick.
- Thread count is not part of the recorded config. A log recorded with 1 thread must verify with 8.
- Phase 1 "replay" is deterministic re-execution. Playback of recorded state for visualization arrives with the blobs in Phase 8.

## Commit-by-commit specification

Each commit builds warning-free, adds the tests listed, and passes `ctest`, the invariants script, and the format check before it is made.

### 1. `channel: add message and ordered delivery queue`

Files: `swarm/message.hpp`, `swarm/delivery_queue.hpp`, `src/delivery_queue.cpp`, `tests/unit/test_delivery_queue.cpp`.

Tests:
- Output sorted by deliver tick.
- Same tick: lower sender first.
- Same tick and sender: lower seq first.
- Same everything but different receiver: lower receiver first.
- Fully equal keys: insertion order preserved.
- Messages with `deliver_tick > tick` are not released and remain queued.
- Pushing the same set in shuffled orders gives identical output when keys are unique.
- `pop_ready` on an empty queue returns nothing.
- Payload bytes survive unchanged.

### 2. `channel: add IChannel and DeterministicChannel with fixed latency`

Files: `swarm/channel.hpp`, `swarm/streams.hpp`, `src/deterministic_channel.cpp`, `tests/unit/test_channel_latency.cpp`.

Tests:
- Message sent at tick `t` with latency `L` appears in the inbox after `deliver(t+L)` and not after `deliver(t+L-1)`.
- Latency 0 is rejected.
- `seq` increases per sender and is independent across senders.
- Inboxes are separate per receiver; `take_inbox` empties the inbox.
- Out-of-range sender or receiver is rejected.
- Stats count sent and delivered.

### 3. `channel: add independent random loss on a dedicated stream`

Tests:
- Loss 0 drops nothing; loss 1 drops everything.
- Observed loss rate within a stated tolerance of the target over 100,000 messages.
- Same seed gives the same drop pattern; different seed gives a different one.
- Probability outside `[0, 1]` is rejected.
- Dropped messages are counted and never delivered.

### 4. `channel: add burst loss model`

Tests:
- Observed mean loss close to the target.
- Observed mean burst length close to `L`.
- Burst state is per directed link: heavy loss on one link does not force losses on another.
- Parameters `p = 0`, `p` near 1, `L < 1`, `p >= 1` with burst: handled or rejected as specified.
- Same seed same pattern.

### 5. `channel: add jitter and per-sender rate limit`

Tests:
- With jitter `J`, delivery tick lies in `[t+L, t+L+J]` and every value in the range occurs over many messages.
- Jitter can reorder messages from one sender, and delivery order still follows the key.
- Enabling jitter does not change which messages are lost.
- Rate limit drops messages beyond the cap within a tick for that sender only; the counter resets next tick; other senders are unaffected.
- Cap 0 means unlimited.

### 6. `sim: add lockstep thread pool`

Tests:
- Every index runs exactly once per call, for several thread counts and sizes including count 0 and count smaller than the thread count.
- 10,000 consecutive calls complete without deadlock.
- Results written to slot `i` are identical for 1, 2, 4, and 8 threads.
- Exceptions: lowest-index exception is rethrown; the pool works afterwards.
- Construction with 0 threads is rejected.
- Destruction while idle joins cleanly.
- The ThreadSanitizer job runs all of the above.

### 7. `sim: add simulation loop, demo agent, and per-tick state hash`

Tests:
- Final and per-tick hashes identical for 1, 2, and 8 threads, with loss, burst, jitter, and rate limit enabled.
- Same seed same hashes; different seed different hashes.
- Hash differs at every tick.
- An agent that changes state changes the hash.
- Messages are delivered no earlier than latency allows (checked through the demo agent's received-tick record).
- Zero agents and one agent run without error.

### 8. `format: add binary log writer and reader`

Tests:
- Round trip of header and records, with and without blobs and config text.
- Exact bytes of a minimal log, written out by hand in the test.
- Bad magic, unsupported version, trailer count mismatch, trailer hash mismatch: each rejected with a distinct error.
- Truncation at every byte offset of a small log is rejected.
- A reader on an empty stream is rejected.
- Values are little-endian regardless of host.

### 9. `sim: add record, replay verification, and runner modes`

Tests:
- `SimConfig` text round trip; unknown key, duplicate key, and malformed number rejected.
- Record then verify succeeds.
- Record with 1 thread, verify with 8 threads succeeds.
- Corrupting one recorded hash makes verify fail and report that tick.
- A log with an altered config text fails verification.
- Truncated log fails with a read error, not a hash error.
- CI determinism job records and verifies through the command line with different thread counts.

## Risks

| Risk | Handling |
|---|---|
| Hidden parallel sharing in a step | Pool contract documented; ThreadSanitizer job; agents receive only their own context |
| Floating-point differences across thread counts | Parallel section does no cross-agent reductions; hash folding is single-threaded in fixed order |
| Statistical tests flaking | Fixed seeds, so results are deterministic; tolerances stated in the test and sized from the sample count |
| Log format regrets | Reserved blob field now; version number; byte-exact test |
| Pool deadlock or lost wakeup | Generation counter, condition-variable predicates, 10,000-call stress test |

## Exit criteria

- All nine commits made, each green locally and in CI.
- `swarm_sim --record run.log` followed by `swarm_sim --replay run.log --verify --threads 8` succeeds after recording with `--threads 1`.
- PLAN.md change log and the README status line updated.
