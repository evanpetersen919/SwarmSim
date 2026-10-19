# Phase 6: Real network backend

Status: planned. Parent documents: [PLAN.md](../PLAN.md), [PHASES.md](../PHASES.md), [DETERMINISM.md](../DETERMINISM.md), [MODELS.md](../MODELS.md), [phase 5](phase-05-scenarios-and-metrics.md).

## Goal

The same drone code runs over real UDP and TCP sockets. State beacons travel as UDP datagrams, commands from a ground station travel over TCP, both through the operating system's network stack on the loopback interface, and the swarm behaves exactly as it does on the simulated channel. A tool measures what the real transport costs and compares it with the simulated model.

## Non-goals

Multi-process drones, epoll or io_uring, real-time scheduling, `tc netem`, IPv6, encryption or authentication, congestion control, NAT traversal, multicast, a free-running real-time mode, and any claim about real radios. All of these are on the plan's cut list or beyond it. The project demonstrates sockets, serialization, and Linux and Windows portability. It does not demonstrate kernel-level or real-time work, and it says so.

## What exists and what is missing

The drone code talks to an `IChannel`. The only implementation is `DeterministicChannel`, an in-process queue with a seeded model of latency, loss, bursts, jitter, a rate limit, and jammers. Messages are opaque payload bytes. There is no serialization envelope, no operating-system networking, and nothing a ground station could send.

## The central problem: real sockets are not deterministic, and the simulation must stay deterministic

A real network delivers when it likes. The simulation's guarantees (same seed, same hash, any thread count, replay by re-execution) depend on every message arriving at a tick decided by the model. Two ways of putting sockets under the swarm were considered.

- **Free-running.** Drones run in real time and receive whatever has arrived. This is how a real swarm behaves, but results then depend on scheduling and load, nothing can be replayed, and the log cannot be verified. It is the real-time mode on the cut list.
- **Sockets under the model, delivered at a barrier.** The channel keeps the simulation's model for what happens to each message (dropped by loss, rate limit, or jamming; delivered after a latency and jitter in ticks) and uses real sockets to carry the messages that the model lets through. A message sent in a tick is put on a real socket immediately. When the simulation reaches the tick at which the model says the message is delivered, the channel waits, with a timeout, until the datagram has actually arrived, and then puts it in the receiver's inbox in the same order the simulated channel would. The network contributes its serialization, its transport, and its failure modes. The model contributes the timing.

This phase builds the second. Its consequence is a property that can be tested directly: when no datagram is lost by the operating system, a run over real sockets is bit-identical to the same run on the simulated channel, so its log verifies by re-execution on the simulated channel. It also says exactly what the real transport is for: it validates the wire format, the framing, the ordering and reliability semantics, and the channel interface against a real stack. It does not validate the radio model, and loopback latency says nothing about radio latency.

If the operating system does drop or delay a datagram past the timeout, the message is lost and counted as a network drop. That is real behavior, it makes the run diverge from the simulated one, and the verification of its log fails, which is the correct signal.

## Decisions

### Wall-clock time is confined to `net/` and the tools

The core never reads wall-clock time. Sockets need it for timeouts and a latency measurement needs it for stamps. Those uses live in `net/` and in the apps, never in `core/`, and `core/` never includes anything from `net/` or any socket header. The invariants script already forbids wall-clock use in `core/`; it gains a rule that `core/` may not include `swarm/net/` or a platform socket header, tested against fixtures with a deliberate violation like the controller boundary. The timeout can only turn a missing datagram into a counted drop. It cannot change which tick anything is delivered at.

### One wire format for both transports

A message is serialized to a fixed envelope followed by its payload (little-endian, using the byte helpers the other binary formats use):

```
u8  magic 'S' | u8 magic 'N' | u8 version = 1 | u8 type
u32 sender | u32 receiver | u64 seq | u64 send_tick | u16 payload_length | payload
```

Thirty bytes of header. Type 1 is a beacon (a neighbor report), type 2 a command. The payload is at most 1,100 bytes, which keeps a datagram under a typical path MTU. A UDP datagram holds one message. A TCP stream holds messages back to back, and the receiver parses the header and then `payload_length` bytes, so no second length prefix is needed. The decoder for a stream accepts bytes in chunks of any size and returns whole messages as they complete, because TCP does not preserve write boundaries. A malformed header (wrong magic, wrong version, unknown type, oversized payload) puts the stream decoder in a failed state it does not leave: on a stream there is no way to find the next message, so the connection is closed. Checksums are not added; UDP and TCP already have them.

`Message` gains a `type` field, defaulting to a beacon, so all existing senders are unchanged.

### Two classes of message: beacons are unreliable, commands are reliable

State beacons go over UDP: they are small, periodic, and a lost one is made up for by the next. Commands go over TCP: a lost hold command would leave a drone flying when told to stop, so it must arrive, once, in order. The simulated channel has to model the same difference or the two would not be comparable. The channel model therefore treats a command as a reliable message: the loss model, the rate limit, and the jammers do not drop it, it is delivered after the base latency with no jitter draw, and it consumes no random numbers, so adding commands to a run never changes the fate of any beacon. This is a stated simplification: it does not model TCP retransmission delays on a lossy or jammed link. It is the behavior of a command link with a margin to spare, and the docs say so.

### The channel model is extracted from the channel

`DeterministicChannel` mixes two jobs: deciding what happens to a message and holding it until its tick. The decision part (rate limit, loss, burst state, jamming, jitter) becomes a `ChannelModel` that both channels use, so the socket channel cannot drift from the simulated one. Extracting it is a refactor with a regression guard: a test fixes the state hash of a run under a harsh configuration, computed before the change, and the existing channel and simulation tests must pass untouched.

`Simulation` accepts a channel built by the caller (the default stays the deterministic one). The event sink moves onto `IChannel`, because the socket channel reports the same per-message events and the logs it produces must carry them.

### Commands: hold and resume

A command is an explicit reason to use the reliable path. A ground station is an agent with an endpoint of its own, placed after the drones, that sends scheduled commands to every drone. Two commands exist: hold (stop at the current position and stay there) and resume. A drone applies a command only if it is newer than the last one it applied (by the tick it was issued at), so a reordered pair cannot leave it in the wrong state. A commanded hold uses the same stopping behavior as the fallback machine's safe hold; it is not a fallback state and the machine is unaffected. Scenarios gain a `commands` list. The station is added only when the list is not empty, so the existing scenarios and their hashes do not change.

### Sockets: portable, blocking, loopback, polled

The wrapper is small and has one job. On both Windows (Winsock2) and Linux (POSIX) it provides RAII sockets, an IPv4 loopback bind to an ephemeral port chosen by the operating system (so tests running in parallel never collide), UDP send and receive, TCP listen, accept, connect, and exact-length send and receive, and a poll with a timeout (`poll` and `WSAPoll`). Sockets block; there is no epoll. Buffers are enlarged and `TCP_NODELAY` is set. The Windows quirk where a UDP socket reports a reset after an ICMP port-unreachable is turned off. Setup failures throw with the operating system's error code and text. A timeout is a status, not an exception.

### The socket channel

Each endpoint owns a UDP socket. TCP connections between a pair of endpoints are opened on the first command between them, so a swarm with one ground station opens one connection per drone and not a mesh. When a message is sent the model decides its fate. A dropped message is recorded as a drop and never touches a socket. A scheduled one is serialized and written to the right socket, and an entry goes into a ledger ordered the way the simulated channel's queue orders messages: by delivery tick, sender, sequence number, and receiver. At the delivery step the channel drains every socket without blocking, then for each ledger entry due at this tick waits, up to the timeout, for its message to be present, and hands the messages to the inboxes in ledger order. Datagrams that arrive early, late, or twice are matched by `(sender, seq)`; a duplicate is counted and ignored, a datagram that is not in the ledger or that fails to decode is counted as unexpected. Only the single-threaded barrier step touches sockets. Reading an inbox from several threads works as in the simulated channel.

The model's loss, burst, rate-limit, and jamming settings are the "loss shim": a test sends a large number of messages through the socket channel and checks that the observed drop rate matches the setting, and that the delivered messages are exactly the ones the simulated channel delivers.

### Running a scenario over sockets, and verifying it

`swarm_sim run SCENARIO --out FILE --transport sockets` runs the scenario with the socket channel and writes a log in the same format. The log does not record the transport, because the transport is not part of what defines the run, in the same way the thread count is not. `--replay FILE --verify` re-executes on the simulated channel. A log recorded over sockets that verifies is the end-to-end proof that the real transport reproduced the simulation. The command prints the network drop count, which is zero in a normal run.

### The comparison tool

`swarm_netcmp` pushes the same traffic through both channels: a number of endpoints each sending periodic beacons, and optionally commands. It reports whether the two deliver identical messages in identical order; the real transport's one-way latency per message in microseconds (the time from the call to `send` to the datagram being readable at the receiver, p50, p95, p99, and maximum, for UDP and TCP separately); the throughput in messages per second; and the simulated model's latency for comparison, in ticks and in milliseconds at the chosen `dt`. The latencies are machine dependent, so the tool prints them with the machine's name for the log and the tests never assert a latency. The honest reading of the result is stated in the output and the documentation: on loopback the real transport costs tens to hundreds of microseconds and the simulated radio latency is milliseconds, so the model parameter, not the transport, dominates, and the comparison validates the transport and its semantics and not the radio.

## Components and contracts

### Wire protocol (commit 47)

`encode_message`, `decode_message` for one datagram (status codes: ok, truncated, bad magic, bad version, unknown type, oversized, trailing bytes), and `FrameDecoder` for a stream (`feed(bytes) -> messages`, sticky failure). `MessageType` in `swarm/message.hpp`.

### Channel model (commit 48)

`ChannelModel::decide(now_tick, message) -> Decision { outcome, deliver_tick }` owning the seeded streams, the burst state, the rate counters, and the jammers. Reliable messages bypass the loss, rate, and jammer steps. `DeterministicChannel` becomes the model plus the queue. `IChannel::set_event_sink`. `Simulation(config, agents, threads, channel)`.

### Commands (commits 49 and 50)

`Command { kind, issue_tick }` with a codec in `swarm/control/command.hpp`; `InboundReport` gains the message type; the brain applies commands; `DroneAgent` routes by type; `GroundStationAgent`; the scenario `commands` list.

### Sockets (commit 51)

`net::UdpSocket`, `net::TcpListener`, `net::TcpStream`, `net::poll`, each move-only RAII.

### Transports (commits 52 and 53)

`UdpEndpoints`: one socket per endpoint, `send(from, to, bytes)`, `receive(timeout) -> datagrams`. `TcpConnections`: lazy connections per pair, `send`, `receive(timeout) -> messages` through a `FrameDecoder` per connection.

### Socket channel (commit 54)

`SocketChannel : IChannel` built from a `ChannelConfig`, the endpoint count, and a receive timeout; statistics add `dropped_network`, duplicates, and unexpected datagrams.

### Command line and tool (commits 55 and 56)

The `--transport` option and `swarm_netcmp`.

## Commit-by-commit specification

Each commit builds warning-free on MSVC, GCC, and Clang, adds the listed tests, and passes `ctest`, the invariants script, and the format check. Linux CI must be green before the phase is closed. `MODELS.md` gains a section in the commit that adds each part. Socket tests use loopback only and ephemeral ports.

### 47. `net: add the wire protocol`

Tests:
- A byte-exact layout test: a known message encodes to a fixed byte string and decodes back.
- Round trip over randomized messages, including an empty payload, the largest allowed payload, and extreme field values.
- Every truncation of a valid datagram is rejected with the truncated status.
- Bad magic, a wrong version, an unknown type, an oversized declared length, and trailing bytes after a datagram are each rejected with their own status.
- The stream decoder returns the same messages whether the bytes arrive whole, one at a time, or in random chunk sizes; two messages in one chunk both come out; a failure is sticky and later bytes are ignored.
- The wire library builds without any socket header.

### 48. `sim: extract the channel model and allow an injected channel`

Tests:
- A fixed state hash of a run with loss, bursts, jitter, a rate limit, and a jammer, recorded before the change, is reproduced; the existing channel, jamming, simulation, and metrics tests pass untouched.
- The model alone gives the same decisions as the channel for the same sequence of calls.
- A reliable message is never dropped by loss 1.0, by a jammer, or by a rate limit of zero extra capacity, is delivered after exactly the base latency, and arrives in send order for a pair.
- Interleaving reliable messages with beacons does not change any beacon's outcome (no random draws are consumed).
- A caller-supplied channel is used by the simulation, and its events reach the sink through the interface.

### 49. `control: add hold and resume commands`

Tests:
- The command codec round-trips and rejects malformed input.
- A brain told to hold decelerates to a stop and stays within a stated distance of where it stopped; told to resume it returns to its slot.
- A command older than the last applied one is ignored, in both orders of arrival; a malformed command is counted and ignored.
- The ground station agent sends each scheduled command to every drone at its tick, once.
- With commands interleaved with 50 percent beacon loss and a jam, every drone still receives every command.
- A swarm told to hold stops and, told to resume, re-forms; hashes are identical for every thread count.

### 50. `scenario: add the commands list`

Tests:
- A commands list loads with defaults, rejects an unknown kind, a negative or non-tick time, and unknown keys, and round-trips through canonical text.
- A scenario without commands builds exactly the same world as before (the station is absent, hashes unchanged).
- A scenario with commands builds the station as the last endpoint, and the snapshot, the verifier, and the metrics ignore it: the drone count in a log stays the number of drones.
- A run with a hold and a resume records, verifies, and gives metrics; the formation error rises during the hold and falls after the resume.

### 51. `net: add a portable socket wrapper`

Tests:
- Open, bind to an ephemeral loopback port, and report the port; two sockets get different ports.
- UDP send and receive between two sockets on loopback; an empty datagram; a datagram larger than the buffer is truncated, not lost silently (reported).
- TCP listen, connect, accept, and exact-length send and receive of a large buffer; receiving from a closed peer is reported as closed.
- A poll with no data times out and a poll with data returns at once; a timeout is a status and not an exception.
- Sockets are move-only and close on destruction (the port is free again).
- The invariants script fails on a fixture in which `core/` includes a socket header or `swarm/net/`, and passes on the real tree.

### 52. `net: add the UDP transport`

Tests:
- Two and then many endpoints exchange messages through the transport and each receives exactly what was sent to it, with the wire format intact.
- Several thousand messages sent in a burst to one receiver all arrive (buffers are large enough), and the count of any that do not is reported.
- A datagram that does not decode, and a foreign datagram sent to an endpoint's port, are counted as unexpected and do not stop the transport.
- Messages are never delivered to the wrong endpoint.

### 53. `net: add the TCP transport`

Tests:
- A command arrives, and ten thousand messages sent in order arrive in order, whole, with none lost.
- A sender that writes the stream in one-byte pieces and a receiver that reads in odd sizes still reassemble every message.
- Connections are opened on first use, once per pair, and reused.
- A message larger than the socket buffer is delivered (the sender does not deadlock against a receiver that reads concurrently).
- A closed connection and a corrupted stream are reported and do not take down other connections.

### 54. `net: add the socket channel`

Tests:
- A simulation of demo agents over the socket channel has the same state hash, tick by tick, as over the deterministic channel, for loss, bursts, jitter, a rate limit, and a jammer, and for 1, 2, 4, and 8 threads.
- The channel's events equal the deterministic channel's events for the same run.
- The observed drop rate matches the loss setting (independent and burst) over a large number of messages, and the delivered set equals the deterministic channel's delivered set.
- A message the transport fails to deliver within the timeout (provoked by dropping it between the socket and the ledger in a test hook) is counted as a network drop and the run continues.
- Commands go over TCP and beacons over UDP, checked by the transport statistics.
- Reading inboxes from several threads is safe (the ThreadSanitizer job runs this).

### 55. `scenario: run a scenario over sockets`

Tests:
- A scenario run over sockets writes a log that is byte-identical to the log of the same run over the simulated channel.
- Verifying that log by re-execution on the simulated channel passes, on a different thread count.
- A scenario with commands runs over sockets, with the commands going through TCP.
- The command prints a network drop count of zero, and exits with the usage status for an unknown transport.

### 56. `tools: add the real versus simulated comparison`

Tests:
- The tool reports identical deliveries for the two channels on a fixed traffic pattern, and exits with a failure status if they differ.
- The output has the documented fields for UDP and TCP; with the network drop count zero, delivered equals sent minus the model's drops.
- Latency percentiles are ordered (p50 not above p95, p95 not above p99, p99 not above the maximum) and positive; no absolute value is asserted.
- Bad arguments exit with the usage status.

## Documentation produced

- `docs/MODELS.md` sections: the wire format, the channel model and message classes, commands, the socket wrapper, the transports, the socket channel, and the comparison, each with parameters, assumptions, limitations, and validation status.
- ADR 0008 (real sockets under the simulation's channel model, delivered at a barrier) and a section in `DETERMINISM.md` on what the socket channel does and does not guarantee.
- `DEVELOPMENT.md`: loopback and firewall notes for Windows, and how to look at the traffic (`ss`, `tcpdump -i lo` on Linux).
- PLAN.md change log and the README status at the end of the phase.

## Risks

| Risk | Handling |
|---|---|
| Real sockets make results nondeterministic | The model decides every message's fate and tick; the sockets carry the bytes; delivery waits at a barrier. Bit-identity with the simulated channel is a test |
| Timing makes socket tests flaky | The only timing is a generous receive timeout that can turn a missing datagram into a counted drop; no test asserts a latency; ephemeral ports avoid collisions between parallel test runs |
| The operating system drops loopback datagrams under bursts | Large receive buffers; the burst test reports any loss; the channel counts network drops and a run with any fails verification, which is the intended signal |
| Windows and Linux behave differently | The wrapper hides the differences it knows (Winsock startup, `closesocket`, `WSAPoll`, the UDP reset quirk); tests run on both; CI covers Linux and local builds cover Windows |
| CI cannot open sockets | GitHub's runners allow loopback; if a runner did not, the tests would fail loudly and not skip |
| Commands change the existing results | The station exists only when a scenario has commands; reliable messages consume no random draws; a regression hash guards the channel refactor |
| The simulated reliable class is too kind | It is stated as a simplification and limits the claim: commands are not delayed by loss or jamming in the model |
| Sockets are mistaken for validating the radio model | The documentation and the tool output say that loopback validates the transport and its semantics only |
| Descriptor limits | One UDP socket per endpoint and one TCP connection per used pair; the default sizes are far below the limits, and the limit is documented |
| Wall-clock time leaks into the core | The invariants script forbids it in `core/` and forbids `core/` from including `net/` or socket headers, with fixtures |

## Exit criteria

- All ten commits made, each green locally and in CI (GCC, Clang, sanitizers, C++20).
- A scenario run over real UDP and TCP sockets writes a log byte-identical to the simulated run, and the log verifies on the simulated channel.
- A swarm obeys hold and resume commands carried over TCP, and loses none of them under heavy beacon loss and jamming.
- The comparison tool runs and states what it does and does not show.
- MODELS.md, the decision record, and DETERMINISM.md written; PLAN.md change log and README status updated.
