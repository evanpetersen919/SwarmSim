# 0008. Real sockets under the simulation's channel model, delivered at a barrier

Date: 2026-10-19. Status: accepted.

## Context

The plan calls for the same drone code to run over real UDP and TCP sockets. The simulation's guarantees (the same seed gives the same state hash for any thread count, and a log can be verified by re-execution) depend on every message arriving at a tick decided by the model. A real network delivers when it likes, so putting sockets under the swarm in the obvious way (drones running in real time and receiving whatever has arrived) would give up determinism, replay, and the verifiable log.

## Decision

The socket channel keeps the simulation's model of what happens to each message and uses sockets only to carry the messages the model lets through.

- A `ChannelModel`, extracted from the deterministic channel and shared by both channels, decides whether the rate limit, the loss model, or a jammer drops a message, and the tick it is delivered at. Dropped messages never touch a socket.
- A message the model lets through is serialized with the wire format and sent at once: beacons as UDP datagrams, commands over TCP.
- The tick of delivery is still the model's. When the simulation reaches that tick, the channel puts the message in the receiver's inbox, waiting up to a timeout for the bytes to arrive if they have not.
- The order of delivery comes from the same `DeliveryQueue` class the deterministic channel uses, holding a placeholder for each message in flight, so the two channels order messages by one piece of code and not by two implementations of one rule.
- A message that does not arrive within the timeout is counted as a network drop and not delivered. That is the only way the real network can change a run.
- Wall-clock time and sockets live in `net/` and the apps. `core/` includes neither, and the invariants script fails otherwise.

## Consequences

- When the operating system loses and delays nothing, a run over sockets is bit-identical to the run over the deterministic channel: the same state hashes at every tick, the same message events, the same logs. Tests assert it, including against a hash recorded before the channel model was extracted. A log recorded over sockets verifies by re-execution on the simulated channel.
- If a datagram is lost by the operating system, the run diverges and its log fails verification. That is the correct signal and the behavior of a real network.
- What the real transport validates is the wire format, the framing, the ordering and reliability semantics, and the channel interface against a real stack. It does not validate the radio model: latency, loss, and jamming are the model's parameters, and loopback says nothing about a radio.
- Loopback is fast enough that the timeout is almost never used. A run over sockets costs the serialization and the system calls, which the comparison tool measures.
- The reliable class (commands) is modeled as a retransmitting transport with unlimited patience, so a command is never late because a link is bad.

## Alternatives considered

- A free-running real-time mode: how a real swarm behaves, but results depend on scheduling and load, nothing can be replayed, and a log cannot be verified. It is the real-time mode on the plan's cut list.
- Multi-process drones with epoll: a larger project, with the same determinism problem, and also on the cut list.
- A second implementation of the ordering rule in the socket channel: smaller to write, and a way for the two channels to drift apart.
- A network emulator such as `tc netem` or ns-3: on the cut list, platform specific, and not part of the same codebase.
