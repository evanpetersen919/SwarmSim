# 0006. Run logs carry snapshots and message events; metrics read only logs

Date: 2026-10-18. Status: accepted.

## Context

The plan says metrics are computed from logs, not inside the simulation. The log written so far holds a tick and a state hash per record, with an empty blob reserved for state snapshots. A hash cannot give formation error, separation, or message latency. Something has to put the evaluation data in the log, and the Unreal viewer in Phase 8 needs the same data to draw a run.

## Decision

Each log record's blob holds a snapshot: the true position, velocity, alive flag, and fallback state of every drone at that tick, and the outcomes of the messages sent in it (scheduled with a delivery tick, or dropped by loss, rate limit, or jamming). The encoding is defined in `core/format/include/swarm/format/snapshot.hpp` with its own version byte and a byte-exact layout test.

- The log container is unchanged: same layout, same version, same layout test. The blob was already an opaque field. The snapshot is a layer on top, versioned separately.
- The codec lives in `core/format` because the viewer reads it. It follows that library's rules: no exceptions, status codes, no Eigen or yaml-cpp.
- The snapshot holds the true state. That is allowed because it is the evaluation record; the brain never sees it. Only the runner writes snapshots and only the metrics and the viewer read them.
- Decoding checks the version, the reserved bits, the event kinds, that numbers are finite, and the counts against the bytes that remain before allocating anything.
- A scenario log stores the scenario text in the header, so a log is self-describing and a metrics function needs the log bytes and nothing else.
- Verification by re-execution compares the blob bytes of every record as well as the hash, so a divergence in anything the metrics read is caught, not only a divergence in what the hash covers.

## Alternatives considered

- Compute metrics inside the simulation: simpler to write, but it breaks the plan's rule, makes every metric change a change to the simulation, and leaves the viewer without data.
- A new log container version with typed record kinds: cleaner on paper, but a larger change to a format that already has tests and replay, for no gain the blob does not give.
- Log only drone positions and recompute messages by replay: slower, and message outcomes depend on the channel state, which is the thing being measured.

## Consequences

- Logs get larger: about 49 bytes per drone and 25 per message event per tick. For eight drones at 50 Hz for 60 s this is a little over a megabyte plus events. Batch runs compute metrics from the in-memory log and discard it.
- The snapshot version must be bumped, with the layout test changed on purpose, whenever its layout changes.
- Metrics are reproducible from a log without running anything, which is also what makes them testable on hand-built logs.
