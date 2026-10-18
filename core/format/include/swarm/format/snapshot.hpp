#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// Per-tick run snapshot: the true state of every drone and the outcomes of the messages sent that
// tick. It is the content of a log record's blob (see log.hpp), written by the scenario runner and
// read by the metrics and by the Unreal viewer. This header is compiled into the Unreal plugin,
// which builds without C++ exceptions, so errors are status codes and nothing here throws.
//
// Layout (little-endian, fixed width):
//   u8  version = 1
//   u32 agent_count
//   agent_count x { u8 flags | f64 position[3] | f64 velocity[3] }        49 bytes each
//   u32 event_count
//   event_count x { u8 kind | u32 sender | u32 receiver | u64 send_tick | u64 deliver_tick }
//                                                                          25 bytes each
// flags: bit 0 alive; bits 1 and 2 the fallback state (0 nominal, 1 degraded comms, 2 degraded
// navigation, 3 safe hold); bits 3 to 7 must be zero. deliver_tick is zero for a dropped message.
namespace swarm {

inline constexpr std::uint8_t kSnapshotVersion = 1;
inline constexpr std::size_t kSnapshotAgentBytes = 1 + 6 * 8;
inline constexpr std::size_t kSnapshotEventBytes = 1 + 4 + 4 + 8 + 8;
inline constexpr std::uint32_t kMaxSnapshotAgents = 4096;
inline constexpr std::uint32_t kMaxSnapshotEvents = 1U << 20;

// The same values as the channel's MessageOutcome, so a writer can convert by cast.
enum class SnapshotEventKind : std::uint8_t {
  kScheduled = 1,    // will be delivered at deliver_tick
  kDroppedLoss = 2,  // dropped by the loss model
  kDroppedRate = 3,  // dropped by the per-sender rate limit
  kDroppedJam = 4,   // dropped by a jammer
};

struct AgentSnapshot {
  bool alive = true;
  std::uint8_t fallback_state = 0;  // 0 to 3
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
};

struct SnapshotEvent {
  SnapshotEventKind kind = SnapshotEventKind::kScheduled;
  std::uint32_t sender = 0;
  std::uint32_t receiver = 0;
  std::uint64_t send_tick = 0;
  std::uint64_t deliver_tick = 0;
};

struct Snapshot {
  std::vector<AgentSnapshot> agents;
  std::vector<SnapshotEvent> events;
};

enum class SnapshotStatus {
  kOk,
  kTruncated,           // the buffer ends before the data does
  kTrailingBytes,       // the buffer holds more than the snapshot
  kUnsupportedVersion,  // the version byte is not kSnapshotVersion
  kInvalidValue,        // reserved flag bits, an unknown event kind, or a non-finite number
  kLimitExceeded,       // more agents or events than the format allows
};

const char* to_string(SnapshotStatus status);

// Replaces the contents of `out` with the encoded snapshot. On an error `out` is empty. Fails with
// kInvalidValue for a fallback state above 3, an event kind that is not one of the four, or a
// non-finite position or velocity, and with kLimitExceeded for too many agents or events.
SnapshotStatus encode_snapshot(const Snapshot& snapshot, std::vector<std::uint8_t>& out);

// Decodes exactly `size` bytes. The counts are checked against the remaining bytes before anything
// is allocated, so a corrupt count cannot cause a large allocation. On an error `out` is empty.
SnapshotStatus decode_snapshot(const std::uint8_t* data, std::size_t size, Snapshot& out);

}  // namespace swarm
