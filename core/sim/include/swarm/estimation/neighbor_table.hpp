#pragma once

// Estimator headers may include only this directory, sensor_data.hpp, swarm/format/byte_io.hpp, and
// vector types. They must never include vehicle truth. Enforced by
// scripts/check_core_invariants.sh.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "swarm/estimation/neighbor_message.hpp"

namespace swarm {

struct NeighborRecord {
  NeighborReport report;
  std::uint64_t receipt_tick = 0;  // the tick at which the report was delivered
};

enum class ReceiveResult {
  kAccepted,       // newer than anything held for this sender
  kStale,          // older than the report already held
  kDuplicate,      // same send tick as the report already held
  kMalformed,      // the payload did not decode
  kUnknownSender,  // sender id outside the table
};

struct NeighborTableCounters {
  std::uint64_t accepted = 0;
  std::uint64_t stale = 0;
  std::uint64_t duplicate = 0;
  std::uint64_t malformed = 0;
  std::uint64_t unknown_sender = 0;
};

// The latest accepted report from each neighbor and how old it is. Reordered and duplicate
// messages are ignored, so jitter in the channel cannot make a neighbor's state go backwards.
class NeighborTable {
 public:
  // Sender ids must be below `max_neighbors`.
  explicit NeighborTable(std::uint32_t max_neighbors);

  ReceiveResult on_message(std::uint32_t sender, const std::uint8_t* payload, std::size_t size,
                           std::uint64_t receipt_tick);

  // The latest accepted report from `sender`, or nullptr if there is none.
  const NeighborRecord* get(std::uint32_t sender) const;

  // Ticks since the latest accepted report from `sender` was delivered, or nothing if there is
  // none. Throws std::invalid_argument if `now_tick` is earlier than the receipt.
  std::optional<std::uint64_t> age(std::uint32_t sender, std::uint64_t now_tick) const;

  const NeighborTableCounters& counters() const { return counters_; }

 private:
  std::vector<std::optional<NeighborRecord>> records_;
  NeighborTableCounters counters_;
};

}  // namespace swarm
