#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"

// A run log read into memory as frames, for the Unreal viewer and for tests. This header is
// compiled into the Unreal plugin, which builds without C++ exceptions, so errors are status codes
// and nothing here throws. It uses only the other headers of core/format.
//
// A frame is one log record: its tick, its state hash, and its decoded snapshot. A snapshot holds
// the drones' true state only on the ticks the scenario's snapshot stride selects and on the last
// tick; on the others `agents` is empty and only the message events are present.
//
// The state of a drone between two frames that hold drone state is a cubic Hermite curve through
// the two logged positions with the two logged velocities. At a logged tick the result is exactly
// the logged position and velocity. Whether a drone is alive, and its fallback state, are taken
// from the earlier of the two frames. Nothing is extrapolated: a time outside the logged range
// gives the first or last logged state.

namespace swarm {

enum class ReplayStatus {
  kOk,
  kBadLog,              // the log container is invalid; see Replay::log_status()
  kBadSnapshot,         // a record's blob does not decode; see Replay::snapshot_status()
  kTickOrder,           // record ticks are not strictly increasing
  kAgentCountMismatch,  // a snapshot's drone count differs from the header's
  kNoState,             // no record holds any drone state
};

const char* to_string(ReplayStatus status);

struct ReplayFrame {
  std::uint64_t tick = 0;
  std::uint64_t hash = 0;
  Snapshot snapshot;

  bool has_state() const { return !snapshot.agents.empty(); }
};

class Replay {
 public:
  // Parses the whole log. On an error the replay is empty. The buffer need not outlive the call.
  ReplayStatus load(const std::uint8_t* data, std::size_t size);

  // The status of the log container or of the snapshot decoder behind kBadLog or kBadSnapshot.
  LogStatus log_status() const { return log_status_; }
  SnapshotStatus snapshot_status() const { return snapshot_status_; }
  // The index of the record that failed, for kBadSnapshot, kTickOrder, and kAgentCountMismatch.
  std::size_t failed_record() const { return failed_record_; }

  const LogHeader& header() const { return header_; }
  const LogTrailer& trailer() const { return trailer_; }
  const std::vector<ReplayFrame>& frames() const { return frames_; }

  // Ticks of the first and last frame (zero when empty).
  std::uint64_t first_tick() const { return frames_.empty() ? 0 : frames_.front().tick; }
  std::uint64_t last_tick() const { return frames_.empty() ? 0 : frames_.back().tick; }

  // The number of frames that hold drone state.
  std::size_t state_frame_count() const { return state_frames_.size(); }

  // The frame for exactly this tick, or null.
  const ReplayFrame* frame_at(std::uint64_t tick) const;

  // The frame with the greatest tick at or below `tick` (the first frame if `tick` is before it),
  // or null for an empty replay. The viewer reads the hash to show from it.
  const ReplayFrame* frame_at_or_before(double tick) const;

  // The state of every drone at a fractional tick, by the rule above. Returns false (and leaves
  // `out` empty) for an empty replay. `velocity` in the result is the derivative of the curve, in
  // meters per second.
  bool sample(double tick, std::vector<AgentSnapshot>& out) const;

 private:
  LogHeader header_;
  LogTrailer trailer_;
  std::vector<ReplayFrame> frames_;
  std::vector<std::size_t> state_frames_;  // indices into frames_ of frames with drone state
  LogStatus log_status_ = LogStatus::kOk;
  SnapshotStatus snapshot_status_ = SnapshotStatus::kOk;
  std::size_t failed_record_ = 0;
};

}  // namespace swarm
