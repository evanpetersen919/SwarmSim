#include "swarm/format/replay.hpp"

#include <algorithm>

namespace swarm {

const char* to_string(ReplayStatus status) {
  switch (status) {
    case ReplayStatus::kOk:
      return "ok";
    case ReplayStatus::kBadLog:
      return "bad log";
    case ReplayStatus::kBadSnapshot:
      return "bad snapshot";
    case ReplayStatus::kTickOrder:
      return "record ticks are not strictly increasing";
    case ReplayStatus::kAgentCountMismatch:
      return "snapshot drone count differs from the header";
    case ReplayStatus::kNoState:
      return "no record holds drone state";
  }
  return "unknown";
}

ReplayStatus Replay::load(const std::uint8_t* data, std::size_t size) {
  header_ = LogHeader();
  trailer_ = LogTrailer();
  frames_.clear();
  state_frames_.clear();
  log_status_ = LogStatus::kOk;
  snapshot_status_ = SnapshotStatus::kOk;
  failed_record_ = 0;

  const auto fail = [this](ReplayStatus status) {
    header_ = LogHeader();
    trailer_ = LogTrailer();
    frames_.clear();
    state_frames_.clear();
    return status;
  };

  LogReader reader(data, size);
  log_status_ = reader.read_header();
  if (log_status_ != LogStatus::kOk) {
    return fail(ReplayStatus::kBadLog);
  }
  header_ = reader.header();

  for (;;) {
    LogRecord record;
    const LogStatus status = reader.next(record);
    if (status == LogStatus::kEndOfLog) {
      break;
    }
    if (status != LogStatus::kOk) {
      log_status_ = status;
      return fail(ReplayStatus::kBadLog);
    }
    const std::size_t index = frames_.size();
    if (index > 0 && record.tick <= frames_.back().tick) {
      failed_record_ = index;
      return fail(ReplayStatus::kTickOrder);
    }
    ReplayFrame frame;
    frame.tick = record.tick;
    frame.hash = record.hash;
    if (!record.blob.empty()) {
      snapshot_status_ = decode_snapshot(record.blob.data(), record.blob.size(), frame.snapshot);
      if (snapshot_status_ != SnapshotStatus::kOk) {
        failed_record_ = index;
        return fail(ReplayStatus::kBadSnapshot);
      }
    }
    if (frame.has_state()) {
      if (frame.snapshot.agents.size() != header_.agent_count) {
        failed_record_ = index;
        return fail(ReplayStatus::kAgentCountMismatch);
      }
      state_frames_.push_back(index);
    }
    frames_.push_back(std::move(frame));
  }
  trailer_ = reader.trailer();

  if (state_frames_.empty()) {
    return fail(ReplayStatus::kNoState);
  }
  return ReplayStatus::kOk;
}

const ReplayFrame* Replay::frame_at(std::uint64_t tick) const {
  const auto it = std::lower_bound(
      frames_.begin(), frames_.end(), tick,
      [](const ReplayFrame& frame, std::uint64_t value) { return frame.tick < value; });
  return it != frames_.end() && it->tick == tick ? &*it : nullptr;
}

const ReplayFrame* Replay::frame_at_or_before(double tick) const {
  if (frames_.empty()) {
    return nullptr;
  }
  if (!(tick > static_cast<double>(frames_.front().tick))) {
    return &frames_.front();
  }
  // The first frame whose tick is above `tick`, then one step back.
  const auto it = std::upper_bound(frames_.begin(), frames_.end(), tick,
                                   [](double value, const ReplayFrame& frame) {
                                     return value < static_cast<double>(frame.tick);
                                   });
  return &*(it - 1);
}

bool Replay::sample(double tick, std::vector<AgentSnapshot>& out) const {
  out.clear();
  if (state_frames_.empty()) {
    return false;
  }
  const ReplayFrame& first = frames_[state_frames_.front()];
  const ReplayFrame& last = frames_[state_frames_.back()];
  if (!(tick > static_cast<double>(first.tick))) {
    out = first.snapshot.agents;
    return true;
  }
  if (!(tick < static_cast<double>(last.tick))) {
    out = last.snapshot.agents;
    return true;
  }

  // The state frames on either side: `hi` is the first with a tick above `tick`.
  const auto hi = std::upper_bound(state_frames_.begin(), state_frames_.end(), tick,
                                   [this](double value, std::size_t index) {
                                     return value < static_cast<double>(frames_[index].tick);
                                   });
  const ReplayFrame& b = frames_[*hi];
  const ReplayFrame& a = frames_[*(hi - 1)];
  if (tick == static_cast<double>(a.tick)) {
    out = a.snapshot.agents;  // a logged tick is returned exactly, not through the curve
    return true;
  }

  const double span_ticks = static_cast<double>(b.tick - a.tick);
  const double s = (tick - static_cast<double>(a.tick)) / span_ticks;
  const double h = span_ticks * header_.dt;  // seconds between the two frames

  // Cubic Hermite basis and its derivative with respect to s.
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  const double d00 = 6.0 * s2 - 6.0 * s;
  const double d10 = 3.0 * s2 - 4.0 * s + 1.0;
  const double d01 = -6.0 * s2 + 6.0 * s;
  const double d11 = 3.0 * s2 - 2.0 * s;

  out.resize(a.snapshot.agents.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    const AgentSnapshot& pa = a.snapshot.agents[i];
    const AgentSnapshot& pb = b.snapshot.agents[i];
    AgentSnapshot& result = out[i];
    result.alive = pa.alive;
    result.fallback_state = pa.fallback_state;
    for (std::size_t k = 0; k < 3; ++k) {
      result.position[k] = h00 * pa.position[k] + h10 * h * pa.velocity[k] + h01 * pb.position[k] +
                           h11 * h * pb.velocity[k];
      result.velocity[k] = (d00 * pa.position[k] + d10 * h * pa.velocity[k] + d01 * pb.position[k] +
                            d11 * h * pb.velocity[k]) /
                           h;
    }
  }
  return true;
}

}  // namespace swarm
