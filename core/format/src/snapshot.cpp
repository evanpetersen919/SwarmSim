#include "swarm/format/snapshot.hpp"

#include <cmath>

#include "swarm/format/byte_io.hpp"

namespace swarm {

namespace {

constexpr std::uint8_t kAliveBit = 0x01;
constexpr unsigned kStateShift = 1;
constexpr std::uint8_t kStateMask = 0x03;
constexpr std::uint8_t kReservedMask = 0xF8;

bool finite3(const std::array<double, 3>& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool valid_kind(std::uint8_t kind) {
  return kind >= static_cast<std::uint8_t>(SnapshotEventKind::kScheduled) &&
         kind <= static_cast<std::uint8_t>(SnapshotEventKind::kDroppedJam);
}

}  // namespace

const char* to_string(SnapshotStatus status) {
  switch (status) {
    case SnapshotStatus::kOk:
      return "ok";
    case SnapshotStatus::kTruncated:
      return "truncated";
    case SnapshotStatus::kTrailingBytes:
      return "trailing bytes";
    case SnapshotStatus::kUnsupportedVersion:
      return "unsupported version";
    case SnapshotStatus::kInvalidValue:
      return "invalid value";
    case SnapshotStatus::kLimitExceeded:
      return "limit exceeded";
  }
  return "unknown";
}

SnapshotStatus encode_snapshot(const Snapshot& snapshot, std::vector<std::uint8_t>& out) {
  out.clear();
  if (snapshot.agents.size() > kMaxSnapshotAgents || snapshot.events.size() > kMaxSnapshotEvents) {
    return SnapshotStatus::kLimitExceeded;
  }
  for (const AgentSnapshot& a : snapshot.agents) {
    if (a.fallback_state > kStateMask || !finite3(a.position) || !finite3(a.velocity)) {
      return SnapshotStatus::kInvalidValue;
    }
  }
  for (const SnapshotEvent& e : snapshot.events) {
    if (!valid_kind(static_cast<std::uint8_t>(e.kind))) {
      return SnapshotStatus::kInvalidValue;
    }
  }

  out.reserve(1 + 4 + snapshot.agents.size() * kSnapshotAgentBytes + 4 +
              snapshot.events.size() * kSnapshotEventBytes);
  detail::put_u8(out, kSnapshotVersion);
  detail::put_u32(out, static_cast<std::uint32_t>(snapshot.agents.size()));
  for (const AgentSnapshot& a : snapshot.agents) {
    detail::put_u8(out, static_cast<std::uint8_t>((a.alive ? kAliveBit : 0U) |
                                                  (a.fallback_state << kStateShift)));
    for (double v : a.position) {
      detail::put_f64(out, v);
    }
    for (double v : a.velocity) {
      detail::put_f64(out, v);
    }
  }
  detail::put_u32(out, static_cast<std::uint32_t>(snapshot.events.size()));
  for (const SnapshotEvent& e : snapshot.events) {
    detail::put_u8(out, static_cast<std::uint8_t>(e.kind));
    detail::put_u32(out, e.sender);
    detail::put_u32(out, e.receiver);
    detail::put_u64(out, e.send_tick);
    detail::put_u64(out, e.deliver_tick);
  }
  return SnapshotStatus::kOk;
}

SnapshotStatus decode_snapshot(const std::uint8_t* data, std::size_t size, Snapshot& out) {
  out.agents.clear();
  out.events.clear();
  const auto fail = [&out](SnapshotStatus status) {
    out.agents.clear();
    out.events.clear();
    return status;
  };

  std::size_t pos = 0;
  const auto has = [&](std::size_t count) { return size - pos >= count; };

  if (!has(1)) {
    return fail(SnapshotStatus::kTruncated);
  }
  if (data[pos++] != kSnapshotVersion) {
    return fail(SnapshotStatus::kUnsupportedVersion);
  }

  if (!has(4)) {
    return fail(SnapshotStatus::kTruncated);
  }
  const std::uint32_t agent_count = detail::get_u32(data + pos);
  pos += 4;
  if (agent_count > kMaxSnapshotAgents) {
    return fail(SnapshotStatus::kLimitExceeded);
  }
  if (!has(static_cast<std::size_t>(agent_count) * kSnapshotAgentBytes)) {
    return fail(SnapshotStatus::kTruncated);
  }
  out.agents.reserve(agent_count);
  for (std::uint32_t i = 0; i < agent_count; ++i) {
    AgentSnapshot a;
    const std::uint8_t flags = data[pos];
    if ((flags & kReservedMask) != 0) {
      return fail(SnapshotStatus::kInvalidValue);
    }
    a.alive = (flags & kAliveBit) != 0;
    a.fallback_state = static_cast<std::uint8_t>((flags >> kStateShift) & kStateMask);
    for (std::size_t k = 0; k < 3; ++k) {
      a.position[k] = detail::get_f64(data + pos + 1 + 8 * k);
      a.velocity[k] = detail::get_f64(data + pos + 1 + 24 + 8 * k);
    }
    if (!finite3(a.position) || !finite3(a.velocity)) {
      return fail(SnapshotStatus::kInvalidValue);
    }
    out.agents.push_back(a);
    pos += kSnapshotAgentBytes;
  }

  if (!has(4)) {
    return fail(SnapshotStatus::kTruncated);
  }
  const std::uint32_t event_count = detail::get_u32(data + pos);
  pos += 4;
  if (event_count > kMaxSnapshotEvents) {
    return fail(SnapshotStatus::kLimitExceeded);
  }
  if (!has(static_cast<std::size_t>(event_count) * kSnapshotEventBytes)) {
    return fail(SnapshotStatus::kTruncated);
  }
  out.events.reserve(event_count);
  for (std::uint32_t i = 0; i < event_count; ++i) {
    SnapshotEvent e;
    if (!valid_kind(data[pos])) {
      return fail(SnapshotStatus::kInvalidValue);
    }
    e.kind = static_cast<SnapshotEventKind>(data[pos]);
    e.sender = detail::get_u32(data + pos + 1);
    e.receiver = detail::get_u32(data + pos + 5);
    e.send_tick = detail::get_u64(data + pos + 9);
    e.deliver_tick = detail::get_u64(data + pos + 17);
    out.events.push_back(e);
    pos += kSnapshotEventBytes;
  }

  if (pos != size) {
    return fail(SnapshotStatus::kTrailingBytes);
  }
  return SnapshotStatus::kOk;
}

}  // namespace swarm
