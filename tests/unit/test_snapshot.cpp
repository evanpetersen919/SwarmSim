#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "swarm/format/snapshot.hpp"
#include "swarm/rng.hpp"

using swarm::AgentSnapshot;
using swarm::Snapshot;
using swarm::SnapshotEvent;
using swarm::SnapshotEventKind;
using swarm::SnapshotStatus;

namespace {

using Bytes = std::vector<std::uint8_t>;

Snapshot sample() {
  Snapshot s;
  AgentSnapshot a;
  a.alive = true;
  a.fallback_state = 0;
  a.position = {1.0, 2.0, 3.0};
  a.velocity = {0.5, -1.0, 0.0};
  s.agents.push_back(a);
  AgentSnapshot b;
  b.alive = false;
  b.fallback_state = 3;
  b.position = {-2.0, 0.0, 4.0};
  b.velocity = {0.0, 0.0, 0.0};
  s.agents.push_back(b);
  s.events.push_back({SnapshotEventKind::kScheduled, 0, 1, 5, 7});
  s.events.push_back({SnapshotEventKind::kDroppedJam, 258, 3, 0x0102030405060708ULL, 0});
  return s;
}

// Hand-checked bytes of sample(). Changing the format means changing this on purpose and bumping
// kSnapshotVersion.
Bytes sample_bytes() {
  return {
      0x01,                                            // version
      0x02, 0x00, 0x00, 0x00,                          // agent_count
      0x01,                                            // agent 0: alive, state 0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F,  // x = 1.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,  // y = 2.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x40,  // z = 3.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x3F,  // vx = 0.5
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xBF,  // vy = -1.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // vz = 0.0
      0x06,                                            // agent 1: not alive, state 3
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0,  // x = -2.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // y = 0.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40,  // z = 4.0
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // vx
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // vy
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // vz
      0x02, 0x00, 0x00, 0x00,                          // event_count
      0x01,                                            // event 0: scheduled
      0x00, 0x00, 0x00, 0x00,                          // sender 0
      0x01, 0x00, 0x00, 0x00,                          // receiver 1
      0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // send_tick 5
      0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // deliver_tick 7
      0x04,                                            // event 1: dropped by a jammer
      0x02, 0x01, 0x00, 0x00,                          // sender 258
      0x03, 0x00, 0x00, 0x00,                          // receiver 3
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,  // send_tick
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // deliver_tick 0
  };
}

bool same(const AgentSnapshot& a, const AgentSnapshot& b) {
  return a.alive == b.alive && a.fallback_state == b.fallback_state && a.position == b.position &&
         a.velocity == b.velocity;
}

bool same(const SnapshotEvent& a, const SnapshotEvent& b) {
  return a.kind == b.kind && a.sender == b.sender && a.receiver == b.receiver &&
         a.send_tick == b.send_tick && a.deliver_tick == b.deliver_tick;
}

bool same(const Snapshot& a, const Snapshot& b) {
  if (a.agents.size() != b.agents.size() || a.events.size() != b.events.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.agents.size(); ++i) {
    if (!same(a.agents[i], b.agents[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.events.size(); ++i) {
    if (!same(a.events[i], b.events[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(Snapshot, EncodesToTheDocumentedBytes) {
  Bytes out;
  ASSERT_EQ(swarm::encode_snapshot(sample(), out), SnapshotStatus::kOk);
  EXPECT_EQ(out, sample_bytes());
  EXPECT_EQ(out.size(),
            1 + 4 + 2 * swarm::kSnapshotAgentBytes + 4 + 2 * swarm::kSnapshotEventBytes);
  EXPECT_EQ(out.size(), 157U);
}

TEST(Snapshot, DecodesTheDocumentedBytes) {
  const Bytes bytes = sample_bytes();
  Snapshot s;
  ASSERT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kOk);
  EXPECT_TRUE(same(s, sample()));
  EXPECT_FALSE(s.agents[1].alive);
  EXPECT_EQ(s.agents[1].fallback_state, 3);
  EXPECT_EQ(s.events[1].sender, 258U);
}

TEST(Snapshot, RoundTripsRandomSnapshotsIncludingEmptyOnes) {
  swarm::Rng rng(11, 0);
  for (int n = 0; n < 200; ++n) {
    Snapshot s;
    const std::uint32_t agents = n == 0 ? 0 : rng.below(20);
    const std::uint32_t events = n == 1 ? 0 : rng.below(40);
    for (std::uint32_t i = 0; i < agents; ++i) {
      AgentSnapshot a;
      a.alive = rng.bernoulli(0.8);
      a.fallback_state = static_cast<std::uint8_t>(rng.below(4));
      for (std::size_t k = 0; k < 3; ++k) {
        a.position[k] = rng.uniform(-1000.0, 1000.0);
        a.velocity[k] = rng.normal();
      }
      s.agents.push_back(a);
    }
    for (std::uint32_t i = 0; i < events; ++i) {
      SnapshotEvent e;
      e.kind = static_cast<SnapshotEventKind>(1 + rng.below(4));
      e.sender = rng.next_u32();
      e.receiver = rng.next_u32();
      e.send_tick = (static_cast<std::uint64_t>(rng.next_u32()) << 32) | rng.next_u32();
      e.deliver_tick = e.kind == SnapshotEventKind::kScheduled ? e.send_tick + rng.below(10) : 0;
      s.events.push_back(e);
    }
    Bytes bytes;
    ASSERT_EQ(swarm::encode_snapshot(s, bytes), SnapshotStatus::kOk);
    Snapshot back;
    ASSERT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), back), SnapshotStatus::kOk) << n;
    EXPECT_TRUE(same(s, back)) << n;
  }
}

TEST(Snapshot, EmptyAndSingleElementSnapshotsHaveTheExpectedSizes) {
  Bytes out;
  ASSERT_EQ(swarm::encode_snapshot(Snapshot{}, out), SnapshotStatus::kOk);
  EXPECT_EQ(out, (Bytes{0x01, 0, 0, 0, 0, 0, 0, 0, 0}));
  Snapshot s;
  Snapshot back;
  ASSERT_EQ(swarm::decode_snapshot(out.data(), out.size(), back), SnapshotStatus::kOk);
  EXPECT_TRUE(back.agents.empty() && back.events.empty());
}

TEST(Snapshot, EveryTruncationIsRejectedWithoutReadingPastTheEnd) {
  const Bytes full = sample_bytes();
  for (std::size_t length = 0; length < full.size(); ++length) {
    // A copy of exactly `length` bytes, so a read past the end is caught by the sanitizers.
    const Bytes cut(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(length));
    Snapshot s;
    s.agents.resize(3);
    const SnapshotStatus status = swarm::decode_snapshot(cut.data(), cut.size(), s);
    EXPECT_EQ(status, SnapshotStatus::kTruncated) << "length " << length;
    EXPECT_TRUE(s.agents.empty() && s.events.empty()) << "length " << length;
  }
}

TEST(Snapshot, TrailingBytesAreRejected) {
  Bytes bytes = sample_bytes();
  bytes.push_back(0);
  Snapshot s;
  EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kTrailingBytes);
}

TEST(Snapshot, AnUnsupportedVersionIsRejected) {
  for (int version : {0, 2, 255}) {
    Bytes bytes = sample_bytes();
    bytes[0] = static_cast<std::uint8_t>(version);
    Snapshot s;
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s),
              SnapshotStatus::kUnsupportedVersion)
        << version;
  }
}

TEST(Snapshot, InvalidValuesAreRejected) {
  Snapshot s;
  {
    Bytes bytes = sample_bytes();
    bytes[5] = 0x09;  // a reserved flag bit set on agent 0 (flags are at offset 5)
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kInvalidValue);
  }
  for (int kind : {0, 5, 200}) {
    Bytes bytes = sample_bytes();
    bytes[5 + 2 * swarm::kSnapshotAgentBytes + 4] =
        static_cast<std::uint8_t>(kind);  // the first event's kind
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kInvalidValue)
        << kind;
  }
  {
    Bytes bytes = sample_bytes();
    // Make agent 0's x a NaN: exponent all ones, nonzero mantissa.
    for (int i = 0; i < 6; ++i) {
      bytes[6 + static_cast<std::size_t>(i)] = 0x00;
    }
    bytes[6 + 6] = 0xF8;
    bytes[6 + 7] = 0x7F;
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kInvalidValue);
    // And infinity.
    bytes[6 + 6] = 0xF0;
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kInvalidValue);
  }
}

TEST(Snapshot, CorruptCountsAreRejectedBeforeAnythingIsAllocated) {
  Snapshot s;
  {
    // 4096 agents are allowed by the limit but the buffer holds none of them.
    Bytes bytes = {0x01, 0x00, 0x10, 0x00, 0x00};
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kTruncated);
  }
  {
    Bytes bytes = {0x01, 0xFF, 0xFF, 0xFF, 0xFF};
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s),
              SnapshotStatus::kLimitExceeded);
  }
  {
    // Zero agents, then an event count above the limit, then one just under it with no data.
    Bytes bytes = {0x01, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF};
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s),
              SnapshotStatus::kLimitExceeded);
    bytes = {0x01, 0, 0, 0, 0, 0x00, 0x00, 0x10, 0x00};  // 1 << 20 events: at the limit
    EXPECT_EQ(swarm::decode_snapshot(bytes.data(), bytes.size(), s), SnapshotStatus::kTruncated);
  }
}

TEST(Snapshot, EncodingRejectsWhatTheFormatCannotRepresent) {
  Bytes out = {1, 2, 3};
  {
    Snapshot s = sample();
    s.agents[0].fallback_state = 4;
    EXPECT_EQ(swarm::encode_snapshot(s, out), SnapshotStatus::kInvalidValue);
    EXPECT_TRUE(out.empty());
  }
  {
    Snapshot s = sample();
    s.agents[1].velocity[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(swarm::encode_snapshot(s, out), SnapshotStatus::kInvalidValue);
  }
  {
    Snapshot s = sample();
    s.agents[0].position[0] = std::numeric_limits<double>::infinity();
    EXPECT_EQ(swarm::encode_snapshot(s, out), SnapshotStatus::kInvalidValue);
  }
  {
    Snapshot s = sample();
    s.events[0].kind = static_cast<SnapshotEventKind>(9);
    EXPECT_EQ(swarm::encode_snapshot(s, out), SnapshotStatus::kInvalidValue);
  }
  {
    Snapshot s;
    s.agents.resize(swarm::kMaxSnapshotAgents + 1);
    EXPECT_EQ(swarm::encode_snapshot(s, out), SnapshotStatus::kLimitExceeded);
  }
  {
    Snapshot s;
    s.agents.resize(swarm::kMaxSnapshotAgents);  // exactly the limit is fine
    EXPECT_EQ(swarm::encode_snapshot(s, out), SnapshotStatus::kOk);
    Snapshot back;
    EXPECT_EQ(swarm::decode_snapshot(out.data(), out.size(), back), SnapshotStatus::kOk);
    EXPECT_EQ(back.agents.size(), swarm::kMaxSnapshotAgents);
  }
}

TEST(Snapshot, StatusNamesAreDistinct) {
  const SnapshotStatus all[] = {SnapshotStatus::kOk,
                                SnapshotStatus::kTruncated,
                                SnapshotStatus::kTrailingBytes,
                                SnapshotStatus::kUnsupportedVersion,
                                SnapshotStatus::kInvalidValue,
                                SnapshotStatus::kLimitExceeded};
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = i + 1; j < 6; ++j) {
      EXPECT_STRNE(swarm::to_string(all[i]), swarm::to_string(all[j]));
    }
  }
}
