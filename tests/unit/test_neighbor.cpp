#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/estimation/neighbor_message.hpp"
#include "swarm/estimation/neighbor_table.hpp"

using swarm::DecodeStatus;
using swarm::NeighborReport;
using swarm::NeighborTable;
using swarm::ReceiveResult;
using swarm::Vec3;

namespace {

using Bytes = std::vector<std::uint8_t>;

NeighborReport sample_report(std::uint64_t send_tick = 3) {
  NeighborReport r;
  r.send_tick = send_tick;
  r.position = {1.0, 2.0, -1.0};
  r.velocity = {0.5, 0.0, -0.25};
  r.position_sigma = 0.25;
  return r;
}

void f64_bits(Bytes& b, std::uint64_t bits) {
  for (int i = 0; i < 8; ++i) {
    b.push_back(static_cast<std::uint8_t>((bits >> (8 * i)) & 0xFFU));
  }
}

// Hand-written bytes for sample_report(3). A change to the layout means changing this on purpose
// and bumping kNeighborReportVersion.
Bytes expected_bytes() {
  Bytes b = {1};                    // version
  f64_bits(b, 3);                   // send_tick, as a little-endian u64
  f64_bits(b, 0x3FF0000000000000);  // position.x 1.0
  f64_bits(b, 0x4000000000000000);  // position.y 2.0
  f64_bits(b, 0xBFF0000000000000);  // position.z -1.0
  f64_bits(b, 0x3FE0000000000000);  // velocity.x 0.5
  f64_bits(b, 0x0000000000000000);  // velocity.y 0.0
  f64_bits(b, 0xBFD0000000000000);  // velocity.z -0.25
  f64_bits(b, 0x3FD0000000000000);  // position_sigma 0.25
  return b;
}

ReceiveResult deliver(NeighborTable& table, std::uint32_t sender, const NeighborReport& r,
                      std::uint64_t receipt_tick) {
  const Bytes payload = swarm::encode_neighbor_report(r);
  return table.on_message(sender, payload.data(), payload.size(), receipt_tick);
}

}  // namespace

TEST(NeighborMessage, EncodesToTheDocumentedBytes) {
  EXPECT_EQ(swarm::encode_neighbor_report(sample_report()), expected_bytes());
  EXPECT_EQ(expected_bytes().size(), swarm::kNeighborReportBytes);
}

TEST(NeighborMessage, RoundTripsEveryFieldExactly) {
  NeighborReport r;
  r.send_tick = 0x8000000000000001ULL;  // beyond the range of a signed 64-bit integer
  r.position = {-1234.5678901234567, 9.87654321e-9, 4e12};
  r.velocity = {1e-300, -1e300, 0.1};
  r.position_sigma = 3.141592653589793;
  NeighborReport out;
  const Bytes bytes = swarm::encode_neighbor_report(r);
  ASSERT_EQ(swarm::decode_neighbor_report(bytes.data(), bytes.size(), out), DecodeStatus::kOk);
  EXPECT_EQ(out.send_tick, r.send_tick);
  EXPECT_EQ(out.position, r.position);
  EXPECT_EQ(out.velocity, r.velocity);
  EXPECT_EQ(out.position_sigma, r.position_sigma);
}

TEST(NeighborMessage, WrongSizesAreRejected) {
  const Bytes good = expected_bytes();
  NeighborReport out;
  EXPECT_EQ(swarm::decode_neighbor_report(good.data(), 0, out), DecodeStatus::kBadSize);
  for (std::size_t length = 1; length < good.size(); ++length) {
    EXPECT_EQ(swarm::decode_neighbor_report(good.data(), length, out), DecodeStatus::kBadSize)
        << "length " << length;
  }
  Bytes longer = good;
  longer.push_back(0);
  EXPECT_EQ(swarm::decode_neighbor_report(longer.data(), longer.size(), out),
            DecodeStatus::kBadSize);
}

TEST(NeighborMessage, WrongVersionsAreRejected) {
  NeighborReport out;
  const std::uint8_t versions[] = {0, 2, 255};
  for (std::uint8_t version : versions) {
    Bytes b = expected_bytes();
    b[0] = version;
    EXPECT_EQ(swarm::decode_neighbor_report(b.data(), b.size(), out), DecodeStatus::kBadVersion);
  }
}

TEST(NeighborMessage, NonFiniteValuesAndNegativeSigmaAreRejectedOnDecode) {
  NeighborReport out;
  const std::uint8_t nan_bytes[8] = {0, 0, 0, 0, 0, 0, 0xF8, 0x7F};
  const std::uint8_t inf_bytes[8] = {0, 0, 0, 0, 0, 0, 0xF0, 0x7F};
  // Each of the seven double fields starts at offset 9 + 8 * k.
  for (std::size_t field = 0; field < 7; ++field) {
    for (const std::uint8_t* poison : {nan_bytes, inf_bytes}) {
      Bytes b = expected_bytes();
      for (std::size_t i = 0; i < 8; ++i) {
        b[9 + 8 * field + i] = poison[i];
      }
      EXPECT_EQ(swarm::decode_neighbor_report(b.data(), b.size(), out), DecodeStatus::kInvalidValue)
          << "field " << field;
    }
  }
  Bytes negative_sigma = expected_bytes();
  negative_sigma[9 + 8 * 6 + 7] = 0xBF;  // sign and exponent of -0.25
  EXPECT_EQ(swarm::decode_neighbor_report(negative_sigma.data(), negative_sigma.size(), out),
            DecodeStatus::kInvalidValue);
}

TEST(NeighborMessage, EncodingInvalidValuesThrows) {
  NeighborReport r = sample_report();
  r.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(swarm::encode_neighbor_report(r), std::invalid_argument);
  r = sample_report();
  r.velocity.z = std::numeric_limits<double>::infinity();
  EXPECT_THROW(swarm::encode_neighbor_report(r), std::invalid_argument);
  r = sample_report();
  r.position_sigma = -1.0;
  EXPECT_THROW(swarm::encode_neighbor_report(r), std::invalid_argument);
}

TEST(NeighborTable, AgeGrowsEveryTickWithoutAMessageAndResetsWhenOneArrives) {
  NeighborTable table(4);
  EXPECT_FALSE(table.age(2, 10).has_value());
  EXPECT_EQ(table.get(2), nullptr);

  ASSERT_EQ(deliver(table, 2, sample_report(9), 10), ReceiveResult::kAccepted);
  EXPECT_EQ(*table.age(2, 10), 0U);
  for (std::uint64_t now = 11; now <= 20; ++now) {
    EXPECT_EQ(*table.age(2, now), now - 10);
  }
  ASSERT_EQ(deliver(table, 2, sample_report(19), 21), ReceiveResult::kAccepted);
  EXPECT_EQ(*table.age(2, 21), 0U);
  EXPECT_EQ(*table.age(2, 25), 4U);
}

TEST(NeighborTable, ReorderedAndDuplicateReportsAreIgnored) {
  NeighborTable table(2);
  ASSERT_EQ(deliver(table, 0, sample_report(5), 10), ReceiveResult::kAccepted);
  NeighborReport newer = sample_report(5);
  newer.position = {50.0, 0.0, 0.0};

  EXPECT_EQ(deliver(table, 0, sample_report(3), 11), ReceiveResult::kStale);  // older send tick
  EXPECT_EQ(deliver(table, 0, newer, 12), ReceiveResult::kDuplicate);         // same send tick
  EXPECT_EQ(table.get(0)->report.position, sample_report(5).position);        // unchanged
  EXPECT_EQ(table.get(0)->receipt_tick, 10U);                                 // age not reset

  NeighborReport later = sample_report(6);
  later.position = {7.0, 7.0, 7.0};
  EXPECT_EQ(deliver(table, 0, later, 13), ReceiveResult::kAccepted);
  EXPECT_EQ(table.get(0)->report.position, later.position);
  EXPECT_EQ(table.get(0)->receipt_tick, 13U);

  EXPECT_EQ(table.counters().accepted, 2U);
  EXPECT_EQ(table.counters().stale, 1U);
  EXPECT_EQ(table.counters().duplicate, 1U);
}

TEST(NeighborTable, NeighborsAreTrackedIndependently) {
  NeighborTable table(3);
  NeighborReport a = sample_report(4);
  a.position = {1.0, 1.0, 1.0};
  NeighborReport b = sample_report(8);
  b.position = {2.0, 2.0, 2.0};
  ASSERT_EQ(deliver(table, 0, a, 5), ReceiveResult::kAccepted);
  ASSERT_EQ(deliver(table, 1, b, 9), ReceiveResult::kAccepted);
  EXPECT_EQ(table.get(0)->report.position, a.position);
  EXPECT_EQ(table.get(1)->report.position, b.position);
  EXPECT_EQ(*table.age(0, 10), 5U);
  EXPECT_EQ(*table.age(1, 10), 1U);
  EXPECT_FALSE(table.age(2, 10).has_value());  // never heard from
}

TEST(NeighborTable, MalformedPayloadsAndUnknownSendersAreCountedAndIgnored) {
  NeighborTable table(2);
  const Bytes junk = {1, 2, 3};
  EXPECT_EQ(table.on_message(0, junk.data(), junk.size(), 1), ReceiveResult::kMalformed);
  const Bytes good = swarm::encode_neighbor_report(sample_report());
  EXPECT_EQ(table.on_message(2, good.data(), good.size(), 1), ReceiveResult::kUnknownSender);
  EXPECT_EQ(table.on_message(99, good.data(), good.size(), 1), ReceiveResult::kUnknownSender);
  EXPECT_EQ(table.get(0), nullptr);
  EXPECT_EQ(table.counters().malformed, 1U);
  EXPECT_EQ(table.counters().unknown_sender, 2U);
  EXPECT_EQ(table.counters().accepted, 0U);
}

TEST(NeighborTable, AskingForTheAgeBeforeTheReceiptIsAnError) {
  NeighborTable table(1);
  ASSERT_EQ(deliver(table, 0, sample_report(), 10), ReceiveResult::kAccepted);
  EXPECT_THROW(table.age(0, 9), std::invalid_argument);
  EXPECT_NO_THROW(table.age(0, 10));
}
