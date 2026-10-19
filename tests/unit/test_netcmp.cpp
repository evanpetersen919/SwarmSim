#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "swarm/net/compare.hpp"

using swarm::net::CompareConfig;
using swarm::net::CompareResult;

namespace {

CompareConfig small() {
  CompareConfig c;
  c.endpoints = 4;
  c.ticks = 80;
  c.samples = 60;
  return c;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

}  // namespace

TEST(NetCompare, TheSimulatedAndRealChannelsDeliverTheSameMessages) {
  const CompareConfig c = small();
  const CompareResult r = swarm::net::run_comparison(c);
  EXPECT_TRUE(r.identical) << r.first_difference;
  EXPECT_TRUE(r.first_difference.empty());
  EXPECT_GT(r.sent, 0U);
  EXPECT_GT(r.dropped_by_model, 0U);
  EXPECT_EQ(r.dropped_by_network, 0U);
  EXPECT_EQ(r.delivered_real, r.delivered_simulated);
  EXPECT_GT(r.delivered_real, 0U);
}

TEST(NetCompare, TheCostMeasurementsAreOrderedAndPositive) {
  const CompareResult r = swarm::net::run_comparison(small());
  for (const auto& s : {r.udp, r.tcp}) {
    EXPECT_GT(s.samples, 0U);
    EXPECT_GT(s.p50_us, 0.0);
    EXPECT_LE(s.p50_us, s.p95_us);
    EXPECT_LE(s.p95_us, s.p99_us);
    EXPECT_LE(s.p99_us, s.max_us);
  }
  EXPECT_GT(r.udp_messages_per_second, 0.0);
  EXPECT_GT(r.tcp_messages_per_second, 0.0);
}

TEST(NetCompare, AMessageLostInTransitMakesTheChannelsDifferAndIsReported) {
  CompareConfig c = small();
  c.receive_timeout_ms = 30;
  c.lose_in_transit = [](const swarm::Message& m) { return m.seq % 7 == 3; };
  const CompareResult r = swarm::net::run_comparison(c);
  EXPECT_FALSE(r.identical);
  EXPECT_FALSE(r.first_difference.empty());
  EXPECT_GT(r.dropped_by_network, 0U);
  const std::string report = swarm::net::format_report(c, r);
  EXPECT_TRUE(contains(report, "identical deliveries: no"));
  EXPECT_TRUE(contains(report, "first difference:"));
}

TEST(NetCompare, TheReportHasTheDocumentedParts) {
  const CompareConfig c = small();
  const std::string report = swarm::net::format_report(c, swarm::net::run_comparison(c));
  for (const char* part :
       {"platform:", "traffic: 4 endpoints", "identical deliveries: yes", "UDP  p50", "TCP  p50",
        "messages per second", "simulated channel model latency: 2 to 3 ticks, 40.0 to 60.0 ms",
        "reading:"}) {
    EXPECT_TRUE(contains(report, part)) << part;
  }
}

TEST(NetCompare, ACommandFreeRunStillMatches) {
  CompareConfig c = small();
  c.commands = false;
  EXPECT_TRUE(swarm::net::run_comparison(c).identical);
}

TEST(NetCompare, InvalidConfigurationsAreRejected) {
  const auto rejects = [](auto edit) {
    CompareConfig c = small();
    edit(c);
    EXPECT_THROW(swarm::net::run_comparison(c), std::invalid_argument);
  };
  rejects([](CompareConfig& c) { c.endpoints = 1; });
  rejects([](CompareConfig& c) { c.endpoints = 100000; });
  rejects([](CompareConfig& c) { c.ticks = 0; });
  rejects([](CompareConfig& c) { c.payload_bytes = 5000; });
  rejects([](CompareConfig& c) { c.samples = 0; });
  rejects([](CompareConfig& c) { c.loss = -0.1; });
  rejects([](CompareConfig& c) { c.loss = 1.5; });
  rejects([](CompareConfig& c) { c.latency_ticks = 0; });
  rejects([](CompareConfig& c) { c.dt = 0.0; });
  rejects([](CompareConfig& c) { c.receive_timeout_ms = -1; });
}
