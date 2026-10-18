#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "hand_logs.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/metrics.hpp"

using swarm::MetricsStatus;
using swarm::Scenario;
using swarm::Snapshot;
using swarm::SnapshotEvent;
using swarm::SnapshotEventKind;
using namespace swarm::testing;

namespace {

SnapshotEvent scheduled(std::uint64_t send, std::uint64_t deliver) {
  return {SnapshotEventKind::kScheduled, 0, 1, send, deliver};
}

SnapshotEvent dropped(SnapshotEventKind kind, std::uint64_t send) { return {kind, 0, 1, send, 0}; }

// A log of `ticks` ticks (dt = 1 s) with two drones on their slots, in which each event sits in the
// record of its send tick.
swarm::MetricsOutcome with_events(std::uint64_t ticks, const std::vector<SnapshotEvent>& events,
                                  bool with_state = true) {
  const Scenario s = make_scenario(2, ticks);
  std::vector<Snapshot> snaps;
  for (std::uint64_t k = 0; k < ticks; ++k) {
    Snapshot snap = with_state ? on_slots(s) : Snapshot{};
    for (const SnapshotEvent& e : events) {
      if (e.send_tick == k) {
        snap.events.push_back(e);
      }
    }
    snaps.push_back(snap);
  }
  return analyze(make_log(s, snaps));
}

// Messages sent at tick 0 with the given latencies in ticks, in a log long enough to deliver them.
swarm::MetricsOutcome with_latencies(const std::vector<std::uint64_t>& latencies) {
  std::vector<SnapshotEvent> events;
  for (std::uint64_t l : latencies) {
    events.push_back(scheduled(0, l));
  }
  return with_events(latencies.empty() ? 5 : latencies.back() + 2, events);
}

}  // namespace

TEST(MetricsMessages, KnownLatenciesGiveKnownNearestRankPercentiles) {
  const auto ten = with_latencies({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
  ASSERT_TRUE(ten.ok()) << ten.message;
  EXPECT_DOUBLE_EQ(*ten.result.latency_p50_s, 5.0);   // rank ceil(0.5 * 10) = 5
  EXPECT_DOUBLE_EQ(*ten.result.latency_p95_s, 10.0);  // rank ceil(0.95 * 10) = 10

  std::vector<std::uint64_t> hundred;
  for (std::uint64_t i = 1; i <= 100; ++i) {
    hundred.push_back(i);
  }
  const auto h = with_latencies(hundred);
  EXPECT_DOUBLE_EQ(*h.result.latency_p50_s, 50.0);
  EXPECT_DOUBLE_EQ(*h.result.latency_p95_s, 95.0);

  std::vector<std::uint64_t> twenty;
  for (std::uint64_t i = 1; i <= 20; ++i) {
    twenty.push_back(i);
  }
  const auto t = with_latencies(twenty);
  EXPECT_DOUBLE_EQ(*t.result.latency_p50_s, 10.0);
  EXPECT_DOUBLE_EQ(*t.result.latency_p95_s, 19.0);  // 0.95 * 20 is exactly 19
}

TEST(MetricsMessages, OneAndTwoSamplesFollowTheNearestRankRule) {
  const auto one = with_latencies({3});
  EXPECT_DOUBLE_EQ(*one.result.latency_p50_s, 3.0);
  EXPECT_DOUBLE_EQ(*one.result.latency_p95_s, 3.0);

  const auto two = with_latencies({2, 6});
  EXPECT_DOUBLE_EQ(*two.result.latency_p50_s, 2.0);  // rank ceil(1.0) = 1: the lower value
  EXPECT_DOUBLE_EQ(*two.result.latency_p95_s, 6.0);  // rank ceil(1.9) = 2
}

TEST(MetricsMessages, TheOrderOfTheEventsDoesNotMatter) {
  const auto sorted = with_latencies({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
  std::vector<SnapshotEvent> shuffled;
  for (int l : {7, 2, 10, 4, 1, 9, 3, 8, 5, 6}) {
    shuffled.push_back(scheduled(0, static_cast<std::uint64_t>(l)));
  }
  const auto out = with_events(12, shuffled);
  EXPECT_EQ(*out.result.latency_p50_s, *sorted.result.latency_p50_s);
  EXPECT_EQ(*out.result.latency_p95_s, *sorted.result.latency_p95_s);
}

TEST(MetricsMessages, LatencyIsInSecondsWhateverTheTickLength) {
  // dt = 0.5 s: a delivery 4 ticks after the send is 2 seconds.
  Scenario s = make_scenario(2, 10);
  s.dt = 0.5;
  Snapshot first = on_slots(s);
  first.events.push_back(scheduled(0, 4));
  std::vector<Snapshot> snaps(10, on_slots(s));
  snaps[0] = first;
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok()) << out.message;
  EXPECT_DOUBLE_EQ(*out.result.latency_p50_s, 2.0);
}

TEST(MetricsMessages, MessagesStillInFlightAtTheEndAreLeftOut) {
  // A log of ticks 0 to 4. A message sent at tick 3 for delivery at tick 4 arrives in time; one for
  // tick 5 is still in flight.
  const auto out =
      with_events(5, {scheduled(3, 4), scheduled(3, 5), scheduled(4, 9), scheduled(0, 1)});
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.messages_sent, 4U);
  EXPECT_EQ(out.result.messages_delivered, 2U);
  EXPECT_EQ(out.result.messages_in_flight, 2U);
  EXPECT_DOUBLE_EQ(*out.result.delivery_ratio, 1.0);  // in-flight messages are not decided yet
  EXPECT_DOUBLE_EQ(*out.result.latency_p95_s, 1.0);   // latencies of the two delivered: 1 and 1
}

TEST(MetricsMessages, NoMessagesGivesNoValuesAtAll) {
  const auto out = with_events(5, {});
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.messages_sent, 0U);
  EXPECT_FALSE(out.result.delivery_ratio.has_value());
  EXPECT_FALSE(out.result.latency_p50_s.has_value());
  EXPECT_FALSE(out.result.latency_p95_s.has_value());
}

TEST(MetricsMessages, OnlyInFlightMessagesAlsoGiveNoRatioAndNoLatency) {
  const auto out = with_events(3, {scheduled(2, 8)});
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.messages_in_flight, 1U);
  EXPECT_FALSE(out.result.delivery_ratio.has_value());
  EXPECT_FALSE(out.result.latency_p50_s.has_value());
}

TEST(MetricsMessages, TheDeliveryRatioAndTheDropCountsMatchTheEvents) {
  std::vector<SnapshotEvent> events;
  for (std::uint64_t k = 0; k < 6; ++k) {
    events.push_back(scheduled(k, k + 1));
  }
  events.push_back(dropped(SnapshotEventKind::kDroppedLoss, 1));
  events.push_back(dropped(SnapshotEventKind::kDroppedLoss, 2));
  events.push_back(dropped(SnapshotEventKind::kDroppedRate, 3));
  events.push_back(dropped(SnapshotEventKind::kDroppedJam, 4));
  const auto out = with_events(10, events);
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.messages_sent, 10U);
  EXPECT_EQ(out.result.messages_delivered, 6U);
  EXPECT_EQ(out.result.dropped_loss, 2U);
  EXPECT_EQ(out.result.dropped_rate, 1U);
  EXPECT_EQ(out.result.dropped_jam, 1U);
  EXPECT_DOUBLE_EQ(*out.result.delivery_ratio, 0.6);
}

TEST(MetricsMessages, AllDroppedIsARatioOfZeroAndNoLatency) {
  const auto out = with_events(
      4, {dropped(SnapshotEventKind::kDroppedLoss, 0), dropped(SnapshotEventKind::kDroppedJam, 1)});
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(*out.result.delivery_ratio, 0.0);
  EXPECT_FALSE(out.result.latency_p50_s.has_value());
}

TEST(MetricsMessages, EventsCountEvenOnRecordsWithoutDroneState) {
  const auto with = with_events(6, {scheduled(1, 2), dropped(SnapshotEventKind::kDroppedLoss, 2)});
  const auto without =
      with_events(6, {scheduled(1, 2), dropped(SnapshotEventKind::kDroppedLoss, 2)}, false);
  ASSERT_TRUE(with.ok() && without.ok());
  EXPECT_EQ(without.result.messages_sent, 2U);
  EXPECT_EQ(*without.result.delivery_ratio, *with.result.delivery_ratio);
}

TEST(MetricsMessages, AMessageDeliveredBeforeItIsSentIsABadSnapshot) {
  const auto out = with_events(6, {scheduled(3, 2)});
  EXPECT_EQ(out.status, MetricsStatus::kBadSnapshot);
  EXPECT_NE(out.message.find("tick 3"), std::string::npos);
}
