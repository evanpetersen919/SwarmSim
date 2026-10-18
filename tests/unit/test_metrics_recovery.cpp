#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <vector>

#include "hand_logs.hpp"
#include "swarm/scenario/metrics.hpp"

using swarm::DisturbanceKind;
using swarm::Recovery;
using swarm::Scenario;
using swarm::Snapshot;
using swarm::Vec3;
using namespace swarm::testing;

namespace {

// A log of `errors.size()` ticks in which every alive drone is `errors[k]` meters from its slot at
// tick k, so the formation RMSE at tick k is exactly errors[k]. dt is one second. The threshold is
// 1 m. `edit` may change the scenario (jammers, faults, hold time) before the log is built.
template <typename Edit>
swarm::MetricsOutcome trace(const std::vector<double>& errors, Edit edit) {
  Scenario s = make_scenario(2, errors.size());
  s.metrics.recovery_threshold_m = 1.0;
  s.metrics.recovery_hold_s = 2.0;
  edit(s);
  std::vector<Snapshot> snaps;
  for (double e : errors) {
    snaps.push_back(on_slots(s, std::vector<Vec3>(2, Vec3{e, 0, 0})));
  }
  return analyze(make_log(s, snaps));
}

// A disturbance at `tick` that is allowed to be tick 0: a drone fails then. The snapshots of these
// tests keep every drone alive; only the disturbance time matters to them.
void disturbance_at(Scenario& s, std::uint64_t tick) { s.faults.push_back({1, tick}); }

void jam_ends_at(Scenario& s, std::uint64_t tick) {
  swarm::Jammer j;
  j.radius = 50.0;
  j.start_tick = 0;
  j.end_tick = tick;
  s.channel.jammers.push_back(j);
}

}  // namespace

TEST(MetricsRecovery, AnErrorThatReturnsUnderTheThresholdAndStaysGivesTheKnownTime) {
  // The jam ends at tick 2. The error is back under 1 m from tick 3 and stays there.
  const auto out =
      trace({5, 5, 5, 0.5, 0.5, 0.5, 0.5, 0.5}, [](Scenario& s) { jam_ends_at(s, 2); });
  ASSERT_TRUE(out.ok()) << out.message;
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  const Recovery& r = out.result.recoveries[0];
  EXPECT_EQ(r.kind, DisturbanceKind::kJamEnd);
  EXPECT_EQ(r.tick, 2U);
  EXPECT_FALSE(r.censored);
  EXPECT_DOUBLE_EQ(r.seconds, 1.0);
  EXPECT_DOUBLE_EQ(*out.result.recovery_longest, 1.0);
  EXPECT_FALSE(out.result.any_recovery_censored);
}

TEST(MetricsRecovery, ADipUnderTheThresholdShorterThanTheHoldTimeDoesNotCount) {
  // Under the threshold at tick 1 for one tick only; the hold is two seconds. The real recovery
  // starts at tick 4.
  const auto out = trace({5, 0.5, 5, 5, 0.5, 0.5, 0.5}, [](Scenario& s) { disturbance_at(s, 0); });
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_DOUBLE_EQ(out.result.recoveries[0].seconds, 4.0);
}

TEST(MetricsRecovery, ATraceAlreadyUnderTheThresholdAtTheDisturbanceRecoversInZeroTime) {
  const auto out = trace({5, 5, 0.5, 0.5, 0.5, 0.5}, [](Scenario& s) { jam_ends_at(s, 2); });
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_EQ(out.result.recoveries[0].seconds, 0.0);
}

TEST(MetricsRecovery, ATraceThatNeverRecoversIsCensoredWithTheTimeRemaining) {
  const auto out = trace({5, 5, 5, 5, 5, 5, 5, 5, 5, 5}, [](Scenario& s) { jam_ends_at(s, 3); });
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  const Recovery& r = out.result.recoveries[0];
  EXPECT_TRUE(r.censored);
  EXPECT_DOUBLE_EQ(r.seconds, 7.0);  // from tick 3 to the end of a 10 tick log
  EXPECT_TRUE(out.result.any_recovery_censored);
  EXPECT_DOUBLE_EQ(*out.result.recovery_longest, 7.0);
}

TEST(MetricsRecovery, ALogThatEndsBeforeTheHoldTimeHasElapsedIsCensored) {
  // Back under the threshold at the last tick only: one tick of a two second hold.
  const auto out = trace({5, 5, 0.5}, [](Scenario& s) { disturbance_at(s, 0); });
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  EXPECT_TRUE(out.result.recoveries[0].censored);
}

TEST(MetricsRecovery, ErrorExactlyAtTheThresholdCountsAsWithin) {
  const auto out = trace({5, 1.0, 1.0, 1.0}, [](Scenario& s) { disturbance_at(s, 0); });
  ASSERT_TRUE(out.ok());
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_DOUBLE_EQ(out.result.recoveries[0].seconds, 1.0);
}

TEST(MetricsRecovery, AZeroHoldTimeNeedsASingleRecordUnderTheThreshold) {
  const auto out = trace({5, 5, 0.5, 5, 5}, [](Scenario& s) {
    disturbance_at(s, 0);
    s.metrics.recovery_hold_s = 0.0;
  });
  ASSERT_TRUE(out.ok());
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_DOUBLE_EQ(out.result.recoveries[0].seconds, 2.0);
}

TEST(MetricsRecovery, TheHoldTimeIsMeasuredInTicksOfTheScenario) {
  // A hold of 3 s at dt = 1 s needs three ticks under the threshold.
  const auto two = trace({5, 0.5, 0.5, 5, 5}, [](Scenario& s) {
    disturbance_at(s, 0);
    s.metrics.recovery_hold_s = 3.0;
  });
  EXPECT_TRUE(two.result.recoveries[0].censored);
  const auto three = trace({5, 0.5, 0.5, 0.5, 5}, [](Scenario& s) {
    disturbance_at(s, 0);
    s.metrics.recovery_hold_s = 3.0;
  });
  EXPECT_FALSE(three.result.recoveries[0].censored);
  EXPECT_DOUBLE_EQ(three.result.recoveries[0].seconds, 1.0);
}

TEST(MetricsRecovery, SeveralDisturbancesGiveSeveralResultsInTickOrder) {
  // A jam ends at tick 2 and a drone fails at tick 8; after the jam the formation recovers at
  // tick 3, and after the fault it recovers at tick 11.
  const auto out =
      trace({5, 5, 5, 0.5, 0.5, 0.5, 0.5, 0.5, 6, 6, 6, 0.5, 0.5, 0.5}, [](Scenario& s) {
        s.faults.push_back({1, 8});
        jam_ends_at(s, 2);
      });
  ASSERT_TRUE(out.ok());
  // After drone 1 fails it is excluded; drone 0 still has the 6 m error at ticks 8 to 10.
  ASSERT_EQ(out.result.recoveries.size(), 2U);
  EXPECT_EQ(out.result.recoveries[0].kind, DisturbanceKind::kJamEnd);
  EXPECT_EQ(out.result.recoveries[0].tick, 2U);
  EXPECT_DOUBLE_EQ(out.result.recoveries[0].seconds, 1.0);
  EXPECT_EQ(out.result.recoveries[1].kind, DisturbanceKind::kFault);
  EXPECT_EQ(out.result.recoveries[1].tick, 8U);
  EXPECT_DOUBLE_EQ(out.result.recoveries[1].seconds, 3.0);
  EXPECT_DOUBLE_EQ(*out.result.recovery_longest, 3.0);
  EXPECT_FALSE(out.result.any_recovery_censored);
}

TEST(MetricsRecovery, TheSummaryReportsTheLongestAndWhetherAnyWasCensored) {
  const auto out = trace({5, 0.5, 0.5, 5, 5, 5, 5, 5, 5, 5}, [](Scenario& s) {
    jam_ends_at(s, 1);           // already within the threshold at tick 1: recovered in 0 s
    s.faults.push_back({0, 3});  // never recovers: censored, 7 s remaining
  });
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 2U);
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_EQ(out.result.recoveries[0].seconds, 0.0);
  EXPECT_TRUE(out.result.recoveries[1].censored);
  EXPECT_DOUBLE_EQ(*out.result.recovery_longest, 7.0);
  EXPECT_TRUE(out.result.any_recovery_censored);
}

TEST(MetricsRecovery, AJammerWithNoEndGivesNoDisturbance) {
  const auto out = trace({5, 5, 5, 5}, [](Scenario& s) {
    swarm::Jammer j;
    j.radius = 10.0;  // end_tick keeps its default: it never ends
    s.channel.jammers.push_back(j);
  });
  ASSERT_TRUE(out.ok());
  EXPECT_TRUE(out.result.recoveries.empty());
  EXPECT_FALSE(out.result.recovery_longest.has_value());
  EXPECT_FALSE(out.result.any_recovery_censored);
}

TEST(MetricsRecovery, ADisturbanceAfterTheEndOfTheLogIsNotObserved) {
  const auto out = trace({0.5, 0.5, 0.5}, [](Scenario& s) {
    jam_ends_at(s, 50);
    s.faults.push_back({1, 3});  // the first tick after the last record
  });
  ASSERT_TRUE(out.ok());
  EXPECT_TRUE(out.result.recoveries.empty());
}

TEST(MetricsRecovery, ARunWithoutDisturbancesHasNoRecoveryTime) {
  const auto out = trace({0.5, 0.5, 0.5, 0.5}, [](Scenario&) {});
  ASSERT_TRUE(out.ok());
  EXPECT_TRUE(out.result.recoveries.empty());
  EXPECT_FALSE(out.result.recovery_longest.has_value());
}

TEST(MetricsRecovery, AFailedDronesErrorIsNotPartOfTheTrace) {
  // Drone 1 fails at tick 2 and sits 100 m from its slot; drone 0 is on its slot. The survivors'
  // error is zero, so the formation has recovered at once.
  Scenario s = make_scenario(2, 6);
  s.metrics.recovery_threshold_m = 1.0;
  s.metrics.recovery_hold_s = 2.0;
  s.faults.push_back({1, 2});
  std::vector<Snapshot> snaps;
  for (int k = 0; k < 6; ++k) {
    Snapshot snap = on_slots(s);
    if (k >= 2) {
      snap.agents[1] = at(kCenter + slots(s)[1] + Vec3{100, 0, 0}, false);
    }
    snaps.push_back(snap);
  }
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_EQ(out.result.recoveries[0].seconds, 0.0);
}

TEST(MetricsRecovery, RecordsWithoutDroneStateAreSkipped) {
  // Ticks 1 and 2 carry no state. The error is high at tick 0 and low at tick 3 onward.
  Scenario s = make_scenario(2, 6);
  s.metrics.recovery_threshold_m = 1.0;
  s.metrics.recovery_hold_s = 2.0;
  disturbance_at(s, 0);
  std::vector<Snapshot> snaps = {on_slots(s, {Vec3{5, 0, 0}, Vec3{5, 0, 0}}),
                                 Snapshot{},
                                 Snapshot{},
                                 on_slots(s),
                                 on_slots(s),
                                 on_slots(s)};
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  ASSERT_EQ(out.result.recoveries.size(), 1U);
  EXPECT_FALSE(out.result.recoveries[0].censored);
  EXPECT_DOUBLE_EQ(out.result.recoveries[0].seconds, 3.0);
}
