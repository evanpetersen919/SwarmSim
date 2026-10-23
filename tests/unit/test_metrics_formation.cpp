#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "hand_logs.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/rng.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"

using swarm::AgentSnapshot;
using swarm::MetricsStatus;
using swarm::Scenario;
using swarm::Snapshot;
using swarm::Vec3;

using namespace swarm::testing;

TEST(MetricsFormation, DronesExactlyOnTheirSlotsGiveZeroError) {
  const Scenario s = make_scenario(5, 10);
  const std::vector<Snapshot> snaps(10, on_slots(s));
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok()) << out.message;
  ASSERT_TRUE(out.result.formation_rmse.has_value());
  EXPECT_EQ(*out.result.formation_rmse, 0.0);
  EXPECT_EQ(out.result.ticks, 10U);
  EXPECT_EQ(out.result.drones, 5U);
  EXPECT_EQ(out.result.alive_at_end, 5U);
}

TEST(MetricsFormation, AKnownConstantOffsetGivesExactlyThatError) {
  const Scenario s = make_scenario(4, 6);
  // Every drone 3 m east and 4 m north of its slot: an error of exactly 5 m.
  const std::vector<Snapshot> snaps(6, on_slots(s, std::vector<Vec3>(4, Vec3{3, 4, 0})));
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok()) << out.message;
  EXPECT_DOUBLE_EQ(*out.result.formation_rmse, 5.0);
}

TEST(MetricsFormation, AMixOfErrorsGivesTheHandComputedRms) {
  const Scenario s = make_scenario(4, 3);
  // Errors 0, 3, 4, 0 m: the mean square is (0 + 9 + 16 + 0) / 4, so the RMS is 2.5.
  const std::vector<Vec3> shift = {{0, 0, 0}, {3, 0, 0}, {0, 0, 4}, {0, 0, 0}};
  const auto out = analyze(make_log(s, std::vector<Snapshot>(3, on_slots(s, shift))));
  ASSERT_TRUE(out.ok());
  EXPECT_DOUBLE_EQ(*out.result.formation_rmse, 2.5);
}

TEST(MetricsFormation, TheWholeRunAndTheSettledValueDifferWhenTheStartIsWorse) {
  // dt is one second, so tick k describes time k + 1. With settle_s = 3 the settled ticks are
  // those with k + 1 >= 3, that is ticks 2 and 3.
  const Scenario s = make_scenario(2, 4, 3.0);
  std::vector<Snapshot> snaps;
  snaps.push_back(on_slots(s, {Vec3{10, 0, 0}, Vec3{10, 0, 0}}));
  snaps.push_back(on_slots(s, {Vec3{10, 0, 0}, Vec3{10, 0, 0}}));
  snaps.push_back(on_slots(s));
  snaps.push_back(on_slots(s));
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  EXPECT_DOUBLE_EQ(*out.result.formation_rmse, std::sqrt(50.0));  // (100 + 100 + 0 + 0) / 4
  ASSERT_TRUE(out.result.formation_rmse_settled.has_value());
  EXPECT_EQ(*out.result.formation_rmse_settled, 0.0);
}

TEST(MetricsFormation, NoRecordAtOrAfterTheSettleTimeGivesNoSettledValue) {
  const Scenario s = make_scenario(2, 3, 100.0);
  const auto out = analyze(make_log(s, std::vector<Snapshot>(3, on_slots(s))));
  ASSERT_TRUE(out.ok());
  EXPECT_TRUE(out.result.formation_rmse.has_value());
  EXPECT_FALSE(out.result.formation_rmse_settled.has_value());  // empty, not zero
}

TEST(MetricsFormation, AFailedDroneIsLeftOutOfTheErrorButNotOutOfTheSeparation) {
  const Scenario s = make_scenario(3, 4);
  std::vector<Snapshot> snaps;
  for (int k = 0; k < 4; ++k) {
    Snapshot snap = on_slots(s);
    if (k >= 2) {
      // Drone 1 has failed, 50 m from its slot, and lies 5 m from drone 2's slot position.
      snap.agents[1] = at(kCenter + slots(s)[2] + Vec3{5, 0, 0}, false);
    }
    snaps.push_back(snap);
  }
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(*out.result.formation_rmse, 0.0);  // the failed drone's error does not count
  EXPECT_EQ(out.result.alive_at_end, 2U);
  // The failed drone is still an object: it comes within 5 m of drone 2.
  EXPECT_DOUBLE_EQ(*out.result.min_separation, 5.0);
}

TEST(MetricsFormation, TheMinimumSeparationMatchesABruteForceMinimumOverARandomLog) {
  swarm::Rng rng(21, 0);
  const Scenario s = make_scenario(7, 40);
  std::vector<Snapshot> snaps;
  double expected = std::numeric_limits<double>::infinity();
  for (int k = 0; k < 40; ++k) {
    Snapshot snap;
    for (std::uint32_t i = 0; i < 7; ++i) {
      snap.agents.push_back(
          at({rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(0, 30)}, rng.bernoulli(0.8)));
    }
    for (std::size_t i = 0; i < 7; ++i) {
      for (std::size_t j = i + 1; j < 7; ++j) {
        const auto& a = snap.agents[i].position;
        const auto& b = snap.agents[j].position;
        expected = std::min(
            expected, std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                                (a[2] - b[2]) * (a[2] - b[2])));
      }
    }
    snaps.push_back(snap);
  }
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  ASSERT_TRUE(out.result.min_separation.has_value());
  EXPECT_NEAR(*out.result.min_separation, expected, 1e-12);
}

TEST(MetricsFormation, ASingleDroneHasNoSeparationAndNoCollisions) {
  const Scenario s = make_scenario(1, 5);
  const auto out = analyze(make_log(s, std::vector<Snapshot>(5, on_slots(s))));
  ASSERT_TRUE(out.ok());
  EXPECT_FALSE(out.result.min_separation.has_value());
  EXPECT_EQ(out.result.collision_episodes, 0U);
}

namespace {

// Two drones whose distance over time is `gaps` meters, on the x axis.
Bytes two_drones_with_gaps(const std::vector<double>& gaps) {
  const Scenario s = make_scenario(2, gaps.size());
  std::vector<Snapshot> snaps;
  for (double gap : gaps) {
    Snapshot snap;
    snap.agents.push_back(at({0, 0, 20}));
    snap.agents.push_back(at({gap, 0, 20}));
    snaps.push_back(snap);
  }
  return make_log(s, snaps);
}

}  // namespace

TEST(MetricsFormation, ACollisionIsAnEpisodeCountedOnceHoweverLongItLasts) {
  // The radius is 1 m. The pair is inside for five ticks in a row: one episode.
  const auto out = analyze(two_drones_with_gaps({5, 0.5, 0.4, 0.3, 0.6, 0.9, 5}));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.collision_episodes, 1U);
  EXPECT_DOUBLE_EQ(*out.result.min_separation, 0.3);
}

TEST(MetricsFormation, ADipInsideTheRadiusTwiceIsTwoEpisodes) {
  const auto out = analyze(two_drones_with_gaps({5, 0.5, 3, 4, 0.2, 5, 0.9}));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.collision_episodes, 3U);  // ticks 1, 4, and 6
  const auto two = analyze(two_drones_with_gaps({5, 0.5, 3, 0.2, 5}));
  EXPECT_EQ(two.result.collision_episodes, 2U);
}

TEST(MetricsFormation, ADistanceEqualToTheRadiusIsNotACollision) {
  const auto out = analyze(two_drones_with_gaps({5, 1.0, 1.0, 5}));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.collision_episodes, 0U);
  EXPECT_DOUBLE_EQ(*out.result.min_separation, 1.0);
}

TEST(MetricsFormation, APairAlreadyInsideInTheFirstRecordStartsAnEpisodeThere) {
  const auto out = analyze(two_drones_with_gaps({0.5, 0.5, 5}));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.collision_episodes, 1U);
}

TEST(MetricsFormation, EpisodesAreCountedPerPairAcrossAllPairs) {
  const Scenario s = make_scenario(3, 4);
  std::vector<Snapshot> snaps;
  const double gaps01[4] = {5, 0.5, 5, 5};
  const double gaps12[4] = {5, 5, 5, 0.5};
  for (int k = 0; k < 4; ++k) {
    Snapshot snap;
    snap.agents.push_back(at({0, 0, 20}));
    snap.agents.push_back(at({gaps01[k], 0, 20}));
    snap.agents.push_back(at({gaps01[k] + gaps12[k], 0, 20}));
    snaps.push_back(snap);
  }
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.collision_episodes, 2U);  // pair (0,1) at tick 1, pair (1,2) at tick 3
}

TEST(MetricsFormation, RecordsWithoutDroneStateAreSkipped) {
  const Scenario s = make_scenario(2, 4);
  std::vector<Snapshot> snaps = {on_slots(s, {Vec3{2, 0, 0}, Vec3{2, 0, 0}}), Snapshot{},
                                 Snapshot{}, on_slots(s, {Vec3{4, 0, 0}, Vec3{4, 0, 0}})};
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok());
  EXPECT_EQ(out.result.ticks, 4U);
  EXPECT_DOUBLE_EQ(*out.result.formation_rmse, std::sqrt((4.0 + 16.0) / 2.0));
}

TEST(MetricsFormation, UnusableLogsReturnAClearStatus) {
  const Scenario s = make_scenario(2, 3);

  // No records at all.
  EXPECT_EQ(analyze(make_log(s, {})).status, MetricsStatus::kNoRecords);

  // Not a log.
  const Bytes junk = {1, 2, 3};
  EXPECT_EQ(analyze(junk).status, MetricsStatus::kLogError);

  // A demo log is not a scenario log.
  swarm::RunConfig demo;
  demo.agents = 2;
  demo.ticks = 5;
  Bytes demo_bytes;
  ASSERT_EQ(swarm::record_run(demo, 1, demo_bytes), swarm::LogStatus::kOk);
  EXPECT_EQ(analyze(demo_bytes).status, MetricsStatus::kNotScenarioLog);

  // A header whose scenario does not load.
  swarm::LogHeader header;
  header.agent_count = 2;
  header.dt = 1.0;
  header.config = std::string(swarm::kScenarioLogTag) + "version: 1\ncolour: red\n";
  swarm::LogWriter writer;
  ASSERT_EQ(writer.begin(header), swarm::LogStatus::kOk);
  ASSERT_EQ(writer.finish(), swarm::LogStatus::kOk);
  const auto bad = analyze(writer.bytes());
  EXPECT_EQ(bad.status, MetricsStatus::kBadScenario);
  EXPECT_FALSE(bad.message.empty());

  // A header that disagrees with its scenario.
  swarm::LogHeader mismatched;
  mismatched.agent_count = 9;
  mismatched.dt = 1.0;
  mismatched.config = std::string(swarm::kScenarioLogTag) + swarm::canonical_text(s);
  swarm::LogWriter w2;
  ASSERT_EQ(w2.begin(mismatched), swarm::LogStatus::kOk);
  ASSERT_EQ(w2.finish(), swarm::LogStatus::kOk);
  EXPECT_EQ(analyze(w2.bytes()).status, MetricsStatus::kBadScenario);
}

TEST(MetricsFormation, ACorruptSnapshotAndAWrongDroneCountAreReportedWithTheirTick) {
  const Scenario s = make_scenario(2, 3);
  {
    swarm::LogHeader header;
    header.agent_count = 2;
    header.dt = 1.0;
    header.config = std::string(swarm::kScenarioLogTag) + swarm::canonical_text(s);
    swarm::LogWriter writer;
    ASSERT_EQ(writer.begin(header), swarm::LogStatus::kOk);
    swarm::LogRecord good;
    good.tick = 0;
    ASSERT_EQ(swarm::encode_snapshot(on_slots(s), good.blob), swarm::SnapshotStatus::kOk);
    ASSERT_EQ(writer.append(good), swarm::LogStatus::kOk);
    swarm::LogRecord bad;
    bad.tick = 1;
    bad.blob = {0x09, 0x00};  // an unsupported snapshot version
    ASSERT_EQ(writer.append(bad), swarm::LogStatus::kOk);
    ASSERT_EQ(writer.finish(), swarm::LogStatus::kOk);
    const auto out = analyze(writer.bytes());
    EXPECT_EQ(out.status, MetricsStatus::kBadSnapshot);
    EXPECT_NE(out.message.find("tick 1"), std::string::npos);
  }
  {
    Snapshot three;
    three.agents.resize(3);
    const auto out = analyze(make_log(s, {three}));
    EXPECT_EQ(out.status, MetricsStatus::kBadSnapshot);
    EXPECT_NE(out.message.find("tick 0"), std::string::npos);
  }
}

TEST(MetricsFormation, AMovingMissionIsEvaluatedAtTheTimeTheStateDescribes) {
  // A mission along +x at 2 m/s with dt = 1 s: after the step of tick k the reference is at
  // (k + 1) * 2 m. Drones placed exactly there have zero error; one tick off, they do not.
  Scenario s = make_scenario(1, 4);
  s.mission.waypoints = {{0, 0, 20}, {100, 0, 20}};
  s.mission.speed = 2.0;
  std::vector<Snapshot> exact;
  std::vector<Snapshot> lagging;
  for (int k = 0; k < 4; ++k) {
    Snapshot a;
    a.agents.push_back(at({2.0 * (k + 1), 0, 20}));
    exact.push_back(a);
    Snapshot b;
    b.agents.push_back(at({2.0 * k, 0, 20}));
    lagging.push_back(b);
  }
  EXPECT_EQ(*analyze(make_log(s, exact)).result.formation_rmse, 0.0);
  EXPECT_DOUBLE_EQ(*analyze(make_log(s, lagging)).result.formation_rmse, 2.0);
}

TEST(MetricsFormation, FallbackOccupancyIsTheShareOfAliveDronesInEachState) {
  const Scenario s = make_scenario(5, 4);
  std::vector<Snapshot> snaps(4, on_slots(s));
  for (Snapshot& snap : snaps) {
    snap.agents[0].fallback_state = 1;  // degraded comms in every record
    snap.agents[4].alive = false;  // a failed drone that reports safe hold, which is not counted
    snap.agents[4].fallback_state = 3;
  }
  snaps[0].agents[1].fallback_state = 2;  // degraded navigation in two records
  snaps[1].agents[1].fallback_state = 2;
  snaps[2].agents[2].fallback_state = 3;  // safe hold in one record
  const auto out = analyze(make_log(s, snaps));
  ASSERT_TRUE(out.ok()) << out.message;
  // Four alive drones in each of four records.
  ASSERT_TRUE(out.result.fallback_comms.has_value());
  EXPECT_DOUBLE_EQ(*out.result.fallback_comms, 4.0 / 16.0);
  EXPECT_DOUBLE_EQ(*out.result.fallback_navigation, 2.0 / 16.0);
  EXPECT_DOUBLE_EQ(*out.result.fallback_safe_hold, 1.0 / 16.0);
}

TEST(MetricsFormation, ANominalRunHasZeroFallbackOccupancy) {
  const Scenario s = make_scenario(5, 4);
  const auto out = analyze(make_log(s, std::vector<Snapshot>(4, on_slots(s))));
  ASSERT_TRUE(out.ok()) << out.message;
  EXPECT_EQ(*out.result.fallback_comms, 0.0);
  EXPECT_EQ(*out.result.fallback_navigation, 0.0);
  EXPECT_EQ(*out.result.fallback_safe_hold, 0.0);
}
