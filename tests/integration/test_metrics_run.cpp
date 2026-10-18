#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "swarm/control/mission.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/world.hpp"
#include "test_scale.hpp"

using swarm::Scenario;
using swarm::Vec3;

namespace {

constexpr std::uint64_t kTicks = 600;  // 12 s

Scenario scenario() {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/formation_jam.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  Scenario s = r.scenario;
  s.drones = 6;
  s.duration_ticks = kTicks;
  s.metrics.settle_s = 6.0;
  s.faults.push_back({3, 400});
  return s;
}

}  // namespace

TEST(MetricsRun, AnalyzingARecordedRunMatchesAnIndependentComputationFromTheSimulation) {
  const Scenario s = scenario();
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome out = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(out.ok()) << out.message;

  // The same quantities from the simulation's own state, computed here, not by the library.
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  const swarm::MissionPath path(s.mission);
  double sum_all = 0.0, sum_settled = 0.0;
  std::uint64_t n_all = 0, n_settled = 0;
  double min_sep = std::numeric_limits<double>::infinity();
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
    const double time = static_cast<double>(tick + 1) * s.dt;
    const Vec3 center = path.at(time).position;
    for (std::size_t i = 0; i < 6; ++i) {
      const auto& a = dynamic_cast<const swarm::DroneAgent&>(sim->agent(i));
      if (a.alive()) {
        const double e = swarm::distance(a.truth().pos, center + world.offsets[i]);
        sum_all += e * e;
        ++n_all;
        if (time >= s.metrics.settle_s) {
          sum_settled += e * e;
          ++n_settled;
        }
      }
      for (std::size_t j = i + 1; j < 6; ++j) {
        const auto& b = dynamic_cast<const swarm::DroneAgent&>(sim->agent(j));
        min_sep = std::min(min_sep, swarm::distance(a.truth().pos, b.truth().pos));
      }
    }
  }

  ASSERT_TRUE(out.result.formation_rmse.has_value());
  ASSERT_TRUE(out.result.formation_rmse_settled.has_value());
  ASSERT_TRUE(out.result.min_separation.has_value());
  EXPECT_NEAR(*out.result.formation_rmse, std::sqrt(sum_all / static_cast<double>(n_all)), 1e-9);
  EXPECT_NEAR(*out.result.formation_rmse_settled,
              std::sqrt(sum_settled / static_cast<double>(n_settled)), 1e-9);
  EXPECT_NEAR(*out.result.min_separation, min_sep, 1e-9);
  EXPECT_EQ(out.result.ticks, kTicks);
  EXPECT_EQ(out.result.drones, 6U);
  EXPECT_EQ(out.result.alive_at_end, 5U);  // drone 3 failed at tick 400
  EXPECT_EQ(out.result.collision_episodes, 0U);
  EXPECT_GT(*out.result.formation_rmse, *out.result.formation_rmse_settled);  // assembly is worse
}

TEST(MetricsRun, TheResultIsReproducibleAndDoesNotDependOnTheThreadCountThatRecordedTheLog) {
  const Scenario s = scenario();
  std::vector<std::uint8_t> one;
  std::vector<std::uint8_t> eight;
  ASSERT_EQ(swarm::run_scenario(s, 1, one), swarm::LogStatus::kOk);
  ASSERT_EQ(swarm::run_scenario(s, 8, eight), swarm::LogStatus::kOk);
  const auto a = swarm::analyze_log(one.data(), one.size());
  const auto b = swarm::analyze_log(one.data(), one.size());
  const auto c = swarm::analyze_log(eight.data(), eight.size());
  ASSERT_TRUE(a.ok() && b.ok() && c.ok());
  EXPECT_EQ(*a.result.formation_rmse, *b.result.formation_rmse);
  EXPECT_EQ(*a.result.formation_rmse, *c.result.formation_rmse);
  EXPECT_EQ(*a.result.min_separation, *c.result.min_separation);
}

TEST(MetricsRun, ASnapshotStrideSeesFewerStatesSoItCanOnlyMissTheClosestApproach) {
  Scenario s = scenario();
  std::vector<std::uint8_t> every;
  ASSERT_EQ(swarm::run_scenario(s, 1, every), swarm::LogStatus::kOk);
  s.snapshot_every_ticks = 25;
  std::vector<std::uint8_t> strided;
  ASSERT_EQ(swarm::run_scenario(s, 1, strided), swarm::LogStatus::kOk);

  const auto full = swarm::analyze_log(every.data(), every.size());
  const auto some = swarm::analyze_log(strided.data(), strided.size());
  ASSERT_TRUE(full.ok() && some.ok());
  EXPECT_EQ(some.result.ticks, kTicks);  // every tick still has a record
  EXPECT_GE(*some.result.min_separation, *full.result.min_separation);
  EXPECT_EQ(some.result.alive_at_end, 5U);
}

TEST(MetricsRun, RecoveryTimesOnARecordedRunMatchAnIndependentScan) {
  const std::uint64_t kLong = swarm::testing::scaled(1800, 1500);
  // formation_jam.yaml jams every link from tick 100 to tick 400, and scenario() fails drone 3 at
  // tick 400, so there are two disturbances at the same tick.
  Scenario s = scenario();
  s.duration_ticks = kLong;
  s.metrics.recovery_threshold_m = 1.5;
  s.metrics.recovery_hold_s = 1.0;
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome out = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(out.ok()) << out.message;
  ASSERT_EQ(out.result.recoveries.size(), 2U);
  EXPECT_EQ(out.result.recoveries[0].kind, swarm::DisturbanceKind::kJamEnd);
  EXPECT_EQ(out.result.recoveries[1].kind, swarm::DisturbanceKind::kFault);
  EXPECT_EQ(out.result.recoveries[0].tick, 400U);
  EXPECT_EQ(out.result.recoveries[1].tick, 400U);

  // The error trace over alive drones, from the simulation's own state.
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  const swarm::MissionPath path(s.mission);
  std::vector<double> error;
  for (std::uint64_t tick = 0; tick < kLong; ++tick) {
    sim->step();
    const Vec3 center = path.at(static_cast<double>(tick + 1) * s.dt).position;
    double sum = 0.0;
    int alive = 0;
    for (std::size_t i = 0; i < 6; ++i) {
      const auto& a = dynamic_cast<const swarm::DroneAgent&>(sim->agent(i));
      if (a.alive()) {
        const double e = swarm::distance(a.truth().pos, center + world.offsets[i]);
        sum += e * e;
        ++alive;
      }
    }
    error.push_back(std::sqrt(sum / alive));
  }
  const std::uint64_t hold = 50;  // one second at 50 Hz
  bool found = false;
  std::uint64_t start = 0;
  for (std::uint64_t t = 400; t + hold <= kLong && !found; ++t) {
    bool all = true;
    for (std::uint64_t k = t; k < t + hold; ++k) {
      all = all && error[k] <= 1.5;
    }
    if (all) {
      found = true;
      start = t;
    }
  }
  for (const swarm::Recovery& r : out.result.recoveries) {
    EXPECT_EQ(r.censored, !found);
    if (found) {
      EXPECT_NEAR(r.seconds, static_cast<double>(start - 400) * s.dt, 1e-9);
    }
  }
  RecordProperty("recovery_after_jam_and_fault_s",
                 std::to_string(found ? static_cast<double>(start - 400) * s.dt : -1.0));
}

TEST(MetricsRun, MessageMetricsOnARecordedRunMatchTheChannelAndTheLossSetting) {
  Scenario s = scenario();
  s.faults.clear();
  s.drones = 4;
  s.duration_ticks = 600;
  s.channel.jammers.clear();
  s.channel.loss_model = swarm::LossModel::kIndependent;
  s.channel.loss = 0.3;
  s.channel.latency_ticks = 2;
  s.channel.jitter_ticks = 1;

  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome out = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(out.ok()) << out.message;
  const swarm::MetricsResult& m = out.result;

  // The channel's own statistics, from a separate run.
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  for (std::uint64_t tick = 0; tick < 600; ++tick) {
    sim->step();
  }
  const swarm::ChannelStats& stats = sim->channel().stats();
  EXPECT_EQ(m.messages_sent, stats.sent);
  EXPECT_EQ(m.messages_delivered, stats.delivered);
  EXPECT_EQ(m.dropped_loss, stats.dropped_loss);
  EXPECT_EQ(m.dropped_rate, 0U);
  EXPECT_EQ(m.dropped_jam, 0U);
  EXPECT_EQ(m.messages_sent, m.messages_delivered + m.dropped_loss + m.messages_in_flight);

  // 4 drones, 3 neighbors each, a report every 5 ticks for 600 ticks.
  EXPECT_EQ(m.messages_sent, 4U * 3U * 120U);
  ASSERT_TRUE(m.delivery_ratio.has_value());
  RecordProperty("delivery_ratio_at_30_percent_loss", std::to_string(*m.delivery_ratio));
  EXPECT_NEAR(*m.delivery_ratio, 0.7, 0.05);  // about 3.4 standard deviations at 1,440 messages

  // Latency is the channel latency plus a jitter draw of 0 or 1 ticks, at 0.02 s per tick.
  ASSERT_TRUE(m.latency_p50_s.has_value() && m.latency_p95_s.has_value());
  EXPECT_GE(*m.latency_p50_s, 2 * s.dt - 1e-12);
  EXPECT_LE(*m.latency_p95_s, 3 * s.dt + 1e-12);
  EXPECT_LE(*m.latency_p50_s, *m.latency_p95_s);

  // The snapshot stride does not change anything about messages.
  s.snapshot_every_ticks = 25;
  std::vector<std::uint8_t> strided;
  ASSERT_EQ(swarm::run_scenario(s, 1, strided), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome other = swarm::analyze_log(strided.data(), strided.size());
  ASSERT_TRUE(other.ok());
  EXPECT_EQ(other.result.messages_sent, m.messages_sent);
  EXPECT_EQ(other.result.messages_delivered, m.messages_delivered);
  EXPECT_EQ(*other.result.delivery_ratio, *m.delivery_ratio);
  EXPECT_EQ(*other.result.latency_p95_s, *m.latency_p95_s);
}

TEST(MetricsRun, JamDropsAppearInTheMessageMetrics) {
  Scenario s = scenario();
  s.faults.clear();
  s.drones = 4;
  s.duration_ticks = 500;  // the scenario's jam covers ticks 100 to 400
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 1, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome out = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(out.ok());
  EXPECT_GT(out.result.dropped_jam, 0U);
  EXPECT_LT(*out.result.delivery_ratio, 0.8);  // about 300 of 500 ticks are jammed
}
