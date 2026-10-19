#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "test_scale.hpp"

using swarm::Scenario;
using swarm::Vec3;

namespace {

Scenario hold_resume() {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/hold_resume.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  Scenario s = r.scenario;
  s.duration_ticks = static_cast<std::uint64_t>(swarm::testing::scaled(3200, 2800));
  return s;
}

// The formation RMSE at every tick, from the snapshots in a log.
std::vector<double> error_series(const std::vector<std::uint8_t>& bytes, const Scenario& s) {
  swarm::LogReader reader(bytes.data(), bytes.size());
  EXPECT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  const swarm::MissionPath path(s.mission);
  const std::vector<Vec3> offsets =
      swarm::make_formation(s.formation_shape, s.drones, s.formation_spacing_m);
  std::vector<double> out;
  swarm::LogRecord rec;
  while (reader.next(rec) == swarm::LogStatus::kOk) {
    swarm::Snapshot snap;
    EXPECT_EQ(swarm::decode_snapshot(rec.blob.data(), rec.blob.size(), snap),
              swarm::SnapshotStatus::kOk);
    EXPECT_EQ(snap.agents.size(), s.drones);  // the station is not a drone
    const Vec3 center = path.at(static_cast<double>(rec.tick + 1) * s.dt).position;
    double sum = 0.0;
    for (std::size_t i = 0; i < snap.agents.size(); ++i) {
      const Vec3 p{snap.agents[i].position[0], snap.agents[i].position[1],
                   snap.agents[i].position[2]};
      sum += swarm::norm_squared(p - (center + offsets[i]));
    }
    out.push_back(std::sqrt(sum / static_cast<double>(snap.agents.size())));
  }
  return out;
}

}  // namespace

TEST(ScenarioCommands, AHoldAndAResumeShowUpAsTheFormationErrorRisingAndFalling) {
  const Scenario s = hold_resume();
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 2, bytes), swarm::LogStatus::kOk);
  const std::vector<double> e = error_series(bytes, s);
  ASSERT_EQ(e.size(), s.duration_ticks);

  RecordProperty("error_before_hold_m", std::to_string(e[1490]));
  RecordProperty("error_end_of_hold_m", std::to_string(e[1790]));
  RecordProperty("error_end_m", std::to_string(e.back()));
  EXPECT_LT(e[1490], 1.5);  // flying the formation
  // Held for 6 s while the reference moves on at 4 m/s: about 24 m behind.
  EXPECT_GT(e[1790], 15.0);
  EXPECT_LT(e[1790], 30.0);
  EXPECT_LT(e.back(), 2.0);  // caught up again after the resume
}

TEST(ScenarioCommands, TheLogHoldsOnlyTheDronesAndVerifiesOnAnyThreadCount) {
  const Scenario s = hold_resume();
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 1, bytes), swarm::LogStatus::kOk);

  swarm::LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  EXPECT_EQ(reader.header().agent_count, 6U);  // the station is not counted

  for (std::size_t threads : {2U, 4U}) {
    const swarm::VerifyResult v = swarm::verify_run_log(bytes.data(), bytes.size(), threads);
    EXPECT_EQ(v.status, swarm::VerifyStatus::kOk) << v.message << " threads " << threads;
  }
  std::vector<std::uint8_t> other;
  ASSERT_EQ(swarm::run_scenario(s, 4, other), swarm::LogStatus::kOk);
  EXPECT_EQ(other, bytes);
}

TEST(ScenarioCommands, MetricsCountTheDronesAndIgnoreTheStation) {
  const Scenario s = hold_resume();
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_EQ(m.result.drones, 6U);
  EXPECT_EQ(m.result.alive_at_end, 6U);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  ASSERT_TRUE(m.result.min_separation.has_value());
  EXPECT_GT(*m.result.min_separation, 1.0);
  // The jam is a disturbance with a recovery time; a hold and a resume are not disturbances.
  EXPECT_EQ(m.result.recoveries.size(), 1U);
  EXPECT_EQ(m.result.recoveries[0].kind, swarm::DisturbanceKind::kJamEnd);
  EXPECT_GT(m.result.dropped_jam, 0U);
  EXPECT_GT(m.result.dropped_loss, 0U);
}

TEST(ScenarioCommands, ACommandAfterTheEndOfTheRunIsNeverSent) {
  Scenario s = hold_resume();
  s.duration_ticks = 1400;  // ends before the hold at tick 1500
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 1, bytes), swarm::LogStatus::kOk);
  EXPECT_EQ(swarm::verify_run_log(bytes.data(), bytes.size(), 1).status, swarm::VerifyStatus::kOk);
  const std::vector<double> e = error_series(bytes, s);
  EXPECT_LT(e.back(), 3.0);  // no hold happened
}
