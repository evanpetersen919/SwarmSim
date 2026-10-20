#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::Scenario;

namespace {

Scenario load(const std::string& name) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/" + name, std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << name;
  return r.scenario;
}

// Eight drones with burst loss and a jam of every link, long enough for the fallback to matter.
Scenario jammed() {
  Scenario s = load("formation_jam.yaml");
  s.duration_ticks = 600;
  return s;
}

std::vector<std::uint8_t> record(const Scenario& s) {
  std::vector<std::uint8_t> log;
  EXPECT_EQ(swarm::run_scenario(s, 2, log), swarm::LogStatus::kOk);
  return log;
}

}  // namespace

TEST(AblationRuns, TheSwitchesAtTheirDefaultsLeaveTheLogBytesUnchanged) {
  const Scenario s = jammed();
  Scenario explicit_on = s;
  explicit_on.fallback_enabled = true;
  explicit_on.separation_enabled = true;
  EXPECT_EQ(record(s), record(explicit_on));
}

TEST(AblationRuns, EachSwitchChangesTheRunAndTheLogStillVerifies) {
  const Scenario s = jammed();
  const std::vector<std::uint8_t> reference = record(s);

  Scenario no_fallback = s;
  no_fallback.fallback_enabled = false;
  Scenario no_separation = s;
  no_separation.separation_enabled = false;
  const std::vector<std::uint8_t> a = record(no_fallback);
  const std::vector<std::uint8_t> b = record(no_separation);
  EXPECT_NE(a, reference);
  EXPECT_NE(b, reference);
  EXPECT_NE(a, b);

  // The switch is part of the scenario in the header, so the log re-executes to the same run.
  for (const auto* log : {&a, &b}) {
    const swarm::VerifyResult v = swarm::verify_run_log(log->data(), log->size(), 3);
    EXPECT_EQ(v.status, swarm::VerifyStatus::kOk) << v.message;
  }
}

TEST(AblationRuns, WithoutSeparationDronesGetCloserWhileAssembling) {
  // A crowded start: the drones begin close together and are pushed apart by the rule.
  Scenario s = load("formation_jam.yaml");
  s.duration_ticks = 500;
  s.start.spacing_m = 6.0;
  s.start.jitter_m = 2.0;
  Scenario off = s;
  off.separation_enabled = false;
  const std::vector<std::uint8_t> with_log = record(s);
  const std::vector<std::uint8_t> without_log = record(off);
  const swarm::MetricsOutcome with = swarm::analyze_log(with_log.data(), with_log.size());
  const swarm::MetricsOutcome without = swarm::analyze_log(without_log.data(), without_log.size());
  ASSERT_TRUE(with.ok());
  ASSERT_TRUE(without.ok());
  ASSERT_TRUE(with.result.min_separation.has_value());
  ASSERT_TRUE(without.result.min_separation.has_value());
  EXPECT_LT(*without.result.min_separation, *with.result.min_separation);
}
