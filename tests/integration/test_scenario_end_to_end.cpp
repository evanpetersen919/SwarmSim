#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "swarm/drone_agent.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/metrics_csv.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "test_scale.hpp"

using swarm::DroneAgent;
using swarm::PredictorChoice;
using swarm::Scenario;
using swarm::Vec3;

namespace {

Scenario load(const std::string& name) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/" + name, std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << name;
  return r.scenario;
}

std::vector<std::string> cells(const std::string& line) {
  std::vector<std::string> out;
  std::string cell;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cell);
      cell.clear();
    } else if (c != '\n') {
      cell += c;
    }
  }
  out.push_back(cell);
  return out;
}

std::uint64_t ticks_for(int full, int sanitized) {
  return static_cast<std::uint64_t>(swarm::testing::scaled(full, sanitized));
}

struct Observed {
  swarm::MetricsResult metrics;
  double neighbor_error = 0.0;  // mean distance of an estimate from the previous tick's truth
  std::uint64_t fixes_applied = 0;
  std::uint64_t fixes_unavailable = 0;
  std::set<swarm::FallbackState> states;
};

// Records the scenario to a log and analyzes it, and separately steps the same scenario to read
// what the metrics do not cover: the estimate error, the fixes, and the fallback states.
Observed observe(const Scenario& s, std::uint64_t error_from_tick) {
  Observed o;
  std::vector<std::uint8_t> bytes;
  EXPECT_EQ(swarm::run_scenario(s, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  EXPECT_TRUE(m.ok()) << m.message;
  o.metrics = m.result;

  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(2);
  std::vector<Vec3> previous;
  for (const auto& a : world.agents) {
    previous.push_back(a.start);
  }
  double sum = 0.0;
  std::size_t count = 0;
  for (std::uint64_t tick = 0; tick < s.duration_ticks; ++tick) {
    sim->step();
    for (std::size_t i = 0; i < s.drones; ++i) {
      const auto& d = dynamic_cast<const DroneAgent&>(sim->agent(i));
      o.states.insert(d.brain().state());
      if (tick >= error_from_tick) {
        for (std::size_t j = 0; j < s.drones; ++j) {
          if (j != i) {
            sum += swarm::distance(d.brain().neighbor(static_cast<std::uint32_t>(j)).position,
                                   previous[j]);
            ++count;
          }
        }
      }
    }
    for (std::size_t i = 0; i < s.drones; ++i) {
      previous[i] = dynamic_cast<const DroneAgent&>(sim->agent(i)).truth().pos;
    }
  }
  for (std::size_t i = 0; i < s.drones; ++i) {
    const auto& d = dynamic_cast<const DroneAgent&>(sim->agent(i));
    o.fixes_applied += d.brain().pose_counters().fixes_applied;
    o.fixes_unavailable += d.brain().pose_counters().fixes_unavailable;
  }
  o.neighbor_error = count > 0 ? sum / static_cast<double>(count) : 0.0;
  return o;
}

// formation_jam.yaml without the jam and with a clean channel of one tick of latency.
Scenario clean_scenario(PredictorChoice predictor) {
  Scenario s = load("formation_jam.yaml");
  s.channel.jammers.clear();
  s.channel.loss_model = swarm::LossModel::kNone;
  s.channel.latency_ticks = 1;
  s.channel.jitter_ticks = 0;
  s.predictor = predictor;
  return s;
}

}  // namespace

TEST(EndToEnd, AtZeroLossAndInClearAirTheKalmanPredictorMatchesTheOracle) {
  const Observed k = observe(clean_scenario(PredictorChoice::kKalman), 1500);
  const Observed o = observe(clean_scenario(PredictorChoice::kOracle), 1500);
  ASSERT_TRUE(k.metrics.formation_rmse_settled && o.metrics.formation_rmse_settled);
  const double kalman = *k.metrics.formation_rmse_settled;
  const double oracle = *o.metrics.formation_rmse_settled;
  RecordProperty("kalman_rmse_settled_m", std::to_string(kalman));
  RecordProperty("oracle_rmse_settled_m", std::to_string(oracle));
  RecordProperty("kalman_neighbor_error_m", std::to_string(k.neighbor_error));
  RecordProperty("kalman_min_separation_m", std::to_string(*k.metrics.min_separation));
  RecordProperty("oracle_min_separation_m", std::to_string(*o.metrics.min_separation));
  EXPECT_LT(std::abs(kalman - oracle), 0.05);  // measured: under 0.01 m apart
  EXPECT_EQ(o.neighbor_error, 0.0);
  EXPECT_LT(k.neighbor_error, 0.5);
  EXPECT_EQ(k.metrics.collision_episodes, 0U);
  EXPECT_EQ(o.metrics.collision_episodes, 0U);
  EXPECT_EQ(k.states, std::set<swarm::FallbackState>{swarm::FallbackState::kNominal});
}

TEST(EndToEnd, PacketLossRaisesTheNeighborEstimateErrorButBarelyMovesTheFormation) {
  // Steady straight flight is easy to predict, and the shared mission reference anchors the
  // slots, so loss costs little. The error still grows with loss, and bursts cost more than
  // independent drops at the same mean.
  const std::vector<double> levels = swarm::testing::kSanitized
                                         ? std::vector<double>{0.0, 0.5}
                                         : std::vector<double>{0.0, 0.1, 0.3, 0.5};
  const std::vector<swarm::LossModel> models =
      swarm::testing::kSanitized
          ? std::vector<swarm::LossModel>{swarm::LossModel::kBurst}
          : std::vector<swarm::LossModel>{swarm::LossModel::kIndependent, swarm::LossModel::kBurst};
  const std::uint64_t ticks = ticks_for(3000, 1800);
  const std::uint64_t from = ticks_for(1500, 800);

  double burst_at_half = 0.0;
  double independent_at_half = 0.0;
  for (swarm::LossModel model : models) {
    double previous_error = -1.0;
    double rmse_at_zero = 0.0;
    for (double loss : levels) {
      Scenario s = clean_scenario(PredictorChoice::kKalman);
      s.duration_ticks = ticks;
      s.channel.loss_model = loss == 0.0 ? swarm::LossModel::kNone : model;
      s.channel.loss = loss;
      s.channel.mean_burst = 4.0;
      s.channel.latency_ticks = 2;
      s.channel.jitter_ticks = 1;
      const Observed o = observe(s, from);
      ASSERT_TRUE(o.metrics.formation_rmse_settled.has_value());
      ASSERT_TRUE(o.metrics.delivery_ratio.has_value());
      const std::string tag = std::string(model == swarm::LossModel::kBurst ? "burst" : "indep") +
                              "_" + std::to_string(static_cast<int>(loss * 100));
      RecordProperty("neighbor_error_m_" + tag, std::to_string(o.neighbor_error));
      RecordProperty("rmse_settled_m_" + tag, std::to_string(*o.metrics.formation_rmse_settled));
      RecordProperty("delivery_ratio_" + tag, std::to_string(*o.metrics.delivery_ratio));

      EXPECT_NEAR(*o.metrics.delivery_ratio, 1.0 - loss, 0.05) << tag;
      EXPECT_GE(o.neighbor_error, previous_error) << tag;  // grows with loss, or stays
      previous_error = o.neighbor_error;
      if (loss == 0.0) {
        rmse_at_zero = *o.metrics.formation_rmse_settled;
      }
      EXPECT_NEAR(*o.metrics.formation_rmse_settled, rmse_at_zero, 0.05) << tag;
      EXPECT_TRUE(std::isfinite(o.neighbor_error));
      if (loss == 0.5) {
        (model == swarm::LossModel::kBurst ? burst_at_half : independent_at_half) =
            o.neighbor_error;
      }
    }
  }
  if (!swarm::testing::kSanitized) {
    EXPECT_GT(burst_at_half, independent_at_half);
  }
}

TEST(EndToEnd, FogDenseEnoughToHideTheGroundRemovesTheFixesAndBreaksTheFormation) {
  // The loss of fixes is a cliff, not a slope: at 200 m visibility every fix survives, at 150 m
  // almost none do, because the steepest beam meets the ground 40 m away and its return is too
  // weak to detect beyond that.
  const std::vector<double> visibilities = swarm::testing::kSanitized
                                               ? std::vector<double>{20000.0, 150.0}
                                               : std::vector<double>{20000.0, 200.0, 150.0, 40.0};
  const std::uint64_t ticks = ticks_for(1500, 1200);
  std::uint64_t clear_fixes = 0;
  double clear_rmse = 0.0;
  for (double visibility : visibilities) {
    Scenario s = clean_scenario(PredictorChoice::kKalman);
    s.weather = swarm::Weather::fog(visibility);
    s.duration_ticks = ticks;
    const Observed o = observe(s, 500);
    ASSERT_TRUE(o.metrics.formation_rmse_settled.has_value());
    const std::string tag = std::to_string(static_cast<int>(visibility));
    RecordProperty("fixes_applied_" + tag, std::to_string(o.fixes_applied));
    RecordProperty("rmse_settled_m_" + tag, std::to_string(*o.metrics.formation_rmse_settled));
    if (visibility == 20000.0) {
      clear_fixes = o.fixes_applied;
      clear_rmse = *o.metrics.formation_rmse_settled;
      EXPECT_EQ(o.states.size(), 1U);
      continue;
    }
    if (visibility >= 200.0) {
      EXPECT_GE(o.fixes_applied + 8, clear_fixes);  // every fix survives
      EXPECT_LT(*o.metrics.formation_rmse_settled, 3.0 * clear_rmse);
    } else {
      EXPECT_LT(o.fixes_applied, clear_fixes / 50);
      EXPECT_GT(o.fixes_unavailable, 0U);
      EXPECT_TRUE(o.states.count(swarm::FallbackState::kDegradedNavigation) > 0);
      EXPECT_GT(*o.metrics.formation_rmse_settled, 10.0 * clear_rmse);
    }
  }
}

TEST(EndToEnd, EveryShippedExampleRunsAndVerifiesAndYieldsAMetricsRow) {
  for (const char* name : {"minimal.yaml", "formation_jam.yaml", "hills_fog_fault.yaml"}) {
    Scenario s = load(name);
    s.duration_ticks = 250;  // five seconds
    std::vector<std::uint8_t> bytes;
    ASSERT_EQ(swarm::run_scenario(s, 1, bytes), swarm::LogStatus::kOk) << name;
    const swarm::VerifyResult v = swarm::verify_run_log(bytes.data(), bytes.size(), 3);
    EXPECT_EQ(v.status, swarm::VerifyStatus::kOk) << name << ": " << v.message;
    const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
    ASSERT_TRUE(m.ok()) << name << ": " << m.message;
    EXPECT_EQ(m.result.ticks, 250U);
    EXPECT_EQ(m.result.scenario_name, s.name);
    const std::string row = swarm::metrics_csv_row(m.result);
    EXPECT_EQ(cells(row).size(), cells(swarm::metrics_csv_header()).size()) << name;
  }
}

TEST(EndToEnd, AFullScenarioWithLossAJamAndAFailureRecordsVerifiesAndFillsEveryColumn) {
  Scenario s = load("formation_jam.yaml");  // eight drones, burst loss, a jam from 2 s to 8 s
  s.duration_ticks = ticks_for(3000, 2000);
  s.faults.push_back({3, 1000});  // drone 3 fails at 20 s
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(s, 4, bytes), swarm::LogStatus::kOk);
  EXPECT_EQ(swarm::verify_run_log(bytes.data(), bytes.size(), 2).status, swarm::VerifyStatus::kOk);

  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_EQ(m.result.drones, 8U);
  EXPECT_EQ(m.result.alive_at_end, 7U);
  EXPECT_EQ(m.result.recoveries.size(), 2U);  // the end of the jam, and the failure
  EXPECT_GT(m.result.dropped_jam, 0U);
  EXPECT_GT(m.result.dropped_loss, 0U);

  const std::vector<std::string> row = cells(swarm::metrics_csv_row(m.result));
  const std::vector<std::string> header = cells(swarm::metrics_csv_header());
  ASSERT_EQ(row.size(), header.size());
  for (std::size_t i = 0; i < row.size(); ++i) {
    EXPECT_FALSE(row[i].empty()) << "empty cell: " << header[i];
  }
}
