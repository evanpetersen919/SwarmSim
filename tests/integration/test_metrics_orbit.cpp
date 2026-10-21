#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"

using swarm::LogStatus;
using swarm::MetricsOutcome;
using swarm::Scenario;

namespace {

using Bytes = std::vector<std::uint8_t>;

std::string read(const std::string& name) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/" + name, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::string replace(std::string text, const std::string& from, const std::string& to) {
  const std::size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  if (at != std::string::npos) {
    text.replace(at, from.size(), to);
  }
  return text;
}

MetricsOutcome analyze(const std::string& text, int duration_s) {
  const swarm::LoadResult r = swarm::load_scenario(
      replace(text, "duration_s: 300", "duration_s: " + std::to_string(duration_s)));
  EXPECT_TRUE(r.ok());
  Bytes bytes;
  EXPECT_EQ(swarm::run_scenario(r.scenario, 2, bytes), LogStatus::kOk);
  return swarm::analyze_log(bytes.data(), bytes.size());
}

}  // namespace

TEST(MetricsOrbit, AFixedWingRunHasOrbitMetricsAndAQuadRunHasNone) {
  const MetricsOutcome wing = analyze(read("fixed_wing_orbit.yaml"), 60);
  ASSERT_TRUE(wing.ok()) << wing.message;
  EXPECT_TRUE(wing.result.orbit_radial_rmse.has_value());
  EXPECT_TRUE(wing.result.orbit_altitude_rmse.has_value());
  EXPECT_TRUE(wing.result.orbit_phase_rmse.has_value());
  EXPECT_TRUE(wing.result.time_on_station.has_value());

  const swarm::LoadResult q = swarm::load_scenario(read("minimal.yaml"));
  ASSERT_TRUE(q.ok());
  Bytes bytes;
  ASSERT_EQ(swarm::run_scenario(q.scenario, 1, bytes), LogStatus::kOk);
  const MetricsOutcome quad = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(quad.ok());
  EXPECT_FALSE(quad.result.orbit_radial_rmse.has_value());
  EXPECT_FALSE(quad.result.time_on_station.has_value());
}

TEST(MetricsOrbit, AnEstablishedSwarmIsCloseToItsReferenceAndOnStation) {
  // The aircraft start on their slots with 10 m of jitter, so the error is small and stays small.
  const MetricsOutcome out = analyze(read("fixed_wing_orbit.yaml"), 60);
  ASSERT_TRUE(out.ok()) << out.message;
  const swarm::MetricsResult& m = out.result;
  EXPECT_LT(*m.formation_rmse, 30.0);
  EXPECT_LT(*m.orbit_radial_rmse, 20.0);
  EXPECT_LT(*m.orbit_altitude_rmse, 20.0);
  EXPECT_LT(*m.orbit_phase_rmse, 30.0);
  EXPECT_GT(*m.time_on_station, 0.9);
  EXPECT_LE(*m.time_on_station, 1.0);
}

TEST(MetricsOrbit, TheFormationErrorIsTheDistanceToTheOrbitReferencePoint) {
  // The 3D error is at least each of its components, and is no more than their sum.
  const MetricsOutcome out = analyze(read("fixed_wing_orbit.yaml"), 60);
  ASSERT_TRUE(out.ok()) << out.message;
  const swarm::MetricsResult& m = out.result;
  const double radial = *m.orbit_radial_rmse;
  const double altitude = *m.orbit_altitude_rmse;
  const double phase = *m.orbit_phase_rmse;
  // The squared 3D error is, up to the chord-versus-arc difference of the phase term, the sum of
  // the squares of its three components, so the RMS errors satisfy this loose consistency check.
  const double combined = std::sqrt(radial * radial + altitude * altitude + phase * phase);
  EXPECT_NEAR(*m.formation_rmse, combined, 0.15 * combined);
}

TEST(MetricsOrbit, BlindingTheCameraRaisesTheOrbitError) {
  const std::string text = read("fixed_wing_orbit.yaml");
  const MetricsOutcome clear =
      analyze(replace(text, "visibility_m: 3000", "visibility_m: 20000"), 120);
  const MetricsOutcome blind =
      analyze(replace(text, "visibility_m: 3000", "visibility_m: 200"), 120);
  ASSERT_TRUE(clear.ok() && blind.ok());
  EXPECT_GT(*blind.result.orbit_phase_rmse, *clear.result.orbit_phase_rmse);
  EXPECT_LE(*blind.result.time_on_station, *clear.result.time_on_station);
}

TEST(MetricsOrbit, TheMetricsAreRepeatable) {
  const std::string text = read("fixed_wing_orbit.yaml");
  const MetricsOutcome a = analyze(text, 30);
  const MetricsOutcome b = analyze(text, 30);
  ASSERT_TRUE(a.ok() && b.ok());
  EXPECT_EQ(*a.result.orbit_phase_rmse, *b.result.orbit_phase_rmse);
  EXPECT_EQ(*a.result.time_on_station, *b.result.time_on_station);
}
