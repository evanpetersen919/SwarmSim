#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "swarm/scenario/scenario.hpp"
#include "swarm/weather.hpp"

using swarm::Weather;

// The fog check against a published relation.
//
// Source: R. H. Rasshofer, M. Spies, and H. Spies, "Influences of weather phenomena on automotive
// laser radar systems", Advances in Radio Science 9, 49-60 (2011), doi:10.5194/ars-9-49-2011.
// Fig. 7 compares laser radar return signals measured in fog at visibilities of 50, 100, 145 and
// 205 m with a simulation at 50, 100, 150 and 200 m over a range of 0 to 30 m, for a sensor whose
// droplet refractive index is given for 905 nm. The caption states the simulation assumed
//   alpha = 3 / Visibility  and  beta = 0.046 / Visibility per sr.
// The paper reports "very good agreement" between measurement and simulation; it gives no
// numeric error, so this test does not claim one. What it checks is that this project's fog model
// can reproduce the extinction relation that the paper validated, and by how much the default
// differs from it.

namespace {

constexpr double kPublishedConstant = 3.0;
constexpr double kVisibilities[] = {50.0, 100.0, 150.0, 200.0};
constexpr double kRanges[] = {5.0, 10.0, 20.0, 30.0};

double published_extinction(double visibility) { return kPublishedConstant / visibility; }

// Two-way transmission (the factor on a return) with the published extinction.
double published_two_way(double visibility, double range) {
  return std::exp(-2.0 * published_extinction(visibility) * range);
}

}  // namespace

TEST(FogFit, WithTheConstantSetToThePublishedValueTheModelReproducesTheRelation) {
  for (double v : kVisibilities) {
    Weather w = Weather::fog(v);
    w.fog_constant = kPublishedConstant;
    EXPECT_NEAR(w.extinction_per_m(), published_extinction(v), 1e-15) << v;
    for (double r : kRanges) {
      EXPECT_NEAR(w.intensity(1.0, r), published_two_way(v, r), 1e-14) << v << " m, " << r << " m";
    }
  }
}

TEST(FogFit, ThePublishedConstantIsTheFivePercentConventionAndTheDefaultIsTheTwoPercentOne) {
  // ln(1/0.05) = 2.996, which rounds to the paper's 3; ln(1/0.02) = 3.912 is the default.
  EXPECT_NEAR(std::log(1.0 / 0.05), kPublishedConstant, 0.005);
  EXPECT_NEAR(std::log(1.0 / 0.02), swarm::kKoschmiederConstant, 0.0005);
  EXPECT_EQ(Weather{}.fog_constant, swarm::kKoschmiederConstant);
}

TEST(FogFit, TheDefaultExtinctionIsLargerThanThePublishedOneByTheRatioOfTheConstants) {
  const double ratio = swarm::kKoschmiederConstant / kPublishedConstant;  // about 1.304
  for (double v : kVisibilities) {
    const double ours = Weather::fog(v).extinction_per_m();
    EXPECT_NEAR(ours / published_extinction(v), ratio, 1e-12) << v;
    EXPECT_GT(ours, published_extinction(v));
    // So the default returns are weaker, by a factor that grows with range: at 30 m and 100 m
    // visibility it is exp(-2 * (0.03912 - 0.03) * 30).
    for (double r : kRanges) {
      EXPECT_LT(Weather::fog(v).intensity(1.0, r), published_two_way(v, r));
    }
  }
  const double expected = std::exp(-2.0 * (3.912 - 3.0) / 100.0 * 30.0);
  EXPECT_NEAR(Weather::fog(100.0).intensity(1.0, 30.0) / published_two_way(100.0, 30.0), expected,
              1e-12);
}

TEST(FogFit, TheDetectionRangeWithThePublishedConstantIsTheDefaultsTimesTheRatio) {
  for (double v : kVisibilities) {
    Weather published = Weather::fog(v);
    published.fog_constant = kPublishedConstant;
    const Weather standard = Weather::fog(v);
    const double a = published.max_detection_range(0.6, 0.05);
    const double b = standard.max_detection_range(0.6, 0.05);
    EXPECT_NEAR(a / b, swarm::kKoschmiederConstant / kPublishedConstant, 1e-12) << v;
  }
  // At 200 m visibility: about 63 m by default, about 82 m with the published constant.
  Weather published = Weather::fog(200.0);
  published.fog_constant = kPublishedConstant;
  EXPECT_NEAR(Weather::fog(200.0).max_detection_range(0.6, 0.05), 63.5, 0.1);
  EXPECT_NEAR(published.max_detection_range(0.6, 0.05), 82.8, 0.1);
}

TEST(FogFit, TheFogConstantIsValidatedAndReadFromScenarios) {
  Weather w;
  w.fog_constant = 0.0;
  EXPECT_THROW(w.validate(), std::invalid_argument);
  w.fog_constant = -1.0;
  EXPECT_THROW(w.validate(), std::invalid_argument);
  w.fog_constant = 20.5;
  EXPECT_THROW(w.validate(), std::invalid_argument);
  w.fog_constant = std::nan("");
  EXPECT_THROW(w.validate(), std::invalid_argument);
  w.fog_constant = 3.0;
  EXPECT_NO_THROW(w.validate());
}

TEST(FogFit, AScenarioCanSetTheConstantAndItRoundTripsAndIsAbsentAtTheDefault) {
  const std::string head =
      "version: 1\nname: f\nseed: 1\nduration_s: 1\nswarm: {drones: 2}\n"
      "mission: {waypoints: [[0, 0, 20]], speed_mps: 3}\n";
  const swarm::LoadResult plain = swarm::load_scenario(head);
  ASSERT_TRUE(plain.ok());
  EXPECT_EQ(plain.scenario.weather.fog_constant, swarm::kKoschmiederConstant);
  EXPECT_EQ(swarm::canonical_text(plain.scenario).find("fog_constant"), std::string::npos);

  const swarm::LoadResult set =
      swarm::load_scenario(head + "weather: {visibility_m: 150, fog_constant: 3.0}\n");
  ASSERT_TRUE(set.ok());
  EXPECT_EQ(set.scenario.weather.fog_constant, 3.0);
  const std::string text = swarm::canonical_text(set.scenario);
  EXPECT_NE(text.find("  fog_constant: 3"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok());
  EXPECT_EQ(again.scenario.weather.fog_constant, 3.0);

  EXPECT_FALSE(swarm::load_scenario(head + "weather: {fog_constant: 0}\n").ok());
  EXPECT_FALSE(swarm::load_scenario(head + "weather: {fog_constant: 21}\n").ok());
  EXPECT_FALSE(swarm::load_scenario(head + "weather: {fog_constant: abc}\n").ok());
}
