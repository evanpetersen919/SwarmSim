#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/camera_fix.hpp"
#include "swarm/weather.hpp"

using swarm::CameraFixConfig;
using swarm::CameraFixSensor;
using swarm::PositionFix;
using swarm::Vec3;
using swarm::Weather;

namespace {

CameraFixConfig no_dropout() {
  CameraFixConfig c;
  c.dropout_probability = 0.0;
  return c;
}

PositionFix fix_at(CameraFixSensor& s, double agl, const Weather& w) {
  return s.measure({10.0, 20.0, 100.0 + agl}, 100.0, w);
}

}  // namespace

TEST(CameraFix, InvalidConfigurationsAreRejected) {
  const auto rejected = [](CameraFixConfig c) {
    EXPECT_THROW(CameraFixSensor(c, 1, 0), std::invalid_argument);
  };
  CameraFixConfig c;
  c.min_agl_m = c.max_agl_m;
  rejected(c);
  c = CameraFixConfig{};
  c.min_transmission = 1.0;
  rejected(c);
  c = CameraFixConfig{};
  c.dropout_probability = 1.0;
  rejected(c);
  c = CameraFixConfig{};
  c.max_sigma_m = 1.0;
  rejected(c);
}

TEST(CameraFix, ClearAirGivesTheFloorAtLowAltitudeAndGrowsWithHeight) {
  CameraFixSensor sensor(no_dropout(), 1, 0);
  const PositionFix low = fix_at(sensor, 50.0, Weather::clear());
  ASSERT_TRUE(low.available);
  // At 50 m: max(3, 0.02 * 50 = 1) = 3, and clear air has a transmission just under 1.
  EXPECT_NEAR(low.sigma, 3.0, 0.05);
  const PositionFix high = fix_at(sensor, 400.0, Weather::clear());
  ASSERT_TRUE(high.available);
  EXPECT_GT(high.sigma, low.sigma);
  // Even clear air (20 km visibility) transmits a little less than 1 over 400 m.
  const double transmission = std::exp(-Weather::clear().extinction_per_m() * 400.0);
  EXPECT_NEAR(high.sigma, 0.02 * 400.0 / transmission, 1e-9);
}

TEST(CameraFix, FogDegradesTheFixAndRemovesItPastTheTransmissionLimit) {
  CameraFixSensor sensor(no_dropout(), 1, 0);
  const Weather fog = Weather::fog(400.0);
  const PositionFix clear = fix_at(sensor, 100.0, Weather::clear());
  const PositionFix foggy = fix_at(sensor, 100.0, fog);
  ASSERT_TRUE(clear.available);
  ASSERT_TRUE(foggy.available);
  EXPECT_GT(foggy.sigma, clear.sigma);
  // The limit height is ln(1 / min_transmission) / alpha.
  const double limit = std::log(1.0 / 0.2) / fog.extinction_per_m();
  EXPECT_TRUE(fix_at(sensor, 0.95 * limit, fog).available);
  EXPECT_FALSE(fix_at(sensor, 1.05 * limit, fog).available);
}

TEST(CameraFix, SigmaGrowsMonotonicallyWithFogAndIsCapped) {
  CameraFixSensor sensor(no_dropout(), 1, 0);
  double last = 0.0;
  for (const double visibility : {20000.0, 5000.0, 2000.0, 1000.0, 700.0}) {
    const PositionFix f = fix_at(sensor, 100.0, Weather::fog(visibility));
    ASSERT_TRUE(f.available) << visibility;
    EXPECT_GE(f.sigma, last);
    EXPECT_LE(f.sigma, 100.0);
    last = f.sigma;
  }
}

TEST(CameraFix, NoFixOutsideTheHeightBand) {
  CameraFixSensor sensor(no_dropout(), 1, 0);
  EXPECT_FALSE(fix_at(sensor, 5.0, Weather::clear()).available);
  EXPECT_FALSE(fix_at(sensor, 2000.0, Weather::clear()).available);
  EXPECT_FALSE(sensor.measure({0.0, 0.0, 50.0}, 80.0, Weather::clear()).available);
}

TEST(CameraFix, AnUnavailableFixCarriesTheSigmaCeilingAndNoPosition) {
  CameraFixSensor sensor(no_dropout(), 1, 0);
  const PositionFix f = fix_at(sensor, 2000.0, Weather::clear());
  EXPECT_FALSE(f.available);
  EXPECT_EQ(f.sigma, 100.0);
  EXPECT_EQ(f.position, Vec3{});
}

TEST(CameraFix, TheDropoutRateMatchesItsProbability) {
  CameraFixConfig c;
  c.dropout_probability = 0.25;
  CameraFixSensor sensor(c, 7, 3);
  int lost = 0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    lost += fix_at(sensor, 100.0, Weather::clear()).available ? 0 : 1;
  }
  EXPECT_NEAR(static_cast<double>(lost) / n, 0.25, 0.02);
}

TEST(CameraFix, NoiseDoesNotDependOnEarlierOutcomes) {
  // Two sensors with the same seed see different histories, one in fog where the fix is lost and
  // one in clear air. Under the same conditions afterward their noise must match.
  CameraFixSensor a(no_dropout(), 5, 1);
  CameraFixSensor b(no_dropout(), 5, 1);
  fix_at(a, 100.0, Weather::fog(100.0));
  fix_at(b, 100.0, Weather::clear());
  const PositionFix fa = fix_at(a, 100.0, Weather::clear());
  const PositionFix fb = fix_at(b, 100.0, Weather::clear());
  ASSERT_TRUE(fa.available);
  ASSERT_TRUE(fb.available);
  EXPECT_EQ(fa.position, fb.position);
}

TEST(CameraFix, TheSameSeedAndStreamRepeat) {
  CameraFixSensor a(CameraFixConfig{}, 9, 2);
  CameraFixSensor b(CameraFixConfig{}, 9, 2);
  for (int i = 0; i < 200; ++i) {
    const PositionFix fa = fix_at(a, 120.0, Weather::fog(3000.0));
    const PositionFix fb = fix_at(b, 120.0, Weather::fog(3000.0));
    ASSERT_EQ(fa.available, fb.available);
    ASSERT_EQ(fa.position, fb.position);
  }
}

TEST(CameraFix, NonFiniteInputIsRejected) {
  CameraFixSensor sensor(CameraFixConfig{}, 1, 0);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(sensor.measure({nan, 0.0, 100.0}, 0.0, Weather::clear()), std::invalid_argument);
  EXPECT_THROW(sensor.measure({0.0, 0.0, 100.0}, inf, Weather::clear()), std::invalid_argument);
}
