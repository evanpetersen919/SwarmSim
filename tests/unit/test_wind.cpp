#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/dynamics.hpp"
#include "swarm/wind.hpp"

using swarm::Vec3;
using swarm::WindField;
using swarm::WindParams;

namespace {

WindParams gusty() {
  WindParams p;
  p.mean = {4.0, -2.0, 0.0};
  p.gust_sigma = 1.5;
  p.gust_scale = 100.0;
  return p;
}

}  // namespace

TEST(Wind, NoGustsIsTheSteadyWindEverywhere) {
  WindParams p;
  p.mean = {3.0, 1.0, 0.0};
  const WindField field(p, 7);
  for (double t : {0.0, 10.0, 123.4}) {
    const Vec3 w = field.at({100.0 * t, -50.0, 80.0}, t);
    EXPECT_EQ(w.x, 3.0);
    EXPECT_EQ(w.y, 1.0);
    EXPECT_EQ(w.z, 0.0);
  }
}

TEST(Wind, IsAFunctionOfSeedPositionAndTimeOnly) {
  const WindField a(gusty(), 11);
  const WindField b(gusty(), 11);
  const WindField c(gusty(), 12);
  const Vec3 p{120.0, -300.0, 90.0};
  EXPECT_EQ(a.at(p, 42.0).x, b.at(p, 42.0).x);
  EXPECT_EQ(a.at(p, 42.0).y, b.at(p, 42.0).y);
  EXPECT_EQ(a.at(p, 42.0).z, b.at(p, 42.0).z);
  EXPECT_NE(a.at(p, 42.0).x, c.at(p, 42.0).x);
}

TEST(Wind, GustsHaveTheRequestedRms) {
  // Averaged over many places and times, the rms of each horizontal gust component is the sigma,
  // within the sampling error of eight waves.
  WindParams p = gusty();
  p.mean = {};
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  int n = 0;
  for (std::uint64_t seed = 1; seed <= 20; ++seed) {
    const WindField field(p, seed);
    for (int i = 0; i < 200; ++i) {
      const Vec3 w = field.at({37.0 * i, -53.0 * i, 0.0}, 1.7 * i);
      sum_x += w.x * w.x;
      sum_y += w.y * w.y;
      sum_z += w.z * w.z;
      ++n;
    }
  }
  EXPECT_NEAR(std::sqrt(sum_x / n), p.gust_sigma, 0.25 * p.gust_sigma);
  EXPECT_NEAR(std::sqrt(sum_y / n), p.gust_sigma, 0.25 * p.gust_sigma);
  EXPECT_NEAR(std::sqrt(sum_z / n), p.vertical_ratio * p.gust_sigma, 0.25 * p.gust_sigma);
}

TEST(Wind, IsSmoothInSpaceAndTime) {
  const WindField field(gusty(), 5);
  const Vec3 a = field.at({0.0, 0.0, 100.0}, 10.0);
  const Vec3 near_in_space = field.at({1.0, 0.0, 100.0}, 10.0);
  const Vec3 near_in_time = field.at({0.0, 0.0, 100.0}, 10.02);
  // The shortest wave is a fifth of the largest scale, 20 m here, so a meter moves its phase by
  // 2 pi / 20 radians and a fiftieth of a second moves it by much less. The sum of eight waves of
  // amplitude 2 sigma / sqrt(8) cannot change by more than the sum of their phase changes.
  const double amplitude = 2.0 * gusty().gust_sigma / std::sqrt(8.0);
  const double per_meter = 8.0 * amplitude * 2.0 * swarm::kPi / 20.0;
  EXPECT_LT(std::abs(near_in_space.x - a.x), per_meter);
  EXPECT_LT(std::abs(near_in_time.x - a.x), 0.1);
}

TEST(Wind, GustsAreCarriedByTheMeanWind) {
  // With no change of their own the gusts move with the mean wind, so the air a parcel carries is
  // the same: the field at (x + U t, t) equals the field at (x, 0). The eddy rate is small, so the
  // difference after a few seconds stays small next to the sigma.
  const WindParams p = gusty();
  const WindField field(p, 9);
  const double t = 3.0;
  const Vec3 later = field.at({50.0 + p.mean.x * t, 20.0 + p.mean.y * t, 100.0}, t);
  const Vec3 earlier = field.at({50.0, 20.0, 100.0}, 0.0);
  EXPECT_LT(std::abs(later.x - earlier.x), 0.6 * p.gust_sigma);
}

TEST(Wind, RejectsBadInput) {
  WindParams p = gusty();
  p.gust_sigma = -1.0;
  EXPECT_THROW(WindField(p, 1), std::invalid_argument);
  p = gusty();
  p.gust_scale = 0.0;
  EXPECT_THROW(WindField(p, 1), std::invalid_argument);
  p = gusty();
  p.vertical_ratio = 1.5;
  EXPECT_THROW(WindField(p, 1), std::invalid_argument);
  p = gusty();
  p.mean.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(WindField(p, 1), std::invalid_argument);
  const WindField ok(gusty(), 1);
  EXPECT_THROW(ok.at({std::numeric_limits<double>::infinity(), 0.0, 0.0}, 0.0),
               std::invalid_argument);
  EXPECT_THROW(ok.at({0.0, 0.0, 0.0}, std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}
