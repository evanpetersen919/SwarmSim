#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/control/formation_control.hpp"
#include "swarm/control/separation.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/rng.hpp"

using swarm::SeparationNeighbor;
using swarm::SeparationParams;
using swarm::Vec3;

namespace {

SeparationNeighbor at(std::uint32_t slot, Vec3 position, double sigma = 0.0) {
  SeparationNeighbor n;
  n.slot = slot;
  n.position = position;
  n.sigma = sigma;
  return n;
}

}  // namespace

TEST(Separation, NothingOutsideTheMargin) {
  const SeparationParams p;
  const Vec3 a =
      swarm::separation_accel(0, {0, 0, 0}, {at(1, {p.d_safe, 0, 0}), at(2, {0, 50, 0})}, p);
  EXPECT_EQ(a.x, 0.0);
  EXPECT_EQ(a.y, 0.0);
  EXPECT_EQ(a.z, 0.0);
  const Vec3 none = swarm::separation_accel(0, {0, 0, 0}, {}, p);
  EXPECT_EQ(swarm::norm(none), 0.0);
}

TEST(Separation, InsideTheMarginItPointsAwayFromTheNeighbor) {
  const SeparationParams p;
  const Vec3 a = swarm::separation_accel(0, {0, 0, 0}, {at(1, {3, 4, 0})}, p);  // distance 5
  const double expected = p.gain * (p.d_safe - 5.0);
  EXPECT_NEAR(a.x, -expected * 0.6, 1e-12);
  EXPECT_NEAR(a.y, -expected * 0.8, 1e-12);
  EXPECT_NEAR(a.z, 0.0, 1e-12);
}

TEST(Separation, MagnitudeRisesWithPenetrationAndIsCapped) {
  SeparationParams p;
  double previous = 0.0;
  for (double d : {7.5, 7.0, 6.5, 6.0}) {
    const double m = swarm::norm(swarm::separation_accel(0, {0, 0, 0}, {at(1, {d, 0, 0})}, p));
    EXPECT_GT(m, previous) << "d " << d;
    previous = m;
  }
  const double close = swarm::norm(swarm::separation_accel(0, {0, 0, 0}, {at(1, {0.1, 0, 0})}, p));
  EXPECT_DOUBLE_EQ(close, p.max_accel);
}

TEST(Separation, TheMarginWidensWithTheNeighborsUncertainty) {
  const SeparationParams p;
  const Vec3 position{p.d_safe + 1.0, 0, 0};  // just outside the margin of an exact neighbor
  EXPECT_EQ(swarm::norm(swarm::separation_accel(0, {0, 0, 0}, {at(1, position, 0.0)}, p)), 0.0);
  const Vec3 widened = swarm::separation_accel(0, {0, 0, 0}, {at(1, position, 2.0)}, p);
  EXPECT_NEAR(widened.x, -p.gain * (p.kappa * 2.0 - 1.0), 1e-12);
}

TEST(Separation, TwoDronesGetEqualAndOppositePushes) {
  const SeparationParams p;
  const Vec3 pa{1, 2, 3};
  const Vec3 pb{4, 2, 5};
  const Vec3 a = swarm::separation_accel(0, pa, {at(1, pb)}, p);
  const Vec3 b = swarm::separation_accel(1, pb, {at(0, pa)}, p);
  EXPECT_NEAR(a.x, -b.x, 1e-12);
  EXPECT_NEAR(a.y, -b.y, 1e-12);
  EXPECT_NEAR(a.z, -b.z, 1e-12);
  EXPECT_GT(swarm::norm(a), 0.0);
}

TEST(Separation, SeveralNeighborsSumAndTheSumIsCapped) {
  SeparationParams p;
  p.max_accel = 100.0;
  const Vec3 left = swarm::separation_accel(0, {0, 0, 0}, {at(1, {4, 0, 0})}, p);
  const Vec3 up = swarm::separation_accel(0, {0, 0, 0}, {at(2, {0, 0, -5})}, p);
  const Vec3 both = swarm::separation_accel(0, {0, 0, 0}, {at(1, {4, 0, 0}), at(2, {0, 0, -5})}, p);
  EXPECT_NEAR(both.x, left.x + up.x, 1e-12);
  EXPECT_NEAR(both.z, left.z + up.z, 1e-12);

  p.max_accel = 5.0;  // each contribution is capped, and so is the sum
  const Vec3 capped = swarm::separation_accel(
      0, {0, 0, 0}, {at(1, {1, 0, 0}), at(2, {0, 1, 0}), at(3, {0, 0, 1})}, p);
  EXPECT_NEAR(swarm::norm(capped), 5.0, 1e-12);
}

TEST(Separation, CoincidentPositionsGiveAFiniteDeterministicOpposedDirection) {
  const SeparationParams p;
  const Vec3 spot{7, 7, 7};
  const Vec3 low = swarm::separation_accel(1, spot, {at(2, spot)}, p);
  const Vec3 high = swarm::separation_accel(2, spot, {at(1, spot)}, p);
  EXPECT_TRUE(std::isfinite(low.x) && std::isfinite(high.x));
  EXPECT_DOUBLE_EQ(low.x, -p.max_accel);
  EXPECT_DOUBLE_EQ(high.x, p.max_accel);
  EXPECT_EQ(low.y, 0.0);
  EXPECT_EQ(high.z, 0.0);
}

TEST(Separation, UnusableNeighborsAreIgnoredAndBadParametersThrow) {
  const SeparationParams p;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Vec3 a = swarm::separation_accel(
      0, {0, 0, 0},
      {at(0, {1, 0, 0}), at(1, {nan, 0, 0}), at(2, {1, 0, 0}, -1.0), at(3, {1, 0, 0}, nan)}, p);
  EXPECT_EQ(swarm::norm(a), 0.0);

  const auto rejects = [](SeparationParams q) {
    EXPECT_THROW(swarm::separation_accel(0, {}, {}, q), std::invalid_argument);
  };
  SeparationParams q;
  q.d_safe = 0.0;
  rejects(q);
  q = SeparationParams{};
  q.gain = -1.0;
  rejects(q);
  q = SeparationParams{};
  q.max_accel = nan;
  rejects(q);
  q = SeparationParams{};
  q.kappa = -0.1;
  rejects(q);
  EXPECT_THROW(swarm::separation_accel(0, {nan, 0, 0}, {}, SeparationParams{}),
               std::invalid_argument);
}

namespace {

// Two drones start 60 m apart on random courses and are each sent to the other's start point, so
// they must pass through one another. Perfect information, sigma zero. Returns the smallest
// distance between them over 500 seeded runs.
double worst_closest_approach(double max_speed) {
  const SeparationParams sep;
  swarm::FormationParams pd;
  swarm::DroneParams vehicle;
  vehicle.max_accel = sep.max_accel;
  vehicle.max_speed = max_speed;
  const double dt = 0.02;

  double worst = std::numeric_limits<double>::infinity();
  for (std::uint64_t seed = 0; seed < 500; ++seed) {
    swarm::Rng rng(seed, 0x500);
    const Vec3 direction =
        swarm::normalized({rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-0.3, 0.3)});
    const Vec3 start[2] = {direction * -30.0, direction * 30.0};
    swarm::DroneState state[2];
    for (int i = 0; i < 2; ++i) {
      state[i].pos = start[i];
    }
    for (int k = 0; k < 1500; ++k) {  // 30 s
      swarm::DroneCommand cmd[2];
      for (int i = 0; i < 2; ++i) {
        swarm::FormationTarget goal;
        goal.position = start[1 - i];
        const Vec3 pull = swarm::formation_accel(goal, state[i].pos, state[i].vel, pd);
        const Vec3 push =
            swarm::separation_accel(static_cast<std::uint32_t>(i), state[i].pos,
                                    {at(static_cast<std::uint32_t>(1 - i), state[1 - i].pos)}, sep);
        cmd[i].accel = swarm::combine_accel(pull, push, sep.max_accel);
      }
      for (int i = 0; i < 2; ++i) {
        state[i] = swarm::step(state[i], cmd[i], vehicle, dt);
      }
      worst = std::min(worst, swarm::distance(state[0].pos, state[1].pos));
    }
  }
  return worst;
}

}  // namespace

TEST(Separation, ConvergingPairsAtCruiseSpeedKeepTheirDistance) {
  const double worst = worst_closest_approach(5.0);
  EXPECT_GT(worst, 3.0) << "closest approach " << worst;
}

TEST(Separation, TheMarginIsNotEnoughWhenTheClosingSpeedExceedsWhatTheCapCanStop) {
  // At 15 m/s each, the closing speed is 30 m/s and stopping within the margin needs far more than
  // max_accel. This records the limitation: the closest approach falls below one meter.
  EXPECT_LT(worst_closest_approach(15.0), 1.0);
}

TEST(Separation, TheCombinedCommandGivesRepulsionPriorityWithinTheLimit) {
  const Vec3 pull{8, 0, 0};
  const Vec3 push{-12, 0, 0};
  const Vec3 both = swarm::combine_accel(pull, push, 12.0);
  EXPECT_NEAR(both.x, -12.0, 1e-12);  // the pull gets no headroom

  const Vec3 weak = swarm::combine_accel(pull, Vec3{-2, 0, 0}, 12.0);
  EXPECT_NEAR(weak.x, -2.0 + 8.0, 1e-12);  // headroom 10 covers the whole pull

  const Vec3 limited = swarm::combine_accel(Vec3{20, 0, 0}, Vec3{0, -6, 0}, 12.0);
  EXPECT_NEAR(limited.y, -6.0, 1e-12);
  EXPECT_NEAR(limited.x, 6.0, 1e-12);  // headroom 6
  EXPECT_THROW(swarm::combine_accel(pull, push, 0.0), std::invalid_argument);
}
