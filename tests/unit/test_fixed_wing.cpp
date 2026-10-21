#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include "swarm/fixed_wing.hpp"

using swarm::DroneCommand;
using swarm::DroneState;
using swarm::FixedWingParams;
using swarm::FixedWingVehicle;
using swarm::IVehicle;
using swarm::kPi;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.01;

DroneState flying(double speed, double heading = 0.0) {
  DroneState s;
  s.vel = {speed * std::cos(heading), speed * std::sin(heading), 0.0};
  s.yaw = heading;
  return s;
}

DroneState run(const IVehicle& v, DroneState s, const DroneCommand& c, int steps) {
  for (int i = 0; i < steps; ++i) {
    s = v.step(s, c, kDt);
  }
  return s;
}

double speed_of(const DroneState& s) { return std::hypot(s.vel.x, s.vel.y); }

// The command of a pilot holding a constant sideways acceleration: the world-frame vector turns
// with the heading, as a controller recomputes it every tick.
DroneCommand sideways(const DroneState& s, double lateral) {
  DroneCommand c;
  c.accel = {-lateral * std::sin(s.yaw), lateral * std::cos(s.yaw), 0.0};
  return c;
}

}  // namespace

TEST(FixedWing, ADefaultParameterSetIsValid) {
  EXPECT_NO_THROW(swarm::validate(FixedWingParams{}));
}

TEST(FixedWing, InvalidParametersAreRejected) {
  const auto rejected = [](FixedWingParams p) {
    EXPECT_THROW(FixedWingVehicle(p).step(DroneState{}, DroneCommand{}, kDt),
                 std::invalid_argument);
  };
  FixedWingParams p;
  p.min_speed = p.max_speed;
  rejected(p);
  p = FixedWingParams{};
  p.max_bank = 1.6;
  rejected(p);
  p = FixedWingParams{};
  p.max_climb_rate = 0.0;
  rejected(p);
  p = FixedWingParams{};
  p.gravity = std::numeric_limits<double>::quiet_NaN();
  rejected(p);
}

TEST(FixedWing, NonFiniteInputsAndBadStepsAreRejected) {
  const FixedWingVehicle v{FixedWingParams{}};
  DroneCommand bad;
  bad.accel.x = std::numeric_limits<double>::infinity();
  EXPECT_THROW(v.step(flying(15.0), bad, kDt), std::invalid_argument);
  EXPECT_THROW(v.step(flying(15.0), DroneCommand{}, 0.0), std::invalid_argument);
  DroneState nan_state = flying(15.0);
  nan_state.pos.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(v.step(nan_state, DroneCommand{}, kDt), std::invalid_argument);
}

TEST(FixedWing, AnAircraftAtRestFliesAtMinimumSpeedAlongItsHeading) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  DroneState rest;
  rest.yaw = kPi / 2.0;
  const DroneState s = run(v, rest, DroneCommand{}, 100);
  EXPECT_NEAR(speed_of(s), p.min_speed, 1e-9);
  EXPECT_NEAR(s.pos.x, 0.0, 1e-9);
  EXPECT_NEAR(s.pos.y, p.min_speed * 100 * kDt, 1e-9);
}

TEST(FixedWing, NeverSlowerThanMinimumSpeedWhateverIsCommanded) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  DroneCommand stop;
  stop.accel = {-100.0, 0.0, 0.0};
  const DroneState s = run(v, flying(20.0), stop, 2000);
  EXPECT_NEAR(speed_of(s), p.min_speed, 1e-9);
}

TEST(FixedWing, SpeedIsLimitedToMaximumAndRateLimited) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  DroneCommand fast;
  fast.accel = {100.0, 0.0, 0.0};
  const DroneState one = v.step(flying(15.0), fast, kDt);
  EXPECT_NEAR(speed_of(one), 15.0 + p.max_accel * kDt, 1e-9);
  const DroneState many = run(v, flying(15.0), fast, 3000);
  EXPECT_NEAR(speed_of(many), p.max_speed, 1e-9);
}

TEST(FixedWing, ASteadyTurnFollowsTheCoordinatedTurnRate) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  const double speed = 20.0;
  const double lateral = 0.5 * p.max_lateral_accel();  // inside the bank limit
  const int n = 300;
  DroneState s = flying(speed);
  for (int i = 0; i < n; ++i) {
    s = v.step(s, sideways(s, lateral), kDt);
  }
  EXPECT_NEAR(speed_of(s), speed, 1e-9);
  EXPECT_NEAR(s.yaw, swarm::wrap_angle(lateral / speed * n * kDt), 1e-9);
}

TEST(FixedWing, ATurnTracesACircleOfRadiusVSquaredOverLateralAccel) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  const double speed = 20.0;
  const double lateral = 0.6 * p.max_lateral_accel();
  const double radius = speed * speed / lateral;
  DroneState s = flying(speed);
  // The center is `radius` to the left of the start, at (0, radius).
  double lo = 1e18;
  double hi = 0.0;
  const int n = static_cast<int>(2.0 * kPi * radius / speed / kDt);
  for (int i = 0; i < n; ++i) {
    s = v.step(s, sideways(s, lateral), kDt);
    const double d = std::hypot(s.pos.x, s.pos.y - radius);
    lo = std::min(lo, d);
    hi = std::max(hi, d);
  }
  EXPECT_NEAR(lo, radius, 0.01 * radius);
  EXPECT_NEAR(hi, radius, 0.01 * radius);
  // After one lap it is back near where it began.
  EXPECT_LT(std::hypot(s.pos.x, s.pos.y), 0.02 * radius);
}

TEST(FixedWing, TurnRateIsLimitedByTheBankAngle) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  const double speed = 20.0;
  DroneCommand hard;
  hard.accel = {0.0, 1000.0, 0.0};
  const DroneState s = v.step(flying(speed), hard, kDt);
  const double limit = p.max_lateral_accel() / speed;
  EXPECT_NEAR(s.yaw_rate, limit, 1e-9);
  const DroneState right = v.step(flying(speed), DroneCommand{{0.0, -1000.0, 0.0}, 0.0}, kDt);
  EXPECT_NEAR(right.yaw_rate, -limit, 1e-9);
}

TEST(FixedWing, TheYawRateCommandIsIgnored) {
  const FixedWingVehicle v{FixedWingParams{}};
  DroneCommand spin;
  spin.yaw_rate = 3.0;
  const DroneState s = run(v, flying(15.0), spin, 100);
  EXPECT_NEAR(s.yaw, 0.0, 1e-12);
}

TEST(FixedWing, ClimbRateAndVerticalAccelerationAreLimited) {
  const FixedWingParams p;
  const FixedWingVehicle v{p};
  DroneCommand up;
  up.accel = {0.0, 0.0, 100.0};
  const DroneState one = v.step(flying(15.0), up, kDt);
  EXPECT_NEAR(one.vel.z, p.max_vertical_accel * kDt, 1e-9);
  const DroneState climbing = run(v, flying(15.0), up, 2000);
  EXPECT_NEAR(climbing.vel.z, p.max_climb_rate, 1e-9);
  DroneCommand down;
  down.accel = {0.0, 0.0, -100.0};
  const DroneState descending = run(v, flying(15.0), down, 2000);
  EXPECT_NEAR(descending.vel.z, -p.max_descent_rate, 1e-9);
}

TEST(FixedWing, ReportedAccelerationIsTheVelocityChangeOverTheStep) {
  const FixedWingVehicle v{FixedWingParams{}};
  DroneCommand c;
  c.accel = {1.0, 2.0, 0.5};
  const DroneState start = flying(18.0, 0.4);
  const DroneState s = v.step(start, c, kDt);
  const Vec3 expected = (s.vel - start.vel) / kDt;
  EXPECT_NEAR(s.accel.x, expected.x, 1e-12);
  EXPECT_NEAR(s.accel.y, expected.y, 1e-12);
  EXPECT_NEAR(s.accel.z, expected.z, 1e-12);
}

TEST(FixedWing, TheSameInputsGiveIdenticalOutputs) {
  const FixedWingVehicle v{FixedWingParams{}};
  DroneCommand c;
  c.accel = {0.7, -1.9, 0.3};
  const DroneState a = run(v, flying(17.0, 1.1), c, 777);
  const DroneState b = run(v, flying(17.0, 1.1), c, 777);
  EXPECT_EQ(a.pos, b.pos);
  EXPECT_EQ(a.vel, b.vel);
  EXPECT_EQ(a.yaw, b.yaw);
}
