#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/dynamics.hpp"

using swarm::DroneCommand;
using swarm::DroneParams;
using swarm::DroneState;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.01;

DroneParams generous() {
  DroneParams p;
  p.max_speed = 1000.0;
  p.max_accel = 1000.0;
  p.max_yaw_rate = 100.0;
  return p;
}

DroneState run(DroneState s, const DroneCommand& c, const DroneParams& p, int steps) {
  for (int i = 0; i < steps; ++i) {
    s = swarm::step(s, c, p, kDt);
  }
  return s;
}

}  // namespace

TEST(Dynamics, ConstantAccelerationFollowsTheDiscreteClosedForm) {
  const Vec3 a{1.0, 2.0, -0.5};
  DroneCommand command;
  command.accel = a;
  const int n = 250;
  const DroneState s = run(DroneState{}, command, generous(), n);

  // Semi-implicit Euler: v = a*n*dt and p = a*dt^2*n*(n+1)/2.
  const double vn = n * kDt;
  const double pn = kDt * kDt * n * (n + 1) / 2.0;
  EXPECT_NEAR(s.vel.x, a.x * vn, 1e-9);
  EXPECT_NEAR(s.vel.y, a.y * vn, 1e-9);
  EXPECT_NEAR(s.vel.z, a.z * vn, 1e-9);
  EXPECT_NEAR(s.pos.x, a.x * pn, 1e-9);
  EXPECT_NEAR(s.pos.y, a.y * pn, 1e-9);
  EXPECT_NEAR(s.pos.z, a.z * pn, 1e-9);
}

TEST(Dynamics, StaysCloseToTheContinuousSolution) {
  DroneCommand command;
  command.accel = {1.0, 0.0, 0.0};
  const int n = 500;
  const DroneState s = run(DroneState{}, command, generous(), n);
  const double t = n * kDt;
  // Position error of the scheme is 0.5*a*dt*t, first order in dt.
  EXPECT_NEAR(s.pos.x, 0.5 * t * t, 0.5 * kDt * t + 1e-9);
}

TEST(Dynamics, NoCommandKeepsConstantVelocity) {
  DroneState start;
  start.vel = {2.0, -1.0, 0.5};
  start.pos = {10.0, 0.0, 5.0};
  const DroneState s = run(start, DroneCommand{}, DroneParams{}, 100);
  EXPECT_EQ(s.vel, start.vel);
  EXPECT_NEAR(s.pos.x, 10.0 + 2.0 * 100 * kDt, 1e-9);
  EXPECT_NEAR(s.pos.y, 0.0 - 1.0 * 100 * kDt, 1e-9);
  EXPECT_EQ(s.accel, Vec3{});
}

TEST(Dynamics, CommandedAccelerationIsClampedKeepingDirection) {
  DroneParams p;
  p.max_accel = 5.0;
  DroneCommand command;
  command.accel = {30.0, 40.0, 0.0};  // length 50
  const DroneState s = swarm::step(DroneState{}, command, p, kDt);
  EXPECT_NEAR(swarm::norm(s.accel), 5.0, 1e-9);
  EXPECT_NEAR(s.accel.x, 3.0, 1e-9);
  EXPECT_NEAR(s.accel.y, 4.0, 1e-9);
}

TEST(Dynamics, SpeedSaturatesAtTheLimitAndAppliedAccelerationFallsToZero) {
  DroneParams p;
  p.max_speed = 4.0;
  p.max_accel = 10.0;
  DroneCommand command;
  command.accel = {6.0, 8.0, 0.0};
  const DroneState s = run(DroneState{}, command, p, 200);
  EXPECT_NEAR(swarm::norm(s.vel), 4.0, 1e-9);
  EXPECT_NEAR(s.vel.x / s.vel.y, 6.0 / 8.0, 1e-9);  // direction kept
  EXPECT_NEAR(swarm::norm(s.accel), 0.0, 1e-9);
}

TEST(Dynamics, VelocityNeverExceedsTheLimitAtAnyStep) {
  DroneParams p;
  p.max_speed = 3.0;
  DroneCommand command;
  command.accel = {100.0, -100.0, 50.0};
  DroneState s;
  for (int i = 0; i < 500; ++i) {
    s = swarm::step(s, command, p, kDt);
    ASSERT_LE(swarm::norm(s.vel), 3.0 + 1e-12);
  }
}

TEST(Dynamics, YawRateIsClampedAndYawWraps) {
  DroneParams p;
  p.max_yaw_rate = 2.0;
  DroneCommand command;
  command.yaw_rate = 50.0;
  DroneState s = swarm::step(DroneState{}, command, p, kDt);
  EXPECT_DOUBLE_EQ(s.yaw_rate, 2.0);
  EXPECT_NEAR(s.yaw, 2.0 * kDt, 1e-12);

  command.yaw_rate = -50.0;
  s = swarm::step(DroneState{}, command, p, kDt);
  EXPECT_DOUBLE_EQ(s.yaw_rate, -2.0);

  command.yaw_rate = 2.0;
  s = DroneState{};
  for (int i = 0; i < 2000; ++i) {
    s = swarm::step(s, command, p, kDt);
    ASSERT_GT(s.yaw, -swarm::kPi);
    ASSERT_LE(s.yaw, swarm::kPi);
  }
}

TEST(Dynamics, WrapAngleMapsIntoHalfOpenInterval) {
  EXPECT_NEAR(swarm::wrap_angle(0.0), 0.0, 1e-15);
  EXPECT_NEAR(swarm::wrap_angle(swarm::kPi), swarm::kPi, 1e-15);
  EXPECT_NEAR(swarm::wrap_angle(-swarm::kPi), swarm::kPi, 1e-15);
  EXPECT_NEAR(swarm::wrap_angle(3.0 * swarm::kPi), swarm::kPi, 1e-12);
  EXPECT_NEAR(swarm::wrap_angle(2.0 * swarm::kPi + 0.5), 0.5, 1e-12);
  EXPECT_NEAR(swarm::wrap_angle(-2.0 * swarm::kPi - 0.5), -0.5, 1e-12);
  EXPECT_NEAR(swarm::wrap_angle(100.0 * swarm::kPi + 0.25), 0.25, 1e-9);
}

TEST(Dynamics, RejectsInvalidInputs) {
  const DroneState s;
  const DroneCommand c;
  const DroneParams p;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW(swarm::step(s, c, p, 0.0), std::invalid_argument);
  EXPECT_THROW(swarm::step(s, c, p, -0.01), std::invalid_argument);
  EXPECT_THROW(swarm::step(s, c, p, nan), std::invalid_argument);
  EXPECT_THROW(swarm::step(s, c, p, inf), std::invalid_argument);

  DroneCommand bad;
  bad.accel = {nan, 0, 0};
  EXPECT_THROW(swarm::step(s, bad, p, kDt), std::invalid_argument);
  bad = DroneCommand{};
  bad.yaw_rate = inf;
  EXPECT_THROW(swarm::step(s, bad, p, kDt), std::invalid_argument);

  DroneParams zero_speed;
  zero_speed.max_speed = 0.0;
  EXPECT_THROW(swarm::step(s, c, zero_speed, kDt), std::invalid_argument);
  DroneParams negative_accel;
  negative_accel.max_accel = -1.0;
  EXPECT_THROW(swarm::validate(negative_accel), std::invalid_argument);
}

TEST(Dynamics, IsDeterministic) {
  DroneCommand command;
  command.accel = {0.3, -0.7, 0.1};
  command.yaw_rate = 0.4;
  const DroneState a = run(DroneState{}, command, DroneParams{}, 1000);
  const DroneState b = run(DroneState{}, command, DroneParams{}, 1000);
  EXPECT_EQ(a.pos, b.pos);
  EXPECT_EQ(a.vel, b.vel);
  EXPECT_EQ(a.yaw, b.yaw);
}
