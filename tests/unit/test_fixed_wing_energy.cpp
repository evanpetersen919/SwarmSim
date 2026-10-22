#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/fixed_wing_energy.hpp"

using swarm::DroneCommand;
using swarm::DroneState;
using swarm::EnergyParams;
using swarm::FixedWingEnergyVehicle;
using swarm::FixedWingParams;
using swarm::FixedWingVehicle;
using swarm::kPi;
using swarm::QuadVehicle;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.02;
constexpr double kG = 9.80665;

FixedWingEnergyVehicle make() { return FixedWingEnergyVehicle(FixedWingParams{}, EnergyParams{}); }

DroneState flying(double speed, double heading = 0.0, double climb = 0.0) {
  DroneState s;
  s.vel = {speed * std::cos(heading), speed * std::sin(heading), climb};
  s.yaw = heading;
  return s;
}

double airspeed(const DroneState& s, const Vec3& wind = {}) {
  const Vec3 v = s.vel - wind;
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// The command of a pilot holding a bank: a sideways acceleration that turns with the heading.
DroneCommand banked(const DroneState& s, double bank) {
  const double a = kG * std::tan(bank);
  const double heading = std::atan2(s.vel.y, s.vel.x);
  DroneCommand c;
  c.accel = {-a * std::sin(heading), a * std::cos(heading), 0.0};
  return c;
}

DroneState run(const FixedWingEnergyVehicle& v, DroneState s, const DroneCommand& c, double secs,
               const Vec3& wind = {}) {
  for (int i = 0; i < static_cast<int>(secs / kDt); ++i) {
    s = v.step_in_wind(s, c, kDt, wind);
  }
  return s;
}

// A pilot holding `bank` for `secs`: the command is recomputed from the heading every step.
DroneState run_banked(const FixedWingEnergyVehicle& v, DroneState s, double bank, double secs) {
  for (int i = 0; i < static_cast<int>(secs / kDt); ++i) {
    s = v.step(s, banked(s, bank), kDt);
  }
  return s;
}

}  // namespace

TEST(FixedWingEnergy, StallAndHoldSpeedsFollowFromTheWing) {
  const EnergyParams p;
  // sqrt(2 m g / (rho S CLmax)) for 12 kg, 0.9 m^2, 1.4: about 12.4 m/s.
  EXPECT_NEAR(p.stall_speed(kG), 12.35, 0.05);
  EXPECT_NEAR(p.hold_speed(kG), 1.15 * p.stall_speed(kG), 1e-12);
}

TEST(FixedWingEnergy, ThrustAtCruiseIsAFewTensOfNewtonsAndPowerIsAHundredsOfWatts) {
  const auto v = make();
  const double drag = v.drag(16.7, 1.0);
  EXPECT_GT(drag, 5.0);
  EXPECT_LT(drag, 20.0);
  // Power available at cruise is well above the power needed, so the aircraft can climb.
  EXPECT_GT(EnergyParams{}.max_power - drag * 16.7, 200.0);
}

TEST(FixedWingEnergy, LevelFlightHoldsSpeedHeadingAndAltitude) {
  const auto v = make();
  const DroneState s = run(v, flying(17.0), DroneCommand{}, 30.0);
  EXPECT_NEAR(airspeed(s), 17.0, 0.05);
  EXPECT_NEAR(s.vel.z, 0.0, 0.05);
  EXPECT_NEAR(s.pos.y, 0.0, 0.5);
  EXPECT_NEAR(s.pos.x, 17.0 * 30.0, 2.0);
}

TEST(FixedWingEnergy, ABankedTurnTurnsAtTheCoordinatedRateAndHoldsSpeed) {
  const auto v = make();
  const double bank = 0.4;
  DroneState s = flying(17.0);
  s = run_banked(v, s, bank, 1.0);  // let the roll settle
  const DroneState a = s;
  DroneState b = a;
  for (int i = 0; i < 50; ++i) {
    b = v.step(b, banked(b, bank), kDt);
  }
  const double rate = swarm::wrap_angle(b.yaw - a.yaw) / (50 * kDt);
  EXPECT_NEAR(rate, kG * std::tan(bank) / 17.0, 0.02);
  EXPECT_NEAR(airspeed(b), 17.0, 0.3);
}

TEST(FixedWingEnergy, RollFollowsWithALagNotAJump) {
  const auto v = make();
  DroneState s = flying(17.0);
  const DroneCommand c = banked(s, 0.5);
  s = v.step(s, c, kDt);
  // After one step the bank is at most the roll rate times dt, far below the 0.5 rad asked for.
  const double bank_after = std::atan2(17.0 * s.yaw_rate, kG);
  EXPECT_LE(std::abs(bank_after), EnergyParams{}.max_roll_rate * kDt + 1e-9);
  EXPECT_GT(std::abs(bank_after), 0.0);
}

TEST(FixedWingEnergy, ATurnCostsSpeedWhenThePropulsionIsAtItsLimit) {
  // At 22 m/s level flight takes about 260 W and a 0.6 rad turn about 300 W, so with 280 W the
  // thrust holds level flight but cannot make up the induced drag of the turn.
  FixedWingParams limits;
  EnergyParams energy;
  energy.max_power = 280.0;
  const FixedWingEnergyVehicle v(limits, energy);
  DroneState s = flying(22.0);
  const DroneState turned = run_banked(v, s, 0.6, 15.0);
  EXPECT_LT(airspeed(turned), 21.5);
  EXPECT_NEAR(airspeed(run(v, s, DroneCommand{}, 6.0)), 22.0, 0.2);
}

TEST(FixedWingEnergy, TheBankNeverExceedsWhatTheWingCanCarryAtTheCurrentSpeed) {
  // Just above the stall speed, with too little power to speed up, a hard bank command must be
  // cut: the bank stays below acos((stall speed / speed)^2), the bank at which the load factor
  // stalls the wing.
  EnergyParams energy;
  energy.max_power = 60.0;
  const FixedWingEnergyVehicle v(FixedWingParams{}, energy);
  const double stall = energy.stall_speed(kG);
  DroneState s = flying(stall * 1.04);
  double highest_excess = -1.0;
  for (int i = 0; i < 40; ++i) {
    s = v.step(s, banked(s, 0.6), kDt);
    const double speed = airspeed(s);
    const double bank = std::abs(std::atan2(speed * s.yaw_rate, kG));
    const double limit = std::acos(std::min(1.0, stall * stall / (speed * speed)));
    highest_excess = std::max(highest_excess, bank - limit);
  }
  EXPECT_LT(highest_excess, 0.03);
}

TEST(FixedWingEnergy, AClimbIsLimitedByTheThrustLeftAfterDrag) {
  const auto v = make();
  DroneState s = flying(17.0);
  DroneCommand up;
  up.accel = {0.0, 0.0, 2.0};
  double max_climb = 0.0;
  double min_speed = 100.0;
  for (int i = 0; i < 1500; ++i) {  // 30 s of asking for more climb than is possible
    s = v.step(s, up, kDt);
    max_climb = std::max(max_climb, s.vel.z);
    min_speed = std::min(min_speed, airspeed(s));
  }
  // The excess power at cruise supports a climb of a few meters per second, and the climb rate
  // limit is 3 m/s.
  EXPECT_GT(max_climb, 1.0);
  EXPECT_LE(max_climb, FixedWingParams{}.max_climb_rate + 1e-9);
  // The speed stays above the hold speed: the climb is cut before the aircraft stalls.
  EXPECT_GT(min_speed, EnergyParams{}.hold_speed(kG) - 0.3);
}

TEST(FixedWingEnergy, ClimbWithTooLittlePowerTradesSpeedAndThenStopsClimbing) {
  FixedWingParams limits;
  EnergyParams energy;
  energy.max_power = 200.0;  // barely more than the cruise drag takes
  const FixedWingEnergyVehicle v(limits, energy);
  DroneState s = flying(17.0);
  DroneCommand up;
  up.accel = {0.0, 0.0, 2.0};
  double min_speed = 100.0;
  for (int i = 0; i < 3000; ++i) {
    s = v.step(s, up, kDt);
    min_speed = std::min(min_speed, airspeed(s));
  }
  EXPECT_GT(min_speed, energy.hold_speed(kG) - 0.6);
  EXPECT_LT(s.vel.z, 1.0);
}

TEST(FixedWingEnergy, BelowTheStallSpeedTheNoseDropsAndSpeedReturns) {
  const auto v = make();
  DroneState s = flying(9.0);
  const DroneState after = run(v, s, DroneCommand{}, 0.5);
  EXPECT_LT(after.vel.z, -0.2);
  const DroneState later = run(v, after, DroneCommand{}, 10.0);
  EXPECT_GT(airspeed(later), EnergyParams{}.stall_speed(kG));
}

TEST(FixedWingEnergy, WindMovesTheGroundTrackAndNotTheAirspeed) {
  const auto v = make();
  const Vec3 wind{0.0, 5.0, 0.0};
  const DroneState still = run(v, flying(17.0), DroneCommand{}, 10.0);
  const DroneState windy = run(v, flying(17.0, 0.0), DroneCommand{}, 10.0, wind);
  // The state starts with ground velocity (17, 0); against a wind of 5 m/s toward +y the air
  // velocity is (17, -5), so the heading is not along +x, and the speed is 17.72 m/s.
  EXPECT_NEAR(airspeed(windy, wind), std::hypot(17.0, 5.0), 0.2);
  EXPECT_NE(windy.pos.y, still.pos.y);
}

TEST(FixedWingEnergy, ASteadyWindAddsToTheGroundVelocity) {
  const auto v = make();
  const Vec3 wind{-4.0, 3.0, 0.0};
  // Start in trim in the air mass: air velocity (17, 0), so ground velocity (13, 3).
  DroneState s = flying(17.0);
  s.vel = s.vel + wind;
  const DroneState after = run(v, s, DroneCommand{}, 20.0, wind);
  EXPECT_NEAR(after.vel.x, 13.0, 0.1);
  EXPECT_NEAR(after.vel.y, 3.0, 0.1);
  EXPECT_NEAR(after.pos.x, 13.0 * 20.0, 3.0);
  EXPECT_NEAR(after.pos.y, 3.0 * 20.0, 3.0);
}

TEST(FixedWingEnergy, AGustChangesTheAirspeed) {
  const auto v = make();
  // A sudden loss of headwind: ground velocity unchanged, wind gone, airspeed drops by 4 m/s.
  DroneState s = flying(17.0);
  s.vel = s.vel + Vec3{-4.0, 0.0, 0.0};  // flying in a 4 m/s headwind, ground speed 13
  const DroneState next = v.step_in_wind(s, DroneCommand{}, kDt, Vec3{});
  EXPECT_NEAR(airspeed(next), 13.0, 0.3);
}

TEST(FixedWingEnergy, SpeedNeverExceedsTheLimit) {
  const auto v = make();
  DroneState s = flying(24.0);
  DroneCommand fast;
  fast.accel = {5.0, 0.0, 0.0};
  const DroneState after = run(v, s, fast, 20.0);
  EXPECT_LE(airspeed(after), FixedWingParams{}.max_speed + 1e-9);
}

TEST(FixedWingEnergy, TheDefaultWindStepFliesAnyVehicleInTheAirMass) {
  const FixedWingVehicle simple{FixedWingParams{}};
  const Vec3 wind{0.0, 4.0, 0.0};
  DroneState s = flying(17.0);
  s.vel = s.vel + wind;
  DroneState still = flying(17.0);
  for (int i = 0; i < 500; ++i) {
    s = simple.step_in_wind(s, DroneCommand{}, kDt, wind);
    still = simple.step(still, DroneCommand{}, kDt);
  }
  EXPECT_NEAR(s.pos.x, still.pos.x, 1e-6);
  EXPECT_NEAR(s.pos.y, still.pos.y + 4.0 * 500 * kDt, 1e-6);
}

TEST(FixedWingEnergy, AQuadIgnoresTheWind) {
  const QuadVehicle quad{swarm::DroneParams{}};
  DroneState s;
  s.vel = {1.0, 0.0, 0.0};
  const DroneState a = quad.step(s, DroneCommand{}, kDt);
  const DroneState b = quad.step_in_wind(s, DroneCommand{}, kDt, Vec3{9.0, 9.0, 9.0});
  EXPECT_EQ(a.vel.x, b.vel.x);
  EXPECT_EQ(a.pos.y, b.pos.y);
}

TEST(FixedWingEnergy, RejectsBadInput) {
  const auto v = make();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(v.step(flying(17.0), DroneCommand{}, 0.0), std::invalid_argument);
  EXPECT_THROW(v.step_in_wind(flying(17.0), DroneCommand{}, kDt, Vec3{nan, 0.0, 0.0}),
               std::invalid_argument);
  DroneCommand bad;
  bad.accel = {nan, 0.0, 0.0};
  EXPECT_THROW(v.step(flying(17.0), bad, kDt), std::invalid_argument);
  EnergyParams p;
  p.stall_margin = 0.9;
  EXPECT_THROW(FixedWingEnergyVehicle(FixedWingParams{}, p).step(flying(17.0), DroneCommand{}, kDt),
               std::invalid_argument);
}

TEST(FixedWingEnergy, AVerticalAccelerationCommandIsFollowedNotSmoothedAway) {
  // A command of 1 m/s^2 upward, held for 2 s from level flight, gives a climb rate of about 1 to 2
  // m/s once the pitch lag is through: the command is read as a wish for acceleration, and the
  // lag must not eat it. (A first version took only a fiftieth of it.)
  const auto v = make();
  DroneCommand up;
  up.accel = {0.0, 0.0, 1.0};
  const DroneState after = run(v, flying(17.0), up, 2.0);
  EXPECT_GT(after.vel.z, 0.9);
  EXPECT_LT(after.vel.z, 2.5);
}
