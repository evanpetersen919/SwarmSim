#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/control/avoid.hpp"
#include "swarm/control/drone_brain.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/control/terrain_map.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/simulation.hpp"

using swarm::AvoidInput;
using swarm::AvoidParams;
using swarm::AvoidResult;
using swarm::ForwardRay;
using swarm::ForwardSample;
using swarm::Vec3;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

// A ray at `azimuth_deg` to the left of +x (positive left), with a return `range` meters along it
// at `up` meters above the aircraft, or no return when `range` is zero.
ForwardRay ray(double azimuth_deg, double range, double up) {
  ForwardRay r;
  const double az = azimuth_deg * kDeg;
  const double sin_e = range > 0.0 ? up / range : 0.0;
  const double cos_e = std::sqrt(1.0 - sin_e * sin_e);
  r.direction = {cos_e * std::cos(az), cos_e * std::sin(az), sin_e};
  r.hit = range > 0.0;
  r.range = range;
  r.sigma = 1.0;
  return r;
}

// Five columns, at +20, +10, 0, -10, -20 degrees, one row, from an aircraft at (0, 0, 100).
// `ranges` is the range of the return in each column, 0 for none, and `up` the height of its point
// above the aircraft.
ForwardSample fan(const std::vector<double>& ranges, const std::vector<double>& up) {
  ForwardSample s;
  s.available = true;
  s.columns = 5;
  const double azimuths[5] = {20.0, 10.0, 0.0, -10.0, -20.0};
  for (int c = 0; c < 5; ++c) {
    s.rays.push_back(ray(azimuths[c], ranges[static_cast<size_t>(c)], up[static_cast<size_t>(c)]));
  }
  return s;
}

AvoidInput input_for(const ForwardSample& sample) {
  AvoidInput in;
  in.sample = &sample;
  in.sample_position = {0.0, 0.0, 100.0};
  in.position = {0.0, 0.0, 100.0};
  in.velocity = {30.0, 0.0, 0.0};
  in.clearance = 60.0;
  in.climb_budget = 1.8;
  in.margin_sigmas = 0.0;
  in.min_speed = 12.0;
  return in;
}

const std::vector<double> kNoUp = {0, 0, 0, 0, 0};

}  // namespace

TEST(Avoid, NothingSeenOrOnlyWhatClimbingCanClearIsNotAThreat) {
  const AvoidParams params;
  const ForwardSample empty;
  EXPECT_FALSE(swarm::assess_avoidance(params, input_for(empty)).threatened);

  // Returns, all of them ground 80 m below the aircraft: nothing to climb.
  const ForwardSample low = fan({250, 250, 250, 250, 250}, {-80, -80, -80, -80, -80});
  EXPECT_FALSE(swarm::assess_avoidance(params, input_for(low)).threatened);

  // A rise that the climb budget can take in the time: 20 m needed over 20 s is 1 m/s, under 1.8.
  const ForwardSample gentle = fan({0, 0, 400, 0, 0}, {-40, -40, -40, -40, -40});
  EXPECT_FALSE(swarm::assess_avoidance(params, input_for(gentle)).threatened);

  // No ray hit at all.
  EXPECT_FALSE(swarm::assess_avoidance(params, input_for(fan({0, 0, 0, 0, 0}, kNoUp))).threatened);
}

TEST(Avoid, AReturnBeyondTheHorizonIsLeftToTheAltitudeFloor) {
  const AvoidParams params;  // 12 s horizon: 360 m at 30 m/s
  const ForwardSample far = fan({0, 0, 600, 0, 0}, kNoUp);
  EXPECT_FALSE(swarm::assess_avoidance(params, input_for(far)).threatened);
  const ForwardSample near = fan({0, 0, 300, 0, 0}, kNoUp);
  EXPECT_TRUE(swarm::assess_avoidance(params, input_for(near)).threatened);
}

TEST(Avoid, ABlockedColumnToTheSideIsNotOnTheWay) {
  const AvoidParams params;
  const ForwardSample side = fan({200, 0, 0, 0, 0}, kNoUp);  // only +20 blocked
  EXPECT_FALSE(swarm::assess_avoidance(params, input_for(side)).threatened);
}

TEST(Avoid, ABlockedHeadingTurnsTowardTheNearestFreeColumnAndTheRightOnATie) {
  const AvoidParams params;
  // Only the center is blocked: +10 and -10 are equally near, so it turns right (negative).
  const AvoidResult tie = swarm::assess_avoidance(params, input_for(fan({0, 0, 200, 0, 0}, kNoUp)));
  EXPECT_TRUE(tie.threatened);
  EXPECT_FALSE(tie.all_blocked);
  EXPECT_NEAR(tie.turn_angle, -10.0 * kDeg, 1e-9);
  EXPECT_LT(tie.turn_accel, 0.0);

  // The center and the right are blocked: the nearest free one is +10, to the left.
  const AvoidResult left =
      swarm::assess_avoidance(params, input_for(fan({0, 0, 200, 200, 200}, kNoUp)));
  EXPECT_TRUE(left.threatened);
  EXPECT_NEAR(left.turn_angle, 10.0 * kDeg, 1e-9);
  EXPECT_GT(left.turn_accel, 0.0);

  // The center and the left are blocked: it turns right, to -10.
  const AvoidResult right =
      swarm::assess_avoidance(params, input_for(fan({200, 200, 200, 0, 0}, kNoUp)));
  EXPECT_NEAR(right.turn_angle, -10.0 * kDeg, 1e-9);
  EXPECT_LT(right.turn_accel, 0.0);
}

TEST(Avoid, TheTurnIsLimitedToAFractionOfTheVehiclesSidewaysAcceleration) {
  AvoidParams params;
  params.max_lateral_accel = 6.85;
  params.bank_fraction = 0.9;
  // Only the +/-20 columns are free: the angle is 20 degrees and speed * angle / 1.5 s is 7.0, over
  // the limit of 0.9 * 6.85.
  const AvoidResult capped =
      swarm::assess_avoidance(params, input_for(fan({0, 200, 200, 200, 0}, kNoUp)));
  EXPECT_NEAR(std::abs(capped.turn_accel), 0.9 * 6.85, 1e-9);
  // A small angle gives a proportionally small acceleration.
  const AvoidResult small =
      swarm::assess_avoidance(params, input_for(fan({0, 0, 200, 0, 0}, kNoUp)));
  EXPECT_NEAR(std::abs(small.turn_accel), 30.0 * 10.0 * kDeg / params.turn_time_s, 1e-9);
}

TEST(Avoid, WhenEveryColumnIsBlockedItTurnsTowardTheOneWithTheMostTime) {
  const AvoidParams params;
  // The -20 column's return is the farthest, 330 m; the rest are 200 m.
  const AvoidResult all =
      swarm::assess_avoidance(params, input_for(fan({200, 200, 200, 200, 330}, kNoUp)));
  EXPECT_TRUE(all.threatened);
  EXPECT_TRUE(all.all_blocked);
  EXPECT_TRUE(all.slow);
  EXPECT_NEAR(all.turn_angle, -20.0 * kDeg, 1e-9);
  EXPECT_NEAR(all.turn_accel, -0.9 * params.max_lateral_accel, 1e-9);
}

TEST(Avoid, TheSpeedComesDownOnlyWhenThereIsNoTimeForTheTurn) {
  const AvoidParams params;
  // 20 degrees to the nearest free column: about 1.7 s of turn at the capped acceleration. A threat
  // 330 m ahead is 11 s away; one 40 m ahead is 1.3 s away.
  const AvoidResult far =
      swarm::assess_avoidance(params, input_for(fan({0, 330, 330, 330, 0}, kNoUp)));
  EXPECT_FALSE(far.slow);
  const AvoidResult near =
      swarm::assess_avoidance(params, input_for(fan({0, 40, 40, 40, 0}, kNoUp)));
  EXPECT_TRUE(near.threatened);
  EXPECT_TRUE(near.slow);
  EXPECT_NEAR(near.time_to_threat, 40.0 / 30.0, 1e-9);
}

TEST(Avoid, TheRangeErrorCountsInTheSafeDirection) {
  AvoidParams params;
  // A return that is just clearable with the measured range, 48.5 m below the aircraft at 200 m,
  // becomes unclearable when the range is taken two sigma nearer along a falling ray: the point is
  // then higher.
  ForwardSample s = fan({0, 0, 200, 0, 0}, {0, 0, -48.5, 0, 0});
  AvoidInput in = input_for(s);
  s.rays[2].sigma = 1.0;
  in.margin_sigmas = 0.0;
  const bool plain = swarm::assess_avoidance(params, in).threatened;
  s.rays[2].sigma = 30.0;
  in.margin_sigmas = 2.0;
  const bool careful = swarm::assess_avoidance(params, in).threatened;
  EXPECT_FALSE(plain);
  EXPECT_TRUE(careful);
}

TEST(Avoid, ParametersAndInputsAreChecked) {
  AvoidParams bad;
  bad.horizon_s = 0.0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = AvoidParams{};
  bad.bank_fraction = 0.0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = AvoidParams{};
  bad.bank_fraction = 1.5;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = AvoidParams{};
  bad.max_lateral_accel = -1.0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = AvoidParams{};
  bad.max_vertical_accel = 0.0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);

  const ForwardSample s = fan({0, 0, 200, 0, 0}, kNoUp);
  AvoidInput in = input_for(s);
  in.velocity = {std::nan(""), 0.0, 0.0};
  EXPECT_THROW(swarm::assess_avoidance(AvoidParams{}, in), std::invalid_argument);
  // An aircraft with no horizontal velocity has no heading to avoid along.
  in = input_for(s);
  in.velocity = {0.0, 0.0, 5.0};
  EXPECT_FALSE(swarm::assess_avoidance(AvoidParams{}, in).threatened);
}

namespace {

using swarm::BrainConfig;
using swarm::BrainInput;
using swarm::DroneBrain;

// One aircraft heading +y at 16.7 m/s over a map that says the ground is flat at 0 m, at 150 m: as
// in the forward sensor tests, only a sensor can ask for anything.
BrainConfig avoiding_brain(bool avoid) {
  BrainConfig c;
  c.id = 0;
  c.num_drones = 1;
  c.dt = 0.02;
  c.offsets = swarm::make_formation(swarm::FormationShape::kLine, 1, 15.0);
  c.mission.waypoints = {{0, 0, 150}};
  c.pose.dt = c.dt;
  c.predictor_config.dt = c.dt;
  c.launch.position = {300.0, 0.0, 150.0};
  c.launch.velocity = {0.0, 16.7, 0.0};
  c.launch.position_sigma = 0.1;
  c.launch.velocity_sigma = 0.1;
  c.launch.bias_sigma = 0.05;
  c.guidance = swarm::Guidance::kOrbit;
  c.orbit.center = {0.0, 0.0, 150.0};
  c.orbit.radius = 300.0;
  c.orbit.speed = 16.7;
  c.orbit.min_speed = 12.0;
  c.orbit.max_speed = 25.0;
  c.terrain = std::make_shared<const swarm::TerrainMap>(10, 10, -2000.0, -2000.0, 400.0,
                                                        std::vector<double>(11 * 11, 0.0));
  c.terrain_guidance.enabled = true;
  c.terrain_guidance.clearance = 60.0;
  c.use_forward_sensor = true;
  c.forward_valid_ticks = 10;
  c.avoid.enabled = avoid;
  c.avoid.hold_s = 0.2;  // 10 ticks
  return c;
}

// A fan of three columns, +10, 0, -10 degrees to the left of the heading (+y), with the center
// blocked by ground at the aircraft's own height 150 m ahead.
ForwardSample blocked_center() {
  ForwardSample s;
  s.available = true;
  s.columns = 3;
  const double azimuths[3] = {10.0, 0.0, -10.0};
  for (int c = 0; c < 3; ++c) {
    ForwardRay r;
    const double az = azimuths[c] * kDeg;
    // Heading +y, left is -x.
    r.direction = {-std::sin(az), std::cos(az), 0.0};
    r.hit = c == 1;
    r.range = c == 1 ? 150.0 : 0.0;
    r.sigma = 1.0;
    s.rays.push_back(r);
  }
  return s;
}

Vec3 command(DroneBrain& brain, std::uint64_t tick, const ForwardSample& reading) {
  BrainInput in;
  in.imu.accel = {0.0, 0.0, 9.81};
  in.fix.available = true;
  in.fix.position = {300.0, 0.0, 150.0};
  in.fix.sigma = 0.2;
  in.forward = reading;
  return brain.step(in, tick).command.accel;
}

}  // namespace

namespace {

// The sideways command (x, heading +y) five ticks into a blocked heading whose threat is 9 s away.
double turn_command(double urgent_s) {
  BrainConfig c = avoiding_brain(true);
  c.avoid.urgent_s = urgent_s;
  DroneBrain brain(c);
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    command(brain, t, none);
  }
  return command(brain, 5, blocked_center()).x;
}

}  // namespace

TEST(AvoidBrain, LittleTimeUsesTheWholeBankAndMoreTimeLeavesTheTurnAlone) {
  const double proportional = turn_command(0.0);
  // The threat is 150 m / 16.7 m/s = 9 s away: not urgent at 5 s, urgent at 12 s.
  EXPECT_DOUBLE_EQ(turn_command(5.0), proportional);
  const double urgent = turn_command(12.0);
  EXPECT_GT(urgent, proportional + 2.0);
}

TEST(AvoidBrain, ABlockedHeadingTurnsRightWhenTheLeftAndRightAreEquallyFree) {
  DroneBrain with(avoiding_brain(true));
  DroneBrain without(avoiding_brain(false));
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    command(with, t, none);
    command(without, t, none);
  }
  const Vec3 turning = command(with, 5, blocked_center());
  const Vec3 flying_on = command(without, 5, blocked_center());
  // Heading +y: right is +x. The aircraft is on its circle, so without avoidance it turns left (-x)
  // along the orbit; with avoidance it turns right.
  EXPECT_GT(turning.x, flying_on.x + 1.5);
  EXPECT_GT(turning.x, 0.0);
  // The climb that the blocked heading also asks for is limited to what the vehicle can do, so it
  // does not take the acceleration limit from the turn.
  EXPECT_LE(turning.z, 2.0 + 1e-9);
}

TEST(AvoidBrain, TheHeadingIsHeldAfterTheThreatUntilTheHoldRunsOutThenTheOrbitReturns) {
  BrainConfig c = avoiding_brain(true);
  DroneBrain brain(c);
  DroneBrain orbit_only(avoiding_brain(false));
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    command(brain, t, none);
    command(orbit_only, t, none);
  }
  command(brain, 5, blocked_center());
  command(orbit_only, 5, blocked_center());
  // The reading is valid for 10 ticks, so it is a threat up to tick 15.
  for (std::uint64_t t = 6; t <= 15; ++t) {
    EXPECT_GT(command(brain, t, none).x, 0.0) << "tick " << t;
    command(orbit_only, t, none);
  }
  // From tick 16 there is no threat, and the hold (10 ticks from tick 15) keeps the heading: no
  // horizontal acceleration at all.
  for (std::uint64_t t = 16; t <= 25; ++t) {
    const Vec3 held = command(brain, t, none);
    command(orbit_only, t, none);
    EXPECT_DOUBLE_EQ(held.x, 0.0) << "tick " << t;
    EXPECT_DOUBLE_EQ(held.y, 0.0) << "tick " << t;
  }
  // After it the orbit guidance has the aircraft again and asks for the sideways acceleration of a
  // circle.
  const Vec3 back = command(brain, 26, none);
  EXPECT_GT(std::abs(back.x), 0.1);
}

TEST(AvoidBrain, WithoutTheOptionNothingChanges) {
  DroneBrain with(avoiding_brain(false));
  BrainConfig same = avoiding_brain(false);
  DroneBrain other(same);
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 12; ++t) {
    const Vec3 a = command(with, t, t == 5 ? blocked_center() : none);
    const Vec3 b = command(other, t, t == 5 ? blocked_center() : none);
    EXPECT_EQ(a, b);
  }
}

TEST(AvoidBrain, TheOptionNeedsTheForwardSensor) {
  BrainConfig c = avoiding_brain(true);
  c.use_forward_sensor = false;
  EXPECT_THROW(DroneBrain{c}, std::invalid_argument);
}

namespace {

std::string avoid_scenario(int version, const std::string& vehicle, const std::string& sensors,
                           const std::string& avoid) {
  return "version: " + std::to_string(version) +
         "\nseed: 1\nduration_s: 5\n"
         "scene: {type: field, size_m: [2000, 2000], cell_m: 20, obstacles: {density_per_m2: "
         "0.0}}\n"
         "swarm: {drones: 2}\n"
         "mission:\n  waypoints:\n    - [0, 0, 150]\n  speed_mps: 17\n" +
         vehicle + (sensors.empty() ? std::string() : "sensors:\n" + sensors) +
         "control:\n  orbit: {radius_m: 300, radius_step_m: 12, altitude_step_m: 10}\n" + avoid;
}

std::string first_error(const swarm::LoadResult& r) {
  return r.errors.empty() ? std::string() : swarm::to_string(r.errors.front());
}

}  // namespace

TEST(AvoidScenario, TheKeysAreReadRoundTripAndWrittenOnlyWhenUsed) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string forward = "  forward: {period_ticks: 5}\n";
  const swarm::LoadResult r = swarm::load_scenario(
      avoid_scenario(3, fw, forward, "  avoid: {horizon_s: 8, hold_s: 2.5, bank_fraction: 0.7}\n"));
  ASSERT_TRUE(r.ok()) << first_error(r);
  EXPECT_TRUE(r.scenario.avoid.enabled);
  EXPECT_DOUBLE_EQ(r.scenario.avoid.horizon_s, 8.0);
  EXPECT_DOUBLE_EQ(r.scenario.avoid.hold_s, 2.5);
  EXPECT_DOUBLE_EQ(r.scenario.avoid.bank_fraction, 0.7);
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("  avoid:"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << first_error(again);
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  // An empty section turns it on with the defaults.
  const swarm::LoadResult defaults =
      swarm::load_scenario(avoid_scenario(3, fw, forward, "  avoid: {}\n"));
  ASSERT_TRUE(defaults.ok()) << first_error(defaults);
  EXPECT_TRUE(defaults.scenario.avoid.enabled);
  EXPECT_DOUBLE_EQ(defaults.scenario.avoid.horizon_s, 12.0);

  const swarm::LoadResult plain = swarm::load_scenario(avoid_scenario(3, fw, forward, ""));
  ASSERT_TRUE(plain.ok());
  EXPECT_FALSE(plain.scenario.avoid.enabled);
  EXPECT_EQ(swarm::canonical_text(plain.scenario).find("avoid"), std::string::npos);
}

TEST(AvoidScenario, ItNeedsTheForwardSensorVersionThreeAFixedWingVehicleAndValidValues) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string forward = "  forward: {period_ticks: 5}\n";
  EXPECT_FALSE(swarm::load_scenario(avoid_scenario(3, fw, "", "  avoid: {}\n")).ok());
  EXPECT_FALSE(swarm::load_scenario(avoid_scenario(2, fw, forward, "  avoid: {}\n")).ok());
  EXPECT_FALSE(
      swarm::load_scenario(avoid_scenario(3, "vehicle: {type: quad}\n", forward, "  avoid: {}\n"))
          .ok());
  for (const char* bad : {"horizon_s: 0", "hold_s: 0", "bank_fraction: 0", "bank_fraction: 1.5",
                          "urgent_s: -1", "unknown_key: 1"}) {
    EXPECT_FALSE(swarm::load_scenario(
                     avoid_scenario(3, fw, forward, std::string("  avoid: {") + bad + "}\n"))
                     .ok())
        << bad;
  }
  EXPECT_TRUE(
      swarm::load_scenario(avoid_scenario(3, fw, forward, "  avoid: {bank_fraction: 1.0}\n")).ok());
}

namespace {

// Six aircraft at 70 mph on a 300 m orbit at 100 m, with no map, over hills of 400 m, with the
// forward sensor, with or without avoidance. Returns how many reached the ground over `seeds`.
int aircraft_lost(bool avoid, std::uint64_t seeds) {
  int lost = 0;
  for (std::uint64_t seed = 1; seed <= seeds; ++seed) {
    const std::string text =
        "version: 3\nseed: " + std::to_string(seed) +
        "\nduration_s: 60\n"
        "scene: {type: hills, size_m: [2000, 2000], cell_m: 10, amplitude_m: 400,\n"
        "        obstacles: {density_per_m2: 0.0}}\n"
        "swarm: {drones: 6}\n"
        "mission:\n  waypoints:\n    - [0, 0, 100]\n  speed_mps: 31.3\n"
        "vehicle:\n  type: fixed_wing\n  max_speed_mps: 51\n  dynamics: energy\n"
        "  energy: {mass_kg: 28.5, wing_area_m2: 1.0, wingspan_m: 3.0, max_thrust_n: 90, "
        "max_power_w: 3500}\n"
        "environment: {ground_contact_crashes: true}\n"
        "weather: {visibility_m: 20000}\n"
        "sensors:\n  forward: {period_ticks: 5}\n"
        "control:\n  orbit: {radius_m: 300, radius_step_m: 0, altitude_step_m: 8, "
        "safe_distance_m: 20}\n" +
        std::string(avoid ? "  avoid: {}\n" : "");
    const swarm::LoadResult r = swarm::load_scenario(text);
    EXPECT_TRUE(r.ok()) << first_error(r);
    const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
    const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(1);
    for (std::uint64_t tick = 0; tick < r.scenario.duration_ticks; ++tick) {
      sim->step();
    }
    for (std::size_t i = 0; i < r.scenario.drones; ++i) {
      lost += dynamic_cast<const swarm::DroneAgent&>(sim->agent(i)).crashed() ? 1 : 0;
    }
  }
  return lost;
}

}  // namespace

TEST(AvoidFlight, OnHillsThatClimbingCannotClearTurningKeepsMoreAircraftOffTheGround) {
  const int climbing_only = aircraft_lost(false, 3);
  const int turning = aircraft_lost(true, 3);
  EXPECT_GT(climbing_only, 0);
  EXPECT_LT(turning, climbing_only);
}

TEST(AvoidKinematics, TheReachableClimbFollowsTheRampAndThenTheLimit) {
  // Limit 3 m/s, acceleration 2 m/s^2: from rest the ramp takes 1.5 s and gains 2.25 m.
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(0.0, 3.0, 2.0, 1.0), 1.0);   // a t^2 / 2
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(0.0, 3.0, 2.0, 1.5), 2.25);  // the end of the ramp
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(0.0, 3.0, 2.0, 3.0), 6.75);  // 3 * 3 - 9 / 4
  // Already at the limit: no ramp.
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(3.0, 3.0, 2.0, 2.0), 6.0);
  // Above the limit counts as the limit.
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(5.0, 3.0, 2.0, 2.0), 6.0);
  // Descending at 1 m/s: 2 s of ramp that gains 2 m, then the limit.
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(-1.0, 3.0, 2.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(-1.0, 3.0, 2.0, 4.0), 8.0);  // 12 - 16 / 4
  EXPECT_DOUBLE_EQ(swarm::reachable_climb(0.0, 3.0, 2.0, 0.0), 0.0);
  EXPECT_THROW(swarm::reachable_climb(0.0, 3.0, 2.0, -1.0), std::invalid_argument);
  EXPECT_THROW(swarm::reachable_climb(0.0, 0.0, 2.0, 1.0), std::invalid_argument);
  EXPECT_THROW(swarm::reachable_climb(0.0, 3.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(swarm::reachable_climb(std::nan(""), 3.0, 2.0, 1.0), std::invalid_argument);
}

TEST(AvoidKinematics, AReturnJustWithinTheRampIsClearableOnlyUnderTheKinematicJudgment) {
  // 3.68 m to gain in about 2 s: the budget (1.8 m/s) allows 3.59 m, the vehicle (3 m/s limit,
  // 2 m/s^2) can gain 3.74 m.
  const ForwardSample s = fan({0, 0, 60, 0, 0}, {0, 0, 3.68, 0, 0});
  AvoidInput in = input_for(s);
  in.clearance = 0.0;
  in.climb_limit = 3.0;
  AvoidParams budget;
  AvoidParams kinematic;
  kinematic.feasibility = swarm::AvoidFeasibility::kKinematic;
  EXPECT_TRUE(swarm::assess_avoidance(budget, in).threatened);
  EXPECT_FALSE(swarm::assess_avoidance(kinematic, in).threatened);
  // A climb already under way helps: 5.5 m in the same time is clear at 3 m/s and not from level.
  const ForwardSample more = fan({0, 0, 60, 0, 0}, {0, 0, 5.5, 0, 0});
  AvoidInput climbing = input_for(more);
  climbing.clearance = 0.0;
  climbing.velocity = {30.0, 0.0, 3.0};
  EXPECT_FALSE(swarm::assess_avoidance(kinematic, climbing).threatened);
  AvoidInput resting = climbing;
  resting.velocity = {30.0, 0.0, 0.0};
  EXPECT_TRUE(swarm::assess_avoidance(kinematic, resting).threatened);
}

TEST(AvoidClearance, ASmallerClearanceMakesAReturnClearable) {
  // Ground at the aircraft's own height 200 m ahead: 60 m to climb for the planning clearance,
  // 10 m for a clearance of 10, in 6.7 s, which the climb budget (12 m) allows.
  const ForwardSample s = fan({0, 0, 200, 0, 0}, kNoUp);
  AvoidParams planning;
  AvoidParams small;
  small.clearance_m = 10.0;
  EXPECT_TRUE(swarm::assess_avoidance(planning, input_for(s)).threatened);
  EXPECT_FALSE(swarm::assess_avoidance(small, input_for(s)).threatened);
  AvoidParams negative;
  negative.clearance_m = -1.0;
  EXPECT_THROW(swarm::validate(negative), std::invalid_argument);
}

TEST(AvoidRejoin, TheFreeColumnNearestTheWayTheGuidanceIsTurningIsChosen) {
  AvoidParams plain;
  AvoidParams rejoin;
  rejoin.rejoin = true;
  // Heading +x. Only the center column is blocked.
  const ForwardSample s = fan({0, 0, 200, 0, 0}, kNoUp);

  // Guidance turning left (+y): with rejoin the tie goes left, as against the right of ADR 0014.
  AvoidInput left = input_for(s);
  left.guidance_accel = {0.0, 5.0, 0.0};
  EXPECT_NEAR(swarm::assess_avoidance(plain, left).turn_angle, -10.0 * kDeg, 1e-9);
  EXPECT_NEAR(swarm::assess_avoidance(rejoin, left).turn_angle, 10.0 * kDeg, 1e-9);
  EXPECT_GT(swarm::assess_avoidance(rejoin, left).turn_accel, 0.0);

  // Guidance turning right: the right, with or without rejoin.
  AvoidInput right = input_for(s);
  right.guidance_accel = {0.0, -5.0, 0.0};
  EXPECT_NEAR(swarm::assess_avoidance(rejoin, right).turn_angle, -10.0 * kDeg, 1e-9);

  // Guidance asking for a strong turn left: after 1.5 s the velocity is at about 31 degrees, so
  // the nearest free column is +20, not the +10 of the tie.
  AvoidInput strong = input_for(s);
  strong.guidance_accel = {0.0, 12.0, 0.0};
  EXPECT_NEAR(swarm::assess_avoidance(rejoin, strong).turn_angle, 20.0 * kDeg, 1e-9);

  // A blocked preferred column is not chosen: with +20 and +10 blocked as well, the nearest free
  // one is -10.
  const ForwardSample two_left = fan({200, 200, 200, 0, 0}, kNoUp);
  AvoidInput blocked_left = input_for(two_left);
  blocked_left.guidance_accel = {0.0, 12.0, 0.0};
  EXPECT_NEAR(swarm::assess_avoidance(rejoin, blocked_left).turn_angle, -10.0 * kDeg, 1e-9);

  // With no guidance acceleration, or none sideways, the rule of ADR 0014 applies.
  AvoidInput none = input_for(s);
  EXPECT_NEAR(swarm::assess_avoidance(rejoin, none).turn_angle, -10.0 * kDeg, 1e-9);
  AvoidInput along = input_for(s);
  along.guidance_accel = {3.0, 0.0, 0.0};
  EXPECT_NEAR(swarm::assess_avoidance(rejoin, along).turn_angle, -10.0 * kDeg, 1e-9);

  // The trigger is unchanged: a blocked column to the side is still not on the way.
  const ForwardSample side = fan({200, 0, 0, 0, 0}, kNoUp);
  AvoidInput sideways = input_for(side);
  sideways.guidance_accel = {0.0, 5.0, 0.0};
  EXPECT_FALSE(swarm::assess_avoidance(rejoin, sideways).threatened);
}

namespace {

BrainConfig rejoining_brain(bool rejoin) {
  BrainConfig c = avoiding_brain(true);
  c.avoid.rejoin = rejoin;
  return c;
}

}  // namespace

TEST(AvoidBrain, OnItsCircleTheRejoinTurnIsTowardTheCenterAndTheOldTieIsAwayFromIt) {
  DroneBrain plain(rejoining_brain(false));
  DroneBrain rejoin(rejoining_brain(true));
  const ForwardSample none;
  for (std::uint64_t t = 0; t < 5; ++t) {
    command(plain, t, none);
    command(rejoin, t, none);
  }
  // At (300, 0) heading +y on a counterclockwise circle about the origin, the center is to the left
  // (-x) and the guidance's own acceleration points there. Without rejoin the tie turns right
  // (+x), away from the center; with it the aircraft turns left, toward it.
  const Vec3 away = command(plain, 5, blocked_center());
  const Vec3 toward = command(rejoin, 5, blocked_center());
  EXPECT_GT(away.x, 0.5);
  EXPECT_LT(toward.x, -0.5);
}

TEST(AvoidScenario, TheRejoinFeasibilityAndClearanceKeysAreReadAndWrittenOnlyWhenSet) {
  const std::string fw = "vehicle: {type: fixed_wing}\n";
  const std::string forward = "  forward: {period_ticks: 5}\n";
  const swarm::LoadResult r = swarm::load_scenario(avoid_scenario(
      3, fw, forward,
      "  avoid: {rejoin: true, feasibility: kinematic, clearance_m: 30, urgent_s: 6}\n"));
  ASSERT_TRUE(r.ok()) << first_error(r);
  EXPECT_DOUBLE_EQ(r.scenario.avoid.urgent_s, 6.0);
  EXPECT_TRUE(r.scenario.avoid.rejoin);
  EXPECT_EQ(r.scenario.avoid.feasibility, swarm::AvoidFeasibility::kKinematic);
  ASSERT_TRUE(r.scenario.avoid.clearance_m.has_value());
  EXPECT_DOUBLE_EQ(*r.scenario.avoid.clearance_m, 30.0);
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("rejoin: true"), std::string::npos);
  EXPECT_NE(text.find("feasibility: kinematic"), std::string::npos);
  EXPECT_NE(text.find("clearance_m: 30"), std::string::npos);
  EXPECT_NE(text.find("urgent_s: 6"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << first_error(again);
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  // Set to their defaults they leave no trace, so the text is that of a scenario that sets none.
  const swarm::LoadResult defaults = swarm::load_scenario(
      avoid_scenario(3, fw, forward, "  avoid: {rejoin: false, feasibility: budget}\n"));
  ASSERT_TRUE(defaults.ok()) << first_error(defaults);
  const swarm::LoadResult bare =
      swarm::load_scenario(avoid_scenario(3, fw, forward, "  avoid: {}\n"));
  ASSERT_TRUE(bare.ok());
  EXPECT_EQ(swarm::canonical_text(defaults.scenario), swarm::canonical_text(bare.scenario));
  EXPECT_FALSE(bare.scenario.avoid.rejoin);
  EXPECT_EQ(bare.scenario.avoid.feasibility, swarm::AvoidFeasibility::kBudget);
  EXPECT_FALSE(bare.scenario.avoid.clearance_m.has_value());
  EXPECT_DOUBLE_EQ(bare.scenario.avoid.urgent_s, 0.0);
  EXPECT_EQ(swarm::canonical_text(bare.scenario).find("urgent_s"), std::string::npos);

  for (const char* bad : {"feasibility: fast", "clearance_m: -1", "rejoin: maybe"}) {
    EXPECT_FALSE(swarm::load_scenario(
                     avoid_scenario(3, fw, forward, std::string("  avoid: {") + bad + "}\n"))
                     .ok())
        << bad;
  }
  EXPECT_TRUE(
      swarm::load_scenario(avoid_scenario(3, fw, forward, "  avoid: {clearance_m: 0}\n")).ok());
}
