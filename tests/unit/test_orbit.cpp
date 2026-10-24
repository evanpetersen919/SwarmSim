#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/control/orbit.hpp"
#include "swarm/fixed_wing.hpp"

using swarm::DroneCommand;
using swarm::DroneState;
using swarm::FixedWingParams;
using swarm::FixedWingVehicle;
using swarm::kPi;
using swarm::OrbitNeighbor;
using swarm::OrbitParams;
using swarm::OrbitReference;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.02;

OrbitParams params() {
  OrbitParams p;
  p.center = {0.0, 0.0, 100.0};
  const FixedWingParams wing;
  p.min_speed = wing.min_speed;
  p.max_speed = wing.max_speed;
  return p;
}

// A swarm flown with perfect knowledge of every state, no messages, no estimation.
struct Swarm {
  OrbitParams orbit = params();
  FixedWingVehicle vehicle{FixedWingParams{}};
  std::vector<DroneState> state;
  double time = 0.0;
  bool separation = true;
  double min_speed_seen = 1e9;
  double min_separation_seen = 1e9;

  void step() {
    const std::uint32_t n = static_cast<std::uint32_t>(state.size());
    std::vector<DroneState> next = state;
    for (std::uint32_t i = 0; i < n; ++i) {
      const OrbitReference ref = swarm::orbit_reference(i, n, time, orbit);
      const double pe = swarm::orbit_phase_error(state[i].pos, orbit.center, ref.angle);
      Vec3 a = swarm::orbit_guidance(state[i].pos, state[i].vel, orbit.center, ref.radius,
                                     ref.speed, ref.position.z, pe, orbit, 1.0);
      if (separation) {
        std::vector<OrbitNeighbor> others;
        for (std::uint32_t j = 0; j < n; ++j) {
          if (j != i) {
            others.push_back({j, state[j].pos, state[j].vel, 0.0});
          }
        }
        const Vec3 push =
            swarm::orbit_separation_accel(i, state[i].pos, state[i].vel, others, orbit, 1.0);
        a = swarm::combine_accel(a, push, orbit.max_accel);
      }
      DroneCommand c;
      c.accel = a;
      next[i] = vehicle.step(state[i], c, kDt);
    }
    state = next;
    time += kDt;
    for (std::uint32_t i = 0; i < n; ++i) {
      min_speed_seen = std::min(min_speed_seen, std::hypot(state[i].vel.x, state[i].vel.y));
      for (std::uint32_t j = i + 1; j < n; ++j) {
        min_separation_seen =
            std::min(min_separation_seen, swarm::distance(state[i].pos, state[j].pos));
      }
    }
  }

  void run(double seconds) {
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
      step();
    }
  }
};

DroneState flying(const Vec3& pos, double speed, double heading) {
  DroneState s;
  s.pos = pos;
  s.vel = {speed * std::cos(heading), speed * std::sin(heading), 0.0};
  s.yaw = heading;
  return s;
}

}  // namespace

TEST(Orbit, ParametersAreValidated) {
  EXPECT_NO_THROW(swarm::validate(params()));
  OrbitParams p = params();
  p.speed = p.max_speed;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = params();
  p.radius = 0.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = params();
  p.altitude_step = -1.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = params();
  p.separation.max_margin = 1.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
}

TEST(Orbit, TheReferenceIsEvenlySpacedOnAlternatingRadiiAndStackedInAltitude) {
  const OrbitParams p = params();
  const std::uint32_t n = 6;
  const double omega = p.speed / p.radius;
  for (std::uint32_t i = 0; i < n; ++i) {
    const OrbitReference ref = swarm::orbit_reference(i, n, 0.0, p);
    const double radius = p.radius + (i % 2 == 0 ? 0.0 : p.radius_step);
    EXPECT_NEAR(ref.radius, radius, 1e-12);
    EXPECT_NEAR(std::hypot(ref.position.x, ref.position.y), radius, 1e-9);
    EXPECT_NEAR(ref.position.z, 100.0 + i * p.altitude_step, 1e-12);
    EXPECT_NEAR(std::hypot(ref.velocity.x, ref.velocity.y), omega * radius, 1e-9);
    // The velocity is tangent, counterclockwise.
    EXPECT_NEAR(ref.velocity.x * ref.position.x + ref.velocity.y * ref.position.y, 0.0, 1e-6);
    const double cross = ref.position.x * ref.velocity.y - ref.position.y * ref.velocity.x;
    EXPECT_GT(cross, 0.0);
  }
  // The angular spacing is 2 pi / n and every drone advances at omega.
  const OrbitReference a = swarm::orbit_reference(0, n, 10.0, p);
  const OrbitReference b = swarm::orbit_reference(1, n, 10.0, p);
  double spacing = b.angle - a.angle;
  if (spacing < 0.0) {
    spacing += 2.0 * kPi;
  }
  EXPECT_NEAR(spacing, 2.0 * kPi / n, 1e-9);
  const OrbitReference a_later = swarm::orbit_reference(0, n, 11.0, p);
  double advance = a_later.angle - a.angle;
  if (advance < 0.0) {
    advance += 2.0 * kPi;
  }
  EXPECT_NEAR(advance, omega, 1e-9);
}

TEST(Orbit, BadReferenceArgumentsAreRejected) {
  EXPECT_THROW(swarm::orbit_reference(3, 3, 0.0, params()), std::invalid_argument);
  EXPECT_THROW(swarm::orbit_reference(0, 0, 0.0, params()), std::invalid_argument);
  EXPECT_THROW(swarm::orbit_reference(0, 2, std::numeric_limits<double>::infinity(), params()),
               std::invalid_argument);
}

TEST(Orbit, ASteadyCircleNeedsNoCorrection) {
  const OrbitParams p = params();
  const OrbitReference ref = swarm::orbit_reference(0, 4, 0.0, p);
  const Vec3 a = swarm::orbit_guidance(ref.position, ref.velocity, p.center, ref.radius, ref.speed,
                                       ref.position.z, 0.0, p, 1.0);
  // Only the centripetal feedforward remains: V^2 / R toward the center.
  EXPECT_NEAR(std::hypot(a.x, a.y), ref.speed * ref.speed / ref.radius, 1e-9);
  EXPECT_NEAR(a.x * ref.position.x + a.y * ref.position.y, -ref.speed * ref.speed,
              1e-6);  // along -e_r times R
  EXPECT_NEAR(a.z, 0.0, 1e-12);
}

TEST(Orbit, TheCatchUpCapLimitsOnlyTheSpeedAskedForAboveTheCommandedOne) {
  OrbitParams p = params();
  p.speed = 30.0;
  p.min_speed = 15.0;
  p.max_speed = 50.0;
  p.radius = 300.0;
  // On the +x axis, moving counterclockwise (+y) at the commanded speed: the tangential
  // acceleration is velocity_gain * (desired speed - speed).
  const Vec3 position{300.0, 0.0, 100.0};
  const Vec3 velocity{0.0, 30.0, 0.0};
  const auto tangential = [&](const OrbitParams& q, double phase_error) {
    return swarm::orbit_guidance(position, velocity, p.center, 300.0, 30.0, 100.0, phase_error, q,
                                 1.0)
        .y;
  };
  // Behind by 1 rad asks for 0.05 * 300 * 1 = 15 m/s more, as before without a cap.
  EXPECT_NEAR(tangential(p, 1.0), 15.0, 1e-9);
  OrbitParams capped = p;
  capped.max_catchup = 3.0;
  EXPECT_NEAR(tangential(capped, 1.0), 3.0, 1e-9);
  // Ahead of the slot, slowing down is not capped.
  EXPECT_NEAR(tangential(capped, -0.5), tangential(p, -0.5), 1e-12);
  // A cap above what is asked changes nothing, and a negative one is refused.
  capped.max_catchup = 20.0;
  EXPECT_NEAR(tangential(capped, 1.0), 15.0, 1e-9);
  capped.max_catchup = -1.0;
  EXPECT_THROW(
      swarm::orbit_guidance(position, velocity, p.center, 300.0, 30.0, 100.0, 0.0, capped, 1.0),
      std::invalid_argument);
}

TEST(Orbit, OneAircraftFromSeveralStartsConvergesToItsCircleAndPhase) {
  struct Start {
    Vec3 pos;
    double heading;
  };
  const std::vector<Start> starts = {{{300.0, 0.0, 100.0}, kPi / 2.0},
                                     {{-250.0, 50.0, 140.0}, 0.0},
                                     {{0.0, -220.0, 90.0}, kPi},
                                     {{120.0, 80.0, 100.0}, -kPi / 4.0}};
  for (const Start& start : starts) {
    Swarm s;
    s.separation = false;
    s.state = {flying(start.pos, 17.0, start.heading)};
    s.run(240.0);
    const OrbitReference ref = swarm::orbit_reference(0, 1, s.time, s.orbit);
    const double r = std::hypot(s.state[0].pos.x, s.state[0].pos.y);
    EXPECT_NEAR(r, ref.radius, 3.0) << start.pos.x << "," << start.pos.y;
    EXPECT_NEAR(s.state[0].pos.z, 100.0, 1.0);
    EXPECT_LT(std::abs(swarm::orbit_phase_error(s.state[0].pos, s.orbit.center, ref.angle)), 0.1);
    EXPECT_GE(s.min_speed_seen, 12.0 - 1e-9);
  }
}

TEST(Orbit, EightAircraftWithPerfectKnowledgeKeepApartForFiveMinutes) {
  Swarm s;
  const std::uint32_t n = 8;
  for (std::uint32_t i = 0; i < n; ++i) {
    const OrbitReference ref = swarm::orbit_reference(i, n, 0.0, s.orbit);
    // Start 25 m off the reference in a different direction for each drone.
    const double off = 25.0;
    const Vec3 pos = ref.position + Vec3{off * std::cos(i * 1.3), off * std::sin(i * 1.3), 0.0};
    s.state.push_back(flying(pos, ref.speed, ref.angle + kPi / 2.0));
  }
  s.run(300.0);
  EXPECT_GT(s.min_separation_seen, 20.0);
  EXPECT_GE(s.min_speed_seen, 12.0 - 1e-9);
  for (std::uint32_t i = 0; i < n; ++i) {
    const OrbitReference ref = swarm::orbit_reference(i, n, s.time, s.orbit);
    EXPECT_LT(std::abs(swarm::orbit_phase_error(s.state[i].pos, s.orbit.center, ref.angle)), 0.15);
  }
}

TEST(OrbitSeparation, NoPushWhenTheClosestApproachIsOutsideTheMargin) {
  const OrbitParams p = params();
  const std::vector<OrbitNeighbor> others = {{1, {200.0, 0.0, 0.0}, {-15.0, 0.0, 0.0}, 0.0}};
  // Closing head-on at 200 m, offset 0 laterally, would collide; offset 100 m laterally does not.
  const std::vector<OrbitNeighbor> wide = {{1, {200.0, 100.0, 0.0}, {-15.0, 0.0, 0.0}, 0.0}};
  const Vec3 a = swarm::orbit_separation_accel(0, {}, {15.0, 0.0, 0.0}, wide, p, 1.0);
  EXPECT_EQ(a, Vec3{});
  const Vec3 b = swarm::orbit_separation_accel(0, {}, {15.0, 0.0, 0.0}, others, p, 1.0);
  EXPECT_GT(swarm::norm(b), 0.0);
}

TEST(OrbitSeparation, ThePushPointsAwayFromThePredictedClosestApproach) {
  const OrbitParams p = params();
  // The neighbor will pass 10 m to the left (+y) of this drone.
  const std::vector<OrbitNeighbor> others = {{1, {200.0, 10.0, 0.0}, {-15.0, 0.0, 0.0}, 0.0}};
  const Vec3 a = swarm::orbit_separation_accel(0, {}, {15.0, 0.0, 0.0}, others, p, 1.0);
  EXPECT_LT(a.y, 0.0);
  EXPECT_NEAR(a.x, 0.0, 1e-9);
}

TEST(OrbitSeparation, TheHigherSlotDodgesTwiceAsHardAsTheLower) {
  const OrbitParams p = params();
  const std::vector<OrbitNeighbor> low_sees = {{5, {200.0, 10.0, 0.0}, {-15.0, 0.0, 0.0}, 0.0}};
  const std::vector<OrbitNeighbor> high_sees = {{1, {200.0, 10.0, 0.0}, {-15.0, 0.0, 0.0}, 0.0}};
  const Vec3 low = swarm::orbit_separation_accel(1, {}, {15.0, 0.0, 0.0}, low_sees, p, 1.0);
  const Vec3 high = swarm::orbit_separation_accel(5, {}, {15.0, 0.0, 0.0}, high_sees, p, 1.0);
  ASSERT_GT(swarm::norm(low), 0.0);
  EXPECT_NEAR(swarm::norm(high), 2.0 * swarm::norm(low), 1e-9);
}

TEST(OrbitSeparation, AnUncertainNeighborGetsAWiderMargin) {
  const OrbitParams p = params();
  // 40 m miss distance: outside the 30 m margin of a known neighbor, inside the widened margin.
  const std::vector<OrbitNeighbor> known = {{1, {200.0, 40.0, 0.0}, {-15.0, 0.0, 0.0}, 0.0}};
  const std::vector<OrbitNeighbor> unsure = {{1, {200.0, 40.0, 0.0}, {-15.0, 0.0, 0.0}, 10.0}};
  EXPECT_EQ(swarm::orbit_separation_accel(0, {}, {15.0, 0.0, 0.0}, known, p, 1.0), Vec3{});
  EXPECT_GT(swarm::norm(swarm::orbit_separation_accel(0, {}, {15.0, 0.0, 0.0}, unsure, p, 1.0)),
            0.0);
}

TEST(OrbitSeparation, TwoAircraftHeadOnPassEachOtherOnTheirRight) {
  Swarm s;
  s.orbit.separation.d_safe = 40.0;
  s.orbit.separation.max_margin = 80.0;
  s.state = {flying({-300.0, 0.0, 100.0}, 17.0, 0.0), flying({300.0, 0.0, 100.0}, 17.0, kPi)};
  // Free flight for the tracking law would be an orbit; use only separation plus a straight-line
  // hold of the commanded heading so that the encounter is the thing under test.
  double closest = 1e9;
  for (int i = 0; i < 2500; ++i) {
    std::vector<DroneState> next = s.state;
    for (std::uint32_t k = 0; k < 2; ++k) {
      std::vector<OrbitNeighbor> others = {{1U - k, s.state[1 - k].pos, s.state[1 - k].vel, 0.0}};
      DroneCommand c;
      c.accel =
          swarm::orbit_separation_accel(k, s.state[k].pos, s.state[k].vel, others, s.orbit, 1.0);
      next[k] = s.vehicle.step(s.state[k], c, kDt);
    }
    s.state = next;
    closest = std::min(closest, swarm::distance(s.state[0].pos, s.state[1].pos));
  }
  EXPECT_GT(closest, 15.0);
  // Each turned right of its own heading: the eastbound aircraft ended south (y < 0), the
  // westbound one north (y > 0).
  EXPECT_LT(s.state[0].pos.y, 0.0);
  EXPECT_GT(s.state[1].pos.y, 0.0);
}

TEST(OrbitSeparation, BadInputIsRejectedAndInvalidNeighborsAreIgnored) {
  const OrbitParams p = params();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(swarm::orbit_separation_accel(0, {nan, 0, 0}, {}, {}, p, 1.0),
               std::invalid_argument);
  EXPECT_THROW(swarm::orbit_separation_accel(0, {}, {}, {}, p, 0.0), std::invalid_argument);
  const std::vector<OrbitNeighbor> bad = {{0, {1, 0, 0}, {}, 0.0},    // own slot
                                          {1, {nan, 0, 0}, {}, 0.0},  // not finite
                                          {2, {1, 0, 0}, {}, -1.0}};  // negative sigma
  EXPECT_EQ(swarm::orbit_separation_accel(0, {}, {}, bad, p, 1.0), Vec3{});
}

TEST(OrbitConsensus, WithoutUsableNeighborsThereIsNoError) {
  const OrbitParams p = params();
  double error = 123.0;
  EXPECT_FALSE(swarm::orbit_consensus_phase_error(0, 4, 5.0, {150.0, 0.0, 100.0}, {}, p, error));
  EXPECT_EQ(error, 123.0);
  const std::vector<OrbitNeighbor> unusable = {
      {0, {1, 0, 0}, {}, 0.0}, {9, {1, 0, 0}, {}, 0.0}, {1, {1, 0, 0}, {}, -1.0}};
  EXPECT_FALSE(
      swarm::orbit_consensus_phase_error(0, 4, 5.0, {150.0, 0.0, 100.0}, unusable, p, error));
}

TEST(OrbitConsensus, NeighborsOnTheirReferencesGiveTheOwnPhaseOffset) {
  const OrbitParams p = params();
  const std::uint32_t n = 6;
  const double t = 12.0;
  for (const double behind : {0.0, 0.1, -0.2, 0.5}) {
    const OrbitReference own = swarm::orbit_reference(2, n, t, p);
    // This drone is `behind` radians behind its reference; every other drone is exactly on its own.
    const double angle = own.angle - behind;
    const Vec3 position = {p.center.x + own.radius * std::cos(angle),
                           p.center.y + own.radius * std::sin(angle), own.position.z};
    std::vector<OrbitNeighbor> others;
    for (std::uint32_t j = 0; j < n; ++j) {
      if (j != 2) {
        const OrbitReference r = swarm::orbit_reference(j, n, t, p);
        others.push_back({j, r.position, r.velocity, 1.0});
      }
    }
    double error = 0.0;
    ASSERT_TRUE(swarm::orbit_consensus_phase_error(2, n, t, position, others, p, error));
    EXPECT_NEAR(error, behind, 1e-9) << behind;
    // And the clock says the same thing, so the two agree.
    EXPECT_NEAR(swarm::orbit_phase_error(position, p.center, own.angle), behind, 1e-9);
  }
}

TEST(OrbitConsensus, AnUncertainNeighborCountsForLess) {
  const OrbitParams p = params();
  const std::uint32_t n = 4;
  const double t = 3.0;
  const OrbitReference own = swarm::orbit_reference(0, n, t, p);
  const Vec3 position = own.position;  // this drone is exactly on its reference
  const OrbitReference a = swarm::orbit_reference(1, n, t, p);
  const OrbitReference b = swarm::orbit_reference(2, n, t, p);
  // Neighbor 1 is exactly where it should be; neighbor 2 is reported 0.4 rad further around.
  const double shift = 0.4;
  const double angle_b = b.angle + shift;
  const Vec3 b_shifted = {p.center.x + b.radius * std::cos(angle_b),
                          p.center.y + b.radius * std::sin(angle_b), b.position.z};
  double sure = 0.0;
  double unsure = 0.0;
  ASSERT_TRUE(swarm::orbit_consensus_phase_error(
      0, n, t, position, {{1, a.position, a.velocity, 1.0}, {2, b_shifted, b.velocity, 1.0}}, p,
      sure));
  ASSERT_TRUE(swarm::orbit_consensus_phase_error(
      0, n, t, position, {{1, a.position, a.velocity, 1.0}, {2, b_shifted, b.velocity, 80.0}}, p,
      unsure));
  EXPECT_GT(std::abs(sure), std::abs(unsure));
  EXPECT_LT(std::abs(unsure), 0.1 * shift);
}

TEST(OrbitConsensus, TheWeightIsValidated) {
  OrbitParams p = params();
  p.consensus_weight = 1.5;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = params();
  p.consensus_weight = -0.1;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = params();
  p.consensus_sigma_floor = 0.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
}
