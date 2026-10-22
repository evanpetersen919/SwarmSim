#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/control/path.hpp"
#include "swarm/fixed_wing.hpp"
#include "swarm/format/log.hpp"
#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"

using swarm::FormationPhase;
using swarm::FormationStyle;
using swarm::OrbitParams;
using swarm::PatternKind;
using swarm::PatternParams;
using swarm::PatternReference;
using swarm::Vec3;

namespace {

OrbitParams orbit() {
  OrbitParams o;
  o.center = {0.0, 0.0, 150.0};
  o.speed = 16.7;
  o.min_speed = 12.0;
  o.max_speed = 25.0;
  o.altitude_step = 0.0;
  return o;
}

PatternParams formation() {
  PatternParams p;
  p.kind = PatternKind::kFormation;
  p.heading = 0.0;
  p.leg_length = 3000.0;
  p.turn_radius = 2000.0;
  p.stagger_s = 12.0;
  p.formation.along = 50.0;
  p.formation.across = 30.0;
  p.formation.transition_s = 25.0;
  // Already in echelon at the start, then the left echelon and the vee, as documented for the
  // ALTIUS-600 (Inside Unmanned Systems).
  p.formation.phases = {{0.0, FormationStyle::kEchelonRight},
                        {15.0, FormationStyle::kEchelonLeft},
                        {55.0, FormationStyle::kVee}};
  return p;
}

// Meters back of, and to the left of, the lead for slot `slot` at `t`, in the lead's frame.
struct Rel {
  double back;
  double left;
};

Rel relative(std::uint32_t slot, double t, const PatternParams& p, const OrbitParams& o,
             std::uint32_t n = 5) {
  const PatternReference lead = swarm::pattern_reference(0, n, t, p, o);
  const PatternReference me = swarm::pattern_reference(slot, n, t, p, o);
  const double tx = lead.velocity.x / lead.speed;
  const double ty = lead.velocity.y / lead.speed;
  const double dx = me.position.x - lead.position.x;
  const double dy = me.position.y - lead.position.y;
  return {-(dx * tx + dy * ty), dx * -ty + dy * tx};
}

}  // namespace

TEST(Formation, TheLeadStartsAtTheCenterFlyingAlongTheHeading) {
  PatternParams p = formation();
  p.heading = 0.6;
  const PatternReference lead = swarm::pattern_reference(0, 4, 0.0, p, orbit());
  EXPECT_NEAR(lead.position.x, 0.0, 1e-6);
  EXPECT_NEAR(lead.position.y, 0.0, 1e-6);
  EXPECT_NEAR(lead.velocity.x, 16.7 * std::cos(0.6), 1e-2);
  EXPECT_NEAR(lead.velocity.y, 16.7 * std::sin(0.6), 1e-2);
  EXPECT_EQ(lead.position.z, 150.0);
}

TEST(Formation, AtLaunchTheAircraftAreInTrailOneStaggerApart) {
  PatternParams p = formation();
  p.formation.phases = {{0.0, FormationStyle::kTrail}};
  for (std::uint32_t slot = 0; slot < 4; ++slot) {
    const Rel r = relative(slot, 0.0, p, orbit());
    EXPECT_NEAR(r.back, slot * 12.0 * 16.7, 0.05) << slot;
    EXPECT_NEAR(r.left, 0.0, 0.2) << slot;
  }
}

TEST(Formation, EachShapeHasItsDocumentedOffsets) {
  const PatternParams p = formation();
  const OrbitParams o = orbit();
  // Echelon to the right at the start; the left echelon is in force from 40 s (15 s plus the 25 s
  // transition) to 55 s; the vee from 80 s.
  for (std::uint32_t slot = 0; slot < 4; ++slot) {
    const Rel r = relative(slot, 5.0, p, o);
    EXPECT_NEAR(r.back, slot * 50.0, 0.2);
    EXPECT_NEAR(r.left, -static_cast<double>(slot) * 30.0, 0.2);
  }
  for (std::uint32_t slot = 0; slot < 4; ++slot) {
    const Rel r = relative(slot, 48.0, p, o);
    EXPECT_NEAR(r.back, slot * 50.0, 0.2);
    EXPECT_NEAR(r.left, static_cast<double>(slot) * 30.0, 0.2);
  }
  // The vee: ranks of two, odd slots on the left, even on the right.
  const Rel s1 = relative(1, 85.0, p, o);
  const Rel s2 = relative(2, 85.0, p, o);
  const Rel s3 = relative(3, 85.0, p, o);
  const Rel s4 = relative(4, 85.0, p, o);
  EXPECT_NEAR(s1.left, 30.0, 0.2);
  EXPECT_NEAR(s2.left, -30.0, 0.2);
  EXPECT_NEAR(s3.left, 60.0, 0.2);
  EXPECT_NEAR(s4.left, -60.0, 0.2);
  EXPECT_NEAR(s1.back, 50.0, 0.2);
  EXPECT_NEAR(s2.back, 50.0, 0.2);
  EXPECT_NEAR(s3.back, 100.0, 0.2);
}

TEST(Formation, AChangeOfShapeIsSmoothAndTakesTheTransitionTime) {
  const PatternParams p = formation();
  const OrbitParams o = orbit();
  // Slot 3 goes from the right echelon (back 150, left -90) to the left echelon (150, +90) from 15
  // s to 40 s; the smoothstep is half way at 27.5 s.
  const Rel start = relative(3, 15.0, p, o, 4);
  const Rel mid = relative(3, 27.5, p, o, 4);
  const Rel end = relative(3, 40.0, p, o, 4);
  EXPECT_NEAR(start.left, -90.0, 0.2);
  EXPECT_NEAR(end.left, 90.0, 0.2);
  EXPECT_NEAR(mid.left, 0.0, 0.2);
  // The reference moves smoothly through it: no step in velocity between close times.
  const PatternReference a = swarm::pattern_reference(3, 4, 27.49, p, o);
  const PatternReference b = swarm::pattern_reference(3, 4, 27.51, p, o);
  EXPECT_NEAR(a.velocity.x, b.velocity.x, 0.05);
  EXPECT_NEAR(a.velocity.y, b.velocity.y, 0.05);
}

TEST(Formation, TheShapeChangesAskForSpeedsAndAccelerationsThatAnAircraftCanFly) {
  const PatternParams p = formation();
  const OrbitParams o = orbit();
  const PatternReference steady = swarm::pattern_reference(2, 4, 8.0, p, o);
  EXPECT_NEAR(steady.speed, 16.7, 1e-2);
  EXPECT_NEAR(std::hypot(steady.acceleration.x, steady.acceleration.y), 0.0, 1e-2);
  EXPECT_GT(std::hypot(swarm::pattern_reference(2, 4, 27.0, p, o).acceleration.x,
                       swarm::pattern_reference(2, 4, 27.0, p, o).acceleration.y),
            0.001);
  for (double t = 0.0; t < 100.0; t += 0.5) {
    for (std::uint32_t slot = 1; slot < 4; ++slot) {
      const PatternReference r = swarm::pattern_reference(slot, 4, t, p, o);
      EXPECT_GT(r.speed, 12.0 + 1.0) << t;
      EXPECT_LT(r.speed, 25.0 - 1.0) << t;
      // Within the sideways acceleration of a 35 degree bank.
      EXPECT_LT(std::hypot(r.acceleration.x, r.acceleration.y), 6.0) << t;
    }
  }
}

TEST(Formation, ParametersAreChecked) {
  PatternParams p = formation();
  p.formation.phases = {{10.0, FormationStyle::kVee}, {5.0, FormationStyle::kTrail}};
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = formation();
  p.formation.transition_s = 0.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = formation();
  EXPECT_THROW(swarm::pattern_path(0, 4, p, orbit()), std::invalid_argument);
  EXPECT_THROW(swarm::pattern_reference(4, 4, 0.0, p, orbit()), std::invalid_argument);
}

TEST(Formation, AnAircraftFlyingTheGuidanceHoldsItsPlaceThroughTheShapeChanges) {
  const PatternParams p = formation();
  const OrbitParams o = orbit();
  swarm::FixedWingParams limits;
  limits.min_speed = o.min_speed;
  limits.max_speed = o.max_speed;
  const swarm::FixedWingVehicle vehicle(limits);
  const std::uint32_t n = 4;
  for (std::uint32_t slot = 1; slot < n; ++slot) {
    const PatternReference start = swarm::pattern_reference(slot, n, 0.0, p, o);
    swarm::DroneState s;
    s.pos = start.position;
    s.vel = start.velocity;
    const double dt = 0.02;
    double worst = 0.0;
    for (int i = 0; i * dt < 90.0; ++i) {
      const double t = i * dt;
      const PatternReference ref = swarm::pattern_reference(slot, n, t, p, o);
      swarm::DroneCommand c;
      c.accel = swarm::formation_guidance(s.pos, s.vel, ref, ref.position.z, p, o, 1.0);
      s = vehicle.step(s, c, dt);
      worst = std::max(worst, std::hypot(s.pos.x - ref.position.x, s.pos.y - ref.position.y));
    }
    EXPECT_LT(worst, 6.0) << "slot " << slot;
  }
}

namespace {

std::string scenario(const std::string& extra = "") {
  return "version: 3\n"
         "seed: 3\n"
         "duration_s: 60\n"
         "scene: {type: field, size_m: [4000, 4000], cell_m: 20, obstacles: {density_per_m2: "
         "0.0}}\n"
         "swarm: {drones: 4}\n"
         "mission:\n"
         "  waypoints:\n"
         "    - [0, 0, 150]\n"
         "  speed_mps: 16.7\n"
         "vehicle: {type: fixed_wing, dynamics: energy}\n"
         "weather: {visibility_m: 20000}\n"
         "metrics: {settle_s: 20}\n"
         "control:\n"
         "  predictor: kalman\n"
         "  orbit: {radius_step_m: 0, altitude_step_m: 0, safe_distance_m: 25}\n"
         "  pattern:\n"
         "    type: formation\n"
         "    heading_deg: 0\n"
         "    leg_length_m: 3000\n"
         "    turn_radius_m: 2000\n"
         "    formation:\n"
         "      along_m: 50\n"
         "      across_m: 30\n"
         "      phases:\n"
         "        - {at_s: 0, shape: echelon_right}\n"
         "        - {at_s: 5, shape: echelon_left}\n"
         "        - {at_s: 33, shape: vee}\n" +
         extra;
}

}  // namespace

TEST(FormationScenario, TheFormationSequenceIsFlownWithoutCollisionAndHeldClose) {
  const swarm::LoadResult r = swarm::load_scenario(scenario());
  ASSERT_TRUE(r.ok()) << (r.errors.empty() ? "" : swarm::to_string(r.errors.front()));
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(r.scenario, 2, bytes), swarm::LogStatus::kOk);
  const swarm::MetricsOutcome m = swarm::analyze_log(bytes.data(), bytes.size());
  ASSERT_TRUE(m.ok()) << m.message;
  EXPECT_EQ(m.result.alive_at_end, 4U);
  EXPECT_EQ(m.result.collision_episodes, 0U);
  // Changing from the left echelon to the vee sends one aircraft across another's side while they
  // are 50 m apart along the track, so the closest approach in the sequence is about 25 m; the
  // separation rule keeps it from being less.
  EXPECT_GT(*m.result.min_separation, 15.0);
  // Across and along the direction of flight, an aircraft is held within a few meters of its place.
  EXPECT_LT(*m.result.orbit_radial_rmse, 8.0);
  EXPECT_LT(*m.result.orbit_phase_rmse, 12.0);
}

TEST(FormationScenario, ItRoundTripsAndRefusesAnImpossibleFormation) {
  const swarm::LoadResult r = swarm::load_scenario(scenario());
  ASSERT_TRUE(r.ok());
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("type: formation"), std::string::npos);
  EXPECT_NE(text.find("shape: echelon_left"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << (again.errors.empty() ? "" : swarm::to_string(again.errors.front()));
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);
  EXPECT_EQ(again.scenario.pattern.formation.phases.size(), 3U);

  // A tight lead turn: the wingmen on the inside would fly under the minimum speed.
  std::string tight = scenario();
  const std::string from = "turn_radius_m: 2000";
  tight.replace(tight.find(from), from.size(), "turn_radius_m: 300");
  EXPECT_FALSE(swarm::load_scenario(tight).ok());
  // Phases out of order.
  std::string order = scenario();
  const std::string a = "{at_s: 5, shape: echelon_left}";
  order.replace(order.find(a), a.size(), "{at_s: 50, shape: echelon_left}");
  EXPECT_FALSE(swarm::load_scenario(order).ok());
}

TEST(Rake, TheSpacingIsTheSwathLessTheOverlap) {
  swarm::FormationPlan plan;
  plan.rake_agl = 800.0;
  plan.rake_hfov_deg = 60.0;
  plan.rake_overlap = 0.2;
  // 2 * 800 * tan(30 deg) * 0.8
  EXPECT_NEAR(swarm::rake_spacing(plan), 739.0083, 1e-3);
  plan.rake_overlap = 0.0;
  EXPECT_NEAR(swarm::rake_spacing(plan), 923.7604, 1e-3);
  plan.rake_hfov_deg = 90.0;
  EXPECT_NEAR(swarm::rake_spacing(plan), 1600.0, 1e-9);
}

TEST(Rake, TheAircraftFlyAbreastWithAlternatingAltitude) {
  PatternParams p = formation();
  p.heading = 0.5;
  p.formation.phases = {{0.0, FormationStyle::kRake}};
  p.formation.altitude_stagger = 20.0;
  const double w = swarm::rake_spacing(p.formation);
  const OrbitParams o = orbit();
  const std::uint32_t n = 15;
  for (const double t : {0.0, 20.0, 60.0}) {
    for (std::uint32_t slot = 1; slot < n; ++slot) {
      const Rel r = relative(slot, t, p, o, n);
      EXPECT_NEAR(r.back, 0.0, 1e-6) << "slot " << slot << " t " << t;
      EXPECT_NEAR(r.left, -static_cast<double>(slot) * w, 1e-6) << "slot " << slot << " t " << t;
      const PatternReference me = swarm::pattern_reference(slot, n, t, p, o);
      EXPECT_DOUBLE_EQ(me.position.z, 150.0 + (slot % 2U == 1U ? 20.0 : 0.0));
    }
  }
  // Adjacent aircraft differ in height by the stagger, and all fly the lead's heading and speed.
  const PatternReference a = swarm::pattern_reference(3, n, 10.0, p, o);
  const PatternReference b = swarm::pattern_reference(4, n, 10.0, p, o);
  EXPECT_DOUBLE_EQ(std::abs(a.position.z - b.position.z), 20.0);
  EXPECT_NEAR(a.velocity.x, b.velocity.x, 1e-6);
  EXPECT_NEAR(a.velocity.y, b.velocity.y, 1e-6);
  EXPECT_NEAR(a.speed, 16.7, 1e-6);
}

TEST(Rake, TheRakeParametersAreChecked) {
  PatternParams p = formation();
  p.formation.rake_hfov_deg = 180.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = formation();
  p.formation.rake_overlap = 1.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = formation();
  p.formation.rake_agl = 0.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = formation();
  p.formation.altitude_stagger = -1.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
}
