#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/control/path.hpp"
#include "swarm/fixed_wing.hpp"

using swarm::OrbitParams;
using swarm::PatternKind;
using swarm::PatternParams;
using swarm::StadiumPath;
using swarm::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

OrbitParams orbit() {
  OrbitParams o;
  o.center = {100.0, -50.0, 120.0};
  o.speed = 17.0;
  o.min_speed = 12.0;
  o.max_speed = 25.0;
  o.radius_step = 0.0;
  return o;
}

PatternParams racetrack() {
  PatternParams p;
  p.kind = PatternKind::kRacetrack;
  p.heading = 0.4;
  p.leg_length = 500.0;
  p.turn_radius = 100.0;
  return p;
}

}  // namespace

TEST(StadiumPath, LengthIsTwoLegsAndACircle) {
  const StadiumPath path(0.0, 0.0, 0.0, 500.0, 100.0, +1);
  EXPECT_NEAR(path.length(), 1000.0 + 2.0 * kPi * 100.0, 1e-9);
  const StadiumPath circle(0.0, 0.0, 0.0, 0.0, 100.0, +1);
  EXPECT_NEAR(circle.length(), 2.0 * kPi * 100.0, 1e-9);
}

TEST(StadiumPath, IsClosedContinuousAndHasUnitTangents) {
  for (int direction : {+1, -1}) {
    const StadiumPath path(30.0, -20.0, 0.7, 500.0, 100.0, direction);
    const double step = 0.5;
    StadiumPath::Point prev = path.at(0.0);
    for (double s = step; s <= path.length() + step; s += step) {
      const StadiumPath::Point p = path.at(s);
      // Consecutive points are one step apart, up to the curvature of the step.
      EXPECT_NEAR(std::hypot(p.x - prev.x, p.y - prev.y), step, 1e-3);
      EXPECT_NEAR(std::hypot(p.tangent_x, p.tangent_y), 1.0, 1e-12);
      // The tangent points along the way the path goes.
      const double dx = p.x - prev.x;
      const double dy = p.y - prev.y;
      EXPECT_GT(dx * p.tangent_x + dy * p.tangent_y, 0.0);
      prev = p;
    }
    const StadiumPath::Point start = path.at(0.0);
    const StadiumPath::Point end = path.at(path.length());
    EXPECT_NEAR(start.x, end.x, 1e-9);
    EXPECT_NEAR(start.y, end.y, 1e-9);
  }
}

TEST(StadiumPath, CurvatureIsZeroOnTheStraightsAndOneOverRadiusOnTheTurns) {
  const StadiumPath ccw(0.0, 0.0, 0.0, 500.0, 100.0, +1);
  const StadiumPath cw(0.0, 0.0, 0.0, 500.0, 100.0, -1);
  EXPECT_EQ(ccw.at(100.0).curvature, 0.0);
  EXPECT_NEAR(ccw.at(500.0 + 100.0).curvature, 0.01, 1e-12);
  EXPECT_NEAR(cw.at(500.0 + 100.0).curvature, -0.01, 1e-12);
  EXPECT_EQ(ccw.at(500.0 + kPi * 100.0 + 100.0).curvature, 0.0);
}

TEST(StadiumPath, AClockwisePathTurnsRightAndMirrorsTheCounterclockwiseOne) {
  const StadiumPath ccw(0.0, 0.0, 0.0, 500.0, 100.0, +1);
  const StadiumPath cw(0.0, 0.0, 0.0, 500.0, 100.0, -1);
  // The first straight of the ccw path is on the right of the heading (y = -R) and flown along +x;
  // the cw path is its mirror image across the long axis.
  const StadiumPath::Point a = ccw.at(100.0);
  const StadiumPath::Point b = cw.at(100.0);
  EXPECT_NEAR(a.x, b.x, 1e-9);
  EXPECT_NEAR(a.y, -b.y, 1e-9);
  EXPECT_NEAR(a.y, -100.0, 1e-9);
}

TEST(StadiumPath, NearestFindsThePointOnEachPart) {
  const StadiumPath path(0.0, 0.0, 0.0, 500.0, 100.0, +1);
  // Beside the first straight: the foot is straight across.
  StadiumPath::Nearest n = path.nearest(50.0, -130.0);
  EXPECT_NEAR(n.point.x, 50.0, 1e-9);
  EXPECT_NEAR(n.point.y, -100.0, 1e-9);
  EXPECT_NEAR(n.distance, 30.0, 1e-9);
  // Below the path (to its right when flying +x), the cross-track is negative.
  EXPECT_LT(n.cross_track, 0.0);
  // Inside the stadium, left of the first straight.
  n = path.nearest(50.0, -50.0);
  EXPECT_GT(n.cross_track, 0.0);
  EXPECT_NEAR(n.distance, 50.0, 1e-9);
  // Out beyond the end of the +u turn: the foot is on the arc, along the line from its center.
  n = path.nearest(250.0 + 200.0, 0.0);
  EXPECT_NEAR(n.point.x, 250.0 + 100.0, 1e-9);
  EXPECT_NEAR(n.point.y, 0.0, 1e-9);
  EXPECT_NEAR(n.distance, 100.0, 1e-9);
  // Beyond the other end.
  n = path.nearest(-250.0 - 150.0, 0.0);
  EXPECT_NEAR(n.point.x, -250.0 - 100.0, 1e-9);
  EXPECT_NEAR(n.distance, 50.0, 1e-9);
  // Above, on the return straight.
  n = path.nearest(0.0, 140.0);
  EXPECT_NEAR(n.point.y, 100.0, 1e-9);
  EXPECT_NEAR(n.distance, 40.0, 1e-9);
}

TEST(StadiumPath, NearestMatchesABruteForceSearch) {
  for (int direction : {+1, -1}) {
    const StadiumPath path(10.0, 5.0, 1.1, 400.0, 80.0, direction);
    for (double x = -700.0; x <= 700.0; x += 93.0) {
      for (double y = -700.0; y <= 700.0; y += 87.0) {
        double best = std::numeric_limits<double>::infinity();
        for (double s = 0.0; s < path.length(); s += 0.25) {
          const StadiumPath::Point p = path.at(s);
          best = std::min(best, std::hypot(x - p.x, y - p.y));
        }
        EXPECT_NEAR(path.nearest(x, y).distance, best, 0.2) << x << " " << y;
      }
    }
  }
}

TEST(StadiumPath, RejectsBadInput) {
  EXPECT_THROW(StadiumPath(0, 0, 0, -1.0, 100.0, 1), std::invalid_argument);
  EXPECT_THROW(StadiumPath(0, 0, 0, 100.0, 0.0, 1), std::invalid_argument);
  EXPECT_THROW(StadiumPath(0, 0, 0, 100.0, 100.0, 0), std::invalid_argument);
  EXPECT_THROW(StadiumPath(std::numeric_limits<double>::quiet_NaN(), 0, 0, 100.0, 100.0, 1),
               std::invalid_argument);
}

TEST(Pattern, EverySlotCompletesItsLoopInTheSameTime) {
  const OrbitParams o = orbit();
  const PatternParams p = racetrack();
  const double period = swarm::pattern_path(0, 4, p, o).length() / o.speed;
  for (std::uint32_t slot = 0; slot < 4; ++slot) {
    const swarm::PatternReference a = swarm::pattern_reference(slot, 4, 0.0, p, o);
    const swarm::PatternReference b = swarm::pattern_reference(slot, 4, period, p, o);
    EXPECT_NEAR(a.position.x, b.position.x, 1e-6);
    EXPECT_NEAR(a.position.y, b.position.y, 1e-6);
  }
}

TEST(Pattern, OddSlotsFlyTheWiderStadiumAndTheLayersStack) {
  OrbitParams o = orbit();
  o.radius_step = 12.0;
  o.altitude_step = 10.0;
  const PatternParams p = racetrack();
  EXPECT_NEAR(swarm::pattern_path(0, 4, p, o).radius(), 100.0, 1e-12);
  EXPECT_NEAR(swarm::pattern_path(1, 4, p, o).radius(), 112.0, 1e-12);
  EXPECT_NEAR(swarm::pattern_reference(3, 4, 0.0, p, o).position.z, 150.0, 1e-12);
  // A longer loop is flown faster, so that it takes the same time.
  EXPECT_GT(swarm::pattern_reference(1, 4, 0.0, p, o).speed,
            swarm::pattern_reference(0, 4, 0.0, p, o).speed);
}

TEST(Pattern, AStaggerPutsEachAircraftThatManySecondsBehindTheLast) {
  const OrbitParams o = orbit();
  PatternParams p = racetrack();
  p.stagger_s = 12.0;
  const swarm::PatternReference a = swarm::pattern_reference(0, 4, 30.0, p, o);
  const swarm::PatternReference b = swarm::pattern_reference(1, 4, 30.0, p, o);
  // Aircraft 1 is where aircraft 0 was 12 s ago, if their paths are the same shape; the odd slot
  // has a radius step, which is zero in this orbit, so they are.
  const swarm::PatternReference earlier = swarm::pattern_reference(0, 4, 18.0, p, o);
  EXPECT_NEAR(b.position.x, earlier.position.x, 1e-6);
  EXPECT_NEAR(b.position.y, earlier.position.y, 1e-6);
  EXPECT_GT(std::hypot(a.position.x - b.position.x, a.position.y - b.position.y), 100.0);
}

TEST(Pattern, LanesAreSideBySideAndAlternateDirection) {
  const OrbitParams o = orbit();
  PatternParams p;
  p.kind = PatternKind::kLanes;
  p.heading = 0.0;
  p.leg_length = 600.0;
  p.turn_radius = 100.0;
  p.lane_spacing = 300.0;
  const StadiumPath l0 = swarm::pattern_path(0, 3, p, o);
  const StadiumPath l1 = swarm::pattern_path(1, 3, p, o);
  const StadiumPath l2 = swarm::pattern_path(2, 3, p, o);
  // The lanes are centered on the center across the heading: lane 1 is on it, 0 and 2 either side.
  // Lane 1 is flown clockwise, so its first straight is on the other side of its center.
  EXPECT_NEAR(l1.at(300.0).y, o.center.y + 100.0, 1e-6);
  EXPECT_NEAR(l0.at(0.0).y, o.center.y - 300.0 - 100.0, 1e-6);
  EXPECT_NEAR(l2.at(0.0).y, o.center.y + 300.0 - 100.0, 1e-6);
  // Adjacent lanes turn opposite ways.
  EXPECT_GT(l0.at(600.0 + 50.0).curvature, 0.0);
  EXPECT_LT(l1.at(600.0 + 50.0).curvature, 0.0);
  EXPECT_THROW(swarm::pattern_path(0, 3, PatternParams{}, o), std::invalid_argument);
}

TEST(Pattern, ParametersAreChecked) {
  PatternParams p = racetrack();
  p.turn_radius = 0.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  p = racetrack();
  p.stagger_s = -1.0;
  EXPECT_THROW(swarm::validate(p), std::invalid_argument);
  EXPECT_THROW(swarm::pattern_path(4, 4, racetrack(), orbit()), std::invalid_argument);
}

namespace {

// An aircraft of the simple fixed-wing model following `path` under path_guidance, from `start`.
struct Followed {
  double max_cross_track_late = 0.0;  // after the join, m
  double max_phase_error_late = 0.0;  // after the join, m
  double min_speed = 1e9;
  double max_speed = 0.0;
};

Followed follow(const StadiumPath& path, const PatternParams& pattern, const OrbitParams& orbit,
                swarm::DroneState state, double seconds, double join_seconds, double s0,
                double speed) {
  swarm::FixedWingParams limits;
  limits.min_speed = orbit.min_speed;
  limits.max_speed = orbit.max_speed;
  const swarm::FixedWingVehicle vehicle(limits);
  Followed out;
  const double dt = 0.02;
  for (int i = 0; i * dt < seconds; ++i) {
    const double t = i * dt;
    const double s_ref = s0 + speed * t;
    const double phase = swarm::path_phase_error(path, state.pos, s_ref);
    swarm::DroneCommand c;
    c.accel = swarm::path_guidance(state.pos, state.vel, path, speed, orbit.center.z, phase,
                                   pattern, orbit, 1.0);
    state = vehicle.step(state, c, dt);
    const double v = std::hypot(state.vel.x, state.vel.y);
    out.min_speed = std::min(out.min_speed, v);
    out.max_speed = std::max(out.max_speed, v);
    if (t > join_seconds) {
      out.max_cross_track_late = std::max(
          out.max_cross_track_late, std::abs(path.nearest(state.pos.x, state.pos.y).cross_track));
      out.max_phase_error_late = std::max(out.max_phase_error_late, std::abs(phase));
    }
  }
  return out;
}

}  // namespace

TEST(PathGuidance, AnAircraftOnThePathStaysOnItThroughStraightsAndTurns) {
  const OrbitParams o = orbit();
  PatternParams p = racetrack();
  p.heading = 0.0;
  const StadiumPath path = swarm::pattern_path(0, 1, p, o);
  swarm::DroneState s;
  const StadiumPath::Point start = path.at(0.0);
  s.pos = {start.x, start.y, o.center.z};
  s.vel = {17.0 * start.tangent_x, 17.0 * start.tangent_y, 0.0};
  const Followed f = follow(path, p, o, s, 150.0, 0.0, 0.0, 17.0);
  EXPECT_LT(f.max_cross_track_late, 3.0);
  EXPECT_LT(f.max_phase_error_late, 20.0);
  EXPECT_GE(f.min_speed, o.min_speed);
  EXPECT_LE(f.max_speed, o.max_speed);
}

TEST(PathGuidance, AnAircraftThatStartsOffThePathJoinsItAndHoldsIt) {
  const OrbitParams o = orbit();
  PatternParams p = racetrack();
  p.heading = 0.0;
  const StadiumPath path = swarm::pattern_path(0, 1, p, o);
  swarm::DroneState s;
  s.pos = {o.center.x - 600.0, o.center.y - 450.0, o.center.z};  // well outside the stadium
  s.vel = {17.0, 0.0, 0.0};
  const Followed f = follow(path, p, o, s, 400.0, 250.0, 0.0, 17.0);
  EXPECT_LT(f.max_cross_track_late, 5.0);
}

TEST(PathGuidance, AnAircraftBehindItsReferenceSpeedsUp) {
  const OrbitParams o = orbit();
  const PatternParams p = racetrack();
  const StadiumPath path = swarm::pattern_path(0, 1, p, o);
  const StadiumPath::Point at = path.at(200.0);
  const Vec3 pos{at.x, at.y, o.center.z};
  const Vec3 vel{17.0 * at.tangent_x, 17.0 * at.tangent_y, 0.0};
  const Vec3 behind = swarm::path_guidance(pos, vel, path, 17.0, pos.z, +100.0, p, o, 1.0);
  const Vec3 ahead = swarm::path_guidance(pos, vel, path, 17.0, pos.z, -100.0, p, o, 1.0);
  const double along_behind = behind.x * at.tangent_x + behind.y * at.tangent_y;
  const double along_ahead = ahead.x * at.tangent_x + ahead.y * at.tangent_y;
  EXPECT_GT(along_behind, along_ahead);
}

TEST(PathGuidance, RejectsBadInput) {
  const OrbitParams o = orbit();
  const PatternParams p = racetrack();
  const StadiumPath path = swarm::pattern_path(0, 1, p, o);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(swarm::path_guidance({nan, 0, 0}, {}, path, 17.0, 0.0, 0.0, p, o, 1.0),
               std::invalid_argument);
  EXPECT_THROW(swarm::path_guidance({}, {}, path, 0.0, 0.0, 0.0, p, o, 1.0), std::invalid_argument);
  EXPECT_THROW(swarm::path_guidance({}, {}, path, 17.0, 0.0, 0.0, p, o, -1.0),
               std::invalid_argument);
}
