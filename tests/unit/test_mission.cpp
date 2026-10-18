#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/control/mission.hpp"
#include "swarm/rng.hpp"

using swarm::Mission;
using swarm::MissionPath;
using swarm::MissionReference;
using swarm::Vec3;

namespace {

Mission straight() {
  Mission m;
  m.waypoints = {{0, 0, 10}, {100, 0, 10}};
  m.speed = 5.0;
  return m;
}

Mission ell() {
  Mission m;
  m.waypoints = {{0, 0, 0}, {30, 0, 0}, {30, 40, 0}};
  m.speed = 10.0;
  return m;
}

Mission square(bool loop) {
  Mission m;
  m.waypoints = {{0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0}};
  m.speed = 10.0;
  m.loop = loop;
  return m;
}

void expect_near(const Vec3& a, const Vec3& b, double tol) {
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(a.z, b.z, tol);
}

}  // namespace

TEST(Mission, ConstantSpeedAlongAStraightLine) {
  const MissionPath path(straight());
  EXPECT_DOUBLE_EQ(path.length(), 100.0);
  EXPECT_DOUBLE_EQ(path.duration(), 20.0);

  const MissionReference start = path.at(0.0);
  expect_near(start.position, {0, 0, 10}, 1e-12);
  expect_near(start.velocity, {5, 0, 0}, 1e-12);
  EXPECT_FALSE(start.finished);

  const MissionReference middle = path.at(10.0);
  expect_near(middle.position, {50, 0, 10}, 1e-12);
  expect_near(middle.velocity, {5, 0, 0}, 1e-12);
}

TEST(Mission, ItStopsAtTheLastWaypointAndStaysThere) {
  const MissionPath path(straight());
  for (double t : {20.0, 20.5, 100.0, 1e9}) {
    const MissionReference r = path.at(t);
    expect_near(r.position, {100, 0, 10}, 1e-12);
    expect_near(r.velocity, {0, 0, 0}, 0.0);
    EXPECT_TRUE(r.finished) << "t " << t;
  }
}

TEST(Mission, PositionIsContinuousAtACornerAndVelocityTurns) {
  const MissionPath path(ell());  // 30 m east, then 40 m north, at 10 m/s: the corner is at 3 s
  EXPECT_DOUBLE_EQ(path.length(), 70.0);
  expect_near(path.at(3.0).position, {30, 0, 0}, 1e-12);
  expect_near(path.at(3.0 - 1e-9).position, {30, 0, 0}, 1e-6);
  expect_near(path.at(3.0 + 1e-9).position, {30, 0, 0}, 1e-6);
  expect_near(path.at(2.999).velocity, {10, 0, 0}, 1e-12);
  expect_near(path.at(3.001).velocity, {0, 10, 0}, 1e-12);
  expect_near(path.at(5.0).position, {30, 20, 0}, 1e-12);
  EXPECT_FALSE(path.at(6.999).finished);
  EXPECT_TRUE(path.at(7.0).finished);
}

TEST(Mission, EachWaypointIsReachedAtTheTimeTheDistancesImply) {
  swarm::Rng rng(4, 0);
  Mission m;
  m.speed = 7.0;
  for (int i = 0; i < 8; ++i) {
    m.waypoints.push_back({rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(5, 30)});
  }
  const MissionPath path(m);
  double travelled = 0.0;
  for (std::size_t i = 0; i < m.waypoints.size(); ++i) {
    if (i > 0) {
      travelled += swarm::distance(m.waypoints[i - 1], m.waypoints[i]);
    }
    expect_near(path.at(travelled / m.speed).position, m.waypoints[i], 1e-9);
  }
}

TEST(Mission, SpeedIsConstantAwayFromCorners) {
  const MissionPath path(ell());
  const double dt = 0.01;
  for (double t = 0.0; t < 6.9; t += dt) {
    if (std::abs(t - 3.0) < 2 * dt) {
      continue;  // the step straddles the corner, where the finite difference is shorter
    }
    const double step = swarm::distance(path.at(t + dt).position, path.at(t).position);
    ASSERT_NEAR(step / dt, 10.0, 1e-6) << "t " << t;
    ASSERT_NEAR(swarm::norm(path.at(t).velocity), 10.0, 1e-12);
  }
}

TEST(Mission, ALoopIncludesTheClosingSegmentAndRepeats) {
  const MissionPath path(square(true));
  EXPECT_DOUBLE_EQ(path.length(), 40.0);  // three sides plus the closing side
  expect_near(path.at(3.0).position, {0, 10, 0}, 1e-9);
  expect_near(path.at(3.5).position, {0, 5, 0}, 1e-9);  // on the closing side
  expect_near(path.at(3.5).velocity, {0, -10, 0}, 1e-12);
  expect_near(path.at(4.0).position, {0, 0, 0}, 1e-9);  // back at the start
  for (double t : {0.3, 1.7, 2.9, 3.6}) {
    for (int k = 1; k <= 5; ++k) {
      expect_near(path.at(t + 4.0 * k).position, path.at(t).position, 1e-9);
    }
  }
  EXPECT_FALSE(path.at(1e6).finished);
}

TEST(Mission, TheSameSquareWithoutLoopingStopsAtTheLastCorner) {
  const MissionPath path(square(false));
  EXPECT_DOUBLE_EQ(path.length(), 30.0);
  EXPECT_TRUE(path.at(3.0).finished);
  expect_near(path.at(50.0).position, {0, 10, 0}, 1e-12);
}

TEST(Mission, ASingleWaypointHoldsWithZeroVelocity) {
  Mission m;
  m.waypoints = {{4, 5, 6}};
  const MissionPath still(m);
  EXPECT_EQ(still.length(), 0.0);
  const MissionReference r = still.at(12.0);
  expect_near(r.position, {4, 5, 6}, 0.0);
  expect_near(r.velocity, {0, 0, 0}, 0.0);
  EXPECT_TRUE(r.finished);

  m.loop = true;
  const MissionPath looping(m);
  EXPECT_FALSE(looping.at(12.0).finished);
  expect_near(looping.at(12.0).position, {4, 5, 6}, 0.0);
}

TEST(Mission, RepeatedWaypointsTakeNoTime) {
  Mission m;
  m.waypoints = {{0, 0, 0}, {0, 0, 0}, {10, 0, 0}, {10, 0, 0}};
  m.speed = 5.0;
  const MissionPath path(m);
  EXPECT_DOUBLE_EQ(path.length(), 10.0);
  expect_near(path.at(1.0).position, {5, 0, 0}, 1e-12);
  expect_near(path.at(2.0).position, {10, 0, 0}, 1e-12);
  EXPECT_TRUE(path.at(2.0).finished);
  EXPECT_FALSE(std::isnan(path.at(0.0).position.x));

  m.waypoints = {{3, 3, 3}, {3, 3, 3}, {3, 3, 3}};
  m.loop = true;
  const MissionPath all_same(m);  // zero total length, looping: no division by zero
  expect_near(all_same.at(100.0).position, {3, 3, 3}, 0.0);
  EXPECT_FALSE(all_same.at(100.0).finished);
}

TEST(Mission, NegativeTimeIsTheStart) {
  const MissionPath path(ell());
  expect_near(path.at(-5.0).position, path.at(0.0).position, 0.0);
  expect_near(path.at(-5.0).velocity, path.at(0.0).velocity, 0.0);
}

TEST(Mission, AltitudeChangesAreInterpolatedLikeAnyOtherAxis) {
  Mission m;
  m.waypoints = {{0, 0, 10}, {0, 0, 40}};
  m.speed = 3.0;
  const MissionPath path(m);
  expect_near(path.at(5.0).position, {0, 0, 25}, 1e-12);
  expect_near(path.at(5.0).velocity, {0, 0, 3}, 1e-12);
}

TEST(Mission, ItIsAPureFunctionOfTime) {
  const MissionPath path(square(true));
  std::vector<double> times;
  for (int i = 0; i < 200; ++i) {
    times.push_back(0.137 * i);
  }
  std::vector<Vec3> forward;
  for (double t : times) {
    forward.push_back(path.at(t).position);
  }
  // Evaluate in the opposite order and from a second copy of the path: identical, bit for bit.
  const MissionPath copy(square(true));
  for (std::size_t i = times.size(); i-- > 0;) {
    EXPECT_EQ(copy.at(times[i]).position, forward[i]);
  }
}

TEST(Mission, InvalidMissionsAndTimesAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](Mission m) { EXPECT_THROW(MissionPath{m}, std::invalid_argument); };
  Mission m;
  rejects(m);  // no waypoints
  m = straight();
  m.speed = 0.0;
  rejects(m);
  m = straight();
  m.speed = -1.0;
  rejects(m);
  m = straight();
  m.speed = nan;
  rejects(m);
  m = straight();
  m.waypoints[1].y = nan;
  rejects(m);
  m = straight();
  m.waypoints[0].z = std::numeric_limits<double>::infinity();
  rejects(m);

  const MissionPath path(straight());
  EXPECT_THROW(path.at(nan), std::invalid_argument);
}
