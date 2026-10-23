#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/control/drone_brain.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/control/hazard.hpp"
#include "swarm/control/terrain_map.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/format/hash.hpp"
#include "swarm/message.hpp"
#include "swarm/net/wire.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/simulation.hpp"

using swarm::HazardPoint;
using swarm::HazardReport;
using swarm::HazardStatus;
using swarm::HazardStore;
using swarm::ShareMode;
using swarm::ShareParams;
using swarm::Vec3;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

ShareParams params(std::uint32_t min_sources = 1) {
  ShareParams p;
  p.mode = ShareMode::kShare;
  p.min_sources = min_sources;
  p.cell_m = 20.0;
  p.max_cells = 4;
  return p;
}

}  // namespace

TEST(HazardCodec, TheLayoutIsExactlyThis) {
  HazardReport r;
  r.send_tick = 0x0102030405060708ULL;
  r.sender_sigma = 1.5;  // 0x3FC00000 as a single
  r.points = {{1.0, 2.0, 3.0}, {-4.0, 0.5, 100.0}};
  const std::vector<std::uint8_t> bytes = swarm::encode_hazard_report(r);
  ASSERT_EQ(bytes.size(), swarm::kHazardHeaderBytes + 2 * swarm::kHazardPointBytes);
  const std::vector<std::uint8_t> expected = {
      1,                                               // version
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,  // send_tick, little-endian
      0x00, 0x00, 0xC0, 0x3F,                          // sender_sigma 1.5f
      2,                                               // count
      0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x40,   // 1, 2, 3
      0x00, 0x00, 0x80, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0xC8, 0x42};  // -4, 0.5, 100
  EXPECT_EQ(bytes, expected);
  EXPECT_EQ(swarm::kHazardHeaderBytes, 14U);
  // The largest report is 14 + 8 * 12 = 110 bytes.
  r.points.assign(swarm::kMaxHazardPoints, Vec3{1.0, 2.0, 3.0});
  EXPECT_EQ(swarm::encode_hazard_report(r).size(), 110U);
}

TEST(HazardCodec, ARoundTripKeepsThePointsToSinglePrecision) {
  HazardReport r;
  r.send_tick = 4321;
  r.sender_sigma = 0.25;
  r.points = {{1234.5678, -987.6543, 321.0987}, {0.0, 0.0, 0.0}};
  const std::vector<std::uint8_t> bytes = swarm::encode_hazard_report(r);
  HazardReport out;
  ASSERT_EQ(swarm::decode_hazard_report(bytes.data(), bytes.size(), out), HazardStatus::kOk);
  EXPECT_EQ(out.send_tick, 4321U);
  EXPECT_DOUBLE_EQ(out.sender_sigma, 0.25);
  ASSERT_EQ(out.points.size(), 2U);
  EXPECT_NEAR(out.points[0].x, 1234.5678, 1e-3);
  EXPECT_NEAR(out.points[0].y, -987.6543, 1e-3);
  EXPECT_NEAR(out.points[0].z, 321.0987, 1e-3);
  // An empty report is valid.
  HazardReport empty;
  const std::vector<std::uint8_t> none = swarm::encode_hazard_report(empty);
  EXPECT_EQ(none.size(), swarm::kHazardHeaderBytes);
  EXPECT_EQ(swarm::decode_hazard_report(none.data(), none.size(), out), HazardStatus::kOk);
  EXPECT_TRUE(out.points.empty());
}

TEST(HazardCodec, BadReportsAreRefusedAndLeaveTheOutputAlone) {
  HazardReport r;
  r.points = {{1.0, 2.0, 3.0}};
  const std::vector<std::uint8_t> good = swarm::encode_hazard_report(r);
  HazardReport out;
  out.send_tick = 77;

  EXPECT_EQ(swarm::decode_hazard_report(good.data(), good.size() - 1, out), HazardStatus::kBadSize);
  EXPECT_EQ(swarm::decode_hazard_report(good.data(), 5, out), HazardStatus::kBadSize);
  std::vector<std::uint8_t> bad = good;
  bad[0] = 2;
  EXPECT_EQ(swarm::decode_hazard_report(bad.data(), bad.size(), out), HazardStatus::kBadVersion);
  bad = good;
  bad[13] = 9;  // a count over the limit
  EXPECT_EQ(swarm::decode_hazard_report(bad.data(), bad.size(), out), HazardStatus::kTooMany);
  bad = good;
  bad[13] = 2;  // a count that does not match the size
  EXPECT_EQ(swarm::decode_hazard_report(bad.data(), bad.size(), out), HazardStatus::kBadSize);
  bad = good;
  bad[12] = 0x7F;  // the sigma is a NaN: 0x7FC00000
  bad[11] = 0xC0;
  EXPECT_EQ(swarm::decode_hazard_report(bad.data(), bad.size(), out), HazardStatus::kInvalidValue);
  bad = good;
  bad[12] = 0xC0;  // a negative sigma
  bad[11] = 0x80;
  bad[10] = 0x00;
  EXPECT_EQ(swarm::decode_hazard_report(bad.data(), bad.size(), out), HazardStatus::kInvalidValue);
  bad = good;
  bad[17] = 0x7F;  // a point that is a NaN
  bad[16] = 0xC0;
  bad[15] = 0x00;
  bad[14] = 0x00;
  EXPECT_EQ(swarm::decode_hazard_report(bad.data(), bad.size(), out), HazardStatus::kInvalidValue);
  EXPECT_EQ(out.send_tick, 77U);  // never touched

  // The encoder refuses what the decoder would.
  HazardReport many;
  many.points.assign(swarm::kMaxHazardPoints + 1, Vec3{});
  EXPECT_THROW(swarm::encode_hazard_report(many), std::invalid_argument);
  HazardReport nan;
  nan.points = {{std::nan(""), 0.0, 0.0}};
  EXPECT_THROW(swarm::encode_hazard_report(nan), std::invalid_argument);
  HazardReport negative;
  negative.sender_sigma = -1.0;
  EXPECT_THROW(swarm::encode_hazard_report(negative), std::invalid_argument);
  HazardReport huge;
  huge.points = {{1.0e300, 0.0, 0.0}};
  EXPECT_THROW(swarm::encode_hazard_report(huge), std::invalid_argument);
}

TEST(HazardWire, AHazardMessageCrossesTheWireWithItsType) {
  swarm::Message m;
  m.type = swarm::MessageType::kHazard;
  m.sender = 3;
  m.receiver = 5;
  m.seq = 7;
  m.send_tick = 9;
  HazardReport r;
  r.points = {{1.0, 2.0, 3.0}};
  m.payload = swarm::encode_hazard_report(r);
  const std::vector<std::uint8_t> bytes = swarm::net::encode_message(m);
  swarm::Message out;
  ASSERT_EQ(swarm::net::decode_message(bytes.data(), bytes.size(), out),
            swarm::net::WireStatus::kOk);
  EXPECT_EQ(out.type, swarm::MessageType::kHazard);
  EXPECT_EQ(out.payload, m.payload);
  EXPECT_EQ(static_cast<int>(swarm::MessageType::kHazard), 3);
}

TEST(HazardFalsePoints, AreOnTheCircleAtTheHeightOnAnArcAndAFunctionOfTheirInputs) {
  const Vec3 center{100.0, -50.0, 0.0};
  const std::vector<Vec3> a = swarm::make_false_points(center, 300.0, 150.0, 8, 7, 2, 0);
  ASSERT_EQ(a.size(), 8U);
  for (const Vec3& p : a) {
    EXPECT_NEAR(std::hypot(p.x - center.x, p.y - center.y), 300.0, 1e-9);
    EXPECT_DOUBLE_EQ(p.z, 150.0);
  }
  // The first and the last are 90 degrees apart on the circle.
  const double first = std::atan2(a.front().y - center.y, a.front().x - center.x);
  const double last = std::atan2(a.back().y - center.y, a.back().x - center.x);
  double span = last - first;
  while (span < 0.0) {
    span += 2.0 * 3.14159265358979323846;
  }
  EXPECT_NEAR(span, 90.0 * kDeg, 1e-9);
  // The same inputs give the same points; any one of them changing moves them.
  EXPECT_EQ(swarm::make_false_points(center, 300.0, 150.0, 8, 7, 2, 0)[3], a[3]);
  EXPECT_NE(swarm::make_false_points(center, 300.0, 150.0, 8, 7, 2, 1)[3], a[3]);
  EXPECT_NE(swarm::make_false_points(center, 300.0, 150.0, 8, 8, 2, 0)[3], a[3]);
  EXPECT_NE(swarm::make_false_points(center, 300.0, 150.0, 8, 7, 3, 0)[3], a[3]);
  EXPECT_EQ(swarm::make_false_points(center, 300.0, 150.0, 1, 7, 2, 0).size(), 1U);
}

TEST(HazardStore, KeepsOnePointForEachCellAndItsHighest) {
  HazardStore store(params(), 100);
  store.add_own({5.0, 5.0, 40.0}, 1.0, 10);
  store.add_own({6.0, 7.0, 60.0}, 1.0, 11);   // the same 20 m cell, higher
  store.add_own({8.0, 9.0, 50.0}, 1.0, 12);   // the same cell, lower: the point stays
  store.add_own({45.0, 5.0, 30.0}, 1.0, 12);  // another cell
  EXPECT_EQ(store.cells(), 2U);
  const std::vector<HazardPoint> points = store.usable();
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].position.z, 60.0);  // cell order: x of the cell, then y
  EXPECT_DOUBLE_EQ(points[1].position.z, 30.0);
  // Cells are on a 20 m grid, so a negative coordinate is in the cell below zero.
  store.add_own({-1.0, 5.0, 10.0}, 1.0, 12);
  EXPECT_EQ(store.cells(), 3U);
  EXPECT_DOUBLE_EQ(store.usable()[0].position.x, -1.0);
}

TEST(HazardStore, PointsExpireAfterTheTtlAndAreRefreshedWhenSeenAgain) {
  HazardStore store(params(), 100);
  store.add_own({5.0, 5.0, 40.0}, 1.0, 10);
  store.add_own({45.0, 5.0, 30.0}, 1.0, 10);
  store.expire(110);  // 100 ticks old: kept
  EXPECT_EQ(store.cells(), 2U);
  store.add_own({45.0, 5.0, 20.0}, 1.0, 100);  // seen again, lower: refreshed and not lowered
  store.expire(111);                           // the first is 101 old
  ASSERT_EQ(store.cells(), 1U);
  EXPECT_DOUBLE_EQ(store.usable()[0].position.z, 30.0);
  store.expire(200);  // 100 ticks since it was seen again: kept
  EXPECT_EQ(store.cells(), 1U);
  store.expire(201);
  EXPECT_EQ(store.cells(), 0U);
}

TEST(HazardStore, TheBoundDropsTheOldestCellsFirst) {
  HazardStore store(params(), 1000);  // at most four cells
  for (int i = 0; i < 6; ++i) {
    store.add_own({20.0 * i + 1.0, 5.0, 10.0 + i}, 1.0, static_cast<std::uint64_t>(10 + i));
  }
  EXPECT_EQ(store.cells(), 6U);
  store.expire(20);
  ASSERT_EQ(store.cells(), 4U);
  const std::vector<HazardPoint> points = store.usable();
  EXPECT_DOUBLE_EQ(points.front().position.z, 12.0);  // the two seen first are gone
  EXPECT_DOUBLE_EQ(points.back().position.z, 15.0);
}

TEST(HazardStore, TheOrderAndTheHashDoNotDependOnTheOrderOfArrival) {
  HazardStore a(params(), 100);
  HazardStore b(params(), 100);
  a.add_own({5.0, 5.0, 40.0}, 1.0, 10);
  a.add_remote(2, {45.0, 5.0, 30.0}, 1.0, 11);
  a.add_remote(1, {45.0, 6.0, 31.0}, 1.0, 11);
  b.add_remote(1, {45.0, 6.0, 31.0}, 1.0, 11);
  b.add_remote(2, {45.0, 5.0, 30.0}, 1.0, 11);
  b.add_own({5.0, 5.0, 40.0}, 1.0, 10);
  swarm::Fnv1a64 ha;
  swarm::Fnv1a64 hb;
  a.hash_state(ha);
  b.hash_state(hb);
  EXPECT_EQ(ha.digest(), hb.digest());
  const std::vector<HazardPoint> pa = a.usable();
  const std::vector<HazardPoint> pb = b.usable();
  ASSERT_EQ(pa.size(), pb.size());
  for (std::size_t i = 0; i < pa.size(); ++i) {
    EXPECT_EQ(pa[i].position, pb[i].position);
  }
}

TEST(HazardStore, OthersPointsAreUsedAtTheHeightMinSourcesOfThemSupport) {
  // One liar reports a high point, one honest sender a low one, in one cell.
  const auto fill = [](HazardStore& store) {
    store.add_remote(2, {5.0, 5.0, 100.0}, 1.0, 10);  // the liar
    store.add_remote(1, {6.0, 6.0, 40.0}, 1.0, 10);   // the honest sender
  };
  HazardStore trusting(params(1), 100);
  fill(trusting);
  EXPECT_DOUBLE_EQ(trusting.usable()[0].position.z, 100.0);  // the highest is used
  HazardStore confirmed(params(2), 100);
  fill(confirmed);
  EXPECT_DOUBLE_EQ(confirmed.usable()[0].position.z, 40.0);  // the second highest
  // A cell with one source is not used under the rule of two: the liar alone moves nothing.
  HazardStore alone(params(2), 100);
  alone.add_remote(2, {5.0, 5.0, 100.0}, 1.0, 10);
  EXPECT_TRUE(alone.usable().empty());
  EXPECT_EQ(alone.cells(), 1U);  // kept, waiting for a second source
  alone.add_remote(3, {5.0, 6.0, 90.0}, 1.0, 11);
  ASSERT_EQ(alone.usable().size(), 1U);
  EXPECT_DOUBLE_EQ(alone.usable()[0].position.z, 90.0);
  // The same sender twice is one source.
  HazardStore twice(params(2), 100);
  twice.add_remote(2, {5.0, 5.0, 100.0}, 1.0, 10);
  twice.add_remote(2, {5.0, 6.0, 110.0}, 1.0, 11);
  EXPECT_TRUE(twice.usable().empty());
}

TEST(HazardStore, TheAircraftsOwnSensingIsAlwaysTrusted) {
  HazardStore store(params(2), 100);
  store.add_own({5.0, 5.0, 50.0}, 1.0, 10);
  ASSERT_EQ(store.usable().size(), 1U);  // no other source needed
  EXPECT_DOUBLE_EQ(store.usable()[0].position.z, 50.0);
  // A higher point that two others support wins; a lower one does not.
  store.add_remote(1, {5.0, 5.0, 80.0}, 1.0, 10);
  store.add_remote(2, {6.0, 5.0, 70.0}, 1.0, 10);
  EXPECT_DOUBLE_EQ(store.usable()[0].position.z, 70.0);
  HazardStore lower(params(2), 100);
  lower.add_own({5.0, 5.0, 50.0}, 1.0, 10);
  lower.add_remote(1, {5.0, 5.0, 30.0}, 1.0, 10);
  lower.add_remote(2, {6.0, 5.0, 20.0}, 1.0, 10);
  EXPECT_DOUBLE_EQ(lower.usable()[0].position.z, 50.0);
}

TEST(HazardStore, ParametersAreChecked) {
  ShareParams bad = params();
  bad.ttl_s = 0.0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = params();
  bad.points_per_message = 0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = params();
  bad.points_per_message = 9;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = params();
  bad.min_sources = 0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = params();
  bad.fov_deg = 181.0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  bad = params();
  bad.max_cells = 0;
  EXPECT_THROW(swarm::validate(bad), std::invalid_argument);
  EXPECT_THROW(HazardStore(params(), 0), std::invalid_argument);
}

namespace {

using swarm::BrainConfig;
using swarm::BrainInput;
using swarm::DroneBrain;
using swarm::ForwardRay;
using swarm::ForwardSample;

// One aircraft heading +y at 16.7 m/s over a map that says the ground is flat at 0 m, at 150 m,
// with avoidance on: as in the avoidance tests, only a sensor can ask for anything.
BrainConfig sharing_brain(ShareMode mode, std::uint32_t min_sources = 1) {
  BrainConfig c;
  c.id = 0;
  c.num_drones = 3;
  c.dt = 0.02;
  c.offsets = swarm::make_formation(swarm::FormationShape::kLine, 3, 15.0);
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
  c.separation_enabled = false;
  c.terrain = std::make_shared<const swarm::TerrainMap>(10, 10, -2000.0, -2000.0, 400.0,
                                                        std::vector<double>(11 * 11, 0.0));
  c.terrain_guidance.enabled = true;
  c.terrain_guidance.clearance = 60.0;
  c.use_forward_sensor = true;
  c.forward_valid_ticks = 10;
  c.avoid.enabled = true;
  c.avoid.hold_s = 0.2;
  c.share.mode = mode;
  c.share.min_sources = min_sources;
  c.share.repeat_s = 0.2;  // 10 ticks
  c.share.period_ticks = 5;
  return c;
}

// A fan of three columns, +10, 0, -10 degrees to the left of the heading (+y). With `block`, the
// center column has a return at 150 m, as high as the aircraft.
ForwardSample fan(bool block) {
  ForwardSample s;
  s.available = true;
  s.columns = 3;
  const double azimuths[3] = {10.0, 0.0, -10.0};
  for (int c = 0; c < 3; ++c) {
    ForwardRay r;
    const double az = azimuths[c] * kDeg;
    r.direction = {-std::sin(az), std::cos(az), 0.0};
    r.hit = block && c == 1;
    r.range = r.hit ? 150.0 : 0.0;
    r.sigma = 1.0;
    s.rays.push_back(r);
  }
  return s;
}

swarm::InboundReport hazard_from(std::uint32_t sender, const std::vector<Vec3>& points) {
  HazardReport r;
  r.send_tick = 0;
  r.sender_sigma = 1.0;
  r.points = points;
  swarm::InboundReport in;
  in.sender = sender;
  in.type = swarm::MessageType::kHazard;
  in.payload = swarm::encode_hazard_report(r);
  return in;
}

struct StepResult {
  Vec3 accel;
  swarm::BrainOutput out;
};

StepResult step(DroneBrain& brain, std::uint64_t tick, const ForwardSample& reading,
                std::vector<swarm::InboundReport> inbox = {}) {
  BrainInput in;
  in.imu.accel = {0.0, 0.0, 9.81};
  in.fix.available = true;
  in.fix.position = {300.0, 0.0, 150.0};
  in.fix.sigma = 0.2;
  in.forward = reading;
  in.inbox = std::move(inbox);
  StepResult r;
  r.out = brain.step(in, tick);
  r.accel = r.out.command.accel;
  return r;
}

void warm_up(DroneBrain& brain) {
  for (std::uint64_t t = 0; t < 5; ++t) {
    step(brain, t, fan(false));
  }
}

}  // namespace

TEST(HazardBrain, WhatItSawBeforeStillActsWhenTheFanIsEmptyAgain) {
  DroneBrain off(sharing_brain(ShareMode::kOff));
  DroneBrain memory(sharing_brain(ShareMode::kMemory));
  warm_up(off);
  warm_up(memory);
  // The hill is seen once, on tick 5, 150 m ahead, as high as the aircraft.
  step(off, 5, fan(true));
  step(memory, 5, fan(true));
  // From then on the fan sees nothing. Without memory the aircraft flies on; with it the point
  // 150 m ahead, which it still has, blocks the heading and it turns, to the right of the empty
  // fan's tie (+x), as the rule of ADR 0014 does.
  Vec3 flying_on;
  Vec3 turning;
  for (std::uint64_t t = 6; t <= 30; ++t) {
    flying_on = step(off, t, fan(false)).accel;
    turning = step(memory, t, fan(false)).accel;
  }
  EXPECT_GT(turning.x, flying_on.x + 1.0);
  EXPECT_LE(turning.z, 2.0 + 1e-9);
}

TEST(HazardBrain, AReportFromAnotherAircraftActsBeforeItsOwnSensorSeesAnything) {
  DroneBrain memory(sharing_brain(ShareMode::kMemory));
  DroneBrain share(sharing_brain(ShareMode::kShare));
  warm_up(memory);
  warm_up(share);
  const std::vector<Vec3> hill = {{300.0, 150.0, 150.0}};  // 150 m ahead, as high as the aircraft
  Vec3 ignoring;
  Vec3 acting;
  for (std::uint64_t t = 5; t <= 12; ++t) {
    ignoring = step(memory, t, fan(false), {hazard_from(1, hill)}).accel;
    acting = step(share, t, fan(false), {hazard_from(1, hill)}).accel;
  }
  EXPECT_GT(acting.x, ignoring.x + 1.0);
}

TEST(HazardBrain, AConfirmationRuleOfTwoNeedsTwoSendersAndOneLiarMovesNothing) {
  DroneBrain trusting(sharing_brain(ShareMode::kShare, 1));
  DroneBrain confirmed(sharing_brain(ShareMode::kShare, 2));
  warm_up(trusting);
  warm_up(confirmed);
  const std::vector<Vec3> false_hill = {{300.0, 150.0, 150.0}};
  Vec3 fooled;
  Vec3 safe;
  for (std::uint64_t t = 5; t <= 12; ++t) {
    fooled = step(trusting, t, fan(false), {hazard_from(2, false_hill)}).accel;
    safe = step(confirmed, t, fan(false), {hazard_from(2, false_hill)}).accel;
  }
  // The trusting aircraft turns away from ground that is not there; the confirming one flies on,
  // like an aircraft that heard nothing.
  DroneBrain deaf(sharing_brain(ShareMode::kShare, 2));
  warm_up(deaf);
  Vec3 heard_nothing;
  for (std::uint64_t t = 5; t <= 12; ++t) {
    heard_nothing = step(deaf, t, fan(false)).accel;
  }
  EXPECT_GT(fooled.x, heard_nothing.x + 1.0);
  EXPECT_NEAR(safe.x, heard_nothing.x, 1e-9);
  // A second sender that reports the same cell makes it count.
  Vec3 both;
  for (std::uint64_t t = 13; t <= 20; ++t) {
    both = step(confirmed, t, fan(false), {hazard_from(2, false_hill), hazard_from(1, false_hill)})
               .accel;
  }
  EXPECT_GT(both.x, heard_nothing.x + 1.0);
}

TEST(HazardBrain, APointOutsideTheFieldRaisesNoFloorAndOneBetweenColumnsChoosesNoTurn) {
  DroneBrain with(sharing_brain(ShareMode::kShare));
  DroneBrain without(sharing_brain(ShareMode::kShare));
  warm_up(with);
  warm_up(without);
  // 300 m to the left of the aircraft, as high as it: 90 degrees off the heading, far outside the
  // 40 degree field. It must not raise the altitude command or turn the aircraft.
  const std::vector<Vec3> beside = {{0.0, 0.0, 150.0}};
  Vec3 a;
  Vec3 b;
  for (std::uint64_t t = 5; t <= 12; ++t) {
    a = step(with, t, fan(false), {hazard_from(1, beside)}).accel;
    b = step(without, t, fan(false)).accel;
  }
  EXPECT_NEAR(a.z, b.z, 1e-9);
  EXPECT_NEAR(a.x, b.x, 1e-9);
  // Ahead and inside the field it raises the command.
  DroneBrain ahead(sharing_brain(ShareMode::kShare));
  warm_up(ahead);
  Vec3 c;
  for (std::uint64_t t = 5; t <= 12; ++t) {
    c = step(ahead, t, fan(false), {hazard_from(1, {{300.0, 150.0, 150.0}})}).accel;
  }
  EXPECT_GT(c.z, b.z + 0.1);
  // Between the 0 and +10 degree columns, 5 degrees off: more than half the 10 degree spacing
  // from either... it is exactly half, so it is inside; at 8 degrees from the center it is within
  // half a spacing of the +10 column. At 20 degrees, beyond the fan's edge plus half a spacing, it
  // is ignored by the choice of column.
  DroneBrain wide(sharing_brain(ShareMode::kShare));
  warm_up(wide);
  Vec3 d;
  const double far_angle = 25.0 * kDeg;  // to the left of +y is -x
  for (std::uint64_t t = 5; t <= 12; ++t) {
    d = step(wide, t, fan(false),
             {hazard_from(
                 1, {{300.0 - 150.0 * std::sin(far_angle), 150.0 * std::cos(far_angle), 150.0}})})
            .accel;
  }
  EXPECT_NEAR(d.x, b.x, 1e-9);
}

TEST(HazardBrain, ReportsAreSentWhileTheHazardIsInViewAndForTheRepeatTimeAfter) {
  DroneBrain share(sharing_brain(ShareMode::kShare));
  DroneBrain memory(sharing_brain(ShareMode::kMemory));
  DroneBrain off(sharing_brain(ShareMode::kOff));
  warm_up(share);
  warm_up(memory);
  warm_up(off);
  int sent = 0;
  int memory_sent = 0;
  int off_sent = 0;
  std::uint64_t last = 0;
  // The hill is in view on ticks 5 to 14 and the repeat time is 10 ticks.
  for (std::uint64_t t = 5; t <= 60; ++t) {
    const ForwardSample reading = t <= 14 ? fan(true) : fan(false);
    const StepResult s = step(share, t, reading);
    if (s.out.has_hazard_report) {
      ++sent;
      last = t;
      HazardReport decoded;
      ASSERT_EQ(swarm::decode_hazard_report(s.out.hazard_report.data(), s.out.hazard_report.size(),
                                            decoded),
                HazardStatus::kOk);
      ASSERT_EQ(decoded.points.size(), 1U);
      // 150 m ahead of (300, 0), to within what the pose estimate moves.
      EXPECT_NEAR(decoded.points[0].y, 150.0, 10.0);
      EXPECT_NEAR(decoded.points[0].z, 150.0, 10.0);
    }
    memory_sent += step(memory, t, reading).out.has_hazard_report ? 1 : 0;
    off_sent += step(off, t, reading).out.has_hazard_report ? 1 : 0;
  }
  // Every fifth tick from 5 while in view and for 10 ticks after the last sighting at 14: ticks 5,
  // 10, 15, 20, 25 (24 is the last at which the sighting of tick 14 is within 10 ticks).
  EXPECT_EQ(sent, 4);  // 5, 10, 15, 20
  EXPECT_EQ(last, 20U);
  EXPECT_EQ(memory_sent, 0);
  EXPECT_EQ(off_sent, 0);
}

TEST(HazardBrain, AFaultySenderSendsFalsePointsOnTheCircleFromItsTime) {
  BrainConfig c = sharing_brain(ShareMode::kShare);
  c.id = 2;
  c.share.false_from_tick = 20;
  c.share.false_seed = 11;
  DroneBrain liar(c);
  warm_up(liar);
  int before = 0;
  int after = 0;
  for (std::uint64_t t = 5; t <= 40; ++t) {
    const StepResult s = step(liar, t, fan(false));  // it sees nothing
    if (!s.out.has_hazard_report) {
      continue;
    }
    if (t < 20) {
      ++before;
      continue;
    }
    ++after;
    HazardReport report;
    ASSERT_EQ(
        swarm::decode_hazard_report(s.out.hazard_report.data(), s.out.hazard_report.size(), report),
        HazardStatus::kOk);
    ASSERT_EQ(report.points.size(), swarm::kMaxHazardPoints);
    for (const Vec3& p : report.points) {
      EXPECT_NEAR(std::hypot(p.x, p.y), 300.0, 0.1);
      EXPECT_NEAR(p.z, 150.0, 0.5);
    }
  }
  EXPECT_EQ(before, 0);
  EXPECT_EQ(after, 5);  // 20, 25, 30, 35, 40
  // The fault does nothing when sharing is off or memory only: nothing is sent.
  BrainConfig quiet = sharing_brain(ShareMode::kMemory);
  quiet.id = 2;
  quiet.share.false_from_tick = 20;
  DroneBrain mute(quiet);
  warm_up(mute);
  for (std::uint64_t t = 5; t <= 40; ++t) {
    EXPECT_FALSE(step(mute, t, fan(false)).out.has_hazard_report);
  }
}

TEST(HazardBrain, MalformedReportsAndOneFromItselfAreCountedAndIgnored) {
  DroneBrain brain(sharing_brain(ShareMode::kShare));
  warm_up(brain);
  swarm::InboundReport junk;
  junk.sender = 1;
  junk.type = swarm::MessageType::kHazard;
  junk.payload = {1, 2, 3};
  const swarm::InboundReport mine = hazard_from(0, {{300.0, 150.0, 150.0}});
  const swarm::InboundReport unknown = hazard_from(9, {{300.0, 150.0, 150.0}});
  step(brain, 5, fan(false), {junk, mine, unknown});
  EXPECT_EQ(brain.hazard_malformed(), 3U);
  EXPECT_TRUE(brain.usable_hazards().empty());
}

TEST(HazardBrain, SharingNeedsAvoidanceAndOffLeavesTheStateHashUnchanged) {
  BrainConfig bad = sharing_brain(ShareMode::kShare);
  bad.avoid.enabled = false;
  EXPECT_THROW(DroneBrain{bad}, std::invalid_argument);
  BrainConfig no_sensor = sharing_brain(ShareMode::kShare);
  no_sensor.use_forward_sensor = false;
  EXPECT_THROW(DroneBrain{no_sensor}, std::invalid_argument);
  // With the mode off the hash does not see the store: a brain that was shown a hazard report and
  // one that was not have the same hash.
  DroneBrain a(sharing_brain(ShareMode::kOff));
  DroneBrain b(sharing_brain(ShareMode::kOff));
  warm_up(a);
  warm_up(b);
  step(a, 5, fan(false), {hazard_from(1, {{300.0, 150.0, 150.0}})});
  step(b, 5, fan(false));
  swarm::Fnv1a64 ha;
  swarm::Fnv1a64 hb;
  a.hash_state(ha);
  b.hash_state(hb);
  EXPECT_EQ(ha.digest(), hb.digest());
  // With sharing on it does.
  DroneBrain c(sharing_brain(ShareMode::kShare));
  DroneBrain d(sharing_brain(ShareMode::kShare));
  warm_up(c);
  warm_up(d);
  step(c, 5, fan(false), {hazard_from(1, {{300.0, 150.0, 150.0}})});
  step(d, 5, fan(false));
  swarm::Fnv1a64 hc;
  swarm::Fnv1a64 hd;
  c.hash_state(hc);
  d.hash_state(hd);
  EXPECT_NE(hc.digest(), hd.digest());
}

namespace {

std::string share_scenario(const std::string& sensors, const std::string& control,
                           const std::string& faults = std::string()) {
  return "version: 3\nseed: 1\nduration_s: 5\n"
         "scene: {type: field, size_m: [2000, 2000], cell_m: 20, obstacles: {density_per_m2: "
         "0.0}}\n"
         "swarm: {drones: 3}\n"
         "mission:\n  waypoints:\n    - [0, 0, 150]\n  speed_mps: 17\n"
         "vehicle: {type: fixed_wing}\n" +
         (sensors.empty() ? std::string() : "sensors:\n" + sensors) +
         "control:\n  orbit: {radius_m: 300, radius_step_m: 12, altitude_step_m: 10}\n" + control +
         faults;
}

std::string first_error(const swarm::LoadResult& r) {
  return r.errors.empty() ? std::string() : swarm::to_string(r.errors.front());
}

const char* kForward = "  forward: {period_ticks: 5}\n";

}  // namespace

TEST(HazardScenario, TheShareKeysAreReadRoundTripAndWrittenOnlyWhenUsed) {
  const swarm::LoadResult r = swarm::load_scenario(share_scenario(
      kForward,
      "  avoid: {}\n  share: {mode: share, ttl_s: 90, points_per_message: 4, period_ticks: 7, "
      "repeat_s: 1.5, min_sources: 2}\n"));
  ASSERT_TRUE(r.ok()) << first_error(r);
  EXPECT_EQ(r.scenario.share.mode, ShareMode::kShare);
  EXPECT_DOUBLE_EQ(r.scenario.share.ttl_s, 90.0);
  EXPECT_EQ(r.scenario.share.points_per_message, 4U);
  EXPECT_EQ(r.scenario.share.period_ticks, 7U);
  EXPECT_DOUBLE_EQ(r.scenario.share.repeat_s, 1.5);
  EXPECT_EQ(r.scenario.share.min_sources, 2U);
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("  share:"), std::string::npos);
  EXPECT_NE(text.find("min_sources: 2"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << first_error(again);
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  // An empty section turns sharing on with the defaults; memory is a mode.
  const swarm::LoadResult defaults =
      swarm::load_scenario(share_scenario(kForward, "  avoid: {}\n  share: {}\n"));
  ASSERT_TRUE(defaults.ok()) << first_error(defaults);
  EXPECT_EQ(defaults.scenario.share.mode, ShareMode::kShare);
  EXPECT_DOUBLE_EQ(defaults.scenario.share.ttl_s, 120.0);
  const swarm::LoadResult memory =
      swarm::load_scenario(share_scenario(kForward, "  avoid: {}\n  share: {mode: memory}\n"));
  ASSERT_TRUE(memory.ok());
  EXPECT_EQ(memory.scenario.share.mode, ShareMode::kMemory);
  EXPECT_NE(swarm::canonical_text(memory.scenario).find("mode: memory"), std::string::npos);

  const swarm::LoadResult plain = swarm::load_scenario(share_scenario(kForward, "  avoid: {}\n"));
  ASSERT_TRUE(plain.ok());
  EXPECT_EQ(plain.scenario.share.mode, ShareMode::kOff);
  EXPECT_EQ(swarm::canonical_text(plain.scenario).find("share"), std::string::npos);
}

TEST(HazardScenario, SharingNeedsAvoidanceAndValidValues) {
  EXPECT_FALSE(swarm::load_scenario(share_scenario(kForward, "  share: {}\n")).ok());
  EXPECT_FALSE(swarm::load_scenario(share_scenario("", "  avoid: {}\n  share: {}\n")).ok());
  for (const char* bad :
       {"mode: loud", "ttl_s: 0", "points_per_message: 0", "points_per_message: 9",
        "period_ticks: 0", "repeat_s: 0", "min_sources: 0", "unknown_key: 1"}) {
    EXPECT_FALSE(swarm::load_scenario(
                     share_scenario(kForward, std::string("  avoid: {}\n  share: {") + bad + "}\n"))
                     .ok())
        << bad;
  }
}

TEST(HazardScenario, AFalseHazardsFaultIsReadAndRoundTripsAndIsNotAFailure) {
  const std::string control = "  avoid: {}\n  share: {}\n";
  const swarm::LoadResult r = swarm::load_scenario(
      share_scenario(kForward, control, "faults:\n  - {drone: 2, at_s: 3, kind: false_hazards}\n"));
  ASSERT_TRUE(r.ok()) << first_error(r);
  ASSERT_EQ(r.scenario.faults.size(), 1U);
  EXPECT_EQ(r.scenario.faults[0].kind, swarm::FaultKind::kFalseHazards);
  const std::string text = swarm::canonical_text(r.scenario);
  EXPECT_NE(text.find("kind: false_hazards"), std::string::npos);
  const swarm::LoadResult again = swarm::load_scenario(text);
  ASSERT_TRUE(again.ok()) << first_error(again);
  EXPECT_EQ(swarm::canonical_text(again.scenario), text);

  // A fault without a kind is the failure it always was, and its text has no kind line.
  const swarm::LoadResult fail =
      swarm::load_scenario(share_scenario(kForward, control, "faults:\n  - {drone: 2, at_s: 3}\n"));
  ASSERT_TRUE(fail.ok());
  EXPECT_EQ(fail.scenario.faults[0].kind, swarm::FaultKind::kFail);
  EXPECT_EQ(swarm::canonical_text(fail.scenario).find("kind:"), std::string::npos);
  EXPECT_FALSE(swarm::load_scenario(share_scenario(kForward, control,
                                                   "faults:\n  - {drone: 2, at_s: 3, kind: lie}\n"))
                   .ok());

  // The aircraft with the fault does not stop: it is alive at the end.
  const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
  const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(1);
  for (std::uint64_t tick = 0; tick < r.scenario.duration_ticks; ++tick) {
    sim->step();
  }
  EXPECT_TRUE(dynamic_cast<const swarm::DroneAgent&>(sim->agent(2)).alive());
}

TEST(HazardFlight, ResultsDoNotDependOnTheThreadCountAndSharingLeavesAnOffRunUnchanged) {
  const auto run = [](const std::string& share, std::size_t threads) {
    const std::string text =
        "version: 3\nseed: 3\nduration_s: 20\n"
        "scene: {type: hills, size_m: [2000, 2000], cell_m: 10, amplitude_m: 300, obstacles: "
        "{density_per_m2: 0.0}}\n"
        "swarm: {drones: 6}\n"
        "mission:\n  waypoints:\n    - [0, 0, 100]\n  speed_mps: 31.3\n"
        "vehicle:\n  type: fixed_wing\n  max_speed_mps: 51\n  dynamics: energy\n"
        "  energy: {mass_kg: 28.5, wing_area_m2: 1.0, wingspan_m: 3.0, max_thrust_n: 90, "
        "max_power_w: 3500}\n"
        "environment: {ground_contact_crashes: true}\n"
        "weather: {visibility_m: 375}\n"
        "comms: {latency_ticks: 2, loss_model: independent, loss: 0.3}\n"
        "sensors:\n  forward: {period_ticks: 5}\n"
        "control:\n  orbit: {radius_m: 300, radius_step_m: 0, altitude_step_m: 8, "
        "safe_distance_m: 20}\n  avoid: {rejoin: true, feasibility: kinematic, clearance_m: 30}\n" +
        share;
    const swarm::LoadResult r = swarm::load_scenario(text);
    EXPECT_TRUE(r.ok()) << first_error(r);
    const swarm::ScenarioWorld world = swarm::build_world(r.scenario);
    const std::unique_ptr<swarm::Simulation> sim = world.make_simulation(threads);
    for (std::uint64_t tick = 0; tick < r.scenario.duration_ticks; ++tick) {
      sim->step();
    }
    return sim->state_hash();
  };
  const std::uint64_t one = run("  share: {}\n", 1);
  EXPECT_EQ(run("  share: {}\n", 4), one);
  EXPECT_EQ(run("  share: {mode: memory}\n", 1), run("  share: {mode: memory}\n", 3));
  // The share arm and the off arm are different runs, and the off arm is the same as one that
  // never mentions sharing.
  EXPECT_NE(one, run("", 1));
  EXPECT_EQ(run("  share: {mode: off}\n", 1), run("", 1));
}
