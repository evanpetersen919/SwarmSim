#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include "swarm/control/drone_brain.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/dynamics.hpp"

using swarm::BrainConfig;
using swarm::BrainInput;
using swarm::BrainOutput;
using swarm::DroneBrain;
using swarm::FallbackState;
using swarm::InboundReport;
using swarm::NeighborReport;
using swarm::PredictorKind;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.01;

BrainConfig config(std::uint32_t id, std::uint32_t count, Vec3 start, bool moving) {
  BrainConfig c;
  c.id = id;
  c.num_drones = count;
  c.dt = kDt;
  c.offsets = swarm::make_formation(swarm::FormationShape::kLine, count, 15.0);
  if (moving) {
    c.mission.waypoints = {{0, 0, 20}, {400, 0, 20}};
    c.mission.speed = 5.0;
  } else {
    c.mission.waypoints = {{0, 0, 20}};
  }
  c.pose.dt = kDt;
  c.pose.accel_noise_density = 0.05;
  c.pose.accel_bias_walk = 0.001;
  c.predictor_config.dt = kDt;
  c.launch.position = start;
  c.launch.position_sigma = 0.1;
  c.launch.velocity_sigma = 0.1;
  c.launch.bias_sigma = 0.05;
  return c;
}

// A point-mass vehicle with a perfect IMU and, when enabled, a perfect position fix every 5 ticks.
// It is the test's stand-in for the vehicle; the brain sees only the sensor outputs.
struct Rig {
  DroneBrain brain;
  swarm::DroneState truth;
  swarm::DroneParams vehicle;
  std::uint64_t tick = 0;
  bool fixes = true;
  swarm::DroneCommand command;

  explicit Rig(const BrainConfig& c) : brain(c) {
    truth.pos = c.launch.position;
    truth.vel = c.launch.velocity;
    vehicle.max_accel = c.separation.max_accel;
    vehicle.max_speed = 30.0;
  }

  BrainOutput step(std::vector<InboundReport> inbox = {}) {
    BrainInput in;
    in.imu.accel = truth.accel + Vec3{0.0, 0.0, 9.81};  // specific force in the world frame
    if (fixes && tick % 5 == 0) {
      in.fix.available = true;
      in.fix.position = truth.pos;
      in.fix.sigma = 0.2;
    }
    in.inbox = std::move(inbox);
    const BrainOutput out = brain.step(in, tick);
    command = out.command;
    truth = swarm::step(truth, out.command, vehicle, kDt);
    ++tick;
    return out;
  }
};

InboundReport report_from(std::uint32_t sender, std::uint64_t send_tick, Vec3 position,
                          Vec3 velocity = {}, double sigma = 0.2) {
  NeighborReport r;
  r.send_tick = send_tick;
  r.position = position;
  r.velocity = velocity;
  r.position_sigma = sigma;
  return {sender, swarm::encode_neighbor_report(r)};
}

std::uint64_t digest(const DroneBrain& brain) {
  swarm::Fnv1a64 hash;
  brain.hash_state(hash);
  return hash.digest();
}

}  // namespace

TEST(DroneBrain, ALoneDroneFliesTheMissionAndHoldsItsSlot) {
  Rig rig(config(0, 1, {0, 0, 20}, true));
  for (int i = 0; i < 2000; ++i) {  // 20 s
    rig.step();
  }
  const Vec3 reference{5.0 * 20.0, 0.0, 20.0};
  EXPECT_LT(swarm::distance(rig.truth.pos, reference), 1.0);
  EXPECT_LT(swarm::norm(rig.truth.vel - Vec3{5, 0, 0}), 0.3);
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);
}

TEST(DroneBrain, ADroneThatIsInPlaceAmongConsistentNeighborsCommandsAlmostNothing) {
  // Three drones in a line, 15 m apart, centered on the single waypoint (0, 0, 20). This is drone
  // 1, the middle slot, so it sits at the center.
  Rig rig(config(1, 3, {0, 0, 20}, false));
  const Vec3 left{0, -15, 20};
  const Vec3 right{0, 15, 20};
  double worst = 0.0;
  for (std::uint64_t t = 0; t < 1000; ++t) {
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(0, t, left));
      inbox.push_back(report_from(2, t, right));
    }
    rig.step(std::move(inbox));
    if (t > 100) {
      worst = std::max(worst, swarm::norm(rig.command.accel));
    }
  }
  EXPECT_LT(worst, 0.1);
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);
  EXPECT_EQ(rig.brain.target().neighbors_used, 2U);
}

TEST(DroneBrain, StoppedReportsDriveItToDegradedCommsAndResumedReportsBring_ItBack) {
  Rig rig(config(1, 3, {0, 0, 20}, false));
  const auto reports = [&](std::uint64_t t) {
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(0, t, {0, -15, 20}));
      inbox.push_back(report_from(2, t, {0, 15, 20}));
    }
    return inbox;
  };
  std::set<FallbackState> seen;
  for (std::uint64_t t = 0; t < 200; ++t) {
    rig.step(reports(t));
  }
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);
  for (std::uint64_t t = 200; t < 450; ++t) {  // silence; the last report was at tick 190
    rig.step();
    seen.insert(rig.brain.state());
    if (t < 289) {
      EXPECT_EQ(rig.brain.state(), FallbackState::kNominal) << "tick " << t;  // still fresh
    }
  }
  EXPECT_EQ(rig.brain.state(), FallbackState::kDegradedComms);
  EXPECT_EQ(rig.brain.fresh_neighbors(), 0U);
  for (std::uint64_t t = 450; t < 800; ++t) {
    rig.step(reports(t));
  }
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);
  EXPECT_EQ(rig.brain.fresh_neighbors(), 2U);
}

TEST(DroneBrain, LosingFixesAndReportsPassesThroughDegradedNavigationToSafeHoldAndRecovers) {
  Rig rig(config(1, 3, {0, 0, 20}, true));
  const auto reports = [&](std::uint64_t t) {
    // Neighbors fly the mission too: their positions follow the reference.
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      const double x = 5.0 * kDt * static_cast<double>(t);
      inbox.push_back(report_from(0, t, {x, -15, 20}, {5, 0, 0}));
      inbox.push_back(report_from(2, t, {x, 15, 20}, {5, 0, 0}));
    }
    return inbox;
  };
  for (std::uint64_t t = 0; t < 300; ++t) {
    rig.step(reports(t));
  }
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);

  std::set<FallbackState> seen;
  rig.fixes = false;
  for (std::uint64_t t = 300; t < 1200; ++t) {  // no fixes, no reports
    rig.step();
    seen.insert(rig.brain.state());
  }
  EXPECT_TRUE(seen.count(FallbackState::kDegradedNavigation) > 0);
  EXPECT_EQ(rig.brain.state(), FallbackState::kSafeHold);
  EXPECT_LT(swarm::norm(rig.truth.vel), 0.2);  // it stopped

  rig.fixes = true;
  for (std::uint64_t t = 1200; t < 2100; ++t) {
    // Neighbors held at the position the formation had when they stopped being heard from.
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(0, t, rig.truth.pos + Vec3{0, -15, 0}));
      inbox.push_back(report_from(2, t, rig.truth.pos + Vec3{0, 15, 0}));
    }
    rig.step(std::move(inbox));
  }
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);
}

TEST(DroneBrain, SafeHoldHoldsThePositionItEnteredAt) {
  Rig rig(config(0, 2, {0, 0, 20}, true));
  rig.fixes = false;
  std::vector<Vec3> at_entry;
  Vec3 entry;
  bool entered = false;
  for (std::uint64_t t = 0; t < 1500; ++t) {
    rig.step();
    if (!entered && rig.brain.state() == FallbackState::kSafeHold) {
      entered = true;
      entry = rig.brain.pose().position;
    }
  }
  ASSERT_TRUE(entered);
  // After stopping, the drone sits within a few meters of where it was when it entered.
  EXPECT_LT(swarm::distance(rig.truth.pos, entry), 10.0);
  EXPECT_LT(swarm::norm(rig.truth.vel), 0.05);
}

TEST(DroneBrain, IdenticalInputSequencesGiveBitIdenticalCommandsAndHashes) {
  const BrainConfig c = config(1, 3, {0, 0, 20}, true);
  Rig a(c);
  Rig b(c);
  for (std::uint64_t t = 0; t < 600; ++t) {
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(0, t, {0.05 * static_cast<double>(t), -15, 20}, {5, 0, 0}));
    }
    a.step(inbox);
    b.step(inbox);
    ASSERT_EQ(a.command.accel, b.command.accel) << "tick " << t;
  }
  EXPECT_EQ(digest(a.brain), digest(b.brain));
}

TEST(DroneBrain, InterleavedBrainsDoNotInterfere) {
  const BrainConfig c = config(1, 3, {0, 0, 20}, true);
  Rig alone(c);
  Rig a(c);
  Rig other(config(0, 3, {0, -15, 20}, true));
  for (std::uint64_t t = 0; t < 400; ++t) {
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(2, t, {0.05 * static_cast<double>(t), 15, 20}, {5, 0, 0}));
    }
    alone.step(inbox);
    other.step();
    a.step(inbox);
    ASSERT_EQ(a.command.accel, alone.command.accel) << "tick " << t;
  }
  EXPECT_EQ(digest(a.brain), digest(alone.brain));
}

TEST(DroneBrain, ThePredictorKindComesFromTheConfiguration) {
  const auto estimate_after_silence = [](PredictorKind kind) {
    BrainConfig c = config(0, 2, {0, 0, 20}, false);
    c.predictor = kind;
    Rig rig(c);
    rig.step({report_from(1, 0, {0, 15, 20}, {2, 0, 0})});
    for (int i = 0; i < 50; ++i) {
      rig.step();
    }
    return rig.brain.neighbor(1);
  };
  const auto hold = estimate_after_silence(PredictorKind::kHoldLast);
  const auto dead = estimate_after_silence(PredictorKind::kDeadReckoning);
  EXPECT_NEAR(hold.position.x, 0.0, 1e-9);        // the last report, unchanged
  EXPECT_NEAR(dead.position.x, 2.0 * 0.5, 1e-6);  // 50 ticks at 2 m/s
}

TEST(DroneBrain, EveryInputThatAffectsBehaviorChangesTheHash) {
  const BrainConfig c = config(1, 3, {0, 0, 20}, true);
  const auto run = [&](auto&& tweak) {
    Rig rig(c);
    for (std::uint64_t t = 0; t < 300; ++t) {
      std::vector<InboundReport> inbox;
      if (t % 10 == 0) {
        inbox.push_back(report_from(0, t, {0.05 * static_cast<double>(t), -15, 20}, {5, 0, 0}));
      }
      tweak(rig, t, inbox);
      rig.step(std::move(inbox));
    }
    return digest(rig.brain);
  };
  const auto none = [](Rig&, std::uint64_t, std::vector<InboundReport>&) {};
  const std::uint64_t base = run(none);
  EXPECT_EQ(run(none), base);

  // A neighbor report that differs slightly.
  EXPECT_NE(run([](Rig&, std::uint64_t t, std::vector<InboundReport>& inbox) {
              if (t == 100) {
                inbox.push_back(report_from(2, t, {5.0, 15.5, 20}));
              }
            }),
            base);
  // A missed fix.
  EXPECT_NE(run([](Rig& rig, std::uint64_t t, std::vector<InboundReport>&) {
              rig.fixes = !(t >= 100 && t < 105);
            }),
            base);
  // A malformed payload, which is counted.
  EXPECT_NE(run([](Rig&, std::uint64_t t, std::vector<InboundReport>& inbox) {
              if (t == 50) {
                inbox.push_back({2, {1, 2, 3}});
              }
            }),
            base);
  // A change of the fallback state: all reports silenced from tick 100.
  EXPECT_NE(run([](Rig&, std::uint64_t t, std::vector<InboundReport>& inbox) {
              if (t >= 100) {
                inbox.clear();
              }
            }),
            base);
}

TEST(DroneBrain, BroadcastsItsOwnEstimateOnTheReportPeriodNeverTruth) {
  BrainConfig c = config(0, 2, {0, 0, 20}, false);
  c.launch.position = {0.4, 0.0, 20.0};  // the brain is told a position 0.4 m off the truth
  Rig rig(c);
  rig.truth.pos = {0.0, 0.0, 20.0};
  rig.fixes = false;
  int reports = 0;
  for (std::uint64_t t = 0; t < 100; ++t) {
    const BrainOutput out = rig.step();
    EXPECT_EQ(out.has_report, t % c.report_period_ticks == 0) << "tick " << t;
    if (out.has_report) {
      ++reports;
      NeighborReport decoded;
      ASSERT_EQ(swarm::decode_neighbor_report(out.report.data(), out.report.size(), decoded),
                swarm::DecodeStatus::kOk);
      EXPECT_EQ(decoded.send_tick, t);
      // The estimate started at the launch position; with no fixes it never moves to the truth.
      EXPECT_NEAR(decoded.position.x, rig.brain.pose().position.x, 0.5);
      EXPECT_GT(decoded.position_sigma, 0.0);
    }
  }
  EXPECT_EQ(reports, 10);
}

TEST(DroneBrain, MalformedAndUnknownSenderPayloadsAreCountedNotFatal) {
  Rig rig(config(0, 2, {0, 0, 20}, false));
  rig.step({{1, {1, 2, 3}}, {9, {0}}});
  EXPECT_EQ(rig.brain.table_counters().malformed, 1U);
  EXPECT_EQ(rig.brain.table_counters().unknown_sender, 1U);
  EXPECT_FALSE(rig.brain.neighbor(1).valid);
}

TEST(DroneBrain, InvalidConfigurationAndTicksAreRejected) {
  const auto rejects = [](BrainConfig c) { EXPECT_THROW(DroneBrain{c}, std::invalid_argument); };
  BrainConfig c = config(0, 2, {0, 0, 20}, false);
  c.id = 2;
  rejects(c);
  c = config(0, 2, {0, 0, 20}, false);
  c.offsets.pop_back();
  rejects(c);
  c = config(0, 2, {0, 0, 20}, false);
  c.dt = 0.02;
  rejects(c);
  c = config(0, 2, {0, 0, 20}, false);
  c.report_period_ticks = 0;
  rejects(c);
  c = config(0, 2, {0, 0, 20}, false);
  c.formation.alpha = 2.0;
  rejects(c);
  c = config(0, 2, {0, 0, 20}, false);
  c.mission.waypoints.clear();
  rejects(c);
  c = config(0, 2, {0, 0, 20}, false);
  c.offsets = {{0, 0, 0}, {0, c.separation.max_margin, 0}};  // not farther apart than the margin
  rejects(c);

  DroneBrain brain(config(0, 2, {0, 0, 20}, false));
  BrainInput in;
  brain.step(in, 10);
  EXPECT_THROW(brain.step(in, 9), std::invalid_argument);
  in.imu.accel.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(brain.step(in, 11), std::invalid_argument);
}

TEST(DroneBrain, CommsIsNotJudgedBeforeTheFirstStalenessWindowHasPassed) {
  BrainConfig c = config(0, 3, {0, 0, 20}, false);
  c.staleness_ticks = 100;
  Rig rig(c);
  for (std::uint64_t t = 0; t < 99; ++t) {  // no report ever arrives
    rig.step();
    ASSERT_EQ(rig.brain.state(), FallbackState::kNominal) << "tick " << t;
  }
  EXPECT_EQ(rig.brain.fresh_neighbors(), 0U);
  for (std::uint64_t t = 99; t < 300; ++t) {
    rig.step();
  }
  EXPECT_EQ(rig.brain.state(), FallbackState::kDegradedComms);
}

namespace {

// The brain driven directly with a hovering IMU and no fixes, so its pose stays at the launch
// position and only the neighbor handling changes between runs. Neighbor 0 reports once, at tick 0,
// a position 4 m away moving away at 20 m/s, and is then silent. Returns the command at `at_tick`.
Vec3 command_at(std::uint64_t at_tick, std::uint64_t last_known_after_ticks) {
  BrainConfig c = config(1, 3, {0, 0, 20}, false);
  c.last_known_after_ticks = last_known_after_ticks;
  DroneBrain brain(c);
  Vec3 command;
  for (std::uint64_t t = 0; t <= at_tick; ++t) {
    swarm::BrainInput in;
    in.imu.accel = {0.0, 0.0, 9.81};
    if (t == 0) {
      in.inbox.push_back(report_from(0, 0, {4, 0, 20}, {20, 0, 0}));
    }
    command = brain.step(in, t).command.accel;
  }
  return command;
}

}  // namespace

TEST(DroneBrain, ASilentNeighborsLastReportedPositionStaysAPointToKeepClearOf) {
  // By tick 200 the prediction is 80 m away, so without the rule nothing pushes; the last reported
  // position is still 4 m away, on the +x side, so the rule pushes toward -x.
  const Vec3 off = command_at(200, 0);
  const Vec3 on = command_at(200, 150);
  EXPECT_GT(off.x, -3.0);
  EXPECT_LT(on.x, off.x - 8.0);
}

TEST(DroneBrain, TheLastKnownPositionAppliesOnlyAfterTheConfiguredSilence) {
  // Before the threshold the two configurations are bit-identical; at it they differ.
  EXPECT_EQ(command_at(149, 150), command_at(149, 0));
  EXPECT_NE(command_at(150, 150), command_at(150, 0));
}

TEST(DroneBrain, SuppliedNeighborEstimatesReplaceThePredictorsAndMakeEveryNeighborFresh) {
  const BrainConfig c = config(1, 3, {0, 0, 20}, false);
  DroneBrain with(c);
  DroneBrain without(c);
  std::vector<swarm::NeighborEstimate> truth(3);
  truth[0].valid = true;
  truth[0].position = {0, -15, 20};
  truth[2].valid = true;
  truth[2].position = {0, 15, 20};
  for (std::uint64_t t = 0; t < 400; ++t) {  // no report ever arrives
    swarm::BrainInput in;
    in.imu.accel = {0.0, 0.0, 9.81};
    if (t % 5 == 0) {  // position fixes keep navigation good
      in.fix.available = true;
      in.fix.position = {0, 0, 20};
      in.fix.sigma = 0.2;
    }
    swarm::BrainInput plain = in;
    in.neighbor_override = truth;
    with.step(in, t);
    without.step(plain, t);
  }
  EXPECT_EQ(with.state(), FallbackState::kNominal);
  EXPECT_EQ(with.fresh_neighbors(), 2U);
  EXPECT_EQ(with.neighbor(0).position, truth[0].position);
  EXPECT_EQ(with.neighbor(2).position, truth[2].position);
  EXPECT_EQ(with.target().neighbors_used, 2U);
  // The same silence without the supplied estimates drives the usual fallback.
  EXPECT_EQ(without.state(), FallbackState::kDegradedComms);
  EXPECT_EQ(without.fresh_neighbors(), 0U);
}

TEST(DroneBrain, ASuppliedEstimateListOfTheWrongSizeIsRejected) {
  DroneBrain brain(config(0, 3, {0, 0, 20}, false));
  swarm::BrainInput in;
  in.neighbor_override = std::vector<swarm::NeighborEstimate>(2);
  EXPECT_THROW(brain.step(in, 0), std::invalid_argument);
}

TEST(DroneBrain, SuppliedEstimatesAlsoTurnOffTheLastKnownPositionRule) {
  // With reports that stop and estimates supplied, the silent neighbor's last reported position
  // plays no part, because the channel is not allowed to matter.
  const auto command = [](bool supplied) {
    BrainConfig c = config(1, 3, {0, 0, 20}, false);
    c.last_known_after_ticks = 150;
    DroneBrain brain(c);
    std::vector<swarm::NeighborEstimate> far(3);
    far[0].valid = true;
    far[0].position = {80, 0, 20};
    Vec3 command;
    for (std::uint64_t t = 0; t <= 200; ++t) {
      swarm::BrainInput in;
      in.imu.accel = {0.0, 0.0, 9.81};
      if (t == 0) {
        in.inbox.push_back(report_from(0, 0, {4, 0, 20}, {}));
      }
      if (supplied) {
        in.neighbor_override = far;
      }
      command = brain.step(in, t).command.accel;
    }
    return command;
  };
  EXPECT_LT(command(false).x, command(true).x - 8.0);  // without: pushed away from the 4 m ghost
  EXPECT_GT(command(true).x, -3.0);
}
