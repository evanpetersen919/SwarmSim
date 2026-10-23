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

TEST(DroneBrain, ASilentNeighborLeavesTheFormationConsensusOnlyWhenTheRuleIsOn) {
  // Neighbor 0 reports once at tick 0 and is then silent; neighbor 2 never reports. With the rule
  // on, neighbor 0 is used until it has been silent for more than 100 ticks and is dropped after.
  // With the rule off it stays, predicted onward.
  const auto used_at = [](std::uint64_t at_tick, std::uint64_t drop_after) {
    BrainConfig c = config(1, 3, {0, 0, 20}, false);
    c.drop_stale_after_ticks = drop_after;
    DroneBrain brain(c);
    for (std::uint64_t t = 0; t <= at_tick; ++t) {
      swarm::BrainInput in;
      in.imu.accel = {0.0, 0.0, 9.81};
      if (t == 0) {
        in.inbox.push_back(report_from(0, 0, {4, 0, 20}, {20, 0, 0}));
      }
      brain.step(in, t);
    }
    return brain.target().neighbors_used;
  };
  EXPECT_EQ(used_at(100, 100), 1U);  // silent for exactly the limit: still used
  EXPECT_EQ(used_at(101, 100), 0U);  // one tick past it: dropped
  EXPECT_EQ(used_at(101, 0), 1U);    // rule off: kept, predicted onward
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
    Vec3 last;
    for (std::uint64_t t = 0; t <= 200; ++t) {
      swarm::BrainInput in;
      in.imu.accel = {0.0, 0.0, 9.81};
      if (t == 0) {
        in.inbox.push_back(report_from(0, 0, {4, 0, 20}, {}));
      }
      if (supplied) {
        in.neighbor_override = far;
      }
      last = brain.step(in, t).command.accel;
    }
    return last;
  };
  EXPECT_LT(command(false).x, command(true).x - 8.0);  // without: pushed away from the 4 m ghost
  EXPECT_GT(command(true).x, -3.0);
}

namespace {

swarm::InboundReport command_message(swarm::CommandKind kind, std::uint64_t issue_tick,
                                     std::uint32_t sender = 9) {
  return {sender, swarm::encode_command({kind, issue_tick}), swarm::MessageType::kCommand};
}

// A lone drone flying a mission along +x at 5 m/s.
Rig lone_flier() { return Rig(config(0, 1, {0, 0, 20}, true)); }

double speed(const Rig& rig) { return swarm::norm(rig.truth.vel); }

}  // namespace

TEST(DroneBrainCommands, AHoldCommandStopsTheDroneNearWhereItWasAndKeepsItThere) {
  Rig rig = lone_flier();
  for (int t = 0; t < 400; ++t) {
    rig.step();
  }
  ASSERT_GT(speed(rig), 4.0);  // flying
  const Vec3 at_command = rig.truth.pos;
  rig.step({command_message(swarm::CommandKind::kHold, 400)});
  EXPECT_TRUE(rig.brain.holding_commanded());
  for (int t = 0; t < 800; ++t) {
    rig.step();
  }
  EXPECT_LT(speed(rig), 0.05);
  const Vec3 settled = rig.truth.pos;
  EXPECT_LT(swarm::distance(settled, at_command), 12.0);  // it needs room to stop from 5 m/s
  for (int t = 0; t < 300; ++t) {
    rig.step();
  }
  EXPECT_LT(swarm::distance(rig.truth.pos, settled), 0.05);  // and it stays there
  EXPECT_EQ(rig.brain.state(), FallbackState::kNominal);     // a hold is not a fallback state
  EXPECT_EQ(rig.brain.command_counters().applied, 1U);
}

TEST(DroneBrainCommands, AResumeCommandBringsTheDroneBackToTheFormation) {
  Rig rig = lone_flier();
  for (int t = 0; t < 300; ++t) {
    rig.step();
  }
  rig.step({command_message(swarm::CommandKind::kHold, 300)});
  for (int t = 0; t < 800; ++t) {
    rig.step();
  }
  ASSERT_LT(speed(rig), 0.05);
  rig.step({command_message(swarm::CommandKind::kResume, 1100)});
  EXPECT_FALSE(rig.brain.holding_commanded());
  for (int t = 0; t < 3000; ++t) {  // 30 s to catch the reference up again
    rig.step();
  }
  const Vec3 slot = Vec3{5.0 * 0.01 * static_cast<double>(rig.tick), 0.0, 20.0};
  EXPECT_LT(swarm::distance(rig.truth.pos, slot), 1.5);
  EXPECT_EQ(rig.brain.command_counters().applied, 2U);
}

TEST(DroneBrainCommands, ACommandOlderThanTheLastOneAppliedIsIgnoredWhateverTheArrivalOrder) {
  {
    // The hold was issued at tick 100 and the resume at tick 50; the hold arrives first.
    Rig rig = lone_flier();
    rig.step({command_message(swarm::CommandKind::kHold, 100)});
    rig.step({command_message(swarm::CommandKind::kResume, 50)});
    EXPECT_TRUE(rig.brain.holding_commanded());
    EXPECT_EQ(rig.brain.command_counters().stale, 1U);
  }
  {
    // The resume was issued at 200 and the hold at 100; the resume arrives first.
    Rig rig = lone_flier();
    rig.step({command_message(swarm::CommandKind::kResume, 200)});
    rig.step({command_message(swarm::CommandKind::kHold, 100)});
    EXPECT_FALSE(rig.brain.holding_commanded());
    EXPECT_EQ(rig.brain.command_counters().stale, 1U);
  }
  {
    // The same two commands in either order end in the same state: a hold at 10 and a resume at
    // 20 leave the drone flying.
    Rig a = lone_flier();
    a.step({command_message(swarm::CommandKind::kHold, 10),
            command_message(swarm::CommandKind::kResume, 20)});
    Rig b = lone_flier();
    b.step({command_message(swarm::CommandKind::kResume, 20),
            command_message(swarm::CommandKind::kHold, 10)});
    EXPECT_FALSE(a.brain.holding_commanded());
    EXPECT_FALSE(b.brain.holding_commanded());
  }
}

TEST(DroneBrainCommands, ADuplicateCommandIsIgnored) {
  Rig rig = lone_flier();
  rig.step({command_message(swarm::CommandKind::kHold, 40)});
  rig.step({command_message(swarm::CommandKind::kHold, 40)});
  rig.step({command_message(swarm::CommandKind::kResume, 40)});  // same issue tick: not newer
  EXPECT_TRUE(rig.brain.holding_commanded());
  EXPECT_EQ(rig.brain.command_counters().applied, 1U);
  EXPECT_EQ(rig.brain.command_counters().stale, 2U);
}

TEST(DroneBrainCommands, MalformedCommandsAreCountedAndIgnored) {
  Rig rig = lone_flier();
  swarm::InboundReport too_short{9, {1, 1}, swarm::MessageType::kCommand};
  swarm::InboundReport bad_version = command_message(swarm::CommandKind::kHold, 5);
  bad_version.payload[0] = 7;
  swarm::InboundReport bad_kind = command_message(swarm::CommandKind::kHold, 5);
  bad_kind.payload[1] = 0;
  rig.step({too_short, bad_version, bad_kind});
  EXPECT_FALSE(rig.brain.holding_commanded());
  EXPECT_EQ(rig.brain.command_counters().malformed, 3U);
  EXPECT_EQ(rig.brain.command_counters().applied, 0U);
}

TEST(DroneBrainCommands, ACommandPayloadSentAsABeaconIsNotACommand) {
  Rig rig = lone_flier();
  swarm::InboundReport disguised{9, swarm::encode_command({swarm::CommandKind::kHold, 5}),
                                 swarm::MessageType::kBeacon};
  rig.step({disguised});
  EXPECT_FALSE(rig.brain.holding_commanded());
  EXPECT_EQ(rig.brain.command_counters().applied, 0U);
  EXPECT_EQ(rig.brain.table_counters().unknown_sender, 1U);  // sender 9 is not a drone
}

TEST(DroneBrainCommands, TheCommandedStateIsPartOfTheBrainsHash) {
  const auto digest_after = [](std::vector<swarm::InboundReport> first) {
    Rig rig = lone_flier();
    rig.step(std::move(first));
    for (int t = 0; t < 10; ++t) {
      rig.step();
    }
    return digest(rig.brain);
  };
  const std::uint64_t none = digest_after({});
  const std::uint64_t hold = digest_after({command_message(swarm::CommandKind::kHold, 3)});
  const std::uint64_t hold_later = digest_after({command_message(swarm::CommandKind::kHold, 4)});
  EXPECT_NE(none, hold);
  EXPECT_NE(hold, hold_later);  // the issue tick of the last command is state too
  EXPECT_EQ(hold, digest_after({command_message(swarm::CommandKind::kHold, 3)}));
}

namespace {

BrainConfig with_switches(BrainConfig c, bool fallback, bool separation) {
  c.fallback_enabled = fallback;
  c.separation_enabled = separation;
  return c;
}

}  // namespace

TEST(DroneBrainAblation, BothSwitchesDefaultToOn) {
  const BrainConfig c;
  EXPECT_TRUE(c.fallback_enabled);
  EXPECT_TRUE(c.separation_enabled);
}

TEST(DroneBrainAblation, WithoutSeparationANeighborInsideTheMarginDoesNotPushTheDrone) {
  const BrainConfig base = config(0, 2, {0, 0, 20}, false);
  Rig on(with_switches(base, true, true));
  Rig off(with_switches(base, true, false));
  // The neighbor reports itself 3 m away, well inside the separation margin.
  const InboundReport close = report_from(1, 0, {0, 3, 20});
  const Vec3 pushed = on.step({close}).command.accel;
  const Vec3 free = off.step({close}).command.accel;
  EXPECT_NE(pushed, free);
  EXPECT_GT(swarm::norm(pushed - free), 1.0);
  EXPECT_LT(pushed.y, free.y - 1.0);  // with the rule it is pushed away from the neighbor, -y
}

TEST(DroneBrainAblation, SeparationOffChangesNothingWhileNeighborsAreFarApart) {
  const BrainConfig base = config(1, 3, {0, 0, 20}, false);
  Rig on(with_switches(base, true, true));
  Rig off(with_switches(base, true, false));
  for (std::uint64_t t = 0; t < 300; ++t) {
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(0, t, {0, -15, 20}));
      inbox.push_back(report_from(2, t, {0, 15, 20}));
    }
    ASSERT_EQ(on.step(inbox).command.accel, off.step(inbox).command.accel) << "tick " << t;
  }
}

TEST(DroneBrainAblation, FallbackOffChangesNothingWhileTheStateStaysNominal) {
  const BrainConfig base = config(1, 3, {0, 0, 20}, false);
  Rig on(with_switches(base, true, true));
  Rig off(with_switches(base, false, true));
  for (std::uint64_t t = 0; t < 400; ++t) {
    std::vector<InboundReport> inbox;
    if (t % 10 == 0) {
      inbox.push_back(report_from(0, t, {0, -15, 20}));
      inbox.push_back(report_from(2, t, {0, 15, 20}));
    }
    ASSERT_EQ(on.step(inbox).command.accel, off.step(inbox).command.accel) << "tick " << t;
  }
  EXPECT_EQ(on.brain.state(), FallbackState::kNominal);
  EXPECT_EQ(digest(on.brain), digest(off.brain));
}

TEST(DroneBrainAblation, WithoutFallbackALostFixDoesNotStopTheDroneButTheStateIsStillReported) {
  const BrainConfig base = config(0, 2, {0, 0, 20}, true);
  Rig on(with_switches(base, true, true));
  Rig off(with_switches(base, false, true));
  on.fixes = false;
  off.fixes = false;
  for (int t = 0; t < 1500; ++t) {
    on.step();
    off.step();
  }
  EXPECT_EQ(on.brain.state(), FallbackState::kSafeHold);
  EXPECT_EQ(off.brain.state(), FallbackState::kSafeHold);  // the machine still runs
  EXPECT_LT(speed(on), 0.2);                               // with it, the drone stopped
  EXPECT_GT(speed(off), 2.0);                              // without it, it flies on
}

TEST(DroneBrainAblation, AHoldCommandStillHoldsWithoutFallback) {
  Rig rig(with_switches(config(0, 1, {0, 0, 20}, true), false, true));
  for (int t = 0; t < 400; ++t) {
    rig.step();
  }
  rig.step({command_message(swarm::CommandKind::kHold, 400)});
  for (int t = 0; t < 800; ++t) {
    rig.step();
  }
  EXPECT_TRUE(rig.brain.holding_commanded());
  EXPECT_LT(speed(rig), 0.05);
}

TEST(DroneBrainAblation, WithoutPredictionANeighborIsHeldWhereItWasLastReported) {
  // The prediction ablation is the hold-last predictor. A neighbor reported moving at 5 m/s is
  // extrapolated by dead reckoning and not by hold-last, which shows in the drone's command.
  BrainConfig reckoning = config(0, 2, {0, 0, 20}, false);
  reckoning.predictor = PredictorKind::kDeadReckoning;
  BrainConfig hold = reckoning;
  hold.predictor = PredictorKind::kHoldLast;
  Rig a(reckoning);
  Rig b(hold);
  Vec3 last_a;
  Vec3 last_b;
  for (std::uint64_t t = 0; t < 40; ++t) {
    std::vector<InboundReport> inbox;
    if (t == 0) {
      inbox.push_back(report_from(1, 0, {0, 15, 20}, {0, 5, 0}));
    }
    last_a = a.step(inbox).command.accel;
    last_b = b.step(inbox).command.accel;
  }
  EXPECT_NE(last_a, last_b);
}
