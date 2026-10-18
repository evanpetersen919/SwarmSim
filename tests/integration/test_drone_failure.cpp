#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "swarm/control/mission.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "test_scale.hpp"

using swarm::DroneAgent;
using swarm::Scenario;
using swarm::Vec3;

namespace {

constexpr std::uint32_t kFailed = 2;
constexpr std::uint64_t kFailTick = 600;  // 12 s at 50 Hz
constexpr std::uint64_t kTicks = 1800;
// The aftermath tests fail a drone after the formation has settled, and run long enough for the
// followers to reach it.
constexpr std::uint64_t kLongFailTick = 1500;  // 30 s
constexpr std::uint64_t kLongTicks = 3000;

Scenario base_scenario() {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/formation_jam.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  Scenario s = r.scenario;
  s.duration_ticks = kTicks;
  return s;
}

Scenario with_fault() {
  Scenario s = base_scenario();
  s.faults.push_back({kFailed, kFailTick});
  return s;
}

const DroneAgent& drone(const swarm::Simulation& sim, std::size_t i) {
  return dynamic_cast<const DroneAgent&>(sim.agent(i));
}

}  // namespace

TEST(DroneFailure, ADroneFliesNormallyBeforeItFailsAndIsFrozenAfter) {
  const swarm::ScenarioWorld world = swarm::build_world(with_fault());
  const auto sim = world.make_simulation(1);

  double speed_before = 0.0;
  Vec3 frozen;
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
    const DroneAgent& d = drone(*sim, kFailed);
    if (tick + 1 == kFailTick) {
      EXPECT_TRUE(d.alive());
      speed_before = swarm::norm(d.truth().vel);
    }
    if (tick == kFailTick) {
      EXPECT_FALSE(d.alive());
      frozen = d.truth().pos;
    }
    if (tick >= kFailTick) {
      ASSERT_FALSE(d.alive()) << "tick " << tick;
      ASSERT_EQ(d.truth().pos, frozen) << "tick " << tick;
      ASSERT_EQ(d.truth().vel, Vec3{}) << "tick " << tick;
      ASSERT_EQ(d.truth().accel, Vec3{}) << "tick " << tick;
      ASSERT_EQ(sim->agent(kFailed).position(), frozen);
    }
  }
  EXPECT_GT(speed_before, 1.0);  // the mission is moving at 4 m/s, so it was flying
  // Everyone else is still alive and moving.
  for (std::size_t i = 0; i < 8; ++i) {
    if (i != kFailed) {
      EXPECT_TRUE(drone(*sim, i).alive());
    }
  }
}

TEST(DroneFailure, UntilTheFailureTickTheRunIsIdenticalToOneWithoutAFault) {
  const swarm::ScenarioWorld faulty = swarm::build_world(with_fault());
  const swarm::ScenarioWorld healthy = swarm::build_world(base_scenario());
  const auto a = faulty.make_simulation(2);
  const auto b = healthy.make_simulation(2);
  for (std::uint64_t tick = 0; tick < kFailTick; ++tick) {
    a->step();
    b->step();
    ASSERT_EQ(a->state_hash(), b->state_hash()) << "tick " << tick;
  }
  a->step();
  b->step();
  EXPECT_NE(a->state_hash(), b->state_hash());  // the failure tick itself differs
}

TEST(DroneFailure, AFailedDroneSendsNothingAndIgnoresItsInbox) {
  const swarm::ScenarioWorld world = swarm::build_world(with_fault());
  const auto sim = world.make_simulation(1);
  std::vector<swarm::ChannelEvent> events;
  sim->set_event_sink(&events);

  std::uint64_t accepted_at_failure = 0;
  std::uint64_t sent_by_failed_before = 0;
  std::uint64_t sent_by_failed_after = 0;
  std::uint64_t sent_to_failed_after = 0;
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
    if (tick == kFailTick) {
      accepted_at_failure = drone(*sim, kFailed).brain().table_counters().accepted;
    }
    if (tick > kFailTick) {
      ASSERT_EQ(drone(*sim, kFailed).brain().table_counters().accepted, accepted_at_failure);
    }
  }
  for (const swarm::ChannelEvent& e : events) {
    if (e.sender == kFailed) {
      (e.send_tick < kFailTick ? sent_by_failed_before : sent_by_failed_after) += 1;
    }
    if (e.receiver == kFailed && e.send_tick > kFailTick) {
      ++sent_to_failed_after;
    }
  }
  EXPECT_GT(sent_by_failed_before, 0U);
  EXPECT_EQ(sent_by_failed_after, 0U);
  EXPECT_GT(accepted_at_failure, 0U);
  EXPECT_GT(sent_to_failed_after, 0U);  // the others keep sending to it; it just does not listen
}

TEST(DroneFailure, TheOthersStopHearingFromItAndKeepHearingFromEachOther) {
  const swarm::ScenarioWorld world = swarm::build_world(with_fault());
  const auto sim = world.make_simulation(1);
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
  }
  const std::uint64_t silent_for = kTicks - kFailTick;  // 1500 ticks, minus the last few in flight
  for (std::uint32_t i = 0; i < 8; ++i) {
    if (i == kFailed) {
      continue;
    }
    const swarm::DroneBrain& brain = drone(*sim, i).brain();
    EXPECT_GT(brain.neighbor(kFailed).age, silent_for - 20) << "drone " << i;
    const std::uint32_t other = (i + 1) % 8 == kFailed ? (i + 2) % 8 : (i + 1) % 8;
    EXPECT_LT(brain.neighbor(other).age, 40U) << "drone " << i;
  }
}

TEST(DroneFailure, ASnapshotMarksTheDroneNotAliveFromItsFailureTickAndNotBefore) {
  std::vector<std::uint8_t> bytes;
  ASSERT_EQ(swarm::run_scenario(with_fault(), 2, bytes), swarm::LogStatus::kOk);
  swarm::LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  swarm::LogRecord rec;
  std::uint64_t checked = 0;
  while (reader.next(rec) == swarm::LogStatus::kOk) {
    swarm::Snapshot snap;
    ASSERT_EQ(swarm::decode_snapshot(rec.blob.data(), rec.blob.size(), snap),
              swarm::SnapshotStatus::kOk);
    ASSERT_EQ(snap.agents.size(), 8U);
    for (std::size_t i = 0; i < 8; ++i) {
      const bool expected_alive = !(i == kFailed && rec.tick >= kFailTick);
      ASSERT_EQ(snap.agents[i].alive, expected_alive) << "tick " << rec.tick << " drone " << i;
    }
    for (const swarm::SnapshotEvent& e : snap.events) {
      if (e.sender == kFailed) {
        ASSERT_LT(e.send_tick, kFailTick);
      }
    }
    ++checked;
  }
  EXPECT_EQ(checked, kTicks);
}

TEST(DroneFailure, AFailedDroneStillCountsForJammingZonesAtItsFrozenPosition) {
  // Find where it freezes, then jam a small zone around that point from the failure tick on.
  Vec3 frozen;
  {
    const swarm::ScenarioWorld world = swarm::build_world(with_fault());
    const auto sim = world.make_simulation(1);
    for (std::uint64_t tick = 0; tick <= kFailTick; ++tick) {
      sim->step();
    }
    frozen = drone(*sim, kFailed).truth().pos;
  }
  Scenario s = with_fault();
  swarm::Jammer j;
  j.center = frozen;
  j.radius = 3.0;
  j.start_tick = kFailTick;
  j.loss_inside = 1.0;
  j.affects = swarm::JamAffects::kReceiver;
  s.channel.jammers.push_back(j);

  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  std::vector<swarm::ChannelEvent> events;
  sim->set_event_sink(&events);
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
  }
  std::uint64_t jammed_to_failed = 0;
  std::uint64_t jammed_other = 0;
  for (const swarm::ChannelEvent& e : events) {
    if (e.outcome == swarm::MessageOutcome::kDroppedJam && e.send_tick >= kFailTick &&
        e.send_tick > 400) {  // the scenario's own jam covers ticks 100 to 400
      (e.receiver == kFailed ? jammed_to_failed : jammed_other) += 1;
    }
  }
  EXPECT_GT(jammed_to_failed, 0U);
  EXPECT_EQ(jammed_other, 0U);  // the others fly on and are far from the frozen drone
}

TEST(DroneFailure, TheScenarioFaultListSetsTheFailureTicks) {
  Scenario s = base_scenario();
  s.faults = {{1, 500}, {3, 900}};
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const std::uint64_t never = std::numeric_limits<std::uint64_t>::max();
  for (std::uint32_t i = 0; i < 8; ++i) {
    const std::uint64_t expected = i == 1 ? 500 : (i == 3 ? 900 : never);
    EXPECT_EQ(world.agents[i].fail_tick, expected) << "drone " << i;
  }
}

TEST(DroneFailure, TheLogWithAFailureIsIdenticalForEveryThreadCountAndVerifies) {
  std::vector<std::uint8_t> one;
  ASSERT_EQ(swarm::run_scenario(with_fault(), 1, one), swarm::LogStatus::kOk);
  std::vector<std::uint8_t> four;
  ASSERT_EQ(swarm::run_scenario(with_fault(), 4, four), swarm::LogStatus::kOk);
  EXPECT_EQ(one, four);
  EXPECT_EQ(swarm::verify_run_log(one.data(), one.size(), 3).status, swarm::VerifyStatus::kOk);
}

namespace {

struct Aftermath {
  double min_separation = 1e9;        // closest approach of any two drones, the failed one included
  double worst_survivor_error = 0.0;  // slot error of the others, after the failure
};

// Runs with `failing` failing at `fail_tick`, and measures from the failure on.
Aftermath fail_and_measure(Scenario s, std::uint32_t failing, std::uint64_t last_known_ticks,
                           std::uint64_t fail_tick = kLongFailTick,
                           std::uint64_t ticks = kLongTicks) {
  s.faults.push_back({failing, fail_tick});
  swarm::ScenarioWorld world = swarm::build_world(s);
  for (auto& agent : world.agents) {
    agent.brain.last_known_after_ticks = last_known_ticks;
  }
  const auto sim = world.make_simulation(2);
  const swarm::MissionPath path(world.scenario.mission);
  Aftermath out;
  for (std::uint64_t tick = 0; tick < ticks; ++tick) {
    sim->step();
    if (tick < fail_tick) {
      continue;
    }
    for (std::size_t i = 0; i < 8; ++i) {
      for (std::size_t j = i + 1; j < 8; ++j) {
        out.min_separation =
            std::min(out.min_separation,
                     swarm::distance(drone(*sim, i).truth().pos, drone(*sim, j).truth().pos));
      }
      if (i != failing) {
        const Vec3 center = path.at(static_cast<double>(tick + 1) * s.dt).position;
        out.worst_survivor_error =
            std::max(out.worst_survivor_error,
                     swarm::distance(drone(*sim, i).truth().pos, center + world.offsets[i]));
      }
    }
  }
  return out;
}

// Eight drones in a line flying along the line's own axis, so each follower's path leads straight
// through the drone ahead of it.
Scenario single_file() {
  Scenario s = base_scenario();
  s.formation_shape = swarm::FormationShape::kLine;
  s.mission.waypoints = {{0, -150, 20}, {0, 150, 20}};
  s.channel.jammers.clear();
  return s;
}

}  // namespace

TEST(DroneFailure, WhenAWedgeDroneFailsTheOthersKeepTheirSlots) {
  double worst_error = 0.0;
  double worst_separation = 1e9;
  const std::vector<std::uint32_t> failing_drones = swarm::testing::kSanitized
                                                        ? std::vector<std::uint32_t>{2}
                                                        : std::vector<std::uint32_t>{0, 2, 7};
  for (std::uint32_t failing : failing_drones) {
    const Aftermath a = fail_and_measure(base_scenario(), failing, 150);
    worst_error = std::max(worst_error, a.worst_survivor_error);
    worst_separation = std::min(worst_separation, a.min_separation);
  }
  RecordProperty("wedge_survivor_worst_error_m", std::to_string(worst_error));
  RecordProperty("wedge_min_separation_m", std::to_string(worst_separation));
  EXPECT_LT(worst_error, 2.0);       // the criterion stated in the phase design
  EXPECT_GT(worst_separation, 5.0);  // followers pass the frozen drone at the slot spacing
}

TEST(DroneFailure, InSingleFileFollowersFlyThroughAFailedDroneUnlessTheyKeepItsLastPosition) {
  const std::vector<std::uint32_t> failing_drones = swarm::testing::kSanitized
                                                        ? std::vector<std::uint32_t>{4}
                                                        : std::vector<std::uint32_t>{1, 4, 7};
  for (std::uint32_t failing : failing_drones) {
    // Without the last-known position, the prediction carries the dead drone onward and the
    // followers fly through where it really is.
    const Aftermath blind = fail_and_measure(single_file(), failing, 0);
    EXPECT_LT(blind.min_separation, 1.0) << "failing " << failing;

    const Aftermath careful = fail_and_measure(single_file(), failing, 150);
    RecordProperty("single_file_min_separation_m_drone" + std::to_string(failing),
                   std::to_string(careful.min_separation));
    EXPECT_GT(careful.min_separation, 3.0) << "failing " << failing;
  }
}

TEST(DroneFailure, TheFollowersBehindAFailedDroneStopThereAndTheSlotIsNotReassigned) {
  // The price of staying clear: nothing reassigns the failed drone's slot or routes around it, so
  // the drones behind it cannot reach their slots. This documents the limit; it is not a goal.
  const Aftermath a = fail_and_measure(single_file(), 4, 150);
  RecordProperty("single_file_worst_survivor_error_m", std::to_string(a.worst_survivor_error));
  EXPECT_GT(a.worst_survivor_error, 10.0);
}
