#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "swarm/control/mission.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/ground_station.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/simulation.hpp"

using swarm::CommandKind;
using swarm::DroneAgent;
using swarm::Scenario;
using swarm::Vec3;

namespace {

constexpr std::uint32_t kDrones = 6;

Scenario base() {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/formation_jam.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  Scenario s = r.scenario;
  s.drones = kDrones;
  return s;
}

// The channel at its worst: half the beacons lost in bursts, and every link jammed from tick 1400
// to tick 1750, which covers the hold command at tick 1500. Commands are reliable and should not
// notice.
void hostile(Scenario& s) {
  s.channel.loss_model = swarm::LossModel::kBurst;
  s.channel.loss = 0.5;
  s.channel.mean_burst = 4.0;
  swarm::Jammer j;
  j.radius = std::numeric_limits<double>::infinity();
  j.start_tick = 1400;
  j.end_tick = 1750;
  j.loss_inside = 1.0;
  j.affects = swarm::JamAffects::kEither;
  s.channel.jammers = {j};
}

// The drones of the scenario plus a ground station as the last endpoint.
std::unique_ptr<swarm::Simulation> make_sim(const swarm::ScenarioWorld& world,
                                            const std::vector<swarm::StationCommand>& schedule,
                                            std::size_t threads) {
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (const swarm::DroneAgentConfig& config : world.agents) {
    agents.push_back(std::make_unique<DroneAgent>(config, *world.scene));
  }
  swarm::GroundStationConfig station;
  station.drones = kDrones;
  station.schedule = schedule;
  agents.push_back(std::make_unique<swarm::GroundStationAgent>(station));
  return std::make_unique<swarm::Simulation>(world.sim, std::move(agents), threads);
}

const DroneAgent& drone(const swarm::Simulation& sim, std::size_t i) {
  return dynamic_cast<const DroneAgent&>(sim.agent(i));
}

}  // namespace

TEST(HoldCommand, EveryDroneGetsEveryCommandDespiteHeavyBeaconLossAndAJam) {
  Scenario s = base();
  hostile(s);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = make_sim(world, {{1500, CommandKind::kHold}, {1800, CommandKind::kResume}}, 2);
  for (int t = 0; t < 2000; ++t) {
    sim->step();
  }
  for (std::size_t i = 0; i < kDrones; ++i) {
    const auto& counters = drone(*sim, i).brain().command_counters();
    EXPECT_EQ(counters.applied, 2U) << "drone " << i;
    EXPECT_EQ(counters.stale, 0U);
    EXPECT_EQ(counters.malformed, 0U);
  }
  // The beacons really were being lost, so the test means something.
  EXPECT_GT(sim->channel().stats().dropped_loss, 1000U);
  EXPECT_GT(sim->channel().stats().dropped_jam, 500U);
}

TEST(HoldCommand, ASwarmToldToHoldStopsAndToldToResumeFormsUpAgain) {
  Scenario s = base();
  hostile(s);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const swarm::MissionPath path(s.mission);
  const auto sim = make_sim(world, {{1500, CommandKind::kHold}, {1800, CommandKind::kResume}}, 2);

  std::vector<Vec3> at_1700(kDrones);
  for (int tick = 0; tick < 3300; ++tick) {
    sim->step();
    if (tick == 1700) {  // 4 s into the hold
      for (std::size_t i = 0; i < kDrones; ++i) {
        EXPECT_TRUE(drone(*sim, i).brain().holding_commanded()) << "drone " << i;
        EXPECT_LT(swarm::norm(drone(*sim, i).truth().vel), 0.6) << "drone " << i;  // measured 0.27
        at_1700[i] = drone(*sim, i).truth().pos;
      }
    }
    if (tick == 1790) {  // still holding, and still where it was
      for (std::size_t i = 0; i < kDrones; ++i) {
        EXPECT_LT(swarm::distance(drone(*sim, i).truth().pos, at_1700[i]), 1.0) << "drone " << i;
        EXPECT_LT(swarm::norm(drone(*sim, i).truth().vel), 0.25) << "drone " << i;  // measured 0.12
      }
    }
  }
  // After the resume the drones have caught the reference up again.
  const Vec3 center = path.at(3300 * s.dt).position;
  for (std::size_t i = 0; i < kDrones; ++i) {
    EXPECT_FALSE(drone(*sim, i).brain().holding_commanded());
    EXPECT_LT(swarm::distance(drone(*sim, i).truth().pos, center + world.offsets[i]), 2.0)
        << "drone " << i;
  }
}

TEST(HoldCommand, HashesAreIdenticalForEveryThreadCount) {
  Scenario s = base();
  hostile(s);
  s.channel.jammers[0].start_tick = 300;  // the jam covers the hold at tick 400 in this shorter run
  s.channel.jammers[0].end_tick = 650;
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto run = [&](std::size_t threads) {
    const auto sim =
        make_sim(world, {{400, CommandKind::kHold}, {700, CommandKind::kResume}}, threads);
    std::vector<std::uint64_t> hashes;
    for (int t = 0; t < 1000; ++t) {
      sim->step();
      hashes.push_back(sim->state_hash());
    }
    return hashes;
  };
  const std::vector<std::uint64_t> reference = run(1);
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(run(threads), reference) << "threads " << threads;
  }
}

TEST(HoldCommand, AStationWithNothingToSendDoesNotChangeTheDrones) {
  // With independent loss every message draws once, in order, so an extra silent endpoint changes
  // nothing. (With burst loss it would: each link's starting state is drawn, and there are more
  // links. A scenario without commands has no station at all.)
  Scenario s = base();
  s.channel.loss_model = swarm::LossModel::kIndependent;
  s.channel.loss = 0.3;
  s.channel.jammers.clear();
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto with_station = make_sim(world, {}, 2);
  const auto without = world.make_simulation(2);
  for (int t = 0; t < 600; ++t) {
    with_station->step();
    without->step();
    for (std::size_t i = 0; i < kDrones; ++i) {
      ASSERT_EQ(drone(*with_station, i).truth().pos, drone(*without, i).truth().pos)
          << "tick " << t << " drone " << i;
    }
  }
}
