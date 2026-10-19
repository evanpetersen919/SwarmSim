#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "swarm/channel.hpp"
#include "swarm/channel_model.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/ground_station.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scene.hpp"
#include "swarm/simulation.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// Builds the channel a simulation runs over, given the number of endpoints and the scenario's
// channel configuration with its seed already set. The core cannot include the network library,
// so the program that wants real sockets passes in a factory that makes one.
using ChannelFactory =
    std::function<std::unique_ptr<IChannel>(std::uint32_t endpoints, const ChannelConfig& config)>;

// A scenario turned into things that can run: the generated scene, the simulation configuration,
// and one agent configuration per drone. The world owns the scene, and every simulation made from
// it refers to that scene, so the world must outlive them.
struct ScenarioWorld {
  Scenario scenario;
  std::unique_ptr<Scene> scene;
  SimConfig sim;
  std::vector<Vec3> offsets;  // formation slot offsets, one per drone
  std::vector<DroneAgentConfig> agents;
  // Present only when the scenario has commands. The station is the last endpoint, after the
  // drones, so a scenario without commands has exactly one endpoint per drone, as before.
  std::optional<GroundStationConfig> station;

  // A fresh simulation at tick 0. `threads` never changes the results. With a factory, the
  // simulation runs over the channel it makes; without one, over the deterministic channel. Throws
  // std::invalid_argument if the factory returns no channel.
  std::unique_ptr<Simulation> make_simulation(std::size_t threads,
                                              const ChannelFactory& channel_factory = {}) const;
};

// Start positions: a grid of `start.columns` columns centered on the first waypoint, jittered. The
// jitter draws are made from the stream kStreamStart, three per drone, in drone order, whatever the
// jitter amounts are.
std::vector<Vec3> start_positions(const Scenario& scenario);

// Generates the scene (terrain, then obstacles, both from the scenario seed) and configures the
// simulation and the drones. Throws std::invalid_argument for a combination the loader cannot see
// on its own: formation slots not farther apart than the separation margin.
ScenarioWorld build_world(const Scenario& scenario);

}  // namespace swarm
