#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/agent.hpp"
#include "swarm/control/command.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct StationCommand {
  std::uint64_t tick = 0;  // the tick it is sent at
  CommandKind kind = CommandKind::kHold;
};

struct GroundStationConfig {
  std::uint32_t drones = 0;  // commands go to endpoints 0 to drones - 1
  std::vector<StationCommand> schedule;
  Vec3 position;  // where the station is, for the jamming zones
};

// An agent that sends scheduled commands to every drone. It is placed after the drones, so its
// endpoint is `drones`. It has no sensors and no state beyond how much of the schedule it has
// sent. Each command is sent once, as a reliable message, on its tick.
class GroundStationAgent : public IAgent {
 public:
  // Throws std::invalid_argument if the schedule is not in non-decreasing tick order.
  explicit GroundStationAgent(const GroundStationConfig& config);

  void step(AgentContext& ctx) override;
  Vec3 position() const override { return config_.position; }
  void hash_state(Fnv1a64& hash) const override;

  std::size_t sent() const { return next_; }

 private:
  GroundStationConfig config_;
  std::size_t next_ = 0;
};

}  // namespace swarm
