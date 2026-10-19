#include "swarm/ground_station.hpp"

#include <stdexcept>
#include <utility>

namespace swarm {

GroundStationAgent::GroundStationAgent(const GroundStationConfig& config) : config_(config) {
  for (std::size_t i = 1; i < config_.schedule.size(); ++i) {
    if (config_.schedule[i].tick < config_.schedule[i - 1].tick) {
      throw std::invalid_argument("GroundStationConfig: the schedule must be in tick order");
    }
  }
}

void GroundStationAgent::step(AgentContext& ctx) {
  while (next_ < config_.schedule.size() && config_.schedule[next_].tick <= ctx.tick) {
    const StationCommand& entry = config_.schedule[next_];
    Message m;
    m.type = MessageType::kCommand;
    m.payload = encode_command({entry.kind, ctx.tick});
    for (std::uint32_t drone = 0; drone < config_.drones; ++drone) {
      m.receiver = drone;
      ctx.outbox.push_back(m);
    }
    ++next_;
  }
}

void GroundStationAgent::hash_state(Fnv1a64& hash) const { hash.update_u64(next_); }

}  // namespace swarm
