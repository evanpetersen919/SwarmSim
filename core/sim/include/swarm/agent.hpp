#pragma once

#include <cstdint>
#include <vector>

#include "swarm/format/hash.hpp"
#include "swarm/message.hpp"
#include "swarm/rng.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// Everything an agent may touch during one tick. Agents run in parallel, so an agent must read
// and write only its own state and this context.
struct AgentContext {
  std::uint64_t tick;
  double time;  // seconds
  std::uint32_t id;
  Rng& rng;                    // this agent's own stream
  std::vector<Message> inbox;  // delivered this tick, in delivery order
  // Messages to send. The simulation sets `sender` to this agent's id, so agents need not.
  std::vector<Message>& outbox;
};

class IAgent {
 public:
  virtual ~IAgent() = default;

  virtual void step(AgentContext& ctx) = 0;

  // Ground-truth position after the most recent step, used by the channel to decide which links
  // a jammer covers. Agents without a position can leave this at the origin.
  virtual Vec3 position() const { return Vec3{}; }

  // Folds everything that defines this agent's state into `hash`. Anything left out is not
  // covered by the determinism checks.
  virtual void hash_state(Fnv1a64& hash) const = 0;
};

}  // namespace swarm
