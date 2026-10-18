#pragma once

#include <cstdint>
#include <vector>

#include "swarm/format/hash.hpp"
#include "swarm/message.hpp"
#include "swarm/rng.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// The true position and velocity of one agent at the end of the previous tick (at tick 0, the
// initial state).
struct PeerTruth {
  Vec3 position;
  Vec3 velocity;
};

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
  // Ground truth of every agent, indexed by agent id, as of the end of the previous tick. It exists
  // for the oracle baseline, which is allowed to know the truth by definition. An agent that is
  // not an oracle must not read it. The simulation updates it between ticks, never during one, so
  // reading it from the parallel section is safe.
  const std::vector<PeerTruth>& peers;
};

class IAgent {
 public:
  virtual ~IAgent() = default;

  virtual void step(AgentContext& ctx) = 0;

  // Ground-truth position after the most recent step, used by the channel to decide which links
  // a jammer covers. Agents without a position can leave this at the origin.
  virtual Vec3 position() const { return Vec3{}; }

  // Ground-truth velocity after the most recent step, published to the others as part of `peers`.
  virtual Vec3 velocity() const { return Vec3{}; }

  // Folds everything that defines this agent's state into `hash`. Anything left out is not
  // covered by the determinism checks.
  virtual void hash_state(Fnv1a64& hash) const = 0;
};

}  // namespace swarm
