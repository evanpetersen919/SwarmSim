#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "swarm/agent.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/lockstep_pool.hpp"
#include "swarm/message.hpp"
#include "swarm/rng.hpp"
#include "swarm/sim_clock.hpp"

namespace swarm {

struct SimConfig {
  double dt = 0.01;
  std::uint64_t seed = 0;
  // `channel.seed` is ignored; the channel always uses `seed` above.
  ChannelConfig channel;
};

// The lockstep loop. One call to step() is one tick:
//   1. Deliver due messages into inboxes (single thread).
//   2. Step every agent in parallel on the pool; the pool's return is the barrier.
//   3. Send each agent's outbox through the channel, in agent id order (single thread).
//   4. Fold the tick and all agent state into the running state hash.
// Nothing that depends on order or randomness runs in the parallel section, so the hash does not
// depend on the number of threads.
//
// If an agent throws, step() rethrows and the simulation must be discarded.
class Simulation {
 public:
  Simulation(const SimConfig& config, std::vector<std::unique_ptr<IAgent>> agents,
             std::size_t threads);

  void step();

  std::uint64_t tick() const { return clock_.tick(); }
  // Running hash after the most recent step(), or the initial value before the first one.
  std::uint64_t state_hash() const { return hash_; }
  std::size_t agent_count() const { return agents_.size(); }
  const IAgent& agent(std::size_t index) const { return *agents_[index]; }
  const IChannel& channel() const { return channel_; }
  // See DeterministicChannel::set_event_sink. Set it before the first step.
  void set_event_sink(std::vector<ChannelEvent>* sink) { channel_.set_event_sink(sink); }

 private:
  SimConfig config_;
  SimClock clock_;
  DeterministicChannel channel_;
  LockstepPool pool_;
  std::vector<std::unique_ptr<IAgent>> agents_;
  std::vector<Rng> rngs_;
  std::vector<std::vector<Message>> outboxes_;
  std::vector<Vec3> positions_;
  std::vector<PeerTruth> peers_;
  std::uint64_t hash_ = 0;
};

}  // namespace swarm
