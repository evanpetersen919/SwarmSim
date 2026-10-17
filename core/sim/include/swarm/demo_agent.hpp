#pragma once

#include <cstdint>

#include "swarm/agent.hpp"

namespace swarm {

// Placeholder agent used to exercise the loop, the channel, and the state hash before drone code
// exists. Each tick it folds received payloads into an accumulator and sends two messages: one to
// the next agent in a ring and one to a random agent. Replaced by the drone agent in phase 4.
class DemoAgent : public IAgent {
 public:
  // `num_agents` must be at least 1.
  DemoAgent(std::uint32_t id, std::uint32_t num_agents);

  void step(AgentContext& ctx) override;
  void hash_state(Fnv1a64& hash) const override;

  std::uint64_t received() const { return received_; }
  std::uint64_t malformed() const { return malformed_; }
  std::uint64_t accumulator() const { return accumulator_; }
  // Smallest and largest observed delay, in ticks, between a message's send and its receipt.
  // min_delay() is UINT64_MAX until something has been received.
  std::uint64_t min_delay() const { return min_delay_; }
  std::uint64_t max_delay() const { return max_delay_; }

 private:
  std::uint32_t id_;
  std::uint32_t num_agents_;
  std::uint64_t received_ = 0;
  std::uint64_t malformed_ = 0;
  std::uint64_t accumulator_ = 0;
  std::uint64_t min_delay_ = UINT64_MAX;
  std::uint64_t max_delay_ = 0;
};

}  // namespace swarm
