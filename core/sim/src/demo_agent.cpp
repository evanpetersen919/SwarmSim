#include "swarm/demo_agent.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace swarm {

namespace {

constexpr std::size_t kPayloadSize = 16;  // send tick, value; both little-endian u64

void put_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFU));
  }
}

std::uint64_t get_u64(const std::uint8_t* bytes) {
  std::uint64_t value = 0;
  for (int i = 0; i < 8; ++i) {
    value |= static_cast<std::uint64_t>(bytes[i]) << (8 * i);
  }
  return value;
}

}  // namespace

DemoAgent::DemoAgent(std::uint32_t id, std::uint32_t num_agents)
    : id_(id), num_agents_(num_agents) {
  if (num_agents < 1) {
    throw std::invalid_argument("DemoAgent: num_agents must be at least 1");
  }
}

void DemoAgent::step(AgentContext& ctx) {
  for (const Message& msg : ctx.inbox) {
    if (msg.payload.size() != kPayloadSize) {
      ++malformed_;
      continue;
    }
    const std::uint64_t sent_tick = get_u64(msg.payload.data());
    const std::uint64_t value = get_u64(msg.payload.data() + 8);
    const std::uint64_t delay = ctx.tick - sent_tick;
    min_delay_ = std::min(min_delay_, delay);
    max_delay_ = std::max(max_delay_, delay);
    accumulator_ = accumulator_ * 6364136223846793005ULL + value;
    ++received_;
  }

  const std::uint64_t value =
      (static_cast<std::uint64_t>(ctx.rng.next_u32()) << 32) | ctx.rng.next_u32();

  Message ring;
  ring.receiver = (id_ + 1U) % num_agents_;
  put_u64(ring.payload, ctx.tick);
  put_u64(ring.payload, value);
  ctx.outbox.push_back(std::move(ring));

  Message random;
  random.receiver = ctx.rng.below(num_agents_);
  put_u64(random.payload, ctx.tick);
  put_u64(random.payload, value ^ 0x9E3779B97F4A7C15ULL);
  ctx.outbox.push_back(std::move(random));
}

void DemoAgent::hash_state(Fnv1a64& hash) const {
  hash.update_u64(id_);
  hash.update_u64(received_);
  hash.update_u64(malformed_);
  hash.update_u64(accumulator_);
  hash.update_u64(min_delay_);
  hash.update_u64(max_delay_);
}

}  // namespace swarm
