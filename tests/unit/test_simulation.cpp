#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "swarm/demo_agent.hpp"
#include "swarm/simulation.hpp"

namespace {

using AgentList = std::vector<std::unique_ptr<swarm::IAgent>>;

AgentList demo_agents(std::uint32_t n) {
  AgentList agents;
  for (std::uint32_t i = 0; i < n; ++i) {
    agents.push_back(std::make_unique<swarm::DemoAgent>(i, n));
  }
  return agents;
}

swarm::SimConfig harsh_config(std::uint64_t seed) {
  swarm::SimConfig c;
  c.seed = seed;
  c.channel.latency_ticks = 2;
  c.channel.jitter_ticks = 2;
  c.channel.loss_model = swarm::LossModel::kBurst;
  c.channel.loss = 0.2;
  c.channel.mean_burst = 4.0;
  c.channel.max_msgs_per_sender_per_tick = 3;
  return c;
}

std::vector<std::uint64_t> hashes(const swarm::SimConfig& config, std::uint32_t agents,
                                  std::size_t threads, int ticks) {
  swarm::Simulation sim(config, demo_agents(agents), threads);
  std::vector<std::uint64_t> out;
  for (int t = 0; t < ticks; ++t) {
    sim.step();
    out.push_back(sim.state_hash());
  }
  return out;
}

// Checks inbox ordering and sends a fixed message to agent 0 every tick.
class ProbeAgent : public swarm::IAgent {
 public:
  void step(swarm::AgentContext& ctx) override {
    for (std::size_t i = 1; i < ctx.inbox.size(); ++i) {
      const auto& a = ctx.inbox[i - 1];
      const auto& b = ctx.inbox[i];
      if (std::tie(a.deliver_tick, a.sender, a.seq) > std::tie(b.deliver_tick, b.sender, b.seq)) {
        ++order_violations;
      }
    }
    for (const auto& m : ctx.inbox) {
      if (m.deliver_tick > ctx.tick) {
        ++early_deliveries;
      }
    }
    swarm::Message m;
    m.receiver = 0;
    ctx.outbox.push_back(m);
  }
  void hash_state(swarm::Fnv1a64& hash) const override { hash.update_u64(order_violations); }
  std::uint64_t order_violations = 0;
  std::uint64_t early_deliveries = 0;
};

// Changes one bit of state at a chosen tick.
class PerturbAgent : public swarm::IAgent {
 public:
  explicit PerturbAgent(std::uint64_t perturb_at) : perturb_at_(perturb_at) {}
  void step(swarm::AgentContext& ctx) override {
    if (ctx.tick == perturb_at_) {
      state_ ^= 1U;
    }
  }
  void hash_state(swarm::Fnv1a64& hash) const override { hash.update_u64(state_); }

 private:
  std::uint64_t perturb_at_;
  std::uint64_t state_ = 0;
};

class ThrowingAgent : public swarm::IAgent {
 public:
  void step(swarm::AgentContext&) override { throw std::runtime_error("agent failed"); }
  void hash_state(swarm::Fnv1a64&) const override {}
};

class BadReceiverAgent : public swarm::IAgent {
 public:
  void step(swarm::AgentContext& ctx) override {
    swarm::Message m;
    m.receiver = 99;
    ctx.outbox.push_back(m);
  }
  void hash_state(swarm::Fnv1a64&) const override {}
};

}  // namespace

TEST(Simulation, HashesAreIdenticalForAnyThreadCount) {
  const auto reference = hashes(harsh_config(21), 16, 1, 200);
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(hashes(harsh_config(21), 16, threads, 200), reference) << "threads " << threads;
  }
}

TEST(Simulation, SameSeedReproducesDifferentSeedDiffers) {
  EXPECT_EQ(hashes(harsh_config(5), 8, 2, 100), hashes(harsh_config(5), 8, 2, 100));
  EXPECT_NE(hashes(harsh_config(5), 8, 2, 100).back(), hashes(harsh_config(6), 8, 2, 100).back());
}

TEST(Simulation, HashChangesEveryTick) {
  const auto h = hashes(harsh_config(1), 8, 2, 300);
  EXPECT_EQ(std::set<std::uint64_t>(h.begin(), h.end()).size(), h.size());
}

TEST(Simulation, ChannelSettingsAffectTheHash) {
  swarm::SimConfig lossless = harsh_config(3);
  lossless.channel.loss_model = swarm::LossModel::kNone;
  EXPECT_NE(hashes(harsh_config(3), 8, 1, 50).back(), hashes(lossless, 8, 1, 50).back());
}

TEST(Simulation, AgentStateChangeChangesTheHashFromThatTick) {
  const auto run = [](std::uint64_t perturb_at) {
    AgentList agents;
    agents.push_back(std::make_unique<PerturbAgent>(perturb_at));
    agents.push_back(std::make_unique<PerturbAgent>(1000000));
    swarm::Simulation sim(swarm::SimConfig{}, std::move(agents), 2);
    std::vector<std::uint64_t> out;
    for (int t = 0; t < 20; ++t) {
      sim.step();
      out.push_back(sim.state_hash());
    }
    return out;
  };
  const auto base = run(1000000);
  const auto perturbed = run(10);
  for (std::size_t t = 0; t < 10; ++t) {
    EXPECT_EQ(base[t], perturbed[t]) << "tick " << t;
  }
  for (std::size_t t = 10; t < 20; ++t) {
    EXPECT_NE(base[t], perturbed[t]) << "tick " << t;
  }
}

TEST(Simulation, MessagesArriveNoEarlierThanLatencyAllows) {
  swarm::SimConfig config;
  config.channel.latency_ticks = 3;
  swarm::Simulation sim(config, demo_agents(6), 4);
  for (int t = 0; t < 100; ++t) {
    sim.step();
  }
  std::uint64_t smallest = UINT64_MAX;
  for (std::size_t i = 0; i < sim.agent_count(); ++i) {
    const auto& agent = dynamic_cast<const swarm::DemoAgent&>(sim.agent(i));
    EXPECT_GT(agent.received(), 0U);
    EXPECT_EQ(agent.malformed(), 0U);
    EXPECT_GE(agent.min_delay(), 3U);
    smallest = std::min(smallest, agent.min_delay());
  }
  EXPECT_EQ(smallest, 3U);
}

TEST(Simulation, InboxesAreOrderedAndNeverEarly) {
  swarm::SimConfig config = harsh_config(8);
  AgentList agents;
  for (int i = 0; i < 6; ++i) {
    agents.push_back(std::make_unique<ProbeAgent>());
  }
  swarm::Simulation sim(config, std::move(agents), 4);
  for (int t = 0; t < 200; ++t) {
    sim.step();
  }
  const auto& probe = dynamic_cast<const ProbeAgent&>(sim.agent(0));
  EXPECT_EQ(probe.order_violations, 0U);
  EXPECT_EQ(probe.early_deliveries, 0U);
  EXPECT_GT(sim.channel().stats().delivered, 0U);
}

TEST(Simulation, TickAdvancesAndEmptyOrSingleAgentRunsWork) {
  swarm::Simulation empty(swarm::SimConfig{}, AgentList{}, 3);
  const std::uint64_t before = empty.state_hash();
  empty.step();
  empty.step();
  EXPECT_EQ(empty.tick(), 2U);
  EXPECT_NE(empty.state_hash(), before);

  swarm::Simulation single(swarm::SimConfig{}, demo_agents(1), 2);
  for (int t = 0; t < 20; ++t) {
    single.step();
  }
  const auto& agent = dynamic_cast<const swarm::DemoAgent&>(single.agent(0));
  EXPECT_GT(agent.received(), 0U);
}

TEST(Simulation, AgentExceptionsPropagateFromStep) {
  AgentList agents;
  agents.push_back(std::make_unique<ThrowingAgent>());
  swarm::Simulation sim(swarm::SimConfig{}, std::move(agents), 2);
  EXPECT_THROW(sim.step(), std::runtime_error);
}

TEST(Simulation, SendingToAnUnknownReceiverIsAnError) {
  AgentList agents;
  agents.push_back(std::make_unique<BadReceiverAgent>());
  swarm::Simulation sim(swarm::SimConfig{}, std::move(agents), 1);
  EXPECT_THROW(sim.step(), std::out_of_range);
}

TEST(Simulation, DemoAgentRejectsZeroAgents) {
  EXPECT_THROW(swarm::DemoAgent(0, 0), std::invalid_argument);
}
