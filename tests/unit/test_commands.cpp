#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "swarm/agent.hpp"
#include "swarm/control/command.hpp"
#include "swarm/ground_station.hpp"
#include "swarm/simulation.hpp"

using swarm::Command;
using swarm::CommandKind;
using swarm::CommandStatus;

namespace {

using Bytes = std::vector<std::uint8_t>;

// An agent that records every message it is delivered, with the tick it arrived at.
class Recorder : public swarm::IAgent {
 public:
  struct Seen {
    std::uint64_t tick;
    swarm::Message message;
  };
  void step(swarm::AgentContext& ctx) override {
    for (const swarm::Message& m : ctx.inbox) {
      seen.push_back({ctx.tick, m});
    }
  }
  void hash_state(swarm::Fnv1a64& hash) const override { hash.update_u64(seen.size()); }
  std::vector<Seen> seen;
};

}  // namespace

TEST(Command, EncodesToTheDocumentedBytes) {
  const Bytes bytes = swarm::encode_command({CommandKind::kResume, 0x0807060504030201ULL});
  EXPECT_EQ(bytes, (Bytes{0x01, 0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}));
  EXPECT_EQ(bytes.size(), swarm::kCommandBytes);
}

TEST(Command, RoundTripsBothKindsAndExtremeTicks) {
  for (CommandKind kind : {CommandKind::kHold, CommandKind::kResume}) {
    for (std::uint64_t tick : {0ULL, 1ULL, 123456789ULL, 0xFFFFFFFFFFFFFFFFULL}) {
      const Bytes bytes = swarm::encode_command({kind, tick});
      Command out;
      ASSERT_EQ(swarm::decode_command(bytes.data(), bytes.size(), out), CommandStatus::kOk);
      EXPECT_EQ(out.kind, kind);
      EXPECT_EQ(out.issue_tick, tick);
    }
  }
}

TEST(Command, MalformedInputIsRejectedWithItsOwnStatusAndLeavesTheOutputAlone) {
  const Bytes good = swarm::encode_command({CommandKind::kHold, 5});
  Command out;
  out.kind = CommandKind::kResume;
  out.issue_tick = 77;

  for (std::size_t size = 0; size < good.size(); ++size) {
    const Bytes cut(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(size));
    EXPECT_EQ(swarm::decode_command(cut.data(), cut.size(), out), CommandStatus::kBadSize);
  }
  Bytes longer = good;
  longer.push_back(0);
  EXPECT_EQ(swarm::decode_command(longer.data(), longer.size(), out), CommandStatus::kBadSize);

  for (int version : {0, 2, 255}) {
    Bytes b = good;
    b[0] = static_cast<std::uint8_t>(version);
    EXPECT_EQ(swarm::decode_command(b.data(), b.size(), out), CommandStatus::kBadVersion);
  }
  for (int kind : {0, 3, 255}) {
    Bytes b = good;
    b[1] = static_cast<std::uint8_t>(kind);
    EXPECT_EQ(swarm::decode_command(b.data(), b.size(), out), CommandStatus::kBadKind);
  }
  EXPECT_EQ(out.kind, CommandKind::kResume);
  EXPECT_EQ(out.issue_tick, 77U);
}

TEST(Command, EncodingRejectsAnUnknownKind) {
  EXPECT_THROW(swarm::encode_command({static_cast<CommandKind>(0), 1}), std::invalid_argument);
  EXPECT_THROW(swarm::encode_command({static_cast<CommandKind>(9), 1}), std::invalid_argument);
}

TEST(GroundStation, SendsEachScheduledCommandOnceToEveryDroneOnItsTick) {
  swarm::GroundStationConfig config;
  config.drones = 3;
  config.schedule = {{5, CommandKind::kHold}, {5, CommandKind::kResume}, {9, CommandKind::kHold}};
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (int i = 0; i < 3; ++i) {
    agents.push_back(std::make_unique<Recorder>());
  }
  agents.push_back(std::make_unique<swarm::GroundStationAgent>(config));  // endpoint 3

  swarm::SimConfig sim_config;  // one tick of latency
  swarm::Simulation sim(sim_config, std::move(agents), 2);
  for (int t = 0; t < 30; ++t) {
    sim.step();
  }

  for (std::size_t d = 0; d < 3; ++d) {
    const auto& r = dynamic_cast<const Recorder&>(sim.agent(d));
    ASSERT_EQ(r.seen.size(), 3U) << "drone " << d;
    // Sent at tick 5 (two commands, in schedule order) and tick 9, delivered one tick later.
    EXPECT_EQ(r.seen[0].tick, 6U);
    EXPECT_EQ(r.seen[1].tick, 6U);
    EXPECT_EQ(r.seen[2].tick, 10U);
    Command c;
    for (std::size_t i = 0; i < 3; ++i) {
      const swarm::Message& m = r.seen[i].message;
      EXPECT_EQ(m.type, swarm::MessageType::kCommand);
      EXPECT_EQ(m.sender, 3U);
      EXPECT_EQ(m.receiver, d);
      ASSERT_EQ(swarm::decode_command(m.payload.data(), m.payload.size(), c), CommandStatus::kOk);
    }
    swarm::decode_command(r.seen[0].message.payload.data(), r.seen[0].message.payload.size(), c);
    EXPECT_EQ(c.kind, CommandKind::kHold);
    EXPECT_EQ(c.issue_tick, 5U);
    swarm::decode_command(r.seen[1].message.payload.data(), r.seen[1].message.payload.size(), c);
    EXPECT_EQ(c.kind, CommandKind::kResume);
    swarm::decode_command(r.seen[2].message.payload.data(), r.seen[2].message.payload.size(), c);
    EXPECT_EQ(c.kind, CommandKind::kHold);
    EXPECT_EQ(c.issue_tick, 9U);
  }
  EXPECT_EQ(dynamic_cast<const swarm::GroundStationAgent&>(sim.agent(3)).sent(), 3U);
}

TEST(GroundStation, ACommandScheduledBeforeTheFirstTickIsSentAtTheFirstStep) {
  swarm::GroundStationConfig config;
  config.drones = 1;
  config.schedule = {{0, CommandKind::kHold}};
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  agents.push_back(std::make_unique<Recorder>());
  agents.push_back(std::make_unique<swarm::GroundStationAgent>(config));
  swarm::Simulation sim(swarm::SimConfig{}, std::move(agents), 1);
  for (int t = 0; t < 5; ++t) {
    sim.step();
  }
  EXPECT_EQ(dynamic_cast<const Recorder&>(sim.agent(0)).seen.size(), 1U);
}

TEST(GroundStation, AnEmptyScheduleSendsNothingAndAnUnsortedOneIsRejected) {
  swarm::GroundStationConfig config;
  config.drones = 2;
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  agents.push_back(std::make_unique<Recorder>());
  agents.push_back(std::make_unique<Recorder>());
  agents.push_back(std::make_unique<swarm::GroundStationAgent>(config));
  swarm::Simulation sim(swarm::SimConfig{}, std::move(agents), 1);
  for (int t = 0; t < 10; ++t) {
    sim.step();
  }
  EXPECT_TRUE(dynamic_cast<const Recorder&>(sim.agent(0)).seen.empty());

  config.schedule = {{9, CommandKind::kHold}, {3, CommandKind::kResume}};
  EXPECT_THROW(swarm::GroundStationAgent{config}, std::invalid_argument);
}

TEST(GroundStation, ItsPositionIsWhereTheScenarioPutsItForJamming) {
  swarm::GroundStationConfig config;
  config.drones = 1;
  config.position = {1, 2, 3};
  EXPECT_EQ(swarm::GroundStationAgent(config).position(), (swarm::Vec3{1, 2, 3}));
}
