#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/net/socket_channel.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::ChannelFactory;
using swarm::Scenario;
using swarm::net::SocketChannel;

namespace {

Scenario load(const std::string& name) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/" + name, std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok()) << name;
  return r.scenario;
}

// Four drones for six seconds with burst loss and a jam of every link from tick 100.
Scenario small() {
  Scenario s = load("formation_jam.yaml");
  s.drones = 4;
  s.duration_ticks = 300;
  return s;
}

// A factory for socket channels. `last` receives each channel it makes, so a test can read the
// channel's statistics while the simulation that owns it is alive.
ChannelFactory sockets(int timeout_ms = 3000, SocketChannel** last = nullptr) {
  return [timeout_ms, last](std::uint32_t endpoints, const swarm::ChannelConfig& config) {
    auto channel = std::make_unique<SocketChannel>(endpoints, config, timeout_ms);
    if (last != nullptr) {
      *last = channel.get();
    }
    return std::unique_ptr<swarm::IChannel>(std::move(channel));
  };
}

}  // namespace

TEST(ScenarioSockets, ARunOverRealSocketsWritesTheSameLogAsTheSimulatedRun) {
  const Scenario s = small();
  std::vector<std::uint8_t> simulated;
  std::uint64_t simulated_hash = 0;
  ASSERT_EQ(swarm::run_scenario(s, 2, simulated, &simulated_hash), swarm::LogStatus::kOk);

  std::vector<std::uint8_t> real;
  std::uint64_t real_hash = 0;
  swarm::ChannelStats stats;
  ASSERT_EQ(swarm::run_scenario(s, 2, real, &real_hash, sockets(), &stats), swarm::LogStatus::kOk);

  EXPECT_EQ(real, simulated);  // byte for byte: header, hashes, snapshots, message events
  EXPECT_EQ(real_hash, simulated_hash);
  EXPECT_EQ(stats.dropped_network, 0U);
  EXPECT_GT(stats.delivered, 0U);
  EXPECT_GT(stats.dropped_jam, 0U);
}

TEST(ScenarioSockets, ALogRecordedOverSocketsVerifiesOnTheSimulatedChannelOnAnyThreadCount) {
  const Scenario s = small();
  std::vector<std::uint8_t> real;
  ASSERT_EQ(swarm::run_scenario(s, 1, real, nullptr, sockets()), swarm::LogStatus::kOk);
  for (std::size_t threads : {1U, 3U, 8U}) {
    const swarm::VerifyResult v = swarm::verify_run_log(real.data(), real.size(), threads);
    EXPECT_EQ(v.status, swarm::VerifyStatus::kOk) << v.message << " threads " << threads;
  }
}

TEST(ScenarioSockets, TheThreadCountDoesNotChangeALogRecordedOverSockets) {
  const Scenario s = small();
  std::vector<std::uint8_t> one;
  std::vector<std::uint8_t> four;
  ASSERT_EQ(swarm::run_scenario(s, 1, one, nullptr, sockets()), swarm::LogStatus::kOk);
  ASSERT_EQ(swarm::run_scenario(s, 4, four, nullptr, sockets()), swarm::LogStatus::kOk);
  EXPECT_EQ(one, four);
}

TEST(ScenarioSockets, ACommandsScenarioSendsItsCommandsOverTcpAndItsBeaconsOverUdp) {
  Scenario s = load("hold_resume.yaml");
  s.duration_ticks = 1900;  // past the hold at tick 1500 and the resume at tick 1800

  // Drive the simulation directly, to read the channel's statistics while it is alive.
  {
    const swarm::ScenarioWorld world = swarm::build_world(s);
    SocketChannel* channel = nullptr;
    const auto sim = world.make_simulation(2, sockets(3000, &channel));
    ASSERT_NE(channel, nullptr);
    for (std::uint64_t tick = 0; tick < s.duration_ticks; ++tick) {
      sim->step();
    }
    const SocketChannel::NetStats net = channel->net_stats();
    // Two commands to each of six drones, over one connection per drone from the station.
    EXPECT_EQ(net.tcp.sent, 12U);
    EXPECT_EQ(net.tcp.connections_opened, 6U);
    EXPECT_EQ(net.tcp.received, 12U);
    EXPECT_GT(net.udp.sent, 1000U);  // and the beacons went as datagrams
    EXPECT_EQ(sim->channel().stats().dropped_network, 0U);
  }

  // And the whole log is the same as the simulated run's.
  std::vector<std::uint8_t> simulated;
  std::vector<std::uint8_t> real;
  ASSERT_EQ(swarm::run_scenario(s, 2, simulated), swarm::LogStatus::kOk);
  ASSERT_EQ(swarm::run_scenario(s, 2, real, nullptr, sockets()), swarm::LogStatus::kOk);
  EXPECT_EQ(real, simulated);
}

TEST(ScenarioSockets, ANetworkThatLosesMessagesGivesALogThatFailsVerification) {
  const Scenario s = small();
  const ChannelFactory lossy = [](std::uint32_t endpoints, const swarm::ChannelConfig& config) {
    auto channel = std::make_unique<SocketChannel>(endpoints, config, 30);
    channel->set_loss_in_transit([](const swarm::Message& m) { return m.seq % 50 == 7; });
    return std::unique_ptr<swarm::IChannel>(std::move(channel));
  };
  std::vector<std::uint8_t> log;
  swarm::ChannelStats stats;
  ASSERT_EQ(swarm::run_scenario(s, 2, log, nullptr, lossy, &stats), swarm::LogStatus::kOk);
  EXPECT_GT(stats.dropped_network, 0U);  // the run reports it

  // The simulated channel did not lose them, so re-execution disagrees with the log.
  const swarm::VerifyResult v = swarm::verify_run_log(log.data(), log.size(), 2);
  EXPECT_NE(v.status, swarm::VerifyStatus::kOk);
  EXPECT_TRUE(v.status == swarm::VerifyStatus::kHashMismatch ||
              v.status == swarm::VerifyStatus::kSnapshotMismatch)
      << swarm::to_string(v.status);
}

TEST(ScenarioSockets, AFactoryThatMakesNoChannelOrTheWrongOneIsRejected) {
  const Scenario s = small();
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const ChannelFactory none = [](std::uint32_t, const swarm::ChannelConfig&) {
    return std::unique_ptr<swarm::IChannel>();
  };
  EXPECT_THROW(world.make_simulation(1, none), std::invalid_argument);
  const ChannelFactory wrong_size = [](std::uint32_t endpoints, const swarm::ChannelConfig& c) {
    return std::unique_ptr<swarm::IChannel>(std::make_unique<SocketChannel>(endpoints + 1, c));
  };
  EXPECT_THROW(world.make_simulation(1, wrong_size), std::invalid_argument);
}

TEST(ScenarioSockets, TheFactoryIsGivenTheEndpointCountAndTheSeededChannelConfiguration) {
  Scenario s = load("hold_resume.yaml");  // six drones and a station
  const swarm::ScenarioWorld world = swarm::build_world(s);
  std::uint32_t seen_endpoints = 0;
  std::uint64_t seen_seed = 0;
  const ChannelFactory spy = [&](std::uint32_t endpoints, const swarm::ChannelConfig& config) {
    seen_endpoints = endpoints;
    seen_seed = config.seed;
    return std::unique_ptr<swarm::IChannel>(std::make_unique<SocketChannel>(endpoints, config));
  };
  const auto sim = world.make_simulation(1, spy);
  EXPECT_EQ(seen_endpoints, 7U);  // six drones and the station
  EXPECT_EQ(seen_seed, s.seed);   // seeded as the simulated channel would be
}
