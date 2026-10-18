#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "swarm/drone_agent.hpp"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/run.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::LogReader;
using swarm::LogRecord;
using swarm::LogStatus;
using swarm::Scenario;
using swarm::Snapshot;
using swarm::SnapshotEventKind;
using swarm::SnapshotStatus;
using swarm::VerifyStatus;

namespace {

using Bytes = std::vector<std::uint8_t>;

constexpr std::uint64_t kTicks = 200;

// Four drones for four seconds, with burst loss and a jam of every link from tick 25 to tick 75.
Scenario small_scenario() {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/formation_jam.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  Scenario s = r.scenario;
  s.drones = 4;
  s.duration_ticks = kTicks;
  s.channel.jammers[0].start_tick = 25;
  s.channel.jammers[0].end_tick = 75;
  return s;
}

Bytes record(const Scenario& s, std::size_t threads, std::uint64_t* hash = nullptr) {
  Bytes bytes;
  EXPECT_EQ(swarm::run_scenario(s, threads, bytes, hash), LogStatus::kOk);
  return bytes;
}

struct Parsed {
  swarm::LogHeader header;
  std::vector<LogRecord> records;
  std::vector<Snapshot> snapshots;
};

Parsed parse(const Bytes& bytes) {
  Parsed p;
  LogReader reader(bytes.data(), bytes.size());
  EXPECT_EQ(reader.read_header(), LogStatus::kOk);
  p.header = reader.header();
  LogRecord rec;
  while (reader.next(rec) == LogStatus::kOk) {
    Snapshot snap;
    EXPECT_EQ(swarm::decode_snapshot(rec.blob.data(), rec.blob.size(), snap), SnapshotStatus::kOk)
        << "tick " << rec.tick;
    p.records.push_back(rec);
    p.snapshots.push_back(snap);
  }
  return p;
}

}  // namespace

TEST(ScenarioRun, TheLogHeaderHoldsTheScenarioAndEveryTickHasADecodableRecord) {
  const Scenario s = small_scenario();
  const Bytes bytes = record(s, 2);
  const Parsed p = parse(bytes);

  EXPECT_EQ(p.header.seed, s.seed);
  EXPECT_DOUBLE_EQ(p.header.dt, s.dt);
  EXPECT_EQ(p.header.agent_count, 4U);
  EXPECT_TRUE(swarm::is_scenario_log(p.header));
  EXPECT_EQ(p.header.config, std::string(swarm::kScenarioLogTag) + swarm::canonical_text(s));
  ASSERT_EQ(p.records.size(), kTicks);
  for (std::size_t i = 0; i < p.records.size(); ++i) {
    EXPECT_EQ(p.records[i].tick, i);
    ASSERT_EQ(p.snapshots[i].agents.size(), 4U) << "tick " << i;  // a stride of 1
  }
}

TEST(ScenarioRun, SnapshotsHoldTheTrueStateAndTheFallbackStateOfEveryTick) {
  const Scenario s = small_scenario();
  const Parsed p = parse(record(s, 1));

  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  bool saw_degraded = false;
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
    ASSERT_EQ(p.records[tick].hash, sim->state_hash());
    for (std::size_t i = 0; i < 4; ++i) {
      const auto& drone = dynamic_cast<const swarm::DroneAgent&>(sim->agent(i));
      const swarm::AgentSnapshot& a = p.snapshots[tick].agents[i];
      ASSERT_EQ(a.position[0], drone.truth().pos.x) << "tick " << tick << " drone " << i;
      ASSERT_EQ(a.position[1], drone.truth().pos.y);
      ASSERT_EQ(a.position[2], drone.truth().pos.z);
      ASSERT_EQ(a.velocity[0], drone.truth().vel.x);
      ASSERT_EQ(a.fallback_state, static_cast<std::uint8_t>(drone.brain().state()));
      ASSERT_TRUE(a.alive);
      saw_degraded = saw_degraded || a.fallback_state != 0;
    }
  }
  EXPECT_TRUE(saw_degraded);  // the jam lasts one second, longer than the staleness limit
}

TEST(ScenarioRun, TheLogBytesAndHashesAreIdenticalForEveryThreadCount) {
  const Scenario s = small_scenario();
  std::uint64_t hash1 = 0;
  const Bytes reference = record(s, 1, &hash1);
  EXPECT_NE(hash1, 0U);
  for (std::size_t threads : {2U, 4U, 8U}) {
    std::uint64_t hash = 0;
    EXPECT_EQ(record(s, threads, &hash), reference) << "threads " << threads;
    EXPECT_EQ(hash, hash1);
  }
}

TEST(ScenarioRun, ARecordedLogVerifiesOnAnyThreadCount) {
  const Scenario s = small_scenario();
  const Bytes bytes = record(s, 1);
  for (std::size_t threads : {1U, 3U, 8U}) {
    const swarm::VerifyResult r = swarm::verify_run_log(bytes.data(), bytes.size(), threads);
    EXPECT_EQ(r.status, VerifyStatus::kOk) << r.message << " threads " << threads;
  }
}

TEST(ScenarioRun, OneCorruptedSnapshotByteIsFoundAtItsTick) {
  const Scenario s = small_scenario();
  Bytes bytes = record(s, 1);

  // Find the record at tick 100 and flip a bit in the middle of its blob.
  LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), LogStatus::kOk);
  LogRecord rec;
  const std::uint8_t* blob_start = nullptr;
  while (reader.next(rec) == LogStatus::kOk && rec.tick < 100) {
  }
  ASSERT_EQ(rec.tick, 100U);
  // Locate the blob's bytes inside the log buffer by searching for its content.
  const auto at = std::search(bytes.begin(), bytes.end(), rec.blob.begin(), rec.blob.end());
  ASSERT_NE(at, bytes.end());
  blob_start = &*at;
  bytes[static_cast<std::size_t>(blob_start - bytes.data()) + 20] ^= 0x01;

  const swarm::VerifyResult r = swarm::verify_run_log(bytes.data(), bytes.size(), 2);
  EXPECT_EQ(r.status, VerifyStatus::kSnapshotMismatch);
  EXPECT_EQ(r.tick, 100U);
  EXPECT_NE(r.message.find("tick 100"), std::string::npos);
}

TEST(ScenarioRun, ACorruptedHashIsFoundToo) {
  const Scenario s = small_scenario();
  Bytes bytes = record(s, 1);
  LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), LogStatus::kOk);
  LogRecord rec;
  while (reader.next(rec) == LogStatus::kOk && rec.tick < 50) {
  }
  // The record layout is u8 1 | u64 tick | u64 hash | u32 len | blob; find "tick" then flip the
  // hash.
  Bytes pattern = {1};
  for (int i = 0; i < 8; ++i) {
    pattern.push_back(static_cast<std::uint8_t>((rec.tick >> (8 * i)) & 0xFFU));
  }
  const auto at = std::search(bytes.begin(), bytes.end(), pattern.begin(), pattern.end());
  ASSERT_NE(at, bytes.end());
  bytes[static_cast<std::size_t>(at - bytes.begin()) + 9] ^= 0x80;  // the first hash byte
  const swarm::VerifyResult r = swarm::verify_run_log(bytes.data(), bytes.size(), 1);
  // The log container does not checksum records, so the mismatch shows up in re-execution.
  EXPECT_EQ(r.status, VerifyStatus::kHashMismatch);
  EXPECT_EQ(r.tick, 50U);
}

TEST(ScenarioRun, AHeaderThatDisagreesWithTheScenarioInsideItIsRejected) {
  const Scenario s = small_scenario();
  Bytes bytes = record(s, 1);
  bytes[8] ^= 0x01;  // the seed's lowest byte: magic (4) and version (4) come first
  const swarm::VerifyResult r = swarm::verify_run_log(bytes.data(), bytes.size(), 1);
  EXPECT_EQ(r.status, VerifyStatus::kBadConfig);
}

TEST(ScenarioRun, DemoLogsStillVerifyThroughTheSameEntryPoint) {
  swarm::RunConfig config;
  config.agents = 6;
  config.ticks = 100;
  config.sim.seed = 3;
  config.sim.channel.loss_model = swarm::LossModel::kIndependent;
  config.sim.channel.loss = 0.2;
  Bytes bytes;
  ASSERT_EQ(swarm::record_run(config, 1, bytes), LogStatus::kOk);
  EXPECT_EQ(swarm::verify_run_log(bytes.data(), bytes.size(), 4).status, VerifyStatus::kOk);
  EXPECT_EQ(swarm::verify_log(bytes.data(), bytes.size(), 4).status, VerifyStatus::kOk);

  // And a scenario log is not a demo log: the demo verifier rejects its configuration.
  const Bytes scenario_log = record(small_scenario(), 1);
  EXPECT_EQ(swarm::verify_log(scenario_log.data(), scenario_log.size(), 1).status,
            VerifyStatus::kBadConfig);
}

TEST(ScenarioRun, AStrideSavesAgentStateOnlyOnItsTicksButNeverLosesAMessageEvent) {
  Scenario s = small_scenario();
  const Bytes every = record(s, 1);
  s.snapshot_every_ticks = 10;
  const Bytes strided = record(s, 1);
  const Parsed full = parse(every);
  const Parsed some = parse(strided);
  ASSERT_EQ(some.records.size(), kTicks);

  std::size_t with_agents = 0;
  for (std::size_t i = 0; i < kTicks; ++i) {
    const bool expected = i % 10 == 0 || i == kTicks - 1;
    EXPECT_EQ(!some.snapshots[i].agents.empty(), expected) << "tick " << i;
    with_agents += expected ? 1 : 0;
    // Messages are recorded on every tick whatever the stride is.
    ASSERT_EQ(some.snapshots[i].events.size(), full.snapshots[i].events.size()) << "tick " << i;
    // The state, where present, is the same.
    if (expected) {
      EXPECT_EQ(some.snapshots[i].agents.size(), 4U);
      EXPECT_EQ(some.snapshots[i].agents[0].position, full.snapshots[i].agents[0].position);
    }
  }
  EXPECT_EQ(with_agents, 21U);
  EXPECT_LT(strided.size(), every.size());
  EXPECT_EQ(swarm::verify_run_log(strided.data(), strided.size(), 2).status, VerifyStatus::kOk);
}

TEST(ScenarioRun, MessageEventsMatchTheChannelAndJamDropsFallInsideTheWindow) {
  const Scenario s = small_scenario();
  const Parsed p = parse(record(s, 1));

  std::uint64_t total = 0;
  std::uint64_t jam_drops = 0;
  for (std::size_t tick = 0; tick < p.snapshots.size(); ++tick) {
    for (const swarm::SnapshotEvent& e : p.snapshots[tick].events) {
      ++total;
      EXPECT_EQ(e.send_tick, tick);
      EXPECT_LT(e.sender, 4U);
      EXPECT_LT(e.receiver, 4U);
      if (e.kind == SnapshotEventKind::kDroppedJam) {
        ++jam_drops;
        EXPECT_GE(tick, 25U);
        EXPECT_LT(tick, 75U);
        EXPECT_EQ(e.deliver_tick, 0U);
      }
      if (e.kind == SnapshotEventKind::kScheduled) {
        EXPECT_GE(e.deliver_tick, e.send_tick + 2);
      }
    }
  }

  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
  }
  EXPECT_EQ(total, sim->channel().stats().sent);
  EXPECT_EQ(jam_drops, sim->channel().stats().dropped_jam);
  EXPECT_GT(jam_drops, 0U);
}

TEST(ScenarioRun, ANonScenarioConfigIsRejectedAndAnInvalidScenarioThrows) {
  swarm::LogWriter writer;
  swarm::LogHeader header;
  header.config = "not a scenario";
  ASSERT_EQ(writer.begin(header), LogStatus::kOk);
  ASSERT_EQ(writer.finish(), LogStatus::kOk);
  EXPECT_EQ(swarm::verify_scenario_log(writer.bytes().data(), writer.bytes().size(), 1).status,
            VerifyStatus::kBadConfig);

  Scenario bad = small_scenario();
  bad.formation_spacing_m = 5.0;
  Bytes bytes;
  EXPECT_THROW(swarm::run_scenario(bad, 1, bytes), std::invalid_argument);
}
