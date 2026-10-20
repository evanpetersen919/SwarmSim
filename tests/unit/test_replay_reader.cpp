#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/format/replay.hpp"
#include "swarm/format/scene.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

using swarm::AgentSnapshot;
using swarm::Replay;
using swarm::ReplayStatus;
using swarm::Snapshot;

namespace {

using Bytes = std::vector<std::uint8_t>;

constexpr double kDt = 0.5;

struct HandFrame {
  std::uint64_t tick = 0;
  Snapshot snapshot;
};

AgentSnapshot agent(double x, double y, double z, double vx, double vy, double vz,
                    bool alive = true, std::uint8_t fallback = 0) {
  AgentSnapshot a;
  a.alive = alive;
  a.fallback_state = fallback;
  a.position = {x, y, z};
  a.velocity = {vx, vy, vz};
  return a;
}

Bytes make_log(const std::vector<HandFrame>& frames, std::uint32_t agents,
               const std::vector<Bytes>* raw_blobs = nullptr) {
  swarm::LogHeader header;
  header.seed = 5;
  header.dt = kDt;
  header.agent_count = agents;
  header.config = "hand built";
  swarm::LogWriter writer;
  EXPECT_EQ(writer.begin(header), swarm::LogStatus::kOk);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    swarm::LogRecord record;
    record.tick = frames[i].tick;
    record.hash = 1000 + frames[i].tick;
    if (raw_blobs != nullptr) {
      record.blob = (*raw_blobs)[i];
    } else {
      EXPECT_EQ(swarm::encode_snapshot(frames[i].snapshot, record.blob),
                swarm::SnapshotStatus::kOk);
    }
    EXPECT_EQ(writer.append(record), swarm::LogStatus::kOk);
  }
  EXPECT_EQ(writer.finish(), swarm::LogStatus::kOk);
  return writer.bytes();
}

Snapshot with(std::vector<AgentSnapshot> agents) {
  Snapshot s;
  s.agents = std::move(agents);
  return s;
}

ReplayStatus load(Replay& replay, const Bytes& bytes) {
  return replay.load(bytes.data(), bytes.size());
}

// One drone moving on a cubic x(t) = t^3 (seconds), logged at ticks 0, 4 and 8 (dt 0.5, so at
// t = 0, 2 and 4 s). A cubic Hermite curve reproduces a cubic exactly.
Bytes cubic_log() {
  std::vector<HandFrame> frames;
  for (std::uint64_t tick : {0U, 4U, 8U}) {
    const double t = static_cast<double>(tick) * kDt;
    frames.push_back({tick, with({agent(t * t * t, 2.0 * t, 10.0, 3.0 * t * t, 2.0, 0.0)})});
  }
  return make_log(frames, 1);
}

}  // namespace

TEST(ReplayReader, ALogOfFramesLoadsWithItsHeaderFramesAndTrailer) {
  std::vector<HandFrame> frames;
  for (std::uint64_t tick = 0; tick < 5; ++tick) {
    frames.push_back({tick, with({agent(static_cast<double>(tick), 0, 0, 1, 0, 0),
                                  agent(0, static_cast<double>(tick), 0, 0, 1, 0)})});
  }
  const Bytes bytes = make_log(frames, 2);
  Replay replay;
  ASSERT_EQ(load(replay, bytes), ReplayStatus::kOk);
  EXPECT_EQ(replay.header().agent_count, 2U);
  EXPECT_EQ(replay.header().seed, 5U);
  EXPECT_EQ(replay.header().dt, kDt);
  EXPECT_EQ(replay.header().config, "hand built");
  ASSERT_EQ(replay.frames().size(), 5U);
  EXPECT_EQ(replay.state_frame_count(), 5U);
  EXPECT_EQ(replay.first_tick(), 0U);
  EXPECT_EQ(replay.last_tick(), 4U);
  EXPECT_EQ(replay.trailer().record_count, 5U);
  EXPECT_EQ(replay.trailer().final_hash, replay.frames().back().hash);
  EXPECT_EQ(replay.frames()[3].hash, 1003U);
  EXPECT_EQ(replay.frames()[3].snapshot.agents[1].position[1], 3.0);
}

TEST(ReplayReader, FramesWithoutDroneStateAreKeptAndCarryTheirEvents) {
  Snapshot with_state = with({agent(0, 0, 0, 1, 0, 0)});
  Snapshot only_events;
  swarm::SnapshotEvent event;
  event.sender = 3;
  event.receiver = 4;
  event.send_tick = 1;
  event.deliver_tick = 2;
  only_events.events.push_back(event);
  const Bytes bytes = make_log({{0, with_state}, {1, only_events}, {2, with_state}}, 1);
  Replay replay;
  ASSERT_EQ(load(replay, bytes), ReplayStatus::kOk);
  ASSERT_EQ(replay.frames().size(), 3U);
  EXPECT_EQ(replay.state_frame_count(), 2U);
  EXPECT_FALSE(replay.frames()[1].has_state());
  ASSERT_EQ(replay.frames()[1].snapshot.events.size(), 1U);
  EXPECT_EQ(replay.frames()[1].snapshot.events[0].receiver, 4U);
}

TEST(ReplayReader, ARecordWithAnEmptyBlobIsAFrameWithNoState) {
  const Snapshot state = with({agent(0, 0, 0, 0, 0, 0)});
  Bytes encoded;
  ASSERT_EQ(swarm::encode_snapshot(state, encoded), swarm::SnapshotStatus::kOk);
  const std::vector<Bytes> blobs = {encoded, Bytes{}, encoded};
  const Bytes bytes = make_log({{0, {}}, {1, {}}, {2, {}}}, 1, &blobs);
  Replay replay;
  ASSERT_EQ(load(replay, bytes), ReplayStatus::kOk);
  EXPECT_EQ(replay.state_frame_count(), 2U);
  EXPECT_FALSE(replay.frames()[1].has_state());
}

TEST(ReplayReader, ALogWithNoDroneStateIsReported) {
  Replay replay;
  EXPECT_EQ(load(replay, make_log({{0, {}}, {1, {}}}, 1)), ReplayStatus::kNoState);
  EXPECT_TRUE(replay.frames().empty());
  EXPECT_EQ(load(replay, make_log({}, 1)), ReplayStatus::kNoState);  // a log with no records
}

TEST(ReplayReader, EveryTruncationOfALogIsRejectedWithoutACrash) {
  const Bytes bytes = cubic_log();
  for (std::size_t length = 0; length < bytes.size(); ++length) {
    Replay replay;
    const ReplayStatus status = replay.load(bytes.data(), length);
    EXPECT_NE(status, ReplayStatus::kOk) << "length " << length;
    EXPECT_TRUE(replay.frames().empty()) << "length " << length;
    EXPECT_EQ(replay.state_frame_count(), 0U);
  }
  Replay whole;
  EXPECT_EQ(load(whole, bytes), ReplayStatus::kOk);
}

TEST(ReplayReader, ABadContainerReportsTheLogStatus) {
  Bytes bytes = cubic_log();
  bytes[0] = 'X';
  Replay replay;
  EXPECT_EQ(load(replay, bytes), ReplayStatus::kBadLog);
  EXPECT_EQ(replay.log_status(), swarm::LogStatus::kBadMagic);

  Bytes trailing = cubic_log();
  trailing.back() ^= 0xFF;  // damages the trailer's final hash
  EXPECT_EQ(load(replay, trailing), ReplayStatus::kBadLog);
  EXPECT_EQ(replay.log_status(), swarm::LogStatus::kTrailerHashMismatch);
}

TEST(ReplayReader, ASnapshotThatDoesNotDecodeNamesTheRecord) {
  Bytes good;
  ASSERT_EQ(swarm::encode_snapshot(with({agent(0, 0, 0, 0, 0, 0)}), good),
            swarm::SnapshotStatus::kOk);
  Bytes bad = good;
  bad[0] = 9;  // an unsupported snapshot version
  const std::vector<Bytes> blobs = {good, bad, good};
  Replay replay;
  EXPECT_EQ(load(replay, make_log({{0, {}}, {1, {}}, {2, {}}}, 1, &blobs)),
            ReplayStatus::kBadSnapshot);
  EXPECT_EQ(replay.snapshot_status(), swarm::SnapshotStatus::kUnsupportedVersion);
  EXPECT_EQ(replay.failed_record(), 1U);
}

TEST(ReplayReader, TicksThatDoNotIncreaseAreRejected) {
  const Snapshot s = with({agent(0, 0, 0, 0, 0, 0)});
  Replay replay;
  EXPECT_EQ(load(replay, make_log({{0, s}, {2, s}, {2, s}}, 1)), ReplayStatus::kTickOrder);
  EXPECT_EQ(replay.failed_record(), 2U);
  EXPECT_EQ(load(replay, make_log({{3, s}, {1, s}}, 1)), ReplayStatus::kTickOrder);
}

TEST(ReplayReader, ASnapshotWithTheWrongDroneCountIsRejected) {
  const Snapshot one = with({agent(0, 0, 0, 0, 0, 0)});
  const Snapshot two = with({agent(0, 0, 0, 0, 0, 0), agent(1, 0, 0, 0, 0, 0)});
  Replay replay;
  EXPECT_EQ(load(replay, make_log({{0, one}, {1, two}}, 1)), ReplayStatus::kAgentCountMismatch);
  EXPECT_EQ(replay.failed_record(), 1U);
}

TEST(ReplayReader, ALoadAfterAFailureStartsClean) {
  Replay replay;
  ASSERT_EQ(load(replay, cubic_log()), ReplayStatus::kOk);
  Bytes bad = cubic_log();
  bad[0] = 'X';
  EXPECT_NE(load(replay, bad), ReplayStatus::kOk);
  EXPECT_TRUE(replay.frames().empty());
  EXPECT_EQ(replay.header().agent_count, 0U);
  ASSERT_EQ(load(replay, cubic_log()), ReplayStatus::kOk);
  EXPECT_EQ(replay.frames().size(), 3U);
}

TEST(ReplayReader, FramesAreFoundByTickAndByTickOrEarlier) {
  const Snapshot s = with({agent(0, 0, 0, 0, 0, 0)});
  Replay replay;
  ASSERT_EQ(load(replay, make_log({{2, s}, {5, s}, {9, s}}, 1)), ReplayStatus::kOk);
  ASSERT_NE(replay.frame_at(5), nullptr);
  EXPECT_EQ(replay.frame_at(5)->hash, 1005U);
  EXPECT_EQ(replay.frame_at(6), nullptr);
  EXPECT_EQ(replay.frame_at(0), nullptr);
  EXPECT_EQ(replay.frame_at(10), nullptr);

  EXPECT_EQ(replay.frame_at_or_before(0.0)->tick, 2U);  // before the first: the first
  EXPECT_EQ(replay.frame_at_or_before(2.0)->tick, 2U);
  EXPECT_EQ(replay.frame_at_or_before(4.99)->tick, 2U);
  EXPECT_EQ(replay.frame_at_or_before(5.0)->tick, 5U);
  EXPECT_EQ(replay.frame_at_or_before(8.5)->tick, 5U);
  EXPECT_EQ(replay.frame_at_or_before(9.0)->tick, 9U);
  EXPECT_EQ(replay.frame_at_or_before(1e9)->tick, 9U);

  const Replay empty;
  EXPECT_EQ(empty.frame_at_or_before(1.0), nullptr);
  EXPECT_EQ(empty.frame_at(0), nullptr);
}

TEST(ReplayReaderSample, ALoggedTickIsReturnedExactlyWhetherFirstMiddleOrLast) {
  Replay replay;
  ASSERT_EQ(load(replay, cubic_log()), ReplayStatus::kOk);
  for (std::uint64_t tick : {0U, 4U, 8U}) {
    std::vector<AgentSnapshot> out;
    ASSERT_TRUE(replay.sample(static_cast<double>(tick), out));
    const AgentSnapshot& logged = replay.frame_at(tick)->snapshot.agents[0];
    ASSERT_EQ(out.size(), 1U);
    EXPECT_EQ(out[0].position, logged.position) << tick;  // bit for bit
    EXPECT_EQ(out[0].velocity, logged.velocity) << tick;
  }
}

TEST(ReplayReaderSample, ATimeOutsideTheLoggedRangeGivesTheFirstOrLastState) {
  Replay replay;
  ASSERT_EQ(load(replay, cubic_log()), ReplayStatus::kOk);
  std::vector<AgentSnapshot> out;
  ASSERT_TRUE(replay.sample(-5.0, out));
  EXPECT_EQ(out[0].position, replay.frames().front().snapshot.agents[0].position);
  ASSERT_TRUE(replay.sample(100.0, out));
  EXPECT_EQ(out[0].position, replay.frames().back().snapshot.agents[0].position);
  ASSERT_TRUE(replay.sample(std::nan(""), out));  // not a number is treated as before the start
  EXPECT_EQ(out[0].position, replay.frames().front().snapshot.agents[0].position);
}

TEST(ReplayReaderSample, ACubicPathIsReproducedExactlyBetweenFramesWithItsVelocity) {
  Replay replay;
  ASSERT_EQ(load(replay, cubic_log()), ReplayStatus::kOk);
  std::vector<AgentSnapshot> out;
  for (double tick : {0.5, 1.0, 2.25, 3.0, 3.75, 4.5, 6.0, 7.5}) {
    ASSERT_TRUE(replay.sample(tick, out));
    const double t = tick * kDt;
    EXPECT_NEAR(out[0].position[0], t * t * t, 1e-9) << tick;
    EXPECT_NEAR(out[0].position[1], 2.0 * t, 1e-9) << tick;
    EXPECT_NEAR(out[0].position[2], 10.0, 1e-9) << tick;
    EXPECT_NEAR(out[0].velocity[0], 3.0 * t * t, 1e-9) << tick;
    EXPECT_NEAR(out[0].velocity[1], 2.0, 1e-9) << tick;
  }
}

TEST(ReplayReaderSample, ConstantVelocityFlightIsAStraightLineWithNoOvershoot) {
  std::vector<HandFrame> frames;
  for (std::uint64_t tick = 0; tick <= 20; tick += 10) {
    const double t = static_cast<double>(tick) * kDt;
    frames.push_back({tick, with({agent(5.0 * t, -2.0 * t, 30.0, 5.0, -2.0, 0.0)})});
  }
  Replay replay;
  ASSERT_EQ(load(replay, make_log(frames, 1)), ReplayStatus::kOk);
  double previous_x = -1.0;
  std::vector<AgentSnapshot> out;
  for (int i = 0; i <= 200; ++i) {
    const double tick = 0.1 * i;
    ASSERT_TRUE(replay.sample(tick, out));
    const double t = tick * kDt;
    EXPECT_NEAR(out[0].position[0], 5.0 * t, 1e-9);
    EXPECT_NEAR(out[0].position[1], -2.0 * t, 1e-9);
    EXPECT_NEAR(out[0].velocity[0], 5.0, 1e-9);
    EXPECT_GT(out[0].position[0], previous_x);  // never goes backward
    previous_x = out[0].position[0];
  }
}

TEST(ReplayReaderSample, TheVelocityIsContinuousAcrossALoggedFrame) {
  // A path that turns at the middle frame: the curve on each side meets the logged velocity there.
  const std::vector<HandFrame> frames = {
      {0, with({agent(0, 0, 0, 4, 0, 0)})},
      {10, with({agent(20, 0, 0, 4, 0, 0)})},
      {20, with({agent(40, 5, 0, 4, 2, 0)})},
  };
  Replay replay;
  ASSERT_EQ(load(replay, make_log(frames, 1)), ReplayStatus::kOk);
  std::vector<AgentSnapshot> before;
  std::vector<AgentSnapshot> after;
  ASSERT_TRUE(replay.sample(10.0 - 1e-7, before));
  ASSERT_TRUE(replay.sample(10.0 + 1e-7, after));
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR(before[0].velocity[k], after[0].velocity[k], 1e-5);
    EXPECT_NEAR(before[0].position[k], after[0].position[k], 1e-5);
  }
  EXPECT_NEAR(before[0].velocity[0], 4.0, 1e-5);
}

TEST(ReplayReaderSample, AliveAndFallbackComeFromTheEarlierFrame) {
  const std::vector<HandFrame> frames = {
      {0, with({agent(0, 0, 0, 0, 0, 0, true, 0), agent(1, 0, 0, 0, 0, 0, true, 2)})},
      {10, with({agent(0, 0, 0, 0, 0, 0, false, 3), agent(1, 0, 0, 0, 0, 0, true, 0)})},
  };
  Replay replay;
  ASSERT_EQ(load(replay, make_log(frames, 2)), ReplayStatus::kOk);
  std::vector<AgentSnapshot> out;
  ASSERT_TRUE(replay.sample(5.0, out));
  EXPECT_TRUE(out[0].alive);  // it fails at the later frame, not yet here
  EXPECT_EQ(out[0].fallback_state, 0);
  EXPECT_EQ(out[1].fallback_state, 2);
  ASSERT_TRUE(replay.sample(10.0, out));
  EXPECT_FALSE(out[0].alive);
  EXPECT_EQ(out[0].fallback_state, 3);
}

TEST(ReplayReaderSample, FramesBetweenStateFramesAreIgnored) {
  Snapshot only_events;
  const std::vector<HandFrame> frames = {
      {0, with({agent(0, 0, 0, 2, 0, 0)})},
      {5, only_events},
      {10, with({agent(10, 0, 0, 2, 0, 0)})},
  };
  Replay replay;
  ASSERT_EQ(load(replay, make_log(frames, 1)), ReplayStatus::kOk);
  std::vector<AgentSnapshot> out;
  ASSERT_TRUE(replay.sample(5.0, out));
  EXPECT_NEAR(out[0].position[0], 5.0, 1e-9);  // interpolated across the frame with no state
  ASSERT_TRUE(replay.sample(10.0, out));
  EXPECT_EQ(out[0].position[0], 10.0);
}

TEST(ReplayReaderSample, AnEmptyReplayCannotBeSampled) {
  const Replay replay;
  std::vector<AgentSnapshot> out(3);
  EXPECT_FALSE(replay.sample(1.0, out));
  EXPECT_TRUE(out.empty());
}

namespace {

swarm::Scenario small_scenario(std::uint64_t stride) {
  const char* text = R"(version: 1
name: replay_run
seed: 4
dt: 0.02
duration_s: 2
scene: {type: hills, size_m: [300, 300], cell_m: 10, amplitude_m: 8, obstacles: {density_per_m2: 0.0004, spawn_clear_radius_m: 20}}
swarm:
  drones: 3
  formation: {shape: wedge, spacing_m: 20}
  start: {columns: 2, spacing_m: 30, jitter_m: 4, altitude_m: 20, altitude_jitter_m: 2}
mission:
  waypoints:
    - [-30, 0, 20]
    - [30, 0, 20]
  speed_mps: 3
)";
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  swarm::Scenario s = r.scenario;
  s.snapshot_every_ticks = stride;
  return s;
}

}  // namespace

TEST(ReplayReaderRealLog, ARecordedRunLoadsWithEveryFrameAndTheFinalHash) {
  const swarm::Scenario s = small_scenario(1);
  Bytes log;
  std::uint64_t final_hash = 0;
  ASSERT_EQ(swarm::run_scenario(s, 1, log, &final_hash), swarm::LogStatus::kOk);
  Replay replay;
  ASSERT_EQ(load(replay, log), ReplayStatus::kOk);
  EXPECT_EQ(replay.frames().size(), s.duration_ticks);
  EXPECT_EQ(replay.state_frame_count(), s.duration_ticks);
  EXPECT_EQ(replay.header().agent_count, 3U);
  EXPECT_EQ(replay.header().dt, 0.02);
  EXPECT_EQ(replay.trailer().final_hash, final_hash);
  EXPECT_EQ(replay.frames().back().hash, final_hash);
  EXPECT_EQ(replay.last_tick(), s.duration_ticks - 1);
  for (const auto& frame : replay.frames()) {
    ASSERT_EQ(frame.snapshot.agents.size(), 3U);
  }
}

TEST(ReplayReaderRealLog, ASnapshotStrideLeavesFramesWithoutStateButKeepsTheLastTick) {
  const swarm::Scenario s = small_scenario(25);
  Bytes log;
  ASSERT_EQ(swarm::run_scenario(s, 1, log), swarm::LogStatus::kOk);
  Replay replay;
  ASSERT_EQ(load(replay, log), ReplayStatus::kOk);
  EXPECT_EQ(replay.frames().size(), s.duration_ticks);
  for (const auto& frame : replay.frames()) {
    const bool expected = frame.tick % 25 == 0 || frame.tick + 1 == s.duration_ticks;
    EXPECT_EQ(frame.has_state(), expected) << frame.tick;
  }
  EXPECT_TRUE(replay.frames().back().has_state());

  // Between stride ticks the position moves smoothly between the two logged states.
  std::vector<AgentSnapshot> a;
  std::vector<AgentSnapshot> mid;
  std::vector<AgentSnapshot> b;
  ASSERT_TRUE(replay.sample(25.0, a));
  ASSERT_TRUE(replay.sample(37.5, mid));
  ASSERT_TRUE(replay.sample(50.0, b));
  for (std::size_t k = 0; k < 3; ++k) {
    const double lo = std::min(a[0].position[k], b[0].position[k]);
    const double hi = std::max(a[0].position[k], b[0].position[k]);
    EXPECT_GE(mid[0].position[k], lo - 1.0);
    EXPECT_LE(mid[0].position[k], hi + 1.0);
  }
}

TEST(ReplayReaderRealLog, ThePlaybackFramesAreTheSnapshotsThatWereRecorded) {
  const swarm::Scenario s = small_scenario(1);
  Bytes log;
  ASSERT_EQ(swarm::run_scenario(s, 2, log), swarm::LogStatus::kOk);
  Replay replay;
  ASSERT_EQ(load(replay, log), ReplayStatus::kOk);
  swarm::LogReader reader(log.data(), log.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  swarm::LogRecord record;
  std::size_t index = 0;
  while (reader.next(record) == swarm::LogStatus::kOk) {
    swarm::Snapshot decoded;
    ASSERT_EQ(swarm::decode_snapshot(record.blob.data(), record.blob.size(), decoded),
              swarm::SnapshotStatus::kOk);
    const swarm::ReplayFrame& frame = replay.frames()[index++];
    EXPECT_EQ(frame.tick, record.tick);
    EXPECT_EQ(frame.hash, record.hash);
    ASSERT_EQ(frame.snapshot.agents.size(), decoded.agents.size());
    for (std::size_t i = 0; i < decoded.agents.size(); ++i) {
      EXPECT_EQ(frame.snapshot.agents[i].position, decoded.agents[i].position);
    }
    EXPECT_EQ(frame.snapshot.events.size(), decoded.events.size());
  }
  EXPECT_EQ(index, replay.frames().size());
}

TEST(ReplayReaderRealLog, TheSceneBuiltFromTheScenarioInTheHeaderIsTheSceneOfTheRun) {
  const swarm::Scenario s = small_scenario(1);
  Bytes log;
  ASSERT_EQ(swarm::run_scenario(s, 1, log), swarm::LogStatus::kOk);
  Replay replay;
  ASSERT_EQ(load(replay, log), ReplayStatus::kOk);

  // The scene is not in the log; export-scene rebuilds it from the header. It must be the scene
  // the run used, and survive the scene file's serialization.
  const std::string tag = swarm::kScenarioLogTag;
  ASSERT_EQ(replay.header().config.rfind(tag, 0), 0U);
  const swarm::LoadResult from_header =
      swarm::load_scenario(replay.header().config.substr(tag.size()), "scenario");
  ASSERT_TRUE(from_header.ok());
  const swarm::ScenarioWorld original = swarm::build_world(s);
  const swarm::ScenarioWorld rebuilt = swarm::build_world(from_header.scenario);
  EXPECT_EQ(swarm::scene_hash(rebuilt.scene->data()), swarm::scene_hash(original.scene->data()));
  EXPECT_GT(original.scene->data().obstacles.size(), 0U);

  std::vector<std::uint8_t> file;
  ASSERT_EQ(swarm::serialize_scene(rebuilt.scene->data(), file), swarm::SceneStatus::kOk);
  swarm::SceneData parsed;
  ASSERT_EQ(swarm::parse_scene(file.data(), file.size(), parsed), swarm::SceneStatus::kOk);
  EXPECT_EQ(swarm::scene_hash(parsed), swarm::scene_hash(original.scene->data()));
}
