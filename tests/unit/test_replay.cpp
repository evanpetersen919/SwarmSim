#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "swarm/demo_agent.hpp"
#include "swarm/run.hpp"

namespace {

using Bytes = std::vector<std::uint8_t>;

swarm::RunConfig sample_config() {
  swarm::RunConfig c;
  c.sim.seed = 1234;
  c.sim.dt = 0.02;
  c.sim.channel.latency_ticks = 2;
  c.sim.channel.jitter_ticks = 2;
  c.sim.channel.loss_model = swarm::LossModel::kBurst;
  c.sim.channel.loss = 0.2;
  c.sim.channel.mean_burst = 4.0;
  c.sim.channel.max_msgs_per_sender_per_tick = 3;
  c.agents = 10;
  c.ticks = 100;
  return c;
}

Bytes record(const swarm::RunConfig& config, std::size_t threads) {
  Bytes bytes;
  EXPECT_EQ(swarm::record_run(config, threads, bytes), swarm::LogStatus::kOk);
  return bytes;
}

swarm::VerifyResult verify(const Bytes& bytes, std::size_t threads = 1) {
  return swarm::verify_log(bytes.data(), bytes.size(), threads);
}

// Position of `needle` in the log's bytes, to patch the embedded configuration text.
std::size_t find_text(const Bytes& bytes, const std::string& needle) {
  const auto it = std::search(bytes.begin(), bytes.end(), needle.begin(), needle.end());
  return static_cast<std::size_t>(it - bytes.begin());
}

std::size_t header_size(const swarm::RunConfig& config) {
  return 4 + 4 + 8 + 8 + 4 + 4 + swarm::to_log_header(config).config.size();
}

constexpr std::size_t kRecordSize = 1 + 8 + 8 + 4;  // records without a blob

}  // namespace

TEST(RunConfigText, RoundTripsEveryField) {
  swarm::RunConfig original = sample_config();
  original.sim.dt = 1.0 / 3.0;
  original.sim.channel.loss = 0.1;
  original.sim.channel.mean_burst = 1e-9 + 1.0;
  original.ticks = 18446744073709551615ULL;

  swarm::RunConfig parsed;
  std::string error;
  ASSERT_TRUE(swarm::from_log_header(swarm::to_log_header(original), parsed, error)) << error;
  EXPECT_EQ(parsed.sim.seed, original.sim.seed);
  EXPECT_EQ(parsed.sim.dt, original.sim.dt);
  EXPECT_EQ(parsed.agents, original.agents);
  EXPECT_EQ(parsed.ticks, original.ticks);
  EXPECT_EQ(parsed.sim.channel.latency_ticks, original.sim.channel.latency_ticks);
  EXPECT_EQ(parsed.sim.channel.loss_model, original.sim.channel.loss_model);
  EXPECT_EQ(parsed.sim.channel.loss, original.sim.channel.loss);
  EXPECT_EQ(parsed.sim.channel.mean_burst, original.sim.channel.mean_burst);
  EXPECT_EQ(parsed.sim.channel.jitter_ticks, original.sim.channel.jitter_ticks);
  EXPECT_EQ(parsed.sim.channel.max_msgs_per_sender_per_tick,
            original.sim.channel.max_msgs_per_sender_per_tick);
}

TEST(RunConfigText, RejectsUnknownDuplicateMissingAndMalformedEntries) {
  const swarm::LogHeader good = swarm::to_log_header(sample_config());
  swarm::RunConfig out;
  std::string error;

  swarm::LogHeader h = good;
  h.config += "surprise=1\n";
  EXPECT_FALSE(swarm::from_log_header(h, out, error));
  EXPECT_NE(error.find("surprise"), std::string::npos);

  h = good;
  h.config += "loss=0.5\n";
  EXPECT_FALSE(swarm::from_log_header(h, out, error));
  EXPECT_NE(error.find("duplicate"), std::string::npos);

  h = good;
  h.config.erase(h.config.find("jitter_ticks"),
                 h.config.find("max_msgs") - h.config.find("jitter_ticks"));
  EXPECT_FALSE(swarm::from_log_header(h, out, error));
  EXPECT_NE(error.find("jitter_ticks"), std::string::npos);

  for (const char* bad :
       {"loss=abc\n", "ticks=-1\n", "ticks=12x\n", "loss_model=chaotic\n",
        "latency_ticks=99999999999\n", "mean_burst=\n", "no_equals_sign\n", "=5\n"}) {
    h = good;
    const std::string key = std::string(bad).substr(0, std::string(bad).find('='));
    const std::size_t at = h.config.find(key + "=");
    if (at != std::string::npos) {
      h.config.erase(at, h.config.find('\n', at) - at + 1);
    }
    h.config += bad;
    EXPECT_FALSE(swarm::from_log_header(h, out, error)) << bad;
  }
}

TEST(Replay, RecordThenVerifySucceeds) {
  const Bytes bytes = record(sample_config(), 1);
  const auto result = verify(bytes);
  EXPECT_EQ(result.status, swarm::VerifyStatus::kOk) << result.message;
}

TEST(Replay, RecordingDoesNotDependOnThreadCount) {
  const Bytes one = record(sample_config(), 1);
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(record(sample_config(), threads), one) << "threads " << threads;
  }
}

TEST(Replay, LogRecordedWithOneThreadVerifiesWithAnyOther) {
  const Bytes bytes = record(sample_config(), 1);
  for (std::size_t threads : {2U, 4U, 8U}) {
    const auto result = verify(bytes, threads);
    EXPECT_EQ(result.status, swarm::VerifyStatus::kOk) << "threads " << threads;
  }
}

TEST(Replay, FinalHashMatchesTheLastRecordedHash) {
  const swarm::RunConfig config = sample_config();
  const Bytes bytes = record(config, 2);
  swarm::LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  swarm::LogRecord r;
  std::uint64_t last = 0;
  while (reader.next(r) == swarm::LogStatus::kOk) {
    last = r.hash;
  }
  EXPECT_EQ(last, swarm::final_hash(config, 4));
}

TEST(Replay, CorruptedHashFailsAtThatTick) {
  const swarm::RunConfig config = sample_config();
  Bytes bytes = record(config, 1);
  const std::size_t tick = 50;
  bytes[header_size(config) + tick * kRecordSize + 1 + 8] ^= 0x40;
  const auto result = verify(bytes);
  EXPECT_EQ(result.status, swarm::VerifyStatus::kHashMismatch);
  EXPECT_EQ(result.tick, tick);
}

TEST(Replay, AlteredConfigurationFailsVerification) {
  Bytes bytes = record(sample_config(), 1);
  const std::size_t at = find_text(bytes, "latency_ticks=2");
  ASSERT_LT(at, bytes.size());
  bytes[at + std::string("latency_ticks=").size()] = '3';
  const auto result = verify(bytes);
  EXPECT_EQ(result.status, swarm::VerifyStatus::kHashMismatch);
}

TEST(Replay, UnparsableConfigurationIsReportedAsBadConfig) {
  Bytes bytes = record(sample_config(), 1);
  const std::size_t at = find_text(bytes, "ticks=100");
  ASSERT_LT(at, bytes.size());
  bytes[at] = 'x';
  EXPECT_EQ(verify(bytes).status, swarm::VerifyStatus::kBadConfig);
}

TEST(Replay, InvalidChannelParametersAreReportedAsBadConfig) {
  Bytes bytes = record(sample_config(), 1);
  const std::size_t at = find_text(bytes, "latency_ticks=2");
  ASSERT_LT(at, bytes.size());
  bytes[at + std::string("latency_ticks=").size()] = '0';
  EXPECT_EQ(verify(bytes).status, swarm::VerifyStatus::kBadConfig);
}

TEST(Replay, TruncatedLogIsAReadErrorNotAHashError) {
  Bytes bytes = record(sample_config(), 1);
  bytes.resize(bytes.size() - 10);
  const auto result = verify(bytes);
  EXPECT_EQ(result.status, swarm::VerifyStatus::kLogError);
  EXPECT_EQ(result.log_status, swarm::LogStatus::kTruncated);
}

TEST(Replay, EmptyAndGarbageInputAreReadErrors) {
  EXPECT_EQ(verify(Bytes{}).status, swarm::VerifyStatus::kLogError);
  EXPECT_EQ(verify(Bytes{'n', 'o', 'p', 'e', 'x', 'x'}).log_status, swarm::LogStatus::kBadMagic);
}

// A log whose header says `config.ticks` but which holds `record_count` genuine records.
Bytes log_with_record_count(swarm::RunConfig config, std::uint64_t record_count) {
  swarm::LogWriter writer;
  EXPECT_EQ(writer.begin(swarm::to_log_header(config)), swarm::LogStatus::kOk);
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  for (std::uint32_t i = 0; i < config.agents; ++i) {
    agents.push_back(std::make_unique<swarm::DemoAgent>(i, config.agents));
  }
  swarm::Simulation sim(config.sim, std::move(agents), 1);
  for (std::uint64_t t = 0; t < record_count; ++t) {
    sim.step();
    swarm::LogRecord r;
    r.tick = t;
    r.hash = sim.state_hash();
    EXPECT_EQ(writer.append(r), swarm::LogStatus::kOk);
  }
  EXPECT_EQ(writer.finish(), swarm::LogStatus::kOk);
  return writer.bytes();
}

TEST(Replay, FewerRecordsThanConfiguredIsALengthMismatch) {
  swarm::RunConfig config = sample_config();
  config.ticks = 100;
  EXPECT_EQ(verify(log_with_record_count(config, 50)).status, swarm::VerifyStatus::kLengthMismatch);
}

TEST(Replay, MoreRecordsThanConfiguredIsALengthMismatch) {
  swarm::RunConfig config = sample_config();
  config.ticks = 10;
  EXPECT_EQ(verify(log_with_record_count(config, 11)).status, swarm::VerifyStatus::kLengthMismatch);
}

TEST(Replay, RecordsWithTheWrongTickNumberAreRejected) {
  swarm::RunConfig config = sample_config();
  config.ticks = 5;
  swarm::LogWriter writer;
  ASSERT_EQ(writer.begin(swarm::to_log_header(config)), swarm::LogStatus::kOk);
  for (std::uint64_t t = 0; t < 5; ++t) {
    swarm::LogRecord r;
    r.tick = t + 1;
    ASSERT_EQ(writer.append(r), swarm::LogStatus::kOk);
  }
  ASSERT_EQ(writer.finish(), swarm::LogStatus::kOk);
  const auto result = verify(writer.bytes());
  EXPECT_EQ(result.status, swarm::VerifyStatus::kTickMismatch);
  EXPECT_EQ(result.tick, 0U);
}

TEST(Replay, ZeroTicksAndZeroAgentsRoundTrip) {
  swarm::RunConfig config = sample_config();
  config.ticks = 0;
  EXPECT_EQ(verify(record(config, 1)).status, swarm::VerifyStatus::kOk);

  config.ticks = 20;
  config.agents = 0;
  EXPECT_EQ(verify(record(config, 2)).status, swarm::VerifyStatus::kOk);
}

TEST(Replay, InvalidConfigurationIsRejectedWhenRecording) {
  swarm::RunConfig config = sample_config();
  config.sim.channel.latency_ticks = 0;
  Bytes bytes;
  EXPECT_THROW(swarm::record_run(config, 1, bytes), std::invalid_argument);
}
