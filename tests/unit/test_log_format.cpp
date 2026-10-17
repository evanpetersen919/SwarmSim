#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "swarm/format/log.hpp"

namespace {

using Bytes = std::vector<std::uint8_t>;

swarm::LogHeader sample_header() {
  swarm::LogHeader h;
  h.seed = 1;
  h.dt = 0.5;
  h.agent_count = 2;
  h.config = "ab";
  return h;
}

swarm::LogRecord record(std::uint64_t tick, std::uint64_t hash, Bytes blob = {}) {
  swarm::LogRecord r;
  r.tick = tick;
  r.hash = hash;
  r.blob = std::move(blob);
  return r;
}

// Hand-checked bytes of a log with one record. Changing the format means changing this on purpose
// and bumping kLogVersion.
Bytes minimal_log_bytes() {
  return {
      // header
      'S',
      'W',
      'R',
      'M',  // magic
      0x01,
      0x00,
      0x00,
      0x00,  // version 1
      0x01,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,  // seed 1
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0xE0,
      0x3F,  // dt 0.5 as binary64
      0x02,
      0x00,
      0x00,
      0x00,  // agent_count 2
      0x02,
      0x00,
      0x00,
      0x00,  // config_len 2
      'a',
      'b',  // config
      // record
      0x01,  // kind
      0x03,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,  // tick 3
      0x08,
      0x07,
      0x06,
      0x05,
      0x04,
      0x03,
      0x02,
      0x01,  // hash 0x0102030405060708
      0x01,
      0x00,
      0x00,
      0x00,  // blob_len 1
      0x09,  // blob
      // trailer
      0x02,  // kind
      0x01,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,  // record_count 1
      0x08,
      0x07,
      0x06,
      0x05,
      0x04,
      0x03,
      0x02,
      0x01,  // final_hash
  };
}

Bytes write_minimal() {
  swarm::LogWriter w;
  EXPECT_EQ(w.begin(sample_header()), swarm::LogStatus::kOk);
  EXPECT_EQ(w.append(record(3, 0x0102030405060708ULL, {9})), swarm::LogStatus::kOk);
  EXPECT_EQ(w.finish(), swarm::LogStatus::kOk);
  return w.bytes();
}

// Reads a whole log and returns the first non-ok status (kEndOfLog on success).
swarm::LogStatus read_all(const Bytes& bytes, std::vector<swarm::LogRecord>* records = nullptr) {
  swarm::LogReader reader(bytes.data(), bytes.size());
  swarm::LogStatus s = reader.read_header();
  if (s != swarm::LogStatus::kOk) {
    return s;
  }
  for (;;) {
    swarm::LogRecord r;
    s = reader.next(r);
    if (s != swarm::LogStatus::kOk) {
      return s;
    }
    if (records != nullptr) {
      records->push_back(r);
    }
  }
}

}  // namespace

TEST(LogFormat, WriterProducesTheExactDocumentedBytes) {
  EXPECT_EQ(write_minimal(), minimal_log_bytes());
}

TEST(LogFormat, ReaderParsesTheDocumentedBytes) {
  const Bytes bytes = minimal_log_bytes();
  swarm::LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  EXPECT_EQ(reader.header().version, 1U);
  EXPECT_EQ(reader.header().seed, 1U);
  EXPECT_DOUBLE_EQ(reader.header().dt, 0.5);
  EXPECT_EQ(reader.header().agent_count, 2U);
  EXPECT_EQ(reader.header().config, "ab");

  swarm::LogRecord r;
  ASSERT_EQ(reader.next(r), swarm::LogStatus::kOk);
  EXPECT_EQ(r.tick, 3U);
  EXPECT_EQ(r.hash, 0x0102030405060708ULL);
  EXPECT_EQ(r.blob, Bytes{9});
  EXPECT_EQ(reader.next(r), swarm::LogStatus::kEndOfLog);
  EXPECT_EQ(reader.trailer().record_count, 1U);
  EXPECT_EQ(reader.trailer().final_hash, 0x0102030405060708ULL);
  EXPECT_EQ(reader.next(r), swarm::LogStatus::kEndOfLog);
}

TEST(LogFormat, RoundTripsHeaderRecordsBlobsAndMultilineConfig) {
  swarm::LogHeader header;
  header.seed = 0xFFFFFFFFFFFFFFFFULL;
  header.dt = 0.0166666666666667;
  header.agent_count = 20;
  header.config = "a=1\nb=two\n\n";
  Bytes big_blob(1000);
  for (std::size_t i = 0; i < big_blob.size(); ++i) {
    big_blob[i] = static_cast<std::uint8_t>(i * 7U);
  }

  swarm::LogWriter w;
  ASSERT_EQ(w.begin(header), swarm::LogStatus::kOk);
  ASSERT_EQ(w.append(record(0, 11)), swarm::LogStatus::kOk);
  ASSERT_EQ(w.append(record(1, 22, big_blob)), swarm::LogStatus::kOk);
  ASSERT_EQ(w.append(record(2, 33, {0, 255})), swarm::LogStatus::kOk);
  ASSERT_EQ(w.finish(), swarm::LogStatus::kOk);

  const Bytes bytes = w.bytes();
  swarm::LogReader reader(bytes.data(), bytes.size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  EXPECT_EQ(reader.header().seed, header.seed);
  EXPECT_EQ(reader.header().dt, header.dt);
  EXPECT_EQ(reader.header().agent_count, 20U);
  EXPECT_EQ(reader.header().config, header.config);

  std::vector<swarm::LogRecord> records;
  EXPECT_EQ(read_all(bytes, &records), swarm::LogStatus::kEndOfLog);
  ASSERT_EQ(records.size(), 3U);
  EXPECT_EQ(records[0].hash, 11U);
  EXPECT_TRUE(records[0].blob.empty());
  EXPECT_EQ(records[1].blob, big_blob);
  EXPECT_EQ(records[2].blob, (Bytes{0, 255}));
  EXPECT_EQ(records[2].tick, 2U);
}

TEST(LogFormat, EmptyLogHasZeroRecordsAndZeroFinalHash) {
  swarm::LogWriter w;
  ASSERT_EQ(w.begin(sample_header()), swarm::LogStatus::kOk);
  ASSERT_EQ(w.finish(), swarm::LogStatus::kOk);
  swarm::LogReader reader(w.bytes().data(), w.bytes().size());
  ASSERT_EQ(reader.read_header(), swarm::LogStatus::kOk);
  swarm::LogRecord r;
  EXPECT_EQ(reader.next(r), swarm::LogStatus::kEndOfLog);
  EXPECT_EQ(reader.trailer().record_count, 0U);
  EXPECT_EQ(reader.trailer().final_hash, 0U);
}

TEST(LogFormat, BadMagicIsRejected) {
  Bytes bytes = minimal_log_bytes();
  bytes[0] = 'X';
  EXPECT_EQ(read_all(bytes), swarm::LogStatus::kBadMagic);
}

TEST(LogFormat, UnsupportedVersionIsRejected) {
  Bytes bytes = minimal_log_bytes();
  bytes[4] = 2;
  EXPECT_EQ(read_all(bytes), swarm::LogStatus::kUnsupportedVersion);
}

TEST(LogFormat, TrailerCountMismatchIsRejected) {
  Bytes bytes = minimal_log_bytes();
  bytes[bytes.size() - 16] = 2;  // record_count low byte
  EXPECT_EQ(read_all(bytes), swarm::LogStatus::kTrailerCountMismatch);
}

TEST(LogFormat, TrailerHashMismatchIsRejected) {
  Bytes bytes = minimal_log_bytes();
  bytes[bytes.size() - 1] ^= 0x01;  // final_hash high byte
  EXPECT_EQ(read_all(bytes), swarm::LogStatus::kTrailerHashMismatch);
}

TEST(LogFormat, TrailingBytesAfterTheTrailerAreRejected) {
  Bytes bytes = minimal_log_bytes();
  bytes.push_back(0);
  EXPECT_EQ(read_all(bytes), swarm::LogStatus::kCorrupt);
}

TEST(LogFormat, UnknownRecordKindIsRejected) {
  Bytes bytes = minimal_log_bytes();
  bytes[4 + 4 + 8 + 8 + 4 + 4 + 2] = 7;  // first byte after the header
  EXPECT_EQ(read_all(bytes), swarm::LogStatus::kCorrupt);
}

TEST(LogFormat, OversizedLengthsAreRejectedWithoutAllocating) {
  Bytes bad_config = minimal_log_bytes();
  bad_config[4 + 4 + 8 + 8 + 4 + 3] = 0xFF;  // config_len high byte
  EXPECT_EQ(read_all(bad_config), swarm::LogStatus::kCorrupt);

  Bytes bad_blob = minimal_log_bytes();
  const std::size_t blob_len_at = 4 + 4 + 8 + 8 + 4 + 4 + 2 + 1 + 8 + 8;
  bad_blob[blob_len_at + 3] = 0xFF;  // blob_len high byte
  EXPECT_EQ(read_all(bad_blob), swarm::LogStatus::kCorrupt);
}

TEST(LogFormat, EveryProperPrefixIsReportedAsTruncated) {
  const Bytes full = minimal_log_bytes();
  for (std::size_t length = 0; length < full.size(); ++length) {
    const Bytes prefix(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(length));
    EXPECT_EQ(read_all(prefix), swarm::LogStatus::kTruncated) << "length " << length;
  }
  EXPECT_EQ(read_all(full), swarm::LogStatus::kEndOfLog);
}

TEST(LogFormat, ErrorsAreSticky) {
  Bytes bytes = minimal_log_bytes();
  bytes[0] = 'X';
  swarm::LogReader reader(bytes.data(), bytes.size());
  EXPECT_EQ(reader.read_header(), swarm::LogStatus::kBadMagic);
  swarm::LogRecord r;
  EXPECT_EQ(reader.next(r), swarm::LogStatus::kBadMagic);
  EXPECT_EQ(reader.read_header(), swarm::LogStatus::kBadMagic);
}

TEST(LogFormat, WriterRejectsCallsInTheWrongOrder) {
  swarm::LogWriter w;
  EXPECT_EQ(w.append(record(0, 0)), swarm::LogStatus::kUsage);
  EXPECT_EQ(w.finish(), swarm::LogStatus::kUsage);
  ASSERT_EQ(w.begin(sample_header()), swarm::LogStatus::kOk);
  EXPECT_EQ(w.begin(sample_header()), swarm::LogStatus::kUsage);
  ASSERT_EQ(w.finish(), swarm::LogStatus::kOk);
  EXPECT_EQ(w.append(record(0, 0)), swarm::LogStatus::kUsage);
  EXPECT_EQ(w.finish(), swarm::LogStatus::kUsage);
}

TEST(LogFormat, ReaderRejectsReadingRecordsBeforeTheHeader) {
  const Bytes bytes = minimal_log_bytes();
  swarm::LogReader reader(bytes.data(), bytes.size());
  swarm::LogRecord r;
  EXPECT_EQ(reader.next(r), swarm::LogStatus::kUsage);
}

TEST(LogFormat, WriterEnforcesSizeLimits) {
  swarm::LogHeader header = sample_header();
  header.config.assign(swarm::kMaxLogConfigBytes + 1U, 'x');
  swarm::LogWriter too_big_config;
  EXPECT_EQ(too_big_config.begin(header), swarm::LogStatus::kLimitExceeded);

  swarm::LogWriter w;
  ASSERT_EQ(w.begin(sample_header()), swarm::LogStatus::kOk);
  swarm::LogRecord big;
  big.blob.assign(static_cast<std::size_t>(swarm::kMaxLogBlobBytes) + 1U, 0);
  EXPECT_EQ(w.append(big), swarm::LogStatus::kLimitExceeded);
}

TEST(LogFormat, StatusNamesAreDistinct) {
  EXPECT_STREQ(swarm::to_string(swarm::LogStatus::kTruncated), "truncated");
  EXPECT_STRNE(swarm::to_string(swarm::LogStatus::kBadMagic),
               swarm::to_string(swarm::LogStatus::kCorrupt));
}
