#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Binary run log. This header is compiled into the Unreal plugin, which builds without C++
// exceptions, so errors are reported as status codes and the API works on in-memory buffers.
//
// Layout (little-endian, fixed width):
//   header:  "SWRM" | u32 version | u64 seed | f64 dt | u32 agent_count | u32 config_len | config
//   record:  u8 1 | u64 tick | u64 hash | u32 blob_len | blob          (repeated)
//   trailer: u8 2 | u64 record_count | u64 final_hash
// final_hash is the hash of the last record, or 0 if there are no records.

namespace swarm {

inline constexpr std::uint32_t kLogVersion = 1;
inline constexpr std::uint32_t kMaxLogConfigBytes = 1U << 20;
inline constexpr std::uint32_t kMaxLogBlobBytes = 64U << 20;

enum class LogStatus {
  kOk,        // a record was read or written
  kEndOfLog,  // reader reached a valid trailer
  kBadMagic,
  kUnsupportedVersion,
  kTruncated,
  kCorrupt,
  kTrailerCountMismatch,
  kTrailerHashMismatch,
  kLimitExceeded,  // a config or blob is larger than the format allows
  kUsage,          // calls made in the wrong order
};

const char* to_string(LogStatus status);

struct LogHeader {
  std::uint32_t version = kLogVersion;
  std::uint64_t seed = 0;
  double dt = 0.0;
  std::uint32_t agent_count = 0;
  // Opaque text. The format does not interpret it; the runner stores its configuration here.
  std::string config;
};

struct LogRecord {
  std::uint64_t tick = 0;
  std::uint64_t hash = 0;
  // Reserved for state snapshots read by the Unreal viewer. Empty until then.
  std::vector<std::uint8_t> blob;
};

struct LogTrailer {
  std::uint64_t record_count = 0;
  std::uint64_t final_hash = 0;
};

class LogWriter {
 public:
  // Starts a log. Must be called once, first.
  LogStatus begin(const LogHeader& header);
  LogStatus append(const LogRecord& record);
  // Writes the trailer. After this the log is complete and bytes() is a valid file.
  LogStatus finish();

  const std::vector<std::uint8_t>& bytes() const { return bytes_; }

 private:
  enum class State { kNew, kOpen, kFinished };
  State state_ = State::kNew;
  std::vector<std::uint8_t> bytes_;
  std::uint64_t record_count_ = 0;
  std::uint64_t last_hash_ = 0;
};

class LogReader {
 public:
  // The buffer must outlive the reader.
  LogReader(const std::uint8_t* data, std::size_t size);

  // Parses and validates the header. Must be called once, first.
  LogStatus read_header();
  const LogHeader& header() const { return header_; }

  // Reads the next record into `record`. Returns kOk for a record, kEndOfLog once a valid trailer
  // has been read, or an error. After an error the reader stays in that error.
  LogStatus next(LogRecord& record);
  const LogTrailer& trailer() const { return trailer_; }

 private:
  enum class State { kNew, kHeaderRead, kDone, kFailed };
  LogStatus fail(LogStatus status);
  bool has(std::size_t count) const { return size_ - pos_ >= count; }

  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t pos_ = 0;
  State state_ = State::kNew;
  LogStatus error_ = LogStatus::kOk;
  LogHeader header_;
  LogTrailer trailer_;
  std::uint64_t records_read_ = 0;
  std::uint64_t last_hash_ = 0;
};

}  // namespace swarm
