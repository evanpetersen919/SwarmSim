#include "swarm/format/log.hpp"

#include <cstring>

#include "byte_io.hpp"

namespace swarm {

namespace {

constexpr std::uint8_t kMagic[4] = {'S', 'W', 'R', 'M'};
constexpr std::uint8_t kRecordKind = 1;
constexpr std::uint8_t kTrailerKind = 2;

using detail::get_f64;
using detail::get_u32;
using detail::get_u64;
using detail::put_f64;
using detail::put_u32;
using detail::put_u64;
using detail::put_u8;

}  // namespace

const char* to_string(LogStatus status) {
  switch (status) {
    case LogStatus::kOk:
      return "ok";
    case LogStatus::kEndOfLog:
      return "end of log";
    case LogStatus::kBadMagic:
      return "bad magic";
    case LogStatus::kUnsupportedVersion:
      return "unsupported version";
    case LogStatus::kTruncated:
      return "truncated";
    case LogStatus::kCorrupt:
      return "corrupt";
    case LogStatus::kTrailerCountMismatch:
      return "trailer record count mismatch";
    case LogStatus::kTrailerHashMismatch:
      return "trailer hash mismatch";
    case LogStatus::kLimitExceeded:
      return "limit exceeded";
    case LogStatus::kUsage:
      return "usage error";
  }
  return "unknown";
}

LogStatus LogWriter::begin(const LogHeader& header) {
  if (state_ != State::kNew) {
    return LogStatus::kUsage;
  }
  if (header.config.size() > kMaxLogConfigBytes) {
    return LogStatus::kLimitExceeded;
  }
  bytes_.insert(bytes_.end(), kMagic, kMagic + sizeof(kMagic));
  put_u32(bytes_, kLogVersion);
  put_u64(bytes_, header.seed);
  put_f64(bytes_, header.dt);
  put_u32(bytes_, header.agent_count);
  put_u32(bytes_, static_cast<std::uint32_t>(header.config.size()));
  bytes_.insert(bytes_.end(), header.config.begin(), header.config.end());
  state_ = State::kOpen;
  return LogStatus::kOk;
}

LogStatus LogWriter::append(const LogRecord& record) {
  if (state_ != State::kOpen) {
    return LogStatus::kUsage;
  }
  if (record.blob.size() > kMaxLogBlobBytes) {
    return LogStatus::kLimitExceeded;
  }
  put_u8(bytes_, kRecordKind);
  put_u64(bytes_, record.tick);
  put_u64(bytes_, record.hash);
  put_u32(bytes_, static_cast<std::uint32_t>(record.blob.size()));
  bytes_.insert(bytes_.end(), record.blob.begin(), record.blob.end());
  ++record_count_;
  last_hash_ = record.hash;
  return LogStatus::kOk;
}

LogStatus LogWriter::finish() {
  if (state_ != State::kOpen) {
    return LogStatus::kUsage;
  }
  put_u8(bytes_, kTrailerKind);
  put_u64(bytes_, record_count_);
  put_u64(bytes_, last_hash_);
  state_ = State::kFinished;
  return LogStatus::kOk;
}

LogReader::LogReader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

LogStatus LogReader::fail(LogStatus status) {
  state_ = State::kFailed;
  error_ = status;
  return status;
}

LogStatus LogReader::read_header() {
  if (state_ != State::kNew) {
    return state_ == State::kFailed ? error_ : LogStatus::kUsage;
  }

  if (!has(4)) {
    return fail(LogStatus::kTruncated);
  }
  if (std::memcmp(data_ + pos_, kMagic, sizeof(kMagic)) != 0) {
    return fail(LogStatus::kBadMagic);
  }
  pos_ += 4;

  if (!has(4)) {
    return fail(LogStatus::kTruncated);
  }
  header_.version = get_u32(data_ + pos_);
  pos_ += 4;
  if (header_.version != kLogVersion) {
    return fail(LogStatus::kUnsupportedVersion);
  }

  if (!has(8 + 8 + 4 + 4)) {
    return fail(LogStatus::kTruncated);
  }
  header_.seed = get_u64(data_ + pos_);
  header_.dt = get_f64(data_ + pos_ + 8);
  header_.agent_count = get_u32(data_ + pos_ + 16);
  const std::uint32_t config_len = get_u32(data_ + pos_ + 20);
  pos_ += 24;

  if (config_len > kMaxLogConfigBytes) {
    return fail(LogStatus::kCorrupt);
  }
  if (!has(config_len)) {
    return fail(LogStatus::kTruncated);
  }
  header_.config.assign(reinterpret_cast<const char*>(data_ + pos_), config_len);
  pos_ += config_len;

  state_ = State::kHeaderRead;
  return LogStatus::kOk;
}

LogStatus LogReader::next(LogRecord& record) {
  switch (state_) {
    case State::kNew:
      return LogStatus::kUsage;
    case State::kDone:
      return LogStatus::kEndOfLog;
    case State::kFailed:
      return error_;
    case State::kHeaderRead:
      break;
  }

  if (!has(1)) {
    return fail(LogStatus::kTruncated);
  }
  const std::uint8_t kind = data_[pos_];
  ++pos_;

  if (kind == kRecordKind) {
    if (!has(8 + 8 + 4)) {
      return fail(LogStatus::kTruncated);
    }
    const std::uint64_t tick = get_u64(data_ + pos_);
    const std::uint64_t hash = get_u64(data_ + pos_ + 8);
    const std::uint32_t blob_len = get_u32(data_ + pos_ + 16);
    pos_ += 20;
    if (blob_len > kMaxLogBlobBytes) {
      return fail(LogStatus::kCorrupt);
    }
    if (!has(blob_len)) {
      return fail(LogStatus::kTruncated);
    }
    record.tick = tick;
    record.hash = hash;
    record.blob.assign(data_ + pos_, data_ + pos_ + blob_len);
    pos_ += blob_len;
    ++records_read_;
    last_hash_ = hash;
    return LogStatus::kOk;
  }

  if (kind == kTrailerKind) {
    if (!has(16)) {
      return fail(LogStatus::kTruncated);
    }
    trailer_.record_count = get_u64(data_ + pos_);
    trailer_.final_hash = get_u64(data_ + pos_ + 8);
    pos_ += 16;
    if (trailer_.record_count != records_read_) {
      return fail(LogStatus::kTrailerCountMismatch);
    }
    if (trailer_.final_hash != last_hash_) {
      return fail(LogStatus::kTrailerHashMismatch);
    }
    if (pos_ != size_) {
      return fail(LogStatus::kCorrupt);
    }
    state_ = State::kDone;
    return LogStatus::kEndOfLog;
  }

  return fail(LogStatus::kCorrupt);
}

}  // namespace swarm
