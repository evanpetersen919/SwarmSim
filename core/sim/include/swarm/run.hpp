#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/simulation.hpp"

namespace swarm {

// Everything needed to reproduce a run except the thread count, which must not matter.
struct RunConfig {
  SimConfig sim;
  std::uint32_t agents = 8;
  std::uint64_t ticks = 1000;
};

// The log header holds seed, dt, and agent count directly, and the remaining fields as
// `key=value` lines in `config`. Every key is required; unknown or repeated keys are errors.
LogHeader to_log_header(const RunConfig& config);
bool from_log_header(const LogHeader& header, RunConfig& out, std::string& error);

// Runs the demo simulation for `config.ticks` ticks and writes one log record per tick.
// Throws std::invalid_argument if the configuration is invalid.
LogStatus record_run(const RunConfig& config, std::size_t threads,
                     std::vector<std::uint8_t>& log_bytes);

// Runs the demo simulation for `config.ticks` ticks and returns the final state hash.
std::uint64_t final_hash(const RunConfig& config, std::size_t threads);

enum class VerifyStatus {
  kOk,
  kLogError,        // the log could not be read; see log_status
  kBadConfig,       // the header configuration is missing, malformed, or invalid
  kTickMismatch,    // a record's tick is not the expected tick
  kHashMismatch,    // re-execution produced a different hash at `tick`
  kLengthMismatch,  // the log has a different number of records than its configuration says
};

struct VerifyResult {
  VerifyStatus status = VerifyStatus::kOk;
  LogStatus log_status = LogStatus::kOk;
  std::uint64_t tick = 0;  // for tick and hash mismatches
  std::string message;
};

// Re-executes the run described by the log header and compares every recorded hash. The thread
// count is free: a log recorded with one thread must verify with any other.
VerifyResult verify_log(const std::uint8_t* data, std::size_t size, std::size_t threads);

const char* to_string(VerifyStatus status);

}  // namespace swarm
