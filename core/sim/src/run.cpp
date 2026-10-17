#include "swarm/run.hpp"

#include <charconv>
#include <exception>
#include <memory>
#include <set>
#include <system_error>
#include <utility>

#include "swarm/demo_agent.hpp"

namespace swarm {

namespace {

std::string format_double(double value) {
  char buffer[64];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  return std::string(buffer, result.ptr);
}

template <typename T>
bool parse_integer(const std::string& text, T& out) {
  const char* begin = text.data();
  const char* end = begin + text.size();
  const auto result = std::from_chars(begin, end, out);
  return result.ec == std::errc() && result.ptr == end;
}

bool parse_double(const std::string& text, double& out) {
  const char* begin = text.data();
  const char* end = begin + text.size();
  const auto result = std::from_chars(begin, end, out);
  return result.ec == std::errc() && result.ptr == end;
}

const char* loss_model_name(LossModel model) {
  switch (model) {
    case LossModel::kNone:
      return "none";
    case LossModel::kIndependent:
      return "independent";
    case LossModel::kBurst:
      return "burst";
  }
  return "none";
}

bool parse_loss_model(const std::string& text, LossModel& out) {
  if (text == "none") {
    out = LossModel::kNone;
  } else if (text == "independent") {
    out = LossModel::kIndependent;
  } else if (text == "burst") {
    out = LossModel::kBurst;
  } else {
    return false;
  }
  return true;
}

std::unique_ptr<Simulation> make_simulation(const RunConfig& config, std::size_t threads) {
  std::vector<std::unique_ptr<IAgent>> agents;
  agents.reserve(config.agents);
  for (std::uint32_t i = 0; i < config.agents; ++i) {
    agents.push_back(std::make_unique<DemoAgent>(i, config.agents));
  }
  return std::make_unique<Simulation>(config.sim, std::move(agents), threads);
}

VerifyResult verify_failure(VerifyStatus status, std::string message) {
  VerifyResult result;
  result.status = status;
  result.message = std::move(message);
  return result;
}

}  // namespace

LogHeader to_log_header(const RunConfig& config) {
  LogHeader header;
  header.seed = config.sim.seed;
  header.dt = config.sim.dt;
  header.agent_count = config.agents;
  const ChannelConfig& c = config.sim.channel;
  header.config += "ticks=" + std::to_string(config.ticks) + "\n";
  header.config += "latency_ticks=" + std::to_string(c.latency_ticks) + "\n";
  header.config += std::string("loss_model=") + loss_model_name(c.loss_model) + "\n";
  header.config += "loss=" + format_double(c.loss) + "\n";
  header.config += "mean_burst=" + format_double(c.mean_burst) + "\n";
  header.config += "jitter_ticks=" + std::to_string(c.jitter_ticks) + "\n";
  header.config +=
      "max_msgs_per_sender_per_tick=" + std::to_string(c.max_msgs_per_sender_per_tick) + "\n";
  return header;
}

bool from_log_header(const LogHeader& header, RunConfig& out, std::string& error) {
  RunConfig config;
  config.sim.seed = header.seed;
  config.sim.dt = header.dt;
  config.agents = header.agent_count;

  const std::set<std::string> required = {"ticks",
                                          "latency_ticks",
                                          "loss_model",
                                          "loss",
                                          "mean_burst",
                                          "jitter_ticks",
                                          "max_msgs_per_sender_per_tick"};
  std::set<std::string> seen;

  std::size_t pos = 0;
  const std::string& text = header.config;
  while (pos < text.size()) {
    std::size_t eol = text.find('\n', pos);
    if (eol == std::string::npos) {
      eol = text.size();
    }
    const std::string line = text.substr(pos, eol - pos);
    pos = eol + 1;
    if (line.empty()) {
      continue;
    }

    const std::size_t eq = line.find('=');
    if (eq == std::string::npos || eq == 0) {
      error = "malformed line '" + line + "'";
      return false;
    }
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    if (required.count(key) == 0) {
      error = "unknown key '" + key + "'";
      return false;
    }
    if (!seen.insert(key).second) {
      error = "duplicate key '" + key + "'";
      return false;
    }

    bool ok = false;
    ChannelConfig& c = config.sim.channel;
    if (key == "ticks") {
      ok = parse_integer(value, config.ticks);
    } else if (key == "latency_ticks") {
      ok = parse_integer(value, c.latency_ticks);
    } else if (key == "loss_model") {
      ok = parse_loss_model(value, c.loss_model);
    } else if (key == "loss") {
      ok = parse_double(value, c.loss);
    } else if (key == "mean_burst") {
      ok = parse_double(value, c.mean_burst);
    } else if (key == "jitter_ticks") {
      ok = parse_integer(value, c.jitter_ticks);
    } else if (key == "max_msgs_per_sender_per_tick") {
      ok = parse_integer(value, c.max_msgs_per_sender_per_tick);
    }
    if (!ok) {
      error = "invalid value for '" + key + "': '" + value + "'";
      return false;
    }
  }

  for (const std::string& key : required) {
    if (seen.count(key) == 0) {
      error = "missing key '" + key + "'";
      return false;
    }
  }

  out = config;
  return true;
}

LogStatus record_run(const RunConfig& config, std::size_t threads,
                     std::vector<std::uint8_t>& log_bytes) {
  auto sim = make_simulation(config, threads);
  LogWriter writer;
  LogStatus status = writer.begin(to_log_header(config));
  if (status != LogStatus::kOk) {
    return status;
  }
  for (std::uint64_t t = 0; t < config.ticks; ++t) {
    sim->step();
    LogRecord record;
    record.tick = t;
    record.hash = sim->state_hash();
    status = writer.append(record);
    if (status != LogStatus::kOk) {
      return status;
    }
  }
  status = writer.finish();
  if (status != LogStatus::kOk) {
    return status;
  }
  log_bytes = writer.bytes();
  return LogStatus::kOk;
}

std::uint64_t final_hash(const RunConfig& config, std::size_t threads) {
  auto sim = make_simulation(config, threads);
  for (std::uint64_t t = 0; t < config.ticks; ++t) {
    sim->step();
  }
  return sim->state_hash();
}

VerifyResult verify_log(const std::uint8_t* data, std::size_t size, std::size_t threads) {
  LogReader reader(data, size);
  LogStatus status = reader.read_header();
  if (status != LogStatus::kOk) {
    VerifyResult result = verify_failure(VerifyStatus::kLogError, to_string(status));
    result.log_status = status;
    return result;
  }

  RunConfig config;
  std::string error;
  if (!from_log_header(reader.header(), config, error)) {
    return verify_failure(VerifyStatus::kBadConfig, error);
  }

  std::unique_ptr<Simulation> sim;
  try {
    sim = make_simulation(config, threads);
  } catch (const std::exception& e) {
    return verify_failure(VerifyStatus::kBadConfig, e.what());
  }

  std::uint64_t expected_tick = 0;
  LogRecord record;
  for (;;) {
    status = reader.next(record);
    if (status == LogStatus::kEndOfLog) {
      break;
    }
    if (status != LogStatus::kOk) {
      VerifyResult result = verify_failure(VerifyStatus::kLogError, to_string(status));
      result.log_status = status;
      return result;
    }

    if (record.tick != expected_tick) {
      VerifyResult result =
          verify_failure(VerifyStatus::kTickMismatch,
                         "record has tick " + std::to_string(record.tick) + " where tick " +
                             std::to_string(expected_tick) + " was expected");
      result.tick = expected_tick;
      return result;
    }
    if (expected_tick >= config.ticks) {
      return verify_failure(VerifyStatus::kLengthMismatch,
                            "log has more records than the " + std::to_string(config.ticks) +
                                " ticks its configuration specifies");
    }

    try {
      sim->step();
    } catch (const std::exception& e) {
      return verify_failure(VerifyStatus::kBadConfig, e.what());
    }
    if (sim->state_hash() != record.hash) {
      VerifyResult result =
          verify_failure(VerifyStatus::kHashMismatch,
                         "state hash differs at tick " + std::to_string(expected_tick));
      result.tick = expected_tick;
      return result;
    }
    ++expected_tick;
  }

  if (expected_tick != config.ticks) {
    return verify_failure(VerifyStatus::kLengthMismatch,
                          "log has " + std::to_string(expected_tick) + " records but its " +
                              "configuration specifies " + std::to_string(config.ticks) + " ticks");
  }
  return VerifyResult{};
}

const char* to_string(VerifyStatus status) {
  switch (status) {
    case VerifyStatus::kOk:
      return "ok";
    case VerifyStatus::kLogError:
      return "log error";
    case VerifyStatus::kBadConfig:
      return "bad configuration";
    case VerifyStatus::kTickMismatch:
      return "tick mismatch";
    case VerifyStatus::kHashMismatch:
      return "hash mismatch";
    case VerifyStatus::kLengthMismatch:
      return "length mismatch";
  }
  return "unknown";
}

}  // namespace swarm
