// Command-line runner for the demo simulation: run, record a log, or verify a log by
// re-execution. The scenario runner replaces the demo agents in a later phase.

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

#include "swarm/run.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitVerifyFailed = 1;
constexpr int kExitUsage = 2;

struct Options {
  swarm::RunConfig run;
  std::size_t threads = 1;
  std::string record_path;
  std::string replay_path;
  bool verify = false;
  bool config_flag_given = false;
  bool help = false;
};

void print_usage() {
  std::cout << "usage:\n"
               "  swarm_sim [config options]                 run and print the final state hash\n"
               "  swarm_sim --record FILE [config options]   run and write a log\n"
               "  swarm_sim --replay FILE --verify           re-execute a log and compare hashes\n"
               "\n"
               "config options (defaults in parentheses):\n"
               "  --seed N (42)  --agents N (8)  --ticks N (1000)  --dt SECONDS (0.01)\n"
               "  --latency TICKS (1)  --jitter TICKS (0)  --rate-limit MSGS (0 = none)\n"
               "  --loss-model none|independent|burst (none)  --loss P (0)  --mean-burst L (1)\n"
               "  --jam cx,cy,cz,radius,mode,start,end,period,duty,offset,loss,affects\n"
               "      add a jammer (repeatable); radius may be inf, mode constant|pulsed,\n"
               "      affects receiver|either; e.g. 40,0,0,25,pulsed,50,400,20,0.5,0,1,receiver\n"
               "\n"
               "other options:\n"
               "  --threads N (1)   thread count; never changes results\n"
               "exit status: 0 ok, 1 verification failed, 2 usage or I/O error\n";
}

template <typename T>
bool parse_number(const char* text, T& out) {
  const char* end = text + std::strlen(text);
  const auto result = std::from_chars(text, end, out);
  return result.ec == std::errc() && result.ptr == end;
}

bool parse_options(int argc, char** argv, Options& options, std::string& error) {
  options.run.sim.seed = 42;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--help" || flag == "-h") {
      options.help = true;
      continue;
    }
    if (flag == "--verify") {
      options.verify = true;
      continue;
    }

    if (i + 1 >= argc) {
      error = "missing value for " + flag;
      return false;
    }
    const char* value = argv[++i];
    bool ok = true;
    swarm::ChannelConfig& channel = options.run.sim.channel;

    if (flag == "--record") {
      options.record_path = value;
    } else if (flag == "--replay") {
      options.replay_path = value;
    } else if (flag == "--threads") {
      ok = parse_number(value, options.threads) && options.threads >= 1;
    } else {
      options.config_flag_given = true;
      if (flag == "--seed") {
        ok = parse_number(value, options.run.sim.seed);
      } else if (flag == "--agents") {
        ok = parse_number(value, options.run.agents);
      } else if (flag == "--ticks") {
        ok = parse_number(value, options.run.ticks);
      } else if (flag == "--dt") {
        ok = parse_number(value, options.run.sim.dt);
      } else if (flag == "--latency") {
        ok = parse_number(value, channel.latency_ticks);
      } else if (flag == "--jitter") {
        ok = parse_number(value, channel.jitter_ticks);
      } else if (flag == "--rate-limit") {
        ok = parse_number(value, channel.max_msgs_per_sender_per_tick);
      } else if (flag == "--loss") {
        ok = parse_number(value, channel.loss);
      } else if (flag == "--mean-burst") {
        ok = parse_number(value, channel.mean_burst);
      } else if (flag == "--jam") {
        swarm::Jammer jammer;
        std::string why;
        if (!swarm::parse_jammer(value, jammer, why)) {
          error = "invalid --jam '" + std::string(value) + "': " + why;
          return false;
        }
        channel.jammers.push_back(jammer);
      } else if (flag == "--loss-model") {
        const std::string name = value;
        if (name == "none") {
          channel.loss_model = swarm::LossModel::kNone;
        } else if (name == "independent") {
          channel.loss_model = swarm::LossModel::kIndependent;
        } else if (name == "burst") {
          channel.loss_model = swarm::LossModel::kBurst;
        } else {
          ok = false;
        }
      } else {
        error = "unknown option " + flag;
        return false;
      }
    }
    if (!ok) {
      error = "invalid value '" + std::string(value) + "' for " + flag;
      return false;
    }
  }

  if (!options.record_path.empty() && !options.replay_path.empty()) {
    error = "--record and --replay cannot be combined";
    return false;
  }
  if (!options.replay_path.empty()) {
    if (!options.verify) {
      error = "--replay currently only supports verification; pass --verify";
      return false;
    }
    if (options.config_flag_given) {
      error = "the run configuration comes from the log; only --threads may be given with --replay";
      return false;
    }
  } else if (options.verify) {
    error = "--verify requires --replay";
    return false;
  }
  return true;
}

bool read_file(const std::string& path, std::vector<std::uint8_t>& out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return true;
}

bool write_file(const std::string& path, const std::vector<std::uint8_t>& bytes) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    return false;
  }
  out.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  return static_cast<bool>(out);
}

std::string hex16(std::uint64_t value) {
  char buffer[17];
  std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(value));
  return buffer;
}

int run_verify(const Options& options) {
  std::vector<std::uint8_t> bytes;
  if (!read_file(options.replay_path, bytes)) {
    std::cerr << "cannot read " << options.replay_path << "\n";
    return kExitUsage;
  }
  const swarm::VerifyResult result = swarm::verify_log(bytes.data(), bytes.size(), options.threads);
  if (result.status == swarm::VerifyStatus::kOk) {
    std::cout << "verified " << options.replay_path << ": all recorded hashes match\n";
    return kExitOk;
  }
  std::cerr << "verification failed (" << swarm::to_string(result.status) << "): " << result.message
            << "\n";
  return kExitVerifyFailed;
}

int run_record(const Options& options) {
  std::vector<std::uint8_t> bytes;
  const swarm::LogStatus status = swarm::record_run(options.run, options.threads, bytes);
  if (status != swarm::LogStatus::kOk) {
    std::cerr << "cannot record: " << swarm::to_string(status) << "\n";
    return kExitUsage;
  }
  if (!write_file(options.record_path, bytes)) {
    std::cerr << "cannot write " << options.record_path << "\n";
    return kExitUsage;
  }
  std::cout << "recorded " << options.run.ticks << " ticks to " << options.record_path << "\n";
  return kExitOk;
}

int run_plain(const Options& options) {
  const std::uint64_t hash = swarm::final_hash(options.run, options.threads);
  std::cout << "swarm_sim " << SWARM_VERSION << " seed=" << options.run.sim.seed
            << " agents=" << options.run.agents << " ticks=" << options.run.ticks
            << " hash=" << hex16(hash) << "\n";
  return kExitOk;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  if (!parse_options(argc, argv, options, error)) {
    std::cerr << "swarm_sim: " << error << "\n";
    print_usage();
    return kExitUsage;
  }
  if (options.help) {
    print_usage();
    return kExitOk;
  }

  try {
    if (!options.replay_path.empty()) {
      return run_verify(options);
    }
    if (!options.record_path.empty()) {
      return run_record(options);
    }
    return run_plain(options);
  } catch (const std::exception& e) {
    std::cerr << "swarm_sim: " << e.what() << "\n";
    return kExitUsage;
  }
}
