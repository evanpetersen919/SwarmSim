// Compares the real socket transport with the simulated channel: whether they deliver the same
// messages, and what the real transport costs. See swarm/net/compare.hpp. Timings depend on the
// machine, so run it in a Release build and report the platform with any numbers taken from it.
//
// Exit status: 0 the deliveries were identical, 1 they were not (or the network lost messages),
// 2 usage or setup error.

#include <charconv>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <system_error>

#include "swarm/net/compare.hpp"
#include "swarm/net/socket.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitDifferent = 1;
constexpr int kExitUsage = 2;

void print_usage() {
  std::cout << "usage: swarm_netcmp [options]\n"
               "  --endpoints N (8)       number of endpoints, at least 2\n"
               "  --ticks N (500)         ticks of traffic compared\n"
               "  --payload BYTES (65)    payload size, at most 1100\n"
               "  --samples N (2000)      messages timed one at a time and in a batch\n"
               "  --loss P (0.1)          independent loss in the model, in [0, 1]\n"
               "  --latency TICKS (2)     the model's latency, at least 1\n"
               "  --jitter TICKS (1)      the model's jitter\n"
               "  --dt SECONDS (0.02)     seconds per tick, for showing the model's latency\n"
               "  --seed N (1)\n"
               "  --no-commands           send no commands (no TCP traffic in the comparison)\n"
               "  --receive-timeout-ms N (1000)\n"
               "  --lose-every N          lose every Nth message in transit, to test the tool\n"
               "exit status: 0 identical, 1 different, 2 usage error\n";
}

template <typename T>
bool parse(const char* text, T& out) {
  const char* end = text + std::strlen(text);
  const auto result = std::from_chars(text, end, out);
  return result.ec == std::errc() && result.ptr == end;
}

}  // namespace

int main(int argc, char** argv) {
  swarm::net::CompareConfig config;
  std::uint64_t lose_every = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--help" || flag == "-h") {
      print_usage();
      return kExitOk;
    }
    if (flag == "--no-commands") {
      config.commands = false;
      continue;
    }
    if (i + 1 >= argc) {
      std::cerr << "swarm_netcmp: missing value for " << flag << "\n";
      return kExitUsage;
    }
    const char* value = argv[++i];
    bool ok = true;
    if (flag == "--endpoints") {
      ok = parse(value, config.endpoints);
    } else if (flag == "--ticks") {
      ok = parse(value, config.ticks);
    } else if (flag == "--payload") {
      ok = parse(value, config.payload_bytes);
    } else if (flag == "--samples") {
      ok = parse(value, config.samples);
    } else if (flag == "--loss") {
      ok = parse(value, config.loss);
    } else if (flag == "--latency") {
      ok = parse(value, config.latency_ticks);
    } else if (flag == "--jitter") {
      ok = parse(value, config.jitter_ticks);
    } else if (flag == "--dt") {
      ok = parse(value, config.dt);
    } else if (flag == "--seed") {
      ok = parse(value, config.seed);
    } else if (flag == "--receive-timeout-ms") {
      ok = parse(value, config.receive_timeout_ms);
    } else if (flag == "--lose-every") {
      ok = parse(value, lose_every) && lose_every >= 1;
    } else {
      std::cerr << "swarm_netcmp: unknown option " << flag << "\n";
      return kExitUsage;
    }
    if (!ok) {
      std::cerr << "swarm_netcmp: invalid value '" << value << "' for " << flag << "\n";
      return kExitUsage;
    }
  }
  if (lose_every > 0) {
    config.lose_in_transit = [lose_every](const swarm::Message& m) {
      return m.seq % lose_every == lose_every - 1;
    };
  }

  try {
    const swarm::net::CompareResult result = swarm::net::run_comparison(config);
    std::cout << swarm::net::format_report(config, result);
    return result.identical && result.dropped_by_network == 0 ? kExitOk : kExitDifferent;
  } catch (const std::invalid_argument& e) {
    std::cerr << "swarm_netcmp: " << e.what() << "\n";
    return kExitUsage;
  } catch (const swarm::net::NetError& e) {
    std::cerr << "swarm_netcmp: " << e.what() << "\n";
    return kExitUsage;
  } catch (const std::exception& e) {
    std::cerr << "swarm_netcmp: " << e.what() << "\n";
    return kExitUsage;
  }
}
